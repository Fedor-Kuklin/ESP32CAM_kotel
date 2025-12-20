// Async OTA updater implementation
#include "OTAUpdater.h"
#include "../../src/secret.h"
#include <Update.h>
#include <Arduino.h>
#include <LittleFS.h>
#include <FS.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "DebugLogger.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool _updateError = false;
static String _updateErrorMsg = "";
static bool _updateStarted = false;
static bool _savingToFS = false;
static bool _savingToPSRAM = false;
static File _tmpFile;
static uint8_t* _psramBuf = nullptr;
static size_t _psramPos = 0;
static const size_t PSRAM_MAX_BUFFER = 2 * 1024 * 1024; // 2MB

// OTA status tracking for UI polling
enum OTAState { OTA_IDLE = 0, OTA_UPLOADING, OTA_WRITING, OTA_SUCCESS, OTA_FAILED };
static volatile OTAState ota_state = OTA_IDLE;
static volatile size_t ota_total = 0;
static volatile size_t ota_received = 0;
static String ota_status_msg = "";

static void sendPlain(AsyncWebServerRequest *request, int code, const String &msg) {
  if (!request) return;
  AsyncWebServerResponse *resp = request->beginResponse(code, "text/plain", msg);
  resp->addHeader("Connection", "close");
  request->send(resp);
}

static void sendHtmlMessage(AsyncWebServerRequest *request, int code, const String &title, const String &msg, bool rebootSoon) {
  if (!request) return;
  String page = "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>" + title + "</title></head><body style='font-family:Arial,sans-serif;padding:20px;'>";
  page += "<h2>" + title + "</h2>";
  page += "<p>" + msg + "</p>";
  page += "<p><a href=\"http://"+ String("10.9.8.113:80") + "\"\">Back to Main</a> &nbsp; ";
  page += "<a href=\"http://" + String(""/*placeholder*/ ) + "\">Home</a></p>";
  if (rebootSoon) page += "<p>Device will reboot shortly...</p>";
  page += "</body></html>";
  AsyncWebServerResponse *resp = request->beginResponse(code, "text/html", page);
  resp->addHeader("Connection", "close");
  request->send(resp);
}

static void rebootTask(void *pvParameters) {
  uint32_t ms = (uint32_t)(uintptr_t)pvParameters;
  vTaskDelay(pdMS_TO_TICKS(ms));
  Serial.println("OTA: rebooting now");
  esp_restart();
}

static void scheduleRestart(uint32_t ms) {
  xTaskCreate(rebootTask, "ota_reboot", 4096, (void*)(uintptr_t)ms, 1, NULL);
}

static String buildForm(const char *title, const char *action, bool warn) {
  String page = "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>";
  page += title;
  page += "</title></head><body>";
  page += "<h3>Upload firmware</h3>";
  if (warn) page += "<p style='color:red;'>Use only for testing. This allows uploads without Content-Length.</p>";
  page += "<form method='POST' action='";
  page += action;
  page += "' enctype='multipart/form-data'>";
  page += "<input type='file' name='update'>";
  page += "<input type='submit' value='Upload'>";
  page += "</form>";
  // status area + polling script
  page += "<div id='otaStatus' style='margin-top:12px;font-family:monospace;'></div>";
  page += "<script>function showStatus(j){var s=document.getElementById('otaStatus'); if(!j){s.textContent='No status';return;} s.textContent='State:'+j.state+' | '+j.received+'/'+(j.total?j.total:'?')+' | msg:'+j.msg;} function poll(){fetch('/update_status').then(r=>{ if(r.status==401){return; } return r.json();}).then(j=>{ if(j) showStatus(j); if(j && (j.state=='SUCCESS' || j.state=='FAILED')) clearInterval(iv); }).catch(e=>{});} var iv=setInterval(poll,1000); poll();</script>";
  page += "</body></html>";
  return page;
}

static void onUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (index == 0) {
    _updateError = false; _updateErrorMsg = ""; _updateStarted = false;
    _savingToFS = false; _savingToPSRAM = false; _psramPos = 0;
    if (_psramBuf) { heap_caps_free(_psramBuf); _psramBuf = nullptr; }
    size_t contentLength = request->contentLength();
    ota_total = contentLength;
    ota_received = 0;
    ota_status_msg = "";
    ota_state = OTA_UPLOADING;
    Serial.printf("OTA: Start update: %s, contentLength=%u\n", filename.c_str(), (unsigned)contentLength);
    size_t freeSpace = ESP.getFreeSketchSpace();
    Serial.printf("OTA: free sketch space=%u\n", freeSpace);
    if (contentLength > 0) {
      if (contentLength > freeSpace) { _updateError = true; _updateErrorMsg = "not enough free space"; return; }
      if (!Update.begin((uint32_t)contentLength)) { Update.printError(Serial); _updateError = true; _updateErrorMsg = "Update.begin failed"; return; }
      _updateStarted = true;
    } else {
      String url = request->url();
      if (url == "/update_allow") {
        if (!LittleFS.begin()) {
          if (!LittleFS.format()) {
            _psramBuf = (uint8_t*)heap_caps_malloc(PSRAM_MAX_BUFFER, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (!_psramBuf) { _updateError = true; _updateErrorMsg = "LittleFS and PSRAM unavailable"; return; }
            _psramPos = 0; _savingToPSRAM = true; Serial.println("OTA: PSRAM buffer allocated for upload");
          } else {
            delay(100);
            if (!LittleFS.begin()) {
              _psramBuf = (uint8_t*)heap_caps_malloc(PSRAM_MAX_BUFFER, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
              if (!_psramBuf) { _updateError = true; _updateErrorMsg = "LittleFS and PSRAM unavailable"; return; }
              _psramPos = 0; _savingToPSRAM = true; Serial.println("OTA: PSRAM buffer allocated for upload");
            } else {
              if (LittleFS.exists("/update.bin")) LittleFS.remove("/update.bin");
              _tmpFile = LittleFS.open("/update.bin", FILE_WRITE);
              if (!_tmpFile) { _updateError = true; _updateErrorMsg = "cannot open temp file"; return; }
              _savingToFS = true;
            }
          }
        } else {
          if (LittleFS.exists("/update.bin")) LittleFS.remove("/update.bin");
          _tmpFile = LittleFS.open("/update.bin", FILE_WRITE);
          if (!_tmpFile) { _updateError = true; _updateErrorMsg = "cannot open temp file"; return; }
          _savingToFS = true;
        }
      } else {
        _updateError = true; _updateErrorMsg = "missing Content-Length: upload rejected"; return;
      }
    }
  }

  if (_updateError) return;

  if (_savingToFS) {
    size_t w = _tmpFile.write(data, len);
      ota_received += w;
      if (w != len) { _updateError = true; _updateErrorMsg = "LittleFS write failed"; _tmpFile.close(); return; }
  } else if (_savingToPSRAM) {
    if ((_psramPos + len) > PSRAM_MAX_BUFFER) { _updateError = true; _updateErrorMsg = "PSRAM buffer overflow"; heap_caps_free(_psramBuf); _psramBuf = nullptr; _savingToPSRAM = false; return; }
    memcpy(_psramBuf + _psramPos, data, len); _psramPos += len;
  } else {
    if (!_updateStarted) { _updateError = true; _updateErrorMsg = "update not started"; return; }
    size_t written = Update.write(data, len);
    if (written != len) { Update.printError(Serial); _updateError = true; _updateErrorMsg = "write failed"; Update.end(); return; }
  }

  if (final) {
    if (_updateError) { if (_savingToFS) { if (_tmpFile) _tmpFile.close(); } else if (_updateStarted) Update.end(); return; }
    if (_savingToFS) {
      size_t fsize = 0; if (_tmpFile) { fsize = _tmpFile.size(); _tmpFile.close(); }
      size_t freeSpace2 = ESP.getFreeSketchSpace(); if (fsize > freeSpace2) { _updateError = true; _updateErrorMsg = "not enough free space"; LittleFS.remove("/update.bin"); _savingToFS = false; return; }
      File f = LittleFS.open("/update.bin", FILE_READ); if (!f) { _updateError = true; _updateErrorMsg = "cannot open temp file read"; _savingToFS = false; return; }
      if (!Update.begin((uint32_t)fsize)) { Update.printError(Serial); _updateError = true; _updateErrorMsg = "Update.begin failed (from LittleFS)"; f.close(); LittleFS.remove("/update.bin"); _savingToFS = false; return; }
      const size_t bufSize = 1024; uint8_t buf[bufSize]; while (f.available()) { size_t r = f.read(buf, bufSize); if (r == 0) break; size_t w = Update.write(buf, r); ota_received += w; if (w != r) { Update.printError(Serial); _updateError = true; _updateErrorMsg = "Update write failed (from LittleFS)"; f.close(); Update.end(); LittleFS.remove("/update.bin"); _savingToFS = false; return; } }
      f.close(); if (Update.end(true)) { Serial.printf("OTA: Update OK from LittleFS, %u bytes\n", (unsigned)fsize); } else { Update.printError(Serial); _updateError = true; _updateErrorMsg = "Update.end failed (from LittleFS)"; LittleFS.remove("/update.bin"); _savingToFS = false; return; } LittleFS.remove("/update.bin"); _savingToFS = false;
      ota_state = OTA_SUCCESS; ota_status_msg = "Update OK from LittleFS";
    } else if (_savingToPSRAM) {
      size_t fsize = _psramPos; if (!Update.begin((uint32_t)fsize)) { Update.printError(Serial); _updateError = true; _updateErrorMsg = "Update.begin failed (PSRAM)"; heap_caps_free(_psramBuf); _psramBuf = nullptr; _savingToPSRAM = false; return; }
      size_t written = Update.write(_psramBuf, fsize); if (written != fsize) { Update.printError(Serial); _updateError = true; _updateErrorMsg = "Update write failed (PSRAM)"; Update.end(); heap_caps_free(_psramBuf); _psramBuf = nullptr; _savingToPSRAM = false; return; }
      ota_received = fsize;
      ota_total = fsize;
      if (Update.end(true)) { Serial.printf("OTA: Update OK from PSRAM, %u bytes\n", (unsigned)fsize); } else { Update.printError(Serial); _updateError = true; _updateErrorMsg = "Update.end failed (PSRAM)"; heap_caps_free(_psramBuf); _psramBuf = nullptr; _savingToPSRAM = false; return; }
      ota_state = OTA_SUCCESS; ota_status_msg = "Update OK from PSRAM";
      heap_caps_free(_psramBuf); _psramBuf = nullptr; _savingToPSRAM = false;
    } else {
      if (_updateStarted && Update.end(true)) { Serial.printf("OTA: Update OK, finished\n"); ota_state = OTA_SUCCESS; ota_status_msg = "Update OK"; } else { Update.printError(Serial); _updateError = true; _updateErrorMsg = "Update.end failed"; ota_state = OTA_FAILED; ota_status_msg = _updateErrorMsg; }
    }
  }
}

void OTAUpdater_begin(AsyncWebServer &server) {
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->authenticate(OTA_USER, OTA_PASS)) { request->requestAuthentication(); return; }
    request->send(200, "text/html", buildForm("OTA Update","/update", false));
  });
  server.on("/update_allow", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->authenticate(OTA_USER, OTA_PASS)) { request->requestAuthentication(); return; }
    request->send(200, "text/html", buildForm("OTA Update (allow unknown size)","/update_allow", true));
  });

  server.on("/update", HTTP_POST,
    [](AsyncWebServerRequest *request){
      if (!request->authenticate(OTA_USER, OTA_PASS)) { request->requestAuthentication(); return; }
      if (_updateError) {
        sendHtmlMessage(request, 500, "Обновление не удалось", String("Обновление не удалось: ") + _updateErrorMsg, false);
      } else {
        sendHtmlMessage(request, 200, "Обновление успешно", "Прошивка загружена успешно. Перезагрузка...", true);
        scheduleRestart(1500);
      }
      _updateError = false; _updateErrorMsg = "";
    }, onUpload);

  server.on("/update_allow", HTTP_POST,
    [](AsyncWebServerRequest *request){
      if (!request->authenticate(OTA_USER, OTA_PASS)) { request->requestAuthentication(); return; }
      if (_updateError) {
        sendHtmlMessage(request, 500, "Update Failed", String("Update failed: ") + _updateErrorMsg, false);
      } else {
        sendHtmlMessage(request, 200, "Update Successful", "Firmware uploaded successfully. Rebooting now...", true);
        scheduleRestart(1500);
      }
      _updateError = false; _updateErrorMsg = "";
    }, onUpload);

  // status endpoint for AJAX polling
  server.on("/update_status", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->authenticate(OTA_USER, OTA_PASS)) { request->requestAuthentication(); return; }
    String stateStr = "IDLE";
    switch (ota_state) {
      case OTA_UPLOADING: stateStr = "UPLOADING"; break;
      case OTA_WRITING: stateStr = "WRITING"; break;
      case OTA_SUCCESS: stateStr = "SUCCESS"; break;
      case OTA_FAILED: stateStr = "FAILED"; break;
      default: stateStr = "IDLE"; break;
    }
    String json = "{";
    json += "\"state\":\"" + stateStr + "\",";
    json += "\"received\":" + String(ota_received) + ",";
    json += "\"total\":" + String(ota_total) + ",";
    json += "\"msg\":\"" + ota_status_msg + "\"";
    json += "}";
    AsyncWebServerResponse *resp = request->beginResponse(200, "application/json", json);
    resp->addHeader("Connection", "close");
    request->send(resp);
  });

  // Simple log viewer page (WebSocket client connects to /ws)
  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->authenticate(OTA_USER, OTA_PASS)) { request->requestAuthentication(); return; }
    String html = R"rawliteral(
    <!doctype html>
    <html><head><meta charset="utf-8"><title>Logs</title></head>
    <body style="font-family:Arial,sans-serif;">
      <h3>Device logs (WebSocket)</h3>
      <pre id="out" style="height:60vh; overflow:auto; border:1px solid #ccc; padding:8px; background:#111; color:#0f0;"></pre>
      <script>
        var out = document.getElementById('out');
        var proto = (location.protocol === 'https:') ? 'wss' : 'ws';
        var ws = new WebSocket(proto + '://' + location.host + '/ws');
        ws.onmessage = function(evt){ out.textContent += evt.data + '\n'; out.scrollTop = out.scrollHeight; };
        ws.onopen = function(){ out.textContent += 'WebSocket connected\n'; };
        ws.onclose = function(){ out.textContent += 'WebSocket closed\n'; };
      </script>
    </body></html>
    )rawliteral";
    request->send(200, "text/html", html);
  });

  // Pause / Resume websocket logs
  server.on("/logs/pause", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->authenticate(OTA_USER, OTA_PASS)) { request->requestAuthentication(); return; }
    DebugLogger::pauseWeb();
    AsyncWebServerResponse *r = request->beginResponse(200, "application/json", "{\"paused\":true}");
    request->send(r);
  });
  server.on("/logs/resume", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->authenticate(OTA_USER, OTA_PASS)) { request->requestAuthentication(); return; }
    DebugLogger::resumeWeb();
    AsyncWebServerResponse *r = request->beginResponse(200, "application/json", "{\"paused\":false}");
    request->send(r);
  });
}
