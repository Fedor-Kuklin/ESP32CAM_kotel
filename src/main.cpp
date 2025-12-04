#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <base64.h>

// ---------------------- WiFi ----------------------
const char* WIFI_SSID = "Keenetic-7422";
const char* WIFI_PASS = "uPbkZw3T";

// ---------------------- Оптопары ----------------------
const int gpio_btn_plus  = 12;
const int gpio_btn_minus = 13;
const int gpio_btn_mode  = 14;

// ---------------------- ROI настройки ----------------------
int ROI_X = 10, ROI_Y = 10, ROI_W = 150, ROI_H = 100;

#define FRAME_SIZE FRAMESIZE_QQVGA
#define PIXFORMAT PIXFORMAT_RGB565

// ---------------------- ГЕОМЕТРИЯ СЕГМЕНТОВ ----------------------
struct Rect { 
    int x,y,w,h; 
};

const int DIGITS = 2;
const int SEGMENTS = 7;

Rect segPos[DIGITS][SEGMENTS] = {
  {
    {8,  6, 18, 8}, {26, 6, 8, 18}, {26, 28, 8, 18}, {8, 44, 18, 8},
    {0, 28, 8, 18}, {0, 6, 8, 18}, {8, 26, 18, 8}
  },
  {
    {44, 6, 18, 8}, {62, 6, 8, 18}, {62, 28, 8, 18}, {44, 44, 18, 8},
    {36, 28, 8, 18}, {36, 6, 8, 18}, {44, 26, 18, 8}
  }
};

// ---------------------- Верхние LED ----------------------
Rect topLEDs[] = {
  {2, -6, 8, 8},
  {30, -6, 8, 8},
  {60, -6, 8, 8},
  {88, -6, 8, 8}
};

int threshSegment = 60;
int threshLED = 80;

// Таблица преобразования маски в цифру
int maskToDigit[128];

void initMaskMap() {
  for (int i=0; i<128; i++) maskToDigit[i] = -1;
  maskToDigit[0b0111111] = 0;
  maskToDigit[0b0000110] = 1;
  maskToDigit[0b1011011] = 2;
  maskToDigit[0b1001111] = 3;
  maskToDigit[0b1100110] = 4;
  maskToDigit[0b1101101] = 5;
  maskToDigit[0b1111101] = 6;
  maskToDigit[0b0000111] = 7;
  maskToDigit[0b1111111] = 8;
  maskToDigit[0b1101111] = 9;
}

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
String lastResult = "";
String lastDigits = "--";
String lastLEDs = "0000";

// -----------------------------------------------------------
// Функция считывания сегментов с изображения
// -----------------------------------------------------------
String readDisplay() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return "ERR_NO_FRAME";

  int w = fb->width;
  int h = fb->height;
  String digits = "";
  String leds = "";

  // ---- ЦИФРЫ ----
  for (int d=0; d<DIGITS; d++) {
    int mask = 0;
    for (int s=0; s<SEGMENTS; s++) {
      Rect r = segPos[d][s];
      int ax = ROI_X + r.x;
      int ay = ROI_Y + r.y;
      int aw = r.w, ah = r.h;

      long sumB = 0;
      int cnt = 0;

      for(int yy=ay; yy<ay+ah; yy++){
        for(int xx=ax; xx<ax+aw; xx++){
          if(xx<0||yy<0||xx>=w||yy>=h) continue;

          int idx = (yy * w + xx) * 2;
          uint16_t pix = fb->buf[idx] | (fb->buf[idx+1] << 8);

          int r5 = (pix >> 11) & 0x1F;
          int g6 = (pix >> 5) & 0x3F;
          int b5 = pix & 0x1F;

          int R = (r5 * 255) / 31;
          int G = (g6 * 255) / 63;
          int B = (b5 * 255) / 31;

          int gray = (R*30 + G*59 + B*11) / 100;
          sumB += gray;
          cnt++;
        }
      }

      int avg = (cnt > 0) ? (sumB / cnt) : 0;
      int on = (avg >= threshSegment);
      mask |= (on << s);
    }

    int digit = maskToDigit[mask];
    digits += (digit < 0 ? "?" : String(digit));
  }

  // ---- LED индикаторы ----
  for (auto &led : topLEDs) {
    int ax = ROI_X + led.x;
    int ay = ROI_Y + led.y;

    long sumB = 0;
    int cnt = 0;

    for(int yy=ay; yy<ay+led.h; yy++){
      for(int xx=ax; xx<ax+led.w; xx++){
        if(xx<0||yy<0||xx>=w||yy>=h) continue;

        int idx = (yy*w + xx) * 2;
        uint16_t pix = fb->buf[idx] | (fb->buf[idx+1] << 8);

        int r5 = (pix >> 11) & 0x1F;
        int g6 = (pix >> 5) & 0x3F;
        int b5 = pix & 0x1F;

        int R = (r5 * 255) / 31;
        int G = (g6 * 255) / 63;
        int B = (b5 * 255) / 31;

        int gray = (R*30 + G*59 + B*11) / 100;
        sumB += gray; cnt++;
      }
    }

    int avg = (cnt > 0) ? sumB / cnt : 0;
    leds += (avg >= threshLED ? '1' : '0');
  }

  esp_camera_fb_return(fb);
  
  lastDigits = digits;
  lastLEDs = leds;
  lastResult = digits + "|" + leds;
  
  return lastResult;
}

// -----------------------------------------------------------
// WebSocket обработчик
// -----------------------------------------------------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      Serial.printf("[%u] Connected from %s\n", num, webSocket.remoteIP(num).toString().c_str());
      // Отправляем текущие данные при подключении
      webSocket.sendTXT(num, "DIGITS:" + lastDigits);
      webSocket.sendTXT(num, "LEDS:" + lastLEDs);
      break;
    case WStype_TEXT:
      // Обработка команд
      if (strncmp((char*)payload, "BUTTON:", 7) == 0) {
        String btn = String((char*)payload + 7);
        int pin = -1;
        
        if (btn == "plus")  pin = gpio_btn_plus;
        if (btn == "minus") pin = gpio_btn_minus;
        if (btn == "mode")  pin = gpio_btn_mode;
        
        if (pin >= 0) {
          digitalWrite(pin, HIGH);
          delay(200);
          digitalWrite(pin, LOW);
          webSocket.sendTXT(num, "BUTTON_PRESSED:" + btn);
        }
      }
      break;
  }
}

// -----------------------------------------------------------
// HTML страница с визуализацией
// -----------------------------------------------------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>ESP32-CAM 7-Segment Reader</title>
    <style>
        body { 
            font-family: Arial, sans-serif; 
            margin: 20px; 
            background: #f0f0f0;
        }
        .container {
            display: flex;
            flex-wrap: wrap;
            gap: 20px;
        }
        .panel {
            background: white;
            padding: 15px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        .display-panel {
            flex: 1;
            min-width: 300px;
        }
        .video-panel {
            flex: 2;
            min-width: 400px;
        }
        .controls-panel {
            flex: 1;
            min-width: 200px;
        }
        .display {
            font-family: 'Courier New', monospace;
            font-size: 72px;
            text-align: center;
            background: #000;
            color: #0f0;
            padding: 20px;
            border-radius: 5px;
            margin: 10px 0;
        }
        .led-indicator {
            display: inline-block;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            margin: 0 5px;
            border: 2px solid #333;
        }
        .led-on {
            background: #f00;
            box-shadow: 0 0 10px #f00;
        }
        .led-off {
            background: #444;
        }
        .button {
            padding: 10px 20px;
            margin: 5px;
            border: none;
            border-radius: 5px;
            background: #4CAF50;
            color: white;
            font-size: 16px;
            cursor: pointer;
        }
        .button:hover {
            background: #45a049;
        }
        .button:active {
            background: #3d8b40;
        }
        #videoCanvas {
            border: 2px solid #333;
            background: #000;
            display: block;
            max-width: 100%;
        }
        .roi-box {
            position: absolute;
            border: 2px solid #0f0;
            pointer-events: none;
        }
        .segment-box {
            position: absolute;
            border: 1px solid rgba(255,255,0,0.3);
            pointer-events: none;
        }
        .led-box {
            position: absolute;
            border: 1px solid rgba(255,0,0,0.5);
            pointer-events: none;
        }
        .stats {
            margin-top: 10px;
            font-size: 14px;
            color: #666;
        }
    </style>
</head>
<body>
    <h1>ESP32-CAM 7-Segment Display Reader</h1>
    
    <div class="container">
        <div class="panel display-panel">
            <h2>Display Reading</h2>
            <div class="display" id="digitsDisplay">--</div>
            
            <h3>LED Indicators</h3>
            <div id="ledDisplay">
                <span class="led-indicator led-off"></span>
                <span class="led-indicator led-off"></span>
                <span class="led-indicator led-off"></span>
                <span class="led-indicator led-off"></span>
            </div>
            
            <div class="stats">
                <div>Last Update: <span id="lastUpdate">Never</span></div>
                <div>Frame Rate: <span id="fps">0</span> FPS</div>
                <div>Raw Data: <span id="rawData">--</span></div>
            </div>
        </div>
        
        <div class="panel video-panel">
            <h2>Live Camera View</h2>
            <div style="position: relative; display: inline-block;">
                <canvas id="videoCanvas" width="160" height="120"></canvas>
                <div id="roiOverlay" style="position: absolute; top: 0; left: 0;"></div>
            </div>
            <div class="controls">
                <button class="button" onclick="toggleVideo()" id="videoBtn">Start Video</button>
                <button class="button" onclick="toggleOverlay()" id="overlayBtn">Hide Overlay</button>
                <button class="button" onclick="takeSnapshot()">Snapshot</button>
            </div>
        </div>
        
        <div class="panel controls-panel">
            <h2>Controls</h2>
            <div>
                <button class="button" onclick="pressButton('plus')">+ Button</button>
                <button class="button" onclick="pressButton('minus')">- Button</button>
                <button class="button" onclick="pressButton('mode')">Mode Button</button>
            </div>
            
            <h3>ROI Settings</h3>
            <div>
                <label>X: <input type="number" id="roiX" value="20" style="width: 60px;"></label>
                <label>Y: <input type="number" id="roiY" value="10" style="width: 60px;"></label><br>
                <button class="button" onclick="updateROI()">Update ROI</button>
                <button class="button" onclick="autoDetect()">Auto Detect</button>
            </div>
            
            <h3>Thresholds</h3>
            <div>
                <label>Segment: <input type="range" id="threshSegment" min="0" max="255" value="60"></label>
                <span id="threshSegmentVal">60</span><br>
                <label>LED: <input type="range" id="threshLED" min="0" max="255" value="80"></label>
                <span id="threshLEDVal">80</span>
                <button class="button" onclick="updateThresholds()">Apply</button>
            </div>
        </div>
    </div>

    <script>
        const canvas = document.getElementById('videoCanvas');
        const ctx = canvas.getContext('2d');
        const roiOverlay = document.getElementById('roiOverlay');
        const digitsDisplay = document.getElementById('digitsDisplay');
        const ledDisplay = document.getElementById('ledDisplay');
        
        let ws;
        let streaming = false;
        let showOverlay = true;
        let frameCount = 0;
        let lastTime = Date.now();
        
        // Функция конвертации RGB565 в RGB888
        function rgb565ToRgb888(rgb565Array, width, height) {
            const imgData = ctx.createImageData(width, height);
            const data = imgData.data;
            
            for (let i = 0; i < rgb565Array.length; i++) {
                const pixel = rgb565Array[i];
                const r5 = (pixel >> 11) & 0x1F;
                const g6 = (pixel >> 5) & 0x3F;
                const b5 = pixel & 0x1F;
                
                data[i * 4] = (r5 << 3) | (r5 >> 2);
                data[i * 4 + 1] = (g6 << 2) | (g6 >> 4);
                data[i * 4 + 2] = (b5 << 3) | (b5 >> 2);
                data[i * 4 + 3] = 255;
            }
            
            return imgData;
        }
        
        // Инициализация WebSocket
        function initWebSocket() {
            ws = new WebSocket('ws://' + window.location.hostname + ':81/');
            
            ws.onopen = function() {
                console.log('WebSocket connected');
                document.getElementById('videoBtn').textContent = 'Stop Video';
                streaming = true;
                lastTime = Date.now();
                frameCount = 0;
            };
            
            ws.onmessage = function(event) {
                if (event.data instanceof Blob) {
                    // Бинарные данные (видео)
                    const reader = new FileReader();
                    reader.onload = function() {
                        const arrayBuffer = reader.result;
                        const rgb565Array = new Uint16Array(arrayBuffer);
                        
                        // Обновляем видео
                        const imgData = rgb565ToRgb888(rgb565Array, canvas.width, canvas.height);
                        ctx.putImageData(imgData, 0, 0);
                        
                        // Обновляем статистику
                        frameCount++;
                        const now = Date.now();
                        if (now - lastTime >= 1000) {
                            const fps = Math.round((frameCount * 1000) / (now - lastTime));
                            document.getElementById('fps').textContent = fps;
                            frameCount = 0;
                            lastTime = now;
                        }
                        
                        // Периодически запрашиваем данные с дисплея
                        if (frameCount % 5 === 0) {
                            ws.send('GET_DISPLAY');
                        }
                    };
                    reader.readAsArrayBuffer(event.data);
                } else if (typeof event.data === 'string') {
                    // Текстовые сообщения
                    const msg = event.data;
                    console.log('Message:', msg);
                    
                    if (msg.startsWith('DIGITS:')) {
                        const digits = msg.substring(7);
                        digitsDisplay.textContent = digits;
                        document.getElementById('lastUpdate').textContent = new Date().toLocaleTimeString();
                    } else if (msg.startsWith('LEDS:')) {
                        const leds = msg.substring(5);
                        const ledElements = ledDisplay.querySelectorAll('.led-indicator');
                        for (let i = 0; i < ledElements.length; i++) {
                            if (i < leds.length) {
                                ledElements[i].className = 'led-indicator ' + 
                                    (leds[i] === '1' ? 'led-on' : 'led-off');
                            }
                        }
                    } else if (msg.startsWith('RAW:')) {
                        document.getElementById('rawData').textContent = msg.substring(4);
                    }
                }
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
            };
            
            ws.onclose = function() {
                console.log('WebSocket disconnected');
                if (streaming) {
                    document.getElementById('videoBtn').textContent = 'Start Video';
                    streaming = false;
                    setTimeout(initWebSocket, 2000);
                }
            };
        }
        
        function toggleVideo() {
            if (streaming) {
                ws.close();
                streaming = false;
                document.getElementById('videoBtn').textContent = 'Start Video';
            } else {
                initWebSocket();
            }
        }
        
        function toggleOverlay() {
            showOverlay = !showOverlay;
            roiOverlay.style.display = showOverlay ? 'block' : 'none';
            document.getElementById('overlayBtn').textContent = 
                showOverlay ? 'Hide Overlay' : 'Show Overlay';
        }
        
        function pressButton(btn) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send('BUTTON:' + btn);
            }
        }
        
        function updateROI() {
            const x = document.getElementById('roiX').value;
            const y = document.getElementById('roiY').value;
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send('SET_ROI:' + x + ',' + y);
            }
        }
        
        function updateThresholds() {
            const seg = document.getElementById('threshSegment').value;
            const led = document.getElementById('threshLED').value;
            document.getElementById('threshSegmentVal').textContent = seg;
            document.getElementById('threshLEDVal').textContent = led;
            
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send('SET_THRESH:' + seg + ',' + led);
            }
        }
        
        function autoDetect() {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send('AUTO_DETECT');
            }
        }
        
        function takeSnapshot() {
            const link = document.createElement('a');
            link.download = 'snapshot_' + new Date().getTime() + '.png';
            link.href = canvas.toDataURL('image/png');
            link.click();
        }
        
        // Обновляем значения ползунков
        document.getElementById('threshSegment').oninput = function() {
            document.getElementById('threshSegmentVal').textContent = this.value;
        };
        
        document.getElementById('threshLED').oninput = function() {
            document.getElementById('threshLEDVal').textContent = this.value;
        };
        
        // Инициализация
        window.onload = function() {
            console.log('7-Segment Reader loaded');
        };
    </script>
</body>
</html>
)rawliteral";

// -----------------------------------------------------------
// Web API обработчики
// -----------------------------------------------------------
void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void setupServer() {
  server.on("/", handleRoot);

  server.on("/read", [](){
    server.send(200, "text/plain", readDisplay());
  });

  server.on("/read_json", [](){
    String json = "{";
    json += "\"digits\":\"" + lastDigits + "\",";
    json += "\"leds\":\"" + lastLEDs + "\",";
    json += "\"roi_x\":" + String(ROI_X) + ",";
    json += "\"roi_y\":" + String(ROI_Y) + ",";
    json += "\"thresh_segment\":" + String(threshSegment) + ",";
    json += "\"thresh_led\":" + String(threshLED);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/press", [](){
    if (!server.hasArg("key")) {
      server.send(400, "text/plain", "no key");
      return;
    }

    String key = server.arg("key");
    int pin = -1;

    if (key == "plus")  pin = gpio_btn_plus;
    if (key == "minus") pin = gpio_btn_minus;
    if (key == "mode")  pin = gpio_btn_mode;

    if (pin < 0) {
      server.send(400, "text/plain", "bad key");
      return;
    }

    digitalWrite(pin, HIGH);
    delay(200);
    digitalWrite(pin, LOW);

    server.send(200, "text/plain", "OK");
  });

  server.on("/set_roi", [](){
    if (server.hasArg("x") && server.hasArg("y")) {
      ROI_X = server.arg("x").toInt();
      ROI_Y = server.arg("y").toInt();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Missing x or y");
    }
  });

  server.on("/set_thresh", [](){
    if (server.hasArg("segment") && server.hasArg("led")) {
      threshSegment = server.arg("segment").toInt();
      threshLED = server.arg("led").toInt();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(400, "text/plain", "Missing parameters");
    }
  });

  server.on("/snapshot", [](){
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      server.send(500, "text/plain", "Failed to capture");
      return;
    }

    String encoded = base64::encode(fb->buf, fb->len);
    server.send(200, "text/plain", encoded);
    esp_camera_fb_return(fb);
  });

  server.begin();
}

// -----------------------------------------------------------
// SETUP
// -----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  initMaskMap();

  // Оптопары
  pinMode(gpio_btn_plus, OUTPUT);
  pinMode(gpio_btn_minus, OUTPUT);
  pinMode(gpio_btn_mode, OUTPUT);
  digitalWrite(gpio_btn_plus, LOW);
  digitalWrite(gpio_btn_minus, LOW);
  digitalWrite(gpio_btn_mode, LOW);

  // ---------------- Камера ESP32-CAM ----------------
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = 5;   // Y2_GPIO_NUM
  config.pin_d1       = 18;  // Y3_GPIO_NUM
  config.pin_d2       = 19;  // Y4_GPIO_NUM
  config.pin_d3       = 21;  // Y5_GPIO_NUM
  config.pin_d4       = 36;  // Y6_GPIO_NUM
  config.pin_d5       = 39;  // Y7_GPIO_NUM
  config.pin_d6       = 34;  // Y8_GPIO_NUM
  config.pin_d7       = 35;  // Y9_GPIO_NUM
  config.pin_xclk     = 0;
  config.pin_pclk     = 22;
  config.pin_vsync    = 25;
  config.pin_href     = 23;
  config.pin_sccb_sda = 26;
  config.pin_sccb_scl = 27;
  config.pin_pwdn     = 32;
  config.pin_reset    = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QQVGA;
  config.jpeg_quality = 0;
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return;
  }

  // ---------------- WiFi ----------------
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  setupServer();
  
  // Инициализация WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

// -----------------------------------------------------------
// LOOP
// -----------------------------------------------------------
void loop() {
  server.handleClient();
  webSocket.loop();

  // Отправка видеопотока через WebSocket
  static uint32_t lastFrameTime = 0;
  static uint32_t lastReadTime = 0;
  
  if (webSocket.connectedClients() > 0) {
    // Отправка видео (5 FPS)
    if (millis() - lastFrameTime > 200) {
      camera_fb_t *fb = esp_camera_fb_get();
      if (fb && fb->format == PIXFORMAT_RGB565) {
        webSocket.broadcastBIN(fb->buf, fb->len);
        esp_camera_fb_return(fb);
        lastFrameTime = millis();
      }
    }
    
    // Чтение дисплея (1 раз в секунду)
    if (millis() - lastReadTime > 1000) {
      String result = readDisplay();
      webSocket.broadcastTXT("DIGITS:" + lastDigits);
      webSocket.broadcastTXT("LEDS:" + lastLEDs);
      webSocket.broadcastTXT("RAW:" + result);
      lastReadTime = millis();
    }
  }

  // Вывод в Serial для отладки
  static unsigned long lastSerialPrint = 0;
  if (millis() - lastSerialPrint > 2000) {
    Serial.println("Display: " + lastResult);
    lastSerialPrint = millis();
  }
}