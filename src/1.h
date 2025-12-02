#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "Global_const.h"
#include <TJpg_Decoder.h>

// ------------------------------------------
// CONFIG
// ------------------------------------------
#define CAM_FRAMESIZE FRAMESIZE_SVGA       // 800×600
#define CAM_PIXFORMAT PIXFORMAT_JPEG       // JPEG всегда!
#define CAM_QUALITY 12

int threshSegment = 60;
int threshLED = 80;

int maskToDigit[128];
AsyncWebServer server(80);
String lastResult = "";

// JPEG decode buffer
static uint16_t *rgb565 = nullptr;
static int dec_w = 0, dec_h = 0;

// ----------------------------------------------------------
//  JPEG decoder callback → writes decoded lines to rgb565[]
// ----------------------------------------------------------
bool tjpgRenderCallback(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap) {
    if (!rgb565) return false;

    for (int yy = 0; yy < h; yy++) {
        memcpy(&rgb565[(y + yy) * dec_w + x], &bitmap[yy * w], w * 2);
    }

    return true;
}

// ----------------------------------------------------------
// Segment mask init
// ----------------------------------------------------------
void initMaskMap() {
    for (int i = 0; i < 128; i++) maskToDigit[i] = -1;
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

// ----------------------------------------------------------
// Analyze decoded RGB565 image
// ----------------------------------------------------------
String analyze_from_buffer() {
    if (!rgb565) return "ERR_BUF";

    String out = "";

    // ---- digits ----
    for (int d = 0; d < DIGITS; d++) {
        int mask = 0;

        for (int s = 0; s < SEGMENTS; s++) {
            Rect r = segPos[d][s];
            long sum = 0;
            int cnt = 0;

            for (int yy = ROI_Y + r.y; yy < ROI_Y + r.y + r.h; yy++) {
                if (yy < 0 || yy >= dec_h) continue;

                for (int xx = ROI_X + r.x; xx < ROI_X + r.x + r.w; xx++) {
                    if (xx < 0 || xx >= dec_w) continue;

                    uint16_t pix = rgb565[yy * dec_w + xx];
                    int r5 = (pix >> 11) & 0x1F;
                    int g6 = (pix >> 5) & 0x3F;
                    int b5 = pix & 0x1F;

                    int R = (r5 * 255) / 31;
                    int G = (g6 * 255) / 63;
                    int B = (b5 * 255) / 31;

                    int gray = (R * 30 + G * 59 + B * 11) / 100;
                    sum += gray;
                    cnt++;
                }
            }

            int avg = cnt ? sum / cnt : 0;
            int on = avg >= threshSegment;
            mask |= (on << s);
        }

        int digit = maskToDigit[mask];
        out += (digit >= 0 ? String(digit) : "?");
    }

    // ---- LEDs ----
    out += " | LEDs:";
    for (auto &led : topLEDs) {
        long sum = 0;
        int cnt = 0;

        for (int yy = ROI_Y + led.y; yy < ROI_Y + led.y + led.h; yy++) {
            if (yy < 0 || yy >= dec_h) continue;
            for (int xx = ROI_X + led.x; xx < ROI_X + led.x + led.w; xx++) {
                if (xx < 0 || xx >= dec_w) continue;

                uint16_t pix = rgb565[yy * dec_w + xx];

                int r5 = (pix >> 11) & 0x1F;
                int g6 = (pix >> 5) & 0x3F;
                int b5 = pix & 0x1F;

                int R = (r5 * 255) / 31;
                int G = (g6 * 255) / 63;
                int B = (b5 * 255) / 31;

                int gray = (R * 30 + G * 59 + B * 11) / 100;
                sum += gray;
                cnt++;
            }
        }

        int avg = cnt ? sum / cnt : 0;
        out += (avg >= threshLED ? '1' : '0');
    }

    return out;
}

// ----------------------------------------------------------
// Capture JPEG and decode into rgb565[]
// ----------------------------------------------------------
String captureForAnalysis() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return "ERR_NO_FRAME";

    uint16_t w = 0, h = 0;
    TJpgDec.getJpgSize(&w, &h, fb->buf, fb->len);

    dec_w = w;
    dec_h = h;

    if (!rgb565) {
        rgb565 = (uint16_t*) ps_malloc(dec_w * dec_h * 2);
        if (!rgb565) {
            esp_camera_fb_return(fb);
            return "ERR_NOMEM";
        }
    }

    // декодируем JPEG → RGB565 через callback
    TJpgDec.drawJpg(0, 0, fb->buf, fb->len);

    esp_camera_fb_return(fb);

    String res = analyze_from_buffer();
    lastResult = res;
    return res;
}


// ----------------------------------------------------------
// HTML PAGE
// ----------------------------------------------------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>ESP32-CAM 7-Segment Reader</title>
<style>
body { font-family:Arial; background:#f0f0f0; margin:20px; }
.container {
  max-width:800px; margin:0 auto; padding:20px; background:white;
  border-radius:10px; box-shadow:0 0 10px rgba(0,0,0,0.1);
}
.result { padding:10px; background:#e8ffe8; border-left:4px solid #4CAF50; }
button { padding:10px 20px; margin:5px; }
</style>
</head>
<body>
<div class="container">
<h2>ESP32-CAM 7-Segment Reader</h2>

<div>
<b>Last reading:</b>
<div class="result" id="reading">%LAST%</div>
</div>

<button onclick="readNow()">Read</button>

<h3>Stream</h3>
<img id="stream" src="/stream" style="max-width:100%;border:1px solid #888">

<script>
function readNow(){
  fetch('/read').then(r=>r.text()).then(t=>{
    document.getElementById('reading').textContent = t;
  });
}
document.getElementById('stream').onclick = ()=> {
  document.getElementById('stream').src='/stream?t='+Date.now();
};
</script>

</div>
</body>
</html>
)rawliteral";

// ----------------------------------------------------------
// WEBSERVER HANDLERS
// ----------------------------------------------------------
void setupServer() {

    // ROOT PAGE
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){
        String html = FPSTR(index_html);
        html.replace("%LAST%", lastResult);
        req->send(200, "text/html", html);
    });

    // READ (JPEG → RGB565 → ANALYZE)
    server.on("/read", HTTP_GET, [](AsyncWebServerRequest *req){
        String r = captureForAnalysis();
        req->send(200, "text/plain", r);
    });

    // STREAM (JPEG)
    server.on("/stream", HTTP_GET, [](AsyncWebServerRequest *req){
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            req->send(500, "text/plain", "ERR_FRAME");
            return;
        }
        req->send(200, "image/jpeg", fb->buf, fb->len);
        esp_camera_fb_return(fb);
    });

    server.begin();
    Serial.println("HTTP server started");
}

// ----------------------------------------------------------
// CAMERA INIT (single init — no reinitialization!)
// ----------------------------------------------------------
bool initCamera() {
    camera_config_t config;
    memset(&config, 0, sizeof(config));

    config.pin_pwdn  = 32;
    config.pin_reset = -1;
    config.pin_xclk  = 0;
    config.pin_sccb_sda = 26;
    config.pin_sccb_scl = 27;

    config.pin_d7 = 35;
    config.pin_d6 = 34;
    config.pin_d5 = 39;
    config.pin_d4 = 36;
    config.pin_d3 = 21;
    config.pin_d2 = 19;
    config.pin_d1 = 18;
    config.pin_d0 = 5;

    config.pin_vsync = 25;
    config.pin_href  = 23;
    config.pin_pclk  = 22;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = CAM_PIXFORMAT;
    config.frame_size   = CAM_FRAMESIZE;
    config.jpeg_quality = CAM_QUALITY;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init FAILED 0x%x\n", err);
        return false;
    }

    Serial.println("Camera OK.");
    return true;
}

// ----------------------------------------------------------
// SETUP
// ----------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\nBooting...");

    initMaskMap();

    // INIT CAMERA ONCE
    if (!initCamera()) {
        Serial.println("Fatal camera error! Rebooting...");
        delay(2000);
        ESP.restart();
    }

    // INIT JPEG DECODER
    TJpgDec.setCallback(tjpgRenderCallback);
    TJpgDec.setJpgScale(1);

    // --- WIFI ---
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("WiFi...");
    int t = 0;
    while (WiFi.status() != WL_CONNECTED && t < 30) {
        delay(500);
        Serial.print(".");
        t++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi OK, IP: " + WiFi.localIP().toString());
    } else {
        Serial.println("\nWiFi FAIL → AP mode");
        WiFi.softAP("ESP32-CAM", "12345678");
        Serial.println("AP IP: " + WiFi.softAPIP().toString());
    }

    setupServer();
}

// ----------------------------------------------------------
// LOOP — NOTHING
// ----------------------------------------------------------
void loop() {
    delay(50);
}
