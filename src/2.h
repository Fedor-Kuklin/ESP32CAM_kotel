#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// ---------------------- WiFi ----------------------
const char* WIFI_SSID = "Keenetic-7422";
const char* WIFI_PASS = "uPbkZw3T";

// ---------------------- Оптопары ----------------------
const int gpio_btn_plus  = 12;
const int gpio_btn_minus = 13;
const int gpio_btn_mode  = 14;

// ---------------------- ROI настройки ----------------------
int ROI_X = 20, ROI_Y = 10, ROI_W = 120, ROI_H = 80;

#define FRAME_SIZE FRAMESIZE_QQVGA
#define PIXFORMAT PIXFORMAT_RGB565

// ---------------------- ГЕОМЕТРИЯ СЕГМЕНТОВ ----------------------
struct Rect { int x,y,w,h; };
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
String lastResult = "";

// -----------------------------------------------------------
// Функция считывания сегментов с изображения
// -----------------------------------------------------------
String readDisplay() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return "ERR_NO_FRAME";

  int w = fb->width;
  int h = fb->height;

  String out = "";

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
    out += (digit < 0 ? "?" : String(digit));
  }

  // ---- LED индикаторы ----
  out += " | LEDs:";
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
    out += (avg >= threshLED ? '1' : '0');
  }

  esp_camera_fb_return(fb);
  lastResult = out;
  return out;
}

// -----------------------------------------------------------
// Web API
// -----------------------------------------------------------
void handleRoot() {
  server.send(200, "text/plain", "ESP32-CAM 7seg");
}

void setupServer() {
  server.on("/", handleRoot);

  server.on("/read", [](){
    server.send(200, "text/plain", readDisplay());
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

  server.begin();
}

// -----------------------------------------------------------
// SETUP
// -----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  initMaskMap();

  // Оптопары — HIGH активный (через транзистор)
  // pinMode(gpio_btn_plus, OUTPUT);
  // pinMode(gpio_btn_minus, OUTPUT);
  // pinMode(gpio_btn_mode, OUTPUT);
  // digitalWrite(gpio_btn_plus, LOW);
  // digitalWrite(gpio_btn_minus, LOW);
  // digitalWrite(gpio_btn_mode, LOW);

  // ---------------- Камера ESP32-CAM ----------------
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = 5;
  config.pin_d1       = 18;
  config.pin_d2       = 19;
  config.pin_d3       = 21;
  config.pin_d4       = 36;
  config.pin_d5       = 39;
  config.pin_d6       = 34;
  config.pin_d7       = 35;
  config.pin_xclk     = 0;
  config.pin_pclk     = 22;
  config.pin_vsync    = 25;
  config.pin_href     = 23;
  config.pin_sccb_sda = 26;
  config.pin_sccb_scl = 27;
  config.pin_pwdn     = 32;
  config.pin_reset    = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT;
  config.frame_size   = FRAME_SIZE;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

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
}

// -----------------------------------------------------------
// LOOP
// -----------------------------------------------------------
void loop() {
  server.handleClient();

  static unsigned long last = 0;
  if (millis() - last > 1500) {
    Serial.println(readDisplay());
    last = millis();
  }
}
