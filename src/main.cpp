#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ==== WiFi ====
const char* ssid = "Keenetic-7422";
const char* password = "uPbkZw3T";

// ==== CAMERA PINS (AI Thinker) ====
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ==== FLASH LED ====
#define FLASH_GPIO 4

WebServer server(80);
WebSocketsServer webSocket(81);

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>ESP32-CAM RGB565 Stream</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        #canvas { border: 2px solid #333; background: #000; }
        .controls { margin-top: 10px; }
        .stats { margin-top: 10px; color: #666; font-size: 14px; }
    </style>
</head>
<body>
    <h2>ESP32-CAM RGB565 Live Stream</h2>
    <div>
        <canvas id="canvas" width="320" height="240"></canvas>
    </div>
    <div class="stats">
        FPS: <span id="fps">0</span> |
        Delay: <span id="delay">0</span> ms |
        Size: <span id="size">0</span> KB
    </div>
    <div class="controls">
        <button onclick="toggleStream()" id="toggleBtn">Start Stream</button>
        <button onclick="changeResolution()">Toggle Resolution</button>
        <button onclick="takeSnapshot()">Take Snapshot</button>
    </div>

    <script>
        const canvas = document.getElementById('canvas');
        const ctx = canvas.getContext('2d');
        const imgData = ctx.createImageData(canvas.width, canvas.height);
        let streaming = false;
        let frameCount = 0;
        let lastTime = 0;
        let ws;
        let currentResolution = 'qvga'; // qvga or qqvga
        
        // Функция конвертации RGB565 в RGB888
        function rgb565ToRgb888(rgb565Array) {
            const data = imgData.data;
            for (let i = 0; i < rgb565Array.length; i++) {
                const pixel = rgb565Array[i];
                
                // Распаковка RGB565 (5-6-5 бит)
                const r5 = (pixel >> 11) & 0x1F;
                const g6 = (pixel >> 5) & 0x3F;
                const b5 = pixel & 0x1F;
                
                // Конвертация в 8-битные значения (0-255)
                data[i * 4] = (r5 << 3) | (r5 >> 2);     // 5 бит -> 8 бит
                data[i * 4 + 1] = (g6 << 2) | (g6 >> 4); // 6 бит -> 8 бит
                data[i * 4 + 2] = (b5 << 3) | (b5 >> 2); // 5 бит -> 8 бит
                data[i * 4 + 3] = 255; // Альфа-канал
            }
        }
        
        function initWebSocket() {
            ws = new WebSocket('ws://' + window.location.hostname + ':81/');
            
            ws.onopen = function() {
                console.log('WebSocket connected');
                document.getElementById('toggleBtn').textContent = 'Stop Stream';
                streaming = true;
                lastTime = Date.now();
                frameCount = 0;
            };
            
            ws.onmessage = function(event) {
                if (event.data instanceof Blob) {
                    // Бинарные данные
                    const reader = new FileReader();
                    reader.onload = function() {
                        const arrayBuffer = reader.result;
                        const rgb565Array = new Uint16Array(arrayBuffer);
                        rgb565ToRgb888(rgb565Array);
                        ctx.putImageData(imgData, 0, 0);
                        
                        // Расчет FPS
                        frameCount++;
                        const now = Date.now();
                        if (now - lastTime >= 1000) {
                            const fps = Math.round((frameCount * 1000) / (now - lastTime));
                            document.getElementById('fps').textContent = fps;
                            document.getElementById('delay').textContent = Math.round(1000 / fps);
                            document.getElementById('size').textContent = (arrayBuffer.byteSize / 1024).toFixed(1);
                            frameCount = 0;
                            lastTime = now;
                        }
                    };
                    reader.readAsArrayBuffer(event.data);
                } else if (typeof event.data === 'string') {
                    // Текстовые данные (команды, статусы)
                    console.log('Message:', event.data);
                    if (event.data.startsWith('RES:')) {
                        const res = event.data.split(':')[1];
                        if (res === 'qvga') {
                            canvas.width = 320;
                            canvas.height = 240;
                        } else if (res === 'qqvga') {
                            canvas.width = 160;
                            canvas.height = 120;
                        }
                        imgData = ctx.createImageData(canvas.width, canvas.height);
                    }
                }
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
            };
            
            ws.onclose = function() {
                console.log('WebSocket disconnected');
                document.getElementById('toggleBtn').textContent = 'Start Stream';
                streaming = false;
                setTimeout(() => {
                    if (!streaming) initWebSocket();
                }, 2000);
            };
        }
        
        function toggleStream() {
            if (streaming) {
                ws.close();
                streaming = false;
                document.getElementById('toggleBtn').textContent = 'Start Stream';
            } else {
                initWebSocket();
            }
        }
        
        function changeResolution() {
            if (ws && ws.readyState === WebSocket.OPEN) {
                currentResolution = currentResolution === 'qvga' ? 'qqvga' : 'qvga';
                ws.send('SET_RES:' + currentResolution);
            }
        }
        
        function takeSnapshot() {
            const link = document.createElement('a');
            link.download = 'snapshot_' + new Date().getTime() + '.png';
            link.href = canvas.toDataURL('image/png');
            link.click();
        }
        
        // Инициализация при загрузке страницы
        window.onload = function() {
            console.log('Page loaded');
        };
    </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
      
    case WStype_CONNECTED:
      Serial.printf("[%u] Connected from %s\n", num, webSocket.remoteIP(num).toString().c_str());
      break;
      
    case WStype_TEXT:
      if (strncmp((char*)payload, "SET_RES:", 8) == 0) {
        // Изменение разрешения
        char* res = (char*)payload + 8;
        if (strcmp(res, "qqvga") == 0) {
          sensor_t *s = esp_camera_sensor_get();
          s->set_framesize(s, FRAMESIZE_QQVGA);
          webSocket.sendTXT(num, "RES:qqvga");
        } else {
          sensor_t *s = esp_camera_sensor_get();
          s->set_framesize(s, FRAMESIZE_QVGA);
          webSocket.sendTXT(num, "RES:qvga");
        }
      }
      break;
      
    case WStype_BIN:
      // Обработка бинарных данных (не используется)
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  // Flash LED
  pinMode(FLASH_GPIO, OUTPUT);
  digitalWrite(FLASH_GPIO, LOW);

  // Camera config
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  
  config.frame_size = FRAMESIZE_QVGA; // 320x240
  config.jpeg_quality = 0;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  // Init camera
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }

  Serial.print("\nReady! Open: http://");
  Serial.println(WiFi.localIP());

  // Web Server
  server.on("/", handleRoot);
  server.begin();

  // WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  server.handleClient();
  webSocket.loop();

  // Отправка кадров всем подключенным клиентам
  static uint32_t lastFrameTime = 0;
  if (webSocket.connectedClients() > 0 && millis() - lastFrameTime > 100) { // ~10 FPS
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb && fb->format == PIXFORMAT_RGB565) {
      webSocket.broadcastBIN(fb->buf, fb->len);
      esp_camera_fb_return(fb);
      lastFrameTime = millis();
    }
  }
}