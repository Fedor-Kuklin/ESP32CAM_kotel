#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <config.h>


void handleGetLayout();
void handleSetLayout();
void handleGetThresholds();
void handleSetThresholds();
// ====================== GPIO CONTROL ======================
const int PIN_PLUS  = 14;   // Кнопка "Плюс"
const int PIN_MINUS = 13;   // Кнопка "Минус"
const int PIN_ENTER = 12;   // Кнопка "Ввод"

// Текущее состояние пинов
bool pinStates[3] = {false, false, false}; // [0]-plus, [1]-minus, [2]-enter

// ====================== ROI ======================
int ROI_X = 18;
int ROI_Y = 44;
int ROI_W = 140;
int ROI_H = 70;

// ====================== CAMERA ======================
#define FRAME_SIZE FRAMESIZE_QQVGA
#define PIXFORMAT  PIXFORMAT_RGB565

// ====================== GEOMETRY ======================
struct Rect { int x,y,w,h; };
const int DIGITS = 2;
const int SEGMENTS = 7;

Rect segPos[DIGITS][SEGMENTS] = {
  {
    {57, 37, 6, 3}, 
    {66, 42, 3, 6}, 
    {63, 55, 3, 6}, 
    {53, 63, 6, 3},
    {50, 55, 3, 6}, 
    {51, 41, 3, 6}, 
    {55, 50, 6, 3}
  },
  {
    {82, 37, 6, 3}, 
    {91, 42, 3, 6}, 
    {88, 55, 3, 6}, 
    {78, 63, 6, 3},
    {75, 55, 3, 6}, 
    {76, 41, 3, 6}, 
    {80, 50, 6, 3}
  }
};

Rect topLEDs[] = {
  {17,12,3,3},
  {42,12,3,3},
  {68,12,3,3},
  {94,12,3,3},
  {120,12,3,3}
};

int threshSegment = 180;
int threshLED     = 180;

// ====================== MASK ======================
// Массив соответствия маски цифрам/символам
int maskToDigit[128];

// Функция для инициализации таблицы соответствий
void initMaskMap() {
  memset(maskToDigit, -1, sizeof(maskToDigit)); // Обнуляем весь массив

  // Цифры
  maskToDigit[0b0111111] = '0'; // 0
  maskToDigit[0b0000110] = '1'; // 1
  maskToDigit[0b1011011] = '2'; // 2
  maskToDigit[0b1001111] = '3'; // 3
  maskToDigit[0b1100110] = '4'; // 4
  maskToDigit[0b1101101] = '5'; // 5
  maskToDigit[0b1111101] = '6'; // 6
  maskToDigit[0b0000111] = '7'; // 7
  maskToDigit[0b1111111] = '8'; // 8
  maskToDigit[0b1101111] = '9'; // 9

  // Дополнительные символы
  maskToDigit[0b0000001] = '-'; // Минус
  maskToDigit[0b1110110] = 'F'; // Буква F
  maskToDigit[0b00000001] = '_'; // Нижнее подчёркивание
  maskToDigit[0b1000000] = '^'; // Верхняя черта (используем ^ для обозначения верхней черты)
}

// Конфигурация MQTT брокера
const char* MQTT_BROKER = "10.9.8.8";
const int   MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "esp32-cam-meter";
const char* MQTT_TOPIC_DISPLAY = "home/meter/display"; // Значение с семисегментника
const char* MQTT_TOPIC_LEDS = "home/meter/leds";       // Состояния всех светодиодов
const char* MQTT_TOPIC_LED_PREFIX = "home/meter/led";  // Префикс для топиков каждого
                                                        //  LED (добавляется номер 1-5)
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ====================== MQTT DISCOVERY ======================
// Настройки для Home Assistant Discovery
const char* MQTT_DISCOVERY_PREFIX = "homeassistant"; // Стандартный префикс
const char* DEVICE_NAME = "ESP32-CAM";
const char* DEVICE_MANUFACTURER = "Custom ESP32";
const char* DEVICE_MODEL = "Camera Reader";
const char* DEVICE_SW_VERSION = "1.1";

// Массив для хранения состояний семисегментного индикатора (2 цифры)
char segmentDigits[3] = "00"; // Хранит текущие цифры + нулевой терминатор

// Флаги для отслеживания отправки конфигураций
bool discoveryPublished = false;
unsigned long lastDiscoveryAttempt = 0;

// ====================== MQTT УПРАВЛЕНИЕ ВЫХОДАМИ ======================
// Топики для управления выходами
const char* MQTT_TOPIC_RELAY_PLUS = "home/meter/relay/plus/set";
const char* MQTT_TOPIC_RELAY_MINUS = "home/meter/relay/minus/set";
const char* MQTT_TOPIC_RELAY_ENTER = "home/meter/relay/enter/set";

// Топики для статуса выходов (опционально)
const char* MQTT_TOPIC_RELAY_PLUS_STATE = "home/meter/relay/plus/state";
const char* MQTT_TOPIC_RELAY_MINUS_STATE = "home/meter/relay/minus/state";
const char* MQTT_TOPIC_RELAY_ENTER_STATE = "home/meter/relay/enter/state";

// Таймеры для импульсного управления (кнопка нажимается на время)
const unsigned long BUTTON_PRESS_MS = 500; // Время нажатия кнопки в мс
unsigned long buttonReleaseTime[3] = {0, 0, 0}; // [plus, minus, enter]
bool buttonShouldRelease[3] = {false, false, false};

// ====================== SERVER ======================
WebServer server(80);

// ====================== DRAW ======================
inline void drawBox(uint16_t *buf, int w, Rect r, uint16_t color) {
  for (int x=r.x; x<r.x+r.w; x++) {
    if (r.y>=0 && r.y<w) buf[r.y*w + x] = color;
    if (r.y+r.h>=0 && r.y+r.h<w) buf[(r.y+r.h)*w + x] = color;
  }
  for (int y=r.y; y<r.y+r.h; y++) {
    if (r.x>=0 && r.x<w) buf[y*w + r.x] = color;
    if (r.x+r.w>=0 && r.x+r.w<w) buf[y*w + r.x+r.w] = color;
  }
}

String lastResult = "";

// ====================== ФУНКЦИИ КАМЕРЫ ======================
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
      long sumB = 0; int cnt = 0;

      for(int yy=ay; yy<ay+r.h; yy++){
        for(int xx=ax; xx<ax+r.w; xx++){
          if(xx<0||yy<0||xx>=w||yy>=h) continue;
          int yy2 = (h - 1) - yy;
          int idx = (yy2 * w + xx) * 2;
          uint16_t pix = fb->buf[idx] | (fb->buf[idx+1] << 8);
          
          int r5 = (pix >> 11) & 0x1F;
          int g6 = (pix >> 5) & 0x3F;
          int b5 = pix & 0x1F;
          
          int gray = ((r5*255/31)*30 + (g6*255/63)*59 + (b5*255/31)*11) / 100;
          sumB += gray; cnt++;
        }
      }
      int avg = (cnt > 0) ? (sumB / cnt) : 0;
      mask |= ((avg >= threshSegment) << s);
    }
    int digit = maskToDigit[mask];
    if (digit < 0) out += "?"; else out += (char)digit;
  }

  // ---- LED индикаторы ----
  out += " | LEDs:";
  for (auto &led : topLEDs) {
    int ax = ROI_X + led.x;
    int ay = ROI_Y + led.y;
    long sumB = 0; int cnt = 0;

    for(int yy=ay; yy<ay+led.h; yy++){
      for(int xx=ax; xx<ax+led.w; xx++){
        if(xx<0||yy<0||xx>=w||yy>=h) continue;
        int yy2 = (h - 1) - yy;
        int idx = (yy2 * w + xx) * 2;
        uint16_t pix = fb->buf[idx] | (fb->buf[idx+1] << 8);
        
        int gray = (((pix >> 11) & 0x1F)*255/31*30 + 
                   ((pix >> 5) & 0x3F)*255/63*59 + 
                   (pix & 0x1F)*255/31*11) / 100;
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

// ====================== ОБРАБОТЧИКИ HTTP ======================

// Главная страница управления
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32-CAM Control</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; margin: 20px; background: #f0f0f0; }
    .container { max-width: 600px; margin: auto; background: white; padding: 20px; border-radius: 10px; }
    h2 { color: #333; text-align: center; }
    .button {
      padding: 15px 30px; margin: 10px; border: none; border-radius: 8px;
      font-size: 18px; font-weight: bold; cursor: pointer;
      transition: all 0.3s; display: inline-block; width: 150px;
    }
    .button:active { transform: scale(0.95); }
    .plus { background: #4CAF50; color: white; }
    .minus { background: #f44336; color: white; }
    .enter { background: #2196F3; color: white; }
    .button.active { box-shadow: 0 0 15px rgba(0,0,0,0.5); }
    .status {
      padding: 15px; margin: 20px 0; border-radius: 8px;
      background: #e8f5e8; border: 1px solid #ddd;
    }
    .pin-status { font-weight: bold; font-size: 1.2em; }
    .stream-container { text-align: center; margin: 20px 0; }
    .link-button { 
      display: block; padding: 10px; margin: 10px 0; 
      background: #FF9800; color: white; text-align: center;
      text-decoration: none; border-radius: 5px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>ESP32-CAM Control Panel</h2>
    
    <div style="text-align: center;">
      <h3>Virtual Buttons</h3>
      <button id="btnPlus" class="button plus" 
              onmousedown="controlPin('plus', true)" 
              ontouchstart="controlPin('plus', true)"
              onmouseup="controlPin('plus', false)" 
              ontouchend="controlPin('plus', false)">
        PLUS (GPIO14)
      </button><br>
      
      <button id="btnMinus" class="button minus" 
              onmousedown="controlPin('minus', true)" 
              ontouchstart="controlPin('minus', true)"
              onmouseup="controlPin('minus', false)" 
              ontouchend="controlPin('minus', false)">
        MINUS (GPIO13)
      </button><br>
      
      <button id="btnEnter" class="button enter" 
              onmousedown="controlPin('enter', true)" 
              ontouchstart="controlPin('enter', true)"
              onmouseup="controlPin('enter', false)" 
              ontouchend="controlPin('enter', false)">
        ENTER (GPIO12)
      </button>
    </div>
    
    <div class="status">
      <h3>Current Status</h3>
      <p>GPIO14 (PLUS): <span id="pinPlus" class="pin-status">-</span></p>
      <p>GPIO13 (MINUS): <span id="pinMinus" class="pin-status">-</span></p>
      <p>GPIO12 (ENTER): <span id="pinEnter" class="pin-status">-</span></p>
      <p>Last Display: <span id="lastResult">-</span></p>
    </div>
    
    <div class="stream-container">
      <h3>Camera Stream</h3>
      <p>Stream starts automatically. Click below to view full screen:</p>
      <a href="/stream" class="link-button">Open Video Stream Page</a>
      <div style="margin-top: 15px;">
        <div style="margin-bottom:10px; text-align:left;">
          <strong>ROI (Region of Interest):</strong><br>
          <label>X: <input id="roiX" type="number" style="width:70px"></label>
          <label>Y: <input id="roiY" type="number" style="width:70px"></label>
          <label style="margin-left:10px;"><input id="roiAuto" type="checkbox"> Auto ROI</label>
            <button onclick="applyROI()" style="margin-left:10px;padding:6px 10px;">Apply</button>
            <div style="margin-top:8px;">
              <label>Segment threshold: <input id="threshSeg" type="number" style="width:80px"></label>
              <label style="margin-left:10px;">LED threshold: <input id="threshLed" type="number" style="width:80px"></label>
              <button onclick="applyThresholds()" style="margin-left:10px;padding:6px 10px;">Set Thresholds</button>
            </div>
        </div>
        <img src="/frame?roi=1" width="320" id="streamImg">
      </div>
      
      <div style="margin-top:20px; text-align:left;">
        <h3>Layout Configuration</h3>
        <p>Edit segment rectangles and top LEDs positions:</p>
        <div>
          <div id="segTables"></div>
          <h4>Top LEDs</h4>
          <table id="ledTable" border="1" style="border-collapse:collapse;margin-top:8px;">
            <thead><tr><th>#</th><th>X</th><th>Y</th><th>W</th><th>H</th></tr></thead>
            <tbody></tbody>
          </table>
          <div style="margin-top:8px;">
            <button onclick="applyLayout()" style="padding:6px 10px;">Save Layout</button>
            <button onclick="loadLayout()" style="padding:6px 10px; margin-left:6px;">Reload</button>
          </div>
        </div>
      </div>
    </div>
  </div>

  <script>
    let updateInterval;
    
    function controlPin(pin, state) {
      fetch(`/control?pin=${pin}&state=${state ? 1 : 0}`)
        .then(r => r.text())
        .then(() => updateStatus());
      
      // Визуальная обратная связь
      const btn = document.getElementById(`btn${pin.charAt(0).toUpperCase() + pin.slice(1)}`);
      if (state) {
        btn.classList.add('active');
      } else {
        btn.classList.remove('active');
      }
    }
    
    function updateStatus() {
      fetch('/pinstatus')
        .then(r => r.json())
        .then(data => {
          document.getElementById('pinPlus').textContent = data.plus ? 'ACTIVE' : 'INACTIVE';
          document.getElementById('pinMinus').textContent = data.minus ? 'ACTIVE' : 'INACTIVE';
          document.getElementById('pinEnter').textContent = data.enter ? 'ACTIVE' : 'INACTIVE';
          document.getElementById('lastResult').textContent = data.last_display || '-';
          
          // Обновляем изображение потока
          document.getElementById('streamImg').src = `/frame?roi=1&t=${Date.now()}`;
        });
    }
    
    // Автообновление статуса и изображения
    function startAutoUpdate() {
      updateInterval = setInterval(updateStatus, 500); // Обновление каждые 500мс
    }
    
    // Инициализация
    document.addEventListener('DOMContentLoaded', function() {
      updateStatus();
      startAutoUpdate();
      // Загружаем текущие ROI и заполняем поля
      fetch('/roi').then(r=>r.json()).then(data=>{
        document.getElementById('roiX').value = data.x;
        document.getElementById('roiY').value = data.y;
        // Оставляем Auto unchecked по умолчанию
      }).catch(()=>{});
      // Load thresholds
      fetch('/thresholds').then(r=>r.json()).then(t=>{
        document.getElementById('threshSeg').value = t.seg;
        document.getElementById('threshLed').value = t.led;
      }).catch(()=>{});
    });

    function applyROI() {
      const x = document.getElementById('roiX').value;
      const y = document.getElementById('roiY').value;
      const isAuto = document.getElementById('roiAuto').checked;
      let url = `/setroi?x=${x}&y=${y}`;
      if (isAuto) url += '&auto=1';
      fetch(url)
        .then(r=>{
          if (r.ok) {
            updateStatus();
          } else {
            alert('Failed to set ROI');
          }
        });
    }

    function applyThresholds() {
      const seg = document.getElementById('threshSeg').value;
      const led = document.getElementById('threshLed').value;
      fetch(`/setthresholds?seg=${seg}&led=${led}`).then(r=>{
        if (r.ok) updateStatus(); else alert('Failed to set thresholds');
      });
    }

    // Layout functions
    function createSegTable(d) {
      let html = `<h4>Digit ${d}</h4><table border="1" style="border-collapse:collapse;"><thead><tr><th>#</th><th>X</th><th>Y</th><th>W</th><th>H</th></tr></thead><tbody>`;
      for (let s=0; s<7; s++) {
        html += `<tr><td>${s}</td>`+
                `<td><input id="seg_${d}_${s}_x" type="number" style="width:60px"></td>`+
                `<td><input id="seg_${d}_${s}_y" type="number" style="width:60px"></td>`+
                `<td><input id="seg_${d}_${s}_w" type="number" style="width:60px"></td>`+
                `<td><input id="seg_${d}_${s}_h" type="number" style="width:60px"></td></tr>`;
      }
      html += `</tbody></table>`;
      return html;
    }

    function loadLayout() {
      fetch('/getlayout').then(r=>r.json()).then(data=>{
        // segPos
        for (let d=0; d < data.segPos.length; d++) {
          let segs = data.segPos[d];
          for (let s=0; s<segs.length; s++) {
            document.getElementById(`seg_${d}_${s}_x`).value = segs[s].x;
            document.getElementById(`seg_${d}_${s}_y`).value = segs[s].y;
            document.getElementById(`seg_${d}_${s}_w`).value = segs[s].w;
            document.getElementById(`seg_${d}_${s}_h`).value = segs[s].h;
          }
        }
        // topLEDs
        let tbody = document.querySelector('#ledTable tbody');
        tbody.innerHTML = '';
        for (let i=0; i < data.topLEDs.length; i++) {
          let l = data.topLEDs[i];
          let row = document.createElement('tr');
          row.innerHTML = `<td>${i}</td>`+
                          `<td><input id="led_${i}_x" type="number" style="width:60px" value="${l.x}"></td>`+
                          `<td><input id="led_${i}_y" type="number" style="width:60px" value="${l.y}"></td>`+
                          `<td><input id="led_${i}_w" type="number" style="width:60px" value="${l.w}"></td>`+
                          `<td><input id="led_${i}_h" type="number" style="width:60px" value="${l.h}"></td>`;
          tbody.appendChild(row);
        }
      }).catch(e=>{ console.log('loadLayout error', e); });
    }

    function applyLayout() {
      let obj = { segPos: [], topLEDs: [] };
      for (let d=0; d<2; d++) {
        let arr = [];
        for (let s=0; s<7; s++) {
          arr.push({
            x: parseInt(document.getElementById(`seg_${d}_${s}_x`).value || 0),
            y: parseInt(document.getElementById(`seg_${d}_${s}_y`).value || 0),
            w: parseInt(document.getElementById(`seg_${d}_${s}_w`).value || 0),
            h: parseInt(document.getElementById(`seg_${d}_${s}_h`).value || 0)
          });
        }
        obj.segPos.push(arr);
      }
      // LEDs
      let tbody = document.querySelectorAll('#ledTable tbody tr');
      for (let i=0;i<tbody.length;i++) {
        obj.topLEDs.push({
          x: parseInt(document.getElementById(`led_${i}_x`).value || 0),
          y: parseInt(document.getElementById(`led_${i}_y`).value || 0),
          w: parseInt(document.getElementById(`led_${i}_w`).value || 0),
          h: parseInt(document.getElementById(`led_${i}_h`).value || 0)
        });
      }

      fetch('/setlayout', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(obj)})
        .then(r=>{ if (r.ok) alert('Layout saved'); else alert('Save failed'); });
    }

    // create seg tables and load layout once DOM ready
    document.addEventListener('DOMContentLoaded', function() {
      let st = document.getElementById('segTables');
      st.innerHTML = createSegTable(0) + createSegTable(1);
      loadLayout();
    });
  </script>
</body>
</html>
)rawliteral";
  
  server.send(200, "text/html", html);
}

// Страница с потоковым видео (отдельная)
void handleStream() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32-CAM Live Stream</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { margin: 0; padding: 0; background: #000; text-align: center; }
    .container { max-width: 100%; margin: 0 auto; padding: 10px; }
    h1 { color: white; margin: 10px; }
    .back-link { 
      display: inline-block; padding: 10px 20px; margin: 10px; 
      background: #2196F3; color: white; text-decoration: none;
      border-radius: 5px; font-size: 16px;
    }
    img { max-width: 100%; height: auto; border: 2px solid #555; }
  </style>
</head>
<body>
  <div class="container">
    <h1>ESP32-CAM Live Stream</h1>
    <a href="/" class="back-link">Back to Control Panel</a>
    <div style="margin-top: 20px;">
      <div style="margin-bottom:10px; text-align:left;">
        <strong>ROI (Region of Interest):</strong><br>
        <label>X: <input id="sRoiX" type="number" style="width:70px"></label>
        <label>Y: <input id="sRoiY" type="number" style="width:70px"></label>
        <label style="margin-left:10px;"><input id="sRoiAuto" type="checkbox"> Auto ROI</label>
        <button onclick="applyStreamROI()" style="margin-left:10px;padding:6px 10px;">Apply</button>
      </div>
      <img id="stream" src="/frame?roi=1">
    </div>
  </div>
  
  <script>
    // Автоматическое обновление потока
    function updateStream() {
      document.getElementById('stream').src = `/frame?roi=1&t=${Date.now()}`;
    }
    
    // Обновление каждые 100мс для плавного потока
    setInterval(updateStream, 100);

    // Загружаем текущие ROI для полей
    fetch('/roi').then(r=>r.json()).then(data=>{
      document.getElementById('sRoiX').value = data.x;
      document.getElementById('sRoiY').value = data.y;
    }).catch(()=>{});

    function applyStreamROI() {
      const x = document.getElementById('sRoiX').value;
      const y = document.getElementById('sRoiY').value;
      const isAuto = document.getElementById('sRoiAuto').checked;
      let url = `/setroi?x=${x}&y=${y}`;
      if (isAuto) url += '&auto=1';
      fetch(url).then(r=>{ if (!r.ok) alert('Failed to set ROI'); });
    }

    // Начальное обновление
    updateStream();
  </script>
</body>
</html>
)rawliteral";
  
  server.send(200, "text/html", html);
}

// Обработчик управления пинами (исправленный)
void handleControl() {
  if (server.hasArg("pin") && server.hasArg("state")) {
    String pin = server.arg("pin");
    bool state = server.arg("state").toInt() == 1;
    
    // Обновляем состояние пинов
    if (pin == "plus") {
      pinStates[0] = state;
      digitalWrite(PIN_PLUS, state ? HIGH : LOW);
    } 
    else if (pin == "minus") {
      pinStates[1] = state;
      digitalWrite(PIN_MINUS, state ? HIGH : LOW);
    } 
    else if (pin == "enter") {
      pinStates[2] = state;
      digitalWrite(PIN_ENTER, state ? HIGH : LOW);
    }
    
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing parameters");
  }
}

// JSON статус пинов
void handlePinStatus() {
  String json = "{";
  json += "\"plus\":" + String(pinStates[0] ? "true" : "false") + ",";
  json += "\"minus\":" + String(pinStates[1] ? "true" : "false") + ",";
  json += "\"enter\":" + String(pinStates[2] ? "true" : "false") + ",";
  json += "\"last_display\":\"" + lastResult + "\"";
  json += "}";
  
  server.send(200, "application/json", json);
}

// Возвращает текущие координаты ROI в JSON
void handleGetROI() {
  String json = "{";
  json += "\"x\":" + String(ROI_X) + ",";
  json += "\"y\":" + String(ROI_Y) + ",";
  json += "\"w\":" + String(ROI_W) + ",";
  json += "\"h\":" + String(ROI_H);
  json += "}";
  server.send(200, "application/json", json);
}

// Устанавливает координаты ROI через query-параметры x,y,w,h
void handleSetROI() {
  bool changed = false;
  if (server.hasArg("x")) {
    int v = server.arg("x").toInt(); if (v >= 0) { ROI_X = v; changed = true; }
  }
  if (server.hasArg("y")) {
    int v = server.arg("y").toInt(); if (v >= 0) { ROI_Y = v; changed = true; }
  }

  // Если указан auto=1 - вычисляем ширину/высоту автоматически
  if (server.hasArg("auto") && server.arg("auto").toInt() == 1) {
    int maxX = 0;
    int maxY = 0;
    // сегменты
    for (int d = 0; d < DIGITS; d++) {
      for (int s = 0; s < SEGMENTS; s++) {
        int ex = segPos[d][s].x + segPos[d][s].w;
        int ey = segPos[d][s].y + segPos[d][s].h;
        if (ex > maxX) maxX = ex;
        if (ey > maxY) maxY = ey;
      }
    }
    // светодиоды
    for (unsigned int i = 0; i < sizeof(topLEDs)/sizeof(topLEDs[0]); i++) {
      int ex = topLEDs[i].x + topLEDs[i].w;
      int ey = topLEDs[i].y + topLEDs[i].h;
      if (ex > maxX) maxX = ex;
      if (ey > maxY) maxY = ey;
    }
    const int MARGIN = 4;
    ROI_W = maxX + MARGIN;
    ROI_H = maxY + MARGIN;
    changed = true;
  } else {
    // Для обратной совместимости можно передать w/h, но UI использует только x/y
    if (server.hasArg("w")) {
      int v = server.arg("w").toInt(); if (v > 0) { ROI_W = v; changed = true; }
    }
    if (server.hasArg("h")) {
      int v = server.arg("h").toInt(); if (v > 0) { ROI_H = v; changed = true; }
    }
  }

  if (changed) {
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Missing or invalid parameters");
  }
}

// Обработчик видео (упрощенный)
void handleFrame() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "Camera error");
    return;
  }

  // Всегда рисуем ROI
  uint16_t *p = (uint16_t*)fb->buf;
  int H = fb->height;
  
  // ROI прямоугольник
  Rect r = { ROI_X, ROI_Y, ROI_W, ROI_H };
  r.y = H - (r.y + r.h);
  drawBox(p, fb->width, r, 0x07E0); // Зеленый

  // Сегменты
  for (auto &d : segPos) {
    for (auto &s : d) {
      Rect r2;
      r2.x = ROI_X + s.x;
      r2.y = ROI_Y + s.y;
      r2.w = s.w;
      r2.h = s.h;
      r2.y = H - (r2.y + r2.h);
      drawBox(p, fb->width, r2, 0xFFE0); // Желтый
    }
  }

  // LED индикаторы
  for (auto &l : topLEDs) {
    Rect r3;
    r3.x = ROI_X + l.x;
    r3.y = ROI_Y + l.y;
    r3.w = l.w;
    r3.h = l.h;
    r3.y = H - (r3.y + r3.h);
    drawBox(p, fb->width, r3, 0xF800); // Красный
  }

  // Отправка BMP
  uint32_t size = 54 + fb->len;
  uint8_t h[54] = {
    'B','M', size, size>>8, size>>16, size>>24, 0,0, 0,0, 54,0,0,0,
    40,0,0,0, fb->width, fb->width>>8, 0,0, fb->height, fb->height>>8, 0,0,
    1,0, 16,0
  };

  WiFiClient c = server.client();
  c.println("HTTP/1.1 200 OK");
  c.println("Content-Type: image/bmp");
  c.println("Connection: close");
  c.println();
  c.write(h, 54);
  c.write(fb->buf, fb->len);

  esp_camera_fb_return(fb);
}

// Маппинг индикаторов на названия для Home Assistant
const char* ledNames[] = {
    "Контур отопления",     // LED_1 Контур отопления
    "Контур ГВС",         // LED_2 Контур ГВС
    "кВт",               // LED_3 «кВт»
    "Давление в отоплении", // LED_4 Давление в отоплении
    "Контактор"         // LED_5 Контактор
};

// ====================== ФУНКЦИИ MQTT ======================
void connectToMqtt() {
    DEBUG_PRINT("Connecting to MQTT...");
    
    if (mqttClient.connected()) {
        mqttClient.disconnect();
        delay(100);
    }
    
    // Проверяем размер буфера
    if (mqttClient.getBufferSize() < 1024) {
        DEBUG_PRINTLN("\n⚠️  Buffer size too small, setting to 1024...");
        mqttClient.setBufferSize(1024);
    }
    
    unsigned long startTime = millis();
    int attempts = 0;
    
    while (!mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS) && attempts < 5) {
        attempts++;
        DEBUG_PRINT(".");
        delay(2000);
        
        if (attempts >= 3) {
            DEBUG_PRINTLN("\n⚠️  Multiple failures, checking connection...");
            // Можно добавить сброс WiFi при необходимости
        }
    }
    
    if (mqttClient.connected()) {
        DEBUG_PRINTLN(" ✅");
        DEBUG_PRINT("Connected in ");
        DEBUG_PRINT(millis() - startTime);
        DEBUG_PRINTLN(" ms");
        DEBUG_PRINT("MQTT buffer size: ");
        DEBUG_PRINTLN(mqttClient.getBufferSize());
        
        // Подписываемся на топики управления
        mqttClient.subscribe(MQTT_TOPIC_RELAY_PLUS);
        mqttClient.subscribe(MQTT_TOPIC_RELAY_MINUS);
        mqttClient.subscribe(MQTT_TOPIC_RELAY_ENTER);
        
        // Сразу публикуем статус
        mqttClient.publish("home/meter/status", "online", true);
        
    } else {
        DEBUG_PRINTLN(" ❌");
        DEBUG_PRINT("Failed after ");
        DEBUG_PRINT(attempts);
        DEBUG_PRINTLN(" attempts");
        DEBUG_PRINT("Error code: ");
        DEBUG_PRINTLN(mqttClient.state());
    }
}

void publishMqttData(const String& displayValue, const String& ledStates) {
    if (!mqttClient.connected()) return;
    
    // 1. Извлекаем цифры из семисегментного индикатора
    if (displayValue.length() >= 2) {
        String twoDigits = displayValue.substring(0, 2);
        
        // Публикуем только если значение изменилось
        if (twoDigits != String(segmentDigits)) {
            strncpy(segmentDigits, twoDigits.c_str(), sizeof(segmentDigits)-1);
            mqttClient.publish(MQTT_TOPIC_DISPLAY, segmentDigits, true); // retain=true
            DEBUG_PRINTLN("Published to " + String(MQTT_TOPIC_DISPLAY) + ": " + twoDigits);
        }
    }

    // 2. Обрабатываем состояния светодиодов
    if (ledStates.startsWith(" | LEDs:")) {
        String states = ledStates.substring(8); // Получаем подстроку после " | LEDs:"
        
        // Публикуем все состояния в одном топике
        mqttClient.publish(MQTT_TOPIC_LEDS, ledStates.c_str(), true);
        
        // Публикуем каждый светодиод в отдельный топик
        for (int i = 0; i < 5 && i < states.length(); i++) {
            char stateChar = states[i];
            String topic = String(MQTT_TOPIC_LED_PREFIX) + (i + 1);
            String payload = (stateChar == '1') ? "ON" : "OFF";
            
            mqttClient.publish(topic.c_str(), payload.c_str(), true); // retain=true
            
            // Для отладки - логируем изменения
            static char lastLedStates[6] = "00000";
            if (stateChar != lastLedStates[i]) {
                DEBUG_PRINTLN("LED " + String(i+1) + " (" + ledNames[i] + "): " + payload);
                lastLedStates[i] = stateChar;
            }
        }
    }
}

// ====================== УПРАВЛЕНИЕ КНОПКАМИ ======================
void handleButtonRelease() {
    unsigned long currentTime = millis();
    
    for (int i = 0; i < 3; i++) {
        if (buttonShouldRelease[i] && currentTime >= buttonReleaseTime[i]) {
            switch(i) {
                case 0: // PLUS
                    digitalWrite(PIN_PLUS, LOW);
                    pinStates[0] = false;
                    mqttClient.publish(MQTT_TOPIC_RELAY_PLUS_STATE, "OFF", true);
                    break;
                case 1: // MINUS
                    digitalWrite(PIN_MINUS, LOW);
                    pinStates[1] = false;
                    mqttClient.publish(MQTT_TOPIC_RELAY_MINUS_STATE, "OFF", true);
                    break;
                case 2: // ENTER
                    digitalWrite(PIN_ENTER, LOW);
                    pinStates[2] = false;
                    mqttClient.publish(MQTT_TOPIC_RELAY_ENTER_STATE, "OFF", true);
                    break;
            }
            buttonShouldRelease[i] = false;
            DEBUG_PRINTLN("Button " + String(i) + " released automatically");
        }
    }
}

// ====================== ФУНКЦИИ DISCOVERY ======================
void publishHomeAssistantDiscovery() {
  DEBUG_PRINTLN("\n🔍 Publishing Home Assistant MQTT Discovery...");
  
  if (!mqttClient.connected()) {
    DEBUG_PRINTLN("MQTT not connected, skipping...");
    return;
  }
  
  // Проверяем и логируем размер буфера
  DEBUG_PRINT("Current MQTT buffer: ");
  DEBUG_PRINT(mqttClient.getBufferSize());
  DEBUG_PRINTLN(" bytes");
  
  const char* availabilityTopic = "home/meter/status";
  const char* deviceId = "esp32_meter_reader";
  
  // 1. СЕНСОР ДИСПЛЕЯ (исправленная версия)
  {
    JsonDocument doc; // Вместо StaticJsonDocument<512>
    
    doc["name"] = "Показания индикатора";
    doc["state_topic"] = MQTT_TOPIC_DISPLAY;
    doc["unit_of_measurement"] = "";
    doc["value_template"] = "{{ value }}";
    doc["icon"] = "mdi:led-outline";
    doc["unique_id"] = "esp32_meter_display";
    doc["availability_topic"] = availabilityTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    
    JsonObject device = doc["device"].to<JsonObject>(); // Вместо createNestedObject
    device["name"] = DEVICE_NAME;
    device["manufacturer"] = DEVICE_MANUFACTURER;
    device["model"] = DEVICE_MODEL;
    device["sw_version"] = DEVICE_SW_VERSION;
    
    JsonArray identifiers = device["identifiers"].to<JsonArray>(); // Вместо createNestedArray
    identifiers.add(deviceId);
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/meter_display/config";
    
    DEBUG_PRINT("📊 Display sensor (");
    DEBUG_PRINT(payload.length());
    DEBUG_PRINT("b)... ");
    
    bool success = mqttClient.publish(topic.c_str(), payload.c_str(), true);
    DEBUG_PRINTLN(success ? "✅" : "❌");
    
    delay(200);
  }
  
  // 2. СВЕТОДИОДЫ (исправленная версия)
  const char* ledDeviceClasses[] = {"power", "power", "power", "power", "power"};
  const char* ledIcons[] = {"mdi:radiator", "mdi:water-boiler", "mdi:flash", "mdi:gauge", "mdi:electric-switch"};
  
  for (int i = 0; i < 5; i++) {
    JsonDocument doc; // Вместо StaticJsonDocument<512>
    
    doc["name"] = ledNames[i];
    doc["state_topic"] = String(MQTT_TOPIC_LED_PREFIX) + (i + 1);
    doc["payload_on"] = "ON";
    doc["payload_off"] = "OFF";
    doc["device_class"] = ledDeviceClasses[i];
    doc["icon"] = ledIcons[i];
    doc["unique_id"] = "esp32_meter_led" + String(i + 1);
    doc["availability_topic"] = availabilityTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    
    JsonObject device = doc["device"].to<JsonObject>();
    device["name"] = DEVICE_NAME;
    JsonArray identifiers = device["identifiers"].to<JsonArray>();
    identifiers.add(deviceId);
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = String(MQTT_DISCOVERY_PREFIX) + "/binary_sensor/meter_led" + (i + 1) + "/config";
    
    DEBUG_PRINT("💡 ");
    DEBUG_PRINT(ledNames[i]);
    DEBUG_PRINT(" (");
    DEBUG_PRINT(payload.length());
    DEBUG_PRINT("b)... ");
    
    bool success = mqttClient.publish(topic.c_str(), payload.c_str(), true);
    DEBUG_PRINTLN(success ? "✅" : "❌");
    
    delay(200);
  }
  
  // 3. КНОПКИ УПРАВЛЕНИЯ (исправленная версия)
  const char* buttonNames[] = {"Плюс", "Минус", "Ввод"};
  const char* buttonTopicsSet[] = {
    MQTT_TOPIC_RELAY_PLUS, 
    MQTT_TOPIC_RELAY_MINUS, 
    MQTT_TOPIC_RELAY_ENTER
  };
  const char* buttonTopicsState[] = {
    MQTT_TOPIC_RELAY_PLUS_STATE, 
    MQTT_TOPIC_RELAY_MINUS_STATE, 
    MQTT_TOPIC_RELAY_ENTER_STATE
  };
  
  for (int i = 0; i < 3; i++) {
    JsonDocument doc; // Вместо StaticJsonDocument<512>
    
    doc["name"] = buttonNames[i];
    doc["command_topic"] = buttonTopicsSet[i];
    doc["state_topic"] = buttonTopicsState[i];
    doc["payload_press"] = "PRESS";
    doc["optimistic"] = false;
    doc["retain"] = false;
    doc["icon"] = "mdi:button-pointer";
    doc["unique_id"] = "esp32_meter_button" + String(i + 1);
    doc["availability_topic"] = availabilityTopic;
    doc["payload_available"] = "online";
    doc["payload_not_available"] = "offline";
    
    JsonObject device = doc["device"].to<JsonObject>();
    device["name"] = DEVICE_NAME;
    JsonArray identifiers = device["identifiers"].to<JsonArray>();
    identifiers.add(deviceId);
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = String(MQTT_DISCOVERY_PREFIX) + "/button/meter_button" + (i + 1) + "/config";
    
    DEBUG_PRINT("🔘 ");
    DEBUG_PRINT(buttonNames[i]);
    DEBUG_PRINT(" (");
    DEBUG_PRINT(payload.length());
    DEBUG_PRINT("b)... ");
    
    bool success = mqttClient.publish(topic.c_str(), payload.c_str(), true);
    DEBUG_PRINTLN(success ? "✅" : "❌");
    
    delay(200);
  }
  
  // 4. ПУБЛИКАЦИЯ НАЧАЛЬНЫХ ДАННЫХ
  DEBUG_PRINTLN("\n📤 Publishing initial data...");
  
  // Дисплей
  mqttClient.publish(MQTT_TOPIC_DISPLAY, "00", true);
  
  // Светодиоды
  for (int i = 1; i <= 5; i++) {
    String topic = String(MQTT_TOPIC_LED_PREFIX) + i;
    mqttClient.publish(topic.c_str(), "OFF", true);
  }
  
  // Кнопки
  for (int i = 0; i < 3; i++) {
    mqttClient.publish(buttonTopicsState[i], "OFF", true);
  }
  
  discoveryPublished = true;
  DEBUG_PRINTLN("\n🎯 Discovery completed successfully!");
  DEBUG_PRINTLN("Check Home Assistant: Devices & Services → MQTT");
}

// ====================== MQTT CALLBACK ======================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // Преобразуем payload в строку
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    DEBUG_PRINT("MQTT Message arrived [");
    DEBUG_PRINT(topic);
    DEBUG_PRINT("]: ");
    DEBUG_PRINTLN(message);
    
    // Обрабатываем команды для выходов
    if (String(topic) == MQTT_TOPIC_RELAY_PLUS) {
        if (message == "ON" || message == "1" || message == "PRESS") {
            digitalWrite(PIN_PLUS, HIGH);
            pinStates[0] = true;
            buttonShouldRelease[0] = true;
            buttonReleaseTime[0] = millis() + BUTTON_PRESS_MS;
            
            // Публикуем состояние
            mqttClient.publish(MQTT_TOPIC_RELAY_PLUS_STATE, "ON", true);
            DEBUG_PRINTLN("PLUS button pressed via MQTT");
        }
        else if (message == "OFF" || message == "0") {
            digitalWrite(PIN_PLUS, LOW);
            pinStates[0] = false;
            buttonShouldRelease[0] = false;
            mqttClient.publish(MQTT_TOPIC_RELAY_PLUS_STATE, "OFF", true);
        }
    }
    else if (String(topic) == MQTT_TOPIC_RELAY_MINUS) {
        if (message == "ON" || message == "1" || message == "PRESS") {
            digitalWrite(PIN_MINUS, HIGH);
            pinStates[1] = true;
            buttonShouldRelease[1] = true;
            buttonReleaseTime[1] = millis() + BUTTON_PRESS_MS;
            
            mqttClient.publish(MQTT_TOPIC_RELAY_MINUS_STATE, "ON", true);
            DEBUG_PRINTLN("MINUS button pressed via MQTT");
        }
        else if (message == "OFF" || message == "0") {
            digitalWrite(PIN_MINUS, LOW);
            pinStates[1] = false;
            buttonShouldRelease[1] = false;
            mqttClient.publish(MQTT_TOPIC_RELAY_MINUS_STATE, "OFF", true);
        }
    }
    else if (String(topic) == MQTT_TOPIC_RELAY_ENTER) {
        if (message == "ON" || message == "1" || message == "PRESS") {
            digitalWrite(PIN_ENTER, HIGH);
            pinStates[2] = true;
            buttonShouldRelease[2] = true;
            buttonReleaseTime[2] = millis() + BUTTON_PRESS_MS;
            
            mqttClient.publish(MQTT_TOPIC_RELAY_ENTER_STATE, "ON", true);
            DEBUG_PRINTLN("ENTER button pressed via MQTT");
        }
        else if (message == "OFF" || message == "0") {
            digitalWrite(PIN_ENTER, LOW);
            pinStates[2] = false;
            buttonShouldRelease[2] = false;
            mqttClient.publish(MQTT_TOPIC_RELAY_ENTER_STATE, "OFF", true);
        }
    }
}

void publishMeterData(const String& result) {
    if (!mqttClient.connected() || result.length() == 0) return;
    
    // Парсим результат в формате: "12 | LEDs:10101"
    int pipePos = result.indexOf('|');
    
    if (pipePos > 0) {
        // Цифры с семисегментника
        String digits = result.substring(0, 2);
        if (digits.length() == 2 && isDigit(digits[0]) && isDigit(digits[1])) {
            static String lastDigits = "";
            if (digits != lastDigits) { // Публикуем только при изменении
                mqttClient.publish(MQTT_TOPIC_DISPLAY, digits.c_str(), true);
                lastDigits = digits;
                DEBUG_PRINTLN("Published digits: " + digits);
            }
        }
        
        // Светодиоды
        String ledsPart = result.substring(pipePos);
        if (ledsPart.indexOf("LEDs:") >= 0) {
            int ledsStart = ledsPart.indexOf("LEDs:") + 5;
            String ledStates = ledsPart.substring(ledsStart);
            
            static String lastLedStates[5] = {"", "", "", "", ""};
            
            for (int i = 0; i < 5 && i < ledStates.length(); i++) {
                String topic = String(MQTT_TOPIC_LED_PREFIX) + (i + 1);
                String state = (ledStates[i] == '1') ? "ON" : "OFF";
                
                if (state != lastLedStates[i]) { // Публикуем только при изменении
                    mqttClient.publish(topic.c_str(), state.c_str(), true);
                    lastLedStates[i] = state;
                    
                    DEBUG_PRINT("LED");
                    DEBUG_PRINT(i + 1);
                    DEBUG_PRINT(" (");
                    DEBUG_PRINT(ledNames[i]);
                    DEBUG_PRINT("): ");
                    DEBUG_PRINTLN(state);
                }
            }
        }
    }
}

void checkSystemHealth() {
    static unsigned long lastHeapCheck = 0;
    
    if (millis() - lastHeapCheck > 60000) { // Каждую минуту
        uint32_t freeHeap = ESP.getFreeHeap();
        DEBUG_PRINT("System health - Free heap: ");
        DEBUG_PRINTLN(freeHeap);
        
        if (freeHeap < 5000) { // Критически мало памяти
            DEBUG_PRINTLN("⚠️  Critical memory low! Restarting...");
            delay(1000);
            ESP.restart();
        }
        
        lastHeapCheck = millis();
    }
}

// ====================== SETUP ======================
void setup() {
  Serial.begin(115200);
  DEBUG_PRINTLN("\n\nESP32-CAM Starting...");
  
  initMaskMap();
  
  // Инициализация GPIO
  pinMode(PIN_PLUS, OUTPUT);
  pinMode(PIN_MINUS, OUTPUT);
  pinMode(PIN_ENTER, OUTPUT);
  
  // Устанавливаем LOW по умолчанию
  digitalWrite(PIN_PLUS, LOW);
  digitalWrite(PIN_MINUS, LOW);
  digitalWrite(PIN_ENTER, LOW);
  
  // Инициализация камеры
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5; config.pin_d1 = 18; config.pin_d2 = 19; config.pin_d3 = 21;
  config.pin_d4 = 36; config.pin_d5 = 39; config.pin_d6 = 34; config.pin_d7 = 35;
  config.pin_xclk = 0; config.pin_pclk = 22; config.pin_vsync = 25;
  config.pin_href = 23; config.pin_sccb_sda = 26; config.pin_sccb_scl = 27;
  config.pin_pwdn = 32; config.pin_reset = -1;
  config.xclk_freq_hz = 16000000;
  config.pixel_format = PIXFORMAT;
  config.frame_size = FRAME_SIZE;
  config.jpeg_quality = 0;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_LATEST; // Получаем последний кадр

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    DEBUG_PRINTF("Camera init failed: 0x%x\n", err);
    return;
  }
  
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1); // Коррекция ориентации
  
  // Подключение к WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  DEBUG_PRINT("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DEBUG_PRINT(".");
  }
  DEBUG_PRINTLN("\nConnected!");
  DEBUG_PRINT("IP Address: ");
  DEBUG_PRINTLN(WiFi.localIP());

        // ===== ИНИЦИАЛИЗАЦИЯ MQTT =====
  mqttClient.setBufferSize(1024);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setSocketTimeout(15);  // Увеличиваем timeout
  mqttClient.setKeepAlive(60);      // Keep-alive 60 секунд

  connectToMqtt();

  // Регистрация обработчиков
  server.on("/", handleRoot);
  server.on("/stream", handleStream);        // Отдельная страница потока
  server.on("/frame", handleFrame);          // Изображение с разметкой
  server.on("/control", handleControl);      // Управление пинами
  server.on("/pinstatus", handlePinStatus);  // Статус пинов
  server.on("/roi", handleGetROI);           // Получить текущие ROI
  server.on("/setroi", handleSetROI);        // Установить ROI (x,y,w,h)
  server.on("/getlayout", handleGetLayout);  // Получить таблицу segPos/topLEDs
  server.on("/setlayout", HTTP_POST, handleSetLayout); // Установить новую таблицу
  server.on("/thresholds", handleGetThresholds); // Получить пороги
  server.on("/setthresholds", handleSetThresholds); // Установить пороги
  
  server.begin();

    // Проверка свободной памяти
  DEBUG_PRINT("Free heap: ");
  DEBUG_PRINTLN(ESP.getFreeHeap());
    
  if (ESP.getFreeHeap() < 10000) {
      DEBUG_PRINTLN("⚠️  Warning: Low memory!");
  }

  DEBUG_PRINTLN("✅ HTTP server started");
  DEBUG_PRINTLN("✅ Setup completed");
}


// ====================== LOOP ======================
void loop() {
    // Обработка веб-сервера
    server.handleClient();
    
    // Управление MQTT соединением
    static unsigned long lastMqttCheck = 0;
    if (millis() - lastMqttCheck > 30000) { // Проверка каждые 30 секунд
        if (!mqttClient.connected()) {
            DEBUG_PRINTLN("\n🔌 MQTT disconnected, reconnecting...");
            discoveryPublished = false; // Сброс для повторной отправки Discovery
            connectToMqtt();
        } else {
            // Периодический "ping" для поддержания соединения
            mqttClient.publish("home/meter/heartbeat", "alive", false);
        }
        lastMqttCheck = millis();
    }
    
    // Обработка MQTT
    mqttClient.loop();
    
    // Автоматическое размыкание кнопок
    handleButtonRelease();

    checkSystemHealth(); // Проверка здоровья системы
    
    // Отправка Discovery после подключения
    if (mqttClient.connected() && !discoveryPublished) {
        static unsigned long discoveryDelayStart = 0;
        
        if (discoveryDelayStart == 0) {
            discoveryDelayStart = millis();
        }
        
        // Ждем 3 секунды после подключения перед отправкой Discovery
        if (millis() - discoveryDelayStart > 3000) {
            publishHomeAssistantDiscovery();
            discoveryDelayStart = 0; // Сброс для следующего цикла
        }
    }
    
    // Чтение данных с камеры и публикация
    static unsigned long lastCameraRead = 0;
    if (millis() - lastCameraRead > 2000) { // Каждые 2 секунды
        if (mqttClient.connected() && discoveryPublished) {
            String result = readDisplay();
            DEBUG_PRINT("📸 Camera read: ");
            DEBUG_PRINTLN(result);
            
            // Парсим и публикуем данные
            publishMeterData(result);
        }
        lastCameraRead = millis();
    }
}

// Возвращает текущую таблицу segPos и topLEDs в JSON
void handleGetLayout() {
  DynamicJsonDocument doc(2048);
  JsonArray segs = doc.createNestedArray("segPos");
  for (int d = 0; d < DIGITS; d++) {
    JsonArray segArr = segs.createNestedArray();
    for (int s = 0; s < SEGMENTS; s++) {
      JsonObject o = segArr.createNestedObject();
      o["x"] = segPos[d][s].x;
      o["y"] = segPos[d][s].y;
      o["w"] = segPos[d][s].w;
      o["h"] = segPos[d][s].h;
    }
  }

  JsonArray leds = doc.createNestedArray("topLEDs");
  int ledCount = sizeof(topLEDs) / sizeof(topLEDs[0]);
  for (int i = 0; i < ledCount; i++) {
    JsonObject o = leds.createNestedObject();
    o["x"] = topLEDs[i].x;
    o["y"] = topLEDs[i].y;
    o["w"] = topLEDs[i].w;
    o["h"] = topLEDs[i].h;
  }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// Устанавливает таблицу segPos и topLEDs (ожидает JSON в теле POST)
void handleSetLayout() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "Missing body");
    return;
  }

  String body = server.arg("plain");
  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }

  // Парсим segPos
  if (doc.containsKey("segPos")) {
    JsonArray segs = doc["segPos"].as<JsonArray>();
    int dcount = min((size_t)DIGITS, segs.size());
    for (int d = 0; d < dcount; d++) {
      JsonArray segArr = segs[d].as<JsonArray>();
      int scount = min((size_t)SEGMENTS, segArr.size());
      for (int s = 0; s < scount; s++) {
        JsonObject o = segArr[s].as<JsonObject>();
        if (o.containsKey("x")) segPos[d][s].x = o["x"].as<int>();
        if (o.containsKey("y")) segPos[d][s].y = o["y"].as<int>();
        if (o.containsKey("w")) segPos[d][s].w = o["w"].as<int>();
        if (o.containsKey("h")) segPos[d][s].h = o["h"].as<int>();
      }
    }
  }

  // Парсим topLEDs
  if (doc.containsKey("topLEDs")) {
    JsonArray leds = doc["topLEDs"].as<JsonArray>();
    int ledCount = min((size_t)(sizeof(topLEDs)/sizeof(topLEDs[0])), leds.size());
    for (int i = 0; i < ledCount; i++) {
      JsonObject o = leds[i].as<JsonObject>();
      if (o.containsKey("x")) topLEDs[i].x = o["x"].as<int>();
      if (o.containsKey("y")) topLEDs[i].y = o["y"].as<int>();
      if (o.containsKey("w")) topLEDs[i].w = o["w"].as<int>();
      if (o.containsKey("h")) topLEDs[i].h = o["h"].as<int>();
    }
  }

  server.send(200, "text/plain", "OK");
}

// Возвращает текущие пороги в JSON
void handleGetThresholds() {
  String json = "{";
  json += "\"seg\":" + String(threshSegment) + ",";
  json += "\"led\":" + String(threshLED);
  json += "}";
  server.send(200, "application/json", json);
}

// Устанавливает пороги через query-параметры seg и led
void handleSetThresholds() {
  bool changed = false;
  if (server.hasArg("seg")) {
    int v = server.arg("seg").toInt(); if (v >= 0) { threshSegment = v; changed = true; }
  }
  if (server.hasArg("led")) {
    int v = server.arg("led").toInt(); if (v >= 0) { threshLED = v; changed = true; }
  }

  if (changed) server.send(200, "text/plain", "OK"); else server.send(400, "text/plain", "Missing or invalid parameters");
}