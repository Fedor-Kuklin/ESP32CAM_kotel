#include "DebugLogger.h"
#include <AsyncWebSocket.h>
#include <stdarg.h>

static bool g_enabled = true;
static bool g_serial = false;
static AsyncWebSocket *g_ws = nullptr;
static volatile bool g_ws_paused = false;

void DebugLogger::setEnabled(bool en) { g_enabled = en; }

bool DebugLogger::isEnabled() { return g_enabled; }

void DebugLogger::beginSerial(unsigned long baud) {
  Serial.begin(baud);
  g_serial = true;
}

void DebugLogger::beginAsyncWebSocket(AsyncWebServer &server, const char *path) {
  if (g_ws) return; // already started
  g_ws = new AsyncWebSocket(path);
  server.addHandler(g_ws);
}

void DebugLogger::pauseWeb() { g_ws_paused = true; }
void DebugLogger::resumeWeb() { g_ws_paused = false; }
bool DebugLogger::isWebPaused() { return g_ws_paused; }

static void emit(const String &s) {
  if (!g_enabled) return;
  if (g_serial) Serial.print(s);
  if (g_ws && !g_ws_paused) g_ws->textAll(s);
}

void DebugLogger::print(const String &s) { emit(s); }

void DebugLogger::println(const String &s) {
  emit(s + "\r\n");
}

void DebugLogger::printf(const char *fmt, ...) {
  if (!g_enabled) return;
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  String s = String(buf);
  if (g_serial) Serial.print(s);
  if (g_ws) g_ws->textAll(s);
}
