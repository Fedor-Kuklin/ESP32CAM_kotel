#ifndef DEBUGLOGGER_H
#define DEBUGLOGGER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>

namespace DebugLogger {
  void beginSerial(unsigned long baud = 115200);
  void beginAsyncWebSocket(AsyncWebServer &server, const char *path = "/ws");
  void print(const String &s);
  void println(const String &s);
  void printf(const char *fmt, ...);
  void setEnabled(bool en);
  void pauseWeb();
  void resumeWeb();
  bool isWebPaused();
  bool isEnabled();
}

#endif
