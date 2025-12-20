#ifndef OTA_UPDATER_H
#define OTA_UPDATER_H

#include <ESPAsyncWebServer.h>


// Инициализация OTA: регистрирует веб-эндпойнты на переданном AsyncWebServer
void OTAUpdater_begin(AsyncWebServer &server);

#endif
