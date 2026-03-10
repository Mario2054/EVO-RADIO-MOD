#ifndef DLNA_WEBUI_H
#define DLNA_WEBUI_H

#include "../core/options.h"

#ifdef USE_DLNA

#include <Arduino.h>
#include <ESPAsyncWebServer.h>

/**
 * DLNAWebUI - Interfejs webowy dla przeglądarki DLNA
 * 
 * Zapewnia kompletny interfejs do:
 * 1. Przeglądania kategorii DLNA (root containers)
 * 2. Przeglądania zawartości kategorii (items/subcontainers)
 * 3. Budowania i przełączania playlist DLNA
 * 
 * Użycie:
 *   DLNAWebUI dlnaUI;
 *   dlnaUI.begin(&server);
 */
class DLNAWebUI {
public:
    DLNAWebUI();
    
    /**
     * Inicjalizacja UI - rejestruje wszystkie endpointy
     * @param server Wskaźnik do AsyncWebServer
     */
    void begin(AsyncWebServer* server);
    
    /**
     * Loop - obecnie nieużywany, ale zachowany dla spójności z innymi modułami
     */
    void loop();
    
private:
    AsyncWebServer* _server;
    
    // Route handlers
    void handleRoot(AsyncWebServerRequest *request);
    void handleInit(AsyncWebServerRequest *request);
    void handleCategories(AsyncWebServerRequest *request);
    void handleList(AsyncWebServerRequest *request);
    void handleBuild(AsyncWebServerRequest *request);
    void handleSwitch(AsyncWebServerRequest *request);
    void handleStatus(AsyncWebServerRequest *request);
    void handleListResult(AsyncWebServerRequest *request);
    
    // HTML
    static const char DLNA_HTML[] PROGMEM;
};

#endif // USE_DLNA
#endif // DLNA_WEBUI_H
