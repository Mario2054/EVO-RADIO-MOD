#ifndef WEBUI_MANAGER_H
#define WEBUI_MANAGER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "SDPlayer/SDPlayerWebUI.h"
#include "bt/BTWebUI.h"

// Forward declarations
class Audio;
class SDPlayerOLED;

/**
 * WebUIManager - Menedżer interfejsów webowych dla ESP32 Radio
 * 
 * Zarządza dwoma głównymi interfejsami:
 * 1. SD Player - Sterowanie odtwarzaczem plików z karty SD
 * 2. BT UART - Ustawienia i sterowanie modułem Bluetooth
 * 
 * Użycie:
 * 
 *   WebUIManager webUI;
 *   AsyncWebServer server(80);
 *   
 *   void setup() {
 *     // Inicjalizacja WiFi...
 *     
 *     webUI.begin(&server);
 *     server.begin();
 *   }
 *   
 *   void loop() {
 *     webUI.loop();  // Obsługa UART BT
 *   }
 */

class WebUIManager {
public:
    WebUIManager();
    
    /**
     * Inicjalizacja menedżera UI
     * @param server Wskaźnik do AsyncWebServer
     * @param audioPtr Wskaźnik do globalnego obiektu Audio
     * @param oledPtr Wskaźnik do SDPlayerOLED (opcjonalnie)
     * @param btRxPin Pin RX dla UART Bluetooth (domyślnie 19)
     * @param btTxPin Pin TX dla UART Bluetooth (domyślnie 20)
     * @param btBaud Prędkość UART dla Bluetooth (domyślnie 115200)
     */
    void begin(AsyncWebServer* server, Audio* audioPtr, SDPlayerOLED* oledPtr = nullptr, int btRxPin = 19, int btTxPin = 20, uint32_t btBaud = 115200);
    
    /**
     * Obsługa pętli - wywołuj w loop()
     * Obsługuje komunikację UART z modułem BT
     */
    void loop();
    
    /**
     * Ustaw callback wywoływany gdy użytkownik kliknie "Back to Menu"
     */
    void setBackToMenuCallback(void (*callback)());
    
    /**
     * Dostęp do poszczególnych modułów
     */
    SDPlayerWebUI& getSDPlayer() { return _sdPlayer; }
    BTWebUI& getBTUI() { return _btUI; }
    
private:
    SDPlayerWebUI _sdPlayer;
    BTWebUI _btUI;
    AsyncWebServer* _server;
    void (*_backCallback)();
    
    void handleMainMenu(AsyncWebServerRequest *request);
    void setupMainMenu();
    
    static const char MAIN_MENU_HTML[] PROGMEM;
};
#endif
