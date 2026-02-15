#include "WebUIManager.h"
#include "SDPlayer/SDPlayerOLED.h"

// Definicja HTML dla głównego menu
const char WebUIManager::MAIN_MENU_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>ESP32 Radio - Main Menu</title>
<style>
  body{font-family:Arial,Helvetica,sans-serif;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);margin:0;padding:0;min-height:100vh;display:flex;align-items:center;justify-content:center}
  .container{max-width:500px;width:90%;background:#fff;border-radius:20px;padding:40px 30px;box-shadow:0 10px 40px rgba(0,0,0,.3);text-align:center}
  h1{margin:0 0 10px 0;color:#333;font-size:32px}
  .subtitle{color:#666;margin-bottom:30px;font-size:14px}
  .menu-item{background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);color:#fff;border:0;border-radius:12px;padding:20px;margin:12px 0;width:100%;font-size:18px;font-weight:600;cursor:pointer;transition:all .3s;box-shadow:0 4px 15px rgba(102,126,234,.4)}
  .menu-item:hover{transform:translateY(-3px);box-shadow:0 6px 20px rgba(102,126,234,.6)}
  .menu-item:active{transform:translateY(-1px)}
  .menu-item .icon{font-size:24px;margin-right:10px}
  .footer{margin-top:30px;color:#999;font-size:12px}
</style>
</head>
<body>
<div class="container">
  <h1>🎵 ESP32 Radio</h1>
  <div class="subtitle">Evolution v3.19</div>
  
  <button class="menu-item" onclick="location.href='/sdplayer'">
    <span class="icon">💾</span> SD Player Control
  </button>
  
  <button class="menu-item" onclick="location.href='/bt'">
    <span class="icon">📶</span> Bluetooth Settings
  </button>
  
  <div class="footer">
    ESP32-WROOM-32D | 2026
  </div>
</div>
</body>
</html>
)HTML";

WebUIManager::WebUIManager() 
    : _server(nullptr),
      _backCallback(nullptr) {
}

void WebUIManager::begin(AsyncWebServer* server, Audio* audioPtr, SDPlayerOLED* oledPtr, int btRxPin, int btTxPin, uint32_t btBaud) {
    _server = server;
    
    // NIE ustawiamy głównego menu - używamy istniejącej strony / z main.cpp
    // setupMainMenu();
    
    // Inicjalizacja SD Player UI z Audio i OLED
    _sdPlayer.begin(_server, audioPtr);
    if (oledPtr) {
        _sdPlayer.setOLED(oledPtr);
        oledPtr->begin(&_sdPlayer);  // Połącz OLED z playerem
        Serial.println("WebUIManager: OLED connected to SD Player");
    }
    _sdPlayer.setExitCallback([this]() {
        if (_backCallback) _backCallback();
    });
    
    // Inicjalizacja BT UI
    _btUI.begin(_server, btRxPin, btTxPin, btBaud);
    _btUI.setExitCallback([this]() {
        if (_backCallback) _backCallback();
    });
    
    Serial.println("WebUIManager initialized");
    Serial.println("  - SD Player: http://<IP>/sdplayer");
    Serial.println("  - Bluetooth: http://<IP>/bt");
}

void WebUIManager::loop() {
    // Obsługa komunikacji UART dla BT
    _btUI.loop();
}

void WebUIManager::setBackToMenuCallback(void (*callback)()) {
    _backCallback = callback;
}

void WebUIManager::setupMainMenu() {
    // Główna strona - menu wyboru
    _server->on("/", HTTP_GET, [this](AsyncWebServerRequest *request){
        this->handleMainMenu(request);
    });
}

void WebUIManager::handleMainMenu(AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", MAIN_MENU_HTML);
}
