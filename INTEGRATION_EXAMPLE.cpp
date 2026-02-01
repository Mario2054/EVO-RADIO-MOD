/**
 * PRZYKŁAD INTEGRACJI WEBUI W MAIN.CPP
 * =====================================
 * 
 * Ten plik pokazuje jak zintegrować WebUIManager w istniejącym projekcie.
 * 
 * KROK 1: Dodaj includes na początku main.cpp
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "WebUIManager.h"
// #include "Audio.h"  // Jeśli używasz biblioteki Audio

/**
 * KROK 2: Zdefiniuj globalne obiekty
 */

// WiFi credentials
const char* WIFI_SSID = "YourWiFiNetwork";
const char* WIFI_PASSWORD = "YourPassword";

// Web server
AsyncWebServer server(80);
WebUIManager webUI;

// Audio (opcjonalnie)
// Audio audio;

/**
 * KROK 3: Dodaj funkcję setupWiFi()
 */

void setupWiFi() {
    Serial.println("\n=== WiFi Setup ===");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✓ WiFi Connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal Strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
    } else {
        Serial.println("\n✗ WiFi Connection Failed!");
        Serial.println("Creating AP mode...");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("ESP32-Radio", "12345678");
        Serial.print("AP IP: ");
        Serial.println(WiFi.softAPIP());
    }
}

/**
 * KROK 4: Zmodyfikuj setup()
 */

void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════╗");
    Serial.println("║   ESP32 Radio Evolution v3.19     ║");
    Serial.println("║        WebUI Edition              ║");
    Serial.println("╚════════════════════════════════════╝");
    
    // ===== INICJALIZACJA AUDIO (jeśli używasz) =====
    /*
    Serial.println("\n=== Audio Setup ===");
    audio.setPinout(
        26,  // BCLK
        25,  // LRC/WS
        22   // DOUT
    );
    audio.setVolume(7);
    Serial.println("✓ Audio initialized");
    */
    
    // ===== INICJALIZACJA SD (jeśli nie jest w SDPlayerWebUI) =====
    /*
    Serial.println("\n=== SD Card Setup ===");
    if (!SD.begin()) {
        Serial.println("✗ SD Card Mount Failed");
    } else {
        Serial.println("✓ SD Card Mounted");
    }
    */
    
    // ===== INICJALIZACJA WiFi =====
    setupWiFi();
    
    // ===== INICJALIZACJA WEBUI =====
    Serial.println("\n=== WebUI Setup ===");
    
    // Parametry: server, BT_RX_pin, BT_TX_pin, BT_baud
    webUI.begin(&server, 19, 20, 115200);
    
    // Callback dla przycisku "Back to Menu"
    webUI.setBackToMenuCallback([]() {
        Serial.println("→ User returned to main menu");
    });
    
    Serial.println("✓ WebUI initialized");
    
    // ===== START SERWERA =====
    server.begin();
    Serial.println("✓ Web Server started");
    
    // ===== PODSUMOWANIE =====
    Serial.println("\n╔════════════════════════════════════╗");
    Serial.println("║        SYSTEM READY!              ║");
    Serial.println("╚════════════════════════════════════╝");
    Serial.println("\nAccess WebUI at:");
    Serial.print("  Main Menu:  http://");
    Serial.println(WiFi.localIP());
    Serial.print("  SD Player:  http://");
    Serial.print(WiFi.localIP());
    Serial.println("/sdplayer");
    Serial.print("  Bluetooth:  http://");
    Serial.print(WiFi.localIP());
    Serial.println("/bt");
    Serial.println("\n");
}

/**
 * KROK 5: Zmodyfikuj loop()
 */

void loop() {
    // ===== OBSŁUGA WEBUI (wymagane!) =====
    webUI.loop();  // Obsługuje UART dla BT
    
    // ===== OBSŁUGA AUDIO (jeśli używasz) =====
    // audio.loop();
    
    // ===== SYNCHRONIZACJA VOLUME =====
    /*
    static int lastVol = -1;
    int currentVol = webUI.getSDPlayer().getVolume();
    if (currentVol != lastVol) {
        audio.setVolume(currentVol);
        lastVol = currentVol;
        Serial.printf("Volume changed: %d\n", currentVol);
    }
    */
    
    // ===== INTEGRACJA Z ISTNIEJĄCYM KODEM =====
    
    // Przykład: Automatyczne odtwarzanie z SD Player
    /*
    if (webUI.getSDPlayer().isPlaying()) {
        String currentFile = webUI.getSDPlayer().getCurrentFile();
        if (currentFile != "None" && !audio.isRunning()) {
            audio.connecttoSD(currentFile.c_str());
        }
    }
    */
    
    // Przykład: Reakcja na zmiany BT
    /*
    static String lastBTCmd = "";
    String btResp = webUI.getBTUI().getLastResponse();
    if (btResp != lastBTCmd && btResp.length() > 0) {
        Serial.println("BT: " + btResp);
        lastBTCmd = btResp;
    }
    */
    
    // Twój istniejący kod loop()...
    
    delay(1);
}

/**
 * OPCJONALNE: Funkcje pomocnicze
 */

// Wyślij komendę do modułu BT
void sendBTCommand(const String& cmd) {
    webUI.getBTUI().sendCommand(cmd);
}

// Odtwórz plik z SD
void playSDFile(const String& path) {
    webUI.getSDPlayer().playFile(path);
}

// Ustaw głośność SD Player
void setSDVolume(int vol) {
    webUI.getSDPlayer().setVolume(vol);
}

/**
 * NOTATKI DOTYCZĄCE INTEGRACJI:
 * ==============================
 * 
 * 1. WebUI działa niezależnie od Audio library - musisz zsynchronizować:
 *    - Volume
 *    - Aktualnie odtwarzany plik
 *    - Status odtwarzania (play/pause/stop)
 * 
 * 2. BT UART komunikuje się przez Serial2 (GPIO16/17):
 *    - Automatycznie parsuje odpowiedzi
 *    - Loguje do konsoli webowej
 *    - Możesz wysyłać własne komendy
 * 
 * 3. Pamięć:
 *    - HTML jest w PROGMEM (Flash)
 *    - JSON bufory są dynamiczne
 *    - Logi BT są ograniczone do 2000 znaków
 * 
 * 4. Wydajność:
 *    - Async WebServer nie blokuje loop()
 *    - UART jest sprawdzany co wywołanie loop()
 *    - Polling UI: SD Player (1s), BT (1.2s)
 * 
 * 5. Bezpieczeństwo:
 *    - Brak autoryzacji (dodaj jeśli potrzeba)
 *    - Używaj WPA2 dla WiFi
 *    - Rozważ mDNS dla łatwego dostępu
 */
