# WebUI dla ESP32 Radio - Dokumentacja

## 📁 Struktura plików

```
src/
├── SDPlayer/
│   ├── SDPlayerWebUI.h      # Nagłówek SD Player WebUI
│   └── SDPlayerWebUI.cpp    # Implementacja SD Player WebUI
├── BTWebUI/
│   ├── BTWebUI.h            # Nagłówek Bluetooth WebUI
│   └── BTWebUI.cpp          # Implementacja Bluetooth WebUI
├── WebUIManager.h           # Główny menedżer UI
├── WebUIManager.cpp         # Implementacja menedżera
└── main.cpp                 # Główny program
```

## 🚀 Funkcjonalności

### 1. SD Player WebUI (`/sdplayer`)
- ✅ Przeglądanie plików i katalogów na karcie SD
- ✅ Odtwarzanie plików audio (MP3, WAV, FLAC, AAC, M4A, OGG)
- ✅ Kontrola odtwarzania: Play, Pause, Stop, Next, Previous
- ✅ Regulacja głośności (0-21)
- ✅ Nawigacja po katalogach
- ✅ Automatyczne odświeżanie listy plików (1s)

### 2. Bluetooth WebUI (`/bt`)
- ✅ Kontrola trybu BT: OFF, RX (odbiornik), TX (nadajnik), AUTO
- ✅ Zarządzanie połączeniami Bluetooth
- ✅ Skanowanie urządzeń BT
- ✅ Regulacja głośności (0-100)
- ✅ Wzmocnienie sygnału BOOST (100-400%)
- ✅ Konsola diagnostyczna UART
- ✅ Wysyłanie komend do modułu BT
- ✅ Usuwanie sparowanych urządzeń
- ✅ Zapis ustawień

### 3. Menu główne (`/`)
- ✅ Elegancki interfejs wyboru modułów
- ✅ Responsywny design

## 📦 Wymagane biblioteki

W pliku `platformio.ini` dodaj:

```ini
lib_deps = 
    me-no-dev/ESP Async WebServer@^1.2.3
    me-no-dev/AsyncTCP@^1.1.1
    bblanchon/ArduinoJson@^6.21.3
```

## 🔧 Integracja z main.cpp

### Podstawowa integracja:

```cpp
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "WebUIManager.h"

// WiFi credentials
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";

// Web server i UI manager
AsyncWebServer server(80);
WebUIManager webUI;

void setup() {
    Serial.begin(115200);
    
    // Połącz z WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    
    // Inicjalizacja WebUI
    // Parametry: server, BT_RX_pin, BT_TX_pin, BT_baud
    webUI.begin(&server, 19, 20, 115200);
    
    // Opcjonalnie: callback dla przycisku "Back to Menu"
    webUI.setBackToMenuCallback([]() {
        Serial.println("User returned to main menu");
    });
    
    // Uruchom serwer
    server.begin();
    Serial.println("Web server started!");
}

void loop() {
    // Obsługa UART dla BT
    webUI.loop();
    
    // Twój kod...
}
```

### Zaawansowana integracja z Audio library:

```cpp
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "WebUIManager.h"
#include "Audio.h"  // https://github.com/schreibfaul1/ESP32-audioI2S

// Audio setup
Audio audio;
AsyncWebServer server(80);
WebUIManager webUI;

void setup() {
    Serial.begin(115200);
    
    // Inicjalizacja Audio I2S
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(7);
    
    // WiFi connection...
    
    // Inicjalizacja WebUI
    webUI.begin(&server, 19, 20, 115200);
    
    // Synchronizacja kontroli głośności
    webUI.getSDPlayer().setVolume(audio.getVolume());
    
    server.begin();
}

void loop() {
    audio.loop();
    webUI.loop();
    
    // Synchronizuj volume z UI do Audio
    static int lastVol = -1;
    int currentVol = webUI.getSDPlayer().getVolume();
    if (currentVol != lastVol) {
        audio.setVolume(currentVol);
        lastVol = currentVol;
    }
}
```

## 🔌 Konfiguracja pinów

### Bluetooth UART (domyślnie):
- **RX**: GPIO 19
- **TX**: GPIO 20
- **Baud**: 115200

Możesz zmienić w wywołaniu `webUI.begin()`:
```cpp
webUI.begin(&server, 
    19,      // RX pin
    20,      // TX pin  
    115200   // Baud rate
);
```

### SD Card:
Domyślnie używa standardowych pinów SD dla ESP32.
Konfiguracja w kodzie SD Player.

## 📡 Endpointy API

### SD Player API

| Endpoint | Metoda | Parametry | Opis |
|----------|--------|-----------|------|
| `/sdplayer` | GET | - | Główna strona SD Player |
| `/sdplayer/api/list` | GET | - | Lista plików (JSON) |
| `/sdplayer/api/play` | POST | `i` (index) | Odtwórz plik o indeksie |
| `/sdplayer/api/playSelected` | POST | - | Odtwórz zaznaczony plik |
| `/sdplayer/api/pause` | POST | - | Pauza/Wznów |
| `/sdplayer/api/stop` | POST | - | Stop |
| `/sdplayer/api/next` | POST | - | Następny |
| `/sdplayer/api/prev` | POST | - | Poprzedni |
| `/sdplayer/api/vol` | POST | `v` (0-21) | Ustaw głośność |
| `/sdplayer/api/cd` | GET | `p` (path) | Zmień katalog |
| `/sdplayer/api/up` | POST | - | Katalog wyżej |
| `/sdplayer/api/back` | POST | - | Powrót do menu |

### Bluetooth API

| Endpoint | Metoda | Parametry | Opis |
|----------|--------|-----------|------|
| `/bt` | GET | - | Główna strona BT |
| `/bt/api/state` | GET | - | Status BT (JSON) |
| `/bt/api/log` | GET | - | Log konsoli (text) |
| `/bt/api/mode` | POST | `m` (OFF/RX/TX/AUTO) | Ustaw tryb |
| `/bt/api/vol` | POST | `v` (0-100) | Ustaw głośność |
| `/bt/api/boost` | POST | `b` (100-400) | Ustaw boost |
| `/bt/api/scan` | POST | - | Skanuj urządzenia |
| `/bt/api/disconnect` | POST | - | Rozłącz |
| `/bt/api/delall` | POST | - | Usuń sparowane |
| `/bt/api/save` | POST | - | Zapisz ustawienia |
| `/bt/api/cmd` | POST | `c` (command) | Wyślij komendę UART |
| `/bt/api/back` | POST | - | Powrót do menu |

## 🎨 Responsywność

Wszystkie interfejsy są w pełni responsywne i działają na:
- 📱 Telefonach (iOS/Android)
- 💻 Tabletach
- 🖥️ Komputerach

## 🐛 Debugowanie

### Serial Monitor:
```
WebUIManager initialized
  - Main Menu: http://<IP>/
  - SD Player: http://<IP>/sdplayer
  - Bluetooth: http://<IP>/bt
SD Card initialized.
Scanned 15 items in /music
BT UART initialized on RX:16 TX:17
BT CMD: STATUS?
```

### Dostęp do modułów:
```cpp
// Dostęp do SD Player
SDPlayerWebUI& sdp = webUI.getSDPlayer();
Serial.println(sdp.getCurrentDirectory());
Serial.println(sdp.getCurrentFile());

// Dostęp do BT UI
BTWebUI& bt = webUI.getBTUI();
bt.sendCommand("PING");
Serial.println(bt.getLastResponse());
```

## 📝 Komendy BT UART

Moduł BT obsługuje komendy zgodne z kodem EVO-BT-TX:

```
HELP              - Lista komend
PING              - Test połączenia (odpowiedź: PONG)
GET / STATUS?     - Pobranie statusu
BT ON / BT OFF    - Włącz/wyłącz BT
MODE OFF|TX|RX|AUTO - Ustaw tryb
VOL 0..100        - Ustaw głośność
BOOST 100..400    - Ustaw wzmocnienie
SCAN              - Skanuj urządzenia
CONNECT <idx|MAC> - Połącz z urządzeniem
DISCONNECT        - Rozłącz
PAIRED?           - Lista sparowanych
DELPAIRED ALL     - Usuń wszystkie sparowane
SAVE              - Zapisz ustawienia
DBG 0|1           - Debug włącz/wyłącz
HARDRESET         - Restart ESP32
```

## 🎯 TODO / Rozszerzenia

- [ ] Integracja z biblioteką Audio dla rzeczywistego odtwarzania
- [ ] Playlist manager
- [ ] Equalizer graficzny
- [ ] Streamer internetowy (radio)
- [ ] OTA Updates
- [ ] mDNS (dostęp przez radio.local)
- [ ] Harmonogram odtwarzania
- [ ] Alarmy/Timer

## 📄 Licencja

Część projektu ESP32 Radio Evolution v3.19

---
**Autor**: ESP32 Radio Team  
**Data**: 2026  
**Wersja**: 3.19
