# 🎵 SD Player - System Dwóch Trybów

## 📋 Przegląd

System obsługi SD Playera z możliwością wyboru między dwoma trybami:

### **Tryb 1: WebUI (Basic)** 🌐
- Podstawowy tryb z interfejsem webowym
- Przeglądanie plików przez przeglądarkę
- Kontrola przez WiFi
- Idealny dla użytkowania zdalnego

### **Tryb 2: Advanced (Full)** 🚀
- ✅ **Auto-play** - automatyczne przejście do kolejnego utworu
- ✅ **Smart Sort** - inteligentne sortowanie numerów (Track 1, Track 2...)
- ✅ **ID3 Tags** - wyświetlanie Artist, Title, Album
- ✅ **Audio Info** - Bitrate, SampleRate, BitsPerSample
- ✅ **Play Timer** - timer odtwarzania (mm:ss)
- ✅ **Polskie znaki** - pełna obsługa znaków diakrytycznych
- ✅ **Zapamiętywanie** - zapisywanie ostatniej pozycji na SD
- ✅ **Rozszerzone formaty** - .mp3, .wav, .flac, .aac, .m4a, .ogg, .wma, .aiff, .alac
- ✅ **Repeat modes** - OFF, ONE, ALL, FOLDER
- ✅ **Sortowanie** - Name, Numeric, Size, Date

---

## 📁 Struktura Plików

```
src/SDPlayer/
├── SDPlayerConfig.h          # Konfiguracja i enum trybów
├── SDPlayerManager.h/cpp     # Menadżer przełączania trybów
├── SDPlayerWebUI.h/cpp       # Tryb WebUI (istniejący)
├── SDPlayerAdvanced.h/cpp    # Tryb Advanced (nowy)
└── SDPlayerOLED.h/cpp        # Wyświetlacz (wspólny dla obu)
```

---

## 🔧 Integracja z main.cpp

### 1. **Include'y na górze pliku**

```cpp
#include "SDPlayer/SDPlayerConfig.h"
#include "SDPlayer/SDPlayerManager.h"
#include "SDPlayer/SDPlayerWebUI.h"
#include "SDPlayer/SDPlayerAdvanced.h"
#include "SDPlayer/SDPlayerOLED.h"
```

### 2. **Deklaracje globalne**

```cpp
// SD Player - obiekty obu trybów
SDPlayerWebUI sdPlayerWebUI;
SDPlayerAdvanced sdPlayerAdvanced;
SDPlayerManager sdPlayerManager;  // Menadżer trybów
SDPlayerOLED sdPlayerOLED(u8g2);

// Flaga czy SD Player odtwarza muzykę (wspólna dla obu trybów)
bool sdPlayerPlayingMusic = false;
bool sdPlayerScanRequested = false;
```

### 3. **Inicjalizacja w setup()**

```cpp
void setup() {
    // ... twoja istniejąca inicjalizacja ...
    
    // Inicjalizuj oba tryby
    sdPlayerWebUI.begin(&server, &audio);
    sdPlayerAdvanced.begin(&audio, &u8g2);
    
    // Inicjalizuj menadżer
    sdPlayerManager.begin(&audio, &u8g2);
    sdPlayerManager.setWebUI(&sdPlayerWebUI);
    sdPlayerManager.setAdvanced(&sdPlayerAdvanced);
    sdPlayerManager.setOLED(&sdPlayerOLED);
    
    // Wczytaj ostatni wybrany tryb z SD
    sdPlayerManager.loadSettings();
    
    // Inicjalizuj OLED
    sdPlayerOLED.begin(&sdPlayerWebUI);  // lub &sdPlayerAdvanced w zależności od trybu
    
    Serial.print("SD Player Mode: ");
    Serial.println(getSDPlayerModeName(sdPlayerManager.getMode()));
}
```

### 4. **Główna pętla loop()**

```cpp
void loop() {
    // ... twój istniejący kod ...
    
    // Obsługa SD Player
    sdPlayerManager.loop();
    
    // Obsługa OLED (jeśli aktywny)
    if (sdPlayerOLED.isActive()) {
        sdPlayerOLED.loop();
    }
    
    // ... reszta kodu ...
}
```

### 5. **Audio Callbacks - ZASTĄP ISTNIEJĄCE**

```cpp
// Zamień istniejące funkcje audio_xxx na:

void audio_info(const char *info) {
    Serial.print("audio_info: ");
    Serial.println(info);
    
    // Przekaż do SD Player Manager
    sdPlayerManager.onAudioInfo(info);
}

void audio_id3data(const char *info) {
    Serial.print("audio_id3data: ");
    Serial.println(info);
    
    sdPlayerManager.onAudioID3(info);
}

void audio_bitrate(const char *info) {
    Serial.print("audio_bitrate: ");
    Serial.println(info);
    
    sdPlayerManager.onAudioBitrate(info);
}

void audio_eof_mp3(const char *info) {
    Serial.println("audio_eof_mp3: Track ended");
    
    // WAŻNE: Przekaż EOF do managera - obsłuży auto-play
    sdPlayerManager.onAudioEOF(info);
}
```

### 6. **Obsługa pilota - dodaj przycisk zmiany trybu**

```cpp
void handleRemoteControl() {
    // ... twój istniejący kod ...
    
    // Nowy przycisk: zmiana trybu SD Player (np. długie przytrzymanie SRC)
    if (IRlongPressSRC) {  // Przykład
        IRlongPressSRC = false;
        sdPlayerManager.toggleMode();
        
        // Pokaż komunikat
        String msg = "SD Player: ";
        msg += getSDPlayerModeName(sdPlayerManager.getMode());
        showMessage(msg);
    }
}
```

---

## ⚙️ Menu Ustawień - Wybór Trybu

### Dodaj opcję w menu settings:

```cpp
void showSettingsMenu() {
    // ... istniejące opcje ...
    
    // Nowa opcja: SD Player Mode
    menuItems.push_back("SD Player Mode");
}

void handleSettingsSelection(int index) {
    if (index == MENU_SDPLAYER_MODE) {
        // Przełącz tryb
        sdPlayerManager.toggleMode();
        
        // Wyświetl aktualny tryb
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(10, 20, "SD Player Mode:");
        u8g2.setFont(u8g2_font_ncenB14_tr);
        u8g2.drawStr(10, 40, getSDPlayerModeName(sdPlayerManager.getMode()));
        u8g2.sendBuffer();
        delay(2000);
    }
}
```

### Lub stwórz dedykowane submenu:

```cpp
void showSDPlayerModeMenu() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(10, 10, "SD Player Mode:");
    
    // Opcja 1: WebUI
    if (sdPlayerManager.getMode() == SD_MODE_WEBUI) {
        u8g2.drawStr(10, 25, "> WebUI (Basic)");  // Zaznaczone
        u8g2.drawStr(10, 40, "  Advanced (Full)");
    } else {
        u8g2.drawStr(10, 25, "  WebUI (Basic)");
        u8g2.drawStr(10, 40, "> Advanced (Full)"); // Zaznaczone
    }
    
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(10, 55, getSDPlayerModeDescription(sdPlayerManager.getMode()));
    u8g2.sendBuffer();
}
```

---

## 🎮 Kontrola z Pilota/Enkodera

### Przypisanie przycisków dla SD Player:

```cpp
// W trybie SD Player:
void handleSDPlayerControls() {
    if (IRupArrow) {
        IRupArrow = false;
        sdPlayerOLED.onRemoteUp();
    }
    
    if (IRdownArrow) {
        IRdownArrow = false;
        sdPlayerOLED.onRemoteDown();
    }
    
    if (IRokButton) {
        IRokButton = false;
        sdPlayerOLED.onRemoteOK();  // Play/Select
    }
    
    if (IRleftArrow) {
        IRleftArrow = false;
        sdPlayerManager.previous();
    }
    
    if (IRrightArrow) {
        IRrightArrow = false;
        sdPlayerManager.next();
    }
    
    if (IRpauseResume) {
        IRpauseResume = false;
        sdPlayerManager.togglePause();
    }
    
    if (IRstop) {
        IRstop = false;
        sdPlayerManager.stop();
    }
    
    // Zmiana trybu - długie przytrzymanie MODE
    if (IRlongPressMode) {
        IRlongPressMode = false;
        sdPlayerManager.toggleMode();
        showMessage(getSDPlayerModeName(sdPlayerManager.getMode()));
    }
}
```

---

## 💾 Zapisywanie Ustawień

System automatycznie zapisuje na SD kartę:

### Pliki konfiguracyjne:
- `/sdplayer_mode.cfg` - wybrany tryb (WebUI/Advanced)
- `/sdplayer_adv.cfg` - ustawienia trybu Advanced:
  - Ostatni katalog
  - Ostatni indeks pliku
  - Tryb sortowania
  - Tryb powtarzania
  - Auto-play włączony/wyłączony

### Zapisz ręcznie:
```cpp
sdPlayerManager.saveSettings();
```

### Wczytaj ręcznie:
```cpp
sdPlayerManager.loadSettings();
```

---

## 🧪 Testowanie

### 1. **Test przełączania trybów:**
```cpp
// W setup() lub po naciśnięciu przycisku:
Serial.println("=== Test SD Player Modes ===");

// Tryb WebUI
sdPlayerManager.setMode(SD_MODE_WEBUI);
Serial.print("Mode: ");
Serial.println(getSDPlayerModeName(sdPlayerManager.getMode()));

delay(2000);

// Tryb Advanced
sdPlayerManager.setMode(SD_MODE_ADVANCED);
Serial.print("Mode: ");
Serial.println(getSDPlayerModeName(sdPlayerManager.getMode()));
```

### 2. **Test auto-play (Advanced mode):**
```cpp
sdPlayerManager.setMode(SD_MODE_ADVANCED);
sdPlayerManager.activate();
sdPlayerAdvanced.setAutoPlay(true);
sdPlayerAdvanced.play(0);  // Uruchomi pierwszy utwór i automatycznie przejdzie do kolejnych
```

### 3. **Test ID3 tags:**
```cpp
// Odtwórz plik MP3 z tagami ID3
sdPlayerManager.setMode(SD_MODE_ADVANCED);
sdPlayerAdvanced.playFile("/MUZYKA/song.mp3");

// Po chwili sprawdź info:
const auto& info = sdPlayerAdvanced.getTrackInfo();
Serial.print("Artist: ");
Serial.println(info.artist);
Serial.print("Title: ");
Serial.println(info.title);
Serial.print("Bitrate: ");
Serial.println(info.bitrate);
```

---

## 📊 Porównanie Trybów

| Funkcja | WebUI | Advanced |
|---------|-------|----------|
| Interfejs web | ✅ | ❌ |
| Auto-play kolejnych | ❌ | ✅ |
| Inteligentne sortowanie | ❌ | ✅ |
| Tagi ID3 | ❌ | ✅ |
| Parametry audio | ❌ | ✅ |
| Timer odtwarzania | ❌ | ✅ |
| Polskie znaki | ⚠️ | ✅ |
| Zapisywanie pozycji | ❌ | ✅ |
| Tryby powtarzania | ❌ | ✅ |
| Sortowanie zaawansowane | ❌ | ✅ |

---

## 🐛 Rozwiązywanie Problemów

### Problem: Nie przełącza się między trybami
**Rozwiązanie:** Sprawdź czy obie instancje są poprawnie zainicjalizowane:
```cpp
if (!sdPlayerWebUI || !sdPlayerAdvanced) {
    Serial.println("ERROR: SD Player instances not initialized!");
}
```

### Problem: Auto-play nie działa
**Rozwiązanie:** Upewnij się że audio_eof_mp3() przekierowuje do managera:
```cpp
void audio_eof_mp3(const char *info) {
    sdPlayerManager.onAudioEOF(info);  // <-- To musi być!
}
```

### Problem: Brak tagów ID3
**Rozwiązanie:** Sprawdź czy plik ma tagi:
```cpp
const auto& info = sdPlayerAdvanced.getTrackInfo();
if (!info.hasID3) {
    Serial.println("File has no ID3 tags - using filename");
}
```

### Problem: Crashuje przy zmianie trybu
**Rozwiązanie:** Zawsze zatrzymuj odtwarzanie przed zmianą:
```cpp
sdPlayerManager.stop();
delay(100);
sdPlayerManager.setMode(newMode);
```

---

## 📝 Przykład Pełnej Integracji

Plik: `EXAMPLE_INTEGRATION.cpp`

```cpp
#include <Arduino.h>
#include "Audio.h"
#include "SDPlayer/SDPlayerManager.h"
#include "SDPlayer/SDPlayerWebUI.h"
#include "SDPlayer/SDPlayerAdvanced.h"
#include "SDPlayer/SDPlayerOLED.h"

// Globals
Audio audio;
U8G2_SSD1322_NHD_256X64_F_4W_SW_SPI u8g2(...);
AsyncWebServer server(80);

SDPlayerWebUI sdPlayerWebUI;
SDPlayerAdvanced sdPlayerAdvanced;
SDPlayerManager sdPlayerManager;
SDPlayerOLED sdPlayerOLED(u8g2);

bool sdPlayerPlayingMusic = false;

void setup() {
    Serial.begin(115200);
    
    // Init SD, WiFi, etc...
    
    // Init SD Player
    sdPlayerWebUI.begin(&server, &audio);
    sdPlayerAdvanced.begin(&audio, &u8g2);
    
    sdPlayerManager.begin(&audio, &u8g2);
    sdPlayerManager.setWebUI(&sdPlayerWebUI);
    sdPlayerManager.setAdvanced(&sdPlayerAdvanced);
    sdPlayerManager.setOLED(&sdPlayerOLED);
    sdPlayerManager.loadSettings();
    
    sdPlayerOLED.begin(&sdPlayerWebUI);
    
    Serial.println("SD Player ready!");
}

void loop() {
    audio.loop();
    sdPlayerManager.loop();
    
    if (sdPlayerOLED.isActive()) {
        sdPlayerOLED.loop();
    }
}

// Audio callbacks
void audio_info(const char *info) {
    sdPlayerManager.onAudioInfo(info);
}

void audio_id3data(const char *info) {
    sdPlayerManager.onAudioID3(info);
}

void audio_bitrate(const char *info) {
    sdPlayerManager.onAudioBitrate(info);
}

void audio_eof_mp3(const char *info) {
    sdPlayerManager.onAudioEOF(info);
}
```

---

## ✅ Checklist Integracji

- [ ] Dodano wszystkie pliki do `src/SDPlayer/`
- [ ] Dodano include'y w main.cpp
- [ ] Utworzono obiekty globalne (WebUI, Advanced, Manager, OLED)
- [ ] Zainicjalizowano w setup()
- [ ] Dodano wywołania w loop()
- [ ] Zaktualizowano audio callbacks (audio_xxx)
- [ ] Dodano opcję w menu ustawień
- [ ] Dodano obsługę pilota/enkodera
- [ ] Przetestowano przełączanie trybów
- [ ] Przetestowano auto-play w Advanced
- [ ] Zapisano/wczytano ustawienia z SD

---

## 🎉 Gotowe!

System jest w pełni funkcjonalny. Możesz teraz:
1. Przełączać między trybami przez menu lub przycisk
2. Cieszyć się auto-play w trybie Advanced
3. Widzieć tagi ID3 i parametry audio
4. Korzystać z inteligentnego sortowania
5. Wszystko jest zapisywane automatycznie

Powodzenia! 🚀
