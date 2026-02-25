# Debugowanie SD Player Advanced - Brak panelu na OLED

## Problem
Po aktywacji SD Playera nie wyświetla się panel Advanced z ikonami i ID3 tags.

## Kroki diagnostyczne (Serial Monitor 115200 baud)

### 1. Sprawdź wiadomości przy starcie urządzenia

Po resecie ESP32 powinieneś zobaczyć:

```
SDPlayerManager: Initialized
[SDPlayerManager] Display and Audio passed to Advanced
[SDPlayerManager] Initialized in ADVANCED mode
```

**Jeśli brakuje któregoś komunikatu:**
- Brak "Display and Audio passed" → Problem z przekazywaniem U8G2 display
- Sprawdź czy `display` nie jest nullptr w main.cpp

---

### 2. Triple-click enkoder aby aktywować

Kliknij **3 razy szybko** przycisk enkodera (w ciągu 600ms).

**Powinieneś zobaczyć:**
```
[SDPlayerManager] Aktywny w trybie: ADVANCED
Advanced mode - Full-featured player with ID3, auto-play, timer
SDPlayerAdvanced: Activated
SDPlayerAdvanced: Scanned X items
```

**Jeśli NIE widzisz tych komunikatów:**
- Triple-click nie został wykryty
- Spróbuj aktywować przez pilot IR (przycisk 9)
- Sprawdź czy enkoder fizycznie działa

---

### 3. Sprawdź czy renderOLED() jest wywoływane

Po aktywacji SD Playera **co 5 sekund** powinieneś widzieć:

```
[SDPlayerAdvanced] Rendering OLED...
```

**a) Jeśli widzisz:**
```
[SDPlayerAdvanced] ERROR: _display is NULL!
```
**Rozwiązanie:** Problem z przekazaniem U8G2 display. Sprawdź:
- Czy `g_sdPlayerManager->begin(&audio, display)` ma prawidłowy wskaźnik display
- Czy display nie jest nullptr w main.cpp przy inicjalizacji

**b) Jeśli widzisz:**
```
[SDPlayerAdvanced] WARNING: Not active
```
**Rozwiązanie:** SDPlayerAdvanced nie został aktywowany. Sprawdź:
- Czy triple-click działa?
- Czy `g_sdPlayerManager->activate()` jest wywoływane?

**c) Jeśli NIE widzisz ŻADNEGO komunikatu:**
**Rozwiązanie:** 
- `loop()` nie jest wywoływane
- Sprawdź czy `SDPlayerOLED_loop()` jest wywołane w main loop()
- Sprawdź czy `g_sdPlayerManager->loop()` jest wywoływane

---

### 4. Sprawdź czy stary system nie blokuje nowego

Szukaj w kodzie wywołań:

```cpp
g_sdPlayerOLED->loop()  // STARY system - może blokować rendering
```

**Rozwiązanie:** SDPlayerOLED_loop() powinien mieć EARLY RETURN:

```cpp
void SDPlayerOLED_loop() {
  // PRIORYTET: Użyj SDPlayerManager (dual-mode system)
  if (g_sdPlayerManager && g_sdPlayerManager->isActive()) {
    g_sdPlayerManager->loop();
    return;  // ← KRYTYCZNE!
  }
  
  // LEGACY: Stara obsługa...
```

---

## Kompletny przykład komunikatów SUCCESS

```
========== STARTUP ==========
SDPlayerManager: Initialized
[SDPlayerManager] Display and Audio passed to Advanced
[SDPlayerManager] Initialized in ADVANCED mode

========== AKTYWACJA (triple-click) ==========
[SDPlayerManager] Aktywny w trybie: ADVANCED
Advanced mode - Full-featured player with ID3, auto-play, timer
SDPlayerAdvanced: Activated
SDPlayerAdvanced: Scanned 42 items

========== RENDERING (co 5s) ==========
[SDPlayerAdvanced] Rendering OLED...
[SDPlayerAdvanced] Rendering OLED...
[SDPlayerAdvanced] Rendering OLED...

========== ODTWARZANIE PLIKU ==========
SDPlayerAdvanced: Playing: /Music/song.mp3
SDPlayerAdvanced: Bitrate: 320 kbps
[ID3] Artist: Artist Name
[ID3] Title: Song Title
[SDPlayerAdvanced] Rendering OLED...
```

---

## Rozwiązania typowych problemów

### Problem: Czarny ekran, brak żadnych komunikatów
**Przyczyna:** U8G2 display nie działa lub nie jest zainicjowany  
**Rozwiązanie:** 
1. Sprawdź czy radio normalne wyświetla się OK
2. Sprawdź czy U8G2 jest zainicjowany w setup()
3. Sprawdź połączenia SPI z OLED

### Problem: "ERROR: _display is NULL!"
**Przyczyna:** SDPlayerAdvanced nie dostał wskaźnika do U8G2  
**Rozwiązanie:**
```cpp
// W main.cpp - sprawdź KOLEJNOŚĆ:
g_sdPlayerManager->begin(&audio, display);  // ← 1. Najpierw begin
g_sdPlayerManager->setAdvanced(g_sdPlayerAdvanced);  // ← 2. Potem setAdvanced
```

### Problem: Stary styl SDPlayer zamiast Advanced
**Przyczyna:** Stary SDPlayerOLED nadpisuje renderowanie  
**Rozwiązanie:** 
- Upewnij się że w SDPlayerOLED_loop() jest EARLY RETURN gdy manager aktywny
- Sprawdź czy nie ma wielokrotnych wywołań u8g2.clearBuffer()

### Problem: Panel Advanced pojawia się na 0.1s i znika
**Przyczyna:** Konflikt z innymi funkcjami rysującymi na OLED  
**Rozwiązanie:**
- Sprawdź czy displayRadio() nie jest wywołane gdy SDPlayer aktywny
- Dodaj sprawdzenie `if (sdPlayerOLEDActive) return;` w innych funkcjach OLED

---

## Testowanie krok po kroku

### Test 1: Sprawdź czy display działa
```cpp
// Dodaj w SDPlayerAdvanced::renderOLED() na początku:
_display->clearBuffer();
_display->drawStr(0, 20, "TEST ADVANCED");
_display->sendBuffer();
Serial.println("TEST: Drawing 'TEST ADVANCED'");
```

Jeśli widzisz "TEST ADVANCED" na OLED → Display działa!

### Test 2: Sprawdź czy ikony są rysowane
```cpp
// Dodaj po drawXBMP:
Serial.print("Drawing icon at (");
Serial.print(iconX);
Serial.print(",2) status: ");
Serial.println(_isPlaying ? "PLAY" : "STOP");
```

### Test 3: Sprawdź częstotliwość renderowania
```cpp
// Zmień w loop() na:
if (millis() - lastRender > 1000) {  // Co 1 sekundę zamiast 100ms
    lastRender = millis();
    renderOLED();
    Serial.println("RENDER CALLED");
}
```

---

## Upload firmware i monitor

```powershell
# 1. Upload
cd "c:\YO RADIO\EVO RADIO\src\ESP32_radio_evo3.19\Platformio"
C:\Users\Mariu\.platformio\penv\Scripts\platformio.exe run --target upload

# 2. Monitor Serial
C:\Users\Mariu\.platformio\penv\Scripts\platformio.exe device monitor --baud 115200
```

Aktywuj SD Player i obserwuj komunikaty!

---

## Szybkie FAQ

**Q: Kompilacja OK, ale nic nie działa**  
A: Sprawdź komunikaty Serial Monitor - tam są wszystkie info

**Q: Widzę "Rendering OLED..." ale czarny ekran**  
A: Sprawdź czy u8g2.sendBuffer() jest wywoływane na końcu renderOLED()

**Q: Panel pojawia się tylko na chwilę**  
A: Inna funkcja nadpisuje ekran - dodaj return gdy SDPlayer aktywny

**Q: Enkoder nie reaguje**  
A: Sprawdź czy `g_sdPlayerManager->onEncoderButton()` jest wywoływane

---

**Powodzenia!** 🎵
