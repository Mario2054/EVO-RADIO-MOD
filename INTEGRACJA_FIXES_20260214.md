# Integracja poprawek i diagnostyki - 14.02.2026

## Status kompilacji
✅ **SUKCES** - RAM: 23.6% (77232B), Flash: 82.6% (2.5MB)

## Zmiany w main.cpp

### 1. MUTE Bug Fix - Synchronizacja ekranu (linia ~11098, ~7056)

**Problem:** Po naciśnięciu MUTE na stylach 5, 6, 10 pojawiało się dodatkowe menu

**Przyczyna:** `displayRadio()` wywoływane bez natychmiastowego `u8g2.sendBuffer()`, co pozwalało innym funkcjom modyfikować bufor przed wysłaniem

**Rozwiązanie:**
```cpp
// IR Handler - rcCmdMute
if (!EQ16_isMenuActive()) {
  displayRadio();
  u8g2.sendBuffer(); // Natychmiastowa aktualizacja ekranu po MUTE
}

// Button2 Handler - #ifdef twoEncoders
if (!EQ16_isMenuActive()) {
  displayRadio();
  u8g2.sendBuffer(); // Natychmiastowa aktualizacja ekranu po MUTE
}
```

### 2. Analyzer Config Debug - Setup markers (linia ~8795)

**Cel:** Widoczne logowanie procesu ładowania konfiguracji analizatora podczas startu

```cpp
Serial.println("\n*** LOADING ANALYZER CONFIGURATION ***");
Serial.println("DEBUG: Odczyt analyzer.cfg w main.cpp...");
// ... kod ładowania ...
Serial.println("*** ANALYZER CONFIGURATION LOAD FINISHED ***\n");
```

### 3. Diagnostic Endpoints (linia ~10320)

#### A. `/analyzerDebug` - Diagnostyka pliku konfiguracji
- Pokazuje czy plik analyzer.cfg istnieje
- Wyświetla rozmiar pliku w bajtach
- Pokazuje pełną zawartość pliku w przeglądarce
- Przeładowuje config z pełnymi logami Serial
- Instrukcja: "Zobacz Serial Monitor dla szczegółowych logów!"

#### B. `/analyzerTestSave` - Test zapisu
- Ręczne wywołanie `analyzerStyleSave()`
- Pełne logowanie procesu zapisu w Serial Monitor
- Zwraca: "Test save complete - check Serial Monitor for logs!"

**Użycie:**
1. Otwórz `http://[IP_ESP32]/analyzerDebug` - zobacz zawartość pliku
2. Otwórz `http://[IP_ESP32]/analyzerTestSave` - test zapisu config
3. Monitoruj Serial @ 115200 baud dla szczegółowych logów

## Zmiany w EQ_AnalyzerDisplay.cpp

### Usuniecie stylów 7, 8, 9 (nieistniejące)

**Problem:** Funkcje save/load zawierały kod dla stylów 7, 8, 9, które nie są zaimplementowane

**Rozwiązanie:**
- `analyzerStyleLoad()` - usunięto parsing s7*, s8*, s9*
- `analyzerStyleSave()` - usunięto sekcje Style7, Style8, Style9
- `analyzerStyleToSaveString()` - usunięto s7*, s8*, s9*
- `analyzerStyleLoadFromString()` - usunięto parsing s7*, s8*, s9*
- `analyzerStyleToJson()` - usunięto pola s7_*, s8_*, s9_*

**Efekt:** Plik analyzer.cfg zawiera tylko używane style: 5 (VU), 6 (Segmented), 10 (Floating Peaks)

### Debug logging

Dodano szczegółowe logi w:
- `analyzerStyleLoad()`:
  ```
  === ANALYZER CONFIG LOAD START ===
  File open: SUCCESS/FAIL
  File size: XXX bytes
  Loaded parameters: XX
  === ANALYZER CONFIG LOAD COMPLETE ===
  ```

- `analyzerStyleSave()`:
  ```
  === ANALYZER CONFIG SAVE START ===
  File open for write: SUCCESS/FAIL
  Bytes written: XXX
  === ANALYZER CONFIG SAVE COMPLETE ===
  ```

## Poprawki techniczne

### Font Linkage Fix (linia 1822)

**Problem:** Linker error - `undefined reference to 'spleen6x12PL'`

**Przyczyna:** Konflikt między `const uint8_t` w main.cpp a `extern uint8_t` w SDPlayerOLED.cpp

**Rozwiązanie:**
- main.cpp: `uint8_t spleen6x12PL[2958]` (bez const)
- SDPlayerOLED.cpp: `extern uint8_t spleen6x12PL[]`

## Testowanie

### Kroki po wgraniu firmware:

1. **Serial Monitor (115200 baud)** podczas restartu:
   ```
   *** LOADING ANALYZER CONFIGURATION ***
   DEBUG: Odczyt analyzer.cfg w main.cpp...
   DEBUG: Odczytano XXX bajtów z analyzer.cfg
   === ANALYZER CONFIG LOAD START ===
   ...
   === ANALYZER CONFIG LOAD COMPLETE ===
   *** ANALYZER CONFIGURATION LOAD FINISHED ***
   ```

2. **Test MUTE:**
   - Uruchom style 5, 6 lub 10
   - Naciśnij MUTE na pilocie (0xB916)
   - Sprawdź czy nie pojawia się dodatkowe menu

3. **Test persistence konfiguracji:**
   - Otwórz `http://[IP]/analyzer`
   - Zmień parametry stylu (np. barWidth, gap)
   - Kliknij SAVE
   - Sprawdź Serial - powinien pokazać:
     ```
     === ANALYZER CONFIG SAVE START ===
     ...
     === ANALYZER CONFIG SAVE COMPLETE ===
     ```
   - Restart ESP32
   - Sprawdź czy zmiany zostały zachowane

4. **Diagnostyka:**
   - `/analyzerDebug` - zobacz plik analyzer.cfg
   - `/analyzerTestSave` - test zapisu (sprawdź Serial)

## Pliki zmodyfikowane

1. `src/main.cpp` - MUTE fix, setup markers, diagnostic endpoints, font fix
2. `src/EQ_AnalyzerDisplay.cpp` - usunięcie stylów 7-9, debug logging
3. `src/SDPlayer/SDPlayerOLED.cpp` - extern declaration fix

## Notatki

- Styles 7, 8, 9 były w pliku config ale NIGDY nie zostały zaimplementowane w kodzie
- Funkcja `displayMode >= 7 && displayMode <= 9` w renderze była zawsze pomijana
- Zachowanie tylko aktywnych stylów: 5, 6, 10
- Peak hold time działa globalnie dla wszystkich stylów

## Wersja
- ESP32-S3, PlatformIO, Arduino Framework 3.2.0
- Kompilacja: 14.02.2026
- Build: SUCCESS
