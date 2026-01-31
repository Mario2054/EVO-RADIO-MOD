# Instrukcja Integracji EQ16 - 16-pasmowy Equalizer

## 1. PLIKI DO SKOPIOWANIA

### A) Pliki zewnętrzne EQ16:
```
src/EQ16_GraphicEQ.h          - Definicje klasy i extern funkcji
src/EQ16_GraphicEQ.cpp        - Implementacja equalizera 16-pasmowego
```

### B) Zależności wymagane:
- U8g2 library (wyświetlacz)
- FS.h / SD.h (zapis ustawień)
- Arduino.h
- ESP32-audioI2S library (do przetwarzania audio)

## 2. ZMIENNE GLOBALNE W MAIN.CPP

```cpp
// Zmienne EQ16 - dodać na początku main.cpp
bool useEQ16 = false;           // false = 3-point EQ, true = 16-band EQ16
bool eq16MenuActive = false;    // Flaga aktywnego menu EQ16
bool eq16BandMode = true;       // Tryb wyboru pasma (true) vs gain (false)
```

## 3. DEKLARACJE EXTERN W MAIN.CPP

```cpp
// Deklaracje extern dla EQ16 - dodać przed setup()
extern U8G2_SSD1322_NHD_256X64_2_4W_HW_SPI u8g2;  // Obiekt wyświetlacza
```

## 4. INICJALIZACJA W SETUP()

```cpp
// W funkcji setup() po inicjalizacji wyświetlacza:
Serial.println("DEBUG: Initializing EQ16 system...");
EQ16_init();                    // Inicjalizacja EQ16
EQ16_loadFromSD();              // Wczytanie ustawień z SD
Serial.println("DEBUG: EQ16 initialization complete");
delay(500);                     // Krótkie opóźnienie po init
```

## 5. OBSŁUGA W LOOP()

```cpp
// W głównej pętli loop():
if (EQ16_isEnabled()) {
  EQ16_autoSave();  // Automatyczny zapis co 5 sekund
}
```

## 6. OBSŁUGA PILOTA IR - PRZYCISK EQ

```cpp
// W obsłudze IR (np. rcCmdEqulizer):
if (ir_code == rcCmdEqulizer) {
  Serial.println("DEBUG: EQ button pressed");
  
  if (!useEQ16) {
    // Przełącz na system EQ16
    switchEqualizerSystem();
  } else {
    // Toggle menu EQ16
    if (!EQ16_isMenuActive()) {
      Serial.println("DEBUG: Activating EQ16 menu");
      EQ16_setMenuActive(true);
      eq16MenuActive = true;
      eq16BandMode = true;
      EQ16_displayMenu();
    } else {
      Serial.println("DEBUG: Deactivating EQ16 menu - saving settings");
      EQ16_autoSave();           // Zapis ustawień EQ16 przy wyjściu z menu
      EQ16_setMenuActive(false);
      eq16MenuActive = false;
      displayRadio();
    }
  }
}
```

## 7. OBSŁUGA STRZAŁEK W MENU EQ16

```cpp
// Strzałka LEWO/PRAWO - zmiana pasma
if ((ir_code == rcCmdArrowLeft) && eq16MenuActive) {
  Serial.println("DEBUG: EQ16 - Arrow LEFT pressed");
  EQ16_selectPrevBand();
  EQ16_displayMenu();
}

if ((ir_code == rcCmdArrowRight) && eq16MenuActive) {
  Serial.println("DEBUG: EQ16 - Arrow RIGHT pressed");
  EQ16_selectNextBand();
  EQ16_displayMenu();
}

// Strzałka GÓRA/DÓŁ - zmiana wzmocnienia
if ((ir_code == rcCmdArrowUp) && eq16MenuActive) {
  Serial.println("DEBUG: EQ16 - Arrow UP pressed");
  EQ16_increaseBandGain();
  EQ16_displayMenu();
}

if ((ir_code == rcCmdArrowDown) && eq16MenuActive) {
  Serial.println("DEBUG: EQ16 - Arrow DOWN pressed");
  EQ16_decreaseBandGain();
  EQ16_displayMenu();
}
```

## 8. OBSŁUGA ENCODERA (OPCJONALNE)

```cpp
// Funkcja obsługi encodera dla EQ16:
void handleEQ16Encoder() {
  if (!EQ16_isMenuActive()) return;
  
  encoder.loop();
  
  if (encoder.isPressed()) {
    // Toggle trybu: pasmo <-> gain
    eq16BandMode = !eq16BandMode;
    Serial.printf("DEBUG: EQ16 encoder - switched to %s mode\n", 
                  eq16BandMode ? "BAND" : "GAIN");
    EQ16_displayMenu();
    delay(200);
  }
  
  int direction = encoder.getDirection();
  if (direction != 0) {
    if (eq16BandMode) {
      // Tryb wyboru pasma
      if (direction > 0) EQ16_selectNextBand();
      else EQ16_selectPrevBand();
    } else {
      // Tryb regulacji gain
      if (direction > 0) EQ16_increaseBandGain();
      else EQ16_decreaseBandGain();
    }
    EQ16_displayMenu();
  }
}
```

## 9. PRZYCISK OK - ZAPIS USTAWIEŃ

```cpp
// W obsłudze rcCmdOk:
if (ir_code == rcCmdOk) {
  // ... inne obsługi ...
  
  if (eq16MenuActive == true) { 
    // Komunikat o zapisie EQ16
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_fub14_tf);
    u8g2.drawStr(1, 33, "Saving EQ16 settings");
    u8g2.sendBuffer();
    EQ16_autoSave(); 
    Serial.println("DEBUG: EQ16 settings saved via OK button");
    // Wyłączamy menu EQ16 po zapisie
    EQ16_setMenuActive(false);
    eq16MenuActive = false;
    delay(1000); // Pokazujemy komunikat przez sekundę
    displayRadio(); // Powracamy do głównego ekranu
    return; // Kończymy obsługę - nie wykonujemy changeStation()
  }
  
  // WAŻNE: Warunek musi wykluczać eq16MenuActive
  if ((equalizerMenuEnable == false) && (eq16MenuActive == false)) {
    // Tutaj normalna obsługa changeStation() itp.
    if ((!urlPlaying) || (listedStations)) { changeStation();}
    // ... reszta kodu ...
  }
}
```

## 10. FUNKCJA PRZEŁĄCZANIA SYSTEMÓW

```cpp
// Funkcja przełączania między systemami
void switchEqualizerSystem() {
  // Zamykanie aktywnych menu
  if (eq16MenuActive) {
    EQ16_setMenuActive(false);
    eq16MenuActive = false;
  }
  if (equalizerMenuEnable) {
    equalizerMenuEnable = false;
  }
  
  // Przełączenie systemu
  useEQ16 = !useEQ16;
  
  // Zastosowanie ustawień
  applyEqualizerSettings();
  
  Serial.printf("DEBUG: Switched to %s equalizer system\n", 
                useEQ16 ? "EQ16 (16-band)" : "3-point");
  
  // Komunikat na wyświetlaczu
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(10, 25, useEQ16 ? "EQ16 SYSTEM" : "3-POINT EQ");
  u8g2.drawStr(10, 45, "ACTIVATED");
  u8g2.sendBuffer();
  delay(1500);
  displayRadio();  // Powrót do głównego ekranu
}

// Funkcja stosowania ustawień equalizera
void applyEqualizerSettings() {
  if (useEQ16 && EQ16_isEnabled()) {
    // Użyj systemu EQ16 - wyłącz 3-punktowy
    audio.setTone(0, 0, 0);  // Reset 3-point EQ do neutralnego
    Serial.println("DEBUG: Applied EQ16 settings - 3-point EQ disabled, 16-band processing enabled");
  } else {
    // Użyj systemu 3-punktowego
    audio.setTone(toneLowValue, toneMidValue, toneHiValue);
    Serial.printf("DEBUG: Applied 3-point EQ - Low:%d Mid:%d High:%d - 16-band processing disabled\n", 
                  toneLowValue, toneMidValue, toneHiValue);
  }
}
```

## 11. INTEGRACJA Z AUDIO PROCESSING

```cpp
// W callback audio_process_i2s:
void audio_process_i2s(int16_t* outBuff, int32_t validSamples, bool* continueI2S) {
  // Push audio samples to EQ analyzer (validSamples is number of stereo frames)
  eq_analyzer_push_samples_i16((const int16_t*)outBuff, validSamples);
  
  // EQ16 processing
  if (useEQ16 && EQ16_isEnabled()) {
    const float EQ16_ATTENUATION = 0.01f; // 100x osłabienie sygnału
    
    for (int32_t i = 0; i < validSamples * 2; i += 2) {
      // Process left channel
      float leftSample = (float)outBuff[i];
      leftSample = EQ16_processSample(leftSample);
      leftSample *= EQ16_ATTENUATION; // Osłabienie sygnału
      
      // Process right channel  
      float rightSample = (float)outBuff[i + 1];
      rightSample = EQ16_processSample(rightSample);
      rightSample *= EQ16_ATTENUATION; // Osłabienie sygnału
      
      // Convert back to int16_t with clamping
      outBuff[i] = (int16_t)constrain(leftSample, -32768.0f, 32767.0f);
      outBuff[i + 1] = (int16_t)constrain(rightSample, -32768.0f, 32767.0f);
    }
  }
}
```

## 12. OCHRONA WYŚWIETLACZA

```cpp
// W funkcji displayRadioScroller() - dodać ochronę:
void displayRadioScroller() {
  // WAŻNE: Nie nadpisuj wyświetlacza gdy aktywne menu EQ16
  if (EQ16_isMenuActive()) {
    return;
  }
  
  // ... reszta kodu scrollera ...
}

// W funkcji wyświetlania VU meters:
void displayVUMeters() {
  // WAŻNE: Nie wyświetlaj VU gdy aktywne menu EQ16
  if (EQ16_isMenuActive()) {
    return;
  }
  
  // ... kod VU meters ...
}
```

## 13. INTEGRACJA Z WEB INTERFACE (OPCJONALNE)

```cpp
// Endpoint do przełączania systemów EQ:
server.on("/switch_eq_system", HTTP_GET, [](AsyncWebServerRequest *request){
  switchEqualizerSystem();
  request->send(200, "text/plain", useEQ16 ? "EQ16 System Active" : "3-Point EQ Active");
});

// Endpoint do sprawdzania stanu:
server.on("/eq_status", HTTP_GET, [](AsyncWebServerRequest *request){
  String response = "{\"system\":\"" + String(useEQ16 ? "EQ16" : "3-point") + "\",";
  response += "\"menu_active\":" + String(EQ16_isMenuActive() ? "true" : "false") + "}";
  request->send(200, "application/json", response);
});
```

## 14. NAJWAŻNIEJSZE BŁĘDY DO UNIKNIĘCIA

### A) Problem ze slownym uruchomieniem:
```cpp
// BŁĄD - wywoływanie w każdej iteracji loop():
void loop() {
  EQ16_autoSave();  // ❌ TO SPOWALNIA SYSTEM!
}

// POPRAWKA - dodaj throttling:
unsigned long lastEQ16Save = 0;
void loop() {
  if (millis() - lastEQ16Save > 5000) {  // Co 5 sekund
    if (EQ16_isEnabled()) {
      EQ16_autoSave();
      lastEQ16Save = millis();
    }
  }
}
```

### B) Problem z wyświetlaniem "lista stacji":
```cpp
// BŁĄD - niepoprawny warunek:
if ((equalizerMenuEnable == false)) {  // ❌ Wykonuje się gdy eq16MenuActive=true
  changeStation();  // Powoduje wyświetlanie listy stacji
}

// POPRAWKA - dodaj wykluczenie EQ16:
if ((equalizerMenuEnable == false) && (eq16MenuActive == false)) {  // ✅ POPRAWNE
  changeStation();
}
```

### C) Problem z nadpisywaniem menu EQ16:
```cpp
// BŁĄD - brak ochrony:
void displayRadioScroller() {
  u8g2.clearBuffer();  // ❌ Nadpisuje menu EQ16!
}

// POPRAWKA - dodaj ochronę:
void displayRadioScroller() {
  if (EQ16_isMenuActive()) return;  // ✅ POPRAWNE
  u8g2.clearBuffer();
}
```

## 15. TESTOWANIE

1. **Test przełączania systemów**: Przycisk EQ na pilocie
2. **Test menu EQ16**: Wejście/wyjście z menu
3. **Test nawigacji**: Strzałki lewo/prawo (pasma), góra/dół (gain)
4. **Test zapisu**: Przycisk OK w menu
5. **Test auto-zapisu**: Regulacja i oczekiwanie 5 sekund
6. **Test audio processing**: Słuchanie różnic w dźwięku
7. **Test po restarcie**: Sprawdzenie czy ustawienia się wczytały

## 16. KONFIGURACJA CZĘSTOTLIWOŚCI

System EQ16 używa 16 pasm częstotliwościowych:
- 31 Hz, 62 Hz, 125 Hz, 250 Hz, 500 Hz, 1 kHz, 2 kHz, 4 kHz
- 8 kHz, 16 kHz, 31 Hz (alt), 62 Hz (alt), 125 Hz (alt), 250 Hz (alt), 500 Hz (alt), 1 kHz (alt)

Zakres regulacji: ±12dB
Filtr: IIR Peaking Filter
Częstotliwość próbkowania: 44100 Hz

## 17. PLIKI KONFIGURACYJNE

System zapisuje ustawienia w pliku `/eq16_settings.txt` na karcie SD w formacie:
```
pasmo1_gain
pasmo2_gain
...
pasmo16_gain
```

Każda wartość w zakresie -12.0 do +12.0 (float).

---

**AUTOR**: System EQ16 zintegrowany z ESP32 Radio EVO 3.19
**DATA**: Grudzień 2025
**WERSJA**: 1.0 - Kompletna integracja z dual-equalizer system