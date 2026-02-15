# Lista Modyfikacji - ESP32 Radio & SDPlayer

## Data modyfikacji: Luty 2026
## Wersja bazowa: ESP32_radio_evo3.19

---

## 1. SDPLAYER - NOWE STYLE WIZUALNE (11-14)

### 1.1 Style 11 - Bazujący na Radio Mode 0
**Plik:** `src/SDPlayer/SDPlayerOLED.cpp` (linie 1855-1948)

**Opis:**
- Duża nazwa pliku na górze (u8g2_font_helvB14_tr)
- Scrolling pełnej nazwy w środku (y=33) - dokładnie jak Radio Mode 0
- Dolny pasek: parametry audio, codec, czas
- **UWAGA:** Usunięto ikony odtwarzania z dolnego paska (>, ||, [])

**Kluczowe elementy:**
```cpp
void SDPlayerOLED::renderStyle11() {
    // Górna nazwa - duża czcionka z białym boxem numeru
    _display.setFont(u8g2_font_helvB14_tr);
    _display.drawRBox(1, 1, 21, 16, 4);
    
    // SCROLLING w środku - y=33 (DOKŁADNIE jak Radio Mode 0)
    _display.setFont(spleen6x12PL);
    int scrollW = _display.getStrWidth(scrollText.c_str());
    if (scrollW > 250) {
        _scrollPosition = (_scrollPosition + 1) % (scrollW + 20);
        int x = 0 - _scrollPosition;
        int xPos = x;
        do {
            _display.drawStr(xPos, 33, scrollText.c_str());  // y=33 KRYTYCZNE
            xPos += scrollW + 20;
        } while (xPos < 256);
    }
    
    // Dolny pasek - bez ikon odtwarzania
    _display.drawStr(0, 63, displayString.c_str());
    _display.drawStr(135, 63, streamCodec.c_str());
    _display.drawStr(226, 63, timeStr);
}
```

### 1.2 Style 12 - Bazujący na Radio Mode 1 (Duży Zegar)
**Plik:** `src/SDPlayer/SDPlayerOLED.cpp` (linie 1950-2099)

**Opis:**
- Duży zegar 7-segmentowy (HH:MM) + małe sekundy (spleen6x12PL)
- Kalendarz po prawej: dzień, miesiąc, dzień tygodnia w boxie
- POD KALENDARZEM (y=47): Speaker+Volume, Status (>, ||, []), Format pliku
- Dolny pasek: scrolling nazwy utworu (max 250px)

**Kluczowe elementy:**
```cpp
void SDPlayerOLED::renderStyle12() {
    // ZEGAR - duże godziny/minuty + małe sekundy
    _display.setFont(u8g2_font_7Segments_26x42_mn);
    snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    _display.drawStr(xtime+7, 45, timeString);
    
    // SEKUNDY - MAŁA czcionka spleen6x12PL (y=45)
    snprintf(timeString, sizeof(timeString), ":%02d", timeinfo.tm_sec);
    _display.setFont(spleen6x12PL);
    _display.drawStr(xtime+163, 45, timeString);
    
    // POD KALENDARZEM (y=47) - 3 elementy:
    // 1. Speaker + Volume (x=200)
    _display.drawGlyph(200, 47, 0x9E);
    _display.drawStr(208, 47, String(vol).c_str());
    
    // 2. Status odtwarzania (x=220) - CD player style
    if (isPlaying && !isPaused) _display.drawStr(220, 47, ">");
    else if (isPaused) _display.drawStr(220, 47, "||");
    else _display.drawStr(220, 47, "[]");
    
    // 3. Format pliku (x=235)
    _display.drawStr(235, 47, fileExt.c_str());
    
    // KALENDARZ
    _display.drawRBox(198, 20, 58, 15, 3);  // Box dzień tygodnia
    _display.drawRFrame(198, 0, 58, 35, 3); // Ramka całości
}
```

### 1.3 Style 13 - Bazujący na Radio Mode 2 (Kompaktowy)
**Plik:** `src/SDPlayer/SDPlayerOLED.cpp` (linie 2101-2165)

**Opis:**
- Góra: numer utworu w boxie + nazwa pliku
- Środek: status tekstowy "PLAYING" / "PAUSED" / "STOP" (wyśrodkowany y=30)
- Dół: parametry audio, codec, czas
- **UWAGA:** Brak ikon odtwarzania w dolnym pasku

**Kluczowe elementy:**
```cpp
void SDPlayerOLED::renderStyle13() {
    // Górny box z numerem
    _display.drawRBox(1, 1, 18, 13, 4);
    
    // Status tekstowy w środku (y=30)
    String statusText = isPlaying ? (isPaused ? "PAUSED" : "PLAYING") : "STOP";
    int statusX = (256 - _display.getStrWidth(statusText.c_str())) / 2;
    _display.drawStr(statusX, 30, statusText.c_str());
    
    // Dolny pasek - separator y=52
    _display.drawLine(0, 52, 255, 52);
}
```

### 1.4 Style 14 - Bazujący na Radio Mode 3 (Linia Statusu)
**Plik:** `src/SDPlayer/SDPlayerOLED.cpp` (linie 2167-2296)

**Opis:**
- Górna linia (y=10): TRACK:XX, codec/bitrate w ramce, volume, czas
- Środek: duża nazwa utworu (y=27) + status tekstowy (y=42)
- Dół: scrolling pełnej nazwy (y=52)
- Dolny pasek: samplerate+format, ikona statusu (>, ||, []), bitrate

**Kluczowe elementy:**
```cpp
void SDPlayerOLED::renderStyle14() {
    // GÓRNY PASEK (y=10)
    _display.drawStr(1, 10, "TRACK:");
    
    // Codec/Bitrate w ramce (jak Radio Mode 3)
    _display.setFont(u8g2_font_5x8_tr);
    uint8_t x_codec = 103;
    _display.drawFrame(x_codec, 2, 39, 9);
    _display.drawBox(x_codec + 1, 2, 21, 9);
    _display.drawStr(x_codec + 2, 9, streamCodec.c_str());
    _display.drawStr(x_codec + 23, 9, bitrateString.c_str());
    
    // ŚRODEK - duża nazwa (y=27)
    _display.setFont(u8g2_font_UnnamedDOSFontIV_tr);
    _display.drawStr(nameX, 27, displayName.c_str());
    
    // STATUS TEKSTOWY (y=42)
    String statusText = isPlaying ? (isPaused ? "PAUSED" : "PLAYING") : "STOP";
    _display.drawStr(statusX, 42, statusText.c_str());
    
    // DOLNY PASEK - IKONA STATUSU w środku (x=120)
    if (isPlaying && !isPaused) _display.drawStr(120, 63, ">");
    else if (isPaused) _display.drawStr(120, 63, "||");
    else _display.drawStr(120, 63, "[]");
}
```

---

## 2. SDPLAYER - ENUM I CYKL STYLÓW

### 2.1 Dodanie Style 14 do enum
**Plik:** `src/SDPlayer/SDPlayerOLED.h` (linie 57-69)

```cpp
enum DisplayStyle {
    STYLE_1 = 1,
    STYLE_2 = 2,
    STYLE_3 = 3,
    STYLE_4 = 4,
    STYLE_5 = 5,
    STYLE_6 = 6,
    STYLE_7 = 7,
    STYLE_10 = 10,
    STYLE_11 = 11,
    STYLE_12 = 12,
    STYLE_13 = 13,
    STYLE_14 = 14  // DODANY
};
```

### 2.2 Cykl przełączania stylów
**Plik:** `src/SDPlayer/SDPlayerOLED.cpp` (linie 1836-1850)

```cpp
void SDPlayerOLED::nextStyle() {
    switch (_currentStyle) {
        case STYLE_1: _currentStyle = STYLE_2; break;
        case STYLE_2: _currentStyle = STYLE_3; break;
        case STYLE_3: _currentStyle = STYLE_4; break;
        case STYLE_4: _currentStyle = STYLE_5; break;
        case STYLE_5: _currentStyle = STYLE_6; break;
        case STYLE_6: _currentStyle = STYLE_7; break;
        case STYLE_7: _currentStyle = STYLE_10; break;
        case STYLE_10: _currentStyle = STYLE_11; break;
        case STYLE_11: _currentStyle = STYLE_12; break;
        case STYLE_12: _currentStyle = STYLE_13; break;
        case STYLE_13: _currentStyle = STYLE_14; break;  // ZMIENIONE
        case STYLE_14: _currentStyle = STYLE_1; break;    // DODANE
        default: _currentStyle = STYLE_1; break;
    }
}
```

### 2.3 Deklaracje funkcji renderowania
**Plik:** `src/SDPlayer/SDPlayerOLED.h` (linie 141-152)

```cpp
void renderStyle1();
void renderStyle2();
void renderStyle3();
void renderStyle4();
void renderStyle5();
void renderStyle6();
void renderStyle7();
void renderStyle10();
void renderStyle11();   // Radio Mode 0 - scrolling y=33
void renderStyle12();   // Radio Mode 1 - duży zegar + małe sekundy
void renderStyle13();   // Radio Mode 2 - kompaktowy 3-liniowy
void renderStyle14();   // Radio Mode 3 - linia statusu
```

---

## 3. ANALYZER CONFIG - AUTOMATYCZNE ŁADOWANIE

### 3.1 Dodanie analyzerStyleLoad() w setup()
**Plik:** `src/main.cpp` (linia 9475)

**KRYTYCZNE:** Plik `/analyzer.cfg` musi być załadowany przy starcie, inaczej analizator nie działa poprawnie.

**Lokalizacja w setup():**
```cpp
void setup() {
    // ... inne inicjalizacje ...
    
    readConfig();          // Istniejąca funkcja
    analyzerStyleLoad();   // DODANE - ładuje /analyzer.cfg
    
    // ... reszta setup() ...
}
```

**Opis:**
- Ładuje konfigurację analizatora z pliku `/analyzer.cfg` na SD/SPIFFS
- Musi być wywołane BEZPOŚREDNIO PO `readConfig()`
- Ustawia parametry EQ, kolory, style wizualizacji

---

## 4. MUTE FUNCTIONALITY - RADIO STYLES 5, 6-10

### 4.1 Przekreślony głośnik w górnym pasku - Style 6-10
**Plik:** `src/main.cpp` (linie 3463-3469)

**Opis:**
- Rysuje przekreślony głośnik w środku górnego paska gdy `volumeMute == true`
- Analizator dalej działa normalnie, tylko pokazuje ikonę mute

**Kod w displayRadio() dla stylów 6-10:**
```cpp
else if (displayMode >= 6 && displayMode <= 10) {
    u8g2.clearBuffer();
    u8g2.drawLine(128,0,128,64);
    
    // ... rysowanie interfejsu ...
    
    u8g2.setDrawColor(1);
    
    // PRZEKREŚLONY GŁOŚNIK PRZY MUTE (w górnym pasku)
    if (volumeMute) {
      u8g2.setFont(spleen6x12PL);
      u8g2.drawGlyph(60, 10, 0x9E); // Głośnik w środku górnego paska
      u8g2.drawLine(55, 0, 70, 12); // Linia przekreślająca /
    }
    
    // ... reszta kodu ...
}
```

### 4.2 Przekreślony głośnik w górnym pasku - Style 5
**Plik:** `src/main.cpp` (linie 3704-3710)

**Kod w displayRadio() dla stylu 5:**
```cpp
else if (displayMode == 5) {
    u8g2.clearBuffer();
    u8g2.setFont(spleen6x12PL);
    
    // ... rysowanie interfejsu ...
    
    // PRZEKREŚLONY GŁOŚNIK PRZY MUTE (w górnym pasku)
    if (volumeMute) {
      u8g2.setFont(spleen6x12PL);
      u8g2.drawGlyph(60, 10, 0x9E); // Głośnik w środku górnego paska
      u8g2.drawLine(55, 0, 70, 12); // Linia przekreślająca /
    }
}
```

### 4.3 Usunięcie clearBuffer przy MUTE dla stylów 5-10
**Plik:** `src/main.cpp` (linie 3300-3310, 3420-3443)

**WAŻNE:** Usunięto bloki kodu które czyściły ekran i pokazywały "MUTED" na środku dla stylów 5 i 6-10:

**PRZED (USUNIĘTE):**
```cpp
// USUNIĘTO TEN KOD:
if (volumeMute) {
    u8g2.clearBuffer();
    u8g2.setFont(spleen6x12PL);
    u8g2.drawGlyph(110, 35, 0x9E);
    u8g2.drawLine(105, 20, 135, 45);
    u8g2.drawStr(95, 50, "MUTED");
    u8g2.sendBuffer();
    return;
}
```

**PO (OBECNE):**
```cpp
// Style 5 i 6-10 - BEZ bloku MUTE na początku funkcji
if (displayMode == 5) {
    if (stationString == "") { ... }
    // Przekreślony głośnik rysowany później w kodzie
}
```

### 4.4 Modyfikacja głównej pętli MUTE
**Plik:** `src/main.cpp` (linie 10838-10870)

**Opis:**
- Dla stylów 0-4: pokazuj tekst "> MUTED <"
- Dla stylów 5-10: NIE rysuj niczego (przekreślony głośnik już jest w displayRadio())

**Kod:**
```cpp
else // Obsługa wyciszenia dźwięku, wprowadzamy napis MUTE na ekran
{
    // Style 0-4: Pokazuj tekst "> MUTED <"
    if (displayMode == 0) {
        u8g2.setDrawColor(0);
        u8g2.drawStr(0,48, "> MUTED <");
        u8g2.setDrawColor(1);
    }
    else if (displayMode == 1) {
        u8g2.setDrawColor(0);
        u8g2.drawStr(200,47, "> MUTED <");
        u8g2.setDrawColor(1);
    }
    else if (displayMode == 2) {
        u8g2.setDrawColor(0);
        u8g2.drawStr(0,48, "> MUTED <");
        u8g2.setDrawColor(1);
    }
    else if (displayMode == 3) {
        u8g2.setDrawColor(0);
        u8g2.drawStr(101,63, "> MUTED <");
        u8g2.setDrawColor(1);
    }
    else if (displayMode == 4) {
        u8g2.setDrawColor(1);
        vuMeterMode4();
        u8g2.setFont(spleen6x12PL);
        u8g2.setDrawColor(0);
        u8g2.drawStr(103,57, "> MUTED <");
        u8g2.setDrawColor(1);
    }
    // Style 5-10: Przekreślony głośnik jest już narysowany w displayRadio(), nic nie robimy tutaj
}
```

---

## 5. ZMIENNE ZEWNĘTRZNE UŻYWANE W SDPLAYER

### 5.1 Lista wymaganych extern
**Plik:** `src/SDPlayer/SDPlayerOLED.cpp`

SDPlayer używa tych zmiennych z głównego programu:

```cpp
extern String streamCodec;           // Format audio (MP3, AAC, FLAC, etc.)
extern String bitrateString;         // Bitrate jako string
extern uint32_t SampleRate;          // Częstotliwość próbkowania (główna część)
extern uint8_t SampleRateRest;       // Częstotliwość próbkowania (część dziesiętna)
extern String bitsPerSampleString;   // Głębia bitowa
```

**UWAGA:** Te zmienne muszą być zdefiniowane w `main.cpp` jako globalne.

---

## 6. CZCIONKI UŻYWANE W NOWYCH STYLACH

### 6.1 Lista czcionek
```cpp
// Główne czcionki SDPlayer:
u8g2_font_helvB14_tr          // Duża czcionka nagłówków (Style 11)
u8g2_font_7Segments_26x42_mn  // 7-segmentowy zegar (Style 12)
u8g2_font_fub14_tf            // Kalendarz (Style 12)
spleen6x12PL                  // Tekst scrollujący, małe elementy
u8g2_font_04b_03_tr           // Bardzo małe ikony
u8g2_font_UnnamedDOSFontIV_tr // Duża nazwa utworu (Style 14)
u8g2_font_5x8_tr              // Małe teksty (Style 14 codec/bitrate)

// Czcionki Radio (dla stylów 5-10 MUTE):
spleen6x12PL                  // Głośnik i "MUTED"
mono04b03b                    // Codec info (Radio Mode 5)
```

---

## 7. PARAMETRY POZYCJONOWANIA EKRANU

### 7.1 Krytyczne współrzędne Y
```cpp
// Style 11 (Radio Mode 0):
y=16  - Duża nazwa pliku na górze
y=33  - Scrolling tekstu (DOKŁADNIE jak Radio Mode 0)
y=52  - Linia separatora
y=63  - Dolny pasek info

// Style 12 (Radio Mode 1):
y=45  - Zegar (duży) i sekundy (małe)
y=47  - Informacje pod kalendarzem (volume, status, format)
y=50  - Separator
y=63  - Scrolling nazwy utworu

// Style 13 (Radio Mode 2):
y=11  - Nazwa pliku
y=30  - Status tekstowy (PLAYING/PAUSED/STOP)
y=52  - Separator
y=63  - Dolny pasek

// Style 14 (Radio Mode 3):
y=10  - Górna linia statusu (track, codec, volume, czas)
y=27  - Duża nazwa utworu
y=42  - Status tekstowy
y=52  - Scrolling pełnej nazwy
y=56  - Separator
y=63  - Dolny pasek (info + ikona statusu + bitrate)
```

### 7.2 Pozycje ikon i elementów
```cpp
// Style 12 - pod kalendarzem (y=47):
x=200 - Speaker icon + volume
x=220 - Status icon (>, ||, [])
x=235 - Format pliku

// Style 14 - górny pasek (y=10):
x=1   - "TRACK:XX"
x=103 - Codec/bitrate box
x=180 - Speaker icon
x=189 - Volume value
x=226 - Czas

// Style 14 - dolny pasek (y=63):
x=0   - Samplerate + format
x=120 - Ikona statusu
x=256-bitrateW - Bitrate (wyrównany do prawej)
```

---

## 8. INSTRUKCJE INTEGRACJI DO NOWEGO SOFTU

### 8.1 Integracja SDPlayer Styles 11-14

**Krok 1:** Dodaj deklaracje do `SDPlayerOLED.h`:
```cpp
// Do enum DisplayStyle:
STYLE_11 = 11,
STYLE_12 = 12,
STYLE_13 = 13,
STYLE_14 = 14

// Do deklaracji funkcji:
void renderStyle11();
void renderStyle12();
void renderStyle13();
void renderStyle14();
```

**Krok 2:** Skopiuj funkcje z `SDPlayerOLED.cpp`:
- `renderStyle11()` (linie 1855-1948)
- `renderStyle12()` (linie 1950-2099)
- `renderStyle13()` (linie 2101-2165)
- `renderStyle14()` (linie 2167-2296)

**Krok 3:** Zaktualizuj `nextStyle()` (linie 1836-1850)

**Krok 4:** Dodaj case do `render()`:
```cpp
case STYLE_11: renderStyle11(); break;
case STYLE_12: renderStyle12(); break;
case STYLE_13: renderStyle13(); break;
case STYLE_14: renderStyle14(); break;
```

**Krok 5:** Upewnij się że zmienne extern są dostępne:
```cpp
extern String streamCodec;
extern String bitrateString;
extern uint32_t SampleRate;
extern uint8_t SampleRateRest;
extern String bitsPerSampleString;
```

### 8.2 Integracja Analyzer Config Loading

**Krok 1:** W `setup()` w `main.cpp`, dodaj ZARAZ PO `readConfig()`:
```cpp
readConfig();
analyzerStyleLoad();  // DODAJ TĘ LINIĘ
```

### 8.3 Integracja MUTE dla Radio Styles 5-10

**Krok 1:** W funkcji `displayRadio()`, dla stylu 5 DODAJ przed końcem:
```cpp
// PRZED zamknięciem bloku displayMode == 5:
if (volumeMute) {
    u8g2.setFont(spleen6x12PL);
    u8g2.drawGlyph(60, 10, 0x9E);
    u8g2.drawLine(55, 0, 70, 12);
}
```

**Krok 2:** W funkcji `displayRadio()`, dla stylów 6-10 DODAJ przed końcem:
```cpp
// PRZED zamknięciem bloku displayMode >= 6 && displayMode <= 10:
if (volumeMute) {
    u8g2.setFont(spleen6x12PL);
    u8g2.drawGlyph(60, 10, 0x9E);
    u8g2.drawLine(55, 0, 70, 12);
}
```

**Krok 3:** USUŃ bloki clearBuffer przy MUTE na początku funkcji displayRadio() dla stylów 5 i 6-10:
```cpp
// USUŃ TEN KOD (jeśli istnieje):
if (volumeMute) {
    u8g2.clearBuffer();
    u8g2.setFont(spleen6x12PL);
    u8g2.drawGlyph(110, 35, 0x9E);
    u8g2.drawLine(105, 20, 135, 45);
    u8g2.drawStr(95, 50, "MUTED");
    u8g2.sendBuffer();
    return;
}
```

**Krok 4:** W głównej pętli (loop), w sekcji obsługi MUTE, ZAMIEŃ:
```cpp
// STARY KOD:
else {
    u8g2.setDrawColor(0);
    if (displayMode == 0) {u8g2.drawStr(0,48, "> MUTED <");}
    // ... itd dla wszystkich trybów ...
    u8g2.setDrawColor(1);
}

// NOWY KOD:
else {
    if (displayMode == 0) {
        u8g2.setDrawColor(0);
        u8g2.drawStr(0,48, "> MUTED <");
        u8g2.setDrawColor(1);
    }
    else if (displayMode == 1) {
        u8g2.setDrawColor(0);
        u8g2.drawStr(200,47, "> MUTED <");
        u8g2.setDrawColor(1);
    }
    else if (displayMode == 2) {
        u8g2.setDrawColor(0);
        u8g2.drawStr(0,48, "> MUTED <");
        u8g2.setDrawColor(1);
    }
    else if (displayMode == 3) {
        u8g2.setDrawColor(0);
        u8g2.drawStr(101,63, "> MUTED <");
        u8g2.setDrawColor(1);
    }
    else if (displayMode == 4) {
        u8g2.setDrawColor(1);
        vuMeterMode4();
        u8g2.setFont(spleen6x12PL);
        u8g2.setDrawColor(0);
        u8g2.drawStr(103,57, "> MUTED <");
        u8g2.setDrawColor(1);
    }
    // Style 5-10: Przekreślony głośnik jest już narysowany w displayRadio()
}
```

---

## 9. WERYFIKACJA POPRAWNOŚCI INTEGRACJI

### 9.1 Checklist
- [ ] SDPlayer ma 12 stylów (1-7, 10-14)
- [ ] Style 11-14 dokładnie pasują do Radio Mode 0-3
- [ ] Zegar w stylu 12 ma MAŁE sekundy (spleen6x12PL)
- [ ] Style 12 ma 3 elementy pod kalendarzem (volume, status, format)
- [ ] Scrolling w stylu 11 jest na y=33
- [ ] `analyzerStyleLoad()` jest w setup() zaraz po `readConfig()`
- [ ] Radio styles 5-10 pokazują tylko przekreślony głośnik przy MUTE
- [ ] Radio styles 5-10 NIE czyszczą ekranu przy MUTE
- [ ] Kompilacja przechodzi bez błędów
- [ ] RAM: ~23.6%, Flash: ~82.3%

### 9.2 Testy funkcjonalne
1. Przełączanie stylów SDPlayer (przycisk STYLE) - sprawdź 12 stylów
2. Sprawdź zegar w stylu 12 - sekundy mają być małe
3. Sprawdź scrolling w stylu 11 - pozycja y=33
4. Sprawdź MUTE w stylach radia 5-10 - tylko przekreślony głośnik
5. Sprawdź analizator - czy działa po załadowaniu /analyzer.cfg

---

## 10. ZNANE PROBLEMY I OGRANICZENIA

### 10.1 Brak obsługi błędów
- Jeśli `/analyzer.cfg` nie istnieje, analizator może nie działać
- Brak sprawdzania czy czcionki są dostępne

### 10.2 Hardcoded współrzędne
- Pozycje elementów są ustawione na sztywno dla ekranu 256x64
- Zmiana rozdzielczości wymaga modyfikacji wszystkich współrzędnych

### 10.3 Zależności od zmiennych globalnych
- SDPlayer wymaga zmiennych `extern` z main.cpp
- Brak enkapsulacji danych audio

---

## 11. PRZYSZŁE ULEPSZENIA (OPCJONALNE)

1. **Konfiguracja pozycji przez plik config**
   - Współrzędne X/Y w /sdplayer.cfg
   - Wybór czcionek przez config

2. **Obsługa różnych rozdzielczości**
   - Automatyczne skalowanie elementów
   - Layout responsive

3. **Więcej ikon statusu**
   - Shuffle, Repeat
   - Tryby EQ

4. **Animacje przejść**
   - Fade między stylami
   - Smooth scrolling

---

## PODSUMOWANIE

**Zmodyfikowane pliki:**
1. `src/SDPlayer/SDPlayerOLED.h` - deklaracje stylów 11-14
2. `src/SDPlayer/SDPlayerOLED.cpp` - implementacje stylów 11-14
3. `src/main.cpp` - analyzerStyleLoad(), MUTE dla stylów 5-10

**Kluczowe funkcjonalności:**
- 4 nowe style SDPlayer (11-14) dopasowane do Radio Modes (0-3)
- Automatyczne ładowanie konfiguracji analizatora przy starcie
- Przekreślony głośnik przy MUTE dla stylów radia 5-10 (bez blokowania analizatora)

**Testowane na:**
- Hardware: ESP32-S3, OLED SSD1322 256x64
- Flash: 82.3% (2588570 bytes)
- RAM: 23.6% (77400 bytes)
- Kompilacja: SUCCESS

---

**Data dokumentu:** 14 lutego 2026  
**Autor:** GitHub Copilot  
**Wersja:** 1.0
