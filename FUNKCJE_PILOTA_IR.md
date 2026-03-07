# 📡 EVO RADIO - Kompletny Przewodnik Obsługi Pilota IR

> **Wersja:** v3.19.70  
> **Data:** 2 marca 2026  
> **Standard IR:** NEC  

---

## 📋 Spis Treści

1. [Funkcje Podstawowe](#-funkcje-podstawowe)
2. [Nawigacja i Menu](#-nawigacja-i-menu)
3. [SDPlayer - Odtwarzacz](#-sdplayer---odtwarzacz)
4. [Moduł Bluetooth](#-moduł-bluetooth)
5. [Equalizer](#-equalizer)
6. [Nagrywanie](#-nagrywanie)
7. [Przyciski Zarezerwowane](#-przyciski-zarezerwowane)
8. [Tabela Szybkiego Dostępu](#-tabela-szybkiego-dostępu)

---

## 🎵 Funkcje Podstawowe

### 🔊 Głośność
- **VOL+** (`rcCmdVolumeUp`)
  - Zwiększa głośność o 1 poziom
  - Zakres: 1-21 (standardowy) lub 0-42 (rozszerzony)
  - Działa globalnie we wszystkich trybach

- **VOL-** (`rcCmdVolumeDown`)
  - Zmniejsza głośność o 1 poziom
  - Zakres: 1-21 (standardowy) lub 0-42 (rozszerzony)
  - Działa globalnie we wszystkich trybach

- **MUTE** (`rcCmdMute`)
  - Toggle wyciszenia dźwięku (ON/OFF)
  - Zachowuje poziom głośności
  - Ikona przekreślonego głośnika w stylach 5, 6, 10

### ⚡ Power
- **POWER** (`rcCmdPower`)
  - Power OFF z animacją (jeśli włączona)
  - Działa w trybie Radio i SDPlayer
  - Zapisuje stan do pamięci (bank, stacja, volume)
  - Przejście do Light Sleep (zużycie ~5mA)

- **RED** (`rcCmdRed`)
  - Alternatywny przycisk Power OFF
  - Identyczna funkcja jak POWER
  - Z animacją jeśli włączona

### 🖥️ Wyświetlacz
- **SRC** (`rcCmdSrc`)
  - Zmiana stylu wyświetlacza: 0→1→2→3→4→6→8→10→0
  - Pomija style: 5 (usunięty), 7, 9
  - Auto-pomija wyłączone style (6, 8, 10 konfigurowalne)
  - **Style dostępne:**
    - **0** - Podstawowy z scrollującym tekstem
    - **1** - Duży zegar
    - **2** - 3 linijki tekstu stacji
    - **3** - Linia statusu z VU meter
    - **4** - Duży wskaźnik VU wskazówkowy
    - **6** - Z analizatorem spektrum (włączalny)
    - **8** - Kompaktowy layout
    - **10** - Pełnoekranowy z animacją (włączalny)

- **DIRECT** (`rcCmdDirect`)
  - **W Radio:** Toggle nagrywania strumienia (START/STOP)
  - **W Menu Bank:** Przełącznik GitHub ↔ karta SD
  - **W Equalizer:** Reset wszystkich wartości do 0
  - **W Display Mode 4:** Debug audio buffer (wyświetla bufor)
  - **Inne tryby:** Dimmer OLED (toggle przyciemnienia)

- **CH+** (`rcCmdCHPlus`)
  - Zwiększa jasność OLED o 20 (zakres: 10-255)
  - Wyświetla wartość jasności na ekranie przez 1.5s
  - Funkcja pomocnicza

- **CH-** (`rcCmdCHMinus`)
  - Zmniejsza jasność OLED o 20 (zakres: 10-255)
  - Wyświetla wartość jasności na ekranie przez 1.5s
  - Funkcja pomocnicza

---

## 🧭 Nawigacja i Menu

### Strzałki Kierunkowe
- **→ PRAWO** (`rcCmdArrowRight`)
  - **Radio:** Następna stacja (station_nr+1)
  - **Menu Bank:** Bank+1 (z zawijaniem 1-17)
  - **Equalizer:** Parametr+ (zwiększ wartość Low/Mid/High)
  - **EQ16:** Następne pasmo (0-15)

- **← LEWO** (`rcCmdArrowLeft`)
  - **Radio:** Poprzednia stacja (station_nr-1)
  - **Menu Bank:** Bank-1 (z zawijaniem 17-1)
  - **Equalizer:** Parametr- (zmniejsz wartość Low/Mid/High)
  - **EQ16:** Poprzednie pasmo (15-0)

- **↑ GÓRA** (`rcCmdArrowUp`)
  - **Radio:** Lista stacji - przewiń w górę
  - **SDPlayer:** Lista utworów - przewiń w górę
  - **Equalizer 3-band:** Poprzedni parametr (3→2→1)
  - **EQ16:** Zwiększ wzmocnienie pasma (+1, max +16)

- **↓ DÓŁ** (`rcCmdArrowDown`)
  - **Radio:** Lista stacji - przewiń w dół
  - **SDPlayer:** Lista utworów - przewiń w dół
  - **Equalizer 3-band:** Następny parametr (1→2→3)
  - **EQ16:** Zmniejsz wzmocnienie pasma (-1, min -16)

### Akcje
- **OK** (`rcCmdOk`)
  - **Lista stacji:** Zatwierdź wybór i uruchom stację
  - **Menu Bank:** Zatwierdź i załaduj bank z SD/GitHub
  - **Equalizer:** Zapisz ustawienia do pliku `/eq.txt`
  - **URL WebUI:** Zatwierdź i uruchom streamowanie
  - **SDPlayer:** Zatwierdź wybór utworu i rozpocznij odtwarzanie

- **BACK** (`rcCmdBack`)
  - Powrót do głównego ekranu radia
  - Czyści wszystkie flagi menu (lista, bank, EQ)
  - **Jeśli nagrywanie aktywne:** Zatrzymuje nagrywanie
  - Przywraca stan `displayRadio()`

### Klawisze Numeryczne (0-9)
- **KEY 1-9** (`rcCmdKey1` - `rcCmdKey9`)
  - **Radio:** Wybór stacji wielocyfrowy (timeout 3s)
    - Jedna cyfra: Stacja 1-9
    - Dwie cyfry: Stacja 10-99 (np. 2→5 = stacja 25)
  - **SDPlayer:** Wybór utworu wielocyfrowy (1-100)
  - **Equalizer:** Normalnie zablokowane

- **KEY 0** (`rcCmdKey0`)
  - **Equalizer aktywny:** Toggle EQ3 ↔ EQ16
  - **Radio/SDPlayer:** Cyfra 0 w wielocyfrowym wyborze
  - **Combo:** 000 (triple zero) - specjalna funkcja

### Menu Banków
- **BANK+** (`rcCmdBankPlus`)
  - **Pierwsze naciśnięcie:** Otwiera menu wyboru banku
  - **W menu:** Bank+1 (zakres 1-17 z zawijaniem)
  - Wyświetla aktualny numer banku na ekranie

- **BANK-** (`rcCmdBankMinus`)
  - **Pierwsze naciśnięcie:** Otwiera menu wyboru banku
  - **W menu:** Bank-1 (zakres 17-1 z zawijaniem)
  - Wyświetla aktualny numer banku na ekranie

- **W Menu Bank:**
  - **RED/DIRECT:** Przełącz źródło (GitHub ↔ SD karta)
  - **OK:** Załaduj wybrany bank
  - **BACK:** Anuluj i wróć do radia

---

## 📀 SDPlayer - Odtwarzacz

### Aktywacja
- **HELP** (`rcCmdHELP`)
  - **Toggle ON/OFF** odtwarzacza SDPlayer
  - **ON:** Zapamiętuje aktualny bank/stację → Splash "SD PLAYER" → Aktywacja
  - **OFF:** Dezaktywacja → Powrót do zapamiętanej stacji radia
  - Włącza 3x większą dynamikę analizatora

- **AUD (podwójne kliknięcie)** (`rcCmdAud`)
  - **Podwójne klikniecie w 600ms:** Aktywuje SDPlayer (jak HELP)
  - **Pojedyncze kliknięcie:** Otwiera Equalizer
  - Po aktywacji pokazuje splash screen 1s

### Kontrola Odtwarzania
- **PLAY/PAUSE** (`rcCmdPLAY` / `rcCmdPAUSE`)
  - Uruchomienie odtwarzania wybranego utworu
  - Toggle PLAY ↔ PAUSE
  - Synchronizacja z WebUI
  - **W Radio:** Blokowany

- **STOP** (`rcCmdSTOP`)
  - Całkowite zatrzymanie odtwarzania
  - Reset pozycji do początku pliku
  - **W Radio:** Blokowany

- **WELEMENT** (`rcCmdWELEMENT`)
  - Alternatywny przycisk PAUSE/PLAY
  - Toggle wstrzymania/wznowienia
  - Identyczna funkcja jak PAUSE

- **WINPOD** (`rcCmdWINPOD`)
  - Alternatywny przycisk STOP
  - Zatrzymuje i resetuje pozycję
  - Identyczna funkcja jak STOP

### Nawigacja
- **REV** (`rcCmdREV`)
  - **Pojedyncze kliknięcie:** Przewiń do tyłu -10s (seek -10000ms)
  - **Podwójne kliknięcie (600ms):** Poprzedni utwór
  - Double-click detection z timeout

- **RER** (`rcCmdRER`)
  - **Pojedyncze kliknięcie:** Przewiń do przodu +10s (seek +10000ms)
  - **Podwójne kliknięcie (600ms):** Następny utwór
  - Double-click detection z timeout

- **FILES/MENU** (`rcCmdFILES` / `rcCmdMENU`)
  - Pokazuje listę utworów z katalogu `/music/`
  - Maksymalnie 100 plików
  - Nawigacja: ↑/↓, wybór: OK
  - **W Radio:** Blokowany

- **↑/↓** (w liście utworów)
  - Przewijanie listy utworów
  - Zakres: 1-100 (tracksCount)
  - 4 widoczne pozycje na ekranie
  - Zawijanie na końcach listy

### Wyjście
- **EXIT** (`rcCmdEXIT`)
  - Wyjście z SDPlayer
  - Powrót do radia (przywraca bank/stację)
  - Wyłącza analizator SDPlayer mode

- **RADIO** (`rcCmdRADIO`)
  - Szybkie wyjście do trybu radio (alias EXIT)
  - Przywraca ostatnią stację
  - Identyczna funkcja jak EXIT

### Style Wyświetlania
- **BT** (`rcCmdBT`) - **W SDPlayer**
  - Zmiana stylu wyświetlania (1-14)
  - Pętla: 1→2→3→...→14→1
  - Pomija wyłączone style (konfigurowalne w WebUI)
  - **W Radio:** Otwiera stronę BT

- **MINIMA** (`rcCmdMINIMA`)
  - Następny styl wyświetlania
  - Identyczna funkcja jak BT w SDPlayer

- **MAXIMA** (`rcCmdMAXIMA`)
  - Zmiana stylu (obecnie też nextStyle)
  - Może być rozszerzone o prevStyle()

### Klawisze Numeryczne w SDPlayer
- **1-9, 0** - Wybór utworu wielocyfrowy
  - Timeout 3 sekundy na wprowadzenie liczby
  - Zakres: 1-100
  - Buffer: `irInputBuffer`, aktywacja: `irInputActive`

---

## 📶 Moduł Bluetooth

### Kontrola Modułu
- **KOL** (`rcCmdKOL`)
  - **Toggle ON/OFF** modułu BT UART
  - Wyświetla "BT: ON" lub "BT: OFF" na OLED przez 2s
  - Globalna flaga: `btModuleEnabled`

- **INFO** (`rcCmdINFO`)
  - Wyświetla informacje o module BT:
    - Status: OK / OFF
    - Config path: `/bt`
  - Ekran informacyjny 3 sekundy

- **CHAT** (`rcCmdCHAT`)
  - Wyświetla pełny status BT na OLED:
    - Włączony/Wyłączony
    - IP: `currentIP/bt`
  - Ekran statusu 3 sekundy

### Dostęp do WebUI
- **GLOS** (`rcCmdGLOS`)
  - Otwiera stronę BT w przeglądarce
  - Wyświetla URL: `http://IP/bt` na OLED
  - Przez 3 sekundy, potem powrót do radia

- **POWROT** (`rcCmdPOWROT`)
  - Alternatywny dostęp do strony BT
  - Identyczna funkcja jak GLOS
  - Wyświetla "BT WebUI: IP/bt"

- **BT** (`rcCmdBT`) - **W Radio**
  - Otwiera stronę BT w przeglądarce
  - Wyświetla "Bluetooth: IP/bt"
  - **W SDPlayer:** Zmiana stylu

### Funkcje Eksperymentalne
- **REDLEFT** (`rcCmdREDLEFT`)
  - **Restart modułu BT** (placeholder)
  - Wyświetla "BT: Restart..." → "BT: Gotowy"
  - Obecnie brak faktycznej implementacji restartu
  - Przygotowane do przyszłej rozbudowy

- **GREENL** (`rcCmdGREENL`)
  - **Test połączenia BT** (eksperymentalne)
  - Wyświetla status: "Modul OK" / "Brak modulu"
  - Czas wyświetlania: 2.5s

---

## 🎚️ Equalizer

### Aktywacja
- **AUD** (`rcCmdAud`)
  - **Pojedyncze kliknięcie:** Otwiera Equalizer
  - **Podwójne kliknięcie (600ms):** Aktywuje SDPlayer
  - Tryb wyboru: EQ3 lub EQ16 (jeśli `eq16ModeSelectActive`)

### Equalizer 3-Pasmowy
- **Parametry:** Low / Mid / High
- **Zakres:** -40 dB do +6 dB (w `audio.setTone()`)
- **Nawigacja:**
  - **↑/↓:** Przełączanie między parametrami (1→2→3)
  - **←/→:** Zmniejsz/Zwiększ wartość parametru
  - **OK:** Zapisz do `/eq.txt`
  - **DIRECT:** Reset wszystkich do 0

### Equalizer 16-Pasmowy (EQ16)
- **Aktywacja:** Przycisk **0** lub **PIP**
- **Parametry:** 16 pasm (0-15)
- **Zakres:** -16 dB do +16 dB
- **Nawigacja:**
  - **↑:** Zwiększ wzmocnienie pasma (+1)
  - **↓:** Zmniejsz wzmocnienie pasma (-1)
  - **←/→:** Poprzednie/Następne pasmo
  - **OK:** Potwierdź i zastosuj (exit select mode)
  - **BACK:** Wyjście bez zapisu

### Przełączanie EQ3 ↔ EQ16
- **PIP** (`rcCmdPIP`)
  - Przełączenie między EQ3 a EQ16
  - Wyświetla "EQ 3-BAND" lub "EQ 16-BAND" przez 1.5s
  - Wymaga: `#define ENABLE_EQ16 1` w kompilacji

- **KEY 0** (`rcCmdKey0`) - w Equalizer
  - Alternatywny toggle EQ3 ↔ EQ16
  - Identyczna funkcja jak PIP

---

## 🔴 Nagrywanie

### Nagrywanie Strumienia Radiowego
- **REC** (`rcCmdREC`)
  - **Toggle START/STOP** nagrywania
  - Format: MP3 (z dekodowaniem do WAV + enkoding)
  - Katalog: `/recordings/`
  - Nazwa pliku: `YYYYMMDD_HHMMSS_StationName.mp3`
  - Status wyświetlany na OLED:
    - "NAGRYWANIE" + nazwa stacji
    - Czas trwania (MM:SS)
    - Rozmiar pliku (KB/MB)

- **BLUE** (`rcCmdBlue`)
  - Alias dla REC
  - Identyczna funkcja
  - Toggle START/STOP nagrywania

- **DIRECT** (`rcCmdDirect`) - w Radio
  - Alternatywny przycisk nagrywania
  - Priorytet: Nagrywanie > inne funkcje
  - Identyczna funkcja jak REC/BLUE

### Informacje o Nagrywaniu
- Wyświetlane w czasie rzeczywistym:
  - Nazwa pliku
  - Czas nagrywania (format: MM:SS)
  - Rozmiar pliku (KB → MB po przekroczeniu 1024 KB)
  - Ikona REC (mrugająca co 500ms)
- Pozycja na ekranie: Linia 1 (Mode 0, 2, 3, 8)
- Automatyczne zatrzymanie przez **BACK** lub **EXIT**

---

## 🕐 Timer i Funkcje Specjalne

### Sleep Timer
- **GREEN** (`rcCmdGreen`) - **Poza listą stacji**
  - Ustawienie sleep timera
  - Zakres: 0, 15, 30, 45, 60, 75, 90 minut
  - Krok: 15 minut
  - Wyświetlanie: "SLEEP X MINUTES" lub "SLEEP OFF"
  - Po upływie czasu: Fade out → Power OFF

- **GREEN** (`rcCmdGreen`) - **W liście stacji**
  - Odtwórz głosowy czas z pliku `/voice/time_XX.mp3`
  - Format: Godzina:Minuta (np. "Godzina dwunasta piętnaście")
  - Auto-play co pełną godzinę (jeśli `timeVoiceInfoEveryHour == true`)

### Analyzer Spektrum
- **YELLOW** (`rcCmdYellow`)
  - **Toggle Analyzer ON/OFF**
  - Włącza/wyłącza analizator FFT
  - Style z analyzerem: 6, 10
  - Globalna flaga: `analyzerEnabled`
  - Presety: Classic, Modern, Compact, Retro, Custom

---

## 🚫 Przyciski Zarezerwowane

Następujące przyciski są zarezerwowane na przyszłe funkcje i obecnie nie mają przypisanej akcji:

- **TV** (`rcCmdTV`) - Zarezerwowane
- **WWW** (`rcCmdWWW`) - Zarezerwowane
- **GAZE** (`rcCmdGAZE`) - Zarezerwowane
- **EPG** (`rcCmdEPG`) - Zarezerwowane

**W SDPlayer:** Te przyciski są blokowane i nie wywołują żadnej akcji.

---

## 📊 Tabela Szybkiego Dostępu

| Przycisk | Radio | SDPlayer | Menu Bank | Equalizer |
|----------|-------|----------|-----------|-----------|
| **VOL+** | Głośniej | Głośniej | Głośniej | Głośniej |
| **VOL-** | Ciszej | Ciszej | Ciszej | Ciszej |
| **→** | Następna stacja | - | Bank+1 | Parametr+ |
| **←** | Poprzednia stacja | - | Bank-1 | Parametr- |
| **↑** | Lista w górę | Lista w górę | - | Poprzedni param |
| **↓** | Lista w dół | Lista w dół | - | Następny param |
| **OK** | Zagraj stację | Zagraj utwór | Załaduj bank | Zapisz EQ |
| **BACK** | Główny ekran | - | Anuluj | Wyjdź |
| **MUTE** | Toggle mute | Toggle mute | - | - |
| **SRC** | Tryb display | - | - | - |
| **POWER** | Power OFF | Power OFF | - | - |
| **1-9** | Wybór stacji | Wybór utworu | - | Zablokowane |
| **0** | Wybór (0) | Wybór (0) | - | Toggle EQ3↔16 |
| **BANK+** | Menu banków | - | Bank+1 | - |
| **BANK-** | Menu banków | - | Bank-1 | - |
| **AUD** | Equalizer | - | - | [Aktywny] |
| **AUD×2** | **SDPlayer** | - | - | - |
| **HELP** | Toggle SDPlayer | **Dezaktywacja** | - | - |
| **PIP** | Toggle EQ3↔16 | - | - | Toggle EQ3↔16 |
| **PLAY** | Blokowany | Play/Pause | - | - |
| **PAUSE** | Blokowany | Play/Pause | - | - |
| **STOP** | Blokowany | Stop | - | - |
| **REV** | Blokowany | -10s / Prev | - | - |
| **RER** | Blokowany | +10s / Next | - | - |
| **FILES** | Blokowany | Lista plików | - | - |
| **MENU** | Blokowany | Lista plików | - | - |
| **EXIT** | - | Wyjście | - | - |
| **RADIO** | - | Wyjście | - | - |
| **REC** | Toggle nagrywania | - | - | - |
| **BLUE** | Toggle nagrywania | - | - | - |
| **DIRECT** | Nagrywanie/Dimmer | - | GitHub↔SD | Reset EQ |
| **YELLOW** | Toggle Analyzer | - | - | - |
| **GREEN** | Sleep Timer | - | - | - |
| **KOL** | BT ON/OFF | - | - | - |
| **CHAT** | Status BT | - | - | - |
| **INFO** | Info BT | - | - | - |
| **GLOS** | Strona BT | - | - | - |
| **POWROT** | Strona BT | - | - | - |
| **BT** | Strona BT | Zmiana stylu | - | - |
| **CH+** | Jasność+20 | Jasność+20 | - | - |
| **CH-** | Jasność-20 | Jasność-20 | - | - |
| **REDLEFT** | Restart BT | - | - | - |
| **GREENL** | Test BT | - | - | - |
| **MINIMA** | - | Następny styl | - | - |
| **MAXIMA** | - | Zmiana stylu | - | - |
| **WELEMENT** | - | Pause/Play | - | - |
| **WINPOD** | - | Stop | - | - |

---

## 🎯 Combo i Funkcje Specjalne

### Podwójne Kliknięcia
1. **AUD × 2** (600ms)
   - Aktywacja SDPlayer
   - Zapamiętuje pozycję radia

2. **REV × 2** (600ms) - w SDPlayer
   - Poprzedni utwór
   - Jeśli brak poprzedniego: zostaje na pierwszym

3. **RER × 2** (600ms) - w SDPlayer
   - Następny utwór
   - Jeśli brak następnego: zostaje na ostatnim

### Wielocyfrowy Wybór Stacji/Utworu
- **Timeout:** 3 sekundy (`IR_INPUT_TIMEOUT_MS`)
- **Buffer:** `irInputBuffer` (0-100)
- **Przykłady:**
  - `2` → Stacja/Utwór #2
  - `2` + `5` (w 3s) → Stacja/Utwór #25
  - `9` + `9` (w 3s) → Stacja/Utwór #99
  - `1` + `0` + `0` (w 3s) → Utwór #100 (tylko SDPlayer)

### Triple Zero (000)
- **Kombinacja:** `0` + `0` + `0` (w 3s)
- **Funkcja:** Specjalna akcja (obecnie placeholder)
- **Możliwe zastosowanie:** Reset ustawień / Easter egg

---

## 🔧 Konfiguracja i Ustawienia

### Źródła Banków Stacji
- **GitHub:** `https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bankXX.txt`
- **Karta SD:** `/bankXX.txt`
- Przełączanie: **DIRECT** w Menu Bank
- Flaga: `bankNetworkUpdate` (true = GitHub, false = SD)

### Pliki Konfiguracyjne
- `/config.txt` - Główna konfiguracja (45 parametrów)
- `/remote.txt` - Kody pilota IR (30 przycisków)
- `/eq.txt` - Ustawienia Equalizer 3-band
- `/eq16.txt` - Ustawienia Equalizer 16-band
- `/volume.txt` - Ostatni poziom głośności
- `/station.txt` - Ostatni bank i numer stacji
- `/sdplayer_session.txt` - Sesja SDPlayer (start/end index)
- `/sdplayer_active.txt` - Stan aktywności SDPlayer

### Zakres Wartości
| Parametr | Min | Max | Domyślnie |
|----------|-----|-----|-----------|
| Volume | 1 (0*) | 21 (42*) | 10 |
| Bank nr | 1 | 17 | 1 |
| Station nr | 1 | 99 | 1 |
| Display Mode | 0 | 10 | 6 |
| Display Brightness | 10 | 255 | 180 |
| Sleep Timer | 0 | 90 | 0 (OFF) |
| EQ 3-band | -40 dB | +6 dB | 0 |
| EQ 16-band | -16 dB | +16 dB | 0 |

\* - Zakres rozszerzony (jeśli `maxVolumeExt = true`)

---

## 📝 Notatki Techniczne

### Flagi Globalne
```cpp
bool sdPlayerOLEDActive;        // SDPlayer aktywny
bool sdPlayerPlayingMusic;      // SDPlayer odtwarza muzykę
bool listedStations;            // Wyświetlona lista stacji
bool listedTracks;              // Wyświetlona lista utworów
bool bankMenuEnable;            // Menu wyboru banku aktywne
bool equalizerMenuEnable;       // Menu equalizera aktywne
bool eq16Mode;                  // EQ16 (true) lub EQ3 (false)
bool volumeMute;                // Wyciszenie aktywne
bool btModuleEnabled;           // Moduł BT włączony
bool analyzerEnabled;           // Analyzer aktywny
bool displayDimmerActive;       // Dimmer OLED aktywny
bool f_sleepTimerOn;            // Sleep timer aktywny
bool waitingForSecondClick;     // Oczekiwanie na double-click
bool waitingForSecondAudClick;  // Oczekiwanie na double-click AUD
bool irInputActive;             // Wielocyfrowe wprowadzanie aktywne
```

### Routing IR w SDPlayer
Gdy `sdPlayerOLEDActive == true`:

**Blokowane przyciski:**
- Wszystkie kody IR **poza**:
  - 0-9 (wybór utworu)
  - ↑/↓/←/→ (Equalizer)
  - POWER, RED (Power OFF)
  - BACK (wyjście do folderu nadrzędnego)
  - Przyciski SDPlayer (PLAY, PAUSE, STOP, REV, RER, FILES, MENU, EXIT, RADIO)

**Routing blokady:**
```cpp
if (sdPlayerOLEDActive) {
  // Routing specjalny dla SDPlayer
  // Inne przyciski: ir_code = 0; return;
}
```

### Timeout i Debouncing
- **IR debouncing:** 300ms (`debounceDelay`)
- **Wielocyfrowe wprowadzanie:** 3000ms (`IR_INPUT_TIMEOUT_MS`)
- **Double-click:** 600ms (`doubleClickTimeout`)
- **Display timeout:** 8000ms (`displayTimeout`)

---

## 🚀 Szybki Start

### Podstawowa Obsługa Radia
1. **Power ON** - Przycisk **POWER** (long press z wyłączonego stanu)
2. **Wybór banku** - **BANK+** → wybierz → **OK**
3. **Wybór stacji** - **↑/↓** lub **cyfry 1-99** → **OK**
4. **Regulacja głośności** - **VOL+** / **VOL-**
5. **Zmiana stylu** - **SRC** (0→1→2→3→4→6→8→10)

### SDPlayer - Szybki Start
1. **Aktywacja** - **HELP** lub **AUD × 2**
2. **Lista utworów** - **FILES**
3. **Wybór utworu** - **↑/↓** lub **cyfry 1-100** → **OK**
4. **Play/Pause** - **PLAY** / **PAUSE**
5. **Przewijanie** - **REV** (-10s) / **RER** (+10s)
6. **Double-click** - **REV × 2** (prev) / **RER × 2** (next)
7. **Wyjście** - **EXIT** lub **RADIO**

### Moduł Bluetooth
1. **Włączenie modułu** - **KOL** (toggle ON)
2. **Status** - **CHAT** (wyświetla IP i status)
3. **Otwórz WebUI** - **GLOS** lub **BT**
4. **Informacje** - **INFO**
5. **Wyłączenie** - **KOL** (toggle OFF)

### Equalizer
1. **Otwórz EQ** - **AUD**
2. **Wybór trybu** - **0** lub **PIP** (EQ3 ↔ EQ16)
3. **EQ3:**
   - **↑/↓** - Wybór Low/Mid/High
   - **←/→** - Zmiana wartości
4. **EQ16:**
   - **←/→** - Wybór pasma (0-15)
   - **↑/↓** - Zmiana wzmocnienia
5. **Zapisz** - **OK**
6. **Reset** - **DIRECT**

---

## 📞 Wsparcie i Dokumentacja

- **GitHub:** https://github.com/dzikakuna/ESP32_radio_evo3
- **Wersja:** v3.19.70
- **Data kompilacji:** 2 marca 2026
- **Autor:** Robgold 2026, Made in Poland

---

**✅ KONIEC DOKUMENTACJI**

*Wszystkie 32 przyciski IR + funkcje combo i specjalne zostały opisane.*
