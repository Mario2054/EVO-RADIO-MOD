# Instrukcja integracji SD Player do main.cpp

## Przygotowane pliki modułowe:

1. **SDPlayer.h / SDPlayer.cpp** - Klasa podstawowa (logika, nawigacja, JSON API)
2. **SDPlayer_OLED.h** - Renderowanie OLED (splash, scrolling, lista)
3. **SDPlayer_IR.h** - Obsługa IR (combo 987, nawigacja)
4. **SDPlayer_Web.h** - Handlery AsyncWebServer
5. **sdplayer_page.h** - Strona HTML

## Minimalne zmiany w main.cpp:

### 1. Dodaj include na górze pliku (po istniejących includes):

```cpp
#include "SDPlayer.h"
#include "SDPlayer_OLED.h"
#include "SDPlayer_IR.h"
#include "SDPlayer_Web.h"
```

### 2. Dodaj globalne obiekty (przy innych globalnych):

```cpp
SDPlayer sdPlayer;
SDPlayer_OLED sdPlayerOLED;
SDPlayer_IR sdPlayerIR;
SDPlayer_Web sdPlayerWeb;
```

### 3. W setup() po inicjalizacji Audio i STORAGE:

```cpp
// Inicjalizacja SD Player
sdPlayer.begin(&STORAGE, &audio);
sdPlayerOLED.begin(&sdPlayer, &u8g2);
sdPlayerIR.begin(&sdPlayer);
sdPlayerWeb.begin(&sdPlayer, &server);  // gdzie 'server' to AsyncWebServer
```

### 4. W loop() dodaj (przed lub po istniejącej logice OLED):

```cpp
// SD Player rendering
if (sdPlayer.isActive()) {
  sdPlayerOLED.render();
} else {
  // Tutaj istniejąca logika renderowania radia
}
```

### 5. W obsłudze IR (gdzie przetwarzasz kody z pilota):

#### a) Dla cyfr (0-9):
```cpp
if (irCode >= 0 && irCode <= 9) {  // przykład - dostosuj do swoich kodów
  sdPlayerIR.feedDigit(irCode);
}
```

#### b) Dla klawiszy nawigacyjnych (UP/DOWN/OK/BACK):
```cpp
// Przykład - dostosuj kody IR do swojego pilota
if (sdPlayerIR.handleKey(irCode)) {
  return; // SD Player obsłużył - nie przetwarzaj dalej w radio
}

// Tutaj istniejąca logika IR dla radia
```

### 6. OPCJONALNIE - Pauza radia gdy SD Player aktywny:

W loop() można dodać:

```cpp
if (sdPlayer.isActive() && !audio.isRunning()) {
  // Radio zatrzymane, SD Player przejął kontrolę
  // Można tu dodać logikę zapamiętywania stanu radia
}

if (!sdPlayer.isActive() && previouslyWasActive) {
  // Powrót z SD Playera do radia
  // Wznów odtwarzanie radia jeśli było aktywne
  previouslyWasActive = false;
}
```

## Mapowanie kodów IR:

W pliku `SDPlayer_IR.h` dostosuj kody IR do swojego pilota:

```cpp
case 0xFF18E7:  // UP - ZMIEŃ na swój kod
case 0xFF4AB5:  // DOWN - ZMIEŃ na swój kod
case 0xFF38C7:  // OK - ZMIEŃ na swój kod
case 0xFF22DD:  // BACK - ZMIEŃ na swój kod
case 0xFF906F:  // VOL+ - ZMIEŃ na swój kod
case 0xFFA857:  // VOL- - ZMIEŃ na swój kod
```

## Działanie:

1. Wciśnij **987** na pilocie → wejście do SD Player
2. **UP/DOWN** → nawigacja po plikach
3. **OK** → odtwórz plik lub wejdź do folderu
4. **BACK** → katalog wyżej lub powrót do radia
5. **VOL+/VOL-** → zmiana głośności (0-21)
6. Strona web: http://IP/sdplayer
7. **Volume**: kontrola przez web UI, pilot lub metodę `sdPlayer.setVolume(vol)`

## Kontrola głośności:

SD Player ma własną kontrolę volume (zakres 0-21):
- Web UI: suwak volume na stronie /sdplayer
- Programowo: `sdPlayer.setVolume(15);` lub `uint8_t vol = sdPlayer.getVolume();`
- Volume jest automatycznie ustawiane przy każdym odtwarzaniu pliku
- Volume jest pamiętane między utworami

## Uwagi:

- SDPlayer używa tego samego obiektu `audio` co radio
- Pliki modułowe są niezależne - łatwo wyłączyć/włączyć funkcje
- OLED pokazuje: splash "SD PLAYER" (900ms), scrolling nazwy utworu, separator, lista plików
- Wszystkie zmiany w main.cpp to ~10-15 linii kodu
