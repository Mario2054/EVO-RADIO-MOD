# Instrukcja Aktywacji Rozszerzonego SD Playera

## Tryb Advanced - z ikonami i ID3 tags

System SD Playera ma teraz **DWA TRYBY**:
- **WebUI mode** - prosty odtwarzacz z listą plików w przeglądarce
- **Advanced mode** - zaawansowany z ikonami ▶️⏸️⏹️, ID3 tags (Artist/Title/Album), auto-play

## Jak uruchomić SD Player?

### Metoda 1: Enkoder (TRIPLE-CLICK)
1. **Kliknij 3 razy szybko** przycisk enkodera (w ciągu 600ms)
2. Na Serial Monitor zobaczysz:
   ```
   [SDPlayerManager] Aktywny w trybie: ADVANCED
   Advanced mode - Full-featured player with ID3, auto-play, timer
   ```
3. Na OLED pojawi się panel z ikonami

### Metoda 2: Pilot IR (Przycisk 9)
1. Naciśnij **przycisk 9** na pilocie (lub KEY9 jeśli skonfigurowany)
2. System aktywuje się w trybie Advanced

## Jak rozpoznać że tryb Advanced działa?

### Na ekranie OLED zobaczysz:
- **Ikonę statusu** (▶️ PLAY / ⏸️ PAUSE / ⏹️ STOP) w lewym górnym rogu
- **Artist** - nazwisko wykonawcy (z ID3 taga)
- **Volume** - poziom głośności w prawym górnym rogu (0-21)
- **🎵 Title** - tytuł utworu (z ID3 lub nazwa pliku)
- **Album** - album (z ID3 taga)
- **Parametry audio**:
  - Codec (MP3/FLAC/AAC/OPUS/VORBIS)
  - Bitrate (np. 320 kbps)
  - SampleRate (np. 44.1 kHz)
  - BitsPerSample (np. 16 bit)
- **Timer odtwarzania** (mm:ss) - czas od rozpoczęcia utworu
- **🔁 Repeat mode icon** + tekst (OFF/ONE/ALL/FOLDER)

### Na Serial Monitor:
```
[SDPlayerManager] Aktywny w trybie: ADVANCED
Advanced mode - Full-featured player with ID3, auto-play, timer
```

## Sterowanie w trybie Advanced

### Enkoder:
- **Obrót w lewo/prawo** - Nawigacja po liście utworów
- **Krótkie kliknięcie** - PLAY/PAUSE wybranego utworu
- **Długie przytrzymanie (4s)** - Wyjście do radia

### Pilot IR:
- **OK** - PLAY/PAUSE
- **BACK (2x szybko)** - Wyjście do radia
- **BACK (1x)** - STOP
- **Key0** - Wyjście do radia (Bank 1, Stacja 4)
- **VOL+/VOL-** - Głośność

## Jak przełączyć tryb na WebUI?

### Metoda 1: WebUI w przeglądarce
1. Otwórz http://evoradio.local/sdplayer
2. Kliknij niebieski przycisk **🔄 Switch Mode: Advanced**
3. Tryb zmieni się na **WebUI**

### Metoda 2: Pilot IR (YELLOW button - opcjonalnie)
1. Jeśli masz zmapowany przycisk YELLOW na pilocie:
   - Naciśnij **YELLOW button**
2. System przełączy tryb i pokaże komunikat:
   ```
   [SDPlayerManager] Mode switched: WEBUI -> ADVANCED
   ```

## Funkcje tylko w trybie Advanced

### Auto-Play
- Po zakończeniu utworu automatycznie odtwarza kolejny
- Respektuje ustawienia Repeat Mode

### Repeat Modes (przełączanie przez WebUI):
- **OFF** - Odtwórz tylko wybrany utwór
- **ONE** - Powtarzaj ten sam utwór
- **ALL** - Powtarzaj całą listę
- **FOLDER** - Powtarzaj pliki z tego samego folderu

### ID3 Tag Parsing:
- Automatyczne rozpoznawanie Artist, Title, Album z metadanych MP3
- Fallback do nazwy pliku jeśli brak ID3

### Smart Sorting:
- Numeryczne sortowanie plików (01_song.mp3 przed 02_song.mp3)
- Obsługa polskich znaków (Ą, Ć, Ę, Ł, Ń, Ó, Ś, Ź, Ż)

### Timer odtwarzania:
- Wyświetla czas od rozpoczęcia utworu w formacie mm:ss
- Aktualizacja co 100ms

## Troubleshooting

⚠️ **Jeśli panel Advanced nie wyświetla się**, zobacz szczegółową instrukcję debugowania:  
📄 **[DEBUG_SDPLAYER_ADVANCED.md](DEBUG_SDPLAYER_ADVANCED.md)** - Kompletny przewodnik z komunikatami Serial Monitor

### Nie widzę ikon na OLED
1. Sprawdź czy SD Player jest w trybie **ADVANCED** (Serial Monitor)
2. Jeśli pokazuje "WEBUI", przełącz przez WebUI button

### Brak ID3 tags (tylko nazwa pliku)
1. Twoje pliki MP3 mogą nie mieć ID3 metadanych
2. System automatycznie użyje nazwy pliku jako tytuł

### Enkoder nie reaguje
1. Sprawdź czy SDPlayerManager jest aktywny (`sdPlayerOLEDActive = true`)
2. W Serial Monitor powinieneś widzieć: `[SDPlayerManager] Aktywny w trybie: ADVANCED`

### Jak wyjść z SD Playera?
- **Długie przytrzymanie enkodera (4s)**
- **Podwójne kliknięcie BACK na pilocie**
- **Key0 na pilocie**

## Wgrywanie firmware

```powershell
cd "c:\YO RADIO\EVO RADIO\src\ESP32_radio_evo3.19\Platformio"
C:\Users\Mariu\.platformio\penv\Scripts\platformio.exe run --target upload
```

## Debugowanie przez Serial Monitor

```powershell
pio device monitor --baud 115200
```

Szukaj komunikatów:
- `[SDPlayerManager] Aktywny w trybie: ADVANCED`
- `[SDPlayerAdvanced] Playing: /path/to/file.mp3`
- `[ID3] Artist: ...`
- `[ID3] Title: ...`

---

**Gotowe!** Ciesz się rozszerzonym SD Playerem z ikonami i ID3 tags 🎵
