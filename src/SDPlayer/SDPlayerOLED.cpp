#include "SDPlayerOLED.h"
#include "SDPlayerWebUI.h"
#include "EQ_FFTAnalyzer.h"
#include <SD.h>
#include "Audio.h"  // Importuj definicję klasy Audio

// Extern zmienne z main.cpp do zarządzania trybem odtwarzania
extern bool sdPlayerPlayingMusic;
extern bool sdPlayerOLEDActive;
extern bool equalizerMenuEnable;  // Flaga aktywnego menu equalizera
extern bool listedTracks;          // Flaga pokazywania listy utworów z main.cpp

// Forward declaration funkcji z main.cpp
extern void displayRadio();
extern void volumeUp();      // Funkcja zwiększania głośności z main.cpp
extern void volumeDown();    // Funkcja zmniejszania głośności z main.cpp
extern void wsVolumeChange(); // Funkcja synchronizacji głośności przez WebSocket
extern void changeStation();  // Funkcja zmiany stacji radiowej
extern U8G2 u8g2;

// Extern zmienne stacji dla powrotu do radia
extern uint8_t sdPlayerReturnBank;     // Bank do powrotu po wyjściu z SDPlayera
extern uint8_t sdPlayerReturnStation;  // Stacja do powrotu po wyjściu z SDPlayera
extern uint8_t bank_nr;                // Aktualny bank radia
extern uint8_t station_nr;             // Aktualna stacja radia

// Extern czcionki i zmienne audio z main.cpp
extern uint8_t spleen6x12PL[];
extern String streamCodec;
extern String bitrateString;
extern String bitsPerSampleString;
extern uint32_t SampleRate;
extern uint8_t SampleRateRest;

// Extern metadane ID3 z main.cpp
extern String currentMP3Artist;
extern String currentMP3Title;
extern String currentMP3Album;

// Extern obiekt Audio dla pobierania czasu odtwarzania
extern Audio audio;

// Extern zmienne głośności z main.cpp dla synchronizacji z radiem
extern uint8_t volumeValue;
extern uint8_t maxVolume;
extern bool volumeMute;

// Extern zmienne konfiguracji stylów SDPlayera z main.cpp
extern bool sdPlayerStyle1Enabled;
extern bool sdPlayerStyle2Enabled;
extern bool sdPlayerStyle3Enabled;
extern bool sdPlayerStyle4Enabled;
extern bool sdPlayerStyle5Enabled;
extern bool sdPlayerStyle6Enabled;
extern bool sdPlayerStyle7Enabled;
extern bool sdPlayerStyle9Enabled;
extern bool sdPlayerStyle10Enabled;
extern bool sdPlayerStyle11Enabled;
extern bool sdPlayerStyle12Enabled;
extern bool sdPlayerStyle13Enabled;
extern bool sdPlayerStyle14Enabled;

// Ikony dla STYLE_8 (layout jak SDPlayerAdvanced)
static const unsigned char style8_icon_play_bits[] PROGMEM = {
    0x00, 0x02, 0x06, 0x0E, 0x1E, 0x0E, 0x06, 0x02
};
static const unsigned char style8_icon_pause_bits[] PROGMEM = {
    0x00, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x00
};
static const unsigned char style8_icon_stop_bits[] PROGMEM = {
    0x00, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x00
};
static const unsigned char style8_icon_repeat_bits[] PROGMEM = {
    0x3C, 0x42, 0x81, 0x99, 0x99, 0x81, 0x42, 0x3C
};
static const unsigned char style8_icon_music_bits[] PROGMEM = {
    0x18, 0x18, 0x18, 0x18, 0x1B, 0x3F, 0x3E, 0x1C
};
static const unsigned char style8_icon_speaker_bits[] PROGMEM = {
    0x08, 0x0C, 0x2E, 0x2F, 0x2F, 0x2E, 0x0C, 0x08
};
static const unsigned char style8_icon_speaker_muted_bits[] PROGMEM = {
    0x88, 0x4C, 0x2E, 0x1F, 0x8F, 0xEE, 0x4C, 0x88
};

static void drawStyle8BitmapIcon(U8G2& display, int x, int y, const unsigned char* bitmap) {
    // Rysowanie bitmapy 8x8 wprost piksel po pikselu (XBM: LSB-first)
    for (int row = 0; row < 8; row++) {
        uint8_t bits = bitmap[row];
        for (int col = 0; col < 8; col++) {
            if (bits & (1 << col)) {
                display.drawPixel(x + col, y + row);
            }
        }
    }
}

SDPlayerOLED::SDPlayerOLED(U8G2& display) 
    : _display(display),
      _player(nullptr),
      _active(false),
      _oledIndexChangeCallback(nullptr),
      _style(STYLE_1),
      _infoStyle(INFO_CLOCK_DATE),
      _mode(MODE_NORMAL),
      _selectedIndex(0),
      _scrollOffset(0),
      _splashStartTime(0),
      _volumeShowTime(0),
      _lastUpdate(0),
      _lastSrcPressTime(0),
      _srcClickCount(0),
      _scrollPosition(0),
      _animFrame(0),
      _scrollTextOffset(0),
      _lastScrollTime(0),
      _lastEncoderClickTime(0),
      _encoderClickCount(0),
      _encoderButtonPressStart(0),
      _encoderButtonPressed(false),
      _encoderVolumeMode(false),
      _lastEncoderModeChange(0),
      _actionMessage(""),
      _actionMessageTime(0),
      _showActionMessage(false) {
}

void SDPlayerOLED::begin(SDPlayerWebUI* player) {
    _player = player;
    _display.begin();
    _display.setFont(spleen6x12PL);
}

void SDPlayerOLED::activate() {
    _active = true;
    
    // PUNKT 5: STOP RADIO STREAM przed aktywacją SD Playera
    extern Audio audio;
    audio.stopSong();
    Serial.println("SD Player: Radio stream stopped");
    
    // SYNCHRONIZACJA VOLUME: Skopiuj głośność z radia do SD Playera
    if (_player) {
        _player->setVolume(volumeValue);
        Serial.printf("SD Player: Volume synchronized from radio: %d\n", volumeValue);
    }
    
    // KRYTYCZNE: Pełny reset stanu przy aktywacji
    _mode = MODE_SPLASH;
    _splashStartTime = millis();
    _selectedIndex = 0;
    _scrollOffset = 0;
    _scrollPosition = 0;
    _animFrame = 0;
    _scrollTextOffset = 0;
    
    // Reset liczników kliknięć
    _srcClickCount = 0;
    _encoderClickCount = 0;
    _lastEncoderClickTime = 0;
    _encoderButtonPressed = false;
    
    // KRYTYCZNE: Enkoder zawsze startuje w trybie NAWIGACJI (nie volume)
    _encoderVolumeMode = false;
    _lastEncoderModeChange = millis();
    
    // LAZY LOADING: Wyczyść listę plików i uruchom skanowanie w loop()
    _fileList.clear();
    extern bool sdPlayerScanRequested;
    sdPlayerScanRequested = true;
    
    // PUNKT 3: PERSISTENT STATE - zapisz że SD Player jest aktywny
    extern const char* sdPlayerActiveStateFile;
    File stateFile = SD.open(sdPlayerActiveStateFile, FILE_WRITE);
    if (stateFile) {
        stateFile.println("1");
        stateFile.close();
        Serial.println("SD Player: State saved (active=1)");
    }
    
    Serial.println("SD Player: Activated with full state reset (auto-loading file list)");
}

void SDPlayerOLED::deactivate() {
    _active = false;
    
    // KRYTYCZNE: Reset globalnej flagi SDPlayer
    extern bool sdPlayerOLEDActive;
    sdPlayerOLEDActive = false;
    Serial.println("SDPlayerOLED: sdPlayerOLEDActive set to FALSE in deactivate()");
    
    // SYNCHRONIZACJA VOLUME: Skopiuj głośność z SD Playera z powrotem do radia
    if (_player) {
        volumeValue = _player->getVolume();
        audio.setVolume(volumeValue);
        Serial.printf("SD Player: Volume synchronized back to radio: %d\n", volumeValue);
    }
    
    // PUNKT 3: PERSISTENT STATE - zapisz że SD Player jest nieaktywny
    extern const char* sdPlayerActiveStateFile;
    File stateFile = SD.open(sdPlayerActiveStateFile, FILE_WRITE);
    if (stateFile) {
        stateFile.println("0");
        stateFile.close();
        Serial.println("SD Player: State saved (active=0)");
    }
    
    // WYMUSZENIE: Przywróć tryby radia
    extern bool displayActive;
    extern bool timeDisplay;
    displayActive = true;
    timeDisplay = true;
    
    // Reset stanu przy deaktywacji
    _mode = MODE_NORMAL;
    _selectedIndex = 0;
    _scrollOffset = 0;
    _scrollPosition = 0;
    
    _display.clearBuffer();
    _display.sendBuffer();
    
    Serial.println("SD Player: Deactivated with state reset and radio modes restored");
}

void SDPlayerOLED::showSplash() {
    _mode = MODE_SPLASH;
    _splashStartTime = millis();
}

void SDPlayerOLED::loop() {
    if (!_active) return;
    
    // KRYTYCZNE: Jeśli equalizer lub lista utworów jest aktywna, nie odświeżaj ekranu!
    // Pozwala to main.cpp kontrolować wyświetlanie bez konfliktów
    if (equalizerMenuEnable || listedTracks) {
        return;  // Nie renderuj - equalizer lub lista z main.cpp ma kontrolę nad ekranem
    }
    
    unsigned long now = millis();
    
    // Synchronizuj _selectedIndex z SDPlayerWebUI TYLKO gdy gramy (dla auto-play)
    // NIE podczas ręcznej nawigacji - inaczej pilot nie działa!
    if (_player && _mode == MODE_NORMAL && _player->isPlaying()) {
        int webIndex = _player->getSelectedIndex();
        if (webIndex != _selectedIndex && webIndex >= 0 && webIndex < _fileList.size()) {
            _selectedIndex = webIndex;
            // Dostosuj scroll offset aby kursor był widoczny
            int visibleLines = 4;  // Liczba widocznych linii na ekranie
            if (_selectedIndex < _scrollOffset) {
                _scrollOffset = _selectedIndex;
            }
            if (_selectedIndex >= _scrollOffset + visibleLines) {
                _scrollOffset = _selectedIndex - visibleLines + 1;
            }
        }
    }
    
    // Splash screen przez 1.5s
    if (_mode == MODE_SPLASH) {
        if (now - _splashStartTime > 1500) {
            _mode = MODE_NORMAL;
            // NIE ładuj listy po splash - lazy loading przy pierwszym użyciu
        }
    }
    
    // Volume pokazuje się przez 2s
    if (_mode == MODE_VOLUME) {
        if (now - _volumeShowTime > 2000) {
            _mode = MODE_NORMAL;
        }
    }
    
    // Automatyczny powrót do trybu nawigacji po 6 sekundach bezczynności (jak w radiu)
    if (_encoderVolumeMode && (now - _lastEncoderModeChange > 6000)) {
        _encoderVolumeMode = false;
        Serial.println("SD Player: Auto-return to NAVIGATION mode after timeout");
    }
    
    // Wykonaj pojedyncze kliknięcie po timeout (650ms) jeśli nie było kolejnych kliknięć
    if (_encoderClickCount == 1 && (now - _lastEncoderClickTime > 650)) {
        // POJEDYNCZE KLIKNIĘCIE - PLAY/PAUSE lub wybór utworu
        if (_player && _player->isPlaying()) {
            // Jeśli coś gra - toggle pause
            bool wasPaused = _player->isPaused();
            _player->pause();
            showActionMessage(wasPaused ? "PLAY" : "PAUSE");
            Serial.println("[SDPlayer] Single click - Toggle Play/Pause");
        } else {
            // Jeśli nic nie gra - wybierz i odtwórz utwór
            selectCurrent();
            showActionMessage("PLAY");
            Serial.println("[SDPlayer] Single click - Select and play track");
        }
        _encoderClickCount = 0; // Reset licznika
    }
    
    // Odświeżanie ekranu co 100ms (zmniejszenie częstotliwości dla stabilności)
    if (now - _lastUpdate > 100) {
        _lastUpdate = now;
        _animFrame++;
        render();
    }
}

void SDPlayerOLED::refreshFileList() {
    if (!_player) return;
    
    // KRYTYCZNE: yield() na początku aby upewnić się że async_tcp ma czas
    yield();
    vTaskDelay(1);  // Początkowe opóźnienie dla stabilności async_tcp (optymalizacja FreeRTOS)
    
    // OCHRONA PRZED FRAGMENTACJĄ PAMIĘCI: Sprawdź stan pamięci przed odświeżaniem
    size_t freeBefore = ESP.getFreeHeap();
    if (freeBefore < 15000) {
        Serial.printf("[SDPlayer] WARNING: Low memory before refresh (%u bytes), skipping\n", freeBefore);
        showActionMessage("LOW MEMORY");
        return;
    }
    
    Serial.printf("[SDPlayer] Refreshing file list, free memory: %u bytes\n", freeBefore);
    
    _fileList.clear();
    yield();  // yield() po wyczyszczeniu listy
    
    String currentDir = _player->getCurrentDirectory();
    
    File dir = SD.open(currentDir);
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        return;
    }

    int fileCount = 0;
    File entry = dir.openNextFile();
    while (entry && fileCount < 200) { // LIMIT: Maksymalnie 200 plików
        // KRYTYCZNE: yield() co 5 plików + delay aby zapobiec watchdog timeout
        if (fileCount % 5 == 0) {
            yield();  // Pozwól async_tcp i innym zadaniom na wykonanie
            delay(1);  // Mikro-opóźnienie dla stabilności async_tcp
        }
        
        FileEntry fe;
        fe.name = String(entry.name());
        
        // Usuń ścieżkę - zostaw tylko nazwę
        int lastSlash = fe.name.lastIndexOf('/');
        if (lastSlash >= 0) {
            fe.name = fe.name.substring(lastSlash + 1);
        }
        
        fe.isDir = entry.isDirectory();
        
        // Dodaj tylko katalogi i pliki audio
        if (fe.isDir || 
            fe.name.endsWith(".mp3") || fe.name.endsWith(".MP3") ||
            fe.name.endsWith(".wav") || fe.name.endsWith(".WAV") ||
            fe.name.endsWith(".flac") || fe.name.endsWith(".FLAC")) {
            _fileList.push_back(fe);
            fileCount++;
            
            // Dodatkowy yield() po każdym dodaniu pliku do listy
            if (fileCount % 3 == 0) {
                yield();
            }
        }
        
        entry.close();
        entry = dir.openNextFile();
        
        // Sprawdzenie pamięci podczas ładowania z częstszym yield()
        if (fileCount % 25 == 0) {
            size_t currentFree = ESP.getFreeHeap();
            if (currentFree < 10000) {
                Serial.printf("[SDPlayer] Memory low during refresh (%u bytes), stopping at %d files\n", currentFree, fileCount);
                break;
            }
            yield();  // Dodatkowy yield() przy sprawdzaniu pamięci
            delay(2);  // Dłuższe opóźnienie przy sprawdzaniu pamięci
        }
    }
    dir.close();
    
    if (fileCount >= 200) {
        Serial.printf("[SDPlayer] WARNING: File list truncated to 200 items for memory safety\n");
        showActionMessage("TOO MANY FILES");
    }
    
    // KRYTYCZNE: yield() + delay przed sortowaniem aby uniknąć timeout
    yield();
    delay(3);
    
    // Sortuj: foldery, potem pliki
    std::sort(_fileList.begin(), _fileList.end(), [](const FileEntry& a, const FileEntry& b) {
        if (a.isDir != b.isDir) return a.isDir;
        return a.name.compareTo(b.name) < 0;
    });
    
    // yield() + delay po sortowaniu
    yield();
    delay(2);
    
    size_t freeAfter = ESP.getFreeHeap();
    Serial.printf("[SDPlayer] File list refreshed: %d items, memory: %u -> %u bytes\n", 
                  fileCount, freeBefore, freeAfter);
    
    _selectedIndex = 0;
    _scrollOffset = 0;
    
    // Final yield() na końcu funkcji
    yield();
}

void SDPlayerOLED::render() {
    _display.clearBuffer();
    
    switch (_mode) {
        case MODE_SPLASH:
            renderSplash();
            break;
        case MODE_VOLUME:
            renderVolume();
            break;
        case MODE_NORMAL:
            switch (_style) {
                case STYLE_1: renderStyle1(); break;
                case STYLE_2: renderStyle2(); break;
                case STYLE_3: renderStyle3(); break;
                case STYLE_4: renderStyle4(); break;
                case STYLE_5: renderStyle5(); break;
                case STYLE_6: renderStyle6(); break;
                case STYLE_7: renderStyle7(); break;
                case STYLE_8: renderStyle8(); break;
                case STYLE_9: renderStyle9(); break;
                case STYLE_10: renderStyle10(); break;
                case STYLE_11: renderStyle11(); break;
                case STYLE_12: renderStyle12(); break;
                case STYLE_13: renderStyle13(); break;
                case STYLE_14: renderStyle14(); break;
            }
            // Ikonki kontroli wyłączone - teraz wbudowane w Style 1
            // drawControlIcons();
            break;
    }

    // SDPLAYER MUTE: style bez ikonki głośnika pokazują napis MUTE
    if (isMutedState()) {
        if (_style == STYLE_4 || _style == STYLE_5 || _style == STYLE_6 ||
            _style == STYLE_8 || _style == STYLE_9 || _style == STYLE_10 || _style == STYLE_11 ||
            _style == STYLE_13) {
            drawMuteOverlayTag();
        }
    }
    
    _display.sendBuffer();
}

void SDPlayerOLED::renderSplash() {
    // "SD PLAYER" wyśrodkowany na ekranie - DUŻA CZCIONKA
    _display.setFont(u8g2_font_ncenB24_tr);  // Duża czcionka 24pt (tekst angielski)
    const char* text = "SD PLAYER";
    int w = _display.getStrWidth(text);
    int x = (256 - w) / 2;  // Wyśrodkowanie na 256px szerokości
    int y = 42;             // Przesunięte trochę niżej (wysokość czcionki ~30px + ascent)
    
    _display.drawStr(x, y, text);
}

void SDPlayerOLED::renderVolume() {
    if (!_player) return;
    
    // STYL IDENTYCZNY JAK W RADIU - volumeDisplay() z main.cpp
    int vol = _player->getVolume();
    String volumeValueStr = String(vol);
    
    _display.setFont(u8g2_font_fub14_tf);  // TA SAMA czcionka jak w radiu
    _display.drawStr(65, 33, "VOLUME");
    _display.drawStr(163, 33, volumeValueStr.c_str());
    
    // Ramka dla progress bara głośności
    _display.drawRFrame(21, 42, 214, 14, 3);
    
    // Progress bar głośności - dostosowany do maxVolume
    if (maxVolume == 42 && vol > 0) {
        _display.drawRBox(23, 44, vol * 5, 10, 2);
    }
    if (maxVolume == 21 && vol > 0) {
        _display.drawRBox(23, 44, vol * 10, 10, 2);
    }
}

void SDPlayerOLED::renderStyle1() {
    // STYL 1: PODSTAWOWY - Tytuł utworu + Lista plików + Format/Volume
    
    if (!_player) return;
    
    // KRYTYCZNE: Wyczyść bufor przed rysowaniem
    _display.clearBuffer();
    
    String currentFile = _player->getCurrentFile();
    String originalFile = currentFile;  // Zachowaj dla getTrackNumber()
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "Zatrzymany";
    } else {
        // Usuń rozszerzenie
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) {
            currentFile = currentFile.substring(0, dotPos);
        }
        // Usuń ścieżkę
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) {
            currentFile = currentFile.substring(slashPos + 1);
        }
        // Dodaj numer utworu z total
        int trackNumber = getTrackNumber(originalFile);
        int totalTracks = getTotalTracks();
        if (trackNumber > 0 && totalTracks > 0) {
            currentFile = String(trackNumber) + "/" + String(totalTracks) + " " + currentFile;
        }
    }
    
    _display.setFont(spleen6x12PL);  // Większa czcionka dla tytułu
    
    // === GÓRNY PASEK - TYTUŁ UTWORU ===
    int titleMaxWidth = 180; // Zostaw miejsce na format i volume
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > titleMaxWidth) {
        // Płynne scrollowanie w prawo
        static int scrollOffset = 0;
        static unsigned long lastScrollTime = 0;
        
        if (millis() - lastScrollTime > 100) {
            scrollOffset++;
            if (scrollOffset > titleWidth + 30) scrollOffset = 0;
            lastScrollTime = millis();
        }
        
        _display.setClipWindow(2, 0, titleMaxWidth + 2, 14);
        _display.drawStr(2 - scrollOffset, 11, currentFile.c_str());
        // Powtórz tekst dla ciągłego scrollowania
        _display.drawStr(2 - scrollOffset + titleWidth + 30, 11, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        // Wyśrodkowany jeśli się mieści
        int centerX = (titleMaxWidth - titleWidth) / 2;
        _display.drawStr(2 + centerX, 11, currentFile.c_str());
    }
    
    // FORMAT AUDIO
    String audioFormat = "";
    String fullFileName = _player->getCurrentFile();
    if (fullFileName.length() > 0 && fullFileName != "None") {
        int dotPos = fullFileName.lastIndexOf('.');
        if (dotPos > 0) {
            audioFormat = fullFileName.substring(dotPos + 1);
            audioFormat.toUpperCase();
        }
    }
    
    // IKONKA GŁOŚNICZKA + VOLUME
    int vol = _player->getVolume();
    String volStr = String(vol);
    int volWidth = _display.getStrWidth(volStr.c_str());
    
    int speakerX = 256 - volWidth - 20;
    drawVolumeIcon(speakerX, 3);
    if (isMutedState()) {
        drawVolumeMuteSlash(speakerX, 3);
    }
    _display.drawStr(speakerX + 14, 11, volStr.c_str());
    
    // Format przed głośnikiem
    if (audioFormat.length() > 0) {
        int formatWidth = _display.getStrWidth(audioFormat.c_str());
        int formatX = speakerX - formatWidth - 8;
        _display.drawStr(formatX, 11, audioFormat.c_str());
    }
    
    // === CIENKA LINIA ===
    _display.drawLine(0, 14, 256, 14);
    
    // === INFORMACJE O UTWORZE (wypełnienie wolnego miejsca) ===
    _display.setFont(spleen6x12PL);  // Większa czcionka
    
    // Lewa kolumna - informacje techniczne
    extern String streamCodec;
    extern String bitrateString;
    extern uint32_t SampleRate;
    extern uint8_t SampleRateRest;
    
    _display.drawStr(4, 26, "Codec:");
    _display.drawStr(50, 26, streamCodec.c_str());
    
    _display.drawStr(4, 38, "Bitrate:");
    String bitrateInfo = bitrateString + "kbps";
    _display.drawStr(60, 38, bitrateInfo.c_str());
    
    // Prawa kolumna - czas odtwarzania
    uint32_t currTime = audio.getAudioCurrentTime();
    uint32_t durTime = audio.getAudioFileDuration();
    
    _display.drawStr(140, 26, "Time:");
    if (durTime > 0) {
        char timeStr[15];
        snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu/%02lu:%02lu",
                 currTime / 60, currTime % 60,
                 durTime / 60, durTime % 60);
        _display.drawStr(180, 26, timeStr);
    } else {
        char timeStr[10];
        snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu", currTime / 60, currTime % 60);
        _display.drawStr(180, 26, timeStr);
    }
    
    // Status odtwarzania (bez etykiety)
    const char* playStatus = "";
    if (_player->isPlaying() && !_player->isPaused()) {
        playStatus = "PLAY";
    } else if (_player->isPaused()) {
        playStatus = "PAUSE";
    } else {
        playStatus = "STOP";
    }
    _display.drawStr(140, 38, playStatus);
    
    // === WSKAŹNIKI VU - DWA PASKI Z PODZIAŁKĄ ===
    uint16_t vuRaw = audio.getVUlevel();
    uint8_t vuL = min(vuRaw >> 8, 255);
    uint8_t vuR = min(vuRaw & 0xFF, 255);
    
    int vuX = 10;
    int vuY_L = 46;        // Lewy kanał
    int vuY_R = 54;        // Prawy kanał
    int vuWidth = 235;
    int vuHeight = 6;
    
    // Lewy kanał (L)
    _display.setFont(spleen6x12PL);
    _display.drawStr(2, vuY_L + 5, "L");
    _display.drawFrame(vuX, vuY_L, vuWidth, vuHeight);
    int fillL = (vuL * vuWidth) / 255;
    if (fillL > 0) {
        _display.drawBox(vuX + 1, vuY_L + 1, fillL - 1, vuHeight - 2);
    }
    
    // Prawy kanał (R)
    _display.drawStr(2, vuY_R + 5, "R");
    _display.drawFrame(vuX, vuY_R, vuWidth, vuHeight);
    int fillR = (vuR * vuWidth) / 255;
    if (fillR > 0) {
        _display.drawBox(vuX + 1, vuY_R + 1, fillR - 1, vuHeight - 2);
    }
    
    // Podziałka i cyfry u dołu (co 20% = 47px)
    _display.setFont(spleen6x12PL);
    for (int i = 0; i <= 5; i++) {
        int xMark = vuX + (i * vuWidth / 5);
        _display.drawLine(xMark, vuY_R + vuHeight, xMark, vuY_R + vuHeight + 2);
        
        // Cyfry: 0, 20, 40, 60, 80, 100
        char label[4];
        snprintf(label, sizeof(label), "%d", i * 20);
        int labelW = _display.getStrWidth(label);
        _display.drawStr(xMark - labelW / 2, vuY_R + vuHeight + 9, label);
    }
    
    // === FILE LIST REMOVED - Style 1 now only shows track info and VU meters ===
}

void SDPlayerOLED::drawTopBar() {
    if (!_player) return;
    
    _display.setFont(spleen6x12PL);
    
    if (_infoStyle == INFO_CLOCK_DATE) {
        // **ZEGAR PO LEWEJ** + **DATA W ŚRODKU** + **FORMAT AUDIO** + **GŁOŚNIK + VOLUME PO PRAWEJ**
        struct tm timeinfo;
        if (getLocalTime(&timeinfo, 100)) {
            // Zegar po lewej stronie (HH:MM)
            char timeStr[6];
            snprintf(timeStr, sizeof(timeStr), "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
            _display.drawStr(2, 11, timeStr);  // Lewy górny róg
            
            // Data w środku (DD.MM.YYYY)
            char dateStr[12];
            snprintf(dateStr, sizeof(dateStr), "%02d.%02d.%04d", 
                     timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
            int dateWidth = _display.getStrWidth(dateStr);
            int dateCenterX = (256 - dateWidth) / 2;  // Wyśrodkuj datę
            _display.drawStr(dateCenterX, 11, dateStr);
            
            // Wykryj rozszerzenie pliku audio
            String audioFormat = "";
            String currentFile = _player->getCurrentFile();
            if (currentFile.length() > 0 && currentFile != "None") {
                int dotPos = currentFile.lastIndexOf('.');
                if (dotPos > 0 && dotPos < currentFile.length() - 1) {
                    audioFormat = currentFile.substring(dotPos + 1);
                    audioFormat.toUpperCase();  // MP3, FLAC, WAV, OGG, AAC
                }
            }
            
            // Ikonka głośnika + Volume po prawej stronie
            int vol = _player->getVolume();
            String volStr = String(vol);
            int volWidth = _display.getStrWidth(volStr.c_str());
            
            // Pozycja głośnika i volume na końcu (prawa strona)
            int speakerX = 256 - volWidth - 20;  // 20px = szerokość ikony + margines
            int speakerY = 3;
            drawVolumeIcon(speakerX, speakerY);
            if (isMutedState()) {
                drawVolumeMuteSlash(speakerX, speakerY);
            }
            
            int volX = speakerX + 14;  // Zaraz po ikonie głośnika
            _display.drawStr(volX, 11, volStr.c_str());
            
            // Format audio między datą a głośnikiem (jeśli jest)
            if (audioFormat.length() > 0) {
                int formatWidth = _display.getStrWidth(audioFormat.c_str());
                int formatX = speakerX - formatWidth - 8;  // 8px odstęp od głośnika
                _display.drawStr(formatX, 11, audioFormat.c_str());
            }
        }
    } 
    else {
        // **TYTUŁ UTWORU** - cała szerokość górnego paska
        String currentTrack = _player->getCurrentFile();
        if (currentTrack.length() > 0) {
            // Usuń rozszerzenie
            int dotPos = currentTrack.lastIndexOf('.');
            if (dotPos > 0) {
                currentTrack = currentTrack.substring(0, dotPos);
            }
            
            // Obetnij jeśli za długi
            int maxWidth = 250;  // 256px - marginesy
            while (_display.getStrWidth(currentTrack.c_str()) > maxWidth && currentTrack.length() > 0) {
                currentTrack = currentTrack.substring(0, currentTrack.length() - 1);
            }
            if (currentTrack.length() < _player->getCurrentFile().length() - 4) {
                currentTrack += "...";
            }
            
            // Wyśrodkuj tytuł
            int titleWidth = _display.getStrWidth(currentTrack.c_str());
            int titleX = (256 - titleWidth) / 2;
            _display.drawStr(titleX, 11, currentTrack.c_str());
        }
    }
    
    // **POZIOMA KRESKA** przez cały wyświetlacz
    _display.drawLine(0, 14, 256, 14);
}

void SDPlayerOLED::drawFileList() {
    _display.setFont(spleen6x12PL);
    
    const int lineHeight = 12;
    const int startY = 16 + 12;  // Zaczyna się tuż pod górną kreską (14px) + offset
    const int visibleLines = 4;
    
    // Dostosuj scroll offset
    if (_selectedIndex < _scrollOffset) {
        _scrollOffset = _selectedIndex;
    }
    if (_selectedIndex >= _scrollOffset + visibleLines) {
        _scrollOffset = _selectedIndex - visibleLines + 1;
    }
    
    for (int i = 0; i < visibleLines && (i + _scrollOffset) < _fileList.size(); i++) {
        int idx = i + _scrollOffset;
        int y = startY + i * lineHeight;
        
        // Podświetlenie zaznaczonego
        if (idx == _selectedIndex) {
            _display.drawBox(0, y - 10, 245, 11);  // 245px szerokości (zostaw miejsce na scrollbar)
            _display.setDrawColor(0);
        }
        
        // Oblicz numer utworu (tylko dla plików muzycznych, pomijając foldery)
        int trackNumber = 0;
        if (!_fileList[idx].isDir) {
            // Licz pliki muzyczne przed tym indeksem
            for (int j = 0; j <= idx; j++) {
                if (!_fileList[j].isDir) {
                    trackNumber++;
                }
            }
        }
        
        // Ikona i numer dla plików muzycznych
        if (_fileList[idx].isDir) {
            // Trójkąt wskazujący w prawo ► dla folderów
            _display.drawTriangle(3, y-6, 3, y-2, 7, y-4);
        } else {
            // Numer utworu dla plików muzycznych 
            String trackNum = String(trackNumber) + ".";
            _display.drawStr(2, y, trackNum.c_str());
        }
        
        // Nazwa pliku ze scrollowaniem dla zaznaczonego
        String name = _fileList[idx].name;
        
        // Oblicz pozycję X dla tekstu (zależnie od szerokości numeru/ikony)
        int textStartX = 12;  // Domyślna pozycja dla folderów
        if (!_fileList[idx].isDir && trackNumber > 0) {
            // Dla plików muzycznych - dostosuj pozycję do szerokości numeru
            String trackNum = String(trackNumber) + ".";
            int numberWidth = _display.getStrWidth(trackNum.c_str());
            textStartX = 4 + numberWidth;  // 4px odstęp + szerokość numeru
        }
        
        if (idx == _selectedIndex) {
            // SCROLLOWANIE dla zaznaczonego elementu
            int maxWidth = 240 - textStartX;  // Max szerokość tekstu (dostosowana do pozycji startu)
            int nameWidth = _display.getStrWidth(name.c_str());
            
            if (nameWidth > maxWidth) {
                // Scrolluj tekst
                if (millis() - _lastScrollTime > 200) {
                    _scrollTextOffset++;
                    if (_scrollTextOffset > nameWidth + 20) _scrollTextOffset = -maxWidth;
                    _lastScrollTime = millis();
                }
                
                _display.setClipWindow(textStartX, y - 10, 245, y + 2);
                _display.drawStr(textStartX - _scrollTextOffset, y, name.c_str());
                _display.setMaxClipWindow();
            } else {
                _display.drawStr(textStartX, y, name.c_str());
            }
        } else {
            // Obcięcie dla nie-zaznaczonych (dostosowane do pozycji startu)
            int maxChars = (240 - textStartX) / 6;  // Przybliżona szerokość znaku w foncie 6x10
            if (name.length() > maxChars) {
                name = name.substring(0, maxChars - 3) + "...";
            }
            _display.drawStr(textStartX, y, name.c_str());
        }
        
        if (idx == _selectedIndex) {
            _display.setDrawColor(1);
        }
    }
}

void SDPlayerOLED::drawScrollBar(int itemCount, int visibleCount) {
    if (itemCount <= visibleCount) return;
    
    int barHeight = 48;
    int thumbHeight = (barHeight * visibleCount) / itemCount;
    if (thumbHeight < 4) thumbHeight = 4;
    
    int thumbPos = (barHeight - thumbHeight) * _scrollOffset / (itemCount - visibleCount);
    
    // Scrollbar z prawej strony (254px pozycja)
    _display.drawFrame(254, 16, 2, barHeight);
    _display.drawBox(254, 16 + thumbPos, 2, thumbHeight);
}

void SDPlayerOLED::drawVolumeIcon(int x, int y) {
    // Lepiej wyglądający głośnik
    // Podstawa głośnika (trójkąt + prostokąt)
    _display.drawBox(x, y+2, 2, 4);           // Prostokąt bazowy
    _display.drawPixel(x+2, y+1);             // Trójkąt
    _display.drawPixel(x+2, y+2);
    _display.drawPixel(x+2, y+5);
    _display.drawPixel(x+2, y+6);
    _display.drawBox(x+3, y, 2, 8);           // Membrana
    
    // Fale dźwiękowe (3 poziomy)
    _display.drawPixel(x+6, y+2);             // Fala 1 (cicha)
    _display.drawPixel(x+6, y+5);
    _display.drawPixel(x+7, y+1);             // Fala 2 (średnia)
    _display.drawPixel(x+7, y+3);
    _display.drawPixel(x+7, y+4);
    _display.drawPixel(x+7, y+6);
    _display.drawPixel(x+8, y+1);             // Fala 3 (głośna)
    _display.drawPixel(x+8, y+6);
}

void SDPlayerOLED::drawVolumeMuteSlash(int x, int y) {
    // Przekreślenie ikonki głośnika dla stanu MUTE
    _display.drawLine(x - 1, y + 8, x + 9, y - 1);
    _display.drawLine(x - 1, y + 7, x + 8, y - 1);
}

void SDPlayerOLED::drawMuteOverlayTag() {
    // Napis MUTE dla stylów bez ikonki głośnika
    _display.setFont(spleen6x12PL);
    _display.drawRBox(206, 0, 50, 12, 2);
    _display.setDrawColor(0);
    _display.drawStr(216, 9, "MUTE");
    _display.setDrawColor(1);
}

bool SDPlayerOLED::isMutedState() const {
    return volumeMute;
}

void SDPlayerOLED::renderStyle2() {
    // STYL 2: ODTWARZANIE - DUŻY TYTUŁ
    // Tytuł utworu na górze (scrollowany jeśli za długi)
    // Format audio + ikonka głośniczka + volume po prawej
    
    if (!_player) return;
    
    // KRYTYCZNE: Wyczyść bufor przed rysowaniem
    _display.clearBuffer();
    
    String currentFile = _player->getCurrentFile();
    String originalFile2 = currentFile;  // Zachowaj dla getTrackNumber()
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "Zatrzymany";
    } else {
        // Usuń rozszerzenie
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) {
            currentFile = currentFile.substring(0, dotPos);
        }
        // Usuń ścieżkę
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) {
            currentFile = currentFile.substring(slashPos + 1);
        }
        // Dodaj numer utworu z total
        int trackNumber = getTrackNumber(originalFile2);
        int totalTracks = getTotalTracks();
        if (trackNumber > 0 && totalTracks > 0) {
            currentFile = String(trackNumber) + "/" + String(totalTracks) + " " + currentFile;
        }
    }
    
    _display.setFont(spleen6x12PL);
    
    // === GÓRNY PASEK ===
    // 1. TYTUŁ UTWORU - SCROLLOWANY jeśli za długi
    int titleMaxWidth = 180; // Zostaw miejsce na format i volume
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > titleMaxWidth) {
        // Płynne scrollowanie w prawo
        static int scrollOffset = 0;
        static unsigned long lastScrollTime = 0;
        
        if (millis() - lastScrollTime > 100) {
            scrollOffset++;
            if (scrollOffset > titleWidth + 30) scrollOffset = 0;
            lastScrollTime = millis();
        }
        
        _display.setClipWindow(2, 0, titleMaxWidth + 2, 14);
        _display.drawStr(2 - scrollOffset, 11, currentFile.c_str());
        // Powtórz tekst dla ciągłego scrollowania
        _display.drawStr(2 - scrollOffset + titleWidth + 30, 11, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        // Wyśrodkowany jeśli się mieści
        int centerX = (titleMaxWidth - titleWidth) / 2;
        _display.drawStr(2 + centerX, 11, currentFile.c_str());
    }
    
    // 2. FORMAT AUDIO
    String audioFormat = "";
    String fullFileName = _player->getCurrentFile();
    if (fullFileName.length() > 0 && fullFileName != "None") {
        int dotPos = fullFileName.lastIndexOf('.');
        if (dotPos > 0) {
            audioFormat = fullFileName.substring(dotPos + 1);
            audioFormat.toUpperCase();
        }
    }
    
    // 3. IKONKA GŁOŚNICZKA + VOLUME
    int vol = _player->getVolume();
    String volStr = String(vol);
    int volWidth = _display.getStrWidth(volStr.c_str());
    
    int speakerX = 256 - volWidth - 20;
    drawVolumeIcon(speakerX, 3);
    if (isMutedState()) {
        drawVolumeMuteSlash(speakerX, 3);
    }
    _display.drawStr(speakerX + 14, 11, volStr.c_str());
    
    // Format przed głośnikiem
    if (audioFormat.length() > 0) {
        int formatWidth = _display.getStrWidth(audioFormat.c_str());
        int formatX = speakerX - formatWidth - 8;
        _display.drawStr(formatX, 11, audioFormat.c_str());
    }
    
    // === CIENKA LINIA ===
    _display.drawLine(0, 14, 256, 14);
    
    // === WSKAŹNIKI VU - POZIOME OD ŚRODKA W BOK ===
    uint16_t vuRaw = audio.getVUlevel();
    uint8_t vuL = min(vuRaw >> 8, 255);      // Lewy kanał
    uint8_t vuR = min(vuRaw & 0xFF, 255);    // Prawy kanał
    
    int centerX = 128;    // Środek ekranu (256/2)
    int vuBarHeight = 6;  // Wysokość ok 2mm
    int maxBarLength = 58; // Maksymalna długość paska w każdą stronę
    int vuY = 25;         // Pozycja Y wskaźników (obniżone o ~2mm)
    
    // Oblicz długość pasków
    int vuLengthL = (vuL * maxBarLength) / 255;
    int vuLengthR = (vuR * maxBarLength) / 255;
    
    // Napisy L i R
    _display.setFont(spleen6x12PL);
    _display.drawStr(centerX - maxBarLength - 15, vuY + 5, "L");
    _display.drawStr(centerX + maxBarLength + 8, vuY + 5, "R");
    
    // Ramki dla pasków
    _display.drawFrame(centerX - maxBarLength - 2, vuY, maxBarLength + 2, vuBarHeight);  // Lewy
    _display.drawFrame(centerX, vuY, maxBarLength + 2, vuBarHeight);  // Prawy
    
    // Wypełnienie - Lewy kanał (od środka w lewo)
    if (vuLengthL > 0) {
        _display.drawBox(centerX - vuLengthL, vuY + 1, vuLengthL, vuBarHeight - 2);
    }
    
    // Wypełnienie - Prawy kanał (od środka w prawo)
    if (vuLengthR > 0) {
        _display.drawBox(centerX + 1, vuY + 1, vuLengthR, vuBarHeight - 2);
    }
    
    // === INFORMACJE O UTWORZE PONIŻEJ VU METERA ===
    _display.setFont(spleen6x12PL);  // Większa czcionka
    
    extern String streamCodec;
    extern String bitrateString;
    extern uint32_t SampleRate;
    extern uint8_t SampleRateRest;
    
    // Linia 1: Codec + Bitrate + Samplerate (wszystko w jednej linii)
    String techInfo = streamCodec + " " + bitrateString + "kbps " + 
                      String(SampleRate) + "." + String(SampleRateRest) + "kHz";
    _display.drawStr(4, 41, techInfo.c_str());
    
    // Linia 2: Czas odtwarzania
    uint32_t currTime = audio.getAudioCurrentTime();
    uint32_t durTime = audio.getAudioFileDuration();
    
    _display.drawStr(4, 51, "Time:");
    if (durTime > 0) {
        char timeStr[15];
        snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu/%02lu:%02lu",
                 currTime / 60, currTime % 60,
                 durTime / 60, durTime % 60);
        _display.drawStr(40, 51, timeStr);
    } else {
        char timeStr[10];
        snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu", currTime / 60, currTime % 60);
        _display.drawStr(40, 51, timeStr);
    }
    
    // === KOMPAKTOWA LISTA PLIKÓW (2 linie zamiast 3) ===
    _display.setFont(spleen6x12PL);
    const int lineHeight = 10;
    const int startY = 52;
    const int visibleLines = 2;
    
    // Dostosuj scroll
    if (_selectedIndex < _scrollOffset) _scrollOffset = _selectedIndex;
    if (_selectedIndex >= _scrollOffset + visibleLines) _scrollOffset = _selectedIndex - visibleLines + 1;
    
    for (int i = 0; i < visibleLines && (i + _scrollOffset) < _fileList.size(); i++) {
        int idx = i + _scrollOffset;
        int y = startY + i * lineHeight;
        
        if (idx == _selectedIndex) {
            _display.drawBox(0, y - 8, 256, 9);
            _display.setDrawColor(0);
        }
        
        // Ikona
        if (_fileList[idx].isDir) {
            _display.drawTriangle(2, y-5, 2, y-2, 5, y-3);
        } else {
            _display.drawStr(2, y, "\xB7");
        }
        
        // Nazwa
        String name = _fileList[idx].name;
        if (name.length() > 48) name = name.substring(0, 47) + "...";
        _display.drawStr(8, y, name.c_str());
        
        if (idx == _selectedIndex) _display.setDrawColor(1);
    }
}

void SDPlayerOLED::renderStyle3() {
    // STYL 3: SPEKTRUM AUDIO - ANIMOWANE SŁUPKI
    // Górny pasek jak w stylu 2
    // Poniżej: spektrum częstotliwości z animacją
    
    if (!_player) return;
    
    // KRYTYCZNE: Wyczyść bufor przed rysowaniem
    _display.clearBuffer();
    
    String currentFile = _player->getCurrentFile();
    String originalFile3 = currentFile;  // Zachowaj dla getTrackNumber()
    if (currentFile == "None") currentFile = "---";
    else {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) currentFile = currentFile.substring(0, dotPos);
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) currentFile = currentFile.substring(slashPos + 1);
        // Dodaj numer utworu z total
        int trackNumber = getTrackNumber(originalFile3);
        int totalTracks = getTotalTracks();
        if (trackNumber > 0 && totalTracks > 0) {
            currentFile = String(trackNumber) + "/" + String(totalTracks) + " " + currentFile;
        }
    }
    
    _display.setFont(spleen6x12PL);
    
    // === GÓRNY PASEK ===
    // Tytuł ze scrollowaniem
    int titleMaxWidth = 180;
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > titleMaxWidth) {
        // Scrollowanie jak w stylu 2
        static int scrollOffset3 = 0;
        static unsigned long lastScrollTime3 = 0;
        
        if (millis() - lastScrollTime3 > 120) {
            scrollOffset3++;
            if (scrollOffset3 > titleWidth + 25) scrollOffset3 = 0;
            lastScrollTime3 = millis();
        }
        
        _display.setClipWindow(2, 0, titleMaxWidth + 2, 14);
        _display.drawStr(2 - scrollOffset3, 11, currentFile.c_str());
        _display.drawStr(2 - scrollOffset3 + titleWidth + 25, 11, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        int centerX = (titleMaxWidth - titleWidth) / 2;
        _display.drawStr(2 + centerX, 11, currentFile.c_str());
    }
    
    // Format audio
    String fullFileName = _player->getCurrentFile();
    String audioFormat = "";
    if (fullFileName != "None") {
        int dotPos = fullFileName.lastIndexOf('.');
        if (dotPos > 0) {
            audioFormat = fullFileName.substring(dotPos + 1);
            audioFormat.toUpperCase();
        }
    }
    
    // Volume + głośnik
    int vol = _player->getVolume();
    String volStr = String(vol);
    int volWidth = _display.getStrWidth(volStr.c_str());
    int speakerX = 256 - volWidth - 20;
    
    drawVolumeIcon(speakerX, 3);
    if (isMutedState()) {
        drawVolumeMuteSlash(speakerX, 3);
    }
    _display.drawStr(speakerX + 14, 11, volStr.c_str());
    
    if (audioFormat.length() > 0) {
        int formatWidth = _display.getStrWidth(audioFormat.c_str());
        int formatX = speakerX - formatWidth - 8;
        _display.drawStr(formatX, 11, audioFormat.c_str());
    }
    
    _display.drawLine(0, 14, 256, 14);
    
    // === PRAWDZIWY ANALIZATOR FFT - 16 PASM ===
    float fftLevels[16];
    eq_get_analyzer_levels(fftLevels);
    
    const int numBars = 16;
    const int barWidth = 14;      // Szersze słupki dla 16 pasm
    const int barGap = 2;
    const int barStartY = 18;
    const int maxBarHeight = 20;
    
    for (int i = 0; i < numBars; i++) {
        // Konwersja poziomu FFT (0.0-1.0) na wysokość słupka
        int height = (int)(fftLevels[i] * maxBarHeight);
        if (height > maxBarHeight) height = maxBarHeight;
        if (height < 1 && fftLevels[i] > 0.01f) height = 1; // Min 1px gdy jest sygnał
        
        int x = 2 + i * (barWidth + barGap);
        int y = barStartY + maxBarHeight - height;
        
        // Naprzemienne wypełnione/ramki dla efektu wizualnego
        if (i % 2 == 0) {
            _display.drawBox(x, y, barWidth, height);
        } else {
            _display.drawFrame(x, y, barWidth, height);
            if (height > 2) _display.drawBox(x+1, y+1, barWidth-2, height-2);
        }
    }
    
    // === INFORMACJE O UTWORZE (poniżej analizatora) ===
    _display.setFont(spleen6x12PL);  // Większa czcionka
    
    extern String streamCodec;
    extern String bitrateString;
    extern uint32_t SampleRate;
    extern uint8_t SampleRateRest;
    
    // Linia 1: Codec + Bitrate + Samplerate
    String info1 = streamCodec + " " + bitrateString + "kbps " + 
                   String(SampleRate) + "." + String(SampleRateRest) + "kHz";
    _display.drawStr(4, 49, info1.c_str());
    
    // Linia 2: Czas odtwarzania + Status
    uint32_t currTime3 = audio.getAudioCurrentTime();
    uint32_t durTime3 = audio.getAudioFileDuration();
    
    if (durTime3 > 0) {
        char timeStr[20];
        snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu/%02lu:%02lu",
                 currTime3 / 60, currTime3 % 60,
                 durTime3 / 60, durTime3 % 60);
        _display.drawStr(4, 58, timeStr);
    } else {
        char timeStr[10];
        snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu", currTime3 / 60, currTime3 % 60);
        _display.drawStr(4, 58, timeStr);
    }
    
    // Status odtwarzania
    const char* status = _player->isPlaying() ? (_player->isPaused() ? "PAUSE" : "PLAY") : "STOP";
    _display.drawStr(180, 58, status);
}


void SDPlayerOLED::renderStyle4() {
    // STYL 4: DUŻY TYTUŁ + INFORMACJE O PLIKU + LISTA
    // Skoncentrowany na czytelności i informacjach
    
    if (!_player) return;
    
    // KRYTYCZNE: Wyczyść bufor przed rysowaniem
    _display.clearBuffer();
    
    String currentFile = _player->getCurrentFile();
    String originalFile4 = currentFile;  // Zachowaj dla getTrackNumber()
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "Zatrzymany";
    } else {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) currentFile = currentFile.substring(0, dotPos);
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) currentFile = currentFile.substring(slashPos + 1);
        // Dodaj numer utworu z total
        int trackNumber = getTrackNumber(originalFile4);
        int totalTracks = getTotalTracks();
        if (trackNumber > 0 && totalTracks > 0) {
            currentFile = String(trackNumber) + "/" + String(totalTracks) + " " + currentFile;
        }
    }
    
    // === DUŻY TYTUŁ (większa czcionka) ===
    _display.setFont(spleen6x12PL);
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > 250) {
        static int scrollOffset4 = 0;
        static unsigned long lastScrollTime4 = 0;
        
        if (millis() - lastScrollTime4 > 90) {
            scrollOffset4++;
            if (scrollOffset4 > titleWidth + 35) scrollOffset4 = 0;
            lastScrollTime4 = millis();
        }
        
        _display.setClipWindow(3, 0, 253, 15);
        _display.drawStr(3 - scrollOffset4, 12, currentFile.c_str());
        _display.drawStr(3 - scrollOffset4 + titleWidth + 35, 12, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        int centerX = (256 - titleWidth) / 2;
        _display.drawStr(centerX, 12, currentFile.c_str());
    }
    
    _display.drawLine(0, 15, 256, 15);
    
    // === INFORMACJE O PLIKU (format, volume, status) ===
    _display.setFont(spleen6x12PL);
    
    // Format audio
    String fullFileName = _player->getCurrentFile();
    String audioFormat = "";
    if (fullFileName != "None") {
        int dotPos = fullFileName.lastIndexOf('.');
        if (dotPos > 0) {
            audioFormat = fullFileName.substring(dotPos + 1);
            audioFormat.toUpperCase();
        }
    }
    
    // Status odtwarzania
    String status = "STOP";
    if (_player->isPlaying() && !_player->isPaused()) {
        status = "PLAY";
    } else if (_player->isPaused()) {
        status = "PAUSE";
    }
    
    // Lewa strona: Format
    if (audioFormat.length() > 0) {
        _display.drawStr(4, 26, "Format:");
        _display.drawStr(50, 26, audioFormat.c_str());
    }
    
    // Prawa strona: Status
    _display.drawStr(140, 26, "Status:");
    _display.drawStr(190, 26, status.c_str());
    
    // Druga linia info: Volume
    int vol = _player->getVolume();
    char volStr[20];
    snprintf(volStr, sizeof(volStr), "Volume: %d", vol);
    
    _display.drawStr(4, 36, volStr);
    
    // Pasek volume (wizualizacja)
    int barWidth = (vol * 80) / 21;  // max vol 21 -> 80px
    _display.drawFrame(140, 28, 82, 8);
    if (barWidth > 0) {
        _display.drawBox(141, 29, barWidth, 6);
    }
    
    _display.drawLine(0, 40, 256, 40);
    
    // === INFORMACJE TECHNICZNE O UTWORZE ===
    _display.setFont(spleen6x12PL);  // Większa czcionka
    
    extern String streamCodec;
    extern String bitrateString;
    extern uint32_t SampleRate;
    extern uint8_t SampleRateRest;
    
    // Lewa strona - dane techniczne
    String techInfo = streamCodec + " " + bitrateString + "kbps " + 
                      String(SampleRate) + "." + String(SampleRateRest) + "kHz";
    _display.drawStr(4, 50, techInfo.c_str());
    
    // === WSKAŹNIKI VU - POZIOME OD ŚRODKA W BOK (jak Styl 2) ===
    uint16_t vuRaw = audio.getVUlevel();
    uint8_t vuL = min(vuRaw >> 8, 255);      // Lewy kanał
    uint8_t vuR = min(vuRaw & 0xFF, 255);    // Prawy kanał
    
    int centerX = 200;     // Środek VU metera po prawej stronie
    int vuBarHeight = 6;   // Wysokość ok 2mm
    int maxBarLength = 75; // Maksymalna długość paska w każdą stronę (zwiększone o ~30px)
    int vuY = 50;          // Pozycja Y wskaźników (obniżone)
    
    // Oblicz długość pasków
    int vuLengthL = (vuL * maxBarLength) / 255;
    int vuLengthR = (vuR * maxBarLength) / 255;
    
    // Napisy L i R
    _display.setFont(spleen6x12PL);
    _display.drawStr(centerX - maxBarLength - 12, vuY + 5, "L");
    _display.drawStr(centerX + maxBarLength + 5, vuY + 5, "R");
    
    // Ramki dla pasków
    _display.drawFrame(centerX - maxBarLength - 2, vuY, maxBarLength + 2, vuBarHeight);  // Lewy
    _display.drawFrame(centerX, vuY, maxBarLength + 2, vuBarHeight);  // Prawy
    
    // Wypełnienie - Lewy kanał (od środka w lewo)
    if (vuLengthL > 0) {
        _display.drawBox(centerX - vuLengthL, vuY + 1, vuLengthL, vuBarHeight - 2);
    }
    
    // Wypełnienie - Prawy kanał (od środka w prawo)
    if (vuLengthR > 0) {
        _display.drawBox(centerX + 1, vuY + 1, vuLengthR, vuBarHeight - 2);
    }
    
    // === LISTA PLIKÓW (2 linie) ===
    _display.setFont(spleen6x12PL);
    const int lineHeight = 10;
    const int startY = 52;
    const int visibleLines = 2;
    
    if (_selectedIndex < _scrollOffset) _scrollOffset = _selectedIndex;
    if (_selectedIndex >= _scrollOffset + visibleLines) _scrollOffset = _selectedIndex - visibleLines + 1;
    
    for (int i = 0; i < visibleLines && (i + _scrollOffset) < _fileList.size(); i++) {
        int idx = i + _scrollOffset;
        int y = startY + i * lineHeight;
        
        if (idx == _selectedIndex) {
            _display.drawBox(0, y - 8, 256, 9);
            _display.setDrawColor(0);
        }
        
        // Ikona + numer + nazwa
        if (_fileList[idx].isDir) {
            _display.drawTriangle(2, y-5, 2, y-2, 5, y-3);
            // Nazwa folderu bez numeru
            String name = _fileList[idx].name;
            if (name.length() > 45) name = name.substring(0, 44) + "...";
            _display.drawStr(8, y, name.c_str());
        } else {
            // Numer utworu + nazwa pliku
            String numberedName = String(idx + 1) + ". " + _fileList[idx].name;
            if (numberedName.length() > 43) numberedName = numberedName.substring(0, 42) + "...";
            _display.drawStr(2, y, numberedName.c_str());
        }
        
        if (idx == _selectedIndex) _display.setDrawColor(1);
    }
}

void SDPlayerOLED::renderStyle5() {
    // STYL 5: MINIMALISTYCZNY
    // Tytuł scrollowany na środku + prosty pasek volume
    
    if (!_player) return;
    
    // KRYTYCZNE: Aktywuj analizator dla tego stylu (jeśli używa FFT)
    eq_analyzer_set_runtime_active(true);
    
    // KRYTYCZNE: Wyczyść bufor przed rysowaniem - blokuje przebijanie się radia
    _display.clearBuffer();
    
    // === TYTUŁ SCROLLOWANY NA ŚRODKU ===
    String currentFile = _player->getCurrentFile();
    String originalFile5 = currentFile;  // Zachowaj dla getTrackNumber()
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "---";
    } else {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) currentFile = currentFile.substring(0, dotPos);
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) currentFile = currentFile.substring(slashPos + 1);
        // Dodaj numer utworu z total
        int trackNumber = getTrackNumber(originalFile5);
        int totalTracks = getTotalTracks();
        if (trackNumber > 0 && totalTracks > 0) {
            currentFile = String(trackNumber) + "/" + String(totalTracks) + " " + currentFile;
        }
    }
    
    _display.setFont(spleen6x12PL);
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > 240) {
        static int scrollOffset5 = 0;
        static unsigned long lastScrollTime5 = 0;
        
        if (millis() - lastScrollTime5 > 120) {
            scrollOffset5++;
            if (scrollOffset5 > titleWidth + 30) scrollOffset5 = 0;
            lastScrollTime5 = millis();
        }
        
        _display.setClipWindow(8, 0, 248, 35);
        _display.drawStr(8 - scrollOffset5, 30, currentFile.c_str());
        _display.drawStr(8 - scrollOffset5 + titleWidth + 30, 30, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        int centerX = (256 - titleWidth) / 2;
        _display.drawStr(centerX, 30, currentFile.c_str());
    }
    
    // === VOLUME BAR (mały) ===
    int vol = _player->getVolume();
    int barW = (vol * 200) / 21;
    _display.drawFrame(28, 38, 200, 6);
    _display.drawBox(28, 38, barW, 6);
    
    // === INFORMACJE O UTWORZE ===
    _display.setFont(spleen6x12PL);
    
    extern String streamCodec;
    extern String bitrateString;
    extern uint32_t SampleRate;
    extern uint8_t SampleRateRest;
    
    // Linia 1: Codec + Bitrate
    String info1 = "Codec: " + streamCodec + "  Bitrate: " + bitrateString + "k";
    _display.drawStr(4, 50, info1.c_str());
    
    // Linia 2: Samplerate + Status + Czas odtwarzania
    String info2 = "Sample: " + String(SampleRate) + "." + String(SampleRateRest) + "kHz";
    _display.drawStr(4, 58, info2.c_str());
    
    // Status przesunięty w prawo (ok 4mm ~ 15 pikseli)
    const char* status5 = _player->isPlaying() ? (_player->isPaused() ? "PAUSE" : "PLAY") : "STOP";
    _display.drawStr(165, 58, status5);
    
    // Długość utworu i odliczanie do końca za statusem
    uint32_t currTime5 = audio.getAudioCurrentTime();
    uint32_t durTime5 = audio.getAudioFileDuration();
    
    if (durTime5 > 0) {
        uint32_t remainingTime = durTime5 - currTime5;
        char timeStr[15];
        snprintf(timeStr, sizeof(timeStr), "-%02lu:%02lu/%02lu:%02lu",
                 remainingTime / 60, remainingTime % 60,
                 durTime5 / 60, durTime5 % 60);
        _display.drawStr(205, 58, timeStr);
    }
}

void SDPlayerOLED::renderStyle6() {
    // STYL 6: INFO AUDIO + PROGRESS BAR
    // Tytuł scrollowany na środku + informacje o pliku audio z paskiem postępu
    
    if (!_player) return;
    
    // KRYTYCZNE: Aktywuj analizator FFT dla tego stylu
    eq_analyzer_set_runtime_active(true);
    
    // KRYTYCZNE: Wyczyść bufor przed rysowaniem - blokuje przebijanie się radia
    _display.clearBuffer();
    
    _display.setFont(spleen6x12PL);
    
    // === TYTUŁ SCROLLOWANY NA ŚRODKU ===
    String currentFile = _player->getCurrentFile();
    String originalFile6 = currentFile;  // Zachowaj dla getTrackNumber()
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "Brak utworu";
    } else {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) currentFile = currentFile.substring(0, dotPos);
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) currentFile = currentFile.substring(slashPos + 1);
        // Dodaj numer utworu z total
        int trackNumber = getTrackNumber(originalFile6);
        int totalTracks = getTotalTracks();
        if (trackNumber > 0 && totalTracks > 0) {
            currentFile = String(trackNumber) + "/" + String(totalTracks) + " " + currentFile;
        }
    }
    
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > 240) {
        static int scrollOffset6 = 0;
        static unsigned long lastScrollTime6 = 0;
        
        if (millis() - lastScrollTime6 > 100) {
            scrollOffset6++;
            if (scrollOffset6 > titleWidth + 30) scrollOffset6 = 0;
            lastScrollTime6 = millis();
        }
        
        _display.setClipWindow(8, 0, 248, 14);
        _display.drawStr(8 - scrollOffset6, 11, currentFile.c_str());
        _display.drawStr(8 - scrollOffset6 + titleWidth + 30, 11, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        int centerX = (256 - titleWidth) / 2;
        _display.drawStr(centerX, 11, currentFile.c_str());
    }
    
    _display.drawLine(0, 14, 256, 14);
    
    // === INFORMACJE O AUDIO (2 kolumny) ===
    _display.setFont(spleen6x12PL);
    
    // Lewa kolumna
    _display.drawStr(4, 24, "Format:");
    // USUNIĘTO getCurrentFile() - nie pobieramy nazwy pliku, tylko wyświetlamy stałą wartość
    _display.drawStr(42, 24, "MP3");
    
    _display.drawStr(4, 34, "Bitrate:");
    _display.drawStr(42, 34, "192k");  // Można dodać rzeczywiste dane z audio
    
    // Prawa kolumna
    _display.drawStr(130, 24, "Volume:");
    int vol = _player->getVolume();
    String volStr = String(vol);
    _display.drawStr(168, 24, volStr.c_str());
    
    _display.drawStr(130, 34, "Status:");
    if (_player->isPlaying() && !_player->isPaused()) {
        _display.drawStr(168, 34, "PLAY");
    } else if (_player->isPaused()) {
        _display.drawStr(168, 34, "PAUSE");
    } else {
        _display.drawStr(168, 34, "STOP");
    }
    
    // === PASEK POSTĘPU (z czasem) ===
    _display.setFont(spleen6x12PL);
    
    // Symulacja czasu - w rzeczywistości pobierz z player
    int currentSeconds = 125;  // 2:05
    int totalSeconds = 245;    // 4:05
    
    // Formatowanie czasu
    char currentTime[8];
    char totalTime[8];
    snprintf(currentTime, sizeof(currentTime), "%d:%02d", currentSeconds / 60, currentSeconds % 60);
    snprintf(totalTime, sizeof(totalTime), "%d:%02d", totalSeconds / 60, totalSeconds % 60);
    
    // Wyświetl czas po lewej i prawej
    _display.drawStr(4, 46, currentTime);
    int totalTimeWidth = _display.getStrWidth(totalTime);
    _display.drawStr(256 - totalTimeWidth - 4, 46, totalTime);
    
    // Pasek postępu (między czasami)
    int progressBarX = 30;
    int progressBarY = 40;
    int progressBarWidth = 195;
    int progressBarHeight = 8;
    
    _display.drawFrame(progressBarX, progressBarY, progressBarWidth, progressBarHeight);
    
    // Wypełnienie na podstawie postępu
    int progressFill = 0;
    if (totalSeconds > 0) {
        progressFill = (currentSeconds * (progressBarWidth - 2)) / totalSeconds;
    }
    if (progressFill > 0) {
        _display.drawBox(progressBarX + 1, progressBarY + 1, progressFill, progressBarHeight - 2);
    }
    
    // === INFORMACJE TECHNICZNE (ostatnia linia na dole) ===
    _display.setFont(spleen6x12PL);
    
    extern String streamCodec;
    extern String bitrateString;
    extern uint32_t SampleRate;
    extern uint8_t SampleRateRest;
    
    // Informacje: Codec + Bitrate + Samplerate
    String techInfo = streamCodec + " " + bitrateString + "kbps " + 
                      String(SampleRate) + "." + String(SampleRateRest) + "kHz";
    _display.drawStr(4, 58, techInfo.c_str());
    
    // === VU METERS - PROSTOKĄTY ROZSUWAJĄCE SIĘ OD ŚRODKA ===
    uint16_t vu = audio.getVUlevel();
    uint8_t vL = (vu >> 8) & 0xFF;
    uint8_t vR = vu & 0xFF;
    
    int cX = 185;         // Środek przesunięty w prawo o ~20mm
    int vuY = 52;         // Pozycja Y
    int barHeight = 6;    // Wysokość ~2mm
    int maxBarLength = 50;  // Maksymalna długość paska w każdą stronę
    
    int wL = (vL * maxBarLength) / 255;
    int wR = (vR * maxBarLength) / 255;
    
    // Prostokąty rozsuwające się od środka
    // Lewy kanał - prostokąt od środka w lewo
    if (wL > 0) {
        _display.drawBox(cX - wL, vuY, wL, barHeight);
    }
    
    // Prawy kanał - prostokąt od środka w prawo
    if (wR > 0) {
        _display.drawBox(cX + 2, vuY, wR, barHeight);
    }
    
    // Literki L i R na końcach
    _display.setFont(spleen6x12PL);
    _display.drawStr(cX - maxBarLength - 10, vuY + 5, "L");
    _display.drawStr(cX + maxBarLength + 5, vuY + 5, "R");
}

void SDPlayerOLED::renderStyle7() {
    // STYL 7: ANALIZATOR RETRO
    // Góra: Data | System | Głośniczek+Volume
    // Tytuł utworu (scrollowany)
    // Długa kreska
    // Analizator FFT z trójkątnymi słupkami (podstawa szersza, im wyżej tym węższe)
    
    if (!_player) return;
    
    // KRYTYCZNE: Aktywuj analizator FFT dla tego stylu
    eq_analyzer_set_runtime_active(true);
    
    // KRYTYCZNE: Wyczyść bufor przed rysowaniem
    _display.clearBuffer();
    
    _display.setFont(spleen6x12PL);
    
    // === GÓRNY PASEK ===
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        // 1. DATA PO LEWEJ (DD.MM.YYYY)
        char dateStr[12];
        snprintf(dateStr, sizeof(dateStr), "%02d.%02d.%04d", 
                 timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
        _display.drawStr(2, 11, dateStr);
        
        // 2. SYSTEM W ŚRODKU
        const char* systemText = "SD PLAYER";
        int systemWidth = _display.getStrWidth(systemText);
        int systemCenterX = (256 - systemWidth) / 2;
        _display.drawStr(systemCenterX, 11, systemText);
        
        // 3. IKONKA GŁOŚNICZKA + VOLUME PO PRAWEJ
        int vol = _player->getVolume();
        String volStr = String(vol);
        int volWidth = _display.getStrWidth(volStr.c_str());
        
        int speakerX = 256 - volWidth - 20;
        drawVolumeIcon(speakerX, 3);
        if (isMutedState()) {
            drawVolumeMuteSlash(speakerX, 3);
        }
        _display.drawStr(speakerX + 14, 11, volStr.c_str());
    }
    
    // === CIENKA LINIA POD PASKIEM ===
    _display.drawLine(0, 14, 256, 14);
    
    // === TYTUŁ UTWORU (scrollowany) ===
    String currentFile = _player->getCurrentFile();
    String originalFile7 = currentFile;  // Zachowaj dla getTrackNumber()
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "---";
    } else {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) currentFile = currentFile.substring(0, dotPos);
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) currentFile = currentFile.substring(slashPos + 1);
        // Dodaj numer utworu z total
        int trackNumber = getTrackNumber(originalFile7);
        int totalTracks = getTotalTracks();
        if (trackNumber > 0 && totalTracks > 0) {
            currentFile = String(trackNumber) + "/" + String(totalTracks) + " " + currentFile;
        }
    }
    
    _display.setFont(spleen6x12PL);
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    int titleY = 26;
    
    if (titleWidth > 250) {
        // Scrolluj długi tytuł
        static int scrollOffset = 0;
        static unsigned long lastScrollTime = 0;
        
        if (millis() - lastScrollTime > 100) {
            scrollOffset++;
            if (scrollOffset > titleWidth + 30) scrollOffset = 0;
            lastScrollTime = millis();
        }
        
        _display.setClipWindow(3, 16, 253, 30);
        _display.drawStr(3 - scrollOffset, titleY, currentFile.c_str());
        _display.drawStr(3 - scrollOffset + titleWidth + 30, titleY, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        // Wyśrodkuj krótki tytuł
        int centerX = (256 - titleWidth) / 2;
        _display.drawStr(centerX, titleY, currentFile.c_str());
    }
    
    // === DŁUGA KRESKA POD TYTUŁEM ===
    _display.drawLine(3, 30, 253, 30);
    
    // === ANALIZATOR FFT Z TRÓJKĄTNYMI SŁUPKAMI ===
    // Pobierz dane z analizatora FFT
    float levels[EQ_BANDS];
    eq_get_analyzer_levels(levels);
    
    const int analyzerY = 62;  // Dolna linia analizatora
    const int maxHeight = 28;  // Maksymalna wysokość słupka
    const int barSpacing = 2;  // Odstęp między słupkami
    const int totalWidth = 250; // Szerokość obszaru analizatora
    const int barWidth = (totalWidth - (EQ_BANDS - 1) * barSpacing) / EQ_BANDS;
    const int startX = 3;
    
    for (int i = 0; i < EQ_BANDS; i++) {
        // Wysokość słupka bazująca na poziomie FFT
        int height = (int)(levels[i] * maxHeight);
        if (height > maxHeight) height = maxHeight;
        if (height < 1) height = 1;
        
        int x = startX + i * (barWidth + barSpacing);
        int y = analyzerY;
        
        // Rysuj trójkątny słupek - podstawa szersza, im wyżej tym węższy
        // Używamy kropek/pikseli do stworzenia efektu trójkąta
        for (int h = 0; h < height; h++) {
            // Oblicz szerokość na danej wysokości (od pełnej na dole do 1 na górze)
            int widthAtHeight = barWidth - (h * barWidth / maxHeight);
            if (widthAtHeight < 1) widthAtHeight = 1;
            
            // Wyśrodkuj szerokość na danej wysokości
            int xOffset = (barWidth - widthAtHeight) / 2;
            
            // Rysuj linię o odpowiedniej szerokości
            if (h % 2 == 0) {  // Co drugi piksel dla efektu "kropek"
                _display.drawLine(x + xOffset, y - h, x + xOffset + widthAtHeight - 1, y - h);
            } else {
                // Dla efektu kropkowego rysuj tylko co 2 piksel w poziomie
                for (int px = 0; px < widthAtHeight; px += 2) {
                    _display.drawPixel(x + xOffset + px, y - h);
                }
            }
        }
    }
}

void SDPlayerOLED::renderStyle8() {
    // STYL 8: Layout jak SDPlayerAdvanced (ID3 + parametry + timer)
    if (!_player) return;

    _display.clearBuffer();
    _display.setMaxClipWindow();
    _display.setDrawColor(1);

    _display.setFont(spleen6x12PL);

    int iconX = 2;
    int line1Y = 12;

    if (_player->isPlaying() && !_player->isPaused()) {
        drawStyle8BitmapIcon(_display, iconX, 2, style8_icon_play_bits);
    } else if (_player->isPaused()) {
        drawStyle8BitmapIcon(_display, iconX, 2, style8_icon_pause_bits);
    } else {
        drawStyle8BitmapIcon(_display, iconX, 2, style8_icon_stop_bits);
    }

    String currentFile = _player->getCurrentFile();
    String baseName = currentFile;
    if (baseName.length() > 0 && baseName != "None") {
        int slashPos = baseName.lastIndexOf('/');
        if (slashPos >= 0) baseName = baseName.substring(slashPos + 1);
        int dotPos = baseName.lastIndexOf('.');
        if (dotPos > 0) baseName = baseName.substring(0, dotPos);
    } else {
        baseName = "";
    }

    // NOWE: Użyj metadanych ID3 jeśli są dostępne, w przeciwnym razie parsuj nazwę pliku
    String artistPart = "Unknown";
    String titlePart = baseName;
    String albumPart = "-";
    
    // Jeśli mamy metadane ID3, użyj ich
    if (currentMP3Artist.length() > 0 || currentMP3Title.length() > 0) {
        if (currentMP3Artist.length() > 0) {
            artistPart = currentMP3Artist;
        }
        if (currentMP3Title.length() > 0) {
            titlePart = currentMP3Title;
        }
        if (currentMP3Album.length() > 0) {
            albumPart = currentMP3Album;
        }
    }
    // W przeciwnym razie parsuj nazwę pliku (Artist - Title)
    else {
        String normalizedName = baseName;
        normalizedName.replace("_", " ");

        int sepLen = 0;
        int sep = normalizedName.indexOf(" - ");
        if (sep > 0) {
            sepLen = 3;
        } else {
            sep = normalizedName.indexOf(" – ");
            if (sep > 0) {
                sepLen = 3;
            } else {
                sep = normalizedName.indexOf(" | ");
                if (sep > 0) {
                    sepLen = 3;
                } else {
                    sep = normalizedName.indexOf(" ** ");
                    if (sep > 0) {
                        sepLen = 4;
                    }
                }
            }
        }

        if (sep > 0 && sepLen > 0) {
            artistPart = normalizedName.substring(0, sep);
            titlePart = normalizedName.substring(sep + sepLen);
            artistPart.trim();
            titlePart.trim();
        } else {
            titlePart = normalizedName;
        }

        // Fallback: usuń numerację z początku (np. "01. ", "01 - ", "01_")
        int artistIdx = 0;
        while (artistIdx < artistPart.length() && artistPart[artistIdx] >= '0' && artistPart[artistIdx] <= '9') {
            artistIdx++;
        }
        if (artistIdx > 0) {
            while (artistIdx < artistPart.length() &&
                (artistPart[artistIdx] == ' ' || artistPart[artistIdx] == '.' ||
                    artistPart[artistIdx] == '-' || artistPart[artistIdx] == '_' ||
                    artistPart[artistIdx] == ')' || artistPart[artistIdx] == '(')) {
                artistIdx++;
            }
            if (artistIdx < artistPart.length()) {
                artistPart = artistPart.substring(artistIdx);
                artistPart.trim();
            }
        }

        int titleIdx = 0;
        while (titleIdx < titlePart.length() && titlePart[titleIdx] >= '0' && titlePart[titleIdx] <= '9') {
            titleIdx++;
        }
        if (titleIdx > 0) {
            while (titleIdx < titlePart.length() &&
                   (titlePart[titleIdx] == ' ' || titlePart[titleIdx] == '.' ||
                    titlePart[titleIdx] == '-' || titlePart[titleIdx] == '_' ||
                    titlePart[titleIdx] == ')' || titlePart[titleIdx] == '(')) {
                titleIdx++;
            }
            if (titleIdx < titlePart.length()) {
                titlePart = titlePart.substring(titleIdx);
                titlePart.trim();
            }
        }
    }

    String artistText = "ARTYSTA - " + artistPart;
    while (_display.getStrWidth(artistText.c_str()) > 200 && artistText.length() > 10) {
        artistText = artistText.substring(0, artistText.length() - 1);
    }
    _display.drawStr(14, line1Y, artistText.c_str());

    // Volume z ikoną głośniczka
    int vol = _player->getVolume();
    bool muted = (vol == 0);
    String volStr = String(vol);
    int volWidth = _display.getStrWidth(volStr.c_str());
    
    // Ikona głośniczka (8 pikseli szerokości + 2 pikseli odstępu = 10)
    int iconX_vol = 256 - volWidth - 12;
    if (muted) {
        drawStyle8BitmapIcon(_display, iconX_vol, 2, style8_icon_speaker_muted_bits);
    } else {
        drawStyle8BitmapIcon(_display, iconX_vol, 2, style8_icon_speaker_bits);
    }
    
    // Wartość volume obok ikony
    _display.drawStr(256 - volWidth - 2, line1Y, volStr.c_str());

    int line2Y = 24;
    // Usunięto ikonę muzyki przed tytułem

    // Numer utworu + tytuł - użyj funkcji pomocniczej
    int trackNumber = getTrackNumber(currentFile);
    int totalTracks = getTotalTracks();
    
    String titleText = "";
    if (trackNumber > 0 && totalTracks > 0) {
        titleText = String(trackNumber) + "/" + String(totalTracks) + " ";
    }
    if (titlePart.length() > 0) {
        titleText += titlePart;
    } else {
        titleText += "Stopped";
    }

    while (_display.getStrWidth(titleText.c_str()) > 200 && titleText.length() > 10) {
        titleText = titleText.substring(0, titleText.length() - 1);
    }
    _display.drawStr(14, line2Y, titleText.c_str());

    int line3Y = 36;
    String albumText = "ALBUM - ";
    
    // Jeśli mamy ID3 album, użyj go
    if (albumPart.length() > 0 && albumPart != "-") {
        albumText += albumPart;
    }
    // W przeciwnym razie użyj nazwy folderu
    else {
        String currentDir = _player->getCurrentDirectory();
        // Wyciągnij ostatnią część ścieżki (np. "/MUZYKA/Pink Floyd" → "Pink Floyd")
        String albumName = "-";
        if (currentDir.length() > 0) {
            int lastSlash = currentDir.lastIndexOf('/');
            if (lastSlash >= 0 && lastSlash < currentDir.length() - 1) {
                albumName = currentDir.substring(lastSlash + 1);
            } else if (lastSlash < 0) {
                albumName = currentDir;
            }
        }
        albumText += albumName;
    }

    while (_display.getStrWidth(albumText.c_str()) > 230 && albumText.length() > 10) {
        albumText = albumText.substring(0, albumText.length() - 1);
    }
    _display.drawStr(2, line3Y, albumText.c_str());

    _display.drawLine(0, 38, 256, 38);

    _display.setFont(spleen6x12PL);
    int line4Y = 49;

    String codecStr = streamCodec;
    if (codecStr.length() == 0) codecStr = "-";
    _display.drawStr(2, line4Y, codecStr.c_str());

    String bitrateStr = bitrateString;
    if (bitrateStr.length() == 0 || bitrateStr == "--") bitrateStr = "0";
    bitrateStr += "kbps";
    _display.drawStr(50, line4Y, bitrateStr.c_str());

    String srStr = String(SampleRate) + "." + String(SampleRateRest) + "kHz";
    _display.drawStr(120, line4Y, srStr.c_str());

    String bpsStr = bitsPerSampleString;
    if (bpsStr.length() == 0 || bpsStr == "--") bpsStr = "0";
    bpsStr += "bit";
    _display.drawStr(180, line4Y, bpsStr.c_str());

    int line5Y = 61;
    if (_player->isPlaying() && !_player->isPaused()) {
        uint32_t currTime = audio.getAudioCurrentTime();
        uint32_t durTime = audio.getAudioFileDuration();
        char timerStr[20];
        if (durTime > 0) {
            snprintf(timerStr, sizeof(timerStr), "%02lu:%02lu/%02lu:%02lu",
                     currTime / 60, currTime % 60,
                     durTime / 60, durTime % 60);
        } else {
            snprintf(timerStr, sizeof(timerStr), "%02lu:%02lu", currTime / 60, currTime % 60);
        }
        _display.drawStr(2, line5Y, timerStr);
    } else {
        _display.drawStr(2, line5Y, "--:--");
    }

    drawStyle8BitmapIcon(_display, 180, 52, style8_icon_repeat_bits);
    _display.drawStr(192, line5Y, "ALL");
}

void SDPlayerOLED::renderStyle9() {
    // STYL 9: SCROLLING TITLE + VU METERS POD TEKSTEM
    // Góra: scrollujący tytuł
    // Środek: informacje techniczne
    // Dół: wskaźniki VU - pionowe słupki z pikami dla L/R
    
    if (!_player) return;
    
    // KRYTYCZNE: Wyczyść bufor przed rysowaniem
    _display.clearBuffer();
    
    String currentFile = _player->getCurrentFile();
    String originalFile9 = currentFile;  // Zachowaj dla getTrackNumber()
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "Brak utworu";
    } else {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) currentFile = currentFile.substring(0, dotPos);
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) currentFile = currentFile.substring(slashPos + 1);
        // Dodaj numer utworu z total
        int trackNumber = getTrackNumber(originalFile9);
        int totalTracks = getTotalTracks();
        if (trackNumber > 0 && totalTracks > 0) {
            currentFile = String(trackNumber) + "/" + String(totalTracks) + " " + currentFile;
        }
    }
    
    // === GÓRNA CZĘŚĆ: TYTUŁ UTWORU (scrollowany) ===
    _display.setFont(spleen6x12PL);
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > 250) {
        static int scrollOffset9 = 0;
        static unsigned long lastScrollTime9 = 0;
        
        if (millis() - lastScrollTime9 > 90) {
            scrollOffset9++;
            if (scrollOffset9 > titleWidth + 35) scrollOffset9 = 0;
            lastScrollTime9 = millis();
        }
        
        _display.setClipWindow(3, 0, 253, 14);
        _display.drawStr(3 - scrollOffset9, 12, currentFile.c_str());
        _display.drawStr(3 - scrollOffset9 + titleWidth + 35, 12, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        int centerX = (256 - titleWidth) / 2;
        _display.drawStr(centerX, 12, currentFile.c_str());
    }
    
    _display.drawLine(0, 15, 256, 15);
    
    // === INFORMACJE TECHNICZNE ===
    _display.setFont(spleen6x12PL);
    
    extern String streamCodec;
    extern String bitrateString;
    extern uint32_t SampleRate;
    extern uint8_t SampleRateRest;
    
    // Lewa strona
    _display.drawStr(4, 26, "Codec:");
    _display.drawStr(42, 26, streamCodec.c_str());
    
    _display.drawStr(4, 36, "Bitrate:");
    String bitrateInfo = bitrateString + "kbps";
    _display.drawStr(48, 36, bitrateInfo.c_str());
    
    // Prawa strona
    _display.drawStr(130, 26, "Sample:");
    String sampleInfo = String(SampleRate) + "." + String(SampleRateRest) + "kHz";
    _display.drawStr(178, 26, sampleInfo.c_str());
    
    // Czas odtwarzania
    uint32_t currTime = audio.getAudioCurrentTime();
    uint32_t durTime = audio.getAudioFileDuration();
    
    _display.drawStr(130, 36, "Time:");
    if (durTime > 0) {
        char timeStr[15];
        snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu/%02lu:%02lu",
                 currTime / 60, currTime % 60,
                 durTime / 60, durTime % 60);
        _display.drawStr(165, 36, timeStr);
    } else {
        char timeStr[10];
        snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu", currTime / 60, currTime % 60);
        _display.drawStr(165, 36, timeStr);
    }
    
    _display.drawLine(0, 40, 256, 40);
    
    // === VU METERS - PIONOWE SŁUPKI Z PIKAMI ===
    uint16_t vuRaw = audio.getVUlevel();
    uint8_t vuL = min(vuRaw >> 8, 255);      // Lewy kanał
    uint8_t vuR = min(vuRaw & 0xFF, 255);    // Prawy kanał
    
    // Parametry VU meterów
    int barHeight = 20;  // Wysokość słupków
    int barWidth = 80;   // Szerokość słupka
    int barY = 62;       // Dolna krawędź słupka
    
    // Lewy kanał (L)
    int vuHeightL = (vuL * barHeight) / 255;
    int barXL = 30;
    
    _display.setFont(spleen6x12PL);
    _display.drawStr(barXL + 35, 52, "L");
    _display.drawFrame(barXL, barY - barHeight, barWidth, barHeight);
    if (vuHeightL > 0) {
        _display.drawBox(barXL + 1, barY - vuHeightL, barWidth - 2, vuHeightL);
    }
    
    // Pik dla L (czerwona linia na szczyt)
    static int peakL = 0;
    static unsigned long peakTimeL = 0;
    if (vuHeightL > peakL) {
        peakL = vuHeightL;
        peakTimeL = millis();
    }
    if (millis() - peakTimeL > 1000) {
        peakL = max(0, peakL - 1);  // Opadanie piku
    }
    if (peakL > 0) {
        _display.drawLine(barXL + 1, barY - peakL, barXL + barWidth - 2, barY - peakL);
    }
    
    // Prawy kanał (R)
    int vuHeightR = (vuR * barHeight) / 255;
    int barXR = 146;
    
    _display.drawStr(barXR + 35, 52, "R");
    _display.drawFrame(barXR, barY - barHeight, barWidth, barHeight);
    if (vuHeightR > 0) {
        _display.drawBox(barXR + 1, barY - vuHeightR, barWidth - 2, vuHeightR);
    }
    
    // Pik dla R
    static int peakR = 0;
    static unsigned long peakTimeR = 0;
    if (vuHeightR > peakR) {
        peakR = vuHeightR;
        peakTimeR = millis();
    }
    if (millis() - peakTimeR > 1000) {
        peakR = max(0, peakR - 1);
    }
    if (peakR > 0) {
        _display.drawLine(barXR + 1, barY - peakR, barXR + barWidth - 2, barY - peakR);
    }
}


void SDPlayerOLED::renderStyle10() {
    // === STYL 10 - Floating Peaks (Ulatujące szczyty) - FFT Analyzer ===
    
    // KRYTYCZNE: Aktywuj analizator FFT dla tego stylu
    eq_analyzer_set_runtime_active(true);
    
    // KRYTYCZNE: Wyczyść bufor przed rysowaniem - blokuje przebijanie się radia
    _display.clearBuffer();
    
    // Pobierz poziomy FFT z analizatora
    float fftLevels[16];
    float fftPeaks[16];
    eq_get_analyzer_levels(fftLevels);
    eq_get_analyzer_peaks(fftPeaks);
    
    // Wygładzanie poziomów
    static uint8_t muteLevel[16] = {0};
    static float smoothedLevel[16] = {0.0f};
    const float smoothFactor = 0.55f; // 45% smoothness (100-45)/100
    
    bool isMuted = (_player->getVolume() == 0);
    
    for (uint8_t i = 0; i < 16; i++) {
        if (isMuted) {
            // Animacja wyciszenia - stopniowe zmniejszanie
            if (muteLevel[i] > 2) muteLevel[i] -= 2;
            else muteLevel[i] = 0;
            smoothedLevel[i] = 0.0f;
        } else {
            float lv = fftLevels[i];
            if (lv < 0.0f) lv = 0.0f;
            if (lv > 1.0f) lv = 1.0f;
            
            // Szybszy atak, wolniejsze opadanie
            if (lv > smoothedLevel[i]) {
                float attackSpeed = 0.3f + smoothFactor * 0.7f;
                smoothedLevel[i] = smoothedLevel[i] + attackSpeed * (lv - smoothedLevel[i]);
            } else {
                float releaseSpeed = 0.1f + smoothFactor * 0.6f;
                smoothedLevel[i] = smoothedLevel[i] + releaseSpeed * (lv - smoothedLevel[i]);
            }
            
            muteLevel[i] = (uint8_t)(smoothedLevel[i] * 100.0f + 0.5f);
        }
    }
    
    // === GÓRNY PASEK: Tytuł scrollowany na środku + Volume ===
    _display.setFont(spleen6x12PL);
    
    // === TYTUŁ SCROLLOWANY NA ŚRODKU ===
    String currentFile = _player->getCurrentFile();
    String originalFile10 = currentFile;  // Zachowaj dla getTrackNumber()
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "NO FILE";
    } else {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) currentFile = currentFile.substring(0, dotPos);
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) currentFile = currentFile.substring(slashPos + 1);
        // Dodaj numer utworu z total
        int trackNumber = getTrackNumber(originalFile10);
        int totalTracks = getTotalTracks();
        if (trackNumber > 0 && totalTracks > 0) {
            currentFile = String(trackNumber) + "/" + String(totalTracks) + " " + currentFile;
        }
    }
    
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > 200) {
        // Scrollowanie dla długiego tytułu
        static int scrollOffset10 = 0;
        static unsigned long lastScrollTime10 = 0;
        
        if (millis() - lastScrollTime10 > 100) {
            scrollOffset10++;
            if (scrollOffset10 > titleWidth + 30) scrollOffset10 = 0;
            lastScrollTime10 = millis();
        }
        
        _display.setClipWindow(4, 0, 200, 13);
        _display.drawStr(4 - scrollOffset10, 10, currentFile.c_str());
        _display.drawStr(4 - scrollOffset10 + titleWidth + 30, 10, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        // Wyśrodkowanie krótkiego tytułu
        int centerX = (200 - titleWidth) / 2;
        _display.drawStr(centerX, 10, currentFile.c_str());
    }
    
    // Volume po prawej
    int vol = _player->getVolume();
    char volStr[15];
    snprintf(volStr, sizeof(volStr), "Vol:%d", vol);
    int volW = _display.getStrWidth(volStr);
    _display.drawStr(256 - volW - 4, 10, volStr);
    
    _display.drawLine(0, 13, 256, 13);
    
    // === FLOATING PEAKS ANALYZER ===
    const uint8_t eqTopY = 14;
    const uint8_t eqBottomY = 58;  // Zmniejszone żeby zrobić miejsce na tekst na dole
    const uint8_t maxSegments = 32;
    
    // Parametry słupków - hardcoded (można później przenieść do cfg)
    const uint8_t barWidth = 12;
    const uint8_t barGap = 2;
    const uint8_t segmentHeight = 2;
    const uint8_t segmentGap = 1;
    const uint8_t maxPeaksActive = 3;
    const uint8_t peakHoldTime = 8;
    const uint8_t peakFloatSpeed = 8;
    
    const uint16_t totalBarsWidth = 16 * barWidth + 15 * barGap;
    int16_t startX = (256 - totalBarsWidth) / 2;
    if (startX < 2) startX = 2;
    
    // Floating peaks - wiele peaków na słupek
    static const uint8_t MAX_PEAKS_ARRAY = 5;
    struct FlyingPeak {
        float y;
        uint8_t holdCounter;
        bool active;
    };
    static FlyingPeak flyingPeaks[16][MAX_PEAKS_ARRAY] = {};
    static uint8_t lastPeakSeg[16] = {0};
    static bool wasRising[16] = {false};
    
    for (uint8_t i = 0; i < 16; i++) {
        uint8_t levelPercent = muteLevel[i];
        float peakVal = fftPeaks[i];
        if (peakVal < 0.0f) peakVal = 0.0f;
        if (peakVal > 1.0f) peakVal = 1.0f;
        uint8_t peakPercent = (uint8_t)(peakVal * 100.0f + 0.5f);
        
        uint8_t segments = (levelPercent * maxSegments) / 100;
        if (segments > maxSegments) segments = maxSegments;
        
        uint8_t peakSeg = (peakPercent * maxSegments) / 100;
        if (peakSeg > maxSegments) peakSeg = maxSegments;
        
        int16_t x = startX + i * (barWidth + barGap);
        
        // Rysuj słupki (segmenty)
        for (uint8_t s = 0; s < segments; s++) {
            int16_t segBottom = eqBottomY - (s * (segmentHeight + segmentGap));
            int16_t segTop = segBottom - segmentHeight + 1;
            if (segTop < eqTopY) segTop = eqTopY;
            if (segBottom > eqBottomY) segBottom = eqBottomY;
            if (segTop <= segBottom) {
                _display.drawBox(x, segTop, barWidth, segmentHeight);
            }
        }
        
        // Oblicz pozycję góry słupka
        int16_t barTopY = eqBottomY;
        if (segments > 0) {
            int16_t topSegBottom = eqBottomY - ((segments - 1) * (segmentHeight + segmentGap));
            barTopY = topSegBottom - segmentHeight + 1;
            if (barTopY < eqTopY) barTopY = eqTopY;
        }
        
        // Pozycja peak
        int16_t currentPeakY = eqBottomY;
        if (peakSeg > 0) {
            uint8_t ps = peakSeg - 1;
            int16_t peakSegBottom = eqBottomY - (ps * (segmentHeight + segmentGap));
            currentPeakY = peakSegBottom - segmentHeight + 1 - 2;
            if (currentPeakY < eqTopY) currentPeakY = eqTopY;
        }
        
        // Wykryj nowy szczyt - gdy peak rósł i teraz spada
        bool isRising = (peakSeg > lastPeakSeg[i]);
        bool justPeaked = (wasRising[i] && !isRising && peakSeg > 0);
        
        // Wystrzel nowy floating peak
        if (justPeaked) {
            for (uint8_t p = 0; p < maxPeaksActive; p++) {
                if (!flyingPeaks[i][p].active) {
                    flyingPeaks[i][p].y = (float)currentPeakY;
                    flyingPeaks[i][p].holdCounter = peakHoldTime;
                    flyingPeaks[i][p].active = true;
                    break;
                }
            }
        }
        
        wasRising[i] = isRising;
        lastPeakSeg[i] = peakSeg;
        
        // Aktualizuj i rysuj floating peaks
        for (uint8_t p = 0; p < MAX_PEAKS_ARRAY; p++) {
            if (flyingPeaks[i][p].active) {
                // Hold time - peak czeka
                if (flyingPeaks[i][p].holdCounter > 0) {
                    flyingPeaks[i][p].holdCounter--;
                } else {
                    // Peak odlatuje w górę
                    flyingPeaks[i][p].y -= (float)peakFloatSpeed * 0.5f;
                }
                
                // Rysuj peak TYLKO powyżej słupka
                int16_t peakY = (int16_t)flyingPeaks[i][p].y;
                if (peakY < barTopY && peakY >= eqTopY) {
                    _display.drawBox(x, peakY, barWidth, 1);
                } else if (peakY < eqTopY) {
                    flyingPeaks[i][p].active = false;
                }
            }
        }
    }
}

// ===== KONTROLA PILOTA =====

void SDPlayerOLED::onRemoteUp() {
    // Nawigacja działa zawsze (nie tylko w MODE_NORMAL)
    // aby po zatrzymaniu utworu można było przesuwać kursor
    if (_mode != MODE_SPLASH) {
        scrollUp();
        showActionMessage("UP");
    }
}

void SDPlayerOLED::onRemoteDown() {
    // Nawigacja działa zawsze (nie tylko w MODE_NORMAL)
    // aby po zatrzymaniu utworu można było przesuwać kursor
    if (_mode != MODE_SPLASH) {
        scrollDown();
        showActionMessage("DOWN");
    }
}

void SDPlayerOLED::onRemoteOK() {
    // Ignoruj w trybie splash
    if (_mode == MODE_SPLASH) return;
    
    // Jeśli coś jest odtwarzane lub w pauzie, OK działa jako Play/Pause
    if (_player && _player->isPlaying()) {
        bool wasPaused = _player->isPaused();
        _player->pause();  // Toggle play/pause
        showActionMessage(wasPaused ? "PLAY" : "PAUSE");
        Serial.println("SD Player: OK button - Toggle Play/Pause");
    } else {
        // KRYTYCZNE: Jeśli nic nie gra - ZAWSZE pozwól wybrać utwór
        // niezależnie od trybu (MODE_VOLUME, MODE_NORMAL, itp.)
        if (_mode == MODE_VOLUME) {
            _mode = MODE_NORMAL;  // Wyjdź z trybu volume
        }
        selectCurrent();
        showActionMessage("PLAY");
        Serial.println("SD Player: OK button - Select and play track");
    }
}

void SDPlayerOLED::onRemoteBack() {
    // Przycisk BACK - wyjście do katalogu nadrzędnego
    if (_mode == MODE_SPLASH) return;
    
    if (_mode == MODE_VOLUME) {
        _mode = MODE_NORMAL;  // Wyjdź z trybu volume
        Serial.println("SD Player: BACK button - exit volume mode");
    } else {
        goUp();  // Wyjście do parent directory
        Serial.println("SD Player: BACK button - go to parent directory");
    }
}

void SDPlayerOLED::onRemotePlayPause() {
    // Dedykowany przycisk Play/Pause
    if (_player) {
        bool wasPaused = _player->isPaused();
        _player->pause();  // Toggle play/pause
        showActionMessage(wasPaused ? "PLAY" : "PAUSE");
        Serial.println("SD Player: Play/Pause button pressed");
    }
}

void SDPlayerOLED::onRemoteSRC() {
    // Podwójne kliknięcie SRC = zmiana stylu wyświetlania
    unsigned long currentTime = millis();
    
    if (currentTime - _lastSrcPressTime > 1000) {
        // Timeout - reset licznika
        _srcClickCount = 0;
    }
    
    _srcClickCount++;
    _lastSrcPressTime = currentTime;
    
    if (_srcClickCount >= 2) {
        // Podwójne kliknięcie - zmień styl
        nextStyle();
        _srcClickCount = 0;
        Serial.printf("SDPlayer: Zmiana stylu OLED na %d\n", (int)_style);
    } else {
        // Pojedyncze - zmień InfoStyle (zegar/tytuł)
        nextInfoStyle();
    }
}

// ===== KOMUNIKATY AKCJI I IKONKI =====

void SDPlayerOLED::showActionMessage(const String& message) {
    _actionMessage = message;
    _actionMessageTime = millis();
    _showActionMessage = true;
}

void SDPlayerOLED::drawIconPrev(int x, int y) {
    // ⏮️ Previous - podwójny trójkąt w lewo
    _display.drawTriangle(x+8, y, x+8, y+8, x+3, y+4);
    _display.drawTriangle(x+3, y, x+3, y+8, x, y+4);
}

void SDPlayerOLED::drawIconUp(int x, int y) {
    // ⬆️ Up - strzałka w górę
    _display.drawTriangle(x+4, y, x, y+5, x+8, y+5);
    _display.drawLine(x+3, y+4, x+3, y+8);
    _display.drawLine(x+4, y+4, x+4, y+8);
    _display.drawLine(x+5, y+4, x+5, y+8);
}

void SDPlayerOLED::drawIconPause(int x, int y) {
    // ⏸️ Pause - dwie pionowe kreski
    _display.drawBox(x+1, y, 3, 8);
    _display.drawBox(x+6, y, 3, 8);
}

void SDPlayerOLED::drawIconPlay(int x, int y) {
    // ▶️ Play - trójkąt w prawo
    _display.drawTriangle(x, y, x, y+8, x+7, y+4);
}

void SDPlayerOLED::drawIconStop(int x, int y) {
    // ⏹️ Stop - kwadrat
    _display.drawBox(x+1, y+1, 7, 7);
}

void SDPlayerOLED::drawIconNext(int x, int y) {
    // ⏭️ Next - podwójny trójkąt w prawo
    _display.drawTriangle(x, y, x, y+8, x+5, y+4);
    _display.drawTriangle(x+5, y, x+5, y+8, x+10, y+4);
}

void SDPlayerOLED::drawIconDown(int x, int y) {
    // ⬇️ Down - strzałka w dół
    _display.drawLine(x+3, y, x+3, y+4);
    _display.drawLine(x+4, y, x+4, y+4);
    _display.drawLine(x+5, y, x+5, y+4);
    _display.drawTriangle(x+4, y+8, x, y+3, x+8, y+3);
}

int SDPlayerOLED::getTrackNumber(const String& currentFileName) {
    // Oblicza numer utworu (pomijając foldery) na podstawie nazwy pliku
    // Zwraca 0 jeśli plik nie został znaleziony lub lista jest pusta
    
    if (currentFileName.length() == 0 || currentFileName == "None" || _fileList.size() == 0) {
        return 0;
    }
    
    // POPRAWKA: _fileList zawiera teraz pełne ścieżki względne (np. "MUZYKA/song.mp3")
    // currentFileName może być:
    // - "/MUZYKA/song.mp3" (pełna ścieżka bezwzględna z main.cpp)
    // - "MUZYKA/song.mp3" (względna z _player->getCurrentFile())
    
    String searchName = currentFileName;
    // Usuń początkowy "/" jeśli jest
    if (searchName.startsWith("/")) {
        searchName = searchName.substring(1);
    }
    
    // Znajdź ten plik w _fileList używając pełnej ścieżki względnej
    int actualFileIndex = -1;
    for (int i = 0; i < _fileList.size(); i++) {
        if (!_fileList[i].isDir) {
            String listPath = _fileList[i].name;
            if (listPath.startsWith("/")) {
                listPath = listPath.substring(1);
            }
            
            if (listPath.equals(searchName)) {
                actualFileIndex = i;
                break;
            }
        }
    }
    
    // Jeśli znaleziono plik, oblicz jego numer (zliczając tylko pliki muzyczne przed nim)
    if (actualFileIndex >= 0) {
        int trackNumber = 0;
        for (int j = 0; j <= actualFileIndex; j++) {
            if (!_fileList[j].isDir) {
                trackNumber++;
            }
        }
        return trackNumber;
    }
    
    // Fallback: jeśli nie znaleziono w _fileList, użyj _selectedIndex
    if (_selectedIndex >= 0 && _selectedIndex < _fileList.size() && !_fileList[_selectedIndex].isDir) {
        int trackNumber = 0;
        for (int j = 0; j <= _selectedIndex; j++) {
            if (!_fileList[j].isDir) {
                trackNumber++;
            }
        }
        return trackNumber;
    }
    
    return 0;  // Nie znaleziono
}

int SDPlayerOLED::getTotalTracks() {
    // Zlicza wszystkie pliki muzyczne (nie foldery) w _fileList
    int totalTracks = 0;
    for (int i = 0; i < _fileList.size(); i++) {
        if (!_fileList[i].isDir) {
            totalTracks++;
        }
    }
    return totalTracks;
}

void SDPlayerOLED::drawControlIcons() {
    // Rysuje ikonki kontroli W GÓRNYM PASKU (obok zegara i daty)
    // Układ górnego paska: Zegar | Data | [⬆️] [⏸️/▶️] [⏹️] [⬇️] | Format | Głośnik+Vol
    
    int y = 3;         // Pozycja Y (w górnym pasku, wyrównane z tekstem)
    int spacing = 13;  // Zmniejszony odstęp dla 4 ikon
    int startX = 128;  // Przesunięte w lewo dla 4 ikon
    
    // Ikonka Up (przewijanie w górę)
    drawIconUp(startX, y);
    
    // Ikonka Pause/Play (w zależności od stanu)
    if (_player && _player->isPlaying() && !_player->isPaused()) {
        drawIconPause(startX + spacing, y);
    } else {
        drawIconPlay(startX + spacing, y);
    }
    
    // Ikonka Stop (całkowite zatrzymanie)
    drawIconStop(startX + spacing * 2, y);
    
    // Ikonka Down (przewijanie w dół)
    drawIconDown(startX + spacing * 3, y);
    
    // Jeśli jest aktywny komunikat, narysuj go PONIŻEJ górnego paska (pod kreską)
    if (_showActionMessage && (millis() - _actionMessageTime < 2000)) {
        _display.setFont(spleen6x12PL);
        int msgWidth = _display.getStrWidth(_actionMessage.c_str());
        int msgX = (256 - msgWidth) / 2;  // Wycentruj
        _display.drawStr(msgX, 28, _actionMessage.c_str());  // Obniżone o ~2mm (8 pikseli)
    } else {
        _showActionMessage = false;  // Ukryj po 2 sekundach
    }
}

void SDPlayerOLED::nextInfoStyle() {
    _infoStyle = (_infoStyle == INFO_CLOCK_DATE) ? INFO_TRACK_TITLE : INFO_CLOCK_DATE;
    Serial.printf("SD Player: Info style changed to %s\n", 
                  _infoStyle == INFO_CLOCK_DATE ? "CLOCK/DATE" : "TRACK TITLE");
}

void SDPlayerOLED::setSelectedIndex(int index) {
    if (index >= 0 && index < _fileList.size()) {
        _selectedIndex = index;
        Serial.printf("SD Player OLED: Selected index updated to %d\n", index);
    }
}

void SDPlayerOLED::onRemoteVolUp() {
    // Używamy funkcji volumeUp() z main.cpp dla pełnej synchronizacji z radiem
    volumeUp();
    
    // Synchronizuj z WebUI
    if (_player) {
        _player->setVolume(volumeValue);
    }
    
    _mode = MODE_VOLUME;
    _volumeShowTime = millis();
    Serial.printf("[SDPlayer] Remote VOL+ (%d)\n", volumeValue);
}

void SDPlayerOLED::onRemoteVolDown() {
    // Używamy funkcji volumeDown() z main.cpp dla pełnej synchronizacji z radiem
    volumeDown();
    
    // Synchronizuj z WebUI
    if (_player) {
        _player->setVolume(volumeValue);
    }
    
    _mode = MODE_VOLUME;
    _volumeShowTime = millis();
    Serial.printf("[SDPlayer] Remote VOL- (%d)\n", volumeValue);
}

void SDPlayerOLED::onRemoteStop() {
    if (!_player) return;
    
    _player->stop();
    sdPlayerPlayingMusic = false;
    
    // KRYTYCZNE: Przywróć tryb normalny aby nawigacja działała
    _mode = MODE_NORMAL;
    
    showActionMessage("STOP");
    Serial.println("SD Player: STOP button - Playback stopped, mode reset to NORMAL");
}

// ===== KONTROLA ENKODERA =====

void SDPlayerOLED::onEncoderLeft() {
    // Sprawdź w jakim trybie jesteśmy
    if (_encoderVolumeMode) {
        // TRYB KONTROLI GŁOŚNOŚCI - Lewo = Ciszej
        // KRYTYCZNE: Używamy funkcji volumeDown() z main.cpp dla pełnej synchronizacji z radiem
        volumeDown();
        
        // Synchronizuj z WebUI
        if (_player) {
            _player->setVolume(volumeValue);
        }
        
        _mode = MODE_VOLUME;
        _volumeShowTime = millis();
        _lastEncoderModeChange = millis();  // Resetuj timeout
        showActionMessage("VOL-");
        Serial.printf("[SDPlayer] Encoder volume down (%d)\n", volumeValue);
    } else {
        // TRYB NAWIGACJI - Lewo = Góra (w górę listy)
        scrollUp();
        showActionMessage("UP");
    }
}

void SDPlayerOLED::onEncoderRight() {
    // Sprawdź w jakim trybie jesteśmy
    if (_encoderVolumeMode) {
        // TRYB KONTROLI GŁOŚNOŚCI - Prawo = Głośniej
        // KRYTYCZNE: Używamy funkcji volumeUp() z main.cpp dla pełnej synchronizacji z radiem
        volumeUp();
        
        // Synchronizuj z WebUI
        if (_player) {
            _player->setVolume(volumeValue);
        }
        
        _mode = MODE_VOLUME;
        _volumeShowTime = millis();
        _lastEncoderModeChange = millis();  // Resetuj timeout
        showActionMessage("VOL+");
        Serial.printf("[SDPlayer] Encoder volume up (%d)\n", volumeValue);
    } else {
        // TRYB NAWIGACJI - Prawo = Dół (w dół listy)
        scrollDown();
        showActionMessage("DOWN");
    }
}

bool SDPlayerOLED::checkEncoderLongPress(bool buttonState) {
    unsigned long now = millis();
    
    if (buttonState && !_encoderButtonPressed) {
        // Przycisk właśnie został wciśnięty
        _encoderButtonPressed = true;
        _encoderButtonPressStart = now;
    } else if (!buttonState && _encoderButtonPressed) {
        // Przycisk został zwolniony
        _encoderButtonPressed = false;
    }
    
    // Sprawdź czy przycisk jest przytrzymany przez 4 sekundy
    if (_encoderButtonPressed && (now - _encoderButtonPressStart >= 4000)) {
        // DŁUGIE PRZYTRZYMANIE (4 sek) - WYJŚCIE DO RADIA
        Serial.println("SD Player: Long press detected (4s) - returning to radio");
        showActionMessage("EXIT");
        sdPlayerOLEDActive = false;
        sdPlayerPlayingMusic = false;
        _encoderButtonPressed = false; // Reset flagi
        deactivate();
        displayRadio();
        return true; // Zwracamy true - długie przytrzymanie wykryte
    }
    
    return false; // Brak długiego przytrzymania
}

void SDPlayerOLED::onEncoderButtonHold(unsigned long holdTime) {
    // Ta metoda może być wywołana z main.cpp gdy wykryje długie przytrzymanie
    // Dla kompatybilności - przenosimy do onEncoderButtonLongHold
    onEncoderButtonLongHold(holdTime);
}

void SDPlayerOLED::onEncoderButtonMediumHold(unsigned long holdTime) {
    // ŚREDNIE PRZYTRZYMANIE (3-5s) - ZATWIERDŹ WYBÓR UTWORU
    Serial.println("[SDPlayer] Medium hold (3-5s) - Select current track");
    showActionMessage("SELECT");
    
    // Wybierz i odtwórz aktualnie zaznaczony utwór
    selectCurrent();
}

void SDPlayerOLED::onEncoderButtonLongHold(unsigned long holdTime) {
    // DŁUGIE PRZYTRZYMANIE (>6s) - PRZEŁĄCZ TRYB VOLUME
    _encoderVolumeMode = !_encoderVolumeMode;
    _lastEncoderModeChange = millis();
    
    if (_encoderVolumeMode) {
        // Przełączono na tryb kontroli głośności
        _mode = MODE_VOLUME;
        _volumeShowTime = millis();
        showActionMessage("VOLUME MODE");
        Serial.println("[SDPlayer] Long hold (>6s) - Switched to VOLUME mode");
    } else {
        // Przełączono na tryb nawigacji po liście
        _mode = MODE_NORMAL;
        showActionMessage("NAVIGATION");
        Serial.println("[SDPlayer] Long hold (>6s) - Switched to NAVIGATION mode");
    }
}

void SDPlayerOLED::onEncoderButton() {
    // Nie reaguj jeśli przycisk był długo przytrzymany
    if (_encoderButtonPressed) {
        return;
    }
    
    unsigned long now = millis();
    
    // Wykrywanie potrójnego kliknięcia (w ciągu 600ms) dla wyjścia do radia
    if (now - _lastEncoderClickTime < 600) {
        _encoderClickCount++;
        
        if (_encoderClickCount >= 3) {
            // POTRÓJNE KLIKNIĘCIE - WYJŚCIE DO RADIA
            Serial.println("[SDPlayer] Triple click detected - returning to radio");
            showActionMessage("EXIT TO RADIO");
            sdPlayerOLEDActive = false;
            sdPlayerPlayingMusic = false;
            deactivate();
            
            // Przywróć zapisany bank i stację
            bank_nr = sdPlayerReturnBank;
            station_nr = sdPlayerReturnStation;
            Serial.printf("[SDPlayer] Encoder: Przywrócono pozycję: Bank %d, Stacja %d\n", bank_nr, station_nr);
            changeStation();
            
            displayRadio();
            _encoderClickCount = 0;
            return;
        }
        // Czekamy na ewentualne kolejne kliknięcia
    } else {
        _encoderClickCount = 1;
    }
    
    _lastEncoderClickTime = now;
    
    // Pojedyncze kliknięcie zostanie obsłużone przez timeout w loop() (650ms)
    // To pozwala wykryć potrójne kliknięcie przed wykonaniem akcji
}

// ===== NAWIGACJA =====

void SDPlayerOLED::scrollUp() {
    // LAZY LOADING: Załaduj listę plików przy pierwszym użyciu strzałek
    if (_fileList.empty()) {
        Serial.println("SD Player: Lazy loading file list on first arrow press - ASYNC MODE");
        // KRYTYCZNE: Użyj asynchronicznego skanowania aby uniknąć watchdog timeout
        extern bool sdPlayerScanRequested;
        sdPlayerScanRequested = true;  // Skanowanie w loop() - bezpiecznie dla async_tcp
        showActionMessage("LOADING...");
        return;
    }
    
    if (_selectedIndex > 0) {
        int oldIndex = _selectedIndex;
        _selectedIndex--;
        
        // KRYTYCZNE: Dostosuj scrollOffset aby kursor był widoczny  
        int visibleLines = 4;  // Liczba widocznych linii na ekranie
        if (_selectedIndex < _scrollOffset) {
            _scrollOffset = _selectedIndex;
        }
        
        // SYNCHRONIZACJA: Powiadom WebUI o zmianie indeksu
        notifyWebUIIndexChange(_selectedIndex);
    }
}

void SDPlayerOLED::scrollDown() {
    // LAZY LOADING: Załaduj listę plików przy pierwszym użyciu strzałek
    if (_fileList.empty()) {
        Serial.println("SD Player: Lazy loading file list on first arrow press - ASYNC MODE");
        // KRYTYCZNE: Użyj asynchronicznego skanowania aby uniknąć watchdog timeout
        extern bool sdPlayerScanRequested;
        sdPlayerScanRequested = true;  // Skanowanie w loop() - bezpiecznie dla async_tcp
        showActionMessage("LOADING...");
        return;
    }
    
    if (_selectedIndex < _fileList.size() - 1) {
        int oldIndex = _selectedIndex;
        _selectedIndex++;
        
        // KRYTYCZNE: Dostosuj scrollOffset aby kursor był widoczny
        int visibleLines = 4;  // Liczba widocznych linii na ekranie
        if (_selectedIndex >= _scrollOffset + visibleLines) {
            _scrollOffset = _selectedIndex - visibleLines + 1;
        }
        
        // SYNCHRONIZACJA: Powiadom WebUI o zmianie indeksu
        notifyWebUIIndexChange(_selectedIndex);
    }
}

void SDPlayerOLED::goUp() {
    // Wyjście do katalogu nadrzędnego (parent directory)
    if (!_player) return;
    
    String currentDir = _player->getCurrentDirectory();
    
    // Jeśli już jesteśmy w root, nie możemy iść wyżej
    if (currentDir == "/") {
        showActionMessage("ROOT DIR");
        Serial.println("[SDPlayer] Already at root directory: " + currentDir);
        return;
    }
    
    // Znajdź ostatni slash
    int lastSlash = currentDir.lastIndexOf('/');
    String parentDir;
    
    if (lastSlash == 0) {
        // Parent to root "/"
        parentDir = "/";
    } else if (lastSlash > 0) {
        // Parent to katalog przed ostatnim slashem
        parentDir = currentDir.substring(0, lastSlash);
    } else {
        // Nie powinno się zdarzyć, ale fallback to root
        parentDir = "/";
    }
    
    Serial.printf("[SDPlayer] Going UP: %s -> %s\n", currentDir.c_str(), parentDir.c_str());
    
    // Zmień katalog
    _player->changeDirectory(parentDir);
    
    // Odśwież listę plików
    refreshFileList();
    
    // Reset scroll i zaznaczenia
    _selectedIndex = 0;
    _scrollOffset = 0;
    
    showActionMessage("UP");
}

void SDPlayerOLED::selectCurrent() {
    // LAZY LOADING: Załaduj listę plików przy pierwszym użyciu OK
    if (_fileList.empty()) {
        Serial.println("SD Player: Lazy loading file list on OK press - ASYNC MODE");
        // KRYTYCZNE: Użyj asynchronicznego skanowania aby uniknąć watchdog timeout
        extern bool sdPlayerScanRequested;
        sdPlayerScanRequested = true;  // Skanowanie w loop() - bezpiecznie dla async_tcp
        showActionMessage("LOADING...");
        return;
    }
    
    if (!_player || _selectedIndex >= _fileList.size()) return;
    
    FileEntry& entry = _fileList[_selectedIndex];
    
    if (entry.isDir) {
        // Wejdź do katalogu
        String newPath = _player->getCurrentDirectory();
        if (newPath != "/") newPath += "/";
        newPath += entry.name;
        _player->changeDirectory(newPath);
        refreshFileList();
    } else {
        // ZATRZYMAJ OBECNY UTWÓR przed odtworzeniem nowego
        if (_player->isPlaying()) {
            Serial.println("DEBUG: Stopping current track before playing new one");
            _player->stop();  // Zatrzymaj obecny utwór
            delay(50);  // Krótka pauza dla stabilności
        }
        
        // Odtwórz nowy plik
        _player->playIndex(_selectedIndex);
        sdPlayerPlayingMusic = true;
        Serial.println("DEBUG: Playing selected track from SD Player");
        
        // POZOSTAJEMY w panelu SD Player OLED - nie wychodzimy automatycznie
        Serial.println("DEBUG: Music started from SD - staying in SD Player panel");
    }
}

void SDPlayerOLED::nextStyle() {
    DisplayStyle startStyle = _style;
    int attempts = 0;
    const int maxAttempts = 15;  // Maksymalnie 14 stylów + 1 na powrót do startu
    
    do {
        // Przejdź do następnego stylu w sekwencji
        switch (_style) {
            case STYLE_1: _style = STYLE_2; break;
            case STYLE_2: _style = STYLE_3; break;
            case STYLE_3: _style = STYLE_4; break;
            case STYLE_4: _style = STYLE_5; break;
            case STYLE_5: _style = STYLE_6; break;
            case STYLE_6: _style = STYLE_7; break;
            case STYLE_7: _style = STYLE_8; break;
            case STYLE_8: _style = STYLE_9; break;
            case STYLE_9: _style = STYLE_10; break;
            case STYLE_10: _style = STYLE_11; break;
            case STYLE_11: _style = STYLE_12; break;
            case STYLE_12: _style = STYLE_13; break;
            case STYLE_13: _style = STYLE_14; break;
            case STYLE_14: _style = STYLE_1; break;
        }
        
        // Sprawdź czy nowy styl jest włączony
        bool styleEnabled = false;
        switch (_style) {
            case STYLE_1: styleEnabled = sdPlayerStyle1Enabled; break;
            case STYLE_2: styleEnabled = sdPlayerStyle2Enabled; break;
            case STYLE_3: styleEnabled = sdPlayerStyle3Enabled; break;
            case STYLE_4: styleEnabled = sdPlayerStyle4Enabled; break;
            case STYLE_5: styleEnabled = sdPlayerStyle5Enabled; break;
            case STYLE_6: styleEnabled = sdPlayerStyle6Enabled; break;
            case STYLE_7: styleEnabled = sdPlayerStyle7Enabled; break;
            case STYLE_8: styleEnabled = true; break;
            case STYLE_9: styleEnabled = sdPlayerStyle9Enabled; break;
            case STYLE_10: styleEnabled = sdPlayerStyle10Enabled; break;
            case STYLE_11: styleEnabled = sdPlayerStyle11Enabled; break;
            case STYLE_12: styleEnabled = sdPlayerStyle12Enabled; break;
            case STYLE_13: styleEnabled = sdPlayerStyle13Enabled; break;
            case STYLE_14: styleEnabled = sdPlayerStyle14Enabled; break;
        }
        
        attempts++;
        
        // Jeśli styl włączony lub przekroczono limit prób, wyjdź z pętli
        if (styleEnabled || attempts >= maxAttempts) {
            break;
        }
        
    } while (_style != startStyle);  // Kontynuuj aż znajdziesz włączony styl lub wrócisz do startu
    
    Serial.printf("SDPlayer: Changed to style %d (enabled=%d)\n", _style, 
                  (_style == STYLE_1 ? sdPlayerStyle1Enabled :
                   _style == STYLE_2 ? sdPlayerStyle2Enabled :
                   _style == STYLE_3 ? sdPlayerStyle3Enabled :
                   _style == STYLE_4 ? sdPlayerStyle4Enabled :
                   _style == STYLE_5 ? sdPlayerStyle5Enabled :
                   _style == STYLE_6 ? sdPlayerStyle6Enabled :
                   _style == STYLE_7 ? sdPlayerStyle7Enabled :
                   _style == STYLE_8 ? true :
                   _style == STYLE_9 ? sdPlayerStyle9Enabled :
                   _style == STYLE_10 ? sdPlayerStyle10Enabled :
                   _style == STYLE_11 ? sdPlayerStyle11Enabled :
                   _style == STYLE_12 ? sdPlayerStyle12Enabled :
                   _style == STYLE_13 ? sdPlayerStyle13Enabled :
                   sdPlayerStyle14Enabled));
}

void SDPlayerOLED::setStyle(DisplayStyle style) {
    _style = style;
}
// ========== STYLE 11 - Bazujący na Radio Mode 0 (podstawowy) ==========
void SDPlayerOLED::renderStyle11() {
    // STYL 11: RADIO MODE 0 - dokładnie jak tryb 0 radyjka z DUŻĄ CZCIONKĄ i scrolling nazwą
    if (!_player) return;
    
    // KRYTYCZNE: Wyczyść bufor przed rysowaniem
    _display.clearBuffer();
    
    // Nazwa aktualnego pliku (bez rozszerzenia) - DUŻA CZCIONKA jak w trybie 0
    String currentFile = _player->getCurrentFile();
    String originalFile11 = currentFile;  // Zachowaj dla getTrackNumber()
    if (currentFile == "None") currentFile = "---";
    
    int dotPos = currentFile.lastIndexOf('.');
    if (dotPos > 0) {
        currentFile = currentFile.substring(0, dotPos);
    }
    
    // GÓRNA CZĘŚĆ: Numer + duża nazwa (jak w radyjku)
    int trackNum = getTrackNumber(originalFile11);
    if (trackNum == 0) trackNum = _selectedIndex + 1;  // Fallback
    
    // Biały kwadrat z numerem utworu (jak w trybie 0 radyjka)
    _display.drawRBox(1, 1, 21, 16, 4);
    _display.setDrawColor(0);  // Czarny tekst na białym tle
    _display.setFont(spleen6x12PL);
    char trackStr[4];
    snprintf(trackStr, sizeof(trackStr), "%02d", trackNum);
    _display.setCursor(4, 14);
    _display.print(trackStr);
    _display.setDrawColor(1);  // Powrót do białego
    
    // DUŻA CZCIONKA dla nazwy pliku u góry (jak w trybie 0 radyjka) - TYLKO skrócona
    _display.setFont(spleen6x12PL);
    if (currentFile.length() <= 15) {
        // Krótka nazwa - wyświetl całą
        _display.drawStr(24, 16, currentFile.c_str());
    } else {
        // Długa nazwa - skróć do 12 znaków + "..." (jak stationNameLenghtCut w radyjku)
        String shortName = currentFile.substring(0, 12) + "...";
        _display.drawStr(24, 16, shortName.c_str());
    }
    
    // ŚRODEK EKRANU (y=33): Scrolling pełna nazwa pliku (jak stationStringScroll w radyjku Mode 0)
    _display.setFont(spleen6x12PL);
    String scrollText = currentFile; // Pełna nazwa bez rozszerzenia
    int scrollW = _display.getStrWidth(scrollText.c_str());
    
    // Scrolling tylko jeśli tekst szerszy niż ekran (42 znaki, jak maxStationVisibleStringScrollLength)
    if (scrollW > 250) {
        _scrollPosition = (_scrollPosition + 1) % (scrollW + 20);
        int x = 0 - _scrollPosition;
        if (x < -scrollW) x = 0;
        
        // Rysuj tekst wielokrotnie aby zapełnić ekran (jak w displayRadioScroller)
        int xPos = x;
        do {
            _display.drawStr(xPos, 33, scrollText.c_str());
            xPos += scrollW + 20; // Odstęp między powtórzeniami
        } while (xPos < 256);
    } else {
        // Krótki tekst - wyświetl od lewej (jak w Radio Mode 0)
        _display.drawStr(0, 33, scrollText.c_str());
    }
    
    // Dolna linia separująca (jak w trybie 0)
    _display.drawLine(0, 52, 255, 52);
    
    // Format audio + informacje techniczne na dole (jak w trybie 0 radyjka)
    extern String streamCodec;
    extern String bitrateString;
    extern uint32_t SampleRate;
    extern uint8_t SampleRateRest;
    extern String bitsPerSampleString;
    
    // Lewa strona: parametry audio
    String displayString = String(SampleRate) + "." + String(SampleRateRest) + "kHz " + bitsPerSampleString + "bit " + bitrateString + "kbps";
    _display.setFont(spleen6x12PL);
    _display.drawStr(0, 62, displayString.c_str());
    
    // Środek: codec
    _display.drawStr(135, 62, streamCodec.c_str());
    
    // Prawa strona: Czas (jak w radyjku)
    _display.setFont(spleen6x12PL);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 5)) {
        char timeStr[10];
        bool showDots = (timeinfo.tm_sec % 2 == 0);
        if (showDots) snprintf(timeStr, sizeof(timeStr), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        else snprintf(timeStr, sizeof(timeStr), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);
        _display.drawStr(226, 62, timeStr);
    }
}

// ========== STYLE 12 - Bazujący na Radio Mode 1 (duży zegar) ==========
void SDPlayerOLED::renderStyle12() {
    // STYL 12: RADIO MODE 1 - DOKŁADNIE JAK W RADYJKU z zamiennikiem WiFi na status odtwarzania
    if (!_player) return;
    
    // KRYTYCZNE: Wyczyść bufor przed rysowaniem
    _display.clearBuffer();
    
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 5)) {
        // Duży zegar (7-segmentowy) - godziny i minuty DOKŁADNIE JAK W RADYJKU
        int xtime = 0;
        char timeString[10];
        bool showDots = (timeinfo.tm_sec % 2 == 0);
        if (showDots) snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        else snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);
        
        _display.setFont(u8g2_font_7Segments_26x42_mn);
        _display.drawStr(xtime+7, 45, timeString); 
        
        // KALENDARZ - dzień miesiąca (duża czcionka)
        _display.setFont(spleen6x12PL); // 14x11
        snprintf(timeString, sizeof(timeString), "%02d", timeinfo.tm_mday);
        _display.drawStr(203, 17, timeString);
        
        // Nazwa miesiąca (mała czcionka)
        String month = "";
        switch (timeinfo.tm_mon) {
            case 0: month = "JAN"; break;
            case 1: month = "FEB"; break;
            case 2: month = "MAR"; break;
            case 3: month = "APR"; break;
            case 4: month = "MAY"; break;
            case 5: month = "JUN"; break;
            case 6: month = "JUL"; break;
            case 7: month = "AUG"; break;
            case 8: month = "SEP"; break;
            case 9: month = "OCT"; break;
            case 10: month = "NOV"; break;
            case 11: month = "DEC"; break;
        }
        _display.setFont(spleen6x12PL);
        _display.drawStr(232, 14, month.c_str());
        
        // Dzień tygodnia (w białym boxie)
        String dayOfWeek = "";
        switch (timeinfo.tm_wday) {
            case 0: dayOfWeek = " Sunday  "; break;
            case 1: dayOfWeek = " Monday  "; break;
            case 2: dayOfWeek = " Tuesday "; break;
            case 3: dayOfWeek = "Wednesday"; break;
            case 4: dayOfWeek = "Thursday "; break;
            case 5: dayOfWeek = " Friday  "; break;
            case 6: dayOfWeek = "Saturday "; break;
        }
        
        _display.drawRBox(198, 20, 58, 15, 3);  // Box z zaokrąglonymi rogami, biały pod dniem tygodnia
        _display.drawLine(198, 20, 256, 20); // Linia separacyjna dzień miesiąc / dzień tygodnia
        _display.setDrawColor(0);
        _display.drawStr(201, 31, dayOfWeek.c_str());
        _display.setDrawColor(1);
        _display.drawRFrame(198, 0, 58, 35, 3); // Ramka na całości kalendarza
        
        // SEKUNDY W MAŁEJ CZCIONCE (jak w radyjku mode 1) - spleen6x12PL
        snprintf(timeString, sizeof(timeString), ":%02d", timeinfo.tm_sec);
        _display.setFont(spleen6x12PL);
        _display.drawStr(xtime+163, 45, timeString);
    }
    
    // Pozioma linia separująca (jak w trybie 1)
    _display.drawLine(0, 50, 255, 50);
    
    // --- POD KALENDARZEM: 3 INFORMACJE (GŁOŚNIK+VOLUME, STATUS, FORMAT) ---
    _display.setFont(spleen6x12PL);
    
    // 1. Ikonka głośnika i poziom głośności
    _display.drawGlyph(200, 47, 0x9E); // Symbol głośniczka
    if (isMutedState()) {
        _display.drawLine(198, 47, 208, 37);
        _display.drawLine(198, 46, 207, 37);
    }
    int vol = _player->getVolume();
    _display.drawStr(208, 47, String(vol).c_str());
    
    // 2. Status odtwarzania (ikony jak w CD playerze/magnetofonie)
    if (_player->isPlaying()) {
        if (_player->isPaused()) {
            // Paused: dwie pionowe kreski ||
            _display.drawStr(220, 47, "||");
        } else {
            // Playing: trójkąt w prawo (jak PLAY na magnetofonie)
            _display.drawStr(220, 47, ">");
        }
    } else {
        // Stopped: kwadrat (jak STOP na magnetofonie)
        _display.drawStr(220, 47, "[]");
    }
    
    // 3. Format/system pliku (MP3, FLAC, WAV, etc.)
    String currentFile = _player->getCurrentFile();
    String originalFile12 = currentFile;  // Zachowaj dla getTrackNumber()
    String fileExt = "";
    String fileName = currentFile;
    
    if (currentFile != "None") {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) {
            fileExt = currentFile.substring(dotPos + 1);
            fileExt.toUpperCase(); // MP3, FLAC, WAV
            if (fileExt.length() > 4) fileExt = fileExt.substring(0, 4); // Ogranicz do 4 znaków
            fileName = currentFile.substring(0, dotPos); // Nazwa bez rozszerzenia
        }
    } else {
        fileName = "Brak pliku";
    }
    
    if (fileExt == "") fileExt = "---";
    _display.drawStr(235, 47, fileExt.c_str());
    
    // DOLNA LINIA: Scrolling nazwa utworu (pełna szerokość - format już wyświetlony w linii 47)
    int trackNum = getTrackNumber(originalFile12);
    if (trackNum == 0) trackNum = _selectedIndex + 1;  // Fallback
    String scrollText = String(trackNum) + ". " + fileName;
    
    // Scrolling nazwa utworu (pełna szerokość ekranu)
    _display.setFont(spleen6x12PL);
    int scrollW = _display.getStrWidth(scrollText.c_str());
    if (scrollW > 250) { // Scrolling jeśli tekst szerszy niż ekran
        _scrollPosition = (_scrollPosition + 1) % (scrollW + 250);
        int x = 250 - _scrollPosition;
        if (x < -scrollW) x = 250;
        _display.drawStr(x, 62, scrollText.c_str());
    } else {
        _display.drawStr(0, 62, scrollText.c_str());
    }
}

// ========== STYLE 13 - Bazujący na Radio Mode 2 (3 linijki tekstu) ==========
void SDPlayerOLED::renderStyle13() {
    // STYL 13: RADIO MODE 2 - kompaktowy 3-linijkowy układ
    if (!_player) return;
    
    // KRYTYCZNE: Wyczyść bufor przed rysowaniem
    _display.clearBuffer();
    
    _display.setFont(spleen6x12PL);
    
    // Nazwa pliku na górze z numerem
    String currentFile = _player->getCurrentFile();
    String originalFile13 = currentFile;  // Zachowaj dla getTrackNumber()
    if (currentFile == "None") currentFile = "Brak pliku";
    
    int dotPos = currentFile.lastIndexOf('.');
    if (dotPos > 0) {
        currentFile = currentFile.substring(0, dotPos);
    }
    
    // Kwadrat z numerem (jak w trybie 2 radyjka)
    int trackNum = getTrackNumber(originalFile13);
    if (trackNum == 0) trackNum = _selectedIndex + 1;  // Fallback
    _display.drawRBox(1, 1, 18, 13, 4);
    _display.setDrawColor(0);
    char trackStr[4];
    snprintf(trackStr, sizeof(trackStr), "%02d", trackNum);
    _display.setCursor(3, 11);
    _display.print(trackStr);
    _display.setDrawColor(1);
    
    // Nazwa pliku obok numeru (skrócona aby się zmieściła)
    if (currentFile.length() > 35) {
        currentFile = currentFile.substring(0, 32) + "...";
    }
    _display.drawStr(23, 11, currentFile.c_str());
    
    // Status odtwarzania w środku (linijka 2)
    const char* status = _player->isPlaying() ? (_player->isPaused() ? "PAUSE" : "PLAY") : "STOP";
    int statusW = _display.getStrWidth(status);
    _display.drawStr((256 - statusW) / 2, 30, status);
    
    // === VU METER - POZIOMY (jak były poprzednio) ===
    uint16_t vuRaw = audio.getVUlevel();
    uint8_t vuL = min(vuRaw >> 8, 255);      // Lewy kanał
    uint8_t vuR = min(vuRaw & 0xFF, 255);    // Prawy kanał
    
    // Parametry VU metera poziomego
    int vuX = 10;         // Pozycja X początek
    int vuY = 38;         // Pozycja Y dla L
    int vuWidth = 235;    // Szerokość całkowita
    int vuHeight = 6;     // Wysokość paska
    int vuSpacing = 8;    // Odstęp między L i R
    
    // Oblicz długość wypełnienia
    int vuFillL = (vuL * vuWidth) / 255;
    int vuFillR = (vuR * vuWidth) / 255;
    
    // Lewy kanał (L)
    _display.setFont(spleen6x12PL);
    _display.drawStr(2, vuY + 5, "L");
    _display.drawFrame(vuX, vuY, vuWidth, vuHeight);
    if (vuFillL > 0) {
        _display.drawBox(vuX + 1, vuY + 1, vuFillL - 1, vuHeight - 2);
    }
    
    // Prawy kanał (R)
    int vuYr = vuY + vuSpacing;
    _display.drawStr(2, vuYr + 5, "R");
    _display.drawFrame(vuX, vuYr, vuWidth, vuHeight);
    if (vuFillR > 0) {
        _display.drawBox(vuX + 1, vuYr + 1, vuFillR - 1, vuHeight - 2);
    }
    
    _display.setFont(spleen6x12PL);  // Przywróć oryginalną czcionkę
    
    // Dolna linia separująca (jak w trybie 2)
    _display.drawLine(0, 52, 255, 52);
    
    // Dolna linijka: codec + samplerate + bitrate (jak w trybie 2 radyjka)
    extern String streamCodec;
    extern String bitrateString;
    extern uint32_t SampleRate;
    extern uint8_t SampleRateRest;
    
    // Lewa strona: samplerate + bitrate
    String displayString = String(SampleRate) + "." + String(SampleRateRest) + "kHz " + bitrateString + "kbps";
    _display.drawStr(0, 62, displayString.c_str());
    
    // Środek: codec
    _display.drawStr(135, 62, streamCodec.c_str());
    
    // Prawa strona: Czas odtwarzania utworu (MM:SS/MM:SS)
    _display.setFont(spleen6x12PL);
    uint32_t currTime13 = audio.getAudioCurrentTime(); // sekundy
    uint32_t durTime13 = audio.getAudioFileDuration();  // sekundy
    
    if (durTime13 > 0) {
        // Wyświetl czas z długością całkowitą
        char timeStr[15];
        snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu/%02lu:%02lu",
                 currTime13 / 60, currTime13 % 60,
                 durTime13 / 60, durTime13 % 60);
        _display.drawStr(185, 62, timeStr);
    } else {
        // Brak informacji o długości - pokaż sam aktualny czas
        char timeStr[10];
        snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu", 
                 currTime13 / 60, currTime13 % 60);
        _display.drawStr(226, 62, timeStr);
    }
}

// ========== STYLE 14 - Bazujący na Radio Mode 3 (linia statusu) ==========
void SDPlayerOLED::renderStyle14() {
    // STYL 14: RADIO MODE 3 - czas z zegara, duży tytuł w środku, scrolling na dole
    if (!_player) return;
    
    // KRYTYCZNE: Wyczyść bufor przed rysowaniem
    _display.clearBuffer();
    
    extern String streamCodec;
    extern String bitrateString;
    
    String currentFile = _player->getCurrentFile();
    String originalFile14 = currentFile;  // Zachowaj dla getTrackNumber()
    String fileName = currentFile;
    String fileExt = "";
    
    if (currentFile != "None") {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) {
            fileExt = currentFile.substring(dotPos + 1);
            fileExt.toUpperCase();
            fileName = currentFile.substring(0, dotPos);
        }
    } else {
        fileName = "Brak pliku";
    }
    
    // Numer utworu dla wyświetlania
    int trackNum = getTrackNumber(originalFile14);
    if (trackNum == 0) trackNum = _selectedIndex + 1;  // Fallback
    
    // ========== GÓRNA LINIA (y=11) - ZEGAR + VOLUME ==========
    _display.setFont(spleen6x12PL);
    
    // Lewa strona: Zegar systemowy
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        char timeStr[6];
        snprintf(timeStr, sizeof(timeStr), "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        _display.drawStr(4, 11, timeStr);
    }
    
    // Prawa strona: Volume z ikonką
    int vol = _player->getVolume();
    String volStr = String(vol);
    int volWidth = _display.getStrWidth(volStr.c_str());
    int speakerX = 256 - volWidth - 20;
    drawVolumeIcon(speakerX, 3);
    if (isMutedState()) {
        drawVolumeMuteSlash(speakerX, 3);
    }
    _display.drawStr(speakerX + 14, 11, volStr.c_str());
    
    // ========== ŚRODEK - DUŻA NAZWA UTWORU (wycentrowana) ==========
    // Dodaj numer utworu do nazwy
    String displayName = String(trackNum) + ". " + fileName;
    
    // Skróć nazwę jeśli za długa (max 20 znaków dla dużej czcionki)
    if (displayName.length() > 20) {
        displayName = displayName.substring(0, 17) + "...";
    }
    
    _display.setFont(spleen6x12PL); // Polska czcionka dla nazw utworów
    int nameWidth = _display.getStrWidth(displayName.c_str());
    int nameX = (256 - nameWidth) / 2; // Wycentruj
    _display.drawStr(nameX, 27, displayName.c_str()); // y=27 wyżej niż poprzednio
    
    // ========== STATUS ODTWARZANIA (PO ŚRODKU) ==========
    _display.setFont(spleen6x12PL);
    String statusText = "";
    if (_player->isPlaying()) {
        if (_player->isPaused()) {
            statusText = "PAUSE";
        } else {
            statusText = "PLAY";
        }
    } else {
        statusText = "STOP";
    }
    int statusW = _display.getStrWidth(statusText.c_str());
    int statusX = (256 - statusW) / 2;
    _display.drawStr(statusX, 42, statusText.c_str()); // y=42 pod nazwą
    
    // ========== DÓŁ - SCROLLING PEŁNA NAZWA Z NUMEREM ==========
    _display.setFont(spleen6x12PL);
    String scrollText = String(trackNum) + ". " + fileName + "    "; // Dodaj separatory
    int scrollW = _display.getStrWidth(scrollText.c_str());
    
    // Scrolling jeśli tekst szerszy niż ekran (jak w displayRadioScroller Mode 3)
    if (scrollW > 250) {
        _scrollPosition = (_scrollPosition + 1) % (scrollW + 20);
        int x = 0 - _scrollPosition;
        if (x < -scrollW) x = 0;
        
        // Rysuj tekst wielokrotnie aby zapełnić ekran
        int xPos = x;
        do {
            _display.drawStr(xPos, 52, scrollText.c_str()); // y=52 jak yPositionDisplayScrollerMode3
            xPos += scrollW;
        } while (xPos < 256);
    } else {
        // Krótki tekst - wyśrodkuj (jak w Radio Mode 3)
        int x = (256 - scrollW) / 2;
        _display.drawStr(x, 52, scrollText.c_str());
    }
    
    // Linia separująca nad dolnym paskiem
    _display.drawLine(0, 56, 255, 56);
    
    // ========== DOLNY PASEK (y=63) - INFORMACJE TECHNICZNE ==========
    extern uint32_t SampleRate;
    extern uint8_t SampleRateRest;
    
    // Lewa strona: Samplerate + format
    String infoStr = String(SampleRate) + "." + String(SampleRateRest) + "kHz";
    if (fileExt.length() > 0) {
        infoStr += " " + fileExt;
    }
    _display.setFont(spleen6x12PL);
    _display.drawStr(0, 62, infoStr.c_str());
    
    // Środek: Codec + Bitrate (informacje o utworze)
    String codecInfo = streamCodec + " " + bitrateString + "kbps";
    int codecW = _display.getStrWidth(codecInfo.c_str());
    _display.drawStr((256 - codecW) / 2, 62, codecInfo.c_str());
    
    // Prawa strona: Czas odtwarzania utworu (MM:SS/MM:SS)
    uint32_t currTimeSt14 = audio.getAudioCurrentTime(); // sekundy
    uint32_t durTimeSt14 = audio.getAudioFileDuration();  // sekundy
    
    String timeStr;
    if (durTimeSt14 > 0) {
        // Wyświetl czas z długością całkowitą
        char timeBuffer[15];
        snprintf(timeBuffer, sizeof(timeBuffer), "%02lu:%02lu/%02lu:%02lu",
                 currTimeSt14 / 60, currTimeSt14 % 60,
                 durTimeSt14 / 60, durTimeSt14 % 60);
        timeStr = String(timeBuffer);
    } else {
        // Brak informacji o długości - pokaż sam aktualny czas
        char timeBuffer[10];
        snprintf(timeBuffer, sizeof(timeBuffer), "%02lu:%02lu", 
                 currTimeSt14 / 60, currTimeSt14 % 60);
        timeStr = String(timeBuffer);
    }
    
    int timeW = _display.getStrWidth(timeStr.c_str());
    _display.drawStr(256 - timeW, 62, timeStr.c_str());
}

// ==================== SYNCHRONIZACJA Z WEBUI ====================

void SDPlayerOLED::notifyWebUIIndexChange(int newIndex) {
    // Powiadamia WebUI o zmianie indeksu z pilota/enkodera
    if (_oledIndexChangeCallback && newIndex != _selectedIndex) {
        _oledIndexChangeCallback(newIndex);
        Serial.printf("SDPlayerOLED: Notified WebUI about index change to %d\\n", newIndex);
    }
}

// ==================== SYNCHRONIZACJA LIST PLIKÓW ====================

void SDPlayerOLED::syncFileListFromWebUI(const std::vector<std::pair<String, bool>>& webUIFileList, const String& currentDir) {
    // KRYTYCZNE: Synchronizuj listę plików z WebUI aby uniknąć lazy loading przy każdym scroll
    _fileList.clear();
    
    // Konwertuj FileItem (name, isDir) na FileEntry
    for (const auto& item : webUIFileList) {
        FileEntry entry;
        entry.name = item.first;   // String name
        entry.isDir = item.second; // bool isDir
        _fileList.push_back(entry);
    }
    
    Serial.printf("[SDPlayerOLED] Lista plików zsynchronizowana: %d elementów w %s\n", 
                  _fileList.size(), currentDir.c_str());
    Serial.println("[SDPlayerOLED] LAZY LOADING WYŁĄCZONY - lista jest już załadowana!");
}

void SDPlayerOLED::syncTrackListFromIR(const String* trackFiles, int trackCount) {
    // KRYTYCZNE: Synchronizuj listę plików z IR pilot (trackFiles[] z main.cpp)
    // Ta funkcja wypełnia _fileList[] na podstawie tablicy trackFiles[]
    _fileList.clear();
    
    for (int i = 0; i < trackCount; i++) {
        FileEntry entry;
        entry.name = trackFiles[i];
        entry.isDir = false;  // trackFiles[] zawiera tylko pliki muzyczne, nie foldery
        _fileList.push_back(entry);
    }
    
    Serial.printf("[SDPlayerOLED] Lista utworów IR zsynchronizowana: %d plików\n", _fileList.size());
}