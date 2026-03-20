#include "SDPlayerWebUI.h"
#include "Audio.h"
#include "SDPlayerOLED.h"
#include "EQ_FFTAnalyzer.h"  // Dla eq_analyzer_set_sdplayer_mode()

// Extern zmienne z main.cpp
// STORAGE to makro: #define STORAGE (*_storage)
extern fs::FS* _storage;  // Wskaźnik do aktywnego systemu plików (SD lub LittleFS)
#define STORAGE (*_storage)

extern bool sdPlayerScanRequested; // Flaga żądania skanowania katalogu (obsługa w loop())

SDPlayerWebUI::SDPlayerWebUI() 
    : _server(nullptr),
      _audio(nullptr),
      _oled(nullptr),
      _exitCallback(nullptr),      _indexChangeCallback(nullptr),
      _fileChangeCallback(nullptr),
      _playStateCallback(nullptr),      _currentDir("/"),  // Zmiana: "/" zamiast "/MUZYKA" - pokazuj root z folderami MUZYKA i RECORDINGS
      _currentFile("None"),
    _pausedFilePath(""),
    _pausedPositionSec(0),
        _pausedFilePositionBytes(0),
      _volume(7),
      _isPlaying(false),
      _isPaused(false),
      _selectedIndex(-1) {
}

void SDPlayerWebUI::begin(AsyncWebServer* server, Audio* audioPtr) {
    _server = server;
    _audio = audioPtr;
    
    // Serial.println("SDPlayerWebUI: Registering routes...");
    
    // WAŻNE: API endpoints NAJPIERW - muszą być przed /sdplayer
    // ESPAsyncWebServer dopasowuje pierwszy pasujący route
    
    _server->on("/sdplayer/api/list", HTTP_GET, [this](AsyncWebServerRequest *request){
        // Serial.println("SDPlayerWebUI: /sdplayer/api/list requested");
        this->handleList(request);
    });
    
    _server->on("/sdplayer/api/play", HTTP_POST, [this](AsyncWebServerRequest *request){
        // Serial.println("SDPlayerWebUI: /sdplayer/api/play requested");
        this->handlePlay(request);
    });
    
    _server->on("/sdplayer/api/playSelected", HTTP_POST, [this](AsyncWebServerRequest *request){
        // Serial.println("SDPlayerWebUI: /sdplayer/api/playSelected requested");
        this->handlePlaySelected(request);
    });
    
    _server->on("/sdplayer/api/pause", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handlePause(request);
    });
    
    _server->on("/sdplayer/api/stop", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleStop(request);
    });
    
    _server->on("/sdplayer/api/next", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleNext(request);
    });
    
    _server->on("/sdplayer/api/prev", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handlePrev(request);
    });
    
    _server->on("/sdplayer/api/vol", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleVol(request);
    });
    
    _server->on("/sdplayer/api/seek", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleSeek(request);
    });
    
    _server->on("/sdplayer/api/cd", HTTP_GET, [this](AsyncWebServerRequest *request){
        this->handleCd(request);
    });
    
    _server->on("/sdplayer/api/up", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleUp(request);
    });
    
    _server->on("/sdplayer/api/back", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleBack(request);
    });
    
    // NOWE ENDPOINTY: Przełączanie stylów OLED
    _server->on("/sdplayer/api/nextStyle", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleNextStyle(request);
    });
    
    _server->on("/sdplayer/api/setStyle", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleSetStyle(request);
    });
    
    _server->on("/sdplayer/api/getStyle", HTTP_GET, [this](AsyncWebServerRequest *request){
        this->handleGetStyle(request);
    });
    
    // NOWY: Usuwanie plików (tylko RECORDINGS)
    _server->on("/sdplayer/api/delete", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleDelete(request);
    });
    
    // Główna strona SD Player - NA KOŃCU!
    _server->on("/sdplayer", HTTP_GET, [this](AsyncWebServerRequest *request){
        // Serial.println("SDPlayerWebUI: /sdplayer requested");
        this->handleRoot(request);
    });
    
    // Inicjalizacja
    
    // Sprawdź czy folder /MUZYKA istnieje
    File muzyka = STORAGE.open("/MUZYKA");
    if (!muzyka || !muzyka.isDirectory()) {
        Serial.println("[SDPlayer WebUI] Folder /MUZYKA nie istnieje - tworzę...");
        STORAGE.mkdir("/MUZYKA");
        if (muzyka) muzyka.close();
    } else {
        muzyka.close();
    }
    
    // NIE skanuj przy starcie - to blokuje system!
    // Lista załaduje się automatycznie gdy strona zażąda danych (handleList)
    Serial.println("[SDPlayer WebUI] Zainicjalizowano - lista załaduje się przy pierwszym żądaniu");
}

void SDPlayerWebUI::setExitCallback(std::function<void()> callback) {
    _exitCallback = callback;
}

void SDPlayerWebUI::setOLED(SDPlayerOLED* oled) {
    _oled = oled;
    if (_oled) {
        // Serial.println("SDPlayerWebUI: OLED display connected");
    }
}

void SDPlayerWebUI::handleRoot(AsyncWebServerRequest *request) {
    // Serial.println("SDPlayerWebUI: handleRoot called");
    // Serial.printf("SDPlayerWebUI: Request URL: %s\n", request->url().c_str());
    // Serial.printf("SDPlayerWebUI: Sending HTML, size=%d bytes\n", strlen_P(SDPLAYER_HTML));
    
    // KRYTYCZNE: ZAWSZE przełącz na tryb SDPlayer przy wejściu na stronę
    if (_oled) {
        extern bool sdPlayerOLEDActive;
        extern bool displayActive;
        extern bool timeDisplay;
        extern U8G2 u8g2;  // Główny obiekt wyświetlacza
        
        Serial.println("========================================");
        Serial.println("SDPlayerWebUI: handleRoot() - FORCE ACTIVATION START");
        Serial.printf("Before: sdPlayerOLEDActive=%s _oled->isActive()=%s\n", 
                     sdPlayerOLEDActive ? "TRUE" : "FALSE",
                     _oled->isActive() ? "TRUE" : "FALSE");
        
        // PODWÓJNA BLOKADA: Ustaw flagę PRZED jakąkolwiek operacją
        sdPlayerOLEDActive = true;
        
        // WYMUSZENIE: Zatrzymaj wszystkie tryby radio
        displayActive = false;
        timeDisplay = false;
        
        // Wyczyść bufor wyświetlacza przed przełączeniem
        u8g2.clearBuffer();
        u8g2.sendBuffer();
        delay(150);  // Wydłużona pauza dla stabilności
        
        // Aktywuj OLED SDPlayer (nawet jeśli już był aktywny)
        _oled->activate();
        
        // Włącz 3x większą dynamikę analizatora dla SDPlayera
        eq_analyzer_set_sdplayer_mode(true);
        
        // ZAWSZE pokaż splash screen "SD PLAYER" dla potwierdzenia przełączenia
        _oled->showSplash();
        
        Serial.printf("After: sdPlayerOLEDActive=%s _oled->isActive()=%s\n", 
                     sdPlayerOLEDActive ? "TRUE" : "FALSE",
                     _oled->isActive() ? "TRUE" : "FALSE");
        Serial.println("SDPlayerWebUI: handleRoot() - FORCE ACTIVATION COMPLETE");
        Serial.println("========================================");
    }
    
    request->send_P(200, "text/html; charset=utf-8", SDPLAYER_HTML);
    // Serial.println("SDPlayerWebUI: HTML sent");
}

void SDPlayerWebUI::handleList(AsyncWebServerRequest *request) {
    // Serial.println("========================================");
    // Serial.println("SDPlayerWebUI: handleList called");
    // Serial.printf("SDPlayerWebUI: Request URL: %s\n", request->url().c_str());
    // Serial.println("========================================");
    
    // LAZY LOADING: Skanuj tylko jeśli:
    // 1. Lista jest pusta (pierwsze otwarcie)
    // 2. Parametr ?refresh=1 w URL (wymuszenie odświeżenia)
    bool needScan = (_fileList.size() == 0);
    if (request->hasParam("refresh")) {
        needScan = true;
    }
    
    if (needScan) {
        Serial.println("[SDPlayer WebUI] Żądanie skanowania katalogu: " + _currentDir);
        extern bool sdPlayerScanRequested;
        sdPlayerScanRequested = true;  // Ustaw flagę - skanowanie w loop()
        
        // NIE CZEKAJ synchronicznie - pozwól loop() obsłużyć w tle
        // Zwróć obecny stan listy, nastąpi odświeżenie przy następnym żądaniu
        Serial.println("[SDPlayer WebUI] Skanowanie zainicjowane w tle - brak busy waiting");
    }
    
    DynamicJsonDocument doc(4096);
    doc["cwd"] = _currentDir;
    doc["now"] = _currentFile;
    
    // Status odtwarzania
    if (_isPaused) {
        doc["status"] = "Paused";
    } else if (_isPlaying) {
        doc["status"] = "Playing";
    } else {
        doc["status"] = "Stopped";
    }
    
    // Synchronizuj volume z globalnym Audio
    if (_audio) {
        _volume = _audio->getVolume();
    }
    doc["vol"] = _volume;
    
    // SYNCHRONIZACJA: Sprawdź aktualny stan z OLED jeśli aktywny
    extern bool sdPlayerOLEDActive;
    
    // KRYTYCZNE: Jeśli OLED nie jest aktywny, aktywuj go przy pierwszym żądaniu listy
    if (_oled && !_oled->isActive()) {
        extern bool displayActive;
        extern bool timeDisplay;
        extern U8G2 u8g2;
        
        // Wyłącz tryb radio
        displayActive = false;
        timeDisplay = false;
        
        Serial.println("SDPlayerWebUI: EMERGENCY activation from handleList() - OLED was not active!");
        
        // Wyczyść bufor wyświetlacza
        u8g2.clearBuffer();
        u8g2.sendBuffer();
        delay(50);
        
        // Aktywuj OLED SDPlayer
        _oled->activate();
        sdPlayerOLEDActive = true;
        
        // Włącz 3x większą dynamikę analizatora dla SDPlayera
        eq_analyzer_set_sdplayer_mode(true);
        
        Serial.println("SDPlayerWebUI: Auto-activated OLED from handleList() - first list request");
    } else if (_oled && sdPlayerOLEDActive) {
        Serial.println("SDPlayerWebUI: handleList() - SDPlayer already active, good!");
    }
    
    if (_oled && _oled->isActive() && sdPlayerOLEDActive) {
        // Synchronizuj wybrany indeks z OLED
        int oledSelectedIndex = _oled->getSelectedIndex();
        if (oledSelectedIndex >= 0 && oledSelectedIndex < _fileList.size()) {
            _selectedIndex = oledSelectedIndex;
            // Serial.printf("[WebUI] Synchronized selectedIndex from OLED: %d\\n", _selectedIndex);
        }
    }
    
    // Dodaj szczegółowe informacje o utworze
    if (_audio && _isPlaying) {
        // Czasy odtwarzania
        doc["currentTime"] = _audio->getAudioCurrentTime();  // Aktualny czas w sekundach
        doc["totalTime"] = _audio->getAudioFileDuration();   // Całkowity czas w sekundach
        
        // Informacje techniczne
        doc["bitRate"] = _audio->getBitRate();               // Bitrate (kbps)
        doc["sampleRate"] = _audio->getSampleRate();         // Sample rate (Hz)
        doc["bitsPerSample"] = _audio->getBitsPerSample();   // Bits per sample
        doc["channels"] = _audio->getChannels();             // Liczba kanałów (1=mono, 2=stereo)
        
        // Codec/Format
        if (_audio->getCodecname()) {
            doc["codec"] = String(_audio->getCodecname());   // MP3, FLAC, WAV etc.
        } else {
            doc["codec"] = "MP3";
        }
        
        // Album - wyciągnij ostatni folder ze ścieżki (jak w Stylu 8)
        String album = "-";
        if (_currentDir.length() > 0) {
            int lastSlash = _currentDir.lastIndexOf('/');
            if (lastSlash >= 0 && lastSlash < _currentDir.length() - 1) {
                album = _currentDir.substring(lastSlash + 1);
            } else if (_currentDir != "/" && lastSlash < 0) {
                album = _currentDir;
            }
        }
        doc["album"] = album;
        
        // VU Meter - pobierz bezpośrednio z Audio (dla SDPlayer)
        uint16_t rawVU = _audio->getVUlevel();
        doc["vuL"] = (rawVU >> 8) & 0xFF;  // Lewy kanał z Audio
        doc["vuR"] = rawVU & 0xFF;         // Prawy kanał z Audio
    } else {
        // Gdy nie odtwarzamy - domyślne wartości
        doc["currentTime"] = 0;
        doc["totalTime"] = 0;
        doc["bitRate"] = 0;
        doc["sampleRate"] = 44100;
        doc["bitsPerSample"] = 16;
        doc["channels"] = 2;
        doc["codec"] = "MP3";
        doc["album"] = "-";
        doc["vuL"] = 0;  // VU wyłączone gdy brak muzyki
        doc["vuR"] = 0;
    }

    JsonArray items = doc.createNestedArray("items");
    buildFileList(items);
    
    String response;
    serializeJson(doc, response);
    // Serial.println("SDPlayerWebUI: Sending JSON: " + response);
    request->send(200, "application/json; charset=utf-8", response);
}

void SDPlayerWebUI::handlePlay(AsyncWebServerRequest *request) {
    // Debouncing: Blokuj zbyt częste wywołania play (zapobiega stack overflow)
    static unsigned long lastPlayTime = 0;
    unsigned long now = millis();
    
    if (now - lastPlayTime < 500) { // Minimalna przerwa 500ms między play
        Serial.println("SDPlayerWebUI: Play request IGNORED (debouncing)");
        request->send(200, "text/plain; charset=utf-8", "OK");
        return;
    }
    lastPlayTime = now;
    
    // Serial.println("SDPlayerWebUI: handlePlay called");
    if (request->hasParam("i")) {
        int idx = request->getParam("i")->value().toInt();
        Serial.printf("[SDPlayerWebUI] Playing index: %d from web interface\n", idx);
        playIndex(idx);
    }
    request->send(200, "text/plain; charset=utf-8", "OK");
}

void SDPlayerWebUI::handlePlaySelected(AsyncWebServerRequest *request) {
    // Debouncing: Blokuj zbyt częste wywołania (zapobiega stack overflow)
    static unsigned long lastPlaySelectedTime = 0;
    unsigned long now = millis();
    
    if (now - lastPlaySelectedTime < 500) { // Minimalna przerwa 500ms
        Serial.println("SDPlayerWebUI: PlaySelected request IGNORED (debouncing)");
        request->send(200, "text/plain; charset=utf-8", "OK");
        return;
    }
    lastPlaySelectedTime = now;
    
    // Serial.printf("SDPlayerWebUI: handlePlaySelected called, index=%d, listSize=%d\n", _selectedIndex, _fileList.size());
    if (_selectedIndex >= 0 && _selectedIndex < _fileList.size()) {
        playIndex(_selectedIndex);
    } else {
        // Serial.println("SDPlayerWebUI: No file selected or invalid index");
    }
    request->send(200, "text/plain; charset=utf-8", "OK");
}

void SDPlayerWebUI::handlePause(AsyncWebServerRequest *request) {
    // Debouncing: Blokuj zbyt częste wywołania pause/resume (zapobiega stack overflow)
    static unsigned long lastPauseTime = 0;
    unsigned long now = millis();
    
    if (now - lastPauseTime < 200) { // Minimalna przerwa 200ms między pause/resume
        Serial.println("SDPlayerWebUI: Pause request IGNORED (debouncing)");
        request->send(200, "text/plain; charset=utf-8", "OK");
        return;
    }
    lastPauseTime = now;
    
    // Serial.println("SDPlayerWebUI: handlePause called");
    pause();
    request->send(200, "text/plain; charset=utf-8", "OK");
}

void SDPlayerWebUI::handleStop(AsyncWebServerRequest *request) {
    // Debouncing: Blokuj zbyt częste wywołania (zapobiega stack overflow)
    static unsigned long lastStopTime = 0;
    unsigned long now = millis();
    
    if (now - lastStopTime < 200) { // Minimalna przerwa 200ms
        Serial.println("SDPlayerWebUI: Stop request IGNORED (debouncing)");
        request->send(200, "text/plain; charset=utf-8", "OK");
        return;
    }
    lastStopTime = now;
    
    // Serial.println("SDPlayerWebUI: handleStop called");
    stop();
    request->send(200, "text/plain; charset=utf-8", "OK");
}

void SDPlayerWebUI::handleNext(AsyncWebServerRequest *request) {
    // Debouncing: Blokuj zbyt częste wywołania (zapobiega stack overflow)
    static unsigned long lastNextTime = 0;
    unsigned long now = millis();
    
    if (now - lastNextTime < 200) { // Minimalna przerwa 200ms
        Serial.println("SDPlayerWebUI: Next request IGNORED (debouncing)");
        request->send(200, "text/plain; charset=utf-8", "OK");
        return;
    }
    lastNextTime = now;
    
    // Serial.println("SDPlayerWebUI: handleNext called");
    next();
    request->send(200, "text/plain; charset=utf-8", "OK");
}

void SDPlayerWebUI::handlePrev(AsyncWebServerRequest *request) {
    // Debouncing: Blokuj zbyt częste wywołania (zapobiega stack overflow)
    static unsigned long lastPrevTime = 0;
    unsigned long now = millis();
    
    if (now - lastPrevTime < 200) { // Minimalna przerwa 200ms
        Serial.println("SDPlayerWebUI: Prev request IGNORED (debouncing)");
        request->send(200, "text/plain; charset=utf-8", "OK");
        return;
    }
    lastPrevTime = now;
    
    // Serial.println("SDPlayerWebUI: handlePrev called");
    prev();
    request->send(200, "text/plain; charset=utf-8", "OK");
}

void SDPlayerWebUI::handleVol(AsyncWebServerRequest *request) {
    if (request->hasParam("v")) {
        int vol = request->getParam("v")->value().toInt();
        setVolume(vol);
    }
    request->send(200, "text/plain; charset=utf-8", "OK");
}

void SDPlayerWebUI::handleSeek(AsyncWebServerRequest *request) {
    if (request->hasParam("s")) {
        // Relative seek (s=±5 for ±5 seconds)
        int deltaSeconds = request->getParam("s")->value().toInt();
        seekRelative(deltaSeconds);
    } else if (request->hasParam("pos")) {
        // Absolute seek (pos=120 for 2:00)
        int targetSeconds = request->getParam("pos")->value().toInt();
        seekAbsolute(targetSeconds);
    }
    request->send(200, "text/plain; charset=utf-8", "OK");
}

void SDPlayerWebUI::handleCd(AsyncWebServerRequest *request) {
    // Serial.println("SDPlayerWebUI: handleCd called");
    if (request->hasParam("p")) {
        String path = request->getParam("p")->value();
        // Serial.println("Changing directory to: " + path);
        changeDirectory(path);
    }
    request->send(200, "text/plain; charset=utf-8", "OK");
}

void SDPlayerWebUI::handleUp(AsyncWebServerRequest *request) {
    Serial.println("SDPlayerWebUI: handleUp called");
    
    // BEZPIECZNE: Tylko zmień ścieżkę, nie skanuj od razu
    if (_currentDir == "/") {
        request->send(200, "text/plain; charset=utf-8", "Already at root");
        return;
    }
    
    int lastSlash = _currentDir.lastIndexOf('/');
    if (lastSlash == 0) {
        _currentDir = "/";
    } else if (lastSlash > 0) {
        _currentDir = _currentDir.substring(0, lastSlash);
    }
    
    // KRYTYCZNE: Wyczyść listę i ustaw flagę skanowania dla lazy loading
    _fileList.clear();
    extern bool sdPlayerScanRequested;
    sdPlayerScanRequested = true;  // Skanowanie w loop() aby uniknąć timeout
    
    Serial.println("SDPlayerWebUI: Up to directory: " + _currentDir);
    request->send(200, "text/plain; charset=utf-8", "OK");
}

void SDPlayerWebUI::handleBack(AsyncWebServerRequest *request) {
    Serial.println("========================================");
    Serial.println("SDPlayerWebUI: handleBack called - returning to radio");
    
    // KRYTYCZNE: Reset globalnej flagi PRZED wszystkimi innymi operacjami
    extern bool sdPlayerOLEDActive;
    sdPlayerOLEDActive = false;
    Serial.println("SDPlayerWebUI: sdPlayerOLEDActive set to FALSE");
    
    // Zatrzymaj odtwarzanie przed wyjściem
    stop();
    
    // Deaktywuj OLED display
    if (_oled && _oled->isActive()) {
        _oled->deactivate();
        Serial.println("SDPlayerWebUI: OLED deactivated");
    }
    
    // WYMUSZENIE: Przywróć tryby radia
    extern bool displayActive;
    extern bool timeDisplay;
    displayActive = true;
    timeDisplay = true;
    Serial.println("SDPlayerWebUI: Radio display modes restored");
    
    if (_exitCallback) {
        _exitCallback();
    }
    
    Serial.println("SDPlayerWebUI: Back to radio completed successfully");
    Serial.println("========================================");
    request->send(200, "text/plain; charset=utf-8", "OK");
}

void SDPlayerWebUI::playFile(const String& path) {
    _currentFile = path;
    _pausedFilePath = "";
    _pausedPositionSec = 0;
    _pausedFilePositionBytes = 0;
    _isPlaying = true;
    _isPaused = false;
    // Serial.println("SDPlayerWebUI: Playing file: " + path);
    
    if (_audio) {
        _audio->stopSong();  // Zatrzymaj obecną muzykę
        if (_audio->connecttoFS(STORAGE, path.c_str())) {
            // Serial.println("SDPlayerWebUI: Audio started playing from SD");
            // Ustaw flagę globalną
            extern bool sdPlayerPlayingMusic;
            sdPlayerPlayingMusic = true;
            
            // SYNCHRONIZACJA: Powiadom o zmianach
            notifyFileChange(path);
            notifyPlayStateChange(true);
        } else {
            // Serial.println("SDPlayerWebUI: ERROR - Failed to play file!");
            _isPlaying = false;
            notifyPlayStateChange(false);
        }
    } else {
        // Serial.println("SDPlayerWebUI: ERROR - Audio pointer is NULL!");
    }
    
    // SYNCHRONIZACJA: Odśwież OLED i aktywuj go jeśli nieaktywny
    if (_oled) {
        // Automatyczne przełączenie OLED na SDPlayer tryb
        extern bool sdPlayerOLEDActive;
        extern bool displayActive;
        extern bool timeDisplay;
        
        if (!_oled->isActive()) {
            // Wyłącz tryb radio
            displayActive = false;
            timeDisplay = false;
            
            // Aktywuj OLED SDPlayer
            _oled->activate();
            sdPlayerOLEDActive = true;
            
            // Włącz 3x większą dynamikę analizatora dla SDPlayera
            eq_analyzer_set_sdplayer_mode(true);
            
            // Pokaż splash screen "SD PLAYER"
            _oled->showSplash();
            
            Serial.println("SDPlayerWebUI: Auto-activated OLED for SD Player during playback");
        }
        
        syncOLEDDisplay();  // Odśwież wyświetlacz
    }
}

void SDPlayerWebUI::playIndex(int index) {
    if (index < 0 || index >= _fileList.size()) return;
    
    FileItem& item = _fileList[index];
    if (item.isDir) {
        // Jeśli to katalog, wejdź do niego
        String newPath = _currentDir;
        if (newPath != "/") newPath += "/";
        newPath += item.name;
        Serial.printf("[SDPlayerWebUI] Entering directory: %s\n", newPath.c_str());
        changeDirectory(newPath);
    } else {
        // Jeśli to plik, odtwórz go
        _selectedIndex = index;
        String fullPath = _currentDir;
        if (fullPath != "/") fullPath += "/";
        fullPath += item.name;
        
        Serial.printf("[SDPlayerWebUI] Playing file: %s (index %d)\n", fullPath.c_str(), index);
        
        // SYNCHRONIZACJA: Powiadom o zmianie indeksu przed odtwarzaniem
        notifyIndexChange(index);
        
        playFile(fullPath);
    }
}

void SDPlayerWebUI::pause() {
    if (_audio) {
        if (!_isPlaying && !_isPaused) {
            return;
        }

        if (!_isPaused) {
            // PAUZA: natywna pauza Audio (bez restartu pliku)
            String activePath = _currentFile;
            if (activePath.length() > 0 && !activePath.startsWith("/")) {
                String basePath = _currentDir;
                if (!basePath.endsWith("/")) {
                    basePath += "/";
                }
                activePath = basePath + activePath;
            }
            activePath.replace("//", "/");

            _pausedFilePath = activePath;
            _pausedPositionSec = _audio->getAudioCurrentTime();
            _pausedFilePositionBytes = _audio->getAudioFilePosition();

            Serial.printf("SDPlayerWebUI: Pause request at %lus, filePos=%lu (%s)\n",
                          _pausedPositionSec,
                          (unsigned long)_pausedFilePositionBytes,
                          _pausedFilePath.c_str());

            bool nativePauseOk = _audio->pauseResume();
            if (nativePauseOk) {
                _isPaused = true;
                _isPlaying = true;

                extern bool sdPlayerPlayingMusic;
                sdPlayerPlayingMusic = false;
                Serial.println("SDPlayerWebUI: Native pause OK");
            } else {
                // Fallback: zatrzymanie i wznowienie przez reconnect
                Serial.println("SDPlayerWebUI: Native pause failed, fallback stop-song mode");
                _audio->stopSong();

                _isPaused = true;
                _isPlaying = true;

                extern bool sdPlayerPlayingMusic;
                sdPlayerPlayingMusic = false;
            }
        } else {
            // WZNOWIENIE: natywne audio pause/resume
            bool nativeResumeOk = _audio->pauseResume();
            if (nativeResumeOk) {
                _isPaused = false;
                _isPlaying = true;

                extern bool sdPlayerPlayingMusic;
                sdPlayerPlayingMusic = true;
                Serial.println("SDPlayerWebUI: Native resume OK");
            } else if (_pausedFilePath.length() == 0) {
                Serial.println("SDPlayerWebUI: Resume skipped - no paused file path");
                _isPaused = false;
                _isPlaying = false;
            } else {
                // Fallback: reconnect + wymuszenie pozycji
                Serial.printf("SDPlayerWebUI: Native resume failed, fallback reconnect from %lus (%s)\n",
                              _pausedPositionSec,
                              _pausedFilePath.c_str());
                _audio->stopSong();

                if (_audio->connecttoFS(STORAGE, _pausedFilePath.c_str(), -1)) {
                    _currentFile = _pausedFilePath;

                    bool resumePosApplied = false;
                    if (_pausedFilePositionBytes > 0) {
                        for (uint8_t retry = 0; retry < 25; retry++) {
                            if (_audio->setAudioFilePosition(_pausedFilePositionBytes)) {
                                resumePosApplied = true;
                                break;
                            }
                            delay(20);
                            yield();
                        }
                    }

                    if (!resumePosApplied && _pausedPositionSec > 0) {
                        uint16_t resumeSec = (_pausedPositionSec > 65535U) ? 65535U : (uint16_t)_pausedPositionSec;
                        for (uint8_t retry = 0; retry < 25; retry++) {
                            if (_audio->setAudioPlayTime(resumeSec)) {
                                resumePosApplied = true;
                                break;
                            }
                            delay(20);
                            yield();
                        }
                    }

                    _isPaused = false;
                    _isPlaying = true;

                    extern bool sdPlayerPlayingMusic;
                    sdPlayerPlayingMusic = true;
                } else {
                    Serial.println("SDPlayerWebUI: ERROR - Resume fallback reconnect failed");
                    _isPaused = false;
                    _isPlaying = false;
                }
            }
        }
        
        // Aktualizuj OLED
        if (_oled && _oled->isActive()) {
            _oled->loop();
        }
    } else {
        Serial.println("SDPlayerWebUI: ERROR - Audio pointer is NULL!");
    }
}

void SDPlayerWebUI::stop() {
    _isPlaying = false;
    _isPaused = false;
    _currentFile = "None";
    _pausedFilePath = "";
    _pausedPositionSec = 0;
    _pausedFilePositionBytes = 0;
    // Serial.println("SDPlayerWebUI: Stopped");
    
    if (_audio) {
        _audio->stopSong();
        // Resetuj flagę globalną
        extern bool sdPlayerPlayingMusic;
        sdPlayerPlayingMusic = false;
    } else {
        // Serial.println("SDPlayerWebUI: ERROR - Audio pointer is NULL!");
    }
    
    // SYNCHRONIZACJA: Powiadom o zmianach
    notifyPlayStateChange(false);
    notifyFileChange("None");
    
    // SYNCHRONIZACJA: Odśwież OLED
    syncOLEDDisplay();
}

void SDPlayerWebUI::next() {
    if (_selectedIndex < _fileList.size() - 1) {
        // Znajdź następny plik audio (pomiń katalogi)
        for (int i = _selectedIndex + 1; i < _fileList.size(); i++) {
            if (!_fileList[i].isDir) {
                playIndex(i);
                break;
            }
        }
    }
}

void SDPlayerWebUI::prev() {
    if (_selectedIndex > 0) {
        // Znajdź poprzedni plik audio (pomiń katalogi)
        for (int i = _selectedIndex - 1; i >= 0; i--) {
            if (!_fileList[i].isDir) {
                playIndex(i);
                break;
            }
        }
    }
}

void SDPlayerWebUI::seekRelative(int deltaSeconds) {
    Serial.printf("[SDPlayerWebUI::seekRelative] Wywołane: delta=%ds _audio=%p _isPlaying=%d _isPaused=%d\n", 
                  deltaSeconds, _audio, _isPlaying, _isPaused);
    
    if (!_audio || deltaSeconds == 0) {
        Serial.println("[SDPlayerWebUI::seekRelative] BŁĄD: Brak audio lub deltaSeconds=0");
        return;
    }

    if (!_isPlaying && !_isPaused) {
        Serial.println("[SDPlayerWebUI::seekRelative] BŁĄD: Nie odtwarzam ani nie pauza");
        return;
    }

    String activePath = _currentFile;
    if (activePath.length() == 0 || activePath == "None") {
        return;
    }

    if (!activePath.startsWith("/")) {
        String basePath = _currentDir;
        if (!basePath.endsWith("/")) {
            basePath += "/";
        }
        activePath = basePath + activePath;
    }
    activePath.replace("//", "/");

    uint32_t currentPosSec = _isPaused ? _pausedPositionSec : _audio->getAudioCurrentTime();
    uint32_t durationSec = _audio->getAudioFileDuration();

    int32_t targetSec = (int32_t)currentPosSec + deltaSeconds;
    if (targetSec < 0) {
        targetSec = 0;
    }

    if (durationSec > 0) {
        int32_t maxAllowed = (durationSec > 1) ? (int32_t)durationSec - 1 : 0;
        if (targetSec > maxAllowed) {
            targetSec = maxAllowed;
        }
    }

    if (_isPaused) {
        _pausedPositionSec = (uint32_t)targetSec;
        Serial.printf("SDPlayerWebUI: Seek paused position -> %lus\n", _pausedPositionSec);
        return;
    }

    bool seekOk = _audio->setTimeOffset(deltaSeconds);

    if (!seekOk) {
        if (targetSec <= 65535) {
            seekOk = _audio->setAudioPlayTime((uint16_t)targetSec);
        } else {
            seekOk = _audio->setAudioPlayTime(65535);
        }
    }

    if (seekOk) {
        _currentFile = activePath;
        _isPlaying = true;
        _isPaused = false;

        extern bool sdPlayerPlayingMusic;
        sdPlayerPlayingMusic = true;

        Serial.printf("SDPlayerWebUI: Seek %s %d sec -> %ld sec\n",
                      deltaSeconds > 0 ? "forward" : "backward",
                      abs(deltaSeconds),
                      (long)targetSec);

        if (_oled && _oled->isActive()) {
            _oled->loop();
        }
    } else {
        Serial.println("SDPlayerWebUI: ERROR - seek failed (setTimeOffset/setAudioPlayTime)");
    }
}

void SDPlayerWebUI::seekAbsolute(int targetSeconds) {
    Serial.printf("[SDPlayerWebUI::seekAbsolute] Wywołane: target=%ds _audio=%p _isPlaying=%d _isPaused=%d\n", 
                  targetSeconds, _audio, _isPlaying, _isPaused);
    
    if (!_audio || targetSeconds < 0) {
        Serial.println("[SDPlayerWebUI::seekAbsolute] BŁĄD: Brak audio lub targetSeconds<0");
        return;
    }

    if (!_isPlaying && !_isPaused) {
        Serial.println("[SDPlayerWebUI::seekAbsolute] BŁĄD: Nie odtwarzam ani nie pauza");
        return;
    }

    String activePath = _currentFile;
    if (activePath.length() == 0 || activePath == "None") {
        return;
    }

    if (!activePath.startsWith("/")) {
        String basePath = _currentDir;
        if (!basePath.endsWith("/")) {
            basePath += "/";
        }
        activePath = basePath + activePath;
    }
    activePath.replace("//", "/");

    uint32_t durationSec = _audio->getAudioFileDuration();
    uint32_t finalTargetSec = (uint32_t)targetSeconds;

    // Limit to song duration
    if (durationSec > 0) {
        uint32_t maxAllowed = (durationSec > 1) ? durationSec - 1 : 0;
        if (finalTargetSec > maxAllowed) {
            finalTargetSec = maxAllowed;
        }
    }

    if (_isPaused) {
        _pausedPositionSec = finalTargetSec;
        Serial.printf("SDPlayerWebUI: Seek paused absolute position -> %lus\n", _pausedPositionSec);
        return;
    }

    bool seekOk = false;
    if (finalTargetSec <= 65535) {
        seekOk = _audio->setAudioPlayTime((uint16_t)finalTargetSec);
    } else {
        seekOk = _audio->setAudioPlayTime(65535);
    }

    if (seekOk) {
        _currentFile = activePath;
        _isPlaying = true;
        _isPaused = false;

        extern bool sdPlayerPlayingMusic;
        sdPlayerPlayingMusic = true;

        Serial.printf("SDPlayerWebUI: Absolute seek to %ld sec\n", (long)finalTargetSec);

        if (_oled && _oled->isActive()) {
            _oled->loop();
        }
    } else {
        Serial.println("SDPlayerWebUI: ERROR - absolute seek failed (setAudioPlayTime)");
    }
}

void SDPlayerWebUI::playNextAuto() {
    // Automatyczne odtwarzanie następnego utworu po zakończeniu obecnego
    
    // KRYTYCZNE: Sprawdź czy lista utworów jest wczytana
    if (_fileList.size() == 0) {
        Serial.println("[SDPlayer] Auto-play BŁĄD: Lista utworów pusta - zatrzymuję odtwarzanie");
        Serial.println("[SDPlayer] Wskazówka: Otwórz stronę WWW aby wczytać listę plików");
        // NIE SKANUJ TUTAJ - to blokuje główną pętlę loop()!
        // Użytkownik musi wejść na stronę WWW aby wczytać listę
        extern bool sdPlayerPlayingMusic;
        sdPlayerPlayingMusic = false;
        _isPlaying = false;
        return;
    }
    
    Serial.printf("[SDPlayer] Auto-play: Lista ma %d plików, aktualny indeks: %d\n", _fileList.size(), _selectedIndex);
    
    if (_selectedIndex < _fileList.size() - 1) {
        // Znajdź następny plik audio (pomiń katalogi)
        bool foundNext = false;
        for (int i = _selectedIndex + 1; i < _fileList.size(); i++) {
            if (!_fileList[i].isDir) {
                playIndex(i);
                foundNext = true;
                Serial.println("[SDPlayer] Auto-play: Następny utwór #" + String(i));
                break;
            }
        }
        
        // Jeśli nie znaleziono następnego, wróć na początek listy
        if (!foundNext) {
            // Znajdź pierwszy plik audio od początku
            for (int i = 0; i < _fileList.size(); i++) {
                if (!_fileList[i].isDir) {
                    playIndex(i);
                    Serial.println("[SDPlayer] Auto-play: Koniec listy - powrót na początek, utwór #" + String(i));
                    break;
                }
            }
        }
    } else {
        // Koniec listy - wróć na początek
        for (int i = 0; i < _fileList.size(); i++) {
            if (!_fileList[i].isDir) {
                playIndex(i);
                Serial.println("[SDPlayer] Auto-play: Koniec listy - powrót na początek, utwór #" + String(i));
                break;
            }
        }
    }
}

void SDPlayerWebUI::setVolume(int vol) {
    if (vol < 0) vol = 0;
    if (vol > 21) vol = 21;
    _volume = vol;
    // Serial.println("SDPlayerWebUI: Setting volume to " + String(vol));
    
    // Ustaw globalną głośność Audio
    if (_audio) {
        _audio->setVolume(vol);
        // Serial.println("SDPlayerWebUI: Audio volume set");
    } else {
        // Serial.println("SDPlayerWebUI: WARNING - Audio pointer is NULL!");
    }
}

void SDPlayerWebUI::changeDirectory(const String& path) {
    File dir = STORAGE.open(path);
    if (!dir || !dir.isDirectory()) {
        // Serial.println("Failed to open directory: " + path);
        if (dir) dir.close();
        return;
    }
    dir.close();
    
    _currentDir = path;
    // Usuń podwójne slashe
    _currentDir.replace("//", "/");
    
    // Serial.println("Changed directory to: " + _currentDir);
    scanCurrentDirectory();
}

void SDPlayerWebUI::upDirectory() {
    if (_currentDir == "/") return;
    
    int lastSlash = _currentDir.lastIndexOf('/');
    if (lastSlash == 0) {
        _currentDir = "/";
    } else if (lastSlash > 0) {
        _currentDir = _currentDir.substring(0, lastSlash);
    }
    
    // Serial.println("Up to directory: " + _currentDir);
    scanCurrentDirectory();
}

void SDPlayerWebUI::scanCurrentDirectory() {
    _fileList.clear();
    
    File dir = STORAGE.open(_currentDir);
    if (!dir || !dir.isDirectory()) {
        // Serial.println("Failed to scan directory: " + _currentDir);
        if (dir) dir.close();
        return;
    }
    
    int fileCount = 0;
    File entry = dir.openNextFile();
    while (entry) {
        FileItem item;
        item.name = String(entry.name());
        item.isDir = entry.isDirectory();
        
        // Usuń ścieżkę z nazwy - zostaw tylko nazwę pliku/katalogu
        int lastSlash = item.name.lastIndexOf('/');
        if (lastSlash >= 0) {
            item.name = item.name.substring(lastSlash + 1);
        }
        
        // Dodaj tylko jeśli to katalog lub plik audio
        if (item.isDir || isAudioFile(item.name)) {
            _fileList.push_back(item);
        }
        
        entry.close();
        
        // KRYTYCZNE: Oddaj kontrolę co 10 plików aby uniknąć watchdog timeout
        fileCount++;
        if (fileCount % 10 == 0) {
            yield();  // Pozwól innym taskom działać
        }
        
        entry = dir.openNextFile();
    }
    dir.close();
    
    sortFileList();
    // Serial.println("Scanned " + String(_fileList.size()) + " items in " + _currentDir);
}

void SDPlayerWebUI::buildFileList(JsonArray& items) {
    int trackNumber = 0;  // Licznik utworów (pomijając foldery)
    
    for (const auto& item : _fileList) {
        JsonObject obj = items.createNestedObject();
        obj["n"] = item.name;
        obj["d"] = item.isDir;
        
        if (!item.isDir) {
            // Tylko dla plików muzycznych: dodaj numer utworu
            trackNumber++;
            obj["num"] = trackNumber;
            
            // Wyciągnij folder (album) ze ścieżki
            String path = _currentDir + "/" + item.name;
            int lastSlash = _currentDir.lastIndexOf('/');
            String album = "-";
            if (lastSlash >= 0 && lastSlash < _currentDir.length() - 1) {
                album = _currentDir.substring(lastSlash + 1);
            } else if (_currentDir != "/") {
                album = _currentDir;
            }
            obj["album"] = album;
        }
    }
}

bool SDPlayerWebUI::isAudioFile(const String& filename) {
    String lower = filename;
    lower.toLowerCase();
    return lower.endsWith(".mp3") || 
           lower.endsWith(".wav") || 
           lower.endsWith(".flac") ||
           lower.endsWith(".aac") ||
           lower.endsWith(".m4a") ||
           lower.endsWith(".ogg");
}

void SDPlayerWebUI::sortFileList() {
    // Sortuj: najpierw katalogi, potem pliki (alfabetycznie)
    std::sort(_fileList.begin(), _fileList.end(), [](const FileItem& a, const FileItem& b) {
        if (a.isDir != b.isDir) {
            return a.isDir; // Katalogi na początku
        }
        return a.name.compareTo(b.name) < 0;
    });
}

void SDPlayerWebUI::scanDirectory() {
    // THROTTLING: Zapobiega zbyt częstym skanowaniom
    unsigned long currentTime = millis();
    if (currentTime - _lastScanTime < _minScanInterval) {
        Serial.printf("[SDPlayer] Skanowanie zablokowane (throttling) - ostatnie: %dms temu\n", 
                     currentTime - _lastScanTime);
        return;
    }
    
    _lastScanTime = currentTime;
    
    // SPRAWDŹ CZY KATALOG ISTNIEJE
    File testDir = STORAGE.open(_currentDir);
    if (!testDir || !testDir.isDirectory()) {
        Serial.println("[SDPlayer] BŁĄD: Katalog " + _currentDir + " nie istnieje lub nie jest katalogiem!");
        if (testDir) testDir.close();
        
        // Przejdź do katalogu głównego jeśli obecny nie istnieje
        if (_currentDir != "/") {
            _currentDir = "/";
            Serial.println("[SDPlayer] Przełączono na katalog główny /");
        }
        return;
    }
    testDir.close();
    
    // Publiczna metoda do skanowania katalogu (wrapper dla scanCurrentDirectory)
    // Wywoływana z loop() aby uniknąć watchdog timeout w HTTP handlerze
    scanCurrentDirectory();
    
    // KRYTYCZNE: Synchronizuj listę z OLED po skanowaniu
    syncFileListWithOLED();
}

// ==================== SYNCHRONIZACJA Z OLED ====================

void SDPlayerWebUI::syncOLEDDisplay() {
    // Odświeża OLED po zmianach
    if (_oled && _oled->isActive()) {
        _oled->loop();  // Odśwież wyświetlacz
        Serial.println("SDPlayerWebUI: OLED display synchronized");
    }
}

void SDPlayerWebUI::notifyIndexChange(int newIndex) {
    if (_indexChangeCallback && newIndex != _selectedIndex) {
        _selectedIndex = newIndex;
        _indexChangeCallback(newIndex);
        Serial.printf("SDPlayerWebUI: Notified index change to %d\\n", newIndex);
    }
}

void SDPlayerWebUI::notifyFileChange(const String& newFile) {
    if (_fileChangeCallback && newFile != _currentFile) {
        _currentFile = newFile;
        _fileChangeCallback(newFile);
        Serial.printf("SDPlayerWebUI: Notified file change to %s\\n", newFile.c_str());
    }
}

void SDPlayerWebUI::notifyPlayStateChange(bool playing) {
    if (_playStateCallback && playing != _isPlaying) {
        _isPlaying = playing;
        _playStateCallback(playing);
        Serial.printf("SDPlayerWebUI: Notified play state change to %s\\n", playing ? "PLAYING" : "STOPPED");
    }
}

// ===== NOWE HANDLERY: Przełączanie stylów OLED =====
void SDPlayerWebUI::handleNextStyle(AsyncWebServerRequest *request) {
    Serial.println("SDPlayerWebUI: handleNextStyle called");
    
    if (_oled) {
        _oled->nextStyle();
        Serial.printf("SDPlayerWebUI: Switched to next style: %d\\n", _oled->getStyle());
        request->send(200, "application/json; charset=utf-8", "{\"style\":" + String(_oled->getStyle()) + "}");
    } else {
        request->send(500, "text/plain; charset=utf-8", "OLED not available");
    }
}

void SDPlayerWebUI::handleSetStyle(AsyncWebServerRequest *request) {
    Serial.println("========================================");
    Serial.println("SDPlayerWebUI: handleSetStyle called!");
    Serial.printf("Method: %s, URI: %s\n", request->methodToString(), request->url().c_str());
    
    if (!request->hasParam("style")) {
        Serial.println("ERROR: Missing 'style' parameter");
        request->send(400, "text/plain; charset=utf-8", "Missing style parameter");
        return;
    }
    
    String styleParam = request->getParam("style")->value();
    int style = styleParam.toInt();
    Serial.printf("SDPlayerWebUI: Style parameter received: '%s' -> %d\n", styleParam.c_str(), style);
    
    if (!_oled) {
        Serial.println("ERROR: _oled is NULL!");
        request->send(500, "text/plain; charset=utf-8", "OLED not available - WebUI not linked");
        return;
    }
    
    if (style >= 1 && style <= 14) {
        Serial.printf("SDPlayerWebUI: Setting OLED style to %d\n", style);
        _oled->setStyle((SDPlayerOLED::DisplayStyle)style);
        Serial.printf("SDPlayerWebUI: Style set successfully to %d\n", style);
        request->send(200, "application/json; charset=utf-8", "{\"style\":" + String(style) + "}");
    } else {
        Serial.printf("ERROR: Invalid style number: %d (must be 1-14)\n", style);
        request->send(400, "text/plain; charset=utf-8", "Invalid style number (1-14)");
    }
    Serial.println("========================================");
}

void SDPlayerWebUI::handleGetStyle(AsyncWebServerRequest *request) {
    if (_oled) {
        int currentStyle = _oled->getStyle();
        Serial.printf("SDPlayerWebUI: Current style is %d\n", currentStyle);
        request->send(200, "application/json; charset=utf-8", "{\"style\":" + String(currentStyle) + "}");
    } else {
        request->send(500, "text/plain; charset=utf-8", "OLED not available");
    }
}

// ===== USUWANIE PLIKÓW (TYLKO RECORDINGS) =====
void SDPlayerWebUI::handleDelete(AsyncWebServerRequest *request) {
    if (!request->hasParam("f")) {
        request->send(400, "application/json; charset=utf-8", "{\"success\":false,\"error\":\"Missing filename\"}");
        return;
    }
    
    String filename = request->getParam("f")->value();
    
    // Zabezpieczenie: Usuwanie tylko w folderze RECORDINGS
    if (!_currentDir.startsWith("/RECORDINGS")) {
        Serial.printf("[SDPlayer DELETE] Zabronione usuwanie poza /RECORDINGS: %s\n", _currentDir.c_str());
        request->send(403, "application/json; charset=utf-8", "{\"success\":false,\"error\":\"Deleting allowed only in RECORDINGS folder\"}");
        return;
    }
    
    // Buduj pełną ścieżkę
    String fullPath = _currentDir;
    if (!fullPath.endsWith("/")) {
        fullPath += "/";
    }
    fullPath += filename;
    
    Serial.printf("[SDPlayer DELETE] Próba usunięcia: %s\n", fullPath.c_str());
    
    // Sprawdź czy plik istnieje
    if (!STORAGE.exists(fullPath)) {
        Serial.printf("[SDPlayer DELETE] Plik nie istnieje: %s\n", fullPath.c_str());
        request->send(404, "application/json; charset=utf-8", "{\"success\":false,\"error\":\"File not found\"}");
        return;
    }
    
    // Jeśli aktualnie odtwarzany plik - zatrzymaj
    if (_currentFile == fullPath && _isPlaying) {
        Serial.println("[SDPlayer DELETE] Zatrzymuję odtwarzanie przed usunięciem");
        stop();
    }
    
    // Usuń plik
    if (STORAGE.remove(fullPath)) {
        Serial.printf("[SDPlayer DELETE] SUCCESS: Usunięto %s\n", fullPath.c_str());
        
        // Odśwież listę plików
        scanCurrentDirectory();
        
        request->send(200, "application/json; charset=utf-8", "{\"success\":true}");
    } else {
        Serial.printf("[SDPlayer DELETE] ERROR: Nie udało się usunąć %s\n", fullPath.c_str());
        request->send(500, "application/json; charset=utf-8", "{\"success\":false,\"error\":\"Failed to delete file\"}");
    }
}

// ===== NOWE HANDLERY: Symulacja przycisków pilota UP/DOWN =====
// ==================== SYNCHRONIZACJA LIST PLIKÓW ====================

void SDPlayerWebUI::syncFileListWithOLED() {
    if (!_oled) {
        return;
    }
    
    Serial.printf("[SDPlayer] Synchronizuję listę plików z OLED: %d elementów\n", _fileList.size());
    
    // Konwertuj FileItem na std::pair<String, bool> dla OLED
    std::vector<std::pair<String, bool>> convertedList;
    convertedList.reserve(_fileList.size());
    
    for (const auto& item : _fileList) {
        convertedList.emplace_back(item.name, item.isDir);
    }
    
    // Zsynchronizuj listę plików z OLED
    _oled->syncFileListFromWebUI(convertedList, _currentDir);
    
    Serial.println("[SDPlayer] Lista plików zsynchronizowana z OLED - koniec lazy loading!");
}
