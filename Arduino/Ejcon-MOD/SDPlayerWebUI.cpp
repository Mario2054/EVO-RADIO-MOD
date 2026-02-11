#include "SDPlayerWebUI.h"
#include "Audio.h"
#include "SDPlayerOLED.h"

SDPlayerWebUI::SDPlayerWebUI() 
    : _server(nullptr),
      _audio(nullptr),
      _oled(nullptr),
      _exitCallback(nullptr),
      _currentDir("/"),
      _currentFile("None"),
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
    
    _server->on("/sdplayer/api/cd", HTTP_GET, [this](AsyncWebServerRequest *request){
        this->handleCd(request);
    });
    
    _server->on("/sdplayer/api/up", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleUp(request);
    });
    
    _server->on("/sdplayer/api/back", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleBack(request);
    });
    
    // Główna strona SD Player - NA KOŃCU!
    _server->on("/sdplayer", HTTP_GET, [this](AsyncWebServerRequest *request){
        // Serial.println("SDPlayerWebUI: /sdplayer requested");
        this->handleRoot(request);
    });
    
    // Inicjalizacja
    if (!SD.begin()) {
        // Serial.println("SD Card initialization failed!");
    } else {
        // Serial.println("SD Card initialized.");
        scanCurrentDirectory();
    }
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
    
    // Aktywuj OLED przy wejściu na stronę SD Player
    if (_oled && !_oled->isActive()) {
        _oled->activate();
        _oled->showSplash();
        // Serial.println("SDPlayerWebUI: OLED activated with splash");
    }
    
    request->send_P(200, "text/html", SDPLAYER_HTML);
    // Serial.println("SDPlayerWebUI: HTML sent");
}

void SDPlayerWebUI::handleList(AsyncWebServerRequest *request) {
    // Serial.println("========================================");
    // Serial.println("SDPlayerWebUI: handleList called");
    // Serial.printf("SDPlayerWebUI: Request URL: %s\n", request->url().c_str());
    // Serial.println("========================================");
    scanCurrentDirectory();
    
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
    
    JsonArray items = doc.createNestedArray("items");
    buildFileList(items);
    
    String response;
    serializeJson(doc, response);
    // Serial.println("SDPlayerWebUI: Sending JSON: " + response);
    request->send(200, "application/json", response);
}

void SDPlayerWebUI::handlePlay(AsyncWebServerRequest *request) {
    // Serial.println("SDPlayerWebUI: handlePlay called");
    if (request->hasParam("i")) {
        int idx = request->getParam("i")->value().toInt();
        // Serial.printf("Playing index: %d\n", idx);
        playIndex(idx);
    }
    request->send(200, "text/plain", "OK");
}

void SDPlayerWebUI::handlePlaySelected(AsyncWebServerRequest *request) {
    // Serial.printf("SDPlayerWebUI: handlePlaySelected called, index=%d, listSize=%d\n", _selectedIndex, _fileList.size());
    if (_selectedIndex >= 0 && _selectedIndex < _fileList.size()) {
        playIndex(_selectedIndex);
    } else {
        // Serial.println("SDPlayerWebUI: No file selected or invalid index");
    }
    request->send(200, "text/plain", "OK");
}

void SDPlayerWebUI::handlePause(AsyncWebServerRequest *request) {
    // Serial.println("SDPlayerWebUI: handlePause called");
    pause();
    request->send(200, "text/plain", "OK");
}

void SDPlayerWebUI::handleStop(AsyncWebServerRequest *request) {
    // Serial.println("SDPlayerWebUI: handleStop called");
    stop();
    request->send(200, "text/plain", "OK");
}

void SDPlayerWebUI::handleNext(AsyncWebServerRequest *request) {
    // Serial.println("SDPlayerWebUI: handleNext called");
    next();
    request->send(200, "text/plain", "OK");
}

void SDPlayerWebUI::handlePrev(AsyncWebServerRequest *request) {
    // Serial.println("SDPlayerWebUI: handlePrev called");
    prev();
    request->send(200, "text/plain", "OK");
}

void SDPlayerWebUI::handleVol(AsyncWebServerRequest *request) {
    if (request->hasParam("v")) {
        int vol = request->getParam("v")->value().toInt();
        setVolume(vol);
    }
    request->send(200, "text/plain", "OK");
}

void SDPlayerWebUI::handleCd(AsyncWebServerRequest *request) {
    // Serial.println("SDPlayerWebUI: handleCd called");
    if (request->hasParam("p")) {
        String path = request->getParam("p")->value();
        // Serial.println("Changing directory to: " + path);
        changeDirectory(path);
    }
    request->send(200, "text/plain", "OK");
}

void SDPlayerWebUI::handleUp(AsyncWebServerRequest *request) {
    // Serial.println("SDPlayerWebUI: handleUp called");
    upDirectory();
    request->send(200, "text/plain", "OK");
}

void SDPlayerWebUI::handleBack(AsyncWebServerRequest *request) {
    // Serial.println("SDPlayerWebUI: handleBack called - stopping playback");
    
    // Zatrzymaj odtwarzanie przed wyjściem
    stop();
    
    // Deaktywuj OLED display
    if (_oled && _oled->isActive()) {
        _oled->deactivate();
        // Serial.println("SDPlayerWebUI: OLED deactivated");
    }
    
    if (_exitCallback) {
        _exitCallback();
    }
    request->send(200, "text/plain", "OK");
}

void SDPlayerWebUI::playFile(const String& path) {
    _currentFile = path;
    _isPlaying = true;
    _isPaused = false;
    // Serial.println("SDPlayerWebUI: Playing file: " + path);
    
    if (_audio) {
        _audio->stopSong();  // Zatrzymaj obecną muzykę
        if (_audio->connecttoFS(SD, path.c_str())) {
            // Serial.println("SDPlayerWebUI: Audio started playing from SD");
            // Ustaw flagę globalną
            extern bool sdPlayerPlayingMusic;
            sdPlayerPlayingMusic = true;
        } else {
            // Serial.println("SDPlayerWebUI: ERROR - Failed to play file!");
            _isPlaying = false;
        }
    } else {
        // Serial.println("SDPlayerWebUI: ERROR - Audio pointer is NULL!");
    }
    
    // Aktualizuj OLED
    if (_oled && _oled->isActive()) {
        _oled->loop();  // Odśwież wyświetlacz
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
        changeDirectory(newPath);
    } else {
        // Jeśli to plik, odtwórz go
        _selectedIndex = index;
        String fullPath = _currentDir;
        if (fullPath != "/") fullPath += "/";
        fullPath += item.name;
        playFile(fullPath);
    }
}

void SDPlayerWebUI::pause() {
    if (_audio) {
        _isPaused = !_isPaused;
        
        if (_isPaused) {
            // PAUZA = STOP (bezpieczniejsze niż pauseResume() które crashuje FreeRTOS)
            Serial.println("SDPlayerWebUI: Paused (STOP)");
            _audio->stopSong();
            extern bool sdPlayerPlayingMusic;
            sdPlayerPlayingMusic = false;
        } else {
            // WZNOWIENIE = Odtwórz ten sam plik od nowa
            Serial.println("SDPlayerWebUI: Resumed (RESTART)");
            if (_selectedIndex >= 0 && _selectedIndex < _fileList.size()) {
                FileItem& item = _fileList[_selectedIndex];
                String fullPath = _currentDir;
                if (fullPath != "/") fullPath += "/";
                fullPath += item.name;
                
                _audio->stopSong();
                if (_audio->connecttoFS(SD, fullPath.c_str())) {
                    extern bool sdPlayerPlayingMusic;
                    sdPlayerPlayingMusic = true;
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
    // Serial.println("SDPlayerWebUI: Stopped");
    
    if (_audio) {
        _audio->stopSong();
        // Resetuj flagę globalną
        extern bool sdPlayerPlayingMusic;
        sdPlayerPlayingMusic = false;
    } else {
        // Serial.println("SDPlayerWebUI: ERROR - Audio pointer is NULL!");
    }
    
    // Aktualizuj OLED
    if (_oled && _oled->isActive()) {
        _oled->loop();
    }
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

void SDPlayerWebUI::playNextAuto() {
    // Automatyczne odtwarzanie następnego utworu po zakończeniu obecnego
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
    File dir = SD.open(path);
    if (!dir || !dir.isDirectory()) {
        // Serial.println("Failed to open directory: " + path);
        dir.close();
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
    
    File dir = SD.open(_currentDir);
    if (!dir || !dir.isDirectory()) {
        // Serial.println("Failed to scan directory: " + _currentDir);
        if (dir) dir.close();
        return;
    }
    
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
        entry = dir.openNextFile();
    }
    dir.close();
    
    sortFileList();
    // Serial.println("Scanned " + String(_fileList.size()) + " items in " + _currentDir);
}

void SDPlayerWebUI::buildFileList(JsonArray& items) {
    for (const auto& item : _fileList) {
        JsonObject obj = items.createNestedObject();
        obj["n"] = item.name;
        obj["d"] = item.isDir;
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
