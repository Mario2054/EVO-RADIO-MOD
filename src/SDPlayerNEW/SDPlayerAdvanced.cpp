#include "SDPlayerAdvanced.h"
#include "SDPlayerOLED.h"
#include "Audio.h"
#include <U8g2lib.h>
#include <algorithm>

// Extern globals
extern SPIClass customSPI;

// ==================== IKONY (8x8 pixeli) ====================
// Ikona PLAY (trójkąt)
const unsigned char icon_play_bits[] PROGMEM = {
    0x00, 0x02, 0x06, 0x0E, 0x1E, 0x0E, 0x06, 0x02
};

// Ikona PAUSE (dwie kreski)
const unsigned char icon_pause_bits[] PROGMEM = {
    0x00, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x00
};

// Ikona STOP (kwadrat)
const unsigned char icon_stop_bits[] PROGMEM = {
    0x00, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x7E, 0x00
};

// Ikona REPEAT (strzałki w koło)
const unsigned char icon_repeat_bits[] PROGMEM = {
    0x3C, 0x42, 0x81, 0x99, 0x99, 0x81, 0x42, 0x3C
};

// Ikona FOLDER
const unsigned char icon_folder_bits[] PROGMEM = {
    0x0E, 0x11, 0xFE, 0x82, 0x82, 0x82, 0xFE, 0x00
};

// Ikona MUSIC NOTE
const unsigned char icon_music_bits[] PROGMEM = {
    0x18, 0x18, 0x18, 0x18, 0x1B, 0x3F, 0x3E, 0x1C
};

// Constructor
SDPlayerAdvanced::SDPlayerAdvanced() 
    : _audio(nullptr)
    , _display(nullptr)
    , _oled(nullptr)
    , _active(false)
    , _isPlaying(false)
    , _isPaused(false)
    , _currentDir("/")
    , _selectedIndex(0)
    , _playStartTime(0)
    , _playSeconds(0)
    , _sortMode(SORT_NAME_NUMERIC)
    , _repeatMode(REPEAT_ALL)
    , _autoPlay(true)
    , _trackEndDetected(false)
    , _bitrateReceived(false)
{
}

SDPlayerAdvanced::~SDPlayerAdvanced() {
    deactivate();
}

// ==================== INICJALIZACJA ====================

void SDPlayerAdvanced::begin(Audio* audio, U8G2* display) {
    _audio = audio;
    _display = display;
    
    Serial.println("SDPlayerAdvanced: Initialized");
}

void SDPlayerAdvanced::setOLED(SDPlayerOLED* oled) {
    _oled = oled;
}

// ==================== AKTYWACJA ====================

void SDPlayerAdvanced::activate() {
    _active = true;
    _isPlaying = false;
    _isPaused = false;
    clearTrackInfo();
    
    // Wczytaj ostatnie ustawienia
    loadSettings();
    
    // Skanuj katalog
    scanCurrentDirectory();
    
    Serial.println("SDPlayerAdvanced: Activated");
}

void SDPlayerAdvanced::deactivate() {
    if (_isPlaying && _audio) {
        _audio->stopSong();
    }
    
    _active = false;
    _isPlaying = false;
    _isPaused = false;
    
    // Zapisz ustawienia
    saveSettings();
    
    Serial.println("SDPlayerAdvanced: Deactivated");
}

// ==================== MAIN LOOP ====================

void SDPlayerAdvanced::loop() {
    if (!_active || !_audio) return;
    
    // Aktualizuj timer odtwarzania
    if (_isPlaying && !_isPaused) {
        updatePlayTimer();
    }
    
    // Obsługa końca utworu
    if (_trackEndDetected) {
        _trackEndDetected = false;
        handleTrackEnd();
    }
    
    // Renderuj dedykowany styl OLED dla Advanced mode
    static unsigned long lastRender = 0;
    if (millis() - lastRender > 100) {  // Odświeżanie co 100ms
        lastRender = millis();
        renderOLED();
    }
}

// ==================== NAWIGACJA PO KATALOGACH ====================

bool SDPlayerAdvanced::changeDirectory(const String& path) {
    _currentDir = path;
    _selectedIndex = 0;
    scanCurrentDirectory();
    
    Serial.print("SDPlayerAdvanced: Changed dir to: ");
    Serial.println(path);
    
    return true;
}

bool SDPlayerAdvanced::enterFolder(int index) {
    if (index < 0 || index >= _fileList.size()) return false;
    
    FileItem& item = _fileList[index];
    if (!item.isDir) return false;
    
    String newPath = _currentDir;
    if (newPath != "/") newPath += "/";
    newPath += item.name;
    
    return changeDirectory(newPath);
}

bool SDPlayerAdvanced::goBack() {
    if (_currentDir == "/") return false;
    
    int lastSlash = _currentDir.lastIndexOf('/');
    if (lastSlash == 0) {
        return changeDirectory("/");
    } else {
        String parentDir = _currentDir.substring(0, lastSlash);
        return changeDirectory(parentDir);
    }
}

// ==================== SKANOWANIE KATALOGÓW ====================

void SDPlayerAdvanced::scanCurrentDirectory() {
    _fileList.clear();
    
    File root = SD.open(_currentDir);
    if (!root) {
        Serial.println("SDPlayerAdvanced: Failed to open directory");
        return;
    }
    
    if (!root.isDirectory()) {
        Serial.println("SDPlayerAdvanced: Not a directory");
        root.close();
        return;
    }
    
    File file = root.openNextFile();
    while (file) {
        String name = file.name();
        
        // Pomijamy ukryte pliki i System Volume Information
        if (!name.startsWith(".") && name != "System Volume Information") {
            FileItem item;
            item.name = name;
            item.isDir = file.isDirectory();
            item.size = file.size();
            
            // Pełna ścieżka
            item.fullPath = _currentDir;
            if (item.fullPath != "/") item.fullPath += "/";
            item.fullPath += name;
            
            // Filtruj - dodaj tylko foldery lub pliki audio
            if (item.isDir || isAudioFile(name)) {
                _fileList.push_back(item);
            }
        }
        
        file.close();
        file = root.openNextFile();
    }
    
    root.close();
    
    // Sortuj listę
    sortFileList();
    
    Serial.print("SDPlayerAdvanced: Scanned ");
    Serial.print(_fileList.size());
    Serial.println(" items");
}

// ==================== SORTOWANIE ====================

void SDPlayerAdvanced::setSortMode(SortMode mode) {
    _sortMode = mode;
    sortFileList();
}

void SDPlayerAdvanced::sortFileList() {
    switch (_sortMode) {
        case SORT_NAME_ASC:
            std::sort(_fileList.begin(), _fileList.end(), [](const FileItem& a, const FileItem& b) {
                if (a.isDir != b.isDir) return a.isDir;
                return a.name.compareTo(b.name) < 0;
            });
            break;
            
        case SORT_NAME_DESC:
            std::sort(_fileList.begin(), _fileList.end(), [](const FileItem& a, const FileItem& b) {
                if (a.isDir != b.isDir) return a.isDir;
                return a.name.compareTo(b.name) > 0;
            });
            break;
            
        case SORT_NAME_NUMERIC:
            std::sort(_fileList.begin(), _fileList.end(), [](const FileItem& a, const FileItem& b) {
                if (a.isDir != b.isDir) return a.isDir;
                return compareStringsWithNumbers(a.name, b.name) < 0;
            });
            break;
            
        case SORT_SIZE_ASC:
            std::sort(_fileList.begin(), _fileList.end(), [](const FileItem& a, const FileItem& b) {
                if (a.isDir != b.isDir) return a.isDir;
                return a.size < b.size;
            });
            break;
            
        case SORT_SIZE_DESC:
            std::sort(_fileList.begin(), _fileList.end(), [](const FileItem& a, const FileItem& b) {
                if (a.isDir != b.isDir) return a.isDir;
                return a.size > b.size;
            });
            break;
            
        default:
            // SORT_DATE_xxx wymagałoby dodatkowych metadanych z SD
            break;
    }
}

// Inteligentne sortowanie z uwzględnieniem liczb
int SDPlayerAdvanced::compareStringsWithNumbers(const String& a, const String& b) {
    int i = 0, j = 0;
    
    while (i < a.length() && j < b.length()) {
        char charA = a[i];
        char charB = b[j];
        
        // Jeśli oba znaki to cyfry, porównaj jako liczby
        if (isdigit(charA) && isdigit(charB)) {
            String numA, numB;
            while (i < a.length() && isdigit(a[i])) {
                numA += a[i++];
            }
            while (j < b.length() && isdigit(b[j])) {
                numB += b[j++];
            }
            
            int intA = numA.toInt();
            int intB = numB.toInt();
            
            if (intA != intB) {
                return intA - intB;
            }
        } else {
            // Porównaj inne znaki
            if (charA != charB) {
                return charA - charB;
            }
            i++;
            j++;
        }
    }
    
    // Jeżeli wszystko równe, porównaj długości
    return a.length() - b.length();
}

// ==================== KONTROLA ODTWARZANIA ====================

void SDPlayerAdvanced::play(int index) {
    if (index < 0 || index >= _fileList.size()) return;
    
    _selectedIndex = index;
    FileItem& item = _fileList[index];
    
    if (item.isDir) {
        // Wejdź do folderu
        enterFolder(index);
    } else {
        // Odtwórz plik
        playFile(item.fullPath);
    }
}

void SDPlayerAdvanced::playFile(const String& path) {
    if (!_audio) return;
    
    // Zatrzymaj poprzednie odtwarzanie
    if (_isPlaying) {
        _audio->stopSong();
    }
    
    // Wyczyść poprzednie informacje
    clearTrackInfo();
    
    _currentFile = path;
    _isPlaying = true;
    _isPaused = false;
    _playSeconds = 0;
    _playStartTime = millis();
    _trackEndDetected = false;
    _bitrateReceived = false;
    
    // Ustaw podstawowe info
    int lastSlash = path.lastIndexOf('/');
    if (lastSlash != -1) {
        _currentTrackInfo.filename = path.substring(lastSlash + 1);
        _currentTrackInfo.folder = path.substring(0, lastSlash);
    } else {
        _currentTrackInfo.filename = path;
        _currentTrackInfo.folder = "/";
    }
    
    // Określ codec po rozszerzeniu
    String ext = getFileExtension(path);
    ext.toUpperCase();
    _currentTrackInfo.codecType = ext;
    
    // Rozpocznij odtwarzanie
    if (_audio->connecttoFS(SD, path.c_str())) {
        Serial.print("SDPlayerAdvanced: Playing: ");
        Serial.println(path);
        
        // Ustaw globalną flagę
        extern bool sdPlayerPlayingMusic;
        sdPlayerPlayingMusic = true;
    } else {
        Serial.println("SDPlayerAdvanced: Failed to play file");
        _isPlaying = false;
    }
    
    // Odśwież OLED
    if (_oled && _oled->isActive()) {
        _oled->loop();
    }
}

void SDPlayerAdvanced::pause() {
    if (!_audio || !_isPlaying) return;
    
    _isPaused = true;
    _audio->stopSong();  // Na ESP32 pauseResume() crashuje FreeRTOS, więc używamy stop
    
    extern bool sdPlayerPlayingMusic;
    sdPlayerPlayingMusic = false;
    
    Serial.println("SDPlayerAdvanced: Paused");
}

void SDPlayerAdvanced::resume() {
    if (!_audio || !_isPlaying || !_isPaused) return;
    
    _isPaused = false;
    
    // Wznów od początku tego samego pliku (bezpieczniejsze niż pauseResume)
    if (_audio->connecttoFS(SD, _currentFile.c_str())) {
        extern bool sdPlayerPlayingMusic;
        sdPlayerPlayingMusic = true;
        Serial.println("SDPlayerAdvanced: Resumed");
    }
}

void SDPlayerAdvanced::stop() {
    if (!_audio) return;
    
    if (_isPlaying) {
        _audio->stopSong();
    }
    
    _isPlaying = false;
    _isPaused = false;
    _playSeconds = 0;
    clearTrackInfo();
    
    extern bool sdPlayerPlayingMusic;
    sdPlayerPlayingMusic = false;
    
    Serial.println("SDPlayerAdvanced: Stopped");
}

void SDPlayerAdvanced::togglePause() {
    if (_isPaused) {
        resume();
    } else {
        pause();
    }
}

void SDPlayerAdvanced::next() {
    if (_fileList.empty()) return;
    
    // Znajdź następny plik audio (pomiń foldery)
    int nextIndex = _selectedIndex + 1;
    while (nextIndex < _fileList.size()) {
        if (!_fileList[nextIndex].isDir) {
            play(nextIndex);
            return;
        }
        nextIndex++;
    }
    
    // Jeśli tryb powtarzania, wróć na początek
    if (_repeatMode == REPEAT_ALL || _repeatMode == REPEAT_FOLDER) {
        nextIndex = 0;
        while (nextIndex < _fileList.size()) {
            if (!_fileList[nextIndex].isDir) {
                play(nextIndex);
                return;
            }
            nextIndex++;
        }
    }
    
    // Nie znaleziono następnego - stop
    stop();
}

void SDPlayerAdvanced::previous() {
    if (_fileList.empty()) return;
    
    // Jeśli odtwarzamy dłużej niż 3 sekundy, restart tego samego utworu
    if (_playSeconds > 3) {
        playFile(_currentFile);
        return;
    }
    
    // Znajdź poprzedni plik audio (pomiń foldery)
    int prevIndex = _selectedIndex - 1;
    while (prevIndex >= 0) {
        if (!_fileList[prevIndex].isDir) {
            play(prevIndex);
            return;
        }
        prevIndex--;
    }
    
    // Jeśli tryb powtarzania, idź na koniec
    if (_repeatMode == REPEAT_ALL || _repeatMode == REPEAT_FOLDER) {
        prevIndex = _fileList.size() - 1;
        while (prevIndex >= 0) {
            if (!_fileList[prevIndex].isDir) {
                play(prevIndex);
                return;
            }
            prevIndex--;
        }
    }
}

// ==================== OBSŁUGA KOŃCA UTWORU ====================

void SDPlayerAdvanced::handleTrackEnd() {
    Serial.println("SDPlayerAdvanced: Track ended");
    
    switch (_repeatMode) {
        case REPEAT_ONE:
            // Powtórz ten sam utwór
            playFile(_currentFile);
            break;
            
        case REPEAT_ALL:
        case REPEAT_FOLDER:
            // Przejdź do kolejnego (auto-play)
            if (_autoPlay) {
                next();
            } else {
                stop();
            }
            break;
            
        case REPEAT_OFF:
        default:
            // Zakończ odtwarzanie
            stop();
            break;
    }
}

// ==================== TIMER ODTWARZANIA ====================

void SDPlayerAdvanced::updatePlayTimer() {
    unsigned long currentMillis = millis();
    if (currentMillis - _playStartTime >= 1000) {
        _playSeconds++;
        _playStartTime = currentMillis;
    }
}

String SDPlayerAdvanced::getPlayTimeString() {
    unsigned int minutes = _playSeconds / 60;
    unsigned int seconds = _playSeconds % 60;
    char timeStr[10];
    snprintf(timeStr, sizeof(timeStr), "%02u:%02u", minutes, seconds);
    return String(timeStr);
}

// ==================== CALLBACKS OD AUDIO ====================

void SDPlayerAdvanced::onAudioInfo(const char* info) {
    if (!info) return;
    parseAudioInfo(String(info));
}

void SDPlayerAdvanced::onAudioID3(const char* info) {
    if (!info) return;
    parseID3Data(String(info));
}

void SDPlayerAdvanced::onAudioBitrate(const char* info) {
    if (!info) return;
    
    String bitrateStr = String(info);
    _currentTrackInfo.bitrate = bitrateStr.toInt();
    _bitrateReceived = true;
    
    Serial.print("SDPlayerAdvanced: Bitrate: ");
    Serial.print(_currentTrackInfo.bitrate);
    Serial.println(" kbps");
}

void SDPlayerAdvanced::onAudioEOF(const char* info) {
    // Koniec pliku - ustawiamy flagę do obsłużenia w loop()
    _trackEndDetected = true;
    Serial.println("SDPlayerAdvanced: EOF detected");
}

// ==================== PARSOWANIE INFO/ID3 ====================

void SDPlayerAdvanced::parseAudioInfo(const String& info) {
    // BitRate
    int bitrateIndex = info.indexOf("BitRate:");
    if (bitrateIndex != -1) {
        String bitrate = info.substring(bitrateIndex + 8);
        bitrate.trim();
        _currentTrackInfo.bitrate = bitrate.toInt();
    }
    
    // SampleRate
    int sampleRateIndex = info.indexOf("SampleRate:");
    if (sampleRateIndex != -1) {
        String sampleRate = info.substring(sampleRateIndex + 11);
        sampleRate.trim();
        _currentTrackInfo.sampleRate = sampleRate.toInt();
    }
    
    // BitsPerSample
    int bitsIndex = info.indexOf("BitsPerSample:");
    if (bitsIndex != -1) {
        String bits = info.substring(bitsIndex + 14);
        bits.trim();
        _currentTrackInfo.bitsPerSample = bits.toInt();
    }
    
    // Decoder type
    if (info.indexOf("MP3Decoder") != -1) {
        _currentTrackInfo.codecType = "MP3";
    } else if (info.indexOf("FLACDecoder") != -1) {
        _currentTrackInfo.codecType = "FLAC";
    } else if (info.indexOf("AACDecoder") != -1) {
        _currentTrackInfo.codecType = "AAC";
    } else if (info.indexOf("VORBISDecoder") != -1) {
        _currentTrackInfo.codecType = "VORBIS";
    }
    
    // Skip metadata = brak ID3
    if (info.indexOf("skip metadata") != -1) {
        _currentTrackInfo.hasID3 = false;
    }
}

void SDPlayerAdvanced::parseID3Data(const String& info) {
    // Artist
    int artistIndex1 = info.indexOf("Artist: ");
    int artistIndex2 = info.indexOf("ARTIST=");
    int artistIndex3 = info.indexOf("Artist=");
    
    if (artistIndex1 != -1) {
        _currentTrackInfo.artist = info.substring(artistIndex1 + 8);
        _currentTrackInfo.artist.trim();
        _currentTrackInfo.hasID3 = true;
    } else if (artistIndex2 != -1) {
        _currentTrackInfo.artist = info.substring(artistIndex2 + 7);
        _currentTrackInfo.artist.trim();
        _currentTrackInfo.hasID3 = true;
    } else if (artistIndex3 != -1) {
        _currentTrackInfo.artist = info.substring(artistIndex3 + 7);
        _currentTrackInfo.artist.trim();
        _currentTrackInfo.hasID3 = true;
    }
    
    // Title
    int titleIndex1 = info.indexOf("Title: ");
    int titleIndex2 = info.indexOf("TITLE=");
    int titleIndex3 = info.indexOf("Title=");
    
    if (titleIndex1 != -1) {
        _currentTrackInfo.title = info.substring(titleIndex1 + 7);
        _currentTrackInfo.title.trim();
        _currentTrackInfo.hasID3 = true;
    } else if (titleIndex2 != -1) {
        _currentTrackInfo.title = info.substring(titleIndex2 + 6);
        _currentTrackInfo.title.trim();
        _currentTrackInfo.hasID3 = true;
    } else if (titleIndex3 != -1) {
        _currentTrackInfo.title = info.substring(titleIndex3 + 6);
        _currentTrackInfo.title.trim();
        _currentTrackInfo.hasID3 = true;
    }
    
    // Album
    int albumIndex = info.indexOf("Album: ");
    if (albumIndex != -1) {
        _currentTrackInfo.album = info.substring(albumIndex + 7);
        _currentTrackInfo.album.trim();
    }
    
    // Konwertuj polskie znaki
    if (_currentTrackInfo.hasID3) {
        processPolishText(_currentTrackInfo.artist);
        processPolishText(_currentTrackInfo.title);
        processPolishText(_currentTrackInfo.album);
    }
}

void SDPlayerAdvanced::clearTrackInfo() {
    _currentTrackInfo.artist = "";
    _currentTrackInfo.title = "";
    _currentTrackInfo.album = "";
    _currentTrackInfo.filename = "";
    _currentTrackInfo.folder = "";
    _currentTrackInfo.bitrate = 0;
    _currentTrackInfo.sampleRate = 0;
    _currentTrackInfo.bitsPerSample = 0;
    _currentTrackInfo.hasID3 = false;
    _currentTrackInfo.codecType = "";
}

// ==================== ZAPISYWANIE/WCZYTYWANIE USTAWIEŃ ====================

void SDPlayerAdvanced::saveSettings() {
    File file = SD.open("/sdplayer_adv.cfg", FILE_WRITE);
    if (file) {
        file.println(_currentDir);
        file.println(_selectedIndex);
        file.println((int)_sortMode);
        file.println((int)_repeatMode);
        file.println(_autoPlay ? 1 : 0);
        file.close();
        
        Serial.println("SDPlayerAdvanced: Settings saved");
    } else {
        Serial.println("SDPlayerAdvanced: Failed to save settings");
    }
}

void SDPlayerAdvanced::loadSettings() {
    if (!SD.exists("/sdplayer_adv.cfg")) {
        Serial.println("SDPlayerAdvanced: No settings file found");
        return;
    }
    
    File file = SD.open("/sdplayer_adv.cfg");
    if (file) {
        _currentDir = file.readStringUntil('\n');
        _currentDir.trim();
        _selectedIndex = file.parseInt();
        _sortMode = (SortMode)file.parseInt();
        _repeatMode = (RepeatMode)file.parseInt();
        _autoPlay = file.parseInt() == 1;
        file.close();
        
        Serial.println("SDPlayerAdvanced: Settings loaded");
        Serial.print("  Dir: ");
        Serial.println(_currentDir);
        Serial.print("  Index: ");
        Serial.println(_selectedIndex);
    } else {
        Serial.println("SDPlayerAdvanced: Failed to load settings");
    }
}

// ==================== FUNKCJE POMOCNICZE (STATIC) ====================

bool SDPlayerAdvanced::isAudioFile(const String& filename) {
    String lower = filename;
    lower.toLowerCase();
    
    return lower.endsWith(".mp3") || 
           lower.endsWith(".wav") || 
           lower.endsWith(".flac") ||
           lower.endsWith(".aac") ||
           lower.endsWith(".m4a") ||
           lower.endsWith(".ogg") ||
           lower.endsWith(".wma") ||
           lower.endsWith(".aiff") ||
           lower.endsWith(".alac");
}

String SDPlayerAdvanced::getFileExtension(const String& filename) {
    int dotPos = filename.lastIndexOf('.');
    if (dotPos == -1) return "";
    
    String ext = filename.substring(dotPos + 1);
    ext.toUpperCase();
    return ext;
}

String SDPlayerAdvanced::formatFileSize(long bytes) {
    if (bytes < 1024) {
        return String(bytes) + " B";
    } else if (bytes < 1024 * 1024) {
        return String(bytes / 1024.0, 1) + " KB";
    } else {
        return String(bytes / (1024.0 * 1024.0), 1) + " MB";
    }
}

void SDPlayerAdvanced::processPolishText(String& text) {
    for (int i = 0; i < text.length(); i++) {
        switch (text[i]) {
            case (char)0xC2:
                switch (text[i+1]) {
                    case (char)0xB3: text.setCharAt(i, 0xB3); break; // ł
                    case (char)0x9C: text.setCharAt(i, 0x9C); break; // ś
                    case (char)0x8C: text.setCharAt(i, 0x8C); break; // Ś
                    case (char)0xB9: text.setCharAt(i, 0xB9); break; // ą
                    case (char)0x9B: text.setCharAt(i, 0xEA); break; // ę
                    case (char)0xBF: text.setCharAt(i, 0xBF); break; // ż
                    case (char)0x9F: text.setCharAt(i, 0x9F); break; // ź
                }
                text.remove(i+1, 1);
                break;
                
            case (char)0xC3:
                switch (text[i+1]) {
                    case (char)0xB1: text.setCharAt(i, 0xF1); break; // ń
                    case (char)0xB3: text.setCharAt(i, 0xF3); break; // ó
                    case (char)0xBA: text.setCharAt(i, 0x9F); break; // ź
                    case (char)0xBB: text.setCharAt(i, 0xAF); break; // Ż
                    case (char)0x93: text.setCharAt(i, 0xD3); break; // Ó
                }
                text.remove(i+1, 1);
                break;
                
            case (char)0xC4:
                switch (text[i+1]) {
                    case (char)0x85: text.setCharAt(i, 0xB9); break; // ą
                    case (char)0x99: text.setCharAt(i, 0xEA); break; // ę
                    case (char)0x87: text.setCharAt(i, 0xE6); break; // ć
                    case (char)0x84: text.setCharAt(i, 0xA5); break; // Ą
                    case (char)0x98: text.setCharAt(i, 0xCA); break; // Ę
                    case (char)0x86: text.setCharAt(i, 0xC6); break; // Ć
                }
                text.remove(i+1, 1);
                break;
                
            case (char)0xC5:
                switch (text[i+1]) {
                    case (char)0x82: text.setCharAt(i, 0xB3); break; // ł
                    case (char)0x84: text.setCharAt(i, 0xF1); break; // ń
                    case (char)0x9B: text.setCharAt(i, 0x9C); break; // ś
                    case (char)0xBB: text.setCharAt(i, 0xAF); break; // Ż
                    case (char)0xBC: text.setCharAt(i, 0xBF); break; // ż
                    case (char)0x83: text.setCharAt(i, 0xD1); break; // Ń
                    case (char)0x9A: text.setCharAt(i, 0x97); break; // Ś
                    case (char)0x81: text.setCharAt(i, 0xA3); break; // Ł
                    case (char)0xB9: text.setCharAt(i, 0xAC); break; // Ź
                }
                text.remove(i+1, 1);
                break;
        }
    }
}

// ==================== RENDERING OLED (DEDYKOWANY STYL ADVANCED) ====================

void SDPlayerAdvanced::renderOLED() {
    if (!_display) {
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 5000) {
            Serial.println("[SDPlayerAdvanced] ERROR: _display is NULL!");
            lastDebug = millis();
        }
        return;
    }
    
    if (!_active) {
        static unsigned long lastDebug2 = 0;
        if (millis() - lastDebug2 > 5000) {
            Serial.println("[SDPlayerAdvanced] WARNING: Not active");
            lastDebug2 = millis();
        }
        return;
    }
    
    // DEBUG: Pokazuj co 5 sekund że renderujemy
    static unsigned long lastDebugRender = 0;
    if (millis() - lastDebugRender > 5000) {
        Serial.println("[SDPlayerAdvanced] Rendering OLED...");
        lastDebugRender = millis();
    }
    
    _display->clearBuffer();
    
    // ===== GÓRNA SEKCJA - TAGI ID3 =====
    _display->setFont(u8g2_font_7x13_tf);
    
    // Ikona statusu + Artist
    int iconX = 2;
    int line1Y = 12;
    
    if (_isPlaying && !_isPaused) {
        _display->drawXBMP(iconX, 2, 8, 8, icon_play_bits);
    } else if (_isPaused) {
        _display->drawXBMP(iconX, 2, 8, 8, icon_pause_bits);
    } else {
        _display->drawXBMP(iconX, 2, 8, 8, icon_stop_bits);
    }
    
    // Artist (jeśli dostępny)
    String artistText = "Artist: ";
    if (_currentTrackInfo.hasID3 && _currentTrackInfo.artist.length() > 0) {
        artistText += _currentTrackInfo.artist;
    } else {
        artistText += "Unknown";
    }
    
    // Obetnij jeśli za długi
    int maxWidth = 200;
    while (_display->getStrWidth(artistText.c_str()) > maxWidth && artistText.length() > 10) {
        artistText = artistText.substring(0, artistText.length() - 1);
    }
    
    _display->drawStr(14, line1Y, artistText.c_str());
    
    // Volume w prawym górnym rogu
    if (_audio) {
        int vol = _audio->getVolume();
        String volStr = String(vol);
        int volWidth = _display->getStrWidth(volStr.c_str());
        _display->drawStr(256 - volWidth - 2, line1Y, volStr.c_str());
    }
    
    // Title
    int line2Y = 24;
    _display->drawXBMP(iconX, 14, 8, 8, icon_music_bits);
    
    String titleText = "Title: ";
    if (_currentTrackInfo.hasID3 && _currentTrackInfo.title.length() > 0) {
        titleText += _currentTrackInfo.title;
    } else if (_currentFile.length() > 0 && _currentFile != "None") {
        // Użyj nazwy pliku bez rozszerzenia
        String fileName = _currentFile;
        int slashPos = fileName.lastIndexOf('/');
        if (slashPos >= 0) fileName = fileName.substring(slashPos + 1);
        int dotPos = fileName.lastIndexOf('.');
        if (dotPos > 0) fileName = fileName.substring(0, dotPos);
        titleText += fileName;
    } else {
        titleText += "Stopped";
    }
    
    while (_display->getStrWidth(titleText.c_str()) > maxWidth && titleText.length() > 10) {
        titleText = titleText.substring(0, titleText.length() - 1);
    }
    
    _display->drawStr(14, line2Y, titleText.c_str());
    
    // Album
    int line3Y = 36;
    String albumText = "Album: ";
    if (_currentTrackInfo.hasID3 && _currentTrackInfo.album.length() > 0) {
        albumText += _currentTrackInfo.album;
    } else {
        albumText += "-";
    }
    
    while (_display->getStrWidth(albumText.c_str()) > 230 && albumText.length() > 10) {
        albumText = albumText.substring(0, albumText.length() - 1);
    }
    
    _display->drawStr(2, line3Y, albumText.c_str());
    
    // ===== LINIA SEPARATORA =====
    _display->drawLine(0, 38, 256, 38);
    
    // ===== ŚRODKOWA SEKCJA - PARAMETRY AUDIO =====
    _display->setFont(u8g2_font_6x12_tf);
    int line4Y = 49;
    
    // Codec + Bitrate
    String codecStr = _currentTrackInfo.codecType;
    if (codecStr.length() == 0) codecStr = "-";
    _display->drawStr(2, line4Y, codecStr.c_str());
    
    String bitrateStr = String(_currentTrackInfo.bitrate) + "kbps";
    _display->drawStr(50, line4Y, bitrateStr.c_str());
    
    // SampleRate + BitsPerSample
    String srStr = String(_currentTrackInfo.sampleRate / 1000.0, 1) + "kHz";
    _display->drawStr(120, line4Y, srStr.c_str());
    
    String bpsStr = String(_currentTrackInfo.bitsPerSample) + "bit";
    _display->drawStr(180, line4Y, bpsStr.c_str());
    
    // ===== DOLNA SEKCJA - TIMER + REPEAT MODE =====
    int line5Y = 61;
    
    // Timer odtwarzania (mm:ss)
    if (_isPlaying && !_isPaused) {
        unsigned long elapsedMs = millis() - _playStartTime;
        unsigned int totalSeconds = elapsedMs / 1000;
        int minutes = totalSeconds / 60;
        int seconds = totalSeconds % 60;
        
        char timerStr[10];
        snprintf(timerStr, sizeof(timerStr), "%02d:%02d", minutes, seconds);
        _display->drawStr(2, line5Y, timerStr);
    } else {
        _display->drawStr(2, line5Y, "--:--");
    }
    
    // Repeat mode
    _display->drawXBMP(180, 52, 8, 8, icon_repeat_bits);
    
    String repeatStr = "";
    switch (_repeatMode) {
        case REPEAT_OFF: repeatStr = "OFF"; break;
        case REPEAT_ONE: repeatStr = "ONE"; break;
        case REPEAT_ALL: repeatStr = "ALL"; break;
        case REPEAT_FOLDER: repeatStr = "FOLDER"; break;
    }
    _display->drawStr(192, line5Y, repeatStr.c_str());
    
    _display->sendBuffer();
}

