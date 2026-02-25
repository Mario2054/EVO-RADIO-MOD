#include "SDRecorder.h"
#include <time.h>

// Globalna instancja
SDRecorder* g_sdRecorder = nullptr;

SDRecorder::SDRecorder() 
    : _state(IDLE)
    , _recordPath("/RECORDINGS")
    , _recordStartTime(0)
    , _recordPauseTime(0)
    , _totalPauseTime(0)
    , _fileSize(0)
    , _maxFileSize(0)
    , _writeBuffer(nullptr)
    , _bufferSize(32 * 1024)  // 32KB domyślnie
    , _bufferPos(0)
    , _gain(100.0f)           // Domyślnie 100x wzmocnienie (ekstremalne wzmocnienie z saturacją)
    , _displayCallback(nullptr)
{
}

SDRecorder::~SDRecorder() {
    if (_state != IDLE) {
        stopRecording();
    }
    if (_writeBuffer) {
        free(_writeBuffer);
        _writeBuffer = nullptr;
    }
}

void SDRecorder::begin() {
    // Alokuj bufor zapisu
    if (!_writeBuffer) {
        _writeBuffer = (uint8_t*)malloc(_bufferSize);
        if (!_writeBuffer) {
            Serial.println("[SDRecorder] ERROR: Failed to allocate write buffer!");
            _bufferSize = 0;
            return;
        }
    }
    
    // Utwórz katalog jeśli nie istnieje
    ensureRecordDirectory();
    
    Serial.println("[SDRecorder] Initialized successfully");
}

void SDRecorder::loop() {
    // Okresowy flush bufora (co 5 sekund) podczas nagrywania
    static unsigned long lastFlush = 0;
    if (_state == RECORDING && _bufferPos > 0) {
        if (millis() - lastFlush > 5000) {
            flushBuffer();
            lastFlush = millis();
        }
    }
    
    // Sprawdź limit rozmiaru pliku
    if (_maxFileSize > 0 && _fileSize >= _maxFileSize) {
        Serial.printf("[SDRecorder] Max file size reached: %zu bytes\n", _fileSize);
        stopRecording();
    }
}

bool SDRecorder::startRecording(const String& stationName) {
    if (_state != IDLE) {
        Serial.println("[SDRecorder] Already recording or paused!");
        return false;
    }
    
    if (!_writeBuffer) {
        Serial.println("[SDRecorder] ERROR: Write buffer not allocated!");
        return false;
    }
    
    // Generuj nazwę pliku
    String filename = generateFileName(stationName);
    _currentFileName = filename;
    
    // Otwórz plik
    if (!openRecordFile(filename)) {
        Serial.printf("[SDRecorder] ERROR: Failed to open file: %s\n", filename.c_str());
        return false;
    }
    
    // Zresetuj statystyki
    _recordStartTime = millis();
    _recordPauseTime = 0;
    _totalPauseTime = 0;
    _fileSize = 0;
    _bufferPos = 0;
    _state = RECORDING;
    
    Serial.printf("[SDRecorder] Started recording to: %s\n", filename.c_str());
    return true;
}

void SDRecorder::stopRecording() {
    if (_state == IDLE) {
        return;
    }
    
    // Flush pozostałe dane
    if (_bufferPos > 0) {
        flushBuffer();
    }
    
    // Zamknij plik
    closeRecordFile();
    
    unsigned long totalTime = getRecordTime();
    Serial.printf("[SDRecorder] Stopped recording. File: %s, Size: %zu bytes, Time: %lu sec\n",
                  _currentFileName.c_str(), _fileSize, totalTime);
    
    _state = IDLE;
    _currentFileName = "";
    _fileSize = 0;
    _bufferPos = 0;
}

void SDRecorder::pauseRecording() {
    if (_state != RECORDING) {
        return;
    }
    
    // Flush bufora przed pauzą
    if (_bufferPos > 0) {
        flushBuffer();
    }
    
    _recordPauseTime = millis();
    _state = PAUSED;
    Serial.println("[SDRecorder] Paused recording");
}

void SDRecorder::resumeRecording() {
    if (_state != PAUSED) {
        return;
    }
    
    // Dodaj czas pauzy do sumy
    _totalPauseTime += (millis() - _recordPauseTime);
    _state = RECORDING;
    Serial.println("[SDRecorder] Resumed recording");
}

void SDRecorder::toggleRecording(const String& stationName) {
    if (_state == IDLE) {
        startRecording(stationName);
    } else {
        stopRecording();
    }
}

void SDRecorder::pushAudioData(const int16_t* buffer, int32_t frames) {
    if (_state != RECORDING || !buffer || frames <= 0) {
        return;
    }
    
    // Oblicz rozmiar danych (stereo, 16-bit)
    size_t dataSize = frames * 2 * sizeof(int16_t);  // frames * 2 channels * 2 bytes
    
    // Sprawdź czy bufor się nie przepełni
    if (_bufferPos + dataSize > _bufferSize) {
        flushBuffer();
    }
    
    // Wzmocnienie + saturacja (zapobieganie clippingowi)
    int16_t* destBuffer = (int16_t*)(_writeBuffer + _bufferPos);
    int32_t sampleCount = frames * 2;  // stereo (L+R)
    
    for (int32_t i = 0; i < sampleCount; i++) {
        int32_t sample = (int32_t)(buffer[i] * _gain);
        
        // Saturacja do zakresu int16_t (-32768...32767)
        if (sample > 32767) sample = 32767;
        if (sample < -32768) sample = -32768;
        
        destBuffer[i] = (int16_t)sample;
    }
    
    _bufferPos += dataSize;
    _fileSize += dataSize;
}

unsigned long SDRecorder::getRecordTime() const {
    if (_state == IDLE) {
        return 0;
    }
    
    unsigned long elapsed = millis() - _recordStartTime - _totalPauseTime;
    
    if (_state == PAUSED) {
        elapsed -= (millis() - _recordPauseTime);
    }
    
    return elapsed / 1000;  // Zwróć w sekundach
}

String SDRecorder::getRecordTimeString() const {
    unsigned long totalSec = getRecordTime();
    unsigned long hours = totalSec / 3600;
    unsigned long minutes = (totalSec % 3600) / 60;
    unsigned long seconds = totalSec % 60;
    
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu", hours, minutes, seconds);
    return String(buffer);
}

String SDRecorder::getFileSizeString() const {
    return formatFileSize(_fileSize);
}

void SDRecorder::setBufferSize(size_t size) {
    if (_state != IDLE) {
        Serial.println("[SDRecorder] Cannot change buffer size while recording!");
        return;
    }
    
    if (_writeBuffer) {
        free(_writeBuffer);
    }
    
    _bufferSize = size;
    _writeBuffer = (uint8_t*)malloc(_bufferSize);
    
    if (!_writeBuffer) {
        Serial.println("[SDRecorder] ERROR: Failed to allocate new buffer!");
        _bufferSize = 0;
    }
}

void SDRecorder::setMaxFileSize(size_t maxMB) {
    _maxFileSize = maxMB * 1024 * 1024;  // Konwersja MB na bajty
}

void SDRecorder::setRecordPath(const String& path) {
    _recordPath = path;
    ensureRecordDirectory();
}

String SDRecorder::generateFileName(const String& stationName) {
    // Format: REC_YYYYMMDD_HHMMSS_StationName.raw
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);
    
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", timeinfo);
    
    String sanitizedName = stationName;
    // Usuń niedozwolone znaki z nazwy stacji
    sanitizedName.replace("/", "_");
    sanitizedName.replace("\\", "_");
    sanitizedName.replace(":", "_");
    sanitizedName.replace("*", "_");
    sanitizedName.replace("?", "_");
    sanitizedName.replace("\"", "_");
    sanitizedName.replace("<", "_");
    sanitizedName.replace(">", "_");
    sanitizedName.replace("|", "_");
    sanitizedName.trim();
    
    if (sanitizedName.length() == 0) {
        sanitizedName = "Unknown";
    }
    
    // Ogranicz długość nazwy
    if (sanitizedName.length() > 20) {
        sanitizedName = sanitizedName.substring(0, 20);
    }
    
    String filename = _recordPath + "/REC_" + String(timestamp) + "_" + sanitizedName + ".wav";
    return filename;
}

bool SDRecorder::openRecordFile(const String& filename) {
    _recordFile = SD.open(filename.c_str(), FILE_WRITE);
    if (!_recordFile) {
        return false;
    }
    
    // Zapisz nagłówek WAV (44 bajty)
    // Format: 48kHz (Audio library resampluje wszystko do 48kHz!), Stereo, 16-bit PCM
    writeWavHeader(48000, 2, 16, 0);  // rozmiar danych = 0 (zaktualizujemy przy zamknięciu)
    
    return true;
}

void SDRecorder::closeRecordFile() {
    if (_recordFile) {
        // Zaktualizuj rozmiary w nagłówku WAV przed zamknięciem
        updateWavHeader();
        _recordFile.close();
    }
}

void SDRecorder::flushBuffer() {
    if (!_recordFile || _bufferPos == 0) {
        return;
    }
    
    size_t written = _recordFile.write(_writeBuffer, _bufferPos);
    
    if (written != _bufferPos) {
        Serial.printf("[SDRecorder] WARNING: Write error! Expected %zu, written %zu\n", 
                      _bufferPos, written);
    }
    
    _bufferPos = 0;
    
    // Callback do wyświetlacza (opcjonalnie)
    if (_displayCallback) {
        _displayCallback(
            (_state == RECORDING ? "RECORDING" : "PAUSED"),
            getRecordTimeString(),
            getFileSizeString()
        );
    }
}

void SDRecorder::ensureRecordDirectory() {
    if (!SD.exists(_recordPath.c_str())) {
        if (SD.mkdir(_recordPath.c_str())) {
            Serial.printf("[SDRecorder] Created directory: %s\n", _recordPath.c_str());
        } else {
            Serial.printf("[SDRecorder] ERROR: Failed to create directory: %s\n", _recordPath.c_str());
        }
    }
}

void SDRecorder::writeWavHeader(uint32_t sampleRate, uint16_t numChannels, uint16_t bitsPerSample, uint32_t dataSize) {
    // Nagłówek WAV (RIFF format)
    uint32_t byteRate = sampleRate * numChannels * (bitsPerSample / 8);
    uint16_t blockAlign = numChannels * (bitsPerSample / 8);
    uint32_t chunkSize = 36 + dataSize;  // 36 = rozmiar nagłówka bez "RIFF" i ChunkSize
    
    // RIFF Header (12 bytes)
    _recordFile.write((const uint8_t*)"RIFF", 4);
    _recordFile.write((const uint8_t*)&chunkSize, 4);
    _recordFile.write((const uint8_t*)"WAVE", 4);
    
    // fmt chunk (24 bytes)
    _recordFile.write((const uint8_t*)"fmt ", 4);
    uint32_t fmtSize = 16;  // Rozmiar fmt chunk (PCM)
    _recordFile.write((const uint8_t*)&fmtSize, 4);
    uint16_t audioFormat = 1;  // 1 = PCM
    _recordFile.write((const uint8_t*)&audioFormat, 2);
    _recordFile.write((const uint8_t*)&numChannels, 2);
    _recordFile.write((const uint8_t*)&sampleRate, 4);
    _recordFile.write((const uint8_t*)&byteRate, 4);
    _recordFile.write((const uint8_t*)&blockAlign, 2);
    _recordFile.write((const uint8_t*)&bitsPerSample, 2);
    
    // data chunk header (8 bytes)
    _recordFile.write((const uint8_t*)"data", 4);
    _recordFile.write((const uint8_t*)&dataSize, 4);
    
    Serial.println("[SDRecorder] WAV header written (44 bytes)");
}

void SDRecorder::updateWavHeader() {
    // Przejdź do początku pliku i zaktualizuj rozmiary
    _recordFile.seek(4);  // Pozycja ChunkSize (RIFF)
    uint32_t chunkSize = 36 + _fileSize;
    _recordFile.write((const uint8_t*)&chunkSize, 4);
    
    _recordFile.seek(40);  // Pozycja DataSize (data chunk)
    _recordFile.write((const uint8_t*)&_fileSize, 4);
    
    Serial.printf("[SDRecorder] WAV header updated: data size = %zu bytes\n", _fileSize);
}

String SDRecorder::formatFileSize(size_t bytes) const {
    if (bytes < 1024) {
        return String(bytes) + " B";
    } else if (bytes < 1024 * 1024) {
        return String(bytes / 1024.0, 1) + " KB";
    } else if (bytes < 1024 * 1024 * 1024) {
        return String(bytes / (1024.0 * 1024.0), 1) + " MB";
    } else {
        return String(bytes / (1024.0 * 1024.0 * 1024.0), 2) + " GB";
    }
}
