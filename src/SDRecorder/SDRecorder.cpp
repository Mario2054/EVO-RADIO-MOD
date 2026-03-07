#include "SDRecorder.h"
#include <time.h>
#include <esp_heap_caps.h>

// Globalna instancja
SDRecorder* g_sdRecorder = nullptr;

SDRecorder::SDRecorder() 
    : _state(IDLE)
    , _recordPath("/RECORDINGS")
    , _recordStartTime(0)
    , _recordPauseTime(0)
    , _totalPauseTime(0)
    , _maxFileSize(0)
    , _ringBuffer(nullptr)
    , _gain(10.0f)              // 10x wzmocnienie z saturacją
    , _writerTaskHandle(nullptr)
    , _fileMutex(nullptr)
    , _stopWriterTask(false)
    , _bytesWrittenSinceFlush(0)
    , _displayCallback(nullptr)
{
    _fileSize.store(0);
    _writePos.store(0);
    _readPos.store(0);
}

SDRecorder::~SDRecorder() {
    if (_state != IDLE) {
        stopRecording();
    }
    
    // Stop writer task
    if (_writerTaskHandle) {
        _stopWriterTask = true;
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelete(_writerTaskHandle);
        _writerTaskHandle = nullptr;
    }
    
    // Delete mutex
    if (_fileMutex) {
        vSemaphoreDelete(_fileMutex);
        _fileMutex = nullptr;
    }
    
    // Free ring buffer (PSRAM lub RAM)
    if (_ringBuffer) {
        heap_caps_free(_ringBuffer);  // działa dla obu: PSRAM i RAM  
        _ringBuffer = nullptr;
    }
}

void SDRecorder::begin() {
    // Alokuj ring buffer (2MB w PSRAM - ESP32 ma 8MB PSRAM)
    if (!_ringBuffer) {
        // Próbuj alokować w PSRAM (dużo szybciej niż SD, wolniej niż SRAM)
        _ringBuffer = (uint8_t*)heap_caps_malloc(RING_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
        if (!_ringBuffer) {
            // Fallback do zwykłej RAM (jeśli PSRAM niedostępna)
            Serial.println("[SDRecorder] WARNING: PSRAM not available, using regular RAM");
            _ringBuffer = (uint8_t*)malloc(RING_BUFFER_SIZE);
        }
        
        if (!_ringBuffer) {
            Serial.println("[SDRecorder] ERROR: Failed to allocate ring buffer (2MB)!");
            return;
        }
        Serial.printf("[SDRecorder] Ring buffer allocated: %d KB (in %s)\n", 
                      RING_BUFFER_SIZE / 1024,
                      heap_caps_get_free_size(MALLOC_CAP_SPIRAM) > 0 ? "PSRAM" : "RAM");
    }
    
    // Utwórz mutex
    if (!_fileMutex) {
        _fileMutex = xSemaphoreCreateMutex();
        if (!_fileMutex) {
            Serial.println("[SDRecorder] ERROR: Failed to create mutex!");
            return;
        }
    }
    
    // Utwórz FreeRTOS task na core 0 (zapis na SD)
    if (!_writerTaskHandle) {
        BaseType_t result = xTaskCreatePinnedToCore(
            wavWriterTask,          // Function
            "WAV_Writer",           // Name
            16384,                  // Stack size (16KB - zwiększone dla stabilności)
            this,                   // Parameter
            3,                      // Priority (3 = bardzo wysoki priorytet dla szybkiego zapisu)
            &_writerTaskHandle,     // Handle
            0                       // Core 0
        );
        
        if (result != pdPASS) {
            Serial.println("[SDRecorder] ERROR: Failed to create writer task!");
            return;
        }
        Serial.println("[SDRecorder] Writer task created on core 0");
    }
    
    ensureRecordDirectory();
    Serial.println("[SDRecorder] Initialized (Ring buffer + FreeRTOS task)");
    g_sdRecorder = this;
}

void SDRecorder::loop() {
    if (_state == IDLE) return;
    
    // Sprawdź limit rozmiaru pliku
    if (_maxFileSize > 0 && _fileSize.load() >= _maxFileSize) {
        Serial.println("[SDRecorder] Max file size reached, stopping...");
        stopRecording();
        return;
    }
    
    // Callback wyświetlania (co 500ms)
    static unsigned long lastDisplay = 0;
    unsigned long now = millis();
    if ((now - lastDisplay) >= 500) {
        lastDisplay = now;
        if (_displayCallback) {
            String status = (_state == RECORDING) ? "RECORDING" : "PAUSED";
            _displayCallback(status, getRecordTimeString(), getFileSizeString());
        }
    }
}

bool SDRecorder::startRecording(const String& stationName) {
    if (_state != IDLE) {
        Serial.println("[SDRecorder] Already recording!");
        return false;
    }
    
    String filename = generateFileName(stationName);
    if (!openRecordFile(filename)) {
        return false;
    }
    
    _currentFileName = filename;
    _recordStartTime = millis();
    _totalPauseTime = 0;
    _fileSize.store(0);
    _writePos.store(0);
    _readPos.store(0);
    _bytesWrittenSinceFlush = 0;
    _stopWriterTask = false;
    _state = RECORDING;
    
    Serial.printf("[SDRecorder] Started recording to: %s\n", filename.c_str());
    return true;
}

void SDRecorder::stopRecording() {
    if (_state == IDLE) return;
    
    _state = IDLE;
    
    // Zaczekaj aż writer task oczyści ring buffer
    unsigned long timeout = millis() + 5000;  // 5 sekund timeout
    while (getRingBufferDataSize() > 0 && millis() < timeout) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Zablokuj dostęp do pliku
    if (xSemaphoreTake(_fileMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        // Final flush przed zamknięciem
        _recordFile.flush();
        
        // Update WAV header
        updateWavHeader();
        
        // Close file
        size_t finalSize = _fileSize.load();
        _recordFile.close();
        
        xSemaphoreGive(_fileMutex);
        
        unsigned long recordTime = (millis() - _recordStartTime - _totalPauseTime) / 1000;
        
        Serial.printf("[SDRecorder] Stopped recording. File: %s, Size: %zu bytes, Time: %lu sec\n",
                      _currentFileName.c_str(), finalSize, recordTime);
    } else {
        Serial.println("[SDRecorder] ERROR: Failed to acquire mutex for file close!");
    }
    
    _currentFileName = "";
}

void SDRecorder::pauseRecording() {
    if (_state != RECORDING) return;
    
    _recordPauseTime = millis();
    _state = PAUSED;
    Serial.println("[SDRecorder] Paused");
}

void SDRecorder::resumeRecording() {
    if (_state != PAUSED) return;
    
    _totalPauseTime += (millis() - _recordPauseTime);
    _state = RECORDING;
    Serial.println("[SDRecorder] Resumed");
}

void SDRecorder::toggleRecording(const String& stationName) {
    if (_state == IDLE) {
        startRecording(stationName);
    } else {
        stopRecording();
    }
}

void SDRecorder::pushAudioData(const int16_t* buffer, int32_t frames) {
    if (_state != RECORDING || !buffer || frames <= 0) return;
    
    const int32_t channels = 2;  // stereo
    const int32_t sampleCount = frames * channels;
    const size_t dataSize = sampleCount * sizeof(int16_t);
    
    // Check available space in ring buffer
    size_t availableSpace = getRingBufferAvailableSpace();
    if (availableSpace < dataSize) {
        Serial.println("[SDRecorder] WARNING: Ring buffer full, dropping samples!");
        return;
    }
    
    // Temporary buffer for gain application
    int16_t* processedBuffer = (int16_t*)malloc(dataSize);
    if (!processedBuffer) {
        Serial.println("[SDRecorder] ERROR: Failed to allocate temp buffer!");
        return;
    }
    
    // Apply gain and saturate to prevent clipping
    for (int32_t i = 0; i < sampleCount; i++) {
        int32_t sample = (int32_t)(buffer[i] * _gain);
        
        // Saturate
        if (sample > 32767) sample = 32767;
        if (sample < -32768) sample = -32768;
        
        processedBuffer[i] = (int16_t)sample;
    }
    
    // Write to ring buffer
    if (writeToRingBuffer((uint8_t*)processedBuffer, dataSize)) {
        _fileSize.fetch_add(dataSize);
    }
    
    free(processedBuffer);
}

// Ring Buffer Operations
size_t SDRecorder::getRingBufferAvailableSpace() const {
    size_t writePos = _writePos.load(std::memory_order_acquire);
    size_t readPos = _readPos.load(std::memory_order_acquire);
    
    if (writePos >= readPos) {
        return RING_BUFFER_SIZE - (writePos - readPos) - 1;
    } else {
        return readPos - writePos - 1;
    }
}

size_t SDRecorder::getRingBufferDataSize() const {
    size_t writePos = _writePos.load(std::memory_order_acquire);
    size_t readPos = _readPos.load(std::memory_order_acquire);
    
    if (writePos >= readPos) {
        return writePos - readPos;
    } else {
        return RING_BUFFER_SIZE - (readPos - writePos);
    }
}

bool SDRecorder::writeToRingBuffer(const uint8_t* data, size_t size) {
    if (!data || size == 0) return false;
    
    size_t writePos = _writePos.load(std::memory_order_acquire);
    size_t readPos = _readPos.load(std::memory_order_acquire);
    
    // Check available space
    size_t available;
    if (writePos >= readPos) {
        available = RING_BUFFER_SIZE - (writePos - readPos) - 1;
    } else {
        available = readPos - writePos - 1;
    }
    
    if (available < size) {
        return false;  // Not enough space
    }
    
    // Write data (handle wrap-around)
    size_t firstChunk = min(size, RING_BUFFER_SIZE - writePos);
    memcpy(_ringBuffer + writePos, data, firstChunk);
    
    if (firstChunk < size) {
        // Wrap around
        memcpy(_ringBuffer, data + firstChunk, size - firstChunk);
    }
    
    // Update write position (atomic)
    size_t newWritePos = (writePos + size) % RING_BUFFER_SIZE;
    _writePos.store(newWritePos, std::memory_order_release);
    
    return true;
}

size_t SDRecorder::readFromRingBuffer(uint8_t* data, size_t maxSize) {
    if (!data || maxSize == 0) return 0;
    
    size_t writePos = _writePos.load(std::memory_order_acquire);
    size_t readPos = _readPos.load(std::memory_order_acquire);
    
    // Check available data
    size_t available;
    if (writePos >= readPos) {
        available = writePos - readPos;
    } else {
        available = RING_BUFFER_SIZE - (readPos - writePos);
    }
    
    if (available == 0) {
        return 0;  // No data
    }
    
    size_t toRead = min(available, maxSize);
    
    // Read data (handle wrap-around)
    size_t firstChunk = min(toRead, RING_BUFFER_SIZE - readPos);
    memcpy(data, _ringBuffer + readPos, firstChunk);
    
    if (firstChunk < toRead) {
        // Wrap around
        memcpy(data + firstChunk, _ringBuffer, toRead - firstChunk);
    }
    
    // Update read position (atomic)
    size_t newReadPos = (readPos + toRead) % RING_BUFFER_SIZE;
    _readPos.store(newReadPos, std::memory_order_release);
    
    return toRead;
}

// FreeRTOS Task - static wrapper
void SDRecorder::wavWriterTask(void* parameter) {
    SDRecorder* recorder = (SDRecorder*)parameter;
    recorder->writerTaskLoop();
}

// FreeRTOS Task - main loop
void SDRecorder::writerTaskLoop() {
    uint8_t* writeBuffer = (uint8_t*)malloc(WRITE_CHUNK_SIZE);
    if (!writeBuffer) {
        Serial.println("[SDRecorder Writer] ERROR: Failed to allocate write buffer!");
        vTaskDelete(nullptr);
        return;
    }
    
    Serial.println("[SDRecorder Writer] Task started");
    
    while (!_stopWriterTask) {
        // Sprawdź czy jest coś do zapisu
        size_t dataSize = getRingBufferDataSize();
        
        if (_state == RECORDING && dataSize >= WRITE_CHUNK_SIZE) {
            // Read chunk from ring buffer
            size_t bytesRead = readFromRingBuffer(writeBuffer, WRITE_CHUNK_SIZE);
            
            if (bytesRead > 0) {
                // Zablokuj dostęp do pliku (bez delay - mutex chroni przed konfliktami)
                if (xSemaphoreTake(_fileMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    // Sprawdź czy plik jest otwarty
                    if (_recordFile) {
                        // Retry mechanism: 3 próby z krótszymi opóźnieniami
                        size_t written = 0;
                        int retries = 3;
                        
                        for (int attempt = 1; attempt <= retries; attempt++) {
                            written = _recordFile.write(writeBuffer, bytesRead);
                            
                            if (written == bytesRead) {
                                // Success - flush co 128KB (szybszy zapis)
                                _bytesWrittenSinceFlush += bytesRead;
                                if (_bytesWrittenSinceFlush >= FLUSH_INTERVAL) {
                                    _recordFile.flush();
                                    _bytesWrittenSinceFlush = 0;
                                }
                                break;
                            } else {
                                Serial.printf("[SDRecorder Writer] WARNING: Write error! Expected %zu, written %zu (retry %d/%d)\n", 
                                              bytesRead, written, attempt, retries);
                                
                                if (attempt < retries) {
                                    // Długie opóźnienia dla slow SD cards (dać karcie czas na commit)
                                    int delays[] = {200, 500, 1000};
                                    vTaskDelay(pdMS_TO_TICKS(delays[attempt - 1]));  // 200ms, 500ms, 1000ms
                                    taskYIELD();
                                } else {
                                    Serial.println("[SDRecorder Writer] CRITICAL: Write failed after 3 retries! Data lost.");
                                }
                            }
                        }
                    }
                    
                    xSemaphoreGive(_fileMutex);
                } else {
                    Serial.println("[SDRecorder Writer] WARNING: Failed to acquire mutex!");
                }
            }
            
            // Yield to other tasks (especially display on SPI)
            taskYIELD();
            
        } else {
            // No data or not recording - sleep longer
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
    free(writeBuffer);
    Serial.println("[SDRecorder Writer] Task stopped");
    vTaskDelete(nullptr);
}

String SDRecorder::generateFileName(const String& stationName) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &timeinfo);
    
    String filename = _recordPath + "/REC_" + String(timestamp);
    if (stationName.length() > 0) {
        filename += "_" + stationName;
    }
    filename += ".wav";
    
    return filename;
}

bool SDRecorder::openRecordFile(const String& filename) {
    if (xSemaphoreTake(_fileMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        _recordFile = SD.open(filename.c_str(), FILE_WRITE);
        xSemaphoreGive(_fileMutex);
        
        if (!_recordFile) {
            Serial.printf("[SDRecorder] ERROR: Cannot create file: %s\n", filename.c_str());
            return false;
        }
        
        // Write WAV header (placeholder)
        writeWavHeader(48000, 2, 16, 0);
        
        Serial.printf("[SDRecorder] WAV header written (44 bytes)\n");
        return true;
    } else {
        Serial.println("[SDRecorder] ERROR: Failed to acquire mutex for file open!");
        return false;
    }
}

void SDRecorder::closeRecordFile() {
    if (xSemaphoreTake(_fileMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (_recordFile) {
            _recordFile.close();
        }
        xSemaphoreGive(_fileMutex);
    }
}

void SDRecorder::ensureRecordDirectory() {
    if (!SD.exists(_recordPath.c_str())) {
        SD.mkdir(_recordPath.c_str());
        Serial.printf("[SDRecorder] Created directory: %s\n", _recordPath.c_str());
    }
}

void SDRecorder::setMaxFileSize(size_t maxMB) {
    _maxFileSize = maxMB * 1024 * 1024;
}

void SDRecorder::setRecordPath(const String& path) {
    _recordPath = path;
    ensureRecordDirectory();
}

unsigned long SDRecorder::getRecordTime() const {
    if (_state == IDLE) return 0;
    
    unsigned long elapsed = millis() - _recordStartTime - _totalPauseTime;
    if (_state == PAUSED) {
        elapsed -= (millis() - _recordPauseTime);
    }
    
    return elapsed / 1000;  // seconds
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
    return formatFileSize(_fileSize.load());
}

String SDRecorder::formatFileSize(size_t bytes) const {
    if (bytes < 1024) {
        return String(bytes) + " B";
    } else if (bytes < 1024 * 1024) {
        return String(bytes / 1024.0, 1) + " KB";
    } else if (bytes < 1024 * 1024 * 1024) {
        return String(bytes / (1024.0 * 1024.0), 1) + " MB";
    } else {
        return String(bytes / (1024.0 * 1024.0 * 1024.0), 1) + " GB";
    }
}

void SDRecorder::writeWavHeader(uint32_t sampleRate, uint16_t numChannels, uint16_t bitsPerSample, uint32_t dataSize) {
    uint32_t byteRate = sampleRate * numChannels * (bitsPerSample / 8);
    uint16_t blockAlign = numChannels * (bitsPerSample / 8);
    uint32_t chunkSize = 36 + dataSize;
    
    // RIFF header
    _recordFile.write((const uint8_t*)"RIFF", 4);
    _recordFile.write((const uint8_t*)&chunkSize, 4);
    _recordFile.write((const uint8_t*)"WAVE", 4);
    
    // fmt subchunk
    _recordFile.write((const uint8_t*)"fmt ", 4);
    uint32_t subchunk1Size = 16;
    _recordFile.write((const uint8_t*)&subchunk1Size, 4);
    uint16_t audioFormat = 1;  // PCM
    _recordFile.write((const uint8_t*)&audioFormat, 2);
    _recordFile.write((const uint8_t*)&numChannels, 2);
    _recordFile.write((const uint8_t*)&sampleRate, 4);
    _recordFile.write((const uint8_t*)&byteRate, 4);
    _recordFile.write((const uint8_t*)&blockAlign, 2);
    _recordFile.write((const uint8_t*)&bitsPerSample, 2);
    
    // data subchunk
    _recordFile.write((const uint8_t*)"data", 4);
    _recordFile.write((const uint8_t*)&dataSize, 4);
}

void SDRecorder::updateWavHeader() {
    if (!_recordFile) return;
    
    uint32_t dataSize = _fileSize.load();
    uint32_t chunkSize = 36 + dataSize;
    
    _recordFile.seek(4);
    _recordFile.write((const uint8_t*)&chunkSize, 4);
    
    _recordFile.seek(40);
    _recordFile.write((const uint8_t*)&dataSize, 4);
    
    Serial.printf("[SDRecorder] WAV header updated: data size = %u bytes\n", dataSize);
}
