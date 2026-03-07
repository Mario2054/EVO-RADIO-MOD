#pragma once
#include <Arduino.h>
#include <SD.h>
#include <FS.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

/**
 * SDRecorder - Nagrywanie strumienia radiowego do pliku na SD
 * 
 * Funkcje:
 * ✅ Nagrywanie strumienia audio w formacie WAV (PCM 48kHz stereo 16-bit)
 * ✅ Automatyczna nazwa pliku: REC_YYYYMMDD_HHMMSS_StationName.wav
 * ✅ Ring buffer 512KB z atomic operations (thread-safe)
 * ✅ Osobny FreeRTOS task do zapisu na SD (core 0)
 * ✅ Chunked writing (10KB chunks + flush co 64KB)
 * ✅ Wyświetlanie statusu nagrywania na OLED
 * ✅ Obsługa przez pilot IR (START/STOP/PAUSE)
 * ✅ Licznik czasu nagrywania
 * ✅ Gain 100x z saturacją (zapobieganie clipping)
 */

class SDRecorder {
public:
    // Status nagrywania
    enum RecordState {
        IDLE = 0,           // Nie nagrywa
        RECORDING = 1,      // Aktywnie nagrywa
        PAUSED = 2          // Pauza
    };

    SDRecorder();
    ~SDRecorder();
    
    // Inicjalizacja
    void begin();
    
    // Główna pętla - wywołuj w loop()
    void loop();
    
    // Kontrola nagrywania
    bool startRecording(const String& stationName = "");  // Rozpocznij nagrywanie
    void stopRecording();                                  // Zatrzymaj i zapisz
    void pauseRecording();                                 // Wstrzymaj (bez zamykania pliku)
    void resumeRecording();                                // Wznów nagrywanie
    void toggleRecording(const String& stationName = ""); // Toggle START/STOP
    
    // Push audio data (wywołaj z audio_process_i2s)
    void pushAudioData(const int16_t* buffer, int32_t frames);
    
    // Ustawienie wzmocnienia (gain) - domyślnie 4.0 (4x głośniej)
    void setGain(float gain) { _gain = gain; }
    float getGain() const { return _gain; }
    
    // Gettery statusu
    RecordState getState() const { return _state; }
    bool isRecording() const { return _state == RECORDING; }
    bool isPaused() const { return _state == PAUSED; }
    unsigned long getRecordTime() const;      // Czas nagrywania w sekundach
    String getRecordTimeString() const;       // Format: "HH:MM:SS"
    String getCurrentFileName() const { return _currentFileName; }
    size_t getFileSize() const { return _fileSize; }
    String getFileSizeString() const;         // Format: "12.5 MB"
    
    // Wyświetlanie na OLED (callback - ustaw z main.cpp)
    typedef void (*DisplayCallback)(const String& status, const String& time, const String& size);
    void setDisplayCallback(DisplayCallback callback) { _displayCallback = callback; }
    
    // Konfiguracja
    void setMaxFileSize(size_t maxMB);        // Max rozmiar pliku w MB (0 = unlimited)
    void setRecordPath(const String& path);   // Katalog zapisu (domyślnie "/RECORDINGS")
    
private:
    // Stan
    RecordState _state;
    File _recordFile;
    String _currentFileName;
    String _recordPath;
    
    // Statystyki
    unsigned long _recordStartTime;
    unsigned long _recordPauseTime;
    unsigned long _totalPauseTime;
    std::atomic<size_t> _fileSize;
    size_t _maxFileSize;
    
    // Ring Buffer (2MB w PSRAM) - thread-safe z atomic operations
    static const size_t RING_BUFFER_SIZE = 2 * 1024 * 1024;  // 2MB (~10-12 sekund @ 48kHz stereo)
    uint8_t* _ringBuffer;
    std::atomic<size_t> _writePos;    // atomic write pointer
    std::atomic<size_t> _readPos;     // atomic read pointer
    float _gain;                      // Wzmocnienie sygnału (4.0 = 4x głośniej)
    
    // FreeRTOS Task dla asynchronicznego zapisu
    TaskHandle_t _writerTaskHandle;
    SemaphoreHandle_t _fileMutex;
    volatile bool _stopWriterTask;
    
    // Chunked writing (8KB chunks + longer retry delays dla wolnych SD kart)
    static const size_t WRITE_CHUNK_SIZE = 8 * 1024;    // 8KB chunks (mniej operacji write = mniej overhead)
    static const size_t FLUSH_INTERVAL = 128 * 1024;    // Flush co 128KB (szybszy zapis niż flush po każdym write)
    size_t _bytesWrittenSinceFlush;
    
    // Callback
    DisplayCallback _displayCallback;
    
    // Ring Buffer Operations
    size_t getRingBufferAvailableSpace() const;
    size_t getRingBufferDataSize() const;
    bool writeToRingBuffer(const uint8_t* data, size_t size);
    size_t readFromRingBuffer(uint8_t* data, size_t maxSize);
    
    // FreeRTOS Task
    static void wavWriterTask(void* parameter);
    void writerTaskLoop();
    
    // Metody prywatne
    String generateFileName(const String& stationName);
    bool openRecordFile(const String& filename);
    void closeRecordFile();
    void ensureRecordDirectory();
    String formatFileSize(size_t bytes) const;
    void writeWavHeader(uint32_t sampleRate, uint16_t numChannels, uint16_t bitsPerSample, uint32_t dataSize);
    void updateWavHeader();
};

// Globalna instancja (deklaracja - definicja w .cpp)
extern SDRecorder* g_sdRecorder;
