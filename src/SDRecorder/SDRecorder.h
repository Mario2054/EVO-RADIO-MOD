#pragma once
#include <Arduino.h>
#include <SD.h>
#include <FS.h>

/**
 * SDRecorder - Nagrywanie strumienia radiowego do pliku na SD
 * 
 * Funkcje:
 * ✅ Nagrywanie strumienia audio w formacie surowym (RAW PCM)
 * ✅ Automatyczna nazwa pliku: REC_YYYYMMDD_HHMMSS_StationName.raw
 * ✅ Wyświetlanie statusu nagrywania na OLED
 * ✅ Obsługa przez pilot IR (START/STOP/PAUSE)
 * ✅ Licznik czasu nagrywania
 * ✅ Buforowanie danych przed zapisem (minimalizacja zapisów)
 * ✅ Wsparcie dla różnych formatów (MP3 stream passthrough - przyszłość)
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
    void setBufferSize(size_t size);          // Rozmiar bufora zapisu (domyślnie 32KB)
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
    size_t _fileSize;
    size_t _maxFileSize;
    
    // Bufor
    uint8_t* _writeBuffer;
    size_t _bufferSize;
    size_t _bufferPos;
    float _gain;                  // Wzmocnienie sygnału (1.0 = bez zmian, 4.0 = 4x głośniej)
    
    // Callback
    DisplayCallback _displayCallback;
    
    // Metody prywatne
    String generateFileName(const String& stationName);
    bool openRecordFile(const String& filename);
    void closeRecordFile();
    void flushBuffer();
    void ensureRecordDirectory();
    String formatFileSize(size_t bytes) const;
    void writeWavHeader(uint32_t sampleRate, uint16_t numChannels, uint16_t bitsPerSample, uint32_t dataSize);
    void updateWavHeader();
};

// Globalna instancja (deklaracja - definicja w .cpp)
extern SDRecorder* g_sdRecorder;
