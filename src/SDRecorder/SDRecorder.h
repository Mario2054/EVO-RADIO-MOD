#pragma once
#include <Arduino.h>
#include <SD.h>
#include <FS.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <freertos/stream_buffer.h>

/**
 * SDRecorder v2 - Nagrywanie strumienia radiowego bezposrednio jako MP3
 *
 * Nowe podejscie (przepisane na wzor ESP32_internet_radio_v3):
 * Nagrywanie strumienia audio w formacie MP3 (bezstratna kopia strumienia z serwera)
 * Male pliki: 128 kbps ~ 1 MB/min (vs ~10 MB/min dla WAV w starej wersji)
 * Obsluga HTTP i HTTPS (z samopodpisanymi certyfikatami - setInsecure)
 * Obsluga Transfer-Encoding: chunked (automatyczne wykrywanie i parsowanie)
 * Obsluga redirectow HTTP (rekurencyjnie, max 5 poziomow)
 * Automatyczna nazwa pliku: REC_YYYYMMDD_HHMMSS_StationName.mp3
 * Prosty model: loop() odczytuje TCP -> SD, brak FreeRTOS task, brak ring buffer
 * Timeout 20s - automatyczne zatrzymanie przy braku danych
 * Zgodny interfejs z poprzednia wersja (te same public metody)
 */

class SDRecorder {
public:
    // Stan nagrywania (zgodny wstecz z poprzednia wersja)
    enum RecordState {
        IDLE      = 0,
        RECORDING = 1,
        PAUSED    = 2   // nieuzywany w v2, zachowany dla zgodnosci
    };

    SDRecorder();
    ~SDRecorder();

    // Inicjalizacja - wywolaj raz w setup()
    void begin();

    // Glowna petla - wywoluj w loop() przez SDRecorder_loop()
    // Odczytuje dane z TCP i zapisuje na SD
    void loop();

    // Kontrola nagrywania
    // stationUrl jest wymagany - to adres strumienia (np. "http://stream.example.com/live")
    bool startRecording(const String& stationName = "", const String& stationUrl = "");
    void stopRecording();
    void toggleRecording(const String& stationName = "", const String& stationUrl = "");

    // Gettery statusu
    RecordState   getState()            const { return _state; }
    bool          isRecording()         const { return _state == RECORDING; }
    bool          isPaused()            const { return _state == PAUSED; }
    unsigned long getRecordTime()       const;
    String        getRecordTimeString() const;
    String        getCurrentFileName()  const { return _currentFileName; }
    size_t        getFileSize()         const { return _fileSize; }
    String        getFileSizeString()   const;

    // Callback statusu na OLED (taki sam interfejs jak v1)
    typedef void (*DisplayCallback)(const String& status, const String& time, const String& size);
    void setDisplayCallback(DisplayCallback callback) { _displayCallback = callback; }

    // Reset timeoutu — wywolaj przed dluga operacja (np. TTS), zeby recTask nie przerwał nagrywania
    void resetTimeout() { _lastRecordRead = millis(); }

    // Konfiguracja
    void setMaxFileSize(size_t maxMB);       // 0 = unlimited
    void setRecordPath(const String& path);  // domyslnie "/RECORDINGS"

private:
    // Stan
    RecordState   _state;
    File          _recordFile;
    String        _currentFileName;
    String        _recordPath;
    String        _stationUrl;

    // Statystyki
    unsigned long _recordStartTime;
    size_t        _fileSize;
    size_t        _maxFileSize;

    // Polaczenie TCP (HTTP / HTTPS)
    WiFiClient       _recClient;
    WiFiClientSecure _recSecure;
    bool             _useSSL;

    // Parsowanie HTTP
    bool          _headersRead;    // czy naglowki juz przeczytane w tej sesji
    bool          _isChunked;      // Transfer-Encoding: chunked
    String        _fileExt;        // rozszerzenie pliku wykryte z Content-Type (.mp3/.flac/.aac/.ogg)
    unsigned long _lastRecordRead; // timestamp ostatniego bajtu - do timeoutu

    // Bufor zapisu SD (loop() / stopRecording() — Core 1)
    // ODDZIELNY od bufora TCP w recTask (Core 0) — eliminuje race condition!
    // 512 = rozmiar 1 sektora SD = 1 deskryptor DMA (12 bajtow)
    // 4096 wymaga 8 deskryptorow DMA — przy sfragmentowanym heap alokacja nie wychodzi
    static const size_t BUFFER_SIZE = 512;
    uint8_t*      _buffer;
    size_t        _lastFlush;   // zamiennik static lastFlush z loop()

    // FreeRTOS task (Core 0) — czyta TCP i wrzuca do _streamBuf
    // loop() (Core 1) drainuje _streamBuf i zapisuje na SD
    // Dzieki temu wszystkie operacje SD sa z jednego kontekstu — brak konfliktow SPI
    volatile bool        _stopRequested;
    TaskHandle_t         _recTaskHandle;
    StreamBufferHandle_t _streamBuf;   // thread-safe SPSC, 128 KB
    static void          recTask(void* param);

    // Callback
    DisplayCallback _displayCallback;

    // Polaczenie ze stacja (z obsluga redirectow)
    bool connectStream(const String& url, int depth = 0);

    // Pomocnicze parsowanie HTTP
    String readHttpLine(WiFiClient& c);
    int    readChunkSize(WiFiClient& c);
    int    readHttpHeaders(WiFiClient& c, String& redirectUrl);

    // Generowanie nazwy pliku i folderu
    String generateFileName(const String& stationName);
    void   ensureRecordDirectory();

    // Formatowanie rozmiaru pliku
    String formatFileSize(size_t bytes) const;
};

// Globalna instancja (deklaracja - definicja w .cpp)
extern SDRecorder* g_sdRecorder;
