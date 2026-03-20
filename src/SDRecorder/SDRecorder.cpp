#include "SDRecorder.h"
#include <time.h>

// ============================================================================
// SDRecorder v2 — nagrywanie strumienia HTTP/HTTPS bezposrednio jako MP3
// Przepisane na wzor ESP32_internet_radio_v3 (brak ring buffer, brak FreeRTOS)
// ============================================================================

// Globalna instancja
SDRecorder* g_sdRecorder = nullptr;

SDRecorder::SDRecorder()
    : _state(IDLE)
    , _recordPath("/RECORDINGS")
    , _recordStartTime(0)
    , _fileSize(0)
    , _maxFileSize(0)
    , _useSSL(false)
    , _headersRead(false)
    , _isChunked(false)
    , _fileExt(".mp3")
    , _lastRecordRead(0)
    , _buffer(nullptr)
    , _lastFlush(0)
    , _stopRequested(false)
    , _recTaskHandle(nullptr)
    , _streamBuf(nullptr)
    , _displayCallback(nullptr)
{
}

SDRecorder::~SDRecorder() {
    if (_state != IDLE) stopRecording();
    if (_buffer) { free(_buffer); _buffer = nullptr; }
}

void SDRecorder::begin() {
    // _buffer musi byc w INTERNAL DMA-capable RAM (nie PSRAM!)
    // SD/SPI DMA nie moze czytac z PSRAM — write() zwracaloby 0
    if (!_buffer) {
        _buffer = (uint8_t*)heap_caps_malloc(BUFFER_SIZE,
                      MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        if (!_buffer) _buffer = (uint8_t*)malloc(BUFFER_SIZE);
        if (!_buffer) {
            Serial.println("[SDRecorder] ERROR: Nie mozna zaalokowac bufora SD!");
            return;
        }
        Serial.printf("[SDRecorder] Bufor SD zaalokowany: %zu KB (internal DMA RAM)\n",
                      BUFFER_SIZE / 1024);
    }

    ensureRecordDirectory();
    g_sdRecorder = this;
    Serial.println("[SDRecorder] Initialized (HTTP Stream -> MP3)");
}

// ============================================================================
// loop() — wywolany z SDRecorder_loop() w main.cpp (Core 1, glowna petla)
// Drainuje StreamBuffer do SD — wszystkie operacje SD sa z TEGO samego kontekstu
// co SDPlayer, WebUI etc. Eliminuje konflikty SPI ktore powodowaly utrate danych.
// ============================================================================
void SDRecorder::loop() {
    if (_state == IDLE) return;

    // Drainuj StreamBuffer do SD — max 16 * 4 KB = 64 KB na jedno wywolanie loop()
    // Mniejsza liczba iteracji = krotsze trzymanie magistrali SPI, wiecej czasu dla WiFi/WebSocket
    if (_streamBuf) {
        int maxIter = 16;
        int failCount = 0;
        while (maxIter-- > 0) {
            size_t n = xStreamBufferReceive(_streamBuf, _buffer, BUFFER_SIZE, 0);
            if (n == 0) break;  // bufor pusty

            size_t written = _recordFile.write(_buffer, n);

            // Jesli zapis sie nie udal — czekaj 2ms i probuj jeszcze raz
            // (FatFS chwilowo nie mogl zaalokowac bufora sektorowego w internal RAM)
            if (written == 0) {
                delay(1);
                written = _recordFile.write(_buffer, n);
            }

            if (written > 0) {
                _fileSize += written;
                failCount = 0;
                if (written != n) {
                    Serial.printf("[SDRec] Write short: %u / %u bytes\n", written, n);
                }
            } else {
                failCount++;
                // Loguj tylko co 10 bledow, zeby nie zasmiecac Serial
                if (failCount == 1 || (failCount % 10) == 0) {
                    Serial.printf("[SDRec] BLAD ZAPISU SD! n=%u, heap=%u, maxBlok=%u, fail#%d\n",
                                  n, ESP.getFreeHeap(), ESP.getMaxAllocHeap(), failCount);
                }
                if (failCount >= 3) {
                    _recordFile.flush();
                    failCount = 0;
                }
            }

            // Flush co 64 KB zeby dane trafialy na karte w razie nagłego restartu
            if (_fileSize > _lastFlush && (_fileSize - _lastFlush >= 65536)) {
                _recordFile.flush();
                _lastFlush = _fileSize;
            }
        }

        // Jesli task skonczyl i bufor pusty — zamknij plik
        if (_recTaskHandle == nullptr && xStreamBufferIsEmpty(_streamBuf)) {
            _recordFile.flush();
            _recordFile.close();
            vStreamBufferDelete(_streamBuf);
            _streamBuf = nullptr;
            if (_useSSL) _recSecure.stop(); else _recClient.stop();
            Serial.printf("[SDRecorder] Recording complete. Rozmiar: %s, Czas: %s\n",
                          getFileSizeString().c_str(), getRecordTimeString().c_str());
            _state = IDLE;
            _currentFileName = "";
            return;
        }
    }

    // Sprawdz limit rozmiaru pliku
    if (_maxFileSize > 0 && _fileSize >= _maxFileSize) {
        Serial.println("[SDRecorder] Max rozmiar pliku osiagniety, zatrzymuje...");
        stopRecording();
        return;
    }

    // Callback wyswietlania co 500 ms
    static unsigned long lastDisplay = 0;
    unsigned long now = millis();
    if ((now - lastDisplay) >= 500) {
        lastDisplay = now;
        if (_displayCallback) {
            _displayCallback("RECORDING", getRecordTimeString(), getFileSizeString());
        }
    }
}

// ============================================================================
// startRecording() — otworz plik i polacz sie ze strumieniem
// ============================================================================
bool SDRecorder::startRecording(const String& stationName, const String& stationUrl) {
    if (_state != IDLE) {
        Serial.println("[SDRecorder] Juz nagrywa!");
        return false;
    }

    if (stationUrl.isEmpty()) {
        Serial.println("[SDRecorder] ERROR: Brak URL stacji!");
        return false;
    }

    if (!_buffer) {
        Serial.println("[SDRecorder] ERROR: begin() nie zostalo wywolane!");
        return false;
    }

    ensureRecordDirectory();

    String fileName = generateFileName(stationName);
    _recordFile = SD.open(fileName.c_str(), FILE_WRITE);
    if (!_recordFile) {
        Serial.printf("[SDRecorder] ERROR: Nie mozna otworzyc pliku: %s\n", fileName.c_str());
        return false;
    }

    _stationUrl    = stationUrl;
    _headersRead   = false;
    _isChunked     = false;
    _fileExt       = ".mp3";  // domyslnie, readHttpHeaders nadpisze jesli FLAC/AAC/OGG
    _fileSize      = 0;

    Serial.printf("[SDRecorder] Plik:  %s\n", fileName.c_str());
    Serial.printf("[SDRecorder] URL:   %s\n", stationUrl.c_str());

    // Polacz ze stacja radiowa przez HTTP/HTTPS
    if (!connectStream(stationUrl)) {
        Serial.println("[SDRecorder] ERROR: Polaczenie ze stacja nieudane");
        _recordFile.close();
        return false;
    }

    // connectStream() odczytalo juz naglowki HTTP przez readHttpHeaders()
    _headersRead = true;

    // Utworz StreamBuffer w PSRAM — 256 KB = ~8s bufora przy 250kbps
    // Duzy bufor chroni przed utrata danych gdy loop() jest chwilowo zajety
    static const size_t SB_SIZE = 262144;  // 256 KB w PSRAM
    _streamBuf = xStreamBufferCreateWithCaps(SB_SIZE, 1,
                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!_streamBuf) {
        // Fallback: 64 KB jesli PSRAM nie dostepne
        Serial.println("[SDRecorder] PSRAM 256KB nieudany, probuje 64KB PSRAM...");
        _streamBuf = xStreamBufferCreateWithCaps(65536, 1,
                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!_streamBuf) {
        Serial.println("[SDRecorder] PSRAM bufor nieudany, probuje 8 KB internal RAM...");
        _streamBuf = xStreamBufferCreate(8192, 1);
    }
    if (!_streamBuf) {
        Serial.println("[SDRecorder] ERROR: Brak pamieci na StreamBuffer!");
        _recordFile.close();
        if (_useSSL) _recSecure.stop(); else _recClient.stop();
        return false;
    }
    Serial.printf("[SDRecorder] StreamBuffer OK %u KB (free heap: %u, PSRAM: %u)\n",
                  SB_SIZE / 1024, ESP.getFreeHeap(), ESP.getFreePsram());

    _currentFileName = fileName;
    _recordStartTime = millis();
    _lastRecordRead  = millis();
    _stopRequested   = false;
    _lastFlush       = 0;        // reset flushowania dla nowego nagrania
    _state           = RECORDING;

    // Uruchom task na Core 0 — czyta TCP i wrzuca do StreamBuffer
    // Zapis na SD odbywa sie w loop() na Core 1 (razem z SDPlayer/WebUI)
    if (xTaskCreatePinnedToCore(recTask, "SDRec", 8192, this, 2, &_recTaskHandle, 0) != pdPASS) {
        Serial.println("[SDRecorder] ERROR: Nie mozna utworzyc task!");
        _state = IDLE;
        vStreamBufferDelete(_streamBuf);
        _streamBuf = nullptr;
        _recordFile.close();
        if (_useSSL) _recSecure.stop(); else _recClient.stop();
        return false;
    }

    Serial.printf("[SDRecorder] Nagrywanie STARTED -> %s\n", fileName.c_str());
    return true;
}

// ============================================================================
// stopRecording() — zasygnalizuj stop taskowi, zdrajnuj bufor, zamknij plik
// Wszystkie operacje SD sa z tego samego kontekstu (main loop / stopRecording)
// ============================================================================
void SDRecorder::stopRecording() {
    if (_state == IDLE) return;

    Serial.println("[SDRecorder] Stop requested...");
    _stopRequested = true;

    // Czekaj max 3s az task skonczy czytac TCP
    // W tym czasie drainujemy StreamBuffer na biezaco
    unsigned long waitEnd = millis() + 3000;
    while (_recTaskHandle != nullptr && millis() < waitEnd) {
        if (_streamBuf) {
            size_t n = xStreamBufferReceive(_streamBuf, _buffer, BUFFER_SIZE, 0);
            if (n > 0) {
                size_t written = _recordFile.write(_buffer, n);
                _fileSize += written;  // tylko faktycznie zapisane bajty
            }
        }
        delay(5);
    }

    // Finalny draining — zapisz wszystko co task zdazyl wrzucic do bufora
    if (_streamBuf) {
        size_t n;
        while ((n = xStreamBufferReceive(_streamBuf, _buffer, BUFFER_SIZE, 0)) > 0) {
            size_t written = _recordFile.write(_buffer, n);
            _fileSize += written;  // tylko faktycznie zapisane bajty
        }
        vStreamBufferDelete(_streamBuf);
        _streamBuf = nullptr;
    }

    if (_recordFile) { _recordFile.flush(); _recordFile.close(); }
    if (_useSSL) _recSecure.stop(); else _recClient.stop();

    Serial.printf("[SDRecorder] Nagrywanie zatrzymane. Rozmiar: %s, Czas: %s\n",
                  getFileSizeString().c_str(), getRecordTimeString().c_str());

    _state           = IDLE;
    _currentFileName = "";
}

void SDRecorder::toggleRecording(const String& stationName, const String& stationUrl) {
    if (_state == IDLE) {
        startRecording(stationName, stationUrl);
    } else {
        stopRecording();
    }
}

// ============================================================================
// connectStream() — polaczenie HTTP/HTTPS z obsluga redirectow (max 5)
// ============================================================================
bool SDRecorder::connectStream(const String& url, int depth) {
    if (depth > 5) {
        Serial.println("[SDRecorder] ERROR: Zbyt wiele redirectow!");
        return false;
    }

    String workUrl = url;
    bool isHttps = workUrl.startsWith("https://");
    _useSSL = isHttps;

    if (workUrl.startsWith("http://"))  workUrl = workUrl.substring(7);
    if (workUrl.startsWith("https://")) workUrl = workUrl.substring(8);

    String host, path = "/";
    uint16_t port = isHttps ? 443 : 80;

    int slashPos = workUrl.indexOf('/');
    if (slashPos >= 0) {
        host = workUrl.substring(0, slashPos);
        path = workUrl.substring(slashPos);
    } else {
        host = workUrl;
    }

    int colonPos = host.indexOf(':');
    if (colonPos >= 0) {
        port = (uint16_t)host.substring(colonPos + 1).toInt();
        host = host.substring(0, colonPos);
    }

    Serial.printf("[SDRec] Lacze: %s://%s:%u%s\n",
                  isHttps ? "https" : "http", host.c_str(), port, path.c_str());

    bool ok = false;
    if (isHttps) {
        _recSecure.setInsecure();  // akceptuje samopodpisane certyfikaty
        ok = _recSecure.connect(host.c_str(), port);
    } else {
        ok = _recClient.connect(host.c_str(), port);
    }

    if (!ok) {
        Serial.println("[SDRecorder] ERROR: Nie mozna polaczyc z hostem!");
        return false;
    }

    WiFiClient* c = isHttps ? (WiFiClient*)&_recSecure : (WiFiClient*)&_recClient;

    // Wyslij zadanie GET
    c->print(
        String("GET ") + path + " HTTP/1.1\r\n" +
        "Host: " + host + "\r\n" +
        "Connection: keep-alive\r\n" +
        "Icy-Metadata: 0\r\n\r\n"
    );

    // Odczytaj naglowki odpowiedzi (obsluga redirect)
    String redirectUrl;
    int hdr = readHttpHeaders(*c, redirectUrl);
    if (hdr == 1) {
        Serial.println("[SDRecorder] Redirect -> " + redirectUrl);
        c->stop();
        return connectStream(redirectUrl, depth + 1);
    }

    return true;
}

// ============================================================================
// recTask() — FreeRTOS task na Core 0
// Czyta TCP blokowo i wrzuca dane do StreamBuffer (thread-safe SPSC).
// NIE dotyka SD card — wszystkie zapisy na SD sa w loop() na Core 1.
// Eliminuje konflikty SPI z SDPlayer/WebUI ktore powodowaly utrate danych.
// ============================================================================
void SDRecorder::recTask(void* param) {
    SDRecorder* self = static_cast<SDRecorder*>(param);
    WiFiClient* c = self->_useSSL ? (WiFiClient*)&self->_recSecure
                                  : (WiFiClient*)&self->_recClient;

    // WAZNE: recTask uzywa SWOJEGO bufora TCP — NIE self->_buffer!
    // self->_buffer jest uzywany przez loop() i stopRecording() na Core 1.
    // TCP buf = 4096 bajtow w PSRAM — WiFi nie wymaga DMA-capable RAM dla dst
    // BUFFER_SIZE = 512 (SD sektor), wiec TCP buf alokujemy z jawna wielokoscia 4096
    static const size_t TCP_BUF_SIZE = 4096;
    uint8_t* tcpBuf = (uint8_t*)heap_caps_malloc(TCP_BUF_SIZE,
                          MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!tcpBuf) {
        tcpBuf = (uint8_t*)malloc(TCP_BUF_SIZE);  // fallback: internal RAM
    }
    if (!tcpBuf) {
        Serial.println("[SDRec] ERROR: Brak pamieci na bufor TCP — task przerwany!");
        self->_recTaskHandle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    c->setTimeout(2000); // blokujacy read, max 2s na porcje danych

    Serial.printf("[SDRec] Task Core %d, SSL=%d\n", xPortGetCoreID(), self->_useSSL);

    unsigned long lastLogMs = millis();

    // Dechunking: allokuj bufor wyjsciowy tylko jesli strumien jest chunked
    // Format: <hex_size>\r\n<data>\r\n ... 0\r\n\r\n
    uint8_t* outBuf = nullptr;
    size_t   outBufPos = 0;
    bool     chunkedDone = false;
    // Stan maszynki dechunkujacej
    // 0=CS_SIZE (czytamy rozmiar hex), 1=CS_DATA (czytamy dane), 2=CS_CRLF (pomijamy \r\n za danymi)
    int      chunkState = 0;
    size_t   chunkRemaining = 0;
    String   chunkSizeLine;

    if (self->_isChunked) {
        outBuf = (uint8_t*)heap_caps_malloc(TCP_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!outBuf) outBuf = (uint8_t*)malloc(TCP_BUF_SIZE);
        if (!outBuf) {
            Serial.println("[SDRec] WARN: Brak pamieci na bufor dechunk — wylaczyc chunked");
            self->_isChunked = false;  // fallback: pisz raw (plik bedzie czesciowo uszkodzony)
        }
    }

    while (!self->_stopRequested && !chunkedDone) {
        int n = c->read(tcpBuf, TCP_BUF_SIZE);

        if (n > 0) {
            self->_lastRecordRead = millis();

            if (self->_isChunked && outBuf) {
                // Dechunking: przetworz kazdy bajt przez maszynke stanow
                for (int i = 0; i < n && !chunkedDone; i++) {
                    uint8_t ch = tcpBuf[i];
                    switch (chunkState) {
                        case 0: // CS_SIZE: akumuluj linie hex az do \n
                            if (ch == '\n') {
                                chunkSizeLine.trim();
                                // Usun ewentualny chunk-extension (po sredniiku)
                                int semi = chunkSizeLine.indexOf(';');
                                if (semi >= 0) chunkSizeLine = chunkSizeLine.substring(0, semi);
                                chunkRemaining = (size_t)strtoul(chunkSizeLine.c_str(), nullptr, 16);
                                chunkSizeLine = "";
                                if (chunkRemaining == 0) {
                                    chunkedDone = true;  // terminal chunk = koniec
                                } else {
                                    chunkState = 1;  // CS_DATA
                                }
                            } else if (ch != '\r') {
                                chunkSizeLine += (char)ch;
                            }
                            break;
                        case 1: // CS_DATA: kopiuj bajty danych
                            outBuf[outBufPos++] = ch;
                            chunkRemaining--;
                            if (chunkRemaining == 0) chunkState = 2;  // CS_CRLF
                            // Flush gdy bufor pelny
                            if (outBufPos == TCP_BUF_SIZE) {
                                xStreamBufferSend(self->_streamBuf, outBuf, outBufPos, pdMS_TO_TICKS(200));
                                outBufPos = 0;
                            }
                            break;
                        case 2: // CS_CRLF: pomin \r\n po danych chunka
                            if (ch == '\n') chunkState = 0;  // CS_SIZE
                            break;
                    }
                }
                // Flush pozostalych danych z bufor wyjsciowego
                if (outBufPos > 0) {
                    xStreamBufferSend(self->_streamBuf, outBuf, outBufPos, pdMS_TO_TICKS(200));
                    outBufPos = 0;
                }
            } else {
                // Strumien prosty (bez chunked): wrzuc bezposrednio do StreamBuffer
                xStreamBufferSend(self->_streamBuf, tcpBuf, (size_t)n, pdMS_TO_TICKS(200));
            }

            if (millis() - lastLogMs >= 10000) {
                lastLogMs = millis();
                Serial.printf("[SDRec] Bufor strumienia: ~%u KB w kolejce\n",
                              xStreamBufferBytesAvailable(self->_streamBuf) / 1024);
            }
        } else if (n < 0) {
            Serial.println("[SDRec] TCP rozlaczony -> koniec");
            break;
        }

        if (millis() - self->_lastRecordRead > 120000) {
            Serial.println("[SDRec] Timeout 120s -> koniec");
            break;
        }
    }

    if (outBuf) free(outBuf);
    free(tcpBuf);

    // Task konczy sie tutaj — NIE zamykamy pliku, NIE stopujemy klienta.
    // loop() wykrywa ze _recTaskHandle == nullptr i bufor pusty -> sam zamknie plik.
    // stopRecording() tez to obsluguje (drainuje bufor i zamyka).
    Serial.println("[SDRec] Task zakonczony, loop() zapisze pozostale dane.");
    self->_recTaskHandle = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================================
// HTTP helpers
// ============================================================================

// Czyta jedna linie tekstu zakonczana \r\n z klienta TCP
String SDRecorder::readHttpLine(WiFiClient& c) {
    String line = "";
    unsigned long timeout = millis() + 5000;
    while (millis() < timeout) {
        while (c.available()) {
            int ch = c.read();
            if (ch < 0) return line;
            if (ch == '\n') return line;
            if (ch != '\r') line += (char)ch;
        }
        delay(1);
    }
    return line;
}

// Parsuje rozmiar chunku (hex) z naglowka Transfer-Encoding: chunked
int SDRecorder::readChunkSize(WiFiClient& c) {
    String line = readHttpLine(c);
    return (int)strtol(line.c_str(), nullptr, 16);
}

// Czyta naglowki HTTP — zwraca 1 (redirect), 0 (OK), -1 (blad)
// Wykrywa Content-Type aby ustawic poprawne rozszerzenie pliku
int SDRecorder::readHttpHeaders(WiFiClient& c, String& redirectUrl) {
    unsigned long timeout = millis() + 10000;
    while (c.connected() && millis() < timeout) {
        String line = readHttpLine(c);
        if (line.length() == 0) break;  // pusta linia = koniec naglowkow
        Serial.println("[HDR] " + line);
        if (line.startsWith("Location:")) {
            redirectUrl = line.substring(10);
            redirectUrl.trim();
            return 1;  // redirect
        }
        if (line.startsWith("Transfer-Encoding:") && line.indexOf("chunked") >= 0) {
            _isChunked = true;
            Serial.println("[SDRec] Transfer-Encoding: chunked — dechunking aktywny");
        }
        // Wykryj format audio z Content-Type
        if (line.startsWith("Content-Type:") || line.startsWith("content-type:")) {
            String ct = line;
            ct.toLowerCase();
            if      (ct.indexOf("flac")  >= 0)   _fileExt = ".flac";
            else if (ct.indexOf("ogg")   >= 0)   _fileExt = ".ogg";
            else if (ct.indexOf("aac")   >= 0)   _fileExt = ".aac";
            else if (ct.indexOf("aacp")  >= 0)   _fileExt = ".aac";
            else if (ct.indexOf("opus")  >= 0)   _fileExt = ".opus";
            else if (ct.indexOf("mpeg")  >= 0)   _fileExt = ".mp3";
            else if (ct.indexOf("mp3")   >= 0)   _fileExt = ".mp3";
            Serial.printf("[SDRec] Content-Type -> rozszerzenie: %s\n", _fileExt.c_str());
        }
    }
    return 0;  // OK
}

// ============================================================================
// Generowanie nazwy pliku
// ============================================================================
String SDRecorder::generateFileName(const String& stationName) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &timeinfo);

    // Oczyszczenie nazwy stacji z niedozwolonych znakow FAT32/SD
    String name = stationName;
    name.replace("/",  "_");
    name.replace("\\", "_");
    name.replace(":",  "_");
    name.replace("*",  "_");
    name.replace("?",  "_");
    name.replace("\"", "_");
    name.replace("<",  "_");
    name.replace(">",  "_");
    name.replace("|",  "_");
    name.trim();
    if (name.length() > 20) name = name.substring(0, 20);

    String filename = _recordPath + "/REC_" + String(timestamp);
    if (name.length() > 0) filename += "_" + name;
    filename += _fileExt;  // .mp3 / .flac / .aac / .ogg / .opus

    return filename;
}

void SDRecorder::ensureRecordDirectory() {
    if (!SD.exists(_recordPath.c_str())) {
        SD.mkdir(_recordPath.c_str());
        Serial.printf("[SDRecorder] Utworzono folder: %s\n", _recordPath.c_str());
    }
}

void SDRecorder::setMaxFileSize(size_t maxMB) {
    _maxFileSize = maxMB * 1024 * 1024;
}

void SDRecorder::setRecordPath(const String& path) {
    _recordPath = path;
    ensureRecordDirectory();
}

// ============================================================================
// Gettery czasu i rozmiaru
// ============================================================================
unsigned long SDRecorder::getRecordTime() const {
    if (_state == IDLE) return 0;
    return (millis() - _recordStartTime) / 1000;
}

String SDRecorder::getRecordTimeString() const {
    unsigned long total = getRecordTime();
    unsigned long h = total / 3600;
    unsigned long m = (total % 3600) / 60;
    unsigned long s = total % 60;
    char buf[16];
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, s);
    return String(buf);
}

String SDRecorder::getFileSizeString() const {
    return formatFileSize(_fileSize);
}

String SDRecorder::formatFileSize(size_t bytes) const {
    if (bytes < 1024)              return String(bytes) + " B";
    if (bytes < 1024 * 1024)       return String(bytes / 1024.0f, 1) + " KB";
    return String(bytes / (1024.0f * 1024.0f), 1) + " MB";
}
