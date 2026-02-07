#include "BTWebUI.h"

BTWebUI::BTWebUI() 
    : _server(nullptr),
      _btSerial(nullptr),
      _exitCallback(nullptr),
      _btOn(false),
      _mode("OFF"),
      _volume(15),
      _boost(100),
      _connected(false),
      _deviceName("None"),
      _deviceMac(""),
      _consoleLog(""),
      _uartBuffer(""),
      _lastResponse(""),
      _lastCmdTime(0),
      _lastStatusPoll(0),
      _garbageLines(0),
      _terminalEnabled(false) {
    // Zarezerwuj pamięć dla logu aby uniknąć realokacji
    _consoleLog.reserve(MAX_LOG_SIZE);
}

void BTWebUI::begin(AsyncWebServer* server, int rxPin, int txPin, uint32_t baud) {
    _server = server;
    
    Serial.println("=====================================");
    Serial.println("BTWebUI: Initializing BT interface...");
    Serial.println("=====================================");
    
    // Inicjalizacja UART dla komunikacji z modułem BT
    _btSerial = new HardwareSerial(2);
    _btSerial->begin(baud, SERIAL_8N1, rxPin, txPin);
    
    Serial.println("[BT UART] Port: HardwareSerial(2)");
    Serial.println("[BT UART] RX Pin: " + String(rxPin));
    Serial.println("[BT UART] TX Pin: " + String(txPin));
    Serial.println("[BT UART] Baud: " + String(baud));
    Serial.println("[BT UART] Initialized successfully!");
    Serial.println("BTWebUI: Registering routes...");
    
    // Główna strona BT
    _server->on("/bt", HTTP_GET, [this](AsyncWebServerRequest *request){
        Serial.println("BTWebUI: /bt page requested");
        this->handleRoot(request);
    });
    
    // API endpoints
    _server->on("/bt/api/state", HTTP_GET, [this](AsyncWebServerRequest *request){
        Serial.println("BTWebUI: /bt/api/state requested");
        this->handleState(request);
    });
    
    _server->on("/bt/api/log", HTTP_GET, [this](AsyncWebServerRequest *request){
        Serial.println("BTWebUI: /bt/api/log requested");
        this->handleLog(request);
    });
    
    _server->on("/bt/api/mode", HTTP_POST, [this](AsyncWebServerRequest *request){
        Serial.println("BTWebUI: /bt/api/mode requested");
        this->handleMode(request);
    });
    
    _server->on("/bt/api/vol", HTTP_POST, [this](AsyncWebServerRequest *request){
        Serial.println("BTWebUI: /bt/api/vol requested");
        this->handleVol(request);
    });
    
    _server->on("/bt/api/boost", HTTP_POST, [this](AsyncWebServerRequest *request){
        Serial.println("BTWebUI: /bt/api/boost requested");
        this->handleBoost(request);
    });
    
    _server->on("/bt/api/scan", HTTP_POST, [this](AsyncWebServerRequest *request){
        Serial.println("BTWebUI: /bt/api/scan requested");
        this->handleScan(request);
    });
    
    _server->on("/bt/api/disconnect", HTTP_POST, [this](AsyncWebServerRequest *request){
        Serial.println("BTWebUI: /bt/api/disconnect requested");
        this->handleDisconnect(request);
    });
    
    _server->on("/bt/api/delall", HTTP_POST, [this](AsyncWebServerRequest *request){
        Serial.println("BTWebUI: /bt/api/delall requested");
        this->handleDelAll(request);
    });
    
    _server->on("/bt/api/save", HTTP_POST, [this](AsyncWebServerRequest *request){
        Serial.println("BTWebUI: /bt/api/save requested");
        this->handleSave(request);
    });
    
    _server->on("/bt/api/cmd", HTTP_POST, [this](AsyncWebServerRequest *request){
        Serial.println("BTWebUI: /bt/api/cmd requested");
        this->handleCmd(request);
    });
    
    _server->on("/bt/api/back", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleBack(request);
    });
    
    _server->on("/bt/api/terminal", HTTP_POST, [this](AsyncWebServerRequest *request){
        Serial.println("BTWebUI: /bt/api/terminal requested");
        this->handleTerminalEnable(request);
    });
    
    // Wyślij komendę STATUS? aby pobrać bieżący stan
    delay(100);
    sendCommand("STATUS?");
}

void BTWebUI::setExitCallback(std::function<void()> callback) {
    _exitCallback = callback;
}

void BTWebUI::loop() {
    // ZAWSZE odbieraj dane z UART (niezależnie od stanu terminala)
    while (_btSerial && _btSerial->available()) {
        char c = _btSerial->read();
        
        // Filtruj nieprawidłowe znaki (tylko drukowane ASCII + \r\n)
        if ((c >= 32 && c <= 126) || c == '\n' || c == '\r') {
            if (c == '\n') {
                if (_uartBuffer.length() > 0) {
                    processUartLine(_uartBuffer);
                    _uartBuffer = "";
                }
            } else if (c != '\r') {
                _uartBuffer += c;
                if (_uartBuffer.length() > 500) {
                    _uartBuffer = ""; // Zabezpieczenie przed przepełnieniem
                }
            }
        }
    }
    
    // Regularnie odpytuj o status (zawsze, niezależnie od stanu terminala)
    unsigned long now = millis();
    if (now - _lastStatusPoll > 3000) {
        _lastStatusPoll = now;
        if (_btSerial) {
            _btSerial->println("STATUS?");
            Serial.println("[BT] Wysłano STATUS? (auto-poll)");
        }
    }
}

void BTWebUI::sendCommand(const String& cmd) {
    if (!_btSerial) {
        Serial.println("BT CMD ERROR: Serial not initialized!");
        if (_terminalEnabled) {
            addToLog("[ERROR] UART nie zainicjalizowany!");
        }
        return;
    }
    
    Serial.println("BT CMD SENT: " + cmd);
    _btSerial->println(cmd);
    
    // Dodaj do logu tylko jeśli terminal aktywny
    if (_terminalEnabled) {
        addToLog("> " + cmd);
        
        // Dodaj informację o oczekiwaniu na odpowiedź
        if (cmd == "STATUS?" || cmd == "PING" || cmd == "GET") {
            addToLog("Oczekiwanie na odpowiedź...");
        }
    }
    
    _lastCmdTime = millis();
}

void BTWebUI::addToLog(const String& line) {
    // Dodaj do logu tylko gdy terminal aktywny
    if (!_terminalEnabled) return;
    
    _consoleLog += line + "\n";
    
    // Ogranicz rozmiar logu
    if (_consoleLog.length() > MAX_LOG_SIZE) {
        _consoleLog = _consoleLog.substring(_consoleLog.length() - MAX_LOG_SIZE);
        int firstNewline = _consoleLog.indexOf('\n');
        if (firstNewline >= 0) {
            _consoleLog = _consoleLog.substring(firstNewline + 1);
        }
    }
}

void BTWebUI::processUartLine(const String& line) {
    // Ignoruj puste linie
    if (line.length() == 0) {
        return;
    }
    
    // Sprawdź czy to HTML (zawiera tagi <html>, <body>, <!doctype itp.)
    bool isHtml = (line.indexOf("<html") >= 0 || line.indexOf("<!doctype") >= 0 || 
                   line.indexOf("<body") >= 0 || line.indexOf("<head") >= 0);
    
    // Odrzuć tylko oczywiste śmieci: HTML lub bardzo długie linie (>300 znaków)
    if (isHtml || line.length() > 300) {
        _garbageLines++;
        Serial.println("[BT UART] Odrzucono śmieci #" + String(_garbageLines) + ": " + line.substring(0, min((int)line.length(), 50)) + "...");
        return;
    }
    
    // Loguj każdą poprawną linię do Serial Monitor
    Serial.println("[BT UART RX] " + line);
    
    addToLog("< " + line);
    _lastResponse = line;
    
    // Parsuj odpowiedzi
    if (line.startsWith("STATE ")) {
        Serial.println("[BT] Parsing STATE response...");
        parseStatusResponse(line);
    } else if (line.startsWith("OK ")) {
        // Pomyślna odpowiedź
        if (line.indexOf("MODE") >= 0) {
            // Wyciągnij tryb
            if (line.indexOf("OFF") >= 0) _mode = "OFF";
            else if (line.indexOf("TX") >= 0) _mode = "TX";
            else if (line.indexOf("RX") >= 0) _mode = "RX";
            else if (line.indexOf("AUTO") >= 0) _mode = "AUTO";
        } else if (line.indexOf("VOL") >= 0) {
            // Wyciągnij volume
            int idx = line.indexOf("VOL");
            String volStr = line.substring(idx + 4);
            volStr.trim();
            _volume = volStr.toInt();
        } else if (line.indexOf("BOOST") >= 0) {
            // Wyciągnij boost
            int idx = line.indexOf("BOOST");
            String boostStr = line.substring(idx + 6);
            boostStr.trim();
            _boost = boostStr.toInt();
        } else if (line.indexOf("BT ON") >= 0) {
            _btOn = true;
        }
    } else if (line.indexOf("PONG") >= 0) {
        _btOn = true; // Moduł odpowiada
    }
}

void BTWebUI::parseStatusResponse(const String& line) {
    // Format: STATE BT=ON MODE=TX VOL=100 BOOST=400 SCAN=0 CONN=1 MAC=AA:BB:CC:DD:EE:FF NAME="Device"
    Serial.println("[BT] parseStatusResponse: " + line);
    
    if (line.indexOf("BT=ON") >= 0) {
        _btOn = true;
        Serial.println("[BT] Status: BT ON");
    } else if (line.indexOf("BT=OFF") >= 0) {
        _btOn = false;
        Serial.println("[BT] Status: BT OFF");
    }
    
    if (line.indexOf("MODE=OFF") >= 0) {
        _mode = "OFF";
        Serial.println("[BT] Mode: OFF");
    } else if (line.indexOf("MODE=TX") >= 0) {
        _mode = "TX";
        Serial.println("[BT] Mode: TX");
    } else if (line.indexOf("MODE=RX") >= 0) {
        _mode = "RX";
        Serial.println("[BT] Mode: RX");
    } else if (line.indexOf("MODE=AUTO") >= 0) {
        _mode = "AUTO";
        Serial.println("[BT] Mode: AUTO");
    }
    
    // VOL
    int volIdx = line.indexOf("VOL=");
    if (volIdx >= 0) {
        String volStr = line.substring(volIdx + 4);
        int spaceIdx = volStr.indexOf(' ');
        if (spaceIdx > 0) volStr = volStr.substring(0, spaceIdx);
        _volume = volStr.toInt();
        Serial.println("[BT] Volume: " + String(_volume));
    }
    
    // BOOST
    int boostIdx = line.indexOf("BOOST=");
    if (boostIdx >= 0) {
        String boostStr = line.substring(boostIdx + 6);
        int spaceIdx = boostStr.indexOf(' ');
        if (spaceIdx > 0) boostStr = boostStr.substring(0, spaceIdx);
        _boost = boostStr.toInt();
        Serial.println("[BT] Boost: " + String(_boost));
    }
    
    // CONN
    if (line.indexOf("CONN=1") >= 0) {
        _connected = true;
        Serial.println("[BT] Connected: YES");
    } else if (line.indexOf("CONN=0") >= 0) {
        _connected = false;
        Serial.println("[BT] Connected: NO");
    }
    
    // MAC
    int macIdx = line.indexOf("MAC=");
    if (macIdx >= 0) {
        String macStr = line.substring(macIdx + 4);
        int spaceIdx = macStr.indexOf(' ');
        if (spaceIdx > 0) {
            macStr = macStr.substring(0, spaceIdx);
            _deviceMac = macStr;
            Serial.println("[BT] MAC: " + _deviceMac);
        }
    }
    
    // NAME
    int nameIdx = line.indexOf("NAME=\"");
    if (nameIdx >= 0) {
        String nameStr = line.substring(nameIdx + 6);
        int endIdx = nameStr.indexOf('"');
        if (endIdx > 0) {
            _deviceName = nameStr.substring(0, endIdx);
            Serial.println("[BT] Name: " + _deviceName);
        }
    }
    
    // Podsumowanie sparsowanego stanu
    Serial.println("[BT] === Parsed state summary ===");
    Serial.println("[BT] BT: " + String(_btOn ? "ON" : "OFF"));
    Serial.println("[BT] Mode: " + _mode);
    Serial.println("[BT] Vol: " + String(_volume));
    Serial.println("[BT] Boost: " + String(_boost));
    Serial.println("[BT] Conn: " + String(_connected ? "YES" : "NO"));
    Serial.println("[BT] Device: " + _deviceName + " (" + _deviceMac + ")");
    Serial.println("[BT] ==============================");
}

void BTWebUI::handleRoot(AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", BTUART_HTML);
}

void BTWebUI::handleState(AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(512);
    
    doc["btOn"] = _btOn;
    doc["mode"] = _mode;
    doc["vol"] = _volume;
    doc["boost"] = _boost;
    doc["conn"] = _connected;
    doc["name"] = _deviceName;
    doc["mac"] = _deviceMac;
    doc["terminalEnabled"] = _terminalEnabled;
    
    String response;
    serializeJson(doc, response);
    
    // Dodaj nagłówki zapobiegające cache'owaniu
    AsyncWebServerResponse *resp = request->beginResponse(200, "application/json", response);
    resp->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    request->send(resp);
}

void BTWebUI::handleLog(AsyncWebServerRequest *request) {
    // Debug: Sprawdź długość logu
    Serial.println("[BT LOG] ===== handleLog() CALLED =====");
    Serial.println("[BT LOG] Request URI: " + request->url());
    Serial.println("[BT LOG] Log size: " + String(_consoleLog.length()) + " bytes");
    Serial.println("[BT LOG] Terminal enabled: " + String(_terminalEnabled ? "YES" : "NO"));
    
    // Jeśli log pusty, wyślij placeholder
    String logContent = _consoleLog;
    if (logContent.length() == 0) {
        logContent = "[INFO] Terminal log is empty. Enable terminal to see UART communication.\n";
        Serial.println("[BT LOG] Log is EMPTY - sending placeholder");
    }
    
    Serial.println("[BT LOG] Sending response with Content-Type: text/plain");
    Serial.println("[BT LOG] First 100 chars: " + logContent.substring(0, min(100, (int)logContent.length())));
    
    // WAŻNE: Użyj request->send() bezpośrednio z text/plain
    request->send(200, "text/plain; charset=utf-8", logContent);
    
    Serial.println("[BT LOG] Response sent successfully");
}

void BTWebUI::handleMode(AsyncWebServerRequest *request) {
    Serial.println("BTWebUI: handleMode called");
    if (request->hasParam("m")) {
        String mode = request->getParam("m")->value();
        mode.toUpperCase();
        Serial.println("BT Mode change to: " + mode);
        
        if (mode == "OFF" || mode == "TX" || mode == "RX" || mode == "AUTO") {
            sendCommand("MODE " + mode);
        }
    }
    request->send(200, "text/plain", "OK");
}

void BTWebUI::handleVol(AsyncWebServerRequest *request) {
    Serial.println("BTWebUI: handleVol called");
    if (request->hasParam("v")) {
        int vol = request->getParam("v")->value().toInt();
        if (vol < 0) vol = 0;
        if (vol > 100) vol = 100;
        Serial.printf("BT Volume set to: %d\n", vol);
        
        sendCommand("VOL " + String(vol));
    }
    request->send(200, "text/plain", "OK");
}

void BTWebUI::handleBoost(AsyncWebServerRequest *request) {
    Serial.println("BTWebUI: handleBoost called");
    if (request->hasParam("b")) {
        int boost = request->getParam("b")->value().toInt();
        if (boost < 100) boost = 100;
        if (boost > 400) boost = 400;
        Serial.printf("BT Boost set to: %d\n", boost);
        
        sendCommand("BOOST " + String(boost));
    }
    request->send(200, "text/plain", "OK");
}

void BTWebUI::handleScan(AsyncWebServerRequest *request) {
    Serial.println("BTWebUI: handleScan - Scanning BT devices...");
    sendCommand("SCAN");
    request->send(200, "text/plain", "OK");
}

void BTWebUI::handleDisconnect(AsyncWebServerRequest *request) {
    Serial.println("BTWebUI: handleDisconnect - Disconnecting BT...");
    sendCommand("DISCONNECT");
    request->send(200, "text/plain", "OK");
}

void BTWebUI::handleDelAll(AsyncWebServerRequest *request) {
    Serial.println("BTWebUI: handleDelAll - Deleting all paired devices...");
    sendCommand("DELPAIRED ALL");
    request->send(200, "text/plain", "OK");
}

void BTWebUI::handleSave(AsyncWebServerRequest *request) {
    Serial.println("BTWebUI: handleSave - Saving BT settings...");
    sendCommand("SAVE");
    request->send(200, "text/plain", "OK");
}

void BTWebUI::handleCmd(AsyncWebServerRequest *request) {
    Serial.println("BTWebUI: handleCmd called");
    if (request->hasParam("cmd")) {
        String cmd = request->getParam("cmd")->value();
        cmd.trim();
        Serial.println("BT Command: " + cmd);
        
        if (cmd.length() > 0) {
            sendCommand(cmd);
        }
    }
    request->send(200, "text/plain", "OK");
}

void BTWebUI::handleBack(AsyncWebServerRequest *request) {
    // Wyłącz terminal przy wyjściu
    _terminalEnabled = false;
    _consoleLog = "";
    
    if (_exitCallback) {
        _exitCallback();
    }
    request->send(200, "text/plain", "OK");
}

void BTWebUI::handleTerminalEnable(AsyncWebServerRequest *request) {
    if (request->hasParam("enable")) {
        int enable = request->getParam("enable")->value().toInt();
        _terminalEnabled = (enable != 0);
        
        if (_terminalEnabled) {
            // Wyczyść bufor UART z ewentualnych śmieci
            int cleared = 0;
            while (_btSerial && _btSerial->available()) {
                _btSerial->read();
                cleared++;
            }
            _uartBuffer.clear();
            _uartBuffer = "";
            
            // WYCZYŚĆ LOG - użyj clear() i reserve() aby uniknąć problemów z pamięcią
            _consoleLog.clear();
            _consoleLog = "";
            _consoleLog.reserve(MAX_LOG_SIZE);
            
            // Resetuj licznik śmieci
            int prevGarbage = _garbageLines;
            _garbageLines = 0;
            
            // Dodaj nagłówek
            _consoleLog = "=== Terminal BT aktywny ===\n";
            _consoleLog += "Połączenie UART: RX=19, TX=20, 115200 baud\n";
            
            if (cleared > 0) {
                _consoleLog += "[INFO] Wyczyszczono " + String(cleared) + " bajtów z bufora UART\n";
            }
            
            if (prevGarbage > 0) {
                _consoleLog += "[UWAGA] Odrzucono " + String(prevGarbage) + " linii śmieci przed włączeniem\n";
                _consoleLog += "        (szumy z niezainicjalizowanego UART - moduł BT nie podłączony?)\n";
            }
            
            _consoleLog += "\nDostępne komendy:\n";
            _consoleLog += "  STATUS? - pobierz status modułu BT\n";
            _consoleLog += "  PING    - sprawdź połączenie z modułem\n";
            _consoleLog += "  HELP    - wyświetl listę komend\n";
            _consoleLog += "  GET     - odczytaj bieżące ustawienia\n\n";
            
            // Wyślij STATUS? aby pobrać stan
            if (_btSerial) {
                delay(50);  // Krótkie opóźnienie przed wysłaniem komendy
                _btSerial->println("STATUS?");
                _consoleLog += "> STATUS?\n";
                _consoleLog += "Oczekiwanie na odpowiedź (3 sek)...\n";
                _consoleLog += "\n[TIP] Jeśli brak odpowiedzi:\n";
                _consoleLog += "      1. Sprawdź podłączenie modułu BT (RX19/TX20)\n";
                _consoleLog += "      2. Sprawdź zasilanie modułu BT\n";
                _consoleLog += "      3. Sprawdź prędkość UART (115200 baud)\n";
            } else {
                _consoleLog += "\n[ERROR] UART nie zainicjalizowany!\n";
                _consoleLog += "        Skontaktuj się z deweloperem.\n";
            }
            
            Serial.println("BT Terminal: ENABLED");
            if (prevGarbage > 0) {
                Serial.println("[BT] Odrzucono śmieci przed włączeniem: " + String(prevGarbage) + " linii");
            }
        } else {
            // Przy wyłączaniu terminala - wyczyść log
            _consoleLog.clear();
            _consoleLog = "";
            Serial.println("BT Terminal: DISABLED");
        }
    }
    request->send(200, "text/plain", "OK");
}
