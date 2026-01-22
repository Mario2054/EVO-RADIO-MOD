// =====================================================================================
// BT_KCX_Module.cpp - Moduł obsługi KCX_BT_EMITTER dla EVO Radio
// =====================================================================================
// Obsługa Bluetooth RX/TX/AUTO bez dodatkowych tasków
// Logika oparta na prostych if-ach w loop()
// =====================================================================================

#include "BT_KCX_Module.h"
#include <FS.h>

// =====================================================================================
// ZEWNĘTRZNE DEKLARACJE
// =====================================================================================
extern fs::FS& getStorage();  // Dostęp do systemu plików (SD/SPIFFS)

// =====================================================================================
// OBIEKT KCX_BT_EMITTER
// =====================================================================================
KCX_BT_Emitter bt_emitter(BT_EMITTER_RX_PIN, BT_EMITTER_TX_PIN, 
                          BT_EMITTER_LINK_PIN, BT_EMITTER_MODE_PIN);

// =====================================================================================
// STAN WEWNĘTRZNY MODUŁU
// =====================================================================================
static BTState btState = {
    .mode = BT_MODE_OFF,
    .status = BT_STATUS_OFF,
    .enabled = false,
    .connected = false,
    .menuActive = false,
    .menuSelection = 0,
    .deviceName = "",
    .deviceAddr = "",
    .volume = 15
};

static bool btInitialized = false;
static unsigned long lastBTCheck = 0;
static const unsigned long BT_CHECK_INTERVAL = 100;  // 100ms

// Auto mode zmienne
static unsigned long autoModeLastSwitch = 0;
static const unsigned long AUTO_MODE_SWITCH_DELAY = 5000;  // 5s
static bool autoModeRxActive = true;

// =====================================================================================
// CALLBACK FUNKCJE DLA KCX_BT_EMITTER
// =====================================================================================
void kcx_bt_info(const char* info, const char* val) {
    Serial.printf("BT INFO: %s %s\n", info, val);
    
    if (strcmp(info, "KCX_BT_Emitter found") == 0) {
        btInitialized = true;
        Serial.println("BT: KCX_BT_EMITTER wykryty i gotowy");
    }
    else if (strcmp(info, "KCX_BT_Emitter not found") == 0) {
        btInitialized = false;
        Serial.println("BT: KCX_BT_EMITTER NIE WYKRYTY!");
    }
    else if (strcmp(info, "my name:") == 0) {
        btState.deviceName = String(val);
    }
}

void kcx_bt_status(bool status) {
    btState.connected = status;
    if (status) {
        btState.status = BT_STATUS_CONNECTED;
        Serial.println("BT: Połączono z urządzeniem");
    } else {
        btState.status = BT_STATUS_DISCONNECTED;
        Serial.println("BT: Rozłączono");
    }
}

void kcx_bt_memItems(const char* jsonItems) {
    Serial.printf("BT MEM: %s\n", jsonItems);
}

void kcx_bt_scanItems(const char* jsonItems) {
    Serial.printf("BT SCAN: %s\n", jsonItems);
}

void kcx_bt_modeChanged(const char* m) {
    Serial.printf("BT MODE CHANGED: %s\n", m);
    if (strcmp(m, "RX") == 0) {
        btState.mode = BT_MODE_RX;
    } else if (strcmp(m, "TX") == 0) {
        btState.mode = BT_MODE_TX;
    }
}

// =====================================================================================
// INICJALIZACJA
// =====================================================================================
void BT_init(void) {
    Serial.println("BT: Inicjalizacja modułu KCX_BT_EMITTER...");
    
    // Załaduj konfigurację
    BT_loadConfig();
    
    // Inicjalizuj sprzęt
    bt_emitter.begin();
    
    // Poczekaj na wykrycie modułu
    delay(100);
    
    // Pobierz informacje o module
    bt_emitter.userCommand("AT+GMR?");      // Wersja firmware
    bt_emitter.userCommand("AT+BT_MODE?");  // Aktualny tryb
    bt_emitter.userCommand("AT+VOL?");      // Głośność
    
    // Ustaw zapisany tryb
    if (btState.enabled && btState.mode != BT_MODE_OFF) {
        BT_setMode(btState.mode);
    }
    
    Serial.println("BT: Inicjalizacja zakończona");
}

// =====================================================================================
// PĘTLA GŁÓWNA - WYWOŁYWAĆ W loop()
// =====================================================================================
void BT_loop(void) {
    // Obsługa komunikacji z modułem KCX
    bt_emitter.loop();
    
    // Sprawdzanie statusu co BT_CHECK_INTERVAL ms
    if (millis() - lastBTCheck >= BT_CHECK_INTERVAL) {
        lastBTCheck = millis();
        
        // Aktualizuj status połączenia
        if (btState.enabled && btState.mode != BT_MODE_OFF) {
            btState.connected = bt_emitter.isConnected();
            
            if (btState.connected) {
                btState.status = BT_STATUS_CONNECTED;
            } else if (btState.status != BT_STATUS_SCANNING) {
                btState.status = BT_STATUS_DISCONNECTED;
            }
        }
        
        // Obsługa trybu AUTO
        if (btState.mode == BT_MODE_AUTO && btState.enabled) {
            // W trybie AUTO przełączamy między RX i TX co AUTO_MODE_SWITCH_DELAY
            // gdy nie ma połączenia
            if (!btState.connected) {
                if (millis() - autoModeLastSwitch >= AUTO_MODE_SWITCH_DELAY) {
                    autoModeLastSwitch = millis();
                    autoModeRxActive = !autoModeRxActive;
                    
                    if (autoModeRxActive) {
                        bt_emitter.setMode("RX");
                        Serial.println("BT AUTO: Przełączono na RX");
                    } else {
                        bt_emitter.setMode("TX");
                        Serial.println("BT AUTO: Przełączono na TX");
                    }
                }
            }
        }
    }
}

// =====================================================================================
// STEROWANIE TRYBEM
// =====================================================================================
void BT_setMode(BTMode mode) {
    btState.mode = mode;
    
    switch (mode) {
        case BT_MODE_OFF:
            btState.enabled = false;
            btState.status = BT_STATUS_OFF;
            btState.connected = false;
            bt_emitter.cmd_PowerOff();
            Serial.println("BT: Tryb OFF");
            break;
            
        case BT_MODE_RX:
            btState.enabled = true;
            btState.status = BT_STATUS_DISCONNECTED;
            bt_emitter.setMode("RX");
            Serial.println("BT: Tryb RX (odbiornik)");
            break;
            
        case BT_MODE_TX:
            btState.enabled = true;
            btState.status = BT_STATUS_DISCONNECTED;
            bt_emitter.setMode("TX");
            Serial.println("BT: Tryb TX (nadajnik)");
            break;
            
        case BT_MODE_AUTO:
            btState.enabled = true;
            btState.status = BT_STATUS_DISCONNECTED;
            autoModeLastSwitch = millis();
            autoModeRxActive = true;
            bt_emitter.setMode("RX");  // Startuj od RX
            Serial.println("BT: Tryb AUTO (RX/TX)");
            break;
    }
    
    BT_saveConfig();
}

BTMode BT_getMode(void) {
    return btState.mode;
}

void BT_toggleMode(void) {
    BTMode newMode;
    
    switch (btState.mode) {
        case BT_MODE_OFF:
            newMode = BT_MODE_RX;
            break;
        case BT_MODE_RX:
            newMode = BT_MODE_TX;
            break;
        case BT_MODE_TX:
            newMode = BT_MODE_AUTO;
            break;
        case BT_MODE_AUTO:
        default:
            newMode = BT_MODE_OFF;
            break;
    }
    
    BT_setMode(newMode);
}

// =====================================================================================
// STEROWANIE WŁĄCZENIEM
// =====================================================================================
void BT_enable(bool enabled) {
    btState.enabled = enabled;
    
    if (!enabled) {
        btState.status = BT_STATUS_OFF;
        btState.connected = false;
        bt_emitter.cmd_PowerOff();
    } else if (btState.mode != BT_MODE_OFF) {
        BT_setMode(btState.mode);  // Reaktywuj aktualny tryb
    }
}

bool BT_isEnabled(void) {
    return btState.enabled;
}

// =====================================================================================
// STATUS POŁĄCZENIA
// =====================================================================================
BTStatus BT_getStatus(void) {
    return btState.status;
}

bool BT_isConnected(void) {
    return btState.connected;
}

String BT_getDeviceName(void) {
    return btState.deviceName;
}

// =====================================================================================
// MENU OBSŁUGA
// =====================================================================================
bool BT_isMenuActive(void) {
    return btState.menuActive;
}

void BT_setMenuActive(bool active) {
    btState.menuActive = active;
    if (active) {
        btState.menuSelection = (uint8_t)btState.mode;
    }
}

void BT_menuUp(void) {
    if (btState.menuSelection > 0) {
        btState.menuSelection--;
    } else {
        btState.menuSelection = 3;  // Wrap to AUTO
    }
}

void BT_menuDown(void) {
    if (btState.menuSelection < 3) {
        btState.menuSelection++;
    } else {
        btState.menuSelection = 0;  // Wrap to OFF
    }
}

void BT_menuSelect(void) {
    BT_setMode((BTMode)btState.menuSelection);
    btState.menuActive = false;
}

// =====================================================================================
// WYŚWIETLANIE - MENU
// =====================================================================================
void BT_displayMenu(U8G2& u8g2) {
    u8g2.clearBuffer();
    
    // Nagłówek
    u8g2.setFont(u8g2_font_helvB12_tr);
    u8g2.drawStr(70, 14, "BLUETOOTH");
    
    u8g2.drawLine(0, 18, 255, 18);
    
    // Opcje menu
    u8g2.setFont(u8g2_font_helvR10_tr);
    
    const char* options[] = {"OFF", "RX (Odbiornik)", "TX (Nadajnik)", "AUTO (RX/TX)"};
    const char* icons[] = {"   ", "\xB7 ", "\xB7 ", "\xB7 "};  // Punktory
    
    int yStart = 32;
    int yStep = 12;
    
    for (int i = 0; i < 4; i++) {
        int y = yStart + i * yStep;
        
        // Zaznaczenie wybranej opcji
        if (i == btState.menuSelection) {
            u8g2.drawRBox(20, y - 10, 216, 12, 2);
            u8g2.setDrawColor(0);
        }
        
        // Ikona aktualnego trybu
        if (i == (int)btState.mode && btState.enabled) {
            u8g2.drawStr(25, y, ">");
        }
        
        u8g2.drawStr(40, y, options[i]);
        
        // Status dla aktywnego trybu
        if (i == (int)btState.mode && btState.enabled && i != 0) {
            const char* statusStr = btState.connected ? " [CONNECTED]" : " [...]";
            u8g2.drawStr(160, y, statusStr);
        }
        
        u8g2.setDrawColor(1);
    }
    
    // Dolna linia z instrukcją
    u8g2.drawLine(0, 56, 255, 56);
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(5, 63, "Encoder: wybor | Click: zatwierdz | Back: wyjscie");
    
    u8g2.sendBuffer();
}

// =====================================================================================
// WYŚWIETLANIE - IKONA STATUSU (dla głównego ekranu)
// =====================================================================================
void BT_drawStatusIcon(U8G2& u8g2, int x, int y) {
    if (!btState.enabled || btState.mode == BT_MODE_OFF) {
        return;  // Nie rysuj nic gdy BT wyłączony
    }
    
    // Symbol Bluetooth (B w kółku)
    u8g2.setFont(u8g2_font_5x7_tr);
    
    // Kolor ikony zależy od trybu i statusu
    // RX = "B<", TX = "B>", AUTO = "BA"
    char icon[4];
    
    switch (btState.mode) {
        case BT_MODE_RX:
            strcpy(icon, "B<");
            break;
        case BT_MODE_TX:
            strcpy(icon, "B>");
            break;
        case BT_MODE_AUTO:
            strcpy(icon, "BA");
            break;
        default:
            return;
    }
    
    // Ramka wokół ikony
    if (btState.connected) {
        u8g2.drawRBox(x, y - 7, 14, 9, 1);
        u8g2.setDrawColor(0);
        u8g2.drawStr(x + 2, y, icon);
        u8g2.setDrawColor(1);
        // Znacznik połączenia
        u8g2.drawStr(x + 15, y, "*");
    } else {
        u8g2.drawFrame(x, y - 7, 14, 9);
        u8g2.drawStr(x + 2, y, icon);
    }
}

// =====================================================================================
// WYŚWIETLANIE - PASEK STATUSU (dla trybu analizatora/głównego)
// =====================================================================================
void BT_drawStatusBar(U8G2& u8g2, int x, int y) {
    if (!btState.enabled || btState.mode == BT_MODE_OFF) {
        return;
    }
    
    u8g2.setFont(u8g2_font_5x7_tr);
    
    // Tryb
    const char* modeStr = "";
    switch (btState.mode) {
        case BT_MODE_RX:   modeStr = "BT:RX"; break;
        case BT_MODE_TX:   modeStr = "BT:TX"; break;
        case BT_MODE_AUTO: modeStr = "BT:AUTO"; break;
        default: return;
    }
    
    u8g2.drawStr(x, y, modeStr);
    
    // Status połączenia
    if (btState.connected) {
        u8g2.drawStr(x + 35, y, "OK");
    } else {
        // Migająca kropka gdy szuka
        if ((millis() / 500) % 2) {
            u8g2.drawStr(x + 35, y, "...");
        }
    }
}

// =====================================================================================
// GŁOŚNOŚĆ BT (dla trybu RX)
// =====================================================================================
void BT_volumeUp(void) {
    bt_emitter.upvolume();
    btState.volume = bt_emitter.getVolume();
}

void BT_volumeDown(void) {
    bt_emitter.downvolume();
    btState.volume = bt_emitter.getVolume();
}

void BT_setVolume(uint8_t vol) {
    bt_emitter.setVolume(vol);
    btState.volume = vol;
}

uint8_t BT_getVolume(void) {
    return btState.volume;
}

// =====================================================================================
// PAROWANIE
// =====================================================================================
void BT_startScan(void) {
    btState.status = BT_STATUS_SCANNING;
    bt_emitter.userCommand("AT+RESET");  // Reset uruchamia skanowanie
    Serial.println("BT: Rozpoczęto skanowanie...");
}

void BT_stopScan(void) {
    if (btState.status == BT_STATUS_SCANNING) {
        btState.status = BT_STATUS_DISCONNECTED;
    }
}

void BT_connectToDevice(const char* addr) {
    btState.status = BT_STATUS_CONNECTING;
    bt_emitter.addLinkAddr(addr);
    Serial.printf("BT: Łączenie z %s...\n", addr);
}

void BT_disconnect(void) {
    bt_emitter.userCommand("AT+RESET");
    btState.connected = false;
    btState.status = BT_STATUS_DISCONNECTED;
    Serial.println("BT: Rozłączono");
}

void BT_deleteAllPairedDevices(void) {
    bt_emitter.deleteVMlinks();
    Serial.println("BT: Usunięto wszystkie sparowane urządzenia");
}

// =====================================================================================
// ZAPIS/ODCZYT KONFIGURACJI
// =====================================================================================
void BT_saveConfig(void) {
    fs::FS& storage = getStorage();
    
    File f = storage.open("/bt_config.txt", FILE_WRITE);
    if (f) {
        f.printf("mode=%d\n", (int)btState.mode);
        f.printf("enabled=%d\n", btState.enabled ? 1 : 0);
        f.printf("volume=%d\n", btState.volume);
        f.close();
        Serial.println("BT: Konfiguracja zapisana");
    } else {
        Serial.println("BT: Błąd zapisu konfiguracji!");
    }
}

void BT_loadConfig(void) {
    fs::FS& storage = getStorage();
    
    File f = storage.open("/bt_config.txt", FILE_READ);
    if (f) {
        while (f.available()) {
            String line = f.readStringUntil('\n');
            line.trim();
            
            if (line.startsWith("mode=")) {
                btState.mode = (BTMode)line.substring(5).toInt();
            }
            else if (line.startsWith("enabled=")) {
                btState.enabled = line.substring(8).toInt() == 1;
            }
            else if (line.startsWith("volume=")) {
                btState.volume = line.substring(7).toInt();
            }
        }
        f.close();
        Serial.println("BT: Konfiguracja wczytana");
    } else {
        // Domyślne wartości
        btState.mode = BT_MODE_OFF;
        btState.enabled = false;
        btState.volume = 15;
        Serial.println("BT: Brak pliku konfiguracji - użyto domyślnych");
    }
}

// =====================================================================================
// POBIERANIE STANU
// =====================================================================================
BTState BT_getState(void) {
    return btState;
}

// =====================================================================================
// TEKSTY TRYBU/STATUSU
// =====================================================================================
const char* BT_getModeString(BTMode mode) {
    switch (mode) {
        case BT_MODE_OFF:  return "OFF";
        case BT_MODE_RX:   return "RX";
        case BT_MODE_TX:   return "TX";
        case BT_MODE_AUTO: return "AUTO";
        default:           return "?";
    }
}

const char* BT_getStatusString(BTStatus status) {
    switch (status) {
        case BT_STATUS_OFF:          return "OFF";
        case BT_STATUS_SCANNING:     return "SCAN";
        case BT_STATUS_DISCONNECTED: return "DISCONN";
        case BT_STATUS_CONNECTING:   return "CONNECTING";
        case BT_STATUS_CONNECTED:    return "CONNECTED";
        default:                     return "?";
    }
}
