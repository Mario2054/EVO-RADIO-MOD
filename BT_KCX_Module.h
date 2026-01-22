// =====================================================================================
// BT_KCX_Module.h - Moduł obsługi KCX_BT_EMITTER dla EVO Radio
// =====================================================================================
// Połączenie GPIO:
//   RX (ESP32)     -> GPIO 18  -> TX (KCX)
//   TX (ESP32)     -> GPIO 44  -> RX (KCX)  
//   LINK           -> GPIO 43  -> LINK (KCX) - status połączenia
//   MODE           -> GPIO 2   -> MODE (KCX) - przełączanie RX/TX
// =====================================================================================

#pragma once
#include <Arduino.h>
#include <U8g2lib.h>
#include "KCX_BT_Emitter.h"

// =====================================================================================
// DEFINICJE GPIO DLA KCX_BT_EMITTER
// =====================================================================================
#define BT_EMITTER_RX_PIN    18   // ESP32 RX <- KCX TX
#define BT_EMITTER_TX_PIN    44   // ESP32 TX -> KCX RX  
#define BT_EMITTER_LINK_PIN  43   // Status połączenia BT (HIGH = connected)
#define BT_EMITTER_MODE_PIN  2    // Przełączanie trybu (LOW = RX, HIGH = TX)

// =====================================================================================
// TRYBY PRACY BLUETOOTH
// =====================================================================================
enum BTMode {
    BT_MODE_OFF  = 0,    // Bluetooth wyłączony
    BT_MODE_RX   = 1,    // Tryb odbiornika (słuchawki -> radio)
    BT_MODE_TX   = 2,    // Tryb nadajnika (radio -> słuchawki BT)
    BT_MODE_AUTO = 3     // Automatyczne przełączanie RX/TX
};

// =====================================================================================
// STATUS POŁĄCZENIA BLUETOOTH
// =====================================================================================
enum BTStatus {
    BT_STATUS_OFF          = 0,   // BT wyłączony
    BT_STATUS_SCANNING     = 1,   // Skanowanie urządzeń
    BT_STATUS_DISCONNECTED = 2,   // Rozłączony
    BT_STATUS_CONNECTING   = 3,   // Łączenie
    BT_STATUS_CONNECTED    = 4    // Połączony
};

// =====================================================================================
// STRUKTURA STANU BLUETOOTH
// =====================================================================================
struct BTState {
    BTMode mode;
    BTStatus status;
    bool enabled;
    bool connected;
    bool menuActive;
    uint8_t menuSelection;      // 0=OFF, 1=RX, 2=TX, 3=AUTO
    String deviceName;
    String deviceAddr;
    int8_t volume;
};

// =====================================================================================
// DEKLARACJE FUNKCJI PUBLICZNYCH
// =====================================================================================

// Inicjalizacja modułu BT
void BT_init(void);

// Pętla główna - wywoływać w loop()
void BT_loop(void);

// Sterowanie trybem
void BT_setMode(BTMode mode);
BTMode BT_getMode(void);
void BT_toggleMode(void);      // Przełączanie: OFF->RX->TX->AUTO->OFF

// Sterowanie włączeniem
void BT_enable(bool enabled);
bool BT_isEnabled(void);

// Status połączenia
BTStatus BT_getStatus(void);
bool BT_isConnected(void);
String BT_getDeviceName(void);

// Menu obsługa
bool BT_isMenuActive(void);
void BT_setMenuActive(bool active);
void BT_menuUp(void);
void BT_menuDown(void);
void BT_menuSelect(void);

// Wyświetlanie
void BT_displayMenu(U8G2& u8g2);
void BT_drawStatusIcon(U8G2& u8g2, int x, int y);
void BT_drawStatusBar(U8G2& u8g2, int x, int y);

// Głośność BT (dla trybu RX)
void BT_volumeUp(void);
void BT_volumeDown(void);
void BT_setVolume(uint8_t vol);
uint8_t BT_getVolume(void);

// Parowanie
void BT_startScan(void);
void BT_stopScan(void);
void BT_connectToDevice(const char* addr);
void BT_disconnect(void);
void BT_deleteAllPairedDevices(void);

// Zapisywanie/odczyt konfiguracji
void BT_saveConfig(void);
void BT_loadConfig(void);

// Pobieranie stanu
BTState BT_getState(void);

// Tekst trybu
const char* BT_getModeString(BTMode mode);
const char* BT_getStatusString(BTStatus status);
