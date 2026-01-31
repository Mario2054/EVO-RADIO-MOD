#pragma once
#include <Arduino.h>
#include <U8g2lib.h>

// =====================================================================================
// BT_KCX_Module.* (NO-KCX) — sterowanie BT przez UART do drugiego ESP32-WROOM
// =====================================================================================
// Ten plik ZASTĘPUJE starą obsługę KCX_BT_Emitter.
//
// UART (ESP32-S3 -> ESP32-WROOM):
//   S3 TX (GPIO20)  -> WROOM RX
//   S3 RX (GPIO19)  <- WROOM TX
//   GND wspólne
//
// Komendy (WROOM firmware v3):
//   HELP, PING, GET/STATUS?
//   BT ON, BT OFF
//   MODE OFF|TX|AUTO   (RX zależy od firmware WROOM)
//   VOL 0..100
//   BOOST 100..400
//   SCAN, CONNECT <idx|MAC>, DISCONNECT
//   PAIRED?, DELPAIRED ALL, SAVE, DBG 0|1, HARDRESET
// =====================================================================================

#define BT_UART_PORT   1
#define BT_UART_RX_PIN 19
#define BT_UART_TX_PIN 20
#define BT_UART_BAUD   115200

enum BTMode {
  BT_MODE_OFF  = 0,
  BT_MODE_RX   = 1,   // jeśli WROOM nie obsługuje, dostaniesz ERR MODE
  BT_MODE_TX   = 2,
  BT_MODE_AUTO = 3
};

enum BTStatus {
  BT_STATUS_OFF = 0,
  BT_STATUS_ON  = 1
};

struct BTState {
  BTMode mode;
  BTStatus status;
  bool enabled;
  bool connected;
  bool menuActive;
  uint8_t menuSelection;  // używane w menu OLED
  String deviceName;
  String deviceAddr;
  int volume;             // 0..100
  int boost;              // 100..400
  bool scanning;
};

// --- API używane przez main.cpp i EQ_AnalyzerDisplay.cpp ---
void BT_init(U8G2* u8g2 = nullptr);
void BT_loop();

BTState BT_getState();
BTMode  BT_getMode();
String  BT_getModeString();
String  BT_getModeString(int mode);
String  BT_getModeString(BTMode mode);
BTStatus BT_getStatus();
String  BT_getStatusString();

bool BT_isEnabled();
bool BT_isConnected();
int  BT_getVolume();
String BT_getDeviceName();

void BT_enable(bool en);
void BT_setMode(BTMode mode);
void BT_toggleMode();
void BT_setVolume(int vol_0_100);
void BT_volumeUp();
void BT_volumeDown();

void BT_startScan();
void BT_stopScan();
void BT_disconnect();
void BT_deleteAllPairedDevices();
void BT_saveConfig();
void BT_loadConfig(); // (opcjonalnie) — w UART i tak trzyma WROOM, ale zostawiamy kompatybilność

// menu OLED
bool BT_isMenuActive();
void BT_setMenuActive(bool active);
void BT_menuUp();
void BT_menuDown();
void BT_menuSelect();
// menu OLED (bez argumentu używa wskaźnika ustawionego w BT_init lub BT_displayMenu(u8g2))
void BT_displayMenu();
// kompatybilność: main.cpp wywołuje BT_displayMenu(u8g2)
void BT_displayMenu(U8G2 &u8g2);
void BT_drawStatusIcon(U8G2 &u8g2, int x, int y); // mała ikonka BT na pasku

// --- Dodatki dla WWW / debug ---
String BT_sendRaw(const String& cmd);   // wyślij komendę 1:1
String BT_getLog();                    // log UART (ostatnie linie)
void   BT_clearLog();
