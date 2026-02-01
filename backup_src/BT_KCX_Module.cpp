#include "BT_KCX_Module.h"

static HardwareSerial BTSerial(BT_UART_PORT);
static U8G2* g_u8g2 = nullptr;

static BTState st = {
  .mode = BT_MODE_OFF,
  .status = BT_STATUS_OFF,
  .enabled = false,
  .connected = false,
  .menuActive = false,
  .menuSelection = 0,
  .deviceName = "",
  .deviceAddr = "",
  .volume = 50,
  .boost = 100,
  .scanning = false
};

static bool inited = false;

// --- prosta lista urządzeń z SCAN ---
struct ScanDev {
  int idx = -1;
  String mac;
  int rssi = 0;
  String name;
  bool valid = false;
};
static ScanDev scanList[25];
static int scanCount = 0;

static String rxLineBuf;

// --- log ring-buffer ---
static String logBuf;
static const size_t LOG_MAX = 12000;

static void logAppend(const String& s){
  logBuf += s;
  if (logBuf.length() > (int)LOG_MAX){
    logBuf.remove(0, logBuf.length() - LOG_MAX);
  }
}

static void logLine(const String& s){
  logAppend(s);
  logAppend("\n");
}

static void scanClear(){
  for (auto &d: scanList) d = ScanDev();
  scanCount = 0;
}

static void scanStore(int idx, const String& mac, int rssi, const String& name){
  if (idx < 0) return;
  if (idx >= (int)(sizeof(scanList)/sizeof(scanList[0]))) return;
  scanList[idx].idx = idx;
  scanList[idx].mac = mac;
  scanList[idx].rssi = rssi;
  scanList[idx].name = name;
  scanList[idx].valid = true;
  if (idx+1 > scanCount) scanCount = idx+1;
}

static void sendCmd(const String& cmd){
  BTSerial.print(cmd);
  BTSerial.print("\n");
  logLine(String(">> ") + cmd);
}

static String modeToStr(BTMode m){
  switch(m){
    case BT_MODE_OFF: return "OFF";
    case BT_MODE_RX:  return "RX";
    case BT_MODE_TX:  return "TX";
    case BT_MODE_AUTO:return "AUTO";
  }
  return "OFF";
}

static void parseStateLine(const String& line){
  // Przykład: STATE BT=ON MODE=TX VOL=100 BOOST=400 SCAN=0 CONN=1 MAC=.. NAME=".."
  st.enabled = (line.indexOf("BT=ON") >= 0);
  st.status  = st.enabled ? BT_STATUS_ON : BT_STATUS_OFF;
  st.scanning = (line.indexOf("SCAN=1") >= 0);
  st.connected = (line.indexOf("CONN=1") >= 0);

  // MODE=
  int p = line.indexOf("MODE=");
  if (p >= 0){
    int e = line.indexOf(' ', p);
    String m = (e>p) ? line.substring(p+5, e) : line.substring(p+5);
    m.trim();
    if (m=="OFF") st.mode = BT_MODE_OFF;
    else if (m=="RX") st.mode = BT_MODE_RX;
    else if (m=="TX") st.mode = BT_MODE_TX;
    else if (m=="AUTO") st.mode = BT_MODE_AUTO;
  }

  // VOL=
  p = line.indexOf("VOL=");
  if (p >= 0){
    int e = line.indexOf(' ', p);
    String v = (e>p) ? line.substring(p+4, e) : line.substring(p+4);
    st.volume = v.toInt();
    if (st.volume < 0) st.volume = 0;
    if (st.volume > 100) st.volume = 100;
  }

  // BOOST=
  p = line.indexOf("BOOST=");
  if (p >= 0){
    int e = line.indexOf(' ', p);
    String b = (e>p) ? line.substring(p+6, e) : line.substring(p+6);
    st.boost = b.toInt();
    if (st.boost < 100) st.boost = 100;
    if (st.boost > 400) st.boost = 400;
  }

  // MAC=
  p = line.indexOf("MAC=");
  if (p >= 0){
    int e = line.indexOf(' ', p);
    String mac = (e>p) ? line.substring(p+4, e) : line.substring(p+4);
    mac.trim();
    if (mac == "None") mac = "";
    st.deviceAddr = mac;
  }

  // NAME="..."
  p = line.indexOf("NAME=\"");
  if (p >= 0){
    int q = line.indexOf('"', p+6);
    int q2 = (q>=0) ? line.indexOf('"', q+1) : -1;
    if (q2>q){
      st.deviceName = line.substring(q+1, q2);
    }
  }
}

static void parseDevLine(const String& line){
  // DEV <idx> <MAC> RSSI=<n> NAME="<name>"
  // przykład: DEV 0 AA:BB:.. RSSI=-55 NAME="XYZ"
  int p = line.indexOf("DEV ");
  if (p < 0) return;
  String s = line.substring(p+4);
  s.trim();
  int sp1 = s.indexOf(' ');
  if (sp1 < 0) return;
  int idx = s.substring(0, sp1).toInt();
  s = s.substring(sp1+1);
  s.trim();

  int sp2 = s.indexOf(' ');
  if (sp2 < 0) return;
  String mac = s.substring(0, sp2);
  s = s.substring(sp2+1);

  int rssi = 0;
  p = s.indexOf("RSSI=");
  if (p >= 0){
    int e = s.indexOf(' ', p);
    String rr = (e>p) ? s.substring(p+5, e) : s.substring(p+5);
    rssi = rr.toInt();
  }

  String name;
  p = s.indexOf("NAME=\"");
  if (p >= 0){
    int q1 = s.indexOf('"', p+6);
    int q2 = (q1>=0) ? s.indexOf('"', q1+1) : -1;
    if (q2 > q1) name = s.substring(q1+1, q2);
  }

  scanStore(idx, mac, rssi, name);
}

static void handleLine(const String& line){
  logLine(String("<< ") + line);

  if (line.startsWith("STATE ")){
    parseStateLine(line);
    return;
  }
  if (line.startsWith("DEV ")){
    parseDevLine(line);
    return;
  }
  if (line.startsWith("SCAN START")){ st.scanning = true; scanClear(); return; }
  if (line.startsWith("SCAN DONE")){ st.scanning = false; return; }
}

static void readUart(){
  while(BTSerial.available()){
    char c = (char)BTSerial.read();
    if (c=='\n'){
      String line = rxLineBuf;
      rxLineBuf = "";
      line.trim();
      if (line.length()) handleLine(line);
    } else if (c!='\r'){
      rxLineBuf += c;
      if (rxLineBuf.length() > 300) rxLineBuf = "";
    }
  }
}

void BT_init(U8G2* u8g2){
  g_u8g2 = u8g2;
  if (inited) return;

  BTSerial.begin(BT_UART_BAUD, SERIAL_8N1, BT_UART_RX_PIN, BT_UART_TX_PIN);
  delay(50);

  scanClear();
  logLine("BT UART READY (NO-KCX)");

  // podstawowe ustawienie: włącz BT po stronie WROOM i pobierz status
  sendCmd("BT ON");
  sendCmd("GET");

  inited = true;
  inited = true;
}

void BT_loop(){
  if (!inited) return;
  readUart();
}

// --- getters ---
BTState BT_getState(){ return st; }
BTMode  BT_getMode(){ return st.mode; }
String  BT_getModeString(){ return modeToStr(st.mode); }
BTStatus BT_getStatus(){ return st.status; }
String  BT_getStatusString(){ return st.enabled ? "ON" : "OFF"; }

bool BT_isEnabled(){ return st.enabled; }
bool BT_isConnected(){ return st.connected; }
int  BT_getVolume(){ return st.volume; }
String BT_getDeviceName(){ return st.deviceName; }

// --- setters / actions ---
void BT_enable(bool en){
  if (en) sendCmd("BT ON");
  else sendCmd("BT OFF");
}

void BT_setMode(BTMode mode){
  st.mode = mode;
  sendCmd(String("MODE ") + modeToStr(mode));
  sendCmd("GET");
}

void BT_toggleMode(){
  // OFF -> RX -> TX -> AUTO -> OFF
  int m = (int)st.mode;
  m = (m + 1) % 4;
  BT_setMode((BTMode)m);
}

void BT_setVolume(int vol_0_100){
  if (vol_0_100 < 0) vol_0_100 = 0;
  if (vol_0_100 > 100) vol_0_100 = 100;
  st.volume = vol_0_100;
  sendCmd(String("VOL ") + String(st.volume));
}

void BT_volumeUp(){ BT_setVolume(st.volume + 1); }
void BT_volumeDown(){ BT_setVolume(st.volume - 1); }

void BT_startScan(){
  st.scanning = true;
  scanClear();
  sendCmd("SCAN");
}

void BT_stopScan(){
  // firmware WROOM nie ma STOP, więc tylko stan lokalny
  st.scanning = false;
}

void BT_disconnect(){
  sendCmd("DISCONNECT");
  st.connected = false;
  st.deviceAddr = "";
  st.deviceName = "";
}

void BT_deleteAllPairedDevices(){
  sendCmd("DELPAIRED ALL");
}

void BT_saveConfig(){
  sendCmd("SAVE");
}

void BT_loadConfig(){
  // ustawienia trzyma WROOM; na start robimy GET
  sendCmd("GET");
}

// --- OLED menu ---
bool BT_isMenuActive(){ return st.menuActive; }
void BT_setMenuActive(bool active){ st.menuActive = active; }

static const char* menuItems[] = {
  "MODE: OFF/RX/TX/AUTO",
  "VOL +",
  "VOL -",
  "SCAN",
  "CONNECT 0",
  "DISCONNECT",
  "DELPAIRED ALL",
  "SAVE",
  "BACK"
};
static const int MENU_COUNT = sizeof(menuItems)/sizeof(menuItems[0]);

void BT_menuUp(){
  if (!st.menuActive) return;
  if (st.menuSelection == 0) st.menuSelection = MENU_COUNT-1;
  else st.menuSelection--;
}

void BT_menuDown(){
  if (!st.menuActive) return;
  st.menuSelection = (st.menuSelection + 1) % MENU_COUNT;
}

void BT_menuSelect(){
  if (!st.menuActive) return;

  switch(st.menuSelection){
    case 0: BT_toggleMode(); break;
    case 1: BT_volumeUp(); break;
    case 2: BT_volumeDown(); break;
    case 3: BT_startScan(); break;
    case 4: sendCmd("CONNECT 0"); break;
    case 5: BT_disconnect(); break;
    case 6: BT_deleteAllPairedDevices(); break;
    case 7: BT_saveConfig(); break;
    case 8: st.menuActive = false; break;
  }
}

void BT_displayMenu(){
  if (!g_u8g2 || !st.menuActive) return;

  g_u8g2->clearBuffer();
  g_u8g2->setFont(u8g2_font_6x12_tf);

  String top = "BT: " + BT_getStatusString() + "  MODE:" + BT_getModeString();
  g_u8g2->drawStr(0, 12, top.c_str());
  String v = "VOL:" + String(st.volume) + "  BOOST:" + String(st.boost);
  g_u8g2->drawStr(0, 24, v.c_str());

  int y0 = 38;
  for(int i=0;i<MENU_COUNT && i<5;i++){
    int idx = i;
    int y = y0 + i*12;
    String line = String(menuItems[idx]);
    if (idx == st.menuSelection){
      g_u8g2->drawBox(0, y-10, 256, 12);
      g_u8g2->setDrawColor(0);
      g_u8g2->drawStr(2, y, line.c_str());
      g_u8g2->setDrawColor(1);
    } else {
      g_u8g2->drawStr(2, y, line.c_str());
    }
  }

  // pokaz 0..3 urządzenia z listy skanu (dla podglądu)
  int y = 62;
  if (scanCount > 0){
    String d0 = "DEV0: " + scanList[0].name;
    g_u8g2->drawStr(0, y, d0.c_str());
  }

  g_u8g2->sendBuffer();
}

void BT_displayMenu(U8G2 &u8g2){
  g_u8g2 = &u8g2;
  BT_displayMenu();
}


void BT_drawStatusIcon(U8G2 &u8g2, int x, int y){

  // prosta ikonka: mały "BT" + kropka jak połączone
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(x, y, "BT");
  if (st.connected){
    u8g2.drawDisc(x+18, y-4, 2);
  }
}

// --- WWW helpers ---
String BT_sendRaw(const String& cmd){
  if (!inited) return "ERR";
  sendCmd(cmd);
  return "OK";
}

String BT_getLog(){ return logBuf; }
void BT_clearLog(){ logBuf = ""; }
static String _btModeToStr(int m){
  switch(m){
    case BT_MODE_OFF:  return "OFF";
    case BT_MODE_RX:   return "RX";
    case BT_MODE_TX:   return "TX";
    case BT_MODE_AUTO: return "AUTO";
    default:           return "UNK";
  }
}

String BT_getModeString(int mode){
  return _btModeToStr(mode);
}

String BT_getModeString(BTMode mode){
  return _btModeToStr((int)mode);
}


