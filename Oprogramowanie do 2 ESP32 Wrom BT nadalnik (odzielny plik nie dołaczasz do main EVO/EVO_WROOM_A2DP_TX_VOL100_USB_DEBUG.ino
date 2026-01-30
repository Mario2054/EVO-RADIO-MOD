/*
  EVO-BT-TX v3 (ESP32-WROOM-32D) — Arduino IDE + USB DEBUG + VOL 0..100
  --------------------------------------------------------------------
  Zmiany:
  - VOL ma zakres 0..100 (zamiast 0..30)
  - Mapowanie do A2DP: 0..127
  - Dodatkowo (opcjonalnie): BOOST 100..400 (% wzmocnienia cyfrowego próbek PCM)
      BOOST 100 = bez zmian
      BOOST 200 = 2x (może przesterować)

  Komendy:
    HELP, PING, GET/STATUS?
    BT ON, BT OFF
    MODE OFF|TX|AUTO
    VOL 0..100
    BOOST 100..400
    SCAN
    CONNECT <idx> lub CONNECT AA:BB:CC:DD:EE:FF
    DISCONNECT
    PAIRED?
    DELPAIRED ALL
    SAVE
    DBG 0|1
    HARDRESET
*/

#include <Arduino.h>
#include "BluetoothA2DPSource.h"

extern "C" {
  #include "nvs_flash.h"
  #include "nvs.h"
  #include "driver/i2s.h"
  #include "esp_gap_bt_api.h"
}

#include <stdarg.h>

// ====== KONFIG PINÓW (DOPASUJ) ======
static const int PIN_UART_RX = 16;
static const int PIN_UART_TX = 17;
static const uint32_t UART_BAUD = 115200;

// I2S (podsłuch z S3, równolegle z PCM5102A)
static const int PIN_I2S_BCLK = 26;
static const int PIN_I2S_WS   = 25;
static const int PIN_I2S_DIN  = 22;

// Audio: 48 kHz / 16-bit
static const int AUDIO_SR = 48000;
static const i2s_bits_per_sample_t AUDIO_BITS = I2S_BITS_PER_SAMPLE_16BIT;
// =====================================

HardwareSerial CTRL(2);
BluetoothA2DPSource a2dp;
static const i2s_port_t I2S_PORT = I2S_NUM_0;

// ====== DEBUG / LOG ======
static bool USB_DEBUG = true;

static void logLn(const char* s){
  CTRL.println(s);
  if (USB_DEBUG) Serial.println(s);
}

static void logF(const char* fmt, ...){
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  CTRL.print(buf);
  if (USB_DEBUG) Serial.print(buf);
}
// =========================

enum Mode : uint8_t { MODE_OFF=0, MODE_TX=1, MODE_AUTO=2 };
static volatile Mode g_mode = MODE_TX;

static bool g_btReady = false;
static bool g_scanning = false;

static String g_connMac = "";
static String g_connName = "";

// VOL: 0..100
static int g_vol_ui = 100;
static uint8_t g_vol_127 = 127;

// BOOST: 100..400 (%)
static int g_boost_pct = 400;

struct Dev {
  esp_bd_addr_t bda{};
  int rssi = 0;
  String name;
  bool valid = false;
};
static Dev g_scan[25];
static int g_scanCount = 0;

// --------- Utils ----------
static String bdaToStr(const esp_bd_addr_t bda){
  char s[18];
  snprintf(s, sizeof(s), "%02X:%02X:%02X:%02X:%02X:%02X",
           bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return String(s);
}

static bool parseMac(const String& mac, esp_bd_addr_t out){
  int b[6];
  if (sscanf(mac.c_str(), "%x:%x:%x:%x:%x:%x", &b[0],&b[1],&b[2],&b[3],&b[4],&b[5]) != 6) return false;
  for(int i=0;i<6;i++) out[i] = (uint8_t)b[i];
  return true;
}

static void scanClear(){
  for (auto &d: g_scan) d = Dev();
  g_scanCount = 0;
}

static void scanStore(const esp_bd_addr_t bda, int rssi, const String& name){
  for(int i=0;i<g_scanCount;i++){
    if (g_scan[i].valid && memcmp(g_scan[i].bda, bda, 6) == 0){
      g_scan[i].rssi = rssi;
      if (name.length()) g_scan[i].name = name;
      return;
    }
  }
  if (g_scanCount >= (int)(sizeof(g_scan)/sizeof(g_scan[0]))) return;
  memcpy(g_scan[g_scanCount].bda, bda, 6);
  g_scan[g_scanCount].rssi = rssi;
  g_scan[g_scanCount].name = name;
  g_scan[g_scanCount].valid = true;
  g_scanCount++;
}

// --------- I2S ----------
static void i2s_init_slave_rx(){
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX);
  cfg.sample_rate = AUDIO_SR;
  cfg.bits_per_sample = AUDIO_BITS;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = PIN_I2S_BCLK;
  pins.ws_io_num = PIN_I2S_WS;
  pins.data_out_num = -1;
  pins.data_in_num = PIN_I2S_DIN;
#if ESP_IDF_VERSION_MAJOR >= 5
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
#endif

  i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
  i2s_set_pin(I2S_PORT, &pins);
  i2s_zero_dma_buffer(I2S_PORT);
}

// callback dla A2DP: pobiera dane audio
static int32_t get_data(uint8_t *data, int32_t len){
  size_t bytesRead = 0;
  if (i2s_read(I2S_PORT, data, len, &bytesRead, portMAX_DELAY) != ESP_OK) return 0;
  if (bytesRead == 0) return 0;

  if (g_boost_pct != 100){
    int16_t *s = (int16_t*)data;
    int count = bytesRead / 2;
    int gain_q10 = (g_boost_pct * 1024) / 100; // 100% = 1024
    for(int i=0;i<count;i++){
      int32_t v = (int32_t)s[i] * gain_q10;
      v >>= 10;
      if (v > 32767) v = 32767;
      if (v < -32768) v = -32768;
      s[i] = (int16_t)v;
    }
  }

  return (int32_t)bytesRead;
}

// --------- Discovery callbacks (z biblioteki) ----------
static bool ssid_found_cb(const char *ssid, esp_bd_addr_t address, int rssi){
  String name = ssid ? String(ssid) : String("");
  scanStore(address, rssi, name);
  logF("DEV %d %s RSSI=%d NAME=\"%s\"\n", g_scanCount-1, bdaToStr(address).c_str(), rssi, name.c_str());
  return true;
}

static void discovery_state_cb(esp_bt_gap_discovery_state_t st){
  if (st == ESP_BT_GAP_DISCOVERY_STARTED){
    g_scanning = true;
    logLn("SCAN START");
  } else if (st == ESP_BT_GAP_DISCOVERY_STOPPED){
    g_scanning = false;
    logF("SCAN DONE COUNT=%d\n", g_scanCount);
  }
}

// --------- NVS save/load ----------
static void cfg_save(){
  nvs_handle_t h;
  if (nvs_open("btcfg", NVS_READWRITE, &h) != ESP_OK){ logLn("ERR SAVE"); return; }
  nvs_set_i32(h, "mode", (int)g_mode);
  nvs_set_i32(h, "vol", g_vol_ui);
  nvs_set_i32(h, "boost", g_boost_pct);
  nvs_set_str(h, "mac", g_connMac.c_str());
  nvs_commit(h);
  nvs_close(h);
  logLn("OK SAVE");
}

static void cfg_load(){
  nvs_handle_t h;
  if (nvs_open("btcfg", NVS_READONLY, &h) != ESP_OK) return;

  int32_t m=0, v=50, b=100;
  size_t len=0;
  if (nvs_get_i32(h, "mode", &m) == ESP_OK) g_mode = (Mode)m;
  if (nvs_get_i32(h, "vol", &v) == ESP_OK) g_vol_ui = (int)v;
  if (nvs_get_i32(h, "boost", &b) == ESP_OK) g_boost_pct = (int)b;

  nvs_get_str(h, "mac", nullptr, &len);
  if (len > 1 && len < 32){
    char buf[32];
    if (nvs_get_str(h, "mac", buf, &len) == ESP_OK) g_connMac = String(buf);
  }
  nvs_close(h);

  if (g_vol_ui < 0) g_vol_ui = 0;
  if (g_vol_ui > 100) g_vol_ui = 100;
  g_vol_127 = (uint8_t)lround((double)g_vol_ui * 127.0 / 100.0);

  if (g_boost_pct < 100) g_boost_pct = 100;
  if (g_boost_pct > 400) g_boost_pct = 400;
}

// --------- BT start (raz) ----------
static void ensureBtStarted(){
  if (g_btReady) return;

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
    nvs_flash_erase();
    nvs_flash_init();
  }

  i2s_init_slave_rx();

  a2dp.set_local_name("EVO-BT-TX");
  a2dp.set_data_callback(get_data);
  a2dp.set_auto_reconnect(false);
  a2dp.set_ssid_callback(ssid_found_cb);
  a2dp.set_discovery_mode_callback(discovery_state_cb);

  a2dp.start();
  a2dp.set_volume(g_vol_127);

  g_btReady = true;
}

// --------- actions ----------
static void status_send(){
  logF("STATE BT=%s MODE=%s VOL=%d BOOST=%d SCAN=%d CONN=%d MAC=%s NAME=\"%s\"\n",
    g_btReady ? "ON":"OFF",
    g_mode==MODE_OFF?"OFF":(g_mode==MODE_TX?"TX":"AUTO"),
    g_vol_ui,
    g_boost_pct,
    g_scanning ? 1:0,
    g_connMac.length()?1:0,
    g_connMac.length()?g_connMac.c_str():"None",
    g_connName.c_str()
  );
}

static void soft_off(){
  if (g_scanning){
    esp_bt_gap_cancel_discovery();
    g_scanning = false;
  }
  a2dp.disconnect();
  g_connMac = "";
  g_connName = "";
  g_mode = MODE_OFF;
  logLn("OK MODE OFF");
}

static void scan_start(){
  ensureBtStarted();
  scanClear();
  esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 12, 0);
}

static void connect_mac(const String& mac){
  ensureBtStarted();
  esp_bd_addr_t bda{};
  if (!parseMac(mac, bda)){
    logLn("ERR CONNECT MAC");
    return;
  }
  bool ok = a2dp.connect_to(bda);
  if (ok){
    g_connMac = mac;
    g_connName = "";
    logF("OK CONNECT %s\n", mac.c_str());
  } else {
    logLn("ERR CONNECT");
  }
}

static void connect_idx(int idx){
  if (idx < 0 || idx >= g_scanCount || !g_scan[idx].valid){
    logLn("ERR CONNECT IDX");
    return;
  }
  String mac = bdaToStr(g_scan[idx].bda);
  connect_mac(mac);
  g_connName = g_scan[idx].name;
}

static void disconnect_bt(){
  a2dp.disconnect();
  g_connMac = "";
  g_connName = "";
  logLn("OK DISCONNECT");
}

static void paired_list(){
  ensureBtStarted();
  int n = esp_bt_gap_get_bond_device_num();
  if (n <= 0){
    logLn("PAIRED DONE COUNT=0");
    return;
  }
  esp_bd_addr_t *list = (esp_bd_addr_t*)malloc(n * sizeof(esp_bd_addr_t));
  if (!list){ logLn("ERR MEM"); return; }
  esp_bt_gap_get_bond_device_list(&n, list);
  for (int i=0;i<n;i++){
    logF("PAIRED %d %s NAME=\"\"\n", i, bdaToStr(list[i]).c_str());
  }
  free(list);
  logF("PAIRED DONE COUNT=%d\n", n);
}

// --------- DELPAIRED in background ----------
static TaskHandle_t g_delTask = nullptr;

static void delpaired_task(void *){
  ensureBtStarted();
  a2dp.disconnect();
  if (g_scanning){
    esp_bt_gap_cancel_discovery();
    g_scanning = false;
  }

  int n = esp_bt_gap_get_bond_device_num();
  if (n <= 0){
    logLn("OK DELPAIRED 0");
    g_delTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  esp_bd_addr_t *list = (esp_bd_addr_t*)malloc(n * sizeof(esp_bd_addr_t));
  if (!list){
    logLn("ERR MEM");
    g_delTask = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  esp_bt_gap_get_bond_device_list(&n, list);
  int removed = 0;
  for (int i=0;i<n;i++){
    if (esp_bt_gap_remove_bond_device(list[i]) == ESP_OK) removed++;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  free(list);

  g_connMac = "";
  g_connName = "";
  logF("OK DELPAIRED %d\n", removed);

  g_delTask = nullptr;
  vTaskDelete(nullptr);
}

static void delpaired_all_async(){
  if (g_delTask){
    logLn("ERR DELPAIRED BUSY");
    return;
  }
  logLn("OK DELPAIRED START");
  xTaskCreatePinnedToCore(delpaired_task, "delpaired", 4096, nullptr, 1, &g_delTask, 1);
}

// --------- cmd parser ----------
static void help_print(){
  logLn("CMDS: HELP, PING, GET, STATUS?, BT ON, BT OFF, MODE OFF|TX|AUTO, VOL 0..100, BOOST 100..400, SCAN, CONNECT <idx|MAC>, DISCONNECT, PAIRED?, DELPAIRED ALL, SAVE, DBG 0|1, HARDRESET");
}

static void handle_cmd(String s, const char* src){
  s.trim();
  if (!s.length()) return;

  logF("CMD[%s] \"%s\"\n", src, s.c_str());

  if (s == "PING"){ logLn("PONG"); return; }
  if (s == "HELP"){ help_print(); return; }

  if (s.startsWith("DBG ")){
    int v = s.substring(4).toInt();
    USB_DEBUG = (v != 0);
    logF("OK DBG %d\n", USB_DEBUG ? 1 : 0);
    return;
  }

  if (s=="GET" || s=="STATUS?"){ status_send(); return; }

  if (s=="BT ON"){ ensureBtStarted(); logLn("OK BT ON"); return; }
  if (s=="BT OFF"){ soft_off(); return; }

  if (s.startsWith("MODE ")){
    String m = s.substring(5); m.trim();
    if (m=="OFF"){ soft_off(); return; }
    if (m=="TX"){ ensureBtStarted(); g_mode=MODE_TX; logLn("OK MODE TX"); return; }
    if (m=="AUTO"){ ensureBtStarted(); g_mode=MODE_AUTO; logLn("OK MODE AUTO"); return; }
    logLn("ERR MODE"); return;
  }

  if (s.startsWith("VOL ")){
    int v = s.substring(4).toInt();
    if (v<0) v=0;
    if (v>100) v=100;
    g_vol_ui = v;
    g_vol_127 = (uint8_t)lround((double)g_vol_ui * 127.0 / 100.0);
    if (g_btReady) a2dp.set_volume(g_vol_127);
    logF("OK VOL %d\n", g_vol_ui);
    return;
  }

  if (s.startsWith("BOOST ")){
    int b = s.substring(6).toInt();
    if (b < 100) b = 100;
    if (b > 400) b = 400;
    g_boost_pct = b;
    logF("OK BOOST %d\n", g_boost_pct);
    return;
  }

  if (s=="SCAN"){ scan_start(); return; }

  if (s.startsWith("CONNECT ")){
    String a = s.substring(8); a.trim();
    if (a.indexOf(':') >= 0){ connect_mac(a); return; }
    int idx = a.toInt();
    connect_idx(idx);
    return;
  }

  if (s=="DISCONNECT"){ disconnect_bt(); return; }
  if (s=="PAIRED?"){ paired_list(); return; }
  if (s=="DELPAIRED ALL"){ delpaired_all_async(); return; }
  if (s=="SAVE"){ cfg_save(); return; }

  if (s=="HARDRESET"){ logLn("OK HARDRESET"); delay(50); ESP.restart(); }

  logLn("ERR UNKNOWN");
}

// --------- read line from stream ----------
static bool readLineFrom(Stream& st, String& buf, String& outLine){
  while (st.available()){
    char c = (char)st.read();
    if (c == '\n'){
      outLine = buf;
      buf = "";
      return outLine.length() > 0;
    } else if (c != '\r'){
      buf += c;
      if (buf.length() > 300) buf = "";
    }
  }
  return false;
}

void setup(){
  Serial.begin(115200);
  CTRL.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

  logLn("READY EVO-BT-TX");

  cfg_load();
  ensureBtStarted();
}

void loop(){
  static String bufUart, bufUsb, line;

  if (readLineFrom(CTRL, bufUart, line)){
    handle_cmd(line, "UART");
  }
  if (readLineFrom(Serial, bufUsb, line)){
    handle_cmd(line, "USB");
  }

  delay(1);
}
