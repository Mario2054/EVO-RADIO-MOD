/*
  EVO-BT-TX v5 (ESP32-WROOM-32D) — AUTO 44.1/48k + RESAMPLE + BETTER SCAN + UART EVENTS
  -----------------------------------------------------------------------------------
  Co robi:
  - Wykrywa częstotliwość WS/LRCLK (~44100 lub ~48000) na PIN_I2S_WS
  - Gdy źródło = 44.1k: passthrough
  - Gdy źródło = 48k: resampling 48k -> 44.1k (linear interpolation)
  - Ring buffer PCM (żeby uniknąć blokowania i underrunów)
  - GAP scan + EIR (lepsze nazwy urządzeń)
  - Eventy po UART/USB: A2DP_CONN / A2DP_AUDIO / SRC_FS
  - VOL 0..100 -> 0..127
  - BOOST 100..400 (%), domyślnie 100

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
  #include "esp_bt_main.h"
  #include "esp_bt_device.h"
  #include "esp_a2dp_api.h"
}

#include <stdarg.h>

// ====== KONFIG PINÓW (DOPASUJ) ======
static const int PIN_UART_RX = 16;
static const int PIN_UART_TX = 17;
static const uint32_t UART_BAUD = 115200;

// I2S (podsłuch z S3)
static const int PIN_I2S_BCLK = 26;
static const int PIN_I2S_WS   = 25;  // LRCLK/WS - tu mierzymy Fs
static const int PIN_I2S_DIN  = 22;

// audio format
static const i2s_bits_per_sample_t AUDIO_BITS = I2S_BITS_PER_SAMPLE_16BIT;

// A2DP output (stałe) — w praktyce ESP32 A2DP source lubi 44.1k
static const int OUT_SR = 44100;
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

// VOL: 0..100 -> 0..127
static int g_vol_ui = 100;
static uint8_t g_vol_127 = 127;

// BOOST: 100..400 (%), domyślnie 100
static int g_boost_pct = 100;

// ===== Scan list =====
struct Dev {
  esp_bd_addr_t bda{};
  int rssi = 0;
  String name;
  bool valid = false;
};
static Dev g_scan[25];
static int g_scanCount = 0;

// ===== Source sample rate detect =====
enum SrcRate : uint8_t { SRC_UNKNOWN=0, SRC_44100=1, SRC_48000=2 };
static volatile SrcRate g_srcRate = SRC_UNKNOWN;
static volatile int g_srcHz = 0;

// ISR counter
static volatile uint32_t g_wsEdges = 0;

// ===== Ring buffer PCM =====
// przechowujemy stereo frames: L,R (int16,int16)
// rozmiar w frames (nie w samplach)
static const int RB_FRAMES = 8192; // 8192 frames ~ 8192/48k = 170 ms
static int16_t rb[RB_FRAMES * 2];  // [L,R,L,R...]
static volatile uint32_t rb_w = 0; // write index in frames
static volatile uint32_t rb_r = 0; // read index in frames
static portMUX_TYPE rb_mux = portMUX_INITIALIZER_UNLOCKED;

static inline uint32_t rb_count_frames(){
  uint32_t w = rb_w, r = rb_r;
  return (w >= r) ? (w - r) : (RB_FRAMES - (r - w));
}

static inline uint32_t rb_free_frames(){
  // zostawiamy 1 frame wolny żeby odróżnić full/empty
  return (RB_FRAMES - 1) - rb_count_frames();
}

static void rb_clear(){
  portENTER_CRITICAL(&rb_mux);
  rb_w = rb_r = 0;
  portEXIT_CRITICAL(&rb_mux);
}

static bool rb_push_frames(const int16_t* framesLR, uint32_t frames){
  bool ok = true;
  portENTER_CRITICAL(&rb_mux);
  uint32_t freeF = rb_free_frames();
  if (frames > freeF){
    // overflow: utnij nadmiar (lepsze niż blokowanie)
    frames = freeF;
    ok = false;
  }
  for(uint32_t i=0;i<frames;i++){
    uint32_t wi = rb_w;
    rb[wi*2 + 0] = framesLR[i*2 + 0];
    rb[wi*2 + 1] = framesLR[i*2 + 1];
    rb_w = (wi + 1) % RB_FRAMES;
  }
  portEXIT_CRITICAL(&rb_mux);
  return ok;
}

static uint32_t rb_pop_frames(int16_t* outLR, uint32_t frames){
  uint32_t got = 0;
  portENTER_CRITICAL(&rb_mux);
  uint32_t avail = rb_count_frames();
  if (frames > avail) frames = avail;
  for(uint32_t i=0;i<frames;i++){
    uint32_t ri = rb_r;
    outLR[i*2 + 0] = rb[ri*2 + 0];
    outLR[i*2 + 1] = rb[ri*2 + 1];
    rb_r = (ri + 1) % RB_FRAMES;
    got++;
  }
  portEXIT_CRITICAL(&rb_mux);
  return got;
}

// ===== Resampler state (48 -> 44.1) =====
// phase w Q16 oznacza pozycję w wejściu (w frames)
// step = input_sr / output_sr w Q16
static uint32_t g_phase_q16 = 0;
static uint32_t g_step_q16  = 0;

// do interpolacji potrzebujemy dwóch kolejnych frames
static bool rb_peek_two(int16_t &l0,int16_t &r0,int16_t &l1,int16_t &r1){
  bool ok = false;
  portENTER_CRITICAL(&rb_mux);
  uint32_t avail = rb_count_frames();
  if (avail >= 2){
    uint32_t ri0 = rb_r;
    uint32_t ri1 = (ri0 + 1) % RB_FRAMES;
    l0 = rb[ri0*2 + 0]; r0 = rb[ri0*2 + 1];
    l1 = rb[ri1*2 + 0]; r1 = rb[ri1*2 + 1];
    ok = true;
  }
  portEXIT_CRITICAL(&rb_mux);
  return ok;
}

static void rb_drop_frames(uint32_t frames){
  portENTER_CRITICAL(&rb_mux);
  uint32_t avail = rb_count_frames();
  if (frames > avail) frames = avail;
  rb_r = (rb_r + frames) % RB_FRAMES;
  portEXIT_CRITICAL(&rb_mux);
}

// ====== Utils ======
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

static int scanFindByMac(const String& mac){
  for(int i=0;i<g_scanCount;i++){
    if (!g_scan[i].valid) continue;
    if (bdaToStr(g_scan[i].bda).equalsIgnoreCase(mac)) return i;
  }
  return -1;
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

// ===== I2S init (slave RX) =====
static void i2s_init_slave_rx(){
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX);
  cfg.sample_rate = OUT_SR; // w SLAVE i tak liczy się zewnętrzny WS, ale zostawiamy sensowną wartość
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

// ===== WS ISR =====
static void IRAM_ATTR ws_isr(){
  g_wsEdges++;
}

// ===== Source rate monitor task =====
static TaskHandle_t g_rateTask = nullptr;

static void reset_resampler_for(SrcRate r){
  // wyczyść bufor żeby nie mieszać starych próbek w nowym trybie
  rb_clear();
  g_phase_q16 = 0;

  if (r == SRC_48000){
    // step = input_sr / output_sr w Q16
    // (48000/44100) * 65536
    g_step_q16 = (uint32_t)(((uint64_t)48000 << 16) / (uint32_t)OUT_SR);
  } else {
    g_step_q16 = 0;
  }
}

static void rate_task(void *){
  // prosta stabilizacja: wymagamy 2 takich samych odczytów pod rząd
  SrcRate lastDec = SRC_UNKNOWN;
  int stable = 0;

  uint32_t lastEdges = 0;
  uint32_t lastMs = millis();

  for(;;){
    vTaskDelay(pdMS_TO_TICKS(250));

    uint32_t nowMs = millis();
    uint32_t e = g_wsEdges;
    uint32_t de = e - lastEdges;
    uint32_t dt = nowMs - lastMs;
    lastEdges = e;
    lastMs = nowMs;

    if (dt < 50){
      continue;
    }

    // rising edges na WS: 1 na frame => ~ sample rate
    float hz = (float)de * 1000.0f / (float)dt;
    int ihz = (int)(hz + 0.5f);
    g_srcHz = ihz;

    SrcRate dec = SRC_UNKNOWN;
    if (ihz > 43000 && ihz < 45500) dec = SRC_44100;
    else if (ihz > 46500 && ihz < 49500) dec = SRC_48000;
    else dec = SRC_UNKNOWN;

    if (dec == lastDec && dec != SRC_UNKNOWN){
      stable++;
    } else {
      stable = 0;
      lastDec = dec;
    }

    // po 2 stabilnych oknach (czyli ok. 0.5s) przełącz tryb
    if (stable >= 2 && dec != g_srcRate){
      g_srcRate = dec;
      reset_resampler_for(dec);
      logF("EVT SRC_FS %dHz MODE=%s\n",
           g_srcHz,
           (dec==SRC_44100) ? "44100" : (dec==SRC_48000 ? "48000" : "UNKNOWN"));
    }

    // jeśli nie wykrywa nic sensownego przez dłużej, oznacz UNKNOWN
    if (dec == SRC_UNKNOWN){
      // nie czyścimy od razu żeby nie skakało, ale jeśli już jest UNKNOWN, to ok
      // (zostawiamy ostatni tryb, bo niektóre źródła potrafią mieć krótkie dziury)
    }
  }
}

// ===== PCM producer task (I2S -> ring buffer) =====
static TaskHandle_t g_pcmTask = nullptr;

static void pcm_task(void *){
  static uint8_t raw[1024]; // musi być wielokrotnością 4 bajtów (stereo 16-bit)
  for(;;){
    if (!g_btReady || g_mode == MODE_OFF){
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    size_t br = 0;
    // krótki timeout żeby nie przywieszać
    esp_err_t r = i2s_read(I2S_PORT, raw, sizeof(raw), &br, pdMS_TO_TICKS(20));
    if (r != ESP_OK || br == 0){
      continue;
    }

    // br bajtów => frames = br / 4 (L16+R16)
    uint32_t frames = (uint32_t)(br / 4);

    // BOOST (opcjonalny) na surowych próbkach zanim trafią do bufora
    if (g_boost_pct != 100){
      int16_t *s = (int16_t*)raw;
      uint32_t count = br / 2;
      int32_t gain_q10 = (g_boost_pct * 1024) / 100;
      for(uint32_t i=0;i<count;i++){
        int32_t v = (int32_t)s[i] * gain_q10;
        v >>= 10;
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
        s[i] = (int16_t)v;
      }
    }

    rb_push_frames((int16_t*)raw, frames);
  }
}

// ===== EIR name helper =====
static String eirToName(uint8_t *eir){
  if (!eir) return "";
  uint8_t len = 0;
  uint8_t *p = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &len);
  if (!p) p = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &len);
  if (p && len){
    char name[64];
    int n = (len < (sizeof(name)-1)) ? len : (sizeof(name)-1);
    memcpy(name, p, n);
    name[n] = 0;
    return String(name);
  }
  return "";
}

// ===== GAP callback =====
static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
  switch(event){
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
      if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED){
        g_scanning = true;
        logLn("SCAN START");
      } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED){
        g_scanning = false;
        logF("SCAN DONE COUNT=%d\n", g_scanCount);
      }
    } break;

    case ESP_BT_GAP_DISC_RES_EVT: {
      int rssi = 0;
      String name = "";

      for (int i=0; i<param->disc_res.num_prop; i++){
        auto &p = param->disc_res.prop[i];
        if (p.type == ESP_BT_GAP_DEV_PROP_RSSI){
          rssi = *(int8_t*)p.val;
        } else if (p.type == ESP_BT_GAP_DEV_PROP_EIR){
          name = eirToName((uint8_t*)p.val);
        }
      }

      scanStore(param->disc_res.bda, rssi, name);

      String mac = bdaToStr(param->disc_res.bda);
      int idx = scanFindByMac(mac);

      logF("DEV %d %s RSSI=%d NAME=\"%s\"\n",
           (idx >= 0 ? idx : 0),
           mac.c_str(),
           rssi,
           name.c_str());
    } break;

    default:
      break;
  }
}

// ===== A2DP eventy =====
static void on_conn_state(esp_a2d_connection_state_t state, void *){
  const char* s =
    (state==ESP_A2D_CONNECTION_STATE_DISCONNECTED) ? "DISCONNECTED" :
    (state==ESP_A2D_CONNECTION_STATE_CONNECTING)   ? "CONNECTING" :
    (state==ESP_A2D_CONNECTION_STATE_CONNECTED)    ? "CONNECTED" :
    (state==ESP_A2D_CONNECTION_STATE_DISCONNECTING)? "DISCONNECTING" : "UNKNOWN";

  logF("EVT A2DP_CONN %s MAC=%s NAME=\"%s\"\n",
       s,
       g_connMac.length()?g_connMac.c_str():"None",
       g_connName.c_str());

  if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED){
    g_connMac = "";
    g_connName = "";
  }
}

static void on_audio_state(esp_a2d_audio_state_t state, void *){
  const char* s =
    (state==ESP_A2D_AUDIO_STATE_STOPPED)   ? "STOPPED" :
    (state==ESP_A2D_AUDIO_STATE_STARTED)   ? "STARTED" :
    (state==ESP_A2D_AUDIO_STATE_SUSPENDED) ? "SUSPENDED" : "UNKNOWN";

  logF("EVT A2DP_AUDIO %s RB=%lu\n", s, (unsigned long)rb_count_frames());
}

// ===== A2DP data callback =====
// A2DP prosi o len bajtów PCM (stereo 16-bit)
static int32_t get_data(uint8_t *data, int32_t len){
  // jeśli nie nadajemy, to cisza
  if (!g_btReady || g_mode == MODE_OFF){
    memset(data, 0, len);
    return len;
  }

  // ile frames wyjściowych potrzeba
  uint32_t outFrames = (uint32_t)(len / 4);
  int16_t *out = (int16_t*)data;

  SrcRate sr = g_srcRate;

  // jeśli jeszcze nie wykryło, to próbuj zachować się bezpiecznie: cisza
  if (sr == SRC_UNKNOWN){
    memset(data, 0, len);
    return len;
  }

  if (sr == SRC_44100){
    // passthrough: pop tyle frames ile się da, resztę uzupełnij zerami
    uint32_t got = rb_pop_frames(out, outFrames);
    if (got < outFrames){
      uint32_t missing = outFrames - got;
      memset(out + got*2, 0, missing * 4);
    }
    return len;
  }

  // sr == SRC_48000: resampling 48 -> 44.1
  // potrzebujemy dwóch kolejnych frames do interpolacji
  for(uint32_t i=0;i<outFrames;i++){
    // phase_q16 mówi ile wejściowych frames "przeszliśmy"
    uint32_t idxInt = (g_phase_q16 >> 16);
    uint32_t frac   = (g_phase_q16 & 0xFFFF);

    // upewnij się, że w buforze jest idxInt+1
    // najprościej: dropnij idxInt frames, potem peek 2
    if (idxInt > 0){
      rb_drop_frames(idxInt);
      g_phase_q16 &= 0xFFFF; // zostaw tylko frac
    }

    int16_t l0,r0,l1,r1;
    if (!rb_peek_two(l0,r0,l1,r1)){
      // brak danych => cisza
      out[i*2 + 0] = 0;
      out[i*2 + 1] = 0;
      // nie przesuwaj fazy za agresywnie, ale też nie stój w miejscu
      g_phase_q16 += g_step_q16;
      continue;
    }

    // linear interpolation
    int32_t dl = (int32_t)l1 - (int32_t)l0;
    int32_t dr = (int32_t)r1 - (int32_t)r0;

    int32_t l = (int32_t)l0 + ((dl * (int32_t)frac) >> 16);
    int32_t r = (int32_t)r0 + ((dr * (int32_t)frac) >> 16);

    out[i*2 + 0] = (int16_t)l;
    out[i*2 + 1] = (int16_t)r;

    g_phase_q16 += g_step_q16;
  }

  return len;
}

// ===== NVS save/load =====
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

  int32_t m=0, v=100, b=100;
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

// ===== BT start (raz) =====
static void ensureBtStarted(){
  if (g_btReady) return;

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
    nvs_flash_erase();
    nvs_flash_init();
  }

  // WS interrupt (frequency detect)
  pinMode(PIN_I2S_WS, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_I2S_WS), ws_isr, RISING);

  i2s_init_slave_rx();

  // GAP scan + names
  esp_bt_gap_register_callback(gap_cb);

  // A2DP
  a2dp.set_local_name("EVO-BT-TX");
  a2dp.set_data_callback(get_data);
  a2dp.set_auto_reconnect(false);
  a2dp.set_on_connection_state_changed(on_conn_state);
  a2dp.set_on_audio_state_changed(on_audio_state);

  a2dp.start();
  a2dp.set_volume(g_vol_127);

  rb_clear();
  g_srcRate = SRC_UNKNOWN;
  g_srcHz = 0;

  // tasks
  xTaskCreatePinnedToCore(rate_task, "rate_task", 4096, nullptr, 2, &g_rateTask, 1);
  xTaskCreatePinnedToCore(pcm_task,  "pcm_task",  4096, nullptr, 2, &g_pcmTask,  1);

  g_btReady = true;
}

// ===== actions =====
static void status_send(){
  const char* sr =
    (g_srcRate==SRC_44100) ? "44100" :
    (g_srcRate==SRC_48000) ? "48000" : "UNKNOWN";

  logF("STATE BT=%s MODE=%s VOL=%d(127=%d) BOOST=%d SRC=%s(%dHz) RB=%lu SCAN=%d CONN=%d MAC=%s NAME=\"%s\"\n",
    g_btReady ? "ON":"OFF",
    g_mode==MODE_OFF?"OFF":(g_mode==MODE_TX?"TX":"AUTO"),
    g_vol_ui,
    (int)g_vol_127,
    g_boost_pct,
    sr,
    g_srcHz,
    (unsigned long)rb_count_frames(),
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

  int idx = scanFindByMac(mac);
  if (idx >= 0) g_connName = g_scan[idx].name; else g_connName = "";

  bool ok = a2dp.connect_to(bda);
  if (ok){
    g_connMac = mac;
    logF("OK CONNECT %s NAME=\"%s\"\n", mac.c_str(), g_connName.c_str());
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
  g_connName = g_scan[idx].name;
  connect_mac(mac);
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

// ===== DELPAIRED =====
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

// ===== cmd parser =====
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

  if (s=="HARDRESET"){ logLn("OK HARDRESET"); delay(50); ESP.restart(); return; }

  logLn("ERR UNKNOWN");
}

// ===== read line =====
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

  logLn("READY EVO-BT-TX v5");

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
