#include "EQ_FFTAnalyzer.h"
#include "EQ_AnalyzerDisplay.h"  // for analyzerGetPeakHoldTime()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

// ======================= USTAWIENIA (lekkie, bez wpływu na audio) =======================

// Rozmiar ramki do analizy – 256 próbek (mono) daje szybki refresh i mały koszt.
static const uint16_t FRAME_N = 256;

// Downsample: dla 44.1kHz/48kHz bierzemy co 2 próbkę do analizy -> mniej CPU.
// (Skuteczny samplerate = SR / DS)
static uint8_t g_downsample = 2;

// Kolejka próbek: trzymamy tylko mono int16
typedef struct {
  int16_t s[FRAME_N];
  uint16_t n; // zawsze FRAME_N jeśli pełna ramka
} frame_t;

static QueueHandle_t g_q = nullptr;
static TaskHandle_t  g_task = nullptr;

static volatile bool g_enabled = false;
static volatile bool g_runtimeActive = false;
static volatile bool g_testGen = false;

static volatile uint32_t g_sr_hz = 44100;         // wejściowy SR
static volatile uint32_t g_sr_eff = 22050;        // efektywny SR po downsample

// Współczynniki dynamiki/AGC (klucz do "żywego" analizatora bez przesteru)
static float g_ref = 1800.0f;     // adaptacyjna referencja energii (AGC) - wyższa dla kontroli basów
static float g_ref_min = 150.0f;  // minimalna referencja (gating)
static float g_ref_max = 8000.0f; // maksymalna referencja - wyższa dla głośnych fragmentów

// Wygładzanie słupków i peak-hold
static float g_levels[EQ_BANDS] = {0};
static float g_peaks [EQ_BANDS] = {0};
static uint32_t g_peak_timers[EQ_BANDS] = {0}; // timery peak hold w ms

// Statystyka: czy naprawdę dostajemy próbki
static volatile uint64_t g_lastPushUs = 0;
static volatile uint32_t g_samplesPushed = 0;
static volatile uint32_t g_samplesPushedPrev = 0;

// snapshot lock (bardzo lekki)
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// Bufor do składania ramki
static int16_t g_acc[FRAME_N];
static uint16_t g_acc_n = 0;
static uint8_t  g_ds_phase = 0;

// ======================= GOERTZEL (tanie "FFT-like" na pasma) =======================

static inline float clamp01(float x){ return (x < 0.f) ? 0.f : (x > 1.f ? 1.f : x); }

// 16 pasm – logarytmicznie (od ~60 Hz do ~12 kHz przy 22.05 kHz eff)
static const float kBandHz[EQ_BANDS] = {
   60,   90,  140,  220,
  330,  500,  750, 1100,
 1600, 2300, 3300, 4700,
 6600, 8200, 9800, 12000
};

// Zbalansowane tłumienie z bardzo mocno podbitymi wysokimi częstotliwościami
static const float kBandGain[EQ_BANDS] = {
  0.35f, 0.45f, 0.55f, 0.65f,  // Umiarkowanie tłumione basy (60-220Hz)
  0.75f, 1.10f, 1.20f, 1.15f,  // Podbite średnie częst. (330Hz-1.1kHz)
  1.35f, 1.50f, 1.65f, 1.70f,  // Bardzo mocno podbite wyższe częst. (1.6-4.7kHz)  
  1.75f, 1.80f, 1.85f, 1.90f   // Maksymalnie podbite najwyższe (6.6-12kHz)
};

static float goertzel_mag(const int16_t* x, uint16_t n, float freq, float sr_eff){
  // Standard Goertzel magnitude (bez sqrt – wystarczy względnie)
  const float w = 2.0f * (float)M_PI * (freq / sr_eff);
  const float cw = cosf(w);
  const float coeff = 2.0f * cw;

  float q0=0, q1=0, q2=0;
  for(uint16_t i=0;i<n;i++){
    q0 = coeff*q1 - q2 + (float)x[i];
    q2 = q1;
    q1 = q0;
  }
  // energia ~ q1^2 + q2^2 - q1*q2*coeff
  float p = q1*q1 + q2*q2 - q1*q2*coeff;
  if(p < 0) p = 0;
  return p; // bez sqrt – szybciej
}

// ======================= Mapowanie energii -> poziom (dynamika) =======================

static float compress_level(float v){
  // v może być >1. Kompresja logarytmiczna -> dużo "życia" przy cichych fragmentach
  // comp=8 daje sensowną dynamikę na FLAC/AAC
  const float comp = 8.0f;
  float y = log1pf(comp * v) / log1pf(comp);
  return clamp01(y);
}

// ======================= TASK ANALIZATORA (Core1) =======================

static void analyzer_task(void*){
  frame_t fr;
  const TickType_t waitTicks = pdMS_TO_TICKS(25);

  // parametry "fizyki" słupków (style 5/6)
  const float attack = 0.55f;      // szybko rośnie
  const float release = 0.08f;     // wolniej opada
  const float peakFall = 0.012f;   // opadanie peak-hold (wolniejsze)
  const uint32_t peakHoldMs = analyzerGetPeakHoldTime(); // czas zatrzymania peak na szczycie w ms

  while(true){
    // gdy OFF lub nieaktywny runtime -> śpimy, nie dotykamy CPU
    if(!g_enabled || !g_runtimeActive){
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if(xQueueReceive(g_q, &fr, waitTicks) != pdTRUE){
      // brak ramki – luz
      continue;
    }

    // 1) policz energię globalną ramki (ref/AGC)
    float sumAbs = 0.f;
    for(uint16_t i=0;i<FRAME_N;i++){
      sumAbs += fabsf((float)fr.s[i]);
    }
    float refNow = (sumAbs / (float)FRAME_N);

    // AGC: szybciej reaguj na wzrost, wolniej na spadek
    if(refNow > g_ref) g_ref = g_ref + 0.20f*(refNow - g_ref);
    else               g_ref = g_ref + 0.05f*(refNow - g_ref);

    if(g_ref < g_ref_min) g_ref = g_ref_min;
    if(g_ref > g_ref_max) g_ref = g_ref_max;

    // 2) pasma – goertzel
    float raw[EQ_BANDS];
    for(uint8_t b=0;b<EQ_BANDS;b++){
      float p = goertzel_mag(fr.s, FRAME_N, kBandHz[b], (float)g_sr_eff);
      // normalizacja: p rośnie z N i amplitudą^2, więc bierzemy sqrt-ish przez pow^0.5
      // zamiast sqrt: powf(p,0.5f) jest wolne -> szybciej sqrtf, bo jest sprzętowo wspierane.
      float mag = sqrtf(p);

      // przeskaluj względem ref (AGC) i pasma - zwiększona dynamika
      float v = (mag / (g_ref * 220.0f)) * kBandGain[b]; // 220 - maksymalna dynamika
      raw[b] = compress_level(v);
    }

    // 3) wygładzanie + peak hold
    portENTER_CRITICAL(&g_mux);
    for(uint8_t b=0;b<EQ_BANDS;b++){
      float cur = g_levels[b];
      float target = raw[b];

      // attack/release
      if(target > cur) cur = cur + attack*(target - cur);
      else             cur = cur + release*(target - cur);

      cur = clamp01(cur);
      g_levels[b] = cur;

      // peaks z hold time
      float pk = g_peaks[b];
      uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
      
      if(cur > pk) {
        // Nowy peak - ustaw wartość i zresetuj timer
        pk = cur;
        g_peak_timers[b] = now_ms;
      } else {
        // Sprawdź czy peak hold time minął
        if((now_ms - g_peak_timers[b]) > peakHoldMs) {
          // Hold time minął - zaczynaj opadanie
          pk -= peakFall;
          if(pk < cur) pk = cur; // nie opadaj poniżej aktualnego poziomu
        }
        // Jeśli hold time jeszcze nie minął, pk zostaje bez zmian
      }
      
      if(pk < 0) pk = 0;
      g_peaks[b] = pk;
    }
    portEXIT_CRITICAL(&g_mux);
  }
}

// ======================= API =======================

bool eq_analyzer_init(void){
  if(g_q) return true;

  g_q = xQueueCreate(6, sizeof(frame_t)); // mała kolejka = małe opóźnienie
  if(!g_q) return false;

  BaseType_t ok = xTaskCreatePinnedToCore(
    analyzer_task,
    "EQAnalyzer",
    4096,          // stack
    nullptr,
    1,             // niski priorytet
    &g_task,
    1              // Core1 (Core0 zostaje dla audio)
  );
  if(ok != pdPASS){
    vQueueDelete(g_q);
    g_q = nullptr;
    return false;
  }

  eq_analyzer_reset();
  return true;
}

void eq_analyzer_deinit(void){
  if(g_task){
    vTaskDelete(g_task);
    g_task = nullptr;
  }
  if(g_q){
    vQueueDelete(g_q);
    g_q = nullptr;
  }
}

void eq_analyzer_reset(void){
  portENTER_CRITICAL(&g_mux);
  for(uint8_t i=0;i<EQ_BANDS;i++){
    g_levels[i]=0;
    g_peaks[i]=0;
  }
  portEXIT_CRITICAL(&g_mux);
  g_ref = 1200.0f;
  g_lastPushUs = 0;
  g_samplesPushed = 0;
  g_samplesPushedPrev = 0;
  g_acc_n = 0;
  g_ds_phase = 0;
}

void eq_analyzer_set_enabled(bool en){
  g_enabled = en;
  if(!en){
    eq_analyzer_reset();
    // opróżnij kolejkę
    if(g_q) xQueueReset(g_q);
  }
}
bool eq_analyzer_get_enabled(void){ return g_enabled; }

void eq_analyzer_set_runtime_active(bool active){
  g_runtimeActive = active;
  if(!active){
    // żeby nie wisiały stare wartości
    portENTER_CRITICAL(&g_mux);
    for(uint8_t i=0;i<EQ_BANDS;i++){
      g_levels[i] *= 0.7f;
      g_peaks[i]  *= 0.7f;
    }
    portEXIT_CRITICAL(&g_mux);
  }
}

void eq_analyzer_set_sample_rate(uint32_t sample_rate_hz){
  if(sample_rate_hz < 8000) sample_rate_hz = 8000;
  g_sr_hz = sample_rate_hz;

  // downsample: dla 44.1/48k -> 2, dla 96k -> 4
  if(sample_rate_hz >= 88000) g_downsample = 4;
  else if(sample_rate_hz >= 32000) g_downsample = 2;
  else g_downsample = 1;

  g_sr_eff = sample_rate_hz / g_downsample;
}

void eq_analyzer_push_samples_i16(const int16_t* interleavedLR, uint32_t frames){
  // UWAGA: ta funkcja leci z audio path – zero printów, zero malloc, zero heavy math.
  if(!g_enabled) return;
  if(!g_q) return;

  // jeśli nieaktywny runtime – też nie zbieramy (oszczędzamy RAM/CPU)
  if(!g_runtimeActive && !g_testGen) return;

  // mono = (L+R)/2, downsample
  for(uint32_t i=0;i<frames;i++){
    if(g_downsample > 1){
      if(g_ds_phase++ < (g_downsample-1)) continue;
      g_ds_phase = 0;
    }

    int32_t L = interleavedLR[i*2 + 0];
    int32_t R = interleavedLR[i*2 + 1];
    
    // 2% wzmocnienia na wejściu analizatora
    int32_t mono32 = ((L + R) / 2) * 102 / 100; // 1.02x wzmocnienie
    
    // Zabezpieczenie przed przepełnieniem
    if (mono32 > 32767) mono32 = 32767;
    else if (mono32 < -32768) mono32 = -32768;
    
    int16_t m = (int16_t)mono32;

    g_acc[g_acc_n++] = m;
    g_samplesPushed++; // liczymy realnie próbki mono po downsample

    if(g_acc_n >= FRAME_N){
      frame_t fr;
      memcpy(fr.s, g_acc, sizeof(g_acc));
      fr.n = FRAME_N;
      g_acc_n = 0;

      // non-blocking send – jak kolejka pełna, wyrzucamy (bez wpływu na audio)
      xQueueSend(g_q, &fr, 0);
      g_lastPushUs = (uint64_t)esp_timer_get_time();
    }
  }
}

void eq_get_analyzer_levels(float out_levels[EQ_BANDS]){
  portENTER_CRITICAL(&g_mux);
  memcpy(out_levels, g_levels, sizeof(float)*EQ_BANDS);
  portEXIT_CRITICAL(&g_mux);
}

void eq_get_analyzer_peaks(float out_peaks[EQ_BANDS]){
  portENTER_CRITICAL(&g_mux);
  memcpy(out_peaks, g_peaks, sizeof(float)*EQ_BANDS);
  portEXIT_CRITICAL(&g_mux);
}

bool eq_analyzer_is_receiving_samples(void){
  // czy w ostatniej 1.2s były próbki
  uint64_t now = (uint64_t)esp_timer_get_time();
  bool recent = (g_lastPushUs != 0) && ((now - g_lastPushUs) < 1200000ULL);

  // czy sample count rośnie
  uint32_t cur = g_samplesPushed;
  bool inc = (cur != g_samplesPushedPrev);
  g_samplesPushedPrev = cur;

  // wynik: recent OR inc (bo przy ciszy ramki mogą się rzadziej składać)
  return recent || inc;
}

void eq_analyzer_print_diagnostics(void){
  Serial.printf("EQ Analyzer: en=%d runtime=%d sr=%u eff=%u ds=%u ref=%.1f q=%u acc=%u samples=%u\n",
    (int)g_enabled, (int)g_runtimeActive, (unsigned)g_sr_hz, (unsigned)g_sr_eff, (unsigned)g_downsample,
    g_ref,
    g_q ? (unsigned)uxQueueMessagesWaiting(g_q) : 0,
    (unsigned)g_acc_n,
    (unsigned)g_samplesPushed
  );
}

void eq_analyzer_enable_test_generator(bool en){
  g_testGen = en;
  // generator prosty: nie implementuję tu, bo to by dodało kod w audio path.
  // Jeśli chcesz – dopiszemy wersję generującą ramki w analyzer_task (bez audio hook).
}
