#include "EQ_AnalyzerDisplay.h"
#include "EQ_FFTAnalyzer.h"

#include <FS.h>
#include <U8g2lib.h>
#include <time.h>
#include <math.h>

// Storage definitions - same as in main.cpp
#ifdef USE_SD
  #include "SD.h"
  #define STORAGE SD
#else
  #include "SPIFFS.h"
  #define STORAGE SPIFFS
#endif

#ifndef PI
#define PI 3.14159265358979323846
#endif

// External variables from main.cpp
extern String stationName;
extern String stationNameStream;
extern String stationStringWeb;
extern uint8_t volumeValue;
extern bool volumeMute;
extern U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2;

// EQ variables - defined here instead of extern
uint8_t eqLevel[EQ_BANDS] = {0};
uint8_t eqPeak[EQ_BANDS] = {0};

// Zmienne globalne dla konfiguracji stylów
bool eqAnalyzerEnabled = true;
uint8_t eq5_maxSegments = 32;
uint8_t eq_barWidth5 = 10;
uint8_t eq_barGap5 = 6;
uint8_t eq6_maxSegments = 48;
uint8_t eq_barWidth6 = 10;
uint8_t eq_barGap6 = 1;

// Brakujące funkcje pomocnicze
void eq_auto_fit_width(uint8_t style, uint16_t screenWidth) {
  // Automatyczne dopasowanie szerokości słupków
  if (style == 5) {
    uint16_t totalWidth = EQ_BANDS * eq_barWidth5 + (EQ_BANDS - 1) * eq_barGap5;
    if (totalWidth > screenWidth) {
      eq_barWidth5 = (screenWidth - (EQ_BANDS - 1) * eq_barGap5) / EQ_BANDS;
      if (eq_barWidth5 < 2) eq_barWidth5 = 2;
    }
  } else if (style == 6) {
    uint16_t totalWidth = EQ_BANDS * eq_barWidth6 + (EQ_BANDS - 1) * eq_barGap6;
    if (totalWidth > screenWidth) {
      eq_barWidth6 = (screenWidth - (EQ_BANDS - 1) * eq_barGap6) / EQ_BANDS;
      if (eq_barWidth6 < 2) eq_barWidth6 = 2;
    }
  }
}

uint32_t eq_analyzer_get_sample_count() {
  // Zwraca liczbę przetworzonych próbek (mock - można rozszerzyć)
  return millis() / 10; // Prosty licznik
}

// ======================= ZOPTYMALIZOWANE POBIERANIE POZIOMÓW =======================

// Buforowane poziomy dla UI (thread-safe, bez obliczeń FFT)
static float g_ui_levels[EQ_BANDS] = {0};
static float g_ui_peaks[EQ_BANDS] = {0};  
static uint32_t g_last_ui_fetch = 0;
static const uint32_t g_ui_fetch_interval = 25; // min. 25ms między fetch'ami (40Hz max)

// Thread-safe pobieranie gotowych poziomów z analizatora FFT
void eq_ui_fetch_levels() {
  uint32_t now = millis();
  if((now - g_last_ui_fetch) < g_ui_fetch_interval) {
    return; // Zbyt wcześnie na fetch
  }
  g_last_ui_fetch = now;
  
  // Pobierz gotowe poziomy z FFT analizatora (bez obliczeń!)
  eq_get_analyzer_levels(g_ui_levels);
  eq_get_analyzer_peaks(g_ui_peaks);
}

// Bezpieczne pobranie poziomów dla UI (bez mutex - kopia z buforowanych)
void eq_ui_get_levels(float out_levels[EQ_BANDS]) {
  memcpy(out_levels, g_ui_levels, sizeof(float) * EQ_BANDS);
}

void eq_ui_get_peaks(float out_peaks[EQ_BANDS]) {
  memcpy(out_peaks, g_ui_peaks, sizeof(float) * EQ_BANDS);  
}

// Konwersja float (0.0-1.0) do uint8_t (0-255) dla kompatybilności
void eq_ui_get_levels_uint8(uint8_t out_levels[EQ_BANDS]) {
  for(uint8_t i = 0; i < EQ_BANDS; i++) {
    float val = g_ui_levels[i];
    if(val < 0.0f) val = 0.0f;
    if(val > 1.0f) val = 1.0f;
    out_levels[i] = (uint8_t)(val * 255.0f);
  }
}

void eq_ui_get_peaks_uint8(uint8_t out_peaks[EQ_BANDS]) {
  for(uint8_t i = 0; i < EQ_BANDS; i++) {
    float val = g_ui_peaks[i];
    if(val < 0.0f) val = 0.0f;
    if(val > 1.0f) val = 1.0f;
    out_peaks[i] = (uint8_t)(val * 255.0f);
  }
}

static const char* kCfgPath = "/analyzer.cfg";
static AnalyzerStyleCfg g_cfg;

static uint8_t clampU8(int v, int lo, int hi) {
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return (uint8_t)v;
}
static float clampF(float v, float lo, float hi) {
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return v;
}

// Funkcje pomocnicze do obsługi jasności - ordered dithering (Bayer 4x4)
static const uint8_t bayerMatrix4x4[4][4] = {
  {  0, 128,  32, 160},
  {192,  64, 224,  96},
  { 48, 176,  16, 144},
  {240, 112, 208,  80}
};

static bool shouldDrawBarPixel(uint8_t brightness, uint16_t x, uint16_t y) {
  if (brightness >= 255) return true;
  if (brightness == 0) return false;
  // Ordered dithering z matrycą Bayera 4x4 - równomierny wzór bez skośnych linii
  uint8_t threshold = bayerMatrix4x4[y & 3][x & 3];
  return brightness > threshold;
}

static bool shouldDrawPeakPixel(uint8_t brightness, uint16_t x, uint16_t y) {
  if (brightness >= 255) return true;
  if (brightness == 0) return false;
  // Ordered dithering z matrycą Bayera 4x4
  uint8_t threshold = bayerMatrix4x4[y & 3][x & 3];
  return brightness > threshold;
}

AnalyzerStyleCfg analyzerGetStyle() { return g_cfg; }

uint32_t analyzerGetPeakHoldTime() {
  return g_cfg.peakHoldTimeMs;
}

void analyzerSetStyle(const AnalyzerStyleCfg& in)
{
  AnalyzerStyleCfg c = in;

  // Globalne ustawienia - pozwalamy na szybkie wartości (10-2000ms)
  c.peakHoldTimeMs = (c.peakHoldTimeMs < 10) ? 10 : (c.peakHoldTimeMs > 2000) ? 2000 : c.peakHoldTimeMs;

  // Styl 5
  c.s5_barWidth = clampU8(c.s5_barWidth, 2, 30);
  c.s5_barGap   = clampU8(c.s5_barGap,   0, 20);
  c.s5_segments = clampU8(c.s5_segments,  4, 48);
  c.s5_fill     = clampF(c.s5_fill,     0.10f, 1.00f);
  c.s5_segHeight = clampU8(c.s5_segHeight, 1, 4);
  c.s5_smoothness = clampU8(c.s5_smoothness, 10, 90);

  // Styl 6
  c.s6_gap    = clampU8(c.s6_gap,    0, 10);
  c.s6_shrink = clampU8(c.s6_shrink, 0, 5);
  c.s6_fill   = clampF(c.s6_fill,   0.10f, 1.00f);
  c.s6_segMin = clampU8(c.s6_segMin, 4, 48);
  c.s6_segMax = clampU8(c.s6_segMax, 4, 48);
  if (c.s6_segMax < c.s6_segMin) c.s6_segMax = c.s6_segMin;
  c.s6_smoothness = clampU8(c.s6_smoothness, 10, 90);

  // Styl 7
  c.s7_circleRadius = clampU8(c.s7_circleRadius, 1, 8);
  c.s7_circleGap = clampU8(c.s7_circleGap, 1, 8);
  c.s7_maxHeight = clampU8(c.s7_maxHeight, 20, 60);

  // Styl 8
  c.s8_lineThickness = clampU8(c.s8_lineThickness, 1, 5);
  c.s8_lineGap = clampU8(c.s8_lineGap, 0, 8);
  c.s8_maxHeight = clampU8(c.s8_maxHeight, 30, 64);

  // Styl 9
  c.s9_starRadius = clampU8(c.s9_starRadius, 10, 30);
  c.s9_armWidth = clampU8(c.s9_armWidth, 1, 6);
  c.s9_armLength = clampU8(c.s9_armLength, 8, 25);
  c.s9_spikeLength = clampU8(c.s9_spikeLength, 2, 10);
  c.s9_centerSize = clampU8(c.s9_centerSize, 2, 8);
  c.s9_smoothness = clampU8(c.s9_smoothness, 10, 90);

  // Styl 10 - Floating Peaks
  c.s10_barWidth = clampU8(c.s10_barWidth, 4, 20);
  c.s10_barGap = clampU8(c.s10_barGap, 1, 6);
  c.s10_segmentHeight = clampU8(c.s10_segmentHeight, 1, 4);
  c.s10_segmentGap = clampU8(c.s10_segmentGap, 0, 3);
  c.s10_maxPeaks = clampU8(c.s10_maxPeaks, 1, 5);
  c.s10_peakHoldTime = clampU8(c.s10_peakHoldTime, 0, 100);
  c.s10_peakFloatSpeed = clampU8(c.s10_peakFloatSpeed, 3, 15);
  c.s10_peakFadeSteps = clampU8(c.s10_peakFadeSteps, 5, 20);
  c.s10_trailLength = clampU8(c.s10_trailLength, 3, 12);
  c.s10_smoothness = clampU8(c.s10_smoothness, 10, 90);
  c.s10_barBrightness = clampU8(c.s10_barBrightness, 50, 255);
  c.s10_peakBrightness = clampU8(c.s10_peakBrightness, 100, 255);
  c.s10_trailBrightness = clampU8(c.s10_trailBrightness, 80, 255);
  c.s10_peakMinHeight = clampU8(c.s10_peakMinHeight, 1, 8);
  c.s10_floatHeight = clampU8(c.s10_floatHeight, 8, 25);

  g_cfg = c;

  // Mapuj konfigurację na nasze zmienne globalne
  eq5_maxSegments = g_cfg.s5_segments;
  eq_barWidth5 = g_cfg.s5_barWidth;
  eq_barGap5 = g_cfg.s5_barGap;
  
  eq6_maxSegments = g_cfg.s6_segMax;
  eq_barWidth6 = g_cfg.s6_width;  // używa konfigurowalnej szerokości słupka
  eq_barGap6 = g_cfg.s6_gap;
}

static bool parseLineKV(const String& line, String& k, String& v) {
  int eq = line.indexOf('=');
  if (eq <= 0) return false;
  k = line.substring(0, eq); k.trim();
  v = line.substring(eq+1);  v.trim();
  return k.length() > 0;
}

void analyzerStyleLoad()
{
  // default
  g_cfg = AnalyzerStyleCfg();
  eq6_maxSegments = g_cfg.s6_segMax;

  File f = STORAGE.open(kCfgPath, FILE_READ);
  if (!f) return;

  AnalyzerStyleCfg c = g_cfg;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length() || line.startsWith("#")) continue;

    String k, v;
    if (!parseLineKV(line, k, v)) continue;

    // Globalne ustawienia
    if (k == "peakHoldMs")  c.peakHoldTimeMs = (uint16_t)v.toInt();
    
    // Styl 5
    if (k == "s5w")         c.s5_barWidth = (uint8_t)v.toInt();
    else if (k == "s5g")    c.s5_barGap   = (uint8_t)v.toInt();
    else if (k == "s5seg")  c.s5_segments = (uint8_t)v.toInt();
    else if (k == "s5fill") c.s5_fill     = v.toFloat();
    else if (k == "s5segH") c.s5_segHeight = (uint8_t)v.toInt();
    else if (k == "s5peaks") c.s5_showPeaks = v.toInt() != 0;
    else if (k == "s5smooth") c.s5_smoothness = (uint8_t)v.toInt();
    else if (k == "s5barBrightness") c.s5_barBrightness = (uint8_t)v.toInt();
    else if (k == "s5peakBrightness") c.s5_peakBrightness = (uint8_t)v.toInt();

    // Styl 6
    else if (k == "s6w")    c.s6_width    = (uint8_t)v.toInt();
    else if (k == "s6g")    c.s6_gap      = (uint8_t)v.toInt();
    else if (k == "s6sh")   c.s6_shrink   = (uint8_t)v.toInt();
    else if (k == "s6fill") c.s6_fill     = v.toFloat();
    else if (k == "s6min")  c.s6_segMin   = (uint8_t)v.toInt();
    else if (k == "s6max")  c.s6_segMax   = (uint8_t)v.toInt();
    else if (k == "s6peaks") c.s6_showPeaks = v.toInt() != 0;
    else if (k == "s6smooth") c.s6_smoothness = (uint8_t)v.toInt();
    else if (k == "s6barBrightness") c.s6_barBrightness = (uint8_t)v.toInt();
    else if (k == "s6peakBrightness") c.s6_peakBrightness = (uint8_t)v.toInt();

    // Styl 7
    else if (k == "s7radius") c.s7_circleRadius = (uint8_t)v.toInt();
    else if (k == "s7gap")   c.s7_circleGap = (uint8_t)v.toInt();
    else if (k == "s7filled") c.s7_filled = v.toInt() != 0;
    else if (k == "s7max")   c.s7_maxHeight = (uint8_t)v.toInt();

    // Styl 8
    else if (k == "s8thick") c.s8_lineThickness = (uint8_t)v.toInt();
    else if (k == "s8gap")   c.s8_lineGap = (uint8_t)v.toInt();
    else if (k == "s8grad")  c.s8_gradient = v.toInt() != 0;
    else if (k == "s8max")   c.s8_maxHeight = (uint8_t)v.toInt();
    
    // Styl 9
    else if (k == "s9radius") c.s9_starRadius = (uint8_t)v.toInt();
    else if (k == "s9armw")   c.s9_armWidth = (uint8_t)v.toInt();
    else if (k == "s9arml")   c.s9_armLength = (uint8_t)v.toInt();
    else if (k == "s9spike")  c.s9_spikeLength = (uint8_t)v.toInt();
    else if (k == "s9spikes") c.s9_showSpikes = v.toInt() != 0;
    else if (k == "s9filled") c.s9_filled = v.toInt() != 0;
    else if (k == "s9center") c.s9_centerSize = (uint8_t)v.toInt();
    else if (k == "s9smooth") c.s9_smoothness = (uint8_t)v.toInt();

    // Styl 10 - Floating Peaks
    else if (k == "s10barw")     c.s10_barWidth = (uint8_t)v.toInt();
    else if (k == "s10gap")      c.s10_barGap = (uint8_t)v.toInt();
    else if (k == "s10segh")     c.s10_segmentHeight = (uint8_t)v.toInt();
    else if (k == "s10segg")     c.s10_segmentGap = (uint8_t)v.toInt();
    else if (k == "s10maxp")     c.s10_maxPeaks = (uint8_t)v.toInt();
    else if (k == "s10hold")     c.s10_peakHoldTime = (uint16_t)v.toInt();
    else if (k == "s10speed")    c.s10_peakFloatSpeed = (uint8_t)v.toInt();
    else if (k == "s10fade")     c.s10_peakFadeSteps = (uint8_t)v.toInt();
    else if (k == "s10trail")    c.s10_trailLength = (uint8_t)v.toInt();
    else if (k == "s10trails")   c.s10_showTrails = v.toInt() != 0;
    else if (k == "s10smooth")   c.s10_smoothness = (uint8_t)v.toInt();
    else if (k == "s10barbr")    c.s10_barBrightness = (uint8_t)v.toInt();
    else if (k == "s10peakbr")   c.s10_peakBrightness = (uint8_t)v.toInt();
    else if (k == "s10trailbr")  c.s10_trailBrightness = (uint8_t)v.toInt();
    else if (k == "s10minh")     c.s10_peakMinHeight = (uint8_t)v.toInt();
    else if (k == "s10floath")   c.s10_floatHeight = (uint8_t)v.toInt();
    else if (k == "s10anim")     c.s10_enableAnimation = v.toInt() != 0;
  }
  f.close();

  analyzerSetStyle(c);
}

void analyzerStyleSave()
{
  File f = STORAGE.open(kCfgPath, FILE_WRITE);
  if (!f) return;

  f.println("# Analyzer style cfg");
  f.println("# Global settings");
  f.printf("peakHoldMs=%u\n", g_cfg.peakHoldTimeMs);
  f.println("# Style5");
  f.printf("s5w=%u\n", g_cfg.s5_barWidth);
  f.printf("s5g=%u\n", g_cfg.s5_barGap);
  f.printf("s5seg=%u\n", g_cfg.s5_segments);
  f.printf("s5fill=%.3f\n", g_cfg.s5_fill);
  f.printf("s5segH=%u\n", g_cfg.s5_segHeight);
  f.printf("s5peaks=%u\n", g_cfg.s5_showPeaks ? 1 : 0);
  f.printf("s5smooth=%u\n", g_cfg.s5_smoothness);
  f.printf("s5barBrightness=%u\n", g_cfg.s5_barBrightness);
  f.printf("s5peakBrightness=%u\n", g_cfg.s5_peakBrightness);

  f.println("# Style6");
  f.printf("s6w=%u\n", g_cfg.s6_width);
  f.printf("s6g=%u\n", g_cfg.s6_gap);
  f.printf("s6sh=%u\n", g_cfg.s6_shrink);
  f.printf("s6fill=%.3f\n", g_cfg.s6_fill);
  f.printf("s6min=%u\n", g_cfg.s6_segMin);
  f.printf("s6max=%u\n", g_cfg.s6_segMax);
  f.printf("s6peaks=%u\n", g_cfg.s6_showPeaks ? 1 : 0);
  f.printf("s6smooth=%u\n", g_cfg.s6_smoothness);
  f.printf("s6barBrightness=%u\n", g_cfg.s6_barBrightness);
  f.printf("s6peakBrightness=%u\n", g_cfg.s6_peakBrightness);

  f.println("# Style7");
  f.printf("s7radius=%u\n", g_cfg.s7_circleRadius);
  f.printf("s7gap=%u\n", g_cfg.s7_circleGap);
  f.printf("s7filled=%u\n", g_cfg.s7_filled ? 1 : 0);
  f.printf("s7max=%u\n", g_cfg.s7_maxHeight);

  f.println("# Style8");
  f.printf("s8thick=%u\n", g_cfg.s8_lineThickness);
  f.printf("s8gap=%u\n", g_cfg.s8_lineGap);
  f.printf("s8grad=%u\n", g_cfg.s8_gradient ? 1 : 0);
  f.printf("s8max=%u\n", g_cfg.s8_maxHeight);
  
  f.println("# Style9");
  f.printf("s9radius=%u\n", g_cfg.s9_starRadius);
  f.printf("s9armw=%u\n", g_cfg.s9_armWidth);
  f.printf("s9arml=%u\n", g_cfg.s9_armLength);
  f.printf("s9spike=%u\n", g_cfg.s9_spikeLength);
  f.printf("s9spikes=%u\n", g_cfg.s9_showSpikes ? 1 : 0);
  f.printf("s9filled=%u\n", g_cfg.s9_filled ? 1 : 0);
  f.printf("s9center=%u\n", g_cfg.s9_centerSize);
  f.printf("s9smooth=%u\n", g_cfg.s9_smoothness);

  // Styl 10 - Floating Peaks
  f.printf("s10barw=%u\n", g_cfg.s10_barWidth);
  f.printf("s10gap=%u\n", g_cfg.s10_barGap);
  f.printf("s10segh=%u\n", g_cfg.s10_segmentHeight);
  f.printf("s10segg=%u\n", g_cfg.s10_segmentGap);
  f.printf("s10maxp=%u\n", g_cfg.s10_maxPeaks);
  f.printf("s10hold=%u\n", g_cfg.s10_peakHoldTime);
  f.printf("s10speed=%u\n", g_cfg.s10_peakFloatSpeed);
  f.printf("s10fade=%u\n", g_cfg.s10_peakFadeSteps);
  f.printf("s10trail=%u\n", g_cfg.s10_trailLength);
  f.printf("s10trails=%u\n", g_cfg.s10_showTrails ? 1 : 0);
  f.printf("s10smooth=%u\n", g_cfg.s10_smoothness);
  f.printf("s10barbr=%u\n", g_cfg.s10_barBrightness);
  f.printf("s10peakbr=%u\n", g_cfg.s10_peakBrightness);
  f.printf("s10trailbr=%u\n", g_cfg.s10_trailBrightness);
  f.printf("s10minh=%u\n", g_cfg.s10_peakMinHeight);
  f.printf("s10floath=%u\n", g_cfg.s10_floatHeight);
  f.printf("s10anim=%u\n", g_cfg.s10_enableAnimation ? 1 : 0);
  
  f.close();
}

// Zmienne dla statusu wczytywania
static String g_loadStatus = "NIEZAINICJALIZOWANY";
static int g_loadedParamsCount = 0;

// Funkcja zwracająca konfigurację jako String (do zapisu w main.cpp)
String analyzerStyleToSaveString()
{
  String s;
  s.reserve(2000);
  s += "# Analyzer style cfg\n";
  s += "# Global settings\n";
  s += "peakHoldMs=" + String(g_cfg.peakHoldTimeMs) + "\n";
  s += "# Style5\n";
  s += "s5w=" + String(g_cfg.s5_barWidth) + "\n";
  s += "s5g=" + String(g_cfg.s5_barGap) + "\n";
  s += "s5seg=" + String(g_cfg.s5_segments) + "\n";
  s += "s5fill=" + String(g_cfg.s5_fill, 3) + "\n";
  s += "s5segH=" + String(g_cfg.s5_segHeight) + "\n";
  s += "s5peaks=" + String(g_cfg.s5_showPeaks ? 1 : 0) + "\n";
  s += "s5smooth=" + String(g_cfg.s5_smoothness) + "\n";
  s += "s5barBrightness=" + String(g_cfg.s5_barBrightness) + "\n";
  s += "s5peakBrightness=" + String(g_cfg.s5_peakBrightness) + "\n";
  
  s += "# Style6\n";
  s += "s6w=" + String(g_cfg.s6_width) + "\n";
  s += "s6g=" + String(g_cfg.s6_gap) + "\n";
  s += "s6sh=" + String(g_cfg.s6_shrink) + "\n";
  s += "s6fill=" + String(g_cfg.s6_fill, 3) + "\n";
  s += "s6min=" + String(g_cfg.s6_segMin) + "\n";
  s += "s6max=" + String(g_cfg.s6_segMax) + "\n";
  s += "s6peaks=" + String(g_cfg.s6_showPeaks ? 1 : 0) + "\n";
  s += "s6smooth=" + String(g_cfg.s6_smoothness) + "\n";
  s += "s6barBrightness=" + String(g_cfg.s6_barBrightness) + "\n";
  s += "s6peakBrightness=" + String(g_cfg.s6_peakBrightness) + "\n";
  
  s += "# Style7\n";
  s += "s7radius=" + String(g_cfg.s7_circleRadius) + "\n";
  s += "s7gap=" + String(g_cfg.s7_circleGap) + "\n";
  s += "s7filled=" + String(g_cfg.s7_filled ? 1 : 0) + "\n";
  s += "s7max=" + String(g_cfg.s7_maxHeight) + "\n";
  
  s += "# Style8\n";
  s += "s8thick=" + String(g_cfg.s8_lineThickness) + "\n";
  s += "s8gap=" + String(g_cfg.s8_lineGap) + "\n";
  s += "s8grad=" + String(g_cfg.s8_gradient ? 1 : 0) + "\n";
  s += "s8max=" + String(g_cfg.s8_maxHeight) + "\n";
  
  s += "# Style9\n";
  s += "s9radius=" + String(g_cfg.s9_starRadius) + "\n";
  s += "s9armw=" + String(g_cfg.s9_armWidth) + "\n";
  s += "s9arml=" + String(g_cfg.s9_armLength) + "\n";
  s += "s9spike=" + String(g_cfg.s9_spikeLength) + "\n";
  s += "s9spikes=" + String(g_cfg.s9_showSpikes ? 1 : 0) + "\n";
  s += "s9filled=" + String(g_cfg.s9_filled ? 1 : 0) + "\n";
  s += "s9center=" + String(g_cfg.s9_centerSize) + "\n";
  s += "s9smooth=" + String(g_cfg.s9_smoothness) + "\n";
  
  s += "# Style10 - Floating Peaks\n";
  s += "s10barw=" + String(g_cfg.s10_barWidth) + "\n";
  s += "s10gap=" + String(g_cfg.s10_barGap) + "\n";
  s += "s10segh=" + String(g_cfg.s10_segmentHeight) + "\n";
  s += "s10segg=" + String(g_cfg.s10_segmentGap) + "\n";
  s += "s10maxp=" + String(g_cfg.s10_maxPeaks) + "\n";
  s += "s10hold=" + String(g_cfg.s10_peakHoldTime) + "\n";
  s += "s10speed=" + String(g_cfg.s10_peakFloatSpeed) + "\n";
  s += "s10fade=" + String(g_cfg.s10_peakFadeSteps) + "\n";
  s += "s10trail=" + String(g_cfg.s10_trailLength) + "\n";
  s += "s10trails=" + String(g_cfg.s10_showTrails ? 1 : 0) + "\n";
  s += "s10smooth=" + String(g_cfg.s10_smoothness) + "\n";
  s += "s10barbr=" + String(g_cfg.s10_barBrightness) + "\n";
  s += "s10peakbr=" + String(g_cfg.s10_peakBrightness) + "\n";
  s += "s10trailbr=" + String(g_cfg.s10_trailBrightness) + "\n";
  s += "s10minh=" + String(g_cfg.s10_peakMinHeight) + "\n";
  s += "s10floath=" + String(g_cfg.s10_floatHeight) + "\n";
  s += "s10anim=" + String(g_cfg.s10_enableAnimation ? 1 : 0) + "\n";
  
  return s;
}

// Funkcja parsująca konfigurację z String (dla main.cpp)
void analyzerStyleLoadFromString(const String& content)
{
  if (content.length() == 0) {
    g_loadStatus = "BŁĄD: Pusty content";
    g_loadedParamsCount = 0;
    return;
  }

  g_loadedParamsCount = 0;
  g_loadStatus = "OK: Rozpoczynam parsowanie";
  
  // default
  AnalyzerStyleCfg c = g_cfg;
  
  // Parsuj linia po linii
  int startPos = 0;
  while (startPos < content.length()) {
    int endPos = content.indexOf('\n', startPos);
    if (endPos == -1) endPos = content.length();
    
    String line = content.substring(startPos, endPos);
    line.trim();
    
    if (line.length() > 0 && !line.startsWith("#")) {
      String k, v;
      if (parseLineKV(line, k, v)) {
        g_loadedParamsCount++;
        
        // Globalne ustawienia
        if (k == "peakHoldMs")  c.peakHoldTimeMs = (uint16_t)v.toInt();
        
        // Styl 5
        else if (k == "s5w")         c.s5_barWidth = (uint8_t)v.toInt();
        else if (k == "s5g")         c.s5_barGap   = (uint8_t)v.toInt();
        else if (k == "s5seg")       c.s5_segments = (uint8_t)v.toInt();
        else if (k == "s5fill")      c.s5_fill     = v.toFloat();
        else if (k == "s5segH")      c.s5_segHeight = (uint8_t)v.toInt();
        else if (k == "s5peaks")     c.s5_showPeaks = v.toInt() != 0;
        else if (k == "s5smooth")    c.s5_smoothness = (uint8_t)v.toInt();
        else if (k == "s5barBrightness") c.s5_barBrightness = (uint8_t)v.toInt();
        else if (k == "s5peakBrightness") c.s5_peakBrightness = (uint8_t)v.toInt();

        // Styl 6
        else if (k == "s6w")    c.s6_width    = (uint8_t)v.toInt();
        else if (k == "s6g")    c.s6_gap      = (uint8_t)v.toInt();
        else if (k == "s6sh")   c.s6_shrink   = (uint8_t)v.toInt();
        else if (k == "s6fill") c.s6_fill     = v.toFloat();
        else if (k == "s6min")  c.s6_segMin   = (uint8_t)v.toInt();
        else if (k == "s6max")  c.s6_segMax   = (uint8_t)v.toInt();
        else if (k == "s6peaks") c.s6_showPeaks = v.toInt() != 0;
        else if (k == "s6smooth") c.s6_smoothness = (uint8_t)v.toInt();
        else if (k == "s6barBrightness") c.s6_barBrightness = (uint8_t)v.toInt();
        else if (k == "s6peakBrightness") c.s6_peakBrightness = (uint8_t)v.toInt();

        // Styl 7
        /*
        else if (k == "s7radius") c.s7_circleRadius = (uint8_t)v.toInt();
        else if (k == "s7gap")   c.s7_circleGap = (uint8_t)v.toInt();
        else if (k == "s7filled") c.s7_filled = v.toInt() != 0;
        else if (k == "s7max")   c.s7_maxHeight = (uint8_t)v.toInt();//

        // Styl 8
        else if (k == "s8thick") c.s8_lineThickness = (uint8_t)v.toInt();
        else if (k == "s8gap")   c.s8_lineGap = (uint8_t)v.toInt();
        else if (k == "s8grad")  c.s8_gradient = v.toInt() != 0;
        else if (k == "s8max")   c.s8_maxHeight = (uint8_t)v.toInt();
        
        // Styl 9
        else if (k == "s9radius") c.s9_starRadius = (uint8_t)v.toInt();
        else if (k == "s9armw")   c.s9_armWidth = (uint8_t)v.toInt();
        else if (k == "s9arml")   c.s9_armLength = (uint8_t)v.toInt();
        else if (k == "s9spike")  c.s9_spikeLength = (uint8_t)v.toInt();
        else if (k == "s9spikes") c.s9_showSpikes = v.toInt() != 0;
        else if (k == "s9filled") c.s9_filled = v.toInt() != 0;
        else if (k == "s9center") c.s9_centerSize = (uint8_t)v.toInt();
        else if (k == "s9smooth") c.s9_smoothness = (uint8_t)v.toInt();
/*
        // Styl 10 - Floating Peaks
        else if (k == "s10barw")     c.s10_barWidth = (uint8_t)v.toInt();
        else if (k == "s10gap")      c.s10_barGap = (uint8_t)v.toInt();
        else if (k == "s10segh")     c.s10_segmentHeight = (uint8_t)v.toInt();
        else if (k == "s10segg")     c.s10_segmentGap = (uint8_t)v.toInt();
        else if (k == "s10maxp")     c.s10_maxPeaks = (uint8_t)v.toInt();
        else if (k == "s10hold")     c.s10_peakHoldTime = (uint16_t)v.toInt();
        else if (k == "s10speed")    c.s10_peakFloatSpeed = (uint8_t)v.toInt();
        else if (k == "s10fade")     c.s10_peakFadeSteps = (uint8_t)v.toInt();
        else if (k == "s10trail")    c.s10_trailLength = (uint8_t)v.toInt();
        else if (k == "s10trails")   c.s10_showTrails = v.toInt() != 0;
        else if (k == "s10smooth")   c.s10_smoothness = (uint8_t)v.toInt();
        else if (k == "s10barbr")    c.s10_barBrightness = (uint8_t)v.toInt();
        else if (k == "s10peakbr")   c.s10_peakBrightness = (uint8_t)v.toInt();
        else if (k == "s10trailbr")  c.s10_trailBrightness = (uint8_t)v.toInt();
        else if (k == "s10minh")     c.s10_peakMinHeight = (uint8_t)v.toInt();
        else if (k == "s10floath")   c.s10_floatHeight = (uint8_t)v.toInt();
        else if (k == "s10anim")     c.s10_enableAnimation = v.toInt() != 0;
      }
    }
    
    startPos = endPos + 1;
  }
  
  analyzerSetStyle(c);
  g_loadStatus = "OK: Załadowano " + String(g_loadedParamsCount) + " parametrów";
}

// Funkcja zwracająca status wczytania
String analyzerGetLoadStatus()
{
  return g_loadStatus;
}

// Funkcja zwracająca liczbę załadowanych parametrów
int analyzerGetLoadedParamsCount()
{
  return g_loadedParamsCount;
}

String analyzerStyleToJson()
{
  String s;
  s.reserve(500);
  s += "{";
  // Globalne ustawienia
  s += "\"peakHoldTimeMs\":" + String(g_cfg.peakHoldTimeMs) + ",";
  // Styl 5
  s += "\"s5_barWidth\":" + String(g_cfg.s5_barWidth) + ",";
  s += "\"s5_barGap\":"   + String(g_cfg.s5_barGap) + ",";
  s += "\"s5_segments\":" + String(g_cfg.s5_segments) + ",";
  s += "\"s5_fill\":"     + String(g_cfg.s5_fill, 3) + ",";
  s += "\"s5_showPeaks\":" + String(g_cfg.s5_showPeaks ? "true" : "false") + ",";
  s += "\"s5_smoothness\":" + String(g_cfg.s5_smoothness) + ",";

  // Styl 6
  s += "\"s6_gap\":"      + String(g_cfg.s6_gap) + ",";
  s += "\"s6_shrink\":"   + String(g_cfg.s6_shrink) + ",";
  s += "\"s6_fill\":"     + String(g_cfg.s6_fill, 3) + ",";
  s += "\"s6_segMin\":"   + String(g_cfg.s6_segMin) + ",";
  s += "\"s6_segMax\":"   + String(g_cfg.s6_segMax) + ",";
  s += "\"s6_showPeaks\":" + String(g_cfg.s6_showPeaks ? "true" : "false") + ",";
  s += "\"s6_smoothness\":" + String(g_cfg.s6_smoothness) + ",";

  // Styl 7
  s += "\"s7_circleRadius\":" + String(g_cfg.s7_circleRadius) + ",";
  s += "\"s7_circleGap\":" + String(g_cfg.s7_circleGap) + ",";
  s += "\"s7_filled\":" + String(g_cfg.s7_filled ? "true" : "false") + ",";
  s += "\"s7_maxHeight\":" + String(g_cfg.s7_maxHeight) + ",";

  // Styl 8
  s += "\"s8_lineThickness\":" + String(g_cfg.s8_lineThickness) + ",";
  s += "\"s8_lineGap\":" + String(g_cfg.s8_lineGap) + ",";
  s += "\"s8_gradient\":" + String(g_cfg.s8_gradient ? "true" : "false") + ",";
  s += "\"s8_maxHeight\":" + String(g_cfg.s8_maxHeight) + ",";
  
  // Styl 9
  s += "\"s9_starRadius\":" + String(g_cfg.s9_starRadius) + ",";
  s += "\"s9_armWidth\":" + String(g_cfg.s9_armWidth) + ",";
  s += "\"s9_armLength\":" + String(g_cfg.s9_armLength) + ",";
  s += "\"s9_spikeLength\":" + String(g_cfg.s9_spikeLength) + ",";
  s += "\"s9_showSpikes\":" + String(g_cfg.s9_showSpikes ? "true" : "false") + ",";
  s += "\"s9_filled\":" + String(g_cfg.s9_filled ? "true" : "false") + ",";
  s += "\"s9_centerSize\":" + String(g_cfg.s9_centerSize) + ",";
  s += "\"s9_smoothness\":" + String(g_cfg.s9_smoothness);
  s += "}";
  return s;
}

static String htmlHeader(const char* title)
{
  String s;
  s.reserve(6000);
  s += "<!doctype html><html><head><meta charset='utf-8'>";
  s += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  s += "<title>"; s += title; s += "</title>";
  s += "<style>";
  s += "body{font-family:Arial;margin:14px;max-width:900px}";
  s += "h2{margin:8px 0 12px 0}";
  s += ".box{border:1px solid #ddd;border-radius:10px;padding:12px;margin:10px 0}";
  s += ".row{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:8px 0}";
  s += "label{min-width:120px} input{width:90px;padding:6px}";
  s += "button{padding:10px 12px;border:0;border-radius:10px;background:#222;color:#fff;cursor:pointer;margin:2px}";
  s += "button.secondary{background:#555} a{color:#0a58ca}";
  s += ".preset-form{display:inline-block;margin:2px}";
  s += "</style></head><body>";
  return s;
}

String analyzerBuildHtmlPage()
{
  String s = htmlHeader("Analyzer / Style 5-9");
  s += "<h2>Analizator – ustawienia stylów 5, 6, 7, 8, 9</h2>";
  s += "<p>Tu regulujesz WYGLĄD słupków. Same słupki ruszą dopiero, gdy w /config zaznaczysz <b>FFT analyzer</b>.</p>";

  s += "<div class='box'><h3>Ustawienia globalne</h3>";
  s += "<div class='row'><label>Peak hold time (ms)</label><input name='peakHoldMs' type='number' min='10' max='2000' step='10' value='" + String(g_cfg.peakHoldTimeMs) + "'></div>";
  s += "</div>";

  // Presety na górze
  s += "<div class='box'><h3>Presety</h3>";
  s += "<div class='row'>";
  s += "<form method='POST' action='/analyzerPreset' class='preset-form'><button name='preset' value='0' type='submit'>Klasyczny</button></form>";
  s += "<form method='POST' action='/analyzerPreset' class='preset-form'><button name='preset' value='1' type='submit'>Nowoczesny</button></form>";
  s += "<form method='POST' action='/analyzerPreset' class='preset-form'><button name='preset' value='2' type='submit'>Kompaktowy</button></form>";
  s += "<form method='POST' action='/analyzerPreset' class='preset-form'><button name='preset' value='3' type='submit'>Retro</button></form>";
  s += "<form method='POST' action='/analyzerPreset' class='preset-form'><button name='preset' value='4' type='submit'>Floating Peaks</button></form>";
  s += "</div></div>";

  s += "<form method='POST'>";

  s += "<div class='box'><h3>Styl 5 (Słupkowy z segmentami)</h3>";
  s += "<div class='row'><label>bar width</label><input name='s5w' type='number' min='2' max='30' value='" + String(g_cfg.s5_barWidth) + "'></div>";
  s += "<div class='row'><label>bar gap</label><input name='s5g' type='number' min='0' max='20' value='" + String(g_cfg.s5_barGap) + "'></div>";
  s += "<div class='row'><label>segments</label><input name='s5seg' type='number' min='4' max='48' value='" + String(g_cfg.s5_segments) + "'></div>";
  s += "<div class='row'><label>segment height</label><input name='s5segH' type='number' min='1' max='4' value='" + String(g_cfg.s5_segHeight) + "'></div>";
  s += "<div class='row'><label>fill (0.1..1)</label><input step='0.05' name='s5fill' type='number' min='0.1' max='1.0' value='" + String(g_cfg.s5_fill,2) + "'></div>";
  s += "<div class='row'><label>show peaks</label><input name='s5peaks' type='checkbox' value='1' " + String(g_cfg.s5_showPeaks ? "checked" : "") + "></div>";
  s += "<div class='row'><label>smoothness</label><input name='s5smooth' type='number' min='10' max='90' value='" + String(g_cfg.s5_smoothness) + "'></div>";
  s += "<div class='row'><label>bar brightness</label><input name='s5barBrightness' type='number' min='0' max='255' value='" + String(g_cfg.s5_barBrightness) + "'></div>";
  s += "<div class='row'><label>peak brightness</label><input name='s5peakBrightness' type='number' min='0' max='255' value='" + String(g_cfg.s5_peakBrightness) + "'></div>";
  s += "</div>";

  s += "<div class='box'><h3>Styl 6 (Cienkie kreski)</h3>";
  s += "<div class='row'><label>bar width</label><input name='s6w' type='number' min='4' max='20' value='" + String(g_cfg.s6_width) + "'></div>";
  s += "<div class='row'><label>gap</label><input name='s6g' type='number' min='0' max='10' value='" + String(g_cfg.s6_gap) + "'></div>";
  s += "<div class='row'><label>shrink</label><input name='s6sh' type='number' min='0' max='5' value='" + String(g_cfg.s6_shrink) + "'></div>";
  s += "<div class='row'><label>fill (0.1..1)</label><input step='0.05' name='s6fill' type='number' min='0.1' max='1.0' value='" + String(g_cfg.s6_fill,2) + "'></div>";
  s += "<div class='row'><label>seg min</label><input name='s6min' type='number' min='4' max='48' value='" + String(g_cfg.s6_segMin) + "'></div>";
  s += "<div class='row'><label>seg max</label><input name='s6max' type='number' min='4' max='48' value='" + String(g_cfg.s6_segMax) + "'></div>";
  s += "<div class='row'><label>show peaks</label><input name='s6peaks' type='checkbox' value='1' " + String(g_cfg.s6_showPeaks ? "checked" : "") + "></div>";
  s += "<div class='row'><label>smoothness</label><input name='s6smooth' type='number' min='10' max='90' value='" + String(g_cfg.s6_smoothness) + "'></div>";
  s += "<div class='row'><label>bar brightness</label><input name='s6barBrightness' type='number' min='0' max='255' value='" + String(g_cfg.s6_barBrightness) + "'></div>";
  s += "<div class='row'><label>peak brightness</label><input name='s6peakBrightness' type='number' min='0' max='255' value='" + String(g_cfg.s6_peakBrightness) + "'></div>";
  s += "</div>";

  s += "<div class='box'><h3>Styl 7 (Okrągły)</h3>";
  s += "<div class='row'><label>circle radius</label><input name='s7radius' type='number' min='1' max='8' value='" + String(g_cfg.s7_circleRadius) + "'></div>";
  s += "<div class='row'><label>circle gap</label><input name='s7gap' type='number' min='1' max='8' value='" + String(g_cfg.s7_circleGap) + "'></div>";
  s += "<div class='row'><label>filled circles</label><input name='s7filled' type='checkbox' value='1' " + String(g_cfg.s7_filled ? "checked" : "") + "></div>";
  s += "<div class='row'><label>max height</label><input name='s7max' type='number' min='20' max='60' value='" + String(g_cfg.s7_maxHeight) + "'></div>";
  s += "</div>";

  s += "<div class='box'><h3>Styl 8 (Liniowy)</h3>";
  s += "<div class='row'><label>line thickness</label><input name='s8thick' type='number' min='1' max='5' value='" + String(g_cfg.s8_lineThickness) + "'></div>";
  s += "<div class='row'><label>line gap</label><input name='s8gap' type='number' min='0' max='8' value='" + String(g_cfg.s8_lineGap) + "'></div>";
  s += "<div class='row'><label>gradient effect</label><input name='s8grad' type='checkbox' value='1' " + String(g_cfg.s8_gradient ? "checked" : "") + "></div>";
  s += "<div class='row'><label>max height</label><input name='s8max' type='number' min='30' max='64' value='" + String(g_cfg.s8_maxHeight) + "'></div>";
  s += "</div>";

  s += "<div class='box'><h3>Styl 9 (Spadające gwiazdki)</h3>";
  s += "<div class='row'><label>unused (radius)</label><input name='s9radius' type='number' min='10' max='30' value='" + String(g_cfg.s9_starRadius) + "' disabled></div>";
  s += "<div class='row'><label>unused (arm width)</label><input name='s9armw' type='number' min='1' max='6' value='" + String(g_cfg.s9_armWidth) + "' disabled></div>";
  s += "<div class='row'><label>max star size</label><input name='s9arml' type='number' min='8' max='25' value='" + String(g_cfg.s9_armLength) + "'></div>";
  s += "<div class='row'><label>spike length</label><input name='s9spike' type='number' min='2' max='10' value='" + String(g_cfg.s9_spikeLength) + "'></div>";
  s += "<div class='row'><label>show spikes</label><input name='s9spikes' type='checkbox' value='1' " + String(g_cfg.s9_showSpikes ? "checked" : "") + "></div>";
  s += "<div class='row'><label>filled centers</label><input name='s9filled' type='checkbox' value='1' " + String(g_cfg.s9_filled ? "checked" : "") + "></div>";
  s += "<div class='row'><label>min star size</label><input name='s9center' type='number' min='2' max='8' value='" + String(g_cfg.s9_centerSize) + "'></div>";
  s += "<div class='row'><label>smoothness</label><input name='s9smooth' type='number' min='10' max='90' value='" + String(g_cfg.s9_smoothness) + "'></div>";
  s += "</div>";

  s += "<div class='box'><h3>Styl 10 (Floating Peaks - Ulatujące szczyty)</h3>";
  s += "<p style='font-size:11px;color:#888'>Bazuje na Styl 5 + szczyty odlatują w górę</p>";
  s += "<div class='row'><label>bar width (px)</label><input name='s10barw' type='number' min='4' max='20' value='" + String(g_cfg.s10_barWidth) + "'></div>";
  s += "<div class='row'><label>bar gap (px)</label><input name='s10gap' type='number' min='1' max='6' value='" + String(g_cfg.s10_barGap) + "'></div>";
  s += "<div class='row'><label>segment height (px)</label><input name='s10segh' type='number' min='1' max='4' value='" + String(g_cfg.s10_segmentHeight) + "'></div>";
  s += "<div class='row'><label>segment gap (px)</label><input name='s10segg' type='number' min='0' max='3' value='" + String(g_cfg.s10_segmentGap) + "'></div>";
  s += "<div class='row'><label>max flying peaks</label><input name='s10maxp' type='number' min='1' max='5' value='" + String(g_cfg.s10_maxPeaks) + "'></div>";
  s += "<div class='row'><label>smoothness</label><input name='s10smooth' type='number' min='10' max='90' value='" + String(g_cfg.s10_smoothness) + "'></div>";
  s += "<div class='row'><label>peak hold time (frames)</label><input name='s10hold' type='number' min='0' max='100' value='" + String(g_cfg.s10_peakHoldTime) + "'></div>";
  s += "<div class='row'><label>float speed (px/frame)</label><input name='s10speed' type='number' min='1' max='15' value='" + String(g_cfg.s10_peakFloatSpeed) + "'></div>";
  s += "</div>";

  s += "<div class='row'>";
  s += "<button formaction='/analyzerApply' type='submit'>Podgląd LIVE</button>";
  s += "<button class='secondary' formaction='/analyzerSave' type='submit'>Zapisz</button>";
  s += "<a href='/analyzerCfg' style='margin-left:12px'>JSON</a>";
  s += "<a href='/analyzerDiag' style='margin-left:12px'>Diagnostyka</a>";
  s += "<a href='/analyzerTest' style='margin-left:12px'>Test Generator</a>";
  s += "</div>";

  s += "<div class='box'><h3>Diagnostyka Audio</h3>";
  s += "<p><strong>Problemy z formatami FLAC/AAC?</strong></p>";
  s += "<ul>";
  s += "<li>Sprawdź diagnostykę analizatora (link powyżej)</li>";
  s += "<li>FLAC wymaga więcej CPU - może powodować przerwy</li>";
  s += "<li>AAC+ i HE-AAC mogą mieć problemy z dekodowaniem</li>";
  s += "<li>Wysokie bitrate (>320kbps) może przeciążać ESP32</li>";
  s += "<li>Test Generator symuluje audio gdy brak sygnału</li>";
  s += "</ul>";
  s += "<p><strong>Rozwiązania:</strong></p>";
  s += "<ul>";
  s += "<li>Użyj stacji MP3 128-192kbps dla najlepszej stabilności</li>";
  s += "<li>Zrestartuj radio przy problemach z FLAC/AAC</li>";
  s += "<li>Sprawdź Serial Monitor dla błędów dekodera</li>";
  s += "</ul>";
  s += "</div>";

  s += "</form></body></html>";
  return s;
}

// ─────────────────────────────────────
// Additional analyzer functions
// ─────────────────────────────────────

void eqAnalyzerSetFromWeb(bool enabled)
{
  eqAnalyzerEnabled = enabled;
  eq_analyzer_set_enabled(enabled);  // Update the FFT analyzer state
}

// ─────────────────────────────────────
// STYL 5 – 16 słupków, zegar + ikonka głośnika
// ─────────────────────────────────────

void vuMeterMode5() // Tryb 5: 16 słupków – dynamiczny analizator z zegarem i ikonką głośnika
{
  // Powiedz analizatorowi, że jest aktywny
  eq_analyzer_set_runtime_active(true);
  
  // Jeśli analizator jest wyłączony – pokaż prosty komunikat
  if (!eqAnalyzerEnabled)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(10, 24);
    u8g2.print("ANALYZER OFF");
    u8g2.setCursor(10, 40);
    u8g2.print("Enable in Web UI");
    u8g2.sendBuffer();
    return;
  }

  // Sprawdź czy próbki napływają
  static uint32_t noSamplesTime5 = 0;
  if (!eq_analyzer_is_receiving_samples())
  {
    if (noSamplesTime5 == 0) noSamplesTime5 = millis();
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(10, 16);
    u8g2.print("NO AUDIO SAMPLES");
    u8g2.setCursor(10, 32);
    u8g2.print("Count: ");
    u8g2.print(eq_analyzer_get_sample_count());
    
    // Po 3 sekundach włącz generator testowy
    if (millis() - noSamplesTime5 > 3000) {
      u8g2.setCursor(10, 48);
      u8g2.print("Enabling test mode...");
      eq_analyzer_enable_test_generator(true);
      noSamplesTime5 = millis(); // Reset timer
    } else {
      u8g2.setCursor(10, 48);
      u8g2.print("Check audio source");
    }
    u8g2.sendBuffer();
    return;
  } else {
    noSamplesTime5 = 0; // Reset when receiving samples
    eq_analyzer_enable_test_generator(false); // Wyłącz generator gdy mamy audio
  }

  // 1. Pobranie poziomów z analizatora FFT (0..1)
  float levels[EQ_BANDS];
  float peaks[EQ_BANDS];
  eq_get_analyzer_levels(levels);
  eq_get_analyzer_peaks(peaks);

  // Debug co 2 sekundy - sprawdź wartości poziomów
  static uint32_t lastDebugDisplay = 0;
  uint32_t now = millis();
  if (now - lastDebugDisplay > 2000) {
    // Serial.print("Display levels: "); // Debug disabled
    for (int i = 0; i < EQ_BANDS; i++) {
      Serial.print(levels[i], 2);
      if (i < EQ_BANDS - 1) Serial.print(" ");
    }
    Serial.println();
    lastDebugDisplay = now;
  }

  // Przepisujemy do eqLevel/eqPeak w skali 0..100 dla rysowania słupków z wygładzaniem
  // Jeśli mute - animacja opadania słupków
  static uint8_t muteLevel[EQ_BANDS] = {0}; // Zapamietane poziomy dla animacji mute
  static float smoothedLevel[EQ_BANDS] = {0.0f}; // Wygładzone poziomy dla smoothness
  
  // Współczynnik wygładzania z s5_smoothness (10-90) -> odwrotnie (0.9-0.1)
  // Mniejsza wartość = więcej wygładzania, większa = mniej wygładzania
  float smoothFactor = (100.0f - g_cfg.s5_smoothness) / 100.0f;
  
  for (uint8_t i = 0; i < EQ_BANDS; i++)
  {
    if (volumeMute) {
      // Podczas mute - stopniowo opuszczaj słupki (animacja)
      if (muteLevel[i] > 2) {
        muteLevel[i] -= 2; // Opadanie o 2% na klatkę
      } else {
        muteLevel[i] = 0;
      }
      eqLevel[i] = muteLevel[i];
      eqPeak[i] = 0; // Peak natychmiast znika
      smoothedLevel[i] = 0.0f; // Reset wygładzania podczas mute
    } else {
      float lv = levels[i];
      float pk = peaks[i];
      if (lv < 0.0f) lv = 0.0f;
      if (lv > 1.0f) lv = 1.0f;
      if (pk < 0.0f) pk = 0.0f;
      if (pk > 1.0f) pk = 1.0f;

      // Wygładzanie poziomów: smoothFactor 0.0-1.0, gdzie 0=max smoothness, 1=no smoothness
      if (lv > smoothedLevel[i]) {
        // Attack - szybsze wchodzenie (mniej wygładzania)
        float attackSpeed = 0.3f + smoothFactor * 0.7f; // 0.3-1.0
        smoothedLevel[i] = smoothedLevel[i] + attackSpeed * (lv - smoothedLevel[i]);
      } else {
        // Release - wolniejsze opadanie (więcej wygładzania)
        float releaseSpeed = 0.1f + smoothFactor * 0.6f; // 0.1-0.7
        smoothedLevel[i] = smoothedLevel[i] + releaseSpeed * (lv - smoothedLevel[i]);
      }

      uint8_t newLevel = (uint8_t)(smoothedLevel[i] * 100.0f + 0.5f);
      eqLevel[i] = newLevel;
      eqPeak[i] = (uint8_t)(pk * 100.0f + 0.5f); // Peak bez wygładzania dla responsywności
      muteLevel[i] = newLevel; // Zapamietaj poziom dla animacji mute
    }
  }

  // 2. Rysowanie – zegar + ikonka głośnika u góry, słupki pod spodem
  u8g2.setDrawColor(1);
  u8g2.clearBuffer();

  // Pasek górny: zegar po lewej, stacja obok, ikonka głośnika po prawej
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5))
  {
    char timeString[9];
    if (timeinfo.tm_sec % 2 == 0)
      snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    else
      snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);

    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(4, 11);
    u8g2.print(timeString);

    // Nazwa stacji obok zegara
    uint8_t timeWidth = u8g2.getStrWidth(timeString);
    uint8_t xStation  = 4 + timeWidth + 6;

    // Zarezerwuj miejsce do ikony głośnika po prawej
    uint8_t iconX = 256 - 40;  // SCREEN_WIDTH = 256
    uint8_t maxStationWidth = 0;
    if (iconX > xStation + 4)
      maxStationWidth = iconX - xStation - 4;

    if (maxStationWidth > 0)
    {
      String nameToShow = stationName;
      if (nameToShow.length() == 0)
      {
        if (stationNameStream.length() > 0)      nameToShow = stationNameStream;
        else if (stationStringWeb.length() > 0)  nameToShow = stationStringWeb;
        else                                     nameToShow = "Radio";
      }

      // Przycinanie tekstu do wolnej szerokości
      while (nameToShow.length() > 0 &&
             u8g2.getStrWidth(nameToShow.c_str()) > maxStationWidth)
      {
        nameToShow.remove(nameToShow.length() - 1);
      }

      u8g2.setCursor(xStation, 11);
      u8g2.print(nameToShow);
    }
  }

  // Ikonka głośnika + wartość głośności po prawej
  uint8_t iconY = 2;
  uint8_t iconX = 256 - 40;  // SCREEN_WIDTH = 256

  // „kolumna" głośnika
  u8g2.drawBox(iconX, iconY + 2, 4, 7);
  // przód głośnika – linie
  u8g2.drawLine(iconX + 4, iconY + 2, iconX + 7, iconY);      // skośna góra
  u8g2.drawLine(iconX + 4, iconY + 8, iconX + 7, iconY + 10); // skośny dół
  u8g2.drawLine(iconX + 7, iconY,     iconX + 7, iconY + 10); // pion

  if (volumeMute) {
    // Przekreślenie dla mute - X nad ikonką
    u8g2.drawLine(iconX - 1, iconY, iconX + 11, iconY + 12);     // skos \
    u8g2.drawLine(iconX - 1, iconY + 12, iconX + 11, iconY);     // skos /
  } else {
    // „fale" dźwięku tylko gdy nie ma mute
    u8g2.drawPixel(iconX + 9,  iconY + 3);
    u8g2.drawPixel(iconX + 10, iconY + 5);
    u8g2.drawPixel(iconX + 9,  iconY + 7);
  }

  // Wartość głośności lub napis MUTED
  u8g2.setFont(u8g2_font_5x8_mr);
  u8g2.setCursor(iconX + 14, 10);
  if (volumeMute) {
    u8g2.print("MUTED");
  } else {
    u8g2.print(volumeValue);
  }

  // Linia oddzielająca pasek od słupków
  u8g2.drawHLine(0, 13, 256);  // SCREEN_WIDTH = 256

  // Obszar słupków – od linii w dół do końca ekranu
  const uint8_t eqTopY      = 14;                    // pod paskiem
  const uint8_t eqBottomY   = 64 - 1;               // SCREEN_HEIGHT = 64, do dołu
  const uint8_t eqMaxHeight = eqBottomY - eqTopY + 1;

  // Konfigurowalna liczba segmentów
  const uint8_t maxSegments = eq5_maxSegments;

  // Auto-dopasowanie do pełnej szerokości ekranu
  eq_auto_fit_width(5, 256);
  
  // Parametry słupków – 16 sztuk (konfigurowalne)
  const uint8_t barWidth = eq_barWidth5;
  const uint8_t barGap   = eq_barGap5;

  const uint16_t totalBarsWidth = EQ_BANDS * barWidth + (EQ_BANDS - 1) * barGap;
  int16_t startX = (256 - totalBarsWidth) / 2;  // SCREEN_WIDTH = 256
  if (startX < 2) startX = 2;  // Minimalny margines

  // Rysowanie słupków z peakami
  for (uint8_t i = 0; i < EQ_BANDS; i++)
  {
    uint8_t levelPercent = eqLevel[i];  // 0-100
    uint8_t peakPercent  = eqPeak[i];   // 0-100

    // Liczba segmentów w słupku
    uint8_t segments = (levelPercent * maxSegments) / 100;
    if (segments > maxSegments) segments = maxSegments;

    // Pozycja „peak" w segmentach
    uint8_t peakSeg = (peakPercent * maxSegments) / 100;
    if (peakSeg > maxSegments) peakSeg = maxSegments;

    // x słupka
    int16_t x = startX + i * (barWidth + barGap);

    // Rysujemy segmenty od dołu - używamy konfigurowalnej wysokości segmentu
    uint8_t segmentGap = 1;  // 1 piksel przerwy między segmentami
    uint8_t segmentHeight = g_cfg.s5_segHeight;  // Konfigurowalna wysokość segmentu
    
    for (uint8_t s = 0; s < segments; s++)
    {
      int16_t segBottom = eqBottomY - (s * (segmentHeight + segmentGap));
      int16_t segTop = segBottom - segmentHeight + 1;

      if (segTop < eqTopY) segTop = eqTopY;
      if (segBottom > eqBottomY) segBottom = eqBottomY;
      
      if (segTop <= segBottom) {
        // Rysuj segment z uwzględnieniem jasności słupków
        if (g_cfg.s5_barBrightness >= 255) {
          u8g2.drawBox(x, segTop, barWidth, segmentHeight);
        } else if (g_cfg.s5_barBrightness > 0) {
          // Rysuj pixel po pixelu z uwzględnieniem jasności
          for (int16_t px = 0; px < barWidth; px++) {
            for (int16_t py = 0; py < segmentHeight; py++) {
              if (shouldDrawBarPixel(g_cfg.s5_barBrightness, x + px, segTop + py)) {
                u8g2.drawPixel(x + px, segTop + py);
              }
            }
          }
        }
      }
    }

    // Peak – pojedyncza kreska nad słupkiem
    if (peakSeg > 0)
    {
      uint8_t ps = peakSeg - 1;
      int16_t peakSegBottom = eqBottomY - (ps * (segmentHeight + segmentGap));
      int16_t peakY = peakSegBottom - segmentHeight + 1 - 2; // 2 piksele nad segmentem
      if (peakY >= eqTopY && peakY <= eqBottomY)
      {
        // Rysuj peak z uwzględnieniem jasności szczytów
        if (g_cfg.s5_peakBrightness >= 255) {
          u8g2.drawBox(x, peakY, barWidth, 1);
        } else if (g_cfg.s5_peakBrightness > 0) {
          // Rysuj pixel po pixelu z uwzględnieniem jasności
          for (int16_t px = 0; px < barWidth; px++) {
            if (shouldDrawPeakPixel(g_cfg.s5_peakBrightness, x + px, peakY)) {
              u8g2.drawPixel(x + px, peakY);
            }
          }
        }
      }
    }
  }

  u8g2.sendBuffer();
}

// ─────────────────────────────────────
// STYL 6 – cienkie kreski + peak + zegar
// ─────────────────────────────────────

void vuMeterMode6() // Tryb 6: 16 słupków z cienkich „kreseczek" + peak, pełny analizator segmentowy
{
  // Powiedz analizatorowi, że jest aktywny
  eq_analyzer_set_runtime_active(true);
  
  if (!eqAnalyzerEnabled)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(10, 24);
    u8g2.print("ANALYZER OFF");
    u8g2.setCursor(10, 40);
    u8g2.print("Enable in Web UI");
    u8g2.sendBuffer();
    return;
  }

  // Sprawdź czy próbki napływają
  static uint32_t noSamplesTime6 = 0;
  if (!eq_analyzer_is_receiving_samples())
  {
    if (noSamplesTime6 == 0) noSamplesTime6 = millis();
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(10, 16);
    u8g2.print("NO AUDIO SAMPLES");
    u8g2.setCursor(10, 32);
    u8g2.print("Count: ");
    u8g2.print(eq_analyzer_get_sample_count());
    
    // Po 3 sekundach włącz generator testowy
    if (millis() - noSamplesTime6 > 3000) {
      u8g2.setCursor(10, 48);
      u8g2.print("Enabling test mode...");
      eq_analyzer_enable_test_generator(true);
      noSamplesTime6 = millis(); // Reset timer
    } else {
      u8g2.setCursor(10, 48);
      u8g2.print("Check audio source");
    }
    u8g2.sendBuffer();
    return;
  } else {
    noSamplesTime6 = 0; // Reset when receiving samples
    eq_analyzer_enable_test_generator(false); // Wyłącz generator gdy mamy audio
  }

  // 1. Pobranie poziomów z analizatora FFT (0..1)
  float levels[EQ_BANDS];
  float peaks[EQ_BANDS];
  eq_get_analyzer_levels(levels);
  eq_get_analyzer_peaks(peaks);

  // Konwersja poziomów z wygładzaniem - jeśli mute to animacja opadania słupków
  static uint8_t muteLevel6[EQ_BANDS] = {0}; // Zapamietane poziomy dla animacji mute
  static float smoothedLevel6[EQ_BANDS] = {0.0f}; // Wygładzone poziomy dla smoothness
  
  // Współczynnik wygładzania z s6_smoothness (10-90) -> odwrotnie (0.9-0.1)
  // Mniejsza wartość = więcej wygładzania, większa = mniej wygładzania
  float smoothFactor6 = (100.0f - g_cfg.s6_smoothness) / 100.0f;
  
  for (uint8_t i = 0; i < EQ_BANDS && i < EQ_BANDS; i++)
  {
    if (volumeMute) {
      // Podczas mute - stopniowo opuszczaj słupki (animacja)
      if (muteLevel6[i] > 2) {
        muteLevel6[i] -= 2; // Opadanie o 2% na klatkę
      } else {
        muteLevel6[i] = 0;
      }
      eqLevel[i] = muteLevel6[i];
      eqPeak[i] = 0; // Peak natychmiast znika
      smoothedLevel6[i] = 0.0f; // Reset wygładzania podczas mute
    } else {
      float lv = levels[i];
      float pk = peaks[i];
      if (lv < 0.0f) lv = 0.0f;
      if (lv > 1.0f) lv = 1.0f;
      if (pk < 0.0f) pk = 0.0f;
      if (pk > 1.0f) pk = 1.0f;

      // Wygładzanie poziomów: smoothFactor6 0.0-1.0, gdzie 0=max smoothness, 1=no smoothness
      if (lv > smoothedLevel6[i]) {
        // Attack - szybsze wchodzenie (mniej wygładzania)
        float attackSpeed = 0.3f + smoothFactor6 * 0.7f; // 0.3-1.0
        smoothedLevel6[i] = smoothedLevel6[i] + attackSpeed * (lv - smoothedLevel6[i]);
      } else {
        // Release - wolniejsze opadanie (więcej wygładzania)
        float releaseSpeed = 0.1f + smoothFactor6 * 0.6f; // 0.1-0.7
        smoothedLevel6[i] = smoothedLevel6[i] + releaseSpeed * (lv - smoothedLevel6[i]);
      }

      uint8_t newLevel = (uint8_t)(smoothedLevel6[i] * 100.0f + 0.5f);
      eqLevel[i] = newLevel;
      eqPeak[i] = (uint8_t)(pk * 100.0f + 0.5f); // Peak bez wygładzania dla responsywności
      muteLevel6[i] = newLevel; // Zapamietaj poziom dla animacji mute
    }
  }

  // 2. Rysowanie – pasek z zegarem + stacja + głośnik u góry, cienkie słupki pod spodem
  u8g2.setDrawColor(1);
  u8g2.clearBuffer();

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5))
  {
    char timeString[9];
    if (timeinfo.tm_sec % 2 == 0)
      snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    else
      snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);

    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(4, 11);
    u8g2.print(timeString);

    uint8_t timeWidth = u8g2.getStrWidth(timeString);
    uint8_t xStation  = 4 + timeWidth + 6;

    uint8_t iconX = 256 - 40;  // SCREEN_WIDTH = 256
    uint8_t maxStationWidth = 0;
    if (iconX > xStation + 4) maxStationWidth = iconX - xStation - 4;

    if (maxStationWidth > 0)
    {
      String nameToShow = stationName;
      if (nameToShow.length() == 0)
      {
        if (stationNameStream.length() > 0)      nameToShow = stationNameStream;
        else if (stationStringWeb.length() > 0)  nameToShow = stationStringWeb;
        else                                     nameToShow = "Radio";
      }

      while (nameToShow.length() > 0 &&
             u8g2.getStrWidth(nameToShow.c_str()) > maxStationWidth)
      {
        nameToShow.remove(nameToShow.length() - 1);
      }

      u8g2.setCursor(xStation, 11);
      u8g2.print(nameToShow);
    }
  }

  // Ikonka głośnika po prawej
  uint8_t iconY = 2;
  uint8_t iconX = 256 - 40;  // SCREEN_WIDTH = 256

  u8g2.drawBox(iconX, iconY + 2, 4, 7);
  u8g2.drawLine(iconX + 4, iconY + 2, iconX + 7, iconY);
  u8g2.drawLine(iconX + 4, iconY + 8, iconX + 7, iconY + 10);
  u8g2.drawLine(iconX + 7, iconY,     iconX + 7, iconY + 10);

  if (volumeMute) {
    // Przekreślenie dla mute - X nad ikonką
    u8g2.drawLine(iconX - 1, iconY, iconX + 11, iconY + 12);     // skos \
    u8g2.drawLine(iconX - 1, iconY + 12, iconX + 11, iconY);     // skos /
  } else {
    // Fale dźwięku tylko gdy nie ma mute
    u8g2.drawPixel(iconX + 9,  iconY + 3);
    u8g2.drawPixel(iconX + 10, iconY + 5);
    u8g2.drawPixel(iconX + 9,  iconY + 7);
  }

  // Wartość głośności lub napis MUTED
  u8g2.setFont(u8g2_font_5x8_mr);
  u8g2.setCursor(iconX + 14, 10);
  if (volumeMute) {
    u8g2.print("MUTED");
  } else {
    u8g2.print(volumeValue);
  }

  u8g2.drawHLine(0, 13, 256);  // SCREEN_WIDTH = 256

  const uint8_t eqTopY      = 14;
  const uint8_t eqBottomY   = 64 - 1;  // SCREEN_HEIGHT = 64
  const uint8_t eqMaxHeight = eqBottomY - eqTopY + 1;
  const uint8_t maxSegments = eq6_maxSegments;

  // Auto-dopasowanie do pełnej szerokości ekranu
  eq_auto_fit_width(6, 256);
  
  const uint8_t barWidth = eq_barWidth6;
  const uint8_t barGap   = eq_barGap6;

  const uint16_t totalBarsWidth = EQ_BANDS * barWidth + (EQ_BANDS - 1) * barGap;
  int16_t startX = (256 - totalBarsWidth) / 2;  // SCREEN_WIDTH = 256
  if (startX < 2) startX = 2;  // Minimalny margines

  for (uint8_t i = 0; i < EQ_BANDS; i++)
  {
    uint8_t levelPercent = eqLevel[i];
    uint8_t peakPercent  = eqPeak[i];

    uint8_t segments = (levelPercent * maxSegments) / 100;
    if (segments > maxSegments) segments = maxSegments;

    uint8_t peakSeg = (peakPercent * maxSegments) / 100;
    if (peakSeg > maxSegments) peakSeg = maxSegments;

    int16_t x = startX + i * (barWidth + barGap);

    // Rysujemy segmenty od dołu - wszystkie segmenty identyczne
    uint8_t segmentGap = 1;  // 1 piksel przerwy między segmentami
    int16_t availableHeight = eqMaxHeight - (maxSegments - 1) * segmentGap;
    uint8_t segmentHeight = (availableHeight > 0) ? (availableHeight / maxSegments) : 1;
    if (segmentHeight < 1) segmentHeight = 1;  // Minimum 1 piksel wysokości
    
    for (uint8_t s = 0; s < segments; s++)
    {
      int16_t segBottom = eqBottomY - (s * (segmentHeight + segmentGap));
      int16_t segTop = segBottom - segmentHeight + 1;
      
      if (segTop < eqTopY) segTop = eqTopY;
      if (segBottom > eqBottomY) segBottom = eqBottomY;
      if (segBottom < segTop) segBottom = segTop;
      
      if (segTop <= segBottom) {
        // Rysuj segment z uwzględnieniem jasności słupków
        if (g_cfg.s6_barBrightness >= 255) {
          u8g2.drawBox(x, segTop, barWidth, segmentHeight);
        } else if (g_cfg.s6_barBrightness > 0) {
          // Rysuj pixel po pixelu z uwzględnieniem jasności
          for (int16_t px = 0; px < barWidth; px++) {
            for (int16_t py = 0; py < segmentHeight; py++) {
              if (shouldDrawBarPixel(g_cfg.s6_barBrightness, x + px, segTop + py)) {
                u8g2.drawPixel(x + px, segTop + py);
              }
            }
          }
        }
      }
    }

    // Peak – pojedyncza kreska nad słupkiem
    if (peakSeg > 0)
    {
      uint8_t ps = peakSeg - 1;
      int16_t peakSegBottom = eqBottomY - (ps * (segmentHeight + segmentGap));
      int16_t peakY = peakSegBottom - segmentHeight + 1 - 2; // 2 piksele nad segmentem
      
      if (peakY >= eqTopY && peakY <= eqBottomY) {
        // Rysuj peak z uwzględnieniem jasności szczytów
        if (g_cfg.s6_peakBrightness >= 255) {
          u8g2.drawBox(x, peakY, barWidth, 1);
        } else if (g_cfg.s6_peakBrightness > 0) {
          // Rysuj pixel po pixelu z uwzględnieniem jasności
          for (int16_t px = 0; px < barWidth; px++) {
            if (shouldDrawPeakPixel(g_cfg.s6_peakBrightness, x + px, peakY)) {
              u8g2.drawPixel(x + px, peakY);
            }
          }
        }
      }
    }
  }

  u8g2.sendBuffer();
}
// 
// NOWE STYLE ANALIZATORA 7 i 8  
// 

void vuMeterMode7() // Styl 7: Okrągły analizator
{
  if (!eqAnalyzerEnabled) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(10, 24);
    u8g2.print("ANALYZER OFF");
    u8g2.sendBuffer();
    return;
  }

  // Pobierz poziomy
  float levels[EQ_BANDS];
  eq_get_analyzer_levels(levels);

  for (uint8_t i = 0; i < EQ_BANDS && i < EQ_BANDS; i++) {
    float lv = (levels[i] < 0.0f) ? 0.0f : ((levels[i] > 1.0f) ? 1.0f : levels[i]);
    eqLevel[i] = (uint8_t)(lv * 100.0f + 0.5f);
  }

  u8g2.clearBuffer();
  
  // Okrągły analizator
  int centerX = 128;
  int centerY = 32;
  
  for (uint8_t i = 0; i < EQ_BANDS; i++) {
    float angle = (i * 2.0 * PI) / EQ_BANDS;
    int radius = 10 + (eqLevel[i] * 20) / 100;
    
    int x = centerX + cos(angle) * radius;
    int y = centerY + sin(angle) * radius;
    
    u8g2.drawCircle(x, y, 2);
  }
  
  u8g2.sendBuffer();
}

void vuMeterMode8() // Styl 8: Liniowy analizator
{
  // Powiedz analizatorowi, że jest aktywny
  eq_analyzer_set_runtime_active(true);
  
  if (!eqAnalyzerEnabled) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(10, 24);
    u8g2.print("ANALYZER OFF");
    u8g2.sendBuffer();
    return;
  }

  // Pobierz poziomy z animacją mute
  float levels[EQ_BANDS];
  eq_get_analyzer_levels(levels);
  
  static uint8_t muteLevel8[EQ_BANDS] = {0};

  for (uint8_t i = 0; i < EQ_BANDS && i < EQ_BANDS; i++) {
    if (volumeMute) {
      if (muteLevel8[i] > 2) {
        muteLevel8[i] -= 2;
      } else {
        muteLevel8[i] = 0;
      }
      eqLevel[i] = muteLevel8[i];
    } else {
      float lv = (levels[i] < 0.0f) ? 0.0f : ((levels[i] > 1.0f) ? 1.0f : levels[i]);
      uint8_t newLevel = (uint8_t)(lv * 100.0f + 0.5f);
      eqLevel[i] = newLevel;
      muteLevel8[i] = newLevel;
    }
  }

  u8g2.clearBuffer();
  
  // Liniowy analizator
  for (uint8_t i = 0; i < EQ_BANDS; i++) {
    int y = 10 + i * 3;
    int lineWidth = (eqLevel[i] * 240) / 100;
    
    if (lineWidth > 0) {
      u8g2.drawBox(8, y, lineWidth, 2);
    }
  }
  
  u8g2.sendBuffer();
}
void vuMeterMode9() // Styl 9: Spadające gwiazdki jak śnieg
{
  // Powiedz analizatorowi, że jest aktywny
  eq_analyzer_set_runtime_active(true);
  
  if (!eqAnalyzerEnabled) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(10, 24);
    u8g2.print("ANALYZER OFF");
    u8g2.sendBuffer();
    return;
  }

  // Pobierz poziomy
  float levels[EQ_BANDS];
  eq_get_analyzer_levels(levels);

  u8g2.clearBuffer();
  
  // Parametry ekranu
  const int screenWidth = 256;
  const int screenHeight = 64;
  
  // Animacja mute dla gwiazdek
  static float muteLevel9[EQ_BANDS] = {0.0f};
  
  // Każdy band FFT reprezentuje jedną gwiazdkę
  for (uint8_t i = 0; i < EQ_BANDS && i < EQ_BANDS; i++) {
    float lv;
    if (volumeMute) {
      if (muteLevel9[i] > 0.02f) {
        muteLevel9[i] -= 0.02f; // Opadanie o 2% na klatkę
      } else {
        muteLevel9[i] = 0.0f;
      }
      lv = muteLevel9[i];
    } else {
      lv = (levels[i] < 0.0f) ? 0.0f : ((levels[i] > 1.0f) ? 1.0f : levels[i]);
      muteLevel9[i] = lv;
    }
    
    // Pozycja X gwiazdki (rozmieszczone równomiernie na szerokości)
    int starX = (i * screenWidth) / EQ_BANDS + (screenWidth / EQ_BANDS) / 2;
    
    // Pozycja Y gwiazdki (im wyższy poziom audio, tym wyżej gwiazdka - efekt spadania)
    // Odwracamy logikę: wysokie poziomy = góra ekranu (niższe Y)
    int starY = screenHeight - (int)(lv * screenHeight * 0.9f) - 5; // -5 żeby nie dotykać góry
    
    // Rozmiar gwiazdki na podstawie poziomu audio
    float starSize = g_cfg.s9_centerSize + (lv * g_cfg.s9_armLength * 0.5f);
    
    // Rysowanie małej 6-ramiennej gwiazdki tylko jeśli poziom > 0.05
    if (lv > 0.05f) {
      // Środek gwiazdki
      if (g_cfg.s9_filled) {
        u8g2.drawDisc(starX, starY, max(1, (int)(starSize * 0.2f)));
      } else {
        u8g2.drawPixel(starX, starY);
      }
      
      // 6 ramion gwiazdki
      for (uint8_t arm = 0; arm < 6; arm++) {
        float angle = arm * PI / 3.0f; // 60 stopni między ramionami
        
        // Długość ramienia proporcjonalna do rozmiaru gwiazdki
        int armLength = (int)(starSize * 0.8f);
        if (armLength < 2) armLength = 2; // Minimalna długość
        
        // Koniec ramienia
        int armEndX = starX + cos(angle) * armLength;
        int armEndY = starY + sin(angle) * armLength;
        
        // Sprawdź czy ramię mieści się na ekranie
        if (armEndX >= 0 && armEndX < screenWidth && armEndY >= 0 && armEndY < screenHeight) {
          u8g2.drawLine(starX, starY, armEndX, armEndY);
          
          // Dyszki na końcach (jeśli włączone i gwiazdka wystarczająco duża)
          if (g_cfg.s9_showSpikes && starSize > 3.0f) {
            float spikeLength = starSize * 0.3f;
            if (spikeLength < 2) spikeLength = 2;
            
            // Dwie dyszki pod kątem ±45 stopni
            float spikeAngle1 = angle - PI/4.0f; // -45 stopni
            float spikeAngle2 = angle + PI/4.0f; // +45 stopni
            
            int spike1X = armEndX + cos(spikeAngle1) * spikeLength;
            int spike1Y = armEndY + sin(spikeAngle1) * spikeLength;
            int spike2X = armEndX + cos(spikeAngle2) * spikeLength;
            int spike2Y = armEndY + sin(spikeAngle2) * spikeLength;
            
            // Rysuj dyszki jeśli mieszczą się na ekranie
            if (spike1X >= 0 && spike1X < screenWidth && spike1Y >= 0 && spike1Y < screenHeight) {
              u8g2.drawLine(armEndX, armEndY, spike1X, spike1Y);
            }
            if (spike2X >= 0 && spike2X < screenWidth && spike2Y >= 0 && spike2Y < screenHeight) {
              u8g2.drawLine(armEndX, armEndY, spike2X, spike2Y);
            }
          }
        }
      }
    }
  }
  
  u8g2.sendBuffer();
}

void vuMeterMode10() // Styl 10: Floating Peaks - szczytowe ulatują w górę (bazuje na Styl 5)
{
  eq_analyzer_set_runtime_active(true);
  
  if (!eqAnalyzerEnabled) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(10, 24);
    u8g2.print("ANALYZER OFF");
    u8g2.setCursor(10, 40);
    u8g2.print("Enable in Web UI");
    u8g2.sendBuffer();
    return;
  }

  // 1. Pobranie poziomów z analizatora FFT (0..1) - tak samo jak w Styl 5
  float levels[EQ_BANDS];
  float peaks[EQ_BANDS];
  eq_get_analyzer_levels(levels);
  eq_get_analyzer_peaks(peaks);

  // Wygładzanie - tak samo jak w Styl 5
  static uint8_t muteLevel[EQ_BANDS] = {0};
  static float smoothedLevel[EQ_BANDS] = {0.0f};
  float smoothFactor = (100.0f - g_cfg.s10_smoothness) / 100.0f;
  
  for (uint8_t i = 0; i < EQ_BANDS; i++) {
    if (volumeMute) {
      if (muteLevel[i] > 2) muteLevel[i] -= 2;
      else muteLevel[i] = 0;
      eqLevel[i] = muteLevel[i];
      eqPeak[i] = 0;
      smoothedLevel[i] = 0.0f;
    } else {
      float lv = levels[i];
      float pk = peaks[i];
      if (lv < 0.0f) lv = 0.0f;
      if (lv > 1.0f) lv = 1.0f;
      if (pk < 0.0f) pk = 0.0f;
      if (pk > 1.0f) pk = 1.0f;

      if (lv > smoothedLevel[i]) {
        float attackSpeed = 0.3f + smoothFactor * 0.7f;
        smoothedLevel[i] = smoothedLevel[i] + attackSpeed * (lv - smoothedLevel[i]);
      } else {
        float releaseSpeed = 0.1f + smoothFactor * 0.6f;
        smoothedLevel[i] = smoothedLevel[i] + releaseSpeed * (lv - smoothedLevel[i]);
      }

      uint8_t newLevel = (uint8_t)(smoothedLevel[i] * 100.0f + 0.5f);
      eqLevel[i] = newLevel;
      eqPeak[i] = (uint8_t)(pk * 100.0f + 0.5f);
      muteLevel[i] = newLevel;
    }
  }

  u8g2.setDrawColor(1);
  u8g2.clearBuffer();

  // Pasek górny: zegar, stacja, głośnik - tak samo jak Styl 5
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5)) {
    char timeString[9];
    if (timeinfo.tm_sec % 2 == 0)
      snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    else
      snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(4, 11);
    u8g2.print(timeString);

    uint8_t timeWidth = u8g2.getStrWidth(timeString);
    uint8_t xStation  = 4 + timeWidth + 6;
    uint8_t iconX = 256 - 40;
    uint8_t maxStationWidth = 0;
    if (iconX > xStation + 4) maxStationWidth = iconX - xStation - 4;

    if (maxStationWidth > 0) {
      String nameToShow = stationName;
      if (nameToShow.length() == 0) {
        if (stationNameStream.length() > 0) nameToShow = stationNameStream;
        else if (stationStringWeb.length() > 0) nameToShow = stationStringWeb;
        else nameToShow = "Radio";
      }
      while (nameToShow.length() > 0 && u8g2.getStrWidth(nameToShow.c_str()) > maxStationWidth) {
        nameToShow.remove(nameToShow.length() - 1);
      }
      u8g2.setCursor(xStation, 11);
      u8g2.print(nameToShow);
    }
  }

  // Ikona głośnika
  uint8_t iconY = 2;
  uint8_t iconX = 256 - 40;
  u8g2.drawBox(iconX, iconY + 2, 4, 7);
  u8g2.drawLine(iconX + 4, iconY + 2, iconX + 7, iconY);
  u8g2.drawLine(iconX + 4, iconY + 8, iconX + 7, iconY + 10);
  u8g2.drawLine(iconX + 7, iconY,     iconX + 7, iconY + 10);
  if (volumeMute) {
    u8g2.drawLine(iconX - 1, iconY, iconX + 11, iconY + 12);
    u8g2.drawLine(iconX - 1, iconY + 12, iconX + 11, iconY);
  } else {
    u8g2.drawPixel(iconX + 9,  iconY + 3);
    u8g2.drawPixel(iconX + 10, iconY + 5);
    u8g2.drawPixel(iconX + 9,  iconY + 7);
  }
  u8g2.setFont(u8g2_font_5x8_mr);
  u8g2.setCursor(iconX + 14, 10);
  if (volumeMute) u8g2.print("MUTED");
  else u8g2.print(volumeValue);
  u8g2.drawHLine(0, 13, 256);

  // Obszar słupków
  const uint8_t eqTopY      = 14;
  const uint8_t eqBottomY   = 64 - 1;
  const uint8_t maxSegments = 32;
  
  // Parametry słupków - używamy s10_barWidth i s10_barGap
  const uint8_t barWidth = g_cfg.s10_barWidth;
  const uint8_t barGap   = g_cfg.s10_barGap;
  const uint16_t totalBarsWidth = EQ_BANDS * barWidth + (EQ_BANDS - 1) * barGap;
  int16_t startX = (256 - totalBarsWidth) / 2;
  if (startX < 2) startX = 2;

  // Floating peaks - WIELE peaków na słupek (konfigurowalny limit)
  static const uint8_t MAX_PEAKS_ARRAY = 5;  // Rozmiar tablicy (stały)
  uint8_t maxPeaksActive = g_cfg.s10_maxPeaks; // Limit aktywnych peaków (konfigurowalny)
  struct FlyingPeak {
    float y;
    uint8_t holdCounter;
    bool active;
  };
  static FlyingPeak flyingPeaks[EQ_BANDS][MAX_PEAKS_ARRAY] = {};
  static uint8_t lastPeakSeg[EQ_BANDS] = {0};     // ostatni segment peak (do wykrycia nowego szczytu)
  static bool wasRising[EQ_BANDS] = {false};      // czy słupek rósł w poprzedniej klatce

  uint8_t segmentGap = g_cfg.s10_segmentGap;
  uint8_t segmentHeight = g_cfg.s10_segmentHeight;

  for (uint8_t i = 0; i < EQ_BANDS; i++) {
    uint8_t levelPercent = eqLevel[i];
    uint8_t peakPercent  = eqPeak[i];
    
    uint8_t segments = (levelPercent * maxSegments) / 100;
    if (segments > maxSegments) segments = maxSegments;

    uint8_t peakSeg = (peakPercent * maxSegments) / 100;
    if (peakSeg > maxSegments) peakSeg = maxSegments;

    int16_t x = startX + i * (barWidth + barGap);

    // Rysowanie słupków (segmenty) - pełna jasność
    for (uint8_t s = 0; s < segments; s++) {
      int16_t segBottom = eqBottomY - (s * (segmentHeight + segmentGap));
      int16_t segTop = segBottom - segmentHeight + 1;
      if (segTop < eqTopY) segTop = eqTopY;
      if (segBottom > eqBottomY) segBottom = eqBottomY;
      if (segTop <= segBottom) {
        u8g2.drawBox(x, segTop, barWidth, segmentHeight);
      }
    }

    // === FLOATING PEAKS - wiele peaków latających ===
    // Oblicz pozycję Y dla aktualnego szczytu słupka (góra słupka)
    int16_t barTopY = eqBottomY; // domyślnie na dole jeśli brak segmentów
    if (segments > 0) {
      int16_t topSegBottom = eqBottomY - ((segments - 1) * (segmentHeight + segmentGap));
      barTopY = topSegBottom - segmentHeight + 1;
      if (barTopY < eqTopY) barTopY = eqTopY;
    }
    
    // Oblicz pozycję Y dla aktualnego peak (używane do wystrzeliwania)
    int16_t currentPeakY = eqBottomY;
    if (peakSeg > 0) {
      uint8_t ps = peakSeg - 1;
      int16_t peakSegBottom = eqBottomY - (ps * (segmentHeight + segmentGap));
      currentPeakY = peakSegBottom - segmentHeight + 1 - 2;
      if (currentPeakY < eqTopY) currentPeakY = eqTopY;
    }

    // Wykryj NOWY szczyt - gdy peak rósł i teraz zaczyna spadać
    bool isRising = (peakSeg > lastPeakSeg[i]);
    bool justPeaked = (wasRising[i] && !isRising && peakSeg > 0);
    
    // Wystrzel nowy floating peak gdy słupek osiągnął szczyt i zaczyna opadać
    if (justPeaked) {
      // Znajdź wolny slot (tylko do limitu maxPeaksActive)
      for (uint8_t p = 0; p < maxPeaksActive; p++) {
        if (!flyingPeaks[i][p].active) {
          flyingPeaks[i][p].y = (float)currentPeakY;
          flyingPeaks[i][p].holdCounter = g_cfg.s10_peakHoldTime;
          flyingPeaks[i][p].active = true;
          break;
        }
      }
    }
    
    wasRising[i] = isRising;
    lastPeakSeg[i] = peakSeg;

    // Aktualizuj i rysuj wszystkie aktywne floating peaks
    for (uint8_t p = 0; p < MAX_PEAKS_ARRAY; p++) {
      if (flyingPeaks[i][p].active) {
        // Hold time - peak czeka na miejscu
        if (flyingPeaks[i][p].holdCounter > 0) {
          flyingPeaks[i][p].holdCounter--;
        } else {
          // Peak odlatuje w górę
          flyingPeaks[i][p].y -= (float)g_cfg.s10_peakFloatSpeed * 0.5f;
        }
        
        // Rysuj peak TYLKO jeśli jest POWYŻEJ szczytu słupka (nie nakłada się na słupek)
        int16_t peakY = (int16_t)flyingPeaks[i][p].y;
        if (peakY < barTopY && peakY >= eqTopY) {
          // Peak jest powyżej słupka - rysuj
          u8g2.drawBox(x, peakY, barWidth, 1);
        } else if (peakY < eqTopY) {
          // Peak wyleciał poza ekran - dezaktywuj
          flyingPeaks[i][p].active = false;
        }
        // Jeśli peak jest w obszarze słupka (peakY >= barTopY) - nie rysuj, ale nie dezaktywuj
        // Peak będzie widoczny gdy słupek opadnie
      }
    }
  }
  
  u8g2.sendBuffer();
}
//
// FUNKCJE ZARZĄDZANIA PRESETAMI
//

void analyzerApplyPreset(uint8_t presetId) {
  switch(presetId) {
    case 0: analyzerPresetClassic(); break;
    case 1: analyzerPresetModern(); break;
    case 2: analyzerPresetCompact(); break;
    case 3: analyzerPresetRetro(); break;
    case 4: analyzerPresetFloatingPeaks(); break;
    default: break; // 5 = Custom - no changes
  }
  
  Serial.printf("DEBUG: Applied preset %d - peakHoldTimeMs=%d\n", presetId, g_cfg.peakHoldTimeMs);
  
  // Apply settings immediately
  analyzerSetStyle(g_cfg);
  
  Serial.printf("DEBUG: After validation - peakHoldTimeMs=%d\n", g_cfg.peakHoldTimeMs);
}

void analyzerSetStyleMode(uint8_t styleMode) {
  // Store mode for reference (this could be used later for limiting available styles)
  // For now, all styles 0-8 are always available
  // This function exists for future extensibility
}

uint8_t analyzerGetMaxDisplayMode() {
  return 10;
}

bool analyzerIsStyleAvailable(uint8_t style) {
  return (style >= 0 && style <= 10);
}

uint8_t analyzerGetAvailableStylesMode() {
  return 2; // Always return 2 = All styles available (0-4-5-6-7-8-9)
}

void analyzerPresetClassic() {
  Serial.printf("DEBUG: Setting Classic preset - peakHoldTimeMs = 40\n");
  g_cfg.peakHoldTimeMs = 40;  // 5x szybciej
  g_cfg.s5_barWidth = 8;
  g_cfg.s5_barGap = 2;
  g_cfg.s5_segments = 32;
  g_cfg.s5_fill = 0.7f;
  g_cfg.s5_showPeaks = true;
  g_cfg.s5_smoothness = 50;  // Średnie wygładzanie
  g_cfg.s5_barBrightness = 220;  // Lekko ściemnione słupki
  g_cfg.s5_peakBrightness = 255; // Pełna jasność szczytów
  
  g_cfg.s6_gap = 1;
  g_cfg.s6_fill = 0.6f;
  g_cfg.s6_segMax = 40;
  g_cfg.s6_showPeaks = true;
  g_cfg.s6_smoothness = 40;  // Średnie wygładzanie
  g_cfg.s6_barBrightness = 220;  // Lekko ściemnione słupki
  g_cfg.s6_peakBrightness = 255; // Pełna jasność szczytów
  
  g_cfg.s7_circleRadius = 3;
  g_cfg.s7_circleGap = 2;
  g_cfg.s7_filled = true;
  g_cfg.s7_maxHeight = 45;
  
  g_cfg.s8_lineThickness = 2;
  g_cfg.s8_lineGap = 3;
  g_cfg.s8_gradient = false;
  g_cfg.s8_maxHeight = 50;
  
  g_cfg.s9_starRadius = 18;
  g_cfg.s9_armWidth = 2;
  g_cfg.s9_armLength = 12;
  g_cfg.s9_spikeLength = 4;
  g_cfg.s9_showSpikes = true;
  g_cfg.s9_filled = true;
  g_cfg.s9_centerSize = 3;
  g_cfg.s9_smoothness = 8;  // 5x szybciej
}

void analyzerPresetModern() {
  g_cfg.peakHoldTimeMs = 60;  // 5x szybciej
  g_cfg.s5_barWidth = 12;
  g_cfg.s5_barGap = 4;
  g_cfg.s5_segments = 48;
  g_cfg.s5_fill = 0.8f;
  g_cfg.s5_showPeaks = true;
  g_cfg.s5_smoothness = 30;  // Szybsze dla Modern
  g_cfg.s5_barBrightness = 200;  // Ściemnione słupki
  g_cfg.s5_peakBrightness = 255; // Rozjaśnione szczyty
  
  g_cfg.s6_gap = 2;
  g_cfg.s6_fill = 0.7f;
  g_cfg.s6_segMax = 48;
  g_cfg.s6_showPeaks = true;
  g_cfg.s6_smoothness = 25;  // Szybsze dla Modern
  g_cfg.s6_barBrightness = 200;  // Ściemnione słupki
  g_cfg.s6_peakBrightness = 255; // Rozjaśnione szczyty
  
  g_cfg.s7_circleRadius = 4;
  g_cfg.s7_circleGap = 3;
  g_cfg.s7_filled = true;
  g_cfg.s7_maxHeight = 55;
  
  g_cfg.s8_lineThickness = 3;
  g_cfg.s8_lineGap = 4;
  g_cfg.s8_gradient = true;
  g_cfg.s8_maxHeight = 58;
  
  g_cfg.s9_starRadius = 22;
  g_cfg.s9_armWidth = 4;
  g_cfg.s9_armLength = 18;
  g_cfg.s9_spikeLength = 6;
  g_cfg.s9_showSpikes = true;
  g_cfg.s9_filled = true;
  g_cfg.s9_centerSize = 5;
  g_cfg.s9_smoothness = 4;  // 5x szybciej
}

void analyzerPresetCompact() {
  g_cfg.peakHoldTimeMs = 30;  // 5x szybciej
  g_cfg.s5_barWidth = 6;
  g_cfg.s5_barGap = 1;
  g_cfg.s5_segments = 24;
  g_cfg.s5_fill = 0.5f;
  g_cfg.s5_showPeaks = false;
  g_cfg.s5_smoothness = 60;  // Wolniejsze dla Compact (oszczędność mocy)
  g_cfg.s5_barBrightness = 180;  // Mocno ściemnione słupki
  g_cfg.s5_peakBrightness = 255; // Pełna jasność szczytów
  
  g_cfg.s6_gap = 0;
  g_cfg.s6_fill = 0.5f;
  g_cfg.s6_segMax = 32;
  g_cfg.s6_showPeaks = false;
  g_cfg.s6_smoothness = 55;  // Wolniejsze dla Compact
  g_cfg.s6_barBrightness = 180;  // Mocno ściemnione słupki
  g_cfg.s6_peakBrightness = 255; // Pełna jasność szczytów
  
  g_cfg.s7_circleRadius = 2;
  g_cfg.s7_circleGap = 1;
  g_cfg.s7_filled = false;
  g_cfg.s7_maxHeight = 35;
  
  g_cfg.s8_lineThickness = 1;
  g_cfg.s8_lineGap = 2;
  g_cfg.s8_gradient = false;
  g_cfg.s8_maxHeight = 40;
  
  g_cfg.s9_starRadius = 12;
  g_cfg.s9_armWidth = 1;
  g_cfg.s9_armLength = 8;
  g_cfg.s9_spikeLength = 3;
  g_cfg.s9_showSpikes = false;
  g_cfg.s9_filled = false;
  g_cfg.s9_centerSize = 2;
  g_cfg.s9_smoothness = 12; // 5x szybciej
}

void analyzerPresetRetro() {
  g_cfg.peakHoldTimeMs = 80;  // 5x szybciej
  g_cfg.s5_barWidth = 10;
  g_cfg.s5_barGap = 6;
  g_cfg.s5_segments = 28;
  g_cfg.s5_fill = 0.6f;
  g_cfg.s5_showPeaks = true;
  g_cfg.s5_smoothness = 70;  // Bardzo wolno dla Retro (klassyczny wygląd)
  g_cfg.s5_barBrightness = 240;  // Lekko przyciemnione słupki
  g_cfg.s5_peakBrightness = 255; // Pełna jasność szczytów
  
  g_cfg.s6_gap = 3;
  g_cfg.s6_fill = 0.6f;
  g_cfg.s6_segMax = 36;
  g_cfg.s6_showPeaks = true;
  g_cfg.s6_smoothness = 65;  // Bardzo wolno dla Retro
  g_cfg.s6_barBrightness = 240;  // Lekko przyciemnione słupki
  g_cfg.s6_peakBrightness = 255; // Pełna jasność szczytów
  
  g_cfg.s7_circleRadius = 3;
  g_cfg.s7_circleGap = 4;
  g_cfg.s7_filled = false;
  g_cfg.s7_maxHeight = 42;
  
  g_cfg.s8_lineThickness = 3;
  g_cfg.s8_lineGap = 2;
  g_cfg.s8_gradient = false;
  g_cfg.s8_maxHeight = 60;
  
  g_cfg.s9_starRadius = 16;
  g_cfg.s9_armWidth = 3;
  g_cfg.s9_armLength = 10;
  g_cfg.s9_spikeLength = 5;
  g_cfg.s9_showSpikes = true;
  g_cfg.s9_filled = false;
  g_cfg.s9_centerSize = 4;
  g_cfg.s9_smoothness = 10; // 5x szybciej
}

void analyzerPresetFloatingPeaks() {
  Serial.printf("DEBUG: Setting Floating Peaks preset - peakHoldTimeMs = 25\n");
  g_cfg.peakHoldTimeMs = 25;  // 8x szybciej dla animacji
  
  // Optymalizacje dla wszystkich stylów
  g_cfg.s5_barWidth = 10;
  g_cfg.s5_barGap = 3;
  g_cfg.s5_segments = 30;
  g_cfg.s5_fill = 0.6f;
  g_cfg.s5_showPeaks = true;
  g_cfg.s5_smoothness = 35; // Szybkie wygładzanie
  g_cfg.s5_barBrightness = 180; // Ściemnione słupki
  g_cfg.s5_peakBrightness = 255; // Rozjaśnione szczyty
  
  g_cfg.s6_gap = 2;
  g_cfg.s6_fill = 0.65f;
  g_cfg.s6_segMax = 35;
  g_cfg.s6_showPeaks = true;
  g_cfg.s6_smoothness = 30;
  g_cfg.s6_barBrightness = 180;
  g_cfg.s6_peakBrightness = 255;
  
  // Styl 10 - Floating Peaks - główne ustawienia
  g_cfg.s10_barWidth = 12;
  g_cfg.s10_barGap = 2;
  g_cfg.s10_peakHoldTime = 8; // Czas zatrzymania (krótki)
  g_cfg.s10_peakFloatSpeed = 8; // Prędkość ulatywania
  g_cfg.s10_peakFadeSteps = 12; // Kroki zanikania
  g_cfg.s10_trailLength = 6; // Długość śladu
  g_cfg.s10_showTrails = true;
  g_cfg.s10_smoothness = 30; // Szybkie wygładzanie
  g_cfg.s10_barBrightness = 180; // Ściemnione słupki
  g_cfg.s10_peakBrightness = 255; // Rozjaśnione szczyty
  g_cfg.s10_trailBrightness = 190; // Widoczny ślad ale nie przesłaniający
  g_cfg.s10_peakMinHeight = 3;
  g_cfg.s10_floatHeight = 15;
  g_cfg.s10_enableAnimation = true;
}
