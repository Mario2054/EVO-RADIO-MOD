#include "EQ_AnalyzerDisplay.h"
#include "EQ_FFTAnalyzer.h"

#include <FS.h>
#include <U8g2lib.h>
#include <time.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

// Function to get storage from main.cpp
extern fs::FS& getStorage();
extern String stationName;
extern String stationNameStream;
extern String stationStringWeb;
extern uint8_t volumeValue;
extern bool volumeMute;
extern U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2;

// EQ variables - defined locally since they are simple arrays
extern uint8_t eqLevel[EQ_BANDS];
extern uint8_t eqPeak[EQ_BANDS];

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

AnalyzerStyleCfg analyzerGetStyle() { return g_cfg; }

uint32_t analyzerGetPeakHoldTime() {
  return g_cfg.peakHoldTimeMs;
}

void analyzerSetStyle(const AnalyzerStyleCfg& in)
{
  AnalyzerStyleCfg c = in;

  // Globalne ustawienia
  c.peakHoldTimeMs = (c.peakHoldTimeMs < 50) ? 50 : (c.peakHoldTimeMs > 2000) ? 2000 : c.peakHoldTimeMs;

  // Styl 5
  c.s5_barWidth = clampU8(c.s5_barWidth, 2, 30);
  c.s5_barGap   = clampU8(c.s5_barGap,   0, 20);
  c.s5_segments = clampU8(c.s5_segments,  4, 48);
  c.s5_fill     = clampF(c.s5_fill,     0.10f, 1.00f);

  // Styl 6
  c.s6_gap    = clampU8(c.s6_gap,    0, 10);
  c.s6_shrink = clampU8(c.s6_shrink, 0, 5);
  c.s6_fill   = clampF(c.s6_fill,   0.10f, 1.00f);
  c.s6_segMin = clampU8(c.s6_segMin, 4, 48);
  c.s6_segMax = clampU8(c.s6_segMax, 4, 48);
  if (c.s6_segMax < c.s6_segMin) c.s6_segMax = c.s6_segMin;

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

  g_cfg = c;

  // Mapuj konfigurację na nasze zmienne globalne
  eq5_maxSegments = g_cfg.s5_segments;
  eq_barWidth5 = g_cfg.s5_barWidth;
  eq_barGap5 = g_cfg.s5_barGap;
  
  eq6_maxSegments = g_cfg.s6_segMax;
  eq_barWidth6 = 10;  // będzie obliczone automatycznie w eq_auto_fit_width
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

  File f = getStorage().open(kCfgPath, FILE_READ);
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
    else if (k == "s5peaks") c.s5_showPeaks = v.toInt() != 0;

    // Styl 6
    else if (k == "s6g")    c.s6_gap      = (uint8_t)v.toInt();
    else if (k == "s6sh")   c.s6_shrink   = (uint8_t)v.toInt();
    else if (k == "s6fill") c.s6_fill     = v.toFloat();
    else if (k == "s6min")  c.s6_segMin   = (uint8_t)v.toInt();
    else if (k == "s6max")  c.s6_segMax   = (uint8_t)v.toInt();
    else if (k == "s6peaks") c.s6_showPeaks = v.toInt() != 0;

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
  }
  f.close();

  analyzerSetStyle(c);
}

void analyzerStyleSave()
{
  File f = getStorage().open(kCfgPath, FILE_WRITE);
  if (!f) return;

  f.println("# Analyzer style cfg");
  f.println("# Global settings");
  f.printf("peakHoldMs=%u\n", g_cfg.peakHoldTimeMs);
  f.println("# Style5");
  f.printf("s5w=%u\n", g_cfg.s5_barWidth);
  f.printf("s5g=%u\n", g_cfg.s5_barGap);
  f.printf("s5seg=%u\n", g_cfg.s5_segments);
  f.printf("s5fill=%.3f\n", g_cfg.s5_fill);
  f.printf("s5peaks=%u\n", g_cfg.s5_showPeaks ? 1 : 0);

  f.println("# Style6");
  f.printf("s6g=%u\n", g_cfg.s6_gap);
  f.printf("s6sh=%u\n", g_cfg.s6_shrink);
  f.printf("s6fill=%.3f\n", g_cfg.s6_fill);
  f.printf("s6min=%u\n", g_cfg.s6_segMin);
  f.printf("s6max=%u\n", g_cfg.s6_segMax);
  f.printf("s6peaks=%u\n", g_cfg.s6_showPeaks ? 1 : 0);

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
  
  f.close();
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

  // Styl 6
  s += "\"s6_gap\":"      + String(g_cfg.s6_gap) + ",";
  s += "\"s6_shrink\":"   + String(g_cfg.s6_shrink) + ",";
  s += "\"s6_fill\":"     + String(g_cfg.s6_fill, 3) + ",";
  s += "\"s6_segMin\":"   + String(g_cfg.s6_segMin) + ",";
  s += "\"s6_segMax\":"   + String(g_cfg.s6_segMax) + ",";
  s += "\"s6_showPeaks\":" + String(g_cfg.s6_showPeaks ? "true" : "false") + ",";

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
  s += "button{padding:10px 12px;border:0;border-radius:10px;background:#222;color:#fff;cursor:pointer}";
  s += "button.secondary{background:#555} a{color:#0a58ca}";
  s += "</style></head><body>";
  return s;
}

String analyzerBuildHtmlPage()
{
  String s = htmlHeader("Analyzer / Style 5-9");
  s += "<h2>Analizator – ustawienia stylów 5, 6, 7, 8, 9</h2>";
  s += "<p>Tu regulujesz WYGLĄD słupków. Same słupki ruszą dopiero, gdy w /config zaznaczysz <b>FFT analyzer</b>.</p>";

  s += "<div class='box'><h3>Ustawienia globalne</h3>";
  s += "<div class='row'><label>Peak hold time (ms)</label><input name='peakHoldMs' type='number' min='50' max='2000' step='50' value='" + String(g_cfg.peakHoldTimeMs) + "'></div>";
  s += "</div>";

  // Presety na górze
  s += "<div class='box'><h3>Presety</h3>";
  s += "<div class='row'>";
  s += "<button formaction='/analyzerPreset' name='preset' value='0' type='submit'>Klasyczny</button>";
  s += "<button formaction='/analyzerPreset' name='preset' value='1' type='submit'>Nowoczesny</button>";
  s += "<button formaction='/analyzerPreset' name='preset' value='2' type='submit'>Kompaktowy</button>";
  s += "<button formaction='/analyzerPreset' name='preset' value='3' type='submit'>Retro</button>";
  s += "</div></div>";

  s += "<form method='POST'>";

  s += "<div class='box'><h3>Styl 5 (Słupkowy z segmentami)</h3>";
  s += "<div class='row'><label>bar width</label><input name='s5w' type='number' min='2' max='30' value='" + String(g_cfg.s5_barWidth) + "'></div>";
  s += "<div class='row'><label>bar gap</label><input name='s5g' type='number' min='0' max='20' value='" + String(g_cfg.s5_barGap) + "'></div>";
  s += "<div class='row'><label>segments</label><input name='s5seg' type='number' min='4' max='48' value='" + String(g_cfg.s5_segments) + "'></div>";
  s += "<div class='row'><label>fill (0.1..1)</label><input step='0.05' name='s5fill' type='number' min='0.1' max='1.0' value='" + String(g_cfg.s5_fill,2) + "'></div>";
  s += "<div class='row'><label>show peaks</label><input name='s5peaks' type='checkbox' value='1' " + String(g_cfg.s5_showPeaks ? "checked" : "") + "></div>";
  s += "</div>";

  s += "<div class='box'><h3>Styl 6 (Cienkie kreski)</h3>";
  s += "<div class='row'><label>gap</label><input name='s6g' type='number' min='0' max='10' value='" + String(g_cfg.s6_gap) + "'></div>";
  s += "<div class='row'><label>shrink</label><input name='s6sh' type='number' min='0' max='5' value='" + String(g_cfg.s6_shrink) + "'></div>";
  s += "<div class='row'><label>fill (0.1..1)</label><input step='0.05' name='s6fill' type='number' min='0.1' max='1.0' value='" + String(g_cfg.s6_fill,2) + "'></div>";
  s += "<div class='row'><label>seg min</label><input name='s6min' type='number' min='4' max='48' value='" + String(g_cfg.s6_segMin) + "'></div>";
  s += "<div class='row'><label>seg max</label><input name='s6max' type='number' min='4' max='48' value='" + String(g_cfg.s6_segMax) + "'></div>";
  s += "<div class='row'><label>show peaks</label><input name='s6peaks' type='checkbox' value='1' " + String(g_cfg.s6_showPeaks ? "checked" : "") + "></div>";
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
    Serial.print("Display levels: ");
    for (int i = 0; i < EQ_BANDS; i++) {
      Serial.print(levels[i], 2);
      if (i < EQ_BANDS - 1) Serial.print(" ");
    }
    Serial.println();
    lastDebugDisplay = now;
  }

  // Przepisujemy do eqLevel/eqPeak w skali 0..100 dla rysowania słupków
  // Jeśli mute - animacja opadania słupków
  static uint8_t muteLevel[EQ_BANDS] = {0}; // Zapamietane poziomy dla animacji mute
  
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
    } else {
      float lv = levels[i];
      float pk = peaks[i];
      if (lv < 0.0f) lv = 0.0f;
      if (lv > 1.0f) lv = 1.0f;
      if (pk < 0.0f) pk = 0.0f;
      if (pk > 1.0f) pk = 1.0f;

      uint8_t newLevel = (uint8_t)(lv * 100.0f + 0.5f);
      eqLevel[i] = newLevel;
      eqPeak[i] = (uint8_t)(pk * 100.0f + 0.5f);
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

  // Wartość głośności
  u8g2.setFont(u8g2_font_5x8_mr);
  u8g2.setCursor(iconX + 14, 10);
  u8g2.print(volumeValue);

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
      
      if (segTop <= segBottom) {
        u8g2.drawBox(x, segTop, barWidth, segmentHeight);  // Wszystkie segmenty mają identyczną wysokość
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
        u8g2.drawBox(x, peakY, barWidth, 1);
      }
    }
  }

  // Komunikat o wyciszeniu – na środku
  if (volumeMute)
  {
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setDrawColor(0);
    u8g2.drawBox(60, (64/2) - 10, 256-120, 20);  // SCREEN_HEIGHT = 64, SCREEN_WIDTH = 256
    u8g2.setDrawColor(1);
    u8g2.setCursor((64/2) - 30, (64/2) + 4);  // SCREEN_HEIGHT = 64
    u8g2.print("MUTED");
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

  // Konwersja poziomów - jeśli mute to animacja opadania słupków
  static uint8_t muteLevel6[EQ_BANDS] = {0}; // Zapamietane poziomy dla animacji mute
  
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
    } else {
      float lv = levels[i];
      float pk = peaks[i];
      if (lv < 0.0f) lv = 0.0f;
      if (lv > 1.0f) lv = 1.0f;
      if (pk < 0.0f) pk = 0.0f;
      if (pk > 1.0f) pk = 1.0f;

      uint8_t newLevel = (uint8_t)(lv * 100.0f + 0.5f);
      eqLevel[i] = newLevel;
      eqPeak[i] = (uint8_t)(pk * 100.0f + 0.5f);
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

  u8g2.setFont(u8g2_font_5x8_mr);
  u8g2.setCursor(iconX + 14, 10);
  u8g2.print(volumeValue);

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
        u8g2.drawBox(x, segTop, barWidth, segmentHeight);  // Wszystkie segmenty mają identyczną wysokość
      }
    }

    if (peakSeg > 0)
    {
      uint8_t ps = peakSeg - 1;
      int16_t peakSegBottom = eqBottomY - (ps * (segmentHeight + segmentGap));
      int16_t peakY = peakSegBottom - segmentHeight + 1 - 2; // 2 piksele nad segmentem
      
      if (peakY >= eqTopY && peakY <= eqBottomY) {
        u8g2.drawBox(x, peakY, barWidth, 1);  // 1 piksel wysokości peak - cienka kreska
      }
    }
  }

  if (volumeMute)
  {
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setDrawColor(0);
    u8g2.drawBox(60, (64/2) - 10, 256-120, 20);  // SCREEN_HEIGHT = 64, SCREEN_WIDTH = 256
    u8g2.setDrawColor(1);
    u8g2.setCursor((64/2) - 30, (64/2) + 4);  // SCREEN_HEIGHT = 64
    u8g2.print("MUTED");
  }

  u8g2.sendBuffer();
}
// 
// NOWE STYLE ANALIZATORA 7 i 8  
// 

void vuMeterMode7() // Styl 7: Okr�g�y analizator
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
  
  // Okr�g�y analizator
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
// 
// FUNKCJE ZARZ�DZANIA PRESETAMI
// 

void analyzerApplyPreset(uint8_t presetId) {
  switch(presetId) {
    case 0: analyzerPresetClassic(); break;
    case 1: analyzerPresetModern(); break;
    case 2: analyzerPresetCompact(); break;
    case 3: analyzerPresetRetro(); break;
    default: break; // 4 = Custom - no changes
  }
  // Apply settings immediately
  analyzerSetStyle(g_cfg);
}

void analyzerSetStyleMode(uint8_t styleMode) {
  // Store mode for reference (this could be used later for limiting available styles)
  // For now, all styles 0-8 are always available
  // This function exists for future extensibility
}

uint8_t analyzerGetMaxDisplayMode() {
  return 9;
}

bool analyzerIsStyleAvailable(uint8_t style) {
  return (style >= 0 && style <= 9);
}

uint8_t analyzerGetAvailableStylesMode() {
  return 2; // Always return 2 = All styles available (0-4-5-6-7-8-9)
}

void analyzerPresetClassic() {
  g_cfg.peakHoldTimeMs = 200;
  g_cfg.s5_barWidth = 8;
  g_cfg.s5_barGap = 2;
  g_cfg.s5_segments = 32;
  g_cfg.s5_fill = 0.7f;
  g_cfg.s5_showPeaks = true;
  
  g_cfg.s6_gap = 1;
  g_cfg.s6_fill = 0.6f;
  g_cfg.s6_segMax = 40;
  g_cfg.s6_showPeaks = true;
  
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
  g_cfg.s9_smoothness = 40;
}

void analyzerPresetModern() {
  g_cfg.peakHoldTimeMs = 300;
  g_cfg.s5_barWidth = 12;
  g_cfg.s5_barGap = 4;
  g_cfg.s5_segments = 48;
  g_cfg.s5_fill = 0.8f;
  g_cfg.s5_showPeaks = true;
  
  g_cfg.s6_gap = 2;
  g_cfg.s6_fill = 0.7f;
  g_cfg.s6_segMax = 48;
  g_cfg.s6_showPeaks = true;
  
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
  g_cfg.s9_smoothness = 20;
}

void analyzerPresetCompact() {
  g_cfg.peakHoldTimeMs = 150;
  g_cfg.s5_barWidth = 6;
  g_cfg.s5_barGap = 1;
  g_cfg.s5_segments = 24;
  g_cfg.s5_fill = 0.5f;
  g_cfg.s5_showPeaks = false;
  
  g_cfg.s6_gap = 0;
  g_cfg.s6_fill = 0.5f;
  g_cfg.s6_segMax = 32;
  g_cfg.s6_showPeaks = false;
  
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
  g_cfg.s9_smoothness = 60;
}

void analyzerPresetRetro() {
  g_cfg.peakHoldTimeMs = 400;
  g_cfg.s5_barWidth = 10;
  g_cfg.s5_barGap = 6;
  g_cfg.s5_segments = 28;
  g_cfg.s5_fill = 0.6f;
  g_cfg.s5_showPeaks = true;
  
  g_cfg.s6_gap = 3;
  g_cfg.s6_fill = 0.6f;
  g_cfg.s6_segMax = 36;
  g_cfg.s6_showPeaks = true;
  
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
  g_cfg.s9_smoothness = 50;
}
