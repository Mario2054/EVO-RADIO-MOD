#pragma once
#include <Arduino.h>
#include "EQ_FFTAnalyzer.h"

// Ten nagłówek daje main.cpp wszystko czego potrzebuje do strony /analyzer:
// - AnalyzerStyleCfg + get/set/load/save
// - HTML generator + JSON
//
// Uwaga: Nie ma tu żadnego AudioRuntimeEQ_Evo.h.

// Tryby dostępnych stylów analizatora
enum AnalyzerStyleModes {
  ANALYZER_STYLES_0_4_5 = 0,    // Style 0,1,2,3,4,5 (bez stylu 6)
  ANALYZER_STYLES_0_4_6 = 1,    // Style 0,1,2,3,4,6 (bez stylu 5) 
  ANALYZER_STYLES_0_4_5_6 = 2   // Style 0,1,2,3,4,5,6 (wszystkie)
};

// Predefiniowane style analizatora
enum AnalyzerPresets {
  PRESET_CLASSIC = 0,       // Klasyczny wygląd
  PRESET_MODERN = 1,        // Nowoczesny wygląd
  PRESET_COMPACT = 2,       // Kompaktowy wygląd
  PRESET_RETRO = 3,         // Retro wygląd
  PRESET_FLOATING = 4,      // Floating Peaks - Ulatujące szczyty
  PRESET_CUSTOM = 5         // Własny (edytowalny)
};

struct AnalyzerStyleCfg {
  // ---- Globalne ustawienia ----
  uint8_t availableStylesMode = ANALYZER_STYLES_0_4_5_6; // Które style są dostępne
  uint8_t currentPreset = PRESET_CLASSIC;                // Aktualny preset
  uint16_t peakHoldTimeMs = 40;                          // Czas zatrzymania peak na szczycie (ms) 50-2000 - 5x szybciej
  
  // ---- Styl 5 - Słupkowy ----
  uint8_t s5_barWidth = 10;     // szerokość słupka (px) 4-16
  uint8_t s5_barGap   = 6;      // przerwa między słupkami (px) 1-8
  uint8_t s5_segments = 32;     // ilość segmentów w pionie 16-64
  uint8_t s5_segHeight = 2;     // wysokość segmentu (px) 1-4
  float   s5_fill     = 0.60f;  // wypełnienie segmentu (0.1..1.0)
  uint8_t s5_peakHeight = 2;    // wysokość peak hold (px) 1-4
  uint8_t s5_peakGap = 1;       // odstęp peak od słupka (px) 0-3
  bool    s5_showPeaks = true;  // czy pokazywać peaks
  uint8_t s5_smoothness = 50;   // wygładzanie 10-90 (10=szybkie, 90=wolne)
  uint8_t s5_barBrightness = 255;  // jasność słupków 0-255 (0=niewidoczne, 255=pełna)
  uint8_t s5_peakBrightness = 255; // jasność peak kreski 0-255

  // ---- Styl 6 - Segmentowy ----
  uint8_t s6_width  = 10;       // szerokość słupka (px) 4-20
  uint8_t s6_gap    = 1;        // przerwa między kolumnami (px) 0-4
  uint8_t s6_shrink = 1;        // ile px odjąć z szerokości 0-3
  float   s6_fill   = 0.60f;    // wypełnienie segmentu (0.1..1.0)
  uint8_t s6_segMin = 4;        // min segmentów 2-8
  uint8_t s6_segMax = 48;       // max segmentów 16-64
  uint8_t s6_segHeight = 2;     // wysokość segmentu (px) 1-4
  uint8_t s6_segGap = 1;        // odstęp między segmentami (px) 0-2
  bool    s6_showPeaks = true;  // czy pokazywać peaks
  uint8_t s6_smoothness = 40;   // wygładzanie 10-90
  uint8_t s6_barBrightness = 255;  // jasność słupków 0-255 (0=niewidoczne, 255=pełna)
  uint8_t s6_peakBrightness = 255; // jasność peak kreski 0-255

  // ---- Styl 7 - Nowy: Okrągły ----
  uint8_t s7_circleRadius = 3;  // promień kółek (px) 2-6
  uint8_t s7_circleGap = 2;     // odstęp między kółkami (px) 1-4
  bool    s7_filled = true;     // wypełnione czy tylko obramowanie
  uint8_t s7_maxHeight = 50;    // maksymalna wysokość (px) 20-60
  
  // ---- Styl 8 - Nowy: Liniowy ----
  uint8_t s8_lineThickness = 2; // grubość linii (px) 1-4
  uint8_t s8_lineGap = 4;       // odstęp między liniami (px) 2-8
  bool    s8_gradient = true;   // gradientowe wypełnienie
  uint8_t s8_maxHeight = 55;    // maksymalna wysokość (px) 20-60
  
  // ---- Styl 9 - Nowy: Spadające gwiazdki jak śnieg ----
  uint8_t s9_starRadius = 20;   // nie używane (zachowane dla kompatybilności)
  uint8_t s9_armWidth = 3;      // nie używane (zachowane dla kompatybilności)
  uint8_t s9_armLength = 15;    // maksymalny rozmiar gwiazdek (px) 8-25
  uint8_t s9_spikeLength = 5;   // długość dyszek na końcach (px) 2-10
  bool    s9_showSpikes = true; // czy pokazywać dyszki na końcach gwiazdek
  bool    s9_filled = true;     // wypełnione środki gwiazdek czy tylko kontury
  uint8_t s9_centerSize = 4;    // minimalny rozmiar gwiazdek (px) 2-8
  uint8_t s9_smoothness = 30;   // wygładzanie ruchu (10-90)
  
  // ---- Styl 10 - Nowy: Floating Peaks (Ulatujące szczyty) ----
  uint8_t s10_barWidth = 12;        // szerokość słupka (px) 4-20
  uint8_t s10_barGap = 2;           // przerwa między słupkami (px) 1-6  
  uint8_t s10_segmentHeight = 2;    // wysokość segmentu (px) 1-4
  uint8_t s10_segmentGap = 1;       // przerwa między segmentami (px) 0-3
  uint8_t s10_maxPeaks = 3;         // max ilość latających peaków na słupek 1-5
  uint8_t s10_peakHoldTime = 8;     // czas zatrzymania peak (ramki) 0-100
  uint8_t s10_peakFloatSpeed = 8;   // prędkość ulatywania (px/frame) 3-15
  uint8_t s10_peakFadeSteps = 12;   // kroki zanikania peak 5-20
  uint8_t s10_trailLength = 6;      // długość śladu peak (px) 3-12
  bool    s10_showTrails = true;    // czy pokazywać ślady peak
  uint8_t s10_smoothness = 45;      // wygładzanie słupków (10-90)
  uint8_t s10_barBrightness = 200;  // jasność słupków 0-255 (ściemnione)
  uint8_t s10_peakBrightness = 255; // jasność peak 0-255 (rozjaśnione)
  uint8_t s10_trailBrightness = 180;// jasność śladów 0-255
  uint8_t s10_peakMinHeight = 3;    // minimalna wysokość peak (px) 1-8
  uint8_t s10_floatHeight = 15;     // wysokość strefy ulatywania (px) 8-25
  bool    s10_enableAnimation = true; // włączenie animacji ulatywania
};

AnalyzerStyleCfg analyzerGetStyle();
void analyzerSetStyle(const AnalyzerStyleCfg& c);

// Wczytaj/zapisz z STORAGE (SD/SPIFFS) – plik tekstowy /analyzer.cfg
void analyzerStyleLoad();
void analyzerStyleSave();

// Funkcje do zapisu/odczytu stylu jako String (dla main.cpp)
String analyzerStyleToSaveString();
void analyzerStyleLoadFromString(const String& content);
String analyzerGetLoadStatus();
int analyzerGetLoadedParamsCount();

// Strona /analyzer i podgląd /analyzerCfg
String analyzerBuildHtmlPage();
String analyzerStyleToJson();

// Funkcje analizatora - style główne
void eqAnalyzerSetFromWeb(bool enabled);
void vuMeterMode5();
void vuMeterMode6();
void vuMeterMode7();  // Nowy styl: Okrągły
void vuMeterMode8();  // Nowy styl: Liniowy
void vuMeterMode9();  // Nowy styl: Spadające gwiazdki jak śnieg
void vuMeterMode10(); // Nowy styl: Floating Peaks - Ulatujące szczyty

// Funkcje presetów i konfiguracji
void analyzerApplyPreset(uint8_t presetId);
void analyzerSetStyleMode(uint8_t styleMode);
uint8_t analyzerGetAvailableStylesMode();
uint8_t analyzerGetMaxDisplayMode();
bool analyzerIsStyleAvailable(uint8_t style);
uint32_t analyzerGetPeakHoldTime();

// Presety do szybkiego wyboru
void analyzerPresetClassic();    // Preset 0: Klasyczny
void analyzerPresetModern();     // Preset 1: Nowoczesny
void analyzerPresetCompact();    // Preset 2: Kompaktowy  
void analyzerPresetRetro();      // Preset 3: Retro
void analyzerPresetFloatingPeaks(); // Preset 4: Floating Peaks - Ulatujące szczyty

// ======================= ZOPTYMALIZOWANE FUNKCJE UI =======================

// Pobieranie gotowych poziomów bez obliczeń FFT w UI thread
void eq_ui_fetch_levels();                              // Fetch poziomów (wywołać przed rysowaniem)
void eq_ui_get_levels(float out_levels[16]);            // Pobierz levels (float 0.0-1.0)
void eq_ui_get_peaks(float out_peaks[16]);              // Pobierz peaks (float 0.0-1.0)
void eq_ui_get_levels_uint8(uint8_t out_levels[16]);    // Pobierz levels (uint8 0-255)
void eq_ui_get_peaks_uint8(uint8_t out_peaks[16]);      // Pobierz peaks (uint8 0-255)

// ============================================================================

// Extern deklaracje zmiennych globalnych - zdefiniowane w EQ_AnalyzerDisplay.cpp
extern uint8_t eqLevel[EQ_BANDS];      // Current bar levels 0-100  
extern uint8_t eqPeak[EQ_BANDS];       // Peak positions 0-100
