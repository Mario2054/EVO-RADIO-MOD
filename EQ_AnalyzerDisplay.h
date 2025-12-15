#pragma once
#include <Arduino.h>

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
  PRESET_CUSTOM = 4         // Własny (edytowalny)
};

struct AnalyzerStyleCfg {
  // ---- Globalne ustawienia ----
  uint8_t availableStylesMode = ANALYZER_STYLES_0_4_5_6; // Które style są dostępne
  uint8_t currentPreset = PRESET_CLASSIC;                // Aktualny preset
  uint16_t peakHoldTimeMs = 200;                         // Czas zatrzymania peak na szczycie (ms) 50-2000
  
  // ---- Styl 5 - Słupkowy ----
  uint8_t s5_barWidth = 10;     // szerokość słupka (px) 4-16
  uint8_t s5_barGap   = 6;      // przerwa między słupkami (px) 1-8
  uint8_t s5_segments = 32;     // ilość segmentów w pionie 16-64
  float   s5_fill     = 0.60f;  // wypełnienie segmentu (0.1..1.0)
  uint8_t s5_peakHeight = 2;    // wysokość peak hold (px) 1-4
  uint8_t s5_peakGap = 1;       // odstęp peak od słupka (px) 0-3
  bool    s5_showPeaks = true;  // czy pokazywać peaks
  uint8_t s5_smoothness = 50;   // wygładzanie 10-90 (10=szybkie, 90=wolne)

  // ---- Styl 6 - Segmentowy ----
  uint8_t s6_gap    = 1;        // przerwa między kolumnami (px) 0-4
  uint8_t s6_shrink = 1;        // ile px odjąć z szerokości 0-3
  float   s6_fill   = 0.60f;    // wypełnienie segmentu (0.1..1.0)
  uint8_t s6_segMin = 4;        // min segmentów 2-8
  uint8_t s6_segMax = 48;       // max segmentów 16-64
  uint8_t s6_segHeight = 2;     // wysokość segmentu (px) 1-4
  uint8_t s6_segGap = 1;        // odstęp między segmentami (px) 0-2
  bool    s6_showPeaks = true;  // czy pokazywać peaks
  uint8_t s6_smoothness = 40;   // wygładzanie 10-90

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
};

AnalyzerStyleCfg analyzerGetStyle();
void analyzerSetStyle(const AnalyzerStyleCfg& c);

// Wczytaj/zapisz z STORAGE (SD/SPIFFS) – plik tekstowy /analyzer.cfg
void analyzerStyleLoad();
void analyzerStyleSave();

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
