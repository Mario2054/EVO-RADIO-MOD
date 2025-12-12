#pragma once
#include <Arduino.h>

// Inicjalizacja parametrów analizatora (opcjonalnie)
void eqAnalyzerInit();

// Włącza/wyłącza analizator z WWW i ustawia zakres stylów (0..4 lub 0..6)
void eqAnalyzerSetFromWeb(bool enabled);

// Style 5 i 6 (analizator)
void vuMeterMode5();
void vuMeterMode6();

