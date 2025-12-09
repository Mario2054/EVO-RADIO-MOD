#pragma once
#include <Arduino.h>

// Inicjalizacja ustawień analizatora (wywołaj w setup, po audio.begin() / init EQ)
void eqAnalyzerInit();

// Włączanie / wyłączanie analizatora z poziomu serwera www
// (np. po kliknięciu checkboxa na stronie)
void eqAnalyzerSetFromWeb(bool enabled);

// Rysowanie OLED – Styl 5 (pełne słupki + zegar + głośnik)
void vuMeterMode5();

// Rysowanie OLED – Styl 6 (cienkie „kreseczki” + peak + zegar)
void vuMeterMode6();
