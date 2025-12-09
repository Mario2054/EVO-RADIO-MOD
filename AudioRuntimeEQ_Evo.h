#pragma once
#include <stdint.h>
#include <stdbool.h>

// Liczba pasm korektora / analizatora
static const int RUNTIME_EQ_BANDS = 16;

// Struktura opisująca pasmo analizatora
struct EQBandInfo {
    int band;           // numer pasma (0-15)
    int frequency;      // częstotliwość środkowa [Hz]
    float level;        // poziom (0.0 .. 1.0)
};

// Częstotliwości środkowe pasm (Hz)
static const int EQ_BAND_FREQUENCIES[RUNTIME_EQ_BANDS] = {
    20, 31, 50, 79, 126, 200, 316, 502, 
    796, 1261, 2000, 3169, 5023, 7962, 12619, 20000
};

// Domyślna czułość analizatora (mnożnik poziomów, 0.1 - 2.0)
// Wartość < 1.0 obniża wyświetlane poziomy
static const float DEFAULT_ANALYZER_SENSITIVITY = 0.3f;

// Maksymalny poziom sygnału przed normalizacją (zapobiega clippingowi w analizatorze)
static const float ANALYZER_MAX_INPUT_LEVEL = 0.5f;  // mocniejsze tłumienie wejścia analizatora

// Zakres automatycznej regulacji poziomu (AGC dla analizatora)
static const float ANALYZER_AGC_TARGET = 0.4f;  // niższy docelowy max poziom (więcej zapasu przed 1.0)
static const float ANALYZER_AGC_SPEED  = 0.95f; // szybkość adaptacji (0.9-0.99)

// Ustaw wszystkie wzmocnienia pasm (dB)
void eq_set_all_gains(const float in[RUNTIME_EQ_BANDS]);

// Odczytaj aktualne wzmocnienia pasm (dB)
void eq_get_all_gains(float out[RUNTIME_EQ_BANDS]);

// Odczytaj poziomy analizatora (0..1 dla 16 pasm)
void eq_get_analyzer_levels(float out[RUNTIME_EQ_BANDS]);

// NOWE: Zwraca poziomy peak hold (osobne kreski szczytowe)
void eq_get_analyzer_peaks(float out[RUNTIME_EQ_BANDS]);

// Odczytaj kompletne informacje o pasmach (pasmo, częstotliwość, poziom)
void eq_get_analyzer_bands(EQBandInfo out[RUNTIME_EQ_BANDS]);

// Odczytaj statystyki analizatora
void eq_get_analyzer_stats(float* levelsSumOut, float* maxLevelOut);

// Ustaw normalizację sygnału wejściowego dla analizatora
// Gdy true, sygnał jest skalowany przed FFT aby uniknąć clippingu
void eq_set_analyzer_normalize(bool normalize);

// Sprawdź czy normalizacja analizatora jest włączona
bool eq_get_analyzer_normalize();

// Ustaw automatyczną regulację wzmocnienia analizatora (AGC)
// Dynamicznie dostosowuje czułość aby max poziom był ~60%
void eq_set_analyzer_agc(bool enableAgc);

// Sprawdź czy AGC analizatora jest włączone
bool eq_get_analyzer_agc();

// Ustaw czułość analizatora (0.1 - 2.0, domyślnie 0.3)
// Niższe wartości = niższe słupki na wyświetlaczu
void eq_set_analyzer_sensitivity(float sensitivity);

// Odczytaj aktualną czułość analizatora
float eq_get_analyzer_sensitivity();

// Ustaw czy analizator ma pracować przed wzmocnieniem wyjściowym (true)
// czy po wzmocnieniu (false). Domyślnie: true
void eq_set_analyzer_pre_gain(bool preGain);

// Sprawdź czy analizator jest przed wzmocnieniem
bool eq_get_analyzer_pre_gain();

// Sprawdź czy audio jest odtwarzane
bool eq_is_audio_playing();

// Debug: Odczytaj stan bufora analizatora
void eq_get_analyzer_debug(int* samplesFilledOut, int* writeIndexOut);

// Debug: Odczytaj surowe wartości z bufora FFT (dla diagnostyki)
void eq_get_fft_buffer_sample(int index, float* valueOut);

// Callback wołany przez ESP32-audioI2S
void audio_process_i2s(int16_t* outBuff, int32_t validSamples, bool *continueI2S);

// Odczytaj informacje o clippingu w analizatorze
// Zwraca procent sampli z clippingiem (0.0 - 1.0)
float eq_get_analyzer_clipping();

// Resetuj statystyki clippingu
void eq_reset_analyzer_clipping();

#ifdef __cplusplus
extern "C" {
#endif

// Globalne flagi (zdefiniowane w main.cpp):
//  • eqEnabled         – włącza/wyłącza działanie korektora DSP
//  • eqAnalyzerEnabled – włącza/wyłącza obliczanie analizatora widma
extern bool eqEnabled;
extern bool eqAnalyzerEnabled;

#ifdef __cplusplus
}
#endif

// Globalne zmienne analizatora (zdefiniowane w AudioRuntimeEQ_Evo.cpp)
extern float eqAnalyzerSensitivity;
extern bool  eqAnalyzerPreGain;
extern bool  eqAnalyzerNormalize;
extern bool  eqAnalyzerAGC;
