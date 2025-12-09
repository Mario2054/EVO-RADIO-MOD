// AudioRuntimeEQ_Evo.cpp
// 16-pasmowy korektor + analizator FFT dla EVO

#include "AudioRuntimeEQ_Evo.h"

#include <Arduino.h>
#include <math.h>
#include "Audio.h"

#ifndef ENABLE_RUNTIME_EQ
#define ENABLE_RUNTIME_EQ 1
#endif

#ifndef ENABLE_RUNTIME_ANALYZER
#define ENABLE_RUNTIME_ANALYZER 1
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ─────────────────────────────────────
// Zewnętrzne globalne z main.cpp
// ─────────────────────────────────────
extern Audio audio;
extern float eqBandGains[RUNTIME_EQ_BANDS];
extern bool  eqEnabled;
extern bool  eqAnalyzerEnabled;
extern float eqAnalyzerSensitivity;
extern bool  eqAnalyzerPreGain;
extern bool  eqAnalyzerNormalize;
extern bool  eqAnalyzerAGC;
extern uint8_t volumeValue;  // dodajemy extern do volumeValue
extern uint8_t maxVolume;   // from main.cpp - max volume steps (21 or 42)

// ─────────────────────────────────────
// Konfiguracja korektora
// ─────────────────────────────────────
static const int EQ_BANDS          = RUNTIME_EQ_BANDS;
static const int EQ_BANDS_FOR_FLAC = 8;   // mniej pasm dla FLAC
static const int CODEC_FLAC        = 5;   // zgodnie z enumem z Audio.h
static const int CODEC_AAC         = 2;   // dodajemy AAC (jeśli enum = 2)
static const int CODEC_PCM         = 0;   // dodane: PCM (surowy dźwięk)

//#define ANALYZER_AGC_TARGET   (0.1f)   // cel dla AGC (przy 0.0 dBFS)
//#define ANALYZER_AGC_SPEED    (0.01f)  // szybkość reakcji AGC (0.01 = wolno, 0.001 = szybko)

static bool  eq_inited  = false;
static float fs_rate    = 48000.0f;
static bool  eq_is_flac = false;
static int   last_codec = -1;

// częstotliwości środkowe 16 pasm (log 20..20k)
static float center_freqs[EQ_BANDS];

// ostatnie zastosowane gainy (dla sprawdzania zmian)
static float last_gains[EQ_BANDS];

// współczynniki biquadów
static float b0_c[EQ_BANDS], b1_c[EQ_BANDS], b2_c[EQ_BANDS], a1_c[EQ_BANDS], a2_c[EQ_BANDS];

// stan filtrów: [pasmo][kanał]
struct BiquadState {
    float x1, x2;
    float y1, y2;
};
static BiquadState state[EQ_BANDS][2];

// ─────────────────────────────────────
// Konfiguracja analizatora (FFT)
// ─────────────────────────────────────
static const int FFT_SIZE = 256;          // 256-punktowe FFT
static float fftBuffer[FFT_SIZE];         // pierścieniowy bufor mono
static int   fftWriteIndex     = 0;
static int   fftSamplesFilled  = 0;

// wygładzony poziom 16 pasm (0..1)
static float analyzerSmooth[RUNTIME_EQ_BANDS] = {0};

// NOWE: Peak hold - osobna kreska szczytowa dla każdego pasma
static float analyzerPeak[RUNTIME_EQ_BANDS] = {0};           // poziom peak hold
static int   analyzerPeakHoldTime[RUNTIME_EQ_BANDS] = {0};  // czas zatrzymania (w cyklach FFT)

// Nowe zmienne dla kontroli poziomu analizatora
static float analyzerCurrentSensitivity = DEFAULT_ANALYZER_SENSITIVITY;
static float analyzerAGCGain = 1.0f;  // dynamiczne wzmocnienie AGC
static int   analyzerClippingSamples = 0;
static int   analyzerTotalSamples = 0;
static float analyzerPeakLevel = 0.0f;

// Nowe: osobne współczynniki smoothingu dla każdego pasma
static float analyzerSmoothAttack[RUNTIME_EQ_BANDS];   // szybkość narastania
static float analyzerSmoothRelease[RUNTIME_EQ_BANDS];  // szybkość opadania

// robocze bufory FFT
static float fft_re[FFT_SIZE];
static float fft_im[FFT_SIZE];
// Precomputed tables for analyzer to avoid heavy math in the hot path
static bool  analyzerTablesInited = false;
// Hann window coefficients (FFT_SIZE)
static float analyzerHann[FFT_SIZE];
// Logarithmic band -> FFT bin mapping
static int   analyzerBandBinStart[RUNTIME_EQ_BANDS];
static int   analyzerBandBinEnd[RUNTIME_EQ_BANDS];
// Precomputed center frequencies per band (for freqGain and smoothing shaping)
static float analyzerBandCenterFreq[RUNTIME_EQ_BANDS];

// Initialize analyzer helper tables.
// This uses floating point math (powf, cosf) only once at startup,
// instead of every frame in the real‑time path.
static void analyzer_init_tables()
{
    if (analyzerTablesInited) {
        return;
    }
    analyzerTablesInited = true;

    // Hann window
    for (int i = 0; i < FFT_SIZE; ++i) {
        analyzerHann[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / (FFT_SIZE - 1));
    }

    const int   bands = RUNTIME_EQ_BANDS;
    const float f0    = 20.0f;
    const float f1    = 20000.0f;
    const float binHz = fs_rate / (float)FFT_SIZE;

    for (int b = 0; b < bands; ++b) {
        // log‑space start/end frequency for band
        float t_start    = (float)b / (float)bands;
        float t_end      = (float)(b + 1) / (float)bands;
        float freq_start = f0 * powf(f1 / f0, t_start);
        float freq_end   = f0 * powf(f1 / f0, t_end);

        if (freq_start < 0.0f) freq_start = 0.0f;
        if (freq_end   < 0.0f) freq_end   = 0.0f;

        int idxStart = (int)(freq_start / binHz);
        int idxEnd   = (int)(freq_end   / binHz);

        if (idxStart < 0) idxStart = 0;
        if (idxEnd >= FFT_SIZE / 2) idxEnd = FFT_SIZE / 2 - 1;
        if (idxEnd < idxStart) idxEnd = idxStart;

        analyzerBandBinStart[b] = idxStart;
        analyzerBandBinEnd[b]   = idxEnd;

        // center frequency for shaping releases / freqGain
        float t_center = (float)b / (float)(bands - 1);
        analyzerBandCenterFreq[b] = f0 * powf(f1 / f0, t_center);
    }
}



// ─────────────────────────────────────
// Pomocnicze – EQ
// ─────────────────────────────────────

static void compute_center_freqs() {
    const float f0 = 20.0f;
    const float f1 = 20000.0f;
    for (int i = 0; i < EQ_BANDS; ++i) {
        float t = (float)i / (float)(EQ_BANDS - 1);
        center_freqs[i] = f0 * powf(f1 / f0, t);
    }
}

static void recompute_coeffs(const float gains_db[EQ_BANDS]) {
    const float Q = 1.0f;  // stałe Q dla prostego grafika

    for (int i = 0; i < EQ_BANDS; ++i) {
        float gdb = gains_db[i];

        // blisko 0 dB → bypass
        if (fabsf(gdb) < 0.25f) {
            b0_c[i] = 1.0f;
            b1_c[i] = 0.0f;
            b2_c[i] = 0.0f;
            a1_c[i] = 0.0f;
            a2_c[i] = 0.0f;
            continue;
        }

        float A     = powf(10.0f, gdb / 40.0f);
        float w0    = 2.0f * (float)M_PI * center_freqs[i] / fs_rate;
        float cosw0 = cosf(w0);
        float sinw0 = sinf(w0);
        float alpha = sinw0 / (2.0f * Q);

        // peaking EQ (RBJ Audio Cookbook)
        float a0 = 1.0f + alpha / A;
        float b0 = 1.0f + alpha * A;
        float b1 = -2.0f * cosw0;
        float b2 = 1.0f - alpha * A;
        float a1 = -2.0f * cosw0;
        float a2 = 1.0f - alpha / A;

        b0_c[i] = b0 / a0;
        b1_c[i] = b1 / a0;
        b2_c[i] = b2 / a0;
        a1_c[i] = a1 / a0;
        a2_c[i] = a2 / a0;
    }
}

// ─────────────────────────────────────
// Proste FFT (Cooley–Tukey, in-place)
// ─────────────────────────────────────
static void fft_complex(float re[], float im[], int N) {
    // bit-reverse
    int j = 0;
    for (int i = 1; i < N; ++i) {
        int bit = N >> 1;
        for (; j & bit; bit >>= 1) {
            j &= ~bit;
        }
        j |= bit;
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }

    // motylki
    for (int len = 2; len <= N; len <<= 1) {
        float ang = -2.0f * (float)M_PI / (float)len;
        float wlen_cos = cosf(ang);
        float wlen_sin = sinf(ang);

        for (int i = 0; i < N; i += len) {
            float w_cos = 1.0f;
            float w_sin = 0.0f;
            int half = len >> 1;
            for (int k = 0; k < half; ++k) {
                int u = i + k;
                int v = i + k + half;

                float ur = re[u];
                float ui = im[u];
                float vr = re[v] * w_cos - im[v] * w_sin;
                float vi = re[v] * w_sin + im[v] * w_cos;

                re[u] = ur + vr;
                im[u] = ui + vi;
                re[v] = ur - vr;
                im[v] = ui - vi;

                float nw_cos = w_cos * wlen_cos - w_sin * wlen_sin;
                float nw_sin = w_cos * wlen_sin + w_sin * wlen_cos;
                w_cos = nw_cos;
                w_sin = nw_sin;
            }
        }
    }
}

// Liczy analizator z aktualnego bufora próbek
static void analyzer_compute_from_buffer() {
#if ENABLE_RUNTIME_ANALYZER
    if (!eqAnalyzerEnabled) {
        return;
    }
#else
    (void)eqAnalyzerEnabled;
    return;
#endif

    if (fftSamplesFilled < FFT_SIZE) {
        return;
    }

    // Initialize analyzer lookup tables (Hann window + band/bin mapping) once
    analyzer_init_tables();

    // Zrzut pierścieniowego bufora do lokalnej tablicy
    float in[FFT_SIZE];
    int idx = fftWriteIndex;

    // Znajdź peak level dla normalizacji/AGC
    float peakInBuffer = 0.0f;
    for (int i = 0; i < FFT_SIZE; ++i) {
        in[i] = fftBuffer[idx];
        float absVal = fabsf(in[i]);
        if (absVal > peakInBuffer) peakInBuffer = absVal;
        idx++;
        if (idx >= FFT_SIZE) idx = 0;
    }

    // Aktualizacja peak level (z wolnym spadkiem)
    analyzerPeakLevel = analyzerPeakLevel * 0.95f + peakInBuffer * 0.05f;

    // AGC - dynamiczna regulacja wzmocnienia
    if (eqAnalyzerAGC && analyzerPeakLevel > 0.01f) {
        float targetGain = ANALYZER_AGC_TARGET / analyzerPeakLevel;
        if (targetGain > 3.0f) targetGain = 3.0f;  // max 3x wzmocnienie
        if (targetGain < 0.3f) targetGain = 0.3f;  // min 0.3x
        analyzerAGCGain = analyzerAGCGain * ANALYZER_AGC_SPEED + targetGain * (1.0f - ANALYZER_AGC_SPEED);
    } else {
        analyzerAGCGain = 1.0f;
    }

    // Normalizacja sygnału
    float normFactor = 1.0f;
    if (eqAnalyzerNormalize && peakInBuffer > ANALYZER_MAX_INPUT_LEVEL) {
        normFactor = ANALYZER_MAX_INPUT_LEVEL / peakInBuffer;
    }

    // Zastosuj normalizację do bufora wejściowego
    if (normFactor != 1.0f) {
        for (int i = 0; i < FFT_SIZE; ++i) {
            in[i] *= normFactor;
        }
    }

    // Okno Hann + kopiowanie do re/im – korzystamy z precomputed analyzerHann[]
    for (int i = 0; i < FFT_SIZE; ++i) {
        fft_re[i] = in[i] * analyzerHann[i];
        fft_im[i] = 0.0f;
    }

    // FFT
    fft_complex(fft_re, fft_im, FFT_SIZE);

    // Magnituda dla połowy widma (przybliżenie bez sqrtf dla szybkości)
    const int N2 = FFT_SIZE / 2;
    float mags[N2];
    const float invFFTSize = 1.0f / (float)FFT_SIZE;
    for (int i = 0; i < N2; ++i) {
        float re = fft_re[i];
        float im = fft_im[i];
        // Przybliżenie: |re| + |im| zamiast sqrt(re^2+im^2) – dużo szybsze, do wizualizacji wystarczy
        mags[i] = (fabsf(re) + fabsf(im)) * invFFTSize;
    }

    // Podział na 16 pasm logarytmicznych – używamy precomputed mapy binów
    const int bands = RUNTIME_EQ_BANDS;

    for (int b = 0; b < bands; ++b) {
        int start = analyzerBandBinStart[b];
        int end   = analyzerBandBinEnd[b];

        // dodatkowe zabezpieczenia
        if (start >= N2) start = N2 - 1;
        if (end > N2) end = N2;
        if (start >= end) end = start + 1;

        float sumSq = 0.0f;
        int   count = 0;
        for (int k = start; k < end; ++k) {
            float v = mags[k];
            sumSq += v * v;
            ++count;
        }

        float rms = (count > 0) ? sqrtf(sumSq / count) : 0.0f;
        if (rms < 1e-9f) rms = 1e-9f;

        float db = 20.0f * log10f(rms);
        if (db < -80.0f) db = -80.0f;
        if (db >   0.0f) db =   0.0f;

        float norm = (db + 80.0f) / 80.0f;
        norm = sqrtf(norm);

        // Częstotliwościowe wzmocnienie - OBNIŻONE dla lepszej dynamiki (max ~85%)
        float centerFreq = analyzerBandCenterFreq[b];

        float freqGain;
        if (centerFreq < 35.0f) {
            freqGain = 0.8f;   // było 1.3f (-38%)
        } else if (centerFreq < 55.0f) {
            freqGain = 0.9f;   // było 1.45f (-38%)
        } else if (centerFreq < 90.0f) {
            freqGain = 1.0f;   // było 1.6f (-38%)
        } else if (centerFreq < 150.0f) {
            freqGain = 1.1f;   // było 1.75f (-37%)
        } else if (centerFreq < 250.0f) {
            freqGain = 1.15f;  // było 1.8f (-36%)
        } else if (centerFreq < 450.0f) {
            freqGain = 1.05f;  // było 1.7f (-38%)
        } else if (centerFreq < 750.0f) {
            freqGain = 1.0f;   // było 1.6f (-38%)
        } else if (centerFreq < 1300.0f) {
            freqGain = 1.1f;   // było 1.75f (-37%)
        } else if (centerFreq < 2200.0f) {
            freqGain = 1.15f;  // było 1.85f (-38%)
        } else if (centerFreq < 4000.0f) {
            freqGain = 1.25f;  // było 2.0f (-38%)
        } else if (centerFreq < 7000.0f) {
            freqGain = 1.45f;  // było 2.3f (-37%)
        } else if (centerFreq < 11000.0f) {
            freqGain = 1.7f;   // było 2.7f (-37%)
        } else if (centerFreq < 16000.0f) {
            freqGain = 1.9f;   // było 3.0f (-37%)
        } else {
            freqGain = 2.0f;   // było 3.2f (-38%)
        }

        // Zastosuj wszystkie wzmocnienia: czułość + AGC + częstotliwość
        norm *= analyzerCurrentSensitivity * analyzerAGCGain * freqGain;
        if (norm > 1.0f) norm = 1.0f;

        // ═══════════════════════════════════════════════════════════
        // KONFIGURACJA SŁUPKÓW - tutaj zmieniasz wartości!
        // ═══════════════════════════════════════════════════════════
        
        // --- 1. SMOOTHING DLA SŁUPKÓW (główne słupki) ---
        float attackSpeed = 1.0f;    // 100% = natychmiastowe narastanie
        float releaseSpeed;          // retention: 0.0 = instant, 1.0 = brak opadania
        
        // LINIA 1: Basy <80Hz
        if (centerFreq < 80.0f) {
            releaseSpeed = 0.60f;   // [EDYTUJ] 40% decay - im niższe, tym szybsze opadanie
        // LINIA 2: Niskie 80-200Hz
        } else if (centerFreq < 200.0f) {
            releaseSpeed = 0.50f;   // [EDYTUJ] 50% decay
        // LINIA 3: Niski środek 200-500Hz
        } else if (centerFreq < 500.0f) {
            releaseSpeed = 0.40f;   // [EDYTUJ] 60% decay
        // LINIA 4: Środek 500-1500Hz
        } else if (centerFreq < 1500.0f) {
            releaseSpeed = 0.30f;   // [EDYTUJ] 70% decay
        // LINIA 5: Górny środek 1500-5000Hz
        } else if (centerFreq < 5000.0f) {
            releaseSpeed = 0.20f;   // [EDYTUJ] 80% decay
        // LINIA 6: Wysokie 5-10kHz
        } else if (centerFreq < 10000.0f) {
            releaseSpeed = 0.10f;   // [EDYTUJ] 90% decay
        // LINIA 7: Ultra-wysokie >10kHz
        } else {
            releaseSpeed = 0.05f;   // [EDYTUJ] 95% decay
        }

        // --- 2. PEAK HOLD (osobna kreska szczytowa) ---
        
        // LINIA 8: Czas zatrzymania peak (w cyklach FFT, ~5ms każdy)
        const int PEAK_HOLD_TIME = 5;      // [EDYTUJ] 50 cykli = ~250ms zatrzymania
        
        // LINIA 9: Szybkość opadania peak po zatrzymaniu
        const float PEAK_DECAY_SPEED = 0.90f;  // [EDYTUJ] 0.98 = wolne, 0.90 = szybkie, 0.0 = instant
        
        // ═══════════════════════════════════════════════════════════
        // KONIEC KONFIGURACJI
        // ═══════════════════════════════════════════════════════════

        // Smoothing dla głównego słupka
        if (norm > analyzerSmooth[b]) {
            // Narastanie - natychmiastowe
            analyzerSmooth[b] = norm;
        } else {
            // Opadanie - z konfiguracją powyżej
            analyzerSmooth[b] = analyzerSmooth[b] * releaseSpeed + norm * (1.0f - releaseSpeed);
            
            // Hard drop dla bardzo niskich wartości
            if (analyzerSmooth[b] < 0.03f) {
                analyzerSmooth[b] = 0.0f;
            }
        }
        
        // Peak hold - osobna kreska szczytowa
        if (norm > analyzerPeak[b]) {
            // Nowy szczyt - ustaw i resetuj timer
            analyzerPeak[b] = norm;
            analyzerPeakHoldTime[b] = PEAK_HOLD_TIME;
        } else {
            // Sprawdź timer
            if (analyzerPeakHoldTime[b] > 0) {
                // Zatrzymanie - peak nie opada
                analyzerPeakHoldTime[b]--;
            } else {
                // Timer wygasł - peak zaczyna opadać
                analyzerPeak[b] *= PEAK_DECAY_SPEED;
                
                // Hard drop dla bardzo niskich wartości
                if (analyzerPeak[b] < 0.03f) {
                    analyzerPeak[b] = 0.0f;
                }
            }
        }
        
        // Ograniczenie do 0..1
        if (analyzerSmooth[b] < 0.0f) analyzerSmooth[b] = 0.0f;
        if (analyzerSmooth[b] > 1.0f) analyzerSmooth[b] = 1.0f;
        if (analyzerPeak[b] < 0.0f) analyzerPeak[b] = 0.0f;
        if (analyzerPeak[b] > 1.0f) analyzerPeak[b] = 1.0f;
    }
}

// ─────────────────────────────────────
// API dla main.cpp
// ─────────────────────────────────────

void eq_set_all_gains(const float in[RUNTIME_EQ_BANDS]) {
    if (!in) return;

    for (int i = 0; i < RUNTIME_EQ_BANDS; ++i) {
        float g = in[i];
        if (g >  18.0f) g =  18.0f;
        if (g < -18.0f) g = -18.0f;
        eqBandGains[i] = g;
    }

    if (eq_inited) {
        for (int i = 0; i < EQ_BANDS; ++i) {
            last_gains[i] = (i < RUNTIME_EQ_BANDS) ? eqBandGains[i] : 0.0f;
        }
        recompute_coeffs(last_gains);
    }
}

void eq_get_all_gains(float out[RUNTIME_EQ_BANDS]) {
    if (!out) return;
    for (int i = 0; i < RUNTIME_EQ_BANDS; ++i) {
        out[i] = eqBandGains[i];
    }
}

// Analizator – zwraca 16 wartości 0..1
void eq_get_analyzer_levels(float out[RUNTIME_EQ_BANDS]) {
    if (!out) return;

#if ENABLE_RUNTIME_ANALYZER
    // przelicz FFT z aktualnego bufora, jeśli analizator jest włączony
    if (eqAnalyzerEnabled) {
        analyzer_compute_from_buffer();
    }

    // zwróć wygładzone wartości 0..1
    for (int i = 0; i < RUNTIME_EQ_BANDS; ++i) {
        float v = analyzerSmooth[i];
        if (v < 0.0f) v = 0.0f;
        if (v > 1.0f) v = 1.0f;
        out[i] = v;
    }
#else
    // analizator wyłączony kompilacyjnie – zawsze 0
    for (int i = 0; i < RUNTIME_EQ_BANDS; ++i) {
        out[i] = 0.0f;
    }
#endif
}

// NOWA FUNKCJA: Zwraca poziomy peak hold
void eq_get_analyzer_peaks(float out[RUNTIME_EQ_BANDS]) {
    if (!out) return;

#if ENABLE_RUNTIME_ANALYZER
    if (eqAnalyzerEnabled) {
        for (int i = 0; i < RUNTIME_EQ_BANDS; ++i) {
            float v = analyzerPeak[i];
            if (v < 0.0f) v = 0.0f;
            if (v > 1.0f) v = 1.0f;
            out[i] = v;
        }
    } else {
        for (int i = 0; i < RUNTIME_EQ_BANDS; ++i) {
            out[i] = 0.0f;
        }
    }
#else
    for (int i = 0; i < RUNTIME_EQ_BANDS; ++i) {
        out[i] = 0.0f;
    }
#endif
}

// Debug: Odczytaj stan bufora analizatora
void eq_get_analyzer_debug(int* samplesFilledOut, int* writeIndexOut) {
    if (samplesFilledOut) *samplesFilledOut = fftSamplesFilled;
    if (writeIndexOut) *writeIndexOut = fftWriteIndex;
}

// Debug: Odczytaj surowe wartości z bufora FFT (dla diagnostyki)
void eq_get_fft_buffer_sample(int index, float* valueOut) {
    if (valueOut && index >= 0 && index < FFT_SIZE) {
        *valueOut = fftBuffer[index];
    }
}

// Nowe funkcje API
void eq_get_analyzer_bands(EQBandInfo out[RUNTIME_EQ_BANDS]) {
    if (!out) return;

    float levels[RUNTIME_EQ_BANDS];
    eq_get_analyzer_levels(levels);

    for (int i = 0; i < RUNTIME_EQ_BANDS; ++i) {
        out[i].band = i;
        out[i].frequency = EQ_BAND_FREQUENCIES[i];
        out[i].level = levels[i];
    }
}

void eq_get_analyzer_stats(float* levelsSumOut, float* maxLevelOut) {
    float levels[RUNTIME_EQ_BANDS];
    eq_get_analyzer_levels(levels);

    float sum = 0.0f;
    float maxLevel = 0.0f;
    for (int i = 0; i < RUNTIME_EQ_BANDS; ++i) {
        sum += levels[i];
        if (levels[i] > maxLevel) maxLevel = levels[i];
    }

    if (levelsSumOut) *levelsSumOut = sum;
    if (maxLevelOut) *maxLevelOut = maxLevel;
}

void eq_set_analyzer_sensitivity(float sensitivity) {
    if (sensitivity < 0.1f) sensitivity = 0.1f;
    if (sensitivity > 2.0f) sensitivity = 2.0f;
    analyzerCurrentSensitivity = sensitivity;
    eqAnalyzerSensitivity = sensitivity;
}

float eq_get_analyzer_sensitivity() {
    return analyzerCurrentSensitivity;
}

void eq_set_analyzer_pre_gain(bool preGain) {
    eqAnalyzerPreGain = preGain;
}

bool eq_get_analyzer_pre_gain() {
    return eqAnalyzerPreGain;
}

void eq_set_analyzer_normalize(bool normalize) {
    eqAnalyzerNormalize = normalize;
}

bool eq_get_analyzer_normalize() {
    return eqAnalyzerNormalize;
}

void eq_set_analyzer_agc(bool enableAgc) {
    eqAnalyzerAGC = enableAgc;
    if (!enableAgc) {
        analyzerAGCGain = 1.0f;
    }
}

bool eq_get_analyzer_agc() {
    return eqAnalyzerAGC;
}

bool eq_is_audio_playing() {
    return (fftSamplesFilled > 0 && analyzerPeakLevel > 0.001f);
}

float eq_get_analyzer_clipping() {
    if (analyzerTotalSamples == 0) return 0.0f;
    return (float)analyzerClippingSamples / (float)analyzerTotalSamples;
}

void eq_reset_analyzer_clipping() {
    analyzerClippingSamples = 0;
    analyzerTotalSamples = 0;
}

// ─────────────────────────────────────
// Główny callback DSP dla ESP32-audioI2S
// ─────────────────────────────────────

void audio_process_i2s(int16_t* outBuff, int32_t validSamples, bool *continueI2S) {
#if ENABLE_RUNTIME_EQ
    if (!outBuff || validSamples <= 0) {
        if (continueI2S) *continueI2S = true;
        return;
    }

    // Korektor działa zawsze, jeśli eqEnabled == true
    bool doEQ = eqEnabled;

    // inicjalizacja przy pierwszym wywołaniu
    if (!eq_inited) {
        fs_rate = (float)audio.getSampleRate();
        if (fs_rate <= 0.0f) fs_rate = 48000.0f;

        compute_center_freqs();

        for (int i = 0; i < EQ_BANDS; ++i) {
            last_gains[i] = (i < RUNTIME_EQ_BANDS) ? eqBandGains[i] : 0.0f;
            state[i][0]   = {0,0,0,0};
            state[i][1]   = {0,0,0,0};
        }

        int codec = audio.getCodec();
        last_codec = codec;
        eq_is_flac = (codec == CODEC_FLAC);

        recompute_coeffs(last_gains);

        // analizator
        fftWriteIndex    = 0;
        fftSamplesFilled = 0;
        for (int i = 0; i < RUNTIME_EQ_BANDS; ++i) analyzerSmooth[i] = 0.0f;

        eq_inited = true;
    } else {
        // wykrycie zmiany kodeka (np. inna stacja)
        int codec = audio.getCodec();
        if (codec != last_codec) {
            last_codec = codec;
            eq_is_flac = (codec == CODEC_FLAC);

            // reset stanów filtrów
            for (int i = 0; i < EQ_BANDS; ++i) {
                state[i][0] = {0,0,0,0};
                state[i][1] = {0,0,0,0};
            }
        }
    }

    // aktualne gainy
    float gains[EQ_BANDS];
    for (int i = 0; i < EQ_BANDS; ++i) {
        gains[i] = (i < RUNTIME_EQ_BANDS) ? eqBandGains[i] : 0.0f;
    }

    // czy trzeba przeliczyć współczynniki
    bool need_recompute = false;
    for (int i = 0; i < EQ_BANDS; ++i) {
        if (fabsf(gains[i] - last_gains[i]) > 0.2f) {
            need_recompute = true;
            break;
        }
    }
    if (need_recompute) {
        recompute_coeffs(gains);
        for (int i = 0; i < EQ_BANDS; ++i) {
            last_gains[i] = gains[i];
        }
    }

    int bands_to_process = EQ_BANDS;
    if (!doEQ) bands_to_process = 0;

    const int channels = 2;

    for (int frame = 0; frame < validSamples; ++frame) {
        int idxL = frame * channels;
        int idxR = idxL + 1;

        // Pobierz oryginalne próbki (przed EQ, przed volume)
        float inL = (float)outBuff[idxL] / 32768.0f;
        float inR = (float)outBuff[idxR] / 32768.0f;

        float yL = inL;
        float yR = inR;

        if (doEQ) {
            // Korekcja – filtrujemy L i R przez biquady
            for (int b = 0; b < EQ_BANDS; ++b) {
                // bypass?
                if (b0_c[b] == 1.0f && b1_c[b] == 0.0f && b2_c[b] == 0.0f &&
                    a1_c[b] == 0.0f && a2_c[b] == 0.0f) {
                    continue;
                }
                // L
                {
                    float outv = b0_c[b] * yL
                               + b1_c[b] * state[b][0].x1
                               + b2_c[b] * state[b][0].x2
                               - a1_c[b] * state[b][0].y1
                               - a2_c[b] * state[b][0].y2;
                    state[b][0].x2 = state[b][0].x1;
                    state[b][0].x1 = yL;
                    state[b][0].y2 = state[b][0].y1;
                    state[b][0].y1 = outv;
                    yL = outv;
                }
                // R
                {
                    float outv = b0_c[b] * yR
                               + b1_c[b] * state[b][1].x1
                               + b2_c[b] * state[b][1].x2
                               - a1_c[b] * state[b][1].y1
                               - a2_c[b] * state[b][1].y2;
                    state[b][1].x2 = state[b][1].x1;
                    state[b][1].x1 = yR;
                    state[b][1].y2 = state[b][1].y1;
                    state[b][1].y1 = outv;
                    yR = outv;
                }
            }
        }

        // Clipping
        if (yL >  1.0f) yL =  1.0f;
        if (yL < -1.0f) yL = -1.0f;
        if (yR >  1.0f) yR =  1.0f;
        if (yR < -1.0f) yR = -1.0f;

        // --- SYGNAŁ VU-METER --- (to jest poziom wyświetlany na wskaźniku VU)
        // float vuLevelL = yL;
        // float vuLevelR = yR;

        // --- Sygnał do analizatora: po EQ, PRZED volume ---
#if ENABLE_RUNTIME_ANALYZER
        if (eqAnalyzerEnabled) {
            float analyzerSignal = sqrtf(0.5f * (yL*yL + yR*yR));
            analyzerTotalSamples++;
            if (analyzerSignal >= 0.99f) {
                analyzerClippingSamples++;
            }
            fftBuffer[fftWriteIndex] = analyzerSignal;
            fftWriteIndex++;
            if (fftWriteIndex >= FFT_SIZE) fftWriteIndex = 0;
            if (fftSamplesFilled < FFT_SIZE) fftSamplesFilled++;
        }
#endif

        // --- TYLKO TUTAJ STOSUJEMY VOLUME ---
// volumeValue w main.cpp jest w krokach 0..maxVolume (np. 21 lub 42)
// skalujemy do 0..1 używając aktualnego maxVolume, żeby nie było ciszej niż w oryginale
float volNorm = 1.0f;
if (maxVolume > 0)
{
    volNorm = (float)volumeValue / (float)maxVolume;
}
if (volNorm < 0.0f) volNorm = 0.0f;
if (volNorm > 1.0f) volNorm = 1.0f;

yL *= volNorm;
yR *= volNorm;
outBuff[idxL] = (int16_t)lroundf(yL * 32768.0f);
        outBuff[idxR] = (int16_t)lroundf(yR * 32768.0f);
    }

    if (continueI2S) *continueI2S = true;
#else
    if (continueI2S) *continueI2S = true;
#endif
}
