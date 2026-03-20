// ============================================================================
// audio_hooks.cpp – silny (strong) symbol audio_process_i2s dla analizatora
// ============================================================================
// WAŻNE: Ten plik NIE może zawierać #include "Audio.h" (nawet pośrednio).
// Powód: Audio.h zawiera `extern __attribute__((weak)) void audio_process_i2s(...)`,
// co powoduje że GCC propaguje atrybut weak na KAŻDĄ definicję w tej samej
// jednostce kompilacji → symbol staje się W (weak) zamiast T (strong), a linker
// wybiera pusty stub z Audio.cpp.o zamiast naszej wersji.
// Rozwiązanie: definiujemy funkcję w osobnym pliku bez Audio.h → symbol jest T.
// ============================================================================

#include <Arduino.h>           // millis(), Serial – NIE zawiera Audio.h
#include "EQ_FFTAnalyzer.h"    // eq_analyzer_push_samples_i16, eq_analyzer_set_runtime_active

// Bufor konwersji int32→int16 (static = BSS, 32 KB)
static int16_t s_audio_i16buf[16384];  // maks 8192 ramek stereo × 2 = 16384 int16

// ============================================================================
// audio_process_i2s – wywoływana z Audio.cpp::playChunk() po dekodowaniu
// outBuff  – stereo int32_t [L32, R32, ...] (dane left-justified: val<<16)
// validSamples – liczba ramek stereo
// ============================================================================
void audio_process_i2s(int32_t* outBuff, int16_t validSamples, bool* continueI2S)
{
    if (validSamples > 0) {
        eq_analyzer_set_runtime_active(true);
    }

    const int numSamples = (int)validSamples * 2;  // L+R

    if (numSamples > 0 && numSamples <= 16384) {
        // Konwersja int32 → int16 (górne 16 bitów 32-bit PCM left-justified)
        for (int i = 0; i < numSamples; i++) {
            s_audio_i16buf[i] = (int16_t)(outBuff[i] >> 16);
        }

        // EQ/FFT analizator spektrum
        eq_analyzer_push_samples_i16(s_audio_i16buf, (uint32_t)validSamples);
    }

    *continueI2S = true;
}
