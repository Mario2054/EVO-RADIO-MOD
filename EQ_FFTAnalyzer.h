#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Stała liczba pasm (w projekcie: 16)
static const uint8_t EQ_BANDS = 16;

// Init / runtime
bool  eq_analyzer_init(void);              // start: kolejka + task na Core1
void  eq_analyzer_deinit(void);            // stop + cleanup
void  eq_analyzer_reset(void);             // wyzeruj poziomy/peaki/ref
void  eq_analyzer_set_enabled(bool en);    // global ON/OFF (z WWW / pilota)
bool  eq_analyzer_get_enabled(void);

// Gdy wyświetlany jest styl 5/6, można podbić aktywność; gdy nie – usypiamy
void  eq_analyzer_set_runtime_active(bool active);

// Parametry próbkowania (ustawiane po wykryciu sample rate przez Audio.cpp)
void  eq_analyzer_set_sample_rate(uint32_t sample_rate_hz);

// Hook na próbki audio (wywoływany z Audio.cpp – MUSI być ultralekki)
void  eq_analyzer_push_samples_i16(const int16_t* interleavedLR, uint32_t frames);

// Wyniki (0..1), thread-safe snapshot
void  eq_get_analyzer_levels(float out_levels[EQ_BANDS]);
void  eq_get_analyzer_peaks (float out_peaks [EQ_BANDS]);

// Diagnostyka (opcjonalnie – NIE włączać stale przy streamie FLAC/AAC)
bool  eq_analyzer_is_receiving_samples(void);
void  eq_analyzer_print_diagnostics(void);

// Generator testowy (opcjonalnie)
void  eq_analyzer_enable_test_generator(bool en);

#ifdef __cplusplus
} // extern "C"
#endif
