#pragma once
#include <Arduino.h>
#include <U8g2lib.h>

class Audio;

// ========================================================================
// 16-BAND GRAPHIC EQUALIZER - ULEPSZONA KONWERSJA DO 3-PUNKTÓW
// ========================================================================
// UWAGA: Nowa biblioteka ESP32-audioI2S (GitHub) nie ma wbudowanego
// 16-pasmowego equalizera. Ta implementacja używa WAŻONEJ KONWERSJI 
// na 3-punktowy equalizer (Low/Mid/High).
//
// MAPOWANIE PASM (z wagami psychoakustycznymi):
//   - Low  (pasma 0-4):   32-500 Hz   (waga: 1.2, 1.5, 1.3, 1.0, 0.8)
//   - Mid  (pasma 5-10):  1k-6k Hz    (waga: 1.0, 1.3, 1.5, 1.3, 1.0, 0.8)
//   - High (pasma 11-15): 8k-16k Hz   (waga: 1.0, 1.2, 1.1, 0.9, 0.8)
//
// ZAKRES WZMOCNIEŃ:
//   - UI (ekran): -16 do +16 dB (16 pasm, wyświetlanie)
//   - Presety: -12 do +12 dB (bezpieczny margines)
//   - Audio (setTone): -40 do +6 dB (hardware limit)
//   - Konwersja: ważona średnia -> ograniczenie do [-12..+6]
//
// PRESETY: 7 profesjonalnych ustawień (Flat, Bass, Vocal, Radio, 
//          V-Shape, Rock, Jazz)
//
// ALGORYTM: Ważona konwersja uwzględnia krzywą psychoakustyczną -
//           pasma 2-3kHz (wokale) mają większy wpływ niż ekstremum
// ========================================================================

namespace APMS_EQ16 {

static const uint8_t BANDS = 16;

void init(Audio* audio);
void setFeatureEnabled(bool enabled);     // gate for menu entry
bool isFeatureEnabled();

void setEnabled(bool enabled);            // audio processing on/off
bool isEnabled();

void setBand(uint8_t band, int8_t gainDb);
int8_t getBand(uint8_t band);
void getAll(int8_t* out16);
void setAll(const int8_t* in16);

void applyToAudio();                      // pushes gains + enable flag to Audio

// UI helpers (drawing only, keys handled in main)
void drawModeSelect(U8G2& u8g2, uint8_t selectedMode); // 0=3-band, 1=16-band
void drawEditor(U8G2& u8g2, const int8_t* gains16, uint8_t selectedBand, bool showHelp);

} // namespace

// ======================= C-STYLE WRAPPER FUNCTIONS =======================
// Dla kompatybilności z main.cpp który używa funkcji EQ16_*

#ifdef __cplusplus
extern "C" {
#endif

// Core functions
void EQ16_init(void);
void EQ16_enable(bool enabled);
bool EQ16_isEnabled(void);

// Band management  
void EQ16_setBand(uint8_t band, int8_t gainDb);
int8_t EQ16_getBand(uint8_t band);
void EQ16_resetAllBands(void);

// Menu management
bool EQ16_isMenuActive(void);
void EQ16_setMenuActive(bool active);
void EQ16_displayMenu(void);

// Band selection and adjustment
void EQ16_selectPrevBand(void);
void EQ16_selectNextBand(void);
void EQ16_increaseBandGain(void);
void EQ16_decreaseBandGain(void);

// Audio processing
float EQ16_processSample(float sample, float unused, bool isLeft);

// Storage management
void EQ16_saveToSD(void);
void EQ16_loadFromSD(void);
void EQ16_autoSave(void);

// Presets
void EQ16_loadPreset(uint8_t presetId);

#ifdef __cplusplus
}
#endif
