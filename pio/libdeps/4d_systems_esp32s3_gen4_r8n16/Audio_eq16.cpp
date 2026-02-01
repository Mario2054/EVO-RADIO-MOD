// --- Implementacja metod do obsługi 16-pasmowego equalizera ---
#include <cstdint>
#include "Audio.h"

void Audio::setGraphicEQ16(const int8_t* gains) {
    if (!gains) return;
    for (int i = 0; i < 16; ++i) {
        m_graphicEQ16Gains[i] = gains[i];
    }
    // Tu można dodać logikę DSP lub wywołać setTone dla uproszczonego działania
}

void Audio::enableGraphicEQ16(bool enabled) {
    m_graphicEQ16Enabled = enabled;
    // Tu można dodać logikę aktywacji/dezaktywacji EQ
}

void Audio::getGraphicEQ16(int8_t* out16) const {
    if (!out16) return;
    for (int i = 0; i < 16; ++i) {
        out16[i] = m_graphicEQ16Gains[i];
    }
}
