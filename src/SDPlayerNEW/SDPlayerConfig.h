#pragma once

/**
 * SDPlayerConfig.h - Konfiguracja SD Player
 * 
 * Zawiera wspólne definicje i wybór trybu obsługi SD Playera
 */

// Tryb obsługi SD Player
enum SDPlayerMode {
    SD_MODE_WEBUI = 0,      // Tryb WebUI - podstawowy, z interfejsem webowym
    SD_MODE_ADVANCED = 1    // Tryb Advanced - zaawansowany z rozszerzonymi funkcjami:
                            //   - Auto-play (automatyczne przejście do kolejnego utworu)
                            //   - Inteligentne sortowanie numerów
                            //   - Tagi ID3 (Artist, Title, Album)
                            //   - Wyświetlanie parametrów audio (Bitrate, SampleRate)
                            //   - Timer odtwarzania (mm:ss)
                            //   - Polskie znaki diakrytyczne
                            //   - Zapisywanie/wczytywanie pozycji z SD
                            //   - Rozszerzone formaty audio
};

// Globalna zmienna wyboru trybu (domyślnie WebUI)
// Zmień na SD_MODE_ADVANCED w settings lub przez menu
extern SDPlayerMode currentSDPlayerMode;

// Nazwy trybów do wyświetlenia
inline const char* getSDPlayerModeName(SDPlayerMode mode) {
    switch (mode) {
        case SD_MODE_WEBUI:    return "WebUI (Basic)";
        case SD_MODE_ADVANCED: return "Advanced (Full)";
        default:               return "Unknown";
    }
}

// Opisy trybów
inline const char* getSDPlayerModeDescription(SDPlayerMode mode) {
    switch (mode) {
        case SD_MODE_WEBUI:
            return "Podstawowy tryb z webowym interfejsem";
            
        case SD_MODE_ADVANCED:
            return "Zaawansowany: auto-play, ID3, smart sort";
            
        default:
            return "";
    }
}
