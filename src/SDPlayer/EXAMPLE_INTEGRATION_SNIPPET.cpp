/* 
 * =================================================================
 * PRZYKŁAD INTEGRACJI SD PLAYER - FRAGMENT DO DODANIA W MAIN.CPP
 * =================================================================
 * 
 * ⚠️ TEN PLIK NIE JEST KOMPILOWANY - TO TYLKO PRZYKŁAD!
 * 
 * Skopiuj odpowiednie sekcje do swojego main.cpp
 * 
 * UWAGA: Poniższy kod zawiera przykładowe zmienne (server, audio, u8g2)
 *        które musisz dopasować do Twojego projektu!
 */

#if 0  // NIE KOMPILUJ - TYLKO PRZYKŁAD

// =====================================================
// 1. INCLUDES (na górze pliku, po istniejących)
// =====================================================
#include "SDPlayer/SDPlayerConfig.h"
#include "SDPlayer/SDPlayerManager.h"
#include "SDPlayer/SDPlayerWebUI.h"
#include "SDPlayer/SDPlayerAdvanced.h"
#include "SDPlayer/SDPlayerOLED.h"


// =====================================================
// 2. DEKLARACJE GLOBALNE (obok innych obiektów)
// =====================================================

// SD Player - oba tryby
SDPlayerWebUI sdPlayerWebUI;
SDPlayerAdvanced sdPlayerAdvanced;
SDPlayerManager sdPlayerManager;  // Menadżer przełączania trybów
// SDPlayerOLED sdPlayerOLED(u8g2); // <-- jeśli już masz, nie duplikuj

// Flagi SD Player
bool sdPlayerPlayingMusic = false;  // Czy SD Player aktualnie odtwarza
bool sdPlayerScanRequested = false; // Żądanie skanowania katalogu


// =====================================================
// 3. W FUNKCJI setup() - po inicjalizacji SD, WiFi, Audio
// =====================================================

void setup() {
    // ... twój istniejący kod inicjalizacji ...
    
    Serial.println("=== Inicjalizacja SD Player ===");
    
    // Inicjalizuj oba tryby
    sdPlayerWebUI.begin(&server, &audio);
    sdPlayerAdvanced.begin(&audio, &u8g2);
    
    // Inicjalizuj menadżer
    sdPlayerManager.begin(&audio, &u8g2);
    sdPlayerManager.setWebUI(&sdPlayerWebUI);
    sdPlayerManager.setAdvanced(&sdPlayerAdvanced);
    sdPlayerManager.setOLED(&sdPlayerOLED);
    
    // Wczytaj ostatnio wybrany tryb z SD
    sdPlayerManager.loadSettings();
    
    // Inicjalizuj OLED dla SD Player
    if (sdPlayerManager.getMode() == SD_MODE_ADVANCED) {
        sdPlayerOLED.begin(&sdPlayerAdvanced);  // Powiąż z Advanced
    } else {
        sdPlayerOLED.begin(&sdPlayerWebUI);     // Powiąż z WebUI
    }
    
    Serial.print("SD Player Mode: ");
    Serial.println(getSDPlayerModeName(sdPlayerManager.getMode()));
    
    // ... reszta setup() ...
}


// =====================================================
// 4. W FUNKCJI loop() - główna pętla
// =====================================================

void loop() {
    // ... twój istniejący kod ...
    
    // === SD Player Manager ===
    // Obsługuje aktywny tryb (WebUI lub Advanced)
    sdPlayerManager.loop();
    
    // === SD Player OLED ===
    // Jeśli aktywny, odświeżaj wyświetlacz
    if (sdPlayerOLED.isActive()) {
        sdPlayerOLED.loop();
    }
    
    // ... reszta loop() ...
}


// =====================================================
// 5. AUDIO CALLBACKS - ZASTĄP ISTNIEJĄCE
// =====================================================

// Te funkcje są wywoływane przez bibliotekę Audio
// Przekieruj je do SD Player Manager

void audio_info(const char *info) {
    Serial.print("audio_info: ");
    Serial.println(info);
    
    // Przekaż do managera (Advanced mode wykorzysta te dane)
    sdPlayerManager.onAudioInfo(info);
}

void audio_id3data(const char *info) {
    Serial.print("audio_id3data: ");
    Serial.println(info);
    
    // Przekaż do managera (Advanced wyodrębni Artist, Title)
    sdPlayerManager.onAudioID3(info);
}

void audio_bitrate(const char *info) {
    Serial.print("audio_bitrate: ");
    Serial.println(info);
    
    sdPlayerManager.onAudioBitrate(info);
}

void audio_eof_mp3(const char *info) {
    Serial.println("audio_eof_mp3: Track ended");
    
    // WAŻNE! To obsługuje auto-play w Advanced mode
    sdPlayerManager.onAudioEOF(info);
    
    // Zresetuj flagę
    sdPlayerPlayingMusic = false;
}


// =====================================================
// 6. OBSŁUGA PILOTA - dodaj w istniejącej funkcji
// =====================================================

void handleRemoteControl() {
    // ... twój istniejący kod obsługi pilota ...
    
    // === Specjalna obsługa dla SD Player ===
    
    // Jeśli jesteśmy w trybie SD Player (currentMode == SD_PLAYER_MODE):
    if (currentMode == SD_PLAYER_MODE) {
        
        // Sterowanie OLED (nawigacja lista)
        if (IRupArrow) {
            IRupArrow = false;
            sdPlayerOLED.onRemoteUp();
        }
        
        if (IRdownArrow) {
            IRdownArrow = false;
            sdPlayerOLED.onRemoteDown();
        }
        
        if (IRokButton) {
            IRokButton = false;
            sdPlayerOLED.onRemoteOK();  // Play/Select
        }
        
        // Kontrola odtwarzania
        if (IRleftArrow) {
            IRleftArrow = false;
            sdPlayerManager.previous();
        }
        
        if (IRrightArrow) {
            IRrightArrow = false;
            sdPlayerManager.next();
        }
        
        if (IRpauseResume) {
            IRpauseResume = false;
            sdPlayerManager.togglePause();
        }
        
        if (IRstop) {
            IRstop = false;
            sdPlayerManager.stop();
        }
        
        // Przełączanie stylu OLED
        if (IRsourceButton) {  // Przykład: przycisk SRC
            IRsourceButton = false;
            sdPlayerOLED.nextStyle();
        }
    }
    
    // === Globalne: przełączanie trybu SD Player ===
    // (działa niezależnie od currentMode, np. długie przytrzymanie)
    
    if (IRlongPressMode) {  // Długie przytrzymanie MODE
        IRlongPressMode = false;
        
        // Przełącz tryb Advanced <-> WebUI
        sdPlayerManager.toggleMode();
        
        // Wyświetl komunikat
        String msg = "SD Mode: ";
        msg += getSDPlayerModeName(sdPlayerManager.getMode());
        showOLEDMessage(msg, 2000);
    }
}


// =====================================================
// 7. MENU USTAWIEŃ - dodaj opcję wyboru trybu
// =====================================================

// Przykład submenu w istniejącym menu Settings:

void showSDPlayerSettingsMenu() {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(10, 10, "SD Player Settings");
    
    // Aktualny tryb
    u8g2.drawStr(10, 25, "Mode:");
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(60, 25, getSDPlayerModeName(sdPlayerManager.getMode()));
    
    // Opis
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(10, 40, getSDPlayerModeDescription(sdPlayerManager.getMode()));
    
    // Instrukcje
    u8g2.drawStr(10, 55, "[OK] Toggle  [BACK] Exit");
    u8g2.sendBuffer();
    
    // Obsługa przycisków
    if (IRokButton) {
        IRokButton = false;
        sdPlayerManager.toggleMode();
        showSDPlayerSettingsMenu();  // Odśwież
    }
}

// Lub dodaj w istniejącym switch/case:
void handleSettingsMenuSelection(int selectedItem) {
    switch (selectedItem) {
        // ... istniejące case'y ...
        
        case MENU_SDPLAYER_MODE:
            sdPlayerManager.toggleMode();
            
            // Wyświetl komunikat
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_ncenB08_tr);
            u8g2.drawStr(10, 20, "SD Player Mode:");
            u8g2.setFont(u8g2_font_ncenB14_tr);
            u8g2.drawStr(10, 40, getSDPlayerModeName(sdPlayerManager.getMode()));
            u8g2.sendBuffer();
            delay(2000);
            break;
            
        // ... reszta case'ów ...
    }
}


// =====================================================
// 8. PRZYKŁAD UŻYCIA - funkcje pomocnicze
// =====================================================

// Sprawdź aktualny tryb:
void checkSDPlayerMode() {
    if (sdPlayerManager.getMode() == SD_MODE_ADVANCED) {
        Serial.println("Advanced mode: Full features enabled");
        
        // Pobierz informacje o utworze (tylko w Advanced)
        const auto& info = sdPlayerAdvanced.getTrackInfo();
        if (info.hasID3) {
            Serial.print("Artist: ");
            Serial.println(info.artist);
            Serial.print("Title: ");
            Serial.println(info.title);
        }
        
        Serial.print("Play time: ");
        Serial.println(sdPlayerAdvanced.getPlayTimeString());
        
    } else {
        Serial.println("WebUI mode: Basic playback");
    }
}

// Uruchom SD Player w konkretnym trybie:
void startSDPlayerInAdvancedMode() {
    // Ustaw tryb Advanced
    sdPlayerManager.setMode(SD_MODE_ADVANCED);
    
    // Włącz auto-play
    sdPlayerAdvanced.setAutoPlay(true);
    
    // Ustaw tryb powtarzania
    sdPlayerAdvanced.setRepeatMode(SDPlayerAdvanced::REPEAT_ALL);
    
    // Aktywuj
    sdPlayerManager.activate();
    
    // Odtwórz pierwszy plik
    sdPlayerManager.play(0);
    
    Serial.println("SD Player started in Advanced mode with auto-play");
}

// Zapisz ustawienia manualnie:
void saveAllSDPlayerSettings() {
    sdPlayerManager.saveSettings();
    Serial.println("SD Player settings saved");
}


// =====================================================
// 9. DEBUGOWANIE - funkcje diagnostyczne
// =====================================================

void printSDPlayerStatus() {
    Serial.println("=== SD Player Status ===");
    Serial.print("Mode: ");
    Serial.println(getSDPlayerModeName(sdPlayerManager.getMode()));
    Serial.print("Active: ");
    Serial.println(sdPlayerManager.isActive() ? "Yes" : "No");
    Serial.print("Playing: ");
    Serial.println(sdPlayerManager.isPlaying() ? "Yes" : "No");
    Serial.print("File: ");
    Serial.println(sdPlayerManager.getCurrentFile());
    
    if (sdPlayerManager.getMode() == SD_MODE_ADVANCED) {
        const auto& info = sdPlayerAdvanced.getTrackInfo();
        Serial.print("Artist: ");
        Serial.println(info.artist.isEmpty() ? "N/A" : info.artist);
        Serial.print("Title: ");
        Serial.println(info.title.isEmpty() ? "N/A" : info.title);
        Serial.print("Bitrate: ");
        Serial.print(info.bitrate);
        Serial.println(" kbps");
        Serial.print("Time: ");
        Serial.println(sdPlayerAdvanced.getPlayTimeString());
        Serial.print("Repeat: ");
        Serial.println(sdPlayerAdvanced.getRepeatMode());
        Serial.print("Auto-play: ");
        Serial.println(sdPlayerAdvanced.getAutoPlay() ? "ON" : "OFF");
    }
    
    Serial.println("========================");
}


// =====================================================
// KOŃCOWE NOTATKI
// =====================================================

/*
 * WAŻNE PUNKTY INTEGRACJI:
 * 
 * 1. Audio callbacks (audio_xxx) MUSZĄ przekierowywać do sdPlayerManager
 * 2. W loop() wywołaj sdPlayerManager.loop()
 * 3. Zapisuj ustawienia przy wyjściu z SD Player (saveSettings)
 * 4. Auto-play działa tylko w Advanced mode
 * 5. ID3 tagi działają tylko w Advanced mode
 * 6. WebUI wymaga AsyncWebServer i WiFi
 * 
 * PROBLEMY?
 * - Sprawdź Serial Monitor - wszystkie operacje są logowane
 * - Użyj printSDPlayerStatus() do debugowania
 * - Upewnij się że karta SD jest dostępna
 * 
 * TESTOWANIE:
 * 1. Włącz tryb Advanced: sdPlayerManager.setMode(SD_MODE_ADVANCED);
 * 2. Odtwórz muzykę: sdPlayerManager.play(0);
 * 3. Sprawdź auto-play - czy po zakończeniu przechodzi do następnego
 * 4. Sprawdź tagi ID3 - printSDPlayerStatus()
 * 5. Przełącz na WebUI i sprawdź interfejs web
 */

#endif  // Koniec bloku przykładowego
