#pragma once
#include <Arduino.h>
#include "SDPlayerConfig.h"

// Forward declarations
class Audio;
class U8G2;
class SDPlayerOLED;
class SDPlayerWebUI;
class SDPlayerAdvanced;

/**
 * SDPlayerManager - Menadżer trybów SD Player
 * 
 * Zarządza przełączaniem między trybami:
 * - SD_MODE_WEBUI: Podstawowy tryb z interfejsem webowym
 * - SD_MODE_ADVANCED: Zaawansowany tryb z rozszerzonymi funkcjami
 * 
 * Wywoływane funkcje audio_xxx przekierowuje do aktywnego trybu
 */

class SDPlayerManager {
public:
    SDPlayerManager();
    ~SDPlayerManager();
    
    // Inicjalizacja
    void begin(Audio* audio, U8G2* display = nullptr);
    void setWebUI(SDPlayerWebUI* webui);
    void setAdvanced(SDPlayerAdvanced* advanced);
    void setOLED(SDPlayerOLED* oled);
    
    // Zarządzanie trybem
    void setMode(SDPlayerMode mode);
    SDPlayerMode getMode() { return _currentMode; }
    void toggleMode();  // Przełącz między trybami
    
    // Aktywacja/deaktywacja
    void activate();
    void deactivate();
    bool isActive() { return _active; }
    
    // Główna pętla
    void loop();
    
    // Kontrola odtwarzania (wspólna dla obu trybów)
    void play(int index);
    void playFile(const String& path);
    void pause();
    void resume();
    void stop();
    void next();
    void previous();
    void togglePause();
    
    // Status
    bool isPlaying();
    bool isPaused();
    String getCurrentFile();
    int getSelectedIndex();
    
    // Callbacks od Audio (przekazywane do aktywnego trybu)
    void onAudioInfo(const char* info);
    void onAudioID3(const char* info);
    void onAudioBitrate(const char* info);
    void onAudioEOF(const char* info);
    
    // Obsługa enkodera (przekazywane do aktywnego trybu lub OLED)
    void onEncoderButton();
    void onEncoderButtonHold(unsigned long duration);
    void onEncoderLeft();
    void onEncoderRight();
    
    // Obsługa pilota (przekazywane do aktywnego trybu lub OLED)
    void onRemoteOK();
    void onRemoteStop();
    void onRemoteVolUp();
    void onRemoteVolDown();
    
    // Zapisywanie/wczytywanie ustawień
    void saveSettings();
    void loadSettings();
    
    // Gettery
    SDPlayerWebUI* getWebUI() { return _webui; }
    SDPlayerAdvanced* getAdvanced() { return _advanced; }
    
private:
    // Wskaźniki do obu trybów
    SDPlayerWebUI* _webui;
    SDPlayerAdvanced* _advanced;
    SDPlayerOLED* _oled;
    Audio* _audio;
    U8G2* _display;
    
    // Status
    SDPlayerMode _currentMode;
    bool _active;
    
    // Pomocnicze
    void switchToWebUI();
    void switchToAdvanced();
};
