#include "SDPlayerManager.h"
#include "SDPlayerWebUI.h"
#include "SDPlayerAdvanced.h"
#include "SDPlayerOLED.h"
#include "Audio.h"
#include <U8g2lib.h>
#include <SD.h>

// Definicja globalnej zmiennej trybu
SDPlayerMode currentSDPlayerMode = SD_MODE_WEBUI;  // Domyślnie WebUI

// Constructor
SDPlayerManager::SDPlayerManager()
    : _webui(nullptr)
    , _advanced(nullptr)
    , _oled(nullptr)
    , _audio(nullptr)
    , _display(nullptr)
    , _currentMode(SD_MODE_WEBUI)
    , _active(false)
{
}

SDPlayerManager::~SDPlayerManager() {
    deactivate();
}

// ==================== INICJALIZACJA ====================

void SDPlayerManager::begin(Audio* audio, U8G2* display) {
    _audio = audio;
    _display = display;
    
    Serial.println("SDPlayerManager: Initialized");
}

void SDPlayerManager::setWebUI(SDPlayerWebUI* webui) {
    _webui = webui;
}

void SDPlayerManager::setAdvanced(SDPlayerAdvanced* advanced) {
    _advanced = advanced;
    
    // KRYTYCZNE: Przekaż display i audio jeśli już są zainicjowane
    if (_display && _audio && _advanced) {
        _advanced->begin(_audio, _display);
        Serial.println("[SDPlayerManager] Display and Audio passed to Advanced");
    }
}

void SDPlayerManager::setOLED(SDPlayerOLED* oled) {
    _oled = oled;
    
    // Przekaż OLED do obu trybów
    if (_webui) _webui->setOLED(oled);
    if (_advanced) _advanced->setOLED(oled);
    
    // SYNCHRONIZACJA: Ustaw callbacki dla dwukierunkowej synchronizacji
    if (_webui && _oled) {
        // WebUI -> OLED: gdy WebUI zmienia indeks, zaktualizuj OLED
        _webui->setIndexChangeCallback([oled](int newIndex) {
            if (oled->isActive() && newIndex >= 0) {
                oled->setSelectedIndex(newIndex);
                Serial.printf("[SDPlayerManager] WebUI->OLED sync: index %d\\n", newIndex);
            }
        });
        
        // OLED -> WebUI: gdy OLED zmienia indeks (pilot/enkoder), zaktualizuj WebUI
        _oled->setIndexChangeCallback([this](int newIndex) {
            if (_webui && newIndex >= 0) {
                _webui->notifyIndexChange(newIndex);
                Serial.printf("[SDPlayerManager] OLED->WebUI sync: index %d\\n", newIndex);
            }
        });
        
        Serial.println("[SDPlayerManager] Bi-directional synchronization callbacks set up");
    }
}

// ==================== ZARZĄDZANIE TRYBEM ====================

void SDPlayerManager::setMode(SDPlayerMode mode) {
    if (_currentMode == mode) return;
    
    Serial.print("SDPlayerManager: Switching mode from ");
    Serial.print(getSDPlayerModeName(_currentMode));
    Serial.print(" to ");
    Serial.println(getSDPlayerModeName(mode));
    
    // Zatrzymaj aktywny tryb
    if (_active) {
        if (_currentMode == SD_MODE_WEBUI && _webui) {
            _webui->stop();
        } else if (_currentMode == SD_MODE_ADVANCED && _advanced) {
            _advanced->deactivate();
        }
    }
    
    // Przełącz tryb
    _currentMode = mode;
    currentSDPlayerMode = mode;  // Aktualizuj globalną zmienną
    
    // Aktywuj nowy tryb jeśli manager był aktywny
    if (_active) {
        if (_currentMode == SD_MODE_WEBUI) {
            switchToWebUI();
        } else if (_currentMode == SD_MODE_ADVANCED) {
            switchToAdvanced();
        }
    }
    
    // Zapisz wybór
    saveSettings();
}

void SDPlayerManager::toggleMode() {
    if (_currentMode == SD_MODE_WEBUI) {
        setMode(SD_MODE_ADVANCED);
    } else {
        setMode(SD_MODE_WEBUI);
    }
}

void SDPlayerManager::switchToWebUI() {
    if (!_webui) {
        Serial.println("SDPlayerManager: WebUI not initialized!");
        return;
    }
    
    Serial.println("SDPlayerManager: Activated WebUI mode");
    // WebUI nie wymaga jawnej aktywacji, działa przez web interface
}

void SDPlayerManager::switchToAdvanced() {
    if (!_advanced) {
        Serial.println("SDPlayerManager: Advanced not initialized!");
        return;
    }
    
    _advanced->activate();
    Serial.println("SDPlayerManager: Activated Advanced mode");
}

// ==================== AKTYWACJA/DEAKTYWACJA ====================

void SDPlayerManager::activate() {
    _active = true;
    
    if (_currentMode == SD_MODE_WEBUI) {
        switchToWebUI();
    } else if (_currentMode == SD_MODE_ADVANCED) {
        switchToAdvanced();
    }
    
    Serial.println("SDPlayerManager: Activated");
}

void SDPlayerManager::deactivate() {
    if (_currentMode == SD_MODE_WEBUI && _webui) {
        _webui->stop();
    } else if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->deactivate();
    }
    
    _active = false;
    Serial.println("SDPlayerManager: Deactivated");
}

// ==================== MAIN LOOP ====================

void SDPlayerManager::loop() {
    static unsigned long lastDebugTime = 0;
    static int callCount = 0;
    callCount++;
    
    if (millis() - lastDebugTime > 2000) {
        Serial.printf("[SDPlayerManager] loop() called %d times - active:%d mode:%d oled:%p advanced:%p\n", 
                      callCount, _active, _currentMode, _oled, _advanced);
        callCount = 0;
        lastDebugTime = millis();
    }
    
    if (!_active) {
        Serial.println("[SDPlayerManager] loop() BLOCKED - not active");
        return;
    }
    
    // Wywołaj loop aktywnego trybu
    if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        Serial.println("[SDPlayerManager] Calling _advanced->loop()");
        _advanced->loop();
    } else if (_currentMode == SD_MODE_WEBUI && _oled) {
        Serial.println("[SDPlayerManager] Calling _oled->loop()");
        _oled->loop();
    } else {
        Serial.printf("[SDPlayerManager] NO LOOP CALLED - mode:%d oled:%p advanced:%p\n", 
                      _currentMode, _oled, _advanced);
    }
}

// ==================== KONTROLA ODTWARZANIA ====================

void SDPlayerManager::play(int index) {
    if (_currentMode == SD_MODE_WEBUI && _webui) {
        _webui->playIndex(index);
    } else if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->play(index);
    }
}

void SDPlayerManager::playFile(const String& path) {
    if (_currentMode == SD_MODE_WEBUI && _webui) {
        _webui->playFile(path);
    } else if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->playFile(path);
    }
}

void SDPlayerManager::pause() {
    if (_currentMode == SD_MODE_WEBUI && _webui) {
        _webui->pause();
    } else if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->pause();
    }
}

void SDPlayerManager::resume() {
    if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->resume();
    } else if (_currentMode == SD_MODE_WEBUI && _webui) {
        _webui->pause();  // WebUI toggle pause/resume
    }
}

void SDPlayerManager::stop() {
    if (_currentMode == SD_MODE_WEBUI && _webui) {
        _webui->stop();
    } else if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->stop();
    }
}

void SDPlayerManager::next() {
    if (_currentMode == SD_MODE_WEBUI && _webui) {
        _webui->next();
    } else if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->next();
    }
}

void SDPlayerManager::previous() {
    if (_currentMode == SD_MODE_WEBUI && _webui) {
        _webui->prev();
    } else if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->previous();
    }
}

void SDPlayerManager::togglePause() {
    if (_currentMode == SD_MODE_WEBUI && _webui) {
        _webui->pause();
    } else if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->togglePause();
    }
}

// ==================== STATUS ====================

bool SDPlayerManager::isPlaying() {
    if (_currentMode == SD_MODE_WEBUI && _webui) {
        return _webui->isPlaying();
    } else if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        return _advanced->isPlaying();
    }
    return false;
}

bool SDPlayerManager::isPaused() {
    if (_currentMode == SD_MODE_WEBUI && _webui) {
        return _webui->isPaused();
    } else if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        return _advanced->isPaused();
    }
    return false;
}

String SDPlayerManager::getCurrentFile() {
    if (_currentMode == SD_MODE_WEBUI && _webui) {
        return _webui->getCurrentFile();
    } else if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        return _advanced->getTrackInfo().filename;
    }
    return "None";
}

int SDPlayerManager::getSelectedIndex() {
    if (_currentMode == SD_MODE_WEBUI && _webui) {
        return _webui->getSelectedIndex();
    } else if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        return _advanced->getSelectedIndex();
    }
    return -1;
}

// ==================== CALLBACKS OD AUDIO ====================

void SDPlayerManager::onAudioInfo(const char* info) {
    // Przekaż do aktywnego trybu
    if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->onAudioInfo(info);
    }
    // WebUI nie obsługuje info callbacks
}

void SDPlayerManager::onAudioID3(const char* info) {
    if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->onAudioID3(info);
    }
}

void SDPlayerManager::onAudioBitrate(const char* info) {
    if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->onAudioBitrate(info);
    }
}

void SDPlayerManager::onAudioEOF(const char* info) {
    // Przekaż tylko do Advanced mode (WebUI nie obsługuje tego eventu)
    if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->onAudioEOF(info);
    }
}

// ==================== OBSŁUGA ENKODERA ====================

void SDPlayerManager::onEncoderButton() {
    if (!_active) return;
    
    // Enkoder jest obsługiwany przez stary system SDPlayerOLED
    if (_oled) {
        _oled->onEncoderButton();
    }
}

void SDPlayerManager::onEncoderButtonHold(unsigned long duration) {
    if (!_active) return;
    
    if (_oled) {
        _oled->onEncoderButtonHold(duration);
    }
}

void SDPlayerManager::onEncoderLeft() {
    if (!_active) return;
    
    if (_oled) {
        _oled->onEncoderLeft();
    }
}

void SDPlayerManager::onEncoderRight() {
    if (!_active) return;
    
    if (_oled) {
        _oled->onEncoderRight();
    }
}

// ==================== OBSŁUGA PILOTA ====================

void SDPlayerManager::onRemoteOK() {
    if (!_active) return;
    
    if (_oled) {
        _oled->onRemoteOK();
    }
}

void SDPlayerManager::onRemoteStop() {
    if (!_active) return;
    
    if (_oled) {
        _oled->onRemoteStop();
    }
}

void SDPlayerManager::onRemoteVolUp() {
    if (!_active) return;
    
    if (_oled) {
        _oled->onRemoteVolUp();
    }
}

void SDPlayerManager::onRemoteVolDown() {
    if (!_active) return;
    
    if (_oled) {
        _oled->onRemoteVolDown();
    }
}

// ==================== ZAPISYWANIE/WCZYTYWANIE USTAWIEŃ ====================

void SDPlayerManager::saveSettings() {
    File file = SD.open("/sdplayer_mode.cfg", FILE_WRITE);
    if (file) {
        file.println((int)_currentMode);
        file.close();
        
        Serial.print("SDPlayerManager: Saved mode: ");
        Serial.println(getSDPlayerModeName(_currentMode));
    }
    
    // Zapisz ustawienia aktywnego trybu
    if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->saveSettings();
    }
}

void SDPlayerManager::loadSettings() {
    if (!SD.exists("/sdplayer_mode.cfg")) {
        Serial.println("SDPlayerManager: No mode config found, using default (WebUI)");
        return;
    }
    
    File file = SD.open("/sdplayer_mode.cfg");
    if (file) {
        int mode = file.parseInt();
        _currentMode = (SDPlayerMode)mode;
        currentSDPlayerMode = _currentMode;
        file.close();
        
        Serial.print("SDPlayerManager: Loaded mode: ");
        Serial.println(getSDPlayerModeName(_currentMode));
    }
    
    // Wczytaj ustawienia wybranego trybu
    if (_currentMode == SD_MODE_ADVANCED && _advanced) {
        _advanced->loadSettings();
    }
}
