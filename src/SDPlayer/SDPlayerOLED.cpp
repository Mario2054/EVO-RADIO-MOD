#include "SDPlayerOLED.h"
#include "SDPlayerWebUI.h"
#include <SD.h>

// Extern zmienne z main.cpp do zarządzania trybem odtwarzania
extern bool sdPlayerPlayingMusic;
extern bool sdPlayerOLEDActive;

// Forward declaration funkcji z main.cpp
extern void displayRadio();
extern U8G2 u8g2;

SDPlayerOLED::SDPlayerOLED(U8G2& display) 
    : _display(display),
      _player(nullptr),
      _active(false),
      _style(STYLE_1),
      _infoStyle(INFO_CLOCK_DATE),
      _mode(MODE_NORMAL),
      _selectedIndex(0),
      _scrollOffset(0),
      _splashStartTime(0),
      _volumeShowTime(0),
      _lastUpdate(0),
      _scrollPosition(0),
      _animFrame(0) {
}

void SDPlayerOLED::begin(SDPlayerWebUI* player) {
    _player = player;
    _display.begin();
    _display.setFont(u8g2_font_6x10_tr);
}

void SDPlayerOLED::activate() {
    _active = true;
    showSplash();
}

void SDPlayerOLED::deactivate() {
    _active = false;
    _display.clearBuffer();
    _display.sendBuffer();
}

void SDPlayerOLED::showSplash() {
    _mode = MODE_SPLASH;
    _splashStartTime = millis();
}

void SDPlayerOLED::loop() {
    if (!_active) return;
    
    unsigned long now = millis();
    
    // Splash screen przez 1.5s
    if (_mode == MODE_SPLASH) {
        if (now - _splashStartTime > 1500) {
            _mode = MODE_NORMAL;
            refreshFileList();
        }
    }
    
    // Volume pokazuje się przez 2s
    if (_mode == MODE_VOLUME) {
        if (now - _volumeShowTime > 2000) {
            _mode = MODE_NORMAL;
        }
    }
    
    // Odświeżanie ekranu co 50ms
    if (now - _lastUpdate > 50) {
        _lastUpdate = now;
        _animFrame++;
        render();
    }
}

void SDPlayerOLED::refreshFileList() {
    if (!_player) return;
    
    _fileList.clear();
    String currentDir = _player->getCurrentDirectory();
    
    File dir = SD.open(currentDir);
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        return;
    }
    
    File entry = dir.openNextFile();
    while (entry) {
        FileEntry fe;
        fe.name = String(entry.name());
        
        // Usuń ścieżkę - zostaw tylko nazwę
        int lastSlash = fe.name.lastIndexOf('/');
        if (lastSlash >= 0) {
            fe.name = fe.name.substring(lastSlash + 1);
        }
        
        fe.isDir = entry.isDirectory();
        
        // Dodaj tylko katalogi i pliki audio
        if (fe.isDir || 
            fe.name.endsWith(".mp3") || fe.name.endsWith(".MP3") ||
            fe.name.endsWith(".wav") || fe.name.endsWith(".WAV") ||
            fe.name.endsWith(".flac") || fe.name.endsWith(".FLAC")) {
            _fileList.push_back(fe);
        }
        
        entry.close();
        entry = dir.openNextFile();
    }
    dir.close();
    
    // Sortuj: foldery, potem pliki
    std::sort(_fileList.begin(), _fileList.end(), [](const FileEntry& a, const FileEntry& b) {
        if (a.isDir != b.isDir) return a.isDir;
        return a.name.compareTo(b.name) < 0;
    });
    
    _selectedIndex = 0;
    _scrollOffset = 0;
}

void SDPlayerOLED::render() {
    _display.clearBuffer();
    
    switch (_mode) {
        case MODE_SPLASH:
            renderSplash();
            break;
        case MODE_VOLUME:
            renderVolume();
            break;
        case MODE_NORMAL:
            switch (_style) {
                case STYLE_1: renderStyle1(); break;
                case STYLE_2: renderStyle2(); break;
                case STYLE_3: renderStyle3(); break;
                case STYLE_4: renderStyle4(); break;
                case STYLE_5: renderStyle5(); break;
                case STYLE_6: renderStyle6(); break;
                case STYLE_10: renderStyle10(); break;
            }
            break;
    }
    
    _display.sendBuffer();
}

void SDPlayerOLED::renderSplash() {
    // "SD PLAYER" wyśrodkowany na ekranie
    _display.setFont(u8g2_font_fub14_tr);  // Duża czcionka
    const char* text = "SD PLAYER";
    int w = _display.getStrWidth(text);
    int x = (256 - w) / 2;  // Wyśrodkowanie na 256px szerokości
    int y = 32;             // Środek wysokości (64px / 2)
    
    _display.drawStr(x, y, text);
}

void SDPlayerOLED::renderVolume() {
    if (!_player) return;
    
    _display.setFont(u8g2_font_fub14_tr);
    
    // "Volume"
    const char* text = "Volume";
    int w = _display.getStrWidth(text);
    _display.drawStr((256 - w) / 2, 25, text);
    
    // Wartość
    int vol = _player->getVolume();
    String volStr = String(vol);
    _display.setFont(u8g2_font_freedoomr25_tn);
    w = _display.getStrWidth(volStr.c_str());
    _display.drawStr((256 - w) / 2, 55, volStr.c_str());
    
    // Pasek
    int barWidth = (vol * 220) / 21;  // 220px szerokości paska dla 256px ekranu
    _display.drawFrame(18, 58, 220, 4);
    _display.drawBox(18, 58, barWidth, 4);
}

void SDPlayerOLED::renderStyle1() {
    drawTopBar();
    drawFileList();
    drawScrollBar(_fileList.size(), 4);
}

void SDPlayerOLED::drawTopBar() {
    if (!_player) return;
    
    _display.setFont(u8g2_font_6x10_tr);
    
    if (_infoStyle == INFO_CLOCK_DATE) {
        // **ZEGAR PO LEWEJ** + **DATA W ŚRODKU** + **FORMAT AUDIO** + **GŁOŚNIK + VOLUME PO PRAWEJ**
        struct tm timeinfo;
        if (getLocalTime(&timeinfo, 100)) {
            // Zegar po lewej stronie (HH:MM)
            char timeStr[6];
            snprintf(timeStr, sizeof(timeStr), "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
            _display.drawStr(2, 11, timeStr);  // Lewy górny róg
            
            // Data w środku (DD.MM.YYYY)
            char dateStr[12];
            snprintf(dateStr, sizeof(dateStr), "%02d.%02d.%04d", 
                     timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
            int dateWidth = _display.getStrWidth(dateStr);
            int dateCenterX = (256 - dateWidth) / 2;  // Wyśrodkuj datę
            _display.drawStr(dateCenterX, 11, dateStr);
            
            // Wykryj rozszerzenie pliku audio
            String audioFormat = "";
            String currentFile = _player->getCurrentFile();
            if (currentFile.length() > 0 && currentFile != "None") {
                int dotPos = currentFile.lastIndexOf('.');
                if (dotPos > 0 && dotPos < currentFile.length() - 1) {
                    audioFormat = currentFile.substring(dotPos + 1);
                    audioFormat.toUpperCase();  // MP3, FLAC, WAV, OGG, AAC
                }
            }
            
            // Ikonka głośnika + Volume po prawej stronie
            int vol = _player->getVolume();
            String volStr = String(vol);
            int volWidth = _display.getStrWidth(volStr.c_str());
            
            // Pozycja głośnika i volume na końcu (prawa strona)
            int speakerX = 256 - volWidth - 20;  // 20px = szerokość ikony + margines
            int speakerY = 3;
            drawVolumeIcon(speakerX, speakerY);
            
            int volX = speakerX + 14;  // Zaraz po ikonie głośnika
            _display.drawStr(volX, 11, volStr.c_str());
            
            // Format audio między datą a głośnikiem (jeśli jest)
            if (audioFormat.length() > 0) {
                int formatWidth = _display.getStrWidth(audioFormat.c_str());
                int formatX = speakerX - formatWidth - 8;  // 8px odstęp od głośnika
                _display.drawStr(formatX, 11, audioFormat.c_str());
            }
        }
    } 
    else {
        // **TYTUŁ UTWORU** - cała szerokość górnego paska
        String currentTrack = _player->getCurrentFile();
        if (currentTrack.length() > 0) {
            // Usuń rozszerzenie
            int dotPos = currentTrack.lastIndexOf('.');
            if (dotPos > 0) {
                currentTrack = currentTrack.substring(0, dotPos);
            }
            
            // Obetnij jeśli za długi
            int maxWidth = 250;  // 256px - marginesy
            while (_display.getStrWidth(currentTrack.c_str()) > maxWidth && currentTrack.length() > 0) {
                currentTrack = currentTrack.substring(0, currentTrack.length() - 1);
            }
            if (currentTrack.length() < _player->getCurrentFile().length() - 4) {
                currentTrack += "...";
            }
            
            // Wyśrodkuj tytuł
            int titleWidth = _display.getStrWidth(currentTrack.c_str());
            int titleX = (256 - titleWidth) / 2;
            _display.drawStr(titleX, 11, currentTrack.c_str());
        }
    }
    
    // **POZIOMA KRESKA** przez cały wyświetlacz
    _display.drawLine(0, 14, 256, 14);
}

void SDPlayerOLED::drawFileList() {
    _display.setFont(u8g2_font_6x10_tr);
    
    const int lineHeight = 12;
    const int startY = 16 + 12;  // Zaczyna się tuż pod górną kreską (14px) + offset
    const int visibleLines = 4;
    
    // Dostosuj scroll offset
    if (_selectedIndex < _scrollOffset) {
        _scrollOffset = _selectedIndex;
    }
    if (_selectedIndex >= _scrollOffset + visibleLines) {
        _scrollOffset = _selectedIndex - visibleLines + 1;
    }
    
    for (int i = 0; i < visibleLines && (i + _scrollOffset) < _fileList.size(); i++) {
        int idx = i + _scrollOffset;
        int y = startY + i * lineHeight;
        
        // Podświetlenie zaznaczonego
        if (idx == _selectedIndex) {
            _display.drawBox(0, y - 10, 245, 11);  // 245px szerokości (zostaw miejsce na scrollbar)
            _display.setDrawColor(0);
        }
        
        // Ikona: folder lub plik
        const char* icon = _fileList[idx].isDir ? ">" : "♪";
        _display.drawStr(2, y, icon);
        
        // Nazwa pliku (obcięta do szerokości)
        String name = _fileList[idx].name;
        if (name.length() > 38) {
            name = name.substring(0, 37) + "~";
        }
        _display.drawStr(12, y, name.c_str());
        
        if (idx == _selectedIndex) {
            _display.setDrawColor(1);
        }
    }
}

void SDPlayerOLED::drawScrollBar(int itemCount, int visibleCount) {
    if (itemCount <= visibleCount) return;
    
    int barHeight = 48;
    int thumbHeight = (barHeight * visibleCount) / itemCount;
    if (thumbHeight < 4) thumbHeight = 4;
    
    int thumbPos = (barHeight - thumbHeight) * _scrollOffset / (itemCount - visibleCount);
    
    // Scrollbar z prawej strony (254px pozycja)
    _display.drawFrame(254, 16, 2, barHeight);
    _display.drawBox(254, 16 + thumbPos, 2, thumbHeight);
}

void SDPlayerOLED::drawVolumeIcon(int x, int y) {
    // Lepiej wyglądający głośnik
    // Podstawa głośnika (trójkąt + prostokąt)
    _display.drawBox(x, y+2, 2, 4);           // Prostokąt bazowy
    _display.drawPixel(x+2, y+1);             // Trójkąt
    _display.drawPixel(x+2, y+2);
    _display.drawPixel(x+2, y+5);
    _display.drawPixel(x+2, y+6);
    _display.drawBox(x+3, y, 2, 8);           // Membrana
    
    // Fale dźwiękowe (3 poziomy)
    _display.drawPixel(x+6, y+2);             // Fala 1 (cicha)
    _display.drawPixel(x+6, y+5);
    _display.drawPixel(x+7, y+1);             // Fala 2 (średnia)
    _display.drawPixel(x+7, y+3);
    _display.drawPixel(x+7, y+4);
    _display.drawPixel(x+7, y+6);
    _display.drawPixel(x+8, y+1);             // Fala 3 (głośna)
    _display.drawPixel(x+8, y+6);
}

void SDPlayerOLED::renderStyle2() {
    // Duży tekst utworu
    if (!_player) return;
    
    _display.setFont(u8g2_font_10x20_tf);
    String current = _player->getCurrentFile();
    if (current == "None") current = "Stopped";
    
    // Podziel na linie
    int y = 20;
    int maxChars = 12;
    for (int i = 0; i < current.length(); i += maxChars) {
        String line = current.substring(i, i + maxChars);
        int w = _display.getStrWidth(line.c_str());
        _display.drawStr((128 - w) / 2, y, line.c_str());
        y += 18;
        if (y > 60) break;
    }
    
    // Volume na dole
    _display.setFont(u8g2_font_6x10_tf);
    String vol = "Vol: " + String(_player->getVolume());
    _display.drawStr(4, 60, vol.c_str());
}

void SDPlayerOLED::renderStyle3() {
    // VU meter + utwór
    drawTopBar();
    
    // Symulacja VU meter (placeholder)
    int vol = _player->getVolume();
    int bars = (vol * 8) / 21;
    
    for (int i = 0; i < 8; i++) {
        int h = 4 + i * 4;
        int x = 10 + i * 14;
        if (i < bars) {
            _display.drawBox(x, 64 - h, 10, h);
        } else {
            _display.drawFrame(x, 64 - h, 10, h);
        }
    }
}

void SDPlayerOLED::renderStyle4() {
    // Spektrum częstotliwości (animacja placeholder)
    drawTopBar();
    
    for (int i = 0; i < 16; i++) {
        int h = 4 + ((_animFrame + i * 3) % 20);
        int x = 4 + i * 8;
        _display.drawBox(x, 64 - h, 6, h);
    }
}

void SDPlayerOLED::renderStyle5() {
    // Minimalistyczny
    if (!_player) return;
    
    _display.setFont(u8g2_font_8x13_tf);
    String current = _player->getCurrentFile();
    if (current == "None") current = "---";
    
    int w = _display.getStrWidth(current.c_str());
    _display.drawStr((128 - w) / 2, 32, current.c_str());
    
    // Volume bar
    int vol = _player->getVolume();
    int barW = (vol * 80) / 21;
    _display.drawFrame(24, 45, 80, 6);
    _display.drawBox(24, 45, barW, 6);
}

void SDPlayerOLED::renderStyle6() {
    // Album art simulation
    _display.drawFrame(30, 5, 68, 54);
    _display.drawBox(32, 7, 64, 50);
    
    // Nazwa na dole
    if (!_player) return;
    _display.setFont(u8g2_font_5x8_tf);
    String current = _player->getCurrentFile();
    if (current.length() > 20) current = current.substring(0, 19) + "~";
    int w = _display.getStrWidth(current.c_str());
    _display.drawStr((128 - w) / 2, 63, current.c_str());
}

void SDPlayerOLED::renderStyle10() {
    // Full screen animated
    renderStyle1(); // Tymczasowo jak Style 1
}

// ===== KONTROLA PILOTA =====

void SDPlayerOLED::onRemoteUp() {
    if (_mode == MODE_NORMAL) {
        scrollUp();
    }
}

void SDPlayerOLED::onRemoteDown() {
    if (_mode == MODE_NORMAL) {
        scrollDown();
    }
}

void SDPlayerOLED::onRemoteOK() {
    if (_mode == MODE_VOLUME) {
        _mode = MODE_NORMAL;
    } else if (_mode == MODE_NORMAL) {
        selectCurrent();
    }
}

void SDPlayerOLED::onRemoteSRC() {
    nextInfoStyle();
}

void SDPlayerOLED::nextInfoStyle() {
    _infoStyle = (_infoStyle == INFO_CLOCK_DATE) ? INFO_TRACK_TITLE : INFO_CLOCK_DATE;
    Serial.printf("SD Player: Info style changed to %s\n", 
                  _infoStyle == INFO_CLOCK_DATE ? "CLOCK/DATE" : "TRACK TITLE");
}

void SDPlayerOLED::onRemoteVolUp() {
    if (!_player) return;
    int vol = _player->getVolume();
    if (vol < 21) {
        _player->setVolume(vol + 1);
    }
    _mode = MODE_VOLUME;
    _volumeShowTime = millis();
}

void SDPlayerOLED::onRemoteVolDown() {
    if (!_player) return;
    int vol = _player->getVolume();
    if (vol > 0) {
        _player->setVolume(vol - 1);
    }
    _mode = MODE_VOLUME;
    _volumeShowTime = millis();
}

// ===== KONTROLA ENKODERA =====

void SDPlayerOLED::onEncoderLeft() {
    scrollUp();
}

void SDPlayerOLED::onEncoderRight() {
    scrollDown();
}

void SDPlayerOLED::onEncoderButton() {
    selectCurrent();
}

// ===== NAWIGACJA =====

void SDPlayerOLED::scrollUp() {
    if (_selectedIndex > 0) {
        _selectedIndex--;
    }
}

void SDPlayerOLED::scrollDown() {
    if (_selectedIndex < _fileList.size() - 1) {
        _selectedIndex++;
    }
}

void SDPlayerOLED::selectCurrent() {
    if (!_player || _selectedIndex >= _fileList.size()) return;
    
    FileEntry& entry = _fileList[_selectedIndex];
    
    if (entry.isDir) {
        // Wejdź do katalogu
        String newPath = _player->getCurrentDirectory();
        if (newPath != "/") newPath += "/";
        newPath += entry.name;
        _player->changeDirectory(newPath);
        refreshFileList();
    } else {
        // Odtwórz plik
        _player->playIndex(_selectedIndex);
        
        // Ustaw flagę że muzyka z SD jest odtwarzana
        sdPlayerPlayingMusic = true;
        
        // Automatycznie wyjdź z panelu SD Player i pokaż style radia
        sdPlayerOLEDActive = false;
        Serial.println("DEBUG: Music started from SD - switching to radio display mode");
        
        // Odśwież wyświetlacz - pokaż style radia
        displayRadio();
        u8g2.sendBuffer();
    }
}

void SDPlayerOLED::nextStyle() {
    switch (_style) {
        case STYLE_1: _style = STYLE_2; break;
        case STYLE_2: _style = STYLE_3; break;
        case STYLE_3: _style = STYLE_4; break;
        case STYLE_4: _style = STYLE_5; break;
        case STYLE_5: _style = STYLE_6; break;
        case STYLE_6: _style = STYLE_10; break;
        case STYLE_10: _style = STYLE_1; break;
    }
}

void SDPlayerOLED::setStyle(DisplayStyle style) {
    _style = style;
}
