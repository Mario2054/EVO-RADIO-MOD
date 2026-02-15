#include "SDPlayerOLED.h"
#include "SDPlayerWebUI.h"
#include "EQ_FFTAnalyzer.h"
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
      _lastSrcPressTime(0),
      _srcClickCount(0),
      _scrollPosition(0),
      _animFrame(0),
      _scrollTextOffset(0),
      _lastScrollTime(0),
      _lastEncoderClickTime(0),
      _encoderClickCount(0),
      _encoderButtonPressStart(0),
      _encoderButtonPressed(false),
      _actionMessage(""),
      _actionMessageTime(0),
      _showActionMessage(false) {
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
    
    // Synchronizuj _selectedIndex z SDPlayerWebUI (dla auto-play)
    if (_player && _mode == MODE_NORMAL) {
        int webIndex = _player->getSelectedIndex();
        if (webIndex != _selectedIndex && webIndex >= 0 && webIndex < _fileList.size()) {
            _selectedIndex = webIndex;
            // Dostosuj scroll offset aby kursor był widoczny
            int visibleLines = 4;  // Liczba widocznych linii na ekranie
            if (_selectedIndex < _scrollOffset) {
                _scrollOffset = _selectedIndex;
            }
            if (_selectedIndex >= _scrollOffset + visibleLines) {
                _scrollOffset = _selectedIndex - visibleLines + 1;
            }
        }
    }
    
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
                case STYLE_7: renderStyle7(); break;
                case STYLE_10: renderStyle10(); break;
            }
            // Ikonki kontroli wyłączone - teraz wbudowane w Style 1
            // drawControlIcons();
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
    // STYL 1: PODSTAWOWY - Tytuł utworu + Lista plików + Format/Volume
    
    if (!_player) return;
    
    String currentFile = _player->getCurrentFile();
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "Zatrzymany";
    } else {
        // Usuń rozszerzenie
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) {
            currentFile = currentFile.substring(0, dotPos);
        }
        // Usuń ścieżkę
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) {
            currentFile = currentFile.substring(slashPos + 1);
        }
    }
    
    _display.setFont(u8g2_font_6x10_tr);
    
    // === GÓRNY PASEK - TYTUŁ UTWORU ===
    int titleMaxWidth = 180; // Zostaw miejsce na format i volume
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > titleMaxWidth) {
        // Płynne scrollowanie w prawo
        static int scrollOffset = 0;
        static unsigned long lastScrollTime = 0;
        
        if (millis() - lastScrollTime > 100) {
            scrollOffset++;
            if (scrollOffset > titleWidth + 30) scrollOffset = 0;
            lastScrollTime = millis();
        }
        
        _display.setClipWindow(2, 0, titleMaxWidth + 2, 14);
        _display.drawStr(2 - scrollOffset, 11, currentFile.c_str());
        // Powtórz tekst dla ciągłego scrollowania
        _display.drawStr(2 - scrollOffset + titleWidth + 30, 11, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        // Wyśrodkowany jeśli się mieści
        int centerX = (titleMaxWidth - titleWidth) / 2;
        _display.drawStr(2 + centerX, 11, currentFile.c_str());
    }
    
    // FORMAT AUDIO
    String audioFormat = "";
    String fullFileName = _player->getCurrentFile();
    if (fullFileName.length() > 0 && fullFileName != "None") {
        int dotPos = fullFileName.lastIndexOf('.');
        if (dotPos > 0) {
            audioFormat = fullFileName.substring(dotPos + 1);
            audioFormat.toUpperCase();
        }
    }
    
    // IKONKA GŁOŚNICZKA + VOLUME
    int vol = _player->getVolume();
    String volStr = String(vol);
    int volWidth = _display.getStrWidth(volStr.c_str());
    
    int speakerX = 256 - volWidth - 20;
    drawVolumeIcon(speakerX, 3);
    _display.drawStr(speakerX + 14, 11, volStr.c_str());
    
    // Format przed głośnikiem
    if (audioFormat.length() > 0) {
        int formatWidth = _display.getStrWidth(audioFormat.c_str());
        int formatX = speakerX - formatWidth - 8;
        _display.drawStr(formatX, 11, audioFormat.c_str());
    }
    
    // === CIENKA LINIA ===
    _display.drawLine(0, 14, 256, 14);
    
    // === LISTA PLIKÓW (3 linie) ===
    _display.setFont(u8g2_font_5x8_tr);
    const int lineHeight = 10;
    const int startY = 28;
    const int visibleLines = 3;
    
    // Dostosuj scroll
    if (_selectedIndex < _scrollOffset) _scrollOffset = _selectedIndex;
    if (_selectedIndex >= _scrollOffset + visibleLines) _scrollOffset = _selectedIndex - visibleLines + 1;
    
    for (int i = 0; i < visibleLines && (i + _scrollOffset) < _fileList.size(); i++) {
        int idx = i + _scrollOffset;
        int y = startY + i * lineHeight;
        
        if (idx == _selectedIndex) {
            _display.drawBox(0, y - 8, 256, 9);
            _display.setDrawColor(0);
        }
        
        // Ikona
        if (_fileList[idx].isDir) {
            _display.drawTriangle(2, y-5, 2, y-2, 5, y-3);
        } else {
            _display.drawStr(2, y, "\xB7");
        }
        
        // Nazwa
        String name = _fileList[idx].name;
        if (name.length() > 48) name = name.substring(0, 47) + "...";
        _display.drawStr(8, y, name.c_str());
        
        if (idx == _selectedIndex) _display.setDrawColor(1);
    }
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
        
        // Ikona: trójkąt dla folderów, nota dla plików
        if (_fileList[idx].isDir) {
            // Trójkąt wskazujący w prawo ►
            _display.drawTriangle(3, y-6, 3, y-2, 7, y-4);
        } else {
            // Nota muzyczna (prostszy symbol)
            _display.drawStr(2, y, "\xB7");  // Kropka jako nota
        }
        
        // Nazwa pliku ze scrollowaniem dla zaznaczonego
        String name = _fileList[idx].name;
        
        if (idx == _selectedIndex) {
            // SCROLLOWANIE dla zaznaczonego elementu
            int maxWidth = 230;  // Max szerokość tekstu
            int nameWidth = _display.getStrWidth(name.c_str());
            
            if (nameWidth > maxWidth) {
                // Scrolluj tekst
                if (millis() - _lastScrollTime > 200) {
                    _scrollTextOffset++;
                    if (_scrollTextOffset > nameWidth + 20) _scrollTextOffset = -maxWidth;
                    _lastScrollTime = millis();
                }
                
                _display.setClipWindow(12, y - 10, 245, y + 2);
                _display.drawStr(12 - _scrollTextOffset, y, name.c_str());
                _display.setMaxClipWindow();
            } else {
                _display.drawStr(12, y, name.c_str());
            }
        } else {
            // Obcięcie dla nie-zaznaczonych
            if (name.length() > 38) {
                name = name.substring(0, 37) + "...";
            }
            _display.drawStr(12, y, name.c_str());
        }
        
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
    // STYL 2: ODTWARZANIE - DUŻY TYTUŁ
    // Tytuł utworu na górze (scrollowany jeśli za długi)
    // Format audio + ikonka głośniczka + volume po prawej
    
    if (!_player) return;
    
    String currentFile = _player->getCurrentFile();
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "Zatrzymany";
    } else {
        // Usuń rozszerzenie
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) {
            currentFile = currentFile.substring(0, dotPos);
        }
        // Usuń ścieżkę
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) {
            currentFile = currentFile.substring(slashPos + 1);
        }
    }
    
    _display.setFont(u8g2_font_6x10_tr);
    
    // === GÓRNY PASEK ===
    // 1. TYTUŁ UTWORU - SCROLLOWANY jeśli za długi
    int titleMaxWidth = 180; // Zostaw miejsce na format i volume
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > titleMaxWidth) {
        // Płynne scrollowanie w prawo
        static int scrollOffset = 0;
        static unsigned long lastScrollTime = 0;
        
        if (millis() - lastScrollTime > 100) {
            scrollOffset++;
            if (scrollOffset > titleWidth + 30) scrollOffset = 0;
            lastScrollTime = millis();
        }
        
        _display.setClipWindow(2, 0, titleMaxWidth + 2, 14);
        _display.drawStr(2 - scrollOffset, 11, currentFile.c_str());
        // Powtórz tekst dla ciągłego scrollowania
        _display.drawStr(2 - scrollOffset + titleWidth + 30, 11, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        // Wyśrodkowany jeśli się mieści
        int centerX = (titleMaxWidth - titleWidth) / 2;
        _display.drawStr(2 + centerX, 11, currentFile.c_str());
    }
    
    // 2. FORMAT AUDIO
    String audioFormat = "";
    String fullFileName = _player->getCurrentFile();
    if (fullFileName.length() > 0 && fullFileName != "None") {
        int dotPos = fullFileName.lastIndexOf('.');
        if (dotPos > 0) {
            audioFormat = fullFileName.substring(dotPos + 1);
            audioFormat.toUpperCase();
        }
    }
    
    // 3. IKONKA GŁOŚNICZKA + VOLUME
    int vol = _player->getVolume();
    String volStr = String(vol);
    int volWidth = _display.getStrWidth(volStr.c_str());
    
    int speakerX = 256 - volWidth - 20;
    drawVolumeIcon(speakerX, 3);
    _display.drawStr(speakerX + 14, 11, volStr.c_str());
    
    // Format przed głośnikiem
    if (audioFormat.length() > 0) {
        int formatWidth = _display.getStrWidth(audioFormat.c_str());
        int formatX = speakerX - formatWidth - 8;
        _display.drawStr(formatX, 11, audioFormat.c_str());
    }
    
    // === CIENKA LINIA ===
    _display.drawLine(0, 14, 256, 14);
    
    // === KOMPAKTOWA LISTA PLIKÓW (3 linie) ===
    _display.setFont(u8g2_font_5x8_tr);
    const int lineHeight = 10;
    const int startY = 28;
    const int visibleLines = 3;
    
    // Dostosuj scroll
    if (_selectedIndex < _scrollOffset) _scrollOffset = _selectedIndex;
    if (_selectedIndex >= _scrollOffset + visibleLines) _scrollOffset = _selectedIndex - visibleLines + 1;
    
    for (int i = 0; i < visibleLines && (i + _scrollOffset) < _fileList.size(); i++) {
        int idx = i + _scrollOffset;
        int y = startY + i * lineHeight;
        
        if (idx == _selectedIndex) {
            _display.drawBox(0, y - 8, 256, 9);
            _display.setDrawColor(0);
        }
        
        // Ikona
        if (_fileList[idx].isDir) {
            _display.drawTriangle(2, y-5, 2, y-2, 5, y-3);
        } else {
            _display.drawStr(2, y, "\xB7");
        }
        
        // Nazwa
        String name = _fileList[idx].name;
        if (name.length() > 48) name = name.substring(0, 47) + "...";
        _display.drawStr(8, y, name.c_str());
        
        if (idx == _selectedIndex) _display.setDrawColor(1);
    }
}

void SDPlayerOLED::renderStyle3() {
    // STYL 3: SPEKTRUM AUDIO - ANIMOWANE SŁUPKI
    // Górny pasek jak w stylu 2
    // Poniżej: spektrum częstotliwości z animacją
    
    if (!_player) return;
    
    String currentFile = _player->getCurrentFile();
    if (currentFile == "None") currentFile = "---";
    else {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) currentFile = currentFile.substring(0, dotPos);
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) currentFile = currentFile.substring(slashPos + 1);
    }
    
    _display.setFont(u8g2_font_6x10_tr);
    
    // === GÓRNY PASEK ===
    // Tytuł ze scrollowaniem
    int titleMaxWidth = 180;
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > titleMaxWidth) {
        // Scrollowanie jak w stylu 2
        static int scrollOffset3 = 0;
        static unsigned long lastScrollTime3 = 0;
        
        if (millis() - lastScrollTime3 > 120) {
            scrollOffset3++;
            if (scrollOffset3 > titleWidth + 25) scrollOffset3 = 0;
            lastScrollTime3 = millis();
        }
        
        _display.setClipWindow(2, 0, titleMaxWidth + 2, 14);
        _display.drawStr(2 - scrollOffset3, 11, currentFile.c_str());
        _display.drawStr(2 - scrollOffset3 + titleWidth + 25, 11, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        int centerX = (titleMaxWidth - titleWidth) / 2;
        _display.drawStr(2 + centerX, 11, currentFile.c_str());
    }
    
    // Format audio
    String fullFileName = _player->getCurrentFile();
    String audioFormat = "";
    if (fullFileName != "None") {
        int dotPos = fullFileName.lastIndexOf('.');
        if (dotPos > 0) {
            audioFormat = fullFileName.substring(dotPos + 1);
            audioFormat.toUpperCase();
        }
    }
    
    // Volume + głośnik
    int vol = _player->getVolume();
    String volStr = String(vol);
    int volWidth = _display.getStrWidth(volStr.c_str());
    int speakerX = 256 - volWidth - 20;
    
    drawVolumeIcon(speakerX, 3);
    _display.drawStr(speakerX + 14, 11, volStr.c_str());
    
    if (audioFormat.length() > 0) {
        int formatWidth = _display.getStrWidth(audioFormat.c_str());
        int formatX = speakerX - formatWidth - 8;
        _display.drawStr(formatX, 11, audioFormat.c_str());
    }
    
    _display.drawLine(0, 14, 256, 14);
    
    // === PRAWDZIWY ANALIZATOR FFT - 16 PASM ===
    float fftLevels[16];
    eq_get_analyzer_levels(fftLevels);
    
    const int numBars = 16;
    const int barWidth = 14;      // Szersze słupki dla 16 pasm
    const int barGap = 2;
    const int barStartY = 18;
    const int maxBarHeight = 20;
    
    for (int i = 0; i < numBars; i++) {
        // Konwersja poziomu FFT (0.0-1.0) na wysokość słupka
        int height = (int)(fftLevels[i] * maxBarHeight);
        if (height > maxBarHeight) height = maxBarHeight;
        if (height < 1 && fftLevels[i] > 0.01f) height = 1; // Min 1px gdy jest sygnał
        
        int x = 2 + i * (barWidth + barGap);
        int y = barStartY + maxBarHeight - height;
        
        // Naprzemienne wypełnione/ramki dla efektu wizualnego
        if (i % 2 == 0) {
            _display.drawBox(x, y, barWidth, height);
        } else {
            _display.drawFrame(x, y, barWidth, height);
            if (height > 2) _display.drawBox(x+1, y+1, barWidth-2, height-2);
        }
    }
    
    // === LISTA PLIKÓW (2 linie) ===
    _display.setFont(u8g2_font_5x8_tr);
    const int lineHeight = 10;
    const int startY = 48;
    const int visibleLines = 2;
    
    if (_selectedIndex < _scrollOffset) _scrollOffset = _selectedIndex;
    if (_selectedIndex >= _scrollOffset + visibleLines) _scrollOffset = _selectedIndex - visibleLines + 1;
    
    for (int i = 0; i < visibleLines && (i + _scrollOffset) < _fileList.size(); i++) {
        int idx = i + _scrollOffset;
        int y = startY + i * lineHeight;
        
        if (idx == _selectedIndex) {
            _display.drawBox(0, y - 8, 256, 9);
            _display.setDrawColor(0);
        }
        
        if (_fileList[idx].isDir) {
            _display.drawTriangle(2, y-5, 2, y-2, 5, y-3);
        } else {
            _display.drawStr(2, y, "\xB7");
        }
        
        String name = _fileList[idx].name;
        if (name.length() > 48) name = name.substring(0, 47) + "...";
        _display.drawStr(8, y, name.c_str());
        
        
        if (idx == _selectedIndex) _display.setDrawColor(1);
    }
}

void SDPlayerOLED::renderStyle4() {
    // STYL 4: DUŻY TYTUŁ + INFORMACJE O PLIKU + LISTA
    // Skoncentrowany na czytelności i informacjach
    
    if (!_player) return;
    
    String currentFile = _player->getCurrentFile();
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "Zatrzymany";
    } else {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) currentFile = currentFile.substring(0, dotPos);
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) currentFile = currentFile.substring(slashPos + 1);
    }
    
    // === DUŻY TYTUŁ (większa czcionka) ===
    _display.setFont(u8g2_font_7x13_tf);
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > 250) {
        static int scrollOffset4 = 0;
        static unsigned long lastScrollTime4 = 0;
        
        if (millis() - lastScrollTime4 > 90) {
            scrollOffset4++;
            if (scrollOffset4 > titleWidth + 35) scrollOffset4 = 0;
            lastScrollTime4 = millis();
        }
        
        _display.setClipWindow(3, 0, 253, 15);
        _display.drawStr(3 - scrollOffset4, 12, currentFile.c_str());
        _display.drawStr(3 - scrollOffset4 + titleWidth + 35, 12, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        int centerX = (256 - titleWidth) / 2;
        _display.drawStr(centerX, 12, currentFile.c_str());
    }
    
    _display.drawLine(0, 15, 256, 15);
    
    // === INFORMACJE O PLIKU (format, volume, status) ===
    _display.setFont(u8g2_font_6x10_tr);
    
    // Format audio
    String fullFileName = _player->getCurrentFile();
    String audioFormat = "";
    if (fullFileName != "None") {
        int dotPos = fullFileName.lastIndexOf('.');
        if (dotPos > 0) {
            audioFormat = fullFileName.substring(dotPos + 1);
            audioFormat.toUpperCase();
        }
    }
    
    // Status odtwarzania
    String status = "STOP";
    if (_player->isPlaying() && !_player->isPaused()) {
        status = "PLAY";
    } else if (_player->isPaused()) {
        status = "PAUSE";
    }
    
    // Lewa strona: Format
    if (audioFormat.length() > 0) {
        _display.drawStr(4, 26, "Format:");
        _display.drawStr(50, 26, audioFormat.c_str());
    }
    
    // Prawa strona: Status
    _display.drawStr(140, 26, "Status:");
    _display.drawStr(190, 26, status.c_str());
    
    // Druga linia info: Volume
    int vol = _player->getVolume();
    char volStr[20];
    snprintf(volStr, sizeof(volStr), "Volume: %d", vol);
    
    _display.drawStr(4, 36, volStr);
    
    // Pasek volume (wizualizacja)
    int barWidth = (vol * 80) / 21;  // max vol 21 -> 80px
    _display.drawFrame(140, 28, 82, 8);
    if (barWidth > 0) {
        _display.drawBox(141, 29, barWidth, 6);
    }
    
    _display.drawLine(0, 40, 256, 40);
    
    // === LISTA PLIKÓW (2 linie) ===
    _display.setFont(u8g2_font_5x8_tr);
    const int lineHeight = 10;
    const int startY = 52;
    const int visibleLines = 2;
    
    if (_selectedIndex < _scrollOffset) _scrollOffset = _selectedIndex;
    if (_selectedIndex >= _scrollOffset + visibleLines) _scrollOffset = _selectedIndex - visibleLines + 1;
    
    for (int i = 0; i < visibleLines && (i + _scrollOffset) < _fileList.size(); i++) {
        int idx = i + _scrollOffset;
        int y = startY + i * lineHeight;
        
        if (idx == _selectedIndex) {
            _display.drawBox(0, y - 8, 256, 9);
            _display.setDrawColor(0);
        }
        
        if (_fileList[idx].isDir) {
            _display.drawTriangle(2, y-5, 2, y-2, 5, y-3);
        } else {
            _display.drawStr(2, y, "\xB7");
        }
        
        String name = _fileList[idx].name;
        if (name.length() > 48) name = name.substring(0, 47) + "...";
        _display.drawStr(8, y, name.c_str());
        
        if (idx == _selectedIndex) _display.setDrawColor(1);
    }
}

void SDPlayerOLED::renderStyle5() {
    // STYL 5: MINIMALISTYCZNY
    // Tytuł scrollowany + prosty pasek volume
    
    if (!_player) return;
    
    // === TYTUŁ SCROLLOWANY ===
    String currentFile = _player->getCurrentFile();
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "---";
    } else {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) currentFile = currentFile.substring(0, dotPos);
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) currentFile = currentFile.substring(slashPos + 1);
    }
    
    _display.setFont(u8g2_font_8x13_tf);
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > 240) {
        static int scrollOffset = 0;
        static unsigned long lastScrollTime = 0;
        
        if (millis() - lastScrollTime > 120) {
            scrollOffset++;
            if (scrollOffset > titleWidth + 30) scrollOffset = 0;
            lastScrollTime = millis();
        }
        
        _display.setClipWindow(8, 0, 248, 35);
        _display.drawStr(8 - scrollOffset, 30, currentFile.c_str());
        _display.drawStr(8 - scrollOffset + titleWidth + 30, 30, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        int centerX = (256 - titleWidth) / 2;
        _display.drawStr(centerX, 30, currentFile.c_str());
    }
    
    // === VOLUME BAR (mały) ===
    int vol = _player->getVolume();
    int barW = (vol * 200) / 21;
    _display.drawFrame(28, 38, 200, 6);
    _display.drawBox(28, 38, barW, 6);
    
    // === LISTA PLIKÓW (2 linie) ===
    _display.setFont(u8g2_font_5x8_tr);
    const int lineHeight = 10;
    const int startY = 50;
    const int visibleLines = 2;
    
    if (_selectedIndex < _scrollOffset) _scrollOffset = _selectedIndex;
    if (_selectedIndex >= _scrollOffset + visibleLines) _scrollOffset = _selectedIndex - visibleLines + 1;
    
    for (int i = 0; i < visibleLines && (i + _scrollOffset) < _fileList.size(); i++) {
        int idx = i + _scrollOffset;
        int y = startY + i * lineHeight;
        
        if (idx == _selectedIndex) {
            _display.drawBox(0, y - 8, 256, 9);
            _display.setDrawColor(0);
        }
        
        if (_fileList[idx].isDir) {
            _display.drawTriangle(2, y-5, 2, y-2, 5, y-3);
        } else {
            _display.drawStr(2, y, "\xB7");
        }
        
        String name = _fileList[idx].name;
        if (name.length() > 48) name = name.substring(0, 47) + "...";
        _display.drawStr(8, y, name.c_str());
        
        if (idx == _selectedIndex) _display.setDrawColor(1);
    }
}

void SDPlayerOLED::renderStyle6() {
    // STYL 6: INFO AUDIO + PROGRESS BAR
    // Szczegółowe informacje o pliku audio z paskiem postępu
    
    if (!_player) return;
    
    _display.setFont(u8g2_font_6x10_tr);
    
    // === TYTUŁ SCROLLOWANY ===
    String currentFile = _player->getCurrentFile();
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "Brak utworu";
    } else {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) currentFile = currentFile.substring(0, dotPos);
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) currentFile = currentFile.substring(slashPos + 1);
    }
    
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    
    if (titleWidth > 240) {
        static int scrollOffset = 0;
        static unsigned long lastScrollTime = 0;
        
        if (millis() - lastScrollTime > 100) {
            scrollOffset++;
            if (scrollOffset > titleWidth + 30) scrollOffset = 0;
            lastScrollTime = millis();
        }
        
        _display.setClipWindow(8, 0, 248, 14);
        _display.drawStr(8 - scrollOffset, 11, currentFile.c_str());
        _display.drawStr(8 - scrollOffset + titleWidth + 30, 11, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        int centerX = (256 - titleWidth) / 2;
        _display.drawStr(centerX, 11, currentFile.c_str());
    }
    
    _display.drawLine(0, 14, 256, 14);
    
    // === INFORMACJE O AUDIO (2 kolumny) ===
    _display.setFont(u8g2_font_5x8_tr);
    
    // Lewa kolumna
    _display.drawStr(4, 24, "Format:");
    String audioFormat = "";
    String fullFileName = _player->getCurrentFile();
    if (fullFileName.length() > 0 && fullFileName != "None") {
        int dotPos = fullFileName.lastIndexOf('.');
        if (dotPos > 0) {
            audioFormat = fullFileName.substring(dotPos + 1);
            audioFormat.toUpperCase();
        }
    }
    if (audioFormat.length() > 0) {
        _display.drawStr(42, 24, audioFormat.c_str());
    } else {
        _display.drawStr(42, 24, "---");
    }
    
    _display.drawStr(4, 34, "Bitrate:");
    _display.drawStr(42, 34, "192k");  // Można dodać rzeczywiste dane z audio
    
    // Prawa kolumna
    _display.drawStr(130, 24, "Volume:");
    int vol = _player->getVolume();
    String volStr = String(vol);
    _display.drawStr(168, 24, volStr.c_str());
    
    _display.drawStr(130, 34, "Status:");
    if (_player->isPlaying() && !_player->isPaused()) {
        _display.drawStr(168, 34, "PLAY");
    } else if (_player->isPaused()) {
        _display.drawStr(168, 34, "PAUSE");
    } else {
        _display.drawStr(168, 34, "STOP");
    }
    
    // === PASEK POSTĘPU (z czasem) ===
    _display.setFont(u8g2_font_5x8_tr);
    
    // Symulacja czasu - w rzeczywistości pobierz z player
    int currentSeconds = 125;  // 2:05
    int totalSeconds = 245;    // 4:05
    
    // Formatowanie czasu
    char currentTime[8];
    char totalTime[8];
    snprintf(currentTime, sizeof(currentTime), "%d:%02d", currentSeconds / 60, currentSeconds % 60);
    snprintf(totalTime, sizeof(totalTime), "%d:%02d", totalSeconds / 60, totalSeconds % 60);
    
    // Wyświetl czas po lewej i prawej
    _display.drawStr(4, 46, currentTime);
    int totalTimeWidth = _display.getStrWidth(totalTime);
    _display.drawStr(256 - totalTimeWidth - 4, 46, totalTime);
    
    // Pasek postępu (między czasami)
    int progressBarX = 30;
    int progressBarY = 40;
    int progressBarWidth = 195;
    int progressBarHeight = 8;
    
    _display.drawFrame(progressBarX, progressBarY, progressBarWidth, progressBarHeight);
    
    // Wypełnienie na podstawie postępu
    int progressFill = 0;
    if (totalSeconds > 0) {
        progressFill = (currentSeconds * (progressBarWidth - 2)) / totalSeconds;
    }
    if (progressFill > 0) {
        _display.drawBox(progressBarX + 1, progressBarY + 1, progressFill, progressBarHeight - 2);
    }
    
    // === LISTA PLIKÓW (1 linia kompaktowa) ===
    _display.setFont(u8g2_font_5x8_tr);
    const int startY = 58;
    
    if (_selectedIndex < _fileList.size()) {
        int y = startY;
        
        if (_fileList[_selectedIndex].isDir) {
            _display.drawTriangle(2, y-5, 2, y-2, 5, y-3);
        } else {
            _display.drawStr(2, y, "\xB7");
        }
        
        String name = _fileList[_selectedIndex].name;
        if (name.length() > 48) name = name.substring(0, 47) + "...";
        _display.drawStr(8, y, name.c_str());
    }
}

void SDPlayerOLED::renderStyle7() {
    // STYL 7: ANALIZATOR RETRO
    // Góra: Data | System | Głośniczek+Volume
    // Tytuł utworu (scrollowany)
    // Długa kreska
    // Analizator FFT z trójkątnymi słupkami (podstawa szersza, im wyżej tym węższe)
    
    if (!_player) return;
    
    _display.setFont(u8g2_font_6x10_tr);
    
    // === GÓRNY PASEK ===
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        // 1. DATA PO LEWEJ (DD.MM.YYYY)
        char dateStr[12];
        snprintf(dateStr, sizeof(dateStr), "%02d.%02d.%04d", 
                 timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
        _display.drawStr(2, 11, dateStr);
        
        // 2. SYSTEM W ŚRODKU
        const char* systemText = "SD PLAYER";
        int systemWidth = _display.getStrWidth(systemText);
        int systemCenterX = (256 - systemWidth) / 2;
        _display.drawStr(systemCenterX, 11, systemText);
        
        // 3. IKONKA GŁOŚNICZKA + VOLUME PO PRAWEJ
        int vol = _player->getVolume();
        String volStr = String(vol);
        int volWidth = _display.getStrWidth(volStr.c_str());
        
        int speakerX = 256 - volWidth - 20;
        drawVolumeIcon(speakerX, 3);
        _display.drawStr(speakerX + 14, 11, volStr.c_str());
    }
    
    // === CIENKA LINIA POD PASKIEM ===
    _display.drawLine(0, 14, 256, 14);
    
    // === TYTUŁ UTWORU (scrollowany) ===
    String currentFile = _player->getCurrentFile();
    if (currentFile == "None" || currentFile.length() == 0) {
        currentFile = "---";
    } else {
        int dotPos = currentFile.lastIndexOf('.');
        if (dotPos > 0) currentFile = currentFile.substring(0, dotPos);
        int slashPos = currentFile.lastIndexOf('/');
        if (slashPos >= 0) currentFile = currentFile.substring(slashPos + 1);
    }
    
    _display.setFont(u8g2_font_7x13_tf);
    int titleWidth = _display.getStrWidth(currentFile.c_str());
    int titleY = 26;
    
    if (titleWidth > 250) {
        // Scrolluj długi tytuł
        static int scrollOffset = 0;
        static unsigned long lastScrollTime = 0;
        
        if (millis() - lastScrollTime > 100) {
            scrollOffset++;
            if (scrollOffset > titleWidth + 30) scrollOffset = 0;
            lastScrollTime = millis();
        }
        
        _display.setClipWindow(3, 16, 253, 30);
        _display.drawStr(3 - scrollOffset, titleY, currentFile.c_str());
        _display.drawStr(3 - scrollOffset + titleWidth + 30, titleY, currentFile.c_str());
        _display.setMaxClipWindow();
    } else {
        // Wyśrodkuj krótki tytuł
        int centerX = (256 - titleWidth) / 2;
        _display.drawStr(centerX, titleY, currentFile.c_str());
    }
    
    // === DŁUGA KRESKA POD TYTUŁEM ===
    _display.drawLine(3, 30, 253, 30);
    
    // === ANALIZATOR FFT Z TRÓJKĄTNYMI SŁUPKAMI ===
    // Pobierz dane z analizatora FFT
    float levels[EQ_BANDS];
    eq_get_analyzer_levels(levels);
    
    const int analyzerY = 62;  // Dolna linia analizatora
    const int maxHeight = 28;  // Maksymalna wysokość słupka
    const int barSpacing = 2;  // Odstęp między słupkami
    const int totalWidth = 250; // Szerokość obszaru analizatora
    const int barWidth = (totalWidth - (EQ_BANDS - 1) * barSpacing) / EQ_BANDS;
    const int startX = 3;
    
    for (int i = 0; i < EQ_BANDS; i++) {
        // Wysokość słupka bazująca na poziomie FFT
        int height = (int)(levels[i] * maxHeight);
        if (height > maxHeight) height = maxHeight;
        if (height < 1) height = 1;
        
        int x = startX + i * (barWidth + barSpacing);
        int y = analyzerY;
        
        // Rysuj trójkątny słupek - podstawa szersza, im wyżej tym węższy
        // Używamy kropek/pikseli do stworzenia efektu trójkąta
        for (int h = 0; h < height; h++) {
            // Oblicz szerokość na danej wysokości (od pełnej na dole do 1 na górze)
            int widthAtHeight = barWidth - (h * barWidth / maxHeight);
            if (widthAtHeight < 1) widthAtHeight = 1;
            
            // Wyśrodkuj szerokość na danej wysokości
            int xOffset = (barWidth - widthAtHeight) / 2;
            
            // Rysuj linię o odpowiedniej szerokości
            if (h % 2 == 0) {  // Co drugi piksel dla efektu "kropek"
                _display.drawLine(x + xOffset, y - h, x + xOffset + widthAtHeight - 1, y - h);
            } else {
                // Dla efektu kropkowego rysuj tylko co 2 piksel w poziomie
                for (int px = 0; px < widthAtHeight; px += 2) {
                    _display.drawPixel(x + xOffset + px, y - h);
                }
            }
        }
    }
}

void SDPlayerOLED::renderStyle10() {
    // === STYL 10 - Floating Peaks (Ulatujące szczyty) - FFT Analyzer ===
    
    // Pobierz poziomy FFT z analizatora
    float fftLevels[16];
    float fftPeaks[16];
    eq_get_analyzer_levels(fftLevels);
    eq_get_analyzer_peaks(fftPeaks);
    
    // Wygładzanie poziomów
    static uint8_t muteLevel[16] = {0};
    static float smoothedLevel[16] = {0.0f};
    const float smoothFactor = 0.55f; // 45% smoothness (100-45)/100
    
    bool isMuted = (_player->getVolume() == 0);
    
    for (uint8_t i = 0; i < 16; i++) {
        if (isMuted) {
            // Animacja wyciszenia - stopniowe zmniejszanie
            if (muteLevel[i] > 2) muteLevel[i] -= 2;
            else muteLevel[i] = 0;
            smoothedLevel[i] = 0.0f;
        } else {
            float lv = fftLevels[i];
            if (lv < 0.0f) lv = 0.0f;
            if (lv > 1.0f) lv = 1.0f;
            
            // Szybszy atak, wolniejsze opadanie
            if (lv > smoothedLevel[i]) {
                float attackSpeed = 0.3f + smoothFactor * 0.7f;
                smoothedLevel[i] = smoothedLevel[i] + attackSpeed * (lv - smoothedLevel[i]);
            } else {
                float releaseSpeed = 0.1f + smoothFactor * 0.6f;
                smoothedLevel[i] = smoothedLevel[i] + releaseSpeed * (lv - smoothedLevel[i]);
            }
            
            muteLevel[i] = (uint8_t)(smoothedLevel[i] * 100.0f + 0.5f);
        }
    }
    
    // === GÓRNY PASEK: Tytuł utworu + Volume ===
    _display.setFont(u8g2_font_6x10_tr);
    
    String currentFile = _player->getCurrentFile();
    if (currentFile == "None") currentFile = "NO FILE";
    
    // Usuń rozszerzenie
    int dotPos = currentFile.lastIndexOf('.');
    if (dotPos > 0) {
        currentFile = currentFile.substring(0, dotPos);
    }
    
    // Skróć jeśli za długi
    if (currentFile.length() > 35) {
        currentFile = currentFile.substring(0, 32) + "...";
    }
    
    _display.drawStr(4, 10, currentFile.c_str());
    
    // Volume po prawej
    int vol = _player->getVolume();
    char volStr[15];
    snprintf(volStr, sizeof(volStr), "Vol:%d", vol);
    int volW = _display.getStrWidth(volStr);
    _display.drawStr(256 - volW - 4, 10, volStr);
    
    _display.drawLine(0, 13, 256, 13);
    
    // === FLOATING PEAKS ANALYZER ===
    const uint8_t eqTopY = 14;
    const uint8_t eqBottomY = 63;
    const uint8_t maxSegments = 32;
    
    // Parametry słupków - hardcoded (można później przenieść do cfg)
    const uint8_t barWidth = 12;
    const uint8_t barGap = 2;
    const uint8_t segmentHeight = 2;
    const uint8_t segmentGap = 1;
    const uint8_t maxPeaksActive = 3;
    const uint8_t peakHoldTime = 8;
    const uint8_t peakFloatSpeed = 8;
    
    const uint16_t totalBarsWidth = 16 * barWidth + 15 * barGap;
    int16_t startX = (256 - totalBarsWidth) / 2;
    if (startX < 2) startX = 2;
    
    // Floating peaks - wiele peaków na słupek
    static const uint8_t MAX_PEAKS_ARRAY = 5;
    struct FlyingPeak {
        float y;
        uint8_t holdCounter;
        bool active;
    };
    static FlyingPeak flyingPeaks[16][MAX_PEAKS_ARRAY] = {};
    static uint8_t lastPeakSeg[16] = {0};
    static bool wasRising[16] = {false};
    
    for (uint8_t i = 0; i < 16; i++) {
        uint8_t levelPercent = muteLevel[i];
        float peakVal = fftPeaks[i];
        if (peakVal < 0.0f) peakVal = 0.0f;
        if (peakVal > 1.0f) peakVal = 1.0f;
        uint8_t peakPercent = (uint8_t)(peakVal * 100.0f + 0.5f);
        
        uint8_t segments = (levelPercent * maxSegments) / 100;
        if (segments > maxSegments) segments = maxSegments;
        
        uint8_t peakSeg = (peakPercent * maxSegments) / 100;
        if (peakSeg > maxSegments) peakSeg = maxSegments;
        
        int16_t x = startX + i * (barWidth + barGap);
        
        // Rysuj słupki (segmenty)
        for (uint8_t s = 0; s < segments; s++) {
            int16_t segBottom = eqBottomY - (s * (segmentHeight + segmentGap));
            int16_t segTop = segBottom - segmentHeight + 1;
            if (segTop < eqTopY) segTop = eqTopY;
            if (segBottom > eqBottomY) segBottom = eqBottomY;
            if (segTop <= segBottom) {
                _display.drawBox(x, segTop, barWidth, segmentHeight);
            }
        }
        
        // Oblicz pozycję góry słupka
        int16_t barTopY = eqBottomY;
        if (segments > 0) {
            int16_t topSegBottom = eqBottomY - ((segments - 1) * (segmentHeight + segmentGap));
            barTopY = topSegBottom - segmentHeight + 1;
            if (barTopY < eqTopY) barTopY = eqTopY;
        }
        
        // Pozycja peak
        int16_t currentPeakY = eqBottomY;
        if (peakSeg > 0) {
            uint8_t ps = peakSeg - 1;
            int16_t peakSegBottom = eqBottomY - (ps * (segmentHeight + segmentGap));
            currentPeakY = peakSegBottom - segmentHeight + 1 - 2;
            if (currentPeakY < eqTopY) currentPeakY = eqTopY;
        }
        
        // Wykryj nowy szczyt - gdy peak rósł i teraz spada
        bool isRising = (peakSeg > lastPeakSeg[i]);
        bool justPeaked = (wasRising[i] && !isRising && peakSeg > 0);
        
        // Wystrzel nowy floating peak
        if (justPeaked) {
            for (uint8_t p = 0; p < maxPeaksActive; p++) {
                if (!flyingPeaks[i][p].active) {
                    flyingPeaks[i][p].y = (float)currentPeakY;
                    flyingPeaks[i][p].holdCounter = peakHoldTime;
                    flyingPeaks[i][p].active = true;
                    break;
                }
            }
        }
        
        wasRising[i] = isRising;
        lastPeakSeg[i] = peakSeg;
        
        // Aktualizuj i rysuj floating peaks
        for (uint8_t p = 0; p < MAX_PEAKS_ARRAY; p++) {
            if (flyingPeaks[i][p].active) {
                // Hold time - peak czeka
                if (flyingPeaks[i][p].holdCounter > 0) {
                    flyingPeaks[i][p].holdCounter--;
                } else {
                    // Peak odlatuje w górę
                    flyingPeaks[i][p].y -= (float)peakFloatSpeed * 0.5f;
                }
                
                // Rysuj peak TYLKO powyżej słupka
                int16_t peakY = (int16_t)flyingPeaks[i][p].y;
                if (peakY < barTopY && peakY >= eqTopY) {
                    _display.drawBox(x, peakY, barWidth, 1);
                } else if (peakY < eqTopY) {
                    flyingPeaks[i][p].active = false;
                }
            }
        }
    }
}

// ===== KONTROLA PILOTA =====

void SDPlayerOLED::onRemoteUp() {
    if (_mode == MODE_NORMAL) {
        scrollUp();
        showActionMessage("UP");
    }
}

void SDPlayerOLED::onRemoteDown() {
    if (_mode == MODE_NORMAL) {
        scrollDown();
        showActionMessage("DOWN");
    }
}

void SDPlayerOLED::onRemoteOK() {
    // Jeśli coś jest odtwarzane lub w pauzie, OK działa jako Play/Pause
    if (_player && _player->isPlaying()) {
        bool wasPaused = _player->isPaused();
        _player->pause();  // Toggle play/pause
        showActionMessage(wasPaused ? "PLAY" : "PAUSE");
        Serial.println("SD Player: OK button - Toggle Play/Pause");
    } else if (_mode == MODE_VOLUME) {
        _mode = MODE_NORMAL;
    } else if (_mode == MODE_NORMAL) {
        selectCurrent();
        showActionMessage("PLAY");
    }
}

void SDPlayerOLED::onRemotePlayPause() {
    // Dedykowany przycisk Play/Pause
    if (_player) {
        bool wasPaused = _player->isPaused();
        _player->pause();  // Toggle play/pause
        showActionMessage(wasPaused ? "PLAY" : "PAUSE");
        Serial.println("SD Player: Play/Pause button pressed");
    }
}

void SDPlayerOLED::onRemoteSRC() {
    // Podwójne kliknięcie SRC = zmiana stylu wyświetlania
    unsigned long currentTime = millis();
    
    if (currentTime - _lastSrcPressTime > 1000) {
        // Timeout - reset licznika
        _srcClickCount = 0;
    }
    
    _srcClickCount++;
    _lastSrcPressTime = currentTime;
    
    if (_srcClickCount >= 2) {
        // Podwójne kliknięcie - zmień styl
        nextStyle();
        _srcClickCount = 0;
        Serial.printf("SDPlayer: Zmiana stylu OLED na %d\n", (int)_style);
    } else {
        // Pojedyncze - zmień InfoStyle (zegar/tytuł)
        nextInfoStyle();
    }
}

// ===== KOMUNIKATY AKCJI I IKONKI =====

void SDPlayerOLED::showActionMessage(const String& message) {
    _actionMessage = message;
    _actionMessageTime = millis();
    _showActionMessage = true;
}

void SDPlayerOLED::drawIconPrev(int x, int y) {
    // ⏮️ Previous - podwójny trójkąt w lewo
    _display.drawTriangle(x+8, y, x+8, y+8, x+3, y+4);
    _display.drawTriangle(x+3, y, x+3, y+8, x, y+4);
}

void SDPlayerOLED::drawIconUp(int x, int y) {
    // ⬆️ Up - strzałka w górę
    _display.drawTriangle(x+4, y, x, y+5, x+8, y+5);
    _display.drawLine(x+3, y+4, x+3, y+8);
    _display.drawLine(x+4, y+4, x+4, y+8);
    _display.drawLine(x+5, y+4, x+5, y+8);
}

void SDPlayerOLED::drawIconPause(int x, int y) {
    // ⏸️ Pause - dwie pionowe kreski
    _display.drawBox(x+1, y, 3, 8);
    _display.drawBox(x+6, y, 3, 8);
}

void SDPlayerOLED::drawIconPlay(int x, int y) {
    // ▶️ Play - trójkąt w prawo
    _display.drawTriangle(x, y, x, y+8, x+7, y+4);
}

void SDPlayerOLED::drawIconStop(int x, int y) {
    // ⏹️ Stop - kwadrat
    _display.drawBox(x+1, y+1, 7, 7);
}

void SDPlayerOLED::drawIconNext(int x, int y) {
    // ⏭️ Next - podwójny trójkąt w prawo
    _display.drawTriangle(x, y, x, y+8, x+5, y+4);
    _display.drawTriangle(x+5, y, x+5, y+8, x+10, y+4);
}

void SDPlayerOLED::drawIconDown(int x, int y) {
    // ⬇️ Down - strzałka w dół
    _display.drawLine(x+3, y, x+3, y+4);
    _display.drawLine(x+4, y, x+4, y+4);
    _display.drawLine(x+5, y, x+5, y+4);
    _display.drawTriangle(x+4, y+8, x, y+3, x+8, y+3);
}

void SDPlayerOLED::drawControlIcons() {
    // Rysuje ikonki kontroli W GÓRNYM PASKU (obok zegara i daty)
    // Układ górnego paska: Zegar | Data | [⬆️] [⏸️/▶️] [⏹️] [⬇️] | Format | Głośnik+Vol
    
    int y = 3;         // Pozycja Y (w górnym pasku, wyrównane z tekstem)
    int spacing = 13;  // Zmniejszony odstęp dla 4 ikon
    int startX = 128;  // Przesunięte w lewo dla 4 ikon
    
    // Ikonka Up (przewijanie w górę)
    drawIconUp(startX, y);
    
    // Ikonka Pause/Play (w zależności od stanu)
    if (_player && _player->isPlaying() && !_player->isPaused()) {
        drawIconPause(startX + spacing, y);
    } else {
        drawIconPlay(startX + spacing, y);
    }
    
    // Ikonka Stop (całkowite zatrzymanie)
    drawIconStop(startX + spacing * 2, y);
    
    // Ikonka Down (przewijanie w dół)
    drawIconDown(startX + spacing * 3, y);
    
    // Jeśli jest aktywny komunikat, narysuj go PONIŻEJ górnego paska (pod kreską)
    if (_showActionMessage && (millis() - _actionMessageTime < 2000)) {
        _display.setFont(u8g2_font_6x10_tr);
        int msgWidth = _display.getStrWidth(_actionMessage.c_str());
        int msgX = (256 - msgWidth) / 2;  // Wycentruj
        _display.drawStr(msgX, 28, _actionMessage.c_str());  // Obniżone o ~2mm (8 pikseli)
    } else {
        _showActionMessage = false;  // Ukryj po 2 sekundach
    }
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

void SDPlayerOLED::onRemoteStop() {
    if (!_player) return;
    
    _player->stop();
    sdPlayerPlayingMusic = false;
    
    showActionMessage("STOP");
    Serial.println("SD Player: STOP button - Playback stopped");
}

// ===== KONTROLA ENKODERA =====

void SDPlayerOLED::onEncoderLeft() {
    // Lewo = Góra (w górę listy)
    scrollUp();
    showActionMessage("UP");
}

void SDPlayerOLED::onEncoderRight() {
    // Prawo = Dół (w dół listy)
    scrollDown();
    showActionMessage("DOWN");
}

bool SDPlayerOLED::checkEncoderLongPress(bool buttonState) {
    unsigned long now = millis();
    
    if (buttonState && !_encoderButtonPressed) {
        // Przycisk właśnie został wciśnięty
        _encoderButtonPressed = true;
        _encoderButtonPressStart = now;
    } else if (!buttonState && _encoderButtonPressed) {
        // Przycisk został zwolniony
        _encoderButtonPressed = false;
    }
    
    // Sprawdź czy przycisk jest przytrzymany przez 4 sekundy
    if (_encoderButtonPressed && (now - _encoderButtonPressStart >= 4000)) {
        // DŁUGIE PRZYTRZYMANIE (4 sek) - WYJŚCIE DO RADIA
        Serial.println("SD Player: Long press detected (4s) - returning to radio");
        showActionMessage("EXIT");
        sdPlayerOLEDActive = false;
        sdPlayerPlayingMusic = false;
        _encoderButtonPressed = false; // Reset flagi
        deactivate();
        displayRadio();
        return true; // Zwracamy true - długie przytrzymanie wykryte
    }
    
    return false; // Brak długiego przytrzymania
}

void SDPlayerOLED::onEncoderButtonHold(unsigned long holdTime) {
    // Ta metoda może być wywołana z main.cpp gdy wykryje długie przytrzymanie
    if (holdTime >= 4000) {
        Serial.println("SD Player: Long hold detected - returning to radio");
        showActionMessage("EXIT");
        sdPlayerOLEDActive = false;
        sdPlayerPlayingMusic = false;
        deactivate();
        displayRadio();
    }
}

void SDPlayerOLED::onEncoderButton() {
    // Nie reaguj jeśli przycisk był długo przytrzymany
    if (_encoderButtonPressed) {
        return;
    }
    
    unsigned long now = millis();
    
    // Wykrywanie potrójnego kliknięcia (w ciągu 600ms)
    if (now - _lastEncoderClickTime < 600) {
        _encoderClickCount++;
        
        if (_encoderClickCount >= 3) {
            // POTRÓJNE KLIKNIĘCIE - WYJŚCIE DO RADIA
            Serial.println("SD Player: Triple click detected - returning to radio");
            showActionMessage("EXIT");
            sdPlayerOLEDActive = false;
            sdPlayerPlayingMusic = false;
            deactivate();
            displayRadio();
            _encoderClickCount = 0;
            return;
        }
    } else {
        _encoderClickCount = 1;
    }
    
    _lastEncoderClickTime = now;
    
    // POJEDYNCZE KLIKNIĘCIE
    if (_player && _player->isPlaying()) {
        // Jeśli coś gra lub jest w pauzie - toggle pause
        bool wasPaused = _player->isPaused();
        _player->pause();
        showActionMessage(wasPaused ? "PLAY" : "PAUSE");
        Serial.println("SD Player: Encoder button - Toggle Play/Pause");
    } else {
        // Jeśli nic nie gra - wybierz i odtwórz utwór
        selectCurrent();
        showActionMessage("PLAY");
    }
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
        // ZATRZYMAJ OBECNY UTWÓR przed odtworzeniem nowego
        if (_player->isPlaying()) {
            Serial.println("DEBUG: Stopping current track before playing new one");
            _player->stop();  // Zatrzymaj obecny utwór
            delay(50);  // Krótka pauza dla stabilności
        }
        
        // Odtwórz nowy plik
        _player->playIndex(_selectedIndex);
        sdPlayerPlayingMusic = true;
        Serial.println("DEBUG: Playing selected track from SD Player");
        
        // POZOSTAJEMY w panelu SD Player OLED - nie wychodzimy automatycznie
        Serial.println("DEBUG: Music started from SD - staying in SD Player panel");
    }
}

void SDPlayerOLED::nextStyle() {
    switch (_style) {
        case STYLE_1: _style = STYLE_2; break;
        case STYLE_2: _style = STYLE_3; break;
        case STYLE_3: _style = STYLE_4; break;
        case STYLE_4: _style = STYLE_5; break;
        case STYLE_5: _style = STYLE_6; break;
        case STYLE_6: _style = STYLE_7; break;
        case STYLE_7: _style = STYLE_10; break;
        case STYLE_10: _style = STYLE_1; break;
    }
}

void SDPlayerOLED::setStyle(DisplayStyle style) {
    _style = style;
}
