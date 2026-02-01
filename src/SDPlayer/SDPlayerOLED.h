#pragma once
#include <Arduino.h>
#include <U8g2lib.h>
#include <vector>

// Forward declaration
class SDPlayerWebUI;

/**
 * SDPlayerOLED - Obsługa wizualizacji SD Player na OLED
 * 
 * Funkcje:
 * - Wyświetlanie aktualnie odtwarzanego utworu
 * - Lista utworów z nawigacją
 * - Wskaźnik volume z ikoną głośnika
 * - 7 stylów wyświetlania (1-6 + 10)
 * - Obsługa pilota (góra/dół/OK/SRC/VOL)
 * - Obsługa enkodera
 */

class SDPlayerOLED {
public:
    SDPlayerOLED(U8G2& display);
    
    // Inicjalizacja
    void begin(SDPlayerWebUI* player);
    
    // Główna pętla - wywołuj w loop()
    void loop();
    
    // Aktywacja/deaktywacja
    void activate();
    void deactivate();
    bool isActive() { return _active; }
    
    // Pokazanie splash screen "SD PLAYER"
    void showSplash();
    
    // Kontrola pilota
    void onRemoteUp();
    void onRemoteDown();
    void onRemoteOK();
    void onRemoteSRC();
    void onRemoteVolUp();
    void onRemoteVolDown();
    
    // Kontrola enkodera
    void onEncoderLeft();
    void onEncoderRight();
    void onEncoderButton();
    
    // Style wyświetlania
    enum DisplayStyle {
        STYLE_1 = 1,  // Lista z paskiem na górze
        STYLE_2 = 2,  // Duży tekst utworu
        STYLE_3 = 3,  // VU meter + utwór
        STYLE_4 = 4,  // Spektrum częstotliwości
        STYLE_5 = 5,  // Minimalistyczny
        STYLE_6 = 6,  // Album art simulation
        STYLE_10 = 10 // Pełny ekran z animacją
    };
    
    void setStyle(DisplayStyle style);
    DisplayStyle getStyle() { return _style; }
    
    // Style informacji na górnym pasku
    enum InfoStyle {
        INFO_CLOCK_DATE = 0,  // Zegar + Data
        INFO_TRACK_TITLE = 1  // Tytuł utworu
    };
    
    void nextInfoStyle();  // Przełączanie przyciskiem SRC
    
private:
    U8G2& _display;
    SDPlayerWebUI* _player;
    bool _active;
    
    // Style i tryby
    DisplayStyle _style;
    InfoStyle _infoStyle;
    enum Mode {
        MODE_NORMAL,    // Normalny panel z listą
        MODE_VOLUME,    // Pokazuje Volume
        MODE_SPLASH     // Splash screen "SD PLAYER"
    };
    Mode _mode;
    
    // Lista utworów
    struct FileEntry {
        String name;
        bool isDir;
    };
    std::vector<FileEntry> _fileList;
    int _selectedIndex;
    int _scrollOffset;
    
    // Timery
    unsigned long _splashStartTime;
    unsigned long _volumeShowTime;
    unsigned long _lastUpdate;
    
    // Animacje
    int _scrollPosition;
    int _animFrame;
    
    // Odświeżanie listy plików
    void refreshFileList();
    
    // Renderowanie
    void render();
    void renderSplash();
    void renderVolume();
    void renderStyle1();  // Lista z paskiem
    void renderStyle2();  // Duży tekst
    void renderStyle3();  // VU meter
    void renderStyle4();  // Spektrum
    void renderStyle5();  // Minimal
    void renderStyle6();  // Album art
    void renderStyle10(); // Full screen animated
    
    // Pomocnicze
    void drawTopBar();
    void drawFileList();
    void drawVolumeIcon(int x, int y);
    void drawScrollBar(int itemCount, int visibleCount);
    String truncateString(const String& str, int maxWidth);
    
    // Nawigacja
    void scrollUp();
    void scrollDown();
    void selectCurrent();
    void nextStyle();
};
