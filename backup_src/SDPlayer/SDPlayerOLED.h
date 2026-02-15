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
    void onRemotePlayPause();  // Play/Pause - dodatkowy przycisk lub OK podczas odtwarzania
    void onRemoteStop();       // Stop - całkowite zatrzymanie odtwarzania
    
    // Kontrola enkodera
    void onEncoderLeft();
    void onEncoderRight();
    void onEncoderButton();
    void onEncoderButtonHold(unsigned long holdTime); // Długie przytrzymanie
    bool checkEncoderLongPress(bool buttonState);     // Sprawdza długie przytrzymanie
    
    // Style wyświetlania
    enum DisplayStyle {
        STYLE_1 = 1,  // Lista z paskiem na górze
        STYLE_2 = 2,  // Duży tekst utworu
        STYLE_3 = 3,  // VU meter + utwór
        STYLE_4 = 4,  // Spektrum częstotliwości
        STYLE_5 = 5,  // Minimalistyczny
        STYLE_6 = 6,  // Album art simulation
        STYLE_7 = 7,  // Analizator retro z trójkątnymi słupkami
        STYLE_10 = 10 // Pełny ekran z animacją
    };
    
    void setStyle(DisplayStyle style);
    DisplayStyle getStyle() { return _style; }
    void nextStyle();  // Przełączanie do następnego stylu
    
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
    unsigned long _lastSrcPressTime;  // Dla podwójnego kliknięcia SRC
    uint8_t _srcClickCount;           // Licznik kliknięć SRC
    
    // Obsługa enkodera - potrójne kliknięcie i długie przytrzymanie
    unsigned long _lastEncoderClickTime;
    uint8_t _encoderClickCount;
    unsigned long _encoderButtonPressStart;
    bool _encoderButtonPressed;
    
    // Animacje
    int _scrollPosition;
    int _animFrame;
    int _scrollTextOffset;        // Offset scrollowania tekstu w liście
    unsigned long _lastScrollTime; // Timer scrollowania tekstu
    
    // Komunikaty akcji (pokazywane na 2 sekundy)
    String _actionMessage;         // Tekst komunikatu (np. "PLAY", "PAUSE", "EXIT")
    unsigned long _actionMessageTime; // Kiedy pokazano komunikat
    bool _showActionMessage;       // Czy pokazywać komunikat
    
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
    void renderStyle7();  // Analizator retro
    void renderStyle10(); // Full screen animated
    
    // Pomocnicze
    void drawTopBar();
    void drawFileList();
    void drawVolumeIcon(int x, int y);
    void drawScrollBar(int itemCount, int visibleCount);
    String truncateString(const String& str, int maxWidth);
    void showActionMessage(const String& message);  // Pokazuje komunikat na 2 sek
    void drawControlIcons();  // Rysuje ikonki przycisków
    void drawIconPrev(int x, int y);    // ⏮️ Previous
    void drawIconUp(int x, int y);      // ⬆️ Up
    void drawIconPause(int x, int y);   // ⏸️ Pause
    void drawIconPlay(int x, int y);    // ▶️ Play
    void drawIconStop(int x, int y);    // ⏹️ Stop
    void drawIconNext(int x, int y);    // ⏭️ Next
    void drawIconDown(int x, int y);    // ⬇️ Down
    
    // Nawigacja
    void scrollUp();
    void scrollDown();
    void selectCurrent();
};
