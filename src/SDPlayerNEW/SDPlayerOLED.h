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
    void onRemoteBack();       // BACK - wyjście do katalogu nadrzędnego
    void onRemoteSRC();
    void onRemoteVolUp();
    void onRemoteVolDown();
    void onRemotePlayPause();  // Play/Pause - dodatkowy przycisk lub OK podczas odtwarzania
    void onRemoteStop();       // Stop - całkowite zatrzymanie odtwarzania
    
    // Kontrola enkodera
    void onEncoderLeft();
    void onEncoderRight();
    void onEncoderButton();
    void onEncoderButtonMediumHold(unsigned long holdTime); // Średnie przytrzymanie 3-5s (select track)
    void onEncoderButtonLongHold(unsigned long holdTime);   // Długie przytrzymanie >6s (tryb volume)
    void onEncoderButtonHold(unsigned long holdTime);       // Stara metoda - kompatybilność
    bool checkEncoderLongPress(bool buttonState);           // Sprawdza długie przytrzymanie
    
    // Style wyświetlania
    enum DisplayStyle {
        STYLE_1 = 1,  // Lista z paskiem na górze
        STYLE_2 = 2,  // Duży tekst utworu
        STYLE_3 = 3,  // VU meter + utwór
        STYLE_4 = 4,  // Spektrum częstotliwości
        STYLE_5 = 5,  // Minimalistyczny
        STYLE_6 = 6,  // Album art simulation
        STYLE_7 = 7,  // Analizator retro z trójkątnymi słupkami
        STYLE_8 = 8,  // Layout jak SDPlayerAdvanced
        STYLE_9 = 9,  // VU meters pod scrollem
        STYLE_10 = 10, // Pełny ekran z animacją
        STYLE_11 = 11, // Styl bazujący na Radio Mode 0 - podstawowy
        STYLE_12 = 12, // Styl bazujący na Radio Mode 1 - duży zegar
        STYLE_13 = 13, // Styl bazujący na Radio Mode 2 - 3 linijki tekstu
        STYLE_14 = 14  // Styl bazujący na Radio Mode 3 - linia statusu
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
    void setSelectedIndex(int index);  // Ustawia _selectedIndex (synchronizacja z WebUI)
    int getSelectedIndex() { return _selectedIndex; }  // Zwraca aktualny indeks
    
    // Synchronizacja listy utworów z IR pilot (trackFiles[] z main.cpp)
    void syncTrackListFromIR(const String* trackFiles, int trackCount);
    
    // Synchronizacja list plików z WebUI (aby uniknąć lazy loading przy każdym scroll)
    void syncFileListFromWebUI(const std::vector<std::pair<String, bool>>& webUIFileList, const String& currentDir);
    
    // Nawigacja folderowa
    void goUp();  // Wyjście do katalogu nadrzędnego (parent directory)
    
    // Synchronizacja z WebUI
    typedef std::function<void(int)> OLEDIndexChangeCallback;
    void setIndexChangeCallback(OLEDIndexChangeCallback callback) { _oledIndexChangeCallback = callback; }
    void notifyWebUIIndexChange(int newIndex);  // Powiadamia WebUI o zmianie indeksu z pilota
    
    // Communicates and messages
    void showActionMessage(const String& message);  // Pokazuje komunikat na 2 sek
    
    // Nawigacja (publiczne dla WebUI)
    void scrollUp();
    void scrollDown(); 
    void selectCurrent();

private:
    U8G2& _display;
    SDPlayerWebUI* _player;
    bool _active;
    
    // Callback dla synchronizacji z WebUI
    OLEDIndexChangeCallback _oledIndexChangeCallback;
    
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

    // --- UTF-8 → polskie znaki dla czcionek 8-bit (rozwiązanie z SDPlayerAdvanced)
    static void processPolishText(String& text);
    void drawStrPL(int x, int y, const String& text);
    bool _encoderButtonPressed;
    
    // Tryb enkodera - kontrola głośności vs nawigacja po liście
    bool _encoderVolumeMode;           // true = kontrola głośności, false = nawigacja po liście
    unsigned long _lastEncoderModeChange; // Timer dla automatycznego powrotu do domyślnego trybu
    
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
    void renderStyle8();  // Layout jak SDPlayerAdvanced
    void renderStyle9();  // VU meters pod scrollem
    void renderStyle10(); // Full screen animated
    void renderStyle11(); // Radio Mode 0 - podstawowy
    void renderStyle12(); // Radio Mode 1 - duży zegar
    void renderStyle13(); // Radio Mode 2 - 3 linijki
    void renderStyle14(); // Radio Mode 3 - linia statusu
    
    // Pomocnicze
    void drawTopBar();
    void drawFileList();
    void drawVolumeIcon(int x, int y);
    void drawVolumeMuteSlash(int x, int y);
    void drawMuteOverlayTag();
    bool isMutedState() const;
    void drawScrollBar(int itemCount, int visibleCount);
    String truncateString(const String& str, int maxWidth);
    void drawControlIcons();  // Rysuje ikonki przycisków
    void drawIconPrev(int x, int y);    // ⏮️ Previous
    void drawIconUp(int x, int y);      // ⬆️ Up
    void drawIconPause(int x, int y);   // ⏸️ Pause
    void drawIconPlay(int x, int y);    // ▶️ Play
    void drawIconStop(int x, int y);    // ⏹️ Stop
    void drawIconNext(int x, int y);    // ⏭️ Next
    void drawIconDown(int x, int y);    // ⬇️ Down
    int getTrackNumber(const String& currentFileName);  // Oblicza numer utworu pomijając foldery
    int getTotalTracks();  // Zlicza wszystkie pliki muzyczne w katalogu
    
};
