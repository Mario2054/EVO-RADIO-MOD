#pragma once
#include <Arduino.h>
#include <SD.h>
#include <vector>

// Forward declarations
class Audio;
class U8G2;
class SDPlayerOLED;

/**
 * SDPlayerAdvanced - Zaawansowana obsługa SD Player
 * 
 * Funkcje rozszerzone:
 * ✅ Automatyczne przejście do kolejnego utworu (auto-play)
 * ✅ Inteligentne sortowanie numerów w nazwach plików (Track 1, Track 2...)
 * ✅ Obsługa tagów ID3 (Artist, Title, Album)
 * ✅ Wyświetlanie parametrów audio (Bitrate, SampleRate, BitsPerSample)
 * ✅ Timer odtwarzania (mm:ss)
 * ✅ Polskie znaki diakrytyczne
 * ✅ Zapisywanie/wczytywanie ostatniej pozycji z SD
 * ✅ Inteligentne wykrywanie końca utworu (EOF)
 * ✅ Rozszerzone formaty audio (.mp3, .wav, .flac, .aac, .m4a, .ogg, .wma, .aiff, .alac)
 */

class SDPlayerAdvanced {
public:
    // Informacje o utworze
    struct TrackInfo {
        String artist;
        String title;
        String album;
        String filename;
        String folder;
        int bitrate;           // w kbps
        int sampleRate;        // w Hz
        int bitsPerSample;     // 16, 24, 32
        bool hasID3;           // Czy ma tagi ID3
        String codecType;      // MP3, FLAC, AAC, VORBIS
        TrackInfo() : bitrate(0), sampleRate(0), bitsPerSample(0), hasID3(false) {}
    };
    
    // Element listy plików
    struct FileItem {
        String name;
        String fullPath;
        bool isDir;
        long size;             // rozmiar w bajtach
        FileItem() : isDir(false), size(0) {}
    };
    
    // Tryb sortowania
    enum SortMode {
        SORT_NAME_ASC,         // Alfabetycznie A-Z
        SORT_NAME_DESC,        // Alfabetycznie Z-A
        SORT_NAME_NUMERIC,     // Inteligentnie (Track 1, Track 2...)
        SORT_SIZE_ASC,         // Rozmiar rosnąco
        SORT_SIZE_DESC,        // Rozmiar malejąco
        SORT_DATE_ASC,         // Data (najstarsze pierwsze)
        SORT_DATE_DESC         // Data (najnowsze pierwsze)
    };
    
    // Tryb powtarzania
    enum RepeatMode {
        REPEAT_OFF,            // Bez powtarzania
        REPEAT_ONE,            // Powtarzaj jeden utwór
        REPEAT_ALL,            // Powtarzaj wszystkie
        REPEAT_FOLDER          // Powtarzaj folder
    };
    
    SDPlayerAdvanced();
    ~SDPlayerAdvanced();
    
    // Inicjalizacja
    void begin(Audio* audio, U8G2* display = nullptr);
    void setOLED(SDPlayerOLED* oled);
    
    // Główna pętla
    void loop();
    
    // Aktywacja/deaktywacja
    void activate();
    void deactivate();
    bool isActive() { return _active; }
    
    // Nawigacja po katalogach
    bool changeDirectory(const String& path);
    bool enterFolder(int index);
    bool goBack();  // Powrót do katalogu nadrzędnego
    String getCurrentDirectory() { return _currentDir; }
    
    // Kontrola odtwarzania
    void play(int index);
    void playFile(const String& path);
    void pause();
    void resume();
    void stop();
    void next();
    void previous();
    void togglePause();
    
    // Gettery statusu
    bool isPlaying() { return _isPlaying; }
    bool isPaused() { return _isPaused; }
    int getSelectedIndex() { return _selectedIndex; }
    const std::vector<FileItem>& getFileList() { return _fileList; }
    int getFileCount() { return _fileList.size(); }
    
    // Informacje o utworze
    const TrackInfo& getTrackInfo() { return _currentTrackInfo; }
    String getPlayTimeString();  // Format: "05:23"
    unsigned int getPlaySeconds() { return _playSeconds; }
    
    // Ustawienia
    void setRepeatMode(RepeatMode mode) { _repeatMode = mode; }
    RepeatMode getRepeatMode() { return _repeatMode; }
    void setSortMode(SortMode mode);
    SortMode getSortMode() { return _sortMode; }
    void setAutoPlay(bool enable) { _autoPlay = enable; }
    bool getAutoPlay() { return _autoPlay; }
    
    // Zapisywanie/wczytywanie ustawień
    void saveSettings();
    void loadSettings();
    
    // Callbacks od Audio (wywoływane z main.cpp przez audio_xxx funkcje)
    void onAudioInfo(const char* info);
    void onAudioID3(const char* info);
    void onAudioBitrate(const char* info);
    void onAudioEOF(const char* info);
    
    // Rendering OLED (dedykowany styl dla Advanced mode)
    void renderOLED();  // Główna metoda renderowania
    
    // Pomocnicze
    static bool isAudioFile(const String& filename);
    static String getFileExtension(const String& filename);
    static String formatFileSize(long bytes);  // Zwraca "2.5 MB"
    static void processPolishText(String& text);  // Konwersja polskich znaków
    
private:
    // Wskaźniki
    Audio* _audio;
    U8G2* _display;
    SDPlayerOLED* _oled;
    
    // Status
    bool _active;
    bool _isPlaying;
    bool _isPaused;
    
    // Nawigacja
    String _currentDir;
    std::vector<FileItem> _fileList;
    int _selectedIndex;
    
    // Aktualny utwór
    String _currentFile;
    TrackInfo _currentTrackInfo;
    
    // Timer odtwarzania
    unsigned long _playStartTime;
    unsigned int _playSeconds;
    
    // Ustawienia
    SortMode _sortMode;
    RepeatMode _repeatMode;
    bool _autoPlay;  // Automatyczne przechodzenie do kolejnego
    
    // Flagi wewnętrzne
    bool _trackEndDetected;
    bool _bitrateReceived;
    
    // Metody prywatne
    void scanCurrentDirectory();
    void sortFileList();
    void clearTrackInfo();
    void updatePlayTimer();
    void handleTrackEnd();
    
    // Sortowanie - funkcje pomocnicze
    static int compareStringsWithNumbers(const String& a, const String& b);
    static int extractNumber(const String& str, int& startPos);
    
    // Parsowanie Info/ID3
    void parseAudioInfo(const String& info);
    void parseID3Data(const String& info);
};
