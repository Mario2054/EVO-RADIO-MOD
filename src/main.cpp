// ################################################################################################################### //
// Evo Internet Radio                                                                                                  //
// ################################################################################################################### //
// Robgold 2026, Made in Poland                                                                                        //
// Source -> https://github.com/dzikakuna/ESP32_radio_evo3/tree/main/src/ESP32_radio_evo3.19                           //
// ################################################################################################################### //
//                                                                                                                     //
// Compilator: Arduino, Platformio                                                                                     //
// ESP32 by Espressif:                          3.3.3 (works on newer but without modified libs for FLAC files)        //
// ESP32-audioI2S-master by schreibfaul1:       3.4.4g with commits 22.01.2026, FLAC 24bit is working!                 //
// ESP Async WebServer:                         3.9.1                                                                  //
// Async TCP:                                   3.4.9                                                                  // 
// ezButton:                                    1.0.6                                                                  //
// U8g2 by Oliver Olikraus:                     2.35.30                                                                //
// WiFiManager by tzapu                         2.0.17                                                                 //
// ################################################################################################################### //

#include "Arduino.h"           // Standardowy nagłówek Arduino, który dostarcza podstawowe funkcje i definicje
#include "core/options.h"      // Flagi funkcji (USE_DLNA, ENABLE_EQ16, etc.)
#include <WiFiManager.h>       // Biblioteka do zarządzania konfiguracją sieci WiFi, opis jak ustawić połączenie WiFi przy pierwszym uruchomieniu jest opisany tu: https://github.com/tzapu/WiFiManager
#include "Audio.h"             // ESP32-audioI2S 3.4.4 - Stabilna wersja audio z pełnym wsparciem dla ESP32/ESP32-S3
#include "SPI.h"               // Biblioteka do obsługi komunikacji SPI
#include "FS.h"                // Biblioteka do obsługi systemu plików
#include <U8g2lib.h>           // Biblioteka do obsługi wyświetlaczy
#include <ezButton.h>          // Biblioteka do obsługi enkodera z przyciskiem
#include <HTTPClient.h>        // Biblioteka do wykonywania żądań HTTP, umożliwia komunikację z serwerami przez protokół HTTP
#include <Ticker.h>            // Mechanizm tickera do odświeżania timera 1s, pomocny do cyklicznych akcji w pętli głównej
#include <EEPROM.h>            // Bilbioteka obsługi emulacji pamięci EEPROM
#include <Time.h>              // Biblioteka do obsługi funkcji związanych z czasem, np. odczytu daty i godziny
#include <ESPAsyncWebServer.h> // Bliblioteka asyncrhionicznego serwera web
#include <AsyncTCP.h>          // Bliblioteka TCP dla serwera web
#include <Update.h>            // Blibioteka dla aktulizacji OTA
#include <ESPmDNS.h>           // Blibioteka mDNS dla ESP

#include "soc/rtc_cntl_reg.h"   // Biblioteki ESP aby móc zrobic pełny reset 
#include "soc/rtc_cntl_struct.h"// Biblioteki ESP aby móc zrobic pełny reset 
#include <esp_sleep.h>
#include "esp_system.h"

#include "rom/gpio.h"     // Biblioteka do mapowania pinu CS_SD

#include "LittleFS.h"
#include "SD.h"

// =============== INTEGRACJA MODUŁÓW ===============
// SDPlayer - odtwarzacz plików z karty SD
#include "SDPlayer/SDPlayerOLED.h"
#include "SDPlayer/SDPlayerWebUI.h"

// SDRecorder - nagrywanie strumienia radiowego
#include "SDRecorder/SDRecorder.h"

// Analyzer - analizator spektrum FFT
#include "EQ_FFTAnalyzer.h"
#include "EQ_AnalyzerDisplay.h"

// Equalizer 16-band - graficzny equalizer 16-pasmowy
#include "APMS_GraphicEQ16.h"

// Bluetooth - moduł BT UART
#include "bt/BTWebUI.h"

// DLNA - przeglądarka i odtwarzacz DLNA
#ifdef USE_DLNA
#include "network/DLNAWebUI.h"
#include "network/dlna_service.h"
#include "network/dlna_worker.h"
#include "network/dlna_http_guard.h"
#endif

// WiFi Animation - animacja gwiazdek podczas łączenia
#include "wifi_animation.h"

// VU Style 11 - analog VU meter
#include "vu_style11.h"
// ==================================================

// --------------- DEFINICJA WERSJI RADIA i NAZWT HOSTA ---------------
#define softwareRev "v3.20.04"  // Wersja oprogramowania radia
#define hostname "evoradio"   // Definicja nazwy hosta widoczna na zewnątrz

// --------------- CZYTNIK KART SD (Opcjonalny) ---------------
#define SD_CS 47    // Pin CS (Chip Select) dla karty SD wybierany jako interfejs SPI
#define SD_SCLK 45  // Pin SCK (Serial Clock) dla karty SD
#define SD_MISO 21  // Pin MISO (Master In Slave Out) dla karty SD
#define SD_MOSI 48  // pin MOSI (Master Out Slave In) dla karty SD


//--------------- KONFIGURACJA ---------------
//Tutaj włączamy karte SD / pamiec SPIFSS. LittleFS, drugi enkoder, rozjanianie logo, animacje power off

#define USE_SD  //odkomentuj dla karty SD , zostawiamy w przypadku AUTOSTORAGE
//#define twoEncoders // odkomentuj dla drugiego enkodera
// AUTOSTORAGE jest teraz zdefiniowane w platformio.ini build_flags (-DAUTOSTORAGE)
const bool f_logoSlowBrightness = 1; 



// ------ AUTO WYKRYWANIE STORAGE - Karta SD / LittleFS -----
#ifdef AUTOSTORAGE
  #define STORAGE (*_storage)
  bool initStorage();
  bool useSD = false;
  const char* storageTextName = nullptr;
  fs::FS* _storage = nullptr;
#else

  #ifdef USE_SD
    #define STORAGE SD
    #define STORAGE_BEGIN() SD.begin(SD_CS, customSPI)
    const bool useSD = true;
    //#define SD_LED 17          // LED - sygnalizacja CS, aktywność karty SD, klon pinu uzyteczny w przypadku uzywania tylnego czytnika SD na PCB
    //static const char storageTextName[] = "SD";
    #define storageTextName "SD"
  #else
    #define STORAGE LittleFS
    #define STORAGE_BEGIN() LittleFS.begin(true)  // Dla LittleFS, stabilniej działa i nie przerywa audio
    const bool useSD = false;
    #define storageTextName "LittleFS"
  #endif

#endif
 
// --------------- OLED Wyswietlacz - definicja pinow ---------------
#define SPI_MOSI_OLED 39  // Pin MOSI (Master Out Slave In) dla interfejsu SPI OLED
#define SPI_MISO_OLED 0   // Pin MISO (Master In Slave Out) brak dla wyswietlacza OLED
#define SPI_SCK_OLED 38   // Pin SCK (Serial Clock) dla interfejsu SPI OLED
#define CS_OLED 42        // Pin CS (Chip Select) dla interfejsu OLED
#define DC_OLED 40        // Pin DC (Data/Command) dla interfejsu OLED
#define RESET_OLED 41     //-1 //41     // Pin Reset dla interfejsu OLED

// Rozmiar wysweitlacz OLED (potrzebny do wyświetlania grafiki)
#define SCREEN_WIDTH 256        // Szerokość ekranu w pikselach
#define SCREEN_HEIGHT 64        // Wysokość ekranu w pikselach

// --------------- TAS5805 / PCM5102A - Wzmacniacz z DAC ---------------
// Odkomentuj poniższą linię aby włączyć obsługę TAS5805 zamiast PCM5102A
//#define TAS5805       // Kompilacja dla wersji ze Wzmacniacz TAS5805 z wbudowanym DAC

#ifdef TAS5805 // PowerAmp with I2S & DSP                 
  #include <Wire.h>           // I2C dla komunikacji z TI TAS5805
  #define I2C_CLK 9           // GPIO CLOCK I2C
  #define I2C_DATA 8          // GPIO DATA I2C
  #define TAS5805_ADDR 0x2D   // Adres I2C TAS5805, pull-up na ADR/#FAULT 10k podbicie adresu o 1
  
  // --------------- TAS5805 - definicja pinow AMPa z przetwornikiem ---------------
  #define I2S_DOUT 16        // Podłączenie do pinu DIN na DAC TAS
  #define I2S_BCLK 14        // Podłączenie po pinu BCK na DAC TAS
  #define I2S_LRC 15         // Podłączenie do pinu LCK na DAC TAS 
#else // PCM5102A - simple DAC
  // --------------- PCM5102A - definicja pinow przetwornika ---------------
  #define I2S_DOUT 13             // Podłączenie do pinu DIN na DAC
  #define I2S_BCLK 12             // Podłączenie po pinu BCK na DAC
  #define I2S_LRC 14              // Podłączenie do pinu LCK na DAC  
#endif

// --------------- ENCODER 2 (MAIN) ---------------
// Jesli jedyny to Volume/Stacje/Banki/PowerOff/PowerOn jesli z Enkoderem1 to Stacje/Banki
#define CLK_PIN2 10             // Podłączenie z pinu 10 do CLK na enkoderze
#define DT_PIN2 11              // Podłączenie z pinu 11 do DT na enkoderze lewym
#define SW_PIN2 1               // Podłączenie z pinu 1 do SW na enkoderze lewym (przycisk)

// --------------- ENCODER 1 (dodatkowy) ---------------
// Jesli zdefinionway to Volume/Mute/PowerOff/PowerOn
#ifdef twoEncoders
  #define CLK_PIN1 6              // Podłączenie z pinu 6 do CLK na enkoderze prawym
  #define DT_PIN1 5               // Podłączenie z pinu 5 do DT na enkoderze prawym
  #define SW_PIN1 4               // Podłączenie z pinu 4 do SW na enkoderze prawym (przycisk)
  const bool use2encoders = true;
  // Zmienne enkodera 1
  int CLK_state1 = 0;
  int prev_CLK_state1 = 0;
  int buttonLongPressTime1 = 150;
  int buttonSuperLongPressTime1 = 2000;
  bool action4Taken = false;        // Flaga Akcji 4 - enkoder 1, powerOFF
  bool action5Taken = false;        // Flaga Akcji 5 - enkoder 1, Menu Bank
#else
  const bool use2encoders = false;
#endif

// --------------- IR odbiornik podczerwieni ---------------
#define recv_pin 15

// --------------- PRZYCISK POWER i LEDow ---------------
#define SW_POWER 8        // Dedykowany przycisk Power
#define STANDBY_LED 17    // LED do informowania o trybie Standby
#define IR_LED 17         // Derfinicja pinu LED pokazujacego aktywność odbiornika IR
#define SPEAKERS_PIN 18   // Pin Enable wzmacniacza (PAM8610/TPA3110 itp.) - HIGH=głośniki ON, LOW=OFF
//


#define WAKEUP_INTERVAL_US (60ULL * 1000000ULL) // 1 minuta

// definicja dlugosci ilosci stacji w banku, dlugosci nazwy stacji w PSRAM/EEPROM, maksymalnej ilosci plikow audio (odtwarzacz)
#define MAX_STATIONS 99          // Maksymalna liczba stacji radiowych, które mogą być przechowywane w jednym banku
#define STATION_NAME_LENGTH 220  // Nazwa stacji wraz z bankiem i numerem stacji do wyświetlenia w pierwszej linii na ekranie
#define MAX_FILES 100            // Maksymalna liczba plików lub katalogów w tablicy directoriesz
#define bank_nr_max 17           // Numer jaki może osiągnac maksymalnie zmienna bank_nr czyli ilość banków
#define displayModeMax 6          // Ogrniczenie maksymalnej ilosci trybów wyswietlacza OLED

// DEBUG PRINTS - ON/OFF
#define f_debug_web_on 0         // Flaga właczenia wydruku debug_web
#define f_debug_on 0             // Flaga właczenia wydruku debug

//String STATIONS_URL1 = "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank01.txt";   // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL1  "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank01.txt"   // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL2  "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank02.txt"   // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL3  "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank03.txt"   // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL4  "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank04.txt"   // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL5  "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank05.txt"   // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL6  "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank06.txt"   // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL7  "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank07.txt"   // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL8  "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank08.txt"   // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL9  "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank09.txt"   // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL10 "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank10.txt"  // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL11 "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank11.txt"  // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL12 "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank12.txt"  // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL13 "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank13.txt"  // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL14 "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank14.txt"  // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL15 "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank15.txt"  // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL16 "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank16.txt"  // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL17 "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank17.txt"  // Adres URL do pliku z listą stacji radiowych
#define STATIONS_URL18 "https://raw.githubusercontent.com/dzikakuna/ESP32_radio_streams/main/bank18.txt"  // Adres URL do pliku z listą stacji radiowych


// --------------- DEFINICJA TEKSTOW ---------------
#define SLEEP_STRING "SLEEP "
#define SLEEP_STRING_VAL " MINUTES"
#define SLEEP_STRING_OFF "OFF"


// --------------- KOMUNIKACJA MODUŁU RADIA PO RS232 ---------------
//#define SERIALCOM        // Dekalarcja kompilacji plikow integracji jako moduł z komunikacja po RS232
#ifdef SERIALCOM
#include "serialcom.h"   // Integracja jako moduł z komunikacja po RS232
#endif


// --------------- Zmienne  DLA PILOTa IR w standardzie NEC - przeniesiona do pliku txt ---------------
uint16_t rcCmdVolumeUp = 0;   // Głośność +
uint16_t rcCmdVolumeDown = 0; // Głośność -
uint16_t rcCmdArrowRight = 0; // Radio: nastepna stacja | Menu Bank: bank+1 | Equalizer: parametr+
uint16_t rcCmdArrowLeft = 0;  // Radio: poprzednia stacja | Menu Bank: bank-1 | Equalizer: parametr-
uint16_t rcCmdArrowUp = 0;    // Radio/SDPlayer: lista stacji/utworów w górę | Equalizer: poprzedni parametr
uint16_t rcCmdArrowDown = 0;  // Radio/SDPlayer: lista stacji/utworów w dół | Equalizer: następny parametr
uint16_t rcCmdBack = 0;	      // Powrót do głównego ekranu | Zatrzymuje nagrywanie jeśli aktywne
uint16_t rcCmdOk = 0;         // Zatwierdzenie wyboru (stacja, bank, equalizer, URL)
uint16_t rcCmdSrc = 0;        // Zmiana trybu wyświetlacza (0-10, pomija 9) - style 5,7 aktywne (nowe VU) z auto-pomijaniem wyłączonych stylów
uint16_t rcCmdMute = 0;       // Wyciszenie/włączenie dźwięku (toggle)
uint16_t rcCmdAud = 0;        // Equalizer 3-band/16-band | Podwójne kliknięcie (600ms): aktywacja SDPlayer
uint16_t rcCmdDirect = 0;     // Radio: toggle nagrywania | Menu Bank: GitHub/SD | Equalizer: reset | Display Mode 4: audio buffer debug | Inne: dimmer OLED
uint16_t rcCmdBankMinus = 0;  // Otwiera menu wyboru banku | W menu: bank-1 (z zawijaniem)
uint16_t rcCmdBankPlus = 0;   // Otwiera menu wyboru banku | W menu: bank+1 (z zawijaniem)
uint16_t rcCmdRed = 0;        // Power OFF (z animacją jeśli włączona)
uint16_t rcCmdGreen = 0;      // W liście stacji: odtwórz głosowy czas | Poza listą: sleep timer (0-90min, krok 15min)
uint16_t rcCmdKey0 = 0;       // W Equalizer: toggle EQ3 ↔ EQ16 | Radio/SDPlayer: wybór stacji/utworu (wielocyfrowe, timeout 3s)
uint16_t rcCmdKey1 = 0;       // Radio/SDPlayer: wybór stacji/utworu nr 1 lub pierwsza cyfra liczby 1x (timeout 3s)
uint16_t rcCmdKey2 = 0;       // Radio/SDPlayer: wybór stacji/utworu nr 2 lub pierwsza cyfra liczby 2x (timeout 3s)
uint16_t rcCmdKey3 = 0;       // Radio/SDPlayer: wybór stacji/utworu nr 3 lub pierwsza cyfra liczby 3x (timeout 3s)
uint16_t rcCmdKey4 = 0;       // Radio/SDPlayer: wybór stacji/utworu nr 4 lub pierwsza cyfra liczby 4x (timeout 3s)
uint16_t rcCmdKey5 = 0;       // Radio/SDPlayer: wybór stacji/utworu nr 5 lub pierwsza cyfra liczby 5x (timeout 3s)
uint16_t rcCmdKey6 = 0;       // Radio/SDPlayer: wybór stacji/utworu nr 6 lub pierwsza cyfra liczby 6x (timeout 3s)
uint16_t rcCmdKey7 = 0;       // Radio/SDPlayer: wybór stacji/utworu nr 7 lub pierwsza cyfra liczby 7x (timeout 3s)
uint16_t rcCmdKey8 = 0;       // Radio/SDPlayer: wybór stacji/utworu nr 8 lub pierwsza cyfra liczby 8x (timeout 3s)
uint16_t rcCmdKey9 = 0;       // Radio/SDPlayer: wybór stacji/utworu nr 9 lub pierwsza cyfra liczby 9x (timeout 3s)
uint16_t rcCmdPower = 0;      // Power OFF (działa także w SDPlayer, z animacją jeśli włączona)
uint16_t rcCmdYellow = 0;     // Toggle Analyzer ON/OFF (analizator spektrum)
uint16_t rcCmdBlue = 0;       // Nagrywanie strumienia radiowego toggle START/STOP (alias dla REC)
uint16_t rcCmdBT = 0;         // W trybie Radio: otwiera stronę BT (IP/bt) | W SDPlayer: zmiana stylu (1-14)
uint16_t rcCmdPLAY = 0;       // SDPlayer: uruchomienie odtwarzania (PLAY) | Radio: blokowany
uint16_t rcCmdSTOP = 0;       // SDPlayer: zatrzymanie odtwarzania (STOP) | Radio: blokowany
uint16_t rcCmdREV = 0;        // SDPlayer: przewiń do tyłu -10s | Podwójne kliknięcie (600ms): poprzedni utwór
uint16_t rcCmdRER = 0;        // SDPlayer: przewiń do przodu +10s | Podwójne kliknięcie (600ms): następny utwór
uint16_t rcCmdTV = 0;         // Zarezerwowane na przyszłe funkcje (aktualnie nieaktywny)
uint16_t rcCmdRADIO = 0;      // SDPlayer: wyjście z SDPlayer i powrót do trybu Radio (przywraca bank i stację)
uint16_t rcCmdFILES = 0;      // SDPlayer: wyświetlenie listy plików/folderów na karcie SD (katalog /music/)
uint16_t rcCmdWWW = 0;        // Zarezerwowane na przyszłe funkcje (aktualnie nieaktywny)
uint16_t rcCmdGAZE = 0;       // Zarezerwowane na przyszłe funkcje (aktualnie nieaktywny)
uint16_t rcCmdEPG = 0;        // Zarezerwowane na przyszłe funkcje (aktualnie nieaktywny)
uint16_t rcCmdMENU = 0;       // SDPlayer: wyświetlenie menu/listy plików (alias dla FILES) | Radio: blokowany
uint16_t rcCmdEXIT = 0;       // SDPlayer: wyjście i powrót do radia (przywraca ostatnią stację)
uint16_t rcCmdREC = 0;        // Nagrywanie strumienia radiowego toggle START/STOP (plik MP3 na SD: /recordings/)
uint16_t rcCmdKOL = 0;        // Moduł BT: toggle ON/OFF (włącz/wyłącz moduł Bluetooth UART)
uint16_t rcCmdCHAT = 0;       // Moduł BT: wyświetl status na OLED (włączony/wyłączony + IP/bt)
uint16_t rcCmdPAUSE = 0;      // SDPlayer: pauza/wznowienie odtwarzania (alias PLAY) | Radio: blokowany
uint16_t rcCmdREDLEFT = 0;    // Moduł BT: restart modułu (eksperymentalne, obecnie placeholder)
uint16_t rcCmdGREENL = 0;     // Moduł BT: test połączenia (eksperymentalne, wyświetla status modułu)
uint16_t rcCmdCHPlus = 0;     // Zwiększ jasność OLED o 20 (zakres 10-255, wyświetla wartość na OLED)
uint16_t rcCmdCHMinus = 0;    // Zmniejsz jasność OLED o 20 (zakres 10-255, wyświetla wartość na OLED)
uint16_t rcCmdINFO = 0;       // Moduł BT: wyświetl informacje o module (status, config path /bt)
uint16_t rcCmdPOWROT = 0;     // Moduł BT: otwiera stronę BT w przeglądarce (jak GLOS, IP/bt)
uint16_t rcCmdHELP = 0;       // Toggle SDPlayer ON/OFF (aktywacja/dezaktywacja odtwarzacza z karty SD)
uint16_t rcCmdPIP = 0;        // Przełączenie Equalizer EQ3 ↔ EQ16 (3-pasmowy ↔ 16-pasmowy)
uint16_t rcCmdMINIMA = 0;     // SDPlayer: następny styl wyświetlania (1→2→3...→14→1, pomija wyłączone)
uint16_t rcCmdMAXIMA = 0;     // SDPlayer: zmiana stylu wyświetlania (obecnie także nextStyle)
uint16_t rcCmdWELEMENT = 0;   // SDPlayer: PAUSE/PLAY toggle (wstrzymaj/wznów odtwarzanie)
uint16_t rcCmdWINPOD = 0;     // SDPlayer: STOP (zatrzymuje odtwarzanie, resetuje pozycję)
uint16_t rcCmdGLOS = 0;       // Moduł BT: otwiera stronę BT w przeglądarce (IP/bt, wyświetla URL na OLED)
uint16_t lastIrCode = 0;      // Ostatni kod IR (używane dla combo double-click REV/RER)

// Double-click variables for seek/track change
bool waitingForSecondClick = false;    // Flaga oczekiwania na drugi klik
unsigned long firstClickTime = 0;      // Czas pierwszego kliknięcia

// Double-click variables for AUD button (SDPlayer activation)
bool waitingForSecondAudClick = false; // Flaga oczekiwania na drugie naciśnięcie AUD
unsigned long firstAudClickTime = 0;   // Czas pierwszego naciśnięcia AUD
int lastClickDirection = 0;            // Kierunek kliknięcia (+1 prawo, -1 lewo)

// IR Track Selection dla SDPlayer
int irRequestedTrackNumber = 0;        // Numer utworu wybrany przez IR (1-100) 
bool irTrackSelectionRequest = false;  // Flaga żądania wyboru utworu przez IR

// ===== SYSTEM WIELOCYFROWEGO WPROWADZANIA IR =====
int irInputBuffer = 0;                 // Bufor dla wprowadzanej liczby (0-100)
unsigned long irInputTimeout = 0;      // Timeout dla wprowadzania wielocyfrowego
const int IR_INPUT_TIMEOUT_MS = 3000;  // 3 sekundy na wprowadzenie liczby
bool irInputActive = false;            // Czy trwa wprowadzanie wielocyfrowe

// Koniec definicji pilota IR

bool  f_callInfo = 0;       // Flaga czy wsiwetlamy "INFO" również na OLED
bool  f_logo = 0;           // Flaga czy wyswietlamy logo
bool  f_simpleMode3 =0;

// SDPlayer global flags
bool sdPlayerAutoPlayNext = false; // Flaga żądania automatycznego odtworzenia następnego utworu
bool sdPlayerScanRequested = false; // Flaga żądania skanowania katalogu (obsługa w loop() aby uniknąć watchdog)

// SDPlayer track list variables
bool listedTracks = false;         // Flaga określająca czy na ekranie jest pokazana lista utworów do wyboru
int currentTrackSelection = 0;     // Numer aktualnie zaznaczonego utworu w liście
int firstVisibleTrack = 0;         // Numer pierwszego widocznego utworu na ekranie
int tracksCount = 0;               // Liczba utworów znalezionych na karcie SD
int maxVisibleTracks = 4;          // Maksymalna liczba widocznych utworów na ekranie OLED
String trackFiles[100];            // Tablica przechowująca nazwy plików utworów (max 100)
int lastPlayedTrackIndex = -1;     // Indeks ostatnio odtwarzanego utworu z listy (-1 = żaden)
int sdPlayerSessionStartIndex = -1; // Indeks utworu startowego ostatniej sesji SDPlayer
int sdPlayerSessionEndIndex = -1;   // Indeks ostatniego utworu w ostatniej sesji SDPlayer
bool sdPlayerSessionCompleted = false; // Czy ostatnia sesja została zakończona/wyłączona
const char* sdPlayerSessionStateFile = "/sdplayer_session.txt";
const char* sdPlayerActiveStateFile = "/sdplayer_active.txt"; // Persistent state: czy SD Player był aktywny przed wyłączeniem

int currentSelection = 0;                     // Numer aktualnego wyboru na ekranie OLED
int firstVisibleLine = 0;                     // Numer pierwszej widocznej linii na ekranie OLED
uint8_t station_nr = 0;                       // Numer aktualnie wybranej stacji radiowej z listy
int stationFromBuffer = 0;                    // Numer stacji radiowej przechowywanej w buforze do przywrocenia na ekran po bezczynności
uint8_t bank_nr;                              // Numer aktualnie wybranego banku stacji z listy
uint8_t previous_bank_nr = 0;                 // Numer banku przed wejsciem do menu zmiany banku
//int bankFromBuffer = 0;                       // Numer aktualnie wybranego banku stacji z listy do przywrócenia na ekran po bezczynności

//Enkoder 2 (podstawowy)
int CLK_state2;                               // Aktualny stan CLK enkodera lewego
int prev_CLK_state2;                          // Poprzedni stan CLK enkodera lewego
int stationsCount = 0;                        // Aktualna liczba przechowywanych stacji w tablicy
int buttonLongPressTime2 = 2000;              // Czas reakcji na długie nacisniecie enkoder 2
int buttonShortPressTime2 = 500;              // Czas rekacjinna krótkie nacisniecie enkodera 2
int buttonSuperLongPressTime2 = 3000;         // Czas reakcji na super długie nacisniecie enkoder 2

uint8_t volumeValue = 10;                     // Wartość głośności, domyślnie ustawiona na 10
uint8_t maxVolume = 21;
bool maxVolumeExt =  false;                   // 0(false) -  zakres standardowy Volume 1-21 , 1 (true) - zakres rozszerzony 0-42
uint8_t volumeBufferValue = 0;                // Wartość głośności, domyślnie ustawiona na 10
int maxVisibleLines = 4;                      // Maksymalna liczba widocznych linii na ekranie OLED
int bitrateStringInt = 0;                     // Deklaracja zmiennej do konwersji Bitrate string na wartosc Int aby podzelic bitrate przez 1000
int SampleRate = 0;
int SampleRateRest = 0;
int audioBufferTime = 0;                      // Zienna trzymajca informacje o ilosci sekund muzyki w buforze audio

uint8_t stationNameLenghtCut = 24;            // 24-> 25 znakow, 25-> 26 znaków, zmienna określająca jak długa nazwę ma nazwa stacji w plikach Bankow liczone od 0- wartosci ustalonej
const uint8_t yPositionDisplayScrollerMode0 = 33;   // Wysokosc (y) wyswietlania przewijanego/stalego tekstu stacji w danym trybie
const uint8_t yPositionDisplayScrollerMode1 = 61;   // Wysokosc (y) wyswietlania przewijanego/stalego tekstu stacji w danym trybie
const uint8_t yPositionDisplayScrollerMode2 = 25;   // Wysokosc (y) wyswietlania przewijanego/stalego tekstu stacji w danym trybie
const uint8_t yPositionDisplayScrollerMode3 = 49;   // Wysokosc (y) wyswietlania przewijanego/stalego tekstu stacji w danym trybie
const uint8_t yPositionDisplayScrollerMode8 = 35;   // Wysokosc (y) wyswietlania przewijanego/stalego tekstu stacji w danym trybie
const uint8_t stationNamePositionYmode3 = 31;
const uint8_t stationNamePositionYmode8 = 26;
uint16_t stationStringScrollLength = 0;
uint8_t maxStationVisibleStringScrollLength = 46;
bool stationNameFromStream = 0;               // Flaga definiujaca czy wyswietlamy nazwe stacji z plikow Banku pamieci czy z informacji nadawanych w streamie
bool f_powerOff = 0;                          // Flaga aktywnego trybu power off (ESP light sleep)
bool f_sleepTimerOn = false;
uint8_t sleepTimerValueSet = 0;           // Zmiena przechowujaca ustawiony czas - nie uzywana obecnie
uint8_t sleepTimerValueCounter = 0;         // Licznik funkcji sleep timer dekrementowany przez Timer3 co 1 minute
uint8_t sleepTimerValueStep = 15;           // Krok funkcji sleep timer o ile zwieksza się timer
uint8_t sleepTimerValueMax = 90;            // Maksymalna wartosc sleep timera jaka da się ustawić
//uint8_t bank_nr_max = 16;


// ---- Głosowe odtwarzanie czasu co godzinę ---- //
bool f_requestVoiceTimePlay = false; 
bool timeVoiceInfoEveryHour = true;
unsigned long voiceTimeMillis = 0;
uint16_t voiceTimeReturnTime = 4000;


// ---- SD Player - zapamiętanie stacji przed aktywacją ---- //
uint8_t sdPlayerReturnBank = 1;          // Bank do którego wracamy po wyjściu z SDPlayera
uint8_t sdPlayerReturnStation = 4;       // Stacja do której wracamy po wyjściu z SDPlayera

// ---- Auto dimmer / auto przyciemnianie wyswietlacza ---- //
uint8_t displayDimmerTimeCounter = 0;  // Zmienna inkrementowana w przerwaniu timera2 do przycimniania wyswietlacz
uint8_t dimmerDisplayBrightness = 10;   // Wartość przyciemnienia wyswietlacza po czasie niekatywnosci
uint8_t dimmerSleepDisplayBrightness = 1; // Wartość przyciemnienia wyswietlacza po czasie niekatywnosci
uint8_t displayBrightness = 180;        // Domyślna maksymalna janość wyswietlacza
uint16_t displayAutoDimmerTime = 5;   // Czas po jakim nastąpi przyciemninie wyswietlacza, liczony w sekundach
bool displayAutoDimmerOn = false;      // Automatyczne przyciemnianie wyswietlacza, domyślnie włączone
bool displayDimmerActive = false;       // Aktywny tryb przyciemnienia
uint16_t displayPowerSaveTime = 30;    // Czas po jakim zostanie wyłączony wyswietlacz OLED (tryb power save)
uint16_t displayPowerSaveTimeCounter = 0;    // Timer trybu Power Save wyswietlacza OLED
bool displayPowerSaveEnabled = false;   // Flaga okreslajaca czy tryb wyłączania wyswietlacza OLED jest właczony
uint8_t displayMode = 0;               // Tryb wyswietlacza 0-displayRadio z przewijaniem "scroller" / 1-Zegar / 2- tryb 3 stałych linijek tekstu stacji

// ---- Equalzier ---- //
int8_t toneLowValue = 0;               // Wartosc filtra dla tonow niskich
int8_t toneMidValue = 0;               // Wartosc flitra dla tonow srednich
int8_t toneHiValue = 0;                // Wartość filtra dla tonow wysokich
int8_t balanceValue = 0;               // Wartość balansu L/R (zakres -16 do +16)
uint8_t toneSelect = 1;                // Zmienna okreslająca, który filtr equalizera regulujemy, startujemy od pierwszego
bool equalizerMenuEnable = false;      // Flaga wyswietlania menu Equalizera

// ---- Equalizer 16-band ---- //
#if defined(ENABLE_EQ16) && (ENABLE_EQ16 == 1)
bool eq16Mode = false;                 // false=3-band, true=16-band
bool eq16ModeSelectActive = false;     // Czy aktywny wybór trybu EQ
uint8_t eq16SelectedBand = 0;         // Aktualnie wybrany band (0-15)
int8_t eq16Gains[16] = {0};           // Bufor dla 16 pasm
#endif

// =============== ZMIENNE GLOBALNE MODUŁÓW ===============
// Analyzer - analizator spektrum
bool analyzerActive = false;     // Czy analyzer jest aktywny
bool analyzerEnabled = true;    // Czy analyzer jest włączony globalnie
int analyzerStyles = 2;          // 0=styles 0-4-5, 1=0-4-6, 2=All(0-4-5-6-10-11)
int analyzerPreset = 4;          // 0=Classic, 1=Modern, 2=Compact, 3=Retro, 4=Custom
bool displayMode6Enabled = true;  // Czy styl 6 jest dostępny w pętli SRC (0-4,8 zawsze dostępne) 
bool displayMode8Enabled = true;  // Czy styl 8 jest dostępny w pętli SRC (przeniesiony z 5)
bool displayMode10Enabled = true; // Czy styl 10 jest dostępny w pętli SRC (0-4,8 zawsze dostępne)

// SDPlayer - odtwarzacz plików (extern dla SDPlayerOLED/WebUI)
extern bool sdPlayerActive;
extern bool sdPlayerPlayingMusic;
extern bool sdPlayerOLEDActive;
bool sdPlayerActive = false;     // Czy SDPlayer jest aktywny
bool sdPlayerPlayingMusic = false; // Czy SDPlayer odtwarza muzykę (używane przez SDPlayerOLED/WebUI)
bool sdPlayerOLEDActive = false;   // Czy OLED SDPlayer jest aktywny

// SDPlayer - style wyświetlania (checkboxy w WebUI dla pętli SRC)
bool sdPlayerStyle1Enabled = true;   // Style 1-14: kontrola dostępności w nextStyle()
bool sdPlayerStyle2Enabled = true;
bool sdPlayerStyle3Enabled = true;
bool sdPlayerStyle4Enabled = true;
bool sdPlayerStyle5Enabled = true;
bool sdPlayerStyle6Enabled = true;
bool sdPlayerStyle7Enabled = true;
bool sdPlayerStyle9Enabled = true;   // STYLE_8 jest dostępny na stałe (layout SDPlayerAdvanced)
bool sdPlayerStyle10Enabled = true;
bool sdPlayerStyle11Enabled = true;
bool sdPlayerStyle12Enabled = true;
bool sdPlayerStyle13Enabled = true;
bool sdPlayerStyle14Enabled = true;

// BTModule - moduł Bluetooth UART
bool btModuleEnabled = false;      // Czy moduł BT przez UART jest włączony

// ====================================================


uint8_t rcInputDigit1 = 0xFF;      // Pierwsza cyfra w przy wprowadzaniu numeru stacji z pilota
uint8_t rcInputDigit2 = 0xFF;      // Druga cyfra w przy wprowadzaniu numeru stacji z pilota


// ---- Zmienne konfiguracji ---- //
uint16_t configArray[47] = {0};  // Zwiększone do 47 (0-46 = radio + SDPlayer + presetsOn + speakersEnabled)
#define CONFIG_COUNT 47          // Zwiększone do 47 (presetsOn=[45], speakersEnabled=[46])
uint8_t rcPage = 0;
uint16_t configRemoteArray[62] = {0};   // Tablica przechowująca kody pilota (rozszerzona z 30 do 62 dla wszystkich przycisków)
uint16_t configAdcArray[20] = { 0};      // Tablica przechowująca wartosci ADC dla przyciskow klawiatury
bool configExist = true;                  // Flaga okreslajaca czy istnieje plik konfiguracji
bool f_displayPowerOffClock = false;      // Flaga okreslajaca czy w trybie sleep ma się wyswietlac zegar
bool f_sleepAfterPowerFail = false;       // Flaga czy idziemy do powerOFF po powrocie zasilania
bool f_saveVolumeStationAlways = false;   // Flaga określająca czy zapisujemy stacje, bank i poziom volume zawsze czy tylko przy power OFF
//bool f_statusLED = false;
bool f_powerOffAnimation = false;             // Animacja przy power OFF
bool presetsOn = false;                       // Tryb Presets: klawisze 1-9 = stacje 1-9 z aktualnego banku
bool speakersEnabled = true;                  // Speakers ON: pin SPEAKERS_PIN aktywny wg stanu; OFF: zawsze LOW

//const int maxVisibleLines = 5;  // Maksymalna liczba widocznych linii na ekranie OLED
bool encoderButton2 = false;      // Flaga określająca, czy przycisk enkodera 2 został wciśnięty
bool encoderFunctionOrder = false; // Flaga okreslająca kolejność funkcji enkodera 2
bool displayActive = false;       // Flaga określająca, czy wyświetlacz jest aktywny

bool mp3 = false;                 // Flaga określająca, czy aktualny plik audio jest w formacie MP3
bool flac = false;                // Flaga określająca, czy aktualny plik audio jest w formacie FLAC
bool aac = false;                 // Flaga określająca, czy aktualny plik audio jest w formacie AAC
bool vorbis = false;              // Flaga określająca, czy aktualny plik audio jest w formacie VORBIS
bool opus = false;
bool timeDisplay = true;          // Flaga określająca kiedy pokazać czas na wyświetlaczu, domyślnie od razu po starcie
bool listedStations = false;      // Flaga określająca czy na ekranie jest pokazana lista stacji do wyboru
bool bankMenuEnable = false;      // Flaga określająca czy na ekranie jest wyświetlone menu wyboru banku
bool bitratePresent = false;      // Flaga określająca, czy na serial terminalu pojawiła się informacja o bitrate - jako ostatnia dana spływajaca z info
bool bankNetworkUpdate = false;   // Flaga wyboru aktualizacji banku z sieci lub karty SD - True aktulizacja z interNETu
bool volumeMute = false;          // Flaga okreslająca stan funkcji wyciszczenia - Mute
bool volumeSet = false;           // Flaga wejscia menu regulacji głosnosci na enkoderze 2

bool action3Taken = false;        // Flaga Akcji 3

bool ActionNeedUpdateTime = false;// Zmiena okresaljaca dla displayRadio potrzebe odczytu aktulizacji czasu
bool debugAudioBuffor = false;    // Wyswietlanie bufora Audio
bool f_audioInfoRefreshStationString = false;    // Flaga wymuszjąca wymagane odsiwezenie ze względu na zmianę info stream - stationstring, title, 
bool f_audioInfoRefreshDisplayRadio = false;    // Flaga wymuszjąca wymagane odsiwezenie ze względu na zmianę info stream - linia kbps, khz, format streamu
bool resumePlay = false;            // Flaga wymaganego uruchomienia odtwarzania po zakonczeniu komunikatu głosowego
bool fwupd = false;               // Flaga blokujaca main loop podczas aktualizacji oprogramowania
bool configIrExist = false;       // Flaga informująca o istnieniu poprawnej konfiguracji pilota IR
bool wsAudioRefresh = false;      // Flaga informujaca o potrzebe odswiezeninia Station Text za pomoca Web Sokcet
uint8_t wifiSignalLevel = 0;           // Aktualny poziom sygnalu WiFi (0-6)
uint8_t wifiSignalLevelLast = 7;       // Pamiec poprzedniego stanu sygnalu WiFi
bool f_voiceTimeBlocked = false;
bool f_displaySleepTime = false;    // Flaga wysiwetlania menu timera
bool f_displaySleepTimeSet = false;    // Flaga ustawienia sleep timera

bool noSDcard = false;              // flaga ustawiana przy braku wykrycia karty SD
bool noStorage = false;             // flaga ustawiana przy problemie z pamiecia SPIFFS/LittleFS

unsigned long debounceDelay = 300;    // Czas trwania debouncingu w milisekundach
unsigned long displayTimeout = 8000;  // Czas wyświetlania komunikatu na ekranie w milisekundach (8 sekund)
unsigned long displayStartTime = 0;   // Czas rozpoczęcia wyświetlania komunikatu
unsigned long seconds = 0;            // Licznik sekund timera
unsigned int PSRAM_lenght = MAX_STATIONS * (STATION_NAME_LENGTH) + MAX_STATIONS; // deklaracjia długości pamięci PSRAM
unsigned long lastCheckTime = 0;      // No stream audio blink
//uint8_t stationNameStreamWidth = 0;   // Test pełnej nazwy stacji
uint64_t seconds2nextMinute;
uint64_t micros2nextMinute;

// ---- VU Meter ---- //
unsigned long vuMeterTime;                 // Czas opznienia odswiezania wskaznikow VU w milisekundach
uint8_t vuMeterL;                          // Wartosc VU dla L kanału zakres 0-255
uint8_t vuMeterR;                          // Wartosc VU dla R kanału zakres 0-255
uint8_t peakL = 0;                         // Wartosc peakHold dla kanalu lewego
uint8_t peakR = 0;                         // Wartosc peakHold dla kanalu prawego
uint8_t peakHoldTimeL = 0;                 // Licznik peakHold dla kanalu lewego
uint8_t peakHoldTimeR = 0;                 // Licznik peakHold dla kanalu prawego
const uint8_t peakHoldThreshold = 5;       // Liczba cykli zanim peak opadnie
const uint8_t vuLy = 41;                   // Koordynata Y wskaznika VU L-lewego (wyzej)
const uint8_t vuRy = 47;                   // Koordynata Y wskaznika VU R-prawego (nizej)
const uint8_t vuLyMode3 = 54;              // Koordynata Y wskaznika VU L-lewego (wyzej)
const uint8_t vuRyMode3 = 60;              // Koordynata Y wskaznika VU R-prawego (nizej
const uint8_t vuLyMode5 = 43;                   // Koordynata Y wskaznika VU L-lewego (wyzej)
const uint8_t vuRyMode5 = 49;                   // Koordynata Y wskaznika VU R-prawego (nizej)
const uint8_t vuThicknessMode3 = 1;        // Grubosc kreseczki wskaznika VU w trybie Mode3
const int vuCenterXmode3 = 128;            // Pozycja centralna startu VU w trybie Mode 3
const int vuYmode3 = 62;                   // Polozenie wyokosci (Y) VU w trybie Mode 3
bool vuPeakHoldOn = 1;                     // Flaga okreslajaca czy funkcja Peak & Hold na wskazniku VUmeter jest wlaczona
bool vuMeterOn = true;                     // Flaga właczajaca wskazniki VU
bool vuMeterMode = false;                  // tryb rysowania vuMeter
uint8_t displayVuL = 0;                    // wartosc VU do wyswietlenia po procesie smooth
uint8_t displayVuR = 0;                    // wartosc VU do wyswietlenia po procesie smooth
uint8_t vuRiseSpeed = 24;                  // szybkość narastania VU
uint8_t vuFallSpeed = 6;                   // szybkość opadania VU
bool vuSmooth = 1;                         // Flaga zalaczenia funkcji smootht (domyslnie wlaczona=1)
uint8_t vuRiseNeedleSpeed = 2;             // szybkość narastania VU igły analogowej
uint8_t vuFallNeedleSpeed = 6;             // szybkość opadania VU igły analogowej
bool reversColorMode4 = true;              // Odwrócone kolory dla Mode 4 (białe tło)

// Volume Fade
unsigned long lastFadeTime = 0;
const unsigned long fadeStepDelay = 25; // ms
bool fadingIn = false;
uint8_t targetVolume = 0;
uint8_t currentVolume = 0;
bool f_volumeFadeOn = 0;
uint8_t volumeFadeOutTime = 25; //ms * volumeValue
uint8_t volumeSleepFadeOutTime = 50;

unsigned long scrollingStationStringTime;  // Czas do odswiezania scorllingu
uint8_t scrollingRefresh = 50;              // Czas w ms przewijania tekstu funkcji Scroller

uint8_t vuMeterRefreshCounterSet = 0;      // Mnoznik co ile petli loopRefreshTime ma byc odswiezony VU Meter
uint8_t scrollerRefreshCounterSet = 0;     // Mnoznik co ile petli loopRefreshTime ma byc odswiezony i przewiniety o 1px Scroller

uint16_t stationStringScrollWidth;         // szerokosc Stringu nazwy stacji w funkcji Scrollera
uint16_t xPositionStationString = 0;       // Pozycja początkowa dla przewijania tekstu StationString
uint16_t offset;                           // Zminnna offsetu dla funkcji Scrollera - przewijania streamtitle na ekranie OLED
unsigned char * psramData;                 // zmienna do trzymania danych stacji w pamieci PSRAM

unsigned long vuMeterMilisTimeUpdate;           // Zmienna przechowujaca czas dla funkci millis VU Meter refresh
uint8_t vuMeterRefreshTime = 50;                // Czas w ms odswiezania VUmetera

// ---- Serwer Web ---- //
unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 2000;
bool urlToPlay = false;
bool urlPlaying = false;


// ---- Sprawdzenie funkcji pilota, zminnne do pomiaru róznicy czasów ---- //
unsigned long runTime = 0;
unsigned long runTime1 = 0;
unsigned long runTime2 = 0;


String stationStringScroll = "";     // Zmienna przechowująca tekst do przewijania na ekranie
String stationName;                  // Nazwa aktualnie wybranej stacji radiowej
String stationString;                // Dodatkowe dane stacji radiowej (jeśli istnieją)
String stationStringWeb;                // Dodatkowe dane stacji radiowej (jeśli istnieją)
String bitrateString;                // Zmienna przechowująca informację o bitrate
String sampleRateString;             // Zmienna przechowująca informację o sample rate
String bitsPerSampleString;          // Zmienna przechowująca informację o liczbie bitów na próbkę
String currentIP;
String stationNameStream;           // Nazwa stacji wyciągnieta z danych wysylanych przez stream
String streamCodec;
String stationLogoUrl;
String timezone;

// Metadane ID3 dla plików MP3 (SDPlayer)
String currentMP3Artist = "";
String currentMP3Title = "";
String currentMP3Album = "";


String header;                      // Zmienna dla serwera www
String sliderValue = "0";
String url2play = "";


File myFile;  // Uchwyt pliku

U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/CS_OLED, /* dc=*/DC_OLED, /* reset=*/RESET_OLED);  // Hardware SPI 3.12inch OLED
//U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/CS_OLED, /* dc=*/DC_OLED, /* reset=*/RESET_OLED);  
//U8G2_SH1122_256X64_F_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/ CS_OLED, /* dc=*/ DC_OLED, /* reset=*/ RESET_OLED);		// Hardware SPI  2.08inch OLED
//U8G2_SSD1363_256X128_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/CS_OLED, /* dc=*/DC_OLED, /* reset=*/RESET_OLED);  // Hardware SPI 3.12inch OLED

// Przypisujemy port serwera www
AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); //Start obsługi Websocketów


// Inicjalizacja WiFiManagera
WiFiManager wifiManager;

// Konfiguracja nowego SPI z wybranymi pinami dla czytnika kart SD
SPIClass customSPI = SPIClass(HSPI);  // Używamy HSPI, ale z własnymi pinami

#ifdef twoEncoders
  ezButton button1(SW_PIN1);  // Utworzenie obiektu przycisku z enkodera 2 ezButton
#endif

ezButton button2(SW_PIN2);  // Utworzenie obiektu przycisku z enkodera 2 ezButton
Audio audio;                // Obiekt do obsługi funkcji związanych z dźwiękiem i audio

// SDPlayer i BTWebUI - globalne instancje
SDPlayerWebUI* g_sdPlayerWeb = nullptr;
SDPlayerOLED* g_sdPlayerOLED = nullptr;
BTWebUI* g_btWebUI = nullptr;

// DLNAWebUI - globalna instancja
#ifdef USE_DLNA
DLNAWebUI* g_dlnaWebUI = nullptr;
#endif

// SDRecorder - globalna instancja (definicja w SDRecorder.cpp)
// extern SDRecorder* g_sdRecorder; // Jest już zdefiniowane w SDRecorder.h

// Wrapper functions for SDPlayer
void SDPlayerOLED_init(U8G2* display) {
  if (!g_sdPlayerWeb) {
    g_sdPlayerWeb = new SDPlayerWebUI();
  }
  if (!g_sdPlayerOLED) {
    g_sdPlayerOLED = new SDPlayerOLED(*display);
    g_sdPlayerOLED->begin(g_sdPlayerWeb);
    
    // KRYTYCZNE: Ustaw referencję OLED w WebUI dla przełączników stylów
    if (g_sdPlayerWeb) {
      g_sdPlayerWeb->setOLED(g_sdPlayerOLED);
      Serial.println("[Main] SDPlayer WebUI->OLED reference established for style switching");
    }
    
    // NOWA SYNCHRONIZACJA: Ustaw callback między WebUI i OLED
    if (g_sdPlayerWeb && g_sdPlayerOLED) {
      // WebUI -> OLED: synchronizuj index wyboru 
      g_sdPlayerWeb->setIndexChangeCallback([](int newIndex) {
        if (g_sdPlayerOLED && g_sdPlayerOLED->isActive() && newIndex >= 0) {
          g_sdPlayerOLED->setSelectedIndex(newIndex);
          Serial.printf("[Main] WebUI->OLED index sync: %d\\n", newIndex);
        }
      });
      
      // OLED -> WebUI: synchronizuj index z pilota/enkodera
      g_sdPlayerOLED->setIndexChangeCallback([](int newIndex) {
        if (g_sdPlayerWeb && newIndex >= 0) {
          g_sdPlayerWeb->notifyIndexChange(newIndex);
          Serial.printf("[Main] OLED->WebUI index sync: %d\\n", newIndex);
        }
      });
      
      Serial.println("[Main] SDPlayer bi-directional sync initialized");
    }
  }
}

void SDPlayerOLED_loop() {
  if (g_sdPlayerOLED && g_sdPlayerOLED->isActive()) {
    // KRYTYCZNE: Nie rysuj stylu SDPlayerOLED gdy aktywna jest lista utworów
    if (!listedTracks) {
      g_sdPlayerOLED->loop();
    }
    // Gdy listedTracks == true, lista pozostaje na ekranie
    // (narysowana przez displayTracks(), nie jest nadpisywana)
  }
}

void SDPlayerWebUI_registerHandlers(AsyncWebServer& server) {
  if (g_sdPlayerWeb) {
    g_sdPlayerWeb->begin(&server, &audio);
  }
}

// Wrapper functions for SDRecorder
void SDRecorder_displayCallback(const String& status, const String& time, const String& size) {
  // Wyświetl status nagrywania na OLED (w przyszłości może być osobny ekran)
  // Na razie loguj do Serial
  Serial.printf("[SDRecorder Display] Status: %s, Time: %s, Size: %s\n", 
                status.c_str(), time.c_str(), size.c_str());
}

void SDRecorder_init() {
  if (!g_sdRecorder) {
    g_sdRecorder = new SDRecorder();
    g_sdRecorder->begin();
    g_sdRecorder->setDisplayCallback(SDRecorder_displayCallback);
    Serial.println("[SDRecorder] Initialized");
  }
}

void SDRecorder_loop() {
  if (g_sdRecorder) {
    g_sdRecorder->loop();
  }
}

// Wrapper functions for BTWebUI
void BTWebUI_init() {
  if (!g_btWebUI) {
    g_btWebUI = new BTWebUI();
  }
}

void BTWebUI_loop() {
  if (g_btWebUI) {
    g_btWebUI->loop();
  }
}

void BTWebUI_registerHandlers(AsyncWebServer& server) {
  if (g_btWebUI) {
    // UART pins for BT module: RX=19, TX=20, baud=115200
    g_btWebUI->begin(&server, 19, 20, 115200);
  }
}

// Wrapper functions for DLNAWebUI
#ifdef USE_DLNA
void DLNAWebUI_init() {
  if (!g_dlnaWebUI) {
    g_dlnaWebUI = new DLNAWebUI();
    Serial.println("[DLNAWebUI] Initialized");
  }
}

void DLNAWebUI_registerHandlers(AsyncWebServer& server) {
  if (g_dlnaWebUI) {
    g_dlnaWebUI->begin(&server);
  }
}

void DLNAWebUI_loop() {
  if (g_dlnaWebUI) {
    g_dlnaWebUI->loop();
  }
}
#endif


Ticker timer1;             // Timer do updateTimer co 1s
Ticker timer2;             // Timer do getWeatherData co 60s
Ticker timer3;           // Timer do przełączania wyświetlania danych pogodoych w ostatniej linii co 10s
Ticker ledBlinker;         // Timer do migania LED Standby podczas startu / problemow WiFi
WiFiClient client;         // Obiekt do obsługi połączenia WiFi dla klienta HTTP

const char stylehead_html[] PROGMEM = R"rawliteral(<!DOCTYPE HTML><html><head><link rel='icon' href='/favicon.ico' type='image/x-icon'><link rel="shortcut icon" href="/favicon.ico" type="image/x-icon"><link rel="apple-touch-icon" sizes="180x180" href="/icon.png"><link rel="icon" type="image/png" sizes="192x192" href="/icon.png"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Evo Web Radio</title><style>html{font-family:Arial;display:inline-block;text-align:center}h2{font-size:1.3rem}p{font-size:.95rem}table{border:1px solid black;border-collapse:collapse;margin:0}td,th{font-size:.8rem;border:1px solid gray;border-collapse:collapse}td:hover{font-weight:bold}a{color:black;text-decoration:none}body{max-width:1380px;margin:0 auto;padding-bottom:25px;background:#D0D0D0}.slider{-webkit-appearance:none;margin:14px;width:330px;height:10px;background:#4CAF50;outline:none;-webkit-transition:.2s;transition:opacity .2s;border-radius:5px}.slider::-webkit-slider-thumb{-webkit-appearance:none;appearance:none;width:35px;height:25px;background:#4a4a4a;cursor:pointer;border-radius:5px}.slider::-moz-range-thumb{width:35px;height:35px;background:#4a4a4a;cursor:pointer;border-radius:5px}.button{background-color:#4CAF50;border:1;color:white;padding:10px 20px;border-radius:5px}.buttonBank{background-color:#4CAF50;border:1;color:white;padding:8px;border-radius:5px;width:35px;height:35px;margin:0 1.5px}.buttonBankSelected{background-color:#505050;border:1;color:white;padding:8px;border-radius:5px;width:35px;height:35px;margin:0 1.5px}.buttonBank:active{background-color:#4a4a4a;box-shadow:0 4px #666;transform:translateY(2px)}.buttonBank:hover{background-color:#4a4a4a}.button:hover{background-color:#4a4a4a}.button:active{background-color:#4a4a4a;box-shadow:0 4px #666;transform:translateY(2px)}.column{padding:5px;display:flex;justify-content:space-between}.columnlist{padding:10px;display:flex;justify-content:center}.stationList{text-align:left;margin-top:0;width:280px;margin-bottom:0;cursor:pointer}.stationNumberList{text-align:center;margin-top:0;width:35px;margin-bottom:0}.stationListSelected{text-align:left;margin-top:0;width:280px;margin-bottom:0;cursor:pointer;background-color:#4CAF50}.stationNumberListSelected{text-align:center;margin-top:0;width:35px;margin-bottom:0;background-color:#4CAF50}</style></head>)rawliteral";

const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML><html>
  <head>
    <link rel='icon' href='/favicon.ico' type='image/x-icon'>
    <link rel="shortcut icon" href="/favicon.ico" type="image/x-icon">
    <link rel="apple-touch-icon" sizes="180x180" href="/icon.png">
    <link rel="icon" type="image/png" sizes="192x192" href="/icon.png">

    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Evo Web Radio</title>
    <style>
      html {font-family: Arial; display: inline-block; text-align: center;}
      h2 {font-size: 1.3rem;}
      p {font-size: 0.95rem;}
      table {border: 1px solid black; border-collapse: collapse; margin: 0px 0px;}
      td, th {font-size: 0.8rem; border: 1px solid gray; border-collapse: collapse;}
      td:hover {font-weight:bold;}
      a {color: black; text-decoration: none;}
      body {max-width: 1380px; margin:0px auto; padding-bottom: 25px; background: #D0D0D0;}
      .slider {-webkit-appearance: none; margin: 14px; width: 330px; height: 10px; background: #4CAF50; outline: none; -webkit-transition: .2s; transition: opacity .2s; border-radius: 5px;}
      .slider::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; width: 35px; height: 25px; background: #4a4a4a; cursor: pointer; border-radius: 5px;}
      .slider::-moz-range-thumb { width: 35px; height: 35px; background: #4a4a4a; cursor: pointer; border-radius: 5px;} 
      .button { background-color: #4CAF50; border: 1; color: white; padding: 10px 20px; border-radius: 5px;}
      .buttonBank { background-color: #4CAF50; border: 1; color: white; padding: 8px 8px; border-radius: 5px; width: 35px; height: 35px; margin: 0 1.5px;}
      .buttonBankSelected { background-color: #505050; border: 1; color: white; padding: 8px 8px; border-radius: 5px; width: 35px; height: 35px; margin: 0 1.5px;}
      .buttonBank:active {background-color: #4a4a4a; box-shadow: 0 4px #666; transform: translateY(2px);}
      .buttonBank:hover {background-color: #4a4a4a;}
      .button:hover {background-color: #4a4a4a;}
      .button:active {background-color: #4a4a4a; box-shadow: 0 4px #666; transform: translateY(2px);}
      .column {padding: 5px; display: flex; justify-content: space-between;}
      .columnlist {padding: 10px; display: flex; justify-content: center;}
      .stationList {text-align:left; margin-top: 0px; width: 280px; margin-bottom:0px;cursor: pointer;}
      .stationNumberList {text-align:center; margin-top: 0px; width: 35px; margin-bottom:0px;}
      .stationListSelected {text-align:left; margin-top: 0px; width: 280px; margin-bottom:0px;cursor: pointer; background-color: #4CAF50;}
      .stationNumberListSelected {text-align:center; margin-top: 0px; width: 35px; margin-bottom:0px; background-color: #4CAF50;}
      .station-name {}
      .wifi-wrapper {display: flex; align-items: flex-end; gap: 1px; height: 13px; margin-top: -3px; margin-bottom: 3px; justify-content: center; }
      .wifiBar {width: 2px;background-color: #555; border-radius: 2px; transition: background-color 0.2s;}
      .bar1 { height: 2px; }
      .bar2 { height: 4px; }
      .bar3 { height: 6px; }
      .bar4 { height: 8px; }
      .bar5 { height: 10px; }
      .bar6 { height: 12px; }
    </style>
  </head>

  <body>
    <h2>Evo Web Radio</h2>
  
    <div id="display" style="display: inline-block; padding: 5px; border: 2px solid #4CAF50; border-radius: 15px; background-color: #4a4a4a; font-size: 1.45rem; 
      color: #AAA; width: 345px; text-align: center; white-space: nowrap; box-shadow: 0 0 20px #4CAF50;" onClick="flashBackground(); displayMode()">
      
      <div style="margin-bottom: 10px; font-weight: bold; overflow: hidden; text-overflow: ellipsis; -webkit-text-stroke: 0.3px black; text-stroke: 0.3px black;">
        <span id="textStationName"><b>STATIONNAME</b></span>
      </div>
      
      <div style="width: 345px; margin-bottom: 10px;">
        <div id="stationTextDiv" style="display: block; text-overflow: ellipsis; white-space: normal; font-size: 1.0rem; color: #999; margin-bottom: 10px; text-align: center; align-items: center; height: 4.2em; justify-content: center; overflow: hidden; line-height: 1.4em;">
          <span id="stationText">STATIONTEXT</span>
        </div>
        <div id="muteIndicatorDiv" style="text-align:center; color:#4CAF50; font-weight:bold; font-size:1.0rem; height:1.4em; margin-top:-5px;">
        <span id="muteIndicator"></span>
        </div>
      </div>
      
      <div style="height: 1px; background-color: #4CAF50; margin: 5px 0;"></div>

      
      <div style="display: flex; justify-content: center; gap: 13px; font-size: 1.00rem; color: #999;">
        <div><span id="bankValue">Bank: --</span></div>
          <div style="display: flex; justify-content: center; gap: 6px; font-size: 0.55rem; color: #999;margin: 4px 0;">
            <div><span id="samplerate">---.-kHz</span></div>
            <div><span id="bitrate">---kbps</span></div>
            <div><span id="bitpersample">---bit</span></div>
            <div><span id="streamformat">----</span></div>
            WiFi: <div class="wifi-wrapper">
            <div class="wifiBar bar1" id="wifiBar1"></div>
            <div class="wifiBar bar2" id="wifiBar2"></div>
            <div class="wifiBar bar3" id="wifiBar3"></div>
            <div class="wifiBar bar4" id="wifiBar4"></div>
            <div class="wifiBar bar5" id="wifiBar5"></div>
            <div class="wifiBar bar6" id="wifiBar6"></div>
  	      </div>
          </div>
        <div><span id="stationNumber">Station: --</span></div>
      </div>

  </div>
  <br>
  <br>
  <script>

  var websocket;

  function updateSliderVolume(element) 
  {
    var sliderValue = document.getElementById("volumeSlider").value;
    document.getElementById("textSliderValue").innerText = sliderValue;

    if (websocket && websocket.readyState === WebSocket.OPEN) 
    {
      websocket.send("volume:" + sliderValue);
    } 
    else 
    {
      console.warn("WebSocket niepołączony");
    }
  }

  function changeStation(number) 
  {
    if (websocket.readyState === WebSocket.OPEN) 
    {
      websocket.send("station:" + number);
    } 
    else 
    {
      console.log("WebSocket nie jest otwarty");
    }
  }

  function changeBank(number) 
  {
    if (websocket.readyState === WebSocket.OPEN) 
    {
      websocket.send("bank:" + number);
    } 
    else 
    {
      console.log("WebSocket nie jest otwarty");
    }
  }


  function displayMode() 
  {
    //fetch("/displayMode")
    if (websocket.readyState === WebSocket.OPEN) 
    {
      websocket.send("displayMode");
    } 
    else 
    {
      console.log("WebSocket nie jest otwarty");
    }
  }
  
  function setWifi(level) 
  {
    for (let i = 1; i <= 6; i++) 
    {
      const bar = document.getElementById("wifiBar" + i);
      if (bar) {bar.style.backgroundColor = (i <= level) ? "#4CAF50" : "#555";}
    }
  }

  
  function connectWebSocket() 
  {
    websocket = new WebSocket('ws://' + window.location.hostname + '/ws');
    websocket.onopen = function () 
    {
      console.log("WebSocket polaczony");
    };
    
    websocket.onclose = function (event) 
    {
      console.log("WebSocket zamkniety. Proba ponownego polaczenia za 3 sekundy...");
      setTimeout(connectWebSocket, 3000); // próba ponownego połączenia
    };

    websocket.onerror = function (error) 
    {
      console.error("Blad WebSocket: ", error);
      websocket.close(); // zamyka połączenie, by wywołać reconnect
    };
    
    websocket.onmessage = function(event) 
    {
      if (event.data === "reload") 
      {
        location.reload();
      }  
      
      if (event.data.startsWith("volume:")) 
      {
        var vol = parseInt(event.data.split(":")[1]);
        document.getElementById("volumeSlider").value = vol;
        document.getElementById("textSliderValue").innerText = vol;
      }
      
      if (event.data.startsWith("station:")) 
      {
        var station = parseInt(event.data.split(":")[1]);
        highlightStation(station);
        //document.getElementById('stationNumber').innerText = event.data.split(':')[1];
        document.getElementById('stationNumber').innerText ='Station: ' + station; 
      }
      
      if (event.data.startsWith("stationname:")) 
      {
        var value = event.data.split(":")[1];
        document.getElementById("textStationName").innerHTML = `<b>${value}</b>`;
        //checkStationTextLength();
      }  

      if (event.data.startsWith("stationtext$")) 
      {
        //var stationtext = event.data.split("$")[1];
        //document.getElementById("stationText").innerHTML = `${stationtext}`;
        
        var stationtext = event.data.substring(event.data.indexOf("$") + 1);
        //document.getElementById("stationText").innerHTML = stationtext;       
        document.getElementById("stationText").textContent = stationtext;
      }  

      if (event.data.startsWith("bank:")) 
      {
        var bankValue = parseInt(event.data.split(":")[1]);
        document.getElementById('bankValue').innerText = 'Bank: ' + bankValue;
      } 

      //if (event.data.startsWith("samplerate:")) 
      //{
      //  var samplerate = parseInt(event.data.split(":")[1]);
      //  var formattedRate = (samplerate / 1000).toFixed(1) + "kHz";
      //  document.getElementById('samplerate').innerText = formattedRate;
      //}
      
      if (event.data.startsWith("samplerate:")) 
      {
        var raw = event.data.split(":")[1];
        var samplerate = parseInt(raw);

        if (isNaN(samplerate)) 
        {
          document.getElementById('samplerate').innerText = '---.-kHz';
        } 
        else 
        {
          var formattedRate = (samplerate / 1000).toFixed(1) + "kHz";
          document.getElementById('samplerate').innerText = formattedRate;
        }
      }

      //if (event.data.startsWith("bitrate:")) 
      //{	
      //  var bitrate = parseInt(event.data.split(":")[1]);
      //  document.getElementById('bitrate').innerText = bitrate + 'kbps';
      //}

      if (event.data.startsWith("bitrate:")) 
      {	
        var raw = event.data.split(":")[1];
        var bitrate = parseInt(raw);

        if (isNaN(bitrate)) 
        {
          document.getElementById('bitrate').innerText = '---kbps';
        } 
        else 
        {
          document.getElementById('bitrate').innerText = bitrate + 'kbps';
        }
      }

      
      //if (event.data.startsWith("bitpersample:")) 
      //{	
      //  var bitpersample = parseInt(event.data.split(":")[1]);
      //  document.getElementById('bitpersample').innerText = bitpersample + 'bit';
      //}

      if (event.data.startsWith("bitpersample:")) 
      {	
        var raw = event.data.split(":")[1];
        var bitpersample = parseInt(raw);

        if (isNaN(bitpersample)) 
        {
          document.getElementById('bitpersample').innerText = '---bit';
        } 
        else 
        {
          document.getElementById('bitpersample').innerText = bitpersample + 'bit';
        }
      }

      
      if (event.data.startsWith("streamformat:")) 
      {	
        var streamformat = event.data.split(":")[1];
        document.getElementById('streamformat').innerText = streamformat;
      }  

      if (event.data.startsWith("mute:")) 
      {
        //var muteInd = parseInt(event.data.split(":")[1]);
        const muteInd = event.data.split(":")[1] === "1" ? 1 : 0;
        document.getElementById('muteIndicator').textContent = (muteInd === 1) ? 'MUTED' : '';       
      }

      if (event.data.startsWith("wifi:")) 
      {
        const level = parseInt(event.data.split(":")[1]);
        if (!isNaN(level)) {setWifi(Math.min(Math.max(level, 1), 6));}
      }

    }

  };
  
  function highlightStation(stationId) 
  {
    // Usuń poprzednie zaznaczenia
    document.querySelectorAll(".stationList").forEach(el => {
    el.classList.remove("stationListSelected");
    //el.innerHTML = el.dataset.stationName || el.innerText; // przywróć oryginalny numer
    el.innerText = el.dataset.stationName || el.innerText;


    });

    document.querySelectorAll(".stationNumberList").forEach(el => {
      el.classList.remove("stationNumberListSelected");
      el.innerHTML = el.dataset.stationNumber || el.innerText; // przywróć oryginalny numer
    });

    // Zaznacz nową stację
    const numCells = document.querySelectorAll(".stationNumberList");
    const stationCells = document.querySelectorAll(".stationList");

    const numCell = numCells[stationId - 1];
    const stationCell = stationCells[stationId - 1];

    if (numCell && stationCell) 
    {
      numCell.classList.add("stationNumberListSelected");
      stationCell.classList.add("stationListSelected");
      
      // Pogrub nazwę stacji
      stationCell.dataset.stationName = stationCell.innerText; // zapisz oryginalny tekst
      stationCell.innerHTML = `<b>${stationCell.innerText}</b>`; // pogrubienie nazwy stacji

      //const original = stationCell.dataset.originalName || stationCell.innerText;
      //stationCell.dataset.originalName = original;
      //stationCell.innerText = original;
      //stationCell.style.fontWeight = "bold";

        
      // Pogrub numer
      numCell.dataset.stationNumber = numCell.innerText; // zapisz oryginalny numer
      numCell.innerHTML = `<b>${numCell.innerText}</b>`; 
      //const originalNum = numCell.dataset.originalNumber || numCell.innerText;
      //numCell.dataset.originalNumber = originalNum;
      //numCell.innerText = originalNum;
      //numCell.style.fontWeight = "bold";


    }
  }
  
  function flashBackground() 
	{
	  const div = document.getElementById('display');
	  const originalColor = div.style.backgroundColor;

	  const textSpan = document.getElementById('textStationName');
    const originalText = textSpan.innerText;


	  div.style.backgroundColor = '#115522'; // kolor flash
	  //textSpan.innerText = 'OLED Display Mode Changed';
    textSpan.innerHTML = '<b>Display Mode Changed</b>';  

	  setTimeout(() => 
	  {
		  div.style.backgroundColor = originalColor;
		  //textSpan.innerText = originalText;
      textSpan.innerHTML = `<b>${originalText}</b>`;
	  }, 150); // czas w ms flasha
	}


  document.addEventListener("DOMContentLoaded", function () 
  {
    connectWebSocket(); // podlaczamy websockety

    const slider = document.getElementById("volumeSlider");
    slider.addEventListener("wheel", function (event) 
    {
      event.preventDefault(); // zapobiega przewijaniu strony

      let currentValue = parseInt(slider.value);
      const step = parseInt(slider.step) || 1;
      const max = parseInt(slider.max);
      const min = parseInt(slider.min);

      if (event.deltaY < 0) {
        // przewijanie w górę (zwiększ)
        slider.value = Math.min(currentValue + step, max);
      } else {
        // przewijanie w dół (zmniejsz)
        slider.value = Math.max(currentValue - step, min);
      }

      updateSliderVolume(slider); // wywołaj aktualizację
    });
  });

  window.onpageshow = function(event) 
  {
    if (event.persisted) 
    {
      window.location.reload();
    }
  };

 </script>

)rawliteral";

const char list_html[] PROGMEM = R"rawliteral(<!DOCTYPE HTML><html><head><link rel='icon' href='/favicon.ico' type='image/x-icon'><link rel="shortcut icon" href="/favicon.ico" type="image/x-icon"><link rel="apple-touch-icon" sizes="180x180" href="/icon.png"><link rel="icon" type="image/png" sizes="192x192" href="/icon.png"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Evo Web Radio</title><style>html{font-family:Arial;display:inline-block;text-align:center}h2{font-size:1.3rem}p{font-size:1.1rem}table{border:1px solid black;border-collapse:collapse;margin:0}td,th{font-size:.8rem;border:1px solid gray;border-collapse:collapse}td:hover{font-weight:bold}a{color:black;text-decoration:none}body{max-width:1380px;margin:0 auto;padding-bottom:25px}.columnlist{align:center;padding:10px;display:flex;justify-content:center}</style></head>)rawliteral";

const char config_html[] PROGMEM = R"rawliteral(<!DOCTYPE HTML><html><head><link rel='icon' href='/favicon.ico' type='image/x-icon'><link rel="shortcut icon" href="/favicon.ico" type="image/x-icon"><link rel="apple-touch-icon" sizes="180x180" href="/icon.png"><link rel="icon" type="image/png" sizes="192x192" href="/icon.png"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Evo Web Radio</title><style>html{font-family:Arial;display:inline-block;text-align:center}h1{font-size:2.3rem}h2{font-size:1.3rem}table{border:1px solid black;border-collapse:collapse;margin:20px auto;width:80%}th,td{font-size:1rem;border:1px solid gray;padding:8px;text-align:left}td:hover{font-weight:bold}a{color:black;text-decoration:none}body{max-width:1380px;margin:0 auto;padding-bottom:25px}.tableSettings{border:2px solid #4CAF50;border-collapse:collapse;margin:10px auto;width:60%}input[type="checkbox"]{accent-color:#66BB6A;color:#FFFF50;width:13px;height:13px}@media (max-width:768px){.about-box,.tableSettings{width:90%!important}table{width:100%!important}th,td{font-size:.9rem}}</style></head><body><h2>Evo Web Radio - Settings</h2><form action="/configupdate" method="POST"><table class="tableSettings">
  <tr><th>Display Setting</th><th>Value</th></tr>
  <tr><td>Normal Display Brightness (0-255), default:180</td><td><input type="number" name="displayBrightness" min="1" max="255" value="%D1"></td></tr>
  <tr><td>Dimmed Display Brightness (0-200), default:20</td><td><input type="number" name="dimmerDisplayBrightness" min="1" max="200" value="%D2"></td></tr>
  <tr><td>Dimmed Display Brightness in Sleep Mode (0-50), default:1</td><td><input type="number" name="dimmerSleepDisplayBrightness" min="1" max="50" value="%D12"></td></tr>
  <tr><td>Auto Dimmer Delay Time (1-255 sec.), default:5</td><td><input type="number" name="displayAutoDimmerTime" min="1" max="255" value="%D3"></td></tr>
  <tr><td>Auto Dimmer, default:On</td><td><input type="checkbox" name="displayAutoDimmerOn" value="1" %S1_checked></td></tr>
  <tr><td>OLED Display Clock in Sleep Mode default:Off</td><td><input type="checkbox" name="f_displayPowerOffClock" value="1" %S19_checked></td></tr>
  <tr><td>OLED Display Power Save Mode, default:Off</td><td><input type="checkbox" name="displayPowerSaveEnabled" value="1" %S9_checked></td></tr>
  <tr><td>OLED Display Power Save Time (1-600sek.), default:20</td><td><input type="number" name="displayPowerSaveTime" min="1" max="600" value="%D9"></td></tr>
  <tr><td>OLED Display Mode: 0-Radio, 1-Clock, 2-Lines, 3-Minimal, 4-VU, 5-Analyzer, 6-Segments, 10-FloatingPeaks</td><td><input type="number" name="displayMode" min="0" max="10" value="%D6"></td></tr>
  <tr><th>Other Setting</th><th></th></tr>
  <tr><td>Time Voice Info Every Hour, default:On</td><td><input type="checkbox" name="timeVoiceInfoEveryHour" value="1" %S3_checked></td></tr>
  <tr><td>Encoder Function Order (0-1), 0-Volume, click for station list, 1-Station list, click for Volume</td><td><input type="number" name="encoderFunctionOrder" min="0" max="1" value="%D5"></td></tr>
  <tr><td>Radio Scroller & VU Meter Refresh Time (15-100ms), default:50</td><td><input type="number" name="scrollingRefresh" min="15" max="100" value="%D8"></td></tr>
  <tr><td>ADC Keyboard Enabled, default:Off</td><td><input type="checkbox" name="adcKeyboardEnabled" value="1" %S7_checked></td></tr>
  <tr><td>Volume Steps 1-21 [Off], 1-42 [On], default:Off</td><td><input type="checkbox" name="maxVolumeExt" value="1" %S11_checked></td></tr>
  <tr><td>Station Name Read From Stream [On-From Stream, Off-From Bank] EXPERIMENTAL</td><td><input type="checkbox" name="stationNameFromStream" value="1" %S17_checked></td></tr>
  <tr><td>Radio switch to Standby After Power Fail, default:On</td><td><input type="checkbox" name="f_sleepAfterPowerFail" value="1" %S21_checked></td></tr>
  <tr><td>Volume Fade on Station Change and Power Off, default:Off</td><td><input type="checkbox" name="f_volumeFadeOn" value="1" %S22_checked></td></tr>
  <tr><td>Save ALWAYS Station No., Bank No., Volume, default:Off</td><td><input type="checkbox" name="f_saveVolumeStationAlways" value="1" %S23_checked></td></tr>
  <tr><td>Power Off Animation, default:Off</td><td><input type="checkbox" name="f_powerOffAnimation" value="1" %S24_checked></td></tr>
  <tr><td>Presets On - klawisze 1-9 = stacje 1-9 z banku (0=stacja 10), default:Off</td><td><input type="checkbox" name="presetsOn" value="1" %S45_checked></td></tr>
  <tr><td>Speakers Enabled (GPIO18 HIGH=wzmacniacz ON, LOW=OFF przy mute/power off)</td><td><input type="checkbox" name="speakersEnabled" value="1" %S46_checked></td></tr>
  <tr><td>Timezone settings</td><td><button type="button" class="button" onclick="location.href='/timezone'">Set</button></td></tr>
  <tr><th><b>VU Meter Settings</b></th></tr>
  <tr><td>VU Meter Mode (0-1), 0-dashed lines, 1-continuous lines</td><td><input type="number" name="vuMeterMode" min="0" max="1" value="%D4"></td></tr>
  <tr><td>VU Meter Visible (Mode 0, 3 only), default:On</td><td><input type="checkbox" name="vuMeterOn" value="1" %S5_checked></td></tr>
  <tr><td>VU Meter Peak & Hold Function, default:On</td><td><input type="checkbox" name="vuPeakHoldOn" value="1" %S13_checked></td></tr>
  <tr><td>VU Meter Smooth Function, default:On</td><td><input type="checkbox" name="vuSmooth" value="1" %S15_checked></td></tr>
  <tr><td>VU Meter Smooth Rise Speed [1 low - 32 High], default:24</td><td><input type="number" name="vuRiseSpeed" min="1" max="32" value="%D10"></td></tr>
  <tr><td>VU Meter Smooth Fall Speed [1 low - 32 High], default:6</td><td><input type="number" name="vuFallSpeed" min="1" max="32" value="%D11"></td></tr>
  <tr><td>FFT / Analyzer for Styles 5,6,10, default:On</td><td><input type="checkbox" name="fftAnalyzerOn" value="1" %S25_checked></td></tr>
  <tr><th><b>Display Mode Selection (SRC Button)</b></th></tr>
  <tr><td colspan="2"><i>Modes 0-4,8 are always available. Enable optional modes below:</i></td></tr>
  <tr><td>Enable Display Mode 6 (Segments Analyzer) in SRC cycle</td><td><input type="checkbox" name="displayMode6Enabled" value="1" %S27_checked></td></tr>
  <tr><td>Enable Display Mode 8 (Bars Analyzer) in SRC cycle</td><td><input type="checkbox" name="displayMode8Enabled" value="1" %S29_checked></td></tr>
  <tr><td>Enable Display Mode 10 (Floating Peaks Analyzer) in SRC cycle</td><td><input type="checkbox" name="displayMode10Enabled" value="1" %S28_checked></td></tr>
  <tr><th><b>Analyzer Configuration</b></th></tr>
  <tr><td>Analyzer Preset: 0=Classic, 1=Modern, 2=Compact, 3=Retro, 4=Custom</td><td><input type="number" name="analyzerPreset" min="0" max="4" value="%A2"></td></tr>
  <tr><td>Analyzer Settings</td><td><button type="button" class="button" onclick="location.href='/analyzer'">Configure</button></td></tr>
  <tr><th><b>SD Player Style Selection (SRC Button)</b></th></tr>
  <tr><td colspan="2"><i>Select which SD Player styles should be available in SRC button cycle:</i></td></tr>
  <tr><td>Enable SD Player Style 1 (List with top bar)</td><td><input type="checkbox" name="sdPlayerStyle1" value="1" %S31_checked></td></tr>
  <tr><td>Enable SD Player Style 2 (Large track text)</td><td><input type="checkbox" name="sdPlayerStyle2" value="1" %S32_checked></td></tr>
  <tr><td>Enable SD Player Style 3 (VU meter + track)</td><td><input type="checkbox" name="sdPlayerStyle3" value="1" %S33_checked></td></tr>
  <tr><td>Enable SD Player Style 4 (Frequency spectrum)</td><td><input type="checkbox" name="sdPlayerStyle4" value="1" %S34_checked></td></tr>
  <tr><td>Enable SD Player Style 5 (Minimalist)</td><td><input type="checkbox" name="sdPlayerStyle5" value="1" %S35_checked></td></tr>
  <tr><td>Enable SD Player Style 6 (Album art simulation)</td><td><input type="checkbox" name="sdPlayerStyle6" value="1" %S36_checked></td></tr>
  <tr><td>Enable SD Player Style 7 (Retro analyzer triangles)</td><td><input type="checkbox" name="sdPlayerStyle7" value="1" %S37_checked></td></tr>
  <tr><td>Enable SD Player Style 9 (VU meters below scroll)</td><td><input type="checkbox" name="sdPlayerStyle9" value="1" %S38_checked></td></tr>
  <tr><td>Enable SD Player Style 10 (Fullscreen animation)</td><td><input type="checkbox" name="sdPlayerStyle10" value="1" %S39_checked></td></tr>
  <tr><td>Enable SD Player Style 11 (Based on Radio Mode 0)</td><td><input type="checkbox" name="sdPlayerStyle11" value="1" %S40_checked></td></tr>
  <tr><td>Enable SD Player Style 12 (Based on Radio Mode 1)</td><td><input type="checkbox" name="sdPlayerStyle12" value="1" %S41_checked></td></tr>
  <tr><td>Enable SD Player Style 13 (Based on Radio Mode 2)</td><td><input type="checkbox" name="sdPlayerStyle13" value="1" %S42_checked></td></tr>
  <tr><td>Enable SD Player Style 14 (Based on Radio Mode 3)</td><td><input type="checkbox" name="sdPlayerStyle14" value="1" %S43_checked></td></tr>
  <tr><th><b>Bluetooth Module (UART)</b></th></tr>
  <tr><td>Enable BT UART Module (RX=19, TX=20), default:Off</td><td><input type="checkbox" name="btModuleEnabled" value="1" %S26_checked></td></tr>
  <tr><th><b>DLNA Server Configuration</b></th></tr>
  <tr><td>DLNA Server IP Address (default: 192.168.1.100)</td><td><input type="text" name="dlnaHost" maxlength="45" value="%DLNA_HOST%" placeholder="192.168.1.100" style="width:150px"></td></tr>
  <tr><td>DLNA Root ObjectID (default: 0)</td><td><input type="text" name="dlnaIDX" maxlength="10" value="%DLNA_IDX%" placeholder="0" style="width:80px"></td></tr>
  </table><input type="submit" value="Update"></form><p style='font-size:.8rem'><a href='/menu'>Go Back</a></p></body></html>)rawliteral";

const char remoteconfig_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Evo Web Radio</title>
<link rel="icon" href="/favicon.ico" type="image/x-icon">
<link rel="shortcut icon" href="/favicon.ico" type="image/x-icon">
<link rel="apple-touch-icon" sizes="180x180" href="/icon.png">
<link rel="icon" type="image/png" sizes="192x192" href="/icon.png">
<style>
html{font-family:Arial;display:inline-block;text-align:center;}
body{max-width:1380px;margin:0 auto;padding-bottom:25px;}
h1{font-size:2.3rem;}h2{font-size:1.3rem;}
table{border:1px solid #000;border-collapse:collapse;margin:20px auto;width:40%;}
th,td{font-size:1rem;border:1px solid gray;padding:8px;text-align:left;}
td:hover{font-weight:bold;}
tr.selected{background:#d0f0d0;}
a{color:black;text-decoration:none;}
.tableSettings{border:2px solid #4CAF50;border-collapse:collapse;margin:5px auto;width:40%;}
input[type="checkbox"]{accent-color:#66BB6A;color:#FFFF50;width:13px;height:13px;}
input[type="text"]{width:70px;box-sizing:border-box;}
#wsBox{border:2px solid #4CAF50;padding:10px;margin:15px auto;width:38%;background:#f5fff5;font-size:1.1rem;}
#wsDetails{padding:5px;margin:5px auto;width:100%;background:#f5fff5;font-size:0.8rem;}
p{font-size:0.7rem;}
#AddrValue{margin-right: 10px;font-size:0.8rem;}
#CmdValue{margin-right: 10px;font-size:0.8rem;}
#Learn{background:#f5fff5;font-size:0.8rem;color:#4CAF50;font-weight:bold;}
@media(max-width:768px){.about-box,.tableSettings,#wsBox{width:90%!important;}table{width:100%!important;}th,td{font-size:.9rem;}}
</style>
</head>
<body>
<h2>Evo Web Radio - Remote Control Config</h2>
<div id="wsBox">
<strong>Received code:</strong> <span id="codeValue">—</span>
<div id="wsDetails">
<p>Address:<span id="AddrValue">—</span>  Command:<span id="CmdValue">—</span></p>
</div>
<div id="Learn"><span id="LearnIndicator"></span></div>
</div>
<p></p>
<div style="margin:10px;">
<button onclick="toggleRemoteConfig()">Learning Mode - ON/OFF</button>
<label><input type="checkbox" id="autoMode">AUTO Mode</label>&nbsp;&nbsp;
<button type="button" onclick="skipCurrent()">Skip Selected</button>
</div>
<p> Single Mouse Click on Row - Select, Double Mouse Click - Reset Value (set 0xFEFE)</p>
<p> Auto Mode, auto increment and select next cell after receiving IR code</p>
<form action="/remoteconfigupdate" method="POST">
<table class="tableSettings">
<tr><th>Codes:</th><th>Value</th></tr>
<tr><td>Volume Up +</td><td><input type="text" name="cmdVolumeUp" value="%D0" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Volume Down -</td><td><input type="text" name="cmdVolumeDown" value="%D1" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Arrow Right (Next Station / Next Bank)</td><td><input type="text" name="cmdArrowRight" value="%D2" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Arrow Left (Prev Station / Prev Bank)</td><td><input type="text" name="cmdArrowLeft" value="%D3" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Arrow Up (Station List Up)</td><td><input type="text" name="cmdArrowUp" value="%D4" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Arrow Down (Station List Down)</td><td><input type="text" name="cmdArrowDown" value="%D5" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Back</td><td><input type="text" name="cmdBack" value="%D6" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>OK / Enter</td><td><input type="text" name="cmdOk" value="%D7" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Display Screen Mode</td><td><input type="text" name="cmdSrc" value="%D8" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Mute</td><td><input type="text" name="cmdMute" value="%D9" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Equalizer</td><td><input type="text" name="cmdAud" value="%D10" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Display Brightness / Bank Network Update</td><td><input type="text" name="cmdDirect" value="%D11" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Bank Menu (-)</td><td><input type="text" name="cmdBankMinus" value="%D12" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Bank Menu (+)</td><td><input type="text" name="cmdBankPlus" value="%D13" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Power On / Off</td><td><input type="text" name="cmdRedPower" value="%D14" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Sleep</td><td><input type="text" name="cmdGreen" value="%D15" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Key 0</td><td><input type="text" name="cmdKey0" value="%D16" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Key 1</td><td><input type="text" name="cmdKey1" value="%D17" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Key 2</td><td><input type="text" name="cmdKey2" value="%D18" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Key 3</td><td><input type="text" name="cmdKey3" value="%D19" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Key 4</td><td><input type="text" name="cmdKey4" value="%D20" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Key 5</td><td><input type="text" name="cmdKey5" value="%D21" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Key 6</td><td><input type="text" name="cmdKey6" value="%D22" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Key 7</td><td><input type="text" name="cmdKey7" value="%D23" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Key 8</td><td><input type="text" name="cmdKey8" value="%D24" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
<tr><td>Key 9</td><td><input type="text" name="cmdKey9" value="%D25" pattern="0x[0-9A-Fa-f]{4}" maxlength="6" title="Format: 0xFFFF"></td></tr>
</table>
<input type="submit" value="Update">
</form>
<p style="font-size:.8rem;"><a href="/menu">Go Back</a></p>
<script>
let websocket;
let activeInput = null;
let inputs = [];
let currentIndex = -1;
let autoMode = false;
function toggleRemoteConfig() {
const xhr = new XMLHttpRequest();
xhr.open("POST", "/toggleRemoteConfig", true);
xhr.send();
}
function clearSelection() {
document.querySelectorAll(".tableSettings tr").forEach(r => r.classList.remove("selected"));
activeInput = null;
currentIndex = -1;
}
function selectInputByIndex(index) {
if (index >= inputs.length) {
autoMode = false;
document.getElementById("autoMode").checked = false;
clearSelection();
return;
}
clearSelection();
currentIndex = index;
let inputElement = inputs[index];
activeInput = inputElement;
let row = inputElement.closest("tr");
if (row) row.classList.add("selected");
}
function skipCurrent() {
if (autoMode && currentIndex >= 0) {
selectInputByIndex(currentIndex + 1);
}
}
window.addEventListener('load', function() {
inputs = Array.from(document.querySelectorAll('input[type="text"]'));
document.querySelectorAll(".tableSettings tr").forEach((row, idx) => {
if (idx === 0) return;
row.addEventListener("click", function() {
clearSelection();
let inputElement = this.querySelector('input[type="text"]');
if (inputElement) {
currentIndex = inputs.indexOf(inputElement);
activeInput = inputElement;
this.classList.add("selected");
}
});
row.addEventListener("dblclick", function() {
let inputElement = this.querySelector('input[type="text"]');
if (inputElement) {
inputElement.value = "0xFEFE";
}
});
});
document.getElementById("autoMode").addEventListener("change", function() {
autoMode = this.checked;
if (autoMode && currentIndex < 0) {
selectInputByIndex(0);
}
});
initWebSocket();
});
function initWebSocket() {
websocket = new WebSocket('ws://' + window.location.hostname + '/ws');
websocket.onopen = function(event) {
console.log('WebSocket connection opened');
};
websocket.onclose = function(event) {
console.log('WebSocket connection closed');
setTimeout(initWebSocket, 2000);
};
websocket.onerror = function(error) {
console.log('WebSocket Error: ' + error);
};
websocket.onmessage = function(event) {
if (event.data.startsWith("ircode:")) {
let code = event.data.substring(7);
document.getElementById("codeValue").textContent = code;
if (activeInput && autoMode) {
activeInput.value = code;
if (currentIndex >= 0) {
selectInputByIndex(currentIndex + 1);
}
}
} else if (event.data.startsWith("addr:")) {
let addr = event.data.substring(5);
document.getElementById("AddrValue").textContent = addr;
} else if (event.data.startsWith("cmd:")) {
let cmd = event.data.substring(4);
document.getElementById("CmdValue").textContent = cmd;
} else if (event.data.startsWith("learn:")) {
let status = event.data.substring(6);
document.getElementById("LearnIndicator").textContent = (status === "1") ? "Learning ON" : "Learning OFF";
} else if (event.data === "clear") {
document.getElementById("codeValue").textContent = "—";
document.getElementById("AddrValue").textContent = "—";
document.getElementById("CmdValue").textContent = "—";
}
};
}
</script>
</body>
</html>
)rawliteral";

const char adc_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
  <head>
    <link rel='icon' href='/favicon.ico' type='image/x-icon'>
    <link rel="shortcut icon" href="/favicon.ico" type="image/x-icon">
    <link rel="apple-touch-icon" sizes="180x180" href="/icon.png">
    <link rel="icon" type="image/png" sizes="192x192" href="/icon.png">

    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Evo Web Radio</title>
    <style>
      html {font-family: Arial; display: inline-block; text-align: center;}
      h1 {font-size: 2.3rem;}
      h2 {font-size: 1.3rem;}
      table {border: 1px solid black; border-collapse: collapse; margin: 10px auto; width: 40%;}
      th, td {font-size: 1rem; border: 1px solid gray; padding: 8px; text-align: left;}
      td:hover {font-weight: bold;}
      a {color: black; text-decoration: none;}
      body {max-width: 1380px; margin:0 auto; padding-bottom: 15px;}
      .tableSettings {border: 2px solid #4CAF50; border-collapse: collapse; margin: 10px auto; width: 40%;}
      .button { background-color: #4CAF50; border: 1; color: white; padding: 10px 20px; border-radius: 5px; width:200px;}
      .button:hover {background-color: #4a4a4a;}
      .button:active {background-color: #4a4a4a; box-shadow: 0 4px #666; transform: translateY(2px);}
    </style>
    </head>

  <body>
  <h2>Evo Web Radio - ADC Settings</h2>
  <form action="/configadc" method="POST">
  <table class="tableSettings">
  <tr><th>Button</th><th>Value</th></tr>
  <tr><td>keyboardButtonThreshold_0</td><td><input type="number" name="keyboardButtonThreshold_0" min="0" max="4095" value="%D0"></td></tr>
  <tr><td>keyboardButtonThreshold_1</td><td><input type="number" name="keyboardButtonThreshold_1" min="0" max="4095" value="%D1"></td></tr>
  <tr><td>keyboardButtonThreshold_2</td><td><input type="number" name="keyboardButtonThreshold_2" min="0" max="4095" value="%D2"></td></tr>
  <tr><td>keyboardButtonThreshold_3</td><td><input type="number" name="keyboardButtonThreshold_3" min="0" max="4095" value="%D3"></td></tr>
  <tr><td>keyboardButtonThreshold_4</td><td><input type="number" name="keyboardButtonThreshold_4" min="0" max="4095" value="%D4"></td></tr>
  <tr><td>keyboardButtonThreshold_5</td><td><input type="number" name="keyboardButtonThreshold_5" min="0" max="4095" value="%D5"></td></tr>
  <tr><td>keyboardButtonThreshold_6</td><td><input type="number" name="keyboardButtonThreshold_6" min="0" max="4095" value="%D6"></td></tr>
  <tr><td>keyboardButtonThreshold_7</td><td><input type="number" name="keyboardButtonThreshold_7" min="0" max="4095" value="%D7"></td></tr>
  <tr><td>keyboardButtonThreshold_8</td><td><input type="number" name="keyboardButtonThreshold_8" min="0" max="4095" value="%D8"></td></tr>
  <tr><td>keyboardButtonThreshold_9</td><td><input type="number" name="keyboardButtonThreshold_9" min="0" max="4095" value="%D9"></td></tr>
  <tr><td>keyboardButtonThreshold_Shift - Ok/Enter</td><td><input type="number" name="keyboardButtonThreshold_Shift" min="0" max="4095" value="%D10"></td></tr>
  <tr><td>keyboardButtonThreshold_Memory - Bank Menu</td><td><input type="number" name="keyboardButtonThreshold_Memory" min="0" max="4095" value="%D11"></td></tr>
  <tr><td>keyboardButtonThreshold_Band -  Back</td><td><input type="number" name="keyboardButtonThreshold_Band" min="0" max="4095" value="%D12"></td></tr>
  <tr><td>keyboardButtonThreshold_Auto -  Display Mode</td><td><input type="number" name="keyboardButtonThreshold_Auto" min="0" max="4095" value="%D13"></td></tr>
  <tr><td>keyboardButtonThreshold_Scan - Dimmer/Network Bank Update</td><td><input type="number" name="keyboardButtonThreshold_Scan" min="0" max="4095" value="%D14"></td></tr>
  <tr><td>keyboardButtonThreshold_Mute - Mute</td><td><input type="number" name="keyboardButtonThreshold_Mute" min="0" max="4095" value="%D15"></td></tr>
  <tr><td>keyboardButtonThresholdTolerance</td><td><input type="number" name="keyboardButtonThresholdTolerance" min="0" max="50" value="%D16"></td></tr>
  <tr><td>keyboardButtonNeutral</td><td><input type="number" name="keyboardButtonNeutral" min="0" max="4095" value="%D17"></td></tr>
  <tr><td>keyboardSampleDelay (30-300ms)</td><td><input type="number" name="keyboardSampleDelay" min="30" max="300" value="%D18"></td></tr>
  </table>
  <input type="submit" value="ADC Thresholds Update">
  </form>
  <br>
  <button onclick="toggleAdcDebug()">ADC Debug ON/OFF</button>
  <script>
  function toggleAdcDebug() {
      const xhr = new XMLHttpRequest();
      xhr.open("POST", "/toggleAdcDebug", true);
      xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
      xhr.onload = function() {
          if (xhr.status === 200) {
              alert("ADC Debug is now " + (xhr.responseText === "1" ? "ON" : "OFF"));
          } else {
              alert("Error: " + xhr.statusText);
          }
      };
      xhr.send(); // Wysyłanie pustego zapytania POST
  }
  </script>


  <p style='font-size: 0.8rem;'><a href='/menu'>Go Back</a></p>
  </body>
  </html>
)rawliteral";

const char menu_html[] PROGMEM = R"rawliteral(<!DOCTYPE HTML><html><head><link rel='icon' href='/favicon.ico' type='image/x-icon'><link rel="shortcut icon" href="/favicon.ico" type="image/x-icon"><link rel="apple-touch-icon" sizes="180x180" href="/icon.png"><link rel="icon" type="image/png" sizes="192x192" href="/icon.png"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Evo Web Radio</title><style>html{font-family:Arial;display:inline-block;text-align:center}h2{font-size:1.3rem}p{font-size:1.1rem}a{color:black;text-decoration:none}body{max-width:1380px;margin:0 auto;padding-bottom:25px}.button{background-color:#4CAF50;border:1;color:white;padding:10px 20px;border-radius:5px;width:200px}.button:hover{background-color:#4a4a4a}.button:active{background-color:#4a4a4a;box-shadow:0 4px #666;transform:translateY(2px)}</style></head><body><h2>Evo Web Radio - Menu</h2><br><button class="button" onclick="location.href='/info'">Info</button><br><br><button class="button" onclick="location.href='/ota'">OTA Update</button><br><br><button class="button" onclick="location.href='/sdplayer'">SD Player</button><br><br><button class="button" onclick="location.href='/dlna'">DLNA Browser</button><br><br><button class="button" onclick="location.href='/analyzer'">Analyzer Edit</button><br><br><button class="button" onclick="location.href='/adc'">ADC Keyboard Settings</button><br><br><button class="button" onclick="location.href='/list'">SD / SPIFFS Explorer</button><br><br><button class="button" onclick="location.href='/editor'">Memory Bank Editor</button><br><br><button class="button" onclick="location.href='/browser'">Radio Browser API</button><br><br><button class="button" onclick="location.href='/playurl'">Play from URL</button><br><br><button class="button" onclick="location.href='/bt'">Bluetooth Settings</button><br><br><button class="button" onclick="location.href='/remoteconfig'">Remote Control</button><br><br><button class="button" onclick="location.href='/config'">Settings</button><br><br><p style='font-size:.8rem'><a href="#" onclick="window.location.replace('/')">Go Back</a></p></body></html>)rawliteral";

const char info_html[] PROGMEM = R"rawliteral(
 <!DOCTYPE HTML>
  <html>
  <head>
    <link rel='icon' href='/favicon.ico' type='image/x-icon'>
    <link rel="shortcut icon" href="/favicon.ico" type="image/x-icon">
    <link rel="apple-touch-icon" sizes="180x180" href="/icon.png">
    <link rel="icon" type="image/png" sizes="192x192" href="/icon.png">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Evo Web Radio</title>
    <style>
      html{font-family:Arial;display:inline-block;text-align:center;} 
      h2{font-size:1.3rem;} 
      table{border:1px solid black;border-collapse:collapse;margin:10px auto;width:40%;} 
      th,td{font-size:1rem;border:1px solid gray;padding:8px;text-align:left;} 
      td:hover{font-weight:bold;} 
      a{color:black;text-decoration:none;} 
      body{max-width:1380px;margin:0 auto;padding-bottom:15px;} 
      .tableSettings{border:2px solid #4CAF50;border-collapse:collapse;margin:10px auto;width:40%;} 
      .signal-bars{display:inline-block;vertical-align:middle;margin-left:10px;} 
      .bar{display:inline-block;width:5px;margin-right:2px;background-color:#F2F2F2;height:10px;} 
      .bar.active{background-color:#4CAF50;} 
      .about-box{border:2px solid #4CAF50;border-collapse:collapse;margin:10px auto;width:40%;text-align:left;font-size:0.9rem;line-height:1.4rem;color:#222;} 
      .about-box h3{color:#2f7a2f;margin-top:0;text-align:center;font-size:1.05rem;} 
      .about-box a{color:#2b5bb3;text-decoration:none;} 
      .about-box a:hover{text-decoration:underline;}
      @media (max-width:768px){.about-box,.tableSettings{width:90%!important;} table{width:100%!important;} th,td{font-size:0.9rem;}}
    </style>
  </head>
  <body>
  <h2>Evo Web Radio - Info</h2>

  <div class="about-box">
    <h3>Project Info</h3>
    <p>This is a project of an Internet radio streamer called <strong>"Evo"</strong>. The hardware was built using an <strong>ESP32-S3</strong> microcontroller and a <strong>PCM5102A DAC codec</strong>.</p>
    <p>The design allows listening to various music stations from all around the world and works properly with streams encoded in <strong>MP3</strong>, <strong>AAC</strong>, <strong>VORBIS</strong>, <strong>OPUS</strong>, and <strong>FLAC</strong> (up to 2000kbs, 24bit).</p>
    <p>All operations (volume control, station change, memory bank selection, power on/off) are handled via a single rotary encoder. It also supports infrared remote controls working on the <strong>NEC standard (38&nbsp;kHz)</strong>.</p>
    <p></p>
    <p>Source code and documentation are available at: <b><a href="https://github.com/dzikakuna/ESP32_radio_evo3" target="_blank">Link: https://github.com/dzikakuna/ESP32_radio_evo3</a></b></p>
    <p>Robgold 2026, Made in Poland</p>
  </div>

  <form action="/configadc" method="POST">
    <table class="tableSettings">
      <tr><td>ESP Serial Number:</td><td><input name="espSerial" value="%D0"></td></tr>
      <tr><td>Firmware Version:</td><td><input name="espFw" value="%D1"></td></tr>
      <tr><td>Hostname:</td><td><input name="hostnameValue" value="%D2"></td></tr>
      <tr><td>WiFi Signal Strength:</td><td><input id="wifiSignal" value="%D3"> dBm
        <div class="signal-bars" id="signalBars">
          <div class="bar" style="height:2px;"></div>
          <div class="bar" style="height:6px;"></div>
          <div class="bar" style="height:10px;"></div>
          <div class="bar" style="height:14px;"></div>
          <div class="bar" style="height:18px;"></div>
          <div class="bar" style="height:22px;"></div>
        </div>
      </td></tr>
      <tr><td>WiFi SSID:</td><td><input name="wifiSsid" value="%D4"></td></tr>
      <tr><td>IP Address:</td><td><input name="ipValue" value="%D5"></td></tr>
      <tr><td>MAC Address:</td><td><input name="macValue" value="%D6"></td></tr>
      <tr><td>Memory type:</td><td><input name="memValue" value="%D7"></td></tr>
    </table>
  </form>

  <br>
  <p style="font-size:0.8rem;"><a href="/menu">Go Back</a></p>

  <script>
    function updateSignalBars(signal){const bars=document.querySelectorAll('#signalBars .bar');
    let level=0;signal=parseInt(signal);
    if(signal>=-50)level=6;
    else if(signal>=-57)level=5;
    else if(signal>=-66)level=4;
    else if(signal>=-74)level=3;
    else if(signal>=-81)level=2;
    else if(signal>=-88)level=1;
    else level=0;
    bars.forEach((bar,index)=>{if(index<level){bar.classList.add('active');}
    else
    {bar.classList.remove('active');}});} 
    const signalInput=document.getElementById('wifiSignal');
    updateSignalBars(signalInput.value);
  </script>
</body>
</html>
)rawliteral";

const char urlplay_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
    <head>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <title>Evo Web Radio</title>

      <style>
          html {font-family: Arial; text-align: center;}
          body {max-width: 1380px; margin: 0 auto; background: #D0D0D0; padding-bottom: 25px;}
          h2 {font-size: 1.3rem;}
          p {font-size: 0.95rem;}
          .slider{-webkit-appearance:none;margin:14px;width:330px;height:10px;background:#4CAF50;outline:none;-webkit-transition:.2s;transition:opacity .2s;border-radius:5px}
          .slider::-webkit-slider-thumb{-webkit-appearance:none;appearance:none;width:35px;height:25px;background:#4a4a4a;cursor:pointer;border-radius:5px}
          .slider::-moz-range-thumb{width:35px;height:35px;background:#4a4a4a;cursor:pointer;border-radius:5px}
          .button {background-color:#4CAF50;border:1; color:white; padding:10px 20px; border-radius:5px;cursor:pointer;}
          .button:hover {background-color:#4a4a4a;}
          .button.active {background-color: #115522; transform: scale(1.05); transition: all 0.15s ease;}
          #urlInput {width:350px; padding:5px; border-radius:5px; font-size:0.9rem;}
          #playBtn {padding:5px 10px; font-size:0.9rem;}
          #display {display:inline-block; padding:5px; border:2px solid #4CAF50; border-radius:15px; background-color:#4a4a4a; font-size:1.45rem; color:#AAA; width:345px; text-align:center; white-space:nowrap; box-shadow:0 0 20px #4CAF50;}
          #stationTextDiv {display:block; text-overflow:ellipsis; white-space:normal; font-size:1.0rem; color:#999; margin-bottom:10px; text-align:center; height:4.2em; overflow:hidden; line-height:1.4em;}
          #muteIndicatorDiv {text-align:center; color:#4CAF50; font-weight:bold; font-size:1.0rem; height:1.4em;}
          #urlInput.flash {background-color: #4CAF50;color: #fff; transition: background-color 0.3s ease;}
        .wifi-wrapper {display: flex; align-items: flex-end; gap: 1px; height: 13px; margin-top: -3px; margin-bottom: 3px; justify-content: center; }
        .wifiBar {width: 2px;background-color: #555; border-radius: 2px; transition: background-color 0.2s;}
        .bar1 { height: 2px; }
        .bar2 { height: 4px; }
        .bar3 { height: 6px; }
        .bar4 { height: 8px; }
        .bar5 { height: 10px; }
        .bar6 { height: 12px; }
      </style>
  </head>

  <body>
  <h2>Evo Web Radio - URL Play</h2>

  <div id="display" onclick="flashBackground(); displayMode();">
      <div style="margin-bottom:10px; font-weight:bold; overflow:hidden; text-overflow:ellipsis;">
          <span id="textStationName"><b>STATIONNAME</b></span>
      </div>

      <div style="width:345px; margin-bottom:10px;">
          <div id="stationTextDiv">
              <span id="stationText">STATIONTEXT</span>
          </div>
          <div id="muteIndicatorDiv">
              <span id="muteIndicator"></span>
          </div>
      </div>

      <div style="height:1px; background-color:#4CAF50; margin:5px 0;"></div>

      <div style="display:flex; justify-content:center; gap:13px; font-size:1.0rem; color:#999;">
          <div><span id="bankValue">Bank: --</span></div>
            <div style="display:flex; gap:6px; font-size:0.55rem; color:#999;margin:4px 0;">
              <div><span id="samplerate">---.-kHz</span></div>
              <div><span id="bitrate">---kbps</span></div>
              <div><span id="bitpersample">---bit</span></div>
              <div><span id="streamformat">----</span></div>
              WiFi: <div class="wifi-wrapper">
              <div class="wifiBar bar1" id="wifiBar1"></div>
              <div class="wifiBar bar2" id="wifiBar2"></div>
              <div class="wifiBar bar3" id="wifiBar3"></div>
              <div class="wifiBar bar4" id="wifiBar4"></div>
              <div class="wifiBar bar5" id="wifiBar5"></div>
              <div class="wifiBar bar6" id="wifiBar6"></div>
  	        </div>
          </div>
          <div><span id="stationNumber">Station: --</span></div>
      </div>
  </div>

  <br><br>

  <!-- URL + PLAY -->
  <p><input id="urlInput" type="text" placeholder="Put your URL here"></p>
  <p><button id="playBtn" class="button" onclick="playStream()">PLAY</button></p>

  <br>

  <!-- Volume -->
  <p>Volume: <span id="textSliderValue">--</span></p>
  <p><input type="range" onchange="updateSliderVolume()" id="volumeSlider" min="1" max="21" value="1" step="1" class="slider"></p>

  <p style='font-size: 0.8rem;'><a href="#" onclick="window.location.replace('/menu')">Go Back</a></p>

  <script>
  var websocket;

  function setWifi(level) 
  {
    for (let i = 1; i <= 6; i++) 
    {
      const bar = document.getElementById("wifiBar" + i);
      if (bar) {bar.style.backgroundColor = (i <= level) ? "#4CAF50" : "#555";}
    }
  }


  function connectWebSocket() 
  {
      websocket = new WebSocket('ws://' + window.location.hostname + '/ws');

      websocket.onopen = function() {
          console.log("WebSocket połączony");
      };

      websocket.onclose = function() {
          console.log("WebSocket zamknięty, reconnect za 3s...");
          setTimeout(connectWebSocket, 3000);
      };

      websocket.onerror = function(err) {
          console.error("Błąd WebSocket:", err);
          websocket.close();
      };

      websocket.onmessage = function(event) {
          const data = event.data;

          if (data.startsWith("volume:")) {
              const vol = parseInt(data.split(":")[1]);
              document.getElementById("volumeSlider").value = vol;
              document.getElementById("textSliderValue").innerText = vol;
          }

          if (data.startsWith("stationname:")) {
              const value = data.split(":")[1];
              document.getElementById("textStationName").innerHTML = "<b>" + value + "</b>";
          }

          if (data.startsWith("stationtext$")) {
              const text = data.substring(data.indexOf("$") + 1);
              document.getElementById("stationText").textContent = text;
          }

          if (data.startsWith("station:")) {
              const station = parseInt(data.split(":")[1]);
              document.getElementById("stationNumber").innerText = "Station: " + station;
          }

          if (data.startsWith("bank:")) {
              const bank = parseInt(data.split(":")[1]);
              document.getElementById("bankValue").innerText = "Bank: " + bank;
          }

          if (data.startsWith("samplerate:")) {
              const rate = parseInt(data.split(":")[1]);
              document.getElementById("samplerate").innerText = (rate / 1000).toFixed(1) + "kHz";
          }

          if (data.startsWith("bitrate:")) {
              document.getElementById("bitrate").innerText = data.split(":")[1] + "kbps";
          }

          if (data.startsWith("bitpersample:")) {
              document.getElementById("bitpersample").innerText = data.split(":")[1] + "bit";
          }

          if (data.startsWith("streamformat:")) {
              document.getElementById("streamformat").innerText = data.split(":")[1];
          }

          if (data.startsWith("mute:")) {
              const mute = parseInt(data.split(":")[1]);
              document.getElementById("muteIndicator").innerText = mute === 1 ? "MUTED" : "";
          }

          if (event.data.startsWith("wifi:")) {
            const level = parseInt(event.data.split(":")[1]);
            if (!isNaN(level)) {setWifi(Math.min(Math.max(level, 1), 6));}
          }
      };
  }

  function updateSliderVolume() {
      const slider = document.getElementById("volumeSlider");
      const value = slider.value;
      document.getElementById("textSliderValue").innerText = value;

      if (websocket && websocket.readyState === WebSocket.OPEN) {
          websocket.send("volume:" + value);
      }
  }

  function playStream() {
      const urlInput = document.getElementById("urlInput");
      const url = urlInput.value.trim();
      const host = window.location.hostname;

      urlInput.classList.add("flash");
      setTimeout(() => urlInput.classList.remove("flash"), 300);

      if (!url) return;

      fetch("http://" + host + "/update?url=" + encodeURIComponent(url))
          .catch(err => console.error("Błąd połączenia:", err));
  }

  function flashBackground() {
      const div = document.getElementById("display");
      const text = document.getElementById("textStationName");
      const origColor = div.style.backgroundColor;
      const origText = text.innerHTML;

      div.style.backgroundColor = "#115522";
      text.innerHTML = "<b>Display Mode Changed</b>";

      setTimeout(() => {
          div.style.backgroundColor = origColor;
          text.innerHTML = origText;
      }, 150);
  }

  function displayMode() {
      if (websocket && websocket.readyState === WebSocket.OPEN) {
          websocket.send("displayMode");
      }
  }

  document.addEventListener("DOMContentLoaded", () => {
      connectWebSocket();
  });
  </script>
  </body>
</html>
)rawliteral";

const char timezone_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
  <meta charset="UTF-8">
  <title>Timezone Settings</title>
  <style>
  body { font-family: Arial; background: #D0D0D0; text-align: center; padding: 20px; }
  .tzField { width: 380px; padding: 8px; border-radius: 6px; border: 2px solid #4CAF50; font-size: 1rem; margin-bottom: 10px; text-align: left; }
  input#tzCustom { width: 380px; display: block; margin: 10px auto; text-align: center; }
  button { padding: 10px 20px; background: #4CAF50; color: white; border: 1; border-radius: 6px; cursor: pointer; font-size: 1rem; }
  button:hover { background: #3e8e41;}
  #msg { margin-top: 15px; font-weight: bold; }
  </style>
  </head>
  <body>

  <h2>Evo Web Radio - Timezone Set</h2>
  <p>Current timezone:
  <b id="currentTZ">Loading...</b></p>

  <p>Select timezone from list</p>

  <select id="tzSelect" class="tzField" onchange="selectChanged()">
      <option value="custom">Custom...</option>

      <!-- Europe -->
      <optgroup label="Europe">
          <option value="UTC0">UTC / GMT - UTC0</option>
          <option value="GMT0BST,M3.5.0/1,M10.5.0/2">United Kingdom - GMT0BST,M3.5.0/1,M10.5.0/2</option>
          <option value="CET-1CEST,M3.5.0/2,M10.5.0/3">Central Europe - CET-1CEST,M3.5.0/2,M10.5.0/3</option>
          <option value="EET-2EEST,M3.5.0/3,M10.5.0/4">Eastern Europe - EET-2EEST,M3.5.0/3,M10.5.0/4</option>
          <option value="GMT-1">Azores - GMT-1</option>
          <option value="WET0WEST,M3.5.0/1,M10.5.0/2">Portugal - WET0WEST,M3.5.0/1,M10.5.0/2</option>
          <option value="MSK-3">Moscow - MSK-3</option>
      </optgroup>

      <!-- North America -->
      <optgroup label="North America">
          <option value="EST5EDT,M3.2.0/2,M11.1.0/2">US Eastern - EST5EDT,M3.2.0/2,M11.1.0/2</option>
          <option value="CST6CDT,M3.2.0/2,M11.1.0/2">US Central - CST6CDT,M3.2.0/2,M11.1.0/2</option>
          <option value="MST7MDT,M3.2.0/2,M11.1.0/2">US Mountain - MST7MDT,M3.2.0/2,M11.1.0/2</option>
          <option value="PST8PDT,M3.2.0/2,M11.1.0/2">US Pacific - PST8PDT,M3.2.0/2,M11.1.0/2</option>
          <option value="AKST9AKDT,M3.2.0/2,M11.1.0/2">Alaska - AKST9AKDT,M3.2.0/2,M11.1.0/2</option>
          <option value="HST10">Hawaii - HST10</option>
      </optgroup>

      <!-- South America -->
      <optgroup label="South America">
          <option value="ART-3">Argentina - ART-3</option>
          <option value="BRT-3">Brazil - BRT-3</option>
          <option value="CLT-4CLST,M9.1.0/0,M4.1.0/0">Chile - CLT-4CLST,M9.1.0/0,M4.1.0/0</option>
      </optgroup>

      <!-- Asia -->
      <optgroup label="Asia">
          <option value="IST-5.5">India - IST-5.5</option>
          <option value="CST-8">China - CST-8</option>
          <option value="JST-9">Japan - JST-9</option>
          <option value="KST-9">Korea - KST-9</option>
          <option value="SGT-8">Singapore - SGT-8</option>
          <option value="WIB-7">Indonesia (WIB) - WIB-7</option>
          <option value="ICT-7">Thailand - ICT-7</option>
      </optgroup>

      <!-- Australia / Oceania -->
      <optgroup label="Australia">
          <option value="AEST-10AEDT,M10.1.0/2,M4.1.0/3">Australia Eastern - AEST-10AEDT,M10.1.0/2,M4.1.0/3</option>
          <option value="ACST-9.5ACDT,M10.1.0/2,M4.1.0/3">Australia Central - ACST-9.5ACDT,M10.1.0/2,M4.1.0/3</option>
          <option value="AWST-8">Australia Western - AWST-8</option>
      </optgroup>

      <!-- Africa -->
      <optgroup label="Africa">
          <option value="CAT-2">Central Africa - CAT-2</option>
          <option value="EAT-3">East Africa - EAT-3</option>
          <option value="WAT-1">West Africa - WAT-1</option>
          <option value="SAST-2">South Africa - SAST-2</option>
      </optgroup>
  </select>

  <input id="tzCustom" class="tzField" type="text" placeholder="Enter custom Timezone string..." readonly>

  <p id="msg"></p>
  <br>
  <button onclick="saveTZ()">Save</button>

  <p style="font-size: 0.8rem;"><a href="/menu">Go Back</a></p>

  <script>
  function loadTZ() {
      fetch("/getTZ").then(r => r.text()).then(tz => {
          document.getElementById("currentTZ").textContent = tz;
          let select = document.getElementById("tzSelect");
          let input = document.getElementById("tzCustom");
          let found = false;

          for (let i = 0; i < select.options.length; i++) {
              if (select.options[i].value === tz) {
                  select.selectedIndex = i;
                  input.value = tz;
                  input.readOnly = true;
                  input.style.display = "block";
                  found = true;
                  break;
              }
          }

          if (!found) {
              select.value = "custom";
              input.value = tz;
              input.readOnly = false;
              input.style.display = "block";
          }
      });
  }

  function selectChanged() {
      let select = document.getElementById("tzSelect");
      let input = document.getElementById("tzCustom");

      if (select.value === "custom") {
          input.readOnly = false;
          input.value = "";
          input.style.display = "block";
      } else {
          input.value = select.value;
          input.readOnly = true;
          input.style.display = "block";
      }
  }

  function saveTZ() {
      let input = document.getElementById("tzCustom");
      let tzToSave = input.value.trim();

      if (!tzToSave) {
          document.getElementById("msg").style.color = "red";
          document.getElementById("msg").textContent = "Please enter a timezone!";
          return;
      }

      fetch("/setTZ?value=" + encodeURIComponent(tzToSave))
          .then(r => r.text())
          .then(() => {
              document.getElementById("msg").style.color = "green";
              document.getElementById("msg").textContent = "Timezone saved and set.";
              loadTZ();
          })
          .catch(err => {
              document.getElementById("msg").style.color = "red";
              document.getElementById("msg").textContent = "Error: " + err;
          });
  }

  loadTZ();
  </script>

  </body>
  </html>
)rawliteral";

const char ota_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE html>
  <html lang='en'>
  <head>
  <meta charset='UTF-8' />
  <meta name='viewport' content='width=device-width, initial-scale=1.0'/>
  <title>ESP OTA Update</title>
  <style>
  body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: #f3f4f6; color: #333; padding: 40px; text-align: center; }
  h2 { margin-bottom: 30px; color: #111; font-size: 1.3rem;}
  #uploadSection { background-color: white; padding: 30px; border-radius: 8px; display: inline-block; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); }
  input[type='file'] { padding: 10px; }
  button { margin-top: 20px; padding: 10px 20px; background-color: #4CAF50; border: none; color: white; font-size: 16px; border-radius: 5px; cursor: pointer; }
  button:disabled { background-color: #ccc; cursor: not-allowed; }
  progress { width: 100%; height: 20px; margin-top: 20px; border-radius: 5px; appearance: none; -webkit-appearance: none; }
  progress::-webkit-progress-bar { background-color: #eee; border-radius: 5px; }
  progress::-webkit-progress-value { background-color: #4CAF50; border-radius: 5px; }
  progress::-moz-progress-bar { background-color: #4CAF50; border-radius: 5px; }
  #status { margin-top: 15px; font-weight: bold; }
  #fileInfo { color: #555; margin-top: 10px; }
  </style>
  </head>
  <body>
  <div id='uploadSection'>
  <h2>Evo Web Radio - OTA Firmware Update</h2>
  <input type='file' id='fileInput' name='update' /><br />
  <div id='fileInfo'>No file selected</div>
  <button id='uploadBtn'>Upload</button>
  <p id='status'></p>
  </div>

  <script>
  const fileInput = document.getElementById('fileInput');
  const uploadBtn = document.getElementById('uploadBtn');
  const status = document.getElementById('status');
  const fileInfo = document.getElementById('fileInfo');

  fileInput.addEventListener('change', function () {
    const file = this.files[0];
    if (file) {
      fileInfo.textContent = `Selected: ${file.name} (${(file.size / 1024).toFixed(2)} KB)`;
    } else {
      fileInfo.textContent = 'No file selected';
    }
  });

  uploadBtn.addEventListener('click', function () {
    const file = fileInput.files[0];
    if (!file) { alert('Please select a file first.'); return; }
    uploadBtn.disabled = true;
    status.textContent = 'Uploading...';

    const xhr = new XMLHttpRequest();
    const formData = new FormData();
    formData.append('update', file);

    xhr.open('POST', '/firmwareota', true);

    xhr.onload = function () {
      if (xhr.status === 200) {
        status.textContent = '✅ Upload completed, reboot in 3 sec.';
        setTimeout(() => { window.location.href = '/'; }, 10000);
      } else {
        status.textContent = '❌ Upload failed.';
      }
      uploadBtn.disabled = false;
    };

    xhr.onerror = function () {
      status.textContent = '❌ Network error.';
      uploadBtn.disabled = false;
    };

    xhr.send(formData);
  });
  </script>

  </body>
  </html>
)rawliteral";



char stations[MAX_STATIONS][STATION_NAME_LENGTH + 1];  // Tablica przechowująca linki do stacji radiowych (jedna na stację) +1 dla terminatora null

const char *ntpServer1 = "pool.ntp.org";  // Adres serwera NTP używany do synchronizacji czasu
const char *ntpServer2 = "time.nist.gov"; // Adres serwera NTP używany do synchronizacji czasu
//const long gmtOffset_sec = 3600;          // Przesunięcie czasu UTC w sekundach
//const int daylightOffset_sec = 3600;      // Przesunięcie czasu letniego w sekundach, dla Polski to 1 godzina

// ---- DLNA Configuration ---- //
#ifdef USE_DLNA
String dlnaIDX = "0";                      // Root ObjectID DLNA (domyślnie "0" dla Music)
String dlnaHost = "192.168.1.100";         // Adres IP serwera DLNA (można zmienić w Settings)
#endif

const int LOGO_SIZE = (SCREEN_WIDTH * SCREEN_HEIGHT) / 8;
uint8_t logo_bits[LOGO_SIZE];

uint8_t spleen6x12PL[2958] U8G2_FONT_SECTION("spleen6x12PL") =
  "\340\1\3\2\3\4\1\3\4\6\14\0\375\10\376\11\377\1\225\3]\13q \7\346\361\363\237\0!\12"
  "\346\361#i\357`\316\0\42\14\346\361\3I\226dI\316/\0#\21\346\361\303I\64HI\226dI"
  "\64HIN\6$\22\346q\205CRK\302\61\311\222,I\206\60\247\0%\15\346\361cQK\32\246"
  "I\324\316\2&\17\346\361#Z\324f\213\22-Zr\42\0'\11\346\361#i\235\237\0(\13\346\361"
  "ia\332s\254\303\0)\12\346\361\310\325\36\63\235\2*\15\346\361S\243L\32&-\312\31\1+\13"
  "\346\361\223\323l\320\322\234\31,\12\346\361\363)\15s\22\0-\11\346\361s\32t\236\0.\10\346\361"
  "\363K\316\0/\15\346q\246a\32\246a\32\246\71\15\60\21\346\361\3S\226DJ\213\224dI\26\355"
  "d\0\61\12\346\361#\241\332\343N\6\62\16\346\361\3S\226\226\246\64\35t*\0\63\16\346\361\3S"
  "\226fr\232d\321N\6\64\14\346q\247\245\236\6\61\315\311\0\65\16\346q\17J\232\16qZ\31r"
  "\62\0\66\20\346\361\3S\232\16Q\226dI\26\355d\0\67\13\346q\17J\226\206\325v\6\70\20\346"
  "\361\3S\226d\321\224%Y\222E;\31\71\17\346\361\3S\226dI\26\15ii'\3:\11\346\361"
  "\263\346L\71\3;\13\346\361\263\346\264\64\314I\0<\12\346\361cak\334N\5=\13\346\361\263\15"
  ":\60\350\334\0>\12\346\361\3qk\330\316\2\77\14\346\361\3S\226\206\325\34\314\31@\21\346\361\3"
  "S\226dI\262$K\262\304CN\5A\22\346\361\3S\226dI\226\14J\226dI\226S\1B\22"
  "\346q\17Q\226d\311\20eI\226d\311\220\223\1C\14\346\361\3C\222\366<\344T\0D\22\346q"
  "\17Q\226dI\226dI\226d\311\220\223\1E\16\346\361\3C\222\246C\224\226\207\234\12F\15\346\361"
  "\3C\222\246C\224\266\63\1G\21\346\361\3C\222V\226,\311\222,\32r*\0H\22\346qgI"
  "\226d\311\240dI\226dI\226S\1I\12\346\361\3c\332\343N\6J\12\346\361\3c\332\233\316\2"
  "K\21\346qgI\226D\321\26\325\222,\311r*\0L\12\346q\247}\36r*\0M\20\346qg"
  "\211eP\272%Y\222%YN\5N\20\346qg\211\224HI\77)\221\222\345T\0O\21\346\361\3"
  "S\226dI\226dI\226d\321N\6P\17\346q\17Q\226dI\226\14QZg\2Q\22\346\361\3"
  "S\226dI\226dI\226d\321\252\303\0R\22\346q\17Q\226dI\226\14Q\226dI\226S\1S"
  "\16\346\361\3C\222\306sZ\31r\62\0T\11\346q\17Z\332w\6U\22\346qgI\226dI\226"
  "dI\226d\321\220S\1V\20\346qgI\226dI\226dI\26m;\31W\21\346qgI\226d"
  "I\226\264\14\212%\313\251\0X\21\346qgI\26%a%\312\222,\311r*\0Y\20\346qgI"
  "\226dI\26\15ie\310\311\0Z\14\346q\17j\330\65\35t*\0[\13\346\361\14Q\332\257C\16"
  "\3\134\15\346q\244q\32\247q\32\247\71\14]\12\346\361\14i\177\32r\30^\12\346\361#a\22e"
  "\71\77_\11\346\361\363\353\240\303\0`\11\346\361\3q\235_\0a\16\346\361S\347hH\262$\213\206"
  "\234\12b\20\346q\247\351\20eI\226dI\226\14\71\31c\14\346\361S\207$m\36r*\0d\21"
  "\346\361ci\64$Y\222%Y\222ECN\5e\17\346\361S\207$K\262dP\342!\247\2f\14"
  "\346\361#S\32\16Y\332\316\2g\21\346\361S\207$K\262$K\262hN\206\34\1h\20\346q\247"
  "\351\20eI\226dI\226d\71\25i\13\346\361#\71\246v\325\311\0j\13\346\361C\71\230\366\246S"
  "\0k\16\346q\247\245J&&YT\313\251\0l\12\346\361\3i\237u\62\0m\15\346\361\23\207("
  "\351\337\222,\247\2n\20\346\361\23\207(K\262$K\262$\313\251\0o\16\346\361S\247,\311\222,"
  "\311\242\235\14p\21\346\361\23\207(K\262$K\262d\210\322*\0q\20\346\361S\207$K\262$K"
  "\262hH[\0r\14\346\361S\207$K\322v&\0s\15\346\361S\207$\236\323d\310\311\0t\13"
  "\346\361\3i\70\246\315:\31u\20\346\361\23\263$K\262$K\262h\310\251\0v\16\346\361\23\263$"
  "K\262$\213\222\60gw\17\346\361\23\263$KZ\6\305\222\345T\0x\16\346\361\23\263$\213\266)"
  "K\262\234\12y\22\346\361\23\263$K\262$K\262hH\223!G\0z\14\346\361\23\7\65l\34t"
  "*\0{\14\346\361iiM\224\323\262\16\3|\10\346q\245\375;\5}\14\346\361\310iY\324\322\232"
  "N\1~\12\346\361s\213\222D\347\10\177\7\346\361\363\237\0\200\6\341\311\243\0\201\6\341\311\243\0\202"
  "\6\341\311\243\0\203\6\341\311\243\0\204\6\341\311\243\0\205\6\341\311\243\0\206\6\341\311\243\0\207\6\341"
  "\311\243\0\210\6\341\311\243\0\211\6\341\311\243\0\212\6\341\311\243\0\213\6\341\311\243\0\214\16\346\361e"
  "C\222\306sZ\31r\62\0\215\6\341\311\243\0\216\6\341\311\243\0\217\14\346qe\203T\354\232\16:"
  "\25\220\6\341\311\243\0\221\6\341\311\243\0\222\6\341\311\243\0\223\6\341\311\243\0\224\6\341\311\243\0\225"
  "\6\341\311\243\0\226\6\341\311\243\0\227\16\346\361eC\222\306sZ\31r\62\0\230\6\341\311\243\0\231"
  "\6\341\311\243\0\232\6\341\311\243\0\233\6\341\311\243\0\234\16\346\361\205\71\66$\361\234&CN\6\235"
  "\6\341\311\243\0\236\21\346\361#%i\210\6eP\206(\221*\71\25\237\15\346\361\205\71\64\250a\343"
  "\240S\1\240\7\346\361\363\237\0\241\23\346\361\3S\226dI\226\14J\226dI\26\306\71\0\242\21\346"
  "\361\23\302!\251%Y\222%\341\220\345\24\0\243\14\346q\247-\231\230\306CN\5\244\22\346\361\3S"
  "\226dI\226\14J\226dI\26\346\4\245\22\346\361\3S\226dI\226\14J\226dI\26\346\4\246\16"
  "\346\361eC\222\306sZ\31r\62\0\247\17\346\361#Z\224\245Z\324\233\232E\231\4\250\11\346\361\3"
  "I\316\237\1\251\21\346\361\3C\22J\211\22)\221bL\206\234\12\252\15\346\361#r\66\325vd\310"
  "\31\1\253\17\346\361\223\243$J\242\266(\213r\42\0\254\14\346qe\203T\354\232\16:\25\255\10\346"
  "\361s\333y\3\256\21\346\361\3C\22*\226d\261$c\62\344T\0\257\14\346qe\203\32vM\7"
  "\235\12\260\12\346\361#Z\324\246\363\11\261\20\346\361S\347hH\262$\213\206\64\314\21\0\262\14\346\361"
  "#Z\224\206\305!\347\6\263\13\346\361\3i\252\251\315:\31\264\11\346\361Ca\235\337\0\265\14\346\361"
  "\23\243\376i\251\346 \0\266\16\346\361\205\71\66$\361\234&CN\6\267\10\346\361s\314y\4\270\11"
  "\346\361\363\207\64\14\1\271\20\346\361S\347hH\262$\213\206\64\314\21\0\272\15\346\361#Z\324\233\16"
  "\15\71#\0\273\17\346\361\23\243,\312\242\226(\211r\62\0\274\15\346\361\205\71\64\250a\343\240S\1"
  "\275\17\346\361\204j-\211\302\26\245\24\26\207\0\276\21\346\361hQ\30'\222\64\206ZR\33\302\64\1"
  "\277\15\346\361#\71\64\250a\343\240S\1\300\21\346\361\304\341\224%Y\62(Y\222%YN\5\301\21"
  "\346\361\205\341\224%Y\62(Y\222%YN\5\302\22\346q\205I\66eI\226\14J\226dI\226S"
  "\1\303\23\346\361DI\242MY\222%\203\222%Y\222\345T\0\304\20\346\361S\347hH\262$\213\206"
  "\64\314\21\0\305\16\346\361eC\222\306sZ\31r\62\0\306\14\346\361eC\222\366<\344T\0\307\15"
  "\346\361\3C\222\366<di\30\2\310\17\346\361\304\341\220\244\351\20\245\361\220S\1\311\17\346\361\205\341"
  "\220\244\351\20\245\361\220S\1\312\20\346\361\3C\222\246C\224\226\207\64\314\21\0\313\17\346\361\324\241!"
  "I\323!J\343!\247\2\314\13\346\361\304\341\230v\334\311\0\315\13\346\361\205\341\230v\334\311\0\316\14"
  "\346q\205I\66\246\35w\62\0\317\13\346\361\324\241\61\355\270\223\1\320\15\346\361\3[\324\262D}\332"
  "\311\0\321\20\346\361EIV\221\22)\351'%\322\251\0\322\20\346\361\304\341\224%Y\222%Y\222E"
  ";\31\323\20\346\361\205\341\224%Y\222%Y\222E;\31\324\21\346q\205I\66eI\226dI\226d"
  "\321N\6\325\22\346\361DI\242MY\222%Y\222%Y\264\223\1\326\21\346\361\324\241)K\262$K"
  "\262$\213v\62\0\327\14\346\361S\243L\324\242\234\33\0\330\20\346qFS\226DJ_\244$\213\246"
  "\234\6\331\21\346\361\304Y%K\262$K\262$\213\206\234\12\332\21\346\361\205Y%K\262$K\262$"
  "\213\206\234\12\333\23\346q\205I\224%Y\222%Y\222%Y\64\344T\0\334\22\346\361\324\221,\311\222"
  ",\311\222,\311\242!\247\2\335\17\346\361\205Y%K\262hH+CN\6\336\21\346\361\243\351\20e"
  "I\226dI\226\14QN\3\337\17\346\361\3Z\324%\213j\211\224$:\31\340\20\346q\305\71\64G"
  "C\222%Y\64\344T\0\341\20\346\361\205\71\66GC\222%Y\64\344T\0\342\11\346\361Ca\235\337"
  "\0\343\21\346\361DI\242Cs\64$Y\222ECN\5\344\20\346\361\3I\16\315\321\220dI\26\15"
  "\71\25\345\20\346q\205I\30\316\321\220dI\26\15\71\25\346\15\346\361Ca\70$i\363\220S\1\347"
  "\15\346\361S\207$m\36\262\64\14\1\350\20\346q\305\71\64$Y\222%\203\22\17\71\25\351\20\346\361"
  "\205\71\66$Y\222%\203\22\17\71\25\352\20\346\361S\207$K\262dP\342!\254C\0\353\21\346\361"
  "\3I\16\15I\226d\311\240\304CN\5\354\13\346q\305\71\244v\325\311\0\355\13\346\361\205\71\246v"
  "\325\311\0\356\14\346q\205I\16\251]u\62\0\357\14\346\361\3I\16\251]u\62\0\360\21\346q$"
  "a%\234\262$K\262$\213v\62\0\361\21\346\361\205\71\64DY\222%Y\222%YN\5\362\20\346"
  "q\305\71\64eI\226dI\26\355d\0\363\20\346\361\205\71\66eI\226dI\26\355d\0\364\20\346"
  "q\205I\16MY\222%Y\222E;\31\365\21\346\361c\222\222HI\226dI\66\15\221N\4\366\20"
  "\346\361\3I\16MY\222%Y\222E;\31\367\13\346\361\223sh\320\241\234\31\370\17\346\361\223\242)"
  "RZ\244$\213\246\234\6\371\21\346q\305\71\222%Y\222%Y\222ECN\5\372\21\346\361\205\71\224"
  "%Y\222%Y\222ECN\5\373\22\346q\205I\216dI\226dI\226d\321\220S\1\374\22\346\361"
  "\3I\216dI\226dI\226d\321\220S\1\375\23\346\361\205\71\224%Y\222%Y\222ECZ\31\42"
  "\0\376\22\346q\247\351\20eI\226dI\226\14Q\232\203\0\377\23\346\361\3I\216dI\226dI\226"
  "d\321\220V\206\10\0\0\0\4\377\377\0"
;

const uint8_t mono04b03b[912] U8G2_FONT_SECTION("mono04b03b") = 
  "`\1\3\3\3\3\1\1\4\5\6\0\377\5\377\5\0\1$\2c\3s \6s{D\0!\10r"
  "\212HZ\14\0\42\10t\214Hv\64\0#\14v\236\244R$T\212\304!\0$\12u\235I\22)"
  "\22\231\3%\14v\16QD\22L\221\204\344\0&\13v\216Y\264\22\12\321!\0'\7r\212H\34"
  "\4(\10s\233\244\264 \0)\10s\213X(%\12*\11t\214H(%\16\6+\10t\334\320("
  "\16\3,\7s{p$\4-\7t|\310\34\14.\7rzH\14\0/\7v\316\274\3\1\60\14"
  "u\15J(\22\212\204\42d\0\61\10u\35a\266\61\0\62\11u\15b\204\22$\3\63\11u\15b"
  "\204\30!\3\64\14u\215P$\24\11E\210a\0\65\11u\15J\220\30!\3\66\12u\215Q\220\22"
  "\212\220\1\67\11u\15b,\61\16\1\70\13u\15J(B\11E\310\0\71\12u\15J(B\14\215"
  "\1:\7r\252X\24\0;\7r\252X$\6<\7t\254\214\251\0=\7t\314\351\34\4>\10t"
  "\214`R:\0\77\12u\15bh\16\210C\0@\14v\216J,\22\231d\241C\0A\14u\15J"
  "(B\11EBa\0B\13u\215Q$D\11E\310\0C\10u\15J\60\221\14D\13u\215QJ"
  "(\22\212\314\1E\11u\15J\220\22$\3F\12u\15J\220\22\214\203\0G\13u\15J\60\42\11"
  "E\310\0H\15u\215P$\24\241\204\42\241\60\0I\10u\235Y\60m\14J\11u-aJ(B"
  "\6K\14u\215P$\24\31\245\204\302\0L\10u\215`F\62\0M\10v\216J\376\357\0N\15u"
  "\215PD\222\42\11EBa\0O\14u\15J(\22\212\204\42d\0P\14u\15J(\22\212P\342"
  " \0Q\14u\15J(\22\212D$d\0R\13u\15J(BI\212\210\1S\11u\15J\220\30"
  "!\3T\10t\214Q,\63\0U\15u\215P$\24\11EB\21\62\0V\15u\215P$\24I\212"
  "\304\342\20\0W\11v\216H\376\227:\0X\14u\215P$\24\22\245\204\302\0Y\13u\215P$\24"
  "!F\310\0Z\11u\15bH\24$\3[\10s\13I(I\10\134\7v\216p\356\0]\10s\13"
  "Q\26!\0^\10t\234P$\216\6_\7u}d\62\0`\7s\213X\34\14a\11u}\340$"
  "\24!\3b\11u\335 %\24!\3c\10t|\310$\66\5d\12u}H\204\22\212\220\1e\10"
  "u}\30%m\14f\11u\355Q\214\24\207\0g\12u}\30%\24\241\205\0h\12u\335 %\24"
  "\11\205\1i\7r\252X$\6j\10r\252X$\5\0k\11u\335`(\62J\6l\7r\252H"
  "\66\0m\10v~h%\337\1n\12u}\30%\24\11\205\1o\11u}\30%\24!\3p\12u"
  "}\30%\24\241\4\1q\12u}\30%\24!F\0r\10t|\310$\26\7s\10u}\30)F"
  "\6t\10t\334\320(&\5u\12u}X(\22\212\220\1v\12u}X(\22\12\311\1w\11v"
  "~h$/u\0x\11t|HRJ\24\0y\13u}X(\22\212\20#\0z\10u}\30-"
  "D\6{\10t\34QbL\12|\7r\212Hn\0}\11t\14Y\60\24\22\3~\7u\235\334\261"
  "\0\177\7t\14\221\316\0\0\0\0\4\377\377\0"
;


// Ikona karty SD wyswietlana przy braku karty podczas startu
static unsigned char sdcard[] PROGMEM = {
  0xf0, 0xff, 0xff, 0x0f, 0xf8, 0xff, 0xff, 0x1f, 0xf8, 0xcf, 0xf3, 0x3f,
  0x38, 0x49, 0x92, 0x3c, 0x38, 0x49, 0x92, 0x3c, 0x38, 0x49, 0x92, 0x3c,
  0x38, 0x49, 0x92, 0x3c, 0x38, 0x49, 0x92, 0x3c, 0x38, 0x49, 0x92, 0x3c,
  0x38, 0x49, 0x92, 0x3c, 0xf8, 0xff, 0xff, 0x3f, 0xf8, 0xff, 0xff, 0x3f,
  0xf8, 0xff, 0xff, 0x3f, 0xf8, 0xff, 0xff, 0x3f, 0xf8, 0xff, 0xff, 0x3f,
  0xfc, 0xff, 0xff, 0x3f, 0xfe, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0x3f,
  0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0x3f,
  0xfc, 0xff, 0xff, 0x3f, 0xfc, 0xc0, 0x80, 0x3f, 0x7c, 0xc0, 0x00, 0x3f,
  0x3c, 0xc0, 0x00, 0x3e, 0x1c, 0xfc, 0x38, 0x3e, 0x1e, 0xfc, 0x78, 0x3c,
  0x1f, 0xe0, 0x78, 0x3c, 0x3f, 0xc0, 0x78, 0x3c, 0x7f, 0x80, 0x78, 0x3c,
  0xff, 0x87, 0x78, 0x3c, 0xff, 0x87, 0x38, 0x3e, 0x3f, 0x80, 0x00, 0x3e,
  0x1f, 0xc0, 0x00, 0x3f, 0x3f, 0xe0, 0x80, 0x3f, 0xff, 0xff, 0xff, 0x3f,
  0xff, 0xff, 0xff, 0x3f, 0xff, 0xff, 0xff, 0x3f, 0xfe, 0xff, 0xff, 0x1f,
  0xfc, 0xff, 0xff, 0x0f 
  };

// Obrazek nutek wyswietlany przy starcie
#define notes_width 256
#define notes_height 46
static unsigned char notes[] PROGMEM = {
  0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0c, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x40, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x0c, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x20, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0e, 0x00,
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0e, 0x00, 0x02, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x20, 0x07, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x07, 0x00,
  0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x80, 0x05, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
  0x00, 0xf0, 0x01, 0xc0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x40,
  0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x08,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
  0x00, 0x01, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x40, 0x08, 0x00, 0x00, 0x00,
  0x00, 0x60, 0x00, 0x00, 0x00, 0x5e, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,
  0x60, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
  0x00, 0x5e, 0x00, 0x80, 0x08, 0x00, 0x00, 0x07, 0x00, 0xe0, 0x01, 0x00,
  0xc0, 0x47, 0x00, 0x08, 0x00, 0x00, 0x07, 0x00, 0xe0, 0x01, 0x00, 0x00,
  0x40, 0x00, 0x18, 0x00, 0x80, 0x00, 0x00, 0x0e, 0x00, 0x4e, 0x00, 0x00,
  0x19, 0x00, 0xf0, 0x07, 0x00, 0x20, 0x01, 0x00, 0xf8, 0x41, 0x00, 0x18,
  0x00, 0xf0, 0x07, 0x00, 0x20, 0x01, 0x00, 0x18, 0x40, 0x00, 0x78, 0x00,
  0x80, 0x00, 0xe0, 0x0f, 0x00, 0x47, 0x00, 0x00, 0x10, 0x00, 0xf0, 0x07,
  0x00, 0x20, 0x03, 0x00, 0x3c, 0x40, 0x00, 0x10, 0x00, 0xf0, 0x07, 0x00,
  0x20, 0x03, 0x00, 0x1c, 0x40, 0x00, 0xe8, 0x00, 0x80, 0x00, 0xe0, 0x0f,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x41, 0x00, 0x00,
  0x10, 0x00, 0x3f, 0x04, 0x00, 0xe0, 0x04, 0xe0, 0x11, 0x40, 0x00, 0x10,
  0x00, 0x3f, 0x04, 0x00, 0xe0, 0x04, 0xe0, 0x11, 0x40, 0x00, 0x38, 0x01,
  0x40, 0x00, 0x7e, 0x08, 0xc0, 0xe0, 0x0f, 0x00, 0x10, 0x00, 0x0f, 0x04,
  0x00, 0xb0, 0x01, 0x78, 0x10, 0x40, 0x00, 0x10, 0x00, 0x0f, 0x04, 0x00,
  0xb0, 0x01, 0x78, 0x10, 0x40, 0x00, 0x68, 0x01, 0x40, 0x00, 0x1e, 0x08,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x40, 0x70, 0x3e, 0x00,
  0x1c, 0x00, 0x01, 0x04, 0x00, 0x10, 0x00, 0x0e, 0x10, 0x40, 0x00, 0x18,
  0x00, 0x01, 0x04, 0x00, 0x10, 0x00, 0x0e, 0x10, 0x40, 0x00, 0xa8, 0x00,
  0x2f, 0x00, 0x02, 0x08, 0x40, 0x98, 0x78, 0x00, 0x1f, 0x00, 0x01, 0x04,
  0x00, 0x10, 0x00, 0x06, 0x10, 0x40, 0x00, 0x1f, 0x00, 0x01, 0x04, 0x00,
  0x10, 0x00, 0x06, 0x10, 0x40, 0x00, 0x48, 0xc0, 0x1f, 0x00, 0x02, 0x08,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x40, 0x88, 0x70, 0x80,
  0x0f, 0x00, 0x01, 0x04, 0x80, 0x17, 0x00, 0x02, 0x10, 0x7c, 0x80, 0x0f,
  0x00, 0x01, 0x04, 0x80, 0x17, 0x00, 0x02, 0x10, 0x7c, 0x00, 0x08, 0x80,
  0x0f, 0x00, 0x02, 0x08, 0x40, 0x88, 0x70, 0x00, 0x07, 0x00, 0x01, 0x04,
  0xc0, 0x1f, 0x00, 0x02, 0x10, 0x7e, 0x00, 0x07, 0x00, 0x01, 0x04, 0xc0,
  0x1f, 0x00, 0x02, 0x10, 0x7e, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x02, 0x08,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x71, 0x00,
  0x00, 0x00, 0xc1, 0x07, 0xe0, 0x07, 0x00, 0x02, 0x1e, 0x00, 0x00, 0x00,
  0x00, 0xc1, 0x07, 0xe0, 0x07, 0x00, 0x02, 0x1e, 0x00, 0xc0, 0x07, 0x00,
  0x00, 0x00, 0x82, 0x0f, 0x80, 0x01, 0x39, 0x00, 0x00, 0x00, 0xe1, 0x07,
  0xc0, 0x03, 0x00, 0x02, 0x1f, 0x00, 0x00, 0x00, 0x00, 0xe1, 0x07, 0xc0,
  0x03, 0x00, 0x02, 0x1f, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0xc2, 0x0f,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0f, 0x1e, 0x00,
  0x00, 0xf0, 0x81, 0x03, 0x00, 0x00, 0x80, 0x03, 0x0e, 0x00, 0x00, 0x00,
  0xf0, 0x81, 0x03, 0x00, 0x00, 0x80, 0x03, 0x0e, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xe0, 0x03, 0x07, 0x00, 0xfc, 0x0f, 0x00, 0x00, 0xf8, 0x01, 0x00,
  0x00, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x01, 0x00, 0x00,
  0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x03, 0x00,
  0x00, 0xf0, 0x03, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03,
  0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00,
  0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00,
  0x70, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x0c, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x02, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x0c, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x01, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


// Funkcja odwracania bitów MSL-LSB <-> LSB-MSB
uint32_t reverse_bits(uint32_t inval, int bits)
{
  if ( bits > 0 )
  {
    bits--;
    return reverse_bits(inval >> 1, bits) | ((inval & 1) << bits);
  }
return 0;
}

/*---------- Definicja portu i deklaracje zmiennych do obsługi odbiornika IR ----------*/

// Zmienne obsługi  pilota IR
bool pulse_ready = false;                // Flaga sygnału gotowości
unsigned long pulse_start_high = 0;      // Czas początkowy impulsu
unsigned long pulse_end_high = 0;        // Czas końcowy impulsu
unsigned long pulse_duration = 0;        // Czas trwania impulsu
unsigned long pulse_duration_9ms = 0;    // Tylko do analizy - Czas trwania impulsu
unsigned long pulse_duration_4_5ms = 0;  // Tylko do analizy - Czas trwania impulsu
unsigned long pulse_duration_560us = 0;  // Tylko do analizy - Czas trwania impulsu
unsigned long pulse_duration_1690us = 0; // Tylko do analizy - Czas trwania impulsu

bool pulse_ready9ms = false;            // Flaga sygnału gotowości puls 9ms
bool pulse_ready_low = false;           // Flaga sygnału gotowości 
unsigned long pulse_start_low = 0;      // Czas początkowy impulsu
unsigned long pulse_end_low = 0;        // Czas końcowy impulsu
unsigned long pulse_duration_low = 0;   // Czas trwania impulsu


volatile unsigned long ir_code = 0;  // Zmienna do przechowywania kodu IR
volatile int bit_count = 0;          // Licznik bitów w odebranym kodzie
bool remoteConfig = false;           // Tryb uczenia pilota (nauka kodów IR)

// Progi przełączeń dla sygnałów czasowych pilota IR
const int LEAD_HIGH = 9050;         // 9 ms sygnał wysoki (początkowy)
const int LEAD_LOW = 4500;          // 4,5 ms sygnał niski (początkowy)
const int TOLERANCE = 150;          // Tolerancja (w mikrosekundach)
const int HIGH_THRESHOLD = 1690;    // Sygnał "1"
const int LOW_THRESHOLD = 600;      // Sygnał "0"

bool data_start_detected = false;  // Flaga dla sygnału wstępnego
bool rcInputDigitsMenuEnable = false;

//================ Definicja portów i pinów dla klaiwatury numerycznej ===========================//

const int keyboardPin = 9;  // wejscie klawiatury (ADC)

unsigned long keyboardValue = 0;
unsigned long keyboardLastSampleTime = 0;
unsigned long keyboardSampleDelay = 50;


// ---- Progi przełaczania ADC dla klawiatury matrycowej 5x3 w tunrze Sony ST-120 ---- //
int keyboardButtonThresholdTolerance = 20; // Tolerancja dla pomiaru ADC
int keyboardButtonNeutral = 4095;          // Pozycja neutralna
int keyboardButtonThreshold_0 = 2375;      // Przycisk 0
int keyboardButtonThreshold_1 = 10;        // Przycisk 1
int keyboardButtonThreshold_2 = 545;       // Przycisk 2
int keyboardButtonThreshold_3 = 1390;      // Przycisk 3
int keyboardButtonThreshold_4 = 1925;      // Przycisk 4
int keyboardButtonThreshold_5 = 2285;      // Przycisk 5
int keyboardButtonThreshold_6 = 385;       // Przycisk 6
int keyboardButtonThreshold_7 = 875;       // Przycisk 7
int keyboardButtonThreshold_8 = 1585;      // Przycisk 8
int keyboardButtonThreshold_9 = 2055;      // Przycisk 9
int keyboardButtonThreshold_Shift = 2455;  // Shift - funkcja Enter/OK
int keyboardButtonThreshold_Memory = 2170; // Memory - funkcja Bank menu
int keyboardButtonThreshold_Band = 1640;   // Przycisk Band - funkcja Back
int keyboardButtonThreshold_Auto = 730;    // Przycisk Auto - przelacza Radio/Zegar
int keyboardButtonThreshold_Scan = 1760;   // Przycisk Scan - funkcja Dimmer ekranu OLED
int keyboardButtonThreshold_Mute = 1130;   // Przycisk Mute - funkcja MUTE

// Flagi do monitorowania stanu klawiatury
bool keyboardButtonPressed = false; // Wcisnięcie klawisza
bool debugKeyboard = false;         // Wyłącza wywoływanie funkcji i zostawia tylko wydruk pomiaru ADC
bool adcKeyboardEnabled = false;    // Flaga właczajaca działanie klawiatury ADC

// ===== PROTOTYPY =====
void displayDimmer(bool dimmerON);
void displayDimmerTimer();
void displayPowerSave(bool mode);
void loadSDPlayerSessionState();
void saveSDPlayerSessionState();
void clearSDPlayerSessionCompleted();

// ========== STORAGE ACCESS HELPER FOR EXTERNAL MODULES ==========
// Funkcja pomocnicza dla modułów zewnętrznych (EQ16, SDPlayer, etc.)
// która potrzebują dostępu do systemu plików
fs::FS& getStorage() {
  #ifdef AUTOSTORAGE
    return STORAGE;  // W trybie AUTOSTORAGE zwraca SD lub LittleFS
  #elif defined(USE_SD)
    return SD;       // W trybie SD
  #else
    return LittleFS; // W trybie LittleFS
  #endif
}
// ================================================================


#ifdef AUTOSTORAGE
  #define STORAGE_BEGIN() initStorage()
  bool initStorage() 
  {
    if (SD.begin(SD_CS, customSPI)) 
    {
      _storage = &SD;
      useSD = true;
      storageTextName = "SD";
      Serial.println("debug SD -> Uzywamy Karty SD");
      //#define SD_LED 17          // LED - sygnalizacja CS, aktywność karty SD, klon pinu uzyteczny w przypadku uzywania tylnego czytnika SD na PCB
      noSDcard = false;
      return true;
    }

    Serial.println("debug SD -> Brak karty SD, przelaczam na LittleFS");
    noSDcard = true; // Flag braku karty SD

    if (LittleFS.begin(true)) 
    {
      _storage = &LittleFS;
      useSD = false;
      storageTextName = "LittleFS";
      noSDcard = true;
      return true;
    }

    Serial.println("debug SD -> BLAD: brak systemu plikow, zapis tylko podstawowych wartosci do EEPROM");
    storageTextName = "EEPROM";
    return false;
  }
#endif

/*
bool isValidUtf8(const String& str) 
{
  const unsigned char* bytes = (const unsigned char*)str.c_str();

  while (*bytes) {
    if (*bytes <= 0x7F) {
      // ASCII (0xxxxxxx)
      bytes++;
    } else if ((*bytes & 0xE0) == 0xC0) {
      // 2-byte sequence (110xxxxx 10xxxxxx)
      if ((bytes[1] & 0xC0) != 0x80) return false;
      bytes += 2;
    } else if ((*bytes & 0xF0) == 0xE0) {
      // 3-byte sequence (1110xxxx 10xxxxxx 10xxxxxx)
      if ((bytes[1] & 0xC0) != 0x80 || (bytes[2] & 0xC0) != 0x80) return false;
      bytes += 3;
    } else if ((*bytes & 0xF8) == 0xF0) {
      // 4-byte sequence (11110xxx 10xxxxxx 10xxxxxx 10xxxxxx)
      if ((bytes[1] & 0xC0) != 0x80 || (bytes[2] & 0xC0) != 0x80 || (bytes[3] & 0xC0) != 0x80) return false;
      bytes += 4;
    } else {
      return false; // Invalid leading byte
    }
  }

  return true;
}
*/

void removeUtf8Bom(String &text) 
{
  if (text.length() >= 3 &&
      (uint8_t)text[0] == 0xEF &&
      (uint8_t)text[1] == 0xBB &&
      (uint8_t)text[2] == 0xBF) {
    text.remove(0, 3);  // usuń bajty EF BB BF
  }
}

// Funkcja przetwarza tekst, zamieniając polskie znaki diakrytyczne
void processText(String &text) 
{
  removeUtf8Bom(text); // Usuwamy znaki BOM na początku nazwy utworu

  for (int i = 0; i < text.length() - 1; i++) 
  {
    if (i + 1 >= text.length()) break; // zabezpieczenie przed wyjściem poza bufor

    switch ((uint8_t)text[i]) {
      case 0xC3:
        switch ((uint8_t)text[i + 1]) {
          case 0xB3: text.setCharAt(i, 0xF3); break;  // ó
          case 0x93: text.setCharAt(i, 0xD3); break;  // Ó
        }
        text.remove(i + 1, 1);
        break;

      case 0xC4:
        switch ((uint8_t)text[i + 1]) {
          case 0x85: text.setCharAt(i, 0xB9); break;  // ą
          case 0x84: text.setCharAt(i, 0xA5); break;  // Ą
          case 0x87: text.setCharAt(i, 0xE6); break;  // ć
          case 0x86: text.setCharAt(i, 0xC6); break;  // Ć
          case 0x99: text.setCharAt(i, 0xEA); break;  // ę
          case 0x98: text.setCharAt(i, 0xCA); break;  // Ę
        }
        text.remove(i + 1, 1);
        break;

      case 0xC5:
        switch ((uint8_t)text[i + 1]) {
          case 0x82: text.setCharAt(i, 0xB3); break;  // ł
          case 0x81: text.setCharAt(i, 0xA3); break;  // Ł
          case 0x84: text.setCharAt(i, 0xF1); break;  // ń
          case 0x83: text.setCharAt(i, 0xD1); break;  // Ń
          case 0x9B: text.setCharAt(i, 0x9C); break;  // ś
          case 0x9A: text.setCharAt(i, 0x8C); break;  // Ś
          case 0xBA: text.setCharAt(i, 0x9F); break;  // ź
          case 0xB9: text.setCharAt(i, 0x8F); break;  // Ź
          case 0xBC: text.setCharAt(i, 0xBF); break;  // ż
          case 0xBB: text.setCharAt(i, 0xAF); break;  // Ż
        }
        text.remove(i + 1, 1);
        break;
    }
  }
}

/*
void win1250ToUtf8(String& input) 
{
  String output = "";
  const uint8_t* bytes = (const uint8_t*)input.c_str();

  while (*bytes) {
    uint8_t c = *bytes;
    switch (c) {
      case 0xA5: output += "Ą"; break;
      case 0xB9: output += "ą"; break;
      case 0xC6: output += "Ć"; break;
      case 0xE6: output += "ć"; break;
      case 0xCA: output += "Ę"; break;
      case 0xEA: output += "ę"; break;
      case 0xA3: output += "Ł"; break;
      case 0xB3: output += "ł"; break;
      case 0xD1: output += "Ń"; break;
      case 0xF1: output += "ń"; break;
      case 0xD3: output += "Ó"; break;
      case 0xF3: output += "ó"; break;
      case 0xA6: output += "Ś"; break;
      case 0xB6: output += "ś"; break;
      case 0xAF: output += "Ż"; break;
      case 0xBF: output += "ż"; break;
      case 0xAC: output += "Ź"; break;
      case 0xBC: output += "ź"; break;
      case 0x8C: output += "Ś"; break;
      case 0x9C: output += "ś"; break;
      default:
        if (c < 0x80) {
          output += (char)c;
        } else {
          output += '?';
        }
        break;
    }
    bytes++;
  }

  input = output; // <-- modyfikujemy oryginalny String
}
*/

bool sanitizeUtf8(String& s)
{
  String out;
  out.reserve(s.length() * 2);

  const uint8_t* p = (const uint8_t*)s.c_str();

  while (*p)
  {
    uint8_t c = *p;

    // ASCII
    if (c < 0x80) {
      out += (char)c;
      p++;
      continue;
    }

    // Poprawny UTF-8 (2–4 bajty)
    uint8_t len = 0;
    if ((c & 0xE0) == 0xC0) len = 2;
    else if ((c & 0xF0) == 0xE0) len = 3;
    else if ((c & 0xF8) == 0xF0) len = 4;

    if (len) {
      bool ok = true;
      for (uint8_t i = 1; i < len; i++) {
        if ((p[i] & 0xC0) != 0x80) {
          ok = false;
          break;
        }
      }
      if (ok) {
        for (uint8_t i = 0; i < len; i++)
          out += (char)p[i];
        p += len;
        continue;
      }
    }

    // Win-1250 na UTF-8 (PL)
    switch (c)
    {
      case 0xA5: out += "Ą"; break;
      case 0xB9: out += "ą"; break;
      case 0xC6: out += "Ć"; break;
      case 0xE6: out += "ć"; break;
      case 0xCA: out += "Ę"; break;
      case 0xEA: out += "ę"; break;
      case 0xA3: out += "Ł"; break;
      case 0xB3: out += "ł"; break;
      case 0xD1: out += "Ń"; break;
      case 0xF1: out += "ń"; break;
      case 0xD3: out += "Ó"; break;
      case 0xF3: out += "ó"; break;
      case 0xA6: out += "Ś"; break;
      case 0xB6: out += "ś"; break;
      case 0xAF: out += "Ż"; break;
      case 0xBF: out += "ż"; break;
      case 0xAC: out += "Ź"; break;
      case 0xBC: out += "ź"; break;
      case 0x8C: out += "Ś"; break;
      case 0x9C: out += "ś"; break;
      case 0x8F: out += "Ź"; break;
      case 0x9F: out += "ź"; break;
      default:   Serial.print("debug - > UTF8 Web Sanitizer, Nieznany znak:"); Serial.println(c, HEX); out += '?'; break;
    }

    p++;
  }

  if (out != s) {
    s = out;
    return true;   // coś poprawiono
  }
  return false;    // było OK
}


void wsStationChange(uint8_t stationId, uint8_t bankId) 
{
  // iteracja po wszystkich podłączonych klientach
  for (auto& client : ws.getClients()) 
  {
 
    client.text("stationname:" + String(stationName.substring(0, stationNameLenghtCut)));

    if (urlPlaying) 
    {
      client.text("bank:" + String(0));
      client.text("station:" + String(0));
    } 
    else 
    {
      client.text("bank:" + String(bankId));
      client.text("station:" + String(stationId));
    }

    client.text("volume:" + String(volumeValue));
    client.text("mute:" + String(volumeMute)); 

    client.text("bitrate:" + String(bitrateString)); 
    client.text("samplerate:" + String(sampleRateString)); 
    client.text("bitpersample:" + String(bitsPerSampleString)); 

    if (mp3)    client.text("streamformat:MP3");
    if (flac)   client.text("streamformat:FLAC");
    if (aac)    client.text("streamformat:AAC");
    if (vorbis) client.text("streamformat:VORBIS");
    if (opus)   client.text("streamformat:OPUS");
  }
}

void wsRefreshPage() // Funckja odswiezania strony za pomocą WebSocket
{
  ws.textAll("reload");  // wyślij komunikat do wszystkich klientów
}

void wsStreamInfoRefresh()
{
  // iteracja po wszystkich podłączonych klientach
  for (auto& client : ws.getClients()) 
  {
 
    client.text("stationname:" + String(stationName.substring(0, stationNameLenghtCut)));

    if (audio.isRunning() == true)
    {  
      sanitizeUtf8(stationStringWeb);
      client.text("stationtext$" + stationStringWeb); 
    }
    else
    {
      client.text("stationtext$... No audio stream ! ...");
    }

    /*
    if (urlPlaying) 
    {
      client.text("bank:" + String(0));
      client.text("station:" + String(0));
    } 
    else 
    {
      client.text("bank:" + String(bank_nr));
      client.text("station:" + String(station_nr));
    }
    */

    //client.text("volume:" + String(volumeValue));
    //client.text("mute:" + String(volumeMute)); 

    client.text("bitrate:" + String(bitrateString)); 
    client.text("samplerate:" + String(sampleRateString)); 
    client.text("bitpersample:" + String(bitsPerSampleString)); 

    if (mp3)    client.text("streamformat:MP3");
    if (flac)   client.text("streamformat:FLAC");
    if (aac)    client.text("streamformat:AAC");
    if (vorbis) client.text("streamformat:VORBIS");
    if (opus)   client.text("streamformat:OPUS");
  }
}

void wsVolumeChange()  
{
  ws.textAll("volume:" + String(volumeValue)); // wysyła wartosc volume do wszystkich połączonych klientów
  ws.textAll("mute:" + String(volumeMute)); // wysyła wartosc mute do wszystkich połączonych klientów
}

void wsSignalLevel()
{
  ws.textAll("wifi:" + String(wifiSignalLevel)); // wysyła wartosc sygnalu wifi do wszystkich połączonych klientów
}

// WebSocket - czyszczenie ekranu nauki pilota
void wsIrCodeClear()
{
  for (auto& client : ws.getClients()) 
  {
    client.text("learn:" + String(remoteConfig));
    client.text(String("clear"));
  }
}

// WebSocket - wysyłanie kodu IR w trybie nauki pilota
void wsIrCode(uint16_t code, uint8_t add, uint8_t cm)
{
  char buf1[7];                    
  char buf2[5];
  char buf3[5];

  sprintf(buf1, "0x%04X", code); // Formatowanie "0xFFFF" + \0
  sprintf(buf2, "0x%02X", add);
  sprintf(buf3, "0x%02X", cm);

  Serial.print("wsIRCode:"); Serial.println(buf1);

  for (auto& client : ws.getClients()) 
  {
    client.text(String("ircode:") + buf1);
    client.text(String("addr:") + buf2);
    client.text(String("cmd:") + buf3);
    client.text("learn:" + String(remoteConfig));
  }
}

//Funkcja odpowiedzialna za zapisywanie informacji o stacji do pamięci PSRAM
void saveStationToPSRAM(const char *station) 
{
  
  // Sprawdź, czy istnieje jeszcze miejsce na kolejną stację w pamięci PSRAM
  if (stationsCount < MAX_STATIONS) {
    int length = strlen(station);

    // Sprawdź, czy długość linku nie przekracza ustalonego maksimum.
    if (length <= STATION_NAME_LENGTH) {
      // Zapisz długość linku jako pierwszy bajt.
      psramData[stationsCount * (STATION_NAME_LENGTH + 1)] = length;
      // Zapisz link jako kolejne bajty w pamięci PSRAM
      for (int i = 0; i < length; i++) 
      {
        psramData[stationsCount * (STATION_NAME_LENGTH + 1) + 1 + i] = station[i];
      }

      // Wydrukuj informację o zapisanej stacji na Serialu.
      Serial.println(String(stationsCount + 1) + "   " + String(station));  // Drukowanie na serialu od nr 1 jak w banku na serwerze

      // Zwiększ licznik zapisanych stacji.
      stationsCount++;

      // progress bar pobieranych stacji
      u8g2.setFont(spleen6x12PL);  
      u8g2.drawStr(21, 36, "Progress:");
      //u8g2.drawStr(21, 36, "Loading: ");
      u8g2.drawStr(75, 36, String(stationsCount).c_str());  // Napisz licznik pobranych stacji

      u8g2.drawRFrame(21, 42, 212, 12, 3);  // Ramka paska postępu ladowania stacji stacji w>8 h>8
      
      int x = (stationsCount * 2) + 8;          // Dodajemy gdy stationCount=1 + 8 aby utrzymac warunek dla zaokrąglonego drawRBox - szerokość W>6 h>6 ma byc W>=2*(r+1), h >= 2*(r+1)
      u8g2.drawRBox(23, 44, x, 8, 2);       // Pasek postepu ladowania stacji z serwera lub karty SD / SPIFFS       
      
      u8g2.sendBuffer();  
    } else {
      // Informacja o błędzie w przypadku zbyt długiego linku do stacji.
      Serial.println("Błąd: Link do stacji jest zbyt długi");
    }
  } else {
    // Informacja o błędzie w przypadku osiągnięcia maksymalnej liczby stacji.
    Serial.println("Błąd: Osiągnięto maksymalną liczbę zapisanych stacji");
  }
}

// Funkcja przetwarza i zapisuje stację do pamięci PSRAM
void sanitizeAndSaveStation(const char *station) {
  // Bufor na przetworzoną stację - o jeden znak dłuższy niż maksymalna długość linku
  char sanitizedStation[STATION_NAME_LENGTH + 1];

  // Indeks pomocniczy dla przetwarzania
  int j = 0;

  // Przeglądaj każdy znak stacji i sprawdź czy jest to drukowalny znak ASCII
  for (int i = 0; i < STATION_NAME_LENGTH && station[i] != '\0'; i++) {
    // Sprawdź, czy znak jest drukowalnym znakiem ASCII
    if (isprint(station[i])) {
      // Jeśli tak, dodaj do przetworzonej stacji
      sanitizedStation[j++] = station[i];
    }
  }

  // Dodaj znak końca ciągu do przetworzonej stacji
  sanitizedStation[j] = '\0';

  // Zapisz przetworzoną stację do pamięci PSRAM
  saveStationToPSRAM(sanitizedStation);
}

// Odczyt banku z PIFFS lub karty SD (jesli dany bank istnieje juz na tej karcie)
void readSDStations() 
{
  stationsCount = 0;
  Serial.println("debuf SD -> Plik Banu isnieje lokalnie, czytamy TYLKO z karty");
  mp3 = flac = aac = vorbis = opus = false;
  stationString.remove(0);  // Usunięcie wszystkich znaków z obiektu stationString

  // Tworzymy nazwę pliku banku
  String fileName = String("/bank") + (bank_nr < 10 ? "0" : "") + String(bank_nr) + ".txt";

  // Sprawdzamy, czy plik istnieje
  if (!STORAGE.exists(fileName)) 
  {
    Serial.println("Błąd: Plik banku nie istnieje.");
    return;
  }

  // Otwieramy plik w trybie do odczytu
  File bankFile = STORAGE.open(fileName, FILE_READ);
  if (!bankFile)  // jesli brak pliku to...
  {
    Serial.println("Błąd: Nie można otworzyć pliku banku.");
    return;
  }

  // Przechodzimy do odpowiedniego wiersza pliku
  int currentLine = 0;
  String stationUrl = "";

  while (bankFile.available())  // & currentLine <= MAX_STATIONS)
  {
    //if (currentLine < MAX_STATIONS)
    //{
    String line = bankFile.readStringUntil('\n');
    currentLine++;

    stationName = line.substring(0, 42);
    int urlStart = line.indexOf("http");  // Szukamy miejsca, gdzie zaczyna się URL
    if (urlStart != -1) 
    {
      stationUrl = line.substring(urlStart);  // Wyciągamy URL od "http"
      stationUrl.trim();                      // Usuwamy białe znaki na początku i końcu
      //Serial.print(" URL stacji:");
      //Serial.println(stationUrl);
      //String station = currentLine + "   " + stationName + "  " + stationUrl;
      String station = stationName + "  " + stationUrl;
      sanitizeAndSaveStation(station.c_str());  // przepisanie stacji do EEPROMu  (RAMU)
    }
  }

  Serial.print("Zamykamy plik bankFile na wartosci currentLine:");
  Serial.println(currentLine);
  bankFile.close();  // Zamykamy plik po odczycie
}

// Funkcja do pobierania listy stacji radiowych z serwera
/*
void fetchStationsFromServer() 
{
  displayActive = true;
  u8g2.setFont(spleen6x12PL);
  u8g2.clearBuffer();
  u8g2.setCursor(21, 23);
  u8g2.print("Loading bank:" + String(bank_nr) + " stations from:");
  u8g2.sendBuffer();
  
  currentSelection = 0;
  firstVisibleLine = 0;
  station_nr = 1;
  previous_bank_nr = bank_nr; // jesli ładujemy stacje to ustawiamy zmienna previous_bank

  // Utwórz obiekt klienta HTTP
  HTTPClient http;

  // URL stacji dla danego banku
  String url;
 
  // Wybierz URL na podstawie bank_nr za pomocą switch
  switch (bank_nr) {
    case 1: url = STATIONS_URL1; break;
    case 2: url = STATIONS_URL2;
      break;
    case 3:
      url = STATIONS_URL3;
      break;
    case 4:
      url = STATIONS_URL4;
      break;
    case 5:
      url = STATIONS_URL5;
      break;
    case 6:
      url = STATIONS_URL6;
      break;
    case 7:
      url = STATIONS_URL7;
      break;
    case 8:
      url = STATIONS_URL8;
      break;
    case 9:
      url = STATIONS_URL9;
      break;
    case 10:
      url = STATIONS_URL10;
      break;
    case 11:
      url = STATIONS_URL11;
      break;
    case 12:
      url = STATIONS_URL12;
      break;
    case 13:
      url = STATIONS_URL13;
      break;
    case 14:
      url = STATIONS_URL14;
      break;
    case 15:
      url = STATIONS_URL15;
      break;
    case 16:
      url = STATIONS_URL16;
      break;
    case 17:
      url = STATIONS_URL17;  // Bank niewykorzystany
      break;
    case 18:
      url = STATIONS_URL18;  // Bank niewykorzystany
      break;    
    default:
      Serial.println("Nieprawidłowy numer banku");
      return;
  }

  // Tworzenie nazwy pliku dla danego banku
  String fileName = String("/bank") + (bank_nr < 10 ? "0" : "") + String(bank_nr) + ".txt";

  // Sprawdzenie, czy plik istnieje
  if (STORAGE.exists(fileName) && bankNetworkUpdate == false) 
  {
    Serial.println("debug SD -> Plik banku " + fileName + " już istnieje.");
    u8g2.setFont(spleen6x12PL);
    //u8g2.drawStr(147, 23, "SD card");
    if (useSD) {u8g2.print("SD Card");} else if (!useSD) {u8g2.print("SPIFFS");}
    u8g2.sendBuffer();
    readSDStations();  // Jesli dany plik banku istnieje to odczytujemy go TYLKO z karty
  } 
  else
  {
    bankNetworkUpdate = false;
    // stworz plik na karcie tylko jesli on nie istnieje GR
    //u8g2.drawStr(205, 23, "GitHub server");
    u8g2.print("GitHub");
    u8g2.sendBuffer();
    {
      // Próba utworzenia pliku, jeśli nie istnieje
      File bankFile = STORAGE.open(fileName, FILE_WRITE);

      if (bankFile) 
      {
        Serial.println("debug SD -> Utworzono plik banku: " + fileName);
        bankFile.close();  // Zamykanie pliku po utworzeniu
      } 
      else 
      {
        Serial.println("debug SD -> Błąd: Nie można utworzyć pliku banku: " + fileName);
      //  return;  // Przerwij dalsze działanie, jeśli nie udało się utworzyć pliku
      }
    }
    // Inicjalizuj żądanie HTTP do podanego adresu URL
    http.begin(url);

    // Wykonaj żądanie GET i zapisz kod odpowiedzi HTTP
    int httpCode = http.GET();

    // Wydrukuj dodatkowe informacje diagnostyczne
    Serial.print("debug http -> Kod odpowiedzi HTTP: ");
    Serial.println(httpCode);

    // Sprawdź, czy żądanie było udane (HTTP_CODE_OK)
    if (httpCode == HTTP_CODE_OK)
    {
      // Pobierz zawartość odpowiedzi HTTP w postaci tekstu
      String payload = http.getString();
      //Serial.println("Stacje pobrane z serwera:");
      //Serial.println(payload);  // Wyświetlenie pobranych danych (payload)
      // Otwórz plik w trybie zapisu, aby zapisać payload
      File bankFile = STORAGE.open(fileName, FILE_WRITE);
      if (bankFile) {
        bankFile.println(payload);  // Zapisz dane do pliku
        bankFile.close();           // Zamknij plik po zapisaniu
        Serial.println("debug SD -> Dane zapisane do pliku: " + fileName);
      } 
      else 
      {
        Serial.println("debug SD -> Błąd: Nie można otworzyć pliku do zapisu: " + fileName);
      }
      // Zapisz każdą niepustą stację do pamięci EEPROM z indeksem
      int startIndex = 0;
      int endIndex;
      stationsCount = 0;
      // Przeszukuj otrzymaną zawartość w poszukiwaniu nowych linii
      while ((endIndex = payload.indexOf('\n', startIndex)) != -1 && stationsCount < MAX_STATIONS) {
        // Wyodrębnij pojedynczą stację z otrzymanego tekstu
        String station = payload.substring(startIndex, endIndex);

        // Sprawdź, czy stacja nie jest pusta, a następnie przetwórz i zapisz
        if (!station.isEmpty()) {
          // Zapisz stację do pliku na karcie SD
          sanitizeAndSaveStation(station.c_str());
        }
        // Przesuń indeks początkowy do kolejnej linii
        startIndex = endIndex + 1;
      }
    } else {
      // W przypadku nieudanego żądania wydrukuj informację o błędzie z kodem HTTP
      Serial.printf("debug http -> Błąd podczas pobierania stacji. Kod HTTP: %d\n", httpCode);
    }
    // Zakończ połączenie HTTP
    http.end();
    
  }
  wsRefreshPage(); // Po odczytaniu banku odswiezamy WebSocket aby zaktualizować strone www
  
}
*/

/*
void fetchStationsFromServer() 
{
  displayActive = true;
  u8g2.setFont(spleen6x12PL);
  u8g2.clearBuffer();
  u8g2.setCursor(21, 23);
  u8g2.print("Loading bank:" + String(bank_nr) + " stations from:");
  u8g2.sendBuffer();

  currentSelection = 0;
  firstVisibleLine = 0;
  station_nr = 1;
  previous_bank_nr = bank_nr;

  HTTPClient http;
  String url;

  switch (bank_nr) {
    case 1:  url = STATIONS_URL1; break;
    case 2:  url = STATIONS_URL2; break;
    case 3:  url = STATIONS_URL3; break;
    case 4:  url = STATIONS_URL4; break;
    case 5:  url = STATIONS_URL5; break;
    case 6:  url = STATIONS_URL6; break;
    case 7:  url = STATIONS_URL7; break;
    case 8:  url = STATIONS_URL8; break;
    case 9:  url = STATIONS_URL9; break;
    case 10: url = STATIONS_URL10; break;
    case 11: url = STATIONS_URL11; break;
    case 12: url = STATIONS_URL12; break;
    case 13: url = STATIONS_URL13; break;
    case 14: url = STATIONS_URL14; break;
    case 15: url = STATIONS_URL15; break;
    case 16: url = STATIONS_URL16; break;
    case 17: url = STATIONS_URL17; break;
    case 18: url = STATIONS_URL18; break;
    default:
      Serial.println("Nieprawidłowy numer banku");
      return;
  }

  String fileName = String("/bank") + (bank_nr < 10 ? "0" : "") + String(bank_nr) + ".txt";

  if (STORAGE.exists(fileName) && bankNetworkUpdate == false) 
  {
    Serial.println("debug SD -> Plik banku " + fileName + " już istnieje.");
    u8g2.setFont(spleen6x12PL);
    if (useSD) { u8g2.print("SD Card"); } 
    else { u8g2.print("SPIFFS"); }
    u8g2.sendBuffer();
    
    readSDStations();
    wsRefreshPage();
    return;
  }

  bankNetworkUpdate = false;

  u8g2.print("GitHub");
  u8g2.sendBuffer();

  // Utwórz pusty plik (żeby mieć pewność, że istnieje)
  {
    File bankFile = STORAGE.open(fileName, FILE_WRITE);
    if (bankFile) {
      Serial.println("debug SD -> Utworzono plik banku: " + fileName);
      bankFile.close();
    } else {
      Serial.println("debug SD -> Błąd tworzenia pliku: " + fileName);
    }
  }

  // ---- START NOWEGO BEZPIECZNEGO POBIERANIA ----

  Serial.println("debug http -> Pobieram URL: " + url);

  http.begin(url);
  http.setUserAgent("ESP32-WebRadio");

  int httpCode = http.GET();
  Serial.print("debug http -> Kod odpowiedzi HTTP: ");
  Serial.println(httpCode);

  // Obsługa redirectów 301/302
  if (httpCode == HTTP_CODE_MOVED_PERMANENTLY || httpCode == HTTP_CODE_FOUND) {
    String newUrl = http.getLocation();
    Serial.println("debug http -> Redirect na: " + newUrl);
    http.end();

    http.begin(newUrl);
    http.setUserAgent("ESP32-WebRadio");
    httpCode = http.GET();
    Serial.println("debug http -> Kod HTTP po redirect: " + String(httpCode));
  }

  if (httpCode != HTTP_CODE_OK) {
    Serial.println("debug http -> Błąd pobierania, kod: " + String(httpCode));
    http.end();
    return;
  }

  // Pobieranie strumieniowe (super stabilne)
  WiFiClient *stream = http.getStreamPtr();
  File bankFile = STORAGE.open(fileName, FILE_WRITE);

  if (!bankFile) {
    Serial.println("debug SD -> Błąd otwarcia pliku do zapisu!");
    http.end();
    return;
  }

  uint8_t buffer[256];
  int total = 0;
  int len = http.getSize();

  while (http.connected() && (len > 0 || len == -1)) {
    size_t sizeAvail = stream->available();
    if (sizeAvail) {
      int c = stream->readBytes(buffer, min((int)sizeAvail, 256));
      bankFile.write(buffer, c);
      total += c;
      if (len > 0) len -= c;
    }
    delay(1);
  }

  bankFile.close();
  http.end();

  Serial.println("debug SD -> Zapisano " + String(total) + " bajtów");

  // Jeśli plik jest pusty → usuń
  File check = STORAGE.open(fileName, FILE_READ);
  if (check && check.size() == 0) {
    Serial.println("debug SD -> BŁĄD: pobrany plik ma 0 KB -> usuwam");
    check.close();
    STORAGE.remove(fileName);
    return;
  }
  check.close();

  // ----- PARSOWANIE (Twój kod – zostawiłem) -----

  File fileRead = STORAGE.open(fileName, FILE_READ);
  if (!fileRead) {
    Serial.println("debug SD -> Nie mogę otworzyć pliku po pobraniu");
    return;
  }

  String payload = fileRead.readString();
  fileRead.close();

  int startIndex = 0;
  int endIndex;
  stationsCount = 0;

  while ((endIndex = payload.indexOf('\n', startIndex)) != -1 && stationsCount < MAX_STATIONS) {
    String station = payload.substring(startIndex, endIndex);
    if (!station.isEmpty()) sanitizeAndSaveStation(station.c_str());
    startIndex = endIndex + 1;
  }

  wsRefreshPage();
}
*/

void fetchStationsFromServer() 
{
  displayActive = true;
  u8g2.setFont(spleen6x12PL);
  u8g2.clearBuffer();
  u8g2.setCursor(21, 23);
  u8g2.print("Loading bank:" + String(bank_nr) + " from:");
  u8g2.sendBuffer();

  currentSelection = 0;
  firstVisibleLine = 0;
  station_nr = 1;
  previous_bank_nr = bank_nr; // jesli ładujemy stacje to ustawiamy zmienna previous_bank

  HTTPClient http; // Utwórz obiekt klienta HTTP
  String url; // URL stacji dla danego banku

  // ---------------------- WYBÓR URL BANKU ----------------------
  switch (bank_nr) 
  {
    case 1:  url = STATIONS_URL1; break;
    case 2:  url = STATIONS_URL2; break;
    case 3:  url = STATIONS_URL3; break;
    case 4:  url = STATIONS_URL4; break;
    case 5:  url = STATIONS_URL5; break;
    case 6:  url = STATIONS_URL6; break;
    case 7:  url = STATIONS_URL7; break;
    case 8:  url = STATIONS_URL8; break;
    case 9:  url = STATIONS_URL9; break;
    case 10: url = STATIONS_URL10; break;
    case 11: url = STATIONS_URL11; break;
    case 12: url = STATIONS_URL12; break;
    case 13: url = STATIONS_URL13; break;
    case 14: url = STATIONS_URL14; break;
    case 15: url = STATIONS_URL15; break;
    case 16: url = STATIONS_URL16; break;
    case 17: url = STATIONS_URL17; break;
    case 18: url = STATIONS_URL18; break;
    default:
      Serial.println("Nieprawidłowy numer banku");
      return;
  }

  // Tworzenie nazwy pliku dla danego banku
  String fileName = String("/bank") + (bank_nr < 10 ? "0" : "") + String(bank_nr) + ".txt";

  // ---------------------- JEŚLI PLIK ISTNIEJE ----------------------
  if (STORAGE.exists(fileName) && bankNetworkUpdate == false) 
  {
    Serial.println("debug SD -> Plik banku " + fileName + " już istnieje.");
    //if (useSD) { u8g2.print("SD Card"); } else { u8g2.print("SPIFFS"); }
    u8g2.print(storageTextName); 
    u8g2.sendBuffer();

    readSDStations(); // Jesli dany plik banku istnieje to odczytujemy go TYLKO z karty
    wsRefreshPage();
    return;
  }

  bankNetworkUpdate = false;

  u8g2.print("GitHub");
  u8g2.sendBuffer();

  // ---------------------- TWORZENIE PUSTEGO PLIKU ----------------------
  {
    File bankFile = STORAGE.open(fileName, FILE_WRITE);
    if (bankFile) 
    {
      Serial.println("debug SD -> Utworzono plik banku: " + fileName);
      bankFile.close(); // Zamykanie pliku po utworzeniu
    } 
    else 
    {
      Serial.println("debug SD -> Błąd tworzenia pliku: " + fileName);
    }
  }

  // ---------------------- POBIERANIE DANYCH HTTP ----------------------

  Serial.println("debug http -> Pobieram URL: " + url);

  http.begin(url); // Inicjalizuj żądanie HTTP do podanego adresu URL
  http.setUserAgent("ESP32-WebRadio");   // ustaweinie userAgent
  
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.useHTTP10(true);                 // <<< NAJWAŻNIEJSZE
  http.addHeader("Connection", "close");
  
  http.setTimeout(5000);                // 5 sekund timeout
  
  http.setConnectTimeout(3000);

  int httpCode = http.GET();  
  Serial.print("debug http -> Kod HTTP: ");
  Serial.println(httpCode); // Wydrukuj dodatkowe informacje o kodzie http

  // ----------- OBSŁUGA REDIRECTÓW 301 / 302 (GitHub to często robi!) -----------
  if (httpCode == HTTP_CODE_MOVED_PERMANENTLY || httpCode == HTTP_CODE_FOUND) // jesli GitHub ustawił redirect
  {
    String newUrl = http.getLocation();
    Serial.println("debug http -> Redirect: " + newUrl);

    http.end();

    http.begin(newUrl);
    http.setUserAgent("ESP32-WebRadio");
  
    http.useHTTP10(true);                 // <<< NAJWAŻNIEJSZE
    http.addHeader("Connection", "close");
    
    http.setTimeout(5000);                // 5 sekund timeout
  
    http.setConnectTimeout(3000);

    httpCode = http.GET();
    Serial.println("debug http -> Kod HTTP po redirect: " + String(httpCode));
  }

  // ---------------------- NIE UDAŁO SIĘ POBRAĆ ----------------------
  if (httpCode != HTTP_CODE_OK) 
  {
    Serial.printf("debug http -> Błąd pobierania. Kod HTTP: %d\n", httpCode);
    http.end();
    return;
  }

  // ---------------------- POBRANIE GETSTRING ----------------------
  String payload = http.getString();

  Serial.println("debug http -> Długość pobranych danych: " + String(payload.length()));

  // Zabezpieczenie przed pustym stringiem
  if (payload.length() == 0) 
  {
    Serial.println("debug http -> BŁĄD! Pobieranie zwróciło pusty payload. Usuwam plik.");
    STORAGE.remove(fileName);
    http.end();
    return;
  }

  // ---------------------- ZAPIS DO PLIKU ----------------------
  File bankFile = STORAGE.open(fileName, FILE_WRITE);
  if (bankFile) 
  {
    bankFile.print(payload);
    bankFile.close();
    Serial.println("debug SD -> Dane zapisane do pliku: " + fileName);
  } 
  else 
  {
    Serial.println("debug SD -> Błąd: Nie można otworzyć pliku do zapisu!");
  }

  http.end();

  // ---------------------- SANITYZACJA STACJI  ----------------------
  int startIndex = 0;
  int endIndex;
  stationsCount = 0;

  while ((endIndex = payload.indexOf('\n', startIndex)) != -1 && stationsCount < MAX_STATIONS) 
  {
    String station = payload.substring(startIndex, endIndex);
    if (!station.isEmpty())
      sanitizeAndSaveStation(station.c_str());

    startIndex = endIndex + 1;
  }

  wsRefreshPage();
}


void readEEPROM() // Funkcja kontrolna DEBUG odczytu EEPROMu, nie uzywan przez inne funkcje
{
  EEPROM.get(0, station_nr);
  EEPROM.get(1, bank_nr);
  EEPROM.get(2, volumeValue); 
}


void calcNec() // Funkcja umozliwajaca przeliczanie odwrotne aby "udawac" przyciski pilota IR w standardzie NEC
{
  //składamy kod pilota do postaci ADDR/CMD/CMD/ADDR aby miec 4 bajty
  uint8_t CMD = (ir_code >> 8) & 0xFF; 
  uint8_t ADDR = ir_code & 0xFF;        
  ir_code =  ADDR;
  ir_code = (ir_code << 8) | CMD;
  ir_code = (ir_code << 8) | CMD;
  ir_code = (ir_code << 8) | ADDR;
  ADDR = (ir_code >> 24) & 0xFF;           // Pierwszy bajt
  uint8_t IADDR = (ir_code >> 16) & 0xFF; // Drugi bajt (inwersja adresu)
  CMD = (ir_code >> 8) & 0xFF;            // Trzeci bajt (komenda)
  uint8_t ICMD = ir_code & 0xFF;          // Czwarty bajt (inwersja komendy)
  
  // Dorabiamy brakujące odwórcone bajty 
  IADDR = IADDR ^ 0xFF;
  ICMD = ICMD ^ 0xFF;

  // Składamy bajty w jeden ciąg          
  ir_code =  ICMD;
  ir_code = (ir_code << 8) | ADDR;
  ir_code = (ir_code << 8) | IADDR;
  ir_code = (ir_code << 8) | CMD;
  ir_code = reverse_bits(ir_code,32);     // rotacja bitów do porządku LSB-MSB jak w NEC        
}


// Funkcja formatowania dla scorllera stationString/stationName  **** stationStringScroll ****
void stationStringFormatting() 
{
  if (displayMode == 0)
  {   
    if (stationString == "") // Jeżeli stationString jest pusty i stacja go nie nadaje to podmieniamy pusty stationString na nazwę staji - stationNameStream
    {    
      if (stationNameStream == "") // jezeli nie ma równiez stationName to wstawiamy 3 kreseczki
      { 
        stationStringScroll = "---" ;
        stationStringWeb = "---" ;
      } 
      else // jezeli jest station name to prawiamy w "-- NAZWA --" i wysylamy do scrollera
      { 
        stationStringScroll = ("-- " + stationNameStream + " --");
        stationStringWeb = ("-- " + stationNameStream + " --");
      }  // Zmienna stationStringScroller przyjmuje wartość stationNameStream
    }
    else // Jezeli stationString zawiera dane to przypisujemy go do stationStringScroll do funkcji scrollera
    {
      stationStringWeb = stationString;
      processText(stationString);  // przetwarzamy polsie znaki
      stationStringScroll = stationString + "    "; // dodajemy separator do przewijanego tekstu jesli się nie miesci na ekranie
    }             
    
    //Liczymy długość napisu stationStringScroll 
    stationStringScrollWidth = stationStringScroll.length() * 6;    
    if (f_debug_on) 
    {
      Serial.print("debug -> StationStringScroll Lenght [chars]:");  Serial.println(stationStringScroll.length());
      Serial.print("debug -> StationStringScroll Width (Lenght * 6px) [px]:"); Serial.println(stationStringScrollWidth);
      
      Serial.print("debug -> Display Mode-0 stationStringScroll Text: @");
      Serial.print(stationStringScroll);
      Serial.println("@");
    }
  }
  // ------ DUZY ZEGAR -------
  else if (displayMode == 1) // Tryb wświetlania zegara z 1 linijką radia na dole
  {
    char StationNrStr[4];
    snprintf(StationNrStr, sizeof(StationNrStr), "%02d", station_nr);  //Formatowanie informacji o stacji i banku do postaci 00
    
    if (urlPlaying) {
      strncpy(StationNrStr, "URL", sizeof(StationNrStr) - 1);
      StationNrStr[sizeof(StationNrStr) - 1] = '\0';  
    }

    int StationNameEnd = stationName.indexOf("  "); // Wycinamy nazwe stacji tylko do miejsca podwojnej spacji 
    stationName = stationName.substring(0, StationNameEnd);
 
    if (stationString == "")                // Jeżeli stationString jest pusty i stacja go nie nadaje
    {   
      if (stationNameStream == "")          // i jezeli nie ma równiez stationName
      {
        stationStringScroll = String(StationNrStr) + "." + stationName + ", ---" ;
        stationStringWeb = "---" ;
      }      // wstawiamy trzy kreseczki do wyswietlenia
      else  // jezeli jest brak "stationString" ale jest "stationName" to składamy NR.Nazwa stacji z pliku, nadawany stationNameStream + separator przerwy
      {       
        stationStringScroll = String(StationNrStr) + "." + stationName + ", " + stationNameStream + "     ";
        //stationStringScroll = String(StationNrStr) + "." + stationName;

        // Obcinnanie scrollera do 43 znakow - ABY BYŁ STAŁY TEKST mode 1
        //if (stationStringScroll.length() > 43) {stationStringScroll = stationStringScroll.substring(0,40) + "..."; }  
        stationStringWeb = stationNameStream;
      }
    }
    else //stationString != "" -> ma wartość
    {
      stationStringWeb = stationString;
      processText(stationString);  // przetwarzamy polsie znaki
      
      stationStringScroll = String(StationNrStr) + "." + stationName + ", " + stationString + "     "; 
      //stationStringScroll = String(StationNrStr) + "." + stationName; 
      
      // Obcinnanie scrollera do 43 znakow - ABY BYŁ STAŁY TEKST mode 1
      //stationStringScroll = String(StationNrStr) + "." + stationName + ", " + stationString; 
      //if (stationStringScroll.length() > 43) {stationStringScroll = stationStringScroll.substring(0,40) + "..."; }
      
      //Serial.println(stationStringScroll);
    }
    //Serial.print("debug -> Display1 (zegar) stationStringScroll: ");
    //Serial.println(stationStringScroll);

    //Liczymy długość napisu stationStringScrollWidth 
    stationStringScrollWidth = stationStringScroll.length() * 6;
  }
  else if (displayMode == 2) // Tryb wświetlania mode 2 - 3 linijki tekstu
  {             
    // Jesli stacja nie nadaje stationString to podmieniamy pusty stationString na nazwę staji - stationNameStream
    if (stationString == "") // Jeżeli stationString jest pusty i stacja go nie nadaje
    {    
      if (stationNameStream == "") // jezeli nie ma równiez stationName
      { 
        stationStringScroll = "---" ;
        stationStringWeb = "---" ;
      } // wstawiamy trzy kreseczki do wyswietlenia
      else // jezeli jest station name to oprawiamy w "-- NAZWA --" i wysylamy do scrollera
      { 
        stationStringScroll = ("-- " + stationNameStream + " --");
        stationStringWeb = stationNameStream;
      }  // Zmienna stationStringScroller przyjmuje wartość stationNameStream
    }
    else // Jezeli stationString zawiera dane to przypisujemy go do stationStringScroll do funkcji scrollera
    {
      stationStringWeb = stationString;
      processText(stationString);  // przetwarzamy polsie znaki
      stationStringScroll = stationString;
    }  
  }
  else if (displayMode == 3) //|| displayMode == 8 Tryb wświetlania mode 3 i mode 8 (małe spectrum)
  {
    if (stationString == "") // Jeżeli stationString jest pusty i stacja go nie nadaje to podmieniamy pusty stationString na nazwę staji - stationNameStream
    {    
      if (stationNameStream == "") // jezeli nie ma równiez stationName to wstawiamy 3 kreseczki
      { 
        stationStringScroll = "---" ;
        stationStringWeb = "---" ;
      } 
      else // jezeli jest station name to oprawiamy w "-- NAZWA --" i wysylamy do scrollera
      { 
        stationStringScroll = ("-- " + stationNameStream + " --");
        stationStringWeb = ("-- " + stationNameStream + " --");
      }  // Zmienna stationStringScroller przyjmuje wartość stationNameStream
    }
    else // Jezeli stationString zawiera dane to przypisujemy go do stationStringScroll do funkcji scrollera
    {
      stationStringWeb = stationString;
      processText(stationString);  // przetwarzamy polsie znaki
      stationStringScroll = "  " + stationString + "  " ; // Nie dodajemy separator do tekstu aby wyswietlał się rowno na srodku
    }             
    //Liczymy długość napisu stationStringScroll 
    stationStringScrollWidth = stationStringScroll.length() * 6;
    //Serial.print("debug -> Display Mode-3 stationStringScroll:@");
    //Serial.print(stationStringScroll);
    //Serial.println("@");
  }
  else if (displayMode == 4) // Tryb wświetlania mode 4 formater dla potrzeb Web
  {
    if (stationString == "") // Jeżeli stationString jest pusty i stacja go nie nadaje to podmieniamy pusty stationString na nazwę staji - stationNameStream
    {    
      if (stationNameStream == "") // jezeli nie ma równiez stationName to wstawiamy 3 kreseczki
      { 
        stationStringScroll = "---" ;
        stationStringWeb = "---" ;
      } 
      else // jezeli jest station name to oprawiamy w "-- NAZWA --" i wysylamy do scrollera
      { 
        stationStringScroll = ("-- " + stationNameStream + " --");
        stationStringWeb = ("-- " + stationNameStream + " --");
      }  // Zmienna stationStringScroller przyjmuje wartość stationNameStream
    }
    else // Jezeli stationString zawiera dane to przypisujemy go do stationStringScroll do funkcji scrollera
    {
      stationStringWeb = stationString;
      processText(stationString);  // przetwarzamy polsie znaki
      stationStringScroll = "  " + stationString + "  " ; // Nie dodajemy separator do tekstu aby wyswietlał się rowno na srodku
    }             
    //Liczymy długość napisu stationStringScroll 
    stationStringScrollWidth = stationStringScroll.length() * 6;
    //Serial.print("debug -> Display Mode-3 stationStringScroll:@");
    //Serial.print(stationStringScroll);
    //Serial.println("@");
  }
  // Styl 5: Analyzer mode
  else if (displayMode == 8)
  {   
    if (stationString == "") // Jeżeli stationString jest pusty i stacja go nie nadaje to podmieniamy pusty stationString na nazwę staji - stationNameStream
    {    
      if (stationNameStream == "") // jezeli nie ma równiez stationName to wstawiamy 3 kreseczki
      { 
        stationStringScroll = "---" ;
        stationStringWeb = "---" ;
      } 
      else // jezeli jest station name to prawiamy w "-- NAZWA --" i wysylamy do scrollera
      { 
        stationStringScroll = ("-- " + stationNameStream + " --");
        stationStringWeb = ("-- " + stationNameStream + " --");
      }  // Zmienna stationStringScroller przyjmuje wartość stationNameStream
    }
    else // Jezeli stationString zawiera dane to przypisujemy go do stationStringScroll do funkcji scrollera
    {
      stationStringWeb = stationString;
      processText(stationString);  // przetwarzamy polsie znaki
      stationStringScroll = stationString + "    "; // dodajemy separator do przewijanego tekstu jesli się nie miesci na ekranie
    }             
    
    stationStringScrollWidth = stationStringScroll.length() * 6;
  }

}

// Obsługa wyświetlacza dla odtwarzanego strumienia radia internetowego
void displayRadio() 
{
  // KRYTYCZNE: Blokuj displayRadio() gdy SDPlayer jest aktywny
  if (sdPlayerOLEDActive) {
    // Serial.println("[displayRadio] BLOCKED - SDPlayer active");
    return;
  }
  
  // DODATKOWA BLOKADA: Sprawdź czy obiekt SDPlayer OLED nie jest aktywny
  if (g_sdPlayerOLED && g_sdPlayerOLED->isActive()) {
    Serial.println("[displayRadio] BLOCKED - g_sdPlayerOLED->isActive() == true");
    return;
  }
  
  int StationNameEnd = stationName.indexOf("  "); // Wycinamy nazwe stacji tylko do miejsca podwojnej spacji 
  stationName = stationName.substring(0, StationNameEnd);

  if (displayMode == 0)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvB14_tr);
    u8g2.drawStr(24, 16, stationName.substring(0, stationNameLenghtCut - 1).c_str());
    u8g2.drawRBox(1, 1, 21, 16, 4);  // Biały kwadrat (tło) pod numerem stacji
        
    // Funkcja wyswietlania numeru Banku na dole ekranu
    u8g2.setFont(spleen6x12PL);
    char BankStr[8];  
    snprintf(BankStr, sizeof(BankStr), "B-%02d", bank_nr); // Formatujemy numer banku do postacji 00

    // Wyswietlamy numer Banku w dolnej linijce
    
    if (!urlPlaying) 
    {
      u8g2.drawBox(161, 54, 1, 12);  // dorysowujemy 1px pasek przed napisem "Bank" dla symetrii
      u8g2.setDrawColor(0);
      u8g2.setCursor(162, 63);  // Pozycja napisu Bank0x na dole ekranu
      u8g2.print(BankStr);
    } //else {u8g2.print("URL");}
    
    u8g2.setDrawColor(0);
    
    char StationNrStr[3];
    snprintf(StationNrStr, sizeof(StationNrStr), "%02d", station_nr);  //Formatowanie informacji o stacji i banku do postaci 00
                                                // Pozycja numeru stacji na gorze po lewej ekranu
    if (!urlPlaying) 
    {
      u8g2.setCursor(4, 14);
      u8g2.setFont(u8g2_font_spleen8x16_mr); 
      u8g2.print(StationNrStr);} 
    else 
    {
      u8g2.setCursor(3, 13);
      u8g2.setFont(spleen6x12PL);
      u8g2.print("URL");
    }
    
    u8g2.setDrawColor(1);
      
    // Logo 3xZZZ w trybie dla timera SLEEP
    if (f_sleepTimerOn) 
    {
      u8g2.setFont(u8g2_font_04b_03_tr); 
      u8g2.drawStr(189,63, "z");
      u8g2.drawStr(192,61, "z");
      u8g2.drawStr(195,59, "z");
    }
    
    stationStringFormatting(); //Formatujemy stationString wyswietlany przez funkcję Scrollera
    
    // Wskaźnik nagrywania (REC) w górnym prawym rogu
    if (g_sdRecorder && g_sdRecorder->isRecording()) {
      u8g2.setFont(spleen6x12PL);
      u8g2.drawStr(220, 12, "\x07REC");  // \x07 to kropka/bullet
      u8g2.drawCircle(223, 9, 3);         // Czerwona kropka (animacja)
    }
    
    u8g2.drawLine(0, 52, 255, 52); // Dolna linia rozdzielajaca
    
    u8g2.setFont(spleen6x12PL); 
    u8g2.drawStr(135, 63, streamCodec.c_str()); // dopisujemy kodek minimalnie przesuniety o 1px aby zmiescil sie napis numeru banku
    String displayString = String(SampleRate) + "." + String(SampleRateRest) + "kHz " + bitsPerSampleString + "bit " + bitrateString + "kbps";
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(0, 63, displayString.c_str());
  }
  
  else if (displayMode == 1) // ZEGAR - Tryb wświetlania zegara z 1 linijką radia na dole
  {
    u8g2.clearBuffer();
    u8g2.setDrawColor(1);
    u8g2.setFont(spleen6x12PL);
    u8g2.drawLine(0, 50, 255, 50); // Linia separacyjna zegar, dolna linijka radia

    // Logo 3xZZZ w trybie dla timera SLEEP
    if (f_sleepTimerOn) 
    {
      u8g2.setFont(u8g2_font_04b_03_tr); 
      u8g2.drawStr(200,47, "z");
      u8g2.drawStr(203,45, "z");
      u8g2.drawStr(206,43, "z");
      
    }
    
    // --- GŁOSNIKCZEK I POZIOM GŁOSOSCI ---
    u8g2.setFont(spleen6x12PL);
    u8g2.drawGlyph(215,47, 0x9E); // 0x9E w czionce Spleen to zakodowany symbol głosniczka
    u8g2.drawStr(223,47, String(volumeValue).c_str());

    stationStringFormatting(); //Formatujemy stationString wyswietlany przez funkcję Scrollera  
  }
  // Style 6-10: Analizatory spectrum (podobna obsługa jak styl 5)
  else if (displayMode >= 6 && displayMode <= 10)
  {
    u8g2.clearBuffer();
    u8g2.drawLine(128,0,128,64);
    
    u8g2.drawLine(13,5,46,5); 
    u8g2.drawLine(82,5,115,5);
    
    u8g2.setFont(spleen6x12PL);
    int stationNameWidth = u8g2.getStrWidth(stationName.substring(0, stationNameLenghtCut - 2).c_str());
    int stationNamePositionX = (127 - stationNameWidth) / 2;
    u8g2.drawStr(stationNamePositionX, 22, stationName.substring(0, stationNameLenghtCut - 2).c_str());
    
    u8g2.setFont(u8g2_font_04b_03_tr);
    char BankStr[8];  
    char StationNrStr[3];
    snprintf(BankStr, sizeof(BankStr), "%02d", bank_nr);
    snprintf(StationNrStr, sizeof(StationNrStr), "%02d", station_nr);
    
    u8g2.drawRBox(0, 0, 13, 10, 3);
    u8g2.drawRBox(115, 0, 13, 10, 3);
    
    if (!urlPlaying) 
    {
      u8g2.setDrawColor(0);
      u8g2.setCursor(117,8);
      u8g2.print(BankStr);
    }
    
    if (!urlPlaying) 
    {
      u8g2.setCursor(2, 8);
      u8g2.print(StationNrStr);
    } 
    else 
    {
      u8g2.setCursor(1, 10);
      u8g2.setFont(spleen6x12PL);
      u8g2.print("URL");
    }
    u8g2.setDrawColor(1);
    
    // PRZEKREŚLONY GŁOŚNIK PRZY MUTE (w górnym pasku)
    if (volumeMute) {
      u8g2.setFont(spleen6x12PL);
      u8g2.drawGlyph(60, 10, 0x9E); // Głośnik w środku górnego paska
      u8g2.drawLine(55, 0, 70, 12); // Linia przekreślająca /
    }
    
    if (f_sleepTimerOn) 
    {
      u8g2.setFont(u8g2_font_04b_03_tr); 
      u8g2.drawStr(189,63, "z");
      u8g2.drawStr(192,61, "z");
      u8g2.drawStr(195,59, "z");
    }
    
    stationStringFormatting();
  }
  // Styl 2: Nazwa stacji na górze z numerem stacji i banku
  else if (displayMode == 2)
  {
    u8g2.clearBuffer();
    u8g2.setFont(spleen6x12PL);
        
    if (!urlPlaying) 
    {
      u8g2.drawStr(23, 11, stationName.substring(0, stationNameLenghtCut).c_str()); // Przyciecie i wyswietlenie dzieki temu nie zmieniamy zawartosci zmiennej stationName
      u8g2.drawRBox(1, 1, 18, 13, 4);  // Rbox pod numerem stacji
    }
    else // gramy z URL powiekszamy pole od numer stacji aby zmiescil sie napis URL
    {
      u8g2.drawStr(29, 11, stationName.substring(0, stationNameLenghtCut).c_str()); // Przyciecie i wyswietlenie dzieki temu nie zmieniamy zawartosci zmiennej stationName
      u8g2.drawRBox(1, 1, 24, 13, 4);  // Rbox pod numerem stacji
    }

    // Funkcja wyswietlania numeru Banku na dole ekranu
    char BankStr[8];  
    snprintf(BankStr, sizeof(BankStr), "B-%02d", bank_nr); // Formatujemy numer banku do postacji 00

    // Wyswietlamy numer Banku w dolnej linijce
    if (!urlPlaying) 
    {
      u8g2.drawBox(161, 54, 1, 12);  // dorysowujemy 1px pasek przed napisem "Bank" dla symetrii
      u8g2.setDrawColor(0);
      u8g2.setCursor(162, 63);  // Pozycja napisu Bank0x na dole ekranu
      u8g2.print(BankStr);
    } //else {u8g2.print("URL");}
   
    u8g2.setDrawColor(0);
    char StationNrStr[3];
    snprintf(StationNrStr, sizeof(StationNrStr), "%02d", station_nr);  //Formatowanie informacji o stacji i banku do postaci 00
                                                // Pozycja numeru stacji na gorze po lewej ekranu
    if (!urlPlaying) { u8g2.setCursor(4, 11); u8g2.print(StationNrStr);} else { u8g2.setCursor(5, 12); u8g2.print("URL");}
    u8g2.setDrawColor(1);

    // Logo 3xZZZ w trybie dla timera SLEEP
    if (f_sleepTimerOn) 
    {
      u8g2.setFont(u8g2_font_04b_03_tr); 
      u8g2.drawStr(189,63, "z");
      u8g2.drawStr(192,61, "z");
      u8g2.drawStr(195,59, "z");
    }

    stationStringFormatting(); //Formatujemy stationString wyswietlany przez funkcję Scrollera

    u8g2.drawLine(0, 52, 255, 52);
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(135, 63, streamCodec.c_str()); // dopisujemy kodek minimalnie przesuniety o 1px aby zmiescil sie napis numeru banku
    String displayString = String(SampleRate) + "." + String(SampleRateRest) + "kHz " + bitsPerSampleString + "bit " + bitrateString + "kbps";
    u8g2.drawStr(0, 63, displayString.c_str());  
  }
  
  else if (displayMode == 3) // Tryb wświetlania mode 3 - linijka statusu (stacja, bank godzina) na gorze i na dole (format stream, wifi zasieg)
  {
    u8g2.clearBuffer();
        
    //-- "IKONA" SLEEP TIMER -- 
    if (f_sleepTimerOn)
    {
      u8g2.setFont(u8g2_font_04b_03_tr); 
      u8g2.drawStr(165,10, "z");
      u8g2.drawStr(168,8, "z");
      u8g2.drawStr(171,6, "z");
    }
    
    // -- IKONA VOLUME I WARTOSC --    
    u8g2.setFont(spleen6x12PL);
    u8g2.drawGlyph(180,10, 0x9E); // 0x9E w czionce Spleen to zakodowany symbol głosniczka
    u8g2.drawStr(189,10, String(volumeValue).c_str());        
    
    
    // -- LOGO FLAC/MP3/AACi BITRATE --
    u8g2.setFont(mono04b03b); // szerokosc 5, wysokosc 6
    uint8_t x_codec = 103;  // Koordynata X dla informacji o kodeku
    
    if (!f_simpleMode3) {x_codec = 47;}
    
    if (flac == true || opus == true) 
    {
      if (u8g2.getStrWidth(bitrateString.c_str()) > 19) {u8g2.drawFrame(x_codec,2,45,9);}  // 128 ->18px, regulujemy ranke w zaleznosci czy bitrate ma 4 czy 3 cyfry
      else { u8g2.drawFrame(x_codec,2,39,9);}
      
      u8g2.drawBox(x_codec+1,2,21,9);
      u8g2.setDrawColor(0);
      u8g2.drawStr(x_codec+2,9, streamCodec.c_str());
      u8g2.setDrawColor(1);
      u8g2.drawStr(x_codec+23,9, bitrateString.c_str());
    } else 
    {
      u8g2.drawFrame(x_codec+6,2,38,9);
      u8g2.drawBox(x_codec+6,2,19,9);
      u8g2.setDrawColor(0);
      u8g2.drawStr(x_codec+8,9, streamCodec.c_str());
      u8g2.setDrawColor(1);
      u8g2.drawStr(x_codec+27,9, bitrateString.c_str());
    }

    // -- WYSWIETL NUMER BANKU i STACJI --
    u8g2.setFont(spleen6x12PL);
    u8g2.setCursor(1, 10);
    
    if (!urlPlaying) // Jesli nie gramy z adresu URL wyslanego ze strony www
    {       
      char StationNrStr[3]; snprintf(StationNrStr, sizeof(StationNrStr), "%02d", station_nr);  //Formatowanie informacji o stacji i banku do postaci 00
      
      if (f_simpleMode3)
      {
        char BankStr[8]; snprintf(BankStr, sizeof(BankStr), "%1d", bank_nr); // Formatujemy numer banku
        u8g2.print("CH.");   
        u8g2.print(BankStr);
        //u8g2.print(".");
        u8g2.setFont(spleen6x12PL);
        u8g2.print(StationNrStr);
      }
      else
      {
        char BankStr[8]; snprintf(BankStr, sizeof(BankStr), "%02d", bank_nr); // Formatujemy numer banku do postacji 00
        u8g2.print("BANK:");
        u8g2.print(BankStr);     
        u8g2.setCursor(99, 10);
        u8g2.print("STATION:");
        u8g2.print(StationNrStr);
      }
    }
    else
    { 
      if (f_simpleMode3) {u8g2.print("URL");} else {u8g2.setCursor(119, 10); u8g2.print("URL");}
    }
   
    //u8g2.setFont(u8g2_font_helvB14_tr);
    //u8g2.setFont(u8g2_font_fub14_tr);
    //u8g2.setFont(u8g2_font_smart_patrol_nbp_tf);
    u8g2.setFont(u8g2_font_UnnamedDOSFontIV_tr);
    int stationNameWidth = u8g2.getStrWidth(stationName.substring(0, stationNameLenghtCut).c_str()); // Liczy pozycje aby wyswietlic stationName na wycentrowane środku
    int stationNamePositionX = (256 - stationNameWidth) / 2;
    
    u8g2.drawStr(stationNamePositionX, stationNamePositionYmode3, stationName.substring(0, stationNameLenghtCut).c_str()); // Przyciecie i wyswietlenie dzieki temu nie zmieniamy zawartosci zmiennej stationName

    stationStringFormatting(); //Formatujemy stationString wyswietlany przez funkcję Scrollera
    u8g2.setFont(spleen6x12PL);
  }
  else if (displayMode == 4) // Duze wskazniki VU - nie wyswietlamy zadnych informacji o stacji radiowej tylko wskazniki VU
  {
    u8g2.clearBuffer();
    u8g2.setFont(spleen6x12PL);
  }
  else if (displayMode == 5) // Mode 5 - VU analogowe z prostokątną skalą
  {
    u8g2.clearBuffer();
    u8g2.setFont(spleen6x12PL);
  }
  else if (displayMode == 7) // Mode 7 - Duże paski VU na cały ekran
  {
    u8g2.clearBuffer();
    stationStringFormatting(); 
  }
  else if (displayMode == 8) // Analyzer - VU Meter Mode 8 (był 5)
  {
    u8g2.clearBuffer();
    u8g2.setFont(spleen6x12PL);
       

    int stationNameWidth = u8g2.getStrWidth(stationName.substring(0, stationNameLenghtCut - 2).c_str()); // Liczy pozycje aby wyswietlic stationName na wycentrowane środku
    int stationNamePositionX = (127 - stationNameWidth) / 2;
    u8g2.drawStr(stationNamePositionX, 22, stationName.substring(0, stationNameLenghtCut - 2).c_str());
        
    // Funkcja wyswietlania numeru Banku na dole ekranu
    //u8g2.setFont(spleen6x12PL);
    //u8g2.setFont(mono04b03b);
    u8g2.setFont(u8g2_font_04b_03_tr);
    char BankStr[8];  
    char StationNrStr[3];
    snprintf(BankStr, sizeof(BankStr), "%02d", bank_nr); // Formatujemy numer banku do postacji 00
    snprintf(StationNrStr, sizeof(StationNrStr), "%02d", station_nr);  //Formatowanie informacji o stacji i banku do postaci 00
        
    u8g2.drawRBox(0, 0, 13, 10, 3);  // Biały kwadrat (tło) pod numerem stacji
    u8g2.drawRBox(115, 0, 13, 10, 3);  // Biały kwadrat (tło) pod numerem stacji
    if (!urlPlaying) 
    {
      //u8g2.drawBox(87, 58, 1, 5);  // dorysowujemy 1px pasek przed napisem "Bank" dla symetrii
      u8g2.setDrawColor(0);
      //u8g2.setCursor(88,63);  // Pozycja napisu Bank0x na dole ekranu
      u8g2.setCursor(117,8);  // Pozycja napisu Bank0x na dole ekranu
      u8g2.print(BankStr);
    }
    //u8g2.setDrawColor(1);
       
    // Pozycja numeru stacji na gorze po lewej ekranu
    if (!urlPlaying) 
    {
      //u8g2.setFont(spleen6x12PL);
      u8g2.setCursor(2, 8);
      //u8g2.print("Station:");  
      u8g2.print(StationNrStr);
    } 
    else 
    {
      u8g2.setCursor(1, 10);
      u8g2.setFont(spleen6x12PL);
      u8g2.print("URL");
    }
    u8g2.setDrawColor(1);
      
    // Logo 3xZZZ w trybie dla timera SLEEP
    if (f_sleepTimerOn) 
    {
      u8g2.setFont(u8g2_font_04b_03_tr); 
      u8g2.drawStr(189,63, "z");
      u8g2.drawStr(192,61, "z");
      u8g2.drawStr(195,59, "z");
    }
    
    stationStringFormatting(); //Formatujemy stationString wyswietlany przez funkcję Scrollera
    u8g2.drawLine(0, 54, 127, 54); // Dolna linia rozdzielajaca
    
    
    
    u8g2.setFont(u8g2_font_04b_03_tr); 
    //u8g2.drawStr(42, 63, String(bitrateString + "k").c_str());
    //u8g2.drawStr(65, 63, streamCodec.c_str()); // dopisujemy kodek minimalnie przesuniety o 1px aby zmiescil sie napis numeru banku
       

    String displayString = String(SampleRate) + "." + String(SampleRateRest) + "kHz " + bitsPerSampleString +"bit " + bitrateString + "kbps  ";
    //u8g2.setFont(u8g2_font_04b_03_tr);
    u8g2.setCursor(0, 63);
    //u8g2.drawStr(0, 63, displayString.c_str());
    u8g2.print(displayString);

    u8g2.setFont(mono04b03b);
    u8g2.print(String(streamCodec));
    
    // PRZEKREŚLONY GŁOŚNIK PRZY MUTE (w górnym pasku)
    if (volumeMute) {
      u8g2.setFont(spleen6x12PL);
      u8g2.drawGlyph(60, 10, 0x9E); // Głośnik w środku górnego paska
      u8g2.drawLine(55, 0, 70, 12); // Linia przekreślająca /
    }

  }


}


// Obsługa callbacka info o audio dla bibliteki 3.4.1 i nowszej.
void my_audio_info(Audio::msg_t m)
{
  switch(m.e)
  {
    case Audio::evt_info:  
    {
      String msg = String(m.msg);   // zamiana na String
      msg.trim(); // usuń spacje i \r\n
      // Sprawdź czy to błąd dekodera
      if (msg.indexOf("Error") != -1 || msg.indexOf("error") != -1) {
        Serial.printf("[AUDIO ERROR] %s\n", msg.c_str());
        // Reset analizatora przy błędzie
        eq_analyzer_reset();
      }

      // --- BitRate ---
      int bitrateIndex = msg.indexOf("Bitrate (b/s):");  
      if (bitrateIndex != -1)
      {
        int endIndex = msg.indexOf('\n', bitrateIndex);
        if (endIndex == -1) endIndex = msg.length();
        bitrateString = msg.substring(bitrateIndex + 14, endIndex);
        bitrateString.trim();

        //Przliczenie bps na Kbps
        bitrateStringInt = bitrateString.toInt();  
        bitrateStringInt = bitrateStringInt / 1000;
        bitrateString = String(bitrateStringInt); 
        
        f_audioInfoRefreshDisplayRadio = true;
        wsAudioRefresh = true;  //Web Socket - audio refresh
        Serial.printf("bitrate: .... %s\n", m.msg); // icy-bitrate or bitrate from metadata
      }
      
      // --- FLAC BitsPerSample ---
      int bitrateIndexFlac = msg.indexOf("FLAC bitspersample:");
      if ((bitrateIndexFlac != -1) && (flac == true))
      {
        Serial.printf("bitrate FLAC: .... %s\n", m.msg); // icy-bitrate or bitrate from metadata
        int endIndex = msg.indexOf('\n', bitrateIndexFlac);
        if (endIndex == -1) endIndex = msg.length();
        bitrateString = msg.substring(bitrateIndexFlac + 19, endIndex);  // POPRAWKA: bitrateIndexFlac zamiast bitrateIndex
        bitrateString.trim();

        // przliczenie bps na Kbps
        bitrateStringInt = bitrateString.toInt();  
        bitrateStringInt = bitrateStringInt / 1000;
        bitrateString = String(bitrateStringInt);
            
        f_audioInfoRefreshDisplayRadio = true;
        wsAudioRefresh = true;  //Web Socket - audio refresh 
      }


      // --- SampleRate ---
      int sampleRateIndex = msg.indexOf("SampleRate (Hz):");
      if (sampleRateIndex != -1)
      {
        int endIndex = msg.indexOf('\n', sampleRateIndex);
        if (endIndex == -1) endIndex = msg.length();
        //sampleRateString = msg.substring(sampleRateIndex + 11, endIndex);
        sampleRateString = msg.substring(sampleRateIndex + 17, endIndex);
        sampleRateString.trim();
           
        uint32_t fullSampleRate = sampleRateString.toInt();
        SampleRate = fullSampleRate;
        SampleRateRest = SampleRate % 1000;
        SampleRateRest = SampleRateRest / 100;
        SampleRate = SampleRate / 1000;
        
        // Ustaw sample rate w analizatorze
        eq_analyzer_set_sample_rate(fullSampleRate);
        Serial.printf("[AUDIO] Sample Rate detected: %u Hz\n", fullSampleRate);
        
        f_audioInfoRefreshDisplayRadio = true;
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }

      // --- BitsPerSample ---
      int bitsPerSampleIndex = msg.indexOf("BitsPerSample:");
      if (bitsPerSampleIndex != -1)
      {     
        int endIndex = msg.indexOf('\n', bitsPerSampleIndex);
        if (endIndex == -1) endIndex = msg.length();
        bitsPerSampleString = msg.substring(bitsPerSampleIndex + 15, endIndex);
        bitsPerSampleString.trim();
        
        uint8_t bitsPerSample = bitsPerSampleString.toInt();
        Serial.printf("[AUDIO] Bits per sample: %u\n", bitsPerSample);
        
        f_audioInfoRefreshDisplayRadio = true;	
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }
    
      // --- Rozpoznawanie dekodera / formatu ---
      if (msg.indexOf("MP3Decoder") != -1)
      {
        mp3 = true;
        flac = false; aac = false; vorbis = false; opus = false;
        streamCodec = "MP3";
        Serial.println("[AUDIO] Codec: MP3 detected");
        eq_analyzer_set_flac_mode(false);
        f_audioInfoRefreshDisplayRadio = true; // refresh displayRadio screen
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }
      if (msg.indexOf("FLACDecoder") != -1)
      {
        flac = true; 
        mp3 = false; aac = false; vorbis = false; opus = false;
        streamCodec = "FLAC";
        Serial.println("[AUDIO] Codec: FLAC detected");
        // Resetuj analizator dla nowego formatu i włącz tryb FLAC
        eq_analyzer_set_flac_mode(true);
        eq_analyzer_reset();
        f_audioInfoRefreshDisplayRadio = true; // refresh displayRadio screen
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }
      if (msg.indexOf("AACDecoder") != -1)
      {
        aac = true;
        flac = false; mp3 = false; vorbis = false; opus = false;
        streamCodec = "AAC";
        Serial.println("[AUDIO] Codec: AAC detected");
        // Resetuj analizator dla nowego formatu
        eq_analyzer_set_flac_mode(false);
        eq_analyzer_reset();
        f_audioInfoRefreshDisplayRadio = true; // refresh displayRadio screen
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }
      if (msg.indexOf("VORBISDecoder") != -1)
      {
        vorbis = true;
        aac = false; flac = false; mp3 = false; opus = false;
        streamCodec = "VRB";
        Serial.println("[AUDIO] Codec: VORBIS detected");
        eq_analyzer_set_flac_mode(false);
        eq_analyzer_reset();
        f_audioInfoRefreshDisplayRadio = true; // refresh displayRadio screen
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }
      if (msg.indexOf("OPUSDecoder") != -1)
      {
        opus = true;
        aac = false; flac = false; mp3 = false; vorbis = false;
        streamCodec = "OPUS";
        Serial.println("[AUDIO] Codec: OPUS detected");
        eq_analyzer_set_flac_mode(false);
        eq_analyzer_reset();
        f_audioInfoRefreshDisplayRadio = true; // refresh displayRadio screen
        wsAudioRefresh = true;  //Web Socket - audio refresh    
      }

      // --- Debug ---
      Serial.printf("info: ....... %s\n", m.msg);      
    }
    break;

    case Audio::evt_log:
      Serial.printf("[AUDIO LOG] %s\n", m.msg);
      // Sprawdź czy to błąd na podstawie treści logu
      if (strstr(m.msg, "error") || strstr(m.msg, "Error") || strstr(m.msg, "ERROR") ||
          strstr(m.msg, "failed") || strstr(m.msg, "Failed") || strstr(m.msg, "FAILED")) {
        eq_analyzer_reset();  // Reset analizatora przy błędzie
      }
      break;


    case Audio::evt_eof:
    {
      Serial.printf("end of file:  %s\n", m.msg);
      
      // Wyczyść metadane ID3 przed kolejnym utworem
      currentMP3Artist = "";
      currentMP3Title = "";
      currentMP3Album = "";
      
      // KRYTYCZNE: Najpierw sprawdź czy SDPlayer jest w trybie odtwarzania muzyki
      if (sdPlayerPlayingMusic) {
        // KRYTYCZNE FIX: NIE wywołuj audio.connecttoFS() bezpośrednio w callbacku evt_eof!
        // To powoduje watchdog timeout bo callback działa w wątku audio.
        // Zamiast tego ustaw TYLKO flagę - obsługa w loop()
        
        Serial.println("[SDPlayer] Koniec utworu - żądanie auto-play w loop()");
        sdPlayerAutoPlayNext = true; // Ustaw flagę dla obu trybów (pilot i WebUI)
      }
      else if (resumePlay == true)
      {
        ir_code = rcCmdOk; // Przypisujemy kod pilota - OK
        bit_count = 32;
        calcNec();        // Przeliczamy kod pilota na pełny kod NEC
        resumePlay = false;
      }
      
    }
    break;

    case Audio::evt_bitrate:
    {        
      bitrateString = String(m.msg);
      bitrateString.trim();
      
      // przliczenie bps na Kbps
      bitrateStringInt = bitrateString.toInt();  
      bitrateStringInt = bitrateStringInt / 1000;
      bitrateString = String(bitrateStringInt);
      
      f_audioInfoRefreshDisplayRadio = true;
      wsAudioRefresh = true;  //Web Socket - audio refresh
      Serial.printf("info: ....... evt_bitrate: %s\n", m.msg); break; // icy-bitrate or bitrate from metadata
    }
    case Audio::evt_icyurl:         Serial.printf("icy URL: .... %s\n", m.msg); break;
    case Audio::evt_id3data:
    {
      String id3Data = String(m.msg);
      id3Data.trim();
      Serial.printf("ID3 data: ... %s\n", m.msg);
      
      // Parsuj dane ID3 - format może być: "Artist: xxx", "Title: xxx", "Album: xxx" lub "TPE1=xxx", "TIT2=xxx", "TALB=xxx"
      if (id3Data.startsWith("Artist: ") || id3Data.startsWith("TPE1=")) {
        int startPos = id3Data.indexOf(": ");
        if (startPos < 0) startPos = id3Data.indexOf("=");
        if (startPos >= 0) {
          currentMP3Artist = id3Data.substring(startPos + 1);
          currentMP3Artist.trim();
          Serial.printf("[ID3] Artist: %s\n", currentMP3Artist.c_str());
        }
      }
      else if (id3Data.startsWith("Title: ") || id3Data.startsWith("TIT2=")) {
        int startPos = id3Data.indexOf(": ");
        if (startPos < 0) startPos = id3Data.indexOf("=");
        if (startPos >= 0) {
          currentMP3Title = id3Data.substring(startPos + 1);
          currentMP3Title.trim();
          Serial.printf("[ID3] Title: %s\n", currentMP3Title.c_str());
        }
      }
      else if (id3Data.startsWith("Album: ") || id3Data.startsWith("TALB=")) {
        int startPos = id3Data.indexOf(": ");
        if (startPos < 0) startPos = id3Data.indexOf("=");
        if (startPos >= 0) {
          currentMP3Album = id3Data.substring(startPos + 1);
          currentMP3Album.trim();
          Serial.printf("[ID3] Album: %s\n", currentMP3Album.c_str());
        }
      }
    }
    break;
    case Audio::evt_lasthost:       Serial.printf("last URL: ... %s\n", m.msg); break;
    case Audio::evt_name:           
	  {
      stationNameStream = String(m.msg);
      stationNameStream.trim();
      
      // Jesli gramy z ze strony WEB URL lub flaga stationNameFromStream=1 to odczytujemy nazwe stacji ze streamu nie z pliku banku
      //if ((bank_nr == 0) || (stationNameFromStream)) stationName = stationNameStream; 
      if ((urlPlaying) || (stationNameFromStream)) stationName = stationNameStream; 

      f_audioInfoRefreshStationString = true;
      wsAudioRefresh = true;
      Serial.printf("info: ....... station name: %s\n", m.msg); // station name lub icy-name
	  }
    break; 

    case Audio::evt_streamtitle: // Zapisz tytuł utworu
    {
      stationString = String(m.msg);
		  stationString.trim();
		
      //ActionNeedUpdateTime = true;
      f_audioInfoRefreshStationString = true;	
      wsAudioRefresh = true;
      
      // Debug
      Serial.printf("info: ....... stream title: %s\n", m.msg);
      //Serial.printf("info: ....... %s\n", m.msg);
    }
    break;
    
    case Audio::evt_icylogo:        
    {
      stationLogoUrl = String(m.msg);
      stationLogoUrl.trim();
      Serial.println("info: ....... stationLogoUrl: " + stationLogoUrl);
      Serial.printf("icy logo: ... %s\n", m.msg); 
    }
    break;
    
    case Audio::evt_icydescription: Serial.printf("icy descr: .. %s\n", m.msg); break;
    case Audio::evt_image: for(int i = 0; i < m.vec.size(); i += 2) { Serial.printf("cover image:  segment %02i, pos %07lu, len %05lu\n", i / 2, m.vec[i], m.vec[i + 1]);} break; // APIC
    case Audio::evt_lyrics:         Serial.printf("sync lyrics:  %s\n", m.msg); break;
    default:                        Serial.printf("message:..... %s\n", m.msg); break;
  }
}


void encoderFunctionOrderChange()
{
  displayActive = true;
  displayStartTime = millis();
  volumeSet = false;
  timeDisplay = false;
  bankMenuEnable = false;
  encoderFunctionOrder = !encoderFunctionOrder;
  u8g2.clearBuffer();
  u8g2.setFont(spleen6x12PL);
  u8g2.drawStr(1,14,"Encoder function order change:");
  if (encoderFunctionOrder == false) {u8g2.drawStr(1,28,"Rotate for volume, press for station list");}
  if (encoderFunctionOrder == true ) {u8g2.drawStr(1,28,"Rotate for station list, press for volume");}
  u8g2.sendBuffer();
}

void bankMenuDisplay()
{
  if (bank_nr < 1) {bank_nr = 1;}
  const uint8_t cornerRadius = 3;
  float segmentWidth = (float)212 / bank_nr_max;

  displayStartTime = millis();
  Serial.println("debug -> BankMenuDisplay");
  volumeSet = false;
  bankMenuEnable = true;
  timeDisplay = false;
  displayActive = true;
  //currentOption = BANK_LIST;  // Ustawienie listy banków do przewijania i wyboru
  String bankNrStr = String(bank_nr);
  Serial.println("debug -> Wyświetlenie listy banków");
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tf);
  u8g2.drawStr(80, 33, "BANK ");
  u8g2.drawStr(145, 33, String(bank_nr).c_str());  // numer banku
  //if ((bankNetworkUpdate == true) || (noSDcard == true))
  if ((bankNetworkUpdate == true) || (storageTextName == "EEPROM"))
  {
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(185, 24, "NETWORK ");
    u8g2.drawStr(188, 34, "UPDATE  ");
    
    //if (noSDcard == true)
    if (storageTextName == "EEPROM")
    {
      //u8g2.drawStr(24, 24, "NO SD");
      u8g2.drawStr(24, 34, "NO CARD");
    }
  
  }
  
  u8g2.drawRFrame(21, 42, 214, 14, cornerRadius);                // Ramka do slidera bankow
    
  uint16_t sliderX = 23 + (uint16_t)round((bank_nr - 1)* segmentWidth); // Przeliczenie zmiennej X z zaokragleniemw  gore dla położenia slidera
  uint16_t sliderWidth = (uint16_t)round(segmentWidth - 2); // Przliczenie szwerokości slidera w zaleznosci od ilosi banków
  u8g2.drawRBox(sliderX, 44, sliderWidth, 10, 2);

  //u8g2.drawRBox((bank_nr * 13) + 10, 44, 15, 10, 2);  // wypełnienie slidera Rbox dla stałej wartosci max_bank = 16, stara wersja
  u8g2.sendBuffer();
  
}

// =========== Funkcja do obsługi przycisków enkoderów, debouncing i długiego naciśnięcia ==============//
void handleButtons() 
{
  // Blokuj obsługę przycisków radia gdy SD Player jest aktywny
  if (sdPlayerOLEDActive) return;
  
  static unsigned long buttonPressTime2 = 0;  // Zmienna do przechowywania czasu naciśnięcia przycisku enkodera 2
  static bool isButton2Pressed = false;       // Flaga do śledzenia, czy przycisk enkodera 2 jest wciśnięty
  static bool action2Taken = false;           // Flaga do śledzenia, czy akcja dla enkodera 2 została wykonana

  static unsigned long lastPressTime2 = 0;  // Zmienna do kontrolowania debouncingu (ostatni czas naciśnięcia)
  const unsigned long debounceDelay2 = 50;  // Opóźnienie debouncingu

  // ===== Obsługa przycisku enkodera 2 =====
  int reading2 = digitalRead(SW_PIN2);

  // Debouncing dla przycisku enkodera 2
  if (reading2 == LOW)  // Przycisk jest wciśnięty (stan niski)
  {
    if (millis() - lastPressTime2 > debounceDelay2) 
    {
      lastPressTime2 = millis();  // Aktualizujemy czas ostatniego naciśnięcia
      // Sprawdzamy, czy przycisk był wciśnięty przez 3 sekundy
      if (!isButton2Pressed) 
      {
        buttonPressTime2 = millis();  // Ustawiamy czas naciśnięcia
        isButton2Pressed = true;      // Ustawiamy flagę, że przycisk jest wciśnięty
        action2Taken = false;         // Resetujemy flagę akcji dla enkodera 2
        action3Taken = false;         // Resetujmy flage akcji Super długiego wcisniecia enkodera 2
        volumeSet = false;
      }

      if (millis() - buttonPressTime2 >= buttonLongPressTime2 && millis() - buttonPressTime2 >= buttonSuperLongPressTime2 && action3Taken == false) 
      {
        ir_code = rcCmdPower; // Udajemy kod pilota Back
        bit_count = 32;
        calcNec();          // Przeliczamy kod pilota na pełny oryginalny kod NEC
        action3Taken = true;
      }


      // Jeśli przycisk jest wciśnięty przez co najmniej 3 sekundy i akcja jeszcze nie była wykonana
      if (millis() - buttonPressTime2 >= buttonLongPressTime2 && !action2Taken && millis() - buttonPressTime2 < buttonSuperLongPressTime2) {
        Serial.println("debug--Bank Menu");
        bankMenuDisplay();
        
        // Ustawiamy flagę akcji, aby wykonała się tylko raz
        action2Taken = true;
      }
    }
  } 
  else 
  {
    isButton2Pressed = false;  // Resetujemy flagę naciśnięcia przycisku enkodera 2
    action2Taken = false;      // Resetujemy flagę akcji dla enkodera 2
    action3Taken = false;
  }

    
  #ifdef twoEncoders

    static unsigned long buttonPressTime1 = 0;  // Zmienna do przechowywania czasu naciśnięcia przycisku enkodera 2
    static bool isButton1Pressed = false;       // Flaga do śledzenia, czy przycisk enkodera 2 jest wciśnięty
    
    static unsigned long lastPressTime1 = 0;  // Zmienna do kontrolowania debouncingu (ostatni czas naciśnięcia)
    const unsigned long debounceDelay1 = 50;  // Opóźnienie debouncingu

    // ===== Obsługa przycisku enkodera 1 =====
    int reading1 = digitalRead(SW_PIN1);
    // Debouncing dla przycisku enkodera 1

    if (reading1 == LOW)  // Przycisk jest wciśnięty (stan niski)
    {
      if (millis() - lastPressTime1 > debounceDelay1) 
      {
        lastPressTime1 = millis();  // Aktualizujemy czas ostatniego naciśnięcia

        // Sprawdzamy, czy przycisk był wciśnięty przez 3 sekundy
        if (!isButton1Pressed) 
        {
          buttonPressTime1 = millis();  // Ustawiamy czas naciśnięcia
          isButton1Pressed = true;      // Ustawiamy flagę, że przycisk jest wciśnięty
          action4Taken = false;         // Resetujemy flagę akcji dla enkodera 2
        }
        
        
        if (millis() - buttonPressTime1 >= buttonLongPressTime1 && millis() - buttonPressTime1 >= buttonSuperLongPressTime1 && action4Taken == false) 
        {
          ir_code = rcCmdPower; // Udajemy kod pilota Back
          bit_count = 32;
          calcNec();          // Przeliczamy kod pilota na pełny oryginalny kod NEC      
          action4Taken = true;
        }
        /*
        if (millis() - buttonPressTime1 >= buttonLongPressTime1 && !action5Taken && millis() - buttonPressTime1 < buttonSuperLongPressTime1) 
        {
          bankMenuDisplay();
          // Ustawiamy flagę akcji, aby wykonała się tylko raz
          action5Taken = true;
        }
        */
      }
    }  
    else
    {
      isButton1Pressed = false;  // Resetujemy flagę naciśnięcia przycisku enkodera 1
      action4Taken = false;
      action5Taken = false;
    }     
  #endif
  



}

int maxSelection() 
{
  //if (currentOption == INTERNET_RADIO) 
  //{
    return stationsCount - 1;
  //} 
  //return 0;  // Zwraca 0, jeśli żaden warunek nie jest spełniony
}

// Funkcja do przewijania w dół
void scrollDown() 
{
  if (currentSelection < maxSelection()) 
  {
    currentSelection++;
    if (currentSelection >= firstVisibleLine + maxVisibleLines) 
    {
      firstVisibleLine++;
    }
  }
    else
  {
    // Jeśli osiągnięto maksymalną wartość, przejdź do najmniejszej (0)
    currentSelection = 0;
    firstVisibleLine = 0; // Przywróć do pierwszej widocznej linii
  }
    
  Serial.print("Scroll Down: CurrentSelection = ");
  Serial.println(currentSelection);
}

// Funkcja do przewijania w górę
void scrollUp() 
{
  if (currentSelection > 0) 
  {
    currentSelection--;
    if (currentSelection < firstVisibleLine) 
    {
      firstVisibleLine = currentSelection;
    }
  }
  else
  {
    // Jeśli osiągnięto wartość 0, przejdź do najwyższej wartości
    currentSelection = maxSelection(); 
    firstVisibleLine = currentSelection - maxVisibleLines + 1; // Ustaw pierwszą widoczną linię na najwyższą
  }
  
  Serial.print("Scroll Up: CurrentSelection = ");
  Serial.println(currentSelection);
}

void readVolumeFromSD() 
{
  // Sprawdź, czy karta SD jest dostępna
  //if (!STORAGE.begin(SD_CS)) 
  if (!STORAGE_BEGIN())
  {
    Serial.println("debug SD -> Nie można znaleźć karty SD, ustawiam wartość Volume z EEPROMu.");
    Serial.print("debug vol -> Wartość Volume: ");
    EEPROM.get(2, volumeValue);
    if (volumeValue > maxVolume) {volumeValue = 10;} // zabezpiczenie przed pusta komorka EEPROM o wartosci FF (255)
    
    if (f_volumeFadeOn == false) {audio.setVolume(volumeValue);}  // zakres 0...21...42
    volumeBufferValue = volumeValue; 
    
    Serial.println(volumeValue);
    return;
  }
  // Sprawdź, czy plik volume.txt istnieje
  if (STORAGE.exists("/volume.txt")) 
  {
    myFile = STORAGE.open("/volume.txt");
    if (myFile) 
    {
      volumeValue = myFile.parseInt();
	    myFile.close();
      
      Serial.print("debug SD -> Wczytano volume.txt z karty SD, wartość odczytana: ");
      Serial.println(volumeValue);    
    } 
	  else 
	  {
      Serial.println("debug SD -> Błąd podczas otwierania pliku volume.txt");
    }
  } 
  else 
  {
    Serial.print("debug SD -> Plik volume.txt nie istnieje, wartość Volume domyślna: ");
	  Serial.println(volumeValue);    
  }
  if (volumeValue > maxVolume) {volumeValue = 10;}  // Ustawiamy bezpieczną wartość
  //audio.setVolume(volumeValue);  // zakres 0...21...42
  volumeBufferValue = volumeValue;
}

void saveVolumeOnSD() 
{
  volumeBufferValue = volumeValue; // Wyrownanie wartosci bufora Volume i Volume przy zapisie
  
  // Sprawdź, czy plik volume.txt istnieje
  Serial.print("debug SD -> Zaspis do pliku wartosci Volume: ");
  Serial.println(volumeValue); 
  
  // Sprawdź, czy plik istnieje
  if (STORAGE.exists("/volume.txt")) 
  {
    Serial.println("debug SD -> Plik volume.txt już istnieje.");

    // Otwórz plik do zapisu i nadpisz aktualną wartość flitrów equalizera
    myFile = STORAGE.open("/volume.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println(volumeValue);
      myFile.close();
      Serial.println("debug SD -> Aktualizacja volume.txt na karcie SD");
    } 
	  else 
	  {
      Serial.println("debug SD -> Błąd podczas otwierania pliku volume.txt.");
    }
  } 
  else 
  {
    Serial.println("debug SD -> Plik volume.txt nie istnieje. Tworzenie...");

    // Utwórz plik i zapisz w nim aktualną wartość głośności
    myFile = STORAGE.open("/volume.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println(volumeValue);
      myFile.close();
      Serial.println("debug SD -> Utworzono i zapisano volume.txt na karcie SD");
    } 
    else 
    {
      Serial.println("debug SD -> Błąd podczas tworzenia pliku volume.txt.");
    }
  }
  if (noSDcard == true) {EEPROM.write(2,volumeValue); EEPROM.commit(); Serial.println("debug eeprom -> Zapis volume do EEPROM");}
}

void drawSignalPower(uint8_t xpwr, uint8_t ypwr, bool print, bool mode)
{
  // Wartosci na podstawie ->  https://www.intuitibits.com/2016/03/23/dbm-to-percent-conversion/
  int signal_dBM[] = { -100, -99, -98, -97, -96, -95, -94, -93, -92, -91, -90, -89, -88, -87, -86, -85, -84, -83, -82, -81, -80, -79, -78, -77, -76, -75, -74, -73, -72, -71, -70, -69, -68, -67, -66, -65, -64, -63, -62, -61, -60, -59, -58, -57, -56, -55, -54, -53, -52, -51, -50, -49, -48, -47, -46, -45, -44, -43, -42, -41, -40, -39, -38, -37, -36, -35, -34, -33, -32, -31, -30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1};
  int signal_percent[] = {0, 0, 0, 0, 0, 0, 4, 6, 8, 11, 13, 15, 17, 19, 21, 23, 26, 28, 30, 32, 34, 35, 37, 39, 41, 43, 45, 46, 48, 50, 52, 53, 55, 56, 58, 59, 61, 62, 64, 65, 67, 68, 69, 71, 72, 73, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 90, 91, 92, 93, 93, 94, 95, 95, 96, 96, 97, 97, 98, 98, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};

  int signalpwr = WiFi.RSSI();
  //uint8_t wifiSignalLevel = 0;  // używa globalnej wifiSignalLevel
    
  
  // Pionowe kreseczki, szerokosc 11px
  
  // Czyscimy obszar pod kreseczkami
  u8g2.setDrawColor(0);
  u8g2.drawBox(xpwr,ypwr-10,11,11);
  u8g2.setDrawColor(1);

  
  // Rysowanie antenki
  int x = xpwr - 3; 
  int y = ypwr - 8;
  u8g2.drawLine(x,y,x,y+7); // kreska srodkowa
  u8g2.drawLine(x,y+4,x-3,y); // lewe ramie
  u8g2.drawLine(x,y+4,x+3,y); // prawe ramie


  // Rysujemy podstawy 1x1px pod kazdą kreseczką
  u8g2.drawBox(xpwr,ypwr - 1, 1, 1);
  u8g2.drawBox(xpwr + 2, ypwr - 1, 1, 1);
  u8g2.drawBox(xpwr + 4, ypwr - 1, 1, 1);
  u8g2.drawBox(xpwr + 6, ypwr - 1, 1, 1);
  u8g2.drawBox(xpwr + 8, ypwr - 1, 1, 1);
  u8g2.drawBox(xpwr + 10, ypwr - 1, 1, 1);

 if ((WiFi.status() == WL_CONNECTED) && (mode == 0))
 {
      // Rysujemy kreseczki
    if (signalpwr > -88) { wifiSignalLevel = 1; u8g2.drawBox(xpwr, ypwr - 2, 1, 1); }     // 0-14
    if (signalpwr > -81) { wifiSignalLevel = 2; u8g2.drawBox(xpwr + 2, ypwr - 3, 1, 2); } // > 28
    if (signalpwr > -74) { wifiSignalLevel = 3; u8g2.drawBox(xpwr + 4, ypwr - 4, 1, 3); } // > 42
    if (signalpwr > -66) { wifiSignalLevel = 4; u8g2.drawBox(xpwr + 6, ypwr - 5, 1, 4); } // > 56
    if (signalpwr > -57) { wifiSignalLevel = 5; u8g2.drawBox(xpwr + 8, ypwr - 6, 1, 5); } // > 70
    if (signalpwr > -50) { wifiSignalLevel = 6; u8g2.drawBox(xpwr + 10, ypwr - 8, 1, 7);} // > 84
  }

  if ((WiFi.status() == WL_CONNECTED) && (mode == 1))
  {
      // Rysujemy kreseczki
    if (signalpwr > -88) { wifiSignalLevel = 1; u8g2.drawBox(xpwr, ypwr - 2, 1, 1); }     // 0-14
    if (signalpwr > -81) { wifiSignalLevel = 2; u8g2.drawBox(xpwr + 2, ypwr - 3, 1, 2); } // > 28
    if (signalpwr > -74) { wifiSignalLevel = 3; u8g2.drawBox(xpwr + 4, ypwr - 4, 1, 3); } // > 42
    if (signalpwr > -66) { wifiSignalLevel = 4; u8g2.drawBox(xpwr + 6, ypwr - 5, 1, 4); } // > 56
    if (signalpwr > -57) { wifiSignalLevel = 5; u8g2.drawBox(xpwr + 8, ypwr - 6, 1, 5); } // > 70
    if (signalpwr > -50) { wifiSignalLevel = 6; u8g2.drawBox(xpwr + 10, ypwr - 7, 1, 6);} // > 84
  }


  if (print == true) // Jesli flaga print =1 to wypisujemy na serialu sile sygnału w % i w skali 1-6
  {
    for (int j = 0; j < 100; j++) 
    {
      if (signal_dBM[j] == signalpwr) 
      {
        Serial.print("debug WiFi -> Sygnału WiFi: ");
        Serial.print(signal_percent[j]);
        Serial.print("%  Poziom: ");
        Serial.print(wifiSignalLevel);
        Serial.print("   dBm: ");
        Serial.println(signalpwr);
        
        break;
      }
    }
  }
}


void rcInputKey(uint8_t i)
{
  // PRIORYTET: SDPlayer track selection (gdy OLED aktywny)
  if (sdPlayerOLEDActive) {
    Serial.printf("[IR] Numeric key %d pressed - SDPlayer active, tracksCount=%d\n", i, tracksCount);
    
    // ===== SYSTEM WIELOCYFROWEGO WPROWADZANIA (0-100) =====
    // Obsługa cyfry 0 jako dodanie do bufora (np. "1"+"0"=10)
    
    if (i == 0) {
      // Cyfra 0 - dodaj do bufora jeśli jest aktywne wprowadzanie
      if (irInputActive && irInputBuffer > 0 && irInputBuffer < 10) {
        // Dodaj 0 jako drugą cyfrę (1→10, 2→20, etc.)
        int newNumber = irInputBuffer * 10;
        if (newNumber <= 100) {
          irInputBuffer = newNumber;
          irInputTimeout = millis() + IR_INPUT_TIMEOUT_MS;
          Serial.printf("[SDPlayer IR] Added 0, buffer now: %d\n", irInputBuffer);
        }
      }
      // Uwaga: Pojedyncze "0" i "00" dla wyjścia obsługiwane są w głównym bloku IR
      return;
    }
    
    if (i >= 1 && i <= 9) {
      // Klawisz 1-9: buduj liczbę wielocyfrową
      if (!irInputActive) {
        // Rozpocznij nowe wprowadzanie
        irInputBuffer = i;
        irInputActive = true;
        irInputTimeout = millis() + IR_INPUT_TIMEOUT_MS;
        Serial.printf("[SDPlayer IR] Multi-digit input started: %d\n", irInputBuffer);
        
        // Jeśli liczba jednocyfrowa i w zakresie - automatyczne potwierdzenie po krótkim czasie
        if (i <= tracksCount) {
          // Daj 1 sekundę na ewentualne dodanie kolejnej cyfry
          irInputTimeout = millis() + 1000;
        }
      } else if (irInputBuffer < 10) {
        // Dodaj drugą cyfrę (max 99)
        int newNumber = irInputBuffer * 10 + i;
        if (newNumber <= 100) {
          irInputBuffer = newNumber;
          irInputTimeout = millis() + IR_INPUT_TIMEOUT_MS;
          Serial.printf("[SDPlayer IR] Multi-digit input updated: %d\n", irInputBuffer);
        } else {
          Serial.printf("[SDPlayer IR] Number too large: %d, ignoring digit %d\n", newNumber, i);
        }
      } else {
        Serial.printf("[SDPlayer IR] Buffer full (%d), ignoring digit %d\n", irInputBuffer, i);
      }
      return;
    }
    
    return; // Wyjście - nie przetwarzaj jako zmiana stacji radiowej
  }

  // Original radio station selection logic
  rcInputDigitsMenuEnable = true;
  if (bankMenuEnable == true)
  {
    
    if (i == 0) {i = 10;}
    bank_nr = i;
    bankMenuDisplay();
  }
  else
  {
    timeDisplay = false;
    displayActive = true;
    displayStartTime = millis(); 
    
    if (rcInputDigit1 == 0xFF)
    {
      rcInputDigit1 = i;
    }
    else 
    {
      rcInputDigit2 = i;
    }

    int y = 35;
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_fub14_tf); // cziocnka 14x11
    u8g2.drawStr(65, y, "Station:"); 
    if (rcInputDigit1 != 0xFF)
    {u8g2.drawStr(153, y, String(rcInputDigit1).c_str());} 
    //else {u8g2.drawStr(153, x,"_" ); }
    else {u8g2.drawStr(153, y,"_" ); }
    
    if (rcInputDigit2 != 0xFF)
    {u8g2.drawStr(164, y, String(rcInputDigit2).c_str());} 
    else {u8g2.drawStr(164, y,"_" ); }



    if ((rcInputDigit1 != 0xFF) && (rcInputDigit2 != 0xFF)) // jezeli obie wartosci nie są puste
    {
      station_nr = (rcInputDigit1 *10) + rcInputDigit2;
    }
    else if ((rcInputDigit1 != 0xFF) && (rcInputDigit2 == 0xFF))  // jezeli tylko podalismy jedna cyfrę
    { 
      station_nr = rcInputDigit1;
    }

    if (station_nr > stationsCount)  // sprawdzamy czy wprowadzona wartość nie wykracza poza licze stacji w danym banku
    {
      station_nr = stationsCount;  // jesli wpisana wartość jest wieksza niz ilosc stacji to ustawiamy war
    }
    
    if (station_nr < 1)
    {
      station_nr = stationFromBuffer;
    }

    // Odczyt stacji pod daną komórka pamieci PSRAM:
    char station[STATION_NAME_LENGTH + 1];  // Tablica na nazwę stacji o maksymalnej długości zdefiniowanej przez STATION_NAME_LENGTH
    memset(station, 0, sizeof(station));    // Wyczyszczenie tablicy zerami przed zapisaniem danych
    int length = psramData[(station_nr - 1) * (STATION_NAME_LENGTH + 1)];   // Odczytaj długość nazwy stacji z PSRAM dla bieżącego indeksu stacji
    
    for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) { // Odczytaj nazwę stacji z PSRAM jako ciąg bajtów, maksymalnie do STATION_NAME_LENGTH
      station[j] = psramData[(station_nr - 1) * (STATION_NAME_LENGTH + 1) + 1 + j];  // Odczytaj znak po znaku nazwę stacji
    }
    u8g2.setFont(spleen6x12PL);
    String stationNameText = String(station);
    stationNameText = stationNameText.substring(0, 25); // Przycinamy do 23 znakow

    u8g2.drawLine(0,48,256,48);
    u8g2.setFont(spleen6x12PL);
    u8g2.setCursor(0, 60);
    u8g2.print("Bank:" + String(bank_nr) + ", 1-" + String(stationsCount) + "     " + stationNameText);
    u8g2.sendBuffer();

    if ((rcInputDigit1 !=0xFF) && (rcInputDigit2 !=0xFF)) // jezeli wpisalismy obie cyfry to czyscimy pola aby mozna bylo je wpisac ponownie
    {
      rcInputDigit1 = 0xFF; // czyscimy cyfre 1, flaga pustej zmiennej F aby naciskajac kolejny raz mozna bylo wypisac cyfre bez czekania 6 sek
      rcInputDigit2 = 0xFF; // czyscimy cyfre 2, flaga pustej zmiennej F
    }
  }  
}

// Funkcja do zapisywania numeru stacji i numeru banku na karcie SD
void saveStationOnSD() 
{
  if ((station_nr !=0) && (bank_nr != 0))
  {
    // Sprawdź, czy plik station_nr.txt istnieje

    Serial.print("debug SD -> Zapisujemy bank: ");
    Serial.println(bank_nr);
    Serial.print("debug SD -> Zapisujemy stacje: ");
    Serial.println(station_nr);

    // Sprawdź, czy plik station_nr.txt istnieje
    if (STORAGE.exists("/station_nr.txt")) {
      Serial.println("debug SD -> Plik station_nr.txt już istnieje.");

      // Otwórz plik do zapisu i nadpisz aktualną wartość station_nr
      myFile = STORAGE.open("/station_nr.txt", FILE_WRITE);
      if (myFile) {
        myFile.println(station_nr);
        myFile.close();
        Serial.println("debug SD -> Aktualizacja station_nr.txt na karcie SD");
      } else {
        Serial.println("debug SD -> Błąd podczas otwierania pliku station_nr.txt.");
      }
    } else {
      Serial.println("debug SD -> Plik station_nr.txt nie istnieje. Tworzenie...");

      // Utwórz plik i zapisz w nim aktualną wartość station_nr
      myFile = STORAGE.open("/station_nr.txt", FILE_WRITE);
      if (myFile) {
        myFile.println(station_nr);
        myFile.close();
        Serial.println("debug SD -> Utworzono i zapisano station_nr.txt na karcie SD");
      } else {
        Serial.println("debug SD -> Błąd podczas tworzenia pliku station_nr.txt.");
      }
    }

    // Sprawdź, czy plik bank_nr.txt istnieje
    if (STORAGE.exists("/bank_nr.txt")) 
    {
      Serial.println("debug SD -> Plik bank_nr.txt już istnieje.");

      // Otwórz plik do zapisu i nadpisz aktualną wartość bank_nr
      myFile = STORAGE.open("/bank_nr.txt", FILE_WRITE);
      if (myFile) {
        myFile.println(bank_nr);
        myFile.close();
        Serial.println("debug SD -> Aktualizacja bank_nr.txt na karcie STORAGE.");
      } else {
        Serial.println("debug SD -> Błąd podczas otwierania pliku bank_nr.txt.");
      }
    } 
    else 
    {
      Serial.println("debug SD -> Plik bank_nr.txt nie istnieje. Tworzenie...");

      // Utwórz plik i zapisz w nim aktualną wartość bank_nr
      myFile = STORAGE.open("/bank_nr.txt", FILE_WRITE);
      if (myFile) {
        myFile.println(bank_nr);
        myFile.close();
        Serial.println("debug SD -> Utworzono i zapisano bank_nr.txt na karcie SD");
      } else {
        Serial.println("debug SD -> Błąd podczas tworzenia pliku bank_nr.txt.");
      }
    }
    
    if (noSDcard == true)
    {
      Serial.println("debug SD -> Brak karty SD zapisujemy do EEPROM");
      EEPROM.write(0, station_nr);
      EEPROM.write(1, bank_nr);
      EEPROM.commit();
    }
  }
  else
  {
    Serial.println("debug ChangeStation -> Gramy z adresu URL, nie zapisujemy stacji i banku");  
  }
}


void startFadeIn(uint8_t target)
{
  targetVolume = target;
  currentVolume = 0;
  audio.setVolume(0);
  fadingIn = true;
  lastFadeTime = millis();
}

void handleFadeIn()
{
  if (!fadingIn) return;  // jeśli fade-in nieaktywny, nic nie rób

  if (millis() - lastFadeTime >= fadeStepDelay)
  {
    lastFadeTime = millis();

    if (currentVolume < targetVolume)
    {
      audio.setVolume(currentVolume);
      currentVolume++;
    }
    else
    {
      fadingIn = false; // zakończ fade-in
      audio.setVolume(targetVolume);
    }
  }
}

void volumeFadeOut(uint8_t time_ms)
{
  for (uint8_t i = volumeValue; i > 0; i--)
  {
    audio.setVolume(i);
    delay(time_ms);
  } 
}


void changeStation() 
{
  fwupd = false;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tf); // cziocnka 14x11
  u8g2.drawStr(34, 33, "Loading stream..."); // 8 znakow  x 11 szer
  //u8g2.drawStr(51, 33, "Loading stream"); // 8 znakow  x 11 szer
  u8g2.sendBuffer();

  mp3 = flac = aac = vorbis = opus = false;
  streamCodec = "-";
  //stationLogoUrl = "";
  
  if (urlPlaying) {bank_nr = previous_bank_nr;}  // Przywracamy ostatni numer banku po graniu z ULR gdzie ustawilismy bank na 0
    
  // Usunięcie wszystkich znaków z obiektów 
  stationString.remove(0);  
  stationNameStream.remove(0);
  stationStringWeb.remove(0);
  stationLogoUrl.remove(0);

  bitrateString.remove(0);
  sampleRateString.remove(0);
  bitsPerSampleString.remove(0); 
  SampleRate = 0;
  SampleRateRest = 0;

  sampleRateString = "--.-";
  bitsPerSampleString = "--";
  bitrateString = "---";

  Serial.println("debug changeStation -> Read station from PSRAM");
  String stationUrl = "";

  // Odczyt stacji pod daną komórka pamieci PSRAM:
  char station[STATION_NAME_LENGTH + 1];  // Tablica na nazwę stacji o maksymalnej długości zdefiniowanej przez STATION_NAME_LENGTH
  memset(station, 0, sizeof(station));    // Wyczyszczenie tablicy zerami przed zapisaniem danych
  int length = psramData[(station_nr - 1) * (STATION_NAME_LENGTH + 1)];   // Odczytaj długość nazwy stacji z PSRAM dla bieżącego indeksu stacji
      
  for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
  { // Odczytaj nazwę stacji z PSRAM jako ciąg bajtów, maksymalnie do STATION_NAME_LENGTH
    station[j] = psramData[(station_nr - 1) * (STATION_NAME_LENGTH + 1) + 1 + j];  // Odczytaj znak po znaku nazwę stacji
  }
  
  //Serial.println("-------- GRAMY OBECNIE ---------- ");
  //Serial.print(station_nr - 1);
  //Serial.print(" ");
  //Serial.println(String(station));

  String line = String(station); // Przypisujemy dane odczytane z PSRAM do zmiennej line
  
  // Wyciągnij pierwsze 42 znaki i przypisz do stationName
  stationName = line.substring(0, 41);  //42 Skopiuj pierwsze 42 znaki z linii
  Serial.print("debug changeStation -> Nazwa stacji: ");
  Serial.println(stationName);

  // Znajdź część URL w linii, np. po numerze stacji
  int urlStart = line.indexOf("http");  // Szukamy miejsca, gdzie zaczyna się URL
  if (urlStart != -1) 
  {
    stationUrl = line.substring(urlStart);  // Wyciągamy URL od "http"
    stationUrl.trim();                      // Usuwamy białe znaki na początku i końcu
  }
  else
  {
	return;
  }
  
  if (stationUrl.isEmpty()) // jezeli link URL jest pusty
  {
    Serial.println("debug changeStation -> Błąd: Nie znaleziono stacji dla podanego numeru.");
    return;
  }
  
  //Serial.print("URL: ");
  //Serial.println(stationUrl);

  // Weryfikacja, czy w linku znajduje się "http" lub "https"
  if (stationUrl.startsWith("http://") || stationUrl.startsWith("https://")) 
  {
    // Wydrukuj nazwę stacji i link na serialu
    Serial.print("debug changeStation -> Aktualnie wybrana stacja: ");
    Serial.println(station_nr);
    Serial.print("debug changeStation -> Link do stacji: ");
    Serial.println(stationUrl);
    
    u8g2.setFont(spleen6x12PL);  // wypisujemy jaki stream jakie stacji jest ładowany
    
    int StationNameEnd = stationName.indexOf("  "); // Wycinamy nazwe stacji tylko do miejsca podwojnej spacji 
    stationName = stationName.substring(0, StationNameEnd);
    
    int stationNameWidth = u8g2.getStrWidth(stationName.substring(0, stationNameLenghtCut).c_str()); // Liczy pozycje aby wyswietlic stationName na środku
    int stationNamePositionX = (SCREEN_WIDTH - stationNameWidth) / 2;
    
    u8g2.drawStr(stationNamePositionX, 55, String(stationName.substring(0, stationNameLenghtCut)).c_str());
    u8g2.sendBuffer();
    
    // Płynne wyciszenie przed zmiana stacji jesli włączone
    if (f_volumeFadeOn && !volumeMute) {volumeFadeOut(volumeFadeOutTime);}
    
    // Połącz z daną stacją
    audio.connecttohost(stationUrl.c_str());
    
    // Właczamy sciszanie tylko jesli MUTE jest wyłaczone. Jesli MUTE jest wyłączone i sciszanie równiez to ustawiamy głośnośc zgodnie z volumeValue 
    if (f_volumeFadeOn && !volumeMute) {startFadeIn(volumeValue);} else if (!volumeMute) {audio.setVolume(volumeValue);}   
    
    // Zapisujemy jaki numer stacji i który bank gramy tylko jesli sie zmieniły
    //if ((station_nr != stationFromBuffer || bank_nr != previous_bank_nr) && f_saveVolumeStationAlways) {saveStationOnSD();}
    
    
    if (station_nr != 0 ) {stationFromBuffer = station_nr;} 
    if (f_saveVolumeStationAlways) {saveStationOnSD();} 
    //saveStationOnSD(); // Zapisujemy jaki numer stacji i który bank gramy

    urlPlaying = false; // Kasujemy flage odtwarzania z adresu przesłanego ze strony WWW
        
  } 
  else 
  {
    Serial.println("debug changeStation -> Błąd: link stacji nie zawiera 'http' lub 'https'");
    Serial.println("debug changeStation -> Odczytany URL: " + stationUrl);
  }
  currentSelection = station_nr - 1; // ustawiamy stacje na liscie na obecnie odtwarzaczną po zmianie stacji
  firstVisibleLine = currentSelection + 1; // pierwsza widoczna lina to grająca stacja przy starcie
  if (currentSelection + 1 >= stationsCount - 1) 
  {
   firstVisibleLine = currentSelection - 3;
  }
  ActionNeedUpdateTime = true;
   
  wsStationChange(station_nr, bank_nr); // Od teraz Event Audio odpowiada za info o stacji
  wsAudioRefresh = true;
}

// Funkcja do wyświetlania listy stacji radiowych z opcją wyboru poprzez zaznaczanie w negatywie
void displayStations() 
{
  listedStations = true;
  u8g2.clearBuffer();  // Wyczyść bufor przed rysowaniem, aby przygotować ekran do nowej zawartości
  u8g2.setFont(spleen6x12PL);
  u8g2.setCursor(20, 10);                                          // Ustaw pozycję kursora (x=60, y=10) dla nagłówka
  u8g2.print("BANK: " + String(bank_nr));                                          // Wyświetl nagłówek "BANK:"
  u8g2.setCursor(68, 10);                                          // Ustaw pozycję kursora (x=60, y=10) dla nagłówka
  u8g2.print(" - RADIO STATIONS: ");                                // Wyświetl nagłówek "Radio Stations:"
  u8g2.print(String(station_nr) + " / " + String(stationsCount));  // Dodaj numer aktualnej stacji i licznik wszystkich stacji
  u8g2.drawLine(0,11,256,11);
  // "BANK: 16 - RADIO STATIONS: 99 / 99


  int displayRow = 1;  // Zmienna dla numeru wiersza, zaczynając od drugiego (pierwszy to nagłówek)
  
  //erial.print("FirstVisibleLine:");
  //Serial.print(firstVisibleLine);

  // Wyświetlanie stacji, zaczynając od drugiej linii (y=21)
  for (int i = firstVisibleLine; i < min(firstVisibleLine + maxVisibleLines, stationsCount); i++) 
  {
    char station[STATION_NAME_LENGTH + 1];  // Tablica na nazwę stacji o maksymalnej długości zdefiniowanej przez STATION_NAME_LENGTH
    memset(station, 0, sizeof(station));    // Wyczyszczenie tablicy zerami przed zapisaniem danych

    // Odczytaj długość nazwy stacji z PSRAM dla bieżącego indeksu stacji
    int length = psramData[i * (STATION_NAME_LENGTH + 1)];  //----------------------------------------------

    // Odczytaj nazwę stacji z PSRAM jako ciąg bajtów, maksymalnie do STATION_NAME_LENGTH
    for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
    {
      station[j] = psramData[i * (STATION_NAME_LENGTH + 1) + 1 + j];  // Odczytaj znak po znaku nazwę stacji
    }

    // Sprawdź, czy bieżąca stacja to ta, która jest aktualnie zaznaczona
    if (i == currentSelection) 
    {
      u8g2.setDrawColor(1);                           // Ustaw biały kolor rysowania
      //u8g2.drawBox(0, displayRow * 13 - 2, 256, 13);  // Narysuj prostokąt jako tło dla zaznaczonej stacji (x=0, szerokość 256, wysokość 10)
      u8g2.drawBox(0, displayRow * 13, 256, 13);  // Narysuj prostokąt jako tło dla zaznaczonej stacji (x=0, szerokość 256, wysokość 10)
      u8g2.setDrawColor(0);                           // Zmień kolor rysowania na czarny dla tekstu zaznaczonej stacji
    } else {
      u8g2.setDrawColor(1);  // Dla niezaznaczonych stacji ustaw zwykły biały kolor tekstu
    }
    // Wyświetl nazwę stacji, ustawiając kursor na odpowiedniej pozycji
    //         u8g2.drawStr(0, displayRow * 13 + 8, String(station).c_str());
    u8g2.drawStr(0, displayRow * 13 + 10, String(station).c_str());
    //u8g2.print(station);  // Wyświetl nazwę stacji

    // Przejdź do następnej linii (następny wiersz na ekranie)
    displayRow++;
  }
  // Przywróć domyślne ustawienia koloru rysowania (biały tekst na czarnym tle)
  u8g2.setDrawColor(1);  // Biały kolor rysowania
  u8g2.sendBuffer();     // Wyślij zawartość bufora do ekranu OLED, aby wyświetlić zmiany
  Serial.print("CurrentSelection = "); Serial.println(currentSelection);
  Serial.print("firstVisibleLine = "); Serial.println(firstVisibleLine);
}

// Funkcja do wyświetlania listy utworów z karty SD z opcją wyboru poprzez zaznaczanie w negatywie
void displayTracks() 
{
  listedTracks = true;
  u8g2.clearBuffer();  // Wyczyść bufor przed rysowaniem, aby przygotować ekran do nowej zawartości
  u8g2.setFont(u8g2_font_6x12_t_cyrillic);  // Czcionka z obsługą rozszerzonych znaków (polskie)
  u8g2.setCursor(20, 10);                                          // Ustaw pozycję kursora dla nagłówka
  u8g2.print("SD PLAYER - TRACKS: ");                              // Wyświetl nagłówek
  u8g2.print(String(currentTrackSelection + 1) + " / " + String(tracksCount));  // Numer aktualnego utworu i licznik wszystkich utworów
  if (sdPlayerSessionCompleted && sdPlayerSessionStartIndex >= 0 && sdPlayerSessionEndIndex >= 0) {
    u8g2.setCursor(140, 10);
    u8g2.print("DONE ");
    u8g2.print(sdPlayerSessionStartIndex + 1);
    u8g2.print("->");
    u8g2.print(sdPlayerSessionEndIndex + 1);
  }
  u8g2.drawLine(0,11,256,11);

  int displayRow = 1;  // Zmienna dla numeru wiersza, zaczynając od drugiego (pierwszy to nagłówek)

  // Wyświetlanie utworów, zaczynając od drugiej linii
  for (int i = firstVisibleTrack; i < min(firstVisibleTrack + maxVisibleTracks, tracksCount); i++) 
  {
    String trackName = trackFiles[i];  // Pobierz ścieżkę pliku z tablicy (np. "MUZYKA/song.mp3")
    
    // Usuń ścieżkę - zostaw tylko nazwę pliku
    int lastSlash = trackName.lastIndexOf('/');
    if (lastSlash >= 0) {
      trackName = trackName.substring(lastSlash + 1);
    }
    
    // Usuń rozszerzenie z nazwy pliku dla czytelności
    int dotIndex = trackName.lastIndexOf('.');
    if (dotIndex > 0) {
      trackName = trackName.substring(0, dotIndex);
    }
    
    // Dodaj numer utworu przed nazwą (numeracja od 1)
    String trackWithNumber = String(i + 1) + ". " + trackName;
    
    // Ogranicz długość nazwy do szerokości ekranu (około 38 znaków dla czcionki 6px - mniej bo dodaliśmy numer)
    if (trackWithNumber.length() > 38) {
      trackWithNumber = trackWithNumber.substring(0, 35) + "...";
    }

    // Sprawdź, czy bieżący utwór to ten, który jest aktualnie zaznaczony
    if (i == currentTrackSelection) 
    {
      u8g2.setDrawColor(1);                           // Ustaw biały kolor rysowania
      u8g2.drawBox(0, displayRow * 13, 256, 13);     // Narysuj prostokąt jako tło dla zaznaczonego utworu
      u8g2.setDrawColor(0);                           // Zmień kolor rysowania na czarny dla tekstu zaznaczonego utworu
    } else {
      u8g2.setDrawColor(1);  // Dla niezaznaczonych utworów ustaw zwykły biały kolor tekstu
    }
    
    // KRYTYCZNE: Ustaw czcionkę przed każdym rysowaniem tekstu (polskie znaki)
    u8g2.setFont(u8g2_font_6x12_t_cyrillic);  // Czcionka z obsługą rozszerzonych znaków
    
    // Wyświetl nazwę utworu z numerem
    u8g2.drawStr(0, displayRow * 13 + 10, trackWithNumber.c_str());

    // Przejdź do następnej linii (następny wiersz na ekranie)
    displayRow++;
  }
  
  // Przywróć domyślne ustawienia koloru rysowania (biały tekst na czarnym tle)
  u8g2.setDrawColor(1);  // Biały kolor rysowania
  u8g2.sendBuffer();     // Wyślij zawartość bufora do ekranu OLED, aby wyświetlić zmiany
  Serial.print("CurrentTrackSelection = "); Serial.println(currentTrackSelection);
  Serial.print("firstVisibleTrack = "); Serial.println(firstVisibleTrack);
}

// Funkcja pomocnicza do sprawdzania czy plik jest muzyczny
bool isAudioFile(const String& fileName) {
  return fileName.endsWith(".mp3") || fileName.endsWith(".MP3") ||
         fileName.endsWith(".flac") || fileName.endsWith(".FLAC") ||
         fileName.endsWith(".aac") || fileName.endsWith(".AAC") ||
         fileName.endsWith(".wav") || fileName.endsWith(".WAV") ||
         fileName.endsWith(".m4a") || fileName.endsWith(".M4A");
}

// Funkcja rekurencyjna do skanowania katalogów
void scanDirectory(const String& path, int& fileCount) {
  File dir = STORAGE.open(path);
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    return;
  }
  
  File entry = dir.openNextFile();
  while (entry && tracksCount < 100) {  // Limit 100 plików
    String entryName = String(entry.name());
    
    // Buduj pełną ścieżkę: path + "/" + entryName (ale unikaj podwójnych "/")
    String fullPath;
    if (path.endsWith("/")) {
      fullPath = path + entryName;
    } else {
      fullPath = path + "/" + entryName;
    }
    
    // Utwórz ścieżkę względną (bez początkowego "/")
    String relativePath = fullPath;
    if (relativePath.startsWith("/")) {
      relativePath = relativePath.substring(1);
    }
    
    if (entry.isDirectory()) {
      // Pomiń systemowe foldery Windows/Mac
      if (entryName != "System Volume Information" && 
          entryName != "$RECYCLE.BIN" && 
          entryName != ".Spotlight-V100" &&
          entryName != ".Trashes" &&
          entryName != ".fseventsd") {
        Serial.printf("[SDPlayer] Skanowanie podfolderu: %s\n", relativePath.c_str());
        scanDirectory(fullPath, fileCount);  // Rekurencja z pełną ścieżką
      }
    } else {
      // Sprawdź czy to plik muzyczny
      if (isAudioFile(entryName)) {
        trackFiles[tracksCount] = relativePath;
        tracksCount++;
        Serial.printf("[SDPlayer] Znaleziono: %s\n", relativePath.c_str());
      }
    }
    
    // KRYTYCZNE: Oddaj kontrolę co 10 plików aby uniknąć watchdog timeout
    fileCount++;
    if (fileCount % 10 == 0) {
      yield();
      delay(1);
    }
    
    entry.close();
    entry = dir.openNextFile();
  }
  
  dir.close();
}

// Funkcja do ładowania listy utworów muzycznych z karty SD (wszystkie foldery)
void loadTracksFromSD() 
{
  tracksCount = 0;  // Resetuj licznik utworów
  
  Serial.println("[SDPlayer] Skanowanie plików muzycznych na karcie SD...");
  
  // Skanuj katalog główny i wszystkie podfoldery
  int fileCount = 0;
  scanDirectory("/", fileCount);
  
  Serial.print("[SDPlayer] Łącznie znaleziono utworów: ");
  Serial.println(tracksCount);

  // KRYTYCZNE: Synchronizuj listę plików z SDPlayerOLED dla numeracji utworów
  if (g_sdPlayerOLED && tracksCount > 0) {
    g_sdPlayerOLED->syncTrackListFromIR(trackFiles, tracksCount);
  }

  if (lastPlayedTrackIndex < 0 && sdPlayerSessionEndIndex >= 0 && sdPlayerSessionEndIndex < tracksCount) {
    lastPlayedTrackIndex = sdPlayerSessionEndIndex;
  }
}

void saveSDPlayerSessionState()
{
#ifdef AUTOSTORAGE
  if (_storage == nullptr) {
    return;
  }
#endif

  File stateFile = STORAGE.open(sdPlayerSessionStateFile, FILE_WRITE);
  if (!stateFile) {
    Serial.println("[SDPlayer] BLAD zapisu stanu sesji");
    return;
  }

  stateFile.print("start=");
  stateFile.println(sdPlayerSessionStartIndex);
  stateFile.print("end=");
  stateFile.println(sdPlayerSessionEndIndex);
  stateFile.print("completed=");
  stateFile.println(sdPlayerSessionCompleted ? 1 : 0);
  stateFile.close();
}

void loadSDPlayerSessionState()
{
  sdPlayerSessionStartIndex = -1;
  sdPlayerSessionEndIndex = -1;
  sdPlayerSessionCompleted = false;

#ifdef AUTOSTORAGE
  if (_storage == nullptr) {
    return;
  }
#endif

  if (!STORAGE.exists(sdPlayerSessionStateFile)) {
    return;
  }

  File stateFile = STORAGE.open(sdPlayerSessionStateFile, FILE_READ);
  if (!stateFile) {
    Serial.println("[SDPlayer] BLAD odczytu stanu sesji");
    return;
  }

  while (stateFile.available()) {
    String line = stateFile.readStringUntil('\n');
    line.trim();

    if (line.startsWith("start=")) {
      sdPlayerSessionStartIndex = line.substring(6).toInt();
    } else if (line.startsWith("end=")) {
      sdPlayerSessionEndIndex = line.substring(4).toInt();
    } else if (line.startsWith("completed=")) {
      sdPlayerSessionCompleted = (line.substring(10).toInt() != 0);
    }
  }
  stateFile.close();

  if (sdPlayerSessionEndIndex >= 0) {
    lastPlayedTrackIndex = sdPlayerSessionEndIndex;
  }

  Serial.printf("[SDPlayer] Odczyt stanu sesji: start=%d end=%d completed=%d\n",
                sdPlayerSessionStartIndex,
                sdPlayerSessionEndIndex,
                sdPlayerSessionCompleted ? 1 : 0);
}

void clearSDPlayerSessionCompleted()
{
  sdPlayerSessionCompleted = false;
  saveSDPlayerSessionState();
}

void updateTimerFlag() 
{
  ActionNeedUpdateTime = true;
}

void displayPowerSave(bool saveON)
{
  if ((saveON == 1) && (displayActive == false) && (fwupd == false) && (audio.isRunning() == true)) {u8g2.setPowerSave(1);}
  if (saveON == 0) {u8g2.setPowerSave(0);}
}



// Funkcja wywoływana co sekundę przez timer do aktualizacji czasu na wyświetlaczu
void updateTime() 
{
  u8g2.setDrawColor(1);  // Ustaw kolor na biały
  bool showDots;

  if (timeDisplay == true) 
  {

    if ((timeDisplay == true) && (audio.isRunning() == true))
    {
      // Struktura przechowująca informacje o czasie
      struct tm timeinfo;
      
      // Sprawdź, czy udało się pobrać czas z lokalnego zegara czasu rzeczywistego
      if (!getLocalTime(&timeinfo, 3000)) 
      {
        // Wyświetl komunikat o niepowodzeniu w pobieraniu czasu
        Serial.println("debug time -> Nie udało się uzyskać czasu");
        return;  // Zakończ funkcję, gdy nie udało się uzyskać czasu
      }
      
      showDots = (timeinfo.tm_sec % 2 == 0); // Parzysta sekunda = pokazuj dwukropek

      // Konwertuj godzinę, minutę i sekundę na stringi w formacie "HH:MM:SS"
      char timeString[9];  // Bufor przechowujący czas w formie tekstowej
      if ((timeinfo.tm_min == 0) && (timeinfo.tm_sec == 0) && (timeVoiceInfoEveryHour == true) && (f_voiceTimeBlocked == false)) {f_requestVoiceTimePlay = true; f_voiceTimeBlocked = true;} // Sprawdzamy czy wybiła pełna godzina, jesli włączony informacja głosowa co godzine to odtwarzamy
      
      if ((displayMode == 0) || (displayMode == 2)) // Tryb podstawowy i 3 linijki tesktu
      { 
        //snprintf(timeString, sizeof(timeString), "%2d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        if (showDots) snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        else snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);
        u8g2.setFont(spleen6x12PL);
        //u8g2.drawStr(208, 63, timeString);
        u8g2.drawStr(226, 63, timeString);
      }
      else if (displayMode == 1) // Tryb - Duzy zegar
      {
        int xtime = 0;
        u8g2.setFont(u8g2_font_7Segments_26x42_mn);
        
        //snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);

        //Miganie kropek
        if (showDots) snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        else snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);
        u8g2.drawStr(xtime+7, 45, timeString); 
        
        u8g2.setFont(u8g2_font_fub14_tf); // 14x11
        snprintf(timeString, sizeof(timeString), "%02d", timeinfo.tm_mday);
        u8g2.drawStr(203,17, timeString);

        String month = "";
        switch (timeinfo.tm_mon) {
        case 0: month = "JAN"; break;     
        case 1: month = "FEB"; break;     
        case 2: month = "MAR"; break;
        case 3: month = "APR"; break;
        case 4: month = "MAY"; break;
        case 5: month = "JUN"; break;
        case 6: month = "JUL"; break;
        case 7: month = "AUG"; break;
        case 8: month = "SEP"; break;
        case 9: month = "OCT"; break;
        case 10: month = "NOV"; break;
        case 11: month = "DEC"; break;                                
        }
        u8g2.setFont(spleen6x12PL);
        u8g2.drawStr(232,14, month.c_str());

        String dayOfWeek = "";
        switch (timeinfo.tm_wday) {
        case 0: dayOfWeek = " Sunday  "; break;     
        case 1: dayOfWeek = " Monday  "; break;     
        case 2: dayOfWeek = " Tuesday "; break;
        case 3: dayOfWeek = "Wednesday"; break;
        case 4: dayOfWeek = "Thursday "; break;
        case 5: dayOfWeek = " Friday  "; break;
        case 6: dayOfWeek = "Saturday "; break;
        }
        
        u8g2.drawRBox(198,20,58,15,3);  // Box z zaokraglonymi rogami, biały pod dniem tygodnia
        u8g2.drawLine(198,20,256,20); // Linia separacyjna dzien miesiac / dzien tygodnia
        u8g2.setDrawColor(0);
        u8g2.drawStr(201,31, dayOfWeek.c_str());
        u8g2.setDrawColor(1);
        u8g2.drawRFrame(198,0,58,35,3); // Ramka na całosci kalendarza

        snprintf(timeString, sizeof(timeString), ":%02d", timeinfo.tm_sec);
        u8g2.drawStr(xtime+163, 45, timeString);
      }
      else if (displayMode == 3)// || displayMode == 8)
      { 
        if (showDots) snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        else snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);
        u8g2.setFont(spleen6x12PL);
        //u8g2.drawStr(113, 11, timeString);
        u8g2.drawStr(226, 10, timeString);
      }
      else if (displayMode == 8)
      { 
        if (showDots) snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
        else snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);
        u8g2.setFont(spleen6x12PL);
        u8g2.drawStr(50, 8, timeString);
      }
    
    }
    else if ((timeDisplay == true) && (audio.isRunning() == false))
    {
      displayPowerSave(0); 
      // Struktura przechowująca informacje o czasie
      struct tm timeinfo;
      
      if (!getLocalTime(&timeinfo,3000)) 
      {
        Serial.println("debug time -> Nie udało się uzyskać czasu");
        return;  // Zakończ funkcję, gdy nie udało się uzyskać czasu
      }
      showDots = (timeinfo.tm_sec % 2 == 0); // Parzysta sekunda = pokazuj dwukropek
      char timeString[9];  // Bufor przechowujący czas w formie tekstowej
      
      //snprintf(timeString, sizeof(timeString), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      if (showDots) snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
      else snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);
      u8g2.setFont(spleen6x12PL);
      
      if ((displayMode == 0) || (displayMode == 2)) { u8g2.drawStr(0, 63, "                         ");}
      if (displayMode == 1) { u8g2.drawStr(0, 33, "                         ");}
      if (displayMode == 3) { u8g2.drawStr(53, 62, "                         ");}

      if (millis() - lastCheckTime >= 1000)
      {
        if ((displayMode == 0) || (displayMode == 2)) { u8g2.drawStr(0, 63, "... No audio stream ! ...");}
        if (displayMode == 1) { u8g2.drawStr(0, 33, "... No audio stream ! ...");}
        if (displayMode == 3) { u8g2.drawStr(53, 62 , "... No audio stream ! ...");}
        lastCheckTime = millis(); // Zaktualizuj czas ostatniego sprawdzenia
      }       
      if ((displayMode == 0) || (displayMode == 1) || (displayMode == 2)) { u8g2.drawStr(226, 63, timeString);}
      if (displayMode == 3) { u8g2.drawStr(226, 10, timeString);}
      //if (displayMode == 8) { u8g2.drawStr(226, 10, timeString);}
    }
  }
}

void saveEqualizerOnSD() 
{
  #if defined(ENABLE_EQ16) && (ENABLE_EQ16 == 1)
  // Jeśli aktywny jest tryb EQ16, zapisz ustawienia EQ16
  if (eq16Mode) {
    EQ16_saveToSD();
    return;
  }
  #endif
  
  // W przeciwnym razie zapisz ustawienia EQ3K
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tf); // cziocnka 14x11
  u8g2.drawStr(1, 33, "Saving EQ3K settings"); // 8 znakow  x 11 szer
  u8g2.sendBuffer();
  
  
  // Sprawdź, czy plik equalizer.txt istnieje

  Serial.print("Filtr High: ");
  Serial.println(toneHiValue);
  
  Serial.print("Filtr Mid: ");
  Serial.println(toneMidValue);
  
  Serial.print("Filtr Low: ");
  Serial.println(toneLowValue);
  
  Serial.print("Balance: ");
  Serial.println(balanceValue);
  
  
  // Sprawdź, czy plik istnieje
  if (STORAGE.exists("/equalizer.txt")) 
  {
    Serial.println("debug SD -> Plik equalizer.txt już istnieje.");

    // Otwórz plik do zapisu i nadpisz aktualną wartość flitrów equalizera
    myFile = STORAGE.open("/equalizer.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println(toneHiValue);
	    myFile.println(toneMidValue);
	    myFile.println(toneLowValue);
      myFile.println(balanceValue);
      myFile.close();
      Serial.println("debug SD -> Aktualizacja equalizer.txt na karcie SD");
    } 
	  else 
	  {
      Serial.println("debug SD -> Błąd podczas otwierania pliku equalizer.txt.");
    }
  } 
  else 
  {
    Serial.println("debug SD -> Plik equalizer.txt nie istnieje. Tworzenie...");

    // Utwórz plik i zapisz w nim aktualną wartość filtrów equalizera
    myFile = STORAGE.open("/equalizer.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println(toneHiValue);
	    myFile.println(toneMidValue);
	    myFile.println(toneLowValue);
      myFile.println(balanceValue);
      myFile.close();
      Serial.println("debug SD -> Utworzono i zapisano equalizer.txt na karcie SD");
    } 
	  else 
	  {
      Serial.println("debug SD -> Błąd podczas tworzenia pliku equalizer.txt.");
    }
  }
}

void readEqualizerFromSD() 
{
  // Sprawdź, czy karta SD jest dostępna
  //if (!STORAGE.begin(SD_CS))
  if (!STORAGE_BEGIN()) 
  {
    Serial.println("debug SD -> Nie można znaleźć karty SD, ustawiam domyślne wartości filtrow Equalziera.");
    toneHiValue = 0;  // Domyślna wartość filtra gdy brak karty SD
	  toneMidValue = 0; // Domyślna wartość filtra gdy brak karty SD
	  toneLowValue = 0; // Domyślna wartość filtra gdy brak karty SD
    balanceValue = 0;
    return;
  }

  // Sprawdź, czy plik equalizer.txt istnieje
  if (STORAGE.exists("/equalizer.txt")) 
  {
    myFile = STORAGE.open("/equalizer.txt");
    if (myFile) 
    {
      toneHiValue = myFile.parseInt();
	    toneMidValue = myFile.parseInt();
	    toneLowValue = myFile.parseInt();
      balanceValue = myFile.parseInt();
      myFile.close();
      
      Serial.println("debug SD -> Wczytano equalizer.txt z karty SD: ");
	  
      Serial.print("debug SD -> Filtr High equalizera odczytany z SD: ");
      Serial.println(toneHiValue);
    
      Serial.print("debug SD -> Filtr Mid equalizera odczytany z SD: ");
      Serial.println(toneMidValue);
    
      Serial.print("debug SD -> Filtr Low equalizera odczytany z SD: ");
      Serial.println(toneLowValue);
	  
      
    } 
	  else 
	  {
      Serial.println("debug SD -> Błąd podczas otwierania pliku equalizer.txt.");
    }
  } 
  else 
  {
    Serial.println("debug SD -> Plik equalizer.txt nie istnieje.");
    toneHiValue = 0;  // Domyślna wartość filtra gdy brak karty SD
	  toneMidValue = 0; // Domyślna wartość filtra gdy brak karty SD
	  toneLowValue = 0; // Domyślna wartość filtra gdy brak karty SD
  }
  audio.setTone(toneLowValue, toneMidValue, toneHiValue); // Ustawiamy filtry - zakres regulacji -40 + 6dB jako int8_t ze znakiem
}

// Funkcja do odczytu danych stacji radiowej z karty SD
void readStationFromSD() 
{
  // Sprawdź, czy karta SD jest dostępna
  //if (!STORAGE.begin(SD_CS)) 
  if (!STORAGE_BEGIN())
  {
    //Serial.println("Nie można znaleźć karty STORAGE. Ustawiam domyślne wartości: Station=1, Bank=1.");
    Serial.println("debug SD -> Nie można znaleźć karty SD, ustawiam wartości z EEPROMu");
    //station_nr = 1;  // Domyślny numer stacji gdy brak karty SD
    //bank_nr = 1;     // Domyślny numer banku gdy brak karty SD
    EEPROM.get(0, station_nr);
    EEPROM.get(1, bank_nr);
    
    Serial.print("debug EEPROM -> Odczyt EEPROM Stacja: ");
    Serial.println(station_nr);
    Serial.print("debug EEPROM -> Odczyt EEPROM Bank: ");
    Serial.println(bank_nr);

    if ((station_nr > 99) || (station_nr == 0)) { station_nr = 1;} // zabezpiecznie na wypadek błędnego odczytu EEPROMu lub wartości
    if ((bank_nr > bank_nr_max) || (bank_nr == 0)) { bank_nr = 1;}
    
    return;
  }

  // Sprawdź, czy plik station_nr.txt istnieje
  if (STORAGE.exists("/station_nr.txt")) {
    myFile = STORAGE.open("/station_nr.txt");
    if (myFile) {
      station_nr = myFile.parseInt();
      myFile.close();
      Serial.print("debug SD -> Wczytano station_nr z karty SD: ");
      Serial.println(station_nr);
    } else {
      Serial.println("debug SD -> Błąd podczas otwierania pliku station_nr.txt.");
    }
  } else {
    Serial.println("debug SD -> Plik station_nr.txt nie istnieje.");
    station_nr = 9;  // ustawiamy stacje w przypadku braku pliku na karcie
  }

  // Sprawdź, czy plik bank_nr.txt istnieje
  if (STORAGE.exists("/bank_nr.txt")) {
    myFile = STORAGE.open("/bank_nr.txt");
    if (myFile) {
      bank_nr = myFile.parseInt();
      myFile.close();
      Serial.print("debug SD -> Wczytano bank_nr z karty SD: ");
      Serial.println(bank_nr);
    } else {
      Serial.println("debug SD -> Błąd podczas otwierania pliku bank_nr.txt.");
    }
  } else {
    Serial.println("debug SD -> Plik bank_nr.txt nie istnieje.");
    bank_nr = 1;  // // ustawiamy bank w przypadku braku pliku na karcie
  }
}

/*
void vuMeterMode0new() 
{
    // Odczyt VU
    uint16_t raw = audio.getVUlevel();
    int L = (raw >> 8) & 0xFF;
    int R = raw & 0xFF;

    // ograniczenie wartości do 0–243
    vuMeterL = constrain(L, 0, 243);
    vuMeterR = constrain(R, 0, 243);

    // zapobiega nagłym spadkom przy wysokich sygnałach
    const int fallLimit = 8;
    if (vuSmooth) 
    {
        // LEFT
        if (vuMeterL > displayVuL) 
        {
            displayVuL += vuRiseSpeed;
            if (displayVuL > vuMeterL) displayVuL = vuMeterL;
        } 
        else 
        {
            int drop = displayVuL - vuMeterL;
            if (drop > fallLimit) drop = fallLimit;
            displayVuL -= drop;
        }

        // RIGHT
        if (vuMeterR > displayVuR) 
        {
            displayVuR += vuRiseSpeed;
            if (displayVuR > vuMeterR) displayVuR = vuMeterR;
        } 
        else 
        {
            int drop = displayVuR - vuMeterR;
            if (drop > fallLimit) drop = fallLimit;
            displayVuR -= drop;
        }
    } 
    else 
    {
        displayVuL = vuMeterL;
        displayVuR = vuMeterR;
    }

    // Aktualizacja peak&hold
    if (vuPeakHoldOn) 
    {
        // LEFT
        if (displayVuL >= peakL) { peakL = displayVuL; peakHoldTimeL = 0; } 
        else if (peakHoldTimeL < peakHoldThreshold) peakHoldTimeL++; 
        else if (peakL > 0) peakL--;

        // RIGHT
        if (displayVuR >= peakR) { peakR = displayVuR; peakHoldTimeR = 0; } 
        else if (peakHoldTimeR < peakHoldThreshold) peakHoldTimeR++; 
        else if (peakR > 0) peakR--;
    }

    // Rysowanie na ekranie
    if (!volumeMute) 
    {
        u8g2.setDrawColor(0);
        u8g2.drawBox(7, vuLy, 256, 3);
        u8g2.drawBox(7, vuRy, 256, 3);
        u8g2.setDrawColor(1);

        // Białe pola pod literami
        u8g2.drawBox(0, vuLy - 3, 7, 7);
        u8g2.drawBox(0, vuRy - 3, 7, 7);

        // Litery L i R
        u8g2.setDrawColor(0);
        u8g2.setFont(u8g2_font_04b_03_tr);
        u8g2.drawStr(2, vuLy + 3, "L");
        u8g2.drawStr(2, vuRy + 3, "R");
        u8g2.setDrawColor(1);

        if (vuMeterMode == 1) // ciągłe paski
        {
            u8g2.drawBox(10, vuLy, displayVuL, 2);
            u8g2.drawBox(10, vuRy, displayVuR, 2);
            u8g2.drawBox(9 + peakL, vuLy, 1, 2);
            u8g2.drawBox(9 + peakR, vuRy, 1, 2);
        } 
        else // przerwane paski
        {
            for (uint8_t i = 0; i < displayVuL; i++) if ((i % 9) < 8) u8g2.drawBox(9 + i, vuLy, 1, 2);
            for (uint8_t i = 0; i < displayVuR; i++) if ((i % 9) < 8) u8g2.drawBox(9 + i, vuRy, 1, 2);
            if (vuPeakHoldOn)
            {
                u8g2.drawBox(9 + peakL, vuLy, 1, 2);
                u8g2.drawBox(9 + peakR, vuRy, 1, 2);
            }
        }
    }
}
*/

void vuMeterMode0() 
{
  // FIX: Clamping do 0-127 (bugfix z ESP32-audioI2S 3.4.4x commit #8)
  uint16_t raw = audio.getVUlevel();
  vuMeterL = min((raw >> 8) & 0x7F, 127);  // Clamp left channel 0-127
  vuMeterR = min(raw & 0x7F, 127);         // Clamp right channel 0-127

  //vuMeterL = constrain(vuMeterL, 0, 243);
  //vuMeterR = constrain(vuMeterR, 0, 243);

  // zapobiega nagłym spadkom przy bardzo silnym sygnale
  //if (vuMeterL < displayVuL - 8) vuMeterL = displayVuL - 8;
  //if (vuMeterR < displayVuR - 8) vuMeterR = displayVuR - 8;

  
  //vuMeterR = audio.getVUlevel() & 0xFF;  // wyciagamy ze zmiennej typu int16 kanał L
  //vuMeterL = audio.getVUlevel() >> 8;  // z wyzszej polowki wyciagamy kanal P

  //vuMeterL = constrain(vuMeterL, 0, 243);
  //vuMeterR = constrain(vuMeterR, 0, 243);

  vuMeterR = map(vuMeterR, 0, 255, 0, 243);
  vuMeterL = map(vuMeterL, 0, 255, 0, 243);  // 244 VU + start od x=10 +  peak hold 2px


  if (vuSmooth)
  {
    // LEFT
    if (vuMeterL > displayVuL) 
    {
      displayVuL += vuRiseSpeed;
      if (displayVuL > vuMeterL) displayVuL = vuMeterL;
    } 
    else if (vuMeterL < displayVuL) 
    {
      if (displayVuL > vuFallSpeed) 
      {
        displayVuL -= vuFallSpeed;
      } 
      else 
      {
        displayVuL = 0;
      }
    }

    // RIGHT
    if (vuMeterR > displayVuR) 
    {
      displayVuR += vuRiseSpeed;
      if (displayVuR > vuMeterR) displayVuR = vuMeterR;
    } 
    else if (vuMeterR < displayVuR) 
    {
      if (displayVuR > vuFallSpeed) 
      {
        displayVuR -= vuFallSpeed;
      } 
      else 
      {
        displayVuR = 0;
      }
    }
  }

  // Aktualizacja peak&hold dla Lewego kanału
  if (vuSmooth)
  {
    if (vuPeakHoldOn)
    {
      // --- Peak L ---
      if (displayVuL >= peakL) 
      {
        peakL = displayVuL;
        peakHoldTimeL = 0;
      } 
      else 
      {
        if (peakHoldTimeL < peakHoldThreshold) 
        {
          peakHoldTimeL++;
        } 
        else 
        {
          if (peakL > 0) peakL--;
        }
      }

      // --- Peak R ---
      if (displayVuR >= peakR) 
      {
        peakR = displayVuR;
        peakHoldTimeR = 0;
      } 
      else 
      {
        if (peakHoldTimeR < peakHoldThreshold) 
        {
          peakHoldTimeR++;
        } 
        else 
        {
          if (peakR > 0) peakR--;
        }
      }
    }    
  }
  else
  {
    if (vuPeakHoldOn)
    {
      if (vuMeterL >= peakL) 
      {
        peakL = vuMeterL;
        peakHoldTimeL = 0;
      } 
      else 
      {
        if (peakHoldTimeL < peakHoldThreshold) 
        {
          peakHoldTimeL++;
        } 
        else 
        {
          if (peakL > 0) peakL--;
        }
      }
      // Aktualizacja peak&hold dla Prawego kanału
      if (vuMeterR >= peakR) 
      {
        peakR = vuMeterR;
        peakHoldTimeR = 0;
      } 
      else 
      {
        if (peakHoldTimeR < peakHoldThreshold) 
        {
          peakHoldTimeR++;
        } 
        else 
        {
          if (peakR > 0) peakR--;
        }
      }
    }
  }


  if (volumeMute == false)  
  {
    if (vuSmooth)
    {
      u8g2.setDrawColor(0);
      u8g2.drawBox(7, vuLy, 249, 3);  //czyszczenie ekranu pod VU meter
      u8g2.drawBox(7, vuRy, 249, 3);
      u8g2.setDrawColor(1);

      // Biale pola pod literami L i R
      u8g2.drawBox(0, vuLy - 3, 7, 7);  
      u8g2.drawBox(0, vuRy - 3, 7, 7);  

      // Rysujemy litery L i R
      u8g2.setDrawColor(0);
      u8g2.setFont(u8g2_font_04b_03_tr);
      u8g2.drawStr(2, vuLy + 3, "L");
      u8g2.drawStr(2, vuRy + 3, "R");
      u8g2.setDrawColor(1);  // Przywracamy białe rysowanie

      if (vuMeterMode == 1)  // tryb 1 ciagle paski
      {
        u8g2.setDrawColor(1);
        //u8g2.drawBox(10, vuLy, vuMeterL, 2);  // rysujemy kreseczki o dlugosci odpowiadajacej wartosci VU
        //u8g2.drawBox(10, vuRy, vuMeterR, 2);

        u8g2.drawBox(10, vuLy, displayVuL, 2);  // rysujemy kreseczki o dlugosci odpowiadajacej wartosci VU
        u8g2.drawBox(10, vuRy, displayVuR, 2);


        // Rysowanie peaków jako cienka kreska
        u8g2.drawBox(9 + peakL, vuLy, 1, 2);
        u8g2.drawBox(9 + peakR, vuRy, 1, 2);
      } 
      else  // vuMeterMode == 0  tryb podstawowy, kreseczki z przerwami
      {      
        for (uint8_t vusize = 0; vusize < displayVuL; vusize++)
        {
          if ((vusize % 9) < 8) u8g2.drawBox(9 + vusize, vuLy, 1, 2);//u8g2.drawBox(9 + vusize, vuLy, 1, 2); // rysuj tylko 8 pikseli, potem 1px przerwy, 9 w osi x to odstep na literke
        }  
        for (uint8_t vusize = 0; vusize < displayVuR; vusize++)
        {
          if ((vusize % 9) < 8) u8g2.drawBox(9 + vusize, vuRy, 1, 2); // rysuj tylko 8 pikseli, potem 1px przerwy, 9 w osi x to odstep na literke
        }    
        
        if (vuPeakHoldOn)
        {
          // Peak - kreski w trybie przerywanym
          u8g2.drawBox(9 + peakL, vuLy, 1, 2);
          u8g2.drawBox(9 + peakR, vuRy, 1, 2);
        }
      }
         
    }
    else
    {
      u8g2.setDrawColor(0);
      u8g2.drawBox(7, vuLy, 256, 3);  //czyszczenie ekranu pod VU meter
      u8g2.drawBox(7, vuRy, 256, 3);
      u8g2.setDrawColor(1);

      // Biale pola pod literami L i R
      u8g2.drawBox(0, vuLy - 3, 7, 7);  
      u8g2.drawBox(0, vuRy - 3, 7, 7);  

      // Rysujemy litery L i R
      u8g2.setDrawColor(0);
      u8g2.setFont(u8g2_font_04b_03_tr);
      u8g2.drawStr(2, vuLy + 3, "L");
      u8g2.drawStr(2, vuRy + 3, "R");
      u8g2.setDrawColor(1);  // Przywracamy białe rysowanie

      if (vuMeterMode == 1)  // tryb 1 ciagle paski
      {
        u8g2.setDrawColor(1);
        u8g2.drawBox(10, vuLy, vuMeterL, 2);  // rysujemy kreseczki o dlugosci odpowiadajacej wartosci VU
        u8g2.drawBox(10, vuRy, vuMeterR, 2);

        // Rysowanie peaków jako cienka kreska
        u8g2.drawBox(9 + peakL, vuLy, 1, 2);
        u8g2.drawBox(9 + peakR, vuRy, 1, 2);
      } 
      else  // vuMeterMode == 0  tryb podstawowy, kreseczki z przerwami
      {      
        for (uint8_t vusize = 0; vusize < vuMeterL; vusize++) 
        {
          if ((vusize % 9) < 8) u8g2.drawBox(9 + vusize, vuLy, 1, 2);//u8g2.drawBox(9 + vusize, vuLy, 1, 2); // rysuj tylko 8 pikseli, potem 1px przerwy, 9 w osi x to odstep na literke
        }  
        for (uint8_t vusize = 0; vusize < vuMeterR; vusize++) 
        {
          if ((vusize % 9) < 8) u8g2.drawBox(9 + vusize, vuRy, 1, 2); // rysuj tylko 8 pikseli, potem 1px przerwy, 9 w osi x to odstep na literke
        } 
       
        if (vuPeakHoldOn)
        {
          // Peak - kreski w trybie przerywanym
          u8g2.drawBox(9 + peakL, vuLy, 1, 2);
          u8g2.drawBox(9 + peakR, vuRy, 1, 2);
        }
      }  
    } // else smooth
  }  // moute off
}

void vuMeterMode3() 
{
  // FIX: Clamping do 0-127 (bugfix z ESP32-audioI2S 3.4.4x commit #8)
  uint16_t raw = audio.getVUlevel();
  vuMeterR = min(raw & 0x7F, 127);         // Clamp right 0-127
  vuMeterL = min((raw >> 8) & 0x7F, 127);  // Clamp left 0-127

  vuMeterR = map(vuMeterR, 0, 255, 0, 127);
  vuMeterL = map(vuMeterL, 0, 255, 0, 127);

  // Wygładzanie
  if (vuSmooth)
  {
    if (vuMeterL > displayVuL) {
      displayVuL += vuRiseSpeed;
      if (displayVuL > vuMeterL) displayVuL = vuMeterL;
    } else {
      if (displayVuL > vuFallSpeed) {
        displayVuL -= vuFallSpeed;
      } else {
        displayVuL = 0;
      }
    }

    if (vuMeterR > displayVuR) {
      displayVuR += vuRiseSpeed;
      if (displayVuR > vuMeterR) displayVuR = vuMeterR;
    } else {
      if (displayVuR > vuFallSpeed) {
        displayVuR -= vuFallSpeed;
      } else {
        displayVuR = 0;
      }
    }
  }

  // Peak & Hold
  if (vuPeakHoldOn)
  {
    // LEFT
    if ((vuSmooth ? displayVuL : vuMeterL) >= peakL) {
      peakL = vuSmooth ? displayVuL : vuMeterL;
      peakHoldTimeL = 0;
    } else {
      if (peakHoldTimeL < peakHoldThreshold) {
        peakHoldTimeL++;
      } else if (peakL > 0) {
        peakL--;
      }
    }

    // RIGHT
    if ((vuSmooth ? displayVuR : vuMeterR) >= peakR) {
      peakR = vuSmooth ? displayVuR : vuMeterR;
      peakHoldTimeR = 0;
    } else {
      if (peakHoldTimeR < peakHoldThreshold) {
        peakHoldTimeR++;
      } else if (peakR > 0) {
        peakR--;
      }
    }
  }

  if (!volumeMute)
  {
    // Czyszczenie linii VU
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, vuYmode3, 240, vuThicknessMode3);
    u8g2.setDrawColor(1);

    // Wybór wartości do rysowania: wygładzone lub surowe
    int leftLevel = vuSmooth ? displayVuL : vuMeterL;
    int rightLevel = vuSmooth ? displayVuR : vuMeterR;

    // Rysowanie LEWEGO kanału - od środka w lewo
    for (int i = 0; i < leftLevel; i++) {
      if ((i % 9) < 8) {  // 8px kreska + 1px przerwa
        int x = vuCenterXmode3 - 2 - i;  // -1 żeby nie nakładać się na środek
        if (x >= 0) u8g2.drawBox(x, vuYmode3, 1, vuThicknessMode3);
      }
    }

    // Rysowanie PRAWEGO kanału - od środka w prawo
    for (int i = 0; i < rightLevel; i++) {
      if ((i % 9) < 8) {
        int x = vuCenterXmode3 + 2 + i;
        if (x < 240) u8g2.drawBox(x, vuYmode3, 1, vuThicknessMode3);
      }
    }

    // Rysowanie PEAKÓW
    if (vuPeakHoldOn) {
      int peakLeftX = vuCenterXmode3 - 1 - peakL;
      int peakRightX = vuCenterXmode3 + peakR;
      if (peakLeftX >= 0) u8g2.drawBox(peakLeftX, vuYmode3, 1, vuThicknessMode3);
      if (peakRightX < 240) u8g2.drawBox(peakRightX, vuYmode3, 1, vuThicknessMode3);
    }
  }
}

void vuMeterMode4() // Mode4 eksperymetn z duzymi wskaznikami VU
{
  // FIX: Clamping do 0-127 (bugfix z ESP32-audioI2S 3.4.4x commit #8)
  uint16_t raw = audio.getVUlevel();
  uint8_t rawL = min((raw >> 8) & 0x7F, 127);  // Clamp left 0-127
  uint8_t rawR = min(raw & 0x7F, 127);         // Clamp right 0-127

  // Skalowanie do 0–100
  int vuL = map(rawL, 0, 255, 0, 100);
  int vuR = map(rawR, 0, 255, 0, 100);

  // Bezwładność analogowa
  if (vuSmooth) {
    // Lewy
    if (vuL > displayVuL) {
      displayVuL += max(1, (vuL - displayVuL) / vuRiseNeedleSpeed);  // szybciej w górę
    } else if (vuL < displayVuL) {
      displayVuL -= max(1, (displayVuL - vuL) / vuFallNeedleSpeed); // wolniej w dół
    }

    // Prawy
    if (vuR > displayVuR) {
      displayVuR += max(1, (vuR - displayVuR) / vuRiseNeedleSpeed);
    } else if (vuR < displayVuR) {
      displayVuR -= max(1, (displayVuR - vuR) / vuFallNeedleSpeed);
    }

    displayVuL = constrain(displayVuL, 0, 100);
    displayVuR = constrain(displayVuR, 0, 100);
  } else {
    displayVuL = vuL;
    displayVuR = vuR;
  }

  // Parametry wskaźników
  const int radius = 45;
  const int frameWidth = 120;
  const int frameHeight = 60;

  int centerXL = 65;
  int centerYL = 63;

  int centerXR = 195;
  int centerYR = 63;

  const int arcStartDeg = -60;
  const int arcEndDeg = 60;

  // Czyszczenie ekranu
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 0, 256, 64);
  u8g2.setDrawColor(1);
  u8g2.setFont(u8g2_font_6x10_tr);

  // Struktura opisów dB
  struct Mark {
    int angle;
    const char* label;
  };

  const Mark scaleMarks[] = {
    { -60, "-20" },
    { -40, "-10" },
    { -20, "-5"  },
    {   0,  "0"   },
    {  20, "+3"  },
    {  40, "+6"  },
    {  60, "+9"  },
  };

  // Funkcja rysująca łuk wskaźnika
  auto drawVUArc = [&](int cx, int cy, const char* label) 
  {
    // Ramka
    u8g2.drawFrame((cx - frameWidth / 2), cy - radius - 18, frameWidth, frameHeight);

    // Skala łuku
    for (int a = arcStartDeg; a <= arcEndDeg; a += 6) 
    {
      float angle = radians(a);
      int x1 = cx + sin(angle) * (radius - 4);
      int y1 = cy - cos(angle) * (radius - 4);
      int x2 = cx + sin(angle) * radius;
      int y2 = cy - cos(angle) * radius;
      u8g2.drawLine(x1, y1, x2, y2);
    }

    // Opisy dB
    for (const Mark& m : scaleMarks) 
    {
      float angle = radians(m.angle);
      int tx = cx + sin(angle) * (radius + 10);
      int ty = cy - cos(angle) * (radius + 10);
      u8g2.setCursor(tx - 8, ty + 5);
      u8g2.print(m.label);
    }

    // Opis kanału (L/R)
    u8g2.setCursor(cx - 2, cy - 18);
    u8g2.print(label);
  };

  // Rysowanie wskaźników L i R
  drawVUArc(centerXL, centerYL,"L"); //x,y,label (L)
  drawVUArc(centerXR, centerYR,"R"); //x,y,label (R)

  // Igła lewa
  float angleL = radians(arcStartDeg + (arcEndDeg - arcStartDeg) * displayVuL / 100.0);
  int xNeedleL = centerXL + sin(angleL) * (radius - 6);
  int yNeedleL = centerYL - cos(angleL) * (radius - 6);
  u8g2.drawLine(centerXL, centerYL - 8, xNeedleL, yNeedleL);

  // Igła prawa
  float angleR = radians(arcStartDeg + (arcEndDeg - arcStartDeg) * displayVuR / 100.0);
  int xNeedleR = centerXR + sin(angleR) * (radius - 6);
  int yNeedleR = centerYR - cos(angleR) * (radius - 6);
  u8g2.drawLine(centerXR, centerYR - 8, xNeedleR, yNeedleR);
}

void vuMeterMode5() // Mode5 – VU analogowe z prostokątną skalą
{
  // ===== Odczyt VU =====
  uint8_t rawL = audio.getVUlevel() >> 8;
  uint8_t rawR = audio.getVUlevel() & 0xFF;

  int vuL = map(rawL, 0, 255, 0, 100);
  int vuR = map(rawR, 0, 255, 0, 100);

  if (vuL < 1) vuL = 1;
  if (vuR < 1) vuR = 1;
  
  // ===== Bezwładność =====
  if (vuSmooth) {
    if (vuL > displayVuL)
      displayVuL += max(1, (vuL - displayVuL) / vuRiseNeedleSpeed);
    else
      displayVuL -= max(1, (displayVuL - vuL) / vuFallNeedleSpeed);

    if (vuR > displayVuR)
      displayVuR += max(1, (vuR - displayVuR) / vuRiseNeedleSpeed);
    else
      displayVuR -= max(1, (displayVuR - vuR) / vuFallNeedleSpeed);

    displayVuL = constrain(displayVuL, 0, 100);
    displayVuR = constrain(displayVuR, 0, 100);
  } else {
    displayVuL = vuL;
    displayVuR = vuR;
  }

  // ===== Parametry wskaźników =====
  const int frameWidth = 120;
  const int frameHeight = 60;
  const int scaleWidth = 100;
  const int scaleYTopOffset = 23;
  const int majorTickHeight = 9;
  const int minorTickHeight = 5;
  const int baseLineThick = 2;

  int centerXL = 65;
  int centerXR = 195;
  int centerY  = 30;

  // ===== Czyszczenie =====
  u8g2.setDrawColor(0);
  u8g2.drawBox(0, 0, 256, 64);
  u8g2.setDrawColor(1);
  u8g2.setFont(mono04b03b); 
  
  // ===== Ramki wskaźników z wypełnieniem =====
  if (reversColorMode4) 
  {
    u8g2.drawRBox(centerXL - frameWidth / 2, centerY - frameHeight / 2, frameWidth, frameHeight,3);
    u8g2.drawRBox(centerXR - frameWidth / 2, centerY - frameHeight / 2, frameWidth, frameHeight,3);
    u8g2.setDrawColor(0); // Rysujemy wszystko w negatywnie z białym tłem VUmeterow
  }
  else
  {
    u8g2.drawRFrame(centerXL - frameWidth / 2, centerY - frameHeight / 2, frameWidth, frameHeight,3);
    u8g2.drawRFrame(centerXR - frameWidth / 2, centerY - frameHeight / 2, frameWidth, frameHeight,3);
  }
  
  struct Label {
    uint8_t pos;       // 0–100
    const char* txt;
  };
  
  // ===== Pozycje DUŻYCH kresek skali (0–100) =====
  const uint8_t majorPos[] = {
    0,   // -20
    18,  // -10
    35,  // -5
    50,  // środek
    62,  // 0 dB
    74,  // +3
    87,  // +6
    100  // +9
  };

  const uint8_t MAJOR_COUNT = sizeof(majorPos) / sizeof(majorPos[0]);

  const Label scaleLabels[] = {
    { 0,   "-20" },
    { 18,  "-10" },
    { 35,  "-5"  },
    { 62,  "0"   },
    { 72,  "+3"  },
    { 86,  "+6"  },
    { 100, "+9"  }
  };
  const uint8_t LABEL_COUNT = sizeof(scaleLabels) / sizeof(scaleLabels[0]);  
  
  auto drawLinearScale = [&](int centerX)
  {
    int leftX  = centerX - scaleWidth / 2;
    int railY1 = centerY - frameHeight / 2 + scaleYTopOffset;
    int railY2 = railY1 - 3;

    // === Helper: tick ===
    auto drawTick = [&](int x, int h)
    {
      int midX = leftX + scaleWidth / 2;
      int dx = 0;

      if (x < midX - 1)      dx = -h / 2;
      else if (x > midX + 1) dx =  h / 2;

      u8g2.drawLine(x, railY1, x + dx, railY1 - h);
    };

    // === Rails ===
    u8g2.drawLine(leftX, railY2+1, leftX + scaleWidth, railY2+1);
    u8g2.drawLine(leftX + 62 , railY2 + 2, leftX + scaleWidth, railY2+2);

    // --- Linia dolna wskaznika ---
    u8g2.drawLine(leftX, railY1, leftX + scaleWidth, railY1);
    u8g2.drawLine(leftX, railY1 + 1, leftX + scaleWidth, railY1 + 1); // pogrubienie

    // === Major ticks ===
    for (uint8_t i = 0; i < MAJOR_COUNT; i++) {
      int x = leftX + (majorPos[i] * scaleWidth) / 100;
      drawTick(x, majorTickHeight);
    }
    
    // === Opisy skali ===
    for (uint8_t i = 0; i < LABEL_COUNT; i++) {
      int x = leftX + (scaleLabels[i].pos * scaleWidth) / 100;
      u8g2.setCursor(x - 5, railY1 + 10);
      u8g2.print(scaleLabels[i].txt);
    }

    // === Minor ticks: 2 pomiędzy KAŻDĄ parą major ===
    for (uint8_t i = 0; i < MAJOR_COUNT - 1; i++)
    {
      int x1 = leftX + (majorPos[i]     * scaleWidth) / 100;
      int x2 = leftX + (majorPos[i + 1] * scaleWidth) / 100;

      int step = (x2 - x1) / 3;
      drawTick(x1 + step,     minorTickHeight);
      drawTick(x1 + 2 * step, minorTickHeight);
    }
  };

  // ===== Rysowanie skal =====
  drawLinearScale(centerXL);
  drawLinearScale(centerXR);

  // ===== Opis kanałów =====
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setCursor(centerXL - 5, centerY + 18); u8g2.print("VU");
  u8g2.setCursor(centerXR - 5, centerY + 18); u8g2.print("VU");

  // ===== Igły =====
  const int radius = 45;
  float angleL = radians(-70 + 140.0 * displayVuL / 100.0);
  float angleR = radians(-70 + 140.0 * displayVuR / 100.0);

  int xNeedleL = centerXL + sin(angleL) * (radius - 6);
  int yNeedleL = centerY + 15 - cos(angleL) * (radius - 6);
  
  int xNeedleR = centerXR + sin(angleR) * (radius - 6);
  int yNeedleR = centerY + 15 - cos(angleR) * (radius - 6);

  u8g2.drawLine(centerXL, centerY + 24, xNeedleL, yNeedleL);
  u8g2.drawLine(centerXR, centerY + 24, xNeedleR, yNeedleR);
  u8g2.setDrawColor(1);
}

void showIP(uint16_t xip, uint16_t yip)
{
  u8g2.setFont(spleen6x12PL);
  u8g2.setDrawColor(1);
  u8g2.setCursor(xip,yip);
  u8g2.print("IP: " + currentIP);
}

void vuMeterMode7() // Mode7 - Duże paski VU na cały ekran
{
  u8g2.clearBuffer(); // Wyczyść cały bufor ekranu przed rysowaniem
  
  uint16_t raw = audio.getVUlevel();
  vuMeterL = (raw >> 8) & 0xFF;
  vuMeterR = raw & 0xFF;

  if (vuMeterR < 1) vuMeterR = 0;
  if (vuMeterL < 1) vuMeterL = 0;

  if (vuSmooth)
  {
    // LEFT
    if (vuMeterL > displayVuL) 
    {
      if (displayVuL > 255 - vuRiseSpeed) displayVuL = 255;
      else
      displayVuL += vuRiseSpeed;

      if (displayVuL > vuMeterL) displayVuL = vuMeterL;
    }
    else if (vuMeterL < displayVuL) 
    {
      if (displayVuL > vuFallSpeed) 
      {
        displayVuL -= vuFallSpeed;
      } 
      else 
      {
        displayVuL = 0;
      }
    }

    // RIGHT
    if (vuMeterR > displayVuR) 
    {
      if (displayVuR > 255 - vuRiseSpeed) displayVuR = 255;
      else
      displayVuR += vuRiseSpeed;

      if (displayVuR > vuMeterR) displayVuR = vuMeterR;
    }
    else if (vuMeterR < displayVuR) 
    {
      if (displayVuR > vuFallSpeed) 
      {
        displayVuR -= vuFallSpeed;
      } 
      else 
      {
        displayVuR = 0;
      }
    }
  }

  // Aktualizacja peak&hold dla Lewego i prawego kanału
  if (vuSmooth)
  {
    if (vuPeakHoldOn)
    {
      // --- Peak L ---
      if (displayVuL >= peakL) 
      {
        peakL = displayVuL;
        peakHoldTimeL = 0;
      } 
      else 
      {
        if (peakHoldTimeL < peakHoldThreshold) 
        {
          peakHoldTimeL++;
        } 
        else 
        {
          if (peakL > 0) peakL--;
        }
      }

      // --- Peak R ---
      if (displayVuR >= peakR) 
      {
        peakR = displayVuR;
        peakHoldTimeR = 0;
      } 
      else 
      {
        if (peakHoldTimeR < peakHoldThreshold) 
        {
          peakHoldTimeR++;
        } 
        else 
        {
          if (peakR > 0) peakR--;
        }
      }
    }    
  }

  if (volumeMute == false)  
  {
    if (vuSmooth)
    {
      // Rysujemy litery L i R
      u8g2.setFont(spleen6x12PL);
      u8g2.drawStr(0, 5 + 14, "L");
      u8g2.drawStr(0, 40 + 14, "R");

      for (uint8_t vusize = 0; vusize < displayVuL; vusize++)
      {       
        if ((vusize % 5) < 4) u8g2.drawBox(9 + vusize, 5, 1, 20);
      }
      
      for (uint8_t vusize = 0; vusize < displayVuR; vusize++)
      {
        if ((vusize % 5) < 4) u8g2.drawBox(9 + vusize, 40, 1, 20);
      }    
      
      if (vuPeakHoldOn)
      {
        // Peak - kreski w trybie przerywanym
        u8g2.drawBox(9 + peakL, 5, 1, 20);
        u8g2.drawBox(9 + peakR, 40, 1, 20);
      }
    }
  } 
}

void displayClearUnderScroller() // Funkcja odpwoiedzialna za przewijanie informacji strem tittle lub stringstation
{
  // KRYTYCZNE: Blokuj gdy SDPlayer aktywny
  if (sdPlayerOLEDActive) return;
  
  if (displayMode == 0) // Tryb normalny Mode 0- radio
  {
    u8g2.setDrawColor(1);
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(0,yPositionDisplayScrollerMode0, "                                           "); //43 spacje - czyszczenie ekranu   
   } 
  else if (displayMode == 1)  // Tryb zegara - Mode 1
  {
    u8g2.drawStr(0,yPositionDisplayScrollerMode1, "                                           "); //43 znaki czyszczenie ekranu
  }
  else if (displayMode == 2)  // Tryb mały tekst - Mode 2
  {
    u8g2.setDrawColor(1);
    u8g2.setFont(spleen6x12PL);   
    u8g2.drawStr(0,yPositionDisplayScrollerMode2, "                                           "); //43 znaki czyszczenie ekranu
    u8g2.drawStr(0,yPositionDisplayScrollerMode2 + 12, "                                           "); //43 znaki czyszczenie ekranu
    u8g2.drawStr(0,yPositionDisplayScrollerMode2 + 12 + 12, "                                           "); //43 znaki czyszczenie ekranu
  }
  else if (displayMode == 3)  // Tryb mały tekst - Mode 3 || displayMode == 8
  {
    u8g2.setDrawColor(1);
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(0,yPositionDisplayScrollerMode3, "                                           "); //43 spacje - czyszczenie ekranu   
  }
  else if (displayMode == 8)  // Tryb mały tekst - Mode 3 || displayMode == 8
  {
    u8g2.setDrawColor(1);
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(0,yPositionDisplayScrollerMode8, "                        "); //43 spacje - czyszczenie ekranu   
  }
  
  u8g2.sendBuffer();  // rysujemy całą zawartosc ekranu.  
}

void displayRadioScroller() // Funkcja odpwoiedzialna za przewijanie informacji strem tittle lub stringstation
{
  // KRYTYCZNE: Blokuj gdy SDPlayer aktywny
  if (sdPlayerOLEDActive) return;
  
  // Jesli zmieniła sie dlugosc wyswietlanego stationString to wyczysc ekran OLED w miescach Scrollera
  if (stationStringScroll.length() != stationStringScrollLength) 
  {
    //Serial.print("debug -- czyscimy obszar scrollera stationStringScrollLength - stara wartosc: ");
    //Serial.print(stationStringScrollLength);
    //Serial.print(" <-> nowa wartosc: ");
    //Serial.println(stationStringScroll.length());
    
    stationStringScrollLength = stationStringScroll.length();
    displayClearUnderScroller();
  }

  if (displayMode == 0) // Tryb normalny - Mode 0, radio + VUmeter
  {
    if (stationStringScroll.length() > maxStationVisibleStringScrollLength) //42 + 4 znaki spacji separatora. Realnie widzimy 42 znaki
    {    
      xPositionStationString = offset;
      u8g2.setFont(spleen6x12PL);
      u8g2.setDrawColor(1);
      do {
        u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode0, stationStringScroll.c_str());
        xPositionStationString = xPositionStationString + stationStringScrollWidth;
      } while (xPositionStationString < 256);
      
      offset = offset - 1;
      if (offset < (65535 - stationStringScrollWidth)) { 
        offset = 0;
      }

    } else {
      xPositionStationString = 0;
      u8g2.setDrawColor(1);
      u8g2.setFont(spleen6x12PL);    
      u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode0, stationStringScroll.c_str());
      
    }
  } 
  else if (displayMode == 1)  // Tryb zegara
  {
    
    if (stationStringScroll.length() > maxStationVisibleStringScrollLength) 
    {     
      xPositionStationString = offset;
      u8g2.setFont(spleen6x12PL);
      u8g2.setDrawColor(1);
      do {
        u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode1, stationStringScroll.c_str());

        xPositionStationString = xPositionStationString + stationStringScrollWidth;
      } while (xPositionStationString < 256);
      
      offset = offset - 1;
      if (offset < (65535 - stationStringScrollWidth)) {
        offset = 0;
      }

    } 
    else 
    {
      xPositionStationString = 0;
      u8g2.setDrawColor(1);
      u8g2.setFont(spleen6x12PL);       
      u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode1, stationStringScroll.c_str());
    }

  }
  else if (displayMode == 2)  // Tryb Mode 2, Radio, 3 linijki station tekst
  {

    // Parametry do obługi wyświetlania w 3 kolejnych wierszach z podzialem do pełnych wyrazów
    const int maxLineLength = 41;  // Maksymalna długość jednej linii w znakach
    String currentLine = "";       // Bieżąca linia
    int yPosition = yPositionDisplayScrollerMode2; // Początkowa pozycja Y


    // Podziel tekst na wyrazy
    String word;
    int wordStart = 0;

    for (int i = 0; i <= stationStringScroll.length(); i++)
    {
      // Sprawdź, czy dotarliśmy do końca słowa lub do końca tekstu
      if (i == stationStringScroll.length() || stationStringScroll.charAt(i) == ' ')
      {
        // Pobierz słowo
        String word = stationStringScroll.substring(wordStart, i);
        wordStart = i + 1;

        // Sprawdź, czy dodanie słowa do bieżącej linii nie przekroczy maxLineLength
        if (currentLine.length() + word.length() <= maxLineLength)
        {
          // Dodaj słowo do bieżącej linii
          if (currentLine.length() > 0)
          {
            currentLine += " ";  // Dodaj spację między słowami
          }
          currentLine += word;
        }
        else
        {
          // Jeśli słowo nie pasuje, wyświetl bieżącą linię i przejdź do nowej linii
          u8g2.setFont(spleen6x12PL);
          u8g2.drawStr(0, yPosition, currentLine.c_str());
          yPosition += 12;  // Przesunięcie w dół dla kolejnej linii
          // Zresetuj bieżącą linię i dodaj nowe słowo
          currentLine = word;
        }
      }
    }
    // Wyświetl ostatnią linię, jeśli coś zostało
    if (currentLine.length() > 0)
    {
      u8g2.setFont(spleen6x12PL);
      u8g2.drawStr(0, yPosition, currentLine.c_str());
    }
  }
  else if (displayMode == 3)//|| displayMode == 8
  {
   if (stationStringScroll.length() > maxStationVisibleStringScrollLength) //42 + 4 znaki spacji separatora. Realnie widzimy 42 znaki
    {    
      xPositionStationString = offset;
      u8g2.setFont(spleen6x12PL);
      u8g2.setDrawColor(1);
      do {
        u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode3, stationStringScroll.c_str());
        xPositionStationString = xPositionStationString + stationStringScrollWidth;
      } while (xPositionStationString < 256);
      
      offset = offset - 1;
      if (offset < (65535 - stationStringScrollWidth)) { 
        offset = 0;
      }
    } else {
      xPositionStationString = 0;
      //xPositionStationString = u8g2.getStrWidth(stationStringScroll.c_str());
      xPositionStationString = (SCREEN_WIDTH - stationStringScrollWidth) / 2;
           
      u8g2.setDrawColor(1);
      u8g2.setFont(spleen6x12PL);    
      u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode3, stationStringScroll.c_str()); 
    } 
  }
  
  // WYŁĄCZONE dla displayMode 8, 6-10: vuMeterMode8/6/10 rysują własną nazwę stacji u góry
  // Scrolling title tu nadpisywałby ich ekran - nie jest potrzebny
  /*
  else if (displayMode == 8)//|| displayMode == 8
  {
   //if (stationStringScroll.length() > maxStationVisibleStringScrollLength) //42 + 4 znaki spacji separatora. Realnie widzimy 42 znaki
   if (stationStringScroll.length() > 24) //42 + 4 znaki spacji separatora. Realnie widzimy 42 znaki
    {    
      xPositionStationString = offset;
      u8g2.setFont(spleen6x12PL);
      u8g2.setDrawColor(1);
      do {
        u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode8, stationStringScroll.c_str());
        xPositionStationString = xPositionStationString + stationStringScrollWidth;
      } while (xPositionStationString < 128);
      
      offset = offset - 1;
      if (offset < (65535 - stationStringScrollWidth)) { 
        offset = 0;
      }
    } else {
      xPositionStationString = 0;
      //xPositionStationString = u8g2.getStrWidth(stationStringScroll.c_str());
      //xPositionStationString = (SCREEN_WIDTH - stationStringScrollWidth) / 2;
      
      xPositionStationString = (128 - stationStringScrollWidth) / 2;
           
      u8g2.setDrawColor(1);
      u8g2.setFont(spleen6x12PL);    
      u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode8, stationStringScroll.c_str()); 
    } 
  }
  // Style 6-10: Analizatory spectrum - scrolling tekstu stacji
  else if (displayMode >= 6 && displayMode <= 10)
  {
   if (stationStringScroll.length() > 24)
    {    
      xPositionStationString = offset;
      u8g2.setFont(spleen6x12PL);
      u8g2.setDrawColor(1);
      do {
        u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode8, stationStringScroll.c_str());
        xPositionStationString = xPositionStationString + stationStringScrollWidth;
      } while (xPositionStationString < 128);
      
      offset = offset - 1;
      if (offset < (65535 - stationStringScrollWidth)) { 
        offset = 0;
      }
    } else {
      xPositionStationString = (128 - stationStringScrollWidth) / 2;
      u8g2.setDrawColor(1);
      u8g2.setFont(spleen6x12PL);    
      u8g2.drawStr(xPositionStationString, yPositionDisplayScrollerMode8, stationStringScroll.c_str()); 
    } 
  }
  */
  
}

void handleKeyboard()
{
  uint8_t key = 17;
  keyboardValue = 0;

  // --- Dummy read (dla stabilnosci ESP32 ADC)
  analogRead(keyboardPin);
  delayMicroseconds(5);

  for (int i = 0; i < 32; i++)
  {
    keyboardValue = keyboardValue + analogRead(keyboardPin);
  }
  //keyboardValue = keyboardValue / 32;  
  keyboardValue >>= 5;


  if (debugKeyboard == 1) 
  { 
    displayActive = true;
    displayStartTime = millis();
    u8g2.clearBuffer();
    u8g2.setCursor(10,10); u8g2.print("ADC:   " + String(keyboardValue));
    u8g2.setCursor(10,23); u8g2.print("BUTTON:" + String(keyboardButtonPressed));
    u8g2.sendBuffer(); 
    Serial.print("debug - ADC odczyt: ");
    Serial.print(keyboardValue);
    Serial.print(" flaga przycisk wcisniety:");
    Serial.println(keyboardButtonPressed);
  }
  // Kasujemy flage nacisnietego przycisku tylko jesli nic nie jest wcisnięte
  if (keyboardValue > keyboardButtonNeutral-keyboardButtonThresholdTolerance)
  {  
    keyboardButtonPressed = false;
  }
  // Jesli nic nie jest wcisniete i nic nie było nacisnięte analizujemy przycisk
  if (keyboardValue < keyboardButtonNeutral-keyboardButtonThresholdTolerance)
  {  
    if (keyboardButtonPressed == false)
    {
      // Sprawdzamy stan klawiatury
      if ((keyboardValue <= keyboardButtonThreshold_1 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_1 - keyboardButtonThresholdTolerance ))
      { key=1;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_2 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_2 - keyboardButtonThresholdTolerance ))
      { key=2;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_3 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_3 - keyboardButtonThresholdTolerance ))
      { key=3;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_4 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_4 - keyboardButtonThresholdTolerance ))
      { key=4;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_5 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_5 - keyboardButtonThresholdTolerance ))
      { key=5;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_6 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_6 - keyboardButtonThresholdTolerance ))
      { key=6;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_7 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_7 - keyboardButtonThresholdTolerance ))
      { key=7;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_8 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_8 - keyboardButtonThresholdTolerance ))
      { key=8;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_9 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_9 - keyboardButtonThresholdTolerance ))
      { key=9;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_0 + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_0 - keyboardButtonThresholdTolerance ))
      { key=0;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_Memory + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_Memory - keyboardButtonThresholdTolerance ))
      { key=11;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_Shift + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_Shift - keyboardButtonThresholdTolerance ))
      { key=12;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_Auto + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_Auto - keyboardButtonThresholdTolerance ))
      { key=13;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_Band + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_Band - keyboardButtonThresholdTolerance ))
      { key=14;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_Mute + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_Mute - keyboardButtonThresholdTolerance ))
      { key=15;keyboardButtonPressed = true;}

      if ((keyboardValue <= keyboardButtonThreshold_Scan + keyboardButtonThresholdTolerance ) && (keyboardValue >= keyboardButtonThreshold_Scan - keyboardButtonThresholdTolerance ))
      { key=16;keyboardButtonPressed = true;}

      if ((key < 17) && (keyboardButtonPressed == true))
      {
        Serial.print("ADC odczyt poprawny: ");
        Serial.print(keyboardValue);
        Serial.print(" przycisk: ");
        Serial.println(key);
        keyboardValue = 0;
     
        if ((debugKeyboard == false) && (key < 10)) // Dla przyciskoq 0-9 wykonujemy akcje jak na pilocie
        { rcInputKey(key);}
        else if ((debugKeyboard == false) && (key == 11)) // Przycisk Memory wywyłuje menu wyboru Banku
        { bankMenuDisplay();}
        else if ((debugKeyboard == false) && (key == 12)) // Przycisk Shift zatwierdza zmiane stacji, banku, działa jak "OK" na pilocie
        { 
         ir_code = rcCmdOk; // Przycisk Auto TUning udaje OK
          bit_count = 32;
          calcNec();  // przeliczamy kod pilota na kod oryginalny pełen kod NEC
        }
        else if ((debugKeyboard == false) && (key == 13)) // Przycisk Auto - przełaczanie tryb Zegar/Radio
        { 
          ir_code = rcCmdSrc; // Przycisk Auto TUning udaje Src
          bit_count = 32;
          calcNec();  // przeliczamy kod pilota na kod oryginalny pełen kod NEC
        }
        else if ((debugKeyboard == false) && (key == 14)) // Przycisk Band udaje Back
        {  
          ir_code = rcCmdBack; // Udajemy komendy pilota
          bit_count = 32;
          calcNec();  // przeliczamy kod pilota na kod oryginalny pełen kod NEC
        }
        else if ((debugKeyboard == false) && (key == 15)) // Przycisk Mute
        { 
          ir_code = rcCmdMute; // Przypisujemy kod polecenia z pilota
          bit_count = 32; // ustawiamy informacje, ze mamy pelen kod NEC do analizy 
          calcNec();  // przeliczamy kod pilota na kod oryginalny pełen kod NEC     
        }
        else if ((debugKeyboard == false) && (key == 16)) // Przycisk Memory Scan - zmiana janości wyswietlacza - funkcja "Dimmer"
        { 
          ir_code = rcCmdDirect; // Udajemy komendy pilota
          bit_count = 32;
          calcNec();  // przeliczamy kod pilota na kod oryginalny pełen kod NEC
        }


        key=17; // Reset "KEY" do pozycji poza zakresem klawiatury
      }
    }
  } 
  //  }
  // vTaskDelete(NULL);
}

// ---- WYSWIETLANIE VOLUME (bez zmiany wartości) ----
void volumeDisplay()
{
  displayStartTime = millis();
  timeDisplay = false;
  displayActive = true;
  volumeSet = true;
  //volumeMute = false;  

  Serial.print("debug volumeDisplay -> Wartość głośności: ");
  Serial.println(volumeValue);
  //audio.setVolume(volumeValue);  // zakres 0...21
  String volumeValueStr = String(volumeValue);  // Zamiana liczby VOLUME na ciąg znaków
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tf);
  u8g2.drawStr(65, 33, "VOLUME");
  u8g2.drawStr(163, 33, volumeValueStr.c_str());

  u8g2.drawRFrame(21, 42, 214, 14, 3);                                                   // Rysujmey ramke dla progress bara głosnosci
  if (maxVolume == 42 && volumeValue > 0) { u8g2.drawRBox(23, 44, volumeValue * 5, 10, 2);}  // Progress bar głosnosci
  if (maxVolume == 21 && volumeValue > 0) { u8g2.drawRBox(23, 44, volumeValue * 10, 10, 2);} // Progress bar głosnosci
  u8g2.sendBuffer();
  
  //wsVolumeChange(volumeValue); // Wyślij aktualizację przez WebSocket na strone WWW
  wsVolumeChange(); // Wyślij aktualizację przez WebSocket na strone WWW
  }


// ---- GŁOSNIEJ +1 ----
void volumeUp()
{
  volumeSet = true;
  timeDisplay = false;
  displayActive = true;
  volumeMute = false;
  displayStartTime = millis();   
  volumeValue++;

  if (volumeValue > maxVolume) {volumeValue = maxVolume;}
  audio.setVolume(volumeValue);  // zakres 0...21 lub 0...42
  volumeDisplay();
}

// ---- CISZEJ -1 ----
void volumeDown()
{
  volumeSet = true;
  timeDisplay = false;
  displayActive = true;
  displayStartTime = millis();
  volumeMute = false;
 
  volumeValue--;
  if (volumeValue < 1) {volumeValue = 1;}
  audio.setVolume(volumeValue);  // zakres 0...21 lub 0...42
  volumeDisplay();
}

// ---- KASOWANIE WSZYSTKICH FLAG ---- 
//Kasuje wszystkie flagi przebywania w menu, funkcjach itd. Pozwala pwrócic do wyswietlania ekranu głownego radia
void clearFlags()  
{
  timeDisplay = true;

  debugKeyboard = false;
  displayActive = false;
  listedStations = false;
  listedTracks = false;  // Wyłącz listę utworów SDPlayer
  volumeSet = false;
  bankMenuEnable = false;
  bankNetworkUpdate = false;
  equalizerMenuEnable = false;
  rcInputDigitsMenuEnable = false;
  f_voiceTimeBlocked = false;
  if (f_displaySleepTime && f_sleepTimerOn) {f_displaySleepTimeSet = true;}
  f_displaySleepTime = false;

  rcInputDigit1 = 0xFF; // czyscimy cyfre 1, flaga pustej zmiennej to FF
  rcInputDigit2 = 0xFF; // czyscimy cyfre 2, flaga pustej zmiennej to FF
  station_nr = stationFromBuffer; // Przywracamy numer stacji z bufora
  bank_nr = previous_bank_nr;     // Przywracamy numer banku z bufora
}


void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) 
{
  if (type == WS_EVT_CONNECT) 
  {
    Serial.println("debug Web -> WebSocket klient podlaczony");

    client->text("station:" + String(station_nr));
    client->text("stationname:" + stationName.substring(0, stationNameLenghtCut));
    client->text("volume:" + String(volumeValue)); 
    client->text("mute:" + String(volumeMute));
    client->text("bank:" + String(bank_nr));
    client->text("wifi:" + String(wifiSignalLevel)); // wysyła aktualny poziom sygnalu WiFi

    if (audio.isRunning() == true)
    {
     sanitizeUtf8(stationStringWeb);
     client->text("stationtext$" + stationStringWeb);
    }
    else
    {
     client->text("stationtext$... No audio stream ! ...");
    }
    
    client->text("bitrate:" + String(bitrateString)); 
    client->text("samplerate:" + String(sampleRateString)); 
    client->text("bitpersample:" + String(bitsPerSampleString)); 

  
  if (mp3 == true) {client->text("streamformat:MP3"); }
  if (flac == true) {client->text("streamformat:FLAC"); }
  if (aac == true) {client->text("streamformat:AAC"); }
  if (vorbis == true) {client->text("streamformat:VORBIS"); }
  if (opus == true) {client->text("streamformat:OPUS"); }

  } 
  else if (type == WS_EVT_DATA) 
  {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) 
    {
      String msg = "";
      for (size_t i = 0; i < len; i++) 
      {
        msg += (char) data[i];
      }

      Serial.println("debug Web -> Odebrano WS: " + msg);

      if (msg.startsWith("volume:")) 
      {
        int newVolume = msg.substring(7).toInt();
        volumeValue = newVolume;
        volumeMute = false;
        audio.setVolume(volumeValue);  // zakres 0...21 lub 0...42
        volumeDisplay();   // wyswietle wartosci Volume na wyswietlaczu OLED
      }
      else if (msg.startsWith("station:")) 
      {
        int newStation = msg.substring(8).toInt();
        station_nr = newStation;
        bank_nr = previous_bank_nr;
        urlPlaying = false;

        ir_code = rcCmdOk; // Przypisujemy kod polecenia z pilota
        bit_count = 32; // ustawiamy informacje, ze mamy pelen kod NEC do analizy 
        calcNec();  // przeliczamy kod pilota na kod oryginalny pełen kod NEC    
      }
      else if (msg.startsWith("bank:")) 
      {
        int newBank = msg.substring(5).toInt();
        bank_nr = newBank;
        
        //bankMenuEnable = true;        
        //displayStartTime = millis();
        //timeDisplay = false;
        
        bankMenuDisplay();
        fetchStationsFromServer();
        clearFlags();
        urlPlaying = false;

        station_nr = 1;
        
        //Zatwirdzenie zmiany stacji
        ir_code = rcCmdOk; // Przypisujemy kod polecenia z pilota
        bit_count = 32; // ustawiamy informacje, ze mamy pelen kod NEC do analizy 
        calcNec();  // przeliczamy kod pilota na kod oryginalny pełen kod NEC    
        
      }
      else if (msg.startsWith("displayMode")) 
      {
        ir_code = rcCmdSrc; // Udajemy kod pilota SRC - zmiana trybu wyswietlacza 
        bit_count = 32;
        calcNec();          // Przeliczamy kod pilota na pełny oryginalny kod NEC
      }

    }
  }

}

void bufforAudioInfo()
{
  int bitRateBps = audio.getBitRate() / 1000;
  int bytesPerSecond = 0;

  if (bitRateBps > 0) { bytesPerSecond = bitRateBps / 8;}   // kB/s

  int inputBufferBytes = audio.inBufferFilled() / 1000;

  // zabezpieczenie przed dzieleniem przez zero i innymi błędami
  if (bytesPerSecond > 0 && inputBufferBytes > 8) 
  {
    audioBufferTime = inputBufferBytes / bytesPerSecond;
  } 
  else 
  {
    audioBufferTime = 0;
  }

  
  
  Serial.print("debug-- Audio Bufor wolne: ");
  Serial.print(audio.inBufferFree());
  Serial.print(" / ");
  Serial.print("zajęte: ");
  Serial.print(audio.inBufferFilled());
  
  Serial.print("  Muzyka w buforze na: " + String(audioBufferTime) + " sek. " );
  Serial.print("  bitrate: " );
  Serial.print(audio.getBitRate());
  Serial.print("  " );
  /*
  if (audioBufferTime >= 10) {Serial.println("##########");}
  if (audioBufferTime >= 9) {Serial.println("#########_");}
  if (audioBufferTime >= 8) {Serial.println("########__");}
  if (audioBufferTime >= 7) {Serial.println("#######___");}
  if (audioBufferTime >= 6) {Serial.println("######____");}
  if (audioBufferTime >= 5) {Serial.println("#####_____");}
  if (audioBufferTime == 4) {Serial.println("####______");}
  if (audioBufferTime == 3) {Serial.println("###_______");}
  if (audioBufferTime == 2) {Serial.println("##________");}
  if (audioBufferTime == 1) {Serial.println("#_________");}
  if (audioBufferTime == 0) {Serial.println("__________");}
  */
  
  if (audioBufferTime < 0)  audioBufferTime = 0;
  if (audioBufferTime > 10) audioBufferTime = 10;

  /*
  char bar[11];
  for (int i = 0; i < 10; i++) {bar[i] = (i < audioBufferTime) ? '#' : '_'; }
  bar[10] = '\0';
  Serial.println(bar);
  */
  
  Serial.print("[");
  for (int i = 0; i < 10; i++) {if (i < audioBufferTime) Serial.print('#'); else Serial.print('_'); }
  Serial.println("]");



}

// Funkcja obsługująca przerwanie (reakcja na zmianę stanu pinu)
void IRAM_ATTR pulseISR()
{
  if (digitalRead(recv_pin) == HIGH)
  {
    pulse_start_high = micros();  // Zapis początku impulsu
  }
  else
  {
    pulse_end_high = micros();    // Zapis końca impulsu
    pulse_ready = true;
  }

  if (digitalRead(recv_pin) == LOW)
  {
    pulse_start_low = micros();  // Zapis początku impulsu
  }
  else
  {
    pulse_end_low = micros();    // Zapis końca impulsu
    pulse_ready_low = true;
  }

   // ----------- ANALIZA PULSOW -----------------------------
  if (pulse_ready_low) // spradzamy czy jest stan niski przez 9ms - start ramki
  {
    pulse_duration_low = pulse_end_low - pulse_start_low;
  
    if (pulse_duration_low > (LEAD_HIGH - TOLERANCE) && pulse_duration_low < (LEAD_HIGH + TOLERANCE))
    {
      pulse_duration_9ms = pulse_duration_low; // przypisz czas trwania puslu Low do zmiennej puls 9ms
      pulse_ready9ms = true; // flaga poprawnego wykrycia pulsu 9ms w granicach tolerancji
    }

  }

  // Sprawdzenie, czy impuls jest gotowy do analizy
  if ((pulse_ready== true) && (pulse_ready9ms = true))
  {
    pulse_ready = false;
    pulse_ready9ms = false; // kasujemy flage wykrycia pulsu 9ms

    // Obliczenie czasu trwania impulsu
    pulse_duration = pulse_end_high - pulse_start_high;
    //Serial.println(pulse_duration); odczyt dlugosci pulsow z pilota - debug
    if (!data_start_detected)
    {
    
      // Oczekiwanie na sygnał 4,5 ms wysoki
      if (pulse_duration > (LEAD_LOW - TOLERANCE) && pulse_duration < (LEAD_LOW + TOLERANCE))
      {
        pulse_duration_4_5ms = pulse_duration;
        // Początek sygnału: 4,5 ms wysoki
        
        data_start_detected = true;  // Ustawienie flagi po wykryciu sygnału wstępnego
        bit_count = 0;               // Reset bit_count przed odebraniem danych
        ir_code = 0;                 // Reset kodu IR przed odebraniem danych
      }
    }
    else
    {
      // Sygnały dla bajtów (adresu ADDR, IADDR, komendy CMD, ICMD) zaczynają się po wstępnym sygnale
      if (pulse_duration > (HIGH_THRESHOLD - TOLERANCE) && pulse_duration < (HIGH_THRESHOLD + TOLERANCE))
      {
        ir_code = (ir_code << 1) | 1;  // Dodanie "1" do kodu IR
        //bit_count++;
        bit_count = bit_count + 1;
        pulse_duration_1690us = pulse_duration;
       

      }
      else if (pulse_duration > (LOW_THRESHOLD - TOLERANCE) && pulse_duration < (LOW_THRESHOLD + TOLERANCE))
      {
        ir_code = (ir_code << 1) | 0;  // Dodanie "0" do kodu IR
        //bit_count++;
        bit_count = bit_count + 1;
        pulse_duration_560us = pulse_duration;
       
      }

      // Sprawdzenie, czy otrzymano pełny 32-bitowy kod IR
      if (bit_count == 32)
      {
        // Symulacja aktywnosci LED IR
        #ifdef IR_LED
        digitalWrite(IR_LED, HIGH);
        //f_statusLED = 1; 
        #endif
        
        // Rozbicie kodu na 4 bajty
        uint8_t ADDR = (ir_code >> 24) & 0xFF;  // Pierwszy bajt
        uint8_t IADDR = (ir_code >> 16) & 0xFF; // Drugi bajt (inwersja adresu)
        uint8_t CMD = (ir_code >> 8) & 0xFF;    // Trzeci bajt (komenda)
        uint8_t ICMD = ir_code & 0xFF;          // Czwarty bajt (inwersja komendy)

        // Sprawdzenie poprawności (inwersja) bajtów adresu i komendy
        if ((ADDR ^ IADDR) == 0xFF && (CMD ^ ICMD) == 0xFF)
        {
          data_start_detected = false;
          //bit_count = 0;
        }
        else
        {
          ir_code = 0; 
          data_start_detected = false;
          //bit_count = 0;        
        }

      }
		  
    }
  }
 //runTime2 = esp_timer_get_time();
}

void displayEqualizer() // Funkcja rysująca menu equalizera (3-band lub 16-band)
{
  displayStartTime = millis();  // Uaktulniamy czas dla funkcji auto-pwrotu z menu
  equalizerMenuEnable = true;   // Ustawiamy flage menu equalizera
  timeDisplay = false;          // Wyłaczamy zegar
  displayActive = true;         // Wyswietlacz aktywny
  
  #if defined(ENABLE_EQ16) && (ENABLE_EQ16 == 1)
  // ========== WYBÓR TRYBU EQUALIZERA (3-BAND vs 16-BAND) ==========
  if (eq16ModeSelectActive) {
    // Ekran wyboru trybu EQ
    APMS_EQ16::drawModeSelect(u8g2, eq16Mode ? 1 : 0);
    return;
  }
  
  // ========== TRYB 16-BAND ==========
  if (eq16Mode) {
    // Załaduj aktualne wartości do bufora
    APMS_EQ16::getAll(eq16Gains);
    // Rysuj edytor 16-band
    APMS_EQ16::drawEditor(u8g2, eq16Gains, eq16SelectedBand, false);
    return;
  }
  #endif
  
  // ========== TRYB 3-BAND (DOMYŚLNY) ==========
  Serial.println("--Equalizer 3-band--");
  Serial.print("Wartość tonów Niskich/Low:   ");
  Serial.println(toneLowValue);
  Serial.print("Wartość tonów Średnich/Mid:  ");
  Serial.println(toneMidValue);
  Serial.print("Wartość tonów Wysokich/High: ");
  Serial.println(toneHiValue);
  Serial.print("Wartość Balansu: ");
  Serial.println(balanceValue);
    
  audio.setTone(toneLowValue, toneMidValue, toneHiValue); // Zakres regulacji -40 + 6dB jako int8_t ze znakiem
  audio.setBalance(balanceValue);

  u8g2.setDrawColor(1);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tf);
  u8g2.drawStr(60, 14, "EQUALIZER");
  //u8g2.drawStr(1, 14, "EQUALIZER");
  u8g2.setFont(spleen6x12PL);
  //u8g2.setCursor(138,12);
  //u8g2.print("P1    P2    P3    P4");
  uint8_t xTone;
  uint8_t yTone;  
  
  // ---- Tony Wysokie ----
  xTone=0; 
  yTone=28;
  u8g2.setCursor(xTone,yTone);
  if (toneHiValue >= 0) {u8g2.print("High:  " + String(toneHiValue) + "dB");}  // Dla wartosci dodatnich piszemy normalnie
  if ((toneHiValue < 0) && (toneHiValue >= -9)) {u8g2.print("High: " + String(toneHiValue) + "dB");}    // Dla wartosci ujemnych piszemy o 1 znak wczesniej
  if ((toneHiValue < 0) && (toneHiValue < -9)) {u8g2.print("High:" + String(toneHiValue) + "dB");}    // Dla wartosci ujemnych ponizej -9 piszemy o 2 znak wczesniej
  
  xTone=20;
  if (toneSelect == 1) 
  { 
    u8g2.drawRBox(xTone+56,yTone-9,154,9,1); // Rysujemy biały pasek pod regulacją danego tonu  
    u8g2.setDrawColor(0); 
    u8g2.drawLine(xTone+57,yTone-5,xTone+208,yTone-5);
    //u8g2.drawLine(xTone+57,yTone-4,xTone+208,yTone-4);
    if (toneHiValue >= 0){ u8g2.drawRBox((10 * toneHiValue) + xTone + 138,yTone-7,10,5,1);}
    if (toneHiValue < 0) { u8g2.drawRBox((2 * toneHiValue) + xTone + 138,yTone-7,10,5,1);}
    u8g2.setDrawColor(1);
  }
  else 
  {
    u8g2.drawLine(xTone+57,yTone-5,xTone+208,yTone-5);
    //u8g2.drawLine(xTone+57,yTone-4,xTone+208,yTone-4);
    if (toneHiValue >= 0){ u8g2.drawRBox((10 * toneHiValue) + xTone + 138,yTone-7,10,5,1);}
    if (toneHiValue < 0) { u8g2.drawRBox((2 * toneHiValue) + xTone + 138,yTone-7,10,5,1);}
    //u8g2.drawRBox((3 * toneHiValue) + xTone + 178,yTone-7,10,6,1);
  }

  // ---- Tony średnie ----
  xTone=0;
  yTone=40;
  u8g2.setCursor(xTone,yTone);
  if (toneMidValue >= 0) {u8g2.print("Mid:   " + String(toneMidValue) + "dB");}  // Dla wartosci dodatnich piszemy normalnie
  if ((toneMidValue < 0) && (toneMidValue >= -9)) {u8g2.print("Mid:  " + String(toneMidValue) + "dB");}
  if ((toneMidValue < 0) && (toneMidValue < -9)) {u8g2.print("Mid: " + String(toneMidValue) + "dB");} 
  
  xTone=20;
  if (toneSelect == 2) 
  { 
    u8g2.drawRBox(xTone+56,yTone-9,154,9,1);
    u8g2.setDrawColor(0);
    u8g2.drawLine(xTone+57,yTone-5,xTone + 208,yTone-5);
    //u8g2.drawLine(xTone+57,yTone-4,xTone + 208,yTone-4);
    if (toneMidValue >= 0) { u8g2.drawRBox((10 * toneMidValue) + xTone + 138,yTone-7,10,5,1);}
    if (toneMidValue < 0) { u8g2.drawRBox((2 * toneMidValue) + xTone + 138,yTone-7,10,5,1);}
    u8g2.setDrawColor(1);
  }
  else
  {
    u8g2.drawLine(xTone+57,yTone-5,xTone + 208,yTone-5);
    //u8g2.drawLine(xTone+57,yTone-4,xTone + 208,yTone-4);
    if (toneMidValue >= 0) { u8g2.drawRBox((10 * toneMidValue) + xTone + 138,yTone-7,10,5,1);}
    if (toneMidValue < 0)  { u8g2.drawRBox((2 * toneMidValue) + xTone + 138,yTone-7,10,5,1);}
    //u8g2.drawRBox((3 * toneMidValue) + xTone + 138,yTone-7,10,6,1);
  }


  // Tony niskie
  xTone=0; 
  yTone=52;
  u8g2.setCursor(xTone,yTone);
  if (toneLowValue >= 0) { u8g2.print("Low:   " + String(toneLowValue) + "dB");}
  if ((toneLowValue < 0) && (toneLowValue >= -9)) { u8g2.print("Low:  " + String(toneLowValue) + "dB");}
  if ((toneLowValue < 0) && (toneLowValue < -9)) { u8g2.print("Low: " + String(toneLowValue) + "dB");}
  xTone=20;
  if (toneSelect == 3) 
  { 
    u8g2.drawRBox(xTone+56,yTone-9,154,9,1);
    u8g2.setDrawColor(0);
    u8g2.drawLine(xTone + 57,yTone-5,xTone + 208,yTone-5);
    if ( toneLowValue >= 0 ) { u8g2.drawRBox((6 * toneLowValue) + xTone + 128,yTone-7,10,5,1);}
    if ( toneLowValue < 0 )  { u8g2.drawRBox((6 * toneLowValue) + xTone + 128,yTone-7,10,5,1);}
    u8g2.setDrawColor(1);
  }
  else
  {
    u8g2.drawLine(xTone + 57,yTone-5,xTone + 208,yTone-5);
    if ( toneLowValue >= 0 ) { u8g2.drawRBox((6 * toneLowValue) + xTone + 128,yTone-7,10,5,1);}
    if ( toneLowValue < 0 )  { u8g2.drawRBox((6 * toneLowValue) + xTone + 128,yTone-7,10,5,1);}
  }

  // Balans L/R
  xTone=0; 
  yTone=64;
  u8g2.setCursor(xTone,yTone);
  if (balanceValue >= 0) { u8g2.print("Bal:   " + String(balanceValue) + "dB");}
  if ((balanceValue < 0) && (balanceValue >= -9)) { u8g2.print("Bal:  " + String(balanceValue) + "dB");}
  if ((balanceValue < 0) && (balanceValue < -9)) { u8g2.print("Bal: " + String(balanceValue) + "dB");}
  xTone=20;
  if (toneSelect == 4) 
  { 
    u8g2.drawRBox(xTone+56,yTone-9,154,9,1);
    u8g2.setDrawColor(0);
    u8g2.drawLine(xTone + 64,yTone-5,xTone + 201,yTone-5);
    u8g2.setFont(u8g2_font_04b_03_tr);
    u8g2.drawStr(xTone + 58, yTone-2, "L");
    u8g2.drawStr(xTone + 204, yTone-2, "R");
    if ( balanceValue >= 0 ) { u8g2.drawRBox((4 * balanceValue) + xTone + 128,yTone-7,10,5,1);}
    if ( balanceValue < 0 )  { u8g2.drawRBox((4 * balanceValue) + xTone + 128,yTone-7,10,5,1);}
    u8g2.setDrawColor(1);
  }
  else
  {
    u8g2.drawLine(xTone + 64,yTone-5,xTone + 201,yTone-5);
    u8g2.setFont(u8g2_font_04b_03_tr);
    u8g2.drawStr(xTone + 58, yTone -2, "L");
    u8g2.drawStr(xTone + 204, yTone-2, "R");
    if ( balanceValue >= 0 ) { u8g2.drawRBox((4 * balanceValue) + xTone + 128,yTone-7,10,5,1);}
    if ( balanceValue < 0 )  { u8g2.drawRBox((4 * balanceValue) + xTone + 128,yTone-7,10,5,1);}
  }
  u8g2.sendBuffer(); 
}


void displayBasicInfo()
{
  displayStartTime = millis();  // Uaktulniamy czas dla funkcji auto-powrotu z menu
  timeDisplay = false;          // Wyłaczamy zegar
  displayActive = true;         // Wyswietlacz aktywny
  
  uint64_t chipid = ESP.getEfuseMac();

  char buf[100];
  snprintf(buf, sizeof(buf),
          "ESP32 SN:%04X%08X,  FW Ver.: %s",
          (uint16_t)(chipid >> 32),
          (uint32_t)chipid,
          softwareRev);

String chipSN = buf;
  u8g2.clearBuffer();
  u8g2.setFont(spleen6x12PL);
  u8g2.drawStr(0, 10, "Info:");
  //u8g2.setCursor(0,25); u8g2.print("ESP32 SN:" + String(ESP.getEfuseMac()) + ",  FW Ver.:" + String(softwareRev));
  u8g2.setCursor(0,25); u8g2.print(chipSN);
  u8g2.setCursor(0,38); u8g2.print("Hostname:" + String(hostname) + ",  WiFi Signal:" + String(WiFi.RSSI()) + "dBm");
  u8g2.setCursor(0,51); u8g2.print("WiFi SSID:" + String(wifiManager.getWiFiSSID()));
  u8g2.setCursor(0,64); u8g2.print("IP:" + currentIP + "  MAC:" + String(WiFi.macAddress()) );
  
  u8g2.sendBuffer();
}
/*
void audioProcessing(void *p)
{
  while (true) {
  audio.loop();
  vTaskDelay(1 / portTICK_PERIOD_MS); // Opóźnienie 1 milisekundy
  }
}
*/


//  OTA update callback dla Wifi Managera i trybu Recovery Mode
void handlePreOtaUpdateCallback()
{
  Update.onProgress([](unsigned int progress, unsigned int total) 
  {
    u8g2.setCursor(1,56); u8g2.printf("Update: %u%%\r", (progress / (total / 100)) );
    u8g2.setCursor(80,56); u8g2.print(String(progress) + "/" + String(total));
    u8g2.sendBuffer();
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
}

void recoveryModeCheck()
{
  unsigned long lastTurnTime = 0;

  if (digitalRead(SW_PIN2) == 0)
  {
    int8_t recoveryMode = 0;
    u8g2.clearBuffer();
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(1,14, "RECOVERY / RESET MODE - release encoder");
    u8g2.sendBuffer();
    delay(2000);

    while (digitalRead(SW_PIN2) == 0) {delay(10);}  
    u8g2.drawStr(1,14, "Please Wait...                         ");
    u8g2.sendBuffer();
    delay(1000);
    u8g2.clearBuffer();
     
    prev_CLK_state2 = digitalRead(CLK_PIN2);
    
    while (true)
    {
      if (millis() - lastTurnTime > 50)
      {  
        CLK_state2 = digitalRead(CLK_PIN2);
        //if (CLK_state2 != prev_CLK_state2 && CLK_state2 == HIGH)  // Sprawdzenie, czy stan CLK zmienił się na wysoki
        if (CLK_state2 != prev_CLK_state2 && CLK_state2 == LOW)  // Sprawdzenie, czy stan CLK zmienił się na wysoki
        //if (CLK_state2 != prev_CLK_state2)
        {
          //if (digitalRead(DT_PIN2) == HIGH)
          if (digitalRead(DT_PIN2) != CLK_state2)
          { recoveryMode++; }
          else
          { recoveryMode--; }

          if (recoveryMode > 1) {recoveryMode =1;}
          if (recoveryMode < 0) {recoveryMode =0;}
          lastTurnTime = millis();
        }
        prev_CLK_state2 = CLK_state2;
      }

      u8g2.drawStr(1,14, "[ -- Rotate Enckoder -- ]            ");

      if (recoveryMode == 0)
      {
        u8g2.drawStr(1,28, ">> RESET BANK=1, STATION=1 <<");
        u8g2.drawStr(1,42, "   RESET WIFI SSID, PASSWD   ");  
      }
      else if (recoveryMode == 1)
      {
        u8g2.drawStr(1,28, "   RESET BANK=1, STATION=1   ");
        u8g2.drawStr(1,42, ">> RESET WIFI SSID, PASSWD <<");      
      }
      u8g2.sendBuffer();
      
      if (digitalRead(SW_PIN2) == 0)
      {
        if (recoveryMode == 0)
        {
          bank_nr = 1;
          station_nr = 1;
          saveStationOnSD();
          u8g2.clearBuffer(); 
          u8g2.drawStr(1,14, "SET BANK=1, STATION=1         ");
          u8g2.drawStr(1,28, "ESP will RESET in 3sec.       ");
          u8g2.sendBuffer();
          delay(3000);
          ESP.restart();
        }  
        else if (recoveryMode == 1)
        {
          u8g2.clearBuffer();
          u8g2.drawStr(1,14, "WIFI SSID, PASSWD CLEARED   ");
          u8g2.drawStr(1,28, "ESP will RESET in 3sec.     ");
          u8g2.sendBuffer();
          wifiManager.resetSettings();
          delay(3000);
          ESP.restart();
        }
      }
      
      delay(20);
    }   
  } 
}


void displayDimmerTimer()
{
  displayDimmerTimeCounter++;
  
  if ((displayActive == true) ) //&& (displayDimmerActive == true) Jezeli wysweitlacz aktywny i jest przyciemniony
  { 
    displayDimmerTimeCounter = 0;
    //Serial.println("debug displayDimmerTimer -> displayDimmerTimeCounter = 0");
    if (displayDimmerActive == true) {displayDimmer(0);}
  }
  // Jesli upłynie czas po którym mamy przyciemnic wyswietlacz i funkcja przyciemniania włączona oraz nie jestemswy w funkcji aktulizacji OTA to
  if ((displayDimmerTimeCounter >= displayAutoDimmerTime) && (displayAutoDimmerOn == true) && (fwupd == false)) 
  {
    if (displayDimmerActive == false) // jesli wyswietlacz nie jest jeszcze przyciemniony
    {
      displayDimmer(1); // wywolujemy funkcje przyciemnienia z parametrem 1 (załacz)
      timer2.detach();
    }
    displayDimmerTimeCounter = 0;
  }

  displayPowerSaveTimeCounter++;
  
  if (displayPowerSaveTimeCounter >= displayPowerSaveTime && displayPowerSaveEnabled && !fwupd) 
  {
    displayPowerSaveTimeCounter = 0;
    displayPowerSave(1);
    timer2.detach();
  }
}

void displayDimmer(bool dimmerON)
{
  displayDimmerActive = dimmerON;
  Serial.print("debug displayDimmer -> displayDimmerActive: ");
  Serial.println(displayDimmerActive);
  
  if ((dimmerON == 1) && (displayActive == false) && (fwupd == false)) { u8g2.setContrast(dimmerDisplayBrightness);}
  if (dimmerON == 0) 
  { 
    displayPowerSave(0);
    u8g2.setContrast(displayBrightness); 
    displayDimmerTimeCounter = 0;
    displayPowerSaveTimeCounter = 0;
    timer2.attach(1, displayDimmerTimer);
  }
}

#ifdef twoEncoders
  void handleEncoder1()
  {
    CLK_state1 = digitalRead(CLK_PIN1);                       // Odczytanie aktualnego stanu pinu CLK enkodera 1
    if (CLK_state1 != prev_CLK_state1 && CLK_state1 == HIGH)  // Sprawdzenie, czy stan CLK zmienił się na wysoki
    {
      timeDisplay = false;
      displayActive = true;
      displayStartTime = millis();

      if (bankMenuEnable == false)  // Przewijanie listy stacji radiowych
      {
        station_nr = currentSelection + 1;
        if (digitalRead(DT_PIN1) == HIGH) 
        {
          station_nr--;
          //if (listedStations == 1) station_nr--;
          if (station_nr < 1) 
          {
            station_nr = stationsCount;//1;
          }
          Serial.print("debug ENC1 -> Numer stacji do tyłu: ");
          Serial.println(station_nr);
          scrollUp();
        } 
        else 
        {
          station_nr++;
          if (station_nr > stationsCount) 
          {
            station_nr = 1;//stationsCount;
          }
          Serial.print("debug ENC1 -> Numer stacji do przodu: ");
          Serial.println(station_nr);
          scrollDown();
        }
        displayStations();
      } 
      else // Przewijanie listy bankow
      {
        if (bankMenuEnable == true)  // Przewijanie listy banków stacji radiowych
        {
          if (digitalRead(DT_PIN1) == HIGH) 
          {
            bank_nr--;
            if (bank_nr < 1) 
            {
              bank_nr = bank_nr_max;
            }
          } 
          else 
          {
            bank_nr++;
            if (bank_nr > bank_nr_max) 
            {
              bank_nr = 1;
            }
          }
          bankMenuDisplay();
        }
      }
    }
    prev_CLK_state1 = CLK_state1;
    


    if ((button1.isReleased()) && (listedStations == false)) 
    {
      timeDisplay = false;
      volumeSet = false;
      displayActive = true;
      displayStartTime = millis();

      if (bankMenuEnable == true)
      {
        bankMenuEnable = false;
        fetchStationsFromServer();
        changeStation();
        if (!sdPlayerOLEDActive) displayRadio(); // Blokuj gdy SDPlayer aktywny
        clearFlags();
        return;
      }
      else if (bankMenuEnable == false)
      {
        bankMenuEnable = true;  
        bankMenuDisplay();// Po nacisnieciu enkodera1 wyswietlamy menu Banków
        return;
      }
    }
    
    
    /*
    if ((button1.isReleased()) && (listedStations == false) && (bankMenuEnable == true)) 
    {
      bankMenuEnable = false;
      displayStartTime = millis();
      timeDisplay = false;
      volumeSet = false;
      displayActive = true;
      previous_bank_nr = bank_nr;
      station_nr = 1;
      fetchStationsFromServer();
      changeStation();
      displayRadio();
      clearFlags();
      return;
    }
    */

    /*
    if (button1.isPressed() && listedStations == false && bankMenuEnable == false) 
    {
      
      bankMenuEnable = true;
      listedStations = false;
      volumeSet = false;
      timeDisplay = false;
      volumeSet = false;
      displayActive = true;
      
      displayStartTime = millis();
      bankMenuDisplay();// Po nacisnieciu enkodera1 wyswietlamy menu Banków
      return;
      
    }
    //*/
    

    if (button1.isReleased() && listedStations) 
    {
      listedStations = false;
      volumeSet = false;
      changeStation();
      if (!sdPlayerOLEDActive) displayRadio(); // Blokuj gdy SDPlayer aktywny
      //u8g2.sendBuffer();
      clearFlags();
      return;
    }

    

    
  }
#endif


void handleEncoder2StationsVolumeClick()
{
  // Double-click detection dla aktywacji SD Player (enkoder) - używa ezButton
  static int encoderClickCount = 0;
  static unsigned long lastEncoderClickTime = 0;
  
  // Sprawdź double-click tylko gdy SD Player nieaktywny - użyj button2.isReleased()
  if (!sdPlayerOLEDActive && button2.isReleased()) {
    unsigned long now = millis();
    
    // Sprawdź czy to szybkie kliknięcie (w oknie 600ms)
    if (now - lastEncoderClickTime < 600) {
      encoderClickCount++;
      Serial.printf("[Encoder2] Kliknięcie #%d\n", encoderClickCount);
    } else {
      encoderClickCount = 1; // Reset po timeout
      Serial.println("[Encoder2] Pierwsze kliknięcie (reset licznika)");
    }
    lastEncoderClickTime = now;
    
    // Sprawdź czy to double-click
    if (encoderClickCount >= 2) {
      // Double-click wykryty - aktywuj SD Player
      if (g_sdPlayerOLED) {
        // Zapamiętaj aktualny bank i stację przed aktywacją
        sdPlayerReturnBank = bank_nr;
        sdPlayerReturnStation = station_nr;
        Serial.printf("[SDPlayer] DOUBLE-CLICK wykryty! Zapisano pozycję: Bank %d, Stacja %d\n", sdPlayerReturnBank, sdPlayerReturnStation);
        
        g_sdPlayerOLED->activate();
        sdPlayerOLEDActive = true;
        Serial.println("[DOUBLE-CLICK] Aktywacja SD Player - handleEncoder2StationsVolumeClick");
      }
      encoderClickCount = 0; // Reset licznika
      return;
    }
  }
  
  // =============== OBSŁUGA ENKODERA DLA ZINTEGROWANYCH MODUŁÓW ===============
  
  // 1. SDPlayer - obsługa enkodera gdy SDPlayer jest aktywny
  if (g_sdPlayerOLED && g_sdPlayerOLED->isActive()) {
    // Obsługa przytrzymania z różnymi czasami:
    // < 1s      = krótkie kliknięcie (play/pause)
    // 3-5s     = średnie przytrzymanie (select track)
    // > 6s     = długie przytrzymanie (tryb volume lub wyjście)
    static unsigned long buttonPressStartTime = 0;
    static bool wasPressed = false;
    static bool mediumHoldExecuted = false;
    static bool longHoldExecuted = false;
    bool isPressed = (digitalRead(SW_PIN2) == LOW); // Zakładając aktywne LOW
    
    if (isPressed && !wasPressed) {
      // Przycisk został właśnie wciśnięty
      buttonPressStartTime = millis();
      wasPressed = true;
      mediumHoldExecuted = false;
      longHoldExecuted = false;
    } else if (!isPressed && wasPressed) {
      // Przycisk został zwolniony
      wasPressed = false;
      unsigned long pressDuration = millis() - buttonPressStartTime;
      
      // Krótkie kliknięcie (< 3s) - jeśli nie wykonano akcji przytrzymania
      if (pressDuration < 3000 && !mediumHoldExecuted && !longHoldExecuted) {
        g_sdPlayerOLED->onEncoderButton();
      }
    } else if (isPressed && wasPressed) {
      // Przycisk jest ciągle przytrzymany - sprawdzamy czas
      unsigned long pressDuration = millis() - buttonPressStartTime;
      
      // Średnie przytrzymanie (3-5s) - zatwierdź wybór utworu
      if (pressDuration >= 3000 && pressDuration < 6000 && !mediumHoldExecuted) {
        g_sdPlayerOLED->onEncoderButtonMediumHold(pressDuration);
        mediumHoldExecuted = true;
      }
      // Długie przytrzymanie (>6s) - przełącz tryb volume
      else if (pressDuration >= 6000 && !longHoldExecuted) {
        g_sdPlayerOLED->onEncoderButtonLongHold(pressDuration);
        longHoldExecuted = true;
        wasPressed = false; // Reset
        return;
      }
    }
    
    // Obsługa obracania enkodera
    CLK_state2 = digitalRead(CLK_PIN2);
    if (CLK_state2 != prev_CLK_state2 && CLK_state2 == HIGH) {
      if (digitalRead(DT_PIN2) == HIGH) {
        g_sdPlayerOLED->onEncoderLeft();  // Lewo = Góra
      } else {
        g_sdPlayerOLED->onEncoderRight(); // Prawo = Dół
      }
    }
    prev_CLK_state2 = CLK_state2;
    
    return; // Wyjdź z funkcji - SDPlayer obsłużony
  }
  
  // ============== KONIEC OBSŁUGI MODUŁÓW - kontynuacja normalnej obsługi radia ==============
  
  CLK_state2 = digitalRead(CLK_PIN2);                       // Odczytanie aktualnego stanu pinu CLK enkodera 2
  if (CLK_state2 != prev_CLK_state2 && CLK_state2 == HIGH)  // Sprawdzenie, czy stan CLK zmienił się na wysoki
  {
    timeDisplay = false;
    displayActive = true;
    displayStartTime = millis();

    if ((volumeSet == false) && (bankMenuEnable == false))  // Przewijanie listy stacji radiowych
    {
      currentSelection = station_nr - 1;

      if (digitalRead(DT_PIN2) == HIGH) 
      {  
        if (listedStations == true)
        {
          station_nr--;
          if (station_nr < 1) {station_nr = stationsCount;} //1;
          Serial.print("Numer stacji do tyłu: "); Serial.println(station_nr);
          
          scrollUp();
        }
        else
        {
          if (maxSelection() - currentSelection < maxVisibleLines) // 98 - 98 = 0 < 4 = YES
          {
            firstVisibleLine = maxSelection() - maxVisibleLines + 1; // 98 - 4- 1 -> 93, za daleko, musi byc 95
          } 
          else 
          {
            firstVisibleLine = currentSelection;
          }
          /*
          if (currentSelection > 0) // jezeli obecne zaznaczenie ma wartosc mniejsza niz pierwsza wyswietlana linia
          { 
            if (currentSelection < firstVisibleLine) {firstVisibleLine = currentSelection;}
          } 
          else // Jeśli osiągnięto wartość 0, przejdź do najwyższej wartości 
          {  
            if (currentSelection == maxSelection()) {firstVisibleLine = currentSelection - maxVisibleLines + 1;} // Ustaw pierwszą widoczną linię na najwyższą
            
          } 
          */  
        } 
      }  
      else 
      {
        if (listedStations == true)
        {
          station_nr++;
          //if (listedStations == 1) station_nr++;
          if (station_nr > stationsCount) {station_nr = 1;}//stationsCount;
          Serial.print("Numer stacji do przodu: "); Serial.println(station_nr);

          scrollDown();
        }
        else
        {    
                 
          //currentSelection = station_nr - 1; // Przywracamy zaznaczenie obecnie grajacej stacji
          
          if (maxSelection() - currentSelection < maxVisibleLines) {firstVisibleLine = maxSelection() - maxVisibleLines + 1;} else {firstVisibleLine = currentSelection;}
          /*
          if (currentSelection > 0)
          {
            if (currentSelection < firstVisibleLine) // jezeli obecne zaznaczenie ma wartosc mniejsza niz pierwsza wyswietlana linia
            {
              firstVisibleLine = currentSelection;
            }
          } 
          else 
          {  // Jeśli osiągnięto wartość 0, przejdź do najwyższej wartości
            if (currentSelection = maxSelection())
            {
              firstVisibleLine = currentSelection - maxVisibleLines + 1;  // Ustaw pierwszą widoczną linię na najwyższą
            }
          }
          */ 
        }
      }
      displayStations();
    } 
    else 
    {
      if (bankMenuEnable == false)
      {
        if (digitalRead(DT_PIN2) == HIGH) // pokrecenie enkoderem 2
        {volumeDown();} 
        else 
        {volumeUp();}
      }
    }

    if (bankMenuEnable == true)  // Przewijanie listy banków stacji radiowych
    {
      if (digitalRead(DT_PIN2) == HIGH) 
      {
        bank_nr--;
        if (bank_nr < 1) {bank_nr = bank_nr_max;}
      } 
      else 
      {
        bank_nr++;
        if (bank_nr > bank_nr_max) {bank_nr = 1;}
      }
      bankMenuDisplay();
    }
  }
  prev_CLK_state2 = CLK_state2;
      
  if ((button2.isReleased()) && (listedStations == true)) 
  {
    listedStations = false;
    volumeSet = false;
    changeStation();
    if (!sdPlayerOLEDActive) displayRadio(); // Blokuj gdy SDPlayer aktywny
    u8g2.sendBuffer();
    clearFlags();
  }

  if ((button2.isPressed()) && (listedStations == false) && (bankMenuEnable == false)) 
  {
    displayStartTime = millis();
    timeDisplay = false;
    volumeSet = true;
    displayActive = true;
    volumeDisplay();  // Po nacisnieciu enkodera2 wyswietlamy menu głośnosci
  }

  if ((button2.isPressed()) && (bankMenuEnable == true)) 
  {
    station_nr = 1;
    fetchStationsFromServer();
    changeStation();
    u8g2.clearBuffer();
    if (!sdPlayerOLEDActive) displayRadio(); // Blokuj gdy SDPlayer aktywny

    volumeSet = false;
    bankMenuEnable = false;
  }

}



void handleEncoder2VolumeStationsClick()
{
  // Double-click detection dla aktywacji SD Player (enkoder) - używa ezButton
  static int encoderClickCount = 0;
  static unsigned long lastEncoderClickTime = 0;
  
  // Sprawdź double-click tylko gdy SD Player nieaktywny - użyj button2.isReleased()
  if (!sdPlayerOLEDActive && button2.isReleased()) {
    unsigned long now = millis();
    
    // Sprawdź czy to szybkie kliknięcie (w oknie 600ms)
    if (now - lastEncoderClickTime < 600) {
      encoderClickCount++;
      Serial.printf("[Encoder2] Kliknięcie #%d\n", encoderClickCount);
    } else {
      encoderClickCount = 1; // Reset po timeout
      Serial.println("[Encoder2] Pierwsze kliknięcie (reset licznika)");
    }
    lastEncoderClickTime = now;
    
    // Sprawdź czy to double-click
    if (encoderClickCount >= 2) {
      // Double-click wykryty - aktywuj SD Player
      if (g_sdPlayerOLED) {
        // Zapamiętaj aktualny bank i stację przed aktywacją
        sdPlayerReturnBank = bank_nr;
        sdPlayerReturnStation = station_nr;
        Serial.printf("[SDPlayer] DOUBLE-CLICK wykryty! Zapisano pozycję: Bank %d, Stacja %d\n", sdPlayerReturnBank, sdPlayerReturnStation);
        
        g_sdPlayerOLED->activate();
        sdPlayerOLEDActive = true;
        Serial.println("[DOUBLE-CLICK] Aktywacja SD Player - handleEncoder2VolumeStationsClick");
      }
      encoderClickCount = 0; // Reset licznika
      return;
    }
  }
  
  // Obsługa SDPlayer OLED
  if (g_sdPlayerOLED && g_sdPlayerOLED->isActive()) {
    // Obsługa przytrzymania z różnymi czasami:
    // < 1s      = krótkie kliknięcie (play/pause)
    // 3-5s     = średnie przytrzymanie (select track)
    // > 6s     = długie przytrzymanie (tryb volume lub wyjście)
    static unsigned long buttonPressStartTime2 = 0;
    static bool wasPressed2 = false;
    static bool mediumHoldExecuted = false;
    static bool longHoldExecuted = false;
    bool isPressed = (digitalRead(SW_PIN2) == LOW); // Zakładając aktywne LOW
    
    if (isPressed && !wasPressed2) {
      // Przycisk został właśnie wciśnięty
      buttonPressStartTime2 = millis();
      wasPressed2 = true;
      mediumHoldExecuted = false;
      longHoldExecuted = false;
    } else if (!isPressed && wasPressed2) {
      // Przycisk został zwolniony
      wasPressed2 = false;
      unsigned long pressDuration = millis() - buttonPressStartTime2;
      
      // Krótkie kliknięcie (< 3s) - jeśli nie wykonano akcji przytrzymania
      if (pressDuration < 3000 && !mediumHoldExecuted && !longHoldExecuted) {
        g_sdPlayerOLED->onEncoderButton();
      }
    } else if (isPressed && wasPressed2) {
      // Przycisk jest ciągle przytrzymany - sprawdzamy czas
      unsigned long pressDuration = millis() - buttonPressStartTime2;
      
      // Średnie przytrzymanie (3-5s) - zatwierdź wybór utworu
      if (pressDuration >= 3000 && pressDuration < 6000 && !mediumHoldExecuted) {
        g_sdPlayerOLED->onEncoderButtonMediumHold(pressDuration);
        mediumHoldExecuted = true;
      }
      // Długie przytrzymanie (>6s) - przełącz tryb volume
      else if (pressDuration >= 6000 && !longHoldExecuted) {
        g_sdPlayerOLED->onEncoderButtonLongHold(pressDuration);
        longHoldExecuted = true;
        wasPressed2 = false; // Reset
        return;
      }
    }
    
    // Obsługa obracania enkodera
    CLK_state2 = digitalRead(CLK_PIN2);
    if (CLK_state2 != prev_CLK_state2 && CLK_state2 == HIGH) {
      if (digitalRead(DT_PIN2) == HIGH) {
        g_sdPlayerOLED->onEncoderLeft();  // Lewo = Góra
      } else {
        g_sdPlayerOLED->onEncoderRight(); // Prawo = Dół
      }
    }
    prev_CLK_state2 = CLK_state2;
    
    return; // Wyjdź z funkcji - SDPlayer obsłużony
  }
  
  // ============== KONIEC OBSŁUGI MODUŁÓW - kontynuacja normalnej obsługi radia ==============
  
  CLK_state2 = digitalRead(CLK_PIN2);                       // Odczytanie aktualnego stanu pinu CLK enkodera 2
  if (CLK_state2 != prev_CLK_state2 && CLK_state2 == HIGH)  // Sprawdzenie, czy stan CLK zmienił się na wysoki
  {
    timeDisplay = false;
    displayActive = true;
    displayStartTime = millis();

    if ((listedStations == false) && (bankMenuEnable == false))  // Przewijanie listy stacji radiowych
    {
      if (digitalRead(DT_PIN2) == HIGH) 
      {
        volumeDown();
      } 
      else 
      {
        volumeUp();
      }
    } 
    else 
    {
      if ((bankMenuEnable == false) && (volumeSet == true)) 
      {
        if (digitalRead(DT_PIN2) == HIGH)  // pokrecenie enkoderem 2
        {
          volumeDown();
        } 
        else 
        {
          volumeUp();
        }
      
      }
    }
    #ifndef twoEncoders // Kompilujemy ten blok jesli nie ma zdefinionwanego "twoEncoders", dla 2 enkoderów jest niepotrzebny
      if ((listedStations == true) && (bankMenuEnable == false)) 
      {
        station_nr = currentSelection + 1;
        if (digitalRead(DT_PIN2) == HIGH) 
        {
          station_nr--;
          if (station_nr < 1) { station_nr = stationsCount; }
          scrollUp();
        } 
        else 
        {
          station_nr++;
          if (station_nr > stationsCount) { station_nr = 1; }  //stationsCount;
          scrollDown();
        }
        displayStations();
      }

      if (bankMenuEnable == true)  // Przewijanie listy banków stacji radiowych
      {
        if (digitalRead(DT_PIN2) == HIGH) 
        {
          bank_nr--;
          if (bank_nr < 1) 
          {
            bank_nr = bank_nr_max;
          }
        } 
        else 
        {
          bank_nr++;
          if (bank_nr > bank_nr_max) 
          {
            bank_nr = 1;
          }
        }
        bankMenuDisplay();
      }
    #endif    
  }
  prev_CLK_state2 = CLK_state2;


  if ((button2.isReleased()) && (encoderButton2 == true))  // jestesmy juz w menu listy stacji to zmieniamy stacje po nacisnieciu przycisku
  {
    encoderButton2 = false;
    //  Serial.println("debug--------------------------------> SET ENCODER button 2 FALSE");
  }

  #ifdef twoEncoders
    if ((button2.isPressed())) // zmieniamy stację
    {
      volumeMute = !volumeMute;
      if (volumeMute == true)
      {
        audio.setVolume(0);   
      }
      else if (volumeMute == false)
      {
        audio.setVolume(volumeValue);   
      }
      updateSpeakersPin(); // Aktualizuj pin głośników
      displayRadio();
      return;
    }
  #endif

  if ((button2.isPressed())) // zmieniamy stację
  {
    
    if ((encoderButton2 == false) && (listedStations == true) && (bankMenuEnable == false) && (volumeSet == false))  // jestesmy juz w menu listy stacji to zmieniamy stacje po nacisnieciu przycisku
    {
      encoderButton2 = true;
      u8g2.clearBuffer();
      changeStation();
      displayRadio();
      clearFlags();       
    }
  
    else if ((encoderButton2 == false) && (listedStations == false) && (bankMenuEnable == false) && (volumeSet == false))  // wchodzimy do listy
    {
      displayStartTime = millis();
      timeDisplay = false;
      displayActive = true;
      listedStations = true;
      currentSelection = station_nr - 1; 

      if (maxSelection() - currentSelection < maxVisibleLines) 
      {
        firstVisibleLine = maxSelection() - maxVisibleLines + 1;
      } 
      else 
      {
        firstVisibleLine = currentSelection;
      }
      displayStations();
    } 
      
    else if ((bankMenuEnable == true) && (volumeSet == false)) 
    {
      displayStartTime = millis();
      timeDisplay = false;
      displayActive = true;
      
      //previous_bank_nr = bank_nr;
      station_nr = 1;

      listedStations = false;
      volumeSet = false;
      bankMenuEnable = false;
      bankNetworkUpdate = false;

      fetchStationsFromServer();
      changeStation();
      u8g2.clearBuffer();
      displayRadio();
    }  
  }
}


void stationNameSwap()
{
  stationNameFromStream = !stationNameFromStream;
  if (stationNameFromStream) stationNameLenghtCut = 40; else stationNameLenghtCut = 24;
}

void saveConfig() 
{
  Serial.println("=== saveConfig() START ===");
  
  #ifdef AUTOSTORAGE
    // KRYTYCZNE: Sprawdz czy wskaznik storage jest prawidlowy
    if (_storage == nullptr) {
      Serial.println("BLAD KRYTYCZNY: _storage jest NULL!");
      return;
    }
  #endif
  
  Serial.printf("STORAGE dostepny: %s\n", storageTextName);
  
  // Usuń stary plik jeśli istnieje (FILE_WRITE na ESP32 dopisuje zamiast nadpisywać!)
  if (STORAGE.exists("/config.txt")) 
  {
    Serial.println("Plik config.txt istnieje - usuwam...");
    if (STORAGE.remove("/config.txt")) {
      Serial.println("Stary plik usuniety pomyslnie");
    } else {
      Serial.println("BLAD: Nie mozna usunac starego pliku!");
      return;
    }
  }
  
  // Utwórz nowy plik
  Serial.println("Tworzenie nowego pliku config.txt...");
  myFile = STORAGE.open("/config.txt", FILE_WRITE);
  if (myFile) 
	  {
      myFile.println("#### Evo Web Radio Config File ####");
      myFile.print("Display Brightness =");    myFile.print(displayBrightness); myFile.println(";");
      myFile.print("Dimmer Display Brightness =");    myFile.print(dimmerDisplayBrightness); myFile.println(";");
      myFile.print("Auto Dimmer Time ="); myFile.print(displayAutoDimmerTime); myFile.println(";");
      myFile.print("Auto Dimmer ="); myFile.print(displayAutoDimmerOn); myFile.println(";");
      myFile.println("Voice Info Every Hour =" + String(timeVoiceInfoEveryHour) + ";");
      myFile.println("VU Meter Mode =" + String(vuMeterMode) + ";");
      myFile.println("Encoder Function Order  =" + String(encoderFunctionOrder) + ";");
      myFile.println("Startup Display Mode  =" + String(displayMode) + ";");
      myFile.println("VU Meter On  =" + String(vuMeterOn) + ";");
	    myFile.println("VU Meter Refresh Time  =" + String(vuMeterRefreshTime) + ";");
      myFile.println("Scroller Refresh Time =" + String(scrollingRefresh) + ";");
      myFile.println("ADC Keyboard Enabled =" + String(adcKeyboardEnabled) + ";");
      myFile.println("Display Power Save Enabled =" + String(displayPowerSaveEnabled) + ";");  
      myFile.println("Display Power Save Time =" + String(displayPowerSaveTime) + ";");
      myFile.println("Max Volume Extended =" + String(maxVolumeExt) + ";");
      myFile.println("VU Meter Peak Hold On =" + String(vuPeakHoldOn) + ";");
      myFile.println("VU Meter Smooth On =" + String(vuSmooth) + ";");
      myFile.println("VU Meter Rise Speed =" + String(vuRiseSpeed) + ";");
      myFile.println("VU Meter Fall Speed =" + String(vuFallSpeed) + ";");
      myFile.println("Display Clock in Sleep =" + String(f_displayPowerOffClock) + ";");
      myFile.print("Dimmer Sleep Display Brightness =");    myFile.print(dimmerSleepDisplayBrightness); myFile.println(";");
      myFile.print("Radio switch to standby after Power Fail =");    myFile.print(f_sleepAfterPowerFail); myFile.println(";");
      myFile.println("Volume fade on station change and power off =" + String(f_volumeFadeOn) + ";");
      myFile.println("Save Always Station Bank Volume or only during power off =" + String(f_saveVolumeStationAlways) + ";");
      myFile.println("Power Off Animation =" + String(f_powerOffAnimation) + ";");
      myFile.println("BT Module UART Enabled =" + String(btModuleEnabled) + ";");
      myFile.println("Analyzer FFT Enabled =" + String(analyzerEnabled) + ";");
      myFile.println("Analyzer Styles =" + String(analyzerStyles) + ";");
      myFile.println("Analyzer Preset =" + String(analyzerPreset) + ";");
      myFile.println("Display Mode 6 Enabled =" + String(displayMode6Enabled) + ";");
      myFile.println("Display Mode 8 Enabled =" + String(displayMode8Enabled) + ";");
      myFile.println("Display Mode 10 Enabled =" + String(displayMode10Enabled) + ";");
      myFile.println("SDPlayer Style 1 Enabled =" + String(sdPlayerStyle1Enabled) + ";");
      myFile.println("SDPlayer Style 2 Enabled =" + String(sdPlayerStyle2Enabled) + ";");
      myFile.println("SDPlayer Style 3 Enabled =" + String(sdPlayerStyle3Enabled) + ";");
      myFile.println("SDPlayer Style 4 Enabled =" + String(sdPlayerStyle4Enabled) + ";");
      myFile.println("SDPlayer Style 5 Enabled =" + String(sdPlayerStyle5Enabled) + ";");
      myFile.println("SDPlayer Style 6 Enabled =" + String(sdPlayerStyle6Enabled) + ";");
      myFile.println("SDPlayer Style 7 Enabled =" + String(sdPlayerStyle7Enabled) + ";");
      myFile.println("SDPlayer Style 9 Enabled =" + String(sdPlayerStyle9Enabled) + ";");
      myFile.println("SDPlayer Style 10 Enabled =" + String(sdPlayerStyle10Enabled) + ";");
      myFile.println("SDPlayer Style 11 Enabled =" + String(sdPlayerStyle11Enabled) + ";");
      myFile.println("SDPlayer Style 12 Enabled =" + String(sdPlayerStyle12Enabled) + ";");
      myFile.println("SDPlayer Style 13 Enabled =" + String(sdPlayerStyle13Enabled) + ";");
      myFile.println("SDPlayer Style 14 Enabled =" + String(sdPlayerStyle14Enabled) + ";");
      myFile.println("Presets On =" + String(presetsOn) + ";");
      myFile.println("Speakers Enabled =" + String(speakersEnabled) + ";");
      
      myFile.flush();  // Wymuś zapis do pamięci fizycznej
      myFile.close();
      
      // Weryfikacja zapisu
      delay(100);
      if (STORAGE.exists("/config.txt")) {
        Serial.println("Weryfikacja: Plik config.txt zostal zapisany");
      } else {
        Serial.println("BLAD WERYFIKACJI: Plik config.txt NIE istnieje po zapisie!");
      }
      
      Serial.println("=== CONFIG ZAPISANY POMYSLNIE ===");
      Serial.printf("Saved values: brightness=%d, dimmer=%d, analyzerEnabled=%d, analyzerStyles=%d\n", 
                    displayBrightness, dimmerDisplayBrightness, analyzerEnabled, analyzerStyles);
      Serial.printf("analyzerPreset=%d\n", analyzerPreset);
  } 
  else 
  {
      Serial.println("BLAD: Nie mozna otworzyc pliku config.txt do zapisu!");
  }
  
  Serial.println("=== saveConfig() END ===");
}

void saveAdcConfig() 
{

  // Sprawdź, czy plik istnieje
  if (STORAGE.exists("/adckbd.txt")) 
  {
    Serial.println("Plik adckbd.txt już istnieje.");

    // Otwórz plik do zapisu i nadpisz aktualną wartość konfiguracji klawiatury ADC
    myFile = STORAGE.open("/adckbd.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println("#### Evo Web Radio Config File - ADC Keyboard ####");
      myFile.println("keyboardButtonThreshold_0 =" + String(keyboardButtonThreshold_0) + ";");
      myFile.println("keyboardButtonThreshold_1 =" + String(keyboardButtonThreshold_1) + ";");
      myFile.println("keyboardButtonThreshold_2 =" + String(keyboardButtonThreshold_2) + ";");
      myFile.println("keyboardButtonThreshold_3 =" + String(keyboardButtonThreshold_3) + ";");
      myFile.println("keyboardButtonThreshold_4 =" + String(keyboardButtonThreshold_4) + ";");
      myFile.println("keyboardButtonThreshold_5 =" + String(keyboardButtonThreshold_5) + ";");
      myFile.println("keyboardButtonThreshold_6 =" + String(keyboardButtonThreshold_6) + ";");
      myFile.println("keyboardButtonThreshold_7 =" + String(keyboardButtonThreshold_7) + ";");
      myFile.println("keyboardButtonThreshold_8 =" + String(keyboardButtonThreshold_8) + ";");
      myFile.println("keyboardButtonThreshold_9 =" + String(keyboardButtonThreshold_9) + ";");
      myFile.println("keyboardButtonThreshold_Shift =" + String(keyboardButtonThreshold_Shift) + ";");
      myFile.println("keyboardButtonThreshold_Memory =" + String(keyboardButtonThreshold_Memory) + ";");
      myFile.println("keyboardButtonThreshold_Band =" + String(keyboardButtonThreshold_Band) + ";");
      myFile.println("keyboardButtonThreshold_Auto =" + String(keyboardButtonThreshold_Auto) + ";");
      myFile.println("keyboardButtonThreshold_Scan =" + String(keyboardButtonThreshold_Scan) + ";");
      myFile.println("keyboardButtonThreshold_Mute =" + String(keyboardButtonThreshold_Mute) + ";");
      myFile.println("keyboardButtonThresholdTolerance =" + String(keyboardButtonThresholdTolerance) + ";");
      myFile.println("keyboardButtonNeutral =" + String(keyboardButtonNeutral) + ";");
      myFile.println("keyboardSampleDelay =" + String(keyboardSampleDelay) + ";");
      myFile.close();
      Serial.println("Aktualizacja adckbd.txt na karcie SD");
    } 
	  else 
	  {
      Serial.println("Błąd podczas otwierania pliku adckbd.txt.");
    }
  } 
  else 
  {
    Serial.println("Plik adckbd.txt nie istnieje. Tworzenie...");

    // Utwórz plik i zapisz w nim aktualne wartości konfiguracji
    myFile = STORAGE.open("/adckbd.txt", FILE_WRITE);
    if (myFile) 
	  {
      myFile.println("#### Evo Web Radio Config File - ADC Keyboard ####");
      myFile.println("keyboardButtonThreshold_0 =" + String(keyboardButtonThreshold_0) + ";");
      myFile.println("keyboardButtonThreshold_1 =" + String(keyboardButtonThreshold_1) + ";");
      myFile.println("keyboardButtonThreshold_2 =" + String(keyboardButtonThreshold_2) + ";");
      myFile.println("keyboardButtonThreshold_3 =" + String(keyboardButtonThreshold_3) + ";");
      myFile.println("keyboardButtonThreshold_4 =" + String(keyboardButtonThreshold_4) + ";");
      myFile.println("keyboardButtonThreshold_5 =" + String(keyboardButtonThreshold_5) + ";");
      myFile.println("keyboardButtonThreshold_6 =" + String(keyboardButtonThreshold_6) + ";");
      myFile.println("keyboardButtonThreshold_7 =" + String(keyboardButtonThreshold_7) + ";");
      myFile.println("keyboardButtonThreshold_8 =" + String(keyboardButtonThreshold_8) + ";");
      myFile.println("keyboardButtonThreshold_9 =" + String(keyboardButtonThreshold_9) + ";");
      myFile.println("keyboardButtonThreshold_Shift =" + String(keyboardButtonThreshold_Shift) + ";");
      myFile.println("keyboardButtonThreshold_Memory =" + String(keyboardButtonThreshold_Memory) + ";");
      myFile.println("keyboardButtonThreshold_Band =" + String(keyboardButtonThreshold_Band) + ";");
      myFile.println("keyboardButtonThreshold_Auto =" + String(keyboardButtonThreshold_Auto) + ";");
      myFile.println("keyboardButtonThreshold_Scan =" + String(keyboardButtonThreshold_Scan) + ";");
      myFile.println("keyboardButtonThreshold_Mute =" + String(keyboardButtonThreshold_Mute) + ";");
      myFile.println("keyboardButtonThresholdTolerance =" + String(keyboardButtonThresholdTolerance) + ";");
      myFile.println("keyboardButtonNeutral =" + String(keyboardButtonNeutral) + ";");
      myFile.println("keyboardSampleDelay =" + String(keyboardSampleDelay) + ";");
      myFile.close();
      Serial.println("Utworzono i zapisano adckbd.txt na karcie SD");
    } 
	  else 
	  {
      Serial.println("Błąd podczas tworzenia pliku adckbd.txt.");
    }
  }


}


void readConfig() 
{
  Serial.println("=== readConfig() START ===");
  
  #ifdef AUTOSTORAGE
    // KRYTYCZNE: Sprawdz czy wskaznik storage jest prawidlowy
    if (_storage == nullptr) {
      Serial.println("BLAD KRYTYCZNY: _storage jest NULL!");
      return;
    }
  #endif
  
  Serial.printf("STORAGE dostepny: %s\n", storageTextName);
  Serial.println("Odczyt pliku config.txt z karty");
  String fileName = String("/config.txt"); // Tworzymy nazwę pliku

  if (!STORAGE.exists(fileName))               // Sprawdzamy, czy plik istnieje
  {
    Serial.println("BLAD: Plik config.txt nie istnieje!");
    configExist = false;
    return;
  }
 
  Serial.println("Plik config.txt istnieje, otwieranie...");
  File configFile = STORAGE.open(fileName, FILE_READ);// Otwieramy plik w trybie do odczytu
  if (!configFile)  // jesli brak pliku to...
  {
    Serial.println("BLAD: Nie mozna otworzyc pliku konfiguracji");
    return;
  }
  
  Serial.println("Plik otwarty pomyslnie, odczyt danych...");
  // Przechodzimy do odpowiedniego wiersza pliku
  int currentLine = 0;
  String configValue = "";
  while (configFile.available() && currentLine < CONFIG_COUNT) 

  {
    String line = configFile.readStringUntil(';'); //('\n');
    
    int lineStart = line.indexOf("=") + 1;  // Szukamy miejsca, gdzie zaczyna wartość zmiennej
    if ((lineStart != -1)) //&& (currentLine != 0)) // Pomijamy pierwszą linijkę gdzie jest opis pliku
	  {
      configValue = line.substring(lineStart);  // Wyciągamy numer od miejsc a"="
      configValue.trim();                       // Usuwamy białe znaki na początku i końcu
      Serial.print("debug SD -> Odczytano zmienna konfiguracji numer:" + String(currentLine) + " wartosc:");
      Serial.println(configValue);
      configArray[currentLine] = configValue.toInt();
    }
    currentLine++;
  }
  Serial.print("Zamykamy plik config na wartosci currentLine:");
  Serial.println(currentLine);
  
  //Odczyt kontrolny
  //for (int i = 0; i < 16; i++) 
  //{
  //  Serial.print("wartosc: " + String(i) + " z tablicy konfiguracji:");
  //  Serial.println(configArray[i]);
  //}

  configFile.close();  // Zamykamy plik po odczycie kodow pilota

  displayBrightness = configArray[0];
  dimmerDisplayBrightness = configArray[1];
  displayAutoDimmerTime = configArray[2];
  displayAutoDimmerOn = configArray[3];
  timeVoiceInfoEveryHour = configArray[4];
  vuMeterMode = configArray[5];
  encoderFunctionOrder = configArray[6];
  displayMode = configArray[7];
  vuMeterOn = configArray[8];
  vuMeterRefreshTime = configArray[9];
  scrollingRefresh = configArray[10];
  adcKeyboardEnabled = configArray[11];
  displayPowerSaveEnabled = configArray[12];
  displayPowerSaveTime = configArray[13];
  maxVolumeExt = configArray[14];
  vuPeakHoldOn = configArray[15];
  vuSmooth = configArray[16];
  vuRiseSpeed = configArray[17];
  vuFallSpeed = configArray[18];
  f_displayPowerOffClock = configArray[19];
  dimmerSleepDisplayBrightness = configArray[20];
  f_sleepAfterPowerFail = configArray[21];
  f_volumeFadeOn = configArray[22];
  f_saveVolumeStationAlways = configArray[23];
  f_powerOffAnimation = configArray[24];
  btModuleEnabled = configArray[25];
  analyzerEnabled = configArray[26];
  analyzerStyles = configArray[27];
  analyzerPreset = configArray[28];
  displayMode6Enabled = configArray[29];
  displayMode8Enabled = configArray[30]; // Poprawione z [31] na [30]
  displayMode10Enabled = configArray[31]; // Poprawione z [30] na [31]
  sdPlayerStyle1Enabled = configArray[32]; // Poprawione z [31] na [32]
  sdPlayerStyle2Enabled = configArray[33]; // Poprawione z [32] na [33]
  sdPlayerStyle3Enabled = configArray[34]; // Poprawione z [33] na [34]
  sdPlayerStyle4Enabled = configArray[35]; // Poprawione z [34] na [35]
  sdPlayerStyle5Enabled = configArray[36]; // Poprawione z [35] na [36]
  sdPlayerStyle6Enabled = configArray[37]; // Poprawione z [36] na [37]
  sdPlayerStyle7Enabled = configArray[38]; // Poprawione z [37] na [38]
  sdPlayerStyle9Enabled = configArray[39]; // Poprawione z [38] na [39]
  sdPlayerStyle10Enabled = configArray[40]; // Poprawione z [39] na [40]
  sdPlayerStyle11Enabled = configArray[41]; // Poprawione z [40] na [41]
  sdPlayerStyle12Enabled = configArray[42]; // Poprawione z [41] na [42]
  sdPlayerStyle13Enabled = configArray[43]; // Poprawione z [42] na [43]
  sdPlayerStyle14Enabled = configArray[44]; // Poprawione z [43] na [44]
  presetsOn       = configArray[45]; // Tryb PRESETS
  speakersEnabled = configArray[46]; // Pin głośników

  Serial.println("=== CONFIG ODCZYTANY POMYSLNIE ===");
  Serial.printf("Loaded values: brightness=%d, dimmer=%d, analyzerEnabled=%d, analyzerStyles=%d\n", 
                displayBrightness, dimmerDisplayBrightness, analyzerEnabled, analyzerStyles);
  Serial.printf("displayMode=%d, encoderFunctionOrder=%d, displayAutoDimmerOn=%d\n",
                displayMode, encoderFunctionOrder, displayAutoDimmerOn);
  Serial.println("=== readConfig() END ===");

  if (maxVolumeExt == 1)
  { 
    maxVolume = 42;
  }
  else
  {
    maxVolume = 21;
  }
  audio.setVolumeSteps(maxVolume);
  //stationNameSwap();
}


void readAdcConfig() 
{

  Serial.println("Odczyt pliku konfiguracji klawiatury ADC adckbd.txt z karty");
  String fileName = String("/adckbd.txt"); // Tworzymy nazwę pliku

  if (!STORAGE.exists(fileName))               // Sprawdzamy, czy plik istnieje
  {
    Serial.println("Błąd: Plik nie istnieje.");
    return;
  }
 
  File configFile = STORAGE.open(fileName, FILE_READ);// Otwieramy plik w trybie do odczytu
  if (!configFile)  // jesli brak pliku to...
  {
    Serial.println("Błąd: Nie można otworzyć pliku konfiguracji klawiatury ADC");
    return;
  }
  // Przechodzimy do odpowiedniego wiersza pliku
  int currentLine = 0;
  String configValue = "";
  while (configFile.available()) 
  {
    String line = configFile.readStringUntil(';'); //('\n');
    
    int lineStart = line.indexOf("=") + 1;  // Szukamy miejsca, gdzie zaczyna wartość zmiennej
    if ((lineStart != -1)) //&& (currentLine != 0)) // Pomijamy pierwszą linijkę gdzie jest opis pliku
	  {
      configValue = line.substring(lineStart);  // Wyciągamy URL od "http"
      configValue.trim();                      // Usuwamy białe znaki na początku i końcu
      Serial.print(" Odczytano ustawienie ADC nr.:" + String(currentLine) + " wartosc:");
      Serial.println(configValue);
      configAdcArray[currentLine] = configValue.toInt();
    }
    currentLine++;
  }
  Serial.print("Zamykamy plik config na wartosci currentLine:");
  Serial.println(currentLine);
  
  //Odczyt kontrolny
  //for (int i = 0; i < 16; i++) 
  //{
  //  Serial.print("wartosc poziomu ADC: " + String(i) + " z tablicy:");
  //  Serial.println(configAdcArray[i]);
 // }

  configFile.close();  // Zamykamy plik po odczycie kodow pilota

 // ---- Progi przełaczania ADC dla klawiatury matrycowej 5x3 w tunrze Sony ST-120 ---- //
        // Pozycja neutralna
  keyboardButtonThreshold_0 = configAdcArray[0];          // Przycisk 0
  keyboardButtonThreshold_1 = configAdcArray[1];          // Przycisk 1
  keyboardButtonThreshold_2 = configAdcArray[2];          // Przycisk 2
  keyboardButtonThreshold_3 = configAdcArray[3];          // Przycisk 3
  keyboardButtonThreshold_4 = configAdcArray[4];          // Przycisk 4
  keyboardButtonThreshold_5 = configAdcArray[5];          // Przycisk 5
  keyboardButtonThreshold_6 = configAdcArray[6];        // Przycisk 6
  keyboardButtonThreshold_7 = configAdcArray[7];        // Przycisk 7
  keyboardButtonThreshold_8 = configAdcArray[8];        // Przycisk 8
  keyboardButtonThreshold_9 = configAdcArray[9];        // Przycisk 9
  keyboardButtonThreshold_Shift = configAdcArray[10];   // Shift - funkcja Enter/OK
  keyboardButtonThreshold_Memory = configAdcArray[11];  // Memory - funkcja Bank menu
  keyboardButtonThreshold_Band = configAdcArray[12];    // Przycisk Band - funkcja Back
  keyboardButtonThreshold_Auto = configAdcArray[13];    // Przycisk Auto - przelacza Radio/Zegar
  keyboardButtonThreshold_Scan = configAdcArray[14];    // Przycisk Scan - funkcja Dimmer ekranu OLED
  keyboardButtonThreshold_Mute = configAdcArray[15];      // Przycisk Mute - funkcja MUTE
  keyboardButtonThresholdTolerance = configAdcArray[16];  // Tolerancja dla pomiaru ADC
  keyboardButtonNeutral = configAdcArray[17];             // Pozycja neutralna 
  keyboardSampleDelay = configAdcArray[18];               // Czas co ile ms odczytywac klawiature

}



void readPSRAMstations()  // Funkcja testowa-debug, do odczytu PSRAMu, nie uzywana przez inne funkcje
{
  Serial.println("-------- POCZATEK LISTY STACJI ---------- ");
  for (int i = 0; i < stationsCount; i++) 
  {
    // Odczyt stacji pod daną komórka pamieci PSRAM:
    char station[STATION_NAME_LENGTH + 1];  // Tablica na nazwę stacji o maksymalnej długości zdefiniowanej przez STATION_NAME_LENGTH
    memset(station, 0, sizeof(station));    // Wyczyszczenie tablicy zerami przed zapisaniem danych
    int length = psramData[(i) * (STATION_NAME_LENGTH + 1)];   // Odczytaj długość nazwy stacji z PSRAM dla bieżącego indeksu stacji
      
    for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
    { // Odczytaj nazwę stacji z PSRAM jako ciąg bajtów, maksymalnie do STATION_NAME_LENGTH
        station[j] = psramData[(i) * (STATION_NAME_LENGTH + 1) + 1 + j];  // Odczytaj znak po znaku nazwę stacji
    }
    String stationNameText = String(station);
    
    Serial.print(i+1);
    Serial.print(" ");
    Serial.println(stationNameText);
  }	
  
  Serial.println("-------- KONIEC LISTY STACJI ---------- ");

  String stationUrl = "";

  // Odczyt stacji pod daną komórka pamieci PSRAM:
  char station[STATION_NAME_LENGTH + 1];  // Tablica na nazwę stacji o maksymalnej długości zdefiniowanej przez STATION_NAME_LENGTH
  memset(station, 0, sizeof(station));    // Wyczyszczenie tablicy zerami przed zapisaniem danych
  int length = psramData[(station_nr - 1) * (STATION_NAME_LENGTH + 1)];   // Odczytaj długość nazwy stacji z PSRAM dla bieżącego indeksu stacji
      
  for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
  { // Odczytaj nazwę stacji z PSRAM jako ciąg bajtów, maksymalnie do STATION_NAME_LENGTH
    station[j] = psramData[(station_nr - 1) * (STATION_NAME_LENGTH + 1) + 1 + j];  // Odczytaj znak po znaku nazwę stacji
  }
  
  //String stationNameText = String(station);
  
  Serial.println("-------- OBECNIE GRAMY  ---------- ");
  Serial.print(station_nr - 1);
  Serial.print(" ");
  Serial.println(String(station));

}

void webUrlStationPlay() 
{
  audio.stopSong();

  // Usunięcie wszystkich znaków z obiektów 
  stationString.remove(0);  
  stationNameStream.remove(0);
  stationStringWeb.remove(0);
  stationLogoUrl.remove(0);

  bitrateString.remove(0);
  sampleRateString.remove(0);
  bitsPerSampleString.remove(0); 
  SampleRate = 0;
  SampleRateRest = 0;

  sampleRateString = "--.-";
  bitsPerSampleString = "--";
  bitrateString = "-?-";
    
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub14_tf); // cziocnka 14x11
  u8g2.drawStr(34, 33, "Loading stream..."); // 8 znakow  x 11 szer
  u8g2.sendBuffer();

  mp3 = flac = aac = vorbis = opus = false;
  streamCodec = "";
    
  Serial.println("debug-- Read station from WEB URL");
    
  if (url2play != "") 
  {
    url2play.trim(); // Usuwamy białe znaki na początku i końcu
  }
  else
  {
	  return;
  }
  
  if (url2play.isEmpty()) // jezeli link URL jest pusty
  {
    Serial.println("Błąd: Nie znaleziono stacji dla podanego numeru.");
    return;
  }
    
  // Weryfikacja, czy w linku znajduje się "http" lub "https"
  if (url2play.startsWith("http://") || url2play.startsWith("https://")) 
  {
    // Wydrukuj nazwę stacji i link na serialu
    Serial.print("Link do stacji: ");
    Serial.println(url2play);
    
    u8g2.setFont(spleen6x12PL);  // wypisujemy jaki stream jakie stacji jest ładowany
    u8g2.drawStr(34, 55, String(url2play).c_str());
    u8g2.sendBuffer();
    
    // Połącz z daną stacją
    audio.connecttohost(url2play.c_str());
    urlPlaying = true;
    station_nr = 0;
    bank_nr = 0;
    //stationName = stationNameStream;
    stationName = "WEB URL";
    //wsStationChange(station_nr); // Od teraz Event Audio odpowiada za info o stacji
    //f_audioInfoRefreshDisplayRadio = true;
    wsStationChange(0,0);
    wsAudioRefresh = true;
  } 
  else 
  {
    Serial.println("Błąd: link stacji nie zawiera 'http' lub 'https'");
    Serial.println("Odczytany URL: " + url2play);
  }

  //url2play = "";
}

String stationBankListHtmlMobile()
{
  String html1;
  
  html1 += "<p>Volume: <span id='textSliderValue'>--</span></p>" + String("\n");
  html1 += "<p><input type='range' onchange='updateSliderVolume(this)' id='volumeSlider' min='1' max='" + String(maxVolume) + "' value='1' step='1' class='slider'></p>" + String("\n");
  html1 += "<p>Memory Bank Selection:</p>" + String("\n");

  html1 += "<center>";  
  html1 += "<p>";
  //for (int i = 1; i < 17; i++) // Przyciski Banków
  for (int i = 1; i < bank_nr_max + 1; i++) // Przyciski Banków
  {
    
    if (i == bank_nr)
    {
      html1 += "<button class='buttonBankSelected' onClick=\"changeBank('" + String(i) + "');\" id=\"Bank\">" + String(i) + "</button>" + String("\n");
    }
    else
    {
      html1 += "<button class=\"buttonBank\" onClick=\"changeBank('" + String(i) + "');\" id=\"Bank\">" + String(i) + "</button>" + String("\n");
    }
    //if (i == 8) {html1 += "</p><p>";}
    if (i % 8 == 0) {html1 += "</p><p>";}
  }
  html1 += "</p>";
  html1 += "</center>";
  html1 += "<center>"; 
 
  html1 += "<table>";


  for (int i = 0; i < stationsCount; i++) // lista stacji
  {
    char station[STATION_NAME_LENGTH + 1];  // Tablica na nazwę stacji o maksymalnej długości zdefiniowanej przez STATION_NAME_LENGTH
    memset(station, 0, sizeof(station));    // Wyczyszczenie tablicy zerami przed zapisaniem danych
    int length = psramData[i * (STATION_NAME_LENGTH + 1)];
    for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
    {
      station[j] = psramData[i * (STATION_NAME_LENGTH + 1) + 1 + j];  // Odczytaj znak po znaku nazwę stacji
    }
    
    html1 += "<tr>";
    html1 += "<td><p class='stationNumberList'>" + String(i + 1) + "</p></td>";
    html1 += "<td><p class='stationList' onClick=\"changeStation('" + String(i + 1) +  "');\">" + String(station).substring(0, stationNameLenghtCut) + "</p></td>";
    html1 += "</tr>" + String("\n");
          
  }
 
  html1 += "</table>" + String("\n");
  html1 += "</div>" + String("\n");

  html1 += "<p style=\"font-size: 0.8rem;\">Web Radio, mobile, Evo: " + String(softwareRev) + "</p>" + String("\n");
  html1 += "<p style='font-size: 0.8rem;'>IP: "+ currentIP + "</p>" + String("\n");
  
  html1 += "<a href='/menu' class='button' style='padding: 0.2rem; font-size: 0.7rem; height: auto; line-height: 1;color: white; width: 65px; border: 1px solid black; display: inline-block; border-radius: 5px; text-decoration: none;'>Menu</a>";
  html1 += "</center></body></html>";
  
  return html1;
}

String stationBankListHtmlPC()
{
  String html2;
  

  html2 += "<p>Volume: <span id='textSliderValue'>--</span></p>" + String("\n");
  html2 += "<p><input type='range' onchange='updateSliderVolume(this)' id='volumeSlider' min='1' max='" + String(maxVolume) + "' value='1' step='1' class='slider'></p>" + String("\n");
  html2 += "<p>Memory Bank Selection:</p>" + String("\n");
  
  
  html2 += "<p>";
  //for (int i = 1; i < 17; i++)
  for (int i = 1; i < bank_nr_max + 1; i++) // Przyciski Banków
  {
    
    if (i == bank_nr)
    {
      html2 += "<button class=\"buttonBankSelected\" onClick=\"changeBank('" + String(i) + "');\" id=\"Bank\">" + String(i) + "</button>" + String("\n");
    }
    else
    {
      html2 += "<button class=\"buttonBank\" onClick=\"changeBank('" + String(i) + "');\" id=\"Bank\">" + String(i) + "</button>" + String("\n");
    }
  }
  
  html2 += "</p>";
  html2 += "<center>" + String("\n");
  html2 += "<div class=\"column\">" + String("\n");
  for (int i = 0; i < MAX_STATIONS; i++) 
  //for (int i = 0; i < stationsCount; i++) 
  {
    char station[STATION_NAME_LENGTH + 1];  // Tablica na nazwę stacji o maksymalnej długości zdefiniowanej przez STATION_NAME_LENGTH
    memset(station, 0, sizeof(station));    // Wyczyszczenie tablicy zerami przed zapisaniem danych

    int length = psramData[i * (STATION_NAME_LENGTH + 1)];
    for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
    {
      station[j] = psramData[i * (STATION_NAME_LENGTH + 1) + 1 + j];  // Odczytaj znak po znaku nazwę stacji
    }     

    
    if ((i == 0) || (i == 25) || (i == 50) || (i == 75))
    { 
      html2 += "<table>" + String("\n");
    } 
    
    // 0-98   >98
    if (i >= stationsCount) { station[0] ='\0'; } // Jesli mamy mniej niz 99 stacji to wypełniamy pozostałe komórki pustymi wartościami
                 
    html2 += "<tr>";
    html2 += "<td><p class='stationNumberList'>" + String(i + 1) + "</p></td>";
    html2 += "<td><p class='stationList' onClick=\"changeStation('" + String(i + 1) +  "');\">" + String(station).substring(0, stationNameLenghtCut) + "</p></td>";
    html2 += "</tr>" + String("\n");

    if ((i == 24) || (i == 49) || (i == 74)) //||(i == 98))
    { 
      html2 += "</table>" + String("\n");
    }
           
  }

  html2 += "</table>" + String("\n");
  html2 += "</div>" + String("\n");
  html2 += "<p style=\"font-size: 0.8rem;\">Web Radio, desktop, Evo: " + String(softwareRev) + "</p>" + String("\n");
  html2 += "<p style='font-size: 0.8rem;'>IP: " + currentIP + "</p>" + String("\n");
  html2 += "<a href='/menu' class='button' style='padding: 0.2rem; font-size: 0.7rem; height: auto; line-height: 1;color: white; width: 65px; border: 1px solid black; display: inline-block; border-radius: 5px; text-decoration: none;'>Menu</a>";
  html2 += "</center></body></html>"; 

  return html2;
}

/*
String stationBankListHtml(bool mobilePage)
{
  String html3;

  html3 += "<p>Volume: <span id='textSliderValue'>%%SLIDERVALUE%%</span></p>";
  html3 += "<p><input type='range' onchange='updateSliderVolume(this)' id='volumeSlider' min='1' max='" + String(maxVolume) + "' value='%%SLIDERVALUE%%' step='1' class='slider'></p>";
  html3 += "<br>";
  html3 += "<p>Bank selection:</p>";


  html3 += "<p>";
  for (int i = 1; i < 17; i++)
  {
    
    if (i == bank_nr)
    {
      html3 += "<button class=\"buttonBankSelected\" onClick=\"bankLoad('" + String(i) + "');\" id=\"Bank\">" + String(i) + "</button>" + String("\n");
    }
    else
    {
      html3 += "<button class=\"buttonBank\" onClick=\"bankLoad('" + String(i) + "');\" id=\"Bank\">" + String(i) + "</button>" + String("\n");
    }

    if ((mobilePage == 1) && (i == 8)) {html3 += "</p><p>";}
  }
  
  html3 += "</p>";
  html3 += "<center>" + String("\n");


  if (mobilePage == 1)
  {
   html3 += "<table>";
  }
  else if (mobilePage == 0)
  { 
    html3 += "<div class=\"column\">" + String("\n");
  }
  
  int limit = (mobilePage == 1) ? stationsCount : MAX_STATIONS;

  for (int i = 0; i < limit; i++)
  {
    char station[STATION_NAME_LENGTH + 1];  // Tablica na nazwę stacji o maksymalnej długości zdefiniowanej przez STATION_NAME_LENGTH
    memset(station, 0, sizeof(station));    // Wyczyszczenie tablicy zerami przed zapisaniem danych

    int length = psramData[i * (STATION_NAME_LENGTH + 1)];
    for (int j = 0; j < min(length, STATION_NAME_LENGTH); j++) 
    {
      station[j] = psramData[i * (STATION_NAME_LENGTH + 1) + 1 + j];  // Odczytaj znak po znaku nazwę stacji
    }     

    
    if ((mobilePage == 0) && ((i == 0) || (i == 25) || (i == 50) || (i == 75)))
    { 
      html3 += "<table>" + String("\n");
    } 
 
    // 0-98   >98
    if (mobilePage == 0) 
    {
      if (i >= stationsCount) { station[0] ='\0'; } // Jesli mamy mniej niz 99 stacji to wypełniamy pozostałe komórki pustymi wartościami
    }

    html3 += "<tr>";
    html3 += "<td><p class='stationNumberList'>" + String(i + 1) + "</p></td>";
    html3 += "<td><p class='stationList' onClick=\"changeStation('" + String(i + 1) +  "');\">" + String(station).substring(0, stationNameLenghtCut) + "</p></td>";
    html3 += "</tr>" + String("\n");
   
    if ((mobilePage == 0) && ((i == 24) || (i == 49) || (i == 74))) //||(i == 98))
    { 
      html3 += "</table>" + String("\n");
    }
           
  }

  html3 += "</table>" + String("\n");
  html3 += "</div>" + String("\n");
  if (mobilePage == 0) { html3 += "<p style=\"font-size: 0.8rem;\">Web Radio, desktop, Evo: " + String(softwareRev) + "</p>" + String("\n"); }
  if (mobilePage == 1) { html3 += "<p style=\"font-size: 0.8rem;\">Web Radio, mobile, Evo: " + String(softwareRev) + "</p>" + String("\n"); }
  html3 += "<p style='font-size: 0.8rem;'>IP: " + currentIP + "</p>" + String("\n");
  html3 += "<p style='font-size: 0.8rem;'><a href='/menu'>MENU</a></p>" + String("\n");

  html3 += "</center></body></html>"; 

  return html3;
}
*/

void voiceTime()
{
  resumePlay = true;
  voiceTimeMillis = millis();
  char chbuf[30];      //Bufor komunikatu tekstowego zegara odczytywanego głosowo
  String time_s;
  struct tm timeinfo;
  char timeString[9];  // Bufor przechowujący czas w formie tekstowej
  if (!getLocalTime(&timeinfo, 1000))
  {
    Serial.println("debug voice time PL -> Nie udało się uzyskać czasu funkcji voice time");
    return;  // Zakończ funkcję, gdy nie udało się uzyskać czasu
  }

  snprintf(timeString, sizeof(timeString), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  time_s = String(timeString);
  int h = time_s.substring(0,2).toInt();
  snprintf(chbuf, sizeof (chbuf), "Jest godzina %i:%02i:00", h, time_s.substring(3,5).toInt());
  Serial.print("debug voice time PL -> ");
  Serial.println(chbuf);
  
  audio.connecttospeech(chbuf, "pl");
}

void voiceTimeEn()
{
  resumePlay = true;
  char chbuf[30];      //Bufor komunikatu tekstowego zegara odczytywanego głosowo
  String time_s;
  char am_pm[5] = "am.";
  struct tm timeinfo;
  char timeString[9];  // Bufor przechowujący czas w formie tekstowej

  if (!getLocalTime(&timeinfo, 1000)) 
  {
    Serial.println("debug voice time EN -> Nie udało się uzyskać czasu funkcji voice time");
    return;  // Zakończ funkcję, gdy nie udało się uzyskać czasu
  }

  snprintf(timeString, sizeof(timeString), "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  time_s = String(timeString);
  int h = time_s.substring(0,2).toInt();
  if(h > 12){h -= 12; strcpy(am_pm,"pm.");}
  //snprintf(chbuf, sizeof (chbuf), "Jest godzina %i:%02i", h, time_s.substring(3,5).toInt());
  sprintf(chbuf, "It is now %i%s and %i minutes", h, am_pm, time_s.substring(3,5).toInt());
  Serial.print("debug voice time EN -> ");
  Serial.println(chbuf);
  audio.connecttospeech(chbuf, "en");
}


void readRemoteControlConfig() 
{

  Serial.println("IR - Config, odczyt pliku konfiguracji pilota IR remote.txt z karty");
  
    // Tworzymy nazwę pliku
  String fileName = String("/remote.txt");

  // Sprawdzamy, czy plik istnieje
  if (!STORAGE.exists(fileName)) 
  {
    Serial.println("IR - Config, błąd, Plik konfiguracji pilota IR remote.txt nie istnieje.");
    configIrExist = false;
    return;
  }

  // Otwieramy plik w trybie do odczytu
  File configRemoteFile = STORAGE.open(fileName, FILE_READ);
  if (!configRemoteFile)  // jesli brak pliku to...
  {
    Serial.println("IR - Config, błąd, nie można otworzyć pliku konfiguracji pilota IR");
    configIrExist = false;
    return;
  }
  // Przechodzimy do odpowiedniego wiersza pliku
  configIrExist = true;
  int currentLine = 0;
  String configValue = "";
  while (configRemoteFile.available())
  {
    String line = configRemoteFile.readStringUntil(';'); //('\n');
    int lineStart = line.indexOf("0x") + 2;  // Szukamy miejsca, gdzie zaczyna wartość zmiennej
    if ((lineStart != -1)) //&& (currentLine != 0)) // Pomijamy pierwszą linijkę gdzie jest opis pliku
	  {
      configValue = line.substring(lineStart, lineStart + 5);  // Wyciągamy adres i komende
      configValue.trim();                      // Usuwamy białe znaki na początku i końcu
      Serial.print(" Odczytano kod: " + String(currentLine) + " wartosc:");
      Serial.print(configValue + " wartosc ConfigArray: ");
      configRemoteArray[currentLine] = strtol(configValue.c_str(), NULL, 16);
      Serial.println(configRemoteArray[currentLine], HEX);
    }
    currentLine++;
  }
  //Serial.print("Zamykamy plik config na wartosci currentLine:");
  //Serial.println(currentLine);
  configRemoteFile.close();  // Zamykamy plik po odczycie kodow pilota
}

void assignRemoteCodes()
{
  Serial.print("IR Config - Plik konfiguracji pilota istnieje, configIrExist: ");
  Serial.println(configIrExist);

  if ((noSDcard == false) && (configIrExist == true)) 
  {
  Serial.println("IR Config - Plik istnieje, przypisuje wartosci z pliku Remote.txt");
  rcCmdVolumeUp = configRemoteArray[0];    // Głosnosc +
  rcCmdVolumeDown = configRemoteArray[1];  // Głośnosc -
  rcCmdArrowRight = configRemoteArray[2];  // strzałka w prawo - nastepna stacja
  rcCmdArrowLeft = configRemoteArray[3];   // strzałka w lewo - poprzednia stacja  
  rcCmdArrowUp = configRemoteArray[4];     // strzałka w góre - lista stacji krok do gory
  rcCmdArrowDown = configRemoteArray[5];   // strzałka w dół - lista stacj krok na dół
  rcCmdBack = configRemoteArray[6]; 	     // Przycisk powrotu
  rcCmdOk = configRemoteArray[7];          // Przycisk Ent - zatwierdzenie stacji
  rcCmdSrc = configRemoteArray[8];         // Przełączanie źródła radio, odtwarzacz
  rcCmdMute = configRemoteArray[9];        // Wyciszenie dzwieku
  rcCmdAud = configRemoteArray[10];        // Equalizer dzwieku
  rcCmdDirect = configRemoteArray[11];     // Janość ekranu, dwa tryby 1/16 lub pełna janość     
  rcCmdBankMinus = configRemoteArray[12];  // Wysweitla wybór banku
  rcCmdBankPlus = configRemoteArray[13];   // Wysweitla wybór banku
  rcCmdRed = configRemoteArray[14];        // Przełacza ładowanie banku kartaSD - serwer GitHub w menu bank
  rcCmdPower = configRemoteArray[14];      // PRZYCISK POWER Z KODEM Czerwonej Słuchawki
  rcCmdGreen = configRemoteArray[15];      // VU wyłaczony, VU tryb 1, VU tryb 2, zegar
  rcCmdKey0 = configRemoteArray[16];       // Przycisk "0"
  rcCmdKey1 = configRemoteArray[17];       // Przycisk "1"
  rcCmdKey2 = configRemoteArray[18];       // Przycisk "2"
  rcCmdKey3 = configRemoteArray[19];       // Przycisk "3"
  rcCmdKey4 = configRemoteArray[20];       // Przycisk "4"
  rcCmdKey5 = configRemoteArray[21];       // Przycisk "5"
  rcCmdKey6 = configRemoteArray[22];       // Przycisk "6"
  rcCmdKey7 = configRemoteArray[23];       // Przycisk "7"
  rcCmdKey8 = configRemoteArray[24];       // Przycisk "8"
  rcCmdKey9 = configRemoteArray[25];       // Przycisk "9"
  // === DODATKOWE PRZYCISKI (26-61) ===
  rcCmdYellow = configRemoteArray[26];     // Toggle Analyzer ON/OFF
  rcCmdBlue = configRemoteArray[27];       // Nagrywanie (REC/BLUE)
  rcCmdBT = configRemoteArray[28];         // Strona BT / SDPlayer style
  rcCmdPLAY = configRemoteArray[29];       // SDPlayer: PLAY
  rcCmdSTOP = configRemoteArray[30];       // SDPlayer: STOP
  rcCmdREV = configRemoteArray[31];        // SDPlayer: Przewiń -10s / Poprzedni utwór (double-click)
  rcCmdRER = configRemoteArray[32];        // SDPlayer: Przewiń +10s / Następny utwór (double-click)
  rcCmdTV = configRemoteArray[33];         // Zarezerwowane
  rcCmdRADIO = configRemoteArray[34];      // SDPlayer: Wyjście do radia
  rcCmdFILES = configRemoteArray[35];      // SDPlayer: Lista plików
  rcCmdWWW = configRemoteArray[36];        // Zarezerwowane
  rcCmdGAZE = configRemoteArray[37];       // Zarezerwowane
  rcCmdEPG = configRemoteArray[38];        // Zarezerwowane
  rcCmdMENU = configRemoteArray[39];       // SDPlayer: Menu/lista (alias FILES)
  rcCmdEXIT = configRemoteArray[40];       // SDPlayer: Wyjście do radia
  rcCmdREC = configRemoteArray[41];        // Nagrywanie strumienia (alias BLUE)
  rcCmdKOL = configRemoteArray[42];        // Moduł BT: toggle ON/OFF
  rcCmdCHAT = configRemoteArray[43];       // Moduł BT: wyświetl status
  rcCmdPAUSE = configRemoteArray[44];      // SDPlayer: Pauza/wznowienie
  rcCmdREDLEFT = configRemoteArray[45];    // Moduł BT: restart
  rcCmdGREENL = configRemoteArray[46];     // Moduł BT: test połączenia
  rcCmdCHPlus = configRemoteArray[47];     // OLED: jasność +20
  rcCmdCHMinus = configRemoteArray[48];    // OLED: jasność -20
  rcCmdINFO = configRemoteArray[49];       // Moduł BT: informacje
  rcCmdPOWROT = configRemoteArray[50];     // Moduł BT: strona /bt
  rcCmdHELP = configRemoteArray[51];       // Toggle SDPlayer ON/OFF
  rcCmdPIP = configRemoteArray[52];        // Przełączenie EQ3 ↔ EQ16
  rcCmdMINIMA = configRemoteArray[53];     // SDPlayer: następny styl
  rcCmdMAXIMA = configRemoteArray[54];     // SDPlayer: zmiana stylu
  rcCmdWELEMENT = configRemoteArray[55];   // SDPlayer: PAUSE/PLAY toggle
  rcCmdWINPOD = configRemoteArray[56];     // SDPlayer: STOP
  rcCmdGLOS = configRemoteArray[57];       // Moduł BT: otwiera stronę /bt
  }
  
  else if ((noSDcard == true) || (configIrExist == false)) // Jesli nie ma karty SD przypisujemy standardowe wartosci dla pilota Kenwood RC-406
  {
    Serial.println("IR Config - BRAK konfiguracji pilota, przypisuje wartosci domyslne");
    rcCmdVolumeUp = 0xB914;   // Głosnosc +
    rcCmdVolumeDown = 0xB915; // Głośnosc -
    rcCmdArrowRight = 0xB90B; // strzałka w prawo - nastepna stacja
    rcCmdArrowLeft = 0xB90A;  // strzałka w lewo - poprzednia stacja  
    rcCmdArrowUp = 0xB987;    // strzałka w góre - lista stacji krok do gory
    rcCmdArrowDown = 0xB986;  // strzałka w dół - lista stacj krok na dół
    rcCmdBack = 0xB985;	   	  // Przycisk powrotu
    rcCmdOk = 0xB90E;         // Przycisk Ent - zatwierdzenie stacji
    rcCmdSrc = 0xB913;        // Przełączanie źródła radio, odtwarzacz
    rcCmdMute = 0xB916;       // Wyciszenie dzwieku
    rcCmdAud = 0xB917;        // Equalizer dzwieku
    rcCmdDirect = 0xB90F;     // Janość ekranu, dwa tryby 1/16 lub pełna janość     
    rcCmdBankMinus = 0xB90C;  // Wysweitla wybór banku
    rcCmdBankPlus = 0xB90D;   // Wysweitla wybór banku
    rcCmdRed = 0xB988;        // Przycisk czerwonej sluchawki - obecnie power
    rcCmdPower = 0xB988;      // Przycisk power - brak obecnie na pilocie
    rcCmdGreen = 0xB992;      // Przycisk zielonej sluchawki - obecnie sleep
    rcCmdYellow = 0xB993;     // Przycisk żółty - toggle Analyzer
    rcCmdBlue = 0xB994;       // Przycisk niebieski - REC (nagrywanie radia)
    rcCmdKey0 = 0xB900;       // Przycisk "0"
    rcCmdKey1 = 0xB901;       // Przycisk "1"
    rcCmdKey2 = 0xB902;       // Przycisk "2"
    rcCmdKey3 = 0xB903;       // Przycisk "3"
    rcCmdKey4 = 0xB904;       // Przycisk "4"
    rcCmdKey5 = 0xB905;       // Przycisk "5"
    rcCmdKey6 = 0xB906;       // Przycisk "6"
    rcCmdKey7 = 0xB907;       // Przycisk "7"
    rcCmdKey8 = 0xB908;       // Przycisk "8"
    rcCmdKey9 = 0xB909;       // Przycisk "9"
    // === DODATKOWE PRZYCISKI - WARTOŚCI DOMYŚLNE ===
    // Kenwood RC-406 nie ma tych przycisków - użyj uniwersalnych kodów NEC lub ustaw na 0 jeśli brak
    rcCmdBT = 0xB995;         // Strona BT (możesz zmienić na kod swojego pilota)
    rcCmdPLAY = 0xB996;       // SDPlayer: PLAY
    rcCmdSTOP = 0xB997;       // SDPlayer: STOP
    rcCmdREV = 0xB998;        // SDPlayer: REV (przewiń -10s)
    rcCmdRER = 0xB999;        // SDPlayer: RER (przewiń +10s)
    rcCmdTV = 0x0000;         // Zarezerwowane (nie przypisane)
    rcCmdRADIO = 0xB99A;      // SDPlayer: Wyjście do radia
    rcCmdFILES = 0xB99B;      // SDPlayer: Lista plików
    rcCmdWWW = 0x0000;        // Zarezerwowane (nie przypisane)
    rcCmdGAZE = 0x0000;       // Zarezerwowane (nie przypisane)
    rcCmdEPG = 0x0000;        // Zarezerwowane (nie przypisane)
    rcCmdMENU = 0xB99C;       // SDPlayer: Menu (alias FILES)
    rcCmdEXIT = 0xB99D;       // SDPlayer: Wyjście (alias RADIO)
    rcCmdREC = 0xB994;        // Nagrywanie (alias BLUE)
    rcCmdKOL = 0xB99E;        // Moduł BT: toggle ON/OFF
    rcCmdCHAT = 0xB99F;       // Moduł BT: status
    rcCmdPAUSE = 0xB9A0;      // SDPlayer: Pauza
    rcCmdREDLEFT = 0xB9A1;    // Moduł BT: restart
    rcCmdGREENL = 0xB9A2;     // Moduł BT: test
    rcCmdCHPlus = 0xB9A3;     // OLED: jasność +
    rcCmdCHMinus = 0xB9A4;    // OLED: jasność -
    rcCmdINFO = 0xB9A5;       // Moduł BT: info
    rcCmdPOWROT = 0xB9A6;     // Moduł BT: strona /bt
    rcCmdHELP = 0xB9A7;       // Toggle SDPlayer
    rcCmdPIP = 0xB9A8;        // EQ3 ↔ EQ16
    rcCmdMINIMA = 0xB9A9;     // SDPlayer: następny styl
    rcCmdMAXIMA = 0xB9AA;     // SDPlayer: zmiana stylu
    rcCmdWELEMENT = 0xB9AB;   // SDPlayer: PAUSE/PLAY
    rcCmdWINPOD = 0xB9AC;     // SDPlayer: STOP
    rcCmdGLOS = 0xB9AD;       // Moduł BT: strona /bt
  }
}

void saveRemoteControlConfig()
{
  Serial.println("RC - Config, zapis pliku konfiguracji pilota IR remote.txt");

  // Sprawdź, czy plik istnieje
  if (STORAGE.exists("/remote.txt")) 
  {
    Serial.println("RC - Config, plik remote.txt już istnieje.");
    
    // Otwórz plik do zapisu i nadpisz aktualną wartość konfiguracji
    myFile = STORAGE.open("/remote.txt", FILE_WRITE);
    if (myFile)
    {
      myFile.println("#### Evo Web Radio Config File - IR Remote ####");
      myFile.printf("%s = 0x%04X;\n", "cmdVolumeUp", configRemoteArray[0]);
      myFile.printf("%s = 0x%04X;\n", "cmdVolumeDown", configRemoteArray[1]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowRight", configRemoteArray[2]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowLeft", configRemoteArray[3]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowUp", configRemoteArray[4]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowDown", configRemoteArray[5]);
      myFile.printf("%s = 0x%04X;\n", "cmdBack", configRemoteArray[6]);
      myFile.printf("%s = 0x%04X;\n", "cmdOk", configRemoteArray[7]);
      myFile.printf("%s = 0x%04X;\n", "cmdSrc", configRemoteArray[8]);
      myFile.printf("%s = 0x%04X;\n", "cmdMute", configRemoteArray[9]);
      myFile.printf("%s = 0x%04X;\n", "cmdAud", configRemoteArray[10]);
      myFile.printf("%s = 0x%04X;\n", "cmdDirect", configRemoteArray[11]);
      myFile.printf("%s = 0x%04X;\n", "cmdBankMinus", configRemoteArray[12]);
      myFile.printf("%s = 0x%04X;\n", "cmdBankPlus", configRemoteArray[13]);
      myFile.printf("%s = 0x%04X;\n", "cmdRed", configRemoteArray[14]);
      myFile.printf("%s = 0x%04X;\n", "cmdGreen", configRemoteArray[15]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey0", configRemoteArray[16]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey1", configRemoteArray[17]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey2", configRemoteArray[18]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey3", configRemoteArray[19]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey4", configRemoteArray[20]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey5", configRemoteArray[21]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey6", configRemoteArray[22]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey7", configRemoteArray[23]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey8", configRemoteArray[24]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey9", configRemoteArray[25]);
      myFile.close();
      Serial.println("RC - Config, aktualizacja remote.txt udana");
    }
    else
    {
      Serial.println("RC - Config, blad podczas otwierania pliku remote.txt");
    }
  }
  else
  {
    Serial.println("RC - Config, plik remote.txt nie istnieje. Tworzenie...");

    // Utwórz plik i zapisz w nim aktualne wartości konfiguracji
    myFile = STORAGE.open("/remote.txt", FILE_WRITE);
    if (myFile)
    {
      myFile.println("#### Evo Web Radio Config File - IR Remote ####");
      myFile.printf("%s = 0x%04X;\n", "cmdVolumeUp", configRemoteArray[0]);
      myFile.printf("%s = 0x%04X;\n", "cmdVolumeDown", configRemoteArray[1]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowRight", configRemoteArray[2]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowLeft", configRemoteArray[3]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowUp", configRemoteArray[4]);
      myFile.printf("%s = 0x%04X;\n", "cmdArrowDown", configRemoteArray[5]);
      myFile.printf("%s = 0x%04X;\n", "cmdBack", configRemoteArray[6]);
      myFile.printf("%s = 0x%04X;\n", "cmdOk", configRemoteArray[7]);
      myFile.printf("%s = 0x%04X;\n", "cmdSrc", configRemoteArray[8]);
      myFile.printf("%s = 0x%04X;\n", "cmdMute", configRemoteArray[9]);
      myFile.printf("%s = 0x%04X;\n", "cmdAud", configRemoteArray[10]);
      myFile.printf("%s = 0x%04X;\n", "cmdDirect", configRemoteArray[11]);
      myFile.printf("%s = 0x%04X;\n", "cmdBankMinus", configRemoteArray[12]);
      myFile.printf("%s = 0x%04X;\n", "cmdBankPlus", configRemoteArray[13]);
      myFile.printf("%s = 0x%04X;\n", "cmdRed", configRemoteArray[14]);
      myFile.printf("%s = 0x%04X;\n", "cmdGreen", configRemoteArray[15]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey0", configRemoteArray[16]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey1", configRemoteArray[17]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey2", configRemoteArray[18]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey3", configRemoteArray[19]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey4", configRemoteArray[20]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey5", configRemoteArray[21]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey6", configRemoteArray[22]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey7", configRemoteArray[23]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey8", configRemoteArray[24]);
      myFile.printf("%s = 0x%04X;\n", "cmdKey9", configRemoteArray[25]);
      myFile.close();
      Serial.println("RC - Config, utworzono remote.txt");
    }
    else
    {
      Serial.println("RC - Config, blad podczas tworzenia pliku remote.txt");
    }
  }
}

void listFiles(String path, String &html) 
{
    File root = STORAGE.open(path);
    if (!root) {
        Serial.println("Nie można otworzyć karty SD!");
        return;
    }

    // Przeglądanie plików w katalogu
    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            html += "<tr>";

            html += "<td>" + String(file.name()) + "</td>";
            html += "<td>" + String(file.size()) + "</td>";
            html += "<td>" + String("\n");
            
            // Kasowanie pliku
            html += "<form action=\"/delete\" method=\"POST\" style=\"display:inline; margin-right: 35px;\">";
            html += "<input type=\"hidden\" name=\"filename\" value=\"" + String(file.name()) + "\">";
            html += "<input type=\"submit\" value=\"Delete\">";
            html += "</form>" + String("\n");

            // Podglad pliku
            html += "<form action=\"/view\" method=\"GET\" style=\"display:inline;\">";
            html += "<input type=\"hidden\" name=\"filename\" value=\"" + String(file.name()) + "\">";
            html += "<input type=\"submit\" value=\"View\">";
            html += "</form>" + String("\n");
            
            // Edycja pliku
            html += "<form action=\"/edit\" method=\"GET\" style=\"display:inline;\">";
            html += "<input type=\"hidden\" name=\"filename\" value=\"" + String(file.name()) + "\">";
            html += "<input type=\"submit\" value=\"Edit\">";
            html += "</form>" + String("\n");            
            
            // Pobieranie pliku
            html += "<form action=\"/download\" method=\"GET\" style=\"display:inline;\">";
            html += "<input type=\"hidden\" name=\"filename\" value=\"" + String(file.name()) + "\">";
            html += "<input type=\"submit\" value=\"Download\">";
            html += "</form>" + String("\n");

            html += "</td>";
            html += "</tr>" + String("\n");
        }
        file = root.openNextFile();
    }
    root.close();
}


void saveTimezone(String timezone) 
{
  if (STORAGE.exists("/timezone.txt")) 
  {
    Serial.println("debug SD -> Plik timezone.txt już istnieje.");

    // Otwórz plik do zapisu i nadpisz aktualną wartość flitrów equalizera
    myFile = STORAGE.open("/timezone.txt", FILE_WRITE);
    if (myFile) 
    {
      myFile.println(timezone);
      myFile.close();
      Serial.println("debug SD -> Aktualizacja timezone.txt na karcie SD");
    } 
    else 
    {
      Serial.println("debug SD -> Błąd podczas otwierania pliku timezone.txt");
    }
  } 
  else 
  {
    Serial.println("debug SD -> Plik timezone.txt nie istnieje. Tworzenie...");

    // Utwórz plik i zapisz w nim aktualną wartość głośności
    myFile = STORAGE.open("/timezone.txt", FILE_WRITE);
    if (myFile) 
    {
      myFile.println(timezone);
      myFile.close();
      Serial.println("debug SD -> Utworzono i zapisano timezone.txt na karcie SD");
    } 
    else 
    {
      Serial.println("debug SD -> Błąd podczas tworzenia pliku timezone.txt");
    }
  }
}

void readTimezone() 
{
  // Sprawdzamy czy plik  istnieje
  if (STORAGE.exists("/timezone.txt")) 
  {
    File myFile = STORAGE.open("/timezone.txt", FILE_READ);

    // Plik istnieje, ale nie udało się go otworzyć
    if (!myFile) 
    {
      timezone = "CET-1CEST,M3.5.0/2,M10.5.0/3";
      Serial.println("debug SD -> Blad pliku timezone.txt, ustawiam strefe:" + timezone);
      return;
    }

    // Plik udało się otworzyć
    timezone = myFile.readString();
    timezone.trim();
    myFile.close();
    Serial.println("debug SD -> Odczytano timezone.txt, strefa:" + timezone);
  } 
  else 
  {
    // Plik nie istnieje ustawiamy wartosc domyslna
    timezone = "CET-1CEST,M3.5.0/2,M10.5.0/3";
    saveTimezone(timezone);
    Serial.println("debug SD -> Pliku timezone.txt, nie istnieje");
  }
}


//####################################################################################### OBSŁUGA POWER OFF / SLEEP ####################################################################################### //

// ---- FUNKCJA WYSWIETLA NAPIS NA SRODKU OLEDa duzą czcionka -----
void displayCenterBigText(String stringText, int stringText_Y)
{
  u8g2.setContrast(displayBrightness); // Ustawiamy maksymalną jasnosc
  u8g2.setFont(u8g2_font_fub14_tr);
  int stringTextWidth = u8g2.getStrWidth(stringText.c_str()); // Liczy pozycje aby wyswietlic TEXT na środku
  int stringText_X = (SCREEN_WIDTH - stringTextWidth) / 2;
  u8g2.clearBuffer();
  u8g2.drawStr(stringText_X, stringText_Y, stringText.c_str());     
  u8g2.sendBuffer();
  //Serial.print("debug sleep -> Chart Max Height: ");       
  //Serial.println(u8g2.getMaxCharHeight(stringText.substring(0,1).c_str()));
  Serial.println(u8g2.getMaxCharHeight());
}

void sleepTimer()
{
  if (f_debug_on) {Serial.println("debug sleep -> Sleep timer check");}
  Serial.println("debug sleep -> Sleep timer check");       
  if (f_sleepTimerOn)
  {
    sleepTimerValueCounter--;
    //if (f_debug_on) 
    //{
      Serial.print("debug sleep -> Sleep timer value counter: ");       
      Serial.println(sleepTimerValueCounter);
    //}
    
    if (sleepTimerValueCounter == 0) 
    {   
      f_sleepTimerOn = false;
      displayActive = true;
      displayStartTime = millis();
      timeDisplay = false;
             
      displayCenterBigText("SLEEP",36);
      //Fadeout volume
      
      ir_code = rcCmdPower; // Power off - udejmy kod pilota
      bit_count = 32;
      calcNec();          // Przeliczamy kod pilota na pełny oryginalny kod NEC

    }
  }
}

void powerOffAnimation() 
{
  int width  = u8g2.getDisplayWidth();
  int height = u8g2.getDisplayHeight();

  // Animacja "zamykania" ekranu od góry i dołu
  for (int i = 0; i < height / 2; i += 2) 
  {
    u8g2.clearBuffer();

    // Rysujemy prostokąt w środku ekranu, który się kurczy
    int boxHeight = height - 2 * i;
    u8g2.drawBox(0, i, width, boxHeight);

    u8g2.sendBuffer();
    delay(20); // czas między krokami animacji
  }

  // Po zamknięciu — "zanikanie" linii środkowej
  for (int t = 0; t < 3; t++) 
  {
    u8g2.clearBuffer();
    u8g2.drawHLine(0, height / 2, width);
    u8g2.sendBuffer();
    delay(50);
  }

  // Całkowite wygaszenie
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}

void displaySleepTimer()
{
  f_displaySleepTime = true;
  displayActive = true;
  displayStartTime = millis();
  timeDisplay = false;
  //u8g2.clearBuffer();
  //u8g2.setFont(u8g2_font_fub14_tf);
  String stringText ="";
  if (sleepTimerValueCounter != 0) {stringText = SLEEP_STRING + String(sleepTimerValueCounter) + SLEEP_STRING_VAL;}
  else {stringText = String(SLEEP_STRING) + SLEEP_STRING_OFF;}
  displayCenterBigText(stringText,36);  // Text, Y cordinate value
  //u8g2.sendBuffer();
}

void sleepTimerSet()
{
  /* // DEBUG
  Serial.print("debug sleep -> f_displaySleepTime: ");       
  Serial.println(f_displaySleepTime);  

  Serial.print("debug sleep -> f_displaySleepTimeSet: ");       
  Serial.println(f_displaySleepTimeSet); 

  Serial.print("debug sleep -> f_sleepTimerOn: ");       
  Serial.println(f_sleepTimerOn); 

  Serial.print("debug sleep -> sleepTimerValueSet: ");       
  Serial.println(sleepTimerValueSet);

  Serial.print("debug sleep -> sleepTimerValueCounter: ");       
  Serial.println(sleepTimerValueCounter);   
  */
  
  if (f_displaySleepTime)
  {  
    if (f_displaySleepTimeSet)
    {
      sleepTimerValueSet = 0;
      sleepTimerValueCounter = 0;
      f_displaySleepTimeSet = false;
      f_sleepTimerOn = false;
      f_displaySleepTime = false;
    }
    else
    {
      // Odliczamy od MAX w doł w z krokiem STEP
      if (sleepTimerValueSet == 0) {sleepTimerValueSet = sleepTimerValueMax;}
      else {sleepTimerValueSet = sleepTimerValueSet - sleepTimerValueStep;}

      // Odliczamy od 0 w góre z krokiem step
      //sleepTimerValueSet = sleepTimerValueSet + sleepTimerValueStep;
      if (sleepTimerValueSet > sleepTimerValueMax) {sleepTimerValueSet = 0;}
      sleepTimerValueCounter = sleepTimerValueSet;
        
    }

    if (sleepTimerValueSet !=0)
    {
      f_sleepTimerOn = true;
      timer3.attach(60, sleepTimer);      // Ustawiamy flage działania funkcji sleep i załaczamy Timer 3
    }  
    else 
    {
      f_sleepTimerOn = false; 
      f_displaySleepTimeSet = false;
      timer3.detach();
    }                                      // Zerujemy flage działania funkcji sleep i odpinamy Timer 3 aby w sleepie nie działał
  }
  else 
  {
    if (f_displaySleepTimeSet == 0) // Jezeli nie mamy wyswietlonego menu Sleep Timer i sam Timer jest OFF to pierwsze nacisniejcie ustawia go na ValueMAX zamiast wyswietlac stan obecny czyli OFF
    {
      // Odliczamy od MAX w doł w z krokiem STEP
        if (sleepTimerValueSet == 0) {sleepTimerValueSet = sleepTimerValueMax;}
        else {sleepTimerValueSet = sleepTimerValueSet - sleepTimerValueStep;}

        // Odliczamy od 0 w góre z krokiem step
        //sleepTimerValueSet = sleepTimerValueSet + sleepTimerValueStep;
        if (sleepTimerValueSet > sleepTimerValueMax) {sleepTimerValueSet = 0;}
        sleepTimerValueCounter = sleepTimerValueSet;

        if (sleepTimerValueSet !=0)
      {
        f_sleepTimerOn = true;
        timer3.attach(60, sleepTimer);      // Ustawiamy flage działania funkcji sleep i załaczamy Timer 3
      }
      else 
      {
        f_sleepTimerOn = false; 
        f_displaySleepTimeSet = false;
        timer3.detach();
      }
    }    
  }

  displaySleepTimer();                   // Obsługa komunikatu na OLED 
}

void powerOffClock()
{
  //u8g2.clearBuffer();
  u8g2.setPowerSave(0);
  u8g2.setContrast(dimmerSleepDisplayBrightness);
  u8g2.setDrawColor(1);  // Ustaw kolor na biały

  // Struktura przechowująca informacje o czasie
  struct tm timeinfo;

  if (!getLocalTime(&timeinfo, 500)) 
  {
    // Wyświetl komunikat o niepowodzeniu w pobieraniu czasu
	  Serial.println("debug time powerOffClock -> Nie udało się uzyskać czasu");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7Segments_26x42_mn);
    u8g2.drawStr(40, 52, "--:--");
    u8g2.sendBuffer();
    return;  // Zakończ funkcję, gdy nie udało się uzyskać czasu
  }
      
  
  seconds2nextMinute = 60 - timeinfo.tm_sec;
  micros2nextMinute = seconds2nextMinute * 1000000ULL - (esp_timer_get_time() % 1000000ULL);
  
  // ---------- ZABEZPIECZENIE ----------
  if (micros2nextMinute < 1000) micros2nextMinute = 1000;
  // -----------------------------------



  //showDots = (timeinfo.tm_sec % 2 == 0); // Parzysta sekunda = pokazuj dwukropek

  // Konwertuj godzinę, minutę i sekundę na stringi w formacie "HH:MM:SS"
  char timeString[9];  // Bufor przechowujący czas w formie tekstowej
  u8g2.setFont(u8g2_font_7Segments_26x42_mn);
  snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
  u8g2.clearBuffer();
  u8g2.drawStr(53, 52, timeString);
  u8g2.sendBuffer();
}

// ---- Aktualizacja stanu pinu głośników (wzmacniacz) ----
void updateSpeakersPin() {
  if (!speakersEnabled || volumeMute || f_powerOff) {
    digitalWrite(SPEAKERS_PIN, LOW);   // Wzmacniacz wyłączony
  } else {
    digitalWrite(SPEAKERS_PIN, HIGH);  // Wzmacniacz włączony
  }
}

void powerOff()
{
  
  // ---- WYSWIETLAMY NAPIS POWER OFF na srodku ----
  displayCenterBigText("POWER OFF",36); // Tekst, pozycja Y
  
  // ---- ZAMYKAMY CALY OBIEKT AUDIO ----
  ws.closeAll();
  if (!volumeMute && f_volumeFadeOn) {volumeFadeOut(volumeSleepFadeOutTime);}
  audio.setVolume(0);
  audio.stopSong();
  delay(1000);
  
  // ---- ZAPIS OSTATNIEGO NR.STACJI, NR.BANKU, POZIOMU VOLUME jesli funkcja saveAlwasy wylaczona ----
  if (!f_saveVolumeStationAlways) {saveVolumeOnSD(); saveStationOnSD();}
  
  // ---- ZAPIS KONFIGURACJI config.txt ----
  Serial.println("=== POWER OFF: Zapisywanie konfiguracji ===");
  
  #ifdef AUTOSTORAGE
    // KRYTYCZNE: Sprawdz czy storage jest dostepny
    if (_storage == nullptr) {
      Serial.println("BLAD KRYTYCZNY: _storage jest NULL - pomijam zapis!");
    } else {
      // Test czy storage jest rzeczywiscie zamontowany
      Serial.printf("Test storageTextName: %s\n", storageTextName);
      
      // Sprobuj sprawdzic czy root directory istnieje
      bool storageOK = false;
      if (useSD) {
        // Test SD card
        File root = SD.open("/");
        if (root) {
          Serial.println("SD card jest dostepna");
          root.close();
          storageOK = true;
        } else {
          Serial.println("BLAD: SD card nie odpowiada!");
        }
      } else {
        // Test LittleFS
        File root = LittleFS.open("/");
        if (root) {
          Serial.println("LittleFS jest dostepny");
          root.close();
          storageOK = true;
        } else {
          Serial.println("BLAD: LittleFS nie odpowiada!");
        }
      }
      
      if (storageOK) {
        // Storage OK - zapisz konfiguracje
        Serial.printf("Wartosci przed zapisem: brightness=%d, analyzerEnabled=%d, analyzerStyles=%d\n",
                      displayBrightness, analyzerEnabled, analyzerStyles);
        saveConfig();
        Serial.println("debug Power -> Zapisano config.txt");
        
        // ---- ZAPIS USTAWIEŃ ANALIZATORA analyzer.cfg ----
        analyzerStyleSave();
        Serial.println("debug Power -> Zapisano analyzer.cfg");
      } else {
        Serial.println("BLAD: Storage nie jest zamontowany - POMIJAM ZAPIS!");
      }
    }
  #else
    // Bez AUTOSTORAGE - standardowy zapis
    Serial.printf("Wartosci przed zapisem: brightness=%d, analyzerEnabled=%d, analyzerStyles=%d\n",
                  displayBrightness, analyzerEnabled, analyzerStyles);
    saveConfig();
    Serial.println("debug Power -> Zapisano config.txt");
    analyzerStyleSave();
    Serial.println("debug Power -> Zapisano analyzer.cfg");
  #endif
  
  // ---- ANIMACJA POWER OFF, jesli wlaczona ----
  if (f_powerOffAnimation) {Serial.println("debug Power -> Animacja PowerOff"); powerOffAnimation(); u8g2.clearBuffer();}

  // ---- ZEGAR W TRYBIE POWER OFF pierwsze wywloane celem synch. ntp----
  if (f_displayPowerOffClock)
  {
    u8g2.setPowerSave(0);  // Jesli zegar wlaczony to nie przelaczamy OLEDa w tryb power save
    Serial.println("debug Power -> Zegar ON");
    powerOffClock();
  } 
  // Wyłącz ekran jesli zegar ma byc wyłaczony
  else 
  {
    u8g2.setPowerSave(1);
  } 
  
  delay(25);
  
  // ----- ZERUJEMY SLEEP TIMER -----
  sleepTimerValueSet = 0;
  sleepTimerValueCounter = 0;
  f_displaySleepTimeSet = false;
  f_sleepTimerOn = false;
  f_displaySleepTime = false;
  timer3.detach();

  f_powerOff = true; 
  Serial.println("debug Power -> Usypiam ESP, power off");

  // -------------- WYŁACZAMY PERYFERIA ------------------
  WiFi.mode(WIFI_OFF);
  btStop();
  // -------------- WYŁACZAMY LED na module ESP32 ------------------
  pinMode(TX, OUTPUT);
  pinMode(RX, OUTPUT);
  digitalWrite(TX, HIGH);
  digitalWrite(RX, HIGH);
  
  Serial.end();
    
  // ---- USTAWIAMY PRZYCISKI NA ENKODERACH do POWER ON -----
  
  pinMode(SW_PIN2, INPUT_PULLUP);
  
  #ifdef SW_POWER
  pinMode(SW_POWER, INPUT_PULLUP);
  #endif
  
  #ifdef twoEncoders
  pinMode(SW_PIN1, INPUT_PULLUP);
  #endif

  // --------LED STANDBY (przygotowanie pod przekaźnik) ----------------
  ledBlinker.detach(); // Zatrzymaj miganie (jeśli było aktywne)
  pinMode(STANDBY_LED, OUTPUT);
  digitalWrite(STANDBY_LED, HIGH); // LED Standby ON = tryb Power OFF
  //digitalWrite(STANDBY_LED, LOW); // Dla LED właczonego w trybie Power ON

  // -------- SPEAKERS PIN: wyłącz głośniki przy Power OFF --------
  pinMode(SPEAKERS_PIN, OUTPUT);
  digitalWrite(SPEAKERS_PIN, LOW); // Głośniki OFF w trybie Power OFF

    
  // ---------------- USTAWIAMY WAKEUP ----------------    
  #ifdef twoEncoders
  gpio_wakeup_enable((gpio_num_t)SW_PIN1, GPIO_INTR_LOW_LEVEL);
  #endif
    
  #ifdef SW_POWER
  gpio_wakeup_enable((gpio_num_t)SW_POWER, GPIO_INTR_LOW_LEVEL);
  #endif
  
  gpio_wakeup_enable((gpio_num_t)SW_PIN2, GPIO_INTR_LOW_LEVEL);

  //gpio_wakeup_enable((gpio_num_t)recv_pin, GPIO_INTR_LOW_LEVEL);
  

  esp_sleep_enable_gpio_wakeup();
  
  while (f_powerOff)
  {
    // ---------------- RESET STANÓW IR ----------------
    bit_count = 0;
    ir_code = 0;
    detachInterrupt(digitalPinToInterrupt(recv_pin)); // WYLACZAMY PRZERWANIE
    delay(10);

    // ---------------- DEZAKTYWACJA PERYFERIOW i AKTYWACJA ZEGARA (jesli właczony) ----------------
    if (f_displayPowerOffClock) 
    {
      powerOffClock();
      esp_sleep_enable_timer_wakeup(micros2nextMinute); 
    }
    else 
    {
      u8g2.setPowerSave(1);
    }
      
    // ---------------- USTAWIAMY WAKEUP ----------------    
    gpio_wakeup_enable((gpio_num_t)recv_pin, GPIO_INTR_LOW_LEVEL);

    // ---------------- SLEEP ----------------
    delay(10);
    esp_light_sleep_start();

    // ------------- TU SIĘ OBUDZIMY ----------------

    // Jesli obudził nas timer to robimy aktulizacje zegera i wracamy na początek petli while
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_TIMER)
    {
      if (f_displayPowerOffClock) {powerOffClock();}
      delay(5);
      continue;   // wraca do while – ESP idzie znowu spać
    }

    // Konfiguracja przerwania IR
    attachInterrupt(digitalPinToInterrupt(recv_pin), pulseISR, CHANGE);
    delay(100); // stabilizacja
                
    // Analiza IR
    ir_code = reverse_bits(ir_code, 32);
    uint8_t CMD  = (ir_code >> 16) & 0xFF;
    uint8_t ADDR = ir_code & 0xFF;
    ir_code = (ADDR << 8) | CMD;
  
    // Warunek POWER UP
    #ifdef twoEncoders
    bool POWER_UP =
        (bit_count == 32 && ir_code == rcCmdPower && f_powerOff) ||
        (digitalRead(SW_POWER) == LOW) ||
        (digitalRead(SW_PIN1) == LOW) ||
        (digitalRead(SW_PIN2) == LOW);
    #else
    bool POWER_UP =
        (bit_count == 32 && ir_code == rcCmdPower && f_powerOff) ||
        (digitalRead(SW_POWER) == LOW) ||
        (digitalRead(SW_PIN2) == LOW);
    #endif    

    // ------------------ WSTAEMY po poprawny rozpoznaiu kodu (lub nie, else) -----------------
    if (POWER_UP)
    {
      detachInterrupt(digitalPinToInterrupt(recv_pin));
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    
      f_powerOff = false;
      displayActive = true;

      u8g2.setPowerSave(0);
      displayCenterBigText("POWER ON", 36);
      
      // ------------ LED STANDBY -----------
      pinMode(STANDBY_LED, OUTPUT);
      //digitalWrite(STANDBY_LED, HIGH); // Dla LED właczonego w trybie Power ON
      digitalWrite(STANDBY_LED, LOW); // Dla LED wyłączonego w trybie PowerON a właczonego w trybie Standby

      // -------- SPEAKERS PIN: przywróć głośniki po Power ON --------
      pinMode(SPEAKERS_PIN, OUTPUT);
      updateSpeakersPin();
      Serial.println("[POWER] SPEAKERS_PIN ustawiony po Power ON");
      
      delay(800);

      // Twardy restart
      REG_WRITE(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_SW_SYS_RST);
      break;
    }

    // Nie to nie POWER_UP -> rozpinamy przerwanie IR
    detachInterrupt(digitalPinToInterrupt(recv_pin));
  }
}

// ---- ŁADOWANIE LOGO STARTOWEGO jesli plik istnieje ----
bool loadXBM(const char* filename) 
{
    File logo = STORAGE.open(filename, FILE_READ);
    if (!logo) return false;
  
    if (logo.size() == 0) {
        logo.close();
        return false;
    }

    int index = 0;

    while (logo.available() && index < LOGO_SIZE) 
    {
        char c = logo.read();

        if (c == '0' && logo.peek() == 'x') 
        {
            logo.read(); // skip 'x'
            
            char hex1 = logo.read();
            char hex2 = logo.read();

            char hexStr[3] = {hex1, hex2, 0};
            logo_bits[index++] = strtol(hexStr, NULL, 16);
        }
    }

    logo.close();
  
    if (index < LOGO_SIZE) return false;

    return true;
}

void startOta()
{
  timeDisplay = false;
  displayActive = true;
  fwupd = true;
  
  ws.closeAll();
  audio.stopSong();
  delay(250);
  if (!f_saveVolumeStationAlways){saveStationOnSD(); saveVolumeOnSD();}
  
  for (int i = 0; i < 3; i++) 
  {
    u8g2.clearBuffer();
    delay(25);
  }        
  unsigned long now = millis();
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.setFont(spleen6x12PL);     
  u8g2.setCursor(5, 12); u8g2.print("Evo Radio, OTA Firwmare Update");
  u8g2.sendBuffer();
}

// =============== FUNKCJE POMOCNICZE DLA SDPLAYER ===============
// Funkcja seek dla głównego systemu SDPlayer (main.cpp)
void performSeekAction(int deltaSeconds) {
  if (deltaSeconds == 0) {
    Serial.println("[SDPlayer] Seek BŁĄD: deltaSeconds=0");
    return;
  }
  
  // Próbuj przez WebUI jeśli jest aktywne
  if (g_sdPlayerWeb && (g_sdPlayerWeb->isPlaying() || g_sdPlayerWeb->isPaused())) {
    g_sdPlayerWeb->seekRelative(deltaSeconds);
    Serial.printf("[SDPlayer] Seek WebUI %s %d sec\n",
                  deltaSeconds > 0 ? "FWD" : "REV", 
                  abs(deltaSeconds));
    return;
  }
  
  // Próbuj przez główny audio jeśli SDPlayer gra
  if (sdPlayerPlayingMusic) {
    bool seekOk = audio.setTimeOffset(deltaSeconds);
    
    if (seekOk) {
      Serial.printf("[SDPlayer] Seek Audio %s %d sec -> OK\n",
                    deltaSeconds > 0 ? "FWD" : "REV", 
                    abs(deltaSeconds));
    } else {
      Serial.printf("[SDPlayer] Seek Audio %s %d sec -> FAILED\n",
                    deltaSeconds > 0 ? "FWD" : "REV", 
                    abs(deltaSeconds));
    }
  } else {
    Serial.println("[SDPlayer] Seek BŁĄD: Nic nie odtwarza");
  }
}
// ==============================================================

/*---------------------  FUNKCJA PILOT IR  / Obsluga pilota IR w kodzie NEC ---------------------*/ 
void handleRemote()
{
  // USUNIĘTO BLOKADĘ - SDPlayer ma swoją obsługę pilota poniżej
  
  if (bit_count == 32) // sprawdzamy czy odczytalismy w przerwaniu pełne 32 bity kodu IR NEC
  {
    if (ir_code != 0) // sprawdzamy czy zmienna ir_code nie jest równa 0
    {
      
      detachInterrupt(recv_pin);            // Rozpinamy przerwanie
      Serial.print("debug IR -> Kod NEC OK:");
      Serial.print(ir_code, HEX);
      ir_code = reverse_bits(ir_code,32);   // Rotacja bitów - zmiana porządku z LSB-MSB na MSB-LSB
      Serial.print("  MSB-LSB: ");
      Serial.print(ir_code, HEX);
    
      uint8_t CMD = (ir_code >> 16) & 0xFF; // Drugi bajt (inwersja adresu)
      uint8_t ADDR = ir_code & 0xFF;        // Czwarty bajt (inwersja komendy)
      
      Serial.print("  ADR:");
      Serial.print(ADDR, HEX);
      Serial.print(" CMD:");
      Serial.println(CMD, HEX);
      ir_code = ADDR << 8 | CMD;      // Łączymy ADDR i CMD w jedną zmienną 0x ADR CMD

      // ===== REMOTE CONTROL LEARNING MODE =====
      if (remoteConfig) {
        wsIrCode(ir_code, ADDR, CMD);
        ir_code = 0;
        bit_count = 0;
        attachInterrupt(recv_pin, pulseISR, CHANGE);
        return;
      }

      // Info o przebiegach czasowytch kodu pilota IR
      Serial.print("debug IR -> puls 9ms:"); 
      Serial.print(pulse_duration_9ms);
      Serial.print("  4.5ms:");
      Serial.print(pulse_duration_4_5ms);
      Serial.print("  1690us:");
      Serial.print(pulse_duration_1690us);
      Serial.print("  560us:");
      Serial.println(pulse_duration_560us);


      fwupd = false;        // Kasujemy flagę aktulizacji OTA gdyby była ustawiona
      //displayActive = true; // jesli odbierzemy kod z pilota to uatywnij wyswietlacz i wyłacz przyciemnienie OLEDa
      //displayDimmer(0); // jesli odbierzemy kod z pilota to wyłaczamy przyciemnienie wyswietlacza OLED
      displayPowerSave(0);
      
      // DEBUG: Sprawdź stan SDPlayera
      if (g_sdPlayerOLED) {
        Serial.printf("DEBUG IR: SDPlayer ptr=%p active=%d sdPlayerOLEDActive=%d\n", 
                     g_sdPlayerOLED, g_sdPlayerOLED->isActive(), sdPlayerOLEDActive);
      }
      
      // ===== SDPLAYER PILOT ROUTING (PRIORITY) =====
      if (g_sdPlayerOLED && g_sdPlayerOLED->isActive()) {
        // Obsługa dedykowanych przycisków SDPlayera
        // WYJAŃTEK: Jeśli equalizer jest aktywny, przepuść strzałki i OK do normalnej obsługi
        if (ir_code == rcCmdBack && !equalizerMenuEnable) {
          // Przycisk BACK - wyjście do katalogu nadrzędnego
          g_sdPlayerOLED->onRemoteBack();
          displayStartTime = millis();
          timeDisplay = false;
          displayActive = true;
          Serial.println("[SDPlayer] BACK button - going up to parent directory");
        }
        else if (ir_code == rcCmdArrowUp && !equalizerMenuEnable) {
          // Jeśli lista utworów jest wyświetlona, nawiguj po niej
          if (listedTracks) {
            currentTrackSelection--;
            if (currentTrackSelection < 0) {
              currentTrackSelection = tracksCount - 1;
              // Ustaw scrolling na koniec listy
              if (tracksCount > maxVisibleTracks) {
                firstVisibleTrack = tracksCount - maxVisibleTracks;
              } else {
                firstVisibleTrack = 0;
              }
            }
            // Dostosuj scrolling
            else if (currentTrackSelection < firstVisibleTrack) {
              firstVisibleTrack = currentTrackSelection;
            }
            displayTracks();
            displayStartTime = millis();  // Reset timera auto-play
          } else {
            // Standardowa obsługa SDPlayer lub pokaż listę utworów
            displayStartTime = millis();
            timeDisplay = false;
            displayActive = true;
            
            // Załaduj listę plików jeśli jeszcze nie była ładowana
            if (tracksCount == 0) {
              loadTracksFromSD();
            }
            
            if (tracksCount > 0) {
              listedTracks = true;
              // Pokaż listę z miejsca ostatnio odtwarzanego utworu
              if (lastPlayedTrackIndex >= 0 && lastPlayedTrackIndex < tracksCount) {
                currentTrackSelection = lastPlayedTrackIndex;
                // Ustaw scrolling aby wybrany był widoczny
                if (currentTrackSelection >= maxVisibleTracks) {
                  firstVisibleTrack = currentTrackSelection - maxVisibleTracks + 1;
                } else {
                  firstVisibleTrack = 0;
                }
              } else {
                currentTrackSelection = 0;
                firstVisibleTrack = 0;
              }
              displayTracks();
            } else {
              Serial.println("[SDPlayer] Brak utworów na karcie SD!");
            }
          }
        }
        else if (ir_code == rcCmdArrowDown && !equalizerMenuEnable) {
          // Jeśli lista utworów jest wyświetlona, nawiguj po niej
          if (listedTracks) {
            currentTrackSelection++;
            if (currentTrackSelection >= tracksCount) {
              currentTrackSelection = 0;
              // Ustaw scrolling na początek listy
              firstVisibleTrack = 0;
            }
            // Dostosuj scrolling
            else if (currentTrackSelection >= firstVisibleTrack + maxVisibleTracks) {
              firstVisibleTrack = currentTrackSelection - maxVisibleTracks + 1;
            }
            displayTracks();
            displayStartTime = millis();  // Reset timera auto-play
          } else {
            // Standardowa obsługa SDPlayer lub pokaż listę utworów
            displayStartTime = millis();
            timeDisplay = false;
            displayActive = true;
            
            // Załaduj listę plików jeśli jeszcze nie była ładowana
            if (tracksCount == 0) {
              loadTracksFromSD();
            }
            
            if (tracksCount > 0) {
              listedTracks = true;
              // Pokaż listę z miejsca ostatnio odtwarzanego utworu
              if (lastPlayedTrackIndex >= 0 && lastPlayedTrackIndex < tracksCount) {
                currentTrackSelection = lastPlayedTrackIndex;
                // Ustaw scrolling aby wybrany był widoczny
                if (currentTrackSelection >= maxVisibleTracks) {
                  firstVisibleTrack = currentTrackSelection - maxVisibleTracks + 1;
                } else {
                  firstVisibleTrack = 0;
                }
              } else {
                currentTrackSelection = 0;
                firstVisibleTrack = 0;
              }
              displayTracks();
            } else {
              Serial.println("[SDPlayer] Brak utworów na karcie SD!");
            }
          }
        }
        else if (ir_code == rcCmdOk && !equalizerMenuEnable) {
          // Jeśli lista utworów jest wyświetlona, odtwórz wybrany utwór
          if (listedTracks && tracksCount > 0) {
            Serial.print("[SDPlayer] Wybrano utwór: ");
            Serial.println(trackFiles[currentTrackSelection]);
            
            // Zapamiętaj wybrany utwór dla auto-play i powrotu do listy
            lastPlayedTrackIndex = currentTrackSelection;
            sdPlayerSessionStartIndex = currentTrackSelection;
            sdPlayerSessionEndIndex = currentTrackSelection;
            sdPlayerSessionCompleted = false;
            saveSDPlayerSessionState();
            
            // KRYTYCZNE: Wyczyść stare metadane ID3 przed załadowaniem nowego utworu
            currentMP3Artist = "";
            currentMP3Title = "";
            currentMP3Album = "";
            
            // Odtwórz wybrany plik - trackFiles[] już zawiera pełną ścieżkę względną
            String filePath = "/" + trackFiles[currentTrackSelection];
            audio.connecttoFS(STORAGE, filePath.c_str());
            
            // WALIDACJA: Sprawdź Duration (ochrona przed IntegerDivideByZero)
            // Czekaj aż Duration > 0 lub timeout 2.5s
            uint32_t duration = 0;
            for(int i = 0; i < 50 && duration == 0; i++) {
              audio.loop();
              delay(50);
              duration = audio.getAudioFileDuration();
            }
            if (duration == 0) {
              Serial.println("[SDPlayer] ERROR - Corrupted file (Duration=0)");
              audio.stopSong();
              sdPlayerPlayingMusic = false;
              if (g_sdPlayerWeb) {
                g_sdPlayerWeb->setCurrentFile("");
                g_sdPlayerWeb->setIsPlaying(false);
              }
            } else {
              sdPlayerPlayingMusic = true;
              
              // KRYTYCZNE: Aktualizuj nazwę pliku i stan odtwarzania w SDPlayerWebUI dla wyświetlacza
              if (g_sdPlayerWeb) {
                g_sdPlayerWeb->setCurrentFile(trackFiles[currentTrackSelection]);
                g_sdPlayerWeb->setIsPlaying(true);  // Ustaw flagę odtwarzania
              }
            }
            
            // KRYTYCZNE: Wyczyść flagę listy i wymuś odświeżenie ekranu
            listedTracks = false;
            displayActive = false;
            timeDisplay = false;
            
            // Wymuś natychmiastowe odświeżenie ekranu SDPlayer
            u8g2.clearBuffer();
            delay(50);  // Krótka pauza dla stabilności
            
            Serial.printf("[SDPlayer] Odtwarzanie rozpoczęte (indeks %d) - ekran zostanie odświeżony przez SDPlayerOLED.loop()\n", lastPlayedTrackIndex);
          } else {
            // Standardowa obsługa OK w SDPlayer
            g_sdPlayerOLED->onRemoteOK();
          }
        }
        else if (ir_code == rcCmdOk && equalizerMenuEnable) {
          // KRYTYCZNE: Obsługa OK podczas otwartego equalizera w SDPlayer
          Serial.println("[SDPlayer] OK w equalizerze - zapisywanie ustawień EQ");
          saveEqualizerOnSD();
          equalizerMenuEnable = false;
          displayActive = false;
          timeDisplay = false;
          u8g2.clearBuffer();
          delay(50);
          Serial.println("[SDPlayer] Equalizer zapisany, powrót do wyświetlacza");
        }
        else if (ir_code == rcCmdSrc) {
          // SRC - zmiana stylu OLED SDPlayera
          g_sdPlayerOLED->nextStyle();
          Serial.println("DEBUG: SDPlayer style changed via SRC");
        }
        else if (ir_code == rcCmdBT) {
          // BT button - ponieważ moduł BT usunięty, używamy jako dodatkowy przycisk zmiany stylu
          g_sdPlayerOLED->nextStyle();
          Serial.println("DEBUG: SDPlayer style changed via BT button (BT module removed)");
        }
        else if ((ir_code == rcCmdArrowRight || ir_code == rcCmdArrowLeft) && !equalizerMenuEnable) {
          // Nowa logika double-click: 1 klik = seek, 2 kliki w 2s = prev/next track
          if (!listedTracks && (sdPlayerOLEDActive || (g_sdPlayerWeb && (g_sdPlayerWeb->isPlaying() || g_sdPlayerWeb->isPaused())))) {
            const int currentDirection = (ir_code == rcCmdArrowRight) ? 1 : -1;
            const unsigned long nowMs = millis();
            const unsigned long doubleClickTimeout = 2000; // 2 sekundy na drugi klik
            
            // Sprawdź czy timeout już minął - jeśli tak, zresetuj stan
            if (waitingForSecondClick && (nowMs - firstClickTime > doubleClickTimeout)) {
              // Timeout minął - wykonaj seek dla pierwszego kliknięcia i zresetuj
              performSeekAction(lastClickDirection * 5);
              waitingForSecondClick = false;
            }
            
            // Sprawdź czy to drugi klik w tym samym kierunku w ciągu timeout
            if (waitingForSecondClick && 
                currentDirection == lastClickDirection && 
                (nowMs - firstClickTime <= doubleClickTimeout)) {
              
              // DRUGI KLIK - przejdź na poprzedni/następny utwór
              waitingForSecondClick = false;
              
              // Najpierw sprawdź czy używamy starego systemu (trackFiles[])
              if (tracksCount > 0 && sdPlayerOLEDActive) {
                if (currentDirection > 0) {
                  // NEXT TRACK
                  currentTrackSelection++;
                  if (currentTrackSelection >= tracksCount) {
                    currentTrackSelection = 0; // Zapętlenie
                  }
                  Serial.printf("[SDPlayer] Double-click RIGHT -> Następny utwór: indeks %d/%d\n", 
                              currentTrackSelection + 1, tracksCount);
                } else {
                  // PREV TRACK  
                  currentTrackSelection--;
                  if (currentTrackSelection < 0) {
                    currentTrackSelection = tracksCount - 1; // Zapętlenie
                  }
                  Serial.printf("[SDPlayer] Double-click LEFT -> Poprzedni utwór: indeks %d/%d\n", 
                              currentTrackSelection + 1, tracksCount);
                }
                
                // KRYTYCZNE: Wyczyść stare metadane ID3 przed załadowaniem nowego utworu
                currentMP3Artist = "";
                currentMP3Title = "";
                currentMP3Album = "";
                
                // Odtwórz nowy plik - trackFiles[] już zawiera pełną ścieżkę względną
                lastPlayedTrackIndex = currentTrackSelection;
                String filePath = "/" + trackFiles[currentTrackSelection];
                Serial.printf("[SDPlayer] Odtwarzanie: %s\n", trackFiles[currentTrackSelection].c_str());
                audio.connecttoFS(STORAGE, filePath.c_str());
                
                // WALIDACJA: Sprawdź Duration (ochrona przed IntegerDivideByZero)
                // Czekaj aż Duration > 0 lub timeout 2.5s
                uint32_t duration = 0;
                for(int i = 0; i < 50 && duration == 0; i++) {
                  audio.loop();
                  delay(50);
                  duration = audio.getAudioFileDuration();
                }
                if (duration == 0) {
                  Serial.println("[SDPlayer] ERROR - Corrupted file (Duration=0)");
                  audio.stopSong();
                  sdPlayerPlayingMusic = false;
                  if (g_sdPlayerWeb) {
                    g_sdPlayerWeb->setCurrentFile("");
                    g_sdPlayerWeb->setIsPlaying(false);
                  }
                } else {
                  sdPlayerPlayingMusic = true;
                  
                  // Synchronizuj z SDPlayerWebUI dla wyświetlacza
                  if (g_sdPlayerWeb) {
                    g_sdPlayerWeb->setCurrentFile(trackFiles[currentTrackSelection]);
                    g_sdPlayerWeb->setIsPlaying(true);
                  }
                }
              }
              // Jeśli nie ma trackFiles, użyj WebUI
              else if (g_sdPlayerWeb) {
                if (currentDirection > 0) {
                  g_sdPlayerWeb->next();
                  Serial.println("[SDPlayer] Double-click RIGHT -> Następny utwór (WebUI)");
                } else {
                  g_sdPlayerWeb->prev();
                  Serial.println("[SDPlayer] Double-click LEFT -> Poprzedni utwór (WebUI)");
                }
              }
              
            } else {
              // PIERWSZY KLIK lub zmiana kierunku - rozpocznij nowe oczekiwanie
              waitingForSecondClick = true;
              firstClickTime = nowMs;
              lastClickDirection = currentDirection;
              
              Serial.printf("[SDPlayer] Pierwszy klik %s - oczekiwanie na drugi klik w ciągu 2s\n",
                            currentDirection > 0 ? "RIGHT" : "LEFT");
            }
          }
        }
        else if (ir_code == rcCmdBack) {
          // BACK - różne działania w zależności od stanu
          if (equalizerMenuEnable) {
            #if defined(ENABLE_EQ16) && (ENABLE_EQ16 == 1)
            // ========== EQUALIZER 16-BAND: BACK = WYJŚCIE ==========
            if (eq16ModeSelectActive) {
              // Wyjście z wyboru trybu - zamknij menu EQ całkowicie
              eq16ModeSelectActive = false;
              equalizerMenuEnable = false;
              displayActive = false;
              timeDisplay = false;
              u8g2.clearBuffer();
              delay(50);
              Serial.println("[EQ16] BACK - zamknięcie menu wyboru trybu");
              return;
            }
            else if (eq16Mode) {
              // Wyjście z edytora 16-band - zapisz i pokaż wybór trybu
              EQ16_saveToSD();
              eq16ModeSelectActive = true;  // Wróć do wyboru trybu
              displayEqualizer();
              Serial.println("[EQ16] BACK - zapisano i powrót do wyboru trybu");
              return;
            }
            #endif
            
            // ========== EQUALIZER 3-BAND: BACK = ZAMKNIJ BEZ ZAPISU ==========
            // Jeśli equalizer jest aktywny - zamknij bez zapisywania
            Serial.println("[EQ] BACK - zamknięcie equalizera bez zapisu");
            equalizerMenuEnable = false;
            displayActive = false;
            timeDisplay = false;
            u8g2.clearBuffer();
            delay(50);
            Serial.println("[EQ] Powrót do normalnego wyświetlacza");
          }
          else if (listedTracks) {
            // Jeśli lista utworów jest aktywna - wyjdź z niej
            Serial.println("[SDPlayer] BACK - wyjście z listy utworów");
            listedTracks = false;
            displayActive = false;
            timeDisplay = false;
            
            // Wymuś odświeżenie ekranu SDPlayer
            u8g2.clearBuffer();
            delay(50);
            Serial.println("[SDPlayer] Ekran zostanie odświeżony przez SDPlayerOLED.loop()");
          } else {
            // Standardowa obsługa BACK - podwójne kliknięcie wychodzi z SDPlayera
            static unsigned long lastBackPress = 0;
            static uint8_t backClickCount = 0;
            unsigned long currentTime = millis();
            
            if (currentTime - lastBackPress > 600) { // 600ms okno
              backClickCount = 0; // Reset po timeout
            }
            
            backClickCount++;
            lastBackPress = currentTime;
            
            if (backClickCount >= 2) {
              // Podwójne kliknięcie - wyjście z SDPlayera
              Serial.println("DEBUG: Double BACK press - exiting SDPlayer to radio");
              backClickCount = 0; // Reset

              if (sdPlayerSessionStartIndex >= 0 && sdPlayerSessionEndIndex >= 0) {
                sdPlayerSessionCompleted = true;
                saveSDPlayerSessionState();
              }
              
              g_sdPlayerOLED->deactivate();
              sdPlayerOLEDActive = false;
              sdPlayerPlayingMusic = false;
              audio.stopSong();
              
              // Wyłącz boost dynamiki analizatora
              eq_analyzer_set_sdplayer_mode(false);
              
              // WAŻNE: Wyczyść bufor przed przełączeniem na radio
              u8g2.clearBuffer();
            
              // Przywróć zapisany bank i stację
              bank_nr = sdPlayerReturnBank;
              station_nr = sdPlayerReturnStation;
              Serial.printf("[SDPlayer] Przywrócono pozycję: Bank %d, Stacja %d\n", bank_nr, station_nr);
              changeStation();
              displayRadio();
              u8g2.sendBuffer();
            } else {
              // Pojedyncze kliknięcie - STOP
              g_sdPlayerOLED->onRemoteStop();
              Serial.println("SD Player: BACK button - STOP (press again quickly to exit)");
            }
          }
        }
        else if (ir_code == rcCmdKey0) {
          // Key0 - ma trzy funkcje:
          // 1. Dodaj "0" jako drugą cyfrę (1→10, 2→20) - obsługiwane w rcInputKey
          // 2. ENTER dla liczby dwucyfrowej (np. "10", "25")
          // 3. Podwójne "00" → Wyjście z SDPlayera do radia
          
          // Najpierw wywołaj rcInputKey(0) - obsłuży dodanie "0" do bufora jeśli potrzebne
          rcInputKey(0);
          
          // Sprawdź czy to ENTER dla liczby >= 10
          if (irInputActive && irInputBuffer >= 10 && irInputBuffer <= 100) {
            // ENTER dla wielocyfrowego wprowadzania - potwierdź liczbę (np. "2" + "5" + "0" = utwór 25)
            Serial.printf("DEBUG: Key0 ENTER - confirming track: %d\n", irInputBuffer);
            irRequestedTrackNumber = irInputBuffer;
            irTrackSelectionRequest = true;
            
            // Resetuj system wprowadzania
            irInputBuffer = 0;
            irInputActive = false;
            irInputTimeout = 0;
            
            Serial.printf("DEBUG: Track %d selected via Key0 ENTER\n", irRequestedTrackNumber);
          } else if (!irInputActive || irInputBuffer == 0) {
            // Podwójne "00" (bez wcześniejszych cyfr) → Wyjście z SDPlayera do radia
            static unsigned long lastKey0Press = 0;
            static uint8_t key0ClickCount = 0;
            unsigned long currentTime = millis();
            
            if (currentTime - lastKey0Press > 600) { // 600ms okno
              key0ClickCount = 0; // Reset po timeout
            }
            
            key0ClickCount++;
            lastKey0Press = currentTime;
            
            if (key0ClickCount >= 2) {
              // Podwójne "00" - wyjście z SDPlayera
              Serial.println("DEBUG: Double Key0 press (00) - Exiting SDPlayer to radio");
              key0ClickCount = 0; // Reset

              if (sdPlayerSessionStartIndex >= 0 && sdPlayerSessionEndIndex >= 0) {
                sdPlayerSessionCompleted = true;
                saveSDPlayerSessionState();
              }

              g_sdPlayerOLED->deactivate();
              sdPlayerOLEDActive = false;
              sdPlayerPlayingMusic = false;
              audio.stopSong();
              
              // Wyłącz boost dynamiki analizatora
              eq_analyzer_set_sdplayer_mode(false);
            
              // WAŻNE: Wyczyść bufor przed przełączeniem na radio
              u8g2.clearBuffer();
            
              // Przywróć zapisany bank i stację
              bank_nr = sdPlayerReturnBank;
              station_nr = sdPlayerReturnStation;
              Serial.printf("[SDPlayer] Key00: Przywrócono pozycję: Bank %d, Stacja %d\n", bank_nr, station_nr);
              changeStation();
              displayRadio();
              u8g2.sendBuffer();
            } else {
              // Pojedyncze "0" - czekaj na drugie
              Serial.println("DEBUG: Single Key0 press - press again (00) to exit");
            }
          }
        }
        else if (ir_code == rcCmdVolumeUp) {
          g_sdPlayerOLED->onRemoteVolUp();
        }
        else if (ir_code == rcCmdVolumeDown) {
          g_sdPlayerOLED->onRemoteVolDown();
        }
        else if (ir_code == rcCmdMute) {
          // MUTE toggle w trybie SDPlayer (normalna obsługa globalna jest zablokowana przez SDPLAYER ROUTING)
          static int sdPlayerVolumeBeforeMute = -1;

          volumeMute = !volumeMute;
          if (volumeMute) {
            sdPlayerVolumeBeforeMute = g_sdPlayerWeb ? g_sdPlayerWeb->getVolume() : volumeValue;
            if (g_sdPlayerWeb) {
              g_sdPlayerWeb->setVolume(0);
            } else {
              audio.setVolume(0);
            }
            Serial.println("[SDPlayer] MUTE ON");
          } else {
            int restoreVolume = (sdPlayerVolumeBeforeMute >= 0) ? sdPlayerVolumeBeforeMute : volumeValue;
            if (g_sdPlayerWeb) {
              g_sdPlayerWeb->setVolume(restoreVolume);
            } else {
              audio.setVolume(restoreVolume);
            }
            sdPlayerVolumeBeforeMute = -1;
            Serial.printf("[SDPlayer] MUTE OFF (vol=%d)\n", restoreVolume);
          }

          wsVolumeChange();
        }
        else if (ir_code == rcCmdAud) {
          // Przycisk AUD - otwórz korektor EQ
          displayEqualizer();
          Serial.println("[SDPlayer] Equalizer opened (AUD button)");
        }
        else if (ir_code == rcCmdPower || ir_code == rcCmdRed) {
          // POWER OFF / RED - pozwól przejść do handleRemote()
          Serial.printf("[SDPlayer] Power OFF button detected (0x%04X) - passing to handler\n", ir_code);
          // NIE BLOKUJ - pozwól mu przejść do handleRemote() gdzie jest pełna obsługa
        }
        else if (ir_code == rcCmdBack) {
          // BACK button - pozwól przejść do handleRemote() gdzie jest obsługa goUp()
          Serial.printf("[SDPlayer] BACK button detected (0x%04X) - for folder UP navigation\n", ir_code);
          // NIE BLOKUJ - będzie obsłużony w handleRemote()
        }
        else if (ir_code == rcCmdKey9) {
          // Key9 - pozostaw bez zmian, obsłuży poniższy kod
          // NIE BLOKUJ Key9 - pozwól mu przejść dalej
        }
        // UWAGA: Strzałki equalizera (gdy equalizerMenuEnable==true) są obsługiwane 
        // przez normalną obsługę w handleRemote() (linie 10539, 10606, 10396, 10465)
        // NIE ma tu specjalnego bloku - po prostu przechodzą przez sekcję SDPlayer
        else if (ir_code == rcCmdPAUSE || ir_code == rcCmdPLAY) {
          // PAUSE/PLAY - pauza/wznowienie odtwarzania
          if (sdPlayerPlayingMusic || (g_sdPlayerWeb && g_sdPlayerWeb->isPlaying())) {
            audio.pauseResume();
            bool newState = !audio.isRunning();
            if (g_sdPlayerWeb) {
              g_sdPlayerWeb->setIsPlaying(newState);
            }
            Serial.printf("[SDPlayer] %s - %s\n", 
                         (ir_code == rcCmdPAUSE ? "PAUSE" : "PLAY"),
                         (newState ? "Wznowiono" : "Zapauzowano"));
          } else {
            Serial.println("[SDPlayer] PLAY/PAUSE - Brak aktywnego utworu");
          }
        }
        else if (ir_code == rcCmdSTOP) {
          // STOP - zatrzymanie odtwarzania
          audio.stopSong();
          sdPlayerPlayingMusic = false;
          if (g_sdPlayerWeb) {
            g_sdPlayerWeb->setIsPlaying(false);
          }
          Serial.println("[SDPlayer] STOP - Zatrzymano odtwarzanie");
        }
        else if (ir_code == rcCmdEXIT) {
          // EXIT - wyjście z SDPlayera do radia
          Serial.println("[SDPlayer] EXIT - Wyjście do trybu radio");
          
          if (sdPlayerSessionStartIndex >= 0 && sdPlayerSessionEndIndex >= 0) {
            sdPlayerSessionCompleted = true;
            saveSDPlayerSessionState();
          }

          g_sdPlayerOLED->deactivate();
          sdPlayerOLEDActive = false;
          sdPlayerPlayingMusic = false;
          audio.stopSong();
          
          // Wyłącz boost dynamiki analizatora
          eq_analyzer_set_sdplayer_mode(false);
        
          // Wyczyść bufor przed przełączeniem na radio
          u8g2.clearBuffer();
        
          // Przywróć zapisany bank i stację
          bank_nr = sdPlayerReturnBank;
          station_nr = sdPlayerReturnStation;
          Serial.printf("[SDPlayer] EXIT: Przywrócono pozycję: Bank %d, Stacja %d\n", bank_nr, station_nr);
          changeStation();
          displayRadio();
          u8g2.sendBuffer();
        }
        else if (ir_code == rcCmdREV) {
          // REV - przewiń do tyłu 10 sekund
          if (sdPlayerPlayingMusic) {
            performSeekAction(-10);
            Serial.println("[SDPlayer] REV - Przewinięcie -10s");
          }
        }
        else if (ir_code == rcCmdRER) {
          // RER - przewiń do przodu 10 sekund
          if (sdPlayerPlayingMusic) {
            performSeekAction(10);
            Serial.println("[SDPlayer] RER - Przewinięcie +10s");
          }
        }
        else if (ir_code == rcCmdFILES || ir_code == rcCmdMENU) {
          // FILES/MENU - pokaż listę utworów
          if (!listedTracks && tracksCount > 0) {
            listedTracks = true;
            currentTrackSelection = lastPlayedTrackIndex >= 0 ? lastPlayedTrackIndex : 0;
            firstVisibleTrack = max(0, currentTrackSelection - 1);
            // SDPlayerOLED.loop() automatycznie wyświetla interfejs
            Serial.println("[SDPlayer] FILES/MENU - Pokazano listę utworów");
          } else {
            Serial.println("[SDPlayer] FILES/MENU - Lista już wyświetlona lub brak utworów");
          }
        }
        else if (ir_code == rcCmdRADIO) {
          // RADIO - szybkie wyjście do trybu radio (alias dla EXIT)
          Serial.println("[SDPlayer] RADIO - Szybkie wyjście do radia");
          
          g_sdPlayerOLED->deactivate();
          sdPlayerOLEDActive = false;
          sdPlayerPlayingMusic = false;
          audio.stopSong();
          eq_analyzer_set_sdplayer_mode(false);
          u8g2.clearBuffer();
          
          bank_nr = sdPlayerReturnBank;
          station_nr = sdPlayerReturnStation;
          changeStation();
          displayRadio();
          u8g2.sendBuffer();
        }
        else if (ir_code == rcCmdTV || ir_code == rcCmdWWW || ir_code == rcCmdGAZE || ir_code == rcCmdEPG) {
          // TV/WWW/GAZE/EPG - zarezerwowane na przyszłe funkcje
          Serial.printf("[SDPlayer] Przycisk 0x%04X - funkcja zarezerwowana\n", ir_code);
        }
        else {
          // WSZYSTKIE inne przyciski są ignorowane gdy SDPlayer aktywny
          Serial.printf("DEBUG: SDPlayer active - blocking IR code 0x%04X\n", ir_code);
        }
        
        // Funkcja sprawdzająca czy kod IR to klawisz numeryczny (0-9)
        bool isNumericKey = (ir_code == rcCmdKey0 || ir_code == rcCmdKey1 || ir_code == rcCmdKey2 || 
                            ir_code == rcCmdKey3 || ir_code == rcCmdKey4 || ir_code == rcCmdKey5 || 
                            ir_code == rcCmdKey6 || ir_code == rcCmdKey7 || ir_code == rcCmdKey8 || 
                            ir_code == rcCmdKey9);
        
        // Funkcja sprawdzająca czy to Power OFF
        bool isPowerOffKey = (ir_code == rcCmdPower || ir_code == rcCmdRed);
        
        // Funkcja sprawdzająca czy to BACK (folder UP)
        bool isBackKey = (ir_code == rcCmdBack);
        
        // Funkcja sprawdzająca czy to przyciski SDPlayera
        bool isSDPlayerKey = (ir_code == rcCmdPAUSE || ir_code == rcCmdPLAY || ir_code == rcCmdSTOP ||
                              ir_code == rcCmdEXIT || ir_code == rcCmdREV || ir_code == rcCmdRER ||
                              ir_code == rcCmdFILES || ir_code == rcCmdMENU || ir_code == rcCmdRADIO ||
                              ir_code == rcCmdTV || ir_code == rcCmdWWW || ir_code == rcCmdGAZE || ir_code == rcCmdEPG);
        
        // Zablokuj WSZYSTKIE komendy oprócz:
        // - klawiszy numerycznych (0-9)
        // - strzałek equalizera
        // - Power OFF i BACK
        // - przycisków SDPlayera (PAUSE, PLAY, STOP, EXIT, etc.)
        if (!isNumericKey && !isPowerOffKey && !isBackKey && !isSDPlayerKey &&
            !(equalizerMenuEnable && (ir_code == rcCmdArrowUp || ir_code == rcCmdArrowDown || 
                                      ir_code == rcCmdArrowLeft || ir_code == rcCmdArrowRight))) {
          ir_code = 0;
          bit_count = 0;
          attachInterrupt(digitalPinToInterrupt(recv_pin), pulseISR, CHANGE);
          #ifdef IR_LED
          digitalWrite(IR_LED, LOW); delay(30);
          #endif
          return; // WAŻNE: Przerwij dalsze przetwarzanie, inaczej ir_code==0 aktywuje combo 000!
        }
      }
      // ===== KONIEC SDPLAYER ROUTING =====
      
      // 2. Przycisk AUD - podwójne kliknięcie aktywuje SDPlayer, pojedyncze otwiera equalizer
      if (ir_code == rcCmdAud) {
        unsigned long nowMs = millis();
        const unsigned long doubleClickTimeout = 600; // 600ms na drugie naciśnięcie
        
        if (waitingForSecondAudClick && (nowMs - firstAudClickTime <= doubleClickTimeout)) {
          // Drugie naciśnięcie w czasie - AKTYWUJ SDPLAYER
          waitingForSecondAudClick = false;
          Serial.println("[AUD] PODWÓJNE KLIKNIĘCIE - Aktywacja SDPlayer");
          
          // Zamknij equalizer jeśli był otwarty
          if (equalizerMenuEnable) {
            equalizerMenuEnable = false;
          }
          
          // KRYTYCZNE: Wyzeruj flagi rcInput
          rcInputDigitsMenuEnable = false;
          rcInputDigit1 = 0xFF;
          rcInputDigit2 = 0xFF;
          
          if (g_sdPlayerOLED) {
            // Zapamiętaj aktualny bank i stację przed aktywacją
            sdPlayerReturnBank = bank_nr;
            sdPlayerReturnStation = station_nr;
            Serial.printf("[SDPlayer] IR: Zapisano pozycję powrotu: Bank %d, Stacja %d\n", sdPlayerReturnBank, sdPlayerReturnStation);
            
            g_sdPlayerOLED->activate();
            sdPlayerOLEDActive = true;
            
            // Włącz 3x większą dynamikę analizatora dla SDPlayera
            eq_analyzer_set_sdplayer_mode(true);
            
            // Pokaż splash screen "SD PLAYER"
            g_sdPlayerOLED->showSplash();
            delay(1000);  // Pokaż przez 1 sekundę
            
            // SYNCHRONIZACJA: Jeśli coś gra, zsynchronizuj WebUI z aktualnym stanem
            if (g_sdPlayerWeb && sdPlayerPlayingMusic) {
              g_sdPlayerWeb->setIsPlaying(true);
              
              if (currentTrackSelection >= 0 && currentTrackSelection < tracksCount) {
                String currentFileName = trackFiles[currentTrackSelection];
                g_sdPlayerWeb->setCurrentFile(currentFileName);
                Serial.printf("[SDPlayer] IR: Synchronized WebUI with current track: %s\n", currentFileName.c_str());
              }
            }
            
            Serial.println("[SDPlayer] Aktywny - użyj ↑/↓ aby zobaczyć listę utworów");
          }
        } else {
          // Pierwsze naciśnięcie LUB timeout minął - OTWÓRZ EQUALIZER
          waitingForSecondAudClick = true;
          firstAudClickTime = nowMs;
          displayEqualizer();
          Serial.println("[AUD] Equalizer otwarty (czekam na ewentualne drugie kliknięcie dla SDPlayer)");
        }
        
        ir_code = 0;
        bit_count = 0;
        attachInterrupt(digitalPinToInterrupt(recv_pin), pulseISR, CHANGE);
        return;
      }
      
      // 4. Toggle Analyzer (np. przycisk YELLOW na pilocie)
      if (ir_code == rcCmdYellow) {
        analyzerEnabled = !analyzerEnabled;
        eq_analyzer_set_enabled(analyzerEnabled);
        Serial.printf("DEBUG: Analyzer %s\n", analyzerEnabled ? "ENABLED" : "DISABLED");
        displayRadio();
        ir_code = 0;
        bit_count = 0;
        attachInterrupt(digitalPinToInterrupt(recv_pin), pulseISR, CHANGE);
        return;
      }
      
      // ============== KONIEC OBSŁUGI MODUŁÓW - kontynuacja normalnej obsługi radia ==============
      lastIrCode = ir_code; // Zapamiętujemy ostatni kod (dla combo)
      
      if (ir_code == rcCmdVolumeUp)  { volumeUp(); }         // Przycisk głośniej
      else if (ir_code == rcCmdVolumeDown) { volumeDown(); } // Przycisk ciszej
      else if (ir_code == rcCmdArrowRight) // strzałka w prawo - nastepna stacja, bank lub nastawy equalizera
      {  
        if (bankMenuEnable == true)
        {
          bank_nr++;
          if (bank_nr > bank_nr_max) 
          {
            bank_nr = 1;
          }
        bankMenuDisplay();
        }
        else if (equalizerMenuEnable == true)
        {
          #if defined(ENABLE_EQ16) && (ENABLE_EQ16 == 1)
          // ========== EQUALIZER 16-BAND: WYBÓR TRYBU ==========
          if (eq16ModeSelectActive) {
            eq16Mode = !eq16Mode;  // Przełącz między 3-band (0) a 16-band (1)
            displayEqualizer();
            Serial.printf("[EQ16] Mode toggled: %s\n", eq16Mode ? "16-band" : "3-band");
            return;
          }
          // ========== EQUALIZER 16-BAND: WYBÓR PASMA ==========
          else if (eq16Mode) {
            // Wybierz następne pasmo (RIGHT)
            if (eq16SelectedBand < 15) {
              eq16SelectedBand++;
            } else {
              eq16SelectedBand = 0;  // Wrap around do początku
            }
            Serial.printf("[EQ16] RIGHT: Selected band %d (gain=%ddB)\n", eq16SelectedBand, APMS_EQ16::getBand(eq16SelectedBand));
            displayEqualizer();
            return;
          }
          #endif
          
          // ========== EQUALIZER 3-BAND (DOMYŚLNY) ==========
          if (toneSelect == 1) {toneHiValue++;}
          if (toneSelect == 2) {toneMidValue++;}
          if (toneSelect == 3) {toneLowValue++;}
          if (toneSelect == 4) {balanceValue++;}
          
          if (toneHiValue > 6) {toneHiValue = 6;}
          if (toneMidValue > 6) {toneMidValue = 6;}
          if (toneLowValue > 6) {toneLowValue = 6;}
          if (balanceValue > 16) {balanceValue = 16;}
          
          audio.setTone(toneLowValue, toneMidValue, toneHiValue);  // Aplikuj zmiany natychmiast
          audio.setBalance(balanceValue);
          Serial.printf("[EQ3] RIGHT: HI=%d MID=%d LOW=%d BAL=%d\n", toneHiValue, toneMidValue, toneLowValue, balanceValue);
          displayEqualizer();
        }     
        else if (listedStations == true) // Szybkie przewijanie o 5 stacji
        {
          timeDisplay = false;
          displayActive = true;
          displayStartTime = millis();    
          station_nr = currentSelection + 1;

          station_nr = station_nr + 5;
          for (int i = 0; i < 5; i++) {scrollDown();}
    
          if (station_nr > stationsCount) {station_nr = station_nr - stationsCount; } //Zbaezpiecznie aby przewijac sie tylko po stacjach w liczbie jaka jest w stationCount
          
          displayStations();
        }
        else
        {
          station_nr++;
          //if (station_nr > stationsCount) { station_nr = stationsCount; }
          if (station_nr > stationsCount) { station_nr = 1; } // Przwijanie listy stacji w pętli po osiągnieciu ostatniej stacji banku przewijamy do pierwszej.
          changeStation();
          displayRadio();
          u8g2.sendBuffer();
        }
      }
      else if (ir_code == rcCmdArrowLeft) // strzałka w lewo - poprzednia stacja, bank lub nastawy equalizera
      {  
        if (bankMenuEnable == true)
        {
          bank_nr--;
          if (bank_nr < 1) 
          {
            bank_nr = bank_nr_max;
          }
        bankMenuDisplay();  
        }
        else if (equalizerMenuEnable == true)
        {
          #if defined(ENABLE_EQ16) && (ENABLE_EQ16 == 1)
          // ========== EQUALIZER 16-BAND: WYBÓR TRYBU ==========
          if (eq16ModeSelectActive) {
            eq16Mode = !eq16Mode;  // Przełącz między 3-band (0) a 16-band (1)
            displayEqualizer();
            Serial.printf("[EQ16] Mode toggled: %s\n", eq16Mode ? "16-band" : "3-band");
            return;
          }
          // ========== EQUALIZER 16-BAND: WYBÓR PASMA ==========
          else if (eq16Mode) {
            // Wybierz poprzednie pasmo (LEFT)
            if (eq16SelectedBand > 0) {
              eq16SelectedBand--;
            } else {
              eq16SelectedBand = 15;  // Wrap around do końca
            }
            Serial.printf("[EQ16] LEFT: Selected band %d (gain=%ddB)\n", eq16SelectedBand, APMS_EQ16::getBand(eq16SelectedBand));
            displayEqualizer();
            return;
          }
          #endif
          
          // ========== EQUALIZER 3-BAND (DOMYŚLNY) ==========
          if (toneSelect == 1) {toneHiValue--;}
          if (toneSelect == 2) {toneMidValue--;}
          if (toneSelect == 3) {toneLowValue--;}
          if (toneSelect == 4) {balanceValue--;}
          
          if (toneHiValue < -40) {toneHiValue = -40;}
          if (toneMidValue < -40) {toneMidValue = -40;}
          if (toneLowValue < -40) {toneLowValue = -40;}
          if (balanceValue < -16) {balanceValue = -16;}
          
          audio.setTone(toneLowValue, toneMidValue, toneHiValue);  // Aplikuj zmiany natychmiast
          audio.setBalance(balanceValue);
          Serial.printf("[EQ3] LEFT: HI=%d MID=%d LOW=%d BAL=%d\n", toneHiValue, toneMidValue, toneLowValue, balanceValue);
          displayEqualizer();
        }     
        else if (listedStations == true)
        {
          timeDisplay = false;
          displayActive = true;
          displayStartTime = millis();    
          station_nr = currentSelection + 1;

          station_nr = station_nr - 5;
          
          for (int i = 0; i < 5; i++) {scrollUp();}
      
          //Zbaezpiecznie aby przewijac sie tylko po stacjach w liczbie jaka jest w stationCount
          if (station_nr > stationsCount) { station_nr = (stationsCount - (255 - station_nr)) - 1 ;} 
          
          // Jesli jestesmy na stacji nr 5 to aby uniknąc station_nr = 0 przewijamy do ostatniej
          if (station_nr == 0) {station_nr = stationsCount;} 

          displayStations();
        }      
        else
        {        
          station_nr--;
          //if (station_nr < 1) { station_nr = 1; }
          if (station_nr < 1) { station_nr = stationsCount; } // Przwijanie listy stacji w pętli po osiągnieciu ostatniej stacji banku przewijamy do pierwszej.
          changeStation();
          displayRadio();
          u8g2.sendBuffer();
        }
      }
      else if ((ir_code == rcCmdArrowUp) && (volumeSet == false) && (equalizerMenuEnable == true))
      {
        #if defined(ENABLE_EQ16) && (ENABLE_EQ16 == 1)
        // ========== EQUALIZER 16-BAND: OK = POTWIERDZ WYBÓR TRYBU ==========
        if (eq16ModeSelectActive) {
          eq16ModeSelectActive = false;  // Zatwierdź wybór i wejdź do edytor
          displayEqualizer();
          return;
        }
        // ========== EQUALIZER 16-BAND: UP = ZWIĘKSZ WZMOCNIENIE ==========
        else if (eq16Mode) {
          int8_t gain = APMS_EQ16::getBand(eq16SelectedBand);
          if (gain < 16) {
            APMS_EQ16::setBand(eq16SelectedBand, gain + 1);
            Serial.printf("[EQ16] UP: Band %d gain %d->%d\n", eq16SelectedBand, gain, gain+1);
            APMS_EQ16::applyToAudio();  // Zastosuj do audio
            displayEqualizer();
          } else {
            Serial.printf("[EQ16] UP: Band %d already at MAX (+16dB)\n", eq16SelectedBand);
          }
          return;
        }
        #endif
        
        // ========== EQUALIZER 3-BAND: UP = POPRZEDNI PARAMETR ==========
        toneSelect--;
        if (toneSelect < 1){toneSelect = 1;}
        displayEqualizer();
      }
      else if ((ir_code == rcCmdArrowUp) && (equalizerMenuEnable == false))// Przycisk w góre
      {  
        if ((volumeSet == true) && (volumeBufferValue != volumeValue))
        {
          if (f_saveVolumeStationAlways) {saveVolumeOnSD();}
          volumeSet = false;
        }
        
        timeDisplay = false;
        displayActive = true;
        displayStartTime = millis();    
        station_nr = currentSelection + 1;

        if (listedStations == true) {station_nr--; scrollUp();} //station_nr-- tylko jesli już wyswietlany liste stacji;
        else
        {        
          currentSelection = station_nr - 1; // Przywracamy zaznaczenie obecnie grajacej stacji
          
          if (currentSelection >= 0)
          {
            if (currentSelection < firstVisibleLine) // jezeli obecne zaznaczenie ma wartosc mniejsza niz pierwsza wyswietlana linia
            {
              firstVisibleLine = currentSelection;
            }
          } 
          else 
          {  // Jeśli osiągnięto wartość 0, przejdź do najwyższej wartości
            if (currentSelection = maxSelection())
            {
            firstVisibleLine = currentSelection - maxVisibleLines + 1;  // Ustaw pierwszą widoczną linię na najwyższą
            }
          }   
        }
        
        if (station_nr < 1) { station_nr = stationsCount; } // jesli dojdziemy do początku listy stacji to przewijamy na koniec
        
        
       
       
      displayStations();
      }
      else if ((ir_code == rcCmdArrowDown) && (volumeSet == false) && (equalizerMenuEnable == true))
      {
        #if defined(ENABLE_EQ16) && (ENABLE_EQ16 == 1)
        // ========== EQUALIZER 16-BAND: DOWN = ZMNIEJSZ WZMOCNIENIE ==========
        if (eq16ModeSelectActive) {
          eq16ModeSelectActive = false;  //  Zatwierdź wybór i wejdź do edytora
          displayEqualizer();
          return;
        }
        else if (eq16Mode) {
          int8_t gain = APMS_EQ16::getBand(eq16SelectedBand);
          if (gain > -16) {
            APMS_EQ16::setBand(eq16SelectedBand, gain - 1);
            Serial.printf("[EQ16] DOWN: Band %d gain %d->%d\n", eq16SelectedBand, gain, gain-1);
            APMS_EQ16::applyToAudio();  // Zastosuj do audio
            displayEqualizer();
          } else {
            Serial.printf("[EQ16] DOWN: Band %d already at MIN (-16dB)\n", eq16SelectedBand);
          }
          return;
        }
        #endif
        
        // ========== EQUALIZER 3-BAND: DOWN = NASTĘPNY PARAMETR ==========
        toneSelect++;
        if (toneSelect > 4){toneSelect = 4;}
        displayEqualizer();
      }
      else if ((ir_code == rcCmdArrowDown) && (equalizerMenuEnable == false)) // Przycisk w dół
      {  
        if ((volumeSet == true) && (volumeBufferValue != volumeValue))
        {
          if (f_saveVolumeStationAlways) {saveVolumeOnSD();}
          volumeSet = false;
        }
        
        timeDisplay = false;
        displayActive = true;
        displayStartTime = millis();     
        station_nr = currentSelection + 1;
        
        //station_nr++;
        if (listedStations == true) {station_nr++; scrollDown(); } //station_nr++ tylko jesli już wyswietlany liste stacji;
        else
        {        
          currentSelection = station_nr - 1; // Przywracamy zaznaczenie obecnie grajacej stacji

          if (currentSelection >= 0)
          {
            if (currentSelection < firstVisibleLine) // jezeli obecne zaznaczenie ma wartosc mniejsza niz pierwsza wyswietlana linia
            {
              firstVisibleLine = currentSelection;
            }
          } 
          else 
          {  // Jeśli osiągnięto wartość 0, przejdź do najwyższej wartości
            if (currentSelection = maxSelection())
            {
            firstVisibleLine = currentSelection - maxVisibleLines + 1;  // Ustaw pierwszą widoczną linię na najwyższą
            }
          }
             
        }
        
        if (station_nr > stationsCount) 
	      {
          station_nr = 1;//stationsCount;
        }
        
        //Serial.println(station_nr);

         
        displayStations();
      }    
      else if (ir_code == rcCmdOk)
      {
        if (bankMenuEnable == true)
        {
          station_nr = 1;
          fetchStationsFromServer(); // Ładujemy stacje z karty lub serwera 
          bankMenuEnable = false;
        }  
        if (equalizerMenuEnable == true) { saveEqualizerOnSD();}    // zapis ustawien equalizera
        //if (volumeSet == true) { saveVolumeOnSD();}                 // zapis ustawien głośnosci po nacisnięciu OK, wyłaczony aby można było przełączyć stacje na www bez czekania
        //if ((equalizerMenuEnable == false) && (volumeSet == false)) // jesli nie zapisywaliśmy equlizer i glonosci to wywolujemy ponizsze funkcje
        if ((equalizerMenuEnable == false)) // jesli nie zapisywaliśmy equlizer 
        {
          if ((!urlPlaying) || (listedStations)) {changeStation();}
          if (rcInputDigitsMenuEnable) {changeStation();}
          else if (urlPlaying) {webUrlStationPlay();}

          clearFlags();                                             // Czyscimy wszystkie flagi przebywania w różnych menu
          displayRadio();
          u8g2.sendBuffer();
        }
        equalizerMenuEnable = false; // Kasujemy flage ustawiania equalizera
        volumeSet = false; // Kasujemy flage ustawiania głośnosci
      } 
      else if (ir_code == rcCmdKey0) {
        if (presetsOn && !bankMenuEnable && !sdPlayerOLEDActive) {
          // PRESETS: klawisz 0 = stacja 10
          station_nr = 10; changeStation(); clearFlags(); displayRadio(); u8g2.sendBuffer();
        } else {
          #if defined(ENABLE_EQ16) && (ENABLE_EQ16 == 1)
          eq16Mode = !eq16Mode; // Przełącz 3-band <-> 16-band
          displayEqualizer();
          #else
          rcInputKey(0);
          #endif
        }
      }
      else if (ir_code == rcCmdKey1) { if (presetsOn && !bankMenuEnable && !sdPlayerOLEDActive) { station_nr = 1;  changeStation(); clearFlags(); displayRadio(); u8g2.sendBuffer(); } else {rcInputKey(1);}}
      else if (ir_code == rcCmdKey2) { if (presetsOn && !bankMenuEnable && !sdPlayerOLEDActive) { station_nr = 2;  changeStation(); clearFlags(); displayRadio(); u8g2.sendBuffer(); } else {rcInputKey(2);}}
      else if (ir_code == rcCmdKey3) { if (presetsOn && !bankMenuEnable && !sdPlayerOLEDActive) { station_nr = 3;  changeStation(); clearFlags(); displayRadio(); u8g2.sendBuffer(); } else {rcInputKey(3);}}
      else if (ir_code == rcCmdKey4) { if (presetsOn && !bankMenuEnable && !sdPlayerOLEDActive) { station_nr = 4;  changeStation(); clearFlags(); displayRadio(); u8g2.sendBuffer(); } else {rcInputKey(4);}}
      else if (ir_code == rcCmdKey5) { if (presetsOn && !bankMenuEnable && !sdPlayerOLEDActive) { station_nr = 5;  changeStation(); clearFlags(); displayRadio(); u8g2.sendBuffer(); } else {rcInputKey(5);}}
      else if (ir_code == rcCmdKey6) { if (presetsOn && !bankMenuEnable && !sdPlayerOLEDActive) { station_nr = 6;  changeStation(); clearFlags(); displayRadio(); u8g2.sendBuffer(); } else {rcInputKey(6);}}
      else if (ir_code == rcCmdKey7) { if (presetsOn && !bankMenuEnable && !sdPlayerOLEDActive) { station_nr = 7;  changeStation(); clearFlags(); displayRadio(); u8g2.sendBuffer(); } else {rcInputKey(7);}}
      else if (ir_code == rcCmdKey8) { if (presetsOn && !bankMenuEnable && !sdPlayerOLEDActive) { station_nr = 8;  changeStation(); clearFlags(); displayRadio(); u8g2.sendBuffer(); } else {rcInputKey(8);}}
      else if (ir_code == rcCmdKey9) { if (presetsOn && !bankMenuEnable && !sdPlayerOLEDActive) { station_nr = 9;  changeStation(); clearFlags(); displayRadio(); u8g2.sendBuffer(); } else {rcInputKey(9);}}
      else if (ir_code == rcCmdBack) 
      {   
        // Jeśli nagrywanie radia aktywne - zatrzymaj przed wyjściem
        if (g_sdRecorder && g_sdRecorder->isRecording()) {
          g_sdRecorder->stopRecording();
          Serial.printf("[SDRecorder] Recording STOPPED via BACK button. Size: %s\n", 
                        g_sdRecorder->getFileSizeString().c_str());
        }
        
        //f_simpleMode3 = !f_simpleMode3;
        displayDimmer(0);
        clearFlags();   // Zerujemy wszystkie flagi
        displayRadio(); // Ładujemy erkran radia
        u8g2.sendBuffer(); // Wysyłamy bufor na wyswietlacz
        currentSelection = station_nr - 1; // Przywracamy zaznaczenie obecnie grajacej stacji
      }
      else if (ir_code == rcCmdMute) 
      {
        volumeMute = !volumeMute;
        if (volumeMute == true)
        {
          audio.setVolume(0);   
        }
        else if (volumeMute == false)
        {
          audio.setVolume(volumeValue);   
        }
        updateSpeakersPin(); // Aktualizuj pin głośników - przy mute LOW, przy unmute HIGH
        // Dla stylów 5, 6, 10 nie wywoływuj displayRadio() - przekreślony głośnik rysuje się w vuMeterMode
        if (displayMode != 5 && displayMode != 6 && displayMode != 10)
        {
          displayRadio();
        }
        //wsVolumeChange(volumeValue);
        wsVolumeChange();
      }
      else if (ir_code == rcCmdDirect) // Przycisk Direct -> Nagrywanie radia (REC), Menu Bank - update GitHub, Menu Equalizer - reset wartosci, Radio Display - dimmer
      {
        if ((bankMenuEnable == true) && (equalizerMenuEnable == false))// flage można zmienic tylko bedąc w menu wyboru banku
        { 
          bankNetworkUpdate = !bankNetworkUpdate; // zmiana flagi czy aktualizujemy bank z sieci czy karty SD
          bankMenuDisplay(); 
        }
        if ((bankMenuEnable == false) && (equalizerMenuEnable == true))
        {
          toneHiValue = 0;
          toneMidValue = 0;
          toneLowValue = 0;    
          displayEqualizer();
        }
        if ((bankMenuEnable == false) && (equalizerMenuEnable == false) && (volumeSet == false))
        { 
          // PRIORTYTET: Nagrywanie radia (Toggle START/STOP)
          if (g_sdRecorder) {
            g_sdRecorder->toggleRecording(stationName);
            
            if (g_sdRecorder->isRecording()) {
              Serial.printf("[SDRecorder] Recording STARTED: %s\n", g_sdRecorder->getCurrentFileName().c_str());
            } else {
              Serial.printf("[SDRecorder] Recording STOPPED. Size: %s, Time: %s\n", 
                            g_sdRecorder->getFileSizeString().c_str(),
                            g_sdRecorder->getRecordTimeString().c_str());
            }
            displayRadio();
          }
          else if (displayMode == 4) // Jesli jestesmy w display Mode 4 czyli duzy VU wsazowkowy to mozna właczyc buffor audio print.
          {
            debugAudioBuffor=!debugAudioBuffor;
          }
          else
          {
            displayDimmer(!displayDimmerActive); // Dimmer OLED
            Serial.println("Właczono display Dimmer rcCmdDirect");
          }
        }
      }      
      else if (ir_code == rcCmdSrc) 
      {
        displayMode++;
        
        // Pominaj nieistniejący styl: 9 (style 5 i 7 są teraz aktywne - nowe VU)
        if (displayMode == 9) {
          displayMode++; // Przeskocz na następny
        }
        
        // Jeśli doszliśmy do 11+, wróć do 0
        if (displayMode >= 11) {
          displayMode = 0;
        }
        
        // Automatyczne pomijanie wyłączonych stylów
        // Styl 6 - jeśli wyłączony, pomiń
        if (displayMode == 6 && !displayMode6Enabled) {
          displayMode++;  // Przejdź do 7 -> zostanie pomięty -> 8
          if (displayMode == 7) displayMode = 8; // Direct skip
        }
        
        // Styl 8 - jeśli wyłączony, pomiń 
        if (displayMode == 8 && !displayMode8Enabled) {
          displayMode++; // Przejdź do 9 -> zostanie pomięte -> 10
          if (displayMode == 9) displayMode = 10; // Direct skip
        }
        
        // Styl 10 - jeśli wyłączony, pomiń i wróć do 0
        if (displayMode == 10 && !displayMode10Enabled) {
          displayMode = 0;
        }
        
        displayRadio();
        clearFlags();
        ActionNeedUpdateTime = true;
      }
      else if (ir_code == rcCmdRed)   {powerOff();}
      else if (ir_code == rcCmdPower) {
        // PUNKT 1: Power OFF działa również w trybie SD Player
        if (sdPlayerOLEDActive && g_sdPlayerOLED) {
          Serial.println("SD Player: Power OFF from IR remote");
          g_sdPlayerOLED->deactivate();
          sdPlayerOLEDActive = false;
        }
        powerOff();
      }
      else if (ir_code == rcCmdGreen) if (listedStations) {voiceTime();} else {sleepTimerSet();}   
      else if (ir_code == rcCmdBankMinus) 
      {
        if (bankMenuEnable == true) 
        { 
          bank_nr--; 
          if (bank_nr < 1) {bank_nr = bank_nr_max;}
        }       
        bankMenuDisplay();
      }
      else if (ir_code == rcCmdBankPlus) 
      {
        if (bankMenuEnable == true)
        {
          bank_nr++;
          if (bank_nr > bank_nr_max) {bank_nr = 1;}
          bankMenuDisplay();
        }
        else
        {
          // FM+ poza menu banku = toggle PRESETS ON/OFF
          presetsOn = !presetsOn;
          clearFlags();
          displayCenterBigText(presetsOn ? "PRESETS ON" : "PRESETS OFF", 36);
          u8g2.sendBuffer();
          delay(1200);
          displayRadio();
          u8g2.sendBuffer();
          Serial.printf("[PRESETS] %s\n", presetsOn ? "ON" : "OFF");
        }
      }
     
      else if (ir_code == rcCmdAud) {
        #if defined(ENABLE_EQ16) && (ENABLE_EQ16 == 1)
          // Włącz ekran wyboru trybu equalizera (3-band lub 16-band)
          eq16ModeSelectActive = true;
        #endif
        displayEqualizer();
      }
      else if (ir_code == rcCmdHELP) {
        // Przycisk HELP - aktywacja/dezaktywacja SDPlayer
        if (sdPlayerOLEDActive && g_sdPlayerOLED) {
          // Dezaktywacja SDPlayer
          g_sdPlayerOLED->deactivate();
          sdPlayerOLEDActive = false;
          Serial.println("[IR] HELP: SDPlayer dezaktywowany");
        } else if (g_sdPlayerOLED) {
          // Aktywacja SDPlayer
          sdPlayerReturnBank = bank_nr;
          sdPlayerReturnStation = station_nr;
          g_sdPlayerOLED->activate();
          sdPlayerOLEDActive = true;
          eq_analyzer_set_sdplayer_mode(true);
          g_sdPlayerOLED->showSplash();
          delay(1000);
          Serial.println("[IR] HELP: SDPlayer aktywowany");
        }
      }
      else if (ir_code == rcCmdPIP) {
        // Przycisk PIP - przełączanie między EQ3 a EQ16
        #if defined(ENABLE_EQ16) && (ENABLE_EQ16 == 1)
          eq16Mode = !eq16Mode;
          Serial.printf("[IR] PIP: Przełączono na %s\n", eq16Mode ? "EQ16" : "EQ3");
          
          // Wyświetl potwierdzenie na OLED
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_helvB12_tf);
          u8g2.setCursor(60, 32);
          u8g2.print(eq16Mode ? "EQ 16-BAND" : "EQ 3-BAND");
          u8g2.sendBuffer();
          delay(1500);
          displayRadio();
        #else
          Serial.println("[IR] PIP: EQ16 nie jest włączone w kompilacji");
        #endif
      }
      else if (ir_code == rcCmdMINIMA) {
        // Przycisk MINIMA - następny styl SDPlayer
        if (sdPlayerOLEDActive && g_sdPlayerOLED) {
          g_sdPlayerOLED->nextStyle();
          Serial.println("[IR] MINIMA: SDPlayer następny styl");
        } else {
          Serial.println("[IR] MINIMA: SDPlayer nieaktywny");
        }
      }
      else if (ir_code == rcCmdMAXIMA) {
        // Przycisk MAXIMA - poprzedni styl SDPlayer
        if (sdPlayerOLEDActive && g_sdPlayerOLED) {
          g_sdPlayerOLED->prevStyle(); // Poprzedni styl (14→13→...→2→1→14)
          Serial.println("[IR] MAXIMA: SDPlayer poprzedni styl");
        } else {
          Serial.println("[IR] MAXIMA: SDPlayer nieaktywny");
        }
      }
      else if (ir_code == rcCmdWELEMENT) {
        // Przycisk WELEMENT - pause/play dla SDPlayer
        if (sdPlayerOLEDActive && g_sdPlayerWeb) {
          if (g_sdPlayerWeb->isPlaying()) {
            audio.pauseResume();
            g_sdPlayerWeb->setIsPlaying(false);
            Serial.println("[IR] WELEMENT: SDPlayer pauza");
          } else {
            audio.pauseResume();
            g_sdPlayerWeb->setIsPlaying(true);
            Serial.println("[IR] WELEMENT: SDPlayer wznowienie");
          }
        } else {
          Serial.println("[IR] WELEMENT: SDPlayer nieaktywny");
        }
      }
      else if (ir_code == rcCmdWINPOD) {
        // Przycisk WINPOD - stop dla SDPlayer
        if (sdPlayerOLEDActive) {
          audio.stopSong();
          sdPlayerPlayingMusic = false;
          if (g_sdPlayerWeb) {
            g_sdPlayerWeb->setIsPlaying(false);
          }
          Serial.println("[IR] WINPOD: SDPlayer stop");
        } else {
          Serial.println("[IR] WINPOD: SDPlayer nieaktywny");
        }
      }
      else if (ir_code == rcCmdGLOS) {
        // Przycisk GŁOS - otwiera stronę BT w przeglądarce
        if (btModuleEnabled) {
          Serial.println("[IR] GLOS: Otwieranie strony BT -> http://" + currentIP + "/bt");
          // Wyświetl informację na OLED
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_helvB10_tf);
          u8g2.setCursor(20, 20);
          u8g2.print("Strona BT:");
          u8g2.setCursor(10, 40);
          u8g2.print(currentIP + "/bt");
          u8g2.sendBuffer();
          delay(3000);
          displayRadio();
        } else {
          Serial.println("[IR] GLOS: Moduł BT nie jest włączony");
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_helvB10_tf);
          u8g2.setCursor(20, 32);
          u8g2.print("BT nieaktywny");
          u8g2.sendBuffer();
          delay(2000);
          displayRadio();
        }
      }
      else if (ir_code == rcCmdKOL) {
        // Przycisk KOL - toggle modułu BT (włącz/wyłącz)
        btModuleEnabled = !btModuleEnabled;
        Serial.printf("[IR] KOL: Moduł BT %s\n", btModuleEnabled ? "WŁĄCZONY" : "WYŁĄCZONY");
        
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_helvB12_tf);
        u8g2.setCursor(40, 32);
        u8g2.print(btModuleEnabled ? "BT: ON" : "BT: OFF");
        u8g2.sendBuffer();
        delay(2000);
        displayRadio();
      }
      else if (ir_code == rcCmdCHAT) {
        // Przycisk CHAT - wyświetl status BT
        if (g_btWebUI) {
          Serial.println("[IR] CHAT: Wyświetlanie statusu BT");
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_helvB10_tf);
          u8g2.setCursor(10, 15);
          u8g2.print("Status BT:");
          u8g2.setCursor(10, 30);
          u8g2.print(btModuleEnabled ? "Wlaczony" : "Wylaczony");
          u8g2.setCursor(10, 45);
          u8g2.print("IP: " + currentIP + "/bt");
          u8g2.sendBuffer();
          delay(3000);
          displayRadio();
        } else {
          Serial.println("[IR] CHAT: Moduł BT niedostępny");
        }
      }
      else if (ir_code == rcCmdINFO) {
        // Przycisk INFO - informacje o module BT
        Serial.println("[IR] INFO: Informacje BT");
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_helvB10_tf);
        u8g2.setCursor(10, 15);
        u8g2.print("Modul BT UART");
        u8g2.setCursor(10, 30);
        u8g2.print("Status: ");
        u8g2.print(btModuleEnabled ? "OK" : "OFF");
        u8g2.setCursor(10, 45);
        u8g2.print("Config: /bt");
        u8g2.sendBuffer();
        delay(3000);
        displayRadio();
      }
      else if (ir_code == rcCmdPOWROT) {
        // Przycisk POWRÓT - alternatywny dostęp do strony BT (jak GLOS)
        if (btModuleEnabled) {
          Serial.println("[IR] POWROT: Strona BT -> http://" + currentIP + "/bt");
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_helvB10_tf);
          u8g2.setCursor(20, 20);
          u8g2.print("BT WebUI:");
          u8g2.setCursor(10, 40);
          u8g2.print(currentIP + "/bt");
          u8g2.sendBuffer();
          delay(3000);
          displayRadio();
        } else {
          Serial.println("[IR] POWROT: BT wyłączony");
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_helvB10_tf);
          u8g2.setCursor(30, 32);
          u8g2.print("BT: wyłączony");
          u8g2.sendBuffer();
          delay(2000);
          displayRadio();
        }
      }
      else if (ir_code == rcCmdCHPlus) {
        // Przycisk CH+ - zwiększ jasność OLED (funkcja pomocnicza)
        if (displayBrightness < 255) {
          displayBrightness += 20;
          if (displayBrightness > 255) displayBrightness = 255;
          u8g2.setContrast(displayBrightness);
          Serial.printf("[IR] CH+: Jasność OLED: %d\n", displayBrightness);
          
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_helvB10_tf);
          u8g2.setCursor(30, 32);
          u8g2.print("Jasnosc: ");
          u8g2.print(displayBrightness);
          u8g2.sendBuffer();
          delay(1500);
          displayRadio();
        }
      }
      else if (ir_code == rcCmdCHMinus) {
        // Przycisk CH- - zmniejsz jasność OLED (funkcja pomocnicza)
        if (displayBrightness > 10) {
          displayBrightness -= 20;
          if (displayBrightness < 10) displayBrightness = 10;
          u8g2.setContrast(displayBrightness);
          Serial.printf("[IR] CH-: Jasność OLED: %d\n", displayBrightness);
          
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_helvB10_tf);
          u8g2.setCursor(30, 32);
          u8g2.print("Jasnosc: ");
          u8g2.print(displayBrightness);
          u8g2.sendBuffer();
          delay(1500);
          displayRadio();
        }
      }
      else if (ir_code == rcCmdREDLEFT) {
        // Przycisk REDLEFT - restart modułu BT
        if (btModuleEnabled && g_btWebUI) {
          Serial.println("[IR] REDLEFT: Restart modułu BT");
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_helvB10_tf);
          u8g2.setCursor(30, 32);
          u8g2.print("BT: Restart...");
          u8g2.sendBuffer();
          
          // Restart: wyłącz i włącz moduł
          btModuleEnabled = false;
          delay(500);
          btModuleEnabled = true;
          
          u8g2.clearBuffer();
          u8g2.setCursor(30, 32);
          u8g2.print("BT: Gotowy");
          u8g2.sendBuffer();
          delay(1500);
          displayRadio();
        } else {
          Serial.println("[IR] REDLEFT: BT nieaktywny");
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_helvB10_tf);
          u8g2.setCursor(20, 32);
          u8g2.print("BT nieaktywny");
          u8g2.sendBuffer();
          delay(1500);
          displayRadio();
        }
      }
      else if (ir_code == rcCmdGREENL) {
        // Przycisk GREENL - test połączenia BT (sprawdza status modułu i UART)
        Serial.println("[IR] GREENL: Test połączenia BT");
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_helvB10_tf);
        u8g2.setCursor(20, 15);
        u8g2.print("Test BT:");
        
        u8g2.setCursor(10, 30);
        if (btModuleEnabled) {
          u8g2.print("Status: Włączony");
          u8g2.setCursor(10, 45);
          if (g_btWebUI) {
            u8g2.print("UART: Gotowy");
            Serial.println("[IR] GREENL: BT moduł włączony, UART OK");
          } else {
            u8g2.print("UART: Brak");
            Serial.println("[IR] GREENL: BT moduł włączony, UART nie zainicjowany");
          }
        } else {
          u8g2.print("Status: Wyłączony");
          u8g2.setCursor(10, 45);
          u8g2.print("Test: FAILED");
          Serial.println("[IR] GREENL: BT moduł wyłączony");
        }
        
        u8g2.setCursor(10, 60);
        u8g2.setFont(u8g2_font_helvB08_tf);
        u8g2.print("IP: " + currentIP + "/bt");
        u8g2.sendBuffer();
        delay(3000);
        displayRadio();
      }
      else if (ir_code == rcCmdBT && !sdPlayerOLEDActive) {
        // Przycisk BT w trybie radio (w SDPlayer ma inną funkcję)
        if (btModuleEnabled) {
          Serial.println("[IR] BT: Otwieranie strony BT -> http://" + currentIP + "/bt");
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_helvB10_tf);
          u8g2.setCursor(20, 20);
          u8g2.print("Bluetooth:");
          u8g2.setCursor(10, 40);
          u8g2.print(currentIP + "/bt");
          u8g2.sendBuffer();
          delay(3000);
          displayRadio();
        } else {
          Serial.println("[IR] BT: Moduł BT wyłączony");
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_helvB10_tf);
          u8g2.setCursor(20, 32);
          u8g2.print("BT nieaktywny");
          u8g2.sendBuffer();
          delay(2000);
          displayRadio();
        }
      }
      else if (ir_code == rcCmdREC || ir_code == rcCmdBlue) {
        // Przycisk REC/BLUE - nagrywanie strumienia radiowego
        if (g_sdRecorder) {
          // Toggle nagrywania - START/STOP
          g_sdRecorder->toggleRecording(stationName);
          
          // Wyświetl status
          if (g_sdRecorder->isRecording()) {
            Serial.printf("[SDRecorder] Recording STARTED: %s\n", g_sdRecorder->getCurrentFileName().c_str());
            
            // Pokaż komunikat na OLED
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_helvB12_tf);
            u8g2.setCursor(40, 25);
            u8g2.print("NAGRYWANIE");
            u8g2.setFont(u8g2_font_helvB08_tf);
            u8g2.setCursor(10, 45);
            u8g2.print(stationName.substring(0, 30));
            u8g2.sendBuffer();
            delay(2000);
            displayRadio();
          } else {
            Serial.printf("[SDRecorder] Recording STOPPED. Size: %s\n", 
                          g_sdRecorder->getFileSizeString().c_str());
            
            // Pokaż komunikat na OLED
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_helvB12_tf);
            u8g2.setCursor(20, 25);
            u8g2.print("ZATRZYMANO");
            u8g2.setFont(u8g2_font_helvB08_tf);
            u8g2.setCursor(10, 45);
            u8g2.print("Rozmiar: " + g_sdRecorder->getFileSizeString());
            u8g2.sendBuffer();
            delay(2000);
            displayRadio();
          }
        } else {
          Serial.println("[IR] REC/BLUE: SDRecorder nie jest zainicjowany");
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_helvB10_tf);
          u8g2.setCursor(10, 32);
          u8g2.print("Recorder nieaktywny");
          u8g2.sendBuffer();
          delay(2000);
          displayRadio();
        }
      }
      else { Serial.println("Inny przycisk"); }
    }
    else
    {
      Serial.println("Błąd - kod pilota NEC jest niepoprawny!");
      Serial.print("debug IR -> puls 9ms:");
      Serial.print(pulse_duration_9ms);
      Serial.print("  4.5ms:");
      Serial.print(pulse_duration_4_5ms);
      Serial.print("  1690us:");
      Serial.print(pulse_duration_1690us);
      Serial.print("  560us:");
      Serial.println(pulse_duration_560us);    
    }
    ir_code = 0;
    bit_count = 0;
    attachInterrupt(digitalPinToInterrupt(recv_pin), pulseISR, CHANGE);
    
    
    // ---- Symulacja aktywnosci LED IR ----
    #ifdef IR_LED
    digitalWrite(IR_LED, LOW); delay(30); 
    #endif
    
  }

}

// ==================== FUNKCJE OBSŁUGI TAS5805 POWERAMP I2C ====================
#ifdef TAS5805

void tasWrite(uint8_t reg, uint8_t val)
{
  Serial.print("TAS Write: ");
  Serial.print(reg, HEX);
  Serial.print(" ");
  Serial.println(val, HEX);

  Wire.beginTransmission(TAS5805_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
   
void tasRead(uint8_t reg)
{
  Wire.beginTransmission(TAS5805_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);   // repeated start

  Wire.requestFrom(TAS5805_ADDR, (uint8_t)1);

  Serial.print("TAS Read: ");
  Serial.print(reg, HEX);
  Serial.print(" = ");

  if (Wire.available()) 
    Serial.println(Wire.read(), HEX);
  else
    Serial.println("ERR");
}

void tas5805init()
{
  tasWrite(0x01, 0x01);      // Software reset
  delay(1);

  tasWrite(0x00, 0x00);      // Book 0
  tasWrite(0x7F, 0x00);      // Page 0
  tasWrite(0x03, 0x02);      // switch to High-Z (powinien byc po resecie równiez)

  tasWrite(0x30, 0x01);      // SDOUT - pre-proccessing, DSP input
  delay(10);
  
  tasWrite(0x02, 0x00);      // RES 000 - 768k 0 0-BTL 00-BD Mode

  tasWrite(0x28, 0x50);      // 1001 BCK Ratio 256FS/ 00 autodedect  0000 FS_mode Audio

  tasWrite(0x33, 0x02);

  tasWrite(0x4c, 0x30);      // Master volume = -48 dB

  tasWrite(0x03, 0x03);      // PLAY
  delay(5);
}

void tas5805vol(uint8_t volume)
{
  tasWrite(0x4c, volume);
}

#endif
// ==================== KONIEC FUNKCJI TAS5805 ====================



//####################################################################################### SETUP ####################################################################################### //

void setup() 
{

  // Inicjalizuj komunikację szeregową (Serial)
  Serial.begin(115200);
  delay(600);
  //while (!Serial) {delay(10);}
  
  // ========== OPTYMALIZACJA CPU DLA STRUMIENIOWANIA ==========
  // Upewnij się że CPU działa na maksymalnej częstotliwości
  setCpuFrequencyMhz(240);  // ESP32S3 @ 240MHz - max wydajność
  Serial.print("[CPU] Frequency set to: ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");
  // ===========================================================
  
  // DIAGNOSTYKA RESTART: Sprawdź powód restartu
  esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.println("========================================");
  Serial.print("[RESTART REASON] ");
  switch (resetReason) {
    case ESP_RST_POWERON:   Serial.println("Power-on reset"); break;
    case ESP_RST_EXT:       Serial.println("External reset"); break;
    case ESP_RST_SW:        Serial.println("Software reset"); break;
    case ESP_RST_PANIC:     Serial.println("PANIC RESET - Exception/Panic!"); break;
    case ESP_RST_INT_WDT:   Serial.println("WATCHDOG RESET - Interrupt watchdog!"); break;
    case ESP_RST_TASK_WDT:  Serial.println("WATCHDOG RESET - Task watchdog!"); break;
    case ESP_RST_WDT:       Serial.println("WATCHDOG RESET - Other watchdog!"); break;
    case ESP_RST_DEEPSLEEP: Serial.println("Deep sleep reset"); break;
    case ESP_RST_BROWNOUT:  Serial.println("BROWNOUT RESET - Power issue!"); break;
    case ESP_RST_SDIO:      Serial.println("SDIO reset"); break;
    default:                Serial.printf("Unknown reset (code: %d)", resetReason); break;
  }
  Serial.printf("[MEMORY] Free Heap: %u bytes, Min Free: %u bytes\n", 
                ESP.getFreeHeap(), ESP.getMinFreeHeap());
  Serial.println("========================================");
    
  /*
  Serial.println("=== START RS232 ===");
  for (int i=0; i<5; i++) 
  {
    Serial.printf("Line %d\n", i);
    delay(100);
  }
  */

  uint64_t chipid = ESP.getEfuseMac();
  Serial.println("");
  Serial.println("------------------ START of Evo Web Radio --------------------");
  Serial.println("-                                                            -");
  Serial.printf("--------- ESP32 SN: %04X%08X,  FW Ver.: %s ---------\n",(uint16_t)(chipid >> 32), (uint32_t)chipid, softwareRev);
  Serial.println("-         FW Config  use SD:" + String(useSD) + ",  Use Two Encoders:" + String(use2encoders) + "           -");
  Serial.println("-                                                            -");
  Serial.println("- Code source: https://github.com/dzikakuna/ESP32_radio_evo3 -");
  Serial.println("--------------------------------------------------------------");
  
  // ----------------- Inicjalizacja TAS5805 PowerAmp I2C -----------------
  #ifdef TAS5805
    Serial.println("[TAS5805] Initializing I2C PowerAmp...");
    Wire.begin(I2C_DATA, I2C_CLK);   // inicjalizacja I2C
    Wire.setClock(400000);           // I2C 400kHz
    delay(200);
    tas5805init();
    delay(100);
    Serial.println("[TAS5805] Initialized successfully");
  #endif
  
  // Inicjalizacja SPI z nowymi pinami dla czytnika kart SD
  customSPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);  // SCLK = 45, MISO = 21, MOSI = 48, CS = 47


  psramData = (unsigned char *)ps_malloc(PSRAM_lenght * sizeof(unsigned char));

  if (psramInit()) {
    Serial.println("Pamiec PSRAM zainicjowana poprawnie.");
    Serial.print("Dostepna pamiec PSRAM: ");
    Serial.println(ESP.getPsramSize());
    Serial.print("Wolna pamiec PSRAM: ");
    Serial.println(ESP.getFreePsram());


  } else {
    Serial.println("debug-- BLAD Pamieci PSRAM");
  }

  wifiManager.setHostname(hostname);
  WiFi.setSleep(false);

  EEPROM.begin(128);

  // ----------------- ENKODER 2 podstaowwy -----------------
  pinMode(CLK_PIN2, INPUT_PULLUP);           // Konfiguruj piny enkodera 2 jako wejścia
  pinMode(DT_PIN2, INPUT_PULLUP); 
  pinMode(SW_PIN2, INPUT_PULLUP);            // Inicjalizacja przycisków enkoderów jako wejścia
  prev_CLK_state2 = digitalRead(CLK_PIN2);   // Odczytaj początkowy stan pinu CLK enkodera
  

  // ----------------- ENKODER 1 dodatkowy (jesli włączony) -----------------
  #ifdef twoEncoders
    pinMode(CLK_PIN1, INPUT_PULLUP);           // Konfiguruj piny enkodera 1 jako wejścia
    pinMode(DT_PIN1, INPUT_PULLUP);
    pinMode(SW_PIN1, INPUT_PULLUP);           // Inicjalizacja przycisków enkoderów jako wejścia
    prev_CLK_state1 = digitalRead(CLK_PIN1);  // Odczytaj początkowy stan pinu CLK enkodera
  #endif  

  // ----------------- SW_POWER dodatkowy (jesli włączony) -----------------
  #ifdef SW_POWER
  pinMode(SW_POWER, INPUT_PULLUP);
  #endif

  // --------LED STANDBY dla stanu HIGH w trybie Power ON ----------------
  pinMode(STANDBY_LED, OUTPUT);
  //digitalWrite(STANDBY_LED, HIGH); // Dla LED właczonego w trybie Power ON
  digitalWrite(STANDBY_LED, LOW); // Dla LED wylaczonego w trybie Power ON, włączonego w trybie Standby

  // -------- SPEAKERS PIN - wzmacniacz domyslnie ON --------
  pinMode(SPEAKERS_PIN, OUTPUT);
  digitalWrite(SPEAKERS_PIN, HIGH); // Domyślnie głośniki włączone

  // -------- LED STANDBY: miganie podczas startu --------
  ledBlinker.attach_ms(500, []() {
    static bool s = false;
    s = !s;
    digitalWrite(STANDBY_LED, s ? HIGH : LOW);
  });
  Serial.println("[LED] Standby LED: miganie podczas startu");

  // ----------------- LED CS karty SD - klon (jesli włączony) -----------------
  #ifdef SD_LED
  pinMode(SD_LED, OUTPUT);
   // PIN, CLONE PIN, INVERTED, EN_INV -Klonowanie sygnału CS kontrolera VirtualSPI (VSPI) na LED aby miec informacje o aktywnosci karty z czytnika z tyłu PCB
  gpio_matrix_out(SD_LED, SPI3_CS0_OUT_IDX, false, false);
  #endif


  // ----------------- IR ODBIORNIK - Konfiguracja pinu -----------------
  pinMode(recv_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(recv_pin), pulseISR, CHANGE);


  // ----------------- PRZETWORNIK ADC KLAWIATURA - Konfiguracja -----------------
  analogReadResolution(12);                 // Ustawienie rozdzielczości na 11 bits (zakres 0-4095)
  analogSetAttenuation(ADC_11db);           // Wzmocnienie ADC (0dB dla zakresu 0-3.3V)

  
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);  // Konfiguruj pinout dla interfejsu I2S audio
  Audio::audio_info_callback = my_audio_info; // Przypisanie własnej funkcji callback do obsługi zdarzeń i informacji audio
  
  // ========== 16-BAND GRAPHIC EQUALIZER INITIALIZATION (PART 1) ==========
  #if defined(ENABLE_EQ16) && (ENABLE_EQ16 == 1)
    APMS_EQ16::init(&audio);                    // Inicjalizuj equalizer z referencją do audio
    APMS_EQ16::setFeatureEnabled(true);         // Włącz funkcję equalizera w menu
    APMS_EQ16::setEnabled(true);                // Włącz przetwarzanie audio przez EQ
    Serial.println("[EQ16] 16-band Graphic Equalizer initialized (waiting for storage)");
  #else
    Serial.println("[EQ16] 16-band EQ disabled (ENABLE_EQ16=0)");
  #endif
  // NOTE: EQ16_loadFromSD() will be called AFTER storage initialization
  // ==============================================================

  
  // ========== MAKSYMALNE OPTYMALIZACJE DLA FLAC STREAMING ==========
  // 1. AUDIO TASK NA DEDYKOWANYM CORE 1 (separacja od WiFi stack)
  //    WiFi działa na Core 0, Arduino loop() na Core 1
  //    Audio task przeniesiony na Core 1 = ZERO konkurencji z WiFi
  audio.setAudioTaskCore(1);  // Core 1: dedykowany dla audio i dekodowania FLAC
  
  // 2. Zwiększone timeouty dla stabilności
  audio.setConnectionTimeout(5000, 8000);      // HTTP: 5s, HTTPS: 8s
  
  // UWAGA: WiFi.setSleep(false) już ustawione - krytyczne!
  // UWAGA: WiFi.setTxPower(19.5dBm) - maksymalna moc
  // UWAGA: PSRAM 640KB buffer - automatycznie używany
  // UWAGA: TCP window 64KB + send buffer 64KB w platformio.ini
  // UWAGA: CPU @ 240MHz - wymuszone w setup()
  // =================================================================
  
  audio.setVolume(0);
  
  // Inicjalizuj interfejs SPI wyświetlacza
  SPI.begin(SPI_SCK_OLED, SPI_MISO_OLED, SPI_MOSI_OLED);
  SPI.setFrequency(2000000);


  // Inicjalizuj wyświetlacz i odczekaj 250 milisekund na włączenie
  u8g2.begin();
  delay(250);

  // ----------------- KARTA SD / PAMIEC SPIFFS/LittleFS - Inicjalizacja -----------------
  // Inicjalizacja STORAGE
  #ifdef AUTOSTORAGE
    if (!initStorage()) 
    {
      while (1);
    }
  #else
    // ----------------- KARTA SD / PAMIEC SPIFFS - Inicjalizacja -----------------
    if (!STORAGE_BEGIN())
    {
      Serial.println("Błąd inicjalizacji pamieci STORAGE!"); // Informacja o problemach lub braku karty SD
      noSDcard = true;                                // Flaga braku karty SD
    }
    else
    {
      Serial.println("Pamiec STORAGE zainicjalizowana pomyślnie.");
      noSDcard = false;
    }
  #endif
  delay(50);

  // ========== 16-BAND GRAPHIC EQUALIZER INITIALIZATION (PART 2) ==========
  // Load EQ16 settings from SD card (must be done AFTER storage initialization)
  #if defined(ENABLE_EQ16) && (ENABLE_EQ16 == 1)
    if (!noSDcard) {
      EQ16_loadFromSD();                        // Załaduj ostatnie ustawienia z SD
      Serial.println("[EQ16] Settings loaded from SD card");
    } else {
      Serial.println("[EQ16] No SD card - using default flat EQ");
    }
  #endif
  // ========================================================================

  loadSDPlayerSessionState();
  
  // PUNKT 3: PERSISTENT STATE - sprawdź czy SD Player był aktywny przed wyłączeniem
  bool shouldAutoStartSDPlayer = false;
  if (STORAGE.exists(sdPlayerActiveStateFile)) {
    File stateFile = STORAGE.open(sdPlayerActiveStateFile, FILE_READ);
    if (stateFile) {
      String state = stateFile.readStringUntil('\n');
      state.trim();
      shouldAutoStartSDPlayer = (state == "1");
      stateFile.close();
      Serial.printf("SD Player: Auto-start state loaded: %s\n", shouldAutoStartSDPlayer ? "YES" : "NO");
    }
  }
  
  // Odczyt konfiguracji
  readConfig();          
  if (configExist == false) { saveConfig(); readConfig();} // Jesli nie ma pliku config.txt to go tworzymy
  
  // KRYTYCZNE: Odczyt konfiguracji analizatora z /analyzer.cfg
  // Sprawdź czy plik analyzer.cfg istnieje (używaj STORAGE, nie SD/SPIFFS!)
  Serial.println("=== INICJALIZACJA ANALIZATORA ===");
  bool analyzerCfgExists = STORAGE.exists("/analyzer.cfg");
  Serial.printf("Plik analyzer.cfg istnieje: %s\n", analyzerCfgExists ? "TAK" : "NIE");
  
  analyzerStyleLoad();  // Wczytaj zapisane ustawienia analizatora lub użyj domyślnych
  
  if (!analyzerCfgExists) {
    // Plik nie istnieje - zapisz domyślne wartości
    Serial.println("Tworzenie analyzer.cfg z domyslnymi wartosciami...");
    analyzerStyleSave();
    Serial.println("Analyzer: Created /analyzer.cfg with default values");
  } else {
    Serial.println("Analyzer: Configuration loaded from /analyzer.cfg");
  }

  if ((esp_reset_reason() != ESP_RST_POWERON) || (!f_sleepAfterPowerFail))
  {
    u8g2.clearBuffer();
    if (f_logoSlowBrightness) {u8g2.setContrast(0);}

    if (!loadXBM("/logo.xbm")) 
    {
      Serial.println("No logo in storage memory, loading notes...");
      u8g2.drawXBMP(0, 5, notes_width, notes_height, notes);  // obrazek - nutki
      u8g2.setFont(u8g2_font_fub14_tf);
      u8g2.drawStr(38, 17, "Evo Internet Radio");
      u8g2.sendBuffer();
    }
    else
    {
      
      u8g2.drawXBMP(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, logo_bits); // obrazek - logo z pamieci
      u8g2.sendBuffer();     
      f_logo = 1;
    }   

    // ---- POWOLNE ROZJASNIANIE LOGO NUTKI / WGRANE LOGO ----
    if (f_logoSlowBrightness)
    {
      for (int i = 0; i <= displayBrightness; i++)
        {
          u8g2.setContrast(i);
          //uint8_t corrected = (i * i) / 255;   // gamma ≈ 2.0
          //u8g2.setContrast(corrected);
          delay(10); 
        }
      u8g2.setContrast(displayBrightness); // Dodatkowe przeładowanie jasnosci          
    }
     
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(208, 62, softwareRev);
    u8g2.sendBuffer();
  }
  
  #ifndef AUTOSTORAGE
  // Informacja na wyswietlaczu o problemach lub braku karty SD
  if (noSDcard) // flaga noSDcard ustawia sie jesli nie wykryto karty
  {
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(5, 62, "Error - Please check SD card");
    u8g2.setDrawColor(0);
    u8g2.drawBox(212, 0, 44, 45);
    u8g2.setDrawColor(1);
    u8g2.drawXBMP(220, 3, 30, 40, sdcard);  // ikona SD karty
    u8g2.sendBuffer();
  }
  #endif

  u8g2.setFont(spleen6x12PL);
  if ((esp_reset_reason() == ESP_RST_POWERON) && (f_sleepAfterPowerFail)) {u8g2.drawStr(5, 50, "Wakeup after power loss, syncing NTP");}
  if (f_logo) {u8g2.drawStr(5, 62, "Evo Web Radio, connecting...");} else {u8g2.drawStr(5, 62, "Connecting to network...    ");}
  u8g2.sendBuffer();
  //delay(1000); // Popatrzmy dłuzej na nutki lub logo :)
  
  #ifdef twoEncoders
    button1.setDebounceTime(50);  // Ustawienie czasu debouncingu dla przycisku enkodera 2
  #endif

  button2.setDebounceTime(50);  // Ustawienie czasu debouncingu dla przycisku enkodera 2
  

    
  // Inicjalizacja WiFiManagera
  wifiManager.setConfigPortalBlocking(false);

  // ---- ODCZYTY RÓZNYCH USTAWIEN Z KARTY SD / PAMIECI SPIFFS ----
  readStationFromSD();       // Odczytujemy zapisaną ostanią stację i bank z karty SD /EEPROMu
  readEqualizerFromSD();     // Odczytujemy ustawienia filtrów equalizera z karty SD 
  readVolumeFromSD();        // Odczytujemy nastawę ostatniego poziomu głośnosci z karty SD /EEPROMu
  readAdcConfig();           // Odczyt konfiguracji klawitury ADC
  readRemoteControlConfig(); // Odczyt konfiguracji pilota IR
  assignRemoteCodes();       // Przypisanie kodów pilota IR
  readTimezone();

  audio.setVolumeSteps(maxVolume);
  //audio.setVolume(0);                  // Ustaw głośność na podstawie wartości zmiennej volumeValue w zakresie 0...21
  
  // ----------------- EKRAN - JASNOŚĆ -----------------
  u8g2.setContrast(displayBrightness); 

  // -------------------- RECOVERY MODE --------------------
  recoveryModeCheck();
  
  // -------------------- SDPLAYER INIT --------------------
  SDPlayerOLED_init(&u8g2);  // Inicjalizacja SD Player z OLED
  Serial.println("SDPlayer: Initialized");
  
  // -------------------- SDRECORDER INIT --------------------
  SDRecorder_init();  // Inicjalizacja modułu nagrywania
  
  // PUNKT 3: AUTO-START SD PLAYER jeśli był aktywny przed wyłączeniem
  if (shouldAutoStartSDPlayer && g_sdPlayerOLED) {
    Serial.println("SD Player: Auto-starting from previous session...");
    g_sdPlayerOLED->activate();
    sdPlayerOLEDActive = true;
  }
  
  // -------------------- BLUETOOTH WEBUI INIT --------------------
  BTWebUI_init();  // Inicjalizacja Bluetooth WebUI (zawsze dostępne)
  Serial.println("BTWebUI: Initialized");
  
  // -------------------- DLNA INIT --------------------
  #ifdef USE_DLNA
  Serial.println("[DLNA] Initializing worker and service...");
  
  // Inicjalizacja mutexów i kolejek
  if (!g_dlnaHttpMux) {
    g_dlnaHttpMux = xSemaphoreCreateMutex();
    Serial.println("[DLNA] HTTP mutex created");
  }
  
  if (!g_spiffsMux) {
    g_spiffsMux = xSemaphoreCreateMutex();
    Serial.println("[DLNA] SPIFFS mutex created");
  }
  
  // Uruchom DLNA worker task
  dlna_service_begin();
  dlna_worker_start();
  Serial.println("[DLNA] Worker and service initialized");
  #endif
  
  // -------------------- BLUETOOTH KCX INIT --------------------
  // BT_init(&u8g2);  // Inicjalizacja modułu Bluetooth KCX_BT_EMITTER
  
  previous_bank_nr = bank_nr;  // wyrównanie wartości przy stacie radia aby nie podmienic bank_nr na wartość 0 po pierwszym upływie czasu menu
  Serial.print("debug1...wartość bank_nr:");
  Serial.println(bank_nr);
  Serial.print("debug1...wartość previous_bank_nr:");
  Serial.println(previous_bank_nr);
  Serial.print("debug1...wartość station_nr:");
  Serial.println(station_nr);

  // Rozpoczęcie konfiguracji Wi-Fi i połączenie z siecią, jeśli konieczne
  // Animacja gwiazdek PRZED uruchomieniem WiFi
  Serial.println("Starting WiFi animation...");
  for (int frame = 0; frame < 100; frame++) {  // 100 klatek = 5 sekund przy 50ms
    drawStarField(&u8g2, frame);
    delay(50); // ~20 FPS
  }
  
  Serial.println("[WiFi] Konfiguracja WiFiManager...");
  
  // OPTYMALIZACJA WIFI - szybsze łączenie i stabilność strumieniowania
  WiFi.mode(WIFI_STA);              // Ustaw tryb Station
  WiFi.setSleep(false);              // KRYTYCZNE: Wyłącz WiFi sleep mode (zapobiega AUTH_EXPIRE)
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // Maksymalna moc WiFi dla stabilności strumieniowania FLAC/AAC
  WiFi.setAutoReconnect(true);       // Automatyczne ponowne łączenie
  WiFi.persistent(true);             // Zapisuj konfigurację WiFi w flash
  
  wifiManager.setConnectTimeout(20); // 20 sekund timeout (wystarczy bez błędów auth expire)
  wifiManager.setTimeout(180); // 3 minuty timeout portalu konfiguracyjnego
  wifiManager.setConfigPortalBlocking(false);
  
  Serial.println("[WiFi] Próba połączenia z zapisaną siecią...");
  Serial.printf("[WiFi] SSID: %s\n", wifiManager.getWiFiSSID().c_str());
  wifiManager.autoConnect("EVO-Radio");
  
  // PUNKT 2: NON-BLOCKING WIFI - czekaj max 15 sekund na połączenie
  Serial.println("[WiFi] Waiting for connection (max 15s)...");
  unsigned long wifiStartTime = millis();
  int wifiRetryCount = 0;
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStartTime < 15000)) {
    delay(500);
    Serial.print(".");
    wifiRetryCount++;
    if (wifiRetryCount % 10 == 0) {
      Serial.printf(" %d%%\n", (int)((millis() - wifiStartTime) / 150));
    }
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) 
  {
    Serial.println("[WiFi] ✓ Połączono z siecią WiFi");
    Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
    currentIP = WiFi.localIP().toString();  //konwersja IP na string
    
    if (MDNS.begin(hostname)) { Serial.println("mDNS wystartowal, adres: " + String(hostname) + ".local w przeglądarce"); MDNS.addService("http", "tcp", 80);}
        
    //configTzTime("CET-1CEST,M3.5.0/2,M10.5.0/3", ntpServer1, ntpServer2);
    configTzTime(timezone.c_str(), ntpServer1, ntpServer2);
    struct tm timeinfo;
    int retry = 0;
    Serial.println("debug RTC -> synchronizacja NTP pierwszy raz od startu");
    while (!getLocalTime(&timeinfo) && retry < 10) 
    {
      Serial.println("debug RTC -> Czekam na synchronizację czasu NTP...");
      delay(1000);
      retry++;
    }


    timer1.attach(0.5, updateTimerFlag);  // Ustaw timer, aby wywoływał funkcję updateTimer
    //timer1.attach(0.7, updateTimerFlag);  // Ustaw timer, aby wywoływał funkcję updateTimer
    timer2.attach(1, displayDimmerTimer);
    //timer3.attach(60, sleepTimer);   // Ustaw timer3,

    if ((esp_reset_reason() == ESP_RST_POWERON) && (f_sleepAfterPowerFail)) // sprawdzamy czy radio wlaczylo sie po braku zasilania jesli tak to idziemy do spania gdy zasilanie powroci
    {
      Serial.println("debug PWR -> power OFF po powrocie zasilania");
      Serial.print("debug PWR -> Funkcja zegara f_displayPowerOffClock: ");
      Serial.println(f_displayPowerOffClock);
      //prePowerOff();
      powerOff();
    }


    uint8_t temp_station_nr = station_nr; // Chowamy na chwile odczytaną stacje z karty SD aby podczas ładowania banku nie zmienic ostatniej odtwarzanej stacji
    fetchStationsFromServer();            // Ładujemy liste stacji z karty SD  lub serwera GitHub
    station_nr = temp_station_nr ;        // Przywracamy numer po odczycie stacji
    changeStation();                      // Ustawiamy stację

    // -------- LED STANDBY: radio gra - gasniemy LED --------
    ledBlinker.detach();
    digitalWrite(STANDBY_LED, LOW);       // LED wygaszony - radio odtwarza
    updateSpeakersPin();                  // Aktywuj pin głośników
    Serial.println("[LED] Standby LED OFF - radio odtwarza");
    
    // ########################################### WEB Server ######################################################
    
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      String userAgent = request->header("User-Agent");
      String html =""; 
      //html.reserve(48000);  // rezerwuje bufor (np. 24KB)

      if (userAgent.indexOf("Mobile") != -1 || userAgent.indexOf("Android") != -1 || userAgent.indexOf("iPhone") != -1) 
      {
        html = stationBankListHtmlMobile();
      } 
      else //Jestemy na komputerze 
      {
        html = stationBankListHtmlPC();
      }
      
      String finalhtml = String(index_html) + html;  // Składamy cześć stałą html z częscią generowaną dynamicznie
      //request->send_P(200, "text/html", finalhtml.c_str());
      //request->send(200, "text/html", finalhtml.c_str());
      request->send(200, "text/html", finalhtml);
    });

    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
          request->send(STORAGE, "/favicon.ico", "image/x-icon");       
    });
    
    server.on("/icon.png", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(STORAGE, "/icon.png", "image/png");       
    });

        server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
          request->send(STORAGE, "favicon.ico", "image/x-icon");       
    });
    
    server.on("/icon.png", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(STORAGE, "icon.png", "image/png");       
    });

    server.on("/menu", HTTP_GET, [](AsyncWebServerRequest *request){   
      String html = FPSTR(menu_html);
      request->send(200, "text/html", html);
    });
    
    server.on("/firmwareota", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      request->send(200, "text/plain", "Update done");
    },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) 
    {

      static size_t total = 0;
      static size_t contentLength = 0;
      static unsigned long lastPrint = 0;

      if (index == 0) 
      {
        // Tylko przy pierwszym pakiecie
        Serial.printf("OTA Start: %s\n", filename.c_str());
        total = 0;
        contentLength = request->contentLength();  // pełna długość pliku
        lastPrint = millis();

        size_t sketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;

        if (contentLength > sketchSpace) 
        {
          Serial.printf("Error: firmware too big (%u > %u bytes)\n", contentLength, sketchSpace);
          return;
        }

        if (!Update.begin(sketchSpace)) 
        {
          Update.printError(Serial);
          return;
        }
      }

      if (Update.write(data, len) != len) 
      {
        Update.printError(Serial);
      } 
      else 
      {
        total += len;
          
        // Wyświetlaj pasek postępu co 300 ms
        unsigned long now = millis();
        if (now - lastPrint >= 200 || final) 
        {     
          int percent = (total * 100) / contentLength;
          Serial.printf("Progress: %d%% (%u/%u bytes)\n", percent, total, contentLength);
          u8g2.setCursor(5, 24); u8g2.print("File: " + String(filename));
          u8g2.setCursor(5, 36); u8g2.print("Flashing... " + String(total / 1024) + " KB");
          u8g2.sendBuffer();
          lastPrint = now;
        }
      }
      if (final) 
      {
        if (Update.end(true)) 
        {
          request->send(200, "text/plain", "Update done - reset in 3sec");
          Serial.println("Update complete");
          u8g2.setCursor(5, 48); u8g2.print("Completed - reset in 3sec");
          u8g2.sendBuffer();
              
          AsyncWebServerRequest *reqCopy = request;
          reqCopy->onDisconnect([]() 
          {
            delay(3000);
            //ESP.restart();
            REG_WRITE(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_SW_SYS_RST);
          });
        } 
        else 
        {
          Update.printError(Serial);
          request->send(500, "text/plain", "Update failed");
        }
      }
    });

    server.on("/editor", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(STORAGE, "/editor.html", "text/html"); //"application/octet-stream");
    });

     server.on("/browser", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request->send(STORAGE, "/browser.html", "text/html"); //"application/octet-stream");
    });

    server.on("/ota", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      
      timeDisplay = false;
      displayActive = true;
      fwupd = true;
      
      ws.closeAll();
      audio.stopSong();
      delay(250);
      if (!f_saveVolumeStationAlways){saveStationOnSD(); saveVolumeOnSD();}
      
      for (int i = 0; i < 3; i++) 
      {
        u8g2.clearBuffer();
        delay(25);
      }        
      unsigned long now = millis();
      u8g2.clearBuffer();
      u8g2.setDrawColor(1);
      u8g2.setFont(spleen6x12PL);     
      u8g2.setCursor(5, 12); u8g2.print("Evo Radio, OTA Firwmare Update");
      u8g2.sendBuffer();

      request->send(200, "text/html", FPSTR(ota_html));
      //request->send(SD, "/ota.html", "text/html"); //"application/octet-stream");
    });

    server.on("/page1", HTTP_GET, [](AsyncWebServerRequest *request)
    { // Strona do celow testowych ladowana ze STORAGE (karty Sd lub pamieci FS)
      request->send(STORAGE, "/page1.html", "text/html"); //"application/octet-stream");
    });

    server.on("/playurl", HTTP_GET, [](AsyncWebServerRequest *request)
    { 
      //String html =""; 
      //String finalhtml = String(urlplay_html);  // Składamy cześć stałą html z częscią generowaną dynamicznie
      //request->send(200, "text/html", finalhtml);
      request->send(200, "text/html", FPSTR(urlplay_html));
      //request->send(STORAGE, "/playurl.html", "text/html");
    });

    server.on("/edit", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      if (!request->hasParam("filename")) 
      {
          request->send(400, "text/plain", "Brak parametru filename");
          return;
      }

      String filename = request->getParam("filename")->value();
      if (!filename.startsWith("/")) { filename = "/" + filename;}
      File file = STORAGE.open(filename, FILE_READ);
      if (!file) {
          request->send(404, "text/plain", "Nie można otworzyć pliku");
          return;
      }

      String html = "<html><head><title>Edit File</title></head><body>";
      html += "<h2 style='font-size: 1.3rem;'>Editing: " + filename + "</h2>";
      html += "<form method='POST' action='/save'>";
      html += "<input type='hidden' name='filename' value='" + filename + "'>";
      html += "<textarea name='content' rows='100' cols='130'>";

      while (file.available()) 
      {
          html += (char)file.read();
      }

      file.close();

      html += "</textarea><br>";
      html += "<input type='submit' value='Save'>";
      html += "</form>";
      html += "<p><a href='/list'>Back to list</a></p>";
      html += "</body></html>";

      request->send(200, "text/html", html);
    });

    server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      if (!request->hasParam("filename", true) || !request->hasParam("content", true)) 
      {
        request->send(400, "text/plain", "Brakuje danych");
        return;
      }

      String filename = request->getParam("filename", true)->value();
      String content = request->getParam("content", true)->value();

      File file = STORAGE.open(filename, FILE_WRITE);
      if (!file) 
      {
        request->send(500, "text/plain", "Nie można zapisać pliku");
        return;
      }

      file.print(content);
      file.close();

      request->send(200, "text/html", "<p>File Saved!</p><p><a href='/list'>Back to list</a></p>");
    });

    server.on("/list", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      String html = FPSTR(list_html);
      
      //html += "<body><h2 style='font-size: 1.3rem;'>Evo Web Radio - SD / SPIFFS:</h2>" + String("\n");
      html += "<body><h2 style='font-size: 1.3rem;'>Evo Web Radio - ";
      html += storageTextName;
      html += "</h2>" + String("\n");
      
      //if (useSD) html += "<body><h2 style='font-size: 1.3rem;'>Evo Web Radio - SD card:</h2>" + String("\n");
      //else html += "<body><h2 style='font-size: 1.3rem;'>Evo Web Radio - SPIFFS memory:</h2>" + String("\n");

      html += "<form action=\"/upload\" method=\"POST\" enctype=\"multipart/form-data\">";
      html += "<input type=\"file\" name=\"file\">";
      html += "<input type=\"submit\" value=\"Upload\">";
      html += "</form>";
      html += "<div class=\"columnlist\">" + String("\n");
      html += "<table><tr><th>File</th><th>Size (Bytes)</th><th>Action</th></tr>";

      listFiles("/", html);
      html += "</table></div>";
      html += "<p style='font-size: 0.8rem;'><a href='/menu'>Go Back</a></p>" + String("\n"); 
      html += "</body></html>";     
      request->send(200, "text/html", html);
      });

      server.on("/delete", HTTP_POST, [](AsyncWebServerRequest *request) {
      String filename = "/"; // Dodajemy sciezke do głownego folderu
        if (request->hasParam("filename", true)) {
          filename += request->getParam("filename", true)->value();
          if (STORAGE.remove(filename.c_str())) {
              Serial.println("Plik usunięty: " + filename);
          } else {
              Serial.println("Nie można usunąć pliku: " + filename);
          }
        }
        request->redirect("/list"); // Przekierowujemy na stronę listy
      });

      server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        //String html = String(config_html);
        String html = FPSTR(config_html);  // czyta z PROGMEM

        // liczby
        html.replace(F("%D10"), String(vuRiseSpeed));
        html.replace(F("%D11"), String(vuFallSpeed));
        html.replace(F("%D12"), String(dimmerSleepDisplayBrightness));
        
        html.replace(F("%D1"), String(displayBrightness));
        html.replace(F("%D2"), String(dimmerDisplayBrightness));
        html.replace(F("%D3"), String(displayAutoDimmerTime));
        html.replace(F("%D4"), String(vuMeterMode));
        html.replace(F("%D5"), String(encoderFunctionOrder));
        html.replace(F("%D6"), String(displayMode));
        html.replace(F("%D7"), String(vuMeterRefreshTime));
        html.replace(F("%D8"), String(scrollingRefresh));
        html.replace(F("%D9"), String(displayPowerSaveTime));
        html.replace(F("%A1"), String(analyzerStyles)); // analyzerStyles
        html.replace(F("%A2"), String(analyzerPreset)); // analyzerPreset

        //Opcje CheckBox
        html.replace(F("%S11_checked"), maxVolumeExt ? " checked" : "");
        html.replace(F("%S13_checked"), vuPeakHoldOn ? " checked" : "");
        html.replace(F("%S15_checked"), vuSmooth ? " checked" : "");
        html.replace(F("%S17_checked"), stationNameFromStream ? " checked" : "");
        html.replace(F("%S19_checked"), f_displayPowerOffClock ? " checked" : "");
        html.replace(F("%S21_checked"), f_sleepAfterPowerFail ? " checked" : "");
        html.replace(F("%S22_checked"), f_volumeFadeOn ? " checked" : "");
        html.replace(F("%S23_checked"), f_saveVolumeStationAlways ? " checked" : "");
        html.replace(F("%S24_checked"), f_powerOffAnimation ? " checked" : "");      
        html.replace(F("%S25_checked"), analyzerEnabled ? " checked" : "");
        html.replace(F("%S26_checked"), btModuleEnabled ? " checked" : "");
        html.replace(F("%S27_checked"), displayMode6Enabled ? " checked" : "");
        html.replace(F("%S45_checked"), presetsOn ? " checked" : "");
        html.replace(F("%S46_checked"), speakersEnabled ? " checked" : "");
        html.replace(F("%S29_checked"), displayMode8Enabled ? " checked" : "");
        html.replace(F("%S28_checked"), displayMode10Enabled ? " checked" : "");
        html.replace(F("%S31_checked"), sdPlayerStyle1Enabled ? " checked" : "");
        html.replace(F("%S32_checked"), sdPlayerStyle2Enabled ? " checked" : "");
        html.replace(F("%S33_checked"), sdPlayerStyle3Enabled ? " checked" : "");
        html.replace(F("%S34_checked"), sdPlayerStyle4Enabled ? " checked" : "");
        html.replace(F("%S35_checked"), sdPlayerStyle5Enabled ? " checked" : "");
        html.replace(F("%S36_checked"), sdPlayerStyle6Enabled ? " checked" : "");
        html.replace(F("%S37_checked"), sdPlayerStyle7Enabled ? " checked" : "");
        html.replace(F("%S38_checked"), sdPlayerStyle9Enabled ? " checked" : "");
        html.replace(F("%S39_checked"), sdPlayerStyle10Enabled ? " checked" : "");
        html.replace(F("%S40_checked"), sdPlayerStyle11Enabled ? " checked" : "");
        html.replace(F("%S41_checked"), sdPlayerStyle12Enabled ? " checked" : "");
        html.replace(F("%S42_checked"), sdPlayerStyle13Enabled ? " checked" : "");
        html.replace(F("%S43_checked"), sdPlayerStyle14Enabled ? " checked" : "");

        html.replace(F("%S1_checked"), displayAutoDimmerOn ? " checked" : "");
        html.replace(F("%S3_checked"), timeVoiceInfoEveryHour ? " checked" : "");
        html.replace(F("%S5_checked"), vuMeterOn ? " checked" : "");
        html.replace(F("%S7_checked"), adcKeyboardEnabled ? " checked" : "");
        html.replace(F("%S9_checked"), displayPowerSaveEnabled ? " checked" : "");
        
        // DLNA Configuration
        #ifdef USE_DLNA
        html.replace(F("%DLNA_HOST%"), dlnaHost);
        html.replace(F("%DLNA_IDX%"), dlnaIDX);
        #else
        html.replace(F("%DLNA_HOST%"), "192.168.1.100");
        html.replace(F("%DLNA_IDX%"), "0");
        #endif

        request->send(200, "text/html", html);
      });

    server.on("/adc", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      String html = FPSTR(adc_html);
      html.replace("%D10", String(keyboardButtonThreshold_Shift).c_str());
      html.replace("%D11", String(keyboardButtonThreshold_Memory).c_str()); 
      html.replace("%D12", String(keyboardButtonThreshold_Band).c_str()); 
      html.replace("%D13", String(keyboardButtonThreshold_Auto).c_str()); 
      html.replace("%D14", String(keyboardButtonThreshold_Scan).c_str()); 
      html.replace("%D15", String(keyboardButtonThreshold_Mute).c_str()); 
      html.replace("%D16", String(keyboardButtonThresholdTolerance).c_str()); 
      html.replace("%D17", String(keyboardButtonNeutral).c_str());
      html.replace("%D18", String(keyboardSampleDelay).c_str()); 
      html.replace("%D0", String(keyboardButtonThreshold_0).c_str()); 
      html.replace("%D1", String(keyboardButtonThreshold_1).c_str()); 
      html.replace("%D2", String(keyboardButtonThreshold_2).c_str()); 
      html.replace("%D3", String(keyboardButtonThreshold_3).c_str()); 
      html.replace("%D4", String(keyboardButtonThreshold_4).c_str()); 
      html.replace("%D5", String(keyboardButtonThreshold_5).c_str()); 
      html.replace("%D6", String(keyboardButtonThreshold_6).c_str()); 
      html.replace("%D7", String(keyboardButtonThreshold_7).c_str()); 
      html.replace("%D8", String(keyboardButtonThreshold_8).c_str()); 
      html.replace("%D9", String(keyboardButtonThreshold_9).c_str()); 
      
      request->send(200, "text/html", html);
    });

    server.on("/configadc", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      if (request->hasParam("keyboardButtonThreshold_Shift", true)) {
        keyboardButtonThreshold_Shift = request->getParam("keyboardButtonThreshold_Shift", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonThreshold_Memory", true)) {
        keyboardButtonThreshold_Memory = request->getParam("keyboardButtonThreshold_Memory", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonThreshold_Band", true)) {
        keyboardButtonThreshold_Band = request->getParam("keyboardButtonThreshold_Band", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonThreshold_Auto", true)) {
        keyboardButtonThreshold_Auto = request->getParam("keyboardButtonThreshold_Auto", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonThreshold_Scan", true)) {
        keyboardButtonThreshold_Scan = request->getParam("keyboardButtonThreshold_Scan", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonThreshold_Mute", true)) {
        keyboardButtonThreshold_Mute = request->getParam("keyboardButtonThreshold_Mute", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonThresholdTolerance", true)) {
        keyboardButtonThresholdTolerance = request->getParam("keyboardButtonThresholdTolerance", true)->value().toInt();
      }
      if (request->hasParam("keyboardButtonNeutral", true)) {
        keyboardButtonNeutral = request->getParam("keyboardButtonNeutral", true)->value().toInt();
      }
      if (request->hasParam("keyboardSampleDelay", true)) {
        keyboardSampleDelay = request->getParam("keyboardSampleDelay", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_0", true)) {
        keyboardButtonThreshold_0 = request->getParam("keyboardButtonThreshold_0", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_1", true)) {
        keyboardButtonThreshold_1 = request->getParam("keyboardButtonThreshold_1", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_2", true)) {
        keyboardButtonThreshold_2 = request->getParam("keyboardButtonThreshold_2", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_3", true)) {
        keyboardButtonThreshold_3 = request->getParam("keyboardButtonThreshold_3", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_4", true)) {
        keyboardButtonThreshold_4 = request->getParam("keyboardButtonThreshold_4", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_5", true)) {
        keyboardButtonThreshold_5 = request->getParam("keyboardButtonThreshold_5", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_6", true)) {
        keyboardButtonThreshold_6 = request->getParam("keyboardButtonThreshold_6", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_7", true)) {
        keyboardButtonThreshold_7 = request->getParam("keyboardButtonThreshold_7", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_8", true)) {
        keyboardButtonThreshold_8 = request->getParam("keyboardButtonThreshold_8", true)->value().toInt();
      } 
      if (request->hasParam("keyboardButtonThreshold_9", true)) {
        keyboardButtonThreshold_9 = request->getParam("keyboardButtonThreshold_9", true)->value().toInt();
      }

      request->send(200, "text/html", "<h1>ADC Keyboard Thresholds Updated!</h1><a href='/menu'>Go Back</a>");
      
      saveAdcConfig(); 
       
      //ODswiezenie ekranu OLED po zmianach konfiguracji
      ir_code = rcCmdBack; // Udajemy kod pilota Back
      bit_count = 32;
      calcNec();          // Przeliczamy kod pilota na pełny oryginalny kod NEC
    });

    server.on("/configupdate", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      if (request->hasParam("displayBrightness", true)) { displayBrightness = request->getParam("displayBrightness", true)->value().toInt(); }
      if (request->hasParam("dimmerDisplayBrightness", true)) {dimmerDisplayBrightness = request->getParam("dimmerDisplayBrightness", true)->value().toInt();}
      if (request->hasParam("dimmerSleepDisplayBrightness", true)) {dimmerSleepDisplayBrightness = request->getParam("dimmerSleepDisplayBrightness", true)->value().toInt();}
      if (request->hasParam("displayAutoDimmerTime", true)) {displayAutoDimmerTime = request->getParam("displayAutoDimmerTime", true)->value().toInt();}

      //  CHECKBOXY – ZAWSZE 0 lub 1
      //displayAutoDimmerOn = request->hasParam("displayAutoDimmerOn", true) ? true : false;
      displayAutoDimmerOn        = request->hasParam("displayAutoDimmerOn", true);
      timeVoiceInfoEveryHour     = request->hasParam("timeVoiceInfoEveryHour", true);
      vuMeterOn                  = request->hasParam("vuMeterOn", true);
      adcKeyboardEnabled         = request->hasParam("adcKeyboardEnabled", true);
      displayPowerSaveEnabled    = request->hasParam("displayPowerSaveEnabled", true);
      maxVolumeExt               = request->hasParam("maxVolumeExt", true);
      vuPeakHoldOn               = request->hasParam("vuPeakHoldOn", true);
      vuSmooth                   = request->hasParam("vuSmooth", true);
      f_displayPowerOffClock     = request->hasParam("f_displayPowerOffClock", true);
      f_sleepAfterPowerFail      = request->hasParam("f_sleepAfterPowerFail", true);
      f_volumeFadeOn             = request->hasParam("f_volumeFadeOn", true);
      f_saveVolumeStationAlways  = request->hasParam("f_saveVolumeStationAlways", true);
      f_powerOffAnimation        = request->hasParam("f_powerOffAnimation", true);
      analyzerEnabled            = request->hasParam("fftAnalyzerOn", true);
      btModuleEnabled            = request->hasParam("btModuleEnabled", true);
      displayMode6Enabled        = request->hasParam("displayMode6Enabled", true);
      displayMode8Enabled        = request->hasParam("displayMode8Enabled", true);
      displayMode10Enabled       = request->hasParam("displayMode10Enabled", true);
      presetsOn                  = request->hasParam("presetsOn", true);
      speakersEnabled            = request->hasParam("speakersEnabled", true);
      updateSpeakersPin(); // Aktualizuj pin głośników natychmiast po zmianie ustawienia
      sdPlayerStyle1Enabled      = request->hasParam("sdPlayerStyle1", true);
      sdPlayerStyle2Enabled      = request->hasParam("sdPlayerStyle2", true);
      sdPlayerStyle3Enabled      = request->hasParam("sdPlayerStyle3", true);
      sdPlayerStyle4Enabled      = request->hasParam("sdPlayerStyle4", true);
      sdPlayerStyle5Enabled      = request->hasParam("sdPlayerStyle5", true);
      sdPlayerStyle6Enabled      = request->hasParam("sdPlayerStyle6", true);
      sdPlayerStyle7Enabled      = request->hasParam("sdPlayerStyle7", true);
      sdPlayerStyle9Enabled      = request->hasParam("sdPlayerStyle9", true);
      sdPlayerStyle10Enabled     = request->hasParam("sdPlayerStyle10", true);
      sdPlayerStyle11Enabled     = request->hasParam("sdPlayerStyle11", true);
      sdPlayerStyle12Enabled     = request->hasParam("sdPlayerStyle12", true);
      sdPlayerStyle13Enabled     = request->hasParam("sdPlayerStyle13", true);
      sdPlayerStyle14Enabled     = request->hasParam("sdPlayerStyle14", true);

      // Jeśli parametr istnieje checkbox był zaznaczony to TRUE
      // Jeśli go nie ma checkbox nie był zaznaczony to FALSE

      if (request->hasParam("vuMeterMode", true)) {vuMeterMode = request->getParam("vuMeterMode", true)->value().toInt();}
      if (request->hasParam("encoderFunctionOrder", true)) {encoderFunctionOrder = request->getParam("encoderFunctionOrder", true)->value().toInt();}
      if (request->hasParam("displayMode", true)) {
        displayMode = request->getParam("displayMode", true)->value().toInt();
        // Pomijamy style 7-9 (brak implementacji)
        if (displayMode >= 7 && displayMode <= 9) {displayMode = 10;}
        // Pomijamy style >= 11 (brak implementacji)
        if (displayMode >= 11) {displayMode = 0;}
      }
      if (request->hasParam("vuMeterRefreshTime", true)) {vuMeterRefreshTime = request->getParam("vuMeterRefreshTime", true)->value().toInt();}
      if (request->hasParam("scrollingRefresh", true)) {scrollingRefresh = request->getParam("scrollingRefresh", true)->value().toInt();}
      if (request->hasParam("displayPowerSaveTime", true)) {displayPowerSaveTime = request->getParam("displayPowerSaveTime", true)->value().toInt();}
      if (request->hasParam("vuRiseSpeed", true)) {vuRiseSpeed = request->getParam("vuRiseSpeed", true)->value().toInt();}
      if (request->hasParam("vuFallSpeed", true)) {vuFallSpeed = request->getParam("vuFallSpeed", true)->value().toInt();}
      if (request->hasParam("analyzerStyles", true)) {
        analyzerStyles = request->getParam("analyzerStyles", true)->value().toInt();
        if (analyzerStyles < 0) analyzerStyles = 0;
        if (analyzerStyles > 2) analyzerStyles = 2;
      }
      
      // Obsługa zmiany presetu analizatora
      bool presetChanged = false;
      int oldPreset = analyzerPreset;
      if (request->hasParam("analyzerPreset", true)) {
        analyzerPreset = request->getParam("analyzerPreset", true)->value().toInt();
        if (analyzerPreset < 0) analyzerPreset = 0;
        if (analyzerPreset > 4) analyzerPreset = 4;
        if (analyzerPreset != oldPreset) {
          presetChanged = true;
        }
      }
      
      // Zastosuj ustawienia analizatora i equalizera 3-point
      eq_analyzer_set_enabled(analyzerEnabled);
      
      // Jeśli preset się zmienił, zastosuj go i zapisz
      if (presetChanged) {
        analyzerApplyPreset(analyzerPreset);
        analyzerStyleSave();
        Serial.println("Analyzer: Applied and saved preset " + String(analyzerPreset));
      }
      
      // DLNA Configuration
      #ifdef USE_DLNA
      if (request->hasParam("dlnaHost", true)) {
        dlnaHost = request->getParam("dlnaHost", true)->value();
        dlnaHost.trim();
        if (dlnaHost.length() == 0) dlnaHost = "192.168.1.100";
        Serial.println("[Config] DLNA Host updated: " + dlnaHost);
      }
      if (request->hasParam("dlnaIDX", true)) {
        dlnaIDX = request->getParam("dlnaIDX", true)->value();
        dlnaIDX.trim();
        if (dlnaIDX.length() == 0) dlnaIDX = "0";
        Serial.println("[Config] DLNA IDX updated: " + dlnaIDX);
      }
      #endif
      
      readEqualizerFromSD();
      audio.setTone(toneLowValue, toneMidValue, toneHiValue);

      saveConfig();
      readConfig();
      //clearFlags();
      
      //Refresh
      ir_code = rcCmdBack; // Udajemy kod pilota Back
      bit_count = 32;
      calcNec();          // Przeliczamy kod pilota na pełny oryginalny kod NEC

      //request->send(200, "text/html","<h1>Config Updated!</h1><a href='/menu'>Go Back</a>");
      request->send(200, "text/html","<!DOCTYPE html><html><head><meta http-equiv='refresh' content='2;url=/'></head><body><h1>Config Updated!</h1></body></html>");
    });
    
    // =====================================================================================
    // REMOTE CONTROL CONFIG ENDPOINTS
    // =====================================================================================
    server.on("/remoteconfig", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      String html = String(remoteconfig_html);
      // Replace placeholders with actual values from configRemoteArray
      for (int i = 0; i < 26; i++) {
        char placeholder[10];
        sprintf(placeholder, "%%D%d", i);
        char value[10];
        sprintf(value, "0x%04X", configRemoteArray[i]);
        html.replace(placeholder, value);
      }
      request->send(200, "text/html", html);
    });

    server.on("/remoteconfigupdate", HTTP_POST, [](AsyncWebServerRequest *request)
    {
      if (request->hasParam("cmdVolumeUp", true))   {configRemoteArray[0] = strtol(request->getParam("cmdVolumeUp", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdVolumeDown", true)) {configRemoteArray[1] = strtol(request->getParam("cmdVolumeDown", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdArrowRight", true)) {configRemoteArray[2] = strtol(request->getParam("cmdArrowRight", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdArrowLeft", true))  {configRemoteArray[3] = strtol(request->getParam("cmdArrowLeft", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdArrowUp", true))    {configRemoteArray[4] = strtol(request->getParam("cmdArrowUp", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdArrowDown", true))  {configRemoteArray[5] = strtol(request->getParam("cmdArrowDown", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdBack", true)) {configRemoteArray[6] = strtol(request->getParam("cmdBack", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdOk", true)) {configRemoteArray[7] = strtol(request->getParam("cmdOk", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdSrc", true)) {configRemoteArray[8] = strtol(request->getParam("cmdSrc", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdMute", true)) {configRemoteArray[9] = strtol(request->getParam("cmdMute", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdAud", true)) {configRemoteArray[10] = strtol(request->getParam("cmdAud", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdDirect", true)) {configRemoteArray[11] = strtol(request->getParam("cmdDirect", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdBankMinus", true)) {configRemoteArray[12] = strtol(request->getParam("cmdBankMinus", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdBankPlus", true)) {configRemoteArray[13] = strtol(request->getParam("cmdBankPlus", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdRedPower", true)) {configRemoteArray[14] = strtol(request->getParam("cmdRedPower", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdGreen", true)) {configRemoteArray[15] = strtol(request->getParam("cmdGreen", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey0", true)) {configRemoteArray[16] = strtol(request->getParam("cmdKey0", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey1", true)) {configRemoteArray[17] = strtol(request->getParam("cmdKey1", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey2", true)) {configRemoteArray[18] = strtol(request->getParam("cmdKey2", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey3", true)) {configRemoteArray[19] = strtol(request->getParam("cmdKey3", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey4", true)) {configRemoteArray[20] = strtol(request->getParam("cmdKey4", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey5", true)) {configRemoteArray[21] = strtol(request->getParam("cmdKey5", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey6", true)) {configRemoteArray[22] = strtol(request->getParam("cmdKey6", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey7", true)) {configRemoteArray[23] = strtol(request->getParam("cmdKey7", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey8", true)) {configRemoteArray[24] = strtol(request->getParam("cmdKey8", true)->value().c_str(), NULL, 16);}
      if (request->hasParam("cmdKey9", true)) {configRemoteArray[25] = strtol(request->getParam("cmdKey9", true)->value().c_str(), NULL, 16);}

      remoteConfig = false;
      saveRemoteControlConfig();
      readRemoteControlConfig();
      assignRemoteCodes();
      request->send(200, "text/html", "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='2;url=/'></head><body><h1>Remote Control Config Codes Updated!</h1></body></html>");
    });

    server.on("/toggleRemoteConfig", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      displayActive = true;
      displayStartTime = millis();
      remoteConfig = !remoteConfig;
      if (remoteConfig) {displayCenterBigText("RC Learning Mode ON",46);} else {displayCenterBigText("RC Learning Mode OFF",46);}
      wsIrCodeClear();
      request->send(200, "text/plain", remoteConfig ? "1" : "0");
    });
    
    // =====================================================================================
    // SDPLAYER WEB ENDPOINTS
    // =====================================================================================
    SDPlayerWebUI_registerHandlers(server);  // Rejestracja handlerów SDPlayer
    
    // =====================================================================================
    // BLUETOOTH WEBUI ENDPOINTS (MUST BE REGISTERED BEFORE OTHER /bt ROUTES)
    // =====================================================================================
    BTWebUI_registerHandlers(server);  // Rejestracja handlerów BTWebUI (zawsze dostępne)
    
    // =====================================================================================
    // DLNA WEBUI ENDPOINTS
    // =====================================================================================
    #ifdef USE_DLNA
    DLNAWebUI_init();                   // Inicjalizacja DLNA WebUI
    DLNAWebUI_registerHandlers(server); // Rejestracja handlerów DLNAWebUI
    #endif
    
    // Analyzer endpoints
    server.on("/analyzer", HTTP_GET, [](AsyncWebServerRequest *request) {
      String html = analyzerBuildHtmlPage();
      request->send(200, "text/html", html);
    });

    server.on("/analyzerApply", HTTP_POST, [](AsyncWebServerRequest *request) {
      // Parse all analyzer parameters from POST request
      int paramCount = request->params();
      String settings = "";
      for(int i = 0; i < paramCount; i++) {
        const AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()) {
          String name = p->name();
          String value = p->value();
          settings += name + "=" + value + "\n";
        }
      }
      
      // Apply configuration from parsed settings
      if(settings.length() > 0) {
        analyzerStyleLoadFromString(settings);
      }
      
      request->send(200, "text/plain", "Settings applied");
    });

    server.on("/analyzerSave", HTTP_POST, [](AsyncWebServerRequest *request) {
      // Same parsing as analyzerApply but also save to file
      int paramCount = request->params();
      String settings = "";
      for(int i = 0; i < paramCount; i++) {
        const AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()) {
          String name = p->name();
          String value = p->value();
          settings += name + "=" + value + "\n";
        }
      }
      
      // Apply and save configuration
      if(settings.length() > 0) {
        analyzerStyleLoadFromString(settings);
        analyzerStyleSave();
      }
      
      request->send(200, "text/plain", "Settings saved");
    });

    server.on("/analyzerPreset", HTTP_POST, [](AsyncWebServerRequest *request) {
      if(request->hasParam("preset", true)) {
        int presetId = request->getParam("preset", true)->value().toInt();
        if(presetId >= 0 && presetId <= 4) {
          analyzerApplyPreset(presetId);
          analyzerStyleSave();
          request->send(200, "text/plain", "Preset " + String(presetId) + " applied");
        } else {
          request->send(400, "text/plain", "Invalid preset ID");
        }
      } else {
        request->send(400, "text/plain", "Missing preset parameter");
      }
    });

    server.on("/analyzerCfg", HTTP_GET, [](AsyncWebServerRequest *request) {
      String json = analyzerStyleToJson();
      request->send(200, "application/json", json);
    });
    
    server.on("/toggleAdcDebug", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      // Przełączanie wartości
      u8g2.clearBuffer();
      displayActive = true;
      displayStartTime = millis();
      debugKeyboard = !debugKeyboard;
      //adcKeyboardEnabled = !adcKeyboardEnabled;
      Serial.print("Pomiar wartości ADC ON/OFF:");
      Serial.println(debugKeyboard);
      request->send(200, "text/plain", debugKeyboard ? "1" : "0"); // Wysyłanie aktualnej wartości
      //debugKeyboard = !debugKeyboard;
      
      if (debugKeyboard == 0)
      {
        ir_code = rcCmdBack; // Przypisujemy kod polecenia z pilota
        bit_count = 32; // ustawiamy informacje, ze mamy pelen kod NEC do analizy 
        calcNec();  // przeliczamy kod pilota na kod oryginalny pełen kod NEC  
      }
        
    });

    
    server.on("/mute", HTTP_POST, [](AsyncWebServerRequest *request) 
    {
      ir_code = rcCmdMute; // Przypisujemy kod polecenia z pilota
      bit_count = 32; // ustawiamy informacje, ze mamy pelen kod NEC do analizy 
      calcNec();  // przeliczamy kod pilota na kod oryginalny pełen kod NEC 
      request->send(204);       // brak odpowiedzi (204)    
    });
    

    server.on("/view", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      String filename = "/";
      if (request->hasParam("filename")) 
      {
        filename += request->getParam("filename")->value();
        //String fullPath = "/" + filename;

        File file = STORAGE.open(filename);
        if (file) 
        {
            String content = "<html><body><h1>File content: " + filename + "</h1><pre>";
            while (file.available()) {
                content += (char)file.read();
            }
            content += "</pre><a href=\"/list\">Back to list</a></body></html>";
            request->send(200, "text/html", content);
            file.close();
          } 
          else 
          {
            request->send(404, "text/plain", "File not found");
          }
        } 
        else 
        {
        request->send(400, "text/plain", "No file name");
      }
    });
    // Format viewweb ver2 na potrzeby zewnetrznego Edytora Bankow HTML
    server.on("/viewweb2", HTTP_GET, [](AsyncWebServerRequest *request)  
    {
      String filename = "/";
      if (request->hasParam("filename")) 
      {
        filename += request->getParam("filename")->value();
        //String fullPath = "/" + filename;

        File file = STORAGE.open(filename);
        if (file) 
        {
            String content; //= "<html><body>";
            while (file.available()) {
                //line = file.readStringUntil('\n');
                content += file.readStringUntil('\n') + String("\n");
            }
            //content +="</body></html>";
            request->send(200, "text/html", content);
            file.close();
          } 
          else 
          {
            request->send(404, "text/plain", "File not found");
          }
        } 
        else 
        {
        request->send(400, "text/plain", "No file name");
      }
    });
  
    /*
    server.on("/editordata", HTTP_GET, [](AsyncWebServerRequest *request) 
    {
      if (!request->hasParam("filename")) {
          request->send(400, "text/plain", "No file name");
          return;
      }

      String filename = request->getParam("filename")->value();
      File file = STORAGE.open(filename);
      if (!file) {
          request->send(404, "text/plain", "File not found");
          return;
      }

      // Tworzymy odpowiedź strumieniową tekstową
      AsyncWebServerResponse *response = request->beginResponseStream("text/plain");
      AsyncWebServerResponseStream *stream = static_cast<AsyncWebServerResponseStream*>(response);

      while (file.available()) {
          String line = file.readStringUntil('\n');
          stream->print(line + "\n");  // tu działa print
      }

      file.close();
      request->send(response);
    });
    */
    server.on("/editordata", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("filename")) {
        request->send(400, "text/plain", "No file name");
        return;
    }

    String filename = "/" + request->getParam("filename")->value();

    File file = STORAGE.open(filename);
    if (!file) {
        request->send(404, "text/plain", "File not found");
        return;
    }

    // Tworzymy strumień odpowiedzi typu text/plain
    AsyncResponseStream *response = request->beginResponseStream("text/plain");

    while (file.available()) {
        String line = file.readStringUntil('\n');
        response->print(line + "\n");  // działa tylko z AsyncResponseStream
    }

    file.close();
    request->send(response);
});



    server.on("/download", HTTP_ANY, [](AsyncWebServerRequest *request) {
      if (request->hasParam("filename")) {
        String filename = request->getParam("filename")->value();
        String fullPath = "/" + filename;
        //String fullPath = filename;

        Serial.println("Pobieranie pliku: " + fullPath);

        File file = STORAGE.open(fullPath);
        //Serial.println("SD: " + SD + "/" + filename);

        if (file) 
        {
            if (file.size() > 0) 
            {         
              request->send(STORAGE, fullPath, "application/octet-stream", true); //"application/octet-stream");"text/plain"
              file.close();
              Serial.println("Plik wysłany: " + filename);
            } 
            else 
            {
              request->send(404, "text/plain", "Plik jest pusty");
            }
        } 
        else 
        {
          Serial.println("Nie znaleziono pliku: " + fullPath);
          request->send(404, "text/plain", "Plik nie znaleziony");
        }
      } 
      else 
      {
        Serial.println("Brak parametru filename");
        request->send(400, "text/plain", "Brak nazwy pliku");
      }
    });
 
    server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "File Uploaded!");
      },
      //nullptr,
      [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (!filename.startsWith("/")) {
        filename = "/" + filename;
      }

      Serial.print("Upload File Name: ");
      Serial.println(filename);

      // Otwórz plik na karcie SD
      static File file; // Użyj statycznej zmiennej do otwierania pliku tylko raz
      if (index == 0) {
          file = STORAGE.open(filename, FILE_WRITE);
          if (!file) {
              Serial.println("Failed to open file for writing");
              request->send(500, "text/plain", "Failed to open file for writing");
              return;
          }
      }

      // Zapisz dane do pliku
      size_t written = file.write(data, len);
      if (written != len) {
          Serial.println("Error writing data to file");
          file.close();
          request->send(500, "text/plain", "Error writing data to file");
          return;
      }

      // Jeśli to ostatni fragment, zamknij plik i wyślij odpowiedź do klienta
      if (final) {
          file.close();
          Serial.println("File upload completed successfully.");
          request->send(200, "text/plain", "File upload successful");
      } else {
          Serial.println("Received chunk of size: " + String(len));
      }
    });

    server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
      if (request->hasParam("url")) 
      {
        String inputMessage = request->getParam("url")->value();
        url2play = inputMessage.c_str();
        urlToPlay = true;
        Serial.println("Odebrany URL: " + inputMessage);
        request->send(200, "text/plain", "URL received");
      } 
      else 
      {
        request->send(400, "text/plain", "Missing URL parameter");
      }  
    });

    server.on("/info", HTTP_GET, [](AsyncWebServerRequest *request) {
      String html = FPSTR(info_html);

      uint64_t chipid = ESP.getEfuseMac();
      char chipStr[20];
      sprintf(chipStr, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
      
      html.replace("%D1", String(softwareRev).c_str()); 
      html.replace("%D2", String(hostname).c_str()); 
      html.replace("%D3", String(WiFi.RSSI()).c_str()); 
      html.replace("%D4", String(wifiManager.getWiFiSSID()).c_str()); 
      html.replace("%D5", currentIP.c_str()); 
      html.replace("%D6", WiFi.macAddress().c_str()); 
      html.replace("%D7", String(storageTextName).c_str()); 
      //if (useSD) html.replace("%D7", String("SD").c_str()); 
      //else html.replace("%D7", String("SPIFFS").c_str()); 
      html.replace("%D0", chipStr); 
      f_callInfo = true;
      
      request->send(200, "text/html", html);

    });

    server.on("/timezone", HTTP_GET, [](AsyncWebServerRequest *request){
      //request->send(STORAGE, "/timezone.html", "text/html");
      request->send(200, "text/html", String(timezone_html));
    });

    server.on("/getTZ", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", timezone);
    });

    server.on("/setTZ", HTTP_GET, [](AsyncWebServerRequest *request){
      if (!request->hasParam("value")) 
      {
        request->send(400, "text/plain", "Missing timezone value");
        return;
      }

      String newTZ = request->getParam("value")->value();
      newTZ.trim();
      timezone = newTZ;

      saveTimezone(newTZ);

      //configTzTime(timezone.c_str(), "pool.ntp.org", "time.nist.gov");
      configTzTime(timezone.c_str(), ntpServer1, ntpServer2);
      
      request->send(200, "text/plain", "OK");
    });

    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    
    // =============== INICJALIZACJA MODUŁÓW - FULL INTEGRATION ===============
    Serial.println("DEBUG: Initializing integrated modules...");
    
    // 2. Analyzer - analizator spektrum FFT
    Serial.println("DEBUG: Initializing FFT Analyzer...");
    if (eq_analyzer_init()) {
        Serial.println("DEBUG: FFT Analyzer initialized successfully");
        eq_analyzer_set_enabled(analyzerEnabled);  // Domyślnie wyłączony
    } else {
        Serial.println("WARNING: FFT Analyzer initialization failed");
    }
    
    // 4. BTWebUI - moduł Bluetooth
    // Używa wrapper functions - registerHandlers wywołany już wcześniej w konfiguracji serwera
    Serial.println("DEBUG: Bluetooth module initialized via wrapper functions");
    
    Serial.println("DEBUG: All modules initialized");
    // =====================================================================
    
    server.begin();
    currentSelection = station_nr - 1; // ustawiamy stacje na liscie na obecnie odtwarzaczną przy starcie radia
    firstVisibleLine = currentSelection + 1; // pierwsza widoczna lina to grająca stacja przy starcie
    if (currentSelection + 1 >= stationsCount - 1) 
    {
      firstVisibleLine = currentSelection - 3;
    }  
    if (!sdPlayerOLEDActive) displayRadio(); // Blokuj gdy SDPlayer aktywny
    ActionNeedUpdateTime = true;
    
    //fadeInVolume
  } 
  else 
  {
    // PUNKT 2: POWER ON BEZ WiFi - radio może działać w trybie offline (tylko SD Player)
    Serial.println("[WiFi] ✗ Brak połączenia z siecią WiFi - OFFLINE MODE");
    Serial.println("[WiFi] Możliwe przyczyny:");
    Serial.println("  - Router WiFi wyłączony lub poza zasięgiem");
    Serial.println("  - Zmieniono nazwę sieci (SSID) lub hasło");
    Serial.println("  - Zbyt słaby sygnał WiFi");
    Serial.println("[WiFi] ℹ Radio działa w trybie OFFLINE - SD Player dostępny");
    Serial.println("[WiFi] ℹ Portal konfiguracyjny: SSID=EVO-Radio, IP=192.168.4.1");

    // -------- LED STANDBY: szybkie miganie = brak WiFi --------
    ledBlinker.detach();
    ledBlinker.attach_ms(150, []() {
      static bool s = false;
      s = !s;
      digitalWrite(STANDBY_LED, s ? HIGH : LOW);
    });
    Serial.println("[LED] Standby LED: szybkie miganie - brak WiFi");
    
    // Wyświetl komunikat na OLED
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_fub14_tf);
    u8g2.drawStr(10, 20, "OFFLINE MODE");
    u8g2.setFont(spleen6x12PL);
    u8g2.drawStr(5, 35, "No WiFi connection");
    u8g2.drawStr(5, 48, "SD Player available");
    u8g2.drawStr(5, 61, "Use encoder to activate");
    u8g2.sendBuffer();
    
    // Inicjalizacja timerów (potrzebne dla loop())
    timer1.attach(0.5, updateTimerFlag);
    timer2.attach(1, displayDimmerTimer);
    
    // Auto-start SD Player jeśli był aktywny przed wyłączeniem
    if (shouldAutoStartSDPlayer && g_sdPlayerOLED) {
      delay(2000); // Pokaż komunikat przez 2 sekundy
      Serial.println("[OFFLINE] Auto-starting SD Player...");
      g_sdPlayerOLED->activate();
      sdPlayerOLEDActive = true;
    }
    
    Serial.println("[OFFLINE] Setup completed - entering main loop");
    // NIE UŻYWAMY while(true) - pozwalamy kodowi przejść do loop() gdzie SD Player działa
  }
}

// #######################################################################################  LOOP  ####################################################################################### //
// ======================= AUDIO PROCESSING CALLBACK =======================
// Funkcja przekazująca próbki audio do analizatora FFT (wywoływana z Audio.cpp)
// UWAGA: Wersja 3.4.4 używa int16_t (kompatybilność backward)
void audio_process_i2s(int16_t* outBuff, int32_t validSamples, bool* continueI2S)
{
  // Push audio samples to SDRecorder (jeśli nagrywa)
  if (g_sdRecorder && g_sdRecorder->isRecording()) {
    g_sdRecorder->pushAudioData(outBuff, validSamples);
  }
  
  // Push audio samples to EQ analyzer (validSamples is number of stereo frames)
  eq_analyzer_push_samples_i16((const int16_t*)outBuff, validSamples);
  
  // Używamy 3-punktowego equalizera z audio.setTone()
  // Continue normal audio processing
  *continueI2S = true;
}

// #######################################################################################  LOOP  ####################################################################################### //
void loop() 
{
  // MONITORING PAMIĘCI: Sprawdzenie czy może być problem z odrestartowywaniem
  static unsigned long memCheckTime = 0;
  static int loopCounter = 0;
  
  if (millis() - memCheckTime > 60000) { // Co 60 sekund (zoptymalizowane)
    size_t freeHeap = ESP.getFreeHeap();
    size_t minFreeHeap = ESP.getMinFreeHeap();
    
    // Loguj tylko przy niskiej pamięci
    if (freeHeap < 20000 || minFreeHeap < 15000) {
      Serial.printf("[WARNING] Low memory! Free: %u bytes, Min: %u bytes, Loop: %d\n", 
                    freeHeap, minFreeHeap, loopCounter);
    }
    
    memCheckTime = millis();
    loopCounter = 0;
  }
  loopCounter++;
  
  runTime1 = esp_timer_get_time();
  audio.loop();           // Wykonuje główną pętlę dla obiektu audio
  
  // =============== OBSŁUGA ZINTEGROWANYCH MODUŁÓW ===============
  // SDPlayer - obsługa odtwarzania plików SD
  SDPlayerOLED_loop();   // Aktualizacja OLED dla SDPlayer (wywołuje loop() tylko gdy aktywny)
  // Gdy SDPlayer jest aktywny - NIE wywoływuj displayRadio() w dalszej części
  
  // SDRecorder - obsługa nagrywania strumienia
  SDRecorder_loop();     // Aktualizacja SDRecorder (flush bufora, sprawdzanie limitów)
  
  // BTWebUI - polling UART dla modułu Bluetooth
  if (btModuleEnabled) {
    BTWebUI_loop();        // Odczyt danych UART z modułu BT (wrapper function)
  }
  
  // Obsługa auto-play SDPlayera (w głównej pętli, nie w callback Audio)
  // KRYTYCZNA ZMIANA: Sprawdzaj TYLKO sdPlayerAutoPlayNext (bez sdPlayerPlayingMusic)
  // bo gdy utwór się kończy, sdPlayerPlayingMusic może być false ale chcemy auto-play!
  if (sdPlayerAutoPlayNext) {
    sdPlayerAutoPlayNext = false; // Zresetuj flagę
    
    // TRYB 1: Odtwarzanie przez PILOTA (lista utworów w pamięci tracksCount > 0)
    if (tracksCount > 0 && lastPlayedTrackIndex >= 0 && sdPlayerOLEDActive) {
      Serial.println("[SDPlayer OLED] Auto-play następnego utworu (pilot)");
      
      // Przejdź do następnego utworu (z zapętleniem)
      lastPlayedTrackIndex++;
      if (lastPlayedTrackIndex >= tracksCount) {
        lastPlayedTrackIndex = 0; // Wróć do początku listy
      }

      if (sdPlayerSessionStartIndex < 0) {
        sdPlayerSessionStartIndex = lastPlayedTrackIndex;
      }
      sdPlayerSessionEndIndex = lastPlayedTrackIndex;
      sdPlayerSessionCompleted = false;
      saveSDPlayerSessionState();
      
      currentTrackSelection = lastPlayedTrackIndex; // Synchronizuj kursor
      
      // KRYTYCZNE: Wyczyść stare metadane ID3 przed załadowaniem nowego utworu
      currentMP3Artist = "";
      currentMP3Title = "";
      currentMP3Album = "";
      
      // Odtwórz następny plik - trackFiles[] już zawiera pełną ścieżkę względną
      String filePath = "/" + trackFiles[lastPlayedTrackIndex];
      Serial.printf("[SDPlayer OLED] Odtwarzanie: %s (indeks %d)\n", 
                    trackFiles[lastPlayedTrackIndex].c_str(), lastPlayedTrackIndex);
      
      audio.connecttoFS(STORAGE, filePath.c_str());
      
      // WALIDACJA: Sprawdź Duration (ochrona przed IntegerDivideByZero)
      // Czekaj aż Duration > 0 lub timeout 2.5s (smart loop)
      uint32_t duration = 0;
      for(int i = 0; i < 50 && duration == 0; i++) {
        audio.loop();
        delay(50);
        duration = audio.getAudioFileDuration();
      }
      if (duration == 0) {
        Serial.println("[SDPlayer OLED] ERROR - Corrupted file (Duration=0)");
        audio.stopSong();
        sdPlayerPlayingMusic = false;
        if (g_sdPlayerWeb) {
          g_sdPlayerWeb->setCurrentFile("");
          g_sdPlayerWeb->setIsPlaying(false);
        }
      } else {
        // KRYTYCZNE: Aktualizuj nazwę pliku i stan odtwarzania SDPlayer
        sdPlayerPlayingMusic = true; // Ustaw flagę odtwarzania SDPlayera
        if (g_sdPlayerWeb) {
          g_sdPlayerWeb->setCurrentFile(trackFiles[lastPlayedTrackIndex]);
          g_sdPlayerWeb->setIsPlaying(true);  // Ustaw flagę odtwarzania
        }
      }
    }
    // TRYB 2: Odtwarzanie przez WebUI (lazy loading, _fileList > 0)
    else if (g_sdPlayerWeb) {
      Serial.println("[SDPlayer WebUI] Auto-play następnego utworu (WebUI)");
      g_sdPlayerWeb->playNextAuto();
      
      // KRYTYCZNE: Synchronizuj indeks między WebUI a OLED
      if (g_sdPlayerOLED && sdPlayerOLEDActive) {
        int webIndex = g_sdPlayerWeb->getSelectedIndex();
        g_sdPlayerOLED->setSelectedIndex(webIndex);
        if (webIndex >= 0) {
          if (sdPlayerSessionStartIndex < 0) {
            sdPlayerSessionStartIndex = webIndex;
          }
          sdPlayerSessionEndIndex = webIndex;
          sdPlayerSessionCompleted = false;
          saveSDPlayerSessionState();
        }
        Serial.printf("[SDPlayer] Synchronized OLED index to %d\n", webIndex);
      }
    }
    // TRYB 3: Brak listy - zatrzymaj odtwarzanie
    else {
      Serial.println("[SDPlayer] Auto-play BŁĄD: Brak listy utworów - zatrzymuję odtwarzanie");
      sdPlayerPlayingMusic = false;
    }
  }
  
  // Obsługa skanowania katalogu SDPlayera (w głównej pętli aby uniknąć watchdog timeout)
  if (sdPlayerScanRequested && g_sdPlayerWeb) {
    sdPlayerScanRequested = false; // Zresetuj flagę
    Serial.println("[SDPlayer] Wykonywanie skanowania katalogu w loop()...");
    g_sdPlayerWeb->scanDirectory();  // Wywołaj publiczną metodę skanowania
    Serial.println("[SDPlayer] Skanowanie zakończone");
  }
  
  // Analyzer - analiza spektrum (działa na osobnym rdzeniu)
  // eq_analyzer_loop() wywoływane jest automatycznie w osobnym wątku
  // Tutaj tylko sprawdzamy czy analyzer jest aktywny
  // ==============================================================
  
  button2.loop();         // Wykonuje pętlę dla obiektu button2 (sprawdza stan przycisku z enkodera 2)
  handleButtons();        // Wywołuje dodatkowe funkcję obsługującą przyciski enkoderow
  handleFadeIn();         // Obsługa sciszania stacji przy przełaczaniu
  
  #ifdef SERIALCOM
    handleSerialRX();
  #endif
  vTaskDelay(1);          // Krótkie opóźnienie, oddaje czas procesora innym zadaniom
  
  /*---------------------  FUNKCJA PILOT IR  / Obsluga pilota IR w kodzie NEC ---------------------*/ 
  handleRemote();         

  /*-- OBSŁUGA IR TIMEOUT dla wielocyfrowego wprowadzania ---------------------*/
  if (irInputActive && millis() > irInputTimeout) {
    // Timeout - potwierdź wprowadzoną liczbę automatycznie
    if (irInputBuffer > 0 && irInputBuffer <= 100) {
      Serial.printf("[IR SDPlayer] Timeout - auto-confirming track: %d\n", irInputBuffer);
      irRequestedTrackNumber = irInputBuffer;
      irTrackSelectionRequest = true;
    } else {
      Serial.printf("[IR SDPlayer] Timeout - invalid buffer: %d, clearing\n", irInputBuffer);
    }
    
    // Resetuj system wprowadzania
    irInputBuffer = 0;
    irInputActive = false;
    irInputTimeout = 0;
  }
  
  /*-- OBSŁUGA IR TRACK SELECTION dla SDPlayera ---------------------*/
  if (irTrackSelectionRequest && sdPlayerOLEDActive) {
    irTrackSelectionRequest = false; // Zresetuj flagę
    
    Serial.printf("[IR SDPlayer] Processing track selection request %d (current tracksCount=%d)\n", 
                  irRequestedTrackNumber, tracksCount);
    
    // Załaduj listę utworów jeśli nie została jeszcze załadowana
    if (tracksCount == 0) {
      Serial.println("[IR SDPlayer] Loading tracks from SD card...");
      loadTracksFromSD();
      Serial.printf("[IR SDPlayer] Loaded %d tracks\n", tracksCount);
    }
    
    if (irRequestedTrackNumber >= 1 && irRequestedTrackNumber <= tracksCount) {
      int trackIndex = irRequestedTrackNumber - 1; // Konwersja na indeks (0-based)
      
      Serial.printf("[IR SDPlayer] Wybieranie utworu %d (indeks %d): %s\n", 
                    irRequestedTrackNumber, trackIndex, trackFiles[trackIndex].c_str());
      
      // Ustaw aktualny wybór i odtwórz utwór
      currentTrackSelection = trackIndex;
      lastPlayedTrackIndex = trackIndex;
      
      // Zapisz stan sesji
      if (sdPlayerSessionStartIndex < 0) {
        sdPlayerSessionStartIndex = trackIndex;
      }
      sdPlayerSessionEndIndex = trackIndex;
      sdPlayerSessionCompleted = false;
      saveSDPlayerSessionState();
      
      // KRYTYCZNE: Wyczyść stare metadane ID3 przed załadowaniem nowego utworu
      currentMP3Artist = "";
      currentMP3Title = "";
      currentMP3Album = "";
      
      // Odtwórz wybrany plik - trackFiles[] już zawiera pełną ścieżkę względną
      String filePath = "/" + trackFiles[trackIndex];
      audio.connecttoFS(STORAGE, filePath.c_str());
      
      // WALIDACJA: Sprawdź Duration (ochrona przed IntegerDivideByZero)
      // Czekaj aż Duration > 0 lub timeout 2.5s
      uint32_t duration = 0;
      for(int i = 0; i < 50 && duration == 0; i++) {
        audio.loop();
        delay(50);
        duration = audio.getAudioFileDuration();
      }
      if (duration == 0) {
        Serial.println("[SDPlayer] ERROR - Corrupted file (Duration=0)");
        audio.stopSong();
        sdPlayerPlayingMusic = false;
        if (g_sdPlayerWeb) {
          g_sdPlayerWeb->setCurrentFile("");
          g_sdPlayerWeb->setIsPlaying(false);
        }
      } else {
        sdPlayerPlayingMusic = true;
        
        // Aktualizuj SDPlayerWebUI
        if (g_sdPlayerWeb) {
          g_sdPlayerWeb->setCurrentFile(trackFiles[trackIndex]);
          g_sdPlayerWeb->setIsPlaying(true);
        }
      }
      
      // Pokaż komunikat na OLED
      displayActive = true;
      displayStartTime = millis();
      timeDisplay = false;
      
      if (g_sdPlayerOLED) {
        String message = "TRACK " + String(irRequestedTrackNumber);
        g_sdPlayerOLED->showActionMessage(message);
      }
      
      Serial.printf("[IR SDPlayer] Utwór %d uruchomiony pomyślnie\n", irRequestedTrackNumber);
    }
    else if (irRequestedTrackNumber == 0) {
      // Klawisz 0 = utwór 10 (jeśli istnieje)
      if (tracksCount >= 10) {
        int trackIndex = 9; // Indeks dla utworu 10
        
        Serial.printf("[IR SDPlayer] Wybieranie utworu 10 (klawisz 0, indeks %d): %s\n", 
                      trackIndex, trackFiles[trackIndex].c_str());
        
        currentTrackSelection = trackIndex;
        lastPlayedTrackIndex = trackIndex;
        
        if (sdPlayerSessionStartIndex < 0) {
          sdPlayerSessionStartIndex = trackIndex;
        }
        sdPlayerSessionEndIndex = trackIndex;
        sdPlayerSessionCompleted = false;
        saveSDPlayerSessionState();
        
        // KRYTYCZNE: Wyczyść stare metadane ID3 przed załadowaniem nowego utworu
        currentMP3Artist = "";
        currentMP3Title = "";
        currentMP3Album = "";
        
        // trackFiles[] już zawiera pełną ścieżkę względną
        String filePath = "/" + trackFiles[trackIndex];
        audio.connecttoFS(STORAGE, filePath.c_str());
        
        // WALIDACJA: Sprawdź Duration (ochrona przed IntegerDivideByZero)
        // Czekaj aż Duration > 0 lub timeout 2.5s
        uint32_t duration = 0;
        for(int i = 0; i < 50 && duration == 0; i++) {
          audio.loop();
          delay(50);
          duration = audio.getAudioFileDuration();
        }
        if (duration == 0) {
          Serial.println("[SDPlayer] ERROR - Corrupted file (Duration=0)");
          audio.stopSong();
          sdPlayerPlayingMusic = false;
          if (g_sdPlayerWeb) {
            g_sdPlayerWeb->setCurrentFile("");
            g_sdPlayerWeb->setIsPlaying(false);
          }
        } else {
          sdPlayerPlayingMusic = true;
          
          if (g_sdPlayerWeb) {
            g_sdPlayerWeb->setCurrentFile(trackFiles[trackIndex]);
            g_sdPlayerWeb->setIsPlaying(true);
          }
        }
        
        displayActive = true;
        displayStartTime = millis();
        timeDisplay = false;
        
        if (g_sdPlayerOLED) {
          g_sdPlayerOLED->showActionMessage("TRACK 10");
        }
        
        Serial.println("[IR SDPlayer] Utwór 10 (klawisz 0) uruchomiony pomyślnie");
      } else {
        Serial.printf("[IR SDPlayer] BŁĄD: Brak utworu 10 (dostępne tylko %d utworów)\n", tracksCount);
      }
    }
    else {
      Serial.printf("[IR SDPlayer] BŁĄD: Nieprawidłowy numer utworu %d (dostępne: 1-%d)\n", 
                    irRequestedTrackNumber, tracksCount);
    }
    
    irRequestedTrackNumber = 0; // Zresetuj numer utworu
  }

  /*-- FUNKCJA KLAWIATURA / Odczyt stanu klawiatura ADC pod GPIO 9 ---------------------*/
  if ((millis() - keyboardLastSampleTime >= keyboardSampleDelay) && (adcKeyboardEnabled)) // Sprawdzenie ADC - klawiatury 
  {
    keyboardLastSampleTime = millis();
    handleKeyboard();
  }
    
  /*-- ENKODER 1 - obsluga opcjonlanego enkodera 1 ---------------------*/
  #ifdef twoEncoders
    button1.loop();
    handleEncoder1();
  #endif

  /*-- ENKODER 2 - Podstaowy, obsluga enkodera 2 ---------------------*/
  if (encoderFunctionOrder) { handleEncoder2StationsVolumeClick();} else {handleEncoder2VolumeStationsClick();}


  //if (encoderFunctionOrder == 0) { handleEncoder2VolumeStationsClick(); } 
  //else if (encoderFunctionOrder == 1) { handleEncoder2StationsVolumeClick(); }
  
  // Obsługa przycisku Power ON/OFF
  #ifdef SW_POWER
    if (digitalRead(SW_POWER) == LOW) {powerOff();}
  #endif

  /*---------------------  FUNKCJA DIMMER ---------------------*/
  if ((displayActive == true) && (displayDimmerActive == true) && (fwupd == false)) {displayDimmer(0);}  

  /*---------------------  FUNKCJA BACK / POWROTU ze wszystkich opcji Menu, Ustawien, itd ---------------------*/
  if ((fwupd == false) && (displayActive) && (millis() - displayStartTime >= displayTimeout))  // Przywracanie poprzedniej zawartości ekranu po 8 sekundach
  {
    if (volumeBufferValue != volumeValue && f_saveVolumeStationAlways) { saveVolumeOnSD(); }    
    if ((rcInputDigitsMenuEnable == true) && (station_nr != stationFromBuffer)) { changeStation(); }  // Jezeli nastapiła zmiana numeru stacji to wczytujemy nową stacje
    
    // AUTO-PLAY dla listy utworów SDPlayer po 8 sekundach
    if (listedTracks && tracksCount > 0 && sdPlayerOLEDActive) {
      Serial.print("[SDPlayer] Auto-play po 8 sekundach: ");
      Serial.println(trackFiles[currentTrackSelection]);
      
      // Zapamiętaj wybrany utwór dla auto-play
      lastPlayedTrackIndex = currentTrackSelection;
      
      // KRYTYCZNE: Wyczyść stare metadane ID3 przed załadowaniem nowego utworu
      currentMP3Artist = "";
      currentMP3Title = "";
      currentMP3Album = "";
      
      // Odtwórz wybrany plik - trackFiles[] już zawiera pełną ścieżkę względną
      String filePath = "/" + trackFiles[currentTrackSelection];
      audio.connecttoFS(STORAGE, filePath.c_str());
      
      // WALIDACJA: Sprawdź Duration przed kontynuacją (ochrona przed IntegerDivideByZero)
      // Czekaj aż Duration > 0 lub timeout 2.5s
      uint32_t duration = 0;
      for(int i = 0; i < 50 && duration == 0; i++) {
        audio.loop();
        delay(50);
        duration = audio.getAudioFileDuration();
      }
      if (duration == 0) {
        Serial.println("[SDPlayer] ERROR - Auto-play: Corrupted file detected (Duration=0)");
        Serial.println("[SDPlayer] Stopping playback to prevent crash");
        audio.stopSong();
        sdPlayerPlayingMusic = false;
        
        // Wyczyść stan SDPlayerWebUI        
        if (g_sdPlayerWeb) {
          g_sdPlayerWeb->setCurrentFile("");
          g_sdPlayerWeb->setIsPlaying(false);
        }
      } else {
        // Plik jest poprawny - kontynuuj normalnie
        sdPlayerPlayingMusic = true;
        
        // KRYTYCZNE: Aktualizuj nazwę pliku i stan odtwarzania w SDPlayerWebUI dla wyświetlacza
        if (g_sdPlayerWeb) {
          g_sdPlayerWeb->setCurrentFile(trackFiles[currentTrackSelection]);
          g_sdPlayerWeb->setIsPlaying(true);  // Ustaw flagę odtwarzania
        }
      }
      
      listedTracks = false;  // Wyłącz flagę listy
      displayActive = false;
      timeDisplay = false;
      u8g2.clearBuffer();
      delay(50);  // Krótka pauza dla stabilności
      Serial.printf("[SDPlayer] Auto-play uruchomiony (indeks %d) - ekran zostanie odświeżony\n", lastPlayedTrackIndex);
    }
    
    displayDimmer(0); 
    clearFlags();
    
    // KRYTYCZNE: Nie nadpisuj ekranu gdy SDPlayer OLED aktywny
    if (!sdPlayerOLEDActive) {
      displayRadio();
      u8g2.sendBuffer();
    }
    
    // Przywracamy zaznaczenie obecnie grajacej stacji
    currentSelection = station_nr - 1; 
    //if (maxSelection() - currentSelection < maxVisibleLines) {firstVisibleLine = maxSelection() - 3;} else {firstVisibleLine = currentSelection;}
  }


  /*---------------------  FUNKCJA PETLI MILLIS SCROLLER / Odswiezanie VU Meter, Time, Scroller, OLED, WiFi ver. 1 ---------------------*/ 
  if ((millis() - scrollingStationStringTime > scrollingRefresh) && (displayActive == false) && !sdPlayerActive && !sdPlayerOLEDActive) // KRYTYCZNE: Dodano !sdPlayerOLEDActive
  {
    scrollingStationStringTime = millis();
    
    if (ActionNeedUpdateTime == true) // Aktualizacja zegara, zegar głosowy, debug Audio, sygnał wifi 
    {
      ActionNeedUpdateTime = false;
      updateTime();
  
      if (f_requestVoiceTimePlay == true) // Zegar głosowy, sprawdzamy czy została ustawiona flaga zegara przy pełnej godzinie
      {
        f_requestVoiceTimePlay = false;
        voiceTime();
      }

      if (debugAudioBuffor == true) {bufforAudioInfo();}
      
      if ((displayMode == 0) || (displayMode == 2))  {drawSignalPower(210,63,0,1);}
      if ((displayMode == 1) && (volumeMute == false)) {drawSignalPower(244,47,0,1);}
      if (displayMode == 3) {drawSignalPower(209,10,0,1);}
      if (wifiSignalLevel != wifiSignalLevelLast) {wifiSignalLevelLast = wifiSignalLevel; wsSignalLevel();}

      if ((f_audioInfoRefreshStationString == true) && (displayActive == false)) // Zmiana streamtitle - wymaga odswiezenia na wyswietlaczu
      { 
        f_audioInfoRefreshStationString = false; //NewAudioInfoSSF
        stationStringFormatting(); // Formatujemy StationString do wyswietlenia przez Scroller
      } 

      if (f_audioInfoRefreshDisplayRadio == true && displayActive == false && !sdPlayerOLEDActive) // Blokuj gdy SD Player OLED aktywny
      { 
        f_audioInfoRefreshDisplayRadio = false;
        ActionNeedUpdateTime = true;
        displayRadio();
      } 
      
      if (wsAudioRefresh == true) // Web Socket StremInfo wymaga odswiezenia
      {
        wsAudioRefresh = false;
        wsStreamInfoRefresh();
      }
      
    }

    if (volumeMute == false) 
    {
      if (vuMeterOn)
      { 
        if (displayMode == 0) {vuMeterMode0();}
        if (displayMode == 3) {vuMeterMode3();}
        if (displayMode == 4) 
        {
          vuMeterMode4(); 
          if (debugAudioBuffor)
          {
            for (int i = 0; i < 10; i++) 
            {
              int y = 1 + (9 - i) * 6; 
              if (audioBufferTime > i) { u8g2.drawBox(126, y, 8, 5);} else {u8g2.drawFrame(126, y, 8, 5);}
            }
          }
        }
        if (displayMode == 5) {vuMeterMode5();}
        if (displayMode == 7) {vuMeterMode7();}
        // STYLE 8 ZEWNĘTRZNY - Analizator 16-pasmowy z zegarem
        // if (displayMode == 8) {
        //   // Tryb 8: Analizator FFT 16-pasmowy
        //   if (analyzerEnabled) {
        //     eq_analyzer_display_mode5();  // Analizator 16-pasmowy
        //   } else {
        //     vuMeterMode8();  // Fallback do VU meter gdy analyzer wyłączony
        //   }
        // }
        else if (displayMode == 6) {
          // Tryb 6: Segmentowy analyzer
          vuMeterMode6();
        }
        else if (displayMode == 8) {
          // Tryb 8: VU Meter Mode 8
          vuMeterMode8();
        }
        else if (displayMode == 10) {
          // Tryb 10: Floating Peaks Analyzer
          vuMeterMode10();
        }
      }
      else if (displayMode == 0) {showIP(1,47);} //y = vuRy
    }
    else // Obsługa wyciszenia dzwięku, wprowadzamy napis MUTE na ekran
    {
      // Style 0-4: Pokazuj tekst "> MUTED <"
      if (displayMode == 0) {
        u8g2.setDrawColor(0);
        u8g2.drawStr(0,48, "> MUTED <");
        u8g2.setDrawColor(1);
      }
      else if (displayMode == 1) {
        u8g2.setDrawColor(0);
        u8g2.drawStr(200,47, "> MUTED <");
        u8g2.setDrawColor(1);
      }
      else if (displayMode == 2) {
        u8g2.setDrawColor(0);
        u8g2.drawStr(0,48, "> MUTED <");
        u8g2.setDrawColor(1);
      }
      else if (displayMode == 3) {
        u8g2.setDrawColor(0);
        u8g2.drawStr(101,63, "> MUTED <");
        u8g2.setDrawColor(1);
      }
      else if (displayMode == 4) {
        u8g2.setDrawColor(1);
        vuMeterMode4();
        u8g2.setFont(spleen6x12PL);
        u8g2.setDrawColor(0);
        u8g2.drawStr(103,57, "> MUTED <");
        u8g2.setDrawColor(1);
      }
      else if (displayMode == 5) {
        u8g2.setDrawColor(1);
        vuMeterMode5();
        u8g2.setFont(spleen6x12PL);
        u8g2.setDrawColor(0);
        u8g2.drawStr(103,57, "> MUTED <");
        u8g2.setDrawColor(1);
      }
      else if (displayMode == 7) {
        u8g2.setDrawColor(1);
        vuMeterMode7();
        u8g2.setFont(spleen6x12PL);
        u8g2.setDrawColor(0);
        u8g2.drawStr(103,40, "> MUTED <");
        u8g2.setDrawColor(1);
      }
      // Style 8, 6, 10: Wywołaj VU Meter Mode z obsługą przekreślonego głośnika
      else if (displayMode == 8) {
        vuMeterMode8();
      }
      else if (displayMode == 6) {
        vuMeterMode6();
      }
      else if (displayMode == 10) {
        vuMeterMode10();
      }
    }  

       
    if (urlToPlay == true) // Jesli web serwer ustawił flagę "odtwarzaj URL" to uruchamiamy funkcje odtwarzania z adresu URL wysłanego przez strone WWW
    {
      urlToPlay = false;
      webUrlStationPlay();
      // KRYTYCZNE: Nie nadpisuj ekranu gdy SDPlayer OLED aktywny
      if (!sdPlayerOLEDActive) {
        displayRadio();
      }
    }
    
    // KRYTYCZNE: Nie rysuj radio scrollera gdy SDPlayer aktywny
    // OPTYMALIZACJA: Display 10 FPS (co 100ms) dla MAKSYMALNEJ przepustowości audio
    static unsigned long lastDisplayUpdate = 0;
    if (!sdPlayerOLEDActive && (millis() - lastDisplayUpdate >= 100)) {
      displayRadioScroller();  // wykonujemy przewijanie tekstu station stringi przygotowujemy bufor ekranu
      u8g2.sendBuffer();  // rysujemy całą zawartosc ekranu.
      lastDisplayUpdate = millis();
    }
    
    //if (f_callInfo) {f_callInfo = false; displayBasicInfo();}  
    
  }
  
  // OCHRONA PRZED WATCHDOG RESET: krótka pauza na końcu loop()
  yield(); // Pozwala systemowi na obsługę zadań w tle
  
  //runTime2 = esp_timer_get_time();
  //runTime = runTime2 - runTime1;
}
