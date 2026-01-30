# EVO-RADIO-MOD
Wersja oprogramwania opata o projkt EVO3 https://github.com/dzikakuna/ESP32_radio_evo3 po małych modyfikacjach .
Dodano dodatkowy opcje do EVO3 
Dodano obsługę BT nadajnik ze stony ip/bt obsługa softu drugiego SEP23Wrom programwanie poprzez arduino 
Dodano prosty pleyer plików audio z kartySD 

 menu głowne z nowymi funkcjami 
<img width="689" height="910" alt="image" src="https://github.com/user-attachments/assets/23efd4a8-2a67-4191-85f1-1051bbf8e0a8" />

Panel do obsługi sdplayera ze strony ip/sdplayer

<img width="1336" height="895" alt="image" src="https://github.com/user-attachments/assets/002a9a94-b4c3-4406-95de-f851f7783bbe" />
Stona Do obsługi BT Nadajnika Audio 
do pracy trzeba pozmieniać ustawienia w sofcie BT nadajnika ESP32Wrom teraz saustawione MODE TX  Volume na 100 BOoST na 400 po wyedytowniu mozna obsługiwać juz ze stony jeszczenie testowałem .
Obecnie soft BT ESP32 Wrom startuje odrazu po podłączeniu zasilania szuka odbiornika BT i automatycznie sie z nim łączy .
<img width="783" height="887" alt="image" src="https://github.com/user-attachments/assets/bceb99de-1ccc-4478-b774-1ee62399c4ee" />
W sofie od BT nadajnika ESP32Wrom takie sa ustawienia do automatycznej pracy 

enum Mode : uint8_t { MODE_OFF=0, MODE_TX=1, MODE_AUTO=2 };
static volatile Mode g_mode = MODE_TX;

static bool g_btReady = false;
static bool g_scanning = false;

static String g_connMac = "";
static String g_connName = "";

// VOL: 0..100
static int g_vol_ui = 100;
static uint8_t g_vol_127 = 127;

// BOOST: 100..400 (%)
static int g_boost_pct = 400; 

Tak trzeba to zmienić w sofcie BT ESP32WROM do obsługi strony BT

enum Mode : uint8_t { MODE_OFF=0, MODE_TX=1, MODE_AUTO=2 };
static volatile Mode g_mode = MODE_OFF;

static bool g_btReady = false;
static bool g_scanning = false;

static String g_connMac = "";
static String g_connName = "";

// VOL: 0..100
static int g_vol_ui = 50;
static uint8_t g_vol_127 = 64;

// BOOST: 100..400 (%)
static int g_boost_pct = 100;

Podłaczenie BT od ESp32S3 n16 R8 
1) I2S (podsłuch audio z ESP32-S3)

To jest najważniejsze dla dźwięku po BT.

Na ESP32-S3 (Twoje radio):

BCLK = GPIO12

WS/LRCLK = GPIO14

DATA (DOUT z S3 do DAC) = GPIO13

Podłącz to równolegle do WROOM:

ESP32-S3 GPIO12 (BCLK) → WROOM GPIO26 (PIN_I2S_BCLK)
ESP32-S3 GPIO14 (WS) → WROOM GPIO25 (PIN_I2S_WS)
ESP32-S3 GPIO13 (DATA) → WROOM GPIO22 (PIN_I2S_DIN)
GND S3 ↔ GND WROOM

To działa tak, że S3 dalej karmi PCM5102A, a WROOM tylko “podsłuchuje”.

2) UART sterowanie (S3 ↔ WROOM)

Do wysyłania komend SCAN/CONNECT itd.

ESP32-S3 TX → WROOM GPIO16 (PIN_UART_RX)
ESP32-S3 RX ← WROOM GPIO17 (PIN_UART_TX)
GND S3 ↔ GND WROOM

Baud: 115200

3) Zasilanie i masa

GND musi być wspólne (inaczej UART i I2S będą wariować).

WROOM zasilasz normalnie z płytki (USB) albo stabilne 3.3V (jeśli goły moduł, to już trzeba przetwornicę i kondensatory).

4) Mega ważne: poziomy napięć

UART i I2S muszą być 3.3V. Nie dawaj 5V na piny.

