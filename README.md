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
do pracy ttrzeba pozmieniać ustawienia w sofcie BT nadajnika ESP32Wrom teraz saustawione MODE TX  Volume na 100 BOoST na 40 po wyedytowniu mozna obsługiwać juz ze stony jeszczenie testowałem .Obecnie soft BT ESO32 Wrom startuje odrazu ppo podłączeniu zasilania szuka odbiornika BT i automatycznie sie z nim łączy .
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

