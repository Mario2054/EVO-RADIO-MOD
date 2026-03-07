Projekt oparty na orginalnym oprogramowaniu https://github.com/dzikakuna/ESP32_radio_evo3/tree/main .
Dodaje modyfikacje do projektu EVO Wersja EVO 3.20.04 MOD SDP_BT_EQ16_ANALIZATOR  SD PLAYER SD RECORDING DLNA 

nkcje Firmware EVO Internet Radio v3.20.04
1. Odtwarzanie radia internetowego (Internet Radio)
17 banków stacji, do 99 stacji na bank
Pobieranie list stacji z GitHub (bank01.txt – bank17.txt)
Wyświetlanie nazwy stacji, tytułu, formatu, bitrate, częstotliwości
Obsługiwane formaty: MP3, FLAC (24bit), AAC, Vorbis, Opus
Piny I2S (DAC PCM5102A): DOUT=GPIO13, BCLK=GPIO12, LRC=GPIO14
Piny I2S (DAC TAS5805, opcja): DOUT=GPIO16, BCLK=GPIO14, LRC=GPIO15
TAS5805 dodaje I2C: CLK=GPIO9, DATA=GPIO8
2. Wyświetlacz OLED 256×64 (SPI)
8 trybów wyświetlania (displayMode 0–8)
Auto-dimmer i tryb power save
Piny SPI OLED: MOSI=GPIO39, MISO=GPIO0, SCK=GPIO38, CS=GPIO42, DC=GPIO40, RESET=GPIO41
3. Enkoder obrotowy (główny – Enkoder 2)
Regulacja głośności, zmiana stacji, nawigacja banku, Power ON/OFF
Piny: CLK=GPIO10, DT=GPIO11, SW (przycisk)=GPIO1
4. Enkoder obrotowy (dodatkowy – Enkoder 1, opcjonalny)
Włączany przez #define twoEncoders
Gdy aktywny: obsługuje Volume/Mute/PowerOFF
Piny: CLK=GPIO6, DT=GPIO5, SW=GPIO4
5. Pilot IR (podczerwień, standard NEC)
Pełna obsługa: głośność, stacje, banki, sleep timer, equalizer, SDPlayer, BT, nagrywanie, dimmer OLED
Wielocyfrowe wprowadzanie numerów stacji (timeout 3s)
Pin: recv_pin=GPIO15
6. Karta SD (SPI)
Przechowywanie konfiguracji, banków stacji, nagrań, plików audio
Piny SPI SD: CS=GPIO47, SCLK=GPIO45, MISO=GPIO21, MOSI=GPIO48
7. SDPlayer – odtwarzacz plików z karty SD
Odtwarzanie muzyki MP3/FLAC z katalogu /music/
14 stylów wyświetlania na OLED (wybierane przez pilota/WebUI)
Przewijanie ±10s, poprzedni/następny utwór, pauza, stop
Powrót do radia z zachowaniem banku i stacji
Zarządzanie przez WebUI i pilota IR
Brak dedykowanych pinów – korzysta z SD i I2S jak wyżej
8. SDRecorder – nagrywanie strumienia radiowego
Nagrywanie audycji radiowej do pliku MP3 na SD /recordings/
Toggle START/STOP przez pilota IR lub WebUI
Brak dedykowanych pinów – korzysta z SD jak wyżej
9. Equalizer 3-pasmowy (EQ3)
Regulacja: basy, środek, wysokie tony + balans L/R
Zarządzany przez enkoder, pilota IR i WebUI
10. Equalizer 16-pasmowy (EQ16, opcjonalny)
Włączany przez ENABLE_EQ16=1 w platformio.ini
Przełączanie EQ3 ↔ EQ16 przez pilota lub WebUI
Brak dedykowanych pinów – softwarowy DSP
11. Analizator spektrum FFT
Wyświetlanie na OLED w różnych stylach (pola 0/4/5/6/10/11)
Toggle ON/OFF przez pilota IR (przycisk YELLOW)
Brak dedykowanych pinów – analizuje sygnał audio I2S
12. VU Meter (analogowy wskaźnik poziomu)
Peak hold, kanały L i R
Dostępny w wielu trybach OLED (style 11)
Brak dedykowanych pinów – softwarowy
13. WiFi + WebUI (serwer async HTTP)
Panel sterowania radiem przez przeglądarkę
OTA (aktualizacja firmware przez sieć)
mDNS – dostęp przez http://evoradio.local
WiFiManager – konfiguracja WiFi przy pierwszym uruchomieniu (AP mode)
14. DLNA (opcjonalny, #define USE_DLNA)
Przeglądanie i odtwarzanie mediów z serwerów DLNA w sieci
SSDP discovery, HTTP streaming
Brak dedykowanych pinów – wyłącznie sieciowy
15. Moduł Bluetooth UART (BTWebUI)
Sterowanie modułem BT przez UART
Zarządzanie przez WebUI (/bt) i pilota IR
Piny: korzysta z UART ESP32 (nie zdefiniowane osobno w config.h)
16. Power Management
Przycisk Power ON/OFF z animacją
Light sleep ESP32 (wyjście przez wakeup)
Sleep timer (0–90 min, krok 15 min)
Głosowe odtwarzanie czasu co godzinę
Piny: SW_POWER=GPIO8, STANDBY_LED=GPIO17, SPEAKERS_PIN=GPIO18 (enable wzmacniacza HIGH=ON)
17. Wyświetlanie czasu (NTP)
Zegar na OLED, głosowe odtwarzanie czasu
Tryb zegara podczas sleep (power save)
18. WiFi Animation
Animacja gwiazdek podczas łączenia z WiFi
Podsumowanie pinów
Funkcja	GPIO
I2S DOUT (PCM5102A)	13
I2S BCLK	12
I2S LRC	14
OLED MOSI	39
OLED SCK	38
OLED CS	42
OLED DC	40
OLED RESET	41
SD CS	47
SD SCLK	45
SD MISO	21
SD MOSI	48
Enkoder2 CLK	10
Enkoder2 DT	11
Enkoder2 SW	1
Enkoder1 CLK (opt.)	6
Enkoder1 DT (opt.)	5
Enkoder1 SW (opt.)	4
IR odbiornik	15
Przycisk POWER	8
LED Standby/IR	17
SPEAKERS enable	18

Do TESTÓW DODANO SD RECORDING TEST nagrywania wav działą lecz przycina sprawdzcie sami . obsługa z pilota DIRECT/OK REC na panelu radyjka nagrywanie DIRECT/OK KONIEC nagrywania

<img width="707" height="471" alt="image" src="https://github.com/user-attachments/assets/47c10274-736e-4965-b44b-cd4f00ef4ac4" />

<img width="404" height="882" alt="image" src="https://github.com/user-attachments/assets/ba32c81e-c158-44a5-8a67-63832afaf8b6" />
<img width="964" height="895" alt="image" src="https://github.com/user-attachments/assets/ec7b0bbd-67bd-4fc4-b4a2-3b69af4a2b00" />

<img width="641" height="917" alt="image" src="https://github.com/user-attachments/assets/66926200-e75e-4483-94e8-d0a83a2f8096" />
<img width="953" height="912" alt="image" src="https://github.com/user-attachments/assets/c345bdc0-6391-47ce-903f-0a00db808e79" />
<img width="932" height="929" alt="image" src="https://github.com/user-attachments/assets/d7890e92-357c-4563-a34c-45104ac45705" />
<img width="978" height="864" alt="image" src="https://github.com/user-attachments/assets/7092c7ec-bf16-4679-a03b-fab3cebe5171" />
<img width="710" height="921" alt="image" src="https://github.com/user-attachments/assets/fc1438bb-a9bf-4304-8ee0-6ef203ff936d" />







