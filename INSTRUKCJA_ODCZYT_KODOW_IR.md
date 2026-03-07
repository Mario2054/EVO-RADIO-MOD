# 📡 INSTRUKCJA ODCZYTU KODÓW IR Z UNIWERSALNEGO PILOTA

## 🎯 CEL
Poznać kody HEX przycisków uniwersalnego pilota IR aby wprowadzić je do konfiguracji radia EVO.

---

## 📋 WYMAGANIA

1. **Hardware:**
   - Radio EVO z odbiornikiem IR (pin 15)
   - Uniwersalny pilot IR (standard NEC)
   - Kabel USB do połączenia z komputerem

2. **Software:**
   - PlatformIO lub Arduino IDE
   - Terminal Serial Monitor (baudrate: 115200)

---

## 🔧 KROK 1: URUCHOM RADIO I SERIAL MONITOR

### PlatformIO:
```powershell
cd "c:\YO RADIO\EVO RADIO\src\ESP32_radio_evo3.19\Platformio"
pio device monitor
```

### Arduino IDE:
1. Otwórz **Tools → Serial Monitor**
2. Ustaw baudrate na **115200**
3. Ustaw **Both NL & CR**

---

## 📖 KROK 2: ODCZYT KODU Z PILOTA

### Sposób odczytu:
1. **Skieruj pilota na odbiornik IR** (pin 15 w radio)
2. **Naciśnij przycisk** na pilocie (np. VOL+)
3. **Sprawdź Serial Monitor** - zobaczysz:

```
debug IR -> Kod NEC OK:B914  MSB-LSB: 29B9  ADR:B9 CMD:29
debug IR -> puls 9ms:8967  4.5ms:4498  1690us:1686  560us:560
```

### Interpretacja wyniku:
- **Kod NEC OK:** `B914` ← odwrotna kolejność (LSB-MSB)
- **MSB-LSB:** `29B9` ← poprawna kolejność dla pilota
- **ADR:** `B9` ← adres pilota (Address)
- **CMD:** `29` ← komenda przycisku (Command)

**✅ TWÓJ KOD TO:** `0xB914` (z linii "Kod NEC OK")

---

## 📝 KROK 3: ZAPISZ WSZYSTKIE KODY

Stwórz tabelę i zapisuj kody dla wszystkich przycisków:

| Przycisk | Kod HEX | Funkcja w Radio |
|----------|---------|-----------------|
| VOL+ | 0xB914 | Głośność + |
| VOL- | 0xB915 | Głośność - |
| ▶ (Right) | 0xB90B | Następna stacja |
| ◀ (Left) | 0xB90A | Poprzednia stacja |
| ▲ (Up) | 0xB987 | Lista w górę |
| ▼ (Down) | 0xB986 | Lista w dół |
| OK | 0xB902 | Zatwierdź wybór |
| BACK | 0xB903 | Powrót |
| ... | ... | ... |

**💡 PRZYKŁAD NAPEŁNIONEJ TABELI:**
```
VOL+     = 0xB914
VOL-     = 0xB915
→ RIGHT  = 0xB90B
← LEFT   = 0xB90A
↑ UP     = 0xB987
↓ DOWN   = 0xB986
OK       = 0xB902
BACK     = 0xB903
MUTE     = 0xB90E
POWER    = 0xB90D
```

---

## 💾 KROK 4: WPROWADŹ KODY DO PLIKU KONFIGURACJI

### Metoda 1: Przez interfejs WebUI (ZALECANE)

1. Otwórz **http://[IP_RADIA]/config**
2. Przejdź do zakładki **"Pilot IR"**
3. Dla każdego przycisku:
   - Wpisz kod HEX (np. `0xB914`)
   - Lub naciśnij przycisk "Learn" i wciśnij przycisk na pilocie
4. Kliknij **"Zapisz konfigurację"**

### Metoda 2: Ręczna edycja pliku `/config_remote.txt` na karcie SD

Utwórz lub edytuj plik `/config_remote.txt`:

```
# Konfiguracja pilota IR - Format: rcCmd = 0xKOD_HEX
rcCmdVolumeUp=0xB914
rcCmdVolumeDown=0xB915
rcCmdArrowRight=0xB90B
rcCmdArrowLeft=0xB90A
rcCmdArrowUp=0xB987
rcCmdArrowDown=0xB986
rcCmdBack=0xB903
rcCmdOk=0xB902
rcCmdSrc=0xB90F
rcCmdMute=0xB90E
rcCmdAud=0xB904
rcCmdDirect=0xB905
rcCmdBankMinus=0xB90C
rcCmdBankPlus=0xB911
rcCmdRed=0xB90D
rcCmdGreen=0xB906
rcCmdKey0=0xB900
rcCmdKey1=0xB901
rcCmdKey2=0xB9C0
rcCmdKey3=0xB9C1
rcCmdKey4=0xB940
rcCmdKey5=0xB941
rcCmdKey6=0xB9E0
rcCmdKey7=0xB9E1
rcCmdKey8=0xB960
rcCmdKey9=0xB961
rcCmdPower=0xB90D
rcCmdYellow=0xB992
rcCmdBlue=0xB994
rcCmdBT=0xB995
rcCmdPLAY=0xB996
rcCmdSTOP=0xB997
rcCmdREV=0xB998
rcCmdRER=0xB999
rcCmdTV=0x0000
rcCmdRADIO=0xB99A
rcCmdFILES=0xB99B
rcCmdWWW=0x0000
rcCmdGAZE=0x0000
rcCmdEPG=0x0000
rcCmdMENU=0xB99C
rcCmdEXIT=0xB99D
rcCmdREC=0xB994
rcCmdKOL=0xB99E
rcCmdCHAT=0xB99F
rcCmdPAUSE=0xB9A0
rcCmdREDLEFT=0xB9A1
rcCmdGREENL=0xB9A2
rcCmdCHPlus=0xB9A3
rcCmdCHMinus=0xB9A4
rcCmdINFO=0xB9A5
rcCmdPOWROT=0xB9A6
rcCmdHELP=0xB9A7
rcCmdPIP=0xB9A8
rcCmdMINIMA=0xB9A9
rcCmdMAXIMA=0xB9AA
rcCmdWELEMENT=0xB9AB
rcCmdWINPOD=0xB9AC
rcCmdGLOS=0xB9AD
```

### Metoda 3: Bezpośrednio w kodzie źródłowym (main.cpp)

Znajdź funkcję `readConfigRemote()` i odszukaj linię ~8871 (domyślne wartości):

```cpp
// Jeśli plik nie istnieje, użyj wartości domyślnych
if (!myFile) {
    Serial.println("Brak pliku konfiguracji pilota - wczytywanie wartości domyślnych");
    configIrExist = false;
    
    // TUTAJ WPISZ SWOJE KODY:
    rcCmdVolumeUp = 0xB914;   // ← Zmień na swój kod
    rcCmdVolumeDown = 0xB915; // ← Zmień na swój kod
    // ... itd dla wszystkich przycisków
}
```

---

## 🔍 KROK 5: WERYFIKACJA KONFIGURACJI

1. **Restart radia** (wyłącz/włącz zasilanie)
2. **Sprawdź Serial Monitor** przy starcie:
   ```
   Wczytywanie konfiguracji pilota z karty SD...
   Wczytano: rcCmdVolumeUp = 0xB914
   Wczytano: rcCmdVolumeDown = 0xB915
   ...
   ```
3. **Testuj przyciski** - każde naciśnięcie wywołuje odpowiednią funkcję:
   ```
   debug IR -> Kod NEC OK:B914  MSB-LSB: 29B9  ADR:B9 CMD:29
   [IR] Volume UP
   ```

---

## ⚠️ ROZWIĄZYWANIE PROBLEMÓW

### Problem 1: "Błąd - kod pilota NEC jest niepoprawny!"
**Przyczyna:** Pilot nie używa standardu NEC  
**Rozwiązanie:** Sprawdź czy pilot obsługuje protokół NEC (większość uniwersalnych pilotów ma taką opcję)

### Problem 2: Odbiornik nie wykrywa kodu
**Przyczyna:** 
- Odbiornik IR źle podłączony
- Słabe baterie w pilocie
- Za duża odległość (max 5m)

**Rozwiązanie:**
1. Sprawdź połączenie odbiornika IR (pin 15, GND, 3.3V)
2. Wymień baterie w pilocie
3. Zbliż pilota do odbiornika (30-50cm)

### Problem 3: Kod się zmienia przy każdym naciśnięciu
**Przyczyna:** Pilot używa kodów zmiennych (rolling code)  
**Rozwiązanie:** Użyj prostego pilota uniwersalnego z kodowaniem NEC (np. pilot TV)

### Problem 4: Po zapisaniu konfiguracji przyciski nie działają
**Przyczyna:** Błędny format zapisu (np. brak `0x` przed kodem HEX)  
**Rozwiązanie:** Sprawdź format:
- ✅ Poprawnie: `rcCmdVolumeUp=0xB914`
- ❌ Błędnie: `rcCmdVolumeUp=B914` (brak 0x)
- ❌ Błędnie: `rcCmdVolumeUp = 0xB914` (spacje wokół =)

---

## 📚 PEŁNA LISTA PRZYCISKÓW DO SKONFIGUROWANIA

### Podstawowe (8 przycisków):
- `rcCmdVolumeUp` - Głośność +
- `rcCmdVolumeDown` - Głośność -
- `rcCmdMute` - Wyciszenie/włączenie dźwięku
- `rcCmdPower` / `rcCmdRed` - Power OFF
- `rcCmdArrowUp` - Strzałka w górę
- `rcCmdArrowDown` - Strzałka w dół
- `rcCmdArrowLeft` - Strzałka w lewo
- `rcCmdArrowRight` - Strzałka w prawo

### Nawigacja (7 przycisków):
- `rcCmdOk` - Zatwierdź wybór
- `rcCmdBack` - Powrót
- `rcCmdSrc` - Zmiana trybu wyświetlacza
- `rcCmdDirect` - Toggle nagrywania/dimmer
- `rcCmdBankMinus` - Menu wyboru banku / bank-1
- `rcCmdBankPlus` - Menu wyboru banku / bank+1
- `rcCmdGreen` - Sleep timer / głosowy czas

### Klawiatura numeryczna (10 przycisków):
- `rcCmdKey0` - Klawisz 0
- `rcCmdKey1` - Klawisz 1
- `rcCmdKey2` - Klawisz 2
- `rcCmdKey3` - Klawisz 3
- `rcCmdKey4` - Klawisz 4
- `rcCmdKey5` - Klawisz 5
- `rcCmdKey6` - Klawisz 6
- `rcCmdKey7` - Klawisz 7
- `rcCmdKey8` - Klawisz 8
- `rcCmdKey9` - Klawisz 9

### Equalizer (2 przyciski):
- `rcCmdAud` - Equalizer 3-band/16-band | Double-click: SDPlayer aktywacja
- `rcCmdPIP` - Przełączenie EQ3 ↔ EQ16

### SDPlayer - odtwarzacz (14 przycisków):
- `rcCmdPLAY` - Uruchomienie odtwarzania
- `rcCmdPAUSE` - Pauza/wznowienie
- `rcCmdSTOP` - Zatrzymanie odtwarzania
- `rcCmdREV` - Przewiń -10s | Double-click: poprzedni utwór
- `rcCmdRER` - Przewiń +10s | Double-click: następny utwór
- `rcCmdFILES` / `rcCmdMENU` - Lista plików
- `rcCmdEXIT` / `rcCmdRADIO` - Wyjście do radia
- `rcCmdHELP` - Toggle SDPlayer ON/OFF
- `rcCmdMINIMA` - Następny styl wyświetlania (1→14)
- `rcCmdMAXIMA` - Poprzedni styl wyświetlania (14→1)
- `rcCmdWELEMENT` - PAUSE/PLAY toggle
- `rcCmdWINPOD` - STOP
- `rcCmdBT` - Zmiana stylu (1-14) w SDPlayer | Strona BT w Radio

### Nagrywanie (2 przyciski):
- `rcCmdREC` / `rcCmdBlue` - Toggle nagrywania START/STOP

### Moduł Bluetooth (9 przycisków):
- `rcCmdKOL` - Toggle BT ON/OFF
- `rcCmdCHAT` - Wyświetl status BT
- `rcCmdINFO` - Informacje o module BT
- `rcCmdPOWROT` / `rcCmdGLOS` - Otwórz stronę BT (IP/bt)
- `rcCmdREDLEFT` - Restart modułu BT
- `rcCmdGREENL` - Test połączenia BT
- `rcCmdCHPlus` - Jasność OLED +20
- `rcCmdCHMinus` - Jasność OLED -20

### Dodatkowe (2 przyciski):
- `rcCmdYellow` - Toggle Analyzer ON/OFF

### Zarezerwowane (4 przyciski - opcjonalne):
- `rcCmdTV` - Zarezerwowane (można ustawić 0x0000)
- `rcCmdWWW` - Zarezerwowane (można ustawić 0x0000)
- `rcCmdGAZE` - Zarezerwowane (można ustawić 0x0000)
- `rcCmdEPG` - Zarezerwowane (można ustawić 0x0000)

**RAZEM: 62 przyciski** (58 aktywnych + 4 zarezerwowane)

---

## 💡 WSKAZÓWKI

1. **Zacznij od podstawowych przycisków:**
   - VOL+/VOL-
   - Strzałki (4)
   - OK, BACK
   - POWER
   
2. **Przypisuj kody stopniowo:**
   - Najpierw 10 podstawowych
   - Potem klawiaturę numeryczną
   - Na końcu specjalne funkcje

3. **Testuj na bieżąco:**
   - Po każdych 5-10 przyciskach zapisz konfigurację
   - Zrestartuj radio
   - Sprawdź czy przyciski działają

4. **Zachowaj kopię:**
   - Zapisz plik `/config_remote.txt` na komputerze
   - Zrób zrzut ekranu tabeli z kodami

---

## ✅ LISTA KONTROLNA

- [ ] Radio połączone przez USB
- [ ] Serial Monitor otwarty (115200 baud)
- [ ] Pilot gotowy (dobre baterie)
- [ ] Tabela przygotowana do zapisywania kodów
- [ ] Wszystkie 62 kody odczytane i zapisane
- [ ] Plik `/config_remote.txt` utworzony na karcie SD
- [ ] Radio zrestartowane
- [ ] Wszystkie przyciski przetestowane
- [ ] Kopia zapasowa konfiguracji wykonana

---

## 📞 POMOC

Jeśli masz problemy z konfiguracją pilota:

1. Sprawdź forum: https://github.com/dzikakuna/ESP32_radio_evo3/issues
2. Wyślij log Serial Monitor z komendą DEBUG
3. Dołącz listę kodów HEX które odczytałeś

**Powodzenia!** 🎉
