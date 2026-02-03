# Przewodnik Użycia - Equalizer EQ16

## Szybki Start

### 1. Kompilacja projektu

```bash
# W PlatformIO:
pio run
```

### 2. Wgranie firmware

```bash
pio run --target upload
```

### 3. Pierwsze uruchomienie

Po wgraniu nowego firmware radio automatycznie:
- Inicjalizuje equalizer EQ16
- Ustawia wszystkie pasma na 0dB (Flat)
- Tworzy plik `/eq16.txt` na karcie SD

## Menu Equalizera

### Przełączanie systemów EQ

**3-punktowy EQ** (domyślny):
- Low (basy)
- Mid (środek)
- High (wysokie)

**16-pasmowy EQ** (zaawansowany):
- 16 pasm od 32Hz do 16kHz
- Konwersja na 3 punkty z wagami
- Bardziej precyzyjna kontrola

### Nawigacja w menu EQ16

**Przycisk Menu** - Wejście/wyjście z menu EQ16  
**Lewo/Prawo** - Wybór pasma (1-16)  
**Góra/Dół** - Regulacja wzmocnienia (-16 do +16 dB)  
**OK** - Zapisz ustawienia

## Presety

### Dostępne presety (0-6):

| ID | Nazwa | Opis | Najlepsze dla |
|----|-------|------|---------------|
| 0 | **Flat** | Neutralne brzmienie | Testy, porównania |
| 1 | **Bass Boost** | Mocne basy +8dB | Hip-hop, EDM, elektronika |
| 2 | **Vocal Clarity** | Wyraźne wokale | Podcasty, audiobooki |
| 3 | **Radio Presence** | Środek podkreślony | Stacje radiowe |
| 4 | **V-Shape** | Basy + wysokie | Pop, rock nowoczesny |
| 5 | **Rock/Metal** | Agresywne | Rock, metal, punk |
| 6 | **Jazz/Classical** | Naturalne | Jazz, klasyka, akustyka |

### Wczytywanie presetu

```cpp
// Z poziomu kodu:
EQ16_loadPreset(1);  // Bass Boost

// Z menu (planowane):
// Menu -> Presety -> Bass Boost
```

## Mapowanie Częstotliwości

### 16 pasm -> 3 punkty (Low/Mid/High)

```
PASMO  | CZĘSTOTLIWOŚĆ | GRUPA | WAGA  | ZASTOSOWANIE
-------|---------------|-------|-------|------------------
  0    |    32 Hz      | LOW   | 1.2   | Sub-bass (syntezatory)
  1    |    64 Hz      | LOW   | 1.5   | Bass (werbel, kick)
  2    |   125 Hz      | LOW   | 1.3   | Low-mid bass
  3    |   250 Hz      | LOW   | 1.0   | Ciepło basu
  4    |   500 Hz      | LOW   | 0.8   | Przejście low-mid
-------|---------------|-------|-------|------------------
  5    |    1 kHz      | MID   | 1.0   | Podstawa mid
  6    |    2 kHz      | MID   | 1.3   | Obecność wokalu
  7    |    3 kHz      | MID   | 1.5   | GŁÓWNY WOKAL (max waga)
  8    |    4 kHz      | MID   | 1.3   | Przejrzystość
  9    |    5 kHz      | MID   | 1.0   | Detale
 10    |    6 kHz      | MID   | 0.8   | Przejście mid-high
-------|---------------|-------|-------|------------------
 11    |    8 kHz      | HIGH  | 1.0   | Podstawa high
 12    |   10 kHz      | HIGH  | 1.2   | Airiness
 13    |   12 kHz      | HIGH  | 1.1   | Szczegóły
 14    |   14 kHz      | HIGH  | 0.9   | Ekstremum
 15    |   16 kHz      | HIGH  | 0.8   | Ultrahigh (łagodzenie)
```

## Przykłady Użycia

### 1. Wzmocnienie wokalów (radio)

```
Pasma 6-8 (2-4 kHz): +4 do +6 dB
Pasma 0-2 (32-125 Hz): -2 do -3 dB (wycięcie basu)
Pozostałe: 0 dB
```

### 2. Więcej basu (elektronika)

```
Pasmo 0 (32 Hz): +8 dB (sub-bass)
Pasmo 1 (64 Hz): +7 dB (kick)
Pasmo 2 (125 Hz): +6 dB
Pasma 3-4: stopniowo do 0 dB
Pozostałe: 0 lub -1 dB
```

### 3. V-Shape (modern pop)

```
Pasma 0-2: +6 do +4 dB (basy)
Pasma 5-8: -2 do -3 dB (wycięcie środka)
Pasma 12-15: +4 do +7 dB (wysokie)
```

## Zapis i Odczyt

### Automatyczny zapis

Ustawienia są automatycznie zapisywane:
- Po 5 sekundach od ostatniej zmiany
- Po wczytaniu presetu
- Po resecie pasm

### Ręczny zapis

```cpp
EQ16_saveToSD();  // Zapis do /eq16.txt
```

### Wczytanie z SD

```cpp
EQ16_loadFromSD();  // Odczyt z /eq16.txt
```

### Format pliku eq16.txt

```
0    // Pasmo 0 (32 Hz)
0    // Pasmo 1 (64 Hz)
0    // Pasmo 2 (125 Hz)
...  // ... itd dla wszystkich 16 pasm
0    // Pasmo 15 (16 kHz)
```

## Debugowanie

### Logi Serial Monitor

```
EQ16_init() - 16-Band Equalizer initialized
EQ16->setTone: Low=2dB, Mid=4dB, High=1dB (weighted avg)
EQ16: Preset 'Bass Boost' loaded, applied and saved
EQ16: All bands reset to 0dB, applied to audio and saved to SD
```

### Monitorowanie pamięci

```
Free Heap: 245678 bytes | Free PSRAM: 3897456 bytes
```

## Rozwiązywanie Problemów

### Problem: Brak zmian w dźwięku

**Rozwiązanie:**
1. Sprawdź czy EQ16 jest włączony: `EQ16_enable(true);`
2. Sprawdź czy wybrano EQ16 (nie 3-punktowy)
3. Sprawdź logi: `EQ16->setTone: ...`

### Problem: Zniekształcenia audio

**Rozwiązanie:**
1. Zmniejsz wzmocnienia pasm (max +6 zamiast +12)
2. Użyj presetu Flat (0) jako punkt wyjścia
3. Reguluj stopniowo po 1-2 dB

### Problem: Plik eq16.txt nie zapisuje się

**Rozwiązanie:**
1. Sprawdź czy karta SD jest zamontowana
2. Sprawdź uprawnienia zapisu
3. Sprawdź logi: `ERROR: Failed to open /eq16.txt`

## Najlepsze Praktyki

### 1. Zacznij od Flat

Zawsze zacznij od presetu Flat (0) i reguluj stopniowo.

### 2. Małe zmiany

Zmiany 1-2 dB są często wystarczające. Unikaj ekstremalnych wartości (+12/-12).

### 3. Odsłuchuj na różnych źródłach

Testuj ustawienia na różnych stacjach radiowych i plikach lokalnych.

### 4. Używaj presetów jako bazy

Zamiast tworzyć od zera, zmodyfikuj istniejący preset.

### 5. Regularnie zapisuj

Po znalezieniu idealnych ustawień, zapisz je ręcznie jako backup.

## Zaawansowane

### Tworzenie własnego presetu

1. Załaduj Flat (0)
2. Reguluj poszczególne pasma
3. Zapisz jako eq16.txt
4. Utwórz backup (eq16_custom.txt)

### Edycja pliku eq16.txt

Możesz edytować plik bezpośrednio:
```
8    // Pasmo 0: +8dB (mocny sub-bass)
6    // Pasmo 1: +6dB
4    // Pasmo 2: +4dB
...
```

Po edycji wywołaj: `EQ16_loadFromSD();`

## Podsumowanie

✅ **7 profesjonalnych presetów** gotowych do użycia  
✅ **Ważona konwersja** dla lepszego brzmienia  
✅ **Automatyczny zapis** - nie tracisz ustawień  
✅ **Kompatybilność** z nową biblioteką ESP32-audioI2S  
✅ **Intuicyjne menu** - łatwa obsługa  

**Miłego słuchania!** 🎵
