# Aktualizacja ESP32-audioI2S do wersji 3.4.4

**Data:** 25 lutego 2026  
**Status:** ✅ Zakończona pomyślnie

---

## 📋 Wykonane Czynności

### 1. Analiza bibliotek
- **Lokalna biblioteka** `lib/ESP32_audioI2S/` - przestarzała wersja z zależnością ESP-DSP
- **Biblioteka repo** `.pio/libdeps/` - najnowsza wersja 3.4.4 z GitHub
- **Różnica:** +20 KB kodu, własna implementacja IIR bez ESP-DSP

### 2. Backup i usunięcie
- ✅ Utworzono backup: `backup/ESP32_audioI2S_backup_20260225_*`
- ✅ Usunięto przestarzałą lokalną bibliotekę
- ✅ Kompilacja używa obecnie tylko wersji z repozytorium

### 3. Test kompilacji
```
✅ RAM:   24.3% (79,576 bajtów / 327,680 bajtów)
✅ Flash: 80.7% (2,538,802 bajtów / 3,145,728 bajtów)
✅ Czas kompilacji: 108 sekund
```

---

## 🆕 Nowe Funkcje w v3.4.4 (135 commitów od v3.4.3)

### 🎵 Wysokiej Rozdzielczości Audio
- **FLAC 24-bit** - profesjonalna jakość dźwięku
- **WAV 24/32-bit** - studio-quality audio
- **32-bitowe przetwarzanie** - lepsza dynamika i czystość

### 🔊 Nowy System Filtrów IIR
**Poprzednio:**
```cpp
// Wersja lokalna - wymagała ESP-DSP library
dsps_biquad_sf32(s, s, 1, coeffs, state_biquad);
```

**Teraz:**
```cpp
// Wersja 3.4.4 - własna implementacja, brak zależności
void IIR_filterChain0_s16(int16_t* iir_in, bool clear);  // 16-bit
void IIR_filterChain0_s32(int32_t* iir_in, bool clear);  // 32-bit
```

**Zalety:**
- ✅ Brak zależności od ESP-DSP
- ✅ Wspiera 16-bit i 32-bit
- ✅ Bardziej przenośny kod
- ✅ Lepsza optymalizacja

### 🎤 OpenAI TTS Integration
```cpp
bool openai_speech(
    const String& api_key,
    const String& model,         // "tts-1" lub "tts-1-hd"
    const String& input,         // Tekst do syntezy
    const String& instructions,  // Dodatkowe instrukcje
    const String& voice,         // "alloy", "echo", "fable", "onyx", "nova", "shimmer"
    const String& response_format, // "mp3", "opus", "aac", "flac"
    const String& speed          // "0.25" - "4.0"
);
```

**Przykład użycia:**
```cpp
audio.openai_speech(
    OPENAI_API_KEY, 
    "tts-1", 
    "Teraz gra: Radio Zet", 
    "", 
    "shimmer", 
    "mp3", 
    "1"
);
```

### ⚙️ Dynamiczna Rekonfiguracja I2S
```cpp
void reconfigI2S();  // Zmiana parametrów bez restartu
```

### 📊 VU Meter dla Hi-Res Audio
```cpp
void computeVUlevel(int16_t sample[2]);   // Standard 16-bit
void computeVUlevel1(int32_t* sample);    // Hi-res 24/32-bit
```

### 🎚️ Nowe Funkcje Audio
```cpp
void Gain(int16_t* sample);                // Wzmocnienie 16-bit
void Gain1(int32_t* sample);               // Wzmocnienie 32-bit
size_t resampleTo48kStereo(const int16_t* input, size_t inputFrames);
```

---

## 📝 Zmiany API

### Sygnatura `audio_process_i2s()`
**PRZED (lokalna lib - błędna):**
```cpp
void audio_process_i2s(int32_t* outBuff, int16_t validSamples, bool* continueI2S);
```

**PO (v3.4.4 - poprawna):**
```cpp
void audio_process_i2s(int16_t* outBuff, int32_t validSamples, bool* continueI2S);
```

**Status:** ✅ Kod w `main.cpp` już zaktualizowany i kompatybilny

---

## 🐛 Poprawione Błędy (wybrane z 135)

### Streaming
- ✅ Podwójny nagłówek ID3 w HLS AAC
- ✅ Zniekształcenia w FLAC wysokiej rozdzielczości
- ✅ Problemy z Vorbis stream title/artist
- ✅ BBC station rebooting (#1194)
- ✅ Google TTS audio digitization (#1217)
- ✅ Wrong URL redirect (#1213)

### Dekodery
- ✅ FLAC z okładkami z webserwera
- ✅ Vorbis/Opus cover image parsing
- ✅ MP3 MAIN_DATA_UNDERFLOW
- ✅ MP3 ID3v1, ID3v2.2, APETAGEX support
- ✅ WAV 8/16/24/32-bit mono/stereo

### Bufory i Pamięć
- ✅ AudioBuffer mutex protection
- ✅ Semaphory dla bezpiecznej wielowątkowości
- ✅ Memory leak precautions w Vorbis
- ✅ Multi-heap poisoning fix

### I2S
- ✅ I2S_CLK_SRC_APLL dla lepszej jakości
- ✅ MCLK multiple 384 dla hi-res
- ✅ DMA frame_num optymalizacja

---

## 🔧 Konfiguracja

### platformio.ini
```ini
lib_deps = 
  https://github.com/schreibfaul1/ESP32-audioI2S.git#3.4.4
```

### Build Info
```
Platform: pioarduino/platform-espressif32 54.03.20
Framework: Arduino ESP32
Toolchain: GCC 8.4.0 (xtensa-esp32s3-elf)
C++ Standard: gnu++17
```

---

## 💡 Rekomendacje Dalszego Rozwoju

### 1. Integracja OpenAI TTS
- Dodać endpoint w WebUI do syntezy mowy
- Parametry: głos, prędkość, format
- Zastosowanie: powiadomienia, ogłoszenia radiowe

### 2. Wsparcie FLAC 24-bit
- Włączyć obsługę hi-res streamów
- Dodać indicator jakości w UI
- Testować z PSRAM cache workarounds

### 3. Optymalizacja VU Meter
- Wykorzystać `computeVUlevel1()` dla 32-bit
- Dodać wskaźnik bitrate/format w czasie rzeczywistym

### 4. Debug Tools
```cpp
audio.inBufferStatus();  // Nowa funkcja debugowania bufora
```

---

## 📚 Linki

- **Repozytorium:** https://github.com/schreibfaul1/ESP32-audioI2S
- **Release 3.4.4:** https://github.com/schreibfaul1/ESP32-audioI2S/releases/tag/3.4.4
- **Dokumentacja:** README w repozytorium

---

## ✅ Checklist Weryfikacji

- [x] Backup starej biblioteki utworzony
- [x] Lokalna biblioteka usunięta
- [x] Kompilacja bez błędów
- [x] Rozmiar firmware w limitach
- [x] API kompatybilne z main.cpp
- [x] Nowe funkcje dostępne do użycia

---

**Konkluzja:** Aktualizacja przebiegła pomyślnie. Firmware używa teraz najnowszej wersji ESP32-audioI2S 3.4.4 z repozytorium GitHub, co zapewnia:
- 135 bugfixów i ulepszeń
- Wsparcie dla audio wysokiej rozdzielczości (24/32-bit)
- OpenAI TTS integration
- Lepszą implementację filtrów IIR bez zależności ESP-DSP
- Bezpieczniejszą wielowątkowość z mutexami
