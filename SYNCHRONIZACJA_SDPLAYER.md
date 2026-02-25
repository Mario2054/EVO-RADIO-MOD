# 🔄 Synchronizacja SDPlayer - Strona ↔ OLED

## ✨ Nowe Funkcje Synchronizacji

System SDPlayer został rozszerzony o pełną **dwukierunkową synchronizację** między stroną internetową a wyświetlaczem OLED.

### 🎯 Główne Funkcjonalności

#### 1. **Automatyczne Przełączanie OLED**
- **Ze strony → OLED**: Gdy uruchomimy SDPlayer ze strony, OLED automatycznie przełącza się na tryb SDPlayer
- **Splash Screen**: Po przełączeniu pokazuje się splash "SD PLAYER" przez 1 sekundę
- **Ten sam stan**: OLED wyświetla te same informacje co strona internetowa

#### 2. **Real-time Synchronizacja Utworów**
- **Zmiana na stronie**: Gdy zmieniamy utwór na stronie, automatycznie aktualizuje się na OLED
- **Zmiana z pilota**: Gdy zmieniamy utwór pilotem/enkoderem, automatycznie aktualizuje się na stronie
- **Stan odtwarzania**: Play/Pause/Stop synchronizuje się w obu kierunkach

#### 3. **Synchronizacja z Pilota**
- **Uruchomienie z pilota**: Gdy aktywujemy SDPlayer pilotem (triple-click Key9 lub enkoder)
- **Auto-sync strony**: Strona automatycznie synchronizuje się z aktualnie odtwarzanym utworem
- **Kontynuacja playbacku**: Jeśli coś już grało, strona pokazuje aktualny stan

## 🔧 Implementacja Techniczna

### Callbacki Synchronizacji

```cpp
// WebUI → OLED
webUI->setIndexChangeCallback([oled](int newIndex) {
    oled->setSelectedIndex(newIndex);
});

// OLED → WebUI  
oled->setIndexChangeCallback([webUI](int newIndex) {
    webUI->notifyIndexChange(newIndex);
});
```

### Automatyczna Aktywacja

```cpp
// W SDPlayerWebUI::playFile()
if (_oled && !_oled->isActive()) {
    _oled->activate();              // Aktywuj OLED
    _oled->showSplash();           // Pokaż splash
    sdPlayerOLEDActive = true;     // Ustaw flagę globalną
}
```

### Synchronizacja Pilota

```cpp
// W main.cpp - aktywacja z pilota
if (g_sdPlayerWeb && sdPlayerPlayingMusic) {
    g_sdPlayerWeb->setIsPlaying(true);
    if (currentTrackSelection >= 0) {
        g_sdPlayerWeb->setCurrentFile(trackFiles[currentTrackSelection]);
    }
}
```

## 🎮 Jak Używać

### Ze Strony Internetowej:
1. **Wejdź na `/sdplayer`** - OLED automatycznie się przełączy
2. **Wybierz utwór** - OLED natychmiast pokaże ten sam utwór
3. **Play/Pause/Stop** - wszystko synchronizuje się z OLED
4. **Nawigacja** - lista na OLED także się synchronizuje

### Z Pilota/Enkodera:
1. **Triple-click** (Key9 lub enkoder) - aktywacja SDPlayer
2. **Strzałki ↑/↓** - nawigacja, synchronizuje z stroną
3. **OK** - play, strona pokazuje ten sam utwór
4. **Double-click Left/Right** - prev/next, strona reaguje

### Real-time w Obu Kierunkach:
- **Strona → OLED**: Zmiana na stronie = natychmiastowa aktualizacja OLED
- **Pilot → Strona**: Zmiana pilotem = automatyczna aktualizacja strony (po 3s refresh)

## 📋 Zmodyfikowane Pliki

### Główne Zmiany:

1. **`SDPlayerWebUI.h/cpp`**
   - Dodane callbacki synchronizacji
   - Auto-aktywacja OLED
   - Funkcje `notifyIndexChange()`, `notifyFileChange()`, `notifyPlayStateChange()`

2. **`SDPlayerOLED.h/cpp`**
   - Callback dla sync z WebUI
   - `notifyWebUIIndexChange()` w `scrollUp()/scrollDown()`
   - Dwukierunkowa komunikacja

3. **`SDPlayerManager.cpp`**
   - Setup callbacków w `setOLED()`
   - Centralne zarządzanie synchronizacją

4. **`main.cpp`**
   - Synchronizacja przy aktywacji z pilota
   - Setup callbacków w `SDPlayerOLED_init()`
   - Real-time sync w `handleList()`

## ✅ Stan Funkcjonalności

| Funkcja | Status | Opis |
|---------|---------|------|
| 🌐 Strona → OLED | ✅ | Auto-przełączanie i sync |
| 🎮 Pilot → Strona | ✅ | Auto-sync po aktywacji |
| 🎵 Real-time Utwory | ✅ | Dwukierunkowo |
| ⏯️ Play/Pause Sync | ✅ | Pełna synchronizacja |
| 📋 Lista Sync | ✅ | Indeksy synchronizowane |
| 🔊 Volume Sync | ✅ | Wspólny stan |

## 🔍 Debug

Wszystkie operacje synchronizacji logują do Serial:

```
[SDPlayerManager] WebUI->OLED sync: index 5
[Main] OLED->WebUI index sync: 3  
[SDPlayer] Synchronized WebUI with current track: song.mp3
```

## 🎯 Rezultat

Teraz **SDPlayer działa jako jeden spójny system** - bez względu na to czy używasz strony internetowej czy pilota, zawsze widzisz te same informacje i możesz sterować tym samym odtwarzaczem. Synchronizacja działa **w czasie rzeczywistym** i **w obu kierunkach**!