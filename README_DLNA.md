# DLNA Browser - Dokumentacja

## Przegląd

DLNA Browser to nowa funkcjonalność dodana do ESP32 Radio Evo, która umożliwia przeglądanie i odtwarzanie muzyki z serwerów DLNA/UPnP w sieci lokalnej.

## Funkcje

### 1. **Inicjalizacja DLNA**
- Automatyczne wykrywanie serwera DLNA w sieci (SSDP)
- Rozpoznawanie ContentDirectory service
- Budowanie indeksu kategorii root

### 2. **Przeglądanie Kategorii**
- Lista głównych folderów/kategorii z serwera DLNA
- Nawigacja po strukturze folderów
- Wyświetlanie kontenerów i itemów

### 3. **Budowanie Playlist**
- **USE DLNA PL** - Buduje playlistę i automatycznie przełącza na tryb DLNA
- **USE WEB PL** - Tylko buduje playlistę (bez przełączenia trybu)

## Użycie

### Konfiguracja

Przed użyciem DLNA Browser, skonfiguruj parametry w pliku `main.cpp`:

```cpp
// ---- DLNA Configuration ---- //
#ifdef USE_DLNA
String dlnaIDX = "0";                      // Root ObjectID DLNA (domyślnie "0" dla Music)
const char* dlnaHost = "192.168.1.100";    // Adres IP serwera DLNA (zmień na swój)
#endif
```

**WAŻNE:** Zmień `dlnaHost` na adres IP swojego serwera DLNA (np. NAS, komputer z Plex, MinimServer, itp.)

### Dostęp do interfejsu

1. Otwórz przeglądarkę
2. Przejdź do `http://<IP_RADIA>/menu`
3. Kliknij **DLNA Browser**

### Instrukcja krok po kroku

1. **Inicjuj DLNA**
   - Kliknij przycisk "Inicjuj DLNA Server"
   - Poczekaj na wykrycie serwera (max 15 sekund)
   
2. **Wybierz kategorię**
   - Z listy rozwijanej wybierz kategorię muzyki (np. "Music", "Artists", "Albums")
   - System automatycznie załaduje zawartość
   
3. **Przeglądaj zawartość**
   - W sekcji ITEMS zobaczysz:
     - 📁 Kontenery (podobne do folderów)
     - 🎵 Pliki audio
     
4. **Użyj playlisty**
   - **USE DLNA PL** - Zbuduje playlistę i rozpocznie odtwarzanie
   - **USE WEB PL** - Zbuduje playlistę do późniejszego użycia

## Zachowanie

### Przełączanie trybu (jak SD Player)

Zachowanie przełączania **USE DLNA PL** jest analogiczne do SD Player:

- Jeśli radio **gra** → automatycznie rozpoczyna odtwarzanie z DLNA po przełączeniu
- Jeśli radio jest **zatrzymane** → tylko przełącza źródło (bez automatycznego play)

## Architektura

### Pliki modułu DLNA

```
src/network/
├── DLNAWebUI.h/cpp          # Interfejs webowy
├── dlna_service.h/cpp       # Serwis DLNA (init, API)
├── dlna_worker.h/cpp        # Worker task (async budowanie playlist)
├── dlna_index.h/cpp         # Przeglądanie i indeksowanie DLNA
├── dlna_ssdp.h/cpp          # SSDP discovery
├── dlna_desc.h/cpp          # Parsowanie opisu urządzenia
└── dlna_http_guard.h/cpp    # Mutex dla HTTP requests

src/core/
├── options.h                # Flagi funkcji (USE_DLNA)
└── config.h                 # Ścieżki plików (PLAYLIST_DLNA_PATH)
```

### Endpointy API

| Endpoint | Metoda | Opis |
|----------|--------|------|
| `/dlna` | GET | Główna strona DLNA Browser |
| `/dlna/api/init` | POST | Inicjalizacja DLNA (SSDP + ContentDirectory) |
| `/dlna/api/categories` | GET | Lista kategorii root |
| `/dlna/api/list?id=xxx` | GET | Zawartość kontenera |
| `/dlna/api/build` | POST | Budowanie playlisty (activate=0/1) |
| `/dlna/api/switch` | POST | Przełączenie na tryb DLNA |
| `/dlna/api/status` | GET | Status operacji |

## Mechanizm działania

### 1. Inicjalizacja

```
SSDP Discovery → Device Description → ContentDirectory URL → Root Browse
```

### 2. Budowanie playlisty

```
Browse Container → Extract Items → Deep Traverse → Save to SPIFFS → Atomic Rename
```

### 3. Worker Task

- Asynchroniczne przetwarzanie (nie blokuje UI)
- FreeRTOS task z kolejką zadań
- Mutexey dla bezpiecznego dostępu do HTTP i SPIFFS
- Timeout i error handling

## Rozwiązywanie problemów

### "SSDP discover failed"

- Sprawdź czy serwer DLNA jest włączony
- Sprawdź czy `dlnaHost` ma poprawny adres IP
- Upewnij się że radio i serwer są w tej samej sieci

### "ContentDirectory not found"

- Serwer DLNA nie udostępnia usługi ContentDirectory
- Spróbuj innego serwera (Plex, MinimServer, Universal Media Server)

### "No tracks in this container"

- Wybrany folder nie zawiera plików audio (tylko podkatalogi)
- Wybierz inny folder lub zagłęb się głębiej w strukturę

### "Playlist build timeout"

- Za dużo plików do przetworzenia (limit: 5000)
- Serwer DLNA nie odpowiada
- Problemy z siecią WiFi

## Ograniczenia

- **Limit playlist:** 5000 utworów
- **Deep traverse:** Maksymalnie 4 poziomy zagnieżdżenia
- **Format:** Tylko pliki audio wspierane przez ESP32-audioI2S (MP3, AAC, FLAC, VORBIS, OPUS)
- **Timeout:** 30 sekund na budowanie playlisty

## Przyszłe usprawnienia

- [ ] Bezpośrednie odtwarzanie pojedynczych utworów
- [ ] Przewijanie w trybie DLNA
- [ ] Cache'owanie struktury folderów
- [ ] Wyszukiwanie po artystach/albumach
- [ ] Integracja z OLED (wyświetlanie statusu)
- [ ] Obsługa pilota IR (nawigacja DLNA)

## Kompatybilność

### Przetestowane serwery DLNA

- ✅ Plex Media Server
- ✅ MinimServer
- ✅ Universal Media Server (UMS)
- ✅ Serviio
- ⚠️ Windows Media Player (podstawowa obsługa)

### Wymagania

- ESP32 z WiFi
- Serwer DLNA/UPnP w sieci lokalnej
- SPIFFS/LittleFS (dla playlist cache)
- Pamięć: ~4KB RAM na worker task

## Licencja

Moduł DLNA jest częścią ESP32 Radio Evo i podlega tej samej licencji co projekt główny.

---

**Autor:** Implementacja DLNA Browser - 2026  
**Projekt:** ESP32 Radio Evo v3.19.70+  
**GitHub:** https://github.com/dzikakuna/ESP32_radio_evo3
