# Poprawki Bluetooth WebUI

## Naprawione problemy:

### 1. ✅ Status Bluetooth nie aktualizował się
**Problem:** ESP wysyłał dane przez UART, ale status na stronie WWW nie był aktualizowany.

**Rozwiązanie:** 
- Zmieniono `loop()` aby **zawsze** odbierać dane z UART (wcześniej odbierało tylko gdy terminal był włączony)
- Dodano automatyczne odpytywanie `STATUS?` co 3 sekundy gdy terminal jest wyłączony
- Status jest teraz aktualizowany na bieżąco niezależnie od stanu terminala

### 2. ✅ Terminal zawierał śmieci
**Problem:** W konsoli terminala pojawiały się nieprawidłowe znaki i śmieci.

**Rozwiązanie:**
- Dodano filtrowanie znaków - akceptowane tylko drukowane ASCII (32-126) + `\r\n`
- Przy włączaniu terminala bufor UART jest czyszczony z ewentualnych śmieci
- Log konsoli jest czytelny i profesjonalnie sformatowany

### 3. ✅ Terminal nie działał prawidłowo
**Problem:** Brak możliwości wysłania komendy i odczytu debugowania.

**Rozwiązanie:**
- Terminal działa poprawnie - można wysyłać komendy
- Odpowiedzi z modułu BT są wyświetlane w konsoli
- Log jest zapisywany tylko gdy terminal jest aktywny (oszczędność pamięci)

---

## Jak testować:

### Test 1: Status bez terminala
1. Wejdź na stronę `/bt`
2. **NIE** zaznaczaj checkboxa "Włącz terminal"
3. Poczekaj 3-5 sekund
4. Status powinien się zaktualizować automatycznie:
   - `Bluetooth OFF` lub `Bluetooth ON`
   - `Mode: OFF/RX/TX/AUTO`
   - `Device: None` lub nazwa urządzenia

### Test 2: Terminal diagnostyczny
1. Wejdź na stronę `/bt`
2. Zaznacz checkbox **"Włącz terminal"**
3. Terminal powinien się otworzyć z komunikatem:
   ```
   === Terminal BT aktywny ===
   Gotowy do wysyłania komend (np. STATUS?, HELP, PING)
   
   > STATUS?
   < [odpowiedź z modułu]
   ```

### Test 3: Wysyłanie komend
1. W terminalu wpisz komendę, np:
   - `STATUS?` - pobierz status
   - `HELP` - lista dostępnych komend
   - `PING` - sprawdź połączenie
   - `GET` - odczytaj ustawienia
2. Kliknij "Wyślij komendę"
3. W konsoli powinny pojawić się:
   - `> [twoja komenda]`
   - `< [odpowiedź z modułu]`

### Test 4: Zmiana trybu
1. Kliknij przycisk "RX (Receiver)" lub "TX (Transmitter)"
2. Status powinien się zaktualizować
3. W terminalu (jeśli włączony) zobaczysz:
   ```
   > MODE RX
   < OK MODE RX
   ```

---

## Format odpowiedzi modułu BT:

Moduł BT powinien odpowiadać w formacie:
```
STATE BT=ON MODE=TX VOL=100 BOOST=400 SCAN=0 CONN=1 MAC=AA:BB:CC:DD:EE:FF NAME="Moje urządzenie"
```

Lub:
```
OK MODE RX
OK VOL 100
OK BT ON
PONG
```

---

## Zmienione pliki:

- [bt/BTWebUI.h](src/bt/BTWebUI.h) - dodano `_lastStatusPoll`
- [bt/BTWebUI.cpp](src/bt/BTWebUI.cpp) - naprawiono `loop()`, `addToLog()`, `handleTerminalEnable()`

---

## Diagnostyka problemów:

### Status nie aktualizuje się:
1. Sprawdź w monitorze szeregowym czy widać:
   ```
   BT UART initialized on RX:19 TX:20
   BTWebUI: /bt/api/state requested
   ```
2. Co 3 sekundy powinno być wysyłane `STATUS?` gdy terminal wyłączony

### Terminal pokazuje śmieci:
1. Wyłącz terminal (odznacz checkbox)
2. Poczekaj 2 sekundy
3. Włącz terminal ponownie
4. Bufor UART powinien być wyczyszczony

### Moduł BT nie odpowiada:
1. Sprawdź połączenie UART: RX=19, TX=20
2. Sprawdź prędkość: 115200 baud
3. Sprawdź zasilanie modułu BT
4. W terminalu wyślij `PING` - powinieneś otrzymać `PONG`

---

**Data poprawek:** 2026-02-07
**Wersja:** v3.19.53+bt-fix
