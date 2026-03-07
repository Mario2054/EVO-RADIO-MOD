<!DOCTYPE html>
<html lang="pl">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>EVO Internet Radio v3.20.04 MOD – Dokumentacja HTML</title>
  <style>
    :root {
      --bg: #0f172a;
      --card: #111827;
      --card-2: #1f2937;
      --line: #334155;
      --text: #e5e7eb;
      --muted: #94a3b8;
      --accent: #22c55e;
      --accent-2: #38bdf8;
      --warn: #f59e0b;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Arial, Helvetica, sans-serif;
      background: linear-gradient(180deg, #020617 0%, var(--bg) 100%);
      color: var(--text);
      line-height: 1.5;
    }
    .wrap {
      max-width: 1400px;
      margin: 0 auto;
      padding: 24px;
    }
    .hero {
      background: linear-gradient(135deg, rgba(34,197,94,.15), rgba(56,189,248,.15));
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 28px;
      margin-bottom: 24px;
      box-shadow: 0 12px 30px rgba(0,0,0,.25);
    }
    h1, h2, h3 { margin-top: 0; }
    h1 { font-size: 30px; }
    h2 {
      margin-top: 28px;
      font-size: 24px;
      padding-bottom: 8px;
      border-bottom: 2px solid var(--line);
    }
    p, li { color: var(--text); }
    .muted { color: var(--muted); }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
      gap: 18px;
    }
    .card {
      background: rgba(17,24,39,.92);
      border: 1px solid var(--line);
      border-radius: 16px;
      padding: 18px;
      box-shadow: 0 8px 24px rgba(0,0,0,.18);
    }
    .card h3 {
      color: var(--accent);
      margin-bottom: 10px;
      font-size: 18px;
    }
    .badge {
      display: inline-block;
      padding: 6px 10px;
      background: rgba(56,189,248,.15);
      color: #bae6fd;
      border: 1px solid rgba(56,189,248,.35);
      border-radius: 999px;
      font-size: 12px;
      margin-right: 8px;
      margin-bottom: 8px;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 12px;
      background: rgba(17,24,39,.9);
      border-radius: 14px;
      overflow: hidden;
    }
    th, td {
      border: 1px solid var(--line);
      padding: 12px 10px;
      text-align: left;
      vertical-align: top;
    }
    th {
      background: #0b1220;
      color: #cbd5e1;
      font-weight: 700;
    }
    tr:nth-child(even) td { background: rgba(31,41,55,.45); }
    .section {
      margin-top: 26px;
    }
    .small { font-size: 13px; color: var(--muted); }
    .note {
      margin-top: 18px;
      padding: 14px 16px;
      border-left: 4px solid var(--warn);
      background: rgba(245,158,11,.08);
      border-radius: 10px;
    }
    code {
      color: #a7f3d0;
      background: rgba(34,197,94,.08);
      padding: 2px 6px;
      border-radius: 6px;
    }
    .footer {
      margin-top: 28px;
      color: var(--muted);
      font-size: 13px;
      text-align: center;
    }
    @media (max-width: 700px) {
      h1 { font-size: 24px; }
      h2 { font-size: 20px; }
      th, td { font-size: 14px; }
      .wrap { padding: 14px; }
      .hero { padding: 18px; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="hero">
      <h1>EVO Internet Radio v3.20.04 MOD</h1>
      <p><strong>Wersja:</strong> EVO 3.20.04 MOD SDP_BT_EQ16_ANALIZATOR SD PLAYER SD RECORDING DLNA</p>
      <p class="muted">
        Dokument przygotowany na podstawie opisu projektu opartego na oryginalnym oprogramowaniu
        <code>ESP32_radio_evo3</code>. Zawiera funkcje firmware, podsumowanie pinów oraz pełną tabelę obsługi pilota IR NEC.
      </p>
      <div>
        <span class="badge">Internet Radio</span>
        <span class="badge">SD Player</span>
        <span class="badge">SD Recording</span>
        <span class="badge">EQ3 / EQ16</span>
        <span class="badge">FFT Analyzer</span>
        <span class="badge">DLNA</span>
        <span class="badge">Bluetooth UART</span>
        <span class="badge">WebUI + OTA</span>
      </div>
    </div>

    <h2>1. Funkcje firmware</h2>
    <div class="grid">
      <div class="card">
        <h3>1. Internet Radio</h3>
        <ul>
          <li>17 banków stacji, do 99 stacji na bank</li>
          <li>Pobieranie list z GitHub: <code>bank01.txt</code> – <code>bank17.txt</code></li>
          <li>Wyświetlanie nazwy stacji, tytułu, formatu, bitrate i częstotliwości</li>
          <li>Formaty: MP3, FLAC 24bit, AAC, Vorbis, Opus</li>
          <li>I2S PCM5102A: DOUT=GPIO13, BCLK=GPIO12, LRC=GPIO14</li>
          <li>I2S TAS5805 opcjonalnie: DOUT=GPIO16, BCLK=GPIO14, LRC=GPIO15</li>
          <li>TAS5805 I2C: CLK=GPIO9, DATA=GPIO8</li>
        </ul>
      </div>

      <div class="card">
        <h3>2. OLED 256×64 SPI</h3>
        <ul>
          <li>8 trybów wyświetlania</li>
          <li>Auto-dimmer i power save</li>
          <li>MOSI=GPIO39, MISO=GPIO0, SCK=GPIO38</li>
          <li>CS=GPIO42, DC=GPIO40, RESET=GPIO41</li>
        </ul>
      </div>

      <div class="card">
        <h3>3. Enkoder główny</h3>
        <ul>
          <li>Regulacja głośności</li>
          <li>Zmiana stacji</li>
          <li>Nawigacja banku</li>
          <li>Power ON/OFF</li>
          <li>CLK=GPIO10, DT=GPIO11, SW=GPIO1</li>
        </ul>
      </div>

      <div class="card">
        <h3>4. Enkoder dodatkowy</h3>
        <ul>
          <li>Aktywny po <code>#define twoEncoders</code></li>
          <li>Obsługa Volume, Mute, PowerOFF</li>
          <li>CLK=GPIO6, DT=GPIO5, SW=GPIO4</li>
        </ul>
      </div>

      <div class="card">
        <h3>5. Pilot IR NEC</h3>
        <ul>
          <li>Pełna obsługa radia, EQ, SDPlayer, BT i nagrywania</li>
          <li>Obsługa wielocyfrowego wpisywania numerów z timeoutem 3 s</li>
          <li>Pin odbiornika IR: GPIO15</li>
        </ul>
      </div>

      <div class="card">
        <h3>6. Karta SD</h3>
        <ul>
          <li>Konfiguracja, banki stacji, nagrania, pliki audio</li>
          <li>CS=GPIO47, SCLK=GPIO45, MISO=GPIO21, MOSI=GPIO48</li>
        </ul>
      </div>

      <div class="card">
        <h3>7. SDPlayer</h3>
        <ul>
          <li>Odtwarzanie MP3 i FLAC z katalogu <code>/music/</code></li>
          <li>14 stylów wyświetlania na OLED</li>
          <li>Przewijanie ±10 s, poprzedni/następny utwór, pauza, stop</li>
          <li>Powrót do radia z zachowaniem banku i stacji</li>
          <li>Sterowanie przez WebUI i pilot</li>
        </ul>
      </div>

      <div class="card">
        <h3>8. SDRecorder</h3>
        <ul>
          <li>Nagrywanie strumienia radiowego do MP3</li>
          <li>Zapis do katalogu <code>/recordings/</code></li>
          <li>START/STOP przez pilot lub WebUI</li>
        </ul>
      </div>

      <div class="card">
        <h3>9. Equalizer EQ3</h3>
        <ul>
          <li>Regulacja basu, środka i wysokich tonów</li>
          <li>Dodatkowo balans L/R</li>
          <li>Obsługa: enkoder, pilot, WebUI</li>
        </ul>
      </div>

      <div class="card">
        <h3>10. Equalizer EQ16</h3>
        <ul>
          <li>Opcjonalny, aktywowany przez <code>ENABLE_EQ16=1</code></li>
          <li>Przełączanie EQ3 ↔ EQ16 przez pilot lub WebUI</li>
          <li>Programowy DSP bez dodatkowych pinów</li>
        </ul>
      </div>

      <div class="card">
        <h3>11. Analizator FFT</h3>
        <ul>
          <li>Wyświetlanie spektrum na OLED</li>
          <li>Style: 0, 4, 5, 6, 10, 11</li>
          <li>Toggle ON/OFF przez klawisz YELLOW</li>
        </ul>
      </div>

      <div class="card">
        <h3>12. VU Meter</h3>
        <ul>
          <li>Peak hold</li>
          <li>Oddzielne kanały L i R</li>
          <li>Dostępny w wybranych trybach OLED</li>
        </ul>
      </div>

      <div class="card">
        <h3>13. WiFi + WebUI</h3>
        <ul>
          <li>Panel sterowania przez przeglądarkę</li>
          <li>OTA przez sieć</li>
          <li>mDNS: <code>http://evoradio.local</code></li>
          <li>WiFiManager przy pierwszym uruchomieniu</li>
        </ul>
      </div>

      <div class="card">
        <h3>14. DLNA</h3>
        <ul>
          <li>Opcjonalny przez <code>#define USE_DLNA</code></li>
          <li>Przeglądanie i odtwarzanie mediów z serwerów DLNA</li>
          <li>SSDP discovery i HTTP streaming</li>
        </ul>
      </div>

      <div class="card">
        <h3>15. Moduł Bluetooth UART</h3>
        <ul>
          <li>Sterowanie przez WebUI pod <code>/bt</code></li>
          <li>Obsługa z pilota IR</li>
          <li>Wykorzystuje UART ESP32</li>
        </ul>
      </div>

      <div class="card">
        <h3>16. Power Management</h3>
        <ul>
          <li>Power ON/OFF z animacją</li>
          <li>Light sleep z wakeup</li>
          <li>Sleep timer 0–90 min co 15 min</li>
          <li>Głosowe odtwarzanie czasu co godzinę</li>
          <li>SW_POWER=GPIO8, STANDBY_LED=GPIO17, SPEAKERS_PIN=GPIO18</li>
        </ul>
      </div>

      <div class="card">
        <h3>17. Zegar NTP</h3>
        <ul>
          <li>Wyświetlanie czasu na OLED</li>
          <li>Głosowe odtwarzanie czasu</li>
          <li>Tryb zegara podczas sleep</li>
        </ul>
      </div>

      <div class="card">
        <h3>18. WiFi Animation</h3>
        <ul>
          <li>Animacja gwiazdek podczas łączenia z WiFi</li>
        </ul>
      </div>
    </div>

    <div class="section">
      <h2>2. Podsumowanie pinów</h2>
      <table>
        <thead>
          <tr>
            <th>Funkcja</th>
            <th>GPIO</th>
          </tr>
        </thead>
        <tbody>
          <tr><td>I2S DOUT (PCM5102A)</td><td>13</td></tr>
          <tr><td>I2S BCLK</td><td>12</td></tr>
          <tr><td>I2S LRC</td><td>14</td></tr>
          <tr><td>OLED MOSI</td><td>39</td></tr>
          <tr><td>OLED SCK</td><td>38</td></tr>
          <tr><td>OLED CS</td><td>42</td></tr>
          <tr><td>OLED DC</td><td>40</td></tr>
          <tr><td>OLED RESET</td><td>41</td></tr>
          <tr><td>SD CS</td><td>47</td></tr>
          <tr><td>SD SCLK</td><td>45</td></tr>
          <tr><td>SD MISO</td><td>21</td></tr>
          <tr><td>SD MOSI</td><td>48</td></tr>
          <tr><td>Enkoder2 CLK</td><td>10</td></tr>
          <tr><td>Enkoder2 DT</td><td>11</td></tr>
          <tr><td>Enkoder2 SW</td><td>1</td></tr>
          <tr><td>Enkoder1 CLK</td><td>6</td></tr>
          <tr><td>Enkoder1 DT</td><td>5</td></tr>
          <tr><td>Enkoder1 SW</td><td>4</td></tr>
          <tr><td>IR odbiornik</td><td>15</td></tr>
          <tr><td>Przycisk POWER</td><td>8</td></tr>
          <tr><td>LED Standby/IR</td><td>17</td></tr>
          <tr><td>SPEAKERS enable</td><td>18</td></tr>
        </tbody>
      </table>
    </div>

    <div class="section">
      <h2>3. Obsługa pilota IR – tabela funkcji</h2>
      <table>
        <thead>
          <tr>
            <th>Kategoria</th>
            <th>Klawisz</th>
            <th>Kod NEC</th>
            <th>Funkcja</th>
          </tr>
        </thead>
        <tbody>
          <tr><td>Sterowanie dźwiękiem</td><td>VOL+</td><td>0xB914</td><td>Głośność +</td></tr>
          <tr><td>Sterowanie dźwiękiem</td><td>VOL-</td><td>0xB915</td><td>Głośność -</td></tr>
          <tr><td>Sterowanie dźwiękiem</td><td>MUTE</td><td>0xB916</td><td>Wyciszenie / przywrócenie dźwięku</td></tr>

          <tr><td>Nawigacja</td><td>→</td><td>0xB90B</td><td>Radio: następna stacja / Menu Bank: bank+1 / Equalizer: parametr+</td></tr>
          <tr><td>Nawigacja</td><td>←</td><td>0xB90A</td><td>Radio: poprzednia stacja / Menu Bank: bank-1 / Equalizer: parametr-</td></tr>
          <tr><td>Nawigacja</td><td>↑</td><td>0xB987</td><td>Radio/SDPlayer: lista stacji lub utworów – krok w górę / Equalizer: poprzedni parametr</td></tr>
          <tr><td>Nawigacja</td><td>↓</td><td>0xB986</td><td>Radio/SDPlayer: lista stacji lub utworów – krok w dół / Equalizer: następny parametr</td></tr>
          <tr><td>Nawigacja</td><td>OK / ENT</td><td>0xB90E</td><td>Zatwierdzenie wyboru stacji, banku, equalizera lub URL</td></tr>
          <tr><td>Nawigacja</td><td>BACK</td><td>0xB985</td><td>Powrót do głównego ekranu / zatrzymanie nagrywania jeśli aktywne</td></tr>
          <tr><td>Nawigacja</td><td>BANK-</td><td>0xB90C</td><td>Otwiera menu banków / w menu bank-1 z zawijaniem</td></tr>
          <tr><td>Nawigacja</td><td>BANK+</td><td>0xB90D</td><td>Otwiera menu banków / w menu bank+1 z zawijaniem</td></tr>

          <tr><td>Cyfry</td><td>0</td><td>0xB900</td><td>EQ: przełączanie EQ3 ↔ EQ16 / Radio i SDPlayer: wybór stacji lub utworu</td></tr>
          <tr><td>Cyfry</td><td>1</td><td>0xB901</td><td>Stacja lub utwór nr 1 albo pierwsza cyfra liczby</td></tr>
          <tr><td>Cyfry</td><td>2</td><td>0xB902</td><td>Stacja lub utwór nr 2 albo pierwsza cyfra liczby</td></tr>
          <tr><td>Cyfry</td><td>3</td><td>0xB903</td><td>Stacja lub utwór nr 3 albo pierwsza cyfra liczby</td></tr>
          <tr><td>Cyfry</td><td>4</td><td>0xB904</td><td>Stacja lub utwór nr 4 albo pierwsza cyfra liczby</td></tr>
          <tr><td>Cyfry</td><td>5</td><td>0xB905</td><td>Stacja lub utwór nr 5 albo pierwsza cyfra liczby</td></tr>
          <tr><td>Cyfry</td><td>6</td><td>0xB906</td><td>Stacja lub utwór nr 6 albo pierwsza cyfra liczby</td></tr>
          <tr><td>Cyfry</td><td>7</td><td>0xB907</td><td>Stacja lub utwór nr 7 albo pierwsza cyfra liczby</td></tr>
          <tr><td>Cyfry</td><td>8</td><td>0xB908</td><td>Stacja lub utwór nr 8 albo pierwsza cyfra liczby</td></tr>
          <tr><td>Cyfry</td><td>9</td><td>0xB909</td><td>Stacja lub utwór nr 9 albo pierwsza cyfra liczby</td></tr>

          <tr><td>Tryby i equalizer</td><td>SRC</td><td>0xB913</td><td>Zmiana trybu wyświetlacza OLED 0–8</td></tr>
          <tr><td>Tryby i equalizer</td><td>AUD</td><td>0xB917</td><td>Equalizer 3-band/16-band / podwójne kliknięcie 600 ms: aktywacja SDPlayer</td></tr>
          <tr><td>Tryby i equalizer</td><td>PIP</td><td>0xB9A8</td><td>Przełączenie EQ3 ↔ EQ16</td></tr>
          <tr><td>Tryby i equalizer</td><td>DIRECT</td><td>0xB90F</td><td>Radio: nagrywanie toggle / Menu Bank: GitHub-SD / EQ: reset / Mode4: debug audio buffer / inne: dimmer OLED</td></tr>
          <tr><td>Tryby i equalizer</td><td>CH+</td><td>0xB9A3</td><td>Zwiększ jasność OLED o 20</td></tr>
          <tr><td>Tryby i equalizer</td><td>CH-</td><td>0xB9A4</td><td>Zmniejsz jasność OLED o 20</td></tr>

          <tr><td>Power i sleep</td><td>RED / POWER</td><td>0xB988</td><td>Power OFF z animacją</td></tr>
          <tr><td>Power i sleep</td><td>GREEN</td><td>0xB992</td><td>W liście stacji: głosowy czas / poza listą: sleep timer 0–90 min</td></tr>

          <tr><td>Analyzer i recording</td><td>YELLOW</td><td>0xB993</td><td>Toggle Analyzer ON/OFF</td></tr>
          <tr><td>Analyzer i recording</td><td>BLUE / REC</td><td>0xB994</td><td>Nagrywanie strumienia radiowego START/STOP do <code>/recordings/</code></td></tr>

          <tr><td>SDPlayer</td><td>HELP</td><td>0xB9A7</td><td>Toggle SDPlayer ON/OFF</td></tr>
          <tr><td>SDPlayer</td><td>PLAY</td><td>0xB996</td><td>Start odtwarzania</td></tr>
          <tr><td>SDPlayer</td><td>STOP</td><td>0xB997</td><td>Zatrzymanie odtwarzania</td></tr>
          <tr><td>SDPlayer</td><td>PAUSE / WELEMENT</td><td>0xB9A0 / 0xB9AB</td><td>Pauza / wznowienie</td></tr>
          <tr><td>SDPlayer</td><td>REV</td><td>0xB998</td><td>Przewiń -10 s / podwójne kliknięcie: poprzedni utwór</td></tr>
          <tr><td>SDPlayer</td><td>RER</td><td>0xB999</td><td>Przewiń +10 s / podwójne kliknięcie: następny utwór</td></tr>
          <tr><td>SDPlayer</td><td>RADIO</td><td>0xB99A</td><td>Wyjście i powrót do trybu Radio</td></tr>
          <tr><td>SDPlayer</td><td>EXIT</td><td>0xB99D</td><td>Wyjście i powrót do radia</td></tr>
          <tr><td>SDPlayer</td><td>FILES / MENU</td><td>0xB99B / 0xB99C</td><td>Wyświetlenie listy plików i folderów <code>/music/</code></td></tr>
          <tr><td>SDPlayer</td><td>MINIMA</td><td>0xB9A9</td><td>Następny styl wyświetlania 1–14</td></tr>
          <tr><td>SDPlayer</td><td>MAXIMA</td><td>0xB9AA</td><td>Zmiana stylu wyświetlania</td></tr>
          <tr><td>SDPlayer</td><td>WINPOD</td><td>0xB9AC</td><td>STOP i reset pozycji</td></tr>
          <tr><td>SDPlayer / BT</td><td>BT</td><td>0xB995</td><td>W Radio: otwiera stronę BT <code>/bt</code> / w SDPlayer: zmiana stylu</td></tr>

          <tr><td>Bluetooth UART</td><td>KOL</td><td>0xB99E</td><td>Włącz / wyłącz moduł BT</td></tr>
          <tr><td>Bluetooth UART</td><td>CHAT</td><td>0xB99F</td><td>Wyświetl status modułu na OLED</td></tr>
          <tr><td>Bluetooth UART</td><td>REDLEFT</td><td>0xB9A1</td><td>Restart modułu BT, funkcja eksperymentalna</td></tr>
          <tr><td>Bluetooth UART</td><td>GREENL</td><td>0xB9A2</td><td>Test połączenia modułu BT</td></tr>
          <tr><td>Bluetooth UART</td><td>INFO</td><td>0xB9A5</td><td>Informacje o module i konfiguracji <code>/bt</code></td></tr>
          <tr><td>Bluetooth UART</td><td>POWROT / GLOS</td><td>0xB9A6 / 0xB9AD</td><td>Otwiera stronę BT w przeglądarce</td></tr>

          <tr><td>Zarezerwowane</td><td>TV</td><td>0x0000</td><td>Nieprzypisane</td></tr>
          <tr><td>Zarezerwowane</td><td>WWW</td><td>0x0000</td><td>Nieprzypisane</td></tr>
          <tr><td>Zarezerwowane</td><td>GAZE</td><td>0x0000</td><td>Nieprzypisane</td></tr>
          <tr><td>Zarezerwowane</td><td>EPG</td><td>0x0000</td><td>Nieprzypisane</td></tr>
        </tbody>
      </table>
    </div>

    <div class="section">
      <h2>4. Uwagi</h2>
      <div class="note">
        Domyślne kody ustawiono dla pilota <strong>Kenwood RC-406</strong>. Kody można zmieniać przez WebUI lub plik konfiguracyjny na karcie SD / LittleFS. Pilot pracuje w standardzie <strong>NEC</strong>.
      </div>
    </div>

    <div class="footer">
      Plik HTML przygotowany jako czytelna dokumentacja projektu EVO Internet Radio v3.20.04 MOD.
    </div>
  </div>
</body>
</html>




<img width="707" height="471" alt="image" src="https://github.com/user-attachments/assets/47c10274-736e-4965-b44b-cd4f00ef4ac4" />

<img width="404" height="882" alt="image" src="https://github.com/user-attachments/assets/ba32c81e-c158-44a5-8a67-63832afaf8b6" />
<img width="964" height="895" alt="image" src="https://github.com/user-attachments/assets/ec7b0bbd-67bd-4fc4-b4a2-3b69af4a2b00" />

<img width="641" height="917" alt="image" src="https://github.com/user-attachments/assets/66926200-e75e-4483-94e8-d0a83a2f8096" />
<img width="953" height="912" alt="image" src="https://github.com/user-attachments/assets/c345bdc0-6391-47ce-903f-0a00db808e79" />
<img width="932" height="929" alt="image" src="https://github.com/user-attachments/assets/d7890e92-357c-4563-a34c-45104ac45705" />
<img width="978" height="864" alt="image" src="https://github.com/user-attachments/assets/7092c7ec-bf16-4679-a03b-fab3cebe5171" />
<img width="710" height="921" alt="image" src="https://github.com/user-attachments/assets/fc1438bb-a9bf-4304-8ee0-6ef203ff936d" />







