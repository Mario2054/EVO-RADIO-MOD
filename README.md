Projekt oparty na orginalnym oprogramowaniu https://github.com/dzikakuna/ESP32_radio_evo3/tree/main .
Dodaje modyfikacje do projektu EVO3 


FFT ANALYZER & GRAPHIC EQUALIZER - WŁĄCZANIE/WYŁĄCZANIE
========================================================================
ENABLE_FFT_ANALYZER - Analizator widma FFT (VU meter style 5-10)
  1 = WŁĄCZONY  - Wyświetlanie spectrum audio
  0 = WYŁĄCZONY - Tylko style VU meter 0-4
  Koszt: ~25KB RAM, ~20% CPU Core1
	; 

ENABLE_EQ16 - Graficzny equalizer 16-pasmowy (modyfikacja audio)
  1 = WŁĄCZONY  - Możliwość kształtowania dźwięku przez 16 pasm
  0 = WYŁĄCZONY - Tylko 3-punktowy equalizer (bass/mid/treble)
  Koszt: ~15KB RAM, ~10% CPU podczas przetwarzania
	; 

ZALECANE USTAWIENIA:
  Tylko widmo:     FFT=1, EQ16=0  (oszczędność CPU na EQ)
  Pełna kontrola:  FFT=1, EQ16=1  (wszystko włączone)
  Minimum:         FFT=0, EQ16=0  (maksymalna wydajność)

Tak ma ygladać bibioteka audio w arduino <img width="840" height="412" alt="image" src="https://github.com/user-attachments/assets/5c69bd33-fc60-4166-a67a-1ea5694cad90" />


<img width="380" height="880" alt="image" src="https://github.com/user-attachments/assets/09441cda-3d18-46ff-8f9d-7c14e1d5918f" />

<img width="1232" height="855" alt="image" src="https://github.com/user-attachments/assets/128e40ed-1cab-4c19-a214-c0fb54191b5b" />
<img width="968" height="903" alt="image" src="https://github.com/user-attachments/assets/3fbbfefd-649d-4ffd-8b1f-a79b0e26d938" />

