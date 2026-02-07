VOL 0..100
- VOL 100 = maks w bibliotece (A2DP 127)

Jeśli nadal za cicho:
- spróbuj BOOST 150 albo BOOST 200
- BOOST może przesterować (clip), więc zwiększaj stopniowo.

Przykład:
  VOL 100
  BOOST 200
  SAVE
Te same linie podajesz równolegle do WROOM-32D (to nie psuje pracy DAC):

S3 GPIO12 (BCLK) → WROOM PIN_I2S_BCLK (domyślnie GPIO26)
S3 GPIO14 (WS) → WROOM PIN_I2S_WS (domyślnie GPIO25)
S3 GPIO13 (DATA) → WROOM PIN_I2S_DIN (domyślnie GPIO22)
GND wspólne

UART sterujący (S3 ↔ WROOM)
S3 TX → WROOM RX (domyślnie GPIO16)
S3 RX ← WROOM TX (domyślnie GPIO17)
GND wspólne

Wpięcie do Twojego main.cpp (radio S3)