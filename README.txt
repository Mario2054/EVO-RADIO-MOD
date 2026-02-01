Jak użyć

1) W PlatformIO:
   - wrzuć main.cpp do src/
   - wrzuć DisplayConfig.h i Theme.h do include/ (albo tam gdzie masz nagłówki)

2) Wybór sterownika wyświetlacza:
   - w main.cpp odkomentuj jedną linię:
       //#define USE_ILI9488
       //#define USE_ILI9341
       //#define USE_ST7796
     albo ustaw w platformio.ini:
       build_flags =
         -DUSE_ILI9488
   (Jeśli nic nie ustawisz, domyślnie wybierze USE_ILI9488)

Uwaga:
- UI w tym projekcie jest robione pod układ 480x320 (ILI9488/ST7796 w poziomie).
  Na ILI9341 (320x240 w poziomie) część elementów może wyjechać poza ekran – wtedy trzeba będzie
  skorygować pozycje / skalę UI.
