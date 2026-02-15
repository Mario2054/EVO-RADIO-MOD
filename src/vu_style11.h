#pragma once
/*
  vu_style11.h

  EVO3 - Styl 11: "EVO3 Radio" - Professional Stereo VU Meter
  
  Podwójny stereo VU meter w stylu profesjonalnego sprzętu audio
  z pokrętłami, skalą dB i elegancką ramką

  Integracja (main):
    #include "vu_style11.h"
    drawVUStyle11_Evo3_DoveAudioSpecial(u8g2, vuL, vuR, peakL, peakR);

  Wejścia:
    vuL, vuR     : 0..255 (poziomy VU dla kanałów L/R)
    peakL, peakR : 0..255 (poziomy peak hold dla kanałów L/R)
*/
#include <Arduino.h>
#include <U8g2lib.h>

void drawVUStyle11_Evo3_DoveAudioSpecial(U8G2 &u8g2,
                                         uint8_t vuL, uint8_t vuR,
                                         uint8_t peakL, uint8_t peakR);

// WiFi Animation for startup
void drawWiFiAnimation(U8G2 &u8g2, int x, int y, uint8_t strength);
