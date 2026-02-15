#ifndef WIFI_ANIMATION_H
#define WIFI_ANIMATION_H

#include <U8g2lib.h>

// Funkcja animacji gwiaździstego nieba podczas łączenia WiFi
void wifiStarsAnimation(U8G2 *display, int duration_ms = 2000);

// Funkcja animacji trwającej do momentu połączenia z WiFi
void wifiStarsAnimationUntilConnected(U8G2 *display, unsigned long maxDuration_ms = 180000);

// Funkcja pomocnicza - pojedyncza klatka animacji
void drawStarField(U8G2 *display, int frameNumber);

#endif // WIFI_ANIMATION_H
