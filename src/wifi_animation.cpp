#include "wifi_animation.h"
#include <WiFi.h>
#include <Arduino.h>

// Struktura dla pojedynczej gwiazdki
struct Star {
  int16_t x;
  int16_t y;
  uint8_t brightness; // 0-3 (rozmiar/jasność)
};

#define MAX_STARS 50

void drawStarField(U8G2 *display, int frameNumber) {
  static Star stars[MAX_STARS];
  static bool initialized = false;
  
  // Inicjalizacja gwiazdek przy pierwszym wywołaniu
  if (!initialized || frameNumber == 0) {
    for (int i = 0; i < MAX_STARS; i++) {
      stars[i].x = random(0, 256);
      stars[i].y = random(0, 64);
      stars[i].brightness = random(0, 4);
    }
    initialized = true;
  }
  
  display->clearBuffer();
  
  // Rysuj gwiazdki
  for (int i = 0; i < MAX_STARS; i++) {
    uint8_t brightness = stars[i].brightness;
    
    // Migotanie gwiazdek - zmiana jasności co kilka klatek
    if ((frameNumber + i) % 20 < 10) {
      brightness = (brightness + 1) % 4;
    }
    
    // Rysuj gwiazdkę w zależności od jasności
    switch(brightness) {
      case 0: // Mała kropka
        display->drawPixel(stars[i].x, stars[i].y);
        break;
      case 1: // Krzyżyk 3x3
        display->drawPixel(stars[i].x, stars[i].y);
        display->drawPixel(stars[i].x-1, stars[i].y);
        display->drawPixel(stars[i].x+1, stars[i].y);
        display->drawPixel(stars[i].x, stars[i].y-1);
        display->drawPixel(stars[i].x, stars[i].y+1);
        break;
      case 2: // Gwiazdka 5x5
        display->drawPixel(stars[i].x, stars[i].y);
        display->drawPixel(stars[i].x-1, stars[i].y);
        display->drawPixel(stars[i].x+1, stars[i].y);
        display->drawPixel(stars[i].x, stars[i].y-1);
        display->drawPixel(stars[i].x, stars[i].y+1);
        display->drawPixel(stars[i].x-2, stars[i].y);
        display->drawPixel(stars[i].x+2, stars[i].y);
        display->drawPixel(stars[i].x, stars[i].y-2);
        display->drawPixel(stars[i].x, stars[i].y+2);
        break;
      case 3: // Duża gwiazdka
        for(int dx = -2; dx <= 2; dx++) {
          for(int dy = -2; dy <= 2; dy++) {
            if (abs(dx) + abs(dy) <= 2) {
              display->drawPixel(stars[i].x + dx, stars[i].y + dy);
            }
          }
        }
        break;
    }
    
    // Powolny ruch gwiazdek w prawo (efekt przesuwania)
    if (frameNumber % 3 == 0) {
      stars[i].x++;
      if (stars[i].x > 256) {
        stars[i].x = 0;
        stars[i].y = random(0, 64);
        stars[i].brightness = random(0, 4);
      }
    }
  }
  
  // Tekst "Connecting WiFi..." na środku
  display->setFont(u8g2_font_helvR08_tr);
  const char* text = "Connecting WiFi...";
  int textWidth = display->getStrWidth(text);
  display->drawStr((256 - textWidth) / 2, 32, text);
  
  display->sendBuffer();
}

void wifiStarsAnimationUntilConnected(U8G2 *display, unsigned long maxDuration_ms) {
  unsigned long startTime = millis();
  int frame = 0;
  
  while (millis() - startTime < maxDuration_ms && WiFi.status() != WL_CONNECTED) {
    drawStarField(display, frame++);
    delay(50); // ~20 FPS
    
    // Sprawdzaj status co 100ms
    if (frame % 2 == 0) {
      if (WiFi.status() == WL_CONNECTED) {
        // Połączenie udane - pokaż komunikat
        display->clearBuffer();
        display->setFont(u8g2_font_helvR08_tr);
        const char* text = "WiFi Connected!";
        int textWidth = display->getStrWidth(text);
        display->drawStr((256 - textWidth) / 2, 32, text);
        display->sendBuffer();
        delay(1000);
        break;
      }
    }
  }
}
