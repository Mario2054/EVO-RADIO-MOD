#include "vu_style11.h"
#include "EQ_AnalyzerDisplay.h"
#include <math.h>
#include <string.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD (PI/180.0)
#endif

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// =============== ANIMACJA WiFi ===============
void drawWiFiAnimation(U8G2 &u8g2, int x, int y, uint8_t strength) {
    u8g2.setDrawColor(1);
    
    // WiFi sygnał animowany
    int wavePhase = (int)(millis() * 0.012) % 360;
    
    // 4 poziomy sygnału WiFi z animacją
    for (int i = 0; i < 4; i++) {
        int level = i + 1;
        int wave = (int)(sin((wavePhase + i * 60) * DEG_TO_RAD) * 2);
        
        // Wysokość pasków wzrasta + efekt fali
        int barHeight = level * 3 + wave;
        if (barHeight < 2) barHeight = 2;
        
        int barX = x + i * 5;
        
        // Paski WiFi z miganiem podczas łączenia
        if (strength == 0) {  // Brak połączenia - miganie
            if ((millis() / 200 + i) % 4 < 2) {  // miganie co 200ms
                u8g2.drawVLine(barX, y - barHeight, barHeight);
                u8g2.drawVLine(barX + 1, y - barHeight, barHeight);
            }
        } else {
            // Połączony - stałe paski
            u8g2.drawVLine(barX, y - barHeight, barHeight);
            u8g2.drawVLine(barX + 1, y - barHeight, barHeight);
        }
        
        // Kropka na górze każdego paska
        if (strength > 0 || (millis() / 150 + i) % 3 == 0) {
            u8g2.drawPixel(barX, y - barHeight - 1);
        }
    }
    
    // Tekst pod sygnałem
    u8g2.setFont(u8g2_font_5x7_tf);
    if (strength == 0) {
        // Conectando... animowany tekst
        int dots = ((millis() / 500) % 4);
        char connecting[16] = "Connecting";
        for (int i = 0; i < dots; i++) {
            strcat(connecting, ".");
        }
        u8g2.drawStr(x - 15, y + 12, connecting);
    } else {
        u8g2.drawStr(x - 8, y + 12, "WiFi OK");
    }
    
    // Dodatkowe efekty podczas łączenia
    if (strength == 0) {
        // Wirujący wskaźnik łączenia
        int spinPhase = (millis() / 100) % 8;
        int spinX = x + 25;
        int spinY = y - 8;
        
        switch(spinPhase) {
            case 0: u8g2.drawStr(spinX, spinY, "|"); break;
            case 1: u8g2.drawStr(spinX, spinY, "/"); break;
            case 2: u8g2.drawStr(spinX, spinY, "-"); break;
            case 3: u8g2.drawStr(spinX, spinY, "\\"); break;
            case 4: u8g2.drawStr(spinX, spinY, "|"); break;
            case 5: u8g2.drawStr(spinX, spinY, "/"); break;
            case 6: u8g2.drawStr(spinX, spinY, "-"); break;
            case 7: u8g2.drawStr(spinX, spinY, "\\"); break;
        }
    }
}
void drawIlluminatedSky(U8G2 &u8g2, uint8_t musicLevel) {
    u8g2.setDrawColor(1);
    
    // Intensywność nieba bazuje na muzyce
    float intensity = (musicLevel / 255.0f);
    
    // === GWIAZDY MIGAJĄCE ===
    // Stałe pozycje gwiazd
    int stars[][2] = {{20,8}, {45,5}, {80,7}, {120,6}, {160,8}, {200,5}, {230,7}, {10,12}, {60,10}, {140,9}, {180,11}, {220,10}};
    
    for (int i = 0; i < 12; i++) {
        int x = stars[i][0];
        int y = stars[i][1];
        
        // Miganie z różnymi fazami
        float twinkle = sin((millis() * 0.005) + (i * 50)) * 0.5 + 0.5;
        
        if (twinkle > 0.3) {  // gwiazda świeci
            u8g2.drawPixel(x, y);
            
            // Większe gwiazdy przy głośnej muzyce
            if (intensity > 0.5 && twinkle > 0.7) {
                u8g2.drawPixel(x-1, y);
                u8g2.drawPixel(x+1, y);
                u8g2.drawPixel(x, y-1);
                u8g2.drawPixel(x, y+1);
            }
            
            // Bardzo jasne gwiazdy przy bardzo głośnej muzyce
            if (intensity > 0.8 && twinkle > 0.9) {
                u8g2.drawPixel(x-1, y-1);
                u8g2.drawPixel(x+1, y-1);
                u8g2.drawPixel(x-1, y+1);
                u8g2.drawPixel(x+1, y+1);
            }
        }
    }
    
    // === BŁYSKI ŚWIETLNE ===
    if (intensity > 0.6) {
        // Dyskretne błyski w różnych miejscach
        int flash1 = (int)(sin(millis() * 0.008) * 127 + 128);
        int flash2 = (int)(sin(millis() * 0.012 + 100) * 127 + 128);
        int flash3 = (int)(sin(millis() * 0.006 + 200) * 127 + 128);
        
        if (flash1 > 200) {
            // Błysk po lewej
            for (int i = 0; i < 8; i++) {
                u8g2.drawPixel(30 + i, 3);
                u8g2.drawPixel(30 + i, 4);
            }
        }
        
        if (flash2 > 210) {
            // Błysk po środku
            for (int i = 0; i < 6; i++) {
                u8g2.drawPixel(125 + i, 2);
                u8g2.drawPixel(125 + i, 3);
            }
        }
        
        if (flash3 > 220) {
            // Błysk po prawej
            for (int i = 0; i < 10; i++) {
                u8g2.drawPixel(200 + i, 4);
                u8g2.drawPixel(200 + i, 5);
            }
        }
    }
    
    // === PROMIENIE SŁOŃCA/KSIĘŻYCA ===
    if (intensity > 0.4) {
        int sunX = 240;  // słońce/księżyc w prawym górnym rogu
        int sunY = 8;
        
        // Środek słońca/księżyca
        u8g2.drawDisc(sunX, sunY, 3, U8G2_DRAW_ALL);
        
        // Promienie wokół
        float sunPhase = millis() * 0.005;
        for (int i = 0; i < 8; i++) {
            float angle = (i * 45 + sunPhase) * DEG_TO_RAD;
            int rayLength = 6 + (int)(intensity * 4);
            
            int x1 = sunX + (int)(cos(angle) * 4);
            int y1 = sunY + (int)(sin(angle) * 4);
            int x2 = sunX + (int)(cos(angle) * rayLength);
            int y2 = sunY + (int)(sin(angle) * rayLength);
            
            u8g2.drawLine(x1, y1, x2, y2);
        }
    }
    
    // === CHMURY W RUCHU ===
    int cloudX1 = ((millis() / 100) % 300) - 50;    // wolno płynąca chmura
    int cloudX2 = ((millis() / 150) % 320) - 60;    // druga chmura
    
    // Chmura 1
    if (cloudX1 > -30 && cloudX1 < 256) {
        u8g2.drawPixel(cloudX1, 15);
        u8g2.drawPixel(cloudX1 + 1, 14);
        u8g2.drawPixel(cloudX1 + 2, 15);
        u8g2.drawPixel(cloudX1 - 1, 15);
    }
    
    // Chmura 2  
    if (cloudX2 > -40 && cloudX2 < 256) {
        u8g2.drawPixel(cloudX2, 12);
        u8g2.drawPixel(cloudX2 + 1, 11);
        u8g2.drawPixel(cloudX2 + 2, 12);
        u8g2.drawPixel(cloudX2 + 3, 12);
        u8g2.drawPixel(cloudX2 - 1, 12);
    }
}

// =============== MAGICZNE OKA - MAGIC EYE TUBES ===============
void drawMagicEye(U8G2 &u8g2, int centerX, int centerY, uint8_t vuLevel, int phase) {
    u8g2.setDrawColor(1);
    
    // Konwersja poziomu VU do kąta zamknięcia oka (0-180 stopni)
    float vuFloat = vuLevel / 255.0f;
    int eyeAngle = (int)(vuFloat * 160 + 10); // 10-170 stopni
    
    // Efekt migotania lampy (dodaje realizmu)
    int flicker = (int)(sin(millis() * 0.05 + phase) * 2);
    eyeAngle += flicker;
    
    // OBUDOWA LAMPY
    int radius = 18;
    u8g2.drawCircle(centerX, centerY, radius, U8G2_DRAW_ALL);
    u8g2.drawCircle(centerX, centerY, radius - 1, U8G2_DRAW_ALL);
    
    // MAGIC EYE - wachlarz świetlny
    for (int angle = 10; angle < eyeAngle; angle += 3) {
        float radAngle = (angle - 90) * DEG_TO_RAD; // -90 żeby zaczynać od góry
        
        // Różne długości promieni dla efektu
        int rayLength = radius - 3;
        if (angle % 9 == 0) rayLength -= 2;  // co 3 promień krótszy
        
        int x1 = centerX + (int)(cos(radAngle) * 3);
        int y1 = centerY + (int)(sin(radAngle) * 3);
        int x2 = centerX + (int)(cos(radAngle) * rayLength);
        int y2 = centerY + (int)(sin(radAngle) * rayLength);
        
        u8g2.drawLine(x1, y1, x2, y2);
    }
    
    // ŚRODKOWY PUNKT (filament lampy)
    u8g2.drawDisc(centerX, centerY, 2, U8G2_DRAW_ALL);
    
    // ŚWIECĄCY EFEKT gdy wysoki poziom
    if (vuLevel > 200) {
        // Dodatkowy krąg świetlny
        u8g2.drawCircle(centerX, centerY, radius + 2, U8G2_DRAW_ALL);
        
        // Promienie na zewnątrz
        for (int i = 0; i < 8; i++) {
            float rayAngle = (i * 45) * DEG_TO_RAD;
            int x1 = centerX + (int)(cos(rayAngle) * (radius + 3));
            int y1 = centerY + (int)(sin(rayAngle) * (radius + 3));
            int x2 = centerX + (int)(cos(rayAngle) * (radius + 6));
            int y2 = centerY + (int)(sin(rayAngle) * (radius + 6));
            u8g2.drawLine(x1, y1, x2, y2);
        }
    }
    
    // EFEKT FOSFORESCENCJI - migające punkty wokół
    if (vuLevel > 100) {
        for (int i = 0; i < 6; i++) {
            int sparklePhase = (millis() / 100 + phase + i * 30) % 360;
            if (sparklePhase < 60) {  // 1/6 czasu świeci
                float sparkleAngle = (i * 60) * DEG_TO_RAD;
                int sx = centerX + (int)(cos(sparkleAngle) * (radius + 8));
                int sy = centerY + (int)(sin(sparkleAngle) * (radius + 8));
                u8g2.drawPixel(sx, sy);
            }
        }
    }
}

// =============== ANIMACJA LAMP EL34/KT88 ===============
void drawTubeGlow(U8G2 &u8g2, int x, int y, uint8_t intensity) {
    u8g2.setDrawColor(1);
    
    // Intensywność bazuje na muzyce
    int glowHeight = (int)(intensity / 255.0f * 15) + 3;
    
    // Podstawa lampy
    u8g2.drawBox(x - 3, y + 8, 7, 4);
    
    // Korpus lampy
    u8g2.drawFrame(x - 2, y - 5, 5, 13);
    
    // Świecówka wewnątrz
    if (intensity > 50) {
        for (int i = 0; i < glowHeight; i++) {
            if ((millis() / 50 + i) % 4 < 2) {  // migotanie
                u8g2.drawPixel(x, y + 7 - i);
                if (i % 2 == 0) {
                    u8g2.drawPixel(x - 1, y + 7 - i);
                    u8g2.drawPixel(x + 1, y + 7 - i);
                }
            }
        }
    }
    
    // Anoda (górna część)
    u8g2.drawBox(x - 1, y - 7, 3, 2);
}

// =============== RYSOWANIE POSTACI KOBIETY ===============
void drawWoman(U8G2 &u8g2, int x, int y) {
    u8g2.setDrawColor(1);
    
    // RUDE DŁUGIE WŁOSY - bardziej realistyczne
    // Włosy po bokach
    for (int i = -8; i <= 8; i += 1) {
        int wave = (int)(sin((i + x*0.1) * 0.4) * 2);  // falowanie
        u8g2.drawLine(x + i, y - 16, x + i, y - 7 + wave);
        // Dodatkowe włosy dla "rudości" - więcej płatków
        if (i % 2 == 0) {
            u8g2.drawPixel(x + i - 1, y - 15 + wave);
            u8g2.drawPixel(x + i + 1, y - 14 + wave);
        }
    }
    
    // Włosy z tyłu - dłuższe
    for (int i = -6; i <= 6; i += 2) {
        int wave = (int)(sin((i + x*0.08) * 0.3) * 1.5);
        u8g2.drawLine(x + i, y - 15, x + i, y - 3 + wave);
    }
    
    // GŁOWA (nieco większa)
    u8g2.drawCircle(x, y - 10, 4, U8G2_DRAW_ALL);
    u8g2.drawPixel(x, y - 10);  // wypełnienie środka
    
    // OCZY
    u8g2.drawPixel(x - 2, y - 11);
    u8g2.drawPixel(x + 2, y - 11);
    
    // USTA (uśmiech)
    u8g2.drawLine(x - 1, y - 8, x + 1, y - 8);
    u8g2.drawPixel(x, y - 7);
    
    // CIAŁO - bardziej kobiecy kształt
    // Szyja
    u8g2.drawLine(x, y - 6, x, y - 4);
    
    // Ramiona/tors (szerszy na wysokości biustu)
    u8g2.drawLine(x - 3, y - 3, x + 3, y - 3);  // barki
    u8g2.drawLine(x - 3, y - 3, x - 2, y + 2);   // lewa strona
    u8g2.drawLine(x + 3, y - 3, x + 2, y + 2);   // prawa strona
    u8g2.drawLine(x - 2, y + 2, x - 1, y + 6);   // talia lewa
    u8g2.drawLine(x + 2, y + 2, x + 1, y + 6);   // talia prawa
    
    // Biodra
    u8g2.drawLine(x - 1, y + 6, x - 2, y + 8);
    u8g2.drawLine(x + 1, y + 6, x + 2, y + 8);
    
    // RĘCE - RUCHY PODCZAS BIEGU
    int armOffset = (int)(sin(millis() * 0.01) * 4);
    u8g2.drawLine(x - 3, y - 1, x - 7 + armOffset, y + 3);
    u8g2.drawLine(x + 3, y - 1, x + 7 - armOffset, y + 3);
    
    // Dłonie
    u8g2.drawDisc(x - 7 + armOffset, y + 3, 1, U8G2_DRAW_ALL);
    u8g2.drawDisc(x + 7 - armOffset, y + 3, 1, U8G2_DRAW_ALL);
    
    // NOGI - ANIMACJA BIEGU
    int legOffset = (int)(sin(millis() * 0.015) * 5);
    u8g2.drawLine(x - 2, y + 8, x - 4 + legOffset, y + 18);
    u8g2.drawLine(x + 2, y + 8, x + 4 - legOffset, y + 18);
    
    // STOPY (buty)
    u8g2.drawBox(x - 6 + legOffset, y + 17, 4, 3);
    u8g2.drawBox(x + 2 - legOffset, y + 17, 4, 3);
    
    // NAPIS "EVA" za postacią
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(x + 15, y + 5, "EVA");
}

// =============== RYSOWANIE POSTACI MĘŻCZYZNY (pojedynczo) ===============
void drawMan(U8G2 &u8g2, int x, int y) {
    u8g2.setDrawColor(1);
    
    // KRÓTKIE WŁOSY
    u8g2.drawLine(x - 4, y - 14, x + 4, y - 14);
    u8g2.drawLine(x - 3, y - 15, x + 3, y - 15);
    
    // GŁOWA
    u8g2.drawCircle(x, y - 10, 4, U8G2_DRAW_ALL);
    
    // OCZY
    u8g2.drawPixel(x - 2, y - 11);
    u8g2.drawPixel(x + 2, y - 11);
    
    // WĄSY
    u8g2.drawLine(x - 3, y - 8, x + 3, y - 8);
    
    // CIAŁO (szerszy od kobiety)
    u8g2.drawBox(x - 1, y - 6, 3, 14);
    
    // RĘCE
    u8g2.drawLine(x - 1, y - 2, x - 6, y + 2);
    u8g2.drawLine(x + 1, y - 2, x + 6, y + 2);
    
    // NOGI
    u8g2.drawLine(x, y + 8, x - 3, y + 18);
    u8g2.drawLine(x, y + 8, x + 3, y + 18);
    
    // BUTY
    u8g2.drawBox(x - 5, y + 17, 4, 3);
    u8g2.drawBox(x + 1, y + 17, 4, 3);
}

// =============== RYSOWANIE TAŃCZĄCEJ PARY ===============
void drawDancingCouple(U8G2 &u8g2, int manX, int womanX, int y, uint8_t musicLevel) {
    u8g2.setDrawColor(1);
    
    // Rytm tańca bazujący na poziomie muzyki
    float beat = (musicLevel / 255.0f) * 2.0f + 0.5f; // 0.5 - 2.5 multiplier
    int danceOffset = (int)(sin(millis() * 0.008 * beat) * 6);
    int armDance = (int)(sin(millis() * 0.012 * beat) * 8);
    int bobbing = (int)(sin(millis() * 0.01 * beat) * 3); // podskakiwanie
    
    // === MĘŻCZYZNA ADAM (prowadzi taniec) ===
    // Włosy
    u8g2.drawLine(manX - 4, y - 14 + bobbing, manX + 4, y - 14 + bobbing);
    u8g2.drawLine(manX - 3, y - 15 + bobbing, manX + 3, y - 15 + bobbing);
    
    // Głowa
    u8g2.drawCircle(manX, y - 10 + bobbing, 4, U8G2_DRAW_ALL);
    
    // Oczy
    u8g2.drawPixel(manX - 2, y - 11 + bobbing);
    u8g2.drawPixel(manX + 2, y - 11 + bobbing);
    
    // Uśmiech podczas tańca
    u8g2.drawLine(manX - 2, y - 8 + bobbing, manX + 2, y - 8 + bobbing);
    
    // Ciało z ruchami tanecznymi
    u8g2.drawBox(manX - 1, y - 6 + bobbing, 3, 14);
    
    // Ręce w pozycji tanecznej
    u8g2.drawLine(manX - 1, y - 2 + bobbing, manX - 8 + armDance, y + 1 + bobbing);
    u8g2.drawLine(manX + 1, y - 2 + bobbing, manX + 6 + danceOffset, y + 2 + bobbing);
    
    // Nogi w tańcu
    u8g2.drawLine(manX, y + 8 + bobbing, manX - 4 + danceOffset, y + 18);
    u8g2.drawLine(manX, y + 8 + bobbing, manX + 4 - danceOffset, y + 18);
    
    // Buty
    u8g2.drawBox(manX - 6 + danceOffset, y + 17, 4, 3);
    u8g2.drawBox(manX + 1 - danceOffset, y + 17, 4, 3);
    
    // === KOBIETA EVA (tańczy z Adamem) ===
    // Rude włosy w ruchu
    for (int i = -6; i <= 6; i += 1) {
        int hairMove = (int)(sin((i + millis()*0.02) * 0.3) * 3) + danceOffset;
        u8g2.drawLine(womanX + i, y - 16 + bobbing, womanX + i + hairMove, y - 7 + bobbing);
        if (i % 2 == 0) {
            u8g2.drawPixel(womanX + i + hairMove - 1, y - 15 + bobbing);
        }
    }
    
    // Głowa z ruchem
    u8g2.drawCircle(womanX + danceOffset/2, y - 10 + bobbing, 4, U8G2_DRAW_ALL);
    
    // Oczy
    u8g2.drawPixel(womanX + danceOffset/2 - 2, y - 11 + bobbing);
    u8g2.drawPixel(womanX + danceOffset/2 + 2, y - 11 + bobbing);
    
    // Uśmiech
    u8g2.drawLine(womanX + danceOffset/2 - 1, y - 8 + bobbing, womanX + danceOffset/2 + 1, y - 8 + bobbing);
    
    // Ciało w tańcu - kobiecy kształt
    u8g2.drawLine(womanX + danceOffset/2, y - 6 + bobbing, womanX + danceOffset/2, y - 4 + bobbing);
    u8g2.drawLine(womanX - 3 + danceOffset/2, y - 3 + bobbing, womanX + 3 + danceOffset/2, y - 3 + bobbing);
    u8g2.drawLine(womanX - 3 + danceOffset/2, y - 3 + bobbing, womanX - 2 + danceOffset/2, y + 2 + bobbing);
    u8g2.drawLine(womanX + 3 + danceOffset/2, y - 3 + bobbing, womanX + 2 + danceOffset/2, y + 2 + bobbing);
    u8g2.drawLine(womanX - 2 + danceOffset/2, y + 2 + bobbing, womanX - 1 + danceOffset/2, y + 6 + bobbing);
    u8g2.drawLine(womanX + 2 + danceOffset/2, y + 2 + bobbing, womanX + 1 + danceOffset/2, y + 6 + bobbing);
    
    // Ręce w tanecznej pozycji (jedna na ramieniu Adama)
    u8g2.drawLine(womanX - 3 + danceOffset/2, y - 1 + bobbing, manX + 2, y - 1 + bobbing); // lewa ręka na ramię
    u8g2.drawLine(womanX + 3 + danceOffset/2, y - 1 + bobbing, womanX + 8 - armDance, y + 3 + bobbing); // prawa ręka
    
    // Nogi w tańcu
    u8g2.drawLine(womanX - 1 + danceOffset/2, y + 6 + bobbing, womanX - 4 - danceOffset, y + 18);
    u8g2.drawLine(womanX + 1 + danceOffset/2, y + 6 + bobbing, womanX + 4 + danceOffset, y + 18);
    
    // Buty EVy
    u8g2.drawBox(womanX - 6 - danceOffset, y + 17, 4, 3);
    u8g2.drawBox(womanX + 2 + danceOffset, y + 17, 4, 3);
    
    // NAPISY POD PARĄ
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(manX - 10, y + 25, "ADAM");
    u8g2.drawStr(womanX + 5, y + 25, "EVA");
    
    // SERDUSZKA NAD PARĄ (gdy muzyka jest głośna)
    if (musicLevel > 150) {
        int heartY = y - 20 + (int)(sin(millis() * 0.01) * 2);
        // Serduszko 1
        u8g2.drawPixel(manX + 8, heartY);
        u8g2.drawPixel(manX + 9, heartY - 1);
        u8g2.drawPixel(manX + 10, heartY);
        u8g2.drawPixel(manX + 9, heartY + 1);
        
        // Serduszko 2
        u8g2.drawPixel(womanX - 8, heartY + 2);
        u8g2.drawPixel(womanX - 7, heartY + 1);
        u8g2.drawPixel(womanX - 6, heartY + 2);
        u8g2.drawPixel(womanX - 7, heartY + 3);
    }
}

// =============== WSKAŹNIKI MUZYKI Z dB ===============
void drawMusicIndicators(U8G2 &u8g2, uint8_t vuL_8bit, uint8_t vuR_8bit) {
    u8g2.setDrawColor(1);
    
    // Konwersja VU do poziomów dB (0 do +5dB)
    float vuL = (vuL_8bit / 255.0f);  // 0-1 
    float vuR = (vuR_8bit / 255.0f);  // 0-1
    
    // Pozycja środkowa ekranu
    int centerX = 128;
    int baseY = 58;
    
    // WSKAŹNIK WYSTEROWANIA - ROZSUWA SIĘ OD ŚRODKA
    // Lewy kanał - rozchodzi się w lewo
    int leftBars = (int)(vuL * 60);  // max 60 pikseli w lewo
    for (int i = 0; i < leftBars; i++) {
        int height = 6 - (i / 15);  // wysokość maleje z odległością
        if (height < 2) height = 2;
        
        // Różne wysokości dla efektu "trzepotania"
        if (i % 3 == 0) height += 2;
        
        u8g2.drawVLine(centerX - 2 - i, baseY - height, height);
        
        // Co kilka pasków - większy wskaźnik
        if (i % 10 == 0 && i > 0) {
            u8g2.drawVLine(centerX - 2 - i, baseY - height - 2, height + 4);
        }
    }
    
    // Prawy kanał - rozchodzi się w prawo  
    int rightBars = (int)(vuR * 60);  // max 60 pikseli w prawo
    for (int i = 0; i < rightBars; i++) {
        int height = 6 - (i / 15);  // wysokość maleje z odległością
        if (height < 2) height = 2;
        
        // Różne wysokości dla efektu "trzepotania"
        if (i % 3 == 0) height += 2;
        
        u8g2.drawVLine(centerX + 2 + i, baseY - height, height);
        
        // Co kilka pasków - większy wskaźnik  
        if (i % 10 == 0 && i > 0) {
            u8g2.drawVLine(centerX + 2 + i, baseY - height - 2, height + 4);
        }
    }
    
    // SKALA dB - OPISY NA DOLE
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(10, 63, "0");
    u8g2.drawStr(50, 63, "+3dB");
    u8g2.drawStr(80, 63, "+5dB");
    
    u8g2.drawStr(170, 63, "+5dB");
    u8g2.drawStr(200, 63, "+3dB");
    u8g2.drawStr(240, 63, "0");
    
    // ŚRODEK - separator z napisem STEREO
    u8g2.drawVLine(128, 45, 10);
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(115, 48, "STEREO");
    
    // Znaczniki poziomów po bokach
    u8g2.drawVLine(70, baseY - 8, 2);   // +5dB lewa
    u8g2.drawVLine(100, baseY - 6, 2);  // +3dB lewa  
    u8g2.drawVLine(156, baseY - 6, 2);  // +3dB prawa
    u8g2.drawVLine(186, baseY - 8, 2);  // +5dB prawa
}

// =============== GŁÓWNA FUNKCJA ANIMACJI ===============
void drawVUStyle11_Evo3_DoveAudioSpecial(U8G2 &u8g2,
                                         uint8_t vuL_8bit, uint8_t vuR_8bit,
                                         uint8_t peakL_8bit, uint8_t peakR_8bit)
{
    u8g2.clearBuffer();
    u8g2.setDrawColor(1);
    
    // Średni poziom muzyki dla efektów
    uint8_t avgMusic = (vuL_8bit + vuR_8bit) / 2;
    
    // =============== ROZŚWIETLANE NIEBO NA GÓRZE ===============
    drawIlluminatedSky(u8g2, avgMusic);
    
    // =============== TYTUŁ W GÓRNEJ CZĘŚCI ===============
    u8g2.setFont(u8g2_font_7x13B_tr);
    int titleX = 128 - (strlen("EVO 3.19.70 MOD EJCON") * 7) / 2;  // wyśrodkowanie
    u8g2.drawStr(titleX, 25, "EVO 3.19.70 MOD EJCON");
    
    // =============== MAGICZNE OKA - STEREO VU METERS ===============
    // Lewe Magic Eye (L channel)
    drawMagicEye(u8g2, 64, 38, vuL_8bit, 0);
    
    // Prawe Magic Eye (R channel)  
    drawMagicEye(u8g2, 192, 38, vuR_8bit, 100);
    
    // OPISY kanałów
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(58, 60, "L");
    u8g2.drawStr(188, 60, "R");
    
    // =============== LAMPY ELEKTRONOWE PO BOKACH ===============
    // Lewe lampy (4 sztuki)
    drawTubeGlow(u8g2, 15, 35, vuL_8bit);
    drawTubeGlow(u8g2, 25, 37, vuL_8bit > 100 ? vuL_8bit - 50 : 0);
    drawTubeGlow(u8g2, 35, 35, vuL_8bit > 150 ? vuL_8bit - 100 : 0);
    drawTubeGlow(u8g2, 45, 37, vuL_8bit > 200 ? vuL_8bit - 150 : 0);
    
    // Prawe lampy (4 sztuki)
    drawTubeGlow(u8g2, 240, 35, vuR_8bit);
    drawTubeGlow(u8g2, 230, 37, vuR_8bit > 100 ? vuR_8bit - 50 : 0);
    drawTubeGlow(u8g2, 220, 35, vuR_8bit > 150 ? vuR_8bit - 100 : 0);
    drawTubeGlow(u8g2, 210, 37, vuR_8bit > 200 ? vuR_8bit - 150 : 0);
    
    // =============== EFEKTY SPECJALNE PRZY GŁOŚNEJ MUZYCE ===============
    if (avgMusic > 180) {
        // Błyskawice między oczami
        if ((millis() / 100) % 3 == 0) {
            u8g2.drawLine(82, 38, 174, 38);
            u8g2.drawLine(80, 39, 176, 39);
            u8g2.drawLine(84, 37, 172, 37);
        }
        
        // Pulsujące ramki wokół oczu
        int pulse = (int)(sin(millis() * 0.02) * 3);
        u8g2.drawCircle(64, 38, 22 + pulse, U8G2_DRAW_ALL);
        u8g2.drawCircle(192, 38, 22 + pulse, U8G2_DRAW_ALL);
    }
    
    // =============== WSKAŹNIKI MUZYKI NA DOLE (zawsze widoczne) ===============
    drawMusicIndicators(u8g2, vuL_8bit, vuR_8bit);
    
    u8g2.sendBuffer();
}
