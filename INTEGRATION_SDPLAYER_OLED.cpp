/**
 * PRZYKŁAD INTEGRACJI SDPlayerOLED w MAIN.CPP
 * ============================================
 */

#include <Wire.h>
#include <U8g2lib.h>
#include "SDPlayer/SDPlayerWebUI.h"
#include "SDPlayer/SDPlayerOLED.h"

// OLED Display (SSD1306 128x64)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// SD Player
SDPlayerWebUI sdPlayer;
SDPlayerOLED sdOLED(u8g2);

// Pilot IR
#include <IRremote.h>
#define IR_PIN 15
IRrecv irrecv(IR_PIN);
decode_results results;

// Enkoder
#define ENCODER_CLK 32
#define ENCODER_DT 33
#define ENCODER_SW 34
int lastCLK = HIGH;

void setup() {
    Serial.begin(115200);
    
    // === OLED ===
    Wire.begin();
    u8g2.begin();
    
    // === SD Player WebUI ===
    // sdPlayer.begin(&server);
    
    // === SD Player OLED ===
    sdOLED.begin(&sdPlayer);
    
    // === Pilot IR ===
    irrecv.enableIRIn();
    
    // === Enkoder ===
    pinMode(ENCODER_CLK, INPUT_PULLUP);
    pinMode(ENCODER_DT, INPUT_PULLUP);
    pinMode(ENCODER_SW, INPUT_PULLUP);
}

void loop() {
    // === SD Player OLED ===
    sdOLED.loop();
    
    // === PILOT IR ===
    if (irrecv.decode(&results)) {
        handleRemote(results.value);
        irrecv.resume();
    }
    
    // === ENKODER ===
    handleEncoder();
}

// ===== OBSŁUGA PILOTA =====
void handleRemote(unsigned long code) {
    
    // Aktywacja SD Player (3x999 lub inny kod)
    if (code == 0x3E3C13EC) { // Przykładowy kod 3x999
        sdOLED.activate();
        return;
    }
    
    // Jeśli SD Player nieaktywny, ignoruj
    if (!sdOLED.isActive()) return;
    
    switch (code) {
        case 0x511DBB:  // Góra
            sdOLED.onRemoteUp();
            break;
            
        case 0x52A3D41F: // Dół
            sdOLED.onRemoteDown();
            break;
            
        case 0x20FE4DBB: // OK
            sdOLED.onRemoteOK();
            break;
            
        case 0xD7E84B1B: // SRC (zmiana stylu)
            sdOLED.onRemoteSRC();
            break;
            
        case 0x3D9AE3F7: // VOL+
            sdOLED.onRemoteVolUp();
            break;
            
        case 0x1BC0157B: // VOL-
            sdOLED.onRemoteVolDown();
            break;
            
        default:
            Serial.printf("Unknown IR: 0x%08X\n", code);
            break;
    }
}

// ===== OBSŁUGA ENKODERA =====
void handleEncoder() {
    static unsigned long lastButtonPress = 0;
    
    // Obrót
    int currentCLK = digitalRead(ENCODER_CLK);
    if (currentCLK != lastCLK && currentCLK == LOW) {
        if (digitalRead(ENCODER_DT) == HIGH) {
            sdOLED.onEncoderRight();
        } else {
            sdOLED.onEncoderLeft();
        }
    }
    lastCLK = currentCLK;
    
    // Przycisk
    if (digitalRead(ENCODER_SW) == LOW) {
        if (millis() - lastButtonPress > 300) {
            sdOLED.onEncoderButton();
            lastButtonPress = millis();
        }
    }
}

// ===== AKTYWACJA ZE STRONY WWW =====
// Wywołaj gdy użytkownik wchodzi na /sdplayer
void onWebSDPlayerOpen() {
    sdOLED.activate();
}
