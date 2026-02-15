#include "APMS_GraphicEQ16.h"
#include "Audio.h"
#include <string.h>
#include <FS.h>
#include <SD.h>

// External references for SD card access
extern fs::FS& getStorage();

namespace APMS_EQ16 {

static Audio* s_audio = nullptr;
static bool   s_featureEnabled = false;   // settings gate
static bool   s_enabled = false;          // audio processing gate
static int8_t s_gains[16] = {0};

void init(Audio* audio){
  s_audio = audio;
}

void setFeatureEnabled(bool enabled){ s_featureEnabled = enabled; }
bool isFeatureEnabled(){ return s_featureEnabled; }

void setEnabled(bool enabled){
  s_enabled = enabled;
  applyToAudio();
}

bool isEnabled(){ return s_enabled; }

void setBand(uint8_t band, int8_t gainDb){
  if(band>=BANDS) return;
  // zakres wzmocnienia: -16 .. +16 dB dla większej kontroli
  if(gainDb>16) gainDb=16;
  if(gainDb<-16) gainDb=-16;
  s_gains[band]=gainDb;
}

int8_t getBand(uint8_t band){
  if(band>=BANDS) return 0;
  return s_gains[band];
}

void getAll(int8_t* out16){
  if(!out16) return;
  for(uint8_t i=0;i<BANDS;i++) out16[i]=s_gains[i];
}

void setAll(const int8_t* in16){
  if(!in16) return;
  for(uint8_t i=0;i<BANDS;i++){
    int v=in16[i];
    if(v>16) v=16;
    if(v<-16) v=-16;
    s_gains[i]=(int8_t)v;
  }
}

void applyToAudio(){
  if(!s_audio) return;
  
  // ========================================================================
  // INTELIGENTNA KONWERSJA 16-PASM -> 3-PUNKTY (LOW/MID/HIGH)
  // ========================================================================
  // Nowa biblioteka ESP32-audioI2S używa wbudowanych filtrów IIR (setTone)
  // Konwersja uwzględnia wagę poszczególnych pasm dla lepszego efektu
  // ========================================================================
  
  if(!s_enabled) {
    // EQ wyłączony - ustaw wszystko na neutralne (0dB)
    s_audio->setTone(0, 0, 0);
    return;
  }
  
  // POPRAWIONA KONWERSJA Z WAGAMI:
  // Pasma dolne (0-4):  32Hz, 64Hz, 125Hz, 250Hz, 500Hz  -> LOW
  // Pasma środkowe (5-10): 1kHz, 2kHz, 3kHz, 4kHz, 5kHz, 6kHz -> MID
  // Pasma górne (11-15): 8kHz, 10kHz, 12kHz, 14kHz, 15kHz, 16kHz -> HIGH
  
  // Wagi pasm - pasma środkowe mają większą wagę (bardziej słyszalne)
  const float lowWeights[5]   = {1.2f, 1.5f, 1.3f, 1.0f, 0.8f}; // Podkreśl subbass
  const float midWeights[6]   = {1.0f, 1.3f, 1.5f, 1.3f, 1.0f, 0.8f}; // Wokale 2-3kHz
  const float highWeights[5]  = {1.0f, 1.2f, 1.1f, 0.9f, 0.8f}; // Łagodź szczyt
  
  float lowSum = 0.0f, midSum = 0.0f, highSum = 0.0f;
  float lowWeight = 0.0f, midWeight = 0.0f, highWeight = 0.0f;
  
  // Ważona suma pasm LOW (0-4)
  for(int i=0; i<5; i++) {
    lowSum += s_gains[i] * lowWeights[i];
    lowWeight += lowWeights[i];
  }
  
  // Ważona suma pasm MID (5-10)
  for(int i=5; i<11; i++) {
    midSum += s_gains[i] * midWeights[i-5];
    midWeight += midWeights[i-5];
  }
  
  // Ważona suma pasm HIGH (11-15)
  for(int i=11; i<16; i++) {
    highSum += s_gains[i] * highWeights[i-11];
    highWeight += highWeights[i-11];
  }
  
  // Oblicz ważone średnie i ZWIĘKSZ WZMOCNIENIE x8.0 (aby było równe z 3-punktowym EQ)
  // EQ16 po konwersji jest ~8x cichsze niż EQ3, więc mnożymy aby wyrównać poziomy
  const float GAIN_MULTIPLIER = 8.0f; // Mnożnik wzmocnienia - wyrównanie z EQ3
  const int8_t GAIN_OFFSET = 0;       // Bez offsetu - czysta korekcja wzmocnienia
  
  int8_t lowGain = (int8_t)((lowSum / lowWeight) * GAIN_MULTIPLIER) + GAIN_OFFSET;
  int8_t midGain = (int8_t)((midSum / midWeight) * GAIN_MULTIPLIER) + GAIN_OFFSET;
  int8_t highGain = (int8_t)((highSum / highWeight) * GAIN_MULTIPLIER) + GAIN_OFFSET;
  
  // Ograniczenie do zakresu [-40..+20] dB (rozszerzony dla większego zakresu regulacji)
  if(lowGain > 20) lowGain = 20;
  if(lowGain < -40) lowGain = -40;
  if(midGain > 20) midGain = 20;
  if(midGain < -40) midGain = -40;
  if(highGain > 20) highGain = 20;
  if(highGain < -40) highGain = -40;
  
  s_audio->setTone(lowGain, midGain, highGain);
  
  Serial.printf("EQ16->setTone: Low=%ddB, Mid=%ddB, High=%ddB (weighted avg)\n", 
                lowGain, midGain, highGain);
}

// --- UI drawings (low cost) ---
static void drawCentered(U8G2& u8g2, int y, const char* txt){
  int w = u8g2.getStrWidth(txt);
  int x = (256 - w) / 2;
  u8g2.drawStr(x, y, txt);
}

void drawModeSelect(U8G2& u8g2, uint8_t selectedMode){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x13_tr);
  drawCentered(u8g2, 14, "EQUALIZER");
  u8g2.setFont(u8g2_font_6x12_tr);

  const int y1=42, y2=66;
  const char* a="3-punktowy (Low/Mid/High)";
  const char* b="16-pasmowy (20Hz-20kHz)";

  if(selectedMode==0){
    u8g2.drawBox(8, y1-12, 240, 16);
    u8g2.setDrawColor(0);
    u8g2.drawStr(12, y1, a);
    u8g2.setDrawColor(1);
    u8g2.drawStr(12, y2, b);
  }else{
    u8g2.drawBox(8, y2-12, 240, 16);
    u8g2.setDrawColor(1);
    u8g2.drawStr(12, y1, a);
    u8g2.setDrawColor(0);
    u8g2.drawStr(12, y2, b);
    u8g2.setDrawColor(1);
  }

  u8g2.setFont(u8g2_font_5x8_tr);
  drawCentered(u8g2, 92, "LEWO/PRAWO wybierz, OK wejdz");
  drawCentered(u8g2, 104, "MENU = wyjscie");
  u8g2.sendBuffer();
}

void drawEditor(U8G2& u8g2, const int8_t* gains16, uint8_t selectedBand, bool showHelp){
  if(!gains16) return;

  // screen 256x128, leave top 12px for title
  const int top=12;
  const int bottom=120;
  const int mid = (top+bottom)/2;          // 0dB line
  const int height = bottom-top;
  const float scale = (height/2.0f) / 16.0f; // 16dB -> half height

  const int left=4, right=252;
  const int bandW = (right-left)/BANDS;    // 15
  const int barW = bandW-3;                // space for gap

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(2, 10, "EQ 16-pasmowy  (-16 .. +16 dB)");
  // 0dB line
  u8g2.drawHLine(0, mid, 256);

  for(uint8_t b=0;b<BANDS;b++){
    int x = left + b*bandW;
    int g = gains16[b];
    if(g>16) g=16;
    if(g<-16) g=-16;

    int h = (int)lrintf(fabsf((float)g)*scale);
    if(h<0) h=0;
    if(h>height/2) h=height/2;

    if(g>=0){
      u8g2.drawFrame(x, mid-h, barW, h);
      u8g2.drawBox(x+1, mid-h+1, barW-2, h-2);
    }else{
      u8g2.drawFrame(x, mid, barW, h);
      u8g2.drawBox(x+1, mid+1, barW-2, h-2);
    }

    if(b==selectedBand){
      u8g2.drawFrame(x-1, top, barW+2, bottom-top);
    }
  }

  // small footer: selected band + value
  u8g2.setFont(u8g2_font_5x8_tr);
  char buf[64];
  snprintf(buf, sizeof(buf), "Pasmo %u  Gain %ddB", (unsigned)(selectedBand+1), (int)gains16[selectedBand]);
  u8g2.drawStr(2, 126, buf);

  if(showHelp){
    u8g2.drawStr(150, 126, "GORA/DOL pasmo");
  }

  u8g2.sendBuffer();
}

} // namespace

// ======================= C-STYLE WRAPPER FUNCTIONS =======================
// Implementacje dla kompatybilności z main.cpp

// Global variables for menu state and UI
static bool g_menuActive = false;
static uint8_t g_selectedBand = 0;
static bool g_needsSave = false;
static uint32_t g_lastSaveTime = 0;

extern "C" {

void EQ16_init(void) {
    // Inicjalizacja namespace-a APMS_EQ16
    extern Audio audio;
    APMS_EQ16::init(&audio);
    APMS_EQ16::setFeatureEnabled(true);
    APMS_EQ16::setEnabled(true);
    
    // Ustaw domyślne wartości (wszystkie na 0dB)
    for(uint8_t i = 0; i < APMS_EQ16::BANDS; i++) {
        APMS_EQ16::setBand(i, 0);
    }
    
    Serial.println("EQ16_init() - 16-Band Equalizer initialized");
    
    // Załaduj zapisane ustawienia z SD card (jeśli istnieją)
    EQ16_loadFromSD();
}

void EQ16_enable(bool enabled) {
    APMS_EQ16::setEnabled(enabled);
}

bool EQ16_isEnabled(void) {
    return APMS_EQ16::isEnabled();
}

void EQ16_setBand(uint8_t band, int8_t gainDb) {
    APMS_EQ16::setBand(band, gainDb);
}

int8_t EQ16_getBand(uint8_t band) {
    return APMS_EQ16::getBand(band);
}

void EQ16_resetAllBands(void) {
    for(uint8_t i = 0; i < APMS_EQ16::BANDS; i++) {
        APMS_EQ16::setBand(i, 0); // Set all bands to 0dB
    }
    APMS_EQ16::applyToAudio();  // ZASTOSUJ ZMIANY DO AUDIO!
    EQ16_saveToSD();  // Automatyczny zapis po resecie
    Serial.println("EQ16: All bands reset to 0dB, applied to audio and saved to SD");
}

bool EQ16_isMenuActive(void) {
    return g_menuActive;
}

void EQ16_setMenuActive(bool active) {
    // Proste ustawienie flagi jak w menu 3-punktowym
    g_menuActive = active;
    
    // Bezpośrednie ustawienie flagi w main.cpp
    extern bool eq16MenuActive;
    eq16MenuActive = active;
    
    if (active) {
        // Proste uruchomienie na wzór displayEqualizer()
        extern unsigned long displayStartTime;
        extern bool timeDisplay;
        extern bool displayActive;
        
        displayStartTime = millis();  // Jak w menu 3-punktowym
        timeDisplay = false;          // Jak w menu 3-punktowym  
        displayActive = true;         // Jak w menu 3-punktowym
        
        Serial.println("EQ16: Menu activated (simplified)");
        EQ16_displayMenu(); // Wywołaj wyświetlanie od razu
    } else {
        Serial.println("EQ16: Menu deactivated");
    }
}

void EQ16_displayMenu(void) {
    // GUARD: Nie rysuj gdy SDPlayer aktywny
    extern bool sdPlayerOLEDActive;
    if (sdPlayerOLEDActive) return;
    
    extern U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2;
    
    // Uproszczone menu EQ16 na wzór menu 3-punktowego - szybkie i niezawodne
    u8g2.clearBuffer();
    u8g2.setDrawColor(1);
    
    // Tytuł - prosty
    u8g2.setFont(u8g2_font_6x10_mf);
    u8g2.drawStr(80, 12, "16-BAND EQ");
    
    // Pobierz wzmocnienia
    int8_t gains[16];
    APMS_EQ16::getAll(gains);
    
    // Aktualne pasmo - uproszczone jak w menu 3-punktowego
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.setCursor(0, 12);
    u8g2.printf("Band %d", g_selectedBand + 1);
    
    u8g2.setCursor(200, 12);
    u8g2.printf("%+ddB", gains[g_selectedBand]);
    
    // Proste słupki - podobnie jak tony w menu 3-punktowym
    const int baseY = 35;
    const int maxHeight = 15;
    
    for (int i = 0; i < 16; i++) {
        int x = 8 + i * 15; // Szerokość 15 pikseli na słupek
        int gain = gains[i];
        if (gain > 16) gain = 16;
        if (gain < -16) gain = -16;
        
        int height = (abs(gain) * maxHeight) / 16;
        
        // Rysuj słupek podobnie jak suwaki w menu 3-punktowym
        if (gain >= 0 && height > 0) {
            u8g2.drawBox(x + 1, baseY - height, 13, height);
        } else if (gain < 0 && height > 0) {
            u8g2.drawBox(x + 1, baseY + 1, 13, height);
        }
        
        // Podświetl wybrany ramką dookoła (nie wypełnionym prostokątem)
        if (i == g_selectedBand) {
            u8g2.drawFrame(x, baseY - maxHeight - 1, 15, (maxHeight * 2) + 2);
            // Nie zmieniamy koloru - pozostaje normalny
        }
        
        // Częstotliwości - tylko dla wybranego i co 4
        if (i == g_selectedBand || i % 4 == 0) {
            const char* freq[] = {"31","63","125","250","500","1k","2k","4k","8k","16k","22","355","710","1.4k","5.7k","11k"};
            u8g2.setFont(u8g2_font_4x6_mf);
            u8g2.drawStr(x, 60, freq[i]);
        }
    }
    
    // Linia 0dB jak suwaki w menu 3-punktowym
    u8g2.drawHLine(6, baseY, 245);
    
    // Status jak w menu 3-punktowym
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.setCursor(0, 25);
    u8g2.printf("EQ16:%s", APMS_EQ16::isEnabled() ? "ON" : "OFF");
    
    u8g2.sendBuffer();
}

void EQ16_selectPrevBand(void) {
    if(g_selectedBand > 0) {
        g_selectedBand--;
    } else {
        g_selectedBand = APMS_EQ16::BANDS - 1; // Wrap around
    }
}

void EQ16_selectNextBand(void) {
    if(g_selectedBand < (APMS_EQ16::BANDS - 1)) {
        g_selectedBand++;
    } else {
        g_selectedBand = 0; // Wrap around
    }
}

void EQ16_increaseBandGain(void) {
    int8_t currentGain = APMS_EQ16::getBand(g_selectedBand);
    if(currentGain < 16) {  // Max +16dB
        APMS_EQ16::setBand(g_selectedBand, currentGain + 1);
        APMS_EQ16::applyToAudio();  // Zastosuj zmianę od razu
        g_needsSave = true;
    }
}

void EQ16_decreaseBandGain(void) {
    int8_t currentGain = APMS_EQ16::getBand(g_selectedBand);
    if(currentGain > -16) {  // Min -16dB
        APMS_EQ16::setBand(g_selectedBand, currentGain - 1);
        APMS_EQ16::applyToAudio();  // Zastosuj zmianę od razu
        g_needsSave = true;
    }
}

float EQ16_processSample(float sample, float unused, bool isLeft) {
    // UWAGA: Ta funkcja nie jest używana w nowej bibliotece ESP32-audioI2S
    // Przetwarzanie audio odbywa się przez wbudowane filtry IIR w Audio.cpp
    // za pomocą metody setTone(low, mid, high)
    
    // Zachowana dla kompatybilności API, ale zwraca próbkę bez zmian
    (void)unused;
    (void)isLeft;
    
    return sample;
}

void EQ16_saveToSD(void) {
    // Save EQ16 settings to SD card
    Serial.println("DEBUG: Saving EQ16 settings to /eq16.txt");
    
    if (getStorage().exists("/eq16.txt")) {
        getStorage().remove("/eq16.txt"); // Remove old file
    }
    
    File myFile = getStorage().open("/eq16.txt", FILE_WRITE);
    if (myFile) {
        // Save all 16 band values
        for (uint8_t i = 0; i < APMS_EQ16::BANDS; i++) {
            myFile.println(APMS_EQ16::getBand(i));
            Serial.printf("Band %d: %d\n", i, APMS_EQ16::getBand(i));
        }
        myFile.close();
        Serial.println("DEBUG: EQ16 settings saved successfully");
    } else {
        Serial.println("ERROR: Failed to open /eq16.txt for writing");
    }
    
    g_needsSave = false;
    g_lastSaveTime = millis();
}

void EQ16_loadFromSD(void) {
    // Load EQ16 settings from SD card
    Serial.println("DEBUG: Loading EQ16 settings from /eq16.txt");
    
    if (!getStorage().exists("/eq16.txt")) {
        Serial.println("DEBUG: /eq16.txt not found, using default EQ16 settings");
        // Set default flat EQ
        for (uint8_t i = 0; i < APMS_EQ16::BANDS; i++) {
            APMS_EQ16::setBand(i, 0);
        }
        APMS_EQ16::applyToAudio();  // Zastosuj domyślne ustawienia
        return;
    }
    
    File myFile = getStorage().open("/eq16.txt", FILE_READ);
    if (myFile) {
        // Read all 16 band values
        for (uint8_t i = 0; i < APMS_EQ16::BANDS && myFile.available(); i++) {
            String line = myFile.readStringUntil('\n');
            line.trim();
            int8_t value = (int8_t)line.toInt();
            
            // Clamp value to valid range (-12 to +12)
            if (value < -12) value = -12;
            if (value > 12) value = 12;
            
            APMS_EQ16::setBand(i, value);
            Serial.printf("Loaded Band %d: %d\n", i, value);
        }
        myFile.close();
        
        // WAŻNE: Zastosuj załadowane ustawienia do audio!
        APMS_EQ16::applyToAudio();
        Serial.println("DEBUG: EQ16 settings loaded and applied to audio successfully");
    } else {
        Serial.println("ERROR: Failed to open /eq16.txt for reading");
    }
}

void EQ16_autoSave(void) {
    // Auto-save every 5 seconds if needed
    if(g_needsSave && (millis() - g_lastSaveTime) > 5000) {
        EQ16_saveToSD();
    }
}

void EQ16_loadPreset(uint8_t presetId) {
    // ========================================================================
    // PROFESJONALNE PRESETY AUDIO - ZOPTYMALIZOWANE POD KONWERSJĘ 16->3
    // ========================================================================
    // Presety zaprojektowane z uwzględnieniem wag i ograniczeń setTone()
    // Zakres: -12 do +12 dB (bezpieczny margines przed ograniczeniem)
    // ========================================================================
    
    int8_t presets[9][16] = {
        // Preset 0: Flat (Reference) - Idealnie płaskie
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        
        // Preset 1: Bass Boost - Podkreślone basy (elektronika/hip-hop)
        // Sub-bass +8dB, Bass +6dB, łagodny roll-off
        {8,7,6,4,2,0,-1,-1,-1,0,0,0,0,0,0,0},
        
        // Preset 2: Vocal Clarity - Wyraźne wokale i czysta mowa
        // Wycięcie niskich, wzmocnienie 1-4kHz (obecność wokalu)
        {-3,-2,-1,0,1,3,6,6,4,2,0,-1,-1,-2,-2,-3},
        
        // Preset 3: Radio Presence - Przejrzystość broadcast
        // Podkreślenie środka, łagodne basy i wysokie
        {0,1,2,3,4,5,6,5,4,3,2,1,0,0,0,0},
        
        // Preset 4: V-Shape (Smile) - Nowoczesne brzmienie
        // Basy i wysokie podkreślone, środek wyciszony
        {6,5,4,2,0,-2,-3,-3,-3,-2,0,2,4,5,6,7},
        
        // Preset 5: Rock/Metal - Agresywne brzmienie
        // Mocne basy, wzmocnione wysokie, lekko wycięte środki
        {7,6,4,2,1,-1,-2,-2,-1,0,2,4,6,7,8,8},
        
        // Preset 6: Jazz/Classical - Naturalne brzmienie
        // Delikatne podniesienie niskich i wysokich
        {2,2,1,1,0,0,0,0,0,0,0,1,2,3,3,2},
        
        // Preset 7: Electronic/EDM - Dynamiczne basy + krystaliczne wysokie
        // Potężne sub-bass, agresywne wysokie, delikatnie wycięty środek
        {10,9,7,5,3,1,-1,-2,-2,-1,1,3,5,7,9,10},
        
        // Preset 8: Custom (Moje) - Załaduj ostatnio zapisane ustawienia
        // Ten preset ładuje to co użytkownik zapisał ręcznie
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}  // Placeholder - będzie załadowane z SD
    };
    
    if(presetId < 9) {
        Serial.printf("EQ16: Loading preset %d...\n", presetId);
        
        if (presetId == 8) {
            // Custom - załaduj z SD (plik eq16.txt)
            Serial.println("EQ16: Loading CUSTOM preset from SD card...");
            EQ16_loadFromSD();  // Załaduje zapisane ustawienia użytkownika + applyToAudio()
        } else {
            // Normalny preset
            Serial.printf("EQ16: Applying preset %d bands...\n", presetId);
            for(uint8_t i = 0; i < APMS_EQ16::BANDS; i++) {
                APMS_EQ16::setBand(i, presets[presetId][i]);
                Serial.printf("  Band %d = %ddB\n", i, presets[presetId][i]);
            }
            
            // ZASTOSUJ ZMIANY DO AUDIO - wywołaj funkcję z namespace
            APMS_EQ16::applyToAudio();
            
            EQ16_saveToSD();  // Automatyczny zapis po załadowaniu presetu
        }
        
        const char* presetNames[] = {
            "Flat", "Bass Boost", "Vocal", "Radio", "V-Shape", "Rock/Metal", "Jazz", "Electronic", "Custom"
        };
        
        Serial.printf("EQ16: Preset '%s' loaded, applied and saved\n", presetNames[presetId]);
    } else {
        Serial.printf("EQ16: Invalid preset ID %d (max 8)\n", presetId);
    }
}

} // extern "C"
