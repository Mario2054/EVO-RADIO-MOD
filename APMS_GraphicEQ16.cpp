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
  s_audio->setGraphicEQ16(s_gains);
  s_audio->enableGraphicEQ16(s_enabled);
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
    g_needsSave = true;
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
        g_needsSave = true;
    }
}

void EQ16_decreaseBandGain(void) {
    int8_t currentGain = APMS_EQ16::getBand(g_selectedBand);
    if(currentGain > -16) {  // Min -16dB
        APMS_EQ16::setBand(g_selectedBand, currentGain - 1);
        g_needsSave = true;
    }
}

float EQ16_processSample(float sample, float unused, bool isLeft) {
    if (!APMS_EQ16::isEnabled()) {
        return sample; // EQ wyłączony - przepuść próbkę bez zmian
    }
    
    (void)unused; // Suppress unused warnings
    (void)isLeft; // Ten EQ jest mono - nie różnicujemy kanałów
    
    // Bez debugowania w pętli audio dla maksymalnej wydajności
    
    // Podstawowe wzmocnienie 0dB (x1.0) - normalne wzmocnienie
    float output = sample * 1.0f;
    
    // Rozszerzona tablica: dB -> współczynnik liniowy dla zakresu [-16..+16]
    static const float dbToLinear[33] = {
        // -16dB do -9dB
        0.158f, 0.178f, 0.200f, 0.224f, 0.251f, 0.282f, 0.316f, 0.355f,
        // -8dB do -1dB  
        0.398f, 0.447f, 0.501f, 0.562f, 0.631f, 0.708f, 0.794f, 0.891f,
        // 0dB
        1.000f,
        // +1dB do +8dB
        1.122f, 1.259f, 1.413f, 1.585f, 1.778f, 1.995f, 2.239f, 2.512f,
        // +9dB do +16dB
        2.818f, 3.162f, 3.548f, 3.981f, 4.467f, 5.012f, 5.623f, 6.310f
    };
    
    int8_t gains[16];
    APMS_EQ16::getAll(gains);
    
    // SZYBKI KOREKTOR: Uproszczony algorytm dla wydajności
    float totalGain = 0.0f;
    for (int i = 0; i < 16; i++) {
        // Szybka konwersja dB -> linear przez lookup table
        int dbIndex = gains[i] + 16; // shift -16..+16 to 0..32
        if (dbIndex < 0) dbIndex = 0;
        if (dbIndex > 32) dbIndex = 32;
        totalGain += dbToLinear[dbIndex];
    }
    
    // Użyj średniej z wszystkich pasm jako wzmocnienie
    output *= (totalGain / 16.0f);
    
    // Minimalne sprawdzenie zmian - tylko co 1000 próbek  
    static uint16_t changeCheckCount = 0;
    static int8_t lastGains[16] = {0};
    if (++changeCheckCount >= 1000) {
        for (int i = 0; i < 16; i++) {
            if (gains[i] != lastGains[i]) {
                Serial.printf("EQ16: Gains changed\n");
                memcpy(lastGains, gains, 16);
                break;
            }
        }
        changeCheckCount = 0;
    }
    
    // Inteligentny limiter - zachowuje dynamikę przy wysokich poziomach
    if (output > 0.8f) {
        float excess = output - 0.8f;
        output = 0.8f + excess * (0.2f / (1.0f + excess * 3.0f)); // soft knee
    } else if (output < -0.8f) {
        float excess = -output - 0.8f;
        output = -0.8f - excess * (0.2f / (1.0f + excess * 3.0f)); // soft knee
    }
    
    return output;
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
        Serial.println("DEBUG: EQ16 settings loaded successfully");
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
    // Professional audio presets for 16-band EQ (32Hz-16kHz)
    // Values in dB: -12 to +12 range
    int8_t presets[5][16] = {
        // Preset 0: Flat (Reference)
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
        
        // Preset 1: Bass Boost (Enhanced low-end for electronic/hip-hop)
        {8,6,4,2,1,0,-1,-1,-1,0,0,0,0,0,0,0},
        
        // Preset 2: Vocal (Clear speech and vocals)
        {-2,-1,0,1,2,4,6,5,3,1,0,-1,-1,-1,-2,-2},
        
        // Preset 3: Presence (Radio/broadcast clarity)
        {0,0,1,2,3,4,5,4,3,2,1,0,0,0,0,0},
        
        // Preset 4: V-Shape (Modern smile curve)
        {4,3,2,1,0,-1,-2,-2,-2,-1,0,1,2,3,4,5}
    };
    
    if(presetId < 5) {
        for(uint8_t i = 0; i < APMS_EQ16::BANDS; i++) {
            APMS_EQ16::setBand(i, presets[presetId][i]);
        }
        g_needsSave = true;
        Serial.printf("EQ16: Loaded preset %d\n", presetId);
    }
}

} // extern "C"
