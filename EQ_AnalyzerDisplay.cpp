#include "EQ_AnalyzerDisplay.h"
#include "AudioRuntimeEQ_Evo.h"

#include <U8g2lib.h>
#include <time.h>

// Rozmiar ekranu OLED używany przez analizator (dla Twojego projektu 256x64).
// Jeśli kiedyś zmienisz rozdzielczość – popraw tu i w main.cpp.
#define SCREEN_WIDTH  256
#define SCREEN_HEIGHT 64

// ─────────────────────────────────────
// ZMIENNE GLOBALNE, KTÓRE JUŻ MASZ W main.cpp
// (tu tylko je „podglądamy” przez extern)
// ─────────────────────────────────────

extern U8G2 u8g2;

extern uint8_t volumeValue;
extern bool volumeMute;

extern String stationName;
extern String stationNameStream;
extern String stationStringWeb;

// font z U8g2 (bibliotekowy)
extern const uint8_t u8g2_font_5x8_mr[];

// ─────────────────────────────────────
// ZMIENNE ANALIZATORA / EQ – współdzielone z AudioRuntimeEQ_Evo.cpp
// ─────────────────────────────────────

// Te dwie flagi są zadeklarowane jako extern w AudioRuntimeEQ_Evo.h,
// a używane m.in. w AudioRuntimeEQ_Evo.cpp. Tu robimy ich faktyczną definicję.
bool eqEnabled         = false;  // DSP EQ – u nas domyślnie wyłączony
bool eqAnalyzerEnabled = false;  // analizator FFT – startowo wyłączony, włączy go Web UI

// Tablica wzmocnień EQ (w dB) – 16 pasm
float eqBandGains[RUNTIME_EQ_BANDS] = {0.0f};

// Globalne parametry analizatora – AudioRuntimeEQ_Evo.cpp widzi je jako extern
float eqAnalyzerSensitivity = DEFAULT_ANALYZER_SENSITIVITY;
bool  eqAnalyzerPreGain     = true;
bool  eqAnalyzerNormalize   = true;
bool  eqAnalyzerAGC         = true;

// 16 pasm widocznych na wyświetlaczu
const uint8_t EQ_BANDS = 16;
uint8_t eqLevel[EQ_BANDS]    = {0};   // aktualna wysokość słupków 0–100
uint8_t eqPeak[EQ_BANDS]     = {0};   // pozycja „peak” dla danego słupka
uint8_t eqPeakHold[EQ_BANDS] = {0};   // licznik opóźnienia dla peak
const uint8_t eqPeakHoldThreshold = 10;  // ile odświeżeń zanim peak zacznie opadać

// Konfiguracja zachowania trybu 6 (segmentowy pseudo-analizator)
uint8_t eq6_maxSegments      = 32;  // ilość kreseczek na wysokość słupka
uint8_t eq6_riseSpeed        = 10;  // szybkość wzrostu słupków (0-100 na krok)
uint8_t eq6_fallSpeed        = 10;  // szybkość opadania słupków
uint8_t eq6_peakHoldFrames   = 2;   // jak długo peak stoi zanim zacznie spadać
uint8_t eq6_peakFallSpeed    = 9;   // jak szybko opada peak

// ─────────────────────────────────────
// FUNKCJE POMOCNICZE / API
// ─────────────────────────────────────

void eqAnalyzerInit()
{
  // DSP EQ wyłączony, analizator też – ruszy dopiero po sygnale z WWW
  eqEnabled         = true;
  eqAnalyzerEnabled = true;

  for (int i = 0; i < RUNTIME_EQ_BANDS; ++i)
    eqBandGains[i] = 0.0f;

  // Ustawiamy domyślne parametry analizatora
  eq_set_analyzer_sensitivity(DEFAULT_ANALYZER_SENSITIVITY);
  eq_set_analyzer_pre_gain(true);
  eq_set_analyzer_normalize(true);
  eq_set_analyzer_agc(true);
}

// to wywołujesz z handlera HTTP / WebSocket
void eqAnalyzerSetFromWeb(bool enabled)
{
  eqAnalyzerEnabled = enabled;
}

// ─────────────────────────────────────
// STYL 5 – 16 słupków, zegar + ikonka głośnika
// ─────────────────────────────────────

void vuMeterMode5() // Tryb 5: 16 słupków – dynamiczny analizator z zegarem i ikonką głośnika
{
  // Jeśli analizator jest wyłączony – pokaż prosty komunikat
  if (!eqAnalyzerEnabled)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(10, 24);
    u8g2.print("ANALYZER OFF");
    u8g2.setCursor(10, 40);
    u8g2.print("Enable in Web UI");
    u8g2.sendBuffer();
    return;
  }

  // 1. Pobranie poziomów z analizatora FFT (0..1)
  float levels[RUNTIME_EQ_BANDS];
  float peaks[RUNTIME_EQ_BANDS];
  eq_get_analyzer_levels(levels);
  eq_get_analyzer_peaks(peaks);

  // Przepisujemy do eqLevel/eqPeak w skali 0..100 dla rysowania słupków
  for (uint8_t i = 0; i < EQ_BANDS && i < RUNTIME_EQ_BANDS; i++)
  {
    float lv = levels[i];
    float pk = peaks[i];
    if (lv < 0.0f) lv = 0.0f;
    if (lv > 1.0f) lv = 1.0f;
    if (pk < 0.0f) pk = 0.0f;
    if (pk > 1.0f) pk = 1.0f;

    eqLevel[i] = (uint8_t)(lv * 100.0f + 0.5f);
    eqPeak[i]  = (uint8_t)(pk * 100.0f + 0.5f);
  }

  // 2. Rysowanie – zegar + ikonka głośnika u góry, słupki pod spodem
  u8g2.setDrawColor(1);
  u8g2.clearBuffer();

  // Pasek górny: zegar po lewej, stacja obok, ikonka głośnika po prawej
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5))
  {
    char timeString[9];
    if (timeinfo.tm_sec % 2 == 0)
      snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    else
      snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);

    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(4, 11);
    u8g2.print(timeString);

    // Nazwa stacji obok zegara
    uint8_t timeWidth = u8g2.getStrWidth(timeString);
    uint8_t xStation  = 4 + timeWidth + 6;

    // Zarezerwuj miejsce do ikony głośnika po prawej
    uint8_t iconX = SCREEN_WIDTH - 40;
    uint8_t maxStationWidth = 0;
    if (iconX > xStation + 4)
      maxStationWidth = iconX - xStation - 4;

    if (maxStationWidth > 0)
    {
      String nameToShow = stationName;
      if (nameToShow.length() == 0)
      {
        if (stationNameStream.length() > 0)      nameToShow = stationNameStream;
        else if (stationStringWeb.length() > 0)  nameToShow = stationStringWeb;
        else                                     nameToShow = "Radio";
      }

      // Przycinanie tekstu do wolnej szerokości
      while (nameToShow.length() > 0 &&
             u8g2.getStrWidth(nameToShow.c_str()) > maxStationWidth)
      {
        nameToShow.remove(nameToShow.length() - 1);
      }

      u8g2.setCursor(xStation, 11);
      u8g2.print(nameToShow);
    }
  }

  // Ikonka głośnika + wartość głośności po prawej
  uint8_t iconY = 2;
  uint8_t iconX = SCREEN_WIDTH - 40;

  // „kolumna” głośnika
  u8g2.drawBox(iconX, iconY + 2, 4, 7);
  // przód głośnika – linie
  u8g2.drawLine(iconX + 4, iconY + 2, iconX + 7, iconY);      // skośna góra
  u8g2.drawLine(iconX + 4, iconY + 8, iconX + 7, iconY + 10); // skośny dół
  u8g2.drawLine(iconX + 7, iconY,     iconX + 7, iconY + 10); // pion

  // „fale” dźwięku
  u8g2.drawPixel(iconX + 9,  iconY + 3);
  u8g2.drawPixel(iconX + 10, iconY + 5);
  u8g2.drawPixel(iconX + 9,  iconY + 7);

  // Wartość głośności
  u8g2.setFont(u8g2_font_5x8_mr);
  u8g2.setCursor(iconX + 14, 10);
  u8g2.print(volumeValue);

  // Linia oddzielająca pasek od słupków
  u8g2.drawHLine(0, 13, SCREEN_WIDTH);

  // Obszar słupków – od linii w dół do końca ekranu
  const uint8_t eqTopY      = 14;                    // pod paskiem
  const uint8_t eqBottomY   = SCREEN_HEIGHT - 1;     // do dołu
  const uint8_t eqMaxHeight = eqBottomY - eqTopY + 1;

  // 32 segmenty na wysokość
  const uint8_t maxSegments = 32;
  const float segmentStep = (float)eqMaxHeight / (float)maxSegments;

  // Parametry słupków – 16 sztuk
  const uint8_t barWidth = 10;
  const uint8_t barGap   = 6;

  const uint16_t totalBarsWidth = EQ_BANDS * barWidth + (EQ_BANDS - 1) * barGap;
  int16_t startX = (SCREEN_WIDTH - totalBarsWidth) / 2;
  if (startX < 0) startX = 0;

  // Rysowanie słupków z peakami
  for (uint8_t i = 0; i < EQ_BANDS; i++)
  {
    uint8_t levelPercent = eqLevel[i];  // 0-100
    uint8_t peakPercent  = eqPeak[i];   // 0-100

    // Liczba segmentów w słupku
    uint8_t segments = (levelPercent * maxSegments) / 100;
    if (segments > maxSegments) segments = maxSegments;

    // Pozycja „peak” w segmentach
    uint8_t peakSeg = (peakPercent * maxSegments) / 100;
    if (peakSeg > maxSegments) peakSeg = maxSegments;

    // x słupka
    int16_t x = startX + i * (barWidth + barGap);

    // Rysujemy segmenty od dołu
    for (uint8_t s = 0; s < segments; s++)
    {
      int16_t segBottom = eqBottomY - (int16_t)(s * segmentStep);
      int16_t segTop    = segBottom - (int16_t)segmentStep + 1;

      if (segTop < eqTopY) segTop = eqTopY;
      uint8_t segH = segBottom - segTop + 1;
      if (segH < 1) segH = 1;

      u8g2.drawBox(x, segTop, barWidth, segH);
    }

    // Peak – pojedyncza kreska nad słupkiem
    if (peakSeg > 0)
    {
      uint8_t ps = peakSeg - 1;
      int16_t peakBottom = eqBottomY - (int16_t)(ps * segmentStep);
      int16_t peakY      = peakBottom - 1;
      if (peakY >= eqTopY && peakY <= eqBottomY)
      {
        u8g2.drawBox(x, peakY, barWidth, 1);
      }
    }
  }

  // Komunikat o wyciszeniu – na środku
  if (volumeMute)
  {
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setDrawColor(0);
    u8g2.drawBox(60, (SCREEN_HEIGHT/2) - 10, SCREEN_WIDTH-120, 20);
    u8g2.setDrawColor(1);
    u8g2.setCursor((SCREEN_HEIGHT/2) - 30, (SCREEN_HEIGHT/2) + 4);
    u8g2.print("MUTED");
  }

  u8g2.sendBuffer();
}

// ─────────────────────────────────────
// STYL 6 – cienkie kreski + peak + zegar
// ─────────────────────────────────────

void vuMeterMode6() // Tryb 6: 16 słupków z cienkich „kreseczek” + peak, pełny analizator segmentowy
{
  if (!eqAnalyzerEnabled)
  {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(10, 24);
    u8g2.print("ANALYZER OFF");
    u8g2.setCursor(10, 40);
    u8g2.print("Enable in Web UI");
    u8g2.sendBuffer();
    return;
  }

  // 1. Pobranie poziomów z analizatora FFT (0..1)
  float levels[RUNTIME_EQ_BANDS];
  float peaks[RUNTIME_EQ_BANDS];
  eq_get_analyzer_levels(levels);
  eq_get_analyzer_peaks(peaks);

  for (uint8_t i = 0; i < EQ_BANDS && i < RUNTIME_EQ_BANDS; i++)
  {
    float lv = levels[i];
    float pk = peaks[i];
    if (lv < 0.0f) lv = 0.0f;
    if (lv > 1.0f) lv = 1.0f;
    if (pk < 0.0f) pk = 0.0f;
    if (pk > 1.0f) pk = 1.0f;

    eqLevel[i] = (uint8_t)(lv * 100.0f + 0.5f);
    eqPeak[i]  = (uint8_t)(pk * 100.0f + 0.5f);
  }

  // 2. Rysowanie – pasek z zegarem + stacja + głośnik u góry, cienkie słupki pod spodem
  u8g2.setDrawColor(1);
  u8g2.clearBuffer();

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 5))
  {
    char timeString[9];
    if (timeinfo.tm_sec % 2 == 0)
      snprintf(timeString, sizeof(timeString), "%2d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    else
      snprintf(timeString, sizeof(timeString), "%2d %02d", timeinfo.tm_hour, timeinfo.tm_min);

    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setCursor(4, 11);
    u8g2.print(timeString);

    uint8_t timeWidth = u8g2.getStrWidth(timeString);
    uint8_t xStation  = 4 + timeWidth + 6;

    uint8_t iconX = SCREEN_WIDTH - 40;
    uint8_t maxStationWidth = 0;
    if (iconX > xStation + 4)
      maxStationWidth = iconX - xStation - 4;

    if (maxStationWidth > 0)
    {
      String nameToShow = stationName;
      if (nameToShow.length() == 0)
      {
        if (stationNameStream.length() > 0)      nameToShow = stationNameStream;
        else if (stationStringWeb.length() > 0)  nameToShow = stationStringWeb;
        else                                     nameToShow = "Radio";
      }

      while (nameToShow.length() > 0 &&
             u8g2.getStrWidth(nameToShow.c_str()) > maxStationWidth)
      {
        nameToShow.remove(nameToShow.length() - 1);
      }

      u8g2.setCursor(xStation, 11);
      u8g2.print(nameToShow);
    }
  }

  // Ikonka głośnika po prawej
  uint8_t iconY = 2;
  uint8_t iconX = SCREEN_WIDTH - 40;

  u8g2.drawBox(iconX, iconY + 2, 4, 7);
  u8g2.drawLine(iconX + 4, iconY + 2, iconX + 7, iconY);
  u8g2.drawLine(iconX + 4, iconY + 8, iconX + 7, iconY + 10);
  u8g2.drawLine(iconX + 7, iconY,     iconX + 7, iconY + 10);

  u8g2.drawPixel(iconX + 9,  iconY + 3);
  u8g2.drawPixel(iconX + 10, iconY + 5);
  u8g2.drawPixel(iconX + 9,  iconY + 7);

  u8g2.setFont(u8g2_font_5x8_mr);
  u8g2.setCursor(iconX + 14, 10);
  u8g2.print(volumeValue);

  u8g2.drawHLine(0, 13, SCREEN_WIDTH);

  const uint8_t eqTopY      = 14;
  const uint8_t eqBottomY   = SCREEN_HEIGHT - 1;
  const uint8_t eqMaxHeight = eqBottomY - eqTopY + 1;

  const uint8_t maxSegments = eq6_maxSegments;
  const float segmentStep   = (float)eqMaxHeight / (float)maxSegments;

  const uint8_t barWidth = 6;
  const uint8_t barGap   = 4;

  const uint16_t totalBarsWidth = EQ_BANDS * barWidth + (EQ_BANDS - 1) * barGap;
  int16_t startX = (SCREEN_WIDTH - totalBarsWidth) / 2;
  if (startX < 0) startX = 0;

  for (uint8_t i = 0; i < EQ_BANDS; i++)
  {
    uint8_t levelPercent = eqLevel[i];
    uint8_t peakPercent  = eqPeak[i];

    uint8_t segments = (levelPercent * maxSegments) / 100;
    if (segments > maxSegments) segments = maxSegments;

    uint8_t peakSeg = (peakPercent * maxSegments) / 100;
    if (peakSeg > maxSegments) peakSeg = maxSegments;

    int16_t x = startX + i * (barWidth + barGap);

    for (uint8_t s = 0; s < segments; s++)
    {
      int16_t segBottom = eqBottomY - (int16_t)(s * segmentStep);
      int16_t segTop    = segBottom - 1;  // cienka kreska

      if (segTop < eqTopY) segTop = eqTopY;
      if (segBottom < segTop) segBottom = segTop;

      u8g2.drawHLine(x, segBottom, barWidth);
    }

    if (peakSeg > 0)
    {
      uint8_t ps = peakSeg - 1;
      int16_t peakBottom = eqBottomY - (int16_t)(ps * segmentStep);
      int16_t peakY      = peakBottom - 1;

      if (peakY >= eqTopY && peakY <= eqBottomY)
        u8g2.drawHLine(x, peakY, barWidth);
    }
  }

  if (volumeMute)
  {
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.setDrawColor(0);
    u8g2.drawBox(60, (SCREEN_HEIGHT/2) - 10, SCREEN_WIDTH-120, 20);
    u8g2.setDrawColor(1);
    u8g2.setCursor((SCREEN_HEIGHT/2) - 30, (SCREEN_HEIGHT/2) + 4);
    u8g2.print("MUTED");
  }

  u8g2.sendBuffer();
}
