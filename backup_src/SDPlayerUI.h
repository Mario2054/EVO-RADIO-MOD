#pragma once
#include <Arduino.h>
#include <FS.h>
#include <U8g2lib.h>
#include "Audio.h"

// Funkcje z main.cpp (już istnieją w projekcie) – używamy do ładnego tekstu na OLED
void processText(String &text);

class SDPlayerUI {
public:
  void begin(fs::FS* fs, Audio* audio, U8G2* u8g2);

  // Wejście/wyjście
  void enter();          // wejście do Player SD (wyłącza radio)
  void exit();           // wyjście do radia (przywraca stream)
  bool isActive() const { return active; }

  // Pętla (odświeżanie ekranu + obsługa “pending”)
  void loop();

  // Nawigacja
  void up();
  void down();
  void pageUp();
  void pageDown();
  void folderUp();
  void ok();             // Enter
  void back();           // Powrót (wyjście)
  void digit(uint8_t d); // 1..9 szybki wybór z listy

  // Transport
  void playSelected();
  void togglePlayPause();
  void stop();
  void next();
  void prev();

  // Z enkodera
  void onEncoderStep(int dir);          // -1/+1
  void onEncoderClick();               // krótki klik
  void onEncoderLongClick();           // dłuższy klik
  void onEncoderVeryLongClick();       // super długi klik

  // 9 na pilocie (krótko/długo)
  void key9Start();                    // pierwszy “down”
  void key9Repeat();                   // repeat frame z NEC

  // Informacja dla main: czy SDUI przejął klawisze radia
  bool consumesRadioInputs() const { return active; }

  // Do wznowienia radia
  void setResumeInfo(int bank, int station, bool wasUrl, const String& url);

  // --- API dla WWW (status / lista / sterowanie) ---
  String jsonStatus() const;
  String jsonList() const;
  bool enterDir(const String& path);
  bool upDir();
  bool play(const String& path);
  void toggle() { togglePlayPause(); }
  void setNowPlaying(const String& p) { nowPlaying = p; }
  String getNowPlaying() const { return nowPlaying; }
  bool isPlaying() const { return playing; }
  String getCwd() const { return cwd; }
  int getSelected() const { return selected; }
  int getItemCount() const { return itemCount; }
  bool getIsDir(int i) const { return (i>=0 && i<itemCount) ? isDir[i] : false; }
  String getItem(int i) const { return (i>=0 && i<itemCount) ? items[i] : String(""); }

private:
  void scanDir();
  void draw();
  void ensureCursorVisible();
  bool isAudioFile(const String& name) const;

  // zależności
  fs::FS* FS_ = nullptr;
  Audio* audio_ = nullptr;
  U8G2* u8g2_ = nullptr;

  // stan UI
  bool active = false;
  String cwd = "/";
  static const int MAX_ITEMS = 200;
  String items[MAX_ITEMS];
  bool isDir[MAX_ITEMS];
  int itemCount = 0;

  int selected = 0;   // 0..itemCount-1
  int top = 0;        // pierwsza widoczna linia (lista)

  // playback
  String nowPlaying = "";
  bool playing = false;

  // resume radio
  int resumeBank = 1;
  int resumeStation = 1;
  bool resumeWasUrl = false;
  String resumeUrl = "";

  // ekran / odświeżanie
  unsigned long lastDraw = 0;
  bool dirty = true;

  // key9 pending logic
  bool key9Pending = false;
  bool key9RepeatSeen = false;
  unsigned long key9T0 = 0;

  // encoder press timing
  bool encPressed = false;
  unsigned long encT0 = 0;
};
