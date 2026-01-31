#include "SDPlayerUI.h"
#include <time.h>

extern int volumeValue; // z main.cpp

static const uint16_t LIST_LINES = 4;          // ile pozycji na ekranie (jak lista stacji)          // ile pozycji na ekranie
static const uint16_t DRAW_PERIOD_MS = 120;    // odświeżanie OLED

void SDPlayerUI::begin(fs::FS* fs, Audio* audio, U8G2* u8g2){
  FS_ = fs;
  audio_ = audio;
  u8g2_ = u8g2;
}

void SDPlayerUI::setResumeInfo(int bank, int station, bool wasUrl, const String& url){
  resumeBank = bank;
  resumeStation = station;
  resumeWasUrl = wasUrl;
  resumeUrl = url;
}

bool SDPlayerUI::isAudioFile(const String& name) const{
  String n = name;
  n.toLowerCase();
  return n.endsWith(".mp3") || n.endsWith(".wav") || n.endsWith(".flac") || n.endsWith(".aac") || n.endsWith(".ogg") || n.endsWith(".opus");
}

void SDPlayerUI::scanDir(){
  itemCount = 0;
  selected = 0;
  top = 0;

  if (!FS_) return;

  File dir = FS_->open(cwd.c_str());
  if (!dir || !dir.isDirectory()){
    return;
  }

  // Najpierw katalogi, potem pliki
  File f = dir.openNextFile();
  while (f && itemCount < MAX_ITEMS){
    String name = String(f.name());
    // usuń prefix katalogu żeby na liście było czytelnie
    if (name.startsWith(cwd)){
      name = name.substring(cwd.length());
      if (name.startsWith("/")) name.remove(0,1);
    }

    if (name.length() == 0){
      f = dir.openNextFile();
      continue;
    }

    bool d = f.isDirectory();
    if (d){
      items[itemCount] = name;
      isDir[itemCount] = true;
      itemCount++;
    }
    f = dir.openNextFile();
  }

  // drugi przebieg: pliki audio
  dir.rewindDirectory();
  f = dir.openNextFile();
  while (f && itemCount < MAX_ITEMS){
    String name = String(f.name());
    if (name.startsWith(cwd)){
      name = name.substring(cwd.length());
      if (name.startsWith("/")) name.remove(0,1);
    }
    if (name.length() == 0){
      f = dir.openNextFile();
      continue;
    }

    if (!f.isDirectory() && isAudioFile(name)){
      items[itemCount] = name;
      isDir[itemCount] = false;
      itemCount++;
    }
    f = dir.openNextFile();
  }
}

void SDPlayerUI::enter(){
  if (active) return;
  active = true;
  cwd = "/";
  nowPlaying = "";
  playing = false;
  key9Pending = false;
  key9RepeatSeen = false;
  scanDir();
  dirty = true;

  // Ekran powitalny
  if (u8g2_){
    u8g2_->clearBuffer();
    u8g2_->setFont(u8g2_font_fub14_tf);
    u8g2_->drawStr(70, 36, "PLAYER SD");
    u8g2_->sendBuffer();
  }
  lastDraw = 0;
}

void SDPlayerUI::exit(){
  if (!active) return;
  active = false;
  dirty = true;
  // UWAGA: wznowienie radia robi main (żeby użyć jego zmiennych i funkcji changeStation/connecttohost)
}

void SDPlayerUI::ensureCursorVisible(){
  if (selected < top) top = selected;
  if (selected >= top + (int)LIST_LINES) top = selected - (int)LIST_LINES + 1;
  if (top < 0) top = 0;
  if (top > max(0, itemCount - (int)LIST_LINES)) top = max(0, itemCount - (int)LIST_LINES);
}

void SDPlayerUI::draw(){
  if (!u8g2_) return;

  auto trimToPx = [&](String s, int maxPx)->String{
    while(s.length() > 0 && u8g2_->getStrWidth(s.c_str()) > maxPx){
      s.remove(s.length()-1);
    }
    return s;
  };

  auto drawTopBar = [&](){
      // pasek u góry: VOL po lewej, SD PLAYER na środku, ikonka głośnika po prawej
      u8g2_->setDrawColor(1);
      u8g2_->drawHLine(0, 12, 128);

      u8g2_->setFont(u8g2_font_5x8_tf);

      // Left: volume
      String vol = "VOL:" + String(volumeValue);
      u8g2_->drawStr(0, 10, vol.c_str());

      // Center: title
      const char* title = "SD PLAYER";
      int titleW = u8g2_->getStrWidth(title);
      u8g2_->drawStr((128 - titleW)/2, 10, title);

      // Right: speaker icon

    };

  u8g2_->clearBuffer();
  drawTopBar();

  // =================== EKRAN ODTWARZANIA ===================
  if (playing && nowPlaying.length()){
    u8g2_->setFont(u8g2_font_6x12_tf);

    String name = nowPlaying;
    processText(name);

    // nazwa w 1-2 linie
    String l1 = name;
    String l2 = "";
    if (u8g2_->getStrWidth(l1.c_str()) > 256){
      int splitPos = -1;
      for(int i=l1.length()-1; i>0; --i){
        char c = l1[i];
        if (c==' ' || c=='-' || c=='_'){ splitPos = i; break; }
      }
      if (splitPos > 8){
        l2 = l1.substring(splitPos+1);
        l1 = l1.substring(0, splitPos);
      } else {
        l1 = trimToPx(l1, 256);
      }
    }

    l1 = trimToPx(l1, 256);
    u8g2_->drawStr(0, 26, l1.c_str());
    if (l2.length()){
      l2 = trimToPx(l2, 256);
      u8g2_->drawStr(0, 38, l2.c_str());
    }

    // format po rozszerzeniu
    String ext = nowPlaying;
    ext.toLowerCase();
    String fmt = "-";
    int p = ext.lastIndexOf('.');
    if (p >= 0){
      fmt = ext.substring(p+1);
      fmt.toUpperCase();
    }

    String info1 = "FORMAT: " + fmt;
    info1 = trimToPx(info1, 256);
    u8g2_->drawStr(0, 52, info1.c_str());

    String info2 = "FOLDER: " + cwd;
    processText(info2);
    info2 = trimToPx(info2, 256);
    u8g2_->drawStr(0, 62, info2.c_str());

    u8g2_->sendBuffer();
    return;
  }

  // =================== LISTA PLIKÓW ===================
  u8g2_->setFont(u8g2_font_6x12_tf);

  String header = "SD: " + cwd + "  " + String(selected+1) + "/" + String(max(1, itemCount));
  processText(header);
  header = trimToPx(header, 256);
  u8g2_->drawStr(0, 24, header.c_str());
  u8g2_->drawLine(0, 26, 256, 26);

  for (int i = 0; i < (int)LIST_LINES; i++){
    int idx = top + i;
    if (idx >= itemCount) break;

    String line = items[idx];
    if (isDir[idx]) line = String("[") + line + "]";
    processText(line);
    line = trimToPx(line, 256);

    int y = 26 + (i+1)*13;

    if (idx == selected){
      u8g2_->setDrawColor(1);
      u8g2_->drawBox(0, y-10, 256, 13);
      u8g2_->setDrawColor(0);
      u8g2_->drawStr(0, y, line.c_str());
      u8g2_->setDrawColor(1);
    } else {
      u8g2_->drawStr(0, y, line.c_str());
    }
  }

  u8g2_->setFont(u8g2_font_5x8_tf);
  u8g2_->drawStr(0, 64, "G/D=lista  ENTER=play  BACK=wyjscie  1-9=wybor");
  u8g2_->sendBuffer();
}

void SDPlayerUI::loop(){
  if (!active) return;

  // pending long-press na “9”
  if (key9Pending){
    unsigned long dt = millis() - key9T0;
    if (key9RepeatSeen && dt > 650){
      // długie przytrzymanie -> zostajemy w SD Playerze (już active),
      // tu tylko kasujemy pending
      key9Pending = false;
    } else if (!key9RepeatSeen && dt > 420){
      // krótko – nic nie robimy tutaj (main obsługuje “9” krótko dla radia),
      // ale gdy już jesteśmy w SD Playerze, to możemy potraktować to jak digit(9)
      key9Pending = false;
      digit(9);
    }
  }

  // encoder timing (jeśli main wywołuje onEncoderClick/LongClick to to niepotrzebne)
  // – zostawione tylko na wypadek, gdyby ktoś chciał sterować bez ezButton.

  if (dirty || (millis() - lastDraw) > DRAW_PERIOD_MS){
    draw();
    lastDraw = millis();
    dirty = false;
  }
}

void SDPlayerUI::up(){
  if (!active) return;
  if (itemCount == 0) return;
  selected--;
  if (selected < 0) selected = itemCount - 1;
  ensureCursorVisible();
  dirty = true;
}

void SDPlayerUI::down(){
  if (!active) return;
  if (itemCount == 0) return;
  selected++;
  if (selected >= itemCount) selected = 0;
  ensureCursorVisible();
  dirty = true;
}

void SDPlayerUI::pageUp(){
  for(int i=0;i<5;i++) up();
}

void SDPlayerUI::pageDown(){
  for(int i=0;i<5;i++) down();
}

void SDPlayerUI::folderUp(){
  if (!active) return;
  if (cwd == "/" || cwd.length() == 0){
    dirty = true;
    return;
  }
  int p = cwd.lastIndexOf('/');
  if (p <= 0) cwd = "/";
  else cwd = cwd.substring(0, p);
  scanDir();
  dirty = true;
}

void SDPlayerUI::ok(){
  if (!active) return;
  if (itemCount == 0) return;

  if (isDir[selected]){
    // wejście do katalogu
    String next = items[selected];
    if (!cwd.endsWith("/")) cwd += "/";
    cwd += next;
    if (!cwd.endsWith("/")) {} // katalog bez końcowego / – zostawiamy
    scanDir();
    dirty = true;
  } else {
    playSelected();
  }
}

void SDPlayerUI::back(){
  if (!active) return;
  // Back: jeżeli jesteśmy w podkatalogu -> folderUp, a jak w root -> wyjście
  if (cwd != "/"){
    folderUp();
  } else {
    exit();
  }
}

void SDPlayerUI::digit(uint8_t d){
  if (!active) return;
  if (d < 1 || d > 9) return;
  int idx = top + (d - 1);
  if (idx >= 0 && idx < itemCount){
    selected = idx;
    ensureCursorVisible();
    dirty = true;
    if (!isDir[selected]){
      playSelected(); // szybki wybór = od razu gra
    }
  }
}

void SDPlayerUI::playSelected(){
  if (!active || !audio_ || !FS_) return;
  if (itemCount == 0) return;
  if (isDir[selected]) return;

  String path = cwd;
  if (!path.endsWith("/")) path += "/";
  path += items[selected];

  nowPlaying = items[selected];
  processText(nowPlaying);

  audio_->stopSong();
  delay(20);
  audio_->connecttoFS(*FS_, path.c_str());

  playing = true;
  dirty = true;
}

void SDPlayerUI::togglePlayPause(){
  if (!active || !audio_) return;
  // ESP32-audioI2S nie ma “pause” w klasyczny sposób dla plików,
  // więc robimy prostą logikę: jak gra -> stop, jak stop -> playSelected
  if (playing){
    audio_->stopSong();
    playing = false;
  } else {
    playSelected();
  }
  dirty = true;
}

void SDPlayerUI::stop(){
  if (!active || !audio_) return;
  audio_->stopSong();
  playing = false;
  dirty = true;
}

void SDPlayerUI::next(){
  if (!active) return;
  down();
  if (itemCount && !isDir[selected]) playSelected();
}

void SDPlayerUI::prev(){
  if (!active) return;
  up();
  if (itemCount && !isDir[selected]) playSelected();
}

void SDPlayerUI::onEncoderStep(int dir){
  if (dir < 0) up();
  else if (dir > 0) down();
}

void SDPlayerUI::onEncoderClick(){
  // klik = Enter
  ok();
}

void SDPlayerUI::onEncoderLongClick(){
  // długi = play/pause
  togglePlayPause();
}

void SDPlayerUI::onEncoderVeryLongClick(){
  // super długi = stop
  stop();
}

void SDPlayerUI::key9Start(){
  if (!active){
    // main obsłuży wejście do playera
    return;
  }
  key9Pending = true;
  key9RepeatSeen = false;
  key9T0 = millis();
}

void SDPlayerUI::key9Repeat(){
  if (!active) return;
  if (!key9Pending) return;
  key9RepeatSeen = true;
}


// ---------------- WWW API ----------------

static String _sduiJsonEscape(const String& s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '\\' || c == '"') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if (c == '\t') {
      out += "\\t";
    } else {
      out += c;
    }
  }
  return out;
}

String SDPlayerUI::jsonStatus() const {
  String json = "{";
  json += "\"active\":" + String(active ? 1 : 0);
  json += ",\"playing\":" + String(playing ? 1 : 0);
  json += ",\"cwd\":\"" + _sduiJsonEscape(cwd) + "\"";
  json += ",\"selected\":" + String(selected);
  json += ",\"count\":" + String(itemCount);
  json += ",\"nowPlaying\":\"" + _sduiJsonEscape(nowPlaying) + "\"";
  json += "}";
  return json;
}

String SDPlayerUI::jsonList() const {
  String json = "{";
  json += "\"cwd\":\"" + _sduiJsonEscape(cwd) + "\",";
  json += "\"items\":[";
  for (int i = 0; i < itemCount; i++) {
    if (i) json += ',';
    json += "{";
    json += "\"name\":\"" + _sduiJsonEscape(items[i]) + "\",";
    json += "\"dir\":" + String(isDir[i] ? 1 : 0);
    json += "}";
  }
  json += "]}";
  return json;
}

bool SDPlayerUI::enterDir(const String& path) {
  if (!FS_) return false;
  if (!path.length()) return false;

  File f = FS_->open(path);
  if (!f) return false;
  bool ok = f.isDirectory();
  f.close();
  if (!ok) return false;

  cwd = path;
  if (!cwd.startsWith("/")) cwd = "/" + cwd;
  if (!cwd.endsWith("/")) cwd += "/";

  scanDir();
  draw();
  return true;
}

bool SDPlayerUI::play(const String& path) {
  if (!FS_ || !audio_) return false;
  if (!path.length()) return false;

  File f = FS_->open(path);
  if (!f) return false;

  if (f.isDirectory()) {
    f.close();
    return enterDir(path);
  }

  f.close();

  audio_->stopSong();
  bool ok = audio_->connecttoFS(*FS_, path.c_str());
  if (ok) {
    nowPlaying = path;
    int slash = nowPlaying.lastIndexOf('/');
    if (slash >= 0) nowPlaying = nowPlaying.substring(slash + 1);
    playing = true;
    active = true;
    draw();
  }
  return ok;
}
// Brakująca definicja funkcji upDir()
bool SDPlayerUI::upDir() {
    // Możesz dodać tu logikę jeśli będzie potrzebna
    return false;
}


