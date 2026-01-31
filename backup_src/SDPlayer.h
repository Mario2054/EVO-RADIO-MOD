#pragma once
#include <Arduino.h>
#include <FS.h>
#include "Audio.h"

#ifndef MAX_FILES
#define MAX_FILES 150
#endif

struct SDPlayerItem {
  String name;
  String path;
  bool isDir;
};

class SDPlayer {
public:
  void begin(fs::FS* fs, Audio* audio);

  void enter();
  void exit();
  bool isActive() const;

  bool loadDir(const String& dir);
  String currentDir() const;

  int itemCount() const;
  const SDPlayerItem& item(int i) const;

  bool playIndex(int idx);
  void stop();

  bool isPlaying() const;
  String nowPlayingName() const;
  String nowPlayingPath() const;

  void onUp();
  void onDown();
  void onOk();
  void onBack();

  String jsonList(const String& dir);
  String jsonStatus();
  String jsonUpDir(const String& dir);

  int selection = 0;

private:
  fs::FS* _fs = nullptr;
  Audio* _audio = nullptr;

  bool _active = false;
  bool _playing = false;

  String _dir = "/";
  String _nowPath = "";
  String _nowName = "";

  SDPlayerItem _items[MAX_FILES];
  int _count = 0;

  String baseName(const String& p);
  bool isAudioFile(const String& name);
  String parentDir(const String& dir);
};
