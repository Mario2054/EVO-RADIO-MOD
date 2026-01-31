#include "SDPlayer.h"

void SDPlayer::begin(fs::FS* fs, Audio* audio){
  _fs = fs;
  _audio = audio;
}

void SDPlayer::enter(){
  _active = true;
  if(_dir.isEmpty()) _dir = "/";
  loadDir(_dir);
}

void SDPlayer::exit(){
  _active = false;
}

bool SDPlayer::isActive() const { return _active; }

String SDPlayer::currentDir() const { return _dir; }
int SDPlayer::itemCount() const { return _count; }
const SDPlayerItem& SDPlayer::item(int i) const { return _items[i]; }

bool SDPlayer::isPlaying() const { return _playing; }
String SDPlayer::nowPlayingName() const { return _nowName; }
String SDPlayer::nowPlayingPath() const { return _nowPath; }

String SDPlayer::baseName(const String& p){
  int s = p.lastIndexOf('/');
  if(s < 0) return p;
  return p.substring(s+1);
}

bool SDPlayer::isAudioFile(const String& name){
  String n = name; n.toLowerCase();
  return n.endsWith(".mp3") || n.endsWith(".wav") || n.endsWith(".aac") ||
         n.endsWith(".flac") || n.endsWith(".ogg") || n.endsWith(".opus");
}

String SDPlayer::parentDir(const String& dir){
  if(dir == "/" || dir.length() == 0) return "/";
  String d = dir;
  if(d.endsWith("/") && d.length() > 1) d.remove(d.length()-1);
  int p = d.lastIndexOf('/');
  if(p <= 0) return "/";
  return d.substring(0, p);
}

bool SDPlayer::loadDir(const String& dir){
  if(!_fs) return false;

  String d = dir;
  if(d.isEmpty()) d = "/";
  if(!d.startsWith("/")) d = "/" + d;

  File root = _fs->open(d, FILE_READ);
  if(!root || !root.isDirectory()){
    if(root) root.close();
    return false;
  }

  _dir = d;
  _count = 0;

  File f = root.openNextFile();
  while(f && _count < MAX_FILES){
    String full = String(f.name());
    String name = baseName(full);

    if(f.isDirectory()){
      _items[_count++] = { name, full, true };
    }else{
      if(isAudioFile(name)){
        _items[_count++] = { name, full, false };
      }
    }
    f = root.openNextFile();
  }

  root.close();
  if(selection >= _count) selection = _count > 0 ? _count-1 : 0;
  if(selection < 0) selection = 0;
  return true;
}

bool SDPlayer::playIndex(int idx){
  if(!_audio || idx < 0 || idx >= _count) return false;

  SDPlayerItem it = _items[idx];
  if(it.isDir){
    selection = idx;
    return loadDir(it.path);
  }

  // drugi ENTER na tym samym pliku -> STOP
  if(_playing && _nowPath == it.path){
    stop();
    return true;
  }

  _audio->stopSong();

  // biblioteka Audio: odtwarzanie z FS (SD/SPIFFS)
  _audio->connecttoFS(*_fs, it.path.c_str());

  _nowPath = it.path;
  _nowName = it.name;
  _playing = true;
  selection = idx;
  return true;
}

void SDPlayer::stop(){
  if(_audio) _audio->stopSong();
  _playing = false;
}

void SDPlayer::onUp(){
  if(_count <= 0) return;
  selection--;
  if(selection < 0) selection = _count - 1;
}

void SDPlayer::onDown(){
  if(_count <= 0) return;
  selection++;
  if(selection >= _count) selection = 0;
}

void SDPlayer::onOk(){
  playIndex(selection);
}

void SDPlayer::onBack(){
  if(_dir != "/"){
    loadDir(parentDir(_dir));
    return;
  }
  exit();
}

String SDPlayer::jsonUpDir(const String& dir){
  String d = dir;
  if(d.isEmpty()) d = "/";
  String up = parentDir(d);
  return String("{\"dir\":\"") + up + "\"}";
}

String SDPlayer::jsonList(const String& dir){
  loadDir(dir);

  String out = "{\"dir\":\"" + _dir + "\",\"items\":[";
  for(int i=0;i<_count;i++){
    if(i) out += ",";
    out += "{\"type\":\"";
    out += _items[i].isDir ? "dir" : "file";
    out += "\",\"name\":\"" + _items[i].name + "\",\"path\":\"" + _items[i].path + "\"}";
  }
  out += "]}";
  return out;
}

String SDPlayer::jsonStatus(){
  String out = "{";
  out += "\"active\":" + String(_active ? "true":"false") + ",";
  out += "\"playing\":" + String(_playing ? "true":"false") + ",";
  out += "\"path\":\"" + _nowPath + "\",";
  out += "\"name\":\"" + _nowName + "\"";
  out += "}";
  return out;
}
