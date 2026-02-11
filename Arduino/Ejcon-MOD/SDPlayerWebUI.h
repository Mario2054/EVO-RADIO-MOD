#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SD.h>
#include <FS.h>
#include <functional>

// Forward declarations
class Audio;
class SDPlayerOLED;

// Minimal HTML/CSS/JS to mimic the screenshot layout.
// Page polls /sdplayer/api/list every ~1s.

static const char SDPLAYER_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>SD Player Control</title>
<style>
  body{font-family:Arial,Helvetica,sans-serif;background:#d8d8d8;margin:0}
  .wrap{max-width:900px;margin:30px auto;padding:10px;text-align:center}
  h1{margin:10px 0 20px 0;color:#333}
  .status{display:inline-block;padding:14px 80px;border-radius:12px;color:#fff;font-weight:700}
  .inactive{background:#e74c3c}
  .active{background:#2ecc71}
  .info{margin:14px 0;color:#111}
  .btnrow{margin:16px 0}
  button{background:#49b24f;border:0;color:#fff;padding:12px 22px;margin:6px;border-radius:6px;font-size:16px;cursor:pointer}
  button:active{transform:scale(0.98)}
  .listbox{margin:20px auto;background:#fff;border-radius:12px;padding:18px;max-width:700px;box-shadow:0 2px 6px rgba(0,0,0,.15)}
  .path{background:#e7f3ff;border-radius:6px;padding:10px 12px;text-align:left;margin-bottom:12px}
  .items{max-height:220px;overflow:auto;text-align:left;border-radius:6px}
  .item{padding:10px 12px;border-radius:6px;margin:4px 0;background:#f4f7fb;display:flex;gap:10px;align-items:center}
  .item:hover{background:#e9f1ff}
  .icon{width:22px}
  .sliderWrap{margin:20px 0}
  input[type=range]{width:340px}
</style>
</head>
<body>
<div class="wrap">
  <h1>SD Player Control</h1>
  <div id="status" class="status inactive">Inactive</div>

  <div class="info">
    <div>Current Directory: <b id="cwd">/</b></div>
    <div>Now Playing: <b id="now">None</b></div>
    <div>Status: <b id="playStatus">Stopped</b></div>
  </div>

  <div class="btnrow">
    <button onclick="post('/sdplayer/api/playSelected')">Play Selected</button>
    <button id="pauseBtn" onclick="post('/sdplayer/api/pause')">Pause / Resume</button>
    <button onclick="post('/sdplayer/api/stop')">Stop</button>
    <button onclick="post('/sdplayer/api/next')">Next</button>
    <button onclick="post('/sdplayer/api/prev')">Previous</button>
  </div>

  <div class="btnrow">
    <button onclick="post('/sdplayer/api/up')">Up Directory</button>
    <button onclick="refresh()">Refresh</button>
  </div>

  <div class="listbox">
    <div class="path">📁 <span id="path2">/</span></div>
    <div class="items" id="items"></div>
  </div>

  <div class="sliderWrap">
    <div>Volume: <b id="vol">7</b></div>
    <input id="volr" type="range" min="0" max="21" value="7" oninput="setVol(this.value)"/>
  </div>

  <div class="btnrow">
    <button onclick="back()">Back to Menu</button>
  </div>
</div>

<script>
console.log('=== SD Player JavaScript START ===');
let data=null;
let refreshTimer=null;

function post(url){
  console.log('POST:',url);
  fetch(url,{method:'POST'})
    .then(r => {
      console.log('POST response:',r.status);
      refresh();
    })
    .catch(e => console.error('POST error:',e));
}

function refresh(){
  console.log('Refresh called');
  fetch('/sdplayer/api/list')
    .then(r=>{
      console.log('List response status:',r.status);
      if(!r.ok) throw new Error('HTTP error! status: '+r.status);
      return r.json();
    })
    .then(j=>{
      console.log('List data received:',j);
      data=j;
      render();
    })
    .catch(e=>{
      console.error('Refresh error:',e);
    });
}

function setVol(v){
  document.getElementById('vol').innerText=v;
  fetch('/sdplayer/api/vol?v='+encodeURIComponent(v),{method:'POST'});
}

function back(){ 
  console.log('Back to menu clicked');
  if(refreshTimer) clearInterval(refreshTimer);  // Zatrzymaj odświeżanie
  fetch('/sdplayer/api/back', {method:'POST'})
    .then(() => {
      console.log('Redirecting to /');
      location.href='/';
    })
    .catch(e => console.error('Back error:',e));
}

function render(){
  console.log('Render called, data:',data);
  if(!data){
    console.log('No data yet, skipping render');
    return;
  }
  
  try{
    document.getElementById('cwd').innerText=data.cwd||'/';
    document.getElementById('path2').innerText=data.cwd||'/';
    document.getElementById('now').innerText=data.now||'None';
    document.getElementById('vol').innerText=data.vol||0;
    document.getElementById('volr').value=data.vol||0;
    
    // Aktualizuj status odtwarzania
    const playStatus = document.getElementById('playStatus');
    if(playStatus){
      playStatus.innerText = data.status || 'Stopped';
    }
    
    // Aktualizuj tekst przycisku Pause/Resume
    const pauseBtn = document.getElementById('pauseBtn');
    if(pauseBtn && data.status){
      if(data.status === 'Playing'){
        pauseBtn.innerText = 'Pause';
      } else if(data.status === 'Paused'){
        pauseBtn.innerText = 'Resume';
      } else {
        pauseBtn.innerText = 'Pause / Resume';
      }
    }

    const s=document.getElementById('status');
    s.className='status active';
    s.innerText='Active';

    const box=document.getElementById('items');
    box.innerHTML='';
    if(data.items && Array.isArray(data.items)){
      console.log('Rendering',data.items.length,'items');
      data.items.forEach((it,idx)=>{
        const row=document.createElement('div');
        row.className='item';
        const ic=document.createElement('div');
        ic.className='icon';
        ic.innerText=it.d?'📁':'🎵';
        const nm=document.createElement('div');
        nm.innerText=it.d?('/'+it.n):it.n;
        row.appendChild(ic); row.appendChild(nm);
        row.onclick=()=>{
          if(it.d){
            fetch('/sdplayer/api/cd?p='+encodeURIComponent(data.cwd=='/'?('/'+it.n):(data.cwd+'/'+it.n)))
              .then(()=>refresh());
          }else{
            fetch('/sdplayer/api/play?i='+idx,{method:'POST'}).then(()=>refresh());
          }
        };
        box.appendChild(row);
      });
    }else{
      console.log('No items or items not array');
    }
  }catch(e){
    console.error('Render error:',e);
  }
}

console.log('SDPlayer script loaded, starting initial refresh');
refresh();  // Pierwsze załadowanie
refreshTimer = setInterval(refresh, 5000);  // Odświeżaj co 5 sekund (tylko status/volume)
</script>
</body>
</html>
)HTML";

class SDPlayerWebUI {
public:
    SDPlayerWebUI();
    void begin(AsyncWebServer* server, Audio* audioPtr = nullptr);
    void setExitCallback(std::function<void()> callback);
    void setOLED(SDPlayerOLED* oled);  // Ustaw OLED display
    
    // Kontrola odtwarzacza
    void playFile(const String& path);
    void playIndex(int index);
    void pause();
    void stop();
    void next();
    void prev();
    void playNextAuto(); // Automatyczne odtwarzanie następnego utworu (zapętlenie)
    void setVolume(int vol);
    
    // Zarządzanie katalogiem
    void changeDirectory(const String& path);
    void upDirectory();
    String getCurrentDirectory() { return _currentDir; }
    
    // Status
    bool isPlaying() { return _isPlaying; }
    bool isPaused() { return _isPaused; }
    String getCurrentFile() { return _currentFile; }
    int getVolume() { return _volume; }
    int getSelectedIndex() const { return _selectedIndex; }  // Zwraca aktualny indeks zaznaczonego pliku

private:
    AsyncWebServer* _server;
    Audio* _audio;
    SDPlayerOLED* _oled;
    std::function<void()> _exitCallback;
    
    String _currentDir;
    String _currentFile;
    int _volume;
    bool _isPlaying;
    bool _isPaused;
    int _selectedIndex;
    
    struct FileItem {
        String name;
        bool isDir;
    };
    std::vector<FileItem> _fileList;
    
    void scanCurrentDirectory();
    void buildFileList(JsonArray& items);
    bool isAudioFile(const String& filename);
    void sortFileList();
    
    // Handler functions
    void handleRoot(AsyncWebServerRequest *request);
    void handleList(AsyncWebServerRequest *request);
    void handlePlay(AsyncWebServerRequest *request);
    void handlePlaySelected(AsyncWebServerRequest *request);
    void handlePause(AsyncWebServerRequest *request);
    void handleStop(AsyncWebServerRequest *request);
    void handleNext(AsyncWebServerRequest *request);
    void handlePrev(AsyncWebServerRequest *request);
    void handleVol(AsyncWebServerRequest *request);
    void handleCd(AsyncWebServerRequest *request);
    void handleUp(AsyncWebServerRequest *request);
    void handleBack(AsyncWebServerRequest *request);
};
