#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <functional>
#include <vector>

// Web UI for BT settings (UART controlled). Layout based on your screenshot.

static const char BTUART_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Ustawienia Bluetooth</title>
<style>
  body{font-family:Arial,Helvetica,sans-serif;background:#f2f2f2;margin:0}
  .wrap{max-width:720px;margin:18px auto 26px auto;padding:10px}
  .title{display:flex;justify-content:center;gap:10px;align-items:center;margin:6px 0 14px 0;color:#333;font-weight:700}
  .card{background:#fff;border-radius:10px;padding:16px;margin:14px 0;box-shadow:0 1px 4px rgba(0,0,0,.12);text-align:center}
  .bar{display:inline-block;width:420px;max-width:100%;padding:10px 14px;border-radius:4px;background:#9e9e9e;color:#fff;font-weight:700}
  .sub{font-size:12px;color:#444;margin-top:8px}
  .row{display:flex;flex-wrap:wrap;gap:8px;justify-content:center;margin-top:12px}
  button{border:0;border-radius:4px;padding:10px 16px;color:#fff;background:#1e88e5;cursor:pointer}
  button.green{background:#43a047}
  button.gray{background:#616161}
  button.red{background:#e53935}
  button.small{padding:8px 14px}
  .hint{font-size:12px;color:#444;margin-top:8px}
  input[type=text], input[type=number]{padding:9px 10px;border:1px solid #cfcfcf;border-radius:4px;font-size:14px}
  .cmdline{width:92%;max-width:520px}
  .console{background:#1c1c1c;color:#cfcfcf;border-radius:6px;padding:10px;height:120px;overflow:auto;font-family:Consolas,monospace;font-size:12px;text-align:left;white-space:pre-wrap}
  .back{display:block;text-align:center;margin-top:6px}
</style>
</head>
<body>
<div class="wrap">
  <div class="title">📶 <span>Ustawienia Bluetooth</span></div>

  <div class="card">
    <div><b>Status</b></div>
    <div id="statusBar" class="bar">Bluetooth OFF</div>
    <div class="sub">Mode: <span id="modeTxt">OFF</span></div>
    <div class="sub">Device: <span id="devTxt">Ładowanie...</span></div>
  </div>

  <div class="card">
    <div><b>Mode Selection</b></div>
    <div class="row">
      <button class="green" onclick="setMode('OFF')">OFF</button>
      <button onclick="setMode('RX')">RX (Receiver)</button>
      <button onclick="setMode('TX')">TX (Transmitter)</button>
      <button onclick="setMode('AUTO')">AUTO</button>
    </div>
    <div class="hint">RX › Phone sends audio to Radio | TX › Radio sends audio to BT headphones</div>
  </div>

  <div class="card">
    <div><b>Volume (BT RX Mode)</b></div>
    <div class="row">
      <input id="volInput" type="number" min="0" max="100" value="15" style="width:70px"/>
      <button class="small" onclick="setVol()">Set</button>
    </div>
    <div class="hint">Wzmocnienie:</div>
    <div class="row">
      <button class="small" onclick="setBoost(100)">100</button>
      <button class="small" onclick="setBoost(200)">200</button>
      <button class="small" onclick="setBoost(300)">300</button>
      <button class="small" onclick="setBoost(400)">400</button>
    </div>
  </div>

  <div class="card">
    <div><b>Actions</b></div>
    <div class="row">
      <button onclick="scanDevices()">🔍 Scan Devices</button>
      <button onclick="post('/bt/api/disconnect')">Disconnect</button>
      <button class="red" onclick="post('/bt/api/delall')">Delete All Paired</button>
    </div>
  </div>

  <div class="card" id="devicesCard" style="display:none;">
    <div><b>📱 Wykryte urządzenia BT</b></div>
    <div id="devicesList" style="margin-top:10px;"></div>
  </div>

  <div class="card">
    <button onclick="post('/bt/api/save')">💾 Save Settings</button>
  </div>

  <div class="card">
    <div><b>⚠️ Terminal diagnostyczny BT UART</b></div>
    <div class="hint" style="color:#e53935;margin-bottom:12px">
      <input type="checkbox" id="terminalEnable" onchange="toggleTerminal(this.checked)" autocomplete="off"/>
      <label for="terminalEnable">Włącz terminal (komunikacja UART RX19/TX20)</label>
    </div>
    <div id="terminalPanel" style="display:none">
      <div class="row">
        <input id="cmd" class="cmdline" type="text" placeholder="Wpisz komendę, np. HELP, PING, GET, STATUS?" disabled/>
      </div>
      <div class="row">
        <button id="sendBtn" onclick="sendCmd()" disabled>Wyślij komendę</button>
      </div>
      <div id="console" class="console"></div>
    </div>
    <div id="terminalDisabled" class="hint" style="color:#999">
      Terminal wyłączony. Zaznacz checkbox powyżej aby aktywować komunikację z modułem BT.
    </div>
  </div>

  <a class="back" href="#" onclick="back();return false;">‹ Back to Menu</a>
</div>

<script>
let terminalActive = false;

function post(url){ fetch(url,{method:'POST'}).then(()=>refresh()); }

function scanDevices(){
  document.getElementById('devicesCard').style.display='none';
  document.getElementById('devicesList').innerHTML='<div class="hint">Skanowanie...</div>';
  fetch('/bt/api/scan',{method:'POST'}).then(()=>{
    setTimeout(()=>{
      fetchDevices();
    }, 3000); // Czekaj 3s na zakończenie skanowania
  });
}

function fetchDevices(){
  fetch('/bt/api/devices').then(r=>r.json()).then(d=>{
    let html='';
    if(d.devices && d.devices.length>0){
      d.devices.forEach(dev=>{
        html+='<div style="border:1px solid #ddd;padding:8px;margin:5px 0;border-radius:4px;">';
        html+='<div style="font-weight:bold;">' + (dev.name||'Nieznane') + '</div>';
        html+='<div style="font-size:11px;color:#666;">' + dev.mac + ' | RSSI: ' + dev.rssi + ' dBm</div>';
        html+='<button class="small" onclick="connectDevice('+dev.id+')" style="margin-top:5px;">Połącz</button>';
        html+='</div>';
      });
      document.getElementById('devicesCard').style.display='block';
    } else {
      html='<div class="hint">Nie znaleziono urządzeń</div>';
    }
    document.getElementById('devicesList').innerHTML=html;
  }).catch(e=>{
    console.error('Error fetching devices:',e);
    document.getElementById('devicesList').innerHTML='<div class="hint" style="color:#e53935;">Błąd pobierania listy</div>';
  });
}

function connectDevice(id){
  fetch('/bt/api/connect?id='+id,{method:'POST'}).then(r=>r.text()).then(msg=>{
    alert(msg);
    refresh();
  });
}

function setMode(m){ fetch('/bt/api/mode?m='+encodeURIComponent(m),{method:'POST'}).then(()=>refresh()); }

function setVol(){
  let v=document.getElementById('volInput').value;
  fetch('/bt/api/vol?v='+encodeURIComponent(v),{method:'POST'}).then(()=>refresh());
}

function setBoost(b){ fetch('/bt/api/boost?b='+encodeURIComponent(b),{method:'POST'}).then(()=>refresh()); }

function toggleTerminal(enabled){
  terminalActive = enabled;
  console.log('BT Terminal:', enabled ? 'ENABLING' : 'DISABLING');
  fetch('/bt/api/terminal?enable='+(enabled?'1':'0'),{method:'POST'}).then(r=>{
    if(!r.ok) throw new Error('HTTP ' + r.status);
    return r.text();
  }).then(()=>{
    document.getElementById('terminalPanel').style.display = enabled ? 'block' : 'none';
    document.getElementById('terminalDisabled').style.display = enabled ? 'none' : 'block';
    document.getElementById('cmd').disabled = !enabled;
    document.getElementById('sendBtn').disabled = !enabled;
    if(!enabled){
      document.getElementById('console').textContent = '';
    }
    console.log('BT Terminal:', enabled ? 'ENABLED' : 'DISABLED');
  }).catch(e=>{
    console.error('Terminal toggle error:', e);
    alert('Błąd przełączania terminala: ' + e.message);
  });
}

function sendCmd(){
  let cmd=document.getElementById('cmd').value;
  if(!cmd) return;
  fetch('/bt/api/cmd?cmd='+encodeURIComponent(cmd),{method:'POST'}).then(()=>{
    document.getElementById('cmd').value='';
  });
}

function refresh(){
  fetch('/bt/api/state').then(r=>r.json()).then(s=>{
    let bar=document.getElementById('statusBar');
    if(s.btOn){ bar.innerText='Bluetooth ON'; bar.style.background='#2e7d32'; }
    else { bar.innerText='Bluetooth OFF'; bar.style.background='#9e9e9e'; }
    document.getElementById('modeTxt').innerText=s.mode;
    
    // Wyświetl urządzenie
    let devText = 'Brak połączenia';
    if(s.conn){
      if(s.name && s.name !== 'None' && s.name !== ''){
        devText = s.name + (s.mac && s.mac !== 'None' && s.mac !== '' ? ' (' + s.mac + ')' : '');
      } else if(s.mac && s.mac !== 'None' && s.mac !== ''){
        devText = s.mac;
      } else {
        devText = 'Połączono (brak danych)';
      }
    }
    let devElement = document.getElementById('devTxt');
    if(devElement){
      devElement.innerText = devText;
    } else {
      console.error('Element devTxt not found!');
    }
    
    document.getElementById('volInput').value=s.vol;
    
    // Synchronizuj stan checkboxa terminalu
    if(s.terminalEnabled !== undefined){
      terminalActive = s.terminalEnabled;
      document.getElementById('terminalEnable').checked = s.terminalEnabled;
      document.getElementById('terminalPanel').style.display = s.terminalEnabled ? 'block' : 'none';
      document.getElementById('terminalDisabled').style.display = s.terminalEnabled ? 'none' : 'block';
      document.getElementById('cmd').disabled = !s.terminalEnabled;
      document.getElementById('sendBtn').disabled = !s.terminalEnabled;
    }
  }).catch(e=>{
    console.error('Refresh state error:', e);
  });

  // Pobieraj logi tylko gdy terminal aktywny
  if(terminalActive){
    fetch('/bt/api/log', {
      method: 'GET',
      headers: {'Accept': 'text/plain'},
      cache: 'no-cache'
    }).then(r=>{
      if(!r.ok) throw new Error('HTTP ' + r.status);
      const ct = r.headers.get('Content-Type');
      if(ct && ct.includes('html')){
        console.error('BT Log endpoint returned HTML instead of text!');
        return '[ERROR] Otrzymano HTML zamiast logu. Sprawdź konsolę przeglądarki.';
      }
      return r.text();
    }).then(t=>{
      const c=document.getElementById('console');
      c.textContent=t;
      c.scrollTop=c.scrollHeight;
    }).catch(e=>{
      console.error('BT Log fetch error:', e);
      const c=document.getElementById('console');
      if(c.textContent.indexOf('[ERROR]') < 0){
        c.textContent += '\n[ERROR] Nie można pobrać logu: ' + e.message;
      }
    });
  }
}

function back(){
  fetch('/bt/api/back',{method:'POST'}).then(()=>{ window.location.href='/'; });
}

// Inicjalizacja przy ładowaniu DOM
window.addEventListener('DOMContentLoaded', function(){
  // Wymuś synchronizację stanu checkbox z serwera
  fetch('/bt/api/state').then(r=>r.json()).then(s=>{
    if(s.terminalEnabled !== undefined){
      terminalActive = s.terminalEnabled;
      document.getElementById('terminalEnable').checked = s.terminalEnabled;
      document.getElementById('terminalPanel').style.display = s.terminalEnabled ? 'block' : 'none';
      document.getElementById('terminalDisabled').style.display = s.terminalEnabled ? 'none' : 'block';
      document.getElementById('cmd').disabled = !s.terminalEnabled;
      document.getElementById('sendBtn').disabled = !s.terminalEnabled;
      console.log('Terminal state synchronized from server:', s.terminalEnabled);
    }
  }).catch(e=>{
    console.error('Failed to sync terminal state:', e);
  });
});

setInterval(refresh, 1200);
refresh();
</script>
</body>
</html>
)HTML";

class BTWebUI {
public:
    BTWebUI();
    void begin(AsyncWebServer* server, int rxPin = 19, int txPin = 20, uint32_t baud = 115200);
    void setExitCallback(std::function<void()> callback);
    void loop(); // Wywołuj w głównej pętli do obsługi UART
    
    // Publiczne metody kontrolne
    void sendCommand(const String& cmd);
    String getLastResponse() { return _lastResponse; }
    
private:
    // Struktura wykrytego urządzenia BT
    struct BTDevice {
        int id;
        String mac;
        int rssi;
        String name;
    };
    
    AsyncWebServer* _server;
    HardwareSerial* _btSerial;
    std::function<void()> _exitCallback;
    
    // Stan BT
    bool _btOn;
    String _mode; // "OFF", "RX", "TX", "AUTO"
    int _volume;
    int _boost;
    bool _connected;
    String _deviceName;
    String _deviceMac;
    
    // Wykryte urządzenia podczas SCAN
    std::vector<BTDevice> _scannedDevices;
    bool _isScanning;
    
    // Bufor konsoli
    String _consoleLog;
    static const int MAX_LOG_SIZE = 2000;
    
    // Bufor UART
    String _uartBuffer;
    String _lastResponse;
    unsigned long _lastCmdTime;
    unsigned long _lastStatusPoll;
    int _garbageLines;  // Licznik odrzuconych śmieci
    
    // Kontrola terminala
    bool _terminalEnabled;
    
    // Diagnostyka UART
    String _lastUartRxLines; // Ostatnie 5 odebranych linii
    unsigned long _lastUartRxTime;
    unsigned long _lastUartResponseTime; // Timestamp ostatniej odpowiedzi STATE/OK/PONG
    String _lastCommandSent;
    static const int MAX_DIAG_SIZE = 500;
    static const unsigned long UART_TIMEOUT_MS = 10000; // 10 sekund bez odpowiedzi = BT OFF
    
    void addToLog(const String& line);
    void processUartLine(const String& line);
    void parseStatusResponse(const String& line);
    
    // Handler functions
    void handleRoot(AsyncWebServerRequest *request);
    void handleState(AsyncWebServerRequest *request);
    void handleLog(AsyncWebServerRequest *request);
    void handleMode(AsyncWebServerRequest *request);
    void handleVol(AsyncWebServerRequest *request);
    void handleBoost(AsyncWebServerRequest *request);
    void handleScan(AsyncWebServerRequest *request);
    void handleDisconnect(AsyncWebServerRequest *request);
    void handleDelAll(AsyncWebServerRequest *request);
    void handleSave(AsyncWebServerRequest *request);
    void handleCmd(AsyncWebServerRequest *request);
    void handleBack(AsyncWebServerRequest *request);
    void handleTerminalEnable(AsyncWebServerRequest *request);
    void handleDevices(AsyncWebServerRequest *request);
    void handleConnect(AsyncWebServerRequest *request);
    void handleDiag(AsyncWebServerRequest *request);
};
