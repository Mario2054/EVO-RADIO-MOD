#include "../core/options.h"

#ifdef USE_DLNA

#include "DLNAWebUI.h"
#include "dlna_service.h"
#include "dlna_worker.h"
#include "dlna_index.h"
#include <SPIFFS.h>

// HTML strona DLNA Browser
const char DLNAWebUI::DLNA_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>DLNA Browser</title>
<style>
  * { box-sizing: border-box; }
  body {
    font-family: Arial, sans-serif;
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    margin: 0;
    padding: 20px;
    min-height: 100vh;
  }
  .container {
    max-width: 800px;
    margin: 0 auto;
    background: #fff;
    border-radius: 15px;
    padding: 20px;
    box-shadow: 0 10px 40px rgba(0,0,0,0.3);
  }
  h1 {
    text-align: center;
    color: #333;
    margin: 0 0 20px 0;
  }
  .section {
    margin-bottom: 20px;
    padding: 15px;
    background: #f8f9fa;
    border-radius: 10px;
  }
  .section h2 {
    margin: 0 0 10px 0;
    color: #667eea;
    font-size: 18px;
  }
  select {
    width: 100%;
    padding: 10px;
    border: 2px solid #667eea;
    border-radius: 8px;
    font-size: 16px;
    background: white;
    cursor: pointer;
  }
  select:disabled {
    background: #e9ecef;
    cursor: not-allowed;
  }
  .btn-group {
    display: flex;
    gap: 10px;
    flex-wrap: wrap;
  }
  button {
    flex: 1;
    min-width: 150px;
    padding: 12px 20px;
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    color: white;
    border: none;
    border-radius: 8px;
    font-size: 16px;
    font-weight: 600;
    cursor: pointer;
    transition: all 0.3s;
  }
  button:hover:not(:disabled) {
    transform: translateY(-2px);
    box-shadow: 0 6px 20px rgba(102,126,234,0.6);
  }
  button:disabled {
    background: #ccc;
    cursor: not-allowed;
  }
  button.secondary {
    background: #6c757d;
  }
  button.secondary:hover:not(:disabled) {
    background: #5a6268;
  }
  .status {
    padding: 10px;
    border-radius: 8px;
    text-align: center;
    font-weight: 600;
    margin-top: 10px;
  }
  .status.info { background: #d1ecf1; color: #0c5460; }
  .status.success { background: #d4edda; color: #155724; }
  .status.error { background: #f8d7da; color: #721c24; }
  .status.warning { background: #fff3cd; color: #856404; }
  .back-link {
    text-align: center;
    margin-top: 20px;
  }
  .back-link a {
    color: #667eea;
    text-decoration: none;
    font-weight: 600;
  }
  .back-link a:hover {
    text-decoration: underline;
  }
  .info-box {
    padding: 10px;
    background: #e7f3ff;
    border-left: 4px solid #667eea;
    border-radius: 5px;
    margin-bottom: 15px;
    font-size: 14px;
    color: #555;
  }
</style>
</head>
<body>
<div class="container">
  <h1>🎵 DLNA Browser</h1>
  
  <div class="info-box">
    <strong>Instrukcja:</strong> Najpierw zainicjuj DLNA, następnie wybierz kategorię i przeglądaj zawartość. Możesz zbudować playlistę i przełączyć się na tryb DLNA.
  </div>

  <!-- INICJALIZACJA -->
  <div class="section">
    <h2>Inicjalizacja DLNA</h2>
    <button id="btnInit" onclick="initDLNA()">Inicjuj DLNA Server</button>
    <div id="statusInit" class="status" style="display:none;"></div>
  </div>

  <!-- KATEGORIE -->
  <div class="section">
    <h2>1) CATEGORY</h2>
    <select id="selectCategory" onchange="onCategoryChange()" disabled>
      <option value="">Wybierz kategorię...</option>
    </select>
    <div id="statusCategory" class="status" style="display:none;"></div>
  </div>

  <!-- ITEMS -->
  <div class="section">
    <h2>2) ITEMS</h2>
    <select id="selectItem" size="10" disabled>
      <option value="">Wybierz item...</option>
    </select>
    <div id="statusItem" class="status" style="display:none;"></div>
  </div>

  <!-- ACTIONS -->
  <div class="section">
    <h2>3) ACTIONS</h2>
    <div class="btn-group">
      <button id="btnUseDLNA" onclick="useDLNAPlaylist()" disabled>USE DLNA PL</button>
      <button id="btnUseWeb" onclick="useWebPlaylist()" disabled class="secondary">USE WEB PL</button>
    </div>
    <div id="statusAction" class="status" style="display:none;"></div>
  </div>

  <div class="back-link">
    <a href="/menu">⬅️ Powrót do Menu</a>
  </div>
</div>

<script>
let dlnaReady = false;
let currentCategoryId = '';
let categories = [];
let items = [];

// Polling helper - czeka aż worker skończy (busy=false)
async function pollUntilDone(maxMs, interval) {
  const deadline = Date.now() + maxMs;
  while (Date.now() < deadline) {
    await new Promise(r => setTimeout(r, interval));
    try {
      const resp = await fetch('/dlna/api/status');
      const data = await resp.json();
      if (!data.busy) return data;
    } catch(e) { /* continue on network error */ }
  }
  return { busy: false, ok: false, msg: 'timeout' };
}

// Inicjalizacja DLNA
async function initDLNA() {
  const btn = document.getElementById('btnInit');
  btn.disabled = true;
  showStatus('statusInit', 'info', 'Inicjalizacja DLNA...');
  try {
    const resp = await fetch('/dlna/api/init', { method: 'POST' });
    const data = await resp.json();
    if (!data.queued) {
      showStatus('statusInit', 'error', 'Błąd: ' + (data.err || 'nieznany'));
      btn.disabled = false;
      return;
    }
    showStatus('statusInit', 'info', 'Inicjalizacja w toku (może ~5–10s)...');
    const result = await pollUntilDone(20000, 500);
    if (result.ok) {
      dlnaReady = true;
      showStatus('statusInit', 'success', 'DLNA zainicjalizowany!');
      loadCategories();
    } else {
      showStatus('statusInit', 'error', 'Błąd: ' + (result.msg || 'nieznany'));
      btn.disabled = false;
    }
  } catch(e) {
    showStatus('statusInit', 'error', 'Błąd połączenia: ' + e.message);
    btn.disabled = false;
  }
}

// Załaduj kategorie
async function loadCategories() {
  const select = document.getElementById('selectCategory');
  showStatus('statusCategory', 'info', 'Ładowanie kategorii...');
  try {
    const resp = await fetch('/dlna/api/categories');
    const data = await resp.json();
    if (data.ok && data.items) {
      categories = data.items;
      select.innerHTML = '<option value="">Wybierz kategorię...</option>';
      data.items.forEach(cat => {
        const opt = document.createElement('option');
        opt.value = cat.id;
        opt.textContent = cat.title || cat.id;
        select.appendChild(opt);
      });
      select.disabled = false;
      showStatus('statusCategory', 'success', `Znaleziono ${data.items.length} kategorii`);
    } else {
      showStatus('statusCategory', 'error', 'Brak kategorii');
    }
  } catch(e) {
    showStatus('statusCategory', 'error', 'Błąd: ' + e.message);
  }
}

// Zmiana kategorii
async function onCategoryChange() {
  const select = document.getElementById('selectCategory');
  const categoryId = select.value;
  if (!categoryId) return;
  currentCategoryId = categoryId;
  await loadItems(categoryId);
}

// Załaduj items w kategorii
async function loadItems(categoryId) {
  const select = document.getElementById('selectItem');
  select.disabled = true;
  showStatus('statusItem', 'info', 'Ładowanie zawartości...');
  try {
    const resp = await fetch('/dlna/api/list?id=' + encodeURIComponent(categoryId));
    const data = await resp.json();
    if (!data.queued) {
      showStatus('statusItem', 'error', data.err || 'Błąd');
      return;
    }
    const status2 = await pollUntilDone(15000, 500);
    if (!status2.ok) {
      showStatus('statusItem', 'warning', 'Brak elementów: ' + (status2.msg || 'błąd'));
      return;
    }
    const rresp = await fetch('/dlna/api/listresult');
    const result = await rresp.json();
    if (result.ok && result.items) {
      items = result.items;
      select.innerHTML = '';
      result.items.forEach(item => {
        const opt = document.createElement('option');
        opt.value = item.id;
        const icon = item.type === 'container' ? '📁' : '🎵';
        opt.textContent = `${icon} ${item.title || item.id}`;
        opt.dataset.type = item.type;
        select.appendChild(opt);
      });
      select.disabled = false;
      showStatus('statusItem', 'success', `Znaleziono ${result.items.length} elementów`);
      document.getElementById('btnUseDLNA').disabled = false;
      document.getElementById('btnUseWeb').disabled = false;
    } else {
      showStatus('statusItem', 'warning', 'Brak elementów w tej kategorii');
    }
  } catch(e) {
    showStatus('statusItem', 'error', 'Błąd: ' + e.message);
  }
}

// USE DLNA PL - zbuduj playlistę i przełącz
async function useDLNAPlaylist() {
  if (!currentCategoryId) { showStatus('statusAction', 'warning', 'Wybierz najpierw kategorię!'); return; }
  showStatus('statusAction', 'info', 'Budowanie playlisty DLNA...');
  try {
    const resp = await fetch('/dlna/api/build', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: 'id=' + encodeURIComponent(currentCategoryId) + '&activate=1'
    });
    const data = await resp.json();
    if (!data.queued) { showStatus('statusAction', 'error', 'Błąd: ' + (data.err || 'nieznany')); return; }
    showStatus('statusAction', 'info', 'Budowanie playlisty (w toku)...');
    const result = await pollUntilDone(90000, 1000);
    if (result.ok) {
      try {
        const swResp = await fetch('/dlna/api/switch', { method: 'POST' });
        const swData = await swResp.json();
        showStatus('statusAction', swData.ok ? 'success' : 'warning',
          swData.ok ? 'Playlista zbudowana i przełączono na DLNA!' : ('Playlista ok, błąd aktywacji: ' + (swData.msg||'')));
      } catch(e) {
        showStatus('statusAction', 'warning', 'Playlista zbudowana, błąd przełączenia: ' + e.message);
      }
    } else {
      showStatus('statusAction', 'error', 'Błąd: ' + (result.msg || 'nieznany'));
    }
  } catch(e) {
    showStatus('statusAction', 'error', 'Błąd połączenia: ' + e.message);
  }
}

// USE WEB PL - tylko zbuduj playlistę (bez przełączenia)
async function useWebPlaylist() {
  if (!currentCategoryId) { showStatus('statusAction', 'warning', 'Wybierz najpierw kategorię!'); return; }
  showStatus('statusAction', 'info', 'Budowanie playlisty (bez przełączenia)...');
  try {
    const resp = await fetch('/dlna/api/build', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: 'id=' + encodeURIComponent(currentCategoryId) + '&activate=0'
    });
    const data = await resp.json();
    if (!data.queued) { showStatus('statusAction', 'error', 'Błąd: ' + (data.err || 'nieznany')); return; }
    showStatus('statusAction', 'info', 'Budowanie playlisty (w toku)...');
    const result = await pollUntilDone(90000, 1000);
    if (result.ok) {
      showStatus('statusAction', 'success', 'Playlista zbudowana (tryb nie zmieniony).');
    } else {
      showStatus('statusAction', 'error', 'Błąd: ' + (result.msg || 'nieznany'));
    }
  } catch(e) {
    showStatus('statusAction', 'error', 'Błąd połączenia: ' + e.message);
  }
}

// Helper do wyświetlania statusu
function showStatus(elemId, type, msg) {
  const elem = document.getElementById(elemId);
  elem.className = 'status ' + type;
  elem.textContent = msg;
  elem.style.display = 'block';
}
</script>
</body>
</html>
)rawliteral";

DLNAWebUI::DLNAWebUI() : _server(nullptr) {
}

void DLNAWebUI::begin(AsyncWebServer* server) {
    _server = server;
    
    if (!_server) return;
    
    Serial.println("[DLNAWebUI] Rejestrowanie endpointów...");
    
    // API endpoints - NAJPIERW (przed /dlna)
    _server->on("/dlna/api/init", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleInit(request);
    });
    
    _server->on("/dlna/api/categories", HTTP_GET, [this](AsyncWebServerRequest *request){
        this->handleCategories(request);
    });
    
    _server->on("/dlna/api/list", HTTP_GET, [this](AsyncWebServerRequest *request){
        this->handleList(request);
    });
    
    _server->on("/dlna/api/build", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleBuild(request);
    });
    
    _server->on("/dlna/api/switch", HTTP_POST, [this](AsyncWebServerRequest *request){
        this->handleSwitch(request);
    });
    
    _server->on("/dlna/api/status", HTTP_GET, [this](AsyncWebServerRequest *request){
        this->handleStatus(request);
    });

    _server->on("/dlna/api/listresult", HTTP_GET, [this](AsyncWebServerRequest *request){
        this->handleListResult(request);
    });
    
    // Główna strona - NA KOŃCU
    _server->on("/dlna", HTTP_GET, [this](AsyncWebServerRequest *request){
        this->handleRoot(request);
    });
    
    Serial.println("[DLNAWebUI] Endpointy zarejestrowane");
    Serial.println("  - /dlna - główna strona");
    Serial.println("  - /dlna/api/init - inicjalizacja");
    Serial.println("  - /dlna/api/categories - lista kategorii");
    Serial.println("  - /dlna/api/list - zawartość kategorii");
    Serial.println("  - /dlna/api/build - budowanie playlisty");
}

void DLNAWebUI::loop() {
    // Obecnie nic nie wymaga ciągłej obsługi
}

void DLNAWebUI::handleRoot(AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", DLNA_HTML);
}

void DLNAWebUI::handleInit(AsyncWebServerRequest *request) {
    Serial.println("[DLNAWebUI] /dlna/api/init wywołane");
    if (g_dlnaStatus.busy) {
        request->send(200, "application/json", "{\"ok\":false,\"err\":\"busy\"}");
        return;
    }
    extern String dlnaIDX;
    DlnaJob job{};
    job.type = DJ_INIT;
    strncpy(job.objectId, dlnaIDX.c_str(), sizeof(job.objectId)-1);
    job.reqId = dlna_next_reqId();
    job.hardLimit = 0;
    dlna_worker_enqueue(job);
    request->send(200, "application/json", "{\"ok\":true,\"queued\":true}");
}

void DLNAWebUI::handleCategories(AsyncWebServerRequest *request) {
    Serial.println("[DLNAWebUI] /dlna/api/categories wywołane");
    
    // Odczytaj plik z kategoriami (tworzony przez dlnaInit)
    File f = SPIFFS.open("/data/dlna_index.json", "r");
    if (!f) {
        request->send(200, "application/json", "{\"ok\":false,\"err\":\"Not initialized\"}");
        return;
    }
    
    String json = f.readString();
    f.close();
    
    request->send(200, "application/json", json);
}

void DLNAWebUI::handleList(AsyncWebServerRequest *request) {
    if (!request->hasParam("id")) {
        request->send(400, "application/json", "{\"ok\":false,\"err\":\"Missing id\"}");
        return;
    }
    
    String objectId = request->getParam("id")->value();
    Serial.printf("[DLNAWebUI] /dlna/api/list?id=%s\n", objectId.c_str());
    
    extern String g_dlnaControlUrl;
    
    if (!g_dlnaControlUrl.length()) {
        request->send(200, "application/json", "{\"ok\":false,\"err\":\"DLNA not initialized\"}");
        return;
    }
    
    if (g_dlnaStatus.busy) {
        request->send(200, "application/json", "{\"ok\":false,\"err\":\"busy\"}");
        return;
    }
    DlnaJob job{};
    job.type = DJ_LIST;
    strncpy(job.objectId, objectId.c_str(), sizeof(job.objectId)-1);
    job.reqId = dlna_next_reqId();
    job.hardLimit = 0;
    dlna_worker_enqueue(job);
    request->send(200, "application/json", "{\"ok\":true,\"queued\":true}");
}

void DLNAWebUI::handleBuild(AsyncWebServerRequest *request) {
    if (!request->hasParam("id", true)) {
        request->send(400, "application/json", "{\"ok\":false,\"err\":\"Missing id\"}");
        return;
    }
    
    String objectId = request->getParam("id", true)->value();
    bool activate = request->hasParam("activate", true) && 
                    request->getParam("activate", true)->value() == "1";
    
    Serial.printf("[DLNAWebUI] /dlna/api/build id=%s activate=%d\n", 
                  objectId.c_str(), activate);
    
    DlnaJob job{};
    job.type = DJ_BUILD;
    strncpy(job.objectId, objectId.c_str(), sizeof(job.objectId)-1);
    job.reqId = dlna_next_reqId();
    job.hardLimit = 5000;
    
    dlna_worker_enqueue(job);
    String respJson = "{\"ok\":true,\"queued\":true,\"activate\":";
    respJson += activate ? "true" : "false";
    respJson += "}";
    request->send(200, "application/json", respJson);
}

void DLNAWebUI::handleSwitch(AsyncWebServerRequest *request) {
    Serial.println("[DLNAWebUI] /dlna/api/switch wywołane");
    extern void activateDLNAMode();
    activateDLNAMode();
    request->send(200, "application/json", "{\"ok\":true,\"msg\":\"DLNA mode activated\"}");
}

void DLNAWebUI::handleStatus(AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"busy\":" + String(g_dlnaStatus.busy ? "true" : "false") + ",";
    json += "\"ok\":" + String(g_dlnaStatus.ok ? "true" : "false") + ",";
    json += "\"err\":" + String(g_dlnaStatus.err) + ",";
    json += "\"msg\":\"" + String(g_dlnaStatus.msg) + "\"";
    json += "}";
    
    request->send(200, "application/json", json);
}

void DLNAWebUI::handleListResult(AsyncWebServerRequest *request) {
    xSemaphoreTake(g_spiffsMux, portMAX_DELAY);
    String result = g_dlnaListResult.length() ? g_dlnaListResult : "{\"ok\":false,\"err\":\"No result\"}";
    xSemaphoreGive(g_spiffsMux);
    request->send(200, "application/json", result);
}

#endif // USE_DLNA
