#include "../core/options.h"
#ifdef USE_DLNA

#include "dlna_service.h"
#include "dlna_ssdp.h"
#include "dlna_desc.h"
#include "dlna_index.h"
#include "dlna_worker.h"

extern String dlnaHost;  // from main.cpp (changed to String for runtime config)
extern void saveDLNAConfig();  // from main.cpp - zapisuje host do pliku na SD

String g_dlnaControlUrl;

static bool s_serviceStarted = false;
static uint32_t s_reqId = 1;
bool g_dlnaReady = false;

bool dlnaInit(const String& rootObjectId, String& err) {

  g_dlnaReady = false;
  g_dlnaControlUrl = "";

  // player.sendCommand({PR_STOP, 0});  // Not used in this project

  DlnaSSDP ssdp;
  DlnaDescription desc;
  DlnaIndex idx;

  String descUrl;
  String controlUrl;

  uint32_t lastYield = millis();

  if (!ssdp.resolve(dlnaHost, descUrl)) {
    err = "SSDP discover failed – no MediaServer on network";
    return false;
  }

  // Auto-update dlnaHost if discovery found a different server (misconfigured or default IP)
  if (descUrl.startsWith("http://")) {
    String h = descUrl.substring(7);
    int colon = h.indexOf(':');
    int slash = h.indexOf('/');
    int end = (colon >= 0 && (slash < 0 || colon < slash)) ? colon :
              (slash >= 0) ? slash : h.length();
    if (end > 0) {
      String discoveredHost = h.substring(0, end);
      if (discoveredHost != dlnaHost) {
        Serial.printf("[DLNA] Auto-updating dlnaHost: %s -> %s\n",
                      dlnaHost.c_str(), discoveredHost.c_str());
        dlnaHost = discoveredHost;
        saveDLNAConfig();  // Zapisz nowy host do pliku na SD - nie zgub po restarcie
      }
    }
  }

  if (millis() - lastYield > 50) {
    vTaskDelay(1);
    lastYield = millis();
  }

  bool cdOk = false;
  for (int i = 0; i < 3; i++) {
    if (desc.resolveControlURL(descUrl, controlUrl)) {
      cdOk = true;
      break;
    }
    // Nie powtarzaj przy bledach 4xx - sa permanentne (403=brak uprawnien, 404=zly URL)
    if (controlUrl == "403" || controlUrl == "404") break;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  if (!cdOk || !controlUrl.length()) {
    err = "ContentDirectory not found";
    Serial.println("[DLNA] ERROR: ContentDirectory control URL not resolved");
    return false;
  }

  Serial.printf("[DLNA] ContentDirectory control URL: %s\n", controlUrl.c_str());

  if (!idx.buildContainerIndex(controlUrl, rootObjectId)) {
    err = "Root container browse failed";
    return false;
  }

  // biztos yield a végén is
  vTaskDelay(1);

  g_dlnaControlUrl = controlUrl;
  g_dlnaReady = true;

  return true;
}

void dlna_service_begin() {
  if (s_serviceStarted) return;
  s_serviceStarted = true;

  dlna_worker_start();
}

uint32_t dlna_next_reqId() {
  uint32_t v = s_reqId++;
  if (v == 0) v = s_reqId++; // 0 ne legyen
  return v;
}

bool dlna_isBusy() {
  return g_dlnaStatus.busy;
}

#endif   // USE_DLNA
