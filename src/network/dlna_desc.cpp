#include "../core/options.h"
#ifdef USE_DLNA

#include "dlna_desc.h"
#include "dlna_http_guard.h"
#include <WiFiClient.h>
#include <HTTPClient.h>

bool DlnaDescription::resolveControlURL(const String& descUrl, String& outControlUrl) {
  outControlUrl = "";

  HTTPClient http;
  WiFiClient client;

  Serial.printf("[DLNA] GET %s\n", descUrl.c_str());

  DlnaHttpGuard lock;

  http.setTimeout(8000);

  if (!http.begin(client, descUrl)) {
    Serial.println("[DLNA] HTTP begin failed");
    return false;
  }

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("[DLNA] HTTP error: %d\n", code);
    http.end();
    return false;
  }

  String xml = http.getString();
  http.end();

  if (xml.length() == 0) {
    Serial.println("[DLNA] Empty XML");
    return false;
  }

  // 🔍 Egyszerű ContentDirectory keresés
  int svc = xml.indexOf("urn:schemas-upnp-org:service:ContentDirectory:1");
  if (svc < 0) {
    Serial.println("[DLNA] ContentDirectory not found");
    return false;
  }

  int ctrlStart = xml.indexOf("<controlURL>", svc);
  if (ctrlStart < 0) {
    Serial.println("[DLNA] controlURL tag not found");
    return false;
  }

  ctrlStart += strlen("<controlURL>");
  int ctrlEnd = xml.indexOf("</controlURL>", ctrlStart);
  if (ctrlEnd < 0) {
    Serial.println("[DLNA] controlURL end tag not found");
    return false;
  }

  String controlPath = xml.substring(ctrlStart, ctrlEnd);

  String base = extractBaseUrl(descUrl);
  outControlUrl = base + controlPath;

  return true;
}

bool parseString(const String& xml, String& controlPath) {
  if (xml.indexOf("ContentDirectory") < 0) return false;

  int p = xml.indexOf("<controlURL>");
  if (p < 0) return false;

  int start = p + 12;
  int end = xml.indexOf("</controlURL>", start);
  if (end < 0) return false;

  controlPath = xml.substring(start, end);
  return true;
}

bool DlnaDescription::parseStream(Stream& s, String& outControlPath) {
  bool inService = false;
  bool isContentDir = false;

  String line;
  while (s.available()) {
    line = s.readStringUntil('\n');
    line.trim();

    if (line.indexOf("<service>") >= 0) {
      inService = true;
      isContentDir = false;
    }

    // EXAKT serviceType ellenőrzés
    if (inService &&
        line.indexOf("<serviceType>urn:schemas-upnp-org:service:ContentDirectory:1</serviceType>") >= 0) {
      isContentDir = true;
    }

    if (inService && isContentDir &&
        line.indexOf("<controlURL>") >= 0) {
      int a = line.indexOf("<controlURL>") + 12;
      int b = line.indexOf("</controlURL>");
      if (b > a) {
        outControlPath = line.substring(a, b);
        outControlPath.trim();
        return true;
      }
    }

    if (line.indexOf("</service>") >= 0) {
      inService = false;
      isContentDir = false;
    }
  }
  return false;
}

String DlnaDescription::extractBaseUrl(const String& fullUrl) {
  // http://IP:PORT/desc/device.xml → http://IP:PORT
  int idx = fullUrl.indexOf('/', 8); // 8 = after http://
  if (idx > 0) return fullUrl.substring(0, idx);
  return fullUrl;
}
#endif // USE_DLNA