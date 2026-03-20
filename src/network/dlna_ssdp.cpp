#include "../core/options.h"
#ifdef USE_DLNA
#include <WiFi.h>
#include "dlna_ssdp.h"

static const IPAddress SSDP_ADDR(239,255,255,250);
static const uint16_t SSDP_PORT = 1900;

bool DlnaSSDP::resolve(const String& expectedHost, String& outDescUrl) {
  outDescUrl = "";
  _udp.stop();

  // plain UDP socket (not multicast API)
  if (!_udp.begin(0)) {
    Serial.println("[DLNA][SSDP] UDP begin failed");
    return false;
  }

  sendSearch();

  unsigned long start = millis();
  unsigned long lastSend = start;
  String fallbackUrl = "";  // best found URL even if host doesn't match

  while (millis() - start < 5000) {
    // Re-send M-SEARCH every 1500 ms in case the first packet was lost
    if (millis() - lastSend >= 1500) {
      sendSearch();
      lastSend = millis();
    }

    String url;
    if (receiveResponse(url)) {
      if (expectedHost.length() == 0) {
        // No host filter — accept the first server found
        outDescUrl = url;
        _udp.stop();
        return true;
      }
      if (url.startsWith(String("http://") + expectedHost)) {
        // Exact host match
        outDescUrl = url;
        _udp.stop();
        return true;
      } else {
        // Server found but IP doesn't match the configured host
        Serial.printf("[DLNA][SSDP] host mismatch: expected %s, got %s\n",
                      expectedHost.c_str(), url.c_str());
        if (fallbackUrl.length() == 0) fallbackUrl = url;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Fallback: use any discovered MediaServer even if the IP didn't match
  // (covers the case where dlnaHost is misconfigured or still at default)
  if (fallbackUrl.length() > 0) {
    Serial.printf("[DLNA][SSDP] using fallback server (host mismatch): %s\n",
                  fallbackUrl.c_str());
    outDescUrl = fallbackUrl;
    _udp.stop();
    return true;
  }

  _udp.stop();
  Serial.println("[DLNA][SSDP] timeout — no MediaServer found on network");

  // Fallback: unicast M-SEARCH directly to configured host (works across subnets)
  if (expectedHost.length() > 0) {
    Serial.printf("[DLNA][SSDP] Trying unicast M-SEARCH to %s:1900...\n", expectedHost.c_str());
    if (_udp.begin(0)) {
      IPAddress targetIP;
      if (targetIP.fromString(expectedHost)) {
        for (int attempt = 0; attempt < 3; attempt++) {
          sendSearchUnicast(targetIP);
          unsigned long uStart = millis();
          while (millis() - uStart < 2000) {
            String url;
            if (receiveResponse(url)) {
              outDescUrl = url;
              _udp.stop();
              Serial.printf("[DLNA][SSDP] unicast response: %s\n", url.c_str());
              return true;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
          }
        }
      }
      _udp.stop();
    }
    Serial.println("[DLNA][SSDP] unicast fallback also failed");
  }

  return false;
}

bool DlnaSSDP::sendSearchUnicast(const IPAddress& targetIP) {
  const char* msg =
    "M-SEARCH * HTTP/1.1\r\n"
    "HOST: 239.255.255.250:1900\r\n"
    "MAN: \"ssdp:discover\"\r\n"
    "MX: 1\r\n"
    "ST: urn:schemas-upnp-org:device:MediaServer:1\r\n"
    "\r\n";

  _udp.beginPacket(targetIP, SSDP_PORT);
  _udp.write((const uint8_t*)msg, strlen(msg));
  _udp.endPacket();
  return true;
}

bool DlnaSSDP::sendSearch() {
  const char* msg =
    "M-SEARCH * HTTP/1.1\r\n"
    "HOST: 239.255.255.250:1900\r\n"
    "MAN: \"ssdp:discover\"\r\n"
    "MX: 2\r\n"
    "ST: urn:schemas-upnp-org:device:MediaServer:1\r\n"
    "\r\n";

  _udp.beginPacket(SSDP_ADDR, SSDP_PORT);
  _udp.write((const uint8_t*)msg, strlen(msg));
  _udp.endPacket();

  Serial.println("[DLNA][SSDP] M-SEARCH sent");
  return true;
}

bool DlnaSSDP::receiveResponse(String& outUrl) {
  int size = _udp.parsePacket();
  if (!size) return false;

  // Fixed buffer avoids String concatenation heap fragmentation
  static char buf[1600];
  int n = 0;
  while (_udp.available() && n < (int)sizeof(buf) - 1) {
    buf[n++] = (char)_udp.read();
  }
  buf[n] = 0;
  String response(buf);

  if (!response.startsWith("HTTP/1.1 200")) return false;

  String url;
  if (!parseLocation(response, url)) return false;
  if (url.length() == 0) return false;

  outUrl = url;
  Serial.printf("[DLNA][SSDP] FOUND: %s\n", outUrl.c_str());
  return true;
}

bool DlnaSSDP::parseLocation(const String& response, String& outUrl) {
/*  int idx = response.indexOf("\nLOCATION:");
  if (idx < 0) idx = response.indexOf("\nLocation:");*/

  // case-insensitive keresés durván, de stabilan
  int idx = response.indexOf("\nLOCATION:");
  if (idx < 0) idx = response.indexOf("\nLocation:");
  if (idx < 0) idx = response.indexOf("\nlocation:");

  if (idx < 0) return false;

  int start = response.indexOf(":", idx) + 1;
  int end   = response.indexOf("\r", start);
  if (start <= 0 || end <= start) return false;

  outUrl = response.substring(start, end);
  outUrl.trim();
  return true;
}
#endif // USE_DLNA