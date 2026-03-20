#pragma once

#include <Arduino.h>
#include <WiFiUdp.h>

class DlnaSSDP {
public:
  bool resolve(const String& expectedHost, String& outDescUrl);

private:
  WiFiUDP _udp;

  bool sendSearch();
  bool sendSearchUnicast(const IPAddress& targetIP);
  bool receiveResponse(String& outUrl);  // host filtering moved to resolve()
  bool parseLocation(const String& response, String& outUrl);
};
