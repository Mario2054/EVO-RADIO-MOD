#pragma once

#include <Arduino.h>
#include <WiFiUdp.h>

class DlnaSSDP {
public:
  bool resolve(const String& expectedHost, String& outDescUrl);

private:
  WiFiUDP _udp;

  bool sendSearch();
  bool receiveResponse(const String& expectedHost, String& outDescUrl);
  bool parseLocation(const String& response, String& outUrl);
};
