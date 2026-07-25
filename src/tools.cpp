#include "tools.h"

void toolsBeginSerial(uint32_t baud) {
  Serial.begin(baud);
  // AVR USB-serial adapters don't need this; keep a short wait for monitors.
  const uint32_t start = millis();
  while (!Serial && (millis() - start) < 1500) {
  }
}

void toolsBanner(const __FlashStringHelper* title) {
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(title);
  Serial.println(F("========================================"));
}

bool every(uint32_t intervalMs, uint32_t& lastMs) {
  const uint32_t now = millis();
  if (now - lastMs < intervalMs) return false;
  lastMs = now;
  return true;
}

float fmap(float x, float inMin, float inMax, float outMin, float outMax) {
  if (inMax == inMin) return outMin;
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
