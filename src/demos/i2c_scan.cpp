#include "config.h"

#if DEMO_I2C_SCAN

#include <Arduino.h>
#include <Wire.h>
#include "board_pins.h"
#include "tools.h"

// I2C — Nano fixed pins: A4=SDA, A5=SCL. Scan for 7-bit addresses.

void demoI2cScanSetup() {
  Wire.begin();  // SDA=A4, SCL=A5
  LOG("demo I2C_SCAN ready (A4/A5)");
}

void demoI2cScanLoop() {
  static uint32_t last = 0;
  if (!every(3000, last)) return;

  uint8_t found = 0;
  Serial.println(F("I2C scan:"));
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    const uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print(F("  device @ 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (found == 0) {
    LOG("  (none — check wiring / 4.7k pull-ups)");
  } else {
    LOGV("  count=", found);
  }
}

#endif
