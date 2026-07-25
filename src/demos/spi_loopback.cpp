#include "config.h"

#if DEMO_SPI_LOOPBACK

#include <Arduino.h>
#include <SPI.h>
#include "board_pins.h"
#include "tools.h"

// SPI — Nano: D10 SS, D11 MOSI, D12 MISO, D13 SCK (+ LED).
// Loopback: jumper MOSI↔MISO; master should read back what it wrote.
// Note: conflicts with DEMO_DIGITAL_OUT (also uses D13).

void demoSpiLoopbackSetup() {
  pinMode(Pins::SPI_SS, OUTPUT);
  digitalWrite(Pins::SPI_SS, HIGH);
  SPI.begin();
  LOG("demo SPI_LOOPBACK ready (jumper D11 <-> D12)");
}

void demoSpiLoopbackLoop() {
  static uint32_t last = 0;
  static uint8_t seq = 0;
  if (!every(1000, last)) return;

  const uint8_t tx = ++seq;
  digitalWrite(Pins::SPI_SS, LOW);
  const uint8_t rx = SPI.transfer(tx);
  digitalWrite(Pins::SPI_SS, HIGH);

  Serial.print(F("SPI tx=0x"));
  Serial.print(tx, HEX);
  Serial.print(F(" rx=0x"));
  Serial.print(rx, HEX);
  Serial.println(rx == tx ? F("  OK") : F("  FAIL (need MOSI-MISO jumper)"));
}

#endif
