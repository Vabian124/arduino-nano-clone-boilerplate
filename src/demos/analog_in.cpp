#include "config.h"

#if DEMO_ANALOG_IN

#include <Arduino.h>
#include "board_pins.h"
#include "tools.h"

// Analog INPUT — ADC 0–1023 on A0–A7 (A6/A7 are ADC-only on Nano).
// Example: potentiometer wiper → A0, other ends → 5V and GND.

void demoAnalogInSetup() {
  // analog pins default to input; no pinMode required for ADC
  analogReference(DEFAULT);  // 5V (use INTERNAL for 1.1V)
  LOG("demo ANALOG_IN ready (read A0)");
}

void demoAnalogInLoop() {
  static uint32_t last = 0;
  if (!every(250, last)) return;

  const int raw = analogRead(Pins::ADC_POT);          // 0..1023
  const float volts = raw * (5.0f / 1023.0f);
  const int pct = (int)fmap(raw, 0, 1023, 0, 100);

  Serial.print(F("ANALOG A0 raw="));
  Serial.print(raw);
  Serial.print(F("  V="));
  Serial.print(volts, 2);
  Serial.print(F("  %="));
  Serial.println(pct);
}

#endif
