#include "config.h"

#if DEMO_PULSE_IN

#include <Arduino.h>
#include "board_pins.h"
#include "tools.h"

// pulseIn — measure HIGH/LOW pulse width (µs). Blocking up to timeout.
// Demo: generate a pulse on D4, measure on D7 (jumper D4→D7).

void demoPulseInSetup() {
  pinMode(Pins::DIG_OUT, OUTPUT);
  digitalWrite(Pins::DIG_OUT, LOW);
  pinMode(Pins::DIG_IN, INPUT);
  LOG("demo PULSE_IN ready (jumper D4 -> D7)");
}

void demoPulseInLoop() {
  static uint32_t last = 0;
  if (!every(1000, last)) return;

  // ~500 µs HIGH pulse
  digitalWrite(Pins::DIG_OUT, HIGH);
  delayMicroseconds(500);
  digitalWrite(Pins::DIG_OUT, LOW);

  const unsigned long us = pulseIn(Pins::DIG_IN, HIGH, 20000UL);
  if (us == 0) {
    LOG("PULSE_IN timeout (check jumper D4->D7)");
  } else {
    LOGV("PULSE_IN width_us=", us);
  }
}

#endif
