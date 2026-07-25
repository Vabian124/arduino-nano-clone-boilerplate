#include "config.h"

#if DEMO_TONE

#include <Arduino.h>
#include "board_pins.h"
#include "tools.h"

// TONE — square wave on any digital pin (uses Timer2 on AVR).
// Piezo + → D8, − → GND. Conflicts with PWM on some pins/timers.

void demoToneSetup() {
  pinMode(Pins::TONE_PIN, OUTPUT);
  LOG("demo TONE ready (beep pattern on D8)");
}

void demoToneLoop() {
  static uint32_t last = 0;
  static uint8_t step = 0;
  if (!every(400, last)) return;

  // short melody loop
  static const uint16_t notes[] = {262, 294, 330, 349, 0};
  const uint16_t f = notes[step];
  step = (step + 1) % (sizeof(notes) / sizeof(notes[0]));

  if (f == 0) {
    noTone(Pins::TONE_PIN);
    LOG("TONE rest");
  } else {
    tone(Pins::TONE_PIN, f, 350);
    LOGV("TONE Hz=", f);
  }
}

#endif
