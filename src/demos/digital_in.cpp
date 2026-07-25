#include "config.h"

#if DEMO_DIGITAL_IN

#include <Arduino.h>
#include "board_pins.h"
#include "tools.h"

// Digital INPUT — button to GND with internal pull-up.
static DebouncedButton btn(Pins::DIG_IN);

void demoDigitalInSetup() {
  btn.begin();
  LOG("demo DIGITAL_IN ready (press D7->GND)");
}

void demoDigitalInLoop() {
  if (btn.pressed()) {
    LOG("DIGITAL_IN: button pressed");
  }
  static uint32_t last = 0;
  if (every(1000, last)) {
    Serial.print(F("DIGITAL_IN held="));
    Serial.println(btn.isDown() ? F("yes") : F("no"));
  }
}

#endif
