#include "config.h"

#if DEMO_DIGITAL_OUT

#include <Arduino.h>
#include "board_pins.h"
#include "tools.h"

// Digital OUTPUT — drive a pin HIGH/LOW (onboard LED on D13).
static SoftBlink led(Pins::LED, 180, 180);

void demoDigitalOutSetup() {
  led.begin();
  LOG("demo DIGITAL_OUT ready (D13 blink, non-blocking)");
}

void demoDigitalOutLoop() {
  led.update();
}

#endif
