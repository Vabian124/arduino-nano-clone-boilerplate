#include "config.h"

#if DEMO_PWM

#include <Arduino.h>
#include "board_pins.h"
#include "tools.h"

// PWM OUTPUT — analogWrite on ~ pins: 3,5,6,9,10,11 (0–255).
// Hook LED+resistor (or logic-level device) to Pins::PWM_A (D5).

void demoPwmSetup() {
  pinMode(Pins::PWM_A, OUTPUT);
  LOG("demo PWM ready (fade on D5)");
}

void demoPwmLoop() {
  // Triangle fade ~2s period, non-blocking via millis phase
  const uint32_t period = 2000;
  const uint32_t t = millis() % period;
  uint8_t duty;
  if (t < period / 2) {
    duty = (uint8_t)map(t, 0, period / 2, 0, 255);
  } else {
    duty = (uint8_t)map(t, period / 2, period, 255, 0);
  }
  analogWrite(Pins::PWM_A, duty);

  static uint32_t last = 0;
  if (every(500, last)) {
    LOGV("PWM D5 duty=", duty);
  }
}

#endif
