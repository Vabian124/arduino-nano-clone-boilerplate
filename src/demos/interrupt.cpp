#include "config.h"

#if DEMO_INTERRUPT

#include <Arduino.h>
#include "board_pins.h"
#include "tools.h"

// External INTERRUPT — Nano: D2=INT0, D3=INT1 only.
// Button D2 → GND with INPUT_PULLUP; ISR counts falling edges.

static volatile uint16_t g_irqCount = 0;

static void onInt0() {
  g_irqCount++;
}

void demoInterruptSetup() {
  pinMode(Pins::INT0_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Pins::INT0_PIN), onInt0, FALLING);
  LOG("demo INTERRUPT ready (pulse/button on D2)");
}

void demoInterruptLoop() {
  static uint32_t last = 0;
  if (!every(1000, last)) return;

  noInterrupts();
  const uint16_t n = g_irqCount;
  g_irqCount = 0;
  interrupts();

  LOGV("INTERRUPT edges/sec~=", n);
}

#endif
