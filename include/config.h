#pragma once

/*
 * Boilerplate config — flip demos on/off before upload.
 * Safe defaults need only USB (no extra wiring).
 *
 * Wiring for optional demos is printed at boot when enabled.
 */

#define SERIAL_BAUD 9600

// ---- Demos (1 = on, 0 = off) ----
#define DEMO_DIGITAL_OUT   1  // blink Pins::LED (D13)
#define DEMO_DIGITAL_IN    0  // button on Pins::DIG_IN (D7) → GND
#define DEMO_PWM           0  // LED/motor on Pins::PWM_A (D5)
#define DEMO_ANALOG_IN     0  // pot/sensor on Pins::ADC_POT (A0)
#define DEMO_INTERRUPT     0  // button on Pins::INT0_PIN (D2) → GND
#define DEMO_TONE          0  // piezo on Pins::TONE_PIN (D8)
#define DEMO_PULSE_IN      0  // echo pulse on Pins::DIG_IN (D7)
#define DEMO_I2C_SCAN      0  // Wire scan on A4/A5 (needs pull-ups / devices)
#define DEMO_SPI_LOOPBACK  0  // jumper MOSI(D11)↔MISO(D12)

// Heartbeat line every N ms when at least one demo runs (0 = off)
#define STATUS_INTERVAL_MS 2000
