#pragma once
#include <Arduino.h>

/*
 * Arduino Nano (ATmega328P) pin map — quick reference
 *
 *  DIGITAL
 *  -------
 *  D0  RX0     Serial RX  (USB) — avoid if using Serial
 *  D1  TX0     Serial TX  (USB) — avoid if using Serial
 *  D2  INT0    external interrupt 0, digital I/O
 *  D3  INT1    external interrupt 1, PWM (~), digital I/O
 *  D4          digital I/O
 *  D5  PWM     (~), digital I/O
 *  D6  PWM     (~), digital I/O
 *  D7          digital I/O
 *  D8          digital I/O
 *  D9  PWM     (~), digital I/O
 *  D10 PWM/SS  (~), SPI Slave Select, digital I/O
 *  D11 PWM/MOSI(~), SPI MOSI, digital I/O
 *  D12 MISO    SPI MISO, digital I/O
 *  D13 SCK/LED SPI SCK + built-in LED
 *
 *  ANALOG (also usable as digital D14–D21)
 *  --------------------------------------
 *  A0–A3       ADC + digital
 *  A4  SDA     I2C data  (+ ADC / digital)
 *  A5  SCL     I2C clock (+ ADC / digital)
 *  A6, A7      ADC only (no digital I/O on Nano)
 *
 *  PWM pins:  3, 5, 6, 9, 10, 11   (analogWrite 0–255)
 *  Interrupts: 2 (INT0), 3 (INT1)
 *  SPI:       10 SS, 11 MOSI, 12 MISO, 13 SCK
 *  I2C:       A4 SDA, A5 SCL
 *  Tone:      any digital pin (prefer non-PWM if you need PWM elsewhere)
 */

// --- Named aliases for demos / your project ---
namespace Pins {
  // UART (USB Serial) — leave alone while debugging
  constexpr uint8_t UART_RX = 0;
  constexpr uint8_t UART_TX = 1;

  // Interrupts
  constexpr uint8_t INT0_PIN = 2;
  constexpr uint8_t INT1_PIN = 3;

  // PWM examples
  constexpr uint8_t PWM_A = 5;   // ~D5
  constexpr uint8_t PWM_B = 6;   // ~D6
  constexpr uint8_t PWM_C = 9;   // ~D9

  // Plain digital
  constexpr uint8_t DIG_OUT = 4;
  constexpr uint8_t DIG_IN  = 7;   // button → GND with INPUT_PULLUP
  constexpr uint8_t TONE_PIN = 8;  // piezo / buzzer (+)

  // WS2812B / NeoPixel data (WCMCU-2812B-24 ring DIN)
  constexpr uint8_t LED_RING = 4;  // D4

  // SPI
  constexpr uint8_t SPI_SS   = 10;
  constexpr uint8_t SPI_MOSI = 11;
  constexpr uint8_t SPI_MISO = 12;
  constexpr uint8_t SPI_SCK  = 13;

  // Onboard
  constexpr uint8_t LED = LED_BUILTIN; // 13 (shares SPI SCK)

  // Analog
  constexpr uint8_t ADC_POT   = A0;  // pot / sensor 0–5V
  // RCSwitch needs INT0/INT1 → use D3 (INT1). Do not use A0/A3.
  constexpr uint8_t RF_DATA   = 3;
  constexpr uint8_t ADC_EXTRA = A1;
  constexpr uint8_t I2C_SDA   = A4;
  constexpr uint8_t I2C_SCL   = A5;
  constexpr uint8_t ADC_ONLY6 = A6;  // ADC only
  constexpr uint8_t ADC_ONLY7 = A7;  // ADC only
}
