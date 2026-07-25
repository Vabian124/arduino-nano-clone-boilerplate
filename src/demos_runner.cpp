#include "demos.h"
#include "tools.h"

void demosPrintWiring() {
  LOG("Enabled demos / wiring:");
#if DEMO_DIGITAL_OUT
  LOG("  DIGITAL_OUT  LED on D13 (onboard)");
#endif
#if DEMO_DIGITAL_IN
  LOG("  DIGITAL_IN   button D7 -> GND  (INPUT_PULLUP)");
#endif
#if DEMO_PWM
  LOG("  PWM          LED/resistor on D5 (~PWM)");
#endif
#if DEMO_ANALOG_IN
  LOG("  ANALOG_IN    pot wiper on A0, ends to 5V and GND");
#endif
#if DEMO_INTERRUPT
  LOG("  INTERRUPT    button D2 -> GND  (INPUT_PULLUP, INT0)");
#endif
#if DEMO_TONE
  LOG("  TONE         piezo + on D8, - to GND");
#endif
#if DEMO_PULSE_IN
  LOG("  PULSE_IN     feed a pulse into D7 (or jumper from D4)");
#endif
#if DEMO_I2C_SCAN
  LOG("  I2C_SCAN     devices on A4(SDA)/A5(SCL), 4.7k pull-ups to 5V");
#endif
#if DEMO_SPI_LOOPBACK
  LOG("  SPI_LOOPBACK jumper D11(MOSI) <-> D12(MISO)");
#endif
#if DEMO_LED_RING
  LOG("  LED_RING     WCMCU-2812B-24: 5V IN + GND, DIN -> D3 (DOUT unused)");
#endif
#if DEMO_RF_RX
  LOG("  RF_RX+LIGHT  remote Dout->D2, LED ring DIN->D3, 5V+GND");
#endif
}

void demosSetup() {
#if DEMO_DIGITAL_OUT
  demoDigitalOutSetup();
#endif
#if DEMO_DIGITAL_IN
  demoDigitalInSetup();
#endif
#if DEMO_PWM
  demoPwmSetup();
#endif
#if DEMO_ANALOG_IN
  demoAnalogInSetup();
#endif
#if DEMO_INTERRUPT
  demoInterruptSetup();
#endif
#if DEMO_TONE
  demoToneSetup();
#endif
#if DEMO_PULSE_IN
  demoPulseInSetup();
#endif
#if DEMO_I2C_SCAN
  demoI2cScanSetup();
#endif
#if DEMO_SPI_LOOPBACK
  demoSpiLoopbackSetup();
#endif
#if DEMO_LED_RING
  demoLedRingSetup();
#endif
#if DEMO_RF_RX
  demoRfRxSetup();
#endif
}

void demosLoop() {
#if DEMO_DIGITAL_OUT
  demoDigitalOutLoop();
#endif
#if DEMO_DIGITAL_IN
  demoDigitalInLoop();
#endif
#if DEMO_PWM
  demoPwmLoop();
#endif
#if DEMO_ANALOG_IN
  demoAnalogInLoop();
#endif
#if DEMO_INTERRUPT
  demoInterruptLoop();
#endif
#if DEMO_TONE
  demoToneLoop();
#endif
#if DEMO_PULSE_IN
  demoPulseInLoop();
#endif
#if DEMO_I2C_SCAN
  demoI2cScanLoop();
#endif
#if DEMO_SPI_LOOPBACK
  demoSpiLoopbackLoop();
#endif
#if DEMO_LED_RING
  demoLedRingLoop();
#endif
#if DEMO_RF_RX
  demoRfRxLoop();
#endif
}
