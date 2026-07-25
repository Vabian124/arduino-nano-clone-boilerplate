#pragma once
#include "config.h"

void demosPrintWiring();
void demosSetup();
void demosLoop();

#if DEMO_DIGITAL_OUT
void demoDigitalOutSetup();
void demoDigitalOutLoop();
#endif

#if DEMO_DIGITAL_IN
void demoDigitalInSetup();
void demoDigitalInLoop();
#endif

#if DEMO_PWM
void demoPwmSetup();
void demoPwmLoop();
#endif

#if DEMO_ANALOG_IN
void demoAnalogInSetup();
void demoAnalogInLoop();
#endif

#if DEMO_INTERRUPT
void demoInterruptSetup();
void demoInterruptLoop();
#endif

#if DEMO_TONE
void demoToneSetup();
void demoToneLoop();
#endif

#if DEMO_PULSE_IN
void demoPulseInSetup();
void demoPulseInLoop();
#endif

#if DEMO_I2C_SCAN
void demoI2cScanSetup();
void demoI2cScanLoop();
#endif

#if DEMO_SPI_LOOPBACK
void demoSpiLoopbackSetup();
void demoSpiLoopbackLoop();
#endif
