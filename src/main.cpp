#include <Arduino.h>
#include "config.h"
#include "board_pins.h"
#include "tools.h"
#include "demos.h"

void setup() {
  toolsBeginSerial(SERIAL_BAUD);
  toolsBanner(F("Arduino Nano boilerplate"));
  Serial.println(F("Board: ATmega328P @ 16MHz"));
  Serial.println(F("Edit include/config.h to enable demos."));
  Serial.println(F("Pin map: include/board_pins.h"));
  Serial.println(F("Helpers: include/tools.h"));
  demosPrintWiring();
  demosSetup();
  LOG("setup() done — entering loop()");
}

void loop() {
  demosLoop();

#if STATUS_INTERVAL_MS > 0
  static uint32_t lastStatus = 0;
  if (every(STATUS_INTERVAL_MS, lastStatus)) {
    LOGV("[status] uptime_ms=", millis());
  }
#endif
}
