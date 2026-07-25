#pragma once
#include <Arduino.h>

// ---- Serial helpers (F() strings save RAM) ----
#define LOG(msg)       do { Serial.println(F(msg)); } while (0)
#define LOGV(msg, v)   do { Serial.print(F(msg)); Serial.println(v); } while (0)
#define LOGF(msg, v)   do { Serial.print(F(msg)); Serial.println(v, 3); } while (0)

void toolsBeginSerial(uint32_t baud = 9600);
void toolsBanner(const __FlashStringHelper* title);

// Non-blocking "every N ms" timer (millis-based)
bool every(uint32_t intervalMs, uint32_t& lastMs);

// Map float (like Arduino map, but float)
float fmap(float x, float inMin, float inMax, float outMin, float outMax);

template <typename T>
T clamp(T v, T lo, T hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Simple debounce for active-LOW buttons (INPUT_PULLUP)
class DebouncedButton {
 public:
  explicit DebouncedButton(uint8_t pin, uint16_t debounceMs = 25)
      : pin_(pin), debounceMs_(debounceMs) {}

  void begin() {
    pinMode(pin_, INPUT_PULLUP);
    stable_ = digitalRead(pin_);
    lastRaw_ = stable_;
    lastChange_ = millis();
  }

  // Call each loop; returns true on press edge (HIGH→LOW)
  bool pressed() {
    update();
    if (fell_) {
      fell_ = false;
      return true;
    }
    return false;
  }

  bool isDown() {
    update();
    return stable_ == LOW;
  }

 private:
  void update() {
    const uint8_t raw = digitalRead(pin_);
    const uint32_t now = millis();
    if (raw != lastRaw_) {
      lastRaw_ = raw;
      lastChange_ = now;
    } else if ((now - lastChange_) >= debounceMs_ && raw != stable_) {
      stable_ = raw;
      if (stable_ == LOW) fell_ = true;
    }
  }

  uint8_t pin_;
  uint16_t debounceMs_;
  uint8_t lastRaw_ = HIGH;
  uint8_t stable_ = HIGH;
  uint32_t lastChange_ = 0;
  bool fell_ = false;
};

// Soft blink without delay()
class SoftBlink {
 public:
  SoftBlink(uint8_t pin, uint16_t onMs = 200, uint16_t offMs = 200)
      : pin_(pin), onMs_(onMs), offMs_(offMs) {}

  void begin() {
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);
    last_ = millis();
    on_ = false;
  }

  void update() {
    const uint32_t now = millis();
    const uint16_t wait = on_ ? onMs_ : offMs_;
    if (now - last_ < wait) return;
    last_ = now;
    on_ = !on_;
    digitalWrite(pin_, on_ ? HIGH : LOW);
  }

 private:
  uint8_t pin_;
  uint16_t onMs_, offMs_;
  uint32_t last_ = 0;
  bool on_ = false;
};
