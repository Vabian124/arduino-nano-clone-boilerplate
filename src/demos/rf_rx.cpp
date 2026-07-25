#include "config.h"

#if DEMO_RF_RX

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "board_pins.h"
#include "tools.h"

/*
 * Remote Dout → D2, LED ring DIN → D3
 *
 * Keys (nibble bits OR together):
 *   0x1 RED | 0x2 BLUE | 0x4 YELLOW | 0x8 GREEN
 *
 * Ring quarters rotated +180° then left 45° from the original map
 * (LED index offset +9 on a 24-LED ring).
 *
 * Hold: only stably decoded key masks update the UI (kills RED+BLUE→YELLOW glitches).
 * Release: long HIGH arms pending; commit after grace. Tap length excludes grace time.
 *
 * Gestures (short taps on committed release):
 *   BLUE ×3 → OFF   YELLOW ×3 → ON
 */

static constexpr uint8_t kMax = 120;
static constexpr unsigned int kGapUs = 5000;
static constexpr unsigned int kSolidHighReleaseUs = 12000;

static constexpr uint8_t kKeyRed = 0x1;
static constexpr uint8_t kKeyBlue = 0x2;
static constexpr uint8_t kKeyYellow = 0x4;
static constexpr uint8_t kKeyGreen = 0x8;

static constexpr uint8_t kGestureNeed = 3;
static constexpr uint32_t kGestureWindowMs = 8000;
static constexpr uint32_t kMinTapMs = 40;
static constexpr uint32_t kMaxTapMs = 900;        // wall-clock press time (excludes grace)
static constexpr uint32_t kReconnectGraceMs = 320;
static constexpr uint32_t kFadeOutMs = 400;
static constexpr uint32_t kGestureHintMs = 2200;
static constexpr uint8_t kKeyStableNeed = 3;      // frames before heldKeys may change

// Original quarters were 0/6/12/18. Offset = 180°(12) then left 45°(-3) → +9.
static constexpr uint8_t kRingRot = 9;
static constexpr uint8_t kQRed = (0 + kRingRot) % LED_RING_COUNT;      // was top
static constexpr uint8_t kQBlue = (6 + kRingRot) % LED_RING_COUNT;     // was right
static constexpr uint8_t kQGreen = (12 + kRingRot) % LED_RING_COUNT;   // was bottom
static constexpr uint8_t kQYellow = (18 + kRingRot) % LED_RING_COUNT;  // was left
static constexpr uint8_t kQLen = 6;

static Adafruit_NeoPixel ring(LED_RING_COUNT, Pins::LED_RING, NEO_GRB + NEO_KHZ800);

static volatile unsigned int timings[kMax];
static volatile uint8_t count = 0;
static volatile uint8_t readyCount = 0;
static volatile unsigned int readyBuf[kMax];
static volatile uint8_t packetReady = 0;
static volatile uint8_t releaseSeen = 0;
static volatile uint32_t lastUs = 0;
static volatile uint8_t prevLevel = 0;

static unsigned int local[kMax];
static uint8_t localN = 0;

static uint8_t heldKeys = 0;
static uint8_t candidateKeys = 0;
static uint8_t keyStable = 0;
static bool holding = false;
static bool pendingRelease = false;
static uint32_t pendingReleaseMs = 0;
static uint32_t pressStartMs = 0;
static bool lightsOn = true;
static uint32_t releaseMs = 0;
static uint32_t lastKeyFrameMs = 0;

static uint8_t blueStreak = 0;
static uint8_t yellowStreak = 0;
static uint32_t blueStreakStartMs = 0;
static uint32_t yellowStreakStartMs = 0;
static uint8_t gestureHintKey = 0;
static uint8_t gestureHintCount = 0;
static uint32_t gestureHintUntil = 0;
static uint8_t pressAckKey = 0;          // single bright LED on press
static uint32_t pressAckUntil = 0;

static uint8_t curR[LED_RING_COUNT];
static uint8_t curG[LED_RING_COUNT];
static uint8_t curB[LED_RING_COUNT];
static uint8_t tgtR[LED_RING_COUNT];
static uint8_t tgtG[LED_RING_COUNT];
static uint8_t tgtB[LED_RING_COUNT];
static uint8_t curBright = 0;
static uint8_t tgtBright = 0;
static uint32_t animClock = 0;
static uint8_t confirmFrames = 0;
static bool confirmTurningOn = false;

static uint32_t lastSeeCode = 0;
static uint32_t lastSeeMs = 0;

static void arm() {
  if (packetReady || count < 8) return;
  for (uint8_t i = 0; i < count; i++) readyBuf[i] = timings[i];
  readyCount = count;
  packetReady = 1;
}

static void rfIsr() {
  const uint32_t now = micros();
  const uint8_t level = digitalRead(Pins::RF_DATA) ? 1 : 0;
  const unsigned int d = (unsigned int)(now - lastUs);
  lastUs = now;

  if (prevLevel == 1 && d >= kSolidHighReleaseUs) {
    releaseSeen = 1;
    if (count >= 8) arm();
    count = 0;
  } else if (d > kGapUs) {
    arm();
    count = 0;
  }

  if (count >= kMax) count = 0;
  timings[count++] = d;
  prevLevel = level;
}

static bool isShort(unsigned int u) { return u >= 200 && u <= 700; }
static bool isLong(unsigned int u) { return u >= 800 && u <= 1600; }

static bool decodePt2262(const unsigned int* w, uint8_t n, uint32_t& codeOut) {
  for (uint8_t start = 0; start < 4 && start + 40 <= n; start++) {
    uint32_t code = 0;
    uint8_t bits = 0;
    bool ok = true;
    for (uint8_t i = start; i + 1 < n && bits < 24; i += 2) {
      const unsigned int a = w[i];
      const unsigned int b = w[i + 1];
      if (isShort(a) && isLong(b)) code <<= 1;
      else if (isLong(a) && isShort(b)) code = (code << 1) | 1UL;
      else {
        ok = false;
        break;
      }
      bits++;
    }
    if (ok && bits == 24) {
      codeOut = code;
      return true;
    }
  }
  return false;
}

static void printKeys(uint8_t keys) {
  bool first = true;
  if (keys & kKeyRed) {
    Serial.print(F("RED"));
    first = false;
  }
  if (keys & kKeyBlue) {
    if (!first) Serial.print('+');
    Serial.print(F("BLUE"));
    first = false;
  }
  if (keys & kKeyYellow) {
    if (!first) Serial.print('+');
    Serial.print(F("YELLOW"));
    first = false;
  }
  if (keys & kKeyGreen) {
    if (!first) Serial.print('+');
    Serial.print(F("GREEN"));
    first = false;
  }
  if (first) Serial.print(F("IDLE"));
}

static void logSee(uint32_t code, uint8_t keys, uint32_t now) {
  if (code == lastSeeCode && (now - lastSeeMs) < 400) return;
  lastSeeCode = code;
  lastSeeMs = now;
  Serial.print(F("SEE 0x"));
  Serial.print(code, HEX);
  Serial.print(F(" key=0x"));
  Serial.print(keys, HEX);
  Serial.print(' ');
  printKeys(keys);
  Serial.println();
}

static uint8_t stepToward(uint8_t cur, uint8_t tgt, uint8_t step) {
  if (cur < tgt) {
    uint16_t n = (uint16_t)cur + step;
    return (n > tgt) ? tgt : (uint8_t)n;
  }
  if (cur > tgt) return (cur - tgt < step) ? tgt : (uint8_t)(cur - step);
  return cur;
}

static void clearTargets() {
  for (uint8_t i = 0; i < LED_RING_COUNT; i++) tgtR[i] = tgtG[i] = tgtB[i] = 0;
}

static void setTargetPixel(uint8_t i, uint8_t r, uint8_t g, uint8_t b) {
  i = (uint8_t)(i % LED_RING_COUNT);
  tgtR[i] = r;
  tgtG[i] = g;
  tgtB[i] = b;
}

static void addTargetPixel(uint8_t i, uint8_t r, uint8_t g, uint8_t b) {
  i = (uint8_t)(i % LED_RING_COUNT);
  uint16_t nr = (uint16_t)tgtR[i] + r;
  uint16_t ng = (uint16_t)tgtG[i] + g;
  uint16_t nb = (uint16_t)tgtB[i] + b;
  tgtR[i] = nr > 255 ? 255 : (uint8_t)nr;
  tgtG[i] = ng > 255 ? 255 : (uint8_t)ng;
  tgtB[i] = nb > 255 ? 255 : (uint8_t)nb;
}

static void colorForKey(uint8_t bit, uint8_t& r, uint8_t& g, uint8_t& b) {
  if (bit == kKeyRed) {
    r = 255;
    g = 36;
    b = 28;
  } else if (bit == kKeyBlue) {
    r = 32;
    g = 110;
    b = 255;
  } else if (bit == kKeyYellow) {
    r = 255;
    g = 190;
    b = 24;
  } else {
    r = 28;
    g = 255;
    b = 64;
  }
}

static uint8_t quarterStartForKey(uint8_t bit) {
  if (bit == kKeyRed) return kQRed;
  if (bit == kKeyBlue) return kQBlue;
  if (bit == kKeyYellow) return kQYellow;
  return kQGreen;
}

static void paintQuarter(uint8_t start, uint8_t r, uint8_t g, uint8_t b, uint8_t level) {
  for (uint8_t i = 0; i < kQLen; i++) {
    setTargetPixel((uint8_t)(start + i),
                   (uint8_t)((uint16_t)r * level / 255),
                   (uint8_t)((uint16_t)g * level / 255),
                   (uint8_t)((uint16_t)b * level / 255));
  }
}

static void paintQuarterPulse(uint8_t start, uint8_t r, uint8_t g, uint8_t b) {
  const uint16_t t = (uint16_t)(animClock % 1000);
  const uint8_t breath = (t < 500) ? (uint8_t)(150 + t / 5) : (uint8_t)(150 + (1000 - t) / 5);
  const uint8_t edge = (uint8_t)((animClock / 45) % kQLen);
  for (uint8_t i = 0; i < kQLen; i++) {
    uint8_t lvl = breath;
    const uint8_t d = (uint8_t)((i + kQLen - edge) % kQLen);
    if (d == 0) lvl = 255;
    else if (d == 1) lvl = 230;
    setTargetPixel((uint8_t)(start + i),
                   (uint8_t)((uint16_t)r * lvl / 255),
                   (uint8_t)((uint16_t)g * lvl / 255),
                   (uint8_t)((uint16_t)b * lvl / 255));
  }
}

static void paintIdleClock() {
  tgtBright = 42;
  clearTargets();
  uint8_t r, g, b;
  // Tiny corner ticks only (2 LEDs) — not full ghost quarters
  colorForKey(kKeyRed, r, g, b);
  setTargetPixel(kQRed + 2, r / 8, g / 8, b / 8);
  setTargetPixel(kQRed + 3, r / 8, g / 8, b / 8);
  colorForKey(kKeyBlue, r, g, b);
  setTargetPixel(kQBlue + 2, r / 8, g / 8, b / 8);
  setTargetPixel(kQBlue + 3, r / 8, g / 8, b / 8);
  colorForKey(kKeyGreen, r, g, b);
  setTargetPixel(kQGreen + 2, r / 8, g / 8, b / 8);
  setTargetPixel(kQGreen + 3, r / 8, g / 8, b / 8);
  colorForKey(kKeyYellow, r, g, b);
  setTargetPixel(kQYellow + 2, r / 8, g / 8, b / 8);
  setTargetPixel(kQYellow + 3, r / 8, g / 8, b / 8);

  const uint8_t handFast = (uint8_t)((animClock / 80) % LED_RING_COUNT);
  for (uint8_t t = 0; t < 4; t++) {
    const uint8_t i = (uint8_t)((handFast + LED_RING_COUNT - t) % LED_RING_COUNT);
    const uint8_t v = (uint8_t)(200 - t * 50);
    addTargetPixel(i, v, v, v);
  }
}

static void paintHoldQuarters(uint8_t keys) {
  tgtBright = LED_RING_BRIGHTNESS;
  clearTargets();
  uint8_t r, g, b;

  // Only ACTIVE colors — no ghost quarters (ghost yellow looked "pressed")
  if (keys & kKeyRed) {
    colorForKey(kKeyRed, r, g, b);
    paintQuarterPulse(kQRed, r, g, b);
  }
  if (keys & kKeyBlue) {
    colorForKey(kKeyBlue, r, g, b);
    paintQuarterPulse(kQBlue, r, g, b);
  }
  if (keys & kKeyYellow) {
    colorForKey(kKeyYellow, r, g, b);
    paintQuarterPulse(kQYellow, r, g, b);
  }
  if (keys & kKeyGreen) {
    colorForKey(kKeyGreen, r, g, b);
    paintQuarterPulse(kQGreen, r, g, b);
  }

  uint8_t n = 0;
  if (keys & kKeyRed) n++;
  if (keys & kKeyBlue) n++;
  if (keys & kKeyYellow) n++;
  if (keys & kKeyGreen) n++;
  if (n >= 2) {
    const uint8_t sp = (uint8_t)((animClock / 40) % LED_RING_COUNT);
    addTargetPixel(sp, 160, 160, 160);
  }
}

// One LED per tap in the gesture quarter (clear 1 / 2 / 3 feedback)
static void paintGestureHint() {
  if (!gestureHintKey || animClock > gestureHintUntil) return;
  uint8_t r, g, b;
  colorForKey(gestureHintKey, r, g, b);
  const uint8_t start = quarterStartForKey(gestureHintKey);
  const uint8_t pips = gestureHintCount > 3 ? 3 : gestureHintCount;
  for (uint8_t i = 0; i < pips; i++) {
    // Spaced single lamps: LED 1, 3, 5 in the quarter
    setTargetPixel((uint8_t)(start + 1 + i * 2), r, g, b);
  }
}

static void paintPressAck() {
  if (!pressAckKey || animClock > pressAckUntil) return;
  uint8_t r, g, b;
  colorForKey(pressAckKey, r, g, b);
  const uint8_t start = quarterStartForKey(pressAckKey);
  // Single bright center LED of that quarter
  setTargetPixel((uint8_t)(start + 2), r, g, b);
  setTargetPixel((uint8_t)(start + 3), r / 3, g / 3, b / 3);
}

static void paintOff() {
  tgtBright = 0;
  clearTargets();
}

static void paintSoftConfirm(bool turningOn) {
  tgtBright = 75;
  const uint8_t filled = (uint8_t)((24 - confirmFrames) * LED_RING_COUNT / 24);
  clearTargets();
  for (uint8_t i = 0; i < filled && i < LED_RING_COUNT; i++) {
    if (turningOn) setTargetPixel(i, 30, 220, 90);
    else setTargetPixel(i, 220, 35, 35);
  }
}

static void updateTargets() {
  if (confirmFrames > 0) {
    paintSoftConfirm(confirmTurningOn);
    confirmFrames--;
    if (confirmFrames == 0 && !confirmTurningOn) paintOff();
    return;
  }

  if (!lightsOn) {
    paintOff();
    tgtBright = 55;
    // Unlock-from-off: must see yellow tap lamps clearly
    paintGestureHint();
    paintPressAck();
    if (!gestureHintKey && !pressAckKey) tgtBright = 0;
    return;
  }

  const bool showHold = (holding || pendingRelease) && heldKeys;
  if (showHold) {
    paintHoldQuarters(heldKeys);
    paintPressAck();  // ack peeks on top of quarter
  } else if (releaseMs && (animClock - releaseMs) < kFadeOutMs && heldKeys) {
    paintHoldQuarters(heldKeys);
    tgtBright = (uint8_t)((uint16_t)tgtBright * (kFadeOutMs - (animClock - releaseMs)) / kFadeOutMs);
    paintGestureHint();
  } else {
    if (releaseMs) {
      releaseMs = 0;
      heldKeys = 0;
    }
    paintIdleClock();
    paintGestureHint();
    paintPressAck();
  }
}

static void fadeStep() {
  curBright = stepToward(curBright, tgtBright, 10);
  for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
    curR[i] = stepToward(curR[i], tgtR[i], 22);
    curG[i] = stepToward(curG[i], tgtG[i], 22);
    curB[i] = stepToward(curB[i], tgtB[i], 22);
    ring.setPixelColor(i, ring.Color(curR[i], curG[i], curB[i]));
  }
  ring.setBrightness(curBright);
  ring.show();
}

static void setGestureHint(uint8_t key, uint8_t count, uint32_t now) {
  gestureHintKey = key;
  gestureHintCount = count;
  gestureHintUntil = now + kGestureHintMs;
}

static void setPressAck(uint8_t key, uint32_t now) {
  // Single-key presses get an immediate one-lamp flash
  if (key != kKeyRed && key != kKeyBlue && key != kKeyYellow && key != kKeyGreen) return;
  // only pure singles
  pressAckKey = key;
  pressAckUntil = now + 350;
}

static void onCommittedTap(uint8_t keys, uint32_t now) {
  if (keys != kKeyBlue && keys != kKeyYellow) {
    blueStreak = 0;
    yellowStreak = 0;
    return;
  }

  if (keys == kKeyBlue) {
    yellowStreak = 0;
    if (blueStreak == 0 || (now - blueStreakStartMs) > kGestureWindowMs) {
      blueStreak = 0;
      blueStreakStartMs = now;
    }
    blueStreak++;
    setGestureHint(kKeyBlue, blueStreak, now);
    Serial.print(F("TAP BLUE "));
    Serial.print(blueStreak);
    Serial.print('/');
    Serial.println(kGestureNeed);
    if (blueStreak >= kGestureNeed) {
      lightsOn = false;
      blueStreak = 0;
      holding = false;
      pendingRelease = false;
      heldKeys = 0;
      confirmTurningOn = false;
      confirmFrames = 24;
      gestureHintKey = 0;
      LOG("LIGHTS OFF (3x BLUE tap)");
    }
    return;
  }

  blueStreak = 0;
  if (yellowStreak == 0 || (now - yellowStreakStartMs) > kGestureWindowMs) {
    yellowStreak = 0;
    yellowStreakStartMs = now;
  }
  yellowStreak++;
  setGestureHint(kKeyYellow, yellowStreak, now);
  setPressAck(kKeyYellow, now);  // extra unlock feedback lamp
  Serial.print(F("TAP YELLOW "));
  Serial.print(yellowStreak);
  Serial.print('/');
  Serial.println(kGestureNeed);
  if (yellowStreak >= kGestureNeed) {
    lightsOn = true;
    yellowStreak = 0;
    confirmTurningOn = true;
    confirmFrames = 28;
    gestureHintKey = 0;
    LOG("LIGHTS ON (3x YELLOW tap)");
  }
}

static void commitRelease(uint32_t now) {
  // Use time until release was *signaled*, not including reconnect grace
  const uint32_t heldFor = (pendingReleaseMs > pressStartMs) ? (pendingReleaseMs - pressStartMs) : 0;
  const uint8_t keys = heldKeys;
  Serial.print(F("RELEASE "));
  printKeys(keys);
  Serial.print(F(" t="));
  Serial.print(heldFor);
  Serial.println(F("ms"));

  holding = false;
  pendingRelease = false;
  releaseMs = now;
  candidateKeys = 0;
  keyStable = 0;

  if (keys && heldFor >= kMinTapMs && heldFor <= kMaxTapMs) {
    onCommittedTap(keys, now);
  }
}

static void applyStableKeys(uint8_t keys, uint32_t now) {
  lastKeyFrameMs = now;

  if (keys == candidateKeys) {
    if (keyStable < 255) keyStable++;
  } else {
    candidateKeys = keys;
    keyStable = 1;
  }

  // First lock-in: accept after 1–2 frames so UI feels snappy
  const uint8_t need = holding ? kKeyStableNeed : 2;
  if (keyStable < need) return;

  if (pendingRelease) {
    pendingRelease = false;  // keys returned within grace
  }

  if (!holding) {
    if (releaseMs && (now - releaseMs) < kReconnectGraceMs && keys == heldKeys) {
      holding = true;
      pressStartMs = now;
      releaseMs = 0;
      Serial.print(F("REHOLD "));
      printKeys(keys);
      Serial.println();
      return;
    }
    holding = true;
    pressStartMs = now;
    heldKeys = keys;
    releaseMs = 0;
    if (keys == kKeyRed || keys == kKeyBlue || keys == kKeyYellow || keys == kKeyGreen) {
      setPressAck(keys, now);
    }
    Serial.print(F("HOLD "));
    printKeys(keys);
    Serial.println();
    return;
  }

  if (heldKeys != keys) {
    heldKeys = keys;
    Serial.print(F("HOLD -> "));
    printKeys(keys);
    Serial.println();
  }
}

void demoRfRxSetup() {
  pinMode(Pins::RF_DATA, INPUT);
  lastUs = micros();
  prevLevel = digitalRead(Pins::RF_DATA) ? 1 : 0;
  count = 0;
  packetReady = 0;
  releaseSeen = 0;
  attachInterrupt(digitalPinToInterrupt(Pins::RF_DATA), rfIsr, CHANGE);

  for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
    curR[i] = curG[i] = curB[i] = 0;
    tgtR[i] = tgtG[i] = tgtB[i] = 0;
  }
  ring.begin();
  ring.setBrightness(0);
  ring.clear();
  ring.show();
  lightsOn = true;
  animClock = 0;

  LOG("RF UI: quarters +180 then left 45 (rot+9)");
  LOG("  Dout->D2  ring->D3");
  LOG("  tap x3 BLUE=OFF  YELLOW=ON (stable keys, sticky release)");
}

void demoRfRxLoop() {
  const uint32_t now = millis();
  animClock = now;

  if (!packetReady) {
    noInterrupts();
    const uint8_t n = count;
    const uint32_t last = lastUs;
    const uint8_t rel = releaseSeen;
    interrupts();
    if (n >= 8 && (uint32_t)(micros() - last) > kGapUs) {
      noInterrupts();
      arm();
      if (packetReady) count = 0;
      interrupts();
    }
    if (rel) {
      noInterrupts();
      releaseSeen = 0;
      interrupts();
      if (holding && !pendingRelease) {
        pendingRelease = true;
        pendingReleaseMs = now;
      }
    }
  }

  if (packetReady) {
    noInterrupts();
    localN = readyCount;
    for (uint8_t i = 0; i < localN; i++) local[i] = readyBuf[i];
    packetReady = 0;
    readyCount = 0;
    interrupts();

    uint32_t code = 0;
    if (localN >= 40 && decodePt2262(local, localN, code)) {
      const uint8_t keys = (uint8_t)(code & 0x0FUL);
      if (keys != 0) {
        logSee(code, keys, now);
        applyStableKeys(keys, now);
      }
    }
  }

  // Arm pending release on stuck HIGH, but only if no key frames recently
  if (holding && !pendingRelease && lastKeyFrameMs && (now - lastKeyFrameMs) > 60) {
    noInterrupts();
    const uint8_t lvl = prevLevel;
    const uint32_t since = (uint32_t)(micros() - lastUs);
    interrupts();
    if (lvl == 1 && since >= kSolidHighReleaseUs) {
      pendingRelease = true;
      pendingReleaseMs = now;
    }
  }

  // Also: if no frames at all for a while after pending, still commit via grace
  if (pendingRelease && (now - pendingReleaseMs) >= kReconnectGraceMs) {
    // If a frame arrived and cleared pending, we won't be here
    commitRelease(now);
  }

  if (blueStreak && (now - blueStreakStartMs) > kGestureWindowMs) blueStreak = 0;
  if (yellowStreak && (now - yellowStreakStartMs) > kGestureWindowMs) yellowStreak = 0;
  if (gestureHintKey && now > gestureHintUntil) gestureHintKey = 0;
  if (pressAckKey && now > pressAckUntil) pressAckKey = 0;

  static uint32_t lastFrameDraw = 0;
  if (!every(20, lastFrameDraw)) return;

  updateTargets();
  fadeStep();
}

#endif
