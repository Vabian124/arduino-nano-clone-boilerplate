#include "config.h"

#if DEMO_RF_RX

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "board_pins.h"
#include "tools.h"

/*
 * Remote Dout → D2, LED ring DIN → D3
 *
 * HOLD MODEL (simple):
 *   Valid hex frames keep arriving  → held
 *   No valid frame for kHoldTimeout → released
 *
 * Inter-frame gaps are ~30–50 ms. Timeout 150 ms ≈ a few missed frames
 * (tolerates brief RF drop) but still feels snappy on real release.
 *
 * Keys (nibble OR):
 *   0x1 RED | 0x2 BLUE | 0x4 YELLOW | 0x8 GREEN
 *
 * Ring quarters: original map +180° then left 45° → LED offset +9.
 *
 * Gestures (short hold on release): BLUE×3 OFF, YELLOW×3 ON
 */

static constexpr uint8_t kMax = 120;
static constexpr unsigned int kGapUs = 5000;

static constexpr uint8_t kKeyRed = 0x1;
static constexpr uint8_t kKeyBlue = 0x2;
static constexpr uint8_t kKeyYellow = 0x4;
static constexpr uint8_t kKeyGreen = 0x8;

// --- hold / release (frame watchdog) ---
static constexpr uint32_t kHoldTimeoutMs = 150;
static constexpr uint8_t kStableNeed = 2;  // matching key frames before UI follows

// --- gestures ---
static constexpr uint8_t kGestureNeed = 3;
static constexpr uint32_t kGestureWindowMs = 8000;
static constexpr uint32_t kMinTapMs = 40;
static constexpr uint32_t kMaxTapMs = 900;
static constexpr uint32_t kGestureHintMs = 2200;
static constexpr uint32_t kPressAckMs = 300;
static constexpr uint32_t kFadeOutMs = 400;

// Quarters (+180° then left 45° ⇒ +9)
static constexpr uint8_t kRingRot = 9;
static constexpr uint8_t kQRed = (0 + kRingRot) % 24;
static constexpr uint8_t kQBlue = (6 + kRingRot) % 24;
static constexpr uint8_t kQGreen = (12 + kRingRot) % 24;
static constexpr uint8_t kQYellow = (18 + kRingRot) % 24;
static constexpr uint8_t kQLen = 6;

static Adafruit_NeoPixel ring(LED_RING_COUNT, Pins::LED_RING, NEO_GRB + NEO_KHZ800);

static volatile unsigned int timings[kMax];
static volatile uint8_t count = 0;
static volatile uint8_t readyCount = 0;
static volatile unsigned int readyBuf[kMax];
static volatile uint8_t packetReady = 0;
static volatile uint32_t lastUs = 0;

static unsigned int local[kMax];
static uint8_t localN = 0;

static bool holding = false;
static uint8_t heldKeys = 0;
static uint8_t candKeys = 0;
static uint8_t candCount = 0;
static uint32_t lastFrameMs = 0;
static uint32_t pressStartMs = 0;
static uint32_t releaseMs = 0;
static uint8_t fadeKeys = 0;
static bool lightsOn = true;

static uint8_t blueStreak = 0;
static uint8_t yellowStreak = 0;
static uint32_t blueStreakStartMs = 0;
static uint32_t yellowStreakStartMs = 0;
static uint8_t hintKey = 0;
static uint8_t hintCount = 0;
static uint32_t hintUntil = 0;
static uint8_t ackKey = 0;
static uint32_t ackUntil = 0;

static uint8_t curR[LED_RING_COUNT], curG[LED_RING_COUNT], curB[LED_RING_COUNT];
static uint8_t tgtR[LED_RING_COUNT], tgtG[LED_RING_COUNT], tgtB[LED_RING_COUNT];
static uint8_t curBright = 0, tgtBright = 0;
static uint32_t animClock = 0;
static uint8_t confirmFrames = 0;
static bool confirmOn = false;

static uint32_t lastLogCode = 0;
static uint32_t lastLogMs = 0;

static void arm() {
  if (packetReady || count < 8) return;
  for (uint8_t i = 0; i < count; i++) readyBuf[i] = timings[i];
  readyCount = count;
  packetReady = 1;
}

// Edge ISR: only collect pulse widths. Release is NOT decided here.
static void rfIsr() {
  const uint32_t now = micros();
  const unsigned int d = (unsigned int)(now - lastUs);
  lastUs = now;

  if (d > kGapUs) {
    arm();
    count = 0;
  }
  if (count >= kMax) count = 0;
  timings[count++] = d;
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
  if (code == lastLogCode && (now - lastLogMs) < 400) return;
  lastLogCode = code;
  lastLogMs = now;
  Serial.print(F("SEE 0x"));
  Serial.print(code, HEX);
  Serial.print(F(" "));
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

static void setPix(uint8_t i, uint8_t r, uint8_t g, uint8_t b) {
  i = (uint8_t)(i % LED_RING_COUNT);
  tgtR[i] = r;
  tgtG[i] = g;
  tgtB[i] = b;
}

static void addPix(uint8_t i, uint8_t r, uint8_t g, uint8_t b) {
  i = (uint8_t)(i % LED_RING_COUNT);
  uint16_t nr = (uint16_t)tgtR[i] + r;
  uint16_t ng = (uint16_t)tgtG[i] + g;
  uint16_t nb = (uint16_t)tgtB[i] + b;
  tgtR[i] = nr > 255 ? 255 : (uint8_t)nr;
  tgtG[i] = ng > 255 ? 255 : (uint8_t)ng;
  tgtB[i] = nb > 255 ? 255 : (uint8_t)nb;
}

static void colorFor(uint8_t bit, uint8_t& r, uint8_t& g, uint8_t& b) {
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

static uint8_t qStart(uint8_t bit) {
  if (bit == kKeyRed) return kQRed;
  if (bit == kKeyBlue) return kQBlue;
  if (bit == kKeyYellow) return kQYellow;
  return kQGreen;
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
    setPix((uint8_t)(start + i),
           (uint8_t)((uint16_t)r * lvl / 255),
           (uint8_t)((uint16_t)g * lvl / 255),
           (uint8_t)((uint16_t)b * lvl / 255));
  }
}

static void paintIdle() {
  tgtBright = 42;
  clearTargets();
  uint8_t r, g, b;
  colorFor(kKeyRed, r, g, b);
  setPix(kQRed + 2, r / 8, g / 8, b / 8);
  setPix(kQRed + 3, r / 8, g / 8, b / 8);
  colorFor(kKeyBlue, r, g, b);
  setPix(kQBlue + 2, r / 8, g / 8, b / 8);
  setPix(kQBlue + 3, r / 8, g / 8, b / 8);
  colorFor(kKeyGreen, r, g, b);
  setPix(kQGreen + 2, r / 8, g / 8, b / 8);
  setPix(kQGreen + 3, r / 8, g / 8, b / 8);
  colorFor(kKeyYellow, r, g, b);
  setPix(kQYellow + 2, r / 8, g / 8, b / 8);
  setPix(kQYellow + 3, r / 8, g / 8, b / 8);

  const uint8_t hand = (uint8_t)((animClock / 80) % LED_RING_COUNT);
  for (uint8_t t = 0; t < 4; t++) {
    const uint8_t v = (uint8_t)(200 - t * 50);
    addPix((uint8_t)((hand + LED_RING_COUNT - t) % LED_RING_COUNT), v, v, v);
  }
}

static void paintHold(uint8_t keys) {
  tgtBright = LED_RING_BRIGHTNESS;
  clearTargets();
  uint8_t r, g, b;
  if (keys & kKeyRed) {
    colorFor(kKeyRed, r, g, b);
    paintQuarterPulse(kQRed, r, g, b);
  }
  if (keys & kKeyBlue) {
    colorFor(kKeyBlue, r, g, b);
    paintQuarterPulse(kQBlue, r, g, b);
  }
  if (keys & kKeyYellow) {
    colorFor(kKeyYellow, r, g, b);
    paintQuarterPulse(kQYellow, r, g, b);
  }
  if (keys & kKeyGreen) {
    colorFor(kKeyGreen, r, g, b);
    paintQuarterPulse(kQGreen, r, g, b);
  }
  uint8_t n = 0;
  if (keys & kKeyRed) n++;
  if (keys & kKeyBlue) n++;
  if (keys & kKeyYellow) n++;
  if (keys & kKeyGreen) n++;
  if (n >= 2) addPix((uint8_t)((animClock / 40) % LED_RING_COUNT), 160, 160, 160);
}

static void paintHint() {
  if (!hintKey || animClock > hintUntil) return;
  uint8_t r, g, b;
  colorFor(hintKey, r, g, b);
  const uint8_t start = qStart(hintKey);
  const uint8_t pips = hintCount > 3 ? 3 : hintCount;
  for (uint8_t i = 0; i < pips; i++) setPix((uint8_t)(start + 1 + i * 2), r, g, b);
}

static void paintAck() {
  if (!ackKey || animClock > ackUntil) return;
  uint8_t r, g, b;
  colorFor(ackKey, r, g, b);
  const uint8_t start = qStart(ackKey);
  setPix((uint8_t)(start + 2), r, g, b);
  setPix((uint8_t)(start + 3), r / 3, g / 3, b / 3);
}

static void paintConfirm() {
  tgtBright = 75;
  clearTargets();
  const uint8_t filled = (uint8_t)((24 - confirmFrames) * LED_RING_COUNT / 24);
  for (uint8_t i = 0; i < filled && i < LED_RING_COUNT; i++) {
    if (confirmOn) setPix(i, 30, 220, 90);
    else setPix(i, 220, 35, 35);
  }
}

static void updateTargets() {
  if (confirmFrames > 0) {
    paintConfirm();
    confirmFrames--;
    if (confirmFrames == 0 && !confirmOn) {
      tgtBright = 0;
      clearTargets();
    }
    return;
  }

  if (!lightsOn) {
    clearTargets();
    tgtBright = 0;
    if ((hintKey && animClock <= hintUntil) || (ackKey && animClock <= ackUntil)) {
      tgtBright = 55;
      paintHint();
      paintAck();
    }
    return;
  }

  if (holding && heldKeys) {
    paintHold(heldKeys);
    paintAck();
  } else if (releaseMs && (animClock - releaseMs) < kFadeOutMs && fadeKeys) {
    paintHold(fadeKeys);
    tgtBright = (uint8_t)((uint16_t)tgtBright * (kFadeOutMs - (animClock - releaseMs)) / kFadeOutMs);
    paintHint();
  } else {
    if (releaseMs) {
      releaseMs = 0;
      fadeKeys = 0;
    }
    paintIdle();
    paintHint();
    paintAck();
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

static void onTap(uint8_t keys, uint32_t now) {
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
    hintKey = kKeyBlue;
    hintCount = blueStreak;
    hintUntil = now + kGestureHintMs;
    Serial.print(F("TAP BLUE "));
    Serial.print(blueStreak);
    Serial.print('/');
    Serial.println(kGestureNeed);
    if (blueStreak >= kGestureNeed) {
      lightsOn = false;
      blueStreak = 0;
      holding = false;
      heldKeys = 0;
      confirmOn = false;
      confirmFrames = 24;
      hintKey = 0;
      LOG("LIGHTS OFF");
    }
    return;
  }

  blueStreak = 0;
  if (yellowStreak == 0 || (now - yellowStreakStartMs) > kGestureWindowMs) {
    yellowStreak = 0;
    yellowStreakStartMs = now;
  }
  yellowStreak++;
  hintKey = kKeyYellow;
  hintCount = yellowStreak;
  hintUntil = now + kGestureHintMs;
  ackKey = kKeyYellow;
  ackUntil = now + kPressAckMs;
  Serial.print(F("TAP YELLOW "));
  Serial.print(yellowStreak);
  Serial.print('/');
  Serial.println(kGestureNeed);
  if (yellowStreak >= kGestureNeed) {
    lightsOn = true;
    yellowStreak = 0;
    confirmOn = true;
    confirmFrames = 28;
    hintKey = 0;
    LOG("LIGHTS ON");
  }
}

static void endHold(uint32_t now) {
  const uint32_t heldFor = now - pressStartMs;
  const uint8_t keys = heldKeys;

  Serial.print(F("RELEASE "));
  printKeys(keys);
  Serial.print(F(" t="));
  Serial.print(heldFor);
  Serial.println(F("ms"));

  holding = false;
  fadeKeys = keys;
  releaseMs = now;
  heldKeys = 0;
  candKeys = 0;
  candCount = 0;

  if (keys && heldFor >= kMinTapMs && heldFor <= kMaxTapMs) onTap(keys, now);
}

static void onKeys(uint8_t keys, uint32_t now) {
  lastFrameMs = now;

  if (keys == candKeys) {
    if (candCount < 255) candCount++;
  } else {
    candKeys = keys;
    candCount = 1;
  }
  if (candCount < kStableNeed) return;

  if (!holding) {
    holding = true;
    pressStartMs = now;
    heldKeys = keys;
    releaseMs = 0;
    fadeKeys = 0;
    if (keys == kKeyRed || keys == kKeyBlue || keys == kKeyYellow || keys == kKeyGreen) {
      ackKey = keys;
      ackUntil = now + kPressAckMs;
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
  count = 0;
  packetReady = 0;
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

  LOG("RF: hold = frames keep coming; release = frames stop");
  LOG("  Dout->D2  ring->D3  timeout 150ms");
  LOG("  tap x3 BLUE=OFF  YELLOW=ON");
}

void demoRfRxLoop() {
  const uint32_t now = millis();
  animClock = now;

  // Flush a train if the line went quiet mid-buffer
  if (!packetReady) {
    noInterrupts();
    const uint8_t n = count;
    const uint32_t last = lastUs;
    interrupts();
    if (n >= 8 && (uint32_t)(micros() - last) > kGapUs) {
      noInterrupts();
      arm();
      if (packetReady) count = 0;
      interrupts();
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
      if (keys) {
        logSee(code, keys, now);
        onKeys(keys, now);
      }
    }
  }

  // THE release rule: no fresh frames for kHoldTimeoutMs
  if (holding && lastFrameMs && (now - lastFrameMs) >= kHoldTimeoutMs) {
    endHold(now);
  }

  if (blueStreak && (now - blueStreakStartMs) > kGestureWindowMs) blueStreak = 0;
  if (yellowStreak && (now - yellowStreakStartMs) > kGestureWindowMs) yellowStreak = 0;
  if (hintKey && now > hintUntil) hintKey = 0;
  if (ackKey && now > ackUntil) ackKey = 0;

  static uint32_t lastDraw = 0;
  if (!every(20, lastDraw)) return;
  updateTargets();
  fadeStep();
}

#endif
