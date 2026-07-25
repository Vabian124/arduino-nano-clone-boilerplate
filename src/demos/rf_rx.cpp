#include "config.h"

#if DEMO_RF_RX

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "board_pins.h"
#include "tools.h"

/*
 * Remote MCU Pin7 (PA4 Dout) → Nano D2
 * LED ring DIN                 → Nano D3
 *
 * Key shape: short HIGH → hex…hex while held → long solid HIGH = release.
 *
 * Color map (key nibble bits OR together):
 *   0x1 RED (top)     0x2 BLUE (right)
 *   0x4 YELLOW (left) 0x8 GREEN (bottom)
 *
 * Gestures: BLUE ×3 → lights OFF, YELLOW ×3 → lights ON
 */

static constexpr uint8_t kMax = 120;
static constexpr unsigned int kGapUs = 5000;
static constexpr unsigned int kSolidHighReleaseUs = 10000;

static constexpr uint8_t kKeyRed = 0x1;
static constexpr uint8_t kKeyBlue = 0x2;
static constexpr uint8_t kKeyYellow = 0x4;
static constexpr uint8_t kKeyGreen = 0x8;

static constexpr uint8_t kBlueOffCount = 3;
static constexpr uint8_t kYellowOnCount = 3;
static constexpr uint32_t kGestureWindowMs = 4000;
static constexpr uint32_t kFadeOutMs = 500;

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

static uint8_t heldKeys = 0;  // 0x1..0xF bitfield
static bool holding = false;
static bool lightsOn = true;
static uint32_t releaseMs = 0;

static uint8_t blueStreak = 0;
static uint8_t yellowStreak = 0;
static uint32_t blueStreakStartMs = 0;
static uint32_t yellowStreakStartMs = 0;

static uint8_t curR[LED_RING_COUNT];
static uint8_t curG[LED_RING_COUNT];
static uint8_t curB[LED_RING_COUNT];
static uint8_t tgtR[LED_RING_COUNT];
static uint8_t tgtG[LED_RING_COUNT];
static uint8_t tgtB[LED_RING_COUNT];
static uint8_t curBright = 0;
static uint8_t tgtBright = 0;
static uint16_t animPhase = 0;
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
  auto one = [&](uint8_t bit, const __FlashStringHelper* name) {
    if (!(keys & bit)) return;
    if (!first) Serial.print('+');
    Serial.print(name);
    first = false;
  };
  one(kKeyRed, F("RED"));
  one(kKeyBlue, F("BLUE"));
  one(kKeyYellow, F("YELLOW"));
  one(kKeyGreen, F("GREEN"));
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
  tgtR[i] = r;
  tgtG[i] = g;
  tgtB[i] = b;
}

static void addTargetPixel(uint8_t i, uint8_t r, uint8_t g, uint8_t b) {
  uint16_t nr = (uint16_t)tgtR[i] + r;
  uint16_t ng = (uint16_t)tgtG[i] + g;
  uint16_t nb = (uint16_t)tgtB[i] + b;
  tgtR[i] = nr > 255 ? 255 : (uint8_t)nr;
  tgtG[i] = ng > 255 ? 255 : (uint8_t)ng;
  tgtB[i] = nb > 255 ? 255 : (uint8_t)nb;
}

static void paintIdle() {
  tgtBright = 40;
  const uint16_t t = (uint16_t)(animClock % 1600);
  const uint8_t breath = (t < 800) ? (uint8_t)(t / 5) : (uint8_t)((1600 - t) / 5);
  for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
    const uint16_t h = animPhase + i * 65536UL / LED_RING_COUNT;
    const uint32_t c = ring.gamma32(ring.ColorHSV(h, 200, (uint8_t)(breath + 15)));
    setTargetPixel(i, (uint8_t)(c >> 16), (uint8_t)(c >> 8), (uint8_t)c);
  }
  animPhase += 28;
}

static void paintKeys(uint8_t keys) {
  tgtBright = LED_RING_BRIGHTNESS;
  clearTargets();
  const uint8_t spin = (uint8_t)((animClock / 28) % LED_RING_COUNT);

  // Each pressed color gets a chase head offset around the ring
  uint8_t slot = 0;
  auto chase = [&](uint8_t bit, uint8_t r, uint8_t g, uint8_t b) {
    if (!(keys & bit)) return;
    const uint8_t head = (uint8_t)((spin + slot * (LED_RING_COUNT / 4)) % LED_RING_COUNT);
    slot++;
    for (uint8_t t = 0; t < 7; t++) {
      const uint8_t i = (head + LED_RING_COUNT - t) % LED_RING_COUNT;
      const uint8_t v = (uint8_t)(255 - t * 32);
      addTargetPixel(i,
                     (uint8_t)((uint16_t)r * v / 255),
                     (uint8_t)((uint16_t)g * v / 255),
                     (uint8_t)((uint16_t)b * v / 255));
    }
  };

  chase(kKeyRed, 255, 30, 20);
  chase(kKeyBlue, 40, 100, 255);
  chase(kKeyYellow, 255, 180, 20);
  chase(kKeyGreen, 40, 255, 50);
}

static void paintOff() {
  tgtBright = 0;
  clearTargets();
}

static void paintSoftConfirm(bool turningOn) {
  tgtBright = 55;
  const uint8_t v = 90;
  for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
    if (turningOn) setTargetPixel(i, 0, v, v / 4);
    else setTargetPixel(i, v, 0, 0);
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
    return;
  }
  if (holding && heldKeys) {
    paintKeys(heldKeys);
  } else if (releaseMs && (millis() - releaseMs) < kFadeOutMs && heldKeys) {
    paintKeys(heldKeys);
    tgtBright = (uint8_t)((uint16_t)tgtBright * (kFadeOutMs - (millis() - releaseMs)) / kFadeOutMs);
  } else {
    if (releaseMs) {
      releaseMs = 0;
      heldKeys = 0;
    }
    paintIdle();
  }
}

static void fadeStep() {
  curBright = stepToward(curBright, tgtBright, 6);
  for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
    curR[i] = stepToward(curR[i], tgtR[i], 12);
    curG[i] = stepToward(curG[i], tgtG[i], 12);
    curB[i] = stepToward(curB[i], tgtB[i], 12);
    ring.setPixelColor(i, ring.Color(curR[i], curG[i], curB[i]));
  }
  ring.setBrightness(curBright);
  ring.show();
}

static void onDistinctTap(uint8_t keys, uint32_t now) {
  if (keys == kKeyBlue) {
    yellowStreak = 0;
    if (blueStreak == 0 || (now - blueStreakStartMs) > kGestureWindowMs) {
      blueStreak = 0;
      blueStreakStartMs = now;
    }
    blueStreak++;
    Serial.print(F("GESTURE BLUE "));
    Serial.print(blueStreak);
    Serial.print('/');
    Serial.println(kBlueOffCount);
    if (blueStreak >= kBlueOffCount) {
      lightsOn = false;
      blueStreak = 0;
      holding = false;
      heldKeys = 0;
      confirmTurningOn = false;
      confirmFrames = 24;
      LOG("LIGHTS OFF (3x BLUE)");
    }
    return;
  }
  if (keys == kKeyYellow) {
    blueStreak = 0;
    if (yellowStreak == 0 || (now - yellowStreakStartMs) > kGestureWindowMs) {
      yellowStreak = 0;
      yellowStreakStartMs = now;
    }
    yellowStreak++;
    Serial.print(F("GESTURE YELLOW "));
    Serial.print(yellowStreak);
    Serial.print('/');
    Serial.println(kYellowOnCount);
    if (yellowStreak >= kYellowOnCount) {
      lightsOn = true;
      yellowStreak = 0;
      confirmTurningOn = true;
      confirmFrames = 24;
      LOG("LIGHTS ON (3x YELLOW)");
    }
    return;
  }
  blueStreak = 0;
  yellowStreak = 0;
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

  LOG("RF colors: RED top, BLUE right, YELLOW left, GREEN bottom");
  LOG("  Dout->D2  ring->D3  (keys OR as nibble 0x1..0xF)");
  LOG("  3x BLUE=OFF  3x YELLOW=ON");
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
      if (holding) {
        Serial.print(F("RELEASE "));
        printKeys(heldKeys);
        Serial.println(F(" (long HIGH)"));
        holding = false;
        releaseMs = now;
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

#if RF_RAW_LOG
    Serial.print(F("RAW n="));
    Serial.print(localN);
    Serial.print(F(" us:"));
    for (uint8_t i = 0; i < localN; i++) {
      Serial.print(' ');
      Serial.print(local[i]);
    }
    Serial.println();
#endif

    uint32_t code = 0;
    if (localN >= 40 && decodePt2262(local, localN, code)) {
      const uint8_t keys = (uint8_t)(code & 0x0FUL);
      if (keys != 0) {
        logSee(code, keys, now);
        if (!holding) {
          onDistinctTap(keys, now);
          Serial.print(F("HOLD "));
          printKeys(keys);
          Serial.println();
        } else if (heldKeys != keys) {
          Serial.print(F("HOLD -> "));
          printKeys(keys);
          Serial.println();
        }
        holding = true;
        heldKeys = keys;
        releaseMs = 0;
      }
    }
  }

  if (holding) {
    noInterrupts();
    const uint8_t lvl = prevLevel;
    const uint32_t since = (uint32_t)(micros() - lastUs);
    interrupts();
    if (lvl == 1 && since >= kSolidHighReleaseUs) releaseSeen = 1;
  }

  if (blueStreak && (now - blueStreakStartMs) > kGestureWindowMs) blueStreak = 0;
  if (yellowStreak && (now - yellowStreakStartMs) > kGestureWindowMs) yellowStreak = 0;

  static uint32_t lastFrameDraw = 0;
  if (!every(20, lastFrameDraw)) return;

  updateTargets();
  fadeStep();
}

#endif
