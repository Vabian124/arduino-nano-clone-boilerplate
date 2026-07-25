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
 * Key shape (raw):
 *   short HIGH → hex frame → hex → hex … (while held)
 *   then, after release, a long solid HIGH
 *
 * Sync between frames is a long LOW (~11 ms) — not a release.
 * Only the long solid HIGH ends a hold.
 *
 * Gestures: LOCK ×3 → lights OFF, UNLOCK ×3 → lights ON
 */

static constexpr uint8_t kMax = 120;
static constexpr unsigned int kGapUs = 5000;           // frame sync LOW (~11 ms)
static constexpr unsigned int kSolidHighReleaseUs = 10000;  // release marker HIGH

static constexpr uint32_t kCodeLock = 0xA45352UL;
static constexpr uint32_t kCodeUnlock = 0xA45354UL;
static constexpr uint32_t kCodeCombo = 0xA45356UL;

static constexpr uint8_t kLockOffCount = 3;
static constexpr uint8_t kUnlockOnCount = 3;
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

enum Mode : uint8_t { MODE_IDLE = 0, MODE_LOCK, MODE_UNLOCK, MODE_COMBO };

static Mode mode = MODE_IDLE;
static bool holding = false;
static bool lightsOn = true;
static uint32_t releaseMs = 0;

static uint8_t lockStreak = 0;
static uint8_t unlockStreak = 0;
static uint32_t lockStreakStartMs = 0;
static uint32_t unlockStreakStartMs = 0;

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

  // Ended level was prevLevel.
  // Long solid HIGH → release. Long LOW → frame gap (arm hex).
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

static Mode modeFromCode(uint32_t code) {
  const uint8_t key = (uint8_t)(code & 0x0FUL);
  if (code == kCodeLock || key == 0x2) return MODE_LOCK;
  if (code == kCodeUnlock || key == 0x4) return MODE_UNLOCK;
  if (code == kCodeCombo || key == 0x6) return MODE_COMBO;
  return MODE_IDLE;
}

static const __FlashStringHelper* modeName(Mode m) {
  if (m == MODE_LOCK) return F("LOCK");
  if (m == MODE_UNLOCK) return F("UNLOCK");
  if (m == MODE_COMBO) return F("BOTH");
  return F("IDLE");
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

static uint32_t wheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85) return ((uint32_t)(255 - pos * 3) << 16) | (pos * 3);
  if (pos < 170) {
    pos -= 85;
    return ((uint32_t)(pos * 3) << 8) | (255 - pos * 3);
  }
  pos -= 170;
  return ((uint32_t)(pos * 3) << 16) | ((uint32_t)(255 - pos * 3) << 8);
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

static void paintLock() {
  tgtBright = LED_RING_BRIGHTNESS;
  clearTargets();
  const uint8_t head = (uint8_t)((animClock / 30) % LED_RING_COUNT);
  for (uint8_t t = 0; t < 9; t++) {
    const uint8_t i = (head + LED_RING_COUNT - t) % LED_RING_COUNT;
    const uint8_t v = (uint8_t)(255 - t * 26);
    if (t == 0) setTargetPixel(i, 120, 210, 255);
    else if (t < 3) setTargetPixel(i, v / 5, v, 255);
    else setTargetPixel(i, 0, v / 5, v / 2);
  }
}

static void paintUnlock() {
  tgtBright = LED_RING_BRIGHTNESS;
  const uint8_t head = (uint8_t)((animClock / 34) % LED_RING_COUNT);
  for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
    const uint8_t d1 = (uint8_t)((i + LED_RING_COUNT - head) % LED_RING_COUNT);
    const uint8_t d2 = (uint8_t)((head + LED_RING_COUNT - i) % LED_RING_COUNT);
    const uint8_t dist = (d1 < d2) ? d1 : d2;
    uint8_t heat = (dist < 6) ? (uint8_t)(230 - dist * 36) : 18;
    heat = (uint8_t)((heat * (190 + ((animClock / 19 + i * 41) & 65))) / 255);
    setTargetPixel(i, heat, (uint8_t)(heat / 5), 0);
  }
  setTargetPixel(head, 255, 160, 40);
}

static void paintCombo() {
  tgtBright = LED_RING_BRIGHTNESS;
  const uint8_t spin = (uint8_t)((animClock / 22) % LED_RING_COUNT);
  for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
    const uint8_t idx = (i + spin) % LED_RING_COUNT;
    const uint32_t c = wheel((uint8_t)(idx * 256 / LED_RING_COUNT + (animPhase >> 1)));
    setTargetPixel(i, (uint8_t)(c >> 16), (uint8_t)(c >> 8), (uint8_t)c);
  }
  animPhase += 5;
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
  if (holding) {
    if (mode == MODE_LOCK) paintLock();
    else if (mode == MODE_UNLOCK) paintUnlock();
    else if (mode == MODE_COMBO) paintCombo();
    else paintIdle();
  } else if (releaseMs && (millis() - releaseMs) < kFadeOutMs && mode != MODE_IDLE) {
    if (mode == MODE_LOCK) paintLock();
    else if (mode == MODE_UNLOCK) paintUnlock();
    else if (mode == MODE_COMBO) paintCombo();
    tgtBright = (uint8_t)((uint16_t)tgtBright * (kFadeOutMs - (millis() - releaseMs)) / kFadeOutMs);
  } else {
    if (releaseMs) {
      releaseMs = 0;
      mode = MODE_IDLE;
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

static void onDistinctTap(Mode btn, uint32_t now) {
  if (btn == MODE_LOCK) {
    unlockStreak = 0;
    if (lockStreak == 0 || (now - lockStreakStartMs) > kGestureWindowMs) {
      lockStreak = 0;
      lockStreakStartMs = now;
    }
    lockStreak++;
    Serial.print(F("GESTURE lock "));
    Serial.print(lockStreak);
    Serial.print(F("/"));
    Serial.println(kLockOffCount);
    if (lockStreak >= kLockOffCount) {
      lightsOn = false;
      lockStreak = 0;
      holding = false;
      mode = MODE_IDLE;
      confirmTurningOn = false;
      confirmFrames = 24;
      LOG("LIGHTS OFF (3x LOCK)");
    }
    return;
  }
  if (btn == MODE_UNLOCK) {
    lockStreak = 0;
    if (unlockStreak == 0 || (now - unlockStreakStartMs) > kGestureWindowMs) {
      unlockStreak = 0;
      unlockStreakStartMs = now;
    }
    unlockStreak++;
    Serial.print(F("GESTURE unlock "));
    Serial.print(unlockStreak);
    Serial.print(F("/"));
    Serial.println(kUnlockOnCount);
    if (unlockStreak >= kUnlockOnCount) {
      lightsOn = true;
      unlockStreak = 0;
      confirmTurningOn = true;
      confirmFrames = 24;
      LOG("LIGHTS ON (3x UNLOCK)");
    }
    return;
  }
  lockStreak = 0;
  unlockStreak = 0;
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

  LOG("RF raw: short HIGH + hex…hex → long HIGH = release");
  LOG("  Dout->D2  ring->D3");
  LOG("  3x LOCK=OFF  3x UNLOCK=ON");
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
        Serial.print(modeName(mode));
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
      const Mode btn = modeFromCode(code);
      if (btn != MODE_IDLE) {
        if (!holding) {
          Serial.print(F("CODE 0x"));
          Serial.print(code, HEX);
          Serial.print(F(" "));
          Serial.println(modeName(btn));
          onDistinctTap(btn, now);
          Serial.print(F("HOLD "));
          Serial.println(modeName(btn));
        } else if (mode != btn) {
          Serial.print(F("HOLD -> "));
          Serial.println(modeName(btn));
        }
        holding = true;
        mode = btn;
        releaseMs = 0;
      }
    }
  }

  // Long HIGH still sitting high (no falling edge yet)
  if (holding) {
    noInterrupts();
    const uint8_t lvl = prevLevel;
    const uint32_t since = (uint32_t)(micros() - lastUs);
    interrupts();
    if (lvl == 1 && since >= kSolidHighReleaseUs) releaseSeen = 1;
  }

  if (lockStreak && (now - lockStreakStartMs) > kGestureWindowMs) lockStreak = 0;
  if (unlockStreak && (now - unlockStreakStartMs) > kGestureWindowMs) unlockStreak = 0;

  static uint32_t lastFrameDraw = 0;
  if (!every(20, lastFrameDraw)) return;

  updateTargets();
  fadeStep();
}

#endif
