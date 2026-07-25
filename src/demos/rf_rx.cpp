#include "config.h"

#if DEMO_RF_RX

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "board_pins.h"
#include "tools.h"

/*
 * Remote MCU Pin7 (PA4 Dout) → Nano D3 (INT1)  — PT2262 codes
 * LED ring DIN                 → Nano D4
 * Ring 5V/GND                  → Nano 5V/GND
 *
 * Known codes (from PulseView / raw capture):
 *   0xA45352  key 0x2
 *   0xA45354  key 0x4
 *   0xA45356  key 0x6  (combo / both)
 */

static constexpr uint8_t kMax = 120;
static constexpr unsigned int kGapUs = 5000;
static constexpr uint32_t kHoldTimeoutMs = 280;

static constexpr uint32_t kCodeA = 0xA45352UL;
static constexpr uint32_t kCodeB = 0xA45354UL;
static constexpr uint32_t kCodeC = 0xA45356UL;

static Adafruit_NeoPixel ring(LED_RING_COUNT, Pins::LED_RING, NEO_GRB + NEO_KHZ800);

static volatile unsigned int timings[kMax];
static volatile uint8_t count = 0;
static volatile uint8_t readyCount = 0;
static volatile unsigned int readyBuf[kMax];
static volatile uint8_t packetReady = 0;
static volatile uint32_t lastUs = 0;

static unsigned int local[kMax];
static uint8_t localN = 0;

enum Mode : uint8_t { MODE_IDLE = 0, MODE_A, MODE_B, MODE_C };

static Mode mode = MODE_IDLE;
static Mode lastLogged = MODE_IDLE;
static uint32_t lastPacketMs = 0;
static uint32_t modeStartMs = 0;
static uint16_t animPhase = 0;
static uint8_t pressFlash = 0;

static void arm() {
  if (packetReady || count < 8) return;
  for (uint8_t i = 0; i < count; i++) readyBuf[i] = timings[i];
  readyCount = count;
  packetReady = 1;
}

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
      if (isShort(a) && isLong(b)) {
        code <<= 1;
      } else if (isLong(a) && isShort(b)) {
        code = (code << 1) | 1UL;
      } else {
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
  if (code == kCodeA || (code & 0x0FUL) == 0x2) return MODE_A;
  if (code == kCodeB || (code & 0x0FUL) == 0x4) return MODE_B;
  if (code == kCodeC || (code & 0x0FUL) == 0x6) return MODE_C;
  return MODE_IDLE;
}

static uint32_t wheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85) return ring.Color(255 - pos * 3, 0, pos * 3);
  if (pos < 170) {
    pos -= 85;
    return ring.Color(0, pos * 3, 255 - pos * 3);
  }
  pos -= 170;
  return ring.Color(pos * 3, 255 - pos * 3, 0);
}

static void renderIdle(uint32_t now) {
  ring.setBrightness(36);
  // Slow aurora breathe (triangle, no libm)
  const uint16_t t = (uint16_t)((now - modeStartMs) % 1600);
  const uint8_t breath = (t < 800) ? (uint8_t)(t / 5) : (uint8_t)((1600 - t) / 5);
  for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
    const uint16_t h = animPhase + i * 65536UL / LED_RING_COUNT;
    ring.setPixelColor(i, ring.gamma32(ring.ColorHSV(h, 180, (uint8_t)(breath + 20))));
  }
  animPhase += 40;
}

static void renderA(uint32_t now) {
  // Ice comet — cool cyan/white head with fading trail
  ring.setBrightness(LED_RING_BRIGHTNESS);
  ring.clear();
  const uint8_t head = (uint8_t)((now / 28) % LED_RING_COUNT);
  for (uint8_t t = 0; t < 8; t++) {
    const uint8_t i = (head + LED_RING_COUNT - t) % LED_RING_COUNT;
    const uint8_t v = (uint8_t)(255 - t * 28);
    if (t == 0) {
      ring.setPixelColor(i, ring.Color(220, 245, 255));
    } else if (t < 3) {
      ring.setPixelColor(i, ring.Color(v / 3, v, 255));
    } else {
      ring.setPixelColor(i, ring.Color(0, v / 4, v / 2));
    }
  }
  // sparkles while held
  if (((now / 40) & 3) == 0) {
    ring.setPixelColor((head + 12) % LED_RING_COUNT, ring.Color(40, 80, 120));
  }
}

static void renderB(uint32_t now) {
  // Ember swirl — warm orange/red fire
  ring.setBrightness(LED_RING_BRIGHTNESS);
  const uint8_t head = (uint8_t)((now / 32) % LED_RING_COUNT);
  for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
    const uint8_t d1 = (uint8_t)((i + LED_RING_COUNT - head) % LED_RING_COUNT);
    const uint8_t d2 = (uint8_t)((head + LED_RING_COUNT - i) % LED_RING_COUNT);
    const uint8_t dist = (d1 < d2) ? d1 : d2;
    uint8_t heat = (dist < 6) ? (uint8_t)(255 - dist * 40) : 25;
    // flicker
    heat = (uint8_t)((heat * (180 + ((now / 17 + i * 37) & 75))) / 255);
    const uint8_t r = heat;
    const uint8_t g = (uint8_t)(heat / 4);
    ring.setPixelColor(i, ring.Color(r, g, 0));
  }
  ring.setPixelColor(head, ring.Color(255, 220, 80));
}

static void renderC(uint32_t now) {
  // Dual-button / combo — split rainbow spin
  ring.setBrightness(LED_RING_BRIGHTNESS);
  const uint8_t spin = (uint8_t)((now / 20) % LED_RING_COUNT);
  for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
    const uint8_t idx = (i + spin) % LED_RING_COUNT;
    ring.setPixelColor(i, wheel((uint8_t)(idx * 256 / LED_RING_COUNT + animPhase)));
  }
  animPhase += 6;
}

static void renderFlash() {
  ring.setBrightness(160);
  ring.fill(ring.Color(255, 255, 255));
}

static void applyMode(Mode m, uint32_t now) {
  if (m != mode) {
    mode = m;
    modeStartMs = now;
    if (m != MODE_IDLE) pressFlash = 4;  // brief white flash on press
    if (m != lastLogged) {
      lastLogged = m;
      if (m == MODE_A) LOG("LIGHT: button A — ice comet");
      else if (m == MODE_B) LOG("LIGHT: button B — ember swirl");
      else if (m == MODE_C) LOG("LIGHT: button C — rainbow spin");
      else LOG("LIGHT: idle aurora");
    }
  }
}

void demoRfRxSetup() {
  pinMode(Pins::RF_DATA, INPUT);
  lastUs = micros();
  count = 0;
  packetReady = 0;
  attachInterrupt(digitalPinToInterrupt(Pins::RF_DATA), rfIsr, CHANGE);

  ring.begin();
  ring.setBrightness(LED_RING_BRIGHTNESS);
  ring.clear();
  ring.show();
  modeStartMs = millis();

  LOG("RF + LED show ready");
  LOG("  Remote Pin7 (PA4) -> D3");
  LOG("  LED ring DIN -> D4  (5V + GND)");
  LOG("  A=ice  B=ember  C/both=rainbow");
}

void demoRfRxLoop() {
  const uint32_t now = millis();

  // Finish RF packet if silence after edges
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
      lastPacketMs = now;
      applyMode(modeFromCode(code), now);
    }
  }

  // Release → idle
  if (mode != MODE_IDLE && (now - lastPacketMs) > kHoldTimeoutMs) {
    applyMode(MODE_IDLE, now);
  }

  // Animate ~40 FPS
  static uint32_t lastFrame = 0;
  if (!every(25, lastFrame)) return;

  if (pressFlash > 0) {
    renderFlash();
    pressFlash--;
  } else if (mode == MODE_A) {
    renderA(now);
  } else if (mode == MODE_B) {
    renderB(now);
  } else if (mode == MODE_C) {
    renderC(now);
  } else {
    renderIdle(now);
  }
  ring.show();
}

#endif
