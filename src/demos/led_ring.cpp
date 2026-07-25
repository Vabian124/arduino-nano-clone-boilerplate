#include "config.h"

#if DEMO_LED_RING

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "board_pins.h"
#include "tools.h"

/*
 * WCMCU-2812B — 24× WS2812B RGB LED ring
 *
 * Ring pads (typical order): 5V IN · DIN · GND · DOUT
 *
 * Wiring (USB power OK at low brightness):
 *   Ring 5V IN → Nano 5V
 *   Ring DIN   → Nano D2  (Pins::LED_RING)
 *   Ring GND   → Nano GND
 *   Ring DOUT  → leave open (or to next ring's DIN if daisy-chaining)
 *
 * Full brightness only when 1–3 LEDs are lit. All-on patterns stay dim for USB.
 */

static Adafruit_NeoPixel ring(LED_RING_COUNT, Pins::LED_RING, NEO_GRB + NEO_KHZ800);

enum Phase : uint8_t {
  PHASE_PIXEL_WALK = 0,  // 1 LED full bright
  PHASE_PAIR_WALK,       // 2 LEDs full bright
  PHASE_TRIPLE_WALK,     // 3 LEDs full bright
  PHASE_COLOR_WIPE,      // fill — dim (many LEDs)
  PHASE_CHASE,           // 3 spinning LEDs full bright
  PHASE_RAINBOW,         // all-on — dim
  PHASE_BREATHE,         // all-on — dim
  PHASE_COUNT
};

static Phase phase = PHASE_PIXEL_WALK;
static uint8_t step = 0;
static uint8_t wipeColor = 0;  // 0=R 1=G 2=B
static uint16_t hue = 0;
static uint32_t lastTick = 0;
static uint32_t phaseStarted = 0;

static uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return ring.Color(r, g, b);
}

static void clearRing() {
  ring.clear();
  ring.show();
}

// NeoPixel brightness is applied on show(); set before writing pixels each frame.
static void useFullBright() { ring.setBrightness(LED_RING_BRIGHTNESS_FULL); }
static void useSafeBright() { ring.setBrightness(LED_RING_BRIGHTNESS); }

static void nextPhase(const __FlashStringHelper* name) {
  phase = static_cast<Phase>((static_cast<uint8_t>(phase) + 1) % PHASE_COUNT);
  step = 0;
  wipeColor = 0;
  hue = 0;
  phaseStarted = millis();
  clearRing();
  Serial.print(F("LED_RING phase: "));
  Serial.println(name);
}

static void enterNamedPhase() {
  switch (phase) {
    case PHASE_PIXEL_WALK:  LOG("LED_RING phase: 1 LED walk (full)"); break;
    case PHASE_PAIR_WALK:   LOG("LED_RING phase: 2 LED walk (full)"); break;
    case PHASE_TRIPLE_WALK: LOG("LED_RING phase: 3 LED walk (full)"); break;
    case PHASE_COLOR_WIPE:  LOG("LED_RING phase: color wipe R/G/B (dim)"); break;
    case PHASE_CHASE:       LOG("LED_RING phase: 3 LED chase (full)"); break;
    case PHASE_RAINBOW:     LOG("LED_RING phase: rainbow (dim)"); break;
    case PHASE_BREATHE:     LOG("LED_RING phase: breathe (dim)"); break;
    default: break;
  }
}

// Light n consecutive white LEDs starting at index, wrapping around.
static void showCluster(uint8_t start, uint8_t n, uint32_t color) {
  useFullBright();
  ring.clear();
  for (uint8_t i = 0; i < n; i++) {
    ring.setPixelColor((start + i) % LED_RING_COUNT, color);
  }
  ring.show();
}

void demoLedRingSetup() {
  ring.begin();
  useSafeBright();
  clearRing();
  phase = PHASE_PIXEL_WALK;
  step = 0;
  phaseStarted = millis();
  LOG("demo LED_RING ready (WCMCU-2812B-24 on D2)");
  LOGV("  LED count=", LED_RING_COUNT);
  LOGV("  dim brightness=", LED_RING_BRIGHTNESS);
  LOGV("  few-LED brightness=", LED_RING_BRIGHTNESS_FULL);
  enterNamedPhase();
}

void demoLedRingLoop() {
  const uint32_t now = millis();

  switch (phase) {
    case PHASE_PIXEL_WALK: {
      if (!every(120, lastTick)) return;
      showCluster(step, 1, rgb(255, 255, 255));
      step++;
      if (step >= LED_RING_COUNT) {
        nextPhase(F("2 LED walk (full)"));
      }
      break;
    }

    case PHASE_PAIR_WALK: {
      if (!every(120, lastTick)) return;
      showCluster(step, 2, rgb(255, 255, 255));
      step++;
      if (step >= LED_RING_COUNT) {
        nextPhase(F("3 LED walk (full)"));
      }
      break;
    }

    case PHASE_TRIPLE_WALK: {
      if (!every(120, lastTick)) return;
      showCluster(step, 3, rgb(255, 255, 255));
      step++;
      if (step >= LED_RING_COUNT) {
        nextPhase(F("color wipe R/G/B (dim)"));
      }
      break;
    }

    case PHASE_COLOR_WIPE: {
      if (!every(50, lastTick)) return;
      useSafeBright();
      uint32_t c = rgb(255, 0, 0);
      if (wipeColor == 1) c = rgb(0, 255, 0);
      if (wipeColor == 2) c = rgb(0, 0, 255);

      ring.setPixelColor(step, c);
      ring.show();
      step++;
      if (step >= LED_RING_COUNT) {
        step = 0;
        wipeColor++;
        if (wipeColor >= 3) {
          nextPhase(F("3 LED chase (full)"));
        }
      }
      break;
    }

    case PHASE_CHASE: {
      if (!every(60, lastTick)) return;
      useFullBright();
      ring.clear();
      ring.setPixelColor(step % LED_RING_COUNT, rgb(255, 255, 255));
      ring.setPixelColor((step + LED_RING_COUNT / 3) % LED_RING_COUNT, rgb(0, 255, 80));
      ring.setPixelColor((step + 2 * LED_RING_COUNT / 3) % LED_RING_COUNT, rgb(0, 120, 255));
      ring.show();
      step++;
      if (now - phaseStarted > 4000) {
        nextPhase(F("rainbow (dim)"));
      }
      break;
    }

    case PHASE_RAINBOW: {
      if (!every(30, lastTick)) return;
      useSafeBright();
      for (uint8_t i = 0; i < LED_RING_COUNT; i++) {
        const uint16_t pixelHue = hue + (i * 65536UL / LED_RING_COUNT);
        ring.setPixelColor(i, ring.gamma32(ring.ColorHSV(pixelHue)));
      }
      ring.show();
      hue += 256;
      if (now - phaseStarted > 6000) {
        nextPhase(F("breathe (dim)"));
      }
      break;
    }

    case PHASE_BREATHE: {
      if (!every(20, lastTick)) return;
      useSafeBright();
      const uint16_t t = (now - phaseStarted) % 2000;
      uint8_t level = (t < 1000) ? (uint8_t)(t * 255 / 1000)
                                 : (uint8_t)((2000 - t) * 255 / 1000);
      ring.fill(rgb(level, level, level));
      ring.show();
      if (now - phaseStarted > 4000) {
        nextPhase(F("1 LED walk (full)"));
      }
      break;
    }

    default:
      phase = PHASE_PIXEL_WALK;
      break;
  }
}

#endif
