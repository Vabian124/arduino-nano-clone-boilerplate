# Arduino Nano Clone Boilerplate

PlatformIO starter template for **Arduino Nano ATmega328P** boards — including common **CH340 USB clones**.

Toggle pin demos in one header, copy the examples you need, and ship.

## Features

- Pin cheat-sheet + named aliases (`include/board_pins.h`)
- Toggleable demos for each pin type (`include/config.h` + `src/demos/`)
- Small helpers: serial macros, non-blocking timers, debounce, soft blink (`include/tools.h`)
- Two upload profiles: **new Optiboot** and **old bootloader** (clone-friendly)

## Quick start

### 1. Use as a GitHub template

1. Click **Use this template** → **Create a new repository**
2. Clone your new repo
3. Open the folder in VS Code / Cursor with the PlatformIO extension

### 2. Or clone directly

```bash
git clone https://github.com/Vabian124/arduino-nano-clone-boilerplate.git
cd arduino-nano-clone-boilerplate
```

### 3. Configure & upload

1. Edit `include/config.h` — set demos to `1` / `0`
2. Set `upload_port` in `platformio.ini` (or remove it for auto-detect)
3. Upload with env **`nano_new`** (most clones with Optiboot):

```bash
pio run -e nano_new -t upload
pio device monitor -e nano_new
```

If you get `stk500_getsync() … not in sync`, try **`nano_old`** (57600 baud / old ATmegaBOOT):

```bash
pio run -e nano_old -t upload
```

**Manual reset tip:** hold **RESET**, start upload, release when you see `Uploading…`.

## Project layout

```
include/
  config.h          # enable/disable demos, baud, status interval
  board_pins.h      # Nano pin map + Pins:: aliases
  tools.h           # LOG, every(), DebouncedButton, SoftBlink, fmap
  demos.h           # demo API
src/
  main.cpp          # setup/loop entry
  tools.cpp
  demos_runner.cpp  # wires enabled demos
  demos/            # one example per pin type
platformio.ini      # nano_new (default) + nano_old
```

## Demos

| Flag in `config.h` | What it shows | Wiring |
|--------------------|---------------|--------|
| `DEMO_DIGITAL_OUT` | Digital output / blink | Onboard LED **D13** (default on) |
| `DEMO_DIGITAL_IN` | Digital input + debounce | Button **D7 → GND** |
| `DEMO_PWM` | `analogWrite` fade | LED/resistor on **D5** |
| `DEMO_ANALOG_IN` | ADC read | Pot on **A0** (5V / wiper / GND) |
| `DEMO_INTERRUPT` | External interrupt | Button **D2 → GND** (INT0) |
| `DEMO_TONE` | `tone()` beeps | Piezo on **D8** |
| `DEMO_PULSE_IN` | `pulseIn` width | Jumper **D4 → D7** |
| `DEMO_I2C_SCAN` | I2C bus scan | Devices on **A4/A5** (+ pull-ups) |
| `DEMO_SPI_LOOPBACK` | SPI transfer check | Jumper **D11 ↔ D12** |
| `DEMO_LED_RING` | Standalone WS2812B ring test | **5V / DIN→D4 / GND** |
| `DEMO_RF_RX` | PT2262 remote → LED light show (default on) | Dout→**D3**, ring DIN→**D4** |

### RF remote + LED ring

Decodes the 2-button PT2262 fob (`0xA45352` / `0xA45354` / `0xA45356`) and runs hold-based effects (ice / ember / rainbow). PulseView capture notes live in `pulseview/`.

| Signal | Nano |
|--------|------|
| Remote MCU Pin7 (PA4 Dout) | **D3** |
| LED ring DIN | **D4** |
| 5V / GND | 5V / GND |

Boot prints which demos are on and how to wire them.

## Nano pin reference (short)

| Role | Pins |
|------|------|
| UART (USB Serial) | D0 RX, D1 TX |
| External interrupts | D2 (INT0), D3 (INT1) |
| PWM (`~`) | D3, D5, D6, D9, D10, D11 |
| SPI | D10 SS, D11 MOSI, D12 MISO, D13 SCK |
| I2C | A4 SDA, A5 SCL |
| ADC | A0–A7 (`A6`/`A7` are ADC-only) |
| Onboard LED | D13 |

Full notes live in `include/board_pins.h`.

## Helpers (`tools.h`)

- `LOG("…")` / `LOGV("label=", value)` — flash-string friendly serial
- `every(ms, last)` — non-blocking interval
- `DebouncedButton` — `INPUT_PULLUP` press edges
- `SoftBlink` — blink without `delay()`
- `fmap` / `clamp` — scaling utilities

## Upload troubleshooting

| Symptom | Try |
|---------|-----|
| `stk500_getsync` / programmer not responding | Switch `nano_new` ↔ `nano_old` |
| Port busy | Close Serial Monitor / other apps on the COM port |
| No COM port | CH340/FTDI drivers; try another **data** USB cable |
| Wrong board | Confirm ATmega328P Nano (not 168 / ESP / etc.) |

## License

MIT — see [LICENSE](LICENSE).
