# LaserDriver-ESP32

Arduino ESP32 project for laser driver control with built-in ST7789 LCD display.

## Serial Commands

Response to each command is preceded by its echo (prefixed with `#`).

| Command | Parameters | Return |
|---------|------------|--------|
| `whois?` | none | `Laser lock by BGMAGLAB` |
| `sweep` | start, stop | none |
| `lock` | x, y, error or OFF | none |
| `lock?` | none | current lock point or "unlocked" |
| `PID` | P I D | P I D |
| `PID?` | none | P I D |
| `help` or `?` | none | Help text |

### Command Details

- **`whois?`** - Identify device when establishing connection
  - Example: `whois?`
  - Response: `Laser lock by BGMAGLAB`

- **`sweep <start> <stop>`** - Set sweep limits (0-65535)
  - Example: `sweep 1234 39865`
  - Sets DAC sweep range from start to stop values

- **`lock <value>`** - Set PID lock point (0-65535)
  - Example: `lock 2256`
  - Tells the PID controller where to lock the laser

- **`PID <P> <I> <D>`** - Set PID parameters
  - Example: `PID 52 64 35`
  - Returns the new accepted PID values

- **`PID?`** - Get current PID parameters
  - Example: `PID?`
  - Returns current P, I, D values

- **`help`** or **`?`** - Display command reference
  - Shows all available commands


## Hardware

- **Board**: Ideaspark ESP32 with 1.9" LCD (170x320)
- **Display Driver**: ST7789
- **Serial Speed**: 115200 baud
- **USB**: Type-C

## Pin Connections

### ST7789 LCD Display (Hardware SPI)
| Pin | GPIO | Description |
|-----|------|-------------|
| MOSI | GPIO23 | SPI Data Out |
| SCLK | GPIO18 | SPI Clock |
| CS | GPIO15 | Chip Select |
| DC | GPIO2 | Data/Command |
| RST | GPIO4 | Reset |
| BL | GPIO32 | Backlight Control |

### I2C Bus (DAC60501 Reference)
| Pin | GPIO | Description |
|-----|------|-------------|
| SDA | GPIO21 | I2C Data |
| SCL | GPIO22 | I2C Clock |
| Address | 0x48 | DAC60501 I2C Address |

### DAC8554 4-Channel DAC (Software SPI)
| Pin | GPIO | Description |
|-----|------|-------------|
| MOSI | GPIO26 | SPI Data Out |
| SCK | GPIO27 | SPI Clock |
| CS | GPIO5 | Chip Select |
| Output Range | - | 0-1.0V (4 channels) |

### ADC Input
| Pin | GPIO | Description |
|-----|------|-------------|
| ADC | GPIO34 | ADC1_CH6 - Photodiode/Error Signal |
| Range | - | 0-3.3V, 12-bit resolution |
| Sampling | - | 10kHz with 17-tap FIR filter |

## Requirements

- PlatformIO IDE extension for VS Code
- TFT_eSPI library (auto-installed)

## Getting Started

1. Connect ESP32 board via USB Type-C
2. Build: `pio run`
3. Upload: `pio run --target upload`
4. Monitor: `pio device monitor`

## Current Features

- **DAC60501** - 1V precision reference voltage via I2C
- **DAC8554** - 4-channel 16-bit DAC (0-1V range) via software SPI
- **ADC with FIR filter** - 10kHz sampling with 17-tap low-pass filter
- **ST7789 LCD display** - Real-time sweep plot visualization
- **Serial command interface** - Remote control via USB
- **Configurable sweep** - 200-point sweep at 1kHz sample rate

## Display Usage

```cpp
tft.fillScreen(TFT_BLACK);
tft.setTextColor(TFT_WHITE);
tft.setTextSize(2);
tft.setCursor(x, y);
tft.println("Text");
```

Display resolution: 170x320 pixels
