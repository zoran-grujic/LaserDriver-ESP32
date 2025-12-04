# LaserDriver-ESP32

Arduino ESP32 project for laser driver control with built-in ST7789 LCD display.

## Hardware

- **Board**: Ideaspark ESP32 with 1.9" LCD (170x320)
- **Display Driver**: ST7789
- **Serial Speed**: 115200 baud
- **USB**: Type-C

### Display Pins (SPI)
- MOSI: GPIO23
- SCLK: GPIO18
- CS: GPIO15
- DC: GPIO2
- RST: GPIO4
- Backlight: GPIO32

## Requirements

- PlatformIO IDE extension for VS Code
- TFT_eSPI library (auto-installed)

## Getting Started

1. Connect ESP32 board via USB Type-C
2. Build: `pio run`
3. Upload: `pio run --target upload`
4. Monitor: `pio device monitor`

## Current Features

- ST7789 LCD display initialization
- Color screen test (red, green, blue)
- Text display with counter
- Serial debug output

## Display Usage

```cpp
tft.fillScreen(TFT_BLACK);
tft.setTextColor(TFT_WHITE);
tft.setTextSize(2);
tft.setCursor(x, y);
tft.println("Text");
```

Display resolution: 170x320 pixels
