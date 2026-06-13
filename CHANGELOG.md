# ESP32 HAM-CLOCK Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] v1.4a - Current Development

### 🚀 New Features

- **ISS Częstotliwości Tab**
  - Dedicated tab showing ARISS amateur radio frequencies
  - Voice Repeater: RX 437.800 MHz | TX 145.990 MHz (CTCSS 67.0 Hz)
  - Packet Digipeater: 145.825 MHz Simplex (AFSK 1200)
  - SSTV Downlink: RX 145.800 MHz
  - Astronaut Voice FM: RX 145.800 | TX 145.200 MHz

- **Wireless OTA Upload Tab**
  - New "📤 Upload" tab in main navigation
  - Drag & drop file upload interface
  - Supports .html, .css, .js, .bmp, .txt files
  - Uploads directly to ESP32 LittleFS via `/update` endpoint
  - Real-time progress bar and status messages

- **ISS Pass Tracking**
  - 14 configurable TFT screens
  - Real-time telemetry display (lat/lng, altitude, speed)
  - Country detection under ISS
  - Azimuth and elevation calculations
  - SGP4 orbital calculations

### 🎨 UI/UX Improvements

- **Premium ISS Frequency Cards**
  - Cyberpunk/modern card design
  - Color-coded badges (FM, Tone, AFSK, AX.25)
  - Responsive grid layout
  - Hover effects and transitions

- **Professional Hardware Documentation**
  - Visual pinout tables with color-coded badges
  - Red alert warning for SPI pin conflict (SDO/MISO)
  - Dark-themed terminal-style deployment guide
  - PlatformIO CLI commands with step-by-step instructions

- **Dark Theme Dashboard**
  - Consistent dark color scheme (#0d1117, #1a2535)
  - Cyan accent colors (#00d4ff)
  - Golden highlights for headers (#ffd700)
  - Professional card-based layouts

### 🛠️ Hardware Guide Updates

- **Complete Pinout Documentation**
  | Component | Signal | ESP32 Pin |
  |-----------|--------|-----------|
  | Display ILI9341 (SPI) | VCC, GND, CS, RST, DC, MOSI, SCK, LED | 3V3, GND, GPIO 15, 4, 2, 23, 18 |
  | Touch XPT2046 (SPI) | T_CLK, T_DIN, T_DO, T_CS, T_IRQ | GPIO 18, 23, 19, 26, 25 |
  | I2C Sensors | SDA, SCL | GPIO 21, 22 |

- **⚠️ Critical SPI Warning**
  - Alert box highlighting SDO (MISO) pin conflict
  - XPT2046 touch panel requires separate MISO line
  - Display SDO must remain unconnected (NC)

### 🔧 Technical Improvements

- **LittleFS Integration**
  - Optimized filesystem for ESP32
  - API endpoint for diagnostics (`/api/debug`)
  - Proper cache-control headers

- **Factory Reset Enhancement**
  - NVS flash erase functionality
  - Dedicated Factory Reset button in UI

- **Build System**
  - Partition configuration (partitions.csv)
  - PlatformIO build targets: erase, upload, buildfs, uploadfs
  - Custom tft_setup.h configuration

---

## [Previous] Pre-v1.4a Baseline

*Based on repository history, the following were foundational features:*

- DX Cluster spot monitoring
- POTA (Parks on the Air) integration
- HamAlert notifications
- APRS-IS tracking
- PSK Reporter integration
- TFT display with 14 configurable screens
- Web-based configuration panel
- WiFi connectivity with fallback AP mode