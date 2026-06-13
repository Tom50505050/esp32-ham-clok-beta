# ESP32 HAM-CLOCK v1.4a

![ESP32](https://img.shields.io/badge/ESP32-WROOM-32-blue?style=for-the-badge)
![PlatformIO](https://img.shields.io/badge/PlatformIO-IDE-green?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)

**Zaawansowany monitor radiostacji dla krótkofalowców (ham radio) oparty na ESP32 z wyświetlaczem TFT.**

---

## 🎯 Funkcje

### 📻 Moduły Radiowe
- **DX Cluster** — Śledzenie spotów DX w czasie rzeczywistym
- **POTA Spots** — Monitorowanie aktywności Parks on the Air
- **HamAlert** — Alerty o aktywności radiowej
- **APRS-IS** — System pozycjonowania radiowego
- **PSK Reporter** — Śledzenie emisji cyfrowych (FT8, FT4, CW, RTTY)

### 🛰️ ISS Tracking
- **ISS Częstotliwości ARISS** — Karty z częstotliwościami:
  - 🔁 Voice Repeater: RX 437.800 | TX 145.990 MHz (CTCSS 67.0 Hz)
  - 📦 APRS Digipeater: 145.825 MHz simplex
  - 🖼️ SSTV: RX 145.800 MHz
  - 🎙️ Voice Direct EU: RX 145.800 | TX 145.200 MHz
- **ISS Pass Tracking** — Pozycja, wysokość, prędkość, kraj pod satelitą

### 📺 Wyświetlacz TFT
- **14 konfigurowalnych ekranów** z automatycznym przełączaniem
- Wyświetlacz ILI9341 3.2" 480x320
- Panel dotykowy XPT2046

### 🌐 Interfejs Web
- **Zakładka Upload** — Bezprzewodowa aktualizacja OTA przez WiFi
- Ciemny, profesjonalny motyw
- Responsywny design

---

## 🔌 Schemat Połączeń

### ⚠️ WAŻNE OSTRZEŻENIE
> Nie podłączaj pinu SDO (MISO/SDOK) z wyświetlacza TFT! Ten pin musi zostać wolny (NC), aby panel dotykowy XPT2046 mógł prawidłowo komunikować się przez magistralę SPI.

### Wyświetlacz TFT (ILI9341)
| Sygnał | ESP32 Pin |
|--------|-----------|
| VCC | 3V3 |
| GND | GND |
| MOSI | GPIO 23 |
| MISO | GPIO 19 |
| SCK | GPIO 18 |
| CS | GPIO 15 |
| DC | GPIO 2 |
| RST | Nie używany (podłączony do 3.3V) |

### Panel Dotykowy (XPT2046)
| Sygnał | ESP32 Pin |
|--------|-----------|
| T_MOSI (DIN) | GPIO 23 (współdzielone z TFT) |
| T_MISO (DO) | GPIO 19 (współdzielone z TFT) |
| T_CLK | GPIO 18 (współdzielone z TFT) |
| T_CS | GPIO 21 |
| T_IRQ | GPIO 22 |

### Moduł I2C (BME280 / RTC)
| Sygnał | ESP32 Pin |
|--------|-----------|
| SDA | GPIO 21 |
| SCL | GPIO 22 |

---

## 💻 Instalacja

### Wymagania
- Visual Studio Code z rozszerzeniem PlatformIO IDE
- Płytka ESP32 DevKit
- Kabel USB

### Krok po kroku

```powershell
# 1. Przejdź do folderu projektu
cd "C:\Sciezka\Do\Twojego\Projektu\esp32-ham-clock"

# 2. Pobierz najnowszy interfejs z GitHub
Invoke-WebRequest -Uri "https://raw.githubusercontent.com/Tom50505050/esp32-ham-clok-beta/main/littlefs_data/index.html" -OutFile "littlefs_data\index.html"

# 3. Pełne czyszczenie pamięci Flash
pio run --target erase

# 4. Kompilacja i wgrywanie programu (Firmware)
pio run --target upload

# 5. Budowanie obrazu strony WWW (LittleFS)
pio run --target buildfs

# 6. Wgrywanie plików interfejsu na płytkę
pio run --target uploadfs
```

### Połączenie z ESP32
1. Po wgraniu, ESP32 utworzy sieć WiFi: **ESP32-HAM-CLOCK**
2. Hasło: **1234567890**
3. Otwórz przeglądarkę: `http://192.168.4.1`
4. Lub połącz z routerem i otwórz `http://<IP_ESP32>`

---

## 📁 Struktura Projektu

```
esp32-ham-clock/
├── src/
│   ├── main.cpp          # Główny kod firmware
│   └── tft_setup.h       # Konfiguracja TFT
├── littlefs_data/
│   ├── index.html        # Interfejs web
│   ├── fonts/            # Czcionki
│   ├── icon50/           # Ikony pogody
│   └── splash.bmp        # Obraz startowy
├── partitions.csv        # Konfiguracja partycji
├── platformio.ini        # Konfiguracja PlatformIO
└── CHANGELOG.md          # Historia zmian
```

---

## 🔧 Konfiguracja

### Ustawienia WiFi
1. Otwórz zakładkę "Ustawienia" w interfejsie web
2. Wprowadź dane sieci WiFi (SSID i hasło)
3. Zapisz ustawienia

### Konfiguracja APRS
- Callsign APRS
- SSID
- Hasło APRS (getpass.io)

### Konfiguracja DX Cluster
- Adres serwera DX
- Port
- Filtr spotów

---

## 📜 Licencja

MIT License — zobacz plik LICENSE dla szczegółów.

---

## 👤 Autor

**Tom50505050**
- GitHub: [Tom50505050/esp32-ham-clok-beta](https://github.com/Tom50505050/esp32-ham-clok-beta)

---

## 🙏 Podziękowania

- Biblioteka [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI)
- Biblioteka [SGP4](https://github.com/m NEVERind/SGP4)
- Projekt [APRS-IS](https://www.aprs-is.net/)
- [ARISS](https://www.ariss.org/) — Amateur Radio on ISS