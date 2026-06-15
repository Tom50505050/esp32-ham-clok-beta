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

## 🚀 METODY WGRYWANIA

ESP32 HAM-CLOCK można wgrać na **4 sposoby**. Wybierz najbardziej odpowiedni dla siebie.

---

### 📋 Metoda 1: PlatformIO CLI (Konsola)

**Zalety:** Pełna kontrola, automatyczne czyszczenie, najszybszy sposób dla zaawansowanych.

**Krok po kroku:**

```powershell
# 1. Otwórz terminal w folderze projektu
cd "C:\Twoja\sciezka\esp32-ham-clok-beta"

# 2. Pobierz najnowszy interfejs web z GitHub
Invoke-WebRequest -Uri "https://raw.githubusercontent.com/Tom50505050/esp32-ham-clok-beta/main/littlefs_data/index.html" -OutFile "littlefs_data\index.html"

# 3. Pełne czyszczenie pamięci Flash ESP32
pio run --target erase

# 4. Kompilacja i wgrywanie firmware (program główny)
pio run --target upload

# 5. Budowanie obrazu systemu plików LittleFS
pio run --target buildfs

# 6. Wgrywanie plików web na ESP32
pio run --target uploadfs
```

> ⚠️ **Uwaga:** Między krokami 3 i 4 poczekaj aż ESP32 się zresetuje (ok. 5 sekund).

---

### 🖥️ Metoda 2: PlatformIO GUI (Visual Studio Code)

**Zalety:** Intuicyjny interfejs graficzny, nie wymaga wpisywania komend.

#### Krok 1: Otwórz projekt w VS Code
```
Plik → Otwórz folder → wybierz folder esp32-ham-clok-beta
```

#### Krok 2: Zainstaluj rozszerzenie PlatformIO
```
Widok → Rozszerzenia → wyszukaj "PlatformIO IDE" → Zainstaluj
```

#### Krok 3: Konfiguracja portu COM
```
Otwórz platformio.ini → zmień upload_port na swój port np. COM3
```

#### Krok 4: Wgrywanie przez ikony PlatformIO

| Ikona | Funkcja | Opis |
|-------|---------|------|
| ➡️ | **Upload** | Wgrywa firmware (.bin) |
| 🗑️ | **Erase** | Czyści pamięć Flash |
| 📦 | **Build Filesystem Image** | Tworzy obraz LittleFS |
| ⬆️ | **Upload Filesystem Image** | Wgrywa pliki web |

```
Kolejność kliknięć:
1. 🗑️ Erase Flash (czyszczenie)
2. ➡️ Upload (firmware)
3. 📦 Build Filesystem Image
4. ⬆️ Upload Filesystem Image
```

#### Lokalizacja ikon:
```
┌─────────────────────────────────────────────────┐
│  [🏠] [✓] [➡️] [🗑️] [📦] [⬆️] [🔧] [🤖]        │
│   Home Compile Upload Erase Build Upload Settings│
└─────────────────────────────────────────────────┘
```

---

### 📡 Metoda 3: Web OTA (Przez WiFi)

**Zalety:** Bez kabla USB, wgrywanie zdalne przez sieć lokalną.

#### Wymagania wstępne:
1. ESP32 musi być już wgrany z poprzedniej metody
2. ESP32 musi być połączony z tą samą siecią WiFi

#### Krok po kroku:

**1. Otwórz interfejs web ESP32:**
```
Przeglądarka → http://192.168.X.X (IP ESP32)
```

**2. Przejdź do zakładki UPLOAD:**
```
[NAVIGACJA] → Kliknij przycisk "UPLOAD" w górnym menu
```

**3. Wgraj plik firmware:**
```
┌─────────────────────────────────────────┐
│         📤 UPLOAD PLIKÓW               │
├─────────────────────────────────────────┤
│                                         │
│    ┌───────────────────────────────┐   │
│    │   📁                           │   │
│    │   Przeciągnij pliki tutaj     │   │
│    │   lub kliknij aby wybrać      │   │
│    └───────────────────────────────┘   │
│                                         │
│    📄 Wybierz plik: firmware.bin       │
│                                         │
│    [████████░░░░░░░░░░] 65%           │
│    Status: Wgrywanie: firmware.bin     │
│                                         │
└─────────────────────────────────────────┘
```

**4. Wgraj pliki LittleFS:**
```
Przeciągnij i upuść wszystkie pliki z folderu littlefs_data/
(lub użyj przycisku do wyboru plików)
```

#### Obsługiwane formaty plików:
| Format | Rozszerzenie | Opis |
|--------|--------------|------|
| HTML | `.html` | Strony internetowe |
| CSS | `.css` | Style |
| JavaScript | `.js` | Skrypty |
| Obrazy | `.bmp`, `.png` | Grafiki, ikony |
| Czcionki | `.bin` | Fonty |
| Tekst | `.txt` | Dokumentacja |
| **Markdown** | **`.md`** | **Pliki instrukcji - automatycznie pojawiają się w zakładce Instrukcja** |

---

### 🔧 Metoda 4: esptool.py (Bez PlatformIO)

**Zalety:** Działa z dowolnym edytorem, nie wymaga PlatformIO.

#### Instalacja esptool:
```powershell
pip install esptool
```

#### Krok po kroku:

```powershell
# 1. Pobierz firmware z folderu .pio/build/esp32dev/
# (po kompilacji w PlatformIO lub pobierz z GitHub releases)

# 2. Zidentyfikuj port COM
python -m esptool --port COM3 flash_id

# 3. Skasuj pamięć Flash
python -m esptool --port COM3 erase_flash

# 4. Wgraj bootloader
python -m esptool --port COM3 write_flash 0x1000 bootloader.bin

# 5. Wgraj partition table
python -m esptool --port COM3 write_flash 0x8000 partitions.bin

# 6. Wgraj firmware główny
python -m esptool --port COM3 write_flash 0x10000 firmware.bin

# 7. Wgraj system plików LittleFS
python -m esptool --port COM3 write_flash 0x390000 littlefs.bin
```

#### Szybka komenda (wszystko jednocześnie):
```powershell
python -m esptool --port COM3 \
  erase_flash \
  write_flash 0x1000 bootloader.bin \
              0x8000 partitions.bin \
              0x10000 firmware.bin \
              0x390000 littlefs.bin
```

---

## 🔄 SEKWENCJA WGRYWANIA - SCHEMAT

```
┌─────────────────────────────────────────────────────────────────┐
│                    SEKWENCJA PEŁNEGO WGRYWANIA                  │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│  1️⃣ ERASE FLASH                                                │
│     Czyści całą pamięć Flash                                    │
│     └─→ Komenda: pio run --target erase                         │
│     └─→ Czas: ~10 sekund                                        │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│  2️⃣ UPLOAD FIRMWARE                                             │
│     Wgrywa program główny (.bin)                                │
│     └─→ Komenda: pio run --target upload                        │
│     └─→ Czas: ~30-60 sekund                                     │
│     └─→ Rozmiar: ~1.5 MB                                        │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│  3️⃣ BUILD FILESYSTEM                                           │
│     Tworzy obraz LittleFS z plików web                          │
│     └─→ Komenda: pio run --target buildfs                       │
│     └─→ Czas: ~5-10 sekund                                      │
│     └─→ Tworzy: littlefs.bin                                    │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│  4️⃣ UPLOAD FILESYSTEM                                          │
│     Wgrywa pliki web na ESP32                                   │
│     └─→ Komenda: pio run --target uploadfs                      │
│     └─→ Czas: ~20-30 sekund                                     │
│     └─→ Rozmiar: ~2 MB                                          │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│  ✅ GOTOWE!                                                      │
│                                                                 │
│     ESP32 uruchomi się automatycznie                            │
│     Utworzy sieć WiFi: ESP32-HAM-CLOCK                          │
│     Hasło: 1234567890                                           │
│     Adres AP: http://192.168.4.1                                │
└─────────────────────────────────────────────────────────────────┘
```

---

## ⚡ ROZWIĄZYWANIE PROBLEMÓW

### Problem: ESP32 nie jest wykrywane przez USB
```
✅ Sprawdź kabel USB (użyj kabla z danymi, nie tylko ładowania)
✅ Sprawdź sterowniki CH340/CP2102 w Menedżerze urządzeń
✅ Spróbuj innego portu USB
✅ Naciśnij przycisk BOOT na ESP32 podczas podłączania
```

### Problem: Błąd "Failed to connect to ESP32"
```
✅ Przytrzymaj przycisk BOOT podczas startu wgrywania
✅ Zwolnij BOOT gdy zobaczysz "Connecting..."
✅ Spróbuj niższej prędkości upload: upload_speed = 460800
```

### Problem: "LittleFS mount failed"
```
✅ Upewnij się, że wykonałeś krok buildfs przed uploadfs
✅ Sprawdź czy partycja w partitions.csv ma właściwy rozmiar
✅ Wykonaj pełny erase przed ponownym wgraniem
```

### Problem: ESP32 nie łączy się z WiFi
```
✅ Przytrzymaj przycisk BOOT + EN przez 10 sekund (reset do ustawień fabrycznych)
✅ ESP32 utworzy własną sieć AP jako fallback
✅ Sprawdź poprawność SSID i hasła w konfiguracji
```

### Problem: Czarny ekran TFT
```
✅ Sprawdź połączenia kabli FPC
✅ Sprawdź pin RST - musi być podłączony do 3.3V
✅ Spróbuj innej wartości TFT_BL_PIN w tft_setup.h
```

---

## 📁 LOKALIZACJA PLIKÓW PO KOMPILACJI

### Po kompilacji (pio run)

```
esp32-ham-clok-beta/
├── .pio/
│   └── build/
│       └── esp32dev/
│           ├── firmware.bin          ← Główny firmware
│           ├── bootloader.bin        ← Bootloader
│           ├── partitions.bin        ← Tablica partycji
│           └── littlefs.bin          ← System plików (po buildfs)
│
├── src/
│   └── main.cpp                      ← Kod źródłowy
│
└── littlefs_data/
    ├── index.html                    ← Strona główna web
    ├── fonts/                        ← Czcionki
    └── icons/                        ← Ikony
```

### Po uruchomieniu rename_bins.bat

```
.pio\build\esp32dev\
├── firmware_0x10000.bin     ← Adres: 0x10000 (główny program)
├── bootloader_0x1000.bin    ← Adres: 0x1000 (bootloader)
├── partitions_0x8000.bin    ← Adres: 0x8000 (partycje)
└── littlefs_0x390000.bin    ← Adres: 0x390000 (pliki web)
```

---

## 📋 PLIKI Binarne - SCHEMAT PAMIĘCI FLASH (4MB)

```
┌─────────────────────────────────────────────────────────────────┐
│                    PAMIĘĆ FLASH ESP32 (4MB)                     │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ 0x1000 (4KB)     bootloader_0x1000.bin   Bootloader             │
├─────────────────────────────────────────────────────────────────┤
│ 0x8000 (4KB)     partitions_0x8000.bin   Tablica partycji       │
├─────────────────────────────────────────────────────────────────┤
│ 0x10000 (~1.5MB) firmware_0x10000.bin   Główny program (APP)    │
│    ...                                                          │
├─────────────────────────────────────────────────────────────────┤
│ 0x390000 (~2MB)  littlefs_0x390000.bin  System plików (web UI)  │
│    ...                                                          │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🚀 SKRYPTY WGRYWANIA (.bat)

### 📌 Kolejność kroków:

```
1. pio run --target buildfs     ← Zbuduj LittleFS
2. rename_bins.bat              ← Zmień nazwy na _0xADDR.bin
3. Uruchom odpowiedni skrypt:
   ├── flash_all.bat            ← Pełne wgranie (wszystko)
   ├── flash_firmware.bat       ← Tylko firmware
   └── flash_littlefs.bat       ← Tylko pliki web
```

### 📝 Opis skryptów:

| Skrypt | Co robi | Kiedy użyć |
|--------|---------|------------|
| `rename_bins.bat` | Zmienia nazwy plików .bin dodając adres | **Zawsze pierwszy** |
| `flash_all.bat` | Czyści flash + wgrywa wszystko | Pierwsze wgranie / recovery |
| `flash_firmware.bat` | Wgrywa bootloader + firmware | Aktualizacja programu |
| `flash_littlefs.bat` | Wgrywa tylko LittleFS | Aktualizacja strony www |

---

## 🔧 KONFIGURACJA PLATFORMIO

W pliku `platformio.ini` możesz dostosować:

| Parametr | Opis | Domyślna wartość |
|----------|------|------------------|
| `upload_speed` | Prędkość wgrywania | 115200 |
| `monitor_speed` | Prędkość serial monitor | 115200 |
| `upload_port` | Port COM | COM3 |
| `board_build.f_cpu` | Częstotliwość CPU | 240MHz |

---

## 🌐 PIERWSZE URUCHOMIENIE

### 1. Tryb Access Point (AP)
Po pierwszym wgraniu ESP32 automatycznie:
```
📶 Sieć WiFi: ESP32-HAM-CLOCK
🔐 Hasło: 1234567890
🌐 Adres: http://192.168.4.1
```

### 2. Konfiguracja WiFi
1. Połącz się z siecią ESP32-HAM-CLOCK
2. Otwórz przeglądarkę: http://192.168.4.1
3. Przejdź do zakładki **USTAWIENIA**
4. Wpisz dane swojej sieci WiFi (SSID i hasło)
5. Zapisz i zrestartuj ESP32

### 3. Tryb Station (ST)
Po połączeniu z routerem:
```
📶 ESP32 połączy się z Twoją siecią
🌐 Adres IP znajdziesz w routerze lub przez Serial Monitor
```

### 4. Serial Monitor
Aby zobaczyć logi debugowania:
```powershell
pio device monitor
```

---

## 📊 SCHEMAT PINÓW (GRAFIKA)

```
        ┌─────────────────────────────┐
        │         ESP32 DevKit        │
        │         (ESP-WROOM-32)      │
        └─────────────────────────────┘
        
        GPIO 2  ──────┬────── DC (TFT)
        GPIO 15 ──────┼────── CS (TFT)  
        GPIO 18 ──────┼────── SCK (TFT + Touch)
        GPIO 19 ──────┼────── MISO (Touch)
        GPIO 21 ──────┼────── T_CS (Touch)
        GPIO 22 ──────┼────── T_IRQ (Touch)
        GPIO 23 ──────┼────── MOSI (TFT + Touch)
        GPIO 32 ──────┼────── TFT BL (Backlight)
        
        ⚠️ GPIO 19 (MISO TFT) = NIE PODŁĄCZAJ! (NC)
        
        3V3 ──────────┴────── VCC (TFT + Touch)
        GND ──────────┴────── GND (TFT + Touch)
```

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