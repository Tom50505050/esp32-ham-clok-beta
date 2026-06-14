# ESP32-HAM-CLOCK v1.3

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32-blue.svg)](https://www.espressif.com/en/products/socs/esp32)
[![Display: TFT](https://img.shields.io/badge/Display-TFT-green.svg)]()

Modyfikacja oryginalnego projektu **ESP32-HAM-CLOCK** autorstwa [Konrad Wiśniewski SP3KON](https://github.com/SP3KON/ESP32-HAM-CLOCK) o zaawansowaną obsługę PSK Reporter z interfejsem dotykowym.

Ten projekt jest objęty licencją MIT.

## 🎯 O projekcie

ESP32-HAM-CLOCK to zaawansowany zegar stacji amatorskiej oparty na ESP32 z obsługą wyświetlacza TFT. Projekt integruje wiele źródeł danych dla radioamatorów:

- **Zegar i kalendarz** z fazami księżyca
- **DX Cluster** - spoty z całego świata
- **APRS** - monitoring pakietów APRS
- **POTA** - aktywacje Parków na Powietrzu
- **HamAlert** - powiadomienia o spotach
- **PSK Reporter** - mapa aktywności cyfrowych trybów (FT8, FT4, JS8, itp.)
- **Propagacja** - warunki propagacyjne i indice

## ✨ Nowości w wersji 1.3 (vs 1.2b)

### Naprawa kolejności ekranów TFT
- **Naprawiono zapisywanie kolejności** - usunięto błąd nadpisujący ustawienia użytkownika domyślną kolejnością przy restarcie ESP32
- **Dodano 12 slotów** dla ekranów TFT w interfejsie WWW (wcześniej brakowało pełnej obsługi)
- **Zsynchronizowano typy ekranów** - wszystkie 14 typów dostępnych w firmware: HAM CLOCK, DX CLUSTER, APRS-IS, APRS RADAR, BAND INFO, SUN SPOTS, WEATHER, WEATHER FORECAST, POTA, HAMALERT, PSK MAP, UNLIS HUNTER, MATRIX, OFF

### Aktualizacja dokumentacji i licencji
- **Dodano plik LICENSE** z pełną treścią licencji MIT
- **Poprawiono autora** - Konrad Wiśniewski SP3KON
- **Zaktualizowano instrukcje** (INSTRUKCJA_MONT.txt, INSTRUKCJA_WGRANIA_BIN.md)

## ✨ Nowości w wersji 1.2b

### Rozszerzony PSK Reporter
- **Filtrowanie spotów** po paśmie, trybie, znakie odbiornika/nadawcy
- **Interfejs dotykowy** na ekranie TFT - menu, klawiatura ekranowa, selektory
- **Zapis ustawień** do pamięci NVS (nieulotnej)
- **Automatyczne odświeżanie** z konfigurowalnym interwałem

### Informacje systemowe w WWW
- Adres IP, SSID, RSSI
- Wolna pamięć RAM i LittleFS
- Temperatura ESP32 i napięcie baterii
- Uptime systemu
- Wersja firmware

### Interfejs dotykowy TFT
- Przycisk menu (≡) na mapie PSK
- Klawiatura ekranowa dla wprowadzania znaków i liczb
- Selektory pasm (160m-6m) i trybów (FT8, FT4, CW, itp.)
- Potwierdzenie zapisu ustawień

## 🔧 Wymagania sprzętowe

| Komponent | Specyfikacja |
|-----------|-------------|
| Mikrokontroler | ESP32 (ESP32-2432S028R) |
| Wyświetlacz | TN3.5 Cal ILI9488 480x320 |
| Touchscreen | XPT2046 |
| Pamięć | 4MB Flash |
| Zasilanie | 5V USB lub akumulator 18650 + TP4056 |

### Mapa pinów (domyślna)

```
TFT:
- MISO: GPIO 19
- MOSI: GPIO 23
- SCLK: GPIO 18
- CS:   GPIO 15
- DC:   GPIO 2
- RST:  NC (-1)

Touchscreen XPT2046:
- CS:   GPIO 21
- IRQ:  GPIO 27

Akumulator (opcjonalnie):
- ADC:  GPIO 34
```

## 🚀 Instalacja

### Metoda 1: PlatformIO (zalecana)

1. Zainstaluj [VS Code](https://code.visualstudio.com/) + [PlatformIO](https://platformio.org/)
2. Sklonuj repozytorium:
   ```bash
   git clone https://github.com/Tom50505050/esp32-ham-clock.git
   cd esp32-ham-clock
   ```
3. Otwórz projekt w VS Code
4. Podłącz ESP32 przez USB
5. Kliknij **Upload** (strzałka w dół) w PlatformIO

### Metoda 2: WebSerial ESPTool (bez kompilacji)

1. Wejdź na: https://jason2866.github.io/WebSerial_ESPTool/
2. Podłącz ESP32 przez USB
3. Wybierz pliki z folderu `build/esp32.esp32.esp32/`:
   - `bootloader.bin` → `0x1000`
   - `partitions.bin` → `0x8000`
   - `firmware.bin` → `0x10000`
   - `littlefs.bin` → `0x290000`
4. Kliknij **Program**

## ⚙️ Konfiguracja

Po pierwszym uruchomieniu ESP32 utworzy sieć WiFi `ESP32-HAM-CLOCK`:

1. Połącz się z siecią WiFi `ESP32-HAM-CLOCK`
2. Otwórz przeglądarkę i wejdź na: `http://192.168.4.1`
3. Skonfiguruj:
   - **WiFi** - SSID i hasło Twojej sieci
   - **Callsign** - Twój znak amatorski
   - **Locator** - lokalizator QTH (np. JO90GA)
   - **DX Cluster** - host i port
   - **APRS** - host, port, filter
   - **HamAlert** - login i hasło
   - **PSK Reporter** - znak do monitorowania, pasmo, tryb, max spotów

## 🖥️ Interfejs WWW

Po połączeniu z siecią WiFi, ESP32 wyświetli adres IP na ekranie TFT. Wejdź na ten adres w przeglądarce aby uzyskać dostęp do:

- **Konfiguracja** - ustawienia wszystkich parametrów
- **Status** - informacje o systemie w czasie rzeczywistym
- **API** - endpointy REST dla danych JSON
- **Instrukcja** - lokalna kopia instrukcji

## 📁 Struktura projektu

```
ESP32-HAM-CLOCK/
├── src/
│   └── main.cpp              # Główny kod źródłowy
├── data/
│   ├── index.html            # Interfejs WWW (LittleFS)
│   ├── instrukcja.txt        # Instrukcja PL
│   ├── manual.txt            # Instrukcja EN
│   └── fonts/, icon50/       # Zasoby graficzne
├── platformio.ini            # Konfiguracja PlatformIO
├── CHANGELOG.md              # Historia zmian
├── INSTRUKCJA_MONT.txt       # Instrukcja montażu
├── INSTRUKCJA_WGRANIA_BIN.md # Instrukcja wgrywania
└── README.md                 # Ten plik
```

## 🌐 API REST

Dostępne endpointy:

| Endpoint | Opis |
|----------|------|
| `GET /api/config` | Pobierz konfigurację |
| `POST /api/config` | Zapisz konfigurację |
| `GET /api/spots` | Pobierz spoty DX |
| `GET /api/psk` | Pobierz dane PSK Reporter |
| `GET /api/system` | Informacje systemowe |
| `GET /instruction` | Instrukcja obsługi |

## 🎨 Zmiana grafiki strony startowej

Ekran startowy (splash screen) wyświetla plik BMP z katalogu `icon50/`. Domyślnie jest to `splash.bmp`.

### Wymagania pliku BMP:
- **Rozdzielczość:** 480x320 (TN3.5 Cal ILI9488)
- **Format:** BMP 24-bit (RGB) lub 16-bit (RGB565)
- **Kompresja:** Brak (uncompressed)
- **Nazwa pliku:** `splash.bmp`

### Jak zmienić grafikę:

1. **Przygotuj obraz** w programie graficznym (Photoshop, GIMP, Paint.NET)
   - Ustaw rozdzielczość 480x320 (dla TN3.5 Cal ILI9488)
   - Zapisz jako BMP 24-bit bez kompresji

2. **Zamień plik** w folderze projektu:
   ```
   data/icon50/splash.bmp
   ```
   lub
   ```
   littlefs_data/icon50/splash.bmp
   ```

3. **Wgraj LittleFS** na ESP32:
   - W PlatformIO: **Project Tasks → Upload File System Image**
   - Lub przez terminal:
     ```bash
     pio run --target uploadfs
     ```

4. **Zrestartuj ESP32** - nowa grafika pojawi się przy kolejnym uruchomieniu

### Konfiguracja czasu wyświetlania:
W pliku `src/main.cpp` zmień wartość (domyślnie 10 sekund):
```cpp
while (!splashSkipped && (millis() - splashStart) < 10000) {
```

### Alternatywa - wyłączenie splash screen:
Jeśli nie chcesz wyświetlać splash screen, usuń lub zmień nazwę pliku `splash.bmp`. ESP32 wyświetli wtedy czarny ekran z ikoną baterii.

## 📝 Licencja

Ten projekt jest objęty licencją **MIT**.

**Autor oryginału:** [Krzysztof Błaszczyk SP3KON](https://github.com/SP3KON/ESP32-HAM-CLOCK)

**Modyfikacje:** Tomasz [SP9TNV](https://github.com/Tom50505050)

Szczegóły w pliku [CHANGELOG.md](CHANGELOG.md).

## 🙏 Podziękowania

- **SP3KON** - oryginalny projekt i baza kodowa
- **PSK Reporter** - API do monitoringu cyfrowych trybów
- **HamAlert** - system powiadomień dla radioamatorów
- **POTA** - program Parków na Powietrzu
- **Społeczność** - wszystkie biblioteki open-source użyte w projekcie

## 📞 Kontakt

**Autor oryginału:**
- **Email:** sp3kon@gmail.com
- **GitHub:** [SP3KON/ESP32-HAM-CLOCK](https://github.com/SP3KON/ESP32-HAM-CLOCK)

**Modyfikacje:**
- **Email:** SP9TNV@gmail.com
- **GitHub:** [Tom50505050/esp32-ham-clock](https://github.com/Tom50505050/esp32-ham-clock)

---

**73!** 📻
