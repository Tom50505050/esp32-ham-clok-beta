# INSTRUKCJA_WGRANIA_BIN.md
# ESP32-HAM-CLOCK v1.3a - UJEDNOLICONA INSTRUKCJA MONTAŻU I WGRANIA FIRMWARE

Niniejszy dokument stanowi oficjalną i kompletną specyfikację techniczną projektu w wersji v1.3a. Wszystkie procedury, tabele oraz konfiguracje pinów zostały w pełni zsynchronizowane z poprawnym schematem sprzętowym, w którym magistrala SPI dla panelu dotykowego oraz wyświetlacza została zintegrowana, a pin **SDO/MISO wyświetlacza pozostał NIEPODŁĄCZONY**.

---

## 1. SPECYFIKACJA SPRZĘTOWA

### Wymagane komponenty:
* **Mikrokontroler:** ESP32-DEVKIT V1 (38 pinów) - seria `-D0WD-V3`
* **Wyświetlacz:** ILI9488 TFT 3.5" (480x320)
* **Panel dotykowy:** XPT2046 (zintegrowany konstrukcyjnie z modułem wyświetlacza)
* **Zasilanie:** Stabilne źródło zasilania 5V/1A za pomocą złącza USB lub dedykowanego modułu zasilającego (zalecany zewnętrzny zasilacz w celu uniknięcia tętnień napięcia).
* **Filtrowanie:** Kondensatory 100nF zamontowane blisko linii zasilających moduł wyświetlacza.

---

## 2. JEDYNY POPRAWNY SCHEMAT POŁĄCZEŃ (PINOUT)

Wyświetlacz ILI9488 oraz panel dotykowy XPT2046 współdzielą fizyczne linie sygnałowe magistrali SPI (SCK oraz MOSI). Pin **SDO/MISO wyświetlacza nie wysyła danych do procesora i należy go pozostawić pustym**. Sygnał MISO mikrokontrolera (GPIO 19) pobiera informacje wyłącznie z linii wyjściowej dotyku (`T_DO`).

### Połączenie Wyświetlacza ILI9488 (9 połączeń)
| ILI9488 PIN | ESP32-D PIN | Kolor przewodu (sugerowany) | Uwagi i funkcja |
| :--- | :--- | :--- | :--- |
| **VCC (5V)** | 5V / VIN | CZERWONY | Zasilanie główne układu |
| **GND** | GND | CZARNY | Masa wspólna układu |
| **CS** | GPIO 15 | FIOLETOWY | Chip Select – aktywacja wyświetlacza |
| **RESET** | EN | ŻÓŁTY | Reset sprzętowy powiązany z mikrokontrolerem |
| **DC/RS** | GPIO 2 | POMARAŃCZOWY | Wybór: Dane / Komendy (Data/Command) |
| **SDI/MOSI** | GPIO 23 | NIEBIESKI | **Wspólna linia danych SPI** (MOSI) z dotykiem |
| **SCK** | GPIO 18 | ZIELONY | **Wspólna linia zegarowa SPI** (SCK) z dotykiem |
| **LED** | GPIO 32 | BIAŁY | Sterowanie podświetleniem ekranu (PWM 3.3V) |
| **SDO/MISO** | **NIE PODŁĄCZAĆ**| — | **Pozostawić całkowicie niepodłączony** |

### Połączenie Dotyku XPT2046 (5 połączeń)
| XPT2046 PIN | ESP32-D PIN | Kolor przewodu (sugerowany) | Uwagi i funkcja |
| :--- | :--- | :--- | :--- |
| **T_CLK** | GPIO 18 | ZIELONY | **Wspólny** z wyświetlaczem (linia zegara SCK) |
| **T_CS** | GPIO 21 | BIAŁY | Chip Select – aktywacja funkcji dotyku |
| **T_DIN** | GPIO 23 | NIEBIESKI | **Wspólny** z wyświetlaczem (wejście danych MOSI) |
| **T_DO** | GPIO 19 | SZARY | Master In Slave Out – **Dedykowany tylko dla dotyku** |
| **T_IRQ** | GPIO 22 | FIOLETOWY | Sygnał przerwania sprzętowego panelu dotykowego |

> ⚠️ **WAŻNE WSKAZÓWKI MONTAŻOWE:**
> * Całkowita liczba fizycznych połączeń przewodowych z pinami ESP32 wynosi dokładnie **11** (Zasilanie 5V, GND + 9 dedykowanych linii GPIO).
> * Wszystkie połączenia magistrali SPI powinny być realizowane przy użyciu krótkich przewodów (zalecana długość poniżej 20 cm) w celu eliminacji zakłóceń elektromagnetycznych oraz błędów w transmisji danych.

---

## 3. ADRESY PAMIĘCI FLASH DLA FIRMWARE

Oprogramowanie układowe składa się z czterech integralnych plików binarnych. Muszą one zostać załadowane do pamięci mikrokontrolera pod ściśle określone adresy startowe:

| Nazwa pliku binarnego | Adres Flash (Hex) | Opis struktury zawartości |
| :--- | :--- | :--- |
| **bootloader.bin** | `0x1000` | Kod rozruchowy architektury ESP32 |
| **partitions.bin** | `0x8000` | Tablica podziału pamięci flash i alokacji partycji |
| **firmware.bin** | `0x10000` | Główny rdzeń aplikacji systemu |
| **littlefs.bin** | `0x210000` | System plików (czcionki, ikony, elementy panelu WWW) |

---

## 4. METODY WGRYWANIA FIRMWARE (FLESZOWANIE)

Przed rozpoczęciem procedury podłącz ESP32 do komputera stabilnym kablem USB. Sprawdź w systemowym *Menedżerze urządzeń* przypisany numer portu w sekcji „Porty (COM i LPT)” (np. **COM7**). W razie problemów zainstaluj oficjalne sterowniki dla układów konwertera USB-UART (`CH340` lub `CP2102`).

### Metoda A: WebSerial ESPTool (Zalecana — przez przeglądarkę Chrome/Edge)
1. Uruchom przeglądarkę internetową (Chrome, Edge lub Opera) i przejdź pod dedykowany adres: `https://jason2866.github.io/WebSerial_ESPTool/`
2. Kliknij przycisk **Connect**, zaznacz na liście systemowej swój port COM i zatwierdź połączenie.
3. Wprowadź ESP32 w stan programowania: przytrzymaj na płytce fizyczny przycisk **BOOT**, naciśnij i zwolnij przycisk **RST (EN)**, a następnie zwolnij przycisk **BOOT**.
4. W interfejsie strony dodaj cztery pliki przyciskiem **Add Files**, wpisując ręcznie przypisane im adresy startowe (dokładnie według danych z sekcji 3).
5. Kliknij przycisk **Program**. Po zakończeniu procesu (ok. 2–3 minuty) zresetuj urządzenie przyciskiem RST.

### Metoda B: ESP32 Flash Download Tools (Oficjalny program Espressif)
1. Pobierz i uruchom aplikację *ESP32 Flash Download Tool*. Wybierz tryb pracy: **ESP32-D0WD-V3 (Develop Mode)**.
2. Dodaj ścieżki do plików `bootloader.bin`, `partitions.bin`, `firmware.bin` oraz `littlefs.bin`.
3. Po prawej stronie każdego pliku wpisz odpowiadający mu adres hex (`0x1000`, `0x8000`, `0x10000`, `0x210000`).
4. Zaznacz wszystkie cztery pola wyboru (checkboxy) aktywujące wgrywanie określonych plików.
5. Skonfiguruj parametry połączenia: prędkość **Baud Rate ustaw na 921600** oraz wskaż właściwy **Port COM**.
6. Kliknij **START**. *Jeśli program nie nawiąże komunikacji automatycznie, przytrzymaj przycisk BOOT podczas uruchamiania procedury fleszowania.*

### Metoda C: Esptool przez Terminal (Linia komend)
Upewnij się, że posiadasz zainstalowane środowisko Python wraz z pakietem esptool. Uruchom konsolę w katalogu zawierającym pobrane pliki `.bin` i wykonaj poniższe polecenie (podmień `COM7` na własny port szeregowy):

```bash
esptool.py --chip esp32 --port COM7 --baud 921600 write_flash 0x1000 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin 0x210000 littlefs.bin