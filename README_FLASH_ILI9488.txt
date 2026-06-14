ESP32 HAM CLOCK - ILI9488 FLASH PACKAGE

Gotowe pliki BIN:
- 1-0x1000-bootloader.bin
- 2-0x8000-partitions.bin
- 3-0xe000-boot_app0.bin
- 4-0x10000-firmware.bin
- 5-0x290000-littlefs.bin

Docelowy sprzet:
- ESP32
- TFT ILI9488
- dotyk XPT2046

Piny TFT:
- TFT_MISO = GPIO19
- TFT_MOSI = GPIO23
- TFT_SCLK = GPIO18
- TFT_CS   = GPIO15
- TFT_DC   = GPIO2
- TFT_RST  = -1
- TFT_BL   = GPIO32

Piny dotyku:
- TOUCH_CS   = GPIO21
- TOUCH_IRQ  = GPIO35
- TOUCH_MOSI = GPIO23
- TOUCH_MISO = GPIO19
- TOUCH_CLK  = GPIO18

Komenda flashowania:
& "C:\Users\tomas\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.1.0\esptool.exe" --chip esp32 --port COM7 --baud 460800 write-flash -z 0x1000 "C:\Users\tomas\Desktop\ESP32-HAM-CLOCK-main1\src\ESP32-HAM-CLOCK\ESP32-HAM-CLOCK1\1-0x1000-bootloader.bin" 0x8000 "C:\Users\tomas\Desktop\ESP32-HAM-CLOCK-main1\src\ESP32-HAM-CLOCK\ESP32-HAM-CLOCK1\2-0x8000-partitions.bin" 0xe000 "C:\Users\tomas\Desktop\ESP32-HAM-CLOCK-main1\src\ESP32-HAM-CLOCK\ESP32-HAM-CLOCK1\3-0xe000-boot_app0.bin" 0x10000 "C:\Users\tomas\Desktop\ESP32-HAM-CLOCK-main1\src\ESP32-HAM-CLOCK\ESP32-HAM-CLOCK1\4-0x10000-firmware.bin" 0x290000 "C:\Users\tomas\Desktop\ESP32-HAM-CLOCK-main1\src\ESP32-HAM-CLOCK\ESP32-HAM-CLOCK1\5-0x290000-littlefs.bin"

Uwagi:
- zmien COM7 na swoj port
- po flashowaniu zrob restart ESP32
- jesli strona WWW sie nie odswiezy, zrob twarde odswiezenie w przegladarce
- uzywaj obrazu LittleFS z offsetem 0x290000; stary plik `4-0x3D0000-littlefs.bin` nie jest uzywany w tym ukladzie partycji
