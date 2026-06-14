ESP32 HAM CLOCK - ILI9488 FLASH PACKAGE (NO NUMBERS)

Pliki BIN:
- 0x1000-bootloader.bin
- 0x8000-partitions.bin
- 0xe000-boot_app0.bin
- 0x10000-firmware.bin
- 0x290000-littlefs.bin

Komenda flashowania:
& "C:\Users\tomas\AppData\Local\Arduino15\packages\esp32\tools\esptool_py\5.1.0\esptool.exe" --chip esp32 --port COM7 --baud 460800 write-flash -z 0x1000 "C:\Users\tomas\Desktop\ESP32-HAM-CLOCK-main1\src\ESP32-HAM-CLOCK\ESP32-HAM-CLOCK1\0x1000-bootloader.bin" 0x8000 "C:\Users\tomas\Desktop\ESP32-HAM-CLOCK-main1\src\ESP32-HAM-CLOCK\ESP32-HAM-CLOCK1\0x8000-partitions.bin" 0xe000 "C:\Users\tomas\Desktop\ESP32-HAM-CLOCK-main1\src\ESP32-HAM-CLOCK\ESP32-HAM-CLOCK1\0xe000-boot_app0.bin" 0x10000 "C:\Users\tomas\Desktop\ESP32-HAM-CLOCK-main1\src\ESP32-HAM-CLOCK\ESP32-HAM-CLOCK1\0x10000-firmware.bin" 0x290000 "C:\Users\tomas\Desktop\ESP32-HAM-CLOCK-main1\src\ESP32-HAM-CLOCK\ESP32-HAM-CLOCK1\0x290000-littlefs.bin"

Uwagi:
- zmien COM7 na swoj port
- frontend WWW siedzi w littlefs na 0x290000
- zestaw jest przygotowany dla ESP32 + ILI9488
- jesli widzisz dodatkowy plik `4-0x3D0000-littlefs.bin`, potraktuj go jako stary artefakt i nie flashuj go do tego ukladu partycji
