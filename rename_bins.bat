@echo off
chcp 65001 >nul
echo ========================================
echo  ESP32 HAM-CLOCK - Zmiana nazw plikow .bin
echo ========================================
echo.

cd /d "%~dp0.pio\build\esp32dev"

if not exist "firmware.bin" (
    echo BLAD: Nie znaleziono firmware.bin
    echo Uruchom najpierw: pio run
    pause
    exit /b 1
)

echo Zmieniam nazwy plikow bin...
echo.

if exist "bootloader.bin" (
    ren bootloader.bin "bootloader_0x1000.bin"
    echo   [OK] bootloader.bin -> bootloader_0x1000.bin
)

if exist "partitions.bin" (
    ren partitions.bin "partitions_0x8000.bin"
    echo   [OK] partitions.bin -> partitions_0x8000.bin
)

if exist "firmware.bin" (
    ren firmware.bin "firmware_0x10000.bin"
    echo   [OK] firmware.bin -> firmware_0x10000.bin
)

if exist "littlefs.bin" (
    ren littlefs.bin "littlefs_0x390000.bin"
    echo   [OK] littlefs.bin -> littlefs_0x390000.bin
)

echo.
echo ========================================
echo  PLIKI Z ADRESAMI:
echo ========================================
dir *.bin
echo.
echo Gotowe!
pause