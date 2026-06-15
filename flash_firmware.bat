@echo off
chcp 65001 >nul
echo ========================================
echo  ESP32 HAM-CLOCK - Upload Firmware
echo  (Tylko program glowny)
echo ========================================
echo.

set PORT=COM3
set FOLDER=%~dp0.pio\build\esp32dev

cd /d "%FOLDER%"

if not exist "bootloader_0x1000.bin" (
    echo BLAD: Brak bootloader_0x1000.bin
    echo Najpierw uruchom rename_bins.bat
    pause
    exit /b 1
)

if not exist "firmware_0x10000.bin" (
    echo BLAD: Brak firmware_0x10000.bin
    echo Najpierw uruchom rename_bins.bat
    pause
    exit /b 1
)

echo.
echo Wgrywanie firmware...
echo.

echo Krok 1/2: Bootloader...
python -m esptool --port %PORT% write_flash 0x1000 bootloader_0x1000.bin

echo.
echo Krok 2/2: Firmware...
python -m esptool --port %PORT% write_flash 0x10000 firmware_0x10000.bin

echo.
if errorlevel 1 (
    echo BLAD podczas wgrywania!
) else (
    echo ========================================
    echo  GOTOWE!
    echo ========================================
    echo.
    echo Firmware zostal wgrany.
    echo Pamietaj o wgraniu LittleFS:
    echo   flash_littlefs.bat
)
echo.
pause