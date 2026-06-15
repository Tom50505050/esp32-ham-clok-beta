@echo off
chcp 65001 >nul
echo ========================================
echo  ESP32 HAM-CLOCK - Upload LittleFS
echo  (Tylko pliki web/HTML)
echo ========================================
echo.

set PORT=COM3
set FOLDER=%~dp0.pio\build\esp32dev

cd /d "%FOLDER%"

if not exist "littlefs_0x390000.bin" (
    echo BLAD: Brak littlefs_0x390000.bin
    echo Najpierw uruchom:
    echo   1. pio run --target buildfs
    echo   2. rename_bins.bat
    pause
    exit /b 1
)

echo.
echo Wgrywanie systemu plikow LittleFS...
echo.

python -m esptool --port %PORT% write_flash 0x390000 littlefs_0x390000.bin

echo.
if errorlevel 1 (
    echo BLAD podczas wgrywania!
) else (
    echo ========================================
    echo  GOTOWE!
    echo ========================================
    echo.
    echo Pliki web zostaly wgrane.
    echo.
    echo Jesli strona nie dziala, wykonaj:
    echo   pio run --target erase
    echo   pio run --target upload
    echo   pio run --target uploadfs
)
echo.
pause