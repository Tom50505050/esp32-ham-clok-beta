@echo off
chcp 65001 >nul
echo ========================================
echo  ESP32 HAM-CLOCK - Flashowanie
echo ========================================
echo.

set PORT=COM3
set FOLDER=%~dp0.pio\build\esp32dev

cd /d "%FOLDER%"

echo Sprawdzam pliki...
echo.

if not exist "bootloader_0x1000.bin" (
    echo BLAD: Brak bootloader_0x1000.bin
    echo Uruchom rename_bins.bat przed flashowaniem!
    pause
    exit /b 1
)

if not exist "firmware_0x10000.bin" (
    echo BLAD: Brak firmware_0x10000.bin
    echo Uruchom rename_bins.bat przed flashowaniem!
    pause
    exit /b 1
)

echo.
echo ========================================
echo  FLASHOWANIE ESP32
echo  Port: %PORT%
echo ========================================
echo.

echo Krok 1/4: Czyszczenie pamieci Flash...
python -m esptool --port %PORT% erase_flash
if errorlevel 1 (
    echo BLAD: Nie mozna wykryc ESP32!
    echo Sprawdz polaczenie USB i port COM
    pause
    exit /b 1
)

echo.
echo Krok 2/4: Wgrywanie bootloader...
python -m esptool --port %PORT% write_flash 0x1000 bootloader_0x1000.bin

echo.
echo Krok 3/4: Wgrywanie firmware...
python -m esptool --port %PORT% write_flash 0x10000 firmware_0x10000.bin

echo.
echo Krok 4/4: Wgrywanie systemu plikow...
if exist "littlefs_0x390000.bin" (
    python -m esptool --port %PORT% write_flash 0x390000 littlefs_0x390000.bin
) else (
    echo [UWAGA] littlefs_0x390000.bin nie znaleziony - pomijam
)

echo.
echo ========================================
echo  GOTOWE!
echo ========================================
echo.
echo ESP32 powinien sie automatycznie uruchomic.
echo.
echo Jesli nie, sprawdz:
echo 1. Czy diode LED miga przy starcie
echo 2. Czy w Serial Monitor widac logi
echo.
pause