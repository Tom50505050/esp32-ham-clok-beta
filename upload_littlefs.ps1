# Skrypt wgrywania LittleFS dla ESP32-HAM-CLOCK
# Użycie: .\upload_littlefs.ps1 [COM_PORT]
# Domyślnie: COM7

param(
    [string]$Port = "COM7"
)

Write-Host "=== Wgrywanie LittleFS dla ESP32-HAM-CLOCK ===" -ForegroundColor Green
Write-Host "Port: $Port"
Write-Host ""

# Sprawdź czy platformio jest zainstalowane
$pio = Get-Command pio -ErrorAction SilentlyContinue
if (-not $pio) {
    Write-Host "ERROR: PlatformIO CLI nie jest zainstalowane!" -ForegroundColor Red
    Write-Host "Zainstaluj: pip install platformio" -ForegroundColor Yellow
    exit 1
}

# Sprawdź czy folder littlefs_data istnieje
if (-not (Test-Path "littlefs_data/index.html")) {
    Write-Host "ERROR: Plik littlefs_data/index.html nie istnieje!" -ForegroundColor Red
    exit 1
}

Write-Host "1. Budowanie obrazu LittleFS..." -ForegroundColor Cyan
pio run --target buildfs --environment esp32dev

if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Budowanie LittleFS nie powiodło się!" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "2. Wgrywanie obrazu LittleFS na ESP32 ($Port)..." -ForegroundColor Cyan
Write-Host "   Upewnij się, że ESP32 jest podłączone i w trybie programowania" -ForegroundColor Yellow
Write-Host ""

pio run --target uploadfs --environment esp32dev --upload-port $Port

if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Wgrywanie LittleFS nie powiodło się!" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "=== SUKCES! LittleFS wgrane ===" -ForegroundColor Green
Write-Host "Zresetuj ESP32 i odśwież stronę: http://192.168.1.3/" -ForegroundColor Cyan
