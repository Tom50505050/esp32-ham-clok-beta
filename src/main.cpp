#include <Arduino.h>

/*
 * ESP32-HAM-CLOCK - Odbiornik DX Cluster i stacji APRS-IS z interfejsem WWW i opcjonalnym wyświetlaczem TFT
 * 
 * - WiFi Manager (AP mode jeśli brak zapisanych danych)
 * - Połączenie z Telnet DX Cluster (TYLKO ODBIERANIE - nie wysyła spotów)
 * - Parsowanie i przechowywanie spotów DX
 * - Obliczanie odległości (Haversine)
 * - Interfejs WWW z polling (odświeżanie co 2 sekundy)
 * - Wyświetlacz TFT (ESP32-2432S028) - opcjonalnie ESP32 WROOM + TFT ILI9341 
 * 
 * UWAGA: Urządzenie działa TYLKO w trybie odbioru.
 * Tak samo z APRS.fi  - tylko odczyt.
*/

// Wczesne forward-deklaracje, aby auto-prototypowanie Arduino znało typy używane w sygnaturach
enum ScreenType : uint8_t;    // pełna definicja niżej
enum Screen6ViewMode : uint8_t;
enum TrKey : uint8_t;
struct DXSpot;                // pełna definicja niżej
struct APRSStation;           // pełna definicja niżej
struct AprsWxDecoded;         // pełna definicja niżej
struct PropagationData;       // pełna definicja niżej
struct WeatherData;           // pełna definicja niżej
namespace fs { class File; }

static const uint8_t WEATHER_DETAIL_COLS = 5;

// Forward declarations for UnlisHunter state used before global definitions.
extern bool unlisRunning;
extern bool unlisGameOver;

void normalizePolish(String &text);

// ========== KONFIGURACJA WYŚWIETLACZA TFT ==========
// Włącz TFT w platformio.ini (build_flags) lub Arduino IDE; guard to avoid redefinition warnings
#ifndef ENABLE_TFT_DISPLAY
#define ENABLE_TFT_DISPLAY
#endif

// Wersja firmware
#define FIRMWARE_VERSION "1.4"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <esp_rom_sys.h>
#include <soc/rtc_cntl_reg.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <FS.h>
#include <LittleFS.h>
#include <math.h>
#include <ctype.h>
#include <WiFiClient.h>
#include <SGP4.h>
#include <TimeLib.h>  

static const char* tr(TrKey key);
static void buildWeatherDetailHeaders(String headers[WEATHER_DETAIL_COLS]);
static uint16_t readBmp16(fs::File &f);
static uint32_t readBmp32(fs::File &f);
String getAPRSSymbolShort(const APRSStation &station);
void drawBootScreen();
void tftBootPrintLine(const String &line);
String formatDistanceOrCountry(const DXSpot &spot, size_t maxLen);
void drawHamClock();
void drawDxCluster();
void drawSunSpots();
void drawBandInfo();
void drawWeather();
void drawWeatherForecast();
void drawAprsIs();
void drawAprsRadar();
void drawPotaCluster();
void drawHamalertCluster();
void drawMatrixClock();
void drawQrzInfoScreen();
void handleQrzInfoTouch(uint16_t x, uint16_t y);
void drawIssPassTracking();

void drawHamburgerMenuButton3D(int x, int y);
void handleTouchCalibrationTouch(int16_t rawX, int16_t rawY, uint16_t x, uint16_t y, bool isNewTap);
static bool isAprsAlertCloseButtonHit(uint16_t x, uint16_t y);
static void dismissAprsAlertScreen();
void drawBrightnessMenu();
void handleBrightnessMenuTouch(uint16_t x, uint16_t y, bool isNewTap);
void unlisStopGame();
void unlisStartResetGame();
void unlisHandlePttPress(unsigned long nowMs);
void handleScreen1MenuTouch(uint16_t x, uint16_t y);
void handleScreen2MenuTouch(uint16_t x, uint16_t y);
void handleScreen6MenuTouch(uint16_t x, uint16_t y);
void handleScreen5MenuTouch(uint16_t x, uint16_t y);
void handleScreen7MenuTouch(uint16_t x, uint16_t y);
void handleScreen8MenuTouch(uint16_t x, uint16_t y);
void handlePskMapTouch(uint16_t x, uint16_t y);
void drawDxClusterFilterMenu();
void drawPotaFilterMenu();
void drawHamalertFilterMenu();
void drawAprsSortMenu();
void drawWeatherMenu();
void drawHamClockTimeMenu();
static bool isScreen6RadarZoomTopHit(uint16_t x, uint16_t y);
static void triggerScreen6RadarHint();
static bool isScreen6RadarZoomBottomHit(uint16_t x, uint16_t y);
void drawHamClockTimeMenu();
float calculateBearing(double lat1, double lon1, double lat2, double lon2);

// ========== DANE PSK REPORTER ==========
struct PskSpot {
  String callsign;
  float lat;
  float lon;
  int band;       // w metrach (40, 20, 80, etc.)
  String mode;
  unsigned long timestamp;
  int snr;        // opcjonalnie
};

const int PSK_MAX_SPOTS = 50;
PskSpot pskSpots[PSK_MAX_SPOTS];
int pskSpotCount = 0;
unsigned long lastPskFetchMs = 0;
const unsigned long PSK_FETCH_INTERVAL_MS = 60000; // co minutę

// Konfiguracja PSKReporter (zapisana w NVS)
String pskReceiverCallsign = "";  // znak odbiornika do filtrowania
int pskMaxSpots = 50;             // maksymalna liczba spotów
int pskHoursWindow = 1;           // okno czasowe w godzinach
String pskFilterBand = "";        // filtr pasmo (pusty = wszystkie)
String pskFilterMode = "";        // filtr tryb (pusty = wszystkie)
String pskCustomUrl = "";         // własny URL API (pusty = domyślny)

// HTTP Monitoring - monitorowanie znaku
String pskMonitorCallsign = "";
int pskReportDays = 0;
int pskAutoRefreshMinutes = 5;

// MQTT PSK Reporter - alternatywny tryb
bool pskMqttEnabled = false;              // Czy używać MQTT zamiast HTTP
String pskMqttServer = "mqtt.pskreporter.info";  // Serwer MQTT
int pskMqttPort = 1883;                   // Port MQTT (1883 dla plain, 8883 dla TLS)
String pskMqttCallsign = "";             // Znak do monitorowania (jeśli pusty - monitoruje receiver)

WiFiClient pskMqttWifiClient;
PubSubClient pskMqttClient(pskMqttWifiClient);

// Deklaracje funkcji MQTT
void setupPskMqtt();
void loopPskMqtt();
void pskMqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectPskMqtt();
void processPskMqttBuffer();

// Menu PSK Map (tymczasowe ustawienia podczas edycji na ekranie)
bool pskMapMenuOpen = false;
String pskTempReceiver = "";
String pskTempBand = "";
String pskTempMode = "";
int pskTempMaxSpots = 50;
bool pskTempMqttEnabled = false;
String pskTempMqttServer = "";
String pskTempMqttCallsign = "";
String pskKeyboardBuffer = "";  // Bufor dla klawiatury ekranowej
int pskKeyboardTarget = 0;        // 1=znak, 2=max, 3=mqtt_server, 4=mqtt_call
enum PskMenuField { PSK_FIELD_NONE, PSK_FIELD_CALL, PSK_FIELD_BAND, PSK_FIELD_MODE, PSK_FIELD_MAXSPOTS,
                    PSK_FIELD_MQTT_MODE, PSK_FIELD_MQTT_SERVER, PSK_FIELD_MQTT_CALL,
                    PSK_FIELD_KEYBOARD, PSK_FIELD_BAND_SELECT, PSK_FIELD_MODE_SELECT };
PskMenuField pskActiveField = PSK_FIELD_NONE;
bool pskKeyboardActive = false;  // Czy klawiatura PSK jest widoczna (blokuje nawigację)

// Ścieżka do pliku BMP z mapą
const char* PSK_MAP_BMP_PATH = "/Mapa swiata.bmp";

// Zakres współrzędnych dla mapy (Plate Carrée - pełny zakres)
const float MAP_LAT_MIN = -90.0f;  // min szerokość geograficzna (Antarktyda)
const float MAP_LAT_MAX = 90.0f;   // max szerokość geograficzna (Arktyka)
const float MAP_LON_MIN = -180.0f; // min długość geograficzna
const float MAP_LON_MAX = 180.0f;  // max długość geograficzna
const int MAP_DISPLAY_X = 0;       // pozycja X mapy na ekranie
const int MAP_DISPLAY_Y = 0;       // pozycja Y (0 = pełny ekran)
const int MAP_DISPLAY_W = 480;     // szerokość mapy
const int MAP_DISPLAY_H = 320;     // wysokość mapy (480x320)

// Forward declarations for Screen Saver functions
void resetScreenSaverActivity();
void checkScreenSaverTimeout();
void drawScreenSaverMenu();
void handleScreenSaverMenuTouch(uint16_t x, uint16_t y);

// Forward declarations for Screen Sleep functions
void drawScreenSleepMenu();
void handleScreenSleepMenuTouch(uint16_t x, uint16_t y);
void enterScreenSleep();
void wakeUpFromSleep();

void savePreferences();
void locatorToLatLon(String locator, double &lat, double &lon);
bool fetchAirPollutionData(double lat, double lon);
static bool drawBmpFromFS(const String &filename, int16_t x, int16_t y);
void sendAPRSLogin();
static String getAprsTxCallsignWithSsid();
void sendAPRSFilter();


// ========== TYPY EKRANÓW (używane w prototypach) ==========
enum ScreenType : uint8_t {
  SCREEN_OFF = 0,
  SCREEN_HAM_CLOCK = 1,
  SCREEN_DX_CLUSTER = 2,
  SCREEN_SUN_SPOTS = 3,
  SCREEN_BAND_INFO = 4,
  SCREEN_WEATHER_DSP = 5,
  SCREEN_APRS_IS = 6,
  SCREEN_POTA_CLUSTER = 7,
  SCREEN_HAMALERT_CLUSTER = 8,
  SCREEN_APRS_RADAR = 9,
  SCREEN_MATRIX_CLOCK = 10,
  SCREEN_UNLIS_HUNTER = 11,
  SCREEN_WEATHER_FORECAST = 12,
  SCREEN_PSK_MAP = 13,
  SCREEN_ISS_PASS_TRACKING = 14
};

#define LOG_VERBOSE false
#define LOGV_PRINT(x) do { if (LOG_VERBOSE) Serial.print(x); } while (0)
#define LOGV_PRINTLN(x) do { if (LOG_VERBOSE) Serial.println(x); } while (0)
#define LOGV_PRINTF(...) do {} while (0)

// ========== WYŚWIETLACZ TFT (ESP32 DevKit + zewnętrzny SPI TFT) ==========
#ifdef ENABLE_TFT_DISPLAY
// Wspólna konfiguracja dla szkicu i kompilowanej osobno biblioteki TFT_eSPI.
// Dzięki temu biblioteka nie wraca do własnego User_Setup.h z innym driverem/pinami.
#include "tft_setup.h"

#define TFT_UI_WIDTH 480
#define TFT_UI_HEIGHT 320

#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

class HamClockTFT : public TFT_eSPI {
public:
  HamClockTFT() : TFT_eSPI() {}

  using TFT_eSPI::drawCentreString;
  using TFT_eSPI::drawString;
  using TFT_eSPI::fontHeight;
  using TFT_eSPI::pushImage;
  using TFT_eSPI::setCursor;
  using TFT_eSPI::textWidth;

  bool usesScaledUi() {
    return false;
  }

  int16_t width(void) {
    return usesScaledUi() ? logicalWidth() : TFT_eSPI::width();
  }

  int16_t height(void) {
    return usesScaledUi() ? logicalHeight() : TFT_eSPI::height();
  }

  void setCursor(int16_t x, int16_t y) {
    TFT_eSPI::setCursor(scaleX(x), scaleY(y));
  }

  void setTextSize(uint8_t size) {
    TFT_eSPI::setTextSize(scaleTextSize(size));
  }

  int16_t drawString(const String &string, int32_t x, int32_t y) {
    return unscaleX(TFT_eSPI::drawString(string, scaleX(x), scaleY(y)));
  }

  int16_t drawString(const char *string, int32_t x, int32_t y) {
    return drawString(String(string), x, y);
  }

  int16_t drawString(const String &string, int32_t x, int32_t y, uint8_t font) {
    return unscaleX(TFT_eSPI::drawString(string, scaleX(x), scaleY(y), font));
  }

  int16_t drawString(const char *string, int32_t x, int32_t y, uint8_t font) {
    return drawString(String(string), x, y, font);
  }

  int16_t drawCentreString(const String &string, int32_t x, int32_t y, uint8_t font) {
    return unscaleX(TFT_eSPI::drawCentreString(string, scaleX(x), scaleY(y), font));
  }

  int16_t drawCentreString(const char *string, int32_t x, int32_t y, uint8_t font) {
    return drawCentreString(String(string), x, y, font);
  }

  int16_t textWidth(const String &string) {
    return unscaleX(TFT_eSPI::textWidth(string));
  }

  int16_t textWidth(const char *string) {
    return textWidth(String(string));
  }

  int16_t fontHeight() {
    return unscaleY(TFT_eSPI::fontHeight());
  }

  void fillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color) {
    TFT_eSPI::fillRect(scaleX(x), scaleY(y), scaleLenX(w), scaleLenY(h), color);
  }

  void drawRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color) {
    TFT_eSPI::drawRect(scaleX(x), scaleY(y), scaleLenX(w), scaleLenY(h), color);
  }

  void drawRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint32_t color) {
    TFT_eSPI::drawRoundRect(scaleX(x), scaleY(y), scaleLenX(w), scaleLenY(h), scaleRadius(r), color);
  }

  void fillRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint32_t color) {
    TFT_eSPI::fillRoundRect(scaleX(x), scaleY(y), scaleLenX(w), scaleLenY(h), scaleRadius(r), color);
  }

  void drawFastHLine(int32_t x, int32_t y, int32_t w, uint32_t color) {
    TFT_eSPI::drawFastHLine(scaleX(x), scaleY(y), scaleLenX(w), color);
  }

  void drawFastVLine(int32_t x, int32_t y, int32_t h, uint32_t color) {
    TFT_eSPI::drawFastVLine(scaleX(x), scaleY(y), scaleLenY(h), color);
  }

  void drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint32_t color) {
    TFT_eSPI::drawLine(scaleX(x0), scaleY(y0), scaleX(x1), scaleY(y1), color);
  }

  void fillTriangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint32_t color) {
    TFT_eSPI::fillTriangle(scaleX(x0), scaleY(y0), scaleX(x1), scaleY(y1), scaleX(x2), scaleY(y2), color);
  }

  void drawCircle(int32_t x, int32_t y, int32_t r, uint32_t color) {
    TFT_eSPI::drawCircle(scaleX(x), scaleY(y), scaleRadius(r), color);
  }

  void fillCircle(int32_t x, int32_t y, int32_t r, uint32_t color) {
    TFT_eSPI::fillCircle(scaleX(x), scaleY(y), scaleRadius(r), color);
  }

  void pushImage(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t *data) {
    pushImageScaled(x, y, w, h, data);
  }

  void pushImage(int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *data) {
    pushImageScaled(x, y, w, h, data);
  }

private:
  int16_t logicalWidth() {
    return (TFT_eSPI::width() >= TFT_eSPI::height()) ? TFT_UI_WIDTH : TFT_UI_HEIGHT;
  }

  int16_t logicalHeight() {
    return (TFT_eSPI::width() >= TFT_eSPI::height()) ? TFT_UI_HEIGHT : TFT_UI_WIDTH;
  }

  int32_t scaleX(int32_t value) {
    return usesScaledUi() ? scaleValue(value, TFT_eSPI::width(), logicalWidth()) : value;
  }

  int32_t scaleY(int32_t value) {
    return usesScaledUi() ? scaleValue(value, TFT_eSPI::height(), logicalHeight()) : value;
  }

  int32_t scaleLenX(int32_t value) {
    return usesScaledUi() ? scaleSize(value, TFT_eSPI::width(), logicalWidth()) : value;
  }

  int32_t scaleLenY(int32_t value) {
    return usesScaledUi() ? scaleSize(value, TFT_eSPI::height(), logicalHeight()) : value;
  }

  int32_t scaleRadius(int32_t value) {
    if (!usesScaledUi()) {
      return value;
    }
    return min(scaleLenX(value), scaleLenY(value));
  }

  int16_t unscaleX(int16_t value) {
    return usesScaledUi() ? (int16_t)unscaleValue(value, logicalWidth(), TFT_eSPI::width()) : value;
  }

  int16_t unscaleY(int16_t value) {
    return usesScaledUi() ? (int16_t)unscaleValue(value, logicalHeight(), TFT_eSPI::height()) : value;
  }

  uint8_t scaleTextSize(uint8_t value) {
    if (!usesScaledUi() || value <= 1) {
      return value;
    }
    float scaleXf = (float)TFT_eSPI::width() / (float)logicalWidth();
    float scaleYf = (float)TFT_eSPI::height() / (float)logicalHeight();
    uint8_t scaled = (uint8_t)lroundf((float)value * min(scaleXf, scaleYf));
    return (scaled < 1) ? 1 : scaled;
  }

  static int32_t scaleValue(int32_t value, int32_t physical, int32_t logical) {
    return (int32_t)(((int64_t)value * (int64_t)physical + (logical / 2)) / (int64_t)logical);
  }

  static int32_t scaleSize(int32_t value, int32_t physical, int32_t logical) {
    if (value <= 0) {
      return 0;
    }
    int32_t scaled = scaleValue(value, physical, logical);
    return (scaled < 1) ? 1 : scaled;
  }

  static int32_t unscaleValue(int32_t value, int32_t logical, int32_t physical) {
    return (int32_t)(((int64_t)value * (int64_t)logical + (physical / 2)) / (int64_t)physical);
  }

  template <typename PixelPtr>
  void pushImageScaled(int32_t x, int32_t y, int32_t w, int32_t h, PixelPtr data) {
    const uint16_t *pixelData = data;
    if (!usesScaledUi()) {
      TFT_eSPI::pushImage(x, y, w, h, const_cast<uint16_t *>(pixelData));
      return;
    }

    int32_t scaledX = scaleX(x);
    int32_t scaledY = scaleY(y);

    // BMP-y ikon pogodowych są przesyłane po jednym wierszu, więc można je
    // bezpiecznie przeskalować w poziomie bez ruszania reszty kodu.
    if (h == 1 && w > 0) {
      int32_t scaledW = scaleLenX(w);
      uint16_t *scaledLine = (uint16_t *)malloc((size_t)scaledW * sizeof(uint16_t));
      if (scaledLine != nullptr) {
        for (int32_t i = 0; i < scaledW; i++) {
          int32_t srcX = (int32_t)(((int64_t)i * (int64_t)w) / (int64_t)scaledW);
          if (srcX >= w) srcX = w - 1;
          scaledLine[i] = pixelData[srcX];
        }
        TFT_eSPI::pushImage(scaledX, scaledY, scaledW, 1, scaledLine);
        free(scaledLine);
        return;
      }
    }

    TFT_eSPI::pushImage(scaledX, scaledY, w, h, const_cast<uint16_t *>(pixelData));
  }
};

HamClockTFT tft;

#define TFT_BL_PIN TFT_BL

#define BACKLIGHT_PWM_CHANNEL 0
#define BACKLIGHT_PWM_FREQ 5000
#define BACKLIGHT_PWM_RES_BITS 8
#define MIN_BACKLIGHT_PERCENT 10
#define TFT_BACKLIGHT 100
int backlightPercent = TFT_BACKLIGHT;

// ========== DOTYK (XPT2046) ==========
#ifndef TOUCH_CS
#define TOUCH_CS 21
#endif
#ifndef TOUCH_IRQ
#define TOUCH_IRQ 22
#endif
#ifndef TOUCH_MOSI
#define TOUCH_MOSI TFT_MOSI
#endif
#ifndef TOUCH_MISO
#define TOUCH_MISO TFT_MISO
#endif
#ifndef TOUCH_CLK
#define TOUCH_CLK TFT_SCLK
#endif

// Kalibracja dotyku (dopasuj jeśli pozycje są przesunięte)
#define TOUCH_X_MIN 200
#define TOUCH_X_MAX 3800
#define TOUCH_Y_MIN 200
#define TOUCH_Y_MAX 3800
#define TOUCH_SWAP_XY false
#define TOUCH_INVERT_X false
#define TOUCH_INVERT_Y false

int touchXMin = TOUCH_X_MIN;
int touchXMax = TOUCH_X_MAX;
int touchYMin = TOUCH_Y_MIN;
int touchYMax = TOUCH_Y_MAX;
bool touchSwapXY = TOUCH_SWAP_XY;
bool touchInvertX = TOUCH_INVERT_X;
bool touchInvertY = TOUCH_INVERT_Y;
uint8_t touchRotation = 1;
uint8_t tftRotation = 1;
#ifdef TFT_INVERSION_ON
bool tftInvertColors = true;
#else
bool tftInvertColors = false;
#endif

// Kolor wyświetlanego Callsignu na ekranie zegara
uint16_t callsignColor = TFT_WHITE;

static bool sanitizeTftInvertSetting(bool requested) {
  return requested;
}

bool tftInitialized = false;
SPIClass touchSPI(VSPI);
XPT2046_Touchscreen touch(TOUCH_CS, TOUCH_IRQ);

int bootLogY = 2;
const int BOOT_LOG_LINE_HEIGHT = 10;
bool bootSequenceActive = true;
bool littleFsReady = false;
bool splashPathLoaded = false;

// ========== SYSTEM MENU I NAWIGACJI ==========
const int SCREEN_PAGE_COUNT = 14;
const ScreenType DEFAULT_SCREEN_ORDER[SCREEN_PAGE_COUNT] = {
  SCREEN_HAM_CLOCK,
  SCREEN_DX_CLUSTER,
  SCREEN_APRS_IS,
  SCREEN_APRS_RADAR,
  SCREEN_BAND_INFO,
  SCREEN_SUN_SPOTS,
  SCREEN_WEATHER_DSP,
  SCREEN_WEATHER_FORECAST,
  SCREEN_POTA_CLUSTER,
  SCREEN_HAMALERT_CLUSTER,
  SCREEN_PSK_MAP,
  SCREEN_UNLIS_HUNTER,
  SCREEN_MATRIX_CLOCK,
  SCREEN_ISS_PASS_TRACKING
};

ScreenType screenOrder[SCREEN_PAGE_COUNT] = {
  SCREEN_HAM_CLOCK,
  SCREEN_DX_CLUSTER,
  SCREEN_APRS_IS,
  SCREEN_APRS_RADAR,
  SCREEN_BAND_INFO,
  SCREEN_SUN_SPOTS,
  SCREEN_WEATHER_DSP,
  SCREEN_WEATHER_FORECAST,
  SCREEN_POTA_CLUSTER,
  SCREEN_HAMALERT_CLUSTER,
  SCREEN_PSK_MAP,
  SCREEN_UNLIS_HUNTER,
  SCREEN_MATRIX_CLOCK,
  SCREEN_ISS_PASS_TRACKING
};
ScreenType currentScreen = SCREEN_OFF;  // Ustawiany po wczytaniu preferencji
bool inMenu = false;    // Czy jesteśmy w menu wewnętrznym strony
int menuOption = 0;     // Aktualna opcja w menu (jeśli inMenu == true)
const int DEFAULT_TFT_SWITCH_TIME_SEC = 30;
bool tftAutoSwitchEnabled = false;
int tftAutoSwitchTimeSec = DEFAULT_TFT_SWITCH_TIME_SEC;
unsigned long tftAutoSwitchLastMs = 0;
ScreenType tftAutoSwitchLastScreen = SCREEN_OFF;
unsigned long lastScreenUpdate = 0;
const unsigned long SCREEN_UPDATE_INTERVAL = 100; // Aktualizuj ekran co 100ms
const unsigned long DX_SCREEN_MIN_REDRAW_MS = 5000; // Ogranicz zapis tabeli DX do max 1 raz/5s

// ========== WYGASZACZ EKRANU (Matrix) ==========
const int DEFAULT_SCREEN_SAVER_TIMEOUT_MIN = 5;  // Domyślny czas w minutach
bool screenSaverEnabled = false;                  // Czy wygaszacz włączony
int screenSaverTimeoutMin = DEFAULT_SCREEN_SAVER_TIMEOUT_MIN;  // Czas w minutach
unsigned long screenSaverLastActivityMs = 0;      // Ostatnia aktywność
bool screenSaverActive = false;                   // Czy wygaszacz aktualnie działa
ScreenType screenSaverPrevScreen = SCREEN_OFF;    // Poprzedni ekran przed wygaszaczem
bool screenSaverMenuActive = false;               // Czy jesteśmy w menu wygaszacza
unsigned long lastScreen1UpdateMs = 0;

// Typy wygaszacza ekranu
enum ScreenSaverType {
  SAVER_ANALOG_CLOCK = 0,   // Analogowy zegar
  SAVER_DIGITAL_CLOCK = 1,  // Cyfrowy zegar (stary bouncing)
  SAVER_MATRIX = 2          // Efekt Matrix
};
const char* SAVER_TYPE_NAMES[] = {"ANALOG", "DIGITAL", "MATRIX"};
ScreenSaverType screenSaverType = SAVER_ANALOG_CLOCK;  // Domyślnie analogowy

// ========== UŚPIENIE EKRANU ==========
const int DEFAULT_SCREEN_SLEEP_TIMEOUT_MIN = 5;  // Domyślny czas w minutach
bool screenSleepEnabled = false;                  // Czy uśpienie włączone
int screenSleepTimeoutMin = DEFAULT_SCREEN_SLEEP_TIMEOUT_MIN;  // Czas w minutach
unsigned long screenSleepLastActivityMs = 0;      // Ostatnia aktywność
bool screenSleepActive = false;                   // Czy uśpienie aktualnie działa
bool screenSleepMenuActive = false;               // Czy jesteśmy w menu uśpienia
ScreenType screenSleepPrevScreen = SCREEN_OFF;    // Poprzedni ekran przed uśpieniem
int screenSleepMenuTimeoutMin = DEFAULT_SCREEN_SLEEP_TIMEOUT_MIN; // Wartość w menu

// ========== QRZ POPUP ==========
bool qrzPopupActive = false;
String qrzPopupCallsign = "";
String qrzPopupGrid = "";
String qrzPopupCountry = "";
String qrzPopupName = "";
String qrzPopupQth = "";
String qrzPopupEmail = "";
float qrzPopupDistance = 0.0;
double qrzPopupLat = 0.0;
double qrzPopupLon = 0.0;
bool qrzPopupHasLatLon = false;
unsigned long qrzPopupFetchedAt = 0;
unsigned long lastScreen3UpdateMs = 0;
unsigned long lastScreen4UpdateMs = 0;
unsigned long lastScreen5UpdateMs = 0;
unsigned long lastScreen7UpdateMs = 0;
unsigned long lastScreen8UpdateMs = 0;
bool screen1HeaderNeedsRedraw = true;

const int TFT_TABLE_TOP = 32;
const int TFT_TABLE_BOTTOM = 320;
const int TFT_TABLE_WIDTH = 460;
const int TFT_TABLE_HEIGHT = TFT_TABLE_BOTTOM - TFT_TABLE_TOP;
// UnlisHunter constants must be visible before touch handling code uses them.
const int UNLIS_CENTER_X = 240;
const int UNLIS_CENTER_Y = 120;
const int UNLIS_OUTER_R = 115; // 2m
const int UNLIS_INNER_R = 60;  // 70cm
const int UNLIS_BTN_SIZE = 100;
const int UNLIS_DRAW_BTN_SIZE = UNLIS_BTN_SIZE / 2;
const int UNLIS_START_X = 0;
const int UNLIS_START_Y = 0;
const int UNLIS_PTT_X = 480 - UNLIS_BTN_SIZE;
const int UNLIS_PTT_Y = 320 - UNLIS_BTN_SIZE;
const int UNLIS_EXIT_X = 0;
const int UNLIS_EXIT_Y = 320 - UNLIS_BTN_SIZE;
const unsigned long UNLIS_FRAME_MS = 40UL;
const float UNLIS_BASE_SCAN_DEG_PER_SEC = 46.0f;
const float UNLIS_ACCEL_PER_CATCH_EARLY = 0.05f;
const float UNLIS_ACCEL_PER_CATCH_LATE = 0.03f;
const float UNLIS_TARGET_LIFE_ROTATIONS = 2.5f;
const unsigned long UNLIS_TARGET_RESPAWN_MIN_MS = 450UL;
const unsigned long UNLIS_TARGET_RESPAWN_MAX_MS = 1200UL;
const unsigned long UNLIS_SECOND_TARGET_DELAY_MS = 60000UL;
const unsigned long UNLIS_SECOND_TARGET_RESPAWN_MIN_MS = 3500UL;
const unsigned long UNLIS_SECOND_TARGET_RESPAWN_MAX_MS = 9500UL;
const unsigned long UNLIS_GREEN_STATION_LIFE_MS = 5000UL;
const unsigned long UNLIS_GREEN_STATION_RESPAWN_MIN_MS = 9000UL;
const unsigned long UNLIS_GREEN_STATION_RESPAWN_MAX_MS = 18000UL;
const unsigned long TABLE_NAV_FOOTER_VISIBLE_MS = 5000UL;
unsigned long tableNavFooterVisibleUntilMs = 0;
#ifndef TFT_TABLE_SPRITE_COLOR_DEPTH
#define TFT_TABLE_SPRITE_COLOR_DEPTH 16
#endif
const uint16_t TFT_TABLE_ALT_ROW_COLOR = 0x0841;
TFT_eSprite sharedTableSprite = TFT_eSprite(&tft);
bool sharedTableSpriteReady = false;
bool sharedTableSpriteInitTried = false;

static bool ensureSharedTableSprite() {
  if (tft.usesScaledUi()) {
    return false;
  }
  if (sharedTableSpriteReady) {
    return true;
  }
  if (sharedTableSpriteInitTried) {
    return false;
  }

  sharedTableSpriteInitTried = true;
  sharedTableSprite.setColorDepth(TFT_TABLE_SPRITE_COLOR_DEPTH);
  sharedTableSpriteReady = (sharedTableSprite.createSprite(TFT_TABLE_WIDTH, TFT_TABLE_HEIGHT) != nullptr);
  if (!sharedTableSpriteReady) {
    LOGV_PRINTLN("[TFT] Shared table sprite alloc failed, fallback to direct draw");
    return false;
  }
  return true;
}

static bool isTableFooterScreen(ScreenType screenNum) {
  return (screenNum == SCREEN_DX_CLUSTER ||
          screenNum == SCREEN_POTA_CLUSTER ||
          screenNum == SCREEN_HAMALERT_CLUSTER ||
          screenNum == SCREEN_APRS_IS ||
          screenNum == SCREEN_WEATHER_DSP ||
          screenNum == SCREEN_WEATHER_FORECAST);
}

static bool isTableNavFooterVisible(ScreenType screenNum) {
  return isTableFooterScreen(screenNum) && millis() < tableNavFooterVisibleUntilMs;
}

static int getTableMaxRowsForScreen(ScreenType screenNum) {
  return isTableNavFooterVisible(screenNum) ? 10 : 11;
}

static int getTableBottomForScreen(ScreenType screenNum) {
  return isTableNavFooterVisible(screenNum) ? TFT_TABLE_BOTTOM : 320;
}

extern uint16_t menuThemeColor;

// Deklaracje zmiennych menu (definiowane później w kodzie)
extern bool brightnessMenuActive;
extern bool screenSaverMenuActive;
extern bool screenSleepMenuActive;
extern bool touchCalActive;
extern bool pskKeyboardActive;  // Klawiatura PSK aktywna

static void drawSwitchScreenFooter() {
  // Nie rysuj strzałek gdy jesteśmy w menu ustawień
  if (inMenu || brightnessMenuActive || screenSaverMenuActive || screenSleepMenuActive || touchCalActive || pskKeyboardActive) {
    return;
  }
  // Strzałki nawigacyjne - duże, blisko krawędzi (takie same na wszystkich ekranach)
  int arrowY = 290;
  int arrowSize = 12;
  tft.fillTriangle(15, arrowY, 15 + arrowSize, arrowY - arrowSize, 15 + arrowSize, arrowY + arrowSize, menuThemeColor);
  tft.fillTriangle(465, arrowY, 465 - arrowSize, arrowY - arrowSize, 465 - arrowSize, arrowY + arrowSize, menuThemeColor);
  tft.setTextColor(0x52AA);
  tft.setTextSize(1);
  tft.setCursor(195, 286);
  tft.print("SWITCH SCREEN");
}

// Ustawienie trybu czasu dla ekranu 1
enum Screen1TimeMode {
  SCREEN1_TIME_UTC = 0,
  SCREEN1_TIME_LOCAL = 1
};
uint8_t screen1TimeMode = SCREEN1_TIME_UTC;

enum TftLang : uint8_t {
  TFT_LANG_PL = 0,
  TFT_LANG_EN = 1
};
uint8_t tftLanguage = TFT_LANG_PL;

enum DxTableSizeMode : uint8_t {
  DX_TABLE_SIZE_NORMAL = 0,
  DX_TABLE_SIZE_ENLARGED = 1
};
uint8_t dxTableSizeMode = DX_TABLE_SIZE_NORMAL;

// Font z polskimi znakami dla daty/dnia tygodnia
#define ROBOTO_FONT10_NAME "/fonts/Inter-Regular10"
#define ROBOTO_FONT10_FILE "/fonts/Inter-Regular10.vlw"
#define ROBOTO_FONT12_NAME "/fonts/Inter-Regular12"
#define ROBOTO_FONT12_FILE "/fonts/Inter-Regular12.vlw"
#define ROBOTO_FONT20_NAME "/fonts/Inter-Regular20"
#define ROBOTO_FONT20_FILE "/fonts/Inter-Regular20.vlw"
#define ROBOTO_FONT24_NAME "/fonts/Inter-Regular24"
#define ROBOTO_FONT24_FILE "/fonts/Inter-Regular24.vlw"

enum TrKey : uint8_t {
  TR_TIME_SHORT = 0,
  TR_CALL_SHORT,
  TR_COUNTRY,
  TR_WAITING_SPOTS,
  TR_WEATHER,
  TR_TEMPERATURE,
  TR_HUMIDITY,
  TR_PRESSURE,
  TR_WIND,
  TR_FORECAST_3H,
  TR_FORECAST_TOMORROW,
  TR_NO_DATA,
  TR_ERROR_PREFIX,
  TR_PAGE,
  TR_TFT_CALIBRATION_HINT,
  TR_TFT_CALIBRATE_BTN,
  TR_ROT_90_RIGHT,
  TR_ROT_90_LEFT,
  TR_DISPLAY_SETTINGS,
  TR_BRIGHTNESS,
  TR_THEME_COLOR,
  TR_HOLD_CAL_HINT,
  TR_SAVE,
  TR_DEFAULT,
  TR_CLOSE,
  TR_LANGUAGE,
  TR_KEY_COUNT
};

static const char* TR_PL[TR_KEY_COUNT] = {
  "Czas",
  "Znak",
  "KRAJ",
  "Oczekiwanie na spoty...",
  "POGODA",
  "TEMPERATURA",
  "WILGOTNOŚĆ",
  "CIŚNIENIE",
  "WIATR",
  "Prognoza na 3 godziny:",
  "Prognoza na jutro:",
  "Brak danych",
  "BŁĄD: ",
  "Strona",
  "Przytrzymaj 5 sek = Kalibracja",
  "TFT Kalibracja",
  "Obrót 90deg w prawo (rot90cw)",
  "Obrót 90deg w lewo (rot90ccw)",
  "USTAWIENIA TFT",
  "JASNOŚĆ:",
  "KOLOR MOTYWU:",
  "Przytrzymaj 3 sek = Kalibracja",
  "ZAPISZ",
  "DOMYŚLNE",
  "ZAMKNIJ",
  "JĘZYK"
};

static const char* TR_EN[TR_KEY_COUNT] = {
  "Time",
  "Call",
  "COUNTRY",
  "Waiting for spots...",
  "WEATHER",
  "TEMPERATURE",
  "HUMIDITY",
  "PRESSURE",
  "WIND",
  "Forecast (3 hours):",
  "Forecast for tomorrow:",
  "No data",
  "ERROR: ",
  "Page",
  "Hold 5 sec anywhere = Calibration:",
  "TFT Calibrate",
  "Rotate 90 deg right (rot90cw)",
  "Rotate 90 deg left (rot90ccw)",
  "DISPLAY SETTINGS",
  "BRIGHTNESS:",
  "THEME COLOR:",
  "Hold 3 sec anywhere = Calibration",
  "SAVE",
  "DEFAULT",
  "CLOSE",
  "LANGUAGE"
};

static const char* tr(TrKey key) {
  uint8_t idx = static_cast<uint8_t>(key);
  if (idx >= TR_KEY_COUNT) {
    return "";
  }
  return (tftLanguage == TFT_LANG_EN) ? TR_EN[idx] : TR_PL[idx];
}

static const char* tftLangToCode(uint8_t lang) {
  return (lang == TFT_LANG_EN) ? "en" : "pl";
}

static uint8_t tftLangFromCode(const String &code) {
  String up = code;
  up.toLowerCase();
  if (up == "en") {
    return TFT_LANG_EN;
  }
  return TFT_LANG_PL;
}

static const char* dxTableSizeToCode(uint8_t mode) {
  return (mode == DX_TABLE_SIZE_ENLARGED) ? "enlarged" : "normal";
}

static uint8_t dxTableSizeFromCode(const String &code) {
  String up = code;
  up.toLowerCase();
  if (up == "enlarged" || up == "large" || up == "big") {
    return DX_TABLE_SIZE_ENLARGED;
  }
  return DX_TABLE_SIZE_NORMAL;
}

static bool isDxTableEnlarged() {
  return dxTableSizeMode == DX_TABLE_SIZE_ENLARGED;
}

static int getDxTableMaxRows() {
  if (!isDxTableEnlarged()) {
    return getTableMaxRowsForScreen(SCREEN_DX_CLUSTER);
  }
  return isTableNavFooterVisible(SCREEN_DX_CLUSTER) ? 6 : 7;
}

static int getPotaTableMaxRows() {
  if (!isDxTableEnlarged()) {
    return getTableMaxRowsForScreen(SCREEN_POTA_CLUSTER);
  }
  return isTableNavFooterVisible(SCREEN_POTA_CLUSTER) ? 6 : 7;
}

static int getHamalertTableMaxRows() {
  if (!isDxTableEnlarged()) {
    return getTableMaxRowsForScreen(SCREEN_HAMALERT_CLUSTER);
  }
  return isTableNavFooterVisible(SCREEN_HAMALERT_CLUSTER) ? 6 : 7;
}

static int getAprsTableMaxRows() {
  if (!isDxTableEnlarged()) {
    return getTableMaxRowsForScreen(SCREEN_APRS_IS);
  }
  return isTableNavFooterVisible(SCREEN_APRS_IS) ? 6 : 7;
}

static uint16_t getAprsCallsignColorForEnlarged(const APRSStation &station) {
  String symbolShort = getAPRSSymbolShort(station);
  if (symbolShort == "HOUSE") return TFT_YELLOW;
  if (symbolShort == "HUMAN") return TFT_BLUE;
  if (symbolShort == "CAR") return TFT_RED;
  return TFT_WHITE;
}
#endif

// Funkcja zamieniająca polskie znaki na ASCII (dla wyświetlacza TFT)
String toAsciiPolish(const String& input) {
  String output = input;
  // Małe litery
  output.replace("ą", "a");
  output.replace("ć", "c");
  output.replace("ę", "e");
  output.replace("ł", "l");
  output.replace("ń", "n");
  output.replace("ó", "o");
  output.replace("ś", "s");
  output.replace("ź", "z");
  output.replace("ż", "z");
  // Duże litery
  output.replace("Ą", "A");
  output.replace("Ć", "C");
  output.replace("Ę", "E");
  output.replace("Ł", "L");
  output.replace("Ń", "N");
  output.replace("Ó", "O");
  output.replace("Ś", "S");
  output.replace("Ź", "Z");
  output.replace("Ż", "Z");
  return output;
}

// Kolorystyka motywu menu
#define DEFAULT_MENU_THEME_COLOR 0xFD20
#define DEFAULT_MENU_THEME_HUE 20
uint8_t menuThemeHue = DEFAULT_MENU_THEME_HUE;
uint16_t menuThemeColor = DEFAULT_MENU_THEME_COLOR;

// Filtry ekranu 2 (TFT)
enum Screen2FilterMode {
  FILTER_MODE_ALL = 0,
  FILTER_MODE_CW = 1,
  FILTER_MODE_SSB = 2,
  FILTER_MODE_DIGI = 3
};
Screen2FilterMode screen2FilterMode = FILTER_MODE_ALL;
int screen2FilterBandIndex = 0; // 0 = ALL
const char *SCREEN2_FILTER_BANDS[] = {"ALL", "240m", "80m", "40m", "20m", "17m", "15m", "12m", "10m"};
const int SCREEN2_FILTER_BANDS_COUNT = sizeof(SCREEN2_FILTER_BANDS) / sizeof(SCREEN2_FILTER_BANDS[0]);
const size_t COUNTRY_COL_MAX_LEN = 10;

// Filtry ekranu 7 (POTA)
Screen2FilterMode screen7FilterMode = FILTER_MODE_ALL;
int screen7FilterBandIndex = 0; // 0 = ALL

// Filtry ekranu 8 (HAMALERT)
Screen2FilterMode screen8FilterMode = FILTER_MODE_ALL;
int screen8FilterBandIndex = 0; // 0 = ALL
const char *SCREEN8_FILTER_BANDS[] = {"ALL", "240m", "80m", "40m", "20m", "17m", "15m", "12m", "10m", "VHF", "UHF", "SHF"};
const int SCREEN8_FILTER_BANDS_COUNT = sizeof(SCREEN8_FILTER_BANDS) / sizeof(SCREEN8_FILTER_BANDS[0]);

// Sortowanie ekranu 6 (APRS)
enum Screen6SortMode {
  APRS_SORT_TIME = 0,
  APRS_SORT_CALLSIGN = 1,
  APRS_SORT_DISTANCE = 2
};

enum Screen6ViewMode : uint8_t {
  APRS_VIEW_LIST = 0,
  APRS_VIEW_RADAR = 1
};

Screen6SortMode screen6SortMode = APRS_SORT_TIME;
Screen6ViewMode screen6ViewMode = APRS_VIEW_LIST;
bool screen6MenuBeaconingTemp = true;
bool screen6MenuAprsAlertTemp = true;
bool screen6MenuRangeAlertTemp = true;
bool screen6MenuLedAlertTemp = true;

const int SCREEN6_VIEW_BTN_ICON_X = 294;
const int SCREEN6_VIEW_BTN_ICON_Y = 7;
const int SCREEN6_VIEW_BTN_HIT_X = 276;
const int SCREEN6_VIEW_BTN_HIT_Y = 0;
const int SCREEN6_VIEW_BTN_HIT_W = 44;
const int SCREEN6_VIEW_BTN_HIT_H = 50;
const float SCREEN6_RADAR_ZOOM_MIN = 1.00f;
const float SCREEN6_RADAR_ZOOM_MAX = 3.00f;
const float SCREEN6_RADAR_ZOOM_STEP = 0.25f;
const unsigned long SCREEN6_RADAR_ZOOM_TAP_COOLDOWN_MS = 220UL;
float screen6RadarZoom = 1.00f;
unsigned long screen6RadarLastZoomTapMs = 0;
const unsigned long SCREEN6_RADAR_HINT_DURATION_MS = 3000UL;
unsigned long screen6RadarHintUntilMs = 0;

// Menu jasnosci (wywolywane dlugim przytrzymaniem)
bool brightnessMenuActive = false;
int brightnessMenuValue = 100;
ScreenType brightnessMenuPrevScreen = SCREEN_HAM_CLOCK;
bool brightnessMenuPrevInMenu = false;
uint8_t brightnessMenuPrevThemeHue = DEFAULT_MENU_THEME_HUE;
int brightnessMenuPrevBacklight = TFT_BACKLIGHT;
unsigned long brightnessMenuOpenedMs = 0;
unsigned long brightnessMenuTouchStartMs = 0;
bool brightnessMenuLongPressHandled = false;

bool touchCalActive = false;
uint8_t touchCalStep = 0;
int16_t touchCalRawX1 = 0;
int16_t touchCalRawY1 = 0;
int16_t touchCalRawX2 = 0;
int16_t touchCalRawY2 = 0;
int16_t touchCalRawX3 = 0;
int16_t touchCalRawY3 = 0;
int16_t touchCalRawX4 = 0;
int16_t touchCalRawY4 = 0;
int touchCalNewXMin = TOUCH_X_MIN;
int touchCalNewXMax = TOUCH_X_MAX;
int touchCalNewYMin = TOUCH_Y_MIN;
int touchCalNewYMax = TOUCH_Y_MAX;

// ========== KONFIGURACJA ==========
#define AP_SSID "ESP32-HAM-CLOCK"
#define AP_PASSWORD "1234567890"
#define DEFAULT_CLUSTER_HOST "dxspots.com"
#define DEFAULT_CLUSTER_PORT 7300
#define DEFAULT_POTA_CLUSTER_HOST ""
#define DEFAULT_POTA_CLUSTER_PORT 7300
#define DEFAULT_POTA_FILTER_COMMAND "accept/spot comment POTA"
#define DEFAULT_POTA_API_URL "https://api.pota.app/v1/spots"
#define DEFAULT_HAMALERT_HOST "hamalert.org"
#define DEFAULT_HAMALERT_PORT 7300
#define NTP_SERVER "pool.ntp.org"
#define MAX_SPOTS 50  // Bufor 50 ostatnich spotów
#define MAX_POTA_SPOTS 30  // Bufor 30 ostatnich spotów (TFT pokaże max 10)
#define GMT_OFFSET_SEC 0  // UTC
#define DEFAULT_TIMEZONE_HOURS 1  // UTC+1 dla Polski (zimowy), DST doda +1 latem
#define DEFAULT_CALLSIGN "SWL"
#define DEFAULT_OPENWEBRX_URL "http://okno.ddns.net:8078"
#define PROPAGATION_URL "https://www.hamqsl.com/solarxml.php"
const unsigned long PROPAGATION_FETCH_INTERVAL_MS = 60UL * 60UL * 1000UL; // 60 min
const unsigned long PROPAGATION_FETCH_RETRY_MS = 5UL * 60UL * 1000UL;      // 5 min retry on error
const unsigned long WEATHER_FETCH_INTERVAL_MS = 10UL * 60UL * 1000UL;      // 10 min
const unsigned long WEATHER_FETCH_RETRY_MS = 2UL * 60UL * 1000UL;          // 2 min retry on error
const unsigned long QRZ_LOOKUP_INTERVAL_DEFAULT_MS = 3000;                 // 3s bazowy interwał lookupów
const unsigned long QRZ_LOOKUP_INTERVAL_DX_MS = 2000;                      // 2s na ekranie DX (tak samo jak POTA)
const unsigned long QRZ_LOOKUP_INTERVAL_POTA_MS = 2000;                    // 2s na ekranie POTA
const unsigned long QRZ_RETRY_DELAY_MS = 5000;                             // 5s retry
const unsigned long QRZ_CACHE_TTL_MS = 15UL * 60UL * 1000UL;               // 15 min cache wpisu
const uint8_t QRZ_RETRY_LIMIT = 2;
const int QRZ_QUEUE_SIZE = 20;

// ========== STRUKTURA DANYCH ==========
struct DXSpot {
  String time;        // Czas UTC
  String spotter;     // Stacja zgłaszająca
  String callsign;    // Znak wywoławczy
  float frequency;    // Częstotliwość (kHz - jak w DX Cluster)
  String comment;     // Komentarz
  float distance;     // Odległość (km)
  String country;     // Kraj (z QRZ, jeśli dostępny)
  String name;        // Imię i nazwisko (z QRZ, jeśli dostępne)
  String locator;     // Maidenhead Locator (jeśli dostępny)
  float lat;          // Szerokosc geo (jesli znana)
  float lon;          // Dlugosc geo (jesli znana)
  bool hasLatLon;     // Czy lat/lon jest znane
  String band;        // Pasmo (240m, 80m, 40m, etc.)
  String mode;        // Modulacja (CW, SSB, FT8/FT4)
};

// Struktura dla stacji APRS
struct APRSStation {
  String time;        // Czas UTC (timestamp)
  String callsign;    // Znak wywoławczy nadawcy
  String symbol;      // Symbol APRS (raw)
  String symbolTable; // Table symbol (znak przed /)
  float lat;          // Szerokość geograficzna
  float lon;          // Długość geograficzna
  String comment;     // Komentarz
  float freqMHz;      // Częstotliwość z komentarza (MHz)
  float distance;     // Odległość w km (Haversine)
  bool hasLatLon;     // Czy pozycja jest znana
};

struct AprsWxDecoded {
  bool hasAny = false;
  bool hasWindDir = false;
  int windDirDeg = 0;
  bool hasWindSpeed = false;
  float windSpeedKmh = 0.0f;
  bool hasTempC = false;
  float tempC = 0.0f;
  bool hasHumidity = false;
  int humidityPct = 0;
  bool hasPressure = false;
  float pressureHpa = 0.0f;

  // Starszy ekran APRS weather używa tych nazw pól bezpośrednio.
  int windDir = 0;
  float windSpeed = 0.0f;
  float temp = 0.0f;
  int humidity = 0;
  float pressure = 0.0f;
};

// ========== ZMIENNE GLOBALNE ==========
WebServer* server = nullptr;
Preferences* preferences = nullptr;
WiFiClient telnetClient;
WiFiClient potaTelnetClient;
WiFiClient aprsClient;  // Klient dla APRS-IS

struct PropagationData {
  String sfi;
  String ssn;
  String kindex;
  String aindex;
  String xray;
  String muf;
  String updated;
  String hfBandLabel[4];
  String hfBandFreq[4];
  String hfBandDay[4];
  String hfBandNight[4];
  bool valid = false;
  String lastError = "";
  unsigned long fetchedAtMs = 0;
};

PropagationData propagationData;
unsigned long lastPropagationFetchMs = 0;
bool lastPropagationFetchOk = true;

struct WeatherData {
  static const uint8_t DETAIL_COLS = WEATHER_DETAIL_COLS;
  String cityName;
  String description;
  String iconCode;  // OWM icon code (e.g., 01d/01n) to detect day/night
  int weatherId = 800; // OpenWeatherMap condition ID (default clear sky)
  float tempC = 0.0f;
  int humidity = 0;
  int pressure = 0;
  float windMs = 0.0f;
  float pm25 = 0.0f;  // PM2.5 w Âµg/mÂł
  float pm10 = 0.0f;  // PM10 w Âµg/mÂł
  // Prognozy
  float forecast3hTempC = 0.0f;
  float forecast3hWindMs = 0.0f;
  String forecast3hDesc;
  bool forecast3hValid = false;
  float forecastNextDayTempC = 0.0f;
  float forecastNextDayWindMs = 0.0f;
  String forecastNextDayDesc;
  bool forecastNextDayValid = false;
  float detailTempC[DETAIL_COLS] = {0, 0, 0, 0, 0};
  int detailHumidity[DETAIL_COLS] = {0, 0, 0, 0, 0};
  float detailWindMs[DETAIL_COLS] = {0, 0, 0, 0, 0};
  int detailWeatherId[DETAIL_COLS] = {800, 800, 800, 800, 800};
  String detailIconCode[DETAIL_COLS];
  bool detailValid[DETAIL_COLS] = {false, false, false, false, false};
  float nightTempC[2] = {0.0f, 0.0f};
  bool nightTempValid[2] = {false, false};
  String updated;
  bool valid = false;
  String lastError = "";
  unsigned long fetchedAtMs = 0;
};

WeatherData weatherData;
unsigned long lastWeatherFetchMs = 0;
bool lastWeatherFetchOk = true;

struct PendingQrzLookup {
  String callsign;
  unsigned long nextTryMs = 0;
  uint8_t attempts = 0;
};

PendingQrzLookup qrzQueue[QRZ_QUEUE_SIZE];
int qrzQueueLen = 0;
unsigned long lastQrzLookupMs = 0;

// Zwraca interwał dla kolejki QRZ zależnie od aktywnego ekranu
unsigned long getQrzLookupIntervalMs() {
#ifdef ENABLE_TFT_DISPLAY
  if (tftInitialized && !inMenu) {
    if (currentScreen == SCREEN_DX_CLUSTER) {
      return QRZ_LOOKUP_INTERVAL_DX_MS;
    }
    if (currentScreen == SCREEN_POTA_CLUSTER) {
      return QRZ_LOOKUP_INTERVAL_POTA_MS;
    }
  }
#endif
  return QRZ_LOOKUP_INTERVAL_DEFAULT_MS;
}

DXSpot spots[MAX_SPOTS];
int spotCount = 0;
DXSpot potaSpots[MAX_POTA_SPOTS];
int potaSpotCount = 0;
DXSpot hamalertSpots[MAX_POTA_SPOTS];
int hamalertSpotCount = 0;
SemaphoreHandle_t dxSpotsMutex = nullptr;

const size_t DX_TIME_MAX_LEN = 24;
const size_t DX_CALLSIGN_MAX_LEN = 16;
const size_t DX_SPOTTER_MAX_LEN = 16;
const size_t DX_COMMENT_MAX_LEN = 96;
const size_t DX_COUNTRY_MAX_LEN = 28;
const size_t DX_NAME_MAX_LEN = 32;
const size_t DX_LOCATOR_MAX_LEN = 8;
const size_t DX_BAND_MAX_LEN = 12;
const size_t DX_MODE_MAX_LEN = 8;

const size_t APRS_TIME_MAX_LEN = 24;
const size_t APRS_CALLSIGN_MAX_LEN = 16;
const size_t APRS_SYMBOL_MAX_LEN = 12;
const size_t APRS_COMMENT_MAX_LEN = 96;

static inline void clampStringLength(String &value, size_t maxLen) {
  if (value.length() > maxLen) {
    value.remove(maxLen);
  }
}

static void compactDxSpotStrings(DXSpot &spot) {
  clampStringLength(spot.time, DX_TIME_MAX_LEN);
  clampStringLength(spot.callsign, DX_CALLSIGN_MAX_LEN);
  clampStringLength(spot.spotter, DX_SPOTTER_MAX_LEN);
  clampStringLength(spot.comment, DX_COMMENT_MAX_LEN);
  clampStringLength(spot.country, DX_COUNTRY_MAX_LEN);
  clampStringLength(spot.name, DX_NAME_MAX_LEN);
  clampStringLength(spot.locator, DX_LOCATOR_MAX_LEN);
  clampStringLength(spot.band, DX_BAND_MAX_LEN);
  clampStringLength(spot.mode, DX_MODE_MAX_LEN);
}

static void compactAprsStationStrings(APRSStation &station) {
  clampStringLength(station.time, APRS_TIME_MAX_LEN);
  clampStringLength(station.callsign, APRS_CALLSIGN_MAX_LEN);
  clampStringLength(station.symbol, APRS_SYMBOL_MAX_LEN);
  clampStringLength(station.symbolTable, APRS_SYMBOL_MAX_LEN);
  clampStringLength(station.comment, APRS_COMMENT_MAX_LEN);
}

static inline void lockDxSpots() {
  if (dxSpotsMutex != nullptr) {
    xSemaphoreTake(dxSpotsMutex, portMAX_DELAY);
  }
}

static inline void unlockDxSpots() {
  if (dxSpotsMutex != nullptr) {
    xSemaphoreGive(dxSpotsMutex);
  }
}

// Forward declarations for timezone conversion
extern int timezoneHours;
bool isEuropeanDST(struct tm *timeinfo);

// Global cache for timezone calculation to avoid repeated time() calls
static int cachedTimezoneOffset = -999;  // -999 means not initialized
static unsigned long lastTimezoneCacheMs = 0;
static const unsigned long TIMEZONE_CACHE_MS = 60000; // Cache for 1 minute

static int getCachedTimezoneOffset() {
  unsigned long nowMs = millis();
  if (cachedTimezoneOffset == -999 || (nowMs - lastTimezoneCacheMs) > TIMEZONE_CACHE_MS) {
    int dstOffset = 0;
    time_t now = time(nullptr);
    if (now > 100000) {
      struct tm utcTm;
      gmtime_r(&now, &utcTm);
      dstOffset = isEuropeanDST(&utcTm) ? 1 : 0;
    }
    cachedTimezoneOffset = timezoneHours + dstOffset;
    lastTimezoneCacheMs = nowMs;
  }
  return cachedTimezoneOffset;
}

String formatSpotUtc(String raw) {
  raw.trim();

  // Obsługa ISO 8601 (np. 2024-12-12T12:34Z)
  int tPos = raw.indexOf('T');
  if (tPos >= 0 && (tPos + 5) <= (int)raw.length()) {
    raw = raw.substring(tPos + 1);
  }

  // Usuń ewentualne końcowe "Z"
  if (raw.endsWith("Z") || raw.endsWith("z")) {
    raw.remove(raw.length() - 1);
  }

  int hour = 0, minute = 0;
  bool parsed = false;
  
  // Jeśli już jest dwukropek, przytnij do HH:MM
  if (raw.length() >= 5 && raw.charAt(2) == ':') {
    hour = raw.substring(0, 2).toInt();
    minute = raw.substring(3, 5).toInt();
    parsed = true;
  } else if (raw.length() >= 4) {
    // Brak dwukropka: spróbuj wstawić między HH a MM (np. "1234" -> "12:34")
    hour = raw.substring(0, 2).toInt();
    minute = raw.substring(2, 4).toInt();
    parsed = true;
  } else if (raw.length() >= 2) {
    // Awaryjnie: tylko godzina
    hour = raw.substring(0, 2).toInt();
    minute = 0;
    parsed = true;
  }
  
  if (!parsed) {
    return raw; // Nie udało się sparsować
  }

  // Konwersja na czas lokalny (użyj cache'owanego offsetu)
  int offset = getCachedTimezoneOffset();
  int localHour = hour + offset;
  
  // Obsługa przejścia przez północ
  if (localHour < 0) localHour += 24;
  else if (localHour >= 24) localHour -= 24;
  
  // Formatuj wynik używając statycznego bufora (bez alokacji String)
  char result[6];
  snprintf(result, sizeof(result), "%02d:%02d", localHour, minute);
  return String(result);
}

// POTA API (HTTP)
const unsigned long POTA_API_FETCH_INTERVAL_MS = 180UL * 1000UL; // 180s
unsigned long lastPotaApiFetchMs = 0;

// HAMALERT Telnet
const unsigned long HAMALERT_FETCH_INTERVAL_MS = 60UL * 1000UL; // 60s
unsigned long lastHamalertFetchMs = 0;

// APRS-IS konfiguracja (domyślne wartości)
#define DEFAULT_APRS_IS_HOST "rotate.aprs2.net"
#define DEFAULT_APRS_IS_PORT 14580
#define DEFAULT_APRS_CALLSIGN "nocall"
#define DEFAULT_APRS_PASSCODE 00000
#define DEFAULT_APRS_SSID 0
#define DEFAULT_APRS_FILTER_RADIUS 50  // Promień w km (domyślnie 50, zakres 1-50)
#define MAX_APRS_STATIONS 20  // Maksymalna liczba stacji do wyświetlenia (bufor dla WWW)
#define MAX_APRS_DISPLAY_LCD 11  // Maksymalna liczba stacji na ekranie LCD

// Zmienne konfiguracyjne APRS-IS
String aprsIsHost = DEFAULT_APRS_IS_HOST;
int aprsIsPort = DEFAULT_APRS_IS_PORT;
String aprsCallsign = DEFAULT_APRS_CALLSIGN;
int aprsPasscode = DEFAULT_APRS_PASSCODE;
int aprsSsid = DEFAULT_APRS_SSID;
int aprsFilterRadius = DEFAULT_APRS_FILTER_RADIUS;  // PromieÄąâ€ž w km (0-30)
// Uwaga: APRS uÄąÄ˝ywa wspÄ‚łÄąâ€šrzĂ„â„˘dnych z sekcji "Moja Stacja" (userLat, userLon)


AprsWxDecoded currentWX;
String lastWxStation = "N/A";
const unsigned long APRS_POSITION_FIRST_DELAY_MS = 60UL * 1000UL; // pierwszy beacon po 1 minucie
const char* aprsServer = "rotate.aprs2.net";
const int aprsPort = 14580;
const char* aprsPass = "12345"; // TWÓJ PASSCODE
const char* aprsFilter = "t/m/t/w"; // Filtruj tylko wiadomości i stacje pogodowe (WX)
const int DEFAULT_APRS_INTERVAL_MIN = 29; // kolejne co 29 minut (domyślnie)
const char *NVS_KEY_APRS_INTERVAL_MIN = "aprs_int_min";
const int DEFAULT_APRS_ALERT_MIN_SEC = 300;
const char *NVS_KEY_APRS_ALERT_MIN_SEC = "aprs_alrt_sec";
const int DEFAULT_APRS_ALERT_SCREEN_SEC = 5;
const char *NVS_KEY_APRS_ALERT_SCREEN_SEC = "alrt_scr_s";
const float DEFAULT_APRS_ALERT_DISTANCE_KM = 1.0f;
const char *NVS_KEY_APRS_ALERT_DISTANCE_KM = "alrt_dst_km";
const char *NVS_KEY_APRS_ALERT_WX_ENABLED = "alrt_wx_en";
const bool DEFAULT_ENABLE_LED_ALERT = true;
const int DEFAULT_LED_ALERT_DURATION_MS = 5000;
const int DEFAULT_LED_ALERT_BLINK_MS = 500;
const char *NVS_KEY_ENABLE_LED_ALERT = "led_al_en";
const char *NVS_KEY_LED_ALERT_DURATION_MS = "led_al_dur";
const char *NVS_KEY_LED_ALERT_BLINK_MS = "led_al_blk";
unsigned long aprsPositionIntervalMs = (unsigned long)DEFAULT_APRS_INTERVAL_MIN * 60UL * 1000UL;
unsigned long lastAPRSPositionTxMs = 0;
unsigned long nextAPRSPositionDueMs = 0;
int aprsIntervalMinutes = DEFAULT_APRS_INTERVAL_MIN;
const char DEFAULT_APRS_SYMBOL_TABLE = '/';
const char DEFAULT_APRS_SYMBOL_CODE = '-';
String aprsSymbolTwoChar = "/-";
char aprsSymbolTable = DEFAULT_APRS_SYMBOL_TABLE;
char aprsSymbolCode = DEFAULT_APRS_SYMBOL_CODE;
String aprsUserComment = "";
String aprsAlertCsv = "";
bool aprsAlertEnabled = true;
int aprsAlertMinSeconds = DEFAULT_APRS_ALERT_MIN_SEC;
int aprsAlertScreenSeconds = DEFAULT_APRS_ALERT_SCREEN_SEC;
bool aprsAlertNearbyEnabled = true;
bool aprsAlertWxEnabled = false;
float aprsAlertDistanceKm = DEFAULT_APRS_ALERT_DISTANCE_KM;
bool enableLedAlert = DEFAULT_ENABLE_LED_ALERT;
int ledAlertDurationMs = DEFAULT_LED_ALERT_DURATION_MS;
int ledAlertBlinkMs = DEFAULT_LED_ALERT_BLINK_MS;
int aprsBeaconTxCount = 0;
const char *APRS_POSITION_COMMENT = "https://github.com/SP3KON/ESP32-HAM-CLOCK";
bool aprsBeaconEnabled = true;
const unsigned long APRS_ALERT_FRAME_PULSE_MS = 500UL;
const int APRS_ALERT_CLOSE_BTN_X = 449;
const int APRS_ALERT_CLOSE_BTN_Y = 7;
const int APRS_ALERT_CLOSE_BTN_W = 26;
const int APRS_ALERT_CLOSE_BTN_H = 26;
const int APRS_ALERT_CLOSE_HIT_W = 50;
const int APRS_ALERT_CLOSE_HIT_H = 50;
bool aprsAlertScreenActive = false;
unsigned long aprsAlertScreenUntilMs = 0;
unsigned long aprsAlertFrameLastToggleMs = 0;
bool aprsAlertFramePulseOn = true;
APRSStation aprsAlertScreenStation;
volatile bool aprsAlertDrawPending = false;
APRSStation aprsAlertPendingStation;
portMUX_TYPE aprsAlertPendingMux = portMUX_INITIALIZER_UNLOCKED;

struct AprsAlertCooldownEntry {
  String callsign;
  unsigned long lastAlertMs = 0;
};

AprsAlertCooldownEntry aprsAlertCooldown[MAX_APRS_STATIONS];
int aprsAlertCooldownReplaceIdx = 0;

APRSStation aprsStations[MAX_APRS_STATIONS];
int aprsStationCount = 0;
bool aprsConnected = false;
bool aprsLoginSent = false;
unsigned long lastAPRSAttempt = 0;
unsigned long lastAPRSRxMs = 0;
String aprsBuffer = "";
const unsigned long APRS_INACTIVITY_RECONNECT_MS = 5UL * 60UL * 1000UL; // 5 minut

void parseAPRSWeather(String line) {
  // Szukamy początku danych pogodowych (znak '_')
  int wxIdx = line.indexOf('_');
  if (wxIdx == -1) return;

  // Pobranie znaku stacji (na początku linii przed '>')
  int callEnd = line.indexOf('>');
  if (callEnd != -1) {
    lastWxStation = line.substring(0, callEnd);
  }

  // Wycinanie danych (format APRS jest stały po znaku '_')
  // Przykład: _MMDDHHmmcCCsSSgGGtTTThHHbBBBBB
  String data = line.substring(wxIdx + 1);

  // Kierunek i prędkość wiatru (c...s...)
  if (data.indexOf('c') != -1 && data.indexOf('s') != -1) {
    currentWX.windDir = data.substring(data.indexOf('c')+1, data.indexOf('c')+4).toInt();
    int miph = data.substring(data.indexOf('s')+1, data.indexOf('s')+4).toInt();
    currentWX.windSpeed = miph * 0.44704; // mph -> m/s
  }

  // Temperatura (t...) - w Fahrenheitach
  int tIdx = data.indexOf('t');
  if (tIdx != -1) {
    int tempF = data.substring(tIdx + 1, tIdx + 4).toInt();
    currentWX.temp = (tempF - 32) * 5.0 / 9.0; // F -> C
  }

  // Wilgotność (h...)
  int hIdx = data.indexOf('h');
  if (hIdx != -1) {
    currentWX.humidity = data.substring(hIdx + 1, hIdx + 3).toInt();
  }

  // Ciśnienie (b...) - w dziesiątych hPa
  int bIdx = data.indexOf('b');
  if (bIdx != -1) {
    float baro = data.substring(bIdx + 1, bIdx + 6).toFloat();
    currentWX.pressure = baro / 10.0;
  }
  
  Serial.printf("WX od %s: %.1fC, %d%%, %.1f hPa\n", lastWxStation.c_str(), currentWX.temp, currentWX.humidity, currentWX.pressure);
}

void updateAPRS() {
  if (!aprsClient.connected()) {
    if (aprsClient.connect(aprsServer, aprsPort)) {
      String loginCall = aprsCallsign.length() ? aprsCallsign : String(DEFAULT_APRS_CALLSIGN);
      String login = "user " + loginCall + " pass " + String(aprsPasscode) + " vers ESP32HamClock 1.0 filter " + aprsFilter;
      aprsClient.println(login);
    }
    return;
  }

  while (aprsClient.available()) {
    String line = aprsClient.readStringUntil('\n');
    if (line.indexOf("_") != -1 && line.indexOf("t") != -1) {
      parseAPRSWeather(line);
      if (currentScreen == SCREEN_WEATHER_DSP) lastScreenUpdate = 0;
    }
  }
}
void drawWeatherScreen() {
  tft.fillRect(0, 0, 480, 40, 0x03E0); // Ciemnozielony nagłówek
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.drawCentreString("APRS WEATHER STATION", 240, 10, 1);

  // 1. Temperatura (Duży odczyt)
  tft.setTextSize(4);
  tft.setTextColor(currentWX.temp > 0 ? TFT_ORANGE : TFT_CYAN);
  String tStr = String(currentWX.temp, 1) + " C";
  tft.drawCentreString(tStr, 240, 70, 1);

  // 2. Pozostałe parametry (Siatka 2x2)
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW);
  
  // Wilgotność
  tft.drawString("HUMIDITY:", 50, 240);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(String(currentWX.humidity) + " %", 200, 240);

  // Ciśnienie
  tft.setTextColor(TFT_YELLOW);
  tft.drawString("BARO:", 280, 240);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(String(currentWX.pressure, 1) + " hPa", 380, 240);

  // Wiatr
  tft.setTextColor(TFT_YELLOW);
  tft.drawString("WIND:", 50, 210);
  tft.setTextColor(TFT_WHITE);
  String wStr = String(currentWX.windSpeed, 1) + " m/s (" + String(currentWX.windDir) + "*)";
  tft.drawString(wStr, 200, 210);

  // 3. Info o stacji
  tft.drawFastHLine(20, 260, 440, TFT_DARKGREY);
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.drawCentreString("Last station: " + lastWxStation, 240, 280, 1);
}

// Konfiguracja WiFi i Cluster
String wifiSSID = "";
String wifiPassword = "";
String wifiSSID2 = "";
String wifiPassword2 = "";
String clusterHost = DEFAULT_CLUSTER_HOST;
int clusterPort = DEFAULT_CLUSTER_PORT;
String potaClusterHost = DEFAULT_POTA_CLUSTER_HOST;
int potaClusterPort = DEFAULT_POTA_CLUSTER_PORT;
String potaFilterCommand = DEFAULT_POTA_FILTER_COMMAND;
String potaApiUrl = DEFAULT_POTA_API_URL;
String hamalertHost = DEFAULT_HAMALERT_HOST;
int hamalertPort = DEFAULT_HAMALERT_PORT;
String hamalertLogin = "";
String hamalertPassword = "";
String userCallsign = "";
String userLocator = "";
double userLat = 0.0;
double userLon = 0.0;
bool userLatLonValid = false;
int timezoneHours = DEFAULT_TIMEZONE_HOURS;
String qrzUsername = "";
String qrzPassword = "";
String qrzStatus = "Callook.info: ready";
String weatherApiKey = "";
String openWebRxUrl = DEFAULT_OPENWEBRX_URL;

// Konfiguracja filtrÄ‚łw CC-Cluster (dxspots.com)
bool clusterNoAnnouncements = true;      // set/noann - wyłącz ogłoszenia
bool clusterNoWWV = true;                // set/nowwv - wyłącz WWV
bool clusterNoWCY = true;                // set/nowcy - wyłącz WCY
bool clusterUseFilters = false;           // Czy używać filtrów (set/filter)
String clusterFilterCommands = "";        // Dodatkowe komendy filtrÄ‚łw (np. "set/filter k,ve/pass")

// Status poÄąâ€šĂ„â€¦czenia
bool wifiConnected = false;
bool telnetConnected = false;
unsigned long lastTelnetAttempt = 0;
unsigned long lastNTPUpdate = 0;
unsigned long lastWiFiReconnectAttempt = 0;
unsigned long lastPotaAttempt = 0;

const uint8_t RGB_LED_RED_PIN = 4;
const uint8_t RGB_LED_GREEN_PIN = 16;
const uint8_t RGB_LED_BLUE_PIN = 17;
const unsigned long RGB_RED_DISCONNECTED_BLINK_MS = 1000UL;
bool rgbLedPrevWifiConnected = false;
unsigned long rgbRedBlinkLastToggleMs = 0;
bool rgbRedBlinkStateOn = false;
bool rgbBlueAprsAlertActive = false;
unsigned long rgbBlueAprsAlertUntilMs = 0;
unsigned long rgbBlueAprsLastToggleMs = 0;
bool rgbBlueAprsStateOn = false;

// ========== KONFIGURACJA PŁYTKI TP4056 I AKUMULATORA 18650 ==========
// Płytka ładowania 5V 1A z modułem TP4056 i ochroną baterii 18650
// - Wejście: 5V (micro USB/USB-C)
// - Wyjście: 4.2V (bateria naładowana) lub 5V (zasilanie zewnętrzne)
// - Pomiar napięcia przez dzielnik 100k/100k na pin ADC

#define BATTERY_MONITORING_ENABLED  // Włącz pomiar napięcia baterii
#define BATTERY_ADC_PIN 34          // Pin ADC1_CH6 (GPIO34) - pomiar napięcia baterii
#define BATTERY_ADC_RESOLUTION 12   // Rozdzielczość ADC (12 bit = 0-4095)
#define BATTERY_VOLTAGE_DIVIDER 2.0f // Dzielnik napięcia 100k/100k (1:2)
#define BATTERY_ADC_VREF 3.3f       // Napięcie referencyjne ADC (3.3V)

// Zakresy napięcia akumulatora 18650 (dla płytki z ochroną DW01A)
#define BATTERY_VOLTAGE_MAX 3.8f      // 100% naładowana (dostosowane do Twojej baterii)
#define BATTERY_VOLTAGE_NOM 3.7f    // 80% nominalna
#define BATTERY_VOLTAGE_LOW 3.5f    // 20% niski poziom (pomarańczowy)
#define BATTERY_VOLTAGE_MIN 3.3f    // 5% krytyczny (czerwony)
#define BATTERY_VOLTAGE_CUTOFF 3.0f // 0% ochrona wyłącza (DW01A)

// Interwały pomiaru i ostrzegania
#define BATTERY_CHECK_INTERVAL_MS 1000UL   // Sprawdzaj co 1 sekundę
#define BATTERY_WARN_INTERVAL_MS 60000UL   // Ostrzegaj co 60 sekund przy niskim poziomie

unsigned long lastBatteryCheckMs = 0;
float batteryVoltage = 0.0f;
int batteryPercentage = 0;
bool batteryLowWarningShown = false;
bool batteryCharging = false;  // Rozpoznawanie ładowania (napięcie > 4.25V)

// Funkcja odczytu napięcia baterii z ADC
float readBatteryVoltage() {
  // Dokładniejszy odczyt - średnia z 8 próbek
  long adcSum = 0;
  const int samples = 8;
  for (int i = 0; i < samples; i++) {
    adcSum += analogRead(BATTERY_ADC_PIN);
    delayMicroseconds(100);
  }
  float adcAvg = adcSum / (float)samples;
  float voltageAtPin = (adcAvg / 4095.0f) * BATTERY_ADC_VREF;
  return voltageAtPin * BATTERY_VOLTAGE_DIVIDER;
}

// Funkcja obliczania procentu naładowania (przybliżona krzywa Li-ion)
int calculateBatteryPercentage(float voltage) {
  if (voltage >= BATTERY_VOLTAGE_MAX) return 100;
  if (voltage <= BATTERY_VOLTAGE_CUTOFF) return 0;
  if (voltage >= BATTERY_VOLTAGE_NOM) {
    // Górna część krzywej (3.7V - 4.2V)
    return (int)(80 + (voltage - BATTERY_VOLTAGE_NOM) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_NOM) * 20);
  } else {
    // Dolna część krzywej (3.0V - 3.7V) - nieliniowa
    float factor = (voltage - BATTERY_VOLTAGE_CUTOFF) / (BATTERY_VOLTAGE_NOM - BATTERY_VOLTAGE_CUTOFF);
    return (int)(factor * factor * 80);  // Kwadratowa aproksymacja
  }
}

// Sprawdź stan baterii i zwróć true jeśli była aktualizacja
bool updateBatteryStatus() {
  unsigned long now = millis();
  if (now - lastBatteryCheckMs < BATTERY_CHECK_INTERVAL_MS) {
    return false;
  }
  lastBatteryCheckMs = now;
  
  batteryVoltage = readBatteryVoltage();
  batteryPercentage = calculateBatteryPercentage(batteryVoltage);
  
  // Wykrywanie ładowania (napięcie > 4.25V wskazuje na podłączone 5V)
  batteryCharging = (batteryVoltage > 4.25f);
  
  return true;
}

// Zwraca kolor dla poziomu baterii (do wyświetlania na TFT)
uint16_t getBatteryColor() {
  if (batteryCharging) return 0x07E0;  // Zielony podczas ładowania
  if (batteryVoltage <= BATTERY_VOLTAGE_MIN) return TFT_RED;
  if (batteryVoltage <= BATTERY_VOLTAGE_LOW) return 0xFDA0;  // Pomarańczowy
  return 0x07E0;  // Zielony
}

// Funkcja rysująca ikonę baterii na TFT
void drawBatteryIcon(int x, int y, int w, int h) {
  if (!tftInitialized) return;
  
  uint16_t color = getBatteryColor();
  int level = batteryPercentage;
  
  // Obrys baterii
  tft.drawRect(x, y, w, h, TFT_WHITE);
  // Plusik (kontakt) baterii
  tft.fillRect(x + w, y + h/4, 3, h/2, TFT_WHITE);
  
  // Wypełnienie poziomem naładowania
  int fillW = (w - 4) * level / 100;
  if (fillW < 0) fillW = 0;
  if (fillW > w - 4) fillW = w - 4;
  
  tft.fillRect(x + 2, y + 2, fillW, h - 4, color);
  // Wyczyść pozostałą część (gdy poziom spada)
  tft.fillRect(x + 2 + fillW, y + 2, (w - 4) - fillW, h - 4, TFT_BLACK);
}

// Funkcja wyświetlająca status baterii na ekranie (w nagłówku)
void drawBatteryStatus(int x, int y) {
  if (!tftInitialized) return;
  
  // Aktualizuj stan baterii
  updateBatteryStatus();
  
  // Rysuj ikonę baterii (30x16 pikseli)
  drawBatteryIcon(x, y, 30, 14);
  
  // Tekst z procentami i napięciem
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  char buf[16];
  snprintf(buf, sizeof(buf), "%d%% %.1fV", batteryPercentage, batteryVoltage);
  tft.setCursor(x + 36, y + 4);
  tft.print(buf);
  
  // Indykator ładowania
  if (batteryCharging) {
    tft.setTextColor(0x07E0);  // Zielony
    tft.setCursor(x + 36, y - 8);
    tft.print("[ładowanie]");
  }
}

// Szybka aktualizacja tylko ikony baterii (bez odświeżania całego nagłówka)
// Używana w loop() aby uniknąć watchdog resetu
void drawBatteryQuickUpdate(bool skipScreenCheck = false) {
  if (!tftInitialized) return;
  if (!skipScreenCheck && (currentScreen != SCREEN_HAM_CLOCK || inMenu)) return;
  if (brightnessMenuActive || screenSaverMenuActive) return;  // Nie rysuj podczas menu
  
  // Sprawdź czy dzielnik jest podłączony
  if (batteryVoltage < 1.5f || batteryVoltage > 5.0f) return;
  
  int screenW = tft.width();
  int x = screenW - 120, y = 10, w = 26, h = 12;  // Prawa strona ekranu (bardziej w lewo)
  
  // Wyczyść obszar ikony baterii przed narysowaniem (unika pasków/śmieci)
  tft.fillRect(x - 2, y - 2, w + 6, h + 4, TFT_BLACK);
  
  // Rysuj ikonę baterii
  uint16_t color = getBatteryColor();
  int level = batteryPercentage;
  
  tft.drawRect(x, y, w, h, TFT_WHITE);
  tft.fillRect(x + w, y + h/4, 3, h/2, TFT_WHITE);
  
  int fillW = (w - 4) * level / 100;
  if (fillW < 0) fillW = 0;
  if (fillW > w - 4) fillW = w - 4;
  
  tft.fillRect(x + 2, y + 2, fillW, h - 4, color);
  tft.fillRect(x + 2 + fillW, y + 2, (w - 4) - fillW, h - 4, TFT_BLACK);
  
  // Tekst z procentem baterii - wyczyść tło przed napisaniem
  tft.fillRect(x + 32, y + 3, 30, 8, TFT_BLACK);  // Wyczyść obszar tekstu
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  char batBuf[16];
  snprintf(batBuf, sizeof(batBuf), "%d%%", batteryPercentage);
  tft.setCursor(x + 32, y + 3);
  tft.print(batBuf);
}

// ========== Restart (np. po zapisaniu konfiguracji)
bool restartRequested = false;
unsigned long restartAtMs = 0;

// Stan sesji DX Cluster (Telnet)
bool clusterLoginSent = false;
bool clusterLoginScheduled = false;
unsigned long clusterSendLoginAtMs = 0;
unsigned long lastClusterKeepAliveMs = 0;
unsigned long lastTelnetRxMs = 0;
const unsigned long TELNET_INACTIVITY_RECONNECT_MS = 5UL * 60UL * 1000UL; // 5 minut

// Stan sesji POTA Cluster (Telnet)
bool potaTelnetConnected = false;
bool potaLoginSent = false;
bool potaLoginScheduled = false;
unsigned long potaSendLoginAtMs = 0;
unsigned long lastPotaKeepAliveMs = 0;
unsigned long lastPotaRxMs = 0;
const unsigned long POTA_TELNET_INACTIVITY_RECONNECT_MS = 5UL * 60UL * 1000UL; // 5 minut

// Bufor dla danych Telnet
String telnetBuffer = "";
String pendingTelnetLine = "";
unsigned long pendingTelnetDropped = 0;

// Bufor dla danych POTA Telnet
String potaTelnetBuffer = "";
String pendingPotaLine = "";
unsigned long pendingPotaDropped = 0;

// QRZ cache (żeby nie pytać w kółko o te same znaki)
struct QrzCacheEntry {
  String callsign;
  String grid;
  String country;
  String name;
  String email;
  String qth;
  float lat;
  float lon;
  bool hasLatLon;
  unsigned long fetchedAtMs;
};
const int QRZ_CACHE_SIZE = 20;
QrzCacheEntry qrzCache[QRZ_CACHE_SIZE];

// Forward declarations
void drawUnlisHunter();
void drawPskMap();

// ========== INICJALIZACJA TFT (ESP32-2432S028) ==========
#ifdef ENABLE_TFT_DISPLAY
void setupBacklightPwm() {
#if (TFT_BL_PIN >= 0)
  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, TFT_BL_INVERTED ? LOW : HIGH);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  // Arduino ESP32 core 3.x LEDC API
  ledcAttach(TFT_BL_PIN, BACKLIGHT_PWM_FREQ, BACKLIGHT_PWM_RES_BITS);
#else
  // Arduino ESP32 core 2.x LEDC API
  ledcSetup(BACKLIGHT_PWM_CHANNEL, BACKLIGHT_PWM_FREQ, BACKLIGHT_PWM_RES_BITS);
  ledcAttachPin(TFT_BL_PIN, BACKLIGHT_PWM_CHANNEL);
#endif
#endif
}

void setBacklightPercent(int percent) {
  if (percent < MIN_BACKLIGHT_PERCENT) percent = MIN_BACKLIGHT_PERCENT;
  if (percent > 100) percent = 100;
  backlightPercent = percent;
#if (TFT_BL_PIN >= 0)
  uint32_t dutyMax = (1U << BACKLIGHT_PWM_RES_BITS) - 1U;
  uint32_t duty = (percent * dutyMax) / 100U;
  if (TFT_BL_INVERTED) {
    duty = dutyMax - duty;
  }
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(TFT_BL_PIN, duty);
#else
  ledcWrite(BACKLIGHT_PWM_CHANNEL, duty);
#endif
#endif
}

void applyTouchRotation() {
  Serial.print("*** Applying touch.setRotation(");
  Serial.print(touchRotation);
  Serial.println(") ***");
  touch.setRotation(touchRotation);
}

void applyTftRotation() {
  Serial.print("*** Applying tft.setRotation(");
  Serial.print(tftRotation);
  Serial.println(") ***");
  tft.setRotation(tftRotation);
}

void applyTftInversion() {
  tftInvertColors = sanitizeTftInvertSetting(tftInvertColors);
  Serial.print("*** Applying tft.invertDisplay(");
  Serial.print(tftInvertColors ? "true" : "false");
  Serial.println(") ***");
  tft.invertDisplay(tftInvertColors);
}

void initTFT() {
  Serial.println("=== Inicjalizacja TFT ===");
  Serial.printf("TFT setup info=%s\n", USER_SETUP_INFO);
  Serial.printf("TFT profile=%s SCLK=%d MOSI=%d MISO=%d CS=%d DC=%d RST=%d BL=%d\n",
                "ILI9488",
                TFT_SCLK, TFT_MOSI, TFT_MISO, TFT_CS, TFT_DC, TFT_RST, TFT_BL_PIN);
  
  // Podświetlenie ustawiamy jawnie, żeby łatwo odróżnić problem BL od problemu SPI/drivera.
  setupBacklightPwm();
  setBacklightPercent(backlightPercent);
  Serial.println("TFT begin()...");
  tft.begin();
  Serial.println("TFT begin() OK");
  
  applyTftRotation();
  applyTftInversion();
  // Dla rotacji 1: szerokości 480, wysokości 320
  tft.fillScreen(TFT_WHITE);

  // Inicjalizacja dotyku (XPT2046, osobny SPI)
  touchSPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
  touch.begin(touchSPI);
  applyTouchRotation();
  
  tftInitialized = true;
  inMenu = false;
  menuOption = 0;
  
  Serial.println("=== TFT zainicjalizowany OK ===");
  
  // Wyświetl ekran startowy (boot log)
  drawBootScreen();
  tftBootPrintLine("=== Inicjalizacja TFT ===");
  tftBootPrintLine("TFT begin()...");
  tftBootPrintLine("TFT begin() OK");
  tftBootPrintLine("=== TFT zainicjalizowany OK ===");
}

void drawBootScreen() {
  if (!tftInitialized) {
    return;
  }
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  bootLogY = 2;
  tft.setCursor(2, bootLogY);
}

void tftBootPrintLine(const String &line) {
  if (!tftInitialized) {
    return;
  }
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(1);
  tft.setCursor(2, bootLogY);
  tft.print(line);
  bootLogY += BOOT_LOG_LINE_HEIGHT;
  if (bootLogY > 230) {
    tft.fillScreen(TFT_BLACK);
    bootLogY = 2;
  }
}

void drawWelcomeScreenYellow() {
  if (!tftInitialized) {
    return;
  }
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  const int centerX = 240;
  const int startY = 60;
  const int lineGap = 30;
  String lines[] = {
    "ESP32-HAM-CLOCK",
    "version 1.4",
    "Original: SP3KON",
    "Modified by: SP9TNV",
    "License: MIT",
    "sp3kon@gmail.com",
    "sp9tnv@gmail.com"
  };
  for (int i = 0; i < 6; i++) {
    int textWidth = tft.textWidth(lines[i]);
    int x = centerX - (textWidth / 2);
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(x, startY + i * lineGap);
    tft.print(lines[i]);
  }
}

void drawWelcomeScreenGreen() {
  if (!tftInitialized) {
    return;
  }
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(3);
  const int centerX = 240;
  const int startY = 140;
  String line = "FOLLOW THE PROPAGATION...";
  int textWidth = tft.textWidth(line);
  int x = centerX - (textWidth / 2);
  tft.setCursor(x, startY);
  tft.print(line);
}

// Ekran startowy z grafiką SP9TNV
void drawSplashScreen() {
  if (!tftInitialized) {
    return;
  }
  
  tft.fillScreen(TFT_BLACK);
  
  const int centerX = 240;
  
  // Spróbuj załadować plik BMP z różnych lokalizacji
  String splashPath = "";
  bool bmpLoaded = false;
  if (littleFsReady) {
    if (LittleFS.exists("/icon50/splash.bmp")) {
      splashPath = "/icon50/splash.bmp";
    } else if (LittleFS.exists("/data/splash.bmp")) {
      splashPath = "/data/splash.bmp";
    } else if (LittleFS.exists("/splash.bmp")) {
      splashPath = "/splash.bmp";
    }
    
    if (splashPath != "") {
      Serial.println("[Splash] Loading: " + splashPath);
      // Wyczyść obszar obrazu przed załadowaniem (na wypadek gdyby BMP miał inne wymiary)
      tft.fillRect(0, 0, 480, 320, TFT_BLACK);
      bmpLoaded = drawBmpFromFS(splashPath.c_str(), 0, 0);
      if (bmpLoaded) {
        // BMP loaded successfully, skip fallback graphics
        splashPathLoaded = true;
      } else {
        Serial.println("[Splash] BMP load failed, will draw fallback");
        splashPathLoaded = false;
      }
    }
  }
  
  // Fallback - czarny ekran (bez grafiki)
  if (!littleFsReady || splashPath == "" || !LittleFS.exists(splashPath) || !splashPathLoaded) {
    Serial.println("[Splash] No BMP found, showing black screen");
    tft.fillScreen(TFT_BLACK);
  }
  
  // Tekst "Kliknij aby kontynuować"
  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(centerX - 80, 310);
  tft.print("Kliknij ekran aby kontynuowac...");
}

// Aktualizuj wyÄąâ€şwietlacz z aktualnym adresem IP
void updateTFT_IP() {
  if (!tftInitialized) {
    return;
  }
  
  IPAddress ip;
  String modeStr = "";
  
  // SprawdÄąĹź czy jesteÄąâ€şmy w trybie STA czy AP
  if (wifiConnected && WiFi.status() == WL_CONNECTED) {
    ip = WiFi.localIP();
    modeStr = "WiFi";
  } else if (WiFi.getMode() & WIFI_AP) {
    ip = WiFi.softAPIP();
    modeStr = "AP Mode";
  } else {
    // Brak IP - wyświetl komunikat
    tft.fillRect(10, 40, 300, 20, TFT_WHITE);
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(10, 45);
    tft.print("No connection");
    return;
  }
  
  // WyczyÄąâ€şĂ„â€ˇ obszar IP (dla rotacji 1: szerokoÄąâ€şĂ„â€ˇ 480)
  tft.fillRect(10, 40, 300, 20, TFT_BLACK);
  
  // WyÄąâ€şwietl tryb i IP w jednej linii (oszczędność miejsca)
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(10, 45);
  tft.print(modeStr);
  tft.print(": ");
  
  // Wyświetl IP
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.print(ip.toString());
}

// Aktualizuj wyÄąâ€şwietlacz z tabelą spotów (podobnie jak na stronie WWW)
void updateTFT_Spots() {
  static unsigned long lastTFTPrint = 0;
  static int tftCallCount = 0;
  tftCallCount++;
  
  if (!tftInitialized) {
    return;
  }
  
  // Print co 10 wywołań (dla debugowania)
  unsigned long now = millis();
  if (now - lastTFTPrint > 10000) { // Co 10 sekund
    Serial.print("[TFT] updateTFT_Spots wywoÄąâ€šane ");
    Serial.print(tftCallCount);
    Serial.print(" razy, spotCount=");
    Serial.println(spotCount);
    lastTFTPrint = now;
    tftCallCount = 0;
  }
  
  // WyczyÄąâ€şĂ„â€ˇ obszar tabeli (zostaw nagÄąâ€šÄ‚łwek i IP)
  // Dla rotacji 1 (krajobraz): szerokoÄąâ€şĂ„â€ˇ 480, wysokoÄąâ€şĂ„â€ˇ 320
  // Zostaw miejsce na nagÄąâ€šÄ‚łwek (0-45) i tabelĂ„â„˘ (65-320)
  tft.fillRect(0, 65, 480, 175, TFT_WHITE);
  
  // WyÄąâ€şwietl nagÄąâ€šÄ‚łwek tabeli
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(1);
  
  // Nagłówki kolumn (dla szerokoÄąâ€şci 480px)
  int yPos = 65;
  tft.setCursor(5, yPos);
  tft.print(tr(TR_TIME_SHORT));
  tft.setCursor(55, yPos);
  tft.print(tr(TR_CALL_SHORT));
  tft.setCursor(130, yPos);
  tft.print("MHz");
  tft.setCursor(200, yPos);
  tft.print("Mode");
  tft.setCursor(260, yPos);
  tft.print("km");
  
  yPos += 15;
  
  // Wyświetl maksymalnie 10 spotÄ‚łw na wyÄąâ€şwietlaczu TFT (ÄąÄ˝eby zmieścić się na ekranie)
  int maxDisplaySpots = min(spotCount, 10);
  
  for (int i = 0; i < maxDisplaySpots; i++) {
    if (yPos >= 230) break; // Nie wchodź poza ekran
    
    // Znak wywoławczy - rozmiar 2 (większy)
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(5, yPos);
    String callStr = spots[i].callsign;
    callStr.toUpperCase();
    if (callStr.length() > 8) callStr = callStr.substring(0, 8);
    tft.print(callStr);
    
    // Częstotliwość (MHz) - rozmiar 2
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(140, yPos);
    float freqMHz = spots[i].frequency / 1000.0;
    tft.print(freqMHz, 3);
    
    // Mode - rozmiar 2
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(260, yPos);
    String modeStr = spots[i].mode;
    if (modeStr.length() > 4) modeStr = modeStr.substring(0, 4);
    tft.print(modeStr);
    
    // Czas i kraj - rozmiar 1
    tft.setTextSize(1);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setCursor(350, yPos + 5);
    String timeStr = spots[i].time;
    if (timeStr.length() > 5) timeStr = timeStr.substring(0, 5);
    tft.print(timeStr);
    
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.setCursor(400, yPos + 5);
    String countryText = formatDistanceOrCountry(spots[i], 6);
    tft.print(countryText);
    
    yPos += 20;
  }
  
  // Jeśli brak spotów, wyświetl komunikat
  if (spotCount == 0) {
    tft.setTextColor(TFT_RED, TFT_WHITE);
    tft.setTextSize(1);
    tft.setCursor(10, 90);
    tft.println(tr(TR_WAITING_SPOTS));
  }
}

// ========== SYSTEM MENU I EKRANÄ‚â€śW ==========

extern TaskHandle_t uiTaskHandle;
static void requestUiScreenRedraw(uint8_t pendingScreenId);

// GÄąâ€šÄ‚łwna funkcja rysujĂ„â€¦ca ekrany
void drawScreen(ScreenType screenNum) {
  if (!tftInitialized) {
    return;
  }

  if (uiTaskHandle != nullptr && xTaskGetCurrentTaskHandle() != uiTaskHandle) {
    requestUiScreenRedraw((uint8_t)screenNum);
    return;
  }

  static ScreenType lastDrawnScreen = SCREEN_OFF;
  if (screenNum != lastDrawnScreen) {
    if (isTableFooterScreen(screenNum)) {
      tableNavFooterVisibleUntilMs = millis() + TABLE_NAV_FOOTER_VISIBLE_MS;
    }
    if (screenNum == SCREEN_APRS_RADAR) {
      // On first enter to radar screen, show zoom/info hints for a short time.
      screen6RadarHintUntilMs = millis() + SCREEN6_RADAR_HINT_DURATION_MS;
    }
    lastDrawnScreen = screenNum;
  }

  if (aprsAlertScreenActive) {
    return;
  }
  
  tft.fillScreen(TFT_BLACK);
  
  switch (screenNum) {
    case SCREEN_HAM_CLOCK:
      drawHamClock();
      break;
    case SCREEN_DX_CLUSTER:
      drawDxCluster();
      break;
    case SCREEN_SUN_SPOTS:
      drawSunSpots();
      break;
    case SCREEN_BAND_INFO:
      drawBandInfo();
      break;
    case SCREEN_WEATHER_DSP:
      drawWeather();
      break;
    case SCREEN_WEATHER_FORECAST:
      drawWeatherForecast();
      break;
    case SCREEN_APRS_IS:
      drawAprsIs();
      break;
    case SCREEN_APRS_RADAR:
      drawAprsRadar();
      break;
    case SCREEN_POTA_CLUSTER:
      drawPotaCluster();
      break;
    case SCREEN_HAMALERT_CLUSTER:
      drawHamalertCluster();
      break;
    case SCREEN_MATRIX_CLOCK:
      drawMatrixClock();
      break;
    case SCREEN_UNLIS_HUNTER:
      drawUnlisHunter();
      break;
    case SCREEN_PSK_MAP:
      drawPskMap();
      break;
    case SCREEN_ISS_PASS_TRACKING:
      drawIssPassTracking();
      break;
    case SCREEN_OFF:
    default:
      drawHamClock();
      break;
  }
}

// Ekran 1: Startowy - Info
uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

uint16_t colorWheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85) {
    return rgb565(255 - pos * 3, 0, pos * 3);
  }
  if (pos < 170) {
    pos -= 85;
    return rgb565(0, pos * 3, 255 - pos * 3);
  }
  pos -= 170;
  return rgb565(pos * 3, 255 - pos * 3, 0);
}

void applyMenuThemeFromHue() {
  menuThemeColor = colorWheel(menuThemeHue);
}

// Definicja koloru "Radio Orange" - dynamiczny motyw
#define TFT_RADIO_ORANGE menuThemeColor

// Kolory i tÄąâ€šo "Matrix" dla ekranu 10
#define MATRIX_DARKGREEN 0x0480
#define MATRIX_BRIGHTGREEN 0x07E0
#define MATRIX_WHITEGREEN 0xAFE5

uint16_t lerpColor565(uint16_t from, uint16_t to, uint8_t t) {
  uint8_t r1 = (from >> 11) & 0x1F;
  uint8_t g1 = (from >> 5) & 0x3F;
  uint8_t b1 = from & 0x1F;
  uint8_t r2 = (to >> 11) & 0x1F;
  uint8_t g2 = (to >> 5) & 0x3F;
  uint8_t b2 = to & 0x1F;
  uint8_t r = r1 + ((r2 - r1) * t) / 255;
  uint8_t g = g1 + ((g2 - g1) * t) / 255;
  uint8_t b = b1 + ((b2 - b1) * t) / 255;
  return (r << 11) | (g << 5) | b;
}

struct MatrixDrop {
  int x;
  int y;
  int speed;
  int len;
  char headChar;
  bool introParticipates;
  int introStartY;
  int introDelayMs;
  float introSpeedPxPerMs;
  bool introActive;
};

const int SCREEN10_WIDTH = 480;
const int SCREEN10_HEIGHT = 320;
const int MATRIX_COL_SPACING = 7;
const int numDrops = SCREEN10_WIDTH / MATRIX_COL_SPACING; // Tyle kolumn, ile się mieści
const char MATRIX_HEAD_TEXT[] = "ESP32   HAM   CLOCK   by   SP3KON";
const int MATRIX_HEAD_TEXT_LEN = (int)(sizeof(MATRIX_HEAD_TEXT) - 1);
MatrixDrop drops[numDrops];
bool matrixInitialized = false;

static inline char getMatrixHeadCharForColumn(int columnIdx) {
  if (MATRIX_HEAD_TEXT_LEN <= 0) {
    return ' ';
  }
  if (numDrops <= MATRIX_HEAD_TEXT_LEN) {
    return MATRIX_HEAD_TEXT[columnIdx];
  }

  const int freeCols = numDrops - MATRIX_HEAD_TEXT_LEN;
  const int leftPad = freeCols / 2;
  const int rightStart = leftPad + MATRIX_HEAD_TEXT_LEN;

  if (columnIdx < leftPad || columnIdx >= rightStart) {
    return ' ';
  }

  return MATRIX_HEAD_TEXT[columnIdx - leftPad];
}

const int SCREEN10_HEADER_H = 0;
const int SCREEN10_BODY_TOP = 0;  // Pełny ekran matrix
const int SCREEN10_BODY_BOTTOM = 320;
const int SCREEN10_BODY_H = SCREEN10_BODY_BOTTOM - SCREEN10_BODY_TOP;
const int CLOCK_TEXT_SIZE = 6;
const int CLOCK_CHAR_W = 6;
const int CLOCK_CHAR_H = 8;
const unsigned long MATRIX_UPDATE_INTERVAL_MS = 120;
const unsigned long MATRIX_INTRO_ALIGN_MS = 3400UL;

unsigned long lastScreen10UpdateMs = 0;
unsigned long lastMatrixUpdateMs = 0;
bool screen10NeedsRedraw = true;
bool matrixIntroActive = false;
unsigned long matrixIntroStartMs = 0;
int clockMaskX = 0;
int clockMaskY = 0;
int clockMaskW = 0;
int clockMaskH = 0;
String lastClockText = "";
bool clockNeedsRedraw = true;

// ========== WYGASZACZ Z PORUSZAJĄCYM SIĘ ZEGAREM ==========
float bounceClockX = 240;  // Pozycja X zegara (środek ekranu)
float bounceClockY = 160;  // Pozycja Y zegara (środek ekranu)
float bounceClockVX = 2.5; // Prędkość X (piksele na klatkę)
float bounceClockVY = 2.0; // Prędkość Y
const int BOUNCE_CLOCK_TEXT_SIZE = 4;  // Rozmiar tekstu zegara
const int BOUNCE_CLOCK_CHAR_W = 6 * BOUNCE_CLOCK_TEXT_SIZE;  // Szerokość znaku
const int BOUNCE_CLOCK_CHAR_H = 8 * BOUNCE_CLOCK_TEXT_SIZE;   // Wysokość znaku
int bounceClockW = 0;  // Szerokość tekstu zegara (obliczana dynamicznie)
int bounceClockH = BOUNCE_CLOCK_CHAR_H;
uint16_t bounceClockColor = TFT_RADIO_ORANGE;
unsigned long lastBounceClockUpdateMs = 0;
const unsigned long BOUNCE_CLOCK_UPDATE_MS = 16; // ~60 FPS
String lastBounceClockText = "";

// Sprite dla zegara Matrix - eliminuje migotanie
TFT_eSprite clockSprite = TFT_eSprite(&tft);

static void resetMatrixDropRandom(int i) {
  drops[i].y = random(-SCREEN10_BODY_H, 0);
  drops[i].speed = random(5, 15);
  drops[i].len = random(4, 17);
  drops[i].headChar = (char)random(33, 126);
  drops[i].introActive = false;
}

static void prepareMatrixIntro() {
  const int charStep = 8;
  const int alignHeadY = SCREEN10_BODY_TOP + (SCREEN10_BODY_H / 2);

  for (int i = 0; i < numDrops; i++) {
    drops[i].headChar = getMatrixHeadCharForColumn(i);
    drops[i].introParticipates = (drops[i].headChar != ' ');
    drops[i].introStartY = random(-140, -40);
    drops[i].introDelayMs = random(0, 1900);
    drops[i].introActive = drops[i].introParticipates;

    int targetDropY = (alignHeadY - SCREEN10_BODY_TOP) - ((drops[i].len - 1) * charStep);
    int travelMs = (int)MATRIX_INTRO_ALIGN_MS - drops[i].introDelayMs;
    if (travelMs < 250) {
      travelMs = 250;
    }

    drops[i].introSpeedPxPerMs = (float)(targetDropY - drops[i].introStartY) / (float)travelMs;
    if (drops[i].introSpeedPxPerMs < 0.03f) {
      drops[i].introSpeedPxPerMs = 0.03f;
    }
  }

  matrixIntroActive = true;
  matrixIntroStartMs = millis();
}

String getUtcTimeString() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1)) {
    char timeBuffer[9];
    strftime(timeBuffer, 9, "%H:%M:%S", &timeinfo);
    return String(timeBuffer);
  }
  return "--:--:--";
}

// Sprawdza czy obowiązuje czas letni w Europie (ostatnia niedziela marca - ostatnia niedziela października)
bool isEuropeanDST(struct tm *timeinfo) {
  int month = timeinfo->tm_mon + 1;  // 1-12
  int day = timeinfo->tm_mday;       // 1-31
  int wday = timeinfo->tm_wday;      // 0=niedziela
  
  // Styczeń, luty, listopad, grudzień - zima
  if (month < 3 || month > 10) return false;
  // Kwiecień-wrzesień - lato
  if (month > 3 && month < 10) return true;
  
  // Marzec - sprawdź ostatnią niedzielę
  if (month == 3) {
    int lastSunday = 31 - ((31 - day + wday) % 7);
    return day > lastSunday || (day == lastSunday && timeinfo->tm_hour >= 1);
  }
  
  // Październik - sprawdź ostatnią niedzielę
  if (month == 10) {
    int lastSunday = 31 - ((31 - day + wday) % 7);
    return day < lastSunday || (day == lastSunday && timeinfo->tm_hour < 1);
  }
  
  return false;
}

bool getTimeWithTimezone(struct tm *outTm) {
  time_t now = time(nullptr);
  if (now < 100000) {
    return false;
  }
  // Pobierz czas UTC aby sprawdzić DST
  struct tm utcTm;
  gmtime_r(&now, &utcTm);
  int dstOffset = isEuropeanDST(&utcTm) ? 1 : 0;  // +1h w czasie letnim
  now += (time_t)(timezoneHours + dstOffset) * 3600;
  gmtime_r(&now, outTm);
  return true;
}

String getTimezoneTimeString(const char *fmt, size_t bufSize) {
  struct tm timeinfo;
  if (!getTimeWithTimezone(&timeinfo)) {
    return "--:--";
  }
  char timeBuffer[16];
  strftime(timeBuffer, bufSize, fmt, &timeinfo);
  return String(timeBuffer);
}

String formatAprsTimeWithTimezone(String timeStr) {
  if (timeStr.endsWith("Z")) {
    timeStr.remove(timeStr.length() - 1);
  }
  if (timeStr.length() < 4) {
    return timeStr;
  }
  int hour = timeStr.substring(0, 2).toInt();
  int minute = timeStr.substring(2, 4).toInt();
  int hourLocal = hour + timezoneHours;
  while (hourLocal < 0) hourLocal += 24;
  while (hourLocal >= 24) hourLocal -= 24;
  String hh = (hourLocal < 10 ? "0" : "") + String(hourLocal);
  String mm = (minute < 10 ? "0" : "") + String(minute);
  return hh + ":" + mm;
}

String sanitizePolishToAscii(const String &input) {
  String out;
  out.reserve(input.length());
  const char *s = input.c_str();
  size_t len = input.length();
  for (size_t i = 0; i < len; i++) {
    uint8_t c = (uint8_t)s[i];
    if (c < 0x80) {
      out += (char)c;
      continue;
    }
    if (i + 1 >= len) {
      continue;
    }
    uint8_t c2 = (uint8_t)s[i + 1];
    // UTF-8 Polish letters
    if (c == 0xC4 && c2 == 0x84) { out += 'a'; i++; continue; } // Ą
    if (c == 0xC4 && c2 == 0x85) { out += 'a'; i++; continue; } // Ą
    if (c == 0xC4 && c2 == 0x86) { out += 'c'; i++; continue; } // Ć
    if (c == 0xC4 && c2 == 0x87) { out += 'c'; i++; continue; } // Ć
    if (c == 0xC4 && c2 == 0x98) { out += 'e'; i++; continue; } // Ę
    if (c == 0xC4 && c2 == 0x99) { out += 'e'; i++; continue; } // Ę
    if (c == 0xC5 && c2 == 0x81) { out += 'l'; i++; continue; } // Ł
    if (c == 0xC5 && c2 == 0x82) { out += 'l'; i++; continue; } // ł
    if (c == 0xC5 && c2 == 0x83) { out += 'n'; i++; continue; } // Ń
    if (c == 0xC5 && c2 == 0x84) { out += 'n'; i++; continue; } // ń
    if (c == 0xC3 && c2 == 0x93) { out += 'o'; i++; continue; } // Ó
    if (c == 0xC3 && c2 == 0xB3) { out += 'o'; i++; continue; } // ó
    if (c == 0xC5 && c2 == 0x9A) { out += 'S'; i++; continue; } // Ś
    if (c == 0xC5 && c2 == 0x9B) { out += 's'; i++; continue; } // ś
    if (c == 0xC5 && c2 == 0xB9) { out += 'z'; i++; continue; } // Ź
    if (c == 0xC5 && c2 == 0xBA) { out += 'z'; i++; continue; } // ź
    if (c == 0xC5 && c2 == 0xBB) { out += 'z'; i++; continue; } // Ż
    if (c == 0xC5 && c2 == 0xBC) { out += 'z'; i++; continue; } // ż
    // Unknown multibyte - skip
  }
  return out;
}

String getPolishDateStringFull() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1)) {
    return "Brak daty";
  }

  static const char* weekdaysFull[] = {
    "Niedziela", "Poniedzialek", "Wtorek",
    "Sroda", "Czwartek", "Piatek", "Sobota"
  };

  String dateBuf = String(timeinfo.tm_mday < 10 ? "0" : "") + String(timeinfo.tm_mday) +
                   "." +
                   String((timeinfo.tm_mon + 1) < 10 ? "0" : "") + String(timeinfo.tm_mon + 1) +
                   "." + String(1900 + timeinfo.tm_year);

  String full = String(weekdaysFull[timeinfo.tm_wday]) + " " + dateBuf;
  return full;  // Zwracamy oryginalny tekst z polskimi znakami
}

String getEnglishDateStringFull() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1)) {
    return "No date";
  }

  static const char* weekdaysFullEn[] = {
    "Sunday", "Monday", "Tuesday", "Wednesday",
    "Thursday", "Friday", "Saturday"
  };

  String dateBuf = String(timeinfo.tm_mday < 10 ? "0" : "") + String(timeinfo.tm_mday) +
                   "." +
                   String((timeinfo.tm_mon + 1) < 10 ? "0" : "") + String(timeinfo.tm_mon + 1) +
                   "." + String(1900 + timeinfo.tm_year);

  String full = String(weekdaysFullEn[timeinfo.tm_wday]) + " " + dateBuf;
  return full;
}

String getPolishDateStringFullWithTimezone() {
  struct tm timeinfo;
  if (!getTimeWithTimezone(&timeinfo)) {
    return "Brak daty";
  }

  static const char* weekdaysFull[] = {
    "Niedziela", "Poniedzialek", "Wtorek",
    "Sroda", "Czwartek", "Piatek", "Sobota"
  };

  String dateBuf = String(timeinfo.tm_mday < 10 ? "0" : "") + String(timeinfo.tm_mday) +
                   "." +
                   String((timeinfo.tm_mon + 1) < 10 ? "0" : "") + String(timeinfo.tm_mon + 1) +
                   "." + String(1900 + timeinfo.tm_year);

  String full = String(weekdaysFull[timeinfo.tm_wday]) + " " + dateBuf;
  return full;  // Zwracamy oryginalny tekst z polskimi znakami
}

String getEnglishDateStringFullWithTimezone() {
  struct tm timeinfo;
  if (!getTimeWithTimezone(&timeinfo)) {
    return "No date";
  }

  static const char* weekdaysFullEn[] = {
    "Sunday", "Monday", "Tuesday", "Wednesday",
    "Thursday", "Friday", "Saturday"
  };

  String dateBuf = String(timeinfo.tm_mday < 10 ? "0" : "") + String(timeinfo.tm_mday) +
                   "." +
                   String((timeinfo.tm_mon + 1) < 10 ? "0" : "") + String(timeinfo.tm_mon + 1) +
                   "." + String(1900 + timeinfo.tm_year);

  String full = String(weekdaysFullEn[timeinfo.tm_wday]) + " " + dateBuf;
  return full;
}

// Funkcja zwracająca imieniny dla danego dnia (polskie imieniny)
String getPolishNameDay() {
  struct tm timeinfo;
  if (!getTimeWithTimezone(&timeinfo)) {
    return "";
  }
  
  // Tablica imienin dla każdego dnia roku (miesiąc 0-11, dzień 1-31)
  // Styczeń
  static const char* nameDays[12][31] = {
    {"Mieszko, Mieczysław", "Izydor, Grzegorz", "Danuta, Genowefa", "Angelika, Aniela", "Edward, Szymon",
     "Kacper, Melchior, Baltazar", "Lucjan, Julian", "Seweryn, Marcin", "Marcelina, Julianna", "Danuta, Piotr",
     "Agata, Małgorzata", "Arkadiusz, Benedykt", "Bogumiła, Wojciech", "Feliks, Nina", "Paweł, Arnold",
     "Marcel, Włodzimierz", "Antoni, Jan", "Małgorzata, Piotr", "Henryk, Marta", "Fabian, Sebastian",
     "Agnieszka, Jarosław", "Anastazy, Wincenty", "Ildefons, Rajmund", "Felicja, Tymoteusz", "Paweł, Michał",
     "Paula, Wanda", "Aniela, Jerzy", "Walery, Karol", "Franciszek, Józef", "Maciej, Martyna",
     "Joanna, Marceli"
    },
    // Luty
    {"Brygida, Ignacy", "Maria, Mirosław", "Błażej, Oskar", "Andrzej, Józef", "Agata, Jakub",
     "Dorota, Paweł", "Ryszard, Teodor", "Hieronim, Sebastian", "Apolonia, Cyryl", "Elwira, Scholastyka",
     "Lucjan, Olgierd", "Eulalia, Radosław", "Grzegorz, Leszek", "Krystyna, Liliana", "Jowita, Faustyn",
     "Danuta, Juliana", "Aleksy, Zbigniew", "Szymon, Konstancja", "Arnold, Konrad", "Leon, Ludomir",
     "Eleonora, Feliks", "Marta, Małgorzata", "Roman, Damian", "Maciej, Marek", "Cezary, Donat",
     "Michał, Tytus", "Aleksander, Gabriel", "Antoni, Romuald", "Baltazar, Kochany", "Marta, Geralda"
    },
    // Marzec
    {"Albina, Antonina", "Helena, Halszka", "Tomasz, Maryna", "Lucja, Kazimierz", "Adrian, Adriana",
     "Róża, Eugeniusz", "Tomasz, Felicyta", "Beatrycze, Wincenty", "Franciszka, Brunon", "Cyprian, Aleksander",
     "Ludwik, Konstantyn", "Grzegorz, Alojzy", "Bożena, Patrycja", "Matylda, Lazarz", "Longin, Klemens",
     "Izabela, Oktawia", "Patryk, Zbigniew", "Cyryl, Edward", "Józef, Bogdan", "Klaudia, Eufemia",
     "Lubomir, Benedykt", "Katarzyna, Bogusław", "Pelagia, Oktawian", "Gabriel, Marek", "Mariola, Więnczysław",
     "Teodor, Olga", "Ernest, Lidia", "Aniela, Sykstus", "Wiktoryna, Jan", "Amelia, Dobromierz",
     "Beniamin, Leon"
    },
    // Kwiecień
    {"Grażyna, Teodor", "Władysław, Franciszek", "Ryszard, Pankracy", "Izydor, Wojciech", "Katarzyna, Irena",
     "Izabela, Wilhelm", "Rufin, Celestyn", "Juliusz, Dionizy", "Maria, Dymitr", "Michał, Makary",
     "Filip, Leon", "Juliusz, Lubosław", "Przemysław, Hermenegild", "Berenika, Tiburcy", "Wiktoryn, Anastazja",
     "Ksenia, Erwin", "Aniceta, Robert", "Alicja, Bogusław", "Waleriana, Tytus", "Czesław, Agnieszka",
     "Anselm, Jarosław", "Kaja, Leon", "Jerzy, Wojciech", "Grzegorz, Paweł", "Erwina, Marek",
     "Marzena, Klaudiusz", "Zygmunt, Florian", "Piotr, Waleria", "Rita, Bogna", "Katarzyna, Marian",
     "Lidia, Trzebomysł"
    },
    // Maj
    {"Józef, Jeremiasz", "Zygmunt, Anatolia", "Maria, Mariola", "Monika, Florian", "Włodzimierz, Iryda",
     "Judyta, Filip", "Benedykt, Gizela", "Stanisław, Lizbona",     "Łukasz, Bożena", "Antonin, Izydor",
     "Iga, Maja", "Joanna, Pankracy", "Glora, Roberta", "Maciej, Bonifacy", "Zofia, Izydora",
     "Jędrzej, Jędrzej", "Brunon, Sławomir", "Aleksandra, Eryk", "Iwa, Piotr", "Bernard, Aleksandra",
     "Jerzy, Jan", "Wiktoria, Julia", "Emilia, Michał", "Joanna, Zuzanna", "Grzegorz, Małgorzata",
     "Filip, Paulina", "Augustyn, Julian", "Jaromir, Justyna", "Magdalena, Zdisława", "Ferdynand, Karolina",
     "Aniela, Petronela"
    },
    // Czerwiec
    {"Justyn, Konrad", "Erazm, Marzenna", "Leszek, Tamara", "Katarzyna, Franciszek", "Walter, Bonifacy",
     "Natalia, Saba", "Antoni, Robert", "Maksym, Seweryn", "Feliks, Pelagia", "Bogdan, Małgorzata",
     "Barnaba, Radomił", "Janina, Jan", "Antoni, Lucylla", "Eliza, Walenty", "Witold, Jolanta",
     "Alina, Benon", "Albert, Ignacy", "Marek, Elżbieta", "Gerwazy, Protazy", "Dina, Bogna",
     "Alicja, Alojzy", "Paulina, Tomasz", "Wanda, Zenon", "Jan, Danuta", "Władysław, Dorota",
     "Piotr, Paweł", "Maryla, Władysława", "Leon, Ireneusz", "Piotr, Paweł", "Emilia, Lucyna",
     "Oktawia, Oktawian"
    },
    // Lipiec
    {"Halina, Marian", "Jagoda, Maria", "Anatol, Tomasz", "Elżbieta, Odon", "Antoni, Maria",
     "Dominika, Gotard", "Cyryl, Metody", "Edgar, Elżbieta", "Łucjan, Weronika", "Sylwia, Witalis",
     "Olga, Kalina", "Jan, Brunon", "Henryk, Kinga", "Bonawentura, Kamil", "Dawid, Henryk",
     "Eustachy, Maria", "Aneta, Bogdan", "Emil, Erwin", "Wincenty, Wodzisław", "Czesław, Eliasz",
     "Piotr, Wiktor", "Magdalena, Władysława", "Ilona, Stefan", "Kinga, Krystyna", "Walentyna, Krzysztof",
     "Anna, Mirosława", "Aurelia, Malwina", "Lilia, Innocenty", "Olaf, Marta", "Julita, Ludmiła",
     "Ignacy, Lubomir"
    },
    // Sierpień
    {"Nadzieja, Piotr", "Karina, Gustaw", "Lidia, Nikodem", "Dominik, Protus", "Emil, Karol",
     "Sławomir, Jan", "Kajetan, Albert", "Cyprian, Emiliana", "Roman, Ryszard", "Borys, Wawrzyniec",
     "Klara, Zuzanna", "Lech, Euzebiusz", "Hildegarda, Diana", "Alfred, Euzebiusz", "Maria, Napoleon",
     "Roch, Stefan", "Anita, Eliza", "Klara, Helena",     "Jan, Bolesław", "Bernard, Samuel",
     "Joanna, Franciszek", "Maria, Cezary", "Róża, Apolinary", "Jerzy, Bartosz", "Luiza, Ludwik",
     "Maria, Zefiryn", "Józef, Klaudiusz", "Patrycja, Wyszomir", "Beata, Jan", "Róża, Szczęsny",
     "Ramona, Rajmund"
    },
    // Wrzesień
    {"Ida, Bronisław", "Stefan, Wilhelm", "Grzegorz, Izabela", "Ida, Julian", "Dorota, Wawrzyniec",
     "Beata, Eugeniusz", "Regina, Melchior", "Mariam, Adriana", "Piotr, Sergiusz", "Elżbieta, Gordian",
     "Jacek, Protus", "Gwido, Maria", "Eugenia, Aureliusz", "Bernard, Roksana", "Albin, Nikodem",
     "Edyta, Kornel", "Franciszek, Hildegarda", "Irma, Stanisław", "Jan, Konstancja", "Filipina, Eustachy",
     "Jonasz, Mateusz", "Tomasz, Maurycy", "Tekla, Bogusław", "Gerard, Herman", "Aurelia, Ladysław",
     "Władysław, Ewa", "Damian, Kosma", "Wacław, Laurencjusz", "Michał, Rafał, Gabriela", "Hieronim, Remigiusz"
    },
    // Październik
    {"Danuta, Remigiusz", "Teofil, Dionizy", "Teresa, Heliodor", "Edwin, Franciszek", "Igor, Placyd",
     "Artur, Brunon", "Maria, Marek", "Pelagia, Brygida", "Arnold, Dionizy", "Paulina, Franciszek",
     "Emil, Aldona", "Maksymilian, Eustachy", "Edward, Gerald", "Łukasz, Manfred", "Teresa, Jadwiga",
     "Gall, Florian", "Ignacy, Rudolfa", "Łukasz, Julian", "Ziemowit, Jadwiga", "Irena, Jan",
     "Urszula, Hilaria", "Filip, Marek", "Marlena, Seweryn", "Rafał, Marcin", "Beata, Daria",
     "Łucjan, Ewaryst", "Iwona, Sabina", "Szymon, Juda", "Euzebia, Wioletta", "Zenobia, Przemysław",
     "Urbain, Saturnin"
    },
    // Listopad
    {"Seweryn, Wiktoryna", "Bohdan, Bożydar", "Sylwia, Marcin", "Karol, Wiktor", "Elżbieta, Zachariasz",
     "Feliks, Leonard", "Antoni, Ernest", "Seweryn, Bogdan", "Aleksander, Ludwik", "Lena, Marcin",
     "Szczęsny, Leon", "Marcin, Witold", "Mikołaj, Stanisław", "Emilian, Wawrzyniec", "Albert, Leopold",
     "Gertruda, Edmund", "Grzegorz, Salomea", "Klaudyna, Karolina", "Elżbieta, Seweryn", "Anatol, Rafał",
     "Albertina, Janusz", "Cecylia, Felicyta", "Adela, Klemens", "Flora, Emma", "Katarzyna, Erazm",
     "Delfina, Sylwester", "Wirgiliusz, Walery", "Jakub, Zdzisław", "Fryderyk, Zygmunt", "Andrzej, Seweryn",
     ""
    },
    // Grudzień
    {"Eligiusz, Natalia", "Bibiana, Balbina", "Franciszek, Ksawery", "Barbara, Krystian", "Norbert, Wawrzyniec",
     "Jadwiga, Jan", "Marcin, Ambroży", "Maria, Świętosława", "Wiesław, Leokadia", "Daniel, Bogdan",
     "Damaszek, Waldemar", "Dagmara, Aleksandra", "Łucja, Otylia", "Alfred, Izydor", "Nina, Celina",
     "Albina, Olimpia", "Lazarz, Florian", "Gracjan, Bogusław", "Michał, Dariusz", "Bogumiła, Dominik",
     "Tomisław, Honorata", "Zenon, Honorata", "Wiktor, Seweryn", "Ewa, Adam", "Anastazja, Eugenia",
     "Dionizy, Szczepan", "Jan, Zenobiusz", "Teofila, Godzisław", "Marta, Tomasz", "Rainer, Eugeniusz",
     "Sylwester, Melania"
    }
  };
  
  int month = timeinfo.tm_mon;  // 0-11
  int day = timeinfo.tm_mday - 1;  // 0-30
  
  if (month >= 0 && month < 12 && day >= 0 && day < 31) {
    String nameDay = nameDays[month][day];
    if (nameDay.length() > 0) {
      return String("Imieniny: ") + nameDay;
    }
  }
  return "";
}

// Rysuje linię daty/tygodnia wyśrodkowaną - zaktualizowana dla 480x320
void drawDateLine(const String &dateText) {
  const int screenW = 480;
  const int timeY = 130;  // Taka sama jak w drawHamClock
  const int dateY = timeY + 65;  // Taka sama jak w drawHamClock

  // Czyść obszar daty (szeroki prostokąt na całej szerokości)
  tft.fillRect(30, dateY - 2, 420, 24, TFT_BLACK);
  
  // Rysuj datę wyśrodkowaną - POWIĘKSZONA (TextSize 3 jak w drawHamClock)
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE);
  int dateWidth = dateText.length() * 18; // ~18px na znak przy TextSize 3
  int dateX = (screenW - dateWidth) / 2;
  tft.setCursor(dateX > 10 ? dateX : 10, dateY);
  tft.print(dateText);
}

// Funkcja rysująca paski sygnału WiFi (belki jak w telefonie)
// x, y - pozycja lewego górnego rogu
// strength - siła sygnału 0-4 (0=brak, 4=max)
void drawWifiSignalBars(int x, int y, int strength = -1) {
  // Automatyczna detekcja siły sygnału jeśli nie podano
  if (strength < 0) {
    if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
      strength = 0;
    } else {
      int rssi = WiFi.RSSI();
      if (rssi > -50) strength = 4;
      else if (rssi > -60) strength = 3;
      else if (rssi > -70) strength = 2;
      else if (rssi > -80) strength = 1;
      else strength = 0;
    }
  }
  
  // Kolory dla belek WiFi - gradient od czerwonego do zielonego
  uint16_t bar1Color = TFT_RED;       // Belka 1 (słaby sygnał)
  uint16_t bar2Color = TFT_ORANGE;    // Belka 2
  uint16_t bar3Color = TFT_YELLOW;    // Belka 3
  uint16_t bar4Color = TFT_GREEN;     // Belka 4 (mocny sygnał)
  uint16_t noSignalColor = TFT_RED;   // Brak zasięgu
  uint16_t emptyColor = TFT_DARKGREY; // Puste belki
  
  // 4 belki o rosnącej wysokości w różnych kolorach (gradient)
  // Belka 1 (najniższa) - CZERWONA
  if (strength >= 1) {
    tft.fillRect(x, y + 12, 6, 4, bar1Color);
  } else if (strength == 0) {
    tft.fillRect(x, y + 12, 6, 4, noSignalColor);
  } else {
    tft.drawRect(x, y + 12, 6, 4, emptyColor);
  }
  
  // Belka 2 - POMARAŃCZOWA
  if (strength >= 2) {
    tft.fillRect(x + 8, y + 8, 6, 8, bar2Color);
  } else if (strength == 0) {
    tft.fillRect(x + 8, y + 8, 6, 8, noSignalColor);
  } else {
    tft.drawRect(x + 8, y + 8, 6, 8, emptyColor);
  }
  
  // Belka 3 - ŻÓŁTA
  if (strength >= 3) {
    tft.fillRect(x + 16, y + 4, 6, 12, bar3Color);
  } else if (strength == 0) {
    tft.fillRect(x + 16, y + 4, 6, 12, noSignalColor);
  } else {
    tft.drawRect(x + 16, y + 4, 6, 12, emptyColor);
  }
  
  // Belka 4 (najwyższa) - ZIELONA
  if (strength >= 4) {
    tft.fillRect(x + 24, y, 6, 16, bar4Color);
  } else if (strength == 0) {
    tft.fillRect(x + 24, y, 6, 16, noSignalColor);
  } else {
    tft.drawRect(x + 24, y, 6, 16, emptyColor);
  }
}

void updateScreen1HeaderClock() {
  if (!tftInitialized || currentScreen != SCREEN_HAM_CLOCK || inMenu) {
    return;
  }

  const int screenW = 480;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1)) {
    return;
  }

  static char lastTimeBuffer[6] = "";
  char timeBuffer[6];
  strftime(timeBuffer, 6, "%H:%M", &timeinfo);
  
  // Aktualizuj tylko jeśli czas się zmienił
  if (strcmp(timeBuffer, lastTimeBuffer) == 0) {
    return;
  }
  strcpy(lastTimeBuffer, timeBuffer);
  
  // ZEGAR PO LEWEJ STRONIE - odświeżanie co sekundę
  String timeStr = String(timeBuffer);
  tft.setTextSize(2);
  int timeWidth = tft.textWidth(timeStr);
  int timeX = 10; // Lewa strona nagłówka
  int timeY = 12;
  
  // Wyczyść tło pod zegarem
  tft.fillRect(timeX - 2, timeY - 2, timeWidth + 4, 20, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(timeX, timeY);
  tft.print(timeStr);
}

void updateDxClusterClock() {
  // ZEGAR WYŁĄCZONY - usunięty z nagłówka
  return;
}

void updateWeatherClock() {
  // ZEGAR WYŁĄCZONY - usunięty z nagłówka
  return;
}

void updateWeatherForecastClock() {
  // ZEGAR WYŁĄCZONY - usunięty z nagłówka
  return;
}

void updateBandInfoClock() {
  // ZEGAR WYŁĄCZONY - usunięty z nagłówka
  return;
}

void updateHamalertClock() {
  // ZEGAR WYŁĄCZONY - usunięty z nagłówka
  return;
}

void updatePotaClusterClock() {
  // ZEGAR WYŁĄCZONY - usunięty z nagłówka
  return;
}

void updateScreen1Header() {
  if (!tftInitialized || currentScreen != SCREEN_HAM_CLOCK || inMenu) {
    return;
  }

  if (!screen1HeaderNeedsRedraw) {
    return;
  }

  const int headerY = 0;
  const int headerH = 40;
  const int screenW = 480;
  
  // 1. TŁO I RAMKA LOGO
  tft.fillRect(0, headerY, screenW, headerH, TFT_RADIO_ORANGE);
  // Ciemniejsza krawędź na dole dla efektu 3D
  tft.drawFastHLine(0, headerH - 1, screenW, 0x8410); 

  // 2. FORMATOWANIE "LOGO" - "ESP32" w czarnej obwódce, "-HAM-CLOCK" normalnie
  String logoPart1 = "ESP32";
  String logoPart2 = "-HAM-CLOCK";
  tft.setTextSize(2);
  int charW = 12; // ~12px na znak przy TextSize 2
  int textH = 16; // ~16px wysokość tekstu przy TextSize 2
  
  // Szerokości części napisu
  int part1W = logoPart1.length() * charW;
  int part2W = logoPart2.length() * charW;
  int totalW = part1W + part2W;
  int startX = (screenW - totalW) / 2;
  int textY = 12;
  int paddingX = 6;  // Odstęp poziomy w obwódce
  int paddingY = 3;  // Odstęp pionowy w obwódce
  
  // Czarna obwódka tylko dla "ESP32"
  tft.fillRect(startX - paddingX, textY - paddingY, 
               part1W + (2 * paddingX), textH + (2 * paddingY), 
               TFT_BLACK);
  
  // "ESP32" w kolorze pomarańczowym (kolor paska)
  tft.setTextColor(TFT_RADIO_ORANGE);
  tft.setCursor(startX, textY);
  tft.print(logoPart1);
  
  // "-HAM-CLOCK" w kolorze czarnym (tak jak było)
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(startX + part1W, textY);
  tft.print(logoPart2);

  // 3. IKONA WiFi W PRAWYM GÓRNYM ROGU - BELKI JAK W TELEFONIE
  drawWifiSignalBars(screenW - 45, 8);

  // 4. ZEGAR W LEWYM GÓRNYM ROGU NAGŁÓWKA
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1)) {
    char timeBuffer[6];
    strftime(timeBuffer, 6, "%H:%M", &timeinfo);
    String timeStr = String(timeBuffer);
    tft.setTextSize(2);
    int timeWidth = tft.textWidth(timeStr);
    int timeX = 10; // Lewa strona nagłówka
    int timeY = 12;
    // Wyczyść tło pod zegarem
    tft.fillRect(timeX - 2, timeY - 2, timeWidth + 4, 20, TFT_RADIO_ORANGE);
    tft.setTextColor(TFT_BLACK);
    tft.setCursor(timeX, timeY);
    tft.print(timeStr);
  }

  screen1HeaderNeedsRedraw = false;
}

void drawHamClock() {
  // Ciemne tło - profesjonalny wygląd i mniejsze zmęczenie oczu
  tft.fillScreen(TFT_BLACK);
  screen1HeaderNeedsRedraw = true;
  
  const int screenW = 480;
  const int screenH = 320;
  
  // 1. NAGŁÓWEK - Pomarańczowa belka z wyśrodkowanym napisem i ikoną WiFi
  updateScreen1Header();

  // 2. RAMKA WOKÓŁ ZNAKU, ZEGARA I DATY - powiększona
  int clockFrameX = 30;     // Odstęp od lewej
  int clockFrameW = 420;    // Szerokość ramki (szersza)
  int clockFrameY = 45;     // Góra ramki
  int clockFrameH = 220;    // Wysokość ramki (powiększona dla większego zegara i imienin)
  tft.drawRoundRect(clockFrameX, clockFrameY, clockFrameW, clockFrameH, 10, TFT_DARKGREY);

  // 3. CALLSIGN - Wyśrodkowany w ramce (dokładne obliczenia)
  tft.setTextColor(callsignColor);
  tft.setTextSize(5);
  String callsign = (userCallsign.length() > 0) ? userCallsign : DEFAULT_CALLSIGN;
  callsign.toUpperCase();
  int callWidth = tft.textWidth(callsign);
  int callX = (screenW - callWidth) / 2;
  int callY = 55;
  tft.setCursor(max(10, callX), callY);
  tft.print(callsign);

  // Napis "OPERATOR STATION" - pod callsign (wyśrodkowany)
  tft.setTextSize(1);
  tft.setTextColor(TFT_RADIO_ORANGE);
  String opText = "OPERATOR STATION";
  int opWidth = tft.textWidth(opText);
  int opX = (screenW - opWidth) / 2;
  tft.setCursor(opX, callY + 40);
  tft.print(opText);

  // 4. ZEGAR - Dokładnie wyśrodkowany
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(7);
  String timeStr = (screen1TimeMode == SCREEN1_TIME_LOCAL)
                     ? getTimezoneTimeString("%H:%M:%S", 9)
                     : getUtcTimeString();
  int timeWidth = tft.textWidth(timeStr);
  int timeX = (screenW - timeWidth) / 2;
  int timeY = 130;
  tft.setCursor(max(10, timeX), timeY);
  tft.print(timeStr);

  // Etykieta czasu (UTC/Local)
  tft.setTextSize(1);
  const char *timeLabel = "UTC";
  if (screen1TimeMode == SCREEN1_TIME_LOCAL) {
    timeLabel = (tftLanguage == TFT_LANG_EN) ? "Local Time" : "Czas Lokalny";
  }
  int labelWidth = tft.textWidth(timeLabel);
  int labelX = (screenW - labelWidth) / 2;
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(labelX, timeY + 50);
  tft.print(timeLabel);

  // 5. DATA - Dokładnie wyśrodkowana
  String dateText;
  if (screen1TimeMode == SCREEN1_TIME_LOCAL) {
    dateText = (tftLanguage == TFT_LANG_EN)
                 ? getEnglishDateStringFullWithTimezone()
                 : getPolishDateStringFullWithTimezone();
  } else {
    dateText = (tftLanguage == TFT_LANG_EN)
                 ? getEnglishDateStringFull()
                 : getPolishDateStringFull();
  }
  
  // Data - używamy fontu VLW jeśli dostępny, inaczej standardowy
  int dateY = timeY + 65;  // Pozycja Y daty - używana też w imieninach
  if (littleFsReady && LittleFS.exists(ROBOTO_FONT20_FILE)) {
    tft.loadFont(ROBOTO_FONT20_NAME, LittleFS);
    tft.setTextColor(TFT_WHITE);
    int dateWidth = tft.textWidth(dateText);
    int dateX = (screenW - dateWidth) / 2;
    tft.setCursor(max(10, dateX), dateY);
    tft.print(dateText);
    tft.unloadFont();
  } else {
    tft.setTextSize(3);
    tft.setTextColor(TFT_WHITE);
    int dateWidth = tft.textWidth(sanitizePolishToAscii(dateText));
    int dateX = (screenW - dateWidth) / 2;
    tft.setCursor(max(10, dateX), dateY);
    tft.print(sanitizePolishToAscii(dateText));
  }

  // 6. IMIENINY - pod datą (tylko dla języka polskiego lub domyślnie)
  if (tftLanguage != TFT_LANG_EN) {
    String nameDayText = getPolishNameDay();
    if (nameDayText.length() > 0) {
      int nameDayY = dateY + 35;  // Pozycja pod datą
      
      // Używamy fontu VLW jeśli dostępny (obsługuje polskie znaki)
      if (littleFsReady && LittleFS.exists(ROBOTO_FONT12_FILE)) {
        tft.loadFont(ROBOTO_FONT12_NAME, LittleFS);
        tft.setTextColor(TFT_RADIO_ORANGE);
        int nameDayWidth = tft.textWidth(nameDayText);
        int nameDayX = (screenW - nameDayWidth) / 2;
        if (nameDayX < 10) nameDayX = 10;
        tft.setCursor(nameDayX, nameDayY);
        tft.print(nameDayText);
        tft.unloadFont();
      } else {
        // Fallback - standardowe fonty bez polskich znaków
        tft.setTextSize(2);
        tft.setTextColor(TFT_RADIO_ORANGE);
        String nameDayAscii = sanitizePolishToAscii(nameDayText);
        int nameDayWidth = nameDayAscii.length() * 12;
        int nameDayX = (screenW - nameDayWidth) / 2;
        if (nameDayX < 10) {
          tft.setTextSize(1);
          nameDayWidth = nameDayAscii.length() * 6;
          nameDayX = (screenW - nameDayWidth) / 2;
        }
        if (nameDayX < 10) nameDayX = 10;
        tft.setCursor(nameDayX, nameDayY);
        tft.print(nameDayAscii);
      }
    }
  }

  // 7. DOLNY PASEK - Adres IP
  IPAddress ip;
  bool connected = false;
  if (wifiConnected && WiFi.status() == WL_CONNECTED) {
    ip = WiFi.localIP();
    connected = true;
  } else if (WiFi.getMode() & WIFI_AP) {
    ip = WiFi.softAPIP();
    connected = true;
  }

  if (connected) {
    tft.setTextColor(TFT_LIGHTGREY);
    tft.setTextSize(2);
    String ipStr = ip.toString();
    int ipWidth = ipStr.length() * 12;
    int ipX = (screenW - ipWidth) / 2;
    tft.setCursor(ipX, screenH - 20);
    tft.print(ipStr);
  } else {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    String noConn = "NO CONNECTION";
    int ncWidth = noConn.length() * 12;
    int ncX = (screenW - ncWidth) / 2;
    tft.setCursor(ncX, screenH - 20);
    tft.print(noConn);
  }
  
  // 8. STRZAŁKI NAWIGACYJNE
  int arrowY = screenH - 30;
  int arrowSize = 12;
  tft.fillTriangle(15, arrowY, 15 + arrowSize, arrowY - arrowSize, 15 + arrowSize, arrowY + arrowSize, TFT_RADIO_ORANGE);
  tft.fillTriangle(screenW - 15, arrowY, screenW - 15 - arrowSize, arrowY - arrowSize, screenW - 15 - arrowSize, arrowY + arrowSize, TFT_RADIO_ORANGE);
  
  // 9. IKONA BATERII - od razu przy starcie (skipScreenCheck = true)
#ifdef BATTERY_MONITORING_ENABLED
  drawBatteryQuickUpdate(true);
#endif
}

void updateScreen1Clock() {
  if (!tftInitialized || currentScreen != SCREEN_HAM_CLOCK || inMenu) {
    return;
  }

  const int screenW = 480;
  const int timeY = 130;  // Zaktualizowana pozycja Y dla większego zegara

  String timeStr = (screen1TimeMode == SCREEN1_TIME_LOCAL)
                     ? getTimezoneTimeString("%H:%M:%S", 9)
                     : getUtcTimeString();
  
  // Wyśrodkowanie na całym ekranie, TextSize 7 (powiększony)
  int timeWidth = timeStr.length() * 42;  // ~42px na znak przy TextSize 7
  int timeX = (screenW - timeWidth) / 2;
  if (timeX < 10) timeX = 10;
  
  // Stała szerokość paddingu dla stabilnego czyszczenia (max 8 znaków * 42px)
  const int fixedPaddingWidth = 8 * 42 + 10; // 336 + 10 margin
  
  // Narysuj zegar z tłem (bez czyszczenia - eliminuje migotanie)
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // Tekst z tłem
  tft.setTextSize(7);  // POWIĘKSZONY zegar
  tft.setTextPadding(fixedPaddingWidth); // Stały padding dla stabilności
  tft.setCursor(timeX, timeY);
  tft.print(timeStr);
  tft.setTextPadding(0); // Reset
}

void updateScreen1Date() {
  if (!tftInitialized || currentScreen != SCREEN_HAM_CLOCK || inMenu) {
    return;
  }

  struct tm timeinfo;
  bool gotTime = false;
  if (screen1TimeMode == SCREEN1_TIME_LOCAL) {
    gotTime = getTimeWithTimezone(&timeinfo);
  } else {
    gotTime = getLocalTime(&timeinfo, 1);
  }
  if (!gotTime) {
    return;
  }

  static int lastDay = -1;
  static int lastMonth = -1;
  static int lastYear = -1;
  if (timeinfo.tm_mday == lastDay &&
      timeinfo.tm_mon == lastMonth &&
      timeinfo.tm_year == lastYear) {
    return;
  }
  lastDay = timeinfo.tm_mday;
  lastMonth = timeinfo.tm_mon;
  lastYear = timeinfo.tm_year;

  String dateText;
  if (screen1TimeMode == SCREEN1_TIME_LOCAL) {
    dateText = (tftLanguage == TFT_LANG_EN)
                 ? getEnglishDateStringFullWithTimezone()
                 : getPolishDateStringFullWithTimezone();
  } else {
    dateText = (tftLanguage == TFT_LANG_EN)
                 ? getEnglishDateStringFull()
                 : getPolishDateStringFull();
  }
  drawDateLine(dateText);
}

void updateScreen2Clock() {
  // ZEGAR WYŁĄCZONY - usunięty z nagłówka DX Cluster
  return;
}

bool spotMatchesScreen2Filters(const DXSpot &spot) {
  if (screen2FilterMode != FILTER_MODE_ALL) {
    if (screen2FilterMode == FILTER_MODE_CW && spot.mode != "CW") return false;
    if (screen2FilterMode == FILTER_MODE_SSB && spot.mode != "SSB") return false;
    if (screen2FilterMode == FILTER_MODE_DIGI && !(spot.mode == "FT8" || spot.mode == "FT4")) return false;
  }
  if (screen2FilterBandIndex > 0 && screen2FilterBandIndex < SCREEN8_FILTER_BANDS_COUNT) {
    String selectedBand = SCREEN8_FILTER_BANDS[screen2FilterBandIndex];
    float freqMHz = spot.frequency;

    if (selectedBand == "VHF") {
      if (!(freqMHz >= 30.0f && freqMHz < 300.0f)) return false;
    } else if (selectedBand == "UHF") {
      if (!(freqMHz >= 300.0f && freqMHz < 3000.0f)) return false;
    } else if (selectedBand == "SHF") {
      if (!(freqMHz >= 3000.0f && freqMHz < 30000.0f)) return false;
    } else {
      if (spot.band != selectedBand) return false;
    }
  }
  return true;
}

bool spotMatchesScreen7Filters(const DXSpot &spot) {
  if (screen7FilterMode != FILTER_MODE_ALL) {
    if (screen7FilterMode == FILTER_MODE_CW && spot.mode != "CW") return false;
    if (screen7FilterMode == FILTER_MODE_SSB && spot.mode != "SSB") return false;
    if (screen7FilterMode == FILTER_MODE_DIGI && !(spot.mode == "FT8" || spot.mode == "FT4")) return false;
  }
  if (screen7FilterBandIndex > 0 && screen7FilterBandIndex < SCREEN2_FILTER_BANDS_COUNT) {
    if (spot.band != SCREEN2_FILTER_BANDS[screen7FilterBandIndex]) return false;
  }
  return true;
}

bool spotMatchesScreen8Filters(const DXSpot &spot) {
  if (screen8FilterMode != FILTER_MODE_ALL) {
    if (screen8FilterMode == FILTER_MODE_CW && spot.mode != "CW") return false;
    if (screen8FilterMode == FILTER_MODE_SSB && spot.mode != "SSB") return false;
    if (screen8FilterMode == FILTER_MODE_DIGI && !(spot.mode == "FT8" || spot.mode == "FT4")) return false;
  }
  if (screen8FilterBandIndex > 0 && screen8FilterBandIndex < SCREEN8_FILTER_BANDS_COUNT) {
    String selectedBand = SCREEN8_FILTER_BANDS[screen8FilterBandIndex];
    float freqMHz = spot.frequency;

    if (selectedBand == "VHF") {
      if (!(freqMHz >= 30.0f && freqMHz < 300.0f)) return false;
    } else if (selectedBand == "UHF") {
      if (!(freqMHz >= 300.0f && freqMHz < 3000.0f)) return false;
    } else if (selectedBand == "SHF") {
      if (!(freqMHz >= 3000.0f && freqMHz < 30000.0f)) return false;
    } else {
      if (spot.band != selectedBand) return false;
    }
  }
  return true;
}

float getAprsSortDistance(const APRSStation &s) {
  if (s.hasLatLon && s.distance > 0) {
    return s.distance;
  }
  return 1.0e9f;
}

bool aprsSortLess(int a, int b) {
  if (screen6SortMode == APRS_SORT_CALLSIGN) {
    String ca = aprsStations[a].callsign;
    String cb = aprsStations[b].callsign;
    ca.toUpperCase();
    cb.toUpperCase();
    if (ca == cb) {
      return a < b;
    }
    return ca < cb;
  }
  if (screen6SortMode == APRS_SORT_DISTANCE) {
    float da = getAprsSortDistance(aprsStations[a]);
    float db = getAprsSortDistance(aprsStations[b]);
    if (da == db) {
      String ca = aprsStations[a].callsign;
      String cb = aprsStations[b].callsign;
      ca.toUpperCase();
      cb.toUpperCase();
      return ca < cb;
    }
    return da < db;
  }
  // APRS_SORT_TIME = aktualna kolejność
  return a < b;
}

void buildAprsDisplayOrder(int *order, int &count) {
  count = min(aprsStationCount, MAX_APRS_DISPLAY_LCD);
  for (int i = 0; i < count; i++) {
    order[i] = i;
  }
  if (screen6SortMode == APRS_SORT_TIME) {
    return;
  }
  for (int i = 0; i < count - 1; i++) {
    int best = i;
    for (int j = i + 1; j < count; j++) {
      if (aprsSortLess(order[j], order[best])) {
        best = j;
      }
    }
    if (best != i) {
      int tmp = order[i];
      order[i] = order[best];
      order[best] = tmp;
    }
  }
}

uint32_t computeScreen2Signature() {
  // Prosty hash treści tabeli (10 lub 11 wierszy zależnie od paska nawigacji)
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  lockDxSpots();
  hash ^= (uint32_t)spotCount;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen2FilterMode;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen2FilterBandIndex;
  hash *= fnvPrime;
  hash ^= (uint32_t)dxTableSizeMode;
  hash *= fnvPrime;
  hash ^= isTableNavFooterVisible(SCREEN_DX_CLUSTER) ? 1u : 0u;
  hash *= fnvPrime;

  int maxRows = getDxTableMaxRows();
  int displayCount = 0;
  for (int i = 0; i < spotCount && displayCount < maxRows; i++) {
    const DXSpot &s = spots[i];
    if (!spotMatchesScreen2Filters(s)) {
      continue;
    }
    for (size_t j = 0; j < s.time.length(); j++) {
      hash ^= (uint8_t)s.time[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.callsign.length(); j++) {
      hash ^= (uint8_t)s.callsign[j];
      hash *= fnvPrime;
    }
    hash ^= (uint32_t)(s.frequency * 100);
    hash *= fnvPrime;
    for (size_t j = 0; j < s.mode.length(); j++) {
      hash ^= (uint8_t)s.mode[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.country.length(); j++) {
      hash ^= (uint8_t)s.country[j];
      hash *= fnvPrime;
    }
    hash ^= (uint32_t)s.distance;
    hash *= fnvPrime;
    displayCount++;
  }

  unlockDxSpots();

  return hash;
}

void updateScreen2Data() {
  if (!tftInitialized || currentScreen != SCREEN_DX_CLUSTER || inMenu) {
    return;
  }

  // 1) Zaktualizuj czas w nagłówku (bez pełnego odświeżania)
  updateScreen2Clock();

  // 2) Odśwież tabelę tylko gdy dane się zmieniły
  static uint32_t lastSig = 0;
  static unsigned long lastTableRedrawMs = 0;
  uint32_t currentSig = computeScreen2Signature();
  if (currentSig == lastSig) {
    return;
  }

  unsigned long now = millis();
  if (lastTableRedrawMs != 0 && (now - lastTableRedrawMs) < DX_SCREEN_MIN_REDRAW_MS) {
    return;
  }

  lastSig = currentSig;
  lastTableRedrawMs = now;

  // 3) Renderuj tabelę do bufora i wypchnij jednym ruchem (bez migotania)
  const bool navVisible = isTableNavFooterVisible(SCREEN_DX_CLUSTER);
  const bool enlarged = isDxTableEnlarged();
  const int maxRows = getDxTableMaxRows();
  const int tableTop = TFT_TABLE_TOP;
  const int tableBottom = getTableBottomForScreen(SCREEN_DX_CLUSTER);
  const int tableHeight = tableBottom - tableTop;
  TFT_eSprite *tableSprite = (navVisible && ensureSharedTableSprite()) ? &sharedTableSprite : nullptr;

  if (tableSprite != nullptr) {
    tableSprite->fillSprite(TFT_BLACK);

    int yPos = 8;
    tableSprite->setTextColor(TFT_DARKGREY);
    tableSprite->setTextSize(1);
    if (enlarged) {
      tableSprite->setCursor(5, yPos);   tableSprite->print("UTC");
      tableSprite->setCursor(74, yPos);  tableSprite->print("CALL");
      tableSprite->setCursor(204, yPos); tableSprite->print("MHz");
      tableSprite->setCursor(282, yPos); tableSprite->print("MODE");
    } else {
      tableSprite->setCursor(5, yPos);   tableSprite->print("UTC");
      tableSprite->setCursor(50, yPos);  tableSprite->print("CALLSIGN");
      tableSprite->setCursor(125, yPos); tableSprite->print("MHz");
      tableSprite->setCursor(190, yPos); tableSprite->print("MODE");
      tableSprite->setCursor(260, yPos); tableSprite->print(sanitizePolishToAscii(String(tr(TR_COUNTRY))));
    }
    tableSprite->drawFastHLine(0, yPos + 10, 480, TFT_DARKGREY);
    yPos += enlarged ? 20 : 18;

    int displayCount = 0;
    lockDxSpots();
    for (int i = 0; i < spotCount && displayCount < maxRows; i++) {
      if (!spotMatchesScreen2Filters(spots[i])) {
        continue;
      }
      if (yPos >= (tableHeight - 2)) {
        break;
      }
      if (enlarged) {
        if (displayCount % 2 == 0) {
          tableSprite->fillRect(0, yPos - 5, 480, 24, TFT_TABLE_ALT_ROW_COLOR);
        }

        tableSprite->setTextSize(2);
        tableSprite->setTextColor(TFT_LIGHTGREY);
        tableSprite->setCursor(5, yPos);
        tableSprite->print(formatSpotUtc(spots[i].time));

        tableSprite->setTextColor(TFT_WHITE);
        tableSprite->setCursor(74, yPos);
        String callText = spots[i].callsign;
        if (callText.length() > 8) callText = callText.substring(0, 8);
        tableSprite->print(callText);

        tableSprite->setTextColor(TFT_YELLOW);
        tableSprite->setCursor(204, yPos);
        tableSprite->print(spots[i].frequency / 1000.0, 3);

        tableSprite->setTextColor(TFT_GREEN);
        tableSprite->setCursor(282, yPos);
        String modeText = spots[i].mode;
        if (modeText.length() > 4) modeText = modeText.substring(0, 4);
        tableSprite->print(modeText);

        yPos += 27;
      } else {
        if (displayCount % 2 == 0) {
          tableSprite->fillRect(0, yPos - 2, 480, 16, TFT_TABLE_ALT_ROW_COLOR);
        }

        tableSprite->setTextSize(1);

        tableSprite->setTextColor(TFT_LIGHTGREY);
        tableSprite->setCursor(5, yPos);
        tableSprite->print(formatSpotUtc(spots[i].time));

        tableSprite->setTextColor(TFT_WHITE, TFT_BLACK);
        tableSprite->setTextSize(1);
        tableSprite->setCursor(50, yPos);
        String callText = spots[i].callsign;
        callText.toUpperCase();
        if (callText.length() > 10) callText = callText.substring(0, 10);
        tableSprite->print(callText);

        tableSprite->setTextColor(TFT_YELLOW);
        tableSprite->setCursor(125, yPos);
        tableSprite->print(spots[i].frequency / 1000.0, 3);

        tableSprite->setTextColor(TFT_GREEN);
        tableSprite->setCursor(190, yPos);
        tableSprite->print(spots[i].mode);

        tableSprite->setTextColor(TFT_RADIO_ORANGE);
        tableSprite->setCursor(260, yPos);
        String countryText = formatDistanceOrCountry(spots[i], COUNTRY_COL_MAX_LEN);
        tableSprite->print(countryText);

        yPos += 17;
      }
      displayCount++;
    }
    unlockDxSpots();

    if (displayCount == 0) {
      tableSprite->setTextColor(TFT_RED);
      tableSprite->setTextSize(2);
      tableSprite->setCursor(40, 120 - tableTop);
      tableSprite->print("WAITING FOR SPOTS...");
    }

    tableSprite->pushSprite(0, tableTop);
    return;
  }

  // Fallback bez sprite (np. gdy zabraknie RAM)
  tft.fillRect(0, tableTop, 480, tableHeight, TFT_BLACK);

  int yPos = 40;
  tft.setTextColor(TFT_DARKGREY);
  tft.setTextSize(1);
  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(204, yPos); tft.print("MHz");
    tft.setCursor(330, yPos); tft.print("MODE");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("MHz");
    tft.setCursor(190, yPos); tft.print("MODE");
    tft.setCursor(260, yPos); tft.print(sanitizePolishToAscii(String(tr(TR_COUNTRY))));
  }
  tft.drawFastHLine(0, yPos + 10, 480, TFT_DARKGREY);
  yPos += enlarged ? 20 : 18;

  int displayCount = 0;
  lockDxSpots();
  for (int i = 0; i < spotCount && displayCount < maxRows; i++) {
    if (!spotMatchesScreen2Filters(spots[i])) {
      continue;
    }
    if (yPos >= (tableBottom - 2)) {
      break;
    }
    if (enlarged) {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 5, 480, 24, TFT_TABLE_ALT_ROW_COLOR);
      }

      tft.setTextSize(2);
      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      tft.print(formatSpotUtc(spots[i].time));

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(74, yPos);
      String callText = spots[i].callsign;
      if (callText.length() > 8) callText = callText.substring(0, 8);
      tft.print(callText);

      tft.setTextColor(TFT_YELLOW);
      tft.setCursor(204, yPos);
      tft.print(spots[i].frequency / 1000.0, 3);

      tft.setTextColor(TFT_GREEN);
      tft.setCursor(330, yPos);
      String modeText = spots[i].mode;
      if (modeText.length() > 4) modeText = modeText.substring(0, 4);
      tft.print(modeText);

      yPos += 27;
    } else {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 2, 480, 16, TFT_TABLE_ALT_ROW_COLOR);
      }

      tft.setTextSize(1);

      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      tft.print(formatSpotUtc(spots[i].time));

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(50, yPos);
      String callStr = spots[i].callsign;
      callStr.toUpperCase();
      tft.print(callStr);

      tft.setTextColor(TFT_YELLOW);
      tft.setCursor(125, yPos);
      tft.print(spots[i].frequency / 1000.0, 3);

      tft.setTextColor(TFT_GREEN);
      tft.setCursor(190, yPos);
      String modeStr = spots[i].mode;
      if (modeStr.length() > 4) modeStr = modeStr.substring(0, 4);
      tft.print(modeStr);

      tft.setTextColor(TFT_RADIO_ORANGE);
      tft.setCursor(260, yPos);
      String countryText = formatDistanceOrCountry(spots[i], COUNTRY_COL_MAX_LEN);
      tft.print(countryText);

      yPos += 17;
    }
    displayCount++;
  }
  unlockDxSpots();

  if (displayCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR SPOTS...");
  }
  
  // Strzałki nawigacyjne - ZAWSZE rysuj po odświeżeniu tabeli
  drawSwitchScreenFooter();
}

// Ekran 7: POTA Cluster (SSB only, 10 spotów)
void drawPotaCluster() {
  // Użyj tych samych współrzędnych co w drawHamClock() - ograniczenie do obszaru ramki

  tft.fillRect(0, 0, 480, 32, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);

  drawHamburgerMenuButton3D(5, 7);

  // Napis POTA.app
  tft.setTextSize(2);
  String potaText = "POTA.app";
  int potaX = 45;
  tft.setCursor(potaX, 8);
  tft.print(potaText);

  // Ikona WiFi w górnym prawym rogu
  drawWifiSignalBars(445, 4);

  // ZEGAR w prawym górnym rogu - USUNIĘTY
  // struct tm timeinfo;
  // if (getLocalTime(&timeinfo, 1)) {
  //   char timeBuffer[10];
  //   strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
  //   int timeWidth = strlen(timeBuffer) * 12;
  //   tft.setCursor(460 - timeWidth, 8);
  //   tft.print(timeBuffer);
  // }

  int yPos = 40;
  const bool enlarged = isDxTableEnlarged();
  tft.setTextColor(TFT_DARKGREY);
  tft.setTextSize(1);

  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(204, yPos); tft.print("MHz");
    tft.setCursor(330, yPos); tft.print("MODE");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(140, yPos); tft.print("MHz");
    tft.setCursor(260, yPos); tft.print("MODE");
    tft.setCursor(360, yPos); tft.print(sanitizePolishToAscii(String(tr(TR_COUNTRY))));
  }
  tft.drawFastHLine(0, yPos + 10, 480, TFT_DARKGREY);
  yPos += enlarged ? 20 : 18;

  int maxRows = getPotaTableMaxRows();
  int displayCount = 0;
  for (int i = 0; i < potaSpotCount && displayCount < maxRows; i++) {
    if (!spotMatchesScreen7Filters(potaSpots[i])) continue;
    if (enlarged) {
      if (displayCount % 2 == 0) tft.fillRect(0, yPos - 5, 480, 24, TFT_TABLE_ALT_ROW_COLOR);

      tft.setTextSize(2);
      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      tft.print(formatSpotUtc(potaSpots[i].time));

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(74, yPos);
      String callText = potaSpots[i].callsign;
      if (callText.length() > 8) callText = callText.substring(0, 8);
      tft.print(callText);

      tft.setTextColor(TFT_YELLOW);
      tft.setCursor(204, yPos);
      tft.print(potaSpots[i].frequency, 3);

      tft.setTextColor(TFT_GREEN);
      tft.setCursor(330, yPos);
      String modeText = potaSpots[i].mode;
      if (modeText.length() > 4) modeText = modeText.substring(0, 4);
      tft.print(modeText);

      yPos += 27;
    } else {
      if (displayCount % 2 == 0) tft.fillRect(0, yPos - 2, 480, 16, TFT_TABLE_ALT_ROW_COLOR);

      tft.setTextSize(1);
      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      String timeStr = potaSpots[i].time;
      // Obsłuż ISO 8601 z datą: wyciągnij HH:MM
      int tPos = timeStr.indexOf('T');
      if (tPos > 0 && tPos + 5 < (int)timeStr.length()) {
        timeStr = timeStr.substring(tPos + 1, tPos + 6);
      }
      if (timeStr.length() > 5) timeStr = timeStr.substring(0, 5);
      tft.print(timeStr);

      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setTextSize(1);
      tft.setCursor(50, yPos);
      String callStr = potaSpots[i].callsign;
      callStr.toUpperCase();
      tft.print(callStr);

      tft.setTextColor(TFT_YELLOW);
      tft.setCursor(125, yPos);
      tft.print(potaSpots[i].frequency, 3); // frequency już w MHz

      tft.setTextColor(TFT_GREEN);
      tft.setCursor(240, yPos);
      tft.print(potaSpots[i].mode);

      tft.setTextColor(TFT_RADIO_ORANGE);
      tft.setCursor(220, yPos);
      String countryText = formatDistanceOrCountry(potaSpots[i], COUNTRY_COL_MAX_LEN);
      tft.print(countryText);

      yPos += 17;
    }
    displayCount++;
  }

  if (isTableNavFooterVisible(SCREEN_POTA_CLUSTER)) {
    // Strzałki nawigacyjne - duże, blisko krawędzi (takie same na wszystkich ekranach)
    int arrowY = 290;
    int arrowSize = 12;
    tft.fillTriangle(15, arrowY, 15 + arrowSize, arrowY - arrowSize, 15 + arrowSize, arrowY + arrowSize, menuThemeColor);
    tft.fillTriangle(465, arrowY, 465 - arrowSize, arrowY - arrowSize, 465 - arrowSize, arrowY + arrowSize, menuThemeColor);
  }

  if (displayCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR SPOTS...");
  }
}

void updateScreen7Clock() {
  if (!tftInitialized || currentScreen != 7 || inMenu) {
    return;
  }

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1)) {
    return;
  }

  char timeBuffer[10];
  strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
  int timeWidth = strlen(timeBuffer) * 12;
  int timeX = 315 - timeWidth;

  tft.fillRect(timeX - 2, 4, timeWidth + 6, 24, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(timeX, 8);
  tft.print(timeBuffer);
}

uint32_t computeScreen7Signature() {
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  hash ^= (uint32_t)potaSpotCount;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen7FilterMode;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen7FilterBandIndex;
  hash *= fnvPrime;
  hash ^= (uint32_t)dxTableSizeMode;
  hash *= fnvPrime;
  hash ^= isTableNavFooterVisible(SCREEN_POTA_CLUSTER) ? 1u : 0u;
  hash *= fnvPrime;

  int maxDisplaySpots = min(potaSpotCount, getPotaTableMaxRows());
  int displayCount = 0;
  for (int i = 0; i < potaSpotCount && displayCount < maxDisplaySpots; i++) {
    const DXSpot &s = potaSpots[i];
    if (!spotMatchesScreen7Filters(s)) continue;
    for (size_t j = 0; j < s.time.length(); j++) {
      hash ^= (uint8_t)s.time[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.callsign.length(); j++) {
      hash ^= (uint8_t)s.callsign[j];
      hash *= fnvPrime;
    }

    hash ^= (uint32_t)(s.frequency * 100);
    hash *= fnvPrime;
    for (size_t j = 0; j < s.mode.length(); j++) {
      hash ^= (uint8_t)s.mode[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.country.length(); j++) {
      hash ^= (uint8_t)s.country[j];
      hash *= fnvPrime;
    }
    displayCount++;
  }

  return hash;
}

void updateScreen7Data() {
  if (!tftInitialized || currentScreen != 7 || inMenu) {
    return;
  }

  // Wyłączono - używamy tylko updatePotaClusterClock dla zegara w prawym rogu
  // updateScreen7Clock();

  static uint32_t lastSig = 0;
  uint32_t currentSig = computeScreen7Signature();
  if (currentSig == lastSig) {
    return;
  }
  lastSig = currentSig;

  const bool navVisible = isTableNavFooterVisible(SCREEN_POTA_CLUSTER);
  const bool enlarged = isDxTableEnlarged();
  const int maxRows = getPotaTableMaxRows();
  const int tableTop = TFT_TABLE_TOP;
  const int tableBottom = getTableBottomForScreen(SCREEN_POTA_CLUSTER);
  const int tableHeight = tableBottom - tableTop;
  TFT_eSprite *tableSprite = (navVisible && ensureSharedTableSprite()) ? &sharedTableSprite : nullptr;

  if (tableSprite != nullptr) {
    tableSprite->fillSprite(TFT_BLACK);

    int yPos = 8;
    tableSprite->setTextColor(TFT_DARKGREY);
    tableSprite->setTextSize(1);
    if (enlarged) {
      tableSprite->setCursor(5, yPos);   tableSprite->print("UTC");
      tableSprite->setCursor(74, yPos);  tableSprite->print("CALL");
      tableSprite->setCursor(204, yPos); tableSprite->print("MHz");
      tableSprite->setCursor(282, yPos); tableSprite->print("MODE");
    } else {
      tableSprite->setCursor(5, yPos);   tableSprite->print("UTC");
      tableSprite->setCursor(50, yPos);  tableSprite->print("CALLSIGN");
      tableSprite->setCursor(125, yPos); tableSprite->print("MHz");
      tableSprite->setCursor(190, yPos); tableSprite->print("MODE");
      tableSprite->setCursor(260, yPos); tableSprite->print(sanitizePolishToAscii(String(tr(TR_COUNTRY))));
    }
    tableSprite->drawFastHLine(0, yPos + 10, 480, TFT_DARKGREY);
    yPos += enlarged ? 20 : 18;

    int displayCount = 0;
    for (int i = 0; i < potaSpotCount && displayCount < maxRows; i++) {
      if (!spotMatchesScreen7Filters(potaSpots[i])) continue;
      if (yPos >= (tableHeight - 2)) {
        break;
      }
      if (enlarged) {
        if (displayCount % 2 == 0) {
          tableSprite->fillRect(0, yPos - 5, 480, 24, TFT_TABLE_ALT_ROW_COLOR);
        }

        tableSprite->setTextSize(2);

        tableSprite->setTextColor(TFT_LIGHTGREY);
        tableSprite->setCursor(5, yPos);
        tableSprite->print(formatSpotUtc(potaSpots[i].time));

        tableSprite->setTextColor(TFT_WHITE);
        tableSprite->setCursor(74, yPos);
        String callText = potaSpots[i].callsign;
        if (callText.length() > 8) callText = callText.substring(0, 8);
        tableSprite->print(callText);

        tableSprite->setTextColor(TFT_YELLOW);
        tableSprite->setCursor(204, yPos);
        tableSprite->print(potaSpots[i].frequency, 3);

        tableSprite->setTextColor(TFT_GREEN);
        tableSprite->setCursor(282, yPos);
        String modeText = potaSpots[i].mode;
        if (modeText.length() > 4) modeText = modeText.substring(0, 4);
        tableSprite->print(modeText);

        yPos += 27;
      } else {
        if (displayCount % 2 == 0) {
          tableSprite->fillRect(0, yPos - 2, 480, 16, TFT_TABLE_ALT_ROW_COLOR);
        }

        tableSprite->setTextSize(1);

        tableSprite->setTextColor(TFT_LIGHTGREY);
        tableSprite->setCursor(5, yPos);
        String timeStr = potaSpots[i].time;
        int tPos = timeStr.indexOf('T');
        if (tPos > 0 && tPos + 5 < (int)timeStr.length()) {
          timeStr = timeStr.substring(tPos + 1, tPos + 6);
        }
        if (timeStr.length() > 5) timeStr = timeStr.substring(0, 5);
        tableSprite->print(timeStr);

        tableSprite->setTextColor(TFT_WHITE);
        tableSprite->setCursor(50, yPos);
        tableSprite->print(potaSpots[i].callsign);

        tableSprite->setTextColor(TFT_YELLOW);
        tableSprite->setCursor(125, yPos);
        tableSprite->print(potaSpots[i].frequency, 3);

        tableSprite->setTextColor(TFT_GREEN);
        tableSprite->setCursor(190, yPos);
        tableSprite->print(potaSpots[i].mode);

        tableSprite->setTextColor(TFT_RADIO_ORANGE);
        tableSprite->setCursor(260, yPos);
        String countryText = formatDistanceOrCountry(potaSpots[i], COUNTRY_COL_MAX_LEN);
        tableSprite->print(countryText);

        yPos += 17;
      }
      displayCount++;
    }

    if (displayCount == 0) {
      tableSprite->setTextColor(TFT_RED);
      tableSprite->setTextSize(2);
      tableSprite->setCursor(40, 120 - tableTop);
      tableSprite->print("WAITING FOR SPOTS...");
    }

    tableSprite->pushSprite(0, tableTop);
    return;
  }

  // Fallback bez sprite (np. gdy zabraknie RAM)
  tft.fillRect(0, tableTop, 480, tableHeight, TFT_BLACK);

  int yPos = 40;
  tft.setTextColor(TFT_DARKGREY);
  tft.setTextSize(1);
  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(204, yPos); tft.print("MHz");
    tft.setCursor(330, yPos); tft.print("MODE");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("MHz");
    tft.setCursor(260, yPos); tft.print("MODE");
    tft.setCursor(360, yPos); tft.print(sanitizePolishToAscii(String(tr(TR_COUNTRY))));
  }
  tft.drawFastHLine(0, yPos + 10, 480, TFT_DARKGREY);
  yPos += enlarged ? 20 : 18;

  int displayCount = 0;
  for (int i = 0; i < potaSpotCount && displayCount < maxRows; i++) {
    if (!spotMatchesScreen7Filters(potaSpots[i])) continue;
    if (yPos >= (tableBottom - 2)) {
      break;
    }
    if (enlarged) {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 5, 480, 24, TFT_TABLE_ALT_ROW_COLOR);
      }

      tft.setTextSize(2);

      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      tft.print(formatSpotUtc(potaSpots[i].time));

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(74, yPos);
      String callText = potaSpots[i].callsign;
      if (callText.length() > 8) callText = callText.substring(0, 8);
      tft.print(callText);

      tft.setTextColor(TFT_YELLOW);
      tft.setCursor(204, yPos);
      tft.print(potaSpots[i].frequency, 3);

      tft.setTextColor(TFT_GREEN);
      tft.setCursor(330, yPos);
      String modeText = potaSpots[i].mode;
      if (modeText.length() > 4) modeText = modeText.substring(0, 4);
      tft.print(modeText);

      yPos += 27;
    } else {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 2, 480, 16, TFT_TABLE_ALT_ROW_COLOR);
      }

      // Znak wywoławczy - rozmiar 1 (standardowy)
      tft.setTextSize(1);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setCursor(50, yPos);
      String callStr = potaSpots[i].callsign;
      callStr.toUpperCase();
      if (callStr.length() > 8) callStr = callStr.substring(0, 8);
      tft.print(callStr);
      
      // Częstotliwość - rozmiar 1
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.setCursor(140, yPos);
      tft.print(potaSpots[i].frequency, 3);
      
      // Mode - rozmiar 1
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setCursor(260, yPos);
      String modeStr = potaSpots[i].mode;
      if (modeStr.length() > 4) modeStr = modeStr.substring(0, 4);
      tft.print(modeStr);
      
      // Czas i kraj - rozmiar 1 (dopasowane do nagłówków)
      tft.setTextSize(1);
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
      tft.setCursor(5, yPos + 5);
      String timeStr = potaSpots[i].time;
      int tPos = timeStr.indexOf('T');
      if (tPos > 0 && tPos + 5 < (int)timeStr.length()) {
        timeStr = timeStr.substring(tPos + 1, tPos + 6);
      }
      if (timeStr.length() > 5) timeStr = timeStr.substring(0, 5);
      tft.print(timeStr);

      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.setCursor(360, yPos + 5);
      String countryText = formatDistanceOrCountry(potaSpots[i], COUNTRY_COL_MAX_LEN);
      tft.print(countryText);

      yPos += 20;
    }
    displayCount++;
  }

  if (displayCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR SPOTS...");
  }
  
  // Strzałki nawigacyjne - ZAWSZE rysuj po odświeżeniu tabeli
  drawSwitchScreenFooter();
}

static String formatHamalertTftTime(const String &rawTime) {
  String t = rawTime;
  t.trim();
  if (t.length() == 0) return "-----";

  int tPos = t.indexOf('T');
  if (tPos > 0 && (tPos + 5) < (int)t.length()) {
    String hhmm = t.substring(tPos + 1, tPos + 6);
    if (hhmm.length() == 5) {
      return hhmm;
    }
  }

  if (t.length() >= 5 && t.charAt(2) == ':') {
    return t.substring(0, 5);
  }

  if (t.length() >= 5 && t.charAt(4) == 'Z' && isDigit((unsigned char)t.charAt(0)) && isDigit((unsigned char)t.charAt(1)) && isDigit((unsigned char)t.charAt(2)) && isDigit((unsigned char)t.charAt(3))) {
    String hhmm = t.substring(0, 4);
    return hhmm.substring(0, 2) + ":" + hhmm.substring(2, 4);
  }

  return t;
}

void drawHamalertCluster() {
  tft.fillScreen(TFT_BLACK);

  tft.fillRect(0, 0, 480, 32, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);

  drawHamburgerMenuButton3D(5, 7);

  tft.setTextSize(2);
  tft.setCursor(35, 8);
  tft.print("HAMALERT.org");

  // Ikona WiFi w górnym prawym rogu
  drawWifiSignalBars(445, 4);

  // ZEGAR - rysowany osobno przez updateHamalertClock()

  int yPos = 40;
  const bool enlarged = isDxTableEnlarged();
  tft.setTextColor(TFT_DARKGREY);
  tft.setTextSize(1);
  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(204, yPos); tft.print("MHz");
    tft.setCursor(330, yPos); tft.print("MODE");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("MHz");
    tft.setCursor(240, yPos); tft.print("MODE");
    tft.setCursor(220, yPos); tft.print(sanitizePolishToAscii(String(tr(TR_COUNTRY))));
  }
  tft.drawFastHLine(0, yPos + 10, 480, TFT_DARKGREY);
  yPos += enlarged ? 20 : 18;

  const int maxRows = getHamalertTableMaxRows();
  int displayCount = 0;
  for (int i = 0; i < hamalertSpotCount && displayCount < maxRows; i++) {
    if (!spotMatchesScreen8Filters(hamalertSpots[i])) {
      continue;
    }
    if (enlarged) {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 5, 480, 24, TFT_TABLE_ALT_ROW_COLOR);
      }

      tft.setTextSize(2);

      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      tft.print(formatHamalertTftTime(hamalertSpots[i].time));

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(74, yPos);
      String callText = hamalertSpots[i].callsign;
      if (callText.length() > 8) callText = callText.substring(0, 8);
      tft.print(callText);

      tft.setTextColor(TFT_YELLOW);
      tft.setCursor(204, yPos);
      tft.print(hamalertSpots[i].frequency, 3);

      tft.setTextColor(TFT_GREEN);
      tft.setCursor(330, yPos);
      String modeText = hamalertSpots[i].mode;
      if (modeText.length() > 4) modeText = modeText.substring(0, 4);
      tft.print(modeText);

      yPos += 27;
    } else {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 2, 480, 16, TFT_TABLE_ALT_ROW_COLOR);
      }

      tft.setTextSize(1);

      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      tft.print(formatHamalertTftTime(hamalertSpots[i].time));

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(50, yPos);
      tft.print(hamalertSpots[i].callsign);

      tft.setTextColor(TFT_YELLOW);
      tft.setCursor(125, yPos);
      tft.print(hamalertSpots[i].frequency, 3);

      tft.setTextColor(TFT_GREEN);
      tft.setCursor(240, yPos);
      tft.print(hamalertSpots[i].mode);

      tft.setTextColor(TFT_RADIO_ORANGE);
      tft.setCursor(220, yPos);
      String countryText = formatDistanceOrCountry(hamalertSpots[i], COUNTRY_COL_MAX_LEN);
      tft.print(countryText);

      yPos += 17;
    }
    displayCount++;
  }

  // Strzałki nawigacyjne - ZAWSZE widoczne
  drawSwitchScreenFooter();

  if (displayCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR SPOTS...");
  }
}

void updateScreen8Clock() {
  if (!tftInitialized || currentScreen != SCREEN_HAMALERT_CLUSTER || inMenu) {
    return;
  }

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 1)) {
    return;
  }

  char timeBuffer[10];
  strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
  int timeWidth = strlen(timeBuffer) * 12;
  int timeX = 475 - timeWidth;

  tft.fillRect(timeX - 2, 4, timeWidth + 6, 24, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(timeX, 8);
  tft.print(timeBuffer);
}

uint32_t computeScreen8Signature() {
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  hash ^= (uint32_t)hamalertSpotCount;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen8FilterMode;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen8FilterBandIndex;
  hash *= fnvPrime;
  hash ^= (uint32_t)dxTableSizeMode;
  hash *= fnvPrime;
  hash ^= isTableNavFooterVisible(SCREEN_HAMALERT_CLUSTER) ? 1u : 0u;
  hash *= fnvPrime;

  int maxRows = getHamalertTableMaxRows();
  int displayCount = 0;
  for (int i = 0; i < hamalertSpotCount && displayCount < maxRows; i++) {
    const DXSpot &s = hamalertSpots[i];
    if (!spotMatchesScreen8Filters(s)) {
      continue;
    }
    for (size_t j = 0; j < s.time.length(); j++) {
      hash ^= (uint8_t)s.time[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.callsign.length(); j++) {
      hash ^= (uint8_t)s.callsign[j];
      hash *= fnvPrime;
    }
    hash ^= (uint32_t)(s.frequency * 100);
    hash *= fnvPrime;
    for (size_t j = 0; j < s.mode.length(); j++) {
      hash ^= (uint8_t)s.mode[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.country.length(); j++) {
      hash ^= (uint8_t)s.country[j];
      hash *= fnvPrime;
    }
    displayCount++;
  }

  return hash;
}

void updateScreen8Data() {
  if (!tftInitialized || currentScreen != SCREEN_HAMALERT_CLUSTER || inMenu) {
    return;
  }

  // Wyłączono - zegar tylko na górnym pasku przez updateHamalertClock
  // updateScreen8Clock();

  static uint32_t lastSig = 0;
  uint32_t currentSig = computeScreen8Signature();
  if (currentSig == lastSig) {
    return;
  }
  lastSig = currentSig;

  const bool enlarged = isDxTableEnlarged();
  const int maxRows = getHamalertTableMaxRows();
  const int tableTop = TFT_TABLE_TOP;
  const int tableBottom = getTableBottomForScreen(SCREEN_HAMALERT_CLUSTER);
  const int tableHeight = tableBottom - tableTop;

  tft.fillRect(0, tableTop, 480, tableHeight, TFT_BLACK);

  int yPos = 40;
  tft.setTextColor(TFT_DARKGREY);
  tft.setTextSize(1);
  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(204, yPos); tft.print("MHz");
    tft.setCursor(330, yPos); tft.print("MODE");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("MHz");
    tft.setCursor(240, yPos); tft.print("MODE");
    tft.setCursor(220, yPos); tft.print(sanitizePolishToAscii(String(tr(TR_COUNTRY))));
  }
  tft.drawFastHLine(0, yPos + 10, 480, TFT_DARKGREY);
  yPos += enlarged ? 20 : 18;

  int displayCount = 0;
  for (int i = 0; i < hamalertSpotCount && displayCount < maxRows; i++) {
    if (!spotMatchesScreen8Filters(hamalertSpots[i])) {
      continue;
    }
    if (yPos >= (tableBottom - 2)) {
      break;
    }
    if (enlarged) {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 5, 480, 24, TFT_TABLE_ALT_ROW_COLOR);
      }

      tft.setTextSize(2);

      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      tft.print(formatHamalertTftTime(hamalertSpots[i].time));

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(74, yPos);
      String callText = hamalertSpots[i].callsign;
      if (callText.length() > 8) callText = callText.substring(0, 8);
      tft.print(callText);

      tft.setTextColor(TFT_YELLOW);
      tft.setCursor(204, yPos);
      tft.print(hamalertSpots[i].frequency, 3);

      tft.setTextColor(TFT_GREEN);
      tft.setCursor(330, yPos);
      String modeText = hamalertSpots[i].mode;
      if (modeText.length() > 4) modeText = modeText.substring(0, 4);
      tft.print(modeText);

      yPos += 27;
    } else {
      if (displayCount % 2 == 0) {
        tft.fillRect(0, yPos - 2, 480, 16, TFT_TABLE_ALT_ROW_COLOR);
      }

      tft.setTextSize(1);

      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      tft.print(formatHamalertTftTime(hamalertSpots[i].time));

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(50, yPos);
      tft.print(hamalertSpots[i].callsign);

      tft.setTextColor(TFT_YELLOW);
      tft.setCursor(125, yPos);
      tft.print(hamalertSpots[i].frequency, 3);

      tft.setTextColor(TFT_GREEN);
      tft.setCursor(240, yPos);
      tft.print(hamalertSpots[i].mode);

      tft.setTextColor(TFT_RADIO_ORANGE);
      tft.setCursor(220, yPos);
      String countryText = formatDistanceOrCountry(hamalertSpots[i], COUNTRY_COL_MAX_LEN);
      tft.print(countryText);

      yPos += 17;
    }
    displayCount++;
  }

  if (displayCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR SPOTS...");
  }
  
  // Strzałki nawigacyjne - ZAWSZE rysuj po odświeżeniu tabeli
  drawSwitchScreenFooter();
}

bool readRawTouchPoint(int16_t &rx, int16_t &ry) {
  if (!touch.touched()) {
    return false;
  }
  TS_Point p = touch.getPoint();
  rx = p.x;
  ry = p.y;
  return true;
}

void mapRawToScreenWithValues(int16_t rx, int16_t ry,
                              int xmin, int xmax, int ymin, int ymax,
                              bool swapXY, bool invertX, bool invertY,
                              uint16_t &x, uint16_t &y) {
  int16_t mx = rx;
  int16_t my = ry;

  if (swapXY) {
    int16_t tmp = mx;
    mx = my;
    my = tmp;
  }
  if (invertX) {
    mx = xmax - (mx - xmin);
  }
  if (invertY) {
    my = ymax - (my - ymin);
  }

  mx = constrain(mx, xmin, xmax);
  my = constrain(my, ymin, ymax);

  int screenW = tft.width();
  int screenH = tft.height();
  x = map(mx, xmin, xmax, 0, screenW - 1);
  y = map(my, ymin, ymax, 0, screenH - 1);
}

void mapRawToScreen(int16_t rx, int16_t ry, uint16_t &x, uint16_t &y) {
  mapRawToScreenWithValues(rx, ry, touchXMin, touchXMax, touchYMin, touchYMax,
                           touchSwapXY, touchInvertX, touchInvertY, x, y);
}

bool readTouchPoint(uint16_t &x, uint16_t &y) {
  int16_t rx = 0;
  int16_t ry = 0;
  if (!readRawTouchPoint(rx, ry)) {
    return false;
  }
  mapRawToScreen(rx, ry, x, y);
  return true;
}

const int SCREEN_ORDER_COUNT = SCREEN_PAGE_COUNT;

ScreenType normalizeScreenType(uint8_t raw) {
  switch (raw) {
    case SCREEN_HAM_CLOCK:
    case SCREEN_DX_CLUSTER:
    case SCREEN_SUN_SPOTS:
    case SCREEN_BAND_INFO:
    case SCREEN_WEATHER_DSP:
    case SCREEN_WEATHER_FORECAST:
    case SCREEN_APRS_IS:
    case SCREEN_APRS_RADAR:
    case SCREEN_POTA_CLUSTER:
    case SCREEN_HAMALERT_CLUSTER:
    case SCREEN_PSK_MAP:
    case SCREEN_MATRIX_CLOCK:
    case SCREEN_UNLIS_HUNTER:
    case SCREEN_ISS_PASS_TRACKING:
      return (ScreenType)raw;
    default:
      return SCREEN_OFF;
  }
}

const char* screenTypeToCodeStr(ScreenType t) {
  switch (t) {
    case SCREEN_HAM_CLOCK: return "hamclock";
    case SCREEN_DX_CLUSTER: return "dxcluster";
    case SCREEN_APRS_IS: return "aprsis";
    case SCREEN_APRS_RADAR: return "aprsradar";
    case SCREEN_BAND_INFO: return "solarindex";
    case SCREEN_SUN_SPOTS: return "propagacja";
    case SCREEN_WEATHER_DSP: return "weather";
    case SCREEN_WEATHER_FORECAST: return "weatherforecast";
    case SCREEN_POTA_CLUSTER: return "pota";
    case SCREEN_HAMALERT_CLUSTER: return "hamalert";
    case SCREEN_PSK_MAP: return "pskmap";
    case SCREEN_MATRIX_CLOCK: return "matrix";
    case SCREEN_UNLIS_HUNTER: return "unlishunter";
    case SCREEN_ISS_PASS_TRACKING: return "isspasstracking";
    case SCREEN_OFF:
    default:
      return "off";
  }
}

ScreenType screenCodeToType(const String &code) {
  String c = code;
  c.toLowerCase();
  c.trim();
  if (c == "hamclock") return SCREEN_HAM_CLOCK;
  if (c == "dxcluster") return SCREEN_DX_CLUSTER;
  if (c == "aprsis") return SCREEN_APRS_IS;
  if (c == "aprsradar") return SCREEN_APRS_RADAR;
  if (c == "bandinfo" || c == "solarindex" || c == "solar indeks") return SCREEN_BAND_INFO;
  if (c == "sunspots" || c == "propagacja") return SCREEN_SUN_SPOTS;
  if (c == "weather") return SCREEN_WEATHER_DSP;
  if (c == "weatherforecast" || c == "forecast") return SCREEN_WEATHER_FORECAST;
  if (c == "pota") return SCREEN_POTA_CLUSTER;
  if (c == "hamalert") return SCREEN_HAMALERT_CLUSTER;
  if (c == "pskmap" || c == "pskreporter") return SCREEN_PSK_MAP;
  if (c == "matrix") return SCREEN_MATRIX_CLOCK;
  if (c == "unlishunter") return SCREEN_UNLIS_HUNTER;
  if (c == "isspasstracking" || c == "iss") return SCREEN_ISS_PASS_TRACKING;
  return SCREEN_OFF;
}

void loadDefaultScreenOrder() {
  for (int i = 0; i < SCREEN_ORDER_COUNT; i++) {
    screenOrder[i] = DEFAULT_SCREEN_ORDER[i];
  }
}

void ensureScreenOrderValid() {
  bool hasActive = false;
  bool seen[15] = {false}; // indeksy odpowiadają ScreenType wartościom (0..14)
  for (int i = 0; i < SCREEN_ORDER_COUNT; i++) {
    screenOrder[i] = normalizeScreenType(screenOrder[i]);
    ScreenType t = screenOrder[i];
    if (t != SCREEN_OFF) {
      if (t >= 0 && t <= SCREEN_ISS_PASS_TRACKING) {
        if (seen[t]) {
          // drugi raz ten sam ekran – usuń duplikat
          screenOrder[i] = SCREEN_OFF;
          continue;
        }
        seen[t] = true;
      }
      hasActive = true;
    }
  }
  if (!hasActive) {
    loadDefaultScreenOrder();
  }
}

int findScreenOrderIndex(ScreenType screenId) {
  for (int i = 0; i < SCREEN_ORDER_COUNT; i++) {
    if (screenOrder[i] == screenId) {
      return i;
    }
  }
  return 0;
}

ScreenType firstActiveScreen() {
  for (int i = 0; i < SCREEN_ORDER_COUNT; i++) {
    if (screenOrder[i] != SCREEN_OFF) {
      return screenOrder[i];
    }
  }
  return SCREEN_HAM_CLOCK;
}

ScreenType getNextScreenId(ScreenType currentId) {
  int idx = findScreenOrderIndex(currentId);
  for (int step = 1; step <= SCREEN_ORDER_COUNT; step++) {
    ScreenType candidate = screenOrder[(idx + step) % SCREEN_ORDER_COUNT];
    if (candidate != SCREEN_OFF) {
      return candidate;
    }
  }
  return SCREEN_HAM_CLOCK;
}

ScreenType getPrevScreenId(ScreenType currentId) {
  int idx = findScreenOrderIndex(currentId);
  for (int step = 1; step <= SCREEN_ORDER_COUNT; step++) {
    int prev = idx - step;
    if (prev < 0) {
      prev += SCREEN_ORDER_COUNT;
    }
    ScreenType candidate = screenOrder[prev];
    if (candidate != SCREEN_OFF) {
      return candidate;
    }
  }
  return SCREEN_HAM_CLOCK;
}

static int normalizeTftAutoSwitchTimeSec(int seconds) {
  if (seconds < 1) return 1;
  if (seconds > 3600) return 3600;
  return seconds;
}

static void applyTftAutoSwitchTimeSec(int seconds) {
  tftAutoSwitchTimeSec = normalizeTftAutoSwitchTimeSec(seconds);
}

static void resetTftAutoSwitchTimer() {
  tftAutoSwitchLastMs = millis();
  tftAutoSwitchLastScreen = currentScreen;
}

void drawQrzPopup();
void handleQrzPopupTouch(uint16_t x, uint16_t y);
void openQrzPopup(const String &callsign);

void handleTouchNavigation() {
  if (!tftInitialized) {
    return;
  }

  const uint16_t menuHitW = 50;
  const uint16_t menuHitH = 50;

  static bool touchActive = false;
  static unsigned long lastTouchMs = 0;
  static unsigned long touchStartMs = 0;
  static uint16_t touchStartX = 0;
  static uint16_t touchStartY = 0;
  static bool longPressHandled = false;
  static bool matrixGameHoldCandidate = false;
  unsigned long now = millis();

  uint16_t x = 0;
  uint16_t y = 0;
  int16_t rawX = 0;
  int16_t rawY = 0;
  if (readRawTouchPoint(rawX, rawY)) {
    // Jeśli ekran jest w trybie uśpienia - wybudź go
    if (screenSleepActive) {
      wakeUpFromSleep();
      return;
    }
    mapRawToScreen(rawX, rawY, x, y);
    resetScreenSaverActivity();  // Reset wygaszacza przy aktywności użytkownika
    // Reset timera uśpienia przy aktywności
    screenSleepLastActivityMs = millis();
    bool isNewTap = false;
    if (!touchActive) {
      if ((now - lastTouchMs) <= 150) {
        return;
      }
      touchActive = true;
      lastTouchMs = now;
      touchStartMs = now;
      touchStartX = x;
      touchStartY = y;
      longPressHandled = false;
      matrixGameHoldCandidate = (currentScreen == SCREEN_MATRIX_CLOCK && !inMenu && x >= 320 && y <= 80);
      isNewTap = true;
    }

    if (touchCalActive) {
      handleTouchCalibrationTouch(rawX, rawY, x, y, isNewTap);
      return;
    }

    if (aprsAlertScreenActive) {
      if (isNewTap && isAprsAlertCloseButtonHit(x, y)) {
        dismissAprsAlertScreen();
      }
      return;
    }

    // Obsługa wygaszacza ekranu - dotknięcie wchodzi w menu ustawień
    if (screenSaverActive) {
      if (isNewTap) {
        resetScreenSaverActivity();
        inMenu = true;
        screenSaverMenuActive = true;
        drawScreenSaverMenu();
      }
      return;
    }

    if (!brightnessMenuActive && !longPressHandled && matrixGameHoldCandidate) {
      if (currentScreen == SCREEN_MATRIX_CLOCK && !inMenu && (now - touchStartMs) >= 2000) {
        currentScreen = SCREEN_UNLIS_HUNTER;
        drawScreen(currentScreen);
        resetTftAutoSwitchTimer();
        longPressHandled = true;
        matrixGameHoldCandidate = false;
        return;
      }
    }

    if (!brightnessMenuActive && !longPressHandled) {
      if (now - touchStartMs >= 3000) {
        brightnessMenuPrevScreen = currentScreen;
        brightnessMenuPrevInMenu = inMenu;
        brightnessMenuPrevBacklight = backlightPercent;
        brightnessMenuPrevThemeHue = menuThemeHue;
        brightnessMenuActive = true;
        brightnessMenuValue = backlightPercent;
        brightnessMenuOpenedMs = now;
        inMenu = true;
        drawBrightnessMenu();
        longPressHandled = true;
        return;
      }
    }

    if (brightnessMenuActive) {
      handleBrightnessMenuTouch(x, y, isNewTap);
      return;
    }

    if (!isNewTap) {
      return;
    }

    if (currentScreen == SCREEN_UNLIS_HUNTER) {
      if (x >= UNLIS_EXIT_X && x < (UNLIS_EXIT_X + UNLIS_BTN_SIZE) &&
          y >= UNLIS_EXIT_Y && y < (UNLIS_EXIT_Y + UNLIS_BTN_SIZE)) {
        currentScreen = SCREEN_MATRIX_CLOCK;
        drawScreen(currentScreen);
        resetTftAutoSwitchTimer();
        return;
      }
      if (x >= UNLIS_START_X && x < (UNLIS_START_X + UNLIS_BTN_SIZE) &&
          y >= UNLIS_START_Y && y < (UNLIS_START_Y + UNLIS_BTN_SIZE)) {
        if (unlisRunning && !unlisGameOver) {
          unlisStopGame();
        } else {
          unlisStartResetGame();
        }
        return;
      }
      if (x >= UNLIS_PTT_X && x < (UNLIS_PTT_X + UNLIS_BTN_SIZE) &&
          y >= UNLIS_PTT_Y && y < (UNLIS_PTT_Y + UNLIS_BTN_SIZE)) {
        unlisHandlePttPress(now);
        return;
      }
      return;
    }

    if (inMenu) {
      if (screenSaverMenuActive) {
        handleScreenSaverMenuTouch(x, y);
        return;
      }
      if (screenSleepMenuActive) {
        handleScreenSleepMenuTouch(x, y);
        return;
      }
      if (currentScreen == SCREEN_HAM_CLOCK) {
        handleScreen1MenuTouch(x, y);
      }
      if (currentScreen == SCREEN_DX_CLUSTER) {
        handleScreen2MenuTouch(x, y);
      }
      if (currentScreen == SCREEN_POTA_CLUSTER) {
        handleScreen7MenuTouch(x, y);
      }
      if (currentScreen == SCREEN_HAMALERT_CLUSTER) {
        handleScreen8MenuTouch(x, y);
      }
      if (currentScreen == SCREEN_APRS_IS || currentScreen == SCREEN_APRS_RADAR) {
        handleScreen6MenuTouch(x, y);
      }
      if (currentScreen == SCREEN_WEATHER_DSP || currentScreen == SCREEN_WEATHER_FORECAST) {
        handleScreen5MenuTouch(x, y);
      }
      return;
    }

    if (qrzPopupActive) {
      handleQrzPopupTouch(x, y);
      return;
    }

    if (currentScreen == SCREEN_DX_CLUSTER) {
      if (x < menuHitW && y < menuHitH) {
          inMenu = true;
          drawDxClusterFilterMenu();
          return;
        }
      // Kliknięcie w znak (callsign) - sprawdź czy dotknięto wiersza z spotem
      if (y >= 40 && y < 280 && x >= 50 && x <= 200) {
        // Oblicz który wiersz został dotknięty
        int rowHeight = isDxTableEnlarged() ? 27 : 20;
        int headerOffset = 40;
        int clickedRow = (y - headerOffset) / rowHeight;
        if (clickedRow >= 0) {
          // Znajdź spot w tablicy
          lockDxSpots();
          int displayCount = 0;
          for (int i = 0; i < spotCount; i++) {
            if (!spotMatchesScreen2Filters(spots[i])) continue;
            if (displayCount == clickedRow) {
              // Kliknięto w ten spot
              String callsign = spots[i].callsign;
              unlockDxSpots();
              openQrzPopup(callsign);
              return;
            }
            displayCount++;
          }
          unlockDxSpots();
        }
      }
    }
    if (currentScreen == SCREEN_POTA_CLUSTER) {
        if (x < menuHitW && y < menuHitH) {
          inMenu = true;
          drawPotaFilterMenu();
          return;
        }
      // Kliknięcie w znak (callsign) - sprawdź czy dotknięto wiersza z spotem
      if (y >= 40 && y < 280 && x >= 50 && x <= 200) {
        // Oblicz który wiersz został dotknięty
        int rowHeight = isDxTableEnlarged() ? 27 : 20;
        int headerOffset = 40;
        int clickedRow = (y - headerOffset) / rowHeight;
        if (clickedRow >= 0) {
          // Znajdź spot w tablicy POTA
          int displayCount = 0;
          for (int i = 0; i < potaSpotCount; i++) {
            if (!spotMatchesScreen7Filters(potaSpots[i])) continue;
            if (displayCount == clickedRow) {
              // Kliknięto w ten spot
              String callsign = potaSpots[i].callsign;
              openQrzPopup(callsign);
              return;
            }
            displayCount++;
          }
        }
      }
    }
    if (currentScreen == SCREEN_HAMALERT_CLUSTER) {
        if (x < menuHitW && y < menuHitH) {
          inMenu = true;
          drawHamalertFilterMenu();
          return;
        }
      }
      if (currentScreen == SCREEN_APRS_IS) {
      if (x < menuHitW && y < menuHitH) {
        inMenu = true;
        screen6MenuBeaconingTemp = aprsBeaconEnabled;
        screen6MenuAprsAlertTemp = aprsAlertEnabled;
        screen6MenuRangeAlertTemp = aprsAlertNearbyEnabled;
        screen6MenuLedAlertTemp = enableLedAlert;
        drawAprsSortMenu();
        return;
      }
    }
      if (currentScreen == SCREEN_APRS_RADAR) {
      if (isScreen6RadarZoomTopHit(x, y)) {
        if ((now - screen6RadarLastZoomTapMs) < SCREEN6_RADAR_ZOOM_TAP_COOLDOWN_MS) {
          return;
        }
        screen6RadarLastZoomTapMs = now;
        float newZoom = min(SCREEN6_RADAR_ZOOM_MAX, screen6RadarZoom + SCREEN6_RADAR_ZOOM_STEP);
        if (newZoom > screen6RadarZoom + 0.0001f) {
          screen6RadarZoom = newZoom;
          triggerScreen6RadarHint();
          drawScreen(SCREEN_APRS_RADAR);
          resetTftAutoSwitchTimer();
        }
        return;
      }
      if (isScreen6RadarZoomBottomHit(x, y)) {
        if ((now - screen6RadarLastZoomTapMs) < SCREEN6_RADAR_ZOOM_TAP_COOLDOWN_MS) {
          return;
        }
        screen6RadarLastZoomTapMs = now;
        float newZoom = max(SCREEN6_RADAR_ZOOM_MIN, screen6RadarZoom - SCREEN6_RADAR_ZOOM_STEP);
        if (newZoom < screen6RadarZoom - 0.0001f) {
          screen6RadarZoom = newZoom;
          triggerScreen6RadarHint();
          drawScreen(SCREEN_APRS_RADAR);
          resetTftAutoSwitchTimer();
        }
        return;
      }
      if (x < menuHitW && y < menuHitH) {
        inMenu = true;
        screen6MenuBeaconingTemp = aprsBeaconEnabled;
        screen6MenuAprsAlertTemp = aprsAlertEnabled;
        screen6MenuRangeAlertTemp = aprsAlertNearbyEnabled;
        screen6MenuLedAlertTemp = enableLedAlert;
        drawAprsSortMenu();
        return;
      }
    }
    if (currentScreen == SCREEN_HAM_CLOCK) {
      if (x > 285 && y < 35) {
        inMenu = true;
        drawHamClockTimeMenu();
        return;
      }
    }
    if (currentScreen == SCREEN_WEATHER_DSP || currentScreen == SCREEN_WEATHER_FORECAST) {
      if (x < menuHitW && y < menuHitH) {
        inMenu = true;
        drawWeatherMenu();
        return;
      }
    }

    // Nawigacja: dolne obszary dotyku (ok. 80x180)
    // Zamienione kierunki: lewa strona = w prawo (next), prawa strona = w lewo (prev)
    // TYLKO gdy nie jesteśmy w żadnym menu i nie ma aktywnej klawiatury
    if (!inMenu && !brightnessMenuActive && !screenSaverMenuActive && !screenSleepMenuActive && !touchCalActive && !pskKeyboardActive) {
      const uint16_t cornerY = 60;
      const uint16_t cornerX = 80;
      if (y >= cornerY && x < cornerX) {
        currentScreen = getNextScreenId(currentScreen);
        drawScreen(currentScreen);
        resetTftAutoSwitchTimer();
        return;
      } else if (y >= cornerY && x >= (480 - cornerX)) {
        currentScreen = getPrevScreenId(currentScreen);
        drawScreen(currentScreen);
        resetTftAutoSwitchTimer();
        return;
      }
    }

    // Obsługa PSK map - na końcu, aby nie blokować nawigacji w rogach
    if (currentScreen == SCREEN_PSK_MAP) {
      handlePskMapTouch(x, y);
      return;
    }
  } else {
    touchActive = false;
    longPressHandled = false;
    matrixGameHoldCandidate = false;
    // Reset liczników long-press dla menu jasności po puszczeniu dotyku
    if (brightnessMenuActive) {
      brightnessMenuTouchStartMs = 0;
      brightnessMenuLongPressHandled = false;
    }
  }
}
// Ekran 2: DX Cluster
void drawDxCluster() {
  tft.fillScreen(TFT_BLACK);

  // 1. NAGÄąÂÄ‚â€śWEK: Belka z menu, nazwĂ„â€¦ klastra i czasem UTC
  tft.fillRect(0, 0, 480, 32, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);

// IKONA MENU (3D)
drawHamburgerMenuButton3D(5, 7);

// Nazwa klastra - przesunięta w prawo (x=35 zamiast 5), by zrobić miejsce na menu
tft.setTextSize(2);
tft.setCursor(35, 8);
tft.print(clusterHost);

  // Ikona WiFi w górnym prawym rogu
  drawWifiSignalBars(445, 4);

  // ZEGAR - rysowany osobno przez updateDxClusterClock()

  // 2. NAGŁÓWKI TABELI
  const bool enlarged = isDxTableEnlarged();
  int yPos = 40;
  tft.setTextColor(TFT_DARKGREY);
  tft.setTextSize(1);

  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("CZAS");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(204, yPos); tft.print("MHz");
    tft.setCursor(330, yPos); tft.print("MODE");
  } else {
    tft.setCursor(5, yPos);   tft.print("CZAS");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(140, yPos); tft.print("MHz");
    tft.setCursor(260, yPos); tft.print("MODE");
    tft.setCursor(360, yPos); tft.print(sanitizePolishToAscii(String(tr(TR_COUNTRY))));
  }
  tft.drawFastHLine(0, yPos + 10, 480, TFT_DARKGREY);
  yPos += enlarged ? 20 : 18;

  // 3. LISTA SPOTÄ‚â€śW
  int maxRows = getDxTableMaxRows();
  int displayCount = 0;
  lockDxSpots();
  for (int i = 0; i < spotCount && displayCount < maxRows; i++) {
    if (!spotMatchesScreen2Filters(spots[i])) {
      continue;
    }
    if (enlarged) {
      if (displayCount % 2 == 0) tft.fillRect(0, yPos - 5, 480, 24, TFT_TABLE_ALT_ROW_COLOR);
      tft.setTextSize(2);
      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      tft.print(formatSpotUtc(spots[i].time));

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(74, yPos);
      String callText = spots[i].callsign;
      if (callText.length() > 8) callText = callText.substring(0, 8);
      tft.print(callText);

      tft.setTextColor(TFT_YELLOW);
      tft.setCursor(204, yPos);
      tft.print(spots[i].frequency / 1000.0, 3);

      tft.setTextColor(TFT_GREEN);
      tft.setCursor(330, yPos);
      String modeText = spots[i].mode;
      if (modeText.length() > 4) modeText = modeText.substring(0, 4);
      tft.print(modeText);
      yPos += 27;
    } else {
      if (displayCount % 2 == 0) tft.fillRect(0, yPos - 2, 480, 16, TFT_TABLE_ALT_ROW_COLOR);

      // Znak wywoławczy - rozmiar 1 (standardowy)
      tft.setTextSize(1);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setCursor(50, yPos);
      String callStr = spots[i].callsign;
      callStr.toUpperCase();
      if (callStr.length() > 8) callStr = callStr.substring(0, 8);
      tft.print(callStr);
      
      // Częstotliwość - rozmiar 1
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
      tft.setCursor(140, yPos);
      tft.print(spots[i].frequency / 1000.0, 3);
      
      // Mode - rozmiar 1
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setCursor(260, yPos);
      String modeStr = spots[i].mode;
      if (modeStr.length() > 4) modeStr = modeStr.substring(0, 4);
      tft.print(modeStr);
      
      // Czas i kraj - rozmiar 1 (dopasowane do nagłówków)
      tft.setTextSize(1);
      tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
      tft.setCursor(5, yPos + 5);
      tft.print(formatSpotUtc(spots[i].time));
      
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.setCursor(360, yPos + 5);
      tft.print(formatDistanceOrCountry(spots[i], COUNTRY_COL_MAX_LEN));

      yPos += 20;
    }
    displayCount++;
  }
  unlockDxSpots();

  // 4. Pasek nawigacji - ZAWSZE widoczny (zmienione z 5 sekund)
  drawSwitchScreenFooter();

  if (displayCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR SPOTS...");
  }
  drawSwitchScreenFooter(); // Dodano rysowanie strzałek nawigacyjnych
}

bool isPointInRect(uint16_t x, uint16_t y, int rx, int ry, int rw, int rh) {
  return (x >= rx && x <= (rx + rw) && y >= ry && y <= (ry + rh));
}

void drawHamburgerMenuButton3D(int x, int y) {
  const int iconW = 20;
  const int iconH = 17;
  const int pad = 3;
  const int btnX = x - pad;
  const int btnY = y - pad;
  const int btnW = iconW + (pad * 2);
  const int btnH = iconH + (pad * 2);
  const uint16_t edgeLight = lerpColor565(TFT_WHITE, TFT_RADIO_ORANGE, 140);
  const uint16_t edgeShadow = lerpColor565(TFT_BLACK, TFT_RADIO_ORANGE, 115);

  tft.fillRoundRect(btnX, btnY, btnW, btnH, 3, TFT_RADIO_ORANGE);
  tft.drawRoundRect(btnX, btnY, btnW, btnH, 3, edgeLight);
  tft.drawLine(btnX + 1, btnY + btnH - 1, btnX + btnW - 2, btnY + btnH - 1, edgeShadow);
  tft.drawLine(btnX + btnW - 1, btnY + 1, btnX + btnW - 1, btnY + btnH - 2, edgeShadow);

  tft.fillRect(x, y, iconW, 3, TFT_BLACK);
  tft.fillRect(x, y + 7, iconW, 3, TFT_BLACK);
  tft.fillRect(x, y + 14, iconW, 3, TFT_BLACK);
}

static void drawScreen6ViewToggleButton3D(int x, int y, Screen6ViewMode mode) {
  const int iconW = 20;
  const int iconH = 17;
  const int pad = 3;
  const int btnX = x - pad;
  const int btnY = y - pad;
  const int btnW = iconW + (pad * 2);
  const int btnH = iconH + (pad * 2);
  const uint16_t edgeLight = lerpColor565(TFT_WHITE, TFT_RADIO_ORANGE, 140);
  const uint16_t edgeShadow = lerpColor565(TFT_BLACK, TFT_RADIO_ORANGE, 115);

  tft.fillRoundRect(btnX, btnY, btnW, btnH, 3, TFT_RADIO_ORANGE);
  tft.drawRoundRect(btnX, btnY, btnW, btnH, 3, edgeLight);
  tft.drawLine(btnX + 1, btnY + btnH - 1, btnX + btnW - 2, btnY + btnH - 1, edgeShadow);
  tft.drawLine(btnX + btnW - 1, btnY + 1, btnX + btnW - 1, btnY + btnH - 2, edgeShadow);

  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(x + 6, y + 1);
  tft.print(mode == APRS_VIEW_RADAR ? "L" : "R");
}

static bool isScreen6ViewToggleHit(uint16_t x, uint16_t y) {
  return (x >= SCREEN6_VIEW_BTN_HIT_X &&
          x <= (SCREEN6_VIEW_BTN_HIT_X + SCREEN6_VIEW_BTN_HIT_W) &&
          y >= SCREEN6_VIEW_BTN_HIT_Y &&
          y <= (SCREEN6_VIEW_BTN_HIT_Y + SCREEN6_VIEW_BTN_HIT_H));
}

static bool isScreen6RadarZoomTopHit(uint16_t x, uint16_t y) {
  if (currentScreen != SCREEN_APRS_RADAR) {
    return false;
  }
  int w = tft.width();
  int h = tft.height();
  int xMin = w / 4;
  int xMax = (3 * w) / 4;
  return (x >= xMin && x <= xMax && y < (h / 2));
}

static bool isScreen6RadarZoomBottomHit(uint16_t x, uint16_t y) {
  if (currentScreen != SCREEN_APRS_RADAR) {
    return false;
  }
  int w = tft.width();
  int h = tft.height();
  int xMin = w / 4;
  int xMax = (3 * w) / 4;
  return (x >= xMin && x <= xMax && y >= (h / 2));
}

static void triggerScreen6RadarHint() {
  screen6RadarHintUntilMs = millis() + SCREEN6_RADAR_HINT_DURATION_MS;
}

static bool isScreen6RadarHintVisible() {
  return (currentScreen == SCREEN_APRS_RADAR && millis() < screen6RadarHintUntilMs);
}

static void drawScreen6RadarZoomHints() {
  if (!isScreen6RadarHintVisible()) {
    return;
  }

  const int screenW = tft.width();
  const int screenH = tft.height();
  const char *topHint = "ZOOM +";
  const char *bottomHint = "ZOOM -";
  const int topW = (int)strlen(topHint) * 12;
  const int bottomW = (int)strlen(bottomHint) * 12;

  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor((screenW - topW) / 2, 6);
  tft.print(topHint);
  tft.setCursor((screenW - bottomW) / 2, screenH - 20);
  tft.print(bottomHint);

  const uint16_t radarAlertColor = rgb565(70, 130, 190);
  int zoomPercent = (int)(screen6RadarZoom * 100.0f + 0.5f);
  float radarVisibleMaxKm = (screen6RadarZoom > 0.0f)
                              ? ((float)aprsFilterRadius / screen6RadarZoom)
                              : (float)aprsFilterRadius;

  String zoomInfo = String("ZOOM:") + String(zoomPercent) + "%";
  String rmaxInfo = String("Rmax:") + String(radarVisibleMaxKm, 1) + "km";
  String alertInfo = String("ALERT:") + String(aprsAlertDistanceKm, 1) + "km";

  const int infoX = 2;
  const int zoomY = screenH - 29;
  const int rmaxY = screenH - 19;
  const int alertY = screenH - 9;

  tft.setTextSize(1);

  int zoomW = (int)zoomInfo.length() * 6;
  int rmaxW = (int)rmaxInfo.length() * 6;
  int alertW = (int)alertInfo.length() * 6;

  tft.fillRect(infoX - 1, zoomY - 1, zoomW + 2, 10, TFT_BLACK);
  tft.fillRect(infoX - 1, rmaxY - 1, rmaxW + 2, 10, TFT_BLACK);
  tft.fillRect(infoX - 1, alertY - 1, alertW + 2, 10, TFT_BLACK);

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(infoX, zoomY);
  tft.print(zoomInfo);

  tft.setCursor(infoX, rmaxY);
  tft.print(rmaxInfo);

  tft.setTextColor(radarAlertColor);
  tft.setCursor(infoX, alertY);
  tft.print(alertInfo);
}

static uint16_t getAprsRadarColorForStation(const APRSStation &station) {
  String symbolShort = getAPRSSymbolShort(station);
  if (symbolShort == "CAR") return TFT_RED;
  if (symbolShort == "HOUSE") return TFT_YELLOW;
  if (symbolShort == "HUMAN") return TFT_BLUE;
  return TFT_GREEN;
}

static void drawAprsRadarBody() {
  const int screenW = tft.width();
  const int screenH = tft.height();
  tft.fillRect(0, 0, screenW, screenH, TFT_BLACK);

  const int centerX = screenW / 2;
  const int centerY = screenH / 2;
  const int pad = 6;
  int baseOuterRadius = min(min(centerX - pad, screenW - centerX - pad),
                            min(centerY - pad, screenH - centerY - pad));
  if (baseOuterRadius < 20) {
    baseOuterRadius = 20;
  }

  int outerRadius = (int)(baseOuterRadius * screen6RadarZoom);
  if (outerRadius < 20) {
    outerRadius = 20;
  }

  const int fixedOuterRadius = baseOuterRadius; // nowy, stały wizualnie pierścień 100%
  const int fixedHalfRadius = max(1, fixedOuterRadius / 2); // nowy, stały wizualnie pierścień 50%

  const float zoomSafe = (screen6RadarZoom > 0.01f) ? screen6RadarZoom : 0.01f;
  const float fixedOuterKm = (aprsFilterRadius > 0)
                              ? ((float)aprsFilterRadius / zoomSafe)
                              : 0.0f;

  const uint16_t radarCircleBgColor = TFT_BLACK;
  const uint16_t radarAlertColor = rgb565(70, 130, 190);
  const uint16_t ringColor = TFT_DARKGREY;
  const int innerRadius = (aprsFilterRadius > 0)
                            ? (int)((aprsAlertDistanceKm / (float)aprsFilterRadius) * outerRadius)
                            : 0;
  const int clampedInnerRadius = constrain(innerRadius, 1, outerRadius);

  if (outerRadius <= baseOuterRadius + 2) {
    tft.fillCircle(centerX, centerY, outerRadius - 1, radarCircleBgColor);
  }

  // Dodane stałe pierścienie referencyjne na spodzie (stacje są rysowane później, nad nimi)
  tft.drawCircle(centerX, centerY, fixedOuterRadius, ringColor); // 100%
  tft.drawCircle(centerX, centerY, fixedHalfRadius, ringColor);  // 50%

  // Oryginalne okręgi radaru
  tft.drawCircle(centerX, centerY, outerRadius, TFT_WHITE);
  tft.drawCircle(centerX, centerY, clampedInnerRadius, radarAlertColor);

  tft.drawFastVLine(centerX, centerY - outerRadius, outerRadius * 2, 0x39C7);
  tft.drawFastHLine(centerX - outerRadius, centerY, outerRadius * 2, 0x39C7);

  auto kmLabel = [](float km) -> String {
    if (km >= 100.0f) return String((int)(km + 0.5f)) + "km";
    if (km >= 10.0f) return String(km, 1) + "km";
    return String(km, 2) + "km";
  };

  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY);
  String ring100Text = kmLabel(fixedOuterKm);
  String ring50Text = kmLabel(fixedOuterKm * 0.5f);

  int ringLabelX = centerX + 4;
  int ring100Y = max(0, centerY - fixedOuterRadius + 2);
  int ring50Y = max(0, centerY - fixedHalfRadius + 2);

  tft.fillRect(ringLabelX - 1, ring100Y - 1, (int)ring100Text.length() * 6 + 2, 10, TFT_BLACK);
  tft.fillRect(ringLabelX - 1, ring50Y - 1, (int)ring50Text.length() * 6 + 2, 10, TFT_BLACK);
  tft.setCursor(ringLabelX, ring100Y);
  tft.print(ring100Text);
  tft.setCursor(ringLabelX, ring50Y);
  tft.print(ring50Text);

  tft.fillCircle(centerX, centerY, 4, TFT_RADIO_ORANGE);

  int order[MAX_APRS_DISPLAY_LCD];
  int displayCount = 0;
  buildAprsDisplayOrder(order, displayCount);
  int maxRows = min(displayCount, MAX_APRS_DISPLAY_LCD);

  for (int i = 0; i < maxRows; i++) {
    const APRSStation &station = aprsStations[order[i]];
    if (!station.hasLatLon || station.distance <= 0.0f) {
      continue;
    }
    if (aprsFilterRadius <= 0 || station.distance > (float)aprsFilterRadius) {
      continue;
    }

    float bearingDeg = calculateBearing(userLat, userLon, (double)station.lat, (double)station.lon);
    float angleRad = bearingDeg * (float)M_PI / 180.0f;
    float scaledRadius = (station.distance / (float)aprsFilterRadius) * outerRadius;
    if (scaledRadius > outerRadius) {
      continue;
    }
    int px = centerX + (int)(sin(angleRad) * scaledRadius);
    int py = centerY - (int)(cos(angleRad) * scaledRadius);

    uint16_t color = getAprsRadarColorForStation(station);
    tft.fillCircle(px, py, 3, color);

    String call = station.callsign;
    if (call.length() > 8) {
      call = call.substring(0, 8);
    }
    int tx = px - ((int)call.length() * 3);
    int ty = py - 10;
    if (tx < 1) tx = 1;
    if (tx > screenW - ((int)call.length() * 6) - 1) tx = screenW - ((int)call.length() * 6) - 1;
    if (ty < 2) ty = py + 6;
    if (ty > screenH - 9) ty = screenH - 9;

    tft.setTextColor(color);
    tft.setTextSize(1);
    tft.setCursor(tx, ty);
    tft.print(call);

    if ((i & 0x03) == 0) {
      yield();
    }
  }

}

void drawFilterTile(int x, int y, int w, int h, const char *label, bool active) {
  uint16_t fill = active ? TFT_RADIO_ORANGE : TFT_DARKGREY;
  uint16_t text = active ? TFT_BLACK : TFT_WHITE;
  tft.fillRect(x, y, w, h, fill);
  tft.drawRect(x, y, w, h, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(text);
  int textW = strlen(label) * 6;
  int textX = x + (w - textW) / 2;
  int textY = y + (h - 8) / 2;
  tft.setCursor(textX, textY);
  tft.print(label);
}

static void drawScreen6MenuCheckButton(int x, int y, const char *label, bool checked) {
  const int boxSize = 18;
  tft.drawRect(x, y, boxSize, boxSize, TFT_WHITE);
  if (checked) {
    tft.fillRect(x + 4, y + 4, boxSize - 8, boxSize - 8, TFT_RADIO_ORANGE);
  } else {
    tft.fillRect(x + 1, y + 1, boxSize - 2, boxSize - 2, TFT_BLACK);
  }

  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(x + boxSize + 10, y + 2);
  tft.print(label);
}

void drawDxClusterFilterMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(3);
  tft.setCursor(10, 8);
  tft.print("FILTERS");

  tft.setTextSize(2);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(10, 45);
  tft.print("MODE");

  const int modeTileW = 100;
  const int modeTileH = 40;
  const int modeGap = 10;
  const int modeStartX = (480 - (4 * modeTileW + 3 * modeGap)) / 2;
  const int modeY = 70;
  drawFilterTile(modeStartX + 0 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "ALL", screen2FilterMode == FILTER_MODE_ALL);
  drawFilterTile(modeStartX + 1 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "CW", screen2FilterMode == FILTER_MODE_CW);
  drawFilterTile(modeStartX + 2 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "SSB", screen2FilterMode == FILTER_MODE_SSB);
  drawFilterTile(modeStartX + 3 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "DIGI", screen2FilterMode == FILTER_MODE_DIGI);

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(10, 125);
  tft.print("BAND");

  const int bandTileW = 100;
  const int bandTileH = 32;
  const int bandGap = 8;
  const int bandStartX = (480 - (4 * bandTileW + 3 * bandGap)) / 2;
  const int bandStartY = 140;
  int bandIdx = 0;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 4; col++) {
      if (bandIdx >= SCREEN8_FILTER_BANDS_COUNT) break;
      int x = bandStartX + col * (bandTileW + bandGap);
      int y = bandStartY + row * (bandTileH + bandGap);
      drawFilterTile(x, y, bandTileW, bandTileH, SCREEN8_FILTER_BANDS[bandIdx], screen2FilterBandIndex == bandIdx);
      bandIdx++;
    }
  }

  const int closeW = 140;
  const int closeH = 40;
  const int closeX = (480 - closeW) / 2;
  const int closeY = 275;
  drawFilterTile(closeX, closeY, closeW, closeH, "CLOSE", false);
}

void handleScreen2MenuTouch(uint16_t x, uint16_t y) {
  const int modeTileW = 100;
  const int modeTileH = 40;
  const int modeGap = 10;
  const int modeStartX = (480 - (4 * modeTileW + 3 * modeGap)) / 2;
  const int modeY = 70;

  for (int i = 0; i < 4; i++) {
    int rx = modeStartX + i * (modeTileW + modeGap);
    if (isPointInRect(x, y, rx, modeY, modeTileW, modeTileH)) {
      screen2FilterMode = (Screen2FilterMode)i;
      drawDxClusterFilterMenu();
      return;
    }
  }

  const int bandTileW = 100;
  const int bandTileH = 32;
  const int bandGap = 8;
  const int bandStartX = (480 - (4 * bandTileW + 3 * bandGap)) / 2;
  const int bandStartY = 140;
  int bandIdx = 0;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 4; col++) {
      if (bandIdx >= SCREEN8_FILTER_BANDS_COUNT) break;
      int rx = bandStartX + col * (bandTileW + bandGap);
      int ry = bandStartY + row * (bandTileH + bandGap);
      if (isPointInRect(x, y, rx, ry, bandTileW, bandTileH)) {
        screen2FilterBandIndex = bandIdx;
        drawDxClusterFilterMenu();
        return;
      }
      bandIdx++;
    }
  }

  const int closeW = 140;
  const int closeH = 40;
  const int closeX = (480 - closeW) / 2;
  const int closeY = 275;
  if (isPointInRect(x, y, closeX, closeY, closeW, closeH)) {
    inMenu = false;
    drawScreen(SCREEN_DX_CLUSTER);
    return;
  }
}

void drawAprsSortMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 4);
  tft.print("SORT BY:");

  const int tileW = 90;
  const int tileH = 26;
  const int gap = 6;
  const int startX = (480 - (3 * tileW + 2 * gap)) / 2;
  const int tileY = 32;
  drawFilterTile(startX + 0 * (tileW + gap), tileY, tileW, tileH, "TIME", screen6SortMode == APRS_SORT_TIME);
  drawFilterTile(startX + 1 * (tileW + gap), tileY, tileW, tileH, "CALL", screen6SortMode == APRS_SORT_CALLSIGN);
  drawFilterTile(startX + 2 * (tileW + gap), tileY, tileW, tileH, "DIST", screen6SortMode == APRS_SORT_DISTANCE);

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(2);
  tft.setCursor(10, 74);
  tft.print("APRS Setting:");

  const int checkX = 34;
  const int checkRowY0 = 94;
  const int checkRowGap = 28;
  drawScreen6MenuCheckButton(checkX, checkRowY0 + 0 * checkRowGap, "Beaconing", screen6MenuBeaconingTemp);
  drawScreen6MenuCheckButton(checkX, checkRowY0 + 1 * checkRowGap, "APRS Alert", screen6MenuAprsAlertTemp);
  drawScreen6MenuCheckButton(checkX, checkRowY0 + 2 * checkRowGap, "Range Alert", screen6MenuRangeAlertTemp);
  drawScreen6MenuCheckButton(checkX, checkRowY0 + 3 * checkRowGap, "LED / Buzzer", screen6MenuLedAlertTemp);

  const int closeW = 100;
  const int saveW = 100;
  const int closeH = 26;
  const int closeX = 54;
  const int saveX = 166;
  const int closeY = 210;
  drawFilterTile(closeX, closeY, closeW, closeH, "CLOSE", false);
  drawFilterTile(saveX, closeY, saveW, closeH, "SAVE", false);
}

void handleScreen6MenuTouch(uint16_t x, uint16_t y) {
  const int tileW = 90;
  const int tileH = 26;
  const int gap = 6;
  const int startX = (480 - (3 * tileW + 2 * gap)) / 2;
  const int tileY = 32;

  if (isPointInRect(x, y, startX + 0 * (tileW + gap), tileY, tileW, tileH)) {
    screen6SortMode = APRS_SORT_TIME;
    drawAprsSortMenu();
    return;
  }
  if (isPointInRect(x, y, startX + 1 * (tileW + gap), tileY, tileW, tileH)) {
    screen6SortMode = APRS_SORT_CALLSIGN;
    drawAprsSortMenu();
    return;
  }
  if (isPointInRect(x, y, startX + 2 * (tileW + gap), tileY, tileW, tileH)) {
    screen6SortMode = APRS_SORT_DISTANCE;
    drawAprsSortMenu();
    return;
  }

  const int checkX = 34;
  const int checkRowY0 = 94;
  const int checkRowGap = 28;
  const int checkHitW = 250;
  const int checkHitH = 22;

  if (isPointInRect(x, y, checkX, checkRowY0 + 0 * checkRowGap, checkHitW, checkHitH)) {
    screen6MenuBeaconingTemp = !screen6MenuBeaconingTemp;
    drawAprsSortMenu();
    return;
  }
  if (isPointInRect(x, y, checkX, checkRowY0 + 1 * checkRowGap, checkHitW, checkHitH)) {
    screen6MenuAprsAlertTemp = !screen6MenuAprsAlertTemp;
    drawAprsSortMenu();
    return;
  }
  if (isPointInRect(x, y, checkX, checkRowY0 + 2 * checkRowGap, checkHitW, checkHitH)) {
    screen6MenuRangeAlertTemp = !screen6MenuRangeAlertTemp;
    drawAprsSortMenu();
    return;
  }
  if (isPointInRect(x, y, checkX, checkRowY0 + 3 * checkRowGap, checkHitW, checkHitH)) {
    screen6MenuLedAlertTemp = !screen6MenuLedAlertTemp;
    drawAprsSortMenu();
    return;
  }

  const int closeW = 100;
  const int saveW = 100;
  const int closeH = 26;
  const int closeX = 54;
  const int saveX = 166;
  const int closeY = 210;
  if (isPointInRect(x, y, closeX, closeY, closeW, closeH)) {
    inMenu = false;
    drawScreen(currentScreen);
    return;
  }

  if (isPointInRect(x, y, saveX, closeY, saveW, closeH)) {
    aprsBeaconEnabled = screen6MenuBeaconingTemp;
    aprsAlertEnabled = screen6MenuAprsAlertTemp;
    aprsAlertNearbyEnabled = screen6MenuRangeAlertTemp;
    enableLedAlert = screen6MenuLedAlertTemp;
    savePreferences();
    inMenu = false;
    drawScreen(currentScreen);
    return;
  }
}

void drawPotaFilterMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 6);
  tft.print("POTA FILTERS");

  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(10, 28);
  tft.print("MODE");

  const int modeTileW = 70;
  const int modeTileH = 26;
  const int modeGap = 6;
  const int modeStartX = (480 - (4 * modeTileW + 3 * modeGap)) / 2;
  const int modeY = 40;
  drawFilterTile(modeStartX + 0 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "ALL", screen7FilterMode == FILTER_MODE_ALL);
  drawFilterTile(modeStartX + 1 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "CW", screen7FilterMode == FILTER_MODE_CW);
  drawFilterTile(modeStartX + 2 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "SSB", screen7FilterMode == FILTER_MODE_SSB);
  drawFilterTile(modeStartX + 3 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "DIGI", screen7FilterMode == FILTER_MODE_DIGI);

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(10, 82);
  tft.print("BAND");

  const int bandTileW = 90;
  const int bandTileH = 24;
  const int bandGap = 6;
  const int bandStartX = (480 - (3 * bandTileW + 2 * bandGap)) / 2;
  const int bandStartY = 96;
  int bandIdx = 0;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      if (bandIdx >= SCREEN2_FILTER_BANDS_COUNT) break;
      int x = bandStartX + col * (bandTileW + bandGap);
      int y = bandStartY + row * (bandTileH + bandGap);
      drawFilterTile(x, y, bandTileW, bandTileH, SCREEN2_FILTER_BANDS[bandIdx], screen7FilterBandIndex == bandIdx);
      bandIdx++;
    }
  }

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (480 - closeW) / 2;
  const int closeY = 210;
  drawFilterTile(closeX, closeY, closeW, closeH, "CLOSE", false);
}

void handleScreen7MenuTouch(uint16_t x, uint16_t y) {
  const int modeTileW = 70;
  const int modeTileH = 26;
  const int modeGap = 6;
  const int modeStartX = (480 - (4 * modeTileW + 3 * modeGap)) / 2;
  const int modeY = 40;

  for (int i = 0; i < 4; i++) {
    int rx = modeStartX + i * (modeTileW + modeGap);
    if (isPointInRect(x, y, rx, modeY, modeTileW, modeTileH)) {
      screen7FilterMode = (Screen2FilterMode)i;
      drawPotaFilterMenu();
      return;
    }
  }

  const int bandTileW = 90;
  const int bandTileH = 24;
  const int bandGap = 6;
  const int bandStartX = (480 - (3 * bandTileW + 2 * bandGap)) / 2;
  const int bandStartY = 96;
  int bandIdx = 0;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      if (bandIdx >= SCREEN2_FILTER_BANDS_COUNT) break;
      int rx = bandStartX + col * (bandTileW + bandGap);
      int ry = bandStartY + row * (bandTileH + bandGap);
      if (isPointInRect(x, y, rx, ry, bandTileW, bandTileH)) {
        screen7FilterBandIndex = bandIdx;
        drawPotaFilterMenu();
        return;
      }
      bandIdx++;
    }
  }

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (480 - closeW) / 2;
  const int closeY = 210;
  if (isPointInRect(x, y, closeX, closeY, closeW, closeH)) {
    inMenu = false;
    drawScreen(SCREEN_POTA_CLUSTER);
    return;
  }
}

void drawHamalertFilterMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 6);
  tft.print("HAMALERT FILTR");

  tft.setTextSize(1);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(10, 28);
  tft.print("MODE");

  const int modeTileW = 70;
  const int modeTileH = 26;
  const int modeGap = 6;
  const int modeStartX = (480 - (4 * modeTileW + 3 * modeGap)) / 2;
  const int modeY = 40;
  drawFilterTile(modeStartX + 0 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "ALL", screen8FilterMode == FILTER_MODE_ALL);
  drawFilterTile(modeStartX + 1 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "CW", screen8FilterMode == FILTER_MODE_CW);
  drawFilterTile(modeStartX + 2 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "SSB", screen8FilterMode == FILTER_MODE_SSB);
  drawFilterTile(modeStartX + 3 * (modeTileW + modeGap), modeY, modeTileW, modeTileH, "DIGI", screen8FilterMode == FILTER_MODE_DIGI);

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(10, 82);
  tft.print("BAND");

  const int bandTileW = 72;
  const int bandTileH = 24;
  const int bandGap = 6;
  const int bandStartX = (480 - (4 * bandTileW + 3 * bandGap)) / 2;
  const int bandStartY = 96;
  int bandIdx = 0;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 4; col++) {
      if (bandIdx >= SCREEN8_FILTER_BANDS_COUNT) break;
      int x = bandStartX + col * (bandTileW + bandGap);
      int y = bandStartY + row * (bandTileH + bandGap);
      drawFilterTile(x, y, bandTileW, bandTileH, SCREEN8_FILTER_BANDS[bandIdx], screen8FilterBandIndex == bandIdx);
      bandIdx++;
    }
  }

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (480 - closeW) / 2;
  const int closeY = 210;
  drawFilterTile(closeX, closeY, closeW, closeH, "CLOSE", false);
}

void handleScreen8MenuTouch(uint16_t x, uint16_t y) {
  const int modeTileW = 70;
  const int modeTileH = 26;
  const int modeGap = 6;
  const int modeStartX = (480 - (4 * modeTileW + 3 * modeGap)) / 2;
  const int modeY = 40;

  for (int i = 0; i < 4; i++) {
    int rx = modeStartX + i * (modeTileW + modeGap);
    if (isPointInRect(x, y, rx, modeY, modeTileW, modeTileH)) {
      screen8FilterMode = (Screen2FilterMode)i;
      drawHamalertFilterMenu();
      return;
    }
  }

  const int bandTileW = 72;
  const int bandTileH = 24;
  const int bandGap = 6;
  const int bandStartX = (480 - (4 * bandTileW + 3 * bandGap)) / 2;
  const int bandStartY = 96;
  int bandIdx = 0;
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 4; col++) {
      if (bandIdx >= SCREEN8_FILTER_BANDS_COUNT) break;
      int rx = bandStartX + col * (bandTileW + bandGap);
      int ry = bandStartY + row * (bandTileH + bandGap);
      if (isPointInRect(x, y, rx, ry, bandTileW, bandTileH)) {
        screen8FilterBandIndex = bandIdx;
        drawHamalertFilterMenu();
        return;
      }
      bandIdx++;
    }
  }

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (480 - closeW) / 2;
  const int closeY = 210;
  if (isPointInRect(x, y, closeX, closeY, closeW, closeH)) {
    inMenu = false;
    drawScreen(SCREEN_HAMALERT_CLUSTER);
    return;
  }
}

void drawHamClockTimeMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 6);
  tft.print("TIME MODE");

  const int tileW = 110;
  const int tileH = 28;
  const int gap = 10;
  const int startX = (480 - (2 * tileW + gap)) / 2;
  const int tileY = 60;
  drawFilterTile(startX, tileY, tileW, tileH, "UTC", screen1TimeMode == SCREEN1_TIME_UTC);
  drawFilterTile(startX + tileW + gap, tileY, tileW, tileH, "LOCAL", screen1TimeMode == SCREEN1_TIME_LOCAL);

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (480 - closeW) / 2;
  const int closeY = 210;
  drawFilterTile(closeX, closeY, closeW, closeH, "CLOSE", false);
}

void drawWeatherMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 6);
  tft.print("WEATHER MENU");

  const int tileW = 140;
  const int tileH = 36;
  const int gap = 12;
  const int startX = (480 - (2 * tileW + gap)) / 2;
  const int tileY = 80;
  
  drawFilterTile(startX, tileY, tileW, tileH, "REFRESH", false);
  drawFilterTile(startX + tileW + gap, tileY, tileW, tileH, "FORECAST", currentScreen == SCREEN_WEATHER_FORECAST);

  const int closeW = 140;
  const int closeH = 36;
  const int closeX = (480 - closeW) / 2;
  const int closeY = 210;
  drawFilterTile(closeX, closeY, closeW, closeH, "CLOSE", false);
}

void handleScreen5MenuTouch(uint16_t x, uint16_t y) {
  const int tileW = 140;
  const int tileH = 36;
  const int gap = 12;
  const int startX = (480 - (2 * tileW + gap)) / 2;
  const int tileY = 80;

  if (isPointInRect(x, y, startX, tileY, tileW, tileH)) {
    // Refresh - force weather data update
    if (weatherApiKey.length() > 0) {
      lastWeatherFetchMs = 0; // Reset to force immediate fetch
    }
    inMenu = false;
    drawScreen(currentScreen);
    return;
  }
  if (isPointInRect(x, y, startX + tileW + gap, tileY, tileW, tileH)) {
    // Toggle between weather and forecast screen
    if (currentScreen == SCREEN_WEATHER_DSP) {
      currentScreen = SCREEN_WEATHER_FORECAST;
    } else {
      currentScreen = SCREEN_WEATHER_DSP;
    }
    inMenu = false;
    drawScreen(currentScreen);
    return;
  }

  const int closeW = 140;
  const int closeH = 36;
  const int closeX = (480 - closeW) / 2;
  const int closeY = 210;
  if (isPointInRect(x, y, closeX, closeY, closeW, closeH)) {
    inMenu = false;
    drawScreen(currentScreen);
    return;
  }
}

void handleScreen1MenuTouch(uint16_t x, uint16_t y) {
  const int tileW = 110;
  const int tileH = 28;
  const int gap = 10;
  const int startX = (480 - (2 * tileW + gap)) / 2;
  const int tileY = 60;

  if (isPointInRect(x, y, startX, tileY, tileW, tileH)) {
    screen1TimeMode = SCREEN1_TIME_UTC;
    savePreferences();
    drawHamClockTimeMenu();
    return;
  }
  if (isPointInRect(x, y, startX + tileW + gap, tileY, tileW, tileH)) {
    screen1TimeMode = SCREEN1_TIME_LOCAL;
    savePreferences();
    drawHamClockTimeMenu();
    return;
  }

  const int closeW = 100;
  const int closeH = 26;
  const int closeX = (480 - closeW) / 2;
  const int closeY = 210;
  if (isPointInRect(x, y, closeX, closeY, closeW, closeH)) {
    inMenu = false;
    drawScreen(SCREEN_HAM_CLOCK);
    return;
  }
}

void redrawCurrentMenu() {
  if (currentScreen == SCREEN_HAM_CLOCK) {
    drawHamClockTimeMenu();
    return;
  }
  if (currentScreen == SCREEN_DX_CLUSTER) {
    drawDxClusterFilterMenu();
    return;
  }
  if (currentScreen == SCREEN_APRS_IS || currentScreen == SCREEN_APRS_RADAR) {
    drawAprsSortMenu();
    return;
  }
  if (currentScreen == SCREEN_WEATHER_DSP || currentScreen == SCREEN_WEATHER_FORECAST) {
    drawWeatherMenu();
    return;
  }
}

void restoreAfterBrightnessMenu() {
  inMenu = brightnessMenuPrevInMenu;
  if (brightnessMenuPrevInMenu) {
    redrawCurrentMenu();
    return;
  }
  currentScreen = brightnessMenuPrevScreen;
  drawScreen(currentScreen);
}

void drawBrightnessSlider() {
  const int sliderX = 40;
  const int sliderY = 90;
  const int sliderW = 400;
  const int sliderH = 12;
  const int knobW = 6;

  // Wyczyść obszar suwaka (z zapasem na pokrętło), żeby nie zostawał ślad
  tft.fillRect(sliderX - knobW, sliderY - 6, sliderW + knobW * 2, sliderH + 12, TFT_BLACK);

  tft.drawRect(sliderX, sliderY, sliderW, sliderH, TFT_WHITE);
  int fillW = (brightnessMenuValue * (sliderW - 2)) / 100;
  tft.fillRect(sliderX + 1, sliderY + 1, sliderW - 2, sliderH - 2, TFT_BLACK);
  tft.fillRect(sliderX + 1, sliderY + 1, fillW, sliderH - 2, TFT_RADIO_ORANGE);

  int knobX = sliderX + (brightnessMenuValue * (sliderW - knobW)) / 100;
  tft.fillRect(knobX, sliderY - 4, knobW, sliderH + 8, TFT_WHITE);

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(10, 70);
  tft.print(sanitizePolishToAscii(String(tr(TR_BRIGHTNESS))));
  tft.fillRect(120, 70, 40, 8, TFT_BLACK);
  tft.setCursor(120, 70);
  tft.print(brightnessMenuValue);
  tft.print("%");
}

void drawThemeSlider() {
  const int sliderX = 40;
  const int sliderY = 130;
  const int sliderW = 400;
  const int sliderH = 12;
  const int knobW = 6;

  // Wyczyść obszar suwaka (z zapasem na pokrętło)
  tft.fillRect(sliderX - knobW, sliderY - 6, sliderW + knobW * 2, sliderH + 12, TFT_BLACK);

  for (int i = 0; i < sliderW; i++) {
    uint8_t hue = (uint8_t)((i * 255) / (sliderW - 1));
    tft.drawFastVLine(sliderX + i, sliderY + 1, sliderH - 2, colorWheel(hue));
  }
  tft.drawRect(sliderX, sliderY, sliderW, sliderH, TFT_WHITE);

  int knobX = sliderX + (menuThemeHue * (sliderW - knobW)) / 255;
  tft.fillRect(knobX, sliderY - 4, knobW, sliderH + 8, TFT_WHITE);

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(10, 110);
  tft.print(sanitizePolishToAscii(String(tr(TR_THEME_COLOR))));
}

void drawBrightnessMenuHeader() {
  tft.fillRect(0, 0, 480, 28, menuThemeColor);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 6);
  tft.print(sanitizePolishToAscii(String(tr(TR_DISPLAY_SETTINGS))));
}

void drawBrightnessMenu() {
  tft.fillScreen(TFT_BLACK);
  drawBrightnessMenuHeader();

  // Podpowiedź: długi tap 3s w dowolnym miejscu uruchamia kalibrację
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(10, 40);
  tft.print(sanitizePolishToAscii(String(tr(TR_TFT_CALIBRATION_HINT))));

  drawBrightnessSlider();
  drawThemeSlider();

  const int langLabelY = 150;
  const int langTileW = 70;
  const int langTileH = 22;
  const int langGap = 10;
  const int langStartX = (480 - (2 * langTileW + langGap)) / 2;
  const int langY = 240;

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(10, langLabelY);
  tft.print(sanitizePolishToAscii(String(tr(TR_LANGUAGE))));
  drawFilterTile(langStartX, langY, langTileW, langTileH, "PL", tftLanguage == TFT_LANG_PL);
  drawFilterTile(langStartX + langTileW + langGap, langY, langTileW, langTileH, "EN", tftLanguage == TFT_LANG_EN);

  const int btnW = 90;
  const int btnH = 26;
  const int btnY = 190;
  const int btnGap = 10;
  const int saveX = 10;
  const int defaultX = saveX + btnW + btnGap;
  const int saverX = defaultX + btnW + btnGap;
  const int closeX = saverX + btnW + btnGap;
  drawFilterTile(saveX, btnY, btnW, btnH, tr(TR_SAVE), false);
  drawFilterTile(defaultX, btnY, btnW, btnH, tr(TR_DEFAULT), false);
  drawFilterTile(saverX, btnY, btnW, btnH, "USPIENIE", false);
  drawFilterTile(closeX, btnY, btnW, btnH, tr(TR_CLOSE), false);
}

void drawTouchCalTarget(int cx, int cy, uint16_t color) {
  tft.drawLine(cx - 10, cy, cx + 10, cy, color);
  tft.drawLine(cx, cy - 10, cx, cy + 10, color);
  tft.drawRect(cx - 12, cy - 12, 24, 24, color);
}

void drawTouchCalibrationScreen() {
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  int centerY = 120;
  
  if (touchCalStep == 0) {
    tft.setCursor(40, centerY);
    tft.print("Touch target 1/4 (top-left)");
    drawTouchCalTarget(20, 20, TFT_RADIO_ORANGE);
  } else if (touchCalStep == 1) {
    tft.setCursor(35, centerY);
    tft.print("Touch target 2/4 (top-right)");
    drawTouchCalTarget(460, 20, TFT_RADIO_ORANGE);
  } else if (touchCalStep == 2) {
    tft.setCursor(30, centerY);
    tft.print("Touch target 3/4 (bottom-right)");
    drawTouchCalTarget(460, 300, TFT_RADIO_ORANGE);
  } else if (touchCalStep == 3) {
    tft.setCursor(35, centerY);
    tft.print("Touch target 4/4 (bottom-left)");
    drawTouchCalTarget(20, 300, TFT_RADIO_ORANGE);
  } else {
    // touchCalStep >= 4 - Review screen
    tft.setCursor(40, centerY);
    tft.print("Calibration complete - Touch to close");
    
    // Narysuj wszystkie 4 targets jako potwierdzenie
    drawTouchCalTarget(20, 20, TFT_DARKGREY);
    drawTouchCalTarget(460, 20, TFT_DARKGREY);
    drawTouchCalTarget(460, 300, TFT_DARKGREY);
    drawTouchCalTarget(20, 300, TFT_DARKGREY);
    
    tft.setTextColor(TFT_LIGHTGREY);
    tft.setCursor(10, 30);
    tft.print("Raw X: ");
    tft.print(touchCalNewXMin);
    tft.print(" - ");
    tft.print(touchCalNewXMax);
    tft.setCursor(10, 45);
    tft.print("Raw Y: ");
    tft.print(touchCalNewYMin);
    tft.print(" - ");
    tft.print(touchCalNewYMax);
    
    tft.setCursor(10, 65);
    tft.print("Detected: Swap:");
    tft.print(touchSwapXY ? "YES" : "NO");
    tft.print(" InvX:");
    tft.print(touchInvertX ? "YES" : "NO");
    tft.setCursor(10, 80);
    tft.print("          InvY:");
    tft.print(touchInvertY ? "YES" : "NO");
    
    // Podpowiedź do ustawienia w WWW
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(10, 145);
    tft.print("Use WWW panel >");
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, 240);
    tft.print(sanitizePolishToAscii(String(tr(TR_TFT_CALIBRATION_HINT))));
    tft.setTextColor(TFT_GREENYELLOW);
    tft.setCursor(10, 180);
    
    // Ustal jaki tryb sugerować
    if (touchSwapXY && touchInvertX && touchInvertY) {
      tft.print("-> SWAP XY + INVERT BOTH");
    } else if (touchSwapXY && touchInvertX) {
      tft.print("-> ");
      tft.print(tr(TR_ROT_90_RIGHT));
    } else if (touchSwapXY && touchInvertY) {
      tft.print("-> ");
      tft.print(tr(TR_ROT_90_LEFT));
    } else if (touchSwapXY) {
      tft.print("-> SWAP XY");
    } else if (touchInvertX && touchInvertY) {
      tft.print("-> INVERT BOTH (180deg)");
    } else if (touchInvertX) {
      tft.print("-> INVERT X");
    } else if (touchInvertY) {
      tft.print("-> INVERT Y");
    } else {
      tft.print("-> NONE (default - OK)");
    }
    
    tft.setTextColor(TFT_LIGHTGREY);
    tft.setCursor(10, 210);
    tft.print("(Touch will NOT save changes)");
  }
}

void startTouchCalibration() {
  touchCalActive = true;
  touchCalStep = 0;
  touchCalRawX1 = 0;
  touchCalRawY1 = 0;
  touchCalRawX2 = 0;
  touchCalRawY2 = 0;
  touchCalRawX3 = 0;
  touchCalRawY3 = 0;
  touchCalRawX4 = 0;
  touchCalRawY4 = 0;
  touchCalNewXMin = touchXMin;
  touchCalNewXMax = touchXMax;
  touchCalNewYMin = touchYMin;
  touchCalNewYMax = touchYMax;
  drawTouchCalibrationScreen();
}

void setBrightnessFromTouch(uint16_t x) {
  const int sliderX = 40;
  const int sliderW = 400;
  int val = (int)((long)(x - sliderX) * 100 / (sliderW - 1));
  if (val < MIN_BACKLIGHT_PERCENT) val = MIN_BACKLIGHT_PERCENT;
  if (val > 100) val = 100;
  if (val != brightnessMenuValue) {
    brightnessMenuValue = val;
    setBacklightPercent(brightnessMenuValue);
    drawBrightnessSlider();
  }
}

void setThemeFromTouch(uint16_t x) {
  const int sliderX = 40;
  const int sliderW = 400;
  int val = (int)((long)(x - sliderX) * 255 / (sliderW - 1));
  if (val < 0) val = 0;
  if (val > 255) val = 255;
  uint8_t hue = (uint8_t)val;
  if (hue != menuThemeHue) {
    menuThemeHue = hue;
    applyMenuThemeFromHue();
    drawBrightnessMenuHeader();
    drawThemeSlider();
    drawBrightnessSlider();
  }
}

void handleTouchCalibrationTouch(int16_t rawX, int16_t rawY, uint16_t x, uint16_t y, bool isNewTap) {
  if (!touchCalActive || !isNewTap) {
    return;
  }

  if (touchCalStep == 0) {
    // Target 1: lewy górny (20, 20)
    touchCalRawX1 = rawX;
    touchCalRawY1 = rawY;
    touchCalStep = 1;
    drawTouchCalibrationScreen();
    return;
  }

  if (touchCalStep == 1) {
    // Target 2: prawy górny (300, 20)
    touchCalRawX2 = rawX;
    touchCalRawY2 = rawY;
    touchCalStep = 2;
    drawTouchCalibrationScreen();
    return;
  }

  if (touchCalStep == 2) {
    // Target 3: prawy dolny (300, 220)
    touchCalRawX3 = rawX;
    touchCalRawY3 = rawY;
    touchCalStep = 3;
    drawTouchCalibrationScreen();
    return;
  }

  if (touchCalStep == 3) {
    // Target 4: lewy dolny (20, 220)
    touchCalRawX4 = rawX;
    touchCalRawY4 = rawY;
    
    // Wszystkie 4 punkty zebrane - oblicz parametry
    // Min/Max ze wszystkich 4 punktów surowych
    touchCalNewXMin = min(min(touchCalRawX1, touchCalRawX2), min(touchCalRawX3, touchCalRawX4));
    touchCalNewXMax = max(max(touchCalRawX1, touchCalRawX2), max(touchCalRawX3, touchCalRawX4));
    touchCalNewYMin = min(min(touchCalRawY1, touchCalRawY2), min(touchCalRawY3, touchCalRawY4));
    touchCalNewYMax = max(max(touchCalRawY1, touchCalRawY2), max(touchCalRawY3, touchCalRawY4));
    
    // Wykryj swap: porównaj rozpiętość surowych wartości
    // Targets rozciągają się w X: 20-300 (280px) i Y: 20-220 (200px)
    // Jeśli deltaRawY > deltaRawX, osie są zamienione
    int deltaRawX = touchCalNewXMax - touchCalNewXMin;
    int deltaRawY = touchCalNewYMax - touchCalNewYMin;
    touchSwapXY = (deltaRawY > deltaRawX);
    
    // Wykryj odwrócenie osi
    // Porównaj punkt 1 (góra-lewo) z punktem 3 (dół-prawo)
    int16_t x1 = touchCalRawX1;
    int16_t y1 = touchCalRawY1;
    int16_t x3 = touchCalRawX3;
    int16_t y3 = touchCalRawY3;
    
    // Jeśli osie są zamienione, swap współrzędne przed sprawdzeniem kierunku
    if (touchSwapXY) {
      int16_t tmp = x1; x1 = y1; y1 = tmp;
      tmp = x3; x3 = y3; y3 = tmp;
    }
    
    // Target 1 (20,20) -> Target 3 (300,220): oczekujemy x3 > x1 i y3 > y1
    touchInvertX = (x3 < x1);
    touchInvertY = (y3 < y1);
    
    touchCalStep = 4;
    drawTouchCalibrationScreen();
    return;
  }

  if (touchCalStep >= 4) {
    // Dowolne dotknięcie zamyka ekran kalibracji bez zapisywania
    // Przywróć poprzednie wartości flag
    touchSwapXY = TOUCH_SWAP_XY;
    touchInvertX = TOUCH_INVERT_X;
    touchInvertY = TOUCH_INVERT_Y;
    touchCalActive = false;
    drawBrightnessMenu();
    return;
  }
}

void handleBrightnessMenuTouch(uint16_t x, uint16_t y, bool isNewTap) {
  unsigned long now = millis();
  
  if (now - brightnessMenuOpenedMs < 1500) {
    return;
  }

  // Zawsze aktualizuj znacznik startu dla bieżącego dotknięcia, zanim obsłużymy kafelki
  if (isNewTap) {
    brightnessMenuTouchStartMs = now;
    brightnessMenuLongPressHandled = false;
  }

  // Długi tap (>=5s) w dowolnym miejscu ekranu jasności uruchamia kalibrację dotyku
  if (!brightnessMenuLongPressHandled && brightnessMenuTouchStartMs > 0 && (now - brightnessMenuTouchStartMs) >= 5000) {
    brightnessMenuLongPressHandled = true;
    startTouchCalibration();
    return;
  }

  const int sliderX = 40;
  const int sliderY = 90;
  const int sliderW = 400;
  const int sliderH = 12;
  const int themeSliderY = 130;
  const int themeSliderH = 12;
  const int langTileW = 70;
  const int langTileH = 22;
  const int langGap = 10;
  const int langStartX = (480 - (2 * langTileW + langGap)) / 2;
  const int langY = 240;

  // Obsługa sliderów
  if (y >= sliderY - 6 && y <= sliderY + sliderH + 6) {
    setBrightnessFromTouch(x);
  }
  if (y >= themeSliderY - 6 && y <= themeSliderY + themeSliderH + 6) {
    setThemeFromTouch(x);
  }

  if (isNewTap) {
    if (isPointInRect(x, y, langStartX, langY, langTileW, langTileH)) {
      tftLanguage = TFT_LANG_PL;
      // Wymuś odświeżenie opisów pogody po zmianie języka
      lastWeatherFetchMs = 0;
      weatherData.valid = false;
      weatherData.forecast3hValid = false;
      weatherData.forecastNextDayValid = false;
      for (uint8_t i = 0; i < WeatherData::DETAIL_COLS; i++) {
        weatherData.detailValid[i] = false;
      }
      savePreferences();
      drawBrightnessMenu();
      return;
    }
    if (isPointInRect(x, y, langStartX + langTileW + langGap, langY, langTileW, langTileH)) {
      tftLanguage = TFT_LANG_EN;
      // Wymuś odświeżenie opisów pogody po zmianie języka
      lastWeatherFetchMs = 0;
      weatherData.valid = false;
      weatherData.forecast3hValid = false;
      weatherData.forecastNextDayValid = false;
      for (uint8_t i = 0; i < WeatherData::DETAIL_COLS; i++) {
        weatherData.detailValid[i] = false;
      }
      savePreferences();
      drawBrightnessMenu();
      return;
    }
  }

  if (!isNewTap) {
    return;
  }

  const int btnW = 90;
  const int btnH = 26;
  const int btnY = 190;
  const int btnGap = 10;
  const int saveX = 10;
  const int defaultX = saveX + btnW + btnGap;
  const int saverX = defaultX + btnW + btnGap;
  const int closeX = saverX + btnW + btnGap;

  if (isPointInRect(x, y, saveX, btnY, btnW, btnH)) {
    backlightPercent = brightnessMenuValue;
    savePreferences();
    brightnessMenuActive = false;
    restoreAfterBrightnessMenu();
    return;
  }
  if (isPointInRect(x, y, defaultX, btnY, btnW, btnH)) {
    menuThemeHue = DEFAULT_MENU_THEME_HUE;
    applyMenuThemeFromHue();
    drawBrightnessMenuHeader();
    drawThemeSlider();
    drawBrightnessSlider();
    return;
  }
  if (isPointInRect(x, y, closeX, btnY, btnW, btnH)) {
    backlightPercent = brightnessMenuPrevBacklight;
    setBacklightPercent(backlightPercent);
    menuThemeHue = brightnessMenuPrevThemeHue;
    applyMenuThemeFromHue();
    brightnessMenuActive = false;
    restoreAfterBrightnessMenu();
    return;
  }
  
  // Przycisk UŚPIENIE - otwórz menu uśpienia
  if (isPointInRect(x, y, saverX, btnY, btnW, btnH)) {
    brightnessMenuActive = false;  // Zamknij menu jasności
    screenSleepMenuActive = true;
    drawScreenSleepMenu();
    return;
  }
}

// Ekran 6: APRS-IS - zegar tylko dla RADAR, nie dla listy
void updateScreen6Clock() {
  if (!tftInitialized || inMenu) {
    return;
  }
  // Zegar tylko dla APRS_RADAR, nie dla APRS_IS (lista)
  if (currentScreen != SCREEN_APRS_RADAR) {
    return;
  }

  static char lastDrawnTime[6] = "";
  static bool lastRadarMode = false;

  struct tm timeinfo;
  if (!getTimeWithTimezone(&timeinfo)) {
    return;
  }

  char timeBuffer[6];
  strftime(timeBuffer, 6, "%H:%M", &timeinfo);
  int timeWidth = strlen(timeBuffer) * 12;
  bool radarMode = true; // Zawsze radar mode tutaj
  uint16_t topClearColor = TFT_BLACK;
  uint16_t timeColor = TFT_WHITE;
  int timeY = 5;

  if (strcmp(lastDrawnTime, timeBuffer) == 0 && lastRadarMode == radarMode) {
    return;
  }

  int timeX = 480 - timeWidth - 4;

  tft.fillRect(timeX - 2, 4, timeWidth + 6, 24, topClearColor);
  tft.setTextColor(timeColor);
  tft.setTextSize(2);
  tft.setCursor(timeX, timeY);
  tft.print(timeBuffer);

  strncpy(lastDrawnTime, timeBuffer, sizeof(lastDrawnTime));
  lastDrawnTime[sizeof(lastDrawnTime) - 1] = '\0';
  lastRadarMode = radarMode;
}

uint32_t computeScreen6Signature() {
  // Prosty hash treÄąâ€şci tabeli (10 lub 11 stacji APRS zależnie od paska nawigacji)
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  hash ^= (uint32_t)aprsStationCount;
  hash *= fnvPrime;
  hash ^= (uint32_t)screen6SortMode;
  hash *= fnvPrime;
  hash ^= (uint32_t)(screen6RadarZoom * 100.0f);
  hash *= fnvPrime;
  hash ^= isScreen6RadarHintVisible() ? 1u : 0u;
  hash *= fnvPrime;
  hash ^= (uint32_t)dxTableSizeMode;
  hash *= fnvPrime;
  hash ^= isTableNavFooterVisible(SCREEN_APRS_IS) ? 1u : 0u;
  hash *= fnvPrime;

  int order[MAX_APRS_DISPLAY_LCD];
  int displayCount = 0;
  buildAprsDisplayOrder(order, displayCount);
  int maxRows = getAprsTableMaxRows();
  int visibleCount = min(displayCount, maxRows);
  for (int i = 0; i < visibleCount; i++) {
    const APRSStation &s = aprsStations[order[i]];
    for (size_t j = 0; j < s.time.length(); j++) {
      hash ^= (uint8_t)s.time[j];
      hash *= fnvPrime;
    }
    for (size_t j = 0; j < s.callsign.length(); j++) {
      hash ^= (uint8_t)s.callsign[j];
      hash *= fnvPrime;
    }
    hash ^= (uint32_t)(s.lat * 1000);
    hash *= fnvPrime;
    hash ^= (uint32_t)(s.lon * 1000);
    hash *= fnvPrime;
    hash ^= (uint32_t)s.distance;
    hash *= fnvPrime;
    hash ^= (uint32_t)(s.freqMHz * 1000);
    hash *= fnvPrime;
  }

  return hash;
}

void updateScreen6Data() {
  if (!tftInitialized || inMenu) {
    return;
  }
  if (currentScreen != SCREEN_APRS_IS && currentScreen != SCREEN_APRS_RADAR) {
    return;
  }

  // 1) Zaktualizuj czas w nagłówku (bez pełnego odświeżania)
  updateScreen6Clock();

  // 2) Odśwież tabelę tylko gdy dane się zmieniły
  static uint32_t lastSig = 0;
  uint32_t currentSig = computeScreen6Signature();
  if (currentSig == lastSig) {
    return;
  }
  lastSig = currentSig;

  if (currentScreen == SCREEN_APRS_RADAR) {
    drawAprsRadar();
    return;
  }

  // 3) Renderuj tabelę do bufora i wypchnij jednym ruchem (bez migotania)
  const bool navVisible = isTableNavFooterVisible(SCREEN_APRS_IS);
  const bool enlarged = isDxTableEnlarged();
  const int maxRows = getAprsTableMaxRows();
  const int tableTop = TFT_TABLE_TOP;
  const int tableBottom = getTableBottomForScreen(SCREEN_APRS_IS);
  const int tableHeight = tableBottom - tableTop;
  TFT_eSprite *tableSprite = (navVisible && ensureSharedTableSprite()) ? &sharedTableSprite : nullptr;

  if (tableSprite != nullptr) {
    tableSprite->fillSprite(TFT_BLACK);

    int yPos = 8;
    tableSprite->setTextColor(TFT_DARKGREY);
    tableSprite->setTextSize(1);
    if (enlarged) {
      tableSprite->setCursor(5, yPos);   tableSprite->print("UTC");
      tableSprite->setCursor(74, yPos);  tableSprite->print("CALL");
      tableSprite->setCursor(198, yPos); tableSprite->print("KM");
      tableSprite->setCursor(224, yPos); tableSprite->print("FREQ");
    } else {
      tableSprite->setCursor(5, yPos);   tableSprite->print("UTC");
      tableSprite->setCursor(50, yPos);  tableSprite->print("CALLSIGN");
      tableSprite->setCursor(125, yPos); tableSprite->print("SYMBOL");
      tableSprite->setCursor(200, yPos); tableSprite->print("KM");
      tableSprite->setCursor(245, yPos); tableSprite->print("FREQ");
    }
    tableSprite->drawFastHLine(0, yPos + 10, 480, TFT_DARKGREY);
    yPos += enlarged ? 20 : 18;

    int order[MAX_APRS_DISPLAY_LCD];
    int displayCount = 0;
    buildAprsDisplayOrder(order, displayCount);
    int visibleCount = min(displayCount, maxRows);
    for (int i = 0; i < visibleCount; i++) {
      if (yPos >= (tableHeight - 2)) {
        break;
      }
      const APRSStation &station = aprsStations[order[i]];
      if (enlarged) {
        if (i % 2 == 0) {
          tableSprite->fillRect(0, yPos - 5, 480, 24, TFT_TABLE_ALT_ROW_COLOR);
        }

        tableSprite->setTextSize(2);

        tableSprite->setTextColor(TFT_LIGHTGREY);
        tableSprite->setCursor(5, yPos);
        String timeStr = formatAprsTimeWithTimezone(station.time);
        tableSprite->print(timeStr);

        tableSprite->setTextColor(getAprsCallsignColorForEnlarged(station));
        tableSprite->setCursor(74, yPos);
        String callText = station.callsign;
        if (callText.length() > 8) callText = callText.substring(0, 8);
        tableSprite->print(callText);

        tableSprite->setTextColor(TFT_RADIO_ORANGE);
        tableSprite->setCursor(198, yPos);
        if (station.hasLatLon && station.distance > 0) {
          tableSprite->print((int)station.distance);
        } else {
          tableSprite->print("-");
        }

        tableSprite->setTextColor(TFT_CYAN);
        tableSprite->setCursor(224, yPos);
        if (station.freqMHz > 0.0f) {
          tableSprite->print(String(station.freqMHz, 3));
        } else {
          tableSprite->print("-");
        }

        yPos += 27;
      } else {
        if (i % 2 == 0) {
          tableSprite->fillRect(0, yPos - 2, 480, 16, TFT_TABLE_ALT_ROW_COLOR);
        }

        tableSprite->setTextSize(1);

        tableSprite->setTextColor(TFT_LIGHTGREY);
        tableSprite->setCursor(5, yPos);
        String timeStr = formatAprsTimeWithTimezone(station.time);
        tableSprite->print(timeStr);

        tableSprite->setTextColor(TFT_WHITE);
        tableSprite->setCursor(50, yPos);
        tableSprite->print(station.callsign);

        String symbolShort = getAPRSSymbolShort(station);
        uint16_t symbolColor = TFT_GREEN;
        if (symbolShort == "HUMAN") {
          symbolColor = TFT_BLUE;
        } else if (symbolShort == "HOUSE") {
          symbolColor = TFT_YELLOW;
        } else if (symbolShort == "CAR") {
          symbolColor = TFT_RED;
        }
        tableSprite->setTextColor(symbolColor);
        tableSprite->setCursor(125, yPos);
        tableSprite->print(symbolShort);

        tableSprite->setTextColor(TFT_RADIO_ORANGE);
        tableSprite->setCursor(200, yPos);
        if (station.hasLatLon && station.distance > 0) {
          tableSprite->print((int)station.distance);
          tableSprite->print(" km");
        } else {
          tableSprite->print("-");
        }

        tableSprite->setTextColor(TFT_CYAN);
        tableSprite->setCursor(245, yPos);
        if (station.freqMHz > 0.0f) {
          tableSprite->print(String(station.freqMHz, 3));
        } else {
          tableSprite->print("-");
        }

        yPos += 17;
      }
    }

    if (visibleCount == 0) {
      tableSprite->setTextColor(TFT_RED);
      tableSprite->setTextSize(2);
      tableSprite->setCursor(40, 120 - tableTop);
      tableSprite->print("WAITING FOR APRS...");
    }

    tableSprite->pushSprite(0, tableTop);
    return;
  }

  // Fallback bez sprite (np. gdy zabraknie RAM)
  tft.fillRect(0, tableTop, 480, tableHeight, TFT_BLACK);

  int yPos = 40;
  tft.setTextColor(TFT_DARKGREY);
  tft.setTextSize(1);
  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(198, yPos); tft.print("KM");
    tft.setCursor(224, yPos); tft.print("FREQ");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("SYMBOL");
    tft.setCursor(200, yPos); tft.print("KM");
    tft.setCursor(245, yPos); tft.print("FREQ");
  }
  tft.drawFastHLine(0, yPos + 10, 480, TFT_DARKGREY);
  yPos += enlarged ? 20 : 18;

  int order[MAX_APRS_DISPLAY_LCD];
  int displayCount = 0;
  buildAprsDisplayOrder(order, displayCount);
  int visibleCount = min(displayCount, maxRows);
  for (int i = 0; i < visibleCount; i++) {
    if (yPos >= (tableBottom - 2)) {
      break;
    }
    const APRSStation &station = aprsStations[order[i]];
    if (enlarged) {
      if (i % 2 == 0) {
        tft.fillRect(0, yPos - 5, 480, 24, TFT_TABLE_ALT_ROW_COLOR);
      }

      tft.setTextSize(2);

      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      String timeStr = formatAprsTimeWithTimezone(station.time);
      tft.print(timeStr);

      tft.setTextColor(getAprsCallsignColorForEnlarged(station));
      tft.setCursor(74, yPos);
      String callText = station.callsign;
      if (callText.length() > 8) callText = callText.substring(0, 8);
      tft.print(callText);

      tft.setTextColor(TFT_RADIO_ORANGE);
      tft.setCursor(198, yPos);
      if (station.hasLatLon && station.distance > 0) {
        tft.print((int)station.distance);
      } else {
        tft.print("-");
      }

      tft.setTextColor(TFT_CYAN);
      tft.setCursor(224, yPos);
      if (station.freqMHz > 0.0f) {
        tft.print(String(station.freqMHz, 3));
      } else {
        tft.print("-");
      }

      yPos += 27;
    } else {
      if (i % 2 == 0) {
        tft.fillRect(0, yPos - 2, 480, 16, TFT_TABLE_ALT_ROW_COLOR);
      }

      tft.setTextSize(1);

      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      String timeStr = formatAprsTimeWithTimezone(station.time);
      tft.print(timeStr);

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(50, yPos);
      tft.print(station.callsign);

      String symbolShort = getAPRSSymbolShort(station);
      uint16_t symbolColor = TFT_GREEN;
      if (symbolShort == "HUMAN") {
        symbolColor = TFT_BLUE;
      } else if (symbolShort == "HOUSE") {
        symbolColor = TFT_YELLOW;
      } else if (symbolShort == "CAR") {
        symbolColor = TFT_RED;
      }
      tft.setTextColor(symbolColor);
      tft.setCursor(125, yPos);
      tft.print(symbolShort);

      tft.setTextColor(TFT_RADIO_ORANGE);
      tft.setCursor(200, yPos);
      if (station.hasLatLon && station.distance > 0) {
        tft.print((int)station.distance);
        tft.print(" km");
      } else {
        tft.print("-");
      }

      tft.setTextColor(TFT_CYAN);
      tft.setCursor(245, yPos);
      if (station.freqMHz > 0.0f) {
        tft.print(String(station.freqMHz, 3));
      } else {
        tft.print("-");
      }

      yPos += 17;
    }
  }

  if (visibleCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR APRS...");
  }
  
  // Strzałki nawigacyjne - ZAWSZE rysuj po odświeżeniu tabeli
  drawSwitchScreenFooter();
}

void drawAprsIs() {
  screen6ViewMode = APRS_VIEW_LIST;
  tft.fillScreen(TFT_BLACK);

  // 1. NAGÄąÂÄ‚â€śWEK: Belka z menu, nazwĂ„â€¦ serwera i czasem UTC
  tft.fillRect(0, 0, 480, 32, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);

  // IKONA MENU (3D)
  drawHamburgerMenuButton3D(5, 7);

  // Nazwa serwera APRS-IS - przesuniÄta w prawo (x=35 zamiast 5), by zrobiÄ‡ miejsce na menu
  tft.setTextSize(2);
  tft.setCursor(35, 8);
  tft.print("APRS-IS");

  // Ikona WiFi w gÃ³rnym prawym rogu
  drawWifiSignalBars(445, 4);

  // ZEGAR USUNIÄTY z nagÅ‚Ã³wka APRS-IS

  // 2. NAGÅÃ“WKI TABELI
  int yPos = 40;
  const bool enlarged = isDxTableEnlarged();
  tft.setTextColor(TFT_DARKGREY);
  tft.setTextSize(1);
  if (enlarged) {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(74, yPos);  tft.print("CALL");
    tft.setCursor(198, yPos); tft.print("KM");
    tft.setCursor(224, yPos); tft.print("FREQ");
  } else {
    tft.setCursor(5, yPos);   tft.print("UTC");
    tft.setCursor(50, yPos);  tft.print("CALLSIGN");
    tft.setCursor(125, yPos); tft.print("SYMBOL");
    tft.setCursor(200, yPos); tft.print("KM");
  }
  tft.drawFastHLine(0, yPos + 10, 480, TFT_DARKGREY);
  yPos += enlarged ? 20 : 18;

  // 3. LISTA STACJI APRS (10 z paskiem nawigacji, 11 bez)
  int order[MAX_APRS_DISPLAY_LCD];
  int maxRows = getAprsTableMaxRows();
  int displayCount = 0;
  buildAprsDisplayOrder(order, displayCount);
  int visibleCount = min(displayCount, maxRows);
  for (int i = 0; i < visibleCount; i++) {
    const APRSStation &station = aprsStations[order[i]];
    if (enlarged) {
      if (i % 2 == 0) tft.fillRect(0, yPos - 5, 480, 24, TFT_TABLE_ALT_ROW_COLOR);

      tft.setTextSize(2);
      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      String timeStr = formatAprsTimeWithTimezone(station.time);
      tft.print(timeStr);

      tft.setTextColor(getAprsCallsignColorForEnlarged(station));
      tft.setCursor(74, yPos);
      String callText = station.callsign;
      if (callText.length() > 8) callText = callText.substring(0, 8);
      tft.print(callText);

      tft.setTextColor(TFT_RADIO_ORANGE);
      tft.setCursor(198, yPos);
      if (station.hasLatLon && station.distance > 0) {
        tft.print((int)station.distance);
      } else {
        tft.print("-");
      }

      tft.setTextColor(TFT_CYAN);
      tft.setCursor(224, yPos);
      if (station.freqMHz > 0.0f) {
        tft.print(String(station.freqMHz, 3));
      } else {
        tft.print("-");
      }

      yPos += 27;
    } else {
      if (i % 2 == 0) tft.fillRect(0, yPos - 2, 480, 16, TFT_TABLE_ALT_ROW_COLOR);

      tft.setTextSize(1);
      tft.setTextColor(TFT_LIGHTGREY);
      tft.setCursor(5, yPos);
      String timeStr = formatAprsTimeWithTimezone(station.time);
      tft.print(timeStr);

      tft.setTextColor(TFT_WHITE);
      tft.setCursor(50, yPos);
      tft.print(station.callsign);

      String symbolShort = getAPRSSymbolShort(station);
      uint16_t symbolColor = TFT_GREEN;
      if (symbolShort == "HUMAN") {
        symbolColor = TFT_BLUE;
      } else if (symbolShort == "HOUSE") {
        symbolColor = TFT_YELLOW;
      } else if (symbolShort == "CAR") {
        symbolColor = TFT_RED;
      }
      tft.setTextColor(symbolColor);
      tft.setCursor(125, yPos);
      tft.print(symbolShort);

      tft.setTextColor(TFT_RADIO_ORANGE);
      tft.setCursor(200, yPos);
      if (station.hasLatLon && station.distance > 0) {
        tft.print((int)station.distance);
        tft.print(" km");
      } else {
        tft.print("-");
      }

      yPos += 17;
    }
  }

  // 4. Pasek nawigacji - ZAWSZE widoczny
  drawSwitchScreenFooter();

  if (visibleCount == 0) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 120);
    tft.print("WAITING FOR APRS...");
  }
}

void drawAprsRadar() {
  screen6ViewMode = APRS_VIEW_RADAR;
  tft.fillScreen(TFT_BLACK);
  drawAprsRadarBody();

  drawHamburgerMenuButton3D(5, 7);

  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  const char *aprsTitleTop = "APRS";
  const char *aprsTitleBottom = "RADAR";
  int aprsTopWidth = strlen(aprsTitleTop) * 12;
  int aprsBottomWidth = strlen(aprsTitleBottom) * 12;
  int aprsTopX = tft.width() - aprsTopWidth - 4;
  int aprsBottomX = tft.width() - aprsBottomWidth - 4;
  tft.setCursor(aprsTopX, tft.height() - 36);
  tft.print(aprsTitleTop);
  tft.setCursor(aprsBottomX, tft.height() - 20);
  tft.print(aprsTitleBottom);

  // ZEGAR W PRAWIN GÓRNYM ROGU
  struct tm timeinfo;
  if (getTimeWithTimezone(&timeinfo)) {
    char timeBuffer[6];
    strftime(timeBuffer, 6, "%H:%M", &timeinfo);
    int timeWidth = strlen(timeBuffer) * 12;
    tft.setCursor(tft.width() - timeWidth - 4, 5);
    tft.print(timeBuffer);
  }

  drawScreen6RadarZoomHints();
}

static String extractXmlTagValue(const String &xml, const char* tag) {
  String openTag = "<" + String(tag) + ">";
  String closeTag = "</" + String(tag) + ">";
  int start = xml.indexOf(openTag);
  if (start < 0) return "";
  start += openTag.length();
  int end = xml.indexOf(closeTag, start);
  if (end < 0) return "";
  return xml.substring(start, end);
}

static String extractBandCondition(const String &xml, const char* bandName, const char* timeName) {
  String token = "<band name=\"" + String(bandName) + "\" time=\"" + String(timeName) + "\">";
  int start = xml.indexOf(token);
  if (start < 0) return "";
  start += token.length();
  int end = xml.indexOf("</band>", start);
  if (end < 0) return "";
  return xml.substring(start, end);
}

static void setPropagationBandDefaults(PropagationData &out) {
  out.hfBandLabel[0] = "80m-40m";
  out.hfBandFreq[0] = "3.5-7.3 MHz";
  out.hfBandLabel[1] = "30m-20m";
  out.hfBandFreq[1] = "10.1-14.35 MHz";
  out.hfBandLabel[2] = "17m-15m";
  out.hfBandFreq[2] = "18.068-21.45 MHz";
  out.hfBandLabel[3] = "12m-10m";
  out.hfBandFreq[3] = "24.89-29.7 MHz";

  for (int i = 0; i < 4; i++) {
    out.hfBandDay[i] = "--";
    out.hfBandNight[i] = "--";
  }
}

// Konwertuje czas UTC z hamqsl.com (format: "DD Mon YYYY at HHMM UTC") na czas lokalny
static String convertPropagationTimeToLocal(const String &utcTime) {
  if (utcTime.length() == 0 || utcTime == "--") return utcTime;
  
  // Sprawdź czy zawiera "UTC" - jeśli nie, zwróć oryginał
  int utcPos = utcTime.indexOf("UTC");
  if (utcPos < 0) return utcTime;
  
  // Sprawdź czy zawiera "at" - format: "DD Mon YYYY at HHMM UTC"
  int atPos = utcTime.indexOf(" at ");
  if (atPos < 0) return utcTime;
  
  // Wyciągnij czas HHMM
  int timeStart = atPos + 4; // Po " at "
  String timePart = utcTime.substring(timeStart, utcPos - 1); // -1 aby usunąć spację przed UTC
  timePart.trim();
  
  if (timePart.length() < 4) return utcTime;
  
  // Parsuj godzinę i minutę
  int hour = timePart.substring(0, 2).toInt();
  int minute = timePart.substring(2, 4).toInt();
  
  // Dodaj offset strefy czasowej i DST
  int dstOffset = 0;
  time_t now = time(nullptr);
  if (now > 100000) {
    struct tm utcTm;
    gmtime_r(&now, &utcTm);
    dstOffset = isEuropeanDST(&utcTm) ? 1 : 0;
  }
  int localHour = hour + timezoneHours + dstOffset;
  
  // Obsługa przejścia przez północ
  while (localHour < 0) localHour += 24;
  while (localHour >= 24) localHour -= 24;
  
  // Sformatuj lokalny czas
  String hh = (localHour < 10 ? "0" : "") + String(localHour);
  String mm = (minute < 10 ? "0" : "") + String(minute);
  
  // Zwróć sformatowany czas lokalny zachowując datę
  String datePart = utcTime.substring(0, atPos);
  return datePart + " " + hh + mm + " LOC";
}

static bool parsePropagationXml(const String &xml, PropagationData &out) {
  setPropagationBandDefaults(out);

  String sfi = extractXmlTagValue(xml, "solarflux");
  String ssn = extractXmlTagValue(xml, "sunspots");
  String kindex = extractXmlTagValue(xml, "kindex");
  String aindex = extractXmlTagValue(xml, "aindex");
  String xray = extractXmlTagValue(xml, "xray");
  String muf = extractXmlTagValue(xml, "muf");
  String updated = extractXmlTagValue(xml, "updated");

  sfi.trim();
  ssn.trim();
  kindex.trim();
  aindex.trim();
  xray.trim();
  muf.trim();
  updated.trim();

  if (sfi.length() == 0 || kindex.length() == 0) {
    return false;
  }

  out.sfi = sfi;
  out.ssn = ssn.length() ? ssn : "--";
  out.kindex = kindex;
  out.aindex = aindex.length() ? aindex : "--";
  out.xray = xray.length() ? xray : "--";
  out.muf = muf.length() ? muf : "--";
  out.updated = updated.length() ? convertPropagationTimeToLocal(updated) : "--";
  out.valid = true;
  out.lastError = "";
  out.fetchedAtMs = millis();

  for (int i = 0; i < 4; i++) {
    String day = extractBandCondition(xml, out.hfBandLabel[i].c_str(), "day");
    String night = extractBandCondition(xml, out.hfBandLabel[i].c_str(), "night");
    day.trim();
    night.trim();
    if (day.length() > 0) out.hfBandDay[i] = day;
    if (night.length() > 0) out.hfBandNight[i] = night;
  }

  return true;
}

bool fetchPropagationData() {
  if (WiFi.status() != WL_CONNECTED) {
    propagationData.lastError = "WiFi offline";
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(client, PROPAGATION_URL)) {
    propagationData.lastError = "HTTP begin failed";
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    propagationData.lastError = "HTTP " + String(httpCode);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  if (!parsePropagationXml(payload, propagationData)) {
    propagationData.lastError = "Parse error";
    propagationData.valid = false;
    return false;
  }

  return true;
}

void updateScreen3Clock() {
  // ZEGAR WYŁĄCZONY - usunięty z nagłówka propagacji (Sun Spots)
  return;
}

uint32_t computeScreen3Signature() {
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  hash ^= (uint32_t)propagationData.valid;
  hash *= fnvPrime;

  const String* fields[] = {
    &propagationData.sfi,
    &propagationData.kindex,
    &propagationData.aindex,
    &propagationData.updated,
    &propagationData.lastError
  };
  for (const String* field : fields) {
    for (size_t i = 0; i < field->length(); i++) {
      hash ^= (uint8_t)(*field)[i];
      hash *= fnvPrime;
    }
  }

  return hash;
}

static uint16_t conditionColor(String cond) {
  String up = cond;
  up.toUpperCase();
  if (up.indexOf("GOOD") >= 0) return TFT_GREEN;
  if (up.indexOf("FAIR") >= 0) return TFT_YELLOW;
  if (up.indexOf("POOR") >= 0) return TFT_RED;
  return TFT_LIGHTGREY;
}

void drawSunSpotsBody() {
  const int bodyTop = 32;
  const int bodyBottom = 220;
  tft.fillRect(0, bodyTop, 480, bodyBottom - bodyTop, TFT_BLACK);

  String sfiText = propagationData.valid && propagationData.sfi.length() ? propagationData.sfi : "--";
  String kText = propagationData.valid && propagationData.kindex.length() ? propagationData.kindex : "--";
  String aText = propagationData.valid && propagationData.aindex.length() ? propagationData.aindex : "--";
  String updatedText = propagationData.valid && propagationData.updated.length() ? propagationData.updated : "--";

  tft.setTextSize(2);
  tft.setTextColor(TFT_DARKGREY);
  tft.setCursor(10, 52);
  tft.print("SFI");

  tft.setTextSize(4);
  tft.setTextColor(TFT_GREEN);
  int sfiWidth = sfiText.length() * 24;
  tft.setCursor(460 - sfiWidth, 44);
  tft.print(sfiText);

  tft.setTextSize(2);
  tft.setTextColor(TFT_DARKGREY);
  tft.setCursor(10, 100);
  tft.print("K-INDEX");

  float kVal = kText.length() ? kText.toFloat() : -1.0f;
  uint16_t kColor = TFT_LIGHTGREY;
  if (kVal >= 0.0f) {
    kColor = (kVal < 4.0f) ? TFT_GREEN : (kVal < 6.0f) ? TFT_ORANGE : TFT_RED;
  }
  tft.setTextSize(4);
  tft.setTextColor(kColor);
  int kWidth = kText.length() * 24;
  tft.setCursor(460 - kWidth, 92);
  tft.print(kText);

  tft.setTextSize(2);
  tft.setTextColor(TFT_DARKGREY);
  tft.setCursor(10, 148);
  tft.print("A-INDEX");

  tft.setTextSize(4);
  tft.setTextColor(TFT_CYAN);
  int aWidth = aText.length() * 24;
  tft.setCursor(460 - aWidth, 140);
  tft.print(aText);

  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY);
  tft.setCursor(10, 190);
  tft.print("UPDATED");
  tft.setTextSize(2);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(10, 202);
  tft.print(updatedText);

  if (!propagationData.valid) {
    tft.setTextSize(1);
    tft.setTextColor(TFT_RED);
    tft.setCursor(10, 176);
    String err = propagationData.lastError.length() ? propagationData.lastError : tr(TR_NO_DATA);
    tft.print(sanitizePolishToAscii(String(tr(TR_ERROR_PREFIX))));
    tft.print(err);
  }
}

void updateScreen3Data() {
  if (!tftInitialized || currentScreen != 3 || inMenu) {
    return;
  }

  updateScreen3Clock();

  static uint32_t lastSig = 0;
  uint32_t currentSig = computeScreen3Signature();
  if (currentSig == lastSig) {
    return;
  }
  lastSig = currentSig;

  drawSunSpotsBody();
}

void drawSunSpots() {
  tft.fillScreen(TFT_BLACK);

  tft.fillRect(0, 0, 480, 32, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);

  // brak hamburger menu na ekranie Propagation

  tft.setTextSize(2);
  tft.setCursor(35, 8);
  tft.print((tftLanguage == TFT_LANG_EN) ? "PROPAGATION" : "PROPAGACJA");

  // Ikona WiFi w górnym prawym rogu
  drawWifiSignalBars(445, 4);

  // Wyłączono - zegar tylko na górnym pasku
  /*
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1)) {
    char timeBuffer[10];
    strftime(timeBuffer, 10, "%H:%M Z", &timeinfo);
    int timeWidth = strlen(timeBuffer) * 12;
    tft.setCursor(460 - timeWidth, 8);
    tft.print(timeBuffer);
  }
  */

  drawSunSpotsBody();

  // Strzałki nawigacyjne - duże, blisko krawędzi (takie same na wszystkich ekranach)
  int arrowY = 290;
  int arrowSize = 12;
  tft.fillTriangle(15, arrowY, 15 + arrowSize, arrowY - arrowSize, 15 + arrowSize, arrowY + arrowSize, TFT_RADIO_ORANGE);
  tft.fillTriangle(465, arrowY, 465 - arrowSize, arrowY - arrowSize, 465 - arrowSize, arrowY + arrowSize, TFT_RADIO_ORANGE);
  tft.setTextColor(0x52AA);
  tft.setTextSize(1);
  tft.setCursor(195, 286);
  tft.print("SWITCH SCREEN");
}

void updateScreen4Clock() {
  // ZEGAR WYŁĄCZONY - usunięty z nagłówka propagacji (Sun Spots)
  return;
}

uint32_t computeScreen4Signature() {
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  hash ^= (uint32_t)propagationData.valid;
  hash *= fnvPrime;

  for (int i = 0; i < 4; i++) {
    const String* fields[] = {
      &propagationData.hfBandLabel[i],
      &propagationData.hfBandFreq[i],
      &propagationData.hfBandDay[i],
      &propagationData.hfBandNight[i]
    };
    for (const String* field : fields) {
      for (size_t j = 0; j < field->length(); j++) {
        hash ^= (uint8_t)(*field)[j];
        hash *= fnvPrime;
      }
    }
  }

  for (size_t j = 0; j < propagationData.updated.length(); j++) {
    hash ^= (uint8_t)propagationData.updated[j];
    hash *= fnvPrime;
  }
  for (size_t j = 0; j < propagationData.lastError.length(); j++) {
    hash ^= (uint8_t)propagationData.lastError[j];
    hash *= fnvPrime;
  }

  return hash;
}

void drawBandInfoBody() {
  const int bodyTop = 45; // Zaczynamy niżej, pod linią
  const int bodyBottom = 280;
  tft.fillRect(0, bodyTop, 480, bodyBottom - bodyTop, TFT_BLACK);

  // === GÓRNA SEKCJA - INDEKSY SŁONECZNE (3 kolumny) ===
  int yPos = bodyTop + 5;
  int lineHeight = 22;

  // Pierwsza linia: SFI | A-idx | MUF (w ramce)
  tft.setTextSize(2);

  // SFI
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(10, yPos);
  tft.print("SFI:");
  tft.setTextColor(TFT_WHITE);
  tft.print(propagationData.sfi.length() ? propagationData.sfi : "--");

  // A-idx (środek)
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(130, yPos);
  tft.print("A-idx:");
  tft.setTextColor(TFT_WHITE);
  tft.print(propagationData.aindex.length() ? propagationData.aindex : "--");

  // MUF w niebieskiej ramce z żółtym tekstem
  tft.fillRect(280, yPos - 2, 145, 22, TFT_BLUE);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(290, yPos);
  String mufText = propagationData.muf.length() ? propagationData.muf : "--";
  tft.print("MUF:" + mufText + "MHz");

  // Druga linia: SSN | K-idx | X-Ray (w ramce)
  yPos += lineHeight;

  // SSN
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(10, yPos);
  tft.print("SSN:");
  tft.setTextColor(TFT_WHITE);
  tft.print(propagationData.ssn.length() ? propagationData.ssn : "--");

  // K-idx (środek)
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(130, yPos);
  tft.print("K-idx:");
  tft.setTextColor(TFT_WHITE);
  tft.print(propagationData.kindex.length() ? propagationData.kindex : "--");

  // X-Ray w niebieskiej ramce z żółtym tekstem
  tft.fillRect(280, yPos - 2, 100, 22, TFT_BLUE);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(290, yPos);
  String xrayText = propagationData.xray.length() ? propagationData.xray : "--";
  tft.print("X-Ray:" + xrayText);

  // === ŚRODKOWA SEKCJA - TABELA PASMA (lewa strona) ===
  yPos += lineHeight + 10;
  int tableTop = yPos;

  // Ramka wokół tabeli
  tft.drawRoundRect(10, tableTop - 5, 260, 140, 5, TFT_DARKGREY);

  // Nagłówki tabeli
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(15, yPos);
  tft.print("HF BAND");
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(105, yPos);
  tft.print("DAY");
  tft.setCursor(175, yPos);
  tft.print("NIGHT");
  tft.drawFastHLine(12, yPos + 16, 256, TFT_DARKGREY);

  yPos += 22;

  // Wiersze tabeli (4 pasma)
  int rowHeight = 30;
  for (int i = 0; i < 4; i++) {
    // Podświetlenie co drugi wiersz (w ramce)
    if (i % 2 == 0) {
      tft.fillRect(12, yPos - 2, 256, rowHeight - 2, 0x1082); // Ciemnoszary
    }

    tft.setTextSize(2);

    // BAND label
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(15, yPos);
    tft.print(propagationData.hfBandLabel[i]);

    // Day - kolor w zależności od warunku
    String dayCond = propagationData.hfBandDay[i];
    tft.setTextColor(conditionColor(dayCond));
    tft.setCursor(105, yPos);
    tft.print(dayCond.length() ? dayCond : "--");

    // Night
    String nightCond = propagationData.hfBandNight[i];
    tft.setTextColor(conditionColor(nightCond));
    tft.setCursor(175, yPos);
    tft.print(nightCond.length() ? nightCond : "--");

    yPos += rowHeight;
  }

  // === PRAWA STRONA - S/N, Aurora, Ikona ===
  // Pozycja ikony (bardziej w lewo i niżej)
  int iconX = 340;
  int iconY = tableTop + 90;
  int iconSize = 50; // Zakładamy rozmiar ikony 50x50

  // S/N - wyśrodkowane nad ikoną
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN);
  String snText = "S/N:";
  String snVal = "S1-S2";
  int snWidth = tft.textWidth(snText) + tft.textWidth(snVal);
  int snX = iconX + (iconSize - snWidth) / 2;
  tft.setCursor(snX, iconY - 55);
  tft.print(snText);
  tft.setTextColor(TFT_WHITE);
  tft.print(snVal);

  // Aurora - wyśrodkowane nad ikoną
  String auroraText = "Aurora:";
  String auroraVal = "2";
  int auroraWidth = tft.textWidth(auroraText) + tft.textWidth(auroraVal);
  int auroraX = iconX + (iconSize - auroraWidth) / 2;
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(auroraX, iconY - 35);
  tft.print(auroraText);
  tft.setTextColor(TFT_WHITE);
  tft.print(auroraVal);

  // === IKONA PROPAGACJI (słońce/chmura) ===
  int kIndexVal = propagationData.kindex.toInt();
  String propIcon;
  if (kIndexVal < 3) {
    propIcon = "/icon50/800.bmp"; // Słońce - dobre warunki
  } else if (kIndexVal < 5) {
    propIcon = "/icon50/801.bmp"; // Częściowe zachmurzenie - średnie
  } else {
    propIcon = "/icon50/200.bmp"; // Burza - słabe warunki
  }
  if (littleFsReady && LittleFS.exists(propIcon)) {
    drawBmpFromFS(propIcon, iconX, iconY);
  }

  // === DOLNA SEKCJA - INFO O ZAMKNIĘTYCH PASMACH ===
  yPos = tableTop + 145; // Pod tabelą
  tft.setTextSize(1);
  tft.setTextColor(TFT_RED);
  tft.setCursor(10, yPos);
  tft.print("Very High Band Closed");
  tft.setCursor(155, yPos);
  tft.print("Es Europe Band Closed");

  yPos += 14;
  tft.setCursor(10, yPos);
  tft.print("Es Asia Band Closed");
  tft.setCursor(155, yPos);
  tft.print("Es Afr Band Closed");

  // === STATUS BŁĘDU ===
  if (!propagationData.valid) {
    tft.setTextSize(1);
    tft.setTextColor(TFT_RED);
    tft.setCursor(10, bodyBottom - 20);
    String err = propagationData.lastError.length() ? propagationData.lastError : tr(TR_NO_DATA);
    tft.print(sanitizePolishToAscii(String(tr(TR_ERROR_PREFIX))));
    tft.print(err);
  }
}

void updateScreen4Data() {
  if (!tftInitialized || currentScreen != 4 || inMenu) {
    return;
  }

  // Wyłączono - zegar tylko na górnym pasku
  // updateScreen4Clock();

  static uint32_t lastSig = 0;
  uint32_t currentSig = computeScreen4Signature();
  if (currentSig == lastSig) {
    return;
  }
  lastSig = currentSig;

  drawBandInfoBody();
}

void drawBandInfo() {
  tft.fillScreen(TFT_BLACK);

  // Nagłówek w kolorze żółtym/złotym na czarnym tle (jak na zdjęciu)
  tft.setTextSize(2);
  tft.setTextColor(TFT_GOLD);
  tft.setCursor(120, 10);
  tft.print("SOLAR PROPAGATION INDEX");

  // Ikona WiFi w górnym prawym rogu
  drawWifiSignalBars(445, 4);

  // Linia pod nagłówkiem
  tft.drawLine(10, 35, 470, 35, TFT_DARKGREY);

  // ZEGAR - rysowany osobno przez updateBandInfoClock()

  drawBandInfoBody();

  // Strzałki nawigacyjne - duże, blisko krawędzi (takie same na wszystkich ekranach)
  int arrowY = 290;
  int arrowSize = 12;
  tft.fillTriangle(15, arrowY, 15 + arrowSize, arrowY - arrowSize, 15 + arrowSize, arrowY + arrowSize, TFT_RADIO_ORANGE);
  tft.fillTriangle(465, arrowY, 465 - arrowSize, arrowY - arrowSize, 465 - arrowSize, arrowY + arrowSize, TFT_RADIO_ORANGE);
  tft.setTextColor(0x52AA);
  tft.setTextSize(1);
  tft.setCursor(195, 286);
  tft.print("SWITCH SCREEN");
}

bool fetchWeatherForecast(double lat, double lon);

bool fetchWeatherData() {
  if (WiFi.status() != WL_CONNECTED) {
    weatherData.lastError = "WiFi offline";
    weatherData.valid = false;
    return false;
  }
  if (weatherApiKey.length() == 0) {
    weatherData.lastError = "No API key";
    weatherData.valid = false;
    return false;
  }

  double lat = 0.0;
  double lon = 0.0;
  if (userLatLonValid) {
    lat = userLat;
    lon = userLon;
  } else if (userLocator.length() >= 4) {
    locatorToLatLon(userLocator, lat, lon);
  } else {
    weatherData.lastError = "No locator";
    weatherData.valid = false;
    return false;
  }

  // Reset prognoz przed nowym pobraniem
  weatherData.forecast3hValid = false;
  weatherData.forecastNextDayValid = false;
  for (uint8_t i = 0; i < WeatherData::DETAIL_COLS; i++) {
    weatherData.detailValid[i] = false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  String langParam = (tftLanguage == TFT_LANG_EN) ? "en" : "pl";
  String url = "https://api.openweathermap.org/data/2.5/weather?lat=" +
               String(lat, 4) + "&lon=" + String(lon, 4) +
               "&appid=" + weatherApiKey + "&units=metric&lang=" + langParam;

  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(client, url)) {
    weatherData.lastError = "HTTP begin failed";
    weatherData.valid = false;
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    weatherData.lastError = "HTTP " + String(httpCode);
    weatherData.valid = false;
    http.end();
    return false;
  }

  WiFiClient *stream = http.getStreamPtr();

  DynamicJsonDocument filter(512);
  filter["cod"] = true;
  filter["message"] = true;
  filter["name"] = true;
  filter["weather"][0]["description"] = true;
  filter["weather"][0]["icon"] = true;
  filter["weather"][0]["id"] = true;
  filter["main"]["temp"] = true;
  filter["main"]["humidity"] = true;
  filter["main"]["pressure"] = true;
  filter["wind"]["speed"] = true;

  DynamicJsonDocument doc(1536);
  DeserializationError err = deserializeJson(doc, *stream, DeserializationOption::Filter(filter));
  http.end();
  if (err) {
    weatherData.lastError = "JSON error";
    weatherData.valid = false;
    return false;
  }

  int cod = doc["cod"] | 0;
  if (cod != 200) {
    String msg = doc["message"] | "";
    weatherData.lastError = msg.length() ? msg : "API error";
    weatherData.valid = false;
    return false;
  }

  String desc = doc["weather"][0]["description"] | "";
  String cityName = doc["name"] | "";
  String icon = doc["weather"][0]["icon"] | ""; // 01d / 01n itp. do wykrycia pory dnia
  int weatherId = doc["weather"][0]["id"] | 800; // pobieranie kodu pogody
  float temp = doc["main"]["temp"] | 0.0f;
  int humidity = doc["main"]["humidity"] | 0;
  int pressure = doc["main"]["pressure"] | 0;
  float wind = doc["wind"]["speed"] | 0.0f;


  if (desc.equalsIgnoreCase("zachmurzenie umiarkowane")) {
    desc = "zachmurzenie";
  }

  weatherData.description = desc.length() ? desc : "--";
  weatherData.cityName = cityName;
  weatherData.iconCode = icon;
  weatherData.weatherId = weatherId; // kod pogody
  weatherData.tempC = temp;
  weatherData.humidity = humidity;
  weatherData.pressure = pressure;
  weatherData.windMs = wind;

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1)) {
    char timeBuf[6];
    strftime(timeBuf, sizeof(timeBuf), "%H:%M", &timeinfo);
    weatherData.updated = String(timeBuf);
  } else {
    weatherData.updated = "--:--";
  }

  weatherData.valid = true;
  weatherData.lastError = "";
  weatherData.fetchedAtMs = millis();

  // Prognozy (3h i jutro)
  fetchWeatherForecast(lat, lon);

  // Pobierz dane o jakoÄąâ€şci powietrza (PM2.5 i PM10)
  fetchAirPollutionData(lat, lon);

  return true;
}

bool fetchWeatherForecast(double lat, double lon) {
  if (WiFi.status() != WL_CONNECTED || weatherApiKey.length() == 0) {
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  String langParam = (tftLanguage == TFT_LANG_EN) ? "en" : "pl";
  String url = "https://api.openweathermap.org/data/2.5/forecast?lat=" +
               String(lat, 4) + "&lon=" + String(lon, 4) +
               "&appid=" + weatherApiKey + "&units=metric&lang=" + langParam + "&cnt=40";

  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(client, url)) {
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    http.end();
    return false;
  }

  WiFiClient *stream = http.getStreamPtr();

  DynamicJsonDocument filter(1024);
  filter["list"][0]["dt"] = true;
  filter["list"][0]["main"]["temp"] = true;
  filter["list"][0]["main"]["humidity"] = true;
  filter["list"][0]["wind"]["speed"] = true;
  filter["list"][0]["weather"][0]["description"] = true;
  filter["list"][0]["weather"][0]["id"] = true;
  filter["list"][0]["weather"][0]["icon"] = true;
  filter["city"]["name"] = true;

  DynamicJsonDocument doc(16384);
  DeserializationError err = deserializeJson(doc, *stream, DeserializationOption::Filter(filter));
  http.end();
  if (err) {
    return false;
  }

  JsonArray list = doc["list"].as<JsonArray>();
  if (list.isNull() || list.size() == 0) {
    return false;
  }

  if (weatherData.cityName.length() == 0) {
    String cityFromForecast = doc["city"]["name"] | "";
    if (cityFromForecast.length() > 0) {
      weatherData.cityName = cityFromForecast;
    }
  }

  const long nowUnix = (long)time(nullptr);
  const long targetOffsetsSec[WeatherData::DETAIL_COLS] = {
    3L * 3600L,
    6L * 3600L,
    24L * 3600L,
    48L * 3600L,
    72L * 3600L
  };
  const int fallbackIdx[WeatherData::DETAIL_COLS] = {0, 1, 8, 16, 24};
  
  // Uwzględnij DST (czas letni) w obliczeniach strefy czasowej
  struct tm utcTm;
  time_t nowTimeT = (time_t)nowUnix;
  gmtime_r(&nowTimeT, &utcTm);
  int dstOffset = isEuropeanDST(&utcTm) ? 1 : 0;
  const long timezoneOffsetSec = (long)(timezoneHours + dstOffset) * 3600L;
  
  const long nowLocalUnix = nowUnix + timezoneOffsetSec;
  const long baseLocalDayStart = (nowLocalUnix / 86400L) * 86400L;

  weatherData.nightTempValid[0] = false;
  weatherData.nightTempValid[1] = false;

  auto assignSlot = [&](uint8_t slot, JsonObject entry) {
    weatherData.detailTempC[slot] = entry["main"]["temp"] | 0.0f;
    weatherData.detailHumidity[slot] = entry["main"]["humidity"] | 0;
    weatherData.detailWindMs[slot] = entry["wind"]["speed"] | 0.0f;
    weatherData.detailWeatherId[slot] = entry["weather"][0]["id"] | 800;
    weatherData.detailIconCode[slot] = String((const char *)(entry["weather"][0]["icon"] | ""));
    weatherData.detailValid[slot] = true;
  };

  for (uint8_t slot = 0; slot < WeatherData::DETAIL_COLS; slot++) {
    int chosenIdx = -1;

    // Dla slotów 0 i 1 (+3h, +6h) - znajdź najbliższy czas do target
    // Dla slotów 2+ (dni) - znajdź najlepsze dopasowanie dnia (południe)
    if (slot >= 2 && nowUnix > 100000L) {
      const long targetDayStartLocal = baseLocalDayStart + (long)(slot - 1) * 86400L;
      const long targetNoonLocal = targetDayStartLocal + 12L * 3600L;
      long bestScore = 0x7FFFFFFF;

      for (uint16_t i = 0; i < list.size(); i++) {
        JsonObject entry = list[i];
        long dt = entry["dt"] | 0L;
        if (dt <= 0L) {
          continue;
        }

        long dtLocal = dt + timezoneOffsetSec;
        long entryDayStartLocal = (dtLocal / 86400L) * 86400L;
        if (entryDayStartLocal != targetDayStartLocal) {
          continue;
        }

        int localHour = (int)((dtLocal % 86400L) / 3600L);
        bool isDayHour = (localHour >= 9 && localHour <= 18);
        String iconCode = String((const char *)(entry["weather"][0]["icon"] | ""));
        bool iconIsDay = iconCode.endsWith("d");
        bool isDayCandidate = isDayHour || iconIsDay;

        long score = labs(dtLocal - targetNoonLocal);
        if (!isDayCandidate) {
          score += 12L * 3600L;
        }

        if (score < bestScore) {
          bestScore = score;
          chosenIdx = (int)i;
        }
      }
    }

    // Dla slotów 0 i 1 (+3h, +6h) lub jeśli nie znaleziono dopasowania dnia
    if (chosenIdx < 0 && nowUnix > 100000L) {
      long target = nowUnix + targetOffsetsSec[slot];
      long bestDiff = 0x7FFFFFFF;

      for (uint16_t i = 0; i < list.size(); i++) {
        JsonObject entry = list[i];
        long dt = entry["dt"] | 0L;
        if (dt <= 0L) {
          continue;
        }
        long diff = labs(dt - target);
        if (diff < bestDiff) {
          bestDiff = diff;
          chosenIdx = (int)i;
        }
      }
    }

    // Fallback gdy czas niezsynchronizowany lub brak dopasowania - użyj indeksów fallback
    if (chosenIdx < 0) {
      int idx = fallbackIdx[slot];
      if (idx < (int)list.size()) {
        chosenIdx = idx;
      } else if (list.size() > 0) {
        chosenIdx = list.size() - 1;  // Ostatni dostępny element
      }
    }

    if (chosenIdx >= 0 && chosenIdx < (int)list.size()) {
      assignSlot(slot, list[chosenIdx]);
    }
  }

  for (uint8_t daySlot = 0; daySlot < 2; daySlot++) {
    const long targetDayStartLocal = baseLocalDayStart + (long)(daySlot + 1) * 86400L;
    int bestNightIdx = -1;
    long bestNightScore = 0x7FFFFFFF;
    int bestAnyIdx = -1;
    float bestAnyTemp = 1000.0f;

    for (uint16_t i = 0; i < list.size(); i++) {
      JsonObject entry = list[i];
      long dt = entry["dt"] | 0L;
      if (dt <= 0L) {
        continue;
      }

      long dtLocal = dt + timezoneOffsetSec;
      long entryDayStartLocal = (dtLocal / 86400L) * 86400L;
      if (entryDayStartLocal != targetDayStartLocal) {
        continue;
      }

      float temp = entry["main"]["temp"] | 0.0f;
      if (bestAnyIdx < 0 || temp < bestAnyTemp) {
        bestAnyTemp = temp;
        bestAnyIdx = (int)i;
      }

      int localHour = (int)((dtLocal % 86400L) / 3600L);
      bool isNightHour = (localHour <= 6 || localHour >= 21);
      if (!isNightHour) {
        continue;
      }

      int refHour = (localHour >= 21) ? 24 + localHour : localHour;
      int refTarget = 26; // 02:00 w oknie 21..30
      long score = labs((long)refHour - (long)refTarget);
      if (score < bestNightScore) {
        bestNightScore = score;
        bestNightIdx = (int)i;
      }
    }

    int chosenNightIdx = (bestNightIdx >= 0) ? bestNightIdx : bestAnyIdx;
    if (chosenNightIdx >= 0 && chosenNightIdx < (int)list.size()) {
      JsonObject entry = list[chosenNightIdx];
      weatherData.nightTempC[daySlot] = entry["main"]["temp"] | 0.0f;
      weatherData.nightTempValid[daySlot] = true;
    }
  }

  if (weatherData.detailValid[0]) {
    weatherData.forecast3hTempC = weatherData.detailTempC[0];
    weatherData.forecast3hWindMs = weatherData.detailWindMs[0];
    weatherData.forecast3hDesc = "--";
    weatherData.forecast3hValid = true;
  }

  if (weatherData.detailValid[2]) {
    weatherData.forecastNextDayTempC = weatherData.detailTempC[2];
    weatherData.forecastNextDayWindMs = weatherData.detailWindMs[2];
    weatherData.forecastNextDayDesc = "--";
    weatherData.forecastNextDayValid = true;
  }

  bool anyDetailValid = false;
  for (uint8_t i = 0; i < WeatherData::DETAIL_COLS; i++) {
    if (weatherData.detailValid[i]) {
      anyDetailValid = true;
      break;
    }
  }

  return anyDetailValid;
}

bool fetchAirPollutionData(double lat, double lon) {
  if (WiFi.status() != WL_CONNECTED || weatherApiKey.length() == 0) {
    Serial.println("[AIR] No WiFi or API key");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  String url = "https://api.openweathermap.org/data/2.5/air_pollution?lat=" +
               String(lat, 4) + "&lon=" + String(lon, 4) +
               "&appid=" + weatherApiKey;

  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(client, url)) {
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    http.end();
    return false;
  }

  WiFiClient *stream = http.getStreamPtr();

  DynamicJsonDocument filter(256);
  filter["list"][0]["components"]["pm2_5"] = true;
  filter["list"][0]["components"]["pm10"] = true;

  DynamicJsonDocument doc(768);
  DeserializationError err = deserializeJson(doc, *stream, DeserializationOption::Filter(filter));
  http.end();
  if (err) {
    return false;
  }

  // Pobierz wartoÄąâ€şci PM2.5 i PM10 z komponentÄ‚łw
  JsonArray list = doc["list"].as<JsonArray>();
  if (list.isNull() || list.size() == 0) {
    return false;
  }

  JsonObject listItem = list[0];
  JsonObject components = listItem["components"].as<JsonObject>();
  if (components.isNull()) {
    Serial.println("[AIR] No components data");
    return false;
  }

  weatherData.pm25 = components["pm2_5"] | 0.0f;
  weatherData.pm10 = components["pm10"] | 0.0f;
  Serial.print("[AIR] PM2.5: ");
  Serial.print(weatherData.pm25);
  Serial.print(", PM10: ");
  Serial.println(weatherData.pm10);
  return true;
}

// Funkcja okreÄąâ€şlajĂ„â€¦ca kolor na podstawie wartoÄąâ€şci PM2.5
uint16_t getPM25Color(float pm25) {
  if (pm25 <= 12) return TFT_GREEN;      // Dobra
  if (pm25 <= 35) return TFT_YELLOW;     // Umiarkowana
  if (pm25 <= 55) return TFT_ORANGE;     // Niezdrowa dla wraÄąÄ˝liwych
  if (pm25 <= 150) return TFT_RED;       // Niezdrowa
  return 0xF800; // Ciemny czerwony - Bardzo niezdrowa
}

// Funkcja okreÄąâ€şlajĂ„â€¦ca kolor na podstawie wartoÄąâ€şci PM10
uint16_t getPM10Color(float pm10) {
  if (pm10 <= 20) return TFT_GREEN;      // Dobra
  if (pm10 <= 50) return TFT_YELLOW;     // Umiarkowana
  if (pm10 <= 100) return TFT_ORANGE;   // Niezdrowa dla wraÄąÄ˝liwych
  if (pm10 <= 200) return TFT_RED;      // Niezdrowa
  return 0xF800; // Ciemny czerwony - Bardzo niezdrowa
}

void updateScreen5Clock() {
  if (!tftInitialized ||
      (currentScreen != SCREEN_WEATHER_DSP && currentScreen != SCREEN_WEATHER_FORECAST) ||
      inMenu) {
    return;
  }

  struct tm timeinfo;
  if (!getTimeWithTimezone(&timeinfo)) {
    return;
  }

  char timeBuffer[6];
  strftime(timeBuffer, 6, "%H:%M", &timeinfo);
  int timeWidth = strlen(timeBuffer) * 12;
  int timeX = 475 - timeWidth;

  tft.fillRect(timeX - 2, 4, timeWidth + 6, 24, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(timeX, 8);
  tft.print(timeBuffer);
}

uint32_t computeScreen5Signature() {
  const uint32_t fnvPrime = 16777619u;
  uint32_t hash = 2166136261u;

  hash ^= (uint32_t)weatherData.valid;
  hash *= fnvPrime;
  bool footerVisible = (currentScreen == SCREEN_WEATHER_FORECAST)
                       ? isTableNavFooterVisible(SCREEN_WEATHER_FORECAST)
                       : isTableNavFooterVisible(SCREEN_WEATHER_DSP);
  hash ^= footerVisible ? 1u : 0u;
  hash *= fnvPrime;

  const String* fields[] = {&weatherData.cityName, &weatherData.description, &weatherData.updated, &weatherData.lastError};
  for (const String* field : fields) {
    for (size_t j = 0; j < field->length(); j++) {
      hash ^= (uint8_t)(*field)[j];
      hash *= fnvPrime;
    }
  }
  hash ^= (uint32_t)(weatherData.tempC * 10);
  hash *= fnvPrime;
  hash ^= (uint32_t)weatherData.humidity;
  hash *= fnvPrime;
  hash ^= (uint32_t)weatherData.pressure;
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.windMs * 10);
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.pm25 * 10);
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.pm10 * 10);
  hash *= fnvPrime;

  // Prognozy
  hash ^= (uint32_t)weatherData.forecast3hValid;
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.forecast3hTempC * 10);
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.forecast3hWindMs * 10);
  hash *= fnvPrime;
  for (size_t j = 0; j < weatherData.forecast3hDesc.length(); j++) {
    hash ^= (uint8_t)weatherData.forecast3hDesc[j];
    hash *= fnvPrime;
  }

  hash ^= (uint32_t)weatherData.forecastNextDayValid;
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.forecastNextDayTempC * 10);
  hash *= fnvPrime;
  hash ^= (uint32_t)(weatherData.forecastNextDayWindMs * 10);
  hash *= fnvPrime;
  for (size_t j = 0; j < weatherData.forecastNextDayDesc.length(); j++) {
    hash ^= (uint8_t)weatherData.forecastNextDayDesc[j];
    hash *= fnvPrime;
  }

  for (uint8_t i = 0; i < WeatherData::DETAIL_COLS; i++) {
    hash ^= (uint32_t)weatherData.detailValid[i];
    hash *= fnvPrime;
    hash ^= (uint32_t)(weatherData.detailTempC[i] * 10);
    hash *= fnvPrime;
    hash ^= (uint32_t)weatherData.detailHumidity[i];
    hash *= fnvPrime;
    hash ^= (uint32_t)(weatherData.detailWindMs[i] * 10);
    hash *= fnvPrime;
    hash ^= (uint32_t)weatherData.detailWeatherId[i];
    hash *= fnvPrime;
    for (size_t j = 0; j < weatherData.detailIconCode[i].length(); j++) {
      hash ^= (uint8_t)weatherData.detailIconCode[i][j];
      hash *= fnvPrime;
    }
  }

  return hash;
}

// Funkcja pomocnicza: Kolor wiatru zaleÄąÄ˝ny od prĂ„â„˘dkoÄąâ€şci (m/s)
uint16_t getWindColor(float speed) {
  if (speed < 5.0)  return TFT_GREEN;       // Bezpiecznie
  if (speed < 10.0) return TFT_YELLOW;      // Umiarkowanie
  if (speed < 15.0) return 0xFBE0;          // Silny (Orange)
  return TFT_RED;                           // Niebezpieczny dla anten
}

// Mapowanie kodu OWM na plik ikony (zgodnie z przygotowaną listą)
String weatherIconPathForId(int id, bool isNight) {
  const String base = "/icon50/";

  // 2xx burza
  if (id >= 200 && id <= 232) return base + "200.bmp";

  // 3xx mżawka
  if (id >= 300 && id <= 321) return base + "300.bmp";

  // 5xx deszcz
  if (id >= 500 && id <= 531) {
    if (id == 500 || id == 501) return base + "500.bmp";
    if (id == 502 || id == 503) return base + "502.bmp";
    if (id == 511 || id == 531) return base + "511.bmp";
    return base + "520.bmp";
  }

  // 6xx śnieg
  if (id >= 600 && id <= 622) {
    if (id == 600 || id == 601 || id == 602 || id == 620 || id == 621 || id == 622) return base + "600.bmp";
    // 611..616 (także 613/615/616) -> 611
    return base + "611.bmp";
  }

  // 7xx atmosfera
  if (id >= 700 && id <= 781) return base + "700.bmp";

  // 800 clear (dzień/noc)
  if (id == 800) return base + (isNight ? "800n.bmp" : "800.bmp");

  // 80x chmury
  if (id == 801 || id == 802) return base + (isNight ? "801n.bmp" : "801.bmp");
  if (id == 803 || id == 804) return base + "803.bmp";

  return base + "unknown.bmp";
}

// Funkcja pomocnicza: Rysowanie ikon pogodowych (priorytet: id -> plik BMP, fallback: opis)
void drawWeatherIcon(int x, int y, int weatherId, String desc) {
  desc.toLowerCase();

  // Dzień/noc z ikony OWM (np. 10d/10n). Jeśli brak, fallback na czas lokalny.
  bool isNight = false;
  if (weatherData.iconCode.endsWith("n")) {
    isNight = true;
  } else if (weatherData.iconCode.endsWith("d")) {
    isNight = false;
  } else {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 1)) {
      isNight = (timeinfo.tm_hour < 6 || timeinfo.tm_hour >= 18);
    }
  }

  String iconFile = weatherIconPathForId(weatherId, isNight);

  // Najpierw próbujemy gotową ikonę BMP
  if (drawBmpFromFS(iconFile, x - 25, y - 25)) {
    return;
  }

  // Fallback: stare ikonki rysowane na podstawie opisu (gdy brak pliku)
  if (desc.indexOf("slon") >= 0 || desc.indexOf("clear") >= 0 || desc.indexOf("pogod") >= 0) {
    tft.fillCircle(x, y, 18, TFT_YELLOW); // Słońce
    for(int i=0; i<360; i+=45) {
       float rad = i * 0.01745;
       tft.drawLine(x+cos(rad)*22, y+sin(rad)*22, x+cos(rad)*32, y+sin(rad)*32, TFT_YELLOW);
    }
  } 
  else if (desc.indexOf("burz") >= 0 || desc.indexOf("thunder") >= 0) {
    tft.fillCircle(x, y-5, 12, TFT_DARKGREY); // Chmura burzowa
    tft.fillTriangle(x+2, y+8, x+10, y+8, x+4, y+22, TFT_YELLOW); // Błysk
  }
  else if (desc.indexOf("mza") >= 0 || desc.indexOf("drizzle") >= 0) {
    tft.fillCircle(x-10, y+5, 12, TFT_LIGHTGREY); // Chmura + mgławka
    tft.fillCircle(x+10, y+5, 12, TFT_LIGHTGREY);
    tft.fillCircle(x, y-5, 15, TFT_LIGHTGREY);
    tft.drawLine(x-6, y+10, x-8, y+16, TFT_CYAN);
    tft.drawLine(x+2, y+10, x, y+16, TFT_CYAN);
  }
  else if (desc.indexOf("snieg") >= 0 || desc.indexOf("snow") >= 0) {
    tft.fillCircle(x, y-5, 12, TFT_LIGHTGREY); // Chmura + śnieg
    tft.drawLine(x-6, y+12, x-2, y+16, TFT_WHITE);
    tft.drawLine(x-2, y+12, x-6, y+16, TFT_WHITE);
    tft.drawLine(x+2, y+12, x+6, y+16, TFT_WHITE);
    tft.drawLine(x+6, y+12, x+2, y+16, TFT_WHITE);
  }
  else if (desc.indexOf("mg") >= 0 || desc.indexOf("mist") >= 0 || desc.indexOf("fog") >= 0 || desc.indexOf("haze") >= 0) {
    tft.drawLine(x-18, y-2, x+18, y-2, TFT_LIGHTGREY); // Mgła
    tft.drawLine(x-20, y+4, x+20, y+4, TFT_LIGHTGREY);
    tft.drawLine(x-16, y+10, x+16, y+10, TFT_LIGHTGREY);
  }
  else if (desc.indexOf("sleet") >= 0 || desc.indexOf("deszcz ze sniegiem") >= 0) {
    tft.fillCircle(x, y-5, 12, TFT_LIGHTGREY); // Deszcz ze śniegiem
    tft.drawLine(x-6, y+10, x-8, y+18, TFT_CYAN);
    tft.drawLine(x+2, y+10, x, y+18, TFT_CYAN);
    tft.drawLine(x+6, y+12, x+2, y+16, TFT_WHITE);
    tft.drawLine(x+2, y+12, x+6, y+16, TFT_WHITE);
  }
  else if (desc.indexOf("zachmur") >= 0 || desc.indexOf("cloud") >= 0 || desc.indexOf("pochmurnie") >= 0) {
    tft.fillCircle(x-10, y+5, 12, TFT_LIGHTGREY); // Chmura
    tft.fillCircle(x+10, y+5, 12, TFT_LIGHTGREY);
    tft.fillCircle(x, y-5, 15, TFT_LIGHTGREY);
  }
  else if (desc.indexOf("deszcz") >= 0 || desc.indexOf("rain") >= 0) {
    tft.fillCircle(x, y-5, 12, TFT_DARKGREY); // Chmura deszczowa
    tft.drawLine(x-5, y+10, x-8, y+20, TFT_CYAN);
    tft.drawLine(x+5, y+10, x+2, y+20, TFT_CYAN);
    tft.drawLine(x, y+12, x-3, y+22, TFT_CYAN);
  } else {
    tft.drawCircle(x, y, 15, TFT_WHITE); // Ikona domyślna (okrąg)
  }
}

// Główna funkcja ciała ekranu pogodowego
void drawWeatherBody() {
  const int bodyTop = 32;
  const int bodyBottom = 220;
  
  // Czyszczenie obszaru roboczego
  tft.fillRect(0, bodyTop, 480, bodyBottom - bodyTop, TFT_BLACK);

  if (!weatherData.valid) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(50, 110);
    tft.print("WAITING FOR DATA...");
    return;
  }

  // 1. IKONA I OPIS (Prawa strona) - dostosowane do 480px
  drawWeatherIcon(400, 70, weatherData.weatherId, weatherData.description);

  // Opis pogody z polskimi znakami, czcionka VLW 20px
  bool fontLoaded = false;
  int descY = 105;
  int descX = 400; // Przesunięte bardziej w prawo dla 480px
  if (littleFsReady && LittleFS.exists(ROBOTO_FONT12_FILE)) {
    tft.loadFont(ROBOTO_FONT12_NAME, LittleFS);
    fontLoaded = true;
    tft.setTextColor(TFT_LIGHTGREY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(weatherData.description, descX, descY);
    tft.setCursor(20, 120);
    tft.print(tr(TR_HUMIDITY));  // Font VLW obsługuje polskie znaki
    tft.setCursor(20, 144);
    tft.print(tr(TR_PRESSURE));
    tft.setCursor(20, 168);
    tft.print(tr(TR_WIND));
    tft.unloadFont();
  } else {
    tft.setTextSize(1);
    tft.setTextColor(TFT_LIGHTGREY);
    String descAscii = sanitizePolishToAscii(weatherData.description);
    int fallbackX = descX - (descAscii.length() * 3);
    tft.setCursor(fallbackX, descY);
    tft.print(descAscii);
    tft.setTextSize(1);
    tft.setTextColor(TFT_DARKGREY);
    tft.setCursor(20, 120);
    tft.print(sanitizePolishToAscii(String(tr(TR_HUMIDITY))));
    tft.setCursor(20, 145);
    tft.print(sanitizePolishToAscii(String(tr(TR_PRESSURE))));
    tft.setCursor(20, 170);
    tft.print(sanitizePolishToAscii(String(tr(TR_WIND))));
  }

  // 2. TEMPERATURA (Lewa strona) - większa dla 480px
  tft.setTextSize(2);
  tft.setTextColor(TFT_DARKGREY);
  tft.setCursor(20, 45);
  tft.print(sanitizePolishToAscii(String(tr(TR_TEMPERATURE))));

  tft.setTextSize(6); // Większa temperatura
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(20, 70);
  tft.print(String(weatherData.tempC, 1));
  tft.setTextSize(3);
  tft.print(" C");

  // 3. PARAMETRY SZCZEGÓŁOWE - dostosowane do 480px
  int startY = 120;
  int valueX = 180; // Więcej miejsca na wartości
  
  // WILGOTNOŚĆ
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(valueX, startY);
  tft.print(String(weatherData.humidity) + "%");

  // CIŚNIENIE
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(valueX, startY + 30);
  tft.print(String(weatherData.pressure) + " hPa");

  // WIATR
  tft.setTextSize(2);
  tft.setTextColor(getWindColor(weatherData.windMs));
  tft.setCursor(valueX, startY + 60);
  tft.print(String(weatherData.windMs, 1) + " m/s");

  // PM2.5 i PM10 - lepsze rozmieszczenie dla 480px
  tft.setTextSize(2);
  
  // PM2.5
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(280, startY);
  tft.print("PM2.5: ");
  uint16_t pm25Color = getPM25Color(weatherData.pm25);
  tft.setTextColor(pm25Color);
  tft.print(String(weatherData.pm25, 1));
  
  // PM10
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(280, startY + 30);
  tft.print("PM10: ");
  uint16_t pm10Color = getPM10Color(weatherData.pm10);
  tft.setTextColor(pm10Color);
  tft.print(String(weatherData.pm10, 1));

}

static void drawWeatherDetailIconCell(int x, int y, int weatherId, const String &iconCode) {
  bool isNight = false;
  if (iconCode.endsWith("n")) {
    isNight = true;
  } else if (iconCode.endsWith("d")) {
    isNight = false;
  } else {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 1)) {
      isNight = (timeinfo.tm_hour < 6 || timeinfo.tm_hour >= 18);
    }
  }

  String iconFile = weatherIconPathForId(weatherId, isNight);
  if (!drawBmpFromFS(iconFile, x - 25, y - 25)) {
    tft.drawCircle(x, y, 10, TFT_LIGHTGREY);
  }
}

static void buildWeatherDetailHeaders(String headers[WEATHER_DETAIL_COLS]) {
  headers[0] = (tftLanguage == TFT_LANG_EN) ? "+3h" : "+3godz";
  headers[1] = (tftLanguage == TFT_LANG_EN) ? "+6h" : "+6godz";

  struct tm timeinfo;
  if (getTimeWithTimezone(&timeinfo)) {
    static const char *daysPl[7] = {"Nd", "Pn", "Wt", "Sr", "Czw", "Pt", "Sob"};
    static const char *daysEn[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    const char **days = (tftLanguage == TFT_LANG_EN) ? daysEn : daysPl;

    for (uint8_t i = 0; i < 3; i++) {
      int dayIdx = (timeinfo.tm_wday + 1 + i) % 7;
      headers[i + 2] = String(days[dayIdx]);
    }
  } else {
    headers[2] = "D+1";
    headers[3] = "D+2";
    headers[4] = "D+3";
  }
}

void drawWeatherDetailPage() {
  const int bodyTop = 32;
  const int bodyBottom = 220;
  const int bodyLeft = 2;
  const int bodyRight = 478;  // Dla wyświetlacza 480px
  const int colCount = WeatherData::DETAIL_COLS;
  const int colW = (bodyRight - bodyLeft + 1) / colCount;

  tft.fillRect(0, bodyTop, 480, bodyBottom - bodyTop, TFT_BLACK);

  if (!weatherData.valid) {
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(12, 96);
    tft.print(weatherData.lastError.length() ? weatherData.lastError : tr(TR_NO_DATA));
    return;
  }

  String headers[WeatherData::DETAIL_COLS];
  buildWeatherDetailHeaders(headers);

  const int yHeader = 38;
  const int yTemp = 78;
  const int yNight = 98;
  const int yHum = 121;
  const int yWind = 152;
  const int yIcon = 183;

  tft.fillRect(bodyLeft, bodyTop + 2, bodyRight - bodyLeft + 1, 31, TFT_TABLE_ALT_ROW_COLOR);

  for (int i = 1; i < colCount; i++) {
    int vx = bodyLeft + i * colW;
    tft.drawFastVLine(vx, bodyTop + 2, bodyBottom - bodyTop - 4, TFT_DARKGREY);
  }

  for (int i = 0; i < colCount; i++) {
    int colX = bodyLeft + i * colW;
    int cx = colX + (colW / 2);

    String hdr = sanitizePolishToAscii(headers[i]);
    tft.setTextSize(2);
    int hdrW = hdr.length() * 12;
    tft.setTextColor(TFT_LIGHTGREY);
    tft.setCursor(cx - hdrW / 2, yHeader);
    tft.print(hdr);

    tft.setTextSize(1);

    if (!weatherData.detailValid[i]) {
      tft.setTextColor(TFT_DARKGREY);
      tft.setCursor(cx - 6, yTemp);
      tft.print("--");
      tft.setCursor(cx - 6, yNight);
      tft.print("--");
      tft.setCursor(cx - 6, yHum);
      tft.print("--");
      tft.setCursor(cx - 6, yWind);
      tft.print("--");
      continue;
    }

    String tempText = String(weatherData.detailTempC[i], 1);
    String humText = String(weatherData.detailHumidity[i]) + "%";
    String windText = String(weatherData.detailWindMs[i], 1) + "m/s";

    tft.setTextSize(2);
    tft.setTextColor(TFT_CYAN);
    int tempX = cx - ((int)tempText.length() * 6);
    tft.setCursor(tempX, yTemp);
    tft.print(tempText);
    tft.drawCircle(tempX + ((int)tempText.length() * 12) + 3, yTemp + 3, 2, TFT_CYAN);

    tft.setTextSize(1);
    tft.setTextColor(TFT_LIGHTGREY);
    String nightText = "--";
    if (i >= 2 && i <= 3 && weatherData.nightTempValid[i - 2]) {
      nightText = String(weatherData.nightTempC[i - 2], 1) + "C";
    }
    tft.setCursor(cx - ((int)nightText.length() * 3), yNight);
    tft.print(nightText);

    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(cx - ((int)humText.length() * 6), yHum);
    tft.print(humText);

    tft.setTextSize(1);
    tft.setTextColor(getWindColor(weatherData.detailWindMs[i]));
    tft.setCursor(cx - ((int)windText.length() * 3), yWind);
    tft.print(windText);

    drawWeatherDetailIconCell(cx - 25, yIcon, weatherData.detailWeatherId[i], weatherData.detailIconCode[i]);
  }
}

static void drawWeatherFooterArea(ScreenType screenId) {
  tft.fillRect(0, 220, 480, 20, TFT_BLACK);

  if (isTableNavFooterVisible(screenId)) {
    drawSwitchScreenFooter();
    return;
  }

  // WIZUALIZACJA JAKOŚCI POWIETRZA (AQI) - nad napisem lokalizacji
  if (weatherData.valid) {
    const int barY = 222; // Pozycja paska AQI (nad tekstem lokalizacji)
    const int barWidth = 100;
    const int barHeight = 6;
    const int barX = 240 - (barWidth / 2); // Wyśrodkowane

    // Kolor na podstawie PM2.5 (gorsze zanieczyszczenie decyduje)
    uint16_t aqiColor = getPM25Color(weatherData.pm25);
    String aqiLabel = "AQI: ";
    if (weatherData.pm25 <= 12) aqiLabel += "DOBRA";
    else if (weatherData.pm25 <= 35) aqiLabel += "UMIARK.";
    else if (weatherData.pm25 <= 55) aqiLabel += "NIEZDROWA";
    else aqiLabel += "SZKODLIWA";

    // Rysuj pasek jakości powietrza
    tft.fillRect(barX, barY, barWidth, barHeight, TFT_DARKGREY);
    tft.fillRect(barX + 1, barY + 1, barWidth - 2, barHeight - 2, aqiColor);

    // Etykieta AQI po lewej stronie paska
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(5, barY);
    tft.print(aqiLabel);

    // Wartości PM2.5/PM10 po prawej stronie
    tft.setTextColor(getPM25Color(weatherData.pm25));
    tft.setCursor(350, barY);
    tft.print("PM2.5:" + String(weatherData.pm25, 0));
    tft.setTextColor(getPM10Color(weatherData.pm10));
    tft.setCursor(420, barY);
    tft.print("PM10:" + String(weatherData.pm10, 0));
  }

  // Tekst lokalizacji (niżej, y=230 zamiast 226 aby zrobić miejsce na AQI)
  String cityLabel = weatherData.cityName;
  cityLabel.trim();
  if (cityLabel.length() == 0) {
    cityLabel = "--";
  }
  cityLabel = sanitizePolishToAscii(cityLabel);

  const String prefix = (tftLanguage == TFT_LANG_EN)
                        ? "Weather for location: "
                        : "Pogoda dla lokalizacji: ";
  const int maxChars = 52;
  int maxCityChars = maxChars - (int)prefix.length();
  if (maxCityChars < 3) {
    maxCityChars = 3;
  }
  if ((int)cityLabel.length() > maxCityChars) {
    cityLabel = cityLabel.substring(0, maxCityChars - 3) + "...";
  }
  String footerLabel = prefix + cityLabel;

  tft.setTextColor(0x52AA);
  tft.setTextSize(1);
  int footerWidth = footerLabel.length() * 6;
  int footerX = (480 - footerWidth) / 2;
  if (footerX < 2) {
    footerX = 2;
  }
  tft.setCursor(footerX, 230);
  tft.print(footerLabel);
}

void updateScreen5Data() {
  if (!tftInitialized ||
      (currentScreen != SCREEN_WEATHER_DSP && currentScreen != SCREEN_WEATHER_FORECAST) ||
      inMenu) {
    return;
  }

  // Wyłączono - zegar rysowany przez updateWeatherClock
  // updateScreen5Clock();

  static uint32_t lastWeatherSig = 0;
  static uint32_t lastForecastSig = 0;
  uint32_t currentSig = computeScreen5Signature();

  if (currentScreen == SCREEN_WEATHER_FORECAST) {
    if (currentSig == lastForecastSig) {
      return;
    }
    lastForecastSig = currentSig;
    drawWeatherDetailPage();
    drawWeatherFooterArea(SCREEN_WEATHER_FORECAST);
    return;
  }

  if (currentSig == lastWeatherSig) {
    return;
  }
  lastWeatherSig = currentSig;

  drawWeatherBody();
  drawWeatherFooterArea(SCREEN_WEATHER_DSP);
}

void drawWeather() {
  tft.fillScreen(TFT_BLACK);

  tft.fillRect(0, 0, 480, 32, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);

  drawHamburgerMenuButton3D(5, 7);

  tft.setTextSize(2);
  tft.setCursor(35, 8);
  tft.print(tr(TR_WEATHER));

  // Ikona WiFi w górnym prawym rogu
  drawWifiSignalBars(445, 4);

  // ZEGAR - rysowany osobno przez updateWeatherClock()

  drawWeatherBody();
  drawWeatherFooterArea(SCREEN_WEATHER_DSP);
}

void drawWeatherForecast() {
  tft.fillScreen(TFT_BLACK);

  tft.fillRect(0, 0, 480, 32, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);

  drawHamburgerMenuButton3D(5, 7);

  const char *detailHeader = (tftLanguage == TFT_LANG_EN) ? "Weather forecast" : "Prognoza pogody";
  tft.setTextSize(2);
  tft.setCursor(35, 8);
  tft.print(detailHeader);

  // Ikona WiFi w górnym prawym rogu
  drawWifiSignalBars(445, 4);

  // ZEGAR - rysowany osobno przez updateWeatherForecastClock()

  drawWeatherDetailPage();
  drawWeatherFooterArea(SCREEN_WEATHER_FORECAST);
}

void setupMatrix() {
  for (int i = 0; i < numDrops; i++) {
    drops[i].x = i * MATRIX_COL_SPACING; // Rozstawienie kolumn
    resetMatrixDropRandom(i);
  }
  matrixInitialized = true;
}

void drawMatrixBackground(int bodyTop, int bodyBottom) {
  const int charStep = 8; // wysokość znaku przy setTextSize(1)
  const int charW = 6;    // szerokość znaku przy setTextSize(1)
  bool anyIntroActive = false;
  for (int i = 0; i < numDrops; i++) {
    int drawX = drops[i].x;
    int drawY = bodyTop + drops[i].y;
    bool dropIntroActive = (matrixIntroActive && drops[i].introActive);

    if (matrixIntroActive && !dropIntroActive && !drops[i].introParticipates) {
      continue;
    }

    if (dropIntroActive) {
      unsigned long introElapsed = millis() - matrixIntroStartMs;
      if (introElapsed >= (unsigned long)drops[i].introDelayMs) {
        unsigned long activeMs = introElapsed;
        float relY = (float)drops[i].introStartY +
                     ((float)((int)activeMs - drops[i].introDelayMs) * drops[i].introSpeedPxPerMs);
        drawY = bodyTop + (int)relY;
      } else {
        drawY = bodyTop + drops[i].introStartY;
      }

      int bodyRelY = drawY - bodyTop;
      if (bodyRelY > (bodyBottom - bodyTop)) {
        drops[i].introActive = false;
        dropIntroActive = false;
        resetMatrixDropRandom(i);
        drawY = bodyTop + drops[i].y;
      } else {
        anyIntroActive = true;
      }
    }

    if (drawX < 0 || drawX > (480 - 6)) {
      if (!dropIntroActive) {
        drops[i].y += drops[i].speed;
        if (drops[i].y > (bodyBottom - bodyTop)) {
          resetMatrixDropRandom(i);
        }
      }
      continue;
    }
    tft.setTextSize(1);
    for (int j = 0; j < drops[i].len; j++) {
      int charY = drawY + (j * charStep);
      if (charY >= bodyTop && charY < bodyBottom) {
        // Blokuj znak tła, jeśli jego prostokąt przecina obszar maski zegara.
        if (drawX < (clockMaskX + clockMaskW) && (drawX + charW) > clockMaskX &&
            charY < (clockMaskY + clockMaskH) && (charY + charStep) > clockMaskY) {
          continue;
        }
        int denom = (drops[i].len > 1) ? (drops[i].len - 1) : 1;
        uint8_t t = (uint8_t)((255 * j) / denom); // 0 (ciemno) -> 255 (jasno)
        // Najniższa litera (głowa) ma być wyraźnie jaśniejsza od drugiej.
        if (j == drops[i].len - 1) {
          t = 255;
        } else if (drops[i].len > 1 && j == drops[i].len - 2) {
          t = min<uint8_t>(t, 135);
        }
        uint16_t color = (j == drops[i].len - 1)
               ? TFT_WHITE
               : lerpColor565(TFT_BLACK, MATRIX_BRIGHTGREEN, t);
        tft.setTextColor(color);
        tft.setCursor(drawX, charY);
        char ch = (j == drops[i].len - 1) ? drops[i].headChar : (char)random(33, 126);
        tft.print(ch);
      }
    }

    if (!dropIntroActive) {
      drops[i].y += drops[i].speed;

      // Reset kropli, gdy wypadnie poza obszar
      if (drops[i].y > (bodyBottom - bodyTop)) {
        resetMatrixDropRandom(i);
      }
    }
  }

  matrixIntroActive = anyIntroActive;
}

void clearMatrixArea(int bodyTop, int bodyBottom) {
  int bodyH = bodyBottom - bodyTop;
  if (clockMaskW <= 0 || clockMaskH <= 0) {
    tft.fillRect(0, bodyTop, SCREEN10_WIDTH, bodyH, TFT_BLACK);
    return;
  }

  int leftW = clockMaskX;
  int rightX = clockMaskX + clockMaskW;
  int rightW = SCREEN10_WIDTH - rightX;
  int topH = clockMaskY - bodyTop;
  int bottomY = clockMaskY + clockMaskH;
  int bottomH = bodyBottom - bottomY;

  if (topH > 0) {
    tft.fillRect(0, bodyTop, SCREEN10_WIDTH, topH, TFT_BLACK);
  }
  if (bottomH > 0) {
    tft.fillRect(0, bottomY, SCREEN10_WIDTH, bottomH, TFT_BLACK);
  }
  if (leftW > 0) {
    tft.fillRect(0, clockMaskY, leftW, clockMaskH, TFT_BLACK);
  }
  if (rightW > 0) {
    tft.fillRect(rightX, clockMaskY, rightW, clockMaskH, TFT_BLACK);
  }
}

void drawMatrixStatic() {
  //tft.fillRect(0, 0, 480, SCREEN10_HEADER_H, TFT_RADIO_ORANGE);
  //tft.setTextColor(TFT_BLACK);
  //tft.setTextSize(2);
 // tft.setCursor(10, 8);
  //tft.print("ZEGAR");

  // StrzaÄąâ€ški nawigacyjne na dole ekranu
  //tft.fillTriangle(10, 230, 20, 222, 20, 238, TFT_RADIO_ORANGE);
  //tft.fillTriangle(310, 230, 300, 222, 300, 238, TFT_RADIO_ORANGE);
  //tft.setTextColor(0x52AA); // Ciemny szary
  //tft.setTextSize(1);
  //tft.setCursor(125, 226);
  //tft.print("SWITCH SCREEN");
}

void drawMatrixFrame() {
  unsigned long now = millis();
  static unsigned long lastMatrixUpdateMs = 0;
  
  // Aktualizuj matrix co 50ms dla płynnego efektu
  if (now - lastMatrixUpdateMs >= 50) {
    lastMatrixUpdateMs = now;
    
    // Rysuj tło matrix na całym ekranie
    drawMatrixBackground(0, SCREEN10_HEIGHT);
  }
  
  // Zegar w stylu Matrix - większy i na środku
  String timeLocal = getTimezoneTimeString("%H:%M:%S", 9);
  const int MATRIX_TEXT_SIZE = 4; // Większy rozmiar
  const int timeCharW = 8 * MATRIX_TEXT_SIZE;
  const int timeCharH = 8 * MATRIX_TEXT_SIZE;
  const int timeWidth = 8 * timeCharW;
  const int timeHeight = timeCharH;
  const int timeX = (SCREEN10_WIDTH - timeWidth) / 2;
  const int timeY = (SCREEN10_HEIGHT - timeHeight) / 2; // Wyśrodkowany
  
  // Rysuj zegar tylko gdy się zmienił
  if (timeLocal != lastClockText) {
    // Wyczyść obszar zegara przed narysowaniem nowego (usuń poprzednie sekundy)
    tft.fillRect(timeX - 5, timeY - 5, timeWidth + 10, timeHeight + 10, TFT_BLACK);
    
    // Efekt świecący - rysuj z cieniem
    tft.setTextSize(MATRIX_TEXT_SIZE);
    
    // Cień/aura wokół tekstu (poświata)
    for (int offset = 3; offset >= 1; offset--) {
      uint16_t glowColor = (offset == 1) ? MATRIX_BRIGHTGREEN : 
                          (offset == 2) ? 0x03E0 : 0x01E0;
      tft.setTextColor(glowColor);
      tft.setCursor(timeX + offset, timeY + offset);
      tft.print(timeLocal);
      tft.setCursor(timeX - offset, timeY + offset);
      tft.print(timeLocal);
      tft.setCursor(timeX + offset, timeY - offset);
      tft.print(timeLocal);
      tft.setCursor(timeX - offset, timeY - offset);
      tft.print(timeLocal);
    }
    
    // Główny tekst - jasnozielony
    tft.setTextColor(MATRIX_BRIGHTGREEN);
    tft.setCursor(timeX, timeY);
    tft.print(timeLocal);
    
    // Data pod zegarem w stylu Matrix
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 1)) {
      char dateStr[20];
      strftime(dateStr, sizeof(dateStr), "%d.%m.%Y", &timeinfo);
      
      int dateTextSize = 2;
      int dateCharW = 8 * dateTextSize;
      int dateWidth = strlen(dateStr) * dateCharW;
      int dateX = (SCREEN10_WIDTH - dateWidth) / 2;
      int dateY = timeY + timeHeight + 20;
      
      // Cień daty
      tft.setTextSize(dateTextSize);
      tft.setTextColor(0x03E0);
      tft.setCursor(dateX + 1, dateY + 1);
      tft.print(dateStr);
      
      // Główna data
      tft.setTextColor(MATRIX_BRIGHTGREEN);
      tft.setCursor(dateX, dateY);
      tft.print(dateStr);
    }
    
    lastClockText = timeLocal;
  }
}

// Funkcja rysująca elegancki analogowy zegar (wygaszacz ekranu)
void drawAnalogClock() {
  unsigned long now = millis();
  
  // Aktualizuj co ~100ms (10 FPS - wystarczy dla zegara analogowego)
  if (now - lastBounceClockUpdateMs >= 100) {
    lastBounceClockUpdateMs = now;
    
    // Pobierz aktualny czas
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 1)) return;
    
    int hour = timeinfo.tm_hour;
    int minute = timeinfo.tm_min;
    int second = timeinfo.tm_sec;
    int day = timeinfo.tm_mday;
    int month = timeinfo.tm_mon + 1;
    int year = timeinfo.tm_year + 1900;
    
    // Środek zegara
    const int cx = 240;
    const int cy = 145;  // Nieco wyżej aby zrobić miejsce na datę
    const int r = 130;   // Promień tarczy
    
    // Gradientowe tło - elegancki niebieski
    for (int y = 0; y < 320; y++) {
      uint8_t intensity = 15 + (y * 25) / 320;  // Gradient od góry do dołu
      uint16_t bgColor = ((intensity & 0x1F) << 11) | ((intensity & 0x3F) << 5) | (intensity & 0x1F);
      tft.drawFastHLine(0, y, 480, bgColor);
    }
    
    // Zewnętrzna obramowanie tarczy - metaliczne złoto/srebro
    for (int i = 8; i >= 0; i--) {
      uint16_t ringColor;
      if (i == 0) ringColor = 0xFFE0;      // Złoty zewnętrzny
      else if (i < 3) ringColor = 0xC618;  // Srebrny
      else ringColor = 0x4A49;             // Ciemny metal
      tft.drawCircle(cx, cy, r + i, ringColor);
    }
    
    // Główne tło tarczy - głęboki niebieski
    tft.fillCircle(cx, cy, r, 0x0015);
    
    // Wewnętrzny pierścień dekoracyjny
    for (int i = 3; i >= 0; i--) {
      tft.drawCircle(cx, cy, r - 10 - i, 0xC618);  // Srebrny pierścień
    }
    
    // Drugie wewnętrzne tło
    tft.fillCircle(cx, cy, r - 14, 0x000C);
    
    // Cyfry rzymskie na tarczy
    const char* romanNums[] = {"", "I", "II", "III", "IV", "V", "VI", "VII", "VIII", "IX", "X", "XI", "XII"};
    for (int i = 1; i <= 12; i++) {
      float angle = (i * 30 - 90) * PI / 180;
      int numX = cx + (r - 25) * cos(angle);
      int numY = cy + (r - 25) * sin(angle);
      
      tft.setTextSize(1);
      tft.setTextColor(0xC618);
      int textW = strlen(romanNums[i]) * 6;
      tft.setCursor(numX - textW/2, numY - 4);
      tft.print(romanNums[i]);
    }
    
    // Kreski godzinowe
    for (int i = 0; i < 60; i++) {
      float angle = i * 6 * PI / 180;
      int len = (i % 5 == 0) ? 12 : 6;
      uint16_t color = (i % 5 == 0) ? TFT_WHITE : 0x8410;
      int x1 = cx + (r - 15) * cos(angle);
      int y1 = cy + (r - 15) * sin(angle);
      int x2 = cx + (r - 15 - len) * cos(angle);
      int y2 = cy + (r - 15 - len) * sin(angle);
      tft.drawLine(x1, y1, x2, y2, color);
    }
    
    // Oblicz kąty wskazówek
    float hourAngle = (timeinfo.tm_hour % 12) * 30 + timeinfo.tm_min * 0.5 - 90;
    float minAngle = timeinfo.tm_min * 6 - 90;
    float secAngle = timeinfo.tm_sec * 6 - 90;
    
    hourAngle *= PI / 180;
    minAngle *= PI / 180;
    secAngle *= PI / 180;
    
    // Wskazówka godzinowa
    int hx = cx + (r * 0.5) * cos(hourAngle);
    int hy = cy + (r * 0.5) * sin(hourAngle);
    tft.drawLine(cx, cy, hx, hy, TFT_WHITE);
    tft.drawLine(cx+1, cy, hx+1, hy, TFT_WHITE);
    tft.drawLine(cx, cy+1, hx, hy+1, TFT_WHITE);
    int mLen = r * 0.75;
    int mX = cx + mLen * cos(minAngle);
    int mY = cy + mLen * sin(minAngle);
    tft.drawLine(cx, cy, mX, mY, 0xC618);
    tft.drawLine(cx-1, cy, mX-1, mY, 0xC618);
    tft.drawLine(cx+1, cy, mX+1, mY, 0xC618);
    tft.drawLine(cx, cy-1, mX, mY-1, 0xC618);
    tft.drawLine(cx, cy+1, mX, mY+1, 0xC618);
    tft.drawLine(cx+2, cy+2, mX+2, mY+2, 0x0005);
    
    // Wskazówka sekundowa (elegancka, czerwona z kontrastem)
    int sLen = r * 0.88;
    int sX = cx + sLen * cos(secAngle);
    int sY = cy + sLen * sin(secAngle);
    tft.drawLine(cx, cy, sX, sY, TFT_RED);
    tft.drawLine(cx-1, cy, sX-1, sY, 0xF800);
    tft.drawLine(cx+1, cy, sX+1, sY, 0xF800);
    // Koniec wskazówki (mała kropka)
    tft.fillCircle(sX, sY, 4, TFT_RED);
    
    // Środek zegara - elegancki medalion
    tft.fillCircle(cx, cy, 12, 0xC618);   // Srebrna obwódka
    tft.fillCircle(cx, cy, 10, 0xFFE0);   // Złote wypełnienie
    tft.fillCircle(cx, cy, 6, 0x0015);    // Niebieskie centrum
    tft.fillCircle(cx, cy, 3, TFT_RED);   // Czerwona kropka
    
    // Napis marki na górze tarczy
    tft.setTextSize(1);
    tft.setTextColor(0xC618);
    tft.setCursor(cx - 25, cy - r + 35);
    tft.print("HAM RADIO");
    
    // Data na dole ekranu (pod zegarem)
    char dateStr[32];
    snprintf(dateStr, sizeof(dateStr), "%02d.%02d.%d", timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
    tft.setTextSize(2);
    int dateW = tft.textWidth(dateStr);
    // Cień daty
    tft.setTextColor(0x0005);
    tft.setCursor(cx - dateW/2 + 2, cy + r + 25 + 2);
    tft.print(dateStr);
    // Główna data - złota
    tft.setTextColor(0xFFE0);
    tft.setCursor(cx - dateW/2, cy + r + 25);
    tft.print(dateStr);
    
    // Dodatkowy napis "AUTOMATIC"
    tft.setTextSize(1);
    tft.setTextColor(0x8410);
    tft.setCursor(cx - 30, cy - r + 50);
    tft.print("AUTOMATIC");
  }
}

// Funkcja rysująca cyfrowy zegar odbijający się (stary bouncing clock)
void drawDigitalClock() {
  unsigned long now = millis();
  static String lastTimeStr = "";
  static bool firstDraw = true;
  
  // Aktualizuj co ~100ms
  if (now - lastBounceClockUpdateMs >= 100) {
    lastBounceClockUpdateMs = now;
    
    // Pobierz aktualny czas
    String timeStr = getTimezoneTimeString("%H:%M:%S", 9);
    
    // Rysuj tylko gdy się zmienił lub pierwszy raz
    if (firstDraw || timeStr != lastTimeStr) {
      firstDraw = false;
      lastTimeStr = timeStr;
      
      // Wyczyść ekran tylko raz
      if (firstDraw) {
        tft.fillScreen(TFT_BLACK);
      }
      
      // Oblicz pozycję - wyśrodkowany
      int textW = timeStr.length() * BOUNCE_CLOCK_CHAR_W;
      int centerX = SCREEN10_WIDTH / 2;
      int centerY = SCREEN10_HEIGHT / 2;
      int x = centerX - textW / 2;
      int y = centerY - BOUNCE_CLOCK_CHAR_H / 2;
      
      // Wyczyść obszar zegara (prostokąt w centrum)
      tft.fillRect(x - 20, y - 20, textW + 40, BOUNCE_CLOCK_CHAR_H + 40, TFT_BLACK);
      
      // Efekt neonowy - cień
      tft.setTextSize(BOUNCE_CLOCK_TEXT_SIZE);
      tft.setTextColor(0x0410); // Ciemny niebieski cień
      tft.setCursor(x + 4, y + 4);
      tft.print(timeStr);
      
      // Główny tekst - cyfrowy zielony
      tft.setTextColor(0x07E0); // Jasny zielony
      tft.setCursor(x, y);
      tft.print(timeStr);
      
      // Dodaj ramkę wokół zegara
      tft.drawRect(x - 10, y - 10, textW + 20, BOUNCE_CLOCK_CHAR_H + 20, 0x07E0);
      tft.drawRect(x - 8, y - 8, textW + 16, BOUNCE_CLOCK_CHAR_H + 16, 0x0410);
      
      // Data pod zegarem
      struct tm timeinfo;
      if (getLocalTime(&timeinfo, 1)) {
        char dateStr[20];
        strftime(dateStr, sizeof(dateStr), "%d-%m-%Y", &timeinfo);
        tft.setTextSize(2);
        tft.setTextColor(0x07E0);
        int dateW = strlen(dateStr) * 12;
        tft.setCursor(centerX - dateW/2, y + BOUNCE_CLOCK_CHAR_H + 25);
        tft.print(dateStr);
      }
    }
  }
}

// Główna funkcja wygaszacza - wybiera odpowiedni typ
void drawScreenSaver() {
  switch (screenSaverType) {
    case SAVER_ANALOG_CLOCK:
      drawAnalogClock();
      break;
    case SAVER_DIGITAL_CLOCK:
      drawDigitalClock();
      break;
    case SAVER_MATRIX:
    default:
      drawMatrixFrame();
      break;
  }
}

// Ekran 10: Zegar z animowanym tłem
void drawMatrixClock() {
  if (!matrixInitialized) {
    setupMatrix();
  }
  prepareMatrixIntro();
  drawMatrixStatic();
  screen10NeedsRedraw = false;
  lastScreen10UpdateMs = millis();
  lastMatrixUpdateMs = 0;
  clockNeedsRedraw = true;
  drawMatrixFrame();
}

void updateScreen10() {
  if (!tftInitialized || currentScreen != SCREEN_MATRIX_CLOCK || inMenu) {
    return;
  }

  unsigned long now = millis();
  if (now - lastScreen10UpdateMs < MATRIX_UPDATE_INTERVAL_MS) {
    return;
  }
  lastScreen10UpdateMs = now;

  if (screen10NeedsRedraw) {
    drawMatrixStatic();
    screen10NeedsRedraw = false;
    clockNeedsRedraw = true;
  }
  drawMatrixFrame();
}

// ========== Ekran 11: UnlisHunter ==========

bool unlisRunning = false;
bool unlisGameOver = false;
bool unlisIntroVisible = true;
bool unlisIntroNeedsRedraw = true;
bool unlisGameOverNeedsRedraw = false;
bool unlisRunningUiNeedsRedraw = false;
int unlisCaught = 0;
int unlisMissed = 0;
unsigned long unlisGameStartMs = 0;
unsigned long unlisLastFrameMs = 0;
unsigned long unlisElapsedSecFrozen = 0;
float unlisScanAngleDeg = 0.0f;
float unlisPrevScanAngleDeg = 0.0f;
bool unlisArrowPrevValid = false;

bool unlisTargetActive = false;
bool unlisTargetOuter = true;
float unlisTargetAngleDeg = 0.0f;
unsigned long unlisTargetShownMs = 0;
unsigned long unlisNextSpawnMs = 0;

bool unlisSecondTargetActive = false;
bool unlisSecondTargetOuter = true;
float unlisSecondTargetAngleDeg = 0.0f;
unsigned long unlisSecondTargetShownMs = 0;
unsigned long unlisSecondTargetNextSpawnMs = 0;

bool unlisGreenStationActive = false;
bool unlisGreenStationOuter = true;
float unlisGreenStationAngleDeg = 0.0f;
unsigned long unlisGreenStationShownMs = 0;
unsigned long unlisGreenStationNextSpawnMs = 0;

bool unlisOuterEdgeNeedsClean = false;
unsigned long unlisLastPttPressMs = 0;
bool unlisUiNeedsRefreshOnTargetChange = true;
unsigned long unlisLastTimerSecDrawn = 0;
bool unlisTimerDrawnValid = false;

static inline float normalizeDeg360(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a < 0.0f) a += 360.0f;
  return a;
}

static float unlisCurrentScanDegPerSec() {
  // Up to 60s: +5% per catch, after 60s: +3% per catch.
  unsigned long elapsedMs = 0;
  if (unlisRunning || unlisGameOver) {
    elapsedMs = millis() - unlisGameStartMs;
  }
  float accelPerCatch = (elapsedMs >= 60000UL) ? UNLIS_ACCEL_PER_CATCH_LATE : UNLIS_ACCEL_PER_CATCH_EARLY;
  return UNLIS_BASE_SCAN_DEG_PER_SEC * (1.0f + ((float)unlisCaught * accelPerCatch));
}

static unsigned long unlisCurrentTargetLifeMs() {
  float degPerSec = unlisCurrentScanDegPerSec();
  float fullRotationSec = 360.0f / degPerSec;
  return (unsigned long)(fullRotationSec * UNLIS_TARGET_LIFE_ROTATIONS * 1000.0f + 0.5f);
}

static unsigned long unlisCurrentPttCooldownMs() {
  float degPerSec = unlisCurrentScanDegPerSec();
  float fullRotationSec = 360.0f / degPerSec;
  return (unsigned long)(fullRotationSec * 0.25f * 1000.0f + 0.5f);
}

static void unlisResetPttCooldown(unsigned long nowMs) {
  unlisLastPttPressMs = nowMs - unlisCurrentPttCooldownMs();
}

static bool unlisPointInButton(int x, int y) {
  if (x >= UNLIS_START_X && x < (UNLIS_START_X + UNLIS_BTN_SIZE) &&
      y >= UNLIS_START_Y && y < (UNLIS_START_Y + UNLIS_BTN_SIZE)) {
    return true;
  }
  if (x >= UNLIS_PTT_X && x < (UNLIS_PTT_X + UNLIS_BTN_SIZE) &&
      y >= UNLIS_PTT_Y && y < (UNLIS_PTT_Y + UNLIS_BTN_SIZE)) {
    return true;
  }
  if (x >= UNLIS_EXIT_X && x < (UNLIS_EXIT_X + UNLIS_BTN_SIZE) &&
      y >= UNLIS_EXIT_Y && y < (UNLIS_EXIT_Y + UNLIS_BTN_SIZE)) {
    return true;
  }
  return false;
}

static void unlisGenerateRandomTarget(bool &targetOuter, float &targetAngleDeg) {
  bool foundVisibleSpot = false;
  bool outer = true;
  float angle = 0.0f;

  for (int i = 0; i < 24; i++) {
    bool candidateOuter = (random(0, 2) == 0);
    float candidateAngle = (float)random(0, 360);
    float a = candidateAngle * (float)M_PI / 180.0f;
    int r = candidateOuter ? UNLIS_OUTER_R : UNLIS_INNER_R;
    int tx = UNLIS_CENTER_X + (int)(sinf(a) * (float)r);
    int ty = UNLIS_CENTER_Y - (int)(cosf(a) * (float)r);
    if (!unlisPointInButton(tx, ty)) {
      outer = candidateOuter;
      angle = candidateAngle;
      foundVisibleSpot = true;
      break;
    }
  }

  if (!foundVisibleSpot) {
    outer = (random(0, 2) == 0);
    angle = (float)random(0, 360);
  }

  targetOuter = outer;
  targetAngleDeg = angle;
}

static bool unlisArrowHitsTarget(bool targetOuter, float targetAngleDeg, int hitPx) {
  float arrowDeg = targetOuter ? unlisScanAngleDeg : normalizeDeg360(unlisScanAngleDeg + 180.0f);
  int ringR = targetOuter ? UNLIS_OUTER_R : UNLIS_INNER_R;
  float arrowA = arrowDeg * (float)M_PI / 180.0f;
  float targetA = targetAngleDeg * (float)M_PI / 180.0f;
  int arrowX = UNLIS_CENTER_X + (int)(sinf(arrowA) * (float)ringR);
  int arrowY = UNLIS_CENTER_Y - (int)(cosf(arrowA) * (float)ringR);
  int targetX = UNLIS_CENTER_X + (int)(sinf(targetA) * (float)ringR);
  int targetY = UNLIS_CENTER_Y - (int)(cosf(targetA) * (float)ringR);
  int dx = arrowX - targetX;
  int dy = arrowY - targetY;
  return (dx * dx + dy * dy) <= (hitPx * hitPx);
}

static void drawUnlisButton(int x, int y, const char* label) {
  const int drawW = UNLIS_DRAW_BTN_SIZE;
  const int drawH = UNLIS_DRAW_BTN_SIZE;
  int drawX = x + 10;
  int drawY = y + 10;

  // Keep visual buttons closer to screen corners (~10 px margin).
  if (x == UNLIS_PTT_X) {
    drawX = x + (UNLIS_BTN_SIZE - drawW - 10);
  }
  if (y == UNLIS_START_Y) {
    drawY = y;
  }
  if (y == UNLIS_PTT_Y || y == UNLIS_EXIT_Y) {
    drawY = y + (UNLIS_BTN_SIZE - drawH);
  }
  const int r = 10;

  // Czarny przycisk z lekkim efektem wypuklosci.
  tft.fillRoundRect(drawX, drawY, drawW, drawH, r, TFT_BLACK);
  tft.drawRoundRect(drawX, drawY, drawW, drawH, r, TFT_LIGHTGREY);
  tft.drawFastHLine(drawX + 8, drawY + 4, drawW - 16, TFT_WHITE);
  tft.drawFastVLine(drawX + 4, drawY + 8, drawH - 16, TFT_WHITE);
  tft.drawFastHLine(drawX + 8, drawY + drawH - 5, drawW - 16, TFT_DARKGREY);
  tft.drawFastVLine(drawX + drawW - 5, drawY + 8, drawH - 16, TFT_DARKGREY);

  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  int labelW = (int)strlen(label) * 12;
  tft.setCursor(drawX + (drawW - labelW) / 2, drawY + 18);
  tft.print(label);
}

static void drawUnlisButtons() {
  const char* startLabel = unlisRunning ? "STOP" : "START";
  drawUnlisButton(UNLIS_START_X, UNLIS_START_Y, startLabel);
  drawUnlisButton(UNLIS_PTT_X, UNLIS_PTT_Y, "PTT");
  drawUnlisButton(UNLIS_EXIT_X, UNLIS_EXIT_Y, "EXIT");
}

static void drawUnlisScoreAndMissMarkers() {
  tft.fillRect(2, 52, 84, 96, TFT_BLACK);

  String scoreText = String(unlisCaught);
  while (scoreText.length() < 3) {
    scoreText = "0" + scoreText;
  }

  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(4, 56);
  tft.print(scoreText);

  const int markerX = 12;
  const int markerMidY = 120;
  const int markerStepY = 18;
  const int markerR = 5;
  for (int i = 0; i < 3; i++) {
    int markerY = markerMidY + ((i - 1) * markerStepY);
    bool missed = (unlisMissed > i);
    if (missed) {
      tft.fillCircle(markerX, markerY, markerR, TFT_RED);
      tft.drawCircle(markerX, markerY, markerR, TFT_RED);
    } else {
      tft.fillCircle(markerX, markerY, markerR, TFT_BLACK);
      tft.drawCircle(markerX, markerY, markerR, TFT_DARKGREY);
    }
  }
}

static void drawUnlisBandLegend() {
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(286, 96);
  tft.print("BAND:");
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(292, 108);
  tft.print("2m");
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.setCursor(286, 120);
  tft.print("70cm");
}

static void drawUnlisTimerValue(unsigned long elapsedSec) {
  String timerText = String(elapsedSec) + "s";
  int tw = timerText.length() * 12;
  tft.fillRect(236, 2, 82, 20, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(316 - tw, 6);
  tft.print(timerText);
}

static void drawUnlisHud() {
  drawUnlisButtons();
  
  unsigned long elapsedSec = 0;
  if (unlisRunning) {
    elapsedSec = (millis() - unlisGameStartMs) / 1000UL;
  } else if (unlisGameOver) {
    elapsedSec = unlisElapsedSecFrozen;
  }

  drawUnlisScoreAndMissMarkers();
  drawUnlisBandLegend();
  drawUnlisTimerValue(elapsedSec);
}

static void drawUnlisStatsOnly() {

  unsigned long elapsedSec = 0;
  if (unlisRunning) {
    elapsedSec = (millis() - unlisGameStartMs) / 1000UL;
  } else if (unlisGameOver) {
    elapsedSec = unlisElapsedSecFrozen;
  }

  drawUnlisScoreAndMissMarkers();
  drawUnlisTimerValue(elapsedSec);
}

static void drawUnlisIntroText() {
  tft.fillRect(40, 72, 320, 92, TFT_BLACK);
  tft.drawRect(40, 72, 320, 92, TFT_DARKGREY);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(52, 82);
  tft.print("UNLIS HUNTER GAME");

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(52, 114);
  tft.print("Use a Yagi antenna to catch and");
  tft.setCursor(52, 130);
  tft.print("jam its signal by pressing PTT on it.");
  tft.setCursor(52, 146);
  tft.print("* Unlis = Unlicensed");
}

static void drawUnlisRadarBase() {
  tft.drawCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, UNLIS_INNER_R, TFT_BLUE);
  tft.fillCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, 3, TFT_GREEN);
}

static void drawUnlisOuterRing() {
  tft.drawCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, UNLIS_OUTER_R, TFT_YELLOW);
  // Black outer outline masks red artifacts outside the yellow ring.
  tft.drawCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, UNLIS_OUTER_R + 1, TFT_BLACK);
  tft.drawCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, UNLIS_OUTER_R + 2, TFT_BLACK);
}

static void drawUnlisTarget() {
  if (unlisTargetActive) {
    float a = unlisTargetAngleDeg * (float)M_PI / 180.0f;
    int r = unlisTargetOuter ? UNLIS_OUTER_R : UNLIS_INNER_R;
    int tx = UNLIS_CENTER_X + (int)(sinf(a) * (float)r);
    int ty = UNLIS_CENTER_Y - (int)(cosf(a) * (float)r);
    int targetRadius = unlisTargetOuter ? 6 : 4;
    tft.fillCircle(tx, ty, targetRadius, TFT_RED);
  }

  if (unlisSecondTargetActive) {
    float a2 = unlisSecondTargetAngleDeg * (float)M_PI / 180.0f;
    int r2 = unlisSecondTargetOuter ? UNLIS_OUTER_R : UNLIS_INNER_R;
    int tx2 = UNLIS_CENTER_X + (int)(sinf(a2) * (float)r2);
    int ty2 = UNLIS_CENTER_Y - (int)(cosf(a2) * (float)r2);
    int targetRadius2 = unlisSecondTargetOuter ? 6 : 4;
    tft.fillCircle(tx2, ty2, targetRadius2, TFT_RED);
  }

  if (unlisGreenStationActive) {
    float ag = unlisGreenStationAngleDeg * (float)M_PI / 180.0f;
    int rg = unlisGreenStationOuter ? UNLIS_OUTER_R : UNLIS_INNER_R;
    int txg = UNLIS_CENTER_X + (int)(sinf(ag) * (float)rg);
    int tyg = UNLIS_CENTER_Y - (int)(cosf(ag) * (float)rg);
    tft.fillCircle(txg, tyg, 4, TFT_GREEN);
  }
}

static void drawUnlisArrows(float angleDeg) {
  float a = angleDeg * (float)M_PI / 180.0f;
  int yx = UNLIS_CENTER_X + (int)(sinf(a) * (float)UNLIS_OUTER_R);
  int yy = UNLIS_CENTER_Y - (int)(cosf(a) * (float)UNLIS_OUTER_R);
  int bx = UNLIS_CENTER_X - (int)(sinf(a) * (float)UNLIS_INNER_R);
  int by = UNLIS_CENTER_Y + (int)(cosf(a) * (float)UNLIS_INNER_R);

  tft.drawLine(UNLIS_CENTER_X, UNLIS_CENTER_Y, yx, yy, TFT_YELLOW);
  tft.drawLine(UNLIS_CENTER_X, UNLIS_CENTER_Y, bx, by, TFT_BLUE);
}

static void drawUnlisGameOver() {
  tft.fillRect(60, 96, 200, 48, TFT_BLACK);
  tft.drawRect(60, 96, 200, 48, TFT_RED);
  tft.setTextColor(TFT_RED);
  tft.setTextSize(2);
  const char* txt = "BAJO JAJO";
  int tw = (int)strlen(txt) * 12;
  tft.setCursor(60 + (200 - tw) / 2, 112);
  tft.print(txt);
}

void unlisStopGame() {
  unlisRunning = false;
  unlisGameOver = false;
  unlisIntroVisible = true;
  unlisIntroNeedsRedraw = true;
  unlisGameOverNeedsRedraw = false;
  unlisRunningUiNeedsRedraw = false;
  unlisTargetActive = false;
  unlisSecondTargetActive = false;
  unlisGreenStationActive = false;
  unlisArrowPrevValid = false;
  unlisUiNeedsRefreshOnTargetChange = true;
  unlisTimerDrawnValid = false;
}

void unlisStartResetGame() {
  unlisRunning = true;
  unlisGameOver = false;
  unlisIntroVisible = false;
  unlisIntroNeedsRedraw = false;
  unlisGameOverNeedsRedraw = false;
  unlisRunningUiNeedsRedraw = true;
  unlisCaught = 0;
  unlisMissed = 0;
  unlisElapsedSecFrozen = 0;
  unlisGameStartMs = millis();
  unlisLastFrameMs = millis();
  unlisScanAngleDeg = 0.0f;
  unlisPrevScanAngleDeg = 0.0f;
  unlisArrowPrevValid = false;
  unlisTargetActive = false;
  unlisSecondTargetActive = false;
  unlisGreenStationActive = false;
  unlisOuterEdgeNeedsClean = true;
  unlisResetPttCooldown(millis());
  unlisUiNeedsRefreshOnTargetChange = true;
  unlisTimerDrawnValid = false;
  unlisNextSpawnMs = millis() + random(200UL, 700UL);
  unlisSecondTargetNextSpawnMs = millis() + UNLIS_SECOND_TARGET_DELAY_MS + random(500UL, 3000UL);
  unlisGreenStationNextSpawnMs = millis() + random(UNLIS_GREEN_STATION_RESPAWN_MIN_MS, UNLIS_GREEN_STATION_RESPAWN_MAX_MS + 1UL);
}

static void unlisSpawnTarget(unsigned long nowMs) {
  bool targetOuter = true;
  float targetAngle = 0.0f;
  unlisGenerateRandomTarget(targetOuter, targetAngle);

  unlisTargetActive = true;
  unlisTargetOuter = targetOuter;
  unlisTargetAngleDeg = targetAngle;
  unlisTargetShownMs = nowMs;
  unlisOuterEdgeNeedsClean = true;
  unlisResetPttCooldown(nowMs);
  unlisUiNeedsRefreshOnTargetChange = true;
}

void unlisHandlePttPress(unsigned long nowMs) {
  if (!unlisRunning || unlisGameOver) return;

  unsigned long pttCooldownMs = unlisCurrentPttCooldownMs();
  if ((nowMs - unlisLastPttPressMs) < pttCooldownMs) return;
  unlisLastPttPressMs = nowMs;

  const int hitPx = 14;

  if (unlisGreenStationActive && unlisArrowHitsTarget(unlisGreenStationOuter, unlisGreenStationAngleDeg, hitPx)) {
    if (unlisCaught > 0) {
      unlisCaught--;
    }
    unlisGreenStationActive = false;
    unlisGreenStationNextSpawnMs = nowMs + random(UNLIS_GREEN_STATION_RESPAWN_MIN_MS, UNLIS_GREEN_STATION_RESPAWN_MAX_MS + 1UL);
    unlisUiNeedsRefreshOnTargetChange = true;
    return;
  }

  bool hitAnyUnlis = false;
  if (unlisTargetActive && unlisArrowHitsTarget(unlisTargetOuter, unlisTargetAngleDeg, hitPx)) {
    unlisCaught++;
    unlisTargetActive = false;
    hitAnyUnlis = true;
  }
  if (unlisSecondTargetActive && unlisArrowHitsTarget(unlisSecondTargetOuter, unlisSecondTargetAngleDeg, hitPx)) {
    unlisCaught++;
    unlisSecondTargetActive = false;
      unlisSecondTargetNextSpawnMs = nowMs + random(UNLIS_SECOND_TARGET_RESPAWN_MIN_MS, UNLIS_SECOND_TARGET_RESPAWN_MAX_MS + 1UL);
    hitAnyUnlis = true;
  }

  if (hitAnyUnlis) {
    unlisResetPttCooldown(nowMs);
    if (!unlisTargetActive && !unlisSecondTargetActive) {
      unlisNextSpawnMs = nowMs + random(UNLIS_TARGET_RESPAWN_MIN_MS, UNLIS_TARGET_RESPAWN_MAX_MS + 1UL);
    }
    unlisUiNeedsRefreshOnTargetChange = true;
  }
}

void drawUnlisHunter() {
  unlisIntroVisible = true;
  unlisIntroNeedsRedraw = true;
  unlisGameOverNeedsRedraw = false;
  unlisRunningUiNeedsRedraw = false;

  unlisRunning = false;
  unlisGameOver = false;
  unlisTargetActive = false;
  unlisSecondTargetActive = false;
  unlisGreenStationActive = false;
  unlisOuterEdgeNeedsClean = false;
  unlisArrowPrevValid = false;
  unlisElapsedSecFrozen = 0;
  unlisUiNeedsRefreshOnTargetChange = true;
  unlisTimerDrawnValid = false;
}

// Ekran śledzenia przelotu ISS
Sgp4 issSat;
char issTleLine1[] = "1 25544U 98067A   26159.21147137  .00014761  00000-0  26477-3 0  9997";
char issTleLine2[] = "2 25544  51.6415 171.1234 0001234  75.1234 320.4321 15.49234123543217";

double issAzimuth = 0.0, issElevation = 0.0, issDistance = 0.0, issAltitude = 0.0;
double issLat = 0.0, issLng = 0.0;
double issSpeed = 0.0;
String issCountry = "CZEKAJ...";
unsigned long lastIssCountryCheck = 0;
unsigned long lastIssDataFetch = 0;

const int issCx = 255;
const int issCy = 145;
const int issRMax = 75;

int lastIssSatX = -1;
int lastIssSatY = -1;

// Global trajectory array to persist across screen switches
#define ISS_TRAJECTORY_POINTS 50
int issTrajectoryX[ISS_TRAJECTORY_POINTS] = {0};
int issTrajectoryY[ISS_TRAJECTORY_POINTS] = {0};
int issTrajectoryIndex = 0;

void drawIssIconProcedural(int16_t x, int16_t y, uint32_t color) {
  // x, y to środek pozycji stacji na mapie/radarze

  // 1. Panele słoneczne (lewy i prawy) - dwa pionowe prostokąty
  tft.fillRect(x - 12, y - 8, 4, 16, TFT_CYAN);
  tft.fillRect(x + 8, y - 8, 4, 16, TFT_CYAN);

  // 2. Kratownica łącząca panele - pozioma linia
  tft.drawFastHLine(x - 8, y, 16, TFT_SILVER);

  // 3. Moduł centralny (kadłub stacji) - mały poziomy prostokąt lub elipsa w środku
  tft.fillRect(x - 4, y - 3, 8, 6, color);
  tft.drawPixel(x, y, TFT_RED); // Czerwony punkt środkowy
}

void initIssTracking() {
  issSat.init(issTleLine1, issTleLine2);
}

void calculateIssObservationData() {
  // Calculate distance, azimuth, and elevation from ISS to user location
  double userLat = userLatLonValid ? userLat : 52.40;
  double userLon = userLatLonValid ? userLon : 16.92;

  // Convert to radians
  double home_lat = userLat * DEG_TO_RAD;
  double iss_lat = issLat * DEG_TO_RAD;
  double home_lon = userLon * DEG_TO_RAD;
  double iss_lon = issLng * DEG_TO_RAD;

  // Earth radius in km
  double R = 6371.0;
  double r_iss = R + issAltitude; // Distance from Earth center to ISS

  // Haversine formula for distance
  double dLat = iss_lat - home_lat;
  double dLon = iss_lon - home_lon;
  double a = sin(dLat/2) * sin(dLat/2) + cos(home_lat) * cos(iss_lat) * sin(dLon/2) * sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  issDistance = R * c;

  // Calculate azimuth
  double y = sin(dLon) * cos(iss_lat);
  double x = cos(home_lat) * sin(iss_lat) - sin(home_lat) * cos(iss_lat) * cos(dLon);
  issAzimuth = atan2(y, x) * RAD_TO_DEG;
  if (issAzimuth < 0) issAzimuth += 360;

  // Calculate elevation using proper radio-astronomical algorithm
  // 1. Calculate central angle (g) between Home and ISS using Haversine
  double cos_g = sin(home_lat) * sin(iss_lat) + cos(home_lat) * cos(iss_lat) * cos(dLon);

  // Protect against floating point errors
  if (cos_g > 1.0) cos_g = 1.0;
  if (cos_g < -1.0) cos_g = -1.0;

  double g = acos(cos_g); // Central angle in radians

  // 2. Calculate slant range (straight-line distance through Earth)
  double slant_range = sqrt(R*R + r_iss*r_iss - 2*R*r_iss * cos_g);

  // 3. Calculate elevation angle using simplified stable formula
  double num = r_iss * cos_g - R;
  double den = sqrt(R*R + r_iss*r_iss - 2*R*r_iss*cos_g);
  double el_rad = atan2(num, sqrt(r_iss*r_iss * (1 - cos_g*cos_g)));
  issElevation = el_rad * RAD_TO_DEG;
}

void calculateDistanceAndAzimuth(double homeLat, double homeLon, double targetLat, double targetLon, double &dist, double &az) {
  // Convert to radians
  double home_lat = homeLat * DEG_TO_RAD;
  double target_lat = targetLat * DEG_TO_RAD;
  double home_lon = homeLon * DEG_TO_RAD;
  double target_lon = targetLon * DEG_TO_RAD;

  // Earth radius in km
  double R = 6371.0;

  // Haversine formula for distance
  double dLat = target_lat - home_lat;
  double dLon = target_lon - home_lon;
  double a = sin(dLat/2) * sin(dLat/2) + cos(home_lat) * cos(target_lat) * sin(dLon/2) * sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  dist = R * c;

  // Calculate azimuth
  double y = sin(dLon) * cos(target_lat);
  double x = cos(home_lat) * sin(target_lat) - sin(home_lat) * cos(target_lat) * cos(dLon);
  az = atan2(y, x) * RAD_TO_DEG;
  if (az < 0) az += 360;
}

void drawIssTrajectory() {
  double userLat = userLatLonValid ? userLat : 52.40;
  double userLon = userLatLonValid ? userLon : 16.92;

  // Extrapolate ISS positions for -5 to +10 minutes in 1-minute intervals
  // ISS orbital velocity ~27,600 km/h = 7.67 km/s
  // Earth circumference at ISS altitude ~42,000 km
  // ISS completes orbit in ~90 minutes = 1.5 degrees per minute
  const int numPoints = 16; // -5 to +10 minutes
  double trajectoryLat[numPoints];
  double trajectoryLon[numPoints];

  for (int i = 0; i < numPoints; i++) {
    int minutesOffset = i - 5; // -5 to +10 minutes
    double angleOffset = minutesOffset * 1.5; // degrees (1.5 deg/min)

    // Simple extrapolation: move along orbital path
    double angleRad = angleOffset * DEG_TO_RAD;
    trajectoryLat[i] = issLat + (sin(angleRad) * 5.0); // Approximate latitude change
    trajectoryLon[i] = issLng + (cos(angleRad) * 5.0); // Approximate longitude change
  }

  // Draw trajectory line
  int16_t lastX = -1, lastY = -1;

  for (int i = 0; i < numPoints; i++) {
    double dist, az;
    calculateDistanceAndAzimuth(userLat, userLon, trajectoryLat[i], trajectoryLon[i], dist, az);

    // Check if point is within 5000 km radar range
    if (dist <= 5000.0) {
      double angleRad = (az - 90.0) * DEG_TO_RAD;
      double pixelDist = dist * (issRMax / 5000.0);

      int16_t currentX = issCx + (cos(angleRad) * pixelDist);
      int16_t currentY = issCy + (sin(angleRad) * pixelDist);

      // Draw line segment if we have a previous point
      if (lastX != -1 && lastY != -1) {
        tft.drawLine(lastX, lastY, currentX, currentY, TFT_DARKGREY);
      }

      lastX = currentX;
      lastY = currentY;
    } else {
      // Reset if point is outside radar range
      lastX = -1;
      lastY = -1;
    }
  }
}

void fetchIssData() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin("https://api.wheretheiss.at/v1/satellites/25544");
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();
    DynamicJsonDocument doc(1024);
    DeserializationError err = deserializeJson(doc, payload);

    if (!err) {
      issLat = doc["latitude"];
      issLng = doc["longitude"];
      issAltitude = doc["altitude"];
      issSpeed = doc["velocity"];
      issCountry = "CZEKAJ..."; // Will be updated by getIssCountryFromCoords

      // Calculate observation data after fetching position
      calculateIssObservationData();
    }
  }
  http.end();
}

void calculateIssPosition(time_t t) {
  int yr = year(t); int mth = month(t); int dy = day(t);
  int hr = hour(t); int mn = minute(t); int sc = second(t);

  issSat.findsat(yr, mth, dy, hr, mn, sc);
  
  double myAlt = 220.0;
  if (userLatLonValid) {
    issSat.get_observe_navigator(userLat, userLon, myAlt / 1000.0, issAzimuth, issElevation, issDistance, issAltitude);
  } else {
    issSat.get_observe_navigator(52.40, 16.92, myAlt / 1000.0, issAzimuth, issElevation, issDistance, issAltitude);
  }

  long double issLat_ld, issLng_ld;
  issSat.get_latlon(issLat_ld, issLng_ld);
  issLat = (double)issLat_ld;
  issLng = (double)issLng_ld;
}

void getIssCountryFromCoords(double lat, double lng) {
  if (WiFi.status() != WL_CONNECTED) return;
  
  HTTPClient http;
  String url = "http://api.bigdatacloud.net/data/reverse-geocode-client?latitude=" + String(lat, 4) + "&longitude=" + String(lng, 4) + "&localityLanguage=pl";
  
  http.begin(url);
  int httpCode = http.GET();
  
  if (httpCode == 200) {
    String payload = http.getString();
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);
    String country = doc["countryName"];
    
    if (country.length() > 0) {
      issCountry = country;
      issCountry.toUpperCase();
    } else {
      issCountry = "OCEAN / MORZE";
    }
  }
  http.end();
}

void drawIssStaticInterface() {
  // Global UI reset - clean black background
  tft.fillScreen(TFT_BLACK);

  // LEFT COLUMN - Top: Country Tracking Box
  tft.drawRoundRect(10, 60, 160, 35, 4, TFT_GREEN);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("KRAJ POD SATELITĄ:", 15, 65, 1);

  // LEFT COLUMN - Middle: Latitude Box
  tft.drawRoundRect(10, 105, 140, 35, 4, TFT_GREEN);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("LATITUDE:", 15, 110, 1);

  // LEFT COLUMN - Bottom: Altitude Box
  tft.drawRoundRect(10, 150, 140, 35, 4, TFT_GREEN);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("ALTITUDE:", 15, 155, 1);

  // BOTTOM ROW - Speed Box
  tft.drawRoundRect(10, 265, 110, 35, 4, TFT_CYAN);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("SPEED:", 15, 270, 1);

  // BOTTOM ROW - Azimuth Box
  tft.drawRoundRect(125, 265, 110, 35, 4, 0x4D3F);
  tft.setTextColor(0x4D3F, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("AZIMUTH:", 130, 270, 1);

  // BOTTOM ROW - Distance Box
  tft.drawRoundRect(240, 265, 110, 35, 4, TFT_ORANGE);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("DISTANCE:", 245, 270, 1);

  // BOTTOM ROW - Elevation Box
  tft.drawRoundRect(355, 265, 115, 35, 4, 0x901F);
  tft.setTextColor(0x901F, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("ELEVATION:", 360, 270, 1);

  // BOTTOM-RIGHT - AOS IN Box
  tft.drawRoundRect(355, 220, 115, 35, 4, TFT_GREEN);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("AOS IN:", 360, 225, 1);

  // CENTRAL POLAR RADAR - Background Grid
  // Concentric circles with labels (Scale: 5000 km max)
  tft.drawCircle(issCx, issCy, issRMax, TFT_WHITE);           // 5000 km
  tft.drawCircle(issCx, issCy, issRMax * 3 / 4, 0x7BEF);      // 3750 km
  tft.drawCircle(issCx, issCy, issRMax / 2, 0x39C7);          // 2500 km
  tft.drawCircle(issCx, issCy, issRMax / 4, 0x39C7);          // 1250 km

  // Cross lines
  tft.drawLine(issCx, issCy - issRMax, issCx, issCy + issRMax, 0x39C7);
  tft.drawLine(issCx - issRMax, issCy, issCx + issRMax, issCy, 0x39C7);

  // Cardinal direction labels
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("N", issCx, issCy - issRMax - 12, 2);
  tft.drawString("S", issCx, issCy + issRMax + 12, 2);
  tft.drawString("W", issCx - issRMax - 15, issCy, 2);
  tft.drawString("E", issCx + issRMax + 15, issCy, 2);

  // Distance labels
  tft.setTextColor(0x7BEF, TFT_BLACK);
  tft.drawString("1250km", issCx, issCy - issRMax / 4, 1);
  tft.drawString("2500km", issCx, issCy - issRMax / 2, 1);
  tft.drawString("3750km", issCx, issCy - issRMax * 3 / 4, 1);
  tft.drawString("5000km", issCx, issCy - issRMax, 1);
}

void updateIssDynamicDisplay() {
  if (!tftInitialized || currentScreen != SCREEN_ISS_PASS_TRACKING || inMenu) {
    return;
  }

  // Force 5-second refresh interval
  if (millis() - lastIssDataFetch > 5000) {
    fetchIssData();
    lastIssDataFetch = millis();
  }

  if (millis() - lastIssCountryCheck > 15000) {
    getIssCountryFromCoords(issLat, issLng);
    lastIssCountryCheck = millis();
  }

  tft.setTextDatum(MC_DATUM);
  char buffer[25];

  // Clear and update Country Tracking Box (bright yellow for ocean/country)
  tft.fillRect(12, 78, 156, 23, TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString(issCountry, 90, 90, 2);

  // Clear and update Latitude Box
  tft.fillRect(12, 123, 136, 23, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  sprintf(buffer, "%0.1f°", issLat); tft.drawString(buffer, 80, 135, 2);

  // Clear and update Altitude Box
  tft.fillRect(12, 168, 136, 23, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  sprintf(buffer, "%0.1f km", issAltitude); tft.drawString(buffer, 80, 180, 2);

  // Clear and update Speed Box
  tft.fillRect(12, 283, 106, 23, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  sprintf(buffer, "%0.0f km/h", issSpeed); tft.drawString(buffer, 65, 295, 2);

  // Clear and update Azimuth Box
  tft.fillRect(127, 283, 106, 23, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  sprintf(buffer, "%0.1f° S", issAzimuth); tft.drawString(buffer, 180, 295, 2);

  // Clear and update Distance Box
  tft.fillRect(242, 283, 106, 23, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  sprintf(buffer, "%0.0f km", issDistance); tft.drawString(buffer, 295, 295, 2);

  // Clear and update Elevation Box
  tft.fillRect(357, 283, 111, 23, TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  sprintf(buffer, "%0.1f°", issElevation); tft.drawString(buffer, 412, 295, 2);

  // Clear and update AOS IN Box with countdown
  tft.fillRect(357, 238, 111, 23, TFT_BLACK);
  if (issElevation > 0) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK); tft.drawString("WIDOCZNY", 412, 250, 2);
    // Draw small ESP32 text/logo and WiFi signal icon in corner
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("ESP32", 365, 242, 1);
    tft.fillRect(440, 242, 3, 3, TFT_GREEN);
    tft.fillRect(444, 240, 3, 5, TFT_GREEN);
    tft.fillRect(448, 238, 3, 7, TFT_GREEN);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK); tft.drawString("NIE WIDOCZNY", 412, 250, 2);
  }

  // Clear old ISS icon position
  if (lastIssSatX != -1 && lastIssSatY != -1) {
    tft.fillRect(lastIssSatX - 15, lastIssSatY - 15, 30, 30, TFT_BLACK);
  }

  // Draw orbital trajectory line
  drawIssTrajectory();

  // Draw ISS icon and tracking path
  if (issDistance > 0 && issDistance <= 5000.0) {
    // Map distance to pixels using 5000 km scale
    double pixelDist = issDistance * (issRMax / 5000.0);
    double angleRad = (issAzimuth - 90.0) * DEG_TO_RAD;

    int satX = issCx + (cos(angleRad) * pixelDist);
    int satY = issCy + (sin(angleRad) * pixelDist);

    // Store position in trajectory array
    issTrajectoryX[issTrajectoryIndex] = satX;
    issTrajectoryY[issTrajectoryIndex] = satY;
    issTrajectoryIndex = (issTrajectoryIndex + 1) % ISS_TRAJECTORY_POINTS;

    // Draw trajectory line from stored points
    for (int i = 0; i < ISS_TRAJECTORY_POINTS; i++) {
      if (issTrajectoryX[i] != 0 && issTrajectoryY[i] != 0) {
        int nextIdx = (i + 1) % ISS_TRAJECTORY_POINTS;
        if (issTrajectoryX[nextIdx] != 0 && issTrajectoryY[nextIdx] != 0) {
          tft.drawLine(issTrajectoryX[i], issTrajectoryY[i], issTrajectoryX[nextIdx], issTrajectoryY[nextIdx], TFT_YELLOW);
        }
      }
    }

    // Draw procedural ISS icon
    drawIssIconProcedural(satX, satY, TFT_YELLOW);

    // Draw tracking path markers
    for (int i = 1; i <= 3; i++) {
      double markerR = pixelDist * i / 4;
      int markerX = issCx + (cos(angleRad) * markerR);
      int markerY = issCy + (sin(angleRad) * markerR);
      tft.fillCircle(markerX, markerY, 2, TFT_YELLOW);
    }

    lastIssSatX = satX; lastIssSatY = satY;
  }

  // Draw header on top (ensure it's not overwritten)
  tft.setTextColor(TFT_GOLD, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString("ISS Pass Tracker", 240, 20, 2);
  tft.setTextSize(1); // Reset text size
  tft.setTextDatum(TL_DATUM); // Reset to default

  // Draw QTH star on top (ensure it's visible)
  tft.fillCircle(issCx, issCy, 4, TFT_RED);
}

void drawIssPassTracking() {
  if (!tftInitialized) return;

  // Reset font scaling and text parameters to prevent "too large" issue
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, 0);

  drawIssStaticInterface();
  
  if (millis() - lastIssDataFetch > 10000) {
    fetchIssData();
    lastIssDataFetch = millis();
  }

  if (millis() - lastIssCountryCheck > 15000) {
    getIssCountryFromCoords(issLat, issLng);
    lastIssCountryCheck = millis();
  }

  updateIssDynamicDisplay();
}

void updateUnlisHunter() {
  if (!tftInitialized || currentScreen != SCREEN_UNLIS_HUNTER || inMenu) {
    return;
  }

  if (!unlisRunning && !unlisGameOver) {
    if (unlisIntroVisible && unlisIntroNeedsRedraw) {
      tft.fillScreen(TFT_BLACK);
      drawUnlisHud();
      drawUnlisIntroText();
      unlisIntroNeedsRedraw = false;
    }
    return;
  }

  if (unlisGameOver) {
    if (unlisGameOverNeedsRedraw) {
      tft.fillScreen(TFT_BLACK);
      drawUnlisHud();
      drawUnlisOuterRing();
      drawUnlisRadarBase();
      drawUnlisArrows(unlisScanAngleDeg);
      drawUnlisGameOver();
      unlisGameOverNeedsRedraw = false;
    }
    return;
  }

  if (unlisRunningUiNeedsRedraw) {
    tft.fillScreen(TFT_BLACK);
    drawUnlisButtons();
    drawUnlisOuterRing();
    drawUnlisBandLegend();
    unlisUiNeedsRefreshOnTargetChange = true;
    unlisTimerDrawnValid = false;
    unlisRunningUiNeedsRedraw = false;
  }

  unsigned long nowMs = millis();
  if ((nowMs - unlisLastFrameMs) < UNLIS_FRAME_MS) {
    return;
  }

  float dt = (float)(nowMs - unlisLastFrameMs) / 1000.0f;
  unlisLastFrameMs = nowMs;

  if (unlisRunning && !unlisGameOver) {
    unlisScanAngleDeg = normalizeDeg360(unlisScanAngleDeg + (unlisCurrentScanDegPerSec() * dt));

    if (!unlisTargetActive && !unlisSecondTargetActive && nowMs >= unlisNextSpawnMs) {
      unlisSpawnTarget(nowMs);
    }

    unsigned long gameElapsedMs = nowMs - unlisGameStartMs;
    if (gameElapsedMs >= UNLIS_SECOND_TARGET_DELAY_MS &&
        !unlisSecondTargetActive &&
        nowMs >= unlisSecondTargetNextSpawnMs) {
      unlisGenerateRandomTarget(unlisSecondTargetOuter, unlisSecondTargetAngleDeg);
      unlisSecondTargetActive = true;
      unlisSecondTargetShownMs = nowMs;
      unlisOuterEdgeNeedsClean = true;
      unlisResetPttCooldown(nowMs);
      unlisUiNeedsRefreshOnTargetChange = true;
    }

    if (!unlisGreenStationActive && nowMs >= unlisGreenStationNextSpawnMs) {
      unlisGenerateRandomTarget(unlisGreenStationOuter, unlisGreenStationAngleDeg);
      unlisGreenStationActive = true;
      unlisGreenStationShownMs = nowMs;
      unlisGreenStationNextSpawnMs = nowMs + random(UNLIS_GREEN_STATION_RESPAWN_MIN_MS, UNLIS_GREEN_STATION_RESPAWN_MAX_MS + 1UL);
      unlisUiNeedsRefreshOnTargetChange = true;
    }

    if (unlisTargetActive && (nowMs - unlisTargetShownMs) >= unlisCurrentTargetLifeMs()) {
      unlisTargetActive = false;
      unlisResetPttCooldown(nowMs);
      unlisMissed++;
      if (unlisMissed >= 3) {
        unlisElapsedSecFrozen = (nowMs - unlisGameStartMs) / 1000UL;
        unlisGameOver = true;
        unlisRunning = false;
        unlisTargetActive = false;
        unlisSecondTargetActive = false;
        unlisGreenStationActive = false;
        unlisGameOverNeedsRedraw = true;
        unlisUiNeedsRefreshOnTargetChange = true;
      } else {
        if (!unlisSecondTargetActive) {
          unlisNextSpawnMs = nowMs + random(UNLIS_TARGET_RESPAWN_MIN_MS, UNLIS_TARGET_RESPAWN_MAX_MS + 1UL);
        }
        unlisUiNeedsRefreshOnTargetChange = true;
      }
    }

    if (unlisSecondTargetActive && (nowMs - unlisSecondTargetShownMs) >= unlisCurrentTargetLifeMs()) {
      unlisSecondTargetActive = false;
      unlisSecondTargetNextSpawnMs = nowMs + random(UNLIS_SECOND_TARGET_RESPAWN_MIN_MS, UNLIS_SECOND_TARGET_RESPAWN_MAX_MS + 1UL);
      unlisResetPttCooldown(nowMs);
      unlisMissed++;
      if (unlisMissed >= 3) {
        unlisElapsedSecFrozen = (nowMs - unlisGameStartMs) / 1000UL;
        unlisGameOver = true;
        unlisRunning = false;
        unlisTargetActive = false;
        unlisSecondTargetActive = false;
        unlisGreenStationActive = false;
        unlisGameOverNeedsRedraw = true;
      } else if (!unlisTargetActive) {
        unlisNextSpawnMs = nowMs + random(UNLIS_TARGET_RESPAWN_MIN_MS, UNLIS_TARGET_RESPAWN_MAX_MS + 1UL);
      }
      unlisUiNeedsRefreshOnTargetChange = true;
    }

    if (unlisGreenStationActive && (nowMs - unlisGreenStationShownMs) >= UNLIS_GREEN_STATION_LIFE_MS) {
      unlisGreenStationActive = false;
      unlisUiNeedsRefreshOnTargetChange = true;
    }
  }

  // Clear and redraw only radar zone; keep static buttons untouched.
  tft.fillCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, UNLIS_OUTER_R - 1, TFT_BLACK);

  bool redrawOuterRingNow = false;

  if (unlisOuterEdgeNeedsClean) {
    // Before drawing a new target, clean ring edge.
    for (int r = UNLIS_OUTER_R - 2; r <= UNLIS_OUTER_R + 6; r++) {
      tft.drawCircle(UNLIS_CENTER_X, UNLIS_CENTER_Y, r, TFT_BLACK);
    }
    redrawOuterRingNow = true;
    unlisOuterEdgeNeedsClean = false;
  }

  bool scoreMarkersRedrawn = false;
  if (unlisUiNeedsRefreshOnTargetChange) {
    drawUnlisScoreAndMissMarkers();
    unlisUiNeedsRefreshOnTargetChange = false;
    scoreMarkersRedrawn = true;
  }

  // Keep ring above left score/marker area and redraw after those elements.
  if (redrawOuterRingNow || scoreMarkersRedrawn) {
    drawUnlisOuterRing();
  }

  unsigned long elapsedSec = (nowMs - unlisGameStartMs) / 1000UL;
  if (!unlisTimerDrawnValid || elapsedSec != unlisLastTimerSecDrawn) {
    drawUnlisTimerValue(elapsedSec);
    unlisLastTimerSecDrawn = elapsedSec;
    unlisTimerDrawnValid = true;
  }

  // Keep radar and moving gameplay elements on top of the screen.
  drawUnlisRadarBase();
  drawUnlisTarget();
  drawUnlisArrows(unlisScanAngleDeg);
  unlisPrevScanAngleDeg = unlisScanAngleDeg;
  unlisArrowPrevValid = true;
}

// ========== Ekrany pomocnicze ==========

// Ekrany 6-10: Puste (tylko napis)
void drawScreenEmpty(int screenNum) {
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(3);
  
  String screenText = String(tr(TR_PAGE)) + " " + String(screenNum);
  int textWidth = screenText.length() * 18;
  int xPos = (480 - textWidth) / 2;
  int yPos = 110; // WyÄąâ€şrodkowane w pionie
  
  tft.setCursor(xPos, yPos);
  tft.print(screenText);

  // Strzałki nawigacyjne - duże, blisko krawędzi (takie same na wszystkich ekranach)
  int arrowY = 290;
  int arrowSize = 12;
  tft.fillTriangle(15, arrowY, 15 + arrowSize, arrowY - arrowSize, 15 + arrowSize, arrowY + arrowSize, TFT_RADIO_ORANGE);
  tft.fillTriangle(465, arrowY, 465 - arrowSize, arrowY - arrowSize, 465 - arrowSize, arrowY + arrowSize, TFT_RADIO_ORANGE);
  
  // Opcjonalny opis między strzałkami (małą czcionką)
  tft.setTextColor(0x52AA); // Ciemny szary
  tft.setTextSize(1);
  tft.setCursor(195, 286);
  tft.print("SWITCH SCREEN");
}

// Rysuj strzałki nawigacyjne < >
void drawNavigationArrows() {
  // Intentionally empty: nawigacja przez dotyk bez elementÄ‚łw UI
}

static volatile bool uiPendingScreen1Redraw = false;
static volatile bool uiPendingScreen2Redraw = false;
static volatile bool uiPendingScreen6Redraw = false;
static volatile bool uiPendingScreen7Redraw = false;
static volatile bool uiPendingAnyScreenRedraw = false;
static volatile uint8_t uiPendingAnyScreenId = SCREEN_HAM_CLOCK;
static portMUX_TYPE uiPendingRedrawMux = portMUX_INITIALIZER_UNLOCKED;

static void requestUiScreenRedraw(uint8_t pendingScreenId) {
  portENTER_CRITICAL(&uiPendingRedrawMux);
  uiPendingAnyScreenId = pendingScreenId;
  uiPendingAnyScreenRedraw = true;
  switch (pendingScreenId) {
    case SCREEN_HAM_CLOCK:
      uiPendingScreen1Redraw = true;
      break;
    case SCREEN_DX_CLUSTER:
      uiPendingScreen2Redraw = true;
      break;
    case SCREEN_APRS_IS:
    case SCREEN_APRS_RADAR:
      uiPendingScreen6Redraw = true;
      break;
    case SCREEN_POTA_CLUSTER:
      uiPendingScreen7Redraw = true;
      break;
    default:
      break;
  }
  portEXIT_CRITICAL(&uiPendingRedrawMux);
}

// Aktualizuj ekran 2 (DX Cluster) - wywoływane gdy przyjdą nowe spoty
void updateScreen2() {
  if (currentScreen == SCREEN_DX_CLUSTER && !inMenu && !aprsAlertScreenActive) {
    requestUiScreenRedraw(SCREEN_DX_CLUSTER);
  }
}

void updateScreen7() {
  if (currentScreen == SCREEN_POTA_CLUSTER && !inMenu && !aprsAlertScreenActive) {
    requestUiScreenRedraw(SCREEN_POTA_CLUSTER);
  }
}

void updateScreen6() {
  if ((currentScreen == SCREEN_APRS_IS || currentScreen == SCREEN_APRS_RADAR) && !inMenu && !aprsAlertScreenActive) {
    requestUiScreenRedraw(currentScreen);
  }
}

// Aktualizuj ekran 1 (Info) - wywoływane gdy zmieni się IP
void updateScreen1() {
  if (bootSequenceActive) {
    return;
  }
  if (currentScreen == SCREEN_HAM_CLOCK && !inMenu && !aprsAlertScreenActive) {
    requestUiScreenRedraw(SCREEN_HAM_CLOCK);
  }
}
#endif

void bootLogLine(const String &line) {
  Serial.println(line);
#ifdef ENABLE_TFT_DISPLAY
  if (tftInitialized) {
    tftBootPrintLine(line);
  }
#endif
  if (bootSequenceActive) {
    delay(500);
  }
}

// ========== FUNKCJE POMOCNICZE ==========
static uint16_t readBmp16(fs::File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

static uint32_t readBmp32(fs::File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read();
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read();
  return result;
}

static bool drawBmpFromFS(const String &filename, int16_t x, int16_t y) {
  if (!littleFsReady) {
    return false;
  }
  if ((x >= tft.width()) || (y >= tft.height())) {
    return false;
  }
  if (!LittleFS.exists(filename)) {
    return false;
  }
  fs::File bmpFS = LittleFS.open(filename, "r");
  if (!bmpFS) {
    return false;
  }

  uint32_t seekOffset;
  uint16_t w, h;
  if (readBmp16(bmpFS) != 0x4D42) {
    bmpFS.close();
    return false;
  }

  readBmp32(bmpFS);
  readBmp32(bmpFS);
  seekOffset = readBmp32(bmpFS);
  readBmp32(bmpFS);
  w = readBmp32(bmpFS);
  h = readBmp32(bmpFS);

  if (readBmp16(bmpFS) != 1 || readBmp16(bmpFS) != 24 || readBmp32(bmpFS) != 0) {
    bmpFS.close();
    return false;
  }

  y += h - 1;
  bool oldSwap = tft.getSwapBytes();
  tft.setSwapBytes(true);
  bmpFS.seek(seekOffset);

  uint16_t padding = (4 - ((w * 3) & 3)) & 3;
  uint8_t lineBuffer[w * 3 + padding];

  for (uint16_t row = 0; row < h; row++) {
    bmpFS.read(lineBuffer, sizeof(lineBuffer));
    uint8_t *bptr = lineBuffer;
    uint16_t *tptr = (uint16_t *)lineBuffer;
    for (uint16_t col = 0; col < w; col++) {
      uint8_t b = *bptr++;
      uint8_t g = *bptr++;
      uint8_t r = *bptr++;
      *tptr++ = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    }
    tft.pushImage(x, y--, w, 1, (uint16_t *)lineBuffer);
  }

  tft.setSwapBytes(oldSwap);
  bmpFS.close();
  return true;
}

static bool drawBmp16FromFS(const String &filename, int16_t x, int16_t y) {
  if (!littleFsReady) {
    return false;
  }
  if ((x >= tft.width()) || (y >= tft.height())) {
    return false;
  }
  if (!LittleFS.exists(filename)) {
    return false;
  }
  fs::File bmpFS = LittleFS.open(filename, "r");
  if (!bmpFS) {
    return false;
  }

  uint32_t seekOffset;
  uint16_t w, h;
  if (readBmp16(bmpFS) != 0x4D42) {
    bmpFS.close();
    return false;
  }

  readBmp32(bmpFS);  // File size
  readBmp32(bmpFS);  // Reserved
  seekOffset = readBmp32(bmpFS);  // Offset to pixel data
  readBmp32(bmpFS);  // DIB header size
  w = readBmp32(bmpFS);  // Width
  h = readBmp32(bmpFS);  // Height

  uint16_t planes = readBmp16(bmpFS);
  uint16_t bpp = readBmp16(bmpFS);
  uint32_t compression = readBmp32(bmpFS);

  // Sprawdź czy to 16-bit z maskami bitowymi (BI_BITFIELDS = 3)
  if (planes != 1 || bpp != 16 || (compression != 0 && compression != 3)) {
    bmpFS.close();
    return false;
  }

  // Jeśli compression == 3, pomiń maski bitowe (12 bajtów)
  if (compression == 3) {
    readBmp32(bmpFS);
    readBmp32(bmpFS);
    readBmp32(bmpFS);
  }

  // BMP zapisuje od dołu do góry, więc zaczynamy od dolnego wiersza
  y += h - 1;
  bool oldSwap = tft.getSwapBytes();
  tft.setSwapBytes(true);
  bmpFS.seek(seekOffset);

  uint16_t rowSize = ((16 * w + 31) / 32) * 4;  // Wyrównane do 4 bajtów
  uint16_t padding = rowSize - (w * 2);
  
  // Użyj alokacji na stercie zamiast stosu dla lepszego wyrównania
  uint16_t lineBufferSize = rowSize;
  uint8_t* lineBuffer = (uint8_t*)malloc(lineBufferSize);
  if (!lineBuffer) {
    tft.setSwapBytes(oldSwap);
    bmpFS.close();
    return false;
  }

  for (uint16_t row = 0; row < h; row++) {
    bmpFS.read(lineBuffer, rowSize);
    // Dane są już w RGB565, przekazujemy bezpośrednio
    tft.pushImage(x, y--, w, 1, (uint16_t *)lineBuffer);
  }

  free(lineBuffer);
  tft.setSwapBytes(oldSwap);
  bmpFS.close();
  return true;
}

String getAPRSSymbolRaw(const APRSStation &station) {
  if (station.symbolTable.length() > 0 && station.symbol.length() > 0) {
    return station.symbolTable + station.symbol;
  }
  if (station.symbol.length() > 0) {
    return station.symbol;
  }
  return "-";
}

// ZwrÄ‚łĂ„â€ˇ krÄ‚łtki opis symbolu (<=10 znakÄ‚łw) zgodny z tabelĂ„â€¦ APRS
String getAPRSSymbolShort(const APRSStation &station) {
  String raw = getAPRSSymbolRaw(station);
  if (raw == "/-") return "HOUSE";
  if (raw == "/>") return "CAR";
  if (raw == "/#") return "DIGI";
  if (raw == "/r") return "REPEATER";
  if (raw == "/[") return "HUMAN";
  if (raw == "/l") return "LAPTOP";
  if (raw == "/L") return "LOGIN";
  if (raw == "/_") return "WX";
  if (raw == "/s") return "SHIP";
  if (raw == "/a") return "AMBUL";
  if (raw == "/b") return "BIKE";
  if (raw == "/c") return "ICP";
  if (raw == "/d") return "FIRE";
  if (raw == "/f") return "FIRETRK";
  if (raw == "/h") return "HOSP";
  if (raw == "/y") return "YAGI";
  if (raw == "/g") return "GLIDER";
  if (raw == "/k") return "TRUCK";
  if (raw == "/u") return "TRUCK";
  if (raw == "/v") return "VAN";
  if (raw == "/p") return "ROVER";
  if (raw == "/o") return "EOC";
  if (raw == "/t") return "TRKSTOP";
  if (raw == "/m") return "MICERPT";
  if (raw == "/n") return "NODE";
  if (raw == "/q") return "GRID";
  if (raw == "La" || raw == "L_" ) return "LORA";
  if (raw == "L#" || raw == "L&") return "LORA";
  return raw;
}

static String getAprsAlertSymbolDescription(const APRSStation &station) {
  String symbolShort = getAPRSSymbolShort(station);
  if (symbolShort == "HOUSE") return "Home station";
  if (symbolShort == "CAR") return "CAR";
  if (symbolShort == "DIGI") return "Digipeater";
  if (symbolShort == "REPEATER") return "Repeater";
  if (symbolShort == "HUMAN") return "Pedestrian";
  if (symbolShort == "LAPTOP") return "Mobile station";
  if (symbolShort == "LOGIN") return "Login point";
  if (symbolShort == "WX") return "Weather station";
  if (symbolShort == "SHIP") return "Ship";
  if (symbolShort == "AMBUL") return "Ambulans";
  if (symbolShort == "BIKE") return "Bicycle";
  if (symbolShort == "FIRE") return "Fire";
  if (symbolShort == "FIRETRK") return "Fire truck";
  if (symbolShort == "HOSP") return "Hospital";
  if (symbolShort == "GLIDER") return "Glider";
  if (symbolShort == "TRUCK") return "Truck";
  if (symbolShort == "VAN") return "Van";
  if (symbolShort == "ROVER") return "Rover";
  if (symbolShort == "NODE") return "Node";
  if (symbolShort == "LORA") return "LoRa";
  return String("Symbol: ") + symbolShort;
}

static bool isAprsWeatherStationSymbol(const APRSStation &station) {
  if (station.symbol == "_") {
    return true;
  }
  String symbolShort = getAPRSSymbolShort(station);
  if (symbolShort == "WX") {
    return true;
  }
  String rawSymbol = getAPRSSymbolRaw(station);
  return rawSymbol.length() >= 2 && rawSymbol.charAt(rawSymbol.length() - 1) == '_';
}

static bool aprsWxReadDigits(const String &text, int start, int len, int &valueOut) {
  if (start < 0 || len <= 0 || (start + len) > text.length()) {
    return false;
  }
  int value = 0;
  for (int i = 0; i < len; i++) {
    char c = text.charAt(start + i);
    if (!isDigit(c)) {
      return false;
    }
    value = (value * 10) + (c - '0');
  }
  valueOut = value;
  return true;
}

static bool decodeAprsWxFromComment(const String &commentRaw, AprsWxDecoded &wx) {
  wx = AprsWxDecoded();

  String text = commentRaw;
  text.trim();
  if (text.length() == 0) {
    return false;
  }

  int dirDeg = 0;
  int speedMph = 0;
  if (text.length() >= 7 &&
      aprsWxReadDigits(text, 0, 3, dirDeg) &&
      text.charAt(3) == '/' &&
      aprsWxReadDigits(text, 4, 3, speedMph)) {
    wx.hasWindDir = true;
    wx.windDirDeg = dirDeg % 360;
    wx.hasWindSpeed = true;
    wx.windSpeedKmh = speedMph * 1.60934f;
  }

  for (int i = 0; i < text.length(); i++) {
    char tag = text.charAt(i);
    int digits = 0;
    switch (tag) {
      case 'c':
      case 's':
      case 'g':
      case 't':
      case 'r':
      case 'p':
      case 'P':
        digits = 3;
        break;
      case 'h':
        digits = 2;
        break;
      case 'b':
        digits = 5;
        break;
      default:
        break;
    }

    if (digits == 0) {
      continue;
    }

    int value = 0;
    if (!aprsWxReadDigits(text, i + 1, digits, value)) {
      continue;
    }

    if (tag == 'c') {
      wx.hasWindDir = true;
      wx.windDirDeg = value % 360;
    } else if (tag == 's' || (tag == 'g' && !wx.hasWindSpeed)) {
      wx.hasWindSpeed = true;
      wx.windSpeedKmh = value * 1.60934f;
    } else if (tag == 't') {
      wx.hasTempC = true;
      wx.tempC = (value - 32.0f) * (5.0f / 9.0f);
    } else if (tag == 'h') {
      wx.hasHumidity = true;
      wx.humidityPct = (value == 0) ? 100 : value;
      if (wx.humidityPct > 100) {
        wx.humidityPct = 100;
      }
    } else if (tag == 'b') {
      wx.hasPressure = true;
      wx.pressureHpa = value / 10.0f;
    }
  }

  wx.hasAny = wx.hasWindDir || wx.hasWindSpeed || wx.hasTempC || wx.hasHumidity || wx.hasPressure;
  return wx.hasAny;
}

static bool isAprsWxPayloadValidForAlert(const APRSStation &station) {
  if (!isAprsWeatherStationSymbol(station)) {
    return false;
  }

  AprsWxDecoded wx;
  if (!decodeAprsWxFromComment(station.comment, wx)) {
    return false;
  }

  // Minimalny warunek poprawnej ramki WX do alertu: temperatura + wilgotnosc.
  return wx.hasTempC && wx.hasHumidity;
}

static void drawAprsAlertWrappedComment(const String &comment, int x, int y, int widthPx, int maxLines, int textSize) {
  String text = comment;
  text.trim();
  if (text.length() == 0) {
    text = "-";
  }

  const int safeTextSize = (textSize < 1) ? 1 : textSize;
  const int charWidth = 6 * safeTextSize;
  const int lineHeight = (8 * safeTextSize) + 2;
  const int maxCharsPerLine = (charWidth > 0) ? (widthPx / charWidth) : widthPx;
  int cursor = 0;
  int line = 0;

  while (cursor < text.length() && line < maxLines) {
    int end = cursor + maxCharsPerLine;
    if (end >= text.length()) {
      end = text.length();
    } else {
      int split = end;
      while (split > cursor && text.charAt(split) != ' ') {
        split--;
      }
      if (split > cursor) {
        end = split;
      }
    }

    String lineText = text.substring(cursor, end);
    lineText.trim();
    if (lineText.length() > 0) {
      tft.setCursor(x, y + line * lineHeight);
      tft.print(lineText);
      line++;
    }

    cursor = end;
    while (cursor < text.length() && text.charAt(cursor) == ' ') {
      cursor++;
    }
  }
}

static void drawAprsAlertCarIcon(int centerX, int topY, uint16_t color) {
  const int bodyW = 60;
  const int bodyH = 14;
  const int roofW = 33;
  const int roofH = 9;

  const int bodyX = centerX - (bodyW / 2);
  const int roofX = centerX - (roofW / 2);
  const int roofY = topY;
  const int bodyY = topY + 9;

  tft.fillRoundRect(bodyX, bodyY, bodyW, bodyH, 2, color);
  tft.fillRoundRect(roofX, roofY, roofW, roofH, 2, color);

  tft.fillCircle(bodyX + 13, bodyY + bodyH, 6, TFT_DARKGREY);
  tft.fillCircle(bodyX + bodyW - 13, bodyY + bodyH, 6, TFT_DARKGREY);
  tft.fillCircle(bodyX + 13, bodyY + bodyH, 3, TFT_LIGHTGREY);
  tft.fillCircle(bodyX + bodyW - 13, bodyY + bodyH, 3, TFT_LIGHTGREY);
  tft.fillCircle(bodyX + 3, bodyY + 3, 2, TFT_YELLOW);
  tft.fillCircle(bodyX + 3, bodyY + 8, 2, TFT_YELLOW);
}

static void drawAprsAlertHouseIcon(int centerX, int topY) {
  const int bodyW = 39;
  const int bodyH = 24;
  const int bodyX = centerX - (bodyW / 2);
  const int bodyY = topY + 10;

  // Ściany domu (żółte)
  tft.fillRect(bodyX, bodyY, bodyW, bodyH, TFT_YELLOW);

  // Dach dwuspadowy (brązowy)
  const uint16_t roofColor = TFT_GREEN;
  int roofTopY = topY;
  tft.fillTriangle(centerX, roofTopY, bodyX - 3, bodyY + 1, bodyX + bodyW + 3, bodyY + 1, roofColor);

  // Drzwi i okna dla czytelności
  tft.fillRect(centerX - 4, bodyY + 12, 8, 12, TFT_DARKGREY);
  tft.fillRect(bodyX + 4, bodyY + 6, 6, 6, TFT_LIGHTGREY);
  tft.fillRect(bodyX + bodyW - 10, bodyY + 6, 6, 6, TFT_LIGHTGREY);
}

static void drawAprsAlertHumanIcon(int centerX, int topY) {
  const uint16_t bodyColor = TFT_RED;
  const uint16_t headColor = TFT_BLUE;
  int headX = centerX;
  int headY = topY + 5;

  // Głowa
  tft.fillCircle(headX, headY, 4, headColor);

  // Tułów (lekko pochylony)
  tft.drawLine(headX, headY + 4, headX + 2, headY + 18, bodyColor);
  tft.drawLine(headX + 1, headY + 4, headX + 3, headY + 18, bodyColor);
  tft.drawLine(headX + 2, headY + 4, headX + 4, headY + 18, bodyColor);
  tft.drawLine(headX + 3, headY + 4, headX + 5, headY + 18, bodyColor);

  // Ręce (ruch spaceru)
  tft.drawLine(headX + 1, headY + 8, headX - 6, headY + 12, bodyColor);
  tft.drawLine(headX + 3, headY + 8, headX + 10, headY + 11, bodyColor);
  tft.drawLine(headX + 2, headY + 9, headX - 5, headY + 13, bodyColor);
  tft.drawLine(headX + 4, headY + 9, headX + 11, headY + 12, bodyColor);
  tft.drawLine(headX + 3, headY + 10, headX - 4, headY + 14, bodyColor);
  tft.drawLine(headX + 5, headY + 10, headX + 12, headY + 13, bodyColor);

  // Nogi (jedna do przodu, druga do tyłu)
  tft.drawLine(headX + 3, headY + 18, headX - 4, headY + 28, bodyColor);
  tft.drawLine(headX + 3, headY + 18, headX + 12, headY + 27, bodyColor);
  tft.drawLine(headX + 4, headY + 18, headX - 3, headY + 28, bodyColor);
  tft.drawLine(headX + 4, headY + 18, headX + 13, headY + 27, bodyColor);
  tft.drawLine(headX + 5, headY + 18, headX - 2, headY + 28, bodyColor);
  tft.drawLine(headX + 5, headY + 18, headX + 14, headY + 27, bodyColor);
}

static void drawAprsAlertCompass(int centerX, int centerY, int radius, bool hasBearing, float bearingDeg) {
  tft.drawCircle(centerX, centerY, radius, TFT_DARKGREY);
  tft.drawCircle(centerX, centerY, radius - 1, TFT_DARKGREY);

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(centerX - 3, centerY - radius - 9);
  tft.print("N");

  tft.drawFastVLine(centerX, centerY - radius + 2, 4, TFT_DARKGREY);
  tft.drawFastVLine(centerX, centerY + radius - 5, 4, TFT_DARKGREY);
  tft.drawFastHLine(centerX - radius + 2, centerY, 4, TFT_DARKGREY);
  tft.drawFastHLine(centerX + radius - 5, centerY, 4, TFT_DARKGREY);

  if (!hasBearing) {
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(centerX - 6, centerY + radius + 2);
    tft.print("---");
    return;
  }

  float angleRad = bearingDeg * M_PI / 180.0f;
  int tipX = centerX + (int)(sin(angleRad) * (radius - 3));
  int tipY = centerY - (int)(cos(angleRad) * (radius - 3));

  int perpX = (int)round(cos(angleRad));
  int perpY = (int)round(sin(angleRad));
  tft.drawLine(centerX, centerY, tipX, tipY, TFT_CYAN);
  tft.drawLine(centerX + perpX, centerY + perpY, tipX + perpX, tipY + perpY, TFT_CYAN);
  tft.fillCircle(tipX, tipY, 2, TFT_CYAN);
  tft.fillCircle(centerX, centerY, 1, TFT_CYAN);

  String bearingTxt = String((int)(bearingDeg + 0.5f)) + "d";
  int txtX = centerX - (bearingTxt.length() * 3);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(txtX, centerY + radius + 2);
  tft.print(bearingTxt);
}

static void drawAprsAlertFrame(bool pulseOn) {
  const int frameX = 10;
  const int frameY = 55;
  const int frameW = 460;
  const int frameH = 260;
  uint16_t color = pulseOn ? TFT_RADIO_ORANGE : TFT_DARKGREY;
  tft.drawRect(frameX, frameY, frameW, frameH, color);
  tft.drawRect(frameX + 1, frameY + 1, frameW - 2, frameH - 2, color);
}

static void drawAprsAlertCloseButton() {
  tft.fillRoundRect(APRS_ALERT_CLOSE_BTN_X, APRS_ALERT_CLOSE_BTN_Y, APRS_ALERT_CLOSE_BTN_W, APRS_ALERT_CLOSE_BTN_H, 5, TFT_RADIO_ORANGE);
  tft.drawRoundRect(APRS_ALERT_CLOSE_BTN_X, APRS_ALERT_CLOSE_BTN_Y, APRS_ALERT_CLOSE_BTN_W, APRS_ALERT_CLOSE_BTN_H, 5, TFT_BLACK);
  tft.drawLine(APRS_ALERT_CLOSE_BTN_X + 7, APRS_ALERT_CLOSE_BTN_Y + 7,
               APRS_ALERT_CLOSE_BTN_X + APRS_ALERT_CLOSE_BTN_W - 8, APRS_ALERT_CLOSE_BTN_Y + APRS_ALERT_CLOSE_BTN_H - 8, TFT_BLACK);
  tft.drawLine(APRS_ALERT_CLOSE_BTN_X + APRS_ALERT_CLOSE_BTN_W - 8, APRS_ALERT_CLOSE_BTN_Y + 7,
               APRS_ALERT_CLOSE_BTN_X + 7, APRS_ALERT_CLOSE_BTN_Y + APRS_ALERT_CLOSE_BTN_H - 8, TFT_BLACK);
  tft.drawLine(APRS_ALERT_CLOSE_BTN_X + 8, APRS_ALERT_CLOSE_BTN_Y + 7,
               APRS_ALERT_CLOSE_BTN_X + APRS_ALERT_CLOSE_BTN_W - 7, APRS_ALERT_CLOSE_BTN_Y + APRS_ALERT_CLOSE_BTN_H - 8, TFT_BLACK);
  tft.drawLine(APRS_ALERT_CLOSE_BTN_X + 7, APRS_ALERT_CLOSE_BTN_Y + 8,
               APRS_ALERT_CLOSE_BTN_X + APRS_ALERT_CLOSE_BTN_W - 8, APRS_ALERT_CLOSE_BTN_Y + APRS_ALERT_CLOSE_BTN_H - 7, TFT_BLACK);
  tft.drawLine(APRS_ALERT_CLOSE_BTN_X + APRS_ALERT_CLOSE_BTN_W - 7, APRS_ALERT_CLOSE_BTN_Y + 7,
               APRS_ALERT_CLOSE_BTN_X + 8, APRS_ALERT_CLOSE_BTN_Y + APRS_ALERT_CLOSE_BTN_H - 8, TFT_BLACK);
  tft.drawLine(APRS_ALERT_CLOSE_BTN_X + APRS_ALERT_CLOSE_BTN_W - 8, APRS_ALERT_CLOSE_BTN_Y + 8,
               APRS_ALERT_CLOSE_BTN_X + 7, APRS_ALERT_CLOSE_BTN_Y + APRS_ALERT_CLOSE_BTN_H - 7, TFT_BLACK);
}

static bool isAprsAlertCloseButtonHit(uint16_t x, uint16_t y) {
  int hitX = APRS_ALERT_CLOSE_BTN_X + ((APRS_ALERT_CLOSE_BTN_W - APRS_ALERT_CLOSE_HIT_W) / 2);
  int hitY = APRS_ALERT_CLOSE_BTN_Y + ((APRS_ALERT_CLOSE_BTN_H - APRS_ALERT_CLOSE_HIT_H) / 2);
  if (hitX < 0) hitX = 0;
  if (hitY < 0) hitY = 0;

  int hitRight = hitX + APRS_ALERT_CLOSE_HIT_W;
  int hitBottom = hitY + APRS_ALERT_CLOSE_HIT_H;
  if (hitRight > 480) hitRight = 480;
  if (hitBottom > 320) hitBottom = 320;

  return x >= hitX && x < hitRight && y >= hitY && y < hitBottom;
}

static void dismissAprsAlertScreen() {
  if (!aprsAlertScreenActive) {
    return;
  }
  aprsAlertScreenActive = false;
  drawScreen(currentScreen);
  if (currentScreen == SCREEN_HAM_CLOCK) {
    updateScreen1();
  }
}

static void setRgbLedChannel(uint8_t pin, bool on) {
  digitalWrite(pin, on ? LOW : HIGH);
}

static void applyRgbLedState(bool redOn, bool greenOn, bool blueOn) {
  setRgbLedChannel(RGB_LED_RED_PIN, redOn);
  setRgbLedChannel(RGB_LED_GREEN_PIN, greenOn);
  setRgbLedChannel(RGB_LED_BLUE_PIN, blueOn);
}

static void initStatusRgbLed() {
  pinMode(RGB_LED_RED_PIN, OUTPUT);
  pinMode(RGB_LED_GREEN_PIN, OUTPUT);
  pinMode(RGB_LED_BLUE_PIN, OUTPUT);

  rgbLedPrevWifiConnected = false;
  rgbRedBlinkLastToggleMs = millis();
  rgbRedBlinkStateOn = false;
  rgbBlueAprsAlertActive = false;
  rgbBlueAprsAlertUntilMs = 0;
  rgbBlueAprsLastToggleMs = 0;
  rgbBlueAprsStateOn = false;

  applyRgbLedState(false, false, false);
}

static int normalizeLedAlertDurationMs(int durationMs) {
  if (durationMs < 100) return 100;
  if (durationMs > 60000) return 60000;
  return durationMs;
}

static void applyLedAlertDurationMs(int durationMs) {
  ledAlertDurationMs = normalizeLedAlertDurationMs(durationMs);
}

static int normalizeLedAlertBlinkMs(int blinkMs) {
  if (blinkMs < 50) return 50;
  if (blinkMs > 5000) return 5000;
  return blinkMs;
}

static void applyLedAlertBlinkMs(int blinkMs) {
  ledAlertBlinkMs = normalizeLedAlertBlinkMs(blinkMs);
}

static void triggerAprsRgbLedAlert() {
  if (!enableLedAlert) {
    return;
  }

  unsigned long nowMs = millis();
  rgbBlueAprsAlertActive = true;
  rgbBlueAprsAlertUntilMs = nowMs + (unsigned long)ledAlertDurationMs;
  rgbBlueAprsLastToggleMs = nowMs;
  rgbBlueAprsStateOn = true;
  applyRgbLedState(false, false, true);
}

static void updateStatusRgbLed() {
  unsigned long nowMs = millis();

  if (rgbBlueAprsAlertActive) {
    if ((long)(nowMs - rgbBlueAprsAlertUntilMs) >= 0) {
      rgbBlueAprsAlertActive = false;
      rgbBlueAprsStateOn = false;
    } else if (nowMs - rgbBlueAprsLastToggleMs >= (unsigned long)ledAlertBlinkMs) {
      rgbBlueAprsLastToggleMs = nowMs;
      rgbBlueAprsStateOn = !rgbBlueAprsStateOn;
    }

    if (rgbBlueAprsAlertActive) {
      applyRgbLedState(false, false, rgbBlueAprsStateOn);
      return;
    }
  }

  applyRgbLedState(false, false, false);
}

void ALERT_Screen(const APRSStation &station) {
  if (!tftInitialized) {
    return;
  }

  aprsAlertScreenStation = station;
  aprsAlertScreenActive = true;
  aprsAlertScreenUntilMs = millis() + ((unsigned long)aprsAlertScreenSeconds * 1000UL);
  aprsAlertFrameLastToggleMs = millis();
  aprsAlertFramePulseOn = true;

  tft.fillScreen(TFT_BLACK);
  
  // Górny pasek z nagłówkiem
  tft.fillRect(0, 0, 480, 45, TFT_RADIO_ORANGE);
  tft.drawFastHLine(0, 44, 480, 0x8410);

  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(3);
  const String headerText = "APRS ALERT";
  const int headerX = (480 - ((int)headerText.length() * 18)) / 2;
  tft.setCursor(headerX, 10);
  tft.print(headerText);
  
  drawAprsAlertCloseButton();

  // Główna ramka wokół zawartości
  const int frameX = 10;
  const int frameY = 55;
  const int frameW = 460;
  const int frameH = 260;
  tft.drawRect(frameX, frameY, frameW, frameH, TFT_RADIO_ORANGE);
  tft.drawRect(frameX + 1, frameY + 1, frameW - 2, frameH - 2, TFT_DARKGREY);

  String callsign = station.callsign;
  callsign.toUpperCase();
  if (callsign.length() == 0) {
    callsign = "-";
  }

  // ZNAK WYWOŁAWCZY - duży, wyśrodkowany w ramce
  tft.setTextColor(TFT_WHITE);
  int callTextSize = (callsign.length() <= 6) ? 6 : 5;
  tft.setTextSize(callTextSize);
  int charWidth = (callTextSize == 6) ? 36 : 30;
  int callX = frameX + (frameW - (callsign.length() * charWidth)) / 2;
  int callY = frameY + 20;
  
  // Ramka wokół znaku
  int callBoxPadding = 10;
  int callBoxW = callsign.length() * charWidth + 2 * callBoxPadding;
  int callBoxH = charWidth + callBoxPadding;
  int callBoxX = frameX + (frameW - callBoxW) / 2;
  tft.drawRect(callBoxX, callY - 5, callBoxW, callBoxH, TFT_WHITE);
  tft.drawRect(callBoxX + 1, callY - 4, callBoxW - 2, callBoxH - 2, TFT_RADIO_ORANGE);
  
  tft.setCursor(callX, callY);
  tft.print(callsign);

  // Częstotliwość pod znakiem
  String freqText = "-";
  if (station.freqMHz > 0.0f) {
    freqText = String(station.freqMHz, 3) + " MHz";
  }
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(3);
  int freqX = frameX + (frameW - (freqText.length() * 18)) / 2;
  tft.setCursor(freqX, callY + callBoxH + 10);
  tft.print(freqText);

  // KOMPAS i ODLEGŁOŚĆ - na dole ramki, obok siebie
  String distanceText = "- km";
  if (station.hasLatLon && station.distance > 0.0f) {
    distanceText = String(station.distance, 1) + " km";
  }
  
  // Kompas po lewej - wyżej
  const int compassX = frameX + 80;
  const int compassY = frameY + frameH - 130;
  const int compassR = 40;
  
  bool hasBearing = false;
  float bearingDeg = 0.0f;
  if (userLatLonValid && station.hasLatLon) {
    hasBearing = true;
    bearingDeg = calculateBearing(userLat, userLon, (double)station.lat, (double)station.lon);
  }
  drawAprsAlertCompass(compassX, compassY, compassR, hasBearing, bearingDeg);

  // Odległość pod kompasem
  tft.setTextColor(TFT_RADIO_ORANGE);
  tft.setTextSize(2);
  int distX = compassX - ((int)distanceText.length() * 12) / 2;
  tft.setCursor(distX, compassY + compassR + 15);
  tft.print(distanceText);

  // Ikona stacji po prawej stronie (bez opisu, tylko ikona)
  String iconShort = getAPRSSymbolShort(station);
  uint16_t iconColor = TFT_WHITE;
  if (iconShort == "CAR") {
    iconColor = TFT_RED;
  } else if (iconShort == "HOUSE") {
    iconColor = TFT_YELLOW;
  } else if (iconShort == "HUMAN") {
    iconColor = TFT_BLUE;
  }
  
  int iconCenterX = frameX + frameW - 130;
  int iconCenterY = frameY + frameH - 140;
  
  if (iconShort == "CAR") {
    drawAprsAlertCarIcon(iconCenterX, iconCenterY, iconColor);
  } else if (iconShort == "HOUSE") {
    drawAprsAlertHouseIcon(iconCenterX, iconCenterY);
  } else if (iconShort == "HUMAN") {
    drawAprsAlertHumanIcon(iconCenterX, iconCenterY);
  } else {
    // Domyślnie dom
    drawAprsAlertHouseIcon(iconCenterX, iconCenterY);
  }

  // Typ stacji pod ikoną - dostosowany do nowej pozycji
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(2);
  String iconDesc = getAprsAlertSymbolDescription(station);
  int descX = iconCenterX - ((int)iconDesc.length() * 12) / 2;
  tft.setCursor(descX, iconCenterY + 55);
  tft.print(iconDesc);
}

void updateAlertScreenTimeout() {
  if (!aprsAlertScreenActive) {
    return;
  }

  unsigned long nowMs = millis();
  if (nowMs - aprsAlertFrameLastToggleMs >= APRS_ALERT_FRAME_PULSE_MS) {
    aprsAlertFrameLastToggleMs = nowMs;
    aprsAlertFramePulseOn = !aprsAlertFramePulseOn;
    drawAprsAlertFrame(aprsAlertFramePulseOn);
  }

  if ((long)(millis() - aprsAlertScreenUntilMs) < 0) {
    return;
  }

  dismissAprsAlertScreen();
}

// WyciĂ„â€¦gnij czĂ„â„˘stotliwoÄąâ€şĂ„â€ˇ z komentarza APRS (MHz)
bool extractAPRSFrequencyMHz(const String &text, float &outMHz) {
  const int len = text.length();
  for (int i = 0; i < len; i++) {
    char c = text.charAt(i);
    if (isDigit(c) && (i == 0 || !isDigit(text.charAt(i - 1)))) {
      int j = i;
      bool hasDot = false;
      while (j < len) {
        char cj = text.charAt(j);
        if (cj == '.') {
          if (hasDot) break;
          hasDot = true;
          j++;
          continue;
        }
        if (!isDigit(cj)) break;
        j++;
      }
      if (j <= i) continue;

      String numStr = text.substring(i, j);
      float value = numStr.toFloat();
      if (value <= 0.0f) continue;

      int k = j;
      while (k < len && (text.charAt(k) == ' ' || text.charAt(k) == '\t')) {
        k++;
      }
      if (k + 2 < len) {
        char m1 = text.charAt(k);
        char m2 = text.charAt(k + 1);
        char m3 = text.charAt(k + 2);
        if ((m1 == 'M' || m1 == 'm') && (m2 == 'H' || m2 == 'h') && (m3 == 'Z' || m3 == 'z')) {
          outMHz = value;
          return true;
        }
      }

      // Bez jednostki: akceptuj tylko liczby dziesiĂ„â„˘tne w paÄąâ€şmie VHF/UHF
      if (hasDot && value >= 30.0f && value <= 1000.0f) {
        outMHz = value;
        return true;
      }
    }
  }
  return false;
}

// Konwersja Maidenhead Locator do wspÄ‚łÄąâ€šrzĂ„â„˘dnych geograficznych
void locatorToLatLon(String locator, double &lat, double &lon) {
  if (locator.length() < 4) {
    lat = 0;
    lon = 0;
    return;
  }
  
  // Pierwsze 2 znaki - kwadrat (field)
  char c1 = toupper(locator.charAt(0));
  char c2 = toupper(locator.charAt(1));
  lon = (c1 - 'A') * 20 - 180;
  lat = (c2 - 'A') * 10 - 90;
  
  // NastĂ„â„˘pne 2 znaki - kwadrat (square)
  if (locator.length() >= 4) {
    int n1 = locator.charAt(2) - '0';
    int n2 = locator.charAt(3) - '0';
    lon += n1 * 2;
    lat += n2 * 1;
  }
  
  // ÄąĹˇrodek kwadratu
  lon += 1;
  lat += 0.5;
}

void updateUserLatLonFromLocator() {
  if (userLocator.length() >= 4) {
    locatorToLatLon(userLocator, userLat, userLon);
    userLatLonValid = true;
  } else {
    userLat = 0.0;
    userLon = 0.0;
    userLatLonValid = false;
  }
}

// SprawdÄąĹź czy tekst wspÄ‚łÄąâ€šrzĂ„â„˘dnych ma tylko cyfry i kropkĂ„â„˘ (nie obsÄąâ€šugujemy formatÄ‚łw skompresowanych)
static bool isAprsNumeric(const String &raw_aprs) {
    if (raw_aprs.length() == 0) {
        return false;
    }
    int dotCount = 0;
    for (size_t i = 0; i < raw_aprs.length(); i++) {
        char c = raw_aprs.charAt(i);
        if (c == '.') {
            dotCount++;
            if (dotCount > 1) {
                return false;
            }
        } else if (!isDigit(c)) {
            return false;
        }
    }
    return true;
}

// Konwersja surowej pozycji APRS na stopnie dziesiĂ„â„˘tne
// Konwersja formatu APRS (DDMM.mmN lub DDDMM.mmE) na stopnie dziesiĂ„â„˘tne
// Format APRS: DDMM.mm dla szerokoÄąâ€şci, DDDMM.mm dla dÄąâ€šugoÄąâ€şci
float convertToDecimal(String raw_aprs, char direction) {
    if (raw_aprs.length() < 4) {
        LOGV_PRINTF("[APRS] convertToDecimal: za krÄ‚łtki string: %s\n", raw_aprs.c_str());
        return NAN;
    }
    if (!isAprsNumeric(raw_aprs)) {
        LOGV_PRINTF("[APRS] convertToDecimal: nie-numeryczny format: %s\n", raw_aprs.c_str());
        return NAN;
    }

    double degrees = 0.0;
    double minutes = 0.0;

    if (direction == 'N' || direction == 'S') {
        // SzerokoÄąâ€şĂ„â€ˇ (Latitude): pierwsze 2 znaki to stopnie (DDMM.mm)
        // PrzykÄąâ€šad: "5202.40" -> degrees=52, minutes=02.40
        if (raw_aprs.length() >= 2) {
            String degStr = raw_aprs.substring(0, 2);
            degrees = degStr.toDouble();
        }
        if (raw_aprs.length() > 2) {
            String minStr = raw_aprs.substring(2);
            minutes = minStr.toDouble();
        }
    } else if (direction == 'E' || direction == 'W') {
        // DÄąâ€šugoÄąâ€şĂ„â€ˇ (Longitude): pierwsze 3 znaki to stopnie (DDDMM.mm)
        // PrzykÄąâ€šad: "01655.12" -> degrees=016, minutes=55.12
        if (raw_aprs.length() >= 3) {
            String degStr = raw_aprs.substring(0, 3);
            degrees = degStr.toDouble();
        }
        if (raw_aprs.length() > 3) {
            String minStr = raw_aprs.substring(3);
            minutes = minStr.toDouble();
        }
    } else {
        LOGV_PRINTF("[APRS] convertToDecimal: nieznany kierunek: %c\n", direction);
        return NAN;
    }

    // Walidacja zakresÄ‚łw
    if (direction == 'N' || direction == 'S') {
        if (degrees < 0 || degrees > 90 || minutes < 0 || minutes >= 60) {
            LOGV_PRINTF("[APRS] convertToDecimal: nieprawidÄąâ€šowa szerokoÄąâ€şĂ„â€ˇ: %s%c (deg=%.1f, min=%.2f)\n", 
                       raw_aprs.c_str(), direction, degrees, minutes);
            return NAN;
        }
    } else {
        if (degrees < 0 || degrees > 180 || minutes < 0 || minutes >= 60) {
            LOGV_PRINTF("[APRS] convertToDecimal: nieprawidÄąâ€šowa dÄąâ€šugoÄąâ€şĂ„â€ˇ: %s%c (deg=%.1f, min=%.2f)\n", 
                       raw_aprs.c_str(), direction, degrees, minutes);
            return NAN;
        }
    }

    double decimal = degrees + (minutes / 60.0);

    // JeÄąâ€şli PoÄąâ€šudnie (S) lub ZachÄ‚łd (W), wynik musi byĂ„â€ˇ ujemny
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    LOGV_PRINTF("[APRS] convertToDecimal: %s%c -> %.6f (deg=%.0f, min=%.2f)\n", 
               raw_aprs.c_str(), direction, decimal, degrees, minutes);

    return (float)decimal;
}

// ===== APRS Mic-E / Compressed / Uncompressed decoding (ported from ESP32APRS_Audio) =====
static bool validSymTableCompressed(char c) {
    return (c == '/' || c == '\\' || (c >= 0x41 && c <= 0x5A) || (c >= 0x61 && c <= 0x6A)); // [\/\\A-Za-j]
}

static bool validSymTableUncompressed(char c) {
    return (c == '/' || c == '\\' || (c >= 0x41 && c <= 0x5A) || (c >= 0x30 && c <= 0x39)); // [\/\\A-Z0-9]
}

static bool parseFixedUnsigned(const char *src, int offset, int digits, unsigned int &out) {
  out = 0;
  for (int i = 0; i < digits; i++) {
    char c = src[offset + i];
    if (c < '0' || c > '9') {
      return false;
    }
    out = out * 10U + (unsigned int)(c - '0');
  }
  return true;
}

static int aprsMicECommentBodyOffset(const unsigned char *body, size_t len) {
  const size_t kMicEFixedLen = 8; // lon(3) + spd/crs(3) + sym(2)
  if (!body || len <= kMicEFixedLen) {
    return (int)kMicEFixedLen;
  }

  size_t idx = kMicEFixedLen;
  unsigned char marker = body[idx];

  // Optional Mic-E telemetry may start with '`', '\'', 0x1C or 0x1D.
  if (marker == 0x60 || marker == 0x27 || marker == 0x1C || marker == 0x1D) {
    size_t probe = idx + 1;
    size_t telemetryLen = 0;
    while (probe < len && telemetryLen < 10) {
      unsigned char c = body[probe];
      if (c < 0x21 || c > 0x7B) {
        break;
      }
      telemetryLen++;
      probe++;
    }

    // Skip telemetry only when it looks like a valid telemetry block.
    if (telemetryLen >= 2) {
      idx = probe;
    }
  }

  return (int)idx;
}

static bool aprsLatLonInRange(float lat, float lon) {
  if (isnan(lat) || isnan(lon)) return false;
  if (lat < -90.0f || lat > 90.0f) return false;
  if (lon < -180.0f || lon > 180.0f) return false;
  // APRS parser conventions: 0,0 is treated as invalid/no position.
  if (fabsf(lat) < 0.0001f && fabsf(lon) < 0.0001f) return false;
  return true;
}

static String normalizeAprsParsedComment(const String &rawComment) {
  String out;
  out.reserve(rawComment.length());
  for (size_t i = 0; i < rawComment.length(); i++) {
    char c = rawComment.charAt(i);
    if (c == '\r' || c == '\n') {
      out += ' ';
      continue;
    }
    if ((unsigned char)c >= 32 || c == '\t') {
      out += c;
    }
  }
  out.trim();
  return out;
}

static bool parseAprsUncompressed(const char *body, size_t len, float &lat, float &lon, char &symTable, char &symCode) {
    if (len < 19) {
        return false;
    }
    char posbuf[20];
    memcpy(posbuf, body, 19);
    posbuf[19] = 0;

    // Position ambiguity handling
    if (posbuf[2] == ' ') posbuf[2] = '3';
    if (posbuf[3] == ' ') posbuf[3] = '5';
    if (posbuf[5] == ' ') posbuf[5] = '5';
    if (posbuf[6] == ' ') posbuf[6] = '5';
    if (posbuf[12] == ' ') posbuf[12] = '3';
    if (posbuf[13] == ' ') posbuf[13] = '5';
    if (posbuf[15] == ' ') posbuf[15] = '5';
    if (posbuf[16] == ' ') posbuf[16] = '5';

    if (posbuf[4] != '.' || posbuf[14] != '.') {
      return false;
    }

    unsigned int latDeg = 0, latMin = 0, latMinFrag = 0;
    unsigned int lonDeg = 0, lonMin = 0, lonMinFrag = 0;
    if (!parseFixedUnsigned(posbuf, 0, 2, latDeg) ||
      !parseFixedUnsigned(posbuf, 2, 2, latMin) ||
      !parseFixedUnsigned(posbuf, 5, 2, latMinFrag) ||
      !parseFixedUnsigned(posbuf, 9, 3, lonDeg) ||
      !parseFixedUnsigned(posbuf, 12, 2, lonMin) ||
      !parseFixedUnsigned(posbuf, 15, 2, lonMinFrag)) {
        return false;
    }

    char latHemi = posbuf[7];
    symTable = posbuf[8];
    char lonHemi = posbuf[17];
    symCode = posbuf[18];

    if (!validSymTableUncompressed(symTable)) {
        symTable = 0;
    }

    bool isSouth = (latHemi == 'S' || latHemi == 's');
    bool isWest = (lonHemi == 'W' || lonHemi == 'w');
    if (!isSouth && latHemi != 'N' && latHemi != 'n') return false;
    if (!isWest && lonHemi != 'E' && lonHemi != 'e') return false;
    if (latDeg > 90 || lonDeg > 180) return false;
    if (latMin >= 60 || lonMin >= 60) return false;

    lat = (float)latDeg + (float)latMin / 60.0f + (float)latMinFrag / 6000.0f;
    lon = (float)lonDeg + (float)lonMin / 60.0f + (float)lonMinFrag / 6000.0f;
    if (isSouth) lat = -lat;
    if (isWest) lon = -lon;
    return aprsLatLonInRange(lat, lon);
}

static bool parseAprsCompressed(const char *body, size_t len, float &lat, float &lon, char &symTable, char &symCode) {
    if (len < 13) {
        return false;
    }
    symTable = body[0];
    symCode = body[9];
    if (!validSymTableCompressed(symTable)) return false;
    for (int i = 1; i <= 8; i++) {
        if (body[i] < 0x21 || body[i] > 0x7b) return false;
    }

    int lat1 = (body[1] - 33);
    int lat2 = (body[2] - 33);
    int lat3 = (body[3] - 33);
    int lat4 = (body[4] - 33);
    lat1 = ((((lat1 * 91) + lat2) * 91) + lat3) * 91 + lat4;

    int lon1 = (body[5] - 33);
    int lon2 = (body[6] - 33);
    int lon3 = (body[7] - 33);
    int lon4 = (body[8] - 33);
    lon1 = ((((lon1 * 91) + lon2) * 91) + lon3) * 91 + lon4;

    lat = 90.0f - ((float)(lat1) / 380926.0f);
    lon = -180.0f + ((float)(lon1) / 190463.0f);

    if (symTable >= 'a' && symTable <= 'j') {
        symTable -= 81; // a-j -> 0-9
    }
    return aprsLatLonInRange(lat, lon);
}

static bool parseAprsMicE(const char *dstcall, const unsigned char *body, size_t len,
                          float &lat, float &lon, char &symTable, char &symCode) {
    if (!dstcall || strlen(dstcall) != 6) return false;
    if (len < 8) return false;

    // Validate destination callsign format
    for (int i = 0; i < 3; i++) {
        if (!((dstcall[i] >= '0' && dstcall[i] <= '9') ||
              (dstcall[i] >= 'A' && dstcall[i] <= 'L') ||
              (dstcall[i] >= 'P' && dstcall[i] <= 'Z'))) {
            return false;
        }
    }
    for (int i = 3; i < 6; i++) {
        if (!((dstcall[i] >= '0' && dstcall[i] <= '9') ||
              (dstcall[i] == 'L') ||
              (dstcall[i] >= 'P' && dstcall[i] <= 'Z'))) {
            return false;
        }
    }

    // Validate info field
    if (body[0] < 0x26 || body[0] > 0x7f) return false;
    if (body[1] < 0x26 || body[1] > 0x61) return false;
    if (body[2] < 0x1c || body[2] > 0x7f) return false;
    if (body[3] < 0x1c || body[3] > 0x7f) return false;
    if (body[4] < 0x1c || body[4] > 0x7d) return false;
    if (body[5] < 0x1c || body[5] > 0x7f) return false;
    if ((body[6] < 0x21 || body[6] > 0x7b) && body[6] != 0x7d) return false;
    if (body[7] != '/' && body[7] != '\\') return false;

    char dst[7];
    strncpy(dst, dstcall, 6);
    dst[6] = 0;

    for (char *p = dst; *p; p++) {
        if (*p >= 'A' && *p <= 'J') *p -= 'A' - '0';
        else if (*p >= 'P' && *p <= 'Y') *p -= 'P' - '0';
        else if (*p == 'K' || *p == 'L' || *p == 'Z') *p = '_';
    }

    int posAmbiguity = 0;
    if (dst[5] == '_') { dst[5] = '5'; posAmbiguity = 1; }
    if (dst[4] == '_') { dst[4] = '5'; posAmbiguity = 2; }
    if (dst[3] == '_') { dst[3] = '5'; posAmbiguity = 3; }
    if (dst[2] == '_') { dst[2] = '3'; posAmbiguity = 4; }
    if (dst[1] == '_' || dst[0] == '_') return false;

    unsigned int latDeg = 0, latMin = 0, latMinFrag = 0;
    if (!parseFixedUnsigned(dst, 0, 2, latDeg) ||
      !parseFixedUnsigned(dst, 2, 2, latMin) ||
      !parseFixedUnsigned(dst, 4, 2, latMinFrag)) {
      return false;
    }
    lat = (float)latDeg + (float)latMin / 60.0f + (float)latMinFrag / 6000.0f;
    if (dstcall[3] <= 0x4c) lat = -lat;

    unsigned int lonDeg = body[0] - 28;
    if (dstcall[4] >= 0x50) lonDeg += 100;
    if (lonDeg >= 180 && lonDeg <= 189) lonDeg -= 80;
    else if (lonDeg >= 190 && lonDeg <= 199) lonDeg -= 190;

    unsigned int lonMin = body[1] - 28;
    if (lonMin >= 60) lonMin -= 60;
    unsigned int lonMinFrag = body[2] - 28;

    switch (posAmbiguity) {
        case 0:
            lon = (float)lonDeg + (float)lonMin / 60.0f + (float)lonMinFrag / 6000.0f;
            break;
        case 1:
            lon = (float)lonDeg + (float)lonMin / 60.0f + (float)(lonMinFrag - lonMinFrag % 10 + 5) / 6000.0f;
            break;
        case 2:
            lon = (float)lonDeg + ((float)lonMin + 0.5f) / 60.0f;
            break;
        case 3:
            lon = (float)lonDeg + (float)(lonMin - lonMin % 10 + 5) / 60.0f;
            break;
        case 4:
            lon = (float)lonDeg + 0.5f;
            break;
        default:
            return false;
    }

    if (dstcall[5] >= 0x50) lon = -lon;

    symCode = body[6];
    symTable = body[7];
    return aprsLatLonInRange(lat, lon);
}

static bool parseAprsAdvancedPosition(const String &line, APRSStation &station) {
    int gtPos = line.indexOf('>');
    if (gtPos < 0) return false;
    int colonPos = line.indexOf(':', gtPos + 1);
    if (colonPos < 0) return false;

    String dstPath = line.substring(gtPos + 1, colonPos);
    int commaPos = dstPath.indexOf(',');
    String dstcall = (commaPos >= 0) ? dstPath.substring(0, commaPos) : dstPath;
    dstcall.trim();
    if (dstcall.length() == 0) return false;

    String payload = line.substring(colonPos + 1);
    if (payload.length() == 0) return false;

    char packettype = payload.charAt(0);
    const char *body = payload.c_str() + 1;
    size_t bodyLen = payload.length() > 0 ? (size_t)(payload.length() - 1) : 0;

    float lat = 0.0f, lon = 0.0f;
    char symTable = 0, symCode = 0;
    bool ok = false;
    int commentOffset = -1; // offset in payload
    String objectOrItemName = "";

    if (packettype == '`' || packettype == '\'') {
        ok = parseAprsMicE(dstcall.c_str(), (const unsigned char *)body, bodyLen, lat, lon, symTable, symCode);
      if (ok) commentOffset = 1 + aprsMicECommentBodyOffset((const unsigned char *)body, bodyLen);
    } else if (packettype == '!' || packettype == '=' || packettype == '/' || packettype == '@') {
        size_t offset = 1;
        if ((packettype == '/' || packettype == '@') && bodyLen >= 7) {
            body += 7;
            bodyLen -= 7;
            offset += 7;
        }
        if (bodyLen > 0) {
            char poschar = body[0];
            if (validSymTableCompressed(poschar)) {
                ok = parseAprsCompressed(body, bodyLen, lat, lon, symTable, symCode);
                if (ok) commentOffset = (int)(offset + 13);
            } else if (poschar >= '0' && poschar <= '9') {
                ok = parseAprsUncompressed(body, bodyLen, lat, lon, symTable, symCode);
                if (ok) commentOffset = (int)(offset + 19);
            }
        }
    } else if (packettype == ';') {
      // Object: name + (* or _) + timestamp(7) + position
      int nameEnd = -1;
      for (int i = 0; i < (int)bodyLen && i < 15; i++) {
        if (body[i] == '*' || body[i] == '_') {
          nameEnd = i;
          break;
        }
      }
      if (nameEnd >= 0) {
        objectOrItemName = payload.substring(1, 1 + nameEnd);
        objectOrItemName.trim();
      }
      if (nameEnd >= 0 && (nameEnd + 8) < (int)bodyLen) {
        const char *posBody = body + nameEnd + 8;
        size_t posLen = bodyLen - (nameEnd + 8);
            char poschar = posBody[0];
            if (validSymTableCompressed(poschar)) {
                ok = parseAprsCompressed(posBody, posLen, lat, lon, symTable, symCode);
          if (ok) commentOffset = 1 + nameEnd + 8 + 13;
            } else if (poschar >= '0' && poschar <= '9') {
                ok = parseAprsUncompressed(posBody, posLen, lat, lon, symTable, symCode);
          if (ok) commentOffset = 1 + nameEnd + 8 + 19;
            }
        }
    } else if (packettype == ')') {
        // Item: name ends with ! or _
        int nameEnd = -1;
        for (int i = 0; i < 9 && i < (int)bodyLen; i++) {
            if (body[i] == '!' || body[i] == '_') {
                nameEnd = i;
                break;
            }
        }
        if (nameEnd >= 0 && (nameEnd + 1) < (int)bodyLen) {
        objectOrItemName = payload.substring(1, 1 + nameEnd);
        objectOrItemName.trim();
            const char *posBody = body + nameEnd + 1;
            size_t posLen = bodyLen - (nameEnd + 1);
            char poschar = posBody[0];
            if (validSymTableCompressed(poschar)) {
                ok = parseAprsCompressed(posBody, posLen, lat, lon, symTable, symCode);
                if (ok) commentOffset = 1 + nameEnd + 1 + 13;
            } else if (poschar >= '0' && poschar <= '9') {
                ok = parseAprsUncompressed(posBody, posLen, lat, lon, symTable, symCode);
                if (ok) commentOffset = 1 + nameEnd + 1 + 19;
            }
        }
    }

    if (!ok) {
        return false;
    }

    station.lat = lat;
    station.lon = lon;
    station.hasLatLon = aprsLatLonInRange(lat, lon);
    if (!station.hasLatLon) {
      return false;
    }
    if (symCode != 0) {
        station.symbol = String(symCode);
    }
    if (symTable != 0) {
        station.symbolTable = String(symTable);
    }

    if (objectOrItemName.length() > 0) {
      // Dla raportów Object/Item użyj nazwy obiektu jako identyfikatora stacji.
      station.callsign = objectOrItemName;
    }

    if (commentOffset > 0 && commentOffset < payload.length()) {
      station.comment = normalizeAprsParsedComment(payload.substring(commentOffset));
    }
    return true;
}

// Obliczanie odlegÄąâ€šoÄąâ€şci metodĂ„â€¦ Haversine (km)
// UÄąÄ˝ywa double dla wiĂ„â„˘kszej precyzji obliczeÄąâ€ž
// UÄąÄ˝ywa promienia Ziemi 6366.71 km (jak w ESP32APRS_Audio) dla zgodnoÄąâ€şci z aprs.fi
float calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double r = 6366.71; // PromieÄąâ€ž Ziemi w km (uÄąÄ˝ywany przez ESP32APRS_Audio i aprs.fi)
    const double dLat = (lat2 - lat1) * M_PI / 180.0;
    const double dLon = (lon2 - lon1) * M_PI / 180.0;

    const double a = sin(dLat/2.0) * sin(dLat/2.0) +
                     cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
                     sin(dLon/2.0) * sin(dLon/2.0);
    const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    
    return (float)(r * c); // Wynik w km
}

// Oblicz azymut początkowy (0..360), gdzie 0 = Północ
float calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    const double phi1 = lat1 * M_PI / 180.0;
    const double phi2 = lat2 * M_PI / 180.0;
    const double dLon = (lon2 - lon1) * M_PI / 180.0;

    const double y = sin(dLon) * cos(phi2);
    const double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dLon);
    double bearingDeg = atan2(y, x) * 180.0 / M_PI;
    if (bearingDeg < 0.0) {
      bearingDeg += 360.0;
    }
    return (float)bearingDeg;
}

String formatDistanceOrCountry(const DXSpot &spot, size_t maxLen) {
  if (spot.country.length() > 0) {
    String c = spot.country;
    c.trim();
    if (c.length() > maxLen) {
      c = c.substring(0, maxLen);
    }
    return c;
  }
  return "-";
}

// OkreÄąâ€şlanie pasma na podstawie czĂ„â„˘stotliwoÄąâ€şci (kHz) wg bandplanu
String getBand(float freq) {
  // Normalize to kHz so both kHz (DX cluster) and MHz (POTA API) inputs map correctly
  float freqKHz = freq;
  if (freqKHz > 0.0f && freqKHz < 1000.0f) {
    freqKHz *= 1000.0f;
  }

  if (freqKHz >= 1810 && freqKHz <= 2000) return "240m";
  if (freqKHz >= 3500 && freqKHz <= 3800) return "80m";
  if (freqKHz >= 7000 && freqKHz <= 7200) return "40m";
  if (freqKHz >= 10100 && freqKHz <= 10150) return "30m";
  if (freqKHz >= 14000 && freqKHz <= 14350) return "20m";
  if (freqKHz >= 18068 && freqKHz <= 18168) return "17m";
  if (freqKHz >= 21000 && freqKHz <= 21450) return "15m";
  if (freqKHz >= 24890 && freqKHz <= 24990) return "12m";
  if (freqKHz >= 28000 && freqKHz <= 29700) return "10m";
  return "Other";
}

// OkreÄąâ€şlanie modulacji na podstawie komentarza
String getMode(String comment) {
  comment.toUpperCase();
  if (comment.indexOf("CW") >= 0) return "CW";
  if (comment.indexOf("FT4") >= 0) return "FT4";
  if (comment.indexOf("FT8") >= 0) return "FT8";
  if (comment.indexOf("SSB") >= 0 || comment.indexOf("USB") >= 0 || comment.indexOf("LSB") >= 0) return "SSB";
  return "SSB"; // DomyÄąâ€şlnie SSB
}

String extractXmlTagValue(const String &xml, const String &tag) {
  String openTag = "<" + tag + ">";
  String closeTag = "</" + tag + ">";
  int start = xml.indexOf(openTag);
  if (start < 0) return "";
  start += openTag.length();
  int end = xml.indexOf(closeTag, start);
  if (end < 0) return "";
  return xml.substring(start, end);
}

// Funkcja do URL-encodowania stringów (używana przy QRZ)
static String urlEncode(const String& value) {
  String encoded = "";
  char c;
  for (size_t i = 0; i < value.length(); i++) {
    c = value.charAt(i);
    if (c == ' ') {
      encoded += "%20";
    } else if (c == '!') {
      encoded += "%21";
    } else if (c == '"') {
      encoded += "%22";
    } else if (c == '#') {
      encoded += "%23";
    } else if (c == '$') {
      encoded += "%24";
    } else if (c == '%') {
      encoded += "%25";
    } else if (c == '&') {
      encoded += "%26";
    } else if (c == '\'') {
      encoded += "%27";
    } else if (c == '(') {
      encoded += "%28";
    } else if (c == ')') {
      encoded += "%29";
    } else if (c == '*') {
      encoded += "%2A";
    } else if (c == '+') {
      encoded += "%2B";
    } else if (c == ',') {
      encoded += "%2C";
    } else if (c == '/') {
      encoded += "%2F";
    } else if (c == ':') {
      encoded += "%3A";
    } else if (c == ';') {
      encoded += "%3B";
    } else if (c == '<') {
      encoded += "%3C";
    } else if (c == '=') {
      encoded += "%3D";
    } else if (c == '>') {
      encoded += "%3E";
    } else if (c == '?') {
      encoded += "%3F";
    } else if (c == '@') {
      encoded += "%40";
    } else if (c == '[') {
      encoded += "%5B";
    } else if (c == '\\') {
      encoded += "%5C";
    } else if (c == ']') {
      encoded += "%5D";
    } else if (c == '^') {
      encoded += "%5E";
    } else if (c == '_') {
      encoded += "%5F";
    } else if (c == '`') {
      encoded += "%60";
    } else if (c == '{') {
      encoded += "%7B";
    } else if (c == '|') {
      encoded += "%7C";
    } else if (c == '}') {
      encoded += "%7D";
    } else if (c == '.') {
      encoded += "%2E";
    } else if (c == '-') {
      encoded += "%2D";
    } else if (c < '0' || (c > '9' && c < 'A') || (c > 'Z' && c < 'a') || (c > 'z' && c < 127)) {
      // Kodowanie wszystkich innych znaków specjalnych i znaków > 127 (UTF-8)
      char hex[4];
      snprintf(hex, sizeof(hex), "%%%02X", (unsigned char)c);
      encoded += hex;
    } else {
      encoded += c;
    }
  }
  return encoded;
}

bool ensureQrzSession(String &sessionKey) {
  static String cachedKey = "";
  static unsigned long cachedAt = 0;
  const unsigned long SESSION_TTL_MS = 30UL * 60UL * 1000UL; // 30 minut

  if (qrzUsername.length() == 0 || qrzPassword.length() == 0) {
    qrzStatus = "QRZ: no credentials";
    return false;
  }

  unsigned long now = millis();
  if (cachedKey.length() > 0 && (now - cachedAt) < SESSION_TTL_MS) {
    sessionKey = cachedKey;
    qrzStatus = "QRZ: session ok";
    return true;
  }

  String url = "https://xmldata.qrz.com/xml/current/?username=" + urlEncode(qrzUsername) +
               ";password=" + urlEncode(qrzPassword);
  
  // Debug log - pokaż URL z zamaskowanym hasłem
  String debugUrl = "https://xmldata.qrz.com/xml/current/?username=" + urlEncode(qrzUsername) +
                    ";password=*** (len=" + String(qrzPassword.length()) + ")";
  Serial.println("[QRZ DEBUG] URL: " + debugUrl);
  Serial.println("[QRZ DEBUG] Username: " + qrzUsername + " (len=" + String(qrzUsername.length()) + ")");
  Serial.println("[QRZ DEBUG] Pass len: " + String(qrzPassword.length()));
  
  // Pokaż pierwsze i ostatnie 3 znaki zakodowanego hasła (dla weryfikacji)
  String encodedPass = urlEncode(qrzPassword);
  String passPreview = encodedPass.substring(0, 6) + "..." + encodedPass.substring(encodedPass.length()-6);
  Serial.println("[QRZ DEBUG] Encoded pass preview: " + passPreview + " (total len=" + String(encodedPass.length()) + ")");
  
  HTTPClient http;
  http.setTimeout(3000);
  http.begin(url);
  int code = http.GET();
  if (code != 200) {
    http.end();
    qrzStatus = "QRZ: login http " + String(code);
    return false;
  }
  String body = http.getString();
  http.end();

  String err = extractXmlTagValue(body, "Error");
  if (err.length() == 0) {
    err = extractXmlTagValue(body, "error");
  }

  String key = extractXmlTagValue(body, "Key");
  if (key.length() == 0) {
    key = extractXmlTagValue(body, "key");
  }
  if (key.length() == 0) {
    if (err.length() > 0) {
      qrzStatus = "QRZ: login " + err;
      Serial.print("[QRZ] Login error: ");
      Serial.println(err);
    } else {
      qrzStatus = "QRZ: login failed";
      Serial.println("[QRZ] Login failed - no session key and no error message");
      Serial.println("[QRZ] Response body (first 500 chars):");
      Serial.println(body.substring(0, 500));
    }
    return false;
  }

  cachedKey = key;
  cachedAt = now;
  sessionKey = key;
  qrzStatus = "QRZ: login ok";
  Serial.println("[QRZ] Session OK");
  return true;
}

// ========== CALLOOK.INFO API (Free alternative to QRZ) ==========
bool fetchCallookCallsignInfo(const String &callsign, String &outGrid, String &outCountry,
                               String &outName, String &outEmail, String &outQth, double &outLat, double &outLon, bool &outHasLatLon) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    return false;
  }

  outGrid = "";
  outCountry = "";
  outName = "";
  outEmail = "";
  outQth = "";
  outLat = 0.0;
  outLon = 0.0;
  outHasLatLon = false;

  String call = callsign;
  call.toUpperCase();
  call.trim();
  if (call.length() == 0) return false;

  // Check cache first
  unsigned long now = millis();
  for (int i = 0; i < QRZ_CACHE_SIZE; i++) {
    if (qrzCache[i].callsign == call && (now - qrzCache[i].fetchedAtMs) < QRZ_CACHE_TTL_MS) {
      outGrid = qrzCache[i].grid;
      outCountry = qrzCache[i].country;
      outName = qrzCache[i].name;
      outEmail = qrzCache[i].email;
      outQth = qrzCache[i].qth;
      outLat = qrzCache[i].lat;
      outLon = qrzCache[i].lon;
      outHasLatLon = qrzCache[i].hasLatLon;
      return (outGrid.length() > 0 || outCountry.length() > 0 || outName.length() > 0);
    }
  }

  String url = "https://callook.info/" + call + "/json";
  HTTPClient http;
  http.setTimeout(5000);
  http.begin(url);
  int code = http.GET();
  if (code != 200) {
    http.end();
    Serial.print("[CALLOOK] HTTP error: ");
    Serial.println(code);
    return false;
  }
  String body = http.getString();
  http.end();

  // Parse JSON response
  DynamicJsonDocument doc(1024);
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    Serial.print("[CALLOOK] JSON parse error: ");
    Serial.println(err.c_str());
    return false;
  }

  // Check if callsign exists
  if (doc["status"] == "INVALID") {
    Serial.println("[CALLOOK] Callsign not found");
    return false;
  }

  // Extract data
  const char* name = doc["name"];
  if (name) outName = String(name);
  
  const char* grid = doc["location"]["gridsquare"];
  if (grid) outGrid = String(grid);
  
  const char* country = doc["address"]["country"];
  if (country) outCountry = String(country);
  
  // Build QTH from address line2 (format: "CITY, STATE ZIP")
  const char* addressLine2 = doc["address"]["line2"];
  if (addressLine2) {
    String addr = String(addressLine2);
    // Extract city and state from "CITY, STATE ZIP" format
    int commaPos = addr.indexOf(',');
    if (commaPos > 0) {
      String city = addr.substring(0, commaPos);
      String stateZip = addr.substring(commaPos + 1);
      stateZip.trim();
      // Extract just state (2 letters before space or zip)
      int spacePos = stateZip.indexOf(' ');
      String state = (spacePos > 0) ? stateZip.substring(0, spacePos) : stateZip;
      outQth = city + ", " + state;
    } else {
      outQth = addr;
    }
  }

  // Get coordinates
  float lat = doc["location"]["latitude"] | 0.0f;
  float lon = doc["location"]["longitude"] | 0.0f;
  if (lat != 0.0f && lon != 0.0f) {
    outLat = lat;
    outLon = lon;
    outHasLatLon = true;
  }

  // Save to cache
  int oldestIdx = 0;
  unsigned long oldestMs = qrzCache[0].fetchedAtMs;
  for (int i = 1; i < QRZ_CACHE_SIZE; i++) {
    if (qrzCache[i].fetchedAtMs < oldestMs) {
      oldestMs = qrzCache[i].fetchedAtMs;
      oldestIdx = i;
    }
  }
  qrzCache[oldestIdx].callsign = call;
  qrzCache[oldestIdx].grid = outGrid;
  qrzCache[oldestIdx].country = outCountry;
  qrzCache[oldestIdx].name = outName;
  qrzCache[oldestIdx].email = outEmail;
  qrzCache[oldestIdx].qth = outQth;
  qrzCache[oldestIdx].lat = (float)outLat;
  qrzCache[oldestIdx].lon = (float)outLon;
  qrzCache[oldestIdx].hasLatLon = outHasLatLon;
  qrzCache[oldestIdx].fetchedAtMs = now;

  Serial.print("[CALLOOK] Fetched: ");
  Serial.print(call);
  Serial.print(" - Name: '");
  Serial.print(outName);
  Serial.print("', Grid: '");
  Serial.print(outGrid);
  Serial.print("', QTH: '");
  Serial.print(outQth);
  Serial.println("'");

  return (outGrid.length() > 0 || outCountry.length() > 0 || outName.length() > 0);
}

// ========== HAMQTH.COM API (Free, global) ==========
// Dodaj mapowanie prefiksu na przybliżony lokalizator
String getApproximateGridFromCallsign(const String &callsign) {
  if (callsign.length() < 2) return "";
  
  // Mapowanie prefiksów na przybliżone lokatory (centrum kraju)
  String prefix = callsign.substring(0, 2);
  prefix.toUpperCase();
  
  // Europa
  if (callsign.startsWith("SP")) return "JO92ii";      // Poland
  if (callsign.startsWith("SQ")) return "JO92ii";      // Poland
  if (callsign.startsWith("SO")) return "JO92ii";    // Poland
  if (callsign.startsWith("HF")) return "JO92ii";    // Poland
  if (callsign.startsWith("SR")) return "JO92ii";    // Poland
  if (callsign.startsWith("SN")) return "JO92ii";    // Poland
  if (callsign.startsWith("3Z")) return "JO92ii";    // Poland
  if (callsign.startsWith("DL")) return "JO62qm";    // Germany
  if (callsign.startsWith("DO")) return "JO62qm";    // Germany
  if (callsign.startsWith("DA")) return "JO62qm";    // Germany
  if (callsign.startsWith("G"))  return "IO91vl";    // England
  if (callsign.startsWith("M"))  return "IO91vl";    // England
  if (callsign.startsWith("2E")) return "IO91vl";    // England
  if (callsign.startsWith("GM")) return "IO86je";    // Scotland
  if (callsign.startsWith("GW")) return "IO81fm";    // Wales
  if (callsign.startsWith("F"))  return "JN18dv";    // France
  if (callsign.startsWith("TM")) return "JN18dv";    // France
  if (callsign.startsWith("ON")) return "JO20fm";    // Belgium
  if (callsign.startsWith("PA")) return "JO22db";    // Netherlands
  if (callsign.startsWith("PB")) return "JO22db";    // Netherlands
  if (callsign.startsWith("PH")) return "JO22db";    // Netherlands
  if (callsign.startsWith("PI")) return "JO22db";    // Netherlands
  if (callsign.startsWith("HB")) return "JN46bt";    // Switzerland
  if (callsign.startsWith("HE")) return "JN46bt";    // Switzerland
  if (callsign.startsWith("OE")) return "JN77tx";    // Austria
  if (callsign.startsWith("OK")) return "JN79jw";    // Czech Republic
  if (callsign.startsWith("OL")) return "JN79jw";    // Czech Republic
  if (callsign.startsWith("OM")) return "JN88ss";    // Slovakia
  if (callsign.startsWith("HA")) return "JN97nm";    // Hungary
  if (callsign.startsWith("SV")) return "KM17uw";    // Greece
  if (callsign.startsWith("5B")) return "KM64ft";    // Cyprus
  if (callsign.startsWith("C3")) return "JN02tw";    // Andorra
  if (callsign.startsWith("CT")) return "IM57nh";    // Portugal
  if (callsign.startsWith("CU")) return "IM57nh";    // Portugal
  if (callsign.startsWith("EA")) return "IN80dj";    // Spain
  if (callsign.startsWith("EB")) return "IN80dj";    // Spain
  if (callsign.startsWith("EC")) return "IN80dj";    // Spain
  if (callsign.startsWith("ED")) return "IN80dj";    // Spain
  if (callsign.startsWith("EI")) return "IO63ci";    // Ireland
  if (callsign.startsWith("EJ")) return "IO63ci";    // Ireland
  if (callsign.startsWith("EK")) return "LM28cx";    // Armenia
  if (callsign.startsWith("EL")) return "JJ06px";    // Liberia
  if (callsign.startsWith("ER")) return "KN37bs";    // Moldova
  if (callsign.startsWith("ES")) return "KO29ih";    // Estonia
  if (callsign.startsWith("ET")) return "JM55bt";    // Ethiopia
  if (callsign.startsWith("EU")) return "JO34qd";    // Belarus
  if (callsign.startsWith("EV")) return "KO33tw";    // Latvia
  if (callsign.startsWith("EW")) return "KO43lb";    // Lithuania
  if (callsign.startsWith("EX")) return "MN52ct";    // Kyrgyzstan
  if (callsign.startsWith("EY")) return "MM47nx";    // Tajikistan
  if (callsign.startsWith("EZ")) return "LM49qp";    // Turkmenistan
  if (callsign.startsWith("I"))  return "JN62ra";    // Italy
  if (callsign.startsWith("IS")) return "JN62ra";    // Italy (Sardinia)
  if (callsign.startsWith("IZ")) return "JN62ra";    // Italy
  if (callsign.startsWith("IT")) return "JN62ra";    // Italy
  if (callsign.startsWith("LA")) return "JO59jp";    // Norway
  if (callsign.startsWith("LB")) return "JO59jp";    // Norway
  if (callsign.startsWith("LC")) return "JO59jp";    // Norway
  if (callsign.startsWith("LD")) return "JO59jp";    // Norway
  if (callsign.startsWith("LE")) return "JO59jp";    // Norway
  if (callsign.startsWith("LF")) return "JO59jp";    // Norway
  if (callsign.startsWith("LG")) return "JO59jp";    // Norway
  if (callsign.startsWith("LH")) return "JO59jp";    // Norway
  if (callsign.startsWith("LI")) return "JO59jp";    // Norway
  if (callsign.startsWith("LJ")) return "JO59jp";    // Norway
  if (callsign.startsWith("LK")) return "JO59jp";    // Norway
  if (callsign.startsWith("LL")) return "JO59jp";    // Norway
  if (callsign.startsWith("LM")) return "JO59jp";    // Norway
  if (callsign.startsWith("LN")) return "JO59jp";    // Norway
  if (callsign.startsWith("LZ")) return "KN12pr";    // Bulgaria
  if (callsign.startsWith("S5")) return "JN75cv";    // Slovenia
  if (callsign.startsWith("S7")) return "LL38on";    // Seychelles
  if (callsign.startsWith("S9")) return "JI12hf";    // Sao Tome
  if (callsign.startsWith("SA")) return "JP80jq";    // Sweden
  if (callsign.startsWith("SB")) return "JP80jq";    // Sweden
  if (callsign.startsWith("SC")) return "JP80jq";    // Sweden
  if (callsign.startsWith("SD")) return "JP80jq";    // Sweden
  if (callsign.startsWith("SE")) return "JP80jq";    // Sweden
  if (callsign.startsWith("SF")) return "JP80jq";    // Sweden
  if (callsign.startsWith("SG")) return "JP80jq";    // Sweden
  if (callsign.startsWith("SH")) return "JP80jq";    // Sweden
  if (callsign.startsWith("SI")) return "JP80jq";    // Sweden
  if (callsign.startsWith("SJ")) return "JP80jq";    // Sweden
  if (callsign.startsWith("SK")) return "JP80jq";    // Sweden
  if (callsign.startsWith("SL")) return "JP80jq";    // Sweden
  if (callsign.startsWith("SM")) return "JP80jq";    // Sweden
  if (callsign.startsWith("SN")) return "JO92ii";    // Poland
  if (callsign.startsWith("TA")) return "KM69kw";    // Turkey
  if (callsign.startsWith("TB")) return "KM69kw";    // Turkey
  if (callsign.startsWith("TC")) return "KM69kw";    // Turkey
  if (callsign.startsWith("TD")) return "KM69kw";    // Turkey
  if (callsign.startsWith("TF")) return "HP94mf";    // Iceland
  if (callsign.startsWith("TG")) return "EK44qm";    // Guatemala
  if (callsign.startsWith("TI")) return "EK70wf";    // Costa Rica
  if (callsign.startsWith("TJ")) return "JJ53rm";    // Cameroon
  if (callsign.startsWith("TK")) return "JN43ji";    // Corsica
  if (callsign.startsWith("TL")) return "JJ48qx";    // Central Africa
  if (callsign.startsWith("TN")) return "JN53dl";    // Congo
  if (callsign.startsWith("TR")) return "JJ47nm";    // Gabon
  if (callsign.startsWith("TT")) return "JJ66af";    // Chad
  if (callsign.startsWith("TU")) return "IJ85ml";    // Ivory Coast
  if (callsign.startsWith("TY")) return "JJ16aa";    // Benin
  if (callsign.startsWith("TZ")) return "IJ52ei";    // Mali
  if (callsign.startsWith("UA")) return "KO85ss";    // Russia (European)
  if (callsign.startsWith("UB")) return "KO85ss";    // Russia
  if (callsign.startsWith("UC")) return "KO85ss";    // Russia
  if (callsign.startsWith("UD")) return "KO85ss";    // Russia
  if (callsign.startsWith("UE")) return "KO85ss";    // Russia
  if (callsign.startsWith("UF")) return "KO85ss";    // Russia
  if (callsign.startsWith("UG")) return "KO85ss";    // Russia
  if (callsign.startsWith("UH")) return "KO85ss";    // Russia
  if (callsign.startsWith("UI")) return "KO85ss";    // Russia
  if (callsign.startsWith("UJ")) return "KO85ss";    // Russia
  if (callsign.startsWith("UK")) return "KO85ss";    // Russia
  if (callsign.startsWith("UL")) return "KO85ss";    // Russia
  if (callsign.startsWith("UM")) return "KO85ss";    // Russia
  if (callsign.startsWith("UN")) return "KO85ss";    // Russia
  if (callsign.startsWith("UO")) return "KO85ss";    // Russia
  if (callsign.startsWith("UP")) return "KO85ss";    // Russia
  if (callsign.startsWith("UQ")) return "KO85ss";    // Russia
  if (callsign.startsWith("UR")) return "KO50dm";    // Ukraine
  if (callsign.startsWith("US")) return "KO50dm";    // Ukraine
  if (callsign.startsWith("UT")) return "KO50dm";    // Ukraine
  if (callsign.startsWith("UU")) return "KO50dm";    // Ukraine
  if (callsign.startsWith("UV")) return "KO50dm";    // Ukraine
  if (callsign.startsWith("UW")) return "KO50dm";    // Ukraine
  if (callsign.startsWith("UX")) return "KO50dm";    // Ukraine
  if (callsign.startsWith("UY")) return "KO50dm";    // Ukraine
  if (callsign.startsWith("UZ")) return "KO50dm";    // Ukraine
  if (callsign.startsWith("YU")) return "JN94sd";    // Serbia
  if (callsign.startsWith("YT")) return "JN94sd";    // Serbia
  if (callsign.startsWith("YZ")) return "KN05fx";    // Macedonia
  if (callsign.startsWith("ZA")) return "JM99ah";    // Albania
  if (callsign.startsWith("ZB")) return "JM99ah";    // Albania
  if (callsign.startsWith("ZC")) return "JM99ah";    // Albania
  if (callsign.startsWith("ZD")) return "JM99ah";    // Albania
  if (callsign.startsWith("ZE")) return "JM99ah";    // Albania
  if (callsign.startsWith("Y2")) return "FM07vx";    // Germany (V Rhodes)
  if (callsign.startsWith("Y3")) return "FM07vx";    // Germany (V Rhodes)
  if (callsign.startsWith("Y4")) return "FM07vx";    // Germany (V Rhodes)
  if (callsign.startsWith("Y5")) return "FM07vx";    // Germany (V Rhodes)
  
  // USA / Kanada (Callook działa, ale dla pewności)
  if (callsign.startsWith("K"))  return "EM48to";    // USA (rough center)
  if (callsign.startsWith("N"))  return "EM48to";    // USA
  if (callsign.startsWith("W"))  return "EM48to";    // USA
  if (callsign.startsWith("AA")) return "EM48to";    // USA
  if (callsign.startsWith("AB")) return "EM48to";    // USA
  if (callsign.startsWith("AC")) return "EM48to";    // USA
  if (callsign.startsWith("AD")) return "EM48to";    // USA
  if (callsign.startsWith("AE")) return "EM48to";    // USA
  if (callsign.startsWith("AF")) return "EM48to";    // USA
  if (callsign.startsWith("AG")) return "EM48to";    // USA
  if (callsign.startsWith("AH")) return "EM48to";    // USA
  if (callsign.startsWith("AI")) return "EM48to";    // USA
  if (callsign.startsWith("AJ")) return "EM48to";    // USA
  if (callsign.startsWith("AK")) return "EM48to";    // USA
  if (callsign.startsWith("AL")) return "EM48to";    // USA
  if (callsign.startsWith("VE")) return "FN25bh";    // Canada
  if (callsign.startsWith("VA")) return "FN25bh";    // Canada
  if (callsign.startsWith("VY")) return "FN25bh";    // Canada
  if (callsign.startsWith("VO")) return "FN25bh";    // Canada
  
  return ""; // Nieznany prefiks
}

bool fetchHamQthCallsignInfo(const String &callsign, String &outGrid, String &outCountry,
                               String &outName, String &outEmail, String &outQth, double &outLat, double &outLon, bool &outHasLatLon) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    return false;
  }

  outGrid = "";
  outCountry = "";
  outName = "";
  outEmail = "";
  outQth = "";
  outLat = 0.0;
  outLon = 0.0;
  outHasLatLon = false;

  String call = callsign;
  call.toUpperCase();
  call.trim();
  if (call.length() == 0) return false;

  // Check cache first
  unsigned long now = millis();
  for (int i = 0; i < QRZ_CACHE_SIZE; i++) {
    if (qrzCache[i].callsign == call && (now - qrzCache[i].fetchedAtMs) < QRZ_CACHE_TTL_MS) {
      outGrid = qrzCache[i].grid;
      outCountry = qrzCache[i].country;
      outName = qrzCache[i].name;
      outEmail = qrzCache[i].email;
      outQth = qrzCache[i].qth;
      outLat = qrzCache[i].lat;
      outLon = qrzCache[i].lon;
      outHasLatLon = qrzCache[i].hasLatLon;
      return (outGrid.length() > 0 || outCountry.length() > 0 || outName.length() > 0);
    }
  }

  // HamQTH API - no login required for basic info
  String url = "https://www.hamqth.com/dxcc_json.php?callsign=" + call + "&apikey=ESP32HAMCLOCK";
  HTTPClient http;
  http.setTimeout(5000);
  http.begin(url);
  int code = http.GET();
  if (code != 200) {
    http.end();
    Serial.print("[HAMQTH] HTTP error: ");
    Serial.println(code);
    return false;
  }
  String body = http.getString();
  http.end();

  // Parse JSON response
  DynamicJsonDocument doc(1024);
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    Serial.print("[HAMQTH] JSON parse error: ");
    Serial.println(err.c_str());
    return false;
  }

  // Check if callsign found
  const char* error = doc["error"];
  if (error) {
    Serial.print("[HAMQTH] Error: ");
    Serial.println(error);
    return false;
  }

  // Extract data
  const char* name = doc["name"];
  if (name) outName = String(name);
  
  const char* grid = doc["grid"];
  if (grid) outGrid = String(grid);
  
  const char* country = doc["country"];
  if (country) outCountry = String(country);
  
  const char* qth = doc["qth"];
  if (qth) outQth = String(qth);
  
  // Get coordinates from grid if available
  if (outGrid.length() >= 4) {
    locatorToLatLon(outGrid, outLat, outLon);
    outHasLatLon = true;
  } else {
    // Brak grid z API - spróbuj przybliżony z prefiksu
    String approxGrid = getApproximateGridFromCallsign(call);
    if (approxGrid.length() >= 4) {
      outGrid = approxGrid + " (approx)";
      locatorToLatLon(approxGrid, outLat, outLon);
      outHasLatLon = true;
      Serial.print("[HAMQTH] Using approximate grid from prefix: ");
      Serial.println(approxGrid);
    }
  }

  // Save to cache
  int oldestIdx = 0;
  unsigned long oldestMs = qrzCache[0].fetchedAtMs;
  for (int i = 1; i < QRZ_CACHE_SIZE; i++) {
    if (qrzCache[i].fetchedAtMs < oldestMs) {
      oldestMs = qrzCache[i].fetchedAtMs;
      oldestIdx = i;
    }
  }
  qrzCache[oldestIdx].callsign = call;
  qrzCache[oldestIdx].grid = outGrid;
  qrzCache[oldestIdx].country = outCountry;
  qrzCache[oldestIdx].name = outName;
  qrzCache[oldestIdx].email = outEmail;
  qrzCache[oldestIdx].qth = outQth;
  qrzCache[oldestIdx].lat = (float)outLat;
  qrzCache[oldestIdx].lon = (float)outLon;
  qrzCache[oldestIdx].hasLatLon = outHasLatLon;
  qrzCache[oldestIdx].fetchedAtMs = now;

  Serial.print("[HAMQTH] Fetched: ");
  Serial.print(call);
  Serial.print(" - Name: '");
  Serial.print(outName);
  Serial.print("', Grid: '");
  Serial.print(outGrid);
  Serial.print("', Country: '");
  Serial.print(outCountry);
  Serial.println("'");

  return (outGrid.length() > 0 || outCountry.length() > 0 || outName.length() > 0);
}

bool fetchQrzCallsignInfo(const String &callsign, String &outGrid, String &outCountry,
                          String &outName, String &outEmail, String &outQth, double &outLat, double &outLon, bool &outHasLatLon) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    qrzStatus = "QRZ: no wifi";
    return false;
  }

  outGrid = "";
  outCountry = "";
  outName = "";
  outEmail = "";
  outQth = "";
  outLat = 0.0;
  outLon = 0.0;
  outHasLatLon = false;

  String call = callsign;
  call.toUpperCase();
  unsigned long now = millis();
  for (int i = 0; i < QRZ_CACHE_SIZE; i++) {
    if (qrzCache[i].callsign == call && (now - qrzCache[i].fetchedAtMs) < QRZ_CACHE_TTL_MS) {
      outGrid = qrzCache[i].grid;
      outCountry = qrzCache[i].country;
      outName = qrzCache[i].name;
      outEmail = qrzCache[i].email;
      outQth = qrzCache[i].qth;
      outLat = qrzCache[i].lat;
      outLon = qrzCache[i].lon;
      outHasLatLon = qrzCache[i].hasLatLon;
      return (outGrid.length() > 0 || outCountry.length() > 0 || outName.length() > 0);
    }
  }

  String sessionKey;
  if (!ensureQrzSession(sessionKey)) {
    return false;
  }

  String url = "https://xmldata.qrz.com/xml/current/?s=" + sessionKey +
               ";callsign=" + callsign;
  HTTPClient http;
  http.setTimeout(3000);
  http.begin(url);
  int code = http.GET();
  if (code != 200) {
    http.end();
    qrzStatus = "QRZ: lookup http " + String(code);
    return false;
  }
  String body = http.getString();
  http.end();

  // Obsługa błędu sesji - spróbuj raz odświeżyć
  if (body.indexOf("Session Timeout") >= 0 || body.indexOf("Invalid session") >= 0) {
    String dummy;
    ensureQrzSession(dummy);
    qrzStatus = "QRZ: session expired";
    return false;
  }

  String grid = extractXmlTagValue(body, "grid");
  String country = extractXmlTagValue(body, "country");
  String name = extractXmlTagValue(body, "name");
  if (name.length() == 0) {
    name = extractXmlTagValue(body, "Name");  // Capitalized variant
  }
  if (name.length() == 0) {
    name = extractXmlTagValue(body, "fname");  // First name
  }
  if (name.length() == 0) {
    name = extractXmlTagValue(body, "Fname");  // Capitalized first name
  }
  String email = extractXmlTagValue(body, "email");
  if (email.length() == 0) {
    email = extractXmlTagValue(body, "Email");  // Capitalized variant
  }
  String qth = extractXmlTagValue(body, "qth");
  if (qth.length() == 0) {
    qth = extractXmlTagValue(body, "Qth");  // Capitalized variant
    if (qth.length() == 0) {
      qth = extractXmlTagValue(body, "QTH");  // All caps variant
    }
  }
  String latStr = extractXmlTagValue(body, "lat");
  String lonStr = extractXmlTagValue(body, "lon");
  grid.trim();
  country.trim();
  name.trim();
  if (country.length() == 0) {
    country = extractXmlTagValue(body, "dxcc");
    country.trim();
  }

  outGrid = grid;
  outCountry = country;
  outName = name;
  outEmail = email;
  outQth = qth;
  
  // Debug log
  Serial.print("[QRZ DEBUG] Name for ");
  Serial.print(callsign);
  Serial.print(": '");
  Serial.print(name);
  Serial.println("'");
  
  latStr.trim();
  lonStr.trim();
  outHasLatLon = (latStr.length() > 0 && lonStr.length() > 0);
  if (outHasLatLon) {
    outLat = latStr.toDouble();
    outLon = lonStr.toDouble();
  } else if (grid.length() >= 4) {
    // JeĹ›li nie ma lat/lon z QRZ ale mamy lokator, przekonwertuj na wspĂłĹ‚rzÄ™dne
    double latFromGrid = 0.0, lonFromGrid = 0.0;
    locatorToLatLon(grid, latFromGrid, lonFromGrid);
    if (latFromGrid != 0.0 || lonFromGrid != 0.0) {
      outLat = latFromGrid;
      outLon = lonFromGrid;
      outHasLatLon = true;
      Serial.print("[QRZ DEBUG] Converted grid ");
      Serial.print(grid);
      Serial.print(" to lat=");
      Serial.print(outLat, 6);
      Serial.print(" lon=");
      Serial.println(outLon, 6);
    } else {
      outLat = 0.0;
      outLon = 0.0;
    }
  } else {
    outLat = 0.0;
    outLon = 0.0;
  }
  qrzStatus = "QRZ: lookup ok";

  // Zapisz do cache (nadpisz najstarszy)
  int oldestIdx = 0;
  unsigned long oldestMs = qrzCache[0].fetchedAtMs;
  for (int i = 1; i < QRZ_CACHE_SIZE; i++) {
    if (qrzCache[i].fetchedAtMs < oldestMs) {
      oldestMs = qrzCache[i].fetchedAtMs;
      oldestIdx = i;
    }
  }
  qrzCache[oldestIdx].callsign = call;
  qrzCache[oldestIdx].grid = grid;
  qrzCache[oldestIdx].country = country;
  qrzCache[oldestIdx].name = name;
  qrzCache[oldestIdx].email = email;
  qrzCache[oldestIdx].qth = qth;
  qrzCache[oldestIdx].lat = (float)outLat;
  qrzCache[oldestIdx].lon = (float)outLon;
  qrzCache[oldestIdx].hasLatLon = outHasLatLon;
  qrzCache[oldestIdx].fetchedAtMs = now;

  return (grid.length() > 0 || country.length() > 0 || name.length() > 0);
}

bool getQrzCacheFresh(const String &callsign, String &outGrid, String &outCountry, String &outName,
                     String &outEmail, String &outQth, double &outLat, double &outLon, bool &outHasLatLon,
                     unsigned long ttlMs) {
  outGrid = "";
  outCountry = "";
  outName = "";
  outEmail = "";
  outQth = "";
  outLat = 0.0;
  outLon = 0.0;
  outHasLatLon = false;

  String call = callsign;
  call.toUpperCase();
  unsigned long now = millis();
  for (int i = 0; i < QRZ_CACHE_SIZE; i++) {
    if (qrzCache[i].callsign == call && (now - qrzCache[i].fetchedAtMs) < ttlMs) {
      outGrid = qrzCache[i].grid;
      outCountry = qrzCache[i].country;
      outName = qrzCache[i].name;
      outEmail = qrzCache[i].email;
      outQth = qrzCache[i].qth;
      outLat = qrzCache[i].lat;
      outLon = qrzCache[i].lon;
      outHasLatLon = qrzCache[i].hasLatLon;
      return (outGrid.length() > 0 || outCountry.length() > 0 || outName.length() > 0);
    }
  }
  return false;
}

void applyQrzCacheToSpot(DXSpot &spot, unsigned long ttlMs) {
  if (spot.callsign.length() == 0) {
    return;
  }

  String grid;
  String country;
  String name;
  String email;
  String qth;
  double lat = 0.0;
  double lon = 0.0;
  bool hasLatLon = false;
  if (!getQrzCacheFresh(spot.callsign, grid, country, name, email, qth, lat, lon, hasLatLon, ttlMs)) {
    return;
  }

  if (spot.country.length() == 0 && country.length() > 0) {
    spot.country = country;
  }
  if (spot.name.length() == 0 && name.length() > 0) {
    spot.name = name;
  }
  if (spot.locator.length() < 4 && grid.length() >= 4) {
    spot.locator = grid;
  }
  if (!spot.hasLatLon && hasLatLon) {
    spot.lat = (float)lat;
    spot.lon = (float)lon;
    spot.hasLatLon = true;
  }
}

void logQrzAllFields(const String &callsign, const String &xml) {
  int csStart = xml.indexOf("<Callsign>");
  int csEnd = xml.indexOf("</Callsign>");
  if (csStart < 0 || csEnd < 0 || csEnd <= csStart) {
    Serial.print("[QRZ][DATA] ");
    Serial.print(callsign);
    Serial.println(" - brak CallSign w XML");
    return;
  }
  String section = xml.substring(csStart + 10, csEnd);

  Serial.print("[QRZ][DATA] ");
  Serial.println(callsign);

  int pos = 0;
  while (pos < section.length()) {
    int openTagStart = section.indexOf('<', pos);
    if (openTagStart < 0) break;
    int openTagEnd = section.indexOf('>', openTagStart + 1);
    if (openTagEnd < 0) break;

    String tag = section.substring(openTagStart + 1, openTagEnd);
    tag.trim();
    if (tag.length() == 0 || tag.indexOf('/') == 0) {
      pos = openTagEnd + 1;
      continue;
    }
    int closeTagStart = section.indexOf("</" + tag + ">", openTagEnd + 1);
    if (closeTagStart < 0) {
      pos = openTagEnd + 1;
      continue;
    }
    String value = section.substring(openTagEnd + 1, closeTagStart);
    value.trim();
    if (value.length() > 0) {
      Serial.print("  ");
      Serial.print(tag);
      Serial.print(": ");
      Serial.println(value);
    }
    pos = closeTagStart + tag.length() + 3;
  }
}

void logSpotList() {
  Serial.println("[SPOTS] Lista spotow (max 8)");
  int count = min(spotCount, 8);
  for (int i = 0; i < count; i++) {
    String call = spots[i].callsign.length() ? spots[i].callsign : "-";
    String timeStr = spots[i].time.length() ? spots[i].time : "----Z";
    float mhz = spots[i].frequency / 1000.0f;
    Serial.print(" ");
    Serial.print(i + 1);
    Serial.print(") ");
    Serial.print(call);
    Serial.print(" ");
    Serial.print(timeStr);
    Serial.print(" ");
    Serial.print(mhz, 3);
    Serial.println(" MHz");

    double lat = 0.0;
    double lon = 0.0;
    bool hasLatLon = spots[i].hasLatLon;
    if (hasLatLon) {
      lat = spots[i].lat;
      lon = spots[i].lon;
    } else if (spots[i].locator.length() >= 4) {
      locatorToLatLon(spots[i].locator, lat, lon);
      hasLatLon = true;
    }

    if (hasLatLon) {
      Serial.print("    lat=");
      Serial.print(lat, 4);
      Serial.print(" lon=");
      Serial.println(lon, 4);
    } else {
      Serial.println("    lat=-- lon=--");
    }

    if (spots[i].distance > 0) {
      Serial.print("    dist=");
      Serial.print(spots[i].distance, 0);
      Serial.println(" km");
    } else {
      Serial.println("    dist=-- km");
    }
  }
}

bool isQrzQueued(const String &callsign) {
  String normalized = callsign;
  normalized.trim();
  normalized.toUpperCase();
  for (int i = 0; i < qrzQueueLen; i++) {
    String queuedCall = qrzQueue[i].callsign;
    queuedCall.toUpperCase();
    if (queuedCall == normalized) {
      return true;
    }
  }
  return false;
}

void enqueueQrzLookup(const String &callsign) {
  String normalized = callsign;
  normalized.trim();
  normalized.toUpperCase();
  if (normalized.length() == 0) {
    return;
  }
  if (qrzQueueLen >= QRZ_QUEUE_SIZE) {
    return;
  }
  if (isQrzQueued(normalized)) {
    return;
  }

  // Nie dodawaj do kolejki, jeżeli mamy już świeży wpis w cache ze wszystkimi danymi.
  String grid;
  String country;
  String name;
  String email;
  String qth;
  double lat = 0.0;
  double lon = 0.0;
  bool hasLatLon = false;
  if (getQrzCacheFresh(normalized, grid, country, name, email, qth, lat, lon, hasLatLon, QRZ_CACHE_TTL_MS)) {
    return;
  }

  PendingQrzLookup item;
  item.callsign = normalized;
  item.nextTryMs = millis();
  item.attempts = 0;
  qrzQueue[qrzQueueLen++] = item;
}

// Pobierz spoty POTA z publicznego API i uzupełnij bufor + kolejkę QRZ
bool fetchPotaApi() {
  if (!wifiConnected) return false;
  auto decodeToSpots = [&](JsonDocument &doc) -> bool {
    JsonArray arr;
    if (doc.is<JsonArray>()) {
      arr = doc.as<JsonArray>();
    } else if (doc["spots"].is<JsonArray>()) {
      arr = doc["spots"].as<JsonArray>();
    }
    if (arr.isNull() || arr.size() == 0) {
      return false;
    }

    potaSpotCount = 0;
    for (JsonObject spot : arr) {
      if (potaSpotCount >= MAX_POTA_SPOTS) break;
      DXSpot s;
      s.time = spot["activatorLastSpotTime"] | spot["spotTime"] | "";
      s.callsign = spot["activator"] | spot["call"] | spot["callsign"] | "";
      float freq = 0.0f;
      if (spot["frequency"].is<float>() || spot["frequency"].is<double>()) {
        freq = spot["frequency"].as<float>();
      } else if (spot["frequency"].is<int>() || spot["frequency"].is<long>()) {
        freq = (float)spot["frequency"].as<long>();
      } else if (spot["frequency"].is<const char*>() || spot["frequency"].is<String>()) {
        String fstr = spot["frequency"].as<String>();
        freq = fstr.toFloat();
      }
      if (freq > 0) {
        s.frequency = freq / 1000.0f;
      } else {
        s.frequency = 0.0f;
      }
      s.band = getBand(s.frequency);
      s.mode = spot["mode"] | "";
      s.mode.toUpperCase();
      if (s.mode.length() == 0) {
        s.mode = getMode(spot["comments"] | "");
      }
      s.country = spot["country"] | "";
      s.spotter = spot["spotter"] | "";
      s.comment = spot["comments"] | "";
      s.distance = 0;
      s.hasLatLon = false;
      applyQrzCacheToSpot(s, QRZ_CACHE_TTL_MS);
      compactDxSpotStrings(s);
      potaSpots[potaSpotCount++] = s;
      if (s.country.length() == 0 && s.callsign.length() > 0) {
        enqueueQrzLookup(s.callsign);
      }
    }
    return potaSpotCount > 0;
  };

  HTTPClient http;
  http.setTimeout(15000);
  if (!http.begin(potaApiUrl)) {
    Serial.println("[POTA] http.begin failed");
    return false;
  }
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.print("[POTA] HTTP GET failed: ");
    Serial.print(code);
    Serial.print(" - ");
    Serial.println(http.errorToString(code));
    http.end();
    return false;
  }
  int contentLength = http.getSize();
  Serial.print("[POTA] HTTP ");
  Serial.print(code);
  Serial.print(", content-length=");
  Serial.print(contentLength);
  Serial.println(" (may be -1 if chunked)");

  WiFiClient *stream = http.getStreamPtr();
  DynamicJsonDocument filter(512);
  JsonObject filterRootArray = filter[0].to<JsonObject>();
  filterRootArray["activatorLastSpotTime"] = true;
  filterRootArray["spotTime"] = true;
  filterRootArray["activator"] = true;
  filterRootArray["call"] = true;
  filterRootArray["callsign"] = true;
  filterRootArray["frequency"] = true;
  filterRootArray["mode"] = true;
  filterRootArray["country"] = true;
  filterRootArray["spotter"] = true;
  filterRootArray["comments"] = true;

  DynamicJsonDocument doc(200000);
  DeserializationError err = deserializeJson(doc, *stream, DeserializationOption::Filter(filter));
  http.end();
  bool parsedOk = (!err && decodeToSpots(doc));
  if (!parsedOk) {
    if (err) {
      Serial.print("[POTA] Filtered JSON parse error: ");
      Serial.print(err.c_str());
      Serial.print(" (content-length was ");
      Serial.print(contentLength);
      Serial.println(")");
    }
    Serial.println("[POTA] Filtered parse empty - retrying without filter");

    HTTPClient retry;
    retry.setTimeout(8000);
    if (!retry.begin(potaApiUrl)) {
      return false;
    }
    int retryCode = retry.GET();
    if (retryCode != HTTP_CODE_OK) {
      retry.end();
      return false;
    }
    WiFiClient *retryStream = retry.getStreamPtr();
    DynamicJsonDocument fullDoc(400000);
    DeserializationError retryErr = deserializeJson(fullDoc, *retryStream);
    retry.end();
    if (retryErr) {
      Serial.print("[POTA] Fallback JSON parse error: ");
      Serial.println(retryErr.c_str());
      return false;
    }
    if (!decodeToSpots(fullDoc)) {
      Serial.println("[POTA] Fallback parse also returned 0 spots");
      return false;
    }
  }

  // Posortuj malejąco po czasie (ISO string porównuje się leksykograficznie poprawnie)
  for (int i = 0; i < potaSpotCount - 1; i++) {
    for (int j = i + 1; j < potaSpotCount; j++) {
      if (potaSpots[j].time > potaSpots[i].time) {
        DXSpot tmp = potaSpots[i];
        potaSpots[i] = potaSpots[j];
        potaSpots[j] = tmp;
      }
    }
  }

  return potaSpotCount > 0;
}

void removeQrzQueueAt(int idx) {
  if (idx < 0 || idx >= qrzQueueLen) {
    return;
  }
  for (int i = idx; i < qrzQueueLen - 1; i++) {
    qrzQueue[i] = qrzQueue[i + 1];
  }
  qrzQueueLen--;
}

void updateSpotsWithQrz(const String &callsign, const String &grid,
                        const String &country, const String &name, double lat, double lon, bool hasLatLon) {
  bool updated = false;
  bool updatedPota = false;
  if (!userLatLonValid && userLocator.length() >= 4) {
    double tmpLat = 0.0;
    double tmpLon = 0.0;
    locatorToLatLon(userLocator, tmpLat, tmpLon);
    userLat = tmpLat;
    userLon = tmpLon;
    userLatLonValid = true;
  }
  lockDxSpots();
  for (int i = 0; i < spotCount; i++) {
    if (!spots[i].callsign.equalsIgnoreCase(callsign)) {
      continue;
    }

    if (grid.length() >= 4) {
      spots[i].locator = grid;
    }
    if (country.length() > 0) {
      spots[i].country = country;
    }
    if (name.length() > 0) {
      spots[i].name = name;
    }

    double spotLat = 0.0;
    double spotLon = 0.0;
    bool spotHasLatLon = false;
    if (hasLatLon) {
      spotLat = lat;
      spotLon = lon;
      spotHasLatLon = true;
      spots[i].lat = (float)lat;
      spots[i].lon = (float)lon;
      spots[i].hasLatLon = true;
    } else if (spots[i].locator.length() >= 4) {
      locatorToLatLon(spots[i].locator, spotLat, spotLon);
      spotHasLatLon = true;
      spots[i].lat = (float)spotLat;
      spots[i].lon = (float)spotLon;
      spots[i].hasLatLon = true;
    }

    double userLatLocal = 0.0;
    double userLonLocal = 0.0;
    bool userHasLatLon = userLatLonValid;
    if (userHasLatLon) {
      userLatLocal = userLat;
      userLonLocal = userLon;
    } else if (userLocator.length() >= 4) {
      locatorToLatLon(userLocator, userLatLocal, userLonLocal);
      userHasLatLon = true;
    }

    if (userHasLatLon && spotHasLatLon) {
      spots[i].distance = calculateDistance(userLatLocal, userLonLocal, spotLat, spotLon);
      updated = true;
    }
  }
  unlockDxSpots();

  for (int i = 0; i < potaSpotCount; i++) {
    if (!potaSpots[i].callsign.equalsIgnoreCase(callsign)) {
      continue;
    }

    if (grid.length() >= 4) {
      potaSpots[i].locator = grid;
    }
    if (country.length() > 0) {
      potaSpots[i].country = country;
    }

    double spotLat = 0.0;
    double spotLon = 0.0;
    bool spotHasLatLon = false;
    if (hasLatLon) {
      spotLat = lat;
      spotLon = lon;
      spotHasLatLon = true;
      potaSpots[i].lat = (float)lat;
      potaSpots[i].lon = (float)lon;
      potaSpots[i].hasLatLon = true;
    } else if (potaSpots[i].locator.length() >= 4) {
      locatorToLatLon(potaSpots[i].locator, spotLat, spotLon);
      spotHasLatLon = true;
      potaSpots[i].lat = (float)spotLat;
      potaSpots[i].lon = (float)spotLon;
      potaSpots[i].hasLatLon = true;
    }

    double userLatLocal = 0.0;
    double userLonLocal = 0.0;
    bool userHasLatLon = userLatLonValid;
    if (userHasLatLon) {
      userLatLocal = userLat;
      userLonLocal = userLon;
    } else if (userLocator.length() >= 4) {
      locatorToLatLon(userLocator, userLatLocal, userLonLocal);
      userHasLatLon = true;
    }

    if (userHasLatLon && spotHasLatLon) {
      potaSpots[i].distance = calculateDistance(userLatLocal, userLonLocal, spotLat, spotLon);
      updatedPota = true;
    }
  }

  if (updated) {
#ifdef ENABLE_TFT_DISPLAY
    updateScreen2();
#endif
  }
  if (updatedPota) {
#ifdef ENABLE_TFT_DISPLAY
    updateScreen7();
#endif
  }
}

bool fetchQrzRawXml(const String &callsign, String &body) {
  if (!wifiConnected || WiFi.status() != WL_CONNECTED) {
    qrzStatus = "QRZ: no wifi";
    return false;
  }
  String sessionKey;
  if (!ensureQrzSession(sessionKey)) {
    return false;
  }

  String url = "https://xmldata.qrz.com/xml/current/?s=" + sessionKey +
               ";callsign=" + callsign;
  HTTPClient http;
  http.setTimeout(3000);
  http.begin(url);
  int code = http.GET();
  if (code != 200) {
    http.end();
    qrzStatus = "QRZ: lookup http " + String(code);
    return false;
  }
  body = http.getString();
  http.end();

  if (body.indexOf("Session Timeout") >= 0 || body.indexOf("Invalid session") >= 0) {
    String dummy;
    ensureQrzSession(dummy);
    qrzStatus = "QRZ: session expired";
    return false;
  }
  return true;
}

void runQrzSingleTest(const String &callsign) {
  (void)callsign;
}

// Parsowanie spotu DX z formatu "DX de [Spotter]: [Freq] [Call] [Comments] [Time]"
bool parseDXSpot(String line, DXSpot &spot) {
  LOGV_PRINT("[PARSE] parseDXSpot START, len=");
  LOGV_PRINTLN(line.length());
  
  // Format: DX de SP5XYZ: 14025.0 SP9ABC Test comment 1234Z
  if (!line.startsWith("DX de")) {
    LOGV_PRINTLN("[PARSE] Nie zaczyna siĂ„â„˘ od 'DX de' - wyjÄąâ€şcie");
    return false;
  }
  
  int dePos = line.indexOf("DX de");
  if (dePos < 0) return false;
  
  int colonPos = line.indexOf(":", dePos);
  if (colonPos < 0) return false;
  
  // WyciĂ„â€¦gnij spottera
  String spotterPart = line.substring(dePos + 5, colonPos);
  spotterPart.trim();
  spot.spotter = spotterPart;
  
  // Reszta po dwukropku
  String rest = line.substring(colonPos + 1);
  rest.trim();
  
  // Parsuj czĂ„â„˘stotliwoÄąâ€şĂ„â€ˇ (pierwsza liczba)
  int spacePos = rest.indexOf(" ");
  if (spacePos < 0) return false;
  
  String freqStr = rest.substring(0, spacePos);
  spot.frequency = freqStr.toFloat();
  rest = rest.substring(spacePos + 1);
  rest.trim();
  
  // Parsuj znak wywoÄąâ€šawczy (nastĂ„â„˘pne sÄąâ€šowo)
  spacePos = rest.indexOf(" ");
  if (spacePos < 0) {
    spot.callsign = rest;
    spot.comment = "";
  } else {
    spot.callsign = rest.substring(0, spacePos);
    rest = rest.substring(spacePos + 1);
    rest.trim();
    
    // Reszta to komentarz (moÄąÄ˝e zawieraĂ„â€ˇ czas na koÄąâ€žcu)
    // Czas jest zwykle na koÄąâ€žcu w formacie HHMMZ
    int timePos = rest.lastIndexOf("Z");
    if (timePos > 0 && rest.length() >= timePos + 1) {
      String timeStr = rest.substring(timePos - 4, timePos + 1);
      if (timeStr.length() == 5) {
        spot.time = timeStr;
        spot.comment = rest.substring(0, timePos - 4);
        spot.comment.trim();
      } else {
        spot.comment = rest;
        // Pobierz czas z NTP
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
          char timeBuffer[6];
          strftime(timeBuffer, 6, "%H%MZ", &timeinfo);
          spot.time = String(timeBuffer);
        } else {
          spot.time = "----Z";
        }
      }
    } else {
      spot.comment = rest;
      // Pobierz czas z NTP (z timeout - nie blokuj jeÄąâ€şli NTP nie dziaÄąâ€ša)
      struct tm timeinfo;
      if (getLocalTime(&timeinfo, 1)) { // Timeout 1 sekunda
        char timeBuffer[6];
        strftime(timeBuffer, 6, "%H%MZ", &timeinfo);
        spot.time = String(timeBuffer);
      } else {
        spot.time = "----Z";
      }
    }
  }
  
  // OkreÄąâ€şl pasmo i modulacjĂ„â„˘
  spot.band = getBand(spot.frequency);
  spot.mode = getMode(spot.comment);
  
  // SprÄ‚łbuj wyciĂ„â€¦gnĂ„â€¦Ă„â€ˇ locator z komentarza (format JO82LK)
  spot.locator = "";
  int commentLen = spot.comment.length();
  if (commentLen >= 6) { // Locator ma minimum 6 znakÄ‚łw
    for (int i = 0; i <= commentLen - 6; i++) {
      String sub = spot.comment.substring(i, i + 6);
      if (sub.length() == 6 && 
          isAlpha(sub.charAt(0)) && isAlpha(sub.charAt(1)) &&
          isDigit(sub.charAt(2)) && isDigit(sub.charAt(3)) &&
          isAlpha(sub.charAt(4)) && isAlpha(sub.charAt(5))) {
        spot.locator = sub;
        break;
      }
    }
  }
  
  LOGV_PRINTLN("[PARSE] Obliczanie odlegÄąâ€šoÄąâ€şci...");
  
  spot.distance = 0;
  spot.country = "";
  spot.lat = 0.0f;
  spot.lon = 0.0f;
  spot.hasLatLon = false;
  bool qrzConfigured = (qrzUsername.length() > 0 && qrzPassword.length() > 0);
  // Najpierw licz z lokatora jeÄąâ€şli mamy lokalizacjĂ„â„˘ i lokator w spocie
  if ((userLatLonValid || userLocator.length() >= 4) && spot.locator.length() >= 4) {
    double userLatLocal, userLonLocal, spotLat, spotLon;
    if (userLatLonValid) {
      userLatLocal = userLat;
      userLonLocal = userLon;
    } else {
      locatorToLatLon(userLocator, userLatLocal, userLonLocal);
    }
    locatorToLatLon(spot.locator, spotLat, spotLon);
    spot.distance = calculateDistance(userLatLocal, userLonLocal, spotLat, spotLon);
    spot.lat = (float)spotLat;
    spot.lon = (float)spotLon;
    spot.hasLatLon = true;
  }

  if (qrzConfigured) {
    applyQrzCacheToSpot(spot, QRZ_CACHE_TTL_MS);
  }

  if (qrzConfigured && spot.callsign.length() > 0 && (spot.country.length() == 0 || !spot.hasLatLon)) {
    String call = spot.callsign;
    call.toUpperCase();
    enqueueQrzLookup(call);
  }

  compactDxSpotStrings(spot);
  
  LOGV_PRINTLN("[PARSE] parseDXSpot END - OK");
  return true;
}

// Prostsze parsowanie spotu dla POTA (bez QRZ i bez obliczeÄąâ€ž dystansu)
bool parsePotaSpot(String line, DXSpot &spot) {
  if (!line.startsWith("DX de")) {
    return false;
  }

  int dePos = line.indexOf("DX de");
  if (dePos < 0) return false;

  int colonPos = line.indexOf(":", dePos);
  if (colonPos < 0) return false;

  String spotterPart = line.substring(dePos + 5, colonPos);
  spotterPart.trim();
  spot.spotter = spotterPart;

  String rest = line.substring(colonPos + 1);
  rest.trim();

  int spacePos = rest.indexOf(" ");
  if (spacePos < 0) return false;

  String freqStr = rest.substring(0, spacePos);
  spot.frequency = freqStr.toFloat();
  rest = rest.substring(spacePos + 1);
  rest.trim();

  spacePos = rest.indexOf(" ");
  if (spacePos < 0) {
    spot.callsign = rest;
    spot.comment = "";
  } else {
    spot.callsign = rest.substring(0, spacePos);
    rest = rest.substring(spacePos + 1);
    rest.trim();

    int timePos = rest.lastIndexOf("Z");
    if (timePos > 0 && rest.length() >= timePos + 1) {
      String timeStr = rest.substring(timePos - 4, timePos + 1);
      if (timeStr.length() == 5) {
        spot.time = timeStr;
        spot.comment = rest.substring(0, timePos - 4);
        spot.comment.trim();
      } else {
        spot.comment = rest;
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
          char timeBuffer[6];
          strftime(timeBuffer, 6, "%H%MZ", &timeinfo);
          spot.time = String(timeBuffer);
        } else {
          spot.time = "----Z";
        }
      }
    } else {
      spot.comment = rest;
      struct tm timeinfo;
      if (getLocalTime(&timeinfo, 1)) {
        char timeBuffer[6];
        strftime(timeBuffer, 6, "%H%MZ", &timeinfo);
        spot.time = String(timeBuffer);
      } else {
        spot.time = "----Z";
      }
    }
  }

  spot.band = getBand(spot.frequency);
  spot.mode = getMode(spot.comment);
  spot.locator = "";
  spot.distance = 0;
  spot.country = "";
  spot.lat = 0.0f;
  spot.lon = 0.0f;
  spot.hasLatLon = false;
  compactDxSpotStrings(spot);

  return true;
}

// Dodaj nowy spot do tablicy
void addSpot(DXSpot spot) {
  compactDxSpotStrings(spot);
  lockDxSpots();
  LOGV_PRINT("[SPOT] addSpot: ");
  LOGV_PRINT(spot.callsign);
  LOGV_PRINT(" @ ");
  LOGV_PRINT(spot.frequency);
  LOGV_PRINT(" kHz, spotCount=");
  LOGV_PRINTLN(spotCount);
  
  // PrzesuÄąâ€ž istniejĂ„â€¦ce spoty
  if (spotCount < MAX_SPOTS) {
    spotCount++;
  }
  
  // PrzesuÄąâ€ž wszystkie spoty o jeden w dÄ‚łÄąâ€š
  for (int i = MAX_SPOTS - 1; i > 0; i--) {
    spots[i] = spots[i - 1];
  }
  
  // Dodaj nowy spot na poczĂ„â€¦tku
  spots[0] = spot;
  
  LOGV_PRINT("[SPOT] addSpot END, nowy spotCount=");
  LOGV_PRINTLN(spotCount);
  unlockDxSpots();
  // logSpotList(); // WyÄąâ€šĂ„â€¦czone - nie wypisuj listy spotÄ‚łw do Serial
}

void addPotaSpot(DXSpot spot) {
  compactDxSpotStrings(spot);
  if (potaSpotCount < MAX_POTA_SPOTS) {
    potaSpotCount++;
  }

  for (int i = MAX_POTA_SPOTS - 1; i > 0; i--) {
    potaSpots[i] = potaSpots[i - 1];
  }

  potaSpots[0] = spot;
}

void addHamalertSpot(DXSpot spot) {
  compactDxSpotStrings(spot);
  if (hamalertSpotCount < MAX_POTA_SPOTS) {
    hamalertSpotCount++;
  }

  for (int i = MAX_POTA_SPOTS - 1; i > 0; i--) {
    hamalertSpots[i] = hamalertSpots[i - 1];
  }

  hamalertSpots[0] = spot;
}

bool parseHamalertJsonSpot(const String &line, DXSpot &spot) {
  DynamicJsonDocument doc(3072);
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    return false;
  }

  JsonVariant root = doc.as<JsonVariant>();
  String callsign = root["dx"] | root["call"] | root["callsign"] | "";
  if (callsign.length() == 0) {
    return false;
  }

  spot.callsign = callsign;
  spot.spotter = root["de"] | root["spotter"] | root["source"] | "";
  spot.comment = root["comment"] | root["info"] | root["message"] | "";
  spot.mode = root["mode"] | "";
  if (spot.mode.length() == 0) {
    spot.mode = getMode(spot.comment);
  }
  spot.mode.toUpperCase();
  spot.country = root["country"] | root["entity"] | "";

  float freq = 0.0f;
  if (root["frequency"].is<float>() || root["frequency"].is<double>()) {
    freq = (float)root["frequency"].as<double>();
  } else if (root["frequency"].is<int>() || root["frequency"].is<long>()) {
    freq = (float)root["frequency"].as<long>();
  } else if (root["freq"].is<float>() || root["freq"].is<double>()) {
    freq = (float)root["freq"].as<double>();
  } else if (root["freq"].is<int>() || root["freq"].is<long>()) {
    freq = (float)root["freq"].as<long>();
  } else if (root["frequency"].is<const char*>()) {
    freq = String(root["frequency"].as<const char*>()).toFloat();
  } else if (root["freq"].is<const char*>()) {
    freq = String(root["freq"].as<const char*>()).toFloat();
  }
  if (freq <= 0.0f) {
    return false;
  }
  // HAMALERT i DX cluster zwykle podają kHz, ale obsłuż także MHz.
  spot.frequency = (freq > 1000.0f) ? (freq / 1000.0f) : freq;
  spot.band = getBand(spot.frequency * 1000.0f);

  String timeStr = root["time"] | root["spotTime"] | root["timestamp"] | "";
  if (timeStr.length() == 0) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 1)) {
      char timeBuffer[6];
      strftime(timeBuffer, 6, "%H%MZ", &timeinfo);
      timeStr = String(timeBuffer);
    } else {
      timeStr = "----Z";
    }
  }
  spot.time = timeStr;
  spot.distance = 0;
  spot.locator = "";
  spot.lat = 0.0f;
  spot.lon = 0.0f;
  spot.hasLatLon = false;
  compactDxSpotStrings(spot);
  return true;
}

bool parseHamalertSpotLine(const String &lineIn, DXSpot &spot) {
  String line = lineIn;
  line.trim();
  if (line.length() < 6) {
    return false;
  }
  if (line == ">" || line == "#" || line.startsWith("Welcome") || line.startsWith("Password")) {
    return false;
  }
  if (line.charAt(0) == '{' && parseHamalertJsonSpot(line, spot)) {
    return true;
  }

  DXSpot parsed;
  if (!parsePotaSpot(line, parsed)) {
    return false;
  }
  parsed.frequency = (parsed.frequency > 1000.0f) ? (parsed.frequency / 1000.0f) : parsed.frequency;
  parsed.band = getBand(parsed.frequency * 1000.0f);
  parsed.mode.toUpperCase();
  compactDxSpotStrings(parsed);
  spot = parsed;
  return true;
}

bool fetchHamalertTelnet() {
  const size_t HAMALERT_MAX_LINE_LEN = 1400;
  Serial.println("[HAMALERT] fetch start");
  if (!wifiConnected) {
    Serial.println("[HAMALERT] skip: WiFi offline");
    return false;
  }
  if (hamalertHost.length() == 0 || hamalertPort <= 0 || hamalertLogin.length() == 0 || hamalertPassword.length() == 0) {
    Serial.println("[HAMALERT] skip: missing host/port/login/password");
    return false;
  }

  String login = hamalertLogin;
  login.trim();
  if (login.length() == 0) {
    Serial.println("[HAMALERT] skip: hamalert_login is empty");
    return false;
  }

  Serial.print("[HAMALERT] connect ");
  Serial.print(hamalertHost);
  Serial.print(":");
  Serial.println(hamalertPort);

  WiFiClient client;
  client.setTimeout(1200);
  if (!client.connect(hamalertHost.c_str(), hamalertPort)) {
    Serial.println("[HAMALERT] connect failed");
    return false;
  }
  Serial.println("[HAMALERT] connected");

  hamalertSpotCount = 0;
  String line;
  line.reserve(HAMALERT_MAX_LINE_LEN + 16);

  // Odczytaj ewentualny banner/prompt przed logowaniem
  unsigned long preLoginUntil = millis() + 1200;
  while (millis() < preLoginUntil && client.connected()) {
    while (client.available()) {
      char c = (char)client.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (line.length() > 0) {
          Serial.print("[HAMALERT] < ");
          Serial.println(line);
        }
        line = "";
      } else if (line.length() < HAMALERT_MAX_LINE_LEN) {
        line += c;
      }
    }
    delay(2);
    yield();
  }
  if (line.length() > 0) {
    Serial.print("[HAMALERT] < ");
    Serial.println(line);
    line = "";
  }

  // HamAlert telnet wymaga username + password
  Serial.print("[HAMALERT] > username: ");
  Serial.println(login);
  client.print(login);
  client.print("\n");

  delay(180);
  Serial.print("[HAMALERT] > password: (");
  Serial.print(hamalertPassword.length());
  Serial.println(" chars)");
  client.print(hamalertPassword);
  client.print("\n");

  delay(220);
  Serial.println("[HAMALERT] > set/json");
  client.print("set/json\n");

  delay(240);
  Serial.println("[HAMALERT] > sh/dx 30");
  client.print("sh/dx 30\n");

  const unsigned long started = millis();
  unsigned long lastRx = millis();
  const unsigned long totalTimeoutMs = 9000;
  const unsigned long idleTimeoutMs = 1800;
  int parsedOkCount = 0;
  int parsedFailCount = 0;
  int diagLineCount = 0;

  while (client.connected() && (millis() - started) < totalTimeoutMs) {
    while (client.available()) {
      char c = (char)client.read();
      lastRx = millis();
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        if (line.length() > 0) {
          DXSpot s;
          if (parseHamalertSpotLine(line, s)) {
            addHamalertSpot(s);
            parsedOkCount++;
            if (hamalertSpotCount >= MAX_POTA_SPOTS) {
              break;
            }
          } else {
            parsedFailCount++;
            String lowerLine = line;
            lowerLine.toLowerCase();
            if (lowerLine.indexOf("invalid") >= 0 || lowerLine.indexOf("error") >= 0 || lowerLine.indexOf("denied") >= 0 ||
                lowerLine.indexOf("login") >= 0 || lowerLine.indexOf("password") >= 0 || lowerLine.indexOf("failed") >= 0) {
              Serial.print("[HAMALERT] server: ");
              Serial.println(line);
            } else if (diagLineCount < 8) {
              Serial.print("[HAMALERT] line(unparsed): ");
              Serial.println(line);
              diagLineCount++;
            }
          }
        }
        line = "";
      } else if (line.length() < HAMALERT_MAX_LINE_LEN) {
        line += c;
      }
    }
    if ((millis() - lastRx) > idleTimeoutMs && hamalertSpotCount > 0) {
      break;
    }
    if (hamalertSpotCount >= MAX_POTA_SPOTS) {
      break;
    }
    yield();
    delay(2);
  }

  if (line.length() > 0 && hamalertSpotCount < MAX_POTA_SPOTS) {
    DXSpot s;
    if (parseHamalertSpotLine(line, s)) {
      addHamalertSpot(s);
      parsedOkCount++;
    } else {
      parsedFailCount++;
    }
  }

  client.stop();
  Serial.print("[HAMALERT] done: spots=");
  Serial.print(hamalertSpotCount);
  Serial.print(", parsed_ok=");
  Serial.print(parsedOkCount);
  Serial.print(", parsed_fail=");
  Serial.println(parsedFailCount);
  return hamalertSpotCount > 0;
}

// ========== APRS-IS FUNKCJE ==========

// PoÄąâ€šĂ„â€¦cz z serwerem APRS-IS
void connectToAPRS() {
  Serial.println("[APRS] connectToAPRS() START");
  
  if (!wifiConnected) {
    Serial.println("[APRS] WiFi nie poÄąâ€šĂ„â€¦czony - wyjÄąâ€şcie");
    return;
  }
  
  if (aprsClient.connected()) {
    // JeÄąâ€şli konfiguracja siĂ„â„˘ zmieniÄąâ€ša, rozÄąâ€šĂ„â€¦cz i poÄąâ€šĂ„â€¦cz ponownie
    Serial.println("[APRS] JuÄąÄ˝ poÄąâ€šĂ„â€¦czony - sprawdzam czy potrzeba reconnect...");
    // MoÄąÄ˝na dodaĂ„â€ˇ sprawdzenie czy konfiguracja siĂ„â„˘ zmieniÄąâ€ša, ale na razie zostawiamy jak jest
    // W razie potrzeby reconnect nastĂ„â€¦pi automatycznie przez watchdog
    return;
  }
  
  unsigned long now = millis();
  if (now - lastAPRSAttempt < 5000) {
    return; // Cichy return - nie spamuj logu
  }
  
  lastAPRSAttempt = now;
  
  Serial.print("[APRS] ÄąÂĂ„â€¦czenie z APRS-IS: ");
  Serial.print(aprsIsHost);
  Serial.print(":");
  Serial.println(aprsIsPort);
  
  // Diagnostyka DNS
  IPAddress resolvedIp;
  if (WiFi.hostByName(aprsIsHost.c_str(), resolvedIp)) {
    Serial.print("DNS OK: ");
    Serial.print(aprsIsHost);
    Serial.print(" -> ");
    Serial.println(resolvedIp);
  } else {
    Serial.print("DNS FAIL dla hosta: ");
    Serial.println(aprsIsHost);
  }
  
  Serial.println("[APRS] WywoÄąâ€šanie aprsClient.connect()...");
  unsigned long connectStart = millis();
  
  // Ustaw timeout poÄąâ€šĂ„â€¦czenia (300 sekund jak w wymaganiach)
  aprsClient.setTimeout(300);
  
  if (aprsClient.connect(aprsIsHost.c_str(), aprsIsPort)) {
    unsigned long connectTime = millis() - connectStart;
    Serial.print("[APRS] PoÄąâ€šĂ„â€¦czono z APRS-IS! (czas: ");
    Serial.print(connectTime);
    Serial.println("ms)");
    aprsConnected = true;
    aprsBuffer = "";
    aprsLoginSent = false;
    lastAPRSRxMs = millis();
    
    // Zaplanuj wysÄąâ€šanie loginu
    delay(500); // KrÄ‚łtkie opÄ‚łÄąĹźnienie przed loginem
    sendAPRSLogin();
  } else {
    unsigned long connectTime = millis() - connectStart;
    Serial.print("[APRS] BÄąâ€šĂ„â€¦d poÄąâ€šĂ„â€¦czenia z APRS-IS (czas: ");
    Serial.print(connectTime);
    Serial.println("ms)");
    aprsConnected = false;
  }
  Serial.println("[APRS] connectToAPRS() END");
}

// Wyślij login do APRS-IS
void sendAPRSLogin() {
  if (!aprsConnected || !aprsClient.connected()) {
    Serial.println("[APRS] Nie mozna wysłać loginu - brak połączenia");
    return;
  }
  
  // Format loginu: user Callsign pass 23123 vers ESP32-HAM-CLOCK 1.3
  String login = "user ";
  login += getAprsTxCallsignWithSsid();
  login += " pass ";
  login += String(aprsPasscode);
  login += " vers ESP32-HAM-CLOCK 1.3";
  
  Serial.print("[APRS] Wysyłanie loginu: ");
  Serial.println(login);
  
  aprsClient.println(login);
  aprsLoginSent = true;
  lastAPRSRxMs = millis();
  
  // Po zalogowaniu wyÄąâ€şlij filtr
  delay(500);
  sendAPRSFilter();
}

// WyÄąâ€şlij komendĂ„â„˘ filtra do APRS-IS
void sendAPRSFilter() {
  if (!aprsConnected || !aprsClient.connected()) {
    Serial.println("[APRS] Nie moÄąÄ˝na wysÄąâ€šaĂ„â€ˇ filtra - brak poÄąâ€šĂ„â€¦czenia");
    return;
  }
  
  // Format filtra: #filter r/52.40/16.92/50
  // UÄąÄ˝ywamy wspÄ‚łÄąâ€šrzĂ„â„˘dnych z sekcji "Moja Stacja"
  double filterLat = userLatLonValid ? userLat : 52.40;  // Fallback jeÄąâ€şli nie ustawione
  double filterLon = userLatLonValid ? userLon : 16.92;
  String filter = "#filter r/";
  filter += String(filterLat, 2);
  filter += "/";
  filter += String(filterLon, 2);
  filter += "/";
  filter += String(aprsFilterRadius);
  
  Serial.print("[APRS] WysyÄąâ€šanie filtra: ");
  Serial.println(filter);
  
  aprsClient.println(filter);
  lastAPRSRxMs = millis();
}

String formatAprsCoordinate(double value, bool isLat) {
  double absVal = fabs(value);
  int degrees = (int)absVal;
  double minutes = (absVal - degrees) * 60.0;
  String degText;
  if (isLat) {
    degText = (degrees < 10 ? "0" : "") + String(degrees);
  } else {
    if (degrees < 10) degText = "00" + String(degrees);
    else if (degrees < 100) degText = "0" + String(degrees);
    else degText = String(degrees);
  }

  String minText = String(minutes, 2);
  if (minutes < 10.0) {
    minText = "0" + minText;
  }

  char hemi = isLat ? ((value >= 0.0) ? 'N' : 'S') : ((value >= 0.0) ? 'E' : 'W');
  return degText + minText + String(hemi);
}

static bool isValidAprsSymbolChar(char c) {
  return (c >= 33 && c <= 126); // widoczne ASCII bez spacji/kontrolnych
}

static String sanitizeAprsSymbol(const String &sym) {
  String s = sym;
  s.trim();
  if (s.length() < 2) {
    return String(DEFAULT_APRS_SYMBOL_TABLE) + String(DEFAULT_APRS_SYMBOL_CODE);
  }
  s = s.substring(0, 2);
  if (!isValidAprsSymbolChar(s.charAt(0)) || !isValidAprsSymbolChar(s.charAt(1))) {
    return String(DEFAULT_APRS_SYMBOL_TABLE) + String(DEFAULT_APRS_SYMBOL_CODE);
  }
  return s;
}

static void applyAprsSymbol(const String &sym) {
  String s = sanitizeAprsSymbol(sym);
  aprsSymbolTwoChar = s;
  aprsSymbolTable = s.charAt(0);
  aprsSymbolCode = s.charAt(1);
}

static int normalizeAprsSsid(int ssid) {
  if (ssid < 0) return 0;
  if (ssid > 15) return 15;
  return ssid;
}

static void applyAprsSsid(int ssid) {
  aprsSsid = normalizeAprsSsid(ssid);
}

static String getAprsBaseCallsign() {
  String call = aprsCallsign.length() ? aprsCallsign : userCallsign;
  call.trim();
  call.toUpperCase();
  int dash = call.indexOf('-');
  if (dash > 0) {
    call = call.substring(0, dash);
  }
  return call;
}

static String getAprsTxCallsignWithSsid() {
  String baseCall = getAprsBaseCallsign();
  if (aprsSsid <= 0) {
    return baseCall;
  }
  return baseCall + "-" + String(aprsSsid);
}

static String sanitizeAprsComment(const String &cmt) {
  String s = cmt;
  s.trim();
  String out = "";
  for (size_t i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (c >= 32 && c <= 126) { // drukowalne ASCII
      out += c;
    }
    if (out.length() >= 43) { // rekomendowany limit komentarza APRS
      break;
    }
  }
  return out;
}

static String sanitizeAprsAlertWatchToken(const String &raw) {
  String token = raw;
  token.trim();
  token.toUpperCase();

  bool wildcardAnySsid = false;
  if (token.endsWith("*")) {
    wildcardAnySsid = true;
    token.remove(token.length() - 1);
  }

  String cleaned = "";
  for (size_t i = 0; i < token.length(); i++) {
    char ch = token.charAt(i);
    bool ok = (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '-' || ch == '/';
    if (ok) cleaned += ch;
  }

  if (cleaned.length() == 0) {
    return "";
  }
  if (wildcardAnySsid) {
    cleaned += "*";
  }
  return cleaned;
}

static String sanitizeAprsAlertList(const String &raw) {
  String src = raw;
  src.trim();
  if (src.length() == 0) return "";

  String out = "";
  int count = 0;
  int start = 0;

  while (start <= src.length() && count < 20) {
    int comma = src.indexOf(',', start);
    int end = (comma >= 0) ? comma : src.length();
    String token = src.substring(start, end);
    String cleaned = sanitizeAprsAlertWatchToken(token);

    if (cleaned.length() > 0) {
      if (count > 0) out += ",";
      out += cleaned;
      count++;
    }

    if (comma < 0) break;
    start = comma + 1;
  }

  return out;
}

static int normalizeAprsAlertMinSeconds(int seconds) {
  if (seconds < 0) return 0;
  if (seconds > 86400) return 86400;
  return seconds;
}

static void applyAprsAlertMinSeconds(int seconds) {
  aprsAlertMinSeconds = normalizeAprsAlertMinSeconds(seconds);
}

static float normalizeAprsAlertDistanceKm(float km) {
  if (km < 0.1f) return 0.1f;
  if (km > 500.0f) return 500.0f;
  return km;
}

static void applyAprsAlertDistanceKm(float km) {
  aprsAlertDistanceKm = normalizeAprsAlertDistanceKm(km);
}

static int normalizeAprsAlertScreenSeconds(int seconds) {
  if (seconds < 1) return 1;
  if (seconds > 60) return 60;
  return seconds;
}

static void applyAprsAlertScreenSeconds(int seconds) {
  aprsAlertScreenSeconds = normalizeAprsAlertScreenSeconds(seconds);
}

static String sanitizeAprsCallsignToken(const String &raw) {
  String token = raw;
  token.trim();
  token.toUpperCase();

  String cleaned = "";
  for (size_t i = 0; i < token.length(); i++) {
    char ch = token.charAt(i);
    bool ok = (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '-' || ch == '/';
    if (ok) cleaned += ch;
  }
  return cleaned;
}

static String getAprsCallsignBase(const String &callsign) {
  String normalized = sanitizeAprsCallsignToken(callsign);
  int dash = normalized.indexOf('-');
  if (dash > 0) {
    normalized = normalized.substring(0, dash);
  }
  return normalized;
}

static bool isAprsAlertCallMatch(const String &watchedToken, const String &incomingCallsign) {
  if (watchedToken.length() == 0 || incomingCallsign.length() == 0) {
    return false;
  }

  String watched = sanitizeAprsAlertWatchToken(watchedToken);
  String incoming = sanitizeAprsCallsignToken(incomingCallsign);
  if (watched.length() == 0 || incoming.length() == 0) {
    return false;
  }

  if (watched.endsWith("*")) {
    String base = watched.substring(0, watched.length() - 1);
    if (base.length() == 0) {
      return false;
    }
    if (incoming.equals(base)) {
      return true;
    }
    String withSsidPrefix = base + "-";
    return incoming.startsWith(withSsidPrefix);
  }

  return watched.equals(incoming);
}

static bool isAprsCallsignOnAlertList(const String &incomingCallsign) {
  String src = aprsAlertCsv;
  src.trim();
  if (src.length() == 0) return false;

  int start = 0;
  while (start <= src.length()) {
    int comma = src.indexOf(',', start);
    int end = (comma >= 0) ? comma : src.length();
    String token = src.substring(start, end);
    token.trim();

    if (isAprsAlertCallMatch(token, incomingCallsign)) {
      return true;
    }

    if (comma < 0) break;
    start = comma + 1;
  }

  return false;
}

static bool isAprsMobileSsid7or9(const String &incomingCallsign) {
  String normalized = sanitizeAprsCallsignToken(incomingCallsign);
  if (normalized.length() == 0) {
    return false;
  }

  int dash = normalized.lastIndexOf('-');
  if (dash <= 0 || dash >= (normalized.length() - 1)) {
    return false;
  }

  String ssid = normalized.substring(dash + 1);
  return ssid.equals("7") || ssid.equals("9");
}

static void resetAprsAlertCooldownState() {
  for (int i = 0; i < MAX_APRS_STATIONS; i++) {
    aprsAlertCooldown[i].callsign = "";
    aprsAlertCooldown[i].lastAlertMs = 0;
  }
  aprsAlertCooldownReplaceIdx = 0;
}

static bool shouldTriggerAprsAlert(const APRSStation &station) {
  bool watchListMatched = isAprsCallsignOnAlertList(station.callsign);

  bool nearbyMobileMatched = false;
  if (aprsAlertNearbyEnabled && station.hasLatLon && station.distance >= 0.0f && station.distance <= aprsAlertDistanceKm) {
    nearbyMobileMatched = isAprsMobileSsid7or9(station.callsign);
  }

  bool weatherStationMatched = aprsAlertWxEnabled && isAprsWxPayloadValidForAlert(station);

  if (!watchListMatched && !nearbyMobileMatched && !weatherStationMatched) {
    return false;
  }

  String normalized = sanitizeAprsCallsignToken(station.callsign);
  if (normalized.length() == 0) {
    return false;
  }

  const unsigned long minGapMs = (unsigned long)aprsAlertMinSeconds * 1000UL;
  const unsigned long nowMs = millis();

  for (int i = 0; i < MAX_APRS_STATIONS; i++) {
    if (aprsAlertCooldown[i].callsign.equals(normalized)) {
      if (minGapMs > 0 && (nowMs - aprsAlertCooldown[i].lastAlertMs) < minGapMs) {
        return false;
      }
      aprsAlertCooldown[i].lastAlertMs = nowMs;
      return true;
    }
  }

  for (int i = 0; i < MAX_APRS_STATIONS; i++) {
    if (aprsAlertCooldown[i].callsign.length() == 0) {
      aprsAlertCooldown[i].callsign = normalized;
      aprsAlertCooldown[i].lastAlertMs = nowMs;
      return true;
    }
  }

  int replaceIdx = aprsAlertCooldownReplaceIdx;
  if (replaceIdx < 0 || replaceIdx >= MAX_APRS_STATIONS) {
    replaceIdx = 0;
  }
  aprsAlertCooldown[replaceIdx].callsign = normalized;
  aprsAlertCooldown[replaceIdx].lastAlertMs = nowMs;
  aprsAlertCooldownReplaceIdx = (replaceIdx + 1) % MAX_APRS_STATIONS;
  return true;
}

static void applyAprsIntervalMinutes(int minutes) {
  int m = minutes;
  if (m < 1) m = 1;
  if (m > 180) m = 180; // sanity bound 1..180 min
  aprsIntervalMinutes = m;
  aprsPositionIntervalMs = (unsigned long)m * 60UL * 1000UL;
  nextAPRSPositionDueMs = 0; // przelicz harmonogram od nowa
}

bool isAprsTxCallValid(const String &call) {
  String c = call;
  c.trim();
  c.toUpperCase();
  if (c.length() == 0) return false;
  int dash = c.indexOf('-');
  if (dash > 0) {
    c = c.substring(0, dash);
  }
  return c.length() > 0 && c != "NOCALL";
}

void sendAprsPosition() {
  if (!aprsBeaconEnabled || !aprsConnected || !aprsClient.connected() || !aprsLoginSent) {
    return;
  }
  if (!userLatLonValid) {
    Serial.println("[APRS] Pomijam TX pozycji - brak poprawnych współrzędnych użytkownika");
    return;
  }

  String txCallsign = getAprsTxCallsignWithSsid();
  txCallsign.trim();
  if (!isAprsTxCallValid(txCallsign)) {
    Serial.println("[APRS] Pomijam TX pozycji - ustaw znak w konfiguracji APRS");
    return;
  }
  txCallsign.toUpperCase();

  String latStr = formatAprsCoordinate(userLat, true);
  String lonStr = formatAprsCoordinate(userLon, false);

  String frame = txCallsign;
  frame += ">APRS,TCPIP*:";
  frame += "!";
  frame += latStr;
  frame += aprsSymbolTable;
  frame += lonStr;
  frame += aprsSymbolCode;
  frame += " ";
  bool useProjectComment = (((aprsBeaconTxCount + 1) % 5) == 0); // co piąty beacon
  String comment = useProjectComment ? String(APRS_POSITION_COMMENT)
                                     : sanitizeAprsComment(aprsUserComment);
  frame += comment;

  aprsClient.println(frame);
  lastAPRSPositionTxMs = millis();
  aprsBeaconTxCount++;

  Serial.print("[APRS] Wysłano pozycję: ");
  Serial.println(frame);
}

// Parsuj ramkĂ„â„˘ APRS
// Format przykÄąâ€šadowy: SP3KON-1>APRS,TCPIP*,qAC,T2POLAND:!5202.40N/01655.12E#PHG5130/Poznan
bool parseAPRSFrame(String line, APRSStation &station) {
  LOGV_PRINTF("[APRS] Parsing frame: %s\n", line.c_str());
  
  // WyczyÄąâ€şĂ„â€ˇ strukturĂ„â„˘
  station.time = "";
  station.callsign = "";
  station.symbol = "";
  station.symbolTable = "";
  station.lat = 0.0f;
  station.lon = 0.0f;
  station.comment = "";
  station.freqMHz = 0.0f;
  station.distance = 0.0f;
  station.hasLatLon = false;
  
  // Pobierz czas UTC
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1)) {
    char timeBuffer[6];
    strftime(timeBuffer, 6, "%H%MZ", &timeinfo);
    station.time = String(timeBuffer);
  } else {
    station.time = "----Z";
  }
  
  // Parsuj callsign (przed >)
  int gtPos = line.indexOf('>');
  if (gtPos < 0) {
    LOGV_PRINTLN("[APRS] Brak znaku '>' w ramce - nieprawidÄąâ€šowy format");
    return false;
  }
  station.callsign = line.substring(0, gtPos);
  station.callsign.trim();
  LOGV_PRINTF("[APRS] Callsign: %s\n", station.callsign.c_str());

  // Najpierw sprÄ‚łbuj dekodowaĂ„â€ˇ Mic-E / skompresowane / obiekty z gotowego parsera
  bool advancedParsed = parseAprsAdvancedPosition(line, station);
  if (advancedParsed) {
    LOGV_PRINTF("[APRS] Advanced decode lat/lon: %.6f, %.6f\n", station.lat, station.lon);
  }
  
  // Parsuj pozycjĂ„â„˘ GPS (prosty parser) tylko jeÄąâ€şli advanced nie zadziaÄąâ€šaÄąâ€š
  // Szukaj formatu: !5202.40N/01655.12E, @5202.40N/01655.12E, =5202.40N/01655.12E, lub ;...*HHMMzDDMM.mmN/DDDMM.mmE
  int posStart = -1;
  char tableSymbol = 0;
  int commentBeforePos = -1; // Pozycja poczĂ„â€¦tku komentarza przed timestampem (dla formatu z ;)
  
  if (!advancedParsed) {
  // Szukaj standardowych formatÄ‚łw pozycji (!, @, =)
  posStart = line.indexOf('!');
  if (posStart >= 0) {
    tableSymbol = '!';
  } else {
    posStart = line.indexOf('@');
    if (posStart >= 0) {
      tableSymbol = '@';
    } else {
      posStart = line.indexOf('=');
      if (posStart >= 0) {
        tableSymbol = '=';
      }
    }
  }
  
  // JeÄąâ€şli nie znaleziono standardowych formatÄ‚łw, szukaj w komentarzu (format z ;)
  if (posStart < 0) {
    int colonPos = line.indexOf(':');
    if (colonPos >= 0) {
      String commentPart = line.substring(colonPos + 1);
      // Szukaj formatu z ; i timestampem: ;...*HHMMzDDMM.mmN/DDDMM.mmE
      int semicolonPos = commentPart.indexOf(';');
      if (semicolonPos >= 0) {
        // Zapisz pozycjĂ„â„˘ poczĂ„â€¦tku komentarza przed timestampem
        commentBeforePos = colonPos + 1 + semicolonPos + 1; // +1 bo colonPos, +1 bo po ';'
        // Szukaj timestampu *HHMMz
        int timestampPos = commentPart.indexOf('*', semicolonPos);
        if (timestampPos >= 0 && timestampPos + 7 < commentPart.length()) {
          // SprawdÄąĹź czy po timestampie jest pozycja (format: *HHMMzDDMM.mmN)
          char zChar = commentPart.charAt(timestampPos + 6);
          if (zChar == 'z' || zChar == 'Z') {
            // Pozycja zaczyna siĂ„â„˘ po 'z'
            posStart = colonPos + 1 + timestampPos + 7; // +1 bo colonPos, +timestampPos, +7 bo "*HHMMz"
            tableSymbol = ';';
          }
        }
      }
    }
  }
  
  if (posStart < 0) {
    // Brak pozycji - sprÄ‚łbuj wyciĂ„â€¦gnĂ„â€¦Ă„â€ˇ tylko callsign i komentarz
    LOGV_PRINTLN("[APRS] Brak pozycji GPS w ramce");
    int colonPos = line.indexOf(':');
    if (colonPos >= 0) {
      station.comment = line.substring(colonPos + 1);
      station.comment.trim();
    }
    return true; // ZwrÄ‚łĂ„â€ˇ true nawet bez pozycji
  }
  
  LOGV_PRINTF("[APRS] Znaleziono pozycjĂ„â„˘ na indeksie %d, symbol tabeli: %c\n", posStart, tableSymbol);
  
  // Symbol table ustawimy po wyciĂ„â€¦gniĂ„â„˘ciu pozycji
  
  // Wyznacz poczĂ„â€¦tek pozycji (lat) zaleÄąÄ˝nie od formatu
  bool posStartIsSymbol = (tableSymbol == '!' || tableSymbol == '@' || tableSymbol == '=' || tableSymbol == '/');
  int latStart = posStartIsSymbol ? (posStart + 1) : posStart; // dla ';' posStart wskazuje juÄąÄ˝ na lat

  // ObsÄąâ€šuga timestampu dla @ lub /: @DDHHMMh... lub /DDHHMMh...
  if (tableSymbol == '@' || tableSymbol == '/') {
    if (posStart + 7 < line.length()) {
      bool tsDigits = true;
      for (int i = 1; i <= 6; i++) {
        char c = line.charAt(posStart + i);
        if (!isDigit(c)) {
          tsDigits = false;
          break;
        }
      }
      char tsChar = line.charAt(posStart + 7);
      if (tsDigits && (tsChar == 'h' || tsChar == 'z' || tsChar == '/')) {
        latStart = posStart + 8;
      }
    }
  }

  // SprawdÄąĹź czy jest format z "/" czy bez "/"
  int slashPos = line.indexOf('/', latStart);
  bool hasSlash = (slashPos >= 0);
  
  bool latOk = false;
  bool lonOk = false;

  if (hasSlash) {
    // Format z "/": !DDMM.mmN/DDDMM.mmEsymbol
    // Parsuj szerokoÄąâ€şĂ„â€ˇ geograficznĂ„â€¦: 5202.40N (format: DDMM.mmN)
    if (latStart + 7 < line.length()) {
      String latStr = line.substring(latStart, latStart + 7);
      if (latStr.length() == 7) {
        char dir = line.charAt(latStart + 7);
        // UÄąÄ˝yj funkcji convertToDecimal do parsowania
        float parsedLat = convertToDecimal(latStr, dir);
        if (!isnan(parsedLat)) {
          station.lat = parsedLat;
          latOk = true;
        }
        
        LOGV_PRINTF("[APRS] Parsed lat: %s%c -> %.6f\n", 
                    latStr.c_str(), dir, station.lat);
      }
    }
    
    // Parsuj dÄąâ€šugoÄąâ€şĂ„â€ˇ geograficznĂ„â€¦: 01655.12E (format: DDDMM.mmE)
    if (slashPos >= 0 && slashPos + 9 < line.length()) {
      String lonStr = line.substring(slashPos + 1, slashPos + 9);
      if (lonStr.length() == 8) {
        char dir = line.charAt(slashPos + 9);
        // UÄąÄ˝yj funkcji convertToDecimal do parsowania
        float parsedLon = convertToDecimal(lonStr, dir);
        if (!isnan(parsedLon)) {
          station.lon = parsedLon;
          lonOk = true;
        }
        
        LOGV_PRINTF("[APRS] Parsed lon: %s%c -> %.6f\n", 
                    lonStr.c_str(), dir, station.lon);
      }
    }
    
    // Symbol table (znak przed /)
    if (slashPos >= 0) {
      station.symbolTable = line.substring(slashPos, slashPos + 1);
    }
    // WyciĂ„â€¦gnij symbol code (znak po dÄąâ€šugoÄąâ€şci geograficznej, po "/")
    // Format: !DDMM.mmN/DDDMM.mmEsymbol
    if (slashPos >= 0 && slashPos + 10 < line.length()) {
      station.symbol = line.substring(slashPos + 10, slashPos + 11);
    }
  } else {
    // Format bez "/": DDMM.mmNsymbolDDDMM.mmE (symbol miĂ„â„˘dzy lat a lon)
    // PrzykÄąâ€šad: 5223.73NW01655.41E lub 5225.05NL01651.66E
    if (latStart + 7 < line.length()) {
      String latStr = line.substring(latStart, latStart + 7);
      if (latStr.length() == 7) {
        char dir = line.charAt(latStart + 7);
        // UÄąÄ˝yj funkcji convertToDecimal do parsowania
        float parsedLat = convertToDecimal(latStr, dir);
        if (!isnan(parsedLat)) {
          station.lat = parsedLat;
          latOk = true;
        }
        
        LOGV_PRINTF("[APRS] Parsed lat: %s%c -> %.6f\n", 
                    latStr.c_str(), dir, station.lat);
        
        // Symbol table jest zaraz po kierunku lat
        if (latStart + 8 < line.length()) {
          station.symbolTable = line.substring(latStart + 8, latStart + 9);
        }
        
        // DÄąâ€šugoÄąâ€şĂ„â€ˇ geograficzna zaczyna siĂ„â„˘ po symbolu table
        int lonStart = latStart + 9;
        if (lonStart + 7 < line.length()) {
          String lonStr = line.substring(lonStart, lonStart + 8);
          if (lonStr.length() == 8) {
            char dir = line.charAt(lonStart + 8);
            // UÄąÄ˝yj funkcji convertToDecimal do parsowania
            float parsedLon = convertToDecimal(lonStr, dir);
            if (!isnan(parsedLon)) {
              station.lon = parsedLon;
              lonOk = true;
            }
            
            LOGV_PRINTF("[APRS] Parsed lon: %s%c -> %.6f\n", 
                        lonStr.c_str(), dir, station.lon);

            // Symbol code jest po kierunku lon (jeÄąâ€şli wystĂ„â„˘puje)
            if (lonStart + 9 < line.length()) {
              station.symbol = line.substring(lonStart + 9, lonStart + 10);
            }
          }
        }
      }
    }
  }
  
  // Ustaw flagĂ„â„˘ poprawnoÄąâ€şci pozycji tylko jeÄąâ€şli mamy lat i lon
  station.hasLatLon = (latOk && lonOk);

  // WyciĂ„â€¦gnij komentarz (po symbolu i pozycji)
  // Format z "/": !5202.40N/01655.12E#symbol/komentarz lub !5202.40N/01655.12E#symbol komentarz
  // Format bez "/": DDMM.mmNsymbolDDDMM.mmE&komentarz
  // Format z ";": ;komentarz_przed*HHMMzDDMM.mmNsymbolDDDMM.mmE&komentarz_po
  int colonPos = line.indexOf(':');
  if (colonPos >= 0 && colonPos + 1 < line.length()) {
    String fullComment = "";
    
    if (tableSymbol == ';' && commentBeforePos >= 0) {
      // Format z ";": komentarz skÄąâ€šada siĂ„â„˘ z czĂ„â„˘Äąâ€şci przed timestampem i po pozycji
      // CzĂ„â„˘Äąâ€şĂ„â€ˇ przed timestampem
      int timestampStart = line.indexOf('*', commentBeforePos);
      if (timestampStart > commentBeforePos) {
        fullComment = line.substring(commentBeforePos, timestampStart);
      }
      
      // CzĂ„â„˘Äąâ€şĂ„â€ˇ po pozycji
      int commentAfterPos = -1;
      if (hasSlash) {
        int afterSymbol = slashPos + 11;
        if (afterSymbol < line.length()) {
          char c = line.charAt(afterSymbol);
          if (c == '&' || c == '#' || c == '/') {
            commentAfterPos = afterSymbol + 1;
          } else {
            commentAfterPos = afterSymbol;
          }
        }
      } else {
        // Format bez "/": komentarz zaczyna siĂ„â„˘ po lon
        int afterLon = latStart + 18;
        if (afterLon < line.length()) {
          char c = line.charAt(afterLon);
          if (c == '&' || c == '#' || c == '/') {
            commentAfterPos = afterLon + 1;
          } else {
            commentAfterPos = afterLon;
          }
        }
      }
      
      if (commentAfterPos >= 0 && commentAfterPos < line.length()) {
        if (fullComment.length() > 0) {
          fullComment += " ";
        }
        fullComment += line.substring(commentAfterPos);
      }
      
      station.comment = fullComment;
      station.comment.trim();
    } else {
      // Standardowe formaty (!, @, =)
      int commentStart = colonPos + 1;
      
      if (hasSlash && slashPos >= 0) {
        // Format z "/": komentarz zaczyna siĂ„â„˘ po symbolu
        int afterSymbol = slashPos + 11;
        if (afterSymbol < line.length()) {
          char c = line.charAt(afterSymbol);
          if (c == '#' || c == '/') {
            commentStart = afterSymbol + 1;
          } else {
            // Komentarz zaczyna siĂ„â„˘ zaraz po symbolu
            commentStart = afterSymbol;
          }
        }
      } else {
        // Format bez "/": komentarz zaczyna siĂ„â„˘ po lon (po kierunku E/W)
        int afterLon = latStart + 18;
        if (afterLon < line.length()) {
          char c = line.charAt(afterLon);
          if (c == '&' || c == '#' || c == '/') {
            commentStart = afterLon + 1;
          } else {
            // Komentarz zaczyna siĂ„â„˘ zaraz po lon
            commentStart = afterLon;
          }
        }
      }
      
      // JeÄąâ€şli commentStart jest przed koÄąâ€žcem linii, wyciĂ„â€¦gnij komentarz
      if (commentStart < line.length() && commentStart >= colonPos + 1) {
        station.comment = line.substring(commentStart);
        station.comment.trim();
      } else if (commentStart < colonPos + 1) {
        // JeÄąâ€şli nie znaleziono komentarza po pozycji, uÄąÄ˝yj caÄąâ€šej czĂ„â„˘Äąâ€şci po ":"
        station.comment = line.substring(colonPos + 1);
        station.comment.trim();
      }
    }
  }
  } // !advancedParsed
  
  // SprÄ‚łbuj wyciĂ„â€¦gnĂ„â€¦Ă„â€ˇ czĂ„â„˘stotliwoÄąâ€şĂ„â€ˇ z komentarza
  if (station.comment.length() > 0) {
    float freq = 0.0f;
    if (extractAPRSFrequencyMHz(station.comment, freq)) {
      station.freqMHz = freq;
    }
  }
  
  // Oblicz odlegÄąâ€šoÄąâ€şĂ„â€ˇ jeÄąâ€şli mamy pozycjĂ„â„˘
  // UÄąÄ˝ywamy wspÄ‚łÄąâ€šrzĂ„â„˘dnych z sekcji "Moja Stacja"
  if (station.hasLatLon) {
    // UÄąÄ˝yj double dla wiĂ„â„˘kszej precyzji obliczeÄąâ€ž
    double userLatForDistance = userLatLonValid ? userLat : 52.40;  // Fallback jeÄąâ€şli nie ustawione
    double userLonForDistance = userLatLonValid ? userLon : 16.92;
    double stationLat = (double)station.lat;
    double stationLon = (double)station.lon;
    
    station.distance = calculateDistance(userLatForDistance, userLonForDistance, 
                                         stationLat, stationLon);
    LOGV_PRINTF("[APRS] Distance: user(%.6f,%.6f) -> station(%.6f,%.6f) = %.1f km\n",
                userLatForDistance, userLonForDistance, stationLat, stationLon, station.distance);
  }
  
  return true;
}

// Dodaj nowĂ„â€¦ stacjĂ„â„˘ APRS do tablicy (bez duplikatÄ‚łw - aktualizuje istniejĂ„â€¦ce)
void addAPRSStation(APRSStation station) {
  compactAprsStationStrings(station);
  LOGV_PRINT("[APRS] addAPRSStation: ");
  LOGV_PRINT(station.callsign);
  if (station.hasLatLon) {
    LOGV_PRINT(" @ ");
    if (LOG_VERBOSE) Serial.print(station.lat, 4);
    LOGV_PRINT(",");
    if (LOG_VERBOSE) Serial.print(station.lon, 4);
    LOGV_PRINT(" (");
    if (LOG_VERBOSE) Serial.print(station.distance, 1);
    LOGV_PRINT(" km)");
  }
  LOGV_PRINTLN();
  
  // SprawdÄąĹź czy stacja juÄąÄ˝ istnieje (po callsign)
  int existingIndex = -1;
  for (int i = 0; i < aprsStationCount; i++) {
    if (aprsStations[i].callsign.equalsIgnoreCase(station.callsign)) {
      existingIndex = i;
      break;
    }
  }
  
  if (existingIndex >= 0) {
    // Stacja juÄąÄ˝ istnieje - usuÄąâ€ž jĂ„â€¦ z obecnej pozycji
    for (int i = existingIndex; i > 0; i--) {
      aprsStations[i] = aprsStations[i - 1];
    }
    // Zaktualizuj dane stacji (nowy czas, pozycja, itp.)
    aprsStations[0] = station;
    LOGV_PRINT("[APRS] Stacja juÄąÄ˝ istnieje - zaktualizowano na pozycji 0");
  } else {
    // Nowa stacja - dodaj na poczĂ„â€¦tku
    if (aprsStationCount < MAX_APRS_STATIONS) {
      aprsStationCount++;
    }
    
    // PrzesuÄąâ€ž wszystkie stacje o jeden w dÄ‚łÄąâ€š
    for (int i = MAX_APRS_STATIONS - 1; i > 0; i--) {
      aprsStations[i] = aprsStations[i - 1];
    }
    
    // Dodaj nowĂ„â€¦ stacjĂ„â„˘ na poczĂ„â€¦tku
    aprsStations[0] = station;
    LOGV_PRINT("[APRS] Dodano nowĂ„â€¦ stacjĂ„â„˘");
  }
  
  LOGV_PRINT("[APRS] addAPRSStation END, nowy aprsStationCount=");
  LOGV_PRINTLN(aprsStationCount);
}

// ObsÄąâ€šuga danych z APRS-IS
void handleAPRSData() {
  if (!aprsConnected || !aprsClient.connected()) {
    if (aprsConnected) {
      LOGV_PRINTLN("[APRS] PoÄąâ€šĂ„â€¦czenie zerwane - reset flagÄ‚łw");
    }
    aprsConnected = false;
    aprsLoginSent = false;
    return;
  }
  
  // Odczytaj dostĂ„â„˘pne dane
  while (aprsClient.available()) {
    unsigned char c = (unsigned char)aprsClient.read();
    lastAPRSRxMs = millis();
    
    if (c == '\n' || c == '\r') {
      if (aprsBuffer.length() > 0) {
        String line = aprsBuffer;
        aprsBuffer = "";
        
        // Ignoruj linie zaczynajĂ„â€¦ce siĂ„â„˘ od # (komentarze serwera)
        if (line.startsWith("#")) {
          Serial.print("[APRS] Server: ");
          Serial.println(line);
          continue;
        }
        
        // Parsuj ramkĂ„â„˘ APRS
        APRSStation station;
        if (parseAPRSFrame(line, station)) {
          addAPRSStation(station);

          if (aprsAlertEnabled && shouldTriggerAprsAlert(station)) {
            Serial.print("[APRS ALERT] stacja wykryta opisana w formacie tnc: ");
            Serial.println(line);
            triggerAprsRgbLedAlert();
            #ifdef ENABLE_TFT_DISPLAY
            portENTER_CRITICAL(&aprsAlertPendingMux);
            aprsAlertPendingStation = station;
            aprsAlertDrawPending = true;
            portEXIT_CRITICAL(&aprsAlertPendingMux);
            #endif
          }
        }
      }
    } else if (c != 0) {
      // Keep control bytes (e.g. 0x1C/0x1D) for Mic-E decode compatibility.
      aprsBuffer += (char)c;
      // Ograniczenie dÄąâ€šugoÄąâ€şci bufora (zapobieganie przepeÄąâ€šnieniu)
      if (aprsBuffer.length() > 512) {
        aprsBuffer = aprsBuffer.substring(aprsBuffer.length() - 256);
      }
    }
  }
}

// ========== WIFI MANAGER ==========

void startAPMode() {
  // Stabilny portal konfiguracyjny: sam AP (bez STA w tle),
  // ÄąÄ˝eby nie gubiĂ„â€ˇ poÄąâ€šĂ„â€¦czenia na telefonie/PC przy zmianie kanaÄąâ€šu przez STA.
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.println("AP Mode uruchomiony");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  
  // Aktualizuj wyÄąâ€şwietlacz TFT z IP AP (jeÄąâ€şli jesteÄąâ€şmy na ekranie 1)
#ifdef ENABLE_TFT_DISPLAY
  updateScreen1();
#endif
}

void requestRestart(unsigned long delayMs = 1500) {
  restartRequested = true;
  restartAtMs = millis() + delayMs;
}

bool connectToWiFi() {
  if (wifiSSID.length() == 0 && wifiSSID2.length() == 0) {
    return false;
  }

  struct WiFiCred {
    String ssid;
    String pass;
  };

  WiFiCred candidates[2] = {{wifiSSID, wifiPassword}, {wifiSSID2, wifiPassword2}};

  auto attemptConnect = [](const WiFiCred &cred) -> bool {
    Serial.println("=== WiFi connect ===");
    Serial.print("SSID: '");
    Serial.print(cred.ssid);
    Serial.print("' (len=");
    Serial.print(cred.ssid.length());
    Serial.println(")");
    Serial.print("PASS len=");
    Serial.println(cred.pass.length());

    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(250);

    WiFi.mode(WIFI_STA);
    delay(50);
    WiFi.begin(cred.ssid.c_str(), cred.pass.c_str());

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
      delay(500);
      yield();
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("");
      return true;
    }

    Serial.println("");
    Serial.println("BÄąâ€šĂ„â€¦d poÄąâ€šĂ„â€¦czenia WiFi");
    Serial.print("WiFi.status(): ");
    Serial.println((int)WiFi.status());

    Serial.println("--- WiFi diag ---");
    WiFi.printDiag(Serial);
    Serial.println("--- Scan networks ---");
    int n = WiFi.scanNetworks(/*async=*/false, /*show_hidden=*/true);
    Serial.print("Znaleziono sieci: ");
    Serial.println(n);
    bool found = false;
    for (int i = 0; i < n; i++) {
      String s = WiFi.SSID(i);
      int32_t rssi = WiFi.RSSI(i);
      wifi_auth_mode_t enc = WiFi.encryptionType(i);
      if (s == cred.ssid) {
        found = true;
        Serial.print("MATCH SSID: ");
        Serial.print(s);
        Serial.print(" RSSI=");
        Serial.print(rssi);
        Serial.print(" enc=");
        Serial.println((int)enc);
      }
    }
    if (!found) {
      Serial.println("UWAGA: Nie widzĂ„â„˘ Twojego SSID w skanie (moÄąÄ˝e 5GHz / inny SSID / poza zasiĂ„â„˘giem / ukryte SSID).");
    }

    return false;
  };

  for (int i = 0; i < 2; i++) {
    const WiFiCred &cred = candidates[i];
    if (cred.ssid.length() == 0) {
      continue;
    }

    if (i == 1 && cred.ssid == candidates[0].ssid) {
      continue; // avoid duplicate attempt
    }

    bool ok = attemptConnect(cred);
    if (ok) {
      wifiConnected = true;
      Serial.print("PoÄąâ€šĂ„â€¦czono z SSID: ");
      Serial.println(cred.ssid);
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());

      // Aktualizuj wyÄąâ€şwietlacz TFT z IP STA (jeÄąâ€şli jesteÄąâ€şmy na ekranie 1)
#ifdef ENABLE_TFT_DISPLAY
      updateScreen1();
#endif

      return true;
    } else {
      Serial.println("PrÄ‚łba poÄąâ€šĂ„â€¦czenia nieudana dla SSID: " + cred.ssid);
    }
  }

  wifiConnected = false;
  return false;
}

// ========== TELNET CLUSTER ==========

void connectToCluster() {
  Serial.println("[CLUSTER] connectToCluster() START");
  
  if (!wifiConnected) {
    Serial.println("[CLUSTER] WiFi nie poÄąâ€šĂ„â€¦czony - wyjÄąâ€şcie");
    return;
  }
  
  if (telnetClient.connected()) {
    Serial.println("[CLUSTER] JuÄąÄ˝ poÄąâ€šĂ„â€¦czony - wyjÄąâ€şcie");
    return;
  }
  
  unsigned long now = millis();
  if (now - lastTelnetAttempt < 5000) {
    return; // Cichy return - nie spamuj logu (to byÄąâ€šo gÄąâ€šÄ‚łwne ÄąĹźrÄ‚łdÄąâ€šo spamu!)
  }
  
  lastTelnetAttempt = now;
  
  Serial.print("[CLUSTER] ÄąÂĂ„â€¦czenie z DX Cluster: ");
  Serial.print(clusterHost);
  Serial.print(":");
  Serial.println(clusterPort);

  // Diagnostyka DNS (czĂ„â„˘sty problem gdy host nie rozwiĂ„â€¦zuje siĂ„â„˘ na ESP32)
  IPAddress resolvedIp;
  if (WiFi.hostByName(clusterHost.c_str(), resolvedIp)) {
    Serial.print("DNS OK: ");
    Serial.print(clusterHost);
    Serial.print(" -> ");
    Serial.println(resolvedIp);
  } else {
    Serial.print("DNS FAIL dla hosta: ");
    Serial.println(clusterHost);
  }
  
  Serial.println("[CLUSTER] WywoÄąâ€šanie telnetClient.connect()...");
  unsigned long connectStart = millis();
  if (telnetClient.connect(clusterHost.c_str(), clusterPort)) {
    unsigned long connectTime = millis() - connectStart;
    Serial.print("[CLUSTER] PoÄąâ€šĂ„â€¦czono z DX Cluster! (czas: ");
    Serial.print(connectTime);
    Serial.println("ms)");
    telnetConnected = true;
    telnetBuffer = "";
    clusterLoginSent = false;
    clusterLoginScheduled = false;
    lastClusterKeepAliveMs = millis();
    lastTelnetRxMs = millis();

    // Zawsze wysyÄąâ€šamy login po poÄąâ€šĂ„â€¦czeniu (tak jak w projekcie referencyjnym).
    // JeÄąâ€şli uÄąÄ˝ytkownik nie ustawiÄąâ€š znaku, uÄąÄ˝ywamy domyÄąâ€şlnego (DEFAULT_CALLSIGN).
    clusterLoginScheduled = true;
    clusterSendLoginAtMs = millis() + 1000;
  } else {
    unsigned long connectTime = millis() - connectStart;
    Serial.print("[CLUSTER] BÄąâ€šĂ„â€¦d poÄąâ€šĂ„â€¦czenia z DX Cluster (czas: ");
    Serial.print(connectTime);
    Serial.println("ms)");
    telnetConnected = false;
  }
  Serial.println("[CLUSTER] connectToCluster() END");
}

void connectToPotaCluster() {
  if (!wifiConnected) {
    return;
  }
  if (potaClusterHost.length() == 0 || potaClusterPort <= 0) {
    return;
  }
  if (potaTelnetClient.connected()) {
    return;
  }

  unsigned long now = millis();
  if (now - lastPotaAttempt < 5000) {
    return;
  }
  lastPotaAttempt = now;

  Serial.print("[POTA] ÄąÂĂ„â€¦czenie z POTA Cluster: ");
  Serial.print(potaClusterHost);
  Serial.print(":");
  Serial.println(potaClusterPort);

  IPAddress resolvedIp;
  if (WiFi.hostByName(potaClusterHost.c_str(), resolvedIp)) {
    Serial.print("[POTA] DNS OK: ");
    Serial.print(potaClusterHost);
    Serial.print(" -> ");
    Serial.println(resolvedIp);
  } else {
    Serial.print("[POTA] DNS FAIL: ");
    Serial.println(potaClusterHost);
  }

  if (potaTelnetClient.connect(potaClusterHost.c_str(), potaClusterPort)) {
    Serial.println("[POTA] PoÄąâ€šĂ„â€¦czono z POTA Cluster!");
    potaTelnetConnected = true;
    potaTelnetBuffer = "";
    pendingPotaLine = "";
    potaLoginSent = false;
    potaLoginScheduled = false;
    lastPotaKeepAliveMs = millis();
    lastPotaRxMs = millis();

    String login = userCallsign;
    login.trim();
    if (login.length() == 0) {
      login = DEFAULT_CALLSIGN;
    }
    if (userLocator.length() >= 4) {
      login += "/";
      login += userLocator;
    }
    potaTelnetClient.print(login);
    potaTelnetClient.print("\r\n");
    potaLoginSent = true;
    lastPotaKeepAliveMs = millis();

    if (potaFilterCommand.length() > 0) {
      potaTelnetClient.print(potaFilterCommand);
      potaTelnetClient.print("\r\n");
    }
  } else {
    Serial.println("[POTA] BÄąâ€šĂ„â€¦d poÄąâ€šĂ„â€¦czenia z POTA Cluster");
    potaTelnetConnected = false;
  }
}

// WyÄąâ€şlij komendĂ„â„˘ konfiguracyjnĂ„â€¦ do DX Cluster (CC-Cluster)
// UWAGA: UÄąÄ˝ywane TYLKO do konfiguracji odbioru (set/noann, set/nowwv, set/filter, etc.)
// NIE wysyÄąâ€ša ÄąÄ˝adnych spotÄ‚łw - urzĂ„â€¦dzenie dziaÄąâ€ša tylko w trybie odbioru
void sendClusterCommand(String command) {
  if (!telnetConnected || !telnetClient.connected()) {
    Serial.print("[CLUSTER] Nie moÄąÄ˝na wysÄąâ€šaĂ„â€ˇ komendy '");
    Serial.print(command);
    Serial.println("' - brak poÄąâ€šĂ„â€¦czenia");
    return;
  }
  
  Serial.print("[CLUSTER] WysyÄąâ€šanie komendy: ");
  Serial.println(command);
  telnetClient.print(command);
  telnetClient.print("\r\n");
  delay(50); // KrÄ‚łtkie opÄ‚łÄąĹźnienie miĂ„â„˘dzy komendami
}

// WyÄąâ€şlij komendy konfiguracyjne CC-Cluster po zalogowaniu
void sendClusterConfigCommands() {
  if (!telnetConnected || !telnetClient.connected()) {
    return;
  }
  
  Serial.println("[CLUSTER] WysyÄąâ€šanie komend konfiguracyjnych CC-Cluster...");
  
  // WyÄąâ€šĂ„â€¦cz ogÄąâ€šoszenia (domyÄąâ€şlnie wÄąâ€šĂ„â€¦czone)
  if (clusterNoAnnouncements) {
    sendClusterCommand("set/noann");
  }
  
  // WyÄąâ€šĂ„â€¦cz WWV (domyÄąâ€şlnie wÄąâ€šĂ„â€¦czone)
  if (clusterNoWWV) {
    sendClusterCommand("set/nowwv");
  }
  
  // WyÄąâ€šĂ„â€¦cz WCY (domyÄąâ€şlnie wÄąâ€šĂ„â€¦czone)
  if (clusterNoWCY) {
    sendClusterCommand("set/nowcy");
  }
  
  // Ustaw filtry (jeÄąâ€şli wÄąâ€šĂ„â€¦czone)
  if (clusterUseFilters && clusterFilterCommands.length() > 0) {
    // Parsuj i wyÄąâ€şlij komendy filtrÄ‚łw (moÄąÄ˝e byĂ„â€ˇ kilka linii oddzielonych \n)
    String filters = clusterFilterCommands;
    int pos = 0;
    while (pos < filters.length()) {
      int nextPos = filters.indexOf('\n', pos);
      if (nextPos < 0) {
        nextPos = filters.length();
      }
      String cmd = filters.substring(pos, nextPos);
      cmd.trim();
      if (cmd.length() > 0) {
        sendClusterCommand(cmd);
      }
      pos = nextPos + 1;
    }
  } else {
    // JeÄąâ€şli filtry wyÄąâ€šĂ„â€¦czone, wyczyÄąâ€şĂ„â€ˇ wszystkie filtry
    sendClusterCommand("set/nofilter");
  }
  
  Serial.println("[CLUSTER] Komendy konfiguracyjne wysÄąâ€šane");
}

void handleTelnetData() {
  static unsigned long lastTelnetPrint = 0;
  static int telnetCallCount = 0;
  telnetCallCount++;
  
  if (!telnetConnected || !telnetClient.connected()) {
    if (telnetConnected) {
      LOGV_PRINTLN("[TELNET] PoÄąâ€šĂ„â€¦czenie zerwane - reset flagÄ‚łw");
    }
    telnetConnected = false;
    clusterLoginSent = false;
    clusterLoginScheduled = false;
    return;
  }
  
  // Print co 1000 wywoÄąâ€šaÄąâ€ž (dla debugowania)
  unsigned long now = millis();
  if (now - lastTelnetPrint > 30000) { // Co 30 sekund
    LOGV_PRINT("[TELNET] handleTelnetData wywoÄąâ€šane ");
    LOGV_PRINT(telnetCallCount);
    LOGV_PRINT(" razy, available=");
    LOGV_PRINTLN(telnetClient.available());
    lastTelnetPrint = now;
    telnetCallCount = 0;
  }
  
  // Nie blokuj pĂ„â„˘tli gÄąâ€šÄ‚łwnej: w jednej iteracji czytaj maksymalnie N bajtÄ‚łw
  // i dawaj yield, ÄąÄ˝eby nie wpaÄąâ€şĂ„â€ˇ w WDT przy duÄąÄ˝ym strumieniu.
  int processed = 0;
  const int maxPerLoop = 256;
  int availableBefore = telnetClient.available();
  
  while (telnetClient.available() && processed < maxPerLoop) {
    char c = telnetClient.read();
    processed++;
    lastTelnetRxMs = millis();
    if ((processed % 64) == 0) {
      yield();
      // obsÄąâ€šuÄąÄ˝ WWW nawet jeÄąâ€şli telnet zalewa danymi
      if (server != nullptr) {
        server->handleClient();
      }
    }
    
    if (c == '\n' || c == '\r') {
      if (telnetBuffer.length() > 0) {
        // Nie parsuj tu (to bywa kosztowne). PrzekaÄąÄ˝ liniĂ„â„˘ do przetworzenia w loop().
        if (pendingTelnetLine.length() == 0) {
          pendingTelnetLine = telnetBuffer;
        } else {
          pendingTelnetDropped++;
        }
        telnetBuffer = "";
      }
    } else if (c >= 32 && c < 127) {
      telnetBuffer += c;
      if (telnetBuffer.length() > 512) {
        telnetBuffer = ""; // Ochrona przed przepeÄąâ€šnieniem
      }
    }
  }
  
  if (processed > 0) {
    LOGV_PRINT("[TELNET] Przetworzono ");
    LOGV_PRINT(processed);
    LOGV_PRINT(" bajtÄ‚łw (byÄąâ€šo ");
    LOGV_PRINT(availableBefore);
    LOGV_PRINT(", zostaÄąâ€šo ");
    LOGV_PRINT(telnetClient.available());
    LOGV_PRINTLN(")");
  }
  
  if (telnetClient.available() && processed >= maxPerLoop) {
    LOGV_PRINTLN("[TELNET] WARNING: ZostaÄąâ€šo wiĂ„â„˘cej danych - nastĂ„â„˘pna iteracja");
    // Zostaw resztĂ„â„˘ na nastĂ„â„˘pnĂ„â€¦ iteracjĂ„â„˘ loop()
    yield();
  }
  
  // SprawdÄąĹź czy poÄąâ€šĂ„â€¦czenie nadal dziaÄąâ€ša
  if (!telnetClient.connected()) {
    LOGV_PRINTLN("[TELNET] RozÄąâ€šĂ„â€¦czono z DX Cluster");
    telnetConnected = false;
    clusterLoginSent = false;
    clusterLoginScheduled = false;
  }
}

void handlePotaTelnetData() {
  if (!potaTelnetConnected || !potaTelnetClient.connected()) {
    if (potaTelnetConnected) {
      LOGV_PRINTLN("[POTA] PoÄąâ€šĂ„â€¦czenie zerwane - reset flag");
    }
    potaTelnetConnected = false;
    potaLoginSent = false;
    potaLoginScheduled = false;
    return;
  }

  int processed = 0;
  const int maxPerLoop = 256;
  while (potaTelnetClient.available() && processed < maxPerLoop) {
    char c = potaTelnetClient.read();
    processed++;
    lastPotaRxMs = millis();
    if ((processed % 64) == 0) {
      yield();
      if (server != nullptr) {
        server->handleClient();
      }
    }

    if (c == '\n' || c == '\r') {
      if (potaTelnetBuffer.length() > 0) {
        if (pendingPotaLine.length() == 0) {
          pendingPotaLine = potaTelnetBuffer;
        } else {
          pendingPotaDropped++;
        }
        potaTelnetBuffer = "";
      }
    } else if (c >= 32 && c < 127) {
      potaTelnetBuffer += c;
      if (potaTelnetBuffer.length() > 512) {
        potaTelnetBuffer = "";
      }
    }
  }

  if (!potaTelnetClient.connected()) {
    LOGV_PRINTLN("[POTA] RozÄąâ€šĂ„â€¦czono z POTA Cluster");
    potaTelnetConnected = false;
    potaLoginSent = false;
    potaLoginScheduled = false;
  }
}

// ========== NTP TIME ==========

void updateNTPTime() {
  unsigned long now = millis();
  if (lastNTPUpdate != 0 && (now - lastNTPUpdate < 3600000)) { // Aktualizuj co godzinĂ„â„˘
    return;
  }
  
  if (!wifiConnected) {
    return;
  }
  
  Serial.println("[NTP] Aktualizacja czasu NTP...");
  configTime(GMT_OFFSET_SEC, 0, NTP_SERVER);
  lastNTPUpdate = now;
  
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.println("[NTP] Czas NTP zsynchronizowany");
  } else {
    Serial.println("[NTP] BÄąÂĂ„â€žD: Nie udaÄąâ€šo siĂ„â„˘ pobraĂ„â€ˇ czasu NTP");
  }
  checkScreenSaverTimeout(); // Dodanie wywoÄąâ€šania funkcji
}

// ========== WYGASZACZ EKRANU (Matrix) ==========

void resetScreenSaverActivity() {
  screenSaverLastActivityMs = millis();
  if (screenSaverActive) {
    // WyÄąâ€šĂ„â€¦cz wygaszacz i wrÄ‚łĂ„â€ˇ do poprzedniego ekranu
    screenSaverActive = false;
    if (screenSaverPrevScreen != SCREEN_OFF) {
      drawScreen(screenSaverPrevScreen);
    }
  }
}

void checkScreenSaverTimeout() {
  if (!screenSaverEnabled || screenSaverActive) {
    return;
  }
  unsigned long now = millis();
  unsigned long timeoutMs = (unsigned long)screenSaverTimeoutMin * 60000UL; // minuty na ms
  if (now - screenSaverLastActivityMs >= timeoutMs) {
    // Włącz wygaszacz z poruszającym się zegarem
    screenSaverActive = true;
    screenSaverPrevScreen = currentScreen;
    // Wyczyść ekran i zresetuj pozycję zegara
    tft.fillScreen(TFT_BLACK);
    bounceClockX = 240;
    bounceClockY = 160;
    bounceClockVX = 2.5;
    bounceClockVY = 2.0;
    bounceClockColor = TFT_RADIO_ORANGE;
    lastBounceClockText = "";
  }
}

void setScreenSaverEnabled(bool enabled) {
  screenSaverEnabled = enabled;
  savePreferences();
}

void setScreenSaverTimeout(int minutes) {
  if (minutes < 1) minutes = 1;
  if (minutes > 60) minutes = 60;
  screenSaverTimeoutMin = minutes;
  savePreferences();
}

void drawQrzPopup() {
  Serial.print("[QRZ POPUP] drawQrzPopup called, active=");
  Serial.print(qrzPopupActive ? "true" : "false");
  Serial.print(" hasLatLon=");
  Serial.print(qrzPopupHasLatLon ? "true" : "false");
  Serial.print(" lat=");
  Serial.print(qrzPopupLat, 6);
  Serial.print(" lon=");
  Serial.println(qrzPopupLon, 6);
  if (!qrzPopupActive) return;

  const int popupW = 400;
  const int popupH = 220;
  const int popupX = (480 - popupW) / 2;
  const int popupY = (320 - popupH) / 2;

  // Tło popupu
  tft.fillRect(popupX, popupY, popupW, popupH, TFT_DARKGREY);
  tft.drawRect(popupX, popupY, popupW, popupH, TFT_WHITE);
  tft.drawRect(popupX + 2, popupY + 2, popupW - 4, popupH - 4, TFT_RADIO_ORANGE);

  // Nagłówek z callsign i imieniem
  tft.fillRect(popupX + 4, popupY + 4, popupW - 8, 30, TFT_RADIO_ORANGE);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(popupX + 10, popupY + 10);
  tft.print(qrzPopupCallsign);
  
  // Imię i nazwisko w nagłówku (jeśli dostępne)
  if (qrzPopupName.length() > 0) {
    tft.setTextSize(1);
    String nameText = " - " + qrzPopupName;
    // Oblicz pozycję za callsign
    int nameX = popupX + 10 + (qrzPopupCallsign.length() * 12);
    tft.setCursor(nameX, popupY + 14);
    tft.print(nameText);
  }

  // Dane
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  int y = popupY + 45;

  if (qrzPopupName.length() > 0) {
    tft.setCursor(popupX + 10, y);
    tft.print("Name: ");
    tft.print(qrzPopupName);
    y += 20;
  }

  if (qrzPopupGrid.length() > 0) {
    tft.setCursor(popupX + 10, y);
    tft.print("Grid: ");
    tft.print(qrzPopupGrid);
    y += 20;
  }

  if (qrzPopupCountry.length() > 0) {
    tft.setCursor(popupX + 10, y);
    tft.print("Country: ");
    tft.print(qrzPopupCountry);
    y += 20;
  }

  if (qrzPopupQth.length() > 0) {
    tft.setCursor(popupX + 10, y);
    tft.print("QTH: ");
    tft.print(qrzPopupQth);
    y += 20;
  }

  if (qrzPopupEmail.length() > 0) {
    tft.setCursor(popupX + 10, y);
    tft.print("Email: ");
    tft.print(qrzPopupEmail);
    y += 20;
  }

  if (qrzPopupDistance > 0.0) {
    tft.setCursor(popupX + 10, y);
    tft.print("Distance: ");
    tft.print(qrzPopupDistance, 0);
    tft.print(" km");
    y += 20;
  }

  // Przycisk zamknij
  const int closeW = 80;
  const int closeH = 30;
  const int closeX = popupX + (popupW - closeW) / 2;
  const int closeY = popupY + popupH - closeH - 15;
  drawFilterTile(closeX, closeY, closeW, closeH, "CLOSE", false);

  // Mapa świata w prawym górnym rogu popupu
  if (qrzPopupHasLatLon) {
    const int mapW = 140;
    const int mapH = 80;
    const int mapX = popupX + popupW - mapW - 10;
    const int mapY = popupY + 45;
    
    Serial.print("[QRZ MAP] Drawing map, hasLatLon=true, lat=");
    Serial.print(qrzPopupLat, 6);
    Serial.print(" lon=");
    Serial.println(qrzPopupLon, 6);
    
    // Wyświetl plik mapa.bmp z folderu data
    bool mapLoaded = false;
    if (littleFsReady && LittleFS.exists("/mapa.bmp")) {
      Serial.println("[QRZ MAP] Found /mapa.bmp, attempting to load...");
      mapLoaded = drawBmp16FromFS("/mapa.bmp", mapX, mapY);
      Serial.print("[QRZ MAP] BMP load result: ");
      Serial.println(mapLoaded ? "SUCCESS" : "FAILED");
    } else {
      Serial.println("[QRZ MAP] /mapa.bmp not found in LittleFS");
    }
    if (!mapLoaded) {
      Serial.println("[QRZ MAP] Using fallback drawing");
    }

    // Jeśli nie ma pliku BMP, użyj fallback (rysowanie)
    if (!mapLoaded) {
      // Tło mapy
      tft.fillRect(mapX, mapY, mapW, mapH, 0x4208); // Ciemny szary
      tft.drawRect(mapX, mapY, mapW, mapH, TFT_LIGHTGREY);
      
      // Uproszczony kontur mapy świata (linie kontynentów)
      tft.drawLine(mapX + 20, mapY + 25, mapX + 50, mapY + 20, TFT_LIGHTGREY);
      tft.drawLine(mapX + 50, mapY + 20, mapX + 60, mapY + 40, TFT_LIGHTGREY);
      tft.drawLine(mapX + 20, mapY + 25, mapX + 25, mapY + 55, TFT_LIGHTGREY);
      tft.drawLine(mapX + 25, mapY + 55, mapX + 40, mapY + 70, TFT_LIGHTGREY);
      tft.drawLine(mapX + 70, mapY + 15, mapX + 110, mapY + 20, TFT_LIGHTGREY);
      tft.drawLine(mapX + 110, mapY + 20, mapX + 120, mapY + 45, TFT_LIGHTGREY);
      tft.drawLine(mapX + 70, mapY + 35, mapX + 100, mapY + 40, TFT_LIGHTGREY);
      tft.drawLine(mapX + 100, mapY + 40, mapX + 95, mapY + 65, TFT_LIGHTGREY);
      tft.drawLine(mapX + 110, mapY + 55, mapX + 130, mapY + 60, TFT_LIGHTGREY);
      
      // Linie siatki
      for (int i = 1; i < 4; i++) {
        tft.drawFastHLine(mapX + 2, mapY + i * 20, mapW - 4, 0x2104);
      }
      for (int i = 1; i < 6; i++) {
        tft.drawFastVLine(mapX + i * 23, mapY + 2, mapH - 4, 0x2104);
      }
    }
    
    // Oblicz pozycję na mapie (prosta projekcja equirectangular)
    // Longitude: -180 do 180 -> 0 do mapW
    // Latitude: -90 do 90 -> mapH do 0 (odwrócone)
    int dotX = mapX + (int)((qrzPopupLon + 180.0) * mapW / 360.0);
    int dotY = mapY + mapH - (int)((qrzPopupLat + 90.0) * mapH / 180.0);
    
    Serial.print("[QRZ MAP] Raw dot position: dotX=");
    Serial.print(dotX);
    Serial.print(" dotY=");
    Serial.println(dotY);
    
    // Ogranicz do obszaru mapy
    if (dotX < mapX + 2) dotX = mapX + 2;
    if (dotX > mapX + mapW - 3) dotX = mapX + mapW - 3;
    if (dotY < mapY + 2) dotY = mapY + 2;
    if (dotY > mapY + mapH - 3) dotY = mapY + mapH - 3;
    
    Serial.print("[QRZ MAP] Clamped dot position: dotX=");
    Serial.print(dotX);
    Serial.print(" dotY=");
    Serial.println(dotY);
    
    // Rysuj punkt stacji (czerwona kropka z białą obwódką)
    Serial.println("[QRZ MAP] Drawing station dot");
    tft.fillCircle(dotX, dotY, 5, TFT_RED);
    tft.drawCircle(dotX, dotY, 5, TFT_WHITE);
    tft.drawCircle(dotX, dotY, 6, TFT_BLACK);
    
    // Mała etykieta z lokalizatorem pod mapą
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(mapX, mapY + mapH + 3);
    tft.print(qrzPopupGrid);
  }
}

void handleQrzPopupTouch(uint16_t x, uint16_t y) {
  if (!qrzPopupActive) return;

  const int popupW = 400;
  const int popupH = 220;
  const int popupX = (480 - popupW) / 2;
  const int popupY = (320 - popupH) / 2;
  const int closeW = 80;
  const int closeH = 30;
  const int closeX = popupX + (popupW - closeW) / 2;
  const int closeY = popupY + popupH - closeH - 15;

  if (isPointInRect(x, y, closeX, closeY, closeW, closeH)) {
    qrzPopupActive = false;
    // Przerysuj ekran
    if (currentScreen == SCREEN_DX_CLUSTER || currentScreen == SCREEN_POTA_CLUSTER || currentScreen == SCREEN_HAMALERT_CLUSTER) {
      drawScreen(currentScreen);
    }
  }
}

void openQrzPopup(const String &callsign) {
  Serial.print("[QRZ POPUP] Opening popup for: ");
  Serial.println(callsign);
  if (callsign.length() == 0) return;
  
  qrzPopupCallsign = callsign;
  qrzPopupCallsign.toUpperCase();

  // Sprawdź cache najpierw
  String grid, country, name, email, qth;
  double lat, lon;
  bool hasLatLon;
  if (getQrzCacheFresh(qrzPopupCallsign, grid, country, name, email, qth, lat, lon, hasLatLon, QRZ_CACHE_TTL_MS)) {
    Serial.print("[QRZ POPUP] Cache hit - hasLatLon=");
    Serial.print(hasLatLon ? "true" : "false");
    Serial.print(" lat=");
    Serial.print(lat, 6);
    Serial.print(" lon=");
    Serial.println(lon, 6);
    qrzPopupGrid = grid;
    qrzPopupCountry = country;
    qrzPopupName = name;
    qrzPopupEmail = email;
    qrzPopupQth = qth;
    qrzPopupLat = lat;
    qrzPopupLon = lon;
    qrzPopupHasLatLon = hasLatLon;
    // Oblicz odległość jeśli mamy współrzędne
    if (hasLatLon && userLatLonValid) {
      qrzPopupDistance = calculateDistance(userLat, userLon, lat, lon);
    } else if (hasLatLon && userLocator.length() >= 4) {
      double userLatLocal, userLonLocal;
      locatorToLatLon(userLocator, userLatLocal, userLonLocal);
      qrzPopupDistance = calculateDistance(userLatLocal, userLonLocal, lat, lon);
    } else {
      qrzPopupDistance = 0.0;
    }
    qrzPopupFetchedAt = millis();
    qrzPopupActive = true;
    drawQrzPopup();
    // Asynchronous fetch for fresh data - update popup after
    if (fetchCallookCallsignInfo(qrzPopupCallsign, grid, country, name, email, qth, lat, lon, hasLatLon)) {
      qrzPopupGrid = grid;
      qrzPopupCountry = country;
      qrzPopupName = name;
      qrzPopupEmail = email;
      qrzPopupQth = qth;
      // Oblicz odległość jeśli mamy współrzędne
      if (hasLatLon && userLatLonValid) {
        qrzPopupDistance = calculateDistance(userLat, userLon, lat, lon);
      } else if (hasLatLon && userLocator.length() >= 4) {
        double userLatLocal, userLonLocal;
        locatorToLatLon(userLocator, userLatLocal, userLonLocal);
        qrzPopupDistance = calculateDistance(userLatLocal, userLonLocal, lat, lon);
      } else {
        qrzPopupDistance = 0.0;
      }
      drawQrzPopup();
    }
    return;
  }

  // Jeśli nie ma w cache, pokaż popup z "Loading..." i pobierz w tle
  qrzPopupGrid = "";
  qrzPopupCountry = "Fetching...";
  qrzPopupName = "";
  qrzPopupQth = "";
  qrzPopupDistance = 0.0;
  qrzPopupFetchedAt = 0;
  qrzPopupActive = true;
  drawQrzPopup();

  // Pobierz dane z QRZ.com jako pierwsze (globalne API, wymaga konta)
  Serial.println("[QRZ POPUP] Trying QRZ.com...");
  if (fetchQrzCallsignInfo(qrzPopupCallsign, grid, country, name, email, qth, lat, lon, hasLatLon)) {
    Serial.print("[QRZ POPUP] QRZ.com fetch - hasLatLon=");
    Serial.print(hasLatLon ? "true" : "false");
    Serial.print(" lat=");
    Serial.print(lat, 6);
    Serial.print(" lon=");
    Serial.println(lon, 6);
    qrzPopupGrid = grid;
    qrzPopupCountry = country;
    qrzPopupName = name;
    qrzPopupEmail = email;
    qrzPopupQth = qth;
    qrzPopupLat = lat;
    qrzPopupLon = lon;
    qrzPopupHasLatLon = hasLatLon;
    // Oblicz odległość jeśli mamy współrzędne
    if (hasLatLon && userLatLonValid) {
      qrzPopupDistance = calculateDistance(userLat, userLon, lat, lon);
    } else if (hasLatLon && userLocator.length() >= 4) {
      double userLatLocal, userLonLocal;
      locatorToLatLon(userLocator, userLatLocal, userLonLocal);
      qrzPopupDistance = calculateDistance(userLatLocal, userLonLocal, lat, lon);
    } else {
      qrzPopupDistance = 0.0;
    }
    qrzPopupFetchedAt = millis();
    drawQrzPopup();
  } else {
    // QRZ.com nie zadziałał - spróbuj Callook.info (darmowe API, tylko USA)
    Serial.println("[QRZ POPUP] QRZ.com failed, trying Callook.info...");
    if (fetchCallookCallsignInfo(qrzPopupCallsign, grid, country, name, email, qth, lat, lon, hasLatLon)) {
    Serial.print("[QRZ POPUP] API fetch - hasLatLon=");
    Serial.print(hasLatLon ? "true" : "false");
    Serial.print(" lat=");
    Serial.print(lat, 6);
    Serial.print(" lon=");
    Serial.println(lon, 6);
    qrzPopupGrid = grid;
    qrzPopupCountry = country;
    qrzPopupName = name;
    qrzPopupEmail = email;
    qrzPopupQth = qth;
    qrzPopupLat = lat;
    qrzPopupLon = lon;
    qrzPopupHasLatLon = hasLatLon;
    // Oblicz odległość jeśli mamy współrzędne
    if (hasLatLon && userLatLonValid) {
      qrzPopupDistance = calculateDistance(userLat, userLon, lat, lon);
    } else if (hasLatLon && userLocator.length() >= 4) {
      double userLatLocal, userLonLocal;
      locatorToLatLon(userLocator, userLatLocal, userLonLocal);
      qrzPopupDistance = calculateDistance(userLatLocal, userLonLocal, lat, lon);
    } else {
      qrzPopupDistance = 0.0;
    }
    qrzPopupFetchedAt = millis();
    drawQrzPopup();
  } else {
    // Błąd pobierania z Callook - spróbuj HamQTH (globalne API)
    Serial.println("[QRZ POPUP] Callook failed, trying HamQTH...");
    
    if (fetchHamQthCallsignInfo(qrzPopupCallsign, grid, country, name, email, qth, lat, lon, hasLatLon)) {
      Serial.print("[QRZ POPUP] HamQTH fetch - hasLatLon=");
      Serial.print(hasLatLon ? "true" : "false");
      Serial.print(" lat=");
      Serial.print(lat, 6);
      Serial.print(" lon=");
      Serial.println(lon, 6);
      qrzPopupGrid = grid;
      qrzPopupCountry = country;
      qrzPopupName = name;
      qrzPopupEmail = email;
      qrzPopupQth = qth;
      qrzPopupLat = lat;
      qrzPopupLon = lon;
      qrzPopupHasLatLon = hasLatLon;
      // Oblicz odległość jeśli mamy współrzędne
      if (hasLatLon && userLatLonValid) {
        qrzPopupDistance = calculateDistance(userLat, userLon, lat, lon);
      } else if (hasLatLon && userLocator.length() >= 4) {
        double userLatLocal, userLonLocal;
        locatorToLatLon(userLocator, userLatLocal, userLonLocal);
        qrzPopupDistance = calculateDistance(userLatLocal, userLonLocal, lat, lon);
      } else {
        qrzPopupDistance = 0.0;
      }
      qrzPopupFetchedAt = millis();
      drawQrzPopup();
      return;
    }
    
    // HamQTH też nie zadziałał - spróbuj znaleźć lokalizator w spotach DX
    String foundLocator = "";
    Serial.print("[QRZ POPUP] Both APIs failed, searching spots for: ");
    Serial.println(qrzPopupCallsign);
    Serial.print("[QRZ POPUP] Total spots in array: ");
    Serial.println(spotCount);
    
    // Szukaj w tablicy spots (DX Cluster)
    for (int i = 0; i < spotCount; i++) {
      Serial.print("[QRZ POPUP] Checking spot ");
      Serial.print(i);
      Serial.print(" callsign=");
      Serial.print(spots[i].callsign);
      Serial.print(" vs ");
      Serial.println(qrzPopupCallsign);
      
      if (spots[i].callsign.equalsIgnoreCase(qrzPopupCallsign)) {
        Serial.print("[QRZ POPUP] FOUND MATCH spot ");
        Serial.print(i);
        Serial.print(" locator=");
        Serial.println(spots[i].locator);
        if (spots[i].locator.length() >= 4) {
          foundLocator = spots[i].locator;
          break;
        }
      }
    }
    // Szukaj w tablicy potaSpots (POTA)
    if (foundLocator.length() < 4) {
      for (int i = 0; i < potaSpotCount; i++) {
        if (potaSpots[i].callsign.equalsIgnoreCase(qrzPopupCallsign)) {
          if (potaSpots[i].locator.length() >= 4) {
            foundLocator = potaSpots[i].locator;
            break;
          }
        }
      }
    }
    
    if (foundLocator.length() >= 4) {
      // Znaleziono lokalizator - użyj go do pokazania na mapie
      double spotLat, spotLon;
      locatorToLatLon(foundLocator, spotLat, spotLon);
      qrzPopupGrid = foundLocator;
      qrzPopupLat = spotLat;
      qrzPopupLon = spotLon;
      qrzPopupHasLatLon = true;
      qrzPopupCountry = "Data not available";
      qrzPopupName = "(using grid from spot)";
      Serial.print("[QRZ POPUP] Using locator from spot: ");
      Serial.print(foundLocator);
      Serial.print(" lat=");
      Serial.print(spotLat, 6);
      Serial.print(" lon=");
      Serial.println(spotLon, 6);
      drawQrzPopup();
    } else {
      // Brak lokalizatora w spotach - sprawdź czy mamy w cache QRZ
      Serial.println("[QRZ POPUP] No spot locator found, checking QRZ cache...");
      
      String cacheGrid, cacheCountry, cacheName, cacheEmail, cacheQth;
      double cacheLat, cacheLon;
      bool cacheHasLatLon;
      
      bool hasCache = getQrzCacheFresh(qrzPopupCallsign, cacheGrid, cacheCountry, cacheName, cacheEmail, cacheQth, cacheLat, cacheLon, cacheHasLatLon, QRZ_CACHE_TTL_MS);
      Serial.print("[QRZ POPUP] Cache check result: ");
      Serial.println(hasCache ? "FOUND" : "NOT FOUND");
      
      if (hasCache) {
        Serial.print("[QRZ POPUP] Cache data - grid: ");
        Serial.print(cacheGrid);
        Serial.print(" hasLatLon: ");
        Serial.println(cacheHasLatLon ? "true" : "false");
        
        // Mamy dane w cache - użyj ich
        if (cacheGrid.length() >= 4) {
          double spotLat, spotLon;
          locatorToLatLon(cacheGrid, spotLat, spotLon);
          qrzPopupGrid = cacheGrid;
          qrzPopupLat = spotLat;
          qrzPopupLon = spotLon;
          qrzPopupHasLatLon = true;
          qrzPopupCountry = cacheCountry.length() > 0 ? cacheCountry : "Data from cache";
          qrzPopupName = cacheName.length() > 0 ? cacheName : "(from QRZ cache)";
          Serial.print("[QRZ POPUP] Using locator from QRZ cache: ");
          Serial.println(cacheGrid);
          drawQrzPopup();
        } else if (cacheHasLatLon) {
          // Mamy lat/lon ale nie mamy grid
          qrzPopupGrid = cacheGrid;
          qrzPopupLat = cacheLat;
          qrzPopupLon = cacheLon;
          qrzPopupHasLatLon = true;
          qrzPopupCountry = cacheCountry.length() > 0 ? cacheCountry : "Data from cache";
          qrzPopupName = cacheName.length() > 0 ? cacheName : "(from QRZ cache)";
          Serial.print("[QRZ POPUP] Using lat/lon from QRZ cache");
          drawQrzPopup();
        } else {
          // Brak lokalizatora - uĹĽyj przybliĹĽonej lokalizacji z prefiksu
          String approxGrid = getApproximateGridFromCallsign(qrzPopupCallsign);
          if (approxGrid.length() >= 4) {
            double approxLat, approxLon;
            locatorToLatLon(approxGrid, approxLat, approxLon);
            qrzPopupGrid = approxGrid;
            qrzPopupLat = approxLat;
            qrzPopupLon = approxLon;
            qrzPopupHasLatLon = true;
            qrzPopupCountry = "Approximate location";
            qrzPopupName = "(from callsign prefix)";
            Serial.print("[QRZ POPUP] Using approximate grid: ");
            Serial.print(approxGrid);
            Serial.print(" lat=");
            Serial.print(approxLat, 6);
            Serial.print(" lon=");
            Serial.println(approxLon, 6);
            drawQrzPopup();
          } else {
            qrzPopupCountry = "Data not available";
            qrzPopupName = "(No locator in spot or cache)";
            drawQrzPopup();
          }
        }
      } else {
        // Brak lokalizatora - uĹĽyj przybliĹĽonej lokalizacji z prefiksu
        String approxGrid = getApproximateGridFromCallsign(qrzPopupCallsign);
        if (approxGrid.length() >= 4) {
          double approxLat, approxLon;
          locatorToLatLon(approxGrid, approxLat, approxLon);
          qrzPopupGrid = approxGrid;
          qrzPopupLat = approxLat;
          qrzPopupLon = approxLon;
          qrzPopupHasLatLon = true;
          qrzPopupCountry = "Approximate location";
          qrzPopupName = "(from callsign prefix)";
          Serial.print("[QRZ POPUP] Using approximate grid: ");
          Serial.print(approxGrid);
          Serial.print(" lat=");
          Serial.print(approxLat, 6);
          Serial.print(" lon=");
          Serial.println(approxLon, 6);
          drawQrzPopup();
        } else {
          qrzPopupCountry = "Data not available";
          qrzPopupName = "(No locator in spot or cache)";
          drawQrzPopup();
        }
      }
    }
  }
}
}

// ========== MENU WYGASZACZA EKRANU ==========

void drawScreenSaverMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 6);
  tft.print("WYGASZACZ EKRANU");

  const int tileW = 100;
  const int tileH = 40;
  const int gap = 10;
  const int startX = (480 - (2 * tileW + gap)) / 2;
  const int tileY = 60;

  // Włącz/Wyłącz
  drawFilterTile(startX, tileY, tileW, tileH, "WLACZ", screenSaverEnabled);
  drawFilterTile(startX + tileW + gap, tileY, tileW, tileH, "WYLACZ", !screenSaverEnabled);

  // Czas w minutach
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(10, 120);
  tft.print("CZAS (min): ");
  tft.setTextColor(TFT_WHITE);
  tft.print(screenSaverTimeoutMin);

  // Typ wygaszacza - przycisk zmiany
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setCursor(200, 120);
  tft.print("TYP: ");
  tft.setTextColor(TFT_YELLOW);
  tft.print(SAVER_TYPE_NAMES[screenSaverType]);
  
  // Przycisk zmiany typu
  const int typeTileW = 100;
  const int typeTileH = 30;
  const int typeX = 300;
  const int typeY = 115;
  drawFilterTile(typeX, typeY, typeTileW, typeTileH, "ZMIEN", false);

  const int minTileW = 60;
  const int minTileH = 35;
  const int minGap = 8;
  const int minStartX = 10;
  const int minY = 140;

  // Przyciski +/- minuty
  drawFilterTile(minStartX, minY, minTileW, minTileH, "-5", false);
  drawFilterTile(minStartX + minTileW + minGap, minY, minTileW, minTileH, "-1", false);
  drawFilterTile(minStartX + 2 * (minTileW + minGap), minY, minTileW, minTileH, "+1", false);
  drawFilterTile(minStartX + 3 * (minTileW + minGap), minY, minTileW, minTileH, "+5", false);

  // CLOSE
  const int closeW = 120;
  const int closeH = 40;
  const int closeX = (480 - closeW) / 2;
  const int closeY = 280;
  drawFilterTile(closeX, closeY, closeW, closeH, "ZAMKNIJ", false);
}

void handleScreenSaverMenuTouch(uint16_t x, uint16_t y) {
  const int tileW = 100;
  const int tileH = 40;
  const int gap = 10;
  const int startX = (480 - (2 * tileW + gap)) / 2;
  const int tileY = 60;

  // Włącz/Wyłącz
  if (isPointInRect(x, y, startX, tileY, tileW, tileH)) {
    setScreenSaverEnabled(true);
    drawScreenSaverMenu();
    return;
  }
  if (isPointInRect(x, y, startX + tileW + gap, tileY, tileW, tileH)) {
    setScreenSaverEnabled(false);
    drawScreenSaverMenu();
    return;
  }

  // Przycisk zmiany typu wygaszacza
  const int typeTileW = 100;
  const int typeTileH = 30;
  const int typeX = 300;
  const int typeY = 115;
  if (isPointInRect(x, y, typeX, typeY, typeTileW, typeTileH)) {
    // Zmień typ na następny
    int currentType = (int)screenSaverType;
    currentType = (currentType + 1) % 3;  // Cykl: 0->1->2->0
    screenSaverType = (ScreenSaverType)currentType;
    // Zapisz w preferencjach
    preferences->putInt("saver_type", currentType);
    drawScreenSaverMenu();
    return;
  }

  const int minTileW = 60;
  const int minTileH = 35;
  const int minGap = 8;
  const int minStartX = 10;
  const int minY = 140;

  // Minus 5 min
  if (isPointInRect(x, y, minStartX, minY, minTileW, minTileH)) {
    setScreenSaverTimeout(screenSaverTimeoutMin - 5);
    drawScreenSaverMenu();
    return;
  }
  // Minus 1 min
  if (isPointInRect(x, y, minStartX + minTileW + minGap, minY, minTileW, minTileH)) {
    setScreenSaverTimeout(screenSaverTimeoutMin - 1);
    drawScreenSaverMenu();
    return;
  }
  // Plus 1 min
  if (isPointInRect(x, y, minStartX + 2 * (minTileW + minGap), minY, minTileW, minTileH)) {
    setScreenSaverTimeout(screenSaverTimeoutMin + 1);
    drawScreenSaverMenu();
    return;
  }
  // Plus 5 min
  if (isPointInRect(x, y, minStartX + 3 * (minTileW + minGap), minY, minTileW, minTileH)) {
    setScreenSaverTimeout(screenSaverTimeoutMin + 5);
    drawScreenSaverMenu();
    return;
  }

  // Zamknij
  const int closeW = 120;
  const int closeH = 40;
  const int closeX = (480 - closeW) / 2;
  const int closeY = 280;
  if (isPointInRect(x, y, closeX, closeY, closeW, closeH)) {
    inMenu = false;
    screenSaverMenuActive = false;
    drawScreen(currentScreen);
    return;
  }
}

// ========== UŚPIENIE EKRANU ==========

void drawScreenSleepMenu() {
  tft.fillScreen(TFT_BLACK);

  // Nagłówek
  tft.fillRect(0, 0, 480, 28, menuThemeColor);
  tft.setTextColor(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 6);
  tft.print("USPIENIE EKRANU");

  // Stan: Włączony/Wyłączony
  const int tileW = 100;
  const int tileH = 40;
  const int gap = 10;
  const int startX = (480 - (2 * tileW + gap)) / 2;
  const int tileY = 60;

  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(10, 50);
  tft.print("Stan:");

  drawFilterTile(startX, tileY, tileW, tileH, "WLACZ", screenSleepEnabled);
  drawFilterTile(startX + tileW + gap, tileY, tileW, tileH, "WYLACZ", !screenSleepEnabled);

  // Ustawienie czasu w minutach
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setTextSize(1);
  tft.setCursor(10, 120);
  tft.print("Czas uspienia:");

  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(3);
  tft.setCursor(220, 115);
  tft.print(screenSleepMenuTimeoutMin);
  tft.print(" min");

  // Przyciski +/- 1 minuta
  const int minTileW = 60;
  const int minTileH = 35;
  const int minGap = 8;
  const int minStartX = 10;
  const int minY = 160;

  drawFilterTile(minStartX, minY, minTileW, minTileH, "-1", false);
  drawFilterTile(minStartX + minTileW + minGap, minY, minTileW, minTileH, "+1", false);

  // Przycisk ZAPISZ
  const int saveW = 90;
  const int saveH = 26;
  const int saveX = 10;
  const int saveY = 240;
  drawFilterTile(saveX, saveY, saveW, saveH, "ZAPISZ", false);

  // Przycisk ANULUJ
  const int cancelX = saveX + saveW + 10;
  drawFilterTile(cancelX, saveY, saveW, saveH, "ANULUJ", false);
}

void handleScreenSleepMenuTouch(uint16_t x, uint16_t y) {
  const int tileW = 100;
  const int tileH = 40;
  const int gap = 10;
  const int startX = (480 - (2 * tileW + gap)) / 2;
  const int tileY = 60;

  // Włącz
  if (isPointInRect(x, y, startX, tileY, tileW, tileH)) {
    screenSleepEnabled = true;
    drawScreenSleepMenu();
    return;
  }
  // Wyłącz
  if (isPointInRect(x, y, startX + tileW + gap, tileY, tileW, tileH)) {
    screenSleepEnabled = false;
    drawScreenSleepMenu();
    return;
  }

  // +/- minuty
  const int minTileW = 60;
  const int minTileH = 35;
  const int minGap = 8;
  const int minStartX = 10;
  const int minY = 160;

  // Minus 1 min
  if (isPointInRect(x, y, minStartX, minY, minTileW, minTileH)) {
    if (screenSleepMenuTimeoutMin > 1) {
      screenSleepMenuTimeoutMin--;
    }
    drawScreenSleepMenu();
    return;
  }
  // Plus 1 min
  if (isPointInRect(x, y, minStartX + minTileW + minGap, minY, minTileW, minTileH)) {
    screenSleepMenuTimeoutMin++;
    drawScreenSleepMenu();
    return;
  }

  // Przycisk ZAPISZ
  const int saveW = 90;
  const int saveH = 26;
  const int saveX = 10;
  const int saveY = 240;
  if (isPointInRect(x, y, saveX, saveY, saveW, saveH)) {
    screenSleepTimeoutMin = screenSleepMenuTimeoutMin;
    screenSleepEnabled = true;
    screenSleepLastActivityMs = millis();
    preferences->putInt("sleep_timeout", screenSleepTimeoutMin);
    preferences->putBool("sleep_en", screenSleepEnabled);
    screenSleepMenuActive = false;
    inMenu = false;
    drawScreen(currentScreen);
    return;
  }

  // Przycisk ANULUJ
  const int cancelX = saveX + saveW + 10;
  if (isPointInRect(x, y, cancelX, saveY, saveW, saveH)) {
    screenSleepMenuActive = false;
    inMenu = false;
    drawScreen(currentScreen);
    return;
  }
}

void enterScreenSleep() {
  if (!screenSleepEnabled || screenSleepActive) return;

  screenSleepActive = true;
  screenSleepPrevScreen = currentScreen;

  // Wyłącz podświetlenie
  ledcWrite(TFT_BL_PIN, 0);

  // Wyczyść ekran
  tft.fillScreen(TFT_BLACK);
}

void wakeUpFromSleep() {
  if (!screenSleepActive) return;

  screenSleepActive = false;
  screenSleepLastActivityMs = millis();

  // Przywróć podświetlenie
  setBacklightPercent(backlightPercent);

  // Przywróć poprzedni ekran
  drawScreen(screenSleepPrevScreen);
}

void checkScreenSleepTimeout() {
  if (!screenSleepEnabled || screenSleepActive) return;

  unsigned long timeoutMs = (unsigned long)screenSleepTimeoutMin * 60 * 1000;
  if (millis() - screenSleepLastActivityMs > timeoutMs) {
    enterScreenSleep();
  }
}

// ========== PREFERENCES ==========

void loadPreferences() {
  if (preferences == nullptr) {
    preferences = new Preferences();
  }
  
  preferences->begin("dxcluster", false);
  yield();

  // Kolejność ekranów
  loadDefaultScreenOrder();
  size_t bytesRead = preferences->getBytes("screen_order", screenOrder, sizeof(screenOrder));
  if (bytesRead != sizeof(screenOrder)) {
    loadDefaultScreenOrder();
    // Zapisz nową konfigurację do Preferences
    preferences->putBytes("screen_order", screenOrder, sizeof(screenOrder));
  }
  ensureScreenOrderValid();
  currentScreen = firstActiveScreen();
  tftAutoSwitchEnabled = preferences->getBool("tft_autosw", false);
  applyTftAutoSwitchTimeSec(preferences->getInt("tft_sw_sec", DEFAULT_TFT_SWITCH_TIME_SEC));
  tftAutoSwitchLastMs = millis();
  tftAutoSwitchLastScreen = currentScreen;
  
  wifiSSID = preferences->getString("wifi_ssid", "");
  yield();
  wifiPassword = preferences->getString("wifi_pass", "");
  yield();
  wifiSSID2 = preferences->getString("wifi_ssid2", "");
  yield();
  wifiPassword2 = preferences->getString("wifi_pass2", "");
  yield();
  
  clusterHost = preferences->getString("cluster_host", DEFAULT_CLUSTER_HOST);
  yield();
  clusterPort = preferences->getInt("cluster_port", DEFAULT_CLUSTER_PORT);
  yield();
  potaClusterHost = preferences->getString("pota_host", DEFAULT_POTA_CLUSTER_HOST);
  yield();
  potaClusterPort = preferences->getInt("pota_port", DEFAULT_POTA_CLUSTER_PORT);
  yield();
  potaFilterCommand = preferences->getString("pota_filter", DEFAULT_POTA_FILTER_COMMAND);
  yield();
  potaApiUrl = preferences->getString("pota_api_url", DEFAULT_POTA_API_URL);
  yield();
  hamalertHost = preferences->getString("hama_host", DEFAULT_HAMALERT_HOST);
  yield();
  hamalertPort = preferences->getInt("hama_port", DEFAULT_HAMALERT_PORT);
  if (hamalertPort <= 0 || hamalertPort > 65535) hamalertPort = DEFAULT_HAMALERT_PORT;
  yield();
  hamalertLogin = preferences->getString("hama_user", "");
  yield();
  hamalertPassword = preferences->getString("hama_pass", "");
  yield();
  userCallsign = preferences->getString("user_callsign", "");
  yield();
  userLocator = preferences->getString("user_locator", "");
  yield();
  timezoneHours = preferences->getInt("timezone", DEFAULT_TIMEZONE_HOURS);
  if (timezoneHours < -12) timezoneHours = -12;
  if (timezoneHours > 14) timezoneHours = 14;
  yield();
  userLat = preferences->getFloat("user_lat", 0.0f);
  yield();
  userLon = preferences->getFloat("user_lon", 0.0f);
  yield();
  userLatLonValid = preferences->getBool("user_ll_ok", false);
  yield();
  
  // Jeśli nie mamy ważnych współrzędnych ale mamy locator, przelicz z locatora
  if (!userLatLonValid && userLocator.length() >= 4) {
    updateUserLatLonFromLocator();
    // Nie zapisujemy do preferencji tutaj - to tylko odczyt
  }
  qrzUsername = preferences->getString("qrz_user", "");
  yield();
  qrzPassword = preferences->getString("qrz_pass", "");
  yield();
  weatherApiKey = preferences->getString("weather_key", "");
  yield();
  openWebRxUrl = preferences->getString("openwebrx_url", DEFAULT_OPENWEBRX_URL);
  yield();
  backlightPercent = preferences->getInt("tft_backlight", TFT_BACKLIGHT);
  if (backlightPercent < MIN_BACKLIGHT_PERCENT) backlightPercent = MIN_BACKLIGHT_PERCENT;
  if (backlightPercent > 100) backlightPercent = 100;
  tftInvertColors = sanitizeTftInvertSetting(preferences->getBool("tft_inv", tftInvertColors));
  callsignColor = preferences->getInt("callsign_color", TFT_WHITE);
  tftLanguage = preferences->getUChar("tft_lang", TFT_LANG_PL);
  if (tftLanguage > TFT_LANG_EN) tftLanguage = TFT_LANG_PL;
  dxTableSizeMode = preferences->getUChar("tft_tbl_size", DX_TABLE_SIZE_NORMAL);
  if (dxTableSizeMode > DX_TABLE_SIZE_ENLARGED) dxTableSizeMode = DX_TABLE_SIZE_NORMAL;
  int savedMenuHue = preferences->getInt("menu_hue", DEFAULT_MENU_THEME_HUE);
  if (savedMenuHue < 0) savedMenuHue = 0;
  if (savedMenuHue > 255) savedMenuHue = 255;
  menuThemeHue = (uint8_t)savedMenuHue;
  applyMenuThemeFromHue();
  touchXMin = preferences->getInt("touch_xmin", TOUCH_X_MIN);
  touchXMax = preferences->getInt("touch_xmax", TOUCH_X_MAX);
  touchYMin = preferences->getInt("touch_ymin", TOUCH_Y_MIN);
  touchYMax = preferences->getInt("touch_ymax", TOUCH_Y_MAX);
  touchSwapXY = preferences->getBool("touch_swap", TOUCH_SWAP_XY);
  touchInvertX = preferences->getBool("touch_invx", TOUCH_INVERT_X);
  touchInvertY = preferences->getBool("touch_invy", TOUCH_INVERT_Y);
  tftRotation = preferences->getUChar("tft_rot", 1);
  if (tftRotation > 3) tftRotation = 1;
  touchRotation = preferences->getUChar("touch_rot", 1);
  if (touchRotation > 3) touchRotation = 1;
  if (touchXMin < 0 || touchXMax > 4095 || touchXMin >= touchXMax) {
    touchXMin = TOUCH_X_MIN;
    touchXMax = TOUCH_X_MAX;
  }
  if (touchYMin < 0 || touchYMax > 4095 || touchYMin >= touchYMax) {
    touchYMin = TOUCH_Y_MIN;
    touchYMax = TOUCH_Y_MAX;
  }

  // Konfiguracja PSKReporter
  pskReceiverCallsign = preferences->getString("psk_receiver", "");
  pskMaxSpots = preferences->getInt("psk_maxspots", 50);
  if (pskMaxSpots < 10) pskMaxSpots = 50;
  if (pskMaxSpots > 100) pskMaxSpots = 50;
  pskHoursWindow = preferences->getInt("psk_hours", 1);
  if (pskHoursWindow < 1) pskHoursWindow = 1;
  if (pskHoursWindow > 24) pskHoursWindow = 1;
  pskFilterBand = preferences->getString("psk_band", "");
  pskFilterMode = preferences->getString("psk_mode", "");
  pskCustomUrl = preferences->getString("psk_url", "");
  // HTTP Monitoring
  pskMonitorCallsign = preferences->getString("psk_monitor", "");
  pskReportDays = preferences->getInt("psk_report_days", 0);
  if (pskReportDays < 0) pskReportDays = 0;
  if (pskReportDays > 62) pskReportDays = 62;
  pskAutoRefreshMinutes = preferences->getInt("psk_autorefresh", 5);
  if (pskAutoRefreshMinutes < 0) pskAutoRefreshMinutes = 0;
  if (pskAutoRefreshMinutes > 60) pskAutoRefreshMinutes = 60;

  // MQTT PSK Reporter settings
  pskMqttEnabled = preferences->getBool("psk_mqtt_en", false);
  pskMqttServer = preferences->getString("psk_mqtt_srv", "mqtt.pskreporter.info");
  pskMqttPort = preferences->getInt("psk_mqtt_port", 1883);
  if (pskMqttPort <= 0 || pskMqttPort > 65535) pskMqttPort = 1883;
  pskMqttCallsign = preferences->getString("psk_mqtt_call", "");

  screen1TimeMode = (uint8_t)preferences->getInt("screen1_time", SCREEN1_TIME_UTC);
  if (screen1TimeMode > SCREEN1_TIME_LOCAL) screen1TimeMode = SCREEN1_TIME_UTC;
  
  // Konfiguracja wygaszacza ekranu (Matrix)
  screenSaverEnabled = preferences->getBool("ss_enabled", false);
  screenSaverTimeoutMin = preferences->getInt("ss_timeout", DEFAULT_SCREEN_SAVER_TIMEOUT_MIN);
  if (screenSaverTimeoutMin < 1) screenSaverTimeoutMin = 1;
  if (screenSaverTimeoutMin > 60) screenSaverTimeoutMin = 60;
  screenSaverLastActivityMs = millis();
  screenSaverActive = false;
  
  // Wczytaj typ wygaszacza
  int savedType = preferences->getInt("saver_type", 0);
  if (savedType < 0 || savedType > 2) savedType = 0;
  screenSaverType = (ScreenSaverType)savedType;
  
  // Konfiguracja filtrów CC-Cluster
  clusterNoAnnouncements = preferences->getBool("cluster_noann", true);
  yield();
  clusterNoWWV = preferences->getBool("cluster_nowwv", true);
  yield();
  clusterNoWCY = preferences->getBool("cluster_nowcy", true);
  yield();
  clusterUseFilters = preferences->getBool("cluster_usefilters", false);
  yield();
  clusterFilterCommands = preferences->getString("cluster_filters", "");
  yield();
  
  // Konfiguracja APRS-IS
  aprsIsHost = preferences->getString("aprs_host", DEFAULT_APRS_IS_HOST);
  yield();
  aprsIsPort = preferences->getInt("aprs_port", DEFAULT_APRS_IS_PORT);
  yield();
  aprsCallsign = preferences->getString("aprs_callsign", DEFAULT_APRS_CALLSIGN);
  yield();
  aprsPasscode = preferences->getInt("aprs_passcode", DEFAULT_APRS_PASSCODE);
  applyAprsSsid(preferences->getInt("aprs_ssid", DEFAULT_APRS_SSID));
  yield();
  aprsFilterRadius = preferences->getInt("aprs_radius", DEFAULT_APRS_FILTER_RADIUS);
  yield();
  aprsBeaconEnabled = preferences->getBool("aprs_beacon", true);
  String aprsSymPref = preferences->getString("aprs_symbol", aprsSymbolTwoChar);
  applyAprsSymbol(aprsSymPref);
  aprsUserComment = preferences->getString("aprs_comment", "");
  aprsAlertCsv = sanitizeAprsAlertList(preferences->getString("aprs_alert", ""));
  aprsAlertEnabled = preferences->getBool("aprs_alert_en", true);
  applyAprsAlertMinSeconds(preferences->getInt(NVS_KEY_APRS_ALERT_MIN_SEC, DEFAULT_APRS_ALERT_MIN_SEC));
  applyAprsAlertScreenSeconds(preferences->getInt(NVS_KEY_APRS_ALERT_SCREEN_SEC, DEFAULT_APRS_ALERT_SCREEN_SEC));
  aprsAlertNearbyEnabled = preferences->getBool("aprs_alert_near_en", true);
  aprsAlertWxEnabled = preferences->getBool(NVS_KEY_APRS_ALERT_WX_ENABLED, false);
  applyAprsAlertDistanceKm(preferences->getFloat(NVS_KEY_APRS_ALERT_DISTANCE_KM, DEFAULT_APRS_ALERT_DISTANCE_KM));
  enableLedAlert = preferences->getBool(NVS_KEY_ENABLE_LED_ALERT, DEFAULT_ENABLE_LED_ALERT);
  applyLedAlertDurationMs(preferences->getInt(NVS_KEY_LED_ALERT_DURATION_MS, DEFAULT_LED_ALERT_DURATION_MS));
  applyLedAlertBlinkMs(preferences->getInt(NVS_KEY_LED_ALERT_BLINK_MS, DEFAULT_LED_ALERT_BLINK_MS));
  resetAprsAlertCooldownState();
  int storedIntervalMin = preferences->getInt(NVS_KEY_APRS_INTERVAL_MIN, DEFAULT_APRS_INTERVAL_MIN);
  applyAprsIntervalMinutes(storedIntervalMin);
  yield();
  // Ograniczenie promienia do 1-50 km
  if (aprsFilterRadius < 1) aprsFilterRadius = 1;
  if (aprsFilterRadius > 50) aprsFilterRadius = 50;
  // Uwaga: APRS uÄąÄ˝ywa wspÄ‚łÄąâ€šrzĂ„â„˘dnych z sekcji "Moja Stacja" (userLat, userLon) - nie ma osobnych pÄ‚łl

  if (!userLatLonValid && userLocator.length() >= 4) {
    updateUserLatLonFromLocator();
    preferences->putFloat("user_lat", (float)userLat);
    preferences->putFloat("user_lon", (float)userLon);
    preferences->putBool("user_ll_ok", userLatLonValid);
  }
  
  preferences->end();

  // Zastosuj inwersję kolorów po wczytaniu ustawień
  if (tftInitialized) {
    applyTftInversion();
  }
  
  Serial.println("Konfiguracja wczytana");
}

void savePreferences() {
  if (preferences == nullptr) {
    preferences = new Preferences();
  }
  
  preferences->begin("dxcluster", false);
  preferences->putBytes("screen_order", screenOrder, sizeof(screenOrder));
  
  preferences->putString("wifi_ssid", wifiSSID);
  preferences->putString("wifi_pass", wifiPassword);
  preferences->putString("wifi_ssid2", wifiSSID2);
  preferences->putString("wifi_pass2", wifiPassword2);
  preferences->putString("cluster_host", clusterHost);
  preferences->putInt("cluster_port", clusterPort);
  preferences->putString("pota_host", potaClusterHost);
  preferences->putInt("pota_port", potaClusterPort);
  preferences->putString("pota_filter", potaFilterCommand);
  preferences->putString("pota_api_url", potaApiUrl);
  preferences->putString("hama_host", hamalertHost);
  preferences->putInt("hama_port", hamalertPort);
  preferences->putString("hama_user", hamalertLogin);
  preferences->putString("hama_pass", hamalertPassword);
  preferences->putString("user_callsign", userCallsign);
  preferences->putString("user_locator", userLocator);
  preferences->putInt("timezone", timezoneHours);
  preferences->putFloat("user_lat", (float)userLat);
  preferences->putFloat("user_lon", (float)userLon);
  preferences->putBool("user_ll_ok", userLatLonValid);
  preferences->putString("qrz_user", qrzUsername);
  preferences->putString("qrz_pass", qrzPassword);
  preferences->putString("weather_key", weatherApiKey);
  preferences->putString("openwebrx_url", openWebRxUrl);
  preferences->putInt("tft_backlight", backlightPercent);
  preferences->putBool("tft_inv", sanitizeTftInvertSetting(tftInvertColors));
  preferences->putInt("callsign_color", callsignColor);
  preferences->putUChar("tft_lang", tftLanguage);
  preferences->putUChar("tft_tbl_size", dxTableSizeMode);
  preferences->putInt("menu_hue", menuThemeHue);
  preferences->putInt("screen1_time", screen1TimeMode);
  preferences->putBool("tft_autosw", tftAutoSwitchEnabled);
  preferences->putInt("tft_sw_sec", tftAutoSwitchTimeSec);
  preferences->putInt("touch_xmin", touchXMin);
  preferences->putInt("touch_xmax", touchXMax);
  preferences->putInt("touch_ymin", touchYMin);
  preferences->putInt("touch_ymax", touchYMax);
  preferences->putBool("touch_swap", touchSwapXY);
  preferences->putBool("touch_invx", touchInvertX);
  preferences->putBool("touch_invy", touchInvertY);
  preferences->putUChar("touch_rot", touchRotation);
  preferences->putUChar("tft_rot", tftRotation);

  // Konfiguracja PSKReporter
  preferences->putString("psk_receiver", pskReceiverCallsign);
  preferences->putInt("psk_maxspots", pskMaxSpots);
  preferences->putInt("psk_hours", pskHoursWindow);
  preferences->putString("psk_band", pskFilterBand);
  preferences->putString("psk_mode", pskFilterMode);
  preferences->putString("psk_url", pskCustomUrl);
  // HTTP Monitoring
  preferences->putString("psk_monitor", pskMonitorCallsign);
  preferences->putInt("psk_report_days", pskReportDays);
  preferences->putInt("psk_autorefresh", pskAutoRefreshMinutes);

  // MQTT PSK Reporter settings
  preferences->putBool("psk_mqtt_en", pskMqttEnabled);
  preferences->putString("psk_mqtt_srv", pskMqttServer);
  preferences->putInt("psk_mqtt_port", pskMqttPort);
  preferences->putString("psk_mqtt_call", pskMqttCallsign);

  // Konfiguracja wygaszacza ekranu (Matrix)
  preferences->putBool("ss_enabled", screenSaverEnabled);
  preferences->putInt("ss_timeout", screenSaverTimeoutMin);
  
  // Konfiguracja APRS-IS
  preferences->putString("aprs_host", aprsIsHost);
  preferences->putInt("aprs_port", aprsIsPort);
  preferences->putString("aprs_callsign", aprsCallsign);
  preferences->putInt("aprs_passcode", aprsPasscode);
  preferences->putInt("aprs_ssid", aprsSsid);
  preferences->putBool("aprs_beacon", aprsBeaconEnabled);
  preferences->putString("aprs_symbol", aprsSymbolTwoChar);
  preferences->putString("aprs_comment", aprsUserComment);
  preferences->putString("aprs_alert", aprsAlertCsv);
  preferences->putBool("aprs_alert_en", aprsAlertEnabled);
  preferences->putInt(NVS_KEY_APRS_ALERT_MIN_SEC, aprsAlertMinSeconds);
  preferences->putInt(NVS_KEY_APRS_ALERT_SCREEN_SEC, aprsAlertScreenSeconds);
  preferences->putBool("aprs_alert_near_en", aprsAlertNearbyEnabled);
  preferences->putBool(NVS_KEY_APRS_ALERT_WX_ENABLED, aprsAlertWxEnabled);
  preferences->putFloat(NVS_KEY_APRS_ALERT_DISTANCE_KM, aprsAlertDistanceKm);
  preferences->putBool(NVS_KEY_ENABLE_LED_ALERT, enableLedAlert);
  preferences->putInt(NVS_KEY_LED_ALERT_DURATION_MS, ledAlertDurationMs);
  preferences->putInt(NVS_KEY_LED_ALERT_BLINK_MS, ledAlertBlinkMs);
  preferences->putInt(NVS_KEY_APRS_INTERVAL_MIN, aprsIntervalMinutes);
  // Ograniczenie promienia do 1-50 km przed zapisem
  int radiusToSave = aprsFilterRadius;
  if (radiusToSave < 1) radiusToSave = 1;
  if (radiusToSave > 50) radiusToSave = 50;
  preferences->putInt("aprs_radius", radiusToSave);
  // Uwaga: APRS używa współrzędnych z sekcji "Moja Stacja" (userLat, userLon) - nie zapisujemy osobnych pól
  
  // Konfiguracja filtrów CC-Cluster
  preferences->putBool("cluster_noann", clusterNoAnnouncements);
  preferences->putBool("cluster_nowwv", clusterNoWWV);
  preferences->putBool("cluster_nowcy", clusterNoWCY);
  preferences->putBool("cluster_usefilters", clusterUseFilters);
  preferences->putString("cluster_filters", clusterFilterCommands);
  
  preferences->end();
  
  Serial.println("Konfiguracja zapisana");
}

// ========== WEB SERVER ==========

String getMainHTML();
String getConfigHTML();

// Zmienna przechowująca czas startu systemu (do obliczania uptime)
static unsigned long bootTimeMs = 0;

// Funkcja formatująca uptime w czytelny sposób (dni, godziny, minuty, sekundy)
String formatUptime(unsigned long ms) {
  unsigned long totalSeconds = ms / 1000;
  unsigned long days = totalSeconds / 86400;
  unsigned long hours = (totalSeconds % 86400) / 3600;
  unsigned long minutes = (totalSeconds % 3600) / 60;
  unsigned long seconds = totalSeconds % 60;

  String result;
  if (days > 0) {
    result += String(days) + "d ";
  }
  if (hours > 0 || days > 0) {
    result += String(hours) + "h ";
  }
  if (minutes > 0 || hours > 0 || days > 0) {
    result += String(minutes) + "m ";
  }
  result += String(seconds) + "s";
  return result;
}

String getManualPL() {
  return String(
    "ESP32-HAM-CLOCK v1.3 (13.04.2026)\n"
    "Autor: Konrad Wisniewski SP3KON (z użyciem AI)\n"
    "Kontakt: sp3kon@gmail.com\n\n"
    "Wgrywanie przez przeglądarkę (ESP Web Tools)\n"
    "- Strona: https://jason2866.github.io/WebSerial_ESPTool/\n"
    "- Wymagany Chrome lub Edge (WebSerial).\n"
    "- Pliki znajdują się w folderze: build\\esp32.esp32.esp32\\\n"
    "- Dotyczy: CYD ESP32-2432S028R (4MB flash, schemat Default 4MB with spiffs).\n\n"
    "Wgrywanie:\n"
    "1. bootloader.bin    0x1000\n"
    "2. partitions.bin    0x8000\n"
    "3. firmware.bin      0x10000\n"
    "4. littlefs.bin      0x290000\n\n"
    "Po starcie urządzenie tworzy AP: SSID ESP32-HAM-CLOCK, hasło 1234567890\n"
    "(jeśli brak konfiguracji WiFi).\n\n"
    "Konfiguracja WWW:\n"
    "- Wejdź na /config\n"
    "- WiFi: dwa zestawy SSID/hasło (podstawowe i zapasowe)\n"
    "- DX Cluster: host/port + opcje filtrów\n"
    "- POTA: External POTA API Link\n"
    "- HAMALERT: telnet host, port, login i hasło\n"
    "- QRZ.com: username i password\n"
    "- OpenWeather: klucz API\n"
    "- TFT: jasność, język, rozmiar tabel, AUTO-SWITCH\n\n"
    "Więcej informacji: https://github.com/SP3KON/ESP32-HAM-CLOCK\n"
  );
}

void setupWebServer() {
  // UtwÄ‚łrz serwer jeÄąâ€şli jeszcze nie istnieje
  if (server == nullptr) {
    server = new WebServer(80);
  }
  
  // Strona główna - najpierw próbuje z LittleFS, potem wbudowany HTML
  server->on("/", HTTP_GET, []() {
    server->sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server->sendHeader("Pragma", "no-cache");
    server->sendHeader("Expires", "0");
    if (littleFsReady && LittleFS.exists("/index.html")) {
      File f = LittleFS.open("/index.html", "r");
      server->streamFile(f, "text/html; charset=utf-8");
      f.close();
    } else {
      server->send(200, "text/html; charset=utf-8", getMainHTML());
    }
  });

  // Strona główna (alias /index.html) - najpierw LittleFS, potem wbudowany HTML
  server->on("/index.html", HTTP_GET, []() {
    server->sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server->sendHeader("Pragma", "no-cache");
    server->sendHeader("Expires", "0");
    if (littleFsReady && LittleFS.exists("/index.html")) {
      File f = LittleFS.open("/index.html", "r");
      server->streamFile(f, "text/html; charset=utf-8");
      f.close();
    } else {
      server->send(200, "text/html; charset=utf-8", getMainHTML());
    }
  });

  // Strona gÄąâ€šÄ‚łwna (EN)
  server->on("/indexEN.html", HTTP_GET, []() {
    if (littleFsReady && LittleFS.exists("/indexEN.html")) {
      File f = LittleFS.open("/indexEN.html", "r");
      server->streamFile(f, "text/html; charset=utf-8");
      f.close();
    } else if (littleFsReady && LittleFS.exists("/index.html")) {
      File f = LittleFS.open("/index.html", "r");
      server->streamFile(f, "text/html; charset=utf-8");
      f.close();
    } else {
      server->send(200, "text/html; charset=utf-8", getMainHTML());
    }
  });

  // Instrukcja (PL) - przekierowanie na nową wersję HTML
  server->on("/instrukcja.txt", HTTP_GET, []() {
    server->send(301, "text/html", "<html><head><meta http-equiv='refresh' content='0; url=/instruction'></head></html>");
  });

  // Manual (EN)
  server->on("/manual.txt", HTTP_GET, []() {
    if (littleFsReady && LittleFS.exists("/manual.txt")) {
      File f = LittleFS.open("/manual.txt", "r");
      server->streamFile(f, "text/plain; charset=utf-8");
      f.close();
    } else {
      server->send(404, "text/plain", "manual.txt missing in LittleFS");
    }
  });

  // Instrukcja montażu i wgrania (HTML)
  server->on("/instruction", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>ESP32-HAM-CLOCK - Instrukcja</title>";
    html += "<style>";
    html += "body{font-family:Arial,sans-serif;margin:20px;background:#f0f0f0;color:#333;line-height:1.6;}";
    html += "h1{color:#0066cc;border-bottom:2px solid #0066cc;padding-bottom:10px;}";
    html += "h2{color:#0099cc;margin-top:30px;border-left:4px solid #0099cc;padding-left:10px;}";
    html += "h3{color:#666;margin-top:20px;}";
    html += "pre{background:#1a1a1a;color:#00ff00;padding:15px;border-radius:5px;overflow-x:auto;font-size:12px;}";
    html += "code{background:#e0e0e0;padding:2px 6px;border-radius:3px;font-family:monospace;}";
    html += "table{border-collapse:collapse;width:100%;margin:15px 0;background:white;}";
    html += "th,td{border:1px solid #ddd;padding:12px;text-align:left;}";
    html += "th{background:#0066cc;color:white;}";
    html += "tr:nth-child(even){background:#f9f9f9;}";
    html += ".note{background:#fff3cd;border-left:4px solid #ffc107;padding:10px;margin:15px 0;}";
    html += ".warning{background:#f8d7da;border-left:4px solid #dc3545;padding:10px;margin:15px 0;}";
    html += ".success{background:#d4edda;border-left:4px solid #28a745;padding:10px;margin:15px 0;}";
    html += "a{color:#0066cc;text-decoration:none;}";
    html += "a:hover{text-decoration:underline;}";
    html += ".back-btn{display:inline-block;background:#0066cc;color:white;padding:10px 20px;border-radius:5px;margin:20px 0;text-decoration:none;}";
    html += ".back-btn:hover{background:#0055aa;text-decoration:none;}";
    html += "</style></head><body>";

    html += "<h1>📻 ESP32-HAM-CLOCK - Instrukcja Montażu i Wgrania</h1>";
    html += "<a href='/' class='back-btn'>← Powrót do głównej</a>";

    // Sekcja montażu
    html += "<h2>🔧 Montaż - Schemat połączeń</h2>";

    html += "<h3>Wymagane komponenty</h3>";
    html += "<table>";
    html += "<tr><th>Element</th><th>Opis</th><th>Alternatywa</th></tr>";
    html += "<tr><td>ESP32</td><td>DevKit v1 (38 pinów)</td><td>WROOM-32 + płytka</td></tr>";
    html += "<tr><td>Wyświetlacz</td><td>TFT 3.5\" ILI9488 480x320</td><td>ILI9341 2.8\"</td></tr>";
    html += "<tr><td>Zasilanie</td><td>Moduł 18650 z ochroną DW01A</td><td>3xAA lub USB</td></tr>";
    html += "<tr><td>Przyciski</td><td>2x tact switch (BOOT, RST)</td><td>-</td></tr>";
    html += "<tr><td>Rezystory</td><td>2x 100kΩ (dzielnik napięcia)</td><td>-</td></tr>";
    html += "</table>";

    html += "<h3>Schemat połączeń ESP32 z TFT ILI9488</h3>";
    html += "<pre>";
    html += "ESP32 DevKit v1         TFT ILI9488 (3.5\" 480x320)\n";
    html += "═══════════════════════════════════════════════════════\n";
    html += "GPIO 18 (SCK)    ──────→  SCK / SCL        [ żółty ]\n";
    html += "GPIO 23 (MOSI)   ──────→  SDI / MOSI       [ zielony ]\n";
    html += "GPIO 19 (MISO)   ──────→  SDO / MISO       [ niebieski ] (opcjonalnie)\n";
    html += "GPIO 15 (CS)     ──────→  CS               [ pomarańczowy ]\n";
    html += "GPIO 2  (DC/RS)  ──────→  DC / RS           [ biały ]\n";
    html += "\nZASILANIE:\n";
    html += "3.3V             ──────→  VCC              [ czerwony ]\n";
    html += "GND              ──────→  GND              [ czarny ]\n";
    html += "3.3V             ──────→  LED (podświetlenie) [ przez rezystor 100Ω ]\n";
    html += "</pre>";

    html += "<h3>Pomiar napięcia baterii (dzielnik)</h3>";
    html += "<pre>";
    html += "Bateria 18650 (+) ────┬─── [R1 100kΩ] ───┬───→ GPIO 34 (ADC)\n";
    html += "                      │                 │\n";
    html += "                     GND              [R2 100kΩ] ───┬─── GND\n";
    html += "                                                   │\n";
    html += "                                              Kondensator 100nF\n";
    html += "</pre>";
    html += "<div class='note'><strong>Formuła:</strong> Vbat = Vadc × 2 (przy R1 = R2 = 100kΩ)</div>";

    html += "<h3>Podłączenie dotyku XPT2046 (opcjonalnie)</h3>";
    html += "<pre>";
    html += "ESP32                   XPT2046 (dotyk)\n";
    html += "══════════════════════════════════════════\n";
    html += "GPIO 4   ──────→  T_IRQ\n";
    html += "GPIO 12  ──────→  T_DO\n";
    html += "GPIO 13  ──────→  T_DIN\n";
    html += "GPIO 14  ──────→  T_CS\n";
    html += "GPIO 25  ──────→  T_CLK\n";
    html += "</pre>";

    // Sekcja wgrania
    html += "<h2>💾 Wgrywanie Firmware</h2>";

    html += "<h3>Pliki w folderze <code>pliki.bin/</code></h3>";
    html += "<table>";
    html += "<tr><th>Plik</th><th>Adres</th><th>Opis</th></tr>";
    html += "<tr><td><code>bootloader.bin</code></td><td><code>0x1000</code></td><td>Bootloader ESP32</td></tr>";
    html += "<tr><td><code>partitions.bin</code></td><td><code>0x8000</code></td><td>Tablica partycji</td></tr>";
    html += "<tr><td><code>firmware.bin</code></td><td><code>0x10000</code></td><td>Główny program</td></tr>";
    html += "<tr><td><code>littlefs.bin</code></td><td><code>0x290000</code></td><td>System plików (ikony, fonty)</td></tr>";
    html += "</table>";

    html += "<h3>Sposób 1: WebSerial ESPTool (przeglądarka) - NAJPROSTSZY</h3>";
    html += "<div class='success'>✅ <strong>Najprostsza metoda - bez instalowania żadnych programów!</strong></div>";
    html += "<ol>";
    html += "<li>Otwórz stronę: <a href='https://jason2866.github.io/WebSerial_ESPTool/' target='_blank'>https://jason2866.github.io/WebSerial_ESPTool/</a></li>";
    html += "<li>Podłącz ESP32 przez USB</li>";
    html += "<li>Kliknij <strong>Connect</strong> i wybierz port COM</li>";
    html += "<li>Przytrzymaj <strong>BOOT</strong>, naciśnij <strong>RST</strong>, puść <strong>BOOT</strong></li>";
    html += "<li>Wybierz pliki do wgrania (bootloader.bin, partitions.bin, firmware.bin, littlefs.bin)</li>";
    html += "<li>Kliknij <strong>Program</strong> i czekaj (~2-3 minuty)</li>";
    html += "<li>Po zakończeniu naciśnij <strong>RST</strong> na ESP32</li>";
    html += "</ol>";

    html += "<h3>Sposób 2: ESP Flash Download Tool (Windows)</h3>";
    html += "<ol>";
    html += "<li>Pobierz <strong>ESP Flash Download Tool</strong> z: https://www.espressif.com/en/support/download/other-tools</li>";
    html += "<li>Wybierz chip: <strong>ESP32</strong>, COM port i BAUD: <strong>921600</strong></li>";
    html += "<li>Dodaj pliki z adresami:<br>";
    html += "<code>bootloader.bin @ 0x1000</code><br>";
    html += "<code>partitions.bin @ 0x8000</code><br>";
    html += "<code>firmware.bin @ 0x10000</code><br>";
    html += "<code>littlefs.bin @ 0x290000</code></li>";
    html += "<li>Zaznacz checkbox przy każdym pliku</li>";
    html += "<li>Kliknij <strong>START</strong> (przytrzymaj BOOT i naciśnij RST)</li>";
    html += "</ol>";

    html += "<h3>Sposób 3: esptool.py (Windows/Linux/Mac)</h3>";
    html += "<pre>";
    html += "pip install esptool\n\n";
    html += "esptool.py --chip esp32 --port COM7 --baud 921600 write_flash \\\n";
    html += "  0x1000 bootloader.bin \\\n";
    html += "  0x8000 partitions.bin \\\n";
    html += "  0x10000 firmware.bin \\\n";
    html += "  0x290000 littlefs.bin\n";
    html += "</pre>";

    html += "<h3>Sposób 4: PlatformIO (VS Code)</h3>";
    html += "<pre>pio run --target upload</pre>";

    // Troubleshooting
    html += "<h2>🔧 Troubleshooting</h2>";

    html += "<div class='warning'>⚠️ <strong>\"Failed to connect to ESP32\"</strong><br>";
    html += "Przytrzymaj przycisk <strong>BOOT</strong> (GPIO0), naciśnij i puść <strong>RST</strong> (EN), puść <strong>BOOT</strong>. ESP32 wejdzie w tryb bootloadera.</div>";

    html += "<div class='warning'>⚠️ <strong>Biały ekran TFT</strong><br>";
    html += "Sprawdź połączenie zasilania i podświetlenia LED. Upewnij się że LED jest podłączony do 3.3V (lub 5V przez rezystor 100Ω).</div>";

    html += "<div class='warning'>⚠️ <strong>Brak obrazu na TFT</strong><br>";
    html += "Sprawdź połączenie SPI: SCK (GPIO18), MOSI (GPIO23), CS (GPIO15), DC (GPIO2). Upewnij się że masz wspólne GND.</div>";

    html += "<div class='note'>💡 <strong>Nie działa touch (XPT2046)</strong><br>";
    html += "XPT2046 używa osobnego SPI. Sprawdź połączenia: T_CLK (GPIO25), T_CS (GPIO14), T_DIN (GPIO13), T_DO (GPIO12), T_IRQ (GPIO4).</div>";

    // Stopka
    html += "<hr style='margin:40px 0;'>";
    html += "<p style='text-align:center;color:#666;'>";
    html += "<strong>ESP32-HAM-CLOCK</strong> | Wersja 1.3 | Autor: SP3KON<br>";
    html += "Modyfikacje: SP9TNV | Licencja: MIT<br>";
    html += "<a href='mailto:sp3kon@gmail.com'>sp3kon@gmail.com</a>";
    html += "</p>";

    html += "<a href='/' class='back-btn'>← Powrót do głównej</a>";
    html += "</body></html>";

    server->send(200, "text/html; charset=utf-8", html);
  });

  // Strona konfiguracji - zawsze używa wbudowanego HTML z poprawnymi nazwami ekranów
  server->on("/config", HTTP_GET, []() {
    server->sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    server->sendHeader("Pragma", "no-cache");
    server->sendHeader("Expires", "0");
    server->send(200, "text/html; charset=utf-8", getConfigHTML());
  });
  
  // API - pobierz wszystkie spoty (maksymalnie 50)
  server->on("/api/spots", HTTP_GET, []() {
    // ZwiĂ„â„˘kszony bufor JSON dla 50 spotÄ‚łw (kaÄąÄ˝dy spot ~150-200 bajtÄ‚łw)
    StaticJsonDocument<12000> doc;  // 50 spotÄ‚łw * ~200 bajtÄ‚łw = ~10KB + margines
    JsonArray spotsArray = doc.createNestedArray("spots");
    
    // ZwrÄ‚łĂ„â€ˇ wszystkie spoty (maksymalnie 50)
    for (int i = 0; i < spotCount; i++) {
      JsonObject spotObj = spotsArray.createNestedObject();
      spotObj["time"] = spots[i].time;
      spotObj["callsign"] = spots[i].callsign;
      spotObj["frequency"] = spots[i].frequency;
      spotObj["distance"] = spots[i].distance;
      spotObj["country"] = spots[i].country;
      spotObj["band"] = spots[i].band;
      spotObj["mode"] = spots[i].mode;
      spotObj["spotter"] = spots[i].spotter;
      spotObj["comment"] = spots[i].comment;
    }
    
    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });
  
  // API - pobierz stacje APRS
  server->on("/api/aprs", HTTP_GET, []() {
    // Bufor JSON dla 20 stacji APRS (kaÄąÄ˝da stacja ~200-250 bajtÄ‚łw)
    StaticJsonDocument<6000> doc;  // 20 stacji * ~250 bajtÄ‚łw = ~5KB + margines
    JsonArray aprsArray = doc.createNestedArray("stations");
    
    // ZwrÄ‚łĂ„â€ˇ wszystkie stacje APRS (maksymalnie 20)
    for (int i = 0; i < aprsStationCount; i++) {
      JsonObject stationObj = aprsArray.createNestedObject();
      stationObj["time"] = aprsStations[i].time;
      stationObj["callsign"] = aprsStations[i].callsign;
      stationObj["symbol"] = getAPRSSymbolShort(aprsStations[i]);
      stationObj["lat"] = aprsStations[i].lat;
      stationObj["lon"] = aprsStations[i].lon;
      stationObj["comment"] = aprsStations[i].comment;
      stationObj["distance"] = aprsStations[i].distance;
      stationObj["hasLatLon"] = aprsStations[i].hasLatLon;
      stationObj["freq_mhz"] = aprsStations[i].freqMHz;
    }
    
    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });

  // API - POTA (bufor z backendu, z krajami z QRZ jeśli dostępne)
  server->on("/api/pota", HTTP_GET, []() {
    StaticJsonDocument<12000> doc; // do 30 spotów * ~300-350 bajtów
    JsonArray spotsArray = doc.createNestedArray("spots");
    const int apiSpotLimit = 30;
    const int potaApiCount = (potaSpotCount < apiSpotLimit) ? potaSpotCount : apiSpotLimit;
    for (int i = 0; i < potaApiCount; i++) {
      JsonObject spotObj = spotsArray.createNestedObject();
      spotObj["time"] = potaSpots[i].time;
      spotObj["spotTime"] = potaSpots[i].time; // zgodność z POTA API
      spotObj["callsign"] = potaSpots[i].callsign;
      spotObj["frequency"] = potaSpots[i].frequency;
      spotObj["mode"] = potaSpots[i].mode;
      spotObj["country"] = potaSpots[i].country;
      spotObj["spotter"] = potaSpots[i].spotter;
      spotObj["comment"] = potaSpots[i].comment;
      spotObj["band"] = potaSpots[i].band;
    }
    doc["count"] = potaApiCount;
    doc["total"] = potaSpotCount;
    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });

  // API - HAMALERT (ostatnie spoty z telnetu)
  server->on("/api/hamalert", HTTP_GET, []() {
    StaticJsonDocument<12000> doc; // do 30 spotów
    JsonArray spotsArray = doc.createNestedArray("spots");
    const int apiSpotLimit = 30;
    const int hamalertApiCount = (hamalertSpotCount < apiSpotLimit) ? hamalertSpotCount : apiSpotLimit;
    for (int i = 0; i < hamalertApiCount; i++) {
      JsonObject spotObj = spotsArray.createNestedObject();
      spotObj["time"] = hamalertSpots[i].time;
      spotObj["spotTime"] = hamalertSpots[i].time;
      spotObj["callsign"] = hamalertSpots[i].callsign;
      spotObj["frequency"] = hamalertSpots[i].frequency;
      spotObj["mode"] = hamalertSpots[i].mode;
      spotObj["country"] = hamalertSpots[i].country;
      spotObj["spotter"] = hamalertSpots[i].spotter;
      spotObj["comment"] = hamalertSpots[i].comment;
      spotObj["band"] = hamalertSpots[i].band;
    }
    doc["count"] = hamalertApiCount;
    doc["total"] = hamalertSpotCount;
    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });
  
  // API - pobierz spoty PSK Reporter
  server->on("/api/psk", HTTP_GET, []() {
    StaticJsonDocument<10000> doc;
    JsonArray spotsArray = doc.createNestedArray("spots");

    for (int i = 0; i < pskSpotCount; i++) {
      JsonObject spotObj = spotsArray.createNestedObject();
      spotObj["callsign"] = pskSpots[i].callsign;
      spotObj["lat"] = pskSpots[i].lat;
      spotObj["lon"] = pskSpots[i].lon;
      spotObj["band"] = pskSpots[i].band;
      spotObj["mode"] = pskSpots[i].mode;
      spotObj["snr"] = pskSpots[i].snr;
    }

    doc["count"] = pskSpotCount;
    doc["lastFetchMs"] = lastPskFetchMs;

    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });

  // API - pobierz dane telemetryczne ISS
  server->on("/api/iss", HTTP_GET, []() {
    StaticJsonDocument<512> doc;
    doc["lat"] = issLat;
    doc["lng"] = issLng;
    doc["altitude"] = issAltitude;
    doc["speed"] = issSpeed;
    doc["azimuth"] = issAzimuth;
    doc["elevation"] = issElevation;
    doc["distance"] = issDistance;
    doc["country"] = issCountry;

    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });

  // API - zapisz konfiguracjĂ„â„˘
  server->on("/api/save", HTTP_POST, []() {
    if (server->hasArg("plain")) {
      String body = server->arg("plain");
      
        StaticJsonDocument<8192> doc;  // Bufor dla pełnego payloadu konfiguracji z WWW
      DeserializationError err = deserializeJson(doc, body);
      if (err) {
        server->send(400, "application/json", "{\"status\":\"error\",\"message\":\"json_parse\"}");
        return;
      }
      
      wifiSSID = doc["wifi_ssid"].as<String>();
      wifiPassword = doc["wifi_pass"].as<String>();
      if (doc["wifi_ssid2"].is<String>()) {
        wifiSSID2 = doc["wifi_ssid2"].as<String>();
      }
      if (doc["wifi_pass2"].is<String>()) {
        wifiPassword2 = doc["wifi_pass2"].as<String>();
      }
      clusterHost = doc["cluster_host"].as<String>();
      if (doc["pota_host"].is<String>()) {
        potaClusterHost = doc["pota_host"].as<String>();
      }
      if (doc["pota_filter"].is<String>()) {
        potaFilterCommand = doc["pota_filter"].as<String>();
      }
      if (doc["pota_api_url"].is<String>()) {
        potaApiUrl = doc["pota_api_url"].as<String>();
      }
      if (doc["hamalert_host"].is<String>()) {
        hamalertHost = doc["hamalert_host"].as<String>();
      }
      if (doc["hamalert_port"].is<int>()) {
        hamalertPort = doc["hamalert_port"].as<int>();
      }
      if (doc["hamalert_login"].is<String>()) {
        hamalertLogin = doc["hamalert_login"].as<String>();
      }
      if (doc["hamalert_password"].is<String>()) {
        hamalertPassword = doc["hamalert_password"].as<String>();
      }
      userCallsign = doc["user_callsign"].as<String>();
      userLocator = doc["user_locator"].as<String>();
      if (doc["timezone"].is<int>()) {
        timezoneHours = doc["timezone"].as<int>();
        if (timezoneHours < -12) timezoneHours = -12;
        if (timezoneHours > 14) timezoneHours = 14;
      }
      
      // WspÄ‚łÄąâ€šrzĂ„â„˘dne geograficzne (LAT/LON)
      bool latProvided = false;
      bool lonProvided = false;
      if (doc["user_lat"].is<float>() || doc["user_lat"].is<double>()) {
        userLat = doc["user_lat"].as<double>();
        latProvided = true;
      }
      if (doc["user_lon"].is<float>() || doc["user_lon"].is<double>()) {
        userLon = doc["user_lon"].as<double>();
        lonProvided = true;
      }
      // JeÄąâ€şli podano obie wspÄ‚łÄąâ€šrzĂ„â„˘dne, ustaw flagĂ„â„˘ jako waÄąÄ˝ne
      // Uwaga: sprawdzamy czy wartoÄąâ€şci sĂ„â€¦ w prawidÄąâ€šowym zakresie (nie tylko != 0)
      if (latProvided && lonProvided && 
          userLat >= -90.0 && userLat <= 90.0 && 
          userLon >= -180.0 && userLon <= 180.0) {
        userLatLonValid = true;
      } else if (latProvided || lonProvided) {
        // JeÄąâ€şli podano tylko jednĂ„â€¦ wspÄ‚łÄąâ€šrzĂ„â„˘dnĂ„â€¦, uznaj za nieprawidÄąâ€šowe
        userLatLonValid = false;
      }
      
      if (doc["qrz_user"].is<String>()) {
        qrzUsername = doc["qrz_user"].as<String>();
      }
      if (doc["qrz_pass"].is<String>()) {
        qrzPassword = doc["qrz_pass"].as<String>();
      }
      if (doc["weather_key"].is<String>()) {
        weatherApiKey = doc["weather_key"].as<String>();
      }
      if (doc["openwebrx_url"].is<String>()) {
        openWebRxUrl = doc["openwebrx_url"].as<String>();
      }
      if (doc["tft_backlight"].is<int>()) {
        backlightPercent = doc["tft_backlight"].as<int>();
        if (backlightPercent < MIN_BACKLIGHT_PERCENT) backlightPercent = MIN_BACKLIGHT_PERCENT;
        if (backlightPercent > 100) backlightPercent = 100;
      }
      if (doc["tft_invert"].is<bool>()) {
        tftInvertColors = sanitizeTftInvertSetting(doc["tft_invert"].as<bool>());
      }
      if (doc["callsign_color"].is<int>()) {
        callsignColor = (uint16_t)doc["callsign_color"].as<int>();
      }
      if (doc["tft_lang"].is<String>()) {
        tftLanguage = tftLangFromCode(doc["tft_lang"].as<String>());
      } else if (doc["tft_lang"].is<int>()) {
        int lang = doc["tft_lang"].as<int>();
        tftLanguage = (lang == TFT_LANG_EN) ? TFT_LANG_EN : TFT_LANG_PL;
      }
      if (doc["table_size"].is<String>()) {
        dxTableSizeMode = dxTableSizeFromCode(doc["table_size"].as<String>());
      } else if (doc["table_size"].is<int>()) {
        int sizeMode = doc["table_size"].as<int>();
        dxTableSizeMode = (sizeMode == DX_TABLE_SIZE_ENLARGED) ? DX_TABLE_SIZE_ENLARGED : DX_TABLE_SIZE_NORMAL;
      }
      if (doc["menu_hue"].is<int>()) {
        int hue = doc["menu_hue"].as<int>();
        if (hue < 0) hue = 0;
        if (hue > 255) hue = 255;
        menuThemeHue = (uint8_t)hue;
        applyMenuThemeFromHue();
      }
      
      // Konfiguracja filtrÄ‚łw CC-Cluster
      if (doc["cluster_noann"].is<bool>()) {
        clusterNoAnnouncements = doc["cluster_noann"].as<bool>();
      }
      if (doc["cluster_nowwv"].is<bool>()) {
        clusterNoWWV = doc["cluster_nowwv"].as<bool>();
      }
      if (doc["cluster_nowcy"].is<bool>()) {
        clusterNoWCY = doc["cluster_nowcy"].as<bool>();
      }
      if (doc["cluster_usefilters"].is<bool>()) {
        clusterUseFilters = doc["cluster_usefilters"].as<bool>();
      }
      if (doc["cluster_filters"].is<String>()) {
        clusterFilterCommands = doc["cluster_filters"].as<String>();
      }
      
      // Konfiguracja APRS-IS
      if (doc["aprs_host"].is<String>()) {
        aprsIsHost = doc["aprs_host"].as<String>();
      }
      if (doc["aprs_port"].is<int>()) {
        aprsIsPort = doc["aprs_port"].as<int>();
      }
      if (doc["aprs_callsign"].is<String>()) {
        aprsCallsign = doc["aprs_callsign"].as<String>();
      }
      if (doc["aprs_passcode"].is<int>()) {
        aprsPasscode = doc["aprs_passcode"].as<int>();
      }
      if (doc["aprs_ssid"].is<int>()) {
        applyAprsSsid(doc["aprs_ssid"].as<int>());
      }
      if (doc["aprs_beacon"].is<bool>()) {
        aprsBeaconEnabled = doc["aprs_beacon"].as<bool>();
      }
      if (doc["aprs_symbol"].is<String>()) {
        applyAprsSymbol(doc["aprs_symbol"].as<String>());
      }
      if (doc["aprs_comment"].is<String>()) {
        aprsUserComment = sanitizeAprsComment(doc["aprs_comment"].as<String>());
      }
      bool aprsAlertConfigChanged = false;
      if (doc["aprs_alert"].is<String>()) {
        aprsAlertCsv = sanitizeAprsAlertList(doc["aprs_alert"].as<String>());
        aprsAlertConfigChanged = true;
      }
      if (doc["aprs_alert_enabled"].is<bool>()) {
        aprsAlertEnabled = doc["aprs_alert_enabled"].as<bool>();
      }
      if (doc["aprs_alert_nearby_enabled"].is<bool>()) {
        aprsAlertNearbyEnabled = doc["aprs_alert_nearby_enabled"].as<bool>();
      }
      if (doc["aprs_alert_wx_enabled"].is<bool>()) {
        aprsAlertWxEnabled = doc["aprs_alert_wx_enabled"].as<bool>();
        aprsAlertConfigChanged = true;
      }
      if (doc.containsKey("aprs_alert_min_sec")) {
        int alertMinSecCandidate = aprsAlertMinSeconds;
        if (doc["aprs_alert_min_sec"].is<int>()) {
          alertMinSecCandidate = doc["aprs_alert_min_sec"].as<int>();
        } else if (doc["aprs_alert_min_sec"].is<long>()) {
          alertMinSecCandidate = (int)doc["aprs_alert_min_sec"].as<long>();
        } else if (doc["aprs_alert_min_sec"].is<float>() || doc["aprs_alert_min_sec"].is<double>()) {
          alertMinSecCandidate = (int)doc["aprs_alert_min_sec"].as<double>();
        } else if (doc["aprs_alert_min_sec"].is<String>()) {
          String alertMinStr = doc["aprs_alert_min_sec"].as<String>();
          alertMinStr.trim();
          alertMinSecCandidate = alertMinStr.toInt();
        }
        applyAprsAlertMinSeconds(alertMinSecCandidate);
        aprsAlertConfigChanged = true;
      }
      if (doc.containsKey("aprs_alert_screen_sec")) {
        int alertScreenSecCandidate = aprsAlertScreenSeconds;
        if (doc["aprs_alert_screen_sec"].is<int>()) {
          alertScreenSecCandidate = doc["aprs_alert_screen_sec"].as<int>();
        } else if (doc["aprs_alert_screen_sec"].is<long>()) {
          alertScreenSecCandidate = (int)doc["aprs_alert_screen_sec"].as<long>();
        } else if (doc["aprs_alert_screen_sec"].is<float>() || doc["aprs_alert_screen_sec"].is<double>()) {
          alertScreenSecCandidate = (int)doc["aprs_alert_screen_sec"].as<double>();
        } else if (doc["aprs_alert_screen_sec"].is<String>()) {
          String alertScreenSecStr = doc["aprs_alert_screen_sec"].as<String>();
          alertScreenSecStr.trim();
          alertScreenSecCandidate = alertScreenSecStr.toInt();
        }
        applyAprsAlertScreenSeconds(alertScreenSecCandidate);
      }
      if (doc.containsKey("aprs_alert_distance_km")) {
        float alertDistanceCandidate = aprsAlertDistanceKm;
        if (doc["aprs_alert_distance_km"].is<float>() || doc["aprs_alert_distance_km"].is<double>()) {
          alertDistanceCandidate = (float)doc["aprs_alert_distance_km"].as<double>();
        } else if (doc["aprs_alert_distance_km"].is<int>()) {
          alertDistanceCandidate = (float)doc["aprs_alert_distance_km"].as<int>();
        } else if (doc["aprs_alert_distance_km"].is<long>()) {
          alertDistanceCandidate = (float)doc["aprs_alert_distance_km"].as<long>();
        } else if (doc["aprs_alert_distance_km"].is<String>()) {
          String alertDistanceStr = doc["aprs_alert_distance_km"].as<String>();
          alertDistanceStr.trim();
          alertDistanceStr.replace(',', '.');
          alertDistanceCandidate = alertDistanceStr.toFloat();
        }
        applyAprsAlertDistanceKm(alertDistanceCandidate);
      }
      if (doc["enable_led_alert"].is<bool>()) {
        enableLedAlert = doc["enable_led_alert"].as<bool>();
      }
      if (doc.containsKey("led_alert_duration_ms")) {
        int ledDurationCandidate = ledAlertDurationMs;
        if (doc["led_alert_duration_ms"].is<int>()) {
          ledDurationCandidate = doc["led_alert_duration_ms"].as<int>();
        } else if (doc["led_alert_duration_ms"].is<long>()) {
          ledDurationCandidate = (int)doc["led_alert_duration_ms"].as<long>();
        } else if (doc["led_alert_duration_ms"].is<float>() || doc["led_alert_duration_ms"].is<double>()) {
          ledDurationCandidate = (int)doc["led_alert_duration_ms"].as<double>();
        } else if (doc["led_alert_duration_ms"].is<String>()) {
          String ledDurationStr = doc["led_alert_duration_ms"].as<String>();
          ledDurationStr.trim();
          ledDurationCandidate = ledDurationStr.toInt();
        }
        applyLedAlertDurationMs(ledDurationCandidate);
      }
      if (doc.containsKey("led_alert_blink_ms")) {
        int ledBlinkCandidate = ledAlertBlinkMs;
        if (doc["led_alert_blink_ms"].is<int>()) {
          ledBlinkCandidate = doc["led_alert_blink_ms"].as<int>();
        } else if (doc["led_alert_blink_ms"].is<long>()) {
          ledBlinkCandidate = (int)doc["led_alert_blink_ms"].as<long>();
        } else if (doc["led_alert_blink_ms"].is<float>() || doc["led_alert_blink_ms"].is<double>()) {
          ledBlinkCandidate = (int)doc["led_alert_blink_ms"].as<double>();
        } else if (doc["led_alert_blink_ms"].is<String>()) {
          String ledBlinkStr = doc["led_alert_blink_ms"].as<String>();
          ledBlinkStr.trim();
          ledBlinkCandidate = ledBlinkStr.toInt();
        }
        applyLedAlertBlinkMs(ledBlinkCandidate);
      }
      if (doc.containsKey("aprs_interval_min")) {
        int intervalCandidate = aprsIntervalMinutes;
        if (doc["aprs_interval_min"].is<int>()) {
          intervalCandidate = doc["aprs_interval_min"].as<int>();
        } else if (doc["aprs_interval_min"].is<long>()) {
          intervalCandidate = (int)doc["aprs_interval_min"].as<long>();
        } else if (doc["aprs_interval_min"].is<float>() || doc["aprs_interval_min"].is<double>()) {
          intervalCandidate = (int)doc["aprs_interval_min"].as<double>();
        } else if (doc["aprs_interval_min"].is<String>()) {
          String intervalStr = doc["aprs_interval_min"].as<String>();
          intervalStr.trim();
          intervalCandidate = intervalStr.toInt();
        }
        applyAprsIntervalMinutes(intervalCandidate);
      }
      if (aprsAlertConfigChanged) {
        resetAprsAlertCooldownState();
      }
      if (doc["aprs_radius"].is<int>()) {
        aprsFilterRadius = doc["aprs_radius"].as<int>();
        // Ograniczenie promienia do 1-50 km
        if (aprsFilterRadius < 1) aprsFilterRadius = 1;
        if (aprsFilterRadius > 50) aprsFilterRadius = 50;
      }
      // Uwaga: APRS uÄąÄ˝ywa wspÄ‚łÄąâ€šrzĂ„â„˘dnych z sekcji "Moja Stacja" (userLat, userLon) - nie ma osobnych pÄ‚łl
      
      // Konfiguracja trybu kalibracji dotyku (ręczne nadpisanie)
      if (doc["touch_swap_mode"].is<String>()) {
        String mode = doc["touch_swap_mode"].as<String>();
        // Resetuj wszystkie flagi
        touchSwapXY = false;
        touchInvertX = false;
        touchInvertY = false;
        // Ustaw odpowiednią flagę lub kombinację
        if (mode == "xy") {
          touchSwapXY = true;
        } else if (mode == "x") {
          touchInvertX = true;
        } else if (mode == "y") {
          touchInvertY = true;
        } else if (mode == "both") {
          touchInvertX = true;
          touchInvertY = true;
        } else if (mode == "rot90cw") {
          // Obrót 90Â° w prawo (clockwise)
          touchSwapXY = true;
          touchInvertX = true;
        } else if (mode == "rot90ccw") {
          // Obrót 90Â° w lewo (counter-clockwise)
          touchSwapXY = true;
          touchInvertY = true;
        } else if (mode == "xy_both") {
          touchSwapXY = true;
          touchInvertX = true;
          touchInvertY = true;
        }
        // "none" pozostawia wszystkie false
        Serial.print("Touch swap mode set to: ");
        Serial.println(mode);
      }
      
      // Touch rotation (0-3)
      if (doc["touch_rotation"].is<int>()) {
        int rot = doc["touch_rotation"].as<int>();
        if (rot >= 0 && rot <= 3) {
          touchRotation = (uint8_t)rot;
          Serial.print("Touch rotation set to: ");
          Serial.println(touchRotation);
        }
      }

      // TFT rotation (0-3)
      if (doc["tft_rotation"].is<int>()) {
        int rot = doc["tft_rotation"].as<int>();
        if (rot >= 0 && rot <= 3) {
          tftRotation = (uint8_t)rot;
          touchRotation = tftRotation; // Synchronizuj rotację dotyku z TFT
          Serial.print("TFT rotation set to: ");
          Serial.print(tftRotation);
          Serial.print(", touch rotation synchronized to: ");
          Serial.println(touchRotation);
        }
      }

      // Indywidualne parametry kalibracji dotyku (xmin, xmax, ymin, ymax)
      if (doc["touch_xmin"].is<int>()) {
        int val = doc["touch_xmin"].as<int>();
        if (val >= 0 && val <= 4095) touchXMin = val;
      }
      if (doc["touch_xmax"].is<int>()) {
        int val = doc["touch_xmax"].as<int>();
        if (val >= 0 && val <= 4095) touchXMax = val;
      }
      if (doc["touch_ymin"].is<int>()) {
        int val = doc["touch_ymin"].as<int>();
        if (val >= 0 && val <= 4095) touchYMin = val;
      }
      if (doc["touch_ymax"].is<int>()) {
        int val = doc["touch_ymax"].as<int>();
        if (val >= 0 && val <= 4095) touchYMax = val;
      }
      // Individual touch flags (override touch_swap_mode if provided)
      if (doc["touch_swap"].is<bool>()) {
        touchSwapXY = doc["touch_swap"].as<bool>();
      }
      if (doc["touch_invx"].is<bool>()) {
        touchInvertX = doc["touch_invx"].as<bool>();
      }
      if (doc["touch_invy"].is<bool>()) {
        touchInvertY = doc["touch_invy"].as<bool>();
      }

      // Ustawienia PSKReporter
      if (doc["psk_receiver"].is<String>()) {
        pskReceiverCallsign = doc["psk_receiver"].as<String>();
        pskReceiverCallsign.trim();
        pskReceiverCallsign.toUpperCase();
      }
      if (doc["psk_maxspots"].is<int>()) {
        pskMaxSpots = doc["psk_maxspots"].as<int>();
        if (pskMaxSpots < 10) pskMaxSpots = 10;
        if (pskMaxSpots > 100) pskMaxSpots = 100;
      }
      if (doc["psk_hours"].is<int>()) {
        pskHoursWindow = doc["psk_hours"].as<int>();
        if (pskHoursWindow < 1) pskHoursWindow = 1;
        if (pskHoursWindow > 24) pskHoursWindow = 24;
      }
      if (doc["psk_band"].is<String>()) {
        pskFilterBand = doc["psk_band"].as<String>();
      }
      if (doc["psk_mode"].is<String>()) {
        pskFilterMode = doc["psk_mode"].as<String>();
        pskFilterMode.toUpperCase();
      }
      if (doc["psk_url"].is<String>()) {
        pskCustomUrl = doc["psk_url"].as<String>();
        pskCustomUrl.trim();
      }
      // HTTP Monitoring
      if (doc["psk_monitor_callsign"].is<String>()) {
        pskMonitorCallsign = doc["psk_monitor_callsign"].as<String>();
        pskMonitorCallsign.trim();
        pskMonitorCallsign.toUpperCase();
      }
      if (doc["psk_report_days"].is<int>()) {
        pskReportDays = doc["psk_report_days"].as<int>();
        if (pskReportDays < 0) pskReportDays = 0;
        if (pskReportDays > 62) pskReportDays = 62;
      }
      if (doc["psk_auto_refresh"].is<int>()) {
        pskAutoRefreshMinutes = doc["psk_auto_refresh"].as<int>();
        if (pskAutoRefreshMinutes < 0) pskAutoRefreshMinutes = 0;
        if (pskAutoRefreshMinutes > 60) pskAutoRefreshMinutes = 60;
      }
      // MQTT PSK Reporter settings
      if (doc["psk_mqtt_enabled"].is<bool>()) {
        pskMqttEnabled = doc["psk_mqtt_enabled"].as<bool>();
      }
      if (doc["psk_mqtt_server"].is<String>()) {
        pskMqttServer = doc["psk_mqtt_server"].as<String>();
        pskMqttServer.trim();
      }
      if (doc["psk_mqtt_port"].is<int>()) {
        pskMqttPort = doc["psk_mqtt_port"].as<int>();
        if (pskMqttPort <= 0 || pskMqttPort > 65535) pskMqttPort = 1883;
      }
      if (doc["psk_mqtt_callsign"].is<String>()) {
        pskMqttCallsign = doc["psk_mqtt_callsign"].as<String>();
        pskMqttCallsign.trim();
        pskMqttCallsign.toUpperCase();
      }

      // Sanitizacja (najczÄ™stsza przyczyna: spacje na koÄ…cu SSID/hasÄ…a)
      wifiSSID.trim();
      wifiPassword.trim();
      wifiSSID2.trim();
      wifiPassword2.trim();
      clusterHost.trim();
      potaClusterHost.trim();
      potaFilterCommand.trim();
      potaApiUrl.trim();
      hamalertHost.trim();
      hamalertLogin.trim();
      hamalertPassword.trim();
      userCallsign.trim();
      userLocator.trim();
      clusterFilterCommands.trim();
      weatherApiKey.trim();
      openWebRxUrl.trim();
      aprsIsHost.trim();
      aprsCallsign.trim();
      // KiwiSDR URL - może być puste (użytkownik nie musi ustawiać)

      if (potaApiUrl.length() == 0) {
        potaApiUrl = DEFAULT_POTA_API_URL;
      }
      if (hamalertHost.length() == 0) {
        hamalertHost = DEFAULT_HAMALERT_HOST;
      }
      if (hamalertPort <= 0 || hamalertPort > 65535) {
        hamalertPort = DEFAULT_HAMALERT_PORT;
      }

      // Kolejność ekranów konfigurowana z WWW
      if (doc["screen_order"].is<JsonArray>()) {
        JsonArray arr = doc["screen_order"].as<JsonArray>();
        int idx = 0;
        for (JsonVariant v : arr) {
          if (idx >= SCREEN_ORDER_COUNT) break;
          if (v.is<const char*>()) {
            String code = v.as<const char*>();
            ScreenType parsed = screenCodeToType(code);
            // Unknown code should not silently clear slot to OFF.
            if (parsed == SCREEN_OFF) {
              String tmp = code;
              tmp.toLowerCase();
              tmp.trim();
              if (tmp != "off") {
                parsed = DEFAULT_SCREEN_ORDER[idx];
              }
            }
            screenOrder[idx] = parsed;
          } else if (v.is<int>()) {
            int raw = v.as<int>();
            ScreenType parsed = normalizeScreenType(raw);
            if (parsed == SCREEN_OFF && raw != 0) {
              parsed = DEFAULT_SCREEN_ORDER[idx];
            }
            screenOrder[idx] = parsed;
          }
          idx++;
        }
        // Jeśli JSON był krótszy, pozostałe uzupełnij wartościami domyślnymi,
        // aby nie gubić ekranów (np. APRS RADAR) przy starszym/okrojonym payloadzie.
        for (; idx < SCREEN_ORDER_COUNT; idx++) {
          screenOrder[idx] = DEFAULT_SCREEN_ORDER[idx];
        }
      }
      ensureScreenOrderValid();

      if (doc["tft_auto_switch"].is<bool>()) {
        tftAutoSwitchEnabled = doc["tft_auto_switch"].as<bool>();
      }
      if (doc.containsKey("tft_switch_time_sec")) {
        int switchTimeCandidate = tftAutoSwitchTimeSec;
        if (doc["tft_switch_time_sec"].is<int>()) {
          switchTimeCandidate = doc["tft_switch_time_sec"].as<int>();
        } else if (doc["tft_switch_time_sec"].is<long>()) {
          switchTimeCandidate = (int)doc["tft_switch_time_sec"].as<long>();
        } else if (doc["tft_switch_time_sec"].is<float>() || doc["tft_switch_time_sec"].is<double>()) {
          switchTimeCandidate = (int)doc["tft_switch_time_sec"].as<double>();
        } else if (doc["tft_switch_time_sec"].is<String>()) {
          String switchTimeStr = doc["tft_switch_time_sec"].as<String>();
          switchTimeStr.trim();
          switchTimeCandidate = switchTimeStr.toInt();
        }
        applyTftAutoSwitchTimeSec(switchTimeCandidate);
      }
      resetTftAutoSwitchTimer();

      // Ustawienia wygaszacza ekranu (Matrix)
      if (doc["screen_saver_enabled"].is<bool>()) {
        screenSaverEnabled = doc["screen_saver_enabled"].as<bool>();
      }
      if (doc.containsKey("screen_saver_timeout_min")) {
        int timeoutCandidate = screenSaverTimeoutMin;
        if (doc["screen_saver_timeout_min"].is<int>()) {
          timeoutCandidate = doc["screen_saver_timeout_min"].as<int>();
        } else if (doc["screen_saver_timeout_min"].is<long>()) {
          timeoutCandidate = (int)doc["screen_saver_timeout_min"].as<long>();
        } else if (doc["screen_saver_timeout_min"].is<float>() || doc["screen_saver_timeout_min"].is<double>()) {
          timeoutCandidate = (int)doc["screen_saver_timeout_min"].as<double>();
        } else if (doc["screen_saver_timeout_min"].is<String>()) {
          String timeoutStr = doc["screen_saver_timeout_min"].as<String>();
          timeoutStr.trim();
          timeoutCandidate = timeoutStr.toInt();
        }
        // Zastosuj limity bez wywoływania savePreferences()
        if (timeoutCandidate < 1) timeoutCandidate = 1;
        if (timeoutCandidate > 60) timeoutCandidate = 60;
        screenSaverTimeoutMin = timeoutCandidate;
      }

      // Port bywa null, jeśli w JS wyszedł NaN
      if (doc["cluster_port"].is<int>()) {
        clusterPort = doc["cluster_port"].as<int>();
      } else {
        clusterPort = DEFAULT_CLUSTER_PORT;
      }
      if (doc["pota_port"].is<int>()) {
        potaClusterPort = doc["pota_port"].as<int>();
      } else {
        potaClusterPort = DEFAULT_POTA_CLUSTER_PORT;
      }

      if (wifiSSID.length() == 0 && wifiSSID2.length() == 0) {
        server->send(400, "application/json", "{\"status\":\"error\",\"message\":\"empty_ssid\"}");
        return;
      }

      // Aktualizuj współrzędne z locatora TYLKO jeśli nie podano bezpośrednio współrzędnych
      // Priorytet: jeÄąâ€şli userLatLonValid == true (podano bezpoÄąâ€şrednio LAT/LON), nie nadpisuj z locatora
      if (!userLatLonValid && userLocator.length() >= 4) {
        updateUserLatLonFromLocator();
      }
      savePreferences();
      setBacklightPercent(backlightPercent);
      if (tftInitialized) {
        applyTftInversion();
      }
      
      server->send(200, "application/json", "{\"status\":\"ok\",\"action\":\"restart\"}");
      
      // W trybie AP (wifiConnected=false) bez restartu nigdy nie przejdziemy na STA.
      // Restart jest najpewniejszy i upraszcza flow.
      Serial.println("Zapisano konfiguracjĂ„â„˘. Restart za chwilĂ„â„˘...");
      requestRestart(1500);
    } else {
      server->send(400, "application/json", "{\"status\":\"error\"}");
    }
  });
  
  // API - resetuj kalibrację dotyku do wartości domyślnych
  server->on("/api/reset_touch", HTTP_POST, []() {
    touchXMin = TOUCH_X_MIN;
    touchXMax = TOUCH_X_MAX;
    touchYMin = TOUCH_Y_MIN;
    touchYMax = TOUCH_Y_MAX;
    touchSwapXY = TOUCH_SWAP_XY;
    touchInvertX = TOUCH_INVERT_X;
    touchInvertY = TOUCH_INVERT_Y;
    touchRotation = 1;
    applyTouchRotation();
    
    savePreferences();
    
    Serial.println("Touch calibration reset to defaults");
    server->send(200, "application/json", "{\"status\":\"ok\"}");
  });

  // API - resetuj rotację TFT do wartości domyślnych
  server->on("/api/reset_tft_rotation", HTTP_POST, []() {
    tftRotation = 1;
    applyTftRotation();
    savePreferences();

    Serial.println("TFT rotation reset to default");
    server->send(200, "application/json", "{\"status\":\"ok\",\"action\":\"restart\"}");
    requestRestart(1500);
  });

  // API - usuń plik z LittleFS
  server->on("/api/delete_file", HTTP_GET, []() {
    if (!server->hasArg("path")) {
      server->send(400, "application/json", "{\"status\":\"error\",\"message\":\"missing_path\"}");
      return;
    }
    String path = server->arg("path");
    if (!path.startsWith("/")) {
      path = "/" + path;
    }
    if (!littleFsReady) {
      server->send(500, "application/json", "{\"status\":\"error\",\"message\":\"littlefs_not_ready\"}");
      return;
    }
    if (!LittleFS.exists(path)) {
      server->send(404, "application/json", "{\"status\":\"error\",\"message\":\"file_not_found\"}");
      return;
    }
    if (LittleFS.remove(path)) {
      server->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"file_deleted\"}");
      Serial.println("File deleted: " + path);
    } else {
      server->send(500, "application/json", "{\"status\":\"error\",\"message\":\"delete_failed\"}");
    }
  });

  // API - usuń wszystkie pliki HTML z LittleFS aby wymusić ładowanie z firmware
  server->on("/api/clear_html_cache", HTTP_POST, []() {
    if (!littleFsReady) {
      server->send(500, "application/json", "{\"status\":\"error\",\"message\":\"littlefs_not_ready\"}");
      return;
    }
    int deleted = 0;
    if (LittleFS.exists("/index.html")) {
      if (LittleFS.remove("/index.html")) deleted++;
    }
    if (LittleFS.exists("/indexEN.html")) {
      if (LittleFS.remove("/indexEN.html")) deleted++;
    }
    if (LittleFS.exists("/config.html")) {
      if (LittleFS.remove("/config.html")) deleted++;
    }
    server->send(200, "application/json", "{\"status\":\"ok\",\"deleted\":" + String(deleted) + "}");
  });

  // API - upload pliku do LittleFS
  server->on("/upload_form", HTTP_GET, []() {
    server->send(200, "text/html", "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Upload</title><style>body{font-family:Arial;background:#1a1a1a;color:#e0e0e0;padding:20px;}button{padding:12px 30px;background:#4a6a4a;border:none;color:#e0e0e0;cursor:pointer;border-radius:5px;font-size:16px;}</style></head><body><h2>Upload index.html</h2><form id='f'><input type='file' id='file' accept='.html'><br><br><button type='submit'>Wgraj</button></form><div id='s'></div><script>document.getElementById('f').addEventListener('submit',function(e){e.preventDefault();const file=document.getElementById('file').files[0];if(!file){alert('Wybierz plik');return;}const fd=new FormData();fd.append('index.html',file);fetch('/api/upload',{method:'POST',body:fd}).then(()=>{document.getElementById('s').innerHTML='<p style=\"color:green\">OK! Odswiez strone glowna (Ctrl+F5)</p>';}).catch(err=>{document.getElementById('s').innerHTML='<p style=\"color:red\">Blad: '+err+'</p>';});});</script></body></html>");
  });
  server->on("/api/upload", HTTP_POST, []() {
    server->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"upload_done\"}");
  }, []() {
    HTTPUpload& upload = server->upload();
    if (upload.status == UPLOAD_FILE_START) {
      String filename = upload.filename;
      if (!filename.startsWith("/")) {
        filename = "/" + filename;
      }
      if (!littleFsReady) {
        Serial.println("LittleFS not ready for upload");
        return;
      }
      Serial.printf("Uploading: %s\n", filename.c_str());
      if (LittleFS.exists(filename)) {
        LittleFS.remove(filename);
      }
      File f = LittleFS.open(filename, "w");
      if (!f) {
        Serial.println("Failed to open file for writing");
        return;
      }
      f.close();
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      String filename = upload.filename;
      if (!filename.startsWith("/")) {
        filename = "/" + filename;
      }
      File f = LittleFS.open(filename, "a");
      if (f) {
        f.write(upload.buf, upload.currentSize);
        f.close();
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      Serial.printf("Upload finished: %s, size: %d\n", upload.filename.c_str(), upload.totalSize);
    }
  });
  
  // API - pobierz konfigurację
  server->on("/api/config", HTTP_GET, []() {
    StaticJsonDocument<4096> doc;
    doc["wifi_ssid"] = wifiSSID;
    doc["wifi_pass"] = wifiPassword;
    doc["wifi_ssid2"] = wifiSSID2;
    doc["wifi_pass2"] = wifiPassword2;
    doc["cluster_host"] = clusterHost;
    doc["cluster_port"] = clusterPort;
    doc["pota_host"] = potaClusterHost;
    doc["pota_port"] = potaClusterPort;
    doc["pota_filter"] = potaFilterCommand;
    doc["pota_api_url"] = potaApiUrl;
    doc["hamalert_host"] = hamalertHost;
    doc["hamalert_port"] = hamalertPort;
    doc["hamalert_login"] = hamalertLogin;
    doc["hamalert_password"] = hamalertPassword;
    doc["user_callsign"] = userCallsign;
    doc["user_locator"] = userLocator;
    doc["timezone"] = timezoneHours;
    doc["user_lat"] = userLat;
    doc["user_lon"] = userLon;
    doc["user_lat_lon_valid"] = userLatLonValid;
    doc["qrz_user"] = qrzUsername;
    doc["qrz_pass"] = qrzPassword;
    doc["weather_key"] = weatherApiKey;
    doc["openwebrx_url"] = openWebRxUrl;
    doc["tft_backlight"] = backlightPercent;
    doc["tft_invert"] = sanitizeTftInvertSetting(tftInvertColors);
    doc["callsign_color"] = callsignColor;
    doc["tft_rotation"] = tftRotation;
    doc["tft_lang"] = tftLangToCode(tftLanguage);
    doc["table_size"] = dxTableSizeToCode(dxTableSizeMode);
    doc["tft_auto_switch"] = tftAutoSwitchEnabled;
    doc["tft_switch_time_sec"] = tftAutoSwitchTimeSec;
    doc["menu_hue"] = menuThemeHue;
    doc["qrz_status"] = qrzStatus;
    doc["cluster_noann"] = clusterNoAnnouncements;
    doc["cluster_nowwv"] = clusterNoWWV;
    doc["cluster_nowcy"] = clusterNoWCY;
    doc["cluster_usefilters"] = clusterUseFilters;
    doc["cluster_filters"] = clusterFilterCommands;
    doc["aprs_host"] = aprsIsHost;
    doc["aprs_port"] = aprsIsPort;
    doc["aprs_callsign"] = aprsCallsign;
    doc["aprs_passcode"] = aprsPasscode;
    doc["aprs_ssid"] = aprsSsid;
    doc["aprs_radius"] = aprsFilterRadius;
    doc["aprs_beacon"] = aprsBeaconEnabled;
    doc["aprs_symbol"] = aprsSymbolTwoChar;
    doc["aprs_comment"] = aprsUserComment;
    doc["aprs_alert"] = aprsAlertCsv;
    doc["aprs_alert_enabled"] = aprsAlertEnabled;
    doc["aprs_alert_nearby_enabled"] = aprsAlertNearbyEnabled;
    doc["aprs_alert_wx_enabled"] = aprsAlertWxEnabled;
    doc["aprs_alert_min_sec"] = aprsAlertMinSeconds;
    doc["aprs_alert_screen_sec"] = aprsAlertScreenSeconds;
    doc["aprs_alert_distance_km"] = aprsAlertDistanceKm;
    doc["enable_led_alert"] = enableLedAlert;
    doc["led_alert_duration_ms"] = ledAlertDurationMs;
    doc["led_alert_blink_ms"] = ledAlertBlinkMs;
    doc["aprs_interval_min"] = aprsIntervalMinutes;
    
    // Ustawienia wygaszacza ekranu (Matrix)
    doc["screen_saver_enabled"] = screenSaverEnabled;
    doc["screen_saver_timeout_min"] = screenSaverTimeoutMin;
    
    // Touch calibration mode - priorytet: pełna kombinacja > rotacje > swap > both > single invert
    String touchMode = "none";
    if (touchSwapXY && touchInvertX && touchInvertY) {
      touchMode = "xy_both";
    } else if (touchSwapXY && touchInvertX) {
      touchMode = "rot90cw";
    } else if (touchSwapXY && touchInvertY) {
      touchMode = "rot90ccw";
    } else if (touchSwapXY) {
      touchMode = "xy";
    } else if (touchInvertX && touchInvertY) {
      touchMode = "both";
    } else if (touchInvertX) {
      touchMode = "x";
    } else if (touchInvertY) {
      touchMode = "y";
    }
    doc["touch_swap_mode"] = touchMode;
    doc["touch_rotation"] = touchRotation;
    // Indywidualne parametry kalibracji dotyku
    doc["touch_xmin"] = touchXMin;
    doc["touch_xmax"] = touchXMax;
    doc["touch_ymin"] = touchYMin;
    doc["touch_ymax"] = touchYMax;
    doc["touch_swap"] = touchSwapXY;
    doc["touch_invx"] = touchInvertX;
    doc["touch_invy"] = touchInvertY;

    // Informacje systemowe
    doc["fw_version"] = FIRMWARE_VERSION;
    doc["uptime"] = formatUptime(millis() - bootTimeMs);
    doc["sys_ip"] = wifiConnected ? WiFi.localIP().toString() : (WiFi.getMode() & WIFI_AP ? WiFi.softAPIP().toString() : "disconnected");
    doc["sys_ssid"] = wifiConnected ? WiFi.SSID() : "";
    doc["sys_rssi"] = wifiConnected ? WiFi.RSSI() : 0;
    doc["sys_mac"] = WiFi.macAddress();
    doc["sys_heap"] = ESP.getFreeHeap();
    doc["sys_heap_max"] = ESP.getMaxAllocHeap();
    doc["sys_chip"] = ESP.getChipModel();
    doc["sys_cpu"] = ESP.getCpuFreqMHz();
    doc["sys_temp"] = temperatureRead();  // temperatura w °C
    #ifdef ENABLE_BATTERY_MONITORING
    doc["sys_vcc"] = readBatteryVoltage();  // napięcie baterii
    #else
    doc["sys_vcc"] = 0;
    #endif
    // LittleFS info
    if (littleFsReady) {
      doc["sys_littlefs_total"] = LittleFS.totalBytes();
      doc["sys_littlefs_free"] = LittleFS.totalBytes() - LittleFS.usedBytes();
    } else {
      doc["sys_littlefs_total"] = 0;
      doc["sys_littlefs_free"] = 0;
    }

    // Ustawienia PSKReporter
    doc["psk_receiver"] = pskReceiverCallsign;
    doc["psk_maxspots"] = pskMaxSpots;
    doc["psk_hours"] = pskHoursWindow;
    doc["psk_band"] = pskFilterBand;
    doc["psk_mode"] = pskFilterMode;
    doc["psk_url"] = pskCustomUrl;
    // HTTP Monitoring
    doc["psk_monitor_callsign"] = pskMonitorCallsign;
    doc["psk_report_days"] = pskReportDays;
    doc["psk_auto_refresh"] = pskAutoRefreshMinutes;
    // MQTT PSK Reporter
    doc["psk_mqtt_enabled"] = pskMqttEnabled;
    doc["psk_mqtt_server"] = pskMqttServer;
    doc["psk_mqtt_port"] = pskMqttPort;
    doc["psk_mqtt_callsign"] = pskMqttCallsign;

    JsonArray orderArr = doc.createNestedArray("screen_order");
    for (int i = 0; i < SCREEN_ORDER_COUNT; i++) {
      orderArr.add(screenTypeToCodeStr(screenOrder[i]));
    }
    
    
    String json;
    serializeJson(doc, json);
    server->send(200, "application/json", json);
  });
  
  // Upload nowej mapy BMP (np. przez curl -X POST -F "file=@Mapa swiata.bmp" http://esp-ip/upload/bmp)
  server->on("/upload/bmp", HTTP_POST, []() {
    server->send(200, "text/plain", "Upload complete");
  }, []() {
    HTTPUpload& upload = server->upload();
    if (upload.status == UPLOAD_FILE_START) {
      String filename = upload.filename;
      if (!filename.endsWith(".bmp") && !filename.endsWith(".BMP")) {
        Serial.println("[UPLOAD] Invalid file type, must be .bmp");
        return;
      }
      if (littleFsReady) {
        LittleFS.remove(PSK_MAP_BMP_PATH); // Usuń starą mapę
        File f = LittleFS.open(PSK_MAP_BMP_PATH, "w");
        if (!f) {
          Serial.println("[UPLOAD] Failed to open file for writing");
          return;
        }
        f.close();
        Serial.printf("[UPLOAD] Starting upload: %s\n", filename.c_str());
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (littleFsReady) {
        File f = LittleFS.open(PSK_MAP_BMP_PATH, "a");
        if (f) {
          f.write(upload.buf, upload.currentSize);
          f.close();
        }
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      Serial.printf("[UPLOAD] Done: %d bytes\n", upload.totalSize);
      // Odśwież mapę jeśli jesteśmy na ekranie PSK
      if (currentScreen == SCREEN_PSK_MAP) {
        drawPskMap();
      }
    }
  });
  
  // Prosty formularz do uploadu mapy
  server->on("/upload", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'></head><body>";
    html += "<h1>Wgraj nową mapę BMP</h1>";
    html += "<form method='POST' action='/upload/bmp' enctype='multipart/form-data'>";
    html += "<input type='file' name='file' accept='.bmp'><br><br>";
    html += "<input type='submit' value='Wgraj mapę'>";
    html += "</form>";
    html += "<p>Rozmiar: 480x320 px</p>";
    html += "<p><a href='/'>← Powrót</a></p>";
    html += "</body></html>";
    server->send(200, "text/html; charset=utf-8", html);
  });
  
  server->begin();
  Serial.println("Serwer WWW uruchomiony");
}

// ========== HTML PAGES ==========

String getMainHTML() {
  return R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 HAM CLOCK</title>
  <style>
    :root {
      --bg: #0f172a;
      --panel: #111827;
      --panel-2: #1f2937;
      --text: #e5e7eb;
      --muted: #94a3b8;
      --accent: #f59e0b;
      --good: #22c55e;
      --border: #334155;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: Arial, sans-serif;
      background: linear-gradient(180deg, #020617 0%, var(--bg) 100%);
      color: var(--text);
    }
    .wrap {
      max-width: 1080px;
      margin: 0 auto;
      padding: 20px;
    }
    .hero, .panel {
      background: rgba(17,24,39,0.92);
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 18px;
      margin-bottom: 16px;
      box-shadow: 0 12px 28px rgba(0,0,0,0.24);
    }
    h1, h2 {
      margin: 0 0 12px 0;
    }
    p {
      color: var(--muted);
      margin: 0;
    }
    .row {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
      gap: 12px;
      margin-top: 16px;
    }
    .card {
      background: var(--panel-2);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 14px;
    }
    .label {
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--muted);
      margin-bottom: 6px;
    }
    .value {
      font-size: 20px;
      font-weight: 700;
      word-break: break-word;
    }
    .toolbar {
      display: flex;
      gap: 10px;
      flex-wrap: wrap;
      margin-top: 16px;
    }
    button, a.btn {
      background: var(--accent);
      color: #111827;
      border: none;
      border-radius: 999px;
      padding: 10px 14px;
      font-weight: 700;
      cursor: pointer;
      text-decoration: none;
      display: inline-block;
    }
    button.secondary {
      background: transparent;
      color: var(--text);
      border: 1px solid var(--border);
    }
    input, select {
      width: 100%;
      background: #020617;
      color: var(--text);
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 10px 12px;
      font-size: 14px;
    }
    .field {
      display: flex;
      flex-direction: column;
      gap: 6px;
    }
    .field label {
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }
    .check {
      display: flex;
      align-items: center;
      gap: 10px;
      padding: 10px 12px;
      background: var(--panel-2);
      border: 1px solid var(--border);
      border-radius: 10px;
    }
    .check input {
      width: auto;
      transform: scale(1.2);
    }
    .sectionTitle {
      margin: 18px 0 10px 0;
      font-size: 15px;
      font-weight: 700;
    }
    details {
      margin-top: 16px;
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 12px;
      background: rgba(2,6,23,0.55);
    }
    summary {
      cursor: pointer;
      font-weight: 700;
    }
    textarea {
      width: 100%;
      min-height: 320px;
      background: #020617;
      color: var(--text);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 12px;
      font-family: Consolas, monospace;
      font-size: 13px;
      resize: vertical;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 12px;
    }
    th, td {
      padding: 10px 8px;
      border-bottom: 1px solid var(--border);
      text-align: left;
      font-size: 14px;
    }
    th {
      color: var(--muted);
      font-weight: 600;
    }
    .ok {
      color: var(--good);
      font-weight: 700;
    }
    .hint {
      margin-top: 10px;
      font-size: 13px;
      color: var(--muted);
    }
    @media (max-width: 640px) {
      .wrap { padding: 12px; }
      .value { font-size: 18px; }
      th, td { font-size: 13px; }
    }
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <h1>ESP32 HAM CLOCK</h1>
      <p>Panel konfiguracyjny i monitorowanie urządzenia.</p>
      <div class="toolbar">
        <button onclick="loadAll()">Odśwież</button>
        <a class="btn" href="/api/config">API config</a>
        <a class="btn" href="/api/spots">API spots</a>
        <a class="btn" href="/api/psk">PSKReporter</a>
        <a class="btn" href="/instruction" style="background:#28a745;">📖 Instrukcja</a>
      </div>
    </section>

    <section class="panel">
      <h2>Status</h2>
      <div class="row">
        <div class="card">
          <div class="label">WiFi SSID</div>
          <div id="ssid" class="value">-</div>
        </div>
        <div class="card">
          <div class="label">Callsign</div>
          <div id="callsign" class="value">-</div>
        </div>
        <div class="card">
          <div class="label">APRS Host</div>
          <div id="aprs" class="value">-</div>
        </div>
        <div class="card">
          <div class="label">Cluster Filter</div>
          <div id="filter" class="value">-</div>
        </div>
      </div>
      <div class="hint">Panel konfiguracyjny wbudowany w firmware urządzenia.</div>
    </section>

    <section class="panel">
      <h2>System</h2>
      <div class="row">
        <div class="card">
          <div class="label">Autor</div>
          <div class="value">SP3KON (oryginał)</div>
        </div>
        <div class="card">
          <div class="label">Modyfikacje</div>
          <div class="value">SP9TNV</div>
        </div>
        <div class="card">
          <div class="label">Wersja</div>
          <div class="value">1.3</div>
        </div>
        <div class="card">
          <div class="label">Licencja</div>
          <div class="value">MIT License</div>
        </div>
      </div>
      <div class="hint">
        Projekt objęty licencją MIT. Możesz używać, modyfikować i rozpowszechniać kod pod warunkiem zachowania informacji o autorze oryginalnym (SP3KON).
      </div>
    </section>

    <section class="panel">
      <h2>Ostatnie spoty</h2>
      <div id="spotsState" class="hint">Ladowanie...</div>
      <table>
        <thead>
          <tr>
            <th>Czas</th>
            <th>Call</th>
            <th>Freq</th>
            <th>Mode</th>
            <th>Kraj</th>
          </tr>
        </thead>
        <tbody id="spotsBody"></tbody>
      </table>
    </section>

    <section class="panel">
      <h2>Konfiguracja</h2>
      <p>Graficzny formularz dla najwazniejszych ustawien. JSON zostal nizej jako tryb zaawansowany.</p>
      <div class="toolbar">
        <button onclick="saveGraphicConfig()">Zapisz konfiguracje</button>
        <button class="secondary" onclick="syncJsonFromForm()">Przepisz do JSON</button>
      </div>
      <div id="saveState" class="hint">Najpierw kliknij Odswiez albo poczekaj na zaladowanie konfiguracji.</div>

      <div class="sectionTitle">WiFi i operator</div>
      <div class="row">
        <div class="field">
          <label for="wifi_ssid">WiFi SSID</label>
          <input id="wifi_ssid" type="text">
        </div>
        <div class="field">
          <label for="wifi_pass">WiFi haslo</label>
          <input id="wifi_pass" type="password">
        </div>
        <div class="field">
          <label for="user_callsign">Twoj znak</label>
          <input id="user_callsign" type="text">
        </div>
        <div class="field">
          <label for="user_locator">Locator</label>
          <input id="user_locator" type="text">
        </div>
        <div class="field">
          <label for="timezone">Strefa czasowa</label>
          <input id="timezone" type="number" min="-12" max="14" step="1">
        </div>
        <div class="field">
          <label for="openwebrx_url">OpenWebRX URL</label>
          <input id="openwebrx_url" type="text">
        </div>
      </div>

      <div class="sectionTitle">DX / POTA / HamAlert</div>
      <div class="row">
        <div class="field">
          <label for="cluster_host">DX cluster host</label>
          <input id="cluster_host" type="text">
        </div>
        <div class="field">
          <label for="cluster_port">DX cluster port</label>
          <input id="cluster_port" type="number">
        </div>
        <div class="field">
          <label for="pota_api_url">POTA API URL</label>
          <input id="pota_api_url" type="text">
        </div>
        <div class="field">
          <label for="hamalert_host">HamAlert host</label>
          <input id="hamalert_host" type="text">
        </div>
        <div class="field">
          <label for="hamalert_login">HamAlert login</label>
          <input id="hamalert_login" type="text">
        </div>
        <div class="field">
          <label for="hamalert_password">HamAlert haslo</label>
          <input id="hamalert_password" type="password">
        </div>
      </div>

      <div class="sectionTitle">APRS</div>
      <div class="row">
        <div class="field">
          <label for="aprs_host">APRS host</label>
          <input id="aprs_host" type="text">
        </div>
        <div class="field">
          <label for="aprs_port">APRS port</label>
          <input id="aprs_port" type="number">
        </div>
        <div class="field">
          <label for="aprs_callsign">APRS callsign</label>
          <input id="aprs_callsign" type="text">
        </div>
        <div class="field">
          <label for="aprs_passcode">APRS passcode</label>
          <input id="aprs_passcode" type="number">
        </div>
        <div class="field">
          <label for="aprs_ssid">APRS SSID</label>
          <input id="aprs_ssid" type="number">
        </div>
        <div class="field">
          <label for="aprs_radius">APRS radius km</label>
          <input id="aprs_radius" type="number">
        </div>
        <div class="field">
          <label for="aprs_symbol">APRS symbol</label>
          <input id="aprs_symbol" type="text">
        </div>
        <div class="field">
          <label for="aprs_comment">APRS komentarz</label>
          <input id="aprs_comment" type="text">
        </div>
      </div>
      <div class="row">
        <label class="check"><input id="aprs_beacon" type="checkbox"> Beacon APRS wlaczony</label>
        <label class="check"><input id="aprs_alert_enabled" type="checkbox"> Alerty APRS wlaczone</label>
        <label class="check"><input id="aprs_alert_nearby_enabled" type="checkbox"> Alerty nearby</label>
        <label class="check"><input id="aprs_alert_wx_enabled" type="checkbox"> Alerty WX</label>
      </div>

      <div class="sectionTitle">PSKReporter</div>
      <div class="row">
        <div class="field">
          <label for="psk_autorefresh">Auto odswiezanie (min)</label>
          <input id="psk_autorefresh" type="number" min="0" max="60">
        </div>
        <div class="field">
          <label for="psk_callsign">Callsign (nadajnik)</label>
          <input id="psk_callsign" type="text" placeholder="np. SP9ABC">
        </div>
        <div class="field">
          <label for="psk_maxage">Max wiek spotow (min)</label>
          <input id="psk_maxage" type="number" min="5" max="120" value="60">
        </div>
      </div>
      <div class="row">
        <label class="check"><input id="psk_mqtt_enabled" type="checkbox"> Uzyj MQTT zamiast HTTP</label>
      </div>
      <div class="row" id="mqtt_fields" style="display:none;">
        <div class="field">
          <label for="psk_mqtt_server">MQTT Serwer</label>
          <input id="psk_mqtt_server" type="text" placeholder="mqtt.pskreporter.info">
        </div>
        <div class="field">
          <label for="psk_mqtt_port">MQTT Port</label>
          <input id="psk_mqtt_port" type="number" min="1" max="65535" value="1883">
        </div>
        <div class="field">
          <label for="psk_mqtt_callsign">MQTT Callsign (filter)</label>
          <input id="psk_mqtt_callsign" type="text" placeholder="np. SP9ABC">
        </div>
      </div>

      <div class="sectionTitle">Ekran i filtry</div>
      <div class="row">
        <div class="field">
          <label for="tft_backlight">Podswietlenie TFT</label>
          <input id="tft_backlight" type="number" min="10" max="100">
        </div>
        <div class="field">
          <label for="tft_rotation">Rotacja TFT</label>
          <select id="tft_rotation">
            <option value="0">0</option>
            <option value="1">1</option>
            <option value="2">2</option>
            <option value="3">3</option>
          </select>
        </div>
        <div class="field">
          <label for="tft_lang">Jezyk TFT</label>
          <select id="tft_lang">
            <option value="pl">pl</option>
            <option value="en">en</option>
          </select>
        </div>
        <div class="field">
          <label for="cluster_filters">Filtry cluster</label>
          <input id="cluster_filters" type="text">
        </div>
      </div>
      <div class="row">
        <label class="check"><input id="tft_invert" type="checkbox"> Odwrocone kolory TFT</label>
        <label class="check"><input id="cluster_usefilters" type="checkbox"> Uzyj filtrow cluster</label>
        <label class="check"><input id="cluster_noann" type="checkbox"> Ukryj ANN</label>
        <label class="check"><input id="cluster_nowwv" type="checkbox"> Ukryj WWV</label>
        <label class="check"><input id="cluster_nowcy" type="checkbox"> Ukryj WCY</label>
      </div>

      <div class="sectionTitle">Kolory zegara</div>
      <div class="row">
        <div class="field">
          <label for="callsign_color">Kolor callsignu</label>
          <select id="callsign_color">
            <option value="65535">Bialy (0xFFFF)</option>
            <option value="63488">Czerwony (0xF800)</option>
            <option value="2016">Zielony (0x07E0)</option>
            <option value="31">Niebieski (0x001F)</option>
            <option value="65504">Zolty (0xFFE0)</option>
            <option value="31">Ciemnoniebieski (0x001F)</option>
            <option value="2047">Jasnoniebieski (0x07FF)</option>
            <option value="63519">Magenta (0xF81F)</option>
            <option value="2016">Ciemnozielony (0x07E0)</option>
            <option value="65535">Jasnoszary (0xFFFF)</option>
            <option value="33840">Ciemnoszary (0x8410)</option>
            <option value="63488">Pomaranczowy (0xF800)</option>
            <option value="65504">Cyjan (0x07FF)</option>
            <option value="63519">Rozowy (0xF81F)</option>
            <option value="0">Czarny (0x0000)</option>
          </select>
        </div>
        <div class="field">
          <label for="menu_hue">Odcien motywu menu (0-255)</label>
          <input id="menu_hue" type="number" min="0" max="255">
        </div>
      </div>
      <div class="hint">Wybierz kolor callsignu z listy 15 predefiniowanych kolorow. Odcien motywu: 20=pomaranczowy (domyslny).</div>

      <div class="sectionTitle">Kolejność ekranów TFT (13 pozycji)</div>
      <div class="hint">Wybierz ekran dla każdej pozycji. 'Wyłączony' oznacza pustą pozycję.</div>
      <div class="row">
        <div class="field"><label for="screen_0">Pozycja 1</label><select id="screen_0"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
        <div class="field"><label for="screen_1">Pozycja 2</label><select id="screen_1"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
        <div class="field"><label for="screen_2">Pozycja 3</label><select id="screen_2"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
        <div class="field"><label for="screen_3">Pozycja 4</label><select id="screen_3"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
        <div class="field"><label for="screen_4">Pozycja 5</label><select id="screen_4"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
        <div class="field"><label for="screen_5">Pozycja 6</label><select id="screen_5"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
        <div class="field"><label for="screen_6">Pozycja 7</label><select id="screen_6"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
        <div class="field"><label for="screen_7">Pozycja 8</label><select id="screen_7"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
        <div class="field"><label for="screen_8">Pozycja 9</label><select id="screen_8"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
        <div class="field"><label for="screen_9">Pozycja 10</label><select id="screen_9"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
        <div class="field"><label for="screen_10">Pozycja 11</label><select id="screen_10"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
        <div class="field"><label for="screen_11">Pozycja 12</label><select id="screen_11"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
        <div class="field"><label for="screen_12">Pozycja 13</label><select id="screen_12"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
        <div class="field"><label for="screen_13">Pozycja 14</label><select id="screen_13"><option value="propagacja">Propagacja</option><option value="solarindex">Solar Index</option><option value="hamclock">Ham Clock</option><option value="dxcluster">DX Cluster</option><option value="aprsis">APRS-IS</option><option value="aprsradar">APRS Radar</option><option value="weather">Pogoda</option><option value="weatherforecast">Prognoza</option><option value="pota">POTA</option><option value="hamalert">HamAlert</option><option value="pskmap">PSK Map</option><option value="unlishunter">Unlis Hunter</option><option value="matrix">Matrix</option><option value="isspasstracking">ISS Pass Tracking</option><option value="off">Wyłączony</option></select></div>
      </div>

      <div class="toolbar">
        <button onclick="saveGraphicConfig()" style="background:#28a745;">Zapisz konfigurację</button>
        <button onclick="clearHtmlCache()" style="background:#dc3545;">Wyczyść cache HTML</button>
      </div>

      <details>
        <summary>Zaawansowany JSON</summary>
        <div class="hint">Mozesz nadal edytowac pelny JSON recznie, jesli chcesz zmienic pola nieobecne w formularzu.</div>
        <div class="toolbar">
          <button onclick="saveConfig()">Zapisz JSON</button>
        </div>
        <textarea id="configEditor" spellcheck="false"></textarea>
      </details>
    </section>
  </div>

  <script>
    let currentConfig = {};

    // Mapowanie kodów ekranów na przyjazne nazwy
    const screenLabels = {
      'propagacja': 'Propagacja',
      'solarindex': 'Solar Index',
      'hamclock': 'Ham Clock',
      'dxcluster': 'DX Cluster',
      'aprsis': 'APRS-IS',
      'aprsradar': 'APRS Radar',
      'weather': 'Pogoda',
      'weatherforecast': 'Prognoza',
      'pota': 'POTA',
      'hamalert': 'HamAlert',
      'pskmap': 'PSK Map',
      'matrix': 'Matrix',
      'unlishunter': 'Unlis Hunter',
      'off': 'Wyłączony'
    };

    function getScreenLabel(code) {
      return screenLabels[code] || code;
    }

    function byId(id) {
      return document.getElementById(id);
    }

    function setValue(id, value) {
      byId(id).value = value ?? '';
    }

    function setChecked(id, value) {
      byId(id).checked = !!value;
    }

    function getInt(id, fallback) {
      const value = parseInt(byId(id).value, 10);
      return Number.isNaN(value) ? fallback : value;
    }

    function getString(id, fallback = '') {
      const value = byId(id).value;
      return value == null ? fallback : value;
    }

    function fillGraphicForm(cfg) {
      setValue('wifi_ssid', cfg.wifi_ssid);
      setValue('wifi_pass', cfg.wifi_pass);
      setValue('user_callsign', cfg.user_callsign);
      setValue('user_locator', cfg.user_locator);
      setValue('timezone', cfg.timezone);
      setValue('openwebrx_url', cfg.openwebrx_url);
      setValue('cluster_host', cfg.cluster_host);
      setValue('cluster_port', cfg.cluster_port);
      setValue('pota_api_url', cfg.pota_api_url);
      setValue('hamalert_host', cfg.hamalert_host);
      setValue('hamalert_login', cfg.hamalert_login);
      setValue('hamalert_password', cfg.hamalert_password);
      setValue('aprs_host', cfg.aprs_host);
      setValue('aprs_port', cfg.aprs_port);
      setValue('aprs_callsign', cfg.aprs_callsign);
      setValue('aprs_passcode', cfg.aprs_passcode);
      setValue('aprs_ssid', cfg.aprs_ssid);
      setValue('aprs_radius', cfg.aprs_radius);
      setValue('aprs_symbol', cfg.aprs_symbol);
      setValue('aprs_comment', cfg.aprs_comment);
      setValue('psk_autorefresh', cfg.psk_autorefresh);
      setValue('psk_callsign', cfg.psk_callsign);
      setValue('psk_maxage', cfg.psk_maxage);
      // MQTT PSK Reporter
      setChecked('psk_mqtt_enabled', cfg.psk_mqtt_enabled);
      setValue('psk_mqtt_server', cfg.psk_mqtt_server);
      setValue('psk_mqtt_port', cfg.psk_mqtt_port);
      setValue('psk_mqtt_callsign', cfg.psk_mqtt_callsign);
      // Pokaz/ukryj pola MQTT
      byId('mqtt_fields').style.display = cfg.psk_mqtt_enabled ? 'flex' : 'none';
      setValue('tft_backlight', cfg.tft_backlight);
      setValue('tft_rotation', cfg.tft_rotation);
      setValue('tft_lang', cfg.tft_lang);
      setValue('cluster_filters', cfg.cluster_filters);
      setChecked('aprs_beacon', cfg.aprs_beacon);
      setChecked('aprs_alert_enabled', cfg.aprs_alert_enabled);
      setChecked('aprs_alert_nearby_enabled', cfg.aprs_alert_nearby_enabled);
      setChecked('aprs_alert_wx_enabled', cfg.aprs_alert_wx_enabled);
      setChecked('tft_invert', cfg.tft_invert);
      setValue('callsign_color', cfg.callsign_color ?? 65535);
      setValue('menu_hue', cfg.menu_hue ?? 20);
      setChecked('cluster_usefilters', cfg.cluster_usefilters);
      setChecked('cluster_noann', cfg.cluster_noann);
      setChecked('cluster_nowwv', cfg.cluster_nowwv);
      setChecked('cluster_nowcy', cfg.cluster_nowcy);
      // Ładowanie kolejności ekranów
      if (Array.isArray(cfg.screen_order)) {
        cfg.screen_order.forEach((code, index) => {
          const select = byId('screen_' + index);
          if (select) select.value = code || 'off';
        });
      }
    }

    function buildConfigFromForm() {
      return {
        ...currentConfig,
        wifi_ssid: getString('wifi_ssid'),
        wifi_pass: getString('wifi_pass'),
        user_callsign: getString('user_callsign'),
        user_locator: getString('user_locator'),
        timezone: getInt('timezone', currentConfig.timezone ?? 0),
        openwebrx_url: getString('openwebrx_url'),
        cluster_host: getString('cluster_host'),
        cluster_port: getInt('cluster_port', currentConfig.cluster_port ?? 7300),
        pota_api_url: getString('pota_api_url'),
        hamalert_host: getString('hamalert_host'),
        hamalert_login: getString('hamalert_login'),
        hamalert_password: getString('hamalert_password'),
        aprs_host: getString('aprs_host'),
        aprs_port: getInt('aprs_port', currentConfig.aprs_port ?? 14580),
        aprs_callsign: getString('aprs_callsign'),
        aprs_passcode: getInt('aprs_passcode', currentConfig.aprs_passcode ?? 0),
        aprs_ssid: getInt('aprs_ssid', currentConfig.aprs_ssid ?? 0),
        aprs_radius: getInt('aprs_radius', currentConfig.aprs_radius ?? 50),
        aprs_symbol: getString('aprs_symbol', '/-'),
        aprs_comment: getString('aprs_comment'),
        psk_autorefresh: getInt('psk_autorefresh', currentConfig.psk_autorefresh ?? 10),
        psk_callsign: getString('psk_callsign'),
        psk_maxage: getInt('psk_maxage', currentConfig.psk_maxage ?? 60),
        // MQTT PSK Reporter
        psk_mqtt_enabled: byId('psk_mqtt_enabled').checked,
        psk_mqtt_server: getString('psk_mqtt_server'),
        psk_mqtt_port: getInt('psk_mqtt_port', currentConfig.psk_mqtt_port ?? 1883),
        psk_mqtt_callsign: getString('psk_mqtt_callsign'),
        aprs_beacon: byId('aprs_beacon').checked,
        aprs_alert_enabled: byId('aprs_alert_enabled').checked,
        aprs_alert_nearby_enabled: byId('aprs_alert_nearby_enabled').checked,
        aprs_alert_wx_enabled: byId('aprs_alert_wx_enabled').checked,
        tft_backlight: getInt('tft_backlight', currentConfig.tft_backlight ?? 100),
        tft_rotation: getInt('tft_rotation', currentConfig.tft_rotation ?? 1),
        tft_lang: getString('tft_lang', 'pl'),
        tft_invert: byId('tft_invert').checked,
        callsign_color: getInt('callsign_color', currentConfig.callsign_color ?? 65535),
        menu_hue: getInt('menu_hue', currentConfig.menu_hue ?? 20),
        cluster_filters: getString('cluster_filters'),
        cluster_usefilters: byId('cluster_usefilters').checked,
        cluster_noann: byId('cluster_noann').checked,
        cluster_nowwv: byId('cluster_nowwv').checked,
        cluster_nowcy: byId('cluster_nowcy').checked,
        screen_order: Array.from({length: 14}, (_, i) => {
          const select = byId('screen_' + i);
          return select ? select.value : 'off';
        })
      };
    }

    async function loadConfig() {
      const res = await fetch('/api/config');
      const cfg = await res.json();
      currentConfig = cfg;
      document.getElementById('ssid').textContent = cfg.wifi_ssid || '-';
      document.getElementById('callsign').textContent = cfg.aprs_callsign || '-';
      document.getElementById('aprs').textContent = (cfg.aprs_host || '-') + ':' + (cfg.aprs_port || '-');
      document.getElementById('filter').textContent = cfg.cluster_filters || '(brak)';
      fillGraphicForm(cfg);
      renderScreenOrder(cfg.screen_order || []);
      document.getElementById('configEditor').value = JSON.stringify(cfg, null, 2);
      document.getElementById('saveState').textContent = 'Konfiguracja zaladowana. Mozesz ja edytowac i zapisac.';
    }

    function renderScreenOrder(screenOrder) {
      const container = document.getElementById('screenOrder');
      if (!container) return;
      container.innerHTML = '';
      if (!Array.isArray(screenOrder) || screenOrder.length === 0) {
        container.innerHTML = '<div class="hint">Brak skonfigurowanych ekranów</div>';
        return;
      }
      screenOrder.forEach((code, index) => {
        if (code === 'off') return;
        const label = getScreenLabel(code);
        const div = document.createElement('div');
        div.className = 'card';
        div.innerHTML = '<div class="label">Ekran ' + (index + 1) + '</div><div class="value">' + label + '</div>';
        container.appendChild(div);
      });
    }

    function formatTimeToLocal(timeStr) {
      if (!timeStr || timeStr === '-') return '-';
      // Parsuj czas HH:MM lub HH:MM:SS
      const match = timeStr.match(/(\d{1,2}):(\d{2})(?::\d{2})?/);
      if (!match) return timeStr;
      
      let hour = parseInt(match[1], 10);
      const minute = match[2];
      
      // Pobierz offset strefy czasowej z konfiguracji
      const tzOffset = currentConfig.timezone || 1;
      
      // Dodaj offset (prost konwersja, bez DST)
      let localHour = hour + tzOffset;
      
      // Obsługa przejścia przez północ
      if (localHour < 0) localHour += 24;
      else if (localHour >= 24) localHour -= 24;
      
      return (localHour < 10 ? '0' : '') + localHour + ':' + minute;
    }

    async function loadSpots() {
      const res = await fetch('/api/spots');
      const data = await res.json();
      const spots = Array.isArray(data) ? data : (Array.isArray(data.spots) ? data.spots : []);
      const body = document.getElementById('spotsBody');
      const state = document.getElementById('spotsState');
      body.innerHTML = '';

      if (!Array.isArray(spots) || spots.length === 0) {
        state.textContent = 'Brak spotow.';
        return;
      }

      state.innerHTML = '<span class="ok">OK</span> Zaladowano ' + spots.length + ' rekordow.';
      spots.slice(0, 20).forEach((spot) => {
        const tr = document.createElement('tr');
        tr.innerHTML =
          '<td>' + formatTimeToLocal(spot.time || '-') + '</td>' +
          '<td>' + (spot.callsign || '-') + '</td>' +
          '<td>' + (spot.frequency || '-') + '</td>' +
          '<td>' + (spot.mode || '-') + '</td>' +
          '<td>' + (spot.country || '-') + '</td>';
        body.appendChild(tr);
      });
    }

    async function loadAll() {
      try {
        await loadConfig();
        await loadSpots();
      } catch (err) {
        document.getElementById('spotsState').textContent = 'Blad odczytu API: ' + err;
      }
    }

    function syncJsonFromForm() {
      const payload = buildConfigFromForm();
      byId('configEditor').value = JSON.stringify(payload, null, 2);
      byId('saveState').textContent = 'JSON zaktualizowany na podstawie formularza.';
    }

    async function saveConfig() {
      const editor = document.getElementById('configEditor');
      const saveState = document.getElementById('saveState');
      let parsed;

      try {
        parsed = JSON.parse(editor.value);
      } catch (err) {
        saveState.textContent = 'Blad JSON: ' + err;
        return;
      }

      saveState.textContent = 'Zapisywanie...';

      try {
        const res = await fetch('/api/save', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(parsed)
        });
        const text = await res.text();
        saveState.textContent = 'Zapisano. Odpowiedz: ' + text;
        await loadConfig();
      } catch (err) {
        saveState.textContent = 'Blad zapisu: ' + err;
      }
    }

    async function saveGraphicConfig() {
      byId('configEditor').value = JSON.stringify(buildConfigFromForm(), null, 2);
      await saveConfig();
    }

    async function clearHtmlCache() {
      if (!confirm('Czy na pewno chcesz wyczyścić cache HTML? To usunie stare pliki HTML z LittleFS i wymusi ładowanie z firmware.')) {
        return;
      }
      try {
        const res = await fetch('/api/clear_html_cache', {method: 'POST'});
        const data = await res.json();
        if (data.status === 'ok') {
          alert('Cache HTML wyczyszczony! Usunięto ' + data.deleted + ' plików. Odśwież stronę (Ctrl+F5).');
        } else {
          alert('Błąd: ' + (data.message || 'nieznany'));
        }
      } catch (err) {
        alert('Błąd połączenia: ' + err);
      }
    }

    // Event listener dla MQTT checkbox - pokaz/ukryj pola MQTT
    byId('psk_mqtt_enabled').addEventListener('change', function() {
      byId('mqtt_fields').style.display = this.checked ? 'flex' : 'none';
    });

    loadAll();
  </script>
</body>
</html>
)rawliteral";
}
String getConfigHTML() {
  return getMainHTML(); // UÄąÄ˝ywa tego samego HTML z JavaScript
}

#ifdef ENABLE_TFT_DISPLAY
TaskHandle_t uiTaskHandle = nullptr;

void uiTaskLoop(void *parameter) {
  (void)parameter;
  for (;;) {
    unsigned long now = millis();

    if (tftInitialized) {
      bool pendingScreen1 = false;
      bool pendingScreen2 = false;
      bool pendingScreen6 = false;
      bool pendingScreen7 = false;
      bool pendingAnyScreen = false;
      uint8_t pendingAnyScreenId = SCREEN_HAM_CLOCK;
      portENTER_CRITICAL(&uiPendingRedrawMux);
      pendingScreen1 = uiPendingScreen1Redraw;
      pendingScreen2 = uiPendingScreen2Redraw;
      pendingScreen6 = uiPendingScreen6Redraw;
      pendingScreen7 = uiPendingScreen7Redraw;
      pendingAnyScreen = uiPendingAnyScreenRedraw;
      pendingAnyScreenId = uiPendingAnyScreenId;
      uiPendingScreen1Redraw = false;
      uiPendingScreen2Redraw = false;
      uiPendingScreen6Redraw = false;
      uiPendingScreen7Redraw = false;
      uiPendingAnyScreenRedraw = false;
      portEXIT_CRITICAL(&uiPendingRedrawMux);

      if (pendingAnyScreen && currentScreen == (ScreenType)pendingAnyScreenId && !inMenu && !aprsAlertScreenActive && !qrzPopupActive) {
        drawScreen((ScreenType)pendingAnyScreenId);
      }

      // Dodatkowe sprawdzenia dla konkretnych ekranów tylko gdy pendingAnyScreen nie było true
      if (!pendingAnyScreen) {
        if (pendingScreen1 && currentScreen == SCREEN_HAM_CLOCK && !inMenu && !aprsAlertScreenActive) {
          drawScreen(SCREEN_HAM_CLOCK);
        }
        if (pendingScreen2 && currentScreen == SCREEN_DX_CLUSTER && !inMenu && !aprsAlertScreenActive && !qrzPopupActive) {
          drawScreen(SCREEN_DX_CLUSTER);
        }
        if (pendingScreen7 && currentScreen == SCREEN_POTA_CLUSTER && !inMenu && !aprsAlertScreenActive && !qrzPopupActive) {
          drawScreen(SCREEN_POTA_CLUSTER);
        }
        if (pendingScreen6 && (currentScreen == SCREEN_APRS_IS || currentScreen == SCREEN_APRS_RADAR) && !inMenu && !aprsAlertScreenActive && !qrzPopupActive) {
          drawScreen(currentScreen);
        }
      }

      // Periodic ISS dynamic update every 5 seconds
      static unsigned long lastIssUiUpdate = 0;
      if (currentScreen == SCREEN_ISS_PASS_TRACKING && !inMenu && !aprsAlertScreenActive && !qrzPopupActive) {
        if (millis() - lastIssUiUpdate >= 5000) {
          lastIssUiUpdate = millis();
          updateIssDynamicDisplay();
        }
      }

      if (aprsAlertDrawPending) {
        APRSStation pendingStation;
        portENTER_CRITICAL(&aprsAlertPendingMux);
        pendingStation = aprsAlertPendingStation;
        aprsAlertDrawPending = false;
        portEXIT_CRITICAL(&aprsAlertPendingMux);
        ALERT_Screen(pendingStation);
      }
      updateAlertScreenTimeout();
      handleTouchNavigation();
      
      // Rysuj wygaszacz gdy jest aktywny
      if (screenSaverActive && !screenSaverMenuActive) {
        drawScreenSaver();
      }
    }

    // Sprawdzanie wygaszacza ekranu - co sekundę
    static unsigned long lastScreenSaverCheckMs = 0;
    if (now - lastScreenSaverCheckMs >= 1000) {
      checkScreenSaverTimeout();
      lastScreenSaverCheckMs = now;
    }

    if (tftAutoSwitchEnabled && tftInitialized && !inMenu && !aprsAlertScreenActive && !touchCalActive && !brightnessMenuActive && !qrzPopupActive) {
      unsigned long nowSwitch = millis();
      if (tftAutoSwitchLastMs == 0 || tftAutoSwitchLastScreen != currentScreen) {
        tftAutoSwitchLastMs = nowSwitch;
        tftAutoSwitchLastScreen = currentScreen;
      } else {
        unsigned long intervalMs = (unsigned long)tftAutoSwitchTimeSec * 1000UL;
        if (intervalMs > 0 && (nowSwitch - tftAutoSwitchLastMs) >= intervalMs) {
          ScreenType nextScreen = getNextScreenId(currentScreen);
          if (nextScreen != SCREEN_OFF && nextScreen != currentScreen) {
            currentScreen = nextScreen;
            drawScreen(currentScreen);
          }
          tftAutoSwitchLastMs = nowSwitch;
          tftAutoSwitchLastScreen = currentScreen;
        }
      }
    }

    if (!restartRequested) {
      if (tftInitialized && currentScreen == SCREEN_HAM_CLOCK && !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreen1UpdateMs > 1000) {
          updateScreen1Clock();
          updateScreen1HeaderClock();
          lastScreen1UpdateMs = now;
        }
        updateScreen1Header();
        updateScreen1Date();
      }

      if (tftInitialized && currentScreen == SCREEN_DX_CLUSTER && !inMenu && !aprsAlertScreenActive && !qrzPopupActive) {
        if (now - lastScreenUpdate > 100) {
          updateScreen2Data();
          lastScreenUpdate = now;
        }
        // Aktualizacja zegara co sekundę - WYŁĄCZONA
        // static unsigned long lastDxClockUpdate = 0;
        // if (now - lastDxClockUpdate >= 1000) {
        //   updateDxClusterClock();
        //   lastDxClockUpdate = now;
        // }
      }

      if (tftInitialized && currentScreen == SCREEN_POTA_CLUSTER && !inMenu && !aprsAlertScreenActive && !qrzPopupActive) {
        if (now - lastScreen7UpdateMs > 200) {
          updateScreen7Data();
          lastScreen7UpdateMs = now;
        }
        // Aktualizacja zegara co sekundę
        static unsigned long lastPotaClockUpdate = 0;
        if (now - lastPotaClockUpdate >= 1000) {
          updatePotaClusterClock();
          lastPotaClockUpdate = now;
        }
      }

      if (tftInitialized && currentScreen == SCREEN_HAMALERT_CLUSTER && !inMenu && !aprsAlertScreenActive && !qrzPopupActive) {
        if (now - lastScreen8UpdateMs > 200) {
          updateScreen8Data();
          lastScreen8UpdateMs = now;
        }
        // Aktualizacja zegara co sekundę
        static unsigned long lastHamalertClockUpdate = 0;
        if (now - lastHamalertClockUpdate >= 1000) {
          updateHamalertClock();
          lastHamalertClockUpdate = now;
        }
      }

      if (tftInitialized && (currentScreen == SCREEN_APRS_IS || currentScreen == SCREEN_APRS_RADAR) && !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreenUpdate > 100) {
          updateScreen6Data();
          lastScreenUpdate = now;
        }
      }

      if (tftInitialized && currentScreen == SCREEN_SUN_SPOTS && !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreen3UpdateMs > 1000) {
          updateScreen3Data();
          lastScreen3UpdateMs = now;
        }
      }

      if (tftInitialized && currentScreen == SCREEN_BAND_INFO && !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreen4UpdateMs > 1000) {
          updateScreen4Data();
          lastScreen4UpdateMs = now;
        }
        // Aktualizacja zegara co sekundę
        static unsigned long lastBandInfoClockUpdate = 0;
        if (now - lastBandInfoClockUpdate >= 1000) {
          updateBandInfoClock();
          lastBandInfoClockUpdate = now;
        }
      }

      if (tftInitialized &&
          (currentScreen == SCREEN_WEATHER_DSP || currentScreen == SCREEN_WEATHER_FORECAST) &&
          !inMenu && !aprsAlertScreenActive) {
        if (now - lastScreen5UpdateMs > 1000) {
          updateScreen5Data();
          lastScreen5UpdateMs = now;
        }
        // Aktualizacja zegara co sekundę
        static unsigned long lastWeatherClockUpdate = 0;
        if (now - lastWeatherClockUpdate >= 1000) {
          if (currentScreen == SCREEN_WEATHER_DSP) {
            updateWeatherClock();
          } else {
            updateWeatherForecastClock();
          }
          lastWeatherClockUpdate = now;
        }
      }

      if (tftInitialized && currentScreen == SCREEN_MATRIX_CLOCK && !inMenu && !aprsAlertScreenActive) {
        updateScreen10();
      }

      if (tftInitialized && currentScreen == SCREEN_UNLIS_HUNTER && !inMenu && !aprsAlertScreenActive) {
        updateUnlisHunter();
      }
      
      // Obsługa MQTT dla PSK Reporter (gdy aktywny ekran PSK i włączony MQTT)
      if (pskMqttEnabled && currentScreen == SCREEN_PSK_MAP && !pskMapMenuOpen) {
        loopPskMqtt();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
#endif

// ========== FUNKCJE PSK REPORTER MAP ==========

// Konwersja współrzędnych geograficznych na piksele ekranu - PROJEKCJA PLATE CARRÉE
static void latLonToScreen(float lat, float lon, int &x, int &y) {
  // Prosta projekcja walcowa (Plate Carrée) - liniowe odwzorowanie
  // X: liniowe odwzorowanie długości geograficznej (-180 do +180)
  x = MAP_DISPLAY_X + (int)((lon - MAP_LON_MIN) / (MAP_LON_MAX - MAP_LON_MIN) * MAP_DISPLAY_W);
  
  // Y: liniowe odwzorowanie szerokości geograficznej (+90 na górze, -90 na dole)
  // Odwrócone: lat_max - lat daje Y=0 dla +90, Y=max dla -90
  float yNormalized = (MAP_LAT_MAX - lat) / (MAP_LAT_MAX - MAP_LAT_MIN);
  y = MAP_DISPLAY_Y + (int)(yNormalized * MAP_DISPLAY_H);
}

// Pobieranie danych z PSK Reporter
bool fetchPskReporterData() {
  if (!wifiConnected) return false;

  HTTPClient http;
  // Użyj własnego URL jeśli ustawiony, w przeciwnym razie domyślny
  String baseUrl = pskCustomUrl.length() > 0 ? pskCustomUrl : "https://retrieve.pskreporter.info/query?";
  String url = baseUrl;
  // Upewnij się że URL kończy się ? lub &
  if (!url.endsWith("?") && !url.endsWith("&")) {
    url += url.indexOf("?") >= 0 ? "&" : "?";
  }
  // Użyj konfigurowalnego okna czasowego (w godzinach) lub raportu z dni wstecz
  int secondsWindow;
  if (pskReportDays > 0) {
    // Raport od X dni wstecz
    secondsWindow = pskReportDays * 24 * 3600;
  } else {
    // Standardowe okno czasowe w godzinach
    secondsWindow = pskHoursWindow * 3600;
  }
  url += "flowStartSeconds=-" + String(secondsWindow) + "&";
  url += "maxrows=" + String(pskMaxSpots) + "&";
  // Opcjonalnie dodaj filtr znaku odbiornika
  if (pskReceiverCallsign.length() > 0) {
    url += "receiverCallsign=" + pskReceiverCallsign + "&";
  }
  // Opcjonalnie dodaj filtr znaku nadawcy (monitorowanie)
  if (pskMonitorCallsign.length() > 0) {
    url += "transmitterCallsign=" + pskMonitorCallsign + "&";
  }
  url += "frags=1&";
  url += "json=1";

  Serial.print("[PSK] URL: ");
  Serial.println(url);

  http.setTimeout(10000);
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode != 200) {
    Serial.print("[PSK] HTTP error: ");
    Serial.println(httpCode);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  DynamicJsonDocument doc(8192);
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.print("[PSK] JSON parse error: ");
    Serial.println(error.c_str());
    return false;
  }

  pskSpotCount = 0;

  JsonArray reports = doc["receptions"].as<JsonArray>();
  if (reports.isNull()) {
    reports = doc["rx"].as<JsonArray>();
  }

  if (!reports.isNull()) {
    for (JsonObject report : reports) {
      if (pskSpotCount >= PSK_MAX_SPOTS) break;

      const char* callsign = report["callsign"];
      const char* locator = report["locator"];
      const char* mode = report["mode"];
      int snr = report["snr"] | 0;
      int frequency = report["frequency"] | 0;

      if (callsign && locator) {
        // Określ pasmo z częstotliwości
        int freq_khz = frequency / 1000;
        int band = 0;
        if (freq_khz >= 3500 && freq_khz <= 4000) band = 80;
        else if (freq_khz >= 7000 && freq_khz <= 7300) band = 40;
        else if (freq_khz >= 14000 && freq_khz <= 14350) band = 20;
        else if (freq_khz >= 21000 && freq_khz <= 21450) band = 15;
        else if (freq_khz >= 28000 && freq_khz <= 29700) band = 10;
        else if (freq_khz >= 50000 && freq_khz <= 54000) band = 6;

        // Filtruj po pasmie jeśli ustawiony
        if (pskFilterBand.length() > 0) {
          int filterBand = pskFilterBand.toInt();
          if (band != filterBand) continue; // Pomiń ten spot
        }

        // Filtruj po trybie jeśli ustawiony
        String modeStr = mode ? String(mode) : String("FT8");
        if (pskFilterMode.length() > 0) {
          if (!modeStr.equalsIgnoreCase(pskFilterMode)) continue; // Pomiń ten spot
        }

        double lat, lon;
        locatorToLatLon(String(locator), lat, lon);

        pskSpots[pskSpotCount].callsign = String(callsign);
        pskSpots[pskSpotCount].lat = (float)lat;
        pskSpots[pskSpotCount].lon = (float)lon;
        pskSpots[pskSpotCount].mode = modeStr;
        pskSpots[pskSpotCount].snr = snr;
        pskSpots[pskSpotCount].timestamp = millis();
        pskSpots[pskSpotCount].band = band;

        pskSpotCount++;
      }
    }
  }

  Serial.print("[PSK] Pobrano ");
  Serial.print(pskSpotCount);
  Serial.println(" spotów");

  return pskSpotCount > 0;
}

// Kolory dla pasm
static uint16_t getPskBandColor(int band) {
  switch (band) {
    case 80: return TFT_RED;
    case 40: return TFT_ORANGE;
    case 20: return TFT_GREEN;
    case 15: return TFT_CYAN;
    case 10: return TFT_BLUE;
    case 6:  return TFT_MAGENTA;
    default: return TFT_WHITE;
  }
}

// Rysowanie nagłówka (przezroczysty - na mapie)
static void drawPskMapHeader() {
  // Bez tła - tekst bezpośrednio na mapie
  tft.setTextSize(2);
  // Czarna obwódka dla kontrastu
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(12, 10);
  tft.print("PSKReporter MAP");
  tft.setCursor(11, 9);
  tft.print("PSKReporter MAP");
  tft.setCursor(13, 11);
  tft.print("PSKReporter MAP");
  // Główny tekst - biały
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(12, 10);
  tft.print("PSKReporter MAP");
  
  // Status auto-odświeżania
  tft.setTextSize(1);
  // Obwódka
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(322, 12);
  tft.print("AUTO:" + String(pskAutoRefreshMinutes > 0 ? String(pskAutoRefreshMinutes) + "min" : "OFF"));
  tft.setCursor(321, 11);
  tft.print("AUTO:" + String(pskAutoRefreshMinutes > 0 ? String(pskAutoRefreshMinutes) + "min" : "OFF"));
  // Tekst
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(322, 12);
  tft.print("AUTO:" + String(pskAutoRefreshMinutes > 0 ? String(pskAutoRefreshMinutes) + "min" : "OFF"));
}

// Rysowanie przycisku menu w lewym górnym rogu
static void drawPskMenuButton() {
  // Przycisk menu 30x26 w lewym górnym rogu (pomarańczowy jak nagłówek)
  tft.fillRect(2, 2, 30, 26, TFT_RADIO_ORANGE);
  tft.drawRect(2, 2, 30, 26, TFT_BLACK);
  // Trzy poziome kreski (hamburger menu)
  tft.fillRect(6, 8, 22, 3, TFT_BLACK);
  tft.fillRect(6, 13, 22, 3, TFT_BLACK);
  tft.fillRect(6, 18, 22, 3, TFT_BLACK);
}

// Rysowanie klawiatury ekranowej
static void drawPskKeyboard() {
  // Tło klawiatury
  tft.fillRect(30, 140, 420, 170, 0x18E3);
  tft.drawRect(30, 140, 420, 170, TFT_WHITE);
  
  // Wyświetl bufor
  tft.fillRect(40, 145, 300, 25, TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(45, 150);
  tft.print(pskKeyboardBuffer);
  tft.print("_");
  
  // Przyciski liter A-Z (3 wiersze)
  const char* rows[] = {"QWERTYUIOP", "ASDFGHJKL", "ZXCVBNM"};
  int startX[] = {40, 55, 85};
  int startY = 175;
  int keyW = 32, keyH = 22, gap = 4;
  
  for (int r = 0; r < 3; r++) {
    for (int i = 0; i < strlen(rows[r]); i++) {
      int x = startX[r] + i * (keyW + gap);
      int y = startY + r * (keyH + gap);
      tft.fillRect(x, y, keyW, keyH, TFT_DARKGREY);
      tft.drawRect(x, y, keyW, keyH, TFT_WHITE);
      tft.setTextSize(1);
      tft.setTextColor(TFT_WHITE);
      tft.setCursor(x + 12, y + 6);
      tft.print(rows[r][i]);
    }
  }
  
  // Cyfry 0-9 (góra)
  for (int i = 0; i < 10; i++) {
    int x = 40 + i * (keyW + gap);
    int y = 175;
    tft.fillRect(x, y, keyW, keyH, TFT_BLUE);
    tft.drawRect(x, y, keyW, keyH, TFT_WHITE);
    tft.setCursor(x + 12, y + 6);
    tft.print(i);
  }
  
  // Przyciski specjalne
  // Backspace
  tft.fillRect(350, 175, 50, keyH, TFT_RED);
  tft.drawRect(350, 175, 50, keyH, TFT_WHITE);
  tft.setCursor(355, 182);
  tft.print("<-");
  
  // Spacja
  tft.fillRect(350, 202, 80, keyH, TFT_DARKGREY);
  tft.drawRect(350, 202, 80, keyH, TFT_WHITE);
  tft.setCursor(370, 209);
  tft.print("_");
  
  // OK
  tft.fillRect(350, 230, 50, 30, TFT_GREEN);
  tft.drawRect(350, 230, 50, 30, TFT_WHITE);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(360, 240);
  tft.print("OK");
  
  // Anuluj
  tft.fillRect(410, 230, 50, 30, TFT_RED);
  tft.drawRect(410, 230, 50, 30, TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(415, 240);
  tft.print("X");
  
  // Slash / (dla znaków)
  tft.fillRect(350, 265, 32, keyH, TFT_BLUE);
  tft.drawRect(350, 265, 32, keyH, TFT_WHITE);
  tft.setCursor(360, 272);
  tft.print("/");
  
  // Minus - (dla znaków)
  tft.fillRect(390, 265, 32, keyH, TFT_BLUE);
  tft.drawRect(390, 265, 32, keyH, TFT_WHITE);
  tft.setCursor(400, 272);
  tft.print("-");
}

// Rysowanie wyboru pasm
static void drawPskBandSelector() {
  // Tło
  tft.fillRect(40, 80, 400, 160, 0x18E3);
  tft.drawRect(40, 80, 400, 160, TFT_WHITE);
  
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(140, 90);
  tft.print("Wybierz pasma:");
  
  const char* bands[] = {"160m", "80m", "40m", "30m", "20m", "17m", "15m", "12m", "10m", "6m", "ALL"};
  int bandCount = 11;
  int tileW = 60, tileH = 30, gap = 8;
  int startX = 60, startY = 120;
  int cols = 5;
  
  for (int i = 0; i < bandCount; i++) {
    int col = i % cols;
    int row = i / cols;
    int x = startX + col * (tileW + gap);
    int y = startY + row * (tileH + gap);
    
    bool selected = (pskTempBand == bands[i]) || 
                    (pskTempBand == "" && strcmp(bands[i], "ALL") == 0);
    
    tft.fillRect(x, y, tileW, tileH, selected ? TFT_GREEN : TFT_DARKGREY);
    tft.drawRect(x, y, tileW, tileH, TFT_WHITE);
    tft.setTextSize(1);
    tft.setTextColor(selected ? TFT_BLACK : TFT_WHITE);
    tft.setCursor(x + 15, y + 10);
    tft.print(bands[i]);
  }
  
  // Przycisk zamknij
  tft.fillRect(200, 200, 80, 30, TFT_RED);
  tft.drawRect(200, 200, 80, 30, TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(215, 210);
  tft.print("ZAMKNIJ");
}

// Rysowanie wyboru trybów
static void drawPskModeSelector() {
  // Tło
  tft.fillRect(40, 80, 400, 160, 0x18E3);
  tft.drawRect(40, 80, 400, 160, TFT_WHITE);
  
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(140, 90);
  tft.print("Wybierz tryb:");
  
  const char* modes[] = {"FT8", "FT4", "JS8", "PSK31", "PSK63", "RTTY", "CW", "SSB", "ALL"};
  int modeCount = 9;
  int tileW = 70, tileH = 30, gap = 10;
  int startX = 60, startY = 120;
  int cols = 4;
  
  for (int i = 0; i < modeCount; i++) {
    int col = i % cols;
    int row = i / cols;
    int x = startX + col * (tileW + gap);
    int y = startY + row * (tileH + gap);
    
    bool selected = (pskTempMode == modes[i]) || 
                    (pskTempMode == "" && strcmp(modes[i], "ALL") == 0);
    
    tft.fillRect(x, y, tileW, tileH, selected ? TFT_GREEN : TFT_DARKGREY);
    tft.drawRect(x, y, tileW, tileH, TFT_WHITE);
    tft.setTextSize(1);
    tft.setTextColor(selected ? TFT_BLACK : TFT_WHITE);
    tft.setCursor(x + 20, y + 10);
    tft.print(modes[i]);
  }
  
  // Przycisk zamknij
  tft.fillRect(200, 200, 80, 30, TFT_RED);
  tft.drawRect(200, 200, 80, 30, TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(215, 210);
  tft.print("ZAMKNIJ");
}

// Rysowanie menu ustawień PSK
static void drawPskSettingsMenu() {
  // Tło menu - półprzezroczyste ciemne (rozszerzone dla MQTT)
  tft.fillRect(40, 40, 400, 230, 0x18E3); // Ciemny szary
  tft.drawRect(40, 40, 400, 230, TFT_WHITE);
  
  // Tytuł
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(140, 50);
  tft.print("PSK Reporter - Filtry");
  
  // Linia podziału
  tft.drawLine(50, 70, 430, 70, TFT_WHITE);
  
  // Pola edycji
  tft.setTextSize(1);
  
  // Znak (receiver)
  tft.setTextColor(pskActiveField == PSK_FIELD_CALL ? TFT_YELLOW : TFT_WHITE);
  tft.setCursor(60, 90);
  tft.print("Znak:");
  tft.fillRect(120, 85, 100, 20, pskActiveField == PSK_FIELD_CALL ? TFT_DARKGREY : TFT_BLACK);
  tft.setCursor(125, 90);
  tft.print(pskTempReceiver.length() > 0 ? pskTempReceiver.c_str() : "(wszystkie)");
  
  // Pasmo
  tft.setTextColor(pskActiveField == PSK_FIELD_BAND ? TFT_YELLOW : TFT_WHITE);
  tft.setCursor(250, 90);
  tft.print("Pasmo:");
  tft.fillRect(300, 85, 80, 20, pskActiveField == PSK_FIELD_BAND ? TFT_DARKGREY : TFT_BLACK);
  tft.setCursor(305, 90);
  tft.print(pskTempBand.length() > 0 ? pskTempBand.c_str() : "(wszystkie)");
  
  // Tryb
  tft.setTextColor(pskActiveField == PSK_FIELD_MODE ? TFT_YELLOW : TFT_WHITE);
  tft.setCursor(60, 120);
  tft.print("Tryb:");
  tft.fillRect(120, 115, 100, 20, pskActiveField == PSK_FIELD_MODE ? TFT_DARKGREY : TFT_BLACK);
  tft.setCursor(125, 120);
  tft.print(pskTempMode.length() > 0 ? pskTempMode.c_str() : "(wszystkie)");
  
  // Max spotów
  tft.setTextColor(pskActiveField == PSK_FIELD_MAXSPOTS ? TFT_YELLOW : TFT_WHITE);
  tft.setCursor(250, 120);
  tft.print("Max:");
  tft.fillRect(300, 115, 50, 20, pskActiveField == PSK_FIELD_MAXSPOTS ? TFT_DARKGREY : TFT_BLACK);
  tft.setCursor(305, 120);
  tft.print(pskTempMaxSpots);
  
  // MQTT Mode
  tft.setTextColor(pskActiveField == PSK_FIELD_MQTT_MODE ? TFT_YELLOW : TFT_WHITE);
  tft.setCursor(60, 150);
  tft.print("Tryb:");
  tft.fillRect(120, 145, 100, 20, pskActiveField == PSK_FIELD_MQTT_MODE ? TFT_DARKGREY : TFT_BLACK);
  tft.setCursor(125, 150);
  tft.print(pskTempMqttEnabled ? "MQTT" : "HTTP");
  
  // MQTT Server (tylko gdy MQTT)
  if (pskTempMqttEnabled) {
    tft.setTextColor(pskActiveField == PSK_FIELD_MQTT_SERVER ? TFT_YELLOW : TFT_WHITE);
    tft.setCursor(250, 150);
    tft.print("Serwer:");
    tft.fillRect(300, 145, 120, 20, pskActiveField == PSK_FIELD_MQTT_SERVER ? TFT_DARKGREY : TFT_BLACK);
    tft.setCursor(305, 150);
    String srv = pskTempMqttServer.length() > 12 ? pskTempMqttServer.substring(0, 12) + "..." : pskTempMqttServer;
    tft.print(srv.length() > 0 ? srv.c_str() : "default");
  }
  
  // MQTT Callsign (tylko gdy MQTT)
  if (pskTempMqttEnabled) {
    tft.setTextColor(pskActiveField == PSK_FIELD_MQTT_CALL ? TFT_YELLOW : TFT_WHITE);
    tft.setCursor(60, 180);
    tft.print("MQTT Znak:");
    tft.fillRect(140, 175, 100, 20, pskActiveField == PSK_FIELD_MQTT_CALL ? TFT_DARKGREY : TFT_BLACK);
    tft.setCursor(145, 180);
    tft.print(pskTempMqttCallsign.length() > 0 ? pskTempMqttCallsign.c_str() : "(wszystkie)");
  }
  
  // Przyciski
  // Zapisz
  tft.fillRect(100, 210, 80, 30, TFT_GREEN);
  tft.drawRect(100, 210, 80, 30, TFT_WHITE);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(115, 220);
  tft.print("ZAPISZ");
  
  // Anuluj
  tft.fillRect(300, 210, 80, 30, TFT_RED);
  tft.drawRect(300, 210, 80, 30, TFT_WHITE);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(310, 220);
  tft.print("ANULUJ");
  
  // Podpowiedź
  tft.setTextColor(TFT_CYAN);
  tft.setCursor(60, 195);
  tft.print("Dotknij pola aby edytowac, ZAPISZ aby zastosowac");
  
  // Jeśli aktywna klawiatura - narysuj ją
  if (pskActiveField == PSK_FIELD_KEYBOARD) {
    drawPskKeyboard();
  }
  // Jeśli wybor pasma - narysuj selektor
  else if (pskActiveField == PSK_FIELD_BAND_SELECT) {
    drawPskBandSelector();
  }
  // Jeśli wybor trybu - narysuj selektor
  else if (pskActiveField == PSK_FIELD_MODE_SELECT) {
    drawPskModeSelector();
  }
}

// Funkcja obliczająca dystans (km) - wzór Haversine
static float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  float p = 0.017453292519943295; // Pi/180
  float a = 0.5 - cos((lat2 - lat1) * p) / 2 +
            cos(lat1 * p) * cos(lat2 * p) *
            (1 - cos((lon2 - lon1) * p)) / 2;
  return 12742 * asin(sqrt(a)); // 2 * R; R = 6371 km
}

// Funkcja zwracająca kolor linii zależny od odległości DX
static uint16_t getDxColor(float distance) {
  if (distance < 500) return TFT_GREEN;      // Bardzo blisko
  else if (distance < 2000) return TFT_YELLOW; // Europa / Średni dystans
  else if (distance < 5000) return TFT_ORANGE; // Daleko
  else return TFT_MAGENTA;                     // DX (Transatlantyki itp.)
}

// Rysowanie legendy
static void drawPskMapLegend() {
  int y = 300, x = 10, boxSize = 8, gap = 45;
  tft.setTextSize(1);
  const char* bands[] = {"80m", "40m", "20m", "15m", "10m", "6m"};
  uint16_t colors[] = {TFT_RED, TFT_ORANGE, TFT_GREEN, TFT_CYAN, TFT_BLUE, TFT_MAGENTA};
  for (int i = 0; i < 6; i++) {
    tft.fillRect(x + i*gap, y, boxSize, boxSize, colors[i]);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(x + i*gap + 12, y);
    tft.print(bands[i]);
  }
  tft.setTextColor(TFT_YELLOW);
  tft.setCursor(300, y);
  tft.print("Tap: Refresh");
}

// Główna funkcja rysująca
void drawPskMap() {
  Serial.printf("[PSK] drawPskMap start, pskMapMenuOpen=%d\n", pskMapMenuOpen);
  if (!tftInitialized) return;
  tft.fillScreen(TFT_BLACK);
  Serial.println("[PSK] Screen cleared");
  drawPskMapHeader();
  bool mapLoaded = false;
  if (littleFsReady) {
    mapLoaded = drawBmpFromFS(PSK_MAP_BMP_PATH, MAP_DISPLAY_X, MAP_DISPLAY_Y);
  }
  if (!mapLoaded) {
    tft.fillRect(MAP_DISPLAY_X, MAP_DISPLAY_Y, MAP_DISPLAY_W, MAP_DISPLAY_H, 0x0011);
    tft.fillRect(220, 80, 80, 100, 0x2222);
    tft.fillRect(200, 140, 100, 80, 0x2222);
    tft.fillRect(300, 60, 120, 80, 0x2222);
    tft.fillRect(60, 50, 100, 90, 0x2222);
    tft.fillRect(80, 150, 70, 90, 0x2222);
    tft.fillRect(380, 160, 60, 40, 0x2222);
  }
  unsigned long now = millis();
  
  // Wybierz tryb pracy: MQTT lub HTTP
  if (pskMqttEnabled) {
    // Tryb MQTT - loop obsługuje wszystko w tle
    loopPskMqtt();
  } else {
    // Tryb HTTP - auto-odświeżanie PSKReporter
    if (pskAutoRefreshMinutes > 0) {
      unsigned long refreshIntervalMs = (unsigned long)pskAutoRefreshMinutes * 60 * 1000;
      if (now - lastPskFetchMs > refreshIntervalMs) {
        fetchPskReporterData();
        lastPskFetchMs = now;
      }
    }
  }
  // Rysowanie linii od pozycji użytkownika do stacji
  Serial.printf("[PSK] drawPskMap START: userLat=%.4f, userLon=%.4f, valid=%d, locator='%s'\n",
                userLat, userLon, userLatLonValid, userLocator.c_str());
  int userX = 0, userY = 0;
  bool userOnMap = false;
  if (userLatLonValid && userLat >= MAP_LAT_MIN && userLat <= MAP_LAT_MAX &&
      userLon >= MAP_LON_MIN && userLon <= MAP_LON_MAX) {
    latLonToScreen(userLat, userLon, userX, userY);
    userOnMap = true;
    Serial.printf("[PSK] User on map: X=%d, Y=%d\n", userX, userY);
  }

  for (int i = 0; i < pskSpotCount; i++) {
    if (pskSpots[i].lat >= MAP_LAT_MIN && pskSpots[i].lat <= MAP_LAT_MAX &&
        pskSpots[i].lon >= MAP_LON_MIN && pskSpots[i].lon <= MAP_LON_MAX) {
      int x, y;
      latLonToScreen(pskSpots[i].lat, pskSpots[i].lon, x, y);
      uint16_t color = getPskBandColor(pskSpots[i].band);
      
      // Rysuj linię od użytkownika do stacji jeśli oba są na mapie
      // Kolor zależy od odległości DX
      if (userOnMap) {
        float dist = calculateDistance(userLat, userLon, pskSpots[i].lat, pskSpots[i].lon);
        uint16_t lineColor = getDxColor(dist);
        tft.drawLine(userX, userY, x, y, lineColor);
      }
      
      tft.fillCircle(x, y, 3, color);
      tft.drawCircle(x, y, 3, TFT_BLACK);
    }
  }

  // Rysuj czerwoną kropkę na pozycji użytkownika (na wierzchu)
  if (userOnMap) {
    Serial.printf("[PSK] Drawing RED DOT at X=%d, Y=%d\n", userX, userY);
    tft.fillCircle(userX, userY, 3, TFT_RED);
    tft.drawCircle(userX, userY, 4, TFT_WHITE);
  } else {
    Serial.printf("[PSK] NO DOT: valid=%d, lat=%.2f (%.1f to %.1f), lon=%.2f (%.1f to %.1f)\n", 
                  userLatLonValid, userLat, MAP_LAT_MIN, MAP_LAT_MAX, userLon, MAP_LON_MIN, MAP_LON_MAX);
  }
  drawPskMapLegend();
  drawSwitchScreenFooter();
  // Przycisk menu w lewym górnym rogu
  drawPskMenuButton();
  // Jeśli menu otwarte, narysuj je na wierzchu
  if (pskMapMenuOpen) {
    Serial.println("[PSK] Drawing settings menu");
    drawPskSettingsMenu();
  }
  Serial.println("[PSK] drawPskMap end");
}

// Obsługa dotyku klawiatury
static void handlePskKeyboardTouch(uint16_t x, uint16_t y) {
  int keyW = 32, keyH = 22, gap = 4;
  
  // OK (350,230,50,30)
  if (x >= 350 && x < 400 && y >= 230 && y < 260) {
    if (pskKeyboardTarget == 1) {
      pskTempReceiver = pskKeyboardBuffer;
    } else if (pskKeyboardTarget == 2) {
      pskTempMaxSpots = pskKeyboardBuffer.toInt();
      if (pskTempMaxSpots < 10) pskTempMaxSpots = 10;
      if (pskTempMaxSpots > 200) pskTempMaxSpots = 200;
    } else if (pskKeyboardTarget == 3) {
      pskTempMqttServer = pskKeyboardBuffer;
      if (pskTempMqttServer.length() == 0) {
        pskTempMqttServer = "mqtt.pskreporter.info"; // default
      }
    } else if (pskKeyboardTarget == 4) {
      pskTempMqttCallsign = pskKeyboardBuffer;
    }
    pskActiveField = PSK_FIELD_NONE;
    pskKeyboardBuffer = "";
    pskKeyboardActive = false;  // Odblokuj nawigację
    drawPskSettingsMenu();
    return;
  }
  
  // Anuluj (410,230,50,30)
  if (x >= 410 && x < 460 && y >= 230 && y < 260) {
    pskActiveField = PSK_FIELD_NONE;
    pskKeyboardBuffer = "";
    pskKeyboardActive = false;  // Odblokuj nawigację
    drawPskSettingsMenu();
    return;
  }
  
  // Backspace (350,175,50,22)
  if (x >= 350 && x < 400 && y >= 175 && y < 197) {
    if (pskKeyboardBuffer.length() > 0) {
      pskKeyboardBuffer.remove(pskKeyboardBuffer.length() - 1);
      drawPskKeyboard();
    }
    return;
  }
  
  // Spacja (350,202,80,22)
  if (x >= 350 && x < 430 && y >= 202 && y < 224) {
    if (pskKeyboardBuffer.length() < 20) {
      pskKeyboardBuffer += " ";
      drawPskKeyboard();
    }
    return;
  }
  
  // Slash / (350,265,32,22)
  if (x >= 350 && x < 382 && y >= 265 && y < 287) {
    if (pskKeyboardBuffer.length() < 20) {
      pskKeyboardBuffer += "/";
      drawPskKeyboard();
    }
    return;
  }
  
  // Minus - (390,265,32,22)
  if (x >= 390 && x < 422 && y >= 265 && y < 287) {
    if (pskKeyboardBuffer.length() < 20) {
      pskKeyboardBuffer += "-";
      drawPskKeyboard();
    }
    return;
  }
  
  // Cyfry 0-9 (wiersz 175)
  for (int i = 0; i < 10; i++) {
    int keyX = 40 + i * (keyW + gap);
    if (x >= keyX && x < keyX + keyW && y >= 175 && y < 197) {
      if (pskKeyboardBuffer.length() < 20) {
        pskKeyboardBuffer += String(i);
        drawPskKeyboard();
      }
      return;
    }
  }
  
  // Litery QWERTYUIOP (wiersz 175, x od 40)
  const char* row1 = "QWERTYUIOP";
  int startX1 = 40;
  for (int i = 0; i < 10; i++) {
    int keyX = startX1 + i * (keyW + gap);
    if (x >= keyX && x < keyX + keyW && y >= 175 && y < 197) {
      if (pskKeyboardBuffer.length() < 20) {
        pskKeyboardBuffer += row1[i];
        drawPskKeyboard();
      }
      return;
    }
  }
  
  // Litery ASDFGHJKL (wiersz 201, x od 55)
  const char* row2 = "ASDFGHJKL";
  int startX2 = 55;
  for (int i = 0; i < 9; i++) {
    int keyX = startX2 + i * (keyW + gap);
    if (x >= keyX && x < keyX + keyW && y >= 201 && y < 223) {
      if (pskKeyboardBuffer.length() < 20) {
        pskKeyboardBuffer += row2[i];
        drawPskKeyboard();
      }
      return;
    }
  }
  
  // Litery ZXCVBNM (wiersz 227, x od 85)
  const char* row3 = "ZXCVBNM";
  int startX3 = 85;
  for (int i = 0; i < 7; i++) {
    int keyX = startX3 + i * (keyW + gap);
    if (x >= keyX && x < keyX + keyW && y >= 227 && y < 249) {
      if (pskKeyboardBuffer.length() < 20) {
        pskKeyboardBuffer += row3[i];
        drawPskKeyboard();
      }
      return;
    }
  }
}

// Obsługa dotyku selektora pasm
static void handlePskBandSelectorTouch(uint16_t x, uint16_t y) {
  const char* bands[] = {"160m", "80m", "40m", "30m", "20m", "17m", "15m", "12m", "10m", "6m", "ALL"};
  int bandCount = 11;
  int tileW = 60, tileH = 30, gap = 8;
  int startX = 60, startY = 120;
  int cols = 5;
  
  // Sprawdź przyciski pasm
  for (int i = 0; i < bandCount; i++) {
    int col = i % cols;
    int row = i / cols;
    int bx = startX + col * (tileW + gap);
    int by = startY + row * (tileH + gap);
    
    if (x >= bx && x < bx + tileW && y >= by && y < by + tileH) {
      if (strcmp(bands[i], "ALL") == 0) {
        pskTempBand = "";
      } else {
        pskTempBand = bands[i];
      }
      pskActiveField = PSK_FIELD_NONE;
      drawPskSettingsMenu();
      return;
    }
  }
  
  // Zamknij (200,200,80,30)
  if (x >= 200 && x < 280 && y >= 200 && y < 230) {
    pskActiveField = PSK_FIELD_NONE;
    drawPskSettingsMenu();
  }
}

// Obsługa dotyku selektora trybów
static void handlePskModeSelectorTouch(uint16_t x, uint16_t y) {
  const char* modes[] = {"FT8", "FT4", "JS8", "PSK31", "PSK63", "RTTY", "CW", "SSB", "ALL"};
  int modeCount = 9;
  int tileW = 70, tileH = 30, gap = 10;
  int startX = 60, startY = 120;
  int cols = 4;
  
  // Sprawdź przyciski trybów
  for (int i = 0; i < modeCount; i++) {
    int col = i % cols;
    int row = i / cols;
    int mx = startX + col * (tileW + gap);
    int my = startY + row * (tileH + gap);
    
    if (x >= mx && x < mx + tileW && y >= my && y < my + tileH) {
      if (strcmp(modes[i], "ALL") == 0) {
        pskTempMode = "";
      } else {
        pskTempMode = modes[i];
      }
      pskActiveField = PSK_FIELD_BAND_SELECT; // Otwórz wybór pasma po wyborze trybu
      drawPskSettingsMenu();
      return;
    }
  }
  
  // Zamknij (200,200,80,30)
  if (x >= 200 && x < 280 && y >= 200 && y < 230) {
    pskActiveField = PSK_FIELD_NONE;
    drawPskSettingsMenu();
  }
}

// Obsługa dotyku - główna funkcja
void handlePskMapTouch(uint16_t x, uint16_t y) {
  // Obsługa menu jeśli otwarte
  if (pskMapMenuOpen) {
    // Jeśli aktywna klawiatura
    if (pskActiveField == PSK_FIELD_KEYBOARD) {
      handlePskKeyboardTouch(x, y);
      return;
    }
    // Jeśli aktywny selektor pasm
    if (pskActiveField == PSK_FIELD_BAND_SELECT) {
      handlePskBandSelectorTouch(x, y);
      return;
    }
    // Jeśli aktywny selektor trybów
    if (pskActiveField == PSK_FIELD_MODE_SELECT) {
      handlePskModeSelectorTouch(x, y);
      return;
    }
    
    // Sprawdź czy dotknięto przycisk ZAPISZ (100,210,80,30)
    Serial.printf("[PSK] Checking ZAPISZ button at x=%d, y=%d, pskMapMenuOpen=%d\n", x, y, pskMapMenuOpen);
    if (x >= 100 && x < 180 && y >= 210 && y < 240) {
      Serial.println("[PSK] ZAPISZ button pressed!");
      // Zapisz ustawienia
      pskReceiverCallsign = pskTempReceiver;
      pskFilterBand = pskTempBand;
      pskFilterMode = pskTempMode;
      pskMaxSpots = pskTempMaxSpots;
      pskMqttEnabled = pskTempMqttEnabled;
      pskMqttServer = pskTempMqttServer;
      pskMqttCallsign = pskTempMqttCallsign;
      Serial.printf("[PSK] Settings: receiver=%s, band=%s, mode=%s, max=%d, mqtt=%d\n", 
                    pskReceiverCallsign.c_str(), pskFilterBand.c_str(), 
                    pskFilterMode.c_str(), pskMaxSpots, pskMqttEnabled);
      // Zapisz do NVS
      preferences->putString("psk_receiver", pskReceiverCallsign);
      preferences->putString("psk_band", pskFilterBand);
      preferences->putString("psk_mode", pskFilterMode);
      preferences->putInt("psk_maxspots", pskMaxSpots);
      preferences->putBool("psk_mqtt_en", pskMqttEnabled);
      preferences->putString("psk_mqtt_srv", pskMqttServer);
      preferences->putString("psk_mqtt_call", pskMqttCallsign);
      pskMapMenuOpen = false;
      pskActiveField = PSK_FIELD_NONE;
      Serial.printf("[PSK] pskMapMenuOpen set to: %d\n", pskMapMenuOpen);
      Serial.println("[PSK] Closing menu and redrawing map...");
      // Zresetuj timer - dane pobiorą się automatycznie przy następnym odświeżeniu (nie blokuj UI)
      lastPskFetchMs = 0;
      Serial.printf("[PSK] Before drawPskMap, pskMapMenuOpen=%d\n", pskMapMenuOpen);
      drawPskMap();
      Serial.printf("[PSK] After drawPskMap, pskMapMenuOpen=%d\n", pskMapMenuOpen);
      Serial.println("[PSK] Map redrawn, menu should be closed");
      // Wyświetl potwierdzenie zapisu
      tft.fillRect(140, 130, 200, 40, TFT_GREEN);
      tft.drawRect(140, 130, 200, 40, TFT_WHITE);
      tft.setTextColor(TFT_BLACK);
      tft.setTextSize(2);
      tft.setCursor(170, 143);
      tft.print("ZAPISANO!");
      delay(1000);
      drawPskMap();  // Odśwież ponownie aby usunąć napis
      return;
    }
    // Sprawdź czy dotknięto przycisk ANULUJ (300,210,80,30)
    if (x >= 300 && x < 380 && y >= 210 && y < 240) {
      Serial.println("[PSK] ANULUJ button pressed!");
      pskMapMenuOpen = false;
      pskActiveField = PSK_FIELD_NONE;
      drawPskMap();
      return;
    }
    // Sprawdź czy dotknięto pole ZNAK (120,85,100,20) - otwórz klawiaturę
    if (x >= 120 && x < 220 && y >= 85 && y < 105) {
      pskKeyboardTarget = 1; // 1 = znak
      pskKeyboardBuffer = pskTempReceiver;
      pskActiveField = PSK_FIELD_KEYBOARD;
      pskKeyboardActive = true;  // Blokuj nawigację
      drawPskSettingsMenu();
      return;
    }
    // Sprawdź czy dotknięto pole PASMO (300,85,80,20) - otwórz selektor pasm
    if (x >= 300 && x < 380 && y >= 85 && y < 105) {
      pskActiveField = PSK_FIELD_BAND_SELECT;
      drawPskSettingsMenu();
      return;
    }
    // Sprawdź czy dotknięto pole TRYB (120,115,100,20) - otwórz selektor trybów
    if (x >= 120 && x < 220 && y >= 115 && y < 135) {
      pskActiveField = PSK_FIELD_MODE_SELECT;
      drawPskSettingsMenu();
      return;
    }
    // Sprawdź czy dotknięto pole MAX (300,115,50,20) - otwórz klawiaturę
    if (x >= 300 && x < 350 && y >= 115 && y < 135) {
      pskKeyboardTarget = 2; // 2 = max
      pskKeyboardBuffer = String(pskTempMaxSpots);
      pskActiveField = PSK_FIELD_KEYBOARD;
      pskKeyboardActive = true;  // Blokuj nawigację
      drawPskSettingsMenu();
      return;
    }
    // Sprawdź czy dotknięto pole TRYB MQTT (120,145,100,20)
    if (x >= 120 && x < 220 && y >= 145 && y < 165) {
      pskTempMqttEnabled = !pskTempMqttEnabled;
      pskActiveField = PSK_FIELD_NONE;
      drawPskSettingsMenu();
      return;
    }
    
    // Sprawdź czy dotknięto pole MQTT SERWER (300,145,120,20) - tylko gdy MQTT
    if (pskTempMqttEnabled && x >= 300 && x < 420 && y >= 145 && y < 165) {
      pskKeyboardTarget = 3; // 3 = mqtt server
      pskKeyboardBuffer = pskTempMqttServer;
      pskActiveField = PSK_FIELD_KEYBOARD;
      pskKeyboardActive = true;
      drawPskSettingsMenu();
      return;
    }
    
    // Sprawdź czy dotknięto pole MQTT ZNAK (140,175,100,20) - tylko gdy MQTT
    if (pskTempMqttEnabled && x >= 140 && x < 240 && y >= 175 && y < 195) {
      pskKeyboardTarget = 4; // 4 = mqtt callsign
      pskKeyboardBuffer = pskTempMqttCallsign;
      pskActiveField = PSK_FIELD_KEYBOARD;
      pskKeyboardActive = true;
      drawPskSettingsMenu();
      return;
    }
    
    // Jeśli dotknięto poza polem menu, zamknij menu
    if (!(x >= 40 && x < 440 && y >= 40 && y < 270)) {
      pskMapMenuOpen = false;
      pskActiveField = PSK_FIELD_NONE;
      drawPskMap();
    }
    return;
  }
  
  // Sprawdź czy dotknięto przycisk menu (2,2,30,26)
  if (x >= 2 && x < 32 && y >= 2 && y < 28) {
    Serial.println("[PSK] Menu button pressed - opening menu");
    pskMapMenuOpen = true;
    pskActiveField = PSK_FIELD_NONE;
    // Skopiuj aktualne ustawienia do tymczasowych
    pskTempReceiver = pskReceiverCallsign;
    pskTempBand = pskFilterBand;
    pskTempMode = pskFilterMode;
    pskTempMaxSpots = pskMaxSpots;
    pskTempMqttEnabled = pskMqttEnabled;
    pskTempMqttServer = pskMqttServer;
    pskTempMqttCallsign = pskMqttCallsign;
    drawPskMap();
    return;
  }
  
  // Najpierw sprawdź rogi ekranu (przełączanie ekranów) - MUSI być przed mapą!
  const uint16_t cornerY = 280, cornerX = 80;  // Podniesione Y dla mapy 320px
  if (y >= cornerY && x < cornerX) {
    currentScreen = getNextScreenId(currentScreen);
    drawScreen(currentScreen);
    resetTftAutoSwitchTimer();
    return;
  } else if (y >= cornerY && x >= (480 - cornerX)) {
    currentScreen = getPrevScreenId(currentScreen);
    drawScreen(currentScreen);
    resetTftAutoSwitchTimer();
    return;
  }
  
  // Dotknięcie mapy odświeża dane (ale tylko jeśli nie w rogach)
  if (x >= MAP_DISPLAY_X && x < MAP_DISPLAY_X + MAP_DISPLAY_W &&
      y >= MAP_DISPLAY_Y && y < MAP_DISPLAY_Y + MAP_DISPLAY_H) {
    lastPskFetchMs = 0;
    drawPskMap();
    return;
  }
}

// ========== MQTT PSK REPORTER IMPLEMENTATION ==========

// Konfiguracja MQTT - konkretny topic zamiast szerokiego wildcard
typedef struct {
  String callsign;
  float lat;
  float lon;
  uint8_t band;
  String mode;
  unsigned long receivedAt;
} PskSpotMqtt;

#define PSK_MQTT_SPOT_BUFFER_SIZE 30
PskSpotMqtt pskMqttSpotBuffer[PSK_MQTT_SPOT_BUFFER_SIZE];
int pskMqttSpotCount = 0;
unsigned long lastPskMqttDrawMs = 0;
const unsigned long PSK_MQTT_DRAW_INTERVAL_MS = 1500; // Rysuj co 1.5 sekundy
bool pskMqttConnected = false;
unsigned long lastPskMqttReconnectAttempt = 0;
const unsigned long PSK_MQTT_RECONNECT_INTERVAL_MS = 5000; // Co 5 sekund próba reconnect

// Generuje precyzyjny topic na podstawie lokatora użytkownika (tylko pierwsze 4 znaki = duży kwadrat ~100x200km)
static String getPskMqttTopicForLocator(const String& locator) {
  if (locator.length() >= 4) {
    String loc4 = locator.substring(0, 4);
    loc4.toUpperCase();
    // Format: pskr/filter/v2/+/+/+/{loc4}/#
    // To ogranicza spam do konkretnego dużego kwadratu lokatora
    return "pskr/filter/v2/+/+/+/" + loc4 + "/#";
  }
  // Fallback na szeroki topic tylko gdy brak lokatora
  return "pskr/filter/v2/+/+/+/+/+";
}

// Callback dla MQTT - parsuje JSON i dodaje do bufora
void pskMqttCallback(char* topic, byte* payload, unsigned int length) {
  // Ogranicz rozmiar payloadu dla bezpieczeństwa
  if (length > 1024) {
    Serial.println("[PSK MQTT] Payload too large, skipping");
    return;
  }

  // Parsuj JSON
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    return; // Ciche pominięcie błędów parsowania
  }

  // Pobierz pola z JSON
  const char* sender = doc["sender"];
  const char* receiver = doc["receiver"];
  const char* receiverLocator = doc["receiverLocator"];
  const char* senderLocator = doc["senderLocator"];
  const char* mode = doc["mode"];
  float frequency = doc["frequency"] | 0.0f;

  if (!sender || !senderLocator) return;

  // Konwersja lokatora na lat/lon
  double lat = 0, lon = 0;
  String locStr(senderLocator);
  locatorToLatLon(locStr, lat, lon);

  // Określ pasmo
  int band = 0;
  int freq_khz = (int)(frequency / 1000);
  if (freq_khz >= 3500 && freq_khz <= 4000) band = 80;
  else if (freq_khz >= 7000 && freq_khz <= 7300) band = 40;
  else if (freq_khz >= 14000 && freq_khz <= 14350) band = 20;
  else if (freq_khz >= 21000 && freq_khz <= 21450) band = 15;
  else if (freq_khz >= 28000 && freq_khz <= 29700) band = 10;
  else if (freq_khz >= 50000 && freq_khz <= 54000) band = 6;

  // Filtruj po pasmie jeśli ustawiony
  if (pskFilterBand.length() > 0) {
    int filterBand = pskFilterBand.toInt();
    if (band != filterBand) return;
  }

  // Filtruj po trybie jeśli ustawiony
  String modeStr = mode ? String(mode) : String("FT8");
  if (pskFilterMode.length() > 0) {
    if (!modeStr.equalsIgnoreCase(pskFilterMode)) return;
  }

  // Sprawdź czy monitorujemy konkretny znak
  if (pskMqttCallsign.length() > 0) {
    String monitorCall = pskMqttCallsign;
    monitorCall.toUpperCase();
    String senderCall(sender);
    senderCall.toUpperCase();
    if (senderCall != monitorCall) return;
  }

  // Dodaj do bufora
  if (pskMqttSpotCount < PSK_MQTT_SPOT_BUFFER_SIZE) {
    pskMqttSpotBuffer[pskMqttSpotCount].callsign = String(sender);
    pskMqttSpotBuffer[pskMqttSpotCount].lat = (float)lat;
    pskMqttSpotBuffer[pskMqttSpotCount].lon = (float)lon;
    pskMqttSpotBuffer[pskMqttSpotCount].band = band;
    pskMqttSpotBuffer[pskMqttSpotCount].mode = modeStr;
    pskMqttSpotBuffer[pskMqttSpotCount].receivedAt = millis();
    pskMqttSpotCount++;

    // Debug co 10 spotów
    if (pskMqttSpotCount % 10 == 0) {
      Serial.printf("[PSK MQTT] Bufor: %d spotów\n", pskMqttSpotCount);
    }
  }
}

// Łączenie z MQTT z timeout i loop() - naprawia problem zawieszania
void reconnectPskMqtt() {
  if (!wifiConnected) return;

  unsigned long now = millis();
  // Sprawdź czy minął czas od ostatniej próby
  if (now - lastPskMqttReconnectAttempt < PSK_MQTT_RECONNECT_INTERVAL_MS) {
    return;
  }
  lastPskMqttReconnectAttempt = now;

  // Jeśli już połączony, tylko loop()
  if (pskMqttClient.connected()) {
    pskMqttClient.loop();
    return;
  }

  Serial.print("[PSK MQTT] Łączenie z MQTT...");

  // Generuj client ID
  String clientId = "ESP32-HAM-CLOCK-" + String(random(0xffff), HEX);

  // Timeout dla połączenia - nie blokujący
  unsigned long connectStart = millis();
  bool connected = false;

  // Próba połączenia z timeout 5 sekund
  while (!connected && (millis() - connectStart < 5000)) {
    connected = pskMqttClient.connect(clientId.c_str());
    if (!connected) {
      pskMqttClient.loop(); // KLUCZOWE: loop() podczas prób!
      delay(100);
    }
  }

  if (connected) {
    Serial.println(" połączono!");
    pskMqttConnected = true;

    // Subskrybuj precyzyjny topic
    String topic;
    if (userLocator.length() >= 4) {
      topic = getPskMqttTopicForLocator(userLocator);
    } else {
      // Fallback - subskrybuj wszystko (może być spam)
      topic = "pskr/filter/v2/+/+/+/+/+";
    }

    Serial.printf("[PSK MQTT] Subskrypcja: %s\n", topic.c_str());
    pskMqttClient.subscribe(topic.c_str());
  } else {
    Serial.println(" nieudane, ponowię za 5s");
    pskMqttConnected = false;
  }
}

// Główna funkcja loop dla MQTT
void loopPskMqtt() {
  if (!pskMqttEnabled) return;
  if (!wifiConnected) {
    pskMqttConnected = false;
    return;
  }

  // Konfiguracja przy pierwszym uruchomieniu
  static bool mqttInitialized = false;
  if (!mqttInitialized) {
    pskMqttClient.setServer(pskMqttServer.c_str(), pskMqttPort);
    pskMqttClient.setCallback(pskMqttCallback);
    mqttInitialized = true;
  }

  // Reconnect lub utrzymanie połączenia (z loop() w środku!)
  reconnectPskMqtt();

  // Obsługa MQTT loop
  if (pskMqttClient.connected()) {
    pskMqttClient.loop();
  }

  // Przetwarzanie bufora i rysowanie co zadany interwał
  processPskMqttBuffer();
}

// Przetwarza bufor i kopiuje do głównej tablicy spotów dla wyświetlenia
void processPskMqttBuffer() {
  unsigned long now = millis();

  // Sprawdź czy czas na rysowanie
  if (now - lastPskMqttDrawMs < PSK_MQTT_DRAW_INTERVAL_MS) {
    return;
  }
  lastPskMqttDrawMs = now;

  if (pskMqttSpotCount == 0) return;

  // Kopiuj do głównej tablicy spotów
  pskSpotCount = 0;
  for (int i = 0; i < pskMqttSpotCount && i < PSK_MAX_SPOTS; i++) {
    pskSpots[pskSpotCount].callsign = pskMqttSpotBuffer[i].callsign;
    pskSpots[pskSpotCount].lat = pskMqttSpotBuffer[i].lat;
    pskSpots[pskSpotCount].lon = pskMqttSpotBuffer[i].lon;
    pskSpots[pskSpotCount].band = pskMqttSpotBuffer[i].band;
    pskSpots[pskSpotCount].mode = pskMqttSpotBuffer[i].mode;
    pskSpots[pskSpotCount].snr = 0; // MQTT nie daje SNR
    pskSpots[pskSpotCount].timestamp = pskMqttSpotBuffer[i].receivedAt;
    pskSpotCount++;
  }

  // Czyść bufor
  pskMqttSpotCount = 0;

  // Odśwież wyświetlacz jeśli na ekranie PSK
  if (currentScreen == SCREEN_PSK_MAP && tftInitialized && !pskMapMenuOpen) {
    Serial.printf("[PSK MQTT] Rysowanie %d spotów\n", pskSpotCount);
    drawPskMap();
  }
}

// Setup MQTT - wywoływane przy starcie
void setupPskMqtt() {
  pskMqttClient.setServer(pskMqttServer.c_str(), pskMqttPort);
  pskMqttClient.setCallback(pskMqttCallback);
  Serial.println("[PSK MQTT] Zainicjalizowano klienta MQTT");
}

// ========== SETUP & LOOP ==========

void setup() {
  // Najpierw inicjalizacja Serial dla diagnostyki
  Serial.begin(115200);
  delay(100);
  Serial.println("\n\n[SETUP] START");
  
  // Disable brownout detector to prevent resets due to voltage drops
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  // Zapisz czas startu systemu
  bootTimeMs = millis();
  
  // Wyczyść cache QRZ przy starcie (aby wymusić pobranie nowych danych z lat/lon)
  for (int i = 0; i < QRZ_CACHE_SIZE; i++) {
    qrzCache[i].callsign = "";
    qrzCache[i].fetchedAtMs = 0;
  }
  Serial.println("[SETUP] QRZ cache cleared");

  // Ten log idzie "kanałem" (zwykle tym samym co bootlog),
  // Na ESP32-C3 (zwłaszcza z USB CDC) warto chwilę poczekać na monitor portu
  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < 2000) {
    delay(10);
  }
  delay(200);
  Serial.println("[SETUP] Serial ready");

  // Daj chwilĂ„â„˘ na ustabilizowanie WiFi/PHY po resecie (zwÄąâ€šaszcza na "Super Mini")
  delay(300);

  if (dxSpotsMutex == nullptr) {
    dxSpotsMutex = xSemaphoreCreateMutex();
  }

  initStatusRgbLed();
  updateStatusRgbLed();

  // Inicjalizacja pomiaru napięcia baterii (TP4056 + 18650)
#ifdef BATTERY_MONITORING_ENABLED
  pinMode(BATTERY_ADC_PIN, INPUT);
  analogSetAttenuation(ADC_11db);  // Pełny zakres 0-3.3V
  analogReadResolution(BATTERY_ADC_RESOLUTION);
  // Pobierz pierwszy odczyt dla wstępnej kalibracji
  delay(10);
  batteryVoltage = readBatteryVoltage();
  batteryPercentage = calculateBatteryPercentage(batteryVoltage);
  batteryCharging = (batteryVoltage > 4.25f);
  Serial.printf("[BATTERY] Init: %.2fV (%d%%) %s\n", 
                batteryVoltage, batteryPercentage, 
                batteryCharging ? "[CHARGING]" : "");
#endif
  
  // WyÄąâ€şwietlacz TFT (ESP32-2432S028) Ă˘â‚¬â€ś inicjalizacja NA POCZĂ„â€žTKU (jak w projekcie referencyjnym)
  // TFT powinien byĂ„â€ˇ inicjalizowany zaraz po Serial.begin(), przed innymi peryferiami
#ifdef ENABLE_TFT_DISPLAY
  initTFT();
  yield();
#endif

  // Inicjalizacja LittleFS z wieloma próbami i diagnostyką
  Serial.println("[LittleFS] Starting initialization...");
  int littleFsAttempts = 0;
  while (!littleFsReady && littleFsAttempts < 3) {
    littleFsAttempts++;
    Serial.printf("[LittleFS] Attempt %d/3...\n", littleFsAttempts);
    if(LittleFS.begin(true)){
      Serial.println("[LittleFS] Mounted successfully!");
      littleFsReady = true;
    } else {
      Serial.printf("[LittleFS] Mount attempt %d FAILED\n", littleFsAttempts);
      delay(200);
    }
  }
  
  if (!littleFsReady) {
    Serial.println("[LittleFS] ERROR: All mount attempts failed!");
    Serial.println("[LittleFS] Check partition scheme in Arduino IDE:");
    Serial.println("  Tools -> Partition Scheme -> should include SPIFFS or LittleFS");
    bootLogLine("LittleFS: BRAK - sprawdz schemat partycji!");
  } else {
    // Sprawdź co jest w LittleFS
    Serial.println("[LittleFS] Checking files:");
    if (LittleFS.exists("/littlefs_data/splash.bmp")) {
      Serial.println("  /littlefs_data/splash.bmp - OK");
    } else {
      Serial.println("  /littlefs_data/splash.bmp - NOT FOUND");
    }
    bootLogLine("LittleFS: OK");
  }

  bootLogLine("");
  bootLogLine("ESP32 DX Cluster Receiver");
  bootLogLine("==============================");
  
  // DIAGNOSTYKA: Sprawdź czy ENABLE_TFT_DISPLAY jest zdefiniowane
#ifdef ENABLE_TFT_DISPLAY
  bootLogLine("ENABLE_TFT_DISPLAY - init TFT");
#else
  bootLogLine("UWAGA: ENABLE_TFT_DISPLAY NIE jest zdefiniowane - TFT dont work!");
  bootLogLine("Upewnij sie, że kompilujesz dla środowiska 'esp32-2432s028'");
#endif
  yield();

  bootLogLine("Config load...");
  bootLogLine("LittleFS ready: " + String(littleFsReady ? "YES" : "NO"));
  loadPreferences();
  
  // TYMCZASOWA NAPRAWA KONFIGURACJI
  if (clusterHost == "Multiplay_C984" || clusterHost.length() == 0) {
    clusterHost = "dxspots.com";
    bootLogLine("FIX: clusterHost reset to dxspots.com");
  }
  if (hamalertHost == "Multiplay_C984" || hamalertHost.length() == 0) {
    hamalertHost = "hamalert.org";
    bootLogLine("FIX: hamalertHost reset to hamalert.org");
  }
  if (hamalertLogin == "Multiplay_C984" || hamalertLogin.length() == 0) {
    hamalertLogin = "";
    bootLogLine("FIX: hamalertLogin reset to empty");
  }
  if (hamalertPassword == "TWOJE_HASLO" || hamalertPassword.length() == 0) {
    hamalertPassword = "";
    bootLogLine("FIX: hamalertPassword reset to empty");
  }
  if (aprsIsHost == "Multiplay_C984" || aprsIsHost.length() == 0) {
    aprsIsHost = "rotate.aprs2.net";
    bootLogLine("FIX: aprsIsHost reset to rotate.aprs2.net");
  }
  if (wifiSSID == "Multiplay_C984") {
    wifiSSID = "Multiplay_C984";  // To jest OK - Twoja sieć
  }
  savePreferences();
  bootLogLine("Config fixed and saved!");
  telnetBuffer.reserve(384);
  pendingTelnetLine.reserve(384);
  potaTelnetBuffer.reserve(384);
  pendingPotaLine.reserve(384);
  aprsBuffer.reserve(384);
  qrzStatus.reserve(64);
  clusterFilterCommands.reserve(128);
  weatherData.description.reserve(96);
  weatherData.cityName.reserve(48);
  weatherData.forecast3hDesc.reserve(64);
  weatherData.forecastNextDayDesc.reserve(64);
  weatherData.lastError.reserve(64);
  weatherData.iconCode.reserve(8);
  propagationData.lastError.reserve(48);
  yield();
#ifdef ENABLE_TFT_DISPLAY
  applyTftRotation();
  applyTouchRotation();
  setBacklightPercent(backlightPercent);
#endif
  
  bootLogLine("WIFI starting...");
  if (wifiSSID.length() > 0) {
    bootLogLine("Connecting WiFi: " + wifiSSID);
    if (connectToWiFi()) {
      bootLogLine("WiFi connected!");
      yield();
      updateNTPTime();
      yield();
      connectToCluster();
      connectToPotaCluster();
    } else {
      bootLogLine("WiFi error, starting AP mode...");
      startAPMode();
    }
  } else {
    bootLogLine("No WiFi configuration, starting AP mode...");
    startAPMode();
  }
  yield();
  
  bootLogLine("Starting web server...");
  setupWebServer();
  yield();
  
  bootLogLine("System ready!");
  String ipStr = wifiConnected ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  bootLogLine("IP: " + ipStr);

  // Inicjalizacja śledzenia ISS
  initIssTracking();
  bootLogLine("ISS tracking initialized");
  
  // Aktualizuj wyświetlacz TFT (ekran startowy)
#ifdef ENABLE_TFT_DISPLAY
  // Standardowe ekrany powitalne
  drawWelcomeScreenYellow();
  delay(2000);
  drawWelcomeScreenGreen();
  delay(3000);
  
  // Ekran splash SP9TNV z opcją pominięcia - wyświetla się PO ekranach powitalnych
  drawSplashScreen();
  
  // Pokaż ikonę baterii od razu na ekranie splash
#ifdef BATTERY_MONITORING_ENABLED
  drawBatteryQuickUpdate(true);
#endif
  
  // Czekaj na dotyk lub timeout 10 sekund
  unsigned long splashStart = millis();
  bool splashSkipped = false;
  unsigned long lastBatteryUpdate = 0;
  while (!splashSkipped && (millis() - splashStart) < 10000) {
    uint16_t tx, ty;
    if (readTouchPoint(tx, ty)) {
      splashSkipped = true;
    }
    // Odświeżaj ikonę baterii co 1 sekundę podczas czekania
    unsigned long now = millis();
    if (now - lastBatteryUpdate >= 1000) {
      lastBatteryUpdate = now;
      updateBatteryStatus();
      drawBatteryQuickUpdate(true);
    }
    delay(50);
  }
  // Odświeżaj ikonę baterii co 1 sekundę podczas czekania
  unsigned long now = millis();
  if (now - lastBatteryUpdate >= 1000) {
    lastBatteryUpdate = now;
    updateBatteryStatus();
    drawBatteryQuickUpdate(true);
  }
  delay(50);
  bootSequenceActive = false;
  drawScreen(currentScreen);
  resetTftAutoSwitchTimer();
  if (currentScreen == SCREEN_HAM_CLOCK) {
    ensureScreenOrderValid();

    // ----- ZADANIA FreeRTOS
    if (uiTaskHandle == nullptr) {
      xTaskCreatePinnedToCore(
        uiTaskLoop,
        "UI_Task",
        8192,
        nullptr,
        2,
        &uiTaskHandle,
        1
      );
    }
  }
#endif
}

void loop() {
  static unsigned long loopCounter = 0;
  static unsigned long lastLoopPrint = 0;
  loopCounter++;
  
  unsigned long now = millis();
  
  // Feed watchdog
  yield();

  // Odroczony restart (ÄąÄ˝eby odpowiedÄąĹź HTTP zdĂ„â€¦ÄąÄ˝yÄąâ€ša wyjÄąâ€şĂ„â€ˇ)
  if (restartRequested && (long)(millis() - restartAtMs) >= 0) {
    LOGV_PRINTLN("[LOOP] Restart requested - restarting...");
    delay(50);
    ESP.restart();
  }
  
  // ObsÄąâ€šuga serwera WWW
  if (server != nullptr) {
    server->handleClient();
  }
  
  // Aktualizuj czas NTP
  updateNTPTime();
  
  // ObsÄąâ€šuga WiFi
  if (!wifiConnected && WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.print("STA połączoneâ€¦czone. IP: ");
    Serial.println(WiFi.localIP());
    // Jeżeli wcześniej był uruchomiony AP do konfiguracji, wyłącz go po udanym połączeniu STA,
    // żeby nie robić wrażenia "zwiechy" (klient AP traci link przy przełączeniu kanału).
    if (WiFi.getMode() & WIFI_AP) {
      Serial.println("Wyłączam AP (portal) po połączeniu STA. Użyj IP z sieci domowej.");
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
    }
    

    updateNTPTime();
    connectToCluster();
    connectToPotaCluster();
    connectToAPRS(); // Połącz również z APRS-IS
  } else if (wifiConnected && WiFi.status() != WL_CONNECTED) {
    LOGV_PRINTLN("[LOOP] WiFi status zmieniony: połączony -> rozłączony");
    wifiConnected = false;
    telnetConnected = false;
    potaTelnetConnected = false;
    aprsConnected = false;
    LOGV_PRINTLN("[LOOP] STA rozłączone (wracam do trybu offline/AP jeśli aktywne).");
  }

  updateStatusRgbLed();

  // Obsługa wygaszacza ekranu (Matrix)
  checkScreenSaverTimeout();

  // Obsługa uśpienia ekranu
  checkScreenSleepTimeout();

  // Jeśli jesteśmy offline, zostajemy w AP (portal) â€” stabilnie.
  
  // Obsługa Telnet
  if (wifiConnected) {
    if (!telnetConnected) {
      LOGV_PRINTLN("[LOOP] WiFi OK, próba połączenia z Cluster...");
      connectToCluster();
    } else {
      handleTelnetData();
    }
  }

  // Obsługa POTA Telnet
  if (wifiConnected) {
    // Połączenie z POTA Cluster (Telnet)
    if (!potaTelnetConnected) {
      connectToPotaCluster();
    } else {
      handlePotaTelnetData();
    }
    
    // Tryb HTTP API zamiast Telnetu (źródło: https://api.pota.app/v1/spots)
    if (lastPotaApiFetchMs == 0 || now - lastPotaApiFetchMs > POTA_API_FETCH_INTERVAL_MS) {
      if (fetchPotaApi()) {
        lastPotaApiFetchMs = now;
      } else {
        // nawet przy błędzie aktualizuj czas, by nie spamować API
        lastPotaApiFetchMs = now;
      }
    }

    // HAMALERT przez Telnet: set/json + sh/dx 30
    if (lastHamalertFetchMs == 0 || now - lastHamalertFetchMs > HAMALERT_FETCH_INTERVAL_MS) {
      fetchHamalertTelnet();
      lastHamalertFetchMs = now;
    }
  }

  // ObsÄąâ€šuga APRS-IS
  if (wifiConnected) {
    if (!aprsConnected) {
      LOGV_PRINTLN("[LOOP] WiFi OK, prÄ‚łba poÄąâ€šĂ„â€¦czenia z APRS-IS...");
      connectToAPRS();
    } else {
      handleAPRSData();
      unsigned long nowAprs = millis();

      if (aprsBeaconEnabled && aprsLoginSent && userLatLonValid) {
        if (nextAPRSPositionDueMs == 0) {
          nextAPRSPositionDueMs = nowAprs + APRS_POSITION_FIRST_DELAY_MS;
        } else if ((long)(nowAprs - nextAPRSPositionDueMs) >= 0) {
          sendAprsPosition();
          nextAPRSPositionDueMs = nowAprs + aprsPositionIntervalMs;
        }
      } else {
        nextAPRSPositionDueMs = 0; // Reset harmonogramu gdy beacon jest wyłączony lub brak loginu/koordynat
        aprsBeaconTxCount = 0;
      }
      
      // Watchdog APRS: jeÄąâ€şli brak danych przez dÄąâ€šuÄąÄ˝szy czas, zrÄ‚łb reconnect
      unsigned long inactivityTime = nowAprs - lastAPRSRxMs;
      if (inactivityTime > APRS_INACTIVITY_RECONNECT_MS) {
        LOGV_PRINT("[LOOP] WARNING: Brak danych z APRS-IS przez ");
        LOGV_PRINT(inactivityTime / 1000);
        LOGV_PRINTLN(" sekund -> reconnect");
        aprsClient.stop();
        aprsConnected = false;
        aprsLoginSent = false;
        // connectToAPRS() odpali siĂ„â„˘ w kolejnych iteracjach
      }
    }
  }

  // Aktualizuj dane propagacyjne (hamqsl solarxml)
  if (wifiConnected) {
    unsigned long now = millis();
    unsigned long interval = lastPropagationFetchOk ? PROPAGATION_FETCH_INTERVAL_MS
                                                   : PROPAGATION_FETCH_RETRY_MS;
    if (lastPropagationFetchMs == 0 || now - lastPropagationFetchMs > interval) {
      lastPropagationFetchOk = fetchPropagationData();
      lastPropagationFetchMs = now;

    }
  }

  // Obsługa kolejki QRZ/Callook (asynchronicznie)
  if (wifiConnected && qrzQueueLen > 0) {
    unsigned long now = millis();
    unsigned long qrzInterval = getQrzLookupIntervalMs();
    if (now - lastQrzLookupMs >= qrzInterval) {
      for (int i = 0; i < qrzQueueLen; i++) {
        if (now < qrzQueue[i].nextTryMs) {
          continue;
        }
        String grid;
        String country;
        String name;
        String email;
        String qth;
        double lat = 0.0;
        double lon = 0.0;
        bool hasLatLon = false;
        bool ok = fetchCallookCallsignInfo(qrzQueue[i].callsign, grid, country, name, email, qth, lat, lon, hasLatLon);
        lastQrzLookupMs = now;
        if (ok) {
          updateSpotsWithQrz(qrzQueue[i].callsign, grid, country, name, lat, lon, hasLatLon);
          removeQrzQueueAt(i);
        } else {
          qrzQueue[i].attempts++;
          if (qrzQueue[i].attempts >= QRZ_RETRY_LIMIT) {
            removeQrzQueueAt(i);
          } else {
            qrzQueue[i].nextTryMs = now + QRZ_RETRY_DELAY_MS;
          }
        }
        break; // jedna prÄ‚łba na iteracjĂ„â„˘
      }
    }
  }

  // Aktualizuj pogodĂ„â„˘ (OpenWeather)
  if (wifiConnected) {
    unsigned long now = millis();
    unsigned long interval = lastWeatherFetchOk ? WEATHER_FETCH_INTERVAL_MS
                                               : WEATHER_FETCH_RETRY_MS;
    if (lastWeatherFetchMs == 0 || now - lastWeatherFetchMs > interval) {
      lastWeatherFetchOk = fetchWeatherData();
      lastWeatherFetchMs = now;

    }
  }

  // Przetwarzaj maks. 1 liniĂ„â„˘ telnet na iteracjĂ„â„˘ (ÄąÄ˝eby nie zamroziĂ„â€ˇ WWW/UI)
  if (pendingTelnetLine.length() > 0) {
    LOGV_PRINT("[LOOP] Przetwarzanie linii telnet, len=");
    LOGV_PRINTLN(pendingTelnetLine.length());
    LOGV_PRINT("[LOOP] Linia: ");
    if (pendingTelnetLine.length() > 80) {
      LOGV_PRINTLN(pendingTelnetLine.substring(0, 80) + "...");
    } else {
      LOGV_PRINTLN(pendingTelnetLine);
    }
    
    String line = pendingTelnetLine;
    pendingTelnetLine = ""; // WyczyÄąâ€şĂ„â€ˇ PRZED parsowaniem (ÄąÄ˝eby nie gromadziĂ„â€ˇ)
    yield(); // Feed watchdog przed dÄąâ€šugĂ„â€¦ operacjĂ„â€¦
    
    DXSpot spot;
    unsigned long parseStart = millis();
    if (parseDXSpot(line, spot)) {
      unsigned long parseTime = millis() - parseStart;
      if (parseTime > 50) {
        LOGV_PRINT("[LOOP] WARNING: parseDXSpot zajĂ„â„˘Äąâ€šo ");
        LOGV_PRINT(parseTime);
        LOGV_PRINTLN("ms");
      }
      addSpot(spot);
      // Aktualizuj wyÄąâ€şwietlacz TFT z nowymi spotami (tylko jeÄąâ€şli jesteÄąâ€şmy na ekranie 2)

    }
  }

  if (pendingPotaLine.length() > 0) {
    String line = pendingPotaLine;
    pendingPotaLine = "";
    
    Serial.print("[POTA DEBUG] Otrzymano linie: ");
    Serial.println(line);

    DXSpot spot;
    if (parsePotaSpot(line, spot)) {
      Serial.print("[POTA DEBUG] Parsowanie OK, mode=");
      Serial.print(spot.mode);
      Serial.print(", callsign=");
      Serial.println(spot.callsign);
      
      if (spot.mode == "SSB") {
        applyQrzCacheToSpot(spot, QRZ_CACHE_TTL_MS);
        addPotaSpot(spot);
        Serial.println("[POTA DEBUG] Spot dodany!");
        
        bool qrzConfigured = (qrzUsername.length() > 0 && qrzPassword.length() > 0);
        if (qrzConfigured && spot.country.length() == 0 && spot.callsign.length() > 0) {
          String call = spot.callsign;
          call.toUpperCase();
          enqueueQrzLookup(call);
        }
      } else {
        Serial.print("[POTA DEBUG] Odrzucono - nie SSB: ");
        Serial.println(spot.mode);
      }
    } else {
      Serial.println("[POTA DEBUG] Parsowanie nieudane");
    }
  }

  // Watchdog telnet: jeÄąâ€şli brak danych z clustra przez dÄąâ€šuÄąÄ˝szy czas, zrÄ‚łb reconnect
  if (telnetConnected && telnetClient.connected()) {
    unsigned long now = millis();
    unsigned long inactivityTime = now - lastTelnetRxMs;
    if (inactivityTime > TELNET_INACTIVITY_RECONNECT_MS) {
      LOGV_PRINT("[LOOP] WARNING: Brak danych z DX Cluster przez ");
      LOGV_PRINT(inactivityTime / 1000);
      LOGV_PRINTLN(" sekund -> reconnect");
      telnetClient.stop();
      telnetConnected = false;
      clusterLoginSent = false;
      clusterLoginScheduled = false;
      // connectToCluster() odpali siĂ„â„˘ w kolejnych iteracjach
    } else if (inactivityTime > 320000) { // OstrzeÄąÄ˝enie po 4 minutach
      LOGV_PRINT("[LOOP] WARNING: Brak danych z Cluster przez ");
      LOGV_PRINT(inactivityTime / 1000);
      LOGV_PRINTLN(" sekund (blisko timeout)");
    }
  }

  if (potaTelnetConnected && potaTelnetClient.connected()) {
    unsigned long now = millis();
    unsigned long inactivityTime = now - lastPotaRxMs;
    if (inactivityTime > POTA_TELNET_INACTIVITY_RECONNECT_MS) {
      LOGV_PRINT("[LOOP] WARNING: Brak danych z POTA Cluster przez ");
      LOGV_PRINT(inactivityTime / 1000);
      LOGV_PRINTLN(" sekund -> reconnect");
      potaTelnetClient.stop();
      potaTelnetConnected = false;
      potaLoginSent = false;
      potaLoginScheduled = false;
    }
  }

  // WyÄąâ€şlij znak (login) jeÄąâ€şli zaplanowany
  if (telnetConnected && telnetClient.connected() && clusterLoginScheduled) {
    if ((long)(millis() - clusterSendLoginAtMs) >= 0) {
      LOGV_PRINTLN("[LOOP] WysyÄąâ€šanie loginu do Cluster...");
      clusterLoginScheduled = false;
      if (!clusterLoginSent) {
        String login = userCallsign;
        login.trim();
        if (login.length() == 0) {
          login = DEFAULT_CALLSIGN;
        }
        
        // JeÄąâ€şli mamy lokator, dodaj go do loginu w formacie: callsign/locator
        // Format zgodny z CC-Cluster (dxspots.com) i wiĂ„â„˘kszoÄąâ€şciĂ„â€¦ DX ClusterÄ‚łw
        if (userLocator.length() >= 4) {
          login += "/";
          login += userLocator;
          Serial.print("[CLUSTER] Login -> ");
          Serial.print(login);
          Serial.print(" (callsign/locator)");
          if (userCallsign.length() == 0) {
            Serial.print(" - domyÄąâ€şlny znak, ustaw swÄ‚łj w Config");
          }
          Serial.println();
        } else {
          Serial.print("[CLUSTER] Login -> ");
          Serial.print(login);
          if (userCallsign.length() == 0) {
            Serial.print(" (domyÄąâ€şlny, ustaw swÄ‚łj znak w Config)");
          } else {
            Serial.print(" (bez lokatora - ustaw w Config dla lepszej funkcjonalnoÄąâ€şci)");
          }
          Serial.println();
        }
        
        telnetClient.print(login);
        telnetClient.print("\r\n");
        clusterLoginSent = true;
        lastClusterKeepAliveMs = millis();
        Serial.println("[CLUSTER] Login wysÄąâ€šany");
        
        // Po zalogowaniu wyÄąâ€şlij komendy konfiguracyjne CC-Cluster (z opÄ‚łÄąĹźnieniem)
        // Daj czas clusterowi na przetworzenie loginu
        delay(500);
        sendClusterConfigCommands();
      }
    }
  }

  if (potaTelnetConnected && potaTelnetClient.connected() && potaLoginScheduled) {
    if ((long)(millis() - potaSendLoginAtMs) >= 0) {
      potaLoginScheduled = false;
      if (!potaLoginSent) {
        String login = userCallsign;
        login.trim();
        if (login.length() == 0) {
          login = DEFAULT_CALLSIGN;
        }
        if (userLocator.length() >= 4) {
          login += "/";
          login += userLocator;
        }
        potaTelnetClient.print(login);
        potaTelnetClient.print("\r\n");
        potaLoginSent = true;
        lastPotaKeepAliveMs = millis();
      }
    }
  }

  // Keepalive: niektÄ‚łre klastry zrywajĂ„â€¦ idle telnet, wiĂ„â„˘c co ~30s wysyÄąâ€šamy CRLF
  if (telnetConnected && telnetClient.connected()) {
    unsigned long now = millis();
    if (now - lastClusterKeepAliveMs > 30000) {
      LOGV_PRINTLN("[LOOP] WysyÄąâ€šanie keepalive do Cluster");
      telnetClient.print("\r\n");
      lastClusterKeepAliveMs = now;
    }
  }

  if (potaTelnetConnected && potaTelnetClient.connected()) {
    unsigned long now = millis();
    if (now - lastPotaKeepAliveMs > 30000) {
      potaTelnetClient.print("\r\n");
      lastPotaKeepAliveMs = now;
    }
  }
  
  // Obsługa MQTT PSK Reporter (w tle, nawet gdy nie na ekranie PSK)
  if (pskMqttEnabled) {
    loopPskMqtt();
  }
  
  // Aktualizacja pomiaru baterii (TP4056 + 18650)
#ifdef BATTERY_MONITORING_ENABLED
  if (updateBatteryStatus()) {
    // Debug co 30 sekund - stan baterii
    Serial.printf("[BATTERY] %.2fV (%d%%) %s\n", 
                  batteryVoltage, batteryPercentage,
                  batteryCharging ? "[CHARGING]" : "");
    
    // Szybka aktualizacja ikony baterii (bez odświeżania całego nagłówka)
    // drawBatteryQuickUpdate() jest lekka i nie powoduje watchdog resetu
    drawBatteryQuickUpdate();
    
    // Ostrzeżenie przy niskim poziomie baterii (< 10%)
    if (batteryPercentage < 10 && !batteryLowWarningShown) {
      Serial.println("[BATTERY] UWAGA: Niski poziom baterii! Podłącz ładowarkę.");
      batteryLowWarningShown = true;
    }
    // Reset flagi ostrzeżenia gdy bateria się naładuje
    if (batteryPercentage > 20 && batteryLowWarningShown) {
      batteryLowWarningShown = false;
    }
  }
#endif

  // Feed watchdog przed delay
  yield();
  delay(10); // Małe opóźnienie dla stabilności
  
}
