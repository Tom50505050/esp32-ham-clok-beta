#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

static const int POSSIBLE_BL_PIN = 32;

static void enablePossibleBacklight() {
  pinMode(POSSIBLE_BL_PIN, OUTPUT);
  digitalWrite(POSSIBLE_BL_PIN, HIGH);
}

static void showColor(uint16_t color, const char *label) {
  tft.fillScreen(color);
  tft.setTextColor(TFT_WHITE, color);
  tft.setTextSize(2);
  tft.setCursor(20, 20);
  tft.println("ILI9488 TEST");
  tft.setCursor(20, 60);
  tft.println(label);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("ILI9488 diagnostic start");
  Serial.println("Pins: MISO=19 MOSI=23 SCLK=18 CS=15 DC=2 RST=-1 BL=32");

  enablePossibleBacklight();
  delay(50);

  tft.begin();
  tft.setRotation(1);
  tft.invertDisplay(true);

  Serial.println("tft.begin done");

  showColor(TFT_RED, "RED");
  delay(1500);
  showColor(TFT_GREEN, "GREEN");
  delay(1500);
  showColor(TFT_BLUE, "BLUE");
  delay(1500);
  showColor(TFT_WHITE, "WHITE");

  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setCursor(20, 100);
  tft.println("If you see this,");
  tft.setCursor(20, 130);
  tft.println("display wiring is OK.");

  Serial.println("diagnostic pattern finished");
}

void loop() {
  static uint32_t lastMs = 0;
  static bool on = false;

  if (millis() - lastMs > 1000) {
    lastMs = millis();
    on = !on;
    tft.fillCircle(430, 20, 10, on ? TFT_YELLOW : TFT_BLACK);
    Serial.println(on ? "blink on" : "blink off");
  }
}
