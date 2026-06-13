// tft_setup.h - Konfiguracja wyświetlacza TFT ILI9488 dla ESP32-HAM-CLOCK
// Autor: Konrad Wiśniewski SP3KON

#ifndef TFT_SETUP_H
#define TFT_SETUP_H

// ============================================
// Konfiguracja sprzętowa ESP32
// ============================================
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1  // Nie używany - podłączony do 3.3V przez resistor
#define TFT_BL_PIN 32

// ============================================
// Konfiguracja wyświetlacza
// ============================================
#define TFT_WIDTH   480
#define TFT_HEIGHT  320
#define SPI_FREQUENCY 20000000

// ============================================
// Konfiguracja dotyku XPT2046
// ============================================
#define TOUCH_CS   21
#define TOUCH_IRQ  22
#define SPI_TOUCH_FREQUENCY 2500000

// ============================================
// Sterownik wyświetlacza
// ============================================
#define ILI9488_DRIVER

// ============================================
// Opcje fontów
// ============================================
#define LOAD_GLCD_FONT
#define LOAD_FONT2
#define SMOOTH_FONT

// ============================================
// Debug
// ============================================
#define CORE_DEBUG_LEVEL 0

#endif // TFT_SETUP_H