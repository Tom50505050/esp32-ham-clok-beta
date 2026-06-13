// tft_setup.h - Konfiguracja wyświetlacza TFT ILI9488 dla ESP32-HAM-CLOCK
// Autor: Konrad Wiśniewski SP3KON

#ifndef TFT_SETUP_H
#define TFT_SETUP_H

// ============================================
// Konfiguracja sprzętowa ESP32
// ============================================
#ifndef TFT_MISO
#define TFT_MISO 19
#endif
#ifndef TFT_MOSI
#define TFT_MOSI 23
#endif
#ifndef TFT_SCLK
#define TFT_SCLK 18
#endif
#ifndef TFT_CS
#define TFT_CS   15
#endif
#ifndef TFT_DC
#define TFT_DC    2
#endif
#ifndef TFT_RST
#define TFT_RST  -1  // Nie używany - podłączony do 3.3V przez resistor
#endif
#ifndef TFT_BL_PIN
#define TFT_BL_PIN 32
#endif
#ifndef TFT_BL
#define TFT_BL TFT_BL_PIN
#endif

// ============================================
// Konfiguracja wyświetlacza
// ============================================
#ifndef TFT_WIDTH
#define TFT_WIDTH   480
#endif
#ifndef TFT_HEIGHT
#define TFT_HEIGHT  320
#endif
#ifndef SPI_FREQUENCY
#define SPI_FREQUENCY 20000000
#endif

// ============================================
// Konfiguracja dotyku XPT2046
// ============================================
#ifndef TOUCH_CS
#define TOUCH_CS   21
#endif
#ifndef TOUCH_IRQ
#define TOUCH_IRQ  22
#endif
#ifndef SPI_TOUCH_FREQUENCY
#define SPI_TOUCH_FREQUENCY 2500000
#endif

// ============================================
// Sterownik wyświetlacza
// ============================================
#ifndef ILI9488_DRIVER
#define ILI9488_DRIVER
#endif

// ============================================
// Opcje fontów
// ============================================
#ifndef LOAD_GLCD_FONT
#define LOAD_GLCD_FONT
#endif
#ifndef LOAD_FONT2
#define LOAD_FONT2
#endif
#ifndef SMOOTH_FONT
#define SMOOTH_FONT
#endif

// ============================================
// Definicje wymagane przez main.cpp
// ============================================
#ifndef TFT_BL_INVERTED
#define TFT_BL_INVERTED false
#endif

#ifndef USER_SETUP_LOADED
#define USER_SETUP_LOADED
#endif

// USER_SETUP_INFO - opis konfiguracji dla debugowania
#ifndef USER_SETUP_INFO
#define USER_SETUP_INFO "ILI9488_480x320_Parallel"
#endif

// ============================================
// Debug
// ============================================
#ifndef CORE_DEBUG_LEVEL
#define CORE_DEBUG_LEVEL 0
#endif

#endif // TFT_SETUP_H