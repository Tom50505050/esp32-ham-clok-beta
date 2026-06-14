#pragma once

// Wspólna konfiguracja TFT_eSPI dla szkicu i samej biblioteki.
// Plik jest automatycznie wykrywany przez TFT_eSPI podczas kompilacji.

#ifndef TFT_PROFILE_ILI9488
#define TFT_PROFILE_ILI9488 0
#endif

#ifndef TFT_PROFILE_ILI9341
#define TFT_PROFILE_ILI9341 1
#endif

// Ten build jest przygotowany wyłącznie pod zewnętrzny moduł ESP32 + ILI9488 480x320.
#ifndef TFT_DRIVER_PROFILE
#define TFT_DRIVER_PROFILE TFT_PROFILE_ILI9488
#endif

#ifndef USER_SETUP_INFO
#define USER_SETUP_INFO "ESP32_HAM_CLOCK_ILI9488_SETUP"
#endif

#ifndef ILI9488_DRIVER
  #define ILI9488_DRIVER
#endif
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
  #define TFT_CS 15
#endif
#ifndef TFT_DC
  #define TFT_DC 2
#endif
#ifndef TFT_RST
  #define TFT_RST -1
#endif
#ifndef TFT_BL
  #define TFT_BL 32
#endif
#ifndef TFT_WIDTH
  #define TFT_WIDTH 480
#endif
#ifndef TFT_HEIGHT
  #define TFT_HEIGHT 320
#endif

#ifndef TFT_BL_INVERTED
#define TFT_BL_INVERTED 0
#endif

#ifndef TFT_RGB_ORDER
#define TFT_RGB_ORDER TFT_BGR
#endif

#ifndef SPI_FREQUENCY
#define SPI_FREQUENCY 20000000
#endif

#ifndef SPI_READ_FREQUENCY
#define SPI_READ_FREQUENCY 20000000
#endif

#ifndef SPI_TOUCH_FREQUENCY
#define SPI_TOUCH_FREQUENCY 2500000
#endif

#ifndef LOAD_GLCD
#define LOAD_GLCD
#endif

#ifndef LOAD_FONT2
#define LOAD_FONT2
#endif

#ifndef LOAD_FONT4
#define LOAD_FONT4
#endif

#ifndef SMOOTH_FONT
#define SMOOTH_FONT
#endif
