// ============================================================
// User_Setup.h — TFT_eSPI configuration for this project
//
// This file is NOT compiled as part of the sketch. Copy its contents
// into <Arduino/libraries>/TFT_eSPI/User_Setup.h (replacing that file's
// contents, or #include this file from User_Setup_Select.h) before
// building. TFT_eSPI reads its pin/driver config at library-compile
// time, not from the sketch folder.
// ============================================================

#define ILI9341_DRIVER

#define TFT_MISO  19
#define TFT_MOSI  23
#define TFT_SCLK  18
#define TFT_CS    33
#define TFT_DC    21
#define TFT_RST   22

#define TOUCH_CS  13

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT

#define SPI_FREQUENCY        40000000
#define SPI_READ_FREQUENCY   20000000
#define SPI_TOUCH_FREQUENCY   2500000
