#include "OLEDDisplayUi.h"
#include "SSD1306Wire.h"
#include "images.h"

// #define OLED_RST NOT_A_PIN

void msOverlay(OLEDDisplay* display, OLEDDisplayUiState* state);
void bluetoothOverlay(OLEDDisplay* display, OLEDDisplayUiState* state);
void wifiOverlay(OLEDDisplay* display, OLEDDisplayUiState* state);

// draw an xbm image.
// Please note that everything that should be transitioned
// needs to be drawn relative to x and y
void drawFrame1(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y);

// Demonstrates the 3 included default sizes. The fonts come from SSD1306Fonts.h file
// Besides the default fonts there will be a program to convert TrueType fonts into this format
void drawFrame2(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y);

void drawFrame3(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y);

// Demo for drawStringMaxWidth:
// with the third parameter you can define the width after which words will be wrapped.
// Currently only spaces and "-" are allowed for wrapping
void drawFrame4(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y);

void drawFrame5(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y);
