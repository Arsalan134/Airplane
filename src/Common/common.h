#include "OLEDDisplayUi.h"
#include "SSD1306Wire.h"

// PS5
#define MAC_ADDRESS "ac:36:1b:41:ac:ed"

extern OLEDDisplayUi display;

// Overlays are statically drawn on top of a frame eg. a clock
// OverlayCallback overlays[] = {msOverlay};
extern OverlayCallback all[];
extern OverlayCallback wifiOverlays[];
extern OverlayCallback bluetoothOverlays[];

extern String recievedMessage;

extern int engineRecieved;
extern int aileronRecieved;
extern int rudderRecieved;
extern int elevatorsRecieved;

#define ENGINE_PIN 4
#define ROLL_LEFT_MOTOR_PIN 15
#define ELEVATION_LEFT_MOTOR_PIN 13
#define ELEVATION_RIGHT_MOTOR_PIN 2
#define RUDDER_MOTOR_PIN 12