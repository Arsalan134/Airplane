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
extern unsigned long lastRecievedTime;

#define ENGINE_PIN 4
#define ROLL_LEFT_MOTOR_PIN 12
#define ROLL_RIGHT_MOTOR_PIN -1  // -1 means not used

#define ELEVATION_LEFT_MOTOR_PIN 13
#define ELEVATION_RIGHT_MOTOR_PIN 2
#define RUDDER_MOTOR_PIN 15

#define timeoutInMilliSeconds 1000  // 1 second

#define rudderHalfAngleFreedom 30  // 30 degrees to the left and right