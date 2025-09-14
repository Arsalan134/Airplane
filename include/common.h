#include "OLEDDisplayUi.h"
#include "SSD1306Wire.h"

/*
🛩️ ESP32-PICO-D4 Airplane Control System
    Chip is ESP32-PICO-D4 (revision v1.1)
    Features:
        📶 WiFi,
        📱 BT,
        ⚡ Dual Core 240MHz,
        💾 Embedded Flash,
        🔧 VRef calibration in efuse,
    🔮 Crystal is 40MHz
    🏷️ MAC: 90:15:06:f6:17:e0
*/

// 🎮 PS5 Controller
#define MAC_ADDRESS "ac:36:1b:41:ac:ed"

extern OLEDDisplayUi display;

// Overlays are statically drawn on top of a frame eg. a clock
// OverlayCallback overlays[] = {msOverlay};
extern OverlayCallback allOverlays[];
extern OverlayCallback wifiOverlays[];
extern OverlayCallback bluetoothOverlays[];

extern String recievedMessage;

extern int engineReceived;
extern int aileronReceived;
extern int rudderReceived;

extern int elevatorsReceived;

extern int elevatorTrimReceived;
extern int elevatorTrimToDisplay;

extern int aileronTrimReceived;
extern int aileronTrimToDisplay;

extern int flapsRecieved;
extern int flapsToDisplay;

extern bool resetAileronTrim;
extern bool resetElevatorTrim;

extern bool airBrakeReceived;

extern int RSSIToDisplay;
extern int elapsedTimeToDisplay;

extern unsigned long lastReceivedTime;

// IMU status for display
extern bool imuStatusDisplay;
