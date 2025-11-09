#include "OLEDDisplayUi.h"  // 📱 OLED display user interface library
#include "SSD1306Wire.h"    // 🖥️ SSD1306 OLED display driver for I2C

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

// 🎮 PS5 Controller MAC Address for Bluetooth pairing
#define MAC_ADDRESS "ac:36:1b:41:ac:ed"

extern OLEDDisplayUi display;  // 📺 Main display controller instance

extern OverlayCallback allOverlays[];        // 🎨 General overlay functions array
extern OverlayCallback wifiOverlays[];       // 📶 WiFi-specific overlay functions
extern OverlayCallback bluetoothOverlays[];  // 📱 Bluetooth overlay functions

extern String recievedMessage;  // 📩 Last received message from ground station

extern int engineReceived;              // 🚁 Engine throttle value from controller
extern int aileronReceived;             // ↔️ Aileron control value (roll)
extern int rudderReceived;              // ↕️ Rudder control value (yaw)
extern int elevatorsReceived;           // ⬆️⬇️ Elevator control value (pitch)
extern int elevatorTrimReceived;        // ⚖️ Elevator trim adjustment value
extern int elevatorTrimToDisplay;       // 📊 Elevator trim value for display
extern int aileronTrimReceived;         // ⚖️ Aileron trim adjustment value
extern int aileronTrimToDisplay;        // 📊 Aileron trim value for display
extern int flapsReceived;               // 🛬 Flaps position value
extern int flapsToDisplay;              // 📊 Flaps position for display
extern bool shouldResetAileronTrim;     // 🔄 Flag to reset aileron trim
extern bool shouldResetElevatorTrim;    // 🔄 Flag to reset elevator trim
extern bool airBrakeReceived;           // 🛑 Air brake activation status
extern unsigned long lastReceivedTime;  // ⏰ Timestamp of last received data

extern int RSSIToDisplay;         // 📶 Signal strength indicator for display
extern int elapsedTimeToDisplay;  // ⏱️ Flight time counter for display

// 🧭 IMU (Inertial Measurement Unit) status for display
extern bool imuStatusDisplay;
