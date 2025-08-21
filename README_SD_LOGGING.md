# Advanced SD Card Logging System

This advanced SD card logging system provides comprehensive data recording and analysis capabilities for both the RC Airplane and Ground LoRa Station projects.

## 🚀 Features

### Core Features
- **High-Performance Logging**: Buffered writes for optimal performance
- **Multiple Data Formats**: CSV and JSON output formats
- **Session Management**: Organized data with unique session IDs
- **Real-time Event Logging**: System events with multiple severity levels
- **Automatic File Rotation**: Prevents files from becoming too large
- **Storage Management**: Automatic cleanup of old logs
- **Error Recovery**: Robust error handling and data preservation

### Data Types Logged

#### RC Airplane
- **Flight Data**: Altitude, speed, heading, attitude (roll/pitch/yaw), throttle, GPS coordinates
- **Telemetry**: RSSI, SNR, packet loss, command acknowledgments
- **System Events**: GPS status, sensor readings, battery levels, safety events

#### Ground LoRa Station  
- **Controller Data**: PS5 stick positions, button presses, trigger values, battery level
- **Telemetry**: Signal quality, response times, packet loss
- **Command Data**: Commands sent, acknowledgments, execution times
- **System Events**: Controller connection, display status, communication errors

## 📋 Requirements

### Hardware
- ESP32 development board (TTGO LoRa32 v2.1)
- MicroSD card (Class 10 recommended, 8GB or larger)
- SD card module (if not integrated)

### Software Dependencies
```ini
lib_deps = 
    arduino-libraries/SD@^1.2.4
    bblanchon/ArduinoJson@^7.0.4
```

## 🔧 Hardware Setup

### SD Card Connections
```cpp
#define SD_CS_PIN 5
#define SD_MOSI_PIN 23
#define SD_MISO_PIN 19  
#define SD_SCK_PIN 18
```

Connect your SD card module to these pins on the ESP32.

## 💻 Software Integration

### Basic Setup

```cpp
#include "SD-Card.h"

void setup() {
    Serial.begin(115200);
    
    // Initialize SD card logging
    if (sdLogger.begin()) {
        Serial.println("✅ SD Card logging ready");
        
        // Start a new session
        sdLogger.startNewSession();
    } else {
        Serial.println("❌ SD Card initialization failed");
    }
}

void loop() {
    // Your main code here
    
    // Log flight data (airplane)
    FlightData data;
    data.timestamp = millis();
    data.altitude = 150.5;
    data.speed = 25.3;
    // ... populate other fields
    sdLogger.logFlightData(data);
    
    // Log events
    sdLogger.logInfo("GPS", "Position updated", "Lat: 40.123, Lon: -74.456");
    
    // Periodic maintenance
    static unsigned long lastFlush = 0;
    if (millis() - lastFlush > 10000) { // Every 10 seconds
        sdLogger.flushAllBuffers();
        lastFlush = millis();
    }
}

void cleanup() {
    // End session and cleanup
    sdLogger.endSession();
    sdLogger.end();
}
```

For complete examples and usage patterns, see the files in the `examples/` directory of each project.
