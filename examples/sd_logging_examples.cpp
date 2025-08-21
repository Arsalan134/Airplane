/**
 * @file sd_logging_examples.cpp
 * @brief Examples of how to use the advanced SD Card logging system in RC Airplane
 * @author RC Airplane Project
 * @date 2025
 *
 * This file contains practical examples of how to integrate the SD card logging
 * system into your airplane firmware for comprehensive flight data recording.
 */

#include "SD-Card.h"

// Example: Initialize SD card logging in setup()
void setup_sd_logging_example() {
  Serial.begin(115200);
  Serial.println("🚀 RC Airplane SD Logging Examples");

  // Initialize SD card
  if (sdLogger.begin()) {
    Serial.println("✅ SD Card logging system ready");

    // Start a new flight session
    if (sdLogger.startNewSession()) {
      Serial.println("📝 Flight session started: " + sdLogger.getSessionId());
    }
  } else {
    Serial.println("❌ SD Card initialization failed!");
    // Handle error - maybe use internal storage or disable logging
  }
}

// Example: Log flight data during main loop
void log_flight_data_example() {
  // Simulate sensor readings
  FlightData flight_data;
  flight_data.timestamp = millis();
  flight_data.altitude = 150.5;        // meters
  flight_data.speed = 25.3;            // m/s
  flight_data.heading = 87.2;          // degrees
  flight_data.roll = -5.1;             // degrees
  flight_data.pitch = 2.8;             // degrees
  flight_data.yaw = 87.2;              // degrees
  flight_data.throttle = 75;           // percentage
  flight_data.battery_voltage = 11.2;  // volts
  flight_data.temperature = 22.5;      // celsius
  flight_data.pressure = 1013.25;      // hPa
  flight_data.gps_lat = 40.712776;     // latitude
  flight_data.gps_lon = -74.005974;    // longitude
  flight_data.gps_alt = 155.2;         // GPS altitude

  // Log the data (buffered for efficiency)
  sdLogger.logFlightData(flight_data);

  // Alternative: Log directly to CSV for immediate writing
  // sdLogger.logFlightDataCSV(flight_data);

  // Alternative: Log as JSON for more structured data
  // sdLogger.logFlightDataJSON(flight_data);
}

// Example: Log telemetry data for communication analysis
void log_telemetry_example() {
  TelemetryData telemetry;
  telemetry.timestamp = millis();
  telemetry.rssi = -85;         // dBm
  telemetry.snr = 7.5;          // dB
  telemetry.packet_loss = 2.1;  // percentage
  telemetry.command_type = "ALTITUDE_CMD";
  telemetry.ack_received = true;
  telemetry.response_time = 45;  // milliseconds

  // Log telemetry data
  sdLogger.logTelemetryData(telemetry);
}

// Example: Event logging for system monitoring
void log_system_events_example() {
  // Log various system events
  sdLogger.logInfo("GPS", "GPS lock acquired", "Satellites: 8, HDOP: 1.2");
  sdLogger.logWarning("BATTERY", "Battery voltage low", "Voltage: 10.8V");
  sdLogger.logError("SENSORS", "Barometer read failed", "I2C timeout");
  sdLogger.logCritical("SAFETY", "Emergency landing triggered", "Lost signal for 10s");

  // Custom event logging with details
  String details = "Temperature: 85°C, Throttle: " + String(95) + "%";
  sdLogger.logWarning("MOTOR", "Motor overheating detected", details);
}

// Example: Pre-flight system check with logging
void preflight_check_example() {
  sdLogger.logInfo("PREFLIGHT", "Starting pre-flight checks");

  // Check GPS
  bool gps_ok = true;  // Your GPS check logic
  if (gps_ok) {
    sdLogger.logInfo("GPS", "GPS system OK", "Signal strength: Good");
  } else {
    sdLogger.logError("GPS", "GPS not ready", "No satellite lock");
  }

  // Check LoRa
  bool lora_ok = true;  // Your LoRa check logic
  if (lora_ok) {
    sdLogger.logInfo("LORA", "LoRa communication OK", "RSSI: -65dBm");
  } else {
    sdLogger.logError("LORA", "LoRa communication failed", "No response from ground");
  }

  // Check battery
  float battery_voltage = 12.1;  // Your battery reading
  if (battery_voltage > 11.0) {
    sdLogger.logInfo("BATTERY", "Battery level OK", "Voltage: " + String(battery_voltage) + "V");
  } else {
    sdLogger.logWarning("BATTERY", "Battery voltage low", "Voltage: " + String(battery_voltage) + "V");
  }

  sdLogger.logInfo("PREFLIGHT", "Pre-flight checks completed");
}

// Example: Emergency logging
void emergency_logging_example() {
  // In case of emergency, immediately flush all buffers
  sdLogger.logCritical("EMERGENCY", "Emergency situation detected", "Immediate data preservation");

  // Force flush all buffers to ensure data is saved
  sdLogger.flushAllBuffers();

  // End session to generate summary
  sdLogger.endSession();
}

// Example: Post-flight analysis preparation
void post_flight_analysis_example() {
  Serial.println("📊 Preparing flight data for analysis...");

  // End the current session
  sdLogger.endSession();

  // List all log files
  sdLogger.listLogFiles();

  // Print storage statistics
  sdLogger.printStorageInfo();

  Serial.println("✅ Flight data ready for analysis");
  Serial.println("📈 CSV files can be imported into Excel, Python, or MATLAB");
  Serial.println("📄 JSON files can be used for detailed analysis or web dashboards");
}

// Example: Periodic maintenance tasks
void maintenance_tasks_example() {
  // Run this periodically (e.g., once per day)

  // Delete logs older than 30 days
  sdLogger.deleteOldLogs(30);

  // Force flush buffers every 5 minutes
  static unsigned long last_flush = 0;
  if (millis() - last_flush > 300000) {  // 5 minutes
    sdLogger.flushAllBuffers();
    last_flush = millis();
    sdLogger.logInfo("MAINTENANCE", "Periodic buffer flush completed");
  }
}

// Example: Complete flight session workflow
void complete_flight_session_example() {
  // 1. Initialize system
  if (!sdLogger.begin()) {
    Serial.println("❌ Cannot proceed without SD card");
    return;
  }

  // 2. Start new flight session
  sdLogger.startNewSession();

  // 3. Pre-flight checks
  preflight_check_example();

  // 4. Flight loop (would be in main loop)
  for (int i = 0; i < 100; i++) {
    log_flight_data_example();
    log_telemetry_example();
    delay(100);  // 10Hz logging
  }

  // 5. Post-flight
  post_flight_analysis_example();

  Serial.println("🎯 Complete flight session logged successfully!");
}

// Example: Data export for plotting (Python/MATLAB compatible)
void export_for_plotting_example() {
  Serial.println("📊 Exporting data for plotting applications...");

  // The CSV files generated are directly compatible with:

  Serial.println("🐍 Python (pandas, matplotlib):");
  Serial.println("   import pandas as pd");
  Serial.println("   import matplotlib.pyplot as plt");
  Serial.println("   data = pd.read_csv('/path/to/flight_data.csv')");
  Serial.println("   plt.plot(data['timestamp'], data['altitude'])");
  Serial.println("   plt.xlabel('Time (ms)')");
  Serial.println("   plt.ylabel('Altitude (m)')");
  Serial.println("   plt.title('Flight Altitude Profile')");
  Serial.println("   plt.show()");

  Serial.println("\n🔢 MATLAB:");
  Serial.println("   data = readtable('flight_data.csv');");
  Serial.println("   plot(data.timestamp, data.altitude);");
  Serial.println("   xlabel('Time (ms)');");
  Serial.println("   ylabel('Altitude (m)');");
  Serial.println("   title('Flight Altitude Profile');");

  Serial.println("\n📊 Excel:");
  Serial.println("   - Open CSV file directly in Excel");
  Serial.println("   - Create charts from timestamp vs any parameter");
  Serial.println("   - Use pivot tables for data analysis");
}
