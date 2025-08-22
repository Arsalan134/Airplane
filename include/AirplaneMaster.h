#ifndef AIRPLANE_MASTER_H
#define AIRPLANE_MASTER_H

#include <Arduino.h>
#include <Wire.h>
#include "FlightProtocol.h"

#define CONNECTION_TIMEOUT 2000  // ⏱️

enum class FlightMode { MANUAL = 0, STABILITY = 1, ACROBATIC = 2, LANDING = 3 };  // 🛩️

class AirplaneMaster {
 private:
  // 🔒 Singleton instance
  static AirplaneMaster* instance;

  // Private constructor for singleton
  AirplaneMaster();

  // Delete copy constructor and assignment operator
  AirplaneMaster(const AirplaneMaster&) = delete;
  AirplaneMaster& operator=(const AirplaneMaster&) = delete;

  // 🛩️ Flight control targets
  uint8_t targetRoll;
  uint8_t targetRudder;
  uint8_t targetElevators;
  uint8_t targetEngine;

  // Trim settings
  int8_t elevatorTrim = 0;
  int8_t aileronTrim = 0;
  uint8_t flaps = 0;
  bool landingAirbrake = false;

  // Safety and status
  unsigned long lastReceivedTime;
  unsigned long lastI2CCommand;
  bool connectionActive;
  bool slaveHealthy;

  // Flight mode
  FlightMode currentFlightMode;

  // Sensor data from slave
  FlightDataPacket latestFlightData;
  bool newFlightDataAvailable;

  // Private helper functions
  void checkConnectionTimeout();
  bool isValidControlValue(uint8_t value);
  void logControlChanges();
  bool sendServoCommands();
  bool requestFlightData();
  bool requestSlaveStatus();

 public:
  // Singleton instance getter
  static AirplaneMaster& getInstance();

  // Public utility functions
  void initialize();
  void update();
  bool isControlInputValid();
  void emergencyShutdown();
  String getStatusString();

  // 🔧 Setters for flight controls
  void setThrottle(uint8_t value);   // Engine throttle 0-100% | 0 - 180 ⚡
  void setRudder(uint8_t value);     // 🎯
  void setElevators(uint8_t value);  // ⬆️⬇️
  void setAilerons(uint8_t value);   // Set both ailerons 🛩️

  // 🔧 Setter for trim
  void setElevatorTrim(int8_t value);
  void setAileronTrim(int8_t value);
  void setFlaps(uint8_t value);          // 🪶
  void setLandingAirbrake(bool active);  // 🛑

  // 🔄 Reset trim functions
  void resetAileronTrim();
  void resetElevatorTrim();

  // ⚡ Safety setters
  void resetToSafeDefaults();
  void updateLastReceivedTime();
  void setConnectionStatus(bool active);

  // High-level flight control functions
  void setRollAngle(float degrees);   // Roll left (-) or right (+)
  void setPitchAngle(float degrees);  // Pitch up (+) or down (-)
  void setYawAngle(float degrees);    // Yaw left (-) or right (+)

  // Combined maneuver functions
  void performLevel();  // Level flight (all controls neutral)
  void performLanding(float glidePath = -3.0);

  // Flight mode functions
  void setFlightMode(FlightMode mode);  // Set flight mode

  // Getters
  bool isConnectionActive() const;
  bool isSlaveHealthy() const;
  FlightMode getFlightMode() const;
  String getFlightModeString() const;

  // Sensor data getters
  bool getFlightData(FlightDataPacket& data);
  float getCurrentRoll() const;
  float getCurrentPitch() const;
  float getCurrentYaw() const;
  float getCurrentAltitude() const;
  float getCurrentTemperature() const;
};

#endif  // AIRPLANE_MASTER_H
