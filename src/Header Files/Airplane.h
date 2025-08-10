#ifndef AIRPLANE_H
#define AIRPLANE_H

#include <Arduino.h>

class Airplane {
 private:
  // Singleton instance
  static Airplane* instance;

  // Private constructor for singleton
  Airplane();

  // Delete copy constructor and assignment operator
  Airplane(const Airplane&) = delete;
  Airplane& operator=(const Airplane&) = delete;

  // Flight control surfaces
  byte aileron;
  byte rudder;
  byte elevators;
  int engine;

  // Trim settings
  byte trim;
  static const byte trimStep = 2;

  // Battery monitoring
  int batteryLevel;
  static const int minAnalogReadFromBattery = 750;
  static const int maxAnalogReadFromBattery = 1000;

  // Safety and status
  unsigned long lastReceivedTime;
  bool connectionActive;

  // High-level flight parameters
  float currentRollAngle;
  float currentPitchAngle;
  float currentYawAngle;
  byte currentThrottle;

  // Flight modes
  bool stabilityModeEnabled;
  bool acrobaticModeEnabled;
  bool landingModeEnabled;

  // Private helper functions
  void validateControlSurfaces();
  void updateBatteryLevel();
  void checkConnectionTimeout();
  void applyTrimToControls();
  bool isValidControlValue(byte value);
  void resetToSafeDefaults();
  void logControlChanges();
  byte mapAngleToServo(float angle);
  float constrainAngle(float angle, float minAngle, float maxAngle);

 public:
  // Singleton instance getter
  static Airplane& getInstance();

  // Setters for flight controls
  void setAileron(byte value);
  void setRudder(byte value);
  void setElevators(byte value);
  void setEngine(int value);

  // Setter for trim
  void setTrim(byte value);
  void adjustTrimUp();
  void adjustTrimDown();

  // Safety setters
  void updateLastReceivedTime();
  void setConnectionStatus(bool active);

  // High-level flight control functions
  void setRollAngle(float degrees);    // Roll left (-) or right (+)
  void setBank(float degrees);         // Banking maneuver (alias for roll)
  void setPitchAngle(float degrees);   // Pitch up (+) or down (-)
  void setAttackAngle(float degrees);  // Set angle of attack
  void setYawAngle(float degrees);     // Yaw left (-) or right (+)
  void setThrottle(float percentage);  // Engine throttle 0-100%

  // Combined maneuver functions
  void performLevel();  // Level flight (all controls neutral)
  void performClimb(float angle, float throttle = 75);
  void performDescent(float angle, float throttle = 40);
  void performTurn(float bankAngle, float rudderInput = 0);
  void performBarrelRoll(int direction = 1);  // 1 = right, -1 = left
  void performLanding(float glidePath = -3.0);
  void performTakeoff(float throttle = 90);

  // Flight mode functions
  void setStabilityMode(bool enabled);  // Auto-leveling mode
  void setAcrobaticMode(bool enabled);  // Full control mode
  void setLandingMode(bool enabled);    // Landing assistance mode

  // Getters
  byte getAileron() const;
  byte getRudder() const;
  byte getElevators() const;
  int getEngine() const;
  byte getTrim() const;
  int getBatteryLevel() const;
  bool isConnectionActive() const;
  unsigned long getLastReceivedTime() const;

  // High-level getters
  float getRollAngle() const;
  float getPitchAngle() const;
  float getYawAngle() const;
  float getThrottle() const;
  String getFlightMode() const;

  // Public utility functions
  void initialize();
  void update();
  bool isControlInputValid();
  void emergencyShutdown();
  String getStatusString();
};

#endif  // AIRPLANE_H
