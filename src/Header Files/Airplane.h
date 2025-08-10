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

  // Private helper functions
  void validateControlSurfaces();
  void updateBatteryLevel();
  void checkConnectionTimeout();
  void applyTrimToControls();
  bool isValidControlValue(byte value);
  void resetToSafeDefaults();
  void logControlChanges();

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

  // Getters
  byte getAileron() const;
  byte getRudder() const;
  byte getElevators() const;
  int getEngine() const;
  byte getTrim() const;
  bool isEmergencyStopActive() const;
  int getBatteryLevel() const;
  bool isConnectionActive() const;
  unsigned long getLastReceivedTime() const;

  // Public utility functions
  void initialize();
  void update();
  bool isControlInputValid();
  void emergencyShutdown();
  String getStatusString();
};

#endif  // AIRPLANE_H
