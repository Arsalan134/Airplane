#ifndef AIRPLANE_H
#define AIRPLANE_H

#include <Arduino.h>
#include <ESP32Servo.h>

enum class FlightMode { MANUAL = 0, STABILITY = 1, ACROBATIC = 2, LANDING = 3 };

class Airplane {
 private:
  // Singleton instance
  static Airplane* instance;

  // Private constructor for singleton
  Airplane();

  // Servo initialization
  void initializeServos();

  // Delete copy constructor and assignment operator
  Airplane(const Airplane&) = delete;
  Airplane& operator=(const Airplane&) = delete;

  // Flight control surfaces
  byte targetRoll;
  byte targetRudder;
  byte targetElevators;
  byte targetEngine;

  // Servo objects
  Servo engineServo;
  Servo rollLeftMotorServo;
  Servo elevationLeftMotorServo;
  Servo elevationRightMotorServo;
  Servo rudderMotorServo;

// Servo pin definitions
#define ENGINE_PIN 4
#define ROLL_LEFT_MOTOR_PIN 12
#define ELEVATION_LEFT_MOTOR_PIN 13
#define ELEVATION_RIGHT_MOTOR_PIN 2
#define RUDDER_MOTOR_PIN 15

#define rudderHalfAngleFreedom 30  // 30 degrees to the left and right
#define TRIM_LIMIT 45              // 45 degrees up and down
#define TRIM_STEP 2
#define FLAP_ANGLE 10

#define CONNECTION_TIMEOUT 2000

  // Trim settings
  int elevatorTrim = 0;
  int aileronTrim = 0;
  int flaps = 0;
  bool landingAirbrake = false;

  // Battery monitoring
  int batteryLevel;
#define MIN_ANALOG_READ_FROM_BATTERY 750
#define MAX_ANALOG_READ_FROM_BATTERY 1000

  // Safety and status
  unsigned long lastReceivedTime;
  bool connectionActive;

  // High-level flight parameters
  float currentRollAngle;
  float currentPitchAngle;
  float currentYawAngle;

  // Flight mode
  FlightMode currentFlightMode;

  // Private helper functions
  void updateBatteryLevel();
  void checkConnectionTimeout();
  bool isValidControlValue(byte value);
  void logControlChanges();
  // byte mapAngleToServo(float angle);
  void writeToServos();

 public:
  // Singleton instance getter
  static Airplane& getInstance();

  // Setters for flight controls
  void setThrottle(byte value);  // Engine throttle 0-100% | 0 - 180
  void setRudder(byte value);
  void setElevators(byte value);
  void setAilerons(byte value);  // Set both ailerons

  // Setter for trim
  void setElevatorTrim(int value);
  void setAileronTrim(int value);
  void setFlaps(int value);

  // Reset trim functions
  void resetAileronTrim();
  void resetElevatorTrim();

  // Safety setters
  void resetToSafeDefaults();
  void updateLastReceivedTime();
  void setConnectionStatus(bool active);

  // High-level flight control functions
  void setRollAngle(float degrees);   // Roll left (-) or right (+)
  void setPitchAngle(float degrees);  // Pitch up (+) or down (-)
  void setYawAngle(float degrees);    // Yaw left (-) or right (+)

  // Combined maneuver functions
  void performLevel();  // Level flight (all controls neutral)
  // void performTurn(float bankAngle, float rudderInput = 0);
  // void performBarrelRoll(int direction = 1);  // 1 = right, -1 = left
  void performLanding(float glidePath = -3.0);

  // Flight mode functions
  void setFlightMode(FlightMode mode);  // Set flight mode

  // Getters
  byte getAileron() const;
  byte getRudder() const;
  byte getElevators() const;
  byte getEngine() const;
  byte getTrim() const;
  byte getBatteryLevel() const;
  bool isConnectionActive() const;
  unsigned long getLastReceivedTime() const;

  // High-level getters
  float getRollAngle() const;
  float getPitchAngle() const;
  float getYawAngle() const;
  float getThrottle() const;
  FlightMode getFlightMode() const;
  String getFlightModeString() const;

  // Public utility functions
  void initialize();
  void update();
  bool isControlInputValid();
  void emergencyShutdown();
  String getStatusString();
};

#endif  // AIRPLANE_H
