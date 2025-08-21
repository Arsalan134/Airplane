#ifndef AIRPLANE_H
#define AIRPLANE_H

#include <Arduino.h>
#include <ESP32Servo.h>

// 🔌 Servo pin definitions
#define ENGINE_PIN 4
#define ROLL_LEFT_MOTOR_PIN 12       // green cable 🟢
#define ELEVATION_LEFT_MOTOR_PIN 13  // blue cable 🔵 used by sd card
#define ELEVATION_RIGHT_MOTOR_PIN 2  // blue cable 🔵 used by sd card
#define RUDDER_MOTOR_PIN 15          // yellow cable 🟡

#define rudderHalfAngleFreedom 30  // 30 degrees to the left and right 🎯
#define TRIM_LIMIT 45              // 45 degrees up and down ⬆️⬇️
#define TRIM_STEP 2
#define FLAP_ANGLE 10  // 🪶

#define CONNECTION_TIMEOUT 2000  // ⏱️

enum class FlightMode { MANUAL = 0, STABILITY = 1, ACROBATIC = 2, LANDING = 3 };  // 🛩️

class Airplane {
 private:
  // 🔒 Singleton instance
  static Airplane* instance;

  // Private constructor for singleton
  Airplane();

  // Servo initialization
  void initializeServos();
  void initializeEngines();

  // Delete copy constructor and assignment operator
  Airplane(const Airplane&) = delete;
  Airplane& operator=(const Airplane&) = delete;

  // 🛩️ Flight control surfaces
  byte targetRoll;
  byte targetRudder;
  byte targetElevators;
  /// @brief Target engine throttle (0-100%) from 0 to 180 ⚡
  byte targetEngine;

  // Servo objects
  Servo engineServos;
  Servo rollLeftMotorServo;
  Servo elevationLeftMotorServo;
  Servo elevationRightMotorServo;
  Servo rudderMotorServo;

  // Trim settings
  int elevatorTrim = 0;
  int aileronTrim = 0;
  int flaps = 0;
  bool landingAirbrake = false;

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
  void checkConnectionTimeout();
  bool isValidControlValue(byte value);
  void logControlChanges();
  void writeToServos();

 public:
  // Singleton instance getter
  static Airplane& getInstance();

  // Public utility functions
  void initialize();
  void update();
  bool isControlInputValid();
  void emergencyShutdown();
  String getStatusString();

  // 🔧 Setters for flight controls
  void setThrottle(byte value);   // Engine throttle 0-100% | 0 - 180 ⚡
  void setRudder(byte value);     // 🎯
  void setElevators(byte value);  // ⬆️⬇️
  void setAilerons(byte value);   // Set both ailerons 🛩️

  // 🔧 Setter for trim
  void setElevatorTrim(int value);
  void setAileronTrim(int value);
  void setFlaps(int value);              // 🪶
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
  // void performTurn(float bankAngle, float rudderInput = 0);
  // void performBarrelRoll(int direction = 1);  // 1 = right, -1 = left
  void performLanding(float glidePath = -3.0);

  // Flight mode functions
  void setFlightMode(FlightMode mode);  // Set flight mode

  // Getters
  bool isConnectionActive() const;

  // High-level getters
  FlightMode getFlightMode() const;
  String getFlightModeString() const;
};

#endif  // AIRPLANE_H
