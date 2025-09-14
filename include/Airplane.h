#include <Arduino.h>
#include <ESP32Servo.h>
#include "FlightProtocol.h"
#include "IMU.h"

// 🔌 Servo pin definitions
#define ENGINE_PIN 4
#define ROLL_RIGHT_MOTOR_PIN 25
#define ROLL_LEFT_MOTOR_PIN 12       // green cable 🟢
#define ELEVATION_LEFT_MOTOR_PIN 13  // blue cable 🔵 used by sd card
#define ELEVATION_RIGHT_MOTOR_PIN 2  // blue cable 🔵 used by sd card
#define RUDDER_MOTOR_PIN 15          // yellow cable 🟡 used by sd card

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

  // Delete copy constructor and assignment operator
  Airplane(const Airplane&) = delete;
  Airplane& operator=(const Airplane&) = delete;

  // Servo objects
  Servo engineServos;
  Servo rollLeftMotorServo;
  Servo rollRightMotorServo;
  Servo elevationLeftMotorServo;
  Servo elevationRightMotorServo;
  Servo rudderMotorServo;

  // 🛩️ Flight control commands
  ServoCommandPacket servoCommands;

  // Safety and status
  unsigned long lastReceivedTime;
  unsigned long lastI2CCommand;
  bool connectionActive;
  bool slaveHealthy;

  // High-level flight parameters
  // float currentRollAngle;
  // float currentPitchAngle;
  // float currentYawAngle;

  // Flight mode
  FlightMode currentFlightMode;

  // IMU integration
  IMU& imu;
  IMUData latestIMUData;
  bool imuDataAvailable;
  unsigned long lastIMUUpdate;

  // Sensor data from slave
  // FlightDataPacket latestFlightData;
  // bool newFlightDataAvailable;

  // Private helper functions
  void checkConnectionTimeout();
  bool isValidControlValue(uint8_t value);
  void logControlChanges();
  bool requestFlightData();
  bool requestSlaveStatus();
  void writeToServos();
  
  // IMU helper functions
  void updateIMU();
  void processIMUData();
  bool isIMUDataFresh() const;

 public:
  // Singleton instance getter
  static Airplane& getInstance();

  // Public utility functions
  void initialize();
  void initializeServos();
  void initializeEngines();
  void initializeIMU();
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
  
  // IMU data getters
  bool getIMUData(IMUData& data);
  float getIMURoll() const;
  float getIMUPitch() const; 
  float getIMUYaw() const;
  float getIMURollRate() const;
  float getIMUPitchRate() const;
  float getIMUYawRate() const;
  bool isIMUReady() const;
  void calibrateIMU();
  void resetIMUOrientation();
};