#include "Header Files\Airplane.h"

// =============================================================================
// STATIC MEMBERS AND SINGLETON IMPLEMENTATION
// =============================================================================

// Initialize static instance pointer
Airplane* Airplane::instance = nullptr;

// Singleton getInstance method
Airplane& Airplane::getInstance() {
  if (instance == nullptr) {
    instance = new Airplane();
  }
  return *instance;
}

// =============================================================================
// CONSTRUCTOR
// =============================================================================

// Private constructor
Airplane::Airplane() {
  // Initialize control surfaces to neutral positions
  targetEngine = 0;
  targetRoll = 90;
  targetRudder = 90;
  targetElevators = 90;

  // Initialize trim
  trim = 0;

  // Initialize safety settings
  lastReceivedTime = millis();
  connectionActive = false;
  batteryLevel = 0;

  // Initialize flight modes
  currentFlightMode = FlightMode::STABILITY;
}

// =============================================================================
// GETTERS (CONST FUNCTIONS)
// =============================================================================

// Basic control getters
byte Airplane::getAileron() const {
  return targetRoll;
}

byte Airplane::getRudder() const {
  return targetRudder;
}

byte Airplane::getElevators() const {
  return targetElevators;
}

byte Airplane::getEngine() const {
  return targetEngine;
}

byte Airplane::getTrim() const {
  return trim;
}

// Status getters
byte Airplane::getBatteryLevel() const {
  return batteryLevel;
}

bool Airplane::isConnectionActive() const {
  return connectionActive;
}

unsigned long Airplane::getLastReceivedTime() const {
  return lastReceivedTime;
}

// High-level flight parameter getters
float Airplane::getRollAngle() const {
  return currentRollAngle;
}

float Airplane::getPitchAngle() const {
  return currentPitchAngle;
}

float Airplane::getYawAngle() const {
  return currentYawAngle;
}

float Airplane::getThrottle() const {
  return targetEngine;
}

FlightMode Airplane::getFlightMode() const {
  return currentFlightMode;
}

String Airplane::getFlightModeString() const {
  switch (currentFlightMode) {
    case FlightMode::LANDING:
      return "Landing";
    case FlightMode::ACROBATIC:
      return "Acrobatic";
    case FlightMode::STABILITY:
      return "Stability";
    case FlightMode::MANUAL:
      return "Manual";
    default:
      return "Unknown";
  }
}

// Complex getters
bool Airplane::isControlInputValid() {
  return connectionActive && (millis() - lastReceivedTime < 1000);
}

String Airplane::getStatusString() {
  String status = "Airplane Status:\n";
  status += "Engine: " + String(targetEngine) + "\n";
  status += "Aileron: " + String(targetRoll) + "\n";
  status += "Rudder: " + String(targetRudder) + "\n";
  status += "Elevators: " + String(targetElevators) + "\n";
  status += "Trim: " + String(trim) + "\n";
  status += "Connection: " + String(connectionActive ? "Active" : "Inactive") + "\n";
  status += "Battery: " + String(batteryLevel) + "%";
  return status;
}

// =============================================================================
// BASIC SETTERS (LOW-LEVEL CONTROL)
// =============================================================================

void Airplane::setElevators(byte value) {
  value += trim;
  if (isValidControlValue(value)) {
    targetElevators = value;
    writeToServos();
  }
}

void Airplane::setAilerons(byte value) {
  if (isValidControlValue(value)) {
    targetRoll = value;
    writeToServos();
  }
}

void Airplane::setRudder(byte value) {
  if (isValidControlValue(value)) {
    targetRudder = map(value, 0, 180, 90 - rudderHalfAngleFreedom, 90 + rudderHalfAngleFreedom);
    writeToServos();
  }
}

void Airplane::setThrottle(byte value) {
  if (isValidControlValue(value)) {
    targetEngine = value;
    writeToServos();
  }
}

void Airplane::setTrim(byte value) {
  if (value > 0)
    adjustTrimUp();
  else if (value < 0)
    adjustTrimDown();
}

void Airplane::adjustTrimUp() {
  trim += TRIM_STEP;
  trim = constrain(trim, -TRIM_LIMIT, TRIM_LIMIT);
  Serial.println("Trim adjusted up to: " + String(trim));
}

void Airplane::adjustTrimDown() {
  trim -= TRIM_STEP;
  trim = constrain(trim, -TRIM_LIMIT, TRIM_LIMIT);
  Serial.println("Trim adjusted down to: " + String(trim));
}

// =============================================================================
// HIGH-LEVEL SETTERS (FLIGHT CONTROL)
// =============================================================================

// TODO:- Need to rethink this part and degrees
void Airplane::setRollAngle(float degrees) {
  // targetAileron = constrainAngle(degrees, -45, 45);
  writeToServos();
  logControlChanges();
}

void Airplane::setPitchAngle(float degrees) {
  // targetElevators = constrainAngle(degrees, 0, 180);
  writeToServos();
  logControlChanges();
}

void Airplane::setYawAngle(float degrees) {
  // targetRudder = constrainAngle(degrees, 0, 180);
  writeToServos();
  logControlChanges();
}

// =============================================================================
// SAFETY AND CONNECTION SETTERS
// =============================================================================

void Airplane::updateLastReceivedTime() {
  lastReceivedTime = millis();
  connectionActive = true;
}

void Airplane::setConnectionStatus(bool active) {
  connectionActive = active;
}

// =============================================================================
// FLIGHT MODE SETTERS
// =============================================================================

void Airplane::setFlightMode(FlightMode mode) {
  currentFlightMode = mode;
  Serial.println("Flight mode set to: " + getFlightModeString());
}

// =============================================================================
// COMBINED MANEUVER FUNCTIONS
// =============================================================================

void Airplane::performLevel() {
  setRollAngle(0);
  setPitchAngle(0);
  setYawAngle(0);
  Serial.println("Performing level flight");
}

void Airplane::performLanding(float glidePath) {
  setFlightMode(FlightMode::LANDING);
  setPitchAngle(glidePath);
  setThrottle(25);  // Low throttle for landing
  Serial.print("Performing landing approach at ");
  Serial.print(glidePath);
  Serial.println(" degree glide path");
}

// =============================================================================
// PUBLIC UTILITY FUNCTIONS
// =============================================================================

void Airplane::initialize() {
  initializeServos();
  resetToSafeDefaults();
  updateBatteryLevel();
  Serial.println("Airplane initialized");
}

void Airplane::initializeServos() {
  engineServo.attach(ENGINE_PIN, 1000, 2000);
  rollLeftMotorServo.attach(ROLL_LEFT_MOTOR_PIN);
  elevationLeftMotorServo.attach(ELEVATION_LEFT_MOTOR_PIN);
  elevationRightMotorServo.attach(ELEVATION_RIGHT_MOTOR_PIN);
  rudderMotorServo.attach(RUDDER_MOTOR_PIN);

  Serial.println("Servos initialized");
}

void Airplane::update() {
  checkConnectionTimeout();
  updateBatteryLevel();
}

void Airplane::emergencyShutdown() {
  resetToSafeDefaults();  // Center all controls
  Serial.println("Emergency shutdown executed");
  delay(100);
}

void Airplane::updateBatteryLevel() {
  // Read battery voltage and convert to percentage
  // int analogValue = analogRead(A0); // Assuming battery connected to A0
  // if (analogValue < minAnalogReadFromBattery) {
  //   batteryLevel = 0;
  // } else if (analogValue > maxAnalogReadFromBattery) {
  //   batteryLevel = 100;
  // } else {
  //   batteryLevel = map(analogValue, minAnalogReadFromBattery, maxAnalogReadFromBattery, 0, 100);
  // }

  // Placeholder for now
  batteryLevel = 85;
}

void Airplane::checkConnectionTimeout() {
  if (millis() - lastReceivedTime > 2000) {  // 2 second timeout
    if (connectionActive) {
      connectionActive = false;
      // setEmergencyStop(true);
      Serial.println("Connection timeout - Emergency stop activated");
    }
  }
}

bool Airplane::isValidControlValue(byte value) {
  return value >= 0 && value <= 180;
}

void Airplane::resetToSafeDefaults() {
  targetEngine = 0;

  targetRoll = 90;
  targetRudder = 90;
  targetElevators = 90;
  trim = 0;

  // Reset to safe flight mode
  currentFlightMode = FlightMode::STABILITY;

  writeToServos();
}

// byte Airplane::mapAngleToServo(float angle) {
//   // Map angle (-45 to +45 degrees) to servo range (0 to 180)
//   // Center position is 90 degrees
//   int servoValue = 90 + (angle * 2);  // Scale angle to servo range
//   return constrain(servoValue, 0, 180);
// }

void Airplane::writeToServos() {
  engineServo.write(targetEngine);
  rollLeftMotorServo.write(targetRoll);
  elevationLeftMotorServo.write(targetElevators);
  elevationRightMotorServo.write(180 - targetElevators);  // Right elevator inverted
  rudderMotorServo.write(targetRudder);

  logControlChanges();
}

void Airplane::logControlChanges() {
  // Log control changes for debugging
  // Serial.print("Controls - A:");
  // Serial.print(targetRoll);
  // Serial.print(" R:");
  // Serial.print(targetRudder);
  // Serial.print(" E:");
  // Serial.print(targetElevators);
  // Serial.print(" Engine:");
  // Serial.print(targetEngine);
  // Serial.print(" Roll:");
  // Serial.print(currentRollAngle);
  // Serial.print(" Pitch:");
  // Serial.print(currentPitchAngle);
  // Serial.print(" Yaw:");
  // Serial.println(currentYawAngle);
}
