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
  engine = 0;
  aileron = 90;
  rudder = 90;
  elevators = 90;

  // Initialize trim
  trim = 90;

  // Initialize safety settings
  lastReceivedTime = millis();
  connectionActive = false;
  batteryLevel = 0;

  // Initialize high-level flight parameters
  currentRollAngle = 0.0;
  currentPitchAngle = 0.0;
  currentYawAngle = 0.0;
  currentThrottle = 0;

  // Initialize flight modes
  currentFlightMode = FlightMode::STABILITY;
}

// =============================================================================
// GETTERS (CONST FUNCTIONS)
// =============================================================================

// Basic control getters
byte Airplane::getAileron() const {
  return aileron;
}

byte Airplane::getRudder() const {
  return rudder;
}

byte Airplane::getElevators() const {
  return elevators;
}

int Airplane::getEngine() const {
  return engine;
}

byte Airplane::getTrim() const {
  return trim;
}

// Status getters
int Airplane::getBatteryLevel() const {
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
  return currentThrottle;
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
  status += "Engine: " + String(engine) + "\n";
  status += "Aileron: " + String(aileron) + "\n";
  status += "Rudder: " + String(rudder) + "\n";
  status += "Elevators: " + String(elevators) + "\n";
  status += "Trim: " + String(trim) + "\n";
  status += "Connection: " + String(connectionActive ? "Active" : "Inactive") + "\n";
  status += "Battery: " + String(batteryLevel) + "%";
  return status;
}

// =============================================================================
// BASIC SETTERS (LOW-LEVEL CONTROL)
// =============================================================================

void Airplane::setAileron(byte value) {
  if (isValidControlValue(value)) {
    aileron = value;
    logControlChanges();
  }
}

void Airplane::setRudder(byte value) {
  if (isValidControlValue(value)) {
    rudder = value;
    logControlChanges();
  }
}

void Airplane::setElevators(byte value) {
  if (isValidControlValue(value)) {
    elevators = value;
    logControlChanges();
  }
}

void Airplane::setEngine(int value) {
  if (value >= 0 && value <= 255) {
    engine = value;
    logControlChanges();
  }
}

void Airplane::setTrim(byte value) {
  if (isValidControlValue(value)) {
    trim = value;
    applyTrimToControls();
  }
}

void Airplane::adjustTrimUp() {
  if (trim + trimStep <= 180) {
    trim += trimStep;
    applyTrimToControls();
  }
}

void Airplane::adjustTrimDown() {
  if (trim - trimStep >= 0) {
    trim -= trimStep;
    applyTrimToControls();
  }
}

// =============================================================================
// HIGH-LEVEL SETTERS (FLIGHT CONTROL)
// =============================================================================

void Airplane::setRollAngle(float degrees) {
  currentRollAngle = constrainAngle(degrees, -45.0, 45.0);
  aileron = mapAngleToServo(currentRollAngle);
  logControlChanges();
}

void Airplane::setBank(float degrees) {
  setRollAngle(degrees);  // Banking is the same as roll
}

void Airplane::setPitchAngle(float degrees) {
  currentPitchAngle = constrainAngle(degrees, -30.0, 30.0);
  elevators = mapAngleToServo(-currentPitchAngle);  // Inverted for elevator control
  logControlChanges();
}

void Airplane::setAttackAngle(float degrees) {
  setPitchAngle(degrees);  // Angle of attack is controlled by pitch
}

void Airplane::setYawAngle(float degrees) {
  currentYawAngle = constrainAngle(degrees, -30.0, 30.0);
  rudder = mapAngleToServo(currentYawAngle);
  logControlChanges();
}

void Airplane::setThrottle(float percentage) {
  percentage = constrain(percentage, 0.0, 100.0);
  currentThrottle = percentage;
  engine = map(percentage, 0, 100, 0, 255);
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
  if (!active) {
    emergencyShutdown();
  }
}

// =============================================================================
// FLIGHT MODE SETTERS
// =============================================================================

void Airplane::setFlightMode(FlightMode mode) {
  currentFlightMode = mode;
  Serial.print("Flight mode set to: ");
  Serial.println(getFlightModeString());
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

void Airplane::performClimb(float angle, float throttle) {
  setPitchAngle(abs(angle));  // Ensure positive angle for climb
  setThrottle(throttle);
  Serial.print("Performing climb at ");
  Serial.print(angle);
  Serial.println(" degrees");
}

void Airplane::performDescent(float angle, float throttle) {
  setPitchAngle(-abs(angle));  // Ensure negative angle for descent
  setThrottle(throttle);
  Serial.print("Performing descent at ");
  Serial.print(angle);
  Serial.println(" degrees");
}

void Airplane::performTurn(float bankAngle, float rudderInput) {
  setRollAngle(bankAngle);
  setYawAngle(rudderInput);
  Serial.print("Performing turn with ");
  Serial.print(bankAngle);
  Serial.println(" degree bank");
}

void Airplane::performBarrelRoll(int direction) {
  if (currentFlightMode != FlightMode::ACROBATIC) {
    Serial.println("Acrobatic mode required for barrel roll");
    return;
  }

  float rollDirection = (direction > 0) ? 45.0 : -45.0;
  setRollAngle(rollDirection);
  Serial.print("Performing barrel roll ");
  Serial.println(direction > 0 ? "right" : "left");
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
  resetToSafeDefaults();
  updateBatteryLevel();
  Serial.println("Airplane initialized");
}

void Airplane::update() {
  checkConnectionTimeout();
  updateBatteryLevel();
  validateControlSurfaces();
}

void Airplane::emergencyShutdown() {
  engine = 0;
  aileron = 90;
  rudder = 90;
  elevators = 90;
  currentThrottle = 0;
  currentRollAngle = 0;
  currentPitchAngle = 0;
  currentYawAngle = 0;
  Serial.println("Emergency shutdown executed");
}

// =============================================================================
// PRIVATE HELPER FUNCTIONS
// =============================================================================

void Airplane::validateControlSurfaces() {
  bool changed = false;

  if (!isValidControlValue(aileron)) {
    aileron = 90;
    changed = true;
  }
  if (!isValidControlValue(rudder)) {
    rudder = 90;
    changed = true;
  }
  if (!isValidControlValue(elevators)) {
    elevators = 90;
    changed = true;
  }
  if (engine < 0 || engine > 255) {
    engine = 0;
    changed = true;
  }

  if (changed) {
    Serial.println("Control surfaces validated and corrected");
  }
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

void Airplane::applyTrimToControls() {
  // Apply trim adjustment to elevators (common for pitch trim)
  int trimmedElevators = elevators + (trim - 90);
  if (trimmedElevators >= 0 && trimmedElevators <= 180) {
    elevators = trimmedElevators;
  }
}

bool Airplane::isValidControlValue(byte value) {
  return value >= 0 && value <= 180;
}

void Airplane::resetToSafeDefaults() {
  aileron = 90;
  rudder = 90;
  elevators = 90;
  engine = 0;
  trim = 90;

  // Reset high-level parameters
  currentRollAngle = 0.0;
  currentPitchAngle = 0.0;
  currentYawAngle = 0.0;
  currentThrottle = 0;

  // Reset to safe flight mode
  currentFlightMode = FlightMode::STABILITY;

  Serial.println("Reset to safe defaults");
}

void Airplane::logControlChanges() {
  // Log control changes for debugging
  Serial.print("Controls - A:");
  Serial.print(aileron);
  Serial.print(" R:");
  Serial.print(rudder);
  Serial.print(" E:");
  Serial.print(elevators);
  Serial.print(" Engine:");
  Serial.print(engine);
  Serial.print(" Roll:");
  Serial.print(currentRollAngle);
  Serial.print(" Pitch:");
  Serial.print(currentPitchAngle);
  Serial.print(" Yaw:");
  Serial.println(currentYawAngle);
}

byte Airplane::mapAngleToServo(float angle) {
  // Map angle (-45 to +45 degrees) to servo range (0 to 180)
  // Center position is 90 degrees
  int servoValue = 90 + (angle * 2);  // Scale angle to servo range
  return constrain(servoValue, 0, 180);
}

float Airplane::constrainAngle(float angle, float minAngle, float maxAngle) {
  return constrain(angle, minAngle, maxAngle);
}
