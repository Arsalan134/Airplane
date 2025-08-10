#include "Header Files\Airplane.h"

// Initialize static instance pointer
Airplane* Airplane::instance = nullptr;

// Singleton getInstance method
Airplane& Airplane::getInstance() {
  if (instance == nullptr) {
    instance = new Airplane();
  }
  return *instance;
}

// Private constructor
Airplane::Airplane() {
  // Initialize control surfaces to neutral positions
  aileron = 90;
  rudder = 90;
  elevators = 90;
  engine = 0;

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
  stabilityModeEnabled = true;
  acrobaticModeEnabled = false;
  landingModeEnabled = false;
}

// Public setter functions
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

// Getter functions
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

int Airplane::getBatteryLevel() const {
  return batteryLevel;
}

bool Airplane::isConnectionActive() const {
  return connectionActive;
}

unsigned long Airplane::getLastReceivedTime() const {
  return lastReceivedTime;
}

// Public utility functions
void Airplane::initialize() {
  resetToSafeDefaults();
  updateBatteryLevel();
}

void Airplane::update() {
  checkConnectionTimeout();
  updateBatteryLevel();
  validateControlSurfaces();
}

bool Airplane::isControlInputValid() {
  return connectionActive && (millis() - lastReceivedTime < 1000);
}

void Airplane::emergencyShutdown() {
  engine = 0;
  aileron = 90;
  rudder = 90;
  elevators = 90;
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

// Private helper functions
void Airplane::validateControlSurfaces() {
  if (!isValidControlValue(aileron))
    aileron = 90;
  if (!isValidControlValue(rudder))
    rudder = 90;
  if (!isValidControlValue(elevators))
    elevators = 90;
  if (engine < 0 || engine > 255)
    engine = 0;
}

void Airplane::updateBatteryLevel() {
  // Read battery voltage and convert to percentage
  //   int analogRead = analogRead(A0);  // Assuming battery connected to A0
  //   if (analogRead < minAnalogReadFromBattery) {
  //     batteryLevel = 0;
  //   } else if (analogRead > maxAnalogReadFromBattery) {
  //     batteryLevel = 100;
  //   } else {
  //     batteryLevel = map(analogRead, minAnalogReadFromBattery, maxAnalogReadFromBattery, 0, 100);
  //   }
}

void Airplane::checkConnectionTimeout() {
  if (millis() - lastReceivedTime > 2000) {  // 2 second timeout
    connectionActive = false;
    // setEmergencyStop(true);
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
  engine = 0;
  aileron = 90;
  rudder = 90;
  elevators = 90;
  trim = 90;
}

void Airplane::logControlChanges() {
  // Log control changes for debugging (can be expanded)
  Serial.print("Controls updated - Aileron: ");
  Serial.print(aileron);
  Serial.print(" Rudder: ");
  Serial.print(rudder);
  Serial.print(" Elevators: ");
  Serial.print(elevators);
  Serial.print(" Engine: ");
  Serial.println(engine);
}

// High-level flight control functions
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

// Combined maneuver functions
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
  if (!acrobaticModeEnabled) {
    Serial.println("Acrobatic mode required for barrel roll");
    return;
  }

  float rollDirection = (direction > 0) ? 45.0 : -45.0;
  setRollAngle(rollDirection);
  Serial.print("Performing barrel roll ");
  Serial.println(direction > 0 ? "right" : "left");
}

void Airplane::performLoop(float throttle) {
  if (!acrobaticModeEnabled) {
    Serial.println("Acrobatic mode required for loop");
    return;
  }

  setThrottle(throttle);
  setPitchAngle(25.0);  // Initial climb for loop
  Serial.println("Performing loop maneuver");
}

void Airplane::performLanding(float glidePath) {
  setLandingMode(true);
  setPitchAngle(glidePath);
  setThrottle(25);  // Low throttle for landing
  Serial.print("Performing landing approach at ");
  Serial.print(glidePath);
  Serial.println(" degree glide path");
}

void Airplane::performTakeoff(float throttle) {
  setPitchAngle(15.0);  // Takeoff angle
  setThrottle(throttle);
  Serial.println("Performing takeoff");
}

// Flight mode functions
void Airplane::setStabilityMode(bool enabled) {
  stabilityModeEnabled = enabled;
  if (enabled) {
    acrobaticModeEnabled = false;
    Serial.println("Stability mode enabled");
  } else {
    Serial.println("Stability mode disabled");
  }
}

void Airplane::setAcrobaticMode(bool enabled) {
  acrobaticModeEnabled = enabled;
  if (enabled) {
    stabilityModeEnabled = false;
    landingModeEnabled = false;
    Serial.println("Acrobatic mode enabled - Full control");
  } else {
    Serial.println("Acrobatic mode disabled");
  }
}

void Airplane::setLandingMode(bool enabled) {
  landingModeEnabled = enabled;
  if (enabled) {
    stabilityModeEnabled = true;
    acrobaticModeEnabled = false;
    Serial.println("Landing mode enabled");
  } else {
    Serial.println("Landing mode disabled");
  }
}

// High-level getters
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

String Airplane::getFlightMode() const {
  if (landingModeEnabled)
    return "Landing";
  if (acrobaticModeEnabled)
    return "Acrobatic";
  if (stabilityModeEnabled)
    return "Stability";
  return "Manual";
}

// Private helper functions for high-level control
byte Airplane::mapAngleToServo(float angle) {
  // Map angle (-45 to +45 degrees) to servo range (0 to 180)
  return constrain(map(angle * 10, -450, 450, 0, 180), 0, 180);
}

float Airplane::constrainAngle(float angle, float minAngle, float maxAngle) {
  return constrain(angle, minAngle, maxAngle);
}