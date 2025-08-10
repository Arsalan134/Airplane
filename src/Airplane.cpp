#include "Header Files\Airplane.h"

// Constructor
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
