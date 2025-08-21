#include "Airplane.h"
#include "common.h"

// =============================================================================
// STATIC MEMBERS AND SINGLETON IMPLEMENTATION ✈️
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
// CONSTRUCTOR 🏗️
// =============================================================================

// Private constructor
Airplane::Airplane() {
  // Initialize control surfaces to neutral positions
  targetEngine = 0;
  targetRoll = 90;
  targetRudder = 90;
  targetElevators = 90;

  // Initialize trim
  elevatorTrim = 0;
  aileronTrim = 0;

  // Initialize safety settings
  lastReceivedTime = millis();
  connectionActive = false;

  // Initialize flight modes
  currentFlightMode = FlightMode::STABILITY;
}

// =============================================================================
// GETTERS (CONST FUNCTIONS) 📊
// =============================================================================

bool Airplane::isConnectionActive() const {
  return connectionActive;
}

// High-level flight parameter getters
FlightMode Airplane::getFlightMode() const {
  return currentFlightMode;
}

String Airplane::getFlightModeString() const {
  switch (currentFlightMode) {
    case FlightMode::LANDING:
      return "🛬 Landing";
    case FlightMode::ACROBATIC:
      return "🎢 Acrobatic";
    case FlightMode::STABILITY:
      return "📐 Stability";
    case FlightMode::MANUAL:
      return "🎮 Manual";
    default:
      return "❓ Unknown";
  }
}

bool Airplane::isControlInputValid() {
  return connectionActive && (millis() - lastReceivedTime < 1000);
}

String Airplane::getStatusString() {
  String status = "✈️ Airplane Status:\n";
  status += "⚡ Engine: " + String(targetEngine) + "\n";
  status += "🛩️ Aileron: " + String(targetRoll) + "\n";
  status += "🎯 Rudder: " + String(targetRudder) + "\n";
  status += "⬆️⬇️ Elevators: " + String(targetElevators) + "\n";
  status += "🔧 Trim: " + String(elevatorTrim) + "\n";
  status += "🔧 Aileron Trim: " + String(aileronTrim) + "\n";
  status += "🪶 Flaps: " + String(flaps) + "\n";
  status += "📡 Connection: " + String(connectionActive ? "🟢 Active" : "🔴 Inactive") + "\n";
  return status;
}

// =============================================================================
// BASIC SETTERS (LOW-LEVEL CONTROL) 🎮
// =============================================================================

void Airplane::setElevators(byte value) {
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

void Airplane::setElevatorTrim(int value) {
  if (value > 0)
    elevatorTrim += TRIM_STEP;
  else if (value < 0)
    elevatorTrim -= TRIM_STEP;

  elevatorTrim = constrain(elevatorTrim, -TRIM_LIMIT, TRIM_LIMIT);
  elevatorTrimToDisplay = elevatorTrim;  // Update trim for compatibility with Lora

  elevatorTrimReceived = 0;
}

void Airplane::setAileronTrim(int value) {
  if (value > 0)
    aileronTrim += TRIM_STEP;
  else if (value < 0)
    aileronTrim -= TRIM_STEP;
  else
    return;

  aileronTrim = constrain(aileronTrim, -TRIM_LIMIT, TRIM_LIMIT);
  aileronTrimToDisplay = aileronTrim;

  aileronTrimReceived = 0;
}

void Airplane::setFlaps(int value) {
  flaps = constrain(value, -TRIM_LIMIT, TRIM_LIMIT);
  flapsToDisplay = flaps;
}

void Airplane::resetAileronTrim() {
  aileronTrim = 0;
  aileronTrimToDisplay = aileronTrim;  // Update trim for compatibility with Lora
  aileronTrimReceived = 0;
}

void Airplane::resetElevatorTrim() {
  elevatorTrim = 0;
  elevatorTrimToDisplay = elevatorTrim;  // Update trim for compatibility with Lora
}

void Airplane::setLandingAirbrake(bool active) {
  landingAirbrake = active;
}

// =============================================================================
// HIGH-LEVEL SETTERS (FLIGHT CONTROL) 🛩️
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
// PUBLIC UTILITY FUNCTIONS 🔧
// =============================================================================

void Airplane::initialize() {
  initializeServos();
  initializeEngines();
  resetToSafeDefaults();

  Serial.println("✅ Airplane initialized");
}

void Airplane::initializeServos() {
  rollLeftMotorServo.attach(ROLL_LEFT_MOTOR_PIN);
  elevationLeftMotorServo.attach(ELEVATION_LEFT_MOTOR_PIN);
  elevationRightMotorServo.attach(ELEVATION_RIGHT_MOTOR_PIN);
  rudderMotorServo.attach(RUDDER_MOTOR_PIN);

  Serial.println("🔧 Servos initialized successfully");
}

void Airplane::initializeEngines() {
  Serial.println("⚡ Engines Test Starting...");

  // Attach servo with proper PWM range for ESC
  engineServos.attach(ENGINE_PIN, 1000, 2000);

  Serial.println("🚀 Engines initialized successfully");
}

void Airplane::update() {
  checkConnectionTimeout();

  // Core-specific performance logging
  static unsigned long lastUpdate = 0;
  static int updateCount = 0;
  updateCount++;

  if (millis() - lastUpdate > 1000) {
    Serial.println("📊 Flight Control Updates/sec: " + String(updateCount) + " (Core: " + String(xPortGetCoreID()) + ")");
    updateCount = 0;
    lastUpdate = millis();
  }
}

void Airplane::emergencyShutdown() {
  resetToSafeDefaults();  // Center all controls
  Serial.println("🚨 Emergency shutdown executed");
  delay(100);
}

void Airplane::checkConnectionTimeout() {
  if (millis() - lastReceivedTime > CONNECTION_TIMEOUT) {  // 2 second timeout
    if (connectionActive) {
      connectionActive = false;
      // setEmergencyStop(true);
      Serial.println("⚠️ Connection timeout - Emergency stop activated");
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

  // Reset to safe flight mode
  currentFlightMode = FlightMode::STABILITY;

  writeToServos();
}

void Airplane::writeToServos() {
  // Engine
  engineServos.write(constrain(targetEngine, 0, 180));

  // Ailerons
  int targetRollForServo = targetRoll;
  bool shouldApplyFlaps = abs(targetRollForServo - 90) < 10;

  targetRollForServo += aileronTrim;
  targetRollForServo += shouldApplyFlaps ? flaps * FLAP_ANGLE : 0;

  if (landingAirbrake)
    targetRollForServo = 0;

  rollLeftMotorServo.write(constrain(180 - targetRollForServo, 0, 180));  // Left aileron

  // Elevators
  elevationLeftMotorServo.write(constrain(targetElevators + elevatorTrim, 0, 180));
  elevationRightMotorServo.write(constrain(180 - targetElevators - elevatorTrim, 0, 180));  // Right elevator inverted

  // Rudder
  rudderMotorServo.write(constrain(targetRudder, 0, 180));

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
