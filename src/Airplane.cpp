#include "Airplane.h"
#include "common.h"

// =============================================================================
// STATIC MEMBERS AND SINGLETON IMPLEMENTATION ✈️
// =============================================================================

// Static instance initialization
Airplane* Airplane::instance = nullptr;

// =============================================================================
// CONSTRUCTOR 🏗️
// =============================================================================

Airplane::Airplane() {
  // Initialize default values
  targetRoll = 90;       // Neutral position
  targetRudder = 90;     // Neutral position
  targetElevators = 90;  // Neutral position
  targetEngine = 0;      // Engine off

  elevatorTrim = 0;
  aileronTrim = 0;
  flaps = 0;
  landingAirbrake = false;

  lastReceivedTime = millis();
  lastI2CCommand = 0;
  connectionActive = false;
  slaveHealthy = false;
  newFlightDataAvailable = false;

  currentFlightMode = FlightMode::MANUAL;

  // Initialize flight data
  memset(&latestFlightData, 0, sizeof(latestFlightData));
}

// Singleton getInstance method
Airplane& Airplane::getInstance() {
  if (instance == nullptr) {
    instance = new Airplane();
  }
  return *instance;
}

void Airplane::initialize() {
  Serial.println("🛩️ Initializing Airplane Master Controller");

  initializeServos();
  initializeEngines();

  // Set safe defaults
  resetToSafeDefaults();

  // Test slave communication
  delay(100);  // Give slave time to initialize
  if (requestSlaveStatus()) {
    Serial.println("✅ Slave communication established");
    slaveHealthy = true;
  } else {
    Serial.println("❌ Warning: Cannot communicate with slave");
    slaveHealthy = false;
  }
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
  unsigned long currentTime = millis();

  // Request flight data every 100ms (10Hz)
  // static unsigned long lastDataRequest = 0;
  // if (currentTime - lastDataRequest >= 100) {
  //   requestFlightData();
  //   lastDataRequest = currentTime;
  // }

  checkConnectionTimeout();

  // Log control changes periodically
  // static unsigned long lastLog = 0;
  // if (currentTime - lastLog >= 1000) {
  //   logControlChanges();
  //   lastLog = currentTime;
  // }

  static unsigned long lastUpdate = 0;
  static int updateCount = 0;
  updateCount++;

  if (millis() - lastUpdate > 1000) {
    Serial.println("📊 Flight Control Updates/sec: " + String(updateCount) + " (Core: " + String(xPortGetCoreID()) + ")");
    updateCount = 0;
    lastUpdate = millis();
  }
}

bool Airplane::requestFlightData() {
  // Request flight data from slave
  Wire.beginTransmission(SERVO_CONTROLLER_ADDRESS);
  Wire.write(FLIGHT_DATA_REQUEST);
  uint8_t result = Wire.endTransmission();

  if (result != 0)
    return false;

  // Read response
  Wire.requestFrom(SERVO_CONTROLLER_ADDRESS, (int)sizeof(FlightDataPacket));

  if (Wire.available() >= sizeof(FlightDataPacket)) {
    FlightDataPacket tempData;
    Wire.readBytes((uint8_t*)&tempData, sizeof(FlightDataPacket));

    // Validate checksum
    if (validateChecksum((uint8_t*)&tempData, sizeof(FlightDataPacket))) {
      latestFlightData = tempData;
      newFlightDataAvailable = true;
      return true;
    } else {
      Serial.println("❌ Flight data checksum failed");
    }
  }

  return false;
}

bool Airplane::requestSlaveStatus() {
  Wire.beginTransmission(SERVO_CONTROLLER_ADDRESS);
  Wire.write(STATUS_REQUEST);
  uint8_t result = Wire.endTransmission();

  if (result != 0)
    return false;

  Wire.requestFrom(SERVO_CONTROLLER_ADDRESS, (int)sizeof(StatusPacket));

  if (Wire.available() >= sizeof(StatusPacket)) {
    StatusPacket status;
    Wire.readBytes((uint8_t*)&status, sizeof(StatusPacket));

    if (validateChecksum((uint8_t*)&status, sizeof(StatusPacket))) {
      Serial.printf("📊 Slave Status - Healthy: %s, Uptime: %lu ms, IMU: %d\n", status.servos_healthy ? "✅" : "❌", status.uptime_ms,
                    status.imu_status);
      return status.servos_healthy;
    }
  }

  return false;
}

// =============================================================================
// BASIC SETTERS (LOW-LEVEL CONTROL) 🎮
// =============================================================================

void Airplane::setThrottle(uint8_t value) {
  if (isValidControlValue(value)) {
    targetEngine = constrain(value, 0, 180);
    updateLastReceivedTime();
    writeToServos();
  }
}

void Airplane::setRudder(uint8_t value) {
  if (isValidControlValue(value)) {
    targetRudder = map(value, 0, 180, 90 - rudderHalfAngleFreedom, 90 + rudderHalfAngleFreedom);
    updateLastReceivedTime();
    writeToServos();
  }
}

void Airplane::setElevators(uint8_t value) {
  if (isValidControlValue(value)) {
    targetElevators = constrain(value, 0, 180);
    updateLastReceivedTime();
    writeToServos();
  }
}

void Airplane::setAilerons(uint8_t value) {
  if (isValidControlValue(value)) {
    targetRoll = constrain(value, 0, 180);
    updateLastReceivedTime();
    writeToServos();
  }
}

void Airplane::setElevatorTrim(int8_t value) {
  if (value > 0)
    elevatorTrim += TRIM_STEP;
  else if (value < 0)
    elevatorTrim -= TRIM_STEP;

  elevatorTrim = constrain(elevatorTrim, -TRIM_LIMIT, TRIM_LIMIT);
  elevatorTrimToDisplay = elevatorTrim;  // Update trim for compatibility with Lora

  elevatorTrimReceived = 0;
}

void Airplane::setAileronTrim(int8_t value) {
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

void Airplane::setFlaps(uint8_t value) {
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

void Airplane::resetToSafeDefaults() {
  targetEngine = 0;         // Engine off
  targetRoll = 90;          // Neutral
  targetElevators = 90;     // Neutral
  targetRudder = 90;        // Neutral
  elevatorTrim = 0;         // No trim
  aileronTrim = 0;          // No trim
  flaps = 0;                // No flaps
  landingAirbrake = false;  // Airbrake off

  // Reset to safe flight mode
  currentFlightMode = FlightMode::STABILITY;

  Serial.println("🔒 Flight controls reset to safe defaults");

  writeToServos();
}

// =============================================================================
// GETTERS (CONST FUNCTIONS) 📊
// =============================================================================

bool Airplane::getFlightData(FlightDataPacket& data) {
  if (newFlightDataAvailable) {
    data = latestFlightData;
    newFlightDataAvailable = false;
    return true;
  }
  return false;
}

float Airplane::getCurrentRoll() const {
  return latestFlightData.roll;
}

float Airplane::getCurrentPitch() const {
  return latestFlightData.pitch;
}

float Airplane::getCurrentYaw() const {
  return latestFlightData.yaw;
}

float Airplane::getCurrentAltitude() const {
  return latestFlightData.altitude;
}

bool Airplane::isSlaveHealthy() const {
  return slaveHealthy;
}

void Airplane::checkConnectionTimeout() {
  if (connectionActive && (millis() - lastReceivedTime > CONNECTION_TIMEOUT)) {
    Serial.println("⚠️ Connection timeout - activating emergency shutdown");
    emergencyShutdown();
    connectionActive = false;
  }
}

void Airplane::emergencyShutdown() {
  Serial.println("🚨 EMERGENCY SHUTDOWN ACTIVATED");
  resetToSafeDefaults();
}

bool Airplane::isValidControlValue(uint8_t value) {
  return (value >= 0 && value <= 180);  // Basic range check
}

void Airplane::updateLastReceivedTime() {
  lastReceivedTime = millis();

  if (!connectionActive) {
    connectionActive = true;
    Serial.println("📡 Connection restored");
  }
}

void Airplane::logControlChanges() {
  Serial.printf("🎮 Controls - Engine: %d, Roll: %d, Elevator: %d, Rudder: %d, Slave: %s\n", targetEngine, targetRoll, targetElevators,
                targetRudder, slaveHealthy ? "✅" : "❌");
}

String Airplane::getStatusString() {
  return String("Master OK - Slave: ") + (slaveHealthy ? "Healthy" : "Disconnected");
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

  // logControlChanges();
}

// String Airplane::getFlightModeString() const {
//   switch (currentFlightMode) {
//     case FlightMode::LANDING:
//       return "🛬 Landing";
//     case FlightMode::ACROBATIC:
//       return "🎢 Acrobatic";
//     case FlightMode::STABILITY:
//       return "📐 Stability";
//     case FlightMode::MANUAL:
//       return "🎮 Manual";
//     default:
//       return "❓ Unknown";
//   }
// }

// String Airplane::getStatusString() {
//   String status = "✈️ Airplane Status:\n";
//   status += "⚡ Engine: " + String(targetEngine) + "\n";
//   status += "🛩️ Aileron: " + String(targetRoll) + "\n";
//   status += "🎯 Rudder: " + String(targetRudder) + "\n";
//   status += "⬆️⬇️ Elevators: " + String(targetElevators) + "\n";
//   status += "🔧 Trim: " + String(elevatorTrim) + "\n";
//   status += "🔧 Aileron Trim: " + String(aileronTrim) + "\n";
//   status += "🪶 Flaps: " + String(flaps) + "\n";
//   status += "📡 Connection: " + String(connectionActive ? "🟢 Active" : "🔴 Inactive") + "\n";
//   return status;
// }

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
