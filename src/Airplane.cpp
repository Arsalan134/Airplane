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
  // Initialize ServoCommandPacket with default values
  servoCommands.engine = 0;               // Engine off
  servoCommands.roll = 90;                // Neutral position
  servoCommands.elevators = 90;           // Neutral position
  servoCommands.rudder = 90;              // Neutral position
  servoCommands.trim_elevator = 0;        // No trim
  servoCommands.trim_aileron = 0;         // No trim
  servoCommands.flaps = 0;                // No flaps
  servoCommands.landingAirbrake = false;  // Airbrake off

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

  delay(200);  // Allow time for hardware to stabilize
  initializeServos();
  initializeEngines();

  // Set safe defaults
  resetToSafeDefaults();

  // Test slave communication
  // delay(100);  // Give slave time to initialize
  // if (requestSlaveStatus()) {
  //   Serial.println("✅ Slave communication established");
  //   slaveHealthy = true;
  // } else {
  //   Serial.println("❌ Warning: Cannot communicate with slave");
  //   slaveHealthy = false;
  // }
}

void Airplane::initializeServos() {
  rollLeftMotorServo.attach(ROLL_LEFT_MOTOR_PIN);
  rollRightMotorServo.attach(ROLL_RIGHT_MOTOR_PIN);
  elevationLeftMotorServo.attach(ELEVATION_LEFT_MOTOR_PIN);
  elevationRightMotorServo.attach(ELEVATION_RIGHT_MOTOR_PIN);
  rudderMotorServo.attach(RUDDER_MOTOR_PIN);

  Serial.println("🔧 Servos initialized successfully");
}

void Airplane::initializeEngines() {
  engineServos.attach(ENGINE_PIN, 1000, 2000);  // Attach servo with proper PWM range for ESC
  Serial.println("⚡Engines initialized successfully");
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
    servoCommands.engine = constrain(value, 0, 180);
    updateLastReceivedTime();
    writeToServos();
  }
}

void Airplane::setRudder(uint8_t value) {
  if (isValidControlValue(value)) {
    servoCommands.rudder = map(value, 0, 180, 90 - rudderHalfAngleFreedom, 90 + rudderHalfAngleFreedom);
    updateLastReceivedTime();
    writeToServos();
  }
}

void Airplane::setElevators(uint8_t value) {
  if (isValidControlValue(value)) {
    servoCommands.elevators = constrain(value, 0, 180);
    updateLastReceivedTime();
    writeToServos();
  }
}

void Airplane::setAilerons(uint8_t value) {
  if (isValidControlValue(value)) {
    servoCommands.roll = constrain(value, 0, 180);
    updateLastReceivedTime();
    writeToServos();
  }
}

void Airplane::setElevatorTrim(int8_t value) {
  if (value > 0)
    servoCommands.trim_elevator += TRIM_STEP;
  else if (value < 0)
    servoCommands.trim_elevator -= TRIM_STEP;

  servoCommands.trim_elevator = constrain(servoCommands.trim_elevator, -TRIM_LIMIT, TRIM_LIMIT);
  elevatorTrimToDisplay = servoCommands.trim_elevator;  // Update trim for compatibility with Lora

  elevatorTrimReceived = 0;
}

void Airplane::setAileronTrim(int8_t value) {
  if (value > 0)
    servoCommands.trim_aileron += TRIM_STEP;
  else if (value < 0)
    servoCommands.trim_aileron -= TRIM_STEP;
  else
    return;

  servoCommands.trim_aileron = constrain(servoCommands.trim_aileron, -TRIM_LIMIT, TRIM_LIMIT);

  aileronTrimToDisplay = servoCommands.trim_aileron;
  aileronTrimReceived = 0;
}

void Airplane::setFlaps(uint8_t value) {
  servoCommands.flaps = constrain(value, -TRIM_LIMIT, TRIM_LIMIT);
  flapsToDisplay = servoCommands.flaps;
}

void Airplane::resetAileronTrim() {
  servoCommands.trim_aileron = 0;
  aileronTrimToDisplay = servoCommands.trim_aileron;  // Update trim for compatibility with Lora
  aileronTrimReceived = 0;
}

void Airplane::resetElevatorTrim() {
  servoCommands.trim_elevator = 0;
  elevatorTrimToDisplay = servoCommands.trim_elevator;  // Update trim for compatibility with Lora
}

void Airplane::setLandingAirbrake(bool active) {
  servoCommands.landingAirbrake = active;
}

void Airplane::resetToSafeDefaults() {
  servoCommands.engine = 0;               // Engine off
  servoCommands.roll = 90;                // Neutral
  servoCommands.elevators = 90;           // Neutral
  servoCommands.rudder = 90;              // Neutral
  servoCommands.trim_elevator = 0;        // No trim
  servoCommands.trim_aileron = 0;         // No trim
  servoCommands.flaps = 0;                // No flaps
  servoCommands.landingAirbrake = false;  // Airbrake off

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
  Serial.printf("🎮 Controls - Engine: %d, Roll: %d, Elevators: %d, Rudder: %d, Slave: %s\n", servoCommands.engine, servoCommands.roll,
                servoCommands.elevators, servoCommands.rudder, slaveHealthy ? "✅" : "❌");
}

String Airplane::getStatusString() {
  return String("Master OK - Slave: ") + (slaveHealthy ? "Healthy" : "Disconnected");
}

void Airplane::writeToServos() {
  // Engine
  engineServos.write(constrain(servoCommands.engine, 0, 180));

  // Ailerons
  int targetRollForServo = servoCommands.roll;
  bool shouldApplyFlaps = abs(targetRollForServo - 90) < 10;

  targetRollForServo += servoCommands.trim_aileron;
  targetRollForServo += shouldApplyFlaps ? servoCommands.flaps * FLAP_ANGLE : 0;

  if (servoCommands.landingAirbrake)
    targetRollForServo = 0;

  rollLeftMotorServo.write(constrain(180 - targetRollForServo, 0, 180));  // Left aileron
  rollRightMotorServo.write(constrain(targetRollForServo, 0, 180));       // Right aileron

  // Elevators
  elevationLeftMotorServo.write(constrain(servoCommands.elevators + servoCommands.trim_elevator, 0, 180));
  elevationRightMotorServo.write(
      constrain(180 - servoCommands.elevators - servoCommands.trim_elevator, 0, 180));  // Right elevator inverted

  // Rudder
  rudderMotorServo.write(constrain(servoCommands.rudder, 0, 180));

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
