#include "AirplaneMaster.h"
#include "common.h"

// Static instance initialization
AirplaneMaster* AirplaneMaster::instance = nullptr;

AirplaneMaster::AirplaneMaster() {
  // Initialize default values
  targetRoll = 90;       // Neutral position
  targetRudder = 90;     // Neutral position
  targetElevators = 90;  // Neutral position
  targetEngine = 0;      // Engine off

  elevatorTrim = 0;
  aileronTrim = 0;
  flaps = 0;
  landingAirbrake = false;

  lastReceivedTime = 0;
  lastI2CCommand = 0;
  connectionActive = false;
  slaveHealthy = false;
  newFlightDataAvailable = false;

  currentFlightMode = FlightMode::MANUAL;

  // Initialize flight data
  memset(&latestFlightData, 0, sizeof(latestFlightData));
}

AirplaneMaster& AirplaneMaster::getInstance() {
  if (instance == nullptr) {
    instance = new AirplaneMaster();
  }
  return *instance;
}

void AirplaneMaster::initialize() {
  Serial.println("🛩️ Initializing Airplane Master Controller");

  // Initialize I2C as master
  Wire.begin();
  Wire.setClock(100000);  // 100kHz for reliable communication

  Serial.println("📡 I2C Master initialized");

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

void AirplaneMaster::update() {
  unsigned long currentTime = millis();

  // Send servo commands every 50ms (20Hz)
  if (currentTime - lastI2CCommand >= 50) {
    bool success = sendServoCommands();
    if (success) {
      slaveHealthy = true;
    } else {
      slaveHealthy = false;
      Serial.println("⚠️ I2C communication failed");
    }
    lastI2CCommand = currentTime;
  }

  // Request flight data every 100ms (10Hz)
  static unsigned long lastDataRequest = 0;
  if (currentTime - lastDataRequest >= 100) {
    requestFlightData();
    lastDataRequest = currentTime;
  }

  // Check connection timeout
  checkConnectionTimeout();

  // Log control changes periodically
  static unsigned long lastLog = 0;
  if (currentTime - lastLog >= 1000) {
    logControlChanges();
    lastLog = currentTime;
  }
}

bool AirplaneMaster::sendServoCommands() {
  ServoCommandPacket packet;
  packet.header = SERVO_COMMAND;
  packet.engine = targetEngine;
  packet.rollLeft = targetRoll;
  packet.elevatorLeft = targetElevators + elevatorTrim;
  packet.elevatorRight = targetElevators + elevatorTrim;
  packet.rudder = targetRudder;
  packet.trim_elevator = elevatorTrim;
  packet.trim_aileron = aileronTrim;
  packet.flaps = flaps;
  packet.landingAirbrake = landingAirbrake;
  packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet));

  Wire.beginTransmission(SERVO_CONTROLLER_ADDRESS);
  Wire.write((uint8_t*)&packet, sizeof(packet));
  uint8_t result = Wire.endTransmission();

  return (result == 0);  // 0 = success
}

bool AirplaneMaster::requestFlightData() {
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

bool AirplaneMaster::requestSlaveStatus() {
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

// Implementation of other methods...
void AirplaneMaster::setThrottle(uint8_t value) {
  if (isValidControlValue(value)) {
    targetEngine = constrain(value, 0, 180);
    updateLastReceivedTime();
  }
}

void AirplaneMaster::setRudder(uint8_t value) {
  if (isValidControlValue(value)) {
    targetRudder = constrain(value, 0, 180);
    updateLastReceivedTime();
  }
}

void AirplaneMaster::setElevators(uint8_t value) {
  if (isValidControlValue(value)) {
    targetElevators = constrain(value, 0, 180);
    updateLastReceivedTime();
  }
}

void AirplaneMaster::setAilerons(uint8_t value) {
  if (isValidControlValue(value)) {
    targetRoll = constrain(value, 0, 180);
    updateLastReceivedTime();
  }
}

void AirplaneMaster::resetToSafeDefaults() {
  targetEngine = 0;         // Engine off
  targetRoll = 90;          // Neutral
  targetElevators = 90;     // Neutral
  targetRudder = 90;        // Neutral
  elevatorTrim = 0;         // No trim
  aileronTrim = 0;          // No trim
  flaps = 0;                // No flaps
  landingAirbrake = false;  // Airbrake off

  Serial.println("🔒 Flight controls reset to safe defaults");
}

bool AirplaneMaster::getFlightData(FlightDataPacket& data) {
  if (newFlightDataAvailable) {
    data = latestFlightData;
    newFlightDataAvailable = false;
    return true;
  }
  return false;
}

float AirplaneMaster::getCurrentRoll() const {
  return latestFlightData.roll;
}

float AirplaneMaster::getCurrentPitch() const {
  return latestFlightData.pitch;
}

float AirplaneMaster::getCurrentYaw() const {
  return latestFlightData.yaw;
}

float AirplaneMaster::getCurrentAltitude() const {
  return latestFlightData.altitude;
}

bool AirplaneMaster::isSlaveHealthy() const {
  return slaveHealthy;
}

void AirplaneMaster::checkConnectionTimeout() {
  if (connectionActive && (millis() - lastReceivedTime > CONNECTION_TIMEOUT)) {
    Serial.println("⚠️ Connection timeout - activating emergency shutdown");
    emergencyShutdown();
    connectionActive = false;
  }
}

void AirplaneMaster::emergencyShutdown() {
  Serial.println("🚨 EMERGENCY SHUTDOWN ACTIVATED");
  resetToSafeDefaults();
  // Send emergency command multiple times
  for (int i = 0; i < 3; i++) {
    sendServoCommands();
    delay(10);
  }
}

bool AirplaneMaster::isValidControlValue(uint8_t value) {
  return (value <= 180);  // Basic range check
}

void AirplaneMaster::updateLastReceivedTime() {
  lastReceivedTime = millis();
  if (!connectionActive) {
    connectionActive = true;
    Serial.println("📡 Connection restored");
  }
}

void AirplaneMaster::logControlChanges() {
  Serial.printf("🎮 Controls - Engine: %d, Roll: %d, Elevator: %d, Rudder: %d, Slave: %s\n", targetEngine, targetRoll, targetElevators,
                targetRudder, slaveHealthy ? "✅" : "❌");
}

String AirplaneMaster::getStatusString() {
  return String("Master OK - Slave: ") + (slaveHealthy ? "Healthy" : "Disconnected");
}
