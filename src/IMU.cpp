#include "IMU.h"

// Static members
IMU* IMU::instance = nullptr;

#ifndef IMU_USE_POLLING_MODE
volatile bool mpuInterrupt = false;

// Static interrupt handler
void IRAM_ATTR IMU::dmpDataReady() {
  mpuInterrupt = true;
}
#endif

// Singleton constructor
IMU::IMU() : dmpReady(false), mpuIntStatus(0), devStatus(0), packetSize(42), fifoCount(0), calibrated(false), lastUpdate(0) {
  // Initialize data structure
  resetData();

  // Initialize calibration offsets
  for (int i = 0; i < 3; i++) {
    gyroOffsets[i] = 0;
    accelOffsets[i] = 0;
  }
}

IMU& IMU::getInstance() {
  if (instance == nullptr) {
    instance = new IMU();
  }
  return *instance;
}

bool IMU::initialize() {
  Serial.println("🔄 Initializing IMU...");

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock

  // Initialize device
  mpu.initialize();

  // Verify connection
  if (!testConnection()) {
    Serial.println("❌ MPU6050 connection failed");
    return false;
  }

  Serial.println("✅ MPU6050 connection successful");

  // Load DMP firmware
  Serial.println("🔄 Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  // Supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  // Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println("🎯 IMU calibration complete");

    // Turn on the DMP, now that it's ready
    Serial.println("🔄 Enabling DMP...");
    mpu.setDMPEnabled(true);

#ifdef IMU_USE_POLLING_MODE
    // Using polling mode instead of interrupts
    Serial.println("📊 IMU configured for polling mode (no interrupt pin needed)");
#else
    // Enable Arduino interrupt detection
    Serial.print("🔌 Enabling interrupt detection (Arduino external interrupt ");
    Serial.print(digitalPinToInterrupt(IMU_INTERRUPT_PIN));
    Serial.println(")...");

    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), dmpDataReady, RISING);
#endif

    mpuIntStatus = mpu.getIntStatus();

    // Set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println("✅ DMP ready! Starting data acquisition...");
    dmpReady = true;

    // Get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    calibrated = true;
    return true;
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    Serial.print("❌ DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    return false;
  }
}

bool IMU::testConnection() {
  return mpu.testConnection();
}

void IMU::calibrate() {
  Serial.println("🎯 Starting IMU calibration...");
  Serial.println("⚠️ Keep the aircraft level and stationary!");

  delay(3000);  // Give user time to position aircraft

  // Reset offsets
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  // Calibrate accelerometer and gyroscope
  Serial.println("🔄 Calibrating accelerometer...");
  mpu.CalibrateAccel(6);

  Serial.println("🔄 Calibrating gyroscope...");
  mpu.CalibrateGyro(6);

  // Store the offsets
  accelOffsets[0] = mpu.getXAccelOffset();
  accelOffsets[1] = mpu.getYAccelOffset();
  accelOffsets[2] = mpu.getZAccelOffset();

  gyroOffsets[0] = mpu.getXGyroOffset();
  gyroOffsets[1] = mpu.getYGyroOffset();
  gyroOffsets[2] = mpu.getZGyroOffset();

  calibrated = true;

  Serial.println("✅ IMU calibration complete!");
  Serial.printf("📊 Accel offsets: X=%d, Y=%d, Z=%d\n", accelOffsets[0], accelOffsets[1], accelOffsets[2]);
  Serial.printf("📊 Gyro offsets: X=%d, Y=%d, Z=%d\n", gyroOffsets[0], gyroOffsets[1], gyroOffsets[2]);
}

bool IMU::update() {
  if (!dmpReady)
    return false;

#ifdef IMU_USE_POLLING_MODE
  // Polling mode - check FIFO directly without relying on interrupt
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  // Check for FIFO overflow
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // Reset FIFO and clear all pending data
    mpu.resetFIFO();
    fifoCount = 0;
    Serial.println("⚠️ FIFO overflow - cleared (polling mode)");
    return false;
  }

  // Check if we have sufficient data
  if (fifoCount < packetSize) {
    return false;  // Not enough data yet
  }

#else
  // Interrupt mode (original code)
  // Early exit if no interrupt and insufficient data
  if (!mpuInterrupt && fifoCount < packetSize) {
    return false;
  }

  // Reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // Get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // Check for overflow - handle multiple packets efficiently
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // Reset FIFO and clear all pending data
    mpu.resetFIFO();
    fifoCount = 0;
    Serial.println("⚠️ FIFO overflow - cleared");
    return false;
  }
#endif

  // Process all available packets to prevent backup
  bool dataProcessed = false;
  while (fifoCount >= packetSize) {
    // Read packet immediately without waiting
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Only process the most recent packet to save CPU
    if (fifoCount < packetSize) {
      updateIMUData();
      dataProcessed = true;
    }

    // Update FIFO count for next iteration
    if (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }

  return dataProcessed;
}
void IMU::updateIMUData() {
  // Get quaternion values in easy matrix form: w, x, y, z
  mpu.dmpGetQuaternion(&q, fifoBuffer);

  // Get gravity vector and yaw/pitch/roll angles in one call
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // Convert angles from radians to degrees (more efficient than separate calls)
  currentData.yaw = ypr[0] * 57.2958f;  // 180/PI = 57.2958
  currentData.pitch = ypr[1] * 57.2958f;
  currentData.roll = ypr[2] * 57.2958f;

  // Calculate angular rates using efficient delta calculation
  static float lastRoll = 0, lastPitch = 0, lastYaw = 0;
  static unsigned long lastTime = 0;

  unsigned long currentTime = millis();

  if (lastTime > 0) {
    float deltaTime = (currentTime - lastTime) * 0.001f;  // Convert to seconds (more efficient)

    if (deltaTime > 0.005f) {                 // Only update if enough time has passed (5ms min)
      float invDeltaTime = 1.0f / deltaTime;  // Calculate once and reuse

      currentData.rollRate = (currentData.roll - lastRoll) * invDeltaTime;
      currentData.pitchRate = (currentData.pitch - lastPitch) * invDeltaTime;
      currentData.yawRate = (currentData.yaw - lastYaw) * invDeltaTime;

      // Apply deadband efficiently
      if (fabsf(currentData.rollRate) < GYRO_DEADBAND)
        currentData.rollRate = 0;
      if (fabsf(currentData.pitchRate) < GYRO_DEADBAND)
        currentData.pitchRate = 0;
      if (fabsf(currentData.yawRate) < GYRO_DEADBAND)
        currentData.yawRate = 0;

      // Update previous values
      lastRoll = currentData.roll;
      lastPitch = currentData.pitch;
      lastYaw = currentData.yaw;
      lastTime = currentTime;
    }
  } else {
    lastTime = currentTime;
  }

  // Get accelerometer data efficiently (only when needed)
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

  // Convert acceleration more efficiently
  static const float accelScale = 9.81f / 16384.0f;  // Precalculate constant
  currentData.accelX = aaWorld.x * accelScale;
  currentData.accelY = aaWorld.y * accelScale;
  currentData.accelZ = aaWorld.z * accelScale;

  // Apply deadband efficiently
  if (fabsf(currentData.accelX) < ACCEL_DEADBAND)
    currentData.accelX = 0;
  if (fabsf(currentData.accelY) < ACCEL_DEADBAND)
    currentData.accelY = 0;
  if (fabsf(currentData.accelZ) < ACCEL_DEADBAND)
    currentData.accelZ = 0;

  // Update metadata
  currentData.dataValid = validateData();
  currentData.timestamp = currentTime;
  lastUpdate = currentTime;
}

bool IMU::validateData() {
  // Use more efficient comparisons and combine checks
  return (fabsf(currentData.roll) <= 180.0f && fabsf(currentData.pitch) <= 90.0f && fabsf(currentData.yaw) <= 180.0f &&
          isfinite(currentData.roll) && isfinite(currentData.pitch) && isfinite(currentData.yaw) && isfinite(currentData.accelX) &&
          isfinite(currentData.accelY) && isfinite(currentData.accelZ));
}

void IMU::resetData() {
  currentData.roll = 0;
  currentData.pitch = 0;
  currentData.yaw = 0;
  currentData.rollRate = 0;
  currentData.pitchRate = 0;
  currentData.yawRate = 0;
  currentData.accelX = 0;
  currentData.accelY = 0;
  currentData.accelZ = 0;
  currentData.dataValid = false;
  currentData.timestamp = 0;
}

bool IMU::isDMPReady() const {
  return dmpReady;
}

bool IMU::hasNewData() const {
  return currentData.timestamp > lastUpdate;
}

IMUData IMU::getCurrentData() const {
  return currentData;
}

float IMU::getRoll() const {
  return currentData.roll;
}

float IMU::getPitch() const {
  return currentData.pitch;
}

float IMU::getYaw() const {
  return currentData.yaw;
}

float IMU::getRollRate() const {
  return currentData.rollRate;
}

float IMU::getPitchRate() const {
  return currentData.pitchRate;
}

float IMU::getYawRate() const {
  return currentData.yawRate;
}

float IMU::getAccelX() const {
  return currentData.accelX;
}

float IMU::getAccelY() const {
  return currentData.accelY;
}

float IMU::getAccelZ() const {
  return currentData.accelZ;
}

bool IMU::isDataValid() const {
  return currentData.dataValid;
}

unsigned long IMU::getLastUpdateTime() const {
  return currentData.timestamp;
}

String IMU::getStatusString() const {
  String status = "IMU Status:\n";
  status += "DMP Ready: " + String(dmpReady ? "✅" : "❌") + "\n";
  status += "Calibrated: " + String(calibrated ? "✅" : "❌") + "\n";
  status += "Data Valid: " + String(currentData.dataValid ? "✅" : "❌") + "\n";
  status += "Last Update: " + String(millis() - currentData.timestamp) + "ms ago\n";
  status += "Roll: " + String(currentData.roll, 2) + "°\n";
  status += "Pitch: " + String(currentData.pitch, 2) + "°\n";
  status += "Yaw: " + String(currentData.yaw, 2) + "°\n";
  return status;
}

void IMU::printDiagnostics() const {
  static unsigned long lastDiagnostics = 0;
  static int updateCount = 0;
  static int overflowCount = 0;

  updateCount++;

  // Print detailed diagnostics every 5 seconds
  if (millis() - lastDiagnostics > 5000) {
    Serial.println("=== 🛩️ IMU Performance Diagnostics ===");
    Serial.println("DMP Ready: " + String(dmpReady ? "✅" : "❌"));
    Serial.println("Calibrated: " + String(calibrated ? "✅" : "❌"));
    Serial.println("Data Valid: " + String(currentData.dataValid ? "✅" : "❌"));
    Serial.printf("Update Rate: %.1f Hz\n", updateCount / 5.0f);
    Serial.printf("FIFO Count: %d bytes\n", fifoCount);
    Serial.printf("Packet Size: %d bytes\n", packetSize);
    Serial.printf("Orientation: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°\n", currentData.roll, currentData.pitch, currentData.yaw);
    Serial.printf("Angular Rates: Roll=%.2f°/s, Pitch=%.2f°/s, Yaw=%.2f°/s\n", currentData.rollRate, currentData.pitchRate,
                  currentData.yawRate);
    Serial.printf("Acceleration: X=%.2fm/s², Y=%.2fm/s², Z=%.2fm/s²\n", currentData.accelX, currentData.accelY, currentData.accelZ);
    Serial.println("Last Update: " + String(millis() - currentData.timestamp) + "ms ago");
    Serial.println("FIFO Overflows: " + String(overflowCount) + " in last 5s");
    Serial.println("🔧 Tip: Keep update rate between 20-50Hz for optimal performance");
    Serial.println("========================================");

    updateCount = 0;
    overflowCount = 0;
    lastDiagnostics = millis();
  }
}

void IMU::setUpdateRate(uint16_t hz) {
  // Note: DMP output rate is fixed at ~200Hz
  // This would require additional filtering/decimation to implement
  Serial.println("⚠️ Update rate change not implemented for DMP mode");
}

void IMU::resetOrientation() {
  Serial.println("🔄 Resetting IMU orientation...");
  mpu.resetFIFO();
  resetData();
}

bool IMU::isCalibrated() const {
  return calibrated;
}

void IMU::saveCalibration() {
  // TODO: Implement EEPROM/preferences storage for calibration data
  Serial.println("⚠️ Calibration save not implemented");
}

bool IMU::loadCalibration() {
  // TODO: Implement EEPROM/preferences loading for calibration data
  Serial.println("⚠️ Calibration load not implemented");
  return false;
}

void IMU::resetCalibration() {
  calibrated = false;
  for (int i = 0; i < 3; i++) {
    gyroOffsets[i] = 0;
    accelOffsets[i] = 0;
  }
  Serial.println("🔄 IMU calibration reset");
}

void IMU::checkPerformance() const {
  static unsigned long lastCheck = 0;
  static int fifoOverflows = 0;

  if (millis() - lastCheck > 10000) {  // Check every 10 seconds
    Serial.println("📊 IMU Performance Check:");

    if (fifoCount > packetSize * 3) {
      Serial.println("⚠️ FIFO buffer is getting full - consider reducing update frequency");
    }

    if (millis() - currentData.timestamp > 50) {
      Serial.println("⚠️ IMU data is stale - check if update() is being called regularly");
    }

    unsigned long timeSinceUpdate = millis() - lastUpdate;
    if (timeSinceUpdate > 100) {
      Serial.println("💡 Consider calling IMU update more frequently for smoother data");
    } else if (timeSinceUpdate < 20) {
      Serial.println("💡 IMU is being updated very frequently - this may cause FIFO overflow");
    }

    lastCheck = millis();
  }
}

void IMU::handleInterrupt() {
  // This can be called from the main loop to handle interrupt processing
  // Currently, the interrupt just sets a flag that is checked in update()
}
