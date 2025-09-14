/*
 * IMU Integration Example for TTGO LoRa32 Aircraft Controller
 *
 * This example demonstrates how to use the integrated MPU6050 IMU
 * with the I2Cdevlib-MPU6050 library in the airplane control system.
 *
 * Hardware:
 * - TTGO LoRa32 v2.1 (ESP32)
 * - MPU6050 IMU connected via I2C
 *   - VCC -> 3.3V
 *   - GND -> GND
 *   - SCL -> GPIO 22 (default I2C clock)
 *   - SDA -> GPIO 21 (default I2C data)
 *   - INT -> GPIO 2 (interrupt pin)
 *
 * Features:
 * - Real-time orientation tracking (roll, pitch, yaw)
 * - Angular velocity measurements
 * - Linear acceleration (gravity compensated)
 * - DMP (Digital Motion Processor) integration
 * - Automatic calibration
 * - Flight stabilization support
 */

#include <Arduino.h>
#include "Airplane.h"
#include "IMU.h"

void setup() {
  Serial.begin(115200);
  Serial.println("🛩️ Aircraft IMU Integration Example");

  // Get the airplane instance
  Airplane& airplane = Airplane::getInstance();

  // Initialize the airplane system (includes IMU)
  airplane.initialize();

  // Check if IMU is ready
  if (airplane.isIMUReady()) {
    Serial.println("✅ IMU is ready and operational");
  } else {
    Serial.println("❌ IMU initialization failed");
  }
}

void loop() {
  // Get the airplane instance
  Airplane& airplane = Airplane::getInstance();

  // Update the airplane systems (includes IMU)
  airplane.update();

  // Print IMU data every second
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    printIMUData(airplane);
    lastPrint = millis();
  }

  // Example of using IMU data for flight control
  demonstrateStabilization(airplane);

  delay(10);  // 100Hz update rate
}

void printIMUData(Airplane& airplane) {
  if (airplane.isIMUReady()) {
    Serial.println("=== 🧭 IMU Data ===");
    Serial.printf("Roll: %6.2f° | Pitch: %6.2f° | Yaw: %6.2f°\n", airplane.getIMURoll(), airplane.getIMUPitch(), airplane.getIMUYaw());

    Serial.printf("Roll Rate: %6.2f°/s | Pitch Rate: %6.2f°/s | Yaw Rate: %6.2f°/s\n", airplane.getIMURollRate(),
                  airplane.getIMUPitchRate(), airplane.getIMUYawRate());

    // Get full IMU data structure
    IMUData imuData;
    if (airplane.getIMUData(imuData)) {
      Serial.printf("Accel: X=%6.2f | Y=%6.2f | Z=%6.2f m/s²\n", imuData.accelX, imuData.accelY, imuData.accelZ);
      Serial.printf("Data Valid: %s | Age: %lu ms\n", imuData.dataValid ? "✅" : "❌", millis() - imuData.timestamp);
    }
    Serial.println("==================");
  } else {
    Serial.println("⚠️ IMU not ready");
  }
}

void demonstrateStabilization(Airplane& airplane) {
  // Example: Simple roll stabilization
  // This would be used in STABILITY flight mode

  if (airplane.getFlightMode() == FlightMode::STABILITY) {
    float currentRoll = airplane.getIMURoll();

    // If aircraft is banking more than 10 degrees, apply correction
    if (abs(currentRoll) > 10.0f) {
      // Calculate correction (proportional control)
      float correction = -currentRoll * 0.5f;       // 50% gain
      correction = constrain(correction, -30, 30);  // Limit to ±30 degrees

      // Apply correction through aileron control
      // Note: This would need integration with manual controls
      Serial.printf("🔧 Roll correction: %.2f° (current roll: %.2f°)\n", correction, currentRoll);

      // airplane.setRollAngle(correction); // Uncomment to apply
    }
  }
}

/*
 * Usage Examples:
 *
 * 1. Basic IMU Reading:
 *    float roll = airplane.getIMURoll();
 *    float pitch = airplane.getIMUPitch();
 *    float yaw = airplane.getIMUYaw();
 *
 * 2. Get Full IMU Data:
 *    IMUData data;
 *    if (airplane.getIMUData(data)) {
 *      // Use data.roll, data.pitch, data.yaw, etc.
 *    }
 *
 * 3. Check IMU Status:
 *    if (airplane.isIMUReady()) {
 *      // IMU is operational
 *    }
 *
 * 4. Calibrate IMU:
 *    airplane.calibrateIMU(); // Keep aircraft level during calibration
 *
 * 5. Reset Orientation:
 *    airplane.resetIMUOrientation(); // Reset to current position as zero
 *
 * 6. Flight Modes with IMU:
 *    airplane.setFlightMode(FlightMode::STABILITY); // Enable stabilization
 *    airplane.setFlightMode(FlightMode::MANUAL);    // Disable stabilization
 */
