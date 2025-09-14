#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

// IMU Configuration
#define MPU6050_ADDRESS 0x68
// #define IMU_INTERRUPT_PIN 2  // Disabled - pin 2 used for servo
#define IMU_USE_POLLING_MODE 1  // Use polling instead of interrupts
#define IMU_UPDATE_RATE_HZ 50   // Reduced from 100 to prevent FIFO overflow

// Calibration parameters
#define CALIBRATION_SAMPLES 1000
#define GYRO_DEADBAND 2.0f   // degrees/second
#define ACCEL_DEADBAND 0.1f  // m/s²

// Performance optimization settings
#define IMU_FIFO_BUFFER_SIZE 1024
#define IMU_MAX_PACKETS_PER_UPDATE 3  // Process max 3 packets per update call

struct IMUData {
  float roll;               // Roll angle in degrees (-180 to 180)
  float pitch;              // Pitch angle in degrees (-90 to 90)
  float yaw;                // Yaw angle in degrees (-180 to 180)
  float rollRate;           // Roll rate in degrees/second
  float pitchRate;          // Pitch rate in degrees/second
  float yawRate;            // Yaw rate in degrees/second
  float accelX;             // Linear acceleration X in m/s²
  float accelY;             // Linear acceleration Y in m/s²
  float accelZ;             // Linear acceleration Z in m/s²
  bool dataValid;           // True if data is valid and updated
  unsigned long timestamp;  // Timestamp of last update
};

class IMU {
 private:
  static IMU* instance;

  // MPU6050 object
  MPU6050 mpu;

  // DMP variables
  bool dmpReady;
  uint8_t mpuIntStatus;
  uint8_t devStatus;
  uint16_t packetSize;
  uint16_t fifoCount;
  uint8_t fifoBuffer[64];

  // Orientation/motion variables
  Quaternion q;         // [w, x, y, z]         quaternion container
  VectorInt16 aa;       // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;  // [x, y, z]            gravity vector
  float euler[3];       // [psi, theta, phi]    Euler angle container
  float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

  // Calibration offsets
  int16_t gyroOffsets[3];
  int16_t accelOffsets[3];
  bool calibrated;

  // Data storage
  IMUData currentData;

  // Timing
  unsigned long lastUpdate;

  // Private constructor for singleton
  IMU();

  // Delete copy constructor and assignment operator
  IMU(const IMU&) = delete;
  IMU& operator=(const IMU&) = delete;

  // Private helper methods
  void applyCalibration();
  void updateIMUData();
  bool validateData();
  void resetData();

 public:
  // Singleton instance getter
  static IMU& getInstance();

  // Initialization and setup
  bool initialize();
  bool testConnection();
  void calibrate();
  bool isDMPReady() const;

  // Data acquisition
  bool update();
  bool hasNewData() const;
  IMUData getCurrentData() const;

  // Individual data getters
  float getRoll() const;
  float getPitch() const;
  float getYaw() const;
  float getRollRate() const;
  float getPitchRate() const;
  float getYawRate() const;
  float getAccelX() const;
  float getAccelY() const;
  float getAccelZ() const;

  // Status and diagnostics
  bool isDataValid() const;
  unsigned long getLastUpdateTime() const;
  String getStatusString() const;
  void printDiagnostics() const;
  void checkPerformance() const;  // New performance monitoring method

  // Configuration
  void setUpdateRate(uint16_t hz);
  void resetOrientation();

  // Calibration management
  bool isCalibrated() const;
  void saveCalibration();
  bool loadCalibration();
  void resetCalibration();

  // Interrupt handling (static for ISR) - only needed in interrupt mode
#ifndef IMU_USE_POLLING_MODE
  static void IRAM_ATTR dmpDataReady();
#endif
  void handleInterrupt();
};

// Global interrupt flag - only needed in interrupt mode
#ifndef IMU_USE_POLLING_MODE
extern volatile bool mpuInterrupt;
#endif

#endif  // IMU_H
