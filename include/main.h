#include <Arduino.h>
#include "common.h"

// ⚡ Dual-Core Task Management
void FlightControlTaskCode(void* pvParameters);
void CommunicationTaskCode(void* pvParameters);

// 💾 SD Card
#include "SD-Card.h"
void setupSD();

// 🖥️ Display
#include "Display.h"
int frameCount = 1;
int overlaysCount = 1;
void setupDisplay();

// 📡 Lora Communication
#include <LoRa.h>
const long frequency = 915E6;  // LoRa Frequency 📻
void setupRadio();
void loraLoop();
uint8_t simple_checksum(const uint8_t* data, size_t len);
void LoRa_rxMode();
void LoRa_txMode();
void onReceive(int packetSize);
void onTxDone();

boolean timeout = false;

/** @brief
 * 🛩️ Active Control System
 */
void ACS();

void sendDataToAirplane();
void printTaskInfo();

// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps612.h"
// #include "RF24.h"

// #include <Adafruit_DPS310.h>
// #include <SD.h>
// #include "bmm150.h"
// #include "bmm150_defs.h"
// #endif

// #ifdef isLeonardo
// Magnetometer BMM150
// BMM150 bmm;
// bmm150_mag_data bmm150_value;
// bmm150_mag_data bmm150_value_offset;
// short headingDegrees = 0;

// Barometer DPS310
// Adafruit_DPS310 dps;
// Adafruit_Sensor* dps_temp = dps.getTemperatureSensor();
// Adafruit_Sensor* dps_pressure = dps.getPressureSensor();
// #endif

// MPU6050 mpu;
// uint8_t fifoBuffer[64];  // FIFO storage buffer

// RF24 radio(12, 8);

// Servo rollLeftMotor;

// Servo pitchMotor;
// Servo yawMotor;

// Sd2Card carda
// SdVolume volume;
// SdFile root;

// // Used by IMU to correct airplane
// short correctedRollAmount = 0;
// short correctedPitchAmount = 0;

// #define multiplierRollACS 3.0
// #define multiplierPitchACS 3.0

// #define degreesOfFreedomAilerons 90

// #define RollLeftBias -20  // negative pitchs down
// #define RollRightBias 20  // negative pitchs down

// float pitchBias = 0;

// Setup
// void calibrate();
// void imuSetup();
// void radioSetup();
// void servoSetup();
// void magnetometerSetup();
// void barometerSetup();

// Loop
// void IMULoop();
// void magnetometerLoop();
// void barometerLoop();

// Radio
// void transmit();
// void radioLoop();
// void printTransmissionData();
// void printRecievedData();

// void readSensors();
// void makeStuffWithRecievedData();

/** @brief
 * Reset all values to default values
 * Throttle down to 0
 * Servos to center position
 */
// void resetAirplaneToDefaults();

/**
 * @brief
 * 0 Degrees is a center. Sending '0' will end up in the default flaps positions
 *
 * @param byDegrees
 * Degrees
 */
// void roll(byte degrees);
// void rollBy(byte byDegrees);

// void pitch(byte degrees);
// void pitchBy(byte byDegrees);

// void yaw(byte byAmount);

// void lostRadio();

// void engineOff();

// void saveToFile(File file);
// void setupSDCard();
// void printCardInfo();
// void printDirectory(File dir, int numTabs);
