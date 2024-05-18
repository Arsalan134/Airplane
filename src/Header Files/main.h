#include <Arduino.h>
// #include <Servo.h>
#include "Common/common.h"

// SD Card
#include "SD-Card.h"
void setupSD();

// Display
#include "Display.h"
int frameCount = 1;
int overlaysCount = 1;
void setupDisplay();

// Lora
#include <LoRa.h>
const long frequency = 915E6;  // LoRa Frequency
boolean runEvery(unsigned long interval);
void setupRadio();
void loraLoop();
void LoRa_rxMode();
void LoRa_txMode();
void LoRa_sendMessage(String message);
void onReceive(int packetSize);
void onTxDone();

// #include <Servo.h>
// #include <printf.h>
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps612.h"
// #include "RF24.h"

// #define isLeonardo
// #define isNano

// #ifdef isLeonardo
// #include <Adafruit_DPS310.h>
// #include <SD.h>
// #include "bmm150.h"
// #include "bmm150_defs.h"
// #endif

// orientation/motion vars
// Quaternion q;         // [w, x, y, z]         quaternion container
// VectorInt16 aa;       // [x, y, z]            accel sensor measurements
// VectorInt16 gy;       // [x, y, z]            gyro sensor measurements
// VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
// VectorFloat gravity;  // [x, y, z]            gravity vector
// float euler[3];       // [psi, theta, phi]    Euler angle container
// float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

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
// Servo rollRightMotor;
// Servo pitchMotor;
// Servo yawMotor;

// Servo engine; uncomment this

// Sd2Card carda
// SdVolume volume;
// SdFile root;

boolean timeout = false;

// byte addresses[2][6] = {"1Node", "2Node"};

// byte transmitData[1];
// byte recievedData[7];

// unsigned long lastRecievedTime = 0;

// #define timeoutInMilliSeconds 500

// Inertial Measurment Unit data
// short currentRollValue = 0;
// short currentPitchValue = 0;

// // Used by IMU to correct airplane
// short correctedRollAmount = 0;
// short correctedPitchAmount = 0;

// #define multiplierRollACS 3.0
// #define multiplierPitchACS 3.0

// #define INTERRUPT_PIN 2

// #define pitchServoPin 5
// #define rollServoLeftPin 6
// #define buzzerPin 7
#define motorPin 10
// #define rollServoRightPin 10
// #define sdCardPin 11
// #define yawServoPin 13

// Indices in recieve payload
// #define rollIndex 0
// #define pitchIndex 1
// #define yawIndex 2
// #define throttleIndex 3
// #define autopilotIsOnIndex 4
// #define pitchBiasIndex 5
// #define calibrateIndex 6

// Indices in transmit payload
// #define batteryIndex 0

// #define delayTime 5

// #define degreesOfFreedomAilerons 90

// #define RollLeftBias -20  // negative pitchs down
// #define RollRightBias 20  // negative pitchs down

// float pitchBias = 0;

// Setup
void calibrate();
void imuSetup();
void radioSetup();
void servoSetup();
void magnetometerSetup();
void barometerSetup();

// Loop
void IMULoop();
void magnetometerLoop();
void barometerLoop();

// Radio
void transmit();
void radioLoop();
void printTransmissionData();
void printRecievedData();

void readSensors();
void makeStuffWithRecievedData();

/** @brief
 * Reset all values to default values
 * Throttle down to 0
 * Servos to center position
 */
void resetAirplaneToDefaults();

/** @brief
 * Active Control System
 */
void ACS();

/**
 * @brief
 * 0 Degrees is a center. Sending '0' will end up in the default flaps positions
 *
 * @param byDegrees
 * Degrees
 */
void roll(byte degrees);
void rollBy(byte byDegrees);

void pitch(byte degrees);
void pitchBy(byte byDegrees);

void yaw(byte byAmount);

void lostRadio();

void engineOff();

// void saveToFile(File file);
// void setupSDCard();
// void printCardInfo();
// void printDirectory(File dir, int numTabs);
