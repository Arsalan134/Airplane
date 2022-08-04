#include <Arduino.h>

#include <Servo.h>
#include <printf.h>
#include <string.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "RF24.h"

RF24 radio(7, 8);
MPU6050 mpu;

uint8_t fifoBuffer[64];  // FIFO storage buffer
uint8_t devStatus = 0;

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 gy;       // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Magnetometer BMM150
// BMM150 bmm;
// bmm150_mag_data bmm150_value;
// bmm150_mag_data bmm150_value_offset;
// short headingDegrees = 0;

// Barometer DPS310
// Adafruit_DPS310 dps;
// Adafruit_Sensor* dps_temp = 0;
// Adafruit_Sensor* dps_pressure = 0;
// short pressure = 0;
// short temperature = 0;

Servo rollLeftMotor;
Servo rollRightMotor;
Servo pitchMotor;
Servo yawMotor;

Servo engine;

// Sd2Card card;
// SdVolume volume;
// SdFile root;

boolean timeout = false;

byte addresses[2][6] = {"1Node", "2Node"};

byte transmitData[1];
byte recievedData[7];

unsigned long lastRecievedTime = 0;

#define timeoutInMilliSeconds 500

// Inertial Measurment Unit data
short currentRollValue = 0;
short currentPitchValue = 0;

// Used by IMU to correct airplane
short correctedRollAmount = 0;
short correctedPitchAmount = 0;

#define multiplierRollACS 3.0
#define multiplierPitchACS 3.0

/*
Pinout

D0
D1
D2
D3  ~   Servo Left Roll
D4      +
D5  ~   Servo Pitch
D6  ~   Servo Right Roll
D7      Radio CE
D8      Radio CSN
D9  ~   Motor
D10 ~   Servo Yaw
D11 ~   +
D12 ~   +
D13 ~   +

*/
#define INTERRUPT_PIN 2
#define rollServoLeftPin 6
#define pitchServoPin 5
#define rollServoRightPin 3
#define motorPin 9
#define yawServoPin 10

#define sdCardPin 11

// Indices in recieve payload
#define rollIndex 0
#define pitchIndex 1
#define yawIndex 2
#define throttleIndex 3
#define autopilotIsOnIndex 4
#define pitchBiasIndex 5
#define calibrateIndex 6

// Indices in transmit payload
#define batteryIndex 0

#define delayTime 5

#define degreesOfFreedomAilerons 90

#define RollLeftBias -20  // negative pitchs down
#define RollRightBias 20  // negative pitchs down

float pitchBias = 0;

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

void transmit();

void IMULoop();
void calibrate(uint32_t timeout);

void magnetometerLoop();

void barometerLoop();

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

void imuSetup();
void radioSetup();
void servoSetup();
void magnetometerSetup();
void barometerSetup();

void dmpDataReady();

void engineOff();

// void saveToFile(File file);
void setupSDCard();
void printCardInfo();
// void printDirectory(File dir, int numTabs);
