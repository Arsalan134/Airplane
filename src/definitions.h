#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <avr/pgmspace.h>
#include <printf.h>
#include "RF24.h"
#include "Wire.h"
#include "printf.h"

RF24 radio(7, 8);
MPU6050 mpu(Wire);

Servo rollLeftMotor;
Servo rollRightMotor;
Servo pitchMotor;
Servo yawMotor;

Servo engine;

boolean timeout = false;

byte addresses[][6] = {"1Node", "2Node"};

byte transmitData[1];
byte recievedData[4];

unsigned long lastRecievedTime = millis();
unsigned long currentTime = millis();
unsigned long timeoutMilliSeconds = 500;
unsigned long elapsedTime = 0;

byte rollValue = 90;
byte pitchValue = 90;
byte yawValue = 90;

float angleX = 0;
float angleY = 0;
float angleZ = 0;

byte correctedRollAngle = 0;
byte correctedPitchAngle = 0;

#define multiplierACS 3.0

/*
Pinout

D0
D1      Interrupt IMU MPU6050
D2
D3  ~
D4      Servo Left Roll
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

#define rollServoLeftPin 3
#define pitchServoPin 5
#define rollServoRightPin 6
#define motorPin 9
#define yawServoPin 10

#define minThrottle 1000
#define maxThrottle 2000

// Indices in recieve payload
#define rollIndex 0
#define pitchIndex 1
#define yawIndex 2
#define throttleIndex 3

// Indices in transmit payload
#define batteryIndex 0

#define delayTime 5

#define degreesOfFreedomAilerons 90

#define pitchBias 0
#define RollRightBias -5
#define RollLeftBias 20

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

/** @brief
 * Active Control System
 */
void ACS();

/**
 * @brief
 * 0 Degrees is a center. Positive values are to the right. Negative are to the
 * left. Passing 0 will  end up in the same valve positions
 *
 * @param byAmount
 * Degrees
 */
void roll(byte byAmount);

void pitch(byte byAmount);

void yaw(byte byAmount);

void imuSetup();
void radioSetup();
void servoSetup();
