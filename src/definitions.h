#include <Arduino.h>

#include <SPI.h>
#include <Servo.h>
#include <avr/pgmspace.h>
#include "RF24.h"
#include "printf.h"

#include <printf.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//-----------------------------------------

RF24 radio(7, 8);

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
byte pitchValue = 140;

/*
Pinout

D0
D1      Interrupt IMU MPU6050
D2      I2C SDA
D3  ~   I2C SCL
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

#define rollServoLeftPin 4
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

void printTransmitData();
void printRecievedData();

void readSensors();
void makeStuffWithRecievedData();
void reset();
void transmit();
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

void imu();
void imuSetup();
