

#include <Arduino.h>

#include "RF24.h"
#include "printf.h"
#include <SPI.h>
#include <Servo.h>
#include <avr/pgmspace.h>

#include <printf.h>

short delayTime = 20;

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

// PINS------------------------------------
//   ~   ~ ~         ~ ~  ~     ~     ~
// 2 3 4 5 6 7  8    9 10 11    12    13
// I + - - - ce csn  - -  mosi  miso  +

//  I for Interrupt

/*

D0
D1      Interrupt
D2
D3  ~   SCL I2C
D4      Left Roll
D5  ~   Pitch
D6  ~   Right Roll
D7      CE
D8      CSN
D9  ~   Motor
D10 ~   Yaw
D11 ~   MOSI ? FREE
D12 ~   MISO ? FREE
D13 ~   === FREE ===

*/

// ANALOG
// short vibroPin = A0;
// #define photoresistorPin A1 // no need because 33 sense has camera

// DIGITAL

// PWM

// look from Top

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
