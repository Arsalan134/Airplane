
#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>

// #include "nRF24L01.h"
// #include "RF24.h"
// #include "printf.h"

#include <avr/pgmspace.h>

// #include <RF24Network.h>
// #include <RF24.h>
// #include "printf.h"

#include "RF24.h"
#include "printf.h"
#include <printf.h>

#include <definitions.h>

short delayTime = 100;

//-----------------------------------------

RF24 radio(7, 8);

Servo rollLeft;
Servo rollRight;
Servo pitch;
Servo yaw;
Servo motor;

boolean timeout = false;

byte addresses[][6] = {"1Node", "2Node"};

byte transmitData[1];
byte recievedData[4];

unsigned long lastRecievedTime = millis();
unsigned long currentTime = millis();
unsigned long timeoutMilliSeconds = 500;
unsigned long elapsedTime = 0;

void setup() {

  Serial.begin(57600);

  delay(2000);

  printf_begin();

  delay(1000);

  radio.begin();
   
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(112);
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  delay(1000);

  radio.printDetails();

  // Attach servos
  motor.attach(motorPin, minThrottle, maxThrottle);
  rollLeft.attach(rollServoLeftPin);
  rollRight.attach(rollServoRightPin);
  pitch.attach(pitchServoPin);
  yaw.attach(yawServoPin);

  Serial.println("Done with setup!");
}

void printTransmitData() {
  Serial.print("Sent: \t\t");
  for (unsigned long i = 0; i < sizeof(transmitData) / sizeof(transmitData[0]);
       i++) {
    Serial.print(transmitData[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void printRecievedData() {
  // Serial.print("Recieved: \t");
  // for (unsigned long i = 0; i < sizeof(recievedData) /
  // sizeof(recievedData[0]); i++) {
  Serial.print("Throttle ");
  Serial.println(recievedData[throttleIndex]);

  Serial.print("Yaw ");
  Serial.println(recievedData[yawIndex]);

  Serial.print("Pitch ");
  Serial.println(recievedData[pitchIndex]);

  Serial.print("Roll ");
  Serial.println(recievedData[rollIndex]);

  Serial.println();
}

void readSensors() { transmitData[batteryIndex] = 0; }

void transmit() {
  Serial.println("Transmitting...");
  radio.stopListening();
  radio.write(&transmitData, sizeof(transmitData));
  radio.startListening();
}

void makeStuffWithRecievedData() {
  byte rollValue = map(recievedData[rollIndex], 0, 255,
                       90 - degreeOfFreedom / 2, 90 + degreeOfFreedom / 2);
  byte pitchValue = map(recievedData[pitchIndex], 0, 255,
                        90 - degreeOfFreedom / 2, 90 + degreeOfFreedom / 2);
  byte yawValue = map(recievedData[yawIndex], 0, 255, 0, 180);

  yawValue = constrain(yawValue, 55, 160);

  rollLeft.write(rollValue);
  rollRight.write(rollValue);
  pitch.write(pitchValue);
  yaw.write(yawValue);
  motor.write(recievedData[throttleIndex]);
}

void reset() {
  recievedData[rollIndex] = 127;
  recievedData[pitchIndex] = 127;
  recievedData[yawIndex] = 127;
}

// void printPressureAndTemp() {
//   Serial.print(F("Temperature = "));
//   Serial.print(bmp.readTemperature());
//   Serial.println("C");

//   Serial.print(F("Pressure = "));
//   Serial.print(bmp.readPressure());
//   Serial.println(" Pa");

//   Serial.print(F("Approx altitude = "));
//   Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
//   Serial.println(" m");

//   Serial.println();
// }

/// Active Control System
void ACS() {
  recievedData[throttleIndex] = 0;
  reset();

  // comment reset fundtion call

  /*
   * 1. Read Accelerometer data
   * 2. Move Wings
   */
}

void loop() {
  // printMotion();  // working
  // printCamera();  // working
  // printPressureAndTemp();
  // readSensors();
  // transmit(); maybe via bluetooth to an iPhone

  if (radio.available()) {

    //   while (radio.available()) {
    radio.read(&recievedData, sizeof(recievedData));
    // }
    lastRecievedTime = millis();

    // printRecievedData();
  }

  currentTime = millis();
  elapsedTime = currentTime - lastRecievedTime;

  // Activate ACS when signal is lost
  if (elapsedTime >= timeoutMilliSeconds) {

    ACS();
    // Read data from 33 Sense
    // Serial.println("Signal Loss");
  }
  // else {
  //   Serial.print("Elapsed: ");
  //   Serial.println(elapsedTime);
  // }

  makeStuffWithRecievedData();
}