#include <Adafruit_BMP280.h> // pressure and temperature
#include <Arduino.h>
// #include <Arduino_APDS9960.h> // camera
// #include <Arduino_LSM9DS1.h>  // IMU
#include <SPI.h>
#include <Servo.h>

#include "RF24.h"
#include "printf.h"

#include <definitions.h>

short delayTime = 100;

//-----------------------------------------

RF24 radio(7, 8);

Servo rollLeft;
Servo rollRight;
Servo pitchRight;
Servo pitchLeft;
// Servo yaw;
Servo motor;

Adafruit_BMP280 bmp;

boolean timeout = false;

byte addresses[][6] = {"1Node", "2Node"};

byte transmitData[1];
byte recievedData[4];

unsigned long lastRecievedTime = millis();
unsigned long currentTime = millis();
unsigned long timeoutMilliSeconds = 500;
unsigned long elapsedTime = 0;

void setup() {
  Serial.begin(115200);

  // pinMode(lightPin, OUTPUT);
  printf_begin();

  radio.begin();
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(112);
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.startListening();

  radio.printDetails();

  // Attach servos
  motor.attach(motorPin, minThrottle, maxThrottle);
  rollLeft.attach(rollServoLeftPin);
  rollRight.attach(rollServoRightPin);
  pitchRight.attach(pitchServoRightPin);
  pitchLeft.attach(pitchServoLeftPin);

  // if (!bmp.begin(0x76)) {

  // }

  /* Default settings from datasheet. */
  // bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  //                 Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  //                 Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling
  //                 */ Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  //                 Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println("Done with setup!");
}

// void printMotion() {
// Accelerometer range is set at [-4,+4]g -/+0.122 mg
// Gyroscope range is set at [-2000, +2000] dps +/-70 mdps
// Magnetometer range is set at [-400, +400] uT +/-0.014 uT
// Accelerometer Output data rate is fixed at 104 Hz
// Gyroscope Output data rate is fixed at 104 Hz
// Magnetometer Output data rate is fixed at 20 Hz
// float x, y, z, x2, y2, z2, x3, y3, z3;

// if (IMU.accelerationAvailable())
// {
//   IMU.readAcceleration(x, y, z);

//   Serial.print(x);
//   Serial.print('\t');
//   Serial.print(y);
//   Serial.print('\t');
//   Serial.print(z);
//   Serial.print('\t');
//   Serial.print('\t');
// }

// if (IMU.gyroscopeAvailable())
// {
//   IMU.readGyroscope(x2, y2, z2);

//   Serial.print(x2);
//   Serial.print('\t');
//   Serial.print(y2);
//   Serial.print('\t');
//   Serial.print(z2);
//   Serial.print('\t');
//   Serial.print('\t');
// }

// if (IMU.magneticFieldAvailable())
// {
//   IMU.readMagneticField(x3, y3, z3);

//   Serial.print(x3);
//   Serial.print('\t');
//   Serial.print(y3);
//   Serial.print('\t');
//   Serial.println(z3);
// }
// delay(100);
// }

// void printCamera()
// {
//   if (!APDS.begin())
//   {
//     Serial.println("Error initializing APDS9960 sensor.");
//   }

//   while (!APDS.colorAvailable())
//   {
//     delay(5);
//   }
//   int r, g, b;

//   APDS.readColor(r, g, b);

//   Serial.print("r = ");
//   Serial.println(r);
//   Serial.print("g = ");
//   Serial.println(g);
//   Serial.print("b = ");
//   Serial.println(b);
//   Serial.println();

//   delay(1000);

//   // 10 to 20 cm
//   while (!APDS.proximityAvailable())
//   {
//     delay(5);
//   }

//   int p = APDS.readProximity();

//   Serial.print("p = ");
//   Serial.println(p);

//   delay(1000);

//   while (!APDS.gestureAvailable())
//   {
//     delay(5);
//   }

//   int gesture = APDS.readGesture();

//   Serial.print("gesture = ");
//   Serial.println(gesture);
//   APDS.end();

//   delay(1000);
// }

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
  // Serial.print(recievedData[i]);
  // Serial.print(" ");
  // }
  // Serial.println();
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

  pitchLeft.write(pitchValue);
  pitchRight.write(180 - pitchValue);

  // yaw.write(yawValue);

  motor.write(recievedData[throttleIndex]);
}

void reset() {
  recievedData[rollIndex] = 127;
  recievedData[pitchIndex] = 127;
  recievedData[yawIndex] = 127;
}

void printPressureAndTemp() {
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println("C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.println();
}

void ACS() {
  recievedData[throttleIndex] = 0;
  reset();
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

    printRecievedData();
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