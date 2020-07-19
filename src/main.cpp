#include <Arduino.h>
#include <Arduino_APDS9960.h>  // camera
#include <Arduino_LSM9DS1.h>   // IMU
#include <Servo.h>

#include "RF24.h"
#include "printf.h"

short delayTime = 500;

// PINS

// ANALOG
short vibroPin = A0;
short photoresistorPin = A1;

// DIGITAL
short lightPin = 2;

// ~ ~ ~ ~ ~         ~ ~
// 2 3 4 5 6 7  8    9 10 11    12    13
// - - ? - - ce csn  - -  mosi  miso  sck

// PWM
short yawServoPin = 3;
short pitchServoPin = 5;
short motorPin = 6;
short rollServoPin = 9;

RF24 radio(7, 8);

Servo roll;
Servo pitch;
Servo yaw;
Servo motor;

int throttleValue = 0;  // to a motor

short minThrottle = 1000;
short maxThrottle = 2000;

short degreeOfFreedom = 90;

boolean timeout = false;

const uint64_t addresses[2] = {0xC1, 0xC2};

byte rollIndex = 0, pitchIndex = 2, yawIndex = 3, throttleIndex = 4;
byte batteryIndex = 0;

byte transmitData[5];
byte recievedData[5];

unsigned long lastRecievedTime = millis();
unsigned long currentTime = millis();

void setup() {
  Serial.begin(115200);

  while (!Serial)
    ;

  delay(1000);

  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");

  pinMode(lightPin, OUTPUT);
  printf_begin();

  delay(1000);

  // Attach servos
  motor.attach(motorPin, minThrottle, maxThrottle);
  roll.attach(rollServoPin);
  pitch.attach(pitchServoPin);
  yaw.attach(yawServoPin);

  radio.begin();

  delay(1000);

  radio.setAutoAck(false);
  radio.setPayloadSize(sizeof(transmitData));
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.setChannel(112);
  radio.printDetails();

  delay(1000);

  radio.startListening();

  Serial.println("Done with setup!");

  // Wait for initialization and calibration to finish
  delay(500);
}

void printMotion() {
  // Accelerometer range is set at [-4,+4]g -/+0.122 mg
  // Gyroscope range is set at [-2000, +2000] dps +/-70 mdps
  // Magnetometer range is set at [-400, +400] uT +/-0.014 uT
  // Accelerometer Output data rate is fixed at 104 Hz
  // Gyroscope Output data rate is fixed at 104 Hz
  // Magnetometer Output data rate is fixed at 20 Hz
  float x, y, z, x2, y2, z2, x3, y3, z3;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
    Serial.print('\t');
    Serial.print('\t');
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x2, y2, z2);

    Serial.print(x2);
    Serial.print('\t');
    Serial.print(y2);
    Serial.print('\t');
    Serial.print(z2);
    Serial.print('\t');
    Serial.print('\t');
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x3, y3, z3);

    Serial.print(x3);
    Serial.print('\t');
    Serial.print(y3);
    Serial.print('\t');
    Serial.println(z3);
  }
  delay(100);
}

void printCamera() {
  if (!APDS.begin()) {
    Serial.println("Error initializing APDS9960 sensor.");
  }

  while (!APDS.colorAvailable()) {
    delay(5);
  }
  int r, g, b;

  APDS.readColor(r, g, b);

  Serial.print("r = ");
  Serial.println(r);
  Serial.print("g = ");
  Serial.println(g);
  Serial.print("b = ");
  Serial.println(b);
  Serial.println();

  delay(1000);

  // 10 to 20 cm
  while (!APDS.proximityAvailable()) {
    delay(5);
  }

  int p = APDS.readProximity();

  Serial.print("p = ");
  Serial.println(p);

  delay(1000);

  while (!APDS.gestureAvailable()) {
    delay(5);
  }

  int gesture = APDS.readGesture();

  Serial.print("gesture = ");
  Serial.println(gesture);
  APDS.end();

  delay(1000);
}

void printPressure() {
  // float pressure, temperature;
  // PressTemp->GetPressure(&pressure);
  // PressTemp->GetTemperature(&temperature);

  // SerialPort.print("Pres[hPa]: ");
  // SerialPort.print(pressure, 2);
  // SerialPort.print(" | Temp[C]: ");
  // SerialPort.println(temperature, 2);

  // Serial.print("Pressuire[C]: ");
  // Serial.println(barometricSensor.readPressureHPA());

  delay(100);
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
  Serial.print("Recieved: \t");
  for (unsigned long i = 0; i < sizeof(recievedData) / sizeof(recievedData[0]);
       i++) {
    Serial.print(recievedData[i]);
    Serial.print(" ");
  }
  Serial.println();
}

// void readSensors() {
//  transmitData[tData.temperature] = byte(bmp.readTemperature());
//  transmitData[tData.vibration] = map(analogRead(vibroPin), 0, 1023, 0, 255);
//  transmitData[tData.altitude] = map(bmp.readAltitude(), 0, 1023, 0, 255);
//}

//  transmitData[2] = map(bmp.readPressure() / 133.3, 0, 1023, 0, 255);

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
  byte yawValue = map(recievedData[yawIndex], 0, 255, 180, 0);

  yawValue = constrain(yawValue, 55, 160);

  roll.write(rollValue);
  pitch.write(pitchValue);
  yaw.write(yawValue);

  throttleValue = recievedData[throttleIndex];
  motor.write(throttleValue);
}

void reset() {
  Serial.println("Reset...");
  recievedData[rollIndex] = 127;
  recievedData[pitchIndex] = 127;
  recievedData[yawIndex] = 127;
  recievedData[throttleIndex] = 0;
}

// Active Correction System
void ACS() {
  // read accelerometer data
  // find difference
  // adjust until the plane is in desired position

  // if no command was sent, try to correct itself. For example strong wind or
  // offset of center of mass.
}

void printTempHumidity() { delay(1000); }

void loop() {
  // printMotion();  // working
  // printCamera();  // working
  // printTempHumidity(); //. is not working
  // printPressure();

  timeout = false;

  currentTime = millis();

  if (currentTime - lastRecievedTime > 500) {
    timeout = true;
    Serial.println("TIMEOUT");
    reset();
  }

  if (radio.available()) {
    radio.read(&recievedData, sizeof(recievedData));
    lastRecievedTime = millis();
  }

  printRecievedData();
  makeStuffWithRecievedData();
}