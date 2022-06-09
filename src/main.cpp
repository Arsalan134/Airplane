#include <definitions.h>

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

void loop() {

  // printMotion();  // working
  // printCamera();  // working
  // printPressureAndTemp();
  delay(delayTime);

  // readSensors();
  transmit();

  if (radio.available()) {
    radio.read(&recievedData, sizeof(recievedData));
    lastRecievedTime = millis();
  }

  elapsedTime = millis() - lastRecievedTime;

  // Activate ACS when signal is lost
  if (elapsedTime >= timeoutMilliSeconds) {
    ACS();
  } else {
    printRecievedData();
    // Serial.print("Elapsed: ");
    // Serial.println(elapsedTime);
  }

  makeStuffWithRecievedData();
}

void makeStuffWithRecievedData() {

  byte rollValue = map(recievedData[rollIndex], 0, 180,
                       90 - degreeOfFreedom / 2, 90 + degreeOfFreedom / 2);
  byte pitchValue = map(recievedData[pitchIndex], 0, 255,
                        90 - degreeOfFreedom / 2, 90 + degreeOfFreedom / 2);
  // byte yawValue = map(recievedData[yawIndex], 0, 255, 0, 180);

  // yawValue = constrain(yawValue, 55, 160);

  rollLeft.write(180 - rollValue);
  rollRight.write(180 - rollValue);

  pitch.write(180 - pitchValue);
  yaw.write(pitchValue);

  motor.write(recievedData[throttleIndex]);
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
  radio.stopListening();
  radio.write(&transmitData, sizeof(transmitData));
  radio.startListening();
}

void reset() {
  recievedData[rollIndex] = 90;
  recievedData[pitchIndex] = 140;
  recievedData[yawIndex] = 90;
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

// I2C device found at address 0x68  !
// I2C device found at address 0x76  !
