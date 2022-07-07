#include <definitions.h>

void setup() {
  Serial.begin(115200);

  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);  // Turn Of LED

  if (!radio.begin()) {
    while (true) {
      Serial.println("Radio hardware is not responding!");
      delay(500);
    }
  } else {
    Serial.println("Radio is working");
  }

  delay(1000);

  printf_begin();

  delay(1000);

  radio.setAutoAck(false);
  // add it to ground scketch too
  radio.setPayloadSize(sizeof(transmitData) / sizeof(byte));
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(112);
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.setDataRate(RF24_250KBPS);
  radio.powerUp();
  radio.startListening();

  delay(1000);

  radio.printDetails();

  // Attach servos
  engine.attach(motorPin, minThrottle, maxThrottle);
  rollLeftMotor.attach(rollServoLeftPin);
  rollRightMotor.attach(rollServoRightPin);
  pitchMotor.attach(pitchServoPin);
  yawMotor.attach(yawServoPin);

  reset();

  Serial.println("Done with setup!");
}

void loop() {
  // readSensors();
  transmit();

  if (radio.available()) {
    radio.read(&recievedData, sizeof(recievedData));
    lastRecievedTime = millis();
  }

  elapsedTime = millis() - lastRecievedTime;

  if (elapsedTime >= timeoutMilliSeconds) {
    ACS();  // Activate ACS when signal is lost
  } else {
    makeStuffWithRecievedData();
    printRecievedData();
    // Serial.print("Elapsed: ");
    // Serial.println(elapsedTime);
  }
}

void makeStuffWithRecievedData() {
  byte rollValue = map(recievedData[rollIndex], 0, 180, -90, 90);
  byte pitchValue = map(recievedData[pitchIndex], 0, 255, -90, 90);
  byte yawValue = map(recievedData[yawIndex], 0, 255, 0, 180);

  roll(rollValue);
  pitch(pitchValue);
  yaw(yawValue);

  engine.write(recievedData[throttleIndex]);
}

void roll(byte byAmount) {
  rollLeftMotor.write(90 + byAmount);
  rollRightMotor.write(90 - byAmount);
}

void pitch(byte byAmount) {
  pitchMotor.write(90 + byAmount);
  ;
}

void yaw(byte byAmount) {
  yawMotor.write(byAmount);
  ;
}

void printTransmitData() {
  Serial.print("Sent: \t\t");
  for (unsigned long i = 0; i < sizeof(transmitData) / sizeof(transmitData[0]); i++) {
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

void readSensors() {
  transmitData[batteryIndex] = 0;
}

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

void printPressureAndTemp() {
  //   Serial.print(F("Temperature = "));
  //   Serial.print(bmp.readTemperature());
  //   Serial.println("C");

  //   Serial.print(F("Pressure = "));
  //   Serial.print(bmp.readPressure());
  //   Serial.println(" Pa");

  //   Serial.print(F("Approx altitude = "));
  //   Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast!
  //                                             */
  //   Serial.println(" m");

  //   Serial.println();
}

/// Active Control System
void ACS() {
  recievedData[throttleIndex] = 0;
  // reset();

  // comment reset function call

  /*
   * 1. Read Accelerometer data
   *
   * 2. Move Wings
   * roll();
   * pitch();
   * yaw();
   */
}

// I2C device found at address 0x68  !
// I2C device found at address 0x76  !
