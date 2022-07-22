#include <definitions.h>

void setup() {
  Serial.begin(115200);

#if !defined(__MIPSEL__)
  while (!Serial) {
    delay(100);  // Wait for serial port to connect - used on Leonardo, Teensy
                 // and other boards with built-in USB CDC serial connection
    Serial.println("Waiting serial");
  }
#endif

  imuSetup();
  radioSetup();
  servoSetup();
}

void radioSetup() {
  printf_begin();

  while (!radio.begin()) {
    Serial.println("Radio hardware is not responding!");
    delay(100);
  }

  Serial.println("Radio is working");

  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(112);
  radio.setCRCLength(RF24_CRC_8);
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.setDataRate(RF24_250KBPS);
  radio.enableDynamicPayloads();
  radio.printDetails();

  radio.startListening();
}

void imuSetup() {
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);

  while (status != 0) {
    status = mpu.begin();
    Serial.println("Connecting to IMU");
    delay(200);
  }

  mpu.calcOffsets();
}

void servoSetup() {
  engine.attach(motorPin, minThrottle, maxThrottle);

  rollLeftMotor.attach(rollServoLeftPin);
  rollRightMotor.attach(rollServoRightPin);
  pitchMotor.attach(pitchServoPin);
  // yawMotor.attach(yawServoPin);

  resetAirplaneToDefaults();
}

void loop() {
  readSensors();

  delay(delayTime);

  transmit();

  delay(delayTime);

  while (radio.available()) {
    radio.read(&recievedData, sizeof(recievedData));
    lastRecievedTime = millis();
  }

  elapsedTime = millis() - lastRecievedTime;

  if (elapsedTime >= timeoutMilliSeconds) {
    ACS();
  } else {
    // printRecievedData();
    // Serial.print("Elapsed: ");
    // Serial.println(elapsedTime);
  }

  makeStuffWithRecievedData();
}

void makeStuffWithRecievedData() {
  rollValue = recievedData[rollIndex];
  pitchValue = recievedData[pitchIndex];
  // yawValue = recievedData[yawIndex];

  roll(map(rollValue, 0, 180, degreesOfFreedomAilerons / 2, 180 - degreesOfFreedomAilerons / 2));
  pitch(pitchValue);
  // yaw(yawValue);

  engine.write(recievedData[throttleIndex]);
}

void roll(byte angle) {
  rollRightMotor.write(angle + RollRightBias);
  rollLeftMotor.write(angle + RollLeftBias);
}

void pitch(byte angle) {
  pitchMotor.write(angle + pitchBias);
}

void yaw(byte angle) {
  yawMotor.write(angle);
}

void readSensors() {
  transmitData[batteryIndex] = 0;
}

void transmit() {
  delay(delayTime);

  radio.stopListening();

  delay(delayTime);

  radio.write(&transmitData, sizeof(transmitData));

  delay(delayTime);

  radio.startListening();

  delay(delayTime);
}

void resetAirplaneToDefaults() {
  recievedData[rollIndex] = 90;
  recievedData[pitchIndex] = 90;
  recievedData[yawIndex] = 90;
  recievedData[throttleIndex] = 0;
}

void ACS() {
  mpu.update();

  angleX = mpu.getAngleX();
  angleY = mpu.getAngleY();

  correctedRollAngle = constrain(angleX * multiplierACS + 90, 0, 180);
  correctedPitchAngle = constrain(angleY * multiplierACS + 90, 0, 180);

  recievedData[rollIndex] = correctedRollAngle;
  recievedData[pitchIndex] = correctedPitchAngle;
}

void printTransmissionData() {
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