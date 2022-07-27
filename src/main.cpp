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
  servoSetup();
  radioSetup();
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

  while (mpu.begin()) {
    Serial.println("Connecting to IMU");
    delay(200);
  }

  mpu.calcOffsets();
}

void servoSetup() {
  engine.attach(motorPin, 1000, 2000);

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

  if (millis() - lastRecievedTime >= timeoutInMilliSeconds) {
    engineOff();
    ACS();
  } else
    makeStuffWithRecievedData();
}

void makeStuffWithRecievedData() {
  printRecievedData();

  pitchBias = recievedData[pitchBiasIndex] - 90;

  if (recievedData[autopilotIsOnIndex])
    ACS();
  else {
    rollValue = recievedData[rollIndex];
    pitchValue = recievedData[pitchIndex];
    // yawValue = recievedData[yawIndex];

    roll(rollValue);
    pitch(pitchValue);
    // yaw(yawValue);
  }

  engine.write(recievedData[throttleIndex]);
}

void engineOff() {
  recievedData[throttleIndex] = 0;
  engine.write(0);
}

void roll(byte angle) {
  angle = map(constrain(angle, 0, 180), 180, 0, degreesOfFreedomAilerons / 2,
              180 - degreesOfFreedomAilerons / 2);

  rollRightMotor.write(constrain(angle + RollRightBias, 0, 180));
  rollLeftMotor.write(constrain(angle + RollLeftBias, 0, 180));
}

void rollBy(byte byDegrees) {
  roll(90 + byDegrees);
}

void pitch(byte angle) {
  pitchMotor.write(constrain(angle + pitchBias, 0, 180));
}

void pitchBy(byte byDegrees) {
  pitch(90 + byDegrees);
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
  recievedData[throttleIndex] = 0;
  recievedData[rollIndex] = 90;
  recievedData[pitchIndex] = 90;
  recievedData[yawIndex] = 90;

  recievedData[autopilotIsOnIndex] = false;
}

void ACS() {
  mpu.update();

  correctedRollAmount = mpu.getAngleX() * multiplierACS;
  correctedRollAmount = constrain(correctedRollAmount, -90, 90);
  rollBy(-correctedRollAmount);

  correctedPitchAmount = mpu.getAngleY() * multiplierACS;
  correctedPitchAmount = constrain(correctedPitchAmount, -90, 90);
  pitchBy(correctedPitchAmount);
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
  // Serial.print("Elapsed: ");
  // Serial.println(elapsedTime);

  Serial.print("Throttle ");
  Serial.println(recievedData[throttleIndex]);

  Serial.print("Yaw ");
  Serial.println(recievedData[yawIndex]);

  Serial.print("Pitch ");
  Serial.println(recievedData[pitchIndex]);

  Serial.print("Roll ");
  Serial.println(recievedData[rollIndex]);

  Serial.print("Autopilot ");
  Serial.println(recievedData[autopilotIsOnIndex]);

  Serial.print("Pitch Bias ");
  Serial.println(recievedData[pitchBiasIndex]);

  Serial.println();
}