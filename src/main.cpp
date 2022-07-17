#include <definitions.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  printf_begin();

  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);  // Turn Of LED

  while (!radio.begin()) {
    Serial.println("Radio hardware is not responding!");
    delay(500);
  }

  Serial.println("Radio is working");

#if !defined(__MIPSEL__)
  while (!Serial) {
    delay(100);  // Wait for serial port to connect - used on Leonardo, Teensy
                 // and other boards with built-in USB CDC serial connection
    Serial.println("Waiting serial");
  }
#endif

  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(112);
  radio.setCRCLength(RF24_CRC_8);
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.setDataRate(RF24_250KBPS);

  radio.startListening();

  radio.printDetails();

  delay(1000);

  // Attach servos
  engine.attach(motorPin, minThrottle, maxThrottle);
  rollLeftMotor.attach(rollServoLeftPin);
  rollRightMotor.attach(rollServoRightPin);
  pitchMotor.attach(pitchServoPin);
  yawMotor.attach(yawServoPin);

  reset();
}

void loop() {
  // imu();

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
    Serial.println("Lost Signal");
    delay(200);
    ACS();
  } else {
    printRecievedData();
    // Serial.print("Elapsed: ");
    // Serial.println(elapsedTime);
  }

  makeStuffWithRecievedData();
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
  delay(delayTime);

  radio.stopListening();

  delay(delayTime);

  radio.write(&transmitData, sizeof(transmitData));

  delay(delayTime);

  radio.startListening();

  delay(delayTime);
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
