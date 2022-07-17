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

void imuSetup() {
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);
}

void imu() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
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
