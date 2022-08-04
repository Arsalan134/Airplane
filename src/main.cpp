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

  radioSetup();
  imuSetup();
  servoSetup();

  // setupSDCard();
  // printCardInfo();
  // barometerSetup();
  // magnetometerSetup();
}

// void barometerSetup() {
//   Serial.println("Initializing Barometer DPS310 ...");

//   if (!dps.begin_I2C()) {
//     Serial.println("Failed to find DPS310");
//     return;
//   }

//   Serial.println("DPS OK!");

//   dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
//   dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

//   // dps_temp->printSensorDetails();
//   dps_pressure->printSensorDetails();
// }

// void magnetometerSetup() {
//   bmm = BMM150();

//   if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
//     Serial.println("BMM150 Chip ID can not read!");
//     return;
//   }

//   Serial.println("BMM150 Initialize done!");

//   Serial.println("BMM150 Start figure-8 calibration after 3 seconds.");
//   delay(3000);

//   // calibrate(10000);

//   Serial.print("\n\rBMM150 Calibrate done..");
// }

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
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu.initialize();

  // verify connection
  Serial.println("Testing device connections... ");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful"
                                      : "MPU6050 connection failed");

  // load and configure the DMP
  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  if (!devStatus) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(7);
    mpu.CalibrateGyro(7);

    Serial.println();

    mpu.PrintActiveOffsets();

    mpu.setDMPEnabled(true);

  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
}

void servoSetup() {
  engine.attach(motorPin, 1000, 2000);

  rollLeftMotor.attach(rollServoLeftPin);
  rollRightMotor.attach(rollServoRightPin);
  pitchMotor.attach(pitchServoPin);
  // yawMotor.attach(yawServoPin);

  resetAirplaneToDefaults();
}

// void setupSDCard() {
//   Serial.print("Initializing SD card...");

//   // pinMode(sdCardPin, OUTPUT); ? necessary ?

//   while (!SD.begin(sdCardPin)) {
//     Serial.println("SD Card initialization failed!");
//     delay(100);
//   }

//   Serial.println("SD Card initialization done.");
// }

void loop() {
  readSensors();

  transmit();

  delay(delayTime);

  while (radio.available()) {
    radio.read(&recievedData, sizeof(recievedData));
    lastRecievedTime = millis();
  }

  if (millis() - lastRecievedTime >= timeoutInMilliSeconds)
    lostRadio();
  else
    makeStuffWithRecievedData();
}

void IMULoop() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    currentRollValue = ypr[2] * 180 / M_PI;
    currentPitchValue = ypr[1] * 180 / M_PI;
  }
}

// void magnetometerLoop() {
//   bmm.read_mag_data();

//   bmm150_value.x = bmm.raw_mag_data.raw_datax - bmm150_value_offset.x;
//   bmm150_value.y = bmm.raw_mag_data.raw_datay - bmm150_value_offset.y;
//   bmm150_value.z = bmm.raw_mag_data.raw_dataz - bmm150_value_offset.z;

//   short heading = atan2(bmm150_value.x, bmm150_value.y);

//   if (heading < 0)
//     heading += 2 * PI;

//   if (heading > 2 * PI)
//     heading -= 2 * PI;

//   headingDegrees = heading * 180 / M_PI;

//   Serial.print("Heading: ");
//   Serial.println(headingDegrees);
// }

// void barometerLoop() {
// sensors_event_t temp_event;
// sensors_event_t pressure_event;

// if (dps.temperatureAvailable()) {
//   dps_temp->getEvent(&temp_event);
// temperature = temp_event.temperature;

//   Serial.print(F("Temperature = "));
//   Serial.print(temperature);
//   Serial.println(" *C");
//   Serial.println();
// }

//   // Reading pressure also reads temp so don't check pressure
//   // before temp!
//   if (dps.pressureAvailable()) {
//     dps_pressure->getEvent(&pressure_event);
//     pressure = pressure_event.pressure;

//     Serial.print(F("Pressure = "));
//     Serial.print(pressure);
//     Serial.println(" hPa");

//     Serial.println();
//   }
// }

void lostRadio() {
  engineOff();
  ACS();
}

void makeStuffWithRecievedData() {
  pitchBias = recievedData[pitchBiasIndex] - 90;

  if (recievedData[autopilotIsOnIndex])
    ACS();
  else {
    // yawValue = recievedData[yawIndex];
    roll(recievedData[rollIndex]);
    pitch(recievedData[pitchIndex]);
    // yaw(yawValue);
  }

  engine.write(recievedData[throttleIndex]);

  // Serial.print("Elapsed: ");
  // Serial.println(millis() - lastRecievedTime);
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

  IMULoop();

  // barometerLoop();
  // magnetometerLoop();
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
  correctedRollAmount = currentRollValue * multiplierRollACS;
  correctedRollAmount = constrain(correctedRollAmount, -90, 90);
  rollBy(-correctedRollAmount);

  correctedPitchAmount = currentPitchValue * multiplierPitchACS;
  correctedPitchAmount = constrain(correctedPitchAmount, -90, 90);
  pitchBy(-correctedPitchAmount);
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

// void printCardInfo() {
//   if (!card.init(SPI_HALF_SPEED, sdCardPin)) {
//     Serial.println("initialization failed. Things to check:");
//     Serial.println("* is a card inserted?");
//     Serial.println("* is your wiring correct?");
//     Serial.println("* did you change the chipSelect pin to match your shield or module?");
//     return;
//   } else
//     Serial.println("Wiring is correct and a card is present.");

//   Serial.println();

//   Serial.print("Card type:         ");

//   switch (card.type()) {
//     case SD_CARD_TYPE_SD1:
//       Serial.println("SD1");
//       break;

//     case SD_CARD_TYPE_SD2:
//       Serial.println("SD2");
//       break;

//     case SD_CARD_TYPE_SDHC:
//       Serial.println("SDHC");
//       break;

//     default:
//       Serial.println("Unknown");
//   }

//   // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
//   if (!volume.init(card)) {
//     Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
//     return;
//   }

//   Serial.print("Clusters:          ");
//   Serial.println(volume.clusterCount());

//   Serial.print("Blocks x Cluster:  ");
//   Serial.println(volume.blocksPerCluster());

//   Serial.print("Total Blocks:      ");
//   Serial.println(volume.blocksPerCluster() * volume.clusterCount());

//   Serial.println();

//   // print the type and size of the first FAT-type volume

//   uint32_t volumesize;

//   Serial.print("Volume type is:    FAT");
//   Serial.println(volume.fatType(), DEC);

//   volumesize = volume.blocksPerCluster();  // clusters are collections of blocks
//   volumesize *= volume.clusterCount();     // we'll have a lot of clusters
//   volumesize /= 2;                         // SD card blocks are always 512 bytes (2 blocks are
//   1KB)

//   Serial.print("Volume size (Kb):  ");
//   Serial.println(volumesize);

//   Serial.print("Volume size (Mb):  ");
//   volumesize /= 1024;

//   Serial.println(volumesize);
//   Serial.print("Volume size (Gb):  ");

//   Serial.println((float)volumesize / 1024.0);
//   Serial.println("\nFiles found on the card (name, date and size in bytes): ");

//   root.openRoot(volume);

//   // list all files in the card with date and size
//   root.ls(LS_R | LS_DATE | LS_SIZE);
//   root.close();
// }

// void saveToFile(String name) {
//   // open the file. note that only one file can be open at a time,
//   // so you have to close this one before opening another.

//   /*
//     Format to store data in micro sd card

//     ReceivedRoll, currentRoll, ReceivedPitch, currentPitch, ReceivedThrottle,
//     accx, accy, accz, gyrox, gyroy, gyroz,
//     HeadingFromMagnetometer, pressure, Temperature,
//     autopilotIsOn, pitchBias,

//   */

//   File file = SD.open(name, FILE_WRITE);

//   if (file) {
//     file.print(String(recievedData[rollIndex]) + ", ");

//     file.print(String(currentRollValue) + ", ");

//     file.print(String(recievedData[pitchIndex]) + ", ");

//     file.print(String(currentPitchValue) + ", ");

//     file.print(String(recievedData[throttleIndex]) + ", ");

//     file.print(String(mpu.getAccX()) + ", ");
//     file.print(String(mpu.getAccY()) + ", ");
//     file.print(String(mpu.getAccZ()) + ", ");

//     file.print(String(mpu.getGyroX()) + ", ");
//     file.print(String(mpu.getGyroY()) + ", ");
//     file.print(String(mpu.getGyroZ()) + ", ");

//     // file.print(String(headingDegrees) + ", ");

//     // file.print(String(pressure) + ", ");

//     file.print(String(recievedData[autopilotIsOnIndex]) + ", ");

//     file.print(String(recievedData[pitchBiasIndex]));

//     file.println();
//     file.close();
//   } else
//     Serial.println("error opening test.txt");
// }

// void calibrate(uint32_t timeout) {
//   int16_t value_x_min = 0;
//   int16_t value_x_max = 0;
//   int16_t value_y_min = 0;
//   int16_t value_y_max = 0;
//   int16_t value_z_min = 0;
//   int16_t value_z_max = 0;
//   uint32_t timeStart = 0;

//   bmm.read_mag_data();
//   value_x_min = bmm.raw_mag_data.raw_datax;
//   value_x_max = bmm.raw_mag_data.raw_datax;
//   value_y_min = bmm.raw_mag_data.raw_datay;
//   value_y_max = bmm.raw_mag_data.raw_datay;
//   value_z_min = bmm.raw_mag_data.raw_dataz;
//   value_z_max = bmm.raw_mag_data.raw_dataz;
//   delay(100);

//   timeStart = millis();

//   while ((millis() - timeStart) < timeout) {
//     bmm.read_mag_data();

//     /* Update x-Axis max/min value */
//     if (value_x_min > bmm.raw_mag_data.raw_datax) {
//       value_x_min = bmm.raw_mag_data.raw_datax;
//       // Serial.print("Update value_x_min: ");
//       // Serial.println(value_x_min);

//     } else if (value_x_max < bmm.raw_mag_data.raw_datax) {
//       value_x_max = bmm.raw_mag_data.raw_datax;
//       // Serial.print("update value_x_max: ");
//       // Serial.println(value_x_max);
//     }

//     /* Update y-Axis max/min value */
//     if (value_y_min > bmm.raw_mag_data.raw_datay) {
//       value_y_min = bmm.raw_mag_data.raw_datay;
//       // Serial.print("Update value_y_min: ");
//       // Serial.println(value_y_min);

//     } else if (value_y_max < bmm.raw_mag_data.raw_datay) {
//       value_y_max = bmm.raw_mag_data.raw_datay;
//       // Serial.print("update value_y_max: ");
//       // Serial.println(value_y_max);
//     }

//     /* Update z-Axis max/min value */
//     if (value_z_min > bmm.raw_mag_data.raw_dataz) {
//       value_z_min = bmm.raw_mag_data.raw_dataz;
//       // Serial.print("Update value_z_min: ");
//       // Serial.println(value_z_min);

//     } else if (value_z_max < bmm.raw_mag_data.raw_dataz) {
//       value_z_max = bmm.raw_mag_data.raw_dataz;
//       // Serial.print("update value_z_max: ");
//       // Serial.println(value_z_max);
//     }

//     Serial.print(".");
//     delay(100);
//   }

//   bmm150_value_offset.x = value_x_min + (value_x_max - value_x_min) / 2;
//   bmm150_value_offset.y = value_y_min + (value_y_max - value_y_min) / 2;
//   bmm150_value_offset.z = value_z_min + (value_z_max - value_z_min) / 2;
// }