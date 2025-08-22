/*
 * Arduino Nano 33 BLE Sense - Flight Servo Controller
 * Slave controller for airplane servo control with IMU sensors
 *
 * Features:
 * - 5 servo control with smooth movement
 * - IMU data collection (roll, pitch, yaw)
 * - Barometric altitude and temperature
 * - I2C communication with master
 * - Health monitoring and diagnostics
 *
 * Author: Flight Control System
 * Date: August 2025
 */

#include <Arduino_HTS221.h>   // Temperature/Humidity
#include <Arduino_LPS22HB.h>  // Pressure sensor
#include <Arduino_LSM9DS1.h>  // IMU
#include <Servo.h>
#include <Wire.h>

// Include the protocol definitions
struct ServoCommandPacket {
  uint8_t header;
  uint8_t engine;
  uint8_t rollLeft;
  uint8_t elevatorLeft;
  uint8_t elevatorRight;
  uint8_t rudder;
  uint8_t trim_elevator;
  uint8_t trim_aileron;
  uint8_t flaps;
  bool landingAirbrake;
  uint8_t checksum;
} __attribute__((packed));

struct FlightDataPacket {
  uint8_t header;
  float roll;
  float pitch;
  float yaw;
  float altitude;
  float temperature;
  float pressure;
  uint8_t servo_status;
  uint8_t checksum;
} __attribute__((packed));

struct StatusPacket {
  uint8_t header;
  bool servos_healthy;
  uint8_t last_command_received;
  uint32_t uptime_ms;
  uint8_t imu_status;
  uint8_t checksum;
} __attribute__((packed));

// I2C Configuration
#define I2C_ADDRESS 0x08
#define FLIGHT_DATA_REQUEST 0x10
#define SERVO_COMMAND 0x20
#define STATUS_REQUEST 0x30

// Servo pin assignments (PWM capable pins)
#define ENGINE_SERVO_PIN 3
#define ROLL_LEFT_SERVO_PIN 5
#define ELEVATOR_LEFT_SERVO_PIN 6
#define ELEVATOR_RIGHT_SERVO_PIN 9
#define RUDDER_SERVO_PIN 10

// Status LED
#define STATUS_LED_PIN LED_BUILTIN

// Servo objects
Servo engineServo;
Servo rollLeftServo;
Servo elevatorLeftServo;
Servo elevatorRightServo;
Servo rudderServo;

// Current servo positions (for smooth movement)
int currentEngine = 90;
int currentRollLeft = 90;
int currentElevatorLeft = 90;
int currentElevatorRight = 90;
int currentRudder = 90;

// Target positions from master
int targetEngine = 90;
int targetRollLeft = 90;
int targetElevatorLeft = 90;
int targetElevatorRight = 90;
int targetRudder = 90;

// IMU and sensor data
float roll, pitch, yaw;
float altitude_offset = 0;
float current_altitude = 0;
float temperature = 0;
float pressure = 0;

// Status variables
bool imu_healthy = false;
bool pressure_healthy = false;
bool temp_healthy = false;
bool servos_attached = false;
unsigned long last_command_time = 0;
uint8_t servo_health_status = 0;

// Communication
volatile bool new_command_received = false;
volatile uint8_t i2c_request_type = 0;
ServoCommandPacket received_command;

// Utility functions
uint8_t calculateChecksum(uint8_t* data, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length - 1; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

bool validateChecksum(uint8_t* data, size_t length) {
  uint8_t calculated = calculateChecksum(data, length);
  return (calculated == data[length - 1]);
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("🛩️ Arduino Nano 33 BLE Sense - Flight Servo Controller");
  Serial.println("📡 Initializing I2C slave...");

  // Initialize I2C as slave
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // Initialize status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Initialize servos
  initializeServos();

  // Initialize sensors
  initializeSensors();

  // Set all servos to neutral position
  setAllServosNeutral();

  Serial.println("✅ Servo controller ready!");
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void loop() {
  unsigned long currentTime = millis();

  // Update sensors every 50ms (20Hz)
  static unsigned long lastSensorUpdate = 0;
  if (currentTime - lastSensorUpdate >= 50) {
    updateSensorData();
    lastSensorUpdate = currentTime;
  }

  // Process new commands
  if (new_command_received) {
    processServoCommand();
    new_command_received = false;
    last_command_time = currentTime;
  }

  // Smooth servo movement every 20ms (50Hz)
  static unsigned long lastServoUpdate = 0;
  if (currentTime - lastServoUpdate >= 20) {
    updateServoPositions();
    lastServoUpdate = currentTime;
  }

  // Check for command timeout (safety feature)
  if (currentTime - last_command_time > 2000) {  // 2 second timeout
    // Return to safe positions if no commands received
    safetyMode();
  }

  // Status LED blink pattern
  static unsigned long lastBlink = 0;
  if (currentTime - lastBlink >= (servos_attached ? 1000 : 200)) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    lastBlink = currentTime;
  }

  delay(1);  // Small delay to prevent watchdog issues
}

void initializeServos() {
  Serial.println("🔧 Initializing servos...");

  // Attach servos to pins
  engineServo.attach(ENGINE_SERVO_PIN);
  rollLeftServo.attach(ROLL_LEFT_SERVO_PIN);
  elevatorLeftServo.attach(ELEVATOR_LEFT_SERVO_PIN);
  elevatorRightServo.attach(ELEVATOR_RIGHT_SERVO_PIN);
  rudderServo.attach(RUDDER_SERVO_PIN);

  servos_attached = true;
  servo_health_status = 0xFF;  // All servos healthy

  Serial.println("✅ All servos attached");
}

void initializeSensors() {
  Serial.println("🔬 Initializing sensors...");

  // Initialize IMU
  if (IMU.begin()) {
    imu_healthy = true;
    Serial.println("✅ IMU initialized");
  } else {
    imu_healthy = false;
    Serial.println("❌ IMU failed to initialize");
  }

  // Initialize pressure sensor
  if (BARO.begin()) {
    pressure_healthy = true;
    Serial.println("✅ Pressure sensor initialized");

    // Calibrate altitude (set current as reference)
    delay(100);
    altitude_offset = BARO.readPressure() * 100;  // Convert to Pa
    altitude_offset = 44330 * (1.0 - pow(altitude_offset / 101325.0, 0.1903));
  } else {
    pressure_healthy = false;
    Serial.println("❌ Pressure sensor failed");
  }

  // Initialize temperature/humidity sensor
  if (HTS.begin()) {
    temp_healthy = true;
    Serial.println("✅ Temperature sensor initialized");
  } else {
    temp_healthy = false;
    Serial.println("❌ Temperature sensor failed");
  }
}

void setAllServosNeutral() {
  Serial.println("🎯 Setting all servos to neutral position...");

  currentEngine = 0;          // Engine off for safety
  currentRollLeft = 90;       // Neutral
  currentElevatorLeft = 90;   // Neutral
  currentElevatorRight = 90;  // Neutral
  currentRudder = 90;         // Neutral

  targetEngine = currentEngine;
  targetRollLeft = currentRollLeft;
  targetElevatorLeft = currentElevatorLeft;
  targetElevatorRight = currentElevatorRight;
  targetRudder = currentRudder;

  // Write to servos
  engineServo.write(currentEngine);
  rollLeftServo.write(currentRollLeft);
  elevatorLeftServo.write(currentElevatorLeft);
  elevatorRightServo.write(currentElevatorRight);
  rudderServo.write(currentRudder);

  delay(500);  // Give servos time to move
}

void updateSensorData() {
  // Read IMU data
  if (imu_healthy && IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    // Calculate roll and pitch from accelerometer
    roll = atan2(ay, az) * 180.0 / PI;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // Simple yaw calculation (not very accurate without proper fusion)
    yaw = atan2(my, mx) * 180.0 / PI;

    // Normalize yaw to 0-360
    if (yaw < 0)
      yaw += 360;
  }

  // Read barometric data
  if (pressure_healthy && BARO.pressureAvailable()) {
    pressure = BARO.readPressure() * 100;  // Convert to Pa

    // Calculate altitude using barometric formula
    current_altitude = 44330 * (1.0 - pow(pressure / 101325.0, 0.1903)) - altitude_offset;
  }

  // Read temperature
  if (temp_healthy && HTS.temperatureAvailable()) {
    temperature = HTS.readTemperature();
  }
}

void processServoCommand() {
  // Validate checksum
  if (!validateChecksum((uint8_t*)&received_command, sizeof(ServoCommandPacket))) {
    Serial.println("❌ Command checksum failed");
    return;
  }

  // Update target positions
  targetEngine = constrain(received_command.engine, 0, 180);
  targetRollLeft = constrain(received_command.rollLeft, 0, 180);
  targetElevatorLeft = constrain(received_command.elevatorLeft, 0, 180);
  targetElevatorRight = constrain(received_command.elevatorRight, 0, 180);
  targetRudder = constrain(received_command.rudder, 0, 180);

  Serial.printf("📡 New targets - E:%d R:%d EL:%d ER:%d Ru:%d\n", targetEngine, targetRollLeft, targetElevatorLeft, targetElevatorRight,
                targetRudder);
}

void updateServoPositions() {
  // Smooth movement - move towards target gradually
  const int max_step = 2;  // Maximum degrees per update (adjust for smoothness vs speed)

  currentEngine = smoothMove(currentEngine, targetEngine, max_step);
  currentRollLeft = smoothMove(currentRollLeft, targetRollLeft, max_step);
  currentElevatorLeft = smoothMove(currentElevatorLeft, targetElevatorLeft, max_step);
  currentElevatorRight = smoothMove(currentElevatorRight, targetElevatorRight, max_step);
  currentRudder = smoothMove(currentRudder, targetRudder, max_step);

  // Write to servos
  if (servos_attached) {
    engineServo.write(currentEngine);
    rollLeftServo.write(currentRollLeft);
    elevatorLeftServo.write(currentElevatorLeft);
    elevatorRightServo.write(currentElevatorRight);
    rudderServo.write(currentRudder);
  }
}

int smoothMove(int current, int target, int max_step) {
  int difference = target - current;

  if (abs(difference) <= max_step) {
    return target;
  } else if (difference > 0) {
    return current + max_step;
  } else {
    return current - max_step;
  }
}

void safetyMode() {
  // Return to safe positions
  targetEngine = 0;          // Engine off
  targetRollLeft = 90;       // Neutral
  targetElevatorLeft = 90;   // Neutral
  targetElevatorRight = 90;  // Neutral
  targetRudder = 90;         // Neutral

  Serial.println("⚠️ Safety mode activated - returning to neutral positions");
}

// I2C event handlers
void receiveEvent(int bytes) {
  if (bytes == 1) {
    // Single byte - it's a request type
    i2c_request_type = Wire.read();
  } else if (bytes == sizeof(ServoCommandPacket)) {
    // Full command packet
    Wire.readBytes((uint8_t*)&received_command, sizeof(ServoCommandPacket));
    new_command_received = true;
  }
}

void requestEvent() {
  switch (i2c_request_type) {
    case FLIGHT_DATA_REQUEST:
      sendFlightData();
      break;

    case STATUS_REQUEST:
      sendStatus();
      break;

    default:
      // Send a simple acknowledgment
      Wire.write(0xFF);
      break;
  }
}

void sendFlightData() {
  FlightDataPacket data;
  data.header = FLIGHT_DATA_REQUEST;
  data.roll = roll;
  data.pitch = pitch;
  data.yaw = yaw;
  data.altitude = current_altitude;
  data.temperature = temperature;
  data.pressure = pressure;
  data.servo_status = servo_health_status;
  data.checksum = calculateChecksum((uint8_t*)&data, sizeof(FlightDataPacket));

  Wire.write((uint8_t*)&data, sizeof(FlightDataPacket));
}

void sendStatus() {
  StatusPacket status;
  status.header = STATUS_REQUEST;
  status.servos_healthy = servos_attached;
  status.last_command_received = (millis() - last_command_time) / 100;  // In 100ms units
  status.uptime_ms = millis();
  status.imu_status = (imu_healthy ? 1 : 0) | (pressure_healthy ? 2 : 0) | (temp_healthy ? 4 : 0);
  status.checksum = calculateChecksum((uint8_t*)&status, sizeof(StatusPacket));

  Wire.write((uint8_t*)&status, sizeof(StatusPacket));
}
