#ifndef FLIGHT_PROTOCOL_H
#define FLIGHT_PROTOCOL_H

#include <Arduino.h>

// I2C Configuration
#define SERVO_CONTROLLER_ADDRESS 0x08
#define FLIGHT_DATA_REQUEST 0x10
#define SERVO_COMMAND 0x20
#define STATUS_REQUEST 0x30

// Flight control packet structure
struct ServoCommandPacket {
  uint8_t header;         // Packet type identifier
  uint8_t engine;         // Engine throttle 0-180
  uint8_t roll;           // Roll ailerons position
  uint8_t elevators;      // Elevators position
  uint8_t rudder;         // Rudder position
  uint8_t trim_elevator;  // Elevator trim
  uint8_t trim_aileron;   // Aileron trim
  uint8_t flaps;          // Flap position
  bool landingAirbrake;   // Airbrake state
  uint8_t checksum;       // Simple checksum for validation
} __attribute__((packed));

// Flight data from sensors (Nano 33 BLE → TTGO)
struct FlightDataPacket {
  uint8_t header;        // Packet type identifier
  float roll;            // Roll angle in degrees
  float pitch;           // Pitch angle in degrees
  float yaw;             // Yaw angle in degrees
  float altitude;        // Barometric altitude
  float temperature;     // Temperature in Celsius
  float pressure;        // Atmospheric pressure
  uint8_t servo_status;  // Servo health status bits
  uint8_t checksum;      // Simple checksum
} __attribute__((packed));

// Status packet from slave
struct StatusPacket {
  uint8_t header;
  bool servos_healthy;
  uint8_t last_command_received;
  uint32_t uptime_ms;
  uint8_t imu_status;
  uint8_t checksum;
} __attribute__((packed));

// Utility functions
inline uint8_t calculateChecksum(uint8_t* data, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length - 1; i++) {  // -1 to exclude checksum byte
    checksum ^= data[i];
  }
  return checksum;
}

inline bool validateChecksum(uint8_t* data, size_t length) {
  uint8_t calculated = calculateChecksum(data, length);
  return (calculated == data[length - 1]);
}

#endif  // FLIGHT_PROTOCOL_H
