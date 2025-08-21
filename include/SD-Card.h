/**
 * @file SD-Card.h
 * @brief Advanced SD Card Management and Data Logging System
 * @author RC Airplane Project
 * @date 2025
 *
 * Features:
 * - High-performance data logging with CSV and JSON formats
 * - Flight data recording for analysis and plotting
 * - Circular buffer for continuous logging
 * - Error handling and recovery
 * - File rotation and size management
 * - Timestamp support with millisecond precision
 */

#pragma once

#include <ArduinoJson.h>
#include <map>
#include <vector>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// SD Card Configuration
#define SD_CS_PIN 13
#define SD_MOSI_PIN 15
#define SD_MISO_PIN 2
#define SD_SCK_PIN 14

// Logging Configuration
#define MAX_LOG_FILE_SIZE (5 * 1024 * 1024)  // 5MB max file size
#define MAX_LOG_FILES 10                     // Keep last 10 files
#define LOG_BUFFER_SIZE 1024                 // Buffer size for batch writes
#define CSV_HEADER_FLIGHT \
  "timestamp,altitude,speed,heading,roll,pitch,yaw,throttle,battery_voltage,temperature,pressure,gps_lat,gps_lon,gps_alt\n"
#define CSV_HEADER_TELEMETRY "timestamp,rssi,snr,packet_loss,command_type,ack_received,response_time\n"

// Data Structures
struct FlightData {
  unsigned long timestamp;
  float altitude;
  float speed;
  float heading;
  float roll;
  float pitch;
  float yaw;
  uint8_t throttle;
  float battery_voltage;
  float temperature;
  float pressure;
  double gps_lat;
  double gps_lon;
  float gps_alt;
};

struct TelemetryData {
  unsigned long timestamp;
  int rssi;
  float snr;
  float packet_loss;
  String command_type;
  bool ack_received;
  unsigned long response_time;
};

struct SystemEvent {
  unsigned long timestamp;
  String level;   // INFO, WARNING, ERROR, CRITICAL
  String module;  // LORA, GPS, SENSORS, SYSTEM
  String message;
  String details;
};

// Log Levels
enum class LogLevel { DEBUG = 0, INFO = 1, WARNING = 2, ERROR = 3, CRITICAL = 4 };

class SDCardLogger {
 private:
  bool initialized;
  String current_session_id;
  unsigned long session_start_time;
  std::vector<FlightData> flight_buffer;
  std::vector<TelemetryData> telemetry_buffer;
  std::vector<SystemEvent> event_buffer;

  // File management
  String getNextLogFileName(const String& prefix);
  bool rotateLogFile(const String& filepath);
  void cleanupOldFiles(const String& prefix, int max_files);
  String formatTimestamp(unsigned long timestamp);

  // Buffer management
  void flushFlightBuffer();
  void flushTelemetryBuffer();
  void flushEventBuffer();

 public:
  SDCardLogger();
  ~SDCardLogger();

  // Initialization and management
  bool begin();
  void end();
  bool isInitialized() const { return initialized; }
  String getSessionId() const { return current_session_id; }

  // Session management
  bool startNewSession();
  bool endSession();

  // Flight data logging
  bool logFlightData(const FlightData& data);
  bool logFlightDataCSV(const FlightData& data);
  bool logFlightDataJSON(const FlightData& data);

  // Telemetry logging
  bool logTelemetryData(const TelemetryData& data);
  bool logTelemetryCSV(const TelemetryData& data);
  bool logTelemetryJSON(const TelemetryData& data);

  // Event logging
  bool logEvent(LogLevel level, const String& module, const String& message, const String& details = "");
  bool logInfo(const String& module, const String& message, const String& details = "");
  bool logWarning(const String& module, const String& message, const String& details = "");
  bool logError(const String& module, const String& message, const String& details = "");
  bool logCritical(const String& module, const String& message, const String& details = "");

  // Data analysis helpers
  bool exportFlightDataForPlotting(const String& session_id, const String& output_file);
  bool generateFlightSummary(const String& session_id);

  // File operations
  bool listLogFiles();
  bool readLogFile(const String& filename);
  bool deleteLogFile(const String& filename);
  bool deleteOldLogs(int days_old);

  // Statistics
  void printStorageInfo();
  unsigned long getTotalFlightTime();
  int getFlightCount();

  // Force flush all buffers
  void flushAllBuffers();
};

// Legacy functions for backward compatibility
void listDir(fs::FS& fs, const char* dirname, uint8_t levels);
void createDir(fs::FS& fs, const char* path);
void removeDir(fs::FS& fs, const char* path);
void readFile(fs::FS& fs, const char* path);
void writeFile(fs::FS& fs, const char* path, const char* message);
void appendFile(fs::FS& fs, const char* path, const char* message);
void renameFile(fs::FS& fs, const char* path1, const char* path2);
void deleteFile(fs::FS& fs, const char* path);
void testFileIO(fs::FS& fs, const char* path);

// Utility functions
String generateSessionId();
bool initializeSDCard();
void setupLogDirectories();

// Global logger instance
extern SDCardLogger sdLogger;