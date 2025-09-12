// /**
//  * @file SD-Card.cpp
//  * @brief Advanced SD Card Management and Data Logging Implementation
//  * @author RC Airplane Project
//  * @date 2025
//  */

// #include "SD-Card.h"
// #include <time.h>

// // Global logger instance
// SDCardLogger sdLogger;

// // Constructor
// SDCardLogger::SDCardLogger() : initialized(false), session_start_time(0) {
//   flight_buffer.reserve(100);
//   telemetry_buffer.reserve(50);
//   event_buffer.reserve(30);
// }

// // Destructor
// SDCardLogger::~SDCardLogger() {
//   end();
// }

// // Initialize SD card
// bool SDCardLogger::begin() {
//   Serial.println("🔧 Initializing SD Card Logger...");

//   // Initialize SPI for SD card
//   SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

//   if (!SD.begin(SD_CS_PIN)) {
//     Serial.println("❌ SD Card initialization failed!");
//     return false;
//   }

//   Serial.println("✅ SD Card initialized successfully");

//   // Check card type and size
//   uint8_t cardType = SD.cardType();
//   if (cardType == CARD_NONE) {
//     Serial.println("❌ No SD card attached");
//     return false;
//   }

//   Serial.print("📱 SD Card Type: ");
//   if (cardType == CARD_MMC) {
//     Serial.println("MMC");
//   } else if (cardType == CARD_SD) {
//     Serial.println("SDSC");
//   } else if (cardType == CARD_SDHC) {
//     Serial.println("SDHC");
//   } else {
//     Serial.println("UNKNOWN");
//   }

//   uint64_t cardSize = SD.cardSize() / (1024 * 1024);
//   Serial.printf("💾 SD Card Size: %lluMB\n", cardSize);

//   setupLogDirectories();
//   initialized = true;

//   // Log initialization
//   logInfo("SD_CARD", "SD Card Logger initialized", "Card size: " + String((unsigned long)cardSize) + "MB, Type: " + String(cardType));

//   return true;
// }

// // End SD card operations
// void SDCardLogger::end() {
//   if (initialized) {
//     flushAllBuffers();
//     logInfo("SD_CARD", "SD Card Logger shutting down");
//     initialized = false;
//   }
// }

// // Start new logging session
// bool SDCardLogger::startNewSession() {
//   if (!initialized)
//     return false;

//   current_session_id = generateSessionId();
//   session_start_time = millis();

//   Serial.println("🚀 Starting new flight session: " + current_session_id);

//   // Create session directory
//   String session_dir = "/flights/" + current_session_id;
//   if (!SD.mkdir(session_dir.c_str())) {
//     Serial.println("⚠️ Warning: Could not create session directory");
//   }

//   logInfo("SESSION", "New flight session started", "Session ID: " + current_session_id);
//   return true;
// }

// // End current session
// bool SDCardLogger::endSession() {
//   if (!initialized || current_session_id.isEmpty())
//     return false;

//   flushAllBuffers();

//   unsigned long flight_duration = millis() - session_start_time;
//   logInfo("SESSION", "Flight session ended", "Duration: " + String(flight_duration) + "ms");

//   generateFlightSummary(current_session_id);

//   current_session_id = "";
//   session_start_time = 0;

//   return true;
// }

// // Log flight data
// bool SDCardLogger::logFlightData(const FlightData& data) {
//   if (!initialized)
//     return false;

//   flight_buffer.push_back(data);

//   // Flush buffer if full
//   if (flight_buffer.size() >= LOG_BUFFER_SIZE / sizeof(FlightData)) {
//     flushFlightBuffer();
//   }

//   return true;
// }

// // Log flight data in CSV format
// bool SDCardLogger::logFlightDataCSV(const FlightData& data) {
//   if (!initialized || current_session_id.isEmpty())
//     return false;

//   String filename = "/flights/" + current_session_id + "/flight_data.csv";

//   // Check if file exists, if not create with header
//   if (!SD.exists(filename.c_str())) {
//     File file = SD.open(filename.c_str(), FILE_WRITE);
//     if (file) {
//       file.print(CSV_HEADER_FLIGHT);
//       file.close();
//     }
//   }

//   File file = SD.open(filename.c_str(), FILE_APPEND);
//   if (!file) {
//     Serial.println("❌ Failed to open flight data CSV file");
//     return false;
//   }

//   // Write CSV data
//   file.printf("%lu,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%d,%.2f,%.1f,%.2f,%.6f,%.6f,%.1f\n", data.timestamp, data.altitude, data.speed,
//               data.heading, data.roll, data.pitch, data.yaw, data.throttle, data.battery_voltage, data.temperature, data.pressure,
//               data.gps_lat, data.gps_lon, data.gps_alt);

//   file.close();
//   return true;
// }

// // Log flight data in JSON format
// bool SDCardLogger::logFlightDataJSON(const FlightData& data) {
//   if (!initialized || current_session_id.isEmpty())
//     return false;

//   String filename = "/flights/" + current_session_id + "/flight_data.json";

//   JsonDocument doc;
//   doc["timestamp"] = data.timestamp;
//   doc["altitude"] = data.altitude;
//   doc["speed"] = data.speed;
//   doc["heading"] = data.heading;
//   doc["attitude"]["roll"] = data.roll;
//   doc["attitude"]["pitch"] = data.pitch;
//   doc["attitude"]["yaw"] = data.yaw;
//   doc["controls"]["throttle"] = data.throttle;
//   doc["systems"]["battery_voltage"] = data.battery_voltage;
//   doc["sensors"]["temperature"] = data.temperature;
//   doc["sensors"]["pressure"] = data.pressure;
//   doc["gps"]["latitude"] = data.gps_lat;
//   doc["gps"]["longitude"] = data.gps_lon;
//   doc["gps"]["altitude"] = data.gps_alt;

//   File file = SD.open(filename.c_str(), FILE_APPEND);
//   if (!file) {
//     Serial.println("❌ Failed to open flight data JSON file");
//     return false;
//   }

//   serializeJson(doc, file);
//   file.println();
//   file.close();

//   return true;
// }

// // Log telemetry data
// bool SDCardLogger::logTelemetryData(const TelemetryData& data) {
//   if (!initialized)
//     return false;

//   telemetry_buffer.push_back(data);

//   // Flush buffer if full
//   if (telemetry_buffer.size() >= LOG_BUFFER_SIZE / sizeof(TelemetryData)) {
//     flushTelemetryBuffer();
//   }

//   return true;
// }

// // Log telemetry data in CSV format
// bool SDCardLogger::logTelemetryCSV(const TelemetryData& data) {
//   if (!initialized || current_session_id.isEmpty())
//     return false;

//   String filename = "/flights/" + current_session_id + "/telemetry.csv";

//   // Check if file exists, if not create with header
//   if (!SD.exists(filename.c_str())) {
//     File file = SD.open(filename.c_str(), FILE_WRITE);
//     if (file) {
//       file.print(CSV_HEADER_TELEMETRY);
//       file.close();
//     }
//   }

//   File file = SD.open(filename.c_str(), FILE_APPEND);
//   if (!file) {
//     Serial.println("❌ Failed to open telemetry CSV file");
//     return false;
//   }

//   file.printf("%lu,%d,%.2f,%.1f,%s,%s,%lu\n", data.timestamp, data.rssi, data.snr, data.packet_loss, data.command_type.c_str(),
//               data.ack_received ? "true" : "false", data.response_time);

//   file.close();
//   return true;
// }

// // Log event with specific level
// bool SDCardLogger::logEvent(LogLevel level, const String& module, const String& message, const String& details) {
//   if (!initialized)
//     return false;

//   SystemEvent event;
//   event.timestamp = millis();
//   event.module = module;
//   event.message = message;
//   event.details = details;

//   switch (level) {
//     case LogLevel::DEBUG:
//       event.level = "DEBUG";
//       break;
//     case LogLevel::INFO:
//       event.level = "INFO";
//       break;
//     case LogLevel::WARNING:
//       event.level = "WARNING";
//       break;
//     case LogLevel::ERROR:
//       event.level = "ERROR";
//       break;
//     case LogLevel::CRITICAL:
//       event.level = "CRITICAL";
//       break;
//   }

//   event_buffer.push_back(event);

//   // Also print to Serial for immediate feedback
//   Serial.printf("[%s] %s: %s", event.level.c_str(), module.c_str(), message.c_str());
//   if (!details.isEmpty()) {
//     Serial.printf(" (%s)", details.c_str());
//   }
//   Serial.println();

//   // Flush buffer if full or if critical error
//   if (event_buffer.size() >= LOG_BUFFER_SIZE / sizeof(SystemEvent) || level == LogLevel::CRITICAL) {
//     flushEventBuffer();
//   }

//   return true;
// }

// // Convenience logging methods
// bool SDCardLogger::logInfo(const String& module, const String& message, const String& details) {
//   return logEvent(LogLevel::INFO, module, message, details);
// }

// bool SDCardLogger::logWarning(const String& module, const String& message, const String& details) {
//   return logEvent(LogLevel::WARNING, module, message, details);
// }

// bool SDCardLogger::logError(const String& module, const String& message, const String& details) {
//   return logEvent(LogLevel::ERROR, module, message, details);
// }

// bool SDCardLogger::logCritical(const String& module, const String& message, const String& details) {
//   return logEvent(LogLevel::CRITICAL, module, message, details);
// }

// // Flush flight data buffer
// void SDCardLogger::flushFlightBuffer() {
//   if (flight_buffer.empty() || current_session_id.isEmpty())
//     return;

//   for (const auto& data : flight_buffer) {
//     logFlightDataCSV(data);
//   }

//   flight_buffer.clear();
//   Serial.printf("📝 Flushed %d flight data records\n", flight_buffer.size());
// }

// // Flush telemetry buffer
// void SDCardLogger::flushTelemetryBuffer() {
//   if (telemetry_buffer.empty() || current_session_id.isEmpty())
//     return;

//   for (const auto& data : telemetry_buffer) {
//     logTelemetryCSV(data);
//   }

//   telemetry_buffer.clear();
//   Serial.printf("📡 Flushed %d telemetry records\n", telemetry_buffer.size());
// }

// // Flush event buffer
// void SDCardLogger::flushEventBuffer() {
//   if (event_buffer.empty())
//     return;

//   String filename = "/logs/system_events.json";
//   File file = SD.open(filename.c_str(), FILE_APPEND);

//   if (file) {
//     for (const auto& event : event_buffer) {
//       JsonDocument doc;
//       doc["timestamp"] = event.timestamp;
//       doc["level"] = event.level;
//       doc["module"] = event.module;
//       doc["message"] = event.message;
//       doc["details"] = event.details;

//       serializeJson(doc, file);
//       file.println();
//     }
//     file.close();
//   }

//   event_buffer.clear();
// }

// // Flush all buffers
// void SDCardLogger::flushAllBuffers() {
//   flushFlightBuffer();
//   flushTelemetryBuffer();
//   flushEventBuffer();
// }

// // Generate flight summary
// bool SDCardLogger::generateFlightSummary(const String& session_id) {
//   if (!initialized || session_id.isEmpty())
//     return false;

//   String csv_file = "/flights/" + session_id + "/flight_data.csv";
//   String summary_file = "/flights/" + session_id + "/flight_summary.json";

//   if (!SD.exists(csv_file.c_str())) {
//     Serial.println("⚠️ No flight data found for summary");
//     return false;
//   }

//   // Read and analyze flight data
//   File file = SD.open(csv_file.c_str(), FILE_READ);
//   if (!file)
//     return false;

//   JsonDocument summary;
//   summary["session_id"] = session_id;
//   summary["analysis_timestamp"] = millis();

//   // Skip header
//   file.readStringUntil('\n');

//   float max_altitude = 0, min_altitude = 999999;
//   float max_speed = 0;
//   float total_distance = 0;
//   int data_points = 0;

//   while (file.available()) {
//     String line = file.readStringUntil('\n');
//     if (line.length() < 10)
//       continue;

//     // Parse CSV line (simplified)
//     int comma_pos = line.indexOf(',');
//     if (comma_pos == -1)
//       continue;

//     line = line.substring(comma_pos + 1);  // Skip timestamp
//     comma_pos = line.indexOf(',');
//     float altitude = line.substring(0, comma_pos).toFloat();

//     max_altitude = max(max_altitude, altitude);
//     min_altitude = min(min_altitude, altitude);
//     data_points++;
//   }

//   file.close();

//   summary["statistics"]["max_altitude"] = max_altitude;
//   summary["statistics"]["min_altitude"] = min_altitude;
//   summary["statistics"]["max_speed"] = max_speed;
//   summary["statistics"]["data_points"] = data_points;
//   summary["statistics"]["flight_duration"] = millis() - session_start_time;

//   // Write summary
//   File summary_f = SD.open(summary_file.c_str(), FILE_WRITE);
//   if (summary_f) {
//     serializeJsonPretty(summary, summary_f);
//     summary_f.close();
//     Serial.println("📊 Flight summary generated: " + summary_file);
//     return true;
//   }

//   return false;
// }

// // Print storage information
// void SDCardLogger::printStorageInfo() {
//   if (!initialized) {
//     Serial.println("❌ SD Card not initialized");
//     return;
//   }

//   uint64_t cardSize = SD.cardSize() / (1024 * 1024);
//   uint64_t totalBytes = SD.totalBytes() / (1024 * 1024);
//   uint64_t usedBytes = SD.usedBytes() / (1024 * 1024);

//   Serial.println("\n📊 SD Card Storage Information:");
//   Serial.printf("   Card Size: %lluMB\n", cardSize);
//   Serial.printf("   Total Space: %lluMB\n", totalBytes);
//   Serial.printf("   Used Space: %lluMB\n", usedBytes);
//   Serial.printf("   Free Space: %lluMB\n", totalBytes - usedBytes);
//   Serial.printf("   Usage: %.1f%%\n", (float)usedBytes / totalBytes * 100);
// }

// // List log files
// bool SDCardLogger::listLogFiles() {
//   if (!initialized)
//     return false;

//   Serial.println("\n📁 Log Files:");
//   listDir(SD, "/flights", 2);
//   listDir(SD, "/logs", 1);

//   return true;
// }

// // Utility functions
// String generateSessionId() {
//   return "FLIGHT_" + String(millis()) + "_" + String(random(1000, 9999));
// }

// bool initializeSDCard() {
//   return sdLogger.begin();
// }

// void setupLogDirectories() {
//   SD.mkdir("/flights");
//   SD.mkdir("/logs");
//   SD.mkdir("/config");
//   SD.mkdir("/export");
// }

// // Legacy functions for backward compatibility
// void listDir(fs::FS& fs, const char* dirname, uint8_t levels) {
//   Serial.printf("📁 Listing directory: %s\n", dirname);

//   File root = fs.open(dirname);
//   if (!root) {
//     Serial.println("❌ Failed to open directory");
//     return;
//   }
//   if (!root.isDirectory()) {
//     Serial.println("❌ Not a directory");
//     return;
//   }

//   File file = root.openNextFile();
//   while (file) {
//     if (file.isDirectory()) {
//       Serial.printf("  📁 DIR : %s\n", file.name());
//       if (levels) {
//         listDir(fs, file.name(), levels - 1);
//       }
//     } else {
//       Serial.printf("  📄 FILE: %-30s SIZE: %8d bytes\n", file.name(), file.size());
//     }
//     file = root.openNextFile();
//   }
// }

// void createDir(fs::FS& fs, const char* path) {
//   Serial.printf("📁 Creating Dir: %s\n", path);
//   if (fs.mkdir(path)) {
//     Serial.println("✅ Dir created");
//   } else {
//     Serial.println("❌ mkdir failed");
//   }
// }

// void removeDir(fs::FS& fs, const char* path) {
//   Serial.printf("🗑️ Removing Dir: %s\n", path);
//   if (fs.rmdir(path)) {
//     Serial.println("✅ Dir removed");
//   } else {
//     Serial.println("❌ rmdir failed");
//   }
// }

// void readFile(fs::FS& fs, const char* path) {
//   Serial.printf("📖 Reading file: %s\n", path);

//   File file = fs.open(path);
//   if (!file) {
//     Serial.println("❌ Failed to open file for reading");
//     return;
//   }

//   Serial.println("📄 File content:");
//   while (file.available()) {
//     Serial.write(file.read());
//   }
//   file.close();
// }

// void writeFile(fs::FS& fs, const char* path, const char* message) {
//   Serial.printf("✏️ Writing file: %s\n", path);

//   File file = fs.open(path, FILE_WRITE);
//   if (!file) {
//     Serial.println("❌ Failed to open file for writing");
//     return;
//   }
//   if (file.print(message)) {
//     Serial.println("✅ File written");
//   } else {
//     Serial.println("❌ Write failed");
//   }
//   file.close();
// }

// void appendFile(fs::FS& fs, const char* path, const char* message) {
//   Serial.printf("➕ Appending to file: %s\n", path);

//   File file = fs.open(path, FILE_APPEND);
//   if (!file) {
//     Serial.println("❌ Failed to open file for appending");
//     return;
//   }
//   if (file.print(message)) {
//     Serial.println("✅ Message appended");
//   } else {
//     Serial.println("❌ Append failed");
//   }
//   file.close();
// }

// void renameFile(fs::FS& fs, const char* path1, const char* path2) {
//   Serial.printf("🔄 Renaming file %s to %s\n", path1, path2);
//   if (fs.rename(path1, path2)) {
//     Serial.println("✅ File renamed");
//   } else {
//     Serial.println("❌ Rename failed");
//   }
// }

// void deleteFile(fs::FS& fs, const char* path) {
//   Serial.printf("🗑️ Deleting file: %s\n", path);
//   if (fs.remove(path)) {
//     Serial.println("✅ File deleted");
//   } else {
//     Serial.println("❌ Delete failed");
//   }
// }

// void testFileIO(fs::FS& fs, const char* path) {
//   Serial.printf("🧪 Testing file I/O with %s\n", path);

//   File file = fs.open(path);
//   static uint8_t buf[512];
//   size_t len = 0;
//   uint32_t start = millis();
//   uint32_t end = start;

//   if (file) {
//     len = file.size();
//     size_t flen = len;
//     start = millis();
//     while (len) {
//       size_t toRead = len;
//       if (toRead > 512) {
//         toRead = 512;
//       }
//       file.read(buf, toRead);
//       len -= toRead;
//     }
//     end = millis() - start;
//     Serial.printf("📖 %u bytes read in %u ms (%.2f KB/s)\n", flen, end, (float)flen / end);
//     file.close();
//   } else {
//     Serial.println("❌ Failed to open file for reading");
//   }

//   file = fs.open(path, FILE_WRITE);
//   if (!file) {
//     Serial.println("❌ Failed to open file for writing");
//     return;
//   }

//   size_t i;
//   start = millis();
//   for (i = 0; i < 2048; i++) {
//     file.write(buf, 512);
//   }
//   end = millis() - start;
//   Serial.printf("✏️ %u bytes written in %u ms (%.2f KB/s)\n", 2048 * 512, end, (float)(2048 * 512) / end);
//   file.close();
// }
