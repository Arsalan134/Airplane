#include "AirplaneMaster.h"  // Use the new master class instead
#include "main.h"

// Forward declarations
void printTaskInfo();
void ACS();

// Task handles for dual-core operation
TaskHandle_t FlightControlTask;
TaskHandle_t CommunicationTask;

// Shared data protection
SemaphoreHandle_t xDataMutex;

// Display
SSD1306Wire ui(0x3c, SDA, SCL);
OLEDDisplayUi display(&ui);

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback all[] = {bluetoothOverlay, wifiOverlay};
OverlayCallback wifiOverlays[] = {wifiOverlay};
OverlayCallback bluetoothOverlays[] = {bluetoothOverlay};

// This array keeps function pointers to all frames
// frames are the single views that slide in
FrameCallback frames[] = {drawFrame1, drawFrame2, drawFrame3};  // Add sensor data frames

AirplaneMaster& airplane = AirplaneMaster::getInstance();  // Use the new master class

static unsigned long lastDisplayUpdate = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("✈️ Starting Master-Slave Airplane Control System... 🚁");
  Serial.println("🔧 Setup running on Core: " + String(xPortGetCoreID()));

  pinMode(BUILTIN_LED, OUTPUT);

  // Create mutex for shared data protection
  xDataMutex = xSemaphoreCreateMutex();
  if (xDataMutex == NULL) {
    Serial.println("❌ Failed to create mutex!");
    return;
  }

  // Initialize systems
  setupDisplay();
  setupRadio();

  // Initialize the master controller (this will establish I2C communication with slave)
  airplane.initialize();

  // Create tasks for dual-core operation
  Serial.println("🚀 Creating dual-core tasks...");

  // Communication and LoRa task on Core 0
  xTaskCreatePinnedToCore(communicationTask,   // Task function
                          "Communication",     // Name of task
                          10000,               // Stack size of task
                          NULL,                // Parameter of the task
                          2,                   // Priority of the task (higher = more priority)
                          &CommunicationTask,  // Task handle to keep track of created task
                          0);                  // Pin task to core 0

  // Flight control and servo task on Core 1
  xTaskCreatePinnedToCore(flightControlTask,   // Task function
                          "FlightControl",     // Name of task
                          10000,               // Stack size of task
                          NULL,                // Parameter of the task
                          3,                   // Priority of the task (higher priority for safety)
                          &FlightControlTask,  // Task handle to keep track of created task
                          1);                  // Pin task to core 1

  Serial.println("✅ System initialization complete!");
  Serial.println("📡 Master controller ready - waiting for slave...");

  // Check slave status
  delay(1000);
  if (airplane.isSlaveHealthy()) {
    Serial.println("🤝 Slave communication established!");
  } else {
    Serial.println("⚠️ Slave not responding - check connections");
  }
}

void loop() {
  // Main loop is light - most work done in tasks

  // Update display every 200ms
  if (millis() - lastDisplayUpdate >= 200) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  // Print system status every 5 seconds
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint >= 5000) {
    printSystemStatus();
    lastStatusPrint = millis();
  }

  // LED heartbeat
  static bool ledState = false;
  static unsigned long lastLedToggle = 0;
  if (millis() - lastLedToggle >= (airplane.isSlaveHealthy() ? 1000 : 200)) {
    digitalWrite(BUILTIN_LED, ledState);
    ledState = !ledState;
    lastLedToggle = millis();
  }

  delay(50);
}

// Communication task - Core 0
void communicationTask(void* pvParameters) {
  Serial.println("📡 Communication task started on Core: " + String(xPortGetCoreID()));

  while (true) {
    // Handle LoRa communication
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
      // Check for incoming LoRa messages
      handleIncomingMessages();

      // Send telemetry data
      static unsigned long lastTelemetry = 0;
      if (millis() - lastTelemetry >= 1000) {  // Send telemetry every 1 second
        sendTelemetryData();
        lastTelemetry = millis();
      }

      xSemaphoreGive(xDataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz communication loop
  }
}

// Flight control task - Core 1
void flightControlTask(void* pvParameters) {
  Serial.println("🛩️ Flight control task started on Core: " + String(xPortGetCoreID()));

  while (true) {
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
      // Update the airplane master controller
      // This handles I2C communication with the servo slave
      airplane.update();

      // Handle any flight control logic
      handleFlightControl();

      xSemaphoreGive(xDataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz flight control loop
  }
}

void handleIncomingMessages() {
  // Check for LoRa packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }

    // Parse and process the message
    processReceivedMessage(received);

    // Update connection status
    airplane.updateLastReceivedTime();

    Serial.println("📨 Received: " + received + " RSSI: " + String(LoRa.packetRssi()));
  }
}

void sendTelemetryData() {
  // Get flight data from slave
  FlightDataPacket flightData;
  if (airplane.getFlightData(flightData)) {
    // Create telemetry message
    String telemetry = "TEL:";
    telemetry += "R:" + String(flightData.roll, 1) + ",";
    telemetry += "P:" + String(flightData.pitch, 1) + ",";
    telemetry += "Y:" + String(flightData.yaw, 1) + ",";
    telemetry += "A:" + String(flightData.altitude, 1) + ",";
    telemetry += "T:" + String(flightData.temperature, 1) + ",";
    telemetry += "S:" + String((airplane.isSlaveHealthy() ? "OK" : "ERR"));

    // Send via LoRa
    LoRa.beginPacket();
    LoRa.print(telemetry);
    LoRa.endPacket();

    Serial.println("📤 Telemetry sent: " + telemetry);
  }
}

void processReceivedMessage(String message) {
  // Parse control commands from ground station
  if (message.startsWith("CMD:")) {
    String command = message.substring(4);
    parseControlCommand(command);
  } else if (message.startsWith("SET:")) {
    String setting = message.substring(4);
    parseSettingCommand(setting);
  }
}

void parseControlCommand(String command) {
  // Example: "E:50,R:90,EL:85,RU:95" (Engine, Roll, Elevator, Rudder)

  int enginePos = command.indexOf("E:");
  int rollPos = command.indexOf("R:");
  int elevatorPos = command.indexOf("EL:");
  int rudderPos = command.indexOf("RU:");

  if (enginePos >= 0) {
    int endPos = command.indexOf(",", enginePos);
    if (endPos < 0)
      endPos = command.length();
    int value = command.substring(enginePos + 2, endPos).toInt();
    airplane.setThrottle(value);
  }

  if (rollPos >= 0) {
    int endPos = command.indexOf(",", rollPos);
    if (endPos < 0)
      endPos = command.length();
    int value = command.substring(rollPos + 2, endPos).toInt();
    airplane.setAilerons(value);
  }

  if (elevatorPos >= 0) {
    int endPos = command.indexOf(",", elevatorPos);
    if (endPos < 0)
      endPos = command.length();
    int value = command.substring(elevatorPos + 3, endPos).toInt();
    airplane.setElevators(value);
  }

  if (rudderPos >= 0) {
    int endPos = command.indexOf(",", rudderPos);
    if (endPos < 0)
      endPos = command.length();
    int value = command.substring(rudderPos + 3, endPos).toInt();
    airplane.setRudder(value);
  }

  Serial.println("🎮 Control command processed: " + command);
}

void parseSettingCommand(String setting) {
  // Handle setting commands like trim, flight mode, etc.
  if (setting.startsWith("MODE:")) {
    String mode = setting.substring(5);
    // Set flight mode based on received command
    Serial.println("🎯 Flight mode: " + mode);
  } else if (setting.startsWith("TRIM:")) {
    String trim = setting.substring(5);
    // Handle trim adjustments
    Serial.println("⚖️ Trim adjustment: " + trim);
  }
}

void handleFlightControl() {
  // Add any automated flight control logic here
  // For example, stability augmentation using IMU data

  static unsigned long lastStabilityCheck = 0;
  if (millis() - lastStabilityCheck >= 100) {  // Check every 100ms

    // Get current attitude from slave
    float currentRoll = airplane.getCurrentRoll();
    float currentPitch = airplane.getCurrentPitch();

    // Simple stability augmentation (if enabled)
    if (airplane.getFlightMode() == FlightMode::STABILITY) {
      // Add small corrections to maintain level flight
      // This is a simple example - real implementation would be more sophisticated

      if (abs(currentRoll) > 5.0) {  // More than 5 degrees roll
        // Add small aileron correction
        // Implementation depends on your control system
      }

      if (abs(currentPitch) > 5.0) {  // More than 5 degrees pitch
        // Add small elevator correction
        // Implementation depends on your control system
      }
    }

    lastStabilityCheck = millis();
  }
}

void updateDisplay() {
  // Update OLED display with current status and sensor data
  // You can add frames to show:
  // - Current control positions
  // - IMU data (roll, pitch, yaw)
  // - Altitude and temperature
  // - Communication status
  // - Slave health status

  display.update();
}

void printSystemStatus() {
  Serial.println("=== SYSTEM STATUS ===");
  Serial.println("Master Controller: " + airplane.getStatusString());
  Serial.println("Slave Healthy: " + String(airplane.isSlaveHealthy() ? "✅" : "❌"));
  Serial.println("Connection: " + String(airplane.isConnectionActive() ? "✅" : "❌"));
  Serial.println("Roll: " + String(airplane.getCurrentRoll(), 1) + "°");
  Serial.println("Pitch: " + String(airplane.getCurrentPitch(), 1) + "°");
  Serial.println("Altitude: " + String(airplane.getCurrentAltitude(), 1) + "m");
  Serial.println("Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
  Serial.println("====================");
}

// Include your existing setup functions
void setupDisplay() {
  // Initialize OLED display
  ui.init();
  ui.flipScreenVertically();

  // Set display parameters
  display.setTargetFPS(30);
  display.setActiveSymbol(activeSymbol);
  display.setInactiveSymbol(inactiveSymbol);
  display.setIndicatorPosition(BOTTOM);
  display.setIndicatorDirection(LEFT_RIGHT);
  display.setFrameAnimation(SLIDE_LEFT);
  display.setFrames(frames, sizeof(frames) / sizeof(frames[0]));
  display.setOverlays(all, sizeof(all) / sizeof(all[0]));

  display.init();

  Serial.println("📺 Display initialized");
}

void setupRadio() {
  // Initialize LoRa
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(18, 23, 26);

  if (!LoRa.begin(915E6)) {  // Adjust frequency as needed
    Serial.println("❌ LoRa initialization failed!");
    while (1)
      ;
  }

  Serial.println("📡 LoRa initialized");
}
