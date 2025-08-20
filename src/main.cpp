#include "Header Files/main.h"
#include "Header Files/Airplane.h"

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
FrameCallback frames[] = {drawFrame1};

Airplane& airplane = Airplane::getInstance();

static unsigned long lastDisplayUpdate = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Dual-Core Airplane Control System...");
  Serial.println("Setup running on Core: " + String(xPortGetCoreID()));

  pinMode(BUILTIN_LED, OUTPUT);

  // Create mutex for shared data protection
  xDataMutex = xSemaphoreCreateMutex();
  if (xDataMutex == NULL) {
    Serial.println("Failed to create mutex!");
    return;
  }

  // Initialize systems
  setupDisplay();
  setupRadio();
  airplane.initialize();

  // Create tasks for dual-core operation
  // Core 1: Real-time flight control (high priority)
  xTaskCreatePinnedToCore(FlightControlTaskCode,  // Task function
                          "FlightControl",        // Name
                          10000,                  // Stack size (words)
                          NULL,                   // Parameters
                          2,                      // Priority (higher = more important)
                          &FlightControlTask,     // Task handle
                          1                       // Core 1
  );

  // Core 0: Communication and display (lower priority)
  xTaskCreatePinnedToCore(CommunicationTaskCode,  // Task function
                          "Communication",        // Name
                          10000,                  // Stack size (words)
                          NULL,                   // Parameters
                          1,                      // Priority
                          &CommunicationTask,     // Task handle
                          0                       // Core 0
  );

  Serial.println("Dual-core tasks created successfully!");
}

void loop() {
  // Main loop now just manages the tasks
  // Core-specific work is handled by the task functions
  vTaskDelay(100 / portTICK_PERIOD_MS);  // Small delay to prevent watchdog issues
}

// =============================================================================
// CORE 1 TASK: Real-time Flight Control (High Priority)
// =============================================================================
void FlightControlTaskCode(void* pvParameters) {
  Serial.println("Flight Control Task started on Core: " + String(xPortGetCoreID()));

  // Task-specific variables
  static unsigned long lastControlUpdate = 0;
  static int controlUpdates = 0;

  while (true) {
    unsigned long taskStartTime = micros();  // For performance monitoring

    // Take mutex to safely access shared data
    if (xSemaphoreTake(xDataMutex, 20 / portTICK_PERIOD_MS) == pdTRUE) {
      // Check for timeout and handle emergency procedures
      if (millis() - lastRecievedTime >= timeoutInMilliSeconds) {
        // Emergency flight control - critical safety function
        // Serial.println("[CORE 1] Emergency mode activated!");
        ACS();
      } else {
        // Normal flight control - apply received control inputs
        airplane.setThrottle(engineReceived);
        airplane.setAilerons(aileronReceived);
        airplane.setElevators(elevatorsReceived);
        airplane.setRudder(rudderReceived);
        airplane.setFlaps(flapsRecieved);
        airplane.setElevatorTrim(elevatorTrimReceived);
        airplane.setAileronTrim(aileronTrimReceived);
        airplane.setLandingAirbrake(airBrakeReceived);

        // Handle trim resets
        if (resetAileronTrim)
          airplane.resetAileronTrim();
        if (resetElevatorTrim)
          airplane.resetElevatorTrim();
      }

      // Update airplane systems
      airplane.update();

      // Release mutex
      xSemaphoreGive(xDataMutex);

      controlUpdates++;
    } else {
      Serial.println("[CORE 1] Warning: Could not acquire mutex!");
    }

    // Performance monitoring
    unsigned long taskDuration = micros() - taskStartTime;
    if (millis() - lastControlUpdate > 1000) {
      Serial.println("[CORE 1] Control loop: " + String(controlUpdates) + " Hz, avg " + String(taskDuration) + "μs");
      controlUpdates = 0;
      lastControlUpdate = millis();
    }

    // Run at ~100Hz for responsive flight control
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// =============================================================================
// CORE 0 TASK: Communication and Display (Lower Priority)
// =============================================================================
void CommunicationTaskCode(void* pvParameters) {
  Serial.println("Communication Task started on Core: " + String(xPortGetCoreID()));

  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastCommUpdate = 0;
  static int commUpdates = 0;

  while (true) {
    unsigned long taskStartTime = micros();

    // Handle LoRa communication (non-blocking)
    // LoRa callbacks will handle received data automatically

    // Update display at ~20Hz to avoid flickering
    if (millis() - lastDisplayUpdate >= 50) {
      display.update();
      lastDisplayUpdate = millis();
    }

    // Print performance info periodically
    printTaskInfo();

    // You can add other communication tasks here:
    // - WiFi management
    // - Bluetooth handling
    // - SD card logging
    // - Data telemetry

    // Performance monitoring
    commUpdates++;
    unsigned long taskDuration = micros() - taskStartTime;
    if (millis() - lastCommUpdate > 1000) {
      Serial.println("[CORE 0] Communication: " + String(commUpdates) + " Hz, avg " + String(taskDuration) + "μs");
      commUpdates = 0;
      lastCommUpdate = millis();
    }

    // Run at ~50Hz - adequate for communication and display
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// =============================================================================
// PERFORMANCE MONITORING
// =============================================================================
void printTaskInfo() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 5000) {  // Print every 5 seconds
    Serial.println("=== DUAL-CORE PERFORMANCE ===");
    Serial.println("Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
    Serial.print("Flight Control Task (Core 1): " + String(uxTaskGetStackHighWaterMark(FlightControlTask)) + " words free");
    Serial.println(" | Communication Task (Core 0): " + String(uxTaskGetStackHighWaterMark(CommunicationTask)) + " words free");
    Serial.println("=============================");
    lastPrint = millis();
  }
}

void ACS() {
  // airplane.resetToSafeDefaults();  // Center the servos
}

void setupSD() {}

void setupRadio() {
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  while (!LoRa.begin(frequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    delay(200);
  }

  Serial.println("LoRa init succeeded.");
  Serial.println();
  Serial.println("LoRa Simple Node");
  Serial.println("Only receive messages from gateways");
  Serial.println("Tx: invertIQ disable");
  Serial.println("Rx: invertIQ enable");
  Serial.println();

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
}

// The ESP is capable of rendering 60fps in 80Mhz mode
// but that won't give you much time for anything else
// run it in 160Mhz mode or just set it to 30 fps
void setupDisplay() {
  display.setTargetFPS(60);

  // Customize the active and inactive symbol
  display.setActiveSymbol(activeSymbol);
  display.setInactiveSymbol(inactiveSymbol);

  // You can change this to
  // TOP, LEFT, BOTTOM, RIGHT
  display.setIndicatorPosition(BOTTOM);

  // Defines where the first frame is located in the bar.
  display.setIndicatorDirection(LEFT_RIGHT);

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  display.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  display.setFrames(frames, frameCount);
  display.disableAutoTransition();
  display.disableAllIndicators();

  // Add overlays
  // display.setOverlays(all, 2);

  // Initialising the UI will init the display too.
  ui.init();

  ui.flipScreenVertically();

  ui.setTextAlignment(TEXT_ALIGN_LEFT);
  ui.setFont(ArialMT_Plain_10);
}
