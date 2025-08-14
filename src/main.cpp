#include "Header Files\main.h"
#include "Header Files/Airplane.h"

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

void setup() {
  Serial.begin(115200);
  pinMode(BUILTIN_LED, OUTPUT);

  setupDisplay();
  setupRadio();

  // Initialize airplane with servos
  airplane.initialize();  // This now handles servo setup
}

void loop() {
  // Display
  int remainingTimeBudget = display.update();

  if (remainingTimeBudget > 0) {
    // You can do some work here
    // Don't do stuff if you are below your time budget.

    delay(remainingTimeBudget);
  }

  if (millis() - lastRecievedTime >= timeoutInMilliSeconds) {
    Serial.println("No message received in the last " + String(millis() - lastRecievedTime) + "ms");

    // Use Airplane class for emergency shutdown
    airplane.emergencyShutdown();

    ACS();
    delay(20);
  } else {
    // Update the controls using Airplane class
    airplane.setThrottle(engineRecieved);
    airplane.setAilerons(aileronRecieved);     // Aileron inverted
    airplane.setTrim(trimRecieved);            // Set trim value
    airplane.setElevators(elevatorsRecieved);  // Left elevator value (right will be auto-inverted)
    airplane.setRudder(rudderRecieved);
  }
}

void ACS() {
  airplane.resetToSafeDefaults();  // Center the servos
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
  display.setTargetFPS(30);

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
