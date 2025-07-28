#include "Header Files\main.h"

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

void setup() {
  Serial.begin(115200);

  pinMode(BUILTIN_LED, OUTPUT);

  setupDisplay();
  // setupSD();
  setupRadio();

  engine.attach(ENGINE_PIN, 1000, 2000);
  rollLeftMotor.attach(ROLL_LEFT_MOTOR_PIN);
  // rollRightMotor.attach(ROLL_RIGHT_MOTOR_PIN);

  elevationLeftMotor.attach(ELEVATION_LEFT_MOTOR_PIN);
  elevationRightMotor.attach(ELEVATION_RIGHT_MOTOR_PIN);

  rudderMotor.attach(RUDDER_MOTOR_PIN);
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

    engine.write(0);          // Turn off the engine
    rollLeftMotor.write(90);  // Center the ailerons
    // rollRightMotor.write(90);       // Center the ailerons
    elevationLeftMotor.write(90);   // Center the elevators
    elevationRightMotor.write(90);  // Center the elevators
    rudderMotor.write(90);          // Center the rudder

    ACS();
    delay(20);
  } else {
    // Update the motors
    engine.write(engineRecieved);
    rollLeftMotor.write(aileronRecieved);
    // rollRightMotor.write(180 - aileronRecieved);

    elevationLeftMotor.write(elevatorsRecieved);
    elevationRightMotor.write(180 - elevatorsRecieved);
    rudderMotor.write(map(rudderRecieved, 0, 180, 90 - rudderHalfAngleFreedom, 90 + rudderHalfAngleFreedom));
  }
}

void ACS() {
  // Center the servos
  engine.write(0);          // Turn off the engine
  rollLeftMotor.write(90);  // Center the ailerons
  // rollRightMotor.write(90);       // Center the ailerons
  elevationLeftMotor.write(90);   // Center the elevators
  elevationRightMotor.write(90);  // Center the elevators
  rudderMotor.write(90);          // Center the rudder

  Serial.println("All servos centered");
}

void setupSD() {
  // SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  // if (!SD.begin(SD_CS)) {
  //   Serial.println("Card Mount Failed");
  //   return;
  // }

  // uint8_t cardType = SD.cardType();

  // if (cardType == CARD_NONE) {
  //   Serial.println("No SD card attached");
  //   return;
  // }

  // Serial.print("SD Card Type: ");
  // if (cardType == CARD_MMC)
  //   Serial.println("MMC");
  // else if (cardType == CARD_SD)
  //   Serial.println("SDSC");
  // else if (cardType == CARD_SDHC)
  //   Serial.println("SDHC");
  // else
  //   Serial.println("UNKNOWN");

  // uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  // Serial.printf("SD Card Size: %lluMB\n", cardSize);

  // listDir(SD, "/", 0);
  // createDir(SD, "/mydir");
  // listDir(SD, "/", 0);
  // removeDir(SD, "/mydir");
  // listDir(SD, "/", 2);
  // writeFile(SD, "/hello.txt", "Hello ");
  // appendFile(SD, "/hello.txt", "World!\n");
  // readFile(SD, "/hello.txt");
  // deleteFile(SD, "/foo.txt");
  // renameFile(SD, "/hello.txt", "/foo.txt");
  // readFile(SD, "/foo.txt");
  // testFileIO(SD, "/test.txt");
  // Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  // Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

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
