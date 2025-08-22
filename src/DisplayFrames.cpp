// Additional display frames for showing sensor data from the slave
#include "AirplaneMaster.h"
#include "Display.h"
#include "common.h"

void drawFrame2(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  // IMU Data Frame
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);

  display->drawString(0 + x, 10 + y, "🧭 Flight Attitude");

  AirplaneMaster& airplane = AirplaneMaster::getInstance();

  display->setFont(ArialMT_Plain_16);
  display->drawString(0 + x, 25 + y, "Roll: " + String(airplane.getCurrentRoll(), 1) + "°");
  display->drawString(0 + x, 40 + y, "Pitch: " + String(airplane.getCurrentPitch(), 1) + "°");

  // Draw a simple artificial horizon indicator
  int centerX = 100 + x;
  int centerY = 35 + y;
  int radius = 15;

  // Draw horizon line (simplified)
  int rollOffset = (int)(airplane.getCurrentRoll() * 0.3);  // Scale down for display
  display->drawLine(centerX - radius, centerY + rollOffset, centerX + radius, centerY - rollOffset);

  // Draw aircraft symbol
  display->drawLine(centerX - 8, centerY, centerX + 8, centerY);
  display->drawLine(centerX, centerY - 3, centerX, centerY + 3);
}

void drawFrame3(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  // Environmental Data Frame
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);

  display->drawString(0 + x, 10 + y, "🌡️ Environment");

  AirplaneMaster& airplane = AirplaneMaster::getInstance();

  display->setFont(ArialMT_Plain_10);
  display->drawString(0 + x, 25 + y, "Alt: " + String(airplane.getCurrentAltitude(), 1) + "m");
  display->drawString(0 + x, 40 + y, "Temp: " + String(airplane.getCurrentTemperature(), 1) + "°C");

  // System status
  display->setFont(ArialMT_Plain_10);
  String status = "Slave: " + String(airplane.isSlaveHealthy() ? "✅" : "❌");
  status += " | Conn: " + String(airplane.isConnectionActive() ? "✅" : "❌");
  display->drawString(0 + x, 55 + y, status);
}

void drawFrame4(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  // Control Positions Frame
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);

  display->drawString(0 + x, 10 + y, "🎮 Control Positions");

  // You would need to add getters to AirplaneMaster for these
  display->setFont(ArialMT_Plain_10);
  display->drawString(0 + x, 22 + y, "Engine: ---%");    // Add getter for engine position
  display->drawString(0 + x, 32 + y, "Roll: ---°");      // Add getter for roll position
  display->drawString(0 + x, 42 + y, "Elevator: ---°");  // Add getter for elevator position
  display->drawString(0 + x, 52 + y, "Rudder: ---°");    // Add getter for rudder position
}
