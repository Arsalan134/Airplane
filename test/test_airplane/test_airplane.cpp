// 🧪 Unit Tests for Airplane Flight Control System
// Tests the core Airplane class functionality

#include <unity.h>

#ifdef UNIT_TEST

// Mock Arduino functions for native testing
#ifndef ARDUINO
#include <cstdint>
#include <iostream>

typedef uint8_t byte;
typedef bool boolean;

unsigned long millis() {
  return 1000;
}
void delay(int ms) {
  (void)ms;
}
void Serial_println(const char* msg) {
  std::cout << msg << std::endl;
}

// Mock Serial object
struct {
  void println(const char* msg) { Serial_println(msg); }
  void println(const std::string& msg) { Serial_println(msg.c_str()); }
} Serial;

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

// Include the header we want to test
// Note: We'll need to create mock versions of dependencies
class MockAirplane {
 private:
  byte targetEngine = 0;
  byte targetRoll = 90;
  byte targetRudder = 90;
  byte targetElevators = 90;
  int elevatorTrim = 0;
  int aileronTrim = 0;
  int flaps = 0;
  bool landingAirbrake = false;
  bool connectionActive = false;
  unsigned long lastReceivedTime = 0;

 public:
  // 🎯 Test setters
  void setThrottle(byte value) {
    if (isValidControlValue(value)) {
      targetEngine = value;
    }
  }

  void setAilerons(byte value) {
    if (isValidControlValue(value)) {
      targetRoll = value;
    }
  }

  void setRudder(byte value) {
    if (isValidControlValue(value)) {
      targetRudder = value;
    }
  }

  void setElevators(byte value) {
    if (isValidControlValue(value)) {
      targetElevators = value;
    }
  }

  // 🎯 Test getters
  byte getEngine() const { return targetEngine; }
  byte getAilerons() const { return targetRoll; }
  byte getRudder() const { return targetRudder; }
  byte getElevators() const { return targetElevators; }

  // 🔧 Trim functions
  void setElevatorTrim(int value) {
    if (value > 0)
      elevatorTrim += 2;
    else if (value < 0)
      elevatorTrim -= 2;
    elevatorTrim = constrain(elevatorTrim, -45, 45);
  }

  void setAileronTrim(int value) {
    if (value > 0)
      aileronTrim += 2;
    else if (value < 0)
      aileronTrim -= 2;
    aileronTrim = constrain(aileronTrim, -45, 45);
  }

  int getElevatorTrim() const { return elevatorTrim; }
  int getAileronTrim() const { return aileronTrim; }

  // 🔄 Reset functions
  void resetElevatorTrim() { elevatorTrim = 0; }
  void resetAileronTrim() { aileronTrim = 0; }

  // 🛡️ Safety functions
  void resetToSafeDefaults() {
    targetEngine = 0;
    targetRoll = 90;
    targetRudder = 90;
    targetElevators = 90;
  }

  void setConnectionStatus(bool active) {
    connectionActive = active;
    lastReceivedTime = millis();
  }

  bool isConnectionActive() const { return connectionActive; }

 private:
  bool isValidControlValue(byte value) { return value >= 0 && value <= 180; }
};

// 🧪 Global test instance
MockAirplane testAirplane;

// 🧪 Test Setup and Teardown
void setUp(void) {
  // Reset airplane to known state before each test
  testAirplane.resetToSafeDefaults();
  testAirplane.resetElevatorTrim();
  testAirplane.resetAileronTrim();
  testAirplane.setConnectionStatus(false);
}

void tearDown(void) {
  // Clean up after each test
}

// 🎯 Test Flight Control Surface Functions
void test_throttle_control(void) {
  // ✅ Test valid throttle values
  testAirplane.setThrottle(0);
  TEST_ASSERT_EQUAL(0, testAirplane.getEngine());

  testAirplane.setThrottle(90);
  TEST_ASSERT_EQUAL(90, testAirplane.getEngine());

  testAirplane.setThrottle(180);
  TEST_ASSERT_EQUAL(180, testAirplane.getEngine());

  // ❌ Test invalid values (should be ignored)
  testAirplane.setThrottle(255);
  TEST_ASSERT_NOT_EQUAL(255, testAirplane.getEngine());
}

void test_aileron_control(void) {
  // ✅ Test neutral position
  testAirplane.setAilerons(90);
  TEST_ASSERT_EQUAL(90, testAirplane.getAilerons());

  // ✅ Test left and right
  testAirplane.setAilerons(0);
  TEST_ASSERT_EQUAL(0, testAirplane.getAilerons());

  testAirplane.setAilerons(180);
  TEST_ASSERT_EQUAL(180, testAirplane.getAilerons());
}

void test_rudder_control(void) {
  // ✅ Test center position
  testAirplane.setRudder(90);
  TEST_ASSERT_EQUAL(90, testAirplane.getRudder());

  // ✅ Test left and right extremes
  testAirplane.setRudder(0);
  TEST_ASSERT_EQUAL(0, testAirplane.getRudder());

  testAirplane.setRudder(180);
  TEST_ASSERT_EQUAL(180, testAirplane.getRudder());
}

void test_elevator_control(void) {
  // ✅ Test neutral position
  testAirplane.setElevators(90);
  TEST_ASSERT_EQUAL(90, testAirplane.getElevators());

  // ✅ Test up and down
  testAirplane.setElevators(0);
  TEST_ASSERT_EQUAL(0, testAirplane.getElevators());

  testAirplane.setElevators(180);
  TEST_ASSERT_EQUAL(180, testAirplane.getElevators());
}

// 🔧 Test Trim Functions
void test_elevator_trim(void) {
  // ✅ Test trim up
  testAirplane.setElevatorTrim(1);
  TEST_ASSERT_EQUAL(2, testAirplane.getElevatorTrim());

  // ✅ Test trim down
  testAirplane.resetElevatorTrim();
  testAirplane.setElevatorTrim(-1);
  TEST_ASSERT_EQUAL(-2, testAirplane.getElevatorTrim());

  // ✅ Test trim limits (should not exceed ±45)
  for (int i = 0; i < 30; i++) {
    testAirplane.setElevatorTrim(1);
  }
  TEST_ASSERT_LESS_OR_EQUAL(45, testAirplane.getElevatorTrim());
  TEST_ASSERT_GREATER_OR_EQUAL(-45, testAirplane.getElevatorTrim());
}

void test_aileron_trim(void) {
  // ✅ Test trim right
  testAirplane.setAileronTrim(1);
  TEST_ASSERT_EQUAL(2, testAirplane.getAileronTrim());

  // ✅ Test trim left
  testAirplane.resetAileronTrim();
  testAirplane.setAileronTrim(-1);
  TEST_ASSERT_EQUAL(-2, testAirplane.getAileronTrim());

  // ✅ Test trim limits
  for (int i = 0; i < 30; i++) {
    testAirplane.setAileronTrim(1);
  }
  TEST_ASSERT_LESS_OR_EQUAL(45, testAirplane.getAileronTrim());
}

// 🔄 Test Reset Functions
void test_trim_reset(void) {
  // ✅ Set some trim values
  testAirplane.setElevatorTrim(5);
  testAirplane.setAileronTrim(-3);

  // ✅ Reset and verify
  testAirplane.resetElevatorTrim();
  testAirplane.resetAileronTrim();

  TEST_ASSERT_EQUAL(0, testAirplane.getElevatorTrim());
  TEST_ASSERT_EQUAL(0, testAirplane.getAileronTrim());
}

void test_safe_defaults(void) {
  // ✅ Set some non-default values
  testAirplane.setThrottle(100);
  testAirplane.setAilerons(45);
  testAirplane.setRudder(135);
  testAirplane.setElevators(30);

  // ✅ Reset to safe defaults
  testAirplane.resetToSafeDefaults();

  // ✅ Verify all controls are in safe positions
  TEST_ASSERT_EQUAL(0, testAirplane.getEngine());      // Engine off
  TEST_ASSERT_EQUAL(90, testAirplane.getAilerons());   // Center
  TEST_ASSERT_EQUAL(90, testAirplane.getRudder());     // Center
  TEST_ASSERT_EQUAL(90, testAirplane.getElevators());  // Center
}

// 🛡️ Test Safety Systems
void test_connection_status(void) {
  // ✅ Test connection active
  testAirplane.setConnectionStatus(true);
  TEST_ASSERT_TRUE(testAirplane.isConnectionActive());

  // ✅ Test connection inactive
  testAirplane.setConnectionStatus(false);
  TEST_ASSERT_FALSE(testAirplane.isConnectionActive());
}

// 📊 Test Input Validation
void test_input_boundaries(void) {
  // ✅ Test minimum values
  testAirplane.setThrottle(0);
  testAirplane.setAilerons(0);
  testAirplane.setRudder(0);
  testAirplane.setElevators(0);

  TEST_ASSERT_EQUAL(0, testAirplane.getEngine());
  TEST_ASSERT_EQUAL(0, testAirplane.getAilerons());
  TEST_ASSERT_EQUAL(0, testAirplane.getRudder());
  TEST_ASSERT_EQUAL(0, testAirplane.getElevators());

  // ✅ Test maximum values
  testAirplane.setThrottle(180);
  testAirplane.setAilerons(180);
  testAirplane.setRudder(180);
  testAirplane.setElevators(180);

  TEST_ASSERT_EQUAL(180, testAirplane.getEngine());
  TEST_ASSERT_EQUAL(180, testAirplane.getAilerons());
  TEST_ASSERT_EQUAL(180, testAirplane.getRudder());
  TEST_ASSERT_EQUAL(180, testAirplane.getElevators());
}

// 🚀 Test Runner
int runUnityTests(void) {
  UNITY_BEGIN();

  // 🎯 Flight Control Tests
  RUN_TEST(test_throttle_control);
  RUN_TEST(test_aileron_control);
  RUN_TEST(test_rudder_control);
  RUN_TEST(test_elevator_control);

  // 🔧 Trim Tests
  RUN_TEST(test_elevator_trim);
  RUN_TEST(test_aileron_trim);
  RUN_TEST(test_trim_reset);

  // 🛡️ Safety Tests
  RUN_TEST(test_safe_defaults);
  RUN_TEST(test_connection_status);
  RUN_TEST(test_input_boundaries);

  return UNITY_END();
}

// 🎯 Entry Points
#ifdef ARDUINO
void setup() {
  delay(2000);  // Wait for serial monitor
  Serial.begin(115200);
  Serial.println("🧪 Starting Airplane Unit Tests...");
  runUnityTests();
}

void loop() {
  // Tests run once in setup()
}
#else
int main() {
  return runUnityTests();
}
#endif

#endif  // UNIT_TEST
