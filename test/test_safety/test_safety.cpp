// 🧪 Unit Tests for Safety Systems and Edge Cases
// Tests timeout handling, emergency procedures, and boundary conditions

#include <unity.h>

#ifdef UNIT_TEST

// Mock Arduino functions for native testing
#ifndef ARDUINO
#include <cstdint>
#include <iostream>

typedef uint8_t byte;
typedef bool boolean;

static unsigned long mockTime = 1000;
unsigned long millis() {
  return mockTime;
}
void delay(int ms) {
  (void)ms;
}
void Serial_println(const char* msg) {
  std::cout << msg << std::endl;
}

struct {
  void println(const char* msg) { Serial_println(msg); }
  void println(const std::string& msg) { Serial_println(msg.c_str()); }
} Serial;

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define CONNECTION_TIMEOUT 2000

// Mock time control for testing
void setMockTime(unsigned long time) {
  mockTime = time;
}
void advanceTime(unsigned long ms) {
  mockTime += ms;
}

#endif

// 🛡️ Mock Safety System
class SafetySystem {
 private:
  unsigned long lastReceivedTime = 0;
  bool connectionActive = false;
  bool emergencyMode = false;
  byte safeThrottle = 0;
  byte safeAilerons = 90;
  byte safeRudder = 90;
  byte safeElevators = 90;

 public:
  // 📡 Connection management
  void updateConnection() {
    lastReceivedTime = millis();
    connectionActive = true;
    emergencyMode = false;
  }

  void checkConnectionTimeout() {
    if (millis() - lastReceivedTime > CONNECTION_TIMEOUT) {
      if (connectionActive) {
        Serial.println("⚠️ Connection timeout detected");
        activateEmergencyMode();
      }
    }
  }

  void activateEmergencyMode() {
    emergencyMode = true;
    connectionActive = false;
    Serial.println("🚨 Emergency mode activated");
  }

  // 🎯 Getters
  bool isConnectionActive() const { return connectionActive; }
  bool isEmergencyMode() const { return emergencyMode; }
  unsigned long getLastReceivedTime() const { return lastReceivedTime; }
  unsigned long getTimeSinceLastPacket() const { return millis() - lastReceivedTime; }

  // 🛡️ Safe value validation
  bool isValidControlValue(byte value) const { return value >= 0 && value <= 180; }

  bool isValidTrimValue(int value) const { return value >= -45 && value <= 45; }

  bool isValidFlapsValue(int value) const { return value >= 0 && value <= 4; }

  // 🔄 Reset to safe state
  void resetToSafeState() {
    // All control surfaces to neutral
    safeThrottle = 0;    // Engine off
    safeAilerons = 90;   // Center
    safeRudder = 90;     // Center
    safeElevators = 90;  // Center
    emergencyMode = false;
  }

  byte getSafeThrottle() const { return safeThrottle; }
  byte getSafeAilerons() const { return safeAilerons; }
  byte getSafeRudder() const { return safeRudder; }
  byte getSafeElevators() const { return safeElevators; }
};

// 🧪 Global test instance
SafetySystem safetySystem;

// 🧪 Test Setup and Teardown
void setUp(void) {
  setMockTime(1000);  // Reset mock time
  safetySystem.resetToSafeState();
}

void tearDown(void) {
  // Clean up after each test
}

// 🧪 Test Connection Timeout Detection
void test_connection_timeout_detection(void) {
  // ✅ Initially no connection
  TEST_ASSERT_FALSE(safetySystem.isConnectionActive());

  // ✅ Simulate receiving packet
  safetySystem.updateConnection();
  TEST_ASSERT_TRUE(safetySystem.isConnectionActive());
  TEST_ASSERT_FALSE(safetySystem.isEmergencyMode());

  // ⏱️ Advance time but stay within timeout
  advanceTime(1500);  // 1.5 seconds
  safetySystem.checkConnectionTimeout();
  TEST_ASSERT_TRUE(safetySystem.isConnectionActive());
  TEST_ASSERT_FALSE(safetySystem.isEmergencyMode());

  // ⏱️ Advance time beyond timeout
  advanceTime(1000);  // Total 2.5 seconds (> 2 second timeout)
  safetySystem.checkConnectionTimeout();
  TEST_ASSERT_FALSE(safetySystem.isConnectionActive());
  TEST_ASSERT_TRUE(safetySystem.isEmergencyMode());
}

// 🧪 Test Emergency Mode Activation
void test_emergency_mode_activation(void) {
  // ✅ Start with active connection
  safetySystem.updateConnection();
  TEST_ASSERT_FALSE(safetySystem.isEmergencyMode());

  // 🚨 Manually trigger emergency
  safetySystem.activateEmergencyMode();
  TEST_ASSERT_TRUE(safetySystem.isEmergencyMode());
  TEST_ASSERT_FALSE(safetySystem.isConnectionActive());
}

// 🧪 Test Safe State Reset
void test_safe_state_reset(void) {
  safetySystem.resetToSafeState();

  // ✅ Verify all controls are in safe positions
  TEST_ASSERT_EQUAL(0, safetySystem.getSafeThrottle());    // Engine off
  TEST_ASSERT_EQUAL(90, safetySystem.getSafeAilerons());   // Center
  TEST_ASSERT_EQUAL(90, safetySystem.getSafeRudder());     // Center
  TEST_ASSERT_EQUAL(90, safetySystem.getSafeElevators());  // Center
  TEST_ASSERT_FALSE(safetySystem.isEmergencyMode());
}

// 🧪 Test Input Validation
void test_control_value_validation(void) {
  // ✅ Valid values
  TEST_ASSERT_TRUE(safetySystem.isValidControlValue(0));
  TEST_ASSERT_TRUE(safetySystem.isValidControlValue(90));
  TEST_ASSERT_TRUE(safetySystem.isValidControlValue(180));

  // ❌ Invalid values
  TEST_ASSERT_FALSE(safetySystem.isValidControlValue(255));
  TEST_ASSERT_FALSE(safetySystem.isValidControlValue(200));
}

void test_trim_value_validation(void) {
  // ✅ Valid trim values
  TEST_ASSERT_TRUE(safetySystem.isValidTrimValue(-45));
  TEST_ASSERT_TRUE(safetySystem.isValidTrimValue(0));
  TEST_ASSERT_TRUE(safetySystem.isValidTrimValue(45));

  // ❌ Invalid trim values
  TEST_ASSERT_FALSE(safetySystem.isValidTrimValue(-50));
  TEST_ASSERT_FALSE(safetySystem.isValidTrimValue(50));
  TEST_ASSERT_FALSE(safetySystem.isValidTrimValue(100));
}

void test_flaps_value_validation(void) {
  // ✅ Valid flap values
  TEST_ASSERT_TRUE(safetySystem.isValidFlapsValue(0));
  TEST_ASSERT_TRUE(safetySystem.isValidFlapsValue(2));
  TEST_ASSERT_TRUE(safetySystem.isValidFlapsValue(4));

  // ❌ Invalid flap values
  TEST_ASSERT_FALSE(safetySystem.isValidFlapsValue(-1));
  TEST_ASSERT_FALSE(safetySystem.isValidFlapsValue(5));
  TEST_ASSERT_FALSE(safetySystem.isValidFlapsValue(10));
}

// 🧪 Test Time Since Last Packet
void test_time_since_last_packet(void) {
  // ✅ Update connection at time 1000
  setMockTime(1000);
  safetySystem.updateConnection();
  TEST_ASSERT_EQUAL(0, safetySystem.getTimeSinceLastPacket());

  // ⏱️ Advance time
  advanceTime(500);
  TEST_ASSERT_EQUAL(500, safetySystem.getTimeSinceLastPacket());

  advanceTime(1000);
  TEST_ASSERT_EQUAL(1500, safetySystem.getTimeSinceLastPacket());
}

// 🧪 Test Connection Recovery
void test_connection_recovery(void) {
  // 🚨 Start in emergency mode (connection lost)
  safetySystem.activateEmergencyMode();
  TEST_ASSERT_TRUE(safetySystem.isEmergencyMode());
  TEST_ASSERT_FALSE(safetySystem.isConnectionActive());

  // 📡 Simulate connection recovery
  safetySystem.updateConnection();
  TEST_ASSERT_TRUE(safetySystem.isConnectionActive());
  TEST_ASSERT_FALSE(safetySystem.isEmergencyMode());
}

// 🧪 Test Rapid Connection Updates
void test_rapid_connection_updates(void) {
  // 📡 Simulate rapid packet reception
  for (int i = 0; i < 10; i++) {
    advanceTime(100);  // 100ms between packets
    safetySystem.updateConnection();
    safetySystem.checkConnectionTimeout();

    // ✅ Should remain connected
    TEST_ASSERT_TRUE(safetySystem.isConnectionActive());
    TEST_ASSERT_FALSE(safetySystem.isEmergencyMode());
  }
}

// 🧪 Test Edge Case: Exactly at Timeout
void test_timeout_boundary_condition(void) {
  safetySystem.updateConnection();
  TEST_ASSERT_TRUE(safetySystem.isConnectionActive());

  // ⏱️ Advance exactly to timeout boundary
  advanceTime(CONNECTION_TIMEOUT + 1);
  safetySystem.checkConnectionTimeout();

  // 🚨 Should trigger timeout (>= timeout triggers it)
  TEST_ASSERT_FALSE(safetySystem.isConnectionActive());
  TEST_ASSERT_TRUE(safetySystem.isEmergencyMode());
}

// 🧪 Test Multiple Timeout Cycles
void test_multiple_timeout_cycles(void) {
  // Cycle 1: Connect -> Timeout
  safetySystem.updateConnection();
  advanceTime(CONNECTION_TIMEOUT + 100);
  safetySystem.checkConnectionTimeout();
  TEST_ASSERT_TRUE(safetySystem.isEmergencyMode());

  // Cycle 2: Recover -> Timeout again
  safetySystem.updateConnection();
  TEST_ASSERT_FALSE(safetySystem.isEmergencyMode());

  advanceTime(CONNECTION_TIMEOUT + 100);
  safetySystem.checkConnectionTimeout();
  TEST_ASSERT_TRUE(safetySystem.isEmergencyMode());
}

// 🚀 Test Runner
int runUnityTests(void) {
  UNITY_BEGIN();

  // 🛡️ Safety System Tests
  RUN_TEST(test_connection_timeout_detection);
  RUN_TEST(test_emergency_mode_activation);
  RUN_TEST(test_safe_state_reset);
  RUN_TEST(test_connection_recovery);

  // ✅ Input Validation Tests
  RUN_TEST(test_control_value_validation);
  RUN_TEST(test_trim_value_validation);
  RUN_TEST(test_flaps_value_validation);

  // ⏱️ Timing Tests
  RUN_TEST(test_time_since_last_packet);
  RUN_TEST(test_rapid_connection_updates);
  RUN_TEST(test_timeout_boundary_condition);
  RUN_TEST(test_multiple_timeout_cycles);

  return UNITY_END();
}

// 🎯 Entry Points
#ifdef ARDUINO
void setup() {
  delay(2000);  // Wait for serial monitor
  Serial.begin(115200);
  Serial.println("🧪 Starting Safety System Tests...");
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
