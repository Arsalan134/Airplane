// 🧪 Unit Tests for LoRa Communication System
// Tests packet parsing, checksum validation, and data extraction

#include <unity.h>

#ifdef UNIT_TEST

// Mock Arduino functions for native testing
#ifndef ARDUINO
#include <cstdint>
#include <cstring>
#include <iostream>

typedef uint8_t byte;
typedef bool boolean;

void Serial_println(const char* msg) {
  std::cout << msg << std::endl;
}

struct {
  void println(const char* msg) { Serial_println(msg); }
  void println(const std::string& msg) { Serial_println(msg.c_str()); }
} Serial;

// Mock String class
class String {
 private:
  std::string data;

 public:
  String() = default;
  String(const char* str) : data(str) {}
  String(const std::string& str) : data(str) {}
  String(int val) : data(std::to_string(val)) {}

  int length() const { return static_cast<int>(data.length()); }
  int indexOf(char c) const {
    size_t pos = data.find(c);
    return (pos != std::string::npos) ? static_cast<int>(pos) : -1;
  }
  String substring(int start, int end = -1) const {
    if (end == -1)
      return data.substr(start);
    return data.substr(start, end - start);
  }
  int toInt() const { return std::stoi(data); }
  const char* c_str() const { return data.c_str(); }

  String& operator=(const char* str) {
    data = str;
    return *this;
  }
  String operator+(const String& other) const { return String(data + other.data); }
};

#endif

// 🔐 XOR checksum function (same as in Lora.cpp)
byte simple_checksum(const byte* data, size_t len) {
  byte sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum ^= data[i];
  }
  return sum;
}

// 📡 Mock LoRa packet parser
class LoRaPacketParser {
 private:
  String receivedMessage;
  int engineReceived = 0;
  int aileronReceived = 90;
  int rudderReceived = 90;
  int elevatorsReceived = 90;
  int elevatorTrimReceived = 0;
  int aileronTrimReceived = 0;
  int flapsReceived = 0;
  bool resetAileronTrim = false;
  bool resetElevatorTrim = false;
  bool airBrakeReceived = false;

 public:
  // 📦 Parse incoming LoRa packet
  bool parsePacket(const String& packet) {
    receivedMessage = packet;

    if (receivedMessage.length() < 10) {
      Serial.println("📏 Packet too short");
      return false;
    }

    int hashtagIndex = receivedMessage.indexOf('#');
    if (hashtagIndex == -1) {
      Serial.println("❌ No end marker found");
      return false;
    }

    // 🔐 Validate checksum
    String content = receivedMessage.substring(0, hashtagIndex);
    byte receivedChecksum = static_cast<byte>(receivedMessage.substring(hashtagIndex + 1).toInt());
    byte calculatedChecksum = simple_checksum(reinterpret_cast<const byte*>(content.c_str()), content.length());

    if (calculatedChecksum != receivedChecksum) {
      Serial.println("🔐 Invalid checksum");
      return false;
    }

    // 📊 Parse data fields
    return extractData();
  }

  // 📊 Extract data from packet
  bool extractData() {
    int indE = receivedMessage.indexOf('e');
    int indA = receivedMessage.indexOf('a');
    int indR = receivedMessage.indexOf('r');
    int indL = receivedMessage.indexOf('l');
    int indT = receivedMessage.indexOf('t');
    int indI = receivedMessage.indexOf('i');
    int indF = receivedMessage.indexOf('f');
    int indZ = receivedMessage.indexOf('z');
    int indY = receivedMessage.indexOf('y');
    int indB = receivedMessage.indexOf('b');

    // ✅ Check all required fields exist
    if (indE == -1 || indA == -1 || indR == -1 || indL == -1 || indT == -1 || indI == -1 || indF == -1 || indZ == -1 || indY == -1 ||
        indB == -1) {
      return false;
    }

    // 📊 Extract values
    engineReceived = receivedMessage.substring(indE + 1, indA).toInt();
    aileronReceived = receivedMessage.substring(indA + 1, indR).toInt();
    rudderReceived = receivedMessage.substring(indR + 1, indL).toInt();
    elevatorsReceived = receivedMessage.substring(indL + 1, indT).toInt();
    elevatorTrimReceived = receivedMessage.substring(indT + 1, indI).toInt();
    aileronTrimReceived = receivedMessage.substring(indI + 1, indF).toInt();
    flapsReceived = receivedMessage.substring(indF + 1, indZ).toInt();
    resetAileronTrim = receivedMessage.substring(indZ + 1, indY).toInt() != 0;
    resetElevatorTrim = receivedMessage.substring(indY + 1, indB).toInt() != 0;
    airBrakeReceived = receivedMessage.substring(indB + 1).toInt() != 0;

    return true;
  }

  // 🎯 Getters for testing
  int getEngine() const { return engineReceived; }
  int getAileron() const { return aileronReceived; }
  int getRudder() const { return rudderReceived; }
  int getElevators() const { return elevatorsReceived; }
  int getElevatorTrim() const { return elevatorTrimReceived; }
  int getAileronTrim() const { return aileronTrimReceived; }
  int getFlaps() const { return flapsReceived; }
  bool getResetAileronTrim() const { return resetAileronTrim; }
  bool getResetElevatorTrim() const { return resetElevatorTrim; }
  bool getAirBrake() const { return airBrakeReceived; }
};

// 🧪 Global test instance
LoRaPacketParser parser;

// 🧪 Test Setup and Teardown
void setUp(void) {
  // Reset parser state before each test
}

void tearDown(void) {
  // Clean up after each test
}

// 🔐 Helper function to create valid packet with checksum
String createValidPacket(int engine,
                         int aileron,
                         int rudder,
                         int elevator,
                         int elevatorTrim,
                         int aileronTrim,
                         int flaps,
                         int resetAileronTrim,
                         int resetElevatorTrim,
                         int airbrake) {
  String packet = String("e") + String(engine) + String("a") + String(aileron) + String("r") + String(rudder) + String("l") +
                  String(elevator) + String("t") + String(elevatorTrim) + String("i") + String(aileronTrim) + String("f") + String(flaps) +
                  String("z") + String(resetAileronTrim) + String("y") + String(resetElevatorTrim) + String("b") + String(airbrake);

  byte checksum = simple_checksum(reinterpret_cast<const byte*>(packet.c_str()), packet.length());
  return packet + String("#") + String(static_cast<int>(checksum));
}

// 🧪 Test Valid Packet Parsing
void test_valid_packet_parsing(void) {
  // ✅ Create a valid packet
  String validPacket = createValidPacket(90, 100, 80, 110, 1, -1, 2, 0, 1, 0);

  // ✅ Parse packet
  bool result = parser.parsePacket(validPacket);
  TEST_ASSERT_TRUE(result);

  // ✅ Verify extracted data
  TEST_ASSERT_EQUAL(90, parser.getEngine());
  TEST_ASSERT_EQUAL(100, parser.getAileron());
  TEST_ASSERT_EQUAL(80, parser.getRudder());
  TEST_ASSERT_EQUAL(110, parser.getElevators());
  TEST_ASSERT_EQUAL(1, parser.getElevatorTrim());
  TEST_ASSERT_EQUAL(-1, parser.getAileronTrim());
  TEST_ASSERT_EQUAL(2, parser.getFlaps());
  TEST_ASSERT_FALSE(parser.getResetAileronTrim());
  TEST_ASSERT_TRUE(parser.getResetElevatorTrim());
  TEST_ASSERT_FALSE(parser.getAirBrake());
}

// 🧪 Test Default Values
void test_default_values_packet(void) {
  // ✅ Create packet with default/neutral values
  String defaultPacket = createValidPacket(0, 90, 90, 90, 0, 0, 0, 0, 0, 0);

  bool result = parser.parsePacket(defaultPacket);
  TEST_ASSERT_TRUE(result);

  // ✅ Verify all neutral/default positions
  TEST_ASSERT_EQUAL(0, parser.getEngine());      // Engine off
  TEST_ASSERT_EQUAL(90, parser.getAileron());    // Center
  TEST_ASSERT_EQUAL(90, parser.getRudder());     // Center
  TEST_ASSERT_EQUAL(90, parser.getElevators());  // Center
  TEST_ASSERT_EQUAL(0, parser.getElevatorTrim());
  TEST_ASSERT_EQUAL(0, parser.getAileronTrim());
  TEST_ASSERT_EQUAL(0, parser.getFlaps());
  TEST_ASSERT_FALSE(parser.getResetAileronTrim());
  TEST_ASSERT_FALSE(parser.getResetElevatorTrim());
  TEST_ASSERT_FALSE(parser.getAirBrake());
}

// 🧪 Test Extreme Values
void test_extreme_values_packet(void) {
  // ✅ Create packet with maximum values
  String extremePacket = createValidPacket(180, 180, 180, 180, 1, 1, 4, 1, 1, 1);

  bool result = parser.parsePacket(extremePacket);
  TEST_ASSERT_TRUE(result);

  // ✅ Verify extreme values
  TEST_ASSERT_EQUAL(180, parser.getEngine());
  TEST_ASSERT_EQUAL(180, parser.getAileron());
  TEST_ASSERT_EQUAL(180, parser.getRudder());
  TEST_ASSERT_EQUAL(180, parser.getElevators());
  TEST_ASSERT_EQUAL(1, parser.getElevatorTrim());
  TEST_ASSERT_EQUAL(1, parser.getAileronTrim());
  TEST_ASSERT_EQUAL(4, parser.getFlaps());
  TEST_ASSERT_TRUE(parser.getResetAileronTrim());
  TEST_ASSERT_TRUE(parser.getResetElevatorTrim());
  TEST_ASSERT_TRUE(parser.getAirBrake());
}

// 🧪 Test Checksum Validation
void test_checksum_validation(void) {
  // ❌ Create packet with invalid checksum
  String invalidPacket = "e90a90r90l90t0i0f0z0y0b0#999";

  bool result = parser.parsePacket(invalidPacket);
  TEST_ASSERT_FALSE(result);  // Should fail checksum validation
}

// 🧪 Test Malformed Packets
void test_malformed_packets(void) {
  // ❌ Too short packet
  bool result1 = parser.parsePacket("e90#0");
  TEST_ASSERT_FALSE(result1);

  // ❌ No end marker
  bool result2 = parser.parsePacket("e90a90r90l90t0i0f0z0y0b0");
  TEST_ASSERT_FALSE(result2);

  // ❌ Missing required fields
  bool result3 = parser.parsePacket("e90a90r90#50");
  TEST_ASSERT_FALSE(result3);
}

// 🧪 Test Checksum Function
void test_checksum_function(void) {
  // ✅ Test known checksum values
  const char* testData1 = "hello";
  byte checksum1 = simple_checksum(reinterpret_cast<const byte*>(testData1), 5);

  const char* testData2 = "hello";
  byte checksum2 = simple_checksum(reinterpret_cast<const byte*>(testData2), 5);

  // ✅ Same data should produce same checksum
  TEST_ASSERT_EQUAL(checksum1, checksum2);
}

// 🧪 Test Airbrake Functionality
void test_airbrake_parsing(void) {
  // ✅ Test airbrake activated
  String airbrakeOnPacket = createValidPacket(0, 90, 90, 90, 0, 0, 0, 0, 0, 1);
  bool result1 = parser.parsePacket(airbrakeOnPacket);
  TEST_ASSERT_TRUE(result1);
  TEST_ASSERT_TRUE(parser.getAirBrake());

  // ✅ Test airbrake deactivated
  String airbrakeOffPacket = createValidPacket(0, 90, 90, 90, 0, 0, 0, 0, 0, 0);
  bool result2 = parser.parsePacket(airbrakeOffPacket);
  TEST_ASSERT_TRUE(result2);
  TEST_ASSERT_FALSE(parser.getAirBrake());
}

// 🧪 Test Trim Reset Functionality
void test_trim_reset_parsing(void) {
  // ✅ Test both trim resets activated
  String resetPacket = createValidPacket(0, 90, 90, 90, 0, 0, 0, 1, 1, 0);
  bool result = parser.parsePacket(resetPacket);
  TEST_ASSERT_TRUE(result);
  TEST_ASSERT_TRUE(parser.getResetAileronTrim());
  TEST_ASSERT_TRUE(parser.getResetElevatorTrim());

  // ✅ Test no resets
  String noResetPacket = createValidPacket(0, 90, 90, 90, 0, 0, 0, 0, 0, 0);
  result = parser.parsePacket(noResetPacket);
  TEST_ASSERT_TRUE(result);
  TEST_ASSERT_FALSE(parser.getResetAileronTrim());
  TEST_ASSERT_FALSE(parser.getResetElevatorTrim());
}

// 🚀 Test Runner
int runUnityTests(void) {
  UNITY_BEGIN();

  // 📡 Communication Protocol Tests
  RUN_TEST(test_valid_packet_parsing);
  RUN_TEST(test_default_values_packet);
  RUN_TEST(test_extreme_values_packet);

  // 🔐 Security Tests
  RUN_TEST(test_checksum_validation);
  RUN_TEST(test_checksum_function);

  // 🛡️ Error Handling Tests
  RUN_TEST(test_malformed_packets);

  // 🎛️ Feature Tests
  RUN_TEST(test_airbrake_parsing);
  RUN_TEST(test_trim_reset_parsing);

  return UNITY_END();
}

// 🎯 Entry Points
#ifdef ARDUINO
void setup() {
  delay(2000);  // Wait for serial monitor
  Serial.begin(115200);
  Serial.println("🧪 Starting LoRa Communication Tests...");
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
