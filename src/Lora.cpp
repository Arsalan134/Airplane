#include <LoRa.h>
#include "Common/common.h"

String message = "e0a90r90l90";
String recievedMessage = "e0a90r90l90";

int engineRecieved = 0;
int aileronRecieved = 90;
int rudderRecieved = 90;
int elevatorsRecieved = 90;
unsigned long lastRecievedTime = millis();

int RSSI = 0;

void LoRa_rxMode() {
  LoRa.enableInvertIQ();  // active invert I and Q signals
  LoRa.receive();         // set receive mode
}

void LoRa_txMode() {
  LoRa.idle();             // set standby mode
  LoRa.disableInvertIQ();  // normal mode
}

void LoRa_sendMessage(String message) {
  LoRa_txMode();         // set tx mode
  LoRa.beginPacket();    // start packet
  LoRa.print(message);   // add payload
  LoRa.endPacket(true);  // finish packet and send it
}

void onReceive(int packetSize) {
  digitalWrite(BUILTIN_LED, 1);

  message = "";

  if (packetSize == 0) {
    Serial.println("Received empty packet");
    return;
  }

  while (LoRa.available())
    message += (char)LoRa.read();

  recievedMessage = message;
  Serial.println("\tMessage: \t" + recievedMessage);

  int indE = recievedMessage.indexOf('e');  // 'e' is used for engine
  if (indE == -1) {
    Serial.println("No engine data found in the message");
    return;
  }

  int indA = recievedMessage.indexOf('a');  // 'a' is used for ailerons
  if (indA == -1) {
    Serial.println("No aileron data found in the message");
    return;
  }

  int indR = recievedMessage.indexOf('r');  // 'r' is used for rudder
  if (indR == -1) {
    Serial.println("No rudder data found in the message");
    return;
  }

  int indEL = recievedMessage.indexOf('l');  // 'l' is used for elevators
  if (indEL == -1) {
    Serial.println("No elevator data found in the message");
    return;
  }

  lastRecievedTime = millis();

  engineRecieved = recievedMessage.substring(indE + 1, indA).toInt();
  aileronRecieved = recievedMessage.substring(indA + 1, indR).toInt();
  rudderRecieved = recievedMessage.substring(indR + 1, indEL).toInt();
  elevatorsRecieved = recievedMessage.substring(indEL + 1).toInt();

  Serial.print("Engine is: " + String(engineRecieved));
  Serial.print("\tAilerons is: " + String(aileronRecieved));
  Serial.print("\t\tRudder is: " + String(rudderRecieved));
  Serial.print("\tElevators is: " + String(elevatorsRecieved));

  RSSI = LoRa.packetRssi();

  Serial.print("\tRSSI: " + String(RSSI));

  Serial.print("\tElapsed: ");
  Serial.print(millis() - lastRecievedTime);

  digitalWrite(BUILTIN_LED, 0);
}

void onTxDone() {
  Serial.println("TxDone");
  LoRa_rxMode();
}

boolean runEvery(unsigned long interval) {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    return true;
  }

  return false;
}

// void loraLoop() {
//   // if (runEvery(1000)) {
//   // try to parse packet
//   if (LoRa.parsePacket()) {
//     // received a packet
//     Serial.print("Received packet ");

//     // read packet
//     if (LoRa.available()) {
//       recievedMessage = LoRa.readString();
//       Serial.print(recievedMessage);
//     }

//     // print RSSI of packet
//     int rssi = LoRa.packetRssi();
//     Serial.print(" with RSSI ");
//     Serial.println(rssi);

//     LoRa.flush();
//     Serial.println("LORA LOOP");
//   }
//   // }
// }

// void radioConnection() {
//   transmitData[throttleIndex] =
//       emergencyStopIsActive ? 0 : max(map(L2Value, 0, 255, 0, 90), map(R2Value, 0, 255, 0,
//       180));

//   trim = constrain(trim, 90 - 45, 90 + 45);

//   transmitData[trimIndex] = trim;

//   radio.stopListening();

//   if (radio.write(&transmitData, sizeof(transmitData)))
//     radio.startListening();
//   else
//     Serial.println("Failed to transmit !");

//   while (radio.available()) {
//     radio.read(&recievedData, sizeof(recievedData));
//     lastRecievedTime = millis();
//   }

//   setLEDColor();

//   // Serial.print("Elapsed: ");
//   // Serial.println(millis() - lastRecievedTime);
// }

// void reset() {
//   transmitData[rollIndex] = 90;
//   transmitData[pitchIndex] = 90;
//   transmitData[yawIndex] = 90;

//   transmitData[autopilotIsOnIndex] = false;
// }

// #define sliderPin A0

// #define rollIndex 0
// #define pitchIndex 1
// #define yawIndex 2
// #define throttleIndex 3
// #define autopilotIsOnIndex 4
// #define trimIndex 5

// #define batteryLevelIndex 0

// byte transmitData[6];
// byte recievedData[1];

// unsigned long lastRecievedTime = millis();
