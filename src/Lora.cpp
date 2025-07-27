#include <LoRa.h>
#include "Common/common.h"

String recievedMessage = "e0a90el90";

int engineRecieved;
int aileronRecieved;
int rudderRecieved;
int elevatorsRecieved;

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

  String message = "e0a90el90";

  while (LoRa.available())
    message += (char)LoRa.read();

  recievedMessage = message;

  int indE = recievedMessage.indexOf('e');
  int indA = recievedMessage.indexOf('a');
  int indEL = recievedMessage.indexOf('el');

  engineRecieved = recievedMessage.substring(indE + 1, indA).toInt();
  aileronRecieved = recievedMessage.substring(indA + 1, indEL).toInt();
  elevatorsRecieved = recievedMessage.substring(indEL + 1).toInt();

  Serial.println("Engine is" + String(engineRecieved));
  Serial.println("Ailerons is" + String(aileronRecieved));
  Serial.println("Elevators is" + String(elevatorsRecieved));

  // int rssi = LoRa.packetRssi();

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
