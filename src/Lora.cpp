#include <LoRa.h>
#include "Common/common.h"

String message = "e0a90r90l90t0#103";
String recievedMessage = "e0a90r90l90t0#103";

int engineRecieved = 0;
int aileronRecieved = 90;
int rudderRecieved = 90;
int elevatorsRecieved = 90;
int trimRecieved = 0;
int trimToDisplay = 0;

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

// XOR checksum function
byte simple_checksum(const byte* data, size_t len) {
  byte sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum ^= data[i];
  }
  return sum;
}

int indE, indA, indR, indEL, indT, indHashtag;

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

  Serial.print("Message: \t" + recievedMessage);

  if (recievedMessage.length() < 10) {
    Serial.println("Received message is too short");
    Serial.println("Expected at least 10 characters, got: " + String(recievedMessage.length()));
    return;
  }

  indHashtag = recievedMessage.indexOf('#');
  if (indHashtag == -1) {
    Serial.println("No end of message found");
    return;
  }

  // Check if the message is valid until the #
  String content = recievedMessage.substring(0, recievedMessage.indexOf('#'));

  byte recievedChecksum = recievedMessage.substring(indHashtag + 1).toInt();
  byte calculatedChecksum = simple_checksum((const byte*)content.c_str(), content.length());

  if (calculatedChecksum != recievedChecksum) {
    Serial.println("Invalid checksum: " + String(calculatedChecksum));
    Serial.println("Expected: " + String(recievedChecksum));
    Serial.println("Message: " + recievedMessage);
    return;
  }

  indE = recievedMessage.indexOf('e');  // 'e' is used for engine
  if (indE == -1) {
    Serial.println("No engine data found in the message");
    return;
  }

  indA = recievedMessage.indexOf('a');  // 'a' is used for ailerons
  if (indA == -1) {
    Serial.println("No aileron data found in the message");
    return;
  }

  indR = recievedMessage.indexOf('r');  // 'r' is used for rudder
  if (indR == -1) {
    Serial.println("No rudder data found in the message");
    return;
  }

  indEL = recievedMessage.indexOf('l');  // 'l' is used for elevators
  if (indEL == -1) {
    Serial.println("No elevator data found in the message");
    return;
  }

  indT = recievedMessage.indexOf('t');  // 't' is used for trim
  if (indT == -1) {
    Serial.println("No trim data found in the message");
    return;
  }

  lastRecievedTime = millis();

  engineRecieved = recievedMessage.substring(indE + 1, indA).toInt();
  aileronRecieved = recievedMessage.substring(indA + 1, indR).toInt();
  rudderRecieved = recievedMessage.substring(indR + 1, indEL).toInt();
  elevatorsRecieved = recievedMessage.substring(indEL + 1, indT).toInt();
  trimRecieved = recievedMessage.substring(indT + 1).toInt();

  Serial.print("\tEngine is: " + String(engineRecieved));
  Serial.print("\tAilerons is: " + String(aileronRecieved));
  Serial.print("\t\tRudder is: " + String(rudderRecieved));
  Serial.print("\tElevators is: " + String(elevatorsRecieved));
  Serial.print("\tTrim is: " + String(trimRecieved));

  RSSI = LoRa.packetRssi();

  Serial.print("\tRSSI: " + String(RSSI));

  Serial.print("\tElapsed: ");
  Serial.println(millis() - lastRecievedTime);

  digitalWrite(BUILTIN_LED, 0);
}

void onTxDone() {
  Serial.println("TxDone");
  LoRa_rxMode();
}

// #define sliderPin A0

// #define rollIndex 0
// #define pitchIndex 1
// #define yawIndex 2
// #define throttleIndex 3
// #define autopilotIsOnIndex 4
// #define trimIndex 5

// #define batteryLevelIndex 0