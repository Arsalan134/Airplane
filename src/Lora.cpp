#include <LoRa.h>
#include "Common/common.h"

String message = "e0a90r90l90t0i0f0z0y0#107";
String recievedMessage = "e0a90r90l90t0i0f0z0y0#107";

int engineReceived = 0;
int aileronReceived = 90;
int rudderReceived = 90;
int elevatorsReceived = 90;

int elevatorTrimReceived = 0;
int elevatorTrimToDisplay = 0;

int aileronTrimReceived = 0;
int aileronTrimToDisplay = 0;

int flapsRecieved = 0;
int flapsToDisplay = 0;

bool resetAileronTrim = false;
bool resetElevatorTrim = false;

unsigned long lastRecievedTime = millis();

int RSSI = 0;

int RSSIToDisplay;
int elapsedTimeToDisplay;

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

int indE, indA, indR, indEL, indT, indI, indF, indZ, indY, indHashtag;

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

  // Serial.print("Message: \t" + recievedMessage);

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

  indI = recievedMessage.indexOf('i');  // 'i' is used for aileron trim
  if (indI == -1) {
    Serial.println("No aileron trim data found in the message");
    return;
  }

  indF = recievedMessage.indexOf('f');  // 'f' is used for flaps
  if (indF == -1) {
    Serial.println("No flaps data found in the message");
    return;
  }

  indZ = recievedMessage.indexOf('z');  // 'z' is used for reset aileron trim
  if (indZ == -1) {
    Serial.println("No reset aileron trim data found in the message");
    return;
  }

  indY = recievedMessage.indexOf('y');  // 'y' is used for reset elevator trim
  if (indY == -1) {
    Serial.println("No reset elevator trim data found in the message");
    return;
  }

  engineReceived = recievedMessage.substring(indE + 1, indA).toInt();
  aileronReceived = recievedMessage.substring(indA + 1, indR).toInt();
  rudderReceived = recievedMessage.substring(indR + 1, indEL).toInt();
  elevatorsReceived = recievedMessage.substring(indEL + 1, indT).toInt();
  elevatorTrimReceived = recievedMessage.substring(indT + 1, indI).toInt();
  aileronTrimReceived = recievedMessage.substring(indI + 1, indF).toInt();
  flapsRecieved = recievedMessage.substring(indF + 1, indZ).toInt();
  resetAileronTrim = recievedMessage.substring(indZ + 1, indY).toInt();
  resetElevatorTrim = recievedMessage.substring(indY + 1).toInt();

  RSSI = LoRa.packetRssi();
  RSSIToDisplay = RSSI;

  elapsedTimeToDisplay = millis() - lastRecievedTime;

  digitalWrite(BUILTIN_LED, 0);
  lastRecievedTime = millis();
}

void onTxDone() {
  Serial.println("TxDone");
  LoRa_rxMode();
}