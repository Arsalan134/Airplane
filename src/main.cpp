#include <definitions.h>

void setup() {
  Serial.begin(115200);
  engine.attach(motorPin, 1000, 2000);
}

void loop() {
  int value = map(analogRead(A0), 0, 1024, 0, 180);

  Serial.println(value);
  engine.write(value);
}
