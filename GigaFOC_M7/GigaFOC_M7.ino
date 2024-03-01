#include "Arduino.h"
#include "RPC.h"

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }
  RPC.begin();
}

void loop() {
  while (RPC.available()) {
    Serial.print((char) RPC.read());
  }
  while (Serial.available()) {
    RPC.print((char) Serial.read());
  }
}
