#include <Arduino.h>
#include "pin_config.h"

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("BOOT OK");
  Serial.printf("CAN TX: %d\n", CAN_TX);
  Serial.printf("CAN RX: %d\n", CAN_RX);
}

void loop() {
  delay(1000);
  Serial.println("RUNNING");
}