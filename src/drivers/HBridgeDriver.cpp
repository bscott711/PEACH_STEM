#include "drivers/HBridgeDriver.h"

void HBridge_Init() {
  pinMode(HB_IN1, OUTPUT);
  pinMode(HB_IN2, OUTPUT);
  pinMode(HB_ENA, OUTPUT);

  digitalWrite(HB_ENA, HIGH); // << FORCE ENABLE
}

void HBridge_Set(ActuatorDirection dir) {
  switch (dir) {
  case ACT_FORWARD:
    digitalWrite(HB_IN1, HIGH);
    digitalWrite(HB_IN2, LOW);
    break;

  case ACT_REVERSE:
    digitalWrite(HB_IN1, LOW);
    digitalWrite(HB_IN2, HIGH);
    break;

  default:
    digitalWrite(HB_IN1, LOW);
    digitalWrite(HB_IN2, LOW);
    break;
  }
}