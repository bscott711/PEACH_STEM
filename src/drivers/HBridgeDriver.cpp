#include "drivers/HBridgeDriver.h"

void HBridge_Init() {
  pinMode(HB_IN1, OUTPUT);
  pinMode(HB_IN2, OUTPUT);
  
  // Initialize PWM on the Enable pin
  ledcSetup(HB_PWM_CH, HB_PWM_FREQ, HB_PWM_RES);
  ledcAttachPin(HB_ENA, HB_PWM_CH);
  ledcWrite(HB_PWM_CH, 0); 
}

void HBridge_Set(ActuatorDirection dir, uint8_t pwm_val) {
  switch (dir) {
  case ACT_FORWARD:
    digitalWrite(HB_IN1, HIGH);
    digitalWrite(HB_IN2, LOW);
    ledcWrite(HB_PWM_CH, pwm_val);
    break;

  case ACT_REVERSE:
    digitalWrite(HB_IN1, LOW);
    digitalWrite(HB_IN2, HIGH);
    ledcWrite(HB_PWM_CH, pwm_val);
    break;

  default:
    digitalWrite(HB_IN1, LOW);
    digitalWrite(HB_IN2, LOW);
    ledcWrite(HB_PWM_CH, 0);
    break;
  }
}