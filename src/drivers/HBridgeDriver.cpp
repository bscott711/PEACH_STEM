#include "drivers/HBridgeDriver.h"

void HBridge_Init() {
  // Setup PWM channels
  ledcSetup(HB_PWM_CH_IN1, HB_PWM_FREQ, HB_PWM_RES);
  ledcSetup(HB_PWM_CH_IN2, HB_PWM_FREQ, HB_PWM_RES);
  
  // Attach pins
  ledcAttachPin(HB_IN1, HB_PWM_CH_IN1);
  ledcAttachPin(HB_IN2, HB_PWM_CH_IN2);
  
  // Initialize to zero (stop/brake)
  ledcWrite(HB_PWM_CH_IN1, 0);
  ledcWrite(HB_PWM_CH_IN2, 0);
}

void HBridge_Set(ActuatorDirection dir, uint8_t pwm_val) {
  switch (dir) {
  case ACT_FORWARD:
    ledcWrite(HB_PWM_CH_IN1, pwm_val);
    ledcWrite(HB_PWM_CH_IN2, 0);
    break;

  case ACT_REVERSE:
    ledcWrite(HB_PWM_CH_IN1, 0);
    ledcWrite(HB_PWM_CH_IN2, pwm_val);
    break;

  default:
    // Brake
    ledcWrite(HB_PWM_CH_IN1, 0);
    ledcWrite(HB_PWM_CH_IN2, 0);
    break;
  }
}