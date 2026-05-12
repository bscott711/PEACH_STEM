#include "drivers/ServoDriver.h"

// Initialize PWM hardware for servo
void ServoDriver_Init() {
  ledcSetup(SERVO_LEDC_CH, SERVO_PWM_HZ, SERVO_RES_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_LEDC_CH);
}

// Write servo angle as percent (0.0–100.0)
void ServoDriver_WritePercent(float pct) {
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;

  // 20ms period at 50Hz
  const uint32_t period_us = 1000000UL / SERVO_PWM_HZ;

  // Calculate exact microsecond pulse width using floating point math
  const float pulse_us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * pct) / 100.0f;

  const uint32_t max_duty = (1UL << SERVO_RES_BITS) - 1;

  // Map microsecond pulse to the 16-bit duty cycle
  const uint32_t duty = (uint32_t)((pulse_us * (float)max_duty) / (float)period_us);

  ledcWrite(SERVO_LEDC_CH, duty);
}

void ServoDriver_Disable() {
  ledcWrite(SERVO_LEDC_CH, 0); // 0 duty cycle -> limp servo
}