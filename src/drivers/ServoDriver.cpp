#include "drivers/ServoDriver.h"

// Initialize PWM hardware for servo
void ServoDriver_Init() {
  ledcSetup(SERVO_LEDC_CH, SERVO_PWM_HZ, SERVO_RES_BITS);
  ledcAttachPin(SERVO_PIN, SERVO_LEDC_CH);
}

// Write servo angle as percent (0–100)
void ServoDriver_WritePercent(int pct) {
  pct = constrain(pct, 0, 100);

  // 20ms period at 50Hz
  const uint32_t period_us = 1000000UL / SERVO_PWM_HZ;

  // Map percent to pulse width (500–2500us typical)
  const uint32_t pulse_us =
      SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * (uint32_t)pct) / 100;

  const uint32_t max_duty = (1UL << SERVO_RES_BITS) - 1;

  // Convert pulse width to duty cycle
  const uint32_t duty = (pulse_us * max_duty) / period_us;

  ledcWrite(SERVO_LEDC_CH, duty);
}