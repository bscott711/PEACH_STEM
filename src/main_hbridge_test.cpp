/**
 * @file main_hbridge_test.cpp
 * @brief Minimal DRV8871 H-Bridge test sketch.
 * 
 * Cycles the actuator: 1s forward, 1s stop, 1s reverse, 1s stop.
 * Built-in LED (GPIO 2) lights up during motion.
 * Serial output at 115200 baud for monitoring.
 */

#include <Arduino.h>

// Pin definitions (from HardwareConfig.h)
#define HB_IN1 33
#define HB_IN2 32

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== DRV8871 H-Bridge Test ===");
  Serial.printf("IN1 = GPIO %d\n", HB_IN1);
  Serial.printf("IN2 = GPIO %d\n", HB_IN2);

  pinMode(HB_IN1, OUTPUT);
  pinMode(HB_IN2, OUTPUT);
  pinMode(2, OUTPUT); // Built-in LED

  digitalWrite(HB_IN1, LOW);
  digitalWrite(HB_IN2, LOW);
  digitalWrite(2, LOW);

  Serial.println("Starting cycle in 2 seconds...\n");
  delay(2000);
}

void loop() {
  // --- FORWARD ---
  Serial.println(">> FORWARD (IN1=HIGH, IN2=LOW)");
  digitalWrite(HB_IN1, HIGH);
  digitalWrite(HB_IN2, LOW);
  digitalWrite(2, HIGH);
  delay(1000);

  // --- STOP ---
  Serial.println("-- STOP (both LOW)");
  digitalWrite(HB_IN1, LOW);
  digitalWrite(HB_IN2, LOW);
  digitalWrite(2, LOW);
  delay(1000);

  // --- REVERSE ---
  Serial.println("<< REVERSE (IN1=LOW, IN2=HIGH)");
  digitalWrite(HB_IN1, LOW);
  digitalWrite(HB_IN2, HIGH);
  digitalWrite(2, HIGH);
  delay(1000);

  // --- STOP ---
  Serial.println("-- STOP (both LOW)");
  digitalWrite(HB_IN1, LOW);
  digitalWrite(HB_IN2, LOW);
  digitalWrite(2, LOW);
  delay(1000);
}
