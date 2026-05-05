#pragma once

// I2C pins defined in main (shared bus)
#define ENCODER_INT_PIN 15

// Function Prototypes
void encoderInit();
void encoderTask(void *pv);
