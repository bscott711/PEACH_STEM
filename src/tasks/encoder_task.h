#pragma once

/**
 * @brief Encoder interrupt handling architecture
 * * Hardware:
 * - Adafruit seesaw chip @ 0x49 on I2C
 * - Single INT pin (ENCODER_INT_PIN=15) signals ANY encoder/button event
 * - 4 rotary encoders + 4 buttons share I2C bus and interrupt line
 * * Software flow:
 * 1. Seesaw pulls INT pin LOW on encoder rotation or button edge
 * 2. encoderISR() fires → yields binary semaphore
 * 3. encoderTask() wakes → calls EncoderDriver_Service()
 * 4. Service() reads all 4 encoders/buttons via I2C, updates g_encoderState
 * 5. controller_task() polls g_encoderState with mutex protection
 * * Mutex protocol:
 * - Takes encoderStateMutex to write g_encoderState
 */

#define ENCODER_INT_PIN 15

void encoderInit();
void encoderTask(void *pv);