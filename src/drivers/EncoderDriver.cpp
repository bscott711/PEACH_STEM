#include "controller.h"
#include "drivers/EncoderDriver.h"
#include <Wire.h>
#include <Adafruit_seesaw.h>

// Seesaw Object bound to I2C. All encoder communication happens through this. 
static Adafruit_seesaw ss(&Wire);

// Global shared struct
EncoderState g_encoderState;

// Stores previous button level for edge detection
static bool lastBtnLevel[4];

// Initialize the 4-encoder Seesaw board
void init_encoder()
{
    // Connect to Seesaw chip over I2C (stall if fails)
    if (!ss.begin(ENCODER_I2C_ADDR)) 
    {
        Serial.printf("Failed to detect encoder.");
        while(true) { vTaskDelay(portMAX_DELAY); } 
    }

    // Configure push buttons by configuring Seesaw's internal GPIO pins
    // - INPUT_PULLUP means 0 when pressed, 1 when released. 
    ss.pinMode(12, INPUT_PULLUP);
    ss.pinMode(14, INPUT_PULLUP);
    ss.pinMode(17, INPUT_PULLUP);
    ss.pinMode(9,  INPUT_PULLUP);

    // Capture initial button states 
    lastBtnLevel[0] = ss.digitalRead(12);
    lastBtnLevel[1] = ss.digitalRead(14);
    lastBtnLevel[2] = ss.digitalRead(17);
    lastBtnLevel[3] = ss.digitalRead(9);

    // Build a 32-bit interrupt mask. Each bit corresponds to a Seesaw GPIO number. 
    // Setting the bit enables interrupts for that pin.
    uint32_t mask = (1UL << 12) | (1UL << 14) | (1UL << 17) | (1UL << 9);
    ss.setGPIOInterrupts(mask, 1);

    // Read and store the current position of each encoder (baseline).
    g_encoderState.position[0] = ss.getEncoderPosition(0);
    g_encoderState.position[1] = ss.getEncoderPosition(1);
    g_encoderState.position[2] = ss.getEncoderPosition(2);
    g_encoderState.position[3] = ss.getEncoderPosition(3);

    // Enable interrupts for encoder movement.
    ss.enableEncoderInterrupt(0);
    ss.enableEncoderInterrupt(1);
    ss.enableEncoderInterrupt(2);
    ss.enableEncoderInterrupt(3);

    // Clear all button flags.
    g_encoderState.buttonPressed[0] = false;
    g_encoderState.buttonPressed[1] = false;
    g_encoderState.buttonPressed[2] = false;
    g_encoderState.buttonPressed[3] = false;
}


void EncoderDriver_Service()
{
    // ********** ENCODER 0 **********
    // ENC0 button (pin 12)
    bool b0 = ss.digitalRead(12);
    if (lastBtnLevel[0] == true && b0 == false)
    {
        // Update Flag Array
        g_encoderState.buttonPressed[0] = true;
        Serial.println("Button 0 pressed");
    }
    lastBtnLevel[0] = b0;

    // ENC0 Encoder
    int32_t d0 = ss.getEncoderDelta(0);
    if (d0 != 0)
    {
        // Update Encoder Value Array
        g_encoderState.position[0] += d0;
        Serial.printf("Enc 0: %d\n", (int)g_encoderState.position[0]);
    }

    // ********** ENCODER 1 **********
    // ENC1 button (pin 14)
    bool b1 = ss.digitalRead(14);
    if (lastBtnLevel[1] == true && b1 == false)
    {
        // Update Flag Array
        g_encoderState.buttonPressed[1] = true;
        Serial.println("Button 1 pressed");
    }
    lastBtnLevel[1] = b1;

    // ENC1 Encoder
    int32_t d1 = ss.getEncoderDelta(1);
    if (d1 != 0)
    {
        // Update Encoder Value Array
        g_encoderState.position[1] += d1;
        Serial.printf("Enc 1: %d\n", (int)g_encoderState.position[1]);
    }


    // ********** ENCODER 2 **********
    // ENC2 button (pin 17)
    bool b2 = ss.digitalRead(17);
    if (lastBtnLevel[2] == true && b2 == false)
    {
        // Update Flag Array
        g_encoderState.buttonPressed[2] = true;
        Serial.println("Button 2 pressed");
    }
    lastBtnLevel[2] = b2;

    // ENC1 Encoder
    int32_t d2 = ss.getEncoderDelta(2);
    if (d2 != 0)
    {
        // Update Encoder Value Array
        g_encoderState.position[2] += d2;
        Serial.printf("Enc 2: %d\n", (int)g_encoderState.position[2]);
    }

    
    // ********** ENCODER 3 **********
    // ENC3 button (pin 9)
    bool b3 = ss.digitalRead(9);
    if (lastBtnLevel[3] == true && b3 == false)
    {
        // Logic for button 3 below
        g_encoderState.buttonPressed[3] = true;
        Serial.println("Button 3 pressed");
    }
    lastBtnLevel[3] = b3;

    // ENC3 Encoder
    int32_t d3 = ss.getEncoderDelta(3);
    if (d3 != 0)
    {
        // Update Encoder Value Array
        g_encoderState.position[3] += d3;
        Serial.printf("Enc 3: %d\n", (int)g_encoderState.position[3]);
    }
}