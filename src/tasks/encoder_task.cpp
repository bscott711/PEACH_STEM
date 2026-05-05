#include <Arduino.h>
#include "tasks/encoder_task.h"
#include "drivers/EncoderDriver.h"

// Create a global sleep/wake token
// -> Used for interrupt based task over polling task
static SemaphoreHandle_t encoderSem;

// Interrupt function, runs when INT pin drives LOW
static void IRAM_ATTR encoderISR()
{
    BaseType_t hp = pdFALSE;

    // Gives the semaphore (wakes the sleeping service task)
    xSemaphoreGiveFromISR(encoderSem, &hp);

    portYIELD_FROM_ISR();
}


void encoderTask(void *pv)
{
    // Initialize the SeeSaw Encoder
    init_encoder();

    while (true)
    {
        // Sleep this code block until ISR releases semaphore
        xSemaphoreTake(encoderSem, portMAX_DELAY);

        // Drain pending interrupt(s)
        // - Pin remains LOW until all interrupts are serviced
        while(digitalRead(ENCODER_INT_PIN) == LOW )
        {
            // Service the interrupt
            EncoderDriver_Service();
            // Delay, not sure if I need
            //vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

// Initialize the Encoder Task
//void EncoderTask_Start()
void encoderInit()
{
    // Create the semaphore object
    encoderSem = xSemaphoreCreateBinary();

    // Configure the INT pin as input
    pinMode(ENCODER_INT_PIN, INPUT_PULLUP);

    // Trigger interrupt when INT pin goes LOW (falling edge)
    attachInterrupt(ENCODER_INT_PIN, encoderISR, FALLING);
}