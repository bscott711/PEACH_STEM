#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <stdint.h>

// Default task update interval in milliseconds (can be overridden per-node)
#ifndef TASK_UPDATE_INTERVAL_MS
#define TASK_UPDATE_INTERVAL_MS 10
#endif

/**
 * @brief Active Object pattern base class for motion control subsystems.
 * 
 * Each subsystem runs as its own FreeRTOS task, managing its own state
 * and communicating via lock-free queues (message passing).
 * 
 * @tparam CmdType Command structure type (e.g., ServoCommand)
 * @tparam TelType Telemetry structure type (e.g., ServoTelemetry)
 */
template<typename CmdType, typename TelType>
class ActiveMotionNode {
protected:
    QueueHandle_t cmdQueue;   // Incoming command queue (size 10)
    QueueHandle_t telQueue;   // Outgoing telemetry mailbox (size 1, overwrite)
    
    TaskHandle_t taskHandle;
    const char* taskName;
    UBaseType_t priority;
    uint32_t stackDepth;
    
    // Timing control
    TickType_t lastWakeTime;
    static constexpr uint32_t TELEMETRY_INTERVAL_MS = 50;  // Telemetry update rate
    
public:
    ActiveMotionNode() 
        : cmdQueue(nullptr)
        , telQueue(nullptr)
        , taskHandle(nullptr)
        , taskName(nullptr)
        , priority(0)
        , stackDepth(0)
        , lastWakeTime(0) {
    }
    
    virtual ~ActiveMotionNode() {
        if (cmdQueue != nullptr) {
            vQueueDelete(cmdQueue);
        }
        if (telQueue != nullptr) {
            vQueueDelete(telQueue);
        }
    }
    
    /**
     * @brief Initialize hardware resources for this subsystem.
     * Called once during task startup.
     */
    virtual void hwInit() = 0;
    
    /**
     * @brief Process a single command from the command queue.
     * @param cmd The command to process
     */
    virtual void processCommand(const CmdType& cmd) = 0;
    
    /**
     * @brief Perform periodic hardware updates (e.g., ramping, PWM output).
     * Called every task cycle regardless of commands.
     */
    virtual void hwUpdate() = 0;
    
    /**
     * @brief Generate current telemetry data for publishing.
     * @return Telemetry structure with current state
     */
    virtual TelType generateTelemetry() = 0;
    
    /**
     * @brief Main task loop - drains commands, updates hardware, publishes telemetry.
     */
    void taskLoop() {
        hwInit();
        lastWakeTime = xTaskGetTickCount();
        
        uint32_t telemetryCounter = 0;
        
        while (1) {
            // 1. Drain all pending commands (non-blocking)
            CmdType cmd;
            while (xQueueReceive(cmdQueue, &cmd, 0) == pdTRUE) {
                processCommand(cmd);
            }
            
            // 2. Perform hardware update
            hwUpdate();
            
            // 3. Publish telemetry at fixed interval
            telemetryCounter += TASK_UPDATE_INTERVAL_MS;
            if (telemetryCounter >= TELEMETRY_INTERVAL_MS) {
                telemetryCounter = 0;
                TelType tel = generateTelemetry();
                // Use overwrite mode (mailbox semantics)
                xQueueOverwrite(telQueue, &tel);
            }
            
            // 4. Wait for next cycle
            vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(TASK_UPDATE_INTERVAL_MS));
        }
    }
    
    /**
     * @brief Start the FreeRTOS task for this node.
     * @param name Task name for debugging
     * @param stackDepth Stack size in words
     * @param priority Task priority (higher = more urgent)
     */
    bool start(const char* name, uint32_t stackDepth, UBaseType_t priority) {
        this->taskName = name;
        this->stackDepth = stackDepth;
        this->priority = priority;
        
        // Create queues before task starts
        cmdQueue = xQueueCreate(10, sizeof(CmdType));
        telQueue = xQueueCreate(1, sizeof(TelType));
        
        if (cmdQueue == nullptr || telQueue == nullptr) {
            return false;
        }
        
        // Create the task - taskLoop is the entry point
        BaseType_t result = xTaskCreate(
            [](void* pvParameters) {
                static_cast<ActiveMotionNode*>(pvParameters)->taskLoop();
            },
            name,
            stackDepth,
            this,
            priority,
            &taskHandle
        );
        
        return (result == pdPASS);
    }
    
    /**
     * @brief Send a command to this node.
     * @param cmd Command to send
     * @param timeout Timeout in ticks (use portMAX_DELAY for infinite)
     * @return pdTRUE if successful
     */
    bool sendCommand(const CmdType& cmd, TickType_t timeout = portMAX_DELAY) {
        if (cmdQueue == nullptr) return false;
        return xQueueSend(cmdQueue, &cmd, timeout) == pdPASS;
    }
    
    /**
     * @brief Peek at current telemetry without removing from queue.
     * @param tel Output telemetry structure
     * @param timeout Timeout in ticks
     * @return pdTRUE if successful
     */
    bool peekTelemetry(TelType& tel, TickType_t timeout = 0) {
        if (telQueue == nullptr) return false;
        return xQueuePeek(telQueue, &tel, timeout) == pdPASS;
    }
    
    /**
     * @brief Get the command queue handle for direct access.
     */
    QueueHandle_t getCmdQueue() const { return cmdQueue; }
    
    /**
     * @brief Get the telemetry queue handle for direct access.
     */
};
