# PEACH_PIT - Robotic Cell Dropper

An ESP32-based embedded control system for automated laboratory cell handling with precision motion control.

## Overview

PEACH_PIT is a multi-axis robotic control system designed for pick-and-drop operations in laboratory environments. Built on the ESP32 platform with FreeRTOS, it provides deterministic real-time control of stepper motors, servos, and linear actuators.

**Current Branch Status**: Refactored autonomous sequence engine with dynamic, interruptible step-based execution supporting E-STOP and UI state synchronization.

## Features

- **Multi-task FreeRTOS Architecture**: Concurrent task management for motor control, encoder reading, LCD display, servo positioning, and actuator control
- **Dynamic Sequence Engine**: Interruptible step-based autonomous operation with E-STOP support
- **Precision Motion Control**:
  - Stepper motor with TMC2209 driver and StallGuard stall detection
  - Servo motor with NVS-persisted calibration
  - Linear actuator with position tracking
- **Real-time Feedback**: Quadrature encoder integration for closed-loop position control
- **User Interface**: OLED LCD display (U8g2) for status monitoring and menu navigation
- **Safety Features**:
  - Emergency stop (E-STOP) with immediate sequence interruption
  - Collision detection with timestamp logging
  - Homing routine with limit detection
- **State Persistence**: Non-volatile storage (NVS) for system state and servo calibration
- **StallGuard Live Tuning**: Real-time threshold adjustment for optimal motor performance

## Hardware Requirements

- **Microcontroller**: ESP32 DevKit
- **Motor Drivers**: 
  - TMC2209 stepper motor driver
  - Servo driver (Adafruit Seesaw compatible)
- **Display**: I2C OLED LCD
- **Encoder**: Quadrature rotary encoder (I2C)
- **Actuators**: Linear actuator with position feedback

## Installation

### Prerequisites

- [PlatformIO](https://platformio.org/install) or VS Code with PlatformIO extension
- Python 3.x

### Setup

1. Clone the repository:
```bash
git clone https://gitlab.com/ericjohnson0987/robotic-cell-dropper.git
cd robotic-cell-dropper
```

2. Install PlatformIO dependencies:
```bash
pio install
```

3. Configure upload port in `platformio.ini` if necessary

4. Build and upload:
```bash
pio run --target upload
```

5. Open serial monitor:
```bash
pio device monitor
```

## Project Structure

```
PEACH_PIT/
├── src/
│   ├── main.cpp              # Entry point and task initialization
│   ├── controller.h          # System state, configuration, and sequence definitions
│   ├── drivers/              # Hardware abstraction layer
│   │   ├── EncoderDriver.*   # Quadrature encoder interface
│   │   ├── HBridgeDriver.*   # Motor bridge control
│   │   ├── LCDDriver.*       # OLED display driver
│   │   ├── MotorDriver.*     # Stepper motor control (TMC2209)
│   │   └── ServoDriver.*     # Servo motor control
│   └── tasks/                # FreeRTOS task implementations
│       ├── encoder_task.*    # Encoder reading task
│       ├── motor_task.*      # Motor control task
│       ├── LCD_task.*        # Display refresh task
│       ├── servo_task.*      # Servo positioning task
│       └── actuator_task.*   # Linear actuator control
├── include/                  # Public headers
├── lib/                      # External libraries
├── test/                     # Unit tests
├── CMakeLists.txt           # CMake build configuration
├── platformio.ini           # PlatformIO project configuration
└── README.md                # This file
```

## Usage

### System Modes

The controller operates in three primary modes:

1. **IDLE**: System initialized, awaiting commands
2. **PICKUP_CELL**: Automated sequence for picking up sample cells
3. **DROPOFF_CELL**: Automated sequence for depositing sample cells

### Sequence Engine

The autonomous sequence engine uses a step-based approach defined in `controller.h`:

```cpp
enum SequenceAction {
  SEQ_MOVE_Z,        // Move Z-axis to target position
  SEQ_MOVE_SERVO,    // Set servo to target percent
  SEQ_MOVE_ACTUATOR, // Set actuator to target percent
  SEQ_WAIT_MS,       // Interruptible delay
  SEQ_WAIT_USER      // Wait for user button press
};
```

Each sequence step can be interrupted by E-STOP events, ensuring safe operation.

### Manual Controls

- **Servo Adjustment Mode**: Fine-tune servo position manually
- **Servo Calibration**: Two-point calibration (start/center) persisted to NVS
- **StallGuard Tuning**: Live threshold adjustment for stall detection sensitivity

## Configuration

Key configuration parameters in `controller.h`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MOTOR_SPEED_SCALE_FACTOR` | 333 | Speed scaling for motor calculations |
| `AUTO_SEQUENCE_SPEED` | 120000 | Default speed for autonomous sequences |
| `AUTO_SEQUENCE_DURATION_MS` | 15000 | Duration for auto sequence moves |
| `SERVO_MIN/MAX_PERCENT` | 0/100 | Servo range limits |
| `ACTUATOR_STEP_PERCENT` | 10 | Actuator movement increment |

## Task Priorities

FreeRTOS task priorities (higher number = higher priority):

- **EncoderTask** (3): Critical for position feedback
- **Controller** (3): Main control logic
- **Update Motor** (2): Motor speed/direction updates
- **Actuator** (2): Linear actuator positioning
- **Servo** (2): Servo angle control
- **LCD** (2): Display refresh

## Safety Considerations

⚠️ **E-STOP**: The system supports emergency stop via event flags. When triggered:
- All autonomous sequences halt immediately
- Motor motion is suspended
- System awaits manual reset

⚠️ **Collision Detection**: The system monitors for unexpected resistance and logs collision events with timestamps.

## Development

### Building

```bash
pio run
```

### Uploading

```bash
pio run --target upload
```

### Serial Monitor

```bash
pio device monitor
```

### Debugging

Enable USB serial output at 115200 baud for debugging information.

## Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Merge Request

## Authors and Acknowledgment

- Eric Johnson and the Robotic Cell Development Team

## License

This project is proprietary software developed for laboratory automation.

## Project Status

**Active Development** - Current focus on refining the autonomous sequence engine and improving E-STOP response times.

### Recent Changes

- Replaced hardcoded autonomous sequences with dynamic step-based engine
- Added interruptible sequence execution with E-STOP support
- Implemented UI state synchronization across all tasks
- Enhanced collision detection and logging

---

*Last updated: Based on commit "refactor: replace hardcoded autonomous sequence with a dynamic, interruptible step-based engine supporting E-STOP and UI state synchronization."*
