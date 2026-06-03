# PEACH PIT - Robotic Cell Dropper

An ESP32-based embedded control system for automated laboratory cell handling with precision motion control.

## Overview

PEACH_PIT is a multi-axis robotic control system designed for pick-and-drop operations in laboratory environments. Built on the ESP32 platform with FreeRTOS, it provides deterministic real-time control of stepper motors, servos, and linear actuators.

**Current Branch Status**: v3.0 - Fully refactored to an Active Object architecture with lock-free queues, dynamic NVS limit mapping for autonomous sequences, and adjustable UI speeds.

## Features

- **Multi-task FreeRTOS Architecture**: Concurrent task management for motor control, encoder reading, LCD display, servo positioning, and actuator control
- **Dynamic Sequence Engine**: Interruptible step-based autonomous operation with E-STOP support and dynamic NVS limit lookups.
- **Precision Motion Control**:
  - Stepper motor with TMC2209 driver, optical endstops, and StallGuard
  - Servo motor (Arm) with NVS-persisted calibration targets
  - Linear actuator with position tracking and adjustable PWM speeds
- **Real-time Feedback**: Quadrature encoder integration for closed-loop position control and UI manipulation
- **User Interface**: OLED LCD display (U8g2) for status monitoring, limit setting, and speed adjustments
- **Safety Features**:
  - Emergency stop (E-STOP) with immediate sequence interruption
  - Interlocks preventing arm collisions with microscopes
  - Homing routine with limit detection
- **State Persistence**: Non-volatile storage (NVS) for physical limits (Z and Actuator) and custom sequence speeds.
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

1. Install PlatformIO dependencies:

```bash
pio install
```

1. Configure upload port in `platformio.ini` if necessary

2. Build and upload:

```bash
pio run --target upload
```

1. Open serial monitor:

```bash
pio device monitor
```

## Project Structure

```bash
PEACH_PIT/
├── src/
│   ├── main.cpp              # Entry point and task initialization
│   ├── messaging.h           # System-wide structs and lock-free queue definitions
│   ├── core/                 # Core system managers
│   │   ├── InputManager.*    # UI and encoder interaction logic
│   │   ├── SequenceEngine.*  # Autonomous 10-step sequence runner
│   │   ├── StorageManager.*  # NVS wrapper for saving/loading limits
│   │   ├── SystemState.h     # Global UI state enum
│   │   └── NetworkManager.*  # OTA updates and WiFi handling
│   ├── tasks/                # FreeRTOS task implementations (Active Objects)
│   │   ├── ActiveMotionNode.h# Base template for all motion nodes
│   │   ├── ActuatorNode.*    # Linear actuator control loop
│   │   ├── ArmNode.*         # Servo motor arm control loop
│   │   └── MotorNode.*       # Stepper motor Z-axis control loop
│   └── drivers/              # Hardware abstraction layer
│       ├── EncoderDriver.*   # Quadrature encoder interface
│       ├── LCDDriver.*       # OLED display driver
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

The autonomous sequence engine uses a step-based approach defined in `SystemState.h`:

```cpp
enum SequenceAction {
  SEQ_MOVE_Z,        // Move Z-axis to target limit index (0=Bot, 1=Mid, 2=Top)
  SEQ_MOVE_ARM,      // Move Arm (0-100%)
  SEQ_MOVE_ACTUATOR, // Set actuator to target limit index (0,1,2) with custom speed
  SEQ_WAIT_MS,       // Interruptible delay
  SEQ_WAIT_USER      // Wait for user button press
};

struct SequenceStep {
  SequenceAction action;
  int target;          // Percent, ms, or limitIndex depending on action
  int limitIdx;        // Z-position limit index
  int actuatorSpeed;   // Actuator PWM speed (0-255)
  const char *message; // LCD message
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
| ----------- | --------- | ----------- |
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

### Recent Changes (v3.0)

- **Architecture Refactor**: Replaced shared-memory `controller` blob with `ActiveMotionNode` architecture. Tasks communicate exclusively via lock-free FreeRTOS queues (`messaging.h`).
- **Data-Driven Subsystems**: Extracted `StorageManager`, `InputManager`, and `SequenceEngine`.
- **Dynamic Limits**: `SequenceEngine` now dynamically looks up calibrated physical positions for the Z-Axis and Actuator at runtime, eliminating hardcoded depths.
- **Adjustable Speeds**: Actuator slow dispense speed is now configurable via the physical Encoder UI and saved to NVS.

---

**Last updated**: Based on commit "refactor: replace hardcoded autonomous sequence with a dynamic, interruptible step-based engine supporting E-STOP and UI state synchronization."
