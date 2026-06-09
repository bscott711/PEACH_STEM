# PEACH STEM - Suspension and Transfer of Exfoliated Macrophages

![PEACH STEM Header](HEADER.png)

An ESP32-based embedded control system for automated laboratory cell handling with precision motion control.

## Overview

PEACH_STEM is a multi-axis robotic control system designed for pick-and-drop operations in laboratory environments. Built on the ESP32 platform with FreeRTOS, it provides deterministic real-time control of 3 stepper motors.

**Current Branch Status**: v1.0 - Fully refactored to an Active Object architecture with lock-free queues, dynamic NVS limit mapping for autonomous sequences, and adjustable UI speeds.

## Features

- **Multi-task FreeRTOS Architecture**: Concurrent task management for motor control, encoder reading, and LCD display.
- **Dynamic Sequence Engine**: Interruptible step-based autonomous operation with E-STOP support and dynamic NVS limit lookups.
- **Precision Motion Control**:
  - **Stepper 0**: Rotates the dish.
  - **Stepper 1**: Lowers and raises the scraper arm.
  - **Stepper 2**: Raises and lowers the petri dish.
  - All axes use TMC2209 stepper motor drivers. Stall Guard support will be added.
- **Real-time Feedback**: Adafruit Seesaw I2C Quadrature encoder integration for closed-loop position control and UI manipulation.
- **User Interface**: SPI OLED LCD display (U8g2) for status monitoring, limit setting, and speed adjustments.
- **Safety Features**:
  - Emergency stop (E-STOP) with immediate sequence interruption.
  - Hardware optical endstop overrides.
- **State Persistence**: Non-volatile storage (NVS) for physical limits and custom sequence speeds (Jog/Go for all axes).

## Hardware Requirements

- **Microcontroller**: ESP32 DevKit
- **Motor Drivers**:
  - 3x TMC2209 stepper motor drivers (UART configurable, Stall Guard ready)
- **Display**: SPI OLED LCD
- **Encoder**: Adafruit I2C Quadrature rotary encoder (4 encoders)
- **Sensors**: Optical Endstops

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
PEACH_STEM/
├── src/
│   ├── main.cpp              # Entry point and FreeRTOS task initialization
│   ├── messaging.h           # System-wide structs and lock-free queue definitions
│   ├── HardwareConfig.h      # Centralized pinout and hardware definitions
│   ├── core/                 # Core system managers
│   │   ├── InputManager.*    # UI and encoder interaction logic
│   │   ├── SequenceEngine.*  # Autonomous step-based sequence runner
│   │   ├── StorageManager.*  # NVS wrapper for saving/loading limits and speeds
│   │   └── SystemState.h     # Global UI state enum and sequence definitions
│   ├── tasks/                # FreeRTOS task implementations (Active Objects)
│   │   ├── ActiveMotionNode.h # Base template for all motion nodes
│   │   └── MotorNode.*       # Stepper motor control loops
│   └── drivers/              # Hardware abstraction layer
│       ├── EncoderDriver.*   # Quadrature encoder interface
│       └── LCDDriver.*       # OLED display driver
├── include/                  # Public headers
├── lib/                      # External libraries
├── test/                     # Unit tests
├── platformio.ini           # PlatformIO project configuration
└── README.md                # This file
```

## Usage

### User Interface

The UI is driven by an encoder with short-press, long-press, and double-press actions:

- **Level 0 (Main Menu)**: Select between Shutdown, Stepper 0, Stepper 1, Stepper 2, and Auto.
- **Level 1 (Sub-Menu)**: Set limits (Top/Mid/Bot), adjust Jog/Go speeds, or clear calibration.

### Sequence Engine

The autonomous sequence engine uses a step-based approach defined in `SystemState.h`:

```cpp
enum SequenceAction {
  SEQ_MOVE_ROTATE,    // Move Stepper 0 (Rotate) to target limit index
  SEQ_MOVE_SCRAPE,    // Move Stepper 1 (Scrape) to target limit index
  SEQ_MOVE_TILT,      // Move Stepper 2 (Tilt) to target limit index
  SEQ_WAIT_MS,        // Interruptible delay
  SEQ_WAIT_USER,      // Wait for user button press
  SEQ_MOVE_ALL        // Move multiple steppers simultaneously
};
```

Each sequence step can be interrupted by E-STOP events, ensuring safe operation.

## Configuration

Hardware pins and global constants are located in `src/HardwareConfig.h`.
Speeds and limits are dynamically configurable at runtime via the UI and persisted to NVS.

## Task Priorities

FreeRTOS task priorities (higher number = higher priority):

- **EncoderTask** (3): Critical for position feedback
- **Controller** (3): Main UI and input control logic
- **MotorNodes** (2): Stepper 0, 1, and 2 control loops
- **LCD** (2): Display refresh

## License

MIT License

Copyright (c) 2026 PEACH STEM Contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Project Status

**Active Development** - Current focus on refining the autonomous sequence engine and UI reliability.

### Recent Changes (v3.0)

- **Architecture Refactor**: Replaced shared-memory `controller` blob with `ActiveMotionNode` architecture. Tasks communicate exclusively via lock-free FreeRTOS queues (`messaging.h`).
- **Data-Driven Subsystems**: Extracted `StorageManager`, `InputManager`, and `SequenceEngine`.
- **Dynamic UI Speeds**: Jog and GOTO speeds for all axes are now configurable via the physical Encoder UI and saved to NVS.
- **Dynamic Limits**: `SequenceEngine` now dynamically looks up calibrated physical positions for the steppers at runtime, eliminating hardcoded depths.
- **Hardware Abstraction**: Centralized pin definitions to `HardwareConfig.h`.
