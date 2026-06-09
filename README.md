# PEACH STEM - Suspension and Transfer of Exfoliated Macrophages

![PEACH STEM Header](HEADER.png)

An ESP32-based embedded control system for automated laboratory cell handling with precision motion control.

## Overview

PEACH_STEM is a multi-axis robotic control system designed for pick-and-drop operations in laboratory environments. Built on the ESP32 platform with FreeRTOS, it provides deterministic real-time control of 3 stepper motors.

**Current Branch Status**: v1.0 - Fully refactored to a DRY Active Object architecture with lock-free generic axis queues, dynamic NVS limit mapping for autonomous sequences, cross-axis safety interlocks, and adjustable UI speeds.

## Features

- **Multi-task FreeRTOS Architecture**: Concurrent task management for motor control, encoder reading, and OLED display.
- **Dynamic Sequence Engine**: Interruptible step-based autonomous operation with E-STOP support and dynamic NVS limit lookups.
- **Precision Motion Control (`StepperAxisNode`)**:
  - **Stepper 0 (Lift Node)**: Raises and lowers the petri dish.
  - **Stepper 1 (Arm Node)**: Lowers and raises the scraper arm.
  - **Stepper 2 (Rotation Node)**: Rotates the dish.
  - All axes use TMC2209 stepper motor drivers with dynamic StallGuard support and procedural GOTO tracking.
- **Real-time Feedback**: Adafruit Seesaw I2C Quadrature encoder integration for closed-loop position control and UI manipulation.
- **User Interface**: SPI OLED LCD display (U8g2) for status monitoring, limit setting, and speed adjustments.
- **Safety Interlocks**:
  - Cross-axis telemetry reads block collisions (e.g., preventing lift movement if the scraper arm is not clear).
  - Emergency stop (E-STOP) with immediate sequence interruption.
- **State Persistence**: Non-volatile storage (NVS) for physical limits and custom sequence speeds (Jog/Go for all axes).

## Hardware Requirements

- **Microcontroller**: ESP32 DevKit
- **Motor Drivers**:
  - 3x TMC2209 stepper motor drivers (UART configurable, StallGuard ready)
- **Display**: SPI OLED LCD
- **Encoder**: Adafruit I2C Quadrature rotary encoder (4 encoders)
- **Sensors**: Motor StallGuard for limit homing.

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

```bash
PEACH_STEM/
├── src/
│   ├── main.cpp              # Entry point and FreeRTOS task initialization
│   ├── messaging.h           # Unified `AxisCommand` structs and lock-free queue definitions
│   ├── HardwareConfig.h      # Centralized pinout and hardware definitions
│   ├── core/                 # Core system managers
│   │   ├── InputManager.*    # UI and encoder interaction logic
│   │   ├── SequenceEngine.*  # Autonomous step-based sequence runner
│   │   ├── StorageManager.*  # NVS wrapper for saving/loading limits and speeds
│   │   └── SystemState.h     # Global UI state enum and sequence definitions
│   ├── tasks/                # FreeRTOS task implementations (Active Objects)
│   │   ├── ActiveMotionNode.h # Base template for all motion nodes
│   │   ├── StepperAxisNode.*  # Generic DRY base class handling tracking & limits
│   │   ├── DishLiftNode.*     # Lift specific interlocks
│   │   ├── ScraperArmNode.*   # Arm specific interlocks
│   │   └── DishRotationNode.* # Rotation specific interlocks
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

- **Level 0 (Main Menu)**: Select between Shutdown, Stepper 0 (Arm), Stepper 1 (Rot), Stepper 2 (Z), and Auto.
- **Level 1 (Sub-Menu)**: Set limits (Clear/Scrape/Home/Tilt), adjust Jog/Go speeds, tune StallGuard, or clear calibration.

### Sequence Engine

The autonomous sequence engine orchestrates lock-free messages to the axis nodes. It implements standard sequences such as:
1.  **Auto**: Lowers the arm, rotates the dish, raises the arm, and iteratively mixes (lifts/lowers) the dish.
2.  **Shutdown**: Lowers the dish (Home) and raises the arm (Clear).

Each sequence step checks for E-STOP events, ensuring immediate hardware shutdown on user intervention.

## Configuration

Hardware pins and global constants are located in `src/HardwareConfig.h`.
Speeds, Limits, and StallGuard thresholds are dynamically configurable at runtime via the UI and automatically persisted to NVS.

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

**Active Development** - Current focus on refining the autonomous sequence engine, StallGuard thresholds, and UI reliability.

### Recent Changes (v1.0 Refactor)

- **DRY Architecture Refactor**: Introduced `StepperAxisNode`, abstracting hundreds of lines of duplicate proportional-tracking, homing, StallGuard, and soft endstop logic.
- **Cross-Axis Interlocks**: Nodes actively peek at sibling telemetry queues to prevent mechanical collisions (e.g. blocking arm movement if the lift isn't cleared).
- **Unified Messaging**: All motion nodes now use a generic `AxisCommand` and `AxisTelemetry` schema (`messaging.h`), drastically simplifying system integration.
- **Dynamic UI Tuning**: StallGuard thresholds and precise Go/Jog speeds are directly tuneable in the physical UI and saved dynamically to NVS.
