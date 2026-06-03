#pragma once
#include <stdint.h>

// --- Enums and Menus ---
enum DeviceMode { IDLE, PICKUP_CELL, DROPOFF_CELL };
enum ActuatorDirection { ACT_STOP = 0, ACT_FORWARD, ACT_REVERSE };

// --- S4 Hierarchical Menu ---
enum S4Level0 { S4_ARM, S4_ACT, S4_Z, S4_AUTO, S4_LEVEL0_COUNT };

// Sub-menu indices for Arm
#define S4_ARM_TIP   0
#define S4_ARM_CLEAR 1
#define S4_ARM_BACK  2
#define S4_ARM_COUNT 3

// Sub-menu indices for Actuator & Z (same layout)
#define S4_POS_TOP   0
#define S4_POS_MID   1
#define S4_POS_BOT   2
#define S4_POS_BACK  3
#define S4_POS_COUNT 4



// --- Sequence Engine Types ---
enum SequenceAction {
  SEQ_MOVE_Z,        // Move Z-axis to target position (deterministic)
  SEQ_MOVE_ARM,      // Move Arm
  SEQ_MOVE_ACTUATOR, // Set actuator to target percent
  SEQ_WAIT_MS,       // Interruptible delay (target = milliseconds)
  SEQ_WAIT_USER      // Wait for user button press to continue
};

struct SequenceStep {
  SequenceAction action;
  int target;          // Position/percent/ms/limitIndex depending on action
  int limitIdx;        // Z-position limit index (0=Bot, 1=Mid, 2=Top) (only for SEQ_MOVE_Z)
  int actuatorSpeed;   // Actuator PWM speed (0-255) (only for SEQ_MOVE_ACTUATOR)
  const char *message; // LCD message (NULL = no update)
};

// Minimal SystemState - only controller-level state, no subsystem tracking
struct SystemState {
  DeviceMode mode;
  
  // S4 Menu state
  S4Level0 s4Menu;
  int s4SubMenu;
  bool s4InSubMenu;
  

  
  // Collision Detection (shared flag)
  bool collisionDetected;
  uint32_t collisionTimestamp;
  
  // UI-configurable global actuator slow speed
  uint8_t actuatorSlowSpeed;
};
