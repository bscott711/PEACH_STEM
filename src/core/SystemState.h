#pragma once
#include <stdint.h>

// --- Enums and Menus ---
enum DeviceMode { IDLE, PICKUP_CELL, DROPOFF_CELL };
enum ActuatorDirection { ACT_STOP = 0, ACT_FORWARD, ACT_REVERSE };
enum Enc1Menu { MENU_ACT_MAN, MENU_ACT_GOTO_TOP, MENU_ACT_GOTO_MID, MENU_ACT_GOTO_BOT, MENU_ACT_SPEED };
enum Enc3Menu { MENU_AUTO, MENU_GOTO_TOP, MENU_GOTO_MID, MENU_GOTO_BOT };

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
  
  // Controller state only (subsystem state moved to Active Nodes)
  Enc1Menu enc1MenuSelection;  // MENU_ACT_MAN, GOTO_TOP, GOTO_MID, GOTO_BOT
  Enc3Menu enc3MenuSelection;
  
  // Collision Detection (shared flag)
  bool collisionDetected;
  uint32_t collisionTimestamp;
  
  // UI-configurable global actuator slow speed
  uint8_t actuatorSlowSpeed;
};
