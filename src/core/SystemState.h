#pragma once
#include <stdint.h>

// --- Enums and Menus ---
enum DeviceMode { IDLE, PICKUP_CELL, DROPOFF_CELL };
enum ActuatorDirection { ACT_STOP = 0, ACT_FORWARD, ACT_REVERSE };

// --- S4 Hierarchical Menu ---
enum S4Level0 {
  S4_STOP,
  S4_SCRAPER,
  S4_ROTATION,
  S4_LIFT,
  S4_AUTO,
  S4_LEVEL0_COUNT
};

// Sub-menu indices for Arm
#define S4_SCRAPER_CLEAR 0
#define S4_SCRAPER_SCRAPE 1
#define S4_SCRAPER_DROP_POS 2
#define S4_SCRAPER_JOG_SPD 3
#define S4_SCRAPER_GO_SPD 4
#define S4_SCRAPER_SG_TUNE 5
#define S4_SCRAPER_TEN_CUR 6
#define S4_SCRAPER_BACK 7
#define S4_SCRAPER_COUNT 8

// Sub-menu indices for Rotation
#define S4_ROT_JOG_SPD 0
#define S4_ROT_GO_SPD 1
#define S4_ROT_NUM_ROTATIONS 2
#define S4_ROT_SG_TUNE 3
#define S4_ROT_BACK 4
#define S4_ROT_COUNT 5

// Sub-menu indices for Lift
#define S4_LIFT_HOME 0
#define S4_LIFT_TILT 1
#define S4_LIFT_JOG_SPD 2
#define S4_LIFT_GO_SPD 3
#define S4_LIFT_NUM_MIX 4
#define S4_LIFT_SG_TUNE 5
#define S4_LIFT_BACK 6
#define S4_LIFT_COUNT 7

// --- Sequence Engine Types ---
enum SequenceAction {
  SEQ_MOVE_LIFT,         // Move Z-axis to target limit
  SEQ_MOVE_SCRAPER,      // Move Arm
  SEQ_MOVE_ROTATION,     // Spin rotation motor
  SEQ_WAIT_MS,           // Interruptible delay (target = milliseconds)
  SEQ_WAIT_USER,         // Wait for user button press to continue
  SEQ_MOVE_SCRAPER_AND_Z // Move Arm and Z simultaneously
};

struct SequenceStep {
  SequenceAction action;
  int target;        // Position/percent/ms/limitIndex depending on action
  int limitIdx;      // Z-position limit index (only for SEQ_MOVE_LIFT)
  int actuatorSpeed; // Motor speed
  const char *message; // LCD message (NULL = no update)
};

// Minimal SystemState - only controller-level state, no subsystem tracking
struct SystemState {
  DeviceMode mode;

  // S4 Menu state
  S4Level0 s4Menu;
  uint8_t s4SubMenu;
  bool s4InSubMenu;
  bool s4InSpeedEdit;

  // Configurable Speeds and SG
  int scraperArmJogSpeed;
  int scraperArmGoSpeed;
  int scraperArmSGThreshold;
  int scraperArmDropPos;
  int scraperArmTenCur;
  
  int dishRotationJogSpeed;
  int dishRotationGoSpeed;
  int dishRotationNumRotations;
  int dishRotationSGThreshold;
  
  int dishLiftJogSpeed;
  int dishLiftGoSpeed;
  int dishLiftNumMix;
  int dishLiftSGThreshold;

  // Collision Detection (shared flag)
  bool collisionDetected;
  uint32_t collisionTimestamp;
};
