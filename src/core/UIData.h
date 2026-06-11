#pragma once
#include "SystemState.h"

struct UIData {
  bool isAutoRunning;
  DeviceMode currentMode;

  // S1 Arm — jog direction + position
  int scraperArmJogDir;     // -1, 0, +1
  float scraperArmPosition; // Current absolute step position
  int scraperArmPosClear;
  int scraperArmPosScrape;
  bool scraperArmIsMoving;
  int scraperArmSGThreshold;
  int scraperArmSGResult;
  int scraperArmDropPos;
  int scraperArmTenCur;

  // S2 Actuator — jog direction + position
  int dishRotationJogDir;  // -1, 0, +1
  float dishRotationPos;
  int dishRotationNumRotations;
  bool dishRotationIsMoving;
  int dishRotationSGThreshold;
  int dishRotationSGResult;

  // S3 Z Motor — jog direction + position
  int dishLiftJogDir; // -1, 0, +1
  float dishLiftPos;
  float dishLiftPosHome;
  float dishLiftPosTilt;
  bool dishLiftPosHomeSet;
  bool dishLiftPosTiltSet;
  int dishLiftNumMix;
  bool dishLiftIsMoving;
  int dishLiftSGThreshold;
  int dishLiftSGResult;

  // S4 Menu state
  S4Level0 s4Menu;
  int s4SubMenu;
  bool s4InSubMenu;
  bool s4InSpeedEdit;

  // Configurable Speeds
  int scraperArmJogSpeed;
  int scraperArmGoSpeed;
  int dishRotationJogSpeed;
  int dishRotationGoSpeed;
  int dishLiftJogSpeed;
  int dishLiftGoSpeed;
};
