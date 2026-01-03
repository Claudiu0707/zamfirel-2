#ifndef CAR_BRAIN_H
#define CAR_BRAIN_H

#include "DriveControl.h"
#include "LineFollower.h"

void readInstruction();
void processInstruction();
void handleModeChange();
void runCurrentMode();
void setupControl();
void calibrateLineFollowerData(int component, int length);

bool shouldRunControlLoop();

float convertStringToFloat(unsigned int length, String data);

enum CarMode {
  SETUP_MODE = 0,
  DRIVER_MODE = 1,
  LINE_FOLLOWER_MODE = 2
};

struct Instruction {
  char type;
  int value;
  bool processed;
};

#endif

