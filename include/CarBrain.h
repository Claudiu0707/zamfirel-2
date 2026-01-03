#ifndef CAR_BRAIN_H
#define CAR_BRAIN_H

#include "DriveControl.h"
#include "LineFollower.h"

void readInstruction();
void processInstruction();

void setupControl();
#endif