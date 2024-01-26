#include "vex.h"

#pragma once

const double intakePow = 100;
const double catPow = 35;

void stopAutoArming();

void intakeSpin(bool reversed = false);

void intakeStop();

void catapultLower();

void catapultArm();

void catapultLaunch();

void catapultLaunch2();

void catapultStop();

bool catInPosArmed();

bool catInPosShoot();

bool catShooted();