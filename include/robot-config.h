#include "vex.h"
#include "drive.h"

using namespace vex;

// A global instance of competition
extern competition Competition;

// define your global instances of motors and other devices here

// 8 motor 4 WHEEL mechanum drive
extern motor LeftMotorA;
extern motor LeftMotorB;
extern motor RightMotorA;
extern motor RightMotorB;

extern motor intake;

extern motor catapultA;
extern motor catapultB;

extern rotation catapultRot;

extern controller Controller1;
extern inertial IMU;
extern gps gps1;

extern Drive drive;