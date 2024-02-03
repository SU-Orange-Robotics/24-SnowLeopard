#include "vex.h"
#include "drive.h"
#include "wings.h"

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

extern motor wingL;
extern motor wingR;

extern motor catapultA;
extern motor catapultB;
extern rotation catapultRot;

extern inertial IMU;
extern gps gps1;
extern rotation odomLeft;
extern rotation odomRight;
extern rotation odomCenter;

extern optical colorSensor;

extern motor wingLeft;
extern motor wingRight;

extern brain Brain;
extern controller Controller1;

extern Drive drive;
extern Wings wings;