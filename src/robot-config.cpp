#include "vex.h"
#include "drive.h"
#include "robot-config.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

// 8 motor 4 WHEEL mechanum drive
// A front, B back
motor LeftMotorA(PORT17, gearSetting::ratio18_1, false);
motor LeftMotorB(PORT18, gearSetting::ratio18_1, true);
motor RightMotorA(PORT13, gearSetting::ratio18_1, true);
motor RightMotorB(PORT12, gearSetting::ratio18_1, false);

motor intake(PORT20, gearSetting::ratio18_1, false);

motor catapultA(PORT19, gearSetting::ratio18_1, false); //left
motor catapultB(PORT11, gearSetting::ratio18_1, true); //right
//motor catapultC(PORT9, gearSetting::ratio36_1, true);

rotation catapultRot(PORT8, false);


brain Brain;
controller Controller1(controllerType::primary);


inertial IMU(PORT5);
gps gps1(PORT19, 0, 0, distanceUnits::mm, 180); // port, x, y, distance units, angle offset, turn direction?

// declare object-oriented stuff here (that should be globally accessible)
Drive drive;
