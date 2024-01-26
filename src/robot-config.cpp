#include "vex.h"
#include "drive.h"
#include "robot-config.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

// 8 motor 4 WHEEL mechanum drive
motor LeftMotorA(PORT20, gearSetting::ratio18_1, true);

motor LeftMotorB(PORT11, gearSetting::ratio18_1, false);

motor RightMotorA(PORT3, gearSetting::ratio18_1, false);

motor RightMotorB(PORT2, gearSetting::ratio18_1, true);

motor intake(PORT7, gearSetting::ratio18_1, true);

motor catapultA(PORT1, gearSetting::ratio36_1, false);
motor catapultB(PORT10, gearSetting::ratio36_1, false);
motor catapultC(PORT9, gearSetting::ratio36_1, true);

rotation catapultRot(PORT8, false);

controller Controller1(controllerType::primary);
inertial IMU(PORT5);
gps gps1(PORT19, 0, 0, distanceUnits::mm, 180); // port, x, y, distance units, angle offset, turn direction?

// declare object-oriented stuff here (that should be globally accessible)
Drive drive;
