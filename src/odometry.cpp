#include "vex.h"
#include "robot-config.h"
#include "odometry.h"

using namespace vex;

/*
use this to detect when bot is going over the bar
IMU.gyroRate(axisType::xaxis, velocityUnits::dps);
IMU.gyroRate(axisType::yaxis, velocityUnits::dps);

will likely set up some sort of state machine to know when the bot transitions from on the bar to back on the ground
this makes it easy to know when to reset the "global" tracked position based on the gps and re-enable tracking wheels
^ proably not that important since we likely won't ever be in a situation where we want position tracking after crossing the bar


*/