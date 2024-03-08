#pragma once

#include "vex.h"

#include <tuple>

std::tuple<bool, double> getTurnStats(double target) {
  double currentHeading = imu.heading();
  if (currentHeading > target) {
    currentHeading -= 360.0;
  }
  double error = target - currentHeading;
  bool right = true;

  if (error >= 180) {
    right = false;
    error = 360.0 - error;
  } else {
    right = true;
  }

  return std::make_tuple(right, error);
}

double limiter(double input, double ceiling) {
  return input > ceiling ? ceiling : input;
}

double flipAngle(double angle) {
  return 360 - angle;
}

// for the target input:
// ^ 90 deg
// |
// |
// |
// | 
// |-----------> 0 deg
void greenTurnToTarget(Drive& drive, double pow, double target, bool holdPosition = false) {
  target = flipAngle(target);
  std::tuple<bool, double> turnDir;

  double error = 0.0;
  bool right;
  double kp = 1.5;
  double ki = 0.01;

  double powOut = 0;

  double integral_error = 0;
  const double treshold = 0.3;

  while (error < treshold || holdPosition) {

    turnDir = getTurnStats(target);
    right = std::get<0>(turnDir);
    error = std::get<1>(turnDir);
    if (error < treshold) {
      drive.stop();
      continue;
    };

    integral_error += error;

    powOut = (pow * (error > 10 ? kp : error/10.0) * kp) + limiter(integral_error * ki * (error / 1.0), 1.5);
  

    if (right) {
      drive.leftDrive(powOut);
      drive.rightDrive(-1 * powOut);
    } else {
      drive.leftDrive(-1 * powOut);
      drive.rightDrive(powOut);
    }

    wait(5, msec);
  }

  //stop bot
  drive.stop();
}