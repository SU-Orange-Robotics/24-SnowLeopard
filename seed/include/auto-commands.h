#pragma once

#include "vex.h"

#include <tuple>

const double initialHeading = 45;

std::tuple<bool, double> getTurnStats(double target) {
  double currentHeading = imu.heading() - initialHeading;
  if (currentHeading > 360.0 || currentHeading > target) {
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

void turnToTargetIMUOnly(Drive& drive, double pow, double target, bool holdPosition = false) {
  target = flipAngle(target);
  std::tuple<bool, double> turnDir;

  double error = 0.0;
  bool right;
  double kp = 0.8;
  double ki = 0.01;

  double powOut = 0;

  double integral_error = 0;
  const double treshold = 0.5;

  turnDir = getTurnStats(target);
  right = std::get<0>(turnDir);
  error = std::get<1>(turnDir);

  while (true) {

    turnDir = getTurnStats(target);
    right = std::get<0>(turnDir);
    error = std::get<1>(turnDir);
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print("error %f", error);
    if (error < treshold) {
      drive.stop();
      break;
    };

    integral_error += error;

    powOut = (pow * (error > 50 ? kp : error/50.0) * kp) + limiter(integral_error * ki * (error / 1.0), 1.5);

    if (right) {
      drive.leftDrive(powOut);
      drive.rightDrive(-1 * powOut);
    } else {
      drive.leftDrive(-1 * powOut);
      drive.rightDrive(powOut);
    }

    wait(10, msec);
  }

  Controller1.Screen.setCursor(2,1);
  Controller1.Screen.print("GOOD");

  //stop bot
  drive.stop();
}