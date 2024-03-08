#include "odometry.h"

// Odometry-> take current heading, target heading, and decide which way to turn.

Odometry::Odometry(inertial* inertialSensor, gps* gpsSensor, mode newMode = defaultMode) 
  : imu(inertialSensor), gpsSensor(gpsSensor), odomMode(newMode)
{
  currentHeading = 0;
}

void Odometry::resetHeading() {
  currentHeading = 0;
}

void Odometry::getIMUHeading() {
  currentHeading = imu->heading();
}

void printLocation(Brain& brain) {

  brain.Screen.setCursor(1, 1);
  // brain.Screen.print("X: %f", x);
  // brain.Screen.setCursor(2, 1);
  // brain.Screen.print("Y: %f", y);
  // brain.Screen.setCursor(3, 1);
  brain.Screen.print("Heading: %f", heading);
}