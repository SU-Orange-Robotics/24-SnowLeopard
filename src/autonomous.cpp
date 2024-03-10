#include "autonomous.h"
#include "vex.h"
#include "robot-config.h"
#include "drive.h"
#include "intakeCat.h"
#include "../seed/include/auto-commands.h"
#include <chrono>
#include"wings.h"


// declare helper functions
void driveForwardTimed(double pow, double time);
void greenTurnTimed(double pow, double time);
void push_ball();
void go_over();
void intake_and_shoot(int loopCount);
int count = 0;

/* ------------------------------------------------ */
/* ------ Actual Competition Auton goes here ------ */
/* ------------------------------------------------ */

// void oneKick() {
//   ballKicker.setReversed(false);
//   ballKicker.spinToPosition(-170, deg, 100, velocityUnits::pct, true);
//   wait(0.2, sec);
//   ballKicker.spinToPosition(0, deg, 100, velocityUnits::pct, true);
// }

// void kickBalls(int numberOfTries) {
//   for (int i = 0; i < numberOfTries; i++) {
//     oneKick();
//     wait(1.5, sec);
//   }
// }

void lowerCat() {
  while (catapultRot.angle(rotationUnits::deg) > 74) {
    catapultLower();
    wait(5, msec);
  }
  catapultStop();
}

// void autonomous_competition(void) {
//   // IMU calibration

//   lowerCat();
//   int b = 2;
//   imu.calibrate();

//   while (imu.isCalibrating()) {
//     wait(100, msec);
//   }

//   // Controller1.Screen.clearScreen();
//   Controller1.Screen.setCursor(2,1);
//   Controller1.Screen.print("IMU Calibrated");
  
//   kickBalls(5);
  
//   driveForwardTimed(-30, 1.1);
//   turnToTargetIMUOnly(drive, 40, 40);
//   driveForwardTimed(-50, 5);
//   turnToTargetIMUOnly(drive, 40, 90);
//   driveForwardTimed(-50, 1);
//   turnToTargetIMUOnly(drive, 40, 135);
//   driveForwardTimed(-100, 2);
//   driveForwardTimed(50, 1);
//   driveForwardTimed(-100, 1.5);
//   driveForwardTimed(50, 1);
//   drive.stop();
// }

void greenReleaseIntake() {
    // lower cat
    catapultLower();
    wait(0.6, sec);
    catapultStop();

    // free intake
    drive.driveForward(80);
    wait(0.2, sec);
    drive.stop();

    drive.driveForward(-100);
    wait(0.6, sec);
    drive.stop();

    drive.driveForward(80);
    wait(0.2, sec);
    drive.stop();
}

// ========== some helper functions & sub-routines ==========
void driveForwardTimedSmoothly(double pow, double time) {
  // auto startTime = std::chrono::high_resolution_clock::now();

  // if (pow > 0) {
  //   for (int i = 10; i < pow; i++) {
  //     // Check elapsed time
  //     auto currentTime = std::chrono::high_resolution_clock::now();
  //     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();
      
  //     if (elapsedTime >= time) break; // Stop if the specified time has passed
      
  //     drive.driveForward(i);
  //     wait(10, msec);
  //   }
  // } else {
  //   for (int i = 10; i > pow; i--) {
  //     // Check elapsed time
  //     auto currentTime = std::chrono::high_resolution_clock::now();
  //     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();
      
  //     if (elapsedTime >= time) break; // Stop if the specified time has passed
      
  //     drive.driveForward(i);
  //     wait(10, msec);
  //   }
  // }

  // drive.stop();
}
void driveForwardTimed(double pow, double time) {
  drive.driveForward(pow);
  wait(time, sec);
  drive.stop();
}

// void greenTurnTimed(double pow, double time) {
//   drive.leftDrive(pow);
//   drive.rightDrive(-1 * pow);
//   wait(time, sec);
//   drive.stop();
// }

void push_ball() {
  int i;
  for (i = 0; i < 1; i++) {
    drive.driveForward(-80);
    wait(0.7, sec);
    drive.stop();

    wait(0.2, sec);

    drive.driveForward(60);
    wait(0.5, sec);
    drive.stop();
  }
}


void go_over() {
  //
    intakeStop();

    turnToTargetIMUOnly(drive, 40, 45);
    driveForwardTimed(-80, 1.2);
    drive.rightDrive(80);
    drive.leftDrive(-80);
    wait(1, sec);
    drive.stop();
    turnToTargetIMUOnly(drive, 40, 90);
  
    // driveForwardTimed(-100, 4);

    double startPosition = gps1.yPosition();
    if (startPosition > 0) {
      while (gps1.yPosition() > -500) {
        drive.driveForward(-100);
      }
    } else {
      while (gps1.yPosition() < 500) {
        drive.driveForward(-100);
      }
    }
    
    drive.stop();
    wait(0.2, sec);
    turnToTargetIMUOnly(drive, 40, 0);
    driveForwardTimed(-60, 0.8);
  
    turnToTargetIMUOnly(drive, 40, 90);
    driveForwardTimed(50, 0.7);
    
    wings.toggleWings();
    wait(0.8, sec);
    driveForwardTimed(-100, 1);
    wait(0.2, sec);
    driveForwardTimed(80, 1);
}

void intake_and_shoot(int loopCount) {

  for (int i = 0; i < loopCount; i++) {
    if (!catInPosArmed()) {
      catapultArm();
    }

    // turn on intake
    intakeSpin(true);

    // move forward
    
    driveForwardTimed(80, .80);

    wait(0.8, sec);

    // check if ball in
    if (colorSensor.isNearObject()) {
      // shoot
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("SHOOTING");
      
      driveForwardTimed(-100, 0.6);

      ///The forloop has been eliminated///
      turnToTargetIMUOnly(drive, 40, 70);
      catapultLaunch();
        // these two lines here are what does the automatic arming of the catapult.
      waitUntil(getCatAccel() <= 0.1); // <-- might be blocking, which isnt desirable
      catapultArm();
      // count++;

      wait(0.1, sec);

    } else {

      driveForwardTimed(-50, 0.3);
      driveForwardTimed(50, 0.7);
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("not shooting");
      Controller1.Screen.print(colorSensor.hue());

      wait(0.5, sec);

      continue;
    }
   
    // drive back
    turnToTargetIMUOnly(drive, 40, 45);
    
    wait(0.2, sec);
  }
}

// ========== retired autonomous routines ==========
void autonomous_skills_auto() {
 


  // push ball in, try three times
  //push_ball();

  if (!catInPosArmed()) {
    catapultArm();
  }
  if (!catInPosArmed()) {
    catapultArm();
  }
  // turn right to 55
  // greenTurnToTarget(35, 30);
  
  driveForwardTimed(40, 1); //pow stands for power, 0~100

  // greenTurnToTarget(50, 315);

  driveForwardTimed(40, 0.7);
 
  intake_and_shoot(8);
  //}
  go_over();
}

void green_autonomous() {
  
  greenReleaseIntake();


  catapultLower();
  wait(0.6, sec);
  catapultStop();
}

