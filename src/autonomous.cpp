#include "autonomous.h"
#include "vex.h"
#include "robot-config.h"
#include "drive.h"
#include "intakeCat.h"
// #include "../seed/include/auto-commands.h"
#include <chrono>

// declare helper functions
void driveForwardTimed(double pow, double time);
void greenTurnTimed(double pow, double time);
void push_ball();
void intake_and_shoot();

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
  for (i = 0; i < 2; i++) {
    drive.driveForward(-80);
    wait(0.7, sec);
    drive.stop();

    wait(0.2, sec);

    drive.driveForward(60);
    wait(0.5, sec);
    drive.stop();
  }
}

void intake_and_shoot() {

  for (int i = 0; i < 12; i++) {
    if (!catInPosArmed()) {
      catapultArm();
    }

    // turn on intake
    intakeSpin(true);

    // move forward
    driveForwardTimed(60, 1.5);

    wait(0.7, sec);

    // check if ball in
    if (colorSensor.isNearObject()) {
      // shoot
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("SHOOTING");
      
      driveForwardTimed(-70, 1.1);

      ///The forloop has been eliminated///
      catapultLaunch();
        // these two lines here are what does the automatic arming of the catapult.
      waitUntil(getCatAccel() <= 0.1); // <-- might be blocking, which isnt desirable
      catapultArm();
      

      wait(0.2, sec);

    } else {
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("not shooting");
      Controller1.Screen.print(colorSensor.hue());

      wait(0.5, sec);

      continue;
    }

    // drive back
    driveForwardTimed(-30, 0.1);
    wait(0.2, sec);
  }
}

// ========== retired autonomous routines ==========
void green_skills_auto() {
  // catapultLower();
  // wait(0.6, sec);
  // catapultStop();



  // push ball in, try three times
  push_ball();

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
  //auto mid_time = std::chrono::system_clock::now();
  //auto rest_time = mid_time- std::chrono::minutes(1);
  //while(std::chrono::system_clock::now() < rest_time){
  // intake and shoot
  intake_and_shoot();
  //}
}

void green_autonomous() {
  
  greenReleaseIntake();


  catapultLower();
  wait(0.6, sec);
  catapultStop();
}

void autonomous_old(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  // catap
  /*
  drive.driveForward(100);
  wait(0.6, sec);
  drive.stop();
  wait(1, sec);

  //drive.goToPointPID(0, 1000);
  */
  //drive.turnAndDrivePID(0, 1000);
  
  /*
  drive.turnPID(0);

  wait(1, sec);

  drive.turnPID(M_PI);

  wait(1, sec);

  drive.turnPID(M_PI / 2);

  wait(1, sec);

  drive.turnPID(5 * M_PI / 6);*/


  // drive.driveForward(-100);
  // wait(650, msec);
  // drive.stop();

  // //catapultArm();

  // drive.turnPID((-1 * M_PI / 4) + 0.0);
  // drive.driveForward(100);
  // wait(800, msec);
  // drive.stop();

  // intakeSpin();

  // int i;
  // for (i = 0; i < 6; i++) {
  //   //reverse away from bar for match load
  //   drive.driveForward(-100);
  //   wait(400, msec);
  //   drive.stop();

  //   catapultLaunch();
  //   wait(500, msec);
  //   catapultArm();

  //   // give time for match load to be loaded (in addition to catapult arm time) and allow for 
  //   drive.turnPID((-1 * M_PI / 4) + 0.0);
  //   wait(500, msec);

  //   //drive forward into bar
  //   drive.driveForward(100);
  //   wait(480, msec);
  //   drive.stop();

  //   //give time for ball to get into catapult
  //   wait(300, msec);
  // }
  // catapultLaunch();
  
  // drive.driveForward(100);

  // green_autonomous();
  green_skills_auto();
}