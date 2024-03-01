/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       zhuowz                                                    */
/*    Created:      10/6/2023, 6:48:52 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "drive.h"
#include "robot-config.h"
#include "intakeCat.h"
#include "wings.h"
#include "autonomous.h"
#include "odometry.h"

using namespace vex;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  wings.initWings();

  LeftMotorA.setStopping(brakeType::brake);
  LeftMotorB.setStopping(brakeType::brake);
  RightMotorA.setStopping(brakeType::brake);
  RightMotorB.setStopping(brakeType::brake);
  
  double maxCurrent = 2.5; //hardware maximum current is 2.5A

  LeftMotorA.setMaxTorque(maxCurrent, currentUnits::amp);
  LeftMotorB.setMaxTorque(maxCurrent, currentUnits::amp);
  RightMotorA.setMaxTorque(maxCurrent, currentUnits::amp);
  RightMotorB.setMaxTorque(maxCurrent, currentUnits::amp);


  catapultA.setStopping(brakeType::hold);
  catapultB.setStopping(brakeType::hold);

  intake.setStopping(brakeType::brake);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void driveForwardTimed(double pow, double time) {
  drive.driveForward(pow);
  wait(time, sec);
  drive.stop();
}

void greenTurnTimed(double pow, double time) {
  drive.leftDrive(pow);
  drive.rightDrive(-1 * pow);
  wait(time, sec);
  drive.stop();
}

void greenTurnToTarget(bool right, double pow, double target) {
  double error = target - IMU.heading();

  if (error < 0) {
    error += 360;
  }

  double p = 1.4;

  double integral_error = 0;

  while (true) {
    error = target - IMU.heading();
    integral_error += error;
  
    if (error/50 >= 1 || integral_error >= 100) {
      pow = pow * 1 * p;
    } else {
      pow = pow * (error/50) * p;
    }

    if (right) {
      drive.leftDrive(pow);
      drive.rightDrive(-1 * pow);
    } else {
      drive.leftDrive(-1 * pow);
      drive.rightDrive(pow);
    }

    if (error < 3) {
      break;
    }

    wait(10, msec);
  }

  //stop bot
  drive.stop();
}


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

  for (int i = 0; i < 10; i++) {
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
      for(int j =0; j < 2; j++) {
        catapultLaunch();
        // these two lines here are what does the automatic arming of the catapult.
        waitUntil(getCatAccel() <= 0.1); // <-- might be blocking, which isnt desirable
        catapultArm();
      }

      wait(0.2, sec);

    } else {
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("not shooting");
      Controller1.Screen.print(colorSensor.hue());

      wait(0.5, sec);

      continue;
    }

    // drive back
    driveForwardTimed(-50, 0.1);
    wait(0.2, sec);
  }
}
void green_skills_auto() {
  catapultLower();
  wait(0.6, sec);
  catapultStop();

  // push ball in, try three times
  push_ball();

  if (!catInPosArmed()) {
    catapultArm();
  }
  if (!catInPosArmed()) {
    catapultArm();
  }
  // turn right to 55
  greenTurnToTarget(true, 35, 30);
  
  driveForwardTimed(40, 1);

  greenTurnToTarget(false, 50, 315);

  driveForwardTimed(40, 0.7);

  // intake and shoot
  intake_and_shoot();
}

void green_autonomous() {
  
  greenReleaseIntake();


  catapultLower();
  wait(0.6, sec);
  catapultStop();
}
void autonomous(void) {
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

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop

  // use this if we have calibration issues. make sure it prevents driver control for at least 2 seconds
  //IMU.calibrate(2000);

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    //drive.tankDrive(Controller1.Axis3.position(), Controller1.Axis2.position());

    // 1 stick arcade
    //drive.arcadeDrive(Controller1.Axis3.position(), Controller1.Axis4.position());

    // 2 stick arcade
    drive.arcadeDrive(Controller1.Axis3.position(), Controller1.Axis1.position());

    if (Controller1.ButtonRight.pressing()){
      ballKicker.spin(directionType::fwd, 100, percentUnits::pct);
    } else if (Controller1.ButtonLeft.pressing()){
      ballKicker.spin(directionType::rev, 100, percentUnits::pct);
    } else {
      ballKicker.stop(brakeType::brake);
    }

    //intake
    Controller1.ButtonL1.pressed([](){
      intakeSpin();
    });

    Controller1.ButtonL1.released([](){
      intakeStop();
    });

    Controller1.ButtonL2.pressed([](){
      intakeSpin(true);
    });

    Controller1.ButtonL2.released([](){
      intakeStop();
    });

    Controller1.ButtonA.pressed([](){
      wings.toggleWings();
    });

    // catapult
    Controller1.ButtonR1.pressed([](){
      catapultLaunch();
      // these three lines here are what does the automatic arming of the catapult.
      wait(50, msec);
      waitUntil(getCatAccel() <= 0.1); // <-- might be blocking, which isnt desirable
      catapultArm();
    });

    Controller1.ButtonR1.released([](){
      // catapultStop();
    });

    Controller1.ButtonR2.pressed([](){
      if (!catInPosArmed()) {
        catapultArm();
      }

      // catapultLower();
    });

    Controller1.ButtonR2.released([](){

    });
    
    Controller1.ButtonDown.pressed([](){
      catapultLower();
    });

    Controller1.ButtonDown.released([](){
      stopAutoArming();
      catapultStop();
    });

    Controller1.ButtonUp.pressed([](){
      catapultRaise();
    });

    Controller1.ButtonUp.released([](){
      stopAutoArming();
      catapultStop();
    });

    //the button formerly known as twitter
    Controller1.ButtonX.pressed([](){
      drive.toggleInvertedDrive();
    });


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//


int main() {
  pre_auton();
  
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  Controller1.Screen.clearScreen();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    updateCatAccel(0.02);
    odomUpdate();

    //Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(gpsHeadingRad());
    Controller1.Screen.setCursor(1,10);
    Controller1.Screen.print(Brain.Battery.capacity());
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print(getX());
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print(getY());
    Controller1.Screen.setCursor(2,12);
    //Controller1.Screen.print(drive.getAngleToPoint(0, 1000));
    Controller1.Screen.print(catapultRot.angle());

    wait(20, msec);
  }
}
