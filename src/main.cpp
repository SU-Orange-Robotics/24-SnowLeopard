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

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  // head to head auto
  /*
  drive.driveForward(100);
  wait(300, msec);
  drive.stop();
  catapultArm();
  drive.driveForward(-100);
  wait(1200, msec);
  drive.stop();

  */
  
  // turning testing auto
  /*
  drive.turnPID(0);

  wait(1, sec);

  drive.turnPID(M_PI);

  wait(1, sec);

  drive.turnPID(M_PI / 2);

  wait(1, sec);

  drive.turnPID(5 * M_PI / 6);
  */
  
  // auto skills code

  drive.driveForward(100);
  wait(300, msec);
  drive.stop();
  catapultArm();
  drive.driveForward(-100);
  wait(600, msec); //tune
  drive.stop();
  drive.turnPID(M_PI / -2);
  drive.driveForward(100);
  wait(1000, msec); //tune
  drive.stop();

  drive.turnPID(0 - 0.3);

  drive.driveForward(-100);
  wait(300, msec); //tune
  drive.stop();

  drive.turnPID((-1 * M_PI / 4) + 0.1);
  drive.driveForward(100);
  wait(1400, msec); // tune
  drive.stop();

  intakeSpin(true);
  wait(300, msec);

  int i;
  for (i = 0; i < 10; i++) {
    //reverse away from bar for match load
    drive.driveForward(-100);
    wait(400, msec); //tune
    drive.stop();

    catapultLaunch();
    waitUntil(getCatAccel() <= 0.05);
    catapultArm();

    // give time for match load to be loaded (in addition to catapult arm time) and allow for 
    drive.turnPID((-1 * M_PI / 4) + 0.03);
    //wait(500, msec);

    //drive forward into bar
    drive.driveForward(100);
    wait(650, msec); //tune
    drive.stop();

    //give time for ball to get into catapult
    wait(600, msec); // tune      or replace with color sensor
  }
  catapultLaunch();
  intakeStop();



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

    Controller1.ButtonX.pressed([](){
      drive.toggleInvertedDrive();
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
      //catapultStop();
    });

    Controller1.ButtonR2.pressed([](){
      if (!catInPosArmed()) {
        catapultArm();
      }
    });

    Controller1.ButtonR2.released([](){
      //catapultStop();
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


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//


int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    updateCatAccel(0.02);
    odomUpdate();

    //Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(gpsHeadingRad());
    Controller1.Screen.setCursor(1,10);
    Controller1.Screen.print(Brain.Battery.capacity()); //gpsAngleRad()
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print(getX());
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print(getY());
    Controller1.Screen.setCursor(2,12);
    //Controller1.Screen.print(drive.getAngleToPoint(0, 1000));
    Controller1.Screen.print(catapultRot.angle(rotationUnits::deg));
    Controller1.Screen.setCursor(3, 12);
    Controller1.Screen.print(drive.getInvertedDrive());

    wait(20, msec);
  }
}
