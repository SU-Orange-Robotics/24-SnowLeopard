#include "vex.h"
#include <math.h>

#pragma once

class Drive {
        
    private:
        const double deadzone = 10; // from 0 to 100
        const double tankForwardDeadzone = 20;
        const double maxOutputPct = 100; // limits maximum motor output percentage

        double originHeading;
        double originCorr;
        bool invertDrive;

        static double maxClamp(double input, double max) {
            return (fabs(input) <= max ? input : max * (input / fabs(input)));
        }

        void inputAdjust(double &fwd, double &str);


        bool activePID;

        const double a_P = 45;  //50
        const double a_I = 0;   //0
        const double a_D = 0.5; //0.3
        vex::timer pid_timer;

        const double d_P = 1.0;
        const double d_D = 3.0;
        const double d_I = 0.00;
        vex::timer pid_timer2;

        double getAngleErrorOLD(double target);

        double getAngleError(double target);

        

        double getDistanceError(double targetX, double targetY, int count);

    public:
        Drive();

        void arcadeDrive(double y, double x);

        void tankDrive(double left, double right);

        void toggleInvertedDrive();

        void resetHeading();

        void stop();

        bool getInvertedDrive();

        // make this private at some point, it is only public for debugging purposes on the controller display
        double getAngleToPoint(double x2, double y2);


        void driveForward(double fwd);

        void adjustCCW(double speed);

        void turnPID(double targetHeading);

        void goToPointPID(double targetX, double targetY);

        void turnToPoint(double targetX, double targetY, bool flipped = false);

        void turnAndDrivePID(double targetX, double targetY);

        void leftDrive(double pow);
        void rightDrive(double pow);

        bool pidActive() {
            return activePID;
        }

};
