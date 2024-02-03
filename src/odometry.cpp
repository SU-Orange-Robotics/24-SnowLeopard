#include "vex.h"
#include "robot-config.h"
#include "odometry.h"
#include <math.h>
#include "utils/util.h"

using namespace vex;

/*
use this to detect when bot is going over the bar
IMU.gyroRate(axisType::xaxis, velocityUnits::dps);
IMU.gyroRate(axisType::yaxis, velocityUnits::dps);

will likely set up some sort of state machine to know when the bot transitions from on the bar to back on the ground
this makes it easy to know when to reset the "global" tracked position based on the gps and re-enable tracking wheels
^ proably not that important since we likely won't ever be in a situation where we want position tracking after crossing the bar


*/

double xPos;
double yPos;
double theta;

double odom_xPos;
double odom_yPos;
double odom_theta;
double xChange = 0;
double yChange = 0;
double tChange = 0;

double encoderLeft = 0.0;
double encoderRight = 0.0;
double encoderCenter = 0.0;

void odomInit() {
    odom_xPos = gps1.xPosition(vex::distanceUnits::in);
    odom_yPos = gps1.yPosition(vex::distanceUnits::in);
    odom_theta = toRadians(gps1.heading(vex::rotationUnits::deg));

    odomLeft.resetPosition();
    odomRight.resetPosition();
    odomCenter.resetPosition();
}

void odomUpdate() {
    double newEncoderLeft = odomLeft.position(turns);
    double newEncoderRight = odomRight.position(turns);
    double newEncoderCenter = odomCenter.position(turns);

    double diffLeft = newEncoderLeft - encoderLeft;
    double diffRight = newEncoderRight - encoderRight;
    double diffCenter = newEncoderCenter - encoderCenter;
  
    // overwrite old values
    encoderLeft = newEncoderLeft;
    encoderRight = newEncoderRight;
    encoderCenter = newEncoderCenter;
  
    // 2. calculate delta u
  
    // 3. calculate delta x,y,theta
    double dx = circ1 * (diffRight + diffLeft) / 2;
    double dy = circ1 * (diffCenter - (diffRight - diffLeft) * (width / length));
    double dt = circ2 * (diffRight - diffLeft) / length;
  
    // 4. use trig to calculate new location x,y,theta
    double newTheta = odom_theta + (dt / 2.0);
    
    xChange = dx * cos(newTheta) - dy * sin(newTheta);
    odom_xPos += xChange;

    yChange = dx * sin(newTheta) + dy * cos(newTheta);
    odom_yPos += yChange;

    tChange = dt;
    odom_theta += dt;
}

//this is where we use the odometry to check if the gps' reported change in position is realistic
void updatePos() {
    //get gps values
    double gpsdx = 0.0;
    double gpsdy = 0.0;
    double gpsdt = 0.0;   

    // calculate vector magnitudes for gps and odom
    double gpsMag = 0.0;
    double odomMag = 0.0;

    // calulate angle difference between vectors
    double vectAngle = acos((gpsdx * xChange + gpsdy * yChange) / (gpsMag * odomMag));

    // compare vector for odom and gps. if the angle difference is small then approve the gps' updated position (overwrite old position)
    // also make sure the magnitude of the change is similar before approving new position

    // if the angle difference is big, leave the position as is (or change is based on odometry?)
}

double getX() {
    return xPos;
}

double getY() {
    return yPos;
}

double getTheta() {
    return theta;
}