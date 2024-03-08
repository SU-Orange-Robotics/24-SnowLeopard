#include "vex.h"
#include "robot-config.h"
#include "odometry.h"
#include <math.h>
#include "utils/util.h"

using namespace vex;

/*
use this to detect when bot is going over the bar
imu.gyroRate(axisType::xaxis, velocityUnits::dps);
imu.gyroRate(axisType::yaxis, velocityUnits::dps);

will likely set up some sort of state machine to know when the bot transitions from on the bar to back on the ground
this makes it easy to know when to reset the "global" tracked position based on the gps and re-enable tracking wheels
^ proably not that important since we likely won't ever be in a situation where we want position tracking after crossing the bar


*/

double xPos;
double yPos;
double theta;
double gps_theta;
double imu_theta;

double odom_xPos;
double odom_yPos;
double odom_theta;
double xChange = 0;
double yChange = 0;

double encoderLeft = 0.0;
double encoderRight = 0.0;
double encoderCenter = 0.0;

void odomInit() {
    odom_xPos = gps1.xPosition(vex::distanceUnits::in);
    odom_yPos = gps1.yPosition(vex::distanceUnits::in);
    odom_theta = toRadians(gps1.heading(vex::rotationUnits::deg));
    gps_theta = toRadians(gps1.rotation(rotationUnits::deg));
    imu_theta = toRadians(imu.rotation(rotationUnits::deg));

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

    odom_theta += dt;



    //this is where we use the odometry to check if the gps' reported change in position is realistic

    double gpsdx = gps1.xPosition(distanceUnits::in) - xPos;
    double gpsdy = gps1.yPosition(distanceUnits::in) - yPos;

    // calculate vector magnitudes for gps and odom
    double gpsMag = hypot(gpsdx, gpsdy);
    double odomMag = hypot(xChange, yChange);

    // calulate angle difference between vectors
    double vectAngle = acos((gpsdx * xChange + gpsdy * yChange) / (gpsMag * odomMag));

    // compare vector for odom and gps. if the angle difference is small then approve the gps' updated position (overwrite old position)
    // also make sure the magnitude of the change is similar before approving new position
    if (vectAngle <= 0.1 && (gpsMag - odomMag <= 2)) { // radians, inches
        xPos = gps1.xPosition(distanceUnits::in);
        yPos = gps1.yPosition(distanceUnits::in);
    }

    //update the angle with gps unless it is not reporting change, then use imu;
    double imuChange = toRadians(imu.rotation(rotationUnits::deg)) - imu_theta;
    double gpsChange = toRadians(gps1.rotation(rotationUnits::deg)) - gps_theta;

    //if (gps1.quality() <= 5) {
    //    theta += imuChange;
    //} else {
        theta = toRadians(gps1.heading(rotationUnits::deg));
    //}

    imu_theta = toRadians(imu.rotation(rotationUnits::deg));
    gps_theta = toRadians(gps1.rotation(rotationUnits::deg));

}

double gpsHeadingRad() {
    double radHeading = (-1 * gps1.heading() * M_PI / 180);
    return radHeading += (radHeading < -1 * M_PI ? 2 * M_PI : 0);
}

double gpsAngleRad() {
    double radAngle = (-1 * gps1.rotation() * M_PI / 180);
    return radAngle += (radAngle < -1 * M_PI ? 2 * M_PI : 0);
}

double getX() {
    return xPos;
}

double getY() {
    return yPos;
}

double getTheta() {
    double thetaRad = (-1 * theta);
    return thetaRad += (theta < -1 * M_PI ? 2 * M_PI : 0);
}