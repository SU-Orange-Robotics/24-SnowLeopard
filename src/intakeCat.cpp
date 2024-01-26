#include "vex.h"
#include "robot-config.h"
#include "intakeCat.h"

using namespace vex;

bool autoArming = false;

void intakeSpin(bool reversed) {
    intake.spin(directionType::fwd, intakePow * (reversed ? -1 : 1), percentUnits::pct);
}

void intakeStop() {
    intake.stop();
}

void catapultLower() {
    catapultA.spin(directionType::fwd, catPow, percentUnits::pct);
    catapultB.spin(directionType::fwd, catPow, percentUnits::pct);
    catapultC.spin(directionType::fwd, catPow, percentUnits::pct);
}

void catapultStop() {
    catapultA.stop();
    catapultB.stop();
    catapultC.stop();
}

void catapultArm() {
    autoArming = true;
    catapultLower();
    waitUntil(catInPosArmed() || !autoArming);
    catapultStop();
    autoArming = false;
}

void catapultLaunch() {
    autoArming = true;
    catapultLower();
    //waitUntil(catInPosShoot() || !autoArming);
    catShooted(); //will block execution until it returns a value, which means the cat has finished shooting
    catapultStop();
}

//this didn't work, too much resistance from the motors in coast mode
void catapultLaunch2() {
    //catapultA.setStopping(brakeType::coast);
    //catapultB.setStopping(brakeType::coast);
    //catapultC.setStopping(brakeType::coast);

    double helpPow = 100;
    catapultA.spin(directionType::rev, helpPow, percentUnits::pct);
    catapultB.spin(directionType::rev, helpPow, percentUnits::pct);
    catapultC.spin(directionType::rev, helpPow, percentUnits::pct);

    wait(200, timeUnits::msec);
    waitUntil(catShooted());

    catapultStop();
    //catapultA.setStopping(brakeType::hold);
    //catapultB.setStopping(brakeType::hold);
    //catapultC.setStopping(brakeType::hold);
}

bool catInPosArmed() {
    double posA1 = 197, posA2 = 200;//197.57, 197.75

    double currPos = catapultRot.angle(rotationUnits::deg);
    //modf(360, &currPos);

    return ((currPos > posA1 && currPos < posA2) /*|| (currPos > posB1 && currPos < posB2)*/);
}

bool catInPosShoot() {
    double posA1 = 248, posA2 = 255;

    double currPos = catapultRot.angle(rotationUnits::deg);
    //modf(360, &currPos);

    return (currPos > posA1 && currPos < posA2);
}

bool catShooted() {
    double velo;
    double lastVelo = catapultRot.velocity(velocityUnits::dps);
    double accl = 100;

    while (accl > 2) {
        wait(20, timeUnits::msec);
        velo = catapultRot.velocity(velocityUnits::dps);
        accl = (velo - lastVelo) / 0.02;
        lastVelo = velo;
    }

    return true;
}

void stopAutoArming() {
    autoArming = false;
}