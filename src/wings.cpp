#include "vex.h"
#include "robot-config.h"
#include "wings.h"

using namespace vex;

Wings::Wings() {
    wingState = false;
}

void Wings::initWings() {
    wingL.resetPosition();
    wingR.resetPosition();

    wingL.setStopping(brakeType::hold);
    wingR.setStopping(brakeType::hold);
}

void Wings::toggleWings() {
    if (wingState == false) {
    expandWings();
    } else {
    retractWings();
    }

    wingState = !wingState;
}

void Wings::expandWings() {
    wingL.spinToPosition(180, deg, false);
    wait(200, timeUnits::msec);
    wingR.spinToPosition(182, deg, false);

}

void Wings::retractWings() {
    wingR.spinToPosition(0, deg, false);
    wait(200, timeUnits::msec);
    wingL.spinToPosition(0, deg, false);
}