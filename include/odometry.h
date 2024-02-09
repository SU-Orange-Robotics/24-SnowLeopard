#include "vex.h"

#pragma once

const double width = 4.25;
const double length = 11.75;
const double circ1 = 4.1875 * M_PI;
const double circ2 = 3.375 * M_PI; //this is for the center wheel

void odomInit();

void odomUpdate();

double gpsHeadingRad();

double gpsAngleRad();

double getX();

double getY();

double getTheta();