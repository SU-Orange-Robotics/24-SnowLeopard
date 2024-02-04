#include "vex.h"

#pragma once

const double width = 0; // forward offset of the center wheel
const double length = 0; // lateral disance between the wheels, more like "width" of the bot
const double circ1 = 0;
const double circ2 = 0; //this is for the center wheel

void odomInit();

void odomUpdate();

double getX();

double getY();

double getTheta();