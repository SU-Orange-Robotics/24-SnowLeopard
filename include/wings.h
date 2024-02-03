
#include "vex.h"

#pragma once

class Wings {
private:
    bool wingState; // true for expanded, false for retracted

public:
    Wings();
    void initWings();
    void toggleWings();
    void expandWings();
    void retractWings();
};