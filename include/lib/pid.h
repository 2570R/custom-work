#pragma once
#include "vex.h"
class PID {
public:
    // Constructor
    PID(float kP, float kI, float kD, float windupRange, bool signFlipReset);
    
    // Methods
    float update(const float error);
    void reset();

private:
    float kP;
    float kI;
    float kD;
    float windupRange;
    bool signFlipReset;
    float integral = 0;
    float prevError = 0;
};

