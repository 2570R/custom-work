#pragma once

#include "vex.h"
#include "robot-config.h"
#include "pose.h"
#include <cmath>

using namespace vex;

// Angle utility functions
double rollAngle180(double angle);
double rollAngle360(double angle);
double DTR(double deg);  // Degrees to Radians
double RTD(double rad);  // Radians to Degrees (alias for RTG)

// Chassis encoder functions
double leftDTTravelDeg();
double rightDTTravelDeg(bool in = false);

// Chassis control functions
void resetChassis();
void stopChassis(brakeType type);
void moveChassisRaw(double leftV, double rightV);

// Sensor functions
double getInertialReading(bool deg);

// Math utility functions
double ema(double current, double previous, double smooth);
float clamp(float input, float min, float max);


// Sign function (returns -1, 0, or 1)
template<typename T>
int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

