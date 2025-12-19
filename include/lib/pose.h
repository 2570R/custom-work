#pragma once

#include <string>
#include <cmath>
#include "vex.h"
#include "utils.h"

class Pose {
public:
    float x;
    float y;
    float theta;
    
    // Constructor
    Pose(float x, float y, float theta = 0);
    
    // Operators
    Pose operator+(const Pose& other) const;
    Pose operator-(const Pose& other) const;
    float operator*(const Pose& other) const;
    Pose operator*(const float& other) const;
    Pose operator/(const float& other) const;
    
    // Utility methods
    Pose lerp(Pose other, float t) const;
    float distance(Pose other) const;
    float angle(Pose other) const;
    Pose rotate(float angle) const;
    float face(Pose other, bool rad = false);
    float parallel(float target);
    float parallel(Pose other);
};

bool hc(Pose pose, Pose target, float theta, float tolerance);