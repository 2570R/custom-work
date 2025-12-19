#pragma once

#include "vex.h"
#include "utils.h"
#include "robot-config.h"
#include "pose.h"
#include <vector>

using namespace vex;

// Global odometry variables (defined in src/lib/odom.cpp)
extern Pose odomPose;
extern Pose odomSpeed;

// Function declarations
Pose getPose();
void setPose(double x, double y, double theta = 0.0);
void update();
float getSpeed();

