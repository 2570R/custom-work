#pragma once

#include "vex.h"
using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller controller_1;
extern motor left_chassis1;
extern motor left_chassis2;
extern motor left_chassis3;
extern motor_group left_chassis;
extern motor right_chassis1;
extern motor right_chassis2;
extern motor right_chassis3;
extern motor_group right_chassis;
extern inertial inertial_sensor;
extern rotation vertical_tracker;
extern rotation horizontal_tracker;
extern motor intake;
extern motor hood;
extern digital_out leftWing;
extern digital_out middleGoal;
extern digital_out matchloader;
extern optical ballSensTop;
extern distance frontDistanceSensor;
extern distance leftDistanceSensor;
extern distance rightDistanceSensor;
