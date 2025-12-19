#pragma once

#include "odom.h"
#include "path.h"
#include "logger.h"
#include "pid.h"
#include "utils.h"
#include "pose.h"
#include <utility>
#include <vector>

// Global variables (defined in src/lib/chassis.cpp)
extern double wheel_distance_in;
extern bool is_moving;
extern Logger logger;
// PID controllers (inline to avoid multiple definition)
// Constructor: PID(float kP, float kI, float kD, float windupRange, bool signFlipReset)
inline PID headingPID(1.0, 0.0, 6.0, 1.0, true);
inline PID drivePID(10.0, 0.0, 35.0, 1.0, true);
inline PID turnPID(2.3, 0.0, 15.0, 1.0, true);
inline PID swingPID(2.0, 0.0, 15.0, 1.0, true);
inline PID angularPID(2.3, 0.0, 15.0, 1.0, true);
inline double pursuitVeloConst = 0.3;
inline double pursuitCurvConst = 0.4;
inline double pursuitCTEConst = 0;
inline double minLookahead = 1;
inline double maxLookahead = 1;
inline double radiusLookahead = 5;
inline double trackwidth = 11;

// Enum for drive side
enum class driveSide {
    LEFT,
    RIGHT
};

// Movement declarations

void move(double target, double maxSpeed, double targetHeading, bool exit, double exitDistRange, double timeout);
void move_using_dist(vex::distance* dist, double target, double maxSpeed, bool exit, double exitDistRange, double timeout);
void tth(double angle, double maxSpeed, bool exit, double exitAngleRange, double timeout);
void ttp(Pose target, double maxSpeed, bool exit, double exitAngleRange, double timeout);
void swing(double angle, driveSide drive, double maxSpeed, double oppositeSpeed, bool exit, double exitAngleRange, double timeout);
void mtp(Pose target, bool forwards, double maxSpeed, bool exit, double exitDistRange, double minSpeed, double timeout);
void mtpose(Pose target, bool forwards, double dLead, double gLead, double chasePower, double maxSpeed, bool exit, double exitDistRange, double minSpeed, double timeout);
void followPursuit(Path path,float timeout, float chasePower, bool reverse, bool exit);
void followStanley(Path path, float timeout, float k, float chasePower);
void driveLock(bool release);
void mirrorMoveDrivetrain(double left, double right, bool reverse);
