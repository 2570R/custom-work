#include "../include/auton.h"
#include "../include/intake.h"
#include "../../include/lib/chassis.h"
#include "../../include/lib/odom.h"
#include "../../include/robot-config.h"
#include "vex.h"
#define EXTEND_MATCHLOADER matchloader.set(true);
#define RETRACT_MATCHLOADER matchloader.set(false);

void antijamtaskfn(){
    vex::task antiJamF([]{
        while(true){
            antiJamTask();
            wait(20, msec);
        }
        return 0;
    });
}

void pushMatchloader(){
    moveChassisRaw(6,6);
    vex::wait(650, msec);
    moveChassisRaw(2,2);
    vex::wait(250, msec);
}
void left7LongandWing(){}
void right7LongandWing(){}
void leftLongAndMid(){}
void leftLongAndMidDisrupt(){}
void rightLongAndLow(){}
void test(){}
void left4(){}
void rifour(){}

void awp(){
    stopChassis(coast);
    antijamtaskfn();
    //points
    Pose approachMatchloader(0.2,-30, 0);
    Pose lineUpFirstGoal(-42,-54, 1);
    Pose firstStack(-30, -35, 1);
    Pose midWayPoint(-31, -11, 1);
    Pose secondStack(-32, 17.5, 1);
    Pose middleGoalPos(-16, -2, -40);

    //--
    setPose(0,0, 180);
    storeIntake();
    EXTEND_MATCHLOADER
    mtp(approachMatchloader, true, 100, true, 0, 20, 1200);
    tth(-90, 100, true, 0, 750);
    pushMatchloader();
    setPose(-72 + frontDistanceSensor.value() / 25.4, -72 + leftDistanceSensor.value() / 25.4, getInertialReading(true));
    vex::wait(10, msec);
    mtp(lineUpFirstGoal, false, 100, false, 1, 50, 1200);
    move(-10, 80, 90, true, 0, 500);
    moveChassisRaw(-1, -1);
    scoreLongGoal();
    vex::wait(0.6, sec);
    RETRACT_MATCHLOADER
    tth(0, 127, true, 0, 1500);
    setPose(72 - leftDistanceSensor.value() / 25.4, getPose().y, getInertialReading(true));
    storeIntake();
    vex::wait(10, msec);
    moveChassisRaw(12, 12);
    vex::wait(50, msec);
    matchloader.set(true);
    mtp(firstStack, true, 100, false, 1, 60, 1100);
    vex::task offm([]{
        vex::wait(200, msec);
        matchloader.set(false);
        return 0;
      });
    mtp(midWayPoint, true, 100, false, 1, 60, 1000);
    mtp(secondStack, true, 100, true, 1, 60, 1000);
    matchloader.set(true);
    vex::task readyMiddle([]{
        vex::wait(300, msec);
        outtake();
        vex::wait(150, msec);
        stopIntake();
        middleGoal.set(true);
        return 0;
      });
    mtpose(middleGoalPos, false, 0.5, 0, 2, 100, false, 1, 20, 1200);
    moveChassisRaw(-1, -1);
    scoreMiddleGoal();
    vex::wait(800, msec);
    
    









}