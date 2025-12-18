#include "robot-config.h"


using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller controller_1 = controller(primary);

// IMPORTANT: Remember to modify the example motors according to the guide. 
// Also remember to add respective device declarations to custom/include/robot-config.h
// Format: motor(port, gearSetting, reversed)
// gearSetting is one of the following: ratio36_1(red), ratio18_1(green), ratio6_1(blue)
// all chassis motors should be reversed appropriately so that they spin vertical when given a positive voltage input
// such as driveChassis(12, 12)
motor left_chassis1 = motor(PORT13, ratio6_1, true);
motor left_chassis2 = motor(PORT12, ratio6_1, true);
motor left_chassis3 = motor(PORT11, ratio6_1, true);
motor_group left_chassis = motor_group(left_chassis1, left_chassis2, left_chassis3);
motor right_chassis1 = motor(PORT17, ratio6_1, false);
motor right_chassis2 = motor(PORT18, ratio6_1, false);
motor right_chassis3 = motor(PORT19, ratio6_1, false);
motor_group right_chassis = motor_group(right_chassis1, right_chassis2, right_chassis3);

inertial inertial_sensor = inertial(PORT2);
optical example_optical_sensor = optical(PORT10);
distance example_distance_sensor = distance(PORT14);
digital_out example_piston = digital_out(Brain.ThreeWirePort.A);

// Format is rotation(port, reversed)
// just set these to random ports if you don't use tracking wheels
rotation horizontal_tracker = rotation(PORT21, true);
rotation vertical_tracker = rotation(PORT21, false);

// game specific devices for high stakes
motor intake = motor(PORT16, ratio6_1, false);
motor hood = motor(PORT1, ratio6_1, true);
digital_out matchloader = digital_out(Brain.ThreeWirePort.C);
digital_out middleGoal = digital_out(Brain.ThreeWirePort.B);
digital_out leftWing = digital_out(Brain.ThreeWirePort.A);
digital_out middleGoalHood = digital_out(Brain.ThreeWirePort.B);
distance backDistanceSensor = distance(PORT8);
distance frontDistanceSensor = distance(PORT15);
distance leftDistanceSensor = distance(PORT3);
distance rightDistanceSensor = distance(PORT4);
optical ballSensTop = optical(14);



bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
}