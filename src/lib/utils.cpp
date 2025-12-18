#include "lib/utils.h"


double rollAngle180(double angle) {
    while(!(angle >= -180 && angle < 180)) {
      if( angle < -180 ) { angle += 360; }
      if(angle >= 180) { angle -= 360; }
    }
    return(angle);
}

double rollAngle360(double angle) {
    while(!(angle >= 0 && angle < 360)) {
      if( angle < 0 ) { angle += 360; }
      if(angle >= 360) { angle -= 360; }
    }
    return(angle);
}

double DTR(double deg) {
    return (deg * M_PI / 180);
}
  
double RTD(double rad) {
    return (rad * 180 / M_PI);
}

double leftDTTravelDeg() {
    return left_chassis.position(degrees);
}
  
 
double rightDTTravelDeg(bool in) {
    return right_chassis.position(degrees);
}

void resetChassis() {
    // Set both chassis motor encoders to zero
    left_chassis.setPosition(0, degrees);
    right_chassis.setPosition(0, degrees);
}

void stopChassis(brakeType type){
    left_chassis.setStopping(type);
    right_chassis.setStopping(type);
}

void moveChassisRaw(double leftV, double rightV){
    left_chassis.spin(fwd, leftV, volt);
    right_chassis.spin(fwd, rightV, volt);
}

double getInertialReading(bool deg){
    if(deg){
        return rollAngle360(inertial_sensor.rotation(vex::deg) * (3600/3558));
    } else{
        return DTR(rollAngle360(inertial_sensor.rotation(vex::deg) * (3600/3558)));
    }
}

double ema(double current, double previous, double smooth) {
    return (current * smooth) + (previous * (1 - smooth));
}

float clamp(float input, float min, float max){
    if( input > max ){ return(max); }
    if(input < min){ return(min); }
    return(input);
  }
  


