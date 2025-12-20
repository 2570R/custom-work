#include "lib/odom.h"

Pose odomPose(0,0,0);
Pose odomSpeed(0,0,0);
std::vector<Pose> positions;
std::vector<float> angles;

Pose getPose(){
    return odomPose;
}

void setPose(double x, double y, double theta){
    odomPose.x = x;
    odomPose.y = y;
    odomPose.theta = theta;
}

void updateVelocities(){
    positions.push_back(Pose(odomPose.x, odomPose.y));
    if (positions.size() > 4) positions.erase(positions.begin());
    angles.push_back(inertial_sensor.rotation(deg));
    if (angles.size() > 4) angles.erase(angles.begin());
}

float getVelocityX() {
    //returns approximated velocity on the horizontal axis in units per second

    return (!positions.empty()) ? (positions.back().x - positions.front().x) * (100.0 / positions.size()) : 0;
}


float getVelocityY() {
    //returns approximated velocity on the vertical axis in units per second

    return (!positions.empty()) ? (positions.back().y - positions.front().y) * (100.0 / positions.size()) : 0;
}

float getSpeed() {

    return sqrt(pow(getVelocityX(), 2) + pow(getVelocityY(), 2));
}
void update(){
    //---
    resetChassis();
    double prev_heading_rad = 0;
    double prev_left_deg = 0, prev_right_deg = 0;
    double delta_local_y_in = 0;
    double heading_rad = 0;
    double x_in;
    double y_in;
    double wheel_distance_in = 3.17;
    double distance_between_wheels = 11;
    double prev_x;
    double prev_y;
    //---
    while (1) {

        //get curs
        heading_rad = getInertialReading(false);
        double left_deg = leftDTTravelDeg();
        double right_deg = rightDTTravelDeg();

        //calcs
        double delta_heading_rad = heading_rad - prev_heading_rad; // Change in heading (radians)
        double delta_left_in = (left_deg - prev_left_deg) * wheel_distance_in / 360.0;   // Left wheel delta (inches)
        double delta_right_in = (right_deg - prev_right_deg) * wheel_distance_in / 360.0; // Right wheel delta (inches)
        
        if (fabs(delta_heading_rad) < 1e-6) {
        delta_local_y_in = (delta_left_in + delta_right_in) / 2.0;
        } else {
        
        double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
        double delta_local_y_left_in = sin_multiplier * (delta_left_in / delta_heading_rad + distance_between_wheels / 2.0);
        double delta_local_y_right_in = sin_multiplier * (delta_right_in / delta_heading_rad + distance_between_wheels / 2.0);
        delta_local_y_in = (delta_local_y_left_in + delta_local_y_right_in) / 2.0;
        }
        
        double polar_angle_rad = prev_heading_rad + delta_heading_rad / 2.0;
        double polar_radius_in = delta_local_y_in;

        x_in += polar_radius_in * sin(polar_angle_rad);
        y_in += polar_radius_in * cos(polar_angle_rad);
        
        odomPose.x = x_in;
        odomPose.y = y_in;
        odomPose.theta = getInertialReading(true);
        
        prev_heading_rad = heading_rad;
        prev_left_deg = left_deg;
        prev_right_deg = right_deg;
        prev_x = x_in;
        prev_y = y_in;

        updateVelocities();

        // calculate speed
        odomSpeed.x = ema((odomPose.x - prev_x) / 0.01, odomSpeed.x, 0.95);
        odomSpeed.y = ema((odomPose.y - prev_y) / 0.01, odomSpeed.y, 0.95);
        odomSpeed.theta = ema((odomPose.theta - RTD(prev_heading_rad)) / 0.01, odomSpeed.theta, 0.95);

        wait(10, msec);
    }

}