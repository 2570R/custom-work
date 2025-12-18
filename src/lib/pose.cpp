#include "lib/pose.h"
#include "lib/utils.h"

Pose::Pose(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

Pose Pose::operator+(const Pose& other) const {
    return Pose(this->x + other.x, this->y + other.y, this->theta);
}

Pose Pose::operator-(const Pose& other) const {
    return Pose(this->x - other.x, this->y - other.y, this->theta);
}

float Pose::operator*(const Pose& other) const { return this->x * other.x + this->y * other.y; }

Pose Pose::operator*(const float& other) const {
    return Pose(this->x * other, this->y * other, this->theta);
}

Pose Pose::operator/(const float& other) const {
    return Pose(this->x / other, this->y / other, this->theta);
}

Pose Pose::lerp(Pose other, float t) const {
    return Pose(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->theta);
}

float Pose::distance(Pose other) const { return hypot(this->x - other.x, this->y - other.y); }

float Pose::angle(Pose other) const { return atan2(other.y - this->y, other.x - this->x); }

Pose Pose::rotate(float angle) const {
    return Pose(this->x * cos(angle) - this->y * sin(angle),
                        this->x * sin(angle) + this->y * cos(angle), this->theta);
}

float Pose::face(Pose other, bool rad) {
    //returns the angle one pose must turn to face the other pose
  
    float theta = rollAngle180(RTD(atan2(other.x - this->x, other.y - this->y)) - this->theta);
    return (rad) ? DTR(theta) : theta;
  }

float Pose::parallel(Pose other) {
    //returns the angle one pose must turn to be parallel with another pose
  
    return rollAngle180(other.theta - this->theta);
}

bool hc(Pose pose, Pose target, float theta, float tolerance) {
    return (pose.y - target.y) * -cos(DTR(theta)) >= sin(DTR(theta)) * (pose.x - target.x) + tolerance;
}
