#include "lib/chassis.h"

double wheel_distance_in = 3.17;
bool is_moving = false;

Logger logger(std::cout, Logger::Level::DEBUG);
std::pair<float, float> reduceRatio(float max, float num1, float num2) {
    //reduces a ratio between two numbers so that the absolute value of the larger number is equal to max
  
    if (fabs(num1) > fabs(max) && fabs(num1) >= fabs(num2)) {
      num2 *= max / fabs(num1);
      num1 = copysign(1.0, num1) * max;
    }
    else if (fabs(num2) > fabs(max) && fabs(num2) >= fabs(num1)) {
      num1 *= max / fabs(num2);
      num2 = copysign(1.0, num2) * max;
    }
  
    std::pair<float, float> nums = std::make_pair(num1, num2);
    return nums;
}

float getRadius(Pose p1, Pose p2) {
    //finds the radius of the circle that intersects two points given a specified theta at p1
  
    return p1.distance(p2) / (sqrt(2.0 - 2.0 * cos(2.0 * p1.face(p2, true))));
}

float getCurvature(Pose p1, Pose p2) {
    //finds the curvature between two points given a specified theta at p1
    
    float side = copysign(1.0, cos(DTR(p1.theta)) * (p2.x - p1.x) - sin(DTR(p1.theta)) * (p2.y - p1.y));
    float radius = getRadius(p1, p2);
    return (radius != 0) ? side / radius : std::numeric_limits<float>::max();
}
float getCurvatureHeading(Pose p1, Pose p2, float heading) {
    //finds the curvature between two points given a specified theta
  
    float side = copysign(1.0, cos(DTR(heading)) * (p2.x - p1.x) - sin(DTR(heading)) * (p2.y - p1.y));
    float radius = getRadius(p1, p2);
    return (radius != 0) ? side / radius : std::numeric_limits<float>::max();
}

std::pair<float, float> scaleToRatio(float factor, float num1, float num2) {

    float ratio = factor / std::fmax(fabs(num1), fabs(num2));
    return std::make_pair(num1 * ratio, num2 * ratio);
}

void mirrorMoveDrivetrain(double left, double right, bool reverse){
    if (reverse) {
        left_chassis.spin(vex::fwd, -right, vex::voltageUnits::volt);
        right_chassis.spin(vex::fwd, -left, vex::voltageUnits::volt);
    }
        
    else {
        left_chassis.spin(vex::fwd, left, vex::voltageUnits::mV); 
        right_chassis.spin(vex::fwd, right, vex::voltageUnits::mV);
    }
}


void move(double target, double maxSpeed, double targetHeading, bool exit, double exitDistRange, double timeout){
    is_moving = true;
    stopChassis(vex::brakeType::coast);
    //grep currents
    double static_left_deg = leftDTTravelDeg();
    double static_right_deg = rightDTTravelDeg();
    double static_heading = rollAngle180(getInertialReading(true));
    double error;

    //drive direction
    int drive_direction = target > 0 ? 1 : -1;
    target = target * drive_direction;

    //reset PIDS
    drivePID.reset();
    headingPID.reset();
    double left_output, right_output, heading_output;
    auto time = Brain.Timer.time();
    
    Brain.Screen.clearScreen(black);
    while((Brain.Timer.time() - time) < timeout){

        //exit conditions
        if(exit && fabs(error) < 1){
            break;
        }
        if(!exit && fabs(error) < exitDistRange){
            break;
        }
        
        double current_distance = (fabs(((leftDTTravelDeg() - static_left_deg) / 360.0) * wheel_distance_in) + fabs(((rightDTTravelDeg() - static_right_deg) / 360.0) * wheel_distance_in)) / 2;
        double current_heading_deg = getInertialReading(true);
        double error = target - current_distance;

        if(target < 0){
            left_output = drivePID.update(error) * -1;
        }
        else left_output = drivePID.update(error) * 1;
        right_output = left_output;

        heading_output = headingPID.update(rollAngle180(targetHeading - current_heading_deg));

        //clip speeds
        left_output = clamp(left_output, -maxSpeed, maxSpeed);
        right_output = clamp(right_output, -maxSpeed, maxSpeed);

        //set speeds
        moveChassisRaw(left_output - heading_output, right_output + heading_output);

        wait(10, msec);    

    }
    logger.colorize("settled...", Logger::Level::INFO);
    logger.info("end x: %.2f, y: %.2f, z: %.2f", getPose().x, getPose().y, getPose().theta);
    //settle screen
    Brain.Screen.clearScreen(red);
    if(exit){
        moveChassisRaw(0,0);
    }
    is_moving = false;
}

void move_using_dist(vex::distance* dist, double target, double maxSpeed, bool exit, double exitDistRange, double timeout){
    is_moving = true;
    stopChassis(vex::brakeType::coast);
    //grep currents
    double curr_in = dist->value() / 25.4;
    double static_heading = rollAngle180(getInertialReading(true));
    double error;
    int drive_direction;
    //drive direction
    if(target > curr_in){
        drive_direction = -1;
    } else{
        drive_direction = 1;
    }

    //reset PIDS
    drivePID.reset();
    headingPID.reset();
    double left_output, right_output, heading_output;
    auto time = Brain.Timer.time();
    
    Brain.Screen.clearScreen(black);
    while((Brain.Timer.time() - time) < timeout){

        //exit conditions
        if(exit && fabs(error) < 1){
            break;
        }
        if(!exit && fabs(error) < exitDistRange){
            break;
        }
        
        double current_distance = dist->value() / 25.4;
        double current_heading_deg = getInertialReading(true);
        double error = current_distance;

        left_output = drivePID.update(error) * drive_direction;
        right_output = left_output;

        heading_output = headingPID.update(rollAngle180(getInertialReading(true) - current_heading_deg));

        //clip speeds
        left_output = clamp(left_output, -maxSpeed, maxSpeed);
        right_output = clamp(right_output, -maxSpeed, maxSpeed);

        //set speeds
        moveChassisRaw(left_output - heading_output, right_output + heading_output);

        wait(10, msec);    

    }
    logger.colorize("settled...", Logger::Level::INFO);
    logger.info("end x: %.2f, y: %.2f, z: %.2f", getPose().x, getPose().y, getPose().theta);
    //settle screen
    Brain.Screen.clearScreen(red);
    if(exit){
        moveChassisRaw(0,0);
    }
    is_moving = false;
}

void tth(double angle, double maxSpeed, bool exit, double exitAngleRange, double timeout){
    is_moving = true;
    stopChassis(vex::brakeType::coast);
    //grep currents
    angle = rollAngle180(angle);
    double static_heading = getInertialReading(true);
    double error;

    //reset PIDS
    turnPID.reset();
    double output;

    auto time = Brain.Timer.time();
    
    Brain.Screen.clearScreen(black);
    while((Brain.Timer.time() - time) < timeout){

        //exit conditions
        if(exit && fabs(error) < 1){
            break;
        }
        if(!exit && fabs(error) < exitAngleRange){
            break;
        }

        double current_heading_deg = getInertialReading(true);
        double error = angle - current_heading_deg;

        output = turnPID.update(error);

        //clip speeds
        output = clamp(output, -maxSpeed, maxSpeed);

        //set speeds
        moveChassisRaw(-output, output);

        wait(10, msec);
    }
    logger.colorize("settled...", Logger::Level::INFO);
    logger.info("end x: %.2f, y: %.2f, z: %.2f", getPose().x, getPose().y, getPose().theta);
    //settle screen
    Brain.Screen.clearScreen(red);
    if(exit){
        moveChassisRaw(0,0);
    }
    is_moving = false;
}

void ttp(Pose target, double maxSpeed, bool exit, double exitAngleRange, double timeout){
    is_moving = true;
    stopChassis(vex::brakeType::coast);
    //grep currents
    double angle = rollAngle180(getPose().angle(target));
    double static_heading = getInertialReading(true);
    double error;

    //reset PIDS
    turnPID.reset();
    double output;

    auto time = Brain.Timer.time();
    
    Brain.Screen.clearScreen(black);
    while((Brain.Timer.time() - time) < timeout){

        //exit conditions
        if(exit && fabs(error) < 1){
            break;
        }
        if(!exit && fabs(error) < exitAngleRange){
            break;
        }

        double current_heading_deg = getInertialReading(true);
        double error = angle - current_heading_deg;

        output = turnPID.update(error);

        //clip speeds
        output = clamp(output, -maxSpeed, maxSpeed);

        //set speeds
        moveChassisRaw(-output, output);

        wait(10, msec);
    }
    logger.colorize("settled...", Logger::Level::INFO);
    logger.info("end x: %.2f, y: %.2f, z: %.2f", getPose().x, getPose().y, getPose().theta);
    //settle screen
    Brain.Screen.clearScreen(red);
    if(exit){
        moveChassisRaw(0,0);
    }
    is_moving = false;
}

void swing(double angle, driveSide drive, double maxSpeed, double oppositeSpeed, bool exit, double exitAngleRange, double timeout){
    is_moving = true;
    stopChassis(vex::brakeType::coast);
    //grep currents
    angle = rollAngle180(angle);
    double static_heading = getInertialReading(true);
    double error;

    //reset PIDS
    swingPID.reset();
    double output;

    auto time = Brain.Timer.time();
    
    Brain.Screen.clearScreen(black);
    while((Brain.Timer.time() - time) < timeout){
        //exit conditions
        if(exit && fabs(error) < 1){
            break;
        }
        if(!exit && fabs(error) < exitAngleRange){
            break;
        }

        double current_heading_deg = getInertialReading(true);
        double error = angle - current_heading_deg;

        output = swingPID.update(error);

        //clip speeds
        output = clamp(output, -maxSpeed, maxSpeed);

        //set speeds
        if(drive == driveSide::LEFT){
            moveChassisRaw(oppositeSpeed, output);
        } else{
            moveChassisRaw(output, oppositeSpeed);
        }

        wait(10, msec);
    }
    logger.colorize("settled...", Logger::Level::INFO);
    logger.info("end x: %.2f, y: %.2f, z: %.2f", getPose().x, getPose().y, getPose().theta);
    //settle screen
    Brain.Screen.clearScreen(red);
    if(exit){
        moveChassisRaw(0,0);
    }
    is_moving = false;
}

void mtp(Pose target, bool forwards, double maxSpeed, bool exit, double exitDistRange, double minSpeed, double timeout){
    is_moving = true;
    stopChassis(vex::brakeType::coast);
    float distanceOutput;
    Pose pose(getPose().x, getPose().y, getPose().theta);
    
    float tolerance = (minSpeed == 0) ? 0.2 : 0.0;
    bool exitStatus = false, lastExitStatus = false, crossedOnce = false;
    maxSpeed = maxSpeed * 1000;
    //reset pids
    drivePID.reset();
    angularPID.reset();

    auto time = Brain.Timer.time();
    
    Brain.Screen.clearScreen(black);
    while((Brain.Timer.time() - time) < timeout){
        //exit conditions
        if(exit && fabs(pose.distance(target)) < 1){
            break;
        }
        if(!exit && fabs(pose.distance(target)) < exitDistRange){
            break;
        }

        lastExitStatus = hc(pose, target, getInertialReading(true), tolerance);
    
        pose = Pose(getPose().x, getPose().y, getPose().theta);

        exitStatus = hc(pose, target, getInertialReading(true), tolerance);

        if (!exitStatus && lastExitStatus) crossedOnce = true;
        
        
        distanceOutput = drivePID.update(pose.distance(target));
        
        
        if (exit && pose.distance(target) < 0.7) distanceOutput *= cos(DTR(pose.face(target, false)));

        //clamp min & max
        distanceOutput = copysign(clamp(fabs(distanceOutput), fabs(minSpeed) * 12000.0, 12000.0), distanceOutput);


        float angleError = pose.face(target, false);
        float angularOutput = turnPID.update(angleError);
        
        if (fabs(pose.distance(target)) < 0.5) angularOutput = 0;
        angularOutput = clamp(angularOutput, -12000.0, 12000.0);

        float rescale = fabs(distanceOutput) + fabs(angularOutput) - 12000.0;
        if (rescale > 0) distanceOutput -= copysign(rescale, distanceOutput);

        std::pair<float, float> outputs = reduceRatio(fabs(maxSpeed) * 12000.0, distanceOutput + angularOutput, distanceOutput - angularOutput);
        mirrorMoveDrivetrain(outputs.first, outputs.second, !forwards);

        wait(10, msec);
    }
    logger.colorize("settled...", Logger::Level::INFO);
    logger.info("end x: %.2f, y: %.2f, z: %.2f", getPose().x, getPose().y, getPose().theta);
    //brain settle screen
    Brain.Screen.clearScreen(red);
    if(exit){
        moveChassisRaw(0,0);
    }
    is_moving = false;

}
//motion.movePose(3, -0.25, 90, 0, 0, 0, 0.5, 5000);
void mtpose(Pose target, bool forwards, double dLead, double gLead, double chasePower, double maxSpeed, bool exit, double exitDistRange, double minSpeed, double timeout){
    is_moving = true;
    stopChassis(vex::brakeType::coast);
    int timer = 0;
    float tolerance = (minSpeed == 0) ? 0.2 : 0.0;
    bool closeEnd = false, closeGhost = false, exitStatus = false, lastExitStatus = false, crossedOnce = false;
    maxSpeed = maxSpeed * 1000;
    if (maxSpeed < 0) target.theta += 180.0;

    Pose end(target.x, target.y, target.theta);
    Pose pose(getPose().x, getPose().y, getPose().theta);
    Pose carrot(end.x - sin(DTR(end.theta)) * dLead, end.y - cos(DTR(end.theta)) * dLead, -1);
    Pose initialCarrot(carrot.x, carrot.y, -1);
    Pose targetPose(0,0,0);
    float minCarrotDist = pose.distance(carrot);
    float initCarrotDist = pose.distance(carrot);

    //reset PID
    drivePID.reset();
    angularPID.reset();

    auto time = Brain.Timer.time();
    
    Brain.Screen.clearScreen(black);
    while((Brain.Timer.time() - time) < timeout){
        //exit conditions
        if(exit && fabs(pose.distance(target)) < 1){
            break;
        }
        if(!exit && fabs(pose.distance(target)) < exitDistRange){
            break;
        }

        lastExitStatus = hc(pose, end, getInertialReading(true), tolerance);

        pose = Pose(getPose().x, getPose().y, getPose().theta);

        if (pose.distance(carrot) < minCarrotDist) minCarrotDist = pose.distance(carrot);
        carrot = Pose(end.x - (minCarrotDist / initCarrotDist) * sin(DTR(end.theta)) * dLead, end.y - (minCarrotDist / initCarrotDist) * cos(DTR(end.theta)) * dLead, -1);
        Pose ghost(initialCarrot.x + (carrot.x - initialCarrot.x) * (1 - gLead), initialCarrot.y + (carrot.y - initialCarrot.y) * (1 - gLead), -1);

        exitStatus = hc(pose, end, getInertialReading(true), tolerance);
        if (!exitStatus && lastExitStatus) crossedOnce = true;

        float angleError;
        if (pose.distance(carrot) < 0.7 || closeEnd) {
            closeEnd = true;
            if (pose.distance(end) < 0.7) angleError = pose.parallel(end);
            else angleError = pose.face(end, false);
            targetPose = Pose(end.x, end.y, end.theta);
        }
        else if (pose.distance(ghost) < 2 * fabs(maxSpeed) || closeGhost) {
            closeGhost = true;
            angleError = pose.face(carrot, false);
            targetPose = Pose(carrot.x, carrot.y, carrot.theta);
        }
        else {
            angleError = pose.face(ghost, false);
            targetPose = Pose(ghost.x, ghost.y, ghost.theta);
        }

        float angularOutput = clamp(angularPID.update(angleError), -12000.0, 12000.0);

        float distanceOutput = (closeEnd || pose.distance(end) > pose.distance(carrot)) ? drivePID.update(pose.distance(end)) : drivePID.update(pose.distance(carrot));
        if(!forwards){
            distanceOutput = -distanceOutput;
        }
        if (exit && closeEnd && pose.distance(end) < 0.7) distanceOutput *= cos(DTR(pose.face(targetPose, false)));
        distanceOutput = copysign(clamp(fabs(distanceOutput), fabs(minSpeed) * 12000.0, 12000.0), distanceOutput);
        float radius = getRadius(pose, targetPose) / std::fmin(pose.distance(targetPose), 1);
        float maxSlipSpeed = sqrt(chasePower * radius * 1000000);
        if (chasePower != -1 && (!closeEnd || pose.distance(end) >= 0.7)) distanceOutput = clamp(distanceOutput, -maxSlipSpeed, maxSlipSpeed);

        float rescale = fabs(distanceOutput) + fabs(angularOutput) - 12000.0;
        if (rescale > 0) distanceOutput -= copysign(rescale, distanceOutput);

        std::pair<float, float> outputs = reduceRatio(fabs(maxSpeed) * 12000.0, distanceOutput + angularOutput, distanceOutput - angularOutput);                                                                                              
        mirrorMoveDrivetrain(outputs.first, outputs.second, !forwards);

        wait(10, msec);
    }
    logger.colorize("settled...", Logger::Level::INFO);
    logger.info("end x: %.2f, y: %.2f, z: %.2f", getPose().x, getPose().y, getPose().theta);
    //brain settle screen
    Brain.Screen.clearScreen(red);
    if(exit){
        moveChassisRaw(0,0);
    }
    is_moving = false;
}


void driveLock(bool release){
    if(!release){
        is_moving = true;
        stopChassis(vex::brakeType::coast);
        //grep currents
        double static_left_deg = leftDTTravelDeg();
        double static_right_deg = rightDTTravelDeg();
        double static_heading = rollAngle180(getInertialReading(true));
        double error;
        double target = 0;
        //drive direction
        int drive_direction = target > 0 ? 1 : -1;
        target = target * drive_direction;

        //reset PIDS
        drivePID.reset();
        headingPID.reset();
        double left_output, right_output, heading_output;
        auto time = Brain.Timer.time();
        
        Brain.Screen.clearScreen(black);
        while(true){
            
            double current_distance = (fabs(((leftDTTravelDeg() - static_left_deg) / 360.0) * wheel_distance_in) + fabs(((rightDTTravelDeg() - static_right_deg) / 360.0) * wheel_distance_in)) / 2;
            double current_heading_deg = getInertialReading(true);
            double error = target - current_distance;

            left_output = drivePID.update(error) * 1;
            right_output = left_output;

            heading_output = headingPID.update(rollAngle180(getInertialReading(true) - current_heading_deg));

            //clip speeds
            left_output = clamp(left_output, -6, 6);
            right_output = clamp(right_output, -6, 6);
            heading_output = clamp(heading_output, -2, 2);

            //set speeds
            moveChassisRaw(left_output - heading_output, right_output + heading_output);

            wait(10, msec);    

        }
    }
}

void followPursuit(Path path,float timeout, float chasePower, bool reverse, bool exit){
    stopChassis(coast);
    int timer = 0, pathIndex = 0;
    Pose pose = getPose();
    auto time = Brain.Timer.time();

    while(((path.lastPoint().theta == 0 && getSpeed() > 0.5)
    || pathIndex != path.lastIndex()) && (Brain.Timer.time() - time) < timeout){

        pose = getPose();
        pathIndex = path.closestIndex(pose);

        float crosstrackError = pose.distance(path.at(pathIndex));
        float lookahead = clamp(getSpeed() * pursuitVeloConst + getRadius(pose, path.at(pathIndex + radiusLookahead)) * pursuitCurvConst - crosstrackError * pursuitCTEConst, minLookahead, maxLookahead);
        Pose lookaheadPoint = path.lookaheadPoint(pose, pathIndex, lookahead);

        float turnError = pose.face(lookaheadPoint, false);
        if (fabs(turnError) >= 90) {
            int dir = std::copysign(1.0, turnError);
            mirrorMoveDrivetrain(path.at(pathIndex).theta * dir, path.at(pathIndex).theta * -dir, reverse);
            continue;
        }

        float voltageScale = path.at(pathIndex).theta;
        if (chasePower != -1) {
            float radius = getRadius(pose, lookaheadPoint);
            voltageScale = std::fmin(sqrt(chasePower * radius / lookahead * 1000000), voltageScale);
        }

        float curvature = getCurvature(pose, lookaheadPoint);
        float leftRatio = 2 + curvature * trackwidth;
        float rightRatio = 2 - curvature * trackwidth;

        std::pair<float, float> outputs = scaleToRatio(voltageScale, leftRatio, rightRatio);
        mirrorMoveDrivetrain(outputs.first, outputs.second, reverse);

        wait(10, msec);
    }
    logger.colorize("settled...", Logger::Level::INFO);
    logger.info("end x: %.2f, y: %.2f, z: %.2f", getPose().x, getPose().y, getPose().theta);
    //brain settle screen
    Brain.Screen.clearScreen(red);
    if(exit){
        mirrorMoveDrivetrain(0,0, false);
    }

    is_moving = false;

}

void followStanley(Path path, float timeout, float k, float chasePower, bool reverse){
    stopChassis(coast);
    int timer = 0, pathIndex = 0;
    Pose pose = getPose();
    auto time = Brain.Timer.time();
    int furthestIndex;
    while(((path.lastPoint().theta == 0 && getSpeed() > 0.5)
    || pathIndex != path.lastIndex()) && (Brain.Timer.time() - time) < timeout){

        pose = getPose();
        pathIndex = path.closestIndex(pose);
        if (pathIndex > furthestIndex) furthestIndex = pathIndex;
        else if (pathIndex < furthestIndex) pathIndex = furthestIndex;

        Pose curPoint = path.at(pathIndex);
        Pose nextPoint = path.at(pathIndex + 1);
        Pose prevPoint = path.at(pathIndex - 1);

        float pathHeading;
        if (pathIndex == path.lastIndex()) pathHeading = prevPoint.angle(curPoint);
        else pathHeading = curPoint.angle(nextPoint);

        float crosstrackError = std::copysign(pose.distance(curPoint), -getCurvatureHeading(curPoint, pose, pathHeading));
        //test Misc::getCurvature(pose, curPoint, pathHeading);

        float headingError = pose.parallel(pathHeading);
        float crosstrackScale = RTD(atan(k * crosstrackError / getSpeed()));
        float steering = 5 * headingError + crosstrackScale;

        if (fabs(steering) >= 90) {
            int dir = std::copysign(1.0, steering);
            mirrorMoveDrivetrain(path.at(pathIndex).theta * dir, path.at(pathIndex).theta * -dir, reverse);
            continue;
        }

        float voltageScale = path.at(pathIndex).theta;
        if (chasePower != -1) {
            float radius = trackwidth / tan(DTR(fabs(steering)));
            voltageScale = std::fmin(sqrt(chasePower * radius * 1000000), voltageScale);
        }

        float leftRatio = 2 + tan(DTR(steering)) / 2.0;
        float rightRatio = 2 - tan(DTR(steering)) / 2.0;

        std::pair<float, float> outputs = scaleToRatio(voltageScale, leftRatio, rightRatio);
        mirrorMoveDrivetrain(outputs.first, outputs.second, reverse);

        wait(10, msec);
    }
    logger.colorize("settled...", Logger::Level::INFO);
    logger.info("end x: %.2f, y: %.2f, z: %.2f", getPose().x, getPose().y, getPose().theta);
    //brain settle screen
    Brain.Screen.clearScreen(red);
    if(exit){
        mirrorMoveDrivetrain(0,0, false);
    }

    is_moving = false;
}
