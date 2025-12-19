#pragma once

#include "pose.h"
#include "utils.h"
#include "vex.h"
#include <vector>

class Path {


  private:

    std::vector<Pose> waypoints;


  public:

    Path(std::vector<Pose> waypoints);

    int closestIndex(Pose pose);
    int lookaheadIndex(Pose pose, int startIndex, float lookahead);
    int lastIndex();
    Pose closestPoint(Pose pose);
    Pose lookaheadPoint(Pose pose, int startIndex, float lookahead);
    Pose lastPoint();
    Pose at(int index);
    bool nearEnd(int index);
};