#ifndef PHASED_H
#define PHASED_H

#include <iostream>
#include <cmath>

#include "MotionPlanRunner.hpp"
#include "MotionStrategy.hpp"
#include "RemoteMapBuilder.hpp"
#include "AutoMapBuilder.hpp"

class PhaseD {
private:
    std::unique_ptr<MotionStrategy> motion;
    // Path Planning Class

    void printInstructions();
    void printTrajectoryInstructions();
    void findStrategy(const int &key, std::unique_ptr<Robot> &robot);
    void setTrajectory(const int &key);

public:
    PhaseD() {}

    void setup(std::unique_ptr<Robot> &robot);
    void process();
    void finish();
};

#endif