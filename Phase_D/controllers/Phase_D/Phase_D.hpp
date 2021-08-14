#ifndef PHASED_H
#define PHASED_H

#include "MotionPlanRunner.hpp"
#include "MotionStrategy.hpp"
#include "RemoteMapBuilder.hpp"

class PhaseD {
private:
    std::unique_ptr<MotionStrategy> motion;
    // Path Planning Class

    void printInstructions();
    void findStrategy(const int &key, std::unique_ptr<Robot> &robot);

public:
    PhaseD() {}

    void setup(std::unique_ptr<Robot> &robot);
    void process();
    void finish();
};

#endif