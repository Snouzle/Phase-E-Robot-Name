#ifndef AMAPB_H
#define AMAPB_H

#include "MapBuilder.hpp"
#include "MotionPlanRunner.hpp"
#include <string>
#include <sstream>

class AutoMapBuilder: public MapBuilder {
private:
    const std::string BUILD_OUT_FILE{"../../AutoMap.txt"};
    std::stringstream mMotionPlan;
    bool runningPlan{false};

    char moveAdjacentSquare();
    void planPath(const int &currPoint, const int &obstacle=-1);

public:
    AutoMapBuilder(std::unique_ptr<Robot> &robot);

    virtual char getNextMotion() override;
    virtual void replan() override;
    virtual int getNumRepeat(const char &letter) override;
};

#endif