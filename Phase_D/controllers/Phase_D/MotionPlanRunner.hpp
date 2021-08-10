#ifndef RUNNER_H
#define RUNNER_H

#include "MotionStrategy.hpp"
#include <sstream>
#include <fstream>

const std::string MOTION_PLAN_FILE_NAME {"../../MotionPlan.txt"};
const std::string MOTION_EXECUTION_FILE_NAME {"../../MotionExecution.csv"};


// All the webots classes are defined in the "webots" namespace
using namespace webots;

// Forward Declaration
class MotionPlanRunner: public MotionStrategy {
private:
	std::stringstream mMotionPlan;

public:
	MotionPlanRunner(std::unique_ptr<Robot> &robot);
	virtual ~MotionPlanRunner();

	virtual void processState() override;
	virtual char getNextMotion() override;
};

#endif