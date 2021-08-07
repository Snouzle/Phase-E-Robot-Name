#ifndef ROBOT_STATE
#define ROBOT_STATE

#include "z5209697_MTRN4110_PhaseA.hpp"
#include <cmath>

constexpr double KP {20};
constexpr double KI = {0.0};
constexpr double KD = {0.0};

// Robot state interface
class RobotState {
private:
public:
    RobotState() {}
    virtual ~RobotState() {}

    virtual void process(MotionPlanRunner &runner) = 0;
};

// Initialising Robot state
class InitialState: public RobotState {
public:
    InitialState(): RobotState() {}

    virtual void process(MotionPlanRunner &runner);
};

// Reading and processing input
class ReadState: public RobotState {
private:
    enum class Direction {
		LEFT = 'L',
		FORWARD = 'F',
		RIGHT = 'R'
	};
public:
    ReadState(): RobotState() {}

    virtual void process(MotionPlanRunner &runner);
};

// Turning left / right state
class TurningState: public RobotState {
private:
    double mPt, mVp{0}, mVd, mAd, mPreviousError {0},
           mErrorIntegral{0};

public:
    TurningState(const double &vd, const double &ad, MotionPlanRunner &runner);

    virtual void process(MotionPlanRunner &runner);
};

// Moving forward state
class RunningState: public RobotState {
private:
    double mPrevLeftSensor{0}, mPrevRightSensor{0};

public:
    RunningState(): RobotState() {}

    virtual void process(MotionPlanRunner &runner);
};

// Robot finished state
class FinishedState: public RobotState {
public:
    FinishedState(MotionPlanRunner &runner);

    virtual void process(MotionPlanRunner &runner) {}
};

// Returns sign of a double
int sign(const double &val);

#endif