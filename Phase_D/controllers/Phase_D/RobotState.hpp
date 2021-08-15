#ifndef ROBOT_STATE
#define ROBOT_STATE

#include "MotionStrategy.hpp"
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

    virtual void process(MotionStrategy &runner) = 0;
};

// Initialising Robot state
class InitialState: public RobotState {
public:
    InitialState(): RobotState() {}

    virtual void process(MotionStrategy &runner);
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

    virtual void process(MotionStrategy &runner);
};

// Turning left / right state
class TurningState: public RobotState {
private:
    double mPt, mVp{0}, mVd, mAd, mPreviousError {0},
           mErrorIntegral{0};

public:
    TurningState(const double &vd, const double &ad, MotionStrategy &runner);

    virtual void process(MotionStrategy &runner);
};

// Moving forward state
class RunningState: public RobotState {
private:
    double mPrevLeftSensor{0}, mPrevRightSensor{0},
           mStartLeftSensor, mStartRightSensor,
           mLeftTarget, mRightTarget;

    bool replan{false};

public:
    RunningState(const double &leftSensor, const double &rightSensor,
                 const int &repeats);

    virtual void process(MotionStrategy &runner);
};

// Robot finished state
class FinishedState: public RobotState {
public:
    FinishedState(MotionStrategy &runner);

    virtual void process(MotionStrategy &runner) {}
};

// Returns sign of a double
int sign(const double &val);

#endif