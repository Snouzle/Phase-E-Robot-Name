#ifndef RUNNER_H
#define RUNNER_H

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>

#include <string>
#include <array>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

// Defined Variables
constexpr double OBSTACLE_THRESHOLD {700.0};
constexpr double PI {3.14159};
constexpr double FORWARD_RADIANS {8.18};
constexpr double TURN_RADIANS {2.2227}; // REPLACED WITH ROBOT ANGLE CONTROLLER
constexpr double FORWARD_VEL {5.0};
constexpr double MAX_SPEED {6.28};

constexpr int TIME_STEP {64};
constexpr int DISTANCE_SENSOR_NUMBER {3};
constexpr int MOTOR_NUMBER {2};
constexpr int NUM_DIRECTIONS {4};

const std::string MOTION_PLAN_FILE_NAME {"../../MotionPlan.txt"};
const std::string MOTION_EXECUTION_FILE_NAME {"../../MotionExecution.csv"};
const std::string PREFIX {"[z5209697_MTRN4110_PhaseA] "};


// All the webots classes are defined in the "webots" namespace
using namespace webots;

// Forward Declaration
class RobotState;

class MotionPlanRunner {
private:
	std::stringstream mMotionPlan;
	std::unique_ptr<Robot> mRobot;
	std::unique_ptr<RobotState> mState;
	const int mTimeStep;
	int mStep{0}, mRow, mCol, mHeading;
	double mLeftMotorTarget{0.0}, mRightMotorTarget{0.0};
	
public:
	MotionPlanRunner(std::unique_ptr<Robot> &robot);
	~MotionPlanRunner();

	enum class ChangeHeading {
		LEFT = -1,
		FORWARD,
		RIGHT
	};

    enum class Heading {
        NORTH = 0,
        EAST,
        SOUTH,
        WEST
    };

	void finishRobot();
	void process();
	void printState();
    void updatePosition();

	void moveRobot(const double &leftVelocity, const double &rightVelocity,
				   const ChangeHeading &heading);

	std::array<double, DISTANCE_SENSOR_NUMBER> getDistanceSensors();
	std::array<char, NUM_DIRECTIONS> getHeadingList();
	std::array<double, MOTOR_NUMBER> getMotorSensors();
	void setHeading(const char &heading);
	char getHeading();
	double getHeadingAngle();
	char getNextMotion();
	int getRow() { return mRow; }
	int getCol() { return mCol; }
	int getStep() { return mStep; }

	double getLeftMotor() { return mRobot->getMotor("left wheel motor")->getVelocity(); }
	void setLeftMotor(const double &vel) { mRobot->getMotor("left wheel motor")->setVelocity(vel); }

	double getRightMotor() { return mRobot->getMotor("right wheel motor")->getVelocity(); }
	void setRightMotor(const double &vel) { mRobot->getMotor("right wheel motor")->setVelocity(vel); }

	std::array<double, MOTOR_NUMBER> getTargetPosition();
	void setTargetPosition(const double &leftPosition, const double &rightPosition);

	const double *getIMU() { return mRobot->getInertialUnit("inertial_unit")->getRollPitchYaw(); }
	double getTimeStep() { return mTimeStep; }
	void setState(std::unique_ptr<RobotState> &state);
};

#include "RobotState.hpp"

#endif