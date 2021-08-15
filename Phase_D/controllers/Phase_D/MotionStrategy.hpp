#ifndef MOSTRAT_H
#define MOSTRAT_H

#include "PathPlanner.hpp" 

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/InertialUnit.hpp>

#include <string>
#include <array>
#include <iostream>
#include <iomanip>
#include <utility>

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

const std::string PREFIX {"[Phase D] "};


// All the webots classes are defined in the "webots" namespace
using namespace webots;

// Forward Declaration
class RobotState;

class MotionStrategy {
public:
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

    enum class Wall {
        LEFT = 0,
        FRONT,
        RIGHT
    };

private:
	std::unique_ptr<Robot> mRobot;
	std::unique_ptr<RobotState> mState;
	const int mTimeStep;
	int mStep{0}, mRow{0}, mCol{0}, mHeading;
	double mLeftMotorTarget{0.0}, mRightMotorTarget{0.0};
	
public:
	MotionStrategy(std::unique_ptr<Robot> &robot);
	virtual ~MotionStrategy();

	void finishRobot();
	void process();
	virtual void processState() = 0;
	virtual void replan() = 0;
	virtual int getNumRepeat(const char &letter) = 0;
    void updatePosition(const int &repeats);

	void moveRobot(const double &leftVelocity, const double &rightVelocity,
				   const ChangeHeading &heading);

	std::array<double, NUM_DIRECTIONS> getDistanceSensors();
	std::array<char, NUM_DIRECTIONS> getHeadingList();
	std::array<double, MOTOR_NUMBER> getMotorSensors();
	void setHeading(const char &heading);
    void setHeading(const Heading &heading) { mHeading = (int)heading; };
	char getHeading();
	double getHeadingAngle();
    int getHeadingDirection() { return mHeading; }
	virtual char getNextMotion() = 0;
	int getRow() { return mRow; }
    void setRow(const int &row) { mRow = row; }
	int getCol() { return mCol; }
    void setCol(const int &col) { mCol = col; }
	int getStep() { return mStep; }

	double getLeftMotor() { return mRobot->getMotor("left wheel motor")->getVelocity(); }
	void setLeftMotor(const double &vel) { mRobot->getMotor("left wheel motor")->setVelocity(vel); }

	double getRightMotor() { return mRobot->getMotor("right wheel motor")->getVelocity(); }
	void setRightMotor(const double &vel) { mRobot->getMotor("right wheel motor")->setVelocity(vel); }

    Keyboard *getKeyboard() { return mRobot->getKeyboard(); }

	std::array<double, MOTOR_NUMBER> getTargetPosition();
	void setTargetPosition(const double &leftPosition, const double &rightPosition);

	const double *getIMU() { return mRobot->getInertialUnit("inertial_unit")->getRollPitchYaw(); }
	double getTimeStep() { return mTimeStep; }
	void setState(std::unique_ptr<RobotState> &state);

    int step() { return mRobot->step(mTimeStep); }
    bool isWall(const int &direction);

	std::pair<int, int> getPreviousPosition();
};

#include "RobotState.hpp"

#endif