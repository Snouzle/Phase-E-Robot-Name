// File:          	z5209697_MTRN4110_PhaseA.cpp
// Date:			
// Description:	 	Controller of E-puck for Phase A - Driving and Perception
// Author:			Daniel Lin
// Modifications:
// Platform: 		MacOS
// Notes: 			Output file is written with ',' delimiter

/**
 * Improvements:
 * Abstracted MotionPlanRunner to allow inheritance of strategy
 * TODO: Make moving forward not stop every step
 **/ 

#include "MotionStrategy.hpp"

/**
 * MotionStrategy Constructor
 **/
MotionStrategy::MotionStrategy(std::unique_ptr<Robot> &robot): mRobot{std::move(robot)},
					   mTimeStep{TIME_STEP}, mState{std::make_unique<InitialState>()} {
    Motor *leftMotor = mRobot->getMotor("left wheel motor");
	Motor *rightMotor = mRobot->getMotor("right wheel motor");

	leftMotor->setPosition(INFINITY);
	rightMotor->setPosition(INFINITY);
	leftMotor->setVelocity(0.0);
	rightMotor->setVelocity(0.0);

	// Initialise wheel sensors
	PositionSensor *leftSensor = mRobot->getPositionSensor("left wheel sensor");
	PositionSensor *rightSensor = mRobot->getPositionSensor("right wheel sensor");
	leftSensor->enable(mTimeStep);
	rightSensor->enable(mTimeStep);

	// Initialise Distance sensors
	std::array<std::string, DISTANCE_SENSOR_NUMBER+1> dsNames {
        "left_sensor", "front_sensor", "right_sensor", "back_sensor"
    };

	for (int i = 0; i < DISTANCE_SENSOR_NUMBER+1; i++)
		mRobot->getDistanceSensor(dsNames[i])->enable(mTimeStep);

	// Initialise IMU
	InertialUnit *imu = mRobot->getInertialUnit("inertial_unit");
	imu->enable(mTimeStep);
}

MotionStrategy::~MotionStrategy() {}

/**
 * Robot finishing process
 **/
void MotionStrategy::finishRobot() {
	moveRobot(0, 0, ChangeHeading::FORWARD);
	std::cout << PREFIX << "Motion plan executed!" << std::endl;
}

/**
 * Robot running process
 **/
void MotionStrategy::process() {
	std::cout << PREFIX << "Executing motion plan..." << std::endl;
	while (step() != -1) {
		mState->process(*this);
	}
}

/**
 * Update the robot's position (row and column) depending on
 * heading direction
 **/ 
void MotionStrategy::updatePosition(const int &repeats=1) {
	switch (mHeading) {
		case (int)Heading::NORTH:
			mRow -= repeats;
			break;
		case (int)Heading::SOUTH:
			mRow += repeats;
			break;
		case (int)Heading::EAST:
			mCol += repeats;
			break;
		case (int)Heading::WEST:
			mCol -= repeats;
			break;
		default:
			break;
	}
}

std::array<double, MOTOR_NUMBER> MotionStrategy::getMotorSensors() {
	return std::array<double, MOTOR_NUMBER> {mRobot->getPositionSensor("left wheel sensor")->getValue(),
	 										 mRobot->getPositionSensor("right wheel sensor")->getValue()};
}

void MotionStrategy::setTargetPosition(const double &leftPosition, const double &rightPosition) {
	mLeftMotorTarget = leftPosition;
	mRightMotorTarget = rightPosition;
}

std::array<double, MOTOR_NUMBER> MotionStrategy::getTargetPosition() {
	return std::array<double, MOTOR_NUMBER> {mLeftMotorTarget, mRightMotorTarget};
}

/**
 * Set Motor Velocities and Position and update Step
 **/ 
void MotionStrategy::moveRobot(const double &leftVelocity, const double &rightVelocity,
							     const ChangeHeading &heading) {
	Motor *leftMotor = mRobot->getMotor("left wheel motor");
	Motor *rightMotor = mRobot->getMotor("right wheel motor");

	// Update heading
	int tempHeading = mHeading + (int)heading;
	tempHeading = (tempHeading >= 0) ? tempHeading : NUM_DIRECTIONS + tempHeading;
	mHeading = tempHeading % NUM_DIRECTIONS;

	leftMotor->setVelocity(leftVelocity);
	rightMotor->setVelocity(rightVelocity);

	leftMotor->setPosition(mLeftMotorTarget);
	rightMotor->setPosition(mRightMotorTarget);

	mStep++;
}

/**
 * Distance Sensors from left to right
 **/ 
std::array<double, NUM_DIRECTIONS> MotionStrategy::getDistanceSensors() {
	std::array<std::string, NUM_DIRECTIONS> dsNames {
        "left_sensor", "front_sensor", "right_sensor", "back_sensor"
    };

	std::array<double, NUM_DIRECTIONS> dSensors;

	for (int i = 0; i < NUM_DIRECTIONS; i++)
		dSensors[i] = mRobot->getDistanceSensor(dsNames[i])->getValue();

	return dSensors;
}

/**
 * Defined order of heading
 **/ 
std::array<char, NUM_DIRECTIONS> MotionStrategy::getHeadingList() { 
	return std::array<char, NUM_DIRECTIONS> {
		'N', 'E', 'S', 'W'
	};
}

/**
 * Set Robot heading direction, given char input
 **/ 
void MotionStrategy::setHeading(const char &heading) {
	auto headings = getHeadingList();
	auto iter = headings.begin();
	int pos = 0;
	mHeading = 0;

	while (iter != headings.end()) {
		if (*iter == heading) mHeading = pos;
		pos++;
		iter++;
	}
}

/**
 * Get the heading character
 **/ 
char MotionStrategy::getHeading() {
	return getHeadingList()[mHeading];
}

/**
 * Get the heading angle from the heading (East has a Yaw of 0)
 **/ 
double MotionStrategy::getHeadingAngle() {
	std::array<double, NUM_DIRECTIONS> angles {PI/2, 0, -PI/2, -PI};

	return angles[mHeading];
}

void MotionStrategy::setState(std::unique_ptr<RobotState> &state) { mState = std::move(state); }

bool MotionStrategy::isWall(const int &direction) {
    if (direction < (int)Wall::LEFT || direction > DISTANCE_SENSOR_NUMBER) return false;

    return getDistanceSensors()[direction] < OBSTACLE_THRESHOLD;
}

std::pair<int, int> MotionStrategy::getPreviousPosition() {
	std::pair<int, int> pair{mRow, mCol};

	switch (mHeading) {
		case (int)Heading::NORTH:
			pair.first++;
			break;
		case (int)Heading::SOUTH:
			pair.first--;
			break;
		case (int)Heading::EAST:
			pair.second--;
			break;
		case (int)Heading::WEST:
			pair.second++;
			break;
		default:
			break;
	}

	return pair;
}