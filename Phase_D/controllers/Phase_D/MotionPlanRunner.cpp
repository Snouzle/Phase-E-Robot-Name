// File:          	z5209697_MTRN4110_PhaseA.cpp
// Date:			
// Description:	 	Controller of E-puck for Phase A - Driving and Perception
// Author:			Daniel Lin
// Modifications:
// Platform: 		MacOS
// Notes: 			Output file is written with ',' delimiter

#include "MotionPlanRunner.hpp"

/**
 * MotionPlanRunner Constructor
 **/
MotionPlanRunner::MotionPlanRunner(std::unique_ptr<Robot> &robot): mRobot{std::move(robot)},
					   mTimeStep{TIME_STEP}, mState{std::make_unique<InitialState>()} {
	// Read Motion plan from file
	std::cout << PREFIX << "Reading in motion plan from " << MOTION_PLAN_FILE_NAME 
			  << "..." << std::endl;
	std::ifstream fd {MOTION_PLAN_FILE_NAME};
	std::string motionPlan;
	std::getline(fd, motionPlan);

	fd.close();

	// Print Motion Plan Line
	mMotionPlan << motionPlan;
	std::cout << PREFIX << "Motion Plan: " << motionPlan << std::endl;
	std::cout << PREFIX << "Motion plan read in!" << std::endl;

	char heading, row, col;
	mMotionPlan >> row >> col >> heading;

	mRow = row - '0';
	mCol = col - '0';

	setHeading(heading);

	// Set up output file
	std::ofstream ofd {MOTION_EXECUTION_FILE_NAME, std::ios_base::out | std::ios_base::trunc};
	ofd << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall" << std::endl;
	ofd.close();

	// Initialise Motors
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
	std::array<std::string, DISTANCE_SENSOR_NUMBER> dsNames {
        "left_sensor", "front_sensor", "right_sensor"
    };

	for (int i = 0; i < DISTANCE_SENSOR_NUMBER; i++)
		mRobot->getDistanceSensor(dsNames[i])->enable(mTimeStep);

	// Initialise IMU
	InertialUnit *imu = mRobot->getInertialUnit("inertial_unit");
	imu->enable(mTimeStep);
}

MotionPlanRunner::~MotionPlanRunner() {}

/**
 * Robot finishing process
 **/
void MotionPlanRunner::finishRobot() {
	moveRobot(0, 0, ChangeHeading::FORWARD);
	std::cout << PREFIX << "Motion plan executed!" << std::endl;
}

/**
 * Robot running process
 **/
void MotionPlanRunner::process() {
	std::cout << PREFIX << "Executing motion plan..." << std::endl;
	while (mRobot->step(mTimeStep) != -1) {
		mState->process(*this);
	}
}

/**
 * Print Current Robot State to Console and Output File
 **/
void MotionPlanRunner::printState() {
	std::ofstream ofd {MOTION_EXECUTION_FILE_NAME, std::ios_base::app | std::ios_base::out};

	// Print status to console
	std::cout << PREFIX << "Step: " << std::setfill('0') << std::setw(3)
	 		  << mStep << ", ";

	std::cout << "Row: " << mRow << ", Column: " << mCol << ", Heading: "
				<< getHeading() << std::flush;

	// Print status to output file
	ofd << mStep << "," << mRow << "," << mCol << "," << getHeading() << std::flush;

	auto dSensors = getDistanceSensors();
	char isWall;

	std::array<std::string, DISTANCE_SENSOR_NUMBER> wallPositions {
		"Left Wall", "Front Wall", "Right Wall"
	};

	// Print wall existence to console and output files
	for (int i = 0; i < DISTANCE_SENSOR_NUMBER; i++) {
		isWall = dSensors[i] < OBSTACLE_THRESHOLD ? 'Y' : 'N';
		std::cout << ", " << wallPositions[i] << ": " << isWall;
		ofd << "," << isWall;
	}

	std::cout << std::endl;
	ofd << std::endl;
	ofd.close();
}

/**
 * Update the robot's position (row and column) depending on
 * heading direction
 **/ 
void MotionPlanRunner::updatePosition() {
	switch (mHeading) {
		case (int)Heading::NORTH:
			mRow--;
			break;
		case (int)Heading::SOUTH:
			mRow++;
			break;
		case (int)Heading::EAST:
			mCol++;
			break;
		case (int)Heading::WEST:
			mCol--;
			break;
		default:
			break;
	}
}

std::array<double, MOTOR_NUMBER> MotionPlanRunner::getMotorSensors() {
	return std::array<double, MOTOR_NUMBER> {mRobot->getPositionSensor("left wheel sensor")->getValue(),
	 										 mRobot->getPositionSensor("right wheel sensor")->getValue()};
}

void MotionPlanRunner::setTargetPosition(const double &leftPosition, const double &rightPosition) {
	mLeftMotorTarget = leftPosition;
	mRightMotorTarget = rightPosition;
}

std::array<double, MOTOR_NUMBER> MotionPlanRunner::getTargetPosition() {
	return std::array<double, MOTOR_NUMBER> {mLeftMotorTarget, mRightMotorTarget};
}

/**
 * Set Motor Velocities and Position and update Step
 **/ 
void MotionPlanRunner::moveRobot(const double &leftVelocity, const double &rightVelocity,
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
std::array<double, DISTANCE_SENSOR_NUMBER> MotionPlanRunner::getDistanceSensors() {
	std::array<std::string, DISTANCE_SENSOR_NUMBER> dsNames {
        "left_sensor", "front_sensor", "right_sensor"
    };

	std::array<double, DISTANCE_SENSOR_NUMBER> dSensors;

	for (int i = 0; i < DISTANCE_SENSOR_NUMBER; i++)
		dSensors[i] = mRobot->getDistanceSensor(dsNames[i])->getValue();

	return dSensors;
}

/**
 * Defined order of heading
 **/ 
std::array<char, NUM_DIRECTIONS> MotionPlanRunner::getHeadingList() { 
	return std::array<char, NUM_DIRECTIONS> {
		'N', 'E', 'S', 'W'
	};
}

/**
 * Set Robot heading direction, given char input
 **/ 
void MotionPlanRunner::setHeading(const char &heading) {
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
char MotionPlanRunner::getHeading() {
	return getHeadingList()[mHeading];
}

/**
 * Get the heading angle from the heading (East has a Yaw of 0)
 **/ 
double MotionPlanRunner::getHeadingAngle() {
	std::array<double, NUM_DIRECTIONS> angles {PI/2, 0, -PI/2, -PI};

	return angles[mHeading];
}

/**
 * Get next letter from motion plan
 **/ 
char MotionPlanRunner::getNextMotion() {
	char motion;
	mMotionPlan >> motion;
	return motion; 
}

void MotionPlanRunner::setState(std::unique_ptr<RobotState> &state) { mState = std::move(state); }