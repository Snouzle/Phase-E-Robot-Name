// File:          	z5209697_MTRN4110_PhaseA.cpp
// Date:			
// Description:	 	Controller of E-puck for Phase A - Driving and Perception
// Author:			Daniel Lin
// Modifications:
// Platform: 		MacOS
// Notes: 			Output file is written with ',' delimiter

#include "Phase_D.hpp"

void PhaseD::setup(std::unique_ptr<Robot> &robot) {
	int key {-1};

	printInstructions();
	Keyboard *keyboard = robot->getKeyboard();
	keyboard->enable(TIME_STEP);

	// Wait for instruction
	while ((robot->step(TIME_STEP) != -1) && (key < '1' || key > '3'))
		key = keyboard->getKey();

	findStrategy(key - '0', robot);
}

void PhaseD::printInstructions() {
	std::cout << PREFIX << "\tPlease select a command:\n";
	std::cout << PREFIX << "\t[1]\trun Robot in Normal mode\n";
	std::cout << PREFIX << "\t[2]\trun Robot in Remote Map Building mode\n";
	std::cout << PREFIX << "\t[3]\trun Robot in Automatic Map Building mode\n";

	std::cout << std::endl;
}

void PhaseD::findStrategy(const int &key, std::unique_ptr<Robot> &robot) {
	switch (key) {
		case 2:
			motion = std::make_unique<RemoteMapBuilder>(robot);
			break;
		case 3:
			motion = std::make_unique<AutoMapBuilder>(robot);
			break;
		default:
			motion = std::make_unique<MotionPlanRunner>(robot);
	}
}

void PhaseD::process() {
	motion->process();
}

void PhaseD::finish() {
	motion->finishRobot();
	motion->process();
}

/**
 * Initialise Robot instance and run robot
 **/ 
int main(int argc, char **argv) {
	// create the Robot instance.
	std::unique_ptr<Robot> robot {std::make_unique<Robot>()};
	PhaseD controller;

	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	try {
		controller.setup(robot);
		controller.process();
	} catch (const std::exception &e) {
		std::cout << e.what() << std::endl;
		controller.finish();
	}

	return 0;
}