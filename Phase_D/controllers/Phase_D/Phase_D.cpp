// File:          	z5209697_MTRN4110_PhaseA.cpp
// Date:			
// Description:	 	Controller of E-puck for Phase A - Driving and Perception
// Author:			Daniel Lin
// Modifications:
// Platform: 		MacOS
// Notes: 			Output file is written with ',' delimiter

#include <iostream>
#include "MotionPlanRunner.hpp"

/**
 * Initialise Robot instance and run robot
 **/ 
int main(int argc, char **argv) {
	// create the Robot instance.
	std::unique_ptr<Robot> robot {std::make_unique<Robot>()};
	MotionPlanRunner racer{robot};

	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	try {
		racer.process();
	} catch (const std::exception &e) {
		std::cout << e.what() << std::endl;
		racer.finishRobot();
		racer.process();
	}

	return 0;
}