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
MotionPlanRunner::MotionPlanRunner(std::unique_ptr<Robot> &robot): MotionStrategy{robot} {
	// Read Motion plan from file
	// std::cout << PREFIX << "Reading in motion plan from " << MOTION_PLAN_FILE_NAME 
	// 		  << "..." << std::endl;
	// std::ifstream fd {MOTION_PLAN_FILE_NAME};
	// std::string motionPlan;
	// std::getline(fd, motionPlan);

	// fd.close();

	// Testing PathPlanner - To use this instead now 
	std::cout << "Reading in Sasha's Phase B .. " << std::endl; 
	std::string path_planner = getPath(); 
	std::cout << path_planner << std::endl;

	// Print Motion Plan Line
	mMotionPlan << path_planner;
	std::cout << PREFIX << "Motion Plan: " << path_planner << std::endl;
	std::cout << PREFIX << "Motion plan read in!" << std::endl;

	char heading, row, col;
	mMotionPlan >> row >> col >> heading;

	setRow(row - '0');
	setCol(col - '0');
	setHeading(heading);

	// Set up output file
	std::ofstream ofd {MOTION_EXECUTION_FILE_NAME, std::ios_base::out | std::ios_base::trunc};
	ofd << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall" << std::endl;
	ofd.close();
}

MotionPlanRunner::~MotionPlanRunner() {}

/**
 * Print Current Robot State to Console and Output File
 **/
void MotionPlanRunner::processState() {
	std::ofstream ofd {MOTION_EXECUTION_FILE_NAME, std::ios_base::app | std::ios_base::out};
	int row{getRow()}, col{getCol()};
	char heading{getHeading()};

	// Print status to console
	std::cout << PREFIX << "Step: " << std::setfill('0') << std::setw(3)
	 		  << getStep() << ", ";

	std::cout << "Row: " << row << ", Column: " << col << ", Heading: "
				<< heading << std::flush;

	// Print status to output file
	ofd << getStep() << "," << row << "," << col << "," << heading << std::flush;

	char wall;

	std::array<std::string, DISTANCE_SENSOR_NUMBER> wallPositions {
		"Left Wall", "Front Wall", "Right Wall"
	};

	// Print wall existence to console and output files
	for (int i = 0; i < DISTANCE_SENSOR_NUMBER; i++) {
		wall = isWall(i) ? 'Y' : 'N';
		std::cout << ", " << wallPositions[i] << ": " << wall;
		ofd << "," << wall;
	}

	std::cout << std::endl;
	ofd << std::endl;
	ofd.close();
}

/**
 * Get next letter from motion plan
 **/ 
char MotionPlanRunner::getNextMotion() {
	char motion;
	mMotionPlan >> motion;
	return motion; 
}

void MotionPlanRunner::replan() {
	std::pair<int, int> posPair{getPreviousPosition()};
	int pos = posPair.first * ROW + posPair.second;
	int obstacle = getRow() * ROW + getCol();

	std::string newPlan = getPath(pos, obstacle, getHeading(), "Map.txt");
	mMotionPlan.str(newPlan.substr(3));
	mMotionPlan.clear();
	std::cout << newPlan.substr(3) << std::endl;
	setRow(posPair.first);
	setCol(posPair.second);
}

int MotionPlanRunner::getNumRepeat(const char &letter) {
	mMotionPlan.putback(letter);
    int repeats = 0;
    char a;
    mMotionPlan >> a;

    while ((!mMotionPlan.eof()) && (a == letter)) {
		mMotionPlan >> a;
        repeats++;
    }

	if (!mMotionPlan.eof()) mMotionPlan.putback(a);

	return repeats;
}