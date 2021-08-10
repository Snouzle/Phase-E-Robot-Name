#ifndef RMAPB_H
#define RMAPB_H

#include "MotionStrategy.hpp"
#include <sstream>
#include <fstream>

const std::string REMOTE_MAP_FILE_NAME {"../../RemoteMap.txt"};


// All the webots classes are defined in the "webots" namespace
using namespace webots;

// Forward Declaration
class RemoteMapBuilder: public MotionStrategy {
private:
	

public:
	RemoteMapBuilder(std::unique_ptr<Robot> &robot);
	virtual ~RemoteMapBuilder() {}

	virtual void processState() override;
	virtual char getNextMotion() override;
};

#endif