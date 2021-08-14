#ifndef MAPB_H
#define MAPB_H

#include "MotionStrategy.hpp"
#include <sstream>
#include <fstream>
#include <vector>
#include <array>
#include <cmath>
#include <map>
#include <string>
#include <algorithm>
#include <iterator>

const std::string REMOTE_MAP_FILE_NAME {"../../RemoteMap.txt"};


// All the webots classes are defined in the "webots" namespace
using namespace webots;

// class PointList {
// private:
//     std::vector<int> points;
//     // std::array<bool, DISTANCE_SENSOR_NUMBER+1> mAdjacency {
//     //     false, false, false, false
//     // };

// public:
//     PointList() {}

//     void setAdjacent(const int &i) { 
//         // if (i >= 0 && i < NUM_DIRECTIONS)
//         //     mAdjacency[i] = true;
//     }

//     bool isAdjacent(const int &i) {
//         // return (i >= 0 && i < NUM_DIRECTIONS && (mAdjacency[i] == true));
//     }

//     void visit() {}
//     std::vector<int>::iterator isVisited(const int &x, const int &y); 
// };

// Forward Declaration
class MapBuilder: public MotionStrategy {
public:
    enum {
        ROWS=9,
        COLS=17
    };

private:
	// std::array<std::array<Point, 17>, 9> mMap;
    // std::map<int, int>
    std::vector<int> mUnvisitedPoints;
    std::array<bool, ROWS * COLS> mVisitedPoints{false};
    std::vector<std::string> mStringMap{
        2*ROWS + 3, std::string(4*COLS+5, ' ')
    };
     
    int minCol{8}, maxCol{8}, minRow{4}, maxRow{4};

    bool isVisited(const int &row, const int &col); 
    void deleteUnvisited(const int &pos);
    void visit(const int &row, const int &col);
    void updateVisit(const int &pos);
    void addUnvisited(const int &row, const int &col);
    void updateMapPosition(const int &row, const int &col);

public:
	MapBuilder(std::unique_ptr<Robot> &robot);
	virtual ~MapBuilder() {}

	virtual void processState() override;

    void showMap();
    std::vector<int> getUnvisitedPoints() { return mUnvisitedPoints; }
};

#endif