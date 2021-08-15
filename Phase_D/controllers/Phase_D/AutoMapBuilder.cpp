#include "AutoMapBuilder.hpp"

AutoMapBuilder::AutoMapBuilder(std::unique_ptr<Robot> &robot):
    MapBuilder{robot} {}

char AutoMapBuilder::getNextMotion() {
    if (getUnvisitedPoints().empty()) return '\0';

    char move;
    mMotionPlan >> move;

    if (mMotionPlan.eof()) {
        move = moveAdjacentSquare();
        std::cout << move << std::endl; 
        if (move == '\0')  {
            int currPoint {(getRow()-getMinRow()) * ROW + (getCol()-getMinCol())};
            planPath(currPoint);
            mMotionPlan >> move;
            runningPlan = true;
        }
        else {
            runningPlan = false;
            return moveAdjacentSquare();
        }
    }

    return move;
}

void AutoMapBuilder::planPath(const int &currPoint, const int &obstacle) {
    std::vector<int> points {getUnvisitedPoints()};
    std::vector<int> transformedPoints;

    writeStringMap(BUILD_OUT_FILE);

    for (auto &point : points) {
        int row {point / (COLS)};
        int col {point % (COLS)};
        int tPoint {(row-getMinRow()) * ROW + col-getMinCol()};
        transformedPoints.push_back(tPoint);
    }

    std::string newPlan = getPath(currPoint, obstacle, getHeading(), BUILD_OUT_FILE, transformedPoints);
    mMotionPlan.str(newPlan.substr(3));
    mMotionPlan.clear();

    std::cout << newPlan << std::endl;
}

char AutoMapBuilder::moveAdjacentSquare() {
    auto adjacentSquares {findAdjacentUnvisited()};
    int heading{getHeadingDirection()};

    if (adjacentSquares[heading]) return 'F';
    else if (adjacentSquares[(3 + heading) % NUM_DIRECTIONS]) return 'L';
    else if (adjacentSquares[(heading + 1) % NUM_DIRECTIONS]) return 'R';
    else if (adjacentSquares[(heading + 2) % NUM_DIRECTIONS]) return 'L';
    else return '\0';
}

void AutoMapBuilder::replan() {
    std::pair<int, int> posPair{getPreviousPosition()};
    int row{getRow()}, col{getCol()};
	
    if (runningPlan) {
        int pathPos = (posPair.first-getMinRow()) * ROW + posPair.second-getMinCol();
        int obstacle = (row-getMinRow()) * ROW + col-getMinCol();
        setRow(posPair.first);
	    setCol(posPair.second);
        planPath(pathPos, obstacle);
    } else {
        int pos{row*COLS + col};
        updateMapPosition(row, col);
        deleteUnvisited(pos);
        setRow(posPair.first);
	    setCol(posPair.second);
    }
}

int AutoMapBuilder::getNumRepeat(const char &letter) {
    if (runningPlan) {
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

    return 1;
}