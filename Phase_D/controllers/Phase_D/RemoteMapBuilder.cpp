#include "RemoteMapBuilder.hpp"

RemoteMapBuilder::RemoteMapBuilder(std::unique_ptr<Robot> &robot):
    MotionStrategy(robot) {
    double imu {getIMU()[2]};

    int heading {3 - ((int)(std::round(imu / (PI/2)) + 2) % 4)};
    setHeading((Heading)heading); 
    setRow(4);
    setCol(8);

    processState();
}

void RemoteMapBuilder::showMap() {
    int minStrCol{minCol * 4}, maxStrCol{maxCol * 4 + 4},
        minStrRow{minRow * 2}, maxStrRow{maxRow * 2 + 2};

    auto map = mStringMap;

    for (auto &i : mUnvisitedPoints) {
        int row{i/COLS}, col{i%COLS};
        int strRow{row*2+1}, strCol{col*4+2};
        map[strRow][strCol] = '*';
    }

    int botRow{getRow()*2+1}, botCol{getCol()*4+2};

    const std::array<char, NUM_DIRECTIONS> directions {
        '^', '>', 'v', '<'
    };
    map[botRow][botCol] = directions[getHeadingDirection()];

    for (int row{minStrRow}; row <= maxStrRow; row++) {
        std::string mapLine{map[row].substr(minStrCol, maxStrCol-minStrCol + 1)};
        std::cout << mapLine << std::endl;
    }
}

bool RemoteMapBuilder::isVisited(const int &row, const int &col) {
    return mVisitedPoints[row*COLS + col];
}

void RemoteMapBuilder::deleteUnvisited(const int &pos) {
    bool found{false};
    auto iter{mUnvisitedPoints.begin()};

    mVisitedPoints[pos] = true;

    while ((iter != mUnvisitedPoints.end()) && !found) {
        if (*iter == pos) found=true;
        else iter++;
    }
    if (found) mUnvisitedPoints.erase(iter);
}

void RemoteMapBuilder::visit(const int &row, const int &col) {
    int pos{row*COLS + col};

    deleteUnvisited(pos);

    const std::array<int, NUM_DIRECTIONS> neighbours{
        pos-COLS, pos-1, pos+1, pos+COLS
    };

    for (const auto &neighbour : neighbours) {
        updateVisit(neighbour);
    }
}

void RemoteMapBuilder::updateVisit(const int &pos) {
    if (pos < 0 || pos >= ROWS * COLS) return;

    const std::array<int, NUM_DIRECTIONS> neighbours{
        pos-COLS, pos-1, pos+1, pos+COLS
    };

    for (const auto &neighbour : neighbours) {
        if (neighbour < 0 || neighbour >= ROWS*COLS ||
            !mVisitedPoints[neighbour]) return;
    }
    
    deleteUnvisited(pos);
}

void RemoteMapBuilder::addUnvisited(const int &row, const int &col) {
    int pos{row*COLS + col};
    
    if (std::find(mUnvisitedPoints.begin(), mUnvisitedPoints.end(), pos) == mUnvisitedPoints.end())
        mUnvisitedPoints.push_back(row*COLS+col);
}

void RemoteMapBuilder::updateMapPosition(const int &row, const int &col) {
    if (row < minRow) minRow = row;
    else if (row > maxRow) maxRow = row;
    if (col < minCol) minCol = col;
    else if (col > maxCol) maxCol = col;
}

void RemoteMapBuilder::processState() {
    int row{getRow()}, col{getCol()},
        strRow{row * 2 + 1}, strCol{col * 4 + 2};
        
    updateMapPosition(row, col);

    if (!isVisited(row, col))
        visit(row, col);

    for (int i = 0; i < NUM_DIRECTIONS; i++) {
        int tempHeading {(i + getHeadingDirection() - 1)}, 
            unvisitedRow{row}, unvisitedCol{col};
        if (tempHeading < 0) tempHeading += 4;
        tempHeading = tempHeading % 4;

        if (tempHeading % 2 == 0) unvisitedRow += (tempHeading-1);
        else unvisitedCol += (2-tempHeading);
        if (!isWall(i)) {
            if (!isVisited(unvisitedRow, unvisitedCol)) {
                updateMapPosition(unvisitedRow, unvisitedCol);
                addUnvisited(unvisitedRow, unvisitedCol);
            }
        } else {
            if (tempHeading % 2 == 0) {
                std::string tmp{mStringMap[strRow+tempHeading-1]};
                tmp.replace(strCol-1, 3, "---");
                mStringMap[strRow+tempHeading-1] = tmp;
            } else {
                mStringMap[strRow][strCol + 4 - 2 * tempHeading] = '|';
            }
        }
    }

}

char RemoteMapBuilder::getNextMotion() {
    Keyboard *keyboard = getKeyboard();
	int key {keyboard->getKey()};

    while (step() != -1) {
        if (key == 'W' && !isWall((int)Wall::FRONT)) return 'F';
        else if (key == 'A') return 'L';
        else if (key == 'D') return 'R';
        else if (key == 'M') showMap();
        else if (key == Keyboard::CONTROL+'D') return '\0';
        key = keyboard->getKey();
    }

    return '\0';
}

