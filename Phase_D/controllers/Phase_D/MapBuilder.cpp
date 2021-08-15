#include "MapBuilder.hpp"

MapBuilder::MapBuilder(std::unique_ptr<Robot> &robot):
    MotionStrategy(robot) {
    double imu {getIMU()[2]};

    int heading {3 - ((int)(std::round(imu / (PI/2)) + 2) % 4)};
    setHeading((Heading)heading); 
    setRow(4);
    setCol(8);

    processState();
}

void MapBuilder::showMap() {
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

bool MapBuilder::isVisited(const int &row, const int &col) {
    return mVisitedPoints[row*COLS + col];
}

void MapBuilder::deleteUnvisited(const int &pos) {
    bool found{false};
    auto iter{mUnvisitedPoints.begin()};

    mVisitedPoints[pos] = true;

    while ((iter != mUnvisitedPoints.end()) && !found) {
        if (*iter == pos) found=true;
        else iter++;
    }
    if (found) mUnvisitedPoints.erase(iter);
}

void MapBuilder::visit(const int &row, const int &col) {
    int pos{row*COLS + col};

    deleteUnvisited(pos);

    const std::array<int, NUM_DIRECTIONS> neighbours{
        pos-COLS, pos-1, pos+1, pos+COLS
    };

    for (const auto &neighbour : neighbours) {
        updateVisit(neighbour);
    }
}

void MapBuilder::updateVisit(const int &pos) {
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

void MapBuilder::addUnvisited(const int &row, const int &col) {
    int pos{row*COLS + col};
    
    if (std::find(mUnvisitedPoints.begin(), mUnvisitedPoints.end(), pos) == mUnvisitedPoints.end())
        mUnvisitedPoints.push_back(row*COLS+col);
}

void MapBuilder::updateMapPosition(const int &row, const int &col) {
    if (row < minRow) minRow = row;
    else if (row > maxRow) maxRow = row;
    if (col < minCol) minCol = col;
    else if (col > maxCol) maxCol = col;
}

void MapBuilder::processState() {
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

    showMap();
}

std::array<bool, NUM_DIRECTIONS> MapBuilder::findAdjacentUnvisited() {
    int strCol{getCol() * 4 + 2}, 
        strRow{getRow() * 2 + 1};

    int pos{getRow() * COLS + getCol()};

    const std::array<int, NUM_DIRECTIONS> neighbours{
        pos-COLS, pos+1, pos+COLS, pos-1,
    };

    const std::array<int, NUM_DIRECTIONS> walls {
        mStringMap[strRow - 1][strCol], mStringMap[strRow][strCol + 2],
        mStringMap[strRow + 1][strCol], mStringMap[strRow][strCol - 2]
    };

    std::array<bool, NUM_DIRECTIONS> adjacent;

    for (int i = 0; i < NUM_DIRECTIONS; i++) {
        if (walls[i] == ' ') {
            auto iter{std::find(begin(mUnvisitedPoints), end(mUnvisitedPoints), neighbours[i])};
            adjacent[i] = (iter != mUnvisitedPoints.end());
        } else adjacent[i] = false;
    }

    return adjacent;
}

void MapBuilder::writeStringMap(const std::string &fileName) {
    std::ofstream fd{fileName, std::ios_base::out | std::ios_base::trunc};

    int minStrCol{minCol * 4}, maxStrCol{maxCol * 4 + 4},
        minStrRow{minRow * 2}, maxStrRow{maxRow * 2 + 2},
        width{maxStrCol - minStrCol + 1};

    for (int i = 0, row=minStrRow; i <= 10; i++, row++) {
        std::string line;
        if (i == 0 || i == 10) line = " --- --- --- --- --- --- --- --- --- ";
        else {
            if (i % 2 == 0) line = "                                     ";
            else line = "|                                   |";
            line.replace(1, width-1, mStringMap[row].substr(minStrCol+1, width-1));
        }
        fd << line << "\n";
    }

    fd.close();
}