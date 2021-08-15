#include "RemoteMapBuilder.hpp"

RemoteMapBuilder::RemoteMapBuilder(std::unique_ptr<Robot> &robot):
    MapBuilder{robot} {}


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

void RemoteMapBuilder::replan() {
    std::pair<int, int> posPair{getPreviousPosition()};
    int row{getRow()}, col{getCol()};
    int pos{row*COLS + col};

    updateMapPosition(row, col);

    deleteUnvisited(pos);

	setRow(posPair.first);
	setCol(posPair.second);
}

int RemoteMapBuilder::getNumRepeat(const char &letter) { return 1; }