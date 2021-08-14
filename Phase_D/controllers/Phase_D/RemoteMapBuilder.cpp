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