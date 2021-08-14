#include "AutoMapBuilder.hpp"

AutoMapBuilder::AutoMapBuilder(std::unique_ptr<Robot> &robot):
    MapBuilder{robot} {}

char AutoMapBuilder::getNextMotion() {
    std::vector<int> points {getUnvisitedPoints()};

    return getRow();
}