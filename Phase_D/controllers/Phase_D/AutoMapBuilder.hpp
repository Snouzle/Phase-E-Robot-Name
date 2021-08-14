#ifndef AMAPB_H
#define AMAPB_H

#include "MapBuilder.hpp"

class AutoMapBuilder: public MapBuilder {
private:
public:
    AutoMapBuilder(std::unique_ptr<Robot> &robot);

    virtual char getNextMotion() override;
};

#endif