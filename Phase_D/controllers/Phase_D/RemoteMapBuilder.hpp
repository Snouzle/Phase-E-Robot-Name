#ifndef RMAPB_H
#define RMAPB_H

#include "MapBuilder.hpp"

class RemoteMapBuilder: public MapBuilder {
private:
public:
    RemoteMapBuilder(std::unique_ptr<Robot> &robot);
    virtual ~RemoteMapBuilder() {}

    virtual char getNextMotion() override;
    virtual void replan() override;
    virtual int getNumRepeat(const char &letter) override;
};

#endif