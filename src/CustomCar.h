#pragma once

#include <string>
#include "Car.h"

class CustomCar : public Car
{
public:
    CustomCar(const std::string & chassis_obj_file);
    ~CustomCar() = default;

    virtual void configToCarSimulation(CarSimulation * simulation, const btVector3 & init_pos);

protected:
    void setCustomCarChassis(CarSimulation * simulation, const btVector3 & init_pos);
    btIndexedMesh constructIndexMesh(const float * vertices, int vcount, const unsigned int * faces, int fcount);

protected:
    std::string chassis_obj_file;
};