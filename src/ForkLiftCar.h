#pragma once

#include "Car.h"

class ForkLiftCar : public Car
{
public:
    ForkLiftCar() = default;
    virtual ~ForkLiftCar() = default;

    virtual void configToCarSimulation(CarSimulation * simulation, const btVector3 & init_pos);
    virtual bool keyboardCallback(int key, int state, bool is_shift_pressed);

    void lockLiftHinge();
    void lockForkSlider();

protected:
    void setForkLiftCarChassis(CarSimulation * simulation, const btVector3 & init_pos);
    void setForkLiftRigidBody(CarSimulation * simulation, const btVector3 & init_pos);

protected:
    btRigidBody * lift_body = nullptr;
    btVector3	lift_start_pos;
    btHingeConstraint* lift_hinge = nullptr;

    btRigidBody * fork_body = nullptr;
    btVector3	fork_start_pos;
    btSliderConstraint* fork_slider = nullptr;
};
