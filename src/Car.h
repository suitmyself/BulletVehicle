#pragma once

#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

class CarSimulation;

class Car
{
public:
    Car();
    ~Car();

    void configToCarSimulation(CarSimulation * simulation, const btVector3 & init_pos);

    void update();
    void updateWheelTransformForRender(CarSimulation * simulation);
    bool keyboardCallback(int key, int state, bool is_shift_pressed);

    void lockLiftHinge();
    void lockForkSlider();

private:
    btRigidBody * m_carChassis = nullptr;
    int m_wheelInstances[4] = { -1, -1, -1, -1 };

    btRigidBody * m_liftBody = nullptr;
    btVector3	m_liftStartPos;
    btHingeConstraint* m_liftHinge = nullptr;

    btRigidBody * m_forkBody = nullptr;
    btVector3	m_forkStartPos;
    btSliderConstraint* m_forkSlider = nullptr;

    btRaycastVehicle::btVehicleTuning m_tuning;
    btVehicleRaycaster * m_vehicleRayCaster = nullptr;
    btRaycastVehicle * m_vehicle = nullptr;
    btCollisionShape * m_wheelShape = nullptr;

    btScalar maxMotorImpulse = 4000.f;
    float gEngineForce = 0.f;

    float defaultBreakingForce = 10.f;
    float gBreakingForce = 100.f;

    float maxEngineForce = 1000.f;
    float maxBreakingForce = 100.f;

    float gVehicleSteering = 0.f;
    float steeringIncrement = 0.08f;
    float steeringClamp = 0.6f;
    float wheelRadius = 0.5f;
    float wheelWidth = 0.4f;
    float wheelFriction = 1000;
    float suspensionStiffness = 20.f;
    float suspensionDamping = 2.3f;
    float suspensionCompression = 4.4f;
    float rollInfluence = 0.1f;

    btScalar suspensionRestLength =0.6f;
};