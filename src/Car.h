#pragma once

#include <vector>
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

class GUIHelperInterface;
class CarSimulation;

class Car
{
public:
    Car() = default;
    virtual ~Car() = default;

    virtual void configToCarSimulation(CarSimulation * simulation, const btVector3 & init_pos);
    virtual bool keyboardCallback(int key, int state, bool is_shift_pressed);

    void update();
    void updateWheelTransformForRender(CarSimulation * simulation);
    std::vector<btTransform> getCarTransform() const;

protected:
    void setCarChassis(CarSimulation * simulation, const btVector3 & init_pos);
    void setCarWheelForGraphics(GUIHelperInterface * m_guiHelper);
    void setCarWheelForSimulation();

protected:
    btRigidBody * car_chassis = nullptr;
    btCollisionShape * car_chassis_shape = nullptr;

    btCollisionShape * wheel_shape = nullptr;
    int wheel_instances[4] = { -1, -1, -1, -1 };

    btRaycastVehicle::btVehicleTuning vehicle_tuning;
    btVehicleRaycaster * vehicle_ray_caster = nullptr;
    btRaycastVehicle * vehicle = nullptr;

    float max_motor_impulse = 4000.f;
    float engine_force = 0.f;
    float max_engine_force = 2000.f;

    float default_breaking_force = 10.f; 
    float breaking_force = 100.f;
    float max_breaking_force = 100.f;

    float vehicle_steering = 0.f;
    float steering_increment = 0.08f;
    float steering_clamp = 0.6f;

    float wheel_radius = 0.5f;
    float wheel_width = 0.4f;
    float wheel_height = 1.0f;
    float wheel_friction = 1000;

    float suspension_stiffness = 20.f;
    float suspension_damping = 2.3f;
    float suspension_compression = 4.4f;
    float suspension_rest_length = 0.6f;

    float roll_influence = 0.1f;
    
};