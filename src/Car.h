#pragma once

#include <vector>
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

class GUIHelperInterface;
class CarSimulation;

class Car
{
public:
    Car();
    virtual ~Car();

    Car(const Car &) = delete;
    Car & operator = (const Car &) = delete;

    virtual void configToCarSimulation(CarSimulation * simulation, const btVector3 & init_pos);
    virtual bool keyboardCallback(int key, int state, bool is_shift_pressed);

    void update();
    void updateWheelTransformForRender(CarSimulation * simulation);
    std::vector<btTransform> getCarTransform() const;

    void setFrontWheelXOffset(float x_offset);
    void setFrontWheelZOffset(float z_offset);
    void setBackWheelXOffset(float x_offset);
    void setBackWheelZOffset(float z_offset);

    void setCarChassisTransform(const btTransform & chassis_transform);
    void setCarChassisBoxShape(const btVector3 & half_extents);
    void setCarChassisCustomShape(btCollisionShape * shape);

protected:
    void setCarChassis(CarSimulation * simulation, const btVector3 & init_pos);
    void setCarWheelForGraphics(GUIHelperInterface * m_guiHelper);
    void setCarWheelForSimulation();

protected:
    btRigidBody * car_chassis = nullptr;
    btCollisionShape * car_chassis_shape = nullptr;
    bool is_default_chassis_shape = true;

    btTransform car_chassis_transform; //default: <0, 1, 0>

    btCollisionShape * wheel_shape = nullptr;
    int wheel_instances[4] = { -1, -1, -1, -1 };

    float front_wheel_x_offset = 0.f; //default: x_half_width_of_chassis - (0.3*wheel_width)
    float front_wheel_z_offset = 0.f; //default: z_half_width_of_chassis - wheel_radius
    float back_wheel_x_offset = 0.f;  //default: x_half_width_of_chassis - (0.3*wheel_width)
    float back_wheel_z_offset = 0.f;  //default: z_half_width_of_chassis - wheel_radius

    btRaycastVehicle::btVehicleTuning vehicle_tuning;
    btVehicleRaycaster * vehicle_ray_caster = nullptr;
    btRaycastVehicle * vehicle = nullptr;

    float car_mass = 800.f;

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