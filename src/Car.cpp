#include <iostream>
#include "Car.h"
#include "CarSimulation.h"

//===============================================================================================

#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923
#define M_PI_4     0.785398163397448309616

#define CUBE_HALF_EXTENTS 1

static int rightIndex = 0;
static int upIndex = 1;
static int forwardIndex = 2;

static btVector3 wheelDirectionCS0(0, -1, 0);
static btVector3 wheelAxleCS(-1, 0, 0);

//===============================================================================================

void Car::setCarChassis(CarSimulation * simulation, const btVector3 & init_pos)
{
    //定义Car的组合体形状
    btCompoundShape* compound = new btCompoundShape();

    //定义Car底盘的形状并添加
    btTransform localTrans;
    localTrans.setIdentity();
    localTrans.setOrigin(btVector3(0, 1, 0));

    car_chassis_shape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
    compound->addChildShape(localTrans, car_chassis_shape);

    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(init_pos);
    car_chassis = simulation->localCreateRigidBody(800, tr, compound);

    car_chassis->setDamping(0.2, 0.2);
    car_chassis->setActivationState(DISABLE_DEACTIVATION); //never deactivate the vehicle
}

void Car::setCarWheelForGraphics(GUIHelperInterface * m_guiHelper)
{
    //定义Car轮胎形状
    wheel_shape = new btCylinderShapeX(btVector3(wheel_width, wheel_radius, wheel_radius));
    m_guiHelper->createCollisionShapeGraphicsObject(wheel_shape);

    const float position[4] = { 0, 0, 0, 0 };
    const float quaternion[4] = { 0, 0, 0, 1 };
    const float color[4] = { 0, 1, 0, 1 };
    const float scaling[4] = { 1, 1, 1, 1 };

    int wheelGraphicsIndex = wheel_shape->getUserIndex();
    for (int i = 0; i < 4; i++)
        wheel_instances[i] = m_guiHelper->registerGraphicsInstance(wheelGraphicsIndex, position, quaternion, color, scaling);
}

void Car::setCarWheelForSimulation()
{
    btVector3 aabb_min, aabb_max;
    car_chassis->getAabb(aabb_min, aabb_max);

    float x_half_width = (aabb_max[0] - aabb_min[0]) / 2;
    float z_half_width = (aabb_max[2] - aabb_min[2]) / 2;

    bool isFrontWheel = true;
    btVector3 connectionPointCS0(x_half_width - (0.3*wheel_width), wheel_height, z_half_width - wheel_radius);
    vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspension_rest_length, wheel_radius, vehicle_tuning, isFrontWheel);

    connectionPointCS0 = btVector3(-x_half_width + (0.3*wheel_width), wheel_height, z_half_width - wheel_radius);
    vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspension_rest_length, wheel_radius, vehicle_tuning, isFrontWheel);

    isFrontWheel = false;
    connectionPointCS0 = btVector3(x_half_width + (0.3*wheel_width), wheel_height, -z_half_width + wheel_radius);
    vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspension_rest_length, wheel_radius, vehicle_tuning, isFrontWheel);

    connectionPointCS0 = btVector3(-x_half_width - (0.3*wheel_width), wheel_height, -z_half_width + wheel_radius);
    vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspension_rest_length, wheel_radius, vehicle_tuning, isFrontWheel);

    for (int i = 0; i < vehicle->getNumWheels(); i++)
    {
        btWheelInfo & wheel = vehicle->getWheelInfo(i);
        wheel.m_suspensionStiffness = suspension_stiffness;
        wheel.m_wheelsDampingRelaxation = suspension_damping;
        wheel.m_wheelsDampingCompression = suspension_compression;
        wheel.m_frictionSlip = wheel_friction;
        wheel.m_rollInfluence = roll_influence;
    }
}

void Car::configToCarSimulation(CarSimulation * simulation, const btVector3 & init_pos)
{
    btDiscreteDynamicsWorld * m_dynamicsWorld = simulation->getDynamicsWorld();
    GUIHelperInterface * m_guiHelper = simulation->getGuiHelper();

    this->setCarChassis(simulation, init_pos);
    this->setCarWheelForGraphics(m_guiHelper);

    //创建Car
    vehicle_ray_caster = new btDefaultVehicleRaycaster(m_dynamicsWorld);

    vehicle = new btRaycastVehicle(vehicle_tuning, car_chassis, vehicle_ray_caster);
    vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex); //choose coordinate system

    m_dynamicsWorld->addVehicle(vehicle);

    this->setCarWheelForSimulation();

    //add to CarSimulation
    simulation->addCar(this);
}

void Car::update()
{
    int wheelIndex = 2;

    vehicle->applyEngineForce(engine_force, wheelIndex);
    vehicle->setBrake(breaking_force, wheelIndex);

    wheelIndex = 3;
    vehicle->applyEngineForce(engine_force, wheelIndex);
    vehicle->setBrake(breaking_force, wheelIndex);

    wheelIndex = 0;
    vehicle->setSteeringValue(vehicle_steering, wheelIndex);

    wheelIndex = 1;
    vehicle->setSteeringValue(vehicle_steering, wheelIndex);
}

void Car::updateWheelTransformForRender(CarSimulation * simulation)
{
    GUIHelperInterface * m_guiHelper = simulation->getGuiHelper();

    for (int i = 0; i < vehicle->getNumWheels(); i++)
    {
        //synchronize the wheels with the (interpolated) chassis world transform
        vehicle->updateWheelTransform(i, true);

        CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
        if (renderer)
        {
            btTransform tr = vehicle->getWheelInfo(i).m_worldTransform;
            btVector3 pos = tr.getOrigin();
            btQuaternion orn = tr.getRotation();
            renderer->writeSingleInstanceTransformToCPU(pos, orn, wheel_instances[i]);
        }
    }
}

std::vector<btTransform> Car::getCarTransform() const
{
    std::vector<btTransform> trans;
    
    // chassis transform
    btTransform chassis_tran;
    this->car_chassis->getMotionState()->getWorldTransform(chassis_tran);
    trans.push_back(chassis_tran);

    //wheel transforms
    for (int i = 0; i < vehicle->getNumWheels(); i++)
    {
        //synchronize the wheels with the (interpolated) chassis world transform
        vehicle->updateWheelTransform(i, true);
        btTransform tr = vehicle->getWheelInfo(i).m_worldTransform;
        trans.push_back(tr);
    }

    return trans;
}

bool Car::keyboardCallback(int key, int state, bool is_shift_pressed)
{
    bool handled = false;

    if (state)
    {
        if(is_shift_pressed == false)
        {
            switch (key)
            {
            case B3G_LEFT_ARROW:
            {
                handled = true;
                vehicle_steering += steering_increment;
                if (vehicle_steering > steering_clamp)
                    vehicle_steering = steering_clamp;

                break;
            }
            case B3G_RIGHT_ARROW:
            {
                handled = true;
                vehicle_steering -= steering_increment;
                if (vehicle_steering < -steering_clamp)
                    vehicle_steering = -steering_clamp;

                break;
            }
            case B3G_UP_ARROW:
            {
                handled = true;
                engine_force = max_engine_force;
                breaking_force = 0.f;
                break;
            }
            case B3G_DOWN_ARROW:
            {
                handled = true;
                engine_force = -max_engine_force;
                breaking_force = 0.f;
                break;
            }

            default:
                break;
            }   
        }
    }
    else
    {
        switch (key)
        {
        case B3G_UP_ARROW:
        {
            engine_force = 0.f;
            breaking_force = default_breaking_force;
            handled = true;
            break;
        }
        case B3G_DOWN_ARROW:
        {
            engine_force = 0.f;
            breaking_force = default_breaking_force;
            handled = true;
            break;
        }
        case B3G_LEFT_ARROW:
        case B3G_RIGHT_ARROW:
        {
            handled = true;
            break;
        }
        default:

            break;
        }
    }
    return handled;
}
