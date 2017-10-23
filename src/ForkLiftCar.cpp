#include "ForkLiftCar.h"
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

void ForkLiftCar::setForkLiftCarChassis(CarSimulation * simulation, const btVector3 & init_pos)
{
    //定义Car的组合体形状
    btCompoundShape* compound = new btCompoundShape();

    //定义Car底盘的形状并添加
    btTransform localTrans;
    localTrans.setIdentity();
    localTrans.setOrigin(btVector3(0, 1, 0));

    btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
    compound->addChildShape(localTrans, chassisShape);

    //定义Car底盘前置形状并添加
    btTransform suppLocalTrans;
    suppLocalTrans.setIdentity();
    suppLocalTrans.setOrigin(btVector3(0, 1.0, 2.5));

    btCollisionShape* suppShape = new btBoxShape(btVector3(0.5f, 0.1f, 0.5f));
    compound->addChildShape(suppLocalTrans, suppShape);

    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(init_pos);
    car_chassis = simulation->localCreateRigidBody(car_mass, tr, compound);

    //never deactivate the vehicle
    car_chassis->setActivationState(DISABLE_DEACTIVATION);
}

void ForkLiftCar::setForkLiftRigidBody(CarSimulation * simulation, const btVector3 & init_pos)
{
    btDiscreteDynamicsWorld * m_dynamicsWorld = simulation->getDynamicsWorld();

    //定义Lift Rigid Body
    btCollisionShape* liftShape = new btBoxShape(btVector3(0.5f, 2.0f, 0.05f));
    //m_collisionShapes.push_back(liftShape);

    lift_start_pos = init_pos + btVector3(0.0f, 2.5f, 3.05f);
    btTransform liftTrans;
    liftTrans.setIdentity();
    liftTrans.setOrigin(lift_start_pos);

    lift_body = simulation->localCreateRigidBody(10, liftTrans, liftShape);

    //定义Lift Constraint
    btTransform localA;
    localA.setIdentity();
    localA.getBasis().setEulerZYX(0, M_PI_2, 0);
    localA.setOrigin(btVector3(0.0, 1.0, 3.05));

    btTransform localB;
    localB.setIdentity();
    localB.getBasis().setEulerZYX(0, M_PI_2, 0);
    localB.setOrigin(btVector3(0.0, -1.5, -0.05));

    lift_hinge = new btHingeConstraint(*car_chassis, *lift_body, localA, localB);
    lift_hinge->setLimit(0.0f, 0.0f);
    m_dynamicsWorld->addConstraint(lift_hinge, true);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //定义Fork Shape Compound
    btCompoundShape* forkCompound = new btCompoundShape();

    //定义Fork Shape A
    btTransform forkLocalTrans;
    forkLocalTrans.setIdentity();

    btCollisionShape* forkShapeA = new btBoxShape(btVector3(1.0f, 0.1f, 0.1f));
    forkCompound->addChildShape(forkLocalTrans, forkShapeA);

    //定义Fork Shape B
    forkLocalTrans.setIdentity();
    forkLocalTrans.setOrigin(btVector3(-0.9f, -0.08f, 0.7f));

    btCollisionShape* forkShapeB = new btBoxShape(btVector3(0.1f, 0.02f, 0.6f));
    forkCompound->addChildShape(forkLocalTrans, forkShapeB);

    //定义Fork Shape C
    forkLocalTrans.setIdentity();
    forkLocalTrans.setOrigin(btVector3(0.9f, -0.08f, 0.7f));

    btCollisionShape* forkShapeC = new btBoxShape(btVector3(0.1f, 0.02f, 0.6f));
    forkCompound->addChildShape(forkLocalTrans, forkShapeC);

    //定义Fork Rigid Body
    btTransform forkTrans;
    fork_start_pos = init_pos + btVector3(0.0f, 0.6f, 3.2f);
    forkTrans.setIdentity();
    forkTrans.setOrigin(fork_start_pos);
    fork_body = simulation->localCreateRigidBody(5, forkTrans, forkCompound);

    //定义Fork Constraint
    localA.setIdentity();
    localA.getBasis().setEulerZYX(0, 0, M_PI_2);
    localA.setOrigin(btVector3(0.0f, -1.9f, 0.05f));

    localB.setIdentity();
    localB.getBasis().setEulerZYX(0, 0, M_PI_2);
    localB.setOrigin(btVector3(0.0, 0.0, -0.1));

    fork_slider = new btSliderConstraint(*lift_body, *fork_body, localA, localB, true);
    fork_slider->setLowerLinLimit(0.1f);
    fork_slider->setUpperLinLimit(0.1f);
    //m_forkSlider->setLowerAngLimit(-LIFT_EPS);
    //m_forkSlider->setUpperAngLimit(LIFT_EPS);
    fork_slider->setLowerAngLimit(0.0f);
    fork_slider->setUpperAngLimit(0.0f);
    m_dynamicsWorld->addConstraint(fork_slider, true);
}

void ForkLiftCar::configToCarSimulation(CarSimulation * simulation, const btVector3 & init_pos)
{
    btDiscreteDynamicsWorld * m_dynamicsWorld = simulation->getDynamicsWorld();
    GUIHelperInterface * m_guiHelper = simulation->getGuiHelper();

    this->setForkLiftCarChassis(simulation, init_pos);
    this->setCarWheelForGraphics(m_guiHelper);

    this->setForkLiftRigidBody(simulation, init_pos);
    
    //创建Car
    vehicle_ray_caster = new btDefaultVehicleRaycaster(m_dynamicsWorld);

    vehicle = new btRaycastVehicle(vehicle_tuning, car_chassis, vehicle_ray_caster);
    vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex); //choose coordinate system

    m_dynamicsWorld->addVehicle(vehicle);

    this->setCarWheelForSimulation();

    //add to CarSimulation
    simulation->addCar(this);
}

bool ForkLiftCar::keyboardCallback(int key, int state, bool is_shift_pressed)
{
    bool handled = false;

    if (state)
    {
        if (is_shift_pressed)
        {
            switch (key)
            {
            case B3G_LEFT_ARROW:
            {

                lift_hinge->setLimit(-M_PI / 16.0f, M_PI / 8.0f);
                lift_hinge->enableAngularMotor(true, -0.1, max_motor_impulse);
                handled = true;
                break;
            }
            case B3G_RIGHT_ARROW:
            {

                lift_hinge->setLimit(-M_PI / 16.0f, M_PI / 8.0f);
                lift_hinge->enableAngularMotor(true, 0.1, max_motor_impulse);
                handled = true;
                break;
            }
            case B3G_UP_ARROW:
            {
                fork_slider->setLowerLinLimit(0.1f);
                fork_slider->setUpperLinLimit(3.9f);
                fork_slider->setPoweredLinMotor(true);
                fork_slider->setMaxLinMotorForce(max_motor_impulse);
                fork_slider->setTargetLinMotorVelocity(1.0);
                handled = true;
                break;
            }
            case B3G_DOWN_ARROW:
            {
                fork_slider->setLowerLinLimit(0.1f);
                fork_slider->setUpperLinLimit(3.9f);
                fork_slider->setPoweredLinMotor(true);
                fork_slider->setMaxLinMotorForce(max_motor_impulse);
                fork_slider->setTargetLinMotorVelocity(-1.0);
                handled = true;
                break;
            }
            default:
                handled = true;
                break;
            }

        }
        else
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
            lockForkSlider();
            engine_force = 0.f;
            breaking_force = default_breaking_force;
            handled = true;
            break;
        }
        case B3G_DOWN_ARROW:
        {
            lockForkSlider();
            engine_force = 0.f;
            breaking_force = default_breaking_force;
            handled = true;
            break;
        }
        case B3G_LEFT_ARROW:
        case B3G_RIGHT_ARROW:
        {
            lockLiftHinge();
            handled = true;
            break;
        }
        default:

            break;
        }
    }
    return handled;
}

void ForkLiftCar::lockLiftHinge()
{
    btScalar hingeAngle = lift_hinge->getHingeAngle();
    btScalar lowLim = lift_hinge->getLowerLimit();
    btScalar hiLim = lift_hinge->getUpperLimit();
    lift_hinge->enableAngularMotor(false, 0, 0);

    if (hingeAngle < lowLim)
    {
        //m_liftHinge->setLimit(lowLim, lowLim + LIFT_EPS);
        lift_hinge->setLimit(lowLim, lowLim);
    }
    else if (hingeAngle > hiLim)
    {
        //m_liftHinge->setLimit(hiLim - LIFT_EPS, hiLim);
        lift_hinge->setLimit(hiLim, hiLim);
    }
    else
    {
        //m_liftHinge->setLimit(hingeAngle - LIFT_EPS, hingeAngle + LIFT_EPS);
        lift_hinge->setLimit(hingeAngle, hingeAngle);
    }

}

void ForkLiftCar::lockForkSlider()
{
    btScalar linDepth = fork_slider->getLinearPos();
    btScalar lowLim = fork_slider->getLowerLinLimit();
    btScalar hiLim = fork_slider->getUpperLinLimit();
    fork_slider->setPoweredLinMotor(false);

    if (linDepth <= lowLim)
    {
        fork_slider->setLowerLinLimit(lowLim);
        fork_slider->setUpperLinLimit(lowLim);
    }
    else if (linDepth > hiLim)
    {
        fork_slider->setLowerLinLimit(hiLim);
        fork_slider->setUpperLinLimit(hiLim);
    }
    else
    {
        fork_slider->setLowerLinLimit(linDepth);
        fork_slider->setUpperLinLimit(linDepth);
    }

}
