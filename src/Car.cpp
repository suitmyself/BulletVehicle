#include "Car.h"
#include "CarSimulation.h"

#define M_PI       3.14159265358979323846
#define M_PI_2     1.57079632679489661923
#define M_PI_4     0.785398163397448309616

#define CUBE_HALF_EXTENTS 1

static int rightIndex = 0;
static int upIndex = 1;
static int forwardIndex = 2;

static btVector3 wheelDirectionCS0(0, -1, 0);
static btVector3 wheelAxleCS(-1, 0, 0);

Car::Car()
{
}

Car::~Car()
{
}

void Car::configToCarSimulation(CarSimulation * simulation, const btVector3 & init_pos)
{
    btDiscreteDynamicsWorld * m_dynamicsWorld = simulation->getDynamicsWorld();
    GUIHelperInterface * m_guiHelper = simulation->getGuiHelper();

    //定义Car的组合体形状
    btCompoundShape* compound = new btCompoundShape();
    //m_collisionShapes.push_back(compound);

    //定义Car底盘的形状并添加
    btTransform localTrans;
    localTrans.setIdentity();
    localTrans.setOrigin(btVector3(0, 1, 0));

    btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
    compound->addChildShape(localTrans, chassisShape);

    //m_collisionShapes.push_back(chassisShape);

    //定义Car底盘前置形状并添加
    btTransform suppLocalTrans;
    suppLocalTrans.setIdentity();
    suppLocalTrans.setOrigin(btVector3(0, 1.0, 2.5));

    btCollisionShape* suppShape = new btBoxShape(btVector3(0.5f, 0.1f, 0.5f));
    compound->addChildShape(suppLocalTrans, suppShape);


    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(init_pos);
    m_carChassis = simulation->localCreateRigidBody(800, tr, compound);

    //m_carChassis = localCreateRigidBody(800, tr, compound,chassisShape);
    //m_carChassis->setDamping(0.2,0.2);

    //定义Car轮胎
    const float position[4] = { 0, 10, 10, 0 };
    const float quaternion[4] = { 0, 0, 0, 1 };
    const float color[4] = { 0, 1, 0, 1 };
    const float scaling[4] = { 1, 1, 1, 1 };

    m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));

    m_guiHelper->createCollisionShapeGraphicsObject(m_wheelShape);
    int wheelGraphicsIndex = m_wheelShape->getUserIndex();

    for (int i = 0; i < 4; i++)
        m_wheelInstances[i] = m_guiHelper->registerGraphicsInstance(wheelGraphicsIndex, position, quaternion, color, scaling);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //定义Lift Rigid Body
    btCollisionShape* liftShape = new btBoxShape(btVector3(0.5f, 2.0f, 0.05f));
    //m_collisionShapes.push_back(liftShape);

    m_liftStartPos = init_pos + btVector3(0.0f, 2.5f, 3.05f);
    btTransform liftTrans;
    liftTrans.setIdentity();
    liftTrans.setOrigin(m_liftStartPos);

    m_liftBody = simulation->localCreateRigidBody(10, liftTrans, liftShape);

    //定义Lift Constraint
    btTransform localA;
    localA.setIdentity();
    localA.getBasis().setEulerZYX(0, M_PI_2, 0);
    localA.setOrigin(btVector3(0.0, 1.0, 3.05));

    btTransform localB;
    localB.setIdentity();
    localB.getBasis().setEulerZYX(0, M_PI_2, 0);
    localB.setOrigin(btVector3(0.0, -1.5, -0.05));

    m_liftHinge = new btHingeConstraint(*m_carChassis, *m_liftBody, localA, localB);
    m_liftHinge->setLimit(0.0f, 0.0f);
    m_dynamicsWorld->addConstraint(m_liftHinge, true);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //定义Fork Shape Compound
    btCompoundShape* forkCompound = new btCompoundShape();
    //m_collisionShapes.push_back(forkCompound);

    //定义Fork Shape A
    btTransform forkLocalTrans;
    forkLocalTrans.setIdentity();

    btCollisionShape* forkShapeA = new btBoxShape(btVector3(1.0f, 0.1f, 0.1f));
    forkCompound->addChildShape(forkLocalTrans, forkShapeA);
    //m_collisionShapes.push_back(forkShapeA);

    //定义Fork Shape B
    forkLocalTrans.setIdentity();
    forkLocalTrans.setOrigin(btVector3(-0.9f, -0.08f, 0.7f));

    btCollisionShape* forkShapeB = new btBoxShape(btVector3(0.1f, 0.02f, 0.6f));
    forkCompound->addChildShape(forkLocalTrans, forkShapeB);
    //m_collisionShapes.push_back(forkShapeB);

    //定义Fork Shape C
    forkLocalTrans.setIdentity();
    forkLocalTrans.setOrigin(btVector3(0.9f, -0.08f, 0.7f));

    btCollisionShape* forkShapeC = new btBoxShape(btVector3(0.1f, 0.02f, 0.6f));
    forkCompound->addChildShape(forkLocalTrans, forkShapeC);
    //m_collisionShapes.push_back(forkShapeC);

    //定义Fork Rigid Body
    btTransform forkTrans;
    m_forkStartPos = init_pos + btVector3(0.0f, 0.6f, 3.2f);
    forkTrans.setIdentity();
    forkTrans.setOrigin(m_forkStartPos);
    m_forkBody = simulation->localCreateRigidBody(5, forkTrans, forkCompound);

    //定义Fork Constraint
    localA.setIdentity();
    localA.getBasis().setEulerZYX(0, 0, M_PI_2);
    localA.setOrigin(btVector3(0.0f, -1.9f, 0.05f));

    localB.setIdentity();
    localB.getBasis().setEulerZYX(0, 0, M_PI_2);
    localB.setOrigin(btVector3(0.0, 0.0, -0.1));

    m_forkSlider = new btSliderConstraint(*m_liftBody, *m_forkBody, localA, localB, true);
    m_forkSlider->setLowerLinLimit(0.1f);
    m_forkSlider->setUpperLinLimit(0.1f);
    //m_forkSlider->setLowerAngLimit(-LIFT_EPS);
    //m_forkSlider->setUpperAngLimit(LIFT_EPS);
    m_forkSlider->setLowerAngLimit(0.0f);
    m_forkSlider->setUpperAngLimit(0.0f);
    m_dynamicsWorld->addConstraint(m_forkSlider, true);


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //创建Car

    ///never deactivate the vehicle
    m_carChassis->setActivationState(DISABLE_DEACTIVATION);

    m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
    m_vehicle = new btRaycastVehicle(m_tuning, m_carChassis, m_vehicleRayCaster);

    //choose coordinate system
    m_vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);

    m_dynamicsWorld->addVehicle(m_vehicle);

    float connectionHeight = 1.2f;
    btVector3 connectionPointCS0(CUBE_HALF_EXTENTS - (0.3*wheelWidth), connectionHeight, 2 * CUBE_HALF_EXTENTS - wheelRadius);

    bool isFrontWheel = true;
    m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
    connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (0.3*wheelWidth), connectionHeight, 2 * CUBE_HALF_EXTENTS - wheelRadius);
    m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
    connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (0.3*wheelWidth), connectionHeight, -2 * CUBE_HALF_EXTENTS + wheelRadius);

    isFrontWheel = false;
    m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
    connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS - (0.3*wheelWidth), connectionHeight, -2 * CUBE_HALF_EXTENTS + wheelRadius);
    m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

    for (int i = 0; i < m_vehicle->getNumWheels(); i++)
    {
        btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
        wheel.m_suspensionStiffness = suspensionStiffness;
        wheel.m_wheelsDampingRelaxation = suspensionDamping;
        wheel.m_wheelsDampingCompression = suspensionCompression;
        wheel.m_frictionSlip = wheelFriction;
        wheel.m_rollInfluence = rollInfluence;
    }

    simulation->addCar(this);
}

void Car::update()
{
    int wheelIndex = 2;

    m_vehicle->applyEngineForce(gEngineForce, wheelIndex);
    m_vehicle->setBrake(gBreakingForce, wheelIndex);

    wheelIndex = 3;
    m_vehicle->applyEngineForce(gEngineForce, wheelIndex);
    m_vehicle->setBrake(gBreakingForce, wheelIndex);

    wheelIndex = 0;
    m_vehicle->setSteeringValue(gVehicleSteering, wheelIndex);

    wheelIndex = 1;
    m_vehicle->setSteeringValue(gVehicleSteering, wheelIndex);
}

void Car::updateWheelTransformForRender(CarSimulation * simulation)
{
    GUIHelperInterface * m_guiHelper = simulation->getGuiHelper();

    for (int i = 0; i < m_vehicle->getNumWheels(); i++)
    {
        //synchronize the wheels with the (interpolated) chassis world transform
        m_vehicle->updateWheelTransform(i, true);

        CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
        if (renderer)
        {
            btTransform tr = m_vehicle->getWheelInfo(i).m_worldTransform;
            btVector3 pos = tr.getOrigin();
            btQuaternion orn = tr.getRotation();
            renderer->writeSingleInstanceTransformToCPU(pos, orn, m_wheelInstances[i]);
        }
    }
}

bool Car::keyboardCallback(int key, int state, bool is_shift_pressed)
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

                m_liftHinge->setLimit(-M_PI / 16.0f, M_PI / 8.0f);
                m_liftHinge->enableAngularMotor(true, -0.1, maxMotorImpulse);
                handled = true;
                break;
            }
            case B3G_RIGHT_ARROW:
            {

                m_liftHinge->setLimit(-M_PI / 16.0f, M_PI / 8.0f);
                m_liftHinge->enableAngularMotor(true, 0.1, maxMotorImpulse);
                handled = true;
                break;
            }
            case B3G_UP_ARROW:
            {
                m_forkSlider->setLowerLinLimit(0.1f);
                m_forkSlider->setUpperLinLimit(3.9f);
                m_forkSlider->setPoweredLinMotor(true);
                m_forkSlider->setMaxLinMotorForce(maxMotorImpulse);
                m_forkSlider->setTargetLinMotorVelocity(1.0);
                handled = true;
                break;
            }
            case B3G_DOWN_ARROW:
            {
                m_forkSlider->setLowerLinLimit(0.1f);
                m_forkSlider->setUpperLinLimit(3.9f);
                m_forkSlider->setPoweredLinMotor(true);
                m_forkSlider->setMaxLinMotorForce(maxMotorImpulse);
                m_forkSlider->setTargetLinMotorVelocity(-1.0);
                handled = true;
                break;
            }
            }

        }
        else
        {
            switch (key)
            {
            case B3G_LEFT_ARROW:
            {
                handled = true;
                gVehicleSteering += steeringIncrement;
                if (gVehicleSteering > steeringClamp)
                    gVehicleSteering = steeringClamp;

                break;
            }
            case B3G_RIGHT_ARROW:
            {
                handled = true;
                gVehicleSteering -= steeringIncrement;
                if (gVehicleSteering < -steeringClamp)
                    gVehicleSteering = -steeringClamp;

                break;
            }
            case B3G_UP_ARROW:
            {
                handled = true;
                gEngineForce = maxEngineForce;
                gBreakingForce = 0.f;
                break;
            }
            case B3G_DOWN_ARROW:
            {
                handled = true;
                gEngineForce = -maxEngineForce;
                gBreakingForce = 0.f;
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
            gEngineForce = 0.f;
            gBreakingForce = defaultBreakingForce;
            handled = true;
            break;
        }
        case B3G_DOWN_ARROW:
        {
            lockForkSlider();
            gEngineForce = 0.f;
            gBreakingForce = defaultBreakingForce;
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

void Car::lockLiftHinge()
{
    btScalar hingeAngle = m_liftHinge->getHingeAngle();
    btScalar lowLim = m_liftHinge->getLowerLimit();
    btScalar hiLim = m_liftHinge->getUpperLimit();
    m_liftHinge->enableAngularMotor(false, 0, 0);

    if (hingeAngle < lowLim)
    {
        //m_liftHinge->setLimit(lowLim, lowLim + LIFT_EPS);
        m_liftHinge->setLimit(lowLim, lowLim);
    }
    else if (hingeAngle > hiLim)
    {
        //m_liftHinge->setLimit(hiLim - LIFT_EPS, hiLim);
        m_liftHinge->setLimit(hiLim, hiLim);
    }
    else
    {
        //m_liftHinge->setLimit(hingeAngle - LIFT_EPS, hingeAngle + LIFT_EPS);
        m_liftHinge->setLimit(hingeAngle, hingeAngle);
    }

}

void Car::lockForkSlider()
{
    btScalar linDepth = m_forkSlider->getLinearPos();
    btScalar lowLim = m_forkSlider->getLowerLinLimit();
    btScalar hiLim = m_forkSlider->getUpperLinLimit();
    m_forkSlider->setPoweredLinMotor(false);

    if (linDepth <= lowLim)
    {
        m_forkSlider->setLowerLinLimit(lowLim);
        m_forkSlider->setUpperLinLimit(lowLim);
    }
    else if (linDepth > hiLim)
    {
        m_forkSlider->setLowerLinLimit(hiLim);
        m_forkSlider->setUpperLinLimit(hiLim);
    }
    else
    {
        m_forkSlider->setLowerLinLimit(linDepth);
        m_forkSlider->setUpperLinLimit(linDepth);
    }

}
