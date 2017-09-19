#include <iostream>
#include "CarSimulation.h"

CarSimulation::CarSimulation(GUIHelperInterface* helper)
    :m_guiHelper(helper)
{
    //helper->setUpAxis(1);
    this->initDynamicsWorld();
}

CarSimulation::~CarSimulation()
{
    //clear();
}

void CarSimulation::initDynamicsWorld()
{
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    btVector3 worldMin(-1000, -1000, -1000);
    btVector3 worldMax(1000, 1000, 1000);
    m_overlappingPairCache = new btAxisSweep3(worldMin, worldMax);

    if (useMCLPSolver)
    {
        btDantzigSolver* mlcp = new btDantzigSolver();
        //btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
        btMLCPSolver* solver = new btMLCPSolver(mlcp);
        m_constraintSolver = solver;
    }
    else
        m_constraintSolver = new btSequentialImpulseConstraintSolver();

    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache, m_constraintSolver, m_collisionConfiguration);

    if (useMCLPSolver)
        m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 1;//for direct solver it is better to have a small A matrix
    else
        m_dynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 128;//for direct solver, it is better to solve multiple objects together, small batches have high overhead

    m_dynamicsWorld->getSolverInfo().m_globalCfm = 0.00001;
    //m_dynamicsWorld->setGravity(btVector3(0,0,0));

    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    int upAxis = 1;
    m_guiHelper->setUpAxis(upAxis);
}

btDiscreteDynamicsWorld * CarSimulation::getDynamicsWorld()
{
    return m_dynamicsWorld;
}

GUIHelperInterface * CarSimulation::getGuiHelper()
{
    return this->m_guiHelper;
}

void CarSimulation::stepSimulation(float deltaTime)
{
    for (int i = 0; i < cars.size(); ++i)
        cars[i]->update();

    float dt = deltaTime;

    if (m_dynamicsWorld)
    {
        //during idle mode, just run 1 simulation step maximum
        int maxSimSubSteps = 2;

        int numSimSteps;
        numSimSteps = m_dynamicsWorld->stepSimulation(dt, maxSimSubSteps);

        if (m_dynamicsWorld->getConstraintSolver()->getSolverType() == BT_MLCP_SOLVER)
        {
            btMLCPSolver* sol = (btMLCPSolver*)m_dynamicsWorld->getConstraintSolver();
            int numFallbacks = sol->getNumFallbacks();
            if (numFallbacks)
            {
                static int totalFailures = 0;
                totalFailures += numFallbacks;
                printf("MLCP solver failed %d times, falling back to btSequentialImpulseSolver (SI)\n", totalFailures);
            }
            sol->setNumFallbacks(0);
        }


        //#define VERBOSE_FEEDBACK
        #ifdef VERBOSE_FEEDBACK
        if (!numSimSteps)
            printf("Interpolated transforms\n");
        else
        {
            if (numSimSteps > maxSimSubSteps)
            {
                //detect dropping frames
                printf("Dropped (%i) simulation steps out of %i\n", numSimSteps - maxSimSubSteps, numSimSteps);
            }
            else
            {
                printf("Simulated (%i) steps\n", numSimSteps);
            }
        }
        #endif 
    }
}

void CarSimulation::clientResetScene()
{
    clear();
    generateGraphicsObjects();
}

void CarSimulation::specialKeyboardUp(int key, int x, int y)
{

}

void CarSimulation::specialKeyboard(int key, int x, int y)
{

}

bool CarSimulation::keyboardCallback(int key, int state)
{
    bool handled = false;
    bool isShiftPressed = m_guiHelper->getAppInterface()->m_window->isModifierKeyPressed(B3G_SHIFT);

    for (int i = 0; i < cars.size(); ++i)
        cars[i]->keyboardCallback(key, state, isShiftPressed);

    if (state && isShiftPressed == false)
    {
        switch (key)
        {
        case B3G_F7:
        {
            handled = true;
            btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*)m_dynamicsWorld;
            world->setLatencyMotionStateInterpolation(!world->getLatencyMotionStateInterpolation());
            printf("world latencyMotionStateInterpolation = %d\n", world->getLatencyMotionStateInterpolation());
            break;
        }
        case B3G_F6:
        {
            handled = true;
            //switch solver (needs demo restart)
            useMCLPSolver = !useMCLPSolver;
            printf("switching to useMLCPSolver = %d\n", useMCLPSolver);

            delete m_constraintSolver;
            if (useMCLPSolver)
            {
                btDantzigSolver* mlcp = new btDantzigSolver();
                //btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
                btMLCPSolver* sol = new btMLCPSolver(mlcp);
                m_constraintSolver = sol;
            }
            else
            {
                m_constraintSolver = new btSequentialImpulseConstraintSolver();
            }

            m_dynamicsWorld->setConstraintSolver(m_constraintSolver);

            //clear();
            //generateGraphicsObjects();
            break;
        }

        case B3G_F5:
            handled = true;
            m_useDefaultCamera = !m_useDefaultCamera;
            break;
        default:
            break;
        }
    }

    return handled;
}

bool CarSimulation::mouseMoveCallback(float x, float y)
{
    return false;
}

bool CarSimulation::mouseButtonCallback(int button, int state, float x, float y)
{
    return false;
}

btRigidBody* CarSimulation::localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        shape->calculateLocalInertia(mass, localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

    #define USE_MOTIONSTATE 1
    #ifdef  USE_MOTIONSTATE
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
    btRigidBody* body = new btRigidBody(cInfo);

    //body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

    #else
    btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
    body->setWorldTransform(startTransform);
    #endif

    m_dynamicsWorld->addRigidBody(body);
    return body;
}

void CarSimulation::displayCallback()
{
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

    //optional but useful: debug drawing
    if (m_dynamicsWorld)
        m_dynamicsWorld->debugDrawWorld();

    //	glFlush();
    //	glutSwapBuffers();
}

void CarSimulation::physicsDebugDraw(int debugFlags)
{
    if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
    {
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
        m_dynamicsWorld->debugDrawWorld();
    }
}

void CarSimulation::renderScene()
{
    m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);

    for (int i = 0; i < cars.size(); ++i)
        cars[i]->updateWheelTransformForRender(this);

    m_guiHelper->render(m_dynamicsWorld);

    /*
    ATTRIBUTE_ALIGNED16(btScalar) m[16];

    btVector3	worldBoundsMin, worldBoundsMax;
    getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin, worldBoundsMax);

    for (int i = 0; i < m_vehicle->getNumWheels(); i++)
    {
        //synchronize the wheels with the (interpolated) chassis world transform
        m_vehicle->updateWheelTransform(i, true);
        //draw wheels (cylinders)
        m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
        //m_shapeDrawer->drawOpenGL(m, m_wheelShape, wheelColor, getDebugMode(), worldBoundsMin, worldBoundsMax);
    }
    */
}

void CarSimulation::addCar(Car * car)
{
    cars.push_back(car);
}

void CarSimulation::addDefaultFloor()
{
    btVector3 groundExtents(50, 50, 50);

    int upAxis = 1;
    groundExtents[upAxis] = 3;

    btCollisionShape* groundShape = new btBoxShape(groundExtents);
    m_collisionShapes.push_back(groundShape);

   
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(0, -3, 0));

    //create ground object
    localCreateRigidBody(0, tr, groundShape);
}

void CarSimulation::addDefaultRigidBody()
{
    btCompoundShape* loadCompound = new btCompoundShape();
    m_collisionShapes.push_back(loadCompound);

    btCollisionShape* loadShapeA = new btBoxShape(btVector3(2.0f, 0.5f, 0.5f));
    m_collisionShapes.push_back(loadShapeA);
    btTransform loadTrans;
    loadTrans.setIdentity();
    loadCompound->addChildShape(loadTrans, loadShapeA);

    btCollisionShape* loadShapeB = new btBoxShape(btVector3(0.1f, 1.0f, 1.0f));
    m_collisionShapes.push_back(loadShapeB);
    loadTrans.setIdentity();
    loadTrans.setOrigin(btVector3(2.1f, 0.0f, 0.0f));
    loadCompound->addChildShape(loadTrans, loadShapeB);

    btCollisionShape* loadShapeC = new btBoxShape(btVector3(0.1f, 1.0f, 1.0f));
    m_collisionShapes.push_back(loadShapeC);
    loadTrans.setIdentity();
    loadTrans.setOrigin(btVector3(-2.1f, 0.0f, 0.0f));
    loadCompound->addChildShape(loadTrans, loadShapeC);


    loadTrans.setIdentity();
    m_loadStartPos = btVector3(0.0f, 3.5f, 7.0f);
    loadTrans.setOrigin(m_loadStartPos);

    //the sequential impulse solver has difficulties dealing with large mass ratios (differences), between loadMass and the fork parts
    btScalar loadMass = 350.f;

    //this should work fine for the SI solver
    //btScalar loadMass = 10.f; 

    m_loadBody = localCreateRigidBody(loadMass, loadTrans, loadCompound);
}

void CarSimulation::generateGraphicsObjects()
{
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void CarSimulation::clear()
{
    //cleanup in the reverse order of creation/initialization

    //remove the rigid bodies from the dynamics world and delete them
    for (int i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {

            while (body->getNumConstraintRefs())
            {
                btTypedConstraint* constraint = body->getConstraintRef(0);
                m_dynamicsWorld->removeConstraint(constraint);
                delete constraint;
            }
            delete body->getMotionState();
            m_dynamicsWorld->removeRigidBody(body);
        }
        else
        {
            m_dynamicsWorld->removeCollisionObject(obj);
        }
        delete obj;
    }

    //delete collision shapes
    for (int j = 0; j < m_collisionShapes.size(); j++)
    {
        btCollisionShape* shape = m_collisionShapes[j];
        delete shape;
    }
    m_collisionShapes.clear();

    delete m_indexVertexArrays;

    //delete dynamics world
    delete m_dynamicsWorld;
    m_dynamicsWorld = nullptr;

    //delete solver
    delete m_constraintSolver;
    m_constraintSolver = nullptr;

    //delete broad phase
    delete m_overlappingPairCache;
    m_overlappingPairCache = nullptr;

    //delete dispatcher
    delete m_dispatcher;
    m_dispatcher = nullptr;

    delete m_collisionConfiguration;
    m_collisionConfiguration = nullptr;
}

void CarSimulation::resetCamera()
{
    float dist = 18;
    float pitch = -45;
    float yaw = 32;
    float targetPos[3] = { -0.33,-0.72,4.5 };
    m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
}
