#include <iostream>
#include "CarSimulation.h"

CarSimulation::CarSimulation(GUIHelperInterface* helper)
    :gui_helper(helper)
{
    //helper->setUpAxis(1);
    this->initDynamicsWorld();
}

CarSimulation::~CarSimulation()
{
    //clear();
}

btDiscreteDynamicsWorld * CarSimulation::getDynamicsWorld()
{
    return dynamics_world;
}

GUIHelperInterface * CarSimulation::getGuiHelper()
{
    return this->gui_helper;
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
    collision_shapes.push_back(groundShape);


    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(0, -3, 0));

    //create ground object
    localCreateRigidBody(0, tr, groundShape);
}

void CarSimulation::addDefaultRigidBody()
{
    btCompoundShape* loadCompound = new btCompoundShape();
    collision_shapes.push_back(loadCompound);

    btCollisionShape* loadShapeA = new btBoxShape(btVector3(2.0f, 0.5f, 0.5f));
    collision_shapes.push_back(loadShapeA);
    btTransform loadTrans;
    loadTrans.setIdentity();
    loadCompound->addChildShape(loadTrans, loadShapeA);

    btCollisionShape* loadShapeB = new btBoxShape(btVector3(0.1f, 1.0f, 1.0f));
    collision_shapes.push_back(loadShapeB);
    loadTrans.setIdentity();
    loadTrans.setOrigin(btVector3(2.1f, 0.0f, 0.0f));
    loadCompound->addChildShape(loadTrans, loadShapeB);

    btCollisionShape* loadShapeC = new btBoxShape(btVector3(0.1f, 1.0f, 1.0f));
    collision_shapes.push_back(loadShapeC);
    loadTrans.setIdentity();
    loadTrans.setOrigin(btVector3(-2.1f, 0.0f, 0.0f));
    loadCompound->addChildShape(loadTrans, loadShapeC);


    loadTrans.setIdentity();
    btVector3 load_start_pos = btVector3(0.0f, 3.5f, 7.0f);
    loadTrans.setOrigin(load_start_pos);

    //the sequential impulse solver has difficulties dealing with large mass ratios (differences), between loadMass and the fork parts
    btScalar loadMass = 350.f;

    //this should work fine for the SI solver
    //btScalar loadMass = 10.f; 

    localCreateRigidBody(loadMass, loadTrans, loadCompound);
}

btIndexedMesh CarSimulation::constructIndexMesh(const float * vertices, int vcount, const unsigned int * faces, int fcount)
{
    assert(fcount % 3 == 0);

    btIndexedMesh indexed_mesh;

    indexed_mesh.m_numTriangles = fcount / 3;
    indexed_mesh.m_triangleIndexBase = (const unsigned char *)faces;
    indexed_mesh.m_triangleIndexStride = sizeof(unsigned int) * 3;

    indexed_mesh.m_numVertices = vcount / 3;
    indexed_mesh.m_vertexBase = (const unsigned char *)vertices;
    indexed_mesh.m_vertexStride = sizeof(float) * 3;
    indexed_mesh.m_vertexType = PHY_FLOAT;

    return indexed_mesh;
}

int CarSimulation::addMeshRigidBody(const std::vector<float> & vertex_data, const std::vector<unsigned int> & face_index, const btTransform & transform, float mass)
{
    return this->addMeshRigidBody(vertex_data.data(), vertex_data.size(), face_index.data(), face_index.size(), transform, mass);
}

int CarSimulation::addMeshRigidBody(const float * vertex_data, unsigned int vcount, const unsigned int * face_index, unsigned int fcount, const btTransform & transform, float mass)
{
    btIndexedMesh indexed_mesh = this->constructIndexMesh(vertex_data, vcount, face_index, fcount);

    btTriangleIndexVertexArray * vertex_array = new btTriangleIndexVertexArray();
    vertex_array->addIndexedMesh(indexed_mesh);

    btBvhTriangleMeshShape * mesh_shape = new btBvhTriangleMeshShape(vertex_array, true);
    collision_shapes.push_back(mesh_shape);

    btRigidBody * rigid_body = localCreateRigidBody(mass, transform, mesh_shape);
    return rigid_body->getUserIndex();
}

int CarSimulation::addBoxRigidBody(const btVector3 & half_extents, const btTransform & transform, float mass)
{
    btCollisionShape * box_shape = new btBoxShape(half_extents);
    collision_shapes.push_back(box_shape);

    btRigidBody * rigid_body = localCreateRigidBody(mass, transform, box_shape);

    return rigid_body->getUserIndex();
}

int CarSimulation::addCylinderRigidBody(const btVector3 & half_extents, const btTransform & transform, float mass)
{
    btCollisionShape * cylinder_shape = new btCylinderShape(half_extents);
    collision_shapes.push_back(cylinder_shape);

    btRigidBody * rigid_body = localCreateRigidBody(mass, transform, cylinder_shape);

    return rigid_body->getUserIndex();
}

int CarSimulation::addSphereRigidBody(float radius, const btTransform & transform, float mass)
{
    btCollisionShape * sphere_shape = new btSphereShape(radius);
    collision_shapes.push_back(sphere_shape);

    btRigidBody * rigid_body = localCreateRigidBody(mass, transform, sphere_shape);

    return rigid_body->getUserIndex();
}

unsigned int CarSimulation::getCarNum() const
{
    return this->cars.size();
}

std::vector<Car *> & CarSimulation::getCarPtrs()
{
    return this->cars;
}

const std::vector<Car *> & CarSimulation::getCarPtrs() const
{
    return this->cars;
}

std::vector<btTransform> CarSimulation::getCarTransform(unsigned int idx) const
{
    return this->cars[idx]->getCarTransform();
}

std::vector<std::pair<int, btTransform>> CarSimulation::getAllRigidBodyTransforms() const
{
    std::vector<std::pair<int, btTransform>> trans;
    for(int i = 0; i < this->dynamics_world->getNumCollisionObjects(); ++i)
    {
        btCollisionObject * obj = this->dynamics_world->getCollisionObjectArray()[i];
        btRigidBody * rigid_body = btRigidBody::upcast(obj);

        btTransform tr;

        if (rigid_body && rigid_body->getMotionState())
            rigid_body->getMotionState()->getWorldTransform(tr);
        else
            tr = obj->getWorldTransform();

        trans.push_back({ obj->getUserIndex(), tr });
    }

    return trans;
}

void CarSimulation::stepSimulation(float deltaTime)
{
    for (int i = 0; i < cars.size(); ++i)
        cars[i]->update();

    float dt = deltaTime;

    if (dynamics_world)
    {
        //during idle mode, just run 1 simulation step maximum
        int maxSimSubSteps = 2;

        int numSimSteps;
        numSimSteps = dynamics_world->stepSimulation(dt, maxSimSubSteps);

        if (dynamics_world->getConstraintSolver()->getSolverType() == BT_MLCP_SOLVER)
        {
            btMLCPSolver* sol = (btMLCPSolver*)dynamics_world->getConstraintSolver();
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
    bool isShiftPressed = gui_helper->getAppInterface()->m_window->isModifierKeyPressed(B3G_SHIFT);

    for (int i = 0; i < cars.size(); ++i)
        cars[i]->keyboardCallback(key, state, isShiftPressed);

    if (state && isShiftPressed == false)
    {
        switch (key)
        {
        case B3G_F7:
        {
            handled = true;
            btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*)dynamics_world;
            world->setLatencyMotionStateInterpolation(!world->getLatencyMotionStateInterpolation());
            printf("world latencyMotionStateInterpolation = %d\n", world->getLatencyMotionStateInterpolation());
            break;
        }
        case B3G_F6:
        {
            handled = true;
            //switch solver (needs demo restart)
            use_MCLP_solver = !use_MCLP_solver;
            printf("switching to useMLCPSolver = %d\n", use_MCLP_solver);

            delete constraint_solver;
            if (use_MCLP_solver)
            {
                btDantzigSolver* mlcp = new btDantzigSolver();
                //btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
                btMLCPSolver* sol = new btMLCPSolver(mlcp);
                constraint_solver = sol;
            }
            else
            {
                constraint_solver = new btSequentialImpulseConstraintSolver();
            }

            dynamics_world->setConstraintSolver(constraint_solver);

            //clear();
            //generateGraphicsObjects();
            break;
        }
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

btRigidBody* CarSimulation::localCreateRigidBody(btScalar mass, const btTransform & startTransform, btCollisionShape* shape)
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

    dynamics_world->addRigidBody(body);
    return body;
}

void CarSimulation::displayCallback()
{
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

    //optional but useful: debug drawing
    if (dynamics_world)
        dynamics_world->debugDrawWorld();

    //glFlush();
    //glutSwapBuffers();
}

void CarSimulation::physicsDebugDraw(int debugFlags)
{
    if (dynamics_world && dynamics_world->getDebugDrawer())
    {
        dynamics_world->getDebugDrawer()->setDebugMode(debugFlags);
        dynamics_world->debugDrawWorld();
    }
}

void CarSimulation::renderScene()
{
    gui_helper->syncPhysicsToGraphics(dynamics_world);

    for (int i = 0; i < cars.size(); ++i)
        cars[i]->updateWheelTransformForRender(this);

    gui_helper->render(dynamics_world);

    /*
    for(int i = 0; i < m_dynamicsWorld->getCollisionObjectArray().size(); ++i)
        std::cout << "id: " << i << " user_index: " << m_dynamicsWorld->getCollisionObjectArray()[i]->getUserIndex() << std::endl;
    */

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

void CarSimulation::generateGraphicsObjects()
{
    gui_helper->autogenerateGraphicsObjects(dynamics_world);
}

void CarSimulation::clear()
{
    //cleanup in the reverse order of creation/initialization

    //remove the rigid bodies from the dynamics world and delete them
    for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = dynamics_world->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {

            while (body->getNumConstraintRefs())
            {
                btTypedConstraint* constraint = body->getConstraintRef(0);
                dynamics_world->removeConstraint(constraint);
                delete constraint;
            }
            delete body->getMotionState();
            dynamics_world->removeRigidBody(body);
        }
        else
        {
            dynamics_world->removeCollisionObject(obj);
        }
        delete obj;
    }

    //delete collision shapes
    for (int j = 0; j < collision_shapes.size(); j++)
    {
        btCollisionShape* shape = collision_shapes[j];
        delete shape;
    }
    collision_shapes.clear();

    //delete dynamics world
    delete dynamics_world;
    dynamics_world = nullptr;

    //delete solver
    delete constraint_solver;
    constraint_solver = nullptr;

    //delete broad phase
    delete overlapping_pair_cache;
    overlapping_pair_cache = nullptr;

    //delete dispatcher
    delete dispatcher;
    dispatcher = nullptr;

    delete collision_configuration;
    collision_configuration = nullptr;
}

void CarSimulation::resetCamera()
{
    float dist = 60;
    float pitch = 0;
    float yaw = 70;
    float targetPos[3] = { 0,0,0 };
    gui_helper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
}

void CarSimulation::initDynamicsWorld()
{
    collision_configuration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collision_configuration);

    btVector3 worldMin(-1000, -1000, -1000);
    btVector3 worldMax(1000, 1000, 1000);
    overlapping_pair_cache = new btAxisSweep3(worldMin, worldMax);

    if (use_MCLP_solver)
    {
        btDantzigSolver* mlcp = new btDantzigSolver();
        //btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
        btMLCPSolver* solver = new btMLCPSolver(mlcp);
        constraint_solver = solver;
    }
    else
        constraint_solver = new btSequentialImpulseConstraintSolver();

    dynamics_world = new btDiscreteDynamicsWorld(dispatcher, overlapping_pair_cache, constraint_solver, collision_configuration);

    if (use_MCLP_solver)
        dynamics_world->getSolverInfo().m_minimumSolverBatchSize = 1;//for direct solver it is better to have a small A matrix
    else
        dynamics_world->getSolverInfo().m_minimumSolverBatchSize = 128;//for direct solver, it is better to solve multiple objects together, small batches have high overhead

    dynamics_world->getSolverInfo().m_globalCfm = 0.00001;
    //m_dynamicsWorld->setGravity(btVector3(0,0,0));

    gui_helper->createPhysicsDebugDrawer(dynamics_world);
    int upAxis = 1;
    gui_helper->setUpAxis(upAxis);
}