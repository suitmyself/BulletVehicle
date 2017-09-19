#pragma once

#include "Car.h"

#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"


#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"


class CarSimulation
{
public:

    CarSimulation(GUIHelperInterface * helper);
    virtual ~CarSimulation();

    btDiscreteDynamicsWorld * getDynamicsWorld();
    GUIHelperInterface * getGuiHelper();

    void addCar(Car * car);
    void addDefaultFloor();
    void addDefaultRigidBody();

    // to do
    void addRigidBody();

    void stepSimulation(float deltaTime);
    void clientResetScene();

    btRigidBody * localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);
    
    void specialKeyboard(int key, int x, int y);
    void specialKeyboardUp(int key, int x, int y);
    bool keyboardCallback(int key, int state);

    bool mouseMoveCallback(float x, float y);
    bool mouseButtonCallback(int button, int state, float x, float y);

    void displayCallback();
    void physicsDebugDraw(int debugFlags);
    void renderScene();
    
    void generateGraphicsObjects();

    void clear();

    virtual void resetCamera();

private:
    void initDynamicsWorld();

private:
    GUIHelperInterface * m_guiHelper;

    btVector3 m_cameraPosition = {30, 30, 30};
    float m_cameraHeight = 4.0f;
    float m_minCameraDistance = 3.0f;
    float m_maxCameraDistance = 10.0f;
    bool m_useDefaultCamera = false;

    bool useMCLPSolver = true;

    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

    btDiscreteDynamicsWorld * m_dynamicsWorld = nullptr;
    btBroadphaseInterface *	m_overlappingPairCache = nullptr;
    btCollisionDispatcher *	m_dispatcher = nullptr;
    btConstraintSolver * m_constraintSolver = nullptr;
    btDefaultCollisionConfiguration * m_collisionConfiguration = nullptr;
    btTriangleIndexVertexArray * m_indexVertexArrays = nullptr;

    btAlignedObjectArray<Car *> cars;

    btRigidBody * m_loadBody = nullptr;
    btVector3	m_loadStartPos;
};
