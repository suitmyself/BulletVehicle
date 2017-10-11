#pragma once

#include <vector>

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

    // Note: for any rigid body, mass = 0 also means static object

    int addMeshRigidBody(const std::vector<float> & vertex_data,
                         const std::vector<unsigned int> & face_index,
                         const btTransform & transform,
                         float mass = 0);

    int addMeshRigidBody(const float * vertex_data,
                         unsigned int vcount, // vertex_num * 3
                         const unsigned int * face_index,
                         unsigned int fcount, // face_num * 3
                         const btTransform & transform,
                         float mass = 0);

    int addBoxRigidBody(const btVector3 & half_extents, const btTransform & transform, float mass = 0);

    // Note: central axis aligned with the Y axis
    int addCylinderRigidBody(const btVector3 & half_extents, const btTransform & transform, float mass = 0);

    int addSphereRigidBody(float radius, const btTransform & transform, float mass = 0);

    unsigned int getCarNum() const;
    std::vector<Car *> & getCarPtrs();
    const std::vector<Car *> & getCarPtrs() const;

    // Note: first btTransform for car chassis, and the following four btTransform for car wheels
    std::vector<btTransform> getCarTransform(unsigned int idx) const;

    std::vector<std::pair<int, btTransform>> getAllRigidBodyTransforms() const;
    
    void stepSimulation(float delta_time);
    void clientResetScene();

    btRigidBody * localCreateRigidBody(btScalar mass, const btTransform & world_transform, btCollisionShape* col_shape);
    
    void specialKeyboard(int key, int x, int y);
    void specialKeyboardUp(int key, int x, int y);
    bool keyboardCallback(int key, int state);

    bool mouseMoveCallback(float x, float y);
    bool mouseButtonCallback(int button, int state, float x, float y);

    void displayCallback();
    void physicsDebugDraw(int debug_flags);
    void renderScene();
    
    void generateGraphicsObjects();

    void clear();

    virtual void resetCamera();

private:
    void initDynamicsWorld();
    btIndexedMesh constructIndexMesh(const float * vertices, int vcount, const unsigned int * faces, int fcount);

private:
    GUIHelperInterface * gui_helper;

    btVector3 camera_position = {30, 30, 30};
    float camera_height = 4.0f;
    float min_camera_distance = 3.0f;
    float max_camera_distance = 10.0f;
    bool use_default_camera = false;

    bool use_MCLP_solver = true;

    btAlignedObjectArray<btCollisionShape*> collision_shapes;

    btDiscreteDynamicsWorld * dynamics_world = nullptr;
    btBroadphaseInterface *	overlapping_pair_cache = nullptr;
    btCollisionDispatcher *	dispatcher = nullptr;
    btConstraintSolver * constraint_solver = nullptr;
    btDefaultCollisionConfiguration * collision_configuration = nullptr;

    std::vector<Car *> cars;
};
