#include <vector>
#include "CustomCar.h"
#include "CarSimulation.h"
#include "Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "OpenGLWindow/GLInstanceGraphicsShape.h"

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

CustomCar::CustomCar(const std::string & chassis_obj_file)
    :chassis_obj_file(chassis_obj_file)
{

}

void CustomCar::setCustomCarChassis(CarSimulation * simulation, const btVector3 & init_pos)
{
    //定义Car的组合体形状
    btCompoundShape* compound = new btCompoundShape();

    //定义Car底盘的形状并添加
    btTransform localTrans;
    localTrans.setIdentity();
    localTrans.setOrigin(btVector3(0, 1, 0));

    GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(chassis_obj_file.c_str(), "");
    printf("[INFO] Obj loaded: Extracted %d vertices from obj file [%s]\n", glmesh->m_numvertices, chassis_obj_file.c_str());

    std::vector<float> vertex_data;
    for (int i = 0; i < glmesh->m_vertices->size(); ++i)
    {
        vertex_data.push_back(glmesh->m_vertices->at(i).xyzw[0]);
        vertex_data.push_back(glmesh->m_vertices->at(i).xyzw[1]);
        vertex_data.push_back(glmesh->m_vertices->at(i).xyzw[2]);
    }

    std::vector<unsigned int> face_index;
    for (int i = 0; i < glmesh->m_indices->size(); ++i)
        face_index.push_back(glmesh->m_indices->at(i));

    btIndexedMesh indexed_mesh = this->constructIndexMesh(vertex_data.data(), vertex_data.size(), face_index.data(), face_index.size());

    btTriangleIndexVertexArray * vertex_array = new btTriangleIndexVertexArray();
    vertex_array->addIndexedMesh(indexed_mesh);

    btBvhTriangleMeshShape * chassis_shape = new btBvhTriangleMeshShape(vertex_array, true);

    //const GLInstanceVertex & v = glmesh->m_vertices->at(0);
    //btConvexHullShape * chassis_shape = new btConvexHullShape((const btScalar*)(&(v.xyzw[0])), glmesh->m_numvertices, sizeof(GLInstanceVertex));

    //btVector3 localScaling(0.2, 0.2, 0.2);
    //chassis_shape->setLocalScaling(localScaling);

    compound->addChildShape(localTrans, chassis_shape);

    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(init_pos);
    car_chassis = simulation->localCreateRigidBody(800, tr, compound);

    car_chassis->setDamping(0.2, 0.2);
    car_chassis->setActivationState(DISABLE_DEACTIVATION); //never deactivate the vehicle
}

btIndexedMesh CustomCar::constructIndexMesh(const float * vertices, int vcount, const unsigned int * faces, int fcount)
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

void CustomCar::configToCarSimulation(CarSimulation * simulation, const btVector3 & init_pos)
{
    btDiscreteDynamicsWorld * m_dynamicsWorld = simulation->getDynamicsWorld();
    GUIHelperInterface * m_guiHelper = simulation->getGuiHelper();

    this->setCustomCarChassis(simulation, init_pos);
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
