#include <stdio.h>
#include <vector>
#include <iostream>
#include "Utils/b3Clock.h"

#include "Car.h"
#include "ForkLiftCar.h"

#include "CarSimulation.h"

#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "OpenGLWindow/GLInstanceGraphicsShape.h"
#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "ExampleBrowser/OpenGLGuiHelper.h"
#include "Importers/ImportObjDemo/LoadMeshFromObj.h"

using namespace  std;

//CommonExampleInterface * example;
CarSimulation * example;

int gSharedMemoryKey = -1;

b3MouseMoveCallback prevMouseMoveCallback = nullptr;
b3MouseButtonCallback prevMouseButtonCallback = nullptr;

static void OnMouseMove( float x, float y)
{
	bool handled = example->mouseMoveCallback(x,y); 	 
	if (!handled)
	{
		if (prevMouseMoveCallback)
			prevMouseMoveCallback (x,y);
	}
}

static void OnMouseDown(int button, int state, float x, float y) 
{
	bool handled = example->mouseButtonCallback(button, state, x,y); 
	if (!handled)
	{
		if (prevMouseButtonCallback )
			prevMouseButtonCallback (button,state,x,y);
	}
}

class LessDummyGuiHelper : public DummyGUIHelper
{
public:
    LessDummyGuiHelper(CommonGraphicsApp * app)
        :m_app(app)
    {
    }

	virtual CommonGraphicsApp * getAppInterface()
	{
		return m_app;
	}

private:
    CommonGraphicsApp * m_app;
};

void MyKeyboardCallback(int key, int state)
{
    example->keyboardCallback(key, state);
}

int main(int argc, char* argv[])
{
	
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Standalone Example",1024,768,true);
	
	prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);
    app->m_window->setKeyboardCallback(MyKeyboardCallback);
	
	OpenGLGuiHelper gui(app,false);
	CommonExampleOptions options(&gui);

	//example = StandaloneExampleCreateFunc(options);
    example = new CarSimulation(&gui);

    example->addDefaultFloor();
    example->addDefaultRigidBody();
	
    Car fir_car;
    //fir_car.configToCarSimulation(example, btVector3(0, 0, 0));
    
    Car sec_car;
    sec_car.setFrontWheelXOffset(2.0);
    sec_car.setFrontWheelZOffset(2.0);
    sec_car.setBackWheelXOffset(1.0);
    sec_car.setBackWheelZOffset(2.0);
    sec_car.configToCarSimulation(example, btVector3(5, 0, 0));
    
    ForkLiftCar fork_lift_car;
    fork_lift_car.configToCarSimulation(example, btVector3(10, 0, 0));

    /*
    btTransform box_tr;
    box_tr.setIdentity();
    box_tr.setOrigin(btVector3(0, 0, 0));
    example->addBoxRigidBody(btVector3(2, 2, 2), box_tr, 5);
    */

    btTransform cylinder_tr;
    cylinder_tr.setIdentity();
    cylinder_tr.setOrigin(btVector3(0, 0, -5));
    example->addCylinderRigidBody(btVector3(0.5, 2, 2), cylinder_tr, 0);

    btTransform sphere_tr;
    sphere_tr.setIdentity();
    sphere_tr.setOrigin(btVector3(0, 0, -7));
    example->addSphereRigidBody(1, sphere_tr, 5);

    GLInstanceGraphicsShape * glmesh = LoadMeshFromObj("car_models/teapot.obj", "");
    //GLInstanceGraphicsShape * glmesh = LoadMeshFromObj("car_models/mount.blend1.obj", "");
    //GLInstanceGraphicsShape * glmesh = LoadMeshFromObj("car_models/Mountain.obj", "");

    vector<float> vertex_data;
    for (int i = 0; i < glmesh->m_vertices->size(); ++i)
    {
        vertex_data.push_back(glmesh->m_vertices->at(i).xyzw[0]);
        //vertex_data.push_back(glmesh->m_vertices->at(i).xyzw[1] * 0.3 - 25.0);
        vertex_data.push_back(glmesh->m_vertices->at(i).xyzw[1]);
        vertex_data.push_back(glmesh->m_vertices->at(i).xyzw[2]);
    }

    vector<unsigned int> face_index;
    for (int i = 0; i < glmesh->m_indices->size(); ++i)
        face_index.push_back(glmesh->m_indices->at(i));

    btTransform mesh_tr;
    mesh_tr.setIdentity();
    example->addMeshRigidBody(vertex_data, face_index, mesh_tr, 5.0);

    example->generateGraphicsObjects();
	example->resetCamera();
	
	b3Clock clock;

	do
	{
		app->m_instancingRenderer->init();
        app->m_instancingRenderer->updateCamera(app->getUpAxis());

		btScalar dtSec = btScalar(clock.getTimeInSeconds());
		if (dtSec<0.1)
			dtSec = 0.1;
	
		example->stepSimulation(dtSec);
	    clock.reset();

		example->renderScene();
 	
		DrawGridData dg;
        dg.upAxis = app->getUpAxis();
		//app->drawGrid(dg);
		
		app->swapBuffer();

        //cout << "transform size: " << example->getAllRigidBodyTransforms().size() << endl;
        //cout << "car size: " << example->getCarTransform(0).size() << endl;
        
        example->getCarTransform(0);

	} while (!app->m_window->requestedExit());

	example->clear();
	delete example;
	delete app;

	return 0;
}

