#include <stdio.h>
#include "Utils/b3Clock.h"

#include "Car.h"
#include "CarSimulation.h"

#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "ExampleBrowser/OpenGLGuiHelper.h"


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
    fir_car.configToCarSimulation(example, btVector3(0, 0, 0));
    Car sec_car;
    sec_car.configToCarSimulation(example, btVector3(10, 0, 0));

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
		app->drawGrid(dg);
		
		app->swapBuffer();
	} while (!app->m_window->requestedExit());

	example->clear();
	delete example;
	delete app;

	return 0;
}

