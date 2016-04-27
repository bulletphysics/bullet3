/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "BasicExample.h"

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"


#ifdef USE_GUI
#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include <stdio.h>
#include "../ExampleBrowser/OpenGLGuiHelper.h"

int main(int argc, char* argv[])
{

	SimpleOpenGL3App* app = new SimpleOpenGL3App("BasicDemoGui",1024,768,true);
	OpenGLGuiHelper gui(app,false);
        CommonExampleOptions options(&gui);
        CommonExampleInterface*    example = BasicExampleCreateFunc(options);
        
	example->initPhysics();

	int frameCount = 0;
	do
	{
		app->m_instancingRenderer->init();
                app->m_instancingRenderer->updateCamera();
		example->stepSimulation(1./60.);
        	example->renderScene();
                app->drawGrid();
                app->swapBuffer();
        } while (!app->m_window->requestedExit());

        example->exitPhysics();
        delete example;
	delete app;
        return 0;
}


#else

int main(int argc, char* argv[])
{
	
	DummyGUIHelper noGfx;

	CommonExampleOptions options(&noGfx);
	CommonExampleInterface*    example = BasicExampleCreateFunc(options);
	
	example->initPhysics();
	example->stepSimulation(1.f/60.f);
	example->exitPhysics();

	delete example;

	return 0;
}
#endif

