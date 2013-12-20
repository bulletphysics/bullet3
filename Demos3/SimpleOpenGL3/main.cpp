#include "../../btgui/OpenGLWindow/SimpleOpenGL3App.h"
#include "Bullet3Common/b3Vector3.h"
#include "assert.h"
#include <stdio.h>

int main(int argc, char* argv[])
{
	
	float dt = 1./120.f;
	
	SimpleOpenGL3App* app = new SimpleOpenGL3App("SimpleOpenGL3App",1024,768);
	app->m_instancingRenderer->setCameraDistance(13);
	app->m_instancingRenderer->setCameraPitch(0);
	app->m_instancingRenderer->setCameraTargetPosition(b3MakeVector3(0,0,0));

	GLint err = glGetError();
    assert(err==GL_NO_ERROR);
	
	do
	{
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera();
		
		app->drawGrid();
		char bla[1024];
		static int frameCount = 0;
		frameCount++;
		sprintf(bla,"Simple test frame %d", frameCount);
		
		app->drawText(bla,10,10);
		app->swapBuffer();
	} while (!app->m_window->requestedExit());

	
	delete app;
	return 0;
}
