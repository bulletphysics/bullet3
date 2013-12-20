#include "../../btgui/OpenGLWindow/SimpleOpenGL3App.h"
#include "Bullet3Common/b3Vector3.h"
#include "assert.h"
#include <stdio.h>

#include "../GpuDemos/gwenUserInterface.h"

GwenUserInterface* gui  = 0;

static void MyMouseMoveCallback( float x, float y)
{
	bool handled = false;
	if (gui)
		handled = gui->mouseMoveCallback(x,y);
	if (!handled)
		b3DefaultMouseMoveCallback(x,y);
}
static void MyMouseButtonCallback(int button, int state, float x, float y)
{
	bool handled = false;
	//try picking first
	if (gui)
		handled = gui->mouseButtonCallback(button,state,x,y);

	if (!handled)
		b3DefaultMouseButtonCallback(button,state,x,y);
}



void	MyComboBoxCallback(int comboId, const char* item)
{
	printf("comboId = %d, item = %s\n",comboId, item);
	/*int numDemos = demoNames.size();
	for (int i=0;i<numDemos;i++)
	{
		if (!strcmp(demoNames[i],item))
		{
			if (selectedDemo != i)
			{
				gReset = true;
				selectedDemo = i;
				printf("selected demo %s!\n", item);
			}
		}
	}
	*/

}

int main(int argc, char* argv[])
{
	
	float dt = 1./120.f;
	int width = 1024;
	int height=768;

	SimpleOpenGL3App* app = new SimpleOpenGL3App("AllBullet2Demos",width,height);
	app->m_instancingRenderer->setCameraDistance(13);
	app->m_instancingRenderer->setCameraPitch(0);
	app->m_instancingRenderer->setCameraTargetPosition(b3MakeVector3(0,0,0));
	app->m_window->setMouseMoveCallback(MyMouseMoveCallback);
	app->m_window->setMouseButtonCallback(MyMouseButtonCallback);
	

	GLint err = glGetError();
    assert(err==GL_NO_ERROR);
	
	sth_stash* fontstash=app->getFontStash();
	gui = new GwenUserInterface;
	gui->init(width,height,fontstash,app->m_window->getRetinaScale());

	const char* names[] = {"test1", "test2","test3"};
	gui->registerComboBox(13,3,&names[0],1);
	const char* names2[] = {"comboF", "comboG","comboH"};
	gui->registerComboBox(2,3,&names2[0],1);

	gui->setComboBoxCallback(MyComboBoxCallback);


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

		static int toggle = 1;
		if (1)
		{
		gui->draw(app->m_instancingRenderer->getScreenWidth(),app->m_instancingRenderer->getScreenHeight());
		}
		toggle=1-toggle;
		app->swapBuffer();
	} while (!app->m_window->requestedExit());

	delete gui;

	delete app;
	return 0;
}
