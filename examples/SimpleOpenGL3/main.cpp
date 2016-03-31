#include "OpenGLWindow/SimpleOpenGL3App.h"

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "assert.h"
#include <stdio.h>
#include "OpenGLWindow/OpenGLInclude.h"

char* gVideoFileName = 0;
char* gPngFileName = 0;

static b3WheelCallback sOldWheelCB = 0;
static b3ResizeCallback sOldResizeCB = 0;
static b3MouseMoveCallback sOldMouseMoveCB = 0;
static b3MouseButtonCallback sOldMouseButtonCB = 0;
static b3KeyboardCallback sOldKeyboardCB = 0;
//static b3RenderCallback sOldRenderCB = 0;


void MyWheelCallback(float deltax, float deltay)
{
	if (sOldWheelCB)
		sOldWheelCB(deltax,deltay);
}
void MyResizeCallback( float width, float height)
{
	if (sOldResizeCB)
		sOldResizeCB(width,height);
}
void MyMouseMoveCallback( float x, float y)
{
	printf("Mouse Move: %f, %f\n", x,y);

	if (sOldMouseMoveCB)
		sOldMouseMoveCB(x,y);
}
void MyMouseButtonCallback(int button, int state, float x, float y)
{
	if (sOldMouseButtonCB)
		sOldMouseButtonCB(button,state,x,y);
}


void MyKeyboardCallback(int keycode, int state)
{
	//keycodes are in examples/CommonInterfaces/CommonWindowInterface.h
	//for example B3G_ESCAPE for escape key
	//state == 1 for pressed, state == 0 for released.
	// use app->m_window->isModifiedPressed(...) to check for shift, escape and alt keys
	printf("MyKeyboardCallback received key:%c in state %d\n",keycode,state);
	if (sOldKeyboardCB)
		sOldKeyboardCB(keycode,state);
}


int main(int argc, char* argv[])
{
    b3CommandLineArgs myArgs(argc,argv);


	SimpleOpenGL3App* app = new SimpleOpenGL3App("SimpleOpenGL3App",1024,768,true);
	app->m_instancingRenderer->getActiveCamera()->setCameraDistance(13);
	app->m_instancingRenderer->getActiveCamera()->setCameraPitch(0);
	app->m_instancingRenderer->getActiveCamera()->setCameraTargetPosition(0,0,0);
	sOldKeyboardCB = app->m_window->getKeyboardCallback();	
	app->m_window->setKeyboardCallback(MyKeyboardCallback);
	sOldMouseMoveCB = app->m_window->getMouseMoveCallback();
	app->m_window->setMouseMoveCallback(MyMouseMoveCallback);
	sOldMouseButtonCB = app->m_window->getMouseButtonCallback();
	app->m_window->setMouseButtonCallback(MyMouseButtonCallback);
	sOldWheelCB = app->m_window->getWheelCallback();
	app->m_window->setWheelCallback(MyWheelCallback);
	sOldResizeCB = app->m_window->getResizeCallback();
	app->m_window->setResizeCallback(MyResizeCallback);
  


  assert(glGetError()==GL_NO_ERROR);

    myArgs.GetCmdLineArgument("mp4_file",gVideoFileName);
    if (gVideoFileName)
        app->dumpFramesToVideo(gVideoFileName);

    myArgs.GetCmdLineArgument("png_file",gPngFileName);
    char fileName[1024];

	do
	{
	    static int frameCount = 0;
		frameCount++;
		if (gPngFileName)
        {
            printf("gPngFileName=%s\n",gPngFileName);

            sprintf(fileName,"%s%d.png",gPngFileName,frameCount++);
            app->dumpNextFrameToPng(fileName);
        }

		assert(glGetError()==GL_NO_ERROR);
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera();

		app->drawGrid();
		char bla[1024];
		sprintf(bla,"Simple test frame %d", frameCount);

		app->drawText(bla,10,10);
		app->swapBuffer();
	} while (!app->m_window->requestedExit());


	delete app;
	return 0;
}
