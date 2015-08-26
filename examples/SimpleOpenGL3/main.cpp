#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "assert.h"
#include <stdio.h>
#include "OpenGLWindow/OpenGLInclude.h"

char* gVideoFileName = 0;
char* gPngFileName = 0;

int main(int argc, char* argv[])
{
    b3CommandLineArgs myArgs(argc,argv);


	SimpleOpenGL3App* app = new SimpleOpenGL3App("SimpleOpenGL3App",1024,768,true);
	app->m_instancingRenderer->getActiveCamera()->setCameraDistance(13);
	app->m_instancingRenderer->getActiveCamera()->setCameraPitch(0);
	app->m_instancingRenderer->getActiveCamera()->setCameraTargetPosition(0,0,0);

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
