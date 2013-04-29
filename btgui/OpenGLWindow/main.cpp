/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans

//
//#include "vld.h"
#ifndef __APPLE__
#include <GL/glew.h>
#endif
#include <assert.h>

#include "GLInstancingRenderer.h"

#ifdef __APPLE__
#include "MacOpenGLWindow.h"
#elif _WIN32
#include "Win32OpenGLWindow.h"
#elif __linux
#include "X11OpenGLWindow.h"
#endif

#include "GLPrimitiveRenderer.h"

extern char OpenSansData[];

#include "renderscene.h"


#include "Bullet3Common/b3Quickprof.h"
#include "Bullet3Common/b3Quaternion.h"

#include "Bullet3Common/b3CommandLineArgs.h"

#include "../OpenGLTrueTypeFont/fontstash.h"
#include "../OpenGLTrueTypeFont/opengl_fontstashcallbacks.h"

bool printStats = false;
bool pauseSimulation = false;
bool shootObject = false;


bool useInterop = false;

extern int NUM_OBJECTS_X;
extern int NUM_OBJECTS_Y;
extern int NUM_OBJECTS_Z;
extern bool keepStaticObjects;
extern float X_GAP;
extern float Y_GAP;
extern float Z_GAP;

const char* fileName="../../bin/1000 stack.bullet";
void Usage()
{
	printf("\nprogram.exe [--pause_simulation=<0 or 1>] [--load_bulletfile=test.bullet] [--enable_interop=<0 or 1>] [--enable_gpusap=<0 or 1>] [--enable_convexheightfield=<0 or 1>] [--enable_static=<0 or 1>] [--x_dim=<int>] [--y_dim=<num>] [--z_dim=<int>] [--x_gap=<float>] [--y_gap=<float>] [--z_gap=<float>]\n"); 
};


#include "gwenWindow.h"


void MyMouseButtonCallback(int button, int state, float x, float y)
{
	//b3DefaultMouseCallback(button,state,x,y);

	if (pCanvas)
	{
		bool handled = pCanvas->InputMouseMoved(x,y,x, y);

		if (button>=0)
		{
			handled = pCanvas->InputMouseButton(button,state);
			if (handled)
			{
				if (!state)
					return;
			}
		}
	}
}

int g_OpenGLWidth = 1024;//650;
int g_OpenGLHeight =800;

void MyResizeCallback(float width, float height)
{
	g_OpenGLWidth = width;
	g_OpenGLHeight = height;
	if (pCanvas)
	{
		pCanvas->SetSize(width,height);
		resizeGUI(width,height);
	}
}

void MyMouseMoveCallback( float x, float y)
{
	//b3DefaultMouseCallback(button,state,x,y);

	static int m_lastmousepos[2] = {0,0};
	static bool isInitialized = false;
	if (pCanvas)
	{
		if (!isInitialized)
		{
			isInitialized = true;
			m_lastmousepos[0] = x+1;
			m_lastmousepos[1] = y+1;
		}
		bool handled = pCanvas->InputMouseMoved(x,y,m_lastmousepos[0],m_lastmousepos[1]);
	}
}

	int droidRegular;//, droidItalic, droidBold, droidJapanese, dejavu;

sth_stash* initFont(GLPrimitiveRenderer* primRender)
{
	GLint err;

		struct sth_stash* stash = 0;
	int datasize;

	float sx,sy,dx,dy,lh;
	GLuint texture;
	OpenGL2RenderCallbacks* renderCallbacks = new OpenGL2RenderCallbacks(primRender);
	stash = sth_create(512,512,renderCallbacks);//256,256);//,1024);//512,512);
    err = glGetError();
    assert(err==GL_NO_ERROR);
    
	if (!stash)
	{
		fprintf(stderr, "Could not create stash.\n");
		return 0;
	}
#ifdef LOAD_FONT_FROM_FILE
	unsigned char* data;
	const char* fontPaths[]={
	"./",
	"../../bin/",
	"../bin/",
	"bin/"
	};

	int numPaths=sizeof(fontPaths)/sizeof(char*);
	
	// Load the first truetype font from memory (just because we can).
    
	FILE* fp = 0;
	const char* fontPath ="./";
	char fullFontFileName[1024];

	for (int i=0;i<numPaths;i++)
	{
		
		fontPath = fontPaths[i];
		//sprintf(fullFontFileName,"%s%s",fontPath,"OpenSans.ttf");//"DroidSerif-Regular.ttf");
		sprintf(fullFontFileName,"%s%s",fontPath,"DroidSerif-Regular.ttf");//OpenSans.ttf");//"DroidSerif-Regular.ttf");
		fp = fopen(fullFontFileName, "rb");
		if (fp)
			break;
	}

    err = glGetError();
    assert(err==GL_NO_ERROR);
    
    assert(fp);
    if (fp)
    {
        fseek(fp, 0, SEEK_END);
        datasize = (int)ftell(fp);
        fseek(fp, 0, SEEK_SET);
        data = (unsigned char*)malloc(datasize);
        if (data == NULL)
        {
            assert(0);
            return 0;
        }
        else
            fread(data, 1, datasize, fp);
        fclose(fp);
        fp = 0;
    }
	if (!(droidRegular = sth_add_font_from_memory(stash, data)))
    {
        assert(0);
        return 0;
    }
    err = glGetError();
    assert(err==GL_NO_ERROR);

	// Load the remaining truetype fonts directly.
    sprintf(fullFontFileName,"%s%s",fontPath,"DroidSerif-Italic.ttf");

	if (!(droidItalic = sth_add_font(stash,fullFontFileName)))
	{
        assert(0);
        return 0;
    }
     sprintf(fullFontFileName,"%s%s",fontPath,"DroidSerif-Bold.ttf");

	if (!(droidBold = sth_add_font(stash,fullFontFileName)))
	{
        assert(0);
        return 0;
    }
    err = glGetError();
    assert(err==GL_NO_ERROR);
    
     sprintf(fullFontFileName,"%s%s",fontPath,"DroidSansJapanese.ttf");
    if (!(droidJapanese = sth_add_font(stash,fullFontFileName)))
	{
        assert(0);
        return 0;
    }
#else//LOAD_FONT_FROM_FILE
	char* data2 = OpenSansData;
	unsigned char* data = (unsigned char*) data2;
	if (!(droidRegular = sth_add_font_from_memory(stash, data)))
	{
		printf("error!\n");
	}
#endif//LOAD_FONT_FROM_FILE
    err = glGetError();
    assert(err==GL_NO_ERROR);

	return stash;
}

int main(int argc, char* argv[])
{
		
	b3CommandLineArgs args(argc,argv);

	if (args.CheckCmdLineFlag("help"))
	{
		Usage();
		return 0;
	}

	args.GetCmdLineArgument("enable_interop", useInterop);
	printf("useInterop=%d\n",useInterop);



	args.GetCmdLineArgument("pause_simulation", pauseSimulation);
	printf("pause_simulation=%d\n",pauseSimulation);
	args.GetCmdLineArgument("x_dim", NUM_OBJECTS_X);
	args.GetCmdLineArgument("y_dim", NUM_OBJECTS_Y);
	args.GetCmdLineArgument("z_dim", NUM_OBJECTS_Z);

	args.GetCmdLineArgument("x_gap", X_GAP);
	args.GetCmdLineArgument("y_gap", Y_GAP);
	args.GetCmdLineArgument("z_gap", Z_GAP);
	printf("x_dim=%d, y_dim=%d, z_dim=%d\n",NUM_OBJECTS_X,NUM_OBJECTS_Y,NUM_OBJECTS_Z);
	printf("x_gap=%f, y_gap=%f, z_gap=%f\n",X_GAP,Y_GAP,Z_GAP);
	
	args.GetCmdLineArgument("enable_static", keepStaticObjects);
	printf("enable_static=%d\n",keepStaticObjects);	

	
	char* tmpfile = 0;
	args.GetCmdLineArgument("load_bulletfile", tmpfile );
	if (tmpfile)
		fileName = tmpfile;

	printf("load_bulletfile=%s\n",fileName);

	
	printf("\n");
	b3gDefaultOpenGLWindow* window = new b3gDefaultOpenGLWindow();
	b3gWindowConstructionInfo wci;
	wci.m_width = g_OpenGLWidth;
	wci.m_height = g_OpenGLHeight;
	
	window->createWindow(wci);
	window->setWindowTitle("render test");
	
	

    float retinaScale = 1;
    
#ifndef __APPLE__
	GLenum err = glewInit();
#else
    retinaScale = window->getRetinaScale();
#endif
    
    window->runMainLoop();
	window->startRendering();
	window->endRendering();

	int maxObjectCapacity=128*1024;
	GLInstancingRenderer render(maxObjectCapacity);

	GLPrimitiveRenderer* primRenderer = new GLPrimitiveRenderer(g_OpenGLWidth,g_OpenGLHeight);
	
	sth_stash* stash = initFont(primRenderer);
		
	render.InitShaders();


	createSceneProgrammatically(render);
    

	render.writeTransforms();

    window->runMainLoop();

	window->setMouseButtonCallback(MyMouseButtonCallback);
	window->setMouseMoveCallback(MyMouseMoveCallback);
	window->setResizeCallback(MyResizeCallback);
	window->setKeyboardCallback(b3DefaultKeyboardCallback);
    window->setWheelCallback(b3DefaultWheelCallback);


    //GLPrimitiveRenderer* pprender = new GLPrimitiveRenderer(g_OpenGLWidth,g_OpenGLHeight);
   
   glUseProgram(0); 

   	
   

////////////////////////////////

	/////////////////////////////////////


	if (pCanvas)
	{
		pCanvas->SetSize(g_OpenGLWidth,g_OpenGLHeight);
	}
	setupGUI(g_OpenGLWidth,g_OpenGLHeight,stash,retinaScale,primRenderer);
	class b3ProfileIterator* m_profileIterator;
	m_profileIterator = b3ProfileManager::Get_Iterator();

	glClearColor(1,1,1,1);
	while (!window->requestedExit())
	{
		b3ProfileManager::Reset();
	
		{
		B3_PROFILE("loop");




		{
			B3_PROFILE("startRendering");
			window->startRendering();
		}
		render.RenderScene();
		glFinish();
		float col[4]={0,1,0,1};
	//	pprender->drawRect(10,50,120,60,col);
//             glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		//glEnable(GL_TEXTURE_2D);
    
            float x = 10;
            float y=220;
            float  dx=0;
            if (1)
            {
                B3_PROFILE("font sth_draw_text");
                
				glEnable(GL_BLEND);
				GLint err = glGetError();
				assert(err==GL_NO_ERROR);

				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				err = glGetError();
				assert(err==GL_NO_ERROR);

				glDisable(GL_DEPTH_TEST);
				err = glGetError();
				assert(err==GL_NO_ERROR);
        

				glDisable(GL_CULL_FACE);

                sth_begin_draw(stash);
                sth_flush_draw(stash);
                sth_draw_text(stash, droidRegular,20.f, x, y, "Non-retina font rendering !@#$", &dx,g_OpenGLWidth,g_OpenGLHeight,0,1);//retinaScale);
                if (retinaScale!=1.f)
                    sth_draw_text(stash, droidRegular,20.f*retinaScale, x, y+20, "Retina font rendering!@#$", &dx,g_OpenGLWidth,g_OpenGLHeight,0,retinaScale);
                sth_flush_draw(stash);
                
                sth_end_draw(stash);
            }
    
			if (1)
		{
			B3_PROFILE("gwen RenderCanvas");
	
			if (pCanvas)
			{
				glEnable(GL_BLEND);
				GLint err = glGetError();
				assert(err==GL_NO_ERROR);

				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		
				err = glGetError();
				assert(err==GL_NO_ERROR);

				err = glGetError();
				assert(err==GL_NO_ERROR);
        
				glDisable(GL_DEPTH_TEST);
				err = glGetError();
				assert(err==GL_NO_ERROR);
        
				//glColor4ub(255,0,0,255);
		
				err = glGetError();
				assert(err==GL_NO_ERROR);
        
		
				err = glGetError();
				assert(err==GL_NO_ERROR);
				glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

			//	saveOpenGLState(width,height);//m_glutScreenWidth,m_glutScreenHeight);
			
				err = glGetError();
				assert(err==GL_NO_ERROR);

			
				err = glGetError();
				assert(err==GL_NO_ERROR);

				glDisable(GL_CULL_FACE);

				glDisable(GL_DEPTH_TEST);
				err = glGetError();
				assert(err==GL_NO_ERROR);

				err = glGetError();
				assert(err==GL_NO_ERROR);
            
				glEnable(GL_BLEND);

            
				err = glGetError();
				assert(err==GL_NO_ERROR);
            
 
            
				pCanvas->RenderCanvas();
				//restoreOpenGLState();
			}

		}

            
       
        

            
		window->endRendering();

		}


		
		b3ProfileManager::Increment_Frame_Counter();



		static bool printStats  = true;

		
		 if (printStats && !pauseSimulation)
		 {
			static int count = 0;
			count--;
			if (count<0)
			{
				count = 100;
				{
					//B3_PROFILE("processProfileData");
					processProfileData(m_profileIterator,false);
				}
				//b3ProfileManager::dumpAll();
				//printStats  = false;
			} else
			{
//				printf(".");
			}
		 }
		

	}

	//delete pprender;
//	render.CleanupShaders();
	window->closeWindow();

	delete window;
	
	
	
	return 0;


}
