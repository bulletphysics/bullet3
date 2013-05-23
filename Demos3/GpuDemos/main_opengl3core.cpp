
//#include "GpuDemo.h"

#ifdef _WIN32
#include <Windows.h> //for GetLocalTime/GetSystemTime
#endif

#ifdef __APPLE__
#include "OpenGLWindow/MacOpenGLWindow.h"
#elif defined _WIN32
#include "OpenGLWindow/Win32OpenGLWindow.h"
#elif defined __linux
#include "OpenGLWindow/X11OpenGLWindow.h"
#endif

#include "OpenGLWindow/GLPrimitiveRenderer.h"
#include "OpenGLWindow/GLInstancingRenderer.h"
//#include "OpenGL3CoreRenderer.h"
#include "Bullet3Common/b3Quickprof.h"
//#include "b3GpuDynamicsWorld.h"
#include <assert.h>
#include <string.h>
#include "OpenGLTrueTypeFont/fontstash.h"
#include "OpenGLTrueTypeFont/opengl_fontstashcallbacks.h"
#include "gwenUserInterface.h"
#include "ParticleDemo.h"
#include "broadphase/PairBench.h"
#include "rigidbody/GpuRigidBodyDemo.h"
#include "rigidbody/ConcaveScene.h"
#include "rigidbody/GpuConvexScene.h"
#include "rigidbody/GpuCompoundScene.h"
#include "rigidbody/GpuSphereScene.h"
#include "rigidbody/Bullet2FileDemo.h"
#include "softbody/GpuSoftBodyDemo.h"

//#include "BroadphaseBenchmark.h"

int g_OpenGLWidth=1024;
int g_OpenGLHeight = 768;
bool dump_timings = false;
extern char OpenSansData[];

static void MyResizeCallback( float width, float height)
{
	g_OpenGLWidth = width;
	g_OpenGLHeight = height;
}

b3gWindowInterface* window=0;
GwenUserInterface* gui  = 0;
bool gPause = true;
bool gReset = false;

enum
{
	MYPAUSE=1,
	MYPROFILE=2,
	MYRESET,
};

enum
{
	MYCOMBOBOX1 = 1,
};

b3AlignedObjectArray<const char*> demoNames;
int selectedDemo = 0;
GpuDemo::CreateFunc* allDemos[]=
{
//		ConcaveCompound2Scene::MyCreateFunc,
		GpuConvexScene::MyCreateFunc,
	GpuBoxPlaneScene::MyCreateFunc,
	GpuConvexPlaneScene::MyCreateFunc,

	ConcaveSphereScene::MyCreateFunc,

	GpuCompoundScene::MyCreateFunc,



	ConcaveSphereScene::MyCreateFunc,

	ConcaveScene::MyCreateFunc,




	ConcaveCompoundScene::MyCreateFunc,

	GpuCompoundPlaneScene::MyCreateFunc,

	GpuSphereScene::MyCreateFunc,

	GpuSoftClothDemo::MyCreateFunc,

	Bullet2FileDemo::MyCreateFunc,

	PairBench::MyCreateFunc,


	//GpuRigidBodyDemo::MyCreateFunc,

	//BroadphaseBenchmark::CreateFunc,
	//GpuBoxDemo::CreateFunc,



	//ParticleDemo::MyCreateFunc,



	//GpuCompoundDemo::CreateFunc,
	//EmptyDemo::CreateFunc,
};


void	MyComboBoxCallback(int comboId, const char* item)
{
	int numDemos = demoNames.size();
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


}
void	MyButtonCallback(int buttonId, int state)
{
	switch (buttonId)
	{
	case MYPAUSE:
		{
		gPause =!gPause;
		break;
		}
	case MYPROFILE:
		{
		dump_timings = !dump_timings;
		break;
		}
	case MYRESET:
		{
		gReset=!gReset;
		break;
		}
	default:
		{
			printf("hello\n");
		}
	}
}

static void MyMouseMoveCallback( float x, float y)
{
	if (gui)
	{
		bool handled = gui ->mouseMoveCallback(x,y);
		if (!handled)
			b3DefaultMouseMoveCallback(x,y);
	}
}
static void MyMouseButtonCallback(int button, int state, float x, float y)
{
	if (gui)
	{
		bool handled = gui->mouseButtonCallback(button,state,x,y);
		if (!handled)
			b3DefaultMouseButtonCallback(button,state,x,y);
	}
}


void MyKeyboardCallback(int key, int state)
{
	if (key==B3G_ESCAPE && window)
	{
		window->setRequestExit();
	}
	b3DefaultKeyboardCallback(key,state);
}



bool enableExperimentalCpuConcaveCollision=false;




	int droidRegular=0;//, droidItalic, droidBold, droidJapanese, dejavu;

sth_stash* stash=0;

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
	unsigned char* data=0;
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




#include "OpenGLWindow/OpenGLInclude.h"
#include "Bullet3Common/b3CommandLineArgs.h"

void Usage()
{
	printf("\nprogram.exe [--selected_demo=<int>] [--cl_device=<int>] [--benchmark] [--disable_opencl] [--cl_platform=<int>]  [--x_dim=<int>] [--y_dim=<num>] [--z_dim=<int>] [--x_gap=<float>] [--y_gap=<float>] [--z_gap=<float>] [--use_concave_mesh] [--new_batching]\n");
};


void	DumpSimulationTime(FILE* f)
{
	b3ProfileIterator* profileIterator = b3ProfileManager::Get_Iterator();

	profileIterator->First();
	if (profileIterator->Is_Done())
		return;

	float accumulated_time=0,parent_time = profileIterator->Is_Root() ? b3ProfileManager::Get_Time_Since_Reset() : profileIterator->Get_Current_Parent_Total_Time();
	int i;
	int frames_since_reset = b3ProfileManager::Get_Frame_Count_Since_Reset();

	//fprintf(f,"%.3f,",	parent_time );
	float totalTime = 0.f;



	static bool headersOnce = true;

	if (headersOnce)
	{
		headersOnce = false;
		fprintf(f,"root,");

			for (i = 0; !profileIterator->Is_Done(); i++,profileIterator->Next())
		{
			float current_total_time = profileIterator->Get_Current_Total_Time();
			accumulated_time += current_total_time;
			float fraction = parent_time > B3_EPSILON ? (current_total_time / parent_time) * 100 : 0.f;
			const char* name = profileIterator->Get_Current_Name();
			fprintf(f,"%s,",name);
		}
		fprintf(f,"\n");
	}


	fprintf(f,"%.3f,",parent_time);
	profileIterator->First();
	for (i = 0; !profileIterator->Is_Done(); i++,profileIterator->Next())
	{
		float current_total_time = profileIterator->Get_Current_Total_Time();
		accumulated_time += current_total_time;
		float fraction = parent_time > B3_EPSILON ? (current_total_time / parent_time) * 100 : 0.f;
		const char* name = profileIterator->Get_Current_Name();
		//if (!strcmp(name,"stepSimulation"))
		{
			fprintf(f,"%.3f,",current_total_time);
		}
		totalTime += current_total_time;
		//recurse into children
	}

	fprintf(f,"\n");


	b3ProfileManager::Release_Iterator(profileIterator);


}
///extern const char* g_deviceName;
const char* g_deviceName = "blaat";
extern bool useNewBatchingKernel;
#include "Bullet3Common/b3Vector3.h"

int main(int argc, char* argv[])
{

	b3Vector3 test(1,2,3);
	test.x = 1;
	test.y = 4;

    printf("main start");

	b3CommandLineArgs args(argc,argv);
	ParticleDemo::ConstructionInfo ci;

	if (args.CheckCmdLineFlag("help"))
	{
		Usage();
		return 0;
	}


	args.GetCmdLineArgument("selected_demo",selectedDemo);


	if (args.CheckCmdLineFlag("new_batching"))
	{
		useNewBatchingKernel = true;
	}
	bool benchmark=args.CheckCmdLineFlag("benchmark");
	dump_timings=args.CheckCmdLineFlag("dump_timings");
	ci.useOpenCL = !args.CheckCmdLineFlag("disable_opencl");
	ci.m_useConcaveMesh = true;//args.CheckCmdLineFlag("use_concave_mesh");
	if (ci.m_useConcaveMesh)
	{
		enableExperimentalCpuConcaveCollision = true;
	}

	args.GetCmdLineArgument("cl_device", ci.preferredOpenCLDeviceIndex);
	args.GetCmdLineArgument("cl_platform", ci.preferredOpenCLPlatformIndex);
	args.GetCmdLineArgument("x_dim", ci.arraySizeX);
	args.GetCmdLineArgument("y_dim", ci.arraySizeY);
	args.GetCmdLineArgument("z_dim", ci.arraySizeZ);
	args.GetCmdLineArgument("x_gap", ci.gapX);
	args.GetCmdLineArgument("y_gap", ci.gapY);
	args.GetCmdLineArgument("z_gap", ci.gapZ);


	printf("Demo settings:\n");
	printf("x_dim=%d, y_dim=%d, z_dim=%d\n",ci.arraySizeX,ci.arraySizeY,ci.arraySizeZ);
	printf("x_gap=%f, y_gap=%f, z_gap=%f\n",ci.gapX,ci.gapY,ci.gapZ);

	printf("Preferred cl_device index %d\n", ci.preferredOpenCLDeviceIndex);
	printf("Preferred cl_platform index%d\n", ci.preferredOpenCLPlatformIndex);
	printf("-----------------------------------------------------\n");

	#ifndef B3_NO_PROFILE
	b3ProfileManager::Reset();
#endif //B3_NO_PROFILE


	window = new b3gDefaultOpenGLWindow();

	b3gWindowConstructionInfo wci(g_OpenGLWidth,g_OpenGLHeight);

	window->createWindow(wci);
	window->setResizeCallback(MyResizeCallback);
	window->setMouseMoveCallback(MyMouseMoveCallback);
	window->setMouseButtonCallback(MyMouseButtonCallback);
	window->setKeyboardCallback(MyKeyboardCallback);

	window->setWindowTitle("Bullet 3.x GPU Rigid Body http://bulletphysics.org");
	printf("-----------------------------------------------------\n");


#ifndef __APPLE__
	glewInit();
#endif

	gui = new GwenUserInterface();

    printf("started GwenUserInterface");


	GLPrimitiveRenderer prim(g_OpenGLWidth,g_OpenGLHeight);

	stash = initFont(&prim);


	gui->init(g_OpenGLWidth,g_OpenGLHeight,stash,window->getRetinaScale());

    printf("init fonts");


	gui->setToggleButtonCallback(MyButtonCallback);

	gui->registerToggleButton(MYPAUSE,"Pause");
	gui->registerToggleButton(MYPROFILE,"Profile");
	gui->registerToggleButton(MYRESET,"Reset");






	int numItems = sizeof(allDemos)/sizeof(ParticleDemo::CreateFunc*);
	demoNames.clear();
	for (int i=0;i<numItems;i++)
	{
		GpuDemo* demo = allDemos[i]();
		demoNames.push_back(demo->getName());
		delete demo;
	}

	gui->registerComboBox(MYCOMBOBOX1,numItems,&demoNames[0]);
	gui->setComboBoxCallback(MyComboBoxCallback);



	do
	{
		bool syncOnly = false;
		gReset = false;




	static bool once=true;




	glClearColor(1,0,0,1);
	glClear(GL_COLOR_BUFFER_BIT);

	{
		window->startRendering();
		glFinish();




		float color[4] = {1,1,1,1};
		prim.drawRect(0,0,200,200,color);
		float retinaScale = 1;

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

		gui->draw(g_OpenGLWidth,g_OpenGLHeight);
		window->endRendering();
		glFinish();
	}
	once=false;

//	OpenGL3CoreRenderer render;

	glClearColor(0,1,0,1);
	glClear(GL_COLOR_BUFFER_BIT);

	window->endRendering();

	glFinish();



	window->setWheelCallback(b3DefaultWheelCallback);




	{
		GpuDemo* demo = allDemos[selectedDemo]();
//		demo->myinit();
		bool useGpu = false;


		int maxObjectCapacity=256*1024;

		ci.m_instancingRenderer = new GLInstancingRenderer(maxObjectCapacity);//render.getInstancingRenderer();
		ci.m_window = window;
		ci.m_gui = gui;
		ci.m_instancingRenderer->init();
		ci.m_instancingRenderer->InitShaders();

//		render.init();

		demo->initPhysics(ci);
		printf("-----------------------------------------------------\n");

		FILE* f = 0;
		if (benchmark)
		{
			gPause = false;
			char fileName[1024];

#ifdef _WIN32
			SYSTEMTIME time;
			GetLocalTime(&time);
			char buf[1024];
			DWORD dwCompNameLen = 1024;
			if (0 != GetComputerName(buf, &dwCompNameLen))
			{
				printf("%s", buf);
			} else
			{
				printf("unknown", buf);
			}
			sprintf(fileName,"%s_%s_%s_%d_%d_%d_date_%d-%d-%d_time_%d-%d-%d.csv",g_deviceName,buf,demoNames[selectedDemo],ci.arraySizeX,ci.arraySizeY,ci.arraySizeZ,time.wDay,time.wMonth,time.wYear,time.wHour,time.wMinute,time.wSecond);

			printf("Open file %s\n", fileName);
#else
			sprintf(fileName,"%s_%d_%d_%d.csv",g_deviceName,ci.arraySizeX,ci.arraySizeY,ci.arraySizeZ);
			printf("Open file %s\n", fileName);
#endif


			//GetSystemTime(&time2);

			f=fopen(fileName,"w");
			//if (f)
			//	fprintf(f,"%s (%dx%dx%d=%d),\n",  g_deviceName,ci.arraySizeX,ci.arraySizeY,ci.arraySizeZ,ci.arraySizeX*ci.arraySizeY*ci.arraySizeZ);
		}

		printf("-----------------------------------------------------\n");
		do
		{
			b3ProfileManager::Reset();
			b3ProfileManager::Increment_Frame_Counter();

//			render.reshape(g_OpenGLWidth,g_OpenGLHeight);

			window->startRendering();

			glClearColor(0.6,0.6,0.6,1);
			glClear(GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);
			glEnable(GL_DEPTH_TEST);


			if (!gPause)
			{
				B3_PROFILE("clientMoveAndDisplay");

				demo->clientMoveAndDisplay();
			}
			else
			{

			}

			{
				B3_PROFILE("renderScene");
				demo->renderScene();
			}


			/*if (demo->getDynamicsWorld() && demo->getDynamicsWorld()->getNumCollisionObjects())
			{
				B3_PROFILE("renderPhysicsWorld");
				b3AlignedObjectArray<b3CollisionObject*> arr = demo->getDynamicsWorld()->getCollisionObjectArray();
				b3CollisionObject** colObjArray = &arr[0];

				render.renderPhysicsWorld(demo->getDynamicsWorld()->getNumCollisionObjects(),colObjArray, syncOnly);
				syncOnly = true;

			}
			*/
			{
				B3_PROFILE("gui->draw");
				gui->draw(g_OpenGLWidth,g_OpenGLHeight);
			}
			{
				B3_PROFILE("window->endRendering");
				window->endRendering();
			}
			{
				B3_PROFILE("glFinish");
			}


		if (dump_timings)
			b3ProfileManager::dumpAll();

		if (f)
		{
			static int count=0;

			if (count>2 && count<102)
			{
				DumpSimulationTime(f);
			}
			if (count>=102)
				window->setRequestExit();
			count++;
		}



		} while (!window->requestedExit() && !gReset);


		demo->exitPhysics();
		b3ProfileManager::CleanupMemory();
		delete demo;
		if (f)
			fclose(f);
	}



	} while (gReset);


	gui->setComboBoxCallback(0);
	delete gui;
	gui=0;

	window->closeWindow();
	delete window;
	window = 0;

	return 0;
}
