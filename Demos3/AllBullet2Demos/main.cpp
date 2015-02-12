#include "OpenGLWindow/OpenGLInclude.h"
//#include "OpenGL/gl.h"
//#define USE_OPENGL2

#include "OpenGLWindow/SimpleOpenGL2App.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"

#include "OpenGLWindow/CommonRenderInterface.h"
#ifdef __APPLE__
#include "OpenGLWindow/MacOpenGLWindow.h"
#else

#include "GL/glew.h"
#ifdef _WIN32
#include "OpenGLWindow/Win32OpenGLWindow.h"
#else
//let's cross the fingers it is Linux/X11
#include "OpenGLWindow/X11OpenGLWindow.h"
#endif //_WIN32
#endif//__APPLE__
#include "Gwen/Renderers/OpenGL_DebugFont.h"

#include "Bullet3Common/b3Vector3.h"
#include "assert.h"
#include <stdio.h>
#include "Bullet3AppSupport/gwenInternalData.h"
#include "Bullet3AppSupport/gwenUserInterface.h"
#include "BulletDemoEntries.h"
#include "Bullet3AppSupport/b3Clock.h"
#include "Bullet3AppSupport/GwenParameterInterface.h"
#include "Bullet3AppSupport/GwenProfileWindow.h"
#include "Bullet3AppSupport/GwenTextureWindow.h"
#include "Bullet3AppSupport/GraphingTexture.h"

#include "OpenGLWindow/SimpleCamera.h"
#include "OpenGLWindow/SimpleOpenGL2Renderer.h"

CommonGraphicsApp* app=0;

b3gWindowInterface* s_window = 0;
CommonParameterInterface*	s_parameterInterface=0;
CommonRenderInterface*	s_instancingRenderer=0;
#define DEMO_SELECTION_COMBOBOX 13
const char* startFileName = "bulletDemo.txt";

static GwenUserInterface* gui  = 0;
static int sCurrentDemoIndex = 0;
static int sCurrentHightlighted = 0;
static BulletDemoInterface* sCurrentDemo = 0;
static b3AlignedObjectArray<const char*> allNames;
bool drawGUI=true;
extern bool useShadowMap;
static bool visualWireframe=false;
static bool renderVisualGeometry=true;
static bool renderGrid = true;
int gDebugDrawFlags = 0;
static bool pauseSimulation=false;
int midiBaseIndex = 176;
extern bool gDisableDeactivation;

//#include <float.h>
//unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);







b3KeyboardCallback prevKeyboardCallback = 0;

void MyKeyboardCallback(int key, int state)
{

	//printf("key=%d, state=%d\n", key, state);
	bool handled = false;
	if (gui)
	{

		handled = gui->keyboardCallback(key, state);
	}
	if (!handled && sCurrentDemo)
	{
		handled = sCurrentDemo->keyboardCallback(key,state);
	}

	//checkout: is it desired to ignore keys, if the demo already handles them?
	//if (handled)
	//	return;

	if (key=='a' && state)
	{
		gDebugDrawFlags ^= btIDebugDraw::DBG_DrawAabb;
	}
	if (key=='c' && state)
	{
		gDebugDrawFlags ^= btIDebugDraw::DBG_DrawConstraints;
		gDebugDrawFlags ^= btIDebugDraw::DBG_DrawContactPoints;
	}
	if (key == 'd' && state)
	{
		gDebugDrawFlags ^= btIDebugDraw::DBG_NoDeactivation;
		gDisableDeactivation = ((gDebugDrawFlags & btIDebugDraw::DBG_NoDeactivation) != 0);
	}
	if (key=='l' && state)
	{
		gDebugDrawFlags ^= btIDebugDraw::DBG_DrawConstraintLimits;
	}
	if (key=='w' && state)
	{
		visualWireframe=!visualWireframe;
		gDebugDrawFlags ^= btIDebugDraw::DBG_DrawWireframe;
	}
	if (key=='v' && state)
	{
		renderVisualGeometry = !renderVisualGeometry;
	}
	if (key=='g' && state)
	{
		renderGrid = !renderGrid;
	}


	if (key=='i' && state)
	{
		pauseSimulation = !pauseSimulation;
	}

	if (key=='s' && state)
	{
		useShadowMap=!useShadowMap;
	}

	if (key==B3G_ESCAPE && s_window)
	{
		s_window->setRequestExit();
	}

	if (prevKeyboardCallback)
		prevKeyboardCallback(key,state);
}

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void MyMouseMoveCallback( float x, float y)
{
	bool handled = false;
	if (sCurrentDemo)
		handled = sCurrentDemo->mouseMoveCallback(x,y);
	if (!handled && gui)
		handled = gui->mouseMoveCallback(x,y);
	if (!handled)
	{
		if (prevMouseMoveCallback)
			prevMouseMoveCallback(x,y);
	}
}

b3MouseButtonCallback prevMouseButtonCallback  = 0;

static void MyMouseButtonCallback(int button, int state, float x, float y)
{
	bool handled = false;
	//try picking first
	if (sCurrentDemo)
		handled = sCurrentDemo->mouseButtonCallback(button,state,x,y);

	if (!handled && gui)
		handled = gui->mouseButtonCallback(button,state,x,y);

	if (!handled)
	{
		if (prevMouseButtonCallback )
			prevMouseButtonCallback (button,state,x,y);
	}
	//	b3DefaultMouseButtonCallback(button,state,x,y);
}

#include <string.h>

void openURDFDemo(const char* filename)
{
   
    if (sCurrentDemo)
    {
        sCurrentDemo->exitPhysics();
        s_instancingRenderer->removeAllInstances();
        delete sCurrentDemo;
        sCurrentDemo=0;
    }
    
    s_parameterInterface->removeAllParameters();
   
    ImportUrdfSetup* physicsSetup = new ImportUrdfSetup();
    physicsSetup->setFileName(filename);
    

    sCurrentDemo = new BasicDemo(app, physicsSetup);
	app->setUpAxis(2);
    
    if (sCurrentDemo)
    {
        sCurrentDemo->initPhysics();
    }

    
}

void selectDemo(int demoIndex)
{
	sCurrentDemoIndex = demoIndex;
	sCurrentHightlighted = demoIndex;
	int numDemos = sizeof(allDemos)/sizeof(BulletDemoEntry);
	if (demoIndex>numDemos)
	{
		demoIndex = 0;
	}
	if (sCurrentDemo)
	{
		sCurrentDemo->exitPhysics();
		s_instancingRenderer->removeAllInstances();
		delete sCurrentDemo;
		sCurrentDemo=0;
	}
	if (allDemos[demoIndex].m_createFunc)
	{
		s_parameterInterface->removeAllParameters();
		sCurrentDemo = (*allDemos[demoIndex].m_createFunc)(app);
		if (sCurrentDemo)
		{
			if (gui)
			{
				bool isLeft = true;
				gui->setStatusBarMessage("Status: OK", false);
			}
			sCurrentDemo->initPhysics();
		}
	}

}

void	MyComboBoxCallback(int comboId, const char* item)
{
	//printf("comboId = %d, item = %s\n",comboId, item);
	if (comboId==DEMO_SELECTION_COMBOBOX)
	{
		//find selected item
		for (int i=0;i<allNames.size();i++)
		{
			if (strcmp(item,allNames[i])==0)
			{
				selectDemo(i);
				saveCurrentDemoEntry(sCurrentDemoIndex,startFileName);
				break;
			}
		}
	}

}



void MyStatusBarPrintf(const char* msg)
{
	printf("b3Printf: %s\n", msg);
	if (gui)
	{
		bool isLeft = true;
		gui->setStatusBarMessage(msg,isLeft);
	}
}


void MyStatusBarWarning(const char* msg)
{
	printf("Warning: %s\n", msg);
	if (gui)
	{
		bool isLeft = false;
		gui->setStatusBarMessage(msg,isLeft);
	}
}

struct MyMenuItemHander :public Gwen::Event::Handler
{
	int					m_buttonId;

	MyMenuItemHander( int buttonId)
		:m_buttonId(buttonId)
	{
	}

	void onButtonA(Gwen::Controls::Base* pControl)
	{
		//const Gwen::String& name = pControl->GetName();
		Gwen::Controls::TreeNode* node = (Gwen::Controls::TreeNode*)pControl;
	//	Gwen::Controls::Label* l = node->GetButton();

		Gwen::UnicodeString la = node->GetButton()->GetText();// node->GetButton()->GetName();// GetText();
		Gwen::String laa = Gwen::Utility::UnicodeToString(la);
	//	const char* ha = laa.c_str();

		//printf("selected %s\n", ha);
		//int dep = but->IsDepressed();
		//int tog = but->GetToggleState();
//		if (m_data->m_toggleButtonCallback)
	//		(*m_data->m_toggleButtonCallback)(m_buttonId, tog);
	}
	void onButtonB(Gwen::Controls::Base* pControl)
	{
		Gwen::Controls::Label* label = (Gwen::Controls::Label*) pControl;
		Gwen::UnicodeString la = label->GetText();// node->GetButton()->GetName();// GetText();
		Gwen::String laa = Gwen::Utility::UnicodeToString(la);
		//const char* ha = laa.c_str();


		selectDemo(sCurrentHightlighted);
		saveCurrentDemoEntry(sCurrentDemoIndex, startFileName);
	}
	void onButtonC(Gwen::Controls::Base* pControl)
	{
//		Gwen::Controls::Label* label = (Gwen::Controls::Label*) pControl;
	//	Gwen::UnicodeString la = label->GetText();// node->GetButton()->GetName();// GetText();
	//	Gwen::String laa = Gwen::Utility::UnicodeToString(la);
	//	const char* ha = laa.c_str();


//		printf("onButtonC ! %s\n", ha);
	}
	void onButtonD(Gwen::Controls::Base* pControl)
	{
/*		Gwen::Controls::Label* label = (Gwen::Controls::Label*) pControl;
		Gwen::UnicodeString la = label->GetText();// node->GetButton()->GetName();// GetText();
		Gwen::String laa = Gwen::Utility::UnicodeToString(la);
		const char* ha = laa.c_str();
		*/

	//	printf("onKeyReturn ! \n");
		selectDemo(sCurrentHightlighted);
		saveCurrentDemoEntry(sCurrentDemoIndex, startFileName);

	}

	void onButtonE(Gwen::Controls::Base* pControl)
	{
	//	printf("select %d\n",m_buttonId);
		sCurrentHightlighted = m_buttonId;
	}

	void onButtonF(Gwen::Controls::Base* pControl)
	{
		//printf("selection changed!\n");
	}

	void onButtonG(Gwen::Controls::Base* pControl)
	{
		//printf("onButtonG !\n");
	}



};
#include "Bullet3Common/b3HashMap.h"

struct GL3TexLoader : public MyTextureLoader
{
	b3HashMap<b3HashString,GLint> m_hashMap;
	
	virtual void LoadTexture( Gwen::Texture* pTexture )
	{
		Gwen::String namestr = pTexture->name.Get();
		const char* n = namestr.c_str();
		GLint* texIdPtr = m_hashMap[n];
		if (texIdPtr)
		{
			pTexture->m_intData = *texIdPtr;
		}
	}
	virtual void FreeTexture( Gwen::Texture* pTexture )
	{
	}
};

void fileOpenCallback()
{

 char filename[1024];
 int len = s_window->fileOpenDialog(filename,1024);
 if (len)
 {
     //todo(erwincoumans) check if it is actually URDF
     //printf("file open:%s\n", filename);
     openURDFDemo(filename);
 }
}

extern float shadowMapWorldSize;
int main(int argc, char* argv[])
{
    shadowMapWorldSize = 25;

	b3Clock clock;

	//float dt = 1./120.f;
    int width = 1024;
    int height=768;

	//	wci.m_resizeCallback = MyResizeCallback;

    SimpleOpenGL3App* simpleApp=0;
    bool useOpenGL2=false;
    if (useOpenGL2)
    {
        app = new SimpleOpenGL2App("AllBullet2Demos",width,height);
        app->m_renderer = new SimpleOpenGL2Renderer(width,height);
    } else
    {
        simpleApp = new SimpleOpenGL3App("AllBullet2Demos",width,height);
        app = simpleApp;
    }
    
    s_instancingRenderer = app->m_renderer;
	s_window  = app->m_window;
	prevMouseMoveCallback  = s_window->getMouseMoveCallback();
	s_window->setMouseMoveCallback(MyMouseMoveCallback);
	
	prevMouseButtonCallback = s_window->getMouseButtonCallback();
	s_window->setMouseButtonCallback(MyMouseButtonCallback);
	prevKeyboardCallback = s_window->getKeyboardCallback();
	s_window->setKeyboardCallback(MyKeyboardCallback);

	app->m_renderer->setCameraDistance(13);
	app->m_renderer->setCameraPitch(0);
	app->m_renderer->setCameraTargetPosition(0,0,0);

	b3SetCustomWarningMessageFunc(MyStatusBarWarning);
	b3SetCustomPrintfFunc(MyStatusBarPrintf);
	

	/*
	SimpleOpenGL3App* app = new SimpleOpenGL3App("AllBullet2Demos",width,height);
	s_instancingRenderer->setCameraDistance(13);
	s_instancingRenderer->setCameraPitch(0);
	s_instancingRenderer->setCameraTargetPosition(0,0,0);
	s_window->setMouseMoveCallback(MyMouseMoveCallback);
	s_window->setMouseButtonCallback(MyMouseButtonCallback);
	s_window->setKeyboardCallback(MyKeyboardCallback);
	
	*/
    assert(glGetError()==GL_NO_ERROR);

	

	gui = new GwenUserInterface;
	GL3TexLoader* myTexLoader = new GL3TexLoader;
    
    Gwen::Renderer::Base* gwenRenderer = 0;
    if (useOpenGL2)
    {
        gwenRenderer = new Gwen::Renderer::OpenGL_DebugFont();
    } else
    {
        sth_stash* fontstash=simpleApp->getFontStash();
        gwenRenderer = new GwenOpenGL3CoreRenderer(simpleApp->m_primRenderer,fontstash,width,height,s_window->getRetinaScale(),myTexLoader);
    }
	//

	gui->init(width,height,gwenRenderer,s_window->getRetinaScale());
//	gui->getInternalData()->m_explorerPage
	Gwen::Controls::TreeControl* tree = gui->getInternalData()->m_explorerTreeCtrl;

	
	//gui->getInternalData()->pRenderer->setTextureLoader(myTexLoader);

	
	MyProfileWindow* profWindow = setupProfileWindow(gui->getInternalData());
	profileWindowSetVisible(profWindow,false);
	gui->setFocus();
#if 0
	{
		MyGraphInput input(gui->getInternalData());
		input.m_width=300;
		input.m_height=300;
		input.m_xPos = 0;
		input.m_yPos = height-input.m_height;
		input.m_name="Test Graph1";
		input.m_texName = "graph1";
		GraphingTexture* gt = new GraphingTexture;
		gt->create(256,256);
		int texId = gt->getTextureId();
		myTexLoader->m_hashMap.insert("graph1", texId);
		//MyGraphWindow* gw = 
		setupTextureWindow(input);
	}
	if (1)
	{
		MyGraphInput input(gui->getInternalData());
		input.m_width=300;
		input.m_height=300;
		input.m_xPos = width-input.m_width;
		input.m_yPos = height-input.m_height;
		input.m_name="Test Graph2";
		input.m_texName = "graph2";
		GraphingTexture* gt = new GraphingTexture;
		int texWidth = 512;
		int texHeight = 512;
		gt->create(texWidth,texHeight);
		for (int i=0;i<texWidth;i++)
		{
			for (int j=0;j<texHeight;j++)
			{
				gt->setPixel(i,j,0,0,0,255);
			}
		}
		gt->uploadImageData();
		
		int texId = gt->getTextureId();
		input.m_xPos = width-input.m_width;
		myTexLoader->m_hashMap.insert("graph2", texId);
		//MyGraphWindow* gw = 
		setupTextureWindow(input);
	}
	//destroyTextureWindow(gw);
#endif 
	s_parameterInterface  = app->m_parameterInterface = new GwenParameterInterface(gui->getInternalData());
	
	//gui->getInternalData()->m_demoPage;

	int numDemos = sizeof(allDemos)/sizeof(BulletDemoEntry);

	//char nodeText[1024];
	//int curDemo = 0;
	int selectedDemo = loadCurrentDemoEntry(startFileName);
	Gwen::Controls::TreeNode* curNode = tree;
	MyMenuItemHander* handler2 = new MyMenuItemHander(-1);

	tree->onReturnKeyDown.Add(handler2, &MyMenuItemHander::onButtonD);

	for (int d = 0; d<numDemos; d++)
	{
//		sprintf(nodeText, "Node %d", i);
		Gwen::UnicodeString nodeUText = Gwen::Utility::StringToUnicode(allDemos[d].m_name);
		if (allDemos[d].m_menuLevel==1)
		{
			Gwen::Controls::TreeNode* pNode = curNode->AddNode(nodeUText);
			if (d == selectedDemo)
			{
				pNode->SetSelected(true);
				tree->ExpandAll();
				selectDemo(d);


			}
			MyMenuItemHander* handler = new MyMenuItemHander(d);
			pNode->onNamePress.Add(handler, &MyMenuItemHander::onButtonA);
			pNode->GetButton()->onDoubleClick.Add(handler, &MyMenuItemHander::onButtonB);
			pNode->GetButton()->onDown.Add(handler, &MyMenuItemHander::onButtonC);
			pNode->onSelect.Add(handler, &MyMenuItemHander::onButtonE);
			pNode->onReturnKeyDown.Add(handler, &MyMenuItemHander::onButtonG);
			pNode->onSelectChange.Add(handler, &MyMenuItemHander::onButtonF);
//			pNode->onKeyReturn.Add(handler, &MyMenuItemHander::onButtonD);
//			pNode->GetButton()->onKeyboardReturn.Add(handler, &MyMenuItemHander::onButtonD);
	//		pNode->onNamePress.Add(handler, &MyMenuItemHander::onButtonD);
//			pNode->onKeyboardPressed.Add(handler, &MyMenuItemHander::onButtonD);
//			pNode->OnKeyPress
		}
		 else
		 {
			 curNode = tree->AddNode(nodeUText);
		 }
	}

/*	for (int i=0;i<numDemos;i++)
	{
		allNames.push_back(allDemos[i].m_name);
	}
	*/
	//selectDemo(loadCurrentDemoEntry(startFileName));
	/*
	gui->registerComboBox(DEMO_SELECTION_COMBOBOX,allNames.size(),&allNames[0],sCurrentDemoIndex);

	//const char* names2[] = {"comboF", "comboG","comboH"};
	//gui->registerComboBox(2,3,&names2[0],0);

	gui->setComboBoxCallback(MyComboBoxCallback);
	*/
	unsigned long int	prevTimeInMicroseconds = clock.getTimeMicroseconds();

    gui->registerFileOpenCallback(fileOpenCallback);
    
	do
	{

		assert(glGetError()==GL_NO_ERROR);
		s_instancingRenderer->init();
        DrawGridData dg;
        dg.upAxis = app->getUpAxis();

        {
            BT_PROFILE("Update Camera");
            s_instancingRenderer->updateCamera(dg.upAxis);
        }

		if (renderGrid)
        {
            BT_PROFILE("Draw Grid");
            app->drawGrid(dg);
        }
		static int frameCount = 0;
		frameCount++;

		if (0)
		{
            BT_PROFILE("Draw frame counter");
            char bla[1024];
            sprintf(bla,"Frame %d", frameCount);
            app->drawText(bla,10,10);
		}

		if (sCurrentDemo)
		{
			if (!pauseSimulation)
			{
				unsigned long int	curTimeInMicroseconds = clock.getTimeMicroseconds();
				unsigned long int diff = curTimeInMicroseconds-prevTimeInMicroseconds;
				float deltaTimeInSeconds = (diff)*1.e-6;
				//printf("---------------------------------------------------\n");
				//printf("Framecount = %d\n",frameCount);

				sCurrentDemo->stepSimulation(deltaTimeInSeconds);//1./60.f);
				prevTimeInMicroseconds = curTimeInMicroseconds;
			}
			
			if (renderVisualGeometry)
            {
				if (visualWireframe)
				{
					glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
				}
                BT_PROFILE("Render Scene");
                sCurrentDemo->renderScene();
            }
            {
				glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
                sCurrentDemo->physicsDebugDraw(gDebugDrawFlags);
            }
		}

		static int toggle = 1;
		if (1)
		{
            if (!pauseSimulation)
                processProfileData(profWindow,false);
            {
                if (useOpenGL2)
				{
					saveOpenGLState(width,height);
				}
                BT_PROFILE("Draw Gwen GUI");
                gui->draw(s_instancingRenderer->getScreenWidth(),s_instancingRenderer->getScreenHeight());
                if (useOpenGL2)
                {
                    restoreOpenGLState();
                }
            }
		}
		toggle=1-toggle;
        {
            BT_PROFILE("Sync Parameters");
            s_parameterInterface->syncParameters();
        }
        {
            BT_PROFILE("Swap Buffers");
            app->swapBuffer();
        }

		gui->forceUpdateScrollBars();
	} while (!s_window->requestedExit());

//	selectDemo(0);
	delete gui;
	delete app;
	return 0;
}
