//#include "OpenGLWindow/OpenGLInclude.h"
//#include "OpenGL/gl.h"
//#define USE_OPENGL2
#ifdef  USE_OPENGL2
#include "OpenGLWindow/SimpleOpenGL2App.h"
#else

#include "OpenGLWindow/SimpleOpenGL3App.h"
#endif


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
#include "OpenGLWindow/CommonRenderInterface.h"
#include "OpenGLWindow/SimpleCamera.h"

CommonGraphicsApp* app=0;
#ifdef USE_OPENGL2
struct TestRenderer : public CommonRenderInterface
{
	int m_width;
	int m_height;
	SimpleCamera	m_camera;

	TestRenderer(int width, int height)
		:m_width(width),
		m_height(height)
	{

	}
	virtual void init()
	{
		
	}
	virtual void updateCamera(int upAxis)
	{
		float projection[16];
		float view[16];
		m_camera.setAspectRatio((float)m_width/(float)m_height);
		m_camera.update();
		m_camera.getCameraProjectionMatrix(projection);
		m_camera.getCameraViewMatrix(view);
		GLfloat projMat[16];
		GLfloat viewMat[16];
		for (int i=0;i<16;i++)
		{
			viewMat[i] = view[i];
			projMat[i] = projection[i];
		}
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMultMatrixf(projMat);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMultMatrixf(viewMat);
	}
	virtual void removeAllInstances()
	{
	}
	virtual void setCameraDistance(float dist)
	{
		m_camera.setCameraDistance(dist);
	}
	virtual void setCameraPitch(float pitch)
	{
		m_camera.setCameraPitch(pitch);
	}
	virtual void setCameraTargetPosition(float x, float y, float z)
	{
		m_camera.setCameraTargetPosition(x,y,z);
	}

	virtual void	getCameraPosition(float cameraPos[4])
	{
		float pos[3];
		m_camera.getCameraPosition(pos);
		cameraPos[0] = pos[0];
		cameraPos[1] = pos[1];
		cameraPos[2] = pos[2];
		
	}
	virtual void	getCameraPosition(double cameraPos[4])
	{
		float pos[3];
		m_camera.getCameraPosition(pos);
		cameraPos[0] = pos[0];
		cameraPos[1] = pos[1];
		cameraPos[2] = pos[2];
	}

	virtual void	setCameraTargetPosition(float cameraPos[4])
	{
		m_camera.setCameraTargetPosition(cameraPos[0],cameraPos[1],cameraPos[2]);
	}
	virtual void	getCameraTargetPosition(float cameraPos[4]) const
	{
		m_camera.getCameraTargetPosition(cameraPos);
	}
    virtual void	getCameraTargetPosition(double cameraPos[4]) const
	{
			cameraPos[0] = 1;
		cameraPos[1] = 1;
		cameraPos[2] = 1;
	}

	virtual void renderScene()
	{
	}


	virtual int getScreenWidth()
	{
		return m_width;
	}
	virtual int getScreenHeight()
	{
		return m_height;
	}
	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling)
	{
		return 0;
	}
	virtual void drawLines(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, const unsigned int* indices, int numIndices, float pointDrawSize)
	{
		int pointStrideInFloats = pointStrideInBytes/4;
		glLineWidth(pointDrawSize);
		for (int i=0;i<numIndices;i+=2)
		{
			int index0 = indices[i];
			int index1 = indices[i+1];

			btVector3 fromColor(color[0],color[1],color[2]);
			btVector3 toColor(color[0],color[1],color[2]);
			
			btVector3 from(positions[index0*pointStrideInFloats],positions[index0*pointStrideInFloats+1],positions[index0*pointStrideInFloats+2]);
			btVector3 to(positions[index1*pointStrideInFloats],positions[index1*pointStrideInFloats+1],positions[index1*pointStrideInFloats+2]);

			glBegin(GL_LINES);
				glColor3f(fromColor.getX(), fromColor.getY(), fromColor.getZ());
				glVertex3d(from.getX(), from.getY(), from.getZ());
				glColor3f(toColor.getX(), toColor.getY(), toColor.getZ());
				glVertex3d(to.getX(), to.getY(), to.getZ());
			glEnd();
			
		}
	}
	virtual void drawLine(const float from[4], const float to[4], const float color[4], float lineWidth)
	{
		glLineWidth(lineWidth);
		glBegin(GL_LINES);
			glColor3f(color[0],color[1],color[2]);
			glVertex3d(from[0],from[1],from[2]);
			glVertex3d(to[0],to[1],to[2]);
		glEnd();
	}
	virtual int registerShape(const float* vertices, int numvertices, const int* indices, int numIndices,int primitiveType=B3_GL_TRIANGLES, int textureIndex=-1)
	{
		return 0;
	}

	virtual void writeSingleInstanceTransformToCPU(const float* position, const float* orientation, int srcIndex)
	{
	}
	virtual void writeSingleInstanceTransformToCPU(const double* position, const double* orientation, int srcIndex)
	{
	}
	virtual void writeTransforms()
	{
	}

};
#endif //USE_OPENGL2
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

static bool pauseSimulation=false;//true;
int midiBaseIndex = 176;

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

	if (key=='w' && state)
	{
		visualWireframe=!visualWireframe;
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

	
#ifdef USE_OPENGL2
	app = new SimpleOpenGL2App("AllBullet2Demos",width,height);
	app->m_renderer = new TestRenderer(width,height);
#else
	SimpleOpenGL3App* simpleApp = new SimpleOpenGL3App("AllBullet2Demos",width,height);
	app = simpleApp;
#endif
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
#ifdef USE_OPENGL2
	Gwen::Renderer::Base* gwenRenderer = new Gwen::Renderer::OpenGL_DebugFont();
#else
	sth_stash* fontstash=simpleApp->getFontStash();
	Gwen::Renderer::Base* gwenRenderer = new GwenOpenGL3CoreRenderer(simpleApp->m_primRenderer,fontstash,width,height,s_window->getRetinaScale(),myTexLoader);
#endif
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
                sCurrentDemo->physicsDebugDraw();
            }
		}

		static int toggle = 1;
		if (1)
		{
            if (!pauseSimulation)
                processProfileData(profWindow,false);
            {
#ifdef  USE_OPENGL2
				{
					saveOpenGLState(width,height);
				}
#endif
                BT_PROFILE("Draw Gwen GUI");
                gui->draw(s_instancingRenderer->getScreenWidth(),s_instancingRenderer->getScreenHeight());
#ifdef  USE_OPENGL2
		  restoreOpenGLState();
#endif
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
