
#include "../../btgui/OpenGLWindow/SimpleOpenGL3App.h"
#include "Bullet3Common/b3Vector3.h"
#include "assert.h"
#include <stdio.h>
#include "../GpuDemos/gwenInternalData.h"
#include "../GpuDemos/gwenUserInterface.h"
#include "BulletDemoEntries.h"
#include "../../btgui/Timing/b3Clock.h"
#include "GwenParameterInterface.h"
#include "GwenProfileWindow.h"
#include "GwenTextureWindow.h"
#include "GraphingTexture.h"


#define DEMO_SELECTION_COMBOBOX 13
const char* startFileName = "bulletDemo.txt";
static SimpleOpenGL3App* app=0;
static GwenUserInterface* gui  = 0;
static int sCurrentDemoIndex = 0;
static int sCurrentHightlighted = 0;
static BulletDemoInterface* sCurrentDemo = 0;
static b3AlignedObjectArray<const char*> allNames;
bool drawGUI=true;
extern bool useShadowMap;
static bool wireframe=false;
static bool pauseSimulation=false;//true;
int midiBaseIndex = 176;

//#include <float.h>
//unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);

#ifdef B3_USE_MIDI
#include "../../btgui/MidiTest/RtMidi.h"
bool chooseMidiPort( RtMidiIn *rtmidi )
{
    if (!rtmidi)
        return false;
    
    std::string portName;
    unsigned int i = 0, nPorts = rtmidi->getPortCount();
    if ( nPorts == 0 ) {
        std::cout << "No input ports available!" << std::endl;
        return false;
    }
    
    if ( nPorts == 1 ) {
        std::cout << "\nOpening " << rtmidi->getPortName() << std::endl;
    }
    else {
        for ( i=0; i<nPorts; i++ ) {
            portName = rtmidi->getPortName(i);
            std::cout << "  Input port #" << i << ": " << portName << '\n';
        }
        
        do {
            std::cout << "\nChoose a port number: ";
            std::cin >> i;
        } while ( i >= nPorts );
    }
    
    //  std::getline( std::cin, keyHit );  // used to clear out stdin
    rtmidi->openPort( i );
    
    return true;
}
void PairMidiCallback( double deltatime, std::vector< unsigned char > *message, void *userData )
{
    unsigned int nBytes = message->size();
    if (nBytes==3)
    {
        //if ( message->at(1)==16)
        {
            printf("midi %d at %f = %d\n", message->at(1), deltatime, message->at(2));
            //test->SetValue(message->at(2));
#if KORG_MIDI
            if (message->at(0)>=midiBaseIndex && message->at(0)<(midiBaseIndex+8))
            {
                int sliderIndex = message->at(0)-midiBaseIndex;//-16;
                printf("sliderIndex = %d\n", sliderIndex);
                float sliderValue = message->at(2);
                printf("sliderValue = %f\n", sliderValue);
                app->m_parameterInterface->setSliderValue(sliderIndex,sliderValue);
            }
#else
            //ICONTROLS
            if (message->at(0)==176)
            {
                int sliderIndex = message->at(1)-32;//-16;
                printf("sliderIndex = %d\n", sliderIndex);
                float sliderValue = message->at(2);
                printf("sliderValue = %f\n", sliderValue);
                app->m_parameterInterface->setSliderValue(sliderIndex,sliderValue);
            }

#endif
            
        }
    }
}

#endif //B3_USE_MIDI






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
		wireframe=!wireframe;
		if (wireframe)
		{
			glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		} else
		{
			glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		}
	}
	if (key=='i' && state)
	{
		pauseSimulation = !pauseSimulation;
	}

	if (key=='s' && state)
	{
		useShadowMap=!useShadowMap;
	}

	if (key==B3G_ESCAPE && app && app->m_window)
	{
		app->m_window->setRequestExit();
	}

	b3DefaultKeyboardCallback(key,state);

}

static void MyMouseMoveCallback( float x, float y)
{
	bool handled = false;
	if (sCurrentDemo)
		handled = sCurrentDemo->mouseMoveCallback(x,y);
	if (!handled && gui)
		handled = gui->mouseMoveCallback(x,y);
	if (!handled)
		b3DefaultMouseMoveCallback(x,y);
}
static void MyMouseButtonCallback(int button, int state, float x, float y)
{
	bool handled = false;
	//try picking first
	if (sCurrentDemo)
		handled = sCurrentDemo->mouseButtonCallback(button,state,x,y);

	if (!handled && gui)
		handled = gui->mouseButtonCallback(button,state,x,y);

	if (!handled)
		b3DefaultMouseButtonCallback(button,state,x,y);
}

#include <string.h>

void openURDFDemo(const char* filename)
{
   
    if (sCurrentDemo)
    {
        sCurrentDemo->exitPhysics();
        app->m_instancingRenderer->removeAllInstances();
        delete sCurrentDemo;
        sCurrentDemo=0;
    }
    
    app->m_parameterInterface->removeAllParameters();
   
    ImportUrdfDemo* physicsSetup = new ImportUrdfDemo();
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
		app->m_instancingRenderer->removeAllInstances();
		delete sCurrentDemo;
		sCurrentDemo=0;
	}
	if (allDemos[demoIndex].m_createFunc && app)
	{
		app->m_parameterInterface->removeAllParameters();
		sCurrentDemo = (*allDemos[demoIndex].m_createFunc)(app);
		if (sCurrentDemo)
		{
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
 int len = app->m_window->fileOpenDialog(filename,1024);
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

	
	app = new SimpleOpenGL3App("AllBullet2Demos",width,height);
	app->m_instancingRenderer->setCameraDistance(13);
	app->m_instancingRenderer->setCameraPitch(0);
	app->m_instancingRenderer->setCameraTargetPosition(b3MakeVector3(0,0,0));
	app->m_window->setMouseMoveCallback(MyMouseMoveCallback);
	app->m_window->setMouseButtonCallback(MyMouseButtonCallback);
	app->m_window->setKeyboardCallback(MyKeyboardCallback);


    assert(glGetError()==GL_NO_ERROR);

	sth_stash* fontstash=app->getFontStash();
	gui = new GwenUserInterface;
	gui->init(width,height,fontstash,app->m_window->getRetinaScale());
//	gui->getInternalData()->m_explorerPage
	Gwen::Controls::TreeControl* tree = gui->getInternalData()->m_explorerTreeCtrl;

	GL3TexLoader* myTexLoader = new GL3TexLoader;
	gui->getInternalData()->pRenderer->setTextureLoader(myTexLoader);

	
	MyProfileWindow* profWindow = setupProfileWindow(gui->getInternalData());
	profileWindowSetVisible(profWindow,false);

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
	
	
	app->m_parameterInterface = new GwenParameterInterface(gui->getInternalData());

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
		app->m_instancingRenderer->init();
        DrawGridData dg;
        dg.upAxis = app->getUpAxis();

        {
            BT_PROFILE("Update Camera");
            app->m_instancingRenderer->updateCamera(dg.upAxis);
        }
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
            {
                BT_PROFILE("Render Scene");
                sCurrentDemo->renderScene();
            }
            {
                sCurrentDemo->physicsDebugDraw();
            }
		}

		static int toggle = 1;
		if (1)
		{
            if (!pauseSimulation)
                processProfileData(profWindow,false);
            {
                BT_PROFILE("Draw Gwen GUI");
                gui->draw(app->m_instancingRenderer->getScreenWidth(),app->m_instancingRenderer->getScreenHeight());
            }
		}
		toggle=1-toggle;
        {
            BT_PROFILE("Sync Parameters");
            app->m_parameterInterface->syncParameters();
        }
        {
            BT_PROFILE("Swap Buffers");
            app->swapBuffer();
        }
	} while (!app->m_window->requestedExit());

//	selectDemo(0);
	delete gui;
	delete app;
	return 0;
}
