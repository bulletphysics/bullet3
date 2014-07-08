#include "../../btgui/OpenGLWindow/SimpleOpenGL3App.h"
#include "Bullet3Common/b3Vector3.h"
#include "assert.h"
#include <stdio.h>
#include "../GpuDemos/gwenInternalData.h"
#include "../GpuDemos/gwenUserInterface.h"
#include "BulletDemoEntries.h"
#include "../../btgui/Timing/b3Clock.h"
#define DEMO_SELECTION_COMBOBOX 13
const char* startFileName = "bulletDemo.txt";
static SimpleOpenGL3App* app=0;
static GwenUserInterface* gui  = 0;
static int sCurrentDemoIndex = 0;
static int sCurrentHightlighted = 0;
static BulletDemoInterface* sCurrentDemo = 0;
static b3AlignedObjectArray<const char*> allNames;
double motorA=0,motorB=0;


bool drawGUI=true;
extern bool useShadowMap;
static bool wireframe=false;
static bool pauseSimulation=false;
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
		const Gwen::String& name = pControl->GetName();
		Gwen::Controls::TreeNode* node = (Gwen::Controls::TreeNode*)pControl;
		Gwen::Controls::Label* l = node->GetButton();

		Gwen::UnicodeString la = node->GetButton()->GetText();// node->GetButton()->GetName();// GetText();
		Gwen::String laa = Gwen::Utility::UnicodeToString(la);
		const char* ha = laa.c_str();

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
		const char* ha = laa.c_str();


		selectDemo(sCurrentHightlighted);
		saveCurrentDemoEntry(sCurrentDemoIndex, startFileName);
	}
	void onButtonC(Gwen::Controls::Base* pControl)
	{
		Gwen::Controls::Label* label = (Gwen::Controls::Label*) pControl;
		Gwen::UnicodeString la = label->GetText();// node->GetButton()->GetName();// GetText();
		Gwen::String laa = Gwen::Utility::UnicodeToString(la);
		const char* ha = laa.c_str();


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



template<typename T>
struct MySliderEventHandler : public Gwen::Event::Handler
{
	Gwen::Controls::TextBox* m_label;
	Gwen::Controls::Slider* m_pSlider;
	char m_variableName[1024];
	T* m_targetValue;

	MySliderEventHandler(const char* varName, Gwen::Controls::TextBox* label, Gwen::Controls::Slider* pSlider,T* target)
		:m_label(label),
		m_pSlider(pSlider),
		m_targetValue(target)
	{
		memcpy(m_variableName,varName,strlen(varName)+1);
	}


	void SliderMoved( Gwen::Controls::Base* pControl )
	{
		Gwen::Controls::Slider* pSlider = (Gwen::Controls::Slider*)pControl;
		//printf("value = %f\n", pSlider->GetValue());//UnitPrint( Utility::Format( L"Slider Value: %.2f", pSlider->GetValue() ) );
		float bla = pSlider->GetValue();
		T v = T(bla);
		SetValue(v);

	}

	void	SetValue(T v)
	{
		if (v < m_pSlider->GetRangeMin())
		{
			printf("?\n");
		}

		if (v > m_pSlider->GetRangeMax())
		{
						printf("?\n");

		}
		m_pSlider->SetValue(v,true);
		(*m_targetValue) = v;
		int val = int(v);//todo: specialize on template type
		char txt[1024];
		sprintf(txt,"%s : %d", m_variableName,val);
		m_label->SetText(txt);

	}
};
void MyParameter(const char* name, GwenInternalData* data, double* param)
{
	Gwen::Controls::TextBox* label = new Gwen::Controls::TextBox(data->m_demoPage->GetPage());
	//m_data->m_myControls.push_back(label);
	label->SetText( name);
	label->SetPos( 10, 10 + 25 );
	label->SetWidth(110);
	label->SetPos(10,data->m_curYposition);
	data->m_curYposition+=22;

	Gwen::Controls::HorizontalSlider* pSlider = new Gwen::Controls::HorizontalSlider( data->m_demoPage->GetPage());
	//m_data->m_myControls.push_back(pSlider);
	pSlider->SetPos( 10, data->m_curYposition );
	pSlider->SetSize( 100, 20 );
	pSlider->SetRange( -10, 10 );
	pSlider->SetNotchCount(10);
	pSlider->SetClampToNotches( true );
	pSlider->SetValue( *param);//dimensions[i] );
	char labelName[1024];
	sprintf(labelName,"%s",name);//axisNames[0]);
	MySliderEventHandler<double>* handler = new MySliderEventHandler<double>(labelName,label,pSlider,param);
	pSlider->onValueChanged.Add( handler, &MySliderEventHandler<double>::SliderMoved );
	handler->SliderMoved(pSlider);
	float v = pSlider->GetValue();
	data->m_curYposition+=22;
}

extern float shadowMapWorldSize;
int main(int argc, char* argv[])
{
    shadowMapWorldSize = 25;

	b3Clock clock;

	float dt = 1./120.f;
	int width = 1024;
	int height=768;

	app = new SimpleOpenGL3App("AllBullet2Demos",width,height);
	app->m_instancingRenderer->setCameraDistance(13);
	app->m_instancingRenderer->setCameraPitch(0);
	app->m_instancingRenderer->setCameraTargetPosition(b3MakeVector3(0,0,0));
	app->m_window->setMouseMoveCallback(MyMouseMoveCallback);
	app->m_window->setMouseButtonCallback(MyMouseButtonCallback);
	app->m_window->setKeyboardCallback(MyKeyboardCallback);

	GLint err = glGetError();
    assert(err==GL_NO_ERROR);

	sth_stash* fontstash=app->getFontStash();
	gui = new GwenUserInterface;
	gui->init(width,height,fontstash,app->m_window->getRetinaScale());
//	gui->getInternalData()->m_explorerPage
	Gwen::Controls::TreeControl* tree = gui->getInternalData()->m_explorerTreeCtrl;

	//gui->getInternalData()->m_demoPage;

	int numDemos = sizeof(allDemos)/sizeof(BulletDemoEntry);

	char nodeText[1024];
	int curDemo = 0;
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

	MyParameter("Motor A",gui->getInternalData(),&motorA);
	MyParameter("Motor B",gui->getInternalData(),&motorB);

	do
	{

		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera();

		app->drawGrid();

		static int frameCount = 0;
		frameCount++;

		if (0)
		{
		char bla[1024];

		sprintf(bla,"Simple test frame %d", frameCount);

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
			sCurrentDemo->renderScene();
		}

		static int toggle = 1;
		if (1)
		{
		gui->draw(app->m_instancingRenderer->getScreenWidth(),app->m_instancingRenderer->getScreenHeight());
		}
		toggle=1-toggle;
		app->swapBuffer();
	} while (!app->m_window->requestedExit());

//	selectDemo(0);
	delete gui;
	delete app;
	return 0;
}
