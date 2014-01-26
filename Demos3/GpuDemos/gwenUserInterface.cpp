
#include "gwenUserInterface.h"
#include "gwenInternalData.h"

GwenUserInterface::GwenUserInterface()
{
	m_data = new GwenInternalData();
	m_data->m_toggleButtonCallback = 0;
	m_data->m_comboBoxCallback = 0;

}
		
GwenUserInterface::~GwenUserInterface()
{
	for (int i=0;i<m_data->m_handlers.size();i++)
	{
		delete m_data->m_handlers[i];
	}

	m_data->m_handlers.clear();


	delete m_data->pCanvas;

	GLPrimitiveRenderer* prim = m_data->m_primRenderer;
	GwenOpenGL3CoreRenderer* coreRend = m_data->pRenderer;

	delete m_data;

	delete prim;
	delete coreRend;

}
		

struct MyTestMenuBar : public Gwen::Controls::MenuStrip
{
	


	MyTestMenuBar(Gwen::Controls::Base* pParent)
		:Gwen::Controls::MenuStrip(pParent)
	{
//		Gwen::Controls::MenuStrip* menu = new Gwen::Controls::MenuStrip( pParent );
		{
			Gwen::Controls::MenuItem* pRoot = AddItem( L"File" );
		
			pRoot = AddItem( L"View" );
//			Gwen::Event::Handler* handler =	GWEN_MCALL(&MyTestMenuBar::MenuItemSelect );
			pRoot->GetMenu()->AddItem( L"Profiler");//,,m_profileWindow,(Gwen::Event::Handler::Function)&MyProfileWindow::MenuItemSelect);

/*			pRoot->GetMenu()->AddItem( L"New", L"test16.png", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			pRoot->GetMenu()->AddItem( L"Load", L"test16.png", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			pRoot->GetMenu()->AddItem( L"Save", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			pRoot->GetMenu()->AddItem( L"Save As..", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			pRoot->GetMenu()->AddItem( L"Quit", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			*/
		}
	}

};

void	GwenUserInterface::resize(int width, int height)
{
	m_data->pCanvas->SetSize(width,height);
}


struct MyComboBoxHander   :public Gwen::Event::Handler
{
	GwenInternalData*	m_data;
	int					m_buttonId;

	MyComboBoxHander  (GwenInternalData* data, int buttonId)
		:m_data(data),
		m_buttonId(buttonId)
	{
	}

	void onSelect( Gwen::Controls::Base* pControl )
	{
		Gwen::Controls::ComboBox* but = (Gwen::Controls::ComboBox*) pControl;
		
		
		
		Gwen::String str = Gwen::Utility::UnicodeToString(	but->GetSelectedItem()->GetText());
		
		if (m_data->m_comboBoxCallback)
			(*m_data->m_comboBoxCallback)(m_buttonId,str.c_str());
	}

};

struct MyButtonHander   :public Gwen::Event::Handler
{
	GwenInternalData*	m_data;
	int					m_buttonId;

	MyButtonHander  (GwenInternalData* data, int buttonId)
		:m_data(data),
		m_buttonId(buttonId)
	{
	}

	void onButtonA( Gwen::Controls::Base* pControl )
	{
		Gwen::Controls::Button* but = (Gwen::Controls::Button*) pControl;
		int dep = but->IsDepressed();
		int tog = but->GetToggleState();
		if (m_data->m_toggleButtonCallback)
			(*m_data->m_toggleButtonCallback)(m_buttonId,tog);
	}

};


void	GwenUserInterface::setStatusBarMessage(const char* message, bool isLeft)
{
	Gwen::UnicodeString msg = Gwen::Utility::StringToUnicode(message);
	if (isLeft)
	{
		m_data->m_leftStatusBar->SetText( msg);
	} else
	{
		m_data->m_rightStatusBar->SetText( msg);
		
	}
}

void	GwenUserInterface::init(int width, int height,struct sth_stash* stash,float retinaScale)
{
	m_data->m_curYposition = 20;
	m_data->m_primRenderer = new GLPrimitiveRenderer(width,height);
	m_data->pRenderer = new GwenOpenGL3CoreRenderer(m_data->m_primRenderer,stash,width,height,retinaScale);

	m_data->skin.SetRender( m_data->pRenderer );

	m_data->pCanvas= new Gwen::Controls::Canvas( &m_data->skin );
	m_data->pCanvas->SetSize( width,height);
	m_data->pCanvas->SetDrawBackground( false);
	m_data->pCanvas->SetBackgroundColor( Gwen::Color( 150, 170, 170, 255 ) );

	MyTestMenuBar* menubar = new MyTestMenuBar(m_data->pCanvas);
	Gwen::Controls::StatusBar* bar = new Gwen::Controls::StatusBar(m_data->pCanvas);
	m_data->m_rightStatusBar = new Gwen::Controls::Label( bar );
	m_data->m_rightStatusBar->SetWidth(width/2);
	//m_data->m_rightStatusBar->SetText( L"Label Added to Right" );
	bar->AddControl( m_data->m_rightStatusBar, true );

	m_data->m_leftStatusBar = new Gwen::Controls::Label( bar );
	//m_data->m_leftStatusBar->SetText( L"Label Added to Left" );
	m_data->m_leftStatusBar->SetWidth(width/2);
	bar->AddControl( m_data->m_leftStatusBar,false);

	/*Gwen::Controls::GroupBox* box = new Gwen::Controls::GroupBox(m_data->pCanvas);
	box->SetText("text");
	box->SetName("name");
	box->SetHeight(500);
	*/
	Gwen::Controls::ScrollControl* windowLeft= new Gwen::Controls::ScrollControl(m_data->pCanvas);
	windowLeft->Dock(Gwen::Pos::Right);
	windowLeft->SetWidth(150);
	windowLeft->SetHeight(250);
	windowLeft->SetScroll(false,true);

	/*Gwen::Controls::WindowControl* windowLeft = new Gwen::Controls::WindowControl(m_data->pCanvas);
	windowLeft->Dock(Gwen::Pos::Left);
	windowLeft->SetTitle("title");
	windowLeft->SetWidth(150);
	windowLeft->SetClosable(false);
	windowLeft->SetShouldDrawBackground(true);
	windowLeft->SetTabable(true);
	*/

	//windowLeft->SetSkin(
	Gwen::Controls::TabControl* tab = new Gwen::Controls::TabControl(windowLeft);
	
	//tab->SetHeight(300);
	tab->SetWidth(140);
	tab->SetHeight(250);
	//tab->Dock(Gwen::Pos::Left);
	tab->Dock( Gwen::Pos::Fill );
	//tab->SetMargin( Gwen::Margin( 2, 2, 2, 2 ) );

	Gwen::UnicodeString str1(L"Main");
	m_data->m_demoPage = tab->AddPage(str1);

	

	
//	Gwen::UnicodeString str2(L"OpenCL");
//	tab->AddPage(str2);
	//Gwen::UnicodeString str3(L"page3");
//	tab->AddPage(str3);
	
		
	
	//but->onPress.Add(handler, &MyHander::onButtonA);

	
	
	//box->Dock(Gwen::Pos::Left);

	/*Gwen::Controls::WindowControl* windowBottom = new Gwen::Controls::WindowControl(m_data->pCanvas);
	windowBottom->SetHeight(100);
	windowBottom->Dock(Gwen::Pos::Bottom);
	windowBottom->SetTitle("bottom");
	*/
//	Gwen::Controls::Property::Text* prop = new Gwen::Controls::Property::Text(m_data->pCanvas);
	//prop->Dock(Gwen::Pos::Bottom);
	/*Gwen::Controls::SplitterBar* split = new Gwen::Controls::SplitterBar(m_data->pCanvas);
	split->Dock(Gwen::Pos::Center);
	split->SetHeight(300);
	split->SetWidth(300);
	*/
	/*
	
	
	*/
}
	
b3ToggleButtonCallback	GwenUserInterface::getToggleButtonCallback()
{
	return m_data->m_toggleButtonCallback;
}

void	GwenUserInterface::setToggleButtonCallback(b3ToggleButtonCallback callback)
{
	m_data->m_toggleButtonCallback = callback;
}
void	GwenUserInterface::registerToggleButton(int buttonId, const char* name)
{
	assert(m_data);
	assert(m_data->m_demoPage);

	Gwen::Controls::Button* but = new Gwen::Controls::Button(m_data->m_demoPage->GetPage());
	
	///some heuristic to find the button location
	int ypos = m_data->m_curYposition;
	but->SetPos(10, ypos );
	but->SetWidth( 100 );
	//but->SetBounds( 200, 30, 300, 200 );
	
	MyButtonHander* handler = new MyButtonHander(m_data, buttonId);
	m_data->m_handlers.push_back(handler);
	m_data->m_curYposition+=22;
	but->onToggle.Add(handler, &MyButtonHander::onButtonA);
	but->SetIsToggle(true);
	but->SetToggleState(false);
	but->SetText(name);

}

void	GwenUserInterface::setComboBoxCallback(b3ComboBoxCallback callback)
{
	m_data->m_comboBoxCallback = callback;
}

b3ComboBoxCallback GwenUserInterface::getComboBoxCallback()
{
	return m_data->m_comboBoxCallback;
}
void	GwenUserInterface::registerComboBox(int comboboxId, int numItems, const char** items, int startItem)
{
	Gwen::Controls::ComboBox* combobox = new Gwen::Controls::ComboBox(m_data->m_demoPage->GetPage());
	MyComboBoxHander* handler = new MyComboBoxHander(m_data, comboboxId);
	m_data->m_handlers.push_back(handler);
	
	combobox->onSelection.Add(handler,&MyComboBoxHander::onSelect);
	int ypos = m_data->m_curYposition;
	combobox->SetPos(10, ypos );
	combobox->SetWidth( 100 );
	//box->SetPos(120,130);
	for (int i=0;i<numItems;i++)
	{
		Gwen::Controls::MenuItem* item = combobox->AddItem(Gwen::Utility::StringToUnicode(items[i]));
		if (i==startItem)
			combobox->OnItemSelected(item);
	}

	m_data->m_curYposition+=22;
	

	
}

void	GwenUserInterface::draw(int width, int height)
{
	
//	printf("width = %d, height=%d\n", width,height);
	if (m_data->pCanvas)
	{
		m_data->pCanvas->SetSize(width,height);
		m_data->m_primRenderer->setScreenSize(width,height);
		m_data->pRenderer->resize(width,height);
		m_data->pCanvas->RenderCanvas();
		//restoreOpenGLState();
	}

}

bool	GwenUserInterface::mouseMoveCallback( float x, float y)
{
	bool handled  = false;

	static int m_lastmousepos[2] = {0,0};
	static bool isInitialized = false;
	if (m_data->pCanvas)
	{
		if (!isInitialized)
		{
			isInitialized = true;
			m_lastmousepos[0] = x+1;
			m_lastmousepos[1] = y+1;
		}
		handled = m_data->pCanvas->InputMouseMoved(x,y,m_lastmousepos[0],m_lastmousepos[1]);
	}
	return handled;
	
}
bool	GwenUserInterface::mouseButtonCallback(int button, int state, float x, float y)
{
	bool handled = false;
	if (m_data->pCanvas)
	{
		handled = m_data->pCanvas->InputMouseMoved(x,y,x, y);

		if (button>=0)
		{
			handled = m_data->pCanvas->InputMouseButton(button,state);
			if (handled)
			{
				//if (!state)
				//	return false;
			}
		}
	}
	return handled;
}
