#include "GwenTextureWindow.h"
#include "gwenUserInterface.h"
#include "gwenInternalData.h"
#include "Gwen/Controls/ImagePanel.h"

class MyGraphWindow : public Gwen::Controls::WindowControl
{
	Gwen::Controls::ImagePanel* m_imgPanel;

public:
	class MyMenuItems2* m_menuItems;

	MyGraphWindow(const MyGraphInput& input)
		: Gwen::Controls::WindowControl(input.m_data->pCanvas),
		  m_menuItems(0)
	{
		Gwen::UnicodeString str = Gwen::Utility::StringToUnicode(input.m_name);
		SetTitle(str);

		SetPos(input.m_xPos, input.m_yPos);
		SetSize(12 + input.m_width + 2 * input.m_borderWidth, 30 + input.m_height + 2 * input.m_borderWidth);

		m_imgPanel = new Gwen::Controls::ImagePanel(this);
		if (input.m_texName)
		{
			Gwen::UnicodeString texName = Gwen::Utility::StringToUnicode(input.m_texName);
			m_imgPanel->SetImage(texName);
		}
		m_imgPanel->SetBounds(input.m_borderWidth, input.m_borderWidth,
							  input.m_width,
							  input.m_height);
		//		this->Dock( Gwen::Pos::Bottom);
	}
	virtual ~MyGraphWindow()
	{
		delete m_imgPanel;
	}
};

class MyMenuItems2 : public Gwen::Controls::Base
{
	MyGraphWindow* m_graphWindow;

public:
	Gwen::Controls::MenuItem* m_item;

	MyMenuItems2(MyGraphWindow* graphWindow)
		: Gwen::Controls::Base(0),
		  m_graphWindow(graphWindow),
		  m_item(0)
	{
	}

	void MenuItemSelect(Gwen::Controls::Base* pControl)
	{
		if (m_graphWindow->Hidden())
		{
			m_graphWindow->SetHidden(false);
			//@TODO(erwincoumans) setCheck/SetCheckable drawing is broken, need to see what's wrong
			//			if (m_item)
			//				m_item->SetCheck(false);
		}
		else
		{
			m_graphWindow->SetHidden(true);
			//			if (m_item)
			//				m_item->SetCheck(true);
		}
	}
};

MyGraphWindow* setupTextureWindow(const MyGraphInput& input)
{
	MyGraphWindow* graphWindow = new MyGraphWindow(input);
	MyMenuItems2* menuItems = new MyMenuItems2(graphWindow);
	graphWindow->m_menuItems = menuItems;

	Gwen::UnicodeString str = Gwen::Utility::StringToUnicode(input.m_name);
	menuItems->m_item = input.m_data->m_viewMenu->GetMenu()->AddItem(str, menuItems, (Gwen::Event::Handler::Function)&MyMenuItems2::MenuItemSelect);
	//	menuItems->m_item->SetCheckable(true);

	return graphWindow;
}

void destroyTextureWindow(MyGraphWindow* window)
{
	delete window->m_menuItems->m_item;
	delete window->m_menuItems;
	delete window;
}
