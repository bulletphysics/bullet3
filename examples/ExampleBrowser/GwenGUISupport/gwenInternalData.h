#ifndef GWEN_INTERNAL_DATA_H
#define GWEN_INTERNAL_DATA_H

#include "../OpenGLWindow/GwenOpenGL3CoreRenderer.h"
#include "../OpenGLWindow/GLPrimitiveRenderer.h"
#include "Gwen/Platform.h"
#include "Gwen/Controls/TreeControl.h"
#include "Gwen/Controls/RadioButtonController.h"
#include "Gwen/Controls/VerticalSlider.h"
#include "Gwen/Controls/HorizontalSlider.h"
#include "Gwen/Controls/GroupBox.h"
#include "Gwen/Controls/CheckBox.h"
#include "Gwen/Controls/StatusBar.h"
#include "Gwen/Controls/Button.h"
#include "Gwen/Controls/ComboBox.h"
#include "Gwen/Controls/MenuStrip.h"
#include "Gwen/Controls/Slider.h"
#include "Gwen/Controls/Property/Text.h"
#include "Gwen/Controls/SplitterBar.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Gwen/Gwen.h"
#include "Gwen/Align.h"
#include "Gwen/Utility.h"
#include "Gwen/Controls/WindowControl.h"
#include "Gwen/Controls/TabControl.h"
#include "Gwen/Controls/ListBox.h"
#include "Gwen/Skins/Simple.h"
//#include "Gwen/Skins/TexturedBase.h"
#include "gwenUserInterface.h"


struct GwenInternalData
{
	//struct sth_stash;
	//class GwenOpenGL3CoreRenderer*	pRenderer;
	Gwen::Renderer::Base*			pRenderer;
	Gwen::Skin::Simple				skin;
	Gwen::Controls::Canvas*			pCanvas;
	//GLPrimitiveRenderer* m_primRenderer;
	Gwen::Controls::TabButton*		m_demoPage;
	Gwen::Controls::TabButton*		m_explorerPage;
	Gwen::Controls::TreeControl*	m_explorerTreeCtrl;
	Gwen::Controls::MenuItem*		m_viewMenu;
    class MyMenuItems*                    m_menuItems;
	Gwen::Controls::ListBox*		m_TextOutput;
	Gwen::Controls::Label*		m_exampleInfoGroupBox;
	Gwen::Controls::ListBox*			m_exampleInfoTextOutput;
	

	int		m_curYposition;

	Gwen::Controls::Label* m_rightStatusBar;
	Gwen::Controls::Label* m_leftStatusBar;
	b3AlignedObjectArray<class Gwen::Event::Handler*>	m_handlers;
	b3ToggleButtonCallback			m_toggleButtonCallback;
	b3ComboBoxCallback				m_comboBoxCallback;
	

};

#endif//GWEN_INTERNAL_DATA_H
