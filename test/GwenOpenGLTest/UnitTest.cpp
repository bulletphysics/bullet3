/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "UnitTest.h"
#include "Gwen/Platform.h"
#include "Gwen/Controls/TreeControl.h"

using namespace Gwen;

#define ADD_UNIT_TEST(name)                                          \
	GUnit* RegisterUnitTest_##name(Gwen::Controls::TabControl* tab); \
	RegisterUnitTest_##name(m_TabControl)->SetUnitTest(this);

GWEN_CONTROL_CONSTRUCTOR(UnitTest)
{
	SetTitle(L"GWEN Unit Test");

	SetSize(600, 450);

	m_TabControl = new Controls::TabControl(this);
	m_TabControl->Dock(Pos::Fill);
	m_TabControl->SetMargin(Margin(2, 2, 2, 2));

	m_TextOutput = new Controls::ListBox(this);
	m_TextOutput->Dock(Pos::Bottom);
	m_TextOutput->SetHeight(100);

	ADD_UNIT_TEST(ImagePanel);

	//ADD_UNIT_TEST( MenuStrip );

	Gwen::UnicodeString str1(L"testje");
	Gwen::Controls::TabButton* tab = m_TabControl->AddPage(str1);

	Gwen::Controls::TreeControl* ctrl = 0;

	{
		ctrl = new Gwen::Controls::TreeControl(tab->GetPage());

		ctrl->SetKeyboardInputEnabled(true);
		ctrl->AddNode(L"Node One");
		{
			Gwen::Controls::TreeNode* pNode = ctrl->AddNode(L"Node Two");
			pNode->AddNode(L"Node Two Inside");
			pNode->AddNode(L"Eyes");
		}
		{
			Gwen::Controls::TreeNode* pNode = ctrl->AddNode(L"Node Two");
			pNode->AddNode(L"Node Two Inside");
			pNode->AddNode(L"Eyes");
		}
		{
			Gwen::Controls::TreeNode* pNode = ctrl->AddNode(L"Node Two");
			pNode->AddNode(L"Node Two Inside");
			pNode->AddNode(L"Eyes");
		}
		{
			Gwen::Controls::TreeNode* pNode = ctrl->AddNode(L"Node Two");
			pNode->AddNode(L"Node Two Inside");
			pNode->AddNode(L"Eyes");
		}
		{
			Gwen::Controls::TreeNode* pNode = ctrl->AddNode(L"Node Two");
			pNode->AddNode(L"Node Two Inside");
			pNode->AddNode(L"Eyes");
		}
		{
			Gwen::Controls::TreeNode* pNode = ctrl->AddNode(L"Node Two");
			pNode->AddNode(L"Node Two Inside");
			pNode->AddNode(L"Eyes");
			pNode->SetSelected(true);

			pNode->AddNode(L"Brown")->AddNode(L"Node Two Inside")->AddNode(L"Eyes")->AddNode(L"Brown");
		}
		ctrl->AddNode(L"Node Three");
		ctrl->Focus();
		ctrl->SetKeyboardInputEnabled(true);

		ctrl->SetBounds(30, 30, 200, 30 + 16 * 10);
		//ctrl->ExpandAll();
		ctrl->ForceUpdateScrollBars();
		ctrl->OnKeyDown(true);
	}

	//GUnit* u = new TreeControl2(m_TabControl);..Gwen::Controls::TreeControl2( m_TabControl );
	//GUnit* RegisterUnitTest_TreeControl2( Gwen::Controls::TabControl* tab );\
	//RegisterUnitTest_TreeControl2( m_TabControl )->SetUnitTest( this );

	//#define DEFINE_UNIT_TEST( name, displayname )
	//GUnit* RegisterUnitTest_TreeControl2( Gwen::Controls::TabControl* tab )
	//{
	//	GUnit* u = new TreeControl2( tab );
	//	tab->AddPage( displayname, u );
	//	return u;
	//}

	//ADD_UNIT_TEST( TreeControl2 );

	ADD_UNIT_TEST(Properties2);

	ADD_UNIT_TEST(TabControl2);
	ADD_UNIT_TEST(ScrollControl);
	ADD_UNIT_TEST(MenuStrip);
	ADD_UNIT_TEST(Numeric);
	ADD_UNIT_TEST(ComboBox);
	ADD_UNIT_TEST(TextBox);
	ADD_UNIT_TEST(ListBox);
	ADD_UNIT_TEST(Slider);
	ADD_UNIT_TEST(ProgressBar);
	ADD_UNIT_TEST(RadioButton2);

	ADD_UNIT_TEST(Label);
	ADD_UNIT_TEST(Checkbox);
	ADD_UNIT_TEST(Button);

	ADD_UNIT_TEST(CrossSplitter);

	ADD_UNIT_TEST(PanelListPanel);
	ADD_UNIT_TEST(GroupBox2);

	ADD_UNIT_TEST(StatusBar);

	ctrl->Focus();
	PrintText(L"Unit Test Started.\n");

	m_fLastSecond = Gwen::Platform::GetTimeInSeconds();
	m_iFrames = 0;
}

void UnitTest::PrintText(const Gwen::UnicodeString& str)
{
	m_TextOutput->AddItem(str);
	m_TextOutput->Scroller()->ScrollToBottom();
}

void UnitTest::Render(Gwen::Skin::Base* skin)
{
	m_iFrames++;

	if (m_fLastSecond < Gwen::Platform::GetTimeInSeconds())
	{
		SetTitle(Gwen::Utility::Format(L"GWEN Unit Test - %i fps", m_iFrames));

		m_fLastSecond = Gwen::Platform::GetTimeInSeconds() + 1.0f;
		m_iFrames = 0;
	}

	BaseClass::Render(skin);
}

void GUnit::UnitPrint(const Gwen::UnicodeString& str)
{
	m_pUnitTest->PrintText(str);
}

void GUnit::UnitPrint(const Gwen::String& str)
{
	UnitPrint(Utility::StringToUnicode(str));
}
