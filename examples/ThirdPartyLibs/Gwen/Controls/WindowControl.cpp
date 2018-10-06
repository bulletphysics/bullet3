/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Controls/WindowControl.h"
#include "Gwen/Controls/ImagePanel.h"
#include "Gwen/Controls/Label.h"
#include "Gwen/Controls/Modal.h"

using namespace Gwen;
using namespace Gwen::Controls;
using namespace Gwen::ControlsInternal;

GWEN_CONTROL_CONSTRUCTOR(WindowControl)
{
	m_Modal = NULL;
	m_bInFocus = false;
	m_bDeleteOnClose = false;

	m_TitleBar = new Dragger(this);
	m_TitleBar->Dock(Pos::Top);
	m_TitleBar->SetHeight(18);
	m_TitleBar->SetPadding(Padding(0, 0, 0, 5));
	m_TitleBar->SetTarget(this);

	m_Title = new Label(m_TitleBar);
	m_Title->SetAlignment(Pos::Center);
	m_Title->SetText("Window");
	m_Title->SetTextColor(Gwen::Colors::White);
	m_Title->Dock(Pos::Fill);

	m_CloseButton = new Button(m_TitleBar);
	m_CloseButton->SetText("");
	m_CloseButton->SetSize(m_TitleBar->Height(), m_TitleBar->Height());
	m_CloseButton->Dock(Pos::Right);
	m_CloseButton->onPress.Add(this, &WindowControl::CloseButtonPressed);
	m_CloseButton->SetTabable(false);
	m_CloseButton->SetName("closeButton");

	//Create a blank content control, dock it to the top - Should this be a ScrollControl?
	m_InnerPanel = new Base(this);
	m_InnerPanel->Dock(Pos::Fill);

	BringToFront();

	SetTabable(false);
	Focus();

	SetMinimumSize(Gwen::Point(100, 40));
	SetClampMovement(true);
	SetKeyboardInputEnabled(false);
}

WindowControl::~WindowControl()
{
	if (m_Modal)
	{
		m_Modal->DelayedDelete();
	}
}

void WindowControl::MakeModal(bool invisible)
{
	if (m_Modal) return;

	m_Modal = new ControlsInternal::Modal(GetCanvas());
	SetParent(m_Modal);

	if (invisible)
	{
		m_Modal->SetShouldDrawBackground(false);
	}
}

bool WindowControl::IsOnTop()
{
	for (Base::List::reverse_iterator iter = GetParent()->Children.rbegin(); iter != GetParent()->Children.rend(); ++iter)
	{
		if (!*iter)
			continue;

		WindowControl* pWindow = (*iter)->DynamicCastWindowControl();

		if (!pWindow)
			continue;

		if (pWindow == this)
			return true;

		return false;
	}

	return false;
}

void WindowControl::Render(Skin::Base* skin)
{
	//This should use m_bInFocus but I need to figure out best way to make layout happen
	skin->DrawWindow(this, m_TitleBar->Bottom(), IsOnTop());
}

void WindowControl::RenderUnder(Skin::Base* skin)
{
	BaseClass::RenderUnder(skin);
	skin->DrawShadow(this);
}

void WindowControl::SetTitle(Gwen::UnicodeString title)
{
	m_Title->SetText(title);
}
void WindowControl::SetClosable(bool closeable)
{
	m_CloseButton->SetHidden(!closeable);
}

void WindowControl::SetHidden(bool hidden)
{
	if (!hidden)
		BringToFront();

	BaseClass::SetHidden(hidden);
}

void WindowControl::Touch()
{
	BaseClass::Touch();
	BringToFront();
	m_bInFocus = IsOnTop();
	//If Keyboard focus isn't one of our children, make it us
}

void WindowControl::CloseButtonPressed(Gwen::Controls::Base* /*pFromPanel*/)
{
	SetHidden(true);

	if (m_bDeleteOnClose)
		DelayedDelete();
}

void WindowControl::RenderFocus(Gwen::Skin::Base* /*skin*/)
{
}