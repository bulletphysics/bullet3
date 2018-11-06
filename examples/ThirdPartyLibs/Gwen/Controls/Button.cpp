/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/Button.h"
#include "Gwen/Controls/ImagePanel.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR(Button)
{
	m_Image = NULL;
	m_bDepressed = false;
	m_bCenterImage = false;

	SetSize(100, 20);
	SetMouseInputEnabled(true);
	SetIsToggle(false);
	SetAlignment(Gwen::Pos::Center);
	SetTextPadding(Padding(3, 0, 3, 0));
	m_bToggleStatus = false;
	SetKeyboardInputEnabled(false);
	SetTabable(false);
}

void Button::Render(Skin::Base* skin)
{
	if (ShouldDrawBackground())
	{
		bool bDrawDepressed = IsDepressed() && IsHovered();
		if (IsToggle()) bDrawDepressed = bDrawDepressed || GetToggleState();

		bool bDrawHovered = IsHovered() && ShouldDrawHover();

		skin->DrawButton(this, bDrawDepressed, bDrawHovered);
	}
}

void Button::OnMouseClickLeft(int /*x*/, int /*y*/, bool bDown)
{
	if (bDown)
	{
		m_bDepressed = true;
		Gwen::MouseFocus = this;
		onDown.Call(this);
	}
	else
	{
		if (IsHovered() && m_bDepressed)
		{
			OnPress();
		}

		m_bDepressed = false;
		Gwen::MouseFocus = NULL;
		onUp.Call(this);
	}

	Redraw();
}

void Button::OnPress()
{
	if (IsToggle())
	{
		SetToggleState(!GetToggleState());
	}

	onPress.Call(this);
}

void Button::SetImage(const TextObject& strName, bool bCenter)
{
	if (strName.GetUnicode() == L"")
	{
		if (m_Image)
		{
			delete m_Image;
			m_Image = NULL;
		}

		return;
	}

	if (!m_Image)
	{
		m_Image = new ImagePanel(this);
	}

	m_Image->SetImage(strName);
	m_Image->SizeToContents();
	m_Image->SetPos(m_Padding.left, 2);
	m_bCenterImage = bCenter;

	int IdealTextPadding = m_Image->Right() + m_Padding.left + 4;
	if (m_rTextPadding.left < IdealTextPadding)
	{
		m_rTextPadding.left = IdealTextPadding;
	}
}

void Button::SetToggleState(bool b)
{
	if (m_bToggleStatus == b) return;

	m_bToggleStatus = b;

	onToggle.Call(this);

	if (m_bToggleStatus)
	{
		onToggleOn.Call(this);
	}
	else
	{
		onToggleOff.Call(this);
	}
}

void Button::SizeToContents()
{
	BaseClass::SizeToContents();

	if (m_Image)
	{
		int height = m_Image->Height() + 4;
		if (Height() < height)
		{
			SetHeight(height);
		}
	}
}

bool Button::OnKeySpace(bool bDown)
{
	OnMouseClickLeft(0, 0, bDown);
	return true;
}

void Button::AcceleratePressed()
{
	OnPress();
}

void Button::Layout(Skin::Base* pSkin)
{
	BaseClass::Layout(pSkin);
	if (m_Image)
	{
		Gwen::Align::CenterVertically(m_Image);

		if (m_bCenterImage)
			Gwen::Align::CenterHorizontally(m_Image);
	}
}

void Button::OnMouseDoubleClickLeft(int x, int y)
{
	OnMouseClickLeft(x, y, true);
	onDoubleClick.Call(this);
};