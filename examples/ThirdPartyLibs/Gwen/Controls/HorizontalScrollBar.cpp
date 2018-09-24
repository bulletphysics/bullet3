/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Controls/ScrollBar.h"
#include "Gwen/Controls/HorizontalScrollBar.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR(HorizontalScrollBar)
{
	m_Bar->SetHorizontal();

	m_ScrollButton[SCROLL_BUTTON_LEFT]->SetDirectionLeft();
	m_ScrollButton[SCROLL_BUTTON_LEFT]->onPress.Add(this, &HorizontalScrollBar::NudgeLeft);

	m_ScrollButton[SCROLL_BUTTON_RIGHT]->SetDirectionRight();
	m_ScrollButton[SCROLL_BUTTON_RIGHT]->onPress.Add(this, &HorizontalScrollBar::NudgeRight);

	m_Bar->onDragged.Add(this, &HorizontalScrollBar::OnBarMoved);
}

void HorizontalScrollBar::Layout(Skin::Base* skin)
{
	BaseClass::Layout(skin);

	m_ScrollButton[SCROLL_BUTTON_LEFT]->SetWidth(Height());
	m_ScrollButton[SCROLL_BUTTON_LEFT]->Dock(Pos::Left);

	m_ScrollButton[SCROLL_BUTTON_RIGHT]->SetWidth(Height());
	m_ScrollButton[SCROLL_BUTTON_RIGHT]->Dock(Pos::Right);

	m_Bar->SetHeight(GetButtonSize());
	m_Bar->SetPadding(Padding(GetButtonSize(), 0, GetButtonSize(), 0));

	float barWidth = (m_fViewableContentSize / m_fContentSize) * (Width() - (GetButtonSize() * 2));

	if (barWidth < GetButtonSize() * 0.5)
		barWidth = GetButtonSize() * 0.5;

	m_Bar->SetWidth(barWidth);
	m_Bar->SetHidden(Width() - (GetButtonSize() * 2) <= barWidth);

	//Based on our last scroll amount, produce a position for the bar
	if (!m_Bar->IsDepressed())
	{
		SetScrolledAmount(GetScrolledAmount(), true);
	}
}

void HorizontalScrollBar::NudgeLeft(Base* /*control*/)
{
	if (!IsDisabled())
		SetScrolledAmount(GetScrolledAmount() - GetNudgeAmount(), true);
}

void HorizontalScrollBar::NudgeRight(Base* /*control*/)
{
	if (!IsDisabled())
		SetScrolledAmount(GetScrolledAmount() + GetNudgeAmount(), true);
}

void HorizontalScrollBar::ScrollToLeft()
{
	SetScrolledAmount(0, true);
}
void HorizontalScrollBar::ScrollToRight()
{
	SetScrolledAmount(1, true);
}

float HorizontalScrollBar::GetNudgeAmount()
{
	if (m_bDepressed)
		return m_fViewableContentSize / m_fContentSize;
	else
		return BaseClass::GetNudgeAmount();
}

void HorizontalScrollBar::OnMouseClickLeft(int x, int y, bool bDown)
{
	if (bDown)
	{
		m_bDepressed = true;
		Gwen::MouseFocus = this;
	}
	else
	{
		Gwen::Point clickPos = CanvasPosToLocal(Gwen::Point(x, y));
		if (clickPos.x < m_Bar->X())
			NudgeLeft(this);
		else if (clickPos.x > m_Bar->X() + m_Bar->Width())
			NudgeRight(this);

		m_bDepressed = false;
		Gwen::MouseFocus = NULL;
	}
}

float HorizontalScrollBar::CalculateScrolledAmount()
{
	return (float)(m_Bar->X() - GetButtonSize()) / (float)(Width() - m_Bar->Width() - (GetButtonSize() * 2));
}

bool HorizontalScrollBar::SetScrolledAmount(float amount, bool forceUpdate)
{
	amount = Gwen::Clamp(amount, 0, 1);

	if (!BaseClass::SetScrolledAmount(amount, forceUpdate))
		return false;

	if (forceUpdate)
	{
		int newX = GetButtonSize() + (amount * ((Width() - m_Bar->Width()) - (GetButtonSize() * 2)));
		m_Bar->MoveTo(newX, m_Bar->Y());
	}

	return true;
}

void HorizontalScrollBar::OnBarMoved(Controls::Base* control)
{
	if (m_Bar->IsDepressed())
	{
		SetScrolledAmount(CalculateScrolledAmount(), false);
		BaseClass::OnBarMoved(control);
	}
	else
		InvalidateParent();
}