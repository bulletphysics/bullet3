/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_VERTICALSCROLLBAR_H
#define GWEN_CONTROLS_VERTICALSCROLLBAR_H
#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/ScrollBar.h"
#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"

namespace Gwen
{
namespace Controls
{
class GWEN_EXPORT VerticalScrollBar : public BaseScrollBar
{
	GWEN_CONTROL(VerticalScrollBar, BaseScrollBar);

	virtual void Layout(Skin::Base* skin);

	virtual void OnMouseClickLeft(int x, int y, bool bDown);
	virtual void OnBarMoved(Controls::Base* control);

	virtual int GetBarSize() { return m_Bar->Height(); }
	virtual int GetBarPos() { return m_Bar->Y() - Width(); }
	virtual void SetBarSize(int size) { m_Bar->SetHeight(size); }
	virtual int GetButtonSize() { return Width(); }

	virtual void ScrollToTop();
	virtual void ScrollToBottom();
	virtual void NudgeUp(Base* control);
	virtual void NudgeDown(Base* control);
	virtual float GetNudgeAmount();

	virtual float CalculateScrolledAmount();
	virtual bool SetScrolledAmount(float amount, bool forceUpdate);
};
}  // namespace Controls
}  // namespace Gwen
#endif
