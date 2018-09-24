/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_TABCONTROL_H
#define GWEN_CONTROLS_TABCONTROL_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Button.h"
#include "Gwen/Controls/TabButton.h"
#include "Gwen/Controls/TabStrip.h"
#include "Gwen/Controls/TabTitleBar.h"

namespace Gwen
{
namespace ControlsInternal
{
class ScrollBarButton;
}

namespace Controls
{
class GWEN_EXPORT TabControl : public Base
{
	GWEN_CONTROL(TabControl, Base);

	virtual TabButton* AddPage(const UnicodeString& strText, Controls::Base* pPage = NULL);
	virtual void AddPage(TabButton* pButton);

	virtual void OnTabPressed(Controls::Base* control);
	virtual void OnLoseTab(TabButton* pButton);

	virtual int TabCount(void);
	virtual TabButton* GetCurrentButton() { return m_pCurrentButton; }
	virtual TabStrip* GetTabStrip() { return m_TabStrip; }

	virtual void SetTabStripPosition(int iDock);

	virtual bool DoesAllowDrag();

	virtual void SetAllowReorder(bool b) { GetTabStrip()->SetAllowReorder(b); }

	Gwen::Event::Caller onLoseTab;
	Gwen::Event::Caller onAddTab;

private:
	virtual void PostLayout(Skin::Base* skin);
	void HandleOverflow();

	void ScrollPressLeft(Base* pFrom);
	void ScrollPressRight(Base* pFrom);

	TabStrip* m_TabStrip;
	TabButton* m_pCurrentButton;

	ControlsInternal::ScrollBarButton* m_pScroll[2];
	int m_iScrollOffset;
};
}  // namespace Controls
}  // namespace Gwen
#endif
