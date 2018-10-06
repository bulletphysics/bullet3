/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_TABBUTTON_H
#define GWEN_CONTROLS_TABBUTTON_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Button.h"

namespace Gwen
{
namespace Controls
{
class TabControl;

class GWEN_EXPORT TabButton : public Button
{
public:
	GWEN_CONTROL(TabButton, Button);
	virtual void Render(Skin::Base* skin);

	void SetPage(Base* page) { m_Page = page; }
	Base* GetPage() { return m_Page; }

	void SetTabControl(TabControl* ctrl);
	TabControl* GetTabControl() { return m_Control; }

	bool IsActive() { return m_Page && m_Page->Visible(); }

	virtual bool DragAndDrop_ShouldStartDrag();
	virtual void DragAndDrop_StartDragging(Gwen::DragAndDrop::Package* /*pPackage*/, int /*x*/, int /*y*/) { SetHidden(true); }
	virtual void DragAndDrop_EndDragging(bool /*bSuccess*/, int /*x*/, int /*y*/) { SetHidden(false); }

	virtual bool OnKeyLeft(bool bDown);
	virtual bool OnKeyRight(bool bDown);
	virtual bool OnKeyUp(bool bDown);
	virtual bool OnKeyDown(bool bDown);

private:
	Base* m_Page;
	TabControl* m_Control;
};

}  // namespace Controls
}  // namespace Gwen
#endif
