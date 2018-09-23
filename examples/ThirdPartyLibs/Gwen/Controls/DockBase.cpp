/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/DockBase.h"
#include "Gwen/Controls/DockedTabControl.h"
#include "Gwen/Controls/Highlight.h"
#include "Gwen/DragAndDrop.h"
#include "Gwen/Controls/Resizer.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR(DockBase)
{
	SetPadding(Padding(1, 1, 1, 1));
	SetSize(200, 200);

	m_DockedTabControl = NULL;
	m_Left = NULL;
	m_Right = NULL;
	m_Top = NULL;
	m_Bottom = NULL;

	m_bDrawHover = false;
}

TabControl* DockBase::GetTabControl()
{
	return m_DockedTabControl;
}

void DockBase::SetupChildDock(int iPos)
{
	if (!m_DockedTabControl)
	{
		m_DockedTabControl = new DockedTabControl(this);
		m_DockedTabControl->onLoseTab.Add(this, &DockBase::OnTabRemoved);
		m_DockedTabControl->SetTabStripPosition(Pos::Bottom);
		m_DockedTabControl->SetShowTitlebar(true);
	}

	Dock(iPos);

	int iSizeDirection = Pos::Left;
	if (iPos == Pos::Left) iSizeDirection = Pos::Right;
	if (iPos == Pos::Top) iSizeDirection = Pos::Bottom;
	if (iPos == Pos::Bottom) iSizeDirection = Pos::Top;

	ControlsInternal::Resizer* sizer = new ControlsInternal::Resizer(this);
	sizer->Dock(iSizeDirection);
	sizer->SetResizeDir(iSizeDirection);
	sizer->SetSize(2, 2);
	sizer->SetTarget(this);
}

void DockBase::Render(Skin::Base* /*skin*/)
{
	//Gwen::Render->SetDrawColor( Colors::Black );
	//Gwen::Render->DrawLinedRect( GetRenderBounds() );
}

DockBase** DockBase::GetChildDockPtr(int iPos)
{
	if (iPos == Pos::Left) return &m_Left;
	if (iPos == Pos::Right) return &m_Right;
	if (iPos == Pos::Top) return &m_Top;
	if (iPos == Pos::Bottom) return &m_Bottom;

	return NULL;
}

DockBase* DockBase::GetChildDock(int iPos)
{
	DockBase** pDock = GetChildDockPtr(iPos);

	if (!(*pDock))
	{
		(*pDock) = new DockBase(this);
		(*pDock)->SetupChildDock(iPos);
	}
	else
	{
		(*pDock)->SetHidden(false);
	}

	return *pDock;
}

int DockBase::GetDroppedTabDirection(int x, int y)
{
	int w = Width();
	int h = Height();

	float top = (float)y / (float)h;
	float left = (float)x / (float)w;
	float right = (float)(w - x) / (float)w;
	float bottom = (float)(h - y) / (float)h;

	float minimum = GwenUtil_Min(GwenUtil_Min(GwenUtil_Min(top, left), right), bottom);
	m_bDropFar = (minimum < 0.2f);
	if (minimum > 0.3) return Pos::Fill;

	if (top == minimum && (!m_Top || m_Top->Hidden())) return Pos::Top;
	if (left == minimum && (!m_Left || m_Left->Hidden())) return Pos::Left;
	if (right == minimum && (!m_Right || m_Right->Hidden())) return Pos::Right;
	if (bottom == minimum && (!m_Bottom || m_Bottom->Hidden())) return Pos::Bottom;

	return Pos::Fill;
}

bool DockBase::DragAndDrop_CanAcceptPackage(Gwen::DragAndDrop::Package* pPackage)
{
	// A TAB button dropped
	if (pPackage->name == "TabButtonMove")
		return true;

	// a TAB window dropped
	if (pPackage->name == "TabWindowMove")
		return true;

	return false;
}

void AddTabToDock(TabButton* pTabButton, DockedTabControl* pControl)
{
	pControl->AddPage(pTabButton);
}

bool DockBase::DragAndDrop_HandleDrop(Gwen::DragAndDrop::Package* pPackage, int x, int y)
{
	Gwen::Point pPos = CanvasPosToLocal(Gwen::Point(x, y));
	int dir = GetDroppedTabDirection(pPos.x, pPos.y);

	DockedTabControl* pAddTo = m_DockedTabControl;
	if (dir == Pos::Fill && pAddTo == NULL) return false;

	if (dir != Pos::Fill)
	{
		DockBase* pDock = GetChildDock(dir);
		pAddTo = pDock->m_DockedTabControl;

		if (!m_bDropFar)
			pDock->BringToFront();
		else
			pDock->SendToBack();
	}

	if (pPackage->name == "TabButtonMove")
	{
		TabButton* pTabButton = DragAndDrop::SourceControl->DynamicCastTabButton();
		if (!pTabButton) return false;

		AddTabToDock(pTabButton, pAddTo);
	}

	if (pPackage->name == "TabWindowMove")
	{
		DockedTabControl* pTabControl = DragAndDrop::SourceControl->DynamicCastDockedTabControl();
		if (!pTabControl) return false;
		if (pTabControl == pAddTo) return false;

		pTabControl->MoveTabsTo(pAddTo);
	}

	Invalidate();

	return true;
}

bool DockBase::IsEmpty()
{
	if (m_DockedTabControl && m_DockedTabControl->TabCount() > 0) return false;

	if (m_Left && !m_Left->IsEmpty()) return false;
	if (m_Right && !m_Right->IsEmpty()) return false;
	if (m_Top && !m_Top->IsEmpty()) return false;
	if (m_Bottom && !m_Bottom->IsEmpty()) return false;

	return true;
}

void DockBase::OnTabRemoved(Gwen::Controls::Base* /*pControl*/)
{
	DoRedundancyCheck();
	DoConsolidateCheck();
}

void DockBase::DoRedundancyCheck()
{
	if (!IsEmpty()) return;

	DockBase* pDockParent = GetParent()->DynamicCastDockBase();
	if (!pDockParent) return;

	pDockParent->OnRedundantChildDock(this);
}

void DockBase::DoConsolidateCheck()
{
	if (IsEmpty()) return;
	if (!m_DockedTabControl) return;
	if (m_DockedTabControl->TabCount() > 0) return;

	if (m_Bottom && !m_Bottom->IsEmpty())
	{
		m_Bottom->m_DockedTabControl->MoveTabsTo(m_DockedTabControl);
		return;
	}

	if (m_Top && !m_Top->IsEmpty())
	{
		m_Top->m_DockedTabControl->MoveTabsTo(m_DockedTabControl);
		return;
	}

	if (m_Left && !m_Left->IsEmpty())
	{
		m_Left->m_DockedTabControl->MoveTabsTo(m_DockedTabControl);
		return;
	}

	if (m_Right && !m_Right->IsEmpty())
	{
		m_Right->m_DockedTabControl->MoveTabsTo(m_DockedTabControl);
		return;
	}
}

void DockBase::OnRedundantChildDock(DockBase* pDockBase)
{
	pDockBase->SetHidden(true);
	DoRedundancyCheck();
	DoConsolidateCheck();
}

void DockBase::DragAndDrop_HoverEnter(Gwen::DragAndDrop::Package* /*pPackage*/, int /*x*/, int /*y*/)
{
	m_bDrawHover = true;
}

void DockBase::DragAndDrop_HoverLeave(Gwen::DragAndDrop::Package* /*pPackage*/)
{
	m_bDrawHover = false;
}

void DockBase::DragAndDrop_Hover(Gwen::DragAndDrop::Package* /*pPackage*/, int x, int y)
{
	Gwen::Point pPos = CanvasPosToLocal(Gwen::Point(x, y));
	int dir = GetDroppedTabDirection(pPos.x, pPos.y);

	if (dir == Pos::Fill)
	{
		if (!m_DockedTabControl)
		{
			m_HoverRect = Gwen::Rect(0, 0, 0, 0);
			return;
		}

		m_HoverRect = GetInnerBounds();
		return;
	}

	m_HoverRect = GetRenderBounds();

	int HelpBarWidth = 0;

	if (dir == Pos::Left)
	{
		HelpBarWidth = m_HoverRect.w * 0.25f;
		m_HoverRect.w = HelpBarWidth;
	}

	if (dir == Pos::Right)
	{
		HelpBarWidth = m_HoverRect.w * 0.25f;
		m_HoverRect.x = m_HoverRect.w - HelpBarWidth;
		m_HoverRect.w = HelpBarWidth;
	}

	if (dir == Pos::Top)
	{
		HelpBarWidth = m_HoverRect.h * 0.25f;
		m_HoverRect.h = HelpBarWidth;
	}

	if (dir == Pos::Bottom)
	{
		HelpBarWidth = m_HoverRect.h * 0.25f;
		m_HoverRect.y = m_HoverRect.h - HelpBarWidth;
		m_HoverRect.h = HelpBarWidth;
	}

	if ((dir == Pos::Top || dir == Pos::Bottom) && !m_bDropFar)
	{
		if (m_Left && m_Left->Visible())
		{
			m_HoverRect.x += m_Left->Width();
			m_HoverRect.w -= m_Left->Width();
		}

		if (m_Right && m_Right->Visible())
		{
			m_HoverRect.w -= m_Right->Width();
		}
	}

	if ((dir == Pos::Left || dir == Pos::Right) && !m_bDropFar)
	{
		if (m_Top && m_Top->Visible())
		{
			m_HoverRect.y += m_Top->Height();
			m_HoverRect.h -= m_Top->Height();
		}

		if (m_Bottom && m_Bottom->Visible())
		{
			m_HoverRect.h -= m_Bottom->Height();
		}
	}
}

void DockBase::RenderOver(Skin::Base* skin)
{
	if (!m_bDrawHover) return;

	Gwen::Renderer::Base* render = skin->GetRender();

	render->SetDrawColor(Gwen::Color(255, 100, 255, 20));
	render->DrawFilledRect(GetRenderBounds());

	if (m_HoverRect.w == 0) return;

	render->SetDrawColor(Gwen::Color(255, 100, 255, 100));
	render->DrawFilledRect(m_HoverRect);

	render->SetDrawColor(Gwen::Color(255, 100, 255, 200));
	render->DrawLinedRect(m_HoverRect);
}