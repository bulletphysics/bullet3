/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/TabStrip.h"
#include "Gwen/Controls/TabControl.h"
#include "Gwen/Controls/Highlight.h"
#include "Gwen/DragAndDrop.h"
#include "Gwen/Utility.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR(TabStrip)
{
	m_TabDragControl = NULL;
	m_bAllowReorder = false;
}

bool TabStrip::DragAndDrop_HandleDrop(Gwen::DragAndDrop::Package* /*pPackage*/, int x, int y)
{
	Gwen::Point LocalPos = CanvasPosToLocal(Gwen::Point(x, y));

	Base* el = DragAndDrop::SourceControl;

	TabButton* pButton = el ? el->DynamicCastTabButton() : 0;
	TabControl* pTabControl = GetParent() ? GetParent()->DynamicCastTabControl() : 0;
	if (pTabControl && pButton)
	{
		if (pButton->GetTabControl() != pTabControl)
		{
			// We've moved tab controls!
			pTabControl->AddPage(pButton);
		}
	}

	Base* DroppedOn = GetControlAt(LocalPos.x, LocalPos.y);
	if (DroppedOn)
	{
		Gwen::Point DropPos = DroppedOn->CanvasPosToLocal(Gwen::Point(x, y));
		DragAndDrop::SourceControl->BringNextToControl(DroppedOn, DropPos.x > DroppedOn->Width() / 2);
	}
	else
	{
		DragAndDrop::SourceControl->BringToFront();
	}
	return true;
}

bool TabStrip::DragAndDrop_CanAcceptPackage(Gwen::DragAndDrop::Package* pPackage)
{
	if (!m_bAllowReorder)
		return false;

	if (pPackage->name == "TabButtonMove")
		return true;

	return false;
}

void TabStrip::Layout(Skin::Base* skin)
{
	Gwen::Point pLargestTab(5, 5);

	int iNum = 0;
	for (Base::List::iterator iter = Children.begin(); iter != Children.end(); ++iter)
	{
		if (!*iter)
			continue;

		TabButton* pButton = (*iter)->DynamicCastTabButton();
		if (!pButton) continue;

		pButton->SizeToContents();

		Margin m;
		int iActive = pButton->IsActive() ? 0 : 2;
		int iNotFirst = iNum > 0 ? -1 : 0;
		int iControlOverhang = -3;

		if (m_iDock == Pos::Top)
		{
			m.top = iActive;
			m.left = iNotFirst;
			m.bottom = iControlOverhang;
			pButton->Dock(Pos::Left);
		}

		if (m_iDock == Pos::Left)
		{
			m.left = iActive * 2;
			m.right = iControlOverhang;
			m.top = iNotFirst;
			pButton->Dock(Pos::Top);
		}

		if (m_iDock == Pos::Right)
		{
			m.right = iActive * 2;
			m.left = iControlOverhang;
			m.top = iNotFirst;
			pButton->Dock(Pos::Top);
		}

		if (m_iDock == Pos::Bottom)
		{
			m.bottom = iActive;
			m.left = iNotFirst;
			m.top = iControlOverhang;
			pButton->Dock(Pos::Left);
		}

		pLargestTab.x = Utility::Max(pLargestTab.x, pButton->Width());
		pLargestTab.y = Utility::Max(pLargestTab.y, pButton->Height());

		pButton->SetMargin(m);
		iNum++;
	}

	if (m_iDock == Pos::Top || m_iDock == Pos::Bottom)
		SetSize(Width(), pLargestTab.y);

	if (m_iDock == Pos::Left || m_iDock == Pos::Right)
		SetSize(pLargestTab.x, Height());

	BaseClass::Layout(skin);
}

void TabStrip::DragAndDrop_HoverEnter(Gwen::DragAndDrop::Package* /*pPackage*/, int /*x*/, int /*y*/)
{
	if (m_TabDragControl)
	{
		Debug::Msg("ERROR! TabStrip::DragAndDrop_HoverEnter\n");
	}

	m_TabDragControl = new ControlsInternal::Highlight(this);
	m_TabDragControl->SetMouseInputEnabled(false);
	m_TabDragControl->SetSize(3, Height());
}

void TabStrip::DragAndDrop_HoverLeave(Gwen::DragAndDrop::Package* /*pPackage*/)
{
	delete m_TabDragControl;
	m_TabDragControl = NULL;
}

void TabStrip::DragAndDrop_Hover(Gwen::DragAndDrop::Package* /*pPackage*/, int x, int y)
{
	Gwen::Point LocalPos = CanvasPosToLocal(Gwen::Point(x, y));

	Base* DroppedOn = GetControlAt(LocalPos.x, LocalPos.y);
	if (DroppedOn && DroppedOn != this)
	{
		Gwen::Point DropPos = DroppedOn->CanvasPosToLocal(Gwen::Point(x, y));
		m_TabDragControl->SetBounds(Gwen::Rect(0, 0, 3, Height()));
		m_TabDragControl->BringToFront();
		m_TabDragControl->SetPos(DroppedOn->X() - 1, 0);

		if (DropPos.x > DroppedOn->Width() / 2)
		{
			m_TabDragControl->MoveBy(DroppedOn->Width() - 1, 0);
		}
		m_TabDragControl->Dock(Pos::None);
	}
	else
	{
		m_TabDragControl->Dock(Pos::Left);
		m_TabDragControl->BringToFront();
	}
}

void TabStrip::SetTabPosition(int iPos)
{
	Dock(iPos);
}