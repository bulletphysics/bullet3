/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/DockedTabControl.h"
#include "Gwen/Controls/Highlight.h"
#include "Gwen/DragAndDrop.h"
#include "Gwen/Controls/WindowControl.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR(DockedTabControl)
{
	m_WindowControl = NULL;

	Dock(Pos::Fill);

	m_pTitleBar = new TabTitleBar(this);
	m_pTitleBar->Dock(Pos::Top);
	m_pTitleBar->SetHidden(true);
}

void DockedTabControl::Layout(Skin::Base* skin)
{
	GetTabStrip()->SetHidden(TabCount() <= 1);
	UpdateTitleBar();
	BaseClass::Layout(skin);
}

void DockedTabControl::UpdateTitleBar()
{
	if (!GetCurrentButton()) return;

	m_pTitleBar->UpdateFromTab(GetCurrentButton());
}

void DockedTabControl::DragAndDrop_StartDragging(Gwen::DragAndDrop::Package* pPackage, int x, int y)
{
	BaseClass::DragAndDrop_StartDragging(pPackage, x, y);

	SetHidden(true);
	// This hiding our parent thing is kind of lousy.
	GetParent()->SetHidden(true);
}

void DockedTabControl::DragAndDrop_EndDragging(bool bSuccess, int /*x*/, int /*y*/)
{
	SetHidden(false);

	if (!bSuccess)
	{
		GetParent()->SetHidden(false);
	}

	/*
	if ( !bSuccess )
	{
		// Create our window control
		if ( !m_WindowControl )
		{
			m_WindowControl = new WindowControl( GetCanvas() );
			m_WindowControl->SetBounds( x, y, Width(), Height() );
		}

		m_WindowControl->SetPosition( x, y );
		SetParent( m_WindowControl );
		SetPosition( 0, 0 );
		Dock( Pos::Fill );
	}
	*/
}

void DockedTabControl::MoveTabsTo(DockedTabControl* pTarget)
{
	Base::List Children = GetTabStrip()->Children;
	for (Base::List::iterator iter = Children.begin(); iter != Children.end(); ++iter)
	{
		TabButton* pButton = (*iter)->DynamicCastTabButton();
		if (!pButton) continue;

		pTarget->AddPage(pButton);
	}

	Invalidate();
}