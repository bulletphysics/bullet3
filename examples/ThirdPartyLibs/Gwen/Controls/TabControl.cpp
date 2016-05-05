/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/TabControl.h"
#include "Gwen/Controls/Highlight.h"
#include "Gwen/DragAndDrop.h"
#include "Gwen/Controls/WindowControl.h"
#include "Gwen/Controls/ScrollBarButton.h"


namespace Gwen
{
	namespace Controls
	{
		class TabControlInner : public Base
		{
			public:

				GWEN_CONTROL_INLINE( TabControlInner, Base )
				{
					m_ButtonRect = Gwen::Rect( 0, 0, 0, 0 );
				}

				void Render( Skin::Base* skin )
				{
					skin->DrawTabControl( this, m_ButtonRect );
				}

				void UpdateCurrentButton( Gwen::Rect rct )
				{
					m_ButtonRect = rct;
				}

				Gwen::Rect m_ButtonRect;
		};
	};
};

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR( TabControl )
{
	m_iScrollOffset = 0;

	m_pCurrentButton = NULL;

	m_TabStrip = new TabStrip( this );
	m_TabStrip->Dock( Pos::Top );
	m_TabStrip->SetWidth( 100 );
	m_TabStrip->SetHeight( 20 );

	// Make this some special control?
	m_pScroll[0] = new ControlsInternal::ScrollBarButton( this );
	m_pScroll[0]->SetDirectionLeft();
	m_pScroll[0]->onPress.Add( this, &TabControl::ScrollPressLeft );
	m_pScroll[0]->SetSize( 14, 16 );

	m_pScroll[1] = new ControlsInternal::ScrollBarButton( this );
	m_pScroll[1]->SetDirectionRight();
	m_pScroll[1]->onPress.Add( this, &TabControl::ScrollPressRight );
	m_pScroll[1]->SetSize( 14, 16 );

	m_InnerPanel = new TabControlInner( this );
	m_InnerPanel->Dock( Pos::Fill );	

	SetTabable( false );
}



TabButton* TabControl::AddPage( const UnicodeString& strText, Controls::Base* pPage )
{
	if ( !pPage )
	{
		pPage = new Base( this );
	}
	else
	{
		pPage->SetParent( this );
	}

	TabButton* pButton = new TabButton( m_TabStrip );
		pButton->SetText( strText );
		pButton->SetPage( pPage );
		pButton->SetTabable( false );

	AddPage( pButton );
	return pButton;
}

void TabControl::AddPage( TabButton* pButton )
{
	Base* pPage = pButton->GetPage();
		pPage->SetParent( this );
		pPage->SetHidden( true );
		pPage->SetMargin( Margin( 6, 6, 6, 6 ) );
		pPage->Dock( Pos::Fill );

	pButton->SetParent( m_TabStrip );
	pButton->Dock( Pos::Left );
	pButton->SizeToContents();
	if ( pButton->GetTabControl() ) pButton->onPress.RemoveHandler( pButton->GetTabControl() );
	pButton->SetTabControl( this );
	pButton->onPress.Add( this, &TabControl::OnTabPressed );



	if ( !m_pCurrentButton )
	{
		pButton->OnPress();
	}

	onAddTab.Call( this );

	Invalidate();
}

void TabControl::OnTabPressed( Controls::Base* control )
{
	if (!control)
		return;

	TabButton* pButton = control->DynamicCastTabButton();
	if ( !pButton ) return;

	Base* pPage = pButton->GetPage();
	if ( !pPage ) return;

	if ( m_pCurrentButton == pButton)
		return;

	if ( m_pCurrentButton )
	{
		Base* pPage = m_pCurrentButton->GetPage();
		if ( pPage )
		{
			pPage->SetHidden( true );
		}
		m_pCurrentButton = NULL;
	}

	m_pCurrentButton = pButton;

	pPage->SetHidden( false );
	
	m_TabStrip->Invalidate();
	Invalidate();	
}

void TabControl::PostLayout( Skin::Base* skin )
{
	BaseClass::PostLayout( skin );

	HandleOverflow();

	if ( m_TabStrip->Hidden() )
	{
		m_InnerPanel->DynamicCastTabControlInner()->UpdateCurrentButton( Gwen::Rect( 0, 0, 0, 0 ) );
	}
	else if ( m_pCurrentButton )
	{
		Gwen::Rect rct;

		Gwen::Point p = m_pCurrentButton->LocalPosToCanvas( Gwen::Point( 0, 0 ) );
		p = m_InnerPanel->CanvasPosToLocal( p );

		rct = Gwen::Rect( p.x+1, p.y+1, m_pCurrentButton->Width()-2, m_pCurrentButton->Height()-2 );
		m_InnerPanel->DynamicCastTabControlInner()->UpdateCurrentButton( rct );
	}
	
}

void TabControl::OnLoseTab( TabButton* pButton )
{
	if ( m_pCurrentButton == pButton )
		m_pCurrentButton = NULL;

	//TODO: Select a tab if any exist.

	onLoseTab.Call( this );

	Invalidate();	
}

int TabControl::TabCount( void )
{
	return m_TabStrip->NumChildren();
}

void TabControl::SetTabStripPosition( int iDock )
{
	m_TabStrip->SetTabPosition( iDock );
}

bool TabControl::DoesAllowDrag()
{
	return m_TabStrip->AllowsTabReorder();
}

void TabControl::HandleOverflow()
{
	Gwen::Point TabsSize = m_TabStrip->ChildrenSize();

	// Only enable the scrollers if the tabs are at the top.
	// This is a limitation we should explore.
	// Really TabControl should have derivitives for tabs placed elsewhere where we could specialize 
	// some functions like this for each direction.
	bool bNeeded = TabsSize.x > Width() && m_TabStrip->GetDock() == Pos::Top;

	m_pScroll[0]->SetHidden( !bNeeded );
	m_pScroll[1]->SetHidden( !bNeeded );

	if ( !bNeeded ) return;

	m_iScrollOffset = Gwen::Clamp( m_iScrollOffset, 0, TabsSize.x - Width() + 32 );

	#if 0
		//
		// This isn't frame rate independent. 
		// Could be better. Get rid of m_iScrollOffset and just use m_TabStrip->GetMargin().left ?
		// Then get a margin animation type and do it properly! 
		// TODO!
		//
		m_TabStrip->SetMargin( Margin( Gwen::Approach( m_TabStrip->GetMargin().left, m_iScrollOffset * -1, 2 ), 0, 0, 0 ) );
		InvalidateParent();
	#else
		m_TabStrip->SetMargin( Margin( m_iScrollOffset * -1, 0, 0, 0 ) );
	#endif 

	m_pScroll[0]->SetPos( Width() - 30 , 5 );
	m_pScroll[1]->SetPos( m_pScroll[0]->Right(), 5 );
}

void TabControl::ScrollPressLeft( Base* pFrom )
{
	m_iScrollOffset -= 120;
}

void TabControl::ScrollPressRight( Base* pFrom )
{
	m_iScrollOffset += 120;
}