/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Controls/ScrollControl.h"
#include "Gwen/Controls/ScrollBar.h"
#include "Gwen/Controls/VerticalScrollBar.h"
#include "Gwen/Controls/HorizontalScrollBar.h"
#include "Gwen/Utility.h"

using namespace Gwen;
using namespace Gwen::Controls;
using namespace Gwen::ControlsInternal;

GWEN_CONTROL_CONSTRUCTOR( ScrollControl )
{
	SetMouseInputEnabled( false );

	m_VerticalScrollBar	= new VerticalScrollBar( this );
	m_VerticalScrollBar->Dock(Pos::Right);
	m_VerticalScrollBar->onBarMoved.Add( this, &ScrollControl::VBarMoved );
	m_VerticalScrollBar->SetNudgeAmount( 30 );
	m_bCanScrollV = true;
	
	m_HorizontalScrollBar = new HorizontalScrollBar( this );
	m_HorizontalScrollBar->Dock( Pos::Bottom );
	m_HorizontalScrollBar->onBarMoved.Add( this, &ScrollControl::HBarMoved );
	m_bCanScrollH = true;
	m_HorizontalScrollBar->SetNudgeAmount( 30 );

	m_InnerPanel = new Base( this );
	m_InnerPanel->SetPos(0, 0);
	m_InnerPanel->SetMargin( Margin(5,5,5,5));
	m_InnerPanel->SendToBack();
	m_InnerPanel->SetMouseInputEnabled( false );

	m_bAutoHideBars = false;
}

void ScrollControl::SetScroll( bool h, bool v )
{
	m_bCanScrollV = v;
	m_bCanScrollH = h;
	m_VerticalScrollBar->SetHidden(	!m_bCanScrollV );
	m_HorizontalScrollBar->SetHidden( !m_bCanScrollH );
}

void ScrollControl::SetInnerSize( int w, int h )
{
	m_InnerPanel->SetSize( w, h );
}
 
void ScrollControl::VBarMoved( Controls::Base * /*control*/ )
{
	Invalidate();
}

void ScrollControl::HBarMoved( Controls::Base * /*control*/ )
{
	Invalidate();
}

void ScrollControl::OnChildBoundsChanged( Gwen::Rect /*oldChildBounds*/, Base* /*pChild*/ )
{
	UpdateScrollBars();
}

void ScrollControl::Layout( Skin::Base* skin )
{
	UpdateScrollBars();
	BaseClass::Layout(skin);
}

bool ScrollControl::OnMouseWheeled( int iDelta )
{
	if ( CanScrollV() && m_VerticalScrollBar->Visible() )
	{
		if ( m_VerticalScrollBar->SetScrolledAmount( m_VerticalScrollBar->GetScrolledAmount() - m_VerticalScrollBar->GetNudgeAmount() * ( (float)iDelta / 60.0f ), true) )
			return true;
	}

	if ( CanScrollH() && m_HorizontalScrollBar->Visible() )
	{
		if ( m_HorizontalScrollBar->SetScrolledAmount( m_HorizontalScrollBar->GetScrolledAmount() - m_HorizontalScrollBar->GetNudgeAmount() * ( (float)iDelta / 60.0f ), true) )
		return true;
	}

	return false;
}
void ScrollControl::Render( Skin::Base* skin )
{

#if 0

	// Debug render - this shouldn't render ANYTHING REALLY - it should be up to the parent!

	Gwen::Rect rect = GetRenderBounds();
	Gwen::Renderer::Base* render = skin->GetRender();

	render->SetDrawColor( Gwen::Color( 255, 255, 0, 100 ) );
	render->DrawFilledRect( rect );

	render->SetDrawColor( Gwen::Color( 255, 0, 0, 100 ) );
	render->DrawFilledRect( m_InnerPanel->GetBounds() );

	render->RenderText( skin->GetDefaultFont(), Gwen::Point( 0, 0 ), Utility::Format( L"Offset: %i %i", m_InnerPanel->X(), m_InnerPanel->Y() ) );

#else //0

	(void)skin;

#endif //0
}

void ScrollControl::UpdateScrollBars()
{ 
	if ( !m_InnerPanel )
		return;

	int childrenWidth = 0;
	int childrenHeight = 0;
	
	//Get the max size of all our children together
	for ( Base::List::iterator iter = m_InnerPanel->Children.begin(); iter != m_InnerPanel->Children.end(); ++iter )
	{
		Base* pChild = *iter;

		childrenWidth = Utility::Max( childrenWidth, pChild->Right() );
		childrenHeight = Utility::Max( childrenHeight, pChild->Bottom() );
	}

	m_InnerPanel->SetSize( Utility::Max(Width(), childrenWidth), Utility::Max(Height(), childrenHeight));
 
	float hg = (float)(childrenWidth + (m_VerticalScrollBar->Hidden() ? 0 : m_VerticalScrollBar->Width()));
	if (hg==0.f)
		hg = 0.00001f;
	float wPercent = (float)Width()  / hg;
	hg = (float)(childrenHeight + (m_HorizontalScrollBar->Hidden() ? 0 : m_HorizontalScrollBar->Height()));
	if (hg==0.f)
		hg = 0.00001f;
	float hPercent = (float)Height() / hg;

	if ( m_bCanScrollV )
		SetVScrollRequired( hPercent >= 1 );
	else
		m_VerticalScrollBar->SetHidden( true );

	if ( m_bCanScrollH )
		SetHScrollRequired( wPercent >= 1 );
	else
		m_HorizontalScrollBar->SetHidden( true );


	m_VerticalScrollBar->SetContentSize( m_InnerPanel->Height() );
	m_VerticalScrollBar->SetViewableContentSize( Height() - (m_HorizontalScrollBar->Hidden() ? 0 : m_HorizontalScrollBar->Height()));


	m_HorizontalScrollBar->SetContentSize( m_InnerPanel->Width() );
	m_HorizontalScrollBar->SetViewableContentSize(  Width() - (m_VerticalScrollBar->Hidden() ? 0 : m_VerticalScrollBar->Width())  );

	int newInnerPanelPosX = 0;
	int newInnerPanelPosY = 0;

	if ( CanScrollV() && !m_VerticalScrollBar->Hidden() )
	{
		newInnerPanelPosY = -( ( m_InnerPanel->Height() ) - Height() + (m_HorizontalScrollBar->Hidden() ? 0 : m_HorizontalScrollBar->Height())   ) * m_VerticalScrollBar->GetScrolledAmount();
	}
	if ( CanScrollH() && !m_HorizontalScrollBar->Hidden() )
	{
		newInnerPanelPosX = - ( ( m_InnerPanel->Width() ) - Width()  + (m_VerticalScrollBar->Hidden() ? 0 : m_VerticalScrollBar->Width()))  * m_HorizontalScrollBar->GetScrolledAmount();
	}

	m_InnerPanel->SetPos( newInnerPanelPosX , newInnerPanelPosY );
}

void ScrollControl::SetVScrollRequired(bool req)
{
	if ( req )
	{
		m_VerticalScrollBar->SetScrolledAmount( 0, true );
		m_VerticalScrollBar->SetDisabled( true );

		if ( m_bAutoHideBars )
			m_VerticalScrollBar->SetHidden( true );
	}
	else
	{
		m_VerticalScrollBar->SetHidden( false );
		m_VerticalScrollBar->SetDisabled( false );
	}
}

void ScrollControl::SetHScrollRequired(bool req)
{
	if ( req )
	{
		m_HorizontalScrollBar->SetScrolledAmount( 0, true );
		m_HorizontalScrollBar->SetDisabled( true );
		if ( m_bAutoHideBars )
			m_HorizontalScrollBar->SetHidden( true );
	}
	else
	{
		m_HorizontalScrollBar->SetHidden( false );
		m_HorizontalScrollBar->SetDisabled( true );
	}
}

void ScrollControl::ScrollToBottom()
{
	if ( CanScrollV() )
	{
		UpdateScrollBars();
		m_VerticalScrollBar->ScrollToBottom();
	}
}
void ScrollControl::ScrollToTop()
{
	if ( CanScrollV() )
	{
		UpdateScrollBars();
		m_VerticalScrollBar->ScrollToTop();
	}
}
void ScrollControl::ScrollToLeft()
{
	if ( CanScrollH() )
	{
		UpdateScrollBars();
		m_HorizontalScrollBar->ScrollToLeft();
	}
}
void ScrollControl::ScrollToRight()
{
	if ( CanScrollH() )
	{	
		UpdateScrollBars();
		m_HorizontalScrollBar->ScrollToRight();
	}
}

void ScrollControl::Clear()
{
	m_InnerPanel->RemoveAllChildren();
}