/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Controls/ScrollBar.h"
#include "Gwen/Controls/ScrollBarBar.h"

using namespace Gwen;
using namespace Gwen::Controls;
using namespace Gwen::ControlsInternal;

//Actual bar representing height of parent

GWEN_CONTROL_CONSTRUCTOR( ScrollBarBar )
{
	RestrictToParent( true );
	SetTarget( this );
}

void ScrollBarBar::Render( Skin::Base* skin )
{
	skin->DrawScrollBarBar(this, m_bDepressed, IsHovered(), m_bHorizontal );
	BaseClass::Render( skin );
}

void ScrollBarBar::OnMouseMoved( int x, int y, int deltaX, int deltaY )
{
	BaseClass::OnMouseMoved( x, y, deltaX, deltaY );

	if ( !m_bDepressed )
		return;

	InvalidateParent();
}

void ScrollBarBar::OnMouseClickLeft( int x, int y, bool bDown )
{
	BaseClass::OnMouseClickLeft( x, y, bDown );
	InvalidateParent();
}

void ScrollBarBar::Layout( Skin::Base* /*skin*/ )
{
	if ( !GetParent() )
		return;

	//Move to our current position to force clamping - is this a hack?
	MoveTo( X(), Y() );
}

void ScrollBarBar::MoveTo( int x, int y )
{
	BaseClass::MoveTo( x, y );
}