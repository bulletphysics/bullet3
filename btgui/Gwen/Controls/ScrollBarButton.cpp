/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Controls/ScrollBar.h"
#include "Gwen/Controls/ScrollBarButton.h"

using namespace Gwen;
using namespace Gwen::Controls;
using namespace Gwen::ControlsInternal;


GWEN_CONTROL_CONSTRUCTOR( ScrollBarButton )
{
	m_iDirection = 0;
	SetBounds(0,0,0,0);
}

void ScrollBarButton::SetDirectionUp()
{
	m_iDirection = Pos::Top;
}

void ScrollBarButton::SetDirectionDown()
{
	m_iDirection = Pos::Bottom;
}

void ScrollBarButton::SetDirectionLeft()
{
	m_iDirection = Pos::Left;
}

void ScrollBarButton::SetDirectionRight()
{
	m_iDirection = Pos::Right;
}

void ScrollBarButton::Render( Skin::Base* skin )
{
	skin->DrawScrollButton( this, m_iDirection, m_bDepressed );
}