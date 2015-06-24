/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Controls/Resizer.h"

using namespace Gwen;
using namespace Gwen::ControlsInternal;


GWEN_CONTROL_CONSTRUCTOR( Resizer )
{
	m_iResizeDir = Pos::Left;
	SetMouseInputEnabled( true );
	SetSize( 6, 6 );
}

void Resizer::OnMouseMoved( int x, int y, int /*deltaX*/, int /*deltaY*/ )
{
	if ( !m_pTarget ) return;
	if ( !m_bDepressed ) return;

//	Gwen::Rect oldBounds = m_pTarget->GetBounds();
	Gwen::Rect pBounds = m_pTarget->GetBounds();

	Gwen::Point pntMin = m_pTarget->GetMinimumSize();

	Gwen::Point pCursorPos = m_pTarget->CanvasPosToLocal( Gwen::Point( x, y ) );

	Gwen::Point pDelta = m_pTarget->LocalPosToCanvas( m_HoldPos );
		pDelta.x -= x;
		pDelta.y -= y;

	if ( m_iResizeDir & Pos::Left )
	{
		pBounds.x -= pDelta.x;
		pBounds.w += pDelta.x;

		// Conform to minimum size here so we don't
		// go all weird when we snap it in the base conrt

		if ( pBounds.w < pntMin.x )
		{
			int diff = pntMin.x - pBounds.w;
			pBounds.w += diff;
			pBounds.x -= diff;
		}

	}

	if ( m_iResizeDir & Pos::Top )
	{
		pBounds.y -= pDelta.y;
		pBounds.h += pDelta.y;

		// Conform to minimum size here so we don't
		// go all weird when we snap it in the base conrt

		if ( pBounds.h < pntMin.y )
		{
			int diff = pntMin.y - pBounds.h;
			pBounds.h += diff;
			pBounds.y -= diff;
		}

	}

	if ( m_iResizeDir & Pos::Right )
	{
		// This is complicated.
		// Basically we want to use the HoldPos, so it doesn't snap to the edge of the control
		// But we need to move the HoldPos with the window movement. Yikes.
		// I actually think this might be a big hack around the way this control works with regards
		// to the holdpos being on the parent panel.

		int woff = pBounds.w - m_HoldPos.x;
		int diff = pBounds.w;
		pBounds.w = pCursorPos.x + woff;		
		if ( pBounds.w < pntMin.x ) pBounds.w = pntMin.x;
		diff -= pBounds.w;

		m_HoldPos.x -= diff;
	}

	if ( m_iResizeDir & Pos::Bottom )
	{
		int hoff = pBounds.h - m_HoldPos.y;
		int diff = pBounds.h;
		pBounds.h = pCursorPos.y + hoff;		
		if ( pBounds.h < pntMin.y ) pBounds.h = pntMin.y;
		diff -= pBounds.h;

		m_HoldPos.y -= diff;
	}

	m_pTarget->SetBounds( pBounds );

	onResize.Call( this );
}

void Resizer::SetResizeDir( int dir )
{
	m_iResizeDir = dir;

	if ( (dir & Pos::Left && dir & Pos::Top) || (dir & Pos::Right && dir & Pos::Bottom) ) 
		return SetCursor( Gwen::CursorType::SizeNWSE );

	if ( (dir & Pos::Right && dir & Pos::Top) || (dir & Pos::Left && dir & Pos::Bottom) ) 
		return SetCursor( Gwen::CursorType::SizeNESW );

	if ( dir & Pos::Right || dir & Pos::Left ) 
		return SetCursor( Gwen::CursorType::SizeWE );

	if ( dir & Pos::Top || dir & Pos::Bottom ) 
		return SetCursor( Gwen::CursorType::SizeNS );
	
}