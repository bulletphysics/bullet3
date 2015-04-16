/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Controls/ScrollControl.h"
#include "Gwen/Controls/ProgressBar.h"
#include "Gwen/Utility.h"

using namespace Gwen;
using namespace Gwen::Controls;


GWEN_CONTROL_CONSTRUCTOR( ProgressBar )
{
	SetMouseInputEnabled( true );
	SetBounds( Gwen::Rect( 0, 0, 128, 32 ) );
	SetTextPadding( Padding( 3, 3, 3, 3 ) );
	SetHorizontal();

	SetAlignment( Gwen::Pos::Center );

	m_fProgress = 0.0f;
	m_bAutoLabel = true;
}

void ProgressBar::SetValue(float val)
{
	if ( val < 0 )
		val = 0;

	if ( val > 1 )
		val = 1;

	m_fProgress = val;
	
	if ( m_bAutoLabel )
	{
		int displayVal = m_fProgress * 100;
		SetText( Utility::ToString( displayVal ) + "%" );
	}
}

void ProgressBar::Render( Skin::Base* skin )
{
	skin->DrawProgressBar( this, m_bHorizontal, m_fProgress);
}