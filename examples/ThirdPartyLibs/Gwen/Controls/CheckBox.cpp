/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Controls/CheckBox.h"

using namespace Gwen;
using namespace Gwen::Controls;



GWEN_CONTROL_CONSTRUCTOR( CheckBox )
{
	SetSize( 13, 13 );

	m_bChecked = true;
	Toggle();
}


void CheckBox::Render( Skin::Base* skin )
{
	skin->DrawCheckBox( this, m_bChecked, IsDepressed() );
}

void CheckBox::OnPress()
{
	if ( IsChecked() && !AllowUncheck() )
		return;

	Toggle();
}

void CheckBox::OnCheckStatusChanged()
{
	if ( IsChecked() )
	{
		onChecked.Call( this );
	}
	else
	{
		onUnChecked.Call( this );
	}

	onCheckChanged.Call( this );
}

void CheckBox::SetChecked( bool bChecked ) 
{ 
	if ( m_bChecked == bChecked ) return;

	m_bChecked = bChecked; 
	OnCheckStatusChanged(); 
}