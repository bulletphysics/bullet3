/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Controls/RadioButton.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR( RadioButton )
{
	SetSize( 11, 11 );
	SetMouseInputEnabled( true );
	SetTabable( false );
}

void RadioButton::Render( Skin::Base* skin )
{
	skin->DrawRadioButton( this, IsChecked(), IsDepressed() );
}

