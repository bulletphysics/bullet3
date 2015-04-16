/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/LabelClickable.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR( LabelClickable )
{
	SetIsToggle( false );

	SetAlignment( Gwen::Pos::Left | Gwen::Pos::CenterV );
}

void LabelClickable::Render( Skin::Base* /*skin*/ )
{
	//skin->DrawButton( this, IsDepressed(), IsToggle() && GetToggleState() );
}