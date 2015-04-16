
#include "Gwen/Gwen.h"
#include "Gwen/Controls/SplitterBar.h"

using namespace Gwen;
using namespace Controls;

GWEN_CONTROL_CONSTRUCTOR( SplitterBar )
{
	SetTarget( this );
	RestrictToParent( true );
}

void SplitterBar::Render( Skin::Base* skin )
{
	if ( ShouldDrawBackground() )
		skin->DrawButton( this, true, false );
}

void SplitterBar::Layout( Skin::Base* /*skin*/ )
{
	MoveTo( X(), Y() );
}