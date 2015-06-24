#include "UnitTest.h"
#include "Gwen/Controls/RadioButtonController.h"
#include "Gwen/Controls/ProgressBar.h"

using namespace Gwen;

class ProgressBar : public GUnit
{
	public:

	GWEN_CONTROL_INLINE( ProgressBar, GUnit )
	{

		{
			Gwen::Controls::ProgressBar* pb = new Gwen::Controls::ProgressBar( this );
			pb->SetBounds( Gwen::Rect( 110, 20, 200, 20 ) );
			pb->SetValue( 0.27f );
		}

		{
			Gwen::Controls::ProgressBar* pb = new Gwen::Controls::ProgressBar( this );
			pb->SetBounds( Gwen::Rect( 110, 50, 200, 20 ) );
			pb->SetValue( 0.66f );
			pb->SetAlignment( Pos::Right | Pos::CenterV );
		}

		{
			Gwen::Controls::ProgressBar* pb = new Gwen::Controls::ProgressBar( this );
			pb->SetBounds( Gwen::Rect( 110, 80, 200, 20 ) );
			pb->SetValue( 0.88f );
			pb->SetAlignment( Pos::Left | Pos::CenterV );
		}

		{
			Gwen::Controls::ProgressBar* pb = new Gwen::Controls::ProgressBar( this );
			pb->SetBounds( Gwen::Rect( 110, 110, 200, 20 ) );
			pb->SetAutoLabel( false );
			pb->SetValue( 0.20f );
			pb->SetAlignment( Pos::Right | Pos::CenterV );
			pb->SetText( L"40,245 MB" );
		}

		{
			Gwen::Controls::ProgressBar* pb = new Gwen::Controls::ProgressBar( this );
			pb->SetBounds( Gwen::Rect( 110, 140, 200, 20 ) );
			pb->SetAutoLabel( false );
			pb->SetValue( 1.00f );
		}

		{
			Gwen::Controls::ProgressBar* pb = new Gwen::Controls::ProgressBar( this );
			pb->SetBounds( Gwen::Rect( 110, 170, 200, 20 ) );
			pb->SetAutoLabel( false );
			pb->SetValue( 0.00f );
		}

		{
			Gwen::Controls::ProgressBar* pb = new Gwen::Controls::ProgressBar( this );
			pb->SetBounds( Gwen::Rect( 110, 200, 200, 20 ) );
			pb->SetAutoLabel( false );
			pb->SetValue( 0.50f );
		}

		{
			Gwen::Controls::ProgressBar* pb = new Gwen::Controls::ProgressBar( this );
			pb->SetBounds( Gwen::Rect( 20, 20, 25, 200 ) );
			pb->SetVertical();
			pb->SetValue( 0.25f );
			pb->SetAlignment( Pos::Top | Pos::CenterH );
		}

		{
			Gwen::Controls::ProgressBar* pb = new Gwen::Controls::ProgressBar( this );
			pb->SetBounds( Gwen::Rect( 50, 20, 25, 200 ) );
			pb->SetVertical();
			pb->SetValue( 0.40f );
		}

		{
			Gwen::Controls::ProgressBar* pb = new Gwen::Controls::ProgressBar( this );
			pb->SetBounds( Gwen::Rect( 80, 20, 25, 200 ) );
			pb->SetVertical();
			pb->SetAlignment( Pos::Bottom | Pos::CenterH );
			pb->SetValue( 0.65f );
		}

	}

};



DEFINE_UNIT_TEST( ProgressBar, L"ProgressBar" );