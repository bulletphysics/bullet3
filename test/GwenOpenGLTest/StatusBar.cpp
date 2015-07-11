#include "UnitTest.h"
#include "Gwen/Controls/StatusBar.h"
#include "Gwen/Controls/Label.h"

using namespace Gwen;

class StatusBar : public GUnit
{
	public:

	GWEN_CONTROL_INLINE( StatusBar, GUnit )
	{
		Gwen::Controls::StatusBar* pStatus = new Gwen::Controls::StatusBar( this );
		pStatus->Dock( Pos::Bottom );

		Gwen::Controls::Label* pLeft = new Gwen::Controls::Label( pStatus );
		pLeft->SetText(L"Label Added to left");
		pStatus->AddControl( pLeft, false );

		Gwen::Controls::Label* pRight = new Gwen::Controls::Label( pStatus );
		pRight->SetText(L"Label Added to Right");
		pStatus->AddControl( pRight, true );
	}
};



DEFINE_UNIT_TEST( StatusBar, L"StatusBar" );