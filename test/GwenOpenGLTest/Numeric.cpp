#include "UnitTest.h"
#include "Gwen/Controls/NumericUpDown.h"

using namespace Gwen;

class Numeric : public GUnit
{
public:
	GWEN_CONTROL_INLINE(Numeric, GUnit)
	{
		Controls::NumericUpDown* pCtrl = new Controls::NumericUpDown(this);
		pCtrl->SetBounds(10, 10, 50, 20);
		pCtrl->SetValue(50);
		pCtrl->SetMax(1000);
		pCtrl->SetMin(-1000);

		//	pCtrl->onPress.Add( this, &ThisClass::onButtonA );
	}

	void onButtonA(Controls::Base* pControl)
	{
		//	UnitPrint( L"Button Pressed (using 'OnPress' event)" );
	}
};

DEFINE_UNIT_TEST(Numeric, L"Numeric");