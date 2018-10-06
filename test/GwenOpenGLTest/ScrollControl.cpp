#include "UnitTest.h"
#include "Gwen/Controls/ScrollControl.h"

using namespace Gwen;

class ScrollControl : public GUnit
{
public:
	GWEN_CONTROL_INLINE(ScrollControl, GUnit)
	{
		{
			Gwen::Controls::ScrollControl* pCtrl = new Gwen::Controls::ScrollControl(this);
			pCtrl->SetBounds(10, 10, 100, 100);

			Controls::Button* pTestButton = new Controls::Button(pCtrl);
			pTestButton->SetText(L"Twice As Big");
			pTestButton->SetBounds(0, 0, 200, 200);
		}

		{
			Gwen::Controls::ScrollControl* pCtrl = new Gwen::Controls::ScrollControl(this);
			pCtrl->SetBounds(110, 10, 100, 100);

			Controls::Button* pTestButton = new Controls::Button(pCtrl);
			pTestButton->SetText(L"Same Size");
			pTestButton->SetBounds(0, 0, 100, 100);
		}

		{
			Gwen::Controls::ScrollControl* pCtrl = new Gwen::Controls::ScrollControl(this);
			pCtrl->SetBounds(210, 10, 100, 100);

			Controls::Button* pTestButton = new Controls::Button(pCtrl);
			pTestButton->SetText(L"Wide");
			pTestButton->SetBounds(0, 0, 200, 50);
		}

		{
			Gwen::Controls::ScrollControl* pCtrl = new Gwen::Controls::ScrollControl(this);
			pCtrl->SetBounds(310, 10, 100, 100);

			Controls::Button* pTestButton = new Controls::Button(pCtrl);
			pTestButton->SetText(L"Tall");
			pTestButton->SetBounds(0, 0, 50, 200);
		}

		{
			Gwen::Controls::ScrollControl* pCtrl = new Gwen::Controls::ScrollControl(this);
			pCtrl->SetBounds(410, 10, 100, 100);
			pCtrl->SetScroll(false, true);

			Controls::Button* pTestButton = new Controls::Button(pCtrl);
			pTestButton->SetText(L"Vertical");
			pTestButton->SetBounds(0, 0, 200, 200);
		}

		{
			Gwen::Controls::ScrollControl* pCtrl = new Gwen::Controls::ScrollControl(this);
			pCtrl->SetBounds(510, 10, 100, 100);
			pCtrl->SetScroll(true, false);

			Controls::Button* pTestButton = new Controls::Button(pCtrl);
			pTestButton->SetText(L"Horinzontal");
			pTestButton->SetBounds(0, 0, 200, 200);
		}

		// Bottom Row

		{
			Gwen::Controls::ScrollControl* pCtrl = new Gwen::Controls::ScrollControl(this);
			pCtrl->SetBounds(10, 110, 100, 100);
			pCtrl->SetAutoHideBars(true);

			Controls::Button* pTestButton = new Controls::Button(pCtrl);
			pTestButton->SetText(L"Twice As Big");
			pTestButton->SetBounds(0, 0, 200, 200);
		}

		{
			Gwen::Controls::ScrollControl* pCtrl = new Gwen::Controls::ScrollControl(this);
			pCtrl->SetBounds(110, 110, 100, 100);
			pCtrl->SetAutoHideBars(true);

			Controls::Button* pTestButton = new Controls::Button(pCtrl);
			pTestButton->SetText(L"Same Size");
			pTestButton->SetBounds(0, 0, 100, 100);
		}

		{
			Gwen::Controls::ScrollControl* pCtrl = new Gwen::Controls::ScrollControl(this);
			pCtrl->SetBounds(210, 110, 100, 100);
			pCtrl->SetAutoHideBars(true);

			Controls::Button* pTestButton = new Controls::Button(pCtrl);
			pTestButton->SetText(L"Wide");
			pTestButton->SetBounds(0, 0, 200, 50);
		}

		{
			Gwen::Controls::ScrollControl* pCtrl = new Gwen::Controls::ScrollControl(this);
			pCtrl->SetBounds(310, 110, 100, 100);
			pCtrl->SetAutoHideBars(true);

			Controls::Button* pTestButton = new Controls::Button(pCtrl);
			pTestButton->SetText(L"Tall");
			pTestButton->SetBounds(0, 0, 50, 200);
		}

		{
			Gwen::Controls::ScrollControl* pCtrl = new Gwen::Controls::ScrollControl(this);
			pCtrl->SetBounds(410, 110, 100, 100);
			pCtrl->SetAutoHideBars(true);
			pCtrl->SetScroll(false, true);

			Controls::Button* pTestButton = new Controls::Button(pCtrl);
			pTestButton->SetText(L"Vertical");
			pTestButton->SetBounds(0, 0, 200, 200);
		}

		{
			Gwen::Controls::ScrollControl* pCtrl = new Gwen::Controls::ScrollControl(this);
			pCtrl->SetBounds(510, 110, 100, 100);
			pCtrl->SetAutoHideBars(true);
			pCtrl->SetScroll(true, false);

			Controls::Button* pTestButton = new Controls::Button(pCtrl);
			pTestButton->SetText(L"Horinzontal");
			pTestButton->SetBounds(0, 0, 200, 200);
		}
	}
};

DEFINE_UNIT_TEST(ScrollControl, L"Scroll");