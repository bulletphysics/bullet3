#include "UnitTest.h"
#include "Gwen/Controls/ComboBox.h"

using namespace Gwen;

class ComboBox : public GUnit
{
public:
	GWEN_CONTROL_INLINE(ComboBox, GUnit)
	{
		{
			Gwen::Controls::ComboBox* combo = new Gwen::Controls::ComboBox(this);
			combo->SetKeyboardInputEnabled(true);
			combo->SetPos(50, 50);
			combo->SetWidth(200);

			combo->AddItem(L"Option One", "one");
			combo->AddItem(L"Number Two", "two");
			combo->AddItem(L"Door Three", "three");
			combo->AddItem(L"Four Legs", "four");
			combo->AddItem(L"Five Birds", "five");

			combo->onSelection.Add(this, &ComboBox::OnComboSelect);
		}

		{
			// Empty..
			Gwen::Controls::ComboBox* combo = new Gwen::Controls::ComboBox(this);
			combo->SetPos(50, 80);
			combo->SetWidth(200);
		}

		{
			// Empty..
			Gwen::Controls::ComboBox* combo = new Gwen::Controls::ComboBox(this);
			combo->SetPos(50, 110);
			combo->SetWidth(200);

			for (int i = 0; i < 500; i++)
				combo->AddItem(L"Lots Of Options");
		}
	}

	void OnComboSelect(Gwen::Controls::Base* pControl)
	{
		Gwen::Controls::ComboBox* combo = (Gwen::Controls::ComboBox*)pControl;

		UnitPrint(Utility::Format(L"Combo Changed: %s", combo->GetSelectedItem()->GetText().c_str()));
	}
};

DEFINE_UNIT_TEST(ComboBox, L"ComboBox");