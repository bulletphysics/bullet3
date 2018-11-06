#include "UnitTest.h"
#include "Gwen/Controls/TextBox.h"

using namespace Gwen;

class TextBox : public GUnit
{
public:
	GWEN_CONTROL_INLINE(TextBox, GUnit)
	{
		{
			Gwen::Controls::TextBox* label = new Gwen::Controls::TextBox(this);
			label->SetText("");
			label->SetPos(10, 10);
			label->onTextChanged.Add(this, &ThisClass::OnEdit);
			label->onReturnPressed.Add(this, &ThisClass::OnSubmit);
		}

		{
			Gwen::Controls::TextBox* label = new Gwen::Controls::TextBox(this);
			label->SetText("Normal Everyday Label");
			label->SetPos(10, 10 + 25);
		}

		{
			Gwen::Controls::TextBox* label = new Gwen::Controls::TextBox(this);
			label->SetText("Select All Text On Focus");
			label->SetPos(10, 10 + 25 * 2);
			label->SetSelectAllOnFocus(true);
		}

		{
			Gwen::Controls::TextBox* label = new Gwen::Controls::TextBox(this);
			label->SetText(L"Different Coloured Text, for some reason");
			label->SetTextColor(Gwen::Color(255, 0, 255, 255));
			label->SetPos(10, 10 + 25 * 3);
		}

		{
			Gwen::Controls::TextBoxNumeric* label = new Gwen::Controls::TextBoxNumeric(this);
			label->SetText(L"2004");
			label->SetTextColor(Gwen::Color(255, 0, 255, 255));
			label->SetPos(10, 10 + 25 * 4);
		}

		{
			m_Font.facename = L"Impact";
			m_Font.size = 50;

			Gwen::Controls::TextBox* label = new Gwen::Controls::TextBox(this);
			label->SetText(L"Different Font");
			label->SetPos(10, 10 + 25 * 5);
			label->SetFont(&m_Font);
			label->SetSize(200, 55);
		}
	}

	void OnEdit(Gwen::Controls::Base* pControl)
	{
		Gwen::Controls::TextBox* textbox = (Gwen::Controls::TextBox*)(pControl);
		UnitPrint(Utility::Format(L"Textbox Edit: [%s]\n", textbox->GetText().c_str()));
	}

	void OnSubmit(Gwen::Controls::Base* pControl)
	{
		Gwen::Controls::TextBox* textbox = (Gwen::Controls::TextBox*)(pControl);
		UnitPrint(Utility::Format(L"Textbox Submit: [%s]\n", textbox->GetText().c_str()));
	}

	Gwen::Font m_Font;
};

DEFINE_UNIT_TEST(TextBox, L"TextBox");