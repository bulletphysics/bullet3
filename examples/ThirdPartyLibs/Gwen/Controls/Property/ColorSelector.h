#pragma once
#ifndef GWEN_CONTROLS_PROPERTY_COLORSELECTOR_H
#define GWEN_CONTROLS_PROPERTY_COLORSELECTOR_H

#include "Gwen/Controls/Properties.h"
#include "Gwen/Controls/WindowControl.h"
#include "Gwen/Controls/HSVColorPicker.h"

namespace Gwen
{
namespace Controls
{
namespace Property
{
class ColorSelector : public Property::Text
{
public:
	GWEN_CONTROL_INLINE(ColorSelector, Property::Text)
	{
		m_Button = new Button(this);
		m_Button->Dock(Pos::Right);
		m_Button->SetWidth(20);
		m_Button->onPress.Add(this, &ThisClass::OnButtonPress);
	}

	void OnButtonPress(Controls::Base* control)
	{
		Gwen::Controls::WindowControl* wind = new Gwen::Controls::WindowControl(GetCanvas());
		wind->SetTitle(L"Color Selection");
		wind->SetSize(256, 180);
		wind->SetPos(GetCanvas()->Width() * 0.5 - 128, GetCanvas()->Height() * 0.5 - 128);
		wind->SetDeleteOnClose(true);
		wind->DisableResizing();
		wind->MakeModal(true);

		Gwen::Controls::HSVColorPicker* picker = new Gwen::Controls::HSVColorPicker(wind);
		picker->SetName("picker");

		float defaultColor[3];
		Gwen::Utility::Strings::To::Floats(Gwen::Utility::UnicodeToString(m_TextBox->GetText()), defaultColor, 3);

		picker->SetColor(Gwen::Color(defaultColor[0], defaultColor[1], defaultColor[2], 255), false, true);
		picker->onColorChanged.Add(this, &ThisClass::ColorChanged);
	}

	void ColorChanged(Controls::Base* control)
	{
		Gwen::Controls::HSVColorPicker* picker = control->DynamicCastHSVColorPicker();

		Gwen::String colorStr;
		colorStr += Gwen::Utility::ToString((int)picker->GetColor().r) + " ";
		colorStr += Gwen::Utility::ToString((int)picker->GetColor().g) + " ";
		colorStr += Gwen::Utility::ToString((int)picker->GetColor().b);

		m_TextBox->SetText(colorStr);
		DoChanged();
	}

	virtual UnicodeString GetPropertyValue()
	{
		return m_TextBox->GetText();
	}

	virtual void SetPropertyValue(const UnicodeString& v, bool bFireChangeEvents)
	{
		m_TextBox->SetText(v, bFireChangeEvents);
	}

	virtual bool IsEditing()
	{
		return m_TextBox == Gwen::KeyboardFocus;
	}

	Button* m_Button;
};
}  // namespace Property
}  // namespace Controls
}  // namespace Gwen
#endif
