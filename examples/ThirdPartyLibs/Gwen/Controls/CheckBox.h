/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_CHECKBOX_H
#define GWEN_CONTROLS_CHECKBOX_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Button.h"
#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/Symbol.h"
#include "Gwen/Controls/LabelClickable.h"

namespace Gwen
{
namespace Controls
{
class GWEN_EXPORT CheckBox : public Button
{
public:
	GWEN_CONTROL(CheckBox, Button);

	virtual void Render(Skin::Base* skin);
	virtual void OnPress();

	virtual void SetChecked(bool Checked);
	virtual void Toggle() { SetChecked(!IsChecked()); }
	virtual bool IsChecked() { return m_bChecked; }

	Gwen::Event::Caller onChecked;
	Gwen::Event::Caller onUnChecked;
	Gwen::Event::Caller onCheckChanged;

private:
	// For derived controls
	virtual bool AllowUncheck() { return true; }

	void OnCheckStatusChanged();

	bool m_bChecked;
};

class GWEN_EXPORT CheckBoxWithLabel : public Base
{
public:
	GWEN_CONTROL_INLINE(CheckBoxWithLabel, Base)
	{
		SetSize(200, 19);

		m_Checkbox = new CheckBox(this);
		m_Checkbox->Dock(Pos::Left);
		m_Checkbox->SetMargin(Margin(0, 3, 3, 3));
		m_Checkbox->SetTabable(false);

		m_Label = new LabelClickable(this);
		m_Label->Dock(Pos::Fill);
		m_Label->onPress.Add(m_Checkbox, &CheckBox::ReceiveEventPress);
		m_Label->SetTabable(false);

		SetTabable(false);
	}

	virtual CheckBox* Checkbox() { return m_Checkbox; }
	virtual LabelClickable* Label() { return m_Label; }
	virtual bool OnKeySpace(bool bDown)
	{
		if (bDown) m_Checkbox->SetChecked(!m_Checkbox->IsChecked());
		return true;
	}

private:
	CheckBox* m_Checkbox;
	LabelClickable* m_Label;
};
}  // namespace Controls
}  // namespace Gwen
#endif
