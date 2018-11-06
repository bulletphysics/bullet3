/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_BUTTON_H
#define GWEN_CONTROLS_BUTTON_H

#include "Gwen/TextObject.h"
#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Label.h"

namespace Gwen
{
namespace Controls
{
class ImagePanel;

class GWEN_EXPORT Button : public Label
{
public:
	GWEN_CONTROL(Button, Label);

	virtual void Render(Skin::Base* skin);
	virtual void OnMouseClickLeft(int x, int y, bool bDown);
	virtual void OnMouseDoubleClickLeft(int x, int y);
	virtual bool OnKeySpace(bool bDown);

	virtual void OnPress();

	virtual void AcceleratePressed();

	virtual bool IsDepressed() const { return m_bDepressed; }

	//
	// Buttons can be toggle type, which means that it is
	// toggled on and off. Its toggle status is in IsDepressed.
	//
	virtual void SetIsToggle(bool b) { m_bToggle = b; }
	virtual bool IsToggle() const { return m_bToggle; }
	virtual bool GetToggleState() const { return m_bToggleStatus; }
	virtual void SetToggleState(bool b);
	virtual void Toggle() { SetToggleState(!GetToggleState()); }

	virtual void SetImage(const TextObject& strName, bool bCenter = false);

	// You can use this to trigger OnPress directly from other controls using GWEN_CALL_EX
	virtual void ReceiveEventPress(Base* /*pControl*/)
	{
		OnPress();
	}

	virtual void SizeToContents();
	virtual void Layout(Skin::Base* pSkin);

	virtual bool OnKeyReturn(bool bDown)
	{
		onKeyboardReturn.Call(this);
		return true;
	}

public:
	Gwen::Event::Caller onPress;
	Gwen::Event::Caller onDown;
	Gwen::Event::Caller onUp;
	Gwen::Event::Caller onDoubleClick;
	Gwen::Event::Caller onKeyboardReturn;

	Gwen::Event::Caller onToggle;
	Gwen::Event::Caller onToggleOn;
	Gwen::Event::Caller onToggleOff;

protected:
	ImagePanel* m_Image;

	bool m_bDepressed;
	bool m_bToggle;
	bool m_bToggleStatus;

	bool m_bCenterImage;
};
}  // namespace Controls
}  // namespace Gwen
#endif
