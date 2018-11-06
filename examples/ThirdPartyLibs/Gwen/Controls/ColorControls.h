/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_COLORCONTROLS_H
#define GWEN_CONTROLS_COLORCONTROLS_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"

namespace Gwen
{
namespace Controls
{
class GWEN_EXPORT ColorLerpBox : public Controls::Base
{
public:
	GWEN_CONTROL(ColorLerpBox, Controls::Base);
	virtual void Render(Gwen::Skin::Base* skin);
	Gwen::Color GetColorAtPos(int x, int y);
	void SetColor(Gwen::Color color, bool onlyHue = true);
	virtual void OnMouseMoved(int x, int y, int deltaX, int deltaY);
	virtual void OnMouseClickLeft(int x, int y, bool bDown);
	Gwen::Color GetSelectedColor();

	Event::Caller onSelectionChanged;

protected:
	Gwen::Point cursorPos;
	bool m_bDepressed;
	int m_Hue;
};

class GWEN_EXPORT ColorSlider : public Controls::Base
{
public:
	GWEN_CONTROL(ColorSlider, Controls::Base);
	virtual void Render(Gwen::Skin::Base* skin);
	virtual void OnMouseMoved(int x, int y, int deltaX, int deltaY);
	virtual void OnMouseClickLeft(int x, int y, bool bDown);
	Gwen::Color GetSelectedColor();
	Gwen::Color GetColorAtHeight(int y);
	void SetColor(Gwen::Color color);

	Event::Caller onSelectionChanged;

protected:
	int m_iSelectedDist;
	bool m_bDepressed;
};
}  // namespace Controls

}  // namespace Gwen
#endif
