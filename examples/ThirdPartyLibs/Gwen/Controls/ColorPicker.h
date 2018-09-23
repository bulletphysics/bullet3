/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_COLORPICKER_H
#define GWEN_CONTROLS_COLORPICKER_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"

namespace Gwen
{
namespace ControlsInternal
{
class GWEN_EXPORT ColorDisplay : public Controls::Base
{
public:
	GWEN_CONTROL_INLINE(ColorDisplay, Controls::Base)
	{
		SetSize(32, 32);
		m_Color = Color(255, 0, 0, 255);
		m_DrawCheckers = true;
	}

	virtual void Render(Gwen::Skin::Base* skin)
	{
		skin->DrawColorDisplay(this, m_Color);
	}

	virtual void SetColor(Gwen::Color color) { m_Color = color; }
	virtual Gwen::Color GetColor() { return m_Color; }

	virtual void SetRed(int red) { m_Color.r = red; }
	virtual void SetGreen(int green) { m_Color.g = green; }
	virtual void SetBlue(int blue) { m_Color.b = blue; }
	virtual void SetAlpha(int alpha) { m_Color.a = alpha; }

	virtual void SetDrawCheckers(bool should) { m_DrawCheckers = should; }

protected:
	Gwen::Color m_Color;
	bool m_DrawCheckers;
};
}  // namespace ControlsInternal
namespace Controls
{
class GWEN_EXPORT ColorPicker : public Base
{
public:
	GWEN_CONTROL(ColorPicker, Base);

	virtual void Render(Skin::Base* skin);
	virtual void Layout(Skin::Base* skin);
	virtual void CreateControls();
	virtual void SlidersMoved(Gwen::Controls::Base* control);
	virtual void NumericTyped(Gwen::Controls::Base* control);
	virtual void UpdateControls();
	virtual void UpdateColorControls(Gwen::String name, Gwen::Color col, int sliderVal);
	virtual void CreateColorControl(Gwen::String name, int y);

	virtual void SetColor(Gwen::Color color);
	virtual Gwen::Color GetColor() { return m_Color; }

	int GetColorByName(Gwen::String colorName);
	void SetColorByName(Gwen::String colorName, int colorValue);
	Gwen::String GetColorFromName(Gwen::String name);
	virtual void SetAlphaVisible(bool visible);

	virtual void SetRed(int red) { m_Color.r = red; }
	virtual void SetGreen(int green) { m_Color.g = green; }
	virtual void SetBlue(int blue) { m_Color.b = blue; }
	virtual void SetAlpha(int alpha) { m_Color.a = alpha; }

	Event::Caller onColorChanged;

protected:
	Gwen::Color m_Color;
};
}  // namespace Controls
}  // namespace Gwen
#endif
