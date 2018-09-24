/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_HORIZONTALSLIDER_H
#define GWEN_CONTROLS_HORIZONTALSLIDER_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Button.h"
#include "Gwen/Controls/Dragger.h"
#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/Slider.h"

namespace Gwen
{
namespace Controls
{
class GWEN_EXPORT HorizontalSlider : public Slider
{
	GWEN_CONTROL(HorizontalSlider, Slider);

	virtual void Layout(Skin::Base* skin);
	virtual void Render(Skin::Base* skin);

	virtual float CalculateValue();
	virtual void UpdateBarFromValue();
	virtual void OnMouseClickLeft(int x, int y, bool bDown);
};
}  // namespace Controls
}  // namespace Gwen
#endif
