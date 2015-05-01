/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_VERTICALSLIDER_H
#define GWEN_CONTROLS_VERTICALSLIDER_H

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
		class GWEN_EXPORT VerticalSlider : public Slider
		{
			GWEN_CONTROL( VerticalSlider, Slider );

			virtual void Layout( Skin::Base* skin );
			virtual void Render( Skin::Base* skin );

			virtual float CalculateValue();
			virtual void UpdateBarFromValue();
			virtual void OnMouseClickLeft( int x, int y, bool bDown );

		};
	}
}
#endif
