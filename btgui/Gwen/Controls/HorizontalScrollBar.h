/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_HORIZONTALSCROLLBAR_H
#define GWEN_CONTROLS_HORIZONTALSCROLLBAR_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Button.h"
#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/Dragger.h"
#include "Gwen/Controls/ScrollBar.h"

namespace Gwen 
{
	namespace Controls
	{
		class GWEN_EXPORT HorizontalScrollBar : public BaseScrollBar
		{
			public:

				GWEN_CONTROL( HorizontalScrollBar, BaseScrollBar );

				virtual void Layout( Skin::Base* skin );

				virtual void OnMouseClickLeft( int x, int y, bool bDown );
				virtual void OnBarMoved( Controls::Base* control );

				virtual int GetBarSize() { return m_Bar->Width(); }
				virtual int GetBarPos() { return m_Bar->X() - Height(); }
				virtual void SetBarSize( int size ) { m_Bar->SetWidth( size ); }
				virtual int GetButtonSize() { return Height(); }

				virtual void ScrollToLeft();
				virtual void ScrollToRight();
				virtual void NudgeLeft( Base* control );
				virtual void NudgeRight( Base* control );
				virtual float GetNudgeAmount();
		
				virtual float CalculateScrolledAmount();
				virtual bool SetScrolledAmount(float amount, bool forceUpdate);
		};
	}
}
#endif
