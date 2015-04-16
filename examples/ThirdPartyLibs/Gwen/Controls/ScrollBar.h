/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_SCROLLBAR_H
#define GWEN_CONTROLS_SCROLLBAR_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/ScrollBarBar.h"
#include "Gwen/Controls/ScrollBarButton.h"
#include "Gwen/Gwen.h"


#define SCROLL_BUTTON_UP 0
#define SCROLL_BUTTON_LEFT 0
#define SCROLL_BUTTON_DOWN 1
#define SCROLL_BUTTON_RIGHT 1
#define NUDGE_DIST 10

namespace Gwen 
{
	namespace Controls
	{
		class GWEN_EXPORT BaseScrollBar : public Base
		{
			public:
				
				GWEN_CONTROL( BaseScrollBar, Base );

				virtual void Render( Skin::Base* skin );

				virtual void SetBarSize(int size) = 0;
				virtual int GetBarSize() = 0;
				virtual int GetBarPos() = 0;

				virtual void OnBarMoved( Controls::Base* control);
				virtual void OnMouseClickLeft( int /*x*/, int /*y*/, bool /*bDown*/ ){}

				virtual void ScrollToLeft(){}
				virtual void ScrollToRight(){}
				virtual void ScrollToTop(){}
				virtual void ScrollToBottom(){}

				virtual float GetNudgeAmount() { return m_fNudgeAmount / m_fContentSize; }
				virtual void SetNudgeAmount( float nudge ) { m_fNudgeAmount = nudge; }

				virtual void BarMovedNotification();
				
				virtual float CalculateScrolledAmount() { return 0; } 
				virtual int CalculateBarSize() { return 0; }
				virtual bool SetScrolledAmount(float amount, bool forceUpdate);
				
				virtual void SetContentSize(float size);
				virtual void SetViewableContentSize(float size);

				virtual int GetButtonSize() { return 0; }
				virtual float GetScrolledAmount() { return m_fScrolledAmount; }

				Gwen::Event::Caller	onBarMoved;

				float getContentSize() 
				{
						return m_fContentSize;
				}
				float getViewableContentSize() const
				{
				
					return m_fViewableContentSize;
				}

			protected:

				ControlsInternal::ScrollBarButton* m_ScrollButton[2];
				ControlsInternal::ScrollBarBar * m_Bar;
				
				bool m_bDepressed;
				float m_fScrolledAmount;
				float m_fContentSize;
				float m_fViewableContentSize;
				float m_fNudgeAmount;
		};
	}
}
#endif
