/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_SCROLLCONTROL_H
#define GWEN_CONTROLS_SCROLLCONTROL_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Button.h"
#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/ScrollBar.h"
#include "Gwen/Controls/VerticalScrollBar.h"
#include "Gwen/Controls/HorizontalScrollBar.h"


namespace Gwen 
{
	namespace Controls
	{
		class GWEN_EXPORT ScrollControl : public Base
		{
			public:

				GWEN_CONTROL( ScrollControl, Base );

				virtual void Layout( Skin::Base* skin );
				virtual void Render( Skin::Base* skin );

				virtual void SetScroll( bool h, bool v );
				virtual void SetAutoHideBars(bool should) { m_bAutoHideBars = should; }
				virtual bool CanScrollH() { return m_bCanScrollH; }
				virtual bool CanScrollV() { return m_bCanScrollV; }
				virtual void OnChildBoundsChanged( Gwen::Rect oldChildBounds, Base* pChild );
				virtual void UpdateScrollBars();

				virtual void SetVScrollRequired(bool req);
				virtual void SetHScrollRequired(bool req);

				virtual void SetInnerSize( int w, int h );

				virtual void VBarMoved(Controls::Base * control);
				virtual void HBarMoved(Controls::Base * control);

				virtual bool OnMouseWheeled( int iDelta );

				virtual void ScrollToBottom();
				virtual void ScrollToTop();
				virtual void ScrollToLeft();
				virtual void ScrollToRight();

				virtual void Clear();

			protected:

				bool m_bCanScrollH;
				bool m_bCanScrollV;

				bool m_bAutoHideBars;

		public:
				Controls::BaseScrollBar* m_VerticalScrollBar;
				Controls::BaseScrollBar* m_HorizontalScrollBar;
		};
	}
}
#endif
