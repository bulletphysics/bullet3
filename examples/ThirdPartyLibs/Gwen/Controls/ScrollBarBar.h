/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_SCROLLBARBAR_H
#define GWEN_CONTROLS_SCROLLBARBAR_H

#include "Gwen/Controls/Dragger.h"
#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"

namespace Gwen 
{
	namespace ControlsInternal
	{
		class GWEN_EXPORT ScrollBarBar : public ControlsInternal::Dragger
		{
			public:

				GWEN_CONTROL( ScrollBarBar, ControlsInternal::Dragger );

				virtual void Render( Skin::Base* skin );
				virtual void Layout( Skin::Base* skin );

				virtual void OnMouseMoved( int x, int y, int deltaX, int deltaY );
				virtual void OnMouseClickLeft( int x, int y, bool bDown );

				virtual void MoveTo(int x, int y);

				virtual void SetHorizontal()	{ m_bHorizontal = true; }
				virtual void SetVertical()		{ m_bHorizontal = false; }
				virtual bool IsVertical()		{ return !m_bHorizontal; }
				virtual bool IsHorizontal()		{ return m_bHorizontal; }
				virtual bool IsDepressed()		{ return m_bDepressed; }

			protected:

				bool m_bHorizontal;

		};
	}
}
#endif
