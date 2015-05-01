/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_DRAGGER_H
#define GWEN_CONTROLS_DRAGGER_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"


namespace Gwen 
{
	namespace ControlsInternal
	{
		class GWEN_EXPORT Dragger : public Controls::Base
		{
			public:

				GWEN_CONTROL( Dragger, Controls::Base );

				virtual void OnMouseMoved( int x, int y, int deltaX, int deltaY );

				virtual void OnMouseClickLeft( int x, int y, bool bDown );
				virtual void Render( Skin::Base* skin );

				virtual void SetTarget( Controls::Base* pBase ){ m_pTarget = pBase; }

				Gwen::Event::Caller	onDragged;

			protected:

				bool				m_bDepressed;
				Gwen::Point				m_HoldPos;
				Controls::Base*		m_pTarget;
		};
	}
}
#endif
