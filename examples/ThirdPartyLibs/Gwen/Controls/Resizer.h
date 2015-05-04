/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_RESIZER_H
#define GWEN_CONTROLS_RESIZER_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/Dragger.h"


namespace Gwen 
{
	namespace ControlsInternal
	{
		class GWEN_EXPORT Resizer : public Dragger
		{
			public:

				GWEN_CONTROL( Resizer, Dragger );

				virtual void OnMouseMoved( int x, int y, int deltaX, int deltaY );
				virtual void SetResizeDir( int dir );

				Event::Caller	onResize;

			protected:

				int		m_iResizeDir;
				
		};
	}
}
#endif
