/*
	GWEN
	Copyright (c) 2011 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_TOOLBAR_H
#define GWEN_CONTROLS_TOOLBAR_H

#include "Gwen/BaseRender.h"
#include "Gwen/Controls/Base.h"
#include "Gwen/Skin.h"

namespace Gwen 
{
	namespace Controls
	{
		/*

		TODO!

		*/

		class ToolBarStrip : public Base
		{
			GWEN_CONTROL_INLINE( ToolBarStrip, Base )
			{
				SetPadding( Padding( 2, 2, 2, 2 ) );
			}

			virtual void Render( Skin::Base* skin )
			{
				skin->DrawMenuStrip( this );
			}

			virtual void RenderUnder( Skin::Base* skin )
			{

			}

			virtual void Layout( Skin::Base* skin )
			{

			}

		};
	}

}
#endif
