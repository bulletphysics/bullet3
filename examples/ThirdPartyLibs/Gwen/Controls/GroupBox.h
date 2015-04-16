/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_GROUPBOX_H
#define GWEN_CONTROLS_GROUPBOX_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Label.h"
#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"


namespace Gwen 
{
	namespace Controls
	{

		class GWEN_EXPORT GroupBox : public Label
		{
			public:

				GWEN_CONTROL( GroupBox, Label );

				virtual void Render( Skin::Base* skin );
				virtual void Layout( Skin::Base* skin );

		};
	}
}
#endif
