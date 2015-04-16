#pragma once
#ifndef GWEN_CONTROLS_SPLITTERBAR_H
#define GWEN_CONTROLS_SPLITTERBAR_H

#include "Gwen/Gwen.h"
#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Dragger.h"

namespace Gwen 
{
	namespace Controls
	{
		class GWEN_EXPORT SplitterBar : public ControlsInternal::Dragger
		{
			public:

				GWEN_CONTROL( SplitterBar, ControlsInternal::Dragger );

				void Render( Skin::Base* skin );
				void Layout( Skin::Base* skin );
		};
	}
}
#endif
