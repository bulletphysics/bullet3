#pragma once
#ifndef GWEN_CONTROLS_STATUSBAR_H
#define GWEN_CONTROLS_STATUSBAR_H

#include "Gwen/Gwen.h"
#include "Gwen/Controls/Base.h"

namespace Gwen 
{
	namespace Controls
	{
		class StatusBar : public Controls::Base
		{
			public:

				GWEN_CONTROL_INLINE( StatusBar, Controls::Base )
				{
					SetBounds( 0, 0, 200, 22 );
					Dock( Pos::Bottom );
					SetPadding( Padding( 2, 2, 2, 2 ) );
				}

				virtual void AddControl( Controls::Base* pCtrl, bool bRight)
				{
					pCtrl->SetParent( this );
					pCtrl->Dock( bRight ? Pos::Right : Pos::Left );
				}

				virtual void Render( Skin::Base* skin )
				{
					skin->DrawStatusBar( this );
				}
		};
	}
}
#endif
