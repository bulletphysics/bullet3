/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_ALIGN_H
#define GWEN_ALIGN_H
#include "Gwen/Controls/Base.h"

namespace Gwen 
{
	namespace Align
	{
		inline void Center( Controls::Base* ctrl )
		{
			Controls::Base* parent = ctrl->GetParent(); 
			if ( !parent ) return;

			ctrl->SetPos( parent->GetPadding().left + (((parent->Width()-parent->GetPadding().left-parent->GetPadding().right) - ctrl->Width()) / 2), 
								(parent->Height() - ctrl->Height()) / 2 );
		}

		inline void AlignLeft( Controls::Base* ctrl )
		{
			Controls::Base* parent = ctrl->GetParent(); 
			if ( !parent ) return;

			ctrl->SetPos( parent->GetPadding().left, ctrl->Y() );
		}

		inline void CenterHorizontally( Controls::Base* ctrl )
		{
			Controls::Base* parent = ctrl->GetParent(); 
			if ( !parent ) return;


			ctrl->SetPos( parent->GetPadding().left + (((parent->Width()-parent->GetPadding().left-parent->GetPadding().right) - ctrl->Width()) / 2), ctrl->Y() );
		}

		inline void AlignRight( Controls::Base* ctrl )
		{
			Controls::Base* parent = ctrl->GetParent(); 
			if ( !parent ) return;


			ctrl->SetPos( parent->Width() - ctrl->Width() - parent->GetPadding().right, ctrl->Y() );
		}

		inline void AlignTop( Controls::Base* ctrl )
		{
			ctrl->SetPos( ctrl->X(), 0 );
		}

		inline void CenterVertically( Controls::Base* ctrl )
		{
			Controls::Base* parent = ctrl->GetParent(); 
			if ( !parent ) return;

			ctrl->SetPos( ctrl->X(), (parent->Height() - ctrl->Height()) / 2 );
		}

		inline void AlignBottom( Controls::Base* ctrl )
		{
			Controls::Base* parent = ctrl->GetParent(); 
			if ( !parent ) return;
	
			ctrl->SetPos( ctrl->X(), parent->Height() - ctrl->Height() );
		}

		inline void PlaceBelow( Controls::Base* ctrl, Controls::Base* below, int iBorder = 0 )
		{
			ctrl->SetPos( ctrl->X(), below->Bottom() + iBorder );
		}

	}
}
#endif
