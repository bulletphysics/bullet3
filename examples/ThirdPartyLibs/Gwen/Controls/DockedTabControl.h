/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_DOCKEDTABCONTROL_H
#define GWEN_CONTROLS_DOCKEDTABCONTROL_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/TabControl.h"

namespace Gwen 
{
	namespace Controls
	{
		class GWEN_EXPORT DockedTabControl : public TabControl
		{
			public:

				GWEN_CONTROL( DockedTabControl, TabControl );

				void SetShowTitlebar( bool bShow ){ m_pTitleBar->SetHidden( !bShow ); }

				void Layout( Skin::Base* skin );
				void UpdateTitleBar();

				virtual void DragAndDrop_StartDragging( Gwen::DragAndDrop::Package* pPackage, int x, int y );
				virtual void DragAndDrop_EndDragging( bool bSuccess, int x, int y );

				void MoveTabsTo( DockedTabControl* pTarget );

			private:

				TabTitleBar*	m_pTitleBar;
				Base*			m_WindowControl;

		};
	}
}
#endif
