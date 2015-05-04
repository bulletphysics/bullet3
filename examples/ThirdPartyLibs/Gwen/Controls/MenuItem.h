/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_MENUITEM_H
#define GWEN_CONTROLS_MENUITEM_H

#include "Gwen/BaseRender.h"
#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Button.h"
#include "Gwen/Controls/Menu.h"
#include "Gwen/Controls/Symbol.h"

namespace Gwen 
{
	namespace Controls
	{
		class Menu;

		class GWEN_EXPORT MenuItem : public Button
		{
			public:

				GWEN_CONTROL( MenuItem, Button );

				virtual ~MenuItem();

				virtual void Render( Skin::Base* skin );
				virtual void Layout( Skin::Base* skin );

				virtual void OnPress();

				Menu* GetMenu();

				bool IsMenuOpen();
				void OpenMenu();
				void CloseMenu();
				void ToggleMenu();

				void SetOnStrip( bool b ){ m_bOnStrip = b;}
				bool OnStrip(){ return m_bOnStrip; }

				virtual void SetCheckable( bool bCheck ) { m_bCheckable = bCheck; }
				virtual void SetCheck( bool bCheck );
				virtual bool GetChecked() { return m_bChecked; }

				Gwen::Event::Caller	onMenuItemSelected;
				Gwen::Event::Caller	onChecked;
				Gwen::Event::Caller	onUnChecked;
				Gwen::Event::Caller	onCheckChange;

			private:

				Menu*	m_Menu;
				bool	m_bOnStrip;
				bool	m_bCheckable;
				bool	m_bChecked;

				

				Symbol::Arrow	*	m_SubmenuArrow;
		};
	}

}
#endif
