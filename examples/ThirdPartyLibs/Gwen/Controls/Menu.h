/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_MENU_H
#define GWEN_CONTROLS_MENU_H

#include "Gwen/BaseRender.h"
#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/MenuItem.h"
#include "Gwen/Controls/ScrollControl.h"

namespace Gwen 
{
	namespace Controls
	{
		class MenuItem;

		class GWEN_EXPORT Menu : public ScrollControl
		{
			public:

				GWEN_CONTROL( Menu, ScrollControl );

				virtual void Render( Skin::Base* skin );
				virtual void RenderUnder( Skin::Base* skin );

				virtual void Layout( Skin::Base* skin );

				virtual MenuItem* AddItem( const Gwen::UnicodeString& strName, const UnicodeString& strIconName, Gwen::Event::Handler* pHandler = NULL, Gwen::Event::Handler::Function fn = NULL );

				virtual MenuItem* AddItem( const Gwen::UnicodeString& strName, Gwen::Event::Handler* pHandler = NULL, Gwen::Event::Handler::Function fn = NULL )
				{
					return AddItem( strName, L"", pHandler, fn );
				}

				virtual MenuItem* AddItem( const Gwen::String& strName, const String& strIconName, Gwen::Event::Handler* pHandler = NULL, Gwen::Event::Handler::Function fn = NULL );

				virtual MenuItem* AddItem( const Gwen::String& strName, Gwen::Event::Handler* pHandler = NULL, Gwen::Event::Handler::Function fn = NULL )
				{
					return AddItem( strName, "", pHandler, fn );
				}

				virtual void AddDivider();

				void OnHoverItem( Gwen::Controls::Base* pControl );
				void CloseAll();
				bool IsMenuOpen();
				void ClearItems();

				virtual void Close();

				virtual bool IsMenuComponent(){ return true; }
				virtual void CloseMenus();

				bool IconMarginDisabled() { return m_bDisableIconMargin; }
				void SetDisableIconMargin( bool bDisable ) { m_bDisableIconMargin = bDisable; }

				virtual bool ShouldClip(){ return false; }

			protected:

				virtual bool ShouldHoverOpenMenu(){ return true; }
				virtual void OnAddItem( MenuItem* item );
			
				bool m_bDisableIconMargin;
		};

		class GWEN_EXPORT MenuDivider : public Base
		{
			public:

				GWEN_CONTROL_INLINE( MenuDivider, Base )
				{
					SetHeight( 1 );
				}

				void Render( Gwen::Skin::Base* skin );
		};
	}

}
#endif
