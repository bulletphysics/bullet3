/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_COMBOBOX_H
#define GWEN_CONTROLS_COMBOBOX_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Button.h"
#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/TextBox.h"
#include "Gwen/Controls/Menu.h"


namespace Gwen 
{
	namespace Controls
	{
		class GWEN_EXPORT ComboBoxButton : public Button
		{
			GWEN_CONTROL_INLINE( ComboBoxButton, Button ){}

			virtual void Render( Skin::Base* skin )
			{
				skin->DrawComboBoxButton( this, m_bDepressed );
			}
		};

		class GWEN_EXPORT ComboBox : public Button
		{
			public:

				GWEN_CONTROL( ComboBox, Button );

				virtual void Render( Skin::Base* skin );

				virtual Gwen::Controls::Label* GetSelectedItem();

				virtual void OnPress();
				void OpenButtonPressed( Controls::Base* /*pControl*/ );

				virtual void OnItemSelected( Controls::Base* pControl );
				virtual void OpenList();
				virtual void CloseList();
				
				virtual Controls::Base* GetControlAt( int x, int y )
				{
					if ( x < 0 || y < 0 || x >= Width() || y >= Height() )
						return NULL;

					return this;
				}
				virtual bool IsMenuComponent()
				{
					return true;
				}

				virtual void ClearItems();

				virtual MenuItem* AddItem( const UnicodeString& strLabel, const String& strName = "", Gwen::Event::Handler* pHandler = NULL, Gwen::Event::Handler::Function fn = NULL );
				virtual bool OnKeyUp( bool bDown );
				virtual bool OnKeyDown( bool bDown );

				virtual void RenderFocus( Gwen::Skin::Base* skin );
				virtual void OnLostKeyboardFocus();
				virtual void OnKeyboardFocus();

				virtual bool IsMenuOpen();

				Gwen::Event::Caller	onSelection;

			protected:

				Menu* m_Menu;
				MenuItem* m_SelectedItem;

				Controls::Base*	m_Button;

		};
		
	}
}
#endif
