/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_TEXT_H
#define GWEN_CONTROLS_TEXT_H

#include "Gwen/BaseRender.h"
#include "Gwen/Controls/Base.h"

namespace Gwen 
{
	namespace ControlsInternal
	{
		class GWEN_EXPORT Text : public Controls::Base
		{
			public:

				GWEN_CONTROL( Text, Controls::Base );

				virtual ~Text();
				Gwen::Font* GetFont();

				void SetString( const UnicodeString& str );
				void SetString( const String& str );

				void Render( Skin::Base* skin );
				void Layout( Skin::Base* skin );

				void RefreshSize();

				void SetFont( Gwen::Font* pFont ){ m_Font = pFont; }

				const UnicodeString& GetText() const { return m_String; }

				Gwen::Point GetCharacterPosition( int iChar );
				int GetClosestCharacter( Gwen::Point p );

				int Length() const { return (int)m_String.size(); }

				virtual void SetTextColor( const Gwen::Color& col ){ m_Color = col; }

				virtual void OnScaleChanged();

				inline const Gwen::Color &TextColor() const { return m_Color; }

			private:

				Gwen::UnicodeString	m_String;
				Gwen::Font*			m_Font;
				Gwen::Color			m_Color;
		};
	}

}
#endif
