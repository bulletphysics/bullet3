/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_LABEL_H
#define GWEN_CONTROLS_LABEL_H

#include "Gwen/BaseRender.h"
#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Text.h"

namespace Gwen 
{
	namespace Controls
	{
		class GWEN_EXPORT Label : public Controls::Base
		{
			public:

				GWEN_CONTROL( Label, Controls::Base );

				virtual void SetText( const UnicodeString& str, bool bDoEvents = true );
				virtual void SetText( const String& str, bool bDoEvents = true );

				virtual const UnicodeString& GetText() const { return m_Text->GetText(); }

				virtual void Render( Skin::Base* /*skin*/ ){}

				virtual void Layout( Skin::Base* skin );

				virtual void SizeToContents();

				virtual void SetAlignment( int iAlign ){ m_iAlign = iAlign; }

				virtual void SetFont( Gwen::Font* pFont ){ m_Text->SetFont( pFont ); }
				virtual Gwen::Font* GetFont(){ return m_Text->GetFont(); }
				virtual void SetTextColor( const Gwen::Color& col ){ m_Text->SetTextColor( col ); }
				inline const Gwen::Color &TextColor() const { return m_Text->TextColor(); }

				virtual int TextWidth() { return m_Text->Width(); }
				virtual int TextRight() { return m_Text->Right(); }
				virtual int TextHeight() { return m_Text->Height(); }
				virtual int TextX() { return m_Text->X(); }
				virtual int TextY() { return m_Text->Y(); }
				virtual int TextLength() { return m_Text->Length(); }

				Gwen::Point GetCharacterPosition( int iChar );

				virtual void SetTextPadding( const Padding& padding ){ m_rTextPadding = padding; Invalidate(); InvalidateParent(); }
				virtual const Padding& GetTextPadding(){ return m_rTextPadding; }

				virtual Gwen::UnicodeString GetText() { return m_Text->GetText(); }

				inline int Alignment() const { return m_iAlign; }
			protected:

				virtual void OnTextChanged(){};

				Padding m_rTextPadding;
				ControlsInternal::Text*	m_Text;
				int m_iAlign;


		};
	}
}
#endif
