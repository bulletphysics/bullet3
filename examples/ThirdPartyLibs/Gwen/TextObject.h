/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_TEXTOBJECT_H
#define GWEN_TEXTOBJECT_H

#include "Gwen/Gwen.h"
#include "Gwen/Utility.h"

namespace Gwen
{
	class TextObject
	{
		public:

			TextObject(){}

			TextObject( const Gwen::String& text )
			{
				*this = text;
			}

			TextObject( const char* text )
			{
				*this = Gwen::String( text );
			}

			TextObject( const wchar_t* text )
			{
				*this = Gwen::UnicodeString( text );
			}

			TextObject( const Gwen::UnicodeString& unicode )
			{
				*this = unicode;
			}
			
			void operator = ( const Gwen::String& str )
			{
				m_Data = Gwen::Utility::StringToUnicode( str );
			}

			void operator = ( const Gwen::UnicodeString& unicodeStr )
			{
				m_Data = unicodeStr;
			}
			
			Gwen::String Get() const
			{
				return Gwen::Utility::UnicodeToString( m_Data );
			}

			const Gwen::UnicodeString& GetUnicode() const
			{
				return m_Data;
			}

			Gwen::UnicodeString m_Data;
	};
}
#endif
