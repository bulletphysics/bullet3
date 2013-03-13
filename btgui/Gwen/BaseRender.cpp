/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Gwen.h"
#include "Gwen/BaseRender.h"
#include "Gwen/Utility.h"
#include "Gwen/Platform.h"

#include <math.h>

namespace Gwen
{
	namespace Renderer
	{

		Base::Base()
		{
			m_RenderOffset = Gwen::Point( 0, 0 );
			m_fScale = 1.0f;
		}

		Base::~Base()
		{
			if ( GetCTT() )
				GetCTT()->ShutDown();
		}

		void Base::RenderText( Gwen::Font* pFont, Gwen::Point pos, const Gwen::String& text )
		{
			Gwen::UnicodeString str = Gwen::Utility::StringToUnicode( text );
			RenderText( pFont, pos, str );
		}

		Gwen::Point Base::MeasureText( Gwen::Font* pFont, const Gwen::String& text )
		{
			Gwen::UnicodeString str = Gwen::Utility::StringToUnicode( text );
			return MeasureText( pFont, str );
		}
				
		void Base::DrawLinedRect( Gwen::Rect rect )
		{
			DrawFilledRect( Gwen::Rect( rect.x, rect.y, rect.w, 1 ) );
			DrawFilledRect( Gwen::Rect( rect.x, rect.y + rect.h-1, rect.w, 1 ) );

			DrawFilledRect( Gwen::Rect( rect.x, rect.y, 1, rect.h ) );
			DrawFilledRect( Gwen::Rect( rect.x + rect.w-1, rect.y, 1, rect.h ) );
		};

		void Base::DrawPixel( int x, int y )
		{
			DrawFilledRect( Gwen::Rect( x, y, 1, 1 ) );
		}

		void Base::DrawShavedCornerRect( Gwen::Rect rect, bool bSlight )
		{
			// Draw INSIDE the w/h.
			rect.w -= 1;
			rect.h -= 1;

			if ( bSlight )
			{
				DrawFilledRect( Gwen::Rect( rect.x+1, rect.y, rect.w-1, 1 ) );
				DrawFilledRect( Gwen::Rect( rect.x+1, rect.y + rect.h, rect.w-1, 1 ) );

				DrawFilledRect( Gwen::Rect( rect.x, rect.y+1, 1, rect.h-1 ) );
				DrawFilledRect( Gwen::Rect( rect.x + rect.w, rect.y+1, 1, rect.h-1 ) );
				return;
			}

			DrawPixel( rect.x+1, rect.y+1 );
			DrawPixel( rect.x+rect.w-1, rect.y+1 );

			DrawPixel( rect.x+1, rect.y+rect.h-1 );
			DrawPixel( rect.x+rect.w-1, rect.y+rect.h-1 );

			DrawFilledRect( Gwen::Rect( rect.x+2, rect.y, rect.w-3, 1 ) );
			DrawFilledRect( Gwen::Rect( rect.x+2, rect.y + rect.h, rect.w-3, 1 ) );

			DrawFilledRect( Gwen::Rect( rect.x, rect.y+2, 1, rect.h-3 ) );
			DrawFilledRect( Gwen::Rect( rect.x + rect.w, rect.y+2, 1, rect.h-3 ) );
		}

		void Base::Translate( int& x, int& y )
		{
			x += m_RenderOffset.x;
			y += m_RenderOffset.y;

			x = ceil( ((float) x ) * m_fScale );
			y = ceil( ((float) y ) * m_fScale );
		}

		void Base::Translate( Gwen::Rect& rect )
		{
			Translate( rect.x, rect.y );

			rect.w = ceil(((float) rect.w ) * m_fScale);
			rect.h = ceil(((float) rect.h ) * m_fScale);
		}

		void Gwen::Renderer::Base::SetClipRegion( Gwen::Rect rect )
		{ 
			m_rectClipRegion = rect; 
		}

		void Base::AddClipRegion( Gwen::Rect rect ) 
		{ 
			rect.x = m_RenderOffset.x;
			rect.y = m_RenderOffset.y;

			Gwen::Rect out = rect;
			if ( rect.x < m_rectClipRegion.x )
			{
				out.w -= ( m_rectClipRegion.x - out.x );
				out.x = m_rectClipRegion.x;
			}

			if ( rect.y < m_rectClipRegion.y )
			{
				out.h -= ( m_rectClipRegion.y - out.y );
				out.y = m_rectClipRegion.y;
			}

			if ( rect.x + rect.w > m_rectClipRegion.x + m_rectClipRegion.w )
			{
				out.w = (m_rectClipRegion.x + m_rectClipRegion.w) - out.x;
			}

			if ( rect.y + rect.h > m_rectClipRegion.y + m_rectClipRegion.h )
			{
				out.h = (m_rectClipRegion.y + m_rectClipRegion.h) - out.y;
			}

			m_rectClipRegion = out;
		}

		const Gwen::Rect& Base::ClipRegion() const
		{ 
			return m_rectClipRegion; 
		}

		bool Base::ClipRegionVisible()
		{
			if ( m_rectClipRegion.w <= 0 || m_rectClipRegion.h <= 0 )
				return false;
			
			return true;
		}

		void Base::DrawMissingImage( Gwen::Rect pTargetRect )
		{
			SetDrawColor( Colors::Red );
			DrawFilledRect( pTargetRect );
		}

		
		/*
			If they haven't defined these font functions in their renderer code
			we just draw some rects where the letters would be to give them an idea.
		*/

		void Base::RenderText( Gwen::Font* pFont, Gwen::Point pos, const Gwen::UnicodeString& text )
		{
			float fSize = pFont->size * Scale();

			for ( float i=0; i<text.length(); i++ )
			{
				wchar_t chr = text[i];

				if ( chr == ' ' ) continue;

				Gwen::Rect r( pos.x + i * fSize * 0.4, pos.y, fSize * 0.4 -1, fSize );

				/*
					This isn't important, it's just me messing around changing the
					shape of the rect based on the letter.. just for fun.
				*/
				if ( chr == 'l' || chr == 'i' || chr == '!' || chr == 't' )
				{
					r.w = 1;
				}
				else if ( chr >= 'a' && chr <= 'z' )
				{
					r.y += fSize * 0.5f;
					r.h -= fSize * 0.4f;
				}
				else if ( chr == '.' || chr == ',' )
				{
					r.x += 2;
					r.y += r.h - 2;
					r.w = 2;
					r.h = 2;
				}
				else if ( chr == '\'' || chr == '`'  || chr == '"' )
				{
					r.x += 3;
					r.w = 2;
					r.h = 2;
				}


				if ( chr == 'o' || chr == 'O' || chr == '0' )
					DrawLinedRect( r );	
				else
					DrawFilledRect( r );
			}
		}

		Gwen::Point Base::MeasureText( Gwen::Font* pFont, const Gwen::UnicodeString& text )
		{
			Gwen::Point p;
			p.x = pFont->size * Scale() * (float)text.length() * 0.4;
			p.y = pFont->size * Scale();

			return p;
		}
	}
}