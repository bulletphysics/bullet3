/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Gwen.h"
#include "Gwen/Controls/Text.h"
#include "Gwen/Skin.h"
#include "Gwen/Utility.h"

using namespace Gwen;
using namespace Gwen::ControlsInternal;

GWEN_CONTROL_CONSTRUCTOR( Text )
{
	m_Font = NULL;
	m_Color = Gwen::Colors::Black; // TODO: From skin somehow.. 
	SetMouseInputEnabled( false );
}

Text::~Text()
{
	// NOTE: This font doesn't need to be released
	// Because it's a pointer to another font somewhere.
}

void Text::RefreshSize()
{
	if ( !GetFont() )
	{
		Debug::AssertCheck( 0, "Text::RefreshSize() - No Font!!\n" );
		return;
	}

	Gwen::Point p( 1, GetFont()->size );
	
	if ( Length() > 0 )
	{
		p = GetSkin()->GetRender()->MeasureText( GetFont(), m_String );
	}

	if ( p.x == Width() && p.y == Height() ) 
		return;

	SetSize( p.x, p.y );
	InvalidateParent();
	Invalidate();
}



Gwen::Font* Text::GetFont()
{
	return m_Font;
}


void Text::SetString( const UnicodeString& str ){ m_String = str; Invalidate(); }
void Text::SetString( const String& str ){ SetString( Gwen::Utility::StringToUnicode( str ) ); }

void Text::Render( Skin::Base* skin )
{
	if ( Length() == 0 || !GetFont() ) return;

	skin->GetRender()->SetDrawColor( m_Color );
	skin->GetRender()->RenderText( GetFont(), Gwen::Point( 0, 0 ), m_String );
}

void Text::Layout( Skin::Base* /*skin*/ )
{
	RefreshSize();
}

Gwen::Point Text::GetCharacterPosition( int iChar )
{
	if ( Length() == 0 || iChar == 0 )
	{
		return Gwen::Point( 1, 0 );
	}

	UnicodeString sub = m_String.substr( 0, iChar );
	Gwen::Point p = GetSkin()->GetRender()->MeasureText( GetFont(), sub );
	
	if ( p.y >= m_Font->size )
		p.y -= m_Font->size;

	return p;
}

int Text::GetClosestCharacter( Gwen::Point p )
{
	int iDistance = 4096;
	int iChar = 0;

	for ( size_t i=0; i<m_String.length()+1; i++ )
	{
		Gwen::Point cp = GetCharacterPosition( i );
		int iDist = abs(cp.x - p.x) + abs(cp.y - p.y); // this isn't proper

		if ( iDist > iDistance ) continue;

		iDistance = iDist;
		iChar = i;
	}

	return iChar;
}

void Text::OnScaleChanged()
{
	Invalidate();
}