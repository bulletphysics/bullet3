/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Gwen.h"
#include "Gwen/Controls/TextBox.h"
#include "Gwen/Skin.h"
#include "Gwen/Utility.h"
#include "Gwen/Platform.h"

#include <math.h>


using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR( TextBox )
{
	SetSize( 200, 20 );

	SetMouseInputEnabled( true );
	SetKeyboardInputEnabled( true );

	SetAlignment( Pos::Left | Pos::CenterV );
	SetTextPadding( Padding( 4, 2, 4, 2 ) );

	m_iCursorPos = 0;
	m_iCursorEnd = 0;
	m_bSelectAll = false;

	SetTextColor( Gwen::Color( 50, 50, 50, 255 ) ); // TODO: From Skin

	SetTabable( true );

	AddAccelerator( L"Ctrl + c", &TextBox::OnCopy );
	AddAccelerator( L"Ctrl + x", &TextBox::OnCut );
	AddAccelerator( L"Ctrl + v", &TextBox::OnPaste );
	AddAccelerator( L"Ctrl + a", &TextBox::OnSelectAll );
}

bool TextBox::OnChar( Gwen::UnicodeChar c )
{
	if ( c == '\t' ) return false;

	Gwen::UnicodeString str;
	str += c;

	InsertText( str );	
	return true;
}

void TextBox::InsertText( const Gwen::UnicodeString& strInsert )
{
	// TODO: Make sure fits (implement maxlength)

	if ( HasSelection() )
	{
		EraseSelection();
	}

	if ( m_iCursorPos > TextLength() ) m_iCursorPos = TextLength();

	if ( !IsTextAllowed( strInsert, m_iCursorPos )  )
		return;

	UnicodeString str = GetText();
	str.insert( m_iCursorPos, strInsert );
	SetText( str );

	m_iCursorPos += (int) strInsert.size();
	m_iCursorEnd = m_iCursorPos;

	RefreshCursorBounds();
}

void TextBox::Render( Skin::Base* skin )
{
	if ( ShouldDrawBackground() )
		skin->DrawTextBox( this );
	

	if ( !HasFocus() ) return;

	// Draw selection.. if selected..
	if ( m_iCursorPos != m_iCursorEnd )
	{
		skin->GetRender()->SetDrawColor( Gwen::Color( 50, 170, 255, 200 ) );
		skin->GetRender()->DrawFilledRect( m_rectSelectionBounds );	
	}

	// Draw caret
	if ( fmod( Gwen::Platform::GetTimeInSeconds()-m_fLastInputTime, 1.0f ) > 0.5f )
		skin->GetRender()->SetDrawColor( Gwen::Color( 255, 255, 255, 255 ) );
	else
		skin->GetRender()->SetDrawColor( Gwen::Color( 0, 0, 0, 255 ) );

	skin->GetRender()->DrawFilledRect( m_rectCaretBounds );	
}

void TextBox::RefreshCursorBounds()
{
	m_fLastInputTime = Gwen::Platform::GetTimeInSeconds();

	MakeCaratVisible();

	Gwen::Point pA = GetCharacterPosition( m_iCursorPos );
	Gwen::Point pB = GetCharacterPosition( m_iCursorEnd );

	m_rectSelectionBounds.x = Utility::Min( pA.x, pB.x );
	m_rectSelectionBounds.y = m_Text->Y() - 1;
	m_rectSelectionBounds.w = Utility::Max( pA.x, pB.x ) - m_rectSelectionBounds.x;
	m_rectSelectionBounds.h = m_Text->Height() + 2;

	m_rectCaretBounds.x = pA.x;
	m_rectCaretBounds.y = m_Text->Y() - 1;
	m_rectCaretBounds.w = 1;
	m_rectCaretBounds.h = m_Text->Height() + 2;
	
	Redraw();
}


void TextBox::OnPaste( Gwen::Controls::Base* /*pCtrl*/ )
{
	InsertText( Platform::GetClipboardText() );
}

void TextBox::OnCopy( Gwen::Controls::Base* /*pCtrl*/ )
{
	if ( !HasSelection() ) return;

	Platform::SetClipboardText( GetSelection() );
}

void TextBox::OnCut( Gwen::Controls::Base* /*pCtrl*/ )
{
	if ( !HasSelection() ) return;

	Platform::SetClipboardText( GetSelection() );
	EraseSelection();
}

void TextBox::OnSelectAll( Gwen::Controls::Base* /*pCtrl*/ )
{
	m_iCursorEnd = 0;
	m_iCursorPos = TextLength();

	RefreshCursorBounds();
}

void TextBox::OnMouseDoubleClickLeft( int /*x*/, int /*y*/ )
{ 
	OnSelectAll( this );
}

UnicodeString TextBox::GetSelection()
{
	if ( !HasSelection() ) return L"";

	int iStart = Utility::Min( m_iCursorPos, m_iCursorEnd );
	int iEnd = Utility::Max( m_iCursorPos, m_iCursorEnd );

	const UnicodeString& str = GetText();
	return str.substr( iStart, iEnd - iStart );
}

bool TextBox::OnKeyReturn( bool bDown )
{
	if ( bDown ) return true;
	
	OnEnter();

	// Try to move to the next control, as if tab had been pressed
	OnKeyTab( true );

	// If we still have focus, blur it.
	if ( HasFocus() )
	{
		Blur();
	}

	return true;
}

bool TextBox::OnKeyBackspace( bool bDown )
{
	if ( !bDown ) return true;
	if ( HasSelection() )
	{
		EraseSelection();
		return true;
	}

	if ( m_iCursorPos == 0 ) return true;

	DeleteText( m_iCursorPos-1, 1 );

	return true;
}

bool TextBox::OnKeyDelete( bool bDown )
{
	if ( !bDown ) return true;
	if ( HasSelection() )
	{
		EraseSelection();
		return true;
	}

	if ( m_iCursorPos >= TextLength() ) return true;

	DeleteText( m_iCursorPos, 1 );

	return true;
}

bool TextBox::OnKeyLeft( bool bDown )
{
	if ( !bDown ) return true;

	if ( m_iCursorPos > 0 )
		m_iCursorPos--;

	if ( !Gwen::Input::IsShiftDown() )
	{
		m_iCursorEnd = m_iCursorPos;
	}

	RefreshCursorBounds();
	return true;
}

bool TextBox::OnKeyRight( bool bDown )
{
	if ( !bDown ) return true;

	if ( m_iCursorPos < TextLength() )
		m_iCursorPos++;

	if ( !Gwen::Input::IsShiftDown() )
	{
		m_iCursorEnd = m_iCursorPos;
	}

	RefreshCursorBounds();
	return true;
}

bool TextBox::OnKeyHome( bool bDown )
{
	if ( !bDown ) return true;
	m_iCursorPos = 0;

	if ( !Gwen::Input::IsShiftDown() )
	{
		m_iCursorEnd = m_iCursorPos;
	}

	RefreshCursorBounds();
	return true;
}

bool TextBox::OnKeyEnd( bool /*bDown*/ )
{
	m_iCursorPos = TextLength();

	if ( !Gwen::Input::IsShiftDown() )
	{
		m_iCursorEnd = m_iCursorPos;
	}

	RefreshCursorBounds();
	return true;
}

void TextBox::SetCursorPos( int i )
{
	if ( m_iCursorPos == i ) return;

	m_iCursorPos = i;
	RefreshCursorBounds();
}

void TextBox::SetCursorEnd( int i )
{
	if ( m_iCursorEnd == i ) return;

	m_iCursorEnd = i;
	RefreshCursorBounds();
}


void TextBox::DeleteText( int iStartPos, int iLength )
{
	UnicodeString str = GetText();
	str.erase( iStartPos, iLength );
	SetText( str );

	if ( m_iCursorPos > iStartPos )
	{
		SetCursorPos( m_iCursorPos - iLength );
	}

	SetCursorEnd( m_iCursorPos );
}

bool TextBox::HasSelection()
{
	return m_iCursorPos != m_iCursorEnd;
}

void TextBox::EraseSelection()
{
	int iStart = Utility::Min( m_iCursorPos, m_iCursorEnd );
	int iEnd = Utility::Max( m_iCursorPos, m_iCursorEnd );

	DeleteText( iStart, iEnd - iStart );

	// Move the cursor to the start of the selection, 
	// since the end is probably outside of the string now.
	m_iCursorPos = iStart;
	m_iCursorEnd = iStart;
}

void TextBox::OnMouseClickLeft( int x, int y, bool bDown )
{
	if ( m_bSelectAll )
	{
		OnSelectAll( this );
		m_bSelectAll = false;
		return;
	}

	int iChar = m_Text->GetClosestCharacter( m_Text->CanvasPosToLocal( Gwen::Point( x, y ) ) );

	if ( bDown )
	{
		SetCursorPos( iChar );

		if ( !Gwen::Input::IsShiftDown() )
			SetCursorEnd( iChar );

		Gwen::MouseFocus = this;
	}
	else
	{
		if ( Gwen::MouseFocus == this )
		{
			SetCursorPos( iChar );
			Gwen::MouseFocus = NULL;
		}
	}
}

void TextBox::OnMouseMoved( int x, int y, int /*deltaX*/, int /*deltaY*/ )
{
	if ( Gwen::MouseFocus != this ) return;

	int iChar = m_Text->GetClosestCharacter( m_Text->CanvasPosToLocal( Gwen::Point( x, y ) ) );

	SetCursorPos( iChar );
}

void TextBox::MakeCaratVisible()
{
	int iCaratPos = m_Text->GetCharacterPosition( m_iCursorPos ).x;

	// If the carat is already in a semi-good position, leave it.
	{
		int iRealCaratPos = iCaratPos + m_Text->X();
		if ( iRealCaratPos > Width() * 0.1f && iRealCaratPos < Width() * 0.9f )
			return;
	}

	// The ideal position is for the carat to be right in the middle
	int idealx = -iCaratPos + Width() * 0.5f;;

	// Don't show too much whitespace to the right
	if ( idealx + m_Text->Width() < Width() - m_rTextPadding.right )
		idealx = -m_Text->Width() + (Width() - m_rTextPadding.right );

	// Or the left
	if ( idealx > m_rTextPadding.left )
		idealx = m_rTextPadding.left;

	m_Text->SetPos( idealx, m_Text->Y() );

}

void TextBox::Layout( Skin::Base* skin )
{
	BaseClass::Layout( skin );

	RefreshCursorBounds();
}

void TextBox::OnTextChanged()
{
	if ( m_iCursorPos > TextLength() ) m_iCursorPos = TextLength();
	if ( m_iCursorEnd > TextLength() ) m_iCursorEnd = TextLength();

	onTextChanged.Call( this );
}

void TextBox::OnEnter()
{
	onReturnPressed.Call( this );
}