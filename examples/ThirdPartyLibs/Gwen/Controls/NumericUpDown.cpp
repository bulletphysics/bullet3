/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Gwen.h"
#include "Gwen/Utility.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/NumericUpDown.h"
#include "Gwen/Controls/Layout/Splitter.h"

using namespace Gwen;
using namespace Gwen::Controls;



GWEN_CONTROL_CONSTRUCTOR( NumericUpDown )
{
	SetSize( 100, 20 );

	Layout::Splitter* pSplitter = new Layout::Splitter( this );
		pSplitter->Dock( Pos::Right );
		pSplitter->SetSize( 13, 13 );

	NumericUpDownButton_Up* pButtonUp = new NumericUpDownButton_Up( pSplitter );
		pButtonUp->onPress.Add( this, &NumericUpDown::OnButtonUp );
		pButtonUp->SetTabable( false );

		pSplitter->SetPanel( 0, pButtonUp );
		

	NumericUpDownButton_Down* pButtonDown = new NumericUpDownButton_Down( pSplitter );
		pButtonDown->onPress.Add( this, &NumericUpDown::OnButtonDown );
		pButtonDown->SetTabable( false );
		pButtonUp->SetPadding( Padding( 0, 1, 1, 0 ) );

		pSplitter->SetPanel( 1, pButtonDown );

	m_iMax = 100;
	m_iMin = 0;
	m_iNumber = 0;
	SetText( "0" );
}

void NumericUpDown::OnButtonUp( Base* /*control*/ )
{
	SyncNumberFromText();
	SetValue( m_iNumber + 1 );
}

void NumericUpDown::OnButtonDown( Base* /*control*/ )
{
	SyncNumberFromText();
	SetValue( m_iNumber - 1 );
}


void NumericUpDown::SyncTextFromNumber()
{
	SetText( Utility::ToString( m_iNumber ) );
}

void NumericUpDown::SyncNumberFromText()
{
	SetValue( (int) GetFloatFromText() );
}

void NumericUpDown::SetMin( int i )
{
	m_iMin = i;
}

void NumericUpDown::SetMax( int i )
{
	m_iMax = i;
}

void NumericUpDown::SetValue( int i )
{
	if ( i > m_iMax ) i = m_iMax;
	if ( i < m_iMin ) i = m_iMin;

	if ( m_iNumber == i )
	{		
		return;
	}

	m_iNumber = i;

	// Don't update the text if we're typing in it..
	if ( !HasFocus() )
	{
		SyncTextFromNumber();
	}

	OnChange();
}

void NumericUpDown::OnChange()
{
	onChanged.Call( this );
}

void NumericUpDown::OnTextChanged()
{
	BaseClass::OnTextChanged();

	SyncNumberFromText();
}

void NumericUpDown::OnEnter()
{
	SyncNumberFromText();
	SyncTextFromNumber();
}