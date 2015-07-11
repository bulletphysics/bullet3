/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Controls/HSVColorPicker.h"
#include "Gwen/Controls/ColorControls.h"
#include "Gwen/Controls/ColorPicker.h"
#include "Gwen/Controls/TextBox.h"
#include "Gwen/Controls/Label.h"
#include "Gwen/Controls/PanelListPanel.h"
#include "Gwen/Utility.h"

using namespace Gwen;
using namespace Gwen::Controls;


GWEN_CONTROL_CONSTRUCTOR( HSVColorPicker )
{
	SetMouseInputEnabled( true );
	SetSize( 256, 128 );
	SetCacheToTexture();

	m_LerpBox = new Gwen::Controls::ColorLerpBox( this );
	m_LerpBox->onSelectionChanged.Add( this, &HSVColorPicker::ColorBoxChanged );
	m_LerpBox->SetPos( 5, 5 );

	m_ColorSlider = new Gwen::Controls::ColorSlider( this );
	m_ColorSlider->SetPos( m_LerpBox->Width() + 15, 5 );
	m_ColorSlider->onSelectionChanged.Add( this, &HSVColorPicker::ColorSliderChanged );

	m_After = new Gwen::ControlsInternal::ColorDisplay( this );
	m_After->SetSize( 48, 24 );
	m_After->SetPos( m_ColorSlider->X() + m_ColorSlider->Width() + 15, 5 );

	m_Before = new Gwen::ControlsInternal::ColorDisplay( this );
	m_Before->SetSize( 48, 24 );
	m_Before->SetPos( m_After->X(), 28 );

	int x = m_Before->X();
	int y = m_Before->Y() + 30;


	{
		Label* label = new Label( this );
		label->SetText(L"R:");
		label->SizeToContents();
		label->SetPos( x, y );

		TextBoxNumeric* numeric = new TextBoxNumeric( this );
		numeric->SetName( "RedBox" );
		numeric->SetPos( x + 15, y -1  );
		numeric->SetSize( 26, 16 );
		numeric->SetSelectAllOnFocus( true );
		numeric->onTextChanged.Add( this, &HSVColorPicker::NumericTyped );

	}

	y+= 20;

	{
		Label* label = new Label( this );
		label->SetText(L"G:");
		label->SizeToContents();
		label->SetPos( x, y );

		
		TextBoxNumeric* numeric = new TextBoxNumeric( this );
		numeric->SetName( "GreenBox" );
		numeric->SetPos( x + 15, y -1  );
		numeric->SetSize( 26, 16 );
		numeric->SetSelectAllOnFocus( true );
		numeric->onTextChanged.Add( this, &HSVColorPicker::NumericTyped );
	}

	y+= 20;

	{
		Label* label = new Label( this );
		label->SetText(L"B:");
		label->SizeToContents();
		label->SetPos( x, y );


		TextBoxNumeric* numeric = new TextBoxNumeric( this );
		numeric->SetName( "BlueBox" );
		numeric->SetPos( x + 15, y -1  );
		numeric->SetSize( 26, 16 );
		numeric->SetSelectAllOnFocus( true );
		numeric->onTextChanged.Add( this, &HSVColorPicker::NumericTyped );
	}
}

void HSVColorPicker::NumericTyped( Gwen::Controls::Base* control )
{
	TextBoxNumeric* box = control->DynamicCastTextBoxNumeric();
	if ( !box ) return;

	if ( box->GetText() == L"" )	return;

	int textValue = atoi( Gwen::Utility::UnicodeToString( box->GetText()).c_str()  );
	if ( textValue < 0 ) textValue = 0;
	if ( textValue > 255 ) textValue = 255;

	Gwen::Color newColor = GetColor();

	if ( box->GetName().find( "Red" ) != Gwen::String::npos )
	{
		newColor.r = textValue;
	}
	else if ( box->GetName().find( "Green" ) != Gwen::String::npos )
	{
		newColor.g = textValue;
	}
	else if ( box->GetName().find( "Blue" ) != Gwen::String::npos )
	{
		newColor.b = textValue;
	}
	else if ( box->GetName().find( "Alpha" ) != Gwen::String::npos )
	{
		newColor.a = textValue;
	}

	SetColor( newColor );
}

void HSVColorPicker::UpdateControls(Gwen::Color color)
{
	TextBoxNumeric* redBox = FindChildByName( "RedBox",   false )->DynamicCastTextBoxNumeric();
	if ( redBox )    redBox->SetText( Gwen::Utility::ToString( (int)color.r), false );

	TextBoxNumeric* greenBox = FindChildByName( "GreenBox",   false )->DynamicCastTextBoxNumeric();
	if ( greenBox )  greenBox->SetText( Gwen::Utility::ToString( (int)color.g ), false );

	TextBoxNumeric* blueBox = FindChildByName( "BlueBox",   false )->DynamicCastTextBoxNumeric();
	if ( blueBox )   blueBox->SetText( Gwen::Utility::ToString( (int)color.b ), false );

	m_After->SetColor( color );
}
void HSVColorPicker::SetColor( Gwen::Color color, bool onlyHue, bool reset )
{

	UpdateControls( color );


	if ( reset )
		m_Before->SetColor( color );

	m_ColorSlider->SetColor( color );
	m_LerpBox->SetColor( color,  onlyHue );
	m_After->SetColor( color );
}

Gwen::Color HSVColorPicker::GetColor()
{
	return m_LerpBox->GetSelectedColor();
}

void HSVColorPicker::ColorBoxChanged( Gwen::Controls::Base* /*pControl*/ )
{
	onColorChanged.Call( this );
	UpdateControls( GetColor() );
	Invalidate();
}
void HSVColorPicker::ColorSliderChanged( Gwen::Controls::Base* /*pControl*/ )
{
	if ( m_LerpBox )
		m_LerpBox->SetColor( m_ColorSlider->GetSelectedColor(),  true );
	Invalidate();
}