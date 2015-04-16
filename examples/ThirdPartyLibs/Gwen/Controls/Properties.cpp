/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/Properties.h"
#include "Gwen/Utility.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR( Properties )
{
	m_SplitterBar = new SplitterBar( this );
	m_SplitterBar->SetPos( 80, 0 );
	m_SplitterBar->SetCursor( Gwen::CursorType::SizeWE );
	m_SplitterBar->onDragged.Add( this, &Properties::OnSplitterMoved );
	m_SplitterBar->SetShouldDrawBackground( false );
}

void Properties::PostLayout( Gwen::Skin::Base* /*skin*/ )
{
	m_SplitterBar->SetHeight( 0 );

	if ( SizeToChildren( false, true ) )
	{
		InvalidateParent();
	}

	m_SplitterBar->SetSize( 3, Height() );
}

void Properties::OnSplitterMoved( Controls::Base * /*control*/ )
{
	InvalidateChildren();
}

int Properties::GetSplitWidth()
{
	return m_SplitterBar->X();
}

PropertyRow* Properties::Add( const UnicodeString& text, const UnicodeString& value )
{
	Property::Base* pProp = new Property::Text( this );
	pProp->SetPropertyValue( value );

	return Add( text, pProp );
}

PropertyRow* Properties::Add( const String& text, const String& value )
{
	return Add( Gwen::Utility::StringToUnicode( text ), Gwen::Utility::StringToUnicode( value ) );
}

PropertyRow* Properties::Add( const UnicodeString& text, Property::Base* pProp )
{
	PropertyRow* row = new PropertyRow( this );
		row->Dock( Pos::Top );
		row->GetLabel()->SetText( text );
		row->SetProperty( pProp );

	m_SplitterBar->BringToFront();
	return row;
}

PropertyRow* Properties::Add( const String& text, Property::Base* pProp )
{
	return Add( Gwen::Utility::StringToUnicode( text ), pProp );
}

void Properties::Clear()
{
	Base::List ChildListCopy = Children;
	for ( Base::List::iterator it = ChildListCopy.begin(); it != ChildListCopy.end(); ++it )
	{
		PropertyRow* row = (*it)->DynamicCastPropertyRow();
		if ( !row ) continue;

		row->DelayedDelete();
	}
}


GWEN_CONTROL_CONSTRUCTOR( PropertyRow )
{
	m_Property = NULL;

	m_Label = new Label( this );
	m_Label->SetAlignment( Pos::CenterV | Pos::Left );
	m_Label->Dock( Pos::Left );
	m_Label->SetMargin( Margin( 2, 0, 0, 0 ) );

	SetHeight( 16 );	
}	

void PropertyRow::Render( Gwen::Skin::Base* skin )
{
	skin->DrawPropertyRow( this, m_Label->Right(), m_Property->IsEditing() );
}

void PropertyRow::Layout( Gwen::Skin::Base* /*skin*/ )
{
	Properties* pParent = GetParent()->DynamicCastProperties();
	if ( !pParent ) return;

	m_Label->SetWidth( pParent->GetSplitWidth() );
}

void PropertyRow::SetProperty( Property::Base* prop )
{
	m_Property = prop;
	m_Property->SetParent( this );
	m_Property->Dock( Pos::Fill );
	m_Property->onChange.Add( this, &ThisClass::OnPropertyValueChanged );
}

void PropertyRow::OnPropertyValueChanged( Gwen::Controls::Base* /*control*/ )
{
	onChange.Call( this );
}