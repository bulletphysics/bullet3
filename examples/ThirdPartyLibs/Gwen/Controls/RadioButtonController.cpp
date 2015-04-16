/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Controls/RadioButtonController.h"
#include "Gwen/Controls/RadioButton.h"
#include "Gwen/Utility.h"

using namespace Gwen;
using namespace Gwen::Controls;


GWEN_CONTROL_CONSTRUCTOR( RadioButtonController )
{
	m_Selected = NULL;
	SetTabable( false );
	SetKeyboardInputEnabled( false );
}

void RadioButtonController::OnRadioClicked( Gwen::Controls::Base* pFromPanel )
{
	RadioButton* pCheckedRadioButton = pFromPanel->DynamicCastRadioButton();

	//Iterate through all other buttons and set them to false;
	for (Base::List::iterator iter = Children.begin(); iter != Children.end(); ++iter)
	{
		Base* pChild = *iter;
		LabeledRadioButton* pLRB = pChild->DynamicCastLabeledRadioButton();
		if ( pLRB )
		{
			RadioButton* pChildRadioButton = pLRB->GetRadioButton();
			if ( pChildRadioButton == pCheckedRadioButton )
			{
				m_Selected = pLRB;
			}
			else
			{
				pLRB->GetRadioButton()->SetChecked( false );
			}
		}
	}

	OnChange();
}

void RadioButtonController::OnChange()
{
	onSelectionChange.Call( this );
}

LabeledRadioButton* RadioButtonController::AddOption( const Gwen::String& strText, const Gwen::String& strOptionName )
{
	return AddOption( Gwen::Utility::StringToUnicode( strText ), strOptionName );
}

LabeledRadioButton* RadioButtonController::AddOption( const Gwen::UnicodeString& strText, const Gwen::String& strOptionName )
{
	LabeledRadioButton* lrb = new LabeledRadioButton( this );

	lrb->SetName( strOptionName );
	lrb->GetLabel()->SetText( strText );
	lrb->GetRadioButton()->onChecked.Add( this, &RadioButtonController::OnRadioClicked );
	lrb->Dock( Pos::Top );
	lrb->SetMargin( Margin( 0, 1, 0, 1 ) );
	lrb->SetKeyboardInputEnabled( false );
	lrb->SetTabable( false );

	Invalidate();

	return lrb;
}