/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_RADIOBOTTONCONTROLLER_H
#define GWEN_CONTROLS_RADIOBOTTONCONTROLLER_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Label.h"
#include "Gwen/Controls/RadioButton.h"


namespace Gwen 
{
	namespace Controls
	{

		class GWEN_EXPORT RadioButtonController : public Base
		{
			public:

				GWEN_CONTROL( RadioButtonController, Base );

				virtual void Render( Skin::Base* /*skin*/ ){};
				virtual void OnRadioClicked( Base* pFromPanel );

				virtual void OnChange();

				virtual LabeledRadioButton* AddOption( const Gwen::String& strText, const Gwen::String& strOptionName = "" );
				virtual LabeledRadioButton* AddOption( const Gwen::UnicodeString& strText, const Gwen::String& strOptionName = "" );

				virtual LabeledRadioButton*	GetSelected(){ return m_Selected; }

				virtual String GetSelectedName(){ return m_Selected->GetName(); }
				virtual UnicodeString GetSelectedLabel(){ return m_Selected->GetLabel()->GetText(); }

				Event::Caller		onSelectionChange;

			private:

				LabeledRadioButton* m_Selected;
		};
	}
}
#endif
