/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_SLIDER_H
#define GWEN_CONTROLS_SLIDER_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Button.h"
#include "Gwen/Controls/Dragger.h"
#include "Gwen/Gwen.h"
#include "Gwen/Skin.h"

namespace Gwen 
{
	namespace ControlsInternal
	{
		class GWEN_EXPORT SliderBar : public ControlsInternal::Dragger
		{
			GWEN_CONTROL( SliderBar, ControlsInternal::Dragger );

			virtual void Render( Skin::Base* skin );
		};
	}

	namespace Controls
	{

		class GWEN_EXPORT Slider : public Base
		{
				GWEN_CONTROL( Slider, Base );

				virtual void Render( Skin::Base* skin ) = 0;
				virtual void Layout( Skin::Base* skin );

				virtual void SetClampToNotches( bool bClamp ) { m_bClampToNotches = bClamp; }

				virtual void SetNotchCount( int num ) { m_iNumNotches = num; }
				virtual int GetNotchCount() { return m_iNumNotches; }

				virtual void SetRange( float fMin, float fMax );
				virtual float GetRangeMin() const
				{
					return m_fMin;
				}
				virtual float GetRangeMax() const
				{
					return m_fMax;
				}
				virtual float GetValue();
				virtual void SetValue( float val, bool forceUpdate = true );

				virtual float CalculateValue();
				virtual void OnMoved( Controls::Base * control );

				virtual void OnMouseClickLeft( int /*x*/, int /*y*/, bool /*bDown*/ ){};

				virtual bool OnKeyRight( bool bDown )	{	if ( bDown ) SetValue( GetValue() + 1, true ); return true; }
				virtual bool OnKeyLeft( bool bDown )	{	if ( bDown ) SetValue( GetValue() - 1, true ); return true; }
				virtual bool OnKeyUp( bool bDown )		{	if ( bDown ) SetValue( GetValue() + 1, true ); return true; }
				virtual bool OnKeyDown( bool bDown )	{	if ( bDown ) SetValue( GetValue() - 1, true ); return true; }

				virtual void RenderFocus( Gwen::Skin::Base* skin);
				
				Gwen::Event::Caller	onValueChanged;

			protected:

				virtual void SetValueInternal( float fVal );
				virtual void UpdateBarFromValue() = 0;

				ControlsInternal::SliderBar * m_SliderBar;
				bool m_bClampToNotches;
				int m_iNumNotches;
				float m_fValue;

				float m_fMin;
				float m_fMax;
				
		};
	}


}
#endif
