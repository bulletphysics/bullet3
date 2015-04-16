/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_CANVAS_H
#define GWEN_CONTROLS_CANVAS_H

#include <set>
#include "Gwen/Controls/Base.h"
#include "Gwen/InputHandler.h"

namespace Gwen 
{
	namespace Controls
	{
		class GWEN_EXPORT Canvas : public Base
		{
			public:

				typedef Controls::Base BaseClass;

				Canvas( Skin::Base* pSkin );

				//
				// For additional initialization 
				// (which is sometimes not appropriate in the constructor)
				//
				virtual void Initialize(){};

				//
				// You should call this to render your canvas.
				//
				virtual void RenderCanvas();

				//
				// Call this whenever you want to process input. This
				// is usually once a frame..
				//
				virtual void DoThink();

				//
				// In most situations you will be rendering the canvas
				// every frame. But in some situations you will only want
				// to render when there have been changes. You can do this
				// by checking NeedsRedraw().
				//
				virtual bool NeedsRedraw(){ return m_bNeedsRedraw; }
				virtual void Redraw(){ m_bNeedsRedraw = true; }

				// Internal. Do not call directly.
				virtual void Render( Skin::Base* pRender );

				// Childpanels call parent->GetCanvas() until they get to 
				// this top level function.
				virtual Controls::Canvas* GetCanvas(){ return this; }

				virtual void SetScale( float f );
				virtual float Scale() const { return m_fScale; }

				virtual void OnBoundsChanged( Gwen::Rect oldBounds );

				//
				// Call this to delete the canvas, and its children
				// in the right order.
				//
				virtual void Release();

				// Delayed deletes
				virtual void AddDelayedDelete( Controls::Base* pControl );
				virtual void ProcessDelayedDeletes();

				Controls::Base*	FirstTab;
				Controls::Base*	NextTab;

				// Input
				virtual bool InputMouseMoved( int x, int y, int deltaX, int deltaY );
				virtual bool InputMouseButton( int iButton, bool bDown );
				virtual bool InputKey( int iKey, bool bDown );
				virtual bool InputCharacter( Gwen::UnicodeChar chr );
				virtual bool InputMouseWheel( int val );

				// Background
				virtual void SetBackgroundColor( const Gwen::Color& color ){ m_BackgroundColor = color; }
				virtual void SetDrawBackground( bool bShouldDraw ){ m_bDrawBackground = bShouldDraw; }

			private:

				bool	m_bNeedsRedraw;
				bool	m_bAnyDelete;
				float	m_fScale;

				Controls::Base::List	m_DeleteList;
				std::set< Controls::Base* > m_DeleteSet;
				friend class Controls::Base;
				void PreDelete( Controls::Base * );

				bool			m_bDrawBackground;
				Gwen::Color		m_BackgroundColor;


		};
	}
}
#endif
