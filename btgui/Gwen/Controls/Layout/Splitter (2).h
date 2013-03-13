/*
	GWEN
	Copyright (c) 2011 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#pragma once
#include "Gwen/Controls/Base.h"

namespace Gwen 
{
	namespace Controls
	{
		namespace Layout
		{

			class Splitter : public Base
			{
				public:

					typedef Base BaseClass;

					Splitter( Base* pParent ) : BaseClass( pParent )
					{
						for ( int i=0; i<2; i++ )
							m_pPanel[i] = NULL;
					}

					void SetPanel( int i, Base* pPanel )
					{
						if ( i < 0 || i > 1 ) return;

						m_pPanel[i] = pPanel;

						if ( m_pPanel[i]  )
						{
							m_pPanel[i] ->SetParent( this );
						}
					}

					Base* GetPanel( int i ) const
					{
						if ( i < 0 || i > 1 ) return NULL;
						return m_pPanel[i];
					}

					void Layout( Skin::Base* skin )
					{
						LayoutVertical( skin );
					}

				private:

					void LayoutVertical( Skin::Base* skin )
					{
						int w = Width();
						int h = Height();

						if ( m_pPanel[0] )
						{
							const Margin& m = m_pPanel[0]->GetMargin();
							m_pPanel[0]->SetBounds( m.left, m.top, w-m.left-m.right, (h * 0.5) - m.top - m.bottom );
						}

						if ( m_pPanel[1] )
						{
							const Margin& m = m_pPanel[1]->GetMargin();
							m_pPanel[1]->SetBounds( m.left, m.top + (h * 0.5f), w-m.left-m.right, (h * 0.5f) - m.top - m.bottom );
						}
					}

					void LayoutHorizontal( Skin::Base* skin )
					{
						// Todo.
					}

					Base*	m_pPanel[2];

			};
		}
	}
}
