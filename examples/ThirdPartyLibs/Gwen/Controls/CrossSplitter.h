#pragma once
#ifndef GWEN_CONTROLS_CROSSSPLITTER_H
#define GWEN_CONTROLS_CROSSSPLITTER_H

#include "Gwen/Gwen.h"
#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/SplitterBar.h"

namespace Gwen 
{
	namespace Controls
	{
		class GWEN_EXPORT CrossSplitter : public Controls::Base
		{
			public:

				GWEN_CONTROL( CrossSplitter, Controls::Base );

				void Layout( Skin::Base* skin );

				virtual float CalculateValueVertical();
				virtual float CalculateValueHorizontal();
				virtual void  CalculateValueCenter();
				virtual void UpdateHSplitter();
				virtual void UpdateVSplitter();
				virtual void UpdateCSplitter();
				virtual void OnVerticalMoved( Controls::Base * control );
				virtual void OnHorizontalMoved( Controls::Base * control );
				virtual void OnCenterMoved( Controls::Base * control );

				virtual void SetPanel( int i, Controls::Base* pPanel );
				virtual Controls::Base* GetPanel( int i );

				virtual bool IsZoomed(){ return m_iZoomedSection != -1; }
				virtual void Zoom( int section );
				virtual void UnZoom();
				virtual void ZoomChanged();
				virtual void CenterPanels() { m_fHVal = 0.5f; m_fVVal = 0.5f; Invalidate(); }

				virtual void SetSplittersVisible( bool b ){ m_VSplitter->SetShouldDrawBackground( b ); m_HSplitter->SetShouldDrawBackground( b ); m_CSplitter->SetShouldDrawBackground( b ); }
				virtual void SetSplitterSize( int size ) { m_fBarSize = size; }
		
			private:

				SplitterBar* m_VSplitter;
				SplitterBar* m_HSplitter;
				SplitterBar* m_CSplitter;

				Controls::Base* m_Sections[4];

				float	m_fHVal;
				float	m_fVVal;
				int		m_fBarSize;

				int 		m_iZoomedSection;

				Gwen::Event::Caller	onZoomed;
				Gwen::Event::Caller	onUnZoomed;
				Gwen::Event::Caller	onZoomChange;
		};
	}
}
#endif
