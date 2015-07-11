#include "UnitTest.h"
#include "Gwen/Controls/CrossSplitter.h"
#include "Gwen/Controls/StatusBar.h"
#include "Gwen/Controls/Button.h"

using namespace Gwen;

class CrossSplitter : public GUnit
{
	public:

	GWEN_CONTROL_INLINE( CrossSplitter, GUnit )
	{

		m_bSplittersVisible = false;
		m_iCurZoom = 0;

		m_Splitter = new Gwen::Controls::CrossSplitter( this );
		m_Splitter->SetPos(0, 0);
		m_Splitter->Dock( Pos::Fill );

		{
			Gwen::Controls::Button* testButton =  new Gwen::Controls::Button( m_Splitter );
			testButton->SetText( "TOPLEFT");
			m_Splitter->SetPanel( 0, testButton );
		}

		{
			Gwen::Controls::Button* testButton =  new Gwen::Controls::Button( m_Splitter );
			testButton->SetText( "TOPRIGHT");
			m_Splitter->SetPanel( 1, testButton );
		}

		{
			Gwen::Controls::Button* testButton =  new Gwen::Controls::Button( m_Splitter );
			testButton->SetText( "BOTTOMRIGHT");
			m_Splitter->SetPanel( 2, testButton );
		}

		{
			Gwen::Controls::Button* testButton =  new Gwen::Controls::Button( m_Splitter );
			testButton->SetText( "BOTTOMLEFT");
			m_Splitter->SetPanel( 3, testButton );
		}
	

		//Status bar to hold unit testing buttons
		Gwen::Controls::StatusBar* pStatus = new Gwen::Controls::StatusBar( this );
		pStatus->Dock( Pos::Bottom );


		{
			Gwen::Controls::Button* pButton = new Gwen::Controls::Button( pStatus );
			pButton->SetText( "Zoom" );
			pButton->onPress.Add( this, &CrossSplitter::ZoomTest );
			pStatus->AddControl( pButton, false );
		}

		{
			Gwen::Controls::Button* pButton = new Gwen::Controls::Button( pStatus );
			pButton->SetText( "UnZoom" );
			pButton->onPress.Add( this, &CrossSplitter::UnZoomTest );
			pStatus->AddControl( pButton, false );
		}

		{
			Gwen::Controls::Button* pButton = new Gwen::Controls::Button( pStatus );
			pButton->SetText( "CenterPanels" );
			pButton->onPress.Add( this, &CrossSplitter::CenterPanels );
			pStatus->AddControl( pButton, true );
		}

		{
			Gwen::Controls::Button* pButton = new Gwen::Controls::Button( pStatus );
			pButton->SetText( "Splitters" );
			pButton->onPress.Add( this, &CrossSplitter::ToggleSplitters );
			pStatus->AddControl( pButton, true );
		}
	}

	void ZoomTest( Gwen::Controls::Base* pFromPanel )
	{
		m_Splitter->Zoom(m_iCurZoom);
		m_iCurZoom++;
		if (m_iCurZoom == 4)
			m_iCurZoom = 0;
	}

	void UnZoomTest( Gwen::Controls::Base* pFromPanel )
	{
		m_Splitter->UnZoom();
	}

	void CenterPanels( Gwen::Controls::Base* pFromPanel )
	{
		m_Splitter->CenterPanels();
		m_Splitter->UnZoom();
	}

	void ToggleSplitters( Gwen::Controls::Base* pFromPanel )
	{
		m_Splitter->SetSplittersVisible( !m_bSplittersVisible );
		m_bSplittersVisible = !m_bSplittersVisible;
	}


	bool m_bSplittersVisible;
	int	m_iCurZoom;
	Controls::CrossSplitter* m_Splitter;
	
};



DEFINE_UNIT_TEST( CrossSplitter, L"CrossSplitter" );