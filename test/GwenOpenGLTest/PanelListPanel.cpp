#include "UnitTest.h"
#include "Gwen/Controls/PanelListPanel.h"
#include "Gwen/Controls/StatusBar.h"
#include "Gwen/Utility.h"

using namespace Gwen;

class PanelListPanel : public GUnit
{
	public:

	GWEN_CONTROL_INLINE( PanelListPanel, GUnit )
	{
		m_PLP = new Gwen::Controls::PanelListPanel( this );
		m_PLP->Dock( Pos::Fill );
		m_PLP->SetPadding( Gwen::Padding( 10, 10 ));
		m_PLP->SetVertical();
		m_PLP->SetSizeToChildren( false );

		for ( int i = 0; i < 16; i++)
		{
			Gwen::String testName = "TEST" +  Utility::ToString( i );
			Gwen::Controls::Button* testButton =  new Gwen::Controls::Button( m_PLP );
			testButton->SetText( testName );
		}

		Gwen::Controls::StatusBar* pStatus = new Gwen::Controls::StatusBar( this );
		pStatus->Dock( Pos::Bottom );

		{
			Gwen::Controls::Button* pButton = new Gwen::Controls::Button( pStatus );
			pButton->SetText( "Horizontal" );
			pButton->onPress.Add( this, &PanelListPanel::GoHorizontal );
			pStatus->AddControl( pButton, false );
		}

		{
			Gwen::Controls::Button* pButton = new Gwen::Controls::Button( pStatus );
			pButton->SetText( "Vertical" );
			pButton->onPress.Add( this, &PanelListPanel::GoVertical );
			pStatus->AddControl( pButton, true );
		}
	}


	void GoVertical( Gwen::Controls::Base* pFromPanel )
	{
		m_PLP->SetVertical();
	}

	void GoHorizontal( Gwen::Controls::Base* pFromPanel )
	{
		m_PLP->SetHorizontal();
	}

	Gwen::Controls::PanelListPanel* m_PLP;
};



DEFINE_UNIT_TEST( PanelListPanel, L"PanelListPanel" );