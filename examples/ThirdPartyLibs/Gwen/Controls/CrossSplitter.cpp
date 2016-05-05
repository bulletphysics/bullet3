#include "Gwen/Gwen.h"
#include "Gwen/Controls/CrossSplitter.h"
#include "Gwen/Controls/Button.h"

using namespace Gwen;
using namespace Controls;

GWEN_CONTROL_CONSTRUCTOR( CrossSplitter )
{
	m_VSplitter = new SplitterBar( this );
	m_VSplitter->SetPos( 0, 128 );
	m_VSplitter->onDragged.Add( this, &CrossSplitter::OnVerticalMoved );
	m_VSplitter->SetCursor( Gwen::CursorType::SizeNS );

	m_HSplitter = new SplitterBar( this );
	m_HSplitter->SetPos( 128, 0 );
	m_HSplitter->onDragged.Add( this, &CrossSplitter::OnHorizontalMoved );
	m_HSplitter->SetCursor( Gwen::CursorType::SizeWE );

	m_CSplitter = new SplitterBar( this );
	m_CSplitter->SetPos( 128, 128 );
	m_CSplitter->onDragged.Add( this, &CrossSplitter::OnCenterMoved );
	m_CSplitter->SetCursor( Gwen::CursorType::SizeAll );

	m_fHVal = 0.5f;
	m_fVVal = 0.5f;

	SetPanel( 0, NULL );
	SetPanel( 1, NULL );
	SetPanel( 2, NULL );
	SetPanel( 3, NULL );

	SetSplitterSize( 5 );
	SetSplittersVisible( false );

	m_iZoomedSection = -1;
}

void CrossSplitter::UpdateVSplitter()
{
	m_VSplitter->MoveTo( m_VSplitter->X(), ( Height() - m_VSplitter->Height() ) * ( m_fVVal ));
}
void CrossSplitter::UpdateHSplitter()
{
	m_HSplitter->MoveTo( ( Width() - m_HSplitter->Width() ) * ( m_fHVal ), m_HSplitter->Y() );
}

void CrossSplitter::OnCenterMoved( Controls::Base * /*control*/ )
{ 
	//Move the other two bars into position
	CalculateValueCenter();
	Invalidate();
}

void CrossSplitter::UpdateCSplitter()
{
	m_CSplitter->MoveTo( ( Width() - m_CSplitter->Width() ) * ( m_fHVal ), ( Height() - m_CSplitter->Height() ) * ( m_fVVal ));
}

void CrossSplitter::OnHorizontalMoved( Controls::Base * /*control*/ )
{ 
	m_fHVal = CalculateValueHorizontal();
	Invalidate();
}
void CrossSplitter::OnVerticalMoved( Controls::Base * /*control*/ )
{
	m_fVVal = CalculateValueVertical();	
	Invalidate();
}

void CrossSplitter::CalculateValueCenter()
{
	m_fHVal = (float)m_CSplitter->X() / (float)( Width() - m_CSplitter->Width() );
	m_fVVal = (float)m_CSplitter->Y() / (float)( Height() - m_CSplitter->Height() );
}

float CrossSplitter::CalculateValueHorizontal()
{
	return  (float)m_HSplitter->X() / (float)( Width() - m_HSplitter->Width() );
}

float CrossSplitter::CalculateValueVertical()
{
	return  (float)m_VSplitter->Y() / (float)( Height() - m_VSplitter->Height() );
}

void CrossSplitter::Layout( Skin::Base* /*skin*/ )
{
	m_VSplitter->SetSize( Width(), m_fBarSize );
	m_HSplitter->SetSize( m_fBarSize, Height() );
	m_CSplitter->SetSize( m_fBarSize, m_fBarSize );

	UpdateVSplitter();
	UpdateHSplitter();
	UpdateCSplitter();

	if (	m_iZoomedSection == -1 )
	{
		if ( m_Sections[0] )
			m_Sections[0]->SetBounds( 0, 0, m_HSplitter->X(), m_VSplitter->Y() );

		if ( m_Sections[1] )
			m_Sections[1]->SetBounds( m_HSplitter->X() + m_fBarSize, 0, Width() - ( m_HSplitter->X() + m_fBarSize ), m_VSplitter->Y() );

		if ( m_Sections[2] )
			m_Sections[2]->SetBounds( 0, m_VSplitter->Y() + m_fBarSize, m_HSplitter->X(), Height() - ( m_VSplitter->Y() + m_fBarSize ) );

		if ( m_Sections[3] )
			m_Sections[3]->SetBounds( m_HSplitter->X() + m_fBarSize, m_VSplitter->Y() + m_fBarSize, Width() - ( m_HSplitter->X() + m_fBarSize ), Height() - ( m_VSplitter->Y() + m_fBarSize ) );
	}
	else
	{
		//This should probably use Fill docking instead
		m_Sections[(int)m_iZoomedSection]->SetBounds( 0, 0, Width(), Height() );
	}
}

void CrossSplitter::SetPanel( int index, Controls::Base* pPanel)
{
	Debug::AssertCheck( index >= 0 && index <= 3, "CrossSplitter::SetPanel out of range" );

	m_Sections[index] = pPanel;

	if ( pPanel )
	{
		pPanel->Dock( Pos::None );
		pPanel->SetParent( this );
	}

	Invalidate();
}

Controls::Base* CrossSplitter::GetPanel( int i )
{
	return m_Sections[i];
}


void CrossSplitter::ZoomChanged()
{
	onZoomChange.Call( this );
	if ( m_iZoomedSection == -1 )
	{
		onUnZoomed.Call( this );
	}
	else
	{
		onZoomed.Call( this );
	}
}

void CrossSplitter::Zoom( int section )
{
	UnZoom();

	if ( m_Sections[section] )
	{
		for (int i = 0; i < 4; i++)
		{
			if ( i != section && m_Sections[i] )
				m_Sections[i]->SetHidden( true );
		}
		m_iZoomedSection = section;

		Invalidate();
	}
	ZoomChanged();
}

void CrossSplitter::UnZoom()
{
	m_iZoomedSection = -1;

	for ( int i = 0; i < 4; i++ )
	{
		if ( m_Sections[i] )
			m_Sections[i]->SetHidden( false );
	}

	Invalidate();
	ZoomChanged();
}
