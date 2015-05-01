
#include "Gwen/Controls/PanelListPanel.h"

using namespace Gwen;
using namespace Controls;

GWEN_CONTROL_CONSTRUCTOR( PanelListPanel )
{
	m_bVertical = false;
	m_bSizeToChildren = true;
	m_iControlSpacing = 5;
	m_iLineSpacing = 5;
	m_bWrapping = true;
}

void PanelListPanel::Render( Gwen::Skin::Base* /*skin*/ )
{
}

Gwen::Point PanelListPanel::GetBiggestChildSize()
{
	int width = 0;
	int height = 0;

	for ( Base::List::iterator it = Children.begin(); it != Children.end(); ++it )
	{
		Controls::Base* pChild = *it;
		if ( pChild->Width() > width )
			width = pChild->Width();

		if ( pChild->Height() > height )
			height = pChild->Height();
	}

	return Gwen::Point( width, height );
}

void PanelListPanel::DoVerticalLayout()
{
	int panelWidth = 0;
	int panelX = GetPadding().left;
	int panelY = GetPadding().top;
	int lastPanelY = panelY;
	int testWrap = 0;

	Gwen::Point childSize = GetBiggestChildSize();
	//Lay my children out accordingly
	for ( Base::List::iterator it = Children.begin(); it != Children.end(); ++it )
	{
		Controls::Base* pChild = *it;
		testWrap = lastPanelY + m_iControlSpacing + childSize.y;
		if ( m_bWrapping && testWrap > Height() - GetPadding().bottom )
		{
			panelY = GetPadding().top;
			panelX = GetPadding().left + panelWidth + m_iLineSpacing;
			lastPanelY = panelY + m_iControlSpacing + childSize.y;
		}
		else
		{
			panelY = lastPanelY;
			lastPanelY = testWrap;
		}

		pChild->SetPos( panelX, panelY );

		if (pChild->X() + childSize.x > panelWidth )
			panelWidth = pChild->X() + childSize.x;
	}

	if ( m_bSizeToChildren )
	{
		Gwen::Point childrenSizeTotal = ChildrenSize();
		SetSize( childrenSizeTotal.x, Height());
	}
}

void PanelListPanel::DoHorizontalLayout()
{
	int panelHeight = 0;
	int panelX = GetPadding().left;
	int panelY = GetPadding().top;
	int lastPanelX = panelX;
	int testWrap = 0;

	Gwen::Point childSize = GetBiggestChildSize();

	for ( Base::List::iterator it = Children.begin(); it != Children.end(); ++it )
	{
		Controls::Base* pChild = *it;

			testWrap = lastPanelX + m_iControlSpacing + childSize.x;
			if ( m_bWrapping && testWrap > Width() - GetPadding().right )
			{
				panelX = GetPadding().left;
				panelY = GetPadding().top + panelHeight + m_iLineSpacing;
				lastPanelX = panelX + m_iControlSpacing + childSize.x;
			}
			else
			{
				panelX = lastPanelX;
				lastPanelX = testWrap;
			}

			pChild->SetPos( panelX, panelY );

			if (pChild->Y() + childSize.y > panelHeight )
				panelHeight = pChild->Y() + childSize.y;
	}

	if ( m_bSizeToChildren )
	{
		Gwen::Point childrenSizeTotal = ChildrenSize();
		SetSize( Width(), childrenSizeTotal.y);
	}
}

void PanelListPanel::Layout( Skin::Base* skin )
{
	BaseClass::Layout( skin );
	if (	IsHorizontalLayout() )
		DoHorizontalLayout();
	else
		DoVerticalLayout();
}