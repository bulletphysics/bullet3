/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Gwen.h"
#include "Gwen/Controls/MenuItem.h"
#include "Gwen/Skin.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR( MenuItem )
{
	m_Menu = NULL;
	m_bOnStrip = false;
	m_SubmenuArrow = NULL;
	SetTabable( false );
	SetCheckable( false );
	SetCheck( false );
}

MenuItem::~MenuItem()
{

}

void MenuItem::Render( Skin::Base* skin )
{
	skin->DrawMenuItem( this, IsMenuOpen(), m_bCheckable ? m_bChecked : false );
}

void MenuItem::Layout( Skin::Base* skin )
{
	BaseClass::Layout( skin );

}

Menu* MenuItem::GetMenu()
{
	if ( !m_Menu )
	{
		m_Menu = new Menu( GetCanvas() );
		m_Menu->SetHidden( true );

		if ( !m_bOnStrip )
		{
			m_SubmenuArrow = new Symbol::Arrow( this );
			m_SubmenuArrow->Dock( Pos::Right );
			m_SubmenuArrow->SetSize( 20, 20 );
		}

		Invalidate();
	}

	return m_Menu;
}

void MenuItem::SetCheck( bool bCheck )
{
	if ( bCheck == m_bChecked)
		return;

	m_bChecked = bCheck;

	onCheckChange.Call( this );

	if ( bCheck )
		onChecked.Call( this );
	else
		onUnChecked.Call( this );
}

void MenuItem::OnPress()
{
	if ( m_Menu )
	{
		ToggleMenu();
	}
	else if ( !m_bOnStrip )
	{
		SetCheck( !GetChecked() );
		onMenuItemSelected.Call( this );
		GetCanvas()->CloseMenus();
	}

	BaseClass::OnPress();
}

void MenuItem::ToggleMenu()
{
	if ( IsMenuOpen() ) CloseMenu();
	else OpenMenu();
}

bool MenuItem::IsMenuOpen()
{
	if ( !m_Menu ) return false;

	return !m_Menu->Hidden();
}

void MenuItem::OpenMenu()
{
	if ( !m_Menu ) return;

	m_Menu->SetHidden( false );
	m_Menu->BringToFront();

	Gwen::Point p = LocalPosToCanvas( Gwen::Point( 0, 0 ) );

	// Strip menus open downwards
	if ( m_bOnStrip )
	{
		m_Menu->SetPos( p.x, p.y + Height() + 1 );
	}
	// Submenus open sidewards
	else
	{
		m_Menu->SetPos( p.x + Width(), p.y);
	}

	// TODO: Option this.
	// TODO: Make sure on screen, open the other side of the 
	// parent if it's better...


}

void MenuItem::CloseMenu()
{
	if ( !m_Menu ) return;
	m_Menu->Close();
	m_Menu->CloseAll();
}
