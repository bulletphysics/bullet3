/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Gwen.h"
#include "Gwen/Controls/MenuStrip.h"
#include "Gwen/Skin.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR( MenuStrip )
{
	SetBounds( 0, 0, 200, 22 );
	Dock( Pos::Top );
	m_InnerPanel->SetPadding( Padding( 5, 2, 2, 2 ) );	
}

void MenuStrip::Render( Skin::Base* skin )
{
	skin->DrawMenuStrip( this );
}

void MenuStrip::Layout( Skin::Base* /*skin*/ )
{
	//TODO: We don't want to do vertical sizing the same as Menu, do nothing for now
}

void MenuStrip::OnAddItem( MenuItem* item )
{
	item->Dock( Pos::Left );
	item->SetPadding( Padding( 5, 0, 5, 0 ) );
	item->SizeToContents();
	item->SetOnStrip( true );
	item->onHoverEnter.Add( this, &Menu::OnHoverItem );
}

bool MenuStrip::ShouldHoverOpenMenu()
{
	return IsMenuOpen();
}