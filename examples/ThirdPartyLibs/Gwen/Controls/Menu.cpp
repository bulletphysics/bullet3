/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Gwen.h"
#include "Gwen/Controls/Menu.h"
#include "Gwen/Skin.h"
#include "Gwen/Utility.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR(Menu)
{
	SetBounds(0, 0, 10, 10);
	SetPadding(Padding(2, 2, 2, 2));

	SetDisableIconMargin(false);

	SetAutoHideBars(true);
	SetScroll(false, true);
}

void Menu::Render(Skin::Base* skin)
{
	skin->DrawMenu(this, IconMarginDisabled());
}

void Menu::RenderUnder(Skin::Base* skin)
{
	BaseClass::RenderUnder(skin);
	skin->DrawShadow(this);
}

void Menu::Layout(Skin::Base* skin)
{
	int childrenHeight = 0;
	for (Base::List::iterator it = m_InnerPanel->Children.begin(); it != m_InnerPanel->Children.end(); ++it)
	{
		Base* pChild = (*it);
		if (!pChild)
			continue;

		childrenHeight += pChild->Height();
	}

	if (Y() + childrenHeight > GetCanvas()->Height())
		childrenHeight = GetCanvas()->Height() - Y();

	SetSize(Width(), childrenHeight);

	BaseClass::Layout(skin);
}

MenuItem* Menu::AddItem(const Gwen::UnicodeString& strName, const UnicodeString& strIconName, Gwen::Event::Handler* pHandler, Gwen::Event::Handler::Function fn)
{
	MenuItem* pItem = new MenuItem(this);
	pItem->SetText(strName);
	pItem->SetImage(strIconName);

	if (fn && pHandler)
	{
		pItem->onMenuItemSelected.Add(pHandler, fn);
	}

	OnAddItem(pItem);

	return pItem;
}

void Menu::OnAddItem(MenuItem* item)
{
	item->Dock(Pos::Top);
	item->SetTextPadding(Padding(IconMarginDisabled() ? 0 : 24, 0, 16, 0));
	item->SetPadding(Padding(4, 4, 4, 4));
	item->SizeToContents();
	item->SetAlignment(Pos::CenterV | Pos::Left);
	item->onHoverEnter.Add(this, &Menu::OnHoverItem);

	// Do this here - after Top Docking these values mean nothing in layout
	int w = item->Width() + 10 + 32;
	if (w < Width()) w = Width();
	SetSize(w, Height());
}

void Menu::ClearItems()
{
	for (Base::List::iterator it = m_InnerPanel->Children.begin(); it != m_InnerPanel->Children.end(); ++it)
	{
		Base* pChild = *it;

		if (!pChild) continue;
		pChild->DelayedDelete();
	}
}

MenuItem* Menu::AddItem(const Gwen::String& strName, const String& strIconName, Gwen::Event::Handler* pHandler, Gwen::Event::Handler::Function fn)
{
	return AddItem(Gwen::Utility::StringToUnicode(strName), Gwen::Utility::StringToUnicode(strIconName), pHandler, fn);
}

void Menu::CloseAll()
{
	for (Base::List::iterator it = m_InnerPanel->Children.begin(); it != m_InnerPanel->Children.end(); ++it)
	{
		Base* pChild = *it;
		MenuItem* pItem = pChild->DynamicCastMenuItem();
		if (!pItem) continue;

		pItem->CloseMenu();
	}
}

bool Menu::IsMenuOpen()
{
	for (Base::List::iterator it = m_InnerPanel->Children.begin(); it != m_InnerPanel->Children.end(); ++it)
	{
		Base* pChild = *it;
		MenuItem* pItem = pChild->DynamicCastMenuItem();
		if (!pItem) continue;

		if (pItem->IsMenuOpen())
			return true;
	}

	return false;
}

void Menu::OnHoverItem(Gwen::Controls::Base* pControl)
{
	if (!ShouldHoverOpenMenu()) return;

	MenuItem* pItem = pControl->DynamicCastMenuItem();
	if (!pItem) return;
	if (pItem->IsMenuOpen()) return;

	CloseAll();
	pItem->OpenMenu();
}

void Menu::Close()
{
	SetHidden(true);
}

void Menu::CloseMenus()
{
	BaseClass::CloseMenus();

	CloseAll();
	Close();
}

void Menu::AddDivider()
{
	MenuDivider* divider = new MenuDivider(this);
	divider->Dock(Pos::Top);
	divider->SetMargin(Margin(IconMarginDisabled() ? 0 : 24, 0, 4, 0));
}

void MenuDivider::Render(Gwen::Skin::Base* skin)
{
	skin->DrawMenuDivider(this);
}