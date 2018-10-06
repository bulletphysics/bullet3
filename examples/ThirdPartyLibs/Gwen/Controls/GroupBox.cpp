/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Controls/GroupBox.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR(GroupBox)
{
	// Set to true, because it's likely that our
	// children will want mouse input, and they
	// can't get it without us..
	SetMouseInputEnabled(true);

	SetTextPadding(Padding(10, 0, 0, 0));

	SetAlignment(Pos::Top | Pos::Left);
	Invalidate();

	m_InnerPanel = new Base(this);
	m_InnerPanel->Dock(Pos::Fill);
}

void GroupBox::Layout(Skin::Base* skin)
{
	m_InnerPanel->SetMargin(Margin(TextHeight() + 3, 6, 6, 6));

	BaseClass::Layout(skin);
}

void GroupBox::Render(Skin::Base* skin)
{
	skin->DrawGroupBox(this, TextX(), TextHeight(), TextWidth());
}
