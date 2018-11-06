/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Gwen.h"
#include "Gwen/Controls/Label.h"
#include "Gwen/Utility.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR(Label)
{
	m_Text = new ControlsInternal::Text(this);
	m_Text->SetFont(GetSkin()->GetDefaultFont());

	SetMouseInputEnabled(false);
	SetBounds(0, 0, 100, 10);
	SetAlignment(Gwen::Pos::Left | Gwen::Pos::Top);
}

void Label::Layout(Skin::Base* /*skin*/)
{
	int iAlign = m_iAlign;

	int x = m_rTextPadding.left + m_Padding.left;
	int y = m_rTextPadding.top + m_Padding.top;

	if (iAlign & Pos::Right) x = Width() - m_Text->Width() - m_rTextPadding.right - m_Padding.right;
	if (iAlign & Pos::CenterH) x = (m_rTextPadding.left + m_Padding.left) + ((Width() - m_Text->Width()) * 0.5f) - m_rTextPadding.right - m_Padding.right;

	if (iAlign & Pos::CenterV) y = (m_rTextPadding.top + m_Padding.top) + ((Height() - m_Text->Height()) * 0.5f) - m_rTextPadding.bottom - m_Padding.bottom;
	if (iAlign & Pos::Bottom) y = Height() - m_Text->Height() - m_rTextPadding.bottom - m_Padding.bottom;

	m_Text->SetPos(x, y);
}

void Label::SetText(const UnicodeString& str, bool bDoEvents)
{
	if (m_Text->GetText() == str) return;

	m_Text->SetString(str);
	Redraw();

	if (bDoEvents)
		OnTextChanged();
}

void Label::SetText(const String& str, bool bDoEvents)
{
	SetText(Gwen::Utility::StringToUnicode(str), bDoEvents);
}

void Label::SizeToContents()
{
	m_Text->SetPos(m_rTextPadding.left + m_Padding.left, m_rTextPadding.top + m_Padding.top);
	m_Text->RefreshSize();

	SetSize(m_Text->Width() + m_Padding.left + m_Padding.right + m_rTextPadding.left + m_rTextPadding.right, m_Text->Height() + m_Padding.top + m_Padding.bottom + m_rTextPadding.top + m_rTextPadding.bottom);
}

Gwen::Point Label::GetCharacterPosition(int iChar)
{
	Gwen::Point p = m_Text->GetCharacterPosition(iChar);
	p.x += m_Text->X();
	p.y += m_Text->Y();

	return p;
}