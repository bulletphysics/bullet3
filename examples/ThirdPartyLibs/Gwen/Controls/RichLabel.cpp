/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Gwen.h"
#include "Gwen/Controls/RichLabel.h"
#include "Gwen/Controls/Label.h"
#include "Gwen/Utility.h"

using namespace Gwen;
using namespace Gwen::Controls;

const unsigned char Type_Text = 0;
const unsigned char Type_Newline = 1;

GWEN_CONTROL_CONSTRUCTOR(RichLabel)
{
	m_bNeedsRebuild = false;
}

void RichLabel::AddLineBreak()
{
	DividedText t;
	t.type = Type_Newline;

	m_TextBlocks.push_back(t);
}

void RichLabel::AddText(const Gwen::TextObject& text, Gwen::Color color, Gwen::Font* font)
{
	if (text.m_Data.size() == 0) return;

	Gwen::Utility::Strings::UnicodeList lst;
	Gwen::Utility::Strings::Split(text.GetUnicode(), L"\n", lst, false);

	for (size_t i = 0; i < lst.size(); i++)
	{
		if (i > 0) AddLineBreak();

		DividedText t;
		t.type = Type_Text;
		t.text = lst[i];
		t.color = color;
		t.font = font;

		m_TextBlocks.push_back(t);
		m_bNeedsRebuild = true;
		Invalidate();
	}
}

bool RichLabel::SizeToChildren(bool w, bool h)
{
	Rebuild();
	return BaseClass::SizeToChildren(w, h);
}

void RichLabel::SplitLabel(const Gwen::UnicodeString& text, Gwen::Font* pFont, const DividedText& txt, int& x, int& y, int& lineheight)
{
	Gwen::Utility::Strings::UnicodeList lst;
	Gwen::Utility::Strings::Split(text, L" ", lst, true);
	if (lst.size() == 0) return;

	int iSpaceLeft = Width() - x;

	// Does the whole word fit in?
	{
		Gwen::Point StringSize = GetSkin()->GetRender()->MeasureText(pFont, text);
		if (iSpaceLeft > StringSize.x)
		{
			return CreateLabel(text, txt, x, y, lineheight, true);
		}
	}

	// If the first word is bigger than the line, just give up.
	{
		Gwen::Point WordSize = GetSkin()->GetRender()->MeasureText(pFont, lst[0]);
		if (WordSize.x >= iSpaceLeft)
		{
			CreateLabel(lst[0], txt, x, y, lineheight, true);
			if (lst[0].size() >= text.size()) return;

			Gwen::UnicodeString LeftOver = text.substr(lst[0].size() + 1);
			return SplitLabel(LeftOver, pFont, txt, x, y, lineheight);
		}
	}

	Gwen::UnicodeString strNewString = L"";
	for (size_t i = 0; i < lst.size(); i++)
	{
		Gwen::Point WordSize = GetSkin()->GetRender()->MeasureText(pFont, strNewString + lst[i]);
		if (WordSize.x > iSpaceLeft)
		{
			CreateLabel(strNewString, txt, x, y, lineheight, true);
			x = 0;
			y += lineheight;
			break;
			;
		}

		strNewString += lst[i];
	}

	Gwen::UnicodeString LeftOver = text.substr(strNewString.size() + 1);
	return SplitLabel(LeftOver, pFont, txt, x, y, lineheight);
}

void RichLabel::CreateLabel(const Gwen::UnicodeString& text, const DividedText& txt, int& x, int& y, int& lineheight, bool NoSplit)
{
	//
	// Use default font or is one set?
	//
	Gwen::Font* pFont = GetSkin()->GetDefaultFont();
	if (txt.font) pFont = txt.font;

	//
	// This string is too long for us, split it up.
	//
	Gwen::Point p = GetSkin()->GetRender()->MeasureText(pFont, text);

	if (lineheight == -1)
	{
		lineheight = p.y;
	}

	if (!NoSplit)
	{
		if (x + p.x > Width())
		{
			return SplitLabel(text, pFont, txt, x, y, lineheight);
		}
	}

	//
	// Wrap
	//
	if (x + p.x >= Width())
	{
		CreateNewline(x, y, lineheight);
	}

	Gwen::Controls::Label* pLabel = new Gwen::Controls::Label(this);
	pLabel->SetText(x == 0 ? Gwen::Utility::Strings::TrimLeft<Gwen::UnicodeString>(text, L" ") : text);
	pLabel->SetTextColor(txt.color);
	pLabel->SetFont(pFont);
	pLabel->SizeToContents();
	pLabel->SetPos(x, y);

	//lineheight = (lineheight + pLabel->Height()) / 2;

	x += pLabel->Width();

	if (x >= Width())
	{
		CreateNewline(x, y, lineheight);
	}
}

void RichLabel::CreateNewline(int& x, int& y, int& lineheight)
{
	x = 0;
	y += lineheight;
}

void RichLabel::Rebuild()
{
	RemoveAllChildren();

	int x = 0;
	int y = 0;
	int lineheight = -1;
	for (DividedText::List::iterator it = m_TextBlocks.begin(); it != m_TextBlocks.end(); ++it)
	{
		if (it->type == Type_Newline)
		{
			CreateNewline(x, y, lineheight);
			continue;
		}

		if (it->type == Type_Text)
		{
			CreateLabel((*it).text, *it, x, y, lineheight, false);
			continue;
		}
	}

	m_bNeedsRebuild = false;
}

void RichLabel::OnBoundsChanged(Gwen::Rect oldBounds)
{
	BaseClass::OnBoundsChanged(oldBounds);

	Rebuild();
}

void RichLabel::Layout(Gwen::Skin::Base* skin)
{
	BaseClass::Layout(skin);

	if (m_bNeedsRebuild)
	{
		Rebuild();
	}
}