/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_RICHLABEL_H
#define GWEN_CONTROLS_RICHLABEL_H

#include "Gwen/BaseRender.h"
#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Text.h"
#include "Gwen/TextObject.h"

namespace Gwen
{
namespace Controls
{
class GWEN_EXPORT RichLabel : public Controls::Base
{
public:
	GWEN_CONTROL(RichLabel, Gwen::Controls::Base);

	void AddLineBreak();
	void AddText(const Gwen::TextObject& text, Gwen::Color color, Gwen::Font* font = NULL);

	virtual bool SizeToChildren(bool w = true, bool h = true);

protected:
	struct DividedText
	{
		typedef std::list<DividedText> List;
		DividedText()
		{
			type = 0;
			font = NULL;
		}

		unsigned char type;
		Gwen::UnicodeString text;
		Gwen::Color color;
		Gwen::Font* font;
	};

	void Layout(Gwen::Skin::Base* skin);
	void SplitLabel(const Gwen::UnicodeString& text, Gwen::Font* pFont, const DividedText& txt, int& x, int& y, int& lineheight);
	void CreateNewline(int& x, int& y, int& lineheight);
	void CreateLabel(const Gwen::UnicodeString& text, const DividedText& txt, int& x, int& y, int& lineheight, bool NoSplit);
	void Rebuild();

	void OnBoundsChanged(Gwen::Rect oldBounds);

	DividedText::List m_TextBlocks;
	bool m_bNeedsRebuild;
};
}  // namespace Controls
}  // namespace Gwen
#endif
