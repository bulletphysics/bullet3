/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_SKIN_H
#define GWEN_SKIN_H

#include "Gwen/BaseRender.h"
#include "Gwen/Font.h"

namespace Gwen
{
namespace Controls
{
class Base;
}

namespace Skin
{
namespace Symbol
{
const unsigned char None = 0;
const unsigned char ArrowRight = 1;
const unsigned char Check = 2;
const unsigned char Dot = 3;
}  // namespace Symbol

class GWEN_EXPORT Base
{
public:
	Base()
	{
		m_DefaultFont.facename = L"Arial";
		m_DefaultFont.size = 10.0f;
		m_Render = NULL;
	}

	virtual ~Base()
	{
		ReleaseFont(&m_DefaultFont);
	}

	virtual void ReleaseFont(Gwen::Font* fnt)
	{
		if (!fnt) return;
		if (!m_Render) return;

		m_Render->FreeFont(fnt);
	}

	virtual void DrawButton(Controls::Base* control, bool bDepressed, bool bHovered) = 0;
	virtual void DrawTabButton(Controls::Base* control, bool bActive) = 0;
	virtual void DrawTabControl(Controls::Base* control, Gwen::Rect CurrentButtonRect) = 0;
	virtual void DrawTabTitleBar(Controls::Base* control) = 0;

	virtual void DrawMenuItem(Controls::Base* control, bool bSubmenuOpen, bool bChecked) = 0;
	virtual void DrawMenuStrip(Controls::Base* control) = 0;
	virtual void DrawMenu(Controls::Base* control, bool bPaddingDisabled) = 0;
	virtual void DrawRadioButton(Controls::Base* control, bool bSelected, bool bDepressed) = 0;
	virtual void DrawCheckBox(Controls::Base* control, bool bSelected, bool bDepressed) = 0;
	virtual void DrawGroupBox(Controls::Base* control, int textStart, int textHeight, int textWidth) = 0;
	virtual void DrawTextBox(Controls::Base* control) = 0;
	virtual void DrawWindow(Controls::Base* control, int topHeight, bool inFocus) = 0;
	virtual void DrawHighlight(Controls::Base* control) = 0;
	virtual void DrawBackground(Controls::Base* control) = 0;
	virtual void DrawStatusBar(Controls::Base* control) = 0;

	virtual void DrawShadow(Controls::Base* control) = 0;
	virtual void DrawScrollBarBar(Controls::Base* control, bool bDepressed, bool isHovered, bool isHorizontal) = 0;
	virtual void DrawScrollBar(Controls::Base* control, bool isHorizontal, bool bDepressed) = 0;
	virtual void DrawScrollButton(Controls::Base* control, int iDirection, bool bDepressed) = 0;
	virtual void DrawProgressBar(Controls::Base* control, bool isHorizontal, float progress) = 0;

	virtual void DrawListBox(Controls::Base* control) = 0;
	virtual void DrawListBoxLine(Controls::Base* control, bool bSelected) = 0;

	virtual void DrawSlider(Controls::Base* control, bool bIsHorizontal, int numNotches, int barSize) = 0;
	virtual void DrawComboBox(Controls::Base* control) = 0;
	virtual void DrawComboBoxButton(Controls::Base* control, bool bDepressed) = 0;
	virtual void DrawKeyboardHighlight(Controls::Base* control, const Gwen::Rect& rect, int offset) = 0;
	//virtual void DrawComboBoxKeyboardHighlight( Controls::Base* control );
	virtual void DrawToolTip(Controls::Base* control) = 0;

	virtual void DrawNumericUpDownButton(Controls::Base* control, bool bDepressed, bool bUp) = 0;

	virtual void DrawTreeButton(Controls::Base* control, bool bOpen) = 0;
	virtual void DrawTreeControl(Controls::Base* control) = 0;
	virtual void DrawTreeNode(Controls::Base* ctrl, bool bOpen, bool bSelected, int iLabelHeight, int iLabelWidth, int iHalfWay, int iLastBranch, bool bIsRoot) = 0;

	virtual void DrawPropertyRow(Controls::Base* control, int iWidth, bool bBeingEdited) = 0;
	virtual void DrawPropertyTreeNode(Controls::Base* control, int BorderLeft, int BorderTop) = 0;
	virtual void DrawColorDisplay(Controls::Base* control, Gwen::Color color) = 0;
	virtual void DrawModalControl(Controls::Base* control) = 0;
	virtual void DrawMenuDivider(Controls::Base* control) = 0;

	virtual void SetRender(Gwen::Renderer::Base* renderer)
	{
		m_Render = renderer;
	}
	virtual Gwen::Renderer::Base* GetRender()
	{
		return m_Render;
	}

	virtual void DrawArrowDown(Gwen::Rect rect);
	virtual void DrawArrowUp(Gwen::Rect rect);
	virtual void DrawArrowLeft(Gwen::Rect rect);
	virtual void DrawArrowRight(Gwen::Rect rect);
	virtual void DrawCheck(Gwen::Rect rect);

public:
	virtual Gwen::Font* GetDefaultFont()
	{
		return &m_DefaultFont;
	}

	virtual void SetDefaultFont(const Gwen::UnicodeString& strFacename, float fSize = 10.0f)
	{
		m_DefaultFont.facename = strFacename;
		m_DefaultFont.size = fSize;
	}

protected:
	Gwen::Font m_DefaultFont;
	Gwen::Renderer::Base* m_Render;
};
};  // namespace Skin
}  // namespace Gwen
#endif
