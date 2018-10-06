#pragma once
#ifndef GWEN_CONTROLS_PANELLISTPANEL_H
#define GWEN_CONTROLS_PANELLISTPANEL_H

#include "Gwen/Gwen.h"
#include "Gwen/Controls/Base.h"

namespace Gwen
{
namespace Controls
{
class GWEN_EXPORT PanelListPanel : public Controls::Base
{
public:
	GWEN_CONTROL(PanelListPanel, Controls::Base);

	void Render(Gwen::Skin::Base* skin);
	void Layout(Skin::Base* skin);

	void DoHorizontalLayout();
	void DoVerticalLayout();

	bool IsVerticalLayout() { return m_bVertical; }
	bool IsHorizontalLayout() { return !m_bVertical; }
	void SetVertical()
	{
		m_bVertical = true;
		Invalidate();
	}
	void SetHorizontal()
	{
		m_bVertical = false;
		Invalidate();
	}

	void SetSizeToChildren(bool bShould) { m_bSizeToChildren = bShould; }
	void SetControlSpacing(int spacing) { m_iControlSpacing = spacing; }
	void SetLineSpacing(int spacing) { m_iLineSpacing = spacing; }
	void SetWrapping(bool wrap) { m_bWrapping = wrap; }

	Gwen::Point GetBiggestChildSize();

protected:
	bool m_bVertical;
	bool m_bSizeToChildren;
	int m_iControlSpacing;
	int m_iLineSpacing;
	bool m_bWrapping;
};
}  // namespace Controls
}  // namespace Gwen
#endif
