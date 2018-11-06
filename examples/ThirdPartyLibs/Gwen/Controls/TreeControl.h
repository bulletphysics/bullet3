/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_TREECONTROL_H
#define GWEN_CONTROLS_TREECONTROL_H

#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/TreeNode.h"

namespace Gwen
{
namespace Controls
{
class GWEN_EXPORT TreeControl : public TreeNode
{
public:
	GWEN_CONTROL(TreeControl, TreeNode);

	virtual void Render(Skin::Base* skin);

	virtual void OnChildBoundsChanged(Gwen::Rect oldChildBounds, Base* pChild);

	ScrollControl* Scroller() { return m_ScrollControl; }

	virtual void Clear();

	virtual void Layout(Skin::Base* skin);
	virtual void PostLayout(Skin::Base* skin);

	virtual void AllowMultiSelect(bool b) { m_bAllowMultipleSelection = b; }

	virtual void OnNodeAdded(TreeNode* pNode);

	virtual bool OnKeyUp(bool bDown);
	virtual bool OnKeyDown(bool bDown);
	virtual bool OnKeyRight(bool bDown);
	virtual bool OnKeyLeft(bool bDown);

	virtual void iterate(int action, int* curIndex, int* resultIndex);

	virtual void ForceUpdateScrollBars();

private:
	void OnNodeSelection(Controls::Base* control);

	ScrollControl* m_ScrollControl;
	bool m_bAllowMultipleSelection;
};
}  // namespace Controls
}  // namespace Gwen
#endif
