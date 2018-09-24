/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Controls/TreeControl.h"
#include "Gwen/Controls/ScrollControl.h"
#include "Gwen/Utility.h"

using namespace Gwen;
using namespace Gwen::Controls;

GWEN_CONTROL_CONSTRUCTOR(TreeControl)
{
	m_TreeControl = this;
	m_bUpdateScrollBar = 2;
	m_ToggleButton->DelayedDelete();
	m_ToggleButton = NULL;
	m_Title->DelayedDelete();
	m_Title = NULL;
	m_InnerPanel->DelayedDelete();
	m_InnerPanel = NULL;

	m_bAllowMultipleSelection = false;

	m_ScrollControl = new ScrollControl(this);
	m_ScrollControl->Dock(Pos::Fill);
	m_ScrollControl->SetScroll(false, true);
	m_ScrollControl->SetAutoHideBars(true);
	m_ScrollControl->SetMargin(Margin(1, 1, 1, 1));

	m_InnerPanel = m_ScrollControl;

	m_ScrollControl->SetInnerSize(1000, 1000);
}

void TreeControl::Render(Skin::Base* skin)
{
	if (ShouldDrawBackground())
		skin->DrawTreeControl(this);
}

void TreeControl::ForceUpdateScrollBars()
{
	m_ScrollControl->UpdateScrollBars();
}

void TreeControl::OnChildBoundsChanged(Gwen::Rect /*oldChildBounds*/, Base* /*pChild*/)
{
}

void TreeControl::Clear()
{
	m_ScrollControl->Clear();
}

void TreeControl::Layout(Skin::Base* skin)
{
	BaseClass::BaseClass::Layout(skin);
}

void TreeControl::PostLayout(Skin::Base* skin)
{
	BaseClass::BaseClass::PostLayout(skin);
}

void TreeControl::OnNodeAdded(TreeNode* pNode)
{
	pNode->onNamePress.Add(this, &TreeControl::OnNodeSelection);
}

void TreeControl::OnNodeSelection(Controls::Base* /*control*/)
{
	//printf("TreeControl::OnNodeSelection\n");
	if (!m_bAllowMultipleSelection || !Gwen::Input::IsKeyDown(Key::Control))
		DeselectAll();
}

void TreeControl::iterate(int action, int* maxItem, int* curItem)
{
	Base::List& children = m_InnerPanel->GetChildren();
	for (Base::List::iterator iter = children.begin(); iter != children.end(); ++iter)
	{
		TreeNode* pChild = (*iter)->DynamicCastTreeNode();
		if (!pChild)
			continue;
		pChild->iterate(action, maxItem, curItem);
	}
}

bool TreeControl::OnKeyUp(bool bDown)
{
	if (bDown)
	{
		//		int maxIndex = 0;
		int newIndex = 0;
		int maxItem = 0;
		int curItem = -1;
		iterate(ITERATE_ACTION_FIND_SELECTED_INDEX, &maxItem, &curItem);
		//	maxIndex = maxItem;
		int targetItem = curItem;
		if (curItem > 0)
		{
			maxItem = 0;
			int deselectIndex = targetItem;
			targetItem--;
			newIndex = targetItem;
			iterate(ITERATE_ACTION_SELECT, &maxItem, &targetItem);
			if (targetItem < 0)
			{
				maxItem = 0;
				iterate(ITERATE_ACTION_DESELECT_INDEX, &maxItem, &deselectIndex);
			}
			curItem = newIndex;
			//	float amount = float(newIndex)/float(maxIndex);
			float viewSize = m_ScrollControl->m_VerticalScrollBar->getViewableContentSize();
			float contSize = m_ScrollControl->m_VerticalScrollBar->getContentSize();

			float curAmount = m_ScrollControl->m_VerticalScrollBar->GetScrolledAmount();
			//	float minCoordViewableWindow = curAmount*contSize;
			//float maxCoordViewableWindow = minCoordViewableWindow+viewSize;
			float minCoordSelectedItem = curItem * 16.f;
			//		float maxCoordSelectedItem = (curItem+1)*16.f;
			if (contSize != viewSize)
			{
				{
					float newAmount = float(minCoordSelectedItem) / (contSize - viewSize);
					if (newAmount < curAmount)
					{
						m_ScrollControl->m_VerticalScrollBar->SetScrolledAmount(newAmount, true);
					}
				}
				{
					int numItems = (viewSize) / 16 - 1;
					float newAmount = float((curItem - numItems) * 16) / (contSize - viewSize);

					if (newAmount > curAmount)
					{
						m_ScrollControl->m_VerticalScrollBar->SetScrolledAmount(newAmount, true);
					}
				}
			}
		}
	}
	ForceUpdateScrollBars();
	return true;
}

bool TreeControl::OnKeyDown(bool bDown)
{
	if (bDown)
	{
		//	int maxIndex = 0;
		int newIndex = 0;
		int maxItem = 0;
		int curItem = -1;
		iterate(ITERATE_ACTION_FIND_SELECTED_INDEX, &maxItem, &curItem);
		//	maxIndex = maxItem;
		int targetItem = curItem;
		if (curItem >= 0)
		{
			maxItem = 0;
			int deselectIndex = targetItem;
			targetItem++;
			newIndex = targetItem;
			iterate(ITERATE_ACTION_SELECT, &maxItem, &targetItem);
			if (targetItem < 0)
			{
				maxItem = 0;
				iterate(ITERATE_ACTION_DESELECT_INDEX, &maxItem, &deselectIndex);
			}
			curItem = newIndex;
			//	float amount = (int)float(newIndex)/float(maxIndex);
			float viewSize = m_ScrollControl->m_VerticalScrollBar->getViewableContentSize();
			float contSize = m_ScrollControl->m_VerticalScrollBar->getContentSize();

			float curAmount = m_ScrollControl->m_VerticalScrollBar->GetScrolledAmount();
			//	float minCoordViewableWindow = curAmount*contSize;
			//float maxCoordViewableWindow = minCoordViewableWindow+viewSize;
			float minCoordSelectedItem = curItem * 16.f;
			//float maxCoordSelectedItem = (curItem+1)*16.f;
			if (contSize != viewSize)
			{
				{
					float newAmount = float(minCoordSelectedItem) / (contSize - viewSize);
					if (newAmount < curAmount)
					{
						m_ScrollControl->m_VerticalScrollBar->SetScrolledAmount(newAmount, true);
					}
				}
				{
					int numItems = (viewSize) / 16 - 1;
					float newAmount = float((curItem - numItems) * 16) / (contSize - viewSize);

					if (newAmount > curAmount)
					{
						m_ScrollControl->m_VerticalScrollBar->SetScrolledAmount(newAmount, true);
					}
				}
			}
		}
	}
	ForceUpdateScrollBars();
	return true;
}
extern int avoidUpdate;

bool TreeControl::OnKeyRight(bool bDown)
{
	if (bDown)
	{
		avoidUpdate = -3;

		iterate(ITERATE_ACTION_OPEN, 0, 0);
		int maxItem = 0;
		int curItem = 0;
		iterate(ITERATE_ACTION_FIND_SELECTED_INDEX, &maxItem, &curItem);
		//	float amount = float(curItem)/float(maxItem);
		float viewSize = m_ScrollControl->m_VerticalScrollBar->getViewableContentSize();
		float contSize = m_ScrollControl->m_VerticalScrollBar->getContentSize();

		float curAmount = m_ScrollControl->m_VerticalScrollBar->GetScrolledAmount();
		//	float minCoordViewableWindow = curAmount*contSize;
		//		float maxCoordViewableWindow = minCoordViewableWindow+viewSize;
		float minCoordSelectedItem = curItem * 16.f;
		//	float maxCoordSelectedItem = (curItem+1)*16.f;
		if (contSize != viewSize)
		{
			{
				float newAmount = float(minCoordSelectedItem) / (contSize - viewSize);
				if (newAmount < curAmount)
				{
					m_ScrollControl->m_VerticalScrollBar->SetScrolledAmount(newAmount, true);
				}
			}
			{
				int numItems = (viewSize) / 16 - 1;
				float newAmount = float((curItem - numItems) * 16) / (contSize - viewSize);

				if (newAmount > curAmount)
				{
					m_ScrollControl->m_VerticalScrollBar->SetScrolledAmount(newAmount, true);
				}
			}
		}
		Invalidate();
	}
	ForceUpdateScrollBars();
	return true;
}
bool TreeControl::OnKeyLeft(bool bDown)
{
	if (bDown)
	{
		avoidUpdate = -3;

		iterate(ITERATE_ACTION_CLOSE, 0, 0);

		int maxItems = 0;
		int curItem = 0;
		iterate(ITERATE_ACTION_FIND_SELECTED_INDEX, &maxItems, &curItem);
		//	float amount = float(curItem)/float(maxItems);

		//	m_ScrollControl->m_VerticalScrollBar->SetScrolledAmount(amount,true);
		float viewSize = m_ScrollControl->m_VerticalScrollBar->getViewableContentSize();
		float contSize = m_ScrollControl->m_VerticalScrollBar->getContentSize();

		float curAmount = m_ScrollControl->m_VerticalScrollBar->GetScrolledAmount();
		//	float minCoordViewableWindow = curAmount*contSize;
		//	float maxCoordViewableWindow = minCoordViewableWindow+viewSize;
		float minCoordSelectedItem = curItem * 16.f;
		//	float maxCoordSelectedItem = (curItem+1)*16.f;
		if (contSize != viewSize)
		{
			{
				float newAmount = float(minCoordSelectedItem) / (contSize - viewSize);
				if (newAmount < curAmount)
				{
					m_ScrollControl->m_VerticalScrollBar->SetScrolledAmount(newAmount, true);
				}
			}
			{
				int numItems = (viewSize) / 16 - 1;
				float newAmount = float((curItem - numItems) * 16) / (contSize - viewSize);

				if (newAmount > curAmount)
				{
					m_ScrollControl->m_VerticalScrollBar->SetScrolledAmount(newAmount, true);
				}
				Invalidate();
			}
		}
		//viewSize/contSize

		//printf("!\n");

		//this->Layout(0);
	}
	ForceUpdateScrollBars();
	return true;
}
