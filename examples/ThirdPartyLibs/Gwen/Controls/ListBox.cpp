/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Controls/ListBox.h"
#include "Gwen/Controls/ScrollControl.h"
#include "Gwen/InputHandler.h"

using namespace Gwen;
using namespace Gwen::Controls;

class ListBoxRow : public Layout::TableRow
{
	GWEN_CONTROL_INLINE(ListBoxRow, Layout::TableRow)
	{
		SetMouseInputEnabled(true);
		SetSelected(false);
	}

	void Render(Skin::Base* skin)
	{
		skin->DrawListBoxLine(this, IsSelected());
	}

	bool IsSelected() const
	{
		return m_bSelected;
	}

	void OnMouseClickLeft(int /*x*/, int /*y*/, bool bDown)
	{
		if (bDown && !m_bSelected)
		{
			SetSelected(true);
			onRowSelected.Call(this);
		}
	}

	void SetSelected(bool b)
	{
		m_bSelected = b;

		// TODO: Get these values from the skin.
		if (b)
			SetTextColor(Gwen::Colors::White);
		else
			SetTextColor(Gwen::Colors::Black);
	}

private:
	bool m_bSelected;
};

GWEN_CONTROL_CONSTRUCTOR(ListBox)
{
	m_ScrollControl = new ScrollControl(this);
	m_ScrollControl->Dock(Pos::Fill);
	m_ScrollControl->SetScroll(false, true);
	m_ScrollControl->SetAutoHideBars(true);
	m_ScrollControl->SetMargin(Margin(1, 1, 1, 1));

	m_InnerPanel = m_ScrollControl;

	m_Table = new Controls::Layout::Table(this);
	m_Table->Dock(Pos::Top);
	m_Table->SetColumnCount(1);

	m_bMultiSelect = false;
}

void ListBox::OnChildBoundsChanged(Gwen::Rect /*oldChildBounds*/, Base* /*pChild*/)
{
	m_ScrollControl->UpdateScrollBars();
}

Layout::TableRow* ListBox::AddItem(const String& strLabel, const String& strName)
{
	return AddItem(Utility::StringToUnicode(strLabel), strName);
}

Layout::TableRow* ListBox::AddItem(const UnicodeString& strLabel, const String& strName)
{
	ListBoxRow* pRow = new ListBoxRow(this);
	m_Table->AddRow(pRow);

	pRow->SetCellText(0, strLabel);
	pRow->SetName(strName);

	pRow->onRowSelected.Add(this, &ListBox::OnRowSelected);

	m_Table->SizeToContents();

	return pRow;
}

void ListBox::Render(Skin::Base* skin)
{
	skin->DrawListBox(this);
}

void ListBox::UnselectAll()
{
	std::list<Layout::TableRow*>::iterator it = m_SelectedRows.begin();
	while (it != m_SelectedRows.end())
	{
		ListBoxRow* pRow = static_cast<ListBoxRow*>(*it);
		it = m_SelectedRows.erase(it);

		pRow->SetSelected(false);
	}
}

void ListBox::OnRowSelected(Base* pControl)
{
	ListBoxRow* pRow = static_cast<ListBoxRow*>(pControl);

	if (!AllowMultiSelect() || !Gwen::Input::IsShiftDown())
	{
		UnselectAll();
	}

	m_SelectedRows.push_back(pRow);

	onRowSelected.Call(this);
}

Layout::TableRow* ListBox::GetSelectedRow()
{
	if (m_SelectedRows.empty()) return NULL;

	return *m_SelectedRows.begin();
}

void ListBox::Clear()
{
	UnselectAll();
	m_Table->Clear();
}