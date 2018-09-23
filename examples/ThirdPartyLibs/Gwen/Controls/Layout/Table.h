/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_CONTROLS_LAYOUT_TABLE_H
#define GWEN_CONTROLS_LAYOUT_TABLE_H

#include "Gwen/Controls/Label.h"
#include "Gwen/Utility.h"

namespace Gwen
{
namespace Controls
{
namespace Layout
{
class Table;

class GWEN_EXPORT TableRow : public Base
{
	static const int MaxColumns = 5;

	GWEN_CONTROL_INLINE(TableRow, Base)
	{
		for (int i = 0; i < MaxColumns; i++)
			m_Columns[i] = NULL;

		m_ColumnCount = 0;
	}

	virtual class TableRow* DynamicCastLayoutTableRow()
	{
		return this;
	}
	virtual const class TableRow* DynamicCastLayoutTableRow() const
	{
		return this;
	}

	void SetColumnCount(int iCount)
	{
		if (iCount == m_ColumnCount) return;

		if (iCount >= MaxColumns)
			m_ColumnCount = MaxColumns;

		for (int i = 0; i < MaxColumns; i++)
		{
			if (i < iCount)
			{
				if (!m_Columns[i])
				{
					m_Columns[i] = new Label(this);
					m_Columns[i]->Dock(Pos::Left);
					m_Columns[i]->SetPadding(Padding(3, 3, 3, 3));
				}
			}
			else if (m_Columns[i])
			{
				m_Columns[i]->DelayedDelete();
				m_Columns[i] = NULL;
			}

			m_ColumnCount = iCount;
		}
	}

	void SetColumnWidth(int i, int iWidth)
	{
		if (!m_Columns[i]) return;
		if (m_Columns[i]->Width() == iWidth) return;

		m_Columns[i]->SetWidth(iWidth);
	}

	template <typename T>
	void SetCellText(int i, const T& strString)
	{
		if (!m_Columns[i]) return;
		m_Columns[i]->SetText(strString);
	}

	void SetCellContents(int i, Base* pControl, bool bEnableMouseInput = false)
	{
		if (!m_Columns[i]) return;
		pControl->SetParent(m_Columns[i]);

		m_Columns[i]->SetMouseInputEnabled(bEnableMouseInput);
	}

	Label* GetCellContents(int i)
	{
		return m_Columns[i];
	}

	void SizeToContents()
	{
		int iHeight = 0;

		for (int i = 0; i < m_ColumnCount; i++)
		{
			if (!m_Columns[i]) continue;

			// Note, more than 1 child here, because the
			// label has a child built in ( The Text )
			if (m_Columns[i]->NumChildren() > 1)
			{
				m_Columns[i]->SizeToChildren();
			}
			else
			{
				m_Columns[i]->SizeToContents();
			}

			iHeight = Utility::Max(iHeight, m_Columns[i]->Height());
		}

		SetHeight(iHeight);
	}

	void SetTextColor(const Gwen::Color& color)
	{
		for (int i = 0; i < m_ColumnCount; i++)
		{
			if (!m_Columns[i]) continue;
			m_Columns[i]->SetTextColor(color);
		}
	}

	//You might hate this. Actually I know you will
	virtual UnicodeString GetText(int i)
	{
		return m_Columns[i]->GetText();
	}
	virtual void SetSelected(bool /*b*/) {}

	//
	// This is sometimes called by derivatives.
	//
	Gwen::Event::Caller onRowSelected;

private:
	int m_ColumnCount;
	Label* m_Columns[MaxColumns];

	friend class Table;
};

class GWEN_EXPORT Table : public Base
{
public:
	GWEN_CONTROL_INLINE(Table, Base)
	{
		m_iColumnCount = 1;
		m_iDefaultRowHeight = 22;

		for (int i = 0; i < TableRow::MaxColumns; i++)
		{
			m_ColumnWidth[i] = 20;
		}

		m_bSizeToContents = false;
	}

	void SetColumnCount(int i)
	{
		if (m_iColumnCount == i) return;

		for (Base::List::iterator it = Children.begin(); it != Children.end(); ++it)
		{
			if (!*it)
				continue;

			TableRow* pRow = (*it)->DynamicCastLayoutTableRow();
			if (!pRow) continue;

			pRow->SetColumnCount(i);
		}

		m_iColumnCount = i;
	}

	void SetColumnWidth(int i, int iWidth)
	{
		if (m_ColumnWidth[i] == iWidth) return;

		m_ColumnWidth[i] = iWidth;
		Invalidate();
	}

	TableRow* AddRow()
	{
		TableRow* row = new TableRow(this);
		row->SetColumnCount(m_iColumnCount);
		row->SetHeight(m_iDefaultRowHeight);
		row->Dock(Pos::Top);
		return row;
	}

	void AddRow(TableRow* pRow)
	{
		pRow->SetParent(this);
		pRow->SetColumnCount(m_iColumnCount);
		pRow->SetHeight(m_iDefaultRowHeight);
		pRow->Dock(Pos::Top);
	}

	void Remove(TableRow* pRow)
	{
		pRow->DelayedDelete();
	}

	void Clear()
	{
		for (Base::List::iterator it = Children.begin(); it != Children.end(); ++it)
		{
			if (!(*it))
				continue;

			TableRow* pRow = (*it)->DynamicCastLayoutTableRow();

			if (!pRow) continue;
			Remove(pRow);
		}
	}

	void Layout(Skin::Base* skin)
	{
		BaseClass::Layout(skin);

		if (m_bSizeToContents)
		{
			DoSizeToContents();
		}

		for (Base::List::iterator it = Children.begin(); it != Children.end(); ++it)
		{
			if (!*it)
				continue;

			TableRow* pRow = (*it)->DynamicCastLayoutTableRow();
			if (!pRow) continue;

			for (int i = 0; i < TableRow::MaxColumns && i < m_iColumnCount; i++)
			{
				pRow->SetColumnWidth(i, m_ColumnWidth[i]);
			}
		}
	}

	void PostLayout(Skin::Base* /*skin*/)
	{
		if (m_bSizeToContents)
		{
			SizeToChildren();
			m_bSizeToContents = false;
		}
	}

	void SizeToContents()
	{
		m_bSizeToContents = true;
		Invalidate();
	}

	void DoSizeToContents()
	{
		for (int i = 0; i < TableRow::MaxColumns; i++)
		{
			m_ColumnWidth[i] = 10;
		}

		for (Base::List::iterator it = Children.begin(); it != Children.end(); ++it)
		{
			if (!*it)
				continue;

			TableRow* pRow = (*it)->DynamicCastLayoutTableRow();
			if (!pRow) continue;

			pRow->SizeToContents();

			for (int i = 0; i < TableRow::MaxColumns; i++)
			{
				if (pRow->m_Columns[i])
				{
					m_ColumnWidth[i] = Utility::Max(m_ColumnWidth[i], pRow->m_Columns[i]->Width());
				}
			}
			//iBottom += pRow->Height();
		}

		InvalidateParent();
	}

private:
	bool m_bSizeToContents;
	int m_iColumnCount;
	int m_iDefaultRowHeight;

	int m_ColumnWidth[TableRow::MaxColumns];
};
}  // namespace Layout
}  // namespace Controls
}  // namespace Gwen
#endif
