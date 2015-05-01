/*
	GWEN
	Copyright (c) 2011 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#pragma once
#include "Gwen/Controls/Label.h"

namespace Gwen 
{
	namespace Controls
	{
		namespace Layout
		{
			class Table;

			class TableRow : public Base
			{
				static const int MaxColumns = 5;

				GWEN_CONTROL_INLINE( TableRow, Base )
				{
					for ( int i=0; i<MaxColumns; i++ )
						m_Columns[i] = NULL;

					m_ColumnCount = 0;
				}

				void SetColumnCount( int iCount )
				{
					if ( iCount == m_ColumnCount ) return;

					if ( iCount >= MaxColumns ) 
						m_ColumnCount = MaxColumns;

					for ( int i=0; i<MaxColumns; i++ )
					{
						if ( i < iCount )
						{
							if ( !m_Columns[i] )
							{
								m_Columns[i] = new Label( this );
								m_Columns[i]->Dock( Pos::Left );
								m_Columns[i]->SetTextPadding( Gwen::Rect( 3, 3, 3, 3 ) );
							}
						}
						else if ( m_Columns[i] )
						{
							m_Columns[i]->DelayedDelete();
							m_Columns[i] = NULL;
						}

						m_ColumnCount = iCount;
					}
				}

				void SetColumnWidth( int i, int iWidth )
				{
					if ( !m_Columns[i] ) return;
					if ( m_Columns[i]->Width() == iWidth ) return;

					m_Columns[i]->SetWidth( iWidth );
				}

				template <typename T>
				void SetCellText( int i, const T& strString )
				{
					if ( !m_Columns[i] ) return;
					m_Columns[i]->SetText( strString );
				}

				void SetCellContents( int i, Base* pControl, bool bEnableMouseInput = false )
				{
					if ( !m_Columns[i] ) return;
					pControl->SetParent( m_Columns[i] );

					m_Columns[i]->SetMouseInputEnabled( bEnableMouseInput );
				}

				Label* GetCellContents( int i )
				{
					return m_Columns[i];
				}

				void SizeToContents()
				{
					int iHeight = 0;

					for ( int i=0; i<m_ColumnCount; i++ )
					{
						if ( !m_Columns[i] ) continue;

						// Note, more than 1 child here, because the 
						// label has a child built in ( The Text )
						if ( m_Columns[i]->NumChildren() > 1 )
						{
							m_Columns[i]->SizeToChildren();
						}
						else
						{
							m_Columns[i]->SizeToContents();
						}

						iHeight = max( iHeight, m_Columns[i]->Height() );
					}

					SetHeight( iHeight );
				}

				void SetTextColor( const Gwen::Color& color )
				{
					for ( int i=0; i<m_ColumnCount; i++ )
					{
						if ( !m_Columns[i] ) continue;
						m_Columns[i]->SetTextColor( color );
					}
				}

				//You might hate this. Actually I know you will
				virtual UnicodeString GetText( int i )
				{
					return m_Columns[i]->GetText();
				}
				virtual void SetSelected( bool b ) {}

			private:

				int		m_ColumnCount;
				Label*	m_Columns[MaxColumns];

				friend class Table;


			};

			class Table : public Base
			{
				public:

					GWEN_CONTROL_INLINE( Table, Base )
					{
						m_iColumnCount = 1;
						m_iDefaultRowHeight = 22;

						for (int i=0; i<TableRow::MaxColumns; i++)
						{
							m_ColumnWidth[i] = 20 + rand()%100;
						}


						m_bSizeToContents = false;
					}

					void SetColumnCount( int i )
					{
						if ( m_iColumnCount == i ) return;

						for ( Base::List::iterator it = m_Children.begin(); it != m_Children.end(); ++it )
						{
							Base* el = *it;
							if (el->getType()!=TypeTableRow)
								continue;

							TableRow* pRow = static_cast<TableRow*>(*it);
							
							pRow->SetColumnCount( i );
						}

						m_iColumnCount = i;
					}

					void SetColumnWidth( int i, int iWidth )
					{
						if ( m_ColumnWidth[i] == iWidth ) return;

						m_ColumnWidth[i] = iWidth;
						Invalidate();
					}

					TableRow* AddRow()
					{
						TableRow* row = new TableRow( this );
						row->SetColumnCount( m_iColumnCount );
						row->SetHeight( m_iDefaultRowHeight );
						row->Dock( Pos::Top );
						return row;
					}

					void AddRow( TableRow* pRow )
					{
						pRow->SetParent( this );
						pRow->SetColumnCount( m_iColumnCount );
						pRow->SetHeight( m_iDefaultRowHeight );
						pRow->Dock( Pos::Top );
					}

					void Layout( Skin::Base* skin )
					{
						Debug::Msg( "TABLE LAYOUT\n" );
						BaseClass::Layout( skin );

						if ( m_bSizeToContents )
						{
							DoSizeToContents();
							m_bSizeToContents = false;
						}

						for ( Base::List::iterator it = m_Children.begin(); it != m_Children.end(); ++it )
						{
							TableRow* pRow = static_cast<TableRow*>(*it);
							if ( !pRow ) continue;

							for (int i=0; i<TableRow::MaxColumns && i < m_iColumnCount; i++)
							{
								pRow->SetColumnWidth( i, m_ColumnWidth[i] );
							}
						}
					}

					void SizeToContents()
					{
						m_bSizeToContents = true;
						Invalidate();
					}

					void DoSizeToContents()
					{
						for (int i=0; i<TableRow::MaxColumns; i++)
						{
							m_ColumnWidth[i] = 10;
						}

						for ( Base::List::iterator it = m_Children.begin(); it != m_Children.end(); ++it )
						{
							TableRow* pRow = static_cast<TableRow*>(*it);
							if ( !pRow ) continue;

							pRow->SizeToContents();

							for (int i=0; i<TableRow::MaxColumns; i++)
							{
								if ( pRow->m_Columns[i] )
								{
									m_ColumnWidth[i] = max( m_ColumnWidth[i], pRow->m_Columns[i]->Width() );
								}
							}
						}

						Invalidate();
					}

				private:

					bool	m_bSizeToContents;
					int		m_iColumnCount;
					int		m_iDefaultRowHeight;

					int		m_ColumnWidth[ TableRow::MaxColumns ];
			};
		}
	}
}
