#include "UnitTest.h"
#include "Gwen/Controls/ListBox.h"

using namespace Gwen;

class ListBox : public GUnit
{
	public:

	GWEN_CONTROL_INLINE( ListBox, GUnit )
	{
		{
			Gwen::Controls::ListBox* ctrl = new Gwen::Controls::ListBox( this );
			ctrl->SetBounds( 10, 10, 100, 200 );

			ctrl->AddItem( L"First" );
			ctrl->AddItem( L"Blue" );
			ctrl->AddItem( L"Yellow" );
			ctrl->AddItem( L"Orange" );
			ctrl->AddItem( L"Brown" );
			ctrl->AddItem( L"Green" );
			ctrl->AddItem( L"Dog" );
			ctrl->AddItem( L"Cat" );
			ctrl->AddItem( L"Shoes" );
			ctrl->AddItem( L"Chair" );
			ctrl->AddItem( L"Last" );

			ctrl->onRowSelected.Add( this, &ThisClass::RowSelected );
		}

		{
			Gwen::Controls::ListBox* ctrl = new Gwen::Controls::ListBox( this );
			ctrl->SetBounds( 120, 10, 200, 200 );
			ctrl->SetColumnCount( 3 );
			ctrl->SetAllowMultiSelect( true );
			ctrl->onRowSelected.Add( this, &ThisClass::RowSelected );


			{
				Gwen::Controls::Layout::TableRow* pRow = ctrl->AddItem( L"Baked Beans" );
				pRow->SetCellText( 1, L"Heinz" );
				pRow->SetCellText( 2, L"£3.50" );
			}

			{
				Gwen::Controls::Layout::TableRow* pRow = ctrl->AddItem( L"Bananas" );
				pRow->SetCellText( 1, L"Trees" );
				pRow->SetCellText( 2, L"£1.27" );
			}

			{
				Gwen::Controls::Layout::TableRow* pRow = ctrl->AddItem( L"Chicken" );
				pRow->SetCellText( 1, L"\u5355\u5143\u6D4B\u8BD5" );
				pRow->SetCellText( 2, L"£8.95" );
			}
		}
	}

	void RowSelected( Gwen::Controls::Base* pControl )
	{
		Gwen::Controls::ListBox* ctrl = (Gwen::Controls::ListBox*)pControl;

		UnitPrint( Utility::Format( L"Listbox Item Selected: %s", ctrl->GetSelectedRow()->GetText( 0 ).c_str() ) );
	}

	Gwen::Font	m_Font;
};



DEFINE_UNIT_TEST( ListBox, L"ListBox" );