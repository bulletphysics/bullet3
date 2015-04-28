#include "UnitTest.h"
#include "Gwen/Controls/Properties.h"
#include "Gwen/Controls/PropertyTree.h"

using namespace Gwen;

class Properties2 : public GUnit
{
	public:

	GWEN_CONTROL_INLINE( Properties2, GUnit )
	{
		{
			Gwen::Controls::Properties* props = new Gwen::Controls::Properties( this );

			props->SetBounds( 10, 10, 150, 300 );

			{
				{
					Gwen::Controls::PropertyRow* pRow = props->Add( L"First Name" );
					pRow->onChange.Add( this, &Properties2::OnFirstNameChanged );
				}

				props->Add( L"Middle Name" );
				props->Add( L"Last Name" );
			}

		}

		{
			Gwen::Controls::PropertyTree* ptree = new Gwen::Controls::PropertyTree( this );
			ptree->SetBounds( 200, 10, 200, 200 );

			{
				Gwen::Controls::Properties* props = ptree->Add( L"Item One" );
				props->Add( L"Middle Name" );
				props->Add( L"Last Name" );
				props->Add( L"Four" );
			}

			{
				Gwen::Controls::Properties* props = ptree->Add( L"Item Two" );
				props->Add( L"More Items" );
				props->Add( L"To Fill" );
				props->Add( L"Out Here" );
			}

			ptree->ExpandAll();

		}
	}

	void OnFirstNameChanged( Controls::Base* pControl )
	{
		Gwen::Controls::PropertyRow* pRow = (Gwen::Controls::PropertyRow*) pControl;
		UnitPrint( Utility::Format( L"First Name Changed: %s", pRow->GetProperty()->GetPropertyValue().c_str() ) );
	}

};



DEFINE_UNIT_TEST( Properties2, L"Properties" );