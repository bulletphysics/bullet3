#include "UnitTest.h"
#include "Gwen/Controls/MenuStrip.h"

using namespace Gwen;

class MenuStrip : public GUnit
{
	public:

	GWEN_CONTROL_INLINE( MenuStrip, GUnit )
	{
		Gwen::Controls::MenuStrip* menu = new Gwen::Controls::MenuStrip( this );

		{
			Gwen::Controls::MenuItem* pRoot = menu->AddItem( L"File" );
			pRoot->GetMenu()->AddItem( L"New", L"test16.png", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			pRoot->GetMenu()->AddItem( L"Load", L"test16.png", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			pRoot->GetMenu()->AddItem( L"Save", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			pRoot->GetMenu()->AddItem( L"Save As..", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			pRoot->GetMenu()->AddItem( L"Quit", GWEN_MCALL( ThisClass::MenuItemSelect ) );
		}

		{
			Gwen::Controls::MenuItem* pRoot = menu->AddItem( L"\u043F\u0438\u0440\u0430\u0442\u0441\u0442\u0432\u043E" );
			pRoot->GetMenu()->AddItem( L"\u5355\u5143\u6D4B\u8BD5", GWEN_MCALL( ThisClass::MenuItemSelect ) );
			pRoot->GetMenu()->AddItem( L"\u0111\u01A1n v\u1ECB th\u1EED nghi\u1EC7m", L"test16.png", GWEN_MCALL( ThisClass::MenuItemSelect ) );
		}

		{
			Gwen::Controls::MenuItem* pRoot = menu->AddItem( L"Submenu" );

			pRoot->GetMenu()->AddItem( "One" )->SetCheckable( true );

			{
				Gwen::Controls::MenuItem* pRootB = pRoot->GetMenu()->AddItem( "Two" );
					pRootB->GetMenu()->AddItem( "Two.One" );
					pRootB->GetMenu()->AddItem( "Two.Two" );
					pRootB->GetMenu()->AddItem( "Two.Three" );
					pRootB->GetMenu()->AddItem( "Two.Four" );
					pRootB->GetMenu()->AddItem( "Two.Five" );
					pRootB->GetMenu()->AddItem( "Two.Six" );
					pRootB->GetMenu()->AddItem( "Two.Seven" );
					pRootB->GetMenu()->AddItem( "Two.Eight" );
					pRootB->GetMenu()->AddItem( "Two.Nine", "test16.png" );
			}

			pRoot->GetMenu()->AddItem( "Three" );
			pRoot->GetMenu()->AddItem( "Four" );
			pRoot->GetMenu()->AddItem( "Five" );

			{
				Gwen::Controls::MenuItem* pRootB = pRoot->GetMenu()->AddItem( "Six" );
				pRootB->GetMenu()->AddItem( "Six.One" );
				pRootB->GetMenu()->AddItem( "Six.Two" );
				pRootB->GetMenu()->AddItem( "Six.Three" );
				pRootB->GetMenu()->AddItem( "Six.Four" );
				pRootB->GetMenu()->AddItem( "Six.Five", "test16.png" );

				{
					Gwen::Controls::MenuItem* pRootC = pRootB->GetMenu()->AddItem( "Six.Six" );
					pRootC->GetMenu()->AddItem( "Sheep" );
					pRootC->GetMenu()->AddItem( "Goose" );
					{
						Gwen::Controls::MenuItem* pRootD = pRootC->GetMenu()->AddItem( "Camel" );
						pRootD->GetMenu()->AddItem( "Eyes" );
						pRootD->GetMenu()->AddItem( "Nose" );
						{
							Gwen::Controls::MenuItem* pRootE = pRootD->GetMenu()->AddItem( "Hair" );
							pRootE->GetMenu()->AddItem( "Blonde" );
							pRootE->GetMenu()->AddItem( "Black" );
							{
								Gwen::Controls::MenuItem* pRootF = pRootE->GetMenu()->AddItem( "Red" );
								pRootF->GetMenu()->AddItem( "Light" );
								pRootF->GetMenu()->AddItem( "Medium" );
								pRootF->GetMenu()->AddItem( "Dark" );
							}
							pRootE->GetMenu()->AddItem( "Brown" );
						}
						pRootD->GetMenu()->AddItem( "Ears" );
					}
					pRootC->GetMenu()->AddItem( "Duck" );
				}

				pRootB->GetMenu()->AddItem( "Six.Seven" );
				pRootB->GetMenu()->AddItem( "Six.Eight" );
				pRootB->GetMenu()->AddItem( "Six.Nine" );
			}

			pRoot->GetMenu()->AddItem( "Seven" );
			
		}
	}

	void MenuItemSelect( Base* pControl )
	{
		Gwen::Controls::MenuItem* pMenuItem = (Gwen::Controls::MenuItem*)pControl;

		UnitPrint( Utility::Format( L"Menu Selected: %s", pMenuItem->GetText().c_str() ) );
	}

};



DEFINE_UNIT_TEST( MenuStrip, L"MenuStrip" );