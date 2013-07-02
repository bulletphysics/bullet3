/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/InputHandler.h"
#include "Gwen/Controls/Base.h"
#include "Gwen/DragAndDrop.h"
#include "Gwen/Hook.h"
#include "Gwen/Platform.h"

#define DOUBLE_CLICK_SPEED 0.5f
#define MAX_MOUSE_BUTTONS 5

using namespace Gwen;


struct Action
{
	unsigned char type;

	int x, y;
	Gwen::UnicodeChar chr;
};

static const float KeyRepeatRate = 0.03f;
static const float KeyRepeatDelay = 0.3f;

struct t_KeyData
{
	t_KeyData()
	{
		for ( int i=0; i<Gwen::Key::Count; i++ )
		{
			KeyState[i] = false;
			NextRepeat[i] = 0;
		}

		Target = NULL;
		LeftMouseDown = false;
		RightMouseDown = false;
	}

	bool KeyState[ Gwen::Key::Count ];
	float NextRepeat[ Gwen::Key::Count ];
	Controls::Base* Target;
	bool LeftMouseDown;
	bool RightMouseDown;

} KeyData;

Gwen::Point	MousePosition;

static float		g_fLastClickTime[MAX_MOUSE_BUTTONS];
static Gwen::Point	g_pntLastClickPos;

enum
{
	ACT_MOUSEMOVE,
	ACT_MOUSEBUTTON,
	ACT_CHAR,
	ACT_MOUSEWHEEL,
	ACT_KEYPRESS,
	ACT_KEYRELEASE,
	ACT_MESSAGE
};

void UpdateHoveredControl( Controls::Base* pInCanvas )
{
	Controls::Base* pHovered = pInCanvas->GetControlAt( MousePosition.x, MousePosition.y );

	if ( Gwen::HoveredControl && pHovered != Gwen::HoveredControl )
	{
		Gwen::HoveredControl->OnMouseLeave();

		pInCanvas->Redraw();
	}

	if ( pHovered != Gwen::HoveredControl )
	{
		Gwen::HoveredControl = pHovered;

		if ( Gwen::HoveredControl )
			Gwen::HoveredControl->OnMouseEnter();

		pInCanvas->Redraw();
	}

	if ( Gwen::MouseFocus && Gwen::MouseFocus->GetCanvas() == pInCanvas )
	{
		Gwen::HoveredControl = Gwen::MouseFocus;
	}

}

void FindKeyboardFocus( Controls::Base* pControl )
{
	if ( !pControl ) return;
	if ( pControl->GetKeyboardInputEnabled() )
	{
		//Make sure none of our children have keyboard focus first - todo recursive
		for (Controls::Base::List::iterator iter = pControl->Children.begin(); iter != pControl->Children.end(); ++iter)
		{
			Controls::Base* pChild = *iter;
			if ( pChild == Gwen::KeyboardFocus )
				return;
		}

		pControl->Focus();
		return;
	}

	return FindKeyboardFocus( pControl->GetParent() );
}

Gwen::Point Gwen::Input::GetMousePosition()
{
	return MousePosition;
}

void Gwen::Input::OnCanvasThink( Controls::Base* pControl )
{
	if ( Gwen::MouseFocus && !Gwen::MouseFocus->Visible() )
		Gwen::MouseFocus = NULL;
	
	if (Gwen::KeyboardFocus )
	{
		bool isVisible = Gwen::KeyboardFocus->Visible();
		bool isEnabled = KeyboardFocus->GetKeyboardInputEnabled();

		if (  !isVisible ||  !isEnabled )
			Gwen::KeyboardFocus = NULL;
	}

	if ( !KeyboardFocus ) return;
	if ( KeyboardFocus->GetCanvas() != pControl ) return;

	float fTime = Gwen::Platform::GetTimeInSeconds();

	//
	// Simulate Key-Repeats
	//
	for ( int i=0; i<Gwen::Key::Count; i++ )
	{
		if ( KeyData.KeyState[i] && KeyData.Target != KeyboardFocus )
		{
			KeyData.KeyState[i] = false;
			continue;
		}

		if ( KeyData.KeyState[i] && fTime > KeyData.NextRepeat[i] )
		{
			KeyData.NextRepeat[i] = Gwen::Platform::GetTimeInSeconds() + KeyRepeatRate;

			if ( KeyboardFocus )
			{
				KeyboardFocus->OnKeyPress( i );
			}
		}
	}
}

bool Gwen::Input::IsKeyDown( int iKey )
{
	return KeyData.KeyState[ iKey ];
}

bool Gwen::Input::IsLeftMouseDown()
{
	return KeyData.LeftMouseDown;
}

bool Gwen::Input::IsRightMouseDown()
{
	return KeyData.RightMouseDown;
}

void Gwen::Input::OnMouseMoved( Controls::Base* pCanvas, int x, int y, int /*deltaX*/, int /*deltaY*/ )
{
	MousePosition.x = x;
	MousePosition.y = y;

	UpdateHoveredControl( pCanvas );
}

bool Gwen::Input::OnMouseClicked( Controls::Base* pCanvas, int iMouseButton, bool bDown )
{
	// If we click on a control that isn't a menu we want to close
	// all the open menus. Menus are children of the canvas.
	if ( bDown && (!Gwen::HoveredControl || !Gwen::HoveredControl->IsMenuComponent()) )
	{
		pCanvas->CloseMenus();
	}

	if ( !Gwen::HoveredControl ) return false;
	if ( Gwen::HoveredControl->GetCanvas() != pCanvas ) return false;
	if ( !Gwen::HoveredControl->Visible() ) return false;
	if ( Gwen::HoveredControl == pCanvas ) return false;

	if ( iMouseButton > MAX_MOUSE_BUTTONS )
		return false;

	if ( iMouseButton == 0 )		KeyData.LeftMouseDown = bDown;
	else if ( iMouseButton == 1 )	KeyData.RightMouseDown = bDown;

	// Double click.
	// Todo: Shouldn't double click if mouse has moved significantly
	bool bIsDoubleClick = false;

	if ( bDown && 
		g_pntLastClickPos.x == MousePosition.x && 
		g_pntLastClickPos.y == MousePosition.y &&
		( Gwen::Platform::GetTimeInSeconds() - g_fLastClickTime[ iMouseButton ] ) < DOUBLE_CLICK_SPEED )
	{
		bIsDoubleClick = true;
	}

	if ( bDown && !bIsDoubleClick )
	{
		g_fLastClickTime[ iMouseButton ] = Gwen::Platform::GetTimeInSeconds();
		g_pntLastClickPos = MousePosition;
	}

	if ( bDown )
	{
		FindKeyboardFocus( Gwen::HoveredControl );
	}

	Gwen::HoveredControl->UpdateCursor();

	// This tells the child it has been touched, which
	// in turn tells its parents, who tell their parents.
	// This is basically so that Windows can pop themselves
	// to the top when one of their children have been clicked.
	if ( bDown )
		Gwen::HoveredControl->Touch();

#ifdef GWEN_HOOKSYSTEM
	if ( bDown )
	{
		if ( Hook::CallHook( &Hook::BaseHook::OnControlClicked, Gwen::HoveredControl, MousePosition.x, MousePosition.y ) )
			return true;
	}
#endif

	switch ( iMouseButton )
	{
		case 0:
			{
				if ( DragAndDrop::OnMouseButton( Gwen::HoveredControl, MousePosition.x, MousePosition.y, bDown ) )
					return true;

				if ( bIsDoubleClick )	Gwen::HoveredControl->OnMouseDoubleClickLeft( MousePosition.x, MousePosition.y );
				else					Gwen::HoveredControl->OnMouseClickLeft( MousePosition.x, MousePosition.y, bDown );
				return true;
			}

		case 1:
			{
				if ( bIsDoubleClick )	Gwen::HoveredControl->OnMouseDoubleClickRight( MousePosition.x, MousePosition.y );
				else					Gwen::HoveredControl->OnMouseClickRight( MousePosition.x, MousePosition.y, bDown );
				return true;
			}
	}

	return false;
}

bool Gwen::Input::HandleAccelerator( Controls::Base* pCanvas, Gwen::UnicodeChar chr )
{
	//Build the accelerator search string
	Gwen::UnicodeString accelString;
	if ( Gwen::Input::IsControlDown() )
		accelString += L"Ctrl + ";
	if ( Gwen::Input::IsShiftDown() )
		accelString += L"Shift + ";

	accelString += chr;

	//Debug::Msg("Accelerator string :%S\n", accelString.c_str());

	if ( Gwen::KeyboardFocus && Gwen::KeyboardFocus->HandleAccelerator( accelString ) )
		return true;

	if ( Gwen::MouseFocus && Gwen::MouseFocus->HandleAccelerator( accelString ) )
		return true;

	if ( pCanvas->HandleAccelerator( accelString ) )
		return true;

	return false;
}

bool Gwen::Input::DoSpecialKeys( Controls::Base* pCanvas, Gwen::UnicodeChar chr )
{
	if ( !Gwen::KeyboardFocus ) return false;
	if ( Gwen::KeyboardFocus->GetCanvas() != pCanvas ) return false;
	if ( !Gwen::KeyboardFocus->Visible() ) return false;
	if ( !Gwen::Input::IsControlDown() ) return false;

	if ( chr == L'C' || chr == L'c' )
	{
		Gwen::KeyboardFocus->OnCopy(NULL);
		return true;
	}

	if ( chr == L'V' || chr == L'v' )
	{
		Gwen::KeyboardFocus->OnPaste(NULL);
		return true;
	}

	if ( chr == L'X' || chr == L'x' )
	{
		Gwen::KeyboardFocus->OnCut(NULL);
		return true;
	}

	if ( chr == L'A' || chr == L'a' )
	{
		Gwen::KeyboardFocus->OnSelectAll(NULL);
		return true;
	}
	
	return false;
}

bool Gwen::Input::OnKeyEvent( Controls::Base* pCanvas, int iKey, bool bDown )
{
	if ( !Gwen::KeyboardFocus ) return false;
	if ( Gwen::KeyboardFocus->GetCanvas() != pCanvas ) return false;
	if ( !Gwen::KeyboardFocus->Visible() ) return false;

	if ( bDown )
	{
		if ( !KeyData.KeyState[ iKey ] )
		{
			KeyData.KeyState[ iKey ] = true;
			KeyData.NextRepeat[ iKey ] = Gwen::Platform::GetTimeInSeconds() + KeyRepeatDelay;
			KeyData.Target = KeyboardFocus;

			return KeyboardFocus->OnKeyPress( iKey );
		}
	}
	else
	{
		if ( KeyData.KeyState[ iKey ] )
		{
			KeyData.KeyState[ iKey ] = false;

			// BUG BUG. This causes shift left arrow in textboxes
			// to not work. What is disabling it here breaking?
			//KeyData.Target = NULL;

			return KeyboardFocus->OnKeyRelease( iKey );
		}
	}
	
	return false;
}