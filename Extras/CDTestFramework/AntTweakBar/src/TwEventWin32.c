//	---------------------------------------------------------------------------
//
//	@file		TwEventWin32.c
//	@brief		Helper: 
//				translate and re-send mouse and keyboard events 
//				from Win32 message proc to AntTweakBar
//	
//	@author		Philippe Decaudin - http://www.antisphere.com
//	@date		2006/05/10
//  @license    This file is part of the AntTweakBar library.
//				Copyright © 2005, 2006 Philippe Decaudin.
//              For conditions of distribution and use, see License.txt
//
//	note:		TAB=4
//
//	---------------------------------------------------------------------------

#include <AntTweakBAr.h>

#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#include <windows.h>

// Mouse wheel support
#if !defined WM_MOUSEWHEEL
#	define   WM_MOUSEWHEEL 0x020A
#endif    // WM_MOUSEWHEEL
#if !defined WHEEL_DELTA
#define      WHEEL_DELTA 120
#endif    // WHEEL_DELTA


//	TwEventWin32 returns zero if msg has not been handled, 
//	and a non-zero value if it has been handled by the AntTweakBar library.
int TW_CALL TwEventWin32(void *wnd, unsigned int msg, unsigned int _W64 wParam, int _W64 lParam)
{
	int handled = 0;
	switch( msg ) 
	{
	case WM_MOUSEMOVE:
		// send signed! mouse coordinates
		handled = TwMouseMotion((short)LOWORD(lParam), (short)HIWORD(lParam));
		break;
	case WM_LBUTTONDOWN:
		SetCapture(wnd);
		handled = TwMouseButton(TW_MOUSE_PRESSED, TW_MOUSE_LEFT);
		break;
	case WM_LBUTTONUP:
		ReleaseCapture();
		handled = TwMouseButton(TW_MOUSE_RELEASED, TW_MOUSE_LEFT);
		break;
	case WM_MBUTTONDOWN:
		SetCapture(wnd);
		handled = TwMouseButton(TW_MOUSE_PRESSED, TW_MOUSE_MIDDLE);
		break;
	case WM_MBUTTONUP:
		ReleaseCapture();
		handled = TwMouseButton(TW_MOUSE_RELEASED, TW_MOUSE_MIDDLE);
		break;
	case WM_RBUTTONDOWN:
		SetCapture(wnd);
		handled = TwMouseButton(TW_MOUSE_PRESSED, TW_MOUSE_RIGHT);
		break;
	case WM_RBUTTONUP:
		ReleaseCapture();
		handled = TwMouseButton(TW_MOUSE_RELEASED, TW_MOUSE_RIGHT);
		break;
	case WM_CHAR:
	case WM_SYSCHAR:
		{
			int key = (int)(wParam&0xff);
			int kmod = 0;

			if( GetAsyncKeyState(VK_SHIFT)<0 )
				kmod |= TW_KMOD_SHIFT;
			if( GetAsyncKeyState(VK_CONTROL)<0 )
			{
				kmod |= TW_KMOD_CTRL;
				if( key>0 && key<27 )
					key += 'a'-1;
			}
			if( GetAsyncKeyState(VK_MENU)<0 )
				kmod |= TW_KMOD_ALT;
			if( key>0 && key<256 )
				handled = TwKeyPressed(key, kmod);
		}
		break;
	case WM_KEYDOWN:
	case WM_SYSKEYDOWN:
		{
			int kmod = 0;
			int testkp = 0;
			int k = 0;

			if( GetAsyncKeyState(VK_SHIFT)<0 )
				kmod |= TW_KMOD_SHIFT;
			if( GetAsyncKeyState(VK_CONTROL)<0 )
			{
				kmod |= TW_KMOD_CTRL;
				testkp = 1;
			}
			if( GetAsyncKeyState(VK_MENU)<0 )
			{
				kmod |= TW_KMOD_ALT;
				testkp = 1;
			}
			if( wParam>=VK_F1 && wParam<=VK_F15 )
				k = TW_KEY_F1 + ((int)wParam-VK_F1);
			else if( testkp && wParam>=VK_NUMPAD0 && wParam<=VK_NUMPAD9 )
				k = '0' + ((int)wParam-VK_NUMPAD0);
			else
			{
				switch( wParam )
				{
				case VK_UP:
					k = TW_KEY_UP;
					break;
				case VK_DOWN:
					k = TW_KEY_DOWN;
					break;
				case VK_LEFT:
					k = TW_KEY_LEFT;
					break;
				case VK_RIGHT:
					k = TW_KEY_RIGHT;
					break;
				case VK_INSERT:
					k = TW_KEY_INSERT;
					break;
				case VK_DELETE:
					k = TW_KEY_DELETE;
					break;
				case VK_PRIOR:
					k = TW_KEY_PAGE_UP;
					break;
				case VK_NEXT:
					k = TW_KEY_PAGE_DOWN;
					break;
				case VK_HOME:
					k = TW_KEY_HOME;
					break;
				case VK_END:
					k = TW_KEY_END;
					break;
				case VK_DIVIDE:
					if( testkp )
						k = '/';
					break;
				case VK_MULTIPLY:
					if( testkp )
						k = '*';
					break;
				case VK_SUBTRACT:
					if( testkp )
						k = '-';
					break;
				case VK_ADD:
					if( testkp )
						k = '+';
					break;
				case VK_DECIMAL:
					if( testkp )
						k = '.';
					break;
				default:
					if( (kmod&TW_KMOD_CTRL) && (kmod&TW_KMOD_ALT) )
						k = MapVirtualKey( (UINT)wParam, 2 ) & 0x0000FFFF;
				}
			}
			if( k!=0 )
				handled = TwKeyPressed(k, kmod);
		}
		break;
	case WM_MOUSEWHEEL:
		{
			static int s_WheelPos = 0;
            s_WheelPos += ((short)HIWORD(wParam))/WHEEL_DELTA;
			handled = TwMouseWheel(s_WheelPos);
		}
		break;
	case WM_SIZE:
		// tell the new size to TweakBar
		TwWindowSize(LOWORD(lParam), HIWORD(lParam));
		// do not set 'handled', WM_SIZE may be also processed by the calling application
		break;
	}

	return handled;
}
