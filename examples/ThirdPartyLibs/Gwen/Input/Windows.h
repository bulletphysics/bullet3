/*
	GWEN
	Copyright (c) 2011 Facepunch Studios
	See license in Gwen.h
*/
#ifndef GWEN_INPUT_WINDOWS_H
#define GWEN_INPUT_WINDOWS_H

#include "Gwen/InputHandler.h"
#include "Gwen/Gwen.h"
#include "Gwen/Controls/Canvas.h"

#include <windows.h>

namespace Gwen
{
namespace Input
{
class Windows
{
public:
	Windows()
	{
		m_Canvas = NULL;
		m_MouseX = 0;
		m_MouseY = 0;
	}

	void Initialize(Gwen::Controls::Canvas* c)
	{
		m_Canvas = c;
	}

	bool ProcessMessage(MSG msg)
	{
		if (!m_Canvas) return false;

		switch (msg.message)
		{
			case WM_MOUSEMOVE:
			{
				int x = (signed short)LOWORD(msg.lParam);
				int y = (signed short)HIWORD(msg.lParam);
				int dx = x - m_MouseX;
				int dy = y - m_MouseY;

				m_MouseX = x;
				m_MouseY = y;

				return m_Canvas->InputMouseMoved(x, y, dx, dy);
			}

			case WM_CHAR:
			{
				Gwen::UnicodeChar chr = (Gwen::UnicodeChar)msg.wParam;
				return m_Canvas->InputCharacter(chr);
			}

			case WM_MOUSEWHEEL:
			{
				return m_Canvas->InputMouseWheel((short)HIWORD(msg.wParam));
			}

			case WM_LBUTTONDOWN:
			{
				SetCapture(msg.hwnd);
				return m_Canvas->InputMouseButton(0, true);
			}

			case WM_LBUTTONUP:
			{
				ReleaseCapture();
				return m_Canvas->InputMouseButton(0, false);
			}

			case WM_RBUTTONDOWN:
			{
				SetCapture(msg.hwnd);
				return m_Canvas->InputMouseButton(1, true);
			}

			case WM_RBUTTONUP:
			{
				ReleaseCapture();
				return m_Canvas->InputMouseButton(1, false);
			}

			case WM_MBUTTONDOWN:
			{
				SetCapture(msg.hwnd);
				return m_Canvas->InputMouseButton(2, true);
			}

			case WM_MBUTTONUP:
			{
				ReleaseCapture();
				return m_Canvas->InputMouseButton(2, true);
			}

			case WM_LBUTTONDBLCLK:
			case WM_RBUTTONDBLCLK:
			case WM_MBUTTONDBLCLK:
			{
				// Filter out those events from the application
				return true;
			}

			case WM_KEYDOWN:
			case WM_KEYUP:
			{
				bool bDown = msg.message == WM_KEYDOWN;
				int iKey = -1;

				// These aren't sent by WM_CHAR when CTRL is down - but we need
				// them internally for copy and paste etc..
				if (bDown && GetKeyState(VK_CONTROL) & 0x80 && msg.wParam >= 'A' && msg.wParam <= 'Z')
				{
					Gwen::UnicodeChar chr = (Gwen::UnicodeChar)msg.wParam;
					return m_Canvas->InputCharacter(chr);
				}

				if (msg.wParam == VK_SHIFT)
					iKey = Gwen::Key::Shift;
				else if (msg.wParam == VK_RETURN)
					iKey = Gwen::Key::Return;
				else if (msg.wParam == VK_BACK)
					iKey = Gwen::Key::Backspace;
				else if (msg.wParam == VK_DELETE)
					iKey = Gwen::Key::Delete;
				else if (msg.wParam == VK_LEFT)
					iKey = Gwen::Key::Left;
				else if (msg.wParam == VK_RIGHT)
					iKey = Gwen::Key::Right;
				else if (msg.wParam == VK_TAB)
					iKey = Gwen::Key::Tab;
				else if (msg.wParam == VK_SPACE)
					iKey = Gwen::Key::Space;
				else if (msg.wParam == VK_HOME)
					iKey = Gwen::Key::Home;
				else if (msg.wParam == VK_END)
					iKey = Gwen::Key::End;
				else if (msg.wParam == VK_CONTROL)
					iKey = Gwen::Key::Control;
				else if (msg.wParam == VK_SPACE)
					iKey = Gwen::Key::Space;
				else if (msg.wParam == VK_UP)
					iKey = Gwen::Key::Up;
				else if (msg.wParam == VK_DOWN)
					iKey = Gwen::Key::Down;

				if (iKey != -1)
				{
					return m_Canvas->InputKey(iKey, bDown);
				}

				break;
			}

			default:
			{
				break;
			}
		}

		return false;
	}

protected:
	Gwen::Controls::Canvas* m_Canvas;
	int m_MouseX;
	int m_MouseY;
};
}  // namespace Input
}  // namespace Gwen
#endif
