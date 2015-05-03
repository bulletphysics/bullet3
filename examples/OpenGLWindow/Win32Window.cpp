/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans


#include "Win32Window.h"

#include "OpenGLInclude.h"

#include <wchar.h>
static InternalData2* sData = 0;

#include "Win32InternalWindowData.h"


enum 
{
	INTERNAL_SHIFT_MODIFIER=1,
	INTERNAL_ALT_MODIFIER=2,
	INTERNAL_CONTROL_MODIFIER=4,
};

void Win32Window::pumpMessage()
{
	MSG msg;
		// check for messages
	//'if' instead of 'while' can make mainloop smoother. 
	//@todo: use separate threads for input and rendering
		while( PeekMessage( &msg, NULL, 0, 0, PM_REMOVE )  )
		{
			
			// handle or dispatch messages
			if ( msg.message == WM_QUIT ) 
			{
				m_data->m_quit = TRUE;
			} 
			else 
			{
				TranslateMessage( &msg );
				DispatchMessage( &msg );
			}
			
//			gDemoApplication->displayCallback();
			

		};
}

int getSpecialKeyFromVirtualKeycode(int virtualKeyCode)
{
	int keycode = -1;
	if (virtualKeyCode >= 'A' &&  virtualKeyCode <= 'Z')
	{
		return virtualKeyCode+32;//todo: fix the ascii A vs a input
	}

	switch (virtualKeyCode)
	{
		case VK_RETURN: {keycode = B3G_RETURN; break; };
		case VK_F1: {keycode = B3G_F1; break;}
		case VK_F2: {keycode = B3G_F2; break;}
		case VK_F3: {keycode = B3G_F3; break;}
		case VK_F4: {keycode = B3G_F4; break;}
		case VK_F5: {keycode = B3G_F5; break;}
		case VK_F6: {keycode = B3G_F6; break;}
		case VK_F7: {keycode = B3G_F7; break;}
		case VK_F8: {keycode = B3G_F8; break;}
		case VK_F9: {keycode = B3G_F9; break;}
		case VK_F10: {keycode= B3G_F10; break;}

		//case VK_SPACE: {keycode= ' '; break;}
		
		case VK_NEXT:	{keycode= B3G_PAGE_DOWN; break;}
		case VK_PRIOR:	{keycode= B3G_PAGE_UP; break;}
		
		case VK_INSERT: {keycode= B3G_INSERT; break;}
		case VK_BACK: {keycode= B3G_BACKSPACE; break;}
		case VK_DELETE: {keycode= B3G_DELETE; break;}

		case VK_END:{keycode= B3G_END; break;}
		case VK_HOME:{keycode= B3G_HOME; break;}
		case VK_LEFT:{keycode= B3G_LEFT_ARROW; break;}
		case VK_UP:{keycode= B3G_UP_ARROW; break;}
		case VK_RIGHT:{keycode= B3G_RIGHT_ARROW; break;}
		case VK_DOWN:{keycode= B3G_DOWN_ARROW; break;}
		case VK_SHIFT:{keycode=B3G_SHIFT;break;}
		case VK_MENU:{keycode=B3G_ALT;break;}
		case VK_CONTROL:{keycode=B3G_CONTROL;break;}
		default:
			{
				//keycode = MapVirtualKey( virtualKeyCode, MAPVK_VK_TO_CHAR ) & 0x0000FFFF;
			}
	};

	return keycode;
}


int getAsciiCodeFromVirtualKeycode(int virtualKeyCode)
{
	int keycode = 0xffffffff;
	
	if (virtualKeyCode >= 'a' &&  virtualKeyCode <= 'z')
	{
		return virtualKeyCode;
	}
	
	if (virtualKeyCode >= 'A' &&  virtualKeyCode <= 'Z')
	{
		return virtualKeyCode+32;//todo: fix the ascii A vs a input
	}
	
	return keycode;
}

bool Win32Window::isModifierKeyPressed(int key)
{
	bool isPressed = false;

	switch (key)
	{
		case B3G_ALT:
		{
			isPressed = ((sData->m_internalKeyModifierFlags&INTERNAL_ALT_MODIFIER)!=0);
			break;
		};
		case B3G_SHIFT:
		{
			isPressed = ((sData->m_internalKeyModifierFlags&INTERNAL_SHIFT_MODIFIER)!=0);
			break;
		};
		case B3G_CONTROL:
		{
			isPressed = ((sData->m_internalKeyModifierFlags&INTERNAL_CONTROL_MODIFIER)!=0);
			break;
		};

		default:
		{
		}
	};
	return isPressed;//m_internalKeyModifierFlags
}


LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	//printf("msg = %d\n", message);
	switch (message)
	{
		
	case WM_PAINT:
		{
			PAINTSTRUCT ps;
			BeginPaint(hWnd, &ps);
			EndPaint(hWnd, &ps);
		}
		return 1;

	case WM_ERASEBKGND:
		return 1;
	
	case WM_CLOSE:
		if (sData)
			sData->m_quit = true;
		//PostQuitMessage(0);
		return 1;

	case WM_DESTROY:
		if (sData)
			sData->m_quit = true;
		//PostQuitMessage(0);
		return 1;

	case WM_SYSKEYUP:
	case WM_KEYUP:
	{

			int keycode = getSpecialKeyFromVirtualKeycode(wParam);
			switch (keycode)
			{
				case B3G_ALT:
				{
					sData->m_internalKeyModifierFlags&=~INTERNAL_ALT_MODIFIER;
					break;
				};
				case B3G_SHIFT:
				{
					sData->m_internalKeyModifierFlags &= ~INTERNAL_SHIFT_MODIFIER;
					break;
				};
				case B3G_CONTROL:
				{
					sData->m_internalKeyModifierFlags &=~INTERNAL_CONTROL_MODIFIER;
					break;
				};
			}

			if (keycode>=0 && sData && sData->m_keyboardCallback )
			{
				int state=0;
				(*sData->m_keyboardCallback)(keycode,state);
			}
			return 0;
		}
	case WM_CHAR:
		{
			//skip 'enter' key, it is processed in WM_KEYUP/WM_KEYDOWN 
			int keycode = getAsciiCodeFromVirtualKeycode(wParam);
			if (keycode < 0)
			{
				if (sData && sData->m_keyboardCallback && ((HIWORD(lParam) & KF_REPEAT) == 0))
				{
					int state = 1;
					(*sData->m_keyboardCallback)(wParam, state);
				}
			}
			return 0;
		}
	case WM_SYSKEYDOWN:
	case WM_KEYDOWN:
		{
			int keycode = getSpecialKeyFromVirtualKeycode(wParam);
			switch (keycode)
			{
				case B3G_ALT:
				{
					sData->m_internalKeyModifierFlags|=INTERNAL_ALT_MODIFIER;
					break;
				};
				case B3G_SHIFT:
				{
					sData->m_internalKeyModifierFlags |= INTERNAL_SHIFT_MODIFIER;
					break;
				};
				case B3G_CONTROL:
				{
					sData->m_internalKeyModifierFlags |=INTERNAL_CONTROL_MODIFIER;
					break;
				};
			}
			if (keycode>=0 && sData && sData->m_keyboardCallback)// && ((HIWORD(lParam) & KF_REPEAT) == 0))
			{
				int state = 1;
				(*sData->m_keyboardCallback)(keycode,state);
				return 1;
			}
			return 0;
		}

		case WM_MBUTTONUP:
	{
			int xPos = LOWORD(lParam); 
			int yPos = HIWORD(lParam); 
			if (sData)
			{
				sData->m_mouseMButton=0;
				sData->m_mouseXpos = xPos;
				sData->m_mouseYpos = yPos;
				if (sData && sData->m_mouseButtonCallback)
					(*sData->m_mouseButtonCallback)(1,0,xPos,yPos);
			}
		break;
	}
	case WM_MBUTTONDOWN:
	{
			int xPos = LOWORD(lParam); 
			int yPos = HIWORD(lParam); 
			if (sData)
			{
				sData->m_mouseMButton=1;
				sData->m_mouseXpos = xPos;
				sData->m_mouseYpos = yPos;
				if (sData && sData->m_mouseButtonCallback)
					(*sData->m_mouseButtonCallback)(1,1,xPos,yPos);
			}
		break;
	}

	case WM_LBUTTONUP:
	{
			int xPos = LOWORD(lParam); 
			int yPos = HIWORD(lParam); 
			if (sData)
			{
				sData->m_mouseLButton=0;
				sData->m_mouseXpos = xPos;
				sData->m_mouseYpos = yPos;
				
				if (sData && sData->m_mouseButtonCallback)
					(*sData->m_mouseButtonCallback)(0,0,xPos,yPos);

			}
		//	gDemoApplication->mouseFunc(0,1,xPos,yPos);
		break;
	}
	case WM_LBUTTONDOWN:
		{
				int xPos = LOWORD(lParam); 
				int yPos = HIWORD(lParam); 
			if (sData)
			{
				sData->m_mouseLButton=1;
				sData->m_mouseXpos = xPos;
				sData->m_mouseYpos = yPos;

				if (sData && sData->m_mouseButtonCallback)
					(*sData->m_mouseButtonCallback)(0,1,xPos,yPos);
			}
			break;
		}

	case 0x020e://WM_MOUSEWHEEL_LEFT_RIGHT
	{

		int  zDelta = (short)HIWORD(wParam);
		int xPos = LOWORD(lParam); 
		int yPos = HIWORD(lParam); 
		//m_cameraDistance -= zDelta*0.01;
		if (sData && sData->m_wheelCallback)
			(*sData->m_wheelCallback)(-float(zDelta)*0.05f,0);
		return 1;
		break;
	}
	case 0x020A://WM_MOUSEWHEEL:
	{
		
		int  zDelta = (short)HIWORD(wParam);
		int xPos = LOWORD(lParam); 
		int yPos = HIWORD(lParam); 
		//m_cameraDistance -= zDelta*0.01;
		if (sData && sData->m_wheelCallback)
			(*sData->m_wheelCallback)(0,float(zDelta)*0.05f);
		return 1;
		break;
	}

	case WM_MOUSEMOVE:
		{
				int xPos = LOWORD(lParam); 
				int yPos = HIWORD(lParam); 
				sData->m_mouseXpos = xPos;
				sData->m_mouseYpos = yPos;

				if (sData && sData->m_mouseMoveCallback)
					(*sData->m_mouseMoveCallback)(xPos,yPos);

			break;
		}
	case WM_RBUTTONUP:
	{
			int xPos = LOWORD(lParam); 
			int yPos = HIWORD(lParam); 
			sData->m_mouseRButton = 1;

			if (sData && sData->m_mouseButtonCallback)
				(*sData->m_mouseButtonCallback)(2,0,sData->m_mouseXpos,sData->m_mouseYpos);

			//gDemoApplication->mouseFunc(2,1,xPos,yPos);
		break;
	}
	case WM_RBUTTONDOWN:
	{
			int xPos = LOWORD(lParam); 
			int yPos = HIWORD(lParam); 
			sData->m_mouseRButton = 0;
			if (sData && sData->m_mouseButtonCallback)
				(*sData->m_mouseButtonCallback)(2,1,sData->m_mouseXpos,sData->m_mouseYpos);

		break;
	}
	case WM_QUIT:
		{
			return 0;
			break;
		}
	case WM_SIZE:													// Size Action Has Taken Place

			RECT clientRect;
			GetClientRect(hWnd,&clientRect);

			switch (wParam)												// Evaluate Size Action
			{

				case SIZE_MINIMIZED:									// Was Window Minimized?
				return 0;												// Return

				case SIZE_MAXIMIZED:									// Was Window Maximized?
				case SIZE_RESTORED:										// Was Window Restored?
					RECT wr;
					GetWindowRect(hWnd,&wr);
					
					sData->m_fullWindowWidth = wr.right-wr.left;
					sData->m_fullWindowHeight = wr.bottom-wr.top;//LOWORD (lParam) HIWORD (lParam);
					sData->m_openglViewportWidth = clientRect.right;
					sData->m_openglViewportHeight = clientRect.bottom;
					glViewport(0, 0, sData->m_openglViewportWidth, sData->m_openglViewportHeight);

					if (sData->m_resizeCallback)
						(*sData->m_resizeCallback)(sData->m_openglViewportWidth,sData->m_openglViewportHeight);
					//if (sOpenGLInitialized)
					//{
					//	//gDemoApplication->reshape(sWidth,sHeight);
					//}
				return 0;												// Return
			}
		break;

	default:{
				

			}
	};

	return DefWindowProc(hWnd, message, wParam, lParam);
}



void Win32Window::setWindowTitle(const char* titleChar)
{
	
	wchar_t  windowTitle[1024];
	swprintf(windowTitle, 1024, L"%hs", titleChar);

	DWORD dwResult;

#ifdef _WIN64
		SetWindowTextW(m_data->m_hWnd, windowTitle);
#else
		SendMessageTimeoutW(m_data->m_hWnd, WM_SETTEXT, 0,
				reinterpret_cast<LPARAM>(windowTitle),
				SMTO_ABORTIFHUNG, 2000, &dwResult);
#endif
}

void	Win32Window::createWindow(const b3gWindowConstructionInfo& ci)
{
	int oglViewportWidth = ci.m_width;
	int oglViewportHeight = ci.m_height;
	bool fullscreen = ci.m_fullscreen;
	int colorBitsPerPixel = ci.m_colorBitsPerPixel;
	void* windowHandle = ci.m_windowHandle;

	// get handle to exe file
	HINSTANCE hInstance = GetModuleHandle(0);


	// create the window if we need to and we do not use the null device
	if (!windowHandle)
	{
#ifdef UNICODE
		const wchar_t * ClassName = L"DeviceWin32";
		const wchar_t*  emptyString= L"";
#else
		const char* ClassName = "DeviceWin32";
		const char* emptyString = "";
#endif
		// Register Class
		WNDCLASSEX wcex;
		wcex.cbSize		= sizeof(WNDCLASSEX);
		wcex.style		= CS_HREDRAW | CS_VREDRAW;
		wcex.lpfnWndProc	= WndProc;
		wcex.cbClsExtra		= 0;
		wcex.cbWndExtra		= 0;
		wcex.hInstance		= hInstance;
		wcex.hIcon		= LoadIcon( NULL, IDI_APPLICATION ); //(HICON)LoadImage(hInstance, "bullet_ico.ico", IMAGE_ICON, 0,0, LR_LOADTRANSPARENT);//LR_LOADFROMFILE);
		wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
		wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
		wcex.lpszMenuName	= 0;
		wcex.lpszClassName	= ClassName;
		wcex.hIconSm		= 0;

		// if there is an icon, load it
//		wcex.hIcon = (HICON)LoadImage(hInstance, "bullet.ico", IMAGE_ICON, 0,0, LR_LOADFROMFILE);

		RegisterClassEx(&wcex);

		// calculate client size

		RECT clientSize;
		clientSize.top = 0;
		clientSize.left = 0;
		clientSize.right = oglViewportWidth;
		clientSize.bottom = oglViewportHeight;

		DWORD style = WS_POPUP;

		if (!fullscreen)
			style = WS_SYSMENU | WS_BORDER | WS_CAPTION | WS_CLIPCHILDREN | WS_CLIPSIBLINGS | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SIZEBOX;

		AdjustWindowRect(&clientSize, style, false);
		
		m_data->m_fullWindowWidth = clientSize.right - clientSize.left;
		m_data->m_fullWindowHeight = clientSize.bottom - clientSize.top;

		int windowLeft = (GetSystemMetrics(SM_CXSCREEN) - m_data->m_fullWindowWidth) / 2;
		int windowTop = (GetSystemMetrics(SM_CYSCREEN) - m_data->m_fullWindowHeight) / 2;

		if (fullscreen)
		{
			windowLeft = 0;
			windowTop = 0;
		}

		// create window

		m_data->m_hWnd = CreateWindow( ClassName, emptyString, style, windowLeft, windowTop,
			m_data->m_fullWindowWidth, m_data->m_fullWindowHeight,NULL, NULL, hInstance, NULL);

		
		RECT clientRect;
		GetClientRect(m_data->m_hWnd,&clientRect);



		ShowWindow(m_data->m_hWnd, SW_SHOW);
		UpdateWindow(m_data->m_hWnd);

		MoveWindow(m_data->m_hWnd, windowLeft, windowTop, m_data->m_fullWindowWidth, m_data->m_fullWindowHeight, TRUE);

		GetClientRect(m_data->m_hWnd,&clientRect);
		int w = clientRect.right-clientRect.left;
		int h = clientRect.bottom-clientRect.top;
//		printf("actual client OpenGL viewport width / height = %d, %d\n",w,h);
		m_data->m_openglViewportHeight = h;
		m_data->m_openglViewportWidth = w;
		
	}
	else if (windowHandle)
	{
		// attach external window
		m_data->m_hWnd = static_cast<HWND>(windowHandle);
		RECT r;
		GetWindowRect(m_data->m_hWnd, &r);
		m_data->m_fullWindowWidth = r.right - r.left;
		m_data->m_fullWindowHeight= r.bottom - r.top;


		//sFullScreen = false;
		//sExternalWindow = true;
	}


	if (fullscreen)
	{
		DEVMODE dm;
		memset(&dm, 0, sizeof(dm));
		dm.dmSize = sizeof(dm);
		// use default values from current setting
		EnumDisplaySettings(NULL, ENUM_CURRENT_SETTINGS, &dm);
		m_data->m_oldScreenWidth = dm.dmPelsWidth;
		m_data->m_oldHeight = dm.dmPelsHeight;
		m_data->m_oldBitsPerPel = dm.dmBitsPerPel;

		dm.dmPelsWidth = oglViewportWidth;
		dm.dmPelsHeight = oglViewportHeight;
		if (colorBitsPerPixel)
		{
			dm.dmBitsPerPel = colorBitsPerPixel;
		}
		dm.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT | DM_DISPLAYFREQUENCY;

		LONG res = ChangeDisplaySettings(&dm, CDS_FULLSCREEN);
		if (res != DISP_CHANGE_SUCCESSFUL)
		{ // try again without forcing display frequency
			dm.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;
			res = ChangeDisplaySettings(&dm, CDS_FULLSCREEN);
		}

	}


}


void	Win32Window::switchFullScreen(bool fullscreen,int width,int height,int colorBitsPerPixel)
{
	
	LONG res;
	DEVMODE dm;
	memset(&dm, 0, sizeof(dm));
	dm.dmSize = sizeof(dm);
	// use default values from current setting
	EnumDisplaySettings(NULL, ENUM_CURRENT_SETTINGS, &dm);

	dm.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT | DM_DISPLAYFREQUENCY;

	if (fullscreen && !m_data->m_oldScreenWidth)
	{
		m_data->m_oldScreenWidth = dm.dmPelsWidth;
		m_data->m_oldHeight = dm.dmPelsHeight;
		m_data->m_oldBitsPerPel = dm.dmBitsPerPel;

		if (width && height)
		{
			dm.dmPelsWidth = width;
			dm.dmPelsHeight = height;
		} else
		{
			dm.dmPelsWidth = m_data->m_fullWindowWidth;
			dm.dmPelsHeight = m_data->m_fullWindowHeight;
		}
		if (colorBitsPerPixel)
		{
			dm.dmBitsPerPel = colorBitsPerPixel;
		}
	} else
	{
		if (m_data->m_oldScreenWidth)
		{
			dm.dmPelsWidth =	m_data->m_oldScreenWidth;
			dm.dmPelsHeight=	m_data->m_oldHeight;
			dm.dmBitsPerPel =   m_data->m_oldBitsPerPel;
		}
	}

	if (fullscreen)
	{

		res = ChangeDisplaySettings(&dm, CDS_FULLSCREEN);
		if (!res)
		{
			dm.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;
			res = ChangeDisplaySettings(&dm, CDS_FULLSCREEN);
		}

		DWORD style = WS_POPUP;
		SetWindowLong(m_data->m_hWnd,  GWL_STYLE, style);

		MoveWindow(m_data->m_hWnd, 0, 0, m_data->m_fullWindowWidth, m_data->m_fullWindowHeight, TRUE);
		
		SetWindowPos(m_data->m_hWnd, NULL,0,0, (int)width, (int)height,
                         SWP_FRAMECHANGED |SWP_SHOWWINDOW);//|SWP_NOACTIVATE | SWP_NOCOPYBITS | SWP_NOOWNERZORDER | SWP_NOREPOSITION | SWP_NOZORDER);


	} else
	{
		res = ChangeDisplaySettings(&dm, 0);

		DWORD style = WS_SYSMENU | WS_BORDER | WS_CAPTION | WS_CLIPCHILDREN | WS_CLIPSIBLINGS | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SIZEBOX;
		SetWindowLong(m_data->m_hWnd,  GWL_STYLE, style);

		SetWindowPos(m_data->m_hWnd, NULL,0,0, (int)width, (int)height,
                         SWP_FRAMECHANGED |SWP_SHOWWINDOW);
		//|SWP_NOACTIVATE | SWP_NOCOPYBITS | SWP_NOOWNERZORDER | SWP_NOREPOSITION | SWP_NOZORDER);

	}


}



Win32Window::Win32Window()
{
	m_data = new InternalData2();
	sData = m_data;
	
}

Win32Window::~Win32Window()
{
	setKeyboardCallback(0);
	setMouseMoveCallback(0);
	setMouseButtonCallback(0);
	setWheelCallback(0);
	setResizeCallback(0);
	
	sData = 0;
	delete m_data;
	
}

void Win32Window::setRenderCallback( b3RenderCallback renderCallback)
{

}

void	Win32Window::closeWindow()
{
	setKeyboardCallback(0);
	setMouseMoveCallback(0);
	setMouseButtonCallback(0);
	setWheelCallback(0);
	setResizeCallback(0);
	setRenderCallback(0);
	
	
	DestroyWindow(this->m_data->m_hWnd);
}

void Win32Window::getMouseCoordinates(int& x, int& y)
{
	x = m_data->m_mouseXpos;
	y = m_data->m_mouseYpos;

}

void Win32Window::runMainLoop()
{

}


void	Win32Window::startRendering()
{
		pumpMessage();

//		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);	//clear buffers
		
		//glCullFace(GL_BACK);
		//glFrontFace(GL_CCW);
	//	glEnable(GL_DEPTH_TEST);



}


void	Win32Window::renderAllObjects()
{
}

void	Win32Window::endRendering()
{
	SwapBuffers( m_data->m_hDC );
}

float	Win32Window::getTimeInSeconds()
{
	return 0.f;
}

void	Win32Window::setDebugMessage(int x,int y,const char* message)
{
}

void	Win32Window::setRequestExit()
{
	m_data->m_quit = true;
}
bool Win32Window::requestedExit() const
{
	return m_data->m_quit;
}

void Win32Window::setWheelCallback(b3WheelCallback wheelCallback)
{
	m_data->m_wheelCallback = wheelCallback;
}

void Win32Window::setMouseMoveCallback(b3MouseMoveCallback	mouseCallback)
{
	m_data->m_mouseMoveCallback = mouseCallback;
}

void Win32Window::setMouseButtonCallback(b3MouseButtonCallback	mouseCallback)
{
	m_data->m_mouseButtonCallback = mouseCallback;
}

void Win32Window::setResizeCallback(b3ResizeCallback	resizeCallback)
{
	m_data->m_resizeCallback = resizeCallback;
	if (m_data->m_resizeCallback)
		(*m_data->m_resizeCallback)(m_data->m_openglViewportWidth,m_data->m_openglViewportHeight);
}

void Win32Window::setKeyboardCallback( b3KeyboardCallback	keyboardCallback)
{
	m_data->m_keyboardCallback = keyboardCallback;
	
}

b3KeyboardCallback	Win32Window::getKeyboardCallback()
{
	return m_data->m_keyboardCallback;
}

b3MouseMoveCallback Win32Window::getMouseMoveCallback()
{
	return m_data->m_mouseMoveCallback;
}
b3MouseButtonCallback Win32Window::getMouseButtonCallback()
{
	return m_data->m_mouseButtonCallback;
}
b3ResizeCallback Win32Window::getResizeCallback()
{
	return m_data->m_resizeCallback;
}
b3WheelCallback Win32Window::getWheelCallback()
{
	return m_data->m_wheelCallback;
}

	
