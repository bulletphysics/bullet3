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


#include "Win32OpenGLRenderManager.h"

#include <windows.h>
#include <GL/gl.h>

static InternalData2* sData = 0;

struct InternalData2
{
	HWND m_hWnd;;
	int m_width;
	int m_height;
	HDC m_hDC;
	HGLRC m_hRC;
	bool m_OpenGLInitialized;
	int m_oldScreenWidth;
	int m_oldHeight;
	int m_oldBitsPerPel;
	bool m_quit;

	
	InternalData2()
	{
		m_hWnd = 0;
		m_width = 0;
		m_height = 0;
		m_hDC = 0;
		m_hRC = 0;
		m_OpenGLInitialized = false;
		m_oldScreenWidth = 0;
		m_oldHeight = 0;
		m_oldBitsPerPel = 0;
		m_quit = false;
	}
};


void Win32OpenGLWindow::enableOpenGL()
{
	
	
	
	PIXELFORMATDESCRIPTOR pfd;
	int format;
	
	// get the device context (DC)
	m_data->m_hDC = GetDC( m_data->m_hWnd );
	
	// set the pixel format for the DC
	ZeroMemory( &pfd, sizeof( pfd ) );
	pfd.nSize = sizeof( pfd );
	pfd.nVersion = 1;
	pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
	pfd.iPixelType = PFD_TYPE_RGBA;
	pfd.cColorBits = 24;
	pfd.cDepthBits = 16;
	pfd.cStencilBits = 1;
	pfd.iLayerType = PFD_MAIN_PLANE;
	format = ChoosePixelFormat( m_data->m_hDC, &pfd );
	SetPixelFormat( m_data->m_hDC, format, &pfd );
	
	// create and enable the render context (RC)
	m_data->m_hRC = wglCreateContext( m_data->m_hDC );
	wglMakeCurrent( m_data->m_hDC, m_data->m_hRC );
	m_data->m_OpenGLInitialized = true;
	
	
}


void Win32OpenGLWindow::disableOpenGL()
{
	m_data->m_OpenGLInitialized = false;

	wglMakeCurrent( NULL, NULL );
	wglDeleteContext( m_data->m_hRC );
	ReleaseDC( m_data->m_hWnd, m_data->m_hDC );
}

void Win32OpenGLWindow::pumpMessage()
{
	MSG msg;
		// check for messages
		if ( PeekMessage( &msg, NULL, 0, 0, PM_REMOVE )  )
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



LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_PAINT:
		{
			PAINTSTRUCT ps;
			BeginPaint(hWnd, &ps);
			EndPaint(hWnd, &ps);
		}
		return 0;

	case WM_ERASEBKGND:
		return 0;
	
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;

	case WM_KEYDOWN:
		{
			switch ( wParam )
			{
				case 'Q':
				case VK_ESCAPE:
					{
						PostQuitMessage(0);
					}
					return 0;
			}
			break;
		}

	case WM_SIZE:													// Size Action Has Taken Place

			switch (wParam)												// Evaluate Size Action
			{
				case SIZE_MINIMIZED:									// Was Window Minimized?
				return 0;												// Return

				case SIZE_MAXIMIZED:									// Was Window Maximized?

					sData->m_width = LOWORD (lParam);
					sData->m_height = HIWORD (lParam);
					//if (sOpenGLInitialized)
					//{
					//	//gDemoApplication->reshape(sWidth,sHeight);
					//}
					glViewport(0, 0, sData->m_width, sData->m_height);
				return 0;												// Return

				case SIZE_RESTORED:										// Was Window Restored?
					sData->m_width = LOWORD (lParam);
					sData->m_height = HIWORD (lParam);
					//if (sOpenGLInitialized)
					//{
					//	gDemoApplication->reshape(sWidth,sHeight);
					//}
					glViewport(0, 0, sData->m_width, sData->m_height);
				return 0;												// Return
			}
		break;

	default:{

			}
	};

	return DefWindowProc(hWnd, message, wParam, lParam);
}




void	Win32OpenGLWindow::init(int width,int height, bool fullscreen,int colorBitsPerPixel, void* windowHandle)
{
	// get handle to exe file
	HINSTANCE hInstance = GetModuleHandle(0);

	// create the window if we need to and we do not use the null device
	if (!windowHandle)
	{
		const char* ClassName = "DeviceWin32";

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
		wcex.hIcon = (HICON)LoadImage(hInstance, "irrlicht.ico", IMAGE_ICON, 0,0, LR_LOADFROMFILE);

		RegisterClassEx(&wcex);

		// calculate client size

		RECT clientSize;
		clientSize.top = 0;
		clientSize.left = 0;
		clientSize.right = width;
		clientSize.bottom = height;

		DWORD style = WS_POPUP;

		if (!fullscreen)
			style = WS_SYSMENU | WS_BORDER | WS_CAPTION | WS_CLIPCHILDREN | WS_CLIPSIBLINGS | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SIZEBOX;

		AdjustWindowRect(&clientSize, style, FALSE);

		m_data->m_width = clientSize.right - clientSize.left;
		m_data->m_height = clientSize.bottom - clientSize.top;

		int windowLeft = (GetSystemMetrics(SM_CXSCREEN) - m_data->m_width) / 2;
		int windowTop = (GetSystemMetrics(SM_CYSCREEN) - m_data->m_height) / 2;

		if (fullscreen)
		{
			windowLeft = 0;
			windowTop = 0;
		}

		// create window

		m_data->m_hWnd = CreateWindow( ClassName, "", style, windowLeft, windowTop,
					m_data->m_width, m_data->m_height, NULL, NULL, hInstance, NULL);

		ShowWindow(m_data->m_hWnd, SW_SHOW);
		UpdateWindow(m_data->m_hWnd);

		MoveWindow(m_data->m_hWnd, windowLeft, windowTop, m_data->m_width, m_data->m_height, TRUE);
	}
	else if (windowHandle)
	{
		// attach external window
		m_data->m_hWnd = static_cast<HWND>(windowHandle);
		RECT r;
		GetWindowRect(m_data->m_hWnd, &r);
		m_data->m_width = r.right - r.left;
		m_data->m_height = r.bottom - r.top;
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

		dm.dmPelsWidth = width;
		dm.dmPelsHeight = height;
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

	//VideoDriver = video::createOpenGLDriver(CreationParams, FileSystem, this);
	enableOpenGL();


	const wchar_t* text= L"OpenCL rigid body demo";

	DWORD dwResult;

#ifdef _WIN64
		SetWindowTextW(m_data->m_hWnd, text);
#else
		SendMessageTimeoutW(m_data->m_hWnd, WM_SETTEXT, 0,
				reinterpret_cast<LPARAM>(text),
				SMTO_ABORTIFHUNG, 2000, &dwResult);
#endif
	

}


void	Win32OpenGLWindow::switchFullScreen(bool fullscreen,int width,int height,int colorBitsPerPixel)
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
			dm.dmPelsWidth = m_data->m_width;
			dm.dmPelsHeight = m_data->m_height;
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
	} else
	{
		res = ChangeDisplaySettings(&dm, 0);
	}
}



Win32OpenGLWindow::Win32OpenGLWindow()
{
	m_data = new InternalData2();
	sData = m_data;
}

Win32OpenGLWindow::~Win32OpenGLWindow()
{
	delete m_data;
	sData = 0;
}

void	Win32OpenGLWindow::init()
{
	init(640,480,false);
}


void	Win32OpenGLWindow::exit()
{
	disableOpenGL();
	DestroyWindow(this->m_data->m_hWnd);
}





void	Win32OpenGLWindow::startRendering()
{
		pumpMessage();

		//glClearColor(1.f,0.f,0.f,1.f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);	//clear buffers
		
		//glCullFace(GL_BACK);
		//glFrontFace(GL_CCW);
		glEnable(GL_DEPTH_TEST);


		float aspect;
		//btVector3 extents;

		if (m_data->m_width > m_data->m_height) 
		{
			aspect = (float)m_data->m_width / (float)m_data->m_height;
			//extents.setValue(aspect * 1.0f, 1.0f,0);
		} else 
		{
			aspect = (float)m_data->m_height / (float)m_data->m_width;
			//extents.setValue(1.0f, aspect*1.f,0);
		}
	
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		if (m_data->m_width > m_data->m_height) 
		{
			glFrustum (-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);
		} else 
		{
			glFrustum (-1.0, 1.0, -aspect, aspect, 1.0, 10000.0);
		}
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

}


void	Win32OpenGLWindow::renderAllObjects()
{
}

void	Win32OpenGLWindow::endRendering()
{
	SwapBuffers( m_data->m_hDC );
}

float	Win32OpenGLWindow::getTimeInSeconds()
{
	return 0.f;
}

void	Win32OpenGLWindow::setDebugMessage(int x,int y,const char* message)
{
}

bool Win32OpenGLWindow::requestedExit()
{
	return m_data->m_quit;
}