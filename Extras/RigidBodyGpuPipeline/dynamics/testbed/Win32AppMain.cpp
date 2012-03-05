#ifdef _WINDOWS
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2010 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include <windows.h>
#include <gl/gl.h>


#include "DemoApplication.h"

#include "GLDebugDrawer.h"
#include "GLDebugFont.h"

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"

/// This Win32AppMain is shared code between all demos. 
/// The actual demo, derived from DemoApplication is created using 'createDemo', in a separate .cpp file
DemoApplication* gDemoApplication = 0;
DemoApplication*	createDemo();


// Function Declarations

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
void EnableOpenGL(HWND hWnd, HDC * hDC, HGLRC * hRC);
void DisableOpenGL(HWND hWnd, HDC hDC, HGLRC hRC);
static bool sOpenGLInitialized = false;
static int sWidth = 0;
static int sHeight =0;
static int quitRequest = 0;

// WinMain

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, 
				   LPSTR lpCmdLine, int iCmdShow)
{
	WNDCLASS wc;
	HWND hWnd;
	HDC hDC;
	HGLRC hRC;
	MSG msg;
	BOOL quit = FALSE;
	float theta = 0.0f;
	
	gDemoApplication = createDemo();
	

	// register window class
	wc.style = CS_OWNDC;
	wc.lpfnWndProc = WndProc;
	wc.cbClsExtra = 0;
	wc.cbWndExtra = 0;
	wc.hInstance = hInstance;
	wc.hIcon = LoadIcon( NULL, IDI_APPLICATION );
	wc.hCursor = LoadCursor( NULL, IDC_ARROW );
	wc.hbrBackground = (HBRUSH)GetStockObject( BLACK_BRUSH );
	wc.lpszMenuName = NULL;
	wc.lpszClassName = "BulletPhysics";
	RegisterClass( &wc );
	
	// create main window
	hWnd = CreateWindow( 
		"BulletPhysics", "Bullet Physics Sample. http://bulletphysics.org", 
		WS_CAPTION | WS_VISIBLE | WS_OVERLAPPEDWINDOW,
//		0, 0, 640, 480,
		0, 0, 1024, 768,
		NULL, NULL, hInstance, NULL );
	
	// enable OpenGL for the window
	EnableOpenGL( hWnd, &hDC, &hRC );
	
	
	GLDebugDrawer debugDraw;
	gDemoApplication->myinit();
	//gDemoApplication->reshape(1024, 768);
	gDemoApplication->initPhysics();
	if (gDemoApplication->getDynamicsWorld())
		gDemoApplication->getDynamicsWorld()->setDebugDrawer(&debugDraw);
	
	gDemoApplication->reshape(sWidth,sHeight);

	// program main loop
	while ( !quit )
	{
		
		// check for messages
		if ( PeekMessage( &msg, NULL, 0, 0, PM_REMOVE )  )
		{
			
			// handle or dispatch messages
			if ( msg.message == WM_QUIT ) 
			{
				quit = TRUE;
			} 
			else 
			{
				TranslateMessage( &msg );
				DispatchMessage( &msg );
			}
			
//			gDemoApplication->displayCallback();
			

		};
		
		// OpenGL animation code goes here
		
		glClearColor( .7f, 0.7f, 0.7f, 1.f );
		
		gDemoApplication->moveAndDisplay();


		SwapBuffers( hDC );
		
		theta += 1.0f;
	
		
	}
	


	// shutdown OpenGL
	DisableOpenGL( hWnd, hDC, hRC );
	
	// destroy the window explicitly
	DestroyWindow( hWnd );

	delete gDemoApplication;

	return msg.wParam;
	
}

// Window Procedure

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	
	

	switch (message)
	{

	case WM_SYSKEYDOWN:
		{
			if (lParam & 1<<29)
			{
				gDemoApplication->m_modifierKeys = VK_LMENU;
			}
			break;
		}
	case WM_SYSKEYUP:
		{
			if (lParam & 1<<29)
			{
				gDemoApplication->m_modifierKeys = VK_LMENU;
			} else
			{
				gDemoApplication->m_modifierKeys = 0;
			}
			
			break;
		}

		
		case WM_SIZE:													// Size Action Has Taken Place

			switch (wParam)												// Evaluate Size Action
			{
				case SIZE_MINIMIZED:									// Was Window Minimized?
				return 0;												// Return

				case SIZE_MAXIMIZED:									// Was Window Maximized?
					sWidth = LOWORD (lParam);
					sHeight = HIWORD (lParam);
					if (sOpenGLInitialized)
					{
						gDemoApplication->reshape(sWidth,sHeight);
					}
				return 0;												// Return

				case SIZE_RESTORED:										// Was Window Restored?
					sWidth = LOWORD (lParam);
					sHeight = HIWORD (lParam);
					if (sOpenGLInitialized)
					{
						gDemoApplication->reshape(sWidth,sHeight);
					}
				return 0;												// Return
			}
		break;	

	case WM_CREATE:
		return 0;
	
	case WM_MBUTTONUP:
	{
			int xPos = LOWORD(lParam); 
			int yPos = HIWORD(lParam); 
			gDemoApplication->mouseFunc(1,1,xPos,yPos);
		break;
	}
	case WM_MBUTTONDOWN:
	{
			int xPos = LOWORD(lParam); 
			int yPos = HIWORD(lParam); 
			gDemoApplication->mouseFunc(1,0,xPos,yPos);
		break;
	}

	case WM_LBUTTONUP:
	{
			int xPos = LOWORD(lParam); 
			int yPos = HIWORD(lParam); 
			gDemoApplication->mouseFunc(0,1,xPos,yPos);
		break;
	}
	case 0x020A://WM_MOUSEWHEEL:
	{

		int  zDelta = (short)HIWORD(wParam);
		int xPos = LOWORD(lParam); 
		int yPos = HIWORD(lParam); 
		if (zDelta>0)
			gDemoApplication->zoomIn();
		else
			gDemoApplication->zoomOut();
		break;
	}

	case WM_MOUSEMOVE:
		{
				int xPos = LOWORD(lParam); 
				int yPos = HIWORD(lParam); 
				gDemoApplication->mouseMotionFunc(xPos,yPos);
			break;
		}
	case WM_RBUTTONUP:
	{
			int xPos = LOWORD(lParam); 
			int yPos = HIWORD(lParam); 
			gDemoApplication->mouseFunc(2,1,xPos,yPos);
		break;
	}
	case WM_RBUTTONDOWN:
	{
			int xPos = LOWORD(lParam); 
			int yPos = HIWORD(lParam); 
			gDemoApplication->mouseFunc(2,0,xPos,yPos);
		break;
	}
	case WM_LBUTTONDOWN:
		{
				int xPos = LOWORD(lParam); 
				int yPos = HIWORD(lParam); 
				gDemoApplication->mouseFunc(0,0,xPos,yPos);
			break;
		}
/*#define WM_LBUTTONUP                    0x0202
#define WM_LBUTTONDBLCLK                0x0203
#define WM_RBUTTONDOWN                  0x0204
#define WM_RBUTTONUP                    0x0205
#define WM_RBUTTONDBLCLK                0x0206
#define WM_MBUTTONDOWN                  0x0207
#define WM_MBUTTONUP                    0x0208
#define WM_MBUTTONDBLCLK                0x0209
*/



	case WM_CLOSE:
		PostQuitMessage( 0 );
		return 0;
		
	case WM_DESTROY:
		return 0;
		
	case WM_KEYUP:
		switch ( wParam )
		{
			
		case VK_PRIOR:
		case VK_NEXT:
		case VK_END:
		case VK_HOME:
		case VK_LEFT:
		case VK_UP:
		case VK_RIGHT:
		case VK_DOWN:
			{
				if (gDemoApplication)
					gDemoApplication->specialKeyboardUp(wParam,0,0);
				return 0;
			}
			default:
				{
					gDemoApplication->keyboardUpCallback(tolower(wParam),0,0);
				}
			return DefWindowProc( hWnd, message, wParam, lParam );
		}

	case WM_KEYDOWN:
		printf("bla\n");
		switch ( wParam )
		{
		case VK_CONTROL:
		case VK_PRIOR:
		case VK_NEXT:
		case VK_END:
		case VK_HOME:
		case VK_LEFT:
		case VK_UP:
		case VK_RIGHT:
		case VK_DOWN:
			{
				if (gDemoApplication)
					gDemoApplication->specialKeyboard(wParam,0,0);
				break;
			}

		case ' ':
			{
				if (gDemoApplication)
					gDemoApplication->clientResetScene();
				break;
			}
		case 'Q':
		case VK_ESCAPE:
			{
				quitRequest = 1;
				PostQuitMessage(0);
			}
			return 0;
			
		}
		return 0;
		
	case WM_CHAR:
		if (!quitRequest)
			gDemoApplication->keyboardCallback(wParam,0,0);
		break;
	
	default:
		return DefWindowProc( hWnd, message, wParam, lParam );
			
	}
	return 0;
}

// Enable OpenGL

void EnableOpenGL(HWND hWnd, HDC * hDC, HGLRC * hRC)
{
	PIXELFORMATDESCRIPTOR pfd;
	int format;
	
	// get the device context (DC)
	*hDC = GetDC( hWnd );
	
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
	format = ChoosePixelFormat( *hDC, &pfd );
	SetPixelFormat( *hDC, format, &pfd );
	
	// create and enable the render context (RC)
	*hRC = wglCreateContext( *hDC );
	wglMakeCurrent( *hDC, *hRC );
	sOpenGLInitialized = true;
	
	
}

// Disable OpenGL

void DisableOpenGL(HWND hWnd, HDC hDC, HGLRC hRC)
{
	sOpenGLInitialized = false;

	wglMakeCurrent( NULL, NULL );
	wglDeleteContext( hRC );
	ReleaseDC( hWnd, hDC );
}

#endif //_WINDOWS
