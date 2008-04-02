//	---------------------------------------------------------------------------
//
//	@file		TwSimpleDX9.c
//	@brief		A simple example that uses AntTweakBar with DirectX9
//
//				AntTweakBar: http://www.antisphere.com/Wiki/tools:anttweakbar
//				DirectX:	 http://msdn.microsoft.com/directx
//	
//	@author		Philippe Decaudin - http://www.antisphere.com
//	@date		2006/05/20
//
//	note:		TAB=4
//
//	Compilation:
//	http://www.antisphere.com/Wiki/tools:anttweakbar:examples#twsimpledx9
//
//	---------------------------------------------------------------------------


#include <AntTweakBar.h>

#include <d3d9.h>
#include <dxerr9.h>

#include <math.h>


// Direct3D structures
IDirect3D9 *			g_D3D = NULL;
IDirect3DDevice9 *		g_D3DDev = NULL;
D3DPRESENT_PARAMETERS	g_D3Dpp;


// D3D states initialization function
void InitD3D()
{
	// Set D3D matrices
	D3DMATRIX matId = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
	g_D3DDev->SetTransform(D3DTS_WORLD, &matId);
	g_D3DDev->SetTransform(D3DTS_VIEW, &matId);
	D3DMATRIX matProj = { (float)g_D3Dpp.BackBufferHeight/g_D3Dpp.BackBufferWidth,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
	g_D3DDev->SetTransform(D3DTS_PROJECTION, &matProj);

	// Disable lighting and culling
	g_D3DDev->SetRenderState( D3DRS_LIGHTING, FALSE );
	g_D3DDev->SetRenderState( D3DRS_CULLMODE, D3DCULL_NONE );
}


// Win32 MessageProc callback
LRESULT CALLBACK MessageProc(HWND wnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	// Send event message to AntTweakBar
	if( TwEventWin32(wnd, msg, wParam, lParam) )
		return 0;	// Event has been handled by AntTweakBar

	switch( msg )
	{
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;
	case WM_SIZE:	// Window size has been changed
		// reset D3D device
		if( g_D3DDev )
		{
			g_D3Dpp.BackBufferWidth  = LOWORD(lParam);
			g_D3Dpp.BackBufferHeight = HIWORD(lParam);
			if( g_D3Dpp.BackBufferWidth>0 && g_D3Dpp.BackBufferHeight>0 )
			{
				g_D3DDev->Reset(&g_D3Dpp);
				InitD3D();	// re-initialize D3D states
			}
			// TwWindowSize has been called by TwEventWin32, so it is not necessary to call it again here.
		}
		return 0;
	default:
		return DefWindowProc(wnd, msg, wParam, lParam);
	}
}


// Main
int WINAPI WinMain(HINSTANCE instance, HINSTANCE, LPSTR, int cmdShow)
{
	// Register our window class
	WNDCLASSEX wcex = { sizeof(WNDCLASSEX), CS_CLASSDC, MessageProc, 0L, 0L, instance, NULL, NULL, NULL, NULL, "TwDX9", NULL };
	RegisterClassEx(&wcex);

	// Create a window
	RECT rect = { 0, 0, 640, 480 };
    AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, FALSE);
	HWND wnd = CreateWindow("TwDX9", "AntTweakBar simple example using DirectX9", WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, rect.right-rect.left, rect.bottom-rect.top, NULL, NULL, instance, NULL);
	if( !wnd )
	{
		MessageBox(NULL, "Cannot create window", "Error", MB_OK|MB_ICONERROR);
		return FALSE;
	}
	ShowWindow(wnd, cmdShow);
	UpdateWindow(wnd);


	// Initialize Direct3D
	g_D3D = Direct3DCreate9(D3D_SDK_VERSION);
	if( !g_D3D )
	{
		MessageBox(wnd, "Cannot initialize DirectX", "Error", MB_OK|MB_ICONERROR);
		return FALSE;
	}

	// Create a Direct3D device
	ZeroMemory( &g_D3Dpp, sizeof(D3DPRESENT_PARAMETERS) );
	g_D3Dpp.Windowed = TRUE;
	g_D3Dpp.BackBufferCount = 1;
	g_D3Dpp.SwapEffect = D3DSWAPEFFECT_FLIP;
	g_D3Dpp.BackBufferFormat = D3DFMT_UNKNOWN;
	g_D3Dpp.EnableAutoDepthStencil = TRUE;
	g_D3Dpp.AutoDepthStencilFormat = D3DFMT_D16;
	g_D3Dpp.PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE;
	HRESULT hr = g_D3D->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, wnd, D3DCREATE_MIXED_VERTEXPROCESSING, &g_D3Dpp, &g_D3DDev);
	if( FAILED(hr) )
	{
		DXTRACE_ERR_MSGBOX("Cannot create DirectX device", hr);
		g_D3D->Release();
		g_D3D = NULL;
		return FALSE;
	}

	// This example draws a moving strip;
	// create a buffer of vertices for the strip
	struct Vertex
	{
		float x, y, z;
		DWORD color;
	};
	Vertex vertices[2002];
	int numSec = 100;			 // number of strip sections
	float color[] = { 1, 0, 0 }; // strip color
	unsigned int bgColor = D3DCOLOR_ARGB(255, 40, 255, 200);	// background color

	// Init some D3D states
	InitD3D();

	// Initialize AntTweakBar
	// (note that the Direct3D device pointer must be passed to TwInit)
	if( !TwInit(TW_DIRECT3D9, g_D3DDev) )
	{
		MessageBox(wnd, TwGetLastError(), "Cannot initialize AntTweakBar", MB_OK|MB_ICONERROR);
		g_D3DDev->Release();
		g_D3DDev = NULL;
		g_D3D->Release();
		g_D3D = NULL;
		return FALSE;
	}


	// Create a tweak bar
	TwBar *bar = TwNewBar("TweakBar");

	// Add 'numSec' to 'bar': it is a modifiable (RW) variable of type TW_TYPE_INT32. Its shortcuts are [s] and [S].
	TwAddVarRW(bar, "NumSec", TW_TYPE_INT32, &numSec, " label='Strip length' min=1 max=1000 keyIncr=s keyDecr=S ");
	// Add 'color' to 'bar': it is a modifable variable of type TW_TYPE_COLOR3F (3 floats color)
	TwAddVarRW(bar, "Color", TW_TYPE_COLOR3F, &color, " label='Strip color' ");
	// Add 'bgColor' to 'bar': it is a modifable variable of type TW_TYPE_COLOR32 (32 bits color)
	TwAddVarRW(bar, "BgColor", TW_TYPE_COLOR32, &bgColor, " label='Background color' ");
	// Add 'width' and 'height' to 'bar': they are read-only (RO) variables of type TW_TYPE_INT32.
	TwAddVarRO(bar, "Width", TW_TYPE_INT32, &g_D3Dpp.BackBufferWidth, " label='wnd width' ");
	TwAddVarRO(bar, "Height", TW_TYPE_INT32, &g_D3Dpp.BackBufferHeight, " label='wnd height' ");


	// Main loop
	bool quit = false;
	while( !quit )
	{
		// Clear screen and begin draw
		g_D3DDev->Clear(0, NULL, D3DCLEAR_TARGET|D3DCLEAR_ZBUFFER, bgColor, 1.0f, 0);
		g_D3DDev->BeginScene();

		// Draw scene
		float s, t = (float)GetTickCount()/1000.0f;
		for( int i=0; i<=numSec; ++i )	// update vertices
		{
			s = (float)i/100;
			vertices[2*i+0].x = 0.2f+0.5f*cosf(2.0f*s+5.0f*t);
			vertices[2*i+1].x = vertices[2*i+0].x + (0.25f+0.1f*cosf(s+t));
			vertices[2*i+0].y = vertices[2*i+1].y = 0.7f*(0.7f+0.3f*sinf(s+t))*sinf(1.5f*s+3.0f*t);
			vertices[2*i+0].z = vertices[2*i+1].z = 0;
			s = (float)i/numSec;
			vertices[2*i+0].color = vertices[2*i+1].color = D3DCOLOR_XRGB((int)(255*color[0]*s), (int)(255*color[1]*s), (int)(255*color[2]*s));
		}
		g_D3DDev->SetFVF(D3DFVF_XYZ|D3DFVF_DIFFUSE);
		g_D3DDev->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2*numSec, vertices, sizeof(Vertex));	// draw strip

		// Draw tweak bars
		TwDraw();

		// End draw
		g_D3DDev->EndScene();

		// Present frame buffer
		g_D3DDev->Present(NULL, NULL, NULL, NULL);

		// Process windows messages
		MSG msg;
		while( PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) )
		{
			if( msg.message==WM_QUIT )
				quit = true;
			else if( !TranslateAccelerator(msg.hwnd, NULL, &msg) ) 
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
	} // End of main loop


	// Terminate AntTweakBar
	TwTerminate();

	// Release Direct3D
	g_D3DDev->Release();
	g_D3DDev = NULL;
	g_D3D->Release();
	g_D3D = NULL;

	return 0;
}

