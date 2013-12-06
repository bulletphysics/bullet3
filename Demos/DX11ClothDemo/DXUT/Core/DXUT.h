//--------------------------------------------------------------------------------------
// File: DXUT.h
//
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------
#pragma once
#ifndef DXUT_H
#define DXUT_H

#ifndef UNICODE
#error "DXUT requires a Unicode build. See the nearby comments for details"
//
// If you are using Microsoft Visual C++ .NET, under the General tab of the project 
// properties change the Character Set to 'Use Unicode Character Set'.  
//
// Windows XP and later are native Unicode so Unicode applications will perform better.  
// For Windows 98 and Windows Me support, consider using the Microsoft Layer for Unicode (MSLU).  
//
// To use MSLU, link against a set of libraries similar to this
//      /nod:kernel32.lib /nod:advapi32.lib /nod:user32.lib /nod:gdi32.lib /nod:shell32.lib /nod:comdlg32.lib /nod:version.lib /nod:mpr.lib /nod:rasapi32.lib /nod:winmm.lib /nod:winspool.lib /nod:vfw32.lib /nod:secur32.lib /nod:oleacc.lib /nod:oledlg.lib /nod:sensapi.lib UnicoWS.lib kernel32.lib advapi32.lib user32.lib gdi32.lib shell32.lib comdlg32.lib version.lib mpr.lib rasapi32.lib winmm.lib winspool.lib vfw32.lib secur32.lib oleacc.lib oledlg.lib sensapi.lib dxerr.lib dxguid.lib d3dx9d.lib d3d9.lib comctl32.lib
// and put the unicows.dll (available for download from msdn.microsoft.com) in the exe's folder.
// 
// For more details see the MSDN article titled:
// "MSLU: Develop Unicode Applications for Windows 9x Platforms with the Microsoft Layer for Unicode"
// at http://msdn.microsoft.com/msdnmag/issues/01/10/MSLU/default.aspx 
//
#endif

#include "dxsdkver.h"
#if ( _DXSDK_PRODUCT_MAJOR < 9 || _DXSDK_BUILD_MAJOR < 1455 )
#error The installed DXSDK is out of date.
#endif

#ifndef STRICT
#define STRICT
#endif

// If app hasn't choosen, set to work with Windows 98, Windows Me, Windows 2000, Windows XP and beyond
#ifndef WINVER
#define WINVER         0x0500
#endif
#ifndef _WIN32_WINDOWS
#define _WIN32_WINDOWS 0x0500
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT   0x0600
#endif

// #define DXUT_AUTOLIB to automatically include the libs needed for DXUT 
#ifdef DXUT_AUTOLIB
#pragma comment( lib, "dxerr.lib" )
#pragma comment( lib, "dxguid.lib" )
#pragma comment( lib, "d3d9.lib" )
#if defined(DEBUG) || defined(_DEBUG)
#pragma comment( lib, "d3dx9d.lib" )
#else
#pragma comment( lib, "d3dx9.lib" )
#endif
#pragma comment( lib, "winmm.lib" )
#pragma comment( lib, "comctl32.lib" )
#endif

#pragma warning( disable : 4100 ) // disable unreference formal parameter warnings for /W4 builds

// Enable extra D3D debugging in debug builds if using the debug DirectX runtime.  
// This makes D3D objects work well in the debugger watch window, but slows down 
// performance slightly.
#if defined(DEBUG) || defined(_DEBUG)
#ifndef D3D_DEBUG_INFO
#define D3D_DEBUG_INFO
#endif
#endif

// Standard Windows includes
#include <windows.h>
#include <initguid.h>
#include <assert.h>
#include <wchar.h>
#include <mmsystem.h>
#include <commctrl.h> // for InitCommonControls() 
#include <shellapi.h> // for ExtractIcon()
#include <new.h>      // for placement new
#include <shlobj.h>
#include <math.h>
#include <limits.h>
#include <stdio.h>

// CRT's memory leak detection
#if defined(DEBUG) || defined(_DEBUG)
#include <crtdbg.h>
#endif

// Direct3D9 includes
#include <d3d9.h>
#include <d3dx9.h>


// Direct3D11 includes
#include <dxgi.h>
#include <d3d11.h>
#include <d3dCompiler.h>
#include <d3dx11.h>

// XInput includes
#include <xinput.h>

// HRESULT translation for Direct3D10 and other APIs 
#include <dxerr.h>


#if defined(DEBUG) || defined(_DEBUG)
#ifndef V
#define V(x)           { hr = (x); if( FAILED(hr) ) { DXUTTrace( __FILE__, (DWORD)__LINE__, hr, L#x, true ); } }
#endif
#ifndef V_RETURN
#define V_RETURN(x)    { hr = (x); if( FAILED(hr) ) { return DXUTTrace( __FILE__, (DWORD)__LINE__, hr, L#x, true ); } }
#endif
#else
#ifndef V
#define V(x)           { hr = (x); }
#endif
#ifndef V_RETURN
#define V_RETURN(x)    { hr = (x); if( FAILED(hr) ) { return hr; } }
#endif
#endif

#ifndef SAFE_DELETE
#define SAFE_DELETE(p)       { if (p) { delete (p);     (p)=NULL; } }
#endif
#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(p) { if (p) { delete[] (p);   (p)=NULL; } }
#endif
#ifndef SAFE_RELEASE
#define SAFE_RELEASE(p)      { if (p) { (p)->Release(); (p)=NULL; } }
#endif



//--------------------------------------------------------------------------------------
// Structs
//--------------------------------------------------------------------------------------
struct DXUTD3D9DeviceSettings
{
    UINT AdapterOrdinal;
    D3DDEVTYPE DeviceType;
    D3DFORMAT AdapterFormat;
    DWORD BehaviorFlags;
    D3DPRESENT_PARAMETERS pp;
};

struct DXUTD3D11DeviceSettings
{
    UINT AdapterOrdinal;
    D3D_DRIVER_TYPE DriverType;
    UINT Output;
    DXGI_SWAP_CHAIN_DESC sd;
    UINT32 CreateFlags;
    UINT32 SyncInterval;
    DWORD PresentFlags;
    bool AutoCreateDepthStencil; // DXUT will create the depth stencil resource and view if true
    DXGI_FORMAT AutoDepthStencilFormat;
    D3D_FEATURE_LEVEL DeviceFeatureLevel;
};

enum DXUTDeviceVersion
{
    DXUT_D3D9_DEVICE,
    DXUT_D3D11_DEVICE
};

struct DXUTDeviceSettings
{
    DXUTDeviceVersion ver;
    D3D_FEATURE_LEVEL MinimumFeatureLevel;
    DXUTD3D9DeviceSettings d3d9; // only valid if ver == DXUT_D3D9_DEVICE
    DXUTD3D11DeviceSettings d3d11; // only valid if ver == DXUT_D3D11_DEVICE
};


//--------------------------------------------------------------------------------------
// Error codes
//--------------------------------------------------------------------------------------
#define DXUTERR_NODIRECT3D              MAKE_HRESULT(SEVERITY_ERROR, FACILITY_ITF, 0x0901)
#define DXUTERR_NOCOMPATIBLEDEVICES     MAKE_HRESULT(SEVERITY_ERROR, FACILITY_ITF, 0x0902)
#define DXUTERR_MEDIANOTFOUND           MAKE_HRESULT(SEVERITY_ERROR, FACILITY_ITF, 0x0903)
#define DXUTERR_NONZEROREFCOUNT         MAKE_HRESULT(SEVERITY_ERROR, FACILITY_ITF, 0x0904)
#define DXUTERR_CREATINGDEVICE          MAKE_HRESULT(SEVERITY_ERROR, FACILITY_ITF, 0x0905)
#define DXUTERR_RESETTINGDEVICE         MAKE_HRESULT(SEVERITY_ERROR, FACILITY_ITF, 0x0906)
#define DXUTERR_CREATINGDEVICEOBJECTS   MAKE_HRESULT(SEVERITY_ERROR, FACILITY_ITF, 0x0907)
#define DXUTERR_RESETTINGDEVICEOBJECTS  MAKE_HRESULT(SEVERITY_ERROR, FACILITY_ITF, 0x0908)
#define DXUTERR_DEVICEREMOVED           MAKE_HRESULT(SEVERITY_ERROR, FACILITY_ITF, 0x090A)
#define DXUTERR_NODIRECT3D11            MAKE_HRESULT(SEVERITY_ERROR, FACILITY_ITF, 0x090)

//--------------------------------------------------------------------------------------
// Callback registration 
//--------------------------------------------------------------------------------------

// General callbacks
typedef void    (CALLBACK *LPDXUTCALLBACKFRAMEMOVE)( double fTime, float fElapsedTime, void* pUserContext );
typedef void    (CALLBACK *LPDXUTCALLBACKKEYBOARD)( UINT nChar, bool bKeyDown, bool bAltDown, void* pUserContext );
typedef void    (CALLBACK *LPDXUTCALLBACKMOUSE)( bool bLeftButtonDown, bool bRightButtonDown, bool bMiddleButtonDown, bool bSideButton1Down, bool bSideButton2Down, int nMouseWheelDelta, int xPos, int yPos, void* pUserContext );
typedef LRESULT (CALLBACK *LPDXUTCALLBACKMSGPROC)( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam, bool* pbNoFurtherProcessing, void* pUserContext );
typedef void    (CALLBACK *LPDXUTCALLBACKTIMER)( UINT idEvent, void* pUserContext );
typedef bool    (CALLBACK *LPDXUTCALLBACKMODIFYDEVICESETTINGS)( DXUTDeviceSettings* pDeviceSettings, void* pUserContext );
typedef bool    (CALLBACK *LPDXUTCALLBACKDEVICEREMOVED)( void* pUserContext );

// Direct3D 9 callbacks
typedef bool    (CALLBACK *LPDXUTCALLBACKISD3D9DEVICEACCEPTABLE)( D3DCAPS9* pCaps, D3DFORMAT AdapterFormat, D3DFORMAT BackBufferFormat, bool bWindowed, void* pUserContext );
typedef HRESULT (CALLBACK *LPDXUTCALLBACKD3D9DEVICECREATED)( IDirect3DDevice9* pd3dDevice, const D3DSURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext );
typedef HRESULT (CALLBACK *LPDXUTCALLBACKD3D9DEVICERESET)( IDirect3DDevice9* pd3dDevice, const D3DSURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext );
typedef void    (CALLBACK *LPDXUTCALLBACKD3D9FRAMERENDER)( IDirect3DDevice9* pd3dDevice, double fTime, float fElapsedTime, void* pUserContext );
typedef void    (CALLBACK *LPDXUTCALLBACKD3D9DEVICELOST)( void* pUserContext );
typedef void    (CALLBACK *LPDXUTCALLBACKD3D9DEVICEDESTROYED)( void* pUserContext );

class CD3D11EnumAdapterInfo;
class CD3D11EnumDeviceInfo;
// Direct3D 11 callbacks
typedef bool    (CALLBACK *LPDXUTCALLBACKISD3D11DEVICEACCEPTABLE)( const CD3D11EnumAdapterInfo *AdapterInfo, UINT Output, const CD3D11EnumDeviceInfo *DeviceInfo, DXGI_FORMAT BackBufferFormat, bool bWindowed, void* pUserContext );
typedef HRESULT (CALLBACK *LPDXUTCALLBACKD3D11DEVICECREATED)( ID3D11Device* pd3dDevice, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext );
typedef HRESULT (CALLBACK *LPDXUTCALLBACKD3D11SWAPCHAINRESIZED)( ID3D11Device* pd3dDevice, IDXGISwapChain *pSwapChain, const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc, void* pUserContext );
typedef void    (CALLBACK *LPDXUTCALLBACKD3D11FRAMERENDER)( ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dImmediateContext, double fTime, float fElapsedTime, void* pUserContext );
typedef void    (CALLBACK *LPDXUTCALLBACKD3D11SWAPCHAINRELEASING)( void* pUserContext );
typedef void    (CALLBACK *LPDXUTCALLBACKD3D11DEVICEDESTROYED)( void* pUserContext );

// General callbacks
void WINAPI DXUTSetCallbackFrameMove( LPDXUTCALLBACKFRAMEMOVE pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackKeyboard( LPDXUTCALLBACKKEYBOARD pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackMouse( LPDXUTCALLBACKMOUSE pCallback, bool bIncludeMouseMove = false, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackMsgProc( LPDXUTCALLBACKMSGPROC pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackDeviceChanging( LPDXUTCALLBACKMODIFYDEVICESETTINGS pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackDeviceRemoved( LPDXUTCALLBACKDEVICEREMOVED pCallback, void* pUserContext = NULL );

// Direct3D 9 callbacks
void WINAPI DXUTSetCallbackD3D9DeviceAcceptable( LPDXUTCALLBACKISD3D9DEVICEACCEPTABLE pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackD3D9DeviceCreated( LPDXUTCALLBACKD3D9DEVICECREATED pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackD3D9DeviceReset( LPDXUTCALLBACKD3D9DEVICERESET pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackD3D9FrameRender( LPDXUTCALLBACKD3D9FRAMERENDER pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackD3D9DeviceLost( LPDXUTCALLBACKD3D9DEVICELOST pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackD3D9DeviceDestroyed( LPDXUTCALLBACKD3D9DEVICEDESTROYED pCallback, void* pUserContext = NULL );

// Direct3D 11 callbacks
void WINAPI DXUTSetCallbackD3D11DeviceAcceptable( LPDXUTCALLBACKISD3D11DEVICEACCEPTABLE pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackD3D11DeviceCreated( LPDXUTCALLBACKD3D11DEVICECREATED pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackD3D11SwapChainResized( LPDXUTCALLBACKD3D11SWAPCHAINRESIZED pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackD3D11FrameRender( LPDXUTCALLBACKD3D11FRAMERENDER pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackD3D11SwapChainReleasing( LPDXUTCALLBACKD3D11SWAPCHAINRELEASING pCallback, void* pUserContext = NULL );
void WINAPI DXUTSetCallbackD3D11DeviceDestroyed( LPDXUTCALLBACKD3D11DEVICEDESTROYED pCallback, void* pUserContext = NULL );


//--------------------------------------------------------------------------------------
// Initialization
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTInit( bool bParseCommandLine = true, 
                         bool bShowMsgBoxOnError = true, 
                         __in_opt WCHAR* strExtraCommandLineParams = NULL,
                         bool bThreadSafeDXUT = false );

// Choose either DXUTCreateWindow or DXUTSetWindow.  If using DXUTSetWindow, consider using DXUTStaticWndProc
HRESULT WINAPI DXUTCreateWindow( const WCHAR* strWindowTitle = L"Direct3D Window", 
                          HINSTANCE hInstance = NULL, HICON hIcon = NULL, HMENU hMenu = NULL,
                          int x = CW_USEDEFAULT, int y = CW_USEDEFAULT );
HRESULT WINAPI DXUTSetWindow( HWND hWndFocus, HWND hWndDeviceFullScreen, HWND hWndDeviceWindowed, bool bHandleMessages = true );
LRESULT CALLBACK DXUTStaticWndProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );

// Choose either DXUTCreateDevice or DXUTSetD3D*Device or DXUTCreateD3DDeviceFromSettings

HRESULT WINAPI DXUTCreateDevice(D3D_FEATURE_LEVEL reqFL,  bool bWindowed= true, int nSuggestedWidth =0, int nSuggestedHeight =0 );
HRESULT WINAPI DXUTCreateDeviceFromSettings( DXUTDeviceSettings* pDeviceSettings, bool bPreserveInput = false, bool bClipWindowToSingleAdapter = true );
HRESULT WINAPI DXUTSetD3D9Device( IDirect3DDevice9* pd3dDevice );
HRESULT WINAPI DXUTSetD3D11Device( ID3D11Device* pd3dDevice, IDXGISwapChain* pSwapChain );

// Choose either DXUTMainLoop or implement your own main loop 
HRESULT WINAPI DXUTMainLoop( HACCEL hAccel = NULL );

// If not using DXUTMainLoop consider using DXUTRender3DEnvironment
void WINAPI DXUTRender3DEnvironment();


//--------------------------------------------------------------------------------------
// Common Tasks 
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTToggleFullScreen();
HRESULT WINAPI DXUTToggleREF();
HRESULT WINAPI DXUTToggleWARP();
void    WINAPI DXUTPause( bool bPauseTime, bool bPauseRendering );
void    WINAPI DXUTSetConstantFrameTime( bool bConstantFrameTime, float fTimePerFrame = 0.0333f );
void    WINAPI DXUTSetCursorSettings( bool bShowCursorWhenFullScreen = false, bool bClipCursorWhenFullScreen = false );
void    WINAPI DXUTSetD3DVersionSupport( bool bAppCanUseD3D9 = true, bool bAppCanUseD3D11 = true );
void    WINAPI DXUTSetHotkeyHandling( bool bAltEnterToToggleFullscreen = true, bool bEscapeToQuit = true, bool bPauseToToggleTimePause = true );
void    WINAPI DXUTSetMultimonSettings( bool bAutoChangeAdapter = true );
void    WINAPI DXUTSetShortcutKeySettings( bool bAllowWhenFullscreen = false, bool bAllowWhenWindowed = true ); // Controls the Windows key, and accessibility shortcut keys
void    WINAPI DXUTSetWindowSettings( bool bCallDefWindowProc = true );
HRESULT WINAPI DXUTSetTimer( LPDXUTCALLBACKTIMER pCallbackTimer, float fTimeoutInSecs = 1.0f, UINT* pnIDEvent = NULL, void* pCallbackUserContext = NULL );
HRESULT WINAPI DXUTKillTimer( UINT nIDEvent );
void    WINAPI DXUTResetFrameworkState();
void    WINAPI DXUTShutdown( int nExitCode = 0 );
void    WINAPI DXUTSetIsInGammaCorrectMode( bool bGammaCorrect );
BOOL    WINAPI DXUTGetMSAASwapChainCreated();

//--------------------------------------------------------------------------------------
// State Retrieval  
//--------------------------------------------------------------------------------------

// Direct3D 9
IDirect3D9*              WINAPI DXUTGetD3D9Object(); // Does not addref unlike typical Get* APIs
IDirect3DDevice9*        WINAPI DXUTGetD3D9Device(); // Does not addref unlike typical Get* APIs
D3DPRESENT_PARAMETERS    WINAPI DXUTGetD3D9PresentParameters();
const D3DSURFACE_DESC*   WINAPI DXUTGetD3D9BackBufferSurfaceDesc();
const D3DCAPS9*          WINAPI DXUTGetD3D9DeviceCaps();
HRESULT                  WINAPI DXUTGetD3D9DeviceCaps( DXUTDeviceSettings* pDeviceSettings, D3DCAPS9* pCaps );
bool                     WINAPI DXUTDoesAppSupportD3D9();
bool                     WINAPI DXUTIsAppRenderingWithD3D9();


// Direct3D 11
IDXGIFactory1*            WINAPI DXUTGetDXGIFactory(); // Does not addref unlike typical Get* APIs
IDXGISwapChain*          WINAPI DXUTGetDXGISwapChain(); // Does not addref unlike typical Get* APIs
const DXGI_SURFACE_DESC* WINAPI DXUTGetDXGIBackBufferSurfaceDesc();
bool                     WINAPI DXUTIsD3D11Available(); // If D3D11 APIs are availible
ID3D11Device*			 WINAPI DXUTGetD3D11Device(); // Does not addref unlike typical Get* APIs
ID3D11DeviceContext*	 WINAPI DXUTGetD3D11DeviceContext(); // Does not addref unlike typical Get* APIs
HRESULT                  WINAPI DXUTSetupD3D11Views( ID3D11DeviceContext* pd3dDeviceContext ); // Supports immediate or deferred context
D3D_FEATURE_LEVEL	     WINAPI DXUTGetD3D11DeviceFeatureLevel(); // Returns the D3D11 devices current feature level
ID3D11RenderTargetView*  WINAPI DXUTGetD3D11RenderTargetView(); // Does not addref unlike typical Get* APIs
ID3D11DepthStencilView*  WINAPI DXUTGetD3D11DepthStencilView(); // Does not addref unlike typical Get* APIs
bool                     WINAPI DXUTDoesAppSupportD3D11();
bool                     WINAPI DXUTIsAppRenderingWithD3D11();


// General
DXUTDeviceSettings WINAPI DXUTGetDeviceSettings(); 
HINSTANCE WINAPI DXUTGetHINSTANCE();
HWND      WINAPI DXUTGetHWND();
HWND      WINAPI DXUTGetHWNDFocus();
HWND      WINAPI DXUTGetHWNDDeviceFullScreen();
HWND      WINAPI DXUTGetHWNDDeviceWindowed();
RECT      WINAPI DXUTGetWindowClientRect();
LONG      WINAPI DXUTGetWindowWidth();
LONG      WINAPI DXUTGetWindowHeight();
RECT      WINAPI DXUTGetWindowClientRectAtModeChange(); // Useful for returning to windowed mode with the same resolution as before toggle to full screen mode
RECT      WINAPI DXUTGetFullsceenClientRectAtModeChange(); // Useful for returning to full screen mode with the same resolution as before toggle to windowed mode
double    WINAPI DXUTGetTime();
float     WINAPI DXUTGetElapsedTime();
bool      WINAPI DXUTIsWindowed();
bool	  WINAPI DXUTIsInGammaCorrectMode();
float     WINAPI DXUTGetFPS();
LPCWSTR   WINAPI DXUTGetWindowTitle();
LPCWSTR   WINAPI DXUTGetFrameStats( bool bIncludeFPS = false );
LPCWSTR   WINAPI DXUTGetDeviceStats();

bool      WINAPI DXUTIsVsyncEnabled();
bool      WINAPI DXUTIsRenderingPaused();
bool      WINAPI DXUTIsTimePaused();
bool      WINAPI DXUTIsActive();
int       WINAPI DXUTGetExitCode();
bool      WINAPI DXUTGetShowMsgBoxOnError();
bool      WINAPI DXUTGetAutomation();  // Returns true if -automation parameter is used to launch the app
bool      WINAPI DXUTIsKeyDown( BYTE vKey ); // Pass a virtual-key code, ex. VK_F1, 'A', VK_RETURN, VK_LSHIFT, etc
bool      WINAPI DXUTWasKeyPressed( BYTE vKey );  // Like DXUTIsKeyDown() but return true only if the key was just pressed
bool      WINAPI DXUTIsMouseButtonDown( BYTE vButton ); // Pass a virtual-key code: VK_LBUTTON, VK_RBUTTON, VK_MBUTTON, VK_XBUTTON1, VK_XBUTTON2
HRESULT   WINAPI DXUTCreateState(); // Optional method to create DXUT's memory.  If its not called by the application it will be automatically called when needed
void      WINAPI DXUTDestroyState(); // Optional method to destroy DXUT's memory.  If its not called by the application it will be automatically called after the application exits WinMain 

//--------------------------------------------------------------------------------------
// DXUT core layer includes
//--------------------------------------------------------------------------------------
#include "DXUTmisc.h"
#include "DXUTDevice9.h"
#include "DXUTDevice11.h"




#endif




