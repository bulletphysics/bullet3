//--------------------------------------------------------------------------------------
// File: DXUT.cpp
//
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------
#include "DXUT.h"
#define DXUT_MIN_WINDOW_SIZE_X 200
#define DXUT_MIN_WINDOW_SIZE_Y 200
#define DXUT_COUNTER_STAT_LENGTH 2048
#undef min // use __min instead inside this source file
#undef max // use __max instead inside this source file

#ifndef ARRAYSIZE
extern "C++" // templates cannot be declared to have 'C' linkage
template <typename T, size_t N>
char (*RtlpNumberOf( UNALIGNED T (&)[N] ))[N];

#define RTL_NUMBER_OF_V2(A) (sizeof(*RtlpNumberOf(A)))
#define ARRAYSIZE(A)    RTL_NUMBER_OF_V2(A)
#endif

//--------------------------------------------------------------------------------------
// Thread safety
//--------------------------------------------------------------------------------------
CRITICAL_SECTION    g_cs;
bool                g_bThreadSafe = true;


//--------------------------------------------------------------------------------------
// Automatically enters & leaves the CS upon object creation/deletion
//--------------------------------------------------------------------------------------
class DXUTLock
{
public:
    inline DXUTLock()  { if( g_bThreadSafe ) EnterCriticalSection( &g_cs ); }
    inline ~DXUTLock() { if( g_bThreadSafe ) LeaveCriticalSection( &g_cs ); }
};

//--------------------------------------------------------------------------------------
// Helper macros to build member functions that access member variables with thread safety
//--------------------------------------------------------------------------------------
#define SET_ACCESSOR( x, y )       inline void Set##y( x t )   { DXUTLock l; m_state.m_##y = t; };
#define GET_ACCESSOR( x, y )       inline x Get##y()           { DXUTLock l; return m_state.m_##y; };
#define GET_SET_ACCESSOR( x, y )   SET_ACCESSOR( x, y ) GET_ACCESSOR( x, y )

#define SETP_ACCESSOR( x, y )      inline void Set##y( x* t )  { DXUTLock l; m_state.m_##y = *t; };
#define GETP_ACCESSOR( x, y )      inline x* Get##y()          { DXUTLock l; return &m_state.m_##y; };
#define GETP_SETP_ACCESSOR( x, y ) SETP_ACCESSOR( x, y ) GETP_ACCESSOR( x, y )


//--------------------------------------------------------------------------------------
// Stores timer callback info
//--------------------------------------------------------------------------------------
struct DXUT_TIMER
{
    LPDXUTCALLBACKTIMER pCallbackTimer;
    void* pCallbackUserContext;
    float fTimeoutInSecs;
    float fCountdown;
    bool bEnabled;
    UINT nID;
};



//--------------------------------------------------------------------------------------
// Stores DXUT state and data access is done with thread safety (if g_bThreadSafe==true)
//--------------------------------------------------------------------------------------
class DXUTState
{
protected:
    struct STATE
    {
        // D3D9 specific
        IDirect3D9*             m_D3D9;                    // the main D3D9 object
        IDirect3DDevice9*       m_D3D9Device;              // the D3D9 rendering device
        DXUTDeviceSettings*     m_CurrentDeviceSettings;   // current device settings
        D3DSURFACE_DESC         m_BackBufferSurfaceDesc9;  // D3D9 back buffer surface description
        D3DCAPS9                m_Caps;                    // D3D caps for current device

        // D3D11 specific
        IDXGIFactory1*           m_DXGIFactory;             // DXGI Factory object
        IDXGIAdapter1*           m_DXGIAdapter;            // The DXGI adapter object for the D3D11 device
        IDXGIOutput**           m_DXGIOutputArray;        // The array of output obj for the D3D11 adapter obj
        UINT                    m_DXGIOutputArraySize;    // Number of elements in m_D3D11OutputArray
        IDXGISwapChain*         m_DXGISwapChain;          // the D3D11 swapchain
        DXGI_SURFACE_DESC       m_BackBufferSurfaceDescDXGI; // D3D11 back buffer surface description
        bool                    m_RenderingOccluded;       // Rendering is occluded by another window
        bool                    m_DoNotStoreBufferSize;    // Do not store the buffer size on WM_SIZE messages

        // D3D11 specific
        bool                    m_D3D11Available;          // if true, then D3D11 is available 
        ID3D11Device*			m_D3D11Device;             // the D3D11 rendering device
        ID3D11DeviceContext*	m_D3D11DeviceContext;	   // the D3D11 immediate device context
        D3D_FEATURE_LEVEL		m_D3D11FeatureLevel;	   // the D3D11 feature level that this device supports
        ID3D11Texture2D*        m_D3D11DepthStencil;       // the D3D11 depth stencil texture (optional)
        ID3D11DepthStencilView* m_D3D11DepthStencilView;   // the D3D11 depth stencil view (optional)
        ID3D11RenderTargetView* m_D3D11RenderTargetView;   // the D3D11 render target view
        ID3D11RasterizerState*  m_D3D11RasterizerState;    // the D3D11 Rasterizer state

        // General
        HWND  m_HWNDFocus;                  // the main app focus window
        HWND  m_HWNDDeviceFullScreen;       // the main app device window in fullscreen mode
        HWND  m_HWNDDeviceWindowed;         // the main app device window in windowed mode
        HMONITOR m_AdapterMonitor;          // the monitor of the adapter 
        HMENU m_Menu;                       // handle to menu

        UINT m_FullScreenBackBufferWidthAtModeChange;  // back buffer size of fullscreen mode right before switching to windowed mode.  Used to restore to same resolution when toggling back to fullscreen
        UINT m_FullScreenBackBufferHeightAtModeChange; // back buffer size of fullscreen mode right before switching to windowed mode.  Used to restore to same resolution when toggling back to fullscreen
        UINT m_WindowBackBufferWidthAtModeChange;  // back buffer size of windowed mode right before switching to fullscreen mode.  Used to restore to same resolution when toggling back to windowed mode
        UINT m_WindowBackBufferHeightAtModeChange; // back buffer size of windowed mode right before switching to fullscreen mode.  Used to restore to same resolution when toggling back to windowed mode
        DWORD m_WindowedStyleAtModeChange;  // window style
        WINDOWPLACEMENT m_WindowedPlacement;// record of windowed HWND position/show state/etc
        bool  m_TopmostWhileWindowed;       // if true, the windowed HWND is topmost 
        bool  m_Minimized;                  // if true, the HWND is minimized
        bool  m_Maximized;                  // if true, the HWND is maximized
        bool  m_MinimizedWhileFullscreen;   // if true, the HWND is minimized due to a focus switch away when fullscreen mode
        bool  m_IgnoreSizeChange;           // if true, DXUT won't reset the device upon HWND size change

        double m_Time;                      // current time in seconds
        double m_AbsoluteTime;              // absolute time in seconds
        float m_ElapsedTime;                // time elapsed since last frame

        HINSTANCE m_HInstance;              // handle to the app instance
        double m_LastStatsUpdateTime;       // last time the stats were updated
        DWORD m_LastStatsUpdateFrames;      // frames count since last time the stats were updated
        float m_FPS;                        // frames per second
        int   m_CurrentFrameNumber;         // the current frame number
        HHOOK m_KeyboardHook;               // handle to keyboard hook
        bool  m_AllowShortcutKeysWhenFullscreen; // if true, when fullscreen enable shortcut keys (Windows keys, StickyKeys shortcut, ToggleKeys shortcut, FilterKeys shortcut) 
        bool  m_AllowShortcutKeysWhenWindowed;   // if true, when windowed enable shortcut keys (Windows keys, StickyKeys shortcut, ToggleKeys shortcut, FilterKeys shortcut) 
        bool  m_AllowShortcutKeys;          // if true, then shortcut keys are currently disabled (Windows key, etc)
        bool  m_CallDefWindowProc;          // if true, DXUTStaticWndProc will call DefWindowProc for unhandled messages. Applications rendering to a dialog may need to set this to false.
        STICKYKEYS m_StartupStickyKeys;     // StickyKey settings upon startup so they can be restored later
        TOGGLEKEYS m_StartupToggleKeys;     // ToggleKey settings upon startup so they can be restored later
        FILTERKEYS m_StartupFilterKeys;     // FilterKey settings upon startup so they can be restored later

        bool  m_AppSupportsD3D9Override;    // true if app sets via DXUTSetD3DVersionSupport()
        bool  m_AppSupportsD3D11Override;   // true if app sets via DXUTSetD3DVersionSupport()
        bool  m_UseD3DVersionOverride;      // true if the app ever calls DXUTSetD3DVersionSupport()

        bool  m_HandleEscape;               // if true, then DXUT will handle escape to quit
        bool  m_HandleAltEnter;             // if true, then DXUT will handle alt-enter to toggle fullscreen
        bool  m_HandlePause;                // if true, then DXUT will handle pause to toggle time pausing
        bool  m_ShowMsgBoxOnError;          // if true, then msgboxes are displayed upon errors
        bool  m_NoStats;                    // if true, then DXUTGetFrameStats() and DXUTGetDeviceStats() will return blank strings
        bool  m_ClipCursorWhenFullScreen;   // if true, then DXUT will keep the cursor from going outside the window when full screen
        bool  m_ShowCursorWhenFullScreen;   // if true, then DXUT will show a cursor when full screen
        bool  m_ConstantFrameTime;          // if true, then elapsed frame time will always be 0.05f seconds which is good for debugging or automated capture
        float m_TimePerFrame;               // the constant time per frame in seconds, only valid if m_ConstantFrameTime==true
        bool  m_WireframeMode;              // if true, then D3DRS_FILLMODE==D3DFILL_WIREFRAME else D3DRS_FILLMODE==D3DFILL_SOLID 
        bool  m_AutoChangeAdapter;          // if true, then the adapter will automatically change if the window is different monitor
        bool  m_WindowCreatedWithDefaultPositions; // if true, then CW_USEDEFAULT was used and the window should be moved to the right adapter
        int   m_ExitCode;                   // the exit code to be returned to the command line

        bool  m_DXUTInited;                 // if true, then DXUTInit() has succeeded
        bool  m_WindowCreated;              // if true, then DXUTCreateWindow() or DXUTSetWindow() has succeeded
        bool  m_DeviceCreated;              // if true, then DXUTCreateDevice() or DXUTSetD3D*Device() has succeeded

        bool  m_DXUTInitCalled;             // if true, then DXUTInit() was called
        bool  m_WindowCreateCalled;         // if true, then DXUTCreateWindow() or DXUTSetWindow() was called
        bool  m_DeviceCreateCalled;         // if true, then DXUTCreateDevice() or DXUTSetD3D*Device() was called

        bool  m_DeviceObjectsCreated;       // if true, then DeviceCreated callback has been called (if non-NULL)
        bool  m_DeviceObjectsReset;         // if true, then DeviceReset callback has been called (if non-NULL)
        bool  m_InsideDeviceCallback;       // if true, then the framework is inside an app device callback
        bool  m_InsideMainloop;             // if true, then the framework is inside the main loop
        bool  m_Active;                     // if true, then the app is the active top level window
        bool  m_TimePaused;                 // if true, then time is paused
        bool  m_RenderingPaused;            // if true, then rendering is paused
        int   m_PauseRenderingCount;        // pause rendering ref count
        int   m_PauseTimeCount;             // pause time ref count
        bool  m_DeviceLost;                 // if true, then the device is lost and needs to be reset
        bool  m_NotifyOnMouseMove;          // if true, include WM_MOUSEMOVE in mousecallback
        bool  m_Automation;                 // if true, automation is enabled
        bool  m_InSizeMove;                 // if true, app is inside a WM_ENTERSIZEMOVE
        UINT  m_TimerLastID;                // last ID of the DXUT timer
        bool  m_MessageWhenD3D11NotAvailable; 
        
        D3D_FEATURE_LEVEL  m_OverrideForceFeatureLevel; // if != -1, then overrid to use a featurelevel
        WCHAR m_ScreenShotName[256];        // command line screen shot name
        bool m_SaveScreenShot;              // command line save screen shot
        bool m_ExitAfterScreenShot;         // command line exit after screen shot
        
        int   m_OverrideForceAPI;               // if != -1, then override to use this Direct3D API version
        int   m_OverrideAdapterOrdinal;         // if != -1, then override to use this adapter ordinal
        bool  m_OverrideWindowed;               // if true, then force to start windowed
        int   m_OverrideOutput;                 // if != -1, then override to use the particular output on the adapter
        bool  m_OverrideFullScreen;             // if true, then force to start full screen
        int   m_OverrideStartX;                 // if != -1, then override to this X position of the window
        int   m_OverrideStartY;                 // if != -1, then override to this Y position of the window
        int   m_OverrideWidth;                  // if != 0, then override to this width
        int   m_OverrideHeight;                 // if != 0, then override to this height
        bool  m_OverrideForceHAL;               // if true, then force to HAL device (failing if one doesn't exist)
        bool  m_OverrideForceREF;               // if true, then force to REF device (failing if one doesn't exist)
        bool  m_OverrideConstantFrameTime;      // if true, then force to constant frame time
        float m_OverrideConstantTimePerFrame;   // the constant time per frame in seconds if m_OverrideConstantFrameTime==true
        int   m_OverrideQuitAfterFrame;         // if != 0, then it will force the app to quit after that frame
        int   m_OverrideForceVsync;             // if == 0, then it will force the app to use D3DPRESENT_INTERVAL_IMMEDIATE, if == 1 force use of D3DPRESENT_INTERVAL_DEFAULT
        bool  m_OverrideRelaunchMCE;            // if true, then force relaunch of MCE at exit
        bool  m_AppCalledWasKeyPressed;         // true if the app ever calls DXUTWasKeyPressed().  Allows for optimzation
        bool  m_ReleasingSwapChain;		        // if true, the app is releasing its swapchain
        bool  m_IsInGammaCorrectMode;		    // Tell DXUTRes and DXUTMisc that we are in gamma correct mode

        LPDXUTCALLBACKMODIFYDEVICESETTINGS      m_ModifyDeviceSettingsFunc;     // modify Direct3D device settings callback
        LPDXUTCALLBACKDEVICEREMOVED             m_DeviceRemovedFunc;            // Direct3D device removed callback
        LPDXUTCALLBACKFRAMEMOVE                 m_FrameMoveFunc;                // frame move callback
        LPDXUTCALLBACKKEYBOARD                  m_KeyboardFunc;                 // keyboard callback
        LPDXUTCALLBACKMOUSE                     m_MouseFunc;                    // mouse callback
        LPDXUTCALLBACKMSGPROC                   m_WindowMsgFunc;                // window messages callback

        LPDXUTCALLBACKISD3D9DEVICEACCEPTABLE    m_IsD3D9DeviceAcceptableFunc;   // D3D9 is device acceptable callback
        LPDXUTCALLBACKD3D9DEVICECREATED         m_D3D9DeviceCreatedFunc;        // D3D9 device created callback
        LPDXUTCALLBACKD3D9DEVICERESET           m_D3D9DeviceResetFunc;          // D3D9 device reset callback
        LPDXUTCALLBACKD3D9DEVICELOST            m_D3D9DeviceLostFunc;           // D3D9 device lost callback
        LPDXUTCALLBACKD3D9DEVICEDESTROYED       m_D3D9DeviceDestroyedFunc;      // D3D9 device destroyed callback
        LPDXUTCALLBACKD3D9FRAMERENDER           m_D3D9FrameRenderFunc;          // D3D9 frame render callback

        LPDXUTCALLBACKISD3D11DEVICEACCEPTABLE   m_IsD3D11DeviceAcceptableFunc;  // D3D11 is device acceptable callback
        LPDXUTCALLBACKD3D11DEVICECREATED        m_D3D11DeviceCreatedFunc;       // D3D11 device created callback
        LPDXUTCALLBACKD3D11SWAPCHAINRESIZED     m_D3D11SwapChainResizedFunc;    // D3D11 SwapChain reset callback
        LPDXUTCALLBACKD3D11SWAPCHAINRELEASING   m_D3D11SwapChainReleasingFunc;  // D3D11 SwapChain lost callback
        LPDXUTCALLBACKD3D11DEVICEDESTROYED      m_D3D11DeviceDestroyedFunc;     // D3D11 device destroyed callback
        LPDXUTCALLBACKD3D11FRAMERENDER          m_D3D11FrameRenderFunc;         // D3D11 frame render callback


        void* m_ModifyDeviceSettingsFuncUserContext;     // user context for modify Direct3D device settings callback
        void* m_DeviceRemovedFuncUserContext;            // user context for Direct3D device removed callback
        void* m_FrameMoveFuncUserContext;                // user context for frame move callback
        void* m_KeyboardFuncUserContext;                 // user context for keyboard callback
        void* m_MouseFuncUserContext;                    // user context for mouse callback
        void* m_WindowMsgFuncUserContext;                // user context for window messages callback

        void* m_IsD3D9DeviceAcceptableFuncUserContext;   // user context for is D3D9 device acceptable callback
        void* m_D3D9DeviceCreatedFuncUserContext;        // user context for D3D9 device created callback
        void* m_D3D9DeviceResetFuncUserContext;          // user context for D3D9 device reset callback
        void* m_D3D9DeviceLostFuncUserContext;           // user context for D3D9 device lost callback
        void* m_D3D9DeviceDestroyedFuncUserContext;      // user context for D3D9 device destroyed callback
        void* m_D3D9FrameRenderFuncUserContext;          // user context for D3D9 frame render callback

        void* m_IsD3D11DeviceAcceptableFuncUserContext;  // user context for is D3D11 device acceptable callback
        void* m_D3D11DeviceCreatedFuncUserContext;       // user context for D3D11 device created callback
        void* m_D3D11SwapChainResizedFuncUserContext;    // user context for D3D11 SwapChain resized callback
        void* m_D3D11SwapChainReleasingFuncUserContext;  // user context for D3D11 SwapChain releasing callback
        void* m_D3D11DeviceDestroyedFuncUserContext;     // user context for D3D11 device destroyed callback
        void* m_D3D11FrameRenderFuncUserContext;         // user context for D3D11 frame render callback

        bool m_Keys[256];                                // array of key state
        bool m_LastKeys[256];                            // array of last key state
        bool m_MouseButtons[5];                          // array of mouse states

        CGrowableArray<DXUT_TIMER>*  m_TimerList;        // list of DXUT_TIMER structs
        WCHAR m_StaticFrameStats[256];                   // static part of frames stats 
        WCHAR m_FPSStats[64];                            // fps stats
        WCHAR m_FrameStats[256];                         // frame stats (fps, width, etc)
        WCHAR m_DeviceStats[256];                        // device stats (description, device type, etc)
        WCHAR m_WindowTitle[256];                        // window title
    };

    STATE m_state;

public:
    DXUTState()  { Create(); }
    ~DXUTState() { Destroy(); }

    void                                                                                    Create()
    {
        g_bThreadSafe = true;
        InitializeCriticalSectionAndSpinCount( &g_cs, 1000 );

        ZeroMemory( &m_state, sizeof( STATE ) );
        m_state.m_OverrideStartX = -1;
        m_state.m_OverrideStartY = -1;
        m_state.m_OverrideForceFeatureLevel = (D3D_FEATURE_LEVEL)0;
        m_state.m_ScreenShotName[0] = 0;
        m_state.m_SaveScreenShot = false;
        m_state.m_ExitAfterScreenShot = false;
        m_state.m_OverrideForceAPI = -1;
        m_state.m_OverrideAdapterOrdinal = -1;
        m_state.m_OverrideOutput = -1;
        m_state.m_OverrideForceVsync = -1;
        m_state.m_AutoChangeAdapter = true;
        m_state.m_ShowMsgBoxOnError = true;
        m_state.m_AllowShortcutKeysWhenWindowed = true;
        m_state.m_Active = true;
        m_state.m_CallDefWindowProc = true;
        m_state.m_HandleEscape = true;
        m_state.m_HandleAltEnter = true;
        m_state.m_HandlePause = true;
        m_state.m_IsInGammaCorrectMode = true;
		m_state.m_FPS = 1.0f;
        m_state.m_MessageWhenD3D11NotAvailable = true;
    }

    void Destroy()
    {
        SAFE_DELETE( m_state.m_TimerList );
        DXUTShutdown();
        DeleteCriticalSection( &g_cs );
    }

    // Macros to define access functions for thread safe access into m_state 
    GET_SET_ACCESSOR( DXUTDeviceSettings*, CurrentDeviceSettings );

    // D3D9 specific
    GET_SET_ACCESSOR( IDirect3D9*, D3D9 );
    GET_SET_ACCESSOR( IDirect3DDevice9*, D3D9Device );
    GETP_SETP_ACCESSOR( D3DSURFACE_DESC, BackBufferSurfaceDesc9 );
    GETP_SETP_ACCESSOR( D3DCAPS9, Caps );

    // D3D11 specific
    GET_SET_ACCESSOR( IDXGIFactory1*, DXGIFactory );
    GET_SET_ACCESSOR( IDXGIAdapter1*, DXGIAdapter );
    GET_SET_ACCESSOR( IDXGIOutput**, DXGIOutputArray );
    GET_SET_ACCESSOR( UINT, DXGIOutputArraySize );
    GET_SET_ACCESSOR( IDXGISwapChain*, DXGISwapChain );
    GETP_SETP_ACCESSOR( DXGI_SURFACE_DESC, BackBufferSurfaceDescDXGI );
    GET_SET_ACCESSOR( bool, RenderingOccluded );
    GET_SET_ACCESSOR( bool, DoNotStoreBufferSize );

    // D3D11 specific
    GET_SET_ACCESSOR( bool, D3D11Available );
    GET_SET_ACCESSOR( ID3D11Device*, D3D11Device );
    GET_SET_ACCESSOR( ID3D11DeviceContext*, D3D11DeviceContext );
    GET_SET_ACCESSOR( D3D_FEATURE_LEVEL, D3D11FeatureLevel );
    GET_SET_ACCESSOR( ID3D11Texture2D*, D3D11DepthStencil );
    GET_SET_ACCESSOR( ID3D11DepthStencilView*, D3D11DepthStencilView );   
    GET_SET_ACCESSOR( ID3D11RenderTargetView*, D3D11RenderTargetView );
    GET_SET_ACCESSOR( ID3D11RasterizerState*, D3D11RasterizerState );


    GET_SET_ACCESSOR( HWND, HWNDFocus );
    GET_SET_ACCESSOR( HWND, HWNDDeviceFullScreen );
    GET_SET_ACCESSOR( HWND, HWNDDeviceWindowed );
    GET_SET_ACCESSOR( HMONITOR, AdapterMonitor );
    GET_SET_ACCESSOR( HMENU, Menu );   


    GET_SET_ACCESSOR( UINT, FullScreenBackBufferWidthAtModeChange );
    GET_SET_ACCESSOR( UINT, FullScreenBackBufferHeightAtModeChange );
    GET_SET_ACCESSOR( UINT, WindowBackBufferWidthAtModeChange );
    GET_SET_ACCESSOR( UINT, WindowBackBufferHeightAtModeChange );
    GETP_SETP_ACCESSOR( WINDOWPLACEMENT, WindowedPlacement );
    GET_SET_ACCESSOR( DWORD, WindowedStyleAtModeChange );
    GET_SET_ACCESSOR( bool, TopmostWhileWindowed );
    GET_SET_ACCESSOR( bool, Minimized );
    GET_SET_ACCESSOR( bool, Maximized );
    GET_SET_ACCESSOR( bool, MinimizedWhileFullscreen );
    GET_SET_ACCESSOR( bool, IgnoreSizeChange );   

    GET_SET_ACCESSOR( double, Time );
    GET_SET_ACCESSOR( double, AbsoluteTime );
    GET_SET_ACCESSOR( float, ElapsedTime );

    GET_SET_ACCESSOR( HINSTANCE, HInstance );
    GET_SET_ACCESSOR( double, LastStatsUpdateTime );   
    GET_SET_ACCESSOR( DWORD, LastStatsUpdateFrames );   
    GET_SET_ACCESSOR( float, FPS );    
    GET_SET_ACCESSOR( int, CurrentFrameNumber );
    GET_SET_ACCESSOR( HHOOK, KeyboardHook );
    GET_SET_ACCESSOR( bool, AllowShortcutKeysWhenFullscreen );
    GET_SET_ACCESSOR( bool, AllowShortcutKeysWhenWindowed );
    GET_SET_ACCESSOR( bool, AllowShortcutKeys );
    GET_SET_ACCESSOR( bool, CallDefWindowProc );
    GET_SET_ACCESSOR( STICKYKEYS, StartupStickyKeys );
    GET_SET_ACCESSOR( TOGGLEKEYS, StartupToggleKeys );
    GET_SET_ACCESSOR( FILTERKEYS, StartupFilterKeys );

    GET_SET_ACCESSOR( bool, AppSupportsD3D9Override );
    GET_SET_ACCESSOR( bool, AppSupportsD3D11Override );
    GET_SET_ACCESSOR( bool, UseD3DVersionOverride );

    GET_SET_ACCESSOR( bool, HandleEscape );
    GET_SET_ACCESSOR( bool, HandleAltEnter );
    GET_SET_ACCESSOR( bool, HandlePause );
    GET_SET_ACCESSOR( bool, ShowMsgBoxOnError );
    GET_SET_ACCESSOR( bool, NoStats );
    GET_SET_ACCESSOR( bool, ClipCursorWhenFullScreen );   
    GET_SET_ACCESSOR( bool, ShowCursorWhenFullScreen );
    GET_SET_ACCESSOR( bool, ConstantFrameTime );
    GET_SET_ACCESSOR( float, TimePerFrame );
    GET_SET_ACCESSOR( bool, WireframeMode );   
    GET_SET_ACCESSOR( bool, AutoChangeAdapter );
    GET_SET_ACCESSOR( bool, WindowCreatedWithDefaultPositions );
    GET_SET_ACCESSOR( int, ExitCode );

    GET_SET_ACCESSOR( bool, DXUTInited );
    GET_SET_ACCESSOR( bool, WindowCreated );
    GET_SET_ACCESSOR( bool, DeviceCreated );
    GET_SET_ACCESSOR( bool, DXUTInitCalled );
    GET_SET_ACCESSOR( bool, WindowCreateCalled );
    GET_SET_ACCESSOR( bool, DeviceCreateCalled );
    GET_SET_ACCESSOR( bool, InsideDeviceCallback );
    GET_SET_ACCESSOR( bool, InsideMainloop );
    GET_SET_ACCESSOR( bool, DeviceObjectsCreated );
    GET_SET_ACCESSOR( bool, DeviceObjectsReset );
    GET_SET_ACCESSOR( bool, Active );
    GET_SET_ACCESSOR( bool, RenderingPaused );
    GET_SET_ACCESSOR( bool, TimePaused );
    GET_SET_ACCESSOR( int, PauseRenderingCount );
    GET_SET_ACCESSOR( int, PauseTimeCount );
    GET_SET_ACCESSOR( bool, DeviceLost );
    GET_SET_ACCESSOR( bool, NotifyOnMouseMove );
    GET_SET_ACCESSOR( bool, Automation );
    GET_SET_ACCESSOR( bool, InSizeMove );
    GET_SET_ACCESSOR( UINT, TimerLastID );
    GET_SET_ACCESSOR( bool, MessageWhenD3D11NotAvailable );
    GET_SET_ACCESSOR( bool, AppCalledWasKeyPressed );

    GET_SET_ACCESSOR( D3D_FEATURE_LEVEL, OverrideForceFeatureLevel );
    GET_ACCESSOR( WCHAR*, ScreenShotName );
    GET_SET_ACCESSOR( bool, SaveScreenShot );
    GET_SET_ACCESSOR( bool, ExitAfterScreenShot );

    
    GET_SET_ACCESSOR( int, OverrideForceAPI );
    GET_SET_ACCESSOR( int, OverrideAdapterOrdinal );
    GET_SET_ACCESSOR( bool, OverrideWindowed );
    GET_SET_ACCESSOR( int, OverrideOutput );
    GET_SET_ACCESSOR( bool, OverrideFullScreen );
    GET_SET_ACCESSOR( int, OverrideStartX );
    GET_SET_ACCESSOR( int, OverrideStartY );
    GET_SET_ACCESSOR( int, OverrideWidth );
    GET_SET_ACCESSOR( int, OverrideHeight );
    GET_SET_ACCESSOR( bool, OverrideForceHAL );
    GET_SET_ACCESSOR( bool, OverrideForceREF );
    GET_SET_ACCESSOR( bool, OverrideConstantFrameTime );
    GET_SET_ACCESSOR( float, OverrideConstantTimePerFrame );
    GET_SET_ACCESSOR( int, OverrideQuitAfterFrame );
    GET_SET_ACCESSOR( int, OverrideForceVsync );
    GET_SET_ACCESSOR( bool, OverrideRelaunchMCE );
    GET_SET_ACCESSOR( bool, ReleasingSwapChain );
    GET_SET_ACCESSOR( bool, IsInGammaCorrectMode );
    
    GET_SET_ACCESSOR( LPDXUTCALLBACKMODIFYDEVICESETTINGS, ModifyDeviceSettingsFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKDEVICEREMOVED, DeviceRemovedFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKFRAMEMOVE, FrameMoveFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKKEYBOARD, KeyboardFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKMOUSE, MouseFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKMSGPROC, WindowMsgFunc );

    GET_SET_ACCESSOR( LPDXUTCALLBACKISD3D9DEVICEACCEPTABLE, IsD3D9DeviceAcceptableFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKD3D9DEVICECREATED, D3D9DeviceCreatedFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKD3D9DEVICERESET, D3D9DeviceResetFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKD3D9DEVICELOST, D3D9DeviceLostFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKD3D9DEVICEDESTROYED, D3D9DeviceDestroyedFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKD3D9FRAMERENDER, D3D9FrameRenderFunc );

    GET_SET_ACCESSOR( LPDXUTCALLBACKISD3D11DEVICEACCEPTABLE, IsD3D11DeviceAcceptableFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKD3D11DEVICECREATED, D3D11DeviceCreatedFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKD3D11SWAPCHAINRESIZED, D3D11SwapChainResizedFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKD3D11SWAPCHAINRELEASING, D3D11SwapChainReleasingFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKD3D11DEVICEDESTROYED, D3D11DeviceDestroyedFunc );
    GET_SET_ACCESSOR( LPDXUTCALLBACKD3D11FRAMERENDER, D3D11FrameRenderFunc );

    GET_SET_ACCESSOR( void*, ModifyDeviceSettingsFuncUserContext );
    GET_SET_ACCESSOR( void*, DeviceRemovedFuncUserContext );
    GET_SET_ACCESSOR( void*, FrameMoveFuncUserContext );
    GET_SET_ACCESSOR( void*, KeyboardFuncUserContext );
    GET_SET_ACCESSOR( void*, MouseFuncUserContext );
    GET_SET_ACCESSOR( void*, WindowMsgFuncUserContext );

    GET_SET_ACCESSOR( void*, IsD3D9DeviceAcceptableFuncUserContext );
    GET_SET_ACCESSOR( void*, D3D9DeviceCreatedFuncUserContext );
    GET_SET_ACCESSOR( void*, D3D9DeviceResetFuncUserContext );
    GET_SET_ACCESSOR( void*, D3D9DeviceLostFuncUserContext );
    GET_SET_ACCESSOR( void*, D3D9DeviceDestroyedFuncUserContext );
    GET_SET_ACCESSOR( void*, D3D9FrameRenderFuncUserContext );

    GET_SET_ACCESSOR( void*, IsD3D11DeviceAcceptableFuncUserContext );
    GET_SET_ACCESSOR( void*, D3D11DeviceCreatedFuncUserContext );
    GET_SET_ACCESSOR( void*, D3D11DeviceDestroyedFuncUserContext );
    GET_SET_ACCESSOR( void*, D3D11SwapChainResizedFuncUserContext );
    GET_SET_ACCESSOR( void*, D3D11SwapChainReleasingFuncUserContext );
    GET_SET_ACCESSOR( void*, D3D11FrameRenderFuncUserContext );

    GET_SET_ACCESSOR( CGrowableArray<DXUT_TIMER>*, TimerList );
    GET_ACCESSOR( bool*, Keys );
    GET_ACCESSOR( bool*, LastKeys );
    GET_ACCESSOR( bool*, MouseButtons );
    GET_ACCESSOR( WCHAR*, StaticFrameStats );
    GET_ACCESSOR( WCHAR*, FPSStats );
    GET_ACCESSOR( WCHAR*, FrameStats );
    GET_ACCESSOR( WCHAR*, DeviceStats );    
    GET_ACCESSOR( WCHAR*, WindowTitle );
};


//--------------------------------------------------------------------------------------
// Global state 
//--------------------------------------------------------------------------------------
DXUTState*          g_pDXUTState = NULL;

HRESULT WINAPI DXUTCreateState()
{
    if( g_pDXUTState == NULL )
    {
        g_pDXUTState = new DXUTState;
        if( NULL == g_pDXUTState )
            return E_OUTOFMEMORY;
    }
    return S_OK;
}

void WINAPI DXUTDestroyState()
{
    SAFE_DELETE( g_pDXUTState );
}

class DXUTMemoryHelper
{
public:
    DXUTMemoryHelper()  { DXUTCreateState(); }
    ~DXUTMemoryHelper() { DXUTDestroyState(); }
};


DXUTState& GetDXUTState()
{
    // This class will auto create the memory when its first accessed and delete it after the program exits WinMain.
    // However the application can also call DXUTCreateState() & DXUTDestroyState() independantly if its wants 
    static DXUTMemoryHelper memory;

    return *g_pDXUTState;
}


//--------------------------------------------------------------------------------------
// Internal functions forward declarations
//--------------------------------------------------------------------------------------
void DXUTParseCommandLine( __inout WCHAR* strCommandLine, 
                           bool bIgnoreFirstCommand = true );
bool DXUTIsNextArg( __inout WCHAR*& strCmdLine, 
                    __inout WCHAR* strArg );
bool DXUTGetCmdParam( __inout WCHAR*& strCmdLine, 
                      __inout_ecount(MAX_PATH) WCHAR* strFlag );
void DXUTAllowShortcutKeys( bool bAllowKeys );
void DXUTUpdateStaticFrameStats();
void DXUTUpdateFrameStats();

LRESULT CALLBACK DXUTStaticWndProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );
void DXUTHandleTimers();
void DXUTDisplayErrorMessage( HRESULT hr );
int DXUTMapButtonToArrayIndex( BYTE vButton );

HRESULT DXUTChangeDevice( DXUTDeviceSettings* pNewDeviceSettings,
                          IDirect3DDevice9* pd3d9DeviceFromApp,
                          ID3D11Device* pd3d11DeviceFromApp,
                          bool bForceRecreate,
                          bool bClipWindowToSingleAdapter );

bool DXUTCanDeviceBeReset( DXUTDeviceSettings* pOldDeviceSettings,
                           DXUTDeviceSettings* pNewDeviceSettings,
                           IDirect3DDevice9* pd3d9DeviceFromApp,
                           ID3D11Device* pd3d11DeviceFromApp );


HRESULT DXUTDelayLoadDXGI();
HRESULT DXUTDelayLoadD3D9();
HRESULT DXUTSnapDeviceSettingsToEnumDevice( DXUTDeviceSettings* pDeviceSettings, bool forceEnum, D3D_FEATURE_LEVEL forceFL = D3D_FEATURE_LEVEL(0) );
void DXUTUpdateDeviceSettingsWithOverrides( DXUTDeviceSettings* pDeviceSettings );
void DXUTCheckForDXGIFullScreenSwitch();
void DXUTResizeDXGIBuffers( UINT Width, UINT Height, BOOL bFullscreen );
void DXUTCheckForDXGIBufferChange();
void DXUTCheckForWindowSizeChange();
void DXUTCheckForWindowChangingMonitors();
void DXUTCleanup3DEnvironment( bool bReleaseSettings );
HMONITOR DXUTGetMonitorFromAdapter( DXUTDeviceSettings* pDeviceSettings );
HRESULT DXUTGetAdapterOrdinalFromMonitor( HMONITOR hMonitor, UINT* pAdapterOrdinal );
HRESULT DXUTGetOutputOrdinalFromMonitor( HMONITOR hMonitor, UINT* pOutputOrdinal );
HRESULT DXUTHandleDeviceRemoved();
void DXUTUpdateBackBufferDesc();
void DXUTSetupCursor();

// Direct3D 9
HRESULT DXUTCreate3DEnvironment9( IDirect3DDevice9* pd3dDeviceFromApp );
HRESULT DXUTReset3DEnvironment9();
void DXUTRender3DEnvironment9();
void DXUTCleanup3DEnvironment9( bool bReleaseSettings = true );
HRESULT DXUTSetD3D9DeviceCursor( IDirect3DDevice9* pd3dDevice, HCURSOR hCursor, bool bAddWatermark );
void DXUTUpdateD3D9DeviceStats( D3DDEVTYPE DeviceType, DWORD BehaviorFlags,
                                D3DADAPTER_IDENTIFIER9* pAdapterIdentifier );
HRESULT DXUTFindD3D9AdapterFormat( UINT AdapterOrdinal, D3DDEVTYPE DeviceType, D3DFORMAT BackBufferFormat,
                                   BOOL Windowed, D3DFORMAT* pAdapterFormat );

// Direct3D 11
HRESULT DXUTCreateD3D11Views( ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dDeviceContext, DXUTDeviceSettings* pDeviceSettings );
HRESULT DXUTCreate3DEnvironment11( ID3D11Device* pd3dDeviceFromApp );
HRESULT DXUTReset3DEnvironment11();
void DXUTRender3DEnvironment11();
void DXUTCleanup3DEnvironment11( bool bReleaseSettings = true );
void DXUTUpdateD3D11DeviceStats( D3D_DRIVER_TYPE DeviceType, DXGI_ADAPTER_DESC* pAdapterDesc );


//--------------------------------------------------------------------------------------
// Internal helper functions 
//--------------------------------------------------------------------------------------
bool DXUTIsD3D9( DXUTDeviceSettings* pDeviceSettings )                          { return (pDeviceSettings && pDeviceSettings->ver == DXUT_D3D9_DEVICE ); };
bool DXUTIsCurrentDeviceD3D9()                                                  { DXUTDeviceSettings* pDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();  return DXUTIsD3D9(pDeviceSettings); };
UINT DXUTGetBackBufferWidthFromDS( DXUTDeviceSettings* pNewDeviceSettings )     
{ 
    if( DXUTIsD3D9( pNewDeviceSettings ) )
        return pNewDeviceSettings->d3d9.pp.BackBufferWidth; 
    else
        return pNewDeviceSettings->d3d11.sd.BufferDesc.Width; 
}
UINT DXUTGetBackBufferHeightFromDS( DXUTDeviceSettings* pNewDeviceSettings )    
{ 
    if( DXUTIsD3D9(pNewDeviceSettings) )
        return pNewDeviceSettings->d3d9.pp.BackBufferHeight;
    else
        return pNewDeviceSettings->d3d11.sd.BufferDesc.Height; 
}
bool DXUTGetIsWindowedFromDS( DXUTDeviceSettings* pNewDeviceSettings )          
{ 
    if (!pNewDeviceSettings) 
        return true; 
    
    if( DXUTIsD3D9(pNewDeviceSettings) )
        return pNewDeviceSettings->d3d9.pp.Windowed ? true : false;
    else
        return pNewDeviceSettings->d3d11.sd.Windowed ? true : false; 
}


//--------------------------------------------------------------------------------------
// External state access functions
//--------------------------------------------------------------------------------------
BOOL WINAPI DXUTGetMSAASwapChainCreated() { 
    DXUTDeviceSettings *psettings = GetDXUTState().GetCurrentDeviceSettings();
    if (psettings->ver == DXUT_D3D11_DEVICE) {
        return psettings->d3d11.sd.SampleDesc.Count > 1;
    }else if (psettings->ver == DXUT_D3D9_DEVICE) {
        return (psettings->d3d9.pp.MultiSampleType >= D3DMULTISAMPLE_2_SAMPLES);
    }
    else return false;
}
IDirect3DDevice9* WINAPI DXUTGetD3D9Device()               { return GetDXUTState().GetD3D9Device(); }
const D3DSURFACE_DESC* WINAPI DXUTGetD3D9BackBufferSurfaceDesc() { return GetDXUTState().GetBackBufferSurfaceDesc9(); }
const D3DCAPS9* WINAPI DXUTGetD3D9DeviceCaps()             { return GetDXUTState().GetCaps(); }
ID3D11Device* WINAPI DXUTGetD3D11Device()                  { return GetDXUTState().GetD3D11Device(); }
D3D_FEATURE_LEVEL	 WINAPI DXUTGetD3D11DeviceFeatureLevel() { return GetDXUTState().GetD3D11FeatureLevel(); }
ID3D11DeviceContext* WINAPI DXUTGetD3D11DeviceContext()    { return GetDXUTState().GetD3D11DeviceContext(); }
IDXGISwapChain* WINAPI DXUTGetDXGISwapChain()              { return GetDXUTState().GetDXGISwapChain(); }
ID3D11RenderTargetView* WINAPI DXUTGetD3D11RenderTargetView() { return GetDXUTState().GetD3D11RenderTargetView(); }
ID3D11DepthStencilView* WINAPI DXUTGetD3D11DepthStencilView() { return GetDXUTState().GetD3D11DepthStencilView(); }
const DXGI_SURFACE_DESC* WINAPI DXUTGetDXGIBackBufferSurfaceDesc() { return GetDXUTState().GetBackBufferSurfaceDescDXGI(); }
HINSTANCE WINAPI DXUTGetHINSTANCE()                        { return GetDXUTState().GetHInstance(); }
HWND WINAPI DXUTGetHWND()                                  { return DXUTIsWindowed() ? GetDXUTState().GetHWNDDeviceWindowed() : GetDXUTState().GetHWNDDeviceFullScreen(); }
HWND WINAPI DXUTGetHWNDFocus()                             { return GetDXUTState().GetHWNDFocus(); }
HWND WINAPI DXUTGetHWNDDeviceFullScreen()                  { return GetDXUTState().GetHWNDDeviceFullScreen(); }
HWND WINAPI DXUTGetHWNDDeviceWindowed()                    { return GetDXUTState().GetHWNDDeviceWindowed(); }
RECT WINAPI DXUTGetWindowClientRect()                      { RECT rc; GetClientRect( DXUTGetHWND(), &rc ); return rc; }
LONG WINAPI DXUTGetWindowWidth()                           { RECT rc = DXUTGetWindowClientRect(); return ((LONG)rc.right - rc.left); }
LONG WINAPI DXUTGetWindowHeight()                          { RECT rc = DXUTGetWindowClientRect(); return ((LONG)rc.bottom - rc.top); }
RECT WINAPI DXUTGetWindowClientRectAtModeChange()          { RECT rc = { 0, 0, GetDXUTState().GetWindowBackBufferWidthAtModeChange(), GetDXUTState().GetWindowBackBufferHeightAtModeChange() }; return rc; }
RECT WINAPI DXUTGetFullsceenClientRectAtModeChange()       { RECT rc = { 0, 0, GetDXUTState().GetFullScreenBackBufferWidthAtModeChange(), GetDXUTState().GetFullScreenBackBufferHeightAtModeChange() }; return rc; }
double WINAPI DXUTGetTime()                                { return GetDXUTState().GetTime(); }
float WINAPI DXUTGetElapsedTime()                          { return GetDXUTState().GetElapsedTime(); }
float WINAPI DXUTGetFPS()                                  { return GetDXUTState().GetFPS(); }
LPCWSTR WINAPI DXUTGetWindowTitle()                        { return GetDXUTState().GetWindowTitle(); }
LPCWSTR WINAPI DXUTGetDeviceStats()                        { return GetDXUTState().GetDeviceStats(); }
bool WINAPI DXUTIsRenderingPaused()                        { return GetDXUTState().GetPauseRenderingCount() > 0; }
bool WINAPI DXUTIsTimePaused()                             { return GetDXUTState().GetPauseTimeCount() > 0; }
bool WINAPI DXUTIsActive()                                 { return GetDXUTState().GetActive(); }
int WINAPI DXUTGetExitCode()                               { return GetDXUTState().GetExitCode(); }
bool WINAPI DXUTGetShowMsgBoxOnError()                     { return GetDXUTState().GetShowMsgBoxOnError(); }
bool WINAPI DXUTGetAutomation()                            { return GetDXUTState().GetAutomation(); }
bool WINAPI DXUTIsWindowed()                               { return DXUTGetIsWindowedFromDS( GetDXUTState().GetCurrentDeviceSettings() ); }
bool WINAPI DXUTIsInGammaCorrectMode()                     { return GetDXUTState().GetIsInGammaCorrectMode(); }
IDirect3D9* WINAPI DXUTGetD3D9Object()                     { DXUTDelayLoadD3D9(); return GetDXUTState().GetD3D9(); }
IDXGIFactory1* WINAPI DXUTGetDXGIFactory()                  { DXUTDelayLoadDXGI(); return GetDXUTState().GetDXGIFactory(); }
bool WINAPI DXUTIsD3D11Available()                         { DXUTDelayLoadDXGI(); return GetDXUTState().GetD3D11Available(); }
bool WINAPI DXUTIsAppRenderingWithD3D9()                   { return (GetDXUTState().GetD3D9Device() != NULL); }
bool WINAPI DXUTIsAppRenderingWithD3D11()                  { return (GetDXUTState().GetD3D11Device() != NULL); }

//--------------------------------------------------------------------------------------
// External callback setup functions
//--------------------------------------------------------------------------------------

// General callbacks
void WINAPI DXUTSetCallbackDeviceChanging( LPDXUTCALLBACKMODIFYDEVICESETTINGS pCallback, void* pUserContext )                  { GetDXUTState().SetModifyDeviceSettingsFunc( pCallback ); GetDXUTState().SetModifyDeviceSettingsFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackDeviceRemoved( LPDXUTCALLBACKDEVICEREMOVED pCallback, void* pUserContext )                          { GetDXUTState().SetDeviceRemovedFunc( pCallback ); GetDXUTState().SetDeviceRemovedFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackFrameMove( LPDXUTCALLBACKFRAMEMOVE pCallback, void* pUserContext )                                  { GetDXUTState().SetFrameMoveFunc( pCallback );  GetDXUTState().SetFrameMoveFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackKeyboard( LPDXUTCALLBACKKEYBOARD pCallback, void* pUserContext )                                    { GetDXUTState().SetKeyboardFunc( pCallback );  GetDXUTState().SetKeyboardFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackMouse( LPDXUTCALLBACKMOUSE pCallback, bool bIncludeMouseMove, void* pUserContext )                  { GetDXUTState().SetMouseFunc( pCallback ); GetDXUTState().SetNotifyOnMouseMove( bIncludeMouseMove );  GetDXUTState().SetMouseFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackMsgProc( LPDXUTCALLBACKMSGPROC pCallback, void* pUserContext )                                      { GetDXUTState().SetWindowMsgFunc( pCallback );  GetDXUTState().SetWindowMsgFuncUserContext( pUserContext ); }

// Direct3D 9 callbacks
void WINAPI DXUTSetCallbackD3D9DeviceAcceptable( LPDXUTCALLBACKISD3D9DEVICEACCEPTABLE pCallback, void* pUserContext )          { GetDXUTState().SetIsD3D9DeviceAcceptableFunc( pCallback ); GetDXUTState().SetIsD3D9DeviceAcceptableFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackD3D9DeviceCreated( LPDXUTCALLBACKD3D9DEVICECREATED pCallback, void* pUserContext )                  { GetDXUTState().SetD3D9DeviceCreatedFunc( pCallback ); GetDXUTState().SetD3D9DeviceCreatedFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackD3D9DeviceReset( LPDXUTCALLBACKD3D9DEVICERESET pCallback, void* pUserContext )                      { GetDXUTState().SetD3D9DeviceResetFunc( pCallback );  GetDXUTState().SetD3D9DeviceResetFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackD3D9DeviceLost( LPDXUTCALLBACKD3D9DEVICELOST pCallback, void* pUserContext )                        { GetDXUTState().SetD3D9DeviceLostFunc( pCallback );  GetDXUTState().SetD3D9DeviceLostFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackD3D9DeviceDestroyed( LPDXUTCALLBACKD3D9DEVICEDESTROYED pCallback, void* pUserContext )              { GetDXUTState().SetD3D9DeviceDestroyedFunc( pCallback );  GetDXUTState().SetD3D9DeviceDestroyedFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackD3D9FrameRender( LPDXUTCALLBACKD3D9FRAMERENDER pCallback, void* pUserContext )                      { GetDXUTState().SetD3D9FrameRenderFunc( pCallback );  GetDXUTState().SetD3D9FrameRenderFuncUserContext( pUserContext ); }
void DXUTGetCallbackD3D9DeviceAcceptable( LPDXUTCALLBACKISD3D9DEVICEACCEPTABLE* ppCallback, void** ppUserContext )             { *ppCallback = GetDXUTState().GetIsD3D9DeviceAcceptableFunc(); *ppUserContext = GetDXUTState().GetIsD3D9DeviceAcceptableFuncUserContext(); }

// Direct3D 11 callbacks
void WINAPI DXUTSetCallbackD3D11DeviceAcceptable( LPDXUTCALLBACKISD3D11DEVICEACCEPTABLE pCallback, void* pUserContext )        { GetDXUTState().SetIsD3D11DeviceAcceptableFunc( pCallback ); GetDXUTState().SetIsD3D11DeviceAcceptableFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackD3D11DeviceCreated( LPDXUTCALLBACKD3D11DEVICECREATED pCallback, void* pUserContext )                { GetDXUTState().SetD3D11DeviceCreatedFunc( pCallback ); GetDXUTState().SetD3D11DeviceCreatedFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackD3D11SwapChainResized( LPDXUTCALLBACKD3D11SWAPCHAINRESIZED pCallback, void* pUserContext )          { GetDXUTState().SetD3D11SwapChainResizedFunc( pCallback );  GetDXUTState().SetD3D11SwapChainResizedFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackD3D11FrameRender( LPDXUTCALLBACKD3D11FRAMERENDER pCallback, void* pUserContext )                    { GetDXUTState().SetD3D11FrameRenderFunc( pCallback );  GetDXUTState().SetD3D11FrameRenderFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackD3D11SwapChainReleasing( LPDXUTCALLBACKD3D11SWAPCHAINRELEASING pCallback, void* pUserContext )      { GetDXUTState().SetD3D11SwapChainReleasingFunc( pCallback );  GetDXUTState().SetD3D11SwapChainReleasingFuncUserContext( pUserContext ); }
void WINAPI DXUTSetCallbackD3D11DeviceDestroyed( LPDXUTCALLBACKD3D11DEVICEDESTROYED pCallback, void* pUserContext )            { GetDXUTState().SetD3D11DeviceDestroyedFunc( pCallback );  GetDXUTState().SetD3D11DeviceDestroyedFuncUserContext( pUserContext ); }
void DXUTGetCallbackD3D11DeviceAcceptable( LPDXUTCALLBACKISD3D11DEVICEACCEPTABLE* ppCallback, void** ppUserContext )           { *ppCallback = GetDXUTState().GetIsD3D11DeviceAcceptableFunc(); *ppUserContext = GetDXUTState().GetIsD3D11DeviceAcceptableFuncUserContext(); }


//--------------------------------------------------------------------------------------
// Optionally parses the command line and sets if default hotkeys are handled
//
//       Possible command line parameters are:
//          -forcefeaturelevel:fl     forces app to use a specified direct3D11 feature level    
//          -screenshotexit:filename save a screenshot to the filename.bmp and exit.
//          -forceapi:#             forces app to use specified Direct3D API version (fails if the application doesn't support this API or if no device is found)
//          -adapter:#              forces app to use this adapter # (fails if the adapter doesn't exist)
//          -output:#               [D3D11 only] forces app to use a particular output on the adapter (fails if the output doesn't exist) 
//          -windowed               forces app to start windowed
//          -fullscreen             forces app to start full screen
//          -forcehal               forces app to use HAL (fails if HAL doesn't exist)
//          -forceref               forces app to use REF (fails if REF doesn't exist)
//          -forcepurehwvp          [D3D9 only] forces app to use pure HWVP (fails if device doesn't support it)
//          -forcehwvp              [D3D9 only] forces app to use HWVP (fails if device doesn't support it)
//          -forceswvp              [D3D9 only] forces app to use SWVP 
//          -forcevsync:#           if # is 0, then vsync is disabled 
//          -width:#                forces app to use # for width. for full screen, it will pick the closest possible supported mode
//          -height:#               forces app to use # for height. for full screen, it will pick the closest possible supported mode
//          -startx:#               forces app to use # for the x coord of the window position for windowed mode
//          -starty:#               forces app to use # for the y coord of the window position for windowed mode
//          -constantframetime:#    forces app to use constant frame time, where # is the time/frame in seconds
//          -quitafterframe:x       forces app to quit after # frames
//          -noerrormsgboxes        prevents the display of message boxes generated by the framework so the application can be run without user interaction
//          -nostats                prevents the display of the stats
//          -relaunchmce            re-launches the MCE UI after the app exits
//          -automation             a hint to other components that automation is active 
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTInit( bool bParseCommandLine, 
                         bool bShowMsgBoxOnError, 
                         __in_opt WCHAR* strExtraCommandLineParams, 
                         bool bThreadSafeDXUT )
{
    g_bThreadSafe = bThreadSafeDXUT;

    GetDXUTState().SetDXUTInitCalled( true );

    // Not always needed, but lets the app create GDI dialogs
    InitCommonControls();

    // Save the current sticky/toggle/filter key settings so DXUT can restore them later
    STICKYKEYS sk = {sizeof(STICKYKEYS), 0};
    SystemParametersInfo(SPI_GETSTICKYKEYS, sizeof(STICKYKEYS), &sk, 0);
    GetDXUTState().SetStartupStickyKeys( sk );

    TOGGLEKEYS tk = {sizeof(TOGGLEKEYS), 0};
    SystemParametersInfo(SPI_GETTOGGLEKEYS, sizeof(TOGGLEKEYS), &tk, 0);
    GetDXUTState().SetStartupToggleKeys( tk );

    FILTERKEYS fk = {sizeof(FILTERKEYS), 0};
    SystemParametersInfo(SPI_GETFILTERKEYS, sizeof(FILTERKEYS), &fk, 0);
    GetDXUTState().SetStartupFilterKeys( fk );

    GetDXUTState().SetShowMsgBoxOnError( bShowMsgBoxOnError );

    if( bParseCommandLine )
        DXUTParseCommandLine( GetCommandLine() );
    if( strExtraCommandLineParams )
        DXUTParseCommandLine( strExtraCommandLineParams, false );

    // Reset the timer
    DXUTGetGlobalTimer()->Reset();

    GetDXUTState().SetDXUTInited( true );

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Parses the command line for parameters.  See DXUTInit() for list 
//--------------------------------------------------------------------------------------
void DXUTParseCommandLine(__inout WCHAR* strCommandLine, 
                          bool bIgnoreFirstCommand  )
{
    WCHAR* strCmdLine;
    WCHAR strFlag[MAX_PATH];

    int nNumArgs;
    LPWSTR* pstrArgList = CommandLineToArgvW( strCommandLine, &nNumArgs );
	int iArgStart = 0;
	if( bIgnoreFirstCommand )
        iArgStart = 1;
    for( int iArg = iArgStart; iArg < nNumArgs; iArg++ )
    {
        strCmdLine = pstrArgList[iArg];

        // Handle flag args
        if( *strCmdLine == L'/' || *strCmdLine == L'-' )
        {
            strCmdLine++;

            if( DXUTIsNextArg( strCmdLine, L"forcefeaturelevel" ) )
            {
                if( DXUTGetCmdParam( strCmdLine, strFlag ) )
                {
                    if (_wcsnicmp( strFlag, L"D3D_FEATURE_LEVEL_11_0", MAX_PATH) == 0 ) {
                        GetDXUTState().SetOverrideForceFeatureLevel(D3D_FEATURE_LEVEL_11_0);
                    }else if (_wcsnicmp( strFlag, L"D3D_FEATURE_LEVEL_10_1", MAX_PATH) == 0 ) {
                        GetDXUTState().SetOverrideForceFeatureLevel(D3D_FEATURE_LEVEL_10_1);
                    }else if (_wcsnicmp( strFlag, L"D3D_FEATURE_LEVEL_10_0", MAX_PATH) == 0 ) {
                        GetDXUTState().SetOverrideForceFeatureLevel(D3D_FEATURE_LEVEL_10_0);
                    }else if (_wcsnicmp( strFlag, L"D3D_FEATURE_LEVEL_9_3", MAX_PATH) == 0 ) {
                        GetDXUTState().SetOverrideForceFeatureLevel(D3D_FEATURE_LEVEL_9_3);
                    }else if (_wcsnicmp( strFlag, L"D3D_FEATURE_LEVEL_9_2", MAX_PATH) == 0 ) {
                        GetDXUTState().SetOverrideForceFeatureLevel(D3D_FEATURE_LEVEL_9_2);
                    }else if (_wcsnicmp( strFlag, L"D3D_FEATURE_LEVEL_9_1", MAX_PATH) == 0 ) {
                        GetDXUTState().SetOverrideForceFeatureLevel(D3D_FEATURE_LEVEL_9_1);
                    }


                    continue;
                }
            }
            
            if( DXUTIsNextArg( strCmdLine, L"forceapi" ) )
            {
                if( DXUTGetCmdParam( strCmdLine, strFlag ) )
                {
                    int nAPIVersion = _wtoi( strFlag );
                    GetDXUTState().SetOverrideForceAPI( nAPIVersion );
                    continue;
                }
            }

            if( DXUTIsNextArg( strCmdLine, L"adapter" ) )
            {
                if( DXUTGetCmdParam( strCmdLine, strFlag ) )
                {
                    int nAdapter = _wtoi( strFlag );
                    GetDXUTState().SetOverrideAdapterOrdinal( nAdapter );
                    continue;
                }
            }

            if( DXUTIsNextArg( strCmdLine, L"windowed" ) )
            {
                GetDXUTState().SetOverrideWindowed( true );
                continue;
            }

            if( DXUTIsNextArg( strCmdLine, L"output" ) )
            {
                if( DXUTGetCmdParam( strCmdLine, strFlag ) )
                {
                    int Output = _wtoi( strFlag );
                    GetDXUTState().SetOverrideOutput( Output );
                    continue;
                }
            }

            if( DXUTIsNextArg( strCmdLine, L"fullscreen" ) )
            {
                GetDXUTState().SetOverrideFullScreen( true );
                continue;
            }

            if( DXUTIsNextArg( strCmdLine, L"forcehal" ) )
            {
                GetDXUTState().SetOverrideForceHAL( true );
                continue;
            }
            if( DXUTIsNextArg( strCmdLine, L"screenshotexit" ) ) {
                if( DXUTGetCmdParam( strCmdLine, strFlag ) )
                {
                    GetDXUTState().SetExitAfterScreenShot( true );
                    GetDXUTState().SetSaveScreenShot( true );
                    swprintf_s( GetDXUTState().GetScreenShotName(), 256, L"%s.bmp", strFlag );
                    continue;
                }
            }
            if( DXUTIsNextArg( strCmdLine, L"forceref" ) )
            {
                GetDXUTState().SetOverrideForceREF( true );
                continue;
            }

            if( DXUTIsNextArg( strCmdLine, L"forcevsync" ) )
            {
                if( DXUTGetCmdParam( strCmdLine, strFlag ) )
                {
                    int nOn = _wtoi( strFlag );
                    GetDXUTState().SetOverrideForceVsync( nOn );
                    continue;
                }
            }

            if( DXUTIsNextArg( strCmdLine, L"width" ) )
            {
                if( DXUTGetCmdParam( strCmdLine, strFlag ) )
                {
                    int nWidth = _wtoi( strFlag );
                    GetDXUTState().SetOverrideWidth( nWidth );
                    continue;
                }
            }

            if( DXUTIsNextArg( strCmdLine, L"height" ) )
            {
                if( DXUTGetCmdParam( strCmdLine, strFlag ) )
                {
                    int nHeight = _wtoi( strFlag );
                    GetDXUTState().SetOverrideHeight( nHeight );
                    continue;
                }
            }

            if( DXUTIsNextArg( strCmdLine, L"startx" ) )
            {
                if( DXUTGetCmdParam( strCmdLine, strFlag ) )
                {
                    int nX = _wtoi( strFlag );
                    GetDXUTState().SetOverrideStartX( nX );
                    continue;
                }
            }

            if( DXUTIsNextArg( strCmdLine, L"starty" ) )
            {
                if( DXUTGetCmdParam( strCmdLine, strFlag ) )
                {
                    int nY = _wtoi( strFlag );
                    GetDXUTState().SetOverrideStartY( nY );
                    continue;
                }
            }

            if( DXUTIsNextArg( strCmdLine, L"constantframetime" ) )
            {
                float fTimePerFrame;
                if( DXUTGetCmdParam( strCmdLine, strFlag ) )
                    fTimePerFrame = ( float )wcstod( strFlag, NULL );
                else
                    fTimePerFrame = 0.0333f;
                GetDXUTState().SetOverrideConstantFrameTime( true );
                GetDXUTState().SetOverrideConstantTimePerFrame( fTimePerFrame );
                DXUTSetConstantFrameTime( true, fTimePerFrame );
                continue;
            }

            if( DXUTIsNextArg( strCmdLine, L"quitafterframe" ) )
            {
                if( DXUTGetCmdParam( strCmdLine, strFlag ) )
                {
                    int nFrame = _wtoi( strFlag );
                    GetDXUTState().SetOverrideQuitAfterFrame( nFrame );
                    continue;
                }
            }

            if( DXUTIsNextArg( strCmdLine, L"noerrormsgboxes" ) )
            {
                GetDXUTState().SetShowMsgBoxOnError( false );
                continue;
            }

            if( DXUTIsNextArg( strCmdLine, L"nostats" ) )
            {
                GetDXUTState().SetNoStats( true );
                continue;
            }

            if( DXUTIsNextArg( strCmdLine, L"relaunchmce" ) )
            {
                GetDXUTState().SetOverrideRelaunchMCE( true );
                continue;
            }

            if( DXUTIsNextArg( strCmdLine, L"automation" ) )
            {
                GetDXUTState().SetAutomation( true );
                continue;
            }
        }

        // Unrecognized flag
        wcscpy_s( strFlag, 256, strCmdLine );
        WCHAR* strSpace = strFlag;
        while( *strSpace && ( *strSpace > L' ' ) )
            strSpace++;
        *strSpace = 0;

        DXUTOutputDebugString( L"Unrecognized flag: %s", strFlag );
        strCmdLine += wcslen( strFlag );
    }

    LocalFree( pstrArgList );
}


//--------------------------------------------------------------------------------------
// Helper function for DXUTParseCommandLine
//--------------------------------------------------------------------------------------
bool DXUTIsNextArg( __inout WCHAR*& strCmdLine, 
                    __inout WCHAR* strArg )
{
    int nArgLen = ( int )wcslen( strArg );
    int nCmdLen = ( int )wcslen( strCmdLine );

    if( nCmdLen >= nArgLen &&
        _wcsnicmp( strCmdLine, strArg, nArgLen ) == 0 &&
        ( strCmdLine[nArgLen] == 0 || strCmdLine[nArgLen] == L':' ) )
    {
        strCmdLine += nArgLen;
        return true;
    }

    return false;
}


//--------------------------------------------------------------------------------------
// Helper function for DXUTParseCommandLine.  Updates strCmdLine and strFlag 
//      Example: if strCmdLine=="-width:1024 -forceref"
// then after: strCmdLine==" -forceref" and strFlag=="1024"
//--------------------------------------------------------------------------------------
bool DXUTGetCmdParam( __inout WCHAR*& strCmdLine, 
                      __inout_ecount(MAX_PATH) WCHAR* strFlag )
{
    if( *strCmdLine == L':' )
    {
        strCmdLine++; // Skip ':'

        // Place NULL terminator in strFlag after current token
        wcscpy_s( strFlag, 256, strCmdLine );
        WCHAR* strSpace = strFlag;
        while( *strSpace && ( *strSpace > L' ' ) )
            strSpace++;
        *strSpace = 0;

        // Update strCmdLine
        strCmdLine += wcslen( strFlag );
        return true;
    }
    else
    {
        strFlag[0] = 0;
        return false;
    }
}


//--------------------------------------------------------------------------------------
// Creates a window with the specified window title, icon, menu, and 
// starting position.  If DXUTInit() has not already been called, it will
// call it with the default parameters.  Instead of calling this, you can 
// call DXUTSetWindow() to use an existing window.  
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTCreateWindow( const WCHAR* strWindowTitle, HINSTANCE hInstance,
                                 HICON hIcon, HMENU hMenu, int x, int y )
{
    HRESULT hr;

    // Not allowed to call this from inside the device callbacks
    if( GetDXUTState().GetInsideDeviceCallback() )
        return DXUT_ERR_MSGBOX( L"DXUTCreateWindow", E_FAIL );

    GetDXUTState().SetWindowCreateCalled( true );

    if( !GetDXUTState().GetDXUTInited() )
    {
        // If DXUTInit() was already called and failed, then fail.
        // DXUTInit() must first succeed for this function to succeed
        if( GetDXUTState().GetDXUTInitCalled() )
            return E_FAIL;

        // If DXUTInit() hasn't been called, then automatically call it
        // with default params
        hr = DXUTInit();
        if( FAILED( hr ) )
            return hr;
    }

    if( DXUTGetHWNDFocus() == NULL )
    {
        if( hInstance == NULL )
            hInstance = ( HINSTANCE )GetModuleHandle( NULL );
        GetDXUTState().SetHInstance( hInstance );

        WCHAR szExePath[MAX_PATH];
        GetModuleFileName( NULL, szExePath, MAX_PATH );
        if( hIcon == NULL ) // If the icon is NULL, then use the first one found in the exe
            hIcon = ExtractIcon( hInstance, szExePath, 0 );

        // Register the windows class
        WNDCLASS wndClass;
        wndClass.style = CS_DBLCLKS;
        wndClass.lpfnWndProc = DXUTStaticWndProc;
        wndClass.cbClsExtra = 0;
        wndClass.cbWndExtra = 0;
        wndClass.hInstance = hInstance;
        wndClass.hIcon = hIcon;
        wndClass.hCursor = LoadCursor( NULL, IDC_ARROW );
        wndClass.hbrBackground = ( HBRUSH )GetStockObject( BLACK_BRUSH );
        wndClass.lpszMenuName = NULL;
        wndClass.lpszClassName = L"Direct3DWindowClass";

        if( !RegisterClass( &wndClass ) )
        {
            DWORD dwError = GetLastError();
            if( dwError != ERROR_CLASS_ALREADY_EXISTS )
                return DXUT_ERR_MSGBOX( L"RegisterClass", HRESULT_FROM_WIN32(dwError) );
        }

        // Override the window's initial & size position if there were cmd line args
        if( GetDXUTState().GetOverrideStartX() != -1 )
            x = GetDXUTState().GetOverrideStartX();
        if( GetDXUTState().GetOverrideStartY() != -1 )
            y = GetDXUTState().GetOverrideStartY();

        GetDXUTState().SetWindowCreatedWithDefaultPositions( false );
        if( x == CW_USEDEFAULT && y == CW_USEDEFAULT )
            GetDXUTState().SetWindowCreatedWithDefaultPositions( true );

        // Find the window's initial size, but it might be changed later
        int nDefaultWidth = 640;
        int nDefaultHeight = 480;
        if( GetDXUTState().GetOverrideWidth() != 0 )
            nDefaultWidth = GetDXUTState().GetOverrideWidth();
        if( GetDXUTState().GetOverrideHeight() != 0 )
            nDefaultHeight = GetDXUTState().GetOverrideHeight();

        RECT rc;
        SetRect( &rc, 0, 0, nDefaultWidth, nDefaultHeight );
        AdjustWindowRect( &rc, WS_OVERLAPPEDWINDOW, ( hMenu != NULL ) ? true : false );

        WCHAR* strCachedWindowTitle = GetDXUTState().GetWindowTitle();
        wcscpy_s( strCachedWindowTitle, 256, strWindowTitle );

        // Create the render window
        HWND hWnd = CreateWindow( L"Direct3DWindowClass", strWindowTitle, WS_OVERLAPPEDWINDOW,
                                  x, y, ( rc.right - rc.left ), ( rc.bottom - rc.top ), 0,
                                  hMenu, hInstance, 0 );
        if( hWnd == NULL )
        {
            DWORD dwError = GetLastError();
            return DXUT_ERR_MSGBOX( L"CreateWindow", HRESULT_FROM_WIN32(dwError) );
        }

        GetDXUTState().SetWindowCreated( true );
        GetDXUTState().SetHWNDFocus( hWnd );
        GetDXUTState().SetHWNDDeviceFullScreen( hWnd );
        GetDXUTState().SetHWNDDeviceWindowed( hWnd );
    }

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Sets a previously created window for the framework to use.  If DXUTInit() 
// has not already been called, it will call it with the default parameters.  
// Instead of calling this, you can call DXUTCreateWindow() to create a new window.  
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTSetWindow( HWND hWndFocus, HWND hWndDeviceFullScreen, HWND hWndDeviceWindowed, bool bHandleMessages )
{
    HRESULT hr;

    // Not allowed to call this from inside the device callbacks
    if( GetDXUTState().GetInsideDeviceCallback() )
        return DXUT_ERR_MSGBOX( L"DXUTCreateWindow", E_FAIL );

    GetDXUTState().SetWindowCreateCalled( true );

    // To avoid confusion, we do not allow any HWND to be NULL here.  The
    // caller must pass in valid HWND for all three parameters.  The same
    // HWND may be used for more than one parameter.
    if( hWndFocus == NULL || hWndDeviceFullScreen == NULL || hWndDeviceWindowed == NULL )
        return DXUT_ERR_MSGBOX( L"DXUTSetWindow", E_INVALIDARG );

    // If subclassing the window, set the pointer to the local window procedure
    if( bHandleMessages )
    {
        // Switch window procedures
#ifdef _WIN64
        LONG_PTR nResult = SetWindowLongPtr( hWndFocus, GWLP_WNDPROC, (LONG_PTR)DXUTStaticWndProc );
#else
        LONG_PTR nResult = SetWindowLongPtr( hWndFocus, GWLP_WNDPROC, ( LONG )( LONG_PTR )DXUTStaticWndProc );
#endif

        DWORD dwError = GetLastError();
        if( nResult == 0 )
            return DXUT_ERR_MSGBOX( L"SetWindowLongPtr", HRESULT_FROM_WIN32(dwError) );
    }

    if( !GetDXUTState().GetDXUTInited() )
    {
        // If DXUTInit() was already called and failed, then fail.
        // DXUTInit() must first succeed for this function to succeed
        if( GetDXUTState().GetDXUTInitCalled() )
            return E_FAIL;

        // If DXUTInit() hasn't been called, then automatically call it
        // with default params
        hr = DXUTInit();
        if( FAILED( hr ) )
            return hr;
    }

    WCHAR* strCachedWindowTitle = GetDXUTState().GetWindowTitle();
    GetWindowText( hWndFocus, strCachedWindowTitle, 255 );
    strCachedWindowTitle[255] = 0;

    HINSTANCE hInstance = ( HINSTANCE )( LONG_PTR )GetWindowLongPtr( hWndFocus, GWLP_HINSTANCE );
    GetDXUTState().SetHInstance( hInstance );
    GetDXUTState().SetWindowCreatedWithDefaultPositions( false );
    GetDXUTState().SetWindowCreated( true );
    GetDXUTState().SetHWNDFocus( hWndFocus );
    GetDXUTState().SetHWNDDeviceFullScreen( hWndDeviceFullScreen );
    GetDXUTState().SetHWNDDeviceWindowed( hWndDeviceWindowed );

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Handles window messages 
//--------------------------------------------------------------------------------------
LRESULT CALLBACK DXUTStaticWndProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    
    // Consolidate the keyboard messages and pass them to the app's keyboard callback
    if( uMsg == WM_KEYDOWN ||
        uMsg == WM_SYSKEYDOWN ||
        uMsg == WM_KEYUP ||
        uMsg == WM_SYSKEYUP )
    {
        bool bKeyDown = ( uMsg == WM_KEYDOWN || uMsg == WM_SYSKEYDOWN );
        DWORD dwMask = ( 1 << 29 );
        bool bAltDown = ( ( lParam & dwMask ) != 0 );

        bool* bKeys = GetDXUTState().GetKeys();
        bKeys[ ( BYTE )( wParam & 0xFF ) ] = bKeyDown;

        LPDXUTCALLBACKKEYBOARD pCallbackKeyboard = GetDXUTState().GetKeyboardFunc();
        if( pCallbackKeyboard )
            pCallbackKeyboard( ( UINT )wParam, bKeyDown, bAltDown, GetDXUTState().GetKeyboardFuncUserContext() );
    }

    // Consolidate the mouse button messages and pass them to the app's mouse callback
    if( uMsg == WM_LBUTTONDOWN ||
        uMsg == WM_LBUTTONUP ||
        uMsg == WM_LBUTTONDBLCLK ||
        uMsg == WM_MBUTTONDOWN ||
        uMsg == WM_MBUTTONUP ||
        uMsg == WM_MBUTTONDBLCLK ||
        uMsg == WM_RBUTTONDOWN ||
        uMsg == WM_RBUTTONUP ||
        uMsg == WM_RBUTTONDBLCLK ||
        uMsg == WM_XBUTTONDOWN ||
        uMsg == WM_XBUTTONUP ||
        uMsg == WM_XBUTTONDBLCLK ||
        uMsg == WM_MOUSEWHEEL ||
        ( GetDXUTState().GetNotifyOnMouseMove() && uMsg == WM_MOUSEMOVE ) )
    {
        int xPos = ( short )LOWORD( lParam );
        int yPos = ( short )HIWORD( lParam );

        if( uMsg == WM_MOUSEWHEEL )
        {
            // WM_MOUSEWHEEL passes screen mouse coords
            // so convert them to client coords
            POINT pt;
            pt.x = xPos; pt.y = yPos;
            ScreenToClient( hWnd, &pt );
            xPos = pt.x; yPos = pt.y;
        }

        int nMouseWheelDelta = 0;
        if( uMsg == WM_MOUSEWHEEL )
            nMouseWheelDelta = ( short )HIWORD( wParam );

        int nMouseButtonState = LOWORD( wParam );
        bool bLeftButton = ( ( nMouseButtonState & MK_LBUTTON ) != 0 );
        bool bRightButton = ( ( nMouseButtonState & MK_RBUTTON ) != 0 );
        bool bMiddleButton = ( ( nMouseButtonState & MK_MBUTTON ) != 0 );
        bool bSideButton1 = ( ( nMouseButtonState & MK_XBUTTON1 ) != 0 );
        bool bSideButton2 = ( ( nMouseButtonState & MK_XBUTTON2 ) != 0 );

        bool* bMouseButtons = GetDXUTState().GetMouseButtons();
        bMouseButtons[0] = bLeftButton;
        bMouseButtons[1] = bMiddleButton;
        bMouseButtons[2] = bRightButton;
        bMouseButtons[3] = bSideButton1;
        bMouseButtons[4] = bSideButton2;

        LPDXUTCALLBACKMOUSE pCallbackMouse = GetDXUTState().GetMouseFunc();
        if( pCallbackMouse )
            pCallbackMouse( bLeftButton, bRightButton, bMiddleButton, bSideButton1, bSideButton2, nMouseWheelDelta,
                            xPos, yPos, GetDXUTState().GetMouseFuncUserContext() );
    }

    // Pass all messages to the app's MsgProc callback, and don't 
    // process further messages if the apps says not to.
    LPDXUTCALLBACKMSGPROC pCallbackMsgProc = GetDXUTState().GetWindowMsgFunc();
    if( pCallbackMsgProc )
    {
        bool bNoFurtherProcessing = false;
        LRESULT nResult = pCallbackMsgProc( hWnd, uMsg, wParam, lParam, &bNoFurtherProcessing,
                                            GetDXUTState().GetWindowMsgFuncUserContext() );
        if( bNoFurtherProcessing )
            return nResult;
    }

    switch( uMsg )
    {
        case WM_PAINT:
        {
            
            // Handle paint messages when the app is paused
            if( DXUTIsRenderingPaused() &&
                GetDXUTState().GetDeviceObjectsCreated() && GetDXUTState().GetDeviceObjectsReset() )
            {
                HRESULT hr;
                double fTime = DXUTGetTime();
                float fElapsedTime = DXUTGetElapsedTime();

                if( DXUTIsCurrentDeviceD3D9() )
                {
                    IDirect3DDevice9* pd3dDevice = DXUTGetD3D9Device();
                    if( pd3dDevice )
                    {
                        LPDXUTCALLBACKD3D9FRAMERENDER pCallbackFrameRender = GetDXUTState().GetD3D9FrameRenderFunc();
                        if( pCallbackFrameRender != NULL )
                            pCallbackFrameRender( pd3dDevice, fTime, fElapsedTime,
                                                  GetDXUTState().GetD3D9FrameRenderFuncUserContext() );

                        hr = pd3dDevice->Present( NULL, NULL, NULL, NULL );
                        if( D3DERR_DEVICELOST == hr )
                        {
                            GetDXUTState().SetDeviceLost( true );
                        }
                        else if( D3DERR_DRIVERINTERNALERROR == hr )
                        {
                            // When D3DERR_DRIVERINTERNALERROR is returned from Present(),
                            // the application can do one of the following:
                            // 
                            // - End, with the pop-up window saying that the application cannot continue 
                            //   because of problems in the display adapter and that the user should 
                            //   contact the adapter manufacturer.
                            //
                            // - Attempt to restart by calling IDirect3DDevice9::Reset, which is essentially the same 
                            //   path as recovering from a lost device. If IDirect3DDevice9::Reset fails with 
                            //   D3DERR_DRIVERINTERNALERROR, the application should end immediately with the message 
                            //   that the user should contact the adapter manufacturer.
                            // 
                            // The framework attempts the path of resetting the device
                            // 
                            GetDXUTState().SetDeviceLost( true );
                        }
                    }
                }
               else
                {
                    ID3D11Device* pd3dDevice = DXUTGetD3D11Device();
                    ID3D11DeviceContext *pDeferred = DXUTGetD3D11DeviceContext();
                    if( pd3dDevice )
                    {
                        LPDXUTCALLBACKD3D11FRAMERENDER pCallbackFrameRender = GetDXUTState().GetD3D11FrameRenderFunc();
                        if( pCallbackFrameRender != NULL &&
                            !GetDXUTState().GetRenderingOccluded() )
                        {
                            pCallbackFrameRender( pd3dDevice,pDeferred, fTime, fElapsedTime,
                                                  GetDXUTState().GetD3D11FrameRenderFuncUserContext() );
                        }

                        DWORD dwFlags = 0;
                        if( GetDXUTState().GetRenderingOccluded() )
                            dwFlags = DXGI_PRESENT_TEST;
                        else
                            dwFlags = GetDXUTState().GetCurrentDeviceSettings()->d3d11.PresentFlags;

                        IDXGISwapChain* pSwapChain = DXUTGetDXGISwapChain();
                        hr = pSwapChain->Present( 0, GetDXUTState().GetCurrentDeviceSettings()->d3d11.PresentFlags );
                        if( DXGI_STATUS_OCCLUDED == hr )
                        {
                            // There is a window covering our entire rendering area.
                            // Don't render until we're visible again.
                            GetDXUTState().SetRenderingOccluded( true );
                        }
                        else if( SUCCEEDED( hr ) )
                        {
                            if( GetDXUTState().GetRenderingOccluded() )
                            {
                                // Now that we're no longer occluded
                                // allow us to render again
                                GetDXUTState().SetRenderingOccluded( false );
                            }
                        }
                    }
                }
            }
            break;
        }

        case WM_SIZE:
            
           if( SIZE_MINIMIZED == wParam )
            {
                DXUTPause( true, true ); // Pause while we're minimized

                GetDXUTState().SetMinimized( true );
                GetDXUTState().SetMaximized( false );
            }
            else
            {
                RECT rcCurrentClient;
                GetClientRect( DXUTGetHWND(), &rcCurrentClient );
                if( rcCurrentClient.top == 0 && rcCurrentClient.bottom == 0 )
                {
                    // Rapidly clicking the task bar to minimize and restore a window
                    // can cause a WM_SIZE message with SIZE_RESTORED when 
                    // the window has actually become minimized due to rapid change
                    // so just ignore this message
                }
                else if( SIZE_MAXIMIZED == wParam )
                {
                    if( GetDXUTState().GetMinimized() )
                        DXUTPause( false, false ); // Unpause since we're no longer minimized
                    GetDXUTState().SetMinimized( false );
                    GetDXUTState().SetMaximized( true );
                    DXUTCheckForWindowSizeChange();
                    DXUTCheckForWindowChangingMonitors();
                }
                else if( SIZE_RESTORED == wParam )
                {
                    //DXUTCheckForDXGIFullScreenSwitch();
                    if( GetDXUTState().GetMaximized() )
                    {
                        GetDXUTState().SetMaximized( false );
                        DXUTCheckForWindowSizeChange();
                        DXUTCheckForWindowChangingMonitors();
                    }
                    else if( GetDXUTState().GetMinimized() )
                    {
                        DXUTPause( false, false ); // Unpause since we're no longer minimized
                        GetDXUTState().SetMinimized( false );
                        DXUTCheckForWindowSizeChange();
                        DXUTCheckForWindowChangingMonitors();
                    }
                    else if( GetDXUTState().GetInSizeMove() )
                    {
                        // If we're neither maximized nor minimized, the window size 
                        // is changing by the user dragging the window edges.  In this 
                        // case, we don't reset the device yet -- we wait until the 
                        // user stops dragging, and a WM_EXITSIZEMOVE message comes.
                    }
                    else
                    {
                        // This WM_SIZE come from resizing the window via an API like SetWindowPos() so 
                        // resize and reset the device now.
                        DXUTCheckForWindowSizeChange();
                        DXUTCheckForWindowChangingMonitors();
                    }
                }
            }
            
            break;


        case WM_GETMINMAXINFO:
            ( ( MINMAXINFO* )lParam )->ptMinTrackSize.x = DXUT_MIN_WINDOW_SIZE_X;
            ( ( MINMAXINFO* )lParam )->ptMinTrackSize.y = DXUT_MIN_WINDOW_SIZE_Y;
            break;

        case WM_ENTERSIZEMOVE:
            // Halt frame movement while the app is sizing or moving
            DXUTPause( true, true );
            GetDXUTState().SetInSizeMove( true );
            break;

        case WM_EXITSIZEMOVE:
            DXUTPause( false, false );
            DXUTCheckForWindowSizeChange();
            DXUTCheckForWindowChangingMonitors();
            GetDXUTState().SetInSizeMove( false );
            break;

        case WM_MOUSEMOVE:
            if( DXUTIsActive() && !DXUTIsWindowed() )
            {
                if( DXUTIsCurrentDeviceD3D9() )
                {
                    IDirect3DDevice9* pd3dDevice = DXUTGetD3D9Device();
                    if( pd3dDevice )
                    {
                        POINT ptCursor;
                        GetCursorPos( &ptCursor );
                        pd3dDevice->SetCursorPosition( ptCursor.x, ptCursor.y, 0 );
                    }
                }
                else
                {
                    // For D3D11, no processing is necessary.  D3D11 cursor
                    // is handled in the traditional Windows manner.
                }
            }
            break;

        case WM_SETCURSOR:
            if( DXUTIsActive() && !DXUTIsWindowed() )
            {
                if( DXUTIsCurrentDeviceD3D9() )
                {
                    IDirect3DDevice9* pd3dDevice = DXUTGetD3D9Device();
                    if( pd3dDevice && GetDXUTState().GetShowCursorWhenFullScreen() )
                        pd3dDevice->ShowCursor( true );
                }
                else
                {
                    if( !GetDXUTState().GetShowCursorWhenFullScreen() )
                        SetCursor( NULL );
                }

                return true; // prevent Windows from setting cursor to window class cursor
            }
            break;

        case WM_ACTIVATEAPP:
            if( wParam == TRUE && !DXUTIsActive() ) // Handle only if previously not active 
            {
                GetDXUTState().SetActive( true );

                // Enable controller rumble & input when activating app
                DXUTEnableXInput( true );

                // The GetMinimizedWhileFullscreen() varible is used instead of !DXUTIsWindowed()
                // to handle the rare case toggling to windowed mode while the fullscreen application 
                // is minimized and thus making the pause count wrong
                if( GetDXUTState().GetMinimizedWhileFullscreen() )
                {
                    if( DXUTIsD3D9( GetDXUTState().GetCurrentDeviceSettings() ) )
                        DXUTPause( false, false ); // Unpause since we're no longer minimized
                    GetDXUTState().SetMinimizedWhileFullscreen( false );

                    if( DXUTIsAppRenderingWithD3D11() )
                    {
                      DXUTToggleFullScreen();
                    }
                }

                // Upon returning to this app, potentially disable shortcut keys 
                // (Windows key, accessibility shortcuts) 
                DXUTAllowShortcutKeys( ( DXUTIsWindowed() ) ? GetDXUTState().GetAllowShortcutKeysWhenWindowed() :
                                       GetDXUTState().GetAllowShortcutKeysWhenFullscreen() );

            }
            else if( wParam == FALSE && DXUTIsActive() ) // Handle only if previously active 
            {
                GetDXUTState().SetActive( false );

                // Disable any controller rumble & input when de-activating app
                DXUTEnableXInput( false );

                if( !DXUTIsWindowed() )
                {
                    // Going from full screen to a minimized state 
                    ClipCursor( NULL );      // don't limit the cursor anymore
                    if( DXUTIsD3D9( GetDXUTState().GetCurrentDeviceSettings() ) )
                        DXUTPause( true, true ); // Pause while we're minimized (take care not to pause twice by handling this message twice)
                    GetDXUTState().SetMinimizedWhileFullscreen( true );
                }

                // Restore shortcut keys (Windows key, accessibility shortcuts) to original state
                //
                // This is important to call here if the shortcuts are disabled, 
                // because if this is not done then the Windows key will continue to 
                // be disabled while this app is running which is very bad.
                // If the app crashes, the Windows key will return to normal.
                DXUTAllowShortcutKeys( true );
            }
            break;

        case WM_ENTERMENULOOP:
            // Pause the app when menus are displayed
            DXUTPause( true, true );
            break;

        case WM_EXITMENULOOP:
            DXUTPause( false, false );
            break;

        case WM_MENUCHAR:
            // A menu is active and the user presses a key that does not correspond to any mnemonic or accelerator key
            // So just ignore and don't beep
            return MAKELRESULT( 0, MNC_CLOSE );
            break;

        case WM_NCHITTEST:
            // Prevent the user from selecting the menu in full screen mode
            if( !DXUTIsWindowed() )
                return HTCLIENT;
            break;

        case WM_POWERBROADCAST:
            switch( wParam )
            {
#ifndef PBT_APMQUERYSUSPEND
#define PBT_APMQUERYSUSPEND 0x0000
#endif
                case PBT_APMQUERYSUSPEND:
                    // At this point, the app should save any data for open
                    // network connections, files, etc., and prepare to go into
                    // a suspended mode.  The app can use the MsgProc callback
                    // to handle this if desired.
                    return true;

#ifndef PBT_APMRESUMESUSPEND
#define PBT_APMRESUMESUSPEND 0x0007
#endif
                case PBT_APMRESUMESUSPEND:
                    // At this point, the app should recover any data, network
                    // connections, files, etc., and resume running from when
                    // the app was suspended. The app can use the MsgProc callback
                    // to handle this if desired.

                    // QPC may lose consistency when suspending, so reset the timer
                    // upon resume.
                    DXUTGetGlobalTimer()->Reset();
                    GetDXUTState().SetLastStatsUpdateTime( 0 );
                    return true;
            }
            break;

        case WM_SYSCOMMAND:
            // Prevent moving/sizing in full screen mode
            switch( ( wParam & 0xFFF0 ) )
            {
                case SC_MOVE:
                case SC_SIZE:
                case SC_MAXIMIZE:
                case SC_KEYMENU:
                    if( !DXUTIsWindowed() )
                        return 0;
                    break;
            }
            break;

        case WM_SYSKEYDOWN:
        {
            switch( wParam )
            {
                case VK_RETURN:
                {
                    if( GetDXUTState().GetHandleAltEnter() && DXUTIsAppRenderingWithD3D9() )
                    {
                        // Toggle full screen upon alt-enter 
                        DWORD dwMask = ( 1 << 29 );
                        if( ( lParam & dwMask ) != 0 ) // Alt is down also
                        {
                            // Toggle the full screen/window mode
                            DXUTPause( true, true );
                            DXUTToggleFullScreen();
                            DXUTPause( false, false );
                            return 0;
                        }
                    }
                   
                }
            }
            break;
        }

        case WM_KEYDOWN:
        {
            switch( wParam )
            {
                case VK_ESCAPE:
                {
                    if( GetDXUTState().GetHandleEscape() )
                        SendMessage( hWnd, WM_CLOSE, 0, 0 );
                    break;
                }

                case VK_PAUSE:
                {
                    if( GetDXUTState().GetHandlePause() )
                    {
                        bool bTimePaused = DXUTIsTimePaused();
                        bTimePaused = !bTimePaused;
                        if( bTimePaused )
                            DXUTPause( true, false );
                        else
                            DXUTPause( false, false );
                    }
                    break;
                }
            }
            break;
        }

        case WM_CLOSE:
        {
            HMENU hMenu;
            hMenu = GetMenu( hWnd );
            if( hMenu != NULL )
                DestroyMenu( hMenu );
            DestroyWindow( hWnd );
            UnregisterClass( L"Direct3DWindowClass", NULL );
            GetDXUTState().SetHWNDFocus( NULL );
            GetDXUTState().SetHWNDDeviceFullScreen( NULL );
            GetDXUTState().SetHWNDDeviceWindowed( NULL );
            return 0;
        }

        case WM_DESTROY:
            PostQuitMessage( 0 );
            break;
    }

    // Don't allow the F10 key to act as a shortcut to the menu bar
    // by not passing these messages to the DefWindowProc only when
    // there's no menu present
    if( !GetDXUTState().GetCallDefWindowProc() || GetDXUTState().GetMenu() == NULL &&
        ( uMsg == WM_SYSKEYDOWN || uMsg == WM_SYSKEYUP ) && wParam == VK_F10 )
        return 0;
    else
        return DefWindowProc( hWnd, uMsg, wParam, lParam );
}


//--------------------------------------------------------------------------------------
// Handles app's message loop and rendering when idle.  If DXUTCreateDevice() or DXUTSetD3D*Device() 
// has not already been called, it will call DXUTCreateWindow() with the default parameters.  
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTMainLoop( HACCEL hAccel )
{
    HRESULT hr;

    // Not allowed to call this from inside the device callbacks or reenter
    if( GetDXUTState().GetInsideDeviceCallback() || GetDXUTState().GetInsideMainloop() )
    {
        if( ( GetDXUTState().GetExitCode() == 0 ) || ( GetDXUTState().GetExitCode() == 10 ) )
            GetDXUTState().SetExitCode( 1 );
        return DXUT_ERR_MSGBOX( L"DXUTMainLoop", E_FAIL );
    }

    GetDXUTState().SetInsideMainloop( true );

    // If DXUTCreateDevice() or DXUTSetD3D*Device() has not already been called, 
    // then call DXUTCreateDevice() with the default parameters.         
    if( !GetDXUTState().GetDeviceCreated() )
    {
        if( GetDXUTState().GetDeviceCreateCalled() )
        {
            if( ( GetDXUTState().GetExitCode() == 0 ) || ( GetDXUTState().GetExitCode() == 10 ) )
                GetDXUTState().SetExitCode( 1 );
            return E_FAIL; // DXUTCreateDevice() must first succeed for this function to succeed
        }

        hr = DXUTCreateDevice(D3D_FEATURE_LEVEL_10_0, true, 640, 480);
        if( FAILED( hr ) )
        {
            if( ( GetDXUTState().GetExitCode() == 0 ) || ( GetDXUTState().GetExitCode() == 10 ) )
                GetDXUTState().SetExitCode( 1 );
            return hr;
        }
    }

    HWND hWnd = DXUTGetHWND();

    // DXUTInit() must have been called and succeeded for this function to proceed
    // DXUTCreateWindow() or DXUTSetWindow() must have been called and succeeded for this function to proceed
    // DXUTCreateDevice() or DXUTCreateDeviceFromSettings() or DXUTSetD3D*Device() must have been called and succeeded for this function to proceed
    if( !GetDXUTState().GetDXUTInited() || !GetDXUTState().GetWindowCreated() || !GetDXUTState().GetDeviceCreated() )
    {
        if( ( GetDXUTState().GetExitCode() == 0 ) || ( GetDXUTState().GetExitCode() == 10 ) )
            GetDXUTState().SetExitCode( 1 );
        return DXUT_ERR_MSGBOX( L"DXUTMainLoop", E_FAIL );
    }

    // Now we're ready to receive and process Windows messages.
    bool bGotMsg;
    MSG msg;
    msg.message = WM_NULL;
    PeekMessage( &msg, NULL, 0U, 0U, PM_NOREMOVE );

    while( WM_QUIT != msg.message )
    {
        // Use PeekMessage() so we can use idle time to render the scene. 
        bGotMsg = ( PeekMessage( &msg, NULL, 0U, 0U, PM_REMOVE ) != 0 );

        if( bGotMsg )
        {
            // Translate and dispatch the message
            if( hAccel == NULL || hWnd == NULL ||
                0 == TranslateAccelerator( hWnd, hAccel, &msg ) )
            {
                TranslateMessage( &msg );
                DispatchMessage( &msg );
            }
        }
        else
        {
            // Render a frame during idle time (no messages are waiting)
            DXUTRender3DEnvironment();
        }
    }

    // Cleanup the accelerator table
    if( hAccel != NULL )
        DestroyAcceleratorTable( hAccel );

    GetDXUTState().SetInsideMainloop( false );

    return S_OK;
}


//======================================================================================
//======================================================================================
// Direct3D section
//======================================================================================
//======================================================================================
HRESULT WINAPI DXUTCreateDevice(D3D_FEATURE_LEVEL reqFL,  bool bWindowed, int nSuggestedWidth, int nSuggestedHeight) {
    HRESULT hr = S_OK;
    
   
    // Not allowed to call this from inside the device callbacks
    if( GetDXUTState().GetInsideDeviceCallback() )
        return DXUT_ERR_MSGBOX( L"DXUTCreateWindow", E_FAIL );

    GetDXUTState().SetDeviceCreateCalled( true );

    // If DXUTCreateWindow() or DXUTSetWindow() has not already been called, 
    // then call DXUTCreateWindow() with the default parameters.         
    if( !GetDXUTState().GetWindowCreated() )
    {
        // If DXUTCreateWindow() or DXUTSetWindow() was already called and failed, then fail.
        // DXUTCreateWindow() or DXUTSetWindow() must first succeed for this function to succeed
        if( GetDXUTState().GetWindowCreateCalled() )
            return E_FAIL;

        // If DXUTCreateWindow() or DXUTSetWindow() hasn't been called, then 
        // automatically call DXUTCreateWindow() with default params
        hr = DXUTCreateWindow();
        if( FAILED( hr ) )
            return hr;
    }
    
    DXUTDeviceSettings deviceSettings ;
    DXUTApplyDefaultDeviceSettings(&deviceSettings);
    deviceSettings.MinimumFeatureLevel = reqFL;
    deviceSettings.d3d11.sd.BufferDesc.Width = nSuggestedWidth;
    deviceSettings.d3d11.sd.BufferDesc.Height = nSuggestedHeight;

    deviceSettings.d3d9.pp.BackBufferWidth= nSuggestedWidth;
    deviceSettings.d3d9.pp.BackBufferHeight = nSuggestedHeight;


    //deviceSettings.d3d11.sd.BufferDesc.Width = 480;

    bool bAppSupportsD3D9 = DXUTDoesAppSupportD3D9();
    bool bAppSupportsD3D11 = DXUTDoesAppSupportD3D11();

    if (bAppSupportsD3D11) {
        deviceSettings.ver = DXUT_D3D11_DEVICE;
    }
    else if (bAppSupportsD3D9) {
        deviceSettings.ver = DXUT_D3D9_DEVICE;
    }

    DXUTUpdateDeviceSettingsWithOverrides(&deviceSettings); 


    // Change to a Direct3D device created from the new device settings.  
    // If there is an existing device, then either reset or recreated the scene
    hr = DXUTChangeDevice( &deviceSettings, NULL, NULL, false, true );

    if ( hr ==  DXUTERR_NODIRECT3D11 && GetDXUTState().GetMessageWhenD3D11NotAvailable() ) {
        
        OSVERSIONINFOEX osv;
        memset( &osv, 0, sizeof(osv) );
        osv.dwOSVersionInfoSize = sizeof(osv);
        GetVersionEx( (LPOSVERSIONINFO)&osv );
        

        if ( ( osv.dwMajorVersion > 6 )
            || ( osv.dwMajorVersion == 6 && osv.dwMinorVersion >= 1 ) 
            || ( osv.dwMajorVersion == 6 && osv.dwMinorVersion == 0 && osv.dwBuildNumber > 6002 ) )
        {

            MessageBox( 0, L"Direct3D 11 components were not found.", L"Error", MB_ICONEXCLAMATION );
           // This should not happen, but is here for completeness as the system could be
           // corrupted or some future OS version could pull D3D11.DLL for some reason
        }
        else if ( osv.dwMajorVersion == 6 && osv.dwMinorVersion == 0 && osv.dwBuildNumber == 6002 )
        {

            MessageBox( 0, L"Direct3D 11 components were not found, but are available for"\
            L" this version of Windows.\n"\
            L"For details see Microsoft Knowledge Base Article #971644\n"\
            L"http://go.microsoft.com/fwlink/?LinkId=160189", L"Error", MB_ICONEXCLAMATION );
           
        }
        else if ( osv.dwMajorVersion == 6 && osv.dwMinorVersion == 0 )
        {
            MessageBox( 0, L"Direct3D 11 components were not found. Please install the latest Service Pack.\n"\
            L"For details see Microsoft Knowledge Base Article #935791\n"\
            L"http://support.microsoft.com/kb/935791/", L"Error", MB_ICONEXCLAMATION );
            
        }
        else
        {
            MessageBox( 0, L"Direct3D 11 is not supported on this OS.", L"Error", MB_ICONEXCLAMATION );
        }



    }


    if( FAILED( hr ) )
        return hr;
    
    return hr;
}

//--------------------------------------------------------------------------------------
// Tells the framework to change to a device created from the passed in device settings
// If DXUTCreateWindow() has not already been called, it will call it with the 
// default parameters.  Instead of calling this, you can call DXUTCreateDevice() 
// or DXUTSetD3D*Device() 
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTCreateDeviceFromSettings( DXUTDeviceSettings* pDeviceSettings, bool bPreserveInput,
                                             bool bClipWindowToSingleAdapter )
{
    HRESULT hr;

    GetDXUTState().SetDeviceCreateCalled( true );

    // If DXUTCreateWindow() or DXUTSetWindow() has not already been called, 
    // then call DXUTCreateWindow() with the default parameters.         
    if( !GetDXUTState().GetWindowCreated() )
    {
        // If DXUTCreateWindow() or DXUTSetWindow() was already called and failed, then fail.
        // DXUTCreateWindow() or DXUTSetWindow() must first succeed for this function to succeed
        if( GetDXUTState().GetWindowCreateCalled() )
            return E_FAIL;

        // If DXUTCreateWindow() or DXUTSetWindow() hasn't been called, then 
        // automatically call DXUTCreateWindow() with default params
        hr = DXUTCreateWindow();
        if( FAILED( hr ) )
            return hr;
    }
    DXUTUpdateDeviceSettingsWithOverrides(pDeviceSettings); 

    
    // Change to a Direct3D device created from the new device settings.  
    // If there is an existing device, then either reset or recreate the scene
    hr = DXUTChangeDevice( pDeviceSettings, NULL, NULL, false, bClipWindowToSingleAdapter );
    if( FAILED( hr ) )
        return hr;

    return S_OK;
}


//--------------------------------------------------------------------------------------
// All device changes are sent to this function.  It looks at the current 
// device (if any) and the new device and determines the best course of action.  It 
// also remembers and restores the window state if toggling between windowed and fullscreen
// as well as sets the proper window and system state for switching to the new device.
//--------------------------------------------------------------------------------------
HRESULT DXUTChangeDevice( DXUTDeviceSettings* pNewDeviceSettings,
                          IDirect3DDevice9* pd3d9DeviceFromApp, 
                          ID3D11Device* pd3d11DeviceFromApp,
                          bool bForceRecreate, bool bClipWindowToSingleAdapter )
{
    HRESULT hr = S_OK;
    DXUTDeviceSettings* pOldDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();

    if( !pNewDeviceSettings )
        return S_FALSE;



    if ( pNewDeviceSettings->ver == DXUT_D3D11_DEVICE ) {
        hr = DXUTDelayLoadDXGI();
    }


    if ( pNewDeviceSettings->ver == DXUT_D3D9_DEVICE || 
        ( FAILED( hr ) && DXUTDoesAppSupportD3D9() ) ) {
        hr = DXUTDelayLoadD3D9();
        pNewDeviceSettings->ver = DXUT_D3D9_DEVICE;
        if ( !FAILED( hr ) ) {
            pNewDeviceSettings->ver = DXUT_D3D9_DEVICE;
        }
    }

    if( FAILED( hr ) )
        return hr;

    // Make a copy of the pNewDeviceSettings on the heap
    DXUTDeviceSettings* pNewDeviceSettingsOnHeap = new DXUTDeviceSettings;
    if( pNewDeviceSettingsOnHeap == NULL )
        return E_OUTOFMEMORY;
    memcpy( pNewDeviceSettingsOnHeap, pNewDeviceSettings, sizeof( DXUTDeviceSettings ) );
    pNewDeviceSettings = pNewDeviceSettingsOnHeap;


    GetDXUTState().SetCurrentDeviceSettings(pNewDeviceSettingsOnHeap);
    DXUTSnapDeviceSettingsToEnumDevice(pNewDeviceSettingsOnHeap, false);

    if( FAILED( hr ) ) // the call will fail if no valid devices were found
    {
        DXUTDisplayErrorMessage( hr );
        return DXUT_ERR( L"DXUTFindValidDeviceSettings", hr );
    }

    // If the ModifyDeviceSettings callback is non-NULL, then call it to let the app 
    // change the settings or reject the device change by returning false.
    LPDXUTCALLBACKMODIFYDEVICESETTINGS pCallbackModifyDeviceSettings = GetDXUTState().GetModifyDeviceSettingsFunc();
    if( pCallbackModifyDeviceSettings && pd3d9DeviceFromApp == NULL )
    {
        bool bContinue = pCallbackModifyDeviceSettings( pNewDeviceSettings,
                                                        GetDXUTState().GetModifyDeviceSettingsFuncUserContext() );
        if( !bContinue )
        {
            // The app rejected the device change by returning false, so just use the current device if there is one.
            if( pOldDeviceSettings == NULL )
                DXUTDisplayErrorMessage( DXUTERR_NOCOMPATIBLEDEVICES );
            SAFE_DELETE( pNewDeviceSettings );
            return E_ABORT;
        }
        if( GetDXUTState().GetD3D9() == NULL && GetDXUTState().GetDXGIFactory() == NULL ) // if DXUTShutdown() was called in the modify callback, just return
        {
            SAFE_DELETE( pNewDeviceSettings );
            return S_FALSE;
        }
        DXUTSnapDeviceSettingsToEnumDevice(pNewDeviceSettingsOnHeap, false); // modify the app specified settings to the closed enumerated settigns

        if( FAILED( hr ) ) // the call will fail if no valid devices were found
        {
            DXUTDisplayErrorMessage( hr );
            return DXUT_ERR( L"DXUTFindValidDeviceSettings", hr );
        }

    }

    GetDXUTState().SetCurrentDeviceSettings( pNewDeviceSettingsOnHeap );

    DXUTPause( true, true );

    // When a WM_SIZE message is received, it calls DXUTCheckForWindowSizeChange().
    // A WM_SIZE message might be sent when adjusting the window, so tell 
    // DXUTCheckForWindowSizeChange() to ignore size changes temporarily
    if( DXUTIsCurrentDeviceD3D9() )
        GetDXUTState().SetIgnoreSizeChange( true );


    // Take note if the backbuffer width & height are 0 now as they will change after pd3dDevice->Reset()
    bool bKeepCurrentWindowSize = false;
    if( DXUTGetBackBufferWidthFromDS( pNewDeviceSettings ) == 0 &&
        DXUTGetBackBufferHeightFromDS( pNewDeviceSettings ) == 0 )
        bKeepCurrentWindowSize = true;

    //////////////////////////
    // Before reset
    /////////////////////////

    // If we are using D3D9, adjust window style when switching from windowed to fullscreen and
    // vice versa.  Note that this is not necessary in D3D11 because DXGI handles this.  If both
    // DXUT and DXGI handle this, incorrect behavior would result.
    if( DXUTIsCurrentDeviceD3D9() )
    {
        if( DXUTGetIsWindowedFromDS( pNewDeviceSettings ) )
        {
            // Going to windowed mode

            if( pOldDeviceSettings && !DXUTGetIsWindowedFromDS( pOldDeviceSettings ) )
            {
                // Going from fullscreen -> windowed
                GetDXUTState().SetFullScreenBackBufferWidthAtModeChange( DXUTGetBackBufferWidthFromDS(
                                                                         pOldDeviceSettings ) );
                GetDXUTState().SetFullScreenBackBufferHeightAtModeChange( DXUTGetBackBufferHeightFromDS(
                                                                          pOldDeviceSettings ) );

                // Restore windowed mode style
                SetWindowLong( DXUTGetHWNDDeviceWindowed(), GWL_STYLE, GetDXUTState().GetWindowedStyleAtModeChange() );
            }

            // If different device windows are used for windowed mode and fullscreen mode,
            // hide the fullscreen window so that it doesn't obscure the screen.
            if( DXUTGetHWNDDeviceFullScreen() != DXUTGetHWNDDeviceWindowed() )
                ShowWindow( DXUTGetHWNDDeviceFullScreen(), SW_HIDE );

            // If using the same window for windowed and fullscreen mode, reattach menu if one exists
            if( DXUTGetHWNDDeviceFullScreen() == DXUTGetHWNDDeviceWindowed() )
            {
                if( GetDXUTState().GetMenu() != NULL )
                    SetMenu( DXUTGetHWNDDeviceWindowed(), GetDXUTState().GetMenu() );
            }
        }
        else
        {
            // Going to fullscreen mode

            if( pOldDeviceSettings == NULL || ( pOldDeviceSettings && DXUTGetIsWindowedFromDS( pOldDeviceSettings ) ) )
            {
                // Transistioning to full screen mode from a standard window so 
                // save current window position/size/style now in case the user toggles to windowed mode later 
                WINDOWPLACEMENT* pwp = GetDXUTState().GetWindowedPlacement();
                ZeroMemory( pwp, sizeof( WINDOWPLACEMENT ) );
                pwp->length = sizeof( WINDOWPLACEMENT );
                GetWindowPlacement( DXUTGetHWNDDeviceWindowed(), pwp );
                bool bIsTopmost = ( ( GetWindowLong( DXUTGetHWNDDeviceWindowed(),
                                                     GWL_EXSTYLE ) & WS_EX_TOPMOST ) != 0 );
                GetDXUTState().SetTopmostWhileWindowed( bIsTopmost );
                DWORD dwStyle = GetWindowLong( DXUTGetHWNDDeviceWindowed(), GWL_STYLE );
                dwStyle &= ~WS_MAXIMIZE & ~WS_MINIMIZE; // remove minimize/maximize style
                GetDXUTState().SetWindowedStyleAtModeChange( dwStyle );
                if( pOldDeviceSettings )
                {
                    GetDXUTState().SetWindowBackBufferWidthAtModeChange( DXUTGetBackBufferWidthFromDS(
                                                                         pOldDeviceSettings ) );
                    GetDXUTState().SetWindowBackBufferHeightAtModeChange( DXUTGetBackBufferHeightFromDS(
                                                                          pOldDeviceSettings ) );
                }
            }

            // Hide the window to avoid animation of blank windows
            ShowWindow( DXUTGetHWNDDeviceFullScreen(), SW_HIDE );

            // Set FS window style
            SetWindowLong( DXUTGetHWNDDeviceFullScreen(), GWL_STYLE, WS_POPUP | WS_SYSMENU );

            // If using the same window for windowed and fullscreen mode, save and remove menu 
            if( DXUTGetHWNDDeviceFullScreen() == DXUTGetHWNDDeviceWindowed() )
            {
                HMENU hMenu = GetMenu( DXUTGetHWNDDeviceFullScreen() );
                GetDXUTState().SetMenu( hMenu );
                SetMenu( DXUTGetHWNDDeviceFullScreen(), NULL );
            }

            WINDOWPLACEMENT wpFullscreen;
            ZeroMemory( &wpFullscreen, sizeof( WINDOWPLACEMENT ) );
            wpFullscreen.length = sizeof( WINDOWPLACEMENT );
            GetWindowPlacement( DXUTGetHWNDDeviceFullScreen(), &wpFullscreen );
            if( ( wpFullscreen.flags & WPF_RESTORETOMAXIMIZED ) != 0 )
            {
                // Restore the window to normal if the window was maximized then minimized.  This causes the 
                // WPF_RESTORETOMAXIMIZED flag to be set which will cause SW_RESTORE to restore the 
                // window from minimized to maxmized which isn't what we want
                wpFullscreen.flags &= ~WPF_RESTORETOMAXIMIZED;
                wpFullscreen.showCmd = SW_RESTORE;
                SetWindowPlacement( DXUTGetHWNDDeviceFullScreen(), &wpFullscreen );
            }
        }
    }
    else
    {
        if( DXUTGetIsWindowedFromDS( pNewDeviceSettings ) )
        {
            // Going to windowed mode
           if( pOldDeviceSettings && !DXUTGetIsWindowedFromDS( pOldDeviceSettings ) )
            {
                // Going from fullscreen -> windowed
                GetDXUTState().SetFullScreenBackBufferWidthAtModeChange( DXUTGetBackBufferWidthFromDS(
                                                                         pOldDeviceSettings ) );
                GetDXUTState().SetFullScreenBackBufferHeightAtModeChange( DXUTGetBackBufferHeightFromDS(
                                                                          pOldDeviceSettings ) );
                //DXGI should handle this, but in the case where switching from d3d9 full screen to windowed d3d11 it does not.
                SetWindowLong( DXUTGetHWNDDeviceWindowed(), GWL_STYLE, GetDXUTState().GetWindowedStyleAtModeChange() );

           }
        }
        else
        {
            // Going to fullscreen mode
            if( pOldDeviceSettings == NULL || ( pOldDeviceSettings && DXUTGetIsWindowedFromDS( pOldDeviceSettings ) ) )
            {
                // Transistioning to full screen mode from a standard window so 
                if( pOldDeviceSettings )
                {
                    GetDXUTState().SetWindowBackBufferWidthAtModeChange( DXUTGetBackBufferWidthFromDS(
                                                                         pOldDeviceSettings ) );
                    GetDXUTState().SetWindowBackBufferHeightAtModeChange( DXUTGetBackBufferHeightFromDS(
                                                                          pOldDeviceSettings ) );
                }
            }
        }
    }

    if( pOldDeviceSettings )
        DXUTCleanup3DEnvironment( false );

    // Create the D3D device and call the app's device callbacks
    if( DXUTIsD3D9( pNewDeviceSettings ) ) {
        hr = DXUTCreate3DEnvironment9( pd3d9DeviceFromApp );
    }
    else {
        hr = DXUTCreate3DEnvironment11( pd3d11DeviceFromApp );
    }
    if( FAILED( hr ) )
    {
        SAFE_DELETE( pOldDeviceSettings );
        DXUTCleanup3DEnvironment( true );
        DXUTDisplayErrorMessage( hr );
        DXUTPause( false, false );
        GetDXUTState().SetIgnoreSizeChange( false );
        return hr;
    }

    // Enable/disable StickKeys shortcut, ToggleKeys shortcut, FilterKeys shortcut, and Windows key 
    // to prevent accidental task switching
    DXUTAllowShortcutKeys( ( DXUTGetIsWindowedFromDS( pNewDeviceSettings ) ) ?
                           GetDXUTState().GetAllowShortcutKeysWhenWindowed() :
                           GetDXUTState().GetAllowShortcutKeysWhenFullscreen() );

    HMONITOR hAdapterMonitor = DXUTGetMonitorFromAdapter( pNewDeviceSettings );
    GetDXUTState().SetAdapterMonitor( hAdapterMonitor );

    // Update the device stats text
    DXUTUpdateStaticFrameStats();

    if( pOldDeviceSettings && !DXUTGetIsWindowedFromDS( pOldDeviceSettings ) &&
        DXUTGetIsWindowedFromDS( pNewDeviceSettings ) )
    {
        // Going from fullscreen -> windowed

        // Restore the show state, and positions/size of the window to what it was
        // It is important to adjust the window size 
        // after resetting the device rather than beforehand to ensure 
        // that the monitor resolution is correct and does not limit the size of the new window.
        WINDOWPLACEMENT* pwp = GetDXUTState().GetWindowedPlacement();
        SetWindowPlacement( DXUTGetHWNDDeviceWindowed(), pwp );

        // Also restore the z-order of window to previous state
        HWND hWndInsertAfter = GetDXUTState().GetTopmostWhileWindowed() ? HWND_TOPMOST : HWND_NOTOPMOST;
        SetWindowPos( DXUTGetHWNDDeviceWindowed(), hWndInsertAfter, 0, 0, 0, 0,
                      SWP_NOMOVE | SWP_NOREDRAW | SWP_NOSIZE );
    }

    // Check to see if the window needs to be resized.  
    // Handle cases where the window is minimized and maxmimized as well.
 
    bool bNeedToResize = false;
    if( DXUTGetIsWindowedFromDS( pNewDeviceSettings ) && // only resize if in windowed mode
        !bKeepCurrentWindowSize )                      // only resize if pp.BackbufferWidth/Height were not 0
    {
        UINT nClientWidth;
        UINT nClientHeight;
        if( IsIconic( DXUTGetHWNDDeviceWindowed() ) )
        {
            // Window is currently minimized. To tell if it needs to resize, 
            // get the client rect of window when its restored the 
            // hard way using GetWindowPlacement()
            WINDOWPLACEMENT wp;
            ZeroMemory( &wp, sizeof( WINDOWPLACEMENT ) );
            wp.length = sizeof( WINDOWPLACEMENT );
            GetWindowPlacement( DXUTGetHWNDDeviceWindowed(), &wp );

            if( ( wp.flags & WPF_RESTORETOMAXIMIZED ) != 0 && wp.showCmd == SW_SHOWMINIMIZED )
            {
                // WPF_RESTORETOMAXIMIZED means that when the window is restored it will
                // be maximized.  So maximize the window temporarily to get the client rect 
                // when the window is maximized.  GetSystemMetrics( SM_CXMAXIMIZED ) will give this 
                // information if the window is on the primary but this will work on multimon.
                ShowWindow( DXUTGetHWNDDeviceWindowed(), SW_RESTORE );
                RECT rcClient;
                GetClientRect( DXUTGetHWNDDeviceWindowed(), &rcClient );
                nClientWidth = ( UINT )( rcClient.right - rcClient.left );
                nClientHeight = ( UINT )( rcClient.bottom - rcClient.top );
                ShowWindow( DXUTGetHWNDDeviceWindowed(), SW_MINIMIZE );
            }
            else
            {
                // Use wp.rcNormalPosition to get the client rect, but wp.rcNormalPosition 
                // includes the window frame so subtract it
                RECT rcFrame = {0};
                AdjustWindowRect( &rcFrame, GetDXUTState().GetWindowedStyleAtModeChange(), GetDXUTState().GetMenu() != NULL );
                LONG nFrameWidth = rcFrame.right - rcFrame.left;
                LONG nFrameHeight = rcFrame.bottom - rcFrame.top;
                nClientWidth = ( UINT )( wp.rcNormalPosition.right - wp.rcNormalPosition.left - nFrameWidth );
                nClientHeight = ( UINT )( wp.rcNormalPosition.bottom - wp.rcNormalPosition.top - nFrameHeight );
            }
        }
        else
        {
            // Window is restored or maximized so just get its client rect
            RECT rcClient;
            GetClientRect( DXUTGetHWNDDeviceWindowed(), &rcClient );
            nClientWidth = ( UINT )( rcClient.right - rcClient.left );
            nClientHeight = ( UINT )( rcClient.bottom - rcClient.top );
        }

        // Now that we know the client rect, compare it against the back buffer size
        // to see if the client rect is already the right size
        if( nClientWidth != DXUTGetBackBufferWidthFromDS( pNewDeviceSettings ) ||
            nClientHeight != DXUTGetBackBufferHeightFromDS( pNewDeviceSettings ) )
        {
            bNeedToResize = true;
        }

        if( bClipWindowToSingleAdapter && !IsIconic( DXUTGetHWNDDeviceWindowed() ) )
        {
            // Get the rect of the monitor attached to the adapter
            MONITORINFO miAdapter;
            miAdapter.cbSize = sizeof( MONITORINFO );
            HMONITOR hAdapterMonitor = DXUTGetMonitorFromAdapter( pNewDeviceSettings );
            DXUTGetMonitorInfo( hAdapterMonitor, &miAdapter );
            HMONITOR hWindowMonitor = DXUTMonitorFromWindow( DXUTGetHWND(), MONITOR_DEFAULTTOPRIMARY );

            // Get the rect of the window
            RECT rcWindow;
            GetWindowRect( DXUTGetHWNDDeviceWindowed(), &rcWindow );

            // Check if the window rect is fully inside the adapter's vitural screen rect
            if( ( rcWindow.left < miAdapter.rcWork.left ||
                  rcWindow.right > miAdapter.rcWork.right ||
                  rcWindow.top < miAdapter.rcWork.top ||
                  rcWindow.bottom > miAdapter.rcWork.bottom ) )
            {
                if( hWindowMonitor == hAdapterMonitor && IsZoomed( DXUTGetHWNDDeviceWindowed() ) )
                {
                    // If the window is maximized and on the same monitor as the adapter, then 
                    // no need to clip to single adapter as the window is already clipped 
                    // even though the rcWindow rect is outside of the miAdapter.rcWork
                }
                else
                {
                    bNeedToResize = true;
                }
            }
        }
    }

    // Only resize window if needed 

    if( bNeedToResize )
    {
        // Need to resize, so if window is maximized or minimized then restore the window
        if( IsIconic( DXUTGetHWNDDeviceWindowed() ) )
            ShowWindow( DXUTGetHWNDDeviceWindowed(), SW_RESTORE );
        if( IsZoomed( DXUTGetHWNDDeviceWindowed() ) ) // doing the IsIconic() check first also handles the WPF_RESTORETOMAXIMIZED case
            ShowWindow( DXUTGetHWNDDeviceWindowed(), SW_RESTORE );

        if( bClipWindowToSingleAdapter  )
        {
            // Get the rect of the monitor attached to the adapter
            MONITORINFO miAdapter;
            miAdapter.cbSize = sizeof( MONITORINFO );
            HMONITOR hAdapterMonitor = DXUTGetMonitorFromAdapter( pNewDeviceSettings );
            DXUTGetMonitorInfo( hAdapterMonitor, &miAdapter );

            // Get the rect of the monitor attached to the window
            MONITORINFO miWindow;
            miWindow.cbSize = sizeof( MONITORINFO );
            DXUTGetMonitorInfo( DXUTMonitorFromWindow( DXUTGetHWND(), MONITOR_DEFAULTTOPRIMARY ), &miWindow );

            // Do something reasonable if the BackBuffer size is greater than the monitor size
            int nAdapterMonitorWidth = miAdapter.rcWork.right - miAdapter.rcWork.left;
            int nAdapterMonitorHeight = miAdapter.rcWork.bottom - miAdapter.rcWork.top;

            int nClientWidth = DXUTGetBackBufferWidthFromDS( pNewDeviceSettings );
            int nClientHeight = DXUTGetBackBufferHeightFromDS( pNewDeviceSettings );

            // Get the rect of the window
            RECT rcWindow;
            GetWindowRect( DXUTGetHWNDDeviceWindowed(), &rcWindow );

            // Make a window rect with a client rect that is the same size as the backbuffer
            RECT rcResizedWindow;
            rcResizedWindow.left = 0;
            rcResizedWindow.right = nClientWidth;
            rcResizedWindow.top = 0;
            rcResizedWindow.bottom = nClientHeight;
            AdjustWindowRect( &rcResizedWindow, GetWindowLong( DXUTGetHWNDDeviceWindowed(), GWL_STYLE ),
                              GetDXUTState().GetMenu() != NULL );

            int nWindowWidth = rcResizedWindow.right - rcResizedWindow.left;
            int nWindowHeight = rcResizedWindow.bottom - rcResizedWindow.top;

            if( nWindowWidth > nAdapterMonitorWidth )
                nWindowWidth = nAdapterMonitorWidth;
            if( nWindowHeight > nAdapterMonitorHeight )
                nWindowHeight = nAdapterMonitorHeight;

            if( rcResizedWindow.left < miAdapter.rcWork.left ||
                rcResizedWindow.top < miAdapter.rcWork.top ||
                rcResizedWindow.right > miAdapter.rcWork.right ||
                rcResizedWindow.bottom > miAdapter.rcWork.bottom )
            {
                int nWindowOffsetX = ( nAdapterMonitorWidth - nWindowWidth ) / 2;
                int nWindowOffsetY = ( nAdapterMonitorHeight - nWindowHeight ) / 2;

                rcResizedWindow.left = miAdapter.rcWork.left + nWindowOffsetX;
                rcResizedWindow.top = miAdapter.rcWork.top + nWindowOffsetY;
                rcResizedWindow.right = miAdapter.rcWork.left + nWindowOffsetX + nWindowWidth;
                rcResizedWindow.bottom = miAdapter.rcWork.top + nWindowOffsetY + nWindowHeight;
            }

            // Resize the window.  It is important to adjust the window size 
            // after resetting the device rather than beforehand to ensure 
            // that the monitor resolution is correct and does not limit the size of the new window.
            SetWindowPos( DXUTGetHWNDDeviceWindowed(), 0, rcResizedWindow.left, rcResizedWindow.top, nWindowWidth,
                          nWindowHeight, SWP_NOZORDER );
        }
        else
        {
            // Make a window rect with a client rect that is the same size as the backbuffer
            RECT rcWindow = {0};
            rcWindow.right = (long)( DXUTGetBackBufferWidthFromDS(pNewDeviceSettings) );
            rcWindow.bottom = (long)( DXUTGetBackBufferHeightFromDS(pNewDeviceSettings) );
            AdjustWindowRect( &rcWindow, GetWindowLong( DXUTGetHWNDDeviceWindowed(), GWL_STYLE ), GetDXUTState().GetMenu() != NULL );

            // Resize the window.  It is important to adjust the window size 
            // after resetting the device rather than beforehand to ensure 
            // that the monitor resolution is correct and does not limit the size of the new window.
            int cx = ( int )( rcWindow.right - rcWindow.left );
            int cy = ( int )( rcWindow.bottom - rcWindow.top );
            SetWindowPos( DXUTGetHWNDDeviceWindowed(), 0, 0, 0, cx, cy, SWP_NOZORDER | SWP_NOMOVE );
        }

        // Its possible that the new window size is not what we asked for.  
        // No window can be sized larger than the desktop, so see if the Windows OS resized the 
        // window to something smaller to fit on the desktop.  Also if WM_GETMINMAXINFO
        // will put a limit on the smallest/largest window size.
        RECT rcClient;
        GetClientRect( DXUTGetHWNDDeviceWindowed(), &rcClient );
        UINT nClientWidth = ( UINT )( rcClient.right - rcClient.left );
        UINT nClientHeight = ( UINT )( rcClient.bottom - rcClient.top );
        if( nClientWidth != DXUTGetBackBufferWidthFromDS( pNewDeviceSettings ) ||
            nClientHeight != DXUTGetBackBufferHeightFromDS( pNewDeviceSettings ) )
        {
            // If its different, then resize the backbuffer again.  This time create a backbuffer that matches the 
            // client rect of the current window w/o resizing the window.
            DXUTDeviceSettings deviceSettings = DXUTGetDeviceSettings();
            if( DXUTIsD3D9( &deviceSettings ) ) deviceSettings.d3d9.pp.BackBufferWidth = 0; else deviceSettings.d3d11.sd.BufferDesc.Width = 0; 
            if( DXUTIsD3D9( &deviceSettings ) ) deviceSettings.d3d9.pp.BackBufferHeight = 0; else deviceSettings.d3d11.sd.BufferDesc.Height = 0;

            hr = DXUTChangeDevice( &deviceSettings, NULL, NULL, false, bClipWindowToSingleAdapter );
            if( FAILED( hr ) )
            {
                SAFE_DELETE( pOldDeviceSettings );
                DXUTCleanup3DEnvironment( true );
                DXUTPause( false, false );
                GetDXUTState().SetIgnoreSizeChange( false );
                return hr;
            }
        }
    }

    //if (DXUTGetIsWindowedFromDS( pNewDeviceSettings )) {
    //    RECT rcFrame = {0};
    //    AdjustWindowRect( &rcFrame, GetDXUTState().GetWindowedStyleAtModeChange(), GetDXUTState().GetMenu() != NULL );
   // }

    // Make the window visible
    if( !IsWindowVisible( DXUTGetHWND() ) )
        ShowWindow( DXUTGetHWND(), SW_SHOW );

    // Ensure that the display doesn't power down when fullscreen but does when windowed
    if( !DXUTIsWindowed() )
        SetThreadExecutionState( ES_DISPLAY_REQUIRED | ES_CONTINUOUS );
    else
        SetThreadExecutionState( ES_CONTINUOUS );

    SAFE_DELETE( pOldDeviceSettings );
    GetDXUTState().SetIgnoreSizeChange( false );
    DXUTPause( false, false );
    GetDXUTState().SetDeviceCreated( true );

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Creates a DXGI factory object if one has not already been created  
//--------------------------------------------------------------------------------------
HRESULT DXUTDelayLoadDXGI()
{
    IDXGIFactory1* pDXGIFactory = GetDXUTState().GetDXGIFactory();
    if( pDXGIFactory == NULL )
    {
        DXUT_Dynamic_CreateDXGIFactory1( __uuidof( IDXGIFactory1 ), ( LPVOID* )&pDXGIFactory );
        GetDXUTState().SetDXGIFactory( pDXGIFactory );
        if( pDXGIFactory == NULL )
        {
            // If still NULL, then DXGI is not availible
            GetDXUTState().SetD3D11Available( false );
            return DXUTERR_NODIRECT3D11;
        }

        // TODO: check for D3D11 support
        GetDXUTState().SetD3D11Available( true );
    }

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Creates a Direct3D object if one has not already been created  
//--------------------------------------------------------------------------------------
HRESULT DXUTDelayLoadD3D9()
{
    IDirect3D9* pD3D = GetDXUTState().GetD3D9();
    if( pD3D == NULL )
    {
        // This may fail if Direct3D 9 isn't installed
        // This may also fail if the Direct3D headers are somehow out of sync with the installed Direct3D DLLs
        pD3D = DXUT_Dynamic_Direct3DCreate9( D3D_SDK_VERSION );
        if( pD3D == NULL )
        {
            // If still NULL, then D3D9 is not availible
            return DXUTERR_NODIRECT3D;
        }

        GetDXUTState().SetD3D9( pD3D );
    }

    return S_OK;
}



//--------------------------------------------------------------------------------------
// Updates the device settings with default values..  
//--------------------------------------------------------------------------------------
void DXUTUpdateDeviceSettingsWithOverrides( DXUTDeviceSettings* pDeviceSettings )
{
    // Override with settings from the command line
    if( GetDXUTState().GetOverrideWidth() != 0 )
    {
        pDeviceSettings->d3d9.pp.BackBufferWidth = GetDXUTState().GetOverrideWidth();
        pDeviceSettings->d3d11.sd.BufferDesc.Width = GetDXUTState().GetOverrideWidth();
    }
    if( GetDXUTState().GetOverrideHeight() != 0 )
    {
        pDeviceSettings->d3d9.pp.BackBufferHeight = GetDXUTState().GetOverrideHeight();
        pDeviceSettings->d3d11.sd.BufferDesc.Height = GetDXUTState().GetOverrideHeight();
    }

    if( GetDXUTState().GetOverrideAdapterOrdinal() != -1 )
    {
        pDeviceSettings->d3d9.AdapterOrdinal = GetDXUTState().GetOverrideAdapterOrdinal();
        pDeviceSettings->d3d11.AdapterOrdinal = GetDXUTState().GetOverrideAdapterOrdinal();
    }

    if( GetDXUTState().GetOverrideFullScreen() )
    {
        pDeviceSettings->d3d9.pp.Windowed = FALSE;
        pDeviceSettings->d3d11.sd.Windowed = FALSE;
    }

    if( GetDXUTState().GetOverrideWindowed() ) {
        pDeviceSettings->d3d9.pp.Windowed = TRUE;
        pDeviceSettings->d3d11.sd.Windowed = TRUE;
    }

    if( GetDXUTState().GetOverrideForceHAL() )
    {
        pDeviceSettings->d3d9.DeviceType = D3DDEVTYPE_HAL;
        pDeviceSettings->d3d11.DriverType = D3D_DRIVER_TYPE_HARDWARE;
    }

    if( GetDXUTState().GetOverrideForceREF() )
    {
        pDeviceSettings->d3d9.DeviceType = D3DDEVTYPE_REF;
        pDeviceSettings->d3d11.DriverType = D3D_DRIVER_TYPE_REFERENCE;
    }

    if( GetDXUTState().GetOverrideForceVsync() == 0 )
    {
        pDeviceSettings->d3d9.pp.PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE;
        pDeviceSettings->d3d11.SyncInterval = 0;
    }
    else if( GetDXUTState().GetOverrideForceVsync() == 1 )
    {
        pDeviceSettings->d3d9.pp.PresentationInterval = D3DPRESENT_INTERVAL_DEFAULT;
        pDeviceSettings->d3d11.SyncInterval = 1;
    }

    if( GetDXUTState().GetOverrideForceAPI() != -1 )
    {
        if( GetDXUTState().GetOverrideForceAPI() == 9 )
        {
            pDeviceSettings->ver = DXUT_D3D9_DEVICE;
        }
        else if( GetDXUTState().GetOverrideForceAPI() == 11 )
        {
            pDeviceSettings->ver = DXUT_D3D11_DEVICE;
        }
    }
    
    if (GetDXUTState().GetOverrideForceFeatureLevel() != 0) {
        pDeviceSettings->d3d11.DeviceFeatureLevel = (D3D_FEATURE_LEVEL)GetDXUTState().GetOverrideForceFeatureLevel();
    }
}


//--------------------------------------------------------------------------------------
// Allows the app to explictly state if it supports D3D9 or D3D11.  Typically
// calling this is not needed as DXUT will auto-detect this based on the callbacks set.
//--------------------------------------------------------------------------------------
void WINAPI DXUTSetD3DVersionSupport( bool bAppCanUseD3D9,  bool bAppCanUseD3D11 )
{
    GetDXUTState().SetUseD3DVersionOverride( true );
    GetDXUTState().SetAppSupportsD3D9Override( bAppCanUseD3D9 );
    GetDXUTState().SetAppSupportsD3D11Override( bAppCanUseD3D11 );
}


//--------------------------------------------------------------------------------------
// Returns true if app has registered any D3D9 callbacks or 
// used the DXUTSetD3DVersionSupport API and passed true for bAppCanUseD3D9
//--------------------------------------------------------------------------------------
bool WINAPI DXUTDoesAppSupportD3D9()
{
    if( GetDXUTState().GetUseD3DVersionOverride() )
        return GetDXUTState().GetAppSupportsD3D9Override();
    else
        return GetDXUTState().GetIsD3D9DeviceAcceptableFunc() ||
            GetDXUTState().GetD3D9DeviceCreatedFunc() ||
            GetDXUTState().GetD3D9DeviceResetFunc() ||
            GetDXUTState().GetD3D9DeviceLostFunc() ||
            GetDXUTState().GetD3D9DeviceDestroyedFunc() ||
            GetDXUTState().GetD3D9FrameRenderFunc();
}


//--------------------------------------------------------------------------------------
// Returns true if app has registered any D3D11 callbacks or 
// used the DXUTSetD3DVersionSupport API and passed true for bAppCanUseD3D11
//--------------------------------------------------------------------------------------
bool WINAPI DXUTDoesAppSupportD3D11()
{
    if( GetDXUTState().GetUseD3DVersionOverride() )
        return GetDXUTState().GetAppSupportsD3D11Override();
    else
        return GetDXUTState().GetIsD3D11DeviceAcceptableFunc() ||
            GetDXUTState().GetD3D11DeviceCreatedFunc() ||
            GetDXUTState().GetD3D11SwapChainResizedFunc() ||
            GetDXUTState().GetD3D11FrameRenderFunc() ||
            GetDXUTState().GetD3D11SwapChainReleasingFunc() ||
            GetDXUTState().GetD3D11DeviceDestroyedFunc();
}


//======================================================================================
//======================================================================================
// Direct3D 9 section
//======================================================================================
//======================================================================================


//--------------------------------------------------------------------------------------
// Passes a previously created Direct3D9 device for use by the framework.  
// If DXUTCreateWindow() has not already been called, it will call it with the 
// default parameters.  Instead of calling this, you can call DXUTCreateDevice() or 
// DXUTCreateDeviceFromSettings() 
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTSetD3D9Device( IDirect3DDevice9* pd3dDevice )
{
    HRESULT hr;

    if( pd3dDevice == NULL )
        return DXUT_ERR_MSGBOX( L"DXUTSetD3D9Device", E_INVALIDARG );

    // Not allowed to call this from inside the device callbacks
    if( GetDXUTState().GetInsideDeviceCallback() )
        return DXUT_ERR_MSGBOX( L"DXUTSetD3D9Device", E_FAIL );

    GetDXUTState().SetDeviceCreateCalled( true );

    // If DXUTCreateWindow() or DXUTSetWindow() has not already been called, 
    // then call DXUTCreateWindow() with the default parameters.         
    if( !GetDXUTState().GetWindowCreated() )
    {
        // If DXUTCreateWindow() or DXUTSetWindow() was already called and failed, then fail.
        // DXUTCreateWindow() or DXUTSetWindow() must first succeed for this function to succeed
        if( GetDXUTState().GetWindowCreateCalled() )
            return E_FAIL;

        // If DXUTCreateWindow() or DXUTSetWindow() hasn't been called, then 
        // automatically call DXUTCreateWindow() with default params
        hr = DXUTCreateWindow();
        if( FAILED( hr ) )
            return hr;
    }

    DXUTDeviceSettings DeviceSettings;
    ZeroMemory( &DeviceSettings, sizeof( DXUTDeviceSettings ) );
    DeviceSettings.ver = DXUT_D3D9_DEVICE;

    // Get the present params from the swap chain
    IDirect3DSurface9* pBackBuffer = NULL;
    hr = pd3dDevice->GetBackBuffer( 0, 0, D3DBACKBUFFER_TYPE_MONO, &pBackBuffer );
    if( SUCCEEDED( hr ) )
    {
        IDirect3DSwapChain9* pSwapChain = NULL;
        hr = pBackBuffer->GetContainer( IID_IDirect3DSwapChain9, ( void** )&pSwapChain );
        if( SUCCEEDED( hr ) )
        {
            pSwapChain->GetPresentParameters( &DeviceSettings.d3d9.pp );
            SAFE_RELEASE( pSwapChain );
        }

        SAFE_RELEASE( pBackBuffer );
    }

    D3DDEVICE_CREATION_PARAMETERS d3dCreationParams;
    pd3dDevice->GetCreationParameters( &d3dCreationParams );

    // Fill out the rest of the device settings struct
    DeviceSettings.d3d9.AdapterOrdinal = d3dCreationParams.AdapterOrdinal;
    DeviceSettings.d3d9.DeviceType = d3dCreationParams.DeviceType;
    DXUTFindD3D9AdapterFormat( DeviceSettings.d3d9.AdapterOrdinal, DeviceSettings.d3d9.DeviceType,
                               DeviceSettings.d3d9.pp.BackBufferFormat, DeviceSettings.d3d9.pp.Windowed,
                               &DeviceSettings.d3d9.AdapterFormat );
    DeviceSettings.d3d9.BehaviorFlags = d3dCreationParams.BehaviorFlags;

    // Change to the Direct3D device passed in
    hr = DXUTChangeDevice( &DeviceSettings, pd3dDevice,  NULL, false, false );
    if( FAILED( hr ) )
        return hr;

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Creates the 3D environment
//--------------------------------------------------------------------------------------
HRESULT DXUTCreate3DEnvironment9( IDirect3DDevice9* pd3dDeviceFromApp )
{
    HRESULT hr = S_OK;

    IDirect3DDevice9* pd3dDevice = NULL;
    DXUTDeviceSettings* pNewDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();

    // Only create a Direct3D device if one hasn't been supplied by the app
    if( pd3dDeviceFromApp == NULL )
    {
        // Try to create the device with the chosen settings
        IDirect3D9* pD3D = DXUTGetD3D9Object();
        hr = pD3D->CreateDevice( pNewDeviceSettings->d3d9.AdapterOrdinal, pNewDeviceSettings->d3d9.DeviceType,
                                 DXUTGetHWNDFocus(), pNewDeviceSettings->d3d9.BehaviorFlags,
                                 &pNewDeviceSettings->d3d9.pp, &pd3dDevice );
        if( hr == D3DERR_DEVICELOST )
        {
            GetDXUTState().SetDeviceLost( true );
            return S_OK;
        }
        else if( FAILED( hr ) )
        {
            DXUT_ERR( L"CreateDevice", hr );
            return DXUTERR_CREATINGDEVICE;
        }
    }
    else
    {
        pd3dDeviceFromApp->AddRef();
        pd3dDevice = pd3dDeviceFromApp;
    }

    GetDXUTState().SetD3D9Device( pd3dDevice );

    // If switching to REF, set the exit code to 10.  If switching to HAL and exit code was 10, then set it back to 0.
    if( pNewDeviceSettings->d3d9.DeviceType == D3DDEVTYPE_REF && GetDXUTState().GetExitCode() == 0 )
        GetDXUTState().SetExitCode( 10 );
    else if( pNewDeviceSettings->d3d9.DeviceType == D3DDEVTYPE_HAL && GetDXUTState().GetExitCode() == 10 )
        GetDXUTState().SetExitCode( 0 );

    // Update back buffer desc before calling app's device callbacks
    DXUTUpdateBackBufferDesc();

    // Setup cursor based on current settings (window/fullscreen mode, show cursor state, clip cursor state)
    DXUTSetupCursor();

    // Update GetDXUTState()'s copy of D3D caps 
    D3DCAPS9* pd3dCaps = GetDXUTState().GetCaps();
    DXUTGetD3D9Device()->GetDeviceCaps( pd3dCaps );

    // Update the device stats text
    CD3D9Enumeration* pd3dEnum = DXUTGetD3D9Enumeration();
    CD3D9EnumAdapterInfo* pAdapterInfo = pd3dEnum->GetAdapterInfo( pNewDeviceSettings->d3d9.AdapterOrdinal );
    DXUTUpdateD3D9DeviceStats( pNewDeviceSettings->d3d9.DeviceType,
                               pNewDeviceSettings->d3d9.BehaviorFlags,
                               &pAdapterInfo->AdapterIdentifier );

    // Call the app's device created callback if non-NULL
    const D3DSURFACE_DESC* pBackBufferSurfaceDesc = DXUTGetD3D9BackBufferSurfaceDesc();
    GetDXUTState().SetInsideDeviceCallback( true );
    LPDXUTCALLBACKD3D9DEVICECREATED pCallbackDeviceCreated = GetDXUTState().GetD3D9DeviceCreatedFunc();
    hr = S_OK;
    if( pCallbackDeviceCreated != NULL )
        hr = pCallbackDeviceCreated( DXUTGetD3D9Device(), pBackBufferSurfaceDesc,
                                     GetDXUTState().GetD3D9DeviceCreatedFuncUserContext() );
    GetDXUTState().SetInsideDeviceCallback( false );
    if( DXUTGetD3D9Device() == NULL ) // Handle DXUTShutdown from inside callback
        return E_FAIL;
    if( FAILED( hr ) )
    {
        DXUT_ERR( L"DeviceCreated callback", hr );
        return ( hr == DXUTERR_MEDIANOTFOUND ) ? DXUTERR_MEDIANOTFOUND : DXUTERR_CREATINGDEVICEOBJECTS;
    }
    GetDXUTState().SetDeviceObjectsCreated( true );

    // Call the app's device reset callback if non-NULL
    GetDXUTState().SetInsideDeviceCallback( true );
    LPDXUTCALLBACKD3D9DEVICERESET pCallbackDeviceReset = GetDXUTState().GetD3D9DeviceResetFunc();
    hr = S_OK;
    if( pCallbackDeviceReset != NULL )
        hr = pCallbackDeviceReset( DXUTGetD3D9Device(), pBackBufferSurfaceDesc,
                                   GetDXUTState().GetD3D9DeviceResetFuncUserContext() );
    GetDXUTState().SetInsideDeviceCallback( false );
    if( DXUTGetD3D9Device() == NULL ) // Handle DXUTShutdown from inside callback
        return E_FAIL;
    if( FAILED( hr ) )
    {
        DXUT_ERR( L"DeviceReset callback", hr );
        return ( hr == DXUTERR_MEDIANOTFOUND ) ? DXUTERR_MEDIANOTFOUND : DXUTERR_RESETTINGDEVICEOBJECTS;
    }
    GetDXUTState().SetDeviceObjectsReset( true );

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Resets the 3D environment by:
//      - Calls the device lost callback 
//      - Resets the device
//      - Stores the back buffer description
//      - Sets up the full screen Direct3D cursor if requested
//      - Calls the device reset callback 
//--------------------------------------------------------------------------------------
HRESULT DXUTReset3DEnvironment9()
{
    HRESULT hr;

    IDirect3DDevice9* pd3dDevice = DXUTGetD3D9Device();
    assert( pd3dDevice != NULL );

    // Call the app's device lost callback
    if( GetDXUTState().GetDeviceObjectsReset() == true )
    {
        GetDXUTState().SetInsideDeviceCallback( true );
        LPDXUTCALLBACKD3D9DEVICELOST pCallbackDeviceLost = GetDXUTState().GetD3D9DeviceLostFunc();
        if( pCallbackDeviceLost != NULL )
            pCallbackDeviceLost( GetDXUTState().GetD3D9DeviceLostFuncUserContext() );
        GetDXUTState().SetDeviceObjectsReset( false );
        GetDXUTState().SetInsideDeviceCallback( false );
    }

    // Reset the device
    DXUTDeviceSettings* pDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();
    hr = pd3dDevice->Reset( &pDeviceSettings->d3d9.pp );
    if( FAILED( hr ) )
    {
        if( hr == D3DERR_DEVICELOST )
            return D3DERR_DEVICELOST; // Reset could legitimately fail if the device is lost
        else
            return DXUT_ERR( L"Reset", DXUTERR_RESETTINGDEVICE );
    }

    // Update back buffer desc before calling app's device callbacks
    DXUTUpdateBackBufferDesc();

    // Setup cursor based on current settings (window/fullscreen mode, show cursor state, clip cursor state)
    DXUTSetupCursor();

    // Call the app's OnDeviceReset callback
    GetDXUTState().SetInsideDeviceCallback( true );
    const D3DSURFACE_DESC* pBackBufferSurfaceDesc = DXUTGetD3D9BackBufferSurfaceDesc();
    LPDXUTCALLBACKD3D9DEVICERESET pCallbackDeviceReset = GetDXUTState().GetD3D9DeviceResetFunc();
    hr = S_OK;
    if( pCallbackDeviceReset != NULL )
        hr = pCallbackDeviceReset( pd3dDevice, pBackBufferSurfaceDesc,
                                   GetDXUTState().GetD3D9DeviceResetFuncUserContext() );
    GetDXUTState().SetInsideDeviceCallback( false );
    if( FAILED( hr ) )
    {
        // If callback failed, cleanup
        DXUT_ERR( L"DeviceResetCallback", hr );
        if( hr != DXUTERR_MEDIANOTFOUND )
            hr = DXUTERR_RESETTINGDEVICEOBJECTS;

        GetDXUTState().SetInsideDeviceCallback( true );
        LPDXUTCALLBACKD3D9DEVICELOST pCallbackDeviceLost = GetDXUTState().GetD3D9DeviceLostFunc();
        if( pCallbackDeviceLost != NULL )
            pCallbackDeviceLost( GetDXUTState().GetD3D9DeviceLostFuncUserContext() );
        GetDXUTState().SetInsideDeviceCallback( false );
        return hr;
    }

    // Success
    GetDXUTState().SetDeviceObjectsReset( true );

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Render the 3D environment by:
//      - Checking if the device is lost and trying to reset it if it is
//      - Get the elapsed time since the last frame
//      - Calling the app's framemove and render callback
//      - Calling Present()
//--------------------------------------------------------------------------------------
void DXUTRender3DEnvironment9()
{
    HRESULT hr;

    if( GetDXUTState().GetDeviceLost() || DXUTIsRenderingPaused() || !DXUTIsActive() )
    {
        // Window is minimized or paused so yield CPU time to other processes
        Sleep( 50 );
    }

    // If no device created yet because device was lost (ie. another fullscreen exclusive device exists), 
    // then wait and try to create every so often.
    IDirect3DDevice9* pd3dDevice = DXUTGetD3D9Device();
    if( NULL == pd3dDevice )
    {
        if( GetDXUTState().GetDeviceLost() )
        {
            DXUTDeviceSettings deviceSettings = DXUTGetDeviceSettings();
            DXUTChangeDevice( &deviceSettings, NULL,  NULL, false, true );
        }

        return;
    }

    if( GetDXUTState().GetDeviceLost() && !GetDXUTState().GetRenderingPaused() )
    {
        // Test the cooperative level to see if it's okay to render.
        if( FAILED( hr = pd3dDevice->TestCooperativeLevel() ) )
        {
            if( D3DERR_DEVICELOST == hr )
            {
                // The device has been lost but cannot be reset at this time.
                // So wait until it can be reset.
                return;
            }

            // If we are windowed, read the desktop format and 
            // ensure that the Direct3D device is using the same format 
            // since the user could have changed the desktop bitdepth 
            if( DXUTIsWindowed() )
            {
                D3DDISPLAYMODE adapterDesktopDisplayMode;
                IDirect3D9* pD3D = DXUTGetD3D9Object();
                DXUTDeviceSettings* pDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();
                pD3D->GetAdapterDisplayMode( pDeviceSettings->d3d9.AdapterOrdinal, &adapterDesktopDisplayMode );
                if( pDeviceSettings->d3d9.AdapterFormat != adapterDesktopDisplayMode.Format )
                {

                    DXUTDeviceSettings deviceSettings = DXUTGetDeviceSettings();
                    deviceSettings.d3d9.AdapterFormat = adapterDesktopDisplayMode.Format;

                    hr = DXUTSnapDeviceSettingsToEnumDevice(&deviceSettings, false);
                    if( FAILED( hr ) ) // the call will fail if no valid devices were found
                    {
                        DXUTDisplayErrorMessage( DXUTERR_NOCOMPATIBLEDEVICES );
                        DXUTShutdown();
                    }

                    // Change to a Direct3D device created from the new device settings.
                    // If there is an existing device, then either reset or recreate the scene
                    hr = DXUTChangeDevice( &deviceSettings, NULL,  NULL, false, false );
                    if( FAILED( hr ) )
                    {
                        // If this fails, try to go fullscreen and if this fails also shutdown.
                        if( FAILED( DXUTToggleFullScreen() ) )
                            DXUTShutdown();
                    }

                    return;
                }
            }

            // Try to reset the device
            if( FAILED( hr = DXUTReset3DEnvironment9() ) )
            {
                if( D3DERR_DEVICELOST == hr )
                {
                    // The device was lost again, so continue waiting until it can be reset.
                    return;
                }
                else if( DXUTERR_RESETTINGDEVICEOBJECTS == hr ||
                         DXUTERR_MEDIANOTFOUND == hr )
                {
                    DXUTDisplayErrorMessage( hr );
                    DXUTShutdown();
                    return;
                }
                else
                {
                    // Reset failed, but the device wasn't lost so something bad happened, 
                    // so recreate the device to try to recover
                    DXUTDeviceSettings* pDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();
                    if( FAILED( DXUTChangeDevice( pDeviceSettings, NULL,  NULL, true, false ) ) )
                    {
                        DXUTShutdown();
                        return;
                    }
                }
            }
        }

        GetDXUTState().SetDeviceLost( false );
    }

    // Get the app's time, in seconds. Skip rendering if no time elapsed
    double fTime, fAbsTime; float fElapsedTime;
    DXUTGetGlobalTimer()->GetTimeValues( &fTime, &fAbsTime, &fElapsedTime );

    // Store the time for the app
    if( GetDXUTState().GetConstantFrameTime() )
    {
        fElapsedTime = GetDXUTState().GetTimePerFrame();
        fTime = DXUTGetTime() + fElapsedTime;
    }

    GetDXUTState().SetTime( fTime );
    GetDXUTState().SetAbsoluteTime( fAbsTime );
    GetDXUTState().SetElapsedTime( fElapsedTime );

    // Update the FPS stats
    DXUTUpdateFrameStats();

    DXUTHandleTimers();

    // Animate the scene by calling the app's frame move callback
    LPDXUTCALLBACKFRAMEMOVE pCallbackFrameMove = GetDXUTState().GetFrameMoveFunc();
    if( pCallbackFrameMove != NULL )
    {
        pCallbackFrameMove( fTime, fElapsedTime, GetDXUTState().GetFrameMoveFuncUserContext() );
        pd3dDevice = DXUTGetD3D9Device();
        if( NULL == pd3dDevice ) // Handle DXUTShutdown from inside callback
            return;
    }

    if( !GetDXUTState().GetRenderingPaused() )
    {
        // Render the scene by calling the app's render callback
        LPDXUTCALLBACKD3D9FRAMERENDER pCallbackFrameRender = GetDXUTState().GetD3D9FrameRenderFunc();
        if( pCallbackFrameRender != NULL )
        {
            pCallbackFrameRender( pd3dDevice, fTime, fElapsedTime,
                                  GetDXUTState().GetD3D9FrameRenderFuncUserContext() );
            pd3dDevice = DXUTGetD3D9Device();
            if( NULL == pd3dDevice ) // Handle DXUTShutdown from inside callback
                return;
        }

#if defined(DEBUG) || defined(_DEBUG)
        // The back buffer should always match the client rect 
        // if the Direct3D backbuffer covers the entire window
        RECT rcClient;
        GetClientRect( DXUTGetHWND(), &rcClient );
        if( !IsIconic( DXUTGetHWND() ) )
        {
            GetClientRect( DXUTGetHWND(), &rcClient );
            assert( DXUTGetD3D9BackBufferSurfaceDesc()->Width == (UINT)rcClient.right );
            assert( DXUTGetD3D9BackBufferSurfaceDesc()->Height == (UINT)rcClient.bottom );
        }
#endif

        // Show the frame on the primary surface.
        hr = pd3dDevice->Present( NULL, NULL, NULL, NULL );
        if( FAILED( hr ) )
        {
            if( D3DERR_DEVICELOST == hr )
            {
                GetDXUTState().SetDeviceLost( true );
            }
            else if( D3DERR_DRIVERINTERNALERROR == hr )
            {
                // When D3DERR_DRIVERINTERNALERROR is returned from Present(),
                // the application can do one of the following:
                // 
                // - End, with the pop-up window saying that the application cannot continue 
                //   because of problems in the display adapter and that the user should 
                //   contact the adapter manufacturer.
                //
                // - Attempt to restart by calling IDirect3DDevice9::Reset, which is essentially the same 
                //   path as recovering from a lost device. If IDirect3DDevice9::Reset fails with 
                //   D3DERR_DRIVERINTERNALERROR, the application should end immediately with the message 
                //   that the user should contact the adapter manufacturer.
                // 
                // The framework attempts the path of resetting the device
                // 
                GetDXUTState().SetDeviceLost( true );
            }
        }
    }

    // If the app called DXUTWasKeyPressed() then do the work 
    // to store the current state of the keys in bLastKeys
    if( GetDXUTState().GetAppCalledWasKeyPressed() )
    {
        bool* bLastKeys = GetDXUTState().GetLastKeys();
        bool* bKeys = GetDXUTState().GetKeys();
        memcpy( bLastKeys, bKeys, sizeof( bool ) * 256 );
    }

    // Update current frame #
    int nFrame = GetDXUTState().GetCurrentFrameNumber();
    nFrame++;
    GetDXUTState().SetCurrentFrameNumber( nFrame );

    // Check to see if the app should shutdown due to cmdline
    if( GetDXUTState().GetOverrideQuitAfterFrame() != 0 )
    {
        if( nFrame > GetDXUTState().GetOverrideQuitAfterFrame() )
            DXUTShutdown();
    }

    return;
}


//--------------------------------------------------------------------------------------
// Cleans up the 3D environment by:
//      - Calls the device lost callback 
//      - Calls the device destroyed callback 
//      - Releases the D3D device
//--------------------------------------------------------------------------------------
void DXUTCleanup3DEnvironment9( bool bReleaseSettings )
{
    IDirect3DDevice9* pd3dDevice = DXUTGetD3D9Device();
    if( pd3dDevice != NULL )
    {
        GetDXUTState().SetInsideDeviceCallback( true );

        // Call the app's device lost callback
        if( GetDXUTState().GetDeviceObjectsReset() == true )
        {
            LPDXUTCALLBACKD3D9DEVICELOST pCallbackDeviceLost = GetDXUTState().GetD3D9DeviceLostFunc();
            if( pCallbackDeviceLost != NULL )
                pCallbackDeviceLost( GetDXUTState().GetD3D9DeviceLostFuncUserContext() );
            GetDXUTState().SetDeviceObjectsReset( false );
        }

        // Call the app's device destroyed callback
        if( GetDXUTState().GetDeviceObjectsCreated() == true )
        {
            LPDXUTCALLBACKD3D9DEVICEDESTROYED pCallbackDeviceDestroyed = GetDXUTState().GetD3D9DeviceDestroyedFunc();
            if( pCallbackDeviceDestroyed != NULL )
                pCallbackDeviceDestroyed( GetDXUTState().GetD3D9DeviceDestroyedFuncUserContext() );
            GetDXUTState().SetDeviceObjectsCreated( false );
        }

        GetDXUTState().SetInsideDeviceCallback( false );

        // Release the D3D device and in debug configs, displays a message box if there 
        // are unrelease objects.
        if( pd3dDevice )
        {
            UINT references = pd3dDevice->Release();
            if( references > 0 )
            {
                DXUTDisplayErrorMessage( DXUTERR_NONZEROREFCOUNT );
                DXUT_ERR( L"DXUTCleanup3DEnvironment", DXUTERR_NONZEROREFCOUNT );
            }
        }
        GetDXUTState().SetD3D9Device( NULL );

        if( bReleaseSettings )
        {
            DXUTDeviceSettings* pOldDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();
            SAFE_DELETE(pOldDeviceSettings);
            GetDXUTState().SetCurrentDeviceSettings( NULL );
        }

        D3DSURFACE_DESC* pBackBufferSurfaceDesc = GetDXUTState().GetBackBufferSurfaceDesc9();
        ZeroMemory( pBackBufferSurfaceDesc, sizeof( D3DSURFACE_DESC ) );

        D3DCAPS9* pd3dCaps = GetDXUTState().GetCaps();
        ZeroMemory( pd3dCaps, sizeof( D3DCAPS9 ) );

        GetDXUTState().SetDeviceCreated( false );
    }
}


//--------------------------------------------------------------------------------------
// Gives the D3D device a cursor with image and hotspot from hCursor.
//--------------------------------------------------------------------------------------
HRESULT DXUTSetD3D9DeviceCursor( IDirect3DDevice9* pd3dDevice, HCURSOR hCursor, bool bAddWatermark )
{
    HRESULT hr = E_FAIL;
    ICONINFO iconinfo;
    bool bBWCursor = false;
    LPDIRECT3DSURFACE9 pCursorSurface = NULL;
    HDC hdcColor = NULL;
    HDC hdcMask = NULL;
    HDC hdcScreen = NULL;
    BITMAP bm;
    DWORD dwWidth = 0;
    DWORD dwHeightSrc = 0;
    DWORD dwHeightDest = 0;
    COLORREF crColor;
    COLORREF crMask;
    UINT x;
    UINT y;
    BITMAPINFO bmi;
    COLORREF* pcrArrayColor = NULL;
    COLORREF* pcrArrayMask = NULL;
    DWORD* pBitmap;
    HGDIOBJ hgdiobjOld;

    ZeroMemory( &iconinfo, sizeof( iconinfo ) );
    if( !GetIconInfo( hCursor, &iconinfo ) )
        goto End;

    if( 0 == GetObject( ( HGDIOBJ )iconinfo.hbmMask, sizeof( BITMAP ), ( LPVOID )&bm ) )
        goto End;
    dwWidth = bm.bmWidth;
    dwHeightSrc = bm.bmHeight;

    if( iconinfo.hbmColor == NULL )
    {
        bBWCursor = TRUE;
        dwHeightDest = dwHeightSrc / 2;
    }
    else
    {
        bBWCursor = FALSE;
        dwHeightDest = dwHeightSrc;
    }

    // Create a surface for the fullscreen cursor
    if( FAILED( hr = pd3dDevice->CreateOffscreenPlainSurface( dwWidth, dwHeightDest,
                                                              D3DFMT_A8R8G8B8, D3DPOOL_SCRATCH, &pCursorSurface,
                                                              NULL ) ) )
    {
        goto End;
    }

    pcrArrayMask = new DWORD[dwWidth * dwHeightSrc];

    ZeroMemory( &bmi, sizeof( bmi ) );
    bmi.bmiHeader.biSize = sizeof( bmi.bmiHeader );
    bmi.bmiHeader.biWidth = dwWidth;
    bmi.bmiHeader.biHeight = dwHeightSrc;
    bmi.bmiHeader.biPlanes = 1;
    bmi.bmiHeader.biBitCount = 32;
    bmi.bmiHeader.biCompression = BI_RGB;

    hdcScreen = GetDC( NULL );
    hdcMask = CreateCompatibleDC( hdcScreen );
    if( hdcMask == NULL )
    {
        hr = E_FAIL;
        goto End;
    }
    hgdiobjOld = SelectObject( hdcMask, iconinfo.hbmMask );
    GetDIBits( hdcMask, iconinfo.hbmMask, 0, dwHeightSrc,
               pcrArrayMask, &bmi, DIB_RGB_COLORS );
    SelectObject( hdcMask, hgdiobjOld );

    if( !bBWCursor )
    {
        pcrArrayColor = new DWORD[dwWidth * dwHeightDest];
        hdcColor = CreateCompatibleDC( hdcScreen );
        if( hdcColor == NULL )
        {
            hr = E_FAIL;
            goto End;
        }
        SelectObject( hdcColor, iconinfo.hbmColor );
        GetDIBits( hdcColor, iconinfo.hbmColor, 0, dwHeightDest,
                   pcrArrayColor, &bmi, DIB_RGB_COLORS );
    }

    // Transfer cursor image into the surface
    D3DLOCKED_RECT lr;
    pCursorSurface->LockRect( &lr, NULL, 0 );
    pBitmap = ( DWORD* )lr.pBits;
    for( y = 0; y < dwHeightDest; y++ )
    {
        for( x = 0; x < dwWidth; x++ )
        {
            if( bBWCursor )
            {
                crColor = pcrArrayMask[dwWidth * ( dwHeightDest - 1 - y ) + x];
                crMask = pcrArrayMask[dwWidth * ( dwHeightSrc - 1 - y ) + x];
            }
            else
            {
                crColor = pcrArrayColor[dwWidth * ( dwHeightDest - 1 - y ) + x];
                crMask = pcrArrayMask[dwWidth * ( dwHeightDest - 1 - y ) + x];
            }
            if( crMask == 0 )
                pBitmap[dwWidth * y + x] = 0xff000000 | crColor;
            else
                pBitmap[dwWidth * y + x] = 0x00000000;

            // It may be helpful to make the D3D cursor look slightly 
            // different from the Windows cursor so you can distinguish 
            // between the two when developing/testing code.  When
            // bAddWatermark is TRUE, the following code adds some
            // small grey "D3D" characters to the upper-left corner of
            // the D3D cursor image.
            if( bAddWatermark && x < 12 && y < 5 )
            {
                // 11.. 11.. 11.. .... CCC0
                // 1.1. ..1. 1.1. .... A2A0
                // 1.1. .1.. 1.1. .... A4A0
                // 1.1. ..1. 1.1. .... A2A0
                // 11.. 11.. 11.. .... CCC0

                const WORD wMask[5] = { 0xccc0, 0xa2a0, 0xa4a0, 0xa2a0, 0xccc0 };
                if( wMask[y] & (1 << (15 - x)) )
                {
                    pBitmap[dwWidth*y + x] |= 0xff808080;
                }
            }
        }
    }
    pCursorSurface->UnlockRect();

    // Set the device cursor
    if( FAILED( hr = pd3dDevice->SetCursorProperties( iconinfo.xHotspot,
                                                      iconinfo.yHotspot, pCursorSurface ) ) )
    {
        goto End;
    }

    hr = S_OK;

End:
    if( iconinfo.hbmMask != NULL )
        DeleteObject( iconinfo.hbmMask );
    if( iconinfo.hbmColor != NULL )
        DeleteObject( iconinfo.hbmColor );
    if( hdcScreen != NULL )
        ReleaseDC( NULL, hdcScreen );
    if( hdcColor != NULL )
        DeleteDC( hdcColor );
    if( hdcMask != NULL )
        DeleteDC( hdcMask );
    SAFE_DELETE_ARRAY( pcrArrayColor );
    SAFE_DELETE_ARRAY( pcrArrayMask );
    SAFE_RELEASE( pCursorSurface );
    return hr;
}


//--------------------------------------------------------------------------------------
// Internal helper function to return the adapter format from the first device settings 
// combo that matches the passed adapter ordinal, device type, backbuffer format, and windowed.  
//--------------------------------------------------------------------------------------
HRESULT DXUTFindD3D9AdapterFormat( UINT AdapterOrdinal, D3DDEVTYPE DeviceType, D3DFORMAT BackBufferFormat,
                                   BOOL Windowed, D3DFORMAT* pAdapterFormat )
{
    CD3D9Enumeration* pd3dEnum = DXUTGetD3D9Enumeration( false );
    CD3D9EnumDeviceInfo* pDeviceInfo = pd3dEnum->GetDeviceInfo( AdapterOrdinal, DeviceType );
    if( pDeviceInfo )
    {
        for( int iDeviceCombo = 0; iDeviceCombo < pDeviceInfo->deviceSettingsComboList.GetSize(); iDeviceCombo++ )
        {
            CD3D9EnumDeviceSettingsCombo* pDeviceSettingsCombo = pDeviceInfo->deviceSettingsComboList.GetAt(
                iDeviceCombo );
            if( pDeviceSettingsCombo->BackBufferFormat == BackBufferFormat &&
                pDeviceSettingsCombo->Windowed == Windowed )
            {
                // Return the adapter format from the first match
                *pAdapterFormat = pDeviceSettingsCombo->AdapterFormat;
                return S_OK;
            }
        }
    }

    *pAdapterFormat = BackBufferFormat;
    return E_FAIL;
}

//--------------------------------------------------------------------------------------
// Sets the viewport, render target view, and depth stencil view.
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTSetupD3D11Views( ID3D11DeviceContext* pd3dDeviceContext )
{
    HRESULT hr = S_OK;

    // Setup the viewport to match the backbuffer
    D3D11_VIEWPORT vp;
    vp.Width = (FLOAT)DXUTGetDXGIBackBufferSurfaceDesc()->Width;
    vp.Height = (FLOAT)DXUTGetDXGIBackBufferSurfaceDesc()->Height;
    vp.MinDepth = 0;
    vp.MaxDepth = 1;
    vp.TopLeftX = 0;
    vp.TopLeftY = 0;
    pd3dDeviceContext->RSSetViewports( 1, &vp );

    // Set the render targets
    ID3D11RenderTargetView* pRTV = GetDXUTState().GetD3D11RenderTargetView();
    ID3D11DepthStencilView* pDSV = GetDXUTState().GetD3D11DepthStencilView();
    pd3dDeviceContext->OMSetRenderTargets( 1, &pRTV, pDSV );

    return hr;
}


//--------------------------------------------------------------------------------------
// Creates a render target view, and depth stencil texture and view.
//--------------------------------------------------------------------------------------
HRESULT DXUTCreateD3D11Views( ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dImmediateContext,
                             DXUTDeviceSettings* pDeviceSettings )
{
    HRESULT hr = S_OK;
    IDXGISwapChain* pSwapChain = DXUTGetDXGISwapChain();
    ID3D11DepthStencilView* pDSV = NULL;
    ID3D11RenderTargetView* pRTV = NULL;

    // Get the back buffer and desc
    ID3D11Texture2D* pBackBuffer;
    hr = pSwapChain->GetBuffer( 0, __uuidof( *pBackBuffer ), ( LPVOID* )&pBackBuffer );
    if( FAILED( hr ) )
        return hr;
    D3D11_TEXTURE2D_DESC backBufferSurfaceDesc;
    pBackBuffer->GetDesc( &backBufferSurfaceDesc );

    // Create the render target view
    hr = pd3dDevice->CreateRenderTargetView( pBackBuffer, NULL, &pRTV );
    SAFE_RELEASE( pBackBuffer );
    if( FAILED( hr ) )
        return hr;
    GetDXUTState().SetD3D11RenderTargetView( pRTV );

    if( pDeviceSettings->d3d11.AutoCreateDepthStencil )
    {
        // Create depth stencil texture
        ID3D11Texture2D* pDepthStencil = NULL;
        D3D11_TEXTURE2D_DESC descDepth;
        descDepth.Width = backBufferSurfaceDesc.Width;
        descDepth.Height = backBufferSurfaceDesc.Height;
        descDepth.MipLevels = 1;
        descDepth.ArraySize = 1;
        descDepth.Format = pDeviceSettings->d3d11.AutoDepthStencilFormat;
        descDepth.SampleDesc.Count = pDeviceSettings->d3d11.sd.SampleDesc.Count;
        descDepth.SampleDesc.Quality = pDeviceSettings->d3d11.sd.SampleDesc.Quality;
        descDepth.Usage = D3D11_USAGE_DEFAULT;
        descDepth.BindFlags = D3D11_BIND_DEPTH_STENCIL;
        descDepth.CPUAccessFlags = 0;
        descDepth.MiscFlags = 0;
        hr = pd3dDevice->CreateTexture2D( &descDepth, NULL, &pDepthStencil );
        if( FAILED( hr ) )
            return hr;
        GetDXUTState().SetD3D11DepthStencil( pDepthStencil );

        // Create the depth stencil view
        D3D11_DEPTH_STENCIL_VIEW_DESC descDSV;
        descDSV.Format = descDepth.Format;
        descDSV.Flags = 0;
        if( descDepth.SampleDesc.Count > 1 )
            descDSV.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2DMS;
        else
            descDSV.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
        descDSV.Texture2D.MipSlice = 0;
        hr = pd3dDevice->CreateDepthStencilView( pDepthStencil, &descDSV, &pDSV );
        if( FAILED( hr ) )
            return hr;
        GetDXUTState().SetD3D11DepthStencilView( pDSV );
    }

    hr = DXUTSetupD3D11Views( pd3dImmediateContext );
    if( FAILED( hr ) )
        return hr;

    return hr;
}


//--------------------------------------------------------------------------------------
// Creates the 3D environment
//--------------------------------------------------------------------------------------
HRESULT DXUTCreate3DEnvironment11( ID3D11Device* pd3d11DeviceFromApp )
{
    HRESULT hr = S_OK;

    ID3D11Device* pd3d11Device = NULL;
    ID3D11DeviceContext* pd3dImmediateContext = NULL;
    D3D_FEATURE_LEVEL FeatureLevel = D3D_FEATURE_LEVEL_11_0;

    IDXGISwapChain* pSwapChain = NULL;
    DXUTDeviceSettings* pNewDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();
        
    IDXGIFactory1* pDXGIFactory = DXUTGetDXGIFactory();
    hr = pDXGIFactory->MakeWindowAssociation( DXUTGetHWND(), 0  );

    // Only create a Direct3D device if one hasn't been supplied by the app
    if( pd3d11DeviceFromApp == NULL )
    {
        // Try to create the device with the chosen settings
        IDXGIAdapter1* pAdapter = NULL;

        hr = S_OK;
        D3D_DRIVER_TYPE ddt = pNewDeviceSettings->d3d11.DriverType;
        if( pNewDeviceSettings->d3d11.DriverType == D3D_DRIVER_TYPE_HARDWARE ) 
        {
            hr = pDXGIFactory->EnumAdapters1( pNewDeviceSettings->d3d11.AdapterOrdinal, &pAdapter );
            if ( FAILED( hr) ) 
            {
                return E_FAIL;
            }
            ddt = D3D_DRIVER_TYPE_UNKNOWN;    
        }
        else if (pNewDeviceSettings->d3d11.DriverType == D3D_DRIVER_TYPE_WARP) 
        {
            ddt = D3D_DRIVER_TYPE_WARP;  
            pAdapter = NULL;
        }
        else if (pNewDeviceSettings->d3d11.DriverType == D3D_DRIVER_TYPE_REFERENCE) 
        {
            ddt = D3D_DRIVER_TYPE_REFERENCE;
            pAdapter = NULL;
        }

        if( SUCCEEDED( hr ) )
        {

            hr = DXUT_Dynamic_D3D11CreateDevice( pAdapter,
                                                 ddt,
                                                 ( HMODULE )0,
                                                 pNewDeviceSettings->d3d11.CreateFlags,
                                                 &pNewDeviceSettings->d3d11.DeviceFeatureLevel,
                                                 1,
                                                 D3D11_SDK_VERSION,
                                                 &pd3d11Device,
                                                 &FeatureLevel,
                                                 &pd3dImmediateContext
                                                 );
            
            if ( FAILED( hr ) ) {
                pAdapter = NULL;
                // Remote desktop does not allow you to enumerate the adapter.  In this case, we let D3D11 do the enumeration.
                if ( ddt == D3D_DRIVER_TYPE_UNKNOWN ) { 
                    hr = DXUT_Dynamic_D3D11CreateDevice( pAdapter,
                                                         D3D_DRIVER_TYPE_HARDWARE,
                                                         ( HMODULE )0,
                                                         pNewDeviceSettings->d3d11.CreateFlags,
                                                         &pNewDeviceSettings->d3d11.DeviceFeatureLevel,
                                                         1,
                                                         D3D11_SDK_VERSION,
                                                         &pd3d11Device,
                                                         &FeatureLevel,
                                                         &pd3dImmediateContext
                                                         );
                }
                if ( FAILED ( hr ) ) {
                    DXUT_ERR( L"D3D11CreateDevice", hr );
                    return DXUTERR_CREATINGDEVICE;
                }
            }
        }

        if( SUCCEEDED( hr ) )
        {
            IDXGIDevice1* pDXGIDev = NULL;
            hr = pd3d11Device->QueryInterface( __uuidof( IDXGIDevice1 ), ( LPVOID* )&pDXGIDev );
            if( SUCCEEDED( hr ) && pDXGIDev )
            {
                if ( pAdapter == NULL ) 
                {
                    IDXGIAdapter *pTempAdapter;
                    pDXGIDev->GetAdapter( &pTempAdapter );
                    V_RETURN( pTempAdapter->QueryInterface( __uuidof( IDXGIAdapter1 ), (LPVOID*) &pAdapter ) );
                    V_RETURN( pAdapter->GetParent( __uuidof( IDXGIFactory1 ), (LPVOID*) &pDXGIFactory ) );
                    SAFE_RELEASE ( pTempAdapter );
                    GetDXUTState().SetDXGIFactory( pDXGIFactory );
                }
            }
            SAFE_RELEASE( pDXGIDev );
            GetDXUTState().SetDXGIAdapter( pAdapter );
        }

        // set default render state to msaa enabled
        D3D11_RASTERIZER_DESC drd = {
            D3D11_FILL_SOLID, //D3D11_FILL_MODE FillMode;
            D3D11_CULL_BACK,//D3D11_CULL_MODE CullMode;
            FALSE, //BOOL FrontCounterClockwise;
            0, //INT DepthBias;
            0.0f,//FLOAT DepthBiasClamp;
            0.0f,//FLOAT SlopeScaledDepthBias;
            TRUE,//BOOL DepthClipEnable;
            FALSE,//BOOL ScissorEnable;
            TRUE,//BOOL MultisampleEnable;
            FALSE//BOOL AntialiasedLineEnable;        
        };
        if( FAILED( hr ) )
        {
            DXUT_ERR( L"D3D11CreateDevice", hr );
            return DXUTERR_CREATINGDEVICE;
        }
        ID3D11RasterizerState* pRS = NULL;
        pd3d11Device->CreateRasterizerState(&drd, &pRS);
        GetDXUTState().SetD3D11RasterizerState(pRS);
        pd3dImmediateContext->RSSetState(pRS);




        // Enumerate its outputs.
        UINT OutputCount, iOutput;
        for( OutputCount = 0; ; ++OutputCount )
        {
            IDXGIOutput* pOutput;
            if( FAILED( pAdapter->EnumOutputs( OutputCount, &pOutput ) ) )
                break;
            SAFE_RELEASE( pOutput );
        }
        IDXGIOutput** ppOutputArray = new IDXGIOutput*[OutputCount];
        if( !ppOutputArray )
            return E_OUTOFMEMORY;
        for( iOutput = 0; iOutput < OutputCount; ++iOutput )
            pAdapter->EnumOutputs( iOutput, ppOutputArray + iOutput );
        GetDXUTState().SetDXGIOutputArray( ppOutputArray );
        GetDXUTState().SetDXGIOutputArraySize( OutputCount );

        // Create the swapchain
        
        hr = pDXGIFactory->CreateSwapChain( pd3d11Device, &pNewDeviceSettings->d3d11.sd, &pSwapChain );
        
        if( FAILED( hr ) )
        {
            DXUT_ERR( L"CreateSwapChain", hr );
            return DXUTERR_CREATINGDEVICE;
        }
    }
    else
    {
        pd3d11DeviceFromApp->AddRef();
        pd3d11Device = pd3d11DeviceFromApp;
    }

    GetDXUTState().SetD3D11Device( pd3d11Device );
    GetDXUTState().SetD3D11DeviceContext( pd3dImmediateContext );
    GetDXUTState().SetD3D11FeatureLevel( FeatureLevel );
    GetDXUTState().SetDXGISwapChain( pSwapChain );

    // If switching to REF, set the exit code to 11.  If switching to HAL and exit code was 11, then set it back to 0.
    if( pNewDeviceSettings->d3d11.DriverType == D3D_DRIVER_TYPE_REFERENCE && GetDXUTState().GetExitCode() == 0 )
        GetDXUTState().SetExitCode( 10 );
    else if( pNewDeviceSettings->d3d11.DriverType == D3D_DRIVER_TYPE_HARDWARE && GetDXUTState().GetExitCode() == 10 )
        GetDXUTState().SetExitCode( 0 );

    // Update back buffer desc before calling app's device callbacks
    DXUTUpdateBackBufferDesc();

    // Setup cursor based on current settings (window/fullscreen mode, show cursor state, clip cursor state)
    DXUTSetupCursor();

    // Update the device stats text
    CD3D11Enumeration* pd3dEnum = DXUTGetD3D11Enumeration();
    CD3D11EnumAdapterInfo* pAdapterInfo = pd3dEnum->GetAdapterInfo( pNewDeviceSettings->d3d11.AdapterOrdinal );
    DXUTUpdateD3D11DeviceStats( pNewDeviceSettings->d3d11.DriverType, &pAdapterInfo->AdapterDesc );

    // Call the app's device created callback if non-NULL
    const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc = DXUTGetDXGIBackBufferSurfaceDesc();
    GetDXUTState().SetInsideDeviceCallback( true );
    LPDXUTCALLBACKD3D11DEVICECREATED pCallbackDeviceCreated = GetDXUTState().GetD3D11DeviceCreatedFunc();
    hr = S_OK;
    if( pCallbackDeviceCreated != NULL )
        hr = pCallbackDeviceCreated( DXUTGetD3D11Device(), pBackBufferSurfaceDesc,
                                     GetDXUTState().GetD3D11DeviceCreatedFuncUserContext() );
    GetDXUTState().SetInsideDeviceCallback( false );
    if( DXUTGetD3D11Device() == NULL ) // Handle DXUTShutdown from inside callback
        return E_FAIL;
    if( FAILED( hr ) )
    {
        DXUT_ERR( L"DeviceCreated callback", hr );
        return ( hr == DXUTERR_MEDIANOTFOUND ) ? DXUTERR_MEDIANOTFOUND : DXUTERR_CREATINGDEVICEOBJECTS;
    }
    GetDXUTState().SetDeviceObjectsCreated( true );

    // Setup the render target view and viewport
    hr = DXUTCreateD3D11Views( pd3d11Device, pd3dImmediateContext, pNewDeviceSettings );
    if( FAILED( hr ) )
    {
        DXUT_ERR( L"DXUTCreateD3D11Views", hr );
        return DXUTERR_CREATINGDEVICEOBJECTS;
    }

    // Create performance counters
    //DXUTCreateD3D11Counters( pd3d11Device );

    // Call the app's swap chain reset callback if non-NULL
    GetDXUTState().SetInsideDeviceCallback( true );
    LPDXUTCALLBACKD3D11SWAPCHAINRESIZED pCallbackSwapChainResized = GetDXUTState().GetD3D11SwapChainResizedFunc();
    hr = S_OK;
    if( pCallbackSwapChainResized != NULL )
        hr = pCallbackSwapChainResized( DXUTGetD3D11Device(), pSwapChain, pBackBufferSurfaceDesc,
                                        GetDXUTState().GetD3D11SwapChainResizedFuncUserContext() );
    GetDXUTState().SetInsideDeviceCallback( false );
    if( DXUTGetD3D11Device() == NULL ) // Handle DXUTShutdown from inside callback
        return E_FAIL;
    if( FAILED( hr ) )
    {
        DXUT_ERR( L"DeviceReset callback", hr );
        return ( hr == DXUTERR_MEDIANOTFOUND ) ? DXUTERR_MEDIANOTFOUND : DXUTERR_RESETTINGDEVICEOBJECTS;
    }
    GetDXUTState().SetDeviceObjectsReset( true );

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Resets the 3D environment by:
//      - Calls the device lost callback 
//      - Resets the device
//      - Stores the back buffer description
//      - Sets up the full screen Direct3D cursor if requested
//      - Calls the device reset callback 
//--------------------------------------------------------------------------------------
HRESULT DXUTReset3DEnvironment11()
{
    HRESULT hr;

    GetDXUTState().SetDeviceObjectsReset( false );
    DXUTPause( true, true );

    bool bDeferredDXGIAction = false;
    DXUTDeviceSettings* pDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();
    IDXGISwapChain* pSwapChain = DXUTGetDXGISwapChain();

    DXGI_SWAP_CHAIN_DESC SCDesc;
    pSwapChain->GetDesc( &SCDesc );

    // Resize backbuffer and target of the swapchain in case they have changed.
    // For windowed mode, use the client rect as the desired size. Unlike D3D9,
    // we can't use 0 for width or height.  Therefore, fill in the values from
    // the window size. For fullscreen mode, the width and height should have
    // already been filled with the desktop resolution, so don't change it.
    if( pDeviceSettings->d3d11.sd.Windowed && SCDesc.Windowed )
    {
        RECT rcWnd;
        GetClientRect( DXUTGetHWND(), &rcWnd );
        pDeviceSettings->d3d11.sd.BufferDesc.Width = rcWnd.right - rcWnd.left;
        pDeviceSettings->d3d11.sd.BufferDesc.Height = rcWnd.bottom - rcWnd.top;
    }

    // If the app wants to switch from windowed to fullscreen or vice versa,
    // call the swapchain's SetFullscreenState
    // mode.
    if( SCDesc.Windowed != pDeviceSettings->d3d11.sd.Windowed )
    {
        // Set the fullscreen state
        if( pDeviceSettings->d3d11.sd.Windowed )
        {
            V_RETURN( pSwapChain->SetFullscreenState( FALSE, NULL ) );
            bDeferredDXGIAction = true;
        }
        else
        {
            // Set fullscreen state by setting the display mode to fullscreen, then changing the resolution
            // to the desired value.

            // SetFullscreenState causes a WM_SIZE message to be sent to the window.  The WM_SIZE message calls
            // DXUTCheckForDXGIBufferChange which normally stores the new height and width in 
            // pDeviceSettings->d3d11.sd.BufferDesc.  SetDoNotStoreBufferSize tells DXUTCheckForDXGIBufferChange
            // not to store the height and width so that we have the correct values when calling ResizeTarget.

            GetDXUTState().SetDoNotStoreBufferSize( true );
            V_RETURN( pSwapChain->SetFullscreenState( TRUE, NULL ) );
            GetDXUTState().SetDoNotStoreBufferSize( false );

            V_RETURN( pSwapChain->ResizeTarget( &pDeviceSettings->d3d11.sd.BufferDesc ) );
            bDeferredDXGIAction = true;
        }
    }
    else
    {
        if( pDeviceSettings->d3d11.sd.BufferDesc.Width == SCDesc.BufferDesc.Width &&
            pDeviceSettings->d3d11.sd.BufferDesc.Height == SCDesc.BufferDesc.Height &&
            pDeviceSettings->d3d11.sd.BufferDesc.Format != SCDesc.BufferDesc.Format )
        {
            DXUTResizeDXGIBuffers( 0, 0, !pDeviceSettings->d3d11.sd.Windowed );
            bDeferredDXGIAction = true;
        }
        else if( pDeviceSettings->d3d11.sd.BufferDesc.Width != SCDesc.BufferDesc.Width ||
                 pDeviceSettings->d3d11.sd.BufferDesc.Height != SCDesc.BufferDesc.Height )
        {
            V_RETURN( pSwapChain->ResizeTarget( &pDeviceSettings->d3d11.sd.BufferDesc ) );
            bDeferredDXGIAction = true;
        }
    }

    // If no deferred DXGI actions are to take place, mark the device as reset.
    // If there is a deferred DXGI action, then the device isn't reset until DXGI sends us a 
    // window message.  Only then can we mark the device as reset.
    if( !bDeferredDXGIAction )
        GetDXUTState().SetDeviceObjectsReset( true );
    DXUTPause( false, false );

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Render the 3D environment by:
//      - Checking if the device is lost and trying to reset it if it is
//      - Get the elapsed time since the last frame
//      - Calling the app's framemove and render callback
//      - Calling Present()
//--------------------------------------------------------------------------------------
void DXUTRender3DEnvironment11()
{
    HRESULT hr;

    ID3D11Device* pd3dDevice = DXUTGetD3D11Device();
    if( NULL == pd3dDevice )
        return;

    ID3D11DeviceContext* pd3dImmediateContext = DXUTGetD3D11DeviceContext();
    if( NULL == pd3dImmediateContext )
        return;

    IDXGISwapChain* pSwapChain = DXUTGetDXGISwapChain();
    if( NULL == pSwapChain )
        return;

    if( DXUTIsRenderingPaused() || !DXUTIsActive() || GetDXUTState().GetRenderingOccluded() )
    {
        // Window is minimized/paused/occluded/or not exclusive so yield CPU time to other processes
        Sleep( 50 );
    }

    // Get the app's time, in seconds. Skip rendering if no time elapsed
    double fTime, fAbsTime; float fElapsedTime;
    DXUTGetGlobalTimer()->GetTimeValues( &fTime, &fAbsTime, &fElapsedTime );

    // Store the time for the app
    if( GetDXUTState().GetConstantFrameTime() )
    {
        fElapsedTime = GetDXUTState().GetTimePerFrame();
        fTime = DXUTGetTime() + fElapsedTime;
    }

    GetDXUTState().SetTime( fTime );
    GetDXUTState().SetAbsoluteTime( fAbsTime );
    GetDXUTState().SetElapsedTime( fElapsedTime );

    // Start Performance Counters

    // Update the FPS stats
    DXUTUpdateFrameStats();

    DXUTHandleTimers();

    // Animate the scene by calling the app's frame move callback
    LPDXUTCALLBACKFRAMEMOVE pCallbackFrameMove = GetDXUTState().GetFrameMoveFunc();
    if( pCallbackFrameMove != NULL )
    {
        pCallbackFrameMove( fTime, fElapsedTime, GetDXUTState().GetFrameMoveFuncUserContext() );
        pd3dDevice = DXUTGetD3D11Device();
        if( NULL == pd3dDevice ) // Handle DXUTShutdown from inside callback
            return;
    }

    if( !GetDXUTState().GetRenderingPaused() )
    {
        // Render the scene by calling the app's render callback
        LPDXUTCALLBACKD3D11FRAMERENDER pCallbackFrameRender = GetDXUTState().GetD3D11FrameRenderFunc();
        if( pCallbackFrameRender != NULL && !GetDXUTState().GetRenderingOccluded() )
        {
            pCallbackFrameRender( pd3dDevice, pd3dImmediateContext, fTime, fElapsedTime,
                                  GetDXUTState().GetD3D11FrameRenderFuncUserContext() );
            
            pd3dDevice = DXUTGetD3D11Device();
            if( NULL == pd3dDevice ) // Handle DXUTShutdown from inside callback
                return;
        }

#if defined(DEBUG) || defined(_DEBUG)
        // The back buffer should always match the client rect 
        // if the Direct3D backbuffer covers the entire window
        RECT rcClient;
        GetClientRect( DXUTGetHWND(), &rcClient );
        if( !IsIconic( DXUTGetHWND() ) )
        {
            GetClientRect( DXUTGetHWND(), &rcClient );
            
            assert( DXUTGetDXGIBackBufferSurfaceDesc()->Width == (UINT)rcClient.right );
            assert( DXUTGetDXGIBackBufferSurfaceDesc()->Height == (UINT)rcClient.bottom );
        }
#endif
    }

    if ( GetDXUTState().GetSaveScreenShot() ) {
        DXUTSnapD3D11Screenshot( GetDXUTState().GetScreenShotName(), D3DX11_IFF_BMP );
    }
    if ( GetDXUTState().GetExitAfterScreenShot() ) {
        DXUTShutdown();
        return;
    }

    DWORD dwFlags = 0;
    if( GetDXUTState().GetRenderingOccluded() )
        dwFlags = DXGI_PRESENT_TEST;
    else
        dwFlags = GetDXUTState().GetCurrentDeviceSettings()->d3d11.PresentFlags;
    UINT SyncInterval = GetDXUTState().GetCurrentDeviceSettings()->d3d11.SyncInterval;

    // Show the frame on the primary surface.
    hr = pSwapChain->Present( SyncInterval, dwFlags );
    if( DXGI_STATUS_OCCLUDED == hr )
    {
        // There is a window covering our entire rendering area.
        // Don't render until we're visible again.
        GetDXUTState().SetRenderingOccluded( true );
    }
    else if( DXGI_ERROR_DEVICE_RESET == hr )
    {
        // If a mode change happened, we must reset the device
        if( FAILED( hr = DXUTReset3DEnvironment11() ) )
        {
            if( DXUTERR_RESETTINGDEVICEOBJECTS == hr ||
                DXUTERR_MEDIANOTFOUND == hr )
            {
                DXUTDisplayErrorMessage( hr );
                DXUTShutdown();
                return;
            }
            else
            {
                // Reset failed, but the device wasn't lost so something bad happened, 
                // so recreate the device to try to recover
                DXUTDeviceSettings* pDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();
                if( FAILED( DXUTChangeDevice( pDeviceSettings, NULL, NULL, true, false ) ) )
                {
                    DXUTShutdown();
                    return;
                }

                // TODO:  Handle display orientation changes in full-screen mode.
            }
        }
    }
    else if( DXGI_ERROR_DEVICE_REMOVED == hr )
    {
        // Use a callback to ask the app if it would like to find a new device.  
        // If no device removed callback is set, then look for a new device
        if( FAILED( DXUTHandleDeviceRemoved() ) )
        {
            // TODO: use pD3DDevice->GetDeviceRemovedReason()
            DXUTDisplayErrorMessage( DXUTERR_DEVICEREMOVED );
            DXUTShutdown();
            return;
        }
    }
    else if( SUCCEEDED( hr ) )
    {
        if( GetDXUTState().GetRenderingOccluded() )
        {
            // Now that we're no longer occluded
            // allow us to render again
            GetDXUTState().SetRenderingOccluded( false );
        }
    }

    // Update current frame #
    int nFrame = GetDXUTState().GetCurrentFrameNumber();
    nFrame++;
    GetDXUTState().SetCurrentFrameNumber( nFrame );


    // Update the D3D11 counter stats
    //DXUTUpdateD3D11CounterStats();

    // Check to see if the app should shutdown due to cmdline
    if( GetDXUTState().GetOverrideQuitAfterFrame() != 0 )
    {
        if( nFrame > GetDXUTState().GetOverrideQuitAfterFrame() )
            DXUTShutdown();
    }

    return;
}

void ClearD3D11DeviceContext( ID3D11DeviceContext* pd3dDeviceContext )
{
    // Unbind all objects from the immediate context
    if (pd3dDeviceContext == NULL) return;

    ID3D11ShaderResourceView* pSRVs[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    ID3D11RenderTargetView* pRTVs[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    ID3D11DepthStencilView* pDSV = NULL;
    ID3D11Buffer* pBuffers[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    ID3D11SamplerState* pSamplers[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    UINT StrideOffset[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

    // Shaders
    pd3dDeviceContext->VSSetShader( NULL, NULL, 0 );
    pd3dDeviceContext->HSSetShader( NULL, NULL, 0 );
    pd3dDeviceContext->DSSetShader( NULL, NULL, 0 );
    pd3dDeviceContext->GSSetShader( NULL, NULL, 0 );
    pd3dDeviceContext->PSSetShader( NULL, NULL, 0 );

    // IA clear
    pd3dDeviceContext->IASetVertexBuffers( 0, 16, pBuffers, StrideOffset, StrideOffset );
    pd3dDeviceContext->IASetIndexBuffer( NULL, DXGI_FORMAT_R16_UINT, 0 );
    pd3dDeviceContext->IASetInputLayout( NULL );

    // Constant buffers
    pd3dDeviceContext->VSSetConstantBuffers( 0, 14, pBuffers );
    pd3dDeviceContext->HSSetConstantBuffers( 0, 14, pBuffers );
    pd3dDeviceContext->DSSetConstantBuffers( 0, 14, pBuffers );
    pd3dDeviceContext->GSSetConstantBuffers( 0, 14, pBuffers );
    pd3dDeviceContext->PSSetConstantBuffers( 0, 14, pBuffers );

    // Resources
    pd3dDeviceContext->VSSetShaderResources( 0, 16, pSRVs );
    pd3dDeviceContext->HSSetShaderResources( 0, 16, pSRVs );
    pd3dDeviceContext->DSSetShaderResources( 0, 16, pSRVs );
    pd3dDeviceContext->GSSetShaderResources( 0, 16, pSRVs );
    pd3dDeviceContext->PSSetShaderResources( 0, 16, pSRVs );

    // Samplers
    pd3dDeviceContext->VSSetSamplers( 0, 16, pSamplers );
    pd3dDeviceContext->HSSetSamplers( 0, 16, pSamplers );
    pd3dDeviceContext->DSSetSamplers( 0, 16, pSamplers );
    pd3dDeviceContext->GSSetSamplers( 0, 16, pSamplers );
    pd3dDeviceContext->PSSetSamplers( 0, 16, pSamplers );

    // Render targets
    pd3dDeviceContext->OMSetRenderTargets( 8, pRTVs, pDSV );

    // States
    FLOAT blendFactor[4] = { 0,0,0,0 };
    pd3dDeviceContext->OMSetBlendState( NULL, blendFactor, 0xFFFFFFFF );
    pd3dDeviceContext->OMSetDepthStencilState( NULL, 0 );
    pd3dDeviceContext->RSSetState( NULL );
}

//--------------------------------------------------------------------------------------
// Cleans up the 3D environment by:
//      - Calls the device lost callback 
//      - Calls the device destroyed callback 
//      - Releases the D3D device
//--------------------------------------------------------------------------------------
void DXUTCleanup3DEnvironment11( bool bReleaseSettings )
{
    ID3D11Device* pd3dDevice = DXUTGetD3D11Device();

    if( pd3dDevice != NULL )
    {
        if (GetDXUTState().GetD3D11RasterizerState()!= NULL ) 
            GetDXUTState().GetD3D11RasterizerState()->Release();

        // Call ClearState to avoid tons of messy debug spew telling us that we're deleting bound objects
        ID3D11DeviceContext* pImmediateContext = DXUTGetD3D11DeviceContext();
        ClearD3D11DeviceContext( pImmediateContext );

        // Clear state and flush
        pImmediateContext->ClearState();
        pImmediateContext->Flush();

        // Call the app's SwapChain lost callback
        GetDXUTState().SetInsideDeviceCallback( true );
        if( GetDXUTState().GetDeviceObjectsReset() )
        {
            LPDXUTCALLBACKD3D11SWAPCHAINRELEASING pCallbackSwapChainReleasing =
                GetDXUTState().GetD3D11SwapChainReleasingFunc();
            if( pCallbackSwapChainReleasing != NULL )
                pCallbackSwapChainReleasing( GetDXUTState().GetD3D11SwapChainReleasingFuncUserContext() );
            GetDXUTState().SetDeviceObjectsReset( false );
        }

        // Release our old depth stencil texture and view 
        ID3D11Texture2D* pDS = GetDXUTState().GetD3D11DepthStencil();
        SAFE_RELEASE( pDS );
        GetDXUTState().SetD3D11DepthStencil( NULL );
        ID3D11DepthStencilView* pDSV = GetDXUTState().GetD3D11DepthStencilView();
        SAFE_RELEASE( pDSV );
        GetDXUTState().SetD3D11DepthStencilView( NULL );

        // Cleanup the render target view
        ID3D11RenderTargetView* pRTV = GetDXUTState().GetD3D11RenderTargetView();
        SAFE_RELEASE( pRTV );
        GetDXUTState().SetD3D11RenderTargetView( NULL );

        // Call the app's device destroyed callback
        if( GetDXUTState().GetDeviceObjectsCreated() )
        {
            LPDXUTCALLBACKD3D11DEVICEDESTROYED pCallbackDeviceDestroyed = GetDXUTState().GetD3D11DeviceDestroyedFunc();
            if( pCallbackDeviceDestroyed != NULL )
                pCallbackDeviceDestroyed( GetDXUTState().GetD3D11DeviceDestroyedFuncUserContext() );
            GetDXUTState().SetDeviceObjectsCreated( false );
        }

        GetDXUTState().SetInsideDeviceCallback( false );

        // Release the swap chain
        GetDXUTState().SetReleasingSwapChain( true );
        IDXGISwapChain* pSwapChain = DXUTGetDXGISwapChain();
        if( pSwapChain )
        {
            pSwapChain->SetFullscreenState( FALSE, 0 );
        }
        SAFE_RELEASE( pSwapChain );
        GetDXUTState().SetDXGISwapChain( NULL );
        GetDXUTState().SetReleasingSwapChain( false );

        // Release the outputs.
        IDXGIOutput** ppOutputArray = GetDXUTState().GetDXGIOutputArray();
        UINT OutputCount = GetDXUTState().GetDXGIOutputArraySize();
        for( UINT o = 0; o < OutputCount; ++o )
        SAFE_RELEASE( ppOutputArray[o] );
        delete[] ppOutputArray;
        GetDXUTState().SetDXGIOutputArray( NULL );
        GetDXUTState().SetDXGIOutputArraySize( 0 );

        // Release the D3D adapter.
        IDXGIAdapter* pAdapter = GetDXUTState().GetDXGIAdapter();
        SAFE_RELEASE( pAdapter );
        GetDXUTState().SetDXGIAdapter( NULL );

        // Release the counters
        //DXUTDestroyD3D11Counters();

        // Release the D3D11 immediate context (if it exists) because it has a extra ref count on it
        ID3D11DeviceContext* pd3d11DeviceContext = GetDXUTState().GetD3D11DeviceContext();
        SAFE_RELEASE( pd3d11DeviceContext );
        GetDXUTState().SetD3D11DeviceContext( NULL );

        // Release the D3D device and in debug configs, displays a message box if there 
        // are unrelease objects.
        if( pd3dDevice )
        {
            UINT references = pd3dDevice->Release();
            if( references > 0 )
            {
                DXUTDisplayErrorMessage( DXUTERR_NONZEROREFCOUNT );
                DXUT_ERR( L"DXUTCleanup3DEnvironment", DXUTERR_NONZEROREFCOUNT );
            }
        }
        GetDXUTState().SetD3D11Device( NULL );

        if( bReleaseSettings )
        {
            DXUTDeviceSettings* pOldDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();
            SAFE_DELETE(pOldDeviceSettings);
            GetDXUTState().SetCurrentDeviceSettings( NULL );
        }

        DXGI_SURFACE_DESC* pBackBufferSurfaceDesc = GetDXUTState().GetBackBufferSurfaceDescDXGI();
        ZeroMemory( pBackBufferSurfaceDesc, sizeof( DXGI_SURFACE_DESC ) );

        GetDXUTState().SetDeviceCreated( false );
    }
}


//--------------------------------------------------------------------------------------
// Low level keyboard hook to disable Windows key to prevent accidental task switching.  
//--------------------------------------------------------------------------------------
LRESULT CALLBACK DXUTLowLevelKeyboardProc( int nCode, WPARAM wParam, LPARAM lParam )
{
    if( nCode < 0 || nCode != HC_ACTION )  // do not process message 
        return CallNextHookEx( GetDXUTState().GetKeyboardHook(), nCode, wParam, lParam );

    bool bEatKeystroke = false;
    KBDLLHOOKSTRUCT* p = ( KBDLLHOOKSTRUCT* )lParam;
    switch( wParam )
    {
        case WM_KEYDOWN:
        case WM_KEYUP:
            {
                bEatKeystroke = ( !GetDXUTState().GetAllowShortcutKeys() &&
                                  ( p->vkCode == VK_LWIN || p->vkCode == VK_RWIN ) );
                break;
            }
    }

    if( bEatKeystroke )
        return 1;
    else
        return CallNextHookEx( GetDXUTState().GetKeyboardHook(), nCode, wParam, lParam );
}



//--------------------------------------------------------------------------------------
// Controls how DXUT behaves when fullscreen and windowed mode with regard to 
// shortcut keys (Windows keys, StickyKeys shortcut, ToggleKeys shortcut, FilterKeys shortcut) 
//--------------------------------------------------------------------------------------
void WINAPI DXUTSetShortcutKeySettings( bool bAllowWhenFullscreen, bool bAllowWhenWindowed )
{
    GetDXUTState().SetAllowShortcutKeysWhenWindowed( bAllowWhenWindowed );
    GetDXUTState().SetAllowShortcutKeysWhenFullscreen( bAllowWhenFullscreen );

    // DXUTInit() records initial accessibility states so don't change them until then
    if( GetDXUTState().GetDXUTInited() )
    {
        if( DXUTIsWindowed() )
            DXUTAllowShortcutKeys( GetDXUTState().GetAllowShortcutKeysWhenWindowed() );
        else
            DXUTAllowShortcutKeys( GetDXUTState().GetAllowShortcutKeysWhenFullscreen() );
    }
}


//--------------------------------------------------------------------------------------
// Enables/disables Windows keys, and disables or restores the StickyKeys/ToggleKeys/FilterKeys 
// shortcut to help prevent accidental task switching
//--------------------------------------------------------------------------------------
void DXUTAllowShortcutKeys( bool bAllowKeys )
{
    GetDXUTState().SetAllowShortcutKeys( bAllowKeys );

    if( bAllowKeys )
    {
        // Restore StickyKeys/etc to original state and enable Windows key      
        STICKYKEYS sk = GetDXUTState().GetStartupStickyKeys();
        TOGGLEKEYS tk = GetDXUTState().GetStartupToggleKeys();
        FILTERKEYS fk = GetDXUTState().GetStartupFilterKeys();

        SystemParametersInfo( SPI_SETSTICKYKEYS, sizeof( STICKYKEYS ), &sk, 0 );
        SystemParametersInfo( SPI_SETTOGGLEKEYS, sizeof( TOGGLEKEYS ), &tk, 0 );
        SystemParametersInfo( SPI_SETFILTERKEYS, sizeof( FILTERKEYS ), &fk, 0 );

        // Remove the keyboard hoook when it isn't needed to prevent any slow down of other apps
        if( GetDXUTState().GetKeyboardHook() )
        {
            UnhookWindowsHookEx( GetDXUTState().GetKeyboardHook() );
            GetDXUTState().SetKeyboardHook( NULL );
        }
    }
    else
    {
        // Set low level keyboard hook if haven't already
        if( GetDXUTState().GetKeyboardHook() == NULL )
        {
            // Set the low-level hook procedure.  Only works on Windows 2000 and above
            OSVERSIONINFO OSVersionInfo;
            OSVersionInfo.dwOSVersionInfoSize = sizeof( OSVersionInfo );
            GetVersionEx( &OSVersionInfo );
            if( OSVersionInfo.dwPlatformId == VER_PLATFORM_WIN32_NT && OSVersionInfo.dwMajorVersion > 4 )
            {
                HHOOK hKeyboardHook = SetWindowsHookEx( WH_KEYBOARD_LL, DXUTLowLevelKeyboardProc,
                                                        GetModuleHandle( NULL ), 0 );
                GetDXUTState().SetKeyboardHook( hKeyboardHook );
            }
        }

        // Disable StickyKeys/etc shortcuts but if the accessibility feature is on, 
        // then leave the settings alone as its probably being usefully used

        STICKYKEYS skOff = GetDXUTState().GetStartupStickyKeys();
        if( ( skOff.dwFlags & SKF_STICKYKEYSON ) == 0 )
        {
            // Disable the hotkey and the confirmation
            skOff.dwFlags &= ~SKF_HOTKEYACTIVE;
            skOff.dwFlags &= ~SKF_CONFIRMHOTKEY;

            SystemParametersInfo( SPI_SETSTICKYKEYS, sizeof( STICKYKEYS ), &skOff, 0 );
        }

        TOGGLEKEYS tkOff = GetDXUTState().GetStartupToggleKeys();
        if( ( tkOff.dwFlags & TKF_TOGGLEKEYSON ) == 0 )
        {
            // Disable the hotkey and the confirmation
            tkOff.dwFlags &= ~TKF_HOTKEYACTIVE;
            tkOff.dwFlags &= ~TKF_CONFIRMHOTKEY;

            SystemParametersInfo( SPI_SETTOGGLEKEYS, sizeof( TOGGLEKEYS ), &tkOff, 0 );
        }

        FILTERKEYS fkOff = GetDXUTState().GetStartupFilterKeys();
        if( ( fkOff.dwFlags & FKF_FILTERKEYSON ) == 0 )
        {
            // Disable the hotkey and the confirmation
            fkOff.dwFlags &= ~FKF_HOTKEYACTIVE;
            fkOff.dwFlags &= ~FKF_CONFIRMHOTKEY;

            SystemParametersInfo( SPI_SETFILTERKEYS, sizeof( FILTERKEYS ), &fkOff, 0 );
        }
    }
}


//--------------------------------------------------------------------------------------
// Pauses time or rendering.  Keeps a ref count so pausing can be layered
//--------------------------------------------------------------------------------------
void WINAPI DXUTPause( bool bPauseTime, bool bPauseRendering )
{
    int nPauseTimeCount = GetDXUTState().GetPauseTimeCount();
    if( bPauseTime ) nPauseTimeCount++;
    else
        nPauseTimeCount--;
    if( nPauseTimeCount < 0 ) nPauseTimeCount = 0;
    GetDXUTState().SetPauseTimeCount( nPauseTimeCount );

    int nPauseRenderingCount = GetDXUTState().GetPauseRenderingCount();
    if( bPauseRendering ) nPauseRenderingCount++;
    else
        nPauseRenderingCount--;
    if( nPauseRenderingCount < 0 ) nPauseRenderingCount = 0;
    GetDXUTState().SetPauseRenderingCount( nPauseRenderingCount );

    if( nPauseTimeCount > 0 )
    {
        // Stop the scene from animating
        DXUTGetGlobalTimer()->Stop();
    }
    else
    {
        // Restart the timer
        DXUTGetGlobalTimer()->Start();
    }

    GetDXUTState().SetRenderingPaused( nPauseRenderingCount > 0 );
    GetDXUTState().SetTimePaused( nPauseTimeCount > 0 );
}


//--------------------------------------------------------------------------------------
// Starts a user defined timer callback
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTSetTimer( LPDXUTCALLBACKTIMER pCallbackTimer, float fTimeoutInSecs, UINT* pnIDEvent,
                             void* pCallbackUserContext )
{
    if( pCallbackTimer == NULL )
        return DXUT_ERR_MSGBOX( L"DXUTSetTimer", E_INVALIDARG );

    HRESULT hr;
    DXUT_TIMER DXUTTimer;
    DXUTTimer.pCallbackTimer = pCallbackTimer;
    DXUTTimer.pCallbackUserContext = pCallbackUserContext;
    DXUTTimer.fTimeoutInSecs = fTimeoutInSecs;
    DXUTTimer.fCountdown = fTimeoutInSecs;
    DXUTTimer.bEnabled = true;
    DXUTTimer.nID = GetDXUTState().GetTimerLastID() + 1;
    GetDXUTState().SetTimerLastID( DXUTTimer.nID );

    CGrowableArray <DXUT_TIMER>* pTimerList = GetDXUTState().GetTimerList();
    if( pTimerList == NULL )
    {
        pTimerList = new CGrowableArray <DXUT_TIMER>;
        if( pTimerList == NULL )
            return E_OUTOFMEMORY;
        GetDXUTState().SetTimerList( pTimerList );
    }

    if( FAILED( hr = pTimerList->Add( DXUTTimer ) ) )
        return hr;

    if( pnIDEvent )
        *pnIDEvent = DXUTTimer.nID;

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Stops a user defined timer callback
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTKillTimer( UINT nIDEvent )
{
    CGrowableArray <DXUT_TIMER>* pTimerList = GetDXUTState().GetTimerList();
    if( pTimerList == NULL )
        return S_FALSE;

    bool bFound = false;

    for( int i = 0; i < pTimerList->GetSize(); i++ )
    {
        DXUT_TIMER DXUTTimer = pTimerList->GetAt( i );
        if( DXUTTimer.nID == nIDEvent )
        {
            DXUTTimer.bEnabled = false;
            pTimerList->SetAt( i, DXUTTimer );
            bFound = true;
            break;
        }
    }

    if( !bFound )
        return DXUT_ERR_MSGBOX( L"DXUTKillTimer", E_INVALIDARG );

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Internal helper function to handle calling the user defined timer callbacks
//--------------------------------------------------------------------------------------
void DXUTHandleTimers()
{
    float fElapsedTime = DXUTGetElapsedTime();

    CGrowableArray <DXUT_TIMER>* pTimerList = GetDXUTState().GetTimerList();
    if( pTimerList == NULL )
        return;

    // Walk through the list of timer callbacks
    for( int i = 0; i < pTimerList->GetSize(); i++ )
    {
        DXUT_TIMER DXUTTimer = pTimerList->GetAt( i );
        if( DXUTTimer.bEnabled )
        {
            DXUTTimer.fCountdown -= fElapsedTime;

            // Call the callback if count down expired
            if( DXUTTimer.fCountdown < 0 )
            {
                DXUTTimer.pCallbackTimer( DXUTTimer.nID, DXUTTimer.pCallbackUserContext );
                // The callback my have changed the timer.
                DXUTTimer = pTimerList->GetAt( i );
                DXUTTimer.fCountdown = DXUTTimer.fTimeoutInSecs;
            }
            pTimerList->SetAt( i, DXUTTimer );
        }
    }
}


//--------------------------------------------------------------------------------------
// Display an custom error msg box 
//--------------------------------------------------------------------------------------
void DXUTDisplayErrorMessage( HRESULT hr )
{
    WCHAR strBuffer[512];

    int nExitCode;
    bool bFound = true;
    switch( hr )
    {
        case DXUTERR_NODIRECT3D:
        {
            nExitCode = 2;
            if( DXUTDoesAppSupportD3D11() && !DXUTDoesAppSupportD3D9() )
                wcscpy_s( strBuffer, ARRAYSIZE(strBuffer), L"Could not initialize Direct3D 11. " );
            else
                wcscpy_s( strBuffer, ARRAYSIZE(strBuffer), L"Could not initialize Direct3D 9. Check that the latest version of DirectX is correctly installed on your system.  Also make sure that this program was compiled with header files that match the installed DirectX DLLs." );
            break;
        }
        case DXUTERR_NOCOMPATIBLEDEVICES:    
            nExitCode = 3; 
            if( GetSystemMetrics(0x1000) != 0 ) // SM_REMOTESESSION
                wcscpy_s( strBuffer, ARRAYSIZE(strBuffer), L"Direct3D does not work over a remote session." ); 
            else
                wcscpy_s( strBuffer, ARRAYSIZE(strBuffer), L"Could not find any compatible Direct3D devices." ); 
            break;
        case DXUTERR_MEDIANOTFOUND:          nExitCode = 4; wcscpy_s( strBuffer, ARRAYSIZE(strBuffer), L"Could not find required media." ); break;
        case DXUTERR_NONZEROREFCOUNT:        nExitCode = 5; wcscpy_s( strBuffer, ARRAYSIZE(strBuffer), L"The Direct3D device has a non-zero reference count, meaning some objects were not released." ); break;
        case DXUTERR_CREATINGDEVICE:         nExitCode = 6; wcscpy_s( strBuffer, ARRAYSIZE(strBuffer), L"Failed creating the Direct3D device." ); break;
        case DXUTERR_RESETTINGDEVICE:        nExitCode = 7; wcscpy_s( strBuffer, ARRAYSIZE(strBuffer), L"Failed resetting the Direct3D device." ); break;
        case DXUTERR_CREATINGDEVICEOBJECTS:  nExitCode = 8; wcscpy_s( strBuffer, ARRAYSIZE(strBuffer), L"An error occurred in the device create callback function." ); break;
        case DXUTERR_RESETTINGDEVICEOBJECTS: nExitCode = 9; wcscpy_s( strBuffer, ARRAYSIZE(strBuffer), L"An error occurred in the device reset callback function." ); break;
        // nExitCode 10 means the app exited using a REF device 
        case DXUTERR_DEVICEREMOVED:          nExitCode = 11; wcscpy_s( strBuffer, ARRAYSIZE(strBuffer), L"The Direct3D device was removed."  ); break;
        default: bFound = false; nExitCode = 1; break; // nExitCode 1 means the API was incorrectly called

    }   

    GetDXUTState().SetExitCode(nExitCode);

    bool bShowMsgBoxOnError = GetDXUTState().GetShowMsgBoxOnError();
    if( bFound && bShowMsgBoxOnError )
    {
        if( DXUTGetWindowTitle()[0] == 0 )
            MessageBox( DXUTGetHWND(), strBuffer, L"DXUT Application", MB_ICONERROR | MB_OK );
        else
            MessageBox( DXUTGetHWND(), strBuffer, DXUTGetWindowTitle(), MB_ICONERROR | MB_OK );
    }
}


//--------------------------------------------------------------------------------------
// Internal function to map MK_* to an array index
//--------------------------------------------------------------------------------------
int DXUTMapButtonToArrayIndex( BYTE vButton )
{
    switch( vButton )
    {
        case MK_LBUTTON:
            return 0;
        case VK_MBUTTON:
        case MK_MBUTTON:
            return 1;
        case MK_RBUTTON:
            return 2;
        case VK_XBUTTON1:
        case MK_XBUTTON1:
            return 3;
        case VK_XBUTTON2:
        case MK_XBUTTON2:
            return 4;
    }

    return 0;
}



//--------------------------------------------------------------------------------------
// Toggle between full screen and windowed
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTToggleFullScreen()
{
    HRESULT hr;
    DXUTDeviceSettings deviceSettings = DXUTGetDeviceSettings();
    DXUTDeviceSettings orginalDeviceSettings = DXUTGetDeviceSettings();
    
    if (deviceSettings.ver == DXUT_D3D11_DEVICE) {
        deviceSettings.d3d11.sd.Windowed = !deviceSettings.d3d11.sd.Windowed; // datut
        if (!deviceSettings.d3d11.sd.Windowed) {
            DXGI_MODE_DESC adapterDesktopDisplayMode =
            {
                800, 600, { 60, 1 }, DXGI_FORMAT_R8G8B8A8_UNORM_SRGB
            };
            DXUTGetD3D11AdapterDisplayMode( deviceSettings.d3d11.AdapterOrdinal, 0, &adapterDesktopDisplayMode );
            
            
            deviceSettings.d3d11.sd.BufferDesc = adapterDesktopDisplayMode;
        }else {
            RECT r = DXUTGetWindowClientRectAtModeChange();
            deviceSettings.d3d11.sd.BufferDesc.Height = r.bottom;
            deviceSettings.d3d11.sd.BufferDesc.Width = r.right;
        }
    }else if (deviceSettings.ver == DXUT_D3D9_DEVICE){
        deviceSettings.d3d9.pp.Windowed = !deviceSettings.d3d9.pp.Windowed; 
        if (!deviceSettings.d3d9.pp.Windowed) {
                D3DDISPLAYMODE adapterDesktopDisplayMode;
                IDirect3D9* pD3D = DXUTGetD3D9Object();
                DXUTDeviceSettings* pDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();
                pD3D->GetAdapterDisplayMode( pDeviceSettings->d3d9.AdapterOrdinal, &adapterDesktopDisplayMode );
                deviceSettings.d3d9.pp.BackBufferWidth = adapterDesktopDisplayMode.Width;
                deviceSettings.d3d9.pp.BackBufferHeight = adapterDesktopDisplayMode.Height;
                deviceSettings.d3d9.pp.BackBufferFormat = adapterDesktopDisplayMode.Format;
        } else {
            RECT r = DXUTGetWindowClientRectAtModeChange();
            deviceSettings.d3d9.pp.BackBufferHeight= r.bottom;
            deviceSettings.d3d9.pp.FullScreen_RefreshRateInHz = 0;
            deviceSettings.d3d9.pp.BackBufferWidth = r.right;
        }
    }

    hr = DXUTChangeDevice( &deviceSettings, NULL, NULL,  false, false );

    // If hr == E_ABORT, this means the app rejected the device settings in the ModifySettingsCallback so nothing changed
    if( FAILED( hr ) && ( hr != E_ABORT ) )
    {
        // Failed creating device, try to switch back.
        HRESULT hr2 = DXUTChangeDevice( &orginalDeviceSettings, NULL,  NULL, false, false );
        if( FAILED( hr2 ) )
        {
            // If this failed, then shutdown
            DXUTShutdown();
        }
    }

    return hr;
}


//--------------------------------------------------------------------------------------
// Toggle between HAL and WARP
//--------------------------------------------------------------------------------------

HRESULT WINAPI DXUTToggleWARP () {
    HRESULT hr;

    DXUTDeviceSettings deviceSettings = DXUTGetDeviceSettings();
    DXUTDeviceSettings orginalDeviceSettings = DXUTGetDeviceSettings();

    // Toggle between REF & HAL
    if( DXUTIsCurrentDeviceD3D9() )
    {

    }
    else
    {
        ID3D11SwitchToRef* pD3D11STR = NULL;
        hr = DXUTGetD3D11Device()->QueryInterface( __uuidof( *pD3D11STR ), ( LPVOID* )&pD3D11STR );
        if( SUCCEEDED( hr ) )
        {
            pD3D11STR->SetUseRef( pD3D11STR->GetUseRef() ? FALSE : TRUE );
            SAFE_RELEASE( pD3D11STR );
            return S_OK;
        }

        if( deviceSettings.d3d11.DriverType == D3D_DRIVER_TYPE_HARDWARE || deviceSettings.d3d11.DriverType == D3D_DRIVER_TYPE_REFERENCE  )
            deviceSettings.d3d11.DriverType = D3D_DRIVER_TYPE_WARP;
        else if( deviceSettings.d3d11.DriverType == D3D_DRIVER_TYPE_REFERENCE || deviceSettings.d3d11.DriverType == D3D_DRIVER_TYPE_WARP )
            deviceSettings.d3d11.DriverType = D3D_DRIVER_TYPE_HARDWARE;
    }
    
    hr = DXUTSnapDeviceSettingsToEnumDevice(&deviceSettings, false);
    if( SUCCEEDED( hr ) )
    {
        // Create a Direct3D device using the new device settings.  
        // If there is an existing device, then it will either reset or recreate the scene.
        hr = DXUTChangeDevice( &deviceSettings, NULL,  NULL, false, false );

        // If hr == E_ABORT, this means the app rejected the device settings in the ModifySettingsCallback so nothing changed
        if( FAILED( hr ) && ( hr != E_ABORT ) )
        {
            // Failed creating device, try to switch back.
            HRESULT hr2 = DXUTChangeDevice( &orginalDeviceSettings, NULL, NULL, false, false );
            if( FAILED( hr2 ) )
            {
                // If this failed, then shutdown
                DXUTShutdown();
            }
        }
    }

    return hr;
}
//--------------------------------------------------------------------------------------
// Toggle between HAL and REF
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTToggleREF()
{
    HRESULT hr;

    DXUTDeviceSettings deviceSettings = DXUTGetDeviceSettings();
    DXUTDeviceSettings orginalDeviceSettings = DXUTGetDeviceSettings();

    // Toggle between REF & HAL
    if( DXUTIsCurrentDeviceD3D9() )
    {
        if( deviceSettings.d3d9.DeviceType == D3DDEVTYPE_HAL )
            deviceSettings.d3d9.DeviceType = D3DDEVTYPE_REF;
        else if( deviceSettings.d3d9.DeviceType == D3DDEVTYPE_REF )
            deviceSettings.d3d9.DeviceType = D3DDEVTYPE_HAL;
    }
    else
    {
        ID3D11SwitchToRef* pD3D11STR = NULL;
        hr = DXUTGetD3D11Device()->QueryInterface( __uuidof( *pD3D11STR ), ( LPVOID* )&pD3D11STR );
        if( SUCCEEDED( hr ) )
        {
            pD3D11STR->SetUseRef( pD3D11STR->GetUseRef() ? FALSE : TRUE );
            SAFE_RELEASE( pD3D11STR );
            return S_OK;
        }

        if( deviceSettings.d3d11.DriverType == D3D_DRIVER_TYPE_HARDWARE )
            deviceSettings.d3d11.DriverType = D3D_DRIVER_TYPE_REFERENCE;
        else if( deviceSettings.d3d11.DriverType == D3D_DRIVER_TYPE_REFERENCE )
            deviceSettings.d3d11.DriverType = D3D_DRIVER_TYPE_HARDWARE;
    }
    
    hr = DXUTSnapDeviceSettingsToEnumDevice(&deviceSettings, false);
    if( SUCCEEDED( hr ) )
    {
        // Create a Direct3D device using the new device settings.  
        // If there is an existing device, then it will either reset or recreate the scene.
        hr = DXUTChangeDevice( &deviceSettings, NULL,  NULL, false, false );

        // If hr == E_ABORT, this means the app rejected the device settings in the ModifySettingsCallback so nothing changed
        if( FAILED( hr ) && ( hr != E_ABORT ) )
        {
            // Failed creating device, try to switch back.
            HRESULT hr2 = DXUTChangeDevice( &orginalDeviceSettings, NULL, NULL, false, false );
            if( FAILED( hr2 ) )
            {
                // If this failed, then shutdown
                DXUTShutdown();
            }
        }
    }

    return hr;
}

//--------------------------------------------------------------------------------------
// Checks to see if DXGI has switched us out of fullscreen or windowed mode
//--------------------------------------------------------------------------------------
void DXUTCheckForDXGIFullScreenSwitch()
{
    DXUTDeviceSettings* pDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();
    if( !DXUTIsD3D9( pDeviceSettings ) )
    {
        IDXGISwapChain* pSwapChain = DXUTGetDXGISwapChain();
        DXGI_SWAP_CHAIN_DESC SCDesc;
        pSwapChain->GetDesc( &SCDesc );

        BOOL bIsWindowed = ( BOOL )DXUTIsWindowed();
        if( bIsWindowed != SCDesc.Windowed )
        {
            pDeviceSettings->d3d11.sd.Windowed = SCDesc.Windowed;

            DXUTDeviceSettings deviceSettings = DXUTGetDeviceSettings();

            if( bIsWindowed )
            {
                GetDXUTState().SetWindowBackBufferWidthAtModeChange( deviceSettings.d3d11.sd.BufferDesc.Width );
                GetDXUTState().SetWindowBackBufferHeightAtModeChange( deviceSettings.d3d11.sd.BufferDesc.Height );
            }
            else
            {
                GetDXUTState().SetFullScreenBackBufferWidthAtModeChange( deviceSettings.d3d11.sd.BufferDesc.Width );
                GetDXUTState().SetFullScreenBackBufferHeightAtModeChange( deviceSettings.d3d11.sd.BufferDesc.Height );
            }
        }
    }
}

void DXUTResizeDXGIBuffers( UINT Width, UINT Height, BOOL bFullScreen )
{
    HRESULT hr = S_OK;
    RECT rcCurrentClient;
    GetClientRect( DXUTGetHWND(), &rcCurrentClient );

    DXUTDeviceSettings* pDevSettings = GetDXUTState().GetCurrentDeviceSettings();
    IDXGISwapChain* pSwapChain = DXUTGetDXGISwapChain();

    ID3D11Device* pd3dDevice = DXUTGetD3D11Device();
    ID3D11DeviceContext* pd3dImmediateContext = DXUTGetD3D11DeviceContext();

    // Determine if we're fullscreen
    pDevSettings->d3d11.sd.Windowed = !bFullScreen;

    // Call releasing
    GetDXUTState().SetInsideDeviceCallback( true );
    LPDXUTCALLBACKD3D11SWAPCHAINRELEASING pCallbackSwapChainReleasing = GetDXUTState().GetD3D11SwapChainReleasingFunc
        ();
    if( pCallbackSwapChainReleasing != NULL )
        pCallbackSwapChainReleasing( GetDXUTState().GetD3D11SwapChainResizedFuncUserContext() );
    GetDXUTState().SetInsideDeviceCallback( false );

    // Release our old depth stencil texture and view 
    ID3D11Texture2D* pDS = GetDXUTState().GetD3D11DepthStencil();
    SAFE_RELEASE( pDS );
    GetDXUTState().SetD3D11DepthStencil( NULL );
    ID3D11DepthStencilView* pDSV = GetDXUTState().GetD3D11DepthStencilView();
    SAFE_RELEASE( pDSV );
    GetDXUTState().SetD3D11DepthStencilView( NULL );

    // Release our old render target view
    ID3D11RenderTargetView* pRTV = GetDXUTState().GetD3D11RenderTargetView();
    SAFE_RELEASE( pRTV );
    GetDXUTState().SetD3D11RenderTargetView( NULL );

    // Alternate between 0 and DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH when resizing buffers.
    // When in windowed mode, we want 0 since this allows the app to change to the desktop
    // resolution from windowed mode during alt+enter.  However, in fullscreen mode, we want
    // the ability to change display modes from the Device Settings dialog.  Therefore, we
    // want to set the DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH flag.
    UINT Flags = 0;
    if( bFullScreen )
        Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;

    // ResizeBuffers
    V( pSwapChain->ResizeBuffers( pDevSettings->d3d11.sd.BufferCount,
                                  Width,
                                  Height,
                                  pDevSettings->d3d11.sd.BufferDesc.Format,
                                  Flags ) );

    if( !GetDXUTState().GetDoNotStoreBufferSize() )
    {
        pDevSettings->d3d11.sd.BufferDesc.Width = ( UINT )rcCurrentClient.right;
        pDevSettings->d3d11.sd.BufferDesc.Height = ( UINT )rcCurrentClient.bottom;
    }

    // Save off backbuffer desc
    DXUTUpdateBackBufferDesc();

    // Update the device stats text
    DXUTUpdateStaticFrameStats();

    // Setup the render target view and viewport
    hr = DXUTCreateD3D11Views( pd3dDevice, pd3dImmediateContext, pDevSettings );
    if( FAILED( hr ) )
    {
        DXUT_ERR( L"DXUTCreateD3D11Views", hr );
        return;
    }

    // Setup cursor based on current settings (window/fullscreen mode, show cursor state, clip cursor state)
    DXUTSetupCursor();

    // Call the app's SwapChain reset callback
    GetDXUTState().SetInsideDeviceCallback( true );
    const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc = DXUTGetDXGIBackBufferSurfaceDesc();
    LPDXUTCALLBACKD3D11SWAPCHAINRESIZED pCallbackSwapChainResized = GetDXUTState().GetD3D11SwapChainResizedFunc();
    hr = S_OK;
    if( pCallbackSwapChainResized != NULL )
        hr = pCallbackSwapChainResized( pd3dDevice, pSwapChain, pBackBufferSurfaceDesc,
                                        GetDXUTState().GetD3D11SwapChainResizedFuncUserContext() );
    GetDXUTState().SetInsideDeviceCallback( false );
    if( FAILED( hr ) )
    {
        // If callback failed, cleanup
        DXUT_ERR( L"DeviceResetCallback", hr );
        if( hr != DXUTERR_MEDIANOTFOUND )
            hr = DXUTERR_RESETTINGDEVICEOBJECTS;

        GetDXUTState().SetInsideDeviceCallback( true );
        LPDXUTCALLBACKD3D11SWAPCHAINRELEASING pCallbackSwapChainReleasing =
            GetDXUTState().GetD3D11SwapChainReleasingFunc();
        if( pCallbackSwapChainReleasing != NULL )
            pCallbackSwapChainReleasing( GetDXUTState().GetD3D11SwapChainResizedFuncUserContext() );
        GetDXUTState().SetInsideDeviceCallback( false );
        DXUTPause( false, false );
        PostQuitMessage( 0 );
    }
    else
    {
        GetDXUTState().SetDeviceObjectsReset( true );
        DXUTPause( false, false );
    }
}

//--------------------------------------------------------------------------------------
// Checks if DXGI buffers need to change
//--------------------------------------------------------------------------------------
void DXUTCheckForDXGIBufferChange()
{
    if(DXUTGetDXGISwapChain() != NULL && !GetDXUTState().GetReleasingSwapChain() )
    {
        //DXUTgetdxgi
        IDXGISwapChain* pSwapChain = DXUTGetDXGISwapChain();

        // Determine if we're fullscreen
        BOOL bFullScreen;
        pSwapChain->GetFullscreenState( &bFullScreen, NULL );

        DXUTResizeDXGIBuffers( 0, 0, bFullScreen );

        ShowWindow( DXUTGetHWND(), SW_SHOW );
    }
}

//--------------------------------------------------------------------------------------
// Checks if the window client rect has changed and if it has, then reset the device
//--------------------------------------------------------------------------------------
void DXUTCheckForWindowSizeChange()
{
    // Skip the check for various reasons
          
    if( GetDXUTState().GetIgnoreSizeChange() || !GetDXUTState().GetDeviceCreated() ||
        ( DXUTIsCurrentDeviceD3D9() && !DXUTIsWindowed() )  )
        return;

    DXUTDeviceSettings deviceSettings = DXUTGetDeviceSettings();
    if( DXUTIsD3D9( &deviceSettings ) )
    {
        RECT rcCurrentClient;
        GetClientRect( DXUTGetHWND(), &rcCurrentClient );

        if( ( UINT )rcCurrentClient.right != DXUTGetBackBufferWidthFromDS( &deviceSettings ) ||
            ( UINT )rcCurrentClient.bottom != DXUTGetBackBufferHeightFromDS( &deviceSettings ) )
        {
            // A new window size will require a new backbuffer size size
            // Tell DXUTChangeDevice and D3D to size according to the HWND's client rect
            if( DXUTIsD3D9( &deviceSettings ) ) deviceSettings.d3d9.pp.BackBufferWidth = 0; else deviceSettings.d3d11.sd.BufferDesc.Width = 0; 
            if( DXUTIsD3D9( &deviceSettings ) ) deviceSettings.d3d9.pp.BackBufferHeight = 0; else deviceSettings.d3d11.sd.BufferDesc.Height = 0;

            DXUTChangeDevice( &deviceSettings, NULL, NULL, false, false );
        }
    }
    else
    {
        DXUTCheckForDXGIBufferChange();
    }
}


//--------------------------------------------------------------------------------------
// Checks to see if the HWND changed monitors, and if it did it creates a device 
// from the monitor's adapter and recreates the scene.
//--------------------------------------------------------------------------------------
void DXUTCheckForWindowChangingMonitors()
{
    // Skip this check for various reasons
    if( !GetDXUTState().GetAutoChangeAdapter() ||
        GetDXUTState().GetIgnoreSizeChange() || !GetDXUTState().GetDeviceCreated() || !DXUTIsWindowed() )
        return;

    HRESULT hr;
    HMONITOR hWindowMonitor = DXUTMonitorFromWindow( DXUTGetHWND(), MONITOR_DEFAULTTOPRIMARY );
    HMONITOR hAdapterMonitor = GetDXUTState().GetAdapterMonitor();
    if( hWindowMonitor != hAdapterMonitor )
    {
        UINT newOrdinal;
        if( SUCCEEDED( DXUTGetAdapterOrdinalFromMonitor( hWindowMonitor, &newOrdinal ) ) )
        {
            // Find the closest valid device settings with the new ordinal
            DXUTDeviceSettings deviceSettings = DXUTGetDeviceSettings();
            if( DXUTIsD3D9( &deviceSettings ) )
            {
                deviceSettings.d3d9.AdapterOrdinal = newOrdinal;
            }
            else
            {
                deviceSettings.d3d11.AdapterOrdinal = newOrdinal;
                UINT newOutput;
                if( SUCCEEDED( DXUTGetOutputOrdinalFromMonitor( hWindowMonitor, &newOutput ) ) )
                    deviceSettings.d3d11.Output = newOutput;
            }

            hr = DXUTSnapDeviceSettingsToEnumDevice( &deviceSettings, false );
            if( SUCCEEDED( hr ) )
            {
                // Create a Direct3D device using the new device settings.  
                // If there is an existing device, then it will either reset or recreate the scene.
                hr = DXUTChangeDevice( &deviceSettings, NULL, NULL, false, false );

                // If hr == E_ABORT, this means the app rejected the device settings in the ModifySettingsCallback
                if( hr == E_ABORT )
                {
                    // so nothing changed and keep from attempting to switch adapters next time
                    GetDXUTState().SetAutoChangeAdapter( false );
                }
                else if( FAILED( hr ) )
                {
                    DXUTShutdown();
                    DXUTPause( false, false );
                    return;
                }
            }
        }
    }
}


//--------------------------------------------------------------------------------------
// Renders the scene using either D3D9 or D3D11
//--------------------------------------------------------------------------------------
void WINAPI DXUTRender3DEnvironment()
{
    if( DXUTIsCurrentDeviceD3D9() )
        DXUTRender3DEnvironment9();
    else
        DXUTRender3DEnvironment11();
}


//--------------------------------------------------------------------------------------
// Cleans up both the D3D9 and D3D11 3D environment (but only one should be active at a time)
//--------------------------------------------------------------------------------------
void DXUTCleanup3DEnvironment( bool bReleaseSettings )
{
    if( DXUTGetD3D9Device() )
        DXUTCleanup3DEnvironment9( bReleaseSettings );
    if( DXUTGetD3D11Device() )
        DXUTCleanup3DEnvironment11( bReleaseSettings );
}


//--------------------------------------------------------------------------------------
// Returns the HMONITOR attached to an adapter/output
//--------------------------------------------------------------------------------------
HMONITOR DXUTGetMonitorFromAdapter( DXUTDeviceSettings* pDeviceSettings )
{
    if( pDeviceSettings->ver == DXUT_D3D9_DEVICE )
    {
        IDirect3D9* pD3D = DXUTGetD3D9Object();
        return pD3D->GetAdapterMonitor( pDeviceSettings->d3d9.AdapterOrdinal );
    }
    else if( pDeviceSettings->ver == DXUT_D3D11_DEVICE )
    {
        CD3D11Enumeration* pD3DEnum = DXUTGetD3D11Enumeration();
        CD3D11EnumOutputInfo* pOutputInfo = pD3DEnum->GetOutputInfo( pDeviceSettings->d3d11.AdapterOrdinal,
                                                                     pDeviceSettings->d3d11.Output );
        if( !pOutputInfo )
            return 0;
        return DXUTMonitorFromRect( &pOutputInfo->Desc.DesktopCoordinates, MONITOR_DEFAULTTONEAREST );
    }

    return 0;
}


//--------------------------------------------------------------------------------------
// Look for an adapter ordinal that is tied to a HMONITOR
//--------------------------------------------------------------------------------------
HRESULT DXUTGetAdapterOrdinalFromMonitor( HMONITOR hMonitor, UINT* pAdapterOrdinal )
{
    *pAdapterOrdinal = 0;

    if( DXUTIsCurrentDeviceD3D9() )
    {
        CD3D9Enumeration* pd3dEnum = DXUTGetD3D9Enumeration();
        IDirect3D9* pD3D = DXUTGetD3D9Object();

        CGrowableArray <CD3D9EnumAdapterInfo*>* pAdapterList = pd3dEnum->GetAdapterInfoList();
        for( int iAdapter = 0; iAdapter < pAdapterList->GetSize(); iAdapter++ )
        {
            CD3D9EnumAdapterInfo* pAdapterInfo = pAdapterList->GetAt( iAdapter );
            HMONITOR hAdapterMonitor = pD3D->GetAdapterMonitor( pAdapterInfo->AdapterOrdinal );
            if( hAdapterMonitor == hMonitor )
            {
                *pAdapterOrdinal = pAdapterInfo->AdapterOrdinal;
                return S_OK;
            }
        }
    }
    else
    {
        // Get the monitor handle information
        MONITORINFOEX mi;
        mi.cbSize = sizeof( MONITORINFOEX );
        DXUTGetMonitorInfo( hMonitor, &mi );

        // Search for this monitor in our enumeration hierarchy.
        CD3D11Enumeration* pd3dEnum = DXUTGetD3D11Enumeration();
        CGrowableArray <CD3D11EnumAdapterInfo*>* pAdapterList = pd3dEnum->GetAdapterInfoList();
        for( int iAdapter = 0; iAdapter < pAdapterList->GetSize(); ++iAdapter )
        {
            CD3D11EnumAdapterInfo* pAdapterInfo = pAdapterList->GetAt( iAdapter );
            for( int o = 0; o < pAdapterInfo->outputInfoList.GetSize(); ++o )
            {
                CD3D11EnumOutputInfo* pOutputInfo = pAdapterInfo->outputInfoList.GetAt( o );
                // Convert output device name from MBCS to Unicode
                if( wcsncmp( pOutputInfo->Desc.DeviceName, mi.szDevice, sizeof( mi.szDevice ) / sizeof
                             ( mi.szDevice[0] ) ) == 0 )
                {
                    *pAdapterOrdinal = pAdapterInfo->AdapterOrdinal;
                    return S_OK;
                }
            }
        }
    }

    return E_FAIL;
}

//--------------------------------------------------------------------------------------
// Look for a monitor ordinal that is tied to a HMONITOR (D3D11-only)
//--------------------------------------------------------------------------------------
HRESULT DXUTGetOutputOrdinalFromMonitor( HMONITOR hMonitor, UINT* pOutputOrdinal )
{
    // Get the monitor handle information
    MONITORINFOEX mi;
    mi.cbSize = sizeof( MONITORINFOEX );
    DXUTGetMonitorInfo( hMonitor, &mi );

    // Search for this monitor in our enumeration hierarchy.
    CD3D11Enumeration* pd3dEnum = DXUTGetD3D11Enumeration();
    CGrowableArray <CD3D11EnumAdapterInfo*>* pAdapterList = pd3dEnum->GetAdapterInfoList();
    for( int iAdapter = 0; iAdapter < pAdapterList->GetSize(); ++iAdapter )
    {
        CD3D11EnumAdapterInfo* pAdapterInfo = pAdapterList->GetAt( iAdapter );
        for( int o = 0; o < pAdapterInfo->outputInfoList.GetSize(); ++o )
        {
            CD3D11EnumOutputInfo* pOutputInfo = pAdapterInfo->outputInfoList.GetAt( o );
            DXGI_OUTPUT_DESC Desc;
            pOutputInfo->m_pOutput->GetDesc( &Desc );

            if( hMonitor == Desc.Monitor )
            {
                *pOutputOrdinal = pOutputInfo->Output;
                return S_OK;
            }
        }
    }

    return E_FAIL;
}

//--------------------------------------------------------------------------------------
// This method is called when D3DERR_DEVICEREMOVED is returned from an API.  DXUT
// calls the application's DeviceRemoved callback to inform it of the event.  The
// application returns true if it wants DXUT to look for a closest device to run on.
// If no device is found, or the app returns false, DXUT shuts down.
//--------------------------------------------------------------------------------------
HRESULT DXUTHandleDeviceRemoved()
{
    HRESULT hr = S_OK;

    // Device has been removed. Call the application's callback if set.  If no callback
    // has been set, then just look for a new device
    bool bLookForNewDevice = true;
    LPDXUTCALLBACKDEVICEREMOVED pDeviceRemovedFunc = GetDXUTState().GetDeviceRemovedFunc();
    if( pDeviceRemovedFunc )
        bLookForNewDevice = pDeviceRemovedFunc( GetDXUTState().GetDeviceRemovedFuncUserContext() );

    if( bLookForNewDevice )
    {
        DXUTDeviceSettings* pDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();


        hr = DXUTSnapDeviceSettingsToEnumDevice( pDeviceSettings, false);
        if( SUCCEEDED( hr ) )
        {
            // Change to a Direct3D device created from the new device settings
            // that is compatible with the removed device.
            hr = DXUTChangeDevice( pDeviceSettings, NULL, NULL, true, false );
            if( SUCCEEDED( hr ) )
                return S_OK;
        }
    }

    // The app does not wish to continue or continuing is not possible.
    return DXUTERR_DEVICEREMOVED;
}


//--------------------------------------------------------------------------------------
// Stores back buffer surface desc in GetDXUTState().GetBackBufferSurfaceDesc10()
//--------------------------------------------------------------------------------------
void DXUTUpdateBackBufferDesc()
{
    if( DXUTIsCurrentDeviceD3D9() )
    {
        HRESULT hr;
        IDirect3DSurface9* pBackBuffer;
        hr = GetDXUTState().GetD3D9Device()->GetBackBuffer( 0, 0, D3DBACKBUFFER_TYPE_MONO, &pBackBuffer );
        D3DSURFACE_DESC* pBBufferSurfaceDesc = GetDXUTState().GetBackBufferSurfaceDesc9();
        ZeroMemory( pBBufferSurfaceDesc, sizeof( D3DSURFACE_DESC ) );
        if( SUCCEEDED( hr ) )
        {
            pBackBuffer->GetDesc( pBBufferSurfaceDesc );
            SAFE_RELEASE( pBackBuffer );
        }
    }
    else
    {
        HRESULT hr;
        ID3D11Texture2D* pBackBuffer;
        hr = GetDXUTState().GetDXGISwapChain()->GetBuffer( 0, __uuidof( *pBackBuffer ), ( LPVOID* )&pBackBuffer );
        DXGI_SURFACE_DESC* pBBufferSurfaceDesc = GetDXUTState().GetBackBufferSurfaceDescDXGI();
        ZeroMemory( pBBufferSurfaceDesc, sizeof( DXGI_SURFACE_DESC ) );
        if( SUCCEEDED( hr ) )
        {
            D3D11_TEXTURE2D_DESC TexDesc;
            pBackBuffer->GetDesc( &TexDesc );
            pBBufferSurfaceDesc->Width = ( UINT )TexDesc.Width;
            pBBufferSurfaceDesc->Height = ( UINT )TexDesc.Height;
            pBBufferSurfaceDesc->Format = TexDesc.Format;
            pBBufferSurfaceDesc->SampleDesc = TexDesc.SampleDesc;
            SAFE_RELEASE( pBackBuffer );
        }
    }
}


//--------------------------------------------------------------------------------------
// Setup cursor based on current settings (window/fullscreen mode, show cursor state, clip cursor state)
//--------------------------------------------------------------------------------------
void DXUTSetupCursor()
{
    if( DXUTIsCurrentDeviceD3D9() )
    {
        // Show the cursor again if returning to fullscreen 
        IDirect3DDevice9* pd3dDevice = DXUTGetD3D9Device();
        if( !DXUTIsWindowed() && pd3dDevice )
        {
            if( GetDXUTState().GetShowCursorWhenFullScreen() )
            {
                SetCursor( NULL ); // Turn off Windows cursor in full screen mode
                HCURSOR hCursor = ( HCURSOR )( ULONG_PTR )GetClassLongPtr( DXUTGetHWNDDeviceFullScreen(),
                                                                           GCLP_HCURSOR );
                DXUTSetD3D9DeviceCursor( pd3dDevice, hCursor, false );
                DXUTGetD3D9Device()->ShowCursor( true );
            }
            else
            {
                SetCursor( NULL ); // Turn off Windows cursor in full screen mode
                DXUTGetD3D9Device()->ShowCursor( false );
            }
        }

        // Clip cursor if requested
        if( !DXUTIsWindowed() && GetDXUTState().GetClipCursorWhenFullScreen() )
        {
            // Confine cursor to full screen window
            RECT rcWindow;
            GetWindowRect( DXUTGetHWNDDeviceFullScreen(), &rcWindow );
            ClipCursor( &rcWindow );
        }
        else
        {
            ClipCursor( NULL );
        }
    }
    else
    {
        // Clip cursor if requested
        if( !DXUTIsWindowed() && GetDXUTState().GetClipCursorWhenFullScreen() )
        {
            // Confine cursor to full screen window
            RECT rcWindow;
            GetWindowRect( DXUTGetHWNDDeviceFullScreen(), &rcWindow );
            ClipCursor( &rcWindow );
        }
        else
        {
            ClipCursor( NULL );
        }
    }
}


//--------------------------------------------------------------------------------------
// Updates the static part of the frame stats so it doesn't have be generated every frame
//--------------------------------------------------------------------------------------
void DXUTUpdateStaticFrameStats()
{
    if( GetDXUTState().GetNoStats() )
        return;

    DXUTDeviceSettings* pDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();
    if( NULL == pDeviceSettings )
        return;

    if( DXUTIsD3D9( pDeviceSettings ) )
    {
        CD3D9Enumeration* pd3dEnum = DXUTGetD3D9Enumeration();
        if( NULL == pd3dEnum )
            return;

        CD3D9EnumDeviceSettingsCombo* pDeviceSettingsCombo = pd3dEnum->GetDeviceSettingsCombo(
            pDeviceSettings->d3d9.AdapterOrdinal, pDeviceSettings->d3d9.DeviceType,
            pDeviceSettings->d3d9.AdapterFormat, pDeviceSettings->d3d9.pp.BackBufferFormat,
            pDeviceSettings->d3d9.pp.Windowed );
        if( NULL == pDeviceSettingsCombo )
            return;

        WCHAR strFmt[100];
        D3DPRESENT_PARAMETERS* pPP = &pDeviceSettings->d3d9.pp;

        if( pDeviceSettingsCombo->AdapterFormat == pDeviceSettingsCombo->BackBufferFormat )
        {
            wcscpy_s( strFmt, 100, DXUTD3DFormatToString( pDeviceSettingsCombo->AdapterFormat, false ) );
        }
        else
        {
            swprintf_s( strFmt, 100, L"backbuf %s, adapter %s",
                             DXUTD3DFormatToString( pDeviceSettingsCombo->BackBufferFormat, false ),
                             DXUTD3DFormatToString( pDeviceSettingsCombo->AdapterFormat, false ) );
        }

        WCHAR strDepthFmt[100];
        if( pPP->EnableAutoDepthStencil )
        {
            swprintf_s( strDepthFmt, 100, L" (%s)", DXUTD3DFormatToString( pPP->AutoDepthStencilFormat, false ) );
        }
        else
        {
            // No depth buffer
            strDepthFmt[0] = 0;
        }

        WCHAR strMultiSample[100];
        switch( pPP->MultiSampleType )
        {
            case D3DMULTISAMPLE_NONMASKABLE:
                wcscpy_s( strMultiSample, 100, L" (Nonmaskable Multisample)" ); break;
            case D3DMULTISAMPLE_NONE:
                wcscpy_s( strMultiSample, 100, L"" ); break;
            default:
                swprintf_s( strMultiSample, 100, L" (%dx Multisample)", pPP->MultiSampleType ); break;
        }

        WCHAR* pstrStaticFrameStats = GetDXUTState().GetStaticFrameStats();
        swprintf_s( pstrStaticFrameStats, 256, L"D3D9 %%sVsync %s (%dx%d), %s%s%s",
                         ( pPP->PresentationInterval == D3DPRESENT_INTERVAL_IMMEDIATE ) ? L"off" : L"on",
                         pPP->BackBufferWidth, pPP->BackBufferHeight,
                         strFmt, strDepthFmt, strMultiSample );
    }
    else
    {
        // D3D11
        CD3D11Enumeration* pd3dEnum = DXUTGetD3D11Enumeration();
        if( NULL == pd3dEnum )
            return;

        CD3D11EnumDeviceSettingsCombo* pDeviceSettingsCombo = pd3dEnum->GetDeviceSettingsCombo(
            pDeviceSettings->d3d11.AdapterOrdinal, pDeviceSettings->d3d11.DriverType, pDeviceSettings->d3d11.Output,
            pDeviceSettings->d3d11.sd.BufferDesc.Format, pDeviceSettings->d3d11.sd.Windowed );
        if( NULL == pDeviceSettingsCombo )
            return;

        WCHAR strFmt[100];

        wcscpy_s( strFmt, 100, DXUTDXGIFormatToString( pDeviceSettingsCombo->BackBufferFormat, false ) );

        WCHAR strMultiSample[100];
        swprintf_s( strMultiSample, 100, L" (MS%u, Q%u)", pDeviceSettings->d3d11.sd.SampleDesc.Count,
                         pDeviceSettings->d3d11.sd.SampleDesc.Quality );

        WCHAR* pstrStaticFrameStats = GetDXUTState().GetStaticFrameStats();
        swprintf_s( pstrStaticFrameStats, 256, L"D3D11 %%sVsync %s (%dx%d), %s%s",
                         ( pDeviceSettings->d3d11.SyncInterval == 0 ) ? L"off" : L"on",
                         pDeviceSettings->d3d11.sd.BufferDesc.Width, pDeviceSettings->d3d11.sd.BufferDesc.Height,
                         strFmt, strMultiSample );
    }
}


//--------------------------------------------------------------------------------------
// Updates the frames/sec stat once per second
//--------------------------------------------------------------------------------------
void DXUTUpdateFrameStats()
{
    if( GetDXUTState().GetNoStats() )
        return;

    // Keep track of the frame count
    double fLastTime = GetDXUTState().GetLastStatsUpdateTime();
    DWORD dwFrames = GetDXUTState().GetLastStatsUpdateFrames();
    double fAbsTime = GetDXUTState().GetAbsoluteTime();
    dwFrames++;
    GetDXUTState().SetLastStatsUpdateFrames( dwFrames );

    // Update the scene stats once per second
    if( fAbsTime - fLastTime > 1.0f )
    {
        float fFPS = ( float )( dwFrames / ( fAbsTime - fLastTime ) );
        GetDXUTState().SetFPS( fFPS );
        GetDXUTState().SetLastStatsUpdateTime( fAbsTime );
        GetDXUTState().SetLastStatsUpdateFrames( 0 );

        WCHAR* pstrFPS = GetDXUTState().GetFPSStats();
        swprintf_s( pstrFPS, 64, L"%0.2f fps ", fFPS );
    }
}

//--------------------------------------------------------------------------------------
// Returns a string describing the current device.  If bShowFPS is true, then
// the string contains the frames/sec.  If "-nostats" was used in 
// the command line, the string will be blank
//--------------------------------------------------------------------------------------
LPCWSTR WINAPI DXUTGetFrameStats( bool bShowFPS )
{
    WCHAR* pstrFrameStats = GetDXUTState().GetFrameStats();
    WCHAR* pstrFPS = ( bShowFPS ) ? GetDXUTState().GetFPSStats() : L"";
    swprintf_s( pstrFrameStats, 256, GetDXUTState().GetStaticFrameStats(), pstrFPS );
    return pstrFrameStats;
}


//--------------------------------------------------------------------------------------
// Updates the string which describes the device 
//--------------------------------------------------------------------------------------
void DXUTUpdateD3D9DeviceStats( D3DDEVTYPE DeviceType, DWORD BehaviorFlags,
                                D3DADAPTER_IDENTIFIER9* pAdapterIdentifier )
{
    if( GetDXUTState().GetNoStats() )
        return;

    // Store device description
    WCHAR* pstrDeviceStats = GetDXUTState().GetDeviceStats();
    if( DeviceType == D3DDEVTYPE_REF )
        wcscpy_s( pstrDeviceStats, 256, L"REF" );
    else if( DeviceType == D3DDEVTYPE_HAL )
        wcscpy_s( pstrDeviceStats, 256, L"HAL" );
    else if( DeviceType == D3DDEVTYPE_SW )
        wcscpy_s( pstrDeviceStats, 256, L"SW" );

    if( DeviceType == D3DDEVTYPE_HAL )
    {
        // Be sure not to overflow m_strDeviceStats when appending the adapter 
        // description, since it can be long.  
        wcscat_s( pstrDeviceStats, 256, L": " );

        // Try to get a unique description from the CD3D9EnumDeviceSettingsCombo
        DXUTDeviceSettings* pDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();
        if( !pDeviceSettings )
            return;

        CD3D9Enumeration* pd3dEnum = DXUTGetD3D9Enumeration();
        CD3D9EnumDeviceSettingsCombo* pDeviceSettingsCombo = pd3dEnum->GetDeviceSettingsCombo(
            pDeviceSettings->d3d9.AdapterOrdinal, pDeviceSettings->d3d9.DeviceType,
            pDeviceSettings->d3d9.AdapterFormat, pDeviceSettings->d3d9.pp.BackBufferFormat,
            pDeviceSettings->d3d9.pp.Windowed );
        if( pDeviceSettingsCombo )
        {
            wcscat_s( pstrDeviceStats, 256, pDeviceSettingsCombo->pAdapterInfo->szUniqueDescription );
        }
        else
        {
            const int cchDesc = sizeof( pAdapterIdentifier->Description );
            WCHAR szDescription[cchDesc];
            MultiByteToWideChar( CP_ACP, 0, pAdapterIdentifier->Description, -1, szDescription, cchDesc );
            szDescription[cchDesc - 1] = 0;
            wcscat_s( pstrDeviceStats, 256, szDescription );
        }
    }
}


//--------------------------------------------------------------------------------------
// Updates the string which describes the device 
//--------------------------------------------------------------------------------------
void DXUTUpdateD3D11DeviceStats( D3D_DRIVER_TYPE DeviceType, DXGI_ADAPTER_DESC* pAdapterDesc )
{
    if( GetDXUTState().GetNoStats() )
        return;

    // Store device description
    WCHAR* pstrDeviceStats = GetDXUTState().GetDeviceStats();
    if( DeviceType == D3D_DRIVER_TYPE_REFERENCE )
        wcscpy_s( pstrDeviceStats, 256, L"REFERENCE" );
    else if( DeviceType == D3D_DRIVER_TYPE_HARDWARE )
        wcscpy_s( pstrDeviceStats, 256, L"HARDWARE" );
    else if( DeviceType == D3D_DRIVER_TYPE_SOFTWARE )
        wcscpy_s( pstrDeviceStats, 256, L"SOFTWARE" );
    else if( DeviceType == D3D_DRIVER_TYPE_WARP )
        wcscpy_s( pstrDeviceStats, 256, L"WARP" );

    if( DeviceType == D3D_DRIVER_TYPE_HARDWARE )
    {
        // Be sure not to overflow m_strDeviceStats when appending the adapter 
        // description, since it can be long.  
        wcscat_s( pstrDeviceStats, 256, L": " );

        // Try to get a unique description from the CD3D11EnumDeviceSettingsCombo
        DXUTDeviceSettings* pDeviceSettings = GetDXUTState().GetCurrentDeviceSettings();
        if( !pDeviceSettings )
            return;

        CD3D11Enumeration* pd3dEnum = DXUTGetD3D11Enumeration();
        CD3D11EnumDeviceSettingsCombo* pDeviceSettingsCombo = pd3dEnum->GetDeviceSettingsCombo(
            pDeviceSettings->d3d11.AdapterOrdinal, pDeviceSettings->d3d11.DriverType, pDeviceSettings->d3d11.Output,
            pDeviceSettings->d3d11.sd.BufferDesc.Format, pDeviceSettings->d3d11.sd.Windowed );
        if( pDeviceSettingsCombo )
            wcscat_s( pstrDeviceStats, 256, pDeviceSettingsCombo->pAdapterInfo->szUniqueDescription );
        else
            wcscat_s( pstrDeviceStats, 256, pAdapterDesc->Description );
    }
}


//--------------------------------------------------------------------------------------
// Misc functions
//--------------------------------------------------------------------------------------
DXUTDeviceSettings WINAPI DXUTGetDeviceSettings()
{
    // Return a copy of device settings of the current device.  If no device exists yet, then
    // return a blank device settings struct
    DXUTDeviceSettings* pDS = GetDXUTState().GetCurrentDeviceSettings();
    if( pDS )
    {
        return *pDS;
    }
    else
    {
        DXUTDeviceSettings ds;
        ZeroMemory( &ds, sizeof( DXUTDeviceSettings ) );
        return ds;
    }
}

D3DPRESENT_PARAMETERS WINAPI DXUTGetD3D9PresentParameters()
{
    // Return a copy of the present params of the current device.  If no device exists yet, then
    // return blank present params
    DXUTDeviceSettings* pDS = GetDXUTState().GetCurrentDeviceSettings();
    if( pDS )
    {
        return pDS->d3d9.pp;
    }
    else
    {
        D3DPRESENT_PARAMETERS pp;
        ZeroMemory( &pp, sizeof( D3DPRESENT_PARAMETERS ) );
        return pp;
    }
}

bool WINAPI DXUTIsVsyncEnabled()
{
    DXUTDeviceSettings* pDS = GetDXUTState().GetCurrentDeviceSettings();
    if( pDS )
    {
        if( DXUTIsD3D9( pDS ) )
            return ( pDS->d3d9.pp.PresentationInterval == D3DPRESENT_INTERVAL_IMMEDIATE );
        else
            return ( pDS->d3d11.SyncInterval == 0 );
    }
    else
    {
        return true;
    }
};

HRESULT WINAPI DXUTGetD3D9DeviceCaps( DXUTDeviceSettings* pDeviceSettings, D3DCAPS9* pCaps )
{
    IDirect3D9* pD3D = DXUTGetD3D9Object();
    return pD3D->GetDeviceCaps( pDeviceSettings->d3d9.AdapterOrdinal, pDeviceSettings->d3d9.DeviceType, pCaps );
}


bool WINAPI DXUTIsKeyDown( BYTE vKey )
{
    bool* bKeys = GetDXUTState().GetKeys();
    if( vKey >= 0xA0 && vKey <= 0xA5 )  // VK_LSHIFT, VK_RSHIFT, VK_LCONTROL, VK_RCONTROL, VK_LMENU, VK_RMENU
        return GetAsyncKeyState( vKey ) != 0; // these keys only are tracked via GetAsyncKeyState()
    else if( vKey >= 0x01 && vKey <= 0x06 && vKey != 0x03 ) // mouse buttons (VK_*BUTTON)
        return DXUTIsMouseButtonDown( vKey );
    else
        return bKeys[vKey];
}

bool WINAPI DXUTWasKeyPressed( BYTE vKey )
{
    bool* bLastKeys = GetDXUTState().GetLastKeys();
    bool* bKeys = GetDXUTState().GetKeys();
    GetDXUTState().SetAppCalledWasKeyPressed( true );
    return ( !bLastKeys[vKey] && bKeys[vKey] );
}

bool WINAPI DXUTIsMouseButtonDown( BYTE vButton )
{
    bool* bMouseButtons = GetDXUTState().GetMouseButtons();
    int nIndex = DXUTMapButtonToArrayIndex( vButton );
    return bMouseButtons[nIndex];
}

void WINAPI DXUTSetMultimonSettings( bool bAutoChangeAdapter )
{
    GetDXUTState().SetAutoChangeAdapter( bAutoChangeAdapter );
}

void WINAPI DXUTSetHotkeyHandling( bool bAltEnterToToggleFullscreen, bool bEscapeToQuit, bool bPauseToToggleTimePause )
{
    GetDXUTState().SetHandleEscape( bEscapeToQuit );
    GetDXUTState().SetHandleAltEnter( bAltEnterToToggleFullscreen );
    GetDXUTState().SetHandlePause( bPauseToToggleTimePause );
}

void WINAPI DXUTSetCursorSettings( bool bShowCursorWhenFullScreen, bool bClipCursorWhenFullScreen )
{
    GetDXUTState().SetClipCursorWhenFullScreen( bClipCursorWhenFullScreen );
    GetDXUTState().SetShowCursorWhenFullScreen( bShowCursorWhenFullScreen );
    DXUTSetupCursor();
}

void WINAPI DXUTSetWindowSettings( bool bCallDefWindowProc )
{
    GetDXUTState().SetCallDefWindowProc( bCallDefWindowProc );
}

void WINAPI DXUTSetConstantFrameTime( bool bEnabled, float fTimePerFrame )
{
    if( GetDXUTState().GetOverrideConstantFrameTime() )
    {
        bEnabled = GetDXUTState().GetOverrideConstantFrameTime();
        fTimePerFrame = GetDXUTState().GetOverrideConstantTimePerFrame();
    }
    GetDXUTState().SetConstantFrameTime( bEnabled );
    GetDXUTState().SetTimePerFrame( fTimePerFrame );
}


//--------------------------------------------------------------------------------------
// Resets the state associated with DXUT 
//--------------------------------------------------------------------------------------
void WINAPI DXUTResetFrameworkState()
{
    GetDXUTState().Destroy();
    GetDXUTState().Create();
}


//--------------------------------------------------------------------------------------
// Closes down the window.  When the window closes, it will cleanup everything
//--------------------------------------------------------------------------------------
void WINAPI DXUTShutdown( int nExitCode )
{
    HWND hWnd = DXUTGetHWND();
    if( hWnd != NULL )
        SendMessage( hWnd, WM_CLOSE, 0, 0 );

    GetDXUTState().SetExitCode( nExitCode );

    DXUTCleanup3DEnvironment( true );

    // Restore shortcut keys (Windows key, accessibility shortcuts) to original state
    // This is important to call here if the shortcuts are disabled, 
    // because accessibility setting changes are permanent.
    // This means that if this is not done then the accessibility settings 
    // might not be the same as when the app was started. 
    // If the app crashes without restoring the settings, this is also true so it
    // would be wise to backup/restore the settings from a file so they can be 
    // restored when the crashed app is run again.
    DXUTAllowShortcutKeys( true );

    // Shutdown D3D9
    IDirect3D9* pD3D = GetDXUTState().GetD3D9();
    SAFE_RELEASE( pD3D );
    GetDXUTState().SetD3D9( NULL );

    // Shutdown D3D11
    IDXGIFactory1* pDXGIFactory = GetDXUTState().GetDXGIFactory();
    SAFE_RELEASE( pDXGIFactory );
    GetDXUTState().SetDXGIFactory( NULL );

    if( GetDXUTState().GetOverrideRelaunchMCE() )
        DXUTReLaunchMediaCenter();
}

//--------------------------------------------------------------------------------------
// Tells DXUT whether to operate in gamma correct mode
//--------------------------------------------------------------------------------------
void WINAPI DXUTSetIsInGammaCorrectMode( bool bGammaCorrect )
{
    GetDXUTState().SetIsInGammaCorrectMode( bGammaCorrect );
}


void DXUTApplyDefaultDeviceSettings(DXUTDeviceSettings *modifySettings) {
    ZeroMemory( modifySettings, sizeof( DXUTDeviceSettings ) );


    modifySettings->ver = DXUT_D3D11_DEVICE;
    modifySettings->d3d11.AdapterOrdinal = 0;
    modifySettings->d3d11.AutoCreateDepthStencil = true;
    modifySettings->d3d11.AutoDepthStencilFormat = DXGI_FORMAT_D24_UNORM_S8_UINT;
#if defined(DEBUG) || defined(_DEBUG)
    modifySettings->d3d11.CreateFlags |= D3D10_CREATE_DEVICE_DEBUG;
#else
    modifySettings->d3d11.CreateFlags = 0;
#endif
    modifySettings->d3d11.DriverType = D3D_DRIVER_TYPE_HARDWARE;
    modifySettings->d3d11.Output = 0;
    modifySettings->d3d11.PresentFlags = 0;
    modifySettings->d3d11.sd.BufferCount = 2;
    modifySettings->d3d11.sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    modifySettings->d3d11.sd.BufferDesc.Height = 480;
    modifySettings->d3d11.sd.BufferDesc.RefreshRate.Numerator = 60;
    modifySettings->d3d11.sd.BufferDesc.RefreshRate.Denominator = 1;
    modifySettings->d3d11.sd.BufferDesc.Scaling = DXGI_MODE_SCALING_UNSPECIFIED;
    modifySettings->d3d11.sd.BufferDesc.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
    modifySettings->d3d11.sd.BufferDesc.Width = 640;
    modifySettings->d3d11.sd.BufferUsage = 32;
    modifySettings->d3d11.sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH ;
    modifySettings->d3d11.sd.OutputWindow = DXUTGetHWND();
    modifySettings->d3d11.sd.SampleDesc.Count = 1;
    modifySettings->d3d11.sd.SampleDesc.Quality = 0;
    modifySettings->d3d11.sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
    modifySettings->d3d11.sd.Windowed = 1;
    modifySettings->d3d11.SyncInterval = 0;

    modifySettings->d3d9.AdapterFormat = D3DFMT_X8R8G8B8;
    modifySettings->d3d9.AdapterOrdinal = 0;
    modifySettings->d3d9.BehaviorFlags = D3DCREATE_HARDWARE_VERTEXPROCESSING;
    modifySettings->d3d9.DeviceType = D3DDEVTYPE_HAL;
    modifySettings->d3d9.pp.AutoDepthStencilFormat = D3DFMT_D24X8;
    modifySettings->d3d9.pp.BackBufferCount = 1;
    modifySettings->d3d9.pp.BackBufferFormat = D3DFMT_X8R8G8B8;
    modifySettings->d3d9.pp.BackBufferHeight = 480;
    modifySettings->d3d9.pp.BackBufferWidth = 640; 
    modifySettings->d3d9.pp.EnableAutoDepthStencil = 1;
    modifySettings->d3d9.pp.Flags = 2;
    modifySettings->d3d9.pp.FullScreen_RefreshRateInHz = 0;
    modifySettings->d3d9.pp.hDeviceWindow = DXUTGetHWND();
    modifySettings->d3d9.pp.MultiSampleQuality = 0;
    modifySettings->d3d9.pp.MultiSampleType = D3DMULTISAMPLE_NONE;
    modifySettings->d3d9.pp.PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE;
    modifySettings->d3d9.pp.SwapEffect = D3DSWAPEFFECT_DISCARD;
    modifySettings->d3d9.pp.Windowed = 1;
}



//--------------------------------------------------------------------------------------
// Update settings based on what is enumeratabled
//--------------------------------------------------------------------------------------
HRESULT DXUTSnapDeviceSettingsToEnumDevice( DXUTDeviceSettings* pDeviceSettings, bool forceEnum,  D3D_FEATURE_LEVEL forceFL ) {
    bool bAppSupportsD3D9 = DXUTDoesAppSupportD3D9();
    bool bAppSupportsD3D11 = DXUTDoesAppSupportD3D11();

    if( GetSystemMetrics(0x1000) != 0 ) {// SM_REMOTESESSION
        pDeviceSettings->d3d11.sd.Windowed = 1;
        pDeviceSettings->d3d9.pp.Windowed = 1;
    }   
    int bestModeIndex=0;
    int bestMSAAIndex=0;


    //DXUTSetDefaultDeviceSettings
    if (bAppSupportsD3D11 && pDeviceSettings->ver == DXUT_D3D11_DEVICE ) {
        CD3D11Enumeration *pEnum = NULL;


        pEnum = DXUTGetD3D11Enumeration( forceEnum, false, forceFL);

        CD3D11EnumAdapterInfo* pAdapterInfo = NULL; 
        CGrowableArray <CD3D11EnumAdapterInfo*>* pAdapterList = pEnum->GetAdapterInfoList();
        CD3D11EnumAdapterInfo* tempAdapterInfo = pAdapterList->GetAt( 0 );
        for( int iAdapter = 0; iAdapter < pAdapterList->GetSize(); iAdapter++ )
        {
            tempAdapterInfo = pAdapterList->GetAt( iAdapter );
            if (tempAdapterInfo->AdapterOrdinal == pDeviceSettings->d3d11.AdapterOrdinal) pAdapterInfo = tempAdapterInfo;
        }
        if (pAdapterInfo == NULL) return E_FAIL; // no adapters found.
        CD3D11EnumDeviceSettingsCombo* pDeviceSettingsCombo = NULL;
        float biggestScore = 0;

        int combo = 0;
        for( int iDeviceCombo = 0; iDeviceCombo < pAdapterInfo->deviceSettingsComboList.GetSize(); iDeviceCombo++ )
        {
            CD3D11EnumDeviceSettingsCombo* tempDeviceSettingsCombo = pAdapterInfo->deviceSettingsComboList.GetAt( iDeviceCombo );
    
            DXGI_MODE_DESC adapterDisplayMode;
            DXUTGetD3D11AdapterDisplayMode( pAdapterInfo->AdapterOrdinal, 0, &adapterDisplayMode );

            int bestMode;
            int bestMSAA;
            float score = DXUTRankD3D11DeviceCombo(tempDeviceSettingsCombo, &(pDeviceSettings->d3d11), &adapterDisplayMode, bestMode, bestMSAA );
            if (score > biggestScore) {
                combo = iDeviceCombo;
                biggestScore = score;
                pDeviceSettingsCombo = tempDeviceSettingsCombo;
                bestModeIndex = bestMode;
                bestMSAAIndex = bestMSAA;
            }
               
        }
        if (NULL == pDeviceSettingsCombo ) {
            return E_FAIL; // no settigns found.
        }

        pDeviceSettings->d3d11.AdapterOrdinal = pDeviceSettingsCombo->AdapterOrdinal;
        pDeviceSettings->d3d11.DriverType = pDeviceSettingsCombo->DeviceType;
        pDeviceSettings->d3d11.Output = pDeviceSettingsCombo->Output;
    
        pDeviceSettings->d3d11.sd.Windowed = pDeviceSettingsCombo->Windowed;
        if( GetSystemMetrics(0x1000) != 0 ) {// SM_REMOTESESSION
            pDeviceSettings->d3d11.sd.Windowed = 1;
        }   
        if (pDeviceSettingsCombo->pOutputInfo != NULL) {
            DXGI_MODE_DESC bestDisplayMode;
            bestDisplayMode = pDeviceSettingsCombo->pOutputInfo->displayModeList.GetAt(bestModeIndex);
            if (!pDeviceSettingsCombo->Windowed) {

                pDeviceSettings->d3d11.sd.BufferDesc.Height = bestDisplayMode.Height;
                pDeviceSettings->d3d11.sd.BufferDesc.Width = bestDisplayMode.Width;
                pDeviceSettings->d3d11.sd.BufferDesc.RefreshRate.Numerator = bestDisplayMode.RefreshRate.Numerator;
                pDeviceSettings->d3d11.sd.BufferDesc.RefreshRate.Denominator = bestDisplayMode.RefreshRate.Denominator;
                pDeviceSettings->d3d11.sd.BufferDesc.Scaling = bestDisplayMode.Scaling;
                pDeviceSettings->d3d11.sd.BufferDesc.ScanlineOrdering = bestDisplayMode.ScanlineOrdering;
            }
        }
        if (pDeviceSettings->d3d11.DeviceFeatureLevel == 0)
            pDeviceSettings->d3d11.DeviceFeatureLevel = pDeviceSettingsCombo->pDeviceInfo->SelectedLevel;
        


        
        pDeviceSettings->d3d11.sd.SampleDesc.Count = pDeviceSettingsCombo->multiSampleCountList.GetAt(bestMSAAIndex);
        if (pDeviceSettings->d3d11.sd.SampleDesc.Quality > pDeviceSettingsCombo->multiSampleQualityList.GetAt(bestMSAAIndex) - 1)
            pDeviceSettings->d3d11.sd.SampleDesc.Quality = pDeviceSettingsCombo->multiSampleQualityList.GetAt(bestMSAAIndex) - 1;
        
        pDeviceSettings->d3d11.sd.BufferDesc.Format = pDeviceSettingsCombo->BackBufferFormat;

        return S_OK;
    }
        // didn't find a D3D11 adapter.
    if (bAppSupportsD3D9) {
      // Find the best combination of:
        //      Adapter Ordinal
        //      Device Type
        //      Adapter Format
        //      Back Buffer Format
        //      Windowed
        // given what's available on the system and the match options combined with the device settings input.
        // This combination of settings is encapsulated by the CD3D9EnumDeviceSettingsCombo class.
        float fBestRanking = -1.0f;
        CD3D9EnumDeviceSettingsCombo* pBestDeviceSettingsCombo = NULL;
        D3DDISPLAYMODE adapterDesktopDisplayMode;

        IDirect3D9* pD3D = DXUTGetD3D9Object();
        CD3D9Enumeration* pd3dEnum = DXUTGetD3D9Enumeration( forceEnum );
        CGrowableArray <CD3D9EnumAdapterInfo*>* pAdapterList = pd3dEnum->GetAdapterInfoList();
        for( int iAdapter = 0; iAdapter < pAdapterList->GetSize(); iAdapter++ )
        {
            CD3D9EnumAdapterInfo* pAdapterInfo = pAdapterList->GetAt( iAdapter );

            // Get the desktop display mode of adapter 
            pD3D->GetAdapterDisplayMode( pAdapterInfo->AdapterOrdinal, &adapterDesktopDisplayMode );

            // Enum all the device types supported by this adapter to find the best device settings
            for( int iDeviceInfo = 0; iDeviceInfo < pAdapterInfo->deviceInfoList.GetSize(); iDeviceInfo++ )
            {
                CD3D9EnumDeviceInfo* pDeviceInfo = pAdapterInfo->deviceInfoList.GetAt( iDeviceInfo );

                // Enum all the device settings combinations.  A device settings combination is 
                // a unique set of an adapter format, back buffer format, and IsWindowed.
                for( int iDeviceCombo = 0; iDeviceCombo < pDeviceInfo->deviceSettingsComboList.GetSize(); iDeviceCombo++ )
                {
                    CD3D9EnumDeviceSettingsCombo* pDeviceSettingsCombo = pDeviceInfo->deviceSettingsComboList.GetAt(
                        iDeviceCombo );

                    // If windowed mode the adapter format has to be the same as the desktop 
                    // display mode format so skip any that don't match
                    if( pDeviceSettingsCombo->Windowed &&
                        ( pDeviceSettingsCombo->AdapterFormat != adapterDesktopDisplayMode.Format ) )
                        continue;

                    // Skip any combo that doesn't meet the preserve match options
                    int bestMode;
                    int bestMSAA;

                    // Get a ranking number that describes how closely this device combo matches the optimal combo
                    float fCurRanking = DXUTRankD3D9DeviceCombo( pDeviceSettingsCombo,
                        &(pDeviceSettings->d3d9), &adapterDesktopDisplayMode, bestMode, bestMSAA );

                    // If this combo better matches the input device settings then save it
                    if( fCurRanking > fBestRanking )
                    {
                        pBestDeviceSettingsCombo = pDeviceSettingsCombo;
                        fBestRanking = fCurRanking;
                        bestModeIndex = bestMode;
                        bestMSAAIndex = bestMSAA;
                    }
                }
            }
        }

        // If no best device combination was found then fail
        if( pBestDeviceSettingsCombo == NULL )
            return DXUTERR_NOCOMPATIBLEDEVICES;

        // Using the best device settings combo found, build valid device settings taking heed of 
        // the match options and the input device settings
        pDeviceSettings->d3d9.AdapterFormat = pBestDeviceSettingsCombo->AdapterFormat;//D3DFMT_X8R8G8B8;
        pDeviceSettings->d3d9.AdapterOrdinal = pBestDeviceSettingsCombo->AdapterOrdinal ;//0;
        pDeviceSettings->d3d9.DeviceType = pBestDeviceSettingsCombo->DeviceType;
        pDeviceSettings->d3d9.pp.BackBufferFormat = pBestDeviceSettingsCombo->BackBufferFormat;//D3DFMT_X8R8G8B8;
        if( GetSystemMetrics(0x1000) != 0 ) {// SM_REMOTESESSION
            pDeviceSettings->d3d9.pp.Windowed = 1;
        }          
        if (!pBestDeviceSettingsCombo->Windowed) {
            D3DDISPLAYMODE displayMode = pBestDeviceSettingsCombo->pAdapterInfo->displayModeList.GetAt( bestModeIndex );
            pDeviceSettings->d3d9.pp.BackBufferHeight = displayMode.Height;
            pDeviceSettings->d3d9.pp.BackBufferWidth = displayMode.Width; 
            pDeviceSettings->d3d9.pp.FullScreen_RefreshRateInHz = displayMode.RefreshRate;
        }
         pDeviceSettings->d3d9.pp.hDeviceWindow = pBestDeviceSettingsCombo->Windowed ? DXUTGetHWNDDeviceWindowed() : DXUTGetHWNDDeviceFullScreen();
        if (pDeviceSettings->d3d9.pp.MultiSampleQuality > pBestDeviceSettingsCombo->multiSampleQualityList.GetAt(bestMSAAIndex) - 1)
            pDeviceSettings->d3d9.pp.MultiSampleQuality = pBestDeviceSettingsCombo->multiSampleQualityList.GetAt( bestMSAAIndex )-1;
        
        pDeviceSettings->d3d9.pp.MultiSampleType = pBestDeviceSettingsCombo->multiSampleTypeList.GetAt( bestMSAAIndex );;
        pDeviceSettings->d3d9.pp.Windowed = pBestDeviceSettingsCombo->Windowed;
        return S_OK;
    }
    return E_FAIL;
}
