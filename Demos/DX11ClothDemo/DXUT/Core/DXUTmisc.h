//--------------------------------------------------------------------------------------
// File: DXUTMisc.h
//
// Helper functions for Direct3D programming.
//
// Copyright (c) Microsoft Corporation. All rights reserved
//--------------------------------------------------------------------------------------
#pragma once
#ifndef DXUT_MISC_H
#define DXUT_MISC_H

#ifndef MAX_FVF_DECL_SIZE
#define MAX_FVF_DECL_SIZE MAXD3DDECLLENGTH + 1 // +1 for END
#endif

//--------------------------------------------------------------------------------------
// XInput helper state/function
// This performs extra processing on XInput gamepad data to make it slightly more convenient to use
// 
// Example usage:
//
//      DXUT_GAMEPAD gamepad[4];
//      for( DWORD iPort=0; iPort<DXUT_MAX_CONTROLLERS; iPort++ )
//          DXUTGetGamepadState( iPort, gamepad[iPort] );
//
//--------------------------------------------------------------------------------------
#define DXUT_MAX_CONTROLLERS 4  // XInput handles up to 4 controllers 

struct DXUT_GAMEPAD
{
    // From XINPUT_GAMEPAD
    WORD wButtons;
    BYTE bLeftTrigger;
    BYTE bRightTrigger;
    SHORT sThumbLX;
    SHORT sThumbLY;
    SHORT sThumbRX;
    SHORT sThumbRY;

    // Device properties
    XINPUT_CAPABILITIES caps;
    bool bConnected; // If the controller is currently connected
    bool bInserted;  // If the controller was inserted this frame
    bool bRemoved;   // If the controller was removed this frame

    // Thumb stick values converted to range [-1,+1]
    float fThumbRX;
    float fThumbRY;
    float fThumbLX;
    float fThumbLY;

    // Records which buttons were pressed this frame.
    // These are only set on the first frame that the button is pressed
    WORD wPressedButtons;
    bool bPressedLeftTrigger;
    bool bPressedRightTrigger;

    // Last state of the buttons
    WORD wLastButtons;
    bool bLastLeftTrigger;
    bool bLastRightTrigger;
};

HRESULT DXUTGetGamepadState( DWORD dwPort, DXUT_GAMEPAD* pGamePad, bool bThumbstickDeadZone = true,
                             bool bSnapThumbstickToCardinals = true );
HRESULT DXUTStopRumbleOnAllControllers();
void DXUTEnableXInput( bool bEnable );


//--------------------------------------------------------------------------------------
// Takes a screen shot of a 32bit D3D9 back buffer and saves the images to a BMP file
//--------------------------------------------------------------------------------------
HRESULT DXUTSnapD3D9Screenshot( LPCTSTR szFileName );

//--------------------------------------------------------------------------------------
// Takes a screen shot of a 32bit D3D11 back buffer and saves the images to a BMP file
//--------------------------------------------------------------------------------------

HRESULT DXUTSnapD3D11Screenshot( LPCTSTR szFileName, D3DX11_IMAGE_FILE_FORMAT iff = D3DX11_IFF_DDS  );


//--------------------------------------------------------------------------------------
// A growable array
//--------------------------------------------------------------------------------------
template<typename TYPE> class CGrowableArray
{
public:
    CGrowableArray()  { m_pData = NULL; m_nSize = 0; m_nMaxSize = 0; }
    CGrowableArray( const CGrowableArray<TYPE>& a ) { for( int i=0; i < a.m_nSize; i++ ) Add( a.m_pData[i] ); }
    ~CGrowableArray() { RemoveAll(); }

    const TYPE& operator[]( int nIndex ) const { return GetAt( nIndex ); }
    TYPE& operator[]( int nIndex ) { return GetAt( nIndex ); }
   
    CGrowableArray& operator=( const CGrowableArray<TYPE>& a ) { if( this == &a ) return *this; RemoveAll(); for( int i=0; i < a.m_nSize; i++ ) Add( a.m_pData[i] ); return *this; }

    HRESULT SetSize( int nNewMaxSize );
    HRESULT Add( const TYPE& value );
    HRESULT Insert( int nIndex, const TYPE& value );
    HRESULT SetAt( int nIndex, const TYPE& value );
    TYPE&   GetAt( int nIndex ) const { assert( nIndex >= 0 && nIndex < m_nSize ); return m_pData[nIndex]; }
    int     GetSize() const { return m_nSize; }
    TYPE*   GetData() { return m_pData; }
    bool    Contains( const TYPE& value ){ return ( -1 != IndexOf( value ) ); }

    int     IndexOf( const TYPE& value ) { return ( m_nSize > 0 ) ? IndexOf( value, 0, m_nSize ) : -1; }
    int     IndexOf( const TYPE& value, int iStart ) { return IndexOf( value, iStart, m_nSize - iStart ); }
    int     IndexOf( const TYPE& value, int nIndex, int nNumElements );

    int     LastIndexOf( const TYPE& value ) { return ( m_nSize > 0 ) ? LastIndexOf( value, m_nSize-1, m_nSize ) : -1; }
    int     LastIndexOf( const TYPE& value, int nIndex ) { return LastIndexOf( value, nIndex, nIndex+1 ); }
    int     LastIndexOf( const TYPE& value, int nIndex, int nNumElements );

    HRESULT Remove( int nIndex );
    void    RemoveAll() { SetSize(0); }
    void	Reset() { m_nSize = 0; }

protected:
    TYPE* m_pData;      // the actual array of data
    int m_nSize;        // # of elements (upperBound - 1)
    int m_nMaxSize;     // max allocated

    HRESULT SetSizeInternal( int nNewMaxSize );  // This version doesn't call ctor or dtor.
};


//--------------------------------------------------------------------------------------
// Performs timer operations
// Use DXUTGetGlobalTimer() to get the global instance
//--------------------------------------------------------------------------------------
class CDXUTTimer
{
public:
    CDXUTTimer();

    void            Reset(); // resets the timer
    void            Start(); // starts the timer
    void            Stop();  // stop (or pause) the timer
    void            Advance(); // advance the timer by 0.1 seconds
    double          GetAbsoluteTime(); // get the absolute system time
    double          GetTime(); // get the current time
    float           GetElapsedTime(); // get the time that elapsed between Get*ElapsedTime() calls
    void            GetTimeValues( double* pfTime, double* pfAbsoluteTime, float* pfElapsedTime ); // get all time values at once
    bool            IsStopped(); // returns true if timer stopped

    // Limit the current thread to one processor (the current one). This ensures that timing code runs
    // on only one processor, and will not suffer any ill effects from power management.
    void            LimitThreadAffinityToCurrentProc();

protected:
    LARGE_INTEGER   GetAdjustedCurrentTime();

    bool m_bUsingQPF;
    bool m_bTimerStopped;
    LONGLONG m_llQPFTicksPerSec;

    LONGLONG m_llStopTime;
    LONGLONG m_llLastElapsedTime;
    LONGLONG m_llBaseTime;
};

CDXUTTimer*                 WINAPI DXUTGetGlobalTimer();


//--------------------------------------------------------------------------------------
// Returns the string for the given D3DFORMAT.
//       bWithPrefix determines whether the string should include the "D3DFMT_"
//--------------------------------------------------------------------------------------
LPCWSTR WINAPI DXUTD3DFormatToString( D3DFORMAT format, bool bWithPrefix );


//--------------------------------------------------------------------------------------
// Returns the string for the given DXGI_FORMAT.
//       bWithPrefix determines whether the string should include the "DXGI_FORMAT_"
//--------------------------------------------------------------------------------------
LPCWSTR WINAPI DXUTDXGIFormatToString( DXGI_FORMAT format, bool bWithPrefix );


//--------------------------------------------------------------------------------------
// Device settings conversion
//--------------------------------------------------------------------------------------
void WINAPI DXUTConvertDeviceSettings11to9( DXUTD3D11DeviceSettings* pIn, DXUTD3D9DeviceSettings* pOut );
void WINAPI DXUTConvertDeviceSettings9to11( DXUTD3D9DeviceSettings* pIn, DXUTD3D11DeviceSettings* pOut );

DXGI_FORMAT WINAPI ConvertFormatD3D9ToDXGI( D3DFORMAT fmt );
D3DFORMAT WINAPI ConvertFormatDXGIToD3D9( DXGI_FORMAT fmt );


//--------------------------------------------------------------------------------------
// Debug printing support
// See dxerr.h for more debug printing support
//--------------------------------------------------------------------------------------
void WINAPI DXUTOutputDebugStringW( LPCWSTR strMsg, ... );
void WINAPI DXUTOutputDebugStringA( LPCSTR strMsg, ... );
HRESULT WINAPI DXUTTrace( const CHAR* strFile, DWORD dwLine, HRESULT hr, const WCHAR* strMsg, bool bPopMsgBox );
void WINAPI DXUTTraceDecl( D3DVERTEXELEMENT9 decl[MAX_FVF_DECL_SIZE] );
WCHAR* WINAPI DXUTTraceD3DDECLUSAGEtoString( BYTE u );
WCHAR* WINAPI DXUTTraceD3DDECLMETHODtoString( BYTE m );
WCHAR* WINAPI DXUTTraceD3DDECLTYPEtoString( BYTE t );
WCHAR* WINAPI DXUTTraceWindowsMessage( UINT uMsg );

#ifdef UNICODE
#define DXUTOutputDebugString DXUTOutputDebugStringW
#else
#define DXUTOutputDebugString DXUTOutputDebugStringA
#endif

// These macros are very similar to dxerr's but it special cases the HRESULT defined
// by DXUT to pop better message boxes. 
#if defined(DEBUG) || defined(_DEBUG)
#define DXUT_ERR(str,hr)           DXUTTrace( __FILE__, (DWORD)__LINE__, hr, str, false )
#define DXUT_ERR_MSGBOX(str,hr)    DXUTTrace( __FILE__, (DWORD)__LINE__, hr, str, true )
#define DXUTTRACE                  DXUTOutputDebugString
#else
#define DXUT_ERR(str,hr)           (hr)
#define DXUT_ERR_MSGBOX(str,hr)    (hr)
#define DXUTTRACE                  (__noop)
#endif


//--------------------------------------------------------------------------------------
// Direct3D9 dynamic linking support -- calls top-level D3D9 APIs with graceful
// failure if APIs are not present.
//--------------------------------------------------------------------------------------

IDirect3D9 * WINAPI DXUT_Dynamic_Direct3DCreate9(UINT SDKVersion);
int WINAPI DXUT_Dynamic_D3DPERF_BeginEvent( D3DCOLOR col, LPCWSTR wszName );
int WINAPI DXUT_Dynamic_D3DPERF_EndEvent( void );
void WINAPI DXUT_Dynamic_D3DPERF_SetMarker( D3DCOLOR col, LPCWSTR wszName );
void WINAPI DXUT_Dynamic_D3DPERF_SetRegion( D3DCOLOR col, LPCWSTR wszName );
BOOL WINAPI DXUT_Dynamic_D3DPERF_QueryRepeatFrame( void );
void WINAPI DXUT_Dynamic_D3DPERF_SetOptions( DWORD dwOptions );
DWORD WINAPI DXUT_Dynamic_D3DPERF_GetStatus( void );
HRESULT WINAPI DXUT_Dynamic_CreateDXGIFactory1( REFIID rInterface, void** ppOut );

HRESULT WINAPI DXUT_Dynamic_D3D11CreateDevice( IDXGIAdapter* pAdapter,
                                               D3D_DRIVER_TYPE DriverType,
                                               HMODULE Software,
                                               UINT32 Flags,
                                               D3D_FEATURE_LEVEL* pFeatureLevels,
                                               UINT FeatureLevels,
                                               UINT32 SDKVersion,
                                               ID3D11Device** ppDevice,
                                               D3D_FEATURE_LEVEL* pFeatureLevel,
                                               ID3D11DeviceContext** ppImmediateContext );

bool DXUT_EnsureD3D11APIs( void );


//--------------------------------------------------------------------------------------
// Profiling/instrumentation support
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// Some D3DPERF APIs take a color that can be used when displaying user events in 
// performance analysis tools.  The following constants are provided for your 
// convenience, but you can use any colors you like.
//--------------------------------------------------------------------------------------
const D3DCOLOR              DXUT_PERFEVENTCOLOR = D3DCOLOR_XRGB( 200, 100, 100 );
const D3DCOLOR              DXUT_PERFEVENTCOLOR2 = D3DCOLOR_XRGB( 100, 200, 100 );
const D3DCOLOR              DXUT_PERFEVENTCOLOR3 = D3DCOLOR_XRGB( 100, 100, 200 );

//--------------------------------------------------------------------------------------
// The following macros provide a convenient way for your code to call the D3DPERF 
// functions only when PROFILE is defined.  If PROFILE is not defined (as for the final 
// release version of a program), these macros evaluate to nothing, so no detailed event
// information is embedded in your shipping program.  It is recommended that you create
// and use three build configurations for your projects:
//     Debug (nonoptimized code, asserts active, PROFILE defined to assist debugging)
//     Profile (optimized code, asserts disabled, PROFILE defined to assist optimization)
//     Release (optimized code, asserts disabled, PROFILE not defined)
//--------------------------------------------------------------------------------------
#ifdef PROFILE
// PROFILE is defined, so these macros call the D3DPERF functions
#define DXUT_BeginPerfEvent( color, pstrMessage )   DXUT_Dynamic_D3DPERF_BeginEvent( color, pstrMessage )
#define DXUT_EndPerfEvent()                         DXUT_Dynamic_D3DPERF_EndEvent()
#define DXUT_SetPerfMarker( color, pstrMessage )    DXUT_Dynamic_D3DPERF_SetMarker( color, pstrMessage )
#else
// PROFILE is not defined, so these macros do nothing
#define DXUT_BeginPerfEvent( color, pstrMessage )   (__noop)
#define DXUT_EndPerfEvent()                         (__noop)
#define DXUT_SetPerfMarker( color, pstrMessage )    (__noop)
#endif

//--------------------------------------------------------------------------------------
// CDXUTPerfEventGenerator is a helper class that makes it easy to attach begin and end
// events to a block of code.  Simply define a CDXUTPerfEventGenerator variable anywhere 
// in a block of code, and the class's constructor will call DXUT_BeginPerfEvent when 
// the block of code begins, and the class's destructor will call DXUT_EndPerfEvent when 
// the block ends.
//--------------------------------------------------------------------------------------
class CDXUTPerfEventGenerator
{
public:
CDXUTPerfEventGenerator( D3DCOLOR color, LPCWSTR pstrMessage )
{
    DXUT_BeginPerfEvent( color, pstrMessage );
}
~CDXUTPerfEventGenerator( void )
{
    DXUT_EndPerfEvent();
}
};


//--------------------------------------------------------------------------------------
// Multimon handling to support OSes with or without multimon API support.  
// Purposely avoiding the use of multimon.h so DXUT.lib doesn't require 
// COMPILE_MULTIMON_STUBS and cause complication with MFC or other users of multimon.h
//--------------------------------------------------------------------------------------
#ifndef MONITOR_DEFAULTTOPRIMARY
#define MONITORINFOF_PRIMARY        0x00000001
#define MONITOR_DEFAULTTONULL       0x00000000
#define MONITOR_DEFAULTTOPRIMARY    0x00000001
#define MONITOR_DEFAULTTONEAREST    0x00000002
typedef struct tagMONITORINFO
{
    DWORD cbSize;
    RECT rcMonitor;
    RECT rcWork;
    DWORD dwFlags;
}                           MONITORINFO, *LPMONITORINFO;
typedef struct tagMONITORINFOEXW : public tagMONITORINFO
{
    WCHAR szDevice[CCHDEVICENAME];
}                           MONITORINFOEXW, *LPMONITORINFOEXW;
typedef MONITORINFOEXW      MONITORINFOEX;
typedef LPMONITORINFOEXW    LPMONITORINFOEX;
#endif

HMONITOR WINAPI DXUTMonitorFromWindow( HWND hWnd, DWORD dwFlags );
HMONITOR WINAPI DXUTMonitorFromRect( LPCRECT lprcScreenCoords, DWORD dwFlags );
BOOL WINAPI DXUTGetMonitorInfo( HMONITOR hMonitor, LPMONITORINFO lpMonitorInfo );
void WINAPI DXUTGetDesktopResolution( UINT AdapterOrdinal, UINT* pWidth, UINT* pHeight );


//--------------------------------------------------------------------------------------
// Implementation of CGrowableArray
//--------------------------------------------------------------------------------------

// This version doesn't call ctor or dtor.
template<typename TYPE> HRESULT CGrowableArray <TYPE>::SetSizeInternal( int nNewMaxSize )
{
    if( nNewMaxSize < 0 || ( nNewMaxSize > INT_MAX / sizeof( TYPE ) ) )
    {
        assert( false );
        return E_INVALIDARG;
    }

    if( nNewMaxSize == 0 )
    {
        // Shrink to 0 size & cleanup
        if( m_pData )
        {
            free( m_pData );
            m_pData = NULL;
        }

        m_nMaxSize = 0;
        m_nSize = 0;
    }
    else if( m_pData == NULL || nNewMaxSize > m_nMaxSize )
    {
        // Grow array
        int nGrowBy = ( m_nMaxSize == 0 ) ? 16 : m_nMaxSize;

        // Limit nGrowBy to keep m_nMaxSize less than INT_MAX
        if( ( UINT )m_nMaxSize + ( UINT )nGrowBy > ( UINT )INT_MAX )
            nGrowBy = INT_MAX - m_nMaxSize;

        nNewMaxSize = __max( nNewMaxSize, m_nMaxSize + nGrowBy );

        // Verify that (nNewMaxSize * sizeof(TYPE)) is not greater than UINT_MAX or the realloc will overrun
        if( sizeof( TYPE ) > UINT_MAX / ( UINT )nNewMaxSize )
            return E_INVALIDARG;

        TYPE* pDataNew = ( TYPE* )realloc( m_pData, nNewMaxSize * sizeof( TYPE ) );
        if( pDataNew == NULL )
            return E_OUTOFMEMORY;

        m_pData = pDataNew;
        m_nMaxSize = nNewMaxSize;
    }

    return S_OK;
}


//--------------------------------------------------------------------------------------
template<typename TYPE> HRESULT CGrowableArray <TYPE>::SetSize( int nNewMaxSize )
{
    int nOldSize = m_nSize;

    if( nOldSize > nNewMaxSize )
    {
        assert( m_pData );
        if( m_pData )
        {
            // Removing elements. Call dtor.

            for( int i = nNewMaxSize; i < nOldSize; ++i )
                m_pData[i].~TYPE();
        }
    }

    // Adjust buffer.  Note that there's no need to check for error
    // since if it happens, nOldSize == nNewMaxSize will be true.)
    HRESULT hr = SetSizeInternal( nNewMaxSize );

    if( nOldSize < nNewMaxSize )
    {
        assert( m_pData );
        if( m_pData )
        {
            // Adding elements. Call ctor.

            for( int i = nOldSize; i < nNewMaxSize; ++i )
                ::new ( &m_pData[i] ) TYPE;
        }
    }

    return hr;
}


//--------------------------------------------------------------------------------------
template<typename TYPE> HRESULT CGrowableArray <TYPE>::Add( const TYPE& value )
{
    HRESULT hr;
    if( FAILED( hr = SetSizeInternal( m_nSize + 1 ) ) )
        return hr;

    // Construct the new element
    ::new ( &m_pData[m_nSize] ) TYPE;

    // Assign
    m_pData[m_nSize] = value;
    ++m_nSize;

    return S_OK;
}


//--------------------------------------------------------------------------------------
template<typename TYPE> HRESULT CGrowableArray <TYPE>::Insert( int nIndex, const TYPE& value )
{
    HRESULT hr;

    // Validate index
    if( nIndex < 0 ||
        nIndex > m_nSize )
    {
        assert( false );
        return E_INVALIDARG;
    }

    // Prepare the buffer
    if( FAILED( hr = SetSizeInternal( m_nSize + 1 ) ) )
        return hr;

    // Shift the array
    MoveMemory( &m_pData[nIndex + 1], &m_pData[nIndex], sizeof( TYPE ) * ( m_nSize - nIndex ) );

    // Construct the new element
    ::new ( &m_pData[nIndex] ) TYPE;

    // Set the value and increase the size
    m_pData[nIndex] = value;
    ++m_nSize;

    return S_OK;
}


//--------------------------------------------------------------------------------------
template<typename TYPE> HRESULT CGrowableArray <TYPE>::SetAt( int nIndex, const TYPE& value )
{
    // Validate arguments
    if( nIndex < 0 ||
        nIndex >= m_nSize )
    {
        assert( false );
        return E_INVALIDARG;
    }

    m_pData[nIndex] = value;
    return S_OK;
}


//--------------------------------------------------------------------------------------
// Searches for the specified value and returns the index of the first occurrence
// within the section of the data array that extends from iStart and contains the 
// specified number of elements. Returns -1 if value is not found within the given 
// section.
//--------------------------------------------------------------------------------------
template<typename TYPE> int CGrowableArray <TYPE>::IndexOf( const TYPE& value, int iStart, int nNumElements )
{
    // Validate arguments
    if( iStart < 0 ||
        iStart >= m_nSize ||
        nNumElements < 0 ||
        iStart + nNumElements > m_nSize )
    {
        assert( false );
        return -1;
    }

    // Search
    for( int i = iStart; i < ( iStart + nNumElements ); i++ )
    {
        if( value == m_pData[i] )
            return i;
    }

    // Not found
    return -1;
}


//--------------------------------------------------------------------------------------
// Searches for the specified value and returns the index of the last occurrence
// within the section of the data array that contains the specified number of elements
// and ends at iEnd. Returns -1 if value is not found within the given section.
//--------------------------------------------------------------------------------------
template<typename TYPE> int CGrowableArray <TYPE>::LastIndexOf( const TYPE& value, int iEnd, int nNumElements )
{
    // Validate arguments
    if( iEnd < 0 ||
        iEnd >= m_nSize ||
        nNumElements < 0 ||
        iEnd - nNumElements < 0 )
    {
        assert( false );
        return -1;
    }

    // Search
    for( int i = iEnd; i > ( iEnd - nNumElements ); i-- )
    {
        if( value == m_pData[i] )
            return i;
    }

    // Not found
    return -1;
}



//--------------------------------------------------------------------------------------
template<typename TYPE> HRESULT CGrowableArray <TYPE>::Remove( int nIndex )
{
    if( nIndex < 0 ||
        nIndex >= m_nSize )
    {
        assert( false );
        return E_INVALIDARG;
    }

    // Destruct the element to be removed
    m_pData[nIndex].~TYPE();

    // Compact the array and decrease the size
    MoveMemory( &m_pData[nIndex], &m_pData[nIndex + 1], sizeof( TYPE ) * ( m_nSize - ( nIndex + 1 ) ) );
    --m_nSize;

    return S_OK;
}

//--------------------------------------------------------------------------------------
// Creates a REF or NULLREF D3D9 device and returns that device.  The caller should call
// Release() when done with the device.
//--------------------------------------------------------------------------------------
IDirect3DDevice9*           WINAPI DXUTCreateRefDevice9( HWND hWnd, bool bNullRef = true );

//--------------------------------------------------------------------------------------
// Creates a REF or NULLREF D3D10 device and returns the device.  The caller should call
// Release() when done with the device.
//--------------------------------------------------------------------------------------
//test d3d10 version ID3D10Device*               WINAPI DXUTCreateRefDevice10( bool bNullRef = true );

//--------------------------------------------------------------------------------------
// Helper function to launch the Media Center UI after the program terminates
//--------------------------------------------------------------------------------------
bool DXUTReLaunchMediaCenter();

//--------------------------------------------------------------------------------------
// Helper functions to create SRGB formats from typeless formats and vice versa
//--------------------------------------------------------------------------------------
DXGI_FORMAT MAKE_SRGB( DXGI_FORMAT format );
DXGI_FORMAT MAKE_TYPELESS( DXGI_FORMAT format );

#endif
