//--------------------------------------------------------------------------------------
// File: DXUTMisc.cpp
//
// Shortcut macros and functions for using DX objects
//
// Copyright (c) Microsoft Corporation. All rights reserved
//--------------------------------------------------------------------------------------
#include "dxut.h"
#include <xinput.h>
#define DXUT_GAMEPAD_TRIGGER_THRESHOLD      30
#undef min // use __min instead
#undef max // use __max instead

CDXUTTimer* WINAPI DXUTGetGlobalTimer()
{
    // Using an accessor function gives control of the construction order
    static CDXUTTimer timer;
    return &timer;
}


//--------------------------------------------------------------------------------------
CDXUTTimer::CDXUTTimer()
{
    m_bTimerStopped = true;
    m_llQPFTicksPerSec = 0;

    m_llStopTime = 0;
    m_llLastElapsedTime = 0;
    m_llBaseTime = 0;

    // Use QueryPerformanceFrequency to get the frequency of the counter
    LARGE_INTEGER qwTicksPerSec = { 0 };
    QueryPerformanceFrequency( &qwTicksPerSec );
    m_llQPFTicksPerSec = qwTicksPerSec.QuadPart;
}


//--------------------------------------------------------------------------------------
void CDXUTTimer::Reset()
{
    LARGE_INTEGER qwTime = GetAdjustedCurrentTime();

    m_llBaseTime = qwTime.QuadPart;
    m_llLastElapsedTime = qwTime.QuadPart;
    m_llStopTime = 0;
    m_bTimerStopped = FALSE;
}


//--------------------------------------------------------------------------------------
void CDXUTTimer::Start()
{
    // Get the current time
    LARGE_INTEGER qwTime = { 0 };
    QueryPerformanceCounter( &qwTime );

    if( m_bTimerStopped )
        m_llBaseTime += qwTime.QuadPart - m_llStopTime;
    m_llStopTime = 0;
    m_llLastElapsedTime = qwTime.QuadPart;
    m_bTimerStopped = FALSE;
}


//--------------------------------------------------------------------------------------
void CDXUTTimer::Stop()
{
    if( !m_bTimerStopped )
    {
        LARGE_INTEGER qwTime = { 0 };
        QueryPerformanceCounter( &qwTime );
        m_llStopTime = qwTime.QuadPart;
        m_llLastElapsedTime = qwTime.QuadPart;
        m_bTimerStopped = TRUE;
    }
}


//--------------------------------------------------------------------------------------
void CDXUTTimer::Advance()
{
    m_llStopTime += m_llQPFTicksPerSec / 10;
}


//--------------------------------------------------------------------------------------
double CDXUTTimer::GetAbsoluteTime()
{
    LARGE_INTEGER qwTime = { 0 };
    QueryPerformanceCounter( &qwTime );

    double fTime = qwTime.QuadPart / ( double )m_llQPFTicksPerSec;

    return fTime;
}


//--------------------------------------------------------------------------------------
double CDXUTTimer::GetTime()
{
    LARGE_INTEGER qwTime = GetAdjustedCurrentTime();

    double fAppTime = ( double )( qwTime.QuadPart - m_llBaseTime ) / ( double )m_llQPFTicksPerSec;

    return fAppTime;
}


//--------------------------------------------------------------------------------------
void CDXUTTimer::GetTimeValues( double* pfTime, double* pfAbsoluteTime, float* pfElapsedTime )
{
    assert( pfTime && pfAbsoluteTime && pfElapsedTime );

    LARGE_INTEGER qwTime = GetAdjustedCurrentTime();

    float fElapsedTime = (float) ((double) ( qwTime.QuadPart - m_llLastElapsedTime ) / (double) m_llQPFTicksPerSec);
    m_llLastElapsedTime = qwTime.QuadPart;

    // Clamp the timer to non-negative values to ensure the timer is accurate.
    // fElapsedTime can be outside this range if processor goes into a 
    // power save mode or we somehow get shuffled to another processor.  
    // However, the main thread should call SetThreadAffinityMask to ensure that 
    // we don't get shuffled to another processor.  Other worker threads should NOT call 
    // SetThreadAffinityMask, but use a shared copy of the timer data gathered from 
    // the main thread.
    if( fElapsedTime < 0.0f )
        fElapsedTime = 0.0f;

    *pfAbsoluteTime = qwTime.QuadPart / ( double )m_llQPFTicksPerSec;
    *pfTime = ( qwTime.QuadPart - m_llBaseTime ) / ( double )m_llQPFTicksPerSec;
    *pfElapsedTime = fElapsedTime;
}


//--------------------------------------------------------------------------------------
float CDXUTTimer::GetElapsedTime()
{
    LARGE_INTEGER qwTime = GetAdjustedCurrentTime();

    double fElapsedTime = (float) ((double) ( qwTime.QuadPart - m_llLastElapsedTime ) / (double) m_llQPFTicksPerSec);
    m_llLastElapsedTime = qwTime.QuadPart;

    // See the explanation about clamping in CDXUTTimer::GetTimeValues()
    if( fElapsedTime < 0.0f )
        fElapsedTime = 0.0f;

    return ( float )fElapsedTime;
}


//--------------------------------------------------------------------------------------
// If stopped, returns time when stopped otherwise returns current time
//--------------------------------------------------------------------------------------
LARGE_INTEGER CDXUTTimer::GetAdjustedCurrentTime()
{
    LARGE_INTEGER qwTime;
    if( m_llStopTime != 0 )
        qwTime.QuadPart = m_llStopTime;
    else
        QueryPerformanceCounter( &qwTime );
    return qwTime;
}

//--------------------------------------------------------------------------------------
bool CDXUTTimer::IsStopped()
{
    return m_bTimerStopped;
}

//--------------------------------------------------------------------------------------
// Limit the current thread to one processor (the current one). This ensures that timing code 
// runs on only one processor, and will not suffer any ill effects from power management.
// See "Game Timing and Multicore Processors" for more details
//--------------------------------------------------------------------------------------
void CDXUTTimer::LimitThreadAffinityToCurrentProc()
{
    HANDLE hCurrentProcess = GetCurrentProcess();

    // Get the processor affinity mask for this process
    DWORD_PTR dwProcessAffinityMask = 0;
    DWORD_PTR dwSystemAffinityMask = 0;

    if( GetProcessAffinityMask( hCurrentProcess, &dwProcessAffinityMask, &dwSystemAffinityMask ) != 0 &&
        dwProcessAffinityMask )
    {
        // Find the lowest processor that our process is allows to run against
        DWORD_PTR dwAffinityMask = ( dwProcessAffinityMask & ( ( ~dwProcessAffinityMask ) + 1 ) );

        // Set this as the processor that our thread must always run against
        // This must be a subset of the process affinity mask
        HANDLE hCurrentThread = GetCurrentThread();
        if( INVALID_HANDLE_VALUE != hCurrentThread )
        {
            SetThreadAffinityMask( hCurrentThread, dwAffinityMask );
            CloseHandle( hCurrentThread );
        }
    }

    CloseHandle( hCurrentProcess );
}


//--------------------------------------------------------------------------------------
// Returns the string for the given D3DFORMAT.
//--------------------------------------------------------------------------------------
LPCWSTR WINAPI DXUTD3DFormatToString( D3DFORMAT format, bool bWithPrefix )
{
    WCHAR* pstr = NULL;
    switch( format )
    {
        case D3DFMT_UNKNOWN:
            pstr = L"D3DFMT_UNKNOWN"; break;
        case D3DFMT_R8G8B8:
            pstr = L"D3DFMT_R8G8B8"; break;
        case D3DFMT_A8R8G8B8:
            pstr = L"D3DFMT_A8R8G8B8"; break;
        case D3DFMT_X8R8G8B8:
            pstr = L"D3DFMT_X8R8G8B8"; break;
        case D3DFMT_R5G6B5:
            pstr = L"D3DFMT_R5G6B5"; break;
        case D3DFMT_X1R5G5B5:
            pstr = L"D3DFMT_X1R5G5B5"; break;
        case D3DFMT_A1R5G5B5:
            pstr = L"D3DFMT_A1R5G5B5"; break;
        case D3DFMT_A4R4G4B4:
            pstr = L"D3DFMT_A4R4G4B4"; break;
        case D3DFMT_R3G3B2:
            pstr = L"D3DFMT_R3G3B2"; break;
        case D3DFMT_A8:
            pstr = L"D3DFMT_A8"; break;
        case D3DFMT_A8R3G3B2:
            pstr = L"D3DFMT_A8R3G3B2"; break;
        case D3DFMT_X4R4G4B4:
            pstr = L"D3DFMT_X4R4G4B4"; break;
        case D3DFMT_A2B10G10R10:
            pstr = L"D3DFMT_A2B10G10R10"; break;
        case D3DFMT_A8B8G8R8:
            pstr = L"D3DFMT_A8B8G8R8"; break;
        case D3DFMT_X8B8G8R8:
            pstr = L"D3DFMT_X8B8G8R8"; break;
        case D3DFMT_G16R16:
            pstr = L"D3DFMT_G16R16"; break;
        case D3DFMT_A2R10G10B10:
            pstr = L"D3DFMT_A2R10G10B10"; break;
        case D3DFMT_A16B16G16R16:
            pstr = L"D3DFMT_A16B16G16R16"; break;
        case D3DFMT_A8P8:
            pstr = L"D3DFMT_A8P8"; break;
        case D3DFMT_P8:
            pstr = L"D3DFMT_P8"; break;
        case D3DFMT_L8:
            pstr = L"D3DFMT_L8"; break;
        case D3DFMT_A8L8:
            pstr = L"D3DFMT_A8L8"; break;
        case D3DFMT_A4L4:
            pstr = L"D3DFMT_A4L4"; break;
        case D3DFMT_V8U8:
            pstr = L"D3DFMT_V8U8"; break;
        case D3DFMT_L6V5U5:
            pstr = L"D3DFMT_L6V5U5"; break;
        case D3DFMT_X8L8V8U8:
            pstr = L"D3DFMT_X8L8V8U8"; break;
        case D3DFMT_Q8W8V8U8:
            pstr = L"D3DFMT_Q8W8V8U8"; break;
        case D3DFMT_V16U16:
            pstr = L"D3DFMT_V16U16"; break;
        case D3DFMT_A2W10V10U10:
            pstr = L"D3DFMT_A2W10V10U10"; break;
        case D3DFMT_UYVY:
            pstr = L"D3DFMT_UYVY"; break;
        case D3DFMT_YUY2:
            pstr = L"D3DFMT_YUY2"; break;
        case D3DFMT_DXT1:
            pstr = L"D3DFMT_DXT1"; break;
        case D3DFMT_DXT2:
            pstr = L"D3DFMT_DXT2"; break;
        case D3DFMT_DXT3:
            pstr = L"D3DFMT_DXT3"; break;
        case D3DFMT_DXT4:
            pstr = L"D3DFMT_DXT4"; break;
        case D3DFMT_DXT5:
            pstr = L"D3DFMT_DXT5"; break;
        case D3DFMT_D16_LOCKABLE:
            pstr = L"D3DFMT_D16_LOCKABLE"; break;
        case D3DFMT_D32:
            pstr = L"D3DFMT_D32"; break;
        case D3DFMT_D15S1:
            pstr = L"D3DFMT_D15S1"; break;
        case D3DFMT_D24S8:
            pstr = L"D3DFMT_D24S8"; break;
        case D3DFMT_D24X8:
            pstr = L"D3DFMT_D24X8"; break;
        case D3DFMT_D24X4S4:
            pstr = L"D3DFMT_D24X4S4"; break;
        case D3DFMT_D16:
            pstr = L"D3DFMT_D16"; break;
        case D3DFMT_L16:
            pstr = L"D3DFMT_L16"; break;
        case D3DFMT_VERTEXDATA:
            pstr = L"D3DFMT_VERTEXDATA"; break;
        case D3DFMT_INDEX16:
            pstr = L"D3DFMT_INDEX16"; break;
        case D3DFMT_INDEX32:
            pstr = L"D3DFMT_INDEX32"; break;
        case D3DFMT_Q16W16V16U16:
            pstr = L"D3DFMT_Q16W16V16U16"; break;
        case D3DFMT_MULTI2_ARGB8:
            pstr = L"D3DFMT_MULTI2_ARGB8"; break;
        case D3DFMT_R16F:
            pstr = L"D3DFMT_R16F"; break;
        case D3DFMT_G16R16F:
            pstr = L"D3DFMT_G16R16F"; break;
        case D3DFMT_A16B16G16R16F:
            pstr = L"D3DFMT_A16B16G16R16F"; break;
        case D3DFMT_R32F:
            pstr = L"D3DFMT_R32F"; break;
        case D3DFMT_G32R32F:
            pstr = L"D3DFMT_G32R32F"; break;
        case D3DFMT_A32B32G32R32F:
            pstr = L"D3DFMT_A32B32G32R32F"; break;
        case D3DFMT_CxV8U8:
            pstr = L"D3DFMT_CxV8U8"; break;
        default:
            pstr = L"Unknown format"; break;
    }
    if( bWithPrefix || wcsstr( pstr, L"D3DFMT_" ) == NULL )
        return pstr;
    else
        return pstr + lstrlen( L"D3DFMT_" );
}


//--------------------------------------------------------------------------------------
// Returns the string for the given DXGI_FORMAT.
//--------------------------------------------------------------------------------------
LPCWSTR WINAPI DXUTDXGIFormatToString( DXGI_FORMAT format, bool bWithPrefix )
{
    WCHAR* pstr = NULL;
    switch( format )
    {
        case DXGI_FORMAT_R32G32B32A32_TYPELESS:
            pstr = L"DXGI_FORMAT_R32G32B32A32_TYPELESS"; break;
        case DXGI_FORMAT_R32G32B32A32_FLOAT:
            pstr = L"DXGI_FORMAT_R32G32B32A32_FLOAT"; break;
        case DXGI_FORMAT_R32G32B32A32_UINT:
            pstr = L"DXGI_FORMAT_R32G32B32A32_UINT"; break;
        case DXGI_FORMAT_R32G32B32A32_SINT:
            pstr = L"DXGI_FORMAT_R32G32B32A32_SINT"; break;
        case DXGI_FORMAT_R32G32B32_TYPELESS:
            pstr = L"DXGI_FORMAT_R32G32B32_TYPELESS"; break;
        case DXGI_FORMAT_R32G32B32_FLOAT:
            pstr = L"DXGI_FORMAT_R32G32B32_FLOAT"; break;
        case DXGI_FORMAT_R32G32B32_UINT:
            pstr = L"DXGI_FORMAT_R32G32B32_UINT"; break;
        case DXGI_FORMAT_R32G32B32_SINT:
            pstr = L"DXGI_FORMAT_R32G32B32_SINT"; break;
        case DXGI_FORMAT_R16G16B16A16_TYPELESS:
            pstr = L"DXGI_FORMAT_R16G16B16A16_TYPELESS"; break;
        case DXGI_FORMAT_R16G16B16A16_FLOAT:
            pstr = L"DXGI_FORMAT_R16G16B16A16_FLOAT"; break;
        case DXGI_FORMAT_R16G16B16A16_UNORM:
            pstr = L"DXGI_FORMAT_R16G16B16A16_UNORM"; break;
        case DXGI_FORMAT_R16G16B16A16_UINT:
            pstr = L"DXGI_FORMAT_R16G16B16A16_UINT"; break;
        case DXGI_FORMAT_R16G16B16A16_SNORM:
            pstr = L"DXGI_FORMAT_R16G16B16A16_SNORM"; break;
        case DXGI_FORMAT_R16G16B16A16_SINT:
            pstr = L"DXGI_FORMAT_R16G16B16A16_SINT"; break;
        case DXGI_FORMAT_R32G32_TYPELESS:
            pstr = L"DXGI_FORMAT_R32G32_TYPELESS"; break;
        case DXGI_FORMAT_R32G32_FLOAT:
            pstr = L"DXGI_FORMAT_R32G32_FLOAT"; break;
        case DXGI_FORMAT_R32G32_UINT:
            pstr = L"DXGI_FORMAT_R32G32_UINT"; break;
        case DXGI_FORMAT_R32G32_SINT:
            pstr = L"DXGI_FORMAT_R32G32_SINT"; break;
        case DXGI_FORMAT_R32G8X24_TYPELESS:
            pstr = L"DXGI_FORMAT_R32G8X24_TYPELESS"; break;
        case DXGI_FORMAT_D32_FLOAT_S8X24_UINT:
            pstr = L"DXGI_FORMAT_D32_FLOAT_S8X24_UINT"; break;
        case DXGI_FORMAT_R32_FLOAT_X8X24_TYPELESS:
            pstr = L"DXGI_FORMAT_R32_FLOAT_X8X24_TYPELESS"; break;
        case DXGI_FORMAT_X32_TYPELESS_G8X24_UINT:
            pstr = L"DXGI_FORMAT_X32_TYPELESS_G8X24_UINT"; break;
        case DXGI_FORMAT_R10G10B10A2_TYPELESS:
            pstr = L"DXGI_FORMAT_R10G10B10A2_TYPELESS"; break;
        case DXGI_FORMAT_R10G10B10A2_UNORM:
            pstr = L"DXGI_FORMAT_R10G10B10A2_UNORM"; break;
        case DXGI_FORMAT_R10G10B10A2_UINT:
            pstr = L"DXGI_FORMAT_R10G10B10A2_UINT"; break;
        case DXGI_FORMAT_R11G11B10_FLOAT:
            pstr = L"DXGI_FORMAT_R11G11B10_FLOAT"; break;
        case DXGI_FORMAT_R8G8B8A8_TYPELESS:
            pstr = L"DXGI_FORMAT_R8G8B8A8_TYPELESS"; break;
        case DXGI_FORMAT_R8G8B8A8_UNORM:
            pstr = L"DXGI_FORMAT_R8G8B8A8_UNORM"; break;
        case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
            pstr = L"DXGI_FORMAT_R8G8B8A8_UNORM_SRGB"; break;
        case DXGI_FORMAT_R8G8B8A8_UINT:
            pstr = L"DXGI_FORMAT_R8G8B8A8_UINT"; break;
        case DXGI_FORMAT_R8G8B8A8_SNORM:
            pstr = L"DXGI_FORMAT_R8G8B8A8_SNORM"; break;
        case DXGI_FORMAT_R8G8B8A8_SINT:
            pstr = L"DXGI_FORMAT_R8G8B8A8_SINT"; break;
        case DXGI_FORMAT_R16G16_TYPELESS:
            pstr = L"DXGI_FORMAT_R16G16_TYPELESS"; break;
        case DXGI_FORMAT_R16G16_FLOAT:
            pstr = L"DXGI_FORMAT_R16G16_FLOAT"; break;
        case DXGI_FORMAT_R16G16_UNORM:
            pstr = L"DXGI_FORMAT_R16G16_UNORM"; break;
        case DXGI_FORMAT_R16G16_UINT:
            pstr = L"DXGI_FORMAT_R16G16_UINT"; break;
        case DXGI_FORMAT_R16G16_SNORM:
            pstr = L"DXGI_FORMAT_R16G16_SNORM"; break;
        case DXGI_FORMAT_R16G16_SINT:
            pstr = L"DXGI_FORMAT_R16G16_SINT"; break;
        case DXGI_FORMAT_R32_TYPELESS:
            pstr = L"DXGI_FORMAT_R32_TYPELESS"; break;
        case DXGI_FORMAT_D32_FLOAT:
            pstr = L"DXGI_FORMAT_D32_FLOAT"; break;
        case DXGI_FORMAT_R32_FLOAT:
            pstr = L"DXGI_FORMAT_R32_FLOAT"; break;
        case DXGI_FORMAT_R32_UINT:
            pstr = L"DXGI_FORMAT_R32_UINT"; break;
        case DXGI_FORMAT_R32_SINT:
            pstr = L"DXGI_FORMAT_R32_SINT"; break;
        case DXGI_FORMAT_R24G8_TYPELESS:
            pstr = L"DXGI_FORMAT_R24G8_TYPELESS"; break;
        case DXGI_FORMAT_D24_UNORM_S8_UINT:
            pstr = L"DXGI_FORMAT_D24_UNORM_S8_UINT"; break;
        case DXGI_FORMAT_R24_UNORM_X8_TYPELESS:
            pstr = L"DXGI_FORMAT_R24_UNORM_X8_TYPELESS"; break;
        case DXGI_FORMAT_X24_TYPELESS_G8_UINT:
            pstr = L"DXGI_FORMAT_X24_TYPELESS_G8_UINT"; break;
        case DXGI_FORMAT_R8G8_TYPELESS:
            pstr = L"DXGI_FORMAT_R8G8_TYPELESS"; break;
        case DXGI_FORMAT_R8G8_UNORM:
            pstr = L"DXGI_FORMAT_R8G8_UNORM"; break;
        case DXGI_FORMAT_R8G8_UINT:
            pstr = L"DXGI_FORMAT_R8G8_UINT"; break;
        case DXGI_FORMAT_R8G8_SNORM:
            pstr = L"DXGI_FORMAT_R8G8_SNORM"; break;
        case DXGI_FORMAT_R8G8_SINT:
            pstr = L"DXGI_FORMAT_R8G8_SINT"; break;
        case DXGI_FORMAT_R16_TYPELESS:
            pstr = L"DXGI_FORMAT_R16_TYPELESS"; break;
        case DXGI_FORMAT_R16_FLOAT:
            pstr = L"DXGI_FORMAT_R16_FLOAT"; break;
        case DXGI_FORMAT_D16_UNORM:
            pstr = L"DXGI_FORMAT_D16_UNORM"; break;
        case DXGI_FORMAT_R16_UNORM:
            pstr = L"DXGI_FORMAT_R16_UNORM"; break;
        case DXGI_FORMAT_R16_UINT:
            pstr = L"DXGI_FORMAT_R16_UINT"; break;
        case DXGI_FORMAT_R16_SNORM:
            pstr = L"DXGI_FORMAT_R16_SNORM"; break;
        case DXGI_FORMAT_R16_SINT:
            pstr = L"DXGI_FORMAT_R16_SINT"; break;
        case DXGI_FORMAT_R8_TYPELESS:
            pstr = L"DXGI_FORMAT_R8_TYPELESS"; break;
        case DXGI_FORMAT_R8_UNORM:
            pstr = L"DXGI_FORMAT_R8_UNORM"; break;
        case DXGI_FORMAT_R8_UINT:
            pstr = L"DXGI_FORMAT_R8_UINT"; break;
        case DXGI_FORMAT_R8_SNORM:
            pstr = L"DXGI_FORMAT_R8_SNORM"; break;
        case DXGI_FORMAT_R8_SINT:
            pstr = L"DXGI_FORMAT_R8_SINT"; break;
        case DXGI_FORMAT_A8_UNORM:
            pstr = L"DXGI_FORMAT_A8_UNORM"; break;
        case DXGI_FORMAT_R1_UNORM:
            pstr = L"DXGI_FORMAT_R1_UNORM"; break;
        case DXGI_FORMAT_R9G9B9E5_SHAREDEXP:
            pstr = L"DXGI_FORMAT_R9G9B9E5_SHAREDEXP"; break;
        case DXGI_FORMAT_R8G8_B8G8_UNORM:
            pstr = L"DXGI_FORMAT_R8G8_B8G8_UNORM"; break;
        case DXGI_FORMAT_G8R8_G8B8_UNORM:
            pstr = L"DXGI_FORMAT_G8R8_G8B8_UNORM"; break;
        case DXGI_FORMAT_BC1_TYPELESS:
            pstr = L"DXGI_FORMAT_BC1_TYPELESS"; break;
        case DXGI_FORMAT_BC1_UNORM:
            pstr = L"DXGI_FORMAT_BC1_UNORM"; break;
        case DXGI_FORMAT_BC1_UNORM_SRGB:
            pstr = L"DXGI_FORMAT_BC1_UNORM_SRGB"; break;
        case DXGI_FORMAT_BC2_TYPELESS:
            pstr = L"DXGI_FORMAT_BC2_TYPELESS"; break;
        case DXGI_FORMAT_BC2_UNORM:
            pstr = L"DXGI_FORMAT_BC2_UNORM"; break;
        case DXGI_FORMAT_BC2_UNORM_SRGB:
            pstr = L"DXGI_FORMAT_BC2_UNORM_SRGB"; break;
        case DXGI_FORMAT_BC3_TYPELESS:
            pstr = L"DXGI_FORMAT_BC3_TYPELESS"; break;
        case DXGI_FORMAT_BC3_UNORM:
            pstr = L"DXGI_FORMAT_BC3_UNORM"; break;
        case DXGI_FORMAT_BC3_UNORM_SRGB:
            pstr = L"DXGI_FORMAT_BC3_UNORM_SRGB"; break;
        case DXGI_FORMAT_BC4_TYPELESS:
            pstr = L"DXGI_FORMAT_BC4_TYPELESS"; break;
        case DXGI_FORMAT_BC4_UNORM:
            pstr = L"DXGI_FORMAT_BC4_UNORM"; break;
        case DXGI_FORMAT_BC4_SNORM:
            pstr = L"DXGI_FORMAT_BC4_SNORM"; break;
        case DXGI_FORMAT_BC5_TYPELESS:
            pstr = L"DXGI_FORMAT_BC5_TYPELESS"; break;
        case DXGI_FORMAT_BC5_UNORM:
            pstr = L"DXGI_FORMAT_BC5_UNORM"; break;
        case DXGI_FORMAT_BC5_SNORM:
            pstr = L"DXGI_FORMAT_BC5_SNORM"; break;
        case DXGI_FORMAT_B5G6R5_UNORM:
            pstr = L"DXGI_FORMAT_B5G6R5_UNORM"; break;
        case DXGI_FORMAT_B5G5R5A1_UNORM:
            pstr = L"DXGI_FORMAT_B5G5R5A1_UNORM"; break;
        case DXGI_FORMAT_B8G8R8A8_UNORM:
            pstr = L"DXGI_FORMAT_B8G8R8A8_UNORM"; break;
        default:
            pstr = L"Unknown format"; break;
    }
    if( bWithPrefix || wcsstr( pstr, L"DXGI_FORMAT_" ) == NULL )
        return pstr;
    else
        return pstr + lstrlen( L"DXGI_FORMAT_" );
}


//--------------------------------------------------------------------------------------
// Outputs to the debug stream a formatted Unicode string with a variable-argument list.
//--------------------------------------------------------------------------------------
VOID WINAPI DXUTOutputDebugStringW( LPCWSTR strMsg, ... )
{
#if defined(DEBUG) || defined(_DEBUG)
    WCHAR strBuffer[512];

    va_list args;
    va_start(args, strMsg);
    vswprintf_s( strBuffer, 512, strMsg, args );
    strBuffer[511] = L'\0';
    va_end(args);

    OutputDebugString( strBuffer );
#else
    UNREFERENCED_PARAMETER( strMsg );
#endif
}


//--------------------------------------------------------------------------------------
// Outputs to the debug stream a formatted MBCS string with a variable-argument list.
//--------------------------------------------------------------------------------------
VOID WINAPI DXUTOutputDebugStringA( LPCSTR strMsg, ... )
{
#if defined(DEBUG) || defined(_DEBUG)
    CHAR strBuffer[512];

    va_list args;
    va_start(args, strMsg);
    sprintf_s( strBuffer, 512, strMsg, args );
    strBuffer[511] = '\0';
    va_end(args);

    OutputDebugStringA( strBuffer );
#else
    UNREFERENCED_PARAMETER( strMsg );
#endif
}


//--------------------------------------------------------------------------------------
// Direct3D9 dynamic linking support -- calls top-level D3D9 APIs with graceful
// failure if APIs are not present.
//--------------------------------------------------------------------------------------

// Function prototypes
typedef IDirect3D9* (WINAPI * LPDIRECT3DCREATE9) (UINT);
typedef INT         (WINAPI * LPD3DPERF_BEGINEVENT)(D3DCOLOR, LPCWSTR);
typedef INT         (WINAPI * LPD3DPERF_ENDEVENT)(void);
typedef VOID        (WINAPI * LPD3DPERF_SETMARKER)(D3DCOLOR, LPCWSTR);
typedef VOID        (WINAPI * LPD3DPERF_SETREGION)(D3DCOLOR, LPCWSTR);
typedef BOOL        (WINAPI * LPD3DPERF_QUERYREPEATFRAME)(void);
typedef VOID        (WINAPI * LPD3DPERF_SETOPTIONS)( DWORD dwOptions );
typedef DWORD       (WINAPI * LPD3DPERF_GETSTATUS)( void );
typedef HRESULT     (WINAPI * LPCREATEDXGIFACTORY)(REFIID, void ** );
typedef HRESULT     (WINAPI * LPD3D11CREATEDEVICE)( IDXGIAdapter*, D3D_DRIVER_TYPE, HMODULE, UINT32, D3D_FEATURE_LEVEL*, UINT, UINT32, ID3D11Device**, D3D_FEATURE_LEVEL*, ID3D11DeviceContext** );

// Module and function pointers
static HMODULE                              s_hModD3D9 = NULL;
static LPDIRECT3DCREATE9                    s_DynamicDirect3DCreate9 = NULL;
static LPD3DPERF_BEGINEVENT                 s_DynamicD3DPERF_BeginEvent = NULL;
static LPD3DPERF_ENDEVENT                   s_DynamicD3DPERF_EndEvent = NULL;
static LPD3DPERF_SETMARKER                  s_DynamicD3DPERF_SetMarker = NULL;
static LPD3DPERF_SETREGION                  s_DynamicD3DPERF_SetRegion = NULL;
static LPD3DPERF_QUERYREPEATFRAME           s_DynamicD3DPERF_QueryRepeatFrame = NULL;
static LPD3DPERF_SETOPTIONS                 s_DynamicD3DPERF_SetOptions = NULL;
static LPD3DPERF_GETSTATUS                  s_DynamicD3DPERF_GetStatus = NULL;
static HMODULE                              s_hModDXGI = NULL;
static LPCREATEDXGIFACTORY                  s_DynamicCreateDXGIFactory = NULL;
static HMODULE                              s_hModD3D11 = NULL;
static LPD3D11CREATEDEVICE                  s_DynamicD3D11CreateDevice = NULL;

// Ensure function pointers are initialized
static bool DXUT_EnsureD3D9APIs( void )
{
    // If the module is non-NULL, this function has already been called.  Note
    // that this doesn't guarantee that all ProcAddresses were found.
    if( s_hModD3D9 != NULL )
        return true;

    // This may fail if Direct3D 9 isn't installed
    s_hModD3D9 = LoadLibrary( L"d3d9.dll" );
    if( s_hModD3D9 != NULL )
    {
        s_DynamicDirect3DCreate9 = (LPDIRECT3DCREATE9)GetProcAddress( s_hModD3D9, "Direct3DCreate9" );
        s_DynamicD3DPERF_BeginEvent = (LPD3DPERF_BEGINEVENT)GetProcAddress( s_hModD3D9, "D3DPERF_BeginEvent" );
        s_DynamicD3DPERF_EndEvent = (LPD3DPERF_ENDEVENT)GetProcAddress( s_hModD3D9, "D3DPERF_EndEvent" );
        s_DynamicD3DPERF_SetMarker = (LPD3DPERF_SETMARKER)GetProcAddress( s_hModD3D9, "D3DPERF_SetMarker" );
        s_DynamicD3DPERF_SetRegion = (LPD3DPERF_SETREGION)GetProcAddress( s_hModD3D9, "D3DPERF_SetRegion" );
        s_DynamicD3DPERF_QueryRepeatFrame = (LPD3DPERF_QUERYREPEATFRAME)GetProcAddress( s_hModD3D9, "D3DPERF_QueryRepeatFrame" );
        s_DynamicD3DPERF_SetOptions = (LPD3DPERF_SETOPTIONS)GetProcAddress( s_hModD3D9, "D3DPERF_SetOptions" );
        s_DynamicD3DPERF_GetStatus = (LPD3DPERF_GETSTATUS)GetProcAddress( s_hModD3D9, "D3DPERF_GetStatus" );
    }

    return s_hModD3D9 != NULL;
}

bool DXUT_EnsureD3D11APIs( void )
{
    // If both modules are non-NULL, this function has already been called.  Note
    // that this doesn't guarantee that all ProcAddresses were found.
    if( s_hModD3D11 != NULL && s_hModDXGI != NULL )
        return true;

    // This may fail if Direct3D 11 isn't installed
    s_hModD3D11 = LoadLibrary( L"d3d11.dll" );
    if( s_hModD3D11 != NULL )
    {
        s_DynamicD3D11CreateDevice = ( LPD3D11CREATEDEVICE )GetProcAddress( s_hModD3D11, "D3D11CreateDevice" );
    }

    if( !s_DynamicCreateDXGIFactory )
    {
        s_hModDXGI = LoadLibrary( L"dxgi.dll" );
        if( s_hModDXGI )
        {
            s_DynamicCreateDXGIFactory = ( LPCREATEDXGIFACTORY )GetProcAddress( s_hModDXGI, "CreateDXGIFactory1" );
        }

        return ( s_hModDXGI != NULL ) && ( s_hModD3D11 != NULL );
    }

    return ( s_hModD3D11 != NULL );
}

IDirect3D9* WINAPI DXUT_Dynamic_Direct3DCreate9( UINT SDKVersion )
{
    if( DXUT_EnsureD3D9APIs() && s_DynamicDirect3DCreate9 != NULL )
        return s_DynamicDirect3DCreate9( SDKVersion );
    else
        return NULL;
}

int WINAPI DXUT_Dynamic_D3DPERF_BeginEvent( D3DCOLOR col, LPCWSTR wszName )
{
    if( DXUT_EnsureD3D9APIs() && s_DynamicD3DPERF_BeginEvent != NULL )
        return s_DynamicD3DPERF_BeginEvent( col, wszName );
    else
        return -1;
}

int WINAPI DXUT_Dynamic_D3DPERF_EndEvent( void )
{
    if( DXUT_EnsureD3D9APIs() && s_DynamicD3DPERF_EndEvent != NULL )
        return s_DynamicD3DPERF_EndEvent();
    else
        return -1;
}

void WINAPI DXUT_Dynamic_D3DPERF_SetMarker( D3DCOLOR col, LPCWSTR wszName )
{
    if( DXUT_EnsureD3D9APIs() && s_DynamicD3DPERF_SetMarker != NULL )
        s_DynamicD3DPERF_SetMarker( col, wszName );
}

void WINAPI DXUT_Dynamic_D3DPERF_SetRegion( D3DCOLOR col, LPCWSTR wszName )
{
    if( DXUT_EnsureD3D9APIs() && s_DynamicD3DPERF_SetRegion != NULL )
        s_DynamicD3DPERF_SetRegion( col, wszName );
}

BOOL WINAPI DXUT_Dynamic_D3DPERF_QueryRepeatFrame( void )
{
    if( DXUT_EnsureD3D9APIs() && s_DynamicD3DPERF_QueryRepeatFrame != NULL )
        return s_DynamicD3DPERF_QueryRepeatFrame();
    else
        return FALSE;
}

void WINAPI DXUT_Dynamic_D3DPERF_SetOptions( DWORD dwOptions )
{
    if( DXUT_EnsureD3D9APIs() && s_DynamicD3DPERF_SetOptions != NULL )
        s_DynamicD3DPERF_SetOptions( dwOptions );
}

DWORD WINAPI DXUT_Dynamic_D3DPERF_GetStatus( void )
{
    if( DXUT_EnsureD3D9APIs() && s_DynamicD3DPERF_GetStatus != NULL )
        return s_DynamicD3DPERF_GetStatus();
    else
        return 0;
}

HRESULT WINAPI DXUT_Dynamic_CreateDXGIFactory1( REFIID rInterface, void** ppOut )
{
    if( DXUT_EnsureD3D11APIs() && s_DynamicCreateDXGIFactory != NULL )
        return s_DynamicCreateDXGIFactory( rInterface, ppOut );
    else
        return DXUTERR_NODIRECT3D11;
}



HRESULT WINAPI DXUT_Dynamic_D3D11CreateDevice( IDXGIAdapter* pAdapter,
                                               D3D_DRIVER_TYPE DriverType,
                                               HMODULE Software,
                                               UINT32 Flags,
                                               D3D_FEATURE_LEVEL* pFeatureLevels,
                                               UINT FeatureLevels,
                                               UINT32 SDKVersion,
                                               ID3D11Device** ppDevice,
                                               D3D_FEATURE_LEVEL* pFeatureLevel,
                                               ID3D11DeviceContext** ppImmediateContext )
{
    if( DXUT_EnsureD3D11APIs() && s_DynamicD3D11CreateDevice != NULL )
        return s_DynamicD3D11CreateDevice( pAdapter, DriverType, Software, Flags, pFeatureLevels, FeatureLevels,
                                           SDKVersion, ppDevice, pFeatureLevel, ppImmediateContext );
    else
        return DXUTERR_NODIRECT3D11;
}

//--------------------------------------------------------------------------------------
// Trace a string description of a decl 
//--------------------------------------------------------------------------------------
void WINAPI DXUTTraceDecl( D3DVERTEXELEMENT9 decl[MAX_FVF_DECL_SIZE] )
{
    int iDecl = 0;
    for( iDecl = 0; iDecl < MAX_FVF_DECL_SIZE; iDecl++ )
    {
        if( decl[iDecl].Stream == 0xFF )
            break;

        DXUTOutputDebugString( L"decl[%d]=Stream:%d, Offset:%d, %s, %s, %s, UsageIndex:%d\n", iDecl,
                               decl[iDecl].Stream,
                               decl[iDecl].Offset,
                               DXUTTraceD3DDECLTYPEtoString( decl[iDecl].Type ),
                               DXUTTraceD3DDECLMETHODtoString( decl[iDecl].Method ),
                               DXUTTraceD3DDECLUSAGEtoString( decl[iDecl].Usage ),
                               decl[iDecl].UsageIndex );
    }

    DXUTOutputDebugString( L"decl[%d]=D3DDECL_END\n", iDecl );
}

#define TRACE_ID(iD) case iD: return L#iD;

//--------------------------------------------------------------------------------------
WCHAR* WINAPI DXUTTraceWindowsMessage( UINT uMsg )
{
    switch( uMsg )
    {
        TRACE_ID(WM_NULL);
        TRACE_ID(WM_CREATE);
        TRACE_ID(WM_DESTROY);
        TRACE_ID(WM_MOVE);
        TRACE_ID(WM_SIZE);
        TRACE_ID(WM_ACTIVATE);
        TRACE_ID(WM_SETFOCUS);
        TRACE_ID(WM_KILLFOCUS);
        TRACE_ID(WM_ENABLE);
        TRACE_ID(WM_SETREDRAW);
        TRACE_ID(WM_SETTEXT);
        TRACE_ID(WM_GETTEXT);
        TRACE_ID(WM_GETTEXTLENGTH);
        TRACE_ID(WM_PAINT);
        TRACE_ID(WM_CLOSE);
        TRACE_ID(WM_QUERYENDSESSION);
        TRACE_ID(WM_QUERYOPEN);
        TRACE_ID(WM_ENDSESSION);
        TRACE_ID(WM_QUIT);
        TRACE_ID(WM_ERASEBKGND);
        TRACE_ID(WM_SYSCOLORCHANGE);
        TRACE_ID(WM_SHOWWINDOW);
        TRACE_ID(WM_WININICHANGE);
        TRACE_ID(WM_DEVMODECHANGE);
        TRACE_ID(WM_ACTIVATEAPP);
        TRACE_ID(WM_FONTCHANGE);
        TRACE_ID(WM_TIMECHANGE);
        TRACE_ID(WM_CANCELMODE);
        TRACE_ID(WM_SETCURSOR);
        TRACE_ID(WM_MOUSEACTIVATE);
        TRACE_ID(WM_CHILDACTIVATE);
        TRACE_ID(WM_QUEUESYNC);
        TRACE_ID(WM_GETMINMAXINFO);
        TRACE_ID(WM_PAINTICON);
        TRACE_ID(WM_ICONERASEBKGND);
        TRACE_ID(WM_NEXTDLGCTL);
        TRACE_ID(WM_SPOOLERSTATUS);
        TRACE_ID(WM_DRAWITEM);
        TRACE_ID(WM_MEASUREITEM);
        TRACE_ID(WM_DELETEITEM);
        TRACE_ID(WM_VKEYTOITEM);
        TRACE_ID(WM_CHARTOITEM);
        TRACE_ID(WM_SETFONT);
        TRACE_ID(WM_GETFONT);
        TRACE_ID(WM_SETHOTKEY);
        TRACE_ID(WM_GETHOTKEY);
        TRACE_ID(WM_QUERYDRAGICON);
        TRACE_ID(WM_COMPAREITEM);
        TRACE_ID(WM_GETOBJECT);
        TRACE_ID(WM_COMPACTING);
        TRACE_ID(WM_COMMNOTIFY);
        TRACE_ID(WM_WINDOWPOSCHANGING);
        TRACE_ID(WM_WINDOWPOSCHANGED);
        TRACE_ID(WM_POWER);
        TRACE_ID(WM_COPYDATA);
        TRACE_ID(WM_CANCELJOURNAL);
        TRACE_ID(WM_NOTIFY);
        TRACE_ID(WM_INPUTLANGCHANGEREQUEST);
        TRACE_ID(WM_INPUTLANGCHANGE);
        TRACE_ID(WM_TCARD);
        TRACE_ID(WM_HELP);
        TRACE_ID(WM_USERCHANGED);
        TRACE_ID(WM_NOTIFYFORMAT);
        TRACE_ID(WM_CONTEXTMENU);
        TRACE_ID(WM_STYLECHANGING);
        TRACE_ID(WM_STYLECHANGED);
        TRACE_ID(WM_DISPLAYCHANGE);
        TRACE_ID(WM_GETICON);
        TRACE_ID(WM_SETICON);
        TRACE_ID(WM_NCCREATE);
        TRACE_ID(WM_NCDESTROY);
        TRACE_ID(WM_NCCALCSIZE);
        TRACE_ID(WM_NCHITTEST);
        TRACE_ID(WM_NCPAINT);
        TRACE_ID(WM_NCACTIVATE);
        TRACE_ID(WM_GETDLGCODE);
        TRACE_ID(WM_SYNCPAINT);
        TRACE_ID(WM_NCMOUSEMOVE);
        TRACE_ID(WM_NCLBUTTONDOWN);
        TRACE_ID(WM_NCLBUTTONUP);
        TRACE_ID(WM_NCLBUTTONDBLCLK);
        TRACE_ID(WM_NCRBUTTONDOWN);
        TRACE_ID(WM_NCRBUTTONUP);
        TRACE_ID(WM_NCRBUTTONDBLCLK);
        TRACE_ID(WM_NCMBUTTONDOWN);
        TRACE_ID(WM_NCMBUTTONUP);
        TRACE_ID(WM_NCMBUTTONDBLCLK);
        TRACE_ID(WM_NCXBUTTONDOWN);
        TRACE_ID(WM_NCXBUTTONUP);
        TRACE_ID(WM_NCXBUTTONDBLCLK);
        TRACE_ID(WM_INPUT);
        TRACE_ID(WM_KEYDOWN);
        TRACE_ID(WM_KEYUP);
        TRACE_ID(WM_CHAR);
        TRACE_ID(WM_DEADCHAR);
        TRACE_ID(WM_SYSKEYDOWN);
        TRACE_ID(WM_SYSKEYUP);
        TRACE_ID(WM_SYSCHAR);
        TRACE_ID(WM_SYSDEADCHAR);
        TRACE_ID(WM_UNICHAR);
        TRACE_ID(WM_IME_STARTCOMPOSITION);
        TRACE_ID(WM_IME_ENDCOMPOSITION);
        TRACE_ID(WM_IME_COMPOSITION);
        TRACE_ID(WM_INITDIALOG);
        TRACE_ID(WM_COMMAND);
        TRACE_ID(WM_SYSCOMMAND);
        TRACE_ID(WM_TIMER);
        TRACE_ID(WM_HSCROLL);
        TRACE_ID(WM_VSCROLL);
        TRACE_ID(WM_INITMENU);
        TRACE_ID(WM_INITMENUPOPUP);
        TRACE_ID(WM_MENUSELECT);
        TRACE_ID(WM_MENUCHAR);
        TRACE_ID(WM_ENTERIDLE);
        TRACE_ID(WM_MENURBUTTONUP);
        TRACE_ID(WM_MENUDRAG);
        TRACE_ID(WM_MENUGETOBJECT);
        TRACE_ID(WM_UNINITMENUPOPUP);
        TRACE_ID(WM_MENUCOMMAND);
        TRACE_ID(WM_CHANGEUISTATE);
        TRACE_ID(WM_UPDATEUISTATE);
        TRACE_ID(WM_QUERYUISTATE);
        TRACE_ID(WM_CTLCOLORMSGBOX);
        TRACE_ID(WM_CTLCOLOREDIT);
        TRACE_ID(WM_CTLCOLORLISTBOX);
        TRACE_ID(WM_CTLCOLORBTN);
        TRACE_ID(WM_CTLCOLORDLG);
        TRACE_ID(WM_CTLCOLORSCROLLBAR);
        TRACE_ID(WM_CTLCOLORSTATIC);
        TRACE_ID(MN_GETHMENU);
        TRACE_ID(WM_MOUSEMOVE);
        TRACE_ID(WM_LBUTTONDOWN);
        TRACE_ID(WM_LBUTTONUP);
        TRACE_ID(WM_LBUTTONDBLCLK);
        TRACE_ID(WM_RBUTTONDOWN);
        TRACE_ID(WM_RBUTTONUP);
        TRACE_ID(WM_RBUTTONDBLCLK);
        TRACE_ID(WM_MBUTTONDOWN);
        TRACE_ID(WM_MBUTTONUP);
        TRACE_ID(WM_MBUTTONDBLCLK);
        TRACE_ID(WM_MOUSEWHEEL);
        TRACE_ID(WM_XBUTTONDOWN);
        TRACE_ID(WM_XBUTTONUP);
        TRACE_ID(WM_XBUTTONDBLCLK);
        TRACE_ID(WM_PARENTNOTIFY);
        TRACE_ID(WM_ENTERMENULOOP);
        TRACE_ID(WM_EXITMENULOOP);
        TRACE_ID(WM_NEXTMENU);
        TRACE_ID(WM_SIZING);
        TRACE_ID(WM_CAPTURECHANGED);
        TRACE_ID(WM_MOVING);
        TRACE_ID(WM_POWERBROADCAST);
        TRACE_ID(WM_DEVICECHANGE);
        TRACE_ID(WM_MDICREATE);
        TRACE_ID(WM_MDIDESTROY);
        TRACE_ID(WM_MDIACTIVATE);
        TRACE_ID(WM_MDIRESTORE);
        TRACE_ID(WM_MDINEXT);
        TRACE_ID(WM_MDIMAXIMIZE);
        TRACE_ID(WM_MDITILE);
        TRACE_ID(WM_MDICASCADE);
        TRACE_ID(WM_MDIICONARRANGE);
        TRACE_ID(WM_MDIGETACTIVE);
        TRACE_ID(WM_MDISETMENU);
        TRACE_ID(WM_ENTERSIZEMOVE);
        TRACE_ID(WM_EXITSIZEMOVE);
        TRACE_ID(WM_DROPFILES);
        TRACE_ID(WM_MDIREFRESHMENU);
        TRACE_ID(WM_IME_SETCONTEXT);
        TRACE_ID(WM_IME_NOTIFY);
        TRACE_ID(WM_IME_CONTROL);
        TRACE_ID(WM_IME_COMPOSITIONFULL);
        TRACE_ID(WM_IME_SELECT);
        TRACE_ID(WM_IME_CHAR);
        TRACE_ID(WM_IME_REQUEST);
        TRACE_ID(WM_IME_KEYDOWN);
        TRACE_ID(WM_IME_KEYUP);
        TRACE_ID(WM_MOUSEHOVER);
        TRACE_ID(WM_MOUSELEAVE);
        TRACE_ID(WM_NCMOUSEHOVER);
        TRACE_ID(WM_NCMOUSELEAVE);
        TRACE_ID(WM_WTSSESSION_CHANGE);
        TRACE_ID(WM_TABLET_FIRST);
        TRACE_ID(WM_TABLET_LAST);
        TRACE_ID(WM_CUT);
        TRACE_ID(WM_COPY);
        TRACE_ID(WM_PASTE);
        TRACE_ID(WM_CLEAR);
        TRACE_ID(WM_UNDO);
        TRACE_ID(WM_RENDERFORMAT);
        TRACE_ID(WM_RENDERALLFORMATS);
        TRACE_ID(WM_DESTROYCLIPBOARD);
        TRACE_ID(WM_DRAWCLIPBOARD);
        TRACE_ID(WM_PAINTCLIPBOARD);
        TRACE_ID(WM_VSCROLLCLIPBOARD);
        TRACE_ID(WM_SIZECLIPBOARD);
        TRACE_ID(WM_ASKCBFORMATNAME);
        TRACE_ID(WM_CHANGECBCHAIN);
        TRACE_ID(WM_HSCROLLCLIPBOARD);
        TRACE_ID(WM_QUERYNEWPALETTE);
        TRACE_ID(WM_PALETTEISCHANGING);
        TRACE_ID(WM_PALETTECHANGED);
        TRACE_ID(WM_HOTKEY);
        TRACE_ID(WM_PRINT);
        TRACE_ID(WM_PRINTCLIENT);
        TRACE_ID(WM_APPCOMMAND);
        TRACE_ID(WM_THEMECHANGED);
        TRACE_ID(WM_HANDHELDFIRST);
        TRACE_ID(WM_HANDHELDLAST);
        TRACE_ID(WM_AFXFIRST);
        TRACE_ID(WM_AFXLAST);
        TRACE_ID(WM_PENWINFIRST);
        TRACE_ID(WM_PENWINLAST);
        TRACE_ID(WM_APP);
        default:
            return L"Unknown";
    }
}


//--------------------------------------------------------------------------------------
WCHAR* WINAPI DXUTTraceD3DDECLTYPEtoString( BYTE t )
{
    switch( t )
    {
        case D3DDECLTYPE_FLOAT1:
            return L"D3DDECLTYPE_FLOAT1";
        case D3DDECLTYPE_FLOAT2:
            return L"D3DDECLTYPE_FLOAT2";
        case D3DDECLTYPE_FLOAT3:
            return L"D3DDECLTYPE_FLOAT3";
        case D3DDECLTYPE_FLOAT4:
            return L"D3DDECLTYPE_FLOAT4";
        case D3DDECLTYPE_D3DCOLOR:
            return L"D3DDECLTYPE_D3DCOLOR";
        case D3DDECLTYPE_UBYTE4:
            return L"D3DDECLTYPE_UBYTE4";
        case D3DDECLTYPE_SHORT2:
            return L"D3DDECLTYPE_SHORT2";
        case D3DDECLTYPE_SHORT4:
            return L"D3DDECLTYPE_SHORT4";
        case D3DDECLTYPE_UBYTE4N:
            return L"D3DDECLTYPE_UBYTE4N";
        case D3DDECLTYPE_SHORT2N:
            return L"D3DDECLTYPE_SHORT2N";
        case D3DDECLTYPE_SHORT4N:
            return L"D3DDECLTYPE_SHORT4N";
        case D3DDECLTYPE_USHORT2N:
            return L"D3DDECLTYPE_USHORT2N";
        case D3DDECLTYPE_USHORT4N:
            return L"D3DDECLTYPE_USHORT4N";
        case D3DDECLTYPE_UDEC3:
            return L"D3DDECLTYPE_UDEC3";
        case D3DDECLTYPE_DEC3N:
            return L"D3DDECLTYPE_DEC3N";
        case D3DDECLTYPE_FLOAT16_2:
            return L"D3DDECLTYPE_FLOAT16_2";
        case D3DDECLTYPE_FLOAT16_4:
            return L"D3DDECLTYPE_FLOAT16_4";
        case D3DDECLTYPE_UNUSED:
            return L"D3DDECLTYPE_UNUSED";
        default:
            return L"D3DDECLTYPE Unknown";
    }
}

WCHAR* WINAPI DXUTTraceD3DDECLMETHODtoString( BYTE m )
{
    switch( m )
    {
        case D3DDECLMETHOD_DEFAULT:
            return L"D3DDECLMETHOD_DEFAULT";
        case D3DDECLMETHOD_PARTIALU:
            return L"D3DDECLMETHOD_PARTIALU";
        case D3DDECLMETHOD_PARTIALV:
            return L"D3DDECLMETHOD_PARTIALV";
        case D3DDECLMETHOD_CROSSUV:
            return L"D3DDECLMETHOD_CROSSUV";
        case D3DDECLMETHOD_UV:
            return L"D3DDECLMETHOD_UV";
        case D3DDECLMETHOD_LOOKUP:
            return L"D3DDECLMETHOD_LOOKUP";
        case D3DDECLMETHOD_LOOKUPPRESAMPLED:
            return L"D3DDECLMETHOD_LOOKUPPRESAMPLED";
        default:
            return L"D3DDECLMETHOD Unknown";
    }
}

WCHAR* WINAPI DXUTTraceD3DDECLUSAGEtoString( BYTE u )
{
    switch( u )
    {
        case D3DDECLUSAGE_POSITION:
            return L"D3DDECLUSAGE_POSITION";
        case D3DDECLUSAGE_BLENDWEIGHT:
            return L"D3DDECLUSAGE_BLENDWEIGHT";
        case D3DDECLUSAGE_BLENDINDICES:
            return L"D3DDECLUSAGE_BLENDINDICES";
        case D3DDECLUSAGE_NORMAL:
            return L"D3DDECLUSAGE_NORMAL";
        case D3DDECLUSAGE_PSIZE:
            return L"D3DDECLUSAGE_PSIZE";
        case D3DDECLUSAGE_TEXCOORD:
            return L"D3DDECLUSAGE_TEXCOORD";
        case D3DDECLUSAGE_TANGENT:
            return L"D3DDECLUSAGE_TANGENT";
        case D3DDECLUSAGE_BINORMAL:
            return L"D3DDECLUSAGE_BINORMAL";
        case D3DDECLUSAGE_TESSFACTOR:
            return L"D3DDECLUSAGE_TESSFACTOR";
        case D3DDECLUSAGE_POSITIONT:
            return L"D3DDECLUSAGE_POSITIONT";
        case D3DDECLUSAGE_COLOR:
            return L"D3DDECLUSAGE_COLOR";
        case D3DDECLUSAGE_FOG:
            return L"D3DDECLUSAGE_FOG";
        case D3DDECLUSAGE_DEPTH:
            return L"D3DDECLUSAGE_DEPTH";
        case D3DDECLUSAGE_SAMPLE:
            return L"D3DDECLUSAGE_SAMPLE";
        default:
            return L"D3DDECLUSAGE Unknown";
    }
}


//--------------------------------------------------------------------------------------
// Multimon API handling for OSes with or without multimon API support
//--------------------------------------------------------------------------------------
#define DXUT_PRIMARY_MONITOR ((HMONITOR)0x12340042)
typedef HMONITOR ( WINAPI* LPMONITORFROMWINDOW )( HWND, DWORD );
typedef BOOL ( WINAPI* LPGETMONITORINFO )( HMONITOR, LPMONITORINFO );
typedef HMONITOR ( WINAPI* LPMONITORFROMRECT )( LPCRECT lprcScreenCoords, DWORD dwFlags );

BOOL WINAPI DXUTGetMonitorInfo( HMONITOR hMonitor, LPMONITORINFO lpMonitorInfo )
{
    static bool s_bInited = false;
    static LPGETMONITORINFO s_pFnGetMonitorInfo = NULL;
    if( !s_bInited )
    {
        s_bInited = true;
        HMODULE hUser32 = GetModuleHandle( L"USER32" );
        if( hUser32 )
        {
            OSVERSIONINFOA osvi =
            {
                0
            }; osvi.dwOSVersionInfoSize = sizeof( osvi ); GetVersionExA( ( OSVERSIONINFOA* )&osvi );
            bool bNT = ( VER_PLATFORM_WIN32_NT == osvi.dwPlatformId );
            s_pFnGetMonitorInfo = ( LPGETMONITORINFO )( bNT ? GetProcAddress( hUser32,
                                                                              "GetMonitorInfoW" ) :
                                                        GetProcAddress( hUser32, "GetMonitorInfoA" ) );
        }
    }

    if( s_pFnGetMonitorInfo )
        return s_pFnGetMonitorInfo( hMonitor, lpMonitorInfo );

    RECT rcWork;
    if( ( hMonitor == DXUT_PRIMARY_MONITOR ) && lpMonitorInfo && ( lpMonitorInfo->cbSize >= sizeof( MONITORINFO ) ) &&
        SystemParametersInfoA( SPI_GETWORKAREA, 0, &rcWork, 0 ) )
    {
        lpMonitorInfo->rcMonitor.left = 0;
        lpMonitorInfo->rcMonitor.top = 0;
        lpMonitorInfo->rcMonitor.right = GetSystemMetrics( SM_CXSCREEN );
        lpMonitorInfo->rcMonitor.bottom = GetSystemMetrics( SM_CYSCREEN );
        lpMonitorInfo->rcWork = rcWork;
        lpMonitorInfo->dwFlags = MONITORINFOF_PRIMARY;
        return TRUE;
    }
    return FALSE;
}


HMONITOR WINAPI DXUTMonitorFromWindow( HWND hWnd, DWORD dwFlags )
{
    static bool s_bInited = false;
    static LPMONITORFROMWINDOW s_pFnGetMonitorFromWindow = NULL;
    if( !s_bInited )
    {
        s_bInited = true;
        HMODULE hUser32 = GetModuleHandle( L"USER32" );
        if( hUser32 ) s_pFnGetMonitorFromWindow = ( LPMONITORFROMWINDOW )GetProcAddress( hUser32,
                                                                                         "MonitorFromWindow" );
    }

    if( s_pFnGetMonitorFromWindow )
        return s_pFnGetMonitorFromWindow( hWnd, dwFlags );
    else
        return DXUT_PRIMARY_MONITOR;
}


HMONITOR WINAPI DXUTMonitorFromRect( LPCRECT lprcScreenCoords, DWORD dwFlags )
{
    static bool s_bInited = false;
    static LPMONITORFROMRECT s_pFnGetMonitorFromRect = NULL;
    if( !s_bInited )
    {
        s_bInited = true;
        HMODULE hUser32 = GetModuleHandle( L"USER32" );
        if( hUser32 ) s_pFnGetMonitorFromRect = ( LPMONITORFROMRECT )GetProcAddress( hUser32, "MonitorFromRect" );
    }

    if( s_pFnGetMonitorFromRect )
        return s_pFnGetMonitorFromRect( lprcScreenCoords, dwFlags );
    else
        return DXUT_PRIMARY_MONITOR;
}


//--------------------------------------------------------------------------------------
// Get the desktop resolution of an adapter. This isn't the same as the current resolution 
// from GetAdapterDisplayMode since the device might be fullscreen 
//--------------------------------------------------------------------------------------
void WINAPI DXUTGetDesktopResolution( UINT AdapterOrdinal, UINT* pWidth, UINT* pHeight )
{
    DXUTDeviceSettings DeviceSettings = DXUTGetDeviceSettings();

    WCHAR strDeviceName[256] = {0};
    DEVMODE devMode;
    ZeroMemory( &devMode, sizeof( DEVMODE ) );
    devMode.dmSize = sizeof( DEVMODE );
    if( DeviceSettings.ver == DXUT_D3D9_DEVICE )
    {
        CD3D9Enumeration* pd3dEnum = DXUTGetD3D9Enumeration();
        CD3D9EnumAdapterInfo* pAdapterInfo = pd3dEnum->GetAdapterInfo( AdapterOrdinal );
        if( pAdapterInfo )
        {
            MultiByteToWideChar( CP_ACP, 0, pAdapterInfo->AdapterIdentifier.DeviceName, -1, strDeviceName, 256 );
            strDeviceName[255] = 0;
        }
    }
    else
    {
        CD3D11Enumeration* pd3dEnum = DXUTGetD3D11Enumeration();
        CD3D11EnumOutputInfo* pOutputInfo = pd3dEnum->GetOutputInfo( AdapterOrdinal, DeviceSettings.d3d11.Output );
        if( pOutputInfo )
        {
            wcscpy_s( strDeviceName, 256, pOutputInfo->Desc.DeviceName );
        }
    }

    EnumDisplaySettings( strDeviceName, ENUM_REGISTRY_SETTINGS, &devMode );
    if( pWidth )
        *pWidth = devMode.dmPelsWidth;
    if( pHeight )
        *pHeight = devMode.dmPelsHeight;
}


//--------------------------------------------------------------------------------------
// Display error msg box to help debug 
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTTrace( const CHAR* strFile, DWORD dwLine, HRESULT hr,
                          const WCHAR* strMsg, bool bPopMsgBox )
{
    bool bShowMsgBoxOnError = DXUTGetShowMsgBoxOnError();
    if( bPopMsgBox && bShowMsgBoxOnError == false )
        bPopMsgBox = false;

    return DXTrace( strFile, dwLine, hr, strMsg, bPopMsgBox );
}


//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
void WINAPI DXUTConvertDeviceSettings11to9( DXUTD3D11DeviceSettings* pIn, DXUTD3D9DeviceSettings* pOut )
{
    pOut->AdapterOrdinal = pIn->AdapterOrdinal;

    if( pIn->DriverType == D3D_DRIVER_TYPE_HARDWARE )
        pOut->DeviceType = D3DDEVTYPE_HAL;
    else if( pIn->DriverType == D3D_DRIVER_TYPE_REFERENCE )
        pOut->DeviceType = D3DDEVTYPE_REF;
    else if( pIn->DriverType == D3D_DRIVER_TYPE_NULL )
        pOut->DeviceType = D3DDEVTYPE_NULLREF;

    pOut->AdapterFormat = ConvertFormatDXGIToD3D9( pIn->sd.BufferDesc.Format );
    pOut->BehaviorFlags = D3DCREATE_HARDWARE_VERTEXPROCESSING;
    pOut->pp.BackBufferWidth = pIn->sd.BufferDesc.Width;
    pOut->pp.BackBufferHeight = pIn->sd.BufferDesc.Height;
    pOut->pp.BackBufferFormat = ConvertFormatDXGIToD3D9( pIn->sd.BufferDesc.Format );
    pOut->pp.BackBufferCount = pIn->sd.BufferCount;
    pOut->pp.MultiSampleType = ( D3DMULTISAMPLE_TYPE )pIn->sd.SampleDesc.Count;
    pOut->pp.MultiSampleQuality = pIn->sd.SampleDesc.Quality;
    pOut->pp.SwapEffect = D3DSWAPEFFECT_DISCARD;
    pOut->pp.hDeviceWindow = pIn->sd.OutputWindow;
    pOut->pp.Windowed = pIn->sd.Windowed;
    pOut->pp.EnableAutoDepthStencil = true;
    pOut->pp.AutoDepthStencilFormat = D3DFMT_D24FS8;
    pOut->pp.Flags = 0;
    if( pIn->sd.BufferDesc.RefreshRate.Denominator == 0 )
        pOut->pp.FullScreen_RefreshRateInHz = 60;
    else
        pOut->pp.FullScreen_RefreshRateInHz = pIn->sd.BufferDesc.RefreshRate.Numerator /
            pIn->sd.BufferDesc.RefreshRate.Denominator;

    switch( pIn->SyncInterval )
    {
        case 0:
            pOut->pp.PresentationInterval = D3DPRESENT_INTERVAL_IMMEDIATE; break;
        case 2:
            pOut->pp.PresentationInterval = D3DPRESENT_INTERVAL_TWO; break;
        case 3:
            pOut->pp.PresentationInterval = D3DPRESENT_INTERVAL_THREE; break;
        case 4:
            pOut->pp.PresentationInterval = D3DPRESENT_INTERVAL_FOUR; break;

        case 1:
        default:
            pOut->pp.PresentationInterval = D3DPRESENT_INTERVAL_DEFAULT;
            break;
    }
}


//--------------------------------------------------------------------------------------
void WINAPI DXUTConvertDeviceSettings9to11( DXUTD3D9DeviceSettings* pIn, DXUTD3D11DeviceSettings* pOut )
{
    pOut->AdapterOrdinal = pIn->AdapterOrdinal;

    if( pIn->DeviceType == D3DDEVTYPE_HAL )
        pOut->DriverType = D3D_DRIVER_TYPE_HARDWARE;
    else if( pIn->DeviceType == D3DDEVTYPE_REF )
        pOut->DriverType = D3D_DRIVER_TYPE_REFERENCE;
    else if( pIn->DeviceType == D3DDEVTYPE_NULLREF )
        pOut->DriverType = D3D_DRIVER_TYPE_NULL;

    pOut->Output = 0;

    pOut->sd.BufferDesc.Width = pIn->pp.BackBufferWidth;
    pOut->sd.BufferDesc.Height = pIn->pp.BackBufferHeight;
    pOut->sd.BufferDesc.RefreshRate.Numerator = pIn->pp.FullScreen_RefreshRateInHz;
    pOut->sd.BufferDesc.RefreshRate.Denominator = 1;
    pOut->sd.BufferDesc.Format = ConvertFormatD3D9ToDXGI( pIn->pp.BackBufferFormat );

    if( pIn->pp.MultiSampleType == D3DMULTISAMPLE_NONMASKABLE )
    {
        pOut->sd.SampleDesc.Count = pIn->pp.MultiSampleQuality;
        pOut->sd.SampleDesc.Quality = 0;
    }
    else
    {
        pOut->sd.SampleDesc.Count = pIn->pp.MultiSampleType;
        pOut->sd.SampleDesc.Quality = pIn->pp.MultiSampleQuality;
    }

    pOut->sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    pOut->sd.BufferCount = pIn->pp.BackBufferCount;
    pOut->sd.OutputWindow = pIn->pp.hDeviceWindow;
    pOut->sd.Windowed = pIn->pp.Windowed;

#if defined(DEBUG) || defined(_DEBUG)
    pOut->CreateFlags = D3D11_CREATE_DEVICE_DEBUG;
#else
    pOut->CreateFlags = 0;
#endif

    switch( pIn->pp.PresentationInterval )
    {
        case D3DPRESENT_INTERVAL_IMMEDIATE:
            pOut->SyncInterval = 0; break;
        case D3DPRESENT_INTERVAL_ONE:
            pOut->SyncInterval = 1; break;
        case D3DPRESENT_INTERVAL_TWO:
            pOut->SyncInterval = 2; break;
        case D3DPRESENT_INTERVAL_THREE:
            pOut->SyncInterval = 3; break;
        case D3DPRESENT_INTERVAL_FOUR:
            pOut->SyncInterval = 4; break;

        case D3DPRESENT_INTERVAL_DEFAULT:
        default:
            pOut->SyncInterval = 1;
            break;
    }

    pOut->PresentFlags = 0;
}



DXGI_FORMAT WINAPI ConvertFormatD3D9ToDXGI( D3DFORMAT fmt )
{
    switch( fmt )
    {
        case D3DFMT_UNKNOWN:
            return DXGI_FORMAT_UNKNOWN;
        case D3DFMT_R8G8B8:
        case D3DFMT_A8R8G8B8:
        case D3DFMT_X8R8G8B8:
            return DXGI_FORMAT_R8G8B8A8_UNORM;
        case D3DFMT_R5G6B5:
            return DXGI_FORMAT_B5G6R5_UNORM;
        case D3DFMT_X1R5G5B5:
        case D3DFMT_A1R5G5B5:
            return DXGI_FORMAT_B5G5R5A1_UNORM;
        case D3DFMT_A4R4G4B4:
            return DXGI_FORMAT_R8G8B8A8_UNORM;
        case D3DFMT_R3G3B2:
            return DXGI_FORMAT_R8G8B8A8_UNORM;
        case D3DFMT_A8:
            return DXGI_FORMAT_A8_UNORM;
        case D3DFMT_A8R3G3B2:
            return DXGI_FORMAT_R8G8B8A8_UNORM;
        case D3DFMT_X4R4G4B4:
            return DXGI_FORMAT_R8G8B8A8_UNORM;
        case D3DFMT_A2B10G10R10:
            return DXGI_FORMAT_R10G10B10A2_UNORM;
        case D3DFMT_A8B8G8R8:
        case D3DFMT_X8B8G8R8:
            return DXGI_FORMAT_B8G8R8A8_UNORM;
        case D3DFMT_G16R16:
            return DXGI_FORMAT_R16G16_UNORM;
        case D3DFMT_A2R10G10B10:
            return DXGI_FORMAT_R10G10B10A2_UNORM;
        case D3DFMT_A16B16G16R16:
            return DXGI_FORMAT_R16G16B16A16_UNORM;
        case D3DFMT_R16F:
            return DXGI_FORMAT_R16_FLOAT;
        case D3DFMT_G16R16F:
            return DXGI_FORMAT_R16G16_FLOAT;
        case D3DFMT_A16B16G16R16F:
            return DXGI_FORMAT_R16G16B16A16_FLOAT;
        case D3DFMT_R32F:
            return DXGI_FORMAT_R32_FLOAT;
        case D3DFMT_G32R32F:
            return DXGI_FORMAT_R32G32_FLOAT;
        case D3DFMT_A32B32G32R32F:
            return DXGI_FORMAT_R32G32B32A32_FLOAT;
    }
    return DXGI_FORMAT_UNKNOWN;
}


D3DFORMAT WINAPI ConvertFormatDXGIToD3D9( DXGI_FORMAT fmt )
{
    switch( fmt )
    {
        case DXGI_FORMAT_UNKNOWN:
            return D3DFMT_UNKNOWN;
        case DXGI_FORMAT_R8G8B8A8_UNORM:
            return D3DFMT_A8R8G8B8;
        case DXGI_FORMAT_B5G6R5_UNORM:
            return D3DFMT_R5G6B5;
        case DXGI_FORMAT_B5G5R5A1_UNORM:
            return D3DFMT_A1R5G5B5;
        case DXGI_FORMAT_A8_UNORM:
            return D3DFMT_A8;
        case DXGI_FORMAT_R10G10B10A2_UNORM:
            return D3DFMT_A2B10G10R10;
        case DXGI_FORMAT_B8G8R8A8_UNORM:
            return D3DFMT_A8B8G8R8;
        case DXGI_FORMAT_R16G16_UNORM:
            return D3DFMT_G16R16;
        case DXGI_FORMAT_R16G16B16A16_UNORM:
            return D3DFMT_A16B16G16R16;
        case DXGI_FORMAT_R16_FLOAT:
            return D3DFMT_R16F;
        case DXGI_FORMAT_R16G16_FLOAT:
            return D3DFMT_G16R16F;
        case DXGI_FORMAT_R16G16B16A16_FLOAT:
            return D3DFMT_A16B16G16R16F;
        case DXGI_FORMAT_R32_FLOAT:
            return D3DFMT_R32F;
        case DXGI_FORMAT_R32G32_FLOAT:
            return D3DFMT_G32R32F;
        case DXGI_FORMAT_R32G32B32A32_FLOAT:
            return D3DFMT_A32B32G32R32F;
    }
    return D3DFMT_UNKNOWN;
}

//--------------------------------------------------------------------------------------
IDirect3DDevice9* WINAPI DXUTCreateRefDevice9( HWND hWnd, bool bNullRef )
{
    HRESULT hr;
    IDirect3D9* pD3D = DXUT_Dynamic_Direct3DCreate9( D3D_SDK_VERSION );
    if( NULL == pD3D )
        return NULL;

    D3DDISPLAYMODE Mode;
    pD3D->GetAdapterDisplayMode( 0, &Mode );

    D3DPRESENT_PARAMETERS pp;
    ZeroMemory( &pp, sizeof( D3DPRESENT_PARAMETERS ) );
    pp.BackBufferWidth = 1;
    pp.BackBufferHeight = 1;
    pp.BackBufferFormat = Mode.Format;
    pp.BackBufferCount = 1;
    pp.SwapEffect = D3DSWAPEFFECT_COPY;
    pp.Windowed = TRUE;
    pp.hDeviceWindow = hWnd;

    IDirect3DDevice9* pd3dDevice = NULL;
    hr = pD3D->CreateDevice( D3DADAPTER_DEFAULT, bNullRef ? D3DDEVTYPE_NULLREF : D3DDEVTYPE_REF,
                             hWnd, D3DCREATE_HARDWARE_VERTEXPROCESSING, &pp, &pd3dDevice );

    SAFE_RELEASE( pD3D );
    return pd3dDevice;
}


//--------------------------------------------------------------------------------------
// Helper function to launch the Media Center UI after the program terminates
//--------------------------------------------------------------------------------------
bool DXUTReLaunchMediaCenter()
{
    // Get the path to Media Center
    WCHAR szExpandedPath[MAX_PATH];
    if( !ExpandEnvironmentStrings( L"%SystemRoot%\\ehome\\ehshell.exe", szExpandedPath, MAX_PATH ) )
        return false;

    // Skip if ehshell.exe doesn't exist
    if( GetFileAttributes( szExpandedPath ) == 0xFFFFFFFF )
        return false;

    // Launch ehshell.exe 
    INT_PTR result = ( INT_PTR )ShellExecute( NULL, TEXT( "open" ), szExpandedPath, NULL, NULL, SW_SHOWNORMAL );
    return ( result > 32 );
}

typedef DWORD ( WINAPI* LPXINPUTGETSTATE )( DWORD dwUserIndex, XINPUT_STATE* pState );
typedef DWORD ( WINAPI* LPXINPUTSETSTATE )( DWORD dwUserIndex, XINPUT_VIBRATION* pVibration );
typedef DWORD ( WINAPI* LPXINPUTGETCAPABILITIES )( DWORD dwUserIndex, DWORD dwFlags,
                                                   XINPUT_CAPABILITIES* pCapabilities );
typedef void ( WINAPI* LPXINPUTENABLE )( BOOL bEnable );

//--------------------------------------------------------------------------------------
// Does extra processing on XInput data to make it slightly more convenient to use
//--------------------------------------------------------------------------------------
HRESULT DXUTGetGamepadState( DWORD dwPort, DXUT_GAMEPAD* pGamePad, bool bThumbstickDeadZone,
                             bool bSnapThumbstickToCardinals )
{
    if( dwPort >= DXUT_MAX_CONTROLLERS || pGamePad == NULL )
        return E_FAIL;

    static LPXINPUTGETSTATE s_pXInputGetState = NULL;
    static LPXINPUTGETCAPABILITIES s_pXInputGetCapabilities = NULL;
    if( NULL == s_pXInputGetState || NULL == s_pXInputGetCapabilities )
    {
        HINSTANCE hInst = LoadLibrary( XINPUT_DLL );
        if( hInst )
        {
            s_pXInputGetState = ( LPXINPUTGETSTATE )GetProcAddress( hInst, "XInputGetState" );
            s_pXInputGetCapabilities = ( LPXINPUTGETCAPABILITIES )GetProcAddress( hInst, "XInputGetCapabilities" );
        }
    }
    if( s_pXInputGetState == NULL )
        return E_FAIL;

    XINPUT_STATE InputState;
    DWORD dwResult = s_pXInputGetState( dwPort, &InputState );

    // Track insertion and removals
    BOOL bWasConnected = pGamePad->bConnected;
    pGamePad->bConnected = ( dwResult == ERROR_SUCCESS );
    pGamePad->bRemoved = ( bWasConnected && !pGamePad->bConnected );
    pGamePad->bInserted = ( !bWasConnected && pGamePad->bConnected );

    // Don't update rest of the state if not connected
    if( !pGamePad->bConnected )
        return S_OK;

    // Store the capabilities of the device
    if( pGamePad->bInserted )
    {
        ZeroMemory( pGamePad, sizeof( DXUT_GAMEPAD ) );
        pGamePad->bConnected = true;
        pGamePad->bInserted = true;
        if( s_pXInputGetCapabilities )
            s_pXInputGetCapabilities( dwPort, XINPUT_DEVTYPE_GAMEPAD, &pGamePad->caps );
    }

    // Copy gamepad to local structure (assumes that XINPUT_GAMEPAD at the front in CONTROLER_STATE)
    memcpy( pGamePad, &InputState.Gamepad, sizeof( XINPUT_GAMEPAD ) );

    if( bSnapThumbstickToCardinals )
    {
        // Apply deadzone to each axis independantly to slightly snap to up/down/left/right
        if( pGamePad->sThumbLX < XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE &&
            pGamePad->sThumbLX > -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE )
            pGamePad->sThumbLX = 0;
        if( pGamePad->sThumbLY < XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE &&
            pGamePad->sThumbLY > -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE )
            pGamePad->sThumbLY = 0;
        if( pGamePad->sThumbRX < XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE &&
            pGamePad->sThumbRX > -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE )
            pGamePad->sThumbRX = 0;
        if( pGamePad->sThumbRY < XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE &&
            pGamePad->sThumbRY > -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE )
            pGamePad->sThumbRY = 0;
    }
    else if( bThumbstickDeadZone )
    {
        // Apply deadzone if centered
        if( ( pGamePad->sThumbLX < XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE &&
              pGamePad->sThumbLX > -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE ) &&
            ( pGamePad->sThumbLY < XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE &&
              pGamePad->sThumbLY > -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE ) )
        {
            pGamePad->sThumbLX = 0;
            pGamePad->sThumbLY = 0;
        }
        if( ( pGamePad->sThumbRX < XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE &&
              pGamePad->sThumbRX > -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE ) &&
            ( pGamePad->sThumbRY < XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE &&
              pGamePad->sThumbRY > -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE ) )
        {
            pGamePad->sThumbRX = 0;
            pGamePad->sThumbRY = 0;
        }
    }

    // Convert [-1,+1] range
    pGamePad->fThumbLX = pGamePad->sThumbLX / 32767.0f;
    pGamePad->fThumbLY = pGamePad->sThumbLY / 32767.0f;
    pGamePad->fThumbRX = pGamePad->sThumbRX / 32767.0f;
    pGamePad->fThumbRY = pGamePad->sThumbRY / 32767.0f;

    // Get the boolean buttons that have been pressed since the last call. 
    // Each button is represented by one bit.
    pGamePad->wPressedButtons = ( pGamePad->wLastButtons ^ pGamePad->wButtons ) & pGamePad->wButtons;
    pGamePad->wLastButtons = pGamePad->wButtons;

    // Figure out if the left trigger has been pressed or released
    bool bPressed = ( pGamePad->bLeftTrigger > DXUT_GAMEPAD_TRIGGER_THRESHOLD );
    pGamePad->bPressedLeftTrigger = ( bPressed ) ? !pGamePad->bLastLeftTrigger : false;
    pGamePad->bLastLeftTrigger = bPressed;

    // Figure out if the right trigger has been pressed or released
    bPressed = ( pGamePad->bRightTrigger > DXUT_GAMEPAD_TRIGGER_THRESHOLD );
    pGamePad->bPressedRightTrigger = ( bPressed ) ? !pGamePad->bLastRightTrigger : false;
    pGamePad->bLastRightTrigger = bPressed;

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Don't pause the game or deactive the window without first stopping rumble otherwise 
// the controller will continue to rumble
//--------------------------------------------------------------------------------------
void DXUTEnableXInput( bool bEnable )
{
    static LPXINPUTENABLE s_pXInputEnable = NULL;
    if( NULL == s_pXInputEnable )
    {
        HINSTANCE hInst = LoadLibrary( XINPUT_DLL );
        if( hInst )
            s_pXInputEnable = ( LPXINPUTENABLE )GetProcAddress( hInst, "XInputEnable" );
    }

    if( s_pXInputEnable )
        s_pXInputEnable( bEnable );
}


//--------------------------------------------------------------------------------------
// Don't pause the game or deactive the window without first stopping rumble otherwise 
// the controller will continue to rumble
//--------------------------------------------------------------------------------------
HRESULT DXUTStopRumbleOnAllControllers()
{
    static LPXINPUTSETSTATE s_pXInputSetState = NULL;
    if( NULL == s_pXInputSetState )
    {
        HINSTANCE hInst = LoadLibrary( XINPUT_DLL );
        if( hInst )
            s_pXInputSetState = ( LPXINPUTSETSTATE )GetProcAddress( hInst, "XInputSetState" );
    }
    if( s_pXInputSetState == NULL )
        return E_FAIL;

    XINPUT_VIBRATION vibration;
    vibration.wLeftMotorSpeed = 0;
    vibration.wRightMotorSpeed = 0;
    for( int iUserIndex = 0; iUserIndex < DXUT_MAX_CONTROLLERS; iUserIndex++ )
        s_pXInputSetState( iUserIndex, &vibration );

    return S_OK;
}

//--------------------------------------------------------------------------------------
// Helper functions to create SRGB formats from typeless formats and vice versa
//--------------------------------------------------------------------------------------
DXGI_FORMAT MAKE_SRGB( DXGI_FORMAT format )
{
    if( !DXUTIsInGammaCorrectMode() )
        return format;

    switch( format )
    {
        case DXGI_FORMAT_R8G8B8A8_TYPELESS:
        case DXGI_FORMAT_R8G8B8A8_UNORM:
        case DXGI_FORMAT_R8G8B8A8_UINT:
        case DXGI_FORMAT_R8G8B8A8_SNORM:
        case DXGI_FORMAT_R8G8B8A8_SINT:
            return DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;

        case DXGI_FORMAT_BC1_TYPELESS:
        case DXGI_FORMAT_BC1_UNORM:
            return DXGI_FORMAT_BC1_UNORM_SRGB;
        case DXGI_FORMAT_BC2_TYPELESS:
        case DXGI_FORMAT_BC2_UNORM:
            return DXGI_FORMAT_BC2_UNORM_SRGB;
        case DXGI_FORMAT_BC3_TYPELESS:
        case DXGI_FORMAT_BC3_UNORM:
            return DXGI_FORMAT_BC3_UNORM_SRGB;

    };

    return format;
}

//--------------------------------------------------------------------------------------
DXGI_FORMAT MAKE_TYPELESS( DXGI_FORMAT format )
{
    if( !DXUTIsInGammaCorrectMode() )
        return format;

    switch( format )
    {
        case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
        case DXGI_FORMAT_R8G8B8A8_UNORM:
        case DXGI_FORMAT_R8G8B8A8_UINT:
        case DXGI_FORMAT_R8G8B8A8_SNORM:
        case DXGI_FORMAT_R8G8B8A8_SINT:
            return DXGI_FORMAT_R8G8B8A8_TYPELESS;

        case DXGI_FORMAT_BC1_UNORM_SRGB:
        case DXGI_FORMAT_BC1_UNORM:
            return DXGI_FORMAT_BC1_TYPELESS;
        case DXGI_FORMAT_BC2_UNORM_SRGB:
        case DXGI_FORMAT_BC2_UNORM:
            return DXGI_FORMAT_BC2_TYPELESS;
        case DXGI_FORMAT_BC3_UNORM_SRGB:
        case DXGI_FORMAT_BC3_UNORM:
            return DXGI_FORMAT_BC3_TYPELESS;
    };

    return format;
}

//--------------------------------------------------------------------------------------
HRESULT DXUTSnapD3D9Screenshot( LPCTSTR szFileName )
{
    HRESULT hr = S_OK;
    IDirect3DDevice9* pDev = DXUTGetD3D9Device();
    if( !pDev )
        return E_FAIL;

    IDirect3DSurface9* pBackBuffer = NULL;
    V_RETURN( pDev->GetBackBuffer( 0, 0, D3DBACKBUFFER_TYPE_MONO, &pBackBuffer ) );

    return D3DXSaveSurfaceToFile( szFileName, D3DXIFF_BMP, pBackBuffer, NULL, NULL );
}




//-------------------------------------------------------------------------------------- 
HRESULT DXUTSnapD3D11Screenshot( LPCTSTR szFileName, D3DX11_IMAGE_FILE_FORMAT iff )
{
    IDXGISwapChain *pSwap = DXUTGetDXGISwapChain();

    if (!pSwap)
        return E_FAIL;
    
    ID3D11Texture2D* pBackBuffer;
    HRESULT hr = pSwap->GetBuffer( 0, __uuidof( *pBackBuffer ), ( LPVOID* )&pBackBuffer );
    if (hr != S_OK)
        return hr;

    ID3D11DeviceContext *dc  = DXUTGetD3D11DeviceContext();
    if (!dc) {
        SAFE_RELEASE(pBackBuffer);
        return E_FAIL;
    }
    ID3D11Device *pDevice = DXUTGetD3D11Device();
    if (!dc) {
        SAFE_RELEASE(pBackBuffer);
        return E_FAIL;
    }

    D3D11_TEXTURE2D_DESC dsc;
    pBackBuffer->GetDesc(&dsc);
    D3D11_RESOURCE_DIMENSION dim;
    pBackBuffer->GetType(&dim);
    // special case msaa textures
    ID3D11Texture2D *pCompatableTexture = pBackBuffer;
    if ( dsc.SampleDesc.Count > 1) {
        D3D11_TEXTURE2D_DESC dsc_new = dsc;
        dsc_new.SampleDesc.Count = 1;
        dsc_new.SampleDesc.Quality = 0;
        dsc_new.Usage = D3D11_USAGE_DEFAULT;
        dsc_new.BindFlags = 0;
        dsc_new.CPUAccessFlags = 0;
        ID3D11Texture2D *resolveTexture;
        hr = pDevice->CreateTexture2D(&dsc_new, NULL, &resolveTexture);
        dc->ResolveSubresource(resolveTexture, 0, pBackBuffer, 0, dsc.Format);
        pCompatableTexture = resolveTexture;
        pCompatableTexture->GetDesc(&dsc);
    }

    hr = D3DX11SaveTextureToFileW(dc, pCompatableTexture, iff, szFileName); 
    
    
    SAFE_RELEASE(pBackBuffer);
    SAFE_RELEASE(pCompatableTexture);

    return hr;

}
