//--------------------------------------------------------------------------------------
// File: DXUTDevice9.cpp
//
// Enumerates D3D adapters, devices, modes, etc.
//
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------
#include "DXUT.h"
#undef min // use __min instead
#undef max // use __max instead

//--------------------------------------------------------------------------------------
// Forward declarations
//--------------------------------------------------------------------------------------
extern void DXUTGetCallbackD3D9DeviceAcceptable( LPDXUTCALLBACKISD3D9DEVICEACCEPTABLE* ppCallbackIsDeviceAcceptable, void** ppUserContext );




static int __cdecl SortModesCallback( const void* arg1, const void* arg2 );


CD3D9Enumeration*   g_pDXUTD3D9Enumeration = NULL;

HRESULT WINAPI DXUTCreateD3D9Enumeration()
{
    if( g_pDXUTD3D9Enumeration == NULL )
    {
        g_pDXUTD3D9Enumeration = new CD3D9Enumeration();
        if( NULL == g_pDXUTD3D9Enumeration )
            return E_OUTOFMEMORY;
    }
    return S_OK;
}

void WINAPI DXUTDestroyD3D9Enumeration()
{
    SAFE_DELETE( g_pDXUTD3D9Enumeration );
}

class DXUTMemoryHelperD3D9Enum
{
public:
DXUTMemoryHelperD3D9Enum()
{
    DXUTCreateD3D9Enumeration();
}
~DXUTMemoryHelperD3D9Enum()
{
    DXUTDestroyD3D9Enumeration();
}
};

//--------------------------------------------------------------------------------------
CD3D9Enumeration* WINAPI DXUTGetD3D9Enumeration( bool bForceEnumerate )
{
    // Using an static class with accessor function to allow control of the construction order
    static DXUTMemoryHelperD3D9Enum d3d9enumMemory;

    if( g_pDXUTD3D9Enumeration && ( !g_pDXUTD3D9Enumeration->HasEnumerated() || bForceEnumerate ) )
    {
        LPDXUTCALLBACKISD3D9DEVICEACCEPTABLE pCallbackIsDeviceAcceptable;
        void* pUserContext;
        DXUTGetCallbackD3D9DeviceAcceptable( &pCallbackIsDeviceAcceptable, &pUserContext );
        g_pDXUTD3D9Enumeration->Enumerate( pCallbackIsDeviceAcceptable, pUserContext );
    }

    return g_pDXUTD3D9Enumeration;
}


//--------------------------------------------------------------------------------------
CD3D9Enumeration::CD3D9Enumeration()
{
    m_bHasEnumerated = false;
    m_pD3D = NULL;
    m_IsD3D9DeviceAcceptableFunc = NULL;
    m_pIsD3D9DeviceAcceptableFuncUserContext = NULL;
    m_bRequirePostPixelShaderBlending = true;

    m_nMinWidth = 640;
    m_nMinHeight = 480;
    m_nMaxWidth = UINT_MAX;
    m_nMaxHeight = UINT_MAX;

    m_nRefreshMin = 0;
    m_nRefreshMax = UINT_MAX;

    m_nMultisampleQualityMax = 0xFFFF;

    ResetPossibleDepthStencilFormats();
    ResetPossibleMultisampleTypeList();
    ResetPossiblePresentIntervalList();
    SetPossibleVertexProcessingList( true, true, true, false );
}


//--------------------------------------------------------------------------------------
CD3D9Enumeration::~CD3D9Enumeration()
{
    ClearAdapterInfoList();
}



//--------------------------------------------------------------------------------------
// Enumerate for each adapter all of the supported display modes, 
// device types, adapter formats, back buffer formats, window/full screen support, 
// depth stencil formats, multisampling types/qualities, and presentations intervals.
//
// For each combination of device type (HAL/REF), adapter format, back buffer format, and
// IsWindowed it will call the app's ConfirmDevice callback.  This allows the app
// to reject or allow that combination based on its caps/etc.  It also allows the 
// app to change the BehaviorFlags.  The BehaviorFlags defaults non-pure HWVP 
// if supported otherwise it will default to SWVP, however the app can change this 
// through the ConfirmDevice callback.
//--------------------------------------------------------------------------------------
HRESULT CD3D9Enumeration::Enumerate( LPDXUTCALLBACKISD3D9DEVICEACCEPTABLE IsD3D9DeviceAcceptableFunc,
                                     void* pIsD3D9DeviceAcceptableFuncUserContext )
{
    CDXUTPerfEventGenerator eventGenerator( DXUT_PERFEVENTCOLOR, L"DXUT D3D9 Enumeration" );
    IDirect3D9* pD3D = DXUTGetD3D9Object();
    if( pD3D == NULL )
    {
        pD3D = DXUTGetD3D9Object();
        if( pD3D == NULL )
            return DXUTERR_NODIRECT3D;
    }

    m_bHasEnumerated = true;
    m_pD3D = pD3D;
    m_IsD3D9DeviceAcceptableFunc = IsD3D9DeviceAcceptableFunc;
    m_pIsD3D9DeviceAcceptableFuncUserContext = pIsD3D9DeviceAcceptableFuncUserContext;

    HRESULT hr;
    ClearAdapterInfoList();
    CGrowableArray <D3DFORMAT> adapterFormatList;

    const D3DFORMAT allowedAdapterFormatArray[] =
    {
        D3DFMT_X8R8G8B8,
        D3DFMT_X1R5G5B5,
        D3DFMT_R5G6B5,
        D3DFMT_A2R10G10B10
    };
    const UINT allowedAdapterFormatArrayCount = sizeof( allowedAdapterFormatArray ) / sizeof
        ( allowedAdapterFormatArray[0] );

    UINT numAdapters = pD3D->GetAdapterCount();
    for( UINT adapterOrdinal = 0; adapterOrdinal < numAdapters; adapterOrdinal++ )
    {
        CD3D9EnumAdapterInfo* pAdapterInfo = new CD3D9EnumAdapterInfo;
        if( pAdapterInfo == NULL )
            return E_OUTOFMEMORY;

        pAdapterInfo->AdapterOrdinal = adapterOrdinal;
        pD3D->GetAdapterIdentifier( adapterOrdinal, 0, &pAdapterInfo->AdapterIdentifier );

        // Get list of all display modes on this adapter.  
        // Also build a temporary list of all display adapter formats.
        adapterFormatList.RemoveAll();

        for( UINT iFormatList = 0; iFormatList < allowedAdapterFormatArrayCount; iFormatList++ )
        {
            D3DFORMAT allowedAdapterFormat = allowedAdapterFormatArray[iFormatList];
            UINT numAdapterModes = pD3D->GetAdapterModeCount( adapterOrdinal, allowedAdapterFormat );
            for( UINT mode = 0; mode < numAdapterModes; mode++ )
            {
                D3DDISPLAYMODE displayMode;
                pD3D->EnumAdapterModes( adapterOrdinal, allowedAdapterFormat, mode, &displayMode );

                if( displayMode.Width < m_nMinWidth ||
                    displayMode.Height < m_nMinHeight ||
                    displayMode.Width > m_nMaxWidth ||
                    displayMode.Height > m_nMaxHeight ||
                    displayMode.RefreshRate < m_nRefreshMin ||
                    displayMode.RefreshRate > m_nRefreshMax )
                {
                    continue;
                }

                pAdapterInfo->displayModeList.Add( displayMode );

                if( !adapterFormatList.Contains( displayMode.Format ) )
                    adapterFormatList.Add( displayMode.Format );
            }

        }

        D3DDISPLAYMODE displayMode;
        pD3D->GetAdapterDisplayMode( adapterOrdinal, &displayMode );
        if( !adapterFormatList.Contains( displayMode.Format ) )
            adapterFormatList.Add( displayMode.Format );

        // Sort displaymode list
        qsort( pAdapterInfo->displayModeList.GetData(),
               pAdapterInfo->displayModeList.GetSize(), sizeof( D3DDISPLAYMODE ),
               SortModesCallback );

        // Get info for each device on this adapter
        if( FAILED( EnumerateDevices( pAdapterInfo, &adapterFormatList ) ) )
        {
            delete pAdapterInfo;
            continue;
        }

        // If at least one device on this adapter is available and compatible
        // with the app, add the adapterInfo to the list
        if( pAdapterInfo->deviceInfoList.GetSize() > 0 )
        {
            hr = m_AdapterInfoList.Add( pAdapterInfo );
            if( FAILED( hr ) )
                return hr;
        }
        else
            delete pAdapterInfo;
    }

    //
    // Check for 2 or more adapters with the same name. Append the name
    // with some instance number if that's the case to help distinguish
    // them.
    //
    bool bUniqueDesc = true;
    CD3D9EnumAdapterInfo* pAdapterInfo;
    for( int i = 0; i < m_AdapterInfoList.GetSize(); i++ )
    {
        CD3D9EnumAdapterInfo* pAdapterInfo1 = m_AdapterInfoList.GetAt( i );

        for( int j = i + 1; j < m_AdapterInfoList.GetSize(); j++ )
        {
            CD3D9EnumAdapterInfo* pAdapterInfo2 = m_AdapterInfoList.GetAt( j );
            if( _stricmp( pAdapterInfo1->AdapterIdentifier.Description,
                          pAdapterInfo2->AdapterIdentifier.Description ) == 0 )
            {
                bUniqueDesc = false;
                break;
            }
        }

        if( !bUniqueDesc )
            break;
    }

    for( int i = 0; i < m_AdapterInfoList.GetSize(); i++ )
    {
        pAdapterInfo = m_AdapterInfoList.GetAt( i );

        MultiByteToWideChar( CP_ACP, 0,
                             pAdapterInfo->AdapterIdentifier.Description, -1,
                             pAdapterInfo->szUniqueDescription, 100 );
        pAdapterInfo->szUniqueDescription[100] = 0;

        if( !bUniqueDesc )
        {
            WCHAR sz[100];
            swprintf_s( sz, 100, L" (#%d)", pAdapterInfo->AdapterOrdinal );
            wcscat_s( pAdapterInfo->szUniqueDescription, 256, sz );

        }
    }

    return S_OK;
}



//--------------------------------------------------------------------------------------
// Enumerates D3D devices for a particular adapter.
//--------------------------------------------------------------------------------------
HRESULT CD3D9Enumeration::EnumerateDevices( CD3D9EnumAdapterInfo* pAdapterInfo,
                                            CGrowableArray <D3DFORMAT>* pAdapterFormatList )
{
    HRESULT hr;

    const D3DDEVTYPE devTypeArray[] =
    {
        D3DDEVTYPE_HAL,
        D3DDEVTYPE_SW,
        D3DDEVTYPE_REF
    };
    const UINT devTypeArrayCount = sizeof( devTypeArray ) / sizeof( devTypeArray[0] );

    // Enumerate each Direct3D device type
    for( UINT iDeviceType = 0; iDeviceType < devTypeArrayCount; iDeviceType++ )
    {
        CD3D9EnumDeviceInfo* pDeviceInfo = new CD3D9EnumDeviceInfo;
        if( pDeviceInfo == NULL )
            return E_OUTOFMEMORY;

        // Fill struct w/ AdapterOrdinal and D3DDEVTYPE
        pDeviceInfo->AdapterOrdinal = pAdapterInfo->AdapterOrdinal;
        pDeviceInfo->DeviceType = devTypeArray[iDeviceType];

        // Store device caps
        if( FAILED( hr = m_pD3D->GetDeviceCaps( pAdapterInfo->AdapterOrdinal, pDeviceInfo->DeviceType,
                                                &pDeviceInfo->Caps ) ) )
        {
            delete pDeviceInfo;
            continue;
        }

        if( pDeviceInfo->DeviceType != D3DDEVTYPE_HAL )
        {
            // Create a temp device to verify that it is really possible to create a REF device 
            // [the developer DirectX redist has to be installed]
            D3DDISPLAYMODE Mode;
            m_pD3D->GetAdapterDisplayMode( 0, &Mode );
            D3DPRESENT_PARAMETERS pp;
            ZeroMemory( &pp, sizeof( D3DPRESENT_PARAMETERS ) );
            pp.BackBufferWidth = 1;
            pp.BackBufferHeight = 1;
            pp.BackBufferFormat = Mode.Format;
            pp.BackBufferCount = 1;
            pp.SwapEffect = D3DSWAPEFFECT_COPY;
            pp.Windowed = TRUE;
            pp.hDeviceWindow = DXUTGetHWNDFocus();
            IDirect3DDevice9* pDevice = NULL;
            if( FAILED( hr = m_pD3D->CreateDevice( pAdapterInfo->AdapterOrdinal, pDeviceInfo->DeviceType,
                                                   DXUTGetHWNDFocus(),
                                                   D3DCREATE_HARDWARE_VERTEXPROCESSING | D3DCREATE_FPU_PRESERVE, &pp,
                                                   &pDevice ) ) )
            {
                delete pDeviceInfo;
                continue;
            }
            SAFE_RELEASE( pDevice );
        }

        // Get info for each devicecombo on this device
        if( FAILED( hr = EnumerateDeviceCombos( pAdapterInfo, pDeviceInfo, pAdapterFormatList ) ) )
        {
            delete pDeviceInfo;
            continue;
        }

        // If at least one devicecombo for this device is found, 
        // add the deviceInfo to the list
        if( pDeviceInfo->deviceSettingsComboList.GetSize() > 0 )
            pAdapterInfo->deviceInfoList.Add( pDeviceInfo );
        else
            delete pDeviceInfo;
    }

    return S_OK;
}



//--------------------------------------------------------------------------------------
// Enumerates DeviceCombos for a particular device.
//--------------------------------------------------------------------------------------
HRESULT CD3D9Enumeration::EnumerateDeviceCombos( CD3D9EnumAdapterInfo* pAdapterInfo, CD3D9EnumDeviceInfo* pDeviceInfo,
                                                 CGrowableArray <D3DFORMAT>* pAdapterFormatList )
{
    const D3DFORMAT backBufferFormatArray[] =
    {
        D3DFMT_A8R8G8B8,
        D3DFMT_X8R8G8B8,
        D3DFMT_A2R10G10B10,
        D3DFMT_R5G6B5,
        D3DFMT_A1R5G5B5,
        D3DFMT_X1R5G5B5
    };
    const UINT backBufferFormatArrayCount = sizeof( backBufferFormatArray ) / sizeof( backBufferFormatArray[0] );

    // See which adapter formats are supported by this device
    for( int iFormat = 0; iFormat < pAdapterFormatList->GetSize(); iFormat++ )
    {
        D3DFORMAT adapterFormat = pAdapterFormatList->GetAt( iFormat );

        for( UINT iBackBufferFormat = 0; iBackBufferFormat < backBufferFormatArrayCount; iBackBufferFormat++ )
        {
            D3DFORMAT backBufferFormat = backBufferFormatArray[iBackBufferFormat];

            for( int nWindowed = 0; nWindowed < 2; nWindowed++ )
            {
                if( !nWindowed && pAdapterInfo->displayModeList.GetSize() == 0 )
                    continue;

                if( FAILED( m_pD3D->CheckDeviceType( pAdapterInfo->AdapterOrdinal, pDeviceInfo->DeviceType,
                                                     adapterFormat, backBufferFormat, nWindowed ) ) )
                {
                    continue;
                }

                if( m_bRequirePostPixelShaderBlending )
                {
                    // If the backbuffer format doesn't support D3DUSAGE_QUERY_POSTPIXELSHADER_BLENDING
                    // then alpha test, pixel fog, render-target blending, color write enable, and dithering. 
                    // are not supported.
                    if( FAILED( m_pD3D->CheckDeviceFormat( pAdapterInfo->AdapterOrdinal, pDeviceInfo->DeviceType,
                                                           adapterFormat, D3DUSAGE_QUERY_POSTPIXELSHADER_BLENDING,
                                                           D3DRTYPE_TEXTURE, backBufferFormat ) ) )
                    {
                        continue;
                    }
                }

                // If an application callback function has been provided, make sure this device
                // is acceptable to the app.
                if( m_IsD3D9DeviceAcceptableFunc != NULL )
                {
                    if( !m_IsD3D9DeviceAcceptableFunc( &pDeviceInfo->Caps, adapterFormat, backBufferFormat,
                                                       FALSE != nWindowed, m_pIsD3D9DeviceAcceptableFuncUserContext ) )
                        continue;
                }

                // At this point, we have an adapter/device/adapterformat/backbufferformat/iswindowed
                // DeviceCombo that is supported by the system and acceptable to the app. We still 
                // need to find one or more suitable depth/stencil buffer format,
                // multisample type, and present interval.
                CD3D9EnumDeviceSettingsCombo* pDeviceCombo = new CD3D9EnumDeviceSettingsCombo;
                if( pDeviceCombo == NULL )
                    return E_OUTOFMEMORY;

                pDeviceCombo->AdapterOrdinal = pAdapterInfo->AdapterOrdinal;
                pDeviceCombo->DeviceType = pDeviceInfo->DeviceType;
                pDeviceCombo->AdapterFormat = adapterFormat;
                pDeviceCombo->BackBufferFormat = backBufferFormat;
                pDeviceCombo->Windowed = ( nWindowed != 0 );

                BuildDepthStencilFormatList( pDeviceCombo );
                BuildMultiSampleTypeList( pDeviceCombo );
                if( pDeviceCombo->multiSampleTypeList.GetSize() == 0 )
                {
                    delete pDeviceCombo;
                    continue;
                }
                BuildDSMSConflictList( pDeviceCombo );
                BuildPresentIntervalList( pDeviceInfo, pDeviceCombo );
                pDeviceCombo->pAdapterInfo = pAdapterInfo;
                pDeviceCombo->pDeviceInfo = pDeviceInfo;

                if( FAILED( pDeviceInfo->deviceSettingsComboList.Add( pDeviceCombo ) ) )
                    delete pDeviceCombo;
            }
        }
    }

    return S_OK;
}



//--------------------------------------------------------------------------------------
// Adds all depth/stencil formats that are compatible with the device 
//       and app to the given D3DDeviceCombo.
//--------------------------------------------------------------------------------------
void CD3D9Enumeration::BuildDepthStencilFormatList( CD3D9EnumDeviceSettingsCombo* pDeviceCombo )
{
    D3DFORMAT depthStencilFmt;
    for( int idsf = 0; idsf < m_DepthStencilPossibleList.GetSize(); idsf++ )
    {
        depthStencilFmt = m_DepthStencilPossibleList.GetAt( idsf );
        if( SUCCEEDED( m_pD3D->CheckDeviceFormat( pDeviceCombo->AdapterOrdinal,
                                                  pDeviceCombo->DeviceType, pDeviceCombo->AdapterFormat,
                                                  D3DUSAGE_DEPTHSTENCIL, D3DRTYPE_SURFACE, depthStencilFmt ) ) )
        {
            if( SUCCEEDED( m_pD3D->CheckDepthStencilMatch( pDeviceCombo->AdapterOrdinal,
                                                           pDeviceCombo->DeviceType, pDeviceCombo->AdapterFormat,
                                                           pDeviceCombo->BackBufferFormat, depthStencilFmt ) ) )
            {
                pDeviceCombo->depthStencilFormatList.Add( depthStencilFmt );
            }
        }
    }
}




//--------------------------------------------------------------------------------------
// Adds all multisample types that are compatible with the device and app to
//       the given D3DDeviceCombo.
//--------------------------------------------------------------------------------------
void CD3D9Enumeration::BuildMultiSampleTypeList( CD3D9EnumDeviceSettingsCombo* pDeviceCombo )
{
    D3DMULTISAMPLE_TYPE msType;
    DWORD msQuality;
    for( int imst = 0; imst < m_MultiSampleTypeList.GetSize(); imst++ )
    {
        msType = m_MultiSampleTypeList.GetAt( imst );
        if( SUCCEEDED( m_pD3D->CheckDeviceMultiSampleType( pDeviceCombo->AdapterOrdinal,
                                                           pDeviceCombo->DeviceType, pDeviceCombo->BackBufferFormat,
                                                           pDeviceCombo->Windowed, msType, &msQuality ) ) )
        {
            pDeviceCombo->multiSampleTypeList.Add( msType );
            if( msQuality > m_nMultisampleQualityMax + 1 )
                msQuality = m_nMultisampleQualityMax + 1;
            pDeviceCombo->multiSampleQualityList.Add( msQuality );
        }
    }
}




//--------------------------------------------------------------------------------------
// Find any conflicts between the available depth/stencil formats and
//       multisample types.
//--------------------------------------------------------------------------------------
void CD3D9Enumeration::BuildDSMSConflictList( CD3D9EnumDeviceSettingsCombo* pDeviceCombo )
{
    CD3D9EnumDSMSConflict DSMSConflict;

    for( int iDS = 0; iDS < pDeviceCombo->depthStencilFormatList.GetSize(); iDS++ )
    {
        D3DFORMAT dsFmt = pDeviceCombo->depthStencilFormatList.GetAt( iDS );

        for( int iMS = 0; iMS < pDeviceCombo->multiSampleTypeList.GetSize(); iMS++ )
        {
            D3DMULTISAMPLE_TYPE msType = pDeviceCombo->multiSampleTypeList.GetAt( iMS );

            if( FAILED( m_pD3D->CheckDeviceMultiSampleType( pDeviceCombo->AdapterOrdinal, pDeviceCombo->DeviceType,
                                                            dsFmt, pDeviceCombo->Windowed, msType, NULL ) ) )
            {
                DSMSConflict.DSFormat = dsFmt;
                DSMSConflict.MSType = msType;
                pDeviceCombo->DSMSConflictList.Add( DSMSConflict );
            }
        }
    }
}


//--------------------------------------------------------------------------------------
// Adds all present intervals that are compatible with the device and app 
//       to the given D3DDeviceCombo.
//--------------------------------------------------------------------------------------
void CD3D9Enumeration::BuildPresentIntervalList( CD3D9EnumDeviceInfo* pDeviceInfo,
                                                 CD3D9EnumDeviceSettingsCombo* pDeviceCombo )
{
    UINT pi;
    for( int ipi = 0; ipi < m_PresentIntervalList.GetSize(); ipi++ )
    {
        pi = m_PresentIntervalList.GetAt( ipi );
        if( pDeviceCombo->Windowed )
        {
            if( pi == D3DPRESENT_INTERVAL_TWO ||
                pi == D3DPRESENT_INTERVAL_THREE ||
                pi == D3DPRESENT_INTERVAL_FOUR )
            {
                // These intervals are not supported in windowed mode.
                continue;
            }
        }
        // Note that D3DPRESENT_INTERVAL_DEFAULT is zero, so you
        // can't do a caps check for it -- it is always available.
        if( pi == D3DPRESENT_INTERVAL_DEFAULT ||
            ( pDeviceInfo->Caps.PresentationIntervals & pi ) )
        {
            pDeviceCombo->presentIntervalList.Add( pi );
        }
    }
}



//--------------------------------------------------------------------------------------
// Release all the allocated CD3D9EnumAdapterInfo objects and empty the list
//--------------------------------------------------------------------------------------
void CD3D9Enumeration::ClearAdapterInfoList()
{
    CD3D9EnumAdapterInfo* pAdapterInfo;
    for( int i = 0; i < m_AdapterInfoList.GetSize(); i++ )
    {
        pAdapterInfo = m_AdapterInfoList.GetAt( i );
        delete pAdapterInfo;
    }

    m_AdapterInfoList.RemoveAll();
}



//--------------------------------------------------------------------------------------
// Call GetAdapterInfoList() after Enumerate() to get a STL vector of 
//       CD3D9EnumAdapterInfo* 
//--------------------------------------------------------------------------------------
CGrowableArray <CD3D9EnumAdapterInfo*>* CD3D9Enumeration::GetAdapterInfoList()
{
    return &m_AdapterInfoList;
}



//--------------------------------------------------------------------------------------
CD3D9EnumAdapterInfo* CD3D9Enumeration::GetAdapterInfo( UINT AdapterOrdinal )
{
    for( int iAdapter = 0; iAdapter < m_AdapterInfoList.GetSize(); iAdapter++ )
    {
        CD3D9EnumAdapterInfo* pAdapterInfo = m_AdapterInfoList.GetAt( iAdapter );
        if( pAdapterInfo->AdapterOrdinal == AdapterOrdinal )
            return pAdapterInfo;
    }

    return NULL;
}


//--------------------------------------------------------------------------------------
CD3D9EnumDeviceInfo* CD3D9Enumeration::GetDeviceInfo( UINT AdapterOrdinal, D3DDEVTYPE DeviceType )
{
    CD3D9EnumAdapterInfo* pAdapterInfo = GetAdapterInfo( AdapterOrdinal );
    if( pAdapterInfo )
    {
        for( int iDeviceInfo = 0; iDeviceInfo < pAdapterInfo->deviceInfoList.GetSize(); iDeviceInfo++ )
        {
            CD3D9EnumDeviceInfo* pDeviceInfo = pAdapterInfo->deviceInfoList.GetAt( iDeviceInfo );
            if( pDeviceInfo->DeviceType == DeviceType )
                return pDeviceInfo;
        }
    }

    return NULL;
}


//--------------------------------------------------------------------------------------
// 
//--------------------------------------------------------------------------------------
CD3D9EnumDeviceSettingsCombo* CD3D9Enumeration::GetDeviceSettingsCombo( UINT AdapterOrdinal, D3DDEVTYPE DeviceType,
                                                                        D3DFORMAT AdapterFormat,
                                                                        D3DFORMAT BackBufferFormat, BOOL bWindowed )
{
    CD3D9EnumDeviceInfo* pDeviceInfo = GetDeviceInfo( AdapterOrdinal, DeviceType );
    if( pDeviceInfo )
    {
        for( int iDeviceCombo = 0; iDeviceCombo < pDeviceInfo->deviceSettingsComboList.GetSize(); iDeviceCombo++ )
        {
            CD3D9EnumDeviceSettingsCombo* pDeviceSettingsCombo = pDeviceInfo->deviceSettingsComboList.GetAt(
                iDeviceCombo );
            if( pDeviceSettingsCombo->AdapterFormat == AdapterFormat &&
                pDeviceSettingsCombo->BackBufferFormat == BackBufferFormat &&
                pDeviceSettingsCombo->Windowed == bWindowed )
                return pDeviceSettingsCombo;
        }
    }

    return NULL;
}


//--------------------------------------------------------------------------------------
// Returns the number of color channel bits in the specified D3DFORMAT
//--------------------------------------------------------------------------------------
UINT WINAPI DXUTGetD3D9ColorChannelBits( D3DFORMAT fmt )
{
    switch( fmt )
    {
        case D3DFMT_R8G8B8:
            return 8;
        case D3DFMT_A8R8G8B8:
            return 8;
        case D3DFMT_X8R8G8B8:
            return 8;
        case D3DFMT_R5G6B5:
            return 5;
        case D3DFMT_X1R5G5B5:
            return 5;
        case D3DFMT_A1R5G5B5:
            return 5;
        case D3DFMT_A4R4G4B4:
            return 4;
        case D3DFMT_R3G3B2:
            return 2;
        case D3DFMT_A8R3G3B2:
            return 2;
        case D3DFMT_X4R4G4B4:
            return 4;
        case D3DFMT_A2B10G10R10:
            return 10;
        case D3DFMT_A8B8G8R8:
            return 8;
        case D3DFMT_A2R10G10B10:
            return 10;
        case D3DFMT_A16B16G16R16:
            return 16;
        default:
            return 0;
    }
}


//--------------------------------------------------------------------------------------
// Returns the number of alpha channel bits in the specified D3DFORMAT
//--------------------------------------------------------------------------------------
UINT WINAPI DXUTGetAlphaChannelBits( D3DFORMAT fmt )
{
    switch( fmt )
    {
        case D3DFMT_R8G8B8:
            return 0;
        case D3DFMT_A8R8G8B8:
            return 8;
        case D3DFMT_X8R8G8B8:
            return 0;
        case D3DFMT_R5G6B5:
            return 0;
        case D3DFMT_X1R5G5B5:
            return 0;
        case D3DFMT_A1R5G5B5:
            return 1;
        case D3DFMT_A4R4G4B4:
            return 4;
        case D3DFMT_R3G3B2:
            return 0;
        case D3DFMT_A8R3G3B2:
            return 8;
        case D3DFMT_X4R4G4B4:
            return 0;
        case D3DFMT_A2B10G10R10:
            return 2;
        case D3DFMT_A8B8G8R8:
            return 8;
        case D3DFMT_A2R10G10B10:
            return 2;
        case D3DFMT_A16B16G16R16:
            return 16;
        default:
            return 0;
    }
}


//--------------------------------------------------------------------------------------
// Returns the number of depth bits in the specified D3DFORMAT
//--------------------------------------------------------------------------------------
UINT WINAPI DXUTGetDepthBits( D3DFORMAT fmt )
{
    switch( fmt )
    {
        case D3DFMT_D32F_LOCKABLE:
        case D3DFMT_D32:
            return 32;

        case D3DFMT_D24X8:
        case D3DFMT_D24S8:
        case D3DFMT_D24X4S4:
        case D3DFMT_D24FS8:
            return 24;

        case D3DFMT_D16_LOCKABLE:
        case D3DFMT_D16:
            return 16;

        case D3DFMT_D15S1:
            return 15;

        default:
            return 0;
    }
}




//--------------------------------------------------------------------------------------
// Returns the number of stencil bits in the specified D3DFORMAT
//--------------------------------------------------------------------------------------
UINT WINAPI DXUTGetStencilBits( D3DFORMAT fmt )
{
    switch( fmt )
    {
        case D3DFMT_D16_LOCKABLE:
        case D3DFMT_D16:
        case D3DFMT_D32F_LOCKABLE:
        case D3DFMT_D32:
        case D3DFMT_D24X8:
            return 0;

        case D3DFMT_D15S1:
            return 1;

        case D3DFMT_D24X4S4:
            return 4;

        case D3DFMT_D24S8:
        case D3DFMT_D24FS8:
            return 8;

        default:
            return 0;
    }
}



//--------------------------------------------------------------------------------------
// Used to sort D3DDISPLAYMODEs
//--------------------------------------------------------------------------------------
static int __cdecl SortModesCallback( const void* arg1, const void* arg2 )
{
    D3DDISPLAYMODE* pdm1 = ( D3DDISPLAYMODE* )arg1;
    D3DDISPLAYMODE* pdm2 = ( D3DDISPLAYMODE* )arg2;

    if( pdm1->Width > pdm2->Width )
        return 1;
    if( pdm1->Width < pdm2->Width )
        return -1;
    if( pdm1->Height > pdm2->Height )
        return 1;
    if( pdm1->Height < pdm2->Height )
        return -1;
    if( pdm1->Format > pdm2->Format )
        return 1;
    if( pdm1->Format < pdm2->Format )
        return -1;
    if( pdm1->RefreshRate > pdm2->RefreshRate )
        return 1;
    if( pdm1->RefreshRate < pdm2->RefreshRate )
        return -1;
    return 0;
}



//--------------------------------------------------------------------------------------
CD3D9EnumAdapterInfo::~CD3D9EnumAdapterInfo( void )
{
    CD3D9EnumDeviceInfo* pDeviceInfo;
    for( int i = 0; i < deviceInfoList.GetSize(); i++ )
    {
        pDeviceInfo = deviceInfoList.GetAt( i );
        delete pDeviceInfo;
    }
    deviceInfoList.RemoveAll();
}




//--------------------------------------------------------------------------------------
CD3D9EnumDeviceInfo::~CD3D9EnumDeviceInfo( void )
{
    CD3D9EnumDeviceSettingsCombo* pDeviceCombo;
    for( int i = 0; i < deviceSettingsComboList.GetSize(); i++ )
    {
        pDeviceCombo = deviceSettingsComboList.GetAt( i );
        delete pDeviceCombo;
    }
    deviceSettingsComboList.RemoveAll();
}


//--------------------------------------------------------------------------------------
void CD3D9Enumeration::ResetPossibleDepthStencilFormats()
{
    m_DepthStencilPossibleList.RemoveAll();
    m_DepthStencilPossibleList.Add( D3DFMT_D16 );
    m_DepthStencilPossibleList.Add( D3DFMT_D15S1 );
    m_DepthStencilPossibleList.Add( D3DFMT_D24X8 );
    m_DepthStencilPossibleList.Add( D3DFMT_D24S8 );
    m_DepthStencilPossibleList.Add( D3DFMT_D24X4S4 );
    m_DepthStencilPossibleList.Add( D3DFMT_D32 );
}


//--------------------------------------------------------------------------------------
CGrowableArray <D3DFORMAT>* CD3D9Enumeration::GetPossibleDepthStencilFormatList()
{
    return &m_DepthStencilPossibleList;
}


//--------------------------------------------------------------------------------------
CGrowableArray <D3DMULTISAMPLE_TYPE>* CD3D9Enumeration::GetPossibleMultisampleTypeList()
{
    return &m_MultiSampleTypeList;
}


//--------------------------------------------------------------------------------------
void CD3D9Enumeration::ResetPossibleMultisampleTypeList()
{
    m_MultiSampleTypeList.RemoveAll();
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_NONE );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_NONMASKABLE );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_2_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_3_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_4_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_5_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_6_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_7_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_8_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_9_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_10_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_11_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_12_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_13_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_14_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_15_SAMPLES );
    m_MultiSampleTypeList.Add( D3DMULTISAMPLE_16_SAMPLES );
}


//--------------------------------------------------------------------------------------
void CD3D9Enumeration::GetPossibleVertexProcessingList( bool* pbSoftwareVP, bool* pbHardwareVP, bool* pbPureHarewareVP,
                                                        bool* pbMixedVP )
{
    *pbSoftwareVP = m_bSoftwareVP;
    *pbHardwareVP = m_bHardwareVP;
    *pbPureHarewareVP = m_bPureHarewareVP;
    *pbMixedVP = m_bMixedVP;
}


//--------------------------------------------------------------------------------------
void CD3D9Enumeration::SetPossibleVertexProcessingList( bool bSoftwareVP, bool bHardwareVP, bool bPureHarewareVP,
                                                        bool bMixedVP )
{
    m_bSoftwareVP = bSoftwareVP;
    m_bHardwareVP = bHardwareVP;
    m_bPureHarewareVP = bPureHarewareVP;
    m_bMixedVP = bMixedVP;
}


//--------------------------------------------------------------------------------------
CGrowableArray <UINT>* CD3D9Enumeration::GetPossiblePresentIntervalList()
{
    return &m_PresentIntervalList;
}


//--------------------------------------------------------------------------------------
void CD3D9Enumeration::ResetPossiblePresentIntervalList()
{
    m_PresentIntervalList.RemoveAll();
    m_PresentIntervalList.Add( D3DPRESENT_INTERVAL_IMMEDIATE );
    m_PresentIntervalList.Add( D3DPRESENT_INTERVAL_DEFAULT );
    m_PresentIntervalList.Add( D3DPRESENT_INTERVAL_ONE );
    m_PresentIntervalList.Add( D3DPRESENT_INTERVAL_TWO );
    m_PresentIntervalList.Add( D3DPRESENT_INTERVAL_THREE );
    m_PresentIntervalList.Add( D3DPRESENT_INTERVAL_FOUR );
}


//--------------------------------------------------------------------------------------
void CD3D9Enumeration::SetResolutionMinMax( UINT nMinWidth, UINT nMinHeight,
                                            UINT nMaxWidth, UINT nMaxHeight )
{
    m_nMinWidth = nMinWidth;
    m_nMinHeight = nMinHeight;
    m_nMaxWidth = nMaxWidth;
    m_nMaxHeight = nMaxHeight;
}


//--------------------------------------------------------------------------------------
void CD3D9Enumeration::SetRefreshMinMax( UINT nMin, UINT nMax )
{
    m_nRefreshMin = nMin;
    m_nRefreshMax = nMax;
}


//--------------------------------------------------------------------------------------
void CD3D9Enumeration::SetMultisampleQualityMax( UINT nMax )
{
    if( nMax > 0xFFFF )
        nMax = 0xFFFF;
    m_nMultisampleQualityMax = nMax;
}



//--------------------------------------------------------------------------------------
// Returns a ranking number that describes how closely this device 
// combo matches the optimal combo based on the match options and the optimal device settings
//--------------------------------------------------------------------------------------
float DXUTRankD3D9DeviceCombo( CD3D9EnumDeviceSettingsCombo* pDeviceSettingsCombo,
                               DXUTD3D9DeviceSettings* pOptimalDeviceSettings,
                               D3DDISPLAYMODE* pAdapterDesktopDisplayMode, 
                               int &bestModeIndex,
                               int &bestMSAAIndex
                               )
{
    float fCurRanking = 0.0f;

    // Arbitrary weights.  Gives preference to the ordinal, device type, and windowed
    const float fAdapterOrdinalWeight   = 1000.0f;
    const float fDeviceTypeWeight       = 100.0f;
    const float fWindowWeight           = 10.0f;
    const float fAdapterFormatWeight    = 1.0f;
    const float fVertexProcessingWeight = 1.0f;
    const float fResolutionWeight       = 1.0f;
    const float fBackBufferFormatWeight = 1.0f;
    const float fMultiSampleWeight      = 1.0f;
    const float fDepthStencilWeight     = 1.0f;
    const float fRefreshRateWeight      = 1.0f;
    const float fPresentIntervalWeight  = 1.0f;

    //---------------------
    // Adapter ordinal
    //---------------------
    if( pDeviceSettingsCombo->AdapterOrdinal == pOptimalDeviceSettings->AdapterOrdinal )
        fCurRanking += fAdapterOrdinalWeight;

    //---------------------
    // Device type
    //---------------------
    if( pDeviceSettingsCombo->DeviceType == pOptimalDeviceSettings->DeviceType )
        fCurRanking += fDeviceTypeWeight;
    // Slightly prefer HAL 
    if( pDeviceSettingsCombo->DeviceType == D3DDEVTYPE_HAL )
        fCurRanking += 0.1f;

    //---------------------
    // Windowed
    //---------------------
    if( pDeviceSettingsCombo->Windowed == pOptimalDeviceSettings->pp.Windowed )
        fCurRanking += fWindowWeight;

    //---------------------
    // Adapter format
    //---------------------
    if( pDeviceSettingsCombo->AdapterFormat == pOptimalDeviceSettings->AdapterFormat )
    {
        fCurRanking += fAdapterFormatWeight;
    }
    else
    {
        int nBitDepthDelta = abs( ( long )DXUTGetD3D9ColorChannelBits( pDeviceSettingsCombo->AdapterFormat ) -
                                  ( long )DXUTGetD3D9ColorChannelBits( pOptimalDeviceSettings->AdapterFormat ) );
        float fScale = __max( 0.9f - ( float )nBitDepthDelta * 0.2f, 0.0f );
        fCurRanking += fScale * fAdapterFormatWeight;
    }

    if( !pDeviceSettingsCombo->Windowed )
    {
        // Slightly prefer when it matches the desktop format or is D3DFMT_X8R8G8B8
        bool bAdapterOptimalMatch;
        if( DXUTGetD3D9ColorChannelBits( pAdapterDesktopDisplayMode->Format ) >= 8 )
            bAdapterOptimalMatch = ( pDeviceSettingsCombo->AdapterFormat == pAdapterDesktopDisplayMode->Format );
        else
            bAdapterOptimalMatch = ( pDeviceSettingsCombo->AdapterFormat == D3DFMT_X8R8G8B8 );

        if( bAdapterOptimalMatch )
            fCurRanking += 0.1f;
    }

    //---------------------
    // Vertex processing
    //---------------------
    if( ( pOptimalDeviceSettings->BehaviorFlags & D3DCREATE_HARDWARE_VERTEXPROCESSING ) != 0 ||
        ( pOptimalDeviceSettings->BehaviorFlags & D3DCREATE_MIXED_VERTEXPROCESSING ) != 0 )
    {
        if( ( pDeviceSettingsCombo->pDeviceInfo->Caps.DevCaps & D3DDEVCAPS_HWTRANSFORMANDLIGHT ) != 0 )
            fCurRanking += fVertexProcessingWeight;
    }
    // Slightly prefer HW T&L
    if( ( pDeviceSettingsCombo->pDeviceInfo->Caps.DevCaps & D3DDEVCAPS_HWTRANSFORMANDLIGHT ) != 0 )
        fCurRanking += 0.1f;

    //---------------------
    // Resolution
    //---------------------
    bool bResolutionFound = false;
    unsigned int best = 0xffffffff;
    bestModeIndex=0;





    for( int idm = 0; idm < pDeviceSettingsCombo->pAdapterInfo->displayModeList.GetSize(); idm++ )
    {
        D3DDISPLAYMODE displayMode = pDeviceSettingsCombo->pAdapterInfo->displayModeList.GetAt( idm );
        if( displayMode.Format != pDeviceSettingsCombo->AdapterFormat )
            continue;
        if( displayMode.Width == pOptimalDeviceSettings->pp.BackBufferWidth &&
            displayMode.Height == pOptimalDeviceSettings->pp.BackBufferHeight )
            bResolutionFound = true;

        unsigned int current = 
            (UINT) abs ((int)displayMode.Width  - (int)pOptimalDeviceSettings->pp.BackBufferWidth) + 
            (UINT) abs ((int)displayMode.Height - (int)pOptimalDeviceSettings->pp.BackBufferHeight );
        if (current < best) {
            best = current;
            bestModeIndex= idm;

        }


    }
    if( bResolutionFound )
        fCurRanking += fResolutionWeight;

    //---------------------
    // Back buffer format
    //---------------------
    if( pDeviceSettingsCombo->BackBufferFormat == pOptimalDeviceSettings->pp.BackBufferFormat )
    {
        fCurRanking += fBackBufferFormatWeight;
    }
    else
    {
        int nBitDepthDelta = abs( ( long )DXUTGetD3D9ColorChannelBits( pDeviceSettingsCombo->BackBufferFormat ) -
                                  ( long )DXUTGetD3D9ColorChannelBits( pOptimalDeviceSettings->pp.BackBufferFormat ) );
        float fScale = __max( 0.9f - ( float )nBitDepthDelta * 0.2f, 0.0f );
        fCurRanking += fScale * fBackBufferFormatWeight;
    }

    // Check if this back buffer format is the same as 
    // the adapter format since this is preferred.
    bool bAdapterMatchesBB = ( pDeviceSettingsCombo->BackBufferFormat == pDeviceSettingsCombo->AdapterFormat );
    if( bAdapterMatchesBB )
        fCurRanking += 0.1f;

    //---------------------
    // Back buffer count
    //---------------------
    // No caps for the back buffer count

    //---------------------
    // Multisample
    //---------------------
    bool bMultiSampleFound = false;
    for( int i = 0; i < pDeviceSettingsCombo->multiSampleTypeList.GetSize(); i++ )
    {
        D3DMULTISAMPLE_TYPE msType = pDeviceSettingsCombo->multiSampleTypeList.GetAt( i );
        DWORD msQuality = pDeviceSettingsCombo->multiSampleQualityList.GetAt( i );

        if( msType == pOptimalDeviceSettings->pp.MultiSampleType &&
            msQuality > pOptimalDeviceSettings->pp.MultiSampleQuality )
        {
            bMultiSampleFound = true;
            bestMSAAIndex = i;
            break;
        }
    }
    if( bMultiSampleFound )
        fCurRanking += fMultiSampleWeight;

    //---------------------
    // Swap effect
    //---------------------
    // No caps for swap effects

    //---------------------
    // Depth stencil 
    //---------------------
    if( pDeviceSettingsCombo->depthStencilFormatList.Contains( pOptimalDeviceSettings->pp.AutoDepthStencilFormat ) )
        fCurRanking += fDepthStencilWeight;

    //---------------------
    // Present flags
    //---------------------
    // No caps for the present flags

    //---------------------
    // Refresh rate
    //---------------------
    bool bRefreshFound = false;
    for( int idm = 0; idm < pDeviceSettingsCombo->pAdapterInfo->displayModeList.GetSize(); idm++ )
    {
        D3DDISPLAYMODE displayMode = pDeviceSettingsCombo->pAdapterInfo->displayModeList.GetAt( idm );
        if( displayMode.Format != pDeviceSettingsCombo->AdapterFormat )
            continue;
        if( displayMode.RefreshRate == pOptimalDeviceSettings->pp.FullScreen_RefreshRateInHz )
            bRefreshFound = true;
    }
    if( bRefreshFound )
        fCurRanking += fRefreshRateWeight;

    //---------------------
    // Present interval
    //---------------------
    // If keep present interval then check that the present interval is supported by this combo
    if( pDeviceSettingsCombo->presentIntervalList.Contains( pOptimalDeviceSettings->pp.PresentationInterval ) )
        fCurRanking += fPresentIntervalWeight;

    return fCurRanking;
}

