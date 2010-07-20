//--------------------------------------------------------------------------------------
// File: DXUTDevice11.cpp
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
extern void DXUTGetCallbackD3D11DeviceAcceptable( LPDXUTCALLBACKISD3D11DEVICEACCEPTABLE* ppCallbackIsDeviceAcceptable, void** ppUserContext );

static int __cdecl SortModesCallback( const void* arg1, const void* arg2 );

CD3D11Enumeration*  g_pDXUTD3D11Enumeration = NULL;




HRESULT WINAPI DXUTCreateD3D11Enumeration()
{
    if( g_pDXUTD3D11Enumeration == NULL )
    {
        g_pDXUTD3D11Enumeration = new CD3D11Enumeration();
        if( NULL == g_pDXUTD3D11Enumeration )
            return E_OUTOFMEMORY;
    }
    return S_OK;
}

void WINAPI DXUTDestroyD3D11Enumeration()
{
    SAFE_DELETE( g_pDXUTD3D11Enumeration );
}

class DXUTMemoryHelperD3D11Enum
{
public:
DXUTMemoryHelperD3D11Enum()
{
    DXUTCreateD3D11Enumeration();
}
~DXUTMemoryHelperD3D11Enum()
{
    DXUTDestroyD3D11Enumeration();
}
};


//--------------------------------------------------------------------------------------
CD3D11Enumeration* WINAPI DXUTGetD3D11Enumeration( bool bForceEnumerate, bool bEnumerateAllAdapterFormats, D3D_FEATURE_LEVEL forceFL )
{
    // Using an static class with accessor function to allow control of the construction order
    static DXUTMemoryHelperD3D11Enum d3d11enumMemory;
    if( g_pDXUTD3D11Enumeration && ( !g_pDXUTD3D11Enumeration->HasEnumerated() || bForceEnumerate ) )
    {
        g_pDXUTD3D11Enumeration->SetEnumerateAllAdapterFormats( bEnumerateAllAdapterFormats );
        LPDXUTCALLBACKISD3D11DEVICEACCEPTABLE pCallbackIsDeviceAcceptable;
        void* pUserContext;
        DXUTGetCallbackD3D11DeviceAcceptable( &pCallbackIsDeviceAcceptable, &pUserContext );
        g_pDXUTD3D11Enumeration->SetForceFeatureLevel(forceFL);

        g_pDXUTD3D11Enumeration->Enumerate( pCallbackIsDeviceAcceptable, pUserContext );
    }

    return g_pDXUTD3D11Enumeration;
}


//--------------------------------------------------------------------------------------
CD3D11Enumeration::CD3D11Enumeration()
{
    m_bHasEnumerated = false;
    m_IsD3D11DeviceAcceptableFunc = NULL;
    m_pIsD3D11DeviceAcceptableFuncUserContext = NULL;

    m_nMinWidth = 640;
    m_nMinHeight = 480;
    m_nMaxWidth = UINT_MAX;
    m_nMaxHeight = UINT_MAX;
    m_bEnumerateAllAdapterFormats = false;

    m_nRefreshMin = 0;
    m_nRefreshMax = UINT_MAX;

    ResetPossibleDepthStencilFormats();
}


//--------------------------------------------------------------------------------------
CD3D11Enumeration::~CD3D11Enumeration()
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
HRESULT CD3D11Enumeration::Enumerate( LPDXUTCALLBACKISD3D11DEVICEACCEPTABLE IsD3D11DeviceAcceptableFunc,
                                      void* pIsD3D11DeviceAcceptableFuncUserContext )
{
    CDXUTPerfEventGenerator eventGenerator( DXUT_PERFEVENTCOLOR, L"DXUT D3D11 Enumeration" );
    HRESULT hr;
    IDXGIFactory1* pFactory = DXUTGetDXGIFactory();
    if( pFactory == NULL )
        return E_FAIL;

    m_bHasEnumerated = true;
    m_IsD3D11DeviceAcceptableFunc = IsD3D11DeviceAcceptableFunc;
    m_pIsD3D11DeviceAcceptableFuncUserContext = pIsD3D11DeviceAcceptableFuncUserContext;

    ClearAdapterInfoList();

    for( int index = 0; ; ++index )
    {
        IDXGIAdapter* pAdapter = NULL;
        hr = pFactory->EnumAdapters( index, &pAdapter );
        if( FAILED( hr ) ) // DXGIERR_NOT_FOUND is expected when the end of the list is hit
            break;

        CD3D11EnumAdapterInfo* pAdapterInfo = new CD3D11EnumAdapterInfo;
        if( !pAdapterInfo )
        {
            SAFE_RELEASE( pAdapter );
            return E_OUTOFMEMORY;
        }
        ZeroMemory( pAdapterInfo, sizeof( CD3D11EnumAdapterInfo ) );
        pAdapterInfo->AdapterOrdinal = index;
        pAdapter->GetDesc( &pAdapterInfo->AdapterDesc );
        pAdapterInfo->m_pAdapter = pAdapter;

        // Enumerate the device driver types on the adapter.
        hr = EnumerateDevices( pAdapterInfo );
        if( FAILED( hr ) )
        {
            delete pAdapterInfo;
            continue;
        }

        hr = EnumerateOutputs( pAdapterInfo );
        if( FAILED( hr ) || pAdapterInfo->outputInfoList.GetSize() <= 0 )
        {
            delete pAdapterInfo;
            continue;
        }

        // Get info for each devicecombo on this device
        if( FAILED( hr = EnumerateDeviceCombos( pFactory, pAdapterInfo ) ) )
        {
            delete pAdapterInfo;
            continue;
        }

        hr = m_AdapterInfoList.Add( pAdapterInfo );
        if( FAILED( hr ) )
        {
            delete pAdapterInfo;
            return hr;
        }
    }


    //  If we did not get an adapter then we should still enumerate WARP and Ref.
    if (m_AdapterInfoList.GetSize() == 0) {


        CD3D11EnumAdapterInfo* pAdapterInfo = new CD3D11EnumAdapterInfo;
        if( !pAdapterInfo )
        {
            return E_OUTOFMEMORY;
        }
        ZeroMemory( pAdapterInfo, sizeof( CD3D11EnumAdapterInfo ) );
        pAdapterInfo->bAdapterUnavailable = true;

        hr = EnumerateDevices( pAdapterInfo );

        // Get info for each devicecombo on this device
        if( FAILED( hr = EnumerateDeviceCombosNoAdapter(  pAdapterInfo ) ) )
        {
            delete pAdapterInfo;
        }

        if (!FAILED(hr)) hr = m_AdapterInfoList.Add( pAdapterInfo );
    }

    //
    // Check for 2 or more adapters with the same name. Append the name
    // with some instance number if that's the case to help distinguish
    // them.
    //
    bool bUniqueDesc = true;
    CD3D11EnumAdapterInfo* pAdapterInfo;
    for( int i = 0; i < m_AdapterInfoList.GetSize(); i++ )
    {
        CD3D11EnumAdapterInfo* pAdapterInfo1 = m_AdapterInfoList.GetAt( i );

        for( int j = i + 1; j < m_AdapterInfoList.GetSize(); j++ )
        {
            CD3D11EnumAdapterInfo* pAdapterInfo2 = m_AdapterInfoList.GetAt( j );
            if( wcsncmp( pAdapterInfo1->AdapterDesc.Description,
                pAdapterInfo2->AdapterDesc.Description, DXGI_MAX_DEVICE_IDENTIFIER_STRING ) == 0 )
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

        wcscpy_s( pAdapterInfo->szUniqueDescription, 100, pAdapterInfo->AdapterDesc.Description );
        if( !bUniqueDesc )
        {
            WCHAR sz[100];
            swprintf_s( sz, 100, L" (#%d)", pAdapterInfo->AdapterOrdinal );
            wcscat_s( pAdapterInfo->szUniqueDescription, DXGI_MAX_DEVICE_IDENTIFIER_STRING, sz );
        }
    }

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CD3D11Enumeration::EnumerateOutputs( CD3D11EnumAdapterInfo* pAdapterInfo )
{
    HRESULT hr;
    IDXGIOutput* pOutput;

    for( int iOutput = 0; ; ++iOutput )
    {
        pOutput = NULL;
        hr = pAdapterInfo->m_pAdapter->EnumOutputs( iOutput, &pOutput );
        if( DXGI_ERROR_NOT_FOUND == hr )
        {
            return S_OK;
        }
        else if( FAILED( hr ) )
        {
            return hr;	//Something bad happened.
        }
        else //Success!
        {
            CD3D11EnumOutputInfo* pOutputInfo = new CD3D11EnumOutputInfo;
            if( !pOutputInfo )
            {
                SAFE_RELEASE( pOutput );
                return E_OUTOFMEMORY;
            }
            ZeroMemory( pOutputInfo, sizeof( CD3D11EnumOutputInfo ) );
            pOutput->GetDesc( &pOutputInfo->Desc );
            pOutputInfo->Output = iOutput;
            pOutputInfo->m_pOutput = pOutput;

            EnumerateDisplayModes( pOutputInfo );
            if( pOutputInfo->displayModeList.GetSize() <= 0 )
            {
                // If this output has no valid display mode, do not save it.
                delete pOutputInfo;
                continue;
            }

            hr = pAdapterInfo->outputInfoList.Add( pOutputInfo );
            if( FAILED( hr ) )
            {
                delete pOutputInfo;
                return hr;
            }
        }
    }
}


//--------------------------------------------------------------------------------------
HRESULT CD3D11Enumeration::EnumerateDisplayModes( CD3D11EnumOutputInfo* pOutputInfo )
{
    HRESULT hr = S_OK;
    DXGI_FORMAT allowedAdapterFormatArray[] =
    {
        DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,     //This is DXUT's preferred mode

        DXGI_FORMAT_R8G8B8A8_UNORM,			
        DXGI_FORMAT_R16G16B16A16_FLOAT,
        DXGI_FORMAT_R10G10B10A2_UNORM
    };
    int allowedAdapterFormatArrayCount = sizeof( allowedAdapterFormatArray ) / sizeof( allowedAdapterFormatArray[0] );

    // Swap perferred modes for apps running in linear space
    DXGI_FORMAT RemoteMode = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    if( !DXUTIsInGammaCorrectMode() )
    {
        allowedAdapterFormatArray[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
        allowedAdapterFormatArray[1] = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
        RemoteMode = DXGI_FORMAT_R8G8B8A8_UNORM;
    }

    // The fast path only enumerates R8G8B8A8_UNORM_SRGB modes
    if( !m_bEnumerateAllAdapterFormats )
        allowedAdapterFormatArrayCount = 1;

    for( int f = 0; f < allowedAdapterFormatArrayCount; ++f )
    {
        // Fast-path: Try to grab at least 512 modes.
        //			  This is to avoid calling GetDisplayModeList more times than necessary.
        //			  GetDisplayModeList is an expensive call.
        UINT NumModes = 512;
        DXGI_MODE_DESC* pDesc = new DXGI_MODE_DESC[ NumModes ];
        assert( pDesc );
        if( !pDesc )
            return E_OUTOFMEMORY;

        hr = pOutputInfo->m_pOutput->GetDisplayModeList( allowedAdapterFormatArray[f],
                                                         DXGI_ENUM_MODES_SCALING,
                                                         &NumModes,
                                                         pDesc );
        if( DXGI_ERROR_NOT_FOUND == hr )
        {
            SAFE_DELETE_ARRAY( pDesc );
            NumModes = 0;
            break;
        }
        else if( MAKE_DXGI_HRESULT( 34 ) == hr && RemoteMode == allowedAdapterFormatArray[f] )
        {
            // DXGI cannot enumerate display modes over a remote session.  Therefore, create a fake display
            // mode for the current screen resolution for the remote session.
            if( 0 != GetSystemMetrics( 0x1000 ) ) // SM_REMOTESESSION
            {
                DEVMODE DevMode;
                DevMode.dmSize = sizeof( DEVMODE );
                if( EnumDisplaySettings( NULL, ENUM_CURRENT_SETTINGS, &DevMode ) )
                {
                    NumModes = 1;
                    pDesc[0].Width = DevMode.dmPelsWidth;
                    pDesc[0].Height = DevMode.dmPelsHeight;
                    pDesc[0].Format = DXGI_FORMAT_R8G8B8A8_UNORM;
                    pDesc[0].RefreshRate.Numerator = 60;
                    pDesc[0].RefreshRate.Denominator = 1;
                    pDesc[0].ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_PROGRESSIVE;
                    pDesc[0].Scaling = DXGI_MODE_SCALING_CENTERED;
                    hr = S_OK;
                }
            }
        }
        else if( DXGI_ERROR_MORE_DATA == hr )
        {
            // Slow path.  There were more than 512 modes.
            SAFE_DELETE_ARRAY( pDesc );
            hr = pOutputInfo->m_pOutput->GetDisplayModeList( allowedAdapterFormatArray[f],
                                                             DXGI_ENUM_MODES_SCALING,
                                                             &NumModes,
                                                             NULL );
            if( FAILED( hr ) )
            {
                NumModes = 0;
                break;
            }

            pDesc = new DXGI_MODE_DESC[ NumModes ];
            assert( pDesc );
            if( !pDesc )
                return E_OUTOFMEMORY;

            hr = pOutputInfo->m_pOutput->GetDisplayModeList( allowedAdapterFormatArray[f],
                                                             DXGI_ENUM_MODES_SCALING,
                                                             &NumModes,
                                                             pDesc );
            if( FAILED( hr ) )
            {
                SAFE_DELETE_ARRAY( pDesc );
                NumModes = 0;
                break;
            }

        }

        if( 0 == NumModes && 0 == f )
        {
            // No R8G8B8A8_UNORM_SRGB modes!
            // Abort the fast-path if we're on it
            allowedAdapterFormatArrayCount = sizeof( allowedAdapterFormatArray ) / sizeof
                ( allowedAdapterFormatArray[0] );
            SAFE_DELETE_ARRAY( pDesc );
            continue;
        }

        if( SUCCEEDED( hr ) )
        {
            for( UINT m = 0; m < NumModes; m++ )
            {
                pOutputInfo->displayModeList.Add( pDesc[m] );
            }
        }

        SAFE_DELETE_ARRAY( pDesc );
    }

    return hr;
}


//--------------------------------------------------------------------------------------
HRESULT CD3D11Enumeration::EnumerateDevices( CD3D11EnumAdapterInfo* pAdapterInfo )
{
    HRESULT hr;
    DXUTDeviceSettings deviceSettings = DXUTGetDeviceSettings();
    const D3D_DRIVER_TYPE devTypeArray[] =
    {
        D3D_DRIVER_TYPE_HARDWARE,
        D3D_DRIVER_TYPE_WARP,
        D3D_DRIVER_TYPE_REFERENCE
    };
    const UINT devTypeArrayCount = sizeof( devTypeArray ) / sizeof( devTypeArray[0] );

    // Enumerate each Direct3D device type
    for( UINT iDeviceType = 0; iDeviceType < devTypeArrayCount; iDeviceType++ )
    {
        CD3D11EnumDeviceInfo* pDeviceInfo = new CD3D11EnumDeviceInfo;
        if( pDeviceInfo == NULL )
            return E_OUTOFMEMORY;

        // Fill struct w/ AdapterOrdinal and D3DX10_DRIVER_TYPE
        pDeviceInfo->AdapterOrdinal = pAdapterInfo->AdapterOrdinal;
        pDeviceInfo->DeviceType = devTypeArray[iDeviceType];

        D3D_FEATURE_LEVEL FeatureLevels[] =
        {
                    D3D_FEATURE_LEVEL_11_0,
                    D3D_FEATURE_LEVEL_10_1,
                    D3D_FEATURE_LEVEL_10_0,
                    D3D_FEATURE_LEVEL_9_3,
                    D3D_FEATURE_LEVEL_9_2,
                    D3D_FEATURE_LEVEL_9_1
        };
        UINT NumFeatureLevels = ARRAYSIZE( FeatureLevels );

        // Call D3D11CreateDevice to ensure that this is a D3D11 device.
        ID3D11Device* pd3dDevice = NULL;
        ID3D11DeviceContext* pd3dDeviceContext = NULL;
        IDXGIAdapter* pAdapter = NULL;
        //if( devTypeArray[iDeviceType] == D3D_DRIVER_TYPE_HARDWARE )
        //    pAdapter = pAdapterInfo->m_pAdapter;
        hr = DXUT_Dynamic_D3D11CreateDevice( pAdapter,
                                             devTypeArray[iDeviceType],
                                             ( HMODULE )0,
                                             0,
                                             FeatureLevels,
                                             NumFeatureLevels,
                                             D3D11_SDK_VERSION,
                                             &pd3dDevice,
                                             &pDeviceInfo->MaxLevel,
                                             &pd3dDeviceContext );
        if( FAILED( hr ) || pDeviceInfo->MaxLevel < deviceSettings.MinimumFeatureLevel)
        {
            delete pDeviceInfo;
            continue;
        }
        
        if (g_forceFL == 0 || g_forceFL == pDeviceInfo->MaxLevel) { 
            pDeviceInfo->SelectedLevel = pDeviceInfo->MaxLevel;
        }
        else if (g_forceFL > pDeviceInfo->MaxLevel) {
            delete pDeviceInfo;
            SAFE_RELEASE( pd3dDevice );
            SAFE_RELEASE( pd3dDeviceContext );        
            continue;
        } else {
            // A device was created with a higher feature level that the user-specified feature level.
            SAFE_RELEASE( pd3dDevice );
            SAFE_RELEASE( pd3dDeviceContext );
            D3D_FEATURE_LEVEL rtFL;
            hr = DXUT_Dynamic_D3D11CreateDevice( pAdapter,
                                             devTypeArray[iDeviceType],
                                             ( HMODULE )0,
                                             0,
                                             &g_forceFL,
                                             1,
                                             D3D11_SDK_VERSION,
                                             &pd3dDevice,
                                             &rtFL,
                                             &pd3dDeviceContext );

            if( !FAILED( hr ) && rtFL == g_forceFL ) {
                
                pDeviceInfo->SelectedLevel = g_forceFL;
            }else {
                delete pDeviceInfo;
                SAFE_RELEASE( pd3dDevice );
                SAFE_RELEASE( pd3dDeviceContext );        
                continue;
            }
        }

        IDXGIDevice1* pDXGIDev = NULL;
        hr = pd3dDevice->QueryInterface( __uuidof( IDXGIDevice1 ), ( LPVOID* )&pDXGIDev );
        if( SUCCEEDED( hr ) && pDXGIDev )
        {
            SAFE_RELEASE( pAdapterInfo->m_pAdapter );
            pDXGIDev->GetAdapter( &pAdapterInfo->m_pAdapter );
        }
        SAFE_RELEASE( pDXGIDev );


        D3D11_FEATURE_DATA_D3D10_X_HARDWARE_OPTIONS ho;
        pd3dDevice->CheckFeatureSupport(D3D11_FEATURE_D3D10_X_HARDWARE_OPTIONS, &ho, sizeof(ho));
        pDeviceInfo->ComputeShaders_Plus_RawAndStructuredBuffers_Via_Shader_4_x = ho.ComputeShaders_Plus_RawAndStructuredBuffers_Via_Shader_4_x; 
        SAFE_RELEASE( pd3dDeviceContext );             
        SAFE_RELEASE( pd3dDevice );
        pAdapterInfo->deviceInfoList.Add( pDeviceInfo );
    }

    return S_OK;
}


HRESULT CD3D11Enumeration::EnumerateDeviceCombosNoAdapter(  CD3D11EnumAdapterInfo* pAdapterInfo )
{
    // Iterate through each combination of device driver type, output,
    // adapter format, and backbuffer format to build the adapter's device combo list.
    //

        for( int device = 0; device < pAdapterInfo->deviceInfoList.GetSize(); ++device )
        {
            CD3D11EnumDeviceInfo* pDeviceInfo = pAdapterInfo->deviceInfoList.GetAt( device );

            DXGI_FORMAT BufferFormatArray[] =
            {
                DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,   //This is DXUT's preferred mode

                DXGI_FORMAT_R8G8B8A8_UNORM,		
                DXGI_FORMAT_R16G16B16A16_FLOAT,
                DXGI_FORMAT_R10G10B10A2_UNORM
            };
            const UINT BufferFormatArrayCount = sizeof( BufferFormatArray ) / sizeof
                ( BufferFormatArray[0] );

            // Swap perferred modes for apps running in linear space
            if( !DXUTIsInGammaCorrectMode() )
            {
                BufferFormatArray[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
                BufferFormatArray[1] = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
            }

            for( UINT iBufferFormat = 0; iBufferFormat < BufferFormatArrayCount; iBufferFormat++ )
            {
                DXGI_FORMAT BufferFormat = BufferFormatArray[iBufferFormat];



                    // determine if there are any modes for this particular format


                    // If an application callback function has been provided, make sure this device
                    // is acceptable to the app.
                    if( m_IsD3D11DeviceAcceptableFunc != NULL )
                    {
                        if( !m_IsD3D11DeviceAcceptableFunc( pAdapterInfo, 
                                                            0,
                                                            pDeviceInfo, 
                                                            BufferFormat,
                                                            TRUE,
                                                            m_pIsD3D11DeviceAcceptableFuncUserContext ) )
                            continue;
                    }

                    // At this point, we have an adapter/device/backbufferformat/iswindowed
                    // DeviceCombo that is supported by the system. We still 
                    // need to find one or more suitable depth/stencil buffer format,
                    // multisample type, and present interval.
                    CD3D11EnumDeviceSettingsCombo* pDeviceCombo = new CD3D11EnumDeviceSettingsCombo;
                    if( pDeviceCombo == NULL )
                        return E_OUTOFMEMORY;

                    pDeviceCombo->AdapterOrdinal = pDeviceInfo->AdapterOrdinal;
                    pDeviceCombo->DeviceType = pDeviceInfo->DeviceType;
                    pDeviceCombo->BackBufferFormat = BufferFormat;
                    pDeviceCombo->Windowed = TRUE;
                    pDeviceCombo->Output = 0;
                    pDeviceCombo->pAdapterInfo = pAdapterInfo;
                    pDeviceCombo->pDeviceInfo = pDeviceInfo;
                    pDeviceCombo->pOutputInfo = NULL;

                    BuildMultiSampleQualityList( BufferFormat, pDeviceCombo );

                    if( FAILED( pAdapterInfo->deviceSettingsComboList.Add( pDeviceCombo ) ) )
                        delete pDeviceCombo;
                }
                    
        }


    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CD3D11Enumeration::EnumerateDeviceCombos( IDXGIFactory1* pFactory, CD3D11EnumAdapterInfo* pAdapterInfo )
{
    // Iterate through each combination of device driver type, output,
    // adapter format, and backbuffer format to build the adapter's device combo list.
    //

    for( int output = 0; output < pAdapterInfo->outputInfoList.GetSize(); ++output )
    {
        CD3D11EnumOutputInfo* pOutputInfo = pAdapterInfo->outputInfoList.GetAt( output );

        for( int device = 0; device < pAdapterInfo->deviceInfoList.GetSize(); ++device )
        {
            CD3D11EnumDeviceInfo* pDeviceInfo = pAdapterInfo->deviceInfoList.GetAt( device );

            DXGI_FORMAT backBufferFormatArray[] =
            {
                DXGI_FORMAT_R8G8B8A8_UNORM_SRGB,   //This is DXUT's preferred mode

                DXGI_FORMAT_R8G8B8A8_UNORM,		
                DXGI_FORMAT_R16G16B16A16_FLOAT,
                DXGI_FORMAT_R10G10B10A2_UNORM
            };
            const UINT backBufferFormatArrayCount = sizeof( backBufferFormatArray ) / sizeof
                ( backBufferFormatArray[0] );

            // Swap perferred modes for apps running in linear space
            if( !DXUTIsInGammaCorrectMode() )
            {
                backBufferFormatArray[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
                backBufferFormatArray[1] = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
            }

            for( UINT iBackBufferFormat = 0; iBackBufferFormat < backBufferFormatArrayCount; iBackBufferFormat++ )
            {
                DXGI_FORMAT backBufferFormat = backBufferFormatArray[iBackBufferFormat];

                for( int nWindowed = 0; nWindowed < 2; nWindowed++ )
                {
                    if( !nWindowed && pOutputInfo->displayModeList.GetSize() == 0 )
                        continue;

                    // determine if there are any modes for this particular format
                    UINT iModes = 0;
                    for( int i = 0; i < pOutputInfo->displayModeList.GetSize(); i++ )
                    {
                        if( backBufferFormat == pOutputInfo->displayModeList.GetAt( i ).Format )
                            iModes ++;
                    }
                    if( 0 == iModes )
                        continue;

                    // If an application callback function has been provided, make sure this device
                    // is acceptable to the app.
                    if( m_IsD3D11DeviceAcceptableFunc != NULL )
                    {
                        if( !m_IsD3D11DeviceAcceptableFunc( pAdapterInfo, output,
                                                            pDeviceInfo, backBufferFormat,
                                                            FALSE != nWindowed,
                                                            m_pIsD3D11DeviceAcceptableFuncUserContext ) )
                            continue;
                    }

                    // At this point, we have an adapter/device/backbufferformat/iswindowed
                    // DeviceCombo that is supported by the system. We still 
                    // need to find one or more suitable depth/stencil buffer format,
                    // multisample type, and present interval.
                    CD3D11EnumDeviceSettingsCombo* pDeviceCombo = new CD3D11EnumDeviceSettingsCombo;
                    if( pDeviceCombo == NULL )
                        return E_OUTOFMEMORY;

                    pDeviceCombo->AdapterOrdinal = pDeviceInfo->AdapterOrdinal;
                    pDeviceCombo->DeviceType = pDeviceInfo->DeviceType;
                    pDeviceCombo->BackBufferFormat = backBufferFormat;
                    pDeviceCombo->Windowed = ( nWindowed != 0 );
                    pDeviceCombo->Output = pOutputInfo->Output;
                    pDeviceCombo->pAdapterInfo = pAdapterInfo;
                    pDeviceCombo->pDeviceInfo = pDeviceInfo;
                    pDeviceCombo->pOutputInfo = pOutputInfo;

                    BuildMultiSampleQualityList( backBufferFormat, pDeviceCombo );

                    if( FAILED( pAdapterInfo->deviceSettingsComboList.Add( pDeviceCombo ) ) )
                        delete pDeviceCombo;
                }
            }
        }
    }

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Release all the allocated CD3D11EnumAdapterInfo objects and empty the list
//--------------------------------------------------------------------------------------
void CD3D11Enumeration::ClearAdapterInfoList()
{
    CD3D11EnumAdapterInfo* pAdapterInfo;
    for( int i = 0; i < m_AdapterInfoList.GetSize(); i++ )
    {
        pAdapterInfo = m_AdapterInfoList.GetAt( i );
        delete pAdapterInfo;
    }

    m_AdapterInfoList.RemoveAll();
}


//--------------------------------------------------------------------------------------
void CD3D11Enumeration::ResetPossibleDepthStencilFormats()
{
    m_DepthStencilPossibleList.RemoveAll();
    m_DepthStencilPossibleList.Add( DXGI_FORMAT_D32_FLOAT_S8X24_UINT );
    m_DepthStencilPossibleList.Add( DXGI_FORMAT_D32_FLOAT );
    m_DepthStencilPossibleList.Add( DXGI_FORMAT_D24_UNORM_S8_UINT );
    m_DepthStencilPossibleList.Add( DXGI_FORMAT_D16_UNORM );
}

//--------------------------------------------------------------------------------------
void CD3D11Enumeration::SetEnumerateAllAdapterFormats( bool bEnumerateAllAdapterFormats )
{
    m_bEnumerateAllAdapterFormats = bEnumerateAllAdapterFormats;
}


//--------------------------------------------------------------------------------------
void CD3D11Enumeration::BuildMultiSampleQualityList( DXGI_FORMAT fmt, CD3D11EnumDeviceSettingsCombo* pDeviceCombo )
{
    ID3D11Device* pd3dDevice = NULL;
    ID3D11DeviceContext* pd3dDeviceContext = NULL;
    IDXGIAdapter* pAdapter = NULL;
    
    //if( pDeviceCombo->DeviceType == D3D_DRIVER_TYPE_HARDWARE )
    //    DXUTGetDXGIFactory()->EnumAdapters( pDeviceCombo->pAdapterInfo->AdapterOrdinal, &pAdapter );

    //DXGI_ADAPTER_DESC dad;
    //pAdapter->GetDesc(&dad);

    D3D_FEATURE_LEVEL *FeatureLevels = &(pDeviceCombo->pDeviceInfo->SelectedLevel);
    D3D_FEATURE_LEVEL returnedFeatureLevel;

    UINT NumFeatureLevels = 1;

    HRESULT hr = DXUT_Dynamic_D3D11CreateDevice( pAdapter, 
                                                pDeviceCombo->DeviceType,
                                                ( HMODULE )0,
                                                0,
                                                FeatureLevels,
                                                NumFeatureLevels,
                                                D3D11_SDK_VERSION,
                                                &pd3dDevice,
                                                &returnedFeatureLevel,
                                                &pd3dDeviceContext )  ;

    if( FAILED( hr))  return;

    if (returnedFeatureLevel != pDeviceCombo->pDeviceInfo->SelectedLevel) return;

    for( int i = 1; i <= D3D11_MAX_MULTISAMPLE_SAMPLE_COUNT; ++i )
    {
        UINT Quality;
        if( SUCCEEDED( pd3dDevice->CheckMultisampleQualityLevels( fmt, i, &Quality ) ) && Quality > 0 )
        {
            //From D3D10 docs: When multisampling a texture, the number of quality levels available for an adapter is dependent on the texture 
            //format used and the number of samples requested. The maximum sample count is defined by 
            //D3D10_MAX_MULTISAMPLE_SAMPLE_COUNT in d3d10.h. If the returned value of pNumQualityLevels is 0, 
            //the format and sample count combination is not supported for the installed adapter.

            if (Quality != 0) {
                pDeviceCombo->multiSampleCountList.Add( i );
                pDeviceCombo->multiSampleQualityList.Add( Quality );
            }
        }
    }

    SAFE_RELEASE( pAdapter );
    SAFE_RELEASE( pd3dDevice );
    SAFE_RELEASE (pd3dDeviceContext);
}


//--------------------------------------------------------------------------------------
// Call GetAdapterInfoList() after Enumerate() to get a STL vector of 
//       CD3D11EnumAdapterInfo* 
//--------------------------------------------------------------------------------------
CGrowableArray <CD3D11EnumAdapterInfo*>* CD3D11Enumeration::GetAdapterInfoList()
{
    return &m_AdapterInfoList;
}


//--------------------------------------------------------------------------------------
CD3D11EnumAdapterInfo* CD3D11Enumeration::GetAdapterInfo( UINT AdapterOrdinal )
{
    for( int iAdapter = 0; iAdapter < m_AdapterInfoList.GetSize(); iAdapter++ )
    {
        CD3D11EnumAdapterInfo* pAdapterInfo = m_AdapterInfoList.GetAt( iAdapter );
        if( pAdapterInfo->AdapterOrdinal == AdapterOrdinal )
            return pAdapterInfo;
    }

    return NULL;
}


//--------------------------------------------------------------------------------------
CD3D11EnumDeviceInfo* CD3D11Enumeration::GetDeviceInfo( UINT AdapterOrdinal, D3D_DRIVER_TYPE DeviceType )
{
    CD3D11EnumAdapterInfo* pAdapterInfo = GetAdapterInfo( AdapterOrdinal );
    if( pAdapterInfo )
    {
        for( int iDeviceInfo = 0; iDeviceInfo < pAdapterInfo->deviceInfoList.GetSize(); iDeviceInfo++ )
        {
            CD3D11EnumDeviceInfo* pDeviceInfo = pAdapterInfo->deviceInfoList.GetAt( iDeviceInfo );
            if( pDeviceInfo->DeviceType == DeviceType )
                return pDeviceInfo;
        }
    }

    return NULL;
}


//--------------------------------------------------------------------------------------
CD3D11EnumOutputInfo* CD3D11Enumeration::GetOutputInfo( UINT AdapterOrdinal, UINT Output )
{
    CD3D11EnumAdapterInfo* pAdapterInfo = GetAdapterInfo( AdapterOrdinal );
    if( pAdapterInfo && pAdapterInfo->outputInfoList.GetSize() > int( Output ) )
    {
        return pAdapterInfo->outputInfoList.GetAt( Output );
    }

    return NULL;
}


//--------------------------------------------------------------------------------------
CD3D11EnumDeviceSettingsCombo* CD3D11Enumeration::GetDeviceSettingsCombo( UINT AdapterOrdinal,
                                                                          D3D_DRIVER_TYPE DeviceType, UINT Output,
                                                                          DXGI_FORMAT BackBufferFormat, BOOL Windowed )
{
    CD3D11EnumAdapterInfo* pAdapterInfo = GetAdapterInfo( AdapterOrdinal );
    if( pAdapterInfo )
    {
        for( int iDeviceCombo = 0; iDeviceCombo < pAdapterInfo->deviceSettingsComboList.GetSize(); iDeviceCombo++ )
        {
            CD3D11EnumDeviceSettingsCombo* pDeviceSettingsCombo = pAdapterInfo->deviceSettingsComboList.GetAt(
                iDeviceCombo );
            if( pDeviceSettingsCombo->BackBufferFormat == BackBufferFormat &&
                pDeviceSettingsCombo->Windowed == Windowed )
                return pDeviceSettingsCombo;
        }
    }

    return NULL;
}


//--------------------------------------------------------------------------------------
CD3D11EnumOutputInfo::~CD3D11EnumOutputInfo( void )
{
    SAFE_RELEASE( m_pOutput );
    displayModeList.RemoveAll();
}


//--------------------------------------------------------------------------------------
CD3D11EnumDeviceInfo::~CD3D11EnumDeviceInfo()
{
}


//--------------------------------------------------------------------------------------
CD3D11EnumAdapterInfo::~CD3D11EnumAdapterInfo( void )
{
    for( int i = 0; i < outputInfoList.GetSize(); i++ )
    {
        CD3D11EnumOutputInfo* pOutputInfo = outputInfoList.GetAt( i );
        delete pOutputInfo;
    }
    outputInfoList.RemoveAll();

    for( int i = 0; i < deviceInfoList.GetSize(); ++i )
    {
        CD3D11EnumDeviceInfo* pDeviceInfo = deviceInfoList.GetAt( i );
        delete pDeviceInfo;
    }
    deviceInfoList.RemoveAll();

    for( int i = 0; i < deviceSettingsComboList.GetSize(); ++i )
    {
        CD3D11EnumDeviceSettingsCombo* pDeviceCombo = deviceSettingsComboList.GetAt( i );
        delete pDeviceCombo;
    }
    deviceSettingsComboList.RemoveAll();

    SAFE_RELEASE( m_pAdapter );
}

//--------------------------------------------------------------------------------------
// Returns the number of color channel bits in the specified DXGI_FORMAT
//--------------------------------------------------------------------------------------
UINT WINAPI DXUTGetDXGIColorChannelBits( DXGI_FORMAT fmt )
{
    switch( fmt )
    {
        case DXGI_FORMAT_R32G32B32A32_TYPELESS:
        case DXGI_FORMAT_R32G32B32A32_FLOAT:
        case DXGI_FORMAT_R32G32B32A32_UINT:
        case DXGI_FORMAT_R32G32B32A32_SINT:
        case DXGI_FORMAT_R32G32B32_TYPELESS:
        case DXGI_FORMAT_R32G32B32_FLOAT:
        case DXGI_FORMAT_R32G32B32_UINT:
        case DXGI_FORMAT_R32G32B32_SINT:
            return 32;

        case DXGI_FORMAT_R16G16B16A16_TYPELESS:
        case DXGI_FORMAT_R16G16B16A16_FLOAT:
        case DXGI_FORMAT_R16G16B16A16_UNORM:
        case DXGI_FORMAT_R16G16B16A16_UINT:
        case DXGI_FORMAT_R16G16B16A16_SNORM:
        case DXGI_FORMAT_R16G16B16A16_SINT:
            return 16;

        case DXGI_FORMAT_R10G10B10A2_TYPELESS:
        case DXGI_FORMAT_R10G10B10A2_UNORM:
        case DXGI_FORMAT_R10G10B10A2_UINT:
            return 10;

        case DXGI_FORMAT_R8G8B8A8_TYPELESS:
        case DXGI_FORMAT_R8G8B8A8_UNORM:
        case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
        case DXGI_FORMAT_R8G8B8A8_UINT:
        case DXGI_FORMAT_R8G8B8A8_SNORM:
        case DXGI_FORMAT_R8G8B8A8_SINT:
            return 8;

        case DXGI_FORMAT_B5G6R5_UNORM:
        case DXGI_FORMAT_B5G5R5A1_UNORM:
            return 5;

        default:
            return 0;
    }
}

//--------------------------------------------------------------------------------------
// Returns a ranking number that describes how closely this device 
// combo matches the optimal combo based on the match options and the optimal device settings
//--------------------------------------------------------------------------------------
float DXUTRankD3D11DeviceCombo( CD3D11EnumDeviceSettingsCombo* pDeviceSettingsCombo,
                                DXUTD3D11DeviceSettings* pOptimalDeviceSettings,
                                DXGI_MODE_DESC* pAdapterDisplayMode,
                                int &bestModeIndex,
                                int &bestMSAAIndex
                                )
{
    float fCurRanking = 0.0f;

    // Arbitrary weights.  Gives preference to the ordinal, device type, and windowed
    const float fAdapterOrdinalWeight   = 1000.0f;
    const float fAdapterOutputWeight    = 500.0f;
    const float fDeviceTypeWeight       = 100.0f;
    const float fWARPOverRefWeight       = 80.0f;

    const float fWindowWeight           = 10.0f;
    const float fResolutionWeight       = 1.0f;
    const float fBackBufferFormatWeight = 1.0f;
    const float fMultiSampleWeight      = 1.0f;
    const float fRefreshRateWeight      = 1.0f;

    //---------------------
    // Adapter ordinal
    //---------------------
    if( pDeviceSettingsCombo->AdapterOrdinal == pOptimalDeviceSettings->AdapterOrdinal )
        fCurRanking += fAdapterOrdinalWeight;

    //---------------------
    // Adapter ordinal
    //---------------------
    if( pDeviceSettingsCombo->Output == pOptimalDeviceSettings->Output )
        fCurRanking += fAdapterOutputWeight;

    //---------------------
    // Device type
    //---------------------
    if( pDeviceSettingsCombo->DeviceType == pOptimalDeviceSettings->DriverType )
        fCurRanking += fDeviceTypeWeight;
    else if (pDeviceSettingsCombo->DeviceType == D3D_DRIVER_TYPE_WARP && pOptimalDeviceSettings->DriverType == D3D_DRIVER_TYPE_HARDWARE) {
        fCurRanking += fWARPOverRefWeight;
    }

    // Slightly prefer HAL 
    if( pDeviceSettingsCombo->DeviceType == D3DDEVTYPE_HAL )
        fCurRanking += 0.1f;

    //---------------------
    // Windowed
    //---------------------
    if( pDeviceSettingsCombo->Windowed == pOptimalDeviceSettings->sd.Windowed )
        fCurRanking += fWindowWeight;

    //---------------------
    // Resolution
    //---------------------
    bool bResolutionFound = false;
    unsigned int best = 0xffffffff;
    bestModeIndex=0;
    for( int idm = 0; pDeviceSettingsCombo->pOutputInfo != NULL && idm < pDeviceSettingsCombo->pOutputInfo->displayModeList.GetSize() && !bResolutionFound; idm++ )
    {
        DXGI_MODE_DESC displayMode = pDeviceSettingsCombo->pOutputInfo->displayModeList.GetAt( idm );
        if( displayMode.Width == pOptimalDeviceSettings->sd.BufferDesc.Width &&
            displayMode.Height == pOptimalDeviceSettings->sd.BufferDesc.Height )
            bResolutionFound = true;

        unsigned int current = 
            (UINT) abs ((int)displayMode.Width  - (int)pOptimalDeviceSettings->sd.BufferDesc.Width) + 
            (UINT) abs ((int)displayMode.Height - (int)pOptimalDeviceSettings->sd.BufferDesc.Height );

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
    if( pDeviceSettingsCombo->BackBufferFormat == pOptimalDeviceSettings->sd.BufferDesc.Format )
    {
        fCurRanking += fBackBufferFormatWeight;
    }
    else
    {
        int nBitDepthDelta = abs( ( long )DXUTGetDXGIColorChannelBits( pDeviceSettingsCombo->BackBufferFormat ) -
                                  ( long )DXUTGetDXGIColorChannelBits(
                                  pOptimalDeviceSettings->sd.BufferDesc.Format ) );
        float fScale = __max( 0.9f - ( float )nBitDepthDelta * 0.2f, 0.0f );
        fCurRanking += fScale * fBackBufferFormatWeight;
    }

    //---------------------
    // Back buffer count
    //---------------------
    // No caps for the back buffer count

    //---------------------
    // Multisample
    //---------------------
    bool bMultiSampleFound = false;
    bestMSAAIndex = 0;
    for( int i = 0; i < pDeviceSettingsCombo->multiSampleCountList.GetSize(); i++ )
    {
        UINT Count = pDeviceSettingsCombo->multiSampleCountList.GetAt( i );

        if( Count == pOptimalDeviceSettings->sd.SampleDesc.Count  )
        {
            bestMSAAIndex = i;
            bMultiSampleFound = true;
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
    // No caps for swap effects

    //---------------------
    // Present flags
    //---------------------
    // No caps for the present flags

    //---------------------
    // Refresh rate
    //---------------------
    bool bRefreshFound = false;
    for( int idm = 0; pDeviceSettingsCombo->pOutputInfo != NULL && idm < pDeviceSettingsCombo->pOutputInfo->displayModeList.GetSize(); idm++ )
    {
        DXGI_MODE_DESC displayMode = pDeviceSettingsCombo->pOutputInfo->displayModeList.GetAt( idm );
        if( fabs( float( displayMode.RefreshRate.Numerator ) / displayMode.RefreshRate.Denominator -
                  float( pOptimalDeviceSettings->sd.BufferDesc.RefreshRate.Numerator ) /
                  pOptimalDeviceSettings->sd.BufferDesc.RefreshRate.Denominator ) < 0.1f )
            bRefreshFound = true;
    }
    if( bRefreshFound )
        fCurRanking += fRefreshRateWeight;

    //---------------------
    // Present interval
    //---------------------
    // No caps for the present flags

    return fCurRanking;
}


//--------------------------------------------------------------------------------------
// Returns the DXGI_MODE_DESC struct for a given adapter and output 
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTGetD3D11AdapterDisplayMode( UINT AdapterOrdinal, UINT nOutput, DXGI_MODE_DESC* pModeDesc )
{
    if( !pModeDesc )
        return E_INVALIDARG;

    CD3D11Enumeration* pD3DEnum = DXUTGetD3D11Enumeration();
    CD3D11EnumOutputInfo* pOutputInfo = pD3DEnum->GetOutputInfo( AdapterOrdinal, nOutput );
    if( pOutputInfo )
    {
        pModeDesc->Width = 640;
        pModeDesc->Height = 480;
        pModeDesc->RefreshRate.Numerator = 60;
        pModeDesc->RefreshRate.Denominator = 1;
        pModeDesc->Format = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
        pModeDesc->Scaling = DXGI_MODE_SCALING_UNSPECIFIED;
        pModeDesc->ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;

        DXGI_OUTPUT_DESC Desc;
        pOutputInfo->m_pOutput->GetDesc( &Desc );
        pModeDesc->Width = Desc.DesktopCoordinates.right - Desc.DesktopCoordinates.left;
        pModeDesc->Height = Desc.DesktopCoordinates.bottom - Desc.DesktopCoordinates.top;
    }

    // TODO: verify this is needed
    if( pModeDesc->Format == DXGI_FORMAT_B8G8R8A8_UNORM )
        pModeDesc->Format = DXGI_FORMAT_R8G8B8A8_UNORM;

    return S_OK;
}
