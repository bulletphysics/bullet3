//--------------------------------------------------------------------------------------
// File: DXUTSettingsDlg.cpp
//
// Dialog for selection of device settings 
//
// Copyright (c) Microsoft Corporation. All rights reserved
//--------------------------------------------------------------------------------------
#include "DXUT.h"
#include "DXUTgui.h"
#include "DXUTsettingsDlg.h"
#undef min // use __min instead
#undef max // use __max instead


//--------------------------------------------------------------------------------------
// Internal functions forward declarations
//--------------------------------------------------------------------------------------
WCHAR*              DXUTAPIVersionToString( DXUTDeviceVersion version );
WCHAR*              DXUTPresentIntervalToString( UINT pi );
WCHAR*              DXUTMultisampleTypeToString( D3DMULTISAMPLE_TYPE MultiSampleType );
WCHAR*              DXUTD3DDeviceTypeToString( D3DDEVTYPE devType );
WCHAR*              DXUTD3DX11DeviceTypeToString( D3D_DRIVER_TYPE devType );
WCHAR*              DXUTVertexProcessingTypeToString( DWORD vpt );


HRESULT DXUTSnapDeviceSettingsToEnumDevice( DXUTDeviceSettings* pDeviceSettings, bool forceEnum, D3D_FEATURE_LEVEL forceFL = D3D_FEATURE_LEVEL(0)  );

//--------------------------------------------------------------------------------------
// Global state
//--------------------------------------------------------------------------------------
DXUTDeviceSettings  g_DeviceSettings;

CD3DSettingsDlg* WINAPI DXUTGetD3DSettingsDialog()
{
    // Using an accessor function gives control of the construction order
    static CD3DSettingsDlg dlg;
    return &dlg;
}


//--------------------------------------------------------------------------------------
CD3DSettingsDlg::CD3DSettingsDlg()
{
    m_pStateBlock = NULL;
    m_bActive = false;
    m_pActiveDialog = NULL;
    
    m_Levels[0] = D3D_FEATURE_LEVEL_9_1;
    m_Levels[1] = D3D_FEATURE_LEVEL_9_2;
    m_Levels[2] = D3D_FEATURE_LEVEL_9_3;
    m_Levels[3] = D3D_FEATURE_LEVEL_10_0;
    m_Levels[4] = D3D_FEATURE_LEVEL_10_1;
    m_Levels[5] = D3D_FEATURE_LEVEL_11_0;
    
}


//--------------------------------------------------------------------------------------
CD3DSettingsDlg::~CD3DSettingsDlg()
{
    // Release the memory used to hold the D3D11 refresh data in the combo box
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_REFRESH_RATE );
    if( pComboBox )
        for( UINT i = 0; i < pComboBox->GetNumItems(); ++i )
        {
            DXGI_RATIONAL* pRate = reinterpret_cast<DXGI_RATIONAL*>( pComboBox->GetItemData( i ) );
            delete pRate;
        }
}


//--------------------------------------------------------------------------------------
void CD3DSettingsDlg::Init( CDXUTDialogResourceManager* pManager )
{
    assert( pManager );
    m_Dialog.Init( pManager, false );  // Don't register this dialog.
    m_RevertModeDialog.Init( pManager, false ); // Don't register this dialog.
    m_pActiveDialog = &m_Dialog;
    CreateControls();
}

//--------------------------------------------------------------------------------------
void CD3DSettingsDlg::Init( CDXUTDialogResourceManager* pManager, LPCWSTR szControlTextureFileName )
{
    assert( pManager );
    m_Dialog.Init( pManager, false, szControlTextureFileName );  // Don't register this dialog.
    m_RevertModeDialog.Init( pManager, false, szControlTextureFileName );   // Don't register this dialog.
    m_pActiveDialog = &m_Dialog;
    CreateControls();
}


//--------------------------------------------------------------------------------------
void CD3DSettingsDlg::Init( CDXUTDialogResourceManager* pManager, LPCWSTR pszControlTextureResourcename,
                            HMODULE hModule )
{
    assert( pManager );
    m_Dialog.Init( pManager, false, pszControlTextureResourcename, hModule );  // Don't register this dialog.
    m_RevertModeDialog.Init( pManager, false, pszControlTextureResourcename, hModule ); // Don't register this dialog
    m_pActiveDialog = &m_Dialog;
    CreateControls();
}


//--------------------------------------------------------------------------------------
void CD3DSettingsDlg::CreateControls()
{
    // Set up main settings dialog
    m_Dialog.EnableKeyboardInput( true );
    m_Dialog.SetFont( 0, L"Arial", 15, FW_NORMAL );
    m_Dialog.SetFont( 1, L"Arial", 28, FW_BOLD );

    // Right-justify static controls
    CDXUTElement* pElement = m_Dialog.GetDefaultElement( DXUT_CONTROL_STATIC, 0 );
    if( pElement )
    {
        pElement->dwTextFormat = DT_VCENTER | DT_RIGHT;

        // Title
        CDXUTStatic* pStatic = NULL;
        m_Dialog.AddStatic( DXUTSETTINGSDLG_STATIC, L"Direct3D Settings", 10, 5, 400, 50, false, &pStatic );
        pElement = pStatic->GetElement( 0 );
        pElement->iFont = 1;
        pElement->dwTextFormat = DT_TOP | DT_LEFT;
    }

    // DXUTSETTINGSDLG_API_VERSION
    m_Dialog.AddStatic( DXUTSETTINGSDLG_STATIC, L"API Version", 10, 35, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_API_VERSION, 200, 35, 300, 23 );


    //DXUTSETTINGSDLG_D3D11_FEATURE_LEVEL
    m_Dialog.AddStatic( DXUTSETTINGSDLG_D3D11_FEATURE_LEVEL_LABEL, L"Feature Level", 10, 60, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_D3D11_FEATURE_LEVEL, 200, 60, 300, 23 );
    m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_FEATURE_LEVEL )->SetDropHeight( 106 );


    // DXUTSETTINGSDLG_ADAPTER
    m_Dialog.AddStatic( DXUTSETTINGSDLG_STATIC, L"Display Adapter", 10, 85, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_ADAPTER, 200, 85, 300, 23 );

    // DXUTSETTINGSDLG_DEVICE_TYPE
    m_Dialog.AddStatic( DXUTSETTINGSDLG_STATIC, L"Render Device", 10, 110, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_DEVICE_TYPE, 200, 110, 300, 23 );

    // DXUTSETTINGSDLG_WINDOWED, DXUTSETTINGSDLG_FULLSCREEN
    m_Dialog.AddCheckBox( DXUTSETTINGSDLG_DEVICECLIP, L"Clip to device when window spans across multiple monitors",
                          250, 136, 500, 16 );
    m_Dialog.AddRadioButton( DXUTSETTINGSDLG_WINDOWED, DXUTSETTINGSDLG_WINDOWED_GROUP, L"Windowed", 
                          360, 157, 100, 16 );
    m_Dialog.AddRadioButton( DXUTSETTINGSDLG_FULLSCREEN, DXUTSETTINGSDLG_WINDOWED_GROUP, L"Full Screen",
                          220, 157, 100, 16 );

    // DXUTSETTINGSDLG_ADAPTER_FORMAT
    m_Dialog.AddStatic( DXUTSETTINGSDLG_ADAPTER_FORMAT_LABEL, L"Adapter Format", 10, 175, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_ADAPTER_FORMAT, 200, 175, 300, 23 );

    // DXUTSETTINGSDLG_RESOLUTION
    m_Dialog.AddStatic( DXUTSETTINGSDLG_RESOLUTION_LABEL, L"Resolution", 10, 200, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_RESOLUTION, 200, 200, 200, 23 );
    m_Dialog.GetComboBox( DXUTSETTINGSDLG_RESOLUTION )->SetDropHeight( 106 );

    // DXUTSETTINGSDLG_RES_SHOW_ALL
    m_Dialog.AddCheckBox( DXUTSETTINGSDLG_RESOLUTION_SHOW_ALL, L"Show All Aspect Ratios", 420, 200, 200, 23, false );

    // DXUTSETTINGSDLG_REFRESH_RATE
    m_Dialog.AddStatic( DXUTSETTINGSDLG_REFRESH_RATE_LABEL, L"Refresh Rate", 10, 225, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_REFRESH_RATE, 200, 225, 300, 23 );

    // DXUTSETTINGSDLG_BACK_BUFFER_FORMAT
    m_Dialog.AddStatic( DXUTSETTINGSDLG_BACK_BUFFER_FORMAT_LABEL, L"Back Buffer Format", 10, 260, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_BACK_BUFFER_FORMAT, 200, 260, 300, 23 );

    // DXUTSETTINGSDLG_DEPTH_STENCIL
    m_Dialog.AddStatic( DXUTSETTINGSDLG_DEPTH_STENCIL_LABEL, L"Depth/Stencil Format", 10, 285, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_DEPTH_STENCIL, 200, 285, 300, 23 );

    // DXUTSETTINGSDLG_MULTISAMPLE_TYPE
    m_Dialog.AddStatic( DXUTSETTINGSDLG_MULTISAMPLE_TYPE_LABEL, L"Multisample Type", 10, 310, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_MULTISAMPLE_TYPE, 200, 310, 300, 23 );

    // DXUTSETTINGSDLG_MULTISAMPLE_QUALITY
    m_Dialog.AddStatic( DXUTSETTINGSDLG_MULTISAMPLE_QUALITY_LABEL, L"Multisample Quality", 10, 335, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_MULTISAMPLE_QUALITY, 200, 335, 300, 23 );

    // DXUTSETTINGSDLG_VERTEX_PROCESSING
    m_Dialog.AddStatic( DXUTSETTINGSDLG_VERTEX_PROCESSING_LABEL, L"Vertex Processing", 10, 360, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_VERTEX_PROCESSING, 200, 360, 300, 23 );

    // DXUTSETTINGSDLG_PRESENT_INTERVAL
    m_Dialog.AddStatic( DXUTSETTINGSDLG_PRESENT_INTERVAL_LABEL, L"Vertical Sync", 10, 385, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_PRESENT_INTERVAL, 200, 385, 300, 23 );

    // DXUTSETTINGSDLG_D3D11_ADAPTER_OUTPUT
    m_Dialog.AddStatic( DXUTSETTINGSDLG_D3D11_ADAPTER_OUTPUT_LABEL, L"Adapter Output", 10, 175, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_D3D11_ADAPTER_OUTPUT, 200, 175, 300, 23 );

    // DXUTSETTINGSDLG_D3D11_RESOLUTION
    m_Dialog.AddStatic( DXUTSETTINGSDLG_D3D11_RESOLUTION_LABEL, L"Resolution", 10, 200, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_D3D11_RESOLUTION, 200, 200, 200, 23 );
    m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_RESOLUTION )->SetDropHeight( 106 );

    // DXUTSETTINGSDLG_D3D11_REFRESH_RATE
    m_Dialog.AddStatic( DXUTSETTINGSDLG_D3D11_REFRESH_RATE_LABEL, L"Refresh Rate", 10, 225, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_D3D11_REFRESH_RATE, 200, 225, 300, 23 );

    // DXUTSETTINGSDLG_D3D11_BACK_BUFFER_FORMAT
    m_Dialog.AddStatic( DXUTSETTINGSDLG_D3D11_BACK_BUFFER_FORMAT_LABEL, L"Back Buffer Format", 10, 260, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_D3D11_BACK_BUFFER_FORMAT, 200, 260, 300, 23 );

    // DXUTSETTINGSDLG_D3D11_MULTISAMPLE_COUNT
    m_Dialog.AddStatic( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_COUNT_LABEL, L"Multisample Count", 10, 285, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_COUNT, 200, 285, 300, 23 );

    // DXUTSETTINGSDLG_D3D11_MULTISAMPLE_QUALITY
    m_Dialog.AddStatic( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_QUALITY_LABEL, L"Multisample Quality", 10, 310, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_QUALITY, 200, 310, 300, 23 );

    // DXUTSETTINGSDLG_D3D11_PRESENT_INTERVAL
    m_Dialog.AddStatic( DXUTSETTINGSDLG_D3D11_PRESENT_INTERVAL_LABEL, L"Vertical Sync", 10, 335, 180, 23 );
    m_Dialog.AddComboBox( DXUTSETTINGSDLG_D3D11_PRESENT_INTERVAL, 200, 335, 300, 23 );

    // DXUTSETTINGSDLG_D3D11_DEBUG_DEVICE
    m_Dialog.AddCheckBox( DXUTSETTINGSDLG_D3D11_DEBUG_DEVICE, L"Create Debug Device", 200, 365, 180, 23 );

    // DXUTSETTINGSDLG_OK, DXUTSETTINGSDLG_CANCEL
    m_Dialog.AddButton( DXUTSETTINGSDLG_OK, L"OK", 230, 440, 73, 31 );
    m_Dialog.AddButton( DXUTSETTINGSDLG_CANCEL, L"Cancel", 315, 440, 73, 31, 0, true );

    // Set up mode change dialog
    m_RevertModeDialog.EnableKeyboardInput( true );
    m_RevertModeDialog.EnableNonUserEvents( true );
    m_RevertModeDialog.SetFont( 0, L"Arial", 15, FW_NORMAL );
    m_RevertModeDialog.SetFont( 1, L"Arial", 28, FW_BOLD );

    pElement = m_RevertModeDialog.GetDefaultElement( DXUT_CONTROL_STATIC, 0 );
    if( pElement )
    {
        pElement->dwTextFormat = DT_VCENTER | DT_RIGHT;

        // Title
        CDXUTStatic* pStatic = NULL;
        m_RevertModeDialog.AddStatic( DXUTSETTINGSDLG_STATIC, L"Do you want to keep these display settings?", 10, 5,
                                      640, 50, false, &pStatic );
        pElement = pStatic->GetElement( 0 );
        pElement->iFont = 1;
        pElement->dwTextFormat = DT_TOP | DT_LEFT;

        // Timeout static text control
        m_RevertModeDialog.AddStatic( DXUTSETTINGSDLG_STATIC_MODE_CHANGE_TIMEOUT, L"", 10, 90, 640, 30,
                                      false, &pStatic );
        pElement = pStatic->GetElement( 0 );
        pElement->iFont = 0;
        pElement->dwTextFormat = DT_TOP | DT_LEFT;
    }

    // DXUTSETTINGSDLG_MODE_CHANGE_ACCEPT, DXUTSETTINGSDLG_MODE_CHANGE_REVERT
    m_RevertModeDialog.AddButton( DXUTSETTINGSDLG_MODE_CHANGE_ACCEPT, L"Yes", 230, 50, 73, 31 );
    m_RevertModeDialog.AddButton( DXUTSETTINGSDLG_MODE_CHANGE_REVERT, L"No", 315, 50, 73, 31, 0, true );
}


//--------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnD3D9CreateDevice( IDirect3DDevice9* pd3dDevice )
{
    if( pd3dDevice == NULL )
        return DXUT_ERR_MSGBOX( L"CD3DSettingsDlg::OnCreatedDevice", E_INVALIDARG );

    // Create the fonts/textures 
    m_Dialog.SetCallback( StaticOnEvent, ( void* )this );
    m_RevertModeDialog.SetCallback( StaticOnEvent, ( void* )this );

    return S_OK;
}


//--------------------------------------------------------------------------------------
// Changes the UI defaults to the current device settings
//--------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::Refresh()
{
    HRESULT hr = S_OK;

    g_DeviceSettings = DXUTGetDeviceSettings();

    CDXUTComboBox* pAPIComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_API_VERSION );
    pAPIComboBox->RemoveAllItems();
    if( DXUTDoesAppSupportD3D9() )
    {
        // Ensure that at least one adapter got enumerated.
        CD3D9Enumeration* pD3DEnum = DXUTGetD3D9Enumeration();
        if( pD3DEnum->GetAdapterInfoList()->GetSize() > 0 )
            AddAPIVersion( DXUT_D3D9_DEVICE );
    }
    if( DXUTDoesAppSupportD3D11() && DXUTIsD3D11Available() )
    {
        // Ensure that at least one adapter got enumerated.
        CD3D11Enumeration* pD3DEnum = DXUTGetD3D11Enumeration();
        if( pD3DEnum->GetAdapterInfoList()->GetSize() > 0 )
            AddAPIVersion( DXUT_D3D11_DEVICE );
    }

    // If no API has been added, something has gone wrong.  Exit the dialog.
    if( pAPIComboBox->GetNumItems() == 0 )
    {
        SetActive( false );
        return S_OK;
    }

    pAPIComboBox->SetSelectedByData( ULongToPtr( g_DeviceSettings.ver ) );

    switch( g_DeviceSettings.ver )
    {
        case DXUT_D3D9_DEVICE:
        {
            // Show all D3D9-specific controls and hide controls for all other D3D versions.
            ShowControlSet( DXUT_D3D9_DEVICE );

            CD3D9Enumeration* pD3DEnum = DXUTGetD3D9Enumeration();

            // Fill the UI with the current settings
            AddDeviceType( g_DeviceSettings.d3d9.DeviceType );
            SetWindowed( FALSE != g_DeviceSettings.d3d9.pp.Windowed );
            SetDeviceClip( 0 != ( g_DeviceSettings.d3d9.pp.Flags & D3DPRESENTFLAG_DEVICECLIP ) );
            AddAdapterFormat( g_DeviceSettings.d3d9.AdapterFormat );
            AddResolution( g_DeviceSettings.d3d9.pp.BackBufferWidth, g_DeviceSettings.d3d9.pp.BackBufferHeight );
            AddRefreshRate( g_DeviceSettings.d3d9.pp.FullScreen_RefreshRateInHz );
            AddBackBufferFormat( g_DeviceSettings.d3d9.pp.BackBufferFormat );
            AddDepthStencilBufferFormat( g_DeviceSettings.d3d9.pp.AutoDepthStencilFormat );
            AddMultisampleType( g_DeviceSettings.d3d9.pp.MultiSampleType );
            AddMultisampleQuality( g_DeviceSettings.d3d9.pp.MultiSampleQuality );

            if( g_DeviceSettings.d3d9.BehaviorFlags & D3DCREATE_PUREDEVICE )
                AddVertexProcessingType( D3DCREATE_PUREDEVICE );
            else if( g_DeviceSettings.d3d9.BehaviorFlags & D3DCREATE_HARDWARE_VERTEXPROCESSING )
                AddVertexProcessingType( D3DCREATE_HARDWARE_VERTEXPROCESSING );
            else if( g_DeviceSettings.d3d9.BehaviorFlags & D3DCREATE_SOFTWARE_VERTEXPROCESSING )
                AddVertexProcessingType( D3DCREATE_SOFTWARE_VERTEXPROCESSING );
            else if( g_DeviceSettings.d3d9.BehaviorFlags & D3DCREATE_MIXED_VERTEXPROCESSING )
                AddVertexProcessingType( D3DCREATE_MIXED_VERTEXPROCESSING );

            CD3D9EnumDeviceSettingsCombo* pBestDeviceSettingsCombo = pD3DEnum->GetDeviceSettingsCombo(
                g_DeviceSettings.d3d9.AdapterOrdinal, g_DeviceSettings.d3d9.DeviceType,
                g_DeviceSettings.d3d9.AdapterFormat, g_DeviceSettings.d3d9.pp.BackBufferFormat,
                ( g_DeviceSettings.d3d9.pp.Windowed != 0 ) );
            if( NULL == pBestDeviceSettingsCombo )
                return DXUT_ERR_MSGBOX( L"GetDeviceSettingsCombo", E_INVALIDARG );

            // Get the adapters list from CD3D9Enumeration object
            CGrowableArray <CD3D9EnumAdapterInfo*>* pAdapterInfoList = pD3DEnum->GetAdapterInfoList();

            if( pAdapterInfoList->GetSize() == 0 )
                return DXUT_ERR_MSGBOX( L"CD3DSettingsDlg::OnCreatedDevice", DXUTERR_NOCOMPATIBLEDEVICES );

            CDXUTComboBox* pAdapterCombo = m_Dialog.GetComboBox( DXUTSETTINGSDLG_ADAPTER );
            pAdapterCombo->RemoveAllItems();

            // Add adapters
            for( int iAdapter = 0; iAdapter < pAdapterInfoList->GetSize(); iAdapter++ )
            {
                CD3D9EnumAdapterInfo* pAdapterInfo = pAdapterInfoList->GetAt( iAdapter );
                AddAdapter( pAdapterInfo->szUniqueDescription, pAdapterInfo->AdapterOrdinal );
            }

            pAdapterCombo->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d9.AdapterOrdinal ) );

            hr = OnAPIVersionChanged( true );
            if( FAILED( hr ) )
                return hr;

            //m_Dialog.Refresh();
            CDXUTDialog::SetRefreshTime( ( float )DXUTGetTime() );
            break;
        }
        case DXUT_D3D11_DEVICE:
        {
            // Show all D3D11-specific controls and hide controls for all other D3D versions.
            ShowControlSet( DXUT_D3D11_DEVICE );

            CD3D11Enumeration* pD3DEnum = DXUTGetD3D11Enumeration();

            // Fill the UI with the current settings
            AddD3D11DeviceType( g_DeviceSettings.d3d11.DriverType );
            SetWindowed( FALSE != g_DeviceSettings.d3d11.sd.Windowed );
            CD3D11EnumOutputInfo* pOutputInfo = GetCurrentD3D11OutputInfo();
            AddD3D11AdapterOutput( pOutputInfo->Desc.DeviceName, g_DeviceSettings.d3d11.Output );
            


            AddD3D11Resolution( g_DeviceSettings.d3d11.sd.BufferDesc.Width,
                                g_DeviceSettings.d3d11.sd.BufferDesc.Height );
            AddD3D11RefreshRate( g_DeviceSettings.d3d11.sd.BufferDesc.RefreshRate );
            AddD3D11BackBufferFormat( g_DeviceSettings.d3d11.sd.BufferDesc.Format );
            AddD3D11MultisampleCount( g_DeviceSettings.d3d11.sd.SampleDesc.Count );
            AddD3D11MultisampleQuality( g_DeviceSettings.d3d11.sd.SampleDesc.Quality );

            CD3D11EnumDeviceSettingsCombo* pBestDeviceSettingsCombo = pD3DEnum->GetDeviceSettingsCombo(
                g_DeviceSettings.d3d11.AdapterOrdinal, g_DeviceSettings.d3d11.DriverType,
                g_DeviceSettings.d3d11.Output, g_DeviceSettings.d3d11.sd.BufferDesc.Format,
                ( g_DeviceSettings.d3d11.sd.Windowed != 0 ) );

            if( NULL == pBestDeviceSettingsCombo )
                return DXUT_ERR_MSGBOX( L"GetDeviceSettingsCombo", E_INVALIDARG );

            CDXUTComboBox *pFeatureLevelBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_FEATURE_LEVEL );
            pFeatureLevelBox->RemoveAllItems();
            for (int fli = 0; fli < TOTAL_FEATURE_LEVLES; fli++) {
                if (m_Levels[fli] >= g_DeviceSettings.MinimumFeatureLevel 
                    && m_Levels[fli] <=pBestDeviceSettingsCombo->pDeviceInfo->MaxLevel) {
                     AddD3D11FeatureLevel( m_Levels[fli] );
                }
            } 
            pFeatureLevelBox->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d11.DeviceFeatureLevel ) );

            
            // Get the adapters list from CD3D11Enumeration object
            CGrowableArray <CD3D11EnumAdapterInfo*>* pAdapterInfoList = pD3DEnum->GetAdapterInfoList();

            if( pAdapterInfoList->GetSize() == 0 )
                return DXUT_ERR_MSGBOX( L"CD3DSettingsDlg::OnCreatedDevice", DXUTERR_NOCOMPATIBLEDEVICES );

            CDXUTComboBox* pAdapterCombo = m_Dialog.GetComboBox( DXUTSETTINGSDLG_ADAPTER );
            pAdapterCombo->RemoveAllItems();

            // Add adapters
            for( int iAdapter = 0; iAdapter < pAdapterInfoList->GetSize(); iAdapter++ )
            {
                CD3D11EnumAdapterInfo* pAdapterInfo = pAdapterInfoList->GetAt( iAdapter );
                AddAdapter( pAdapterInfo->szUniqueDescription, pAdapterInfo->AdapterOrdinal );
            }

            pAdapterCombo->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d11.AdapterOrdinal ) );

            hr = OnAPIVersionChanged( true );
            if( FAILED( hr ) )
                return hr;

            //m_Dialog.Refresh();
            CDXUTDialog::SetRefreshTime( ( float )DXUTGetTime() );
            break;
        }
    }

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnD3D9ResetDevice()
{
    const D3DSURFACE_DESC* pDesc = DXUTGetD3D9BackBufferSurfaceDesc();
    m_Dialog.SetLocation( 0, 0 );
    m_Dialog.SetSize( pDesc->Width, pDesc->Height );
    m_Dialog.SetBackgroundColors( D3DCOLOR_ARGB( 255, 98, 138, 206 ),
                                  D3DCOLOR_ARGB( 255, 54, 105, 192 ),
                                  D3DCOLOR_ARGB( 255, 54, 105, 192 ),
                                  D3DCOLOR_ARGB( 255, 10, 73, 179 ) );

    m_RevertModeDialog.SetLocation( 0, 0 );
    m_RevertModeDialog.SetSize( pDesc->Width, pDesc->Height );
    m_RevertModeDialog.SetBackgroundColors( D3DCOLOR_ARGB( 255, 98, 138, 206 ),
                                            D3DCOLOR_ARGB( 255, 54, 105, 192 ),
                                            D3DCOLOR_ARGB( 255, 54, 105, 192 ),
                                            D3DCOLOR_ARGB( 255, 10, 73, 179 ) );

    IDirect3DDevice9* pd3dDevice = DXUTGetD3D9Device();
    pd3dDevice->BeginStateBlock();
    pd3dDevice->SetRenderState( D3DRS_FILLMODE, D3DFILL_SOLID );
    pd3dDevice->EndStateBlock( &m_pStateBlock );

    return S_OK;
}

//--------------------------------------------------------------------------------------
void CD3DSettingsDlg::SetSelectedD3D11RefreshRate( DXGI_RATIONAL RefreshRate )
{
    CDXUTComboBox* pRefreshRateComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_REFRESH_RATE );

    for( UINT i = 0; i < pRefreshRateComboBox->GetNumItems(); ++i )
    {
        DXGI_RATIONAL* pRate = ( DXGI_RATIONAL* )pRefreshRateComboBox->GetItemData( i );

        if( pRate && pRate->Numerator == RefreshRate.Numerator && pRate->Denominator == RefreshRate.Denominator )
        {
            pRefreshRateComboBox->SetSelectedByIndex( i );
            return;
        }
    }
}

//--------------------------------------------------------------------------------------
void CD3DSettingsDlg::OnRender( float fElapsedTime )
{
    if( DXUTGetD3D11Device() )
        OnRender11( fElapsedTime );
    else
        OnRender9( fElapsedTime );
}


//--------------------------------------------------------------------------------------
void CD3DSettingsDlg::OnRender9( float fElapsedTime )
{
    IDirect3DDevice9* pd3dDevice = DXUTGetD3D9Device();

    // Clear the render target and the zbuffer 
    pd3dDevice->Clear( 0, NULL, D3DCLEAR_TARGET, 0x00003F3F, 1.0f, 0 );

    // Render the scene
    if( SUCCEEDED( pd3dDevice->BeginScene() ) )
    {
        m_pStateBlock->Capture();
        pd3dDevice->SetRenderState( D3DRS_FILLMODE, D3DFILL_SOLID );
        m_pActiveDialog->OnRender( fElapsedTime );
        m_pStateBlock->Apply();
        pd3dDevice->EndScene();
    }
}



//--------------------------------------------------------------------------------------
void CD3DSettingsDlg::OnRender11( float fElapsedTime )
{
    // Render the scene
    m_pActiveDialog->OnRender( fElapsedTime );
}


//--------------------------------------------------------------------------------------
LRESULT CD3DSettingsDlg::MsgProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    m_pActiveDialog->MsgProc( hWnd, uMsg, wParam, lParam );
    if( uMsg == WM_KEYDOWN && wParam == VK_F2 )
        SetActive( false );
    return 0;
}


//--------------------------------------------------------------------------------------
void CD3DSettingsDlg::OnD3D9LostDevice()
{
    SAFE_RELEASE( m_pStateBlock );
}


//--------------------------------------------------------------------------------------
void CD3DSettingsDlg::OnD3D9DestroyDevice()
{
}



//--------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnD3D11CreateDevice( ID3D11Device* pd3dDevice )
{
    //HRESULT hr;

    if( pd3dDevice == NULL )
        return DXUT_ERR_MSGBOX( L"CD3DSettingsDlg::OnCreatedDevice", E_INVALIDARG );

    // Create the fonts/textures 
    m_Dialog.SetCallback( StaticOnEvent, ( void* )this );
    m_RevertModeDialog.SetCallback( StaticOnEvent, ( void* )this );

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnD3D11ResizedSwapChain( ID3D11Device* pd3dDevice,
                                                  const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc )
{
    m_Dialog.SetLocation( 0, 0 );
    m_Dialog.SetSize( pBackBufferSurfaceDesc->Width, pBackBufferSurfaceDesc->Height );
    m_Dialog.SetBackgroundColors( D3DCOLOR_ARGB( 255, 98, 138, 206 ),
                                  D3DCOLOR_ARGB( 255, 54, 105, 192 ),
                                  D3DCOLOR_ARGB( 255, 54, 105, 192 ),
                                  D3DCOLOR_ARGB( 255, 10, 73, 179 ) );

    m_RevertModeDialog.SetLocation( 0, 0 );
    m_RevertModeDialog.SetSize( pBackBufferSurfaceDesc->Width, pBackBufferSurfaceDesc->Height );
    m_RevertModeDialog.SetBackgroundColors( D3DCOLOR_ARGB( 255, 98, 138, 206 ),
                                            D3DCOLOR_ARGB( 255, 54, 105, 192 ),
                                            D3DCOLOR_ARGB( 255, 54, 105, 192 ),
                                            D3DCOLOR_ARGB( 255, 10, 73, 179 ) );

    return S_OK;
}


//--------------------------------------------------------------------------------------
void CD3DSettingsDlg::OnD3D11DestroyDevice()
{


}

//--------------------------------------------------------------------------------------
void CD3DSettingsDlg::ShowControlSet( DXUTDeviceVersion ver )
{
    switch( ver )
    {
        case DXUT_D3D9_DEVICE:
            
            m_Dialog.GetControl( DXUTSETTINGSDLG_ADAPTER_FORMAT )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_ADAPTER_FORMAT_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_RESOLUTION )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_RESOLUTION_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_REFRESH_RATE )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_REFRESH_RATE_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_BACK_BUFFER_FORMAT )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_BACK_BUFFER_FORMAT_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_DEPTH_STENCIL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_DEPTH_STENCIL_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_MULTISAMPLE_TYPE )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_MULTISAMPLE_TYPE_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_MULTISAMPLE_QUALITY )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_MULTISAMPLE_QUALITY_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_VERTEX_PROCESSING )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_VERTEX_PROCESSING_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_PRESENT_INTERVAL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_PRESENT_INTERVAL_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_DEVICECLIP )->SetVisible( true );
            
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_FEATURE_LEVEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_FEATURE_LEVEL_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_ADAPTER_OUTPUT )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_ADAPTER_OUTPUT_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_RESOLUTION )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_RESOLUTION_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_REFRESH_RATE )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_REFRESH_RATE_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_BACK_BUFFER_FORMAT )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_BACK_BUFFER_FORMAT_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_COUNT )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_COUNT_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_QUALITY )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_QUALITY_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_PRESENT_INTERVAL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_PRESENT_INTERVAL_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_DEBUG_DEVICE )->SetVisible( false );

            break;

        case DXUT_D3D11_DEVICE:
            m_Dialog.GetControl( DXUTSETTINGSDLG_ADAPTER_FORMAT )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_ADAPTER_FORMAT_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_FEATURE_LEVEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_FEATURE_LEVEL_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_RESOLUTION )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_RESOLUTION_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_REFRESH_RATE )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_REFRESH_RATE_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_BACK_BUFFER_FORMAT )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_BACK_BUFFER_FORMAT_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_DEPTH_STENCIL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_DEPTH_STENCIL_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_MULTISAMPLE_TYPE )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_MULTISAMPLE_TYPE_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_MULTISAMPLE_QUALITY )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_MULTISAMPLE_QUALITY_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_VERTEX_PROCESSING )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_VERTEX_PROCESSING_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_PRESENT_INTERVAL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_PRESENT_INTERVAL_LABEL )->SetVisible( false );
            m_Dialog.GetControl( DXUTSETTINGSDLG_DEVICECLIP )->SetVisible( false );

            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_ADAPTER_OUTPUT )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_ADAPTER_OUTPUT_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_RESOLUTION )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_RESOLUTION_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_REFRESH_RATE )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_REFRESH_RATE_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_BACK_BUFFER_FORMAT )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_BACK_BUFFER_FORMAT_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_COUNT )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_COUNT_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_QUALITY )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_QUALITY_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_PRESENT_INTERVAL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_PRESENT_INTERVAL_LABEL )->SetVisible( true );
            m_Dialog.GetControl( DXUTSETTINGSDLG_D3D11_DEBUG_DEVICE )->SetVisible( true );
            break;
    }
}


//--------------------------------------------------------------------------------------
void WINAPI CD3DSettingsDlg::StaticOnEvent( UINT nEvent, int nControlID,
                                            CDXUTControl* pControl, void* pUserData )
{
    CD3DSettingsDlg* pD3DSettings = ( CD3DSettingsDlg* )pUserData;
    if( pD3DSettings )
        pD3DSettings->OnEvent( nEvent, nControlID, pControl );
}

//--------------------------------------------------------------------------------------
// Name: CD3DSettingsDlg::StaticOnModeChangeTimer()
// Desc: Timer callback registered by a call to DXUTSetTimer.  It is called each second
//       until mode change timeout limit.
//--------------------------------------------------------------------------------------
void WINAPI CD3DSettingsDlg::StaticOnModeChangeTimer( UINT nIDEvent, void* pUserContext )
{
    CD3DSettingsDlg* pD3DSettings = ( CD3DSettingsDlg* )pUserContext;
    assert( pD3DSettings );
    assert( pD3DSettings->m_pActiveDialog == &pD3DSettings->m_RevertModeDialog );
    assert( pD3DSettings->m_nIDEvent == nIDEvent );

    if( 0 == --pD3DSettings->m_nRevertModeTimeout )
    {
        CDXUTControl* pControl = pD3DSettings->m_RevertModeDialog.GetControl( DXUTSETTINGSDLG_MODE_CHANGE_REVERT );
        assert( pControl );
        pD3DSettings->m_RevertModeDialog.SendEvent( EVENT_BUTTON_CLICKED, false, pControl );
    }
    pD3DSettings->UpdateModeChangeTimeoutText( pD3DSettings->m_nRevertModeTimeout );
}

//--------------------------------------------------------------------------------------
void CD3DSettingsDlg::OnEvent( UINT nEvent, int nControlID,
                               CDXUTControl* pControl )
{
    switch( nControlID )
    {
        case DXUTSETTINGSDLG_ADAPTER:
            OnAdapterChanged(); break;
        case DXUTSETTINGSDLG_DEVICE_TYPE:
            OnDeviceTypeChanged(); break;
        case DXUTSETTINGSDLG_WINDOWED:
            OnWindowedFullScreenChanged(); break;
        case DXUTSETTINGSDLG_FULLSCREEN:
            OnWindowedFullScreenChanged(); break;
        case DXUTSETTINGSDLG_ADAPTER_FORMAT:
            OnAdapterFormatChanged(); break;
        case DXUTSETTINGSDLG_RESOLUTION_SHOW_ALL:
        {
            if( g_DeviceSettings.ver == DXUT_D3D9_DEVICE )
            {
                OnAdapterFormatChanged();
            }
            else
            {
                OnBackBufferFormatChanged();
            }
            break;
        }
        case DXUTSETTINGSDLG_D3D11_RESOLUTION:
            OnD3D11ResolutionChanged(); break;
        case DXUTSETTINGSDLG_RESOLUTION:
            OnResolutionChanged(); break;
        case DXUTSETTINGSDLG_REFRESH_RATE:
            OnRefreshRateChanged(); break;
        case DXUTSETTINGSDLG_BACK_BUFFER_FORMAT:
            OnBackBufferFormatChanged(); break;
        case DXUTSETTINGSDLG_DEPTH_STENCIL:
            OnDepthStencilBufferFormatChanged(); break;
        case DXUTSETTINGSDLG_MULTISAMPLE_TYPE:
            OnMultisampleTypeChanged(); break;
        case DXUTSETTINGSDLG_MULTISAMPLE_QUALITY:
            OnMultisampleQualityChanged(); break;
        case DXUTSETTINGSDLG_VERTEX_PROCESSING:
            OnVertexProcessingChanged(); break;
        case DXUTSETTINGSDLG_PRESENT_INTERVAL:
            OnPresentIntervalChanged(); break;
        case DXUTSETTINGSDLG_DEVICECLIP:
            OnDeviceClipChanged(); break;
        case DXUTSETTINGSDLG_API_VERSION:
            OnAPIVersionChanged(); break;
        case DXUTSETTINGSDLG_D3D11_FEATURE_LEVEL:
            OnFeatureLevelChanged(); break;
        case DXUTSETTINGSDLG_D3D11_ADAPTER_OUTPUT:
            OnAdapterOutputChanged(); break;
        case DXUTSETTINGSDLG_D3D11_REFRESH_RATE:
            OnRefreshRateChanged(); break;
        case DXUTSETTINGSDLG_D3D11_BACK_BUFFER_FORMAT:
            OnBackBufferFormatChanged(); break;
        case DXUTSETTINGSDLG_D3D11_MULTISAMPLE_COUNT:
            OnMultisampleTypeChanged(); break;
        case DXUTSETTINGSDLG_D3D11_MULTISAMPLE_QUALITY:
            OnMultisampleQualityChanged(); break;
        case DXUTSETTINGSDLG_D3D11_PRESENT_INTERVAL:
            OnPresentIntervalChanged(); break;
        case DXUTSETTINGSDLG_D3D11_DEBUG_DEVICE:
            OnDebugDeviceChanged(); break;

        case DXUTSETTINGSDLG_OK:
        {
            bool bFullScreenModeChange = false;
            DXUTDeviceSettings currentSettings = DXUTGetDeviceSettings();
            g_DeviceSettings.MinimumFeatureLevel = currentSettings.MinimumFeatureLevel;
            if( g_DeviceSettings.ver == DXUT_D3D9_DEVICE )
            {
                if( g_DeviceSettings.d3d9.pp.Windowed )
                {
                    g_DeviceSettings.d3d9.pp.FullScreen_RefreshRateInHz = 0;

                    RECT rcClient;
                    if( DXUTIsWindowed() )
                        GetClientRect( DXUTGetHWND(), &rcClient );
                    else
                        rcClient = DXUTGetWindowClientRectAtModeChange();
                    DWORD dwWindowWidth = rcClient.right - rcClient.left;
                    DWORD dwWindowHeight = rcClient.bottom - rcClient.top;

                    g_DeviceSettings.d3d9.pp.BackBufferWidth = dwWindowWidth;
                    g_DeviceSettings.d3d9.pp.BackBufferHeight = dwWindowHeight;
                }
                else
                {
                    // Check for fullscreen mode change
                    bFullScreenModeChange = g_DeviceSettings.d3d9.pp.BackBufferWidth !=
                        currentSettings.d3d9.pp.BackBufferWidth ||
                        g_DeviceSettings.d3d9.pp.BackBufferHeight != currentSettings.d3d9.pp.BackBufferHeight ||
                        g_DeviceSettings.d3d9.pp.FullScreen_RefreshRateInHz !=
                        currentSettings.d3d9.pp.FullScreen_RefreshRateInHz;
                }

                if( g_DeviceSettings.d3d9.pp.MultiSampleType != D3DMULTISAMPLE_NONE )
                {
                    g_DeviceSettings.d3d9.pp.Flags &= ~D3DPRESENTFLAG_LOCKABLE_BACKBUFFER;
                }
            }
            else // D3D11
            {
                if( g_DeviceSettings.d3d11.sd.Windowed )
                {
                    g_DeviceSettings.d3d11.sd.BufferDesc.RefreshRate.Denominator =
                        g_DeviceSettings.d3d11.sd.BufferDesc.RefreshRate.Numerator = 0;

                    RECT rcClient;
                    if( DXUTIsWindowed() )
                        GetClientRect( DXUTGetHWND(), &rcClient );
                    else
                        rcClient = DXUTGetWindowClientRectAtModeChange();
                    DWORD dwWindowWidth = rcClient.right - rcClient.left;
                    DWORD dwWindowHeight = rcClient.bottom - rcClient.top;

                    g_DeviceSettings.d3d11.sd.BufferDesc.Width = dwWindowWidth;
                    g_DeviceSettings.d3d11.sd.BufferDesc.Height = dwWindowHeight;
                }
                else
                {
                    // Check for fullscreen mode change
                    bFullScreenModeChange = g_DeviceSettings.d3d11.sd.BufferDesc.Width !=
                        currentSettings.d3d11.sd.BufferDesc.Width ||
                        g_DeviceSettings.d3d11.sd.BufferDesc.Height != currentSettings.d3d11.sd.BufferDesc.Height ||
                        g_DeviceSettings.d3d11.sd.BufferDesc.RefreshRate.Denominator !=
                        currentSettings.d3d11.sd.BufferDesc.RefreshRate.Denominator ||
                        g_DeviceSettings.d3d11.sd.BufferDesc.RefreshRate.Numerator !=
                        currentSettings.d3d11.sd.BufferDesc.RefreshRate.Numerator;
                }
            }

            if( bFullScreenModeChange )
            {
                // set appropriate global device settings to that of the current device
                // settings.  These will get set to the user-defined settings once the
                // user accepts the mode change
                DXUTDeviceSettings tSettings = g_DeviceSettings;
                if( g_DeviceSettings.ver == DXUT_D3D9_DEVICE )
                {
                    g_DeviceSettings.d3d9.pp.BackBufferWidth = 
                        currentSettings.d3d9.pp.BackBufferWidth;
                    g_DeviceSettings.d3d9.pp.BackBufferHeight = 
                        currentSettings.d3d9.pp.BackBufferHeight;
                    g_DeviceSettings.d3d9.pp.FullScreen_RefreshRateInHz =
                        currentSettings.d3d9.pp.FullScreen_RefreshRateInHz;
                    g_DeviceSettings.d3d9.pp.Windowed = 
                        currentSettings.d3d9.pp.Windowed;
                }
                else
                {
                    
                    g_DeviceSettings.d3d11.sd.BufferDesc.Width = 
                        currentSettings.d3d11.sd.BufferDesc.Width;
                    g_DeviceSettings.d3d11.sd.BufferDesc.Height = 
                        currentSettings.d3d11.sd.BufferDesc.Height;
                    g_DeviceSettings.d3d11.sd.BufferDesc.RefreshRate.Denominator =
                        currentSettings.d3d11.sd.BufferDesc.RefreshRate.Denominator;
                    g_DeviceSettings.d3d11.sd.BufferDesc.RefreshRate.Numerator =
                        currentSettings.d3d11.sd.BufferDesc.RefreshRate.Numerator;
                    g_DeviceSettings.d3d11.sd.Windowed = currentSettings.d3d11.sd.Windowed;
                
                }

                // apply the user-defined settings
                DXUTCreateDeviceFromSettings( &tSettings );
                // create the mode change timeout dialog
                m_pActiveDialog = &m_RevertModeDialog;
                m_nRevertModeTimeout = 15;
                UpdateModeChangeTimeoutText( m_nRevertModeTimeout );
                // activate a timer for 1-second updates
                DXUTSetTimer( StaticOnModeChangeTimer, 1.0f, &m_nIDEvent, ( void* )this );
            }
            else
            {
                DXUTCreateDeviceFromSettings( &g_DeviceSettings );
                SetActive( false );
            }
            break;
        }

        case DXUTSETTINGSDLG_CANCEL:
        {
            SetActive( false );
            break;
        }

        case DXUTSETTINGSDLG_MODE_CHANGE_ACCEPT:
        {
            DXUTKillTimer( m_nIDEvent );
            g_DeviceSettings = DXUTGetDeviceSettings();
            m_pActiveDialog = &m_Dialog;
            SetActive( false );
            break;
        }

        case DXUTSETTINGSDLG_MODE_CHANGE_REVERT:
        {
            DXUTKillTimer( m_nIDEvent );
            m_pActiveDialog = &m_Dialog;
            m_nIDEvent = 0;
            m_nRevertModeTimeout = 0;
            DXUTCreateDeviceFromSettings( &g_DeviceSettings );
            Refresh();
            break;
        }
    }
}


//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::SetDeviceSettingsFromUI()
{
    CDXUTComboBox* pComboBox;
    CDXUTRadioButton* pRadioButton;

    // DXUTSETTINGSDLG_DEVICE_TYPE
    pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_DEVICE_TYPE );
    g_DeviceSettings.d3d9.DeviceType = ( D3DDEVTYPE )PtrToUlong( pComboBox->GetSelectedData() );

    // DXUTSETTINGSDLG_WINDOWED
    pRadioButton = m_Dialog.GetRadioButton( DXUTSETTINGSDLG_WINDOWED );
    g_DeviceSettings.d3d9.pp.Windowed = pRadioButton->GetChecked();

    // DXUTSETTINGSDLG_ADAPTER_FORMAT
    pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_ADAPTER_FORMAT );
    g_DeviceSettings.d3d9.AdapterFormat = ( D3DFORMAT )PtrToUlong( pComboBox->GetSelectedData() );

    if( g_DeviceSettings.d3d9.pp.Windowed )
    {
        g_DeviceSettings.d3d9.pp.BackBufferFormat = D3DFMT_UNKNOWN;
        g_DeviceSettings.d3d9.pp.FullScreen_RefreshRateInHz = 0;
    }
    else
    {
        // DXUTSETTINGSDLG_BACK_BUFFER_FORMAT
        pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_BACK_BUFFER_FORMAT );
        g_DeviceSettings.d3d9.pp.BackBufferFormat = ( D3DFORMAT )PtrToUlong( pComboBox->GetSelectedData() );

        // DXUTSETTINGSDLG_RESOLUTION
        pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_RESOLUTION );
        DWORD dwResolution = PtrToUlong( pComboBox->GetSelectedData() );
        g_DeviceSettings.d3d9.pp.BackBufferWidth = HIWORD( dwResolution );
        g_DeviceSettings.d3d9.pp.BackBufferHeight = LOWORD( dwResolution );

        // DXUTSETTINGSDLG_REFRESH_RATE
        pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_REFRESH_RATE );
        g_DeviceSettings.d3d9.pp.FullScreen_RefreshRateInHz = PtrToUlong( pComboBox->GetSelectedData() );
    }

    // DXUTSETTINGSDLG_DEPTH_STENCIL
    pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_DEPTH_STENCIL );
    g_DeviceSettings.d3d9.pp.AutoDepthStencilFormat = ( D3DFORMAT )PtrToUlong( pComboBox->GetSelectedData() );

    return S_OK;
}


//-------------------------------------------------------------------------------------
CD3D9EnumAdapterInfo* CD3DSettingsDlg::GetCurrentAdapterInfo()
{
    CD3D9Enumeration* pD3DEnum = DXUTGetD3D9Enumeration();
    return pD3DEnum->GetAdapterInfo( g_DeviceSettings.d3d9.AdapterOrdinal );
}


//-------------------------------------------------------------------------------------
CD3D9EnumDeviceInfo* CD3DSettingsDlg::GetCurrentDeviceInfo()
{
    CD3D9Enumeration* pD3DEnum = DXUTGetD3D9Enumeration();
    return pD3DEnum->GetDeviceInfo( g_DeviceSettings.d3d9.AdapterOrdinal,
                                    g_DeviceSettings.d3d9.DeviceType );
}

//-------------------------------------------------------------------------------------
CD3D11EnumAdapterInfo* CD3DSettingsDlg::GetCurrentD3D11AdapterInfo()
{
    CD3D11Enumeration* pD3DEnum = DXUTGetD3D11Enumeration();
    return pD3DEnum->GetAdapterInfo( g_DeviceSettings.d3d11.AdapterOrdinal );
}


//-------------------------------------------------------------------------------------
CD3D11EnumDeviceInfo* CD3DSettingsDlg::GetCurrentD3D11DeviceInfo()
{
    CD3D11Enumeration* pD3DEnum = DXUTGetD3D11Enumeration();
    return pD3DEnum->GetDeviceInfo( g_DeviceSettings.d3d11.AdapterOrdinal,
                                    g_DeviceSettings.d3d11.DriverType );
}


//-------------------------------------------------------------------------------------
CD3D11EnumOutputInfo* CD3DSettingsDlg::GetCurrentD3D11OutputInfo()
{
    CD3D11Enumeration* pD3DEnum = DXUTGetD3D11Enumeration();
    return pD3DEnum->GetOutputInfo( g_DeviceSettings.d3d11.AdapterOrdinal,
                                    g_DeviceSettings.d3d11.Output );
}


//-------------------------------------------------------------------------------------
CD3D9EnumDeviceSettingsCombo* CD3DSettingsDlg::GetCurrentDeviceSettingsCombo()
{
    CD3D9Enumeration* pD3DEnum = DXUTGetD3D9Enumeration();
    return pD3DEnum->GetDeviceSettingsCombo( g_DeviceSettings.d3d9.AdapterOrdinal,
                                             g_DeviceSettings.d3d9.DeviceType,
                                             g_DeviceSettings.d3d9.AdapterFormat,
                                             g_DeviceSettings.d3d9.pp.BackBufferFormat,
                                             ( g_DeviceSettings.d3d9.pp.Windowed == TRUE ) );
}

//-------------------------------------------------------------------------------------
CD3D11EnumDeviceSettingsCombo* CD3DSettingsDlg::GetCurrentD3D11DeviceSettingsCombo()
{
    CD3D11Enumeration* pD3DEnum = DXUTGetD3D11Enumeration();
    return pD3DEnum->GetDeviceSettingsCombo( g_DeviceSettings.d3d11.AdapterOrdinal,
                                             g_DeviceSettings.d3d11.DriverType,
                                             g_DeviceSettings.d3d11.Output,
                                             g_DeviceSettings.d3d11.sd.BufferDesc.Format,
                                             ( g_DeviceSettings.d3d11.sd.Windowed == TRUE ) );
}

HRESULT CD3DSettingsDlg::OnD3D11ResolutionChanged () {
    DWORD dwWidth, dwHeight;
    GetSelectedD3D11Resolution( &dwWidth, &dwHeight );
    g_DeviceSettings.d3d11.sd.BufferDesc.Width= dwWidth;
    g_DeviceSettings.d3d11.sd.BufferDesc.Height = dwHeight;
    
    return S_OK;
}

HRESULT CD3DSettingsDlg::OnFeatureLevelChanged () {
    HRESULT hr = E_FAIL;
    if (g_DeviceSettings.ver == DXUT_D3D11_DEVICE) {
        if (g_DeviceSettings.d3d11.DeviceFeatureLevel == GetSelectedFeatureLevel()) return S_OK;
        //if( !bRefresh )
        //{
            // Obtain a set of valid D3D10 device settings.
            UINT CreateFlags = g_DeviceSettings.d3d11.CreateFlags;
            ZeroMemory( &g_DeviceSettings, sizeof( g_DeviceSettings ) );
            
            DXUTApplyDefaultDeviceSettings(&g_DeviceSettings);
            g_DeviceSettings.d3d11.CreateFlags = CreateFlags;
            g_DeviceSettings.ver = DXUT_D3D11_DEVICE;
            //g_DeviceSettings.d3d11.fl = GetSelectedFeatureLevel();
            hr = DXUTSnapDeviceSettingsToEnumDevice(&g_DeviceSettings, true, GetSelectedFeatureLevel());
            
        //}

        CD3D11Enumeration* pD3DEnum = DXUTGetD3D11Enumeration();
        CGrowableArray <CD3D11EnumAdapterInfo*>* pAdapterInfoList = pD3DEnum->GetAdapterInfoList();

        CDXUTComboBox* pAdapterComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_ADAPTER );
        pAdapterComboBox->RemoveAllItems();

        for( int iAdapter = 0; iAdapter < pAdapterInfoList->GetSize(); ++iAdapter )
        {
            CD3D11EnumAdapterInfo* pAdapterInfo = pAdapterInfoList->GetAt( iAdapter );
            AddAdapter( pAdapterInfo->szUniqueDescription, pAdapterInfo->AdapterOrdinal );
        }

        pAdapterComboBox->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d11.AdapterOrdinal ) );

        CDXUTCheckBox* pCheckBox = m_Dialog.GetCheckBox( DXUTSETTINGSDLG_D3D11_DEBUG_DEVICE );
        pCheckBox->SetChecked( 0 != ( g_DeviceSettings.d3d11.CreateFlags & D3D11_CREATE_DEVICE_DEBUG ) );

        hr = OnAdapterChanged();
        if( FAILED( hr ) )
            return hr;
    }
    
    return hr;
}

//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnAPIVersionChanged( bool bRefresh )
{
    HRESULT hr;

    // Store the API version
    g_DeviceSettings.ver = GetSelectedAPIVersion();

    // Show/hide appropriate dialog controls based on version.
    ShowControlSet( g_DeviceSettings.ver );

    switch( g_DeviceSettings.ver )
    {
        case DXUT_D3D9_DEVICE:
        {
            if( !bRefresh )
            {
                // Obtain a set of valid D3D9 device settings.
                UINT CreateFlags = g_DeviceSettings.d3d11.CreateFlags;
                ZeroMemory( &g_DeviceSettings, sizeof( g_DeviceSettings ) );
                // We want a specific API version, so set up match option to preserve it.
                DXUTApplyDefaultDeviceSettings(&g_DeviceSettings);
                g_DeviceSettings.d3d11.CreateFlags = CreateFlags;
                g_DeviceSettings.ver = DXUT_D3D9_DEVICE;
                DXUTSnapDeviceSettingsToEnumDevice ( &g_DeviceSettings, true);
            }

            CD3D9Enumeration* pD3DEnum = DXUTGetD3D9Enumeration();
            CGrowableArray <CD3D9EnumAdapterInfo*>* pAdapterInfoList = pD3DEnum->GetAdapterInfoList();

            CDXUTComboBox* pAdapterComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_ADAPTER );
            pAdapterComboBox->RemoveAllItems();

            for( int iAdapter = 0; iAdapter < pAdapterInfoList->GetSize(); ++iAdapter )
            {
                CD3D9EnumAdapterInfo* pAdapterInfo = pAdapterInfoList->GetAt( iAdapter );
                AddAdapter( pAdapterInfo->szUniqueDescription, pAdapterInfo->AdapterOrdinal );
            }

            pAdapterComboBox->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d9.AdapterOrdinal ) );

            hr = OnAdapterChanged();
            if( FAILED( hr ) )
                return hr;

            break;
        }

        case DXUT_D3D11_DEVICE:
        {
            if( !bRefresh )
            {
                // Obtain a set of valid D3D10 device settings.
                UINT CreateFlags = g_DeviceSettings.d3d11.CreateFlags;
                ZeroMemory( &g_DeviceSettings, sizeof( g_DeviceSettings ) );
                // We want a specific API version, so set up match option to preserve it.
                DXUTApplyDefaultDeviceSettings(&g_DeviceSettings);
                g_DeviceSettings.d3d11.CreateFlags = CreateFlags;
                g_DeviceSettings.ver = DXUT_D3D11_DEVICE;
                DXUTSnapDeviceSettingsToEnumDevice(&g_DeviceSettings, true);
                
            }

            CD3D11Enumeration* pD3DEnum = DXUTGetD3D11Enumeration();
            CGrowableArray <CD3D11EnumAdapterInfo*>* pAdapterInfoList = pD3DEnum->GetAdapterInfoList();

            CDXUTComboBox* pAdapterComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_ADAPTER );
            pAdapterComboBox->RemoveAllItems();

            for( int iAdapter = 0; iAdapter < pAdapterInfoList->GetSize(); ++iAdapter )
            {
                CD3D11EnumAdapterInfo* pAdapterInfo = pAdapterInfoList->GetAt( iAdapter );
                AddAdapter( pAdapterInfo->szUniqueDescription, pAdapterInfo->AdapterOrdinal );
            }

            pAdapterComboBox->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d11.AdapterOrdinal ) );

            CDXUTCheckBox* pCheckBox = m_Dialog.GetCheckBox( DXUTSETTINGSDLG_D3D11_DEBUG_DEVICE );
            pCheckBox->SetChecked( 0 != ( g_DeviceSettings.d3d11.CreateFlags & D3D11_CREATE_DEVICE_DEBUG ) );

            hr = OnAdapterChanged();
            if( FAILED( hr ) )
                return hr;

            break;
        }
    }

    return S_OK;
}


//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnAdapterChanged()
{
    HRESULT hr = S_OK;

    switch( g_DeviceSettings.ver )
    {
        case DXUT_D3D9_DEVICE:
        {
            // Store the adapter index
            g_DeviceSettings.d3d9.AdapterOrdinal = GetSelectedAdapter();

            // DXUTSETTINGSDLG_DEVICE_TYPE
            CDXUTComboBox* pDeviceTypeComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_DEVICE_TYPE );
            pDeviceTypeComboBox->RemoveAllItems();

            CD3D9EnumAdapterInfo* pAdapterInfo = GetCurrentAdapterInfo();
            if( pAdapterInfo == NULL )
                return E_FAIL;

            for( int iDeviceInfo = 0; iDeviceInfo < pAdapterInfo->deviceInfoList.GetSize(); iDeviceInfo++ )
            {
                CD3D9EnumDeviceInfo* pDeviceInfo = pAdapterInfo->deviceInfoList.GetAt( iDeviceInfo );
                AddDeviceType( pDeviceInfo->DeviceType );
            }

            pDeviceTypeComboBox->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d9.DeviceType ) );

            hr = OnDeviceTypeChanged();
            if( FAILED( hr ) )
                return hr;

            break;
        }

        case DXUT_D3D11_DEVICE:
        {
            // Store the adapter index
            g_DeviceSettings.d3d11.AdapterOrdinal = GetSelectedAdapter();

            // DXUTSETTINGSDLG_DEVICE_TYPE
            CDXUTComboBox* pDeviceTypeComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_DEVICE_TYPE );
            pDeviceTypeComboBox->RemoveAllItems();

            CD3D11EnumAdapterInfo* pAdapterInfo = GetCurrentD3D11AdapterInfo();
            if( pAdapterInfo == NULL )
                return E_FAIL;

            for( int iDeviceInfo = 0; iDeviceInfo < pAdapterInfo->deviceInfoList.GetSize(); iDeviceInfo++ )
            {
                CD3D11EnumDeviceInfo* pDeviceInfo = pAdapterInfo->deviceInfoList.GetAt( iDeviceInfo );
                AddD3D11DeviceType( pDeviceInfo->DeviceType );
            }

            pDeviceTypeComboBox->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d11.DriverType ) );

            hr = OnDeviceTypeChanged();
            if( FAILED( hr ) )
                return hr;

            break;
        }
    }

    return S_OK;
}



//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnDeviceTypeChanged()
{
    HRESULT hr = S_OK;

    switch( g_DeviceSettings.ver )
    {
        case DXUT_D3D9_DEVICE:
        {
            g_DeviceSettings.d3d9.DeviceType = GetSelectedDeviceType();

            // Update windowed/full screen radio buttons
            bool bHasWindowedDeviceCombo = false;
            bool bHasFullScreenDeviceCombo = false;

            CD3D9EnumDeviceInfo* pDeviceInfo = GetCurrentDeviceInfo();
            if( pDeviceInfo == NULL )
                return E_FAIL;

            for( int idc = 0; idc < pDeviceInfo->deviceSettingsComboList.GetSize(); idc++ )
            {
                CD3D9EnumDeviceSettingsCombo* pDeviceSettingsCombo = pDeviceInfo->deviceSettingsComboList.GetAt( idc );

                if( pDeviceSettingsCombo->Windowed )
                    bHasWindowedDeviceCombo = true;
                else
                    bHasFullScreenDeviceCombo = true;
            }

            // DXUTSETTINGSDLG_WINDOWED, DXUTSETTINGSDLG_FULLSCREEN
            m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_WINDOWED, bHasWindowedDeviceCombo );
            m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_FULLSCREEN, bHasFullScreenDeviceCombo );

            SetWindowed( g_DeviceSettings.d3d9.pp.Windowed && bHasWindowedDeviceCombo );

            hr = OnWindowedFullScreenChanged();
            if( FAILED( hr ) )
                return hr;

            break;
        }
        case DXUT_D3D11_DEVICE:
        {
            g_DeviceSettings.d3d11.DriverType = GetSelectedD3D11DeviceType();

            // DXUTSETTINGSDLG_WINDOWED, DXUTSETTINGSDLG_FULLSCREEN
            m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_WINDOWED, true );
            m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_FULLSCREEN, true );

            SetWindowed( g_DeviceSettings.d3d11.sd.Windowed != 0 );

            hr = OnWindowedFullScreenChanged();
            if( FAILED( hr ) )
                return hr;

            break;
        }
    }

    return S_OK;
}



//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnWindowedFullScreenChanged()
{
    HRESULT hr = S_OK;
    bool bWindowed = IsWindowed();

    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_ADAPTER_FORMAT_LABEL, !bWindowed );
    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_RESOLUTION_LABEL, !bWindowed );
    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_REFRESH_RATE_LABEL, !bWindowed );
    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_D3D11_ADAPTER_OUTPUT_LABEL, !bWindowed );
    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_D3D11_RESOLUTION_LABEL, !bWindowed );
    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_D3D11_REFRESH_RATE_LABEL, !bWindowed );

    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_ADAPTER_FORMAT, !bWindowed );
    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_RESOLUTION, !bWindowed );
    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_RESOLUTION_SHOW_ALL, !bWindowed );
    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_REFRESH_RATE, !bWindowed );
    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_DEVICECLIP, bWindowed );
    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_D3D11_ADAPTER_OUTPUT, !bWindowed );
    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_D3D11_RESOLUTION, !bWindowed );
    m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_D3D11_REFRESH_RATE, !bWindowed );

    switch( g_DeviceSettings.ver )
    {
        case DXUT_D3D9_DEVICE:
        {
            g_DeviceSettings.d3d9.pp.Windowed = bWindowed;
            bool bDeviceClip = ( 0x0 != ( g_DeviceSettings.d3d9.pp.Flags & D3DPRESENTFLAG_DEVICECLIP ) );

            // If windowed, get the appropriate adapter format from Direct3D
            if( g_DeviceSettings.d3d9.pp.Windowed )
            {
                IDirect3D9* pD3D = DXUTGetD3D9Object();
                if( pD3D == NULL )
                    return DXTRACE_ERR( L"DXUTGetD3DObject", E_FAIL );

                D3DDISPLAYMODE mode;
                hr = pD3D->GetAdapterDisplayMode( g_DeviceSettings.d3d9.AdapterOrdinal, &mode );
                if( FAILED( hr ) )
                    return DXTRACE_ERR( L"GetAdapterDisplayMode", hr );

                // Default resolution to the fullscreen res that was last used
                RECT rc = DXUTGetFullsceenClientRectAtModeChange();
                if( rc.right == 0 || rc.bottom == 0 )
                {
                    // If nothing last used, then default to the adapter desktop res
                    g_DeviceSettings.d3d9.pp.BackBufferWidth = mode.Width;
                    g_DeviceSettings.d3d9.pp.BackBufferHeight = mode.Height;
                }
                else
                {
                    g_DeviceSettings.d3d9.pp.BackBufferWidth = rc.right;
                    g_DeviceSettings.d3d9.pp.BackBufferHeight = rc.bottom;
                }

                g_DeviceSettings.d3d9.AdapterFormat = mode.Format;
                g_DeviceSettings.d3d9.pp.FullScreen_RefreshRateInHz = mode.RefreshRate;
            }

            const D3DFORMAT adapterFormat = g_DeviceSettings.d3d9.AdapterFormat;
            const DWORD dwWidth = g_DeviceSettings.d3d9.pp.BackBufferWidth;
            const DWORD dwHeight = g_DeviceSettings.d3d9.pp.BackBufferHeight;
            const DWORD dwRefreshRate = g_DeviceSettings.d3d9.pp.FullScreen_RefreshRateInHz;

            // DXUTSETTINGSDLG_DEVICECLIP
            SetDeviceClip( bDeviceClip );

            // DXUTSETTINGSDLG_ADAPTER_FORMAT
            CDXUTComboBox* pAdapterFormatComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_ADAPTER_FORMAT );
            if( pAdapterFormatComboBox == NULL )
                return E_FAIL;
            pAdapterFormatComboBox->RemoveAllItems();

            CD3D9EnumDeviceInfo* pDeviceInfo = GetCurrentDeviceInfo();
            if( pDeviceInfo == NULL )
                return E_FAIL;

            if( bWindowed )
            {
                AddAdapterFormat( adapterFormat );
            }
            else
            {
                for( int iSettingsCombo = 0; iSettingsCombo < pDeviceInfo->deviceSettingsComboList.GetSize();
                     iSettingsCombo++ )
                {
                    CD3D9EnumDeviceSettingsCombo* pSettingsCombo = pDeviceInfo->deviceSettingsComboList.GetAt(
                        iSettingsCombo );
                    AddAdapterFormat( pSettingsCombo->AdapterFormat );
                }
            }

            pAdapterFormatComboBox->SetSelectedByData( ULongToPtr( adapterFormat ) );

            hr = OnAdapterFormatChanged();
            if( FAILED( hr ) )
                return hr;

            // DXUTSETTINGSDLG_RESOLUTION
            CDXUTComboBox* pResolutionComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_RESOLUTION );

            if( bWindowed )
            {
                pResolutionComboBox->RemoveAllItems();
                AddResolution( dwWidth, dwHeight );
            }

            pResolutionComboBox->SetSelectedByData( ULongToPtr( MAKELONG( dwWidth, dwHeight ) ) );

            hr = OnResolutionChanged();
            if( FAILED( hr ) )
                return hr;

            // DXUTSETTINGSDLG_REFRESH_RATE
            CDXUTComboBox* pRefreshRateComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_REFRESH_RATE );

            if( bWindowed )
            {
                pRefreshRateComboBox->RemoveAllItems();
                AddRefreshRate( dwRefreshRate );
            }

            pRefreshRateComboBox->SetSelectedByData( ULongToPtr( dwRefreshRate ) );

            hr = OnRefreshRateChanged();
            if( FAILED( hr ) )
                return hr;

            break;
        }

        case DXUT_D3D11_DEVICE:
        {
            g_DeviceSettings.d3d11.sd.Windowed = bWindowed;

            // Get available adapter output
            CD3D11Enumeration* pD3DEnum = DXUTGetD3D11Enumeration();

            CDXUTComboBox* pOutputComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_ADAPTER_OUTPUT );
            pOutputComboBox->RemoveAllItems();

            CD3D11EnumAdapterInfo* pAdapterInfo = pD3DEnum->GetAdapterInfo( g_DeviceSettings.d3d11.AdapterOrdinal );
            for( int ioutput = 0; ioutput < pAdapterInfo->outputInfoList.GetSize(); ++ioutput )
            {
                CD3D11EnumOutputInfo* pOutputInfo = pAdapterInfo->outputInfoList.GetAt( ioutput );
                AddD3D11AdapterOutput( pOutputInfo->Desc.DeviceName, pOutputInfo->Output );
            }

            pOutputComboBox->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d11.Output ) );

            hr = OnAdapterOutputChanged();
            if( FAILED( hr ) )
                return hr;

            break;
        }
    }

    return S_OK;
}


//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnAdapterOutputChanged()
{
    HRESULT hr;

    switch( g_DeviceSettings.ver )
    {
        case DXUT_D3D11_DEVICE:
        {
            bool bWindowed = IsWindowed();
            g_DeviceSettings.d3d11.sd.Windowed = bWindowed;

            // If windowed, get the appropriate adapter format from Direct3D
            if( g_DeviceSettings.d3d11.sd.Windowed )
            {
                DXGI_MODE_DESC mode;
                hr = DXUTGetD3D11AdapterDisplayMode( g_DeviceSettings.d3d11.AdapterOrdinal,
                                                     g_DeviceSettings.d3d11.Output, &mode );
                if( FAILED( hr ) )
                    return DXTRACE_ERR( L"GetD3D11AdapterDisplayMode", hr );

                // Default resolution to the fullscreen res that was last used
                RECT rc = DXUTGetFullsceenClientRectAtModeChange();
                if( rc.right == 0 || rc.bottom == 0 )
                {
                    // If nothing last used, then default to the adapter desktop res
                    g_DeviceSettings.d3d11.sd.BufferDesc.Width = mode.Width;
                    g_DeviceSettings.d3d11.sd.BufferDesc.Height = mode.Height;
                }
                else
                {
                    g_DeviceSettings.d3d11.sd.BufferDesc.Width = rc.right;
                    g_DeviceSettings.d3d11.sd.BufferDesc.Height = rc.bottom;
                }

                g_DeviceSettings.d3d11.sd.BufferDesc.RefreshRate = mode.RefreshRate;
            }

            const DXGI_RATIONAL RefreshRate = g_DeviceSettings.d3d11.sd.BufferDesc.RefreshRate;

            CD3D11EnumAdapterInfo* pAdapterInfo = GetCurrentD3D11AdapterInfo();
            if( pAdapterInfo == NULL )
                return E_FAIL;

            // DXUTSETTINGSDLG_D3D11_RESOLUTION
            hr = UpdateD3D11Resolutions();
            if( FAILED( hr ) )
                return hr;

            // DXUTSETTINGSDLG_D3D11_BACK_BUFFER_FORMAT
            CDXUTComboBox* pBackBufferFormatComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_BACK_BUFFER_FORMAT
                                                                              );
            pBackBufferFormatComboBox->RemoveAllItems();

            for( int idc = 0; idc < pAdapterInfo->deviceSettingsComboList.GetSize(); idc++ )
            {
                CD3D11EnumDeviceSettingsCombo* pDeviceCombo = pAdapterInfo->deviceSettingsComboList.GetAt( idc );
                if( ( pDeviceCombo->Windowed == TRUE ) == bWindowed )
                {
                    AddD3D11BackBufferFormat( pDeviceCombo->BackBufferFormat );
                }
            }

            pBackBufferFormatComboBox->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d11.sd.BufferDesc.Format ) );

            hr = OnBackBufferFormatChanged();
            if( FAILED( hr ) )
                return hr;

            // DXUTSETTINGSDLG_D3D11_REFRESH_RATE
            if( bWindowed )
            {
                CDXUTComboBox* pRefreshRateComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_REFRESH_RATE );
                for( UINT i = 0; i < pRefreshRateComboBox->GetNumItems(); ++i )
                {
                    DXGI_RATIONAL* pRefreshRate = reinterpret_cast<DXGI_RATIONAL*>(
                        pRefreshRateComboBox->GetItemData( i ) );
                    delete pRefreshRate;
                }
                pRefreshRateComboBox->RemoveAllItems();
                AddD3D11RefreshRate( RefreshRate );
            }

            SetSelectedD3D11RefreshRate( RefreshRate );
            break;
        }
    };

    hr = OnRefreshRateChanged();
    if( FAILED( hr ) )
        return hr;

    return S_OK;
}


//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnAdapterFormatChanged()
{
    HRESULT hr = S_OK;

    switch( g_DeviceSettings.ver )
    {
        case DXUT_D3D9_DEVICE:
        {
            // DXUTSETTINGSDLG_ADAPTER_FORMAT
            D3DFORMAT adapterFormat = GetSelectedAdapterFormat();
            g_DeviceSettings.d3d9.AdapterFormat = adapterFormat;

            // DXUTSETTINGSDLG_RESOLUTION
            CDXUTComboBox* pResolutionComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_RESOLUTION );
            pResolutionComboBox->RemoveAllItems();

            CD3D9EnumAdapterInfo* pAdapterInfo = GetCurrentAdapterInfo();
            if( pAdapterInfo == NULL )
                return E_FAIL;

            bool bShowAll = m_Dialog.GetCheckBox( DXUTSETTINGSDLG_RESOLUTION_SHOW_ALL )->GetChecked();

            // Get the desktop aspect ratio
            D3DDISPLAYMODE dmDesktop;
            DXUTGetDesktopResolution( g_DeviceSettings.d3d9.AdapterOrdinal, &dmDesktop.Width, &dmDesktop.Height );
            float fDesktopAspectRatio = dmDesktop.Width / ( float )dmDesktop.Height;

            for( int idm = 0; idm < pAdapterInfo->displayModeList.GetSize(); idm++ )
            {
                D3DDISPLAYMODE DisplayMode = pAdapterInfo->displayModeList.GetAt( idm );
                float fAspect = ( float )DisplayMode.Width / ( float )DisplayMode.Height;

                if( DisplayMode.Format == g_DeviceSettings.d3d9.AdapterFormat )
                {
                    // If "Show All" is not checked, then hide all resolutions
                    // that don't match the aspect ratio of the desktop resolution
                    if( bShowAll || ( !bShowAll && fabsf( fDesktopAspectRatio - fAspect ) < 0.05f ) )
                    {
                        AddResolution( DisplayMode.Width, DisplayMode.Height );
                    }
                }
            }

            const DWORD dwCurResolution = MAKELONG( g_DeviceSettings.d3d9.pp.BackBufferWidth,
                                                    g_DeviceSettings.d3d9.pp.BackBufferHeight );

            pResolutionComboBox->SetSelectedByData( ULongToPtr( dwCurResolution ) );

            hr = OnResolutionChanged();
            if( FAILED( hr ) )
                return hr;

            // DXUTSETTINGSDLG_BACK_BUFFER_FORMAT
            CDXUTComboBox* pBackBufferFormatComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_BACK_BUFFER_FORMAT );
            pBackBufferFormatComboBox->RemoveAllItems();

            CD3D9EnumDeviceInfo* pDeviceInfo = GetCurrentDeviceInfo();
            if( pDeviceInfo == NULL )
                return E_FAIL;

            const BOOL bWindowed = IsWindowed();
            bool bHasWindowedBackBuffer = false;

            for( int idc = 0; idc < pDeviceInfo->deviceSettingsComboList.GetSize(); idc++ )
            {
                CD3D9EnumDeviceSettingsCombo* pDeviceCombo = pDeviceInfo->deviceSettingsComboList.GetAt( idc );
                if( pDeviceCombo->Windowed == bWindowed &&
                    pDeviceCombo->AdapterFormat == g_DeviceSettings.d3d9.AdapterFormat )
                {
                    AddBackBufferFormat( pDeviceCombo->BackBufferFormat );
                    bHasWindowedBackBuffer = true;
                }
            }

            pBackBufferFormatComboBox->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d9.pp.BackBufferFormat ) );

            hr = OnBackBufferFormatChanged();
            if( FAILED( hr ) )
                return hr;

            if( !bHasWindowedBackBuffer )
            {
                m_Dialog.SetControlEnabled( DXUTSETTINGSDLG_WINDOWED, false );

                if( g_DeviceSettings.d3d9.pp.Windowed )
                {
                    SetWindowed( false );

                    hr = OnWindowedFullScreenChanged();
                    if( FAILED( hr ) )
                        return hr;
                }
            }

            break;
        }
    }

    return S_OK;
}


//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnResolutionChanged()
{
    HRESULT hr = S_OK;

    CD3D9EnumAdapterInfo* pAdapterInfo = GetCurrentAdapterInfo();
    if( pAdapterInfo == NULL )
        return E_FAIL;

    // Set resolution
    DWORD dwWidth, dwHeight;
    GetSelectedResolution( &dwWidth, &dwHeight );
    g_DeviceSettings.d3d9.pp.BackBufferWidth = dwWidth;
    g_DeviceSettings.d3d9.pp.BackBufferHeight = dwHeight;

    DWORD dwRefreshRate = g_DeviceSettings.d3d9.pp.FullScreen_RefreshRateInHz;

    // Update the refresh rate list
    CDXUTComboBox* pRefreshRateComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_REFRESH_RATE );
    pRefreshRateComboBox->RemoveAllItems();

    D3DFORMAT adapterFormat = g_DeviceSettings.d3d9.AdapterFormat;
    for( int idm = 0; idm < pAdapterInfo->displayModeList.GetSize(); idm++ )
    {
        D3DDISPLAYMODE displayMode = pAdapterInfo->displayModeList.GetAt( idm );

        if( displayMode.Format == adapterFormat &&
            displayMode.Width == dwWidth &&
            displayMode.Height == dwHeight )
        {
            AddRefreshRate( displayMode.RefreshRate );
        }
    }

    pRefreshRateComboBox->SetSelectedByData( ULongToPtr( dwRefreshRate ) );

    hr = OnRefreshRateChanged();
    if( FAILED( hr ) )
        return hr;

    return S_OK;
}



//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnRefreshRateChanged()
{
    // Set refresh rate
    switch( g_DeviceSettings.ver )
    {
        case DXUT_D3D9_DEVICE:
            g_DeviceSettings.d3d9.pp.FullScreen_RefreshRateInHz = GetSelectedRefreshRate();
            break;

        case DXUT_D3D11_DEVICE:
            g_DeviceSettings.d3d11.sd.BufferDesc.RefreshRate = GetSelectedD3D11RefreshRate();
            break;
    }

    return S_OK;
}


//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnBackBufferFormatChanged()
{
    HRESULT hr = S_OK;

    switch( g_DeviceSettings.ver )
    {
        case DXUT_D3D9_DEVICE:
        {
            g_DeviceSettings.d3d9.pp.BackBufferFormat = GetSelectedBackBufferFormat();

            D3DFORMAT adapterFormat = g_DeviceSettings.d3d9.AdapterFormat;
            D3DFORMAT backBufferFormat = g_DeviceSettings.d3d9.pp.BackBufferFormat;

            CD3D9EnumDeviceInfo* pDeviceInfo = GetCurrentDeviceInfo();
            if( pDeviceInfo == NULL )
                return E_FAIL;

            bool bAllowSoftwareVP, bAllowHardwareVP, bAllowPureHardwareVP, bAllowMixedVP;
            DXUTGetD3D9Enumeration()->GetPossibleVertexProcessingList( &bAllowSoftwareVP, &bAllowHardwareVP,
                                                                       &bAllowPureHardwareVP, &bAllowMixedVP );

            for( int idc = 0; idc < pDeviceInfo->deviceSettingsComboList.GetSize(); idc++ )
            {
                CD3D9EnumDeviceSettingsCombo* pDeviceCombo = pDeviceInfo->deviceSettingsComboList.GetAt( idc );

                if( pDeviceCombo->Windowed == ( g_DeviceSettings.d3d9.pp.Windowed == TRUE ) &&
                    pDeviceCombo->AdapterFormat == adapterFormat &&
                    pDeviceCombo->BackBufferFormat == backBufferFormat )
                {
                    CDXUTComboBox* pDepthStencilComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_DEPTH_STENCIL );
                    pDepthStencilComboBox->RemoveAllItems();
                    pDepthStencilComboBox->SetEnabled( ( g_DeviceSettings.d3d9.pp.EnableAutoDepthStencil == TRUE ) );

                    if( g_DeviceSettings.d3d9.pp.EnableAutoDepthStencil )
                    {
                        for( int ifmt = 0; ifmt < pDeviceCombo->depthStencilFormatList.GetSize(); ifmt++ )
                        {
                            D3DFORMAT fmt = pDeviceCombo->depthStencilFormatList.GetAt( ifmt );

                            AddDepthStencilBufferFormat( fmt );
                        }

                        pDepthStencilComboBox->SetSelectedByData( ULongToPtr(
                                                                  g_DeviceSettings.d3d9.pp.AutoDepthStencilFormat ) );
                    }
                    else
                    {
                        if( !pDepthStencilComboBox->ContainsItem( L"(not used)" ) )
                            pDepthStencilComboBox->AddItem( L"(not used)", NULL );
                    }

                    hr = OnDepthStencilBufferFormatChanged();
                    if( FAILED( hr ) )
                        return hr;

                    CDXUTComboBox* pVertexProcessingComboBox =
                        m_Dialog.GetComboBox( DXUTSETTINGSDLG_VERTEX_PROCESSING );
                    pVertexProcessingComboBox->RemoveAllItems();

                    // Add valid vertex processing types
                    if( bAllowSoftwareVP )
                        AddVertexProcessingType( D3DCREATE_SOFTWARE_VERTEXPROCESSING );

                    if( bAllowHardwareVP && pDeviceInfo->Caps.DevCaps & D3DDEVCAPS_HWTRANSFORMANDLIGHT )
                        AddVertexProcessingType( D3DCREATE_HARDWARE_VERTEXPROCESSING );

                    if( bAllowPureHardwareVP && pDeviceInfo->Caps.DevCaps & D3DDEVCAPS_PUREDEVICE )
                        AddVertexProcessingType( D3DCREATE_PUREDEVICE );

                    if( bAllowMixedVP && pDeviceInfo->Caps.DevCaps & D3DDEVCAPS_HWTRANSFORMANDLIGHT )
                        AddVertexProcessingType( D3DCREATE_MIXED_VERTEXPROCESSING );

                    if( g_DeviceSettings.d3d9.BehaviorFlags & D3DCREATE_PUREDEVICE )
                        pVertexProcessingComboBox->SetSelectedByData( ULongToPtr( D3DCREATE_PUREDEVICE ) );
                    else if( g_DeviceSettings.d3d9.BehaviorFlags & D3DCREATE_SOFTWARE_VERTEXPROCESSING )
                        pVertexProcessingComboBox->SetSelectedByData( ULongToPtr(
                                                                      D3DCREATE_SOFTWARE_VERTEXPROCESSING ) );
                    else if( g_DeviceSettings.d3d9.BehaviorFlags & D3DCREATE_HARDWARE_VERTEXPROCESSING )
                        pVertexProcessingComboBox->SetSelectedByData( ULongToPtr(
                                                                      D3DCREATE_HARDWARE_VERTEXPROCESSING ) );
                    else if( g_DeviceSettings.d3d9.BehaviorFlags & D3DCREATE_MIXED_VERTEXPROCESSING )
                        pVertexProcessingComboBox->SetSelectedByData( ULongToPtr( D3DCREATE_MIXED_VERTEXPROCESSING ) );

                    hr = OnVertexProcessingChanged();
                    if( FAILED( hr ) )
                        return hr;

                    CDXUTComboBox* pPresentIntervalComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_PRESENT_INTERVAL );
                    pPresentIntervalComboBox->RemoveAllItems();
                    pPresentIntervalComboBox->AddItem( L"On", ULongToPtr( D3DPRESENT_INTERVAL_DEFAULT ) );
                    pPresentIntervalComboBox->AddItem( L"Off", ULongToPtr( D3DPRESENT_INTERVAL_IMMEDIATE ) );

                    pPresentIntervalComboBox->SetSelectedByData( ULongToPtr(
                                                                 g_DeviceSettings.d3d9.pp.PresentationInterval ) );

                    hr = OnPresentIntervalChanged();
                    if( FAILED( hr ) )
                        return hr;
                }
            }

            break;
        }

        case DXUT_D3D11_DEVICE:
        {
            g_DeviceSettings.d3d11.sd.BufferDesc.Format = GetSelectedD3D11BackBufferFormat();

            DXGI_FORMAT backBufferFormat = g_DeviceSettings.d3d11.sd.BufferDesc.Format;

            CD3D11EnumAdapterInfo* pAdapterInfo = GetCurrentD3D11AdapterInfo();
            if( pAdapterInfo == NULL )
                return E_FAIL;

            for( int idc = 0; idc < pAdapterInfo->deviceSettingsComboList.GetSize(); idc++ )
            {
                CD3D11EnumDeviceSettingsCombo* pDeviceCombo = pAdapterInfo->deviceSettingsComboList.GetAt( idc );

                if( pDeviceCombo->Windowed == ( g_DeviceSettings.d3d11.sd.Windowed == TRUE ) &&
                    pDeviceCombo->BackBufferFormat == backBufferFormat &&
                    pDeviceCombo->DeviceType == g_DeviceSettings.d3d11.DriverType )
                {
                    CDXUTComboBox* pMultisampleCountCombo = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_COUNT
                                                                                   );
                    pMultisampleCountCombo->RemoveAllItems();
                    for( int i = 0; i < pDeviceCombo->multiSampleCountList.GetSize(); ++i )
                        AddD3D11MultisampleCount( pDeviceCombo->multiSampleCountList.GetAt( i ) );
                    pMultisampleCountCombo->SetSelectedByData( ULongToPtr(
                                                               g_DeviceSettings.d3d11.sd.SampleDesc.Count ) );

                    hr = OnMultisampleTypeChanged();
                    if( FAILED( hr ) )
                        return hr;

                    CDXUTComboBox* pPresentIntervalComboBox =
                        m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_PRESENT_INTERVAL );
                    pPresentIntervalComboBox->RemoveAllItems();
                    pPresentIntervalComboBox->AddItem( L"On", ULongToPtr( 1 ) );
                    pPresentIntervalComboBox->AddItem( L"Off", ULongToPtr( 0 ) );

                    pPresentIntervalComboBox->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d11.SyncInterval ) );

                    hr = OnPresentIntervalChanged();
                    if( FAILED( hr ) )
                        return hr;

                    hr = UpdateD3D11Resolutions();
                    if( FAILED( hr ) )
                        return hr;
                }
            }

            break;
        }
    }

    return S_OK;
}


//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnDepthStencilBufferFormatChanged()
{
    HRESULT hr = S_OK;

    D3DFORMAT depthStencilBufferFormat = GetSelectedDepthStencilBufferFormat();

    if( g_DeviceSettings.d3d9.pp.EnableAutoDepthStencil )
        g_DeviceSettings.d3d9.pp.AutoDepthStencilFormat = depthStencilBufferFormat;

    CD3D9EnumDeviceSettingsCombo* pDeviceSettingsCombo = GetCurrentDeviceSettingsCombo();
    if( pDeviceSettingsCombo == NULL )
        return E_FAIL;

    CDXUTComboBox* pMultisampleTypeCombo = m_Dialog.GetComboBox( DXUTSETTINGSDLG_MULTISAMPLE_TYPE );
    pMultisampleTypeCombo->RemoveAllItems();

    for( int ims = 0; ims < pDeviceSettingsCombo->multiSampleTypeList.GetSize(); ims++ )
    {
        D3DMULTISAMPLE_TYPE msType = pDeviceSettingsCombo->multiSampleTypeList.GetAt( ims );

        bool bConflictFound = false;
        for( int iConf = 0; iConf < pDeviceSettingsCombo->DSMSConflictList.GetSize(); iConf++ )
        {
            CD3D9EnumDSMSConflict DSMSConf = pDeviceSettingsCombo->DSMSConflictList.GetAt( iConf );
            if( DSMSConf.DSFormat == depthStencilBufferFormat &&
                DSMSConf.MSType == msType )
            {
                bConflictFound = true;
                break;
            }
        }

        if( !bConflictFound )
            AddMultisampleType( msType );
    }

    CDXUTComboBox* pMultisampleQualityCombo = m_Dialog.GetComboBox( DXUTSETTINGSDLG_MULTISAMPLE_TYPE );
    pMultisampleQualityCombo->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d9.pp.MultiSampleType ) );

    hr = OnMultisampleTypeChanged();
    if( FAILED( hr ) )
        return hr;

    return S_OK;
}


//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnMultisampleTypeChanged()
{
    HRESULT hr = S_OK;

    switch( g_DeviceSettings.ver )
    {
        case DXUT_D3D9_DEVICE:
        {
            D3DMULTISAMPLE_TYPE multisampleType = GetSelectedMultisampleType();
            g_DeviceSettings.d3d9.pp.MultiSampleType = multisampleType;

            CD3D9EnumDeviceSettingsCombo* pDeviceSettingsCombo = GetCurrentDeviceSettingsCombo();
            if( pDeviceSettingsCombo == NULL )
                return E_FAIL;

            DWORD dwMaxQuality = 0;
            for( int iType = 0; iType < pDeviceSettingsCombo->multiSampleTypeList.GetSize(); iType++ )
            {
                D3DMULTISAMPLE_TYPE msType = pDeviceSettingsCombo->multiSampleTypeList.GetAt( iType );
                if( msType == multisampleType )
                {
                    dwMaxQuality = pDeviceSettingsCombo->multiSampleQualityList.GetAt( iType );
                    break;
                }
            }

            // DXUTSETTINGSDLG_MULTISAMPLE_QUALITY
            CDXUTComboBox* pMultisampleQualityCombo = m_Dialog.GetComboBox( DXUTSETTINGSDLG_MULTISAMPLE_QUALITY );
            pMultisampleQualityCombo->RemoveAllItems();

            for( UINT iQuality = 0; iQuality < dwMaxQuality; iQuality++ )
            {
                AddMultisampleQuality( iQuality );
            }

            pMultisampleQualityCombo->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d9.pp.MultiSampleQuality ) );

            hr = OnMultisampleQualityChanged();
            if( FAILED( hr ) )
                return hr;

            break;
        }
        case DXUT_D3D11_DEVICE:
        {
            UINT multisampleCount = GetSelectedD3D11MultisampleCount();
            g_DeviceSettings.d3d11.sd.SampleDesc.Count = multisampleCount;

            CD3D11EnumDeviceSettingsCombo* pDeviceSettingsCombo = GetCurrentD3D11DeviceSettingsCombo();
            if( pDeviceSettingsCombo == NULL )
                return E_FAIL;

            UINT MaxQuality = 0;
            for( int iCount = 0; iCount < pDeviceSettingsCombo->multiSampleCountList.GetSize(); iCount++ )
            {
                UINT Count = pDeviceSettingsCombo->multiSampleCountList.GetAt( iCount );
                if( Count == multisampleCount )
                {
                    MaxQuality = pDeviceSettingsCombo->multiSampleQualityList.GetAt( iCount );
                    break;
                }
            }

            // DXUTSETTINGSDLG_D3D11_MULTISAMPLE_QUALITY
            CDXUTComboBox* pMultisampleQualityCombo = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_QUALITY
                                                                             );
            pMultisampleQualityCombo->RemoveAllItems();

            for( UINT iQuality = 0; iQuality < MaxQuality; iQuality++ )
            {
                AddD3D11MultisampleQuality( iQuality );
            }

            pMultisampleQualityCombo->SetSelectedByData( ULongToPtr( g_DeviceSettings.d3d11.sd.SampleDesc.Quality ) );

            hr = OnMultisampleQualityChanged();
            if( FAILED( hr ) )
                return hr;

            break;
        }
    }

    return S_OK;
}


//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnMultisampleQualityChanged()
{
    switch( g_DeviceSettings.ver )
    {
        case DXUT_D3D9_DEVICE:
            g_DeviceSettings.d3d9.pp.MultiSampleQuality = GetSelectedMultisampleQuality();
            break;

        case DXUT_D3D11_DEVICE:
            g_DeviceSettings.d3d11.sd.SampleDesc.Quality = GetSelectedD3D11MultisampleQuality();
            break;
    }

    return S_OK;
}


//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnVertexProcessingChanged()
{
    DWORD dwBehavior = g_DeviceSettings.d3d9.BehaviorFlags;

    // Clear vertex processing flags
    dwBehavior &= ~D3DCREATE_HARDWARE_VERTEXPROCESSING;
    dwBehavior &= ~D3DCREATE_SOFTWARE_VERTEXPROCESSING;
    dwBehavior &= ~D3DCREATE_MIXED_VERTEXPROCESSING;
    dwBehavior &= ~D3DCREATE_PUREDEVICE;

    // Determine new flags
    DWORD dwNewFlags = GetSelectedVertexProcessingType();
    if( dwNewFlags & D3DCREATE_PUREDEVICE )
        dwNewFlags |= D3DCREATE_HARDWARE_VERTEXPROCESSING;

    // Make changes
    g_DeviceSettings.d3d9.BehaviorFlags = dwBehavior | dwNewFlags;

    return S_OK;
}


//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnPresentIntervalChanged()
{
    switch( g_DeviceSettings.ver )
    {
        case DXUT_D3D9_DEVICE:
            g_DeviceSettings.d3d9.pp.PresentationInterval = GetSelectedPresentInterval();
            break;

        case DXUT_D3D11_DEVICE:
            g_DeviceSettings.d3d11.SyncInterval = GetSelectedD3D11PresentInterval();
            break;
    }

    return S_OK;
}


//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnDebugDeviceChanged()
{
    bool bDebugDevice = GetSelectedDebugDeviceValue();

    if( bDebugDevice )
        g_DeviceSettings.d3d11.CreateFlags |= D3D11_CREATE_DEVICE_DEBUG;
    else
        g_DeviceSettings.d3d11.CreateFlags &= ~D3D11_CREATE_DEVICE_DEBUG;

    return S_OK;
}

//-------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::OnDeviceClipChanged()
{
    if( IsDeviceClip() )
        g_DeviceSettings.d3d9.pp.Flags |= D3DPRESENTFLAG_DEVICECLIP;
    else
        g_DeviceSettings.d3d9.pp.Flags &= ~D3DPRESENTFLAG_DEVICECLIP;

    return S_OK;
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddAPIVersion( DXUTDeviceVersion version )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_API_VERSION );

    if( !pComboBox->ContainsItem( DXUTAPIVersionToString( version ) ) )
        pComboBox->AddItem( DXUTAPIVersionToString( version ), ULongToPtr( version ) );
}


//-------------------------------------------------------------------------------------
DXUTDeviceVersion CD3DSettingsDlg::GetSelectedAPIVersion()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_API_VERSION );

    return ( DXUTDeviceVersion )PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddAdapter( const WCHAR* strDescription, UINT iAdapter )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_ADAPTER );

    if( !pComboBox->ContainsItem( strDescription ) )
        pComboBox->AddItem( strDescription, ULongToPtr( iAdapter ) );
}


//-------------------------------------------------------------------------------------
UINT CD3DSettingsDlg::GetSelectedAdapter()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_ADAPTER );

    return PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddDeviceType( D3DDEVTYPE devType )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_DEVICE_TYPE );

    if( !pComboBox->ContainsItem( DXUTD3DDeviceTypeToString( devType ) ) )
        pComboBox->AddItem( DXUTD3DDeviceTypeToString( devType ), ULongToPtr( devType ) );
}


//-------------------------------------------------------------------------------------
D3DDEVTYPE CD3DSettingsDlg::GetSelectedDeviceType()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_DEVICE_TYPE );

    return ( D3DDEVTYPE )PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::SetWindowed( bool bWindowed )
{
    CDXUTRadioButton* pRadioButton = m_Dialog.GetRadioButton( DXUTSETTINGSDLG_WINDOWED );
    pRadioButton->SetChecked( bWindowed );

    pRadioButton = m_Dialog.GetRadioButton( DXUTSETTINGSDLG_FULLSCREEN );
    pRadioButton->SetChecked( !bWindowed );
}


//-------------------------------------------------------------------------------------
bool CD3DSettingsDlg::IsWindowed()
{
    CDXUTRadioButton* pRadioButton = m_Dialog.GetRadioButton( DXUTSETTINGSDLG_WINDOWED );
    return pRadioButton->GetChecked();
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddAdapterFormat( D3DFORMAT format )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_ADAPTER_FORMAT );

    if( !pComboBox->ContainsItem( DXUTD3DFormatToString( format, TRUE ) ) )
        pComboBox->AddItem( DXUTD3DFormatToString( format, TRUE ), ULongToPtr( format ) );
}


//-------------------------------------------------------------------------------------
D3DFORMAT CD3DSettingsDlg::GetSelectedAdapterFormat()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_ADAPTER_FORMAT );

    return ( D3DFORMAT )PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddD3D11AdapterOutput( const WCHAR* strName, UINT Output )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_ADAPTER_OUTPUT );

    if( !pComboBox->ContainsItem( strName ) )
        pComboBox->AddItem( strName, ULongToPtr( Output ) );
}


//-------------------------------------------------------------------------------------
UINT CD3DSettingsDlg::GetSelectedD3D11AdapterOutput()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_ADAPTER_OUTPUT );

    return PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddResolution( DWORD dwWidth, DWORD dwHeight )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_RESOLUTION );

    DWORD dwResolutionData;
    WCHAR strResolution[50];
    dwResolutionData = MAKELONG( dwWidth, dwHeight );
    swprintf_s( strResolution, 50, L"%d by %d", dwWidth, dwHeight );

    if( !pComboBox->ContainsItem( strResolution ) )
        pComboBox->AddItem( strResolution, ULongToPtr( dwResolutionData ) );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::GetSelectedResolution( DWORD* pdwWidth, DWORD* pdwHeight )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_RESOLUTION );

    DWORD dwResolution = PtrToUlong( pComboBox->GetSelectedData() );

    *pdwWidth = LOWORD( dwResolution );
    *pdwHeight = HIWORD( dwResolution );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddD3D11Resolution( DWORD dwWidth, DWORD dwHeight )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_RESOLUTION );

    DWORD dwResolutionData;
    WCHAR strResolution[50];
    dwResolutionData = MAKELONG( dwWidth, dwHeight );
    swprintf_s( strResolution, 50, L"%d by %d", dwWidth, dwHeight );

    if( !pComboBox->ContainsItem( strResolution ) )
        pComboBox->AddItem( strResolution, ULongToPtr( dwResolutionData ) );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::GetSelectedD3D11Resolution( DWORD* pdwWidth, DWORD* pdwHeight )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_RESOLUTION );

    DWORD dwResolution = PtrToUlong( pComboBox->GetSelectedData() );

    *pdwWidth = LOWORD( dwResolution );
    *pdwHeight = HIWORD( dwResolution );
}

void CD3DSettingsDlg::AddD3D11FeatureLevel(D3D_FEATURE_LEVEL fl) {
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_FEATURE_LEVEL );
    switch( fl )
    {
    case D3D_FEATURE_LEVEL_9_1: 
        {
            if( !pComboBox->ContainsItem( L"D3D_FEATURE_LEVEL_9_1" ) )
                pComboBox->AddItem( L"D3D_FEATURE_LEVEL_9_1", ULongToPtr( D3D_FEATURE_LEVEL_9_1 ) ); 
        }
        break;
    case D3D_FEATURE_LEVEL_9_2: 
        {
            if( !pComboBox->ContainsItem( L"D3D_FEATURE_LEVEL_9_2" ) )
                pComboBox->AddItem( L"D3D_FEATURE_LEVEL_9_2", ULongToPtr( D3D_FEATURE_LEVEL_9_2 ) ); 
        }
        break;
    case D3D_FEATURE_LEVEL_9_3: 
        {
            if( !pComboBox->ContainsItem( L"D3D_FEATURE_LEVEL_9_3" ) )
                pComboBox->AddItem( L"D3D_FEATURE_LEVEL_9_3", ULongToPtr( D3D_FEATURE_LEVEL_9_3 ) ); 
        }
        break;
    case D3D_FEATURE_LEVEL_10_0: 
        {
            if( !pComboBox->ContainsItem( L"D3D_FEATURE_LEVEL_10_0" ) )
                pComboBox->AddItem( L"D3D_FEATURE_LEVEL_10_0", ULongToPtr( D3D_FEATURE_LEVEL_10_0 ) ); 
        }
        break;
    case D3D_FEATURE_LEVEL_10_1: 
        {
            if( !pComboBox->ContainsItem( L"D3D_FEATURE_LEVEL_10_1" ) )
                pComboBox->AddItem( L"D3D_FEATURE_LEVEL_10_1", ULongToPtr( D3D_FEATURE_LEVEL_10_1 ) ); 
        }
        break;
    case D3D_FEATURE_LEVEL_11_0: 
        {
            if( !pComboBox->ContainsItem( L"D3D_FEATURE_LEVEL_11_0" ) )
                pComboBox->AddItem( L"D3D_FEATURE_LEVEL_11_0", ULongToPtr( D3D_FEATURE_LEVEL_11_0 ) ); 
        }
        break;
    }

}

D3D_FEATURE_LEVEL CD3DSettingsDlg::GetSelectedFeatureLevel() {
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_FEATURE_LEVEL );

    return (D3D_FEATURE_LEVEL)PtrToUlong( pComboBox->GetSelectedData() );
}
//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddRefreshRate( DWORD dwRate )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_REFRESH_RATE );

    WCHAR strRefreshRate[50];

    if( dwRate == 0 )
        wcscpy_s( strRefreshRate, 50, L"Default Rate" );
    else
        swprintf_s( strRefreshRate, 50, L"%d Hz", dwRate );

    if( !pComboBox->ContainsItem( strRefreshRate ) )
        pComboBox->AddItem( strRefreshRate, ULongToPtr( dwRate ) );
}


//-------------------------------------------------------------------------------------
DWORD CD3DSettingsDlg::GetSelectedRefreshRate()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_REFRESH_RATE );

    return PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddD3D11RefreshRate( DXGI_RATIONAL RefreshRate )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_REFRESH_RATE );

    WCHAR strRefreshRate[50];

    if( RefreshRate.Numerator == 0 && RefreshRate.Denominator == 0 )
        wcscpy_s( strRefreshRate, 50, L"Default Rate" );
    else
        swprintf_s( strRefreshRate, 50, L"%d Hz", RefreshRate.Numerator / RefreshRate.Denominator );

    if( !pComboBox->ContainsItem( strRefreshRate ) )
    {
        DXGI_RATIONAL* pNewRate = new DXGI_RATIONAL;
        if( pNewRate )
        {
            *pNewRate = RefreshRate;
            pComboBox->AddItem( strRefreshRate, pNewRate );
        }
    }
}


//-------------------------------------------------------------------------------------
DXGI_RATIONAL CD3DSettingsDlg::GetSelectedD3D11RefreshRate()
{
    DXGI_RATIONAL dxgiR;
    dxgiR.Numerator = 0;
    dxgiR.Denominator = 1;
    
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_REFRESH_RATE );

    return *reinterpret_cast<DXGI_RATIONAL*>( pComboBox->GetSelectedData() );
     
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddBackBufferFormat( D3DFORMAT format )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_BACK_BUFFER_FORMAT );

    if( !pComboBox->ContainsItem( DXUTD3DFormatToString( format, TRUE ) ) )
        pComboBox->AddItem( DXUTD3DFormatToString( format, TRUE ), ULongToPtr( format ) );
}


//-------------------------------------------------------------------------------------
D3DFORMAT CD3DSettingsDlg::GetSelectedBackBufferFormat()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_BACK_BUFFER_FORMAT );

    return ( D3DFORMAT )PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddD3D11BackBufferFormat( DXGI_FORMAT format )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_BACK_BUFFER_FORMAT );

    if( !pComboBox->ContainsItem( DXUTDXGIFormatToString( format, TRUE ) ) )
        pComboBox->AddItem( DXUTDXGIFormatToString( format, TRUE ), ULongToPtr( format ) );
}


//-------------------------------------------------------------------------------------
DXGI_FORMAT CD3DSettingsDlg::GetSelectedD3D11BackBufferFormat()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_BACK_BUFFER_FORMAT );

    return ( DXGI_FORMAT )PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddDepthStencilBufferFormat( D3DFORMAT format )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_DEPTH_STENCIL );

    if( !pComboBox->ContainsItem( DXUTD3DFormatToString( format, TRUE ) ) )
        pComboBox->AddItem( DXUTD3DFormatToString( format, TRUE ), ULongToPtr( format ) );
}


//-------------------------------------------------------------------------------------
D3DFORMAT CD3DSettingsDlg::GetSelectedDepthStencilBufferFormat()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_DEPTH_STENCIL );

    return ( D3DFORMAT )PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddMultisampleType( D3DMULTISAMPLE_TYPE type )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_MULTISAMPLE_TYPE );

    if( !pComboBox->ContainsItem( DXUTMultisampleTypeToString( type ) ) )
        pComboBox->AddItem( DXUTMultisampleTypeToString( type ), ULongToPtr( type ) );
}


//-------------------------------------------------------------------------------------
D3DMULTISAMPLE_TYPE CD3DSettingsDlg::GetSelectedMultisampleType()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_MULTISAMPLE_TYPE );

    return ( D3DMULTISAMPLE_TYPE )PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddMultisampleQuality( DWORD dwQuality )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_MULTISAMPLE_QUALITY );

    WCHAR strQuality[50];
    swprintf_s( strQuality, 50, L"%d", dwQuality );

    if( !pComboBox->ContainsItem( strQuality ) )
        pComboBox->AddItem( strQuality, ULongToPtr( dwQuality ) );
}


//-------------------------------------------------------------------------------------
DWORD CD3DSettingsDlg::GetSelectedMultisampleQuality()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_MULTISAMPLE_QUALITY );

    return PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddD3D11MultisampleCount( UINT Count )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_COUNT );

    WCHAR str[50];
    swprintf_s( str, 50, L"%u", Count );

    if( !pComboBox->ContainsItem( str ) )
        pComboBox->AddItem( str, ULongToPtr( Count ) );
}


//-------------------------------------------------------------------------------------
UINT CD3DSettingsDlg::GetSelectedD3D11MultisampleCount()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_COUNT );

    return ( UINT )PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddD3D11MultisampleQuality( UINT Quality )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_QUALITY );

    WCHAR strQuality[50];
    swprintf_s( strQuality, 50, L"%d", Quality );

    if( !pComboBox->ContainsItem( strQuality ) )
        pComboBox->AddItem( strQuality, ULongToPtr( Quality ) );
}


//-------------------------------------------------------------------------------------
UINT CD3DSettingsDlg::GetSelectedD3D11MultisampleQuality()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_MULTISAMPLE_QUALITY );

    return ( UINT )PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddVertexProcessingType( DWORD dwType )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_VERTEX_PROCESSING );

    if( !pComboBox->ContainsItem( DXUTVertexProcessingTypeToString( dwType ) ) )
        pComboBox->AddItem( DXUTVertexProcessingTypeToString( dwType ), ULongToPtr( dwType ) );
}


//-------------------------------------------------------------------------------------
DWORD CD3DSettingsDlg::GetSelectedVertexProcessingType()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_VERTEX_PROCESSING );

    return PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
DWORD CD3DSettingsDlg::GetSelectedPresentInterval()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_PRESENT_INTERVAL );

    return PtrToUlong( pComboBox->GetSelectedData() );
}


//-------------------------------------------------------------------------------------
DWORD CD3DSettingsDlg::GetSelectedD3D11PresentInterval()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_PRESENT_INTERVAL );

    return PtrToUlong( pComboBox->GetSelectedData() );
}

//-------------------------------------------------------------------------------------
bool CD3DSettingsDlg::GetSelectedDebugDeviceValue()
{
    CDXUTCheckBox* pCheckBox = m_Dialog.GetCheckBox( DXUTSETTINGSDLG_D3D11_DEBUG_DEVICE );

    return pCheckBox->GetChecked();
}


//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::SetDeviceClip( bool bDeviceClip )
{
    CDXUTCheckBox* pCheckBox = m_Dialog.GetCheckBox( DXUTSETTINGSDLG_DEVICECLIP );
    pCheckBox->SetChecked( bDeviceClip );
}


//-------------------------------------------------------------------------------------
bool CD3DSettingsDlg::IsDeviceClip()
{
    CDXUTCheckBox* pCheckBox = m_Dialog.GetCheckBox( DXUTSETTINGSDLG_DEVICECLIP );
    return pCheckBox->GetChecked();
}

//--------------------------------------------------------------------------------------
// Updates the resolution list for D3D11
//--------------------------------------------------------------------------------------
HRESULT CD3DSettingsDlg::UpdateD3D11Resolutions()
{

    const DWORD dwWidth = g_DeviceSettings.d3d11.sd.BufferDesc.Width;
    const DWORD dwHeight = g_DeviceSettings.d3d11.sd.BufferDesc.Height;

    // DXUTSETTINGSDLG_D3D11_RESOLUTION
    CDXUTComboBox* pResolutionComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_D3D11_RESOLUTION );
    pResolutionComboBox->RemoveAllItems();

    CD3D11EnumOutputInfo* pOutputInfo = GetCurrentD3D11OutputInfo();
    if( pOutputInfo == NULL )
        return E_FAIL;

    bool bShowAll = m_Dialog.GetCheckBox( DXUTSETTINGSDLG_RESOLUTION_SHOW_ALL )->GetChecked();

    // Get the desktop aspect ratio
    DXGI_MODE_DESC dmDesktop;
    DXUTGetDesktopResolution( g_DeviceSettings.d3d11.AdapterOrdinal, &dmDesktop.Width, &dmDesktop.Height );
    float fDesktopAspectRatio = dmDesktop.Width / ( float )dmDesktop.Height;

    for( int idm = 0; idm < pOutputInfo->displayModeList.GetSize(); idm++ )
    {
        DXGI_MODE_DESC DisplayMode = pOutputInfo->displayModeList.GetAt( idm );
        float fAspect = ( float )DisplayMode.Width / ( float )DisplayMode.Height;

        if( DisplayMode.Format == g_DeviceSettings.d3d11.sd.BufferDesc.Format )
        {
            // If "Show All" is not checked, then hide all resolutions
            // that don't match the aspect ratio of the desktop resolution
            if( bShowAll || ( !bShowAll && fabsf( fDesktopAspectRatio - fAspect ) < 0.05f ) )
            {
                AddD3D11Resolution( DisplayMode.Width, DisplayMode.Height );
            }
        }
    }

    const DWORD dwCurResolution = MAKELONG( g_DeviceSettings.d3d11.sd.BufferDesc.Width,
                                            g_DeviceSettings.d3d11.sd.BufferDesc.Height );

    pResolutionComboBox->SetSelectedByData( ULongToPtr( dwCurResolution ) );


    bool bWindowed = IsWindowed();
    if( bWindowed )
    {
        pResolutionComboBox->RemoveAllItems();
        AddD3D11Resolution( dwWidth, dwHeight );

        pResolutionComboBox->SetSelectedByData( ULongToPtr( MAKELONG( dwWidth, dwHeight ) ) );


    }

    return S_OK;
}


//
//-------------------------------------------------------------------------------------
void CD3DSettingsDlg::AddD3D11DeviceType( D3D_DRIVER_TYPE devType )
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_DEVICE_TYPE );

    if( !pComboBox->ContainsItem( DXUTD3DX11DeviceTypeToString( devType ) ) )
        pComboBox->AddItem( DXUTD3DX11DeviceTypeToString( devType ), ULongToPtr( devType ) );
}


//-------------------------------------------------------------------------------------
D3D_DRIVER_TYPE CD3DSettingsDlg::GetSelectedD3D11DeviceType()
{
    CDXUTComboBox* pComboBox = m_Dialog.GetComboBox( DXUTSETTINGSDLG_DEVICE_TYPE );

    return ( D3D_DRIVER_TYPE )PtrToUlong( pComboBox->GetSelectedData() );
}


void CD3DSettingsDlg::UpdateModeChangeTimeoutText( int nSecRemaining )
{
    const WCHAR StrTimeout[] = L"Reverting to previous display settings in %d seconds";
    const DWORD CchBuf = sizeof( StrTimeout ) / sizeof( WCHAR ) + 16;
    WCHAR buf[CchBuf];

    swprintf_s( buf, CchBuf, StrTimeout, nSecRemaining );

    CDXUTStatic* pStatic = m_RevertModeDialog.GetStatic( DXUTSETTINGSDLG_STATIC_MODE_CHANGE_TIMEOUT );
    pStatic->SetText( buf );
}

//--------------------------------------------------------------------------------------
// Returns the string for the given DXUTDeviceVersion.
//--------------------------------------------------------------------------------------
WCHAR* DXUTAPIVersionToString( DXUTDeviceVersion version )
{
    switch( version )
    {
        case DXUT_D3D9_DEVICE:
            return L"Direct3D 9";
		case DXUT_D3D11_DEVICE:
			return L"Direct3D 11";
        default:
            return L"Unknown version";
    }
}


//--------------------------------------------------------------------------------------
// Returns the string for the given D3DDEVTYPE.
//--------------------------------------------------------------------------------------
WCHAR* DXUTD3DDeviceTypeToString( D3DDEVTYPE devType )
{
    switch( devType )
    {
        case D3DDEVTYPE_HAL:
            return L"D3DDEVTYPE_HAL";
        case D3DDEVTYPE_SW:
            return L"D3DDEVTYPE_SW";
        case D3DDEVTYPE_REF:
            return L"D3DDEVTYPE_REF";
        default:
            return L"Unknown devType";
    }
}




//--------------------------------------------------------------------------------------
// Returns the string for the given D3D_DRIVER_TYPE.
//--------------------------------------------------------------------------------------
WCHAR* DXUTD3DX11DeviceTypeToString( D3D_DRIVER_TYPE devType )
{
    switch( devType )
    {
        case D3D_DRIVER_TYPE_HARDWARE:
            return L"D3D_DRIVER_TYPE_HARDWARE";
        case D3D_DRIVER_TYPE_REFERENCE:
            return L"D3D_DRIVER_TYPE_REFERENCE";
        case D3D_DRIVER_TYPE_NULL:
            return L"D3D_DRIVER_TYPE_NULL";
        case D3D_DRIVER_TYPE_WARP:
            return L"D3D_DRIVER_TYPE_WARP";
        default:
            return L"Unknown devType";
    }
}


//--------------------------------------------------------------------------------------
// Returns the string for the given D3DMULTISAMPLE_TYPE.
//--------------------------------------------------------------------------------------
WCHAR* DXUTMultisampleTypeToString( D3DMULTISAMPLE_TYPE MultiSampleType )
{
    switch( MultiSampleType )
    {
        case D3DMULTISAMPLE_NONE:
            return L"D3DMULTISAMPLE_NONE";
        case D3DMULTISAMPLE_NONMASKABLE:
            return L"D3DMULTISAMPLE_NONMASKABLE";
        case D3DMULTISAMPLE_2_SAMPLES:
            return L"D3DMULTISAMPLE_2_SAMPLES";
        case D3DMULTISAMPLE_3_SAMPLES:
            return L"D3DMULTISAMPLE_3_SAMPLES";
        case D3DMULTISAMPLE_4_SAMPLES:
            return L"D3DMULTISAMPLE_4_SAMPLES";
        case D3DMULTISAMPLE_5_SAMPLES:
            return L"D3DMULTISAMPLE_5_SAMPLES";
        case D3DMULTISAMPLE_6_SAMPLES:
            return L"D3DMULTISAMPLE_6_SAMPLES";
        case D3DMULTISAMPLE_7_SAMPLES:
            return L"D3DMULTISAMPLE_7_SAMPLES";
        case D3DMULTISAMPLE_8_SAMPLES:
            return L"D3DMULTISAMPLE_8_SAMPLES";
        case D3DMULTISAMPLE_9_SAMPLES:
            return L"D3DMULTISAMPLE_9_SAMPLES";
        case D3DMULTISAMPLE_10_SAMPLES:
            return L"D3DMULTISAMPLE_10_SAMPLES";
        case D3DMULTISAMPLE_11_SAMPLES:
            return L"D3DMULTISAMPLE_11_SAMPLES";
        case D3DMULTISAMPLE_12_SAMPLES:
            return L"D3DMULTISAMPLE_12_SAMPLES";
        case D3DMULTISAMPLE_13_SAMPLES:
            return L"D3DMULTISAMPLE_13_SAMPLES";
        case D3DMULTISAMPLE_14_SAMPLES:
            return L"D3DMULTISAMPLE_14_SAMPLES";
        case D3DMULTISAMPLE_15_SAMPLES:
            return L"D3DMULTISAMPLE_15_SAMPLES";
        case D3DMULTISAMPLE_16_SAMPLES:
            return L"D3DMULTISAMPLE_16_SAMPLES";
        default:
            return L"Unknown Multisample Type";
    }
}


//--------------------------------------------------------------------------------------
// Returns the string for the given vertex processing type
//--------------------------------------------------------------------------------------
WCHAR* DXUTVertexProcessingTypeToString( DWORD vpt )
{
    switch( vpt )
    {
        case D3DCREATE_SOFTWARE_VERTEXPROCESSING:
            return L"Software vertex processing";
        case D3DCREATE_MIXED_VERTEXPROCESSING:
            return L"Mixed vertex processing";
        case D3DCREATE_HARDWARE_VERTEXPROCESSING:
            return L"Hardware vertex processing";
        case D3DCREATE_PUREDEVICE:
            return L"Pure hardware vertex processing";
        default:
            return L"Unknown vertex processing type";
    }
}


//--------------------------------------------------------------------------------------
// Returns the string for the given present interval.
//--------------------------------------------------------------------------------------
WCHAR* DXUTPresentIntervalToString( UINT pi )
{
    switch( pi )
    {
        case D3DPRESENT_INTERVAL_IMMEDIATE:
            return L"D3DPRESENT_INTERVAL_IMMEDIATE";
        case D3DPRESENT_INTERVAL_DEFAULT:
            return L"D3DPRESENT_INTERVAL_DEFAULT";
        case D3DPRESENT_INTERVAL_ONE:
            return L"D3DPRESENT_INTERVAL_ONE";
        case D3DPRESENT_INTERVAL_TWO:
            return L"D3DPRESENT_INTERVAL_TWO";
        case D3DPRESENT_INTERVAL_THREE:
            return L"D3DPRESENT_INTERVAL_THREE";
        case D3DPRESENT_INTERVAL_FOUR:
            return L"D3DPRESENT_INTERVAL_FOUR";
        default:
            return L"Unknown PresentInterval";
    }
}


