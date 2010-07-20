//--------------------------------------------------------------------------------------
// File: DXUTDevice11.h
//
// Enumerates D3D adapters, devices, modes, etc.
//
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------
#pragma once
#ifndef DXUT_DEVICE11_H
#define DXUT_DEVICE11_H

void DXUTApplyDefaultDeviceSettings(DXUTDeviceSettings *modifySettings);

//--------------------------------------------------------------------------------------
// Functions to get bit depth from formats
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTGetD3D11AdapterDisplayMode( UINT AdapterOrdinal, UINT Output, DXGI_MODE_DESC* pModeDesc ); 




//--------------------------------------------------------------------------------------
// Optional memory create/destory functions.  If not call, these will be called automatically
//--------------------------------------------------------------------------------------
HRESULT WINAPI DXUTCreateD3D11Enumeration();
void WINAPI DXUTDestroyD3D11Enumeration();




//--------------------------------------------------------------------------------------
// Forward declarations
//--------------------------------------------------------------------------------------
class CD3D11EnumAdapterInfo;
class CD3D11EnumDeviceInfo;
class CD3D11EnumOutputInfo;
struct CD3D11EnumDeviceSettingsCombo;



//--------------------------------------------------------------------------------------
// Enumerates available Direct3D10 adapters, devices, modes, etc.
// Use DXUTGetD3D9Enumeration() to access global instance
//--------------------------------------------------------------------------------------
class CD3D11Enumeration
{
public:
    // These should be called before Enumerate(). 
    //
    // Use these calls and the IsDeviceAcceptable to control the contents of 
    // the enumeration object, which affects the device selection and the device settings dialog.
    void SetResolutionMinMax( UINT nMinWidth, UINT nMinHeight, UINT nMaxWidth, UINT nMaxHeight );  
    void SetRefreshMinMax( UINT nMin, UINT nMax );
    void SetForceFeatureLevel( D3D_FEATURE_LEVEL forceFL) {
        g_forceFL = forceFL;
    };
    void SetMultisampleQualityMax( UINT nMax );
    CGrowableArray<D3DFORMAT>* GetPossibleDepthStencilFormatList();
    void ResetPossibleDepthStencilFormats();
    void SetEnumerateAllAdapterFormats( bool bEnumerateAllAdapterFormats );
    
    // Call Enumerate() to enumerate available D3D11 adapters, devices, modes, etc.
    bool HasEnumerated() { return m_bHasEnumerated; }
    HRESULT Enumerate( LPDXUTCALLBACKISD3D11DEVICEACCEPTABLE IsD3D11DeviceAcceptableFunc,
                       void* pIsD3D11DeviceAcceptableFuncUserContext );

    // These should be called after Enumerate() is called
    CGrowableArray<CD3D11EnumAdapterInfo*>*  GetAdapterInfoList();
    CD3D11EnumAdapterInfo*                   GetAdapterInfo( UINT AdapterOrdinal );
    CD3D11EnumDeviceInfo*                    GetDeviceInfo( UINT AdapterOrdinal, D3D_DRIVER_TYPE DeviceType );
    CD3D11EnumOutputInfo*                    GetOutputInfo( UINT AdapterOrdinal, UINT Output );
    CD3D11EnumDeviceSettingsCombo*           GetDeviceSettingsCombo( DXUTD3D11DeviceSettings* pDeviceSettings ) { return GetDeviceSettingsCombo( pDeviceSettings->AdapterOrdinal, pDeviceSettings->DriverType, pDeviceSettings->Output, pDeviceSettings->sd.BufferDesc.Format, pDeviceSettings->sd.Windowed ); }
    CD3D11EnumDeviceSettingsCombo*           GetDeviceSettingsCombo( UINT AdapterOrdinal, D3D_DRIVER_TYPE DeviceType, UINT Output, DXGI_FORMAT BackBufferFormat, BOOL Windowed );

    ~CD3D11Enumeration();

private:
    friend HRESULT WINAPI DXUTCreateD3D11Enumeration();

    // Use DXUTGetD3D11Enumeration() to access global instance
    CD3D11Enumeration();

    bool m_bHasEnumerated;
    LPDXUTCALLBACKISD3D11DEVICEACCEPTABLE m_IsD3D11DeviceAcceptableFunc;
    void* m_pIsD3D11DeviceAcceptableFuncUserContext;

    CGrowableArray<DXGI_FORMAT> m_DepthStencilPossibleList;

    UINT m_nMinWidth;
    UINT m_nMaxWidth;
    UINT m_nMinHeight;
    UINT m_nMaxHeight;
    UINT m_nRefreshMin;
    UINT m_nRefreshMax;
    UINT m_nMultisampleQualityMax;
    bool m_bEnumerateAllAdapterFormats;
    D3D_FEATURE_LEVEL g_forceFL;

    // Array of CD3D9EnumAdapterInfo* with unique AdapterOrdinals
    CGrowableArray<CD3D11EnumAdapterInfo*> m_AdapterInfoList;

    HRESULT EnumerateOutputs( CD3D11EnumAdapterInfo *pAdapterInfo );
    HRESULT EnumerateDevices( CD3D11EnumAdapterInfo *pAdapterInfo );
    HRESULT EnumerateDeviceCombos( IDXGIFactory1 *pFactory, CD3D11EnumAdapterInfo* pAdapterInfo );
    HRESULT EnumerateDeviceCombosNoAdapter( CD3D11EnumAdapterInfo* pAdapterInfo );
    
    HRESULT EnumerateDisplayModes( CD3D11EnumOutputInfo *pOutputInfo );
    void BuildMultiSampleQualityList( DXGI_FORMAT fmt, CD3D11EnumDeviceSettingsCombo* pDeviceCombo );
    void ClearAdapterInfoList();
};

CD3D11Enumeration* WINAPI DXUTGetD3D11Enumeration(bool bForceEnumerate = false, bool EnumerateAllAdapterFormats = false, D3D_FEATURE_LEVEL forceFL = ((D3D_FEATURE_LEVEL )0)  );


#define DXGI_MAX_DEVICE_IDENTIFIER_STRING 128

//--------------------------------------------------------------------------------------
// A class describing an adapter which contains a unique adapter ordinal 
// that is installed on the system
//--------------------------------------------------------------------------------------
class CD3D11EnumAdapterInfo
{
    const CD3D11EnumAdapterInfo &operator = ( const CD3D11EnumAdapterInfo &rhs );

public:
    ~CD3D11EnumAdapterInfo();

    UINT AdapterOrdinal;
    DXGI_ADAPTER_DESC AdapterDesc;
    WCHAR szUniqueDescription[DXGI_MAX_DEVICE_IDENTIFIER_STRING];
    IDXGIAdapter *m_pAdapter;
    bool bAdapterUnavailable;

    CGrowableArray<CD3D11EnumOutputInfo*> outputInfoList; // Array of CD3D11EnumOutputInfo*
    CGrowableArray<CD3D11EnumDeviceInfo*> deviceInfoList; // Array of CD3D11EnumDeviceInfo*
    // List of CD3D11EnumDeviceSettingsCombo* with a unique set 
    // of BackBufferFormat, and Windowed
    CGrowableArray<CD3D11EnumDeviceSettingsCombo*> deviceSettingsComboList;
};


class CD3D11EnumOutputInfo
{
    const CD3D11EnumOutputInfo &operator = ( const CD3D11EnumOutputInfo &rhs );

public:
    ~CD3D11EnumOutputInfo();

    UINT AdapterOrdinal;
    UINT Output;
    IDXGIOutput* m_pOutput;
    DXGI_OUTPUT_DESC Desc;

    CGrowableArray <DXGI_MODE_DESC> displayModeList; // Array of supported D3DDISPLAYMODEs
};


//--------------------------------------------------------------------------------------
// A class describing a Direct3D10 device that contains a 
//       unique supported driver type
//--------------------------------------------------------------------------------------
class CD3D11EnumDeviceInfo
{
    const CD3D11EnumDeviceInfo& operator =( const CD3D11EnumDeviceInfo& rhs );

public:
    ~CD3D11EnumDeviceInfo();

    UINT AdapterOrdinal;
    D3D_DRIVER_TYPE DeviceType;
    D3D_FEATURE_LEVEL SelectedLevel;
    D3D_FEATURE_LEVEL MaxLevel;
    BOOL ComputeShaders_Plus_RawAndStructuredBuffers_Via_Shader_4_x;
};


//--------------------------------------------------------------------------------------
// A struct describing device settings that contains a unique combination of 
// adapter format, back buffer format, and windowed that is compatible with a 
// particular Direct3D device and the app.
//--------------------------------------------------------------------------------------
struct CD3D11EnumDeviceSettingsCombo
{
    UINT AdapterOrdinal;
    D3D_DRIVER_TYPE DeviceType;
    DXGI_FORMAT BackBufferFormat;
    BOOL Windowed;
    UINT Output;

    CGrowableArray <UINT> multiSampleCountList; // List of valid sampling counts (multisampling)
    CGrowableArray <UINT> multiSampleQualityList; // List of number of quality levels for each multisample count

    CD3D11EnumAdapterInfo* pAdapterInfo;
    CD3D11EnumDeviceInfo* pDeviceInfo;
    CD3D11EnumOutputInfo* pOutputInfo;
};

float   DXUTRankD3D11DeviceCombo( CD3D11EnumDeviceSettingsCombo* pDeviceSettingsCombo, 
                                 DXUTD3D11DeviceSettings* pOptimalDeviceSettings, 
                                 DXGI_MODE_DESC* pAdapterDisplayMode, 
                                 int &bestModeIndex,
                                 int &bestMSAAIndex
                                 );




#endif


