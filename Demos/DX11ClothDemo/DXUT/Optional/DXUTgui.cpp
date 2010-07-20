//--------------------------------------------------------------------------------------
// File: DXUTgui.cpp
//
// Copyright (c) Microsoft Corporation. All rights reserved.
//--------------------------------------------------------------------------------------
#include "DXUT.h"
#include "DXUTgui.h"
#include "DXUTsettingsDlg.h"
#include "DXUTres.h"

#include "SDKMisc.h"

#undef min // use __min instead
#undef max // use __max instead

#ifndef WM_XBUTTONDOWN
#define WM_XBUTTONDOWN 0x020B // (not always defined)
#endif
#ifndef WM_XBUTTONUP
#define WM_XBUTTONUP 0x020C // (not always defined)
#endif
#ifndef WM_MOUSEWHEEL
#define WM_MOUSEWHEEL 0x020A // (not always defined)
#endif
#ifndef WHEEL_DELTA
#define WHEEL_DELTA 120 // (not always defined)
#endif

// Minimum scroll bar thumb size
#define SCROLLBAR_MINTHUMBSIZE 8

// Delay and repeat period when clicking on the scroll bar arrows
#define SCROLLBAR_ARROWCLICK_DELAY  0.33
#define SCROLLBAR_ARROWCLICK_REPEAT 0.05

#define DXUT_NEAR_BUTTON_DEPTH 0.6f
#define DXUT_FAR_BUTTON_DEPTH 0.8f

#define DXUT_MAX_GUI_SPRITES 500

D3DCOLORVALUE D3DCOLOR_TO_D3DCOLORVALUE( D3DCOLOR c )
{
    D3DCOLORVALUE cv =
    {
        ( ( c >> 16 ) & 0xFF ) / 255.0f,
        ( ( c >> 8 ) & 0xFF ) / 255.0f,
        ( c & 0xFF ) / 255.0f,
        ( ( c >> 24 ) & 0xFF ) / 255.0f
    };
    return cv;
}

#define UNISCRIBE_DLLNAME L"usp10.dll"

#define GETPROCADDRESS( Module, APIName, Temp ) \
    Temp = GetProcAddress( Module, #APIName ); \
    if( Temp ) \
        *(FARPROC*)&_##APIName = Temp

#define PLACEHOLDERPROC( APIName ) \
    _##APIName = Dummy_##APIName

#define IMM32_DLLNAME L"imm32.dll"
#define VER_DLLNAME L"version.dll"

CHAR g_strUIEffectFile[] = \
    "Texture2D g_Texture;"\
    ""\
    "SamplerState Sampler"\
    "{"\
    "    Filter = MIN_MAG_MIP_LINEAR;"\
    "    AddressU = Wrap;"\
    "    AddressV = Wrap;"\
    "};"\
    ""\
    "BlendState UIBlend"\
    "{"\
    "    AlphaToCoverageEnable = FALSE;"\
    "    BlendEnable[0] = TRUE;"\
    "    SrcBlend = SRC_ALPHA;"\
    "    DestBlend = INV_SRC_ALPHA;"\
    "    BlendOp = ADD;"\
    "    SrcBlendAlpha = ONE;"\
    "    DestBlendAlpha = ZERO;"\
    "    BlendOpAlpha = ADD;"\
    "    RenderTargetWriteMask[0] = 0x0F;"\
    "};"\
    ""\
    "BlendState NoBlending"\
    "{"\
    "    BlendEnable[0] = FALSE;"\
    "    RenderTargetWriteMask[0] = 0x0F;"\
    "};"\
    ""\
    "DepthStencilState DisableDepth"\
    "{"\
    "    DepthEnable = false;"\
    "};"\
    "DepthStencilState EnableDepth"\
    "{"\
    "    DepthEnable = true;"\
    "};"\
    "struct VS_OUTPUT"\
    "{"\
    "    float4 Pos : POSITION;"\
    "    float4 Dif : COLOR;"\
    "    float2 Tex : TEXCOORD;"\
    "};"\
    ""\
    "VS_OUTPUT VS( float3 vPos : POSITION,"\
    "              float4 Dif : COLOR,"\
    "              float2 vTexCoord0 : TEXCOORD )"\
    "{"\
    "    VS_OUTPUT Output;"\
    ""\
    "    Output.Pos = float4( vPos, 1.0f );"\
    "    Output.Dif = Dif;"\
    "    Output.Tex = vTexCoord0;"\
    ""\
    "    return Output;"\
    "}"\
    ""\
    "float4 PS( VS_OUTPUT In ) : SV_Target"\
    "{"\
    "    return g_Texture.Sample( Sampler, In.Tex ) * In.Dif;"\
    "}"\
    ""\
    "float4 PSUntex( VS_OUTPUT In ) : SV_Target"\
    "{"\
    "    return In.Dif;"\
    "}"\
    ""\
    "technique10 RenderUI"\
    "{"\
    "    pass P0"\
    "    {"\
    "        SetVertexShader( CompileShader( vs_4_0, VS() ) );"\
    "        SetGeometryShader( NULL );"\
    "        SetPixelShader( CompileShader( ps_4_0, PS() ) );"\
    "        SetDepthStencilState( DisableDepth, 0 );"\
    "        SetBlendState( UIBlend, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );"\
    "    }"\
    "}"\
    "technique10 RenderUIUntex"\
    "{"\
    "    pass P0"\
    "    {"\
    "        SetVertexShader( CompileShader( vs_4_0, VS() ) );"\
    "        SetGeometryShader( NULL );"\
    "        SetPixelShader( CompileShader( ps_4_0, PSUntex() ) );"\
    "        SetDepthStencilState( DisableDepth, 0 );"\
    "        SetBlendState( UIBlend, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );"\
    "    }"\
    "}"\
    "technique10 RestoreState"\
    "{"\
    "    pass P0"\
    "    {"\
    "        SetDepthStencilState( EnableDepth, 0 );"\
    "        SetBlendState( NoBlending, float4( 0.0f, 0.0f, 0.0f, 0.0f ), 0xFFFFFFFF );"\
    "    }"\
    "}";
const UINT              g_uUIEffectFileSize = sizeof( g_strUIEffectFile );


// DXUT_MAX_EDITBOXLENGTH is the maximum string length allowed in edit boxes,
// including the NULL terminator.
// 
// Uniscribe does not support strings having bigger-than-16-bits length.
// This means that the string must be less than 65536 characters long,
// including the NULL terminator.
#define DXUT_MAX_EDITBOXLENGTH 0xFFFF


double                  CDXUTDialog::s_fTimeRefresh = 0.0f;
CDXUTControl*           CDXUTDialog::s_pControlFocus = NULL;        // The control which has focus
CDXUTControl*           CDXUTDialog::s_pControlPressed = NULL;      // The control currently pressed


struct DXUT_SCREEN_VERTEX
{
    float x, y, z, h;
    D3DCOLOR color;
    float tu, tv;

    static DWORD FVF;
};
DWORD                   DXUT_SCREEN_VERTEX::FVF = D3DFVF_XYZRHW | D3DFVF_DIFFUSE | D3DFVF_TEX1;


struct DXUT_SCREEN_VERTEX_UNTEX
{
    float x, y, z, h;
    D3DCOLOR color;

    static DWORD FVF;
};
DWORD                   DXUT_SCREEN_VERTEX_UNTEX::FVF = D3DFVF_XYZRHW | D3DFVF_DIFFUSE;


struct DXUT_SCREEN_VERTEX_10
{
    float x, y, z;
    D3DCOLORVALUE color;
    float tu, tv;
};


inline int RectWidth( RECT& rc )
{
    return ( ( rc ).right - ( rc ).left );
}
inline int RectHeight( RECT& rc )
{
    return ( ( rc ).bottom - ( rc ).top );
}


HRESULT InitFont11( ID3D11Device* pd3d11Device, ID3D11InputLayout* pInputLayout );
void EndFont11();

//--------------------------------------------------------------------------------------
// CDXUTDialog class
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
CDXUTDialog::CDXUTDialog()
{
    m_x = 0;
    m_y = 0;
    m_width = 0;
    m_height = 0;

    m_pManager = NULL;
    m_bVisible = true;
    m_bCaption = false;
    m_bMinimized = false;
    m_bDrag = false;
    m_wszCaption[0] = L'\0';
    m_nCaptionHeight = 18;

    m_colorTopLeft = 0;
    m_colorTopRight = 0;
    m_colorBottomLeft = 0;
    m_colorBottomRight = 0;

    m_pCallbackEvent = NULL;
    m_pCallbackEventUserContext = NULL;

    m_fTimeLastRefresh = 0;

    m_pControlMouseOver = NULL;

    m_pNextDialog = this;
    m_pPrevDialog = this;

    m_nDefaultControlID = 0xffff;
    m_bNonUserEvents = false;
    m_bKeyboardInput = false;
    m_bMouseInput = true;
}


//--------------------------------------------------------------------------------------
CDXUTDialog::~CDXUTDialog()
{
    int i = 0;

    RemoveAllControls();

    m_Fonts.RemoveAll();
    m_Textures.RemoveAll();

    for( i = 0; i < m_DefaultElements.GetSize(); i++ )
    {
        DXUTElementHolder* pElementHolder = m_DefaultElements.GetAt( i );
        SAFE_DELETE( pElementHolder );
    }

    m_DefaultElements.RemoveAll();
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::Init( CDXUTDialogResourceManager* pManager, bool bRegisterDialog )
{
    m_pManager = pManager;
    if( bRegisterDialog )
        pManager->RegisterDialog( this );

    SetTexture( 0, MAKEINTRESOURCE( 0xFFFF ), ( HMODULE )0xFFFF );
    InitDefaultElements();
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::Init( CDXUTDialogResourceManager* pManager, bool bRegisterDialog, LPCWSTR pszControlTextureFilename )
{
    m_pManager = pManager;
    if( bRegisterDialog )
        pManager->RegisterDialog( this );
    SetTexture( 0, pszControlTextureFilename );
    InitDefaultElements();
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::Init( CDXUTDialogResourceManager* pManager, bool bRegisterDialog,
                        LPCWSTR szControlTextureResourceName, HMODULE hControlTextureResourceModule )
{
    m_pManager = pManager;
    if( bRegisterDialog )
        pManager->RegisterDialog( this );

    SetTexture( 0, szControlTextureResourceName, hControlTextureResourceModule );
    InitDefaultElements();
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::SetCallback( PCALLBACKDXUTGUIEVENT pCallback, void* pUserContext )
{
    // If this assert triggers, you need to call CDXUTDialog::Init() first.  This change
    // was made so that the DXUT's GUI could become seperate and optional from DXUT's core.  The 
    // creation and interfacing with CDXUTDialogResourceManager is now the responsibility 
    // of the application if it wishes to use DXUT's GUI.
    assert( m_pManager != NULL && L"To fix call CDXUTDialog::Init() first.  See comments for details." );

    m_pCallbackEvent = pCallback;
    m_pCallbackEventUserContext = pUserContext;
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::RemoveControl( int ID )
{
    for( int i = 0; i < m_Controls.GetSize(); i++ )
    {
        CDXUTControl* pControl = m_Controls.GetAt( i );
        if( pControl->GetID() == ID )
        {
            // Clean focus first
            ClearFocus();

            // Clear references to this control
            if( s_pControlFocus == pControl )
                s_pControlFocus = NULL;
            if( s_pControlPressed == pControl )
                s_pControlPressed = NULL;
            if( m_pControlMouseOver == pControl )
                m_pControlMouseOver = NULL;

            SAFE_DELETE( pControl );
            m_Controls.Remove( i );

            return;
        }
    }
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::RemoveAllControls()
{
    if( s_pControlFocus && s_pControlFocus->m_pDialog == this )
        s_pControlFocus = NULL;
    if( s_pControlPressed && s_pControlPressed->m_pDialog == this )
        s_pControlPressed = NULL;
    m_pControlMouseOver = NULL;

    for( int i = 0; i < m_Controls.GetSize(); i++ )
    {
        CDXUTControl* pControl = m_Controls.GetAt( i );
        SAFE_DELETE( pControl );
    }

    m_Controls.RemoveAll();
}


//--------------------------------------------------------------------------------------
CDXUTDialogResourceManager::CDXUTDialogResourceManager()
{
    // Begin D3D9-specific
    m_pd3d9Device = NULL;
    m_pStateBlock = NULL;
    m_pSprite = NULL;

    // Begin D3D11-specific
    // Shaders
    m_pVSRenderUI11 = NULL;
    m_pPSRenderUI11 = NULL;
    m_pPSRenderUIUntex11 = NULL;

    // States
    m_pDepthStencilStateUI11 = NULL;
    m_pRasterizerStateUI11 = NULL;
    m_pBlendStateUI11 = NULL;
    m_pSamplerStateUI11 = NULL;
    m_pDepthStencilStateStored11 = NULL;
    m_pRasterizerStateStored11 = NULL;
    m_pBlendStateStored11 = NULL;
    m_pSamplerStateStored11 = NULL;

    m_pInputLayout11 = NULL;
    m_pVBScreenQuad11 = NULL;
    m_pSpriteBuffer11 = NULL;
}


//--------------------------------------------------------------------------------------
CDXUTDialogResourceManager::~CDXUTDialogResourceManager()
{
    int i;
    for( i = 0; i < m_FontCache.GetSize(); i++ )
    {
        DXUTFontNode* pFontNode = m_FontCache.GetAt( i );
        SAFE_DELETE( pFontNode );
    }
    m_FontCache.RemoveAll();

    for( i = 0; i < m_TextureCache.GetSize(); i++ )
    {
        DXUTTextureNode* pTextureNode = m_TextureCache.GetAt( i );
        SAFE_DELETE( pTextureNode );
    }
    m_TextureCache.RemoveAll();

    CUniBuffer::Uninitialize();
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialogResourceManager::OnD3D9CreateDevice( LPDIRECT3DDEVICE9 pd3dDevice )
{
    HRESULT hr = S_OK;
    int i = 0;

    m_pd3d9Device = pd3dDevice;

    for( i = 0; i < m_FontCache.GetSize(); i++ )
    {
        hr = CreateFont9( i );
        if( FAILED( hr ) )
            return hr;
    }

    for( i = 0; i < m_TextureCache.GetSize(); i++ )
    {
        hr = CreateTexture9( i );
        if( FAILED( hr ) )
            return hr;
    }

    hr = D3DXCreateSprite( pd3dDevice, &m_pSprite );
    if( FAILED( hr ) )
        return DXUT_ERR( L"D3DXCreateSprite", hr );

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialogResourceManager::OnD3D9ResetDevice()
{
    HRESULT hr = S_OK;

    for( int i = 0; i < m_FontCache.GetSize(); i++ )
    {
        DXUTFontNode* pFontNode = m_FontCache.GetAt( i );

        if( pFontNode->pFont9 )
            pFontNode->pFont9->OnResetDevice();
    }

    if( m_pSprite )
        m_pSprite->OnResetDevice();

    V_RETURN( m_pd3d9Device->CreateStateBlock( D3DSBT_ALL, &m_pStateBlock ) );

    return S_OK;
}

//--------------------------------------------------------------------------------------
bool CDXUTDialogResourceManager::MsgProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    return false;
}


//--------------------------------------------------------------------------------------
void CDXUTDialogResourceManager::OnD3D9LostDevice()
{
    for( int i = 0; i < m_FontCache.GetSize(); i++ )
    {
        DXUTFontNode* pFontNode = m_FontCache.GetAt( i );

        if( pFontNode->pFont9 )
            pFontNode->pFont9->OnLostDevice();
    }

    if( m_pSprite )
        m_pSprite->OnLostDevice();

    SAFE_RELEASE( m_pStateBlock );
}


//--------------------------------------------------------------------------------------
void CDXUTDialogResourceManager::OnD3D9DestroyDevice()
{
    int i = 0;

    m_pd3d9Device = NULL;

    // Release the resources but don't clear the cache, as these will need to be
    // recreated if the device is recreated
    for( i = 0; i < m_FontCache.GetSize(); i++ )
    {
        DXUTFontNode* pFontNode = m_FontCache.GetAt( i );
        SAFE_RELEASE( pFontNode->pFont9 );
    }

    for( i = 0; i < m_TextureCache.GetSize(); i++ )
    {
        DXUTTextureNode* pTextureNode = m_TextureCache.GetAt( i );
        SAFE_RELEASE( pTextureNode->pTexture9 );
    }

    SAFE_RELEASE( m_pSprite );
}



HRESULT CDXUTDialogResourceManager::OnD3D11CreateDevice( ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3d11DeviceContext )
{
    m_pd3d11Device = pd3dDevice;
    m_pd3d11DeviceContext = pd3d11DeviceContext;

    HRESULT hr = S_OK;

    // Compile Shaders
    ID3DBlob* pVSBlob = NULL;
    ID3DBlob* pPSBlob = NULL;
    ID3DBlob* pPSUntexBlob = NULL;
    V_RETURN( D3DCompile( g_strUIEffectFile, g_uUIEffectFileSize, "none", NULL, NULL, "VS", "vs_4_0_level_9_1", 
         D3D10_SHADER_ENABLE_BACKWARDS_COMPATIBILITY, 0, &pVSBlob, NULL ) );
    V_RETURN( D3DCompile( g_strUIEffectFile, g_uUIEffectFileSize, "none", NULL, NULL, "PS", "ps_4_0_level_9_1", 
         D3D10_SHADER_ENABLE_BACKWARDS_COMPATIBILITY, 0, &pPSBlob, NULL ) );
    V_RETURN( D3DCompile( g_strUIEffectFile, g_uUIEffectFileSize, "none", NULL, NULL, "PSUntex", "ps_4_0_level_9_1", 
         D3D10_SHADER_ENABLE_BACKWARDS_COMPATIBILITY, 0, &pPSUntexBlob, NULL ) );
//D3D10_SHADER_ENABLE_STRICTNESS

    // Create Shaders
    V_RETURN( pd3dDevice->CreateVertexShader( pVSBlob->GetBufferPointer(), pVSBlob->GetBufferSize(), NULL, &m_pVSRenderUI11 ) );
    V_RETURN( pd3dDevice->CreatePixelShader( pPSBlob->GetBufferPointer(), pPSBlob->GetBufferSize(), NULL, &m_pPSRenderUI11 ) );
    V_RETURN( pd3dDevice->CreatePixelShader( pPSUntexBlob->GetBufferPointer(), pPSUntexBlob->GetBufferSize(), NULL, &m_pPSRenderUIUntex11 ) );

    // States
    D3D11_DEPTH_STENCIL_DESC DSDesc;
    ZeroMemory( &DSDesc, sizeof( D3D11_DEPTH_STENCIL_DESC ) );
    DSDesc.DepthEnable = FALSE;
    DSDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
    DSDesc.DepthFunc = D3D11_COMPARISON_LESS;
    DSDesc.StencilEnable = FALSE;
    V_RETURN( pd3dDevice->CreateDepthStencilState( &DSDesc, &m_pDepthStencilStateUI11 ) );

    D3D11_RASTERIZER_DESC RSDesc;
    RSDesc.AntialiasedLineEnable = FALSE;
    RSDesc.CullMode = D3D11_CULL_BACK;
    RSDesc.DepthBias = 0;
    RSDesc.DepthBiasClamp = 0.0f;
    RSDesc.DepthClipEnable = TRUE;
    RSDesc.FillMode = D3D11_FILL_SOLID;
    RSDesc.FrontCounterClockwise = FALSE;
    RSDesc.MultisampleEnable = TRUE;
    RSDesc.ScissorEnable = FALSE;
    RSDesc.SlopeScaledDepthBias = 0.0f;
    V_RETURN( pd3dDevice->CreateRasterizerState( &RSDesc, &m_pRasterizerStateUI11 ) );

    D3D11_BLEND_DESC BSDesc;
    ZeroMemory( &BSDesc, sizeof( D3D11_BLEND_DESC ) );
    
    BSDesc.RenderTarget[0].BlendEnable = TRUE;
    BSDesc.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
    BSDesc.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
    BSDesc.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
    BSDesc.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
    BSDesc.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
    BSDesc.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
    BSDesc.RenderTarget[0].RenderTargetWriteMask = 0x0F;

    V_RETURN( pd3dDevice->CreateBlendState( &BSDesc, &m_pBlendStateUI11 ) );

    D3D11_SAMPLER_DESC SSDesc;
    ZeroMemory( &SSDesc, sizeof( D3D11_SAMPLER_DESC ) );
    SSDesc.Filter = D3D11_FILTER_ANISOTROPIC   ;
    SSDesc.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
    SSDesc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
    SSDesc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
    SSDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
    SSDesc.MaxAnisotropy = 16;
    SSDesc.MinLOD = 0;
    SSDesc.MaxLOD = D3D11_FLOAT32_MAX;
    if ( pd3dDevice->GetFeatureLevel() < D3D_FEATURE_LEVEL_9_3 ) {
        SSDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
        SSDesc.MaxAnisotropy = 0;
    }
    V_RETURN( pd3dDevice->CreateSamplerState( &SSDesc, &m_pSamplerStateUI11 ) );

    // Create the font and texture objects in the cache arrays.
    int i = 0;
    for( i = 0; i < m_FontCache.GetSize(); i++ )
    {
        hr = CreateFont11( i );
        if( FAILED( hr ) )
            return hr;
    }

    for( i = 0; i < m_TextureCache.GetSize(); i++ )
    {
        hr = CreateTexture11( i );
        if( FAILED( hr ) )
            return hr;
    }

    // Create input layout
    const D3D11_INPUT_ELEMENT_DESC layout[] =
        {
            { "POSITION",  0, DXGI_FORMAT_R32G32B32_FLOAT,    0, 0,  D3D11_INPUT_PER_VERTEX_DATA, 0 },
            { "COLOR",     0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
            { "TEXCOORD",  0, DXGI_FORMAT_R32G32_FLOAT,       0, 28, D3D11_INPUT_PER_VERTEX_DATA, 0 },
        };

    V_RETURN( pd3dDevice->CreateInputLayout( layout, ARRAYSIZE( layout ), pVSBlob->GetBufferPointer(), pVSBlob->GetBufferSize(), &m_pInputLayout11 ) );

    // Release the blobs
    SAFE_RELEASE( pVSBlob );
    SAFE_RELEASE( pPSBlob );
    SAFE_RELEASE( pPSUntexBlob );

    // Create a vertex buffer quad for rendering later
    D3D11_BUFFER_DESC BufDesc;
    BufDesc.ByteWidth = sizeof( DXUT_SCREEN_VERTEX_10 ) * 4;
    BufDesc.Usage = D3D11_USAGE_DYNAMIC;
    BufDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    BufDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    BufDesc.MiscFlags = 0;
    V_RETURN( pd3dDevice->CreateBuffer( &BufDesc, NULL, &m_pVBScreenQuad11 ) );

    // Init the D3D11 font
    InitFont11( pd3dDevice, m_pInputLayout11 );

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialogResourceManager::OnD3D11ResizedSwapChain( ID3D11Device* pd3dDevice,
                                                             const DXGI_SURFACE_DESC* pBackBufferSurfaceDesc )
{
    HRESULT hr = S_OK;

    m_nBackBufferWidth = pBackBufferSurfaceDesc->Width;
    m_nBackBufferHeight = pBackBufferSurfaceDesc->Height;

    return hr;
}


//--------------------------------------------------------------------------------------
void CDXUTDialogResourceManager::OnD3D11ReleasingSwapChain()
{
}


//--------------------------------------------------------------------------------------
void CDXUTDialogResourceManager::OnD3D11DestroyDevice()
{
    int i;

    // Release the resources but don't clear the cache, as these will need to be
    // recreated if the device is recreated

    for( i = 0; i < m_TextureCache.GetSize(); i++ )
    {
        DXUTTextureNode* pTextureNode = m_TextureCache.GetAt( i );
        SAFE_RELEASE( pTextureNode->pTexResView11 );
        SAFE_RELEASE( pTextureNode->pTexture11 );
    }

    // D3D11
    SAFE_RELEASE( m_pVBScreenQuad11 );
    SAFE_RELEASE( m_pSpriteBuffer11 );
    m_SpriteBufferBytes11 = 0;
    SAFE_RELEASE( m_pInputLayout11 );

    // Shaders
    SAFE_RELEASE( m_pVSRenderUI11 );
    SAFE_RELEASE( m_pPSRenderUI11 );
    SAFE_RELEASE( m_pPSRenderUIUntex11 );

    // States
    SAFE_RELEASE( m_pDepthStencilStateUI11 );
    SAFE_RELEASE( m_pRasterizerStateUI11 );
    SAFE_RELEASE( m_pBlendStateUI11 );
    SAFE_RELEASE( m_pSamplerStateUI11 );

    SAFE_RELEASE( m_pDepthStencilStateStored11 );
    SAFE_RELEASE( m_pRasterizerStateStored11 );
    SAFE_RELEASE( m_pBlendStateStored11 );
    SAFE_RELEASE( m_pSamplerStateStored11 );

    EndFont11();
}

//--------------------------------------------------------------------------------------
void CDXUTDialogResourceManager::StoreD3D11State( ID3D11DeviceContext* pd3dImmediateContext )
{
    pd3dImmediateContext->OMGetDepthStencilState( &m_pDepthStencilStateStored11, &m_StencilRefStored11 );
    pd3dImmediateContext->RSGetState( &m_pRasterizerStateStored11 );
    pd3dImmediateContext->OMGetBlendState( &m_pBlendStateStored11, m_BlendFactorStored11, &m_SampleMaskStored11 );
    pd3dImmediateContext->PSGetSamplers( 0, 1, &m_pSamplerStateStored11 );
}

//--------------------------------------------------------------------------------------
void CDXUTDialogResourceManager::RestoreD3D11State( ID3D11DeviceContext* pd3dImmediateContext )
{
    pd3dImmediateContext->OMSetDepthStencilState( m_pDepthStencilStateStored11, m_StencilRefStored11 );
    pd3dImmediateContext->RSSetState( m_pRasterizerStateStored11 );
    pd3dImmediateContext->OMSetBlendState( m_pBlendStateStored11, m_BlendFactorStored11, m_SampleMaskStored11 );
    pd3dImmediateContext->PSSetSamplers( 0, 1, &m_pSamplerStateStored11 );

    SAFE_RELEASE( m_pDepthStencilStateStored11 );
    SAFE_RELEASE( m_pRasterizerStateStored11 );
    SAFE_RELEASE( m_pBlendStateStored11 );
    SAFE_RELEASE( m_pSamplerStateStored11 );
}

//--------------------------------------------------------------------------------------
void CDXUTDialogResourceManager::ApplyRenderUI11( ID3D11DeviceContext* pd3dImmediateContext )
{
    // Shaders
    pd3dImmediateContext->VSSetShader( m_pVSRenderUI11, NULL, 0 );
    pd3dImmediateContext->HSSetShader( NULL, NULL, 0 );
    pd3dImmediateContext->DSSetShader( NULL, NULL, 0 );
    pd3dImmediateContext->GSSetShader( NULL, NULL, 0 );
    pd3dImmediateContext->PSSetShader( m_pPSRenderUI11, NULL, 0 );

    // States
    pd3dImmediateContext->OMSetDepthStencilState( m_pDepthStencilStateUI11, 0 );
    pd3dImmediateContext->RSSetState( m_pRasterizerStateUI11 );
    float BlendFactor[4] = { 0, 0, 0, 0 };
    pd3dImmediateContext->OMSetBlendState( m_pBlendStateUI11, BlendFactor, 0xFFFFFFFF );
    pd3dImmediateContext->PSSetSamplers( 0, 1, &m_pSamplerStateUI11 );
}

//--------------------------------------------------------------------------------------
void CDXUTDialogResourceManager::ApplyRenderUIUntex11( ID3D11DeviceContext* pd3dImmediateContext )
{
    // Shaders
    pd3dImmediateContext->VSSetShader( m_pVSRenderUI11, NULL, 0 );
    pd3dImmediateContext->HSSetShader( NULL, NULL, 0 );
    pd3dImmediateContext->DSSetShader( NULL, NULL, 0 );
    pd3dImmediateContext->GSSetShader( NULL, NULL, 0 );
    pd3dImmediateContext->PSSetShader( m_pPSRenderUIUntex11, NULL, 0 );

    // States
    pd3dImmediateContext->OMSetDepthStencilState( m_pDepthStencilStateUI11, 0 );
    pd3dImmediateContext->RSSetState( m_pRasterizerStateUI11 );
    float BlendFactor[4] = { 0, 0, 0, 0 };
    pd3dImmediateContext->OMSetBlendState( m_pBlendStateUI11, BlendFactor, 0xFFFFFFFF );
    pd3dImmediateContext->PSSetSamplers( 0, 1, &m_pSamplerStateUI11 );
}

//--------------------------------------------------------------------------------------
void CDXUTDialogResourceManager::BeginSprites11( )
{
    m_SpriteVertices.Reset();
}

//--------------------------------------------------------------------------------------
void CDXUTDialogResourceManager::EndSprites11( ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3dImmediateContext )
{

    // ensure our buffer size can hold our sprites
    UINT SpriteDataBytes = m_SpriteVertices.GetSize() * sizeof( DXUTSpriteVertex );
    if( m_SpriteBufferBytes11 < SpriteDataBytes )
    {
        SAFE_RELEASE( m_pSpriteBuffer11 );
        m_SpriteBufferBytes11 = SpriteDataBytes;

        D3D11_BUFFER_DESC BufferDesc;
        BufferDesc.ByteWidth = m_SpriteBufferBytes11;
        BufferDesc.Usage = D3D11_USAGE_DYNAMIC;
        BufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        BufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        BufferDesc.MiscFlags = 0;

        pd3dDevice->CreateBuffer( &BufferDesc, NULL, &m_pSpriteBuffer11 );
    }

    // Copy the sprites over
    D3D11_BOX destRegion;
    destRegion.left = 0;
    destRegion.right = SpriteDataBytes;
    destRegion.top = 0;
    destRegion.bottom = 1;
    destRegion.front = 0;
    destRegion.back = 1;
    D3D11_MAPPED_SUBRESOURCE MappedResource;
    if ( S_OK == pd3dImmediateContext->Map( m_pSpriteBuffer11, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource ) ) { 
        CopyMemory( MappedResource.pData, (void*)m_SpriteVertices.GetData(), SpriteDataBytes );
        pd3dImmediateContext->Unmap(m_pSpriteBuffer11, 0);
    }

    // Draw
    UINT Stride = sizeof( DXUTSpriteVertex );
    UINT Offset = 0;
    pd3dImmediateContext->IASetVertexBuffers( 0, 1, &m_pSpriteBuffer11, &Stride, &Offset );
    pd3dImmediateContext->IASetInputLayout( m_pInputLayout11 );
    pd3dImmediateContext->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST );
    pd3dImmediateContext->Draw( m_SpriteVertices.GetSize(), 0 );

    m_SpriteVertices.Reset();
}

//--------------------------------------------------------------------------------------
bool CDXUTDialogResourceManager::RegisterDialog( CDXUTDialog* pDialog )
{
    // Check that the dialog isn't already registered.
    for( int i = 0; i < m_Dialogs.GetSize(); ++i )
        if( m_Dialogs.GetAt( i ) == pDialog )
            return true;

    // Add to the list.
    if( FAILED( m_Dialogs.Add( pDialog ) ) )
        return false;

    // Set up next and prev pointers.
    if( m_Dialogs.GetSize() > 1 )
        m_Dialogs[m_Dialogs.GetSize() - 2]->SetNextDialog( pDialog );
    m_Dialogs[m_Dialogs.GetSize() - 1]->SetNextDialog( m_Dialogs[0] );

    return true;
}


//--------------------------------------------------------------------------------------
void CDXUTDialogResourceManager::UnregisterDialog( CDXUTDialog* pDialog )
{
    // Search for the dialog in the list.
    for( int i = 0; i < m_Dialogs.GetSize(); ++i )
        if( m_Dialogs.GetAt( i ) == pDialog )
        {
            m_Dialogs.Remove( i );
            if( m_Dialogs.GetSize() > 0 )
            {
                int l, r;

                if( 0 == i )
                    l = m_Dialogs.GetSize() - 1;
                else
                    l = i - 1;

                if( m_Dialogs.GetSize() == i )
                    r = 0;
                else
                    r = i;

                m_Dialogs[l]->SetNextDialog( m_Dialogs[r] );
            }
            return;
        }
}


//--------------------------------------------------------------------------------------
void CDXUTDialogResourceManager::EnableKeyboardInputForAllDialogs()
{
    // Enable keyboard input for all registered dialogs
    for( int i = 0; i < m_Dialogs.GetSize(); ++i )
        m_Dialogs[i]->EnableKeyboardInput( true );
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::Refresh()
{
    if( s_pControlFocus )
        s_pControlFocus->OnFocusOut();

    if( m_pControlMouseOver )
        m_pControlMouseOver->OnMouseLeave();

    s_pControlFocus = NULL;
    s_pControlPressed = NULL;
    m_pControlMouseOver = NULL;

    for( int i = 0; i < m_Controls.GetSize(); i++ )
    {
        CDXUTControl* pControl = m_Controls.GetAt( i );
        pControl->Refresh();
    }

    if( m_bKeyboardInput )
        FocusDefaultControl();
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::OnRender( float fElapsedTime )
{
    if( m_pManager->GetD3D9Device() )
        return OnRender9( fElapsedTime );
    else
        return OnRender11( fElapsedTime );
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::OnRender9( float fElapsedTime )
{
    // If this assert triggers, you need to call CDXUTDialogResourceManager::On*Device() from inside
    // the application's device callbacks.  See the SDK samples for an example of how to do this.
    assert( m_pManager->GetD3D9Device() && m_pManager->m_pStateBlock &&
            L"To fix hook up CDXUTDialogResourceManager to device callbacks.  See comments for details" );

    // See if the dialog needs to be refreshed
    if( m_fTimeLastRefresh < s_fTimeRefresh )
    {
        m_fTimeLastRefresh = DXUTGetTime();
        Refresh();
    }

    // For invisible dialog, out now.
    if( !m_bVisible ||
        ( m_bMinimized && !m_bCaption ) )
        return S_OK;

    IDirect3DDevice9* pd3dDevice = m_pManager->GetD3D9Device();

    // Set up a state block here and restore it when finished drawing all the controls
    m_pManager->m_pStateBlock->Capture();

    //pd3dDevice->SetSamplerState(0, D3DSAMP_SRGBTEXTURE, TRUE);
    //pd3dDevice->SetRenderState( D3DRS_SRGBWRITEENABLE, TRUE );
   
    pd3dDevice->SetRenderState( D3DRS_ALPHABLENDENABLE, TRUE );
    pd3dDevice->SetRenderState( D3DRS_SRCBLEND, D3DBLEND_SRCALPHA );
    pd3dDevice->SetRenderState( D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA );
    pd3dDevice->SetRenderState( D3DRS_ALPHATESTENABLE, FALSE );
    pd3dDevice->SetRenderState( D3DRS_SEPARATEALPHABLENDENABLE, FALSE );
    pd3dDevice->SetRenderState( D3DRS_BLENDOP, D3DBLENDOP_ADD );
    pd3dDevice->SetRenderState( D3DRS_COLORWRITEENABLE, D3DCOLORWRITEENABLE_ALPHA | D3DCOLORWRITEENABLE_BLUE |
                                D3DCOLORWRITEENABLE_GREEN | D3DCOLORWRITEENABLE_RED );
    pd3dDevice->SetRenderState( D3DRS_SHADEMODE, D3DSHADE_GOURAUD );
    pd3dDevice->SetRenderState( D3DRS_FOGENABLE, FALSE );
    pd3dDevice->SetRenderState( D3DRS_ZWRITEENABLE, FALSE );
    pd3dDevice->SetRenderState( D3DRS_FILLMODE, D3DFILL_SOLID );
    pd3dDevice->SetRenderState( D3DRS_CULLMODE, D3DCULL_CCW );

    pd3dDevice->SetTextureStageState( 0, D3DTSS_COLOROP, D3DTOP_SELECTARG2 );
    pd3dDevice->SetTextureStageState( 0, D3DTSS_COLORARG2, D3DTA_DIFFUSE );
    pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1 );
    pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAARG1, D3DTA_DIFFUSE );
    pd3dDevice->SetTextureStageState( 0, D3DTSS_RESULTARG, D3DTA_CURRENT );
    pd3dDevice->SetTextureStageState( 1, D3DTSS_COLOROP, D3DTOP_DISABLE );
    pd3dDevice->SetTextureStageState( 1, D3DTSS_ALPHAOP, D3DTOP_DISABLE );

    BOOL bBackgroundIsVisible = ( m_colorTopLeft | m_colorTopRight | m_colorBottomRight | m_colorBottomLeft ) &
        0xff000000;
    if( !m_bMinimized && bBackgroundIsVisible )
    {
        DXUT_SCREEN_VERTEX_UNTEX vertices[4] =
        {
            ( float )m_x,           ( float )m_y,            0.5f, 1.0f, m_colorTopLeft,
            ( float )m_x + m_width, ( float )m_y,            0.5f, 1.0f, m_colorTopRight,
            ( float )m_x + m_width, ( float )m_y + m_height, 0.5f, 1.0f, m_colorBottomRight,
            ( float )m_x,           ( float )m_y + m_height, 0.5f, 1.0f, m_colorBottomLeft,
        };

        pd3dDevice->SetVertexShader( NULL );
        pd3dDevice->SetPixelShader( NULL );

        pd3dDevice->SetRenderState( D3DRS_ZENABLE, FALSE );

        pd3dDevice->SetFVF( DXUT_SCREEN_VERTEX_UNTEX::FVF );
        pd3dDevice->DrawPrimitiveUP( D3DPT_TRIANGLEFAN, 2, vertices, sizeof( DXUT_SCREEN_VERTEX_UNTEX ) );
    }

    pd3dDevice->SetTextureStageState( 0, D3DTSS_COLOROP, D3DTOP_MODULATE );
    pd3dDevice->SetTextureStageState( 0, D3DTSS_COLORARG1, D3DTA_TEXTURE );
    pd3dDevice->SetTextureStageState( 0, D3DTSS_COLORARG2, D3DTA_DIFFUSE );

    pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAOP, D3DTOP_MODULATE );
    pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE );
    pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAARG2, D3DTA_DIFFUSE );

    pd3dDevice->SetSamplerState( 0, D3DSAMP_MINFILTER, D3DTEXF_LINEAR );

    DXUTTextureNode* pTextureNode = GetTexture( 0 );
    pd3dDevice->SetTexture( 0, pTextureNode->pTexture9 );

    m_pManager->m_pSprite->Begin( D3DXSPRITE_DONOTSAVESTATE );

    // Render the caption if it's enabled.
    if( m_bCaption )
    {
        // DrawSprite will offset the rect down by
        // m_nCaptionHeight, so adjust the rect higher
        // here to negate the effect.
        RECT rc =
        {
            0, -m_nCaptionHeight, m_width, 0
        };
        DrawSprite9( &m_CapElement, &rc );
        rc.left += 5; // Make a left margin
        WCHAR wszOutput[256];
        wcscpy_s( wszOutput, 256, m_wszCaption );
        if( m_bMinimized )
            wcscat_s( wszOutput, 256, L" (Minimized)" );
        DrawText9( wszOutput, &m_CapElement, &rc, true );
    }

    // If the dialog is minimized, skip rendering
    // its controls.
    if( !m_bMinimized )
    {
        for( int i = 0; i < m_Controls.GetSize(); i++ )
        {
            CDXUTControl* pControl = m_Controls.GetAt( i );

            // Focused control is drawn last
            if( pControl == s_pControlFocus )
                continue;

            pControl->Render( fElapsedTime );
        }

        if( s_pControlFocus != NULL && s_pControlFocus->m_pDialog == this )
            s_pControlFocus->Render( fElapsedTime );
    }

    m_pManager->m_pSprite->End();

    m_pManager->m_pStateBlock->Apply();

    return S_OK;
}



//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::OnRender11( float fElapsedTime )
{
    // If this assert triggers, you need to call CDXUTDialogResourceManager::On*Device() from inside
    // the application's device callbacks.  See the SDK samples for an example of how to do this.
    assert( m_pManager->GetD3D11Device() &&
            L"To fix hook up CDXUTDialogResourceManager to device callbacks.  See comments for details" );

    // See if the dialog needs to be refreshed
    if( m_fTimeLastRefresh < s_fTimeRefresh )
    {
        m_fTimeLastRefresh = DXUTGetTime();
        Refresh();
    }

    // For invisible dialog, out now.
    if( !m_bVisible ||
        ( m_bMinimized && !m_bCaption ) )
        return S_OK;

    ID3D11Device* pd3dDevice = m_pManager->GetD3D11Device();
    ID3D11DeviceContext* pd3dDeviceContext = m_pManager->GetD3D11DeviceContext();

    // Set up a state block here and restore it when finished drawing all the controls
    m_pManager->StoreD3D11State( pd3dDeviceContext );

    BOOL bBackgroundIsVisible = ( m_colorTopLeft | m_colorTopRight | m_colorBottomRight | m_colorBottomLeft ) &
        0xff000000;
    if( !m_bMinimized && bBackgroundIsVisible )
    {
        // Convert the draw rectangle from screen coordinates to clip space coordinates.
        float Left, Right, Top, Bottom;
        Left = m_x * 2.0f / m_pManager->m_nBackBufferWidth - 1.0f;
        Right = ( m_x + m_width ) * 2.0f / m_pManager->m_nBackBufferWidth - 1.0f;
        Top = 1.0f - m_y * 2.0f / m_pManager->m_nBackBufferHeight;
        Bottom = 1.0f - ( m_y + m_height ) * 2.0f / m_pManager->m_nBackBufferHeight;

        DXUT_SCREEN_VERTEX_10 vertices[4] =
        {
            Left,  Top,    0.5f, D3DCOLOR_TO_D3DCOLORVALUE( m_colorTopLeft ), 0.0f, 0.0f,
            Right, Top,    0.5f, D3DCOLOR_TO_D3DCOLORVALUE( m_colorTopRight ), 1.0f, 0.0f,
            Left,  Bottom, 0.5f, D3DCOLOR_TO_D3DCOLORVALUE( m_colorBottomLeft ), 0.0f, 1.0f,
            Right, Bottom, 0.5f, D3DCOLOR_TO_D3DCOLORVALUE( m_colorBottomRight ), 1.0f, 1.0f,
        };

        //DXUT_SCREEN_VERTEX_10 *pVB;
        D3D11_MAPPED_SUBRESOURCE MappedData;
        if( SUCCEEDED( pd3dDeviceContext->Map( m_pManager->m_pVBScreenQuad11, 0, D3D11_MAP_WRITE_DISCARD,
                                               0, &MappedData ) ) )
        {
            CopyMemory( MappedData.pData, vertices, sizeof( vertices ) );
            pd3dDeviceContext->Unmap( m_pManager->m_pVBScreenQuad11, 0 );
        }

        // Set the quad VB as current
        UINT stride = sizeof( DXUT_SCREEN_VERTEX_10 );
        UINT offset = 0;
        pd3dDeviceContext->IASetVertexBuffers( 0, 1, &m_pManager->m_pVBScreenQuad11, &stride, &offset );
        pd3dDeviceContext->IASetInputLayout( m_pManager->m_pInputLayout11 );
        pd3dDeviceContext->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP );

        // Setup for rendering
        m_pManager->ApplyRenderUIUntex11( pd3dDeviceContext );
        pd3dDeviceContext->Draw( 4, 0 );
    }

    DXUTTextureNode* pTextureNode = GetTexture( 0 );
    pd3dDeviceContext->PSSetShaderResources( 0, 1, &pTextureNode->pTexResView11 );

    // Sort depth back to front
    m_pManager->BeginSprites11();
    BeginText11();

    m_pManager->ApplyRenderUI11( pd3dDeviceContext );

    // Render the caption if it's enabled.
    if( m_bCaption )
    {
        // DrawSprite will offset the rect down by
        // m_nCaptionHeight, so adjust the rect higher
        // here to negate the effect.
        RECT rc = { 0, -m_nCaptionHeight, m_width, 0 };
        DrawSprite11( &m_CapElement, &rc, 0.99f );
        rc.left += 5; // Make a left margin
        WCHAR wszOutput[256];
        wcscpy_s( wszOutput, 256, m_wszCaption );
        if( m_bMinimized )
            wcscat_s( wszOutput, 256, L" (Minimized)" );
        DrawText11( pd3dDevice, pd3dDeviceContext, wszOutput, &m_CapElement, &rc, true );
    }

    // If the dialog is minimized, skip rendering
    // its controls.
    if( !m_bMinimized )
    {
        for( int i = 0; i < m_Controls.GetSize(); i++ )
        {
            CDXUTControl* pControl = m_Controls.GetAt( i );

            // Focused control is drawn last
            if( pControl == s_pControlFocus )
                continue;

            pControl->Render( fElapsedTime );
        }

        if( s_pControlFocus != NULL && s_pControlFocus->m_pDialog == this )
            s_pControlFocus->Render( fElapsedTime );
    }

    // End sprites
    if( m_bCaption )
    {
        m_pManager->EndSprites11( pd3dDevice, pd3dDeviceContext );
        EndText11( pd3dDevice, pd3dDeviceContext );
    }
    m_pManager->RestoreD3D11State( pd3dDeviceContext );

    return S_OK;
}

//--------------------------------------------------------------------------------------
VOID CDXUTDialog::SendEvent( UINT nEvent, bool bTriggeredByUser, CDXUTControl* pControl )
{
    // If no callback has been registered there's nowhere to send the event to
    if( m_pCallbackEvent == NULL )
        return;

    // Discard events triggered programatically if these types of events haven't been
    // enabled
    if( !bTriggeredByUser && !m_bNonUserEvents )
        return;

    m_pCallbackEvent( nEvent, pControl->GetID(), pControl, m_pCallbackEventUserContext );
}


//--------------------------------------------------------------------------------------
int CDXUTDialogResourceManager::AddFont( LPCWSTR strFaceName, LONG height, LONG weight ) 
{
    // See if this font already exists
    for( int i = 0; i < m_FontCache.GetSize(); i++ )
    {
        DXUTFontNode* pFontNode = m_FontCache.GetAt( i );
        size_t nLen = 0;
        nLen = wcsnlen( strFaceName, MAX_PATH);
        if( 0 == _wcsnicmp( pFontNode->strFace, strFaceName, nLen ) &&
            pFontNode->nHeight == height &&
            pFontNode->nWeight == weight )
        {
            return i;
        }
    }

    // Add a new font and try to create it
    DXUTFontNode* pNewFontNode = new DXUTFontNode;
    if( pNewFontNode == NULL )
        return -1;

    ZeroMemory( pNewFontNode, sizeof( DXUTFontNode ) );
    wcscpy_s( pNewFontNode->strFace, MAX_PATH, strFaceName );
    pNewFontNode->nHeight = height;
    pNewFontNode->nWeight = weight;
    m_FontCache.Add( pNewFontNode );

    int iFont = m_FontCache.GetSize() - 1;

    // If a device is available, try to create immediately
    if( m_pd3d9Device )
        CreateFont9( iFont );

    return iFont;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::SetFont( UINT index, LPCWSTR strFaceName, LONG height, LONG weight )
{
    // If this assert triggers, you need to call CDXUTDialog::Init() first.  This change
    // was made so that the DXUT's GUI could become seperate and optional from DXUT's core.  The 
    // creation and interfacing with CDXUTDialogResourceManager is now the responsibility 
    // of the application if it wishes to use DXUT's GUI.
    assert( m_pManager != NULL && L"To fix call CDXUTDialog::Init() first.  See comments for details." );

    // Make sure the list is at least as large as the index being set
    UINT i;
    for( i = m_Fonts.GetSize(); i <= index; i++ )
    {
        m_Fonts.Add( -1 );
    }

    int iFont = m_pManager->AddFont( strFaceName, height, weight );
    m_Fonts.SetAt( index, iFont );

    return S_OK;
}


//--------------------------------------------------------------------------------------
DXUTFontNode* CDXUTDialog::GetFont( UINT index )
{
    if( NULL == m_pManager )
        return NULL;
    return m_pManager->GetFontNode( m_Fonts.GetAt( index ) );
}


//--------------------------------------------------------------------------------------
int CDXUTDialogResourceManager::AddTexture( LPCWSTR strFilename )
{
    // See if this texture already exists
    for( int i = 0; i < m_TextureCache.GetSize(); i++ )
    {
        DXUTTextureNode* pTextureNode = m_TextureCache.GetAt( i );
        size_t nLen = 0;
        nLen = wcsnlen( strFilename, MAX_PATH);
        if( pTextureNode->bFileSource &&  // Sources must match
            0 == _wcsnicmp( pTextureNode->strFilename, strFilename, nLen ) )
        {
            return i;
        }
    }

    // Add a new texture and try to create it
    DXUTTextureNode* pNewTextureNode = new DXUTTextureNode;
    if( pNewTextureNode == NULL )
        return -1;

    ZeroMemory( pNewTextureNode, sizeof( DXUTTextureNode ) );
    pNewTextureNode->bFileSource = true;
    wcscpy_s( pNewTextureNode->strFilename, MAX_PATH, strFilename );

    m_TextureCache.Add( pNewTextureNode );

    int iTexture = m_TextureCache.GetSize() - 1;

    // If a device is available, try to create immediately
    if( m_pd3d9Device )
        CreateTexture9( iTexture );

    return iTexture;
}


//--------------------------------------------------------------------------------------
int CDXUTDialogResourceManager::AddTexture( LPCWSTR strResourceName, HMODULE hResourceModule )
{
    // See if this texture already exists
    for( int i = 0; i < m_TextureCache.GetSize(); i++ )
    {
        DXUTTextureNode* pTextureNode = m_TextureCache.GetAt( i );
        if( !pTextureNode->bFileSource &&      // Sources must match
            pTextureNode->hResourceModule == hResourceModule ) // Module handles must match
        {
            if( IS_INTRESOURCE( strResourceName ) )
            {
                // Integer-based ID
                if( ( INT_PTR )strResourceName == pTextureNode->nResourceID )
                    return i;
            }
            else
            {
                // String-based ID
                size_t nLen = 0;
                nLen = wcsnlen ( strResourceName, MAX_PATH );
                if( 0 == _wcsnicmp( pTextureNode->strFilename, strResourceName, nLen ) )
                    return i;
            }
        }
    }

    // Add a new texture and try to create it
    DXUTTextureNode* pNewTextureNode = new DXUTTextureNode;
    if( pNewTextureNode == NULL )
        return -1;

    ZeroMemory( pNewTextureNode, sizeof( DXUTTextureNode ) );
    pNewTextureNode->hResourceModule = hResourceModule;
    if( IS_INTRESOURCE( strResourceName ) )
    {
        pNewTextureNode->nResourceID = ( int )( size_t )strResourceName;
    }
    else
    {
        pNewTextureNode->nResourceID = 0;
        wcscpy_s( pNewTextureNode->strFilename, MAX_PATH, strResourceName );
    }

    m_TextureCache.Add( pNewTextureNode );

    int iTexture = m_TextureCache.GetSize() - 1;

    // If a device is available, try to create immediately
    if( m_pd3d9Device )
        CreateTexture9( iTexture );

    return iTexture;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::SetTexture( UINT index, LPCWSTR strFilename )
{
    // If this assert triggers, you need to call CDXUTDialog::Init() first.  This change
    // was made so that the DXUT's GUI could become seperate and optional from DXUT's core.  The 
    // creation and interfacing with CDXUTDialogResourceManager is now the responsibility 
    // of the application if it wishes to use DXUT's GUI.
    assert( m_pManager != NULL && L"To fix this, call CDXUTDialog::Init() first.  See comments for details." );

    // Make sure the list is at least as large as the index being set
    for( UINT i = m_Textures.GetSize(); i <= index; i++ )
    {
        m_Textures.Add( -1 );
    }

    int iTexture = m_pManager->AddTexture( strFilename );

    m_Textures.SetAt( index, iTexture );
    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::SetTexture( UINT index, LPCWSTR strResourceName, HMODULE hResourceModule )
{
    // If this assert triggers, you need to call CDXUTDialog::Init() first.  This change
    // was made so that the DXUT's GUI could become seperate and optional from DXUT's core.  The 
    // creation and interfacing with CDXUTDialogResourceManager is now the responsibility 
    // of the application if it wishes to use DXUT's GUI.
    assert( m_pManager != NULL && L"To fix this, call CDXUTDialog::Init() first.  See comments for details." );

    // Make sure the list is at least as large as the index being set
    for( UINT i = m_Textures.GetSize(); i <= index; i++ )
    {
        m_Textures.Add( -1 );
    }

    int iTexture = m_pManager->AddTexture( strResourceName, hResourceModule );

    m_Textures.SetAt( index, iTexture );
    return S_OK;
}


//--------------------------------------------------------------------------------------
DXUTTextureNode* CDXUTDialog::GetTexture( UINT index )
{
    if( NULL == m_pManager )
        return NULL;
    return m_pManager->GetTextureNode( m_Textures.GetAt( index ) );
}



//--------------------------------------------------------------------------------------
bool CDXUTDialog::MsgProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    bool bHandled = false;

    // For invisible dialog, do not handle anything.
    if( !m_bVisible )
        return false;

    // If automation command-line switch is on, enable this dialog's keyboard input
    // upon any key press or mouse click.
    if( DXUTGetAutomation() &&
        ( WM_LBUTTONDOWN == uMsg || WM_LBUTTONDBLCLK == uMsg || WM_KEYDOWN == uMsg ) )
    {
        m_pManager->EnableKeyboardInputForAllDialogs();
    }

    // If caption is enable, check for clicks in the caption area.
    if( m_bCaption )
    {
        if( uMsg == WM_LBUTTONDOWN || uMsg == WM_LBUTTONDBLCLK )
        {
            POINT mousePoint =
            {
                short( LOWORD( lParam ) ), short( HIWORD( lParam ) )
            };

            if( mousePoint.x >= m_x && mousePoint.x < m_x + m_width &&
                mousePoint.y >= m_y && mousePoint.y < m_y + m_nCaptionHeight )
            {
                m_bDrag = true;
                SetCapture( DXUTGetHWND() );
                return true;
            }
        }
        else if( uMsg == WM_LBUTTONUP && m_bDrag )
        {
            POINT mousePoint =
            {
                short( LOWORD( lParam ) ), short( HIWORD( lParam ) )
            };

            if( mousePoint.x >= m_x && mousePoint.x < m_x + m_width &&
                mousePoint.y >= m_y && mousePoint.y < m_y + m_nCaptionHeight )
            {
                ReleaseCapture();
                m_bDrag = false;
                m_bMinimized = !m_bMinimized;
                return true;
            }
        }
    }

    // If the dialog is minimized, don't send any messages to controls.
    if( m_bMinimized )
        return false;

    // If a control is in focus, it belongs to this dialog, and it's enabled, then give
    // it the first chance at handling the message.
    if( s_pControlFocus &&
        s_pControlFocus->m_pDialog == this &&
        s_pControlFocus->GetEnabled() )
    {
        // If the control MsgProc handles it, then we don't.
        if( s_pControlFocus->MsgProc( uMsg, wParam, lParam ) )
            return true;
    }

    switch( uMsg )
    {
        case WM_SIZE:
        case WM_MOVE:
            {
                // Handle sizing and moving messages so that in case the mouse cursor is moved out
                // of an UI control because of the window adjustment, we can properly
                // unhighlight the highlighted control.
                POINT pt =
                {
                    -1, -1
                };
                OnMouseMove( pt );
                break;
            }

        case WM_ACTIVATEAPP:
            // Call OnFocusIn()/OnFocusOut() of the control that currently has the focus
            // as the application is activated/deactivated.  This matches the Windows
            // behavior.
            if( s_pControlFocus &&
                s_pControlFocus->m_pDialog == this &&
                s_pControlFocus->GetEnabled() )
            {
                if( wParam )
                    s_pControlFocus->OnFocusIn();
                else
                    s_pControlFocus->OnFocusOut();
            }
            break;

            // Keyboard messages
        case WM_KEYDOWN:
        case WM_SYSKEYDOWN:
        case WM_KEYUP:
        case WM_SYSKEYUP:
            {
                // If a control is in focus, it belongs to this dialog, and it's enabled, then give
                // it the first chance at handling the message.
                if( s_pControlFocus &&
                    s_pControlFocus->m_pDialog == this &&
                    s_pControlFocus->GetEnabled() )
                {
                    if( s_pControlFocus->HandleKeyboard( uMsg, wParam, lParam ) )
                        return true;
                }

                // Not yet handled, see if this matches a control's hotkey
                // Activate the hotkey if the focus doesn't belong to an
                // edit box.
                if( uMsg == WM_KEYDOWN && ( !s_pControlFocus ||
                                            ( s_pControlFocus->GetType() != DXUT_CONTROL_EDITBOX
                                              && s_pControlFocus->GetType() != DXUT_CONTROL_IMEEDITBOX ) ) )
                {
                    for( int i = 0; i < m_Controls.GetSize(); i++ )
                    {
                        CDXUTControl* pControl = m_Controls.GetAt( i );
                        if( pControl->GetHotkey() == wParam )
                        {
                            pControl->OnHotkey();
                            return true;
                        }
                    }
                }

                // Not yet handled, check for focus messages
                if( uMsg == WM_KEYDOWN )
                {
                    // If keyboard input is not enabled, this message should be ignored
                    if( !m_bKeyboardInput )
                        return false;

                    switch( wParam )
                    {
                        case VK_RIGHT:
                        case VK_DOWN:
                            if( s_pControlFocus != NULL )
                            {
                                return OnCycleFocus( true );
                            }
                            break;

                        case VK_LEFT:
                        case VK_UP:
                            if( s_pControlFocus != NULL )
                            {
                                return OnCycleFocus( false );
                            }
                            break;

                        case VK_TAB:
                        {
                            bool bShiftDown = ( ( GetKeyState( VK_SHIFT ) & 0x8000 ) != 0 );
                            return OnCycleFocus( !bShiftDown );
                        }
                    }
                }

                break;
            }


            // Mouse messages
        case WM_MOUSEMOVE:
        case WM_LBUTTONDOWN:
        case WM_LBUTTONUP:
        case WM_MBUTTONDOWN:
        case WM_MBUTTONUP:
        case WM_RBUTTONDOWN:
        case WM_RBUTTONUP:
        case WM_XBUTTONDOWN:
        case WM_XBUTTONUP:
        case WM_LBUTTONDBLCLK:
        case WM_MBUTTONDBLCLK:
        case WM_RBUTTONDBLCLK:
        case WM_XBUTTONDBLCLK:
        case WM_MOUSEWHEEL:
            {
                // If not accepting mouse input, return false to indicate the message should still 
                // be handled by the application (usually to move the camera).
                if( !m_bMouseInput )
                    return false;

                POINT mousePoint =
                {
                    short( LOWORD( lParam ) ), short( HIWORD( lParam ) )
                };
                mousePoint.x -= m_x;
                mousePoint.y -= m_y;

                // If caption is enabled, offset the Y coordinate by the negative of its height.
                if( m_bCaption )
                    mousePoint.y -= m_nCaptionHeight;

                // If a control is in focus, it belongs to this dialog, and it's enabled, then give
                // it the first chance at handling the message.
                if( s_pControlFocus &&
                    s_pControlFocus->m_pDialog == this &&
                    s_pControlFocus->GetEnabled() )
                {
                    if( s_pControlFocus->HandleMouse( uMsg, mousePoint, wParam, lParam ) )
                        return true;
                }

                // Not yet handled, see if the mouse is over any controls
                CDXUTControl* pControl = GetControlAtPoint( mousePoint );
                if( pControl != NULL && pControl->GetEnabled() )
                {
                    bHandled = pControl->HandleMouse( uMsg, mousePoint, wParam, lParam );
                    if( bHandled )
                        return true;
                }
                else
                {
                    // Mouse not over any controls in this dialog, if there was a control
                    // which had focus it just lost it
                    if( uMsg == WM_LBUTTONDOWN &&
                        s_pControlFocus &&
                        s_pControlFocus->m_pDialog == this )
                    {
                        s_pControlFocus->OnFocusOut();
                        s_pControlFocus = NULL;
                    }
                }

                // Still not handled, hand this off to the dialog. Return false to indicate the
                // message should still be handled by the application (usually to move the camera).
                switch( uMsg )
                {
                    case WM_MOUSEMOVE:
                        OnMouseMove( mousePoint );
                        return false;
                }

                break;
            }

        case WM_CAPTURECHANGED:
        {
            // The application has lost mouse capture.
            // The dialog object may not have received
            // a WM_MOUSEUP when capture changed. Reset
            // m_bDrag so that the dialog does not mistakenly
            // think the mouse button is still held down.
            if( ( HWND )lParam != hWnd )
                m_bDrag = false;
        }
    }

    return false;
}

//--------------------------------------------------------------------------------------
CDXUTControl* CDXUTDialog::GetControlAtPoint( POINT pt )
{
    // Search through all child controls for the first one which
    // contains the mouse point
    for( int i = 0; i < m_Controls.GetSize(); i++ )
    {
        CDXUTControl* pControl = m_Controls.GetAt( i );

        if( pControl == NULL )
        {
            continue;
        }

        // We only return the current control if it is visible
        // and enabled.  Because GetControlAtPoint() is used to do mouse
        // hittest, it makes sense to perform this filtering.
        if( pControl->ContainsPoint( pt ) && pControl->GetEnabled() && pControl->GetVisible() )
        {
            return pControl;
        }
    }

    return NULL;
}


//--------------------------------------------------------------------------------------
bool CDXUTDialog::GetControlEnabled( int ID )
{
    CDXUTControl* pControl = GetControl( ID );
    if( pControl == NULL )
        return false;

    return pControl->GetEnabled();
}



//--------------------------------------------------------------------------------------
void CDXUTDialog::SetControlEnabled( int ID, bool bEnabled )
{
    CDXUTControl* pControl = GetControl( ID );
    if( pControl == NULL )
        return;

    pControl->SetEnabled( bEnabled );
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::OnMouseUp( POINT pt )
{
    s_pControlPressed = NULL;
    m_pControlMouseOver = NULL;
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::OnMouseMove( POINT pt )
{
    // Figure out which control the mouse is over now
    CDXUTControl* pControl = GetControlAtPoint( pt );

    // If the mouse is still over the same control, nothing needs to be done
    if( pControl == m_pControlMouseOver )
        return;

    // Handle mouse leaving the old control
    if( m_pControlMouseOver )
        m_pControlMouseOver->OnMouseLeave();

    // Handle mouse entering the new control
    m_pControlMouseOver = pControl;
    if( pControl != NULL )
        m_pControlMouseOver->OnMouseEnter();
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::SetDefaultElement( UINT nControlType, UINT iElement, CDXUTElement* pElement )
{
    // If this Element type already exist in the list, simply update the stored Element
    for( int i = 0; i < m_DefaultElements.GetSize(); i++ )
    {
        DXUTElementHolder* pElementHolder = m_DefaultElements.GetAt( i );

        if( pElementHolder->nControlType == nControlType &&
            pElementHolder->iElement == iElement )
        {
            pElementHolder->Element = *pElement;
            return S_OK;
        }
    }

    // Otherwise, add a new entry
    DXUTElementHolder* pNewHolder;
    pNewHolder = new DXUTElementHolder;
    if( pNewHolder == NULL )
        return E_OUTOFMEMORY;

    pNewHolder->nControlType = nControlType;
    pNewHolder->iElement = iElement;
    pNewHolder->Element = *pElement;

    HRESULT hr = m_DefaultElements.Add( pNewHolder );
    if( FAILED( hr ) )
    {
        delete pNewHolder;
    }
    return hr;
}


//--------------------------------------------------------------------------------------
CDXUTElement* CDXUTDialog::GetDefaultElement( UINT nControlType, UINT iElement )
{
    for( int i = 0; i < m_DefaultElements.GetSize(); i++ )
    {
        DXUTElementHolder* pElementHolder = m_DefaultElements.GetAt( i );

        if( pElementHolder->nControlType == nControlType &&
            pElementHolder->iElement == iElement )
        {
            return &pElementHolder->Element;
        }
    }

    return NULL;
}



//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::AddStatic( int ID, LPCWSTR strText, int x, int y, int width, int height, bool bIsDefault,
                                CDXUTStatic** ppCreated )
{
    HRESULT hr = S_OK;

    CDXUTStatic* pStatic = new CDXUTStatic( this );

    if( ppCreated != NULL )
        *ppCreated = pStatic;

    if( pStatic == NULL )
        return E_OUTOFMEMORY;

    hr = AddControl( pStatic );
    if( FAILED( hr ) )
        return hr;

    // Set the ID and list index
    pStatic->SetID( ID );
    pStatic->SetText( strText );
    pStatic->SetLocation( x, y );
    pStatic->SetSize( width, height );
    pStatic->m_bIsDefault = bIsDefault;

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::AddButton( int ID, LPCWSTR strText, int x, int y, int width, int height, UINT nHotkey,
                                bool bIsDefault, CDXUTButton** ppCreated )
{
    HRESULT hr = S_OK;

    CDXUTButton* pButton = new CDXUTButton( this );

    if( ppCreated != NULL )
        *ppCreated = pButton;

    if( pButton == NULL )
        return E_OUTOFMEMORY;

    hr = AddControl( pButton );
    if( FAILED( hr ) )
        return hr;

    // Set the ID and list index
    pButton->SetID( ID );
    pButton->SetText( strText );
    pButton->SetLocation( x, y );
    pButton->SetSize( width, height );
    pButton->SetHotkey( nHotkey );
    pButton->m_bIsDefault = bIsDefault;

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::AddCheckBox( int ID, LPCWSTR strText, int x, int y, int width, int height, bool bChecked,
                                  UINT nHotkey, bool bIsDefault, CDXUTCheckBox** ppCreated )
{
    HRESULT hr = S_OK;

    CDXUTCheckBox* pCheckBox = new CDXUTCheckBox( this );

    if( ppCreated != NULL )
        *ppCreated = pCheckBox;

    if( pCheckBox == NULL )
        return E_OUTOFMEMORY;

    hr = AddControl( pCheckBox );
    if( FAILED( hr ) )
        return hr;

    // Set the ID and list index
    pCheckBox->SetID( ID );
    pCheckBox->SetText( strText );
    pCheckBox->SetLocation( x, y );
    pCheckBox->SetSize( width, height );
    pCheckBox->SetHotkey( nHotkey );
    pCheckBox->m_bIsDefault = bIsDefault;
    pCheckBox->SetChecked( bChecked );

    return S_OK;
}



//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::AddRadioButton( int ID, UINT nButtonGroup, LPCWSTR strText, int x, int y, int width, int height,
                                     bool bChecked, UINT nHotkey, bool bIsDefault, CDXUTRadioButton** ppCreated )
{
    HRESULT hr = S_OK;

    CDXUTRadioButton* pRadioButton = new CDXUTRadioButton( this );

    if( ppCreated != NULL )
        *ppCreated = pRadioButton;

    if( pRadioButton == NULL )
        return E_OUTOFMEMORY;

    hr = AddControl( pRadioButton );
    if( FAILED( hr ) )
        return hr;

    // Set the ID and list index
    pRadioButton->SetID( ID );
    pRadioButton->SetText( strText );
    pRadioButton->SetButtonGroup( nButtonGroup );
    pRadioButton->SetLocation( x, y );
    pRadioButton->SetSize( width, height );
    pRadioButton->SetHotkey( nHotkey );
    pRadioButton->SetChecked( bChecked );
    pRadioButton->m_bIsDefault = bIsDefault;
    pRadioButton->SetChecked( bChecked );

    return S_OK;
}




//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::AddComboBox( int ID, int x, int y, int width, int height, UINT nHotkey, bool bIsDefault,
                                  CDXUTComboBox** ppCreated )
{
    HRESULT hr = S_OK;

    CDXUTComboBox* pComboBox = new CDXUTComboBox( this );

    if( ppCreated != NULL )
        *ppCreated = pComboBox;

    if( pComboBox == NULL )
        return E_OUTOFMEMORY;

    hr = AddControl( pComboBox );
    if( FAILED( hr ) )
        return hr;

    // Set the ID and list index
    pComboBox->SetID( ID );
    pComboBox->SetLocation( x, y );
    pComboBox->SetSize( width, height );
    pComboBox->SetHotkey( nHotkey );
    pComboBox->m_bIsDefault = bIsDefault;

    return S_OK;
}



//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::AddSlider( int ID, int x, int y, int width, int height, int min, int max, int value,
                                bool bIsDefault, CDXUTSlider** ppCreated )
{
    HRESULT hr = S_OK;

    CDXUTSlider* pSlider = new CDXUTSlider( this );

    if( ppCreated != NULL )
        *ppCreated = pSlider;

    if( pSlider == NULL )
        return E_OUTOFMEMORY;

    hr = AddControl( pSlider );
    if( FAILED( hr ) )
        return hr;

    // Set the ID and list index
    pSlider->SetID( ID );
    pSlider->SetLocation( x, y );
    pSlider->SetSize( width, height );
    pSlider->m_bIsDefault = bIsDefault;
    pSlider->SetRange( min, max );
    pSlider->SetValue( value );
    pSlider->UpdateRects();

    return S_OK;
}



//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::AddEditBox( int ID, LPCWSTR strText, int x, int y, int width, int height, bool bIsDefault,
                                 CDXUTEditBox** ppCreated )
{
    HRESULT hr = S_OK;

    CDXUTEditBox* pEditBox = new CDXUTEditBox( this );

    if( ppCreated != NULL )
        *ppCreated = pEditBox;

    if( pEditBox == NULL )
        return E_OUTOFMEMORY;

    hr = AddControl( pEditBox );
    if( FAILED( hr ) )
        return hr;

    // Set the ID and position
    pEditBox->SetID( ID );
    pEditBox->SetLocation( x, y );
    pEditBox->SetSize( width, height );
    pEditBox->m_bIsDefault = bIsDefault;

    if( strText )
        pEditBox->SetText( strText );

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::AddListBox( int ID, int x, int y, int width, int height, DWORD dwStyle, CDXUTListBox** ppCreated )
{
    HRESULT hr = S_OK;
    CDXUTListBox* pListBox = new CDXUTListBox( this );

    if( ppCreated != NULL )
        *ppCreated = pListBox;

    if( pListBox == NULL )
        return E_OUTOFMEMORY;

    hr = AddControl( pListBox );
    if( FAILED( hr ) )
        return hr;

    // Set the ID and position
    pListBox->SetID( ID );
    pListBox->SetLocation( x, y );
    pListBox->SetSize( width, height );
    pListBox->SetStyle( dwStyle );

    return S_OK;
}



//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::InitControl( CDXUTControl* pControl )
{
    HRESULT hr;

    if( pControl == NULL )
        return E_INVALIDARG;

    pControl->m_Index = m_Controls.GetSize();

    // Look for a default Element entries
    for( int i = 0; i < m_DefaultElements.GetSize(); i++ )
    {
        DXUTElementHolder* pElementHolder = m_DefaultElements.GetAt( i );
        if( pElementHolder->nControlType == pControl->GetType() )
            pControl->SetElement( pElementHolder->iElement, &pElementHolder->Element );
    }

    V_RETURN( pControl->OnInit() );

    return S_OK;
}



//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::AddControl( CDXUTControl* pControl )
{
    HRESULT hr = S_OK;

    hr = InitControl( pControl );
    if( FAILED( hr ) )
        return DXTRACE_ERR( L"CDXUTDialog::InitControl", hr );

    // Add to the list
    hr = m_Controls.Add( pControl );
    if( FAILED( hr ) )
    {
        return DXTRACE_ERR( L"CGrowableArray::Add", hr );
    }

    return S_OK;
}


//--------------------------------------------------------------------------------------
CDXUTControl* CDXUTDialog::GetControl( int ID )
{
    // Try to find the control with the given ID
    for( int i = 0; i < m_Controls.GetSize(); i++ )
    {
        CDXUTControl* pControl = m_Controls.GetAt( i );

        if( pControl->GetID() == ID )
        {
            return pControl;
        }
    }

    // Not found
    return NULL;
}



//--------------------------------------------------------------------------------------
CDXUTControl* CDXUTDialog::GetControl( int ID, UINT nControlType )
{
    // Try to find the control with the given ID
    for( int i = 0; i < m_Controls.GetSize(); i++ )
    {
        CDXUTControl* pControl = m_Controls.GetAt( i );

        if( pControl->GetID() == ID && pControl->GetType() == nControlType )
        {
            return pControl;
        }
    }

    // Not found
    return NULL;
}



//--------------------------------------------------------------------------------------
CDXUTControl* CDXUTDialog::GetNextControl( CDXUTControl* pControl )
{
    int index = pControl->m_Index + 1;

    CDXUTDialog* pDialog = pControl->m_pDialog;

    // Cycle through dialogs in the loop to find the next control. Note
    // that if only one control exists in all looped dialogs it will
    // be the returned 'next' control.
    while( index >= ( int )pDialog->m_Controls.GetSize() )
    {
        pDialog = pDialog->m_pNextDialog;
        index = 0;
    }

    return pDialog->m_Controls.GetAt( index );
}

//--------------------------------------------------------------------------------------
CDXUTControl* CDXUTDialog::GetPrevControl( CDXUTControl* pControl )
{
    int index = pControl->m_Index - 1;

    CDXUTDialog* pDialog = pControl->m_pDialog;

    // Cycle through dialogs in the loop to find the next control. Note
    // that if only one control exists in all looped dialogs it will
    // be the returned 'previous' control.
    while( index < 0 )
    {
        pDialog = pDialog->m_pPrevDialog;
        if( pDialog == NULL )
            pDialog = pControl->m_pDialog;

        index = pDialog->m_Controls.GetSize() - 1;
    }

    return pDialog->m_Controls.GetAt( index );
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::ClearRadioButtonGroup( UINT nButtonGroup )
{
    // Find all radio buttons with the given group number
    for( int i = 0; i < m_Controls.GetSize(); i++ )
    {
        CDXUTControl* pControl = m_Controls.GetAt( i );

        if( pControl->GetType() == DXUT_CONTROL_RADIOBUTTON )
        {
            CDXUTRadioButton* pRadioButton = ( CDXUTRadioButton* )pControl;

            if( pRadioButton->GetButtonGroup() == nButtonGroup )
                pRadioButton->SetChecked( false, false );
        }
    }
}



//--------------------------------------------------------------------------------------
void CDXUTDialog::ClearComboBox( int ID )
{
    CDXUTComboBox* pComboBox = GetComboBox( ID );
    if( pComboBox == NULL )
        return;

    pComboBox->RemoveAllItems();
}




//--------------------------------------------------------------------------------------
void CDXUTDialog::RequestFocus( CDXUTControl* pControl )
{
    if( s_pControlFocus == pControl )
        return;

    if( !pControl->CanHaveFocus() )
        return;

    if( s_pControlFocus )
        s_pControlFocus->OnFocusOut();

    pControl->OnFocusIn();
    s_pControlFocus = pControl;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::DrawRect( RECT* pRect, D3DCOLOR color )
{
    if( m_pManager->GetD3D9Device() )
        return DrawRect9( pRect, color );
    return E_FAIL;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::DrawRect9( RECT* pRect, D3DCOLOR color )
{
    RECT rcScreen = *pRect;
    OffsetRect( &rcScreen, m_x, m_y );

    // If caption is enabled, offset the Y position by its height.
    if( m_bCaption )
        OffsetRect( &rcScreen, 0, m_nCaptionHeight );

    DXUT_SCREEN_VERTEX vertices[4] =
    {
        ( float )rcScreen.left - 0.5f,  ( float )rcScreen.top - 0.5f,    0.5f, 1.0f, color, 0, 0,
        ( float )rcScreen.right - 0.5f, ( float )rcScreen.top - 0.5f,    0.5f, 1.0f, color, 0, 0,
        ( float )rcScreen.right - 0.5f, ( float )rcScreen.bottom - 0.5f, 0.5f, 1.0f, color, 0, 0,
        ( float )rcScreen.left - 0.5f,  ( float )rcScreen.bottom - 0.5f, 0.5f, 1.0f, color, 0, 0,
    };

    IDirect3DDevice9* pd3dDevice = m_pManager->GetD3D9Device();

    // Since we're doing our own drawing here we need to flush the sprites
    m_pManager->m_pSprite->Flush();
    IDirect3DVertexDeclaration9* pDecl = NULL;
    pd3dDevice->GetVertexDeclaration( &pDecl );  // Preserve the sprite's current vertex decl
    pd3dDevice->SetFVF( DXUT_SCREEN_VERTEX::FVF );

    pd3dDevice->SetTextureStageState( 0, D3DTSS_COLOROP, D3DTOP_SELECTARG2 );
    pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG2 );

    pd3dDevice->DrawPrimitiveUP( D3DPT_TRIANGLEFAN, 2, vertices, sizeof( DXUT_SCREEN_VERTEX ) );

    pd3dDevice->SetTextureStageState( 0, D3DTSS_COLOROP, D3DTOP_MODULATE );
    pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAOP, D3DTOP_MODULATE );

    // Restore the vertex decl
    pd3dDevice->SetVertexDeclaration( pDecl );
    pDecl->Release();

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::DrawPolyLine( POINT* apPoints, UINT nNumPoints, D3DCOLOR color )
{
    DXUT_SCREEN_VERTEX* vertices = new DXUT_SCREEN_VERTEX[ nNumPoints ];
    if( vertices == NULL )
        return E_OUTOFMEMORY;

    DXUT_SCREEN_VERTEX* pVertex = vertices;
    POINT* pt = apPoints;
    for( UINT i = 0; i < nNumPoints; i++ )
    {
        pVertex->x = m_x + ( float )pt->x;
        pVertex->y = m_y + ( float )pt->y;
        pVertex->z = 0.5f;
        pVertex->h = 1.0f;
        pVertex->color = color;
        pVertex->tu = 0.0f;
        pVertex->tv = 0.0f;

        pVertex++;
        pt++;
    }

    IDirect3DDevice9* pd3dDevice = m_pManager->GetD3D9Device();

    // Since we're doing our own drawing here we need to flush the sprites
    m_pManager->m_pSprite->Flush();
    IDirect3DVertexDeclaration9* pDecl = NULL;
    pd3dDevice->GetVertexDeclaration( &pDecl );  // Preserve the sprite's current vertex decl
    pd3dDevice->SetFVF( DXUT_SCREEN_VERTEX::FVF );

    pd3dDevice->SetTextureStageState( 0, D3DTSS_COLOROP, D3DTOP_SELECTARG2 );
    pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG2 );

    pd3dDevice->DrawPrimitiveUP( D3DPT_LINESTRIP, nNumPoints - 1, vertices, sizeof( DXUT_SCREEN_VERTEX ) );

    pd3dDevice->SetTextureStageState( 0, D3DTSS_COLOROP, D3DTOP_MODULATE );
    pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAOP, D3DTOP_MODULATE );

    // Restore the vertex decl
    pd3dDevice->SetVertexDeclaration( pDecl );
    pDecl->Release();

    SAFE_DELETE_ARRAY( vertices );
    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::DrawSprite( CDXUTElement* pElement, RECT* prcDest, float fDepth )
{
    if( m_pManager->GetD3D9Device() )
        return DrawSprite9( pElement, prcDest );
    else
        return DrawSprite11( pElement, prcDest, fDepth );
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::DrawSprite9( CDXUTElement* pElement, RECT* prcDest )
{
    // No need to draw fully transparent layers
    if( pElement->TextureColor.Current.a == 0 )
        return S_OK;

    RECT rcTexture = pElement->rcTexture;

    RECT rcScreen = *prcDest;
    OffsetRect( &rcScreen, m_x, m_y );

    // If caption is enabled, offset the Y position by its height.
    if( m_bCaption )
        OffsetRect( &rcScreen, 0, m_nCaptionHeight );

    DXUTTextureNode* pTextureNode = GetTexture( pElement->iTexture );
    if( pTextureNode == NULL )
        return E_FAIL;

    float fScaleX = ( float )RectWidth( rcScreen ) / RectWidth( rcTexture );
    float fScaleY = ( float )RectHeight( rcScreen ) / RectHeight( rcTexture );

    D3DXMATRIXA16 matTransform;
    D3DXMatrixScaling( &matTransform, fScaleX, fScaleY, 1.0f );

    m_pManager->m_pSprite->SetTransform( &matTransform );

    D3DXVECTOR3 vPos( ( float )rcScreen.left, ( float )rcScreen.top, 0.0f );

    vPos.x /= fScaleX;
    vPos.y /= fScaleY;

    return m_pManager->m_pSprite->Draw( pTextureNode->pTexture9, &rcTexture, NULL, &vPos,
                                        pElement->TextureColor.Current );
}

//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::DrawSprite11( CDXUTElement* pElement, RECT* prcDest, float fDepth )
{
    // No need to draw fully transparent layers
    if( pElement->TextureColor.Current.a == 0 )
        return S_OK;

    RECT rcTexture = pElement->rcTexture;

    RECT rcScreen = *prcDest;
    OffsetRect( &rcScreen, m_x, m_y );

    // If caption is enabled, offset the Y position by its height.
    if( m_bCaption )
        OffsetRect( &rcScreen, 0, m_nCaptionHeight );

    DXUTTextureNode* pTextureNode = GetTexture( pElement->iTexture );
    if( pTextureNode == NULL )
        return E_FAIL;

    float fBBWidth = ( float )m_pManager->m_nBackBufferWidth;
    float fBBHeight = ( float )m_pManager->m_nBackBufferHeight;
    float fTexWidth = ( float )pTextureNode->dwWidth;
    float fTexHeight = ( float )pTextureNode->dwHeight;

    float fRectLeft = rcScreen.left / fBBWidth;
    float fRectTop = 1.0f - rcScreen.top / fBBHeight;
    float fRectRight = rcScreen.right / fBBWidth;
    float fRectBottom = 1.0f - rcScreen.bottom / fBBHeight;

    fRectLeft = fRectLeft * 2.0f - 1.0f;
    fRectTop = fRectTop * 2.0f - 1.0f;
    fRectRight = fRectRight * 2.0f - 1.0f;
    fRectBottom = fRectBottom * 2.0f - 1.0f;
    
    float fTexLeft = rcTexture.left / fTexWidth;
    float fTexTop = rcTexture.top / fTexHeight;
    float fTexRight = rcTexture.right / fTexWidth;
    float fTexBottom = rcTexture.bottom / fTexHeight;

    // Add 6 sprite vertices
    DXUTSpriteVertex SpriteVertex;

    // tri1
    SpriteVertex.vPos = D3DXVECTOR3( fRectLeft, fRectTop, fDepth );
    SpriteVertex.vTex = D3DXVECTOR2( fTexLeft, fTexTop );
    SpriteVertex.vColor = pElement->TextureColor.Current;
    m_pManager->m_SpriteVertices.Add( SpriteVertex );

    SpriteVertex.vPos = D3DXVECTOR3( fRectRight, fRectTop, fDepth );
    SpriteVertex.vTex = D3DXVECTOR2( fTexRight, fTexTop );
    SpriteVertex.vColor = pElement->TextureColor.Current;
    m_pManager->m_SpriteVertices.Add( SpriteVertex );

    SpriteVertex.vPos = D3DXVECTOR3( fRectLeft, fRectBottom, fDepth );
    SpriteVertex.vTex = D3DXVECTOR2( fTexLeft, fTexBottom );
    SpriteVertex.vColor = pElement->TextureColor.Current;
    m_pManager->m_SpriteVertices.Add( SpriteVertex );

    // tri2
    SpriteVertex.vPos = D3DXVECTOR3( fRectRight, fRectTop, fDepth );
    SpriteVertex.vTex = D3DXVECTOR2( fTexRight, fTexTop );
    SpriteVertex.vColor = pElement->TextureColor.Current;
    m_pManager->m_SpriteVertices.Add( SpriteVertex );

    SpriteVertex.vPos = D3DXVECTOR3( fRectRight, fRectBottom, fDepth );
    SpriteVertex.vTex = D3DXVECTOR2( fTexRight, fTexBottom );
    SpriteVertex.vColor = pElement->TextureColor.Current;
    m_pManager->m_SpriteVertices.Add( SpriteVertex );

    SpriteVertex.vPos = D3DXVECTOR3( fRectLeft, fRectBottom, fDepth );
    SpriteVertex.vTex = D3DXVECTOR2( fTexLeft, fTexBottom );
    SpriteVertex.vColor = pElement->TextureColor.Current;
    m_pManager->m_SpriteVertices.Add( SpriteVertex );

    // TODO:  Why are we drawing the sprite every time?  This is very inefficient, but the sprite workaround doesn't have support for sorting now, so we have to
    // draw a sprite every time to keep the order correct between sprites and text.
    m_pManager->EndSprites11( DXUTGetD3D11Device(), DXUTGetD3D11DeviceContext() );

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::CalcTextRect( LPCWSTR strText, CDXUTElement* pElement, RECT* prcDest, int nCount )
{
    HRESULT hr = S_OK;

    DXUTFontNode* pFontNode = GetFont( pElement->iFont );
    if( pFontNode == NULL )
        return E_FAIL;

    DWORD dwTextFormat = pElement->dwTextFormat | DT_CALCRECT;
    // Since we are only computing the rectangle, we don't need a sprite.
    if( pFontNode->pFont9 )
    {
        hr = pFontNode->pFont9->DrawText( NULL, strText, nCount, prcDest, dwTextFormat, pElement->FontColor.Current );
        if( FAILED( hr ) )
            return hr;
    }

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::DrawText( LPCWSTR strText, CDXUTElement* pElement, RECT* prcDest, bool bShadow, int nCount, bool bCenter   )
{
    if( m_pManager->GetD3D9Device() )
        return DrawText9( strText, pElement, prcDest, bShadow, nCount );
    else
        return DrawText11( m_pManager->GetD3D11Device(), m_pManager->GetD3D11DeviceContext(), strText, pElement, prcDest, bShadow, nCount, bCenter );
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::DrawText9( LPCWSTR strText, CDXUTElement* pElement, RECT* prcDest, bool bShadow, int nCount )
{
    HRESULT hr = S_OK;

    // No need to draw fully transparent layers
    if( pElement->FontColor.Current.a == 0 )
        return S_OK;

    RECT rcScreen = *prcDest;
    OffsetRect( &rcScreen, m_x, m_y );

    // If caption is enabled, offset the Y position by its height.
    if( m_bCaption )
        OffsetRect( &rcScreen, 0, m_nCaptionHeight );

    D3DXMATRIX matTransform;
    D3DXMatrixIdentity( &matTransform );
    m_pManager->m_pSprite->SetTransform( &matTransform );

    DXUTFontNode* pFontNode = GetFont( pElement->iFont );

    if( bShadow )
    {
        RECT rcShadow = rcScreen;
        OffsetRect( &rcShadow, 1, 1 );
        hr = pFontNode->pFont9->DrawText( m_pManager->m_pSprite, strText, nCount, &rcShadow, pElement->dwTextFormat,
                                          D3DCOLOR_ARGB( DWORD( pElement->FontColor.Current.a * 255 ), 0, 0, 0 ) );
        if( FAILED( hr ) )
            return hr;
    }

    hr = pFontNode->pFont9->DrawText( m_pManager->m_pSprite, strText, nCount, &rcScreen, pElement->dwTextFormat,
                                      pElement->FontColor.Current );
    if( FAILED( hr ) )
        return hr;

    return S_OK;
}

ID3D11Buffer* g_pFontBuffer11 = NULL;
UINT g_FontBufferBytes11 = 0;
CGrowableArray<DXUTSpriteVertex> g_FontVertices;
ID3D11ShaderResourceView* g_pFont11 = NULL;
ID3D11InputLayout* g_pInputLayout11 = NULL;
HRESULT InitFont11( ID3D11Device* pd3d11Device, ID3D11InputLayout* pInputLayout )
{
    HRESULT hr = S_OK;
    WCHAR str[MAX_PATH];
    V_RETURN( DXUTFindDXSDKMediaFileCch( str, MAX_PATH, L"UI\\Font.dds" ) );
    
    if (pd3d11Device->GetFeatureLevel() < D3D_FEATURE_LEVEL_10_0 ) {

        D3DX11_IMAGE_INFO dii;
        D3DX11GetImageInfoFromFile( str, NULL, &dii, NULL );

        D3DX11_IMAGE_LOAD_INFO dili;
        dili.BindFlags = D3DX11_DEFAULT;
        dili.CpuAccessFlags = D3DX11_DEFAULT;
        dili.Depth = D3DX11_DEFAULT;
        dili.Filter = D3DX11_DEFAULT;
        dili.FirstMipLevel = 0;
        dili.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        dili.Height = D3DX11_DEFAULT;
        dili.MipFilter = D3DX11_DEFAULT;
        dili.MipLevels = 1;
        dili.MiscFlags = D3DX11_DEFAULT;
        dili.pSrcInfo = &dii;
        dili.Usage = D3D11_USAGE_DEFAULT ;
        dili.Width = D3DX11_DEFAULT;

        V_RETURN( D3DX11CreateShaderResourceViewFromFile( pd3d11Device, str, &dili, NULL, &g_pFont11, &hr) );
    } else {
        V_RETURN( D3DX11CreateShaderResourceViewFromFile( pd3d11Device, str, NULL, NULL, &g_pFont11, &hr) );
    
    }


    g_pInputLayout11 = pInputLayout;
    return hr;
}

void EndFont11()
{
    SAFE_RELEASE( g_pFontBuffer11 );
    g_FontBufferBytes11 = 0;
    SAFE_RELEASE( g_pFont11 );
}

void BeginText11()
{
    g_FontVertices.Reset();
}

void DrawText11DXUT( ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3d11DeviceContext,
                 LPCWSTR strText, RECT rcScreen, D3DXCOLOR vFontColor,
                 float fBBWidth, float fBBHeight, bool bCenter )
{
    float fCharTexSizeX = 0.010526315f;
    //float fGlyphSizeX = 14.0f / fBBWidth;
    //float fGlyphSizeY = 32.0f / fBBHeight;
    float fGlyphSizeX = 15.0f / fBBWidth;
    float fGlyphSizeY = 42.0f / fBBHeight;


    float fRectLeft = rcScreen.left / fBBWidth;
    float fRectTop = 1.0f - rcScreen.top / fBBHeight;

    fRectLeft = fRectLeft * 2.0f - 1.0f;
    fRectTop = fRectTop * 2.0f - 1.0f;

    int NumChars = (int)wcslen( strText );
    if (bCenter) {
        float fRectRight = rcScreen.right / fBBWidth;
        fRectRight = fRectRight * 2.0f - 1.0f;
        float fRectBottom = 1.0f - rcScreen.bottom / fBBHeight;
        fRectBottom = fRectBottom * 2.0f - 1.0f;
        float fcenterx = ((fRectRight - fRectLeft) - (float)NumChars*fGlyphSizeX) *0.5f;
        float fcentery = ((fRectTop - fRectBottom) - (float)1*fGlyphSizeY) *0.5f;
        fRectLeft += fcenterx ;    
        fRectTop -= fcentery;
    }
    float fOriginalLeft = fRectLeft;
    float fTexTop = 0.0f;
    float fTexBottom = 1.0f;

    float fDepth = 0.5f;
    for( int i=0; i<NumChars; i++ )
    {
        if( strText[i] == '\n' )
        {
            fRectLeft = fOriginalLeft;
            fRectTop -= fGlyphSizeY;

            continue;
        }
        else if( strText[i] < 32 || strText[i] > 126 )
        {
            continue;
        }

        // Add 6 sprite vertices
        DXUTSpriteVertex SpriteVertex;
        float fRectRight = fRectLeft + fGlyphSizeX;
        float fRectBottom = fRectTop - fGlyphSizeY;
        float fTexLeft = ( strText[i] - 32 ) * fCharTexSizeX;
        float fTexRight = fTexLeft + fCharTexSizeX;

        // tri1
        SpriteVertex.vPos = D3DXVECTOR3( fRectLeft, fRectTop, fDepth );
        SpriteVertex.vTex = D3DXVECTOR2( fTexLeft, fTexTop );
        SpriteVertex.vColor = vFontColor;
        g_FontVertices.Add( SpriteVertex );

        SpriteVertex.vPos = D3DXVECTOR3( fRectRight, fRectTop, fDepth );
        SpriteVertex.vTex = D3DXVECTOR2( fTexRight, fTexTop );
        SpriteVertex.vColor = vFontColor;
        g_FontVertices.Add( SpriteVertex );

        SpriteVertex.vPos = D3DXVECTOR3( fRectLeft, fRectBottom, fDepth );
        SpriteVertex.vTex = D3DXVECTOR2( fTexLeft, fTexBottom );
        SpriteVertex.vColor = vFontColor;
        g_FontVertices.Add( SpriteVertex );

        // tri2
        SpriteVertex.vPos = D3DXVECTOR3( fRectRight, fRectTop, fDepth );
        SpriteVertex.vTex = D3DXVECTOR2( fTexRight, fTexTop );
        SpriteVertex.vColor = vFontColor;
        g_FontVertices.Add( SpriteVertex );

        SpriteVertex.vPos = D3DXVECTOR3( fRectRight, fRectBottom, fDepth );
        SpriteVertex.vTex = D3DXVECTOR2( fTexRight, fTexBottom );
        SpriteVertex.vColor = vFontColor;
        g_FontVertices.Add( SpriteVertex );

        SpriteVertex.vPos = D3DXVECTOR3( fRectLeft, fRectBottom, fDepth );
        SpriteVertex.vTex = D3DXVECTOR2( fTexLeft, fTexBottom );
        SpriteVertex.vColor = vFontColor;
        g_FontVertices.Add( SpriteVertex );

        fRectLeft += fGlyphSizeX;

    }

    // TODO: We have to end text after every line so that rendering order between sprites and fonts is preserved
    EndText11( pd3dDevice, pd3d11DeviceContext );
}

void EndText11( ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3d11DeviceContext )
{

    // ensure our buffer size can hold our sprites
    UINT FontDataBytes = g_FontVertices.GetSize() * sizeof( DXUTSpriteVertex );
    if( g_FontBufferBytes11 < FontDataBytes )
    {
        SAFE_RELEASE( g_pFontBuffer11 );
        g_FontBufferBytes11 = FontDataBytes;

        D3D11_BUFFER_DESC BufferDesc;
        BufferDesc.ByteWidth = g_FontBufferBytes11;
        BufferDesc.Usage = D3D11_USAGE_DYNAMIC;
        BufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        BufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        BufferDesc.MiscFlags = 0;

        pd3dDevice->CreateBuffer( &BufferDesc, NULL, &g_pFontBuffer11 );
    }

    // Copy the sprites over
    D3D11_BOX destRegion;
    destRegion.left = 0;
    destRegion.right = FontDataBytes;
    destRegion.top = 0;
    destRegion.bottom = 1;
    destRegion.front = 0;
    destRegion.back = 1;
    D3D11_MAPPED_SUBRESOURCE MappedResource;
    if ( S_OK == pd3d11DeviceContext->Map( g_pFontBuffer11, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource ) ) { 
        CopyMemory( MappedResource.pData, (void*)g_FontVertices.GetData(), FontDataBytes );
        pd3d11DeviceContext->Unmap(g_pFontBuffer11, 0);
    }

    ID3D11ShaderResourceView* pOldTexture = NULL;
    pd3d11DeviceContext->PSGetShaderResources( 0, 1, &pOldTexture );
    pd3d11DeviceContext->PSSetShaderResources( 0, 1, &g_pFont11 );

    // Draw
    UINT Stride = sizeof( DXUTSpriteVertex );
    UINT Offset = 0;
    pd3d11DeviceContext->IASetVertexBuffers( 0, 1, &g_pFontBuffer11, &Stride, &Offset );
    pd3d11DeviceContext->IASetInputLayout( g_pInputLayout11 );
    pd3d11DeviceContext->IASetPrimitiveTopology( D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST );
    pd3d11DeviceContext->Draw( g_FontVertices.GetSize(), 0 );

    pd3d11DeviceContext->PSSetShaderResources( 0, 1, &pOldTexture );
    SAFE_RELEASE( pOldTexture );

    g_FontVertices.Reset();
}

//--------------------------------------------------------------------------------------
HRESULT CDXUTDialog::DrawText11( ID3D11Device* pd3dDevice, ID3D11DeviceContext* pd3d11DeviceContext,
                                 LPCWSTR strText, CDXUTElement* pElement, RECT* prcDest, bool bShadow, int nCount, bool bCenter  )
{
    //HRESULT hr = S_OK;

    // No need to draw fully transparent layers
    if( pElement->FontColor.Current.a == 0 )
        return S_OK;

    RECT rcScreen = *prcDest;
    OffsetRect( &rcScreen, m_x, m_y);

    // If caption is enabled, offset the Y position by its height.
    if( m_bCaption )
        OffsetRect( &rcScreen, 0, m_nCaptionHeight );

    float fBBWidth = ( float )m_pManager->m_nBackBufferWidth;
    float fBBHeight = ( float )m_pManager->m_nBackBufferHeight;

    if( bShadow )
    {
        RECT rcShadow = rcScreen;
        OffsetRect( &rcShadow, 1, 1 );

        D3DXCOLOR vShadowColor( 0,0,0, 1.0f );
        DrawText11DXUT( pd3dDevice, pd3d11DeviceContext,
                 strText, rcShadow, vShadowColor,
                 fBBWidth, fBBHeight, bCenter );

    }

    D3DXCOLOR vFontColor( pElement->FontColor.Current.r, pElement->FontColor.Current.g, pElement->FontColor.Current.b, 1.0f );
    DrawText11DXUT( pd3dDevice, pd3d11DeviceContext,
             strText, rcScreen, vFontColor,
             fBBWidth, fBBHeight, bCenter );

    return S_OK;
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::SetBackgroundColors( D3DCOLOR colorTopLeft, D3DCOLOR colorTopRight, D3DCOLOR colorBottomLeft,
                                       D3DCOLOR colorBottomRight )
{
    m_colorTopLeft = colorTopLeft;
    m_colorTopRight = colorTopRight;
    m_colorBottomLeft = colorBottomLeft;
    m_colorBottomRight = colorBottomRight;
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::SetNextDialog( CDXUTDialog* pNextDialog )
{
    if( pNextDialog == NULL )
        pNextDialog = this;

    m_pNextDialog = pNextDialog;
    if( pNextDialog )
        m_pNextDialog->m_pPrevDialog = this;
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::ClearFocus()
{
    if( s_pControlFocus )
    {
        s_pControlFocus->OnFocusOut();
        s_pControlFocus = NULL;
    }

    ReleaseCapture();
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::FocusDefaultControl()
{
    // Check for default control in this dialog
    for( int i = 0; i < m_Controls.GetSize(); i++ )
    {
        CDXUTControl* pControl = m_Controls.GetAt( i );
        if( pControl->m_bIsDefault )
        {
            // Remove focus from the current control
            ClearFocus();

            // Give focus to the default control
            s_pControlFocus = pControl;
            s_pControlFocus->OnFocusIn();
            return;
        }
    }
}


//--------------------------------------------------------------------------------------
bool CDXUTDialog::OnCycleFocus( bool bForward )
{
    CDXUTControl* pControl = NULL;
    CDXUTDialog* pDialog = NULL; // pDialog and pLastDialog are used to track wrapping of
    CDXUTDialog* pLastDialog;    // focus from first control to last or vice versa.

    if( s_pControlFocus == NULL )
    {
        // If s_pControlFocus is NULL, we focus the first control of first dialog in
        // the case that bForward is true, and focus the last control of last dialog when
        // bForward is false.
        //
        if( bForward )
        {
            // Search for the first control from the start of the dialog
            // array.
            for( int d = 0; d < m_pManager->m_Dialogs.GetSize(); ++d )
            {
                pDialog = pLastDialog = m_pManager->m_Dialogs.GetAt( d );
                if( pDialog && pDialog->m_Controls.GetSize() > 0 )
                {
                    pControl = pDialog->m_Controls.GetAt( 0 );
                    break;
                }
            }

            if( !pDialog || !pControl )
            {
                // No dialog has been registered yet or no controls have been
                // added to the dialogs. Cannot proceed.
                return true;
            }
        }
        else
        {
            // Search for the first control from the end of the dialog
            // array.
            for( int d = m_pManager->m_Dialogs.GetSize() - 1; d >= 0; --d )
            {
                pDialog = pLastDialog = m_pManager->m_Dialogs.GetAt( d );
                if( pDialog && pDialog->m_Controls.GetSize() > 0 )
                {
                    pControl = pDialog->m_Controls.GetAt( pDialog->m_Controls.GetSize() - 1 );
                    break;
                }
            }

            if( !pDialog || !pControl )
            {
                // No dialog has been registered yet or no controls have been
                // added to the dialogs. Cannot proceed.
                return true;
            }
        }
    }
    else if( s_pControlFocus->m_pDialog != this )
    {
        // If a control belonging to another dialog has focus, let that other
        // dialog handle this event by returning false.
        //
        return false;
    }
    else
    {
        // Focused control belongs to this dialog. Cycle to the
        // next/previous control.
        pLastDialog = s_pControlFocus->m_pDialog;
        pControl = ( bForward ) ? GetNextControl( s_pControlFocus ) : GetPrevControl( s_pControlFocus );
        pDialog = pControl->m_pDialog;
    }

    for( int i = 0; i < 0xffff; i++ )
    {
        // If we just wrapped from last control to first or vice versa,
        // set the focused control to NULL. This state, where no control
        // has focus, allows the camera to work.
        int nLastDialogIndex = m_pManager->m_Dialogs.IndexOf( pLastDialog );
        int nDialogIndex = m_pManager->m_Dialogs.IndexOf( pDialog );
        if( ( !bForward && nLastDialogIndex < nDialogIndex ) ||
            ( bForward && nDialogIndex < nLastDialogIndex ) )
        {
            if( s_pControlFocus )
                s_pControlFocus->OnFocusOut();
            s_pControlFocus = NULL;
            return true;
        }

        // If we've gone in a full circle then focus doesn't change
        if( pControl == s_pControlFocus )
            return true;

        // If the dialog accepts keybord input and the control can have focus then
        // move focus
        if( pControl->m_pDialog->m_bKeyboardInput && pControl->CanHaveFocus() )
        {
            if( s_pControlFocus )
                s_pControlFocus->OnFocusOut();
            s_pControlFocus = pControl;
            s_pControlFocus->OnFocusIn();
            return true;
        }

        pLastDialog = pDialog;
        pControl = ( bForward ) ? GetNextControl( pControl ) : GetPrevControl( pControl );
        pDialog = pControl->m_pDialog;
    }

    // If we reached this point, the chain of dialogs didn't form a complete loop
    DXTRACE_ERR( L"CDXUTDialog: Multiple dialogs are improperly chained together", E_FAIL );
    return false;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialogResourceManager::CreateFont9( UINT iFont )
{
    HRESULT hr = S_OK;

    DXUTFontNode* pFontNode = m_FontCache.GetAt( iFont );

    SAFE_RELEASE( pFontNode->pFont9 );

    V_RETURN( D3DXCreateFont( m_pd3d9Device, pFontNode->nHeight, 0, pFontNode->nWeight, 1, FALSE, DEFAULT_CHARSET,
                              OUT_DEFAULT_PRECIS, DEFAULT_QUALITY, DEFAULT_PITCH | FF_DONTCARE,
                              pFontNode->strFace, &pFontNode->pFont9 ) );

    return S_OK;
}



//--------------------------------------------------------------------------------------
HRESULT CDXUTDialogResourceManager::CreateFont11( UINT iFont )
{
    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialogResourceManager::CreateTexture9( UINT iTexture )
{
    HRESULT hr = S_OK;

    DXUTTextureNode* pTextureNode = m_TextureCache.GetAt( iTexture );


    D3DXIMAGE_INFO info;

    if( !pTextureNode->bFileSource )
    {
        if( pTextureNode->nResourceID == 0xFFFF && pTextureNode->hResourceModule == ( HMODULE )0xFFFF )
        {
            hr = DXUTCreateGUITextureFromInternalArray9( m_pd3d9Device, &pTextureNode->pTexture9, &info );
            if( FAILED( hr ) )
                return DXTRACE_ERR( L"D3DXCreateTextureFromFileInMemoryEx", hr );
        }
        else
        {
            LPCWSTR pID = pTextureNode->nResourceID ? ( LPCWSTR )( size_t )pTextureNode->nResourceID :
                pTextureNode->strFilename;

            // Create texture from resource
            hr = D3DXCreateTextureFromResourceEx( m_pd3d9Device, pTextureNode->hResourceModule, pID, D3DX_DEFAULT,
                                                  D3DX_DEFAULT,
                                                  1, 0, D3DFMT_UNKNOWN, D3DPOOL_MANAGED,
                                                  D3DX_DEFAULT, D3DX_DEFAULT, 0,
                                                  &info, NULL, &pTextureNode->pTexture9 );
            if( FAILED( hr ) )
                return DXTRACE_ERR( L"D3DXCreateTextureFromResourceEx", hr );
        }
    }
    else
    {
        // Make sure there's a texture to create
        if( pTextureNode->strFilename[0] == 0 )
            return S_OK;

        // Create texture from file
        hr = D3DXCreateTextureFromFileEx( m_pd3d9Device, pTextureNode->strFilename, D3DX_DEFAULT, D3DX_DEFAULT,
                                          1, 0, D3DFMT_UNKNOWN, D3DPOOL_MANAGED,
                                          D3DX_DEFAULT, D3DX_DEFAULT, 0,
                                          &info, NULL, &pTextureNode->pTexture9 );
        if( FAILED( hr ) )
        {
            return DXTRACE_ERR( L"D3DXCreateTextureFromFileEx", hr );
        }
    }

    // Store dimensions
    pTextureNode->dwWidth = info.Width;
    pTextureNode->dwHeight = info.Height;

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTDialogResourceManager::CreateTexture11( UINT iTexture )
{
    HRESULT hr = S_OK;

    DXUTTextureNode* pTextureNode = m_TextureCache.GetAt( iTexture );

    if( !pTextureNode->bFileSource )
    {
        if( pTextureNode->nResourceID == 0xFFFF && pTextureNode->hResourceModule == ( HMODULE )0xFFFF )
        {
            hr = DXUTCreateGUITextureFromInternalArray11( m_pd3d11Device, &pTextureNode->pTexture11, NULL );
            if( FAILED( hr ) )
                return DXTRACE_ERR( L"D3DX10CreateResourceFromFileInMemory", hr );
        }
        //else
        //{
        //    LPCWSTR pID = pTextureNode->nResourceID ? ( LPCWSTR )( size_t )pTextureNode->nResourceID :
        //        pTextureNode->strFilename;

        //    D3DX10_IMAGE_INFO SrcInfo;
        //    D3DX10GetImageInfoFromResource( NULL, pID, NULL, &SrcInfo, NULL );

        //    // Create texture from resource
        //    ID3D10Resource* pRes;
        //    D3DX10_IMAGE_LOAD_INFO loadInfo;
        //    loadInfo.Width = D3DX10_DEFAULT;
        //    loadInfo.Height = D3DX10_DEFAULT;
        //    loadInfo.Depth = D3DX10_DEFAULT;
        //    loadInfo.FirstMipLevel = 0;
        //    loadInfo.MipLevels = 1;
        //    loadInfo.Usage = D3D10_USAGE_DEFAULT;
        //    loadInfo.BindFlags = D3D10_BIND_SHADER_RESOURCE;
        //    loadInfo.CpuAccessFlags = 0;
        //    loadInfo.MiscFlags = 0;
        //    loadInfo.Format = MAKE_TYPELESS( SrcInfo.Format );
        //    loadInfo.Filter = D3DX10_FILTER_NONE;
        //    loadInfo.MipFilter = D3DX10_FILTER_NONE;
        //    loadInfo.pSrcInfo = &SrcInfo;

        //    hr = D3DX10CreateTextureFromResource( m_pd3d10Device, pTextureNode->hResourceModule, pID, &loadInfo,
        //                                          NULL, &pRes, NULL );
        //    if( FAILED( hr ) )
        //        return DXTRACE_ERR( L"D3DX10CreateResourceFromResource", hr );
        //    hr = pRes->QueryInterface( __uuidof( ID3D10Texture2D ), ( LPVOID* )&pTextureNode->pTexture10 );
        //    SAFE_RELEASE( pRes );
        //    if( FAILED( hr ) )
        //        return hr;
        //}
    }
    else
    {
        //
        //// Make sure there's a texture to create
        //if( pTextureNode->strFilename[0] == 0 )
        //    return S_OK;

        //D3DX10_IMAGE_INFO SrcInfo;
        //D3DX10GetImageInfoFromFile( pTextureNode->strFilename, NULL, &SrcInfo, NULL );

        //// Create texture from file
        //ID3D10Resource* pRes;
        //D3DX10_IMAGE_LOAD_INFO loadInfo;
        //loadInfo.Width = D3DX10_DEFAULT;
        //loadInfo.Height = D3DX10_DEFAULT;
        //loadInfo.Depth = D3DX10_DEFAULT;
        //loadInfo.FirstMipLevel = 0;
        //loadInfo.MipLevels = 1;
        //loadInfo.Usage = D3D10_USAGE_DEFAULT;
        //loadInfo.BindFlags = D3D10_BIND_SHADER_RESOURCE;
        //loadInfo.CpuAccessFlags = 0;
        //loadInfo.MiscFlags = 0;
        //loadInfo.Format = MAKE_TYPELESS( SrcInfo.Format );
        //loadInfo.Filter = D3DX10_FILTER_NONE;
        //loadInfo.MipFilter = D3DX10_FILTER_NONE;
        //loadInfo.pSrcInfo = &SrcInfo;
        //hr = D3DX10CreateTextureFromFile( m_pd3d10Device, pTextureNode->strFilename, &loadInfo, NULL, &pRes, NULL );
        //if( FAILED( hr ) )
        //{
        //    return DXTRACE_ERR( L"D3DX10CreateResourceFromFileEx", hr );
        //}
        //hr = pRes->QueryInterface( __uuidof( ID3D10Texture2D ), ( LPVOID* )&pTextureNode->pTexture10 );
        //SAFE_RELEASE( pRes );
        //if( FAILED( hr ) )
        //    return hr;
        //
    }

    // Store dimensions
    D3D11_TEXTURE2D_DESC desc;
    pTextureNode->pTexture11->GetDesc( &desc );
    pTextureNode->dwWidth = desc.Width;
    pTextureNode->dwHeight = desc.Height;

    // Create resource view
    D3D11_SHADER_RESOURCE_VIEW_DESC SRVDesc;
    SRVDesc.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
    SRVDesc.Format = MAKE_SRGB( desc.Format );
    SRVDesc.Texture2D.MipLevels = 1;
    SRVDesc.Texture2D.MostDetailedMip = 0;
    hr = m_pd3d11Device->CreateShaderResourceView( pTextureNode->pTexture11, &SRVDesc, &pTextureNode->pTexResView11 );

    return hr;
}


//--------------------------------------------------------------------------------------
void CDXUTDialog::InitDefaultElements()
{
    SetFont( 0, L"Arial", 14, FW_NORMAL );

    CDXUTElement Element;
    RECT rcTexture;

    //-------------------------------------
    // Element for the caption
    //-------------------------------------
    m_CapElement.SetFont( 0 );
    SetRect( &rcTexture, 17, 269, 241, 287 );
    m_CapElement.SetTexture( 0, &rcTexture );
    m_CapElement.TextureColor.States[ DXUT_STATE_NORMAL ] = D3DCOLOR_ARGB( 255, 255, 255, 255 );
    m_CapElement.FontColor.States[ DXUT_STATE_NORMAL ] = D3DCOLOR_ARGB( 255, 255, 255, 255 );
    m_CapElement.SetFont( 0, D3DCOLOR_ARGB( 255, 255, 255, 255 ), DT_LEFT | DT_VCENTER );
    // Pre-blend as we don't need to transition the state
    m_CapElement.TextureColor.Blend( DXUT_STATE_NORMAL, 10.0f );
    m_CapElement.FontColor.Blend( DXUT_STATE_NORMAL, 10.0f );

    //-------------------------------------
    // CDXUTStatic
    //-------------------------------------
    Element.SetFont( 0 );
    Element.FontColor.States[ DXUT_STATE_DISABLED ] = D3DCOLOR_ARGB( 200, 200, 200, 200 );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_STATIC, 0, &Element );


    //-------------------------------------
    // CDXUTButton - Button
    //-------------------------------------
    SetRect( &rcTexture, 0, 0, 136, 54 );
    Element.SetTexture( 0, &rcTexture );
    Element.SetFont( 0 );
    Element.TextureColor.States[ DXUT_STATE_NORMAL ] = D3DCOLOR_ARGB( 150, 255, 255, 255 );
    Element.TextureColor.States[ DXUT_STATE_PRESSED ] = D3DCOLOR_ARGB( 200, 255, 255, 255 );
    Element.FontColor.States[ DXUT_STATE_MOUSEOVER ] = D3DCOLOR_ARGB( 255, 0, 0, 0 );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_BUTTON, 0, &Element );


    //-------------------------------------
    // CDXUTButton - Fill layer
    //-------------------------------------
    SetRect( &rcTexture, 136, 0, 252, 54 );
    Element.SetTexture( 0, &rcTexture, D3DCOLOR_ARGB( 0, 255, 255, 255 ) );
    Element.TextureColor.States[ DXUT_STATE_MOUSEOVER ] = D3DCOLOR_ARGB( 160, 255, 255, 255 );
    Element.TextureColor.States[ DXUT_STATE_PRESSED ] = D3DCOLOR_ARGB( 60, 0, 0, 0 );
    Element.TextureColor.States[ DXUT_STATE_FOCUS ] = D3DCOLOR_ARGB( 30, 255, 255, 255 );


    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_BUTTON, 1, &Element );


    //-------------------------------------
    // CDXUTCheckBox - Box
    //-------------------------------------
    SetRect( &rcTexture, 0, 54, 27, 81 );
    Element.SetTexture( 0, &rcTexture );
    Element.SetFont( 0, D3DCOLOR_ARGB( 255, 255, 255, 255 ), DT_LEFT | DT_VCENTER );
    Element.FontColor.States[ DXUT_STATE_DISABLED ] = D3DCOLOR_ARGB( 200, 200, 200, 200 );
    Element.TextureColor.States[ DXUT_STATE_NORMAL ] = D3DCOLOR_ARGB( 150, 255, 255, 255 );
    Element.TextureColor.States[ DXUT_STATE_FOCUS ] = D3DCOLOR_ARGB( 200, 255, 255, 255 );
    Element.TextureColor.States[ DXUT_STATE_PRESSED ] = D3DCOLOR_ARGB( 255, 255, 255, 255 );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_CHECKBOX, 0, &Element );


    //-------------------------------------
    // CDXUTCheckBox - Check
    //-------------------------------------
    SetRect( &rcTexture, 27, 54, 54, 81 );
    Element.SetTexture( 0, &rcTexture );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_CHECKBOX, 1, &Element );


    //-------------------------------------
    // CDXUTRadioButton - Box
    //-------------------------------------
    SetRect( &rcTexture, 54, 54, 81, 81 );
    Element.SetTexture( 0, &rcTexture );
    Element.SetFont( 0, D3DCOLOR_ARGB( 255, 255, 255, 255 ), DT_LEFT | DT_VCENTER );
    Element.FontColor.States[ DXUT_STATE_DISABLED ] = D3DCOLOR_ARGB( 200, 200, 200, 200 );
    Element.TextureColor.States[ DXUT_STATE_NORMAL ] = D3DCOLOR_ARGB( 150, 255, 255, 255 );
    Element.TextureColor.States[ DXUT_STATE_FOCUS ] = D3DCOLOR_ARGB( 200, 255, 255, 255 );
    Element.TextureColor.States[ DXUT_STATE_PRESSED ] = D3DCOLOR_ARGB( 255, 255, 255, 255 );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_RADIOBUTTON, 0, &Element );


    //-------------------------------------
    // CDXUTRadioButton - Check
    //-------------------------------------
    SetRect( &rcTexture, 81, 54, 108, 81 );
    Element.SetTexture( 0, &rcTexture );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_RADIOBUTTON, 1, &Element );


    //-------------------------------------
    // CDXUTComboBox - Main
    //-------------------------------------
    SetRect( &rcTexture, 7, 81, 247, 123 );
    Element.SetTexture( 0, &rcTexture );
    Element.SetFont( 0 );
    Element.TextureColor.States[ DXUT_STATE_NORMAL ] = D3DCOLOR_ARGB( 150, 200, 200, 200 );
    Element.TextureColor.States[ DXUT_STATE_FOCUS ] = D3DCOLOR_ARGB( 170, 230, 230, 230 );
    Element.TextureColor.States[ DXUT_STATE_DISABLED ] = D3DCOLOR_ARGB( 70, 200, 200, 200 );
    Element.FontColor.States[ DXUT_STATE_MOUSEOVER ] = D3DCOLOR_ARGB( 255, 0, 0, 0 );
    Element.FontColor.States[ DXUT_STATE_PRESSED ] = D3DCOLOR_ARGB( 255, 0, 0, 0 );
    Element.FontColor.States[ DXUT_STATE_DISABLED ] = D3DCOLOR_ARGB( 200, 200, 200, 200 );


    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_COMBOBOX, 0, &Element );


    //-------------------------------------
    // CDXUTComboBox - Button
    //-------------------------------------
    SetRect( &rcTexture, 98, 189, 151, 238 );
    Element.SetTexture( 0, &rcTexture );
    Element.TextureColor.States[ DXUT_STATE_NORMAL ] = D3DCOLOR_ARGB( 150, 255, 255, 255 );
    Element.TextureColor.States[ DXUT_STATE_PRESSED ] = D3DCOLOR_ARGB( 255, 150, 150, 150 );
    Element.TextureColor.States[ DXUT_STATE_FOCUS ] = D3DCOLOR_ARGB( 200, 255, 255, 255 );
    Element.TextureColor.States[ DXUT_STATE_DISABLED ] = D3DCOLOR_ARGB( 70, 255, 255, 255 );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_COMBOBOX, 1, &Element );


    //-------------------------------------
    // CDXUTComboBox - Dropdown
    //-------------------------------------
    SetRect( &rcTexture, 13, 123, 241, 160 );
    Element.SetTexture( 0, &rcTexture );
    Element.SetFont( 0, D3DCOLOR_ARGB( 255, 0, 0, 0 ), DT_LEFT | DT_TOP );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_COMBOBOX, 2, &Element );


    //-------------------------------------
    // CDXUTComboBox - Selection
    //-------------------------------------
    SetRect( &rcTexture, 12, 163, 239, 183 );
    Element.SetTexture( 0, &rcTexture );
    Element.SetFont( 0, D3DCOLOR_ARGB( 255, 255, 255, 255 ), DT_LEFT | DT_TOP );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_COMBOBOX, 3, &Element );


    //-------------------------------------
    // CDXUTSlider - Track
    //-------------------------------------
    SetRect( &rcTexture, 1, 187, 93, 228 );
    Element.SetTexture( 0, &rcTexture );
    Element.TextureColor.States[ DXUT_STATE_NORMAL ] = D3DCOLOR_ARGB( 150, 255, 255, 255 );
    Element.TextureColor.States[ DXUT_STATE_FOCUS ] = D3DCOLOR_ARGB( 200, 255, 255, 255 );
    Element.TextureColor.States[ DXUT_STATE_DISABLED ] = D3DCOLOR_ARGB( 70, 255, 255, 255 );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_SLIDER, 0, &Element );

    //-------------------------------------
    // CDXUTSlider - Button
    //-------------------------------------
    SetRect( &rcTexture, 151, 193, 192, 234 );
    Element.SetTexture( 0, &rcTexture );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_SLIDER, 1, &Element );

    //-------------------------------------
    // CDXUTScrollBar - Track
    //-------------------------------------
    int nScrollBarStartX = 196;
    int nScrollBarStartY = 191;
    SetRect( &rcTexture, nScrollBarStartX + 0, nScrollBarStartY + 21, nScrollBarStartX + 22, nScrollBarStartY + 32 );
    Element.SetTexture( 0, &rcTexture );
    Element.TextureColor.States[ DXUT_STATE_DISABLED ] = D3DCOLOR_ARGB( 255, 200, 200, 200 );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_SCROLLBAR, 0, &Element );

    //-------------------------------------
    // CDXUTScrollBar - Up Arrow
    //-------------------------------------
    SetRect( &rcTexture, nScrollBarStartX + 0, nScrollBarStartY + 1, nScrollBarStartX + 22, nScrollBarStartY + 21 );
    Element.SetTexture( 0, &rcTexture );
    Element.TextureColor.States[ DXUT_STATE_DISABLED ] = D3DCOLOR_ARGB( 255, 200, 200, 200 );


    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_SCROLLBAR, 1, &Element );

    //-------------------------------------
    // CDXUTScrollBar - Down Arrow
    //-------------------------------------
    SetRect( &rcTexture, nScrollBarStartX + 0, nScrollBarStartY + 32, nScrollBarStartX + 22, nScrollBarStartY + 53 );
    Element.SetTexture( 0, &rcTexture );
    Element.TextureColor.States[ DXUT_STATE_DISABLED ] = D3DCOLOR_ARGB( 255, 200, 200, 200 );


    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_SCROLLBAR, 2, &Element );

    //-------------------------------------
    // CDXUTScrollBar - Button
    //-------------------------------------
    SetRect( &rcTexture, 220, 192, 238, 234 );
    Element.SetTexture( 0, &rcTexture );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_SCROLLBAR, 3, &Element );


    //-------------------------------------
    // CDXUTEditBox
    //-------------------------------------
    // Element assignment:
    //   0 - text area
    //   1 - top left border
    //   2 - top border
    //   3 - top right border
    //   4 - left border
    //   5 - right border
    //   6 - lower left border
    //   7 - lower border
    //   8 - lower right border

    Element.SetFont( 0, D3DCOLOR_ARGB( 255, 0, 0, 0 ), DT_LEFT | DT_TOP );

    // Assign the style
    SetRect( &rcTexture, 14, 90, 241, 113 );
    Element.SetTexture( 0, &rcTexture );
    SetDefaultElement( DXUT_CONTROL_EDITBOX, 0, &Element );
    SetRect( &rcTexture, 8, 82, 14, 90 );
    Element.SetTexture( 0, &rcTexture );
    SetDefaultElement( DXUT_CONTROL_EDITBOX, 1, &Element );
    SetRect( &rcTexture, 14, 82, 241, 90 );
    Element.SetTexture( 0, &rcTexture );
    SetDefaultElement( DXUT_CONTROL_EDITBOX, 2, &Element );
    SetRect( &rcTexture, 241, 82, 246, 90 );
    Element.SetTexture( 0, &rcTexture );
    SetDefaultElement( DXUT_CONTROL_EDITBOX, 3, &Element );
    SetRect( &rcTexture, 8, 90, 14, 113 );
    Element.SetTexture( 0, &rcTexture );
    SetDefaultElement( DXUT_CONTROL_EDITBOX, 4, &Element );
    SetRect( &rcTexture, 241, 90, 246, 113 );
    Element.SetTexture( 0, &rcTexture );
    SetDefaultElement( DXUT_CONTROL_EDITBOX, 5, &Element );
    SetRect( &rcTexture, 8, 113, 14, 121 );
    Element.SetTexture( 0, &rcTexture );
    SetDefaultElement( DXUT_CONTROL_EDITBOX, 6, &Element );
    SetRect( &rcTexture, 14, 113, 241, 121 );
    Element.SetTexture( 0, &rcTexture );
    SetDefaultElement( DXUT_CONTROL_EDITBOX, 7, &Element );
    SetRect( &rcTexture, 241, 113, 246, 121 );
    Element.SetTexture( 0, &rcTexture );
    SetDefaultElement( DXUT_CONTROL_EDITBOX, 8, &Element );

    //-------------------------------------
    // CDXUTListBox - Main
    //-------------------------------------
    SetRect( &rcTexture, 13, 123, 241, 160 );
    Element.SetTexture( 0, &rcTexture );
    Element.SetFont( 0, D3DCOLOR_ARGB( 255, 0, 0, 0 ), DT_LEFT | DT_TOP );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_LISTBOX, 0, &Element );

    //-------------------------------------
    // CDXUTListBox - Selection
    //-------------------------------------

    SetRect( &rcTexture, 16, 166, 240, 183 );
    Element.SetTexture( 0, &rcTexture );
    Element.SetFont( 0, D3DCOLOR_ARGB( 255, 255, 255, 255 ), DT_LEFT | DT_TOP );

    // Assign the Element
    SetDefaultElement( DXUT_CONTROL_LISTBOX, 1, &Element );
}



//--------------------------------------------------------------------------------------
// CDXUTControl class
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
CDXUTControl::CDXUTControl( CDXUTDialog* pDialog )
{
    m_Type = DXUT_CONTROL_BUTTON;
    m_pDialog = pDialog;
    m_ID = 0;
    m_Index = 0;
    m_pUserData = NULL;

    m_bEnabled = true;
    m_bVisible = true;
    m_bMouseOver = false;
    m_bHasFocus = false;
    m_bIsDefault = false;

    m_pDialog = NULL;

    m_x = 0;
    m_y = 0;
    m_width = 0;
    m_height = 0;

    ZeroMemory( &m_rcBoundingBox, sizeof( m_rcBoundingBox ) );
}


CDXUTControl::~CDXUTControl()
{
    for( int i = 0; i < m_Elements.GetSize(); ++i )
    {
        delete m_Elements[i];
    }
    m_Elements.RemoveAll();
}


//--------------------------------------------------------------------------------------
void CDXUTControl::SetTextColor( D3DCOLOR Color )
{
    CDXUTElement* pElement = m_Elements.GetAt( 0 );

    if( pElement )
        pElement->FontColor.States[DXUT_STATE_NORMAL] = Color;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTControl::SetElement( UINT iElement, CDXUTElement* pElement )
{
    HRESULT hr = S_OK;

    if( pElement == NULL )
        return E_INVALIDARG;

    // Make certain the array is this large
    for( UINT i = m_Elements.GetSize(); i <= iElement; i++ )
    {
        CDXUTElement* pNewElement = new CDXUTElement();
        if( pNewElement == NULL )
            return E_OUTOFMEMORY;

        hr = m_Elements.Add( pNewElement );
        if( FAILED( hr ) )
        {
            SAFE_DELETE( pNewElement );
            return hr;
        }
    }

    // Update the data
    CDXUTElement* pCurElement = m_Elements.GetAt( iElement );
    *pCurElement = *pElement;

    return S_OK;
}


//--------------------------------------------------------------------------------------
void CDXUTControl::Refresh()
{
    m_bMouseOver = false;
    m_bHasFocus = false;

    for( int i = 0; i < m_Elements.GetSize(); i++ )
    {
        CDXUTElement* pElement = m_Elements.GetAt( i );
        pElement->Refresh();
    }
}


//--------------------------------------------------------------------------------------
void CDXUTControl::UpdateRects()
{
    SetRect( &m_rcBoundingBox, m_x, m_y, m_x + m_width, m_y + m_height );
}


//--------------------------------------------------------------------------------------
// CDXUTStatic class
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
CDXUTStatic::CDXUTStatic( CDXUTDialog* pDialog )
{
    m_Type = DXUT_CONTROL_STATIC;
    m_pDialog = pDialog;

    ZeroMemory( &m_strText, sizeof( m_strText ) );

    for( int i = 0; i < m_Elements.GetSize(); i++ )
    {
        CDXUTElement* pElement = m_Elements.GetAt( i );
        SAFE_DELETE( pElement );
    }

    m_Elements.RemoveAll();
}


//--------------------------------------------------------------------------------------
void CDXUTStatic::Render( float fElapsedTime )
{
    if( m_bVisible == false )
        return;

    DXUT_CONTROL_STATE iState = DXUT_STATE_NORMAL;

    if( m_bEnabled == false )
        iState = DXUT_STATE_DISABLED;

    CDXUTElement* pElement = m_Elements.GetAt( 0 );

    pElement->FontColor.Blend( iState, fElapsedTime );

    m_pDialog->DrawText( m_strText, pElement, &m_rcBoundingBox, false, -1, false);
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTStatic::GetTextCopy( __out_ecount(bufferCount) LPWSTR strDest, 
                                  UINT bufferCount )
{
    // Validate incoming parameters
    if( strDest == NULL || bufferCount == 0 )
    {
        return E_INVALIDARG;
    }

    // Copy the window text
    wcscpy_s( strDest, bufferCount, m_strText );

    return S_OK;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTStatic::SetText( LPCWSTR strText )
{
    if( strText == NULL )
    {
        m_strText[0] = 0;
        return S_OK;
    }

    wcscpy_s( m_strText, MAX_PATH, strText );
    return S_OK;
}


//--------------------------------------------------------------------------------------
// CDXUTButton class
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
CDXUTButton::CDXUTButton( CDXUTDialog* pDialog )
{
    m_Type = DXUT_CONTROL_BUTTON;
    m_pDialog = pDialog;

    m_bPressed = false;
    m_nHotkey = 0;
}

//--------------------------------------------------------------------------------------
bool CDXUTButton::HandleKeyboard( UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    switch( uMsg )
    {
        case WM_KEYDOWN:
        {
            switch( wParam )
            {
                case VK_SPACE:
                    m_bPressed = true;
                    return true;
            }
        }

        case WM_KEYUP:
        {
            switch( wParam )
            {
                case VK_SPACE:
                    if( m_bPressed == true )
                    {
                        m_bPressed = false;
                        m_pDialog->SendEvent( EVENT_BUTTON_CLICKED, true, this );
                    }
                    return true;
            }
        }
    }
    return false;
}


//--------------------------------------------------------------------------------------
bool CDXUTButton::HandleMouse( UINT uMsg, POINT pt, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    switch( uMsg )
    {
        case WM_LBUTTONDOWN:
        case WM_LBUTTONDBLCLK:
            {
                if( ContainsPoint( pt ) )
                {
                    // Pressed while inside the control
                    m_bPressed = true;
                    SetCapture( DXUTGetHWND() );

                    if( !m_bHasFocus )
                        m_pDialog->RequestFocus( this );

                    return true;
                }

                break;
            }

        case WM_LBUTTONUP:
        {
            if( m_bPressed )
            {
                m_bPressed = false;
                ReleaseCapture();

                if( !m_pDialog->m_bKeyboardInput )
                    m_pDialog->ClearFocus();

                // Button click
                if( ContainsPoint( pt ) )
                    m_pDialog->SendEvent( EVENT_BUTTON_CLICKED, true, this );

                return true;
            }

            break;
        }
    };

    return false;
}

//--------------------------------------------------------------------------------------
void CDXUTButton::Render( float fElapsedTime )
{
    if( m_bVisible == false )
        return;

    int nOffsetX = 0;
    int nOffsetY = 0;

    DXUT_CONTROL_STATE iState = DXUT_STATE_NORMAL;

    if( m_bVisible == false )
    {
        iState = DXUT_STATE_HIDDEN;
    }
    else if( m_bEnabled == false )
    {
        iState = DXUT_STATE_DISABLED;
    }
    else if( m_bPressed )
    {
        iState = DXUT_STATE_PRESSED;

        nOffsetX = 1;
        nOffsetY = 2;
    }
    else if( m_bMouseOver )
    {
        iState = DXUT_STATE_MOUSEOVER;

        nOffsetX = -1;
        nOffsetY = -2;
    }
    else if( m_bHasFocus )
    {
        iState = DXUT_STATE_FOCUS;
    }

    // Background fill layer
    //TODO: remove magic numbers
    CDXUTElement* pElement = m_Elements.GetAt( 0 );

    float fBlendRate = ( iState == DXUT_STATE_PRESSED ) ? 0.0f : 0.8f;

    RECT rcWindow = m_rcBoundingBox;
    OffsetRect( &rcWindow, nOffsetX, nOffsetY );


    // Blend current color
    pElement->TextureColor.Blend( iState, fElapsedTime, fBlendRate );
    pElement->FontColor.Blend( iState, fElapsedTime, fBlendRate );

    m_pDialog->DrawSprite( pElement, &rcWindow, DXUT_FAR_BUTTON_DEPTH );
    m_pDialog->DrawText( m_strText, pElement, &rcWindow, false, -1, true );

    // Main button
    pElement = m_Elements.GetAt( 1 );

    // Blend current color
    pElement->TextureColor.Blend( iState, fElapsedTime, fBlendRate );
    pElement->FontColor.Blend( iState, fElapsedTime, fBlendRate );

    m_pDialog->DrawSprite( pElement, &rcWindow, DXUT_NEAR_BUTTON_DEPTH );
    m_pDialog->DrawText( m_strText, pElement, &rcWindow, false, -1, true );
}



//--------------------------------------------------------------------------------------
// CDXUTCheckBox class
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
CDXUTCheckBox::CDXUTCheckBox( CDXUTDialog* pDialog )
{
    m_Type = DXUT_CONTROL_CHECKBOX;
    m_pDialog = pDialog;

    m_bChecked = false;
}


//--------------------------------------------------------------------------------------
bool CDXUTCheckBox::HandleKeyboard( UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    switch( uMsg )
    {
        case WM_KEYDOWN:
        {
            switch( wParam )
            {
                case VK_SPACE:
                    m_bPressed = true;
                    return true;
            }
        }

        case WM_KEYUP:
        {
            switch( wParam )
            {
                case VK_SPACE:
                    if( m_bPressed == true )
                    {
                        m_bPressed = false;
                        SetCheckedInternal( !m_bChecked, true );
                    }
                    return true;
            }
        }
    }
    return false;
}


//--------------------------------------------------------------------------------------
bool CDXUTCheckBox::HandleMouse( UINT uMsg, POINT pt, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    switch( uMsg )
    {
        case WM_LBUTTONDOWN:
        case WM_LBUTTONDBLCLK:
            {
                if( ContainsPoint( pt ) )
                {
                    // Pressed while inside the control
                    m_bPressed = true;
                    SetCapture( DXUTGetHWND() );

                    if( !m_bHasFocus )
                        m_pDialog->RequestFocus( this );

                    return true;
                }

                break;
            }

        case WM_LBUTTONUP:
        {
            if( m_bPressed )
            {
                m_bPressed = false;
                ReleaseCapture();

                // Button click
                if( ContainsPoint( pt ) )
                    SetCheckedInternal( !m_bChecked, true );

                return true;
            }

            break;
        }
    };

    return false;
}


//--------------------------------------------------------------------------------------
void CDXUTCheckBox::SetCheckedInternal( bool bChecked, bool bFromInput )
{
    m_bChecked = bChecked;

    m_pDialog->SendEvent( EVENT_CHECKBOX_CHANGED, bFromInput, this );
}


//--------------------------------------------------------------------------------------
BOOL CDXUTCheckBox::ContainsPoint( POINT pt )
{
    return ( PtInRect( &m_rcBoundingBox, pt ) ||
             PtInRect( &m_rcButton, pt ) );
}



//--------------------------------------------------------------------------------------
void CDXUTCheckBox::UpdateRects()
{
    CDXUTButton::UpdateRects();

    m_rcButton = m_rcBoundingBox;
    m_rcButton.right = m_rcButton.left + RectHeight( m_rcButton );

    m_rcText = m_rcBoundingBox;
    m_rcText.left += ( int )( 1.25f * RectWidth( m_rcButton ) );
}



//--------------------------------------------------------------------------------------
void CDXUTCheckBox::Render( float fElapsedTime )
{
    if( m_bVisible == false )
        return;
    DXUT_CONTROL_STATE iState = DXUT_STATE_NORMAL;

    if( m_bVisible == false )
        iState = DXUT_STATE_HIDDEN;
    else if( m_bEnabled == false )
        iState = DXUT_STATE_DISABLED;
    else if( m_bPressed )
        iState = DXUT_STATE_PRESSED;
    else if( m_bMouseOver )
        iState = DXUT_STATE_MOUSEOVER;
    else if( m_bHasFocus )
        iState = DXUT_STATE_FOCUS;

    CDXUTElement* pElement = m_Elements.GetAt( 0 );

    float fBlendRate = ( iState == DXUT_STATE_PRESSED ) ? 0.0f : 0.8f;

    pElement->TextureColor.Blend( iState, fElapsedTime, fBlendRate );
    pElement->FontColor.Blend( iState, fElapsedTime, fBlendRate );

    m_pDialog->DrawSprite( pElement, &m_rcButton, DXUT_NEAR_BUTTON_DEPTH );
    m_pDialog->DrawText( m_strText, pElement, &m_rcText, false, -1, false );

    if( !m_bChecked )
        iState = DXUT_STATE_HIDDEN;

    pElement = m_Elements.GetAt( 1 );

    pElement->TextureColor.Blend( iState, fElapsedTime, fBlendRate );
    m_pDialog->DrawSprite( pElement, &m_rcButton, DXUT_FAR_BUTTON_DEPTH );
}




//--------------------------------------------------------------------------------------
// CDXUTRadioButton class
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
CDXUTRadioButton::CDXUTRadioButton( CDXUTDialog* pDialog )
{
    m_Type = DXUT_CONTROL_RADIOBUTTON;
    m_pDialog = pDialog;
}



//--------------------------------------------------------------------------------------
bool CDXUTRadioButton::HandleKeyboard( UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    switch( uMsg )
    {
        case WM_KEYDOWN:
        {
            switch( wParam )
            {
                case VK_SPACE:
                    m_bPressed = true;
                    return true;
            }
        }

        case WM_KEYUP:
        {
            switch( wParam )
            {
                case VK_SPACE:
                    if( m_bPressed == true )
                    {
                        m_bPressed = false;

                        m_pDialog->ClearRadioButtonGroup( m_nButtonGroup );
                        m_bChecked = !m_bChecked;

                        m_pDialog->SendEvent( EVENT_RADIOBUTTON_CHANGED, true, this );
                    }
                    return true;
            }
        }
    }
    return false;
}


//--------------------------------------------------------------------------------------
bool CDXUTRadioButton::HandleMouse( UINT uMsg, POINT pt, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    switch( uMsg )
    {
        case WM_LBUTTONDOWN:
        case WM_LBUTTONDBLCLK:
            {
                if( ContainsPoint( pt ) )
                {
                    // Pressed while inside the control
                    m_bPressed = true;
                    SetCapture( DXUTGetHWND() );

                    if( !m_bHasFocus )
                        m_pDialog->RequestFocus( this );

                    return true;
                }

                break;
            }

        case WM_LBUTTONUP:
        {
            if( m_bPressed )
            {
                m_bPressed = false;
                ReleaseCapture();

                // Button click
                if( ContainsPoint( pt ) )
                {
                    m_pDialog->ClearRadioButtonGroup( m_nButtonGroup );
                    m_bChecked = !m_bChecked;

                    m_pDialog->SendEvent( EVENT_RADIOBUTTON_CHANGED, true, this );
                }

                return true;
            }

            break;
        }
    };

    return false;
}

//--------------------------------------------------------------------------------------
void CDXUTRadioButton::SetCheckedInternal( bool bChecked, bool bClearGroup, bool bFromInput )
{
    if( bChecked && bClearGroup )
        m_pDialog->ClearRadioButtonGroup( m_nButtonGroup );

    m_bChecked = bChecked;
    m_pDialog->SendEvent( EVENT_RADIOBUTTON_CHANGED, bFromInput, this );
}




//--------------------------------------------------------------------------------------
// CDXUTComboBox class
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
CDXUTComboBox::CDXUTComboBox( CDXUTDialog* pDialog ) : m_ScrollBar( pDialog )
{
    m_Type = DXUT_CONTROL_COMBOBOX;
    m_pDialog = pDialog;

    m_nDropHeight = 100;

    m_nSBWidth = 16;
    m_bOpened = false;
    m_iSelected = -1;
    m_iFocused = -1;
}


//--------------------------------------------------------------------------------------
CDXUTComboBox::~CDXUTComboBox()
{
    RemoveAllItems();
}


//--------------------------------------------------------------------------------------
void CDXUTComboBox::SetTextColor( D3DCOLOR Color )
{
    CDXUTElement* pElement = m_Elements.GetAt( 0 );

    if( pElement )
        pElement->FontColor.States[DXUT_STATE_NORMAL] = Color;

    pElement = m_Elements.GetAt( 2 );

    if( pElement )
        pElement->FontColor.States[DXUT_STATE_NORMAL] = Color;
}


//--------------------------------------------------------------------------------------
void CDXUTComboBox::UpdateRects()
{

    CDXUTButton::UpdateRects();

    m_rcButton = m_rcBoundingBox;
    m_rcButton.left = m_rcButton.right - RectHeight( m_rcButton );

    m_rcText = m_rcBoundingBox;
    m_rcText.right = m_rcButton.left;

    m_rcDropdown = m_rcText;
    OffsetRect( &m_rcDropdown, 0, ( int )( 0.90f * RectHeight( m_rcText ) ) );
    m_rcDropdown.bottom += m_nDropHeight;
    m_rcDropdown.right -= m_nSBWidth;

    m_rcDropdownText = m_rcDropdown;
    m_rcDropdownText.left += ( int )( 0.1f * RectWidth( m_rcDropdown ) );
    m_rcDropdownText.right -= ( int )( 0.1f * RectWidth( m_rcDropdown ) );
    m_rcDropdownText.top += ( int )( 0.1f * RectHeight( m_rcDropdown ) );
    m_rcDropdownText.bottom -= ( int )( 0.1f * RectHeight( m_rcDropdown ) );

    // Update the scrollbar's rects
    m_ScrollBar.SetLocation( m_rcDropdown.right, m_rcDropdown.top + 2 );
    m_ScrollBar.SetSize( m_nSBWidth, RectHeight( m_rcDropdown ) - 2 );
    DXUTFontNode* pFontNode = m_pDialog->GetManager()->GetFontNode( m_Elements.GetAt( 2 )->iFont );
    if( pFontNode && pFontNode->nHeight )
    {
        m_ScrollBar.SetPageSize( RectHeight( m_rcDropdownText ) / pFontNode->nHeight );

        // The selected item may have been scrolled off the page.
        // Ensure that it is in page again.
        m_ScrollBar.ShowItem( m_iSelected );
    }
}


//--------------------------------------------------------------------------------------
void CDXUTComboBox::OnFocusOut()
{
    CDXUTButton::OnFocusOut();

    m_bOpened = false;
}


//--------------------------------------------------------------------------------------
bool CDXUTComboBox::HandleKeyboard( UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    const DWORD REPEAT_MASK = ( 0x40000000 );

    if( !m_bEnabled || !m_bVisible )
        return false;

    // Let the scroll bar have a chance to handle it first
    if( m_ScrollBar.HandleKeyboard( uMsg, wParam, lParam ) )
        return true;

    switch( uMsg )
    {
        case WM_KEYDOWN:
        {
            switch( wParam )
            {
                case VK_RETURN:
                    if( m_bOpened )
                    {
                        if( m_iSelected != m_iFocused )
                        {
                            m_iSelected = m_iFocused;
                            m_pDialog->SendEvent( EVENT_COMBOBOX_SELECTION_CHANGED, true, this );
                        }
                        m_bOpened = false;

                        if( !m_pDialog->m_bKeyboardInput )
                            m_pDialog->ClearFocus();

                        return true;
                    }
                    break;

                case VK_F4:
                    // Filter out auto-repeats
                    if( lParam & REPEAT_MASK )
                        return true;

                    m_bOpened = !m_bOpened;

                    if( !m_bOpened )
                    {
                        m_pDialog->SendEvent( EVENT_COMBOBOX_SELECTION_CHANGED, true, this );

                        if( !m_pDialog->m_bKeyboardInput )
                            m_pDialog->ClearFocus();
                    }

                    return true;

                case VK_LEFT:
                case VK_UP:
                    if( m_iFocused > 0 )
                    {
                        m_iFocused--;
                        m_iSelected = m_iFocused;

                        if( !m_bOpened )
                            m_pDialog->SendEvent( EVENT_COMBOBOX_SELECTION_CHANGED, true, this );
                    }

                    return true;

                case VK_RIGHT:
                case VK_DOWN:
                    if( m_iFocused + 1 < ( int )GetNumItems() )
                    {
                        m_iFocused++;
                        m_iSelected = m_iFocused;

                        if( !m_bOpened )
                            m_pDialog->SendEvent( EVENT_COMBOBOX_SELECTION_CHANGED, true, this );
                    }

                    return true;
            }
            break;
        }
    }

    return false;
}


//--------------------------------------------------------------------------------------
bool CDXUTComboBox::HandleMouse( UINT uMsg, POINT pt, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    // Let the scroll bar handle it first.
    if( m_ScrollBar.HandleMouse( uMsg, pt, wParam, lParam ) )
        return true;

    switch( uMsg )
    {
        case WM_MOUSEMOVE:
        {
            if( m_bOpened && PtInRect( &m_rcDropdown, pt ) )
            {
                // Determine which item has been selected
                for( int i = 0; i < m_Items.GetSize(); i++ )
                {
                    DXUTComboBoxItem* pItem = m_Items.GetAt( i );
                    if( pItem->bVisible &&
                        PtInRect( &pItem->rcActive, pt ) )
                    {
                        m_iFocused = i;
                    }
                }
                return true;
            }
            break;
        }

        case WM_LBUTTONDOWN:
        case WM_LBUTTONDBLCLK:
            {
                if( ContainsPoint( pt ) )
                {
                    // Pressed while inside the control
                    m_bPressed = true;
                    SetCapture( DXUTGetHWND() );

                    if( !m_bHasFocus )
                        m_pDialog->RequestFocus( this );

                    // Toggle dropdown
                    if( m_bHasFocus )
                    {
                        m_bOpened = !m_bOpened;

                        if( !m_bOpened )
                        {
                            if( !m_pDialog->m_bKeyboardInput )
                                m_pDialog->ClearFocus();
                        }
                    }

                    return true;
                }

                // Perhaps this click is within the dropdown
                if( m_bOpened && PtInRect( &m_rcDropdown, pt ) )
                {
                    // Determine which item has been selected
                    for( int i = m_ScrollBar.GetTrackPos(); i < m_Items.GetSize(); i++ )
                    {
                        DXUTComboBoxItem* pItem = m_Items.GetAt( i );
                        if( pItem->bVisible &&
                            PtInRect( &pItem->rcActive, pt ) )
                        {
                            m_iFocused = m_iSelected = i;
                            m_pDialog->SendEvent( EVENT_COMBOBOX_SELECTION_CHANGED, true, this );
                            m_bOpened = false;

                            if( !m_pDialog->m_bKeyboardInput )
                                m_pDialog->ClearFocus();

                            break;
                        }
                    }

                    return true;
                }

                // Mouse click not on main control or in dropdown, fire an event if needed
                if( m_bOpened )
                {
                    m_iFocused = m_iSelected;

                    m_pDialog->SendEvent( EVENT_COMBOBOX_SELECTION_CHANGED, true, this );
                    m_bOpened = false;
                }

                // Make sure the control is no longer in a pressed state
                m_bPressed = false;

                // Release focus if appropriate
                if( !m_pDialog->m_bKeyboardInput )
                {
                    m_pDialog->ClearFocus();
                }

                break;
            }

        case WM_LBUTTONUP:
        {
            if( m_bPressed && ContainsPoint( pt ) )
            {
                // Button click
                m_bPressed = false;
                ReleaseCapture();
                return true;
            }

            break;
        }

        case WM_MOUSEWHEEL:
        {
            int zDelta = ( short )HIWORD( wParam ) / WHEEL_DELTA;
            if( m_bOpened )
            {
                UINT uLines;
                SystemParametersInfo( SPI_GETWHEELSCROLLLINES, 0, &uLines, 0 );
                m_ScrollBar.Scroll( -zDelta * uLines );
            }
            else
            {
                if( zDelta > 0 )
                {
                    if( m_iFocused > 0 )
                    {
                        m_iFocused--;
                        m_iSelected = m_iFocused;

                        if( !m_bOpened )
                            m_pDialog->SendEvent( EVENT_COMBOBOX_SELECTION_CHANGED, true, this );
                    }
                }
                else
                {
                    if( m_iFocused + 1 < ( int )GetNumItems() )
                    {
                        m_iFocused++;
                        m_iSelected = m_iFocused;

                        if( !m_bOpened )
                            m_pDialog->SendEvent( EVENT_COMBOBOX_SELECTION_CHANGED, true, this );
                    }
                }
            }
            return true;
        }
    };

    return false;
}


//--------------------------------------------------------------------------------------
void CDXUTComboBox::OnHotkey()
{
    if( m_bOpened )
        return;

    if( m_iSelected == -1 )
        return;

    if( m_pDialog->IsKeyboardInputEnabled() )
        m_pDialog->RequestFocus( this );

    m_iSelected++;

    if( m_iSelected >= ( int )m_Items.GetSize() )
        m_iSelected = 0;

    m_iFocused = m_iSelected;
    m_pDialog->SendEvent( EVENT_COMBOBOX_SELECTION_CHANGED, true, this );
}


//--------------------------------------------------------------------------------------
void CDXUTComboBox::Render( float fElapsedTime )
{
    if( m_bVisible == false )
        return;
    DXUT_CONTROL_STATE iState = DXUT_STATE_NORMAL;

    if( !m_bOpened )
        iState = DXUT_STATE_HIDDEN;

    // Dropdown box
    CDXUTElement* pElement = m_Elements.GetAt( 2 );

    // If we have not initialized the scroll bar page size,
    // do that now.
    static bool bSBInit;
    if( !bSBInit )
    {
        // Update the page size of the scroll bar
        if( m_pDialog->GetManager()->GetFontNode( pElement->iFont )->nHeight )
            m_ScrollBar.SetPageSize( RectHeight( m_rcDropdownText ) /
                                     m_pDialog->GetManager()->GetFontNode( pElement->iFont )->nHeight );
        else
            m_ScrollBar.SetPageSize( RectHeight( m_rcDropdownText ) );
        bSBInit = true;
    }

    // Scroll bar
    if( m_bOpened )
        m_ScrollBar.Render( fElapsedTime );

    // Blend current color
    pElement->TextureColor.Blend( iState, fElapsedTime );
    pElement->FontColor.Blend( iState, fElapsedTime );

    m_pDialog->DrawSprite( pElement, &m_rcDropdown, DXUT_NEAR_BUTTON_DEPTH );

    // Selection outline
    CDXUTElement* pSelectionElement = m_Elements.GetAt( 3 );
    pSelectionElement->TextureColor.Current = pElement->TextureColor.Current;
    pSelectionElement->FontColor.Current = pSelectionElement->FontColor.States[ DXUT_STATE_NORMAL ];

    DXUTFontNode* pFont = m_pDialog->GetFont( pElement->iFont );
    if( pFont )
    {
        int curY = m_rcDropdownText.top;
        int nRemainingHeight = RectHeight( m_rcDropdownText );
        //WCHAR strDropdown[4096] = {0};

        for( int i = m_ScrollBar.GetTrackPos(); i < m_Items.GetSize(); i++ )
        {
            DXUTComboBoxItem* pItem = m_Items.GetAt( i );

            // Make sure there's room left in the dropdown
            nRemainingHeight -= pFont->nHeight;
            if( nRemainingHeight < 0 )
            {
                pItem->bVisible = false;
                continue;
            }

            SetRect( &pItem->rcActive, m_rcDropdownText.left, curY, m_rcDropdownText.right, curY + pFont->nHeight );
            curY += pFont->nHeight;

            //debug
            //int blue = 50 * i;
            //m_pDialog->DrawRect( &pItem->rcActive, 0xFFFF0000 | blue );

            pItem->bVisible = true;

            if( m_bOpened )
            {
                if( ( int )i == m_iFocused )
                {
                    RECT rc;
                    SetRect( &rc, m_rcDropdown.left, pItem->rcActive.top - 2, m_rcDropdown.right,
                             pItem->rcActive.bottom + 2 );
                    m_pDialog->DrawSprite( pSelectionElement, &rc, DXUT_NEAR_BUTTON_DEPTH );
                    m_pDialog->DrawText( pItem->strText, pSelectionElement, &pItem->rcActive );
                }
                else
                {
                    m_pDialog->DrawText( pItem->strText, pElement, &pItem->rcActive );
                }
            }
        }
    }

    int nOffsetX = 0;
    int nOffsetY = 0;

    iState = DXUT_STATE_NORMAL;

    if( m_bVisible == false )
        iState = DXUT_STATE_HIDDEN;
    else if( m_bEnabled == false )
        iState = DXUT_STATE_DISABLED;
    else if( m_bPressed )
    {
        iState = DXUT_STATE_PRESSED;

        nOffsetX = 1;
        nOffsetY = 2;
    }
    else if( m_bMouseOver )
    {
        iState = DXUT_STATE_MOUSEOVER;

        nOffsetX = -1;
        nOffsetY = -2;
    }
    else if( m_bHasFocus )
        iState = DXUT_STATE_FOCUS;

    float fBlendRate = ( iState == DXUT_STATE_PRESSED ) ? 0.0f : 0.8f;

    // Button
    pElement = m_Elements.GetAt( 1 );

    // Blend current color
    pElement->TextureColor.Blend( iState, fElapsedTime, fBlendRate );

    RECT rcWindow = m_rcButton;
    OffsetRect( &rcWindow, nOffsetX, nOffsetY );
    m_pDialog->DrawSprite( pElement, &rcWindow, DXUT_FAR_BUTTON_DEPTH );

    if( m_bOpened )
        iState = DXUT_STATE_PRESSED;

    // Main text box
    //TODO: remove magic numbers
    pElement = m_Elements.GetAt( 0 );

    // Blend current color
    pElement->TextureColor.Blend( iState, fElapsedTime, fBlendRate );
    pElement->FontColor.Blend( iState, fElapsedTime, fBlendRate );

    m_pDialog->DrawSprite( pElement, &m_rcText, DXUT_NEAR_BUTTON_DEPTH );

    if( m_iSelected >= 0 && m_iSelected < ( int )m_Items.GetSize() )
    {
        DXUTComboBoxItem* pItem = m_Items.GetAt( m_iSelected );
        if( pItem != NULL )
        {
            m_pDialog->DrawText( pItem->strText, pElement, &m_rcText, false, -1, true );

        }
    }
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTComboBox::AddItem( const WCHAR* strText, void* pData )
{
    // Validate parameters
    if( strText == NULL )
    {
        return E_INVALIDARG;
    }

    // Create a new item and set the data
    DXUTComboBoxItem* pItem = new DXUTComboBoxItem;
    if( pItem == NULL )
    {
        return DXTRACE_ERR_MSGBOX( L"new", E_OUTOFMEMORY );
    }

    ZeroMemory( pItem, sizeof( DXUTComboBoxItem ) );
    wcscpy_s( pItem->strText, 256, strText );
    pItem->pData = pData;

    m_Items.Add( pItem );

    // Update the scroll bar with new range
    m_ScrollBar.SetTrackRange( 0, m_Items.GetSize() );

    // If this is the only item in the list, it's selected
    if( GetNumItems() == 1 )
    {
        m_iSelected = 0;
        m_iFocused = 0;
        m_pDialog->SendEvent( EVENT_COMBOBOX_SELECTION_CHANGED, false, this );
    }

    return S_OK;
}


//--------------------------------------------------------------------------------------
void CDXUTComboBox::RemoveItem( UINT index )
{
    DXUTComboBoxItem* pItem = m_Items.GetAt( index );
    SAFE_DELETE( pItem );
    m_Items.Remove( index );
    m_ScrollBar.SetTrackRange( 0, m_Items.GetSize() );
    if( m_iSelected >= m_Items.GetSize() )
        m_iSelected = m_Items.GetSize() - 1;
}


//--------------------------------------------------------------------------------------
void CDXUTComboBox::RemoveAllItems()
{
    for( int i = 0; i < m_Items.GetSize(); i++ )
    {
        DXUTComboBoxItem* pItem = m_Items.GetAt( i );
        SAFE_DELETE( pItem );
    }

    m_Items.RemoveAll();
    m_ScrollBar.SetTrackRange( 0, 1 );
    m_iFocused = m_iSelected = -1;
}



//--------------------------------------------------------------------------------------
bool CDXUTComboBox::ContainsItem( const WCHAR* strText, UINT iStart )
{
    return ( -1 != FindItem( strText, iStart ) );
}


//--------------------------------------------------------------------------------------
int CDXUTComboBox::FindItem( const WCHAR* strText, UINT iStart )
{
    if( strText == NULL )
        return -1;

    for( int i = iStart; i < m_Items.GetSize(); i++ )
    {
        DXUTComboBoxItem* pItem = m_Items.GetAt( i );

        if( 0 == wcscmp( pItem->strText, strText ) )
        {
            return i;
        }
    }

    return -1;
}


//--------------------------------------------------------------------------------------
void* CDXUTComboBox::GetSelectedData()
{
    if( m_iSelected < 0 )
        return NULL;

    DXUTComboBoxItem* pItem = m_Items.GetAt( m_iSelected );
    return pItem->pData;
}


//--------------------------------------------------------------------------------------
DXUTComboBoxItem* CDXUTComboBox::GetSelectedItem()
{
    if( m_iSelected < 0 )
        return NULL;

    return m_Items.GetAt( m_iSelected );
}


//--------------------------------------------------------------------------------------
void* CDXUTComboBox::GetItemData( const WCHAR* strText )
{
    int index = FindItem( strText );
    if( index == -1 )
    {
        return NULL;
    }

    DXUTComboBoxItem* pItem = m_Items.GetAt( index );
    if( pItem == NULL )
    {
        DXTRACE_ERR( L"CGrowableArray::GetAt", E_FAIL );
        return NULL;
    }

    return pItem->pData;
}


//--------------------------------------------------------------------------------------
void* CDXUTComboBox::GetItemData( int nIndex )
{
    if( nIndex < 0 || nIndex >= m_Items.GetSize() )
        return NULL;

    return m_Items.GetAt( nIndex )->pData;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTComboBox::SetSelectedByIndex( UINT index )
{
    if( index >= GetNumItems() )
        return E_INVALIDARG;

    m_iFocused = m_iSelected = index;
    m_pDialog->SendEvent( EVENT_COMBOBOX_SELECTION_CHANGED, false, this );

    return S_OK;
}



//--------------------------------------------------------------------------------------
HRESULT CDXUTComboBox::SetSelectedByText( const WCHAR* strText )
{
    if( strText == NULL )
        return E_INVALIDARG;

    int index = FindItem( strText );
    if( index == -1 )
        return E_FAIL;

    m_iFocused = m_iSelected = index;
    m_pDialog->SendEvent( EVENT_COMBOBOX_SELECTION_CHANGED, false, this );

    return S_OK;
}



//--------------------------------------------------------------------------------------
HRESULT CDXUTComboBox::SetSelectedByData( void* pData )
{
    for( int i = 0; i < m_Items.GetSize(); i++ )
    {
        DXUTComboBoxItem* pItem = m_Items.GetAt( i );

        if( pItem->pData == pData )
        {
            m_iFocused = m_iSelected = i;
            m_pDialog->SendEvent( EVENT_COMBOBOX_SELECTION_CHANGED, false, this );
            return S_OK;
        }
    }

    return E_FAIL;
}



//--------------------------------------------------------------------------------------
CDXUTSlider::CDXUTSlider( CDXUTDialog* pDialog )
{
    m_Type = DXUT_CONTROL_SLIDER;
    m_pDialog = pDialog;

    m_nMin = 0;
    m_nMax = 100;
    m_nValue = 50;

    m_bPressed = false;
}


//--------------------------------------------------------------------------------------
BOOL CDXUTSlider::ContainsPoint( POINT pt )
{
    return ( PtInRect( &m_rcBoundingBox, pt ) ||
             PtInRect( &m_rcButton, pt ) );
}


//--------------------------------------------------------------------------------------
void CDXUTSlider::UpdateRects()
{
    CDXUTControl::UpdateRects();

    m_rcButton = m_rcBoundingBox;
    m_rcButton.right = m_rcButton.left + RectHeight( m_rcButton );
    OffsetRect( &m_rcButton, -RectWidth( m_rcButton ) / 2, 0 );

    m_nButtonX = ( int )( ( m_nValue - m_nMin ) * ( float )RectWidth( m_rcBoundingBox ) / ( m_nMax - m_nMin ) );
    OffsetRect( &m_rcButton, m_nButtonX, 0 );
}

int CDXUTSlider::ValueFromPos( int x )
{
    float fValuePerPixel = ( float )( m_nMax - m_nMin ) / RectWidth( m_rcBoundingBox );
    return ( int )( 0.5f + m_nMin + fValuePerPixel * ( x - m_rcBoundingBox.left ) );
}

//--------------------------------------------------------------------------------------
bool CDXUTSlider::HandleKeyboard( UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    switch( uMsg )
    {
        case WM_KEYDOWN:
        {
            switch( wParam )
            {
                case VK_HOME:
                    SetValueInternal( m_nMin, true );
                    return true;

                case VK_END:
                    SetValueInternal( m_nMax, true );
                    return true;

                case VK_LEFT:
                case VK_DOWN:
                    SetValueInternal( m_nValue - 1, true );
                    return true;

                case VK_RIGHT:
                case VK_UP:
                    SetValueInternal( m_nValue + 1, true );
                    return true;

                case VK_NEXT:
                    SetValueInternal( m_nValue - ( 10 > ( m_nMax - m_nMin ) / 10 ? 10 : ( m_nMax - m_nMin ) / 10 ),
                                      true );
                    return true;

                case VK_PRIOR:
                    SetValueInternal( m_nValue + ( 10 > ( m_nMax - m_nMin ) / 10 ? 10 : ( m_nMax - m_nMin ) / 10 ),
                                      true );
                    return true;
            }
            break;
        }
    }


    return false;
}


//--------------------------------------------------------------------------------------
bool CDXUTSlider::HandleMouse( UINT uMsg, POINT pt, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    switch( uMsg )
    {
        case WM_LBUTTONDOWN:
        case WM_LBUTTONDBLCLK:
            {
                if( PtInRect( &m_rcButton, pt ) )
                {
                    // Pressed while inside the control
                    m_bPressed = true;
                    SetCapture( DXUTGetHWND() );

                    m_nDragX = pt.x;
                    //m_nDragY = pt.y;
                    m_nDragOffset = m_nButtonX - m_nDragX;

                    //m_nDragValue = m_nValue;

                    if( !m_bHasFocus )
                        m_pDialog->RequestFocus( this );

                    return true;
                }

                if( PtInRect( &m_rcBoundingBox, pt ) )
                {
                    m_nDragX = pt.x;
                    m_nDragOffset = 0;
                    m_bPressed = true;

                    if( !m_bHasFocus )
                        m_pDialog->RequestFocus( this );

                    if( pt.x > m_nButtonX + m_x )
                    {
                        SetValueInternal( m_nValue + 1, true );
                        return true;
                    }

                    if( pt.x < m_nButtonX + m_x )
                    {
                        SetValueInternal( m_nValue - 1, true );
                        return true;
                    }
                }

                break;
            }

        case WM_LBUTTONUP:
        {
            if( m_bPressed )
            {
                m_bPressed = false;
                ReleaseCapture();
                m_pDialog->SendEvent( EVENT_SLIDER_VALUE_CHANGED_UP, true, this );

                return true;
            }

            break;
        }

        case WM_MOUSEMOVE:
        {
            if( m_bPressed )
            {
                SetValueInternal( ValueFromPos( m_x + pt.x + m_nDragOffset ), true );
                return true;
            }

            break;
        }

        case WM_MOUSEWHEEL:
        {
            int nScrollAmount = int( ( short )HIWORD( wParam ) ) / WHEEL_DELTA;
            SetValueInternal( m_nValue - nScrollAmount, true );
            return true;
        }
    };

    return false;
}


//--------------------------------------------------------------------------------------
void CDXUTSlider::SetRange( int nMin, int nMax )
{
    m_nMin = nMin;
    m_nMax = nMax;

    SetValueInternal( m_nValue, false );
}


//--------------------------------------------------------------------------------------
void CDXUTSlider::SetValueInternal( int nValue, bool bFromInput )
{
    // Clamp to range
    nValue = __max( m_nMin, nValue );
    nValue = __min( m_nMax, nValue );

    if( nValue == m_nValue )
        return;

    m_nValue = nValue;
    UpdateRects();

    m_pDialog->SendEvent( EVENT_SLIDER_VALUE_CHANGED, bFromInput, this );
}


//--------------------------------------------------------------------------------------
void CDXUTSlider::Render( float fElapsedTime )
{
    if( m_bVisible == false )
        return;

    int nOffsetX = 0;
    int nOffsetY = 0;

    DXUT_CONTROL_STATE iState = DXUT_STATE_NORMAL;

    if( m_bVisible == false )
    {
        iState = DXUT_STATE_HIDDEN;
    }
    else if( m_bEnabled == false )
    {
        iState = DXUT_STATE_DISABLED;
    }
    else if( m_bPressed )
    {
        iState = DXUT_STATE_PRESSED;

        nOffsetX = 1;
        nOffsetY = 2;
    }
    else if( m_bMouseOver )
    {
        iState = DXUT_STATE_MOUSEOVER;

        nOffsetX = -1;
        nOffsetY = -2;
    }
    else if( m_bHasFocus )
    {
        iState = DXUT_STATE_FOCUS;
    }

    float fBlendRate = ( iState == DXUT_STATE_PRESSED ) ? 0.0f : 0.8f;

    CDXUTElement* pElement = m_Elements.GetAt( 0 );

    // Blend current color
    pElement->TextureColor.Blend( iState, fElapsedTime, fBlendRate );
    m_pDialog->DrawSprite( pElement, &m_rcBoundingBox, DXUT_FAR_BUTTON_DEPTH );

    //TODO: remove magic numbers
    pElement = m_Elements.GetAt( 1 );

    // Blend current color
    pElement->TextureColor.Blend( iState, fElapsedTime, fBlendRate );
    m_pDialog->DrawSprite( pElement, &m_rcButton, DXUT_NEAR_BUTTON_DEPTH );
}


//--------------------------------------------------------------------------------------
// CDXUTScrollBar class
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
CDXUTScrollBar::CDXUTScrollBar( CDXUTDialog* pDialog )
{
    m_Type = DXUT_CONTROL_SCROLLBAR;
    m_pDialog = pDialog;

    m_bShowThumb = true;
    m_bDrag = false;

    SetRect( &m_rcUpButton, 0, 0, 0, 0 );
    SetRect( &m_rcDownButton, 0, 0, 0, 0 );
    SetRect( &m_rcTrack, 0, 0, 0, 0 );
    SetRect( &m_rcThumb, 0, 0, 0, 0 );
    m_nPosition = 0;
    m_nPageSize = 1;
    m_nStart = 0;
    m_nEnd = 1;
    m_Arrow = CLEAR;
    m_dArrowTS = 0.0;
}


//--------------------------------------------------------------------------------------
CDXUTScrollBar::~CDXUTScrollBar()
{
}


//--------------------------------------------------------------------------------------
void CDXUTScrollBar::UpdateRects()
{
    CDXUTControl::UpdateRects();

    // Make the buttons square

    SetRect( &m_rcUpButton, m_rcBoundingBox.left, m_rcBoundingBox.top,
             m_rcBoundingBox.right, m_rcBoundingBox.top + RectWidth( m_rcBoundingBox ) );
    SetRect( &m_rcDownButton, m_rcBoundingBox.left, m_rcBoundingBox.bottom - RectWidth( m_rcBoundingBox ),
             m_rcBoundingBox.right, m_rcBoundingBox.bottom );
    SetRect( &m_rcTrack, m_rcUpButton.left, m_rcUpButton.bottom,
             m_rcDownButton.right, m_rcDownButton.top );
    m_rcThumb.left = m_rcUpButton.left;
    m_rcThumb.right = m_rcUpButton.right;

    UpdateThumbRect();
}


//--------------------------------------------------------------------------------------
// Compute the dimension of the scroll thumb
void CDXUTScrollBar::UpdateThumbRect()
{
    if( m_nEnd - m_nStart > m_nPageSize )
    {
        int nThumbHeight = __max( RectHeight( m_rcTrack ) * m_nPageSize / ( m_nEnd - m_nStart ),
                                  SCROLLBAR_MINTHUMBSIZE );
        int nMaxPosition = m_nEnd - m_nStart - m_nPageSize;
        m_rcThumb.top = m_rcTrack.top + ( m_nPosition - m_nStart ) * ( RectHeight( m_rcTrack ) - nThumbHeight )
            / nMaxPosition;
        m_rcThumb.bottom = m_rcThumb.top + nThumbHeight;
        m_bShowThumb = true;

    }
    else
    {
        // No content to scroll
        m_rcThumb.bottom = m_rcThumb.top;
        m_bShowThumb = false;
    }
}


//--------------------------------------------------------------------------------------
// Scroll() scrolls by nDelta items.  A positive value scrolls down, while a negative
// value scrolls up.
void CDXUTScrollBar::Scroll( int nDelta )
{
    // Perform scroll
    m_nPosition += nDelta;

    // Cap position
    Cap();

    // Update thumb position
    UpdateThumbRect();
}


//--------------------------------------------------------------------------------------
void CDXUTScrollBar::ShowItem( int nIndex )
{
    // Cap the index

    if( nIndex < 0 )
        nIndex = 0;

    if( nIndex >= m_nEnd )
        nIndex = m_nEnd - 1;

    // Adjust position

    if( m_nPosition > nIndex )
        m_nPosition = nIndex;
    else if( m_nPosition + m_nPageSize <= nIndex )
        m_nPosition = nIndex - m_nPageSize + 1;

    UpdateThumbRect();
}


//--------------------------------------------------------------------------------------
bool CDXUTScrollBar::HandleKeyboard( UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    return false;
}


//--------------------------------------------------------------------------------------
bool CDXUTScrollBar::HandleMouse( UINT uMsg, POINT pt, WPARAM wParam, LPARAM lParam )
{
    static int ThumbOffsetY;

    m_LastMouse = pt;
    switch( uMsg )
    {
        case WM_LBUTTONDOWN:
        case WM_LBUTTONDBLCLK:
            {
                // Check for click on up button

                if( PtInRect( &m_rcUpButton, pt ) )
                {
                    SetCapture( DXUTGetHWND() );
                    if( m_nPosition > m_nStart )
                        --m_nPosition;
                    UpdateThumbRect();
                    m_Arrow = CLICKED_UP;
                    m_dArrowTS = DXUTGetTime();
                    return true;
                }

                // Check for click on down button

                if( PtInRect( &m_rcDownButton, pt ) )
                {
                    SetCapture( DXUTGetHWND() );
                    if( m_nPosition + m_nPageSize < m_nEnd )
                        ++m_nPosition;
                    UpdateThumbRect();
                    m_Arrow = CLICKED_DOWN;
                    m_dArrowTS = DXUTGetTime();
                    return true;
                }

                // Check for click on thumb

                if( PtInRect( &m_rcThumb, pt ) )
                {
                    SetCapture( DXUTGetHWND() );
                    m_bDrag = true;
                    ThumbOffsetY = pt.y - m_rcThumb.top;
                    return true;
                }

                // Check for click on track

                if( m_rcThumb.left <= pt.x &&
                    m_rcThumb.right > pt.x )
                {
                    SetCapture( DXUTGetHWND() );
                    if( m_rcThumb.top > pt.y &&
                        m_rcTrack.top <= pt.y )
                    {
                        Scroll( -( m_nPageSize - 1 ) );
                        return true;
                    }
                    else if( m_rcThumb.bottom <= pt.y &&
                             m_rcTrack.bottom > pt.y )
                    {
                        Scroll( m_nPageSize - 1 );
                        return true;
                    }
                }

                break;
            }

        case WM_LBUTTONUP:
        {
            m_bDrag = false;
            ReleaseCapture();
            UpdateThumbRect();
            m_Arrow = CLEAR;
            break;
        }

        case WM_MOUSEMOVE:
        {
            if( m_bDrag )
            {
                m_rcThumb.bottom += pt.y - ThumbOffsetY - m_rcThumb.top;
                m_rcThumb.top = pt.y - ThumbOffsetY;
                if( m_rcThumb.top < m_rcTrack.top )
                    OffsetRect( &m_rcThumb, 0, m_rcTrack.top - m_rcThumb.top );
                else if( m_rcThumb.bottom > m_rcTrack.bottom )
                    OffsetRect( &m_rcThumb, 0, m_rcTrack.bottom - m_rcThumb.bottom );

                // Compute first item index based on thumb position

                int nMaxFirstItem = m_nEnd - m_nStart - m_nPageSize;  // Largest possible index for first item
                int nMaxThumb = RectHeight( m_rcTrack ) - RectHeight( m_rcThumb );  // Largest possible thumb position from the top

                m_nPosition = m_nStart +
                    ( m_rcThumb.top - m_rcTrack.top +
                      nMaxThumb / ( nMaxFirstItem * 2 ) ) * // Shift by half a row to avoid last row covered by only one pixel
                    nMaxFirstItem / nMaxThumb;

                return true;
            }

            break;
        }
    }

    return false;
}


//--------------------------------------------------------------------------------------
bool CDXUTScrollBar::MsgProc( UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    if( WM_CAPTURECHANGED == uMsg )
    {
        // The application just lost mouse capture. We may not have gotten
        // the WM_MOUSEUP message, so reset m_bDrag here.
        if( ( HWND )lParam != DXUTGetHWND() )
            m_bDrag = false;
    }

    return false;
}


//--------------------------------------------------------------------------------------
void CDXUTScrollBar::Render( float fElapsedTime )
{
    if( m_bVisible == false )
        return;

    // Check if the arrow button has been held for a while.
    // If so, update the thumb position to simulate repeated
    // scroll.
    if( m_Arrow != CLEAR )
    {
        double dCurrTime = DXUTGetTime();
        if( PtInRect( &m_rcUpButton, m_LastMouse ) )
        {
            switch( m_Arrow )
            {
                case CLICKED_UP:
                    if( SCROLLBAR_ARROWCLICK_DELAY < dCurrTime - m_dArrowTS )
                    {
                        Scroll( -1 );
                        m_Arrow = HELD_UP;
                        m_dArrowTS = dCurrTime;
                    }
                    break;
                case HELD_UP:
                    if( SCROLLBAR_ARROWCLICK_REPEAT < dCurrTime - m_dArrowTS )
                    {
                        Scroll( -1 );
                        m_dArrowTS = dCurrTime;
                    }
                    break;
            }
        }
        else if( PtInRect( &m_rcDownButton, m_LastMouse ) )
        {
            switch( m_Arrow )
            {
                case CLICKED_DOWN:
                    if( SCROLLBAR_ARROWCLICK_DELAY < dCurrTime - m_dArrowTS )
                    {
                        Scroll( 1 );
                        m_Arrow = HELD_DOWN;
                        m_dArrowTS = dCurrTime;
                    }
                    break;
                case HELD_DOWN:
                    if( SCROLLBAR_ARROWCLICK_REPEAT < dCurrTime - m_dArrowTS )
                    {
                        Scroll( 1 );
                        m_dArrowTS = dCurrTime;
                    }
                    break;
            }
        }
    }

    DXUT_CONTROL_STATE iState = DXUT_STATE_NORMAL;

    if( m_bVisible == false )
        iState = DXUT_STATE_HIDDEN;
    else if( m_bEnabled == false || m_bShowThumb == false )
        iState = DXUT_STATE_DISABLED;
    else if( m_bMouseOver )
        iState = DXUT_STATE_MOUSEOVER;
    else if( m_bHasFocus )
        iState = DXUT_STATE_FOCUS;


    float fBlendRate = ( iState == DXUT_STATE_PRESSED ) ? 0.0f : 0.8f;

    // Background track layer
    CDXUTElement* pElement = m_Elements.GetAt( 0 );

    // Blend current color
    pElement->TextureColor.Blend( iState, fElapsedTime, fBlendRate );
    m_pDialog->DrawSprite( pElement, &m_rcTrack, DXUT_FAR_BUTTON_DEPTH );

    // Up Arrow
    pElement = m_Elements.GetAt( 1 );

    // Blend current color
    pElement->TextureColor.Blend( iState, fElapsedTime, fBlendRate );
    m_pDialog->DrawSprite( pElement, &m_rcUpButton, DXUT_NEAR_BUTTON_DEPTH );

    // Down Arrow
    pElement = m_Elements.GetAt( 2 );

    // Blend current color
    pElement->TextureColor.Blend( iState, fElapsedTime, fBlendRate );
    m_pDialog->DrawSprite( pElement, &m_rcDownButton, DXUT_NEAR_BUTTON_DEPTH );

    // Thumb button
    pElement = m_Elements.GetAt( 3 );

    // Blend current color
    pElement->TextureColor.Blend( iState, fElapsedTime, fBlendRate );
    m_pDialog->DrawSprite( pElement, &m_rcThumb, DXUT_NEAR_BUTTON_DEPTH );

}


//--------------------------------------------------------------------------------------
void CDXUTScrollBar::SetTrackRange( int nStart, int nEnd )
{
    m_nStart = nStart; m_nEnd = nEnd;
    Cap();
    UpdateThumbRect();
}


//--------------------------------------------------------------------------------------
void CDXUTScrollBar::Cap()  // Clips position at boundaries. Ensures it stays within legal range.
{
    if( m_nPosition < m_nStart ||
        m_nEnd - m_nStart <= m_nPageSize )
    {
        m_nPosition = m_nStart;
    }
    else if( m_nPosition + m_nPageSize > m_nEnd )
        m_nPosition = m_nEnd - m_nPageSize;
}

//--------------------------------------------------------------------------------------
// CDXUTListBox class
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
CDXUTListBox::CDXUTListBox( CDXUTDialog* pDialog ) : m_ScrollBar( pDialog )
{
    m_Type = DXUT_CONTROL_LISTBOX;
    m_pDialog = pDialog;

    m_dwStyle = 0;
    m_nSBWidth = 16;
    m_nSelected = -1;
    m_nSelStart = 0;
    m_bDrag = false;
    m_nBorder = 6;
    m_nMargin = 5;
    m_nTextHeight = 0;
}


//--------------------------------------------------------------------------------------
CDXUTListBox::~CDXUTListBox()
{
    RemoveAllItems();
}


//--------------------------------------------------------------------------------------
void CDXUTListBox::UpdateRects()
{
    CDXUTControl::UpdateRects();

    m_rcSelection = m_rcBoundingBox;
    m_rcSelection.right -= m_nSBWidth;
    InflateRect( &m_rcSelection, -m_nBorder, -m_nBorder );
    m_rcText = m_rcSelection;
    InflateRect( &m_rcText, -m_nMargin, 0 );

    // Update the scrollbar's rects
    m_ScrollBar.SetLocation( m_rcBoundingBox.right - m_nSBWidth, m_rcBoundingBox.top );
    m_ScrollBar.SetSize( m_nSBWidth, m_height );
    DXUTFontNode* pFontNode = m_pDialog->GetManager()->GetFontNode( m_Elements.GetAt( 0 )->iFont );
    if( pFontNode && pFontNode->nHeight )
    {
        m_ScrollBar.SetPageSize( RectHeight( m_rcText ) / pFontNode->nHeight );

        // The selected item may have been scrolled off the page.
        // Ensure that it is in page again.
        m_ScrollBar.ShowItem( m_nSelected );
    }
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTListBox::AddItem( const WCHAR* wszText, void* pData )
{
    DXUTListBoxItem* pNewItem = new DXUTListBoxItem;
    if( !pNewItem )
        return E_OUTOFMEMORY;

    wcscpy_s( pNewItem->strText, 256, wszText );
    pNewItem->pData = pData;
    SetRect( &pNewItem->rcActive, 0, 0, 0, 0 );
    pNewItem->bSelected = false;

    HRESULT hr = m_Items.Add( pNewItem );
    if( FAILED( hr ) )
    {
        SAFE_DELETE( pNewItem );
    }
    else
    {
        m_ScrollBar.SetTrackRange( 0, m_Items.GetSize() );
    }

    return hr;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTListBox::InsertItem( int nIndex, const WCHAR* wszText, void* pData )
{
    DXUTListBoxItem* pNewItem = new DXUTListBoxItem;
    if( !pNewItem )
        return E_OUTOFMEMORY;

    wcscpy_s( pNewItem->strText, 256, wszText );
    pNewItem->pData = pData;
    SetRect( &pNewItem->rcActive, 0, 0, 0, 0 );
    pNewItem->bSelected = false;

    HRESULT hr = m_Items.Insert( nIndex, pNewItem );
    if( SUCCEEDED( hr ) )
        m_ScrollBar.SetTrackRange( 0, m_Items.GetSize() );
    else
        SAFE_DELETE( pNewItem );

    return hr;
}


//--------------------------------------------------------------------------------------
void CDXUTListBox::RemoveItem( int nIndex )
{
    if( nIndex < 0 || nIndex >= ( int )m_Items.GetSize() )
        return;

    DXUTListBoxItem* pItem = m_Items.GetAt( nIndex );

    delete pItem;
    m_Items.Remove( nIndex );
    m_ScrollBar.SetTrackRange( 0, m_Items.GetSize() );
    if( m_nSelected >= ( int )m_Items.GetSize() )
        m_nSelected = m_Items.GetSize() - 1;

    m_pDialog->SendEvent( EVENT_LISTBOX_SELECTION, true, this );
}





//--------------------------------------------------------------------------------------
void CDXUTListBox::RemoveAllItems()
{
    for( int i = 0; i < m_Items.GetSize(); ++i )
    {
        DXUTListBoxItem* pItem = m_Items.GetAt( i );
        delete pItem;
    }

    m_Items.RemoveAll();
    m_ScrollBar.SetTrackRange( 0, 1 );
}


//--------------------------------------------------------------------------------------
DXUTListBoxItem* CDXUTListBox::GetItem( int nIndex )
{
    if( nIndex < 0 || nIndex >= ( int )m_Items.GetSize() )
        return NULL;

    return m_Items[nIndex];
}


//--------------------------------------------------------------------------------------
// For single-selection listbox, returns the index of the selected item.
// For multi-selection, returns the first selected item after the nPreviousSelected position.
// To search for the first selected item, the app passes -1 for nPreviousSelected.  For
// subsequent searches, the app passes the returned index back to GetSelectedIndex as.
// nPreviousSelected.
// Returns -1 on error or if no item is selected.
int CDXUTListBox::GetSelectedIndex( int nPreviousSelected )
{
    if( nPreviousSelected < -1 )
        return -1;

    if( m_dwStyle & MULTISELECTION )
    {
        // Multiple selection enabled. Search for the next item with the selected flag.
        for( int i = nPreviousSelected + 1; i < ( int )m_Items.GetSize(); ++i )
        {
            DXUTListBoxItem* pItem = m_Items.GetAt( i );

            if( pItem->bSelected )
                return i;
        }

        return -1;
    }
    else
    {
        // Single selection
        return m_nSelected;
    }
}


//--------------------------------------------------------------------------------------
void CDXUTListBox::SelectItem( int nNewIndex )
{
    // If no item exists, do nothing.
    if( m_Items.GetSize() == 0 )
        return;

    int nOldSelected = m_nSelected;

    // Adjust m_nSelected
    m_nSelected = nNewIndex;

    // Perform capping
    if( m_nSelected < 0 )
        m_nSelected = 0;
    if( m_nSelected >= ( int )m_Items.GetSize() )
        m_nSelected = m_Items.GetSize() - 1;

    if( nOldSelected != m_nSelected )
    {
        if( m_dwStyle & MULTISELECTION )
        {
            m_Items[m_nSelected]->bSelected = true;
        }

        // Update selection start
        m_nSelStart = m_nSelected;

        // Adjust scroll bar
        m_ScrollBar.ShowItem( m_nSelected );
    }

    m_pDialog->SendEvent( EVENT_LISTBOX_SELECTION, true, this );
}


//--------------------------------------------------------------------------------------
bool CDXUTListBox::HandleKeyboard( UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    // Let the scroll bar have a chance to handle it first
    if( m_ScrollBar.HandleKeyboard( uMsg, wParam, lParam ) )
        return true;

    switch( uMsg )
    {
        case WM_KEYDOWN:
            switch( wParam )
            {
                case VK_UP:
                case VK_DOWN:
                case VK_NEXT:
                case VK_PRIOR:
                case VK_HOME:
                case VK_END:
                    {
                        // If no item exists, do nothing.
                        if( m_Items.GetSize() == 0 )
                            return true;

                        int nOldSelected = m_nSelected;

                        // Adjust m_nSelected
                        switch( wParam )
                        {
                            case VK_UP:
                                --m_nSelected; break;
                            case VK_DOWN:
                                ++m_nSelected; break;
                            case VK_NEXT:
                                m_nSelected += m_ScrollBar.GetPageSize() - 1; break;
                            case VK_PRIOR:
                                m_nSelected -= m_ScrollBar.GetPageSize() - 1; break;
                            case VK_HOME:
                                m_nSelected = 0; break;
                            case VK_END:
                                m_nSelected = m_Items.GetSize() - 1; break;
                        }

                        // Perform capping
                        if( m_nSelected < 0 )
                            m_nSelected = 0;
                        if( m_nSelected >= ( int )m_Items.GetSize() )
                            m_nSelected = m_Items.GetSize() - 1;

                        if( nOldSelected != m_nSelected )
                        {
                            if( m_dwStyle & MULTISELECTION )
                            {
                                // Multiple selection

                                // Clear all selection
                                for( int i = 0; i < ( int )m_Items.GetSize(); ++i )
                                {
                                    DXUTListBoxItem* pItem = m_Items[i];
                                    pItem->bSelected = false;
                                }

                                if( GetKeyState( VK_SHIFT ) < 0 )
                                {
                                    // Select all items from m_nSelStart to
                                    // m_nSelected
                                    int nEnd = __max( m_nSelStart, m_nSelected );

                                    for( int n = __min( m_nSelStart, m_nSelected ); n <= nEnd; ++n )
                                        m_Items[n]->bSelected = true;
                                }
                                else
                                {
                                    m_Items[m_nSelected]->bSelected = true;

                                    // Update selection start
                                    m_nSelStart = m_nSelected;
                                }
                            }
                            else
                                m_nSelStart = m_nSelected;

                            // Adjust scroll bar

                            m_ScrollBar.ShowItem( m_nSelected );

                            // Send notification

                            m_pDialog->SendEvent( EVENT_LISTBOX_SELECTION, true, this );
                        }
                        return true;
                    }

                    // Space is the hotkey for double-clicking an item.
                    //
                case VK_SPACE:
                    m_pDialog->SendEvent( EVENT_LISTBOX_ITEM_DBLCLK, true, this );
                    return true;
            }
            break;
    }

    return false;
}


//--------------------------------------------------------------------------------------
bool CDXUTListBox::HandleMouse( UINT uMsg, POINT pt, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    // First acquire focus
    if( WM_LBUTTONDOWN == uMsg )
        if( !m_bHasFocus )
            m_pDialog->RequestFocus( this );

    // Let the scroll bar handle it first.
    if( m_ScrollBar.HandleMouse( uMsg, pt, wParam, lParam ) )
        return true;

    switch( uMsg )
    {
        case WM_LBUTTONDOWN:
        case WM_LBUTTONDBLCLK:
            // Check for clicks in the text area
            if( m_Items.GetSize() > 0 && PtInRect( &m_rcSelection, pt ) )
            {
                // Compute the index of the clicked item

                int nClicked;
                if( m_nTextHeight )
                    nClicked = m_ScrollBar.GetTrackPos() + ( pt.y - m_rcText.top ) / m_nTextHeight;
                else
                    nClicked = -1;

                // Only proceed if the click falls on top of an item.

                if( nClicked >= m_ScrollBar.GetTrackPos() &&
                    nClicked < ( int )m_Items.GetSize() &&
                    nClicked < m_ScrollBar.GetTrackPos() + m_ScrollBar.GetPageSize() )
                {
                    SetCapture( DXUTGetHWND() );
                    m_bDrag = true;

                    // If this is a double click, fire off an event and exit
                    // since the first click would have taken care of the selection
                    // updating.
                    if( uMsg == WM_LBUTTONDBLCLK )
                    {
                        m_pDialog->SendEvent( EVENT_LISTBOX_ITEM_DBLCLK, true, this );
                        return true;
                    }

                    m_nSelected = nClicked;
                    if( !( wParam & MK_SHIFT ) )
                        m_nSelStart = m_nSelected;

                    // If this is a multi-selection listbox, update per-item
                    // selection data.

                    if( m_dwStyle & MULTISELECTION )
                    {
                        // Determine behavior based on the state of Shift and Ctrl

                        DXUTListBoxItem* pSelItem = m_Items.GetAt( m_nSelected );
                        if( ( wParam & ( MK_SHIFT | MK_CONTROL ) ) == MK_CONTROL )
                        {
                            // Control click. Reverse the selection of this item.

                            pSelItem->bSelected = !pSelItem->bSelected;
                        }
                        else if( ( wParam & ( MK_SHIFT | MK_CONTROL ) ) == MK_SHIFT )
                        {
                            // Shift click. Set the selection for all items
                            // from last selected item to the current item.
                            // Clear everything else.

                            int nBegin = __min( m_nSelStart, m_nSelected );
                            int nEnd = __max( m_nSelStart, m_nSelected );

                            for( int i = 0; i < nBegin; ++i )
                            {
                                DXUTListBoxItem* pItem = m_Items.GetAt( i );
                                pItem->bSelected = false;
                            }

                            for( int i = nEnd + 1; i < ( int )m_Items.GetSize(); ++i )
                            {
                                DXUTListBoxItem* pItem = m_Items.GetAt( i );
                                pItem->bSelected = false;
                            }

                            for( int i = nBegin; i <= nEnd; ++i )
                            {
                                DXUTListBoxItem* pItem = m_Items.GetAt( i );
                                pItem->bSelected = true;
                            }
                        }
                        else if( ( wParam & ( MK_SHIFT | MK_CONTROL ) ) == ( MK_SHIFT | MK_CONTROL ) )
                        {
                            // Control-Shift-click.

                            // The behavior is:
                            //   Set all items from m_nSelStart to m_nSelected to
                            //     the same state as m_nSelStart, not including m_nSelected.
                            //   Set m_nSelected to selected.

                            int nBegin = __min( m_nSelStart, m_nSelected );
                            int nEnd = __max( m_nSelStart, m_nSelected );

                            // The two ends do not need to be set here.

                            bool bLastSelected = m_Items.GetAt( m_nSelStart )->bSelected;
                            for( int i = nBegin + 1; i < nEnd; ++i )
                            {
                                DXUTListBoxItem* pItem = m_Items.GetAt( i );
                                pItem->bSelected = bLastSelected;
                            }

                            pSelItem->bSelected = true;

                            // Restore m_nSelected to the previous value
                            // This matches the Windows behavior

                            m_nSelected = m_nSelStart;
                        }
                        else
                        {
                            // Simple click.  Clear all items and select the clicked
                            // item.


                            for( int i = 0; i < ( int )m_Items.GetSize(); ++i )
                            {
                                DXUTListBoxItem* pItem = m_Items.GetAt( i );
                                pItem->bSelected = false;
                            }

                            pSelItem->bSelected = true;
                        }
                    }  // End of multi-selection case

                    m_pDialog->SendEvent( EVENT_LISTBOX_SELECTION, true, this );
                }

                return true;
            }
            break;

        case WM_LBUTTONUP:
        {
            ReleaseCapture();
            m_bDrag = false;

            if( m_nSelected != -1 )
            {
                // Set all items between m_nSelStart and m_nSelected to
                // the same state as m_nSelStart
                int nEnd = __max( m_nSelStart, m_nSelected );

                for( int n = __min( m_nSelStart, m_nSelected ) + 1; n < nEnd; ++n )
                    m_Items[n]->bSelected = m_Items[m_nSelStart]->bSelected;
                m_Items[m_nSelected]->bSelected = m_Items[m_nSelStart]->bSelected;

                // If m_nSelStart and m_nSelected are not the same,
                // the user has dragged the mouse to make a selection.
                // Notify the application of this.
                if( m_nSelStart != m_nSelected )
                    m_pDialog->SendEvent( EVENT_LISTBOX_SELECTION, true, this );

                m_pDialog->SendEvent( EVENT_LISTBOX_SELECTION_END, true, this );
            }
            return false;
        }

        case WM_MOUSEMOVE:
            if( m_bDrag )
            {
                // Compute the index of the item below cursor

                int nItem;
                if( m_nTextHeight )
                    nItem = m_ScrollBar.GetTrackPos() + ( pt.y - m_rcText.top ) / m_nTextHeight;
                else
                    nItem = -1;

                // Only proceed if the cursor is on top of an item.

                if( nItem >= ( int )m_ScrollBar.GetTrackPos() &&
                    nItem < ( int )m_Items.GetSize() &&
                    nItem < m_ScrollBar.GetTrackPos() + m_ScrollBar.GetPageSize() )
                {
                    m_nSelected = nItem;
                    m_pDialog->SendEvent( EVENT_LISTBOX_SELECTION, true, this );
                }
                else if( nItem < ( int )m_ScrollBar.GetTrackPos() )
                {
                    // User drags the mouse above window top
                    m_ScrollBar.Scroll( -1 );
                    m_nSelected = m_ScrollBar.GetTrackPos();
                    m_pDialog->SendEvent( EVENT_LISTBOX_SELECTION, true, this );
                }
                else if( nItem >= m_ScrollBar.GetTrackPos() + m_ScrollBar.GetPageSize() )
                {
                    // User drags the mouse below window bottom
                    m_ScrollBar.Scroll( 1 );
                    m_nSelected = __min( ( int )m_Items.GetSize(), m_ScrollBar.GetTrackPos() +
                                         m_ScrollBar.GetPageSize() ) - 1;
                    m_pDialog->SendEvent( EVENT_LISTBOX_SELECTION, true, this );
                }
            }
            break;

        case WM_MOUSEWHEEL:
        {
            UINT uLines;
            SystemParametersInfo( SPI_GETWHEELSCROLLLINES, 0, &uLines, 0 );
            int nScrollAmount = int( ( short )HIWORD( wParam ) ) / WHEEL_DELTA * uLines;
            m_ScrollBar.Scroll( -nScrollAmount );
            return true;
        }
    }

    return false;
}


//--------------------------------------------------------------------------------------
bool CDXUTListBox::MsgProc( UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    if( WM_CAPTURECHANGED == uMsg )
    {
        // The application just lost mouse capture. We may not have gotten
        // the WM_MOUSEUP message, so reset m_bDrag here.
        if( ( HWND )lParam != DXUTGetHWND() )
            m_bDrag = false;
    }

    return false;
}


//--------------------------------------------------------------------------------------
void CDXUTListBox::Render( float fElapsedTime )
{
    if( m_bVisible == false )
        return;

    CDXUTElement* pElement = m_Elements.GetAt( 0 );
    pElement->TextureColor.Blend( DXUT_STATE_NORMAL, fElapsedTime );
    pElement->FontColor.Blend( DXUT_STATE_NORMAL, fElapsedTime );

    CDXUTElement* pSelElement = m_Elements.GetAt( 1 );
    pSelElement->TextureColor.Blend( DXUT_STATE_NORMAL, fElapsedTime );
    pSelElement->FontColor.Blend( DXUT_STATE_NORMAL, fElapsedTime );

    m_pDialog->DrawSprite( pElement, &m_rcBoundingBox, DXUT_FAR_BUTTON_DEPTH );

    // Render the text
    if( m_Items.GetSize() > 0 )
    {
        // Find out the height of a single line of text
        RECT rc = m_rcText;
        RECT rcSel = m_rcSelection;
        rc.bottom = rc.top + m_pDialog->GetManager()->GetFontNode( pElement->iFont )->nHeight;

        // Update the line height formation
        m_nTextHeight = rc.bottom - rc.top;

        static bool bSBInit;
        if( !bSBInit )
        {
            // Update the page size of the scroll bar
            if( m_nTextHeight )
                m_ScrollBar.SetPageSize( RectHeight( m_rcText ) / m_nTextHeight );
            else
                m_ScrollBar.SetPageSize( RectHeight( m_rcText ) );
            bSBInit = true;
        }

        rc.right = m_rcText.right;
        for( int i = m_ScrollBar.GetTrackPos(); i < ( int )m_Items.GetSize(); ++i )
        {
            if( rc.bottom > m_rcText.bottom )
                break;

            DXUTListBoxItem* pItem = m_Items.GetAt( i );

            // Determine if we need to render this item with the
            // selected element.
            bool bSelectedStyle = false;

            if( !( m_dwStyle & MULTISELECTION ) && i == m_nSelected )
                bSelectedStyle = true;
            else if( m_dwStyle & MULTISELECTION )
            {
                if( m_bDrag &&
                    ( ( i >= m_nSelected && i < m_nSelStart ) ||
                      ( i <= m_nSelected && i > m_nSelStart ) ) )
                    bSelectedStyle = m_Items[m_nSelStart]->bSelected;
                else if( pItem->bSelected )
                    bSelectedStyle = true;
            }

            if( bSelectedStyle )
            {
                rcSel.top = rc.top; rcSel.bottom = rc.bottom;
                m_pDialog->DrawSprite( pSelElement, &rcSel, DXUT_NEAR_BUTTON_DEPTH );
                m_pDialog->DrawText( pItem->strText, pSelElement, &rc );
            }
            else
                m_pDialog->DrawText( pItem->strText, pElement, &rc );

            OffsetRect( &rc, 0, m_nTextHeight );
        }
    }

    // Render the scroll bar

    m_ScrollBar.Render( fElapsedTime );
}


// Static member initialization
HINSTANCE               CUniBuffer::s_hDll = NULL;
HRESULT ( WINAPI*CUniBuffer::_ScriptApplyDigitSubstitution )( const SCRIPT_DIGITSUBSTITUTE*, SCRIPT_CONTROL*,
                                                              SCRIPT_STATE* ) = Dummy_ScriptApplyDigitSubstitution;
HRESULT ( WINAPI*CUniBuffer::_ScriptStringAnalyse )( HDC, const void*, int, int, int, DWORD, int, SCRIPT_CONTROL*,
                                                     SCRIPT_STATE*, const int*, SCRIPT_TABDEF*, const BYTE*,
                                                     SCRIPT_STRING_ANALYSIS* ) = Dummy_ScriptStringAnalyse;
HRESULT ( WINAPI*CUniBuffer::_ScriptStringCPtoX )( SCRIPT_STRING_ANALYSIS, int, BOOL, int* ) = Dummy_ScriptStringCPtoX;
HRESULT ( WINAPI*CUniBuffer::_ScriptStringXtoCP )( SCRIPT_STRING_ANALYSIS, int, int*, int* ) = Dummy_ScriptStringXtoCP;
HRESULT ( WINAPI*CUniBuffer::_ScriptStringFree )( SCRIPT_STRING_ANALYSIS* ) = Dummy_ScriptStringFree;
const SCRIPT_LOGATTR*   ( WINAPI*CUniBuffer::_ScriptString_pLogAttr )( SCRIPT_STRING_ANALYSIS ) =
    Dummy_ScriptString_pLogAttr;
const int*              ( WINAPI*CUniBuffer::_ScriptString_pcOutChars )( SCRIPT_STRING_ANALYSIS ) =
    Dummy_ScriptString_pcOutChars;
bool                    CDXUTEditBox::s_bHideCaret;   // If true, we don't render the caret.



//--------------------------------------------------------------------------------------
// CDXUTEditBox class
//--------------------------------------------------------------------------------------

// When scrolling, EDITBOX_SCROLLEXTENT is reciprocal of the amount to scroll.
// If EDITBOX_SCROLLEXTENT = 4, then we scroll 1/4 of the control each time.
#define EDITBOX_SCROLLEXTENT 4

//--------------------------------------------------------------------------------------
CDXUTEditBox::CDXUTEditBox( CDXUTDialog* pDialog )
{
    m_Type = DXUT_CONTROL_EDITBOX;
    m_pDialog = pDialog;

    m_nBorder = 5;  // Default border width
    m_nSpacing = 4;  // Default spacing

    m_bCaretOn = true;
    m_dfBlink = GetCaretBlinkTime() * 0.001f;
    m_dfLastBlink = DXUTGetGlobalTimer()->GetAbsoluteTime();
    s_bHideCaret = false;
    m_nFirstVisible = 0;
    m_TextColor = D3DCOLOR_ARGB( 255, 16, 16, 16 );
    m_SelTextColor = D3DCOLOR_ARGB( 255, 255, 255, 255 );
    m_SelBkColor = D3DCOLOR_ARGB( 255, 40, 50, 92 );
    m_CaretColor = D3DCOLOR_ARGB( 255, 0, 0, 0 );
    m_nCaret = m_nSelStart = 0;
    m_bInsertMode = true;

    m_bMouseDrag = false;
}


//--------------------------------------------------------------------------------------
CDXUTEditBox::~CDXUTEditBox()
{
}


//--------------------------------------------------------------------------------------
// PlaceCaret: Set the caret to a character position, and adjust the scrolling if
//             necessary.
//--------------------------------------------------------------------------------------
void CDXUTEditBox::PlaceCaret( int nCP )
{
    assert( nCP >= 0 && nCP <= m_Buffer.GetTextSize() );
    m_nCaret = nCP;

    // Obtain the X offset of the character.
    int nX1st, nX, nX2;
    m_Buffer.CPtoX( m_nFirstVisible, FALSE, &nX1st );  // 1st visible char
    m_Buffer.CPtoX( nCP, FALSE, &nX );  // LEAD
    // If nCP is the NULL terminator, get the leading edge instead of trailing.
    if( nCP == m_Buffer.GetTextSize() )
        nX2 = nX;
    else
        m_Buffer.CPtoX( nCP, TRUE, &nX2 );  // TRAIL

    // If the left edge of the char is smaller than the left edge of the 1st visible char,
    // we need to scroll left until this char is visible.
    if( nX < nX1st )
    {
        // Simply make the first visible character the char at the new caret position.
        m_nFirstVisible = nCP;
    }
    else // If the right of the character is bigger than the offset of the control's
    // right edge, we need to scroll right to this character.
    if( nX2 > nX1st + RectWidth( m_rcText ) )
    {
        // Compute the X of the new left-most pixel
        int nXNewLeft = nX2 - RectWidth( m_rcText );

        // Compute the char position of this character
        int nCPNew1st, nNewTrail;
        m_Buffer.XtoCP( nXNewLeft, &nCPNew1st, &nNewTrail );

        // If this coordinate is not on a character border,
        // start from the next character so that the caret
        // position does not fall outside the text rectangle.
        int nXNew1st;
        m_Buffer.CPtoX( nCPNew1st, FALSE, &nXNew1st );
        if( nXNew1st < nXNewLeft )
            ++nCPNew1st;

        m_nFirstVisible = nCPNew1st;
    }
}


//--------------------------------------------------------------------------------------
void CDXUTEditBox::ClearText()
{
    m_Buffer.Clear();
    m_nFirstVisible = 0;
    PlaceCaret( 0 );
    m_nSelStart = 0;
}


//--------------------------------------------------------------------------------------
void CDXUTEditBox::SetText( LPCWSTR wszText, bool bSelected )
{
    assert( wszText != NULL );

    m_Buffer.SetText( wszText );
    m_nFirstVisible = 0;
    // Move the caret to the end of the text
    PlaceCaret( m_Buffer.GetTextSize() );
    m_nSelStart = bSelected ? 0 : m_nCaret;
}


//--------------------------------------------------------------------------------------
HRESULT CDXUTEditBox::GetTextCopy( __out_ecount(bufferCount) LPWSTR strDest, 
                                   UINT bufferCount  )
{
    assert( strDest );

    wcscpy_s( strDest, bufferCount, m_Buffer.GetBuffer() );

    return S_OK;
}


//--------------------------------------------------------------------------------------
void CDXUTEditBox::DeleteSelectionText()
{
    int nFirst = __min( m_nCaret, m_nSelStart );
    int nLast = __max( m_nCaret, m_nSelStart );
    // Update caret and selection
    PlaceCaret( nFirst );
    m_nSelStart = m_nCaret;
    // Remove the characters
    for( int i = nFirst; i < nLast; ++i )
        m_Buffer.RemoveChar( nFirst );
}


//--------------------------------------------------------------------------------------
void CDXUTEditBox::UpdateRects()
{
    CDXUTControl::UpdateRects();

    // Update the text rectangle
    m_rcText = m_rcBoundingBox;
    // First inflate by m_nBorder to compute render rects
    InflateRect( &m_rcText, -m_nBorder, -m_nBorder );

    // Update the render rectangles
    m_rcRender[0] = m_rcText;
    SetRect( &m_rcRender[1], m_rcBoundingBox.left, m_rcBoundingBox.top, m_rcText.left, m_rcText.top );
    SetRect( &m_rcRender[2], m_rcText.left, m_rcBoundingBox.top, m_rcText.right, m_rcText.top );
    SetRect( &m_rcRender[3], m_rcText.right, m_rcBoundingBox.top, m_rcBoundingBox.right, m_rcText.top );
    SetRect( &m_rcRender[4], m_rcBoundingBox.left, m_rcText.top, m_rcText.left, m_rcText.bottom );
    SetRect( &m_rcRender[5], m_rcText.right, m_rcText.top, m_rcBoundingBox.right, m_rcText.bottom );
    SetRect( &m_rcRender[6], m_rcBoundingBox.left, m_rcText.bottom, m_rcText.left, m_rcBoundingBox.bottom );
    SetRect( &m_rcRender[7], m_rcText.left, m_rcText.bottom, m_rcText.right, m_rcBoundingBox.bottom );
    SetRect( &m_rcRender[8], m_rcText.right, m_rcText.bottom, m_rcBoundingBox.right, m_rcBoundingBox.bottom );

    // Inflate further by m_nSpacing
    InflateRect( &m_rcText, -m_nSpacing, -m_nSpacing );
}


void CDXUTEditBox::CopyToClipboard()
{
    // Copy the selection text to the clipboard
    if( m_nCaret != m_nSelStart && OpenClipboard( NULL ) )
    {
        EmptyClipboard();

        HGLOBAL hBlock = GlobalAlloc( GMEM_MOVEABLE, sizeof( WCHAR ) * ( m_Buffer.GetTextSize() + 1 ) );
        if( hBlock )
        {
            WCHAR* pwszText = ( WCHAR* )GlobalLock( hBlock );
            if( pwszText )
            {
                int nFirst = __min( m_nCaret, m_nSelStart );
                int nLast = __max( m_nCaret, m_nSelStart );
                if( nLast - nFirst > 0 )
                    CopyMemory( pwszText, m_Buffer.GetBuffer() + nFirst, ( nLast - nFirst ) * sizeof( WCHAR ) );
                pwszText[nLast - nFirst] = L'\0';  // Terminate it
                GlobalUnlock( hBlock );
            }
            SetClipboardData( CF_UNICODETEXT, hBlock );
        }
        CloseClipboard();
        // We must not free the object until CloseClipboard is called.
        if( hBlock )
            GlobalFree( hBlock );
    }
}


void CDXUTEditBox::PasteFromClipboard()
{
    DeleteSelectionText();

    if( OpenClipboard( NULL ) )
    {
        HANDLE handle = GetClipboardData( CF_UNICODETEXT );
        if( handle )
        {
            // Convert the ANSI string to Unicode, then
            // insert to our buffer.
            WCHAR* pwszText = ( WCHAR* )GlobalLock( handle );
            if( pwszText )
            {
                // Copy all characters up to null.
                if( m_Buffer.InsertString( m_nCaret, pwszText ) )
                    PlaceCaret( m_nCaret + lstrlenW( pwszText ) );
                m_nSelStart = m_nCaret;
                GlobalUnlock( handle );
            }
        }
        CloseClipboard();
    }
}


//--------------------------------------------------------------------------------------
bool CDXUTEditBox::HandleKeyboard( UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    bool bHandled = false;

    switch( uMsg )
    {
        case WM_KEYDOWN:
        {
            switch( wParam )
            {
                case VK_TAB:
                    // We don't process Tab in case keyboard input is enabled and the user
                    // wishes to Tab to other controls.
                    break;

                case VK_HOME:
                    PlaceCaret( 0 );
                    if( GetKeyState( VK_SHIFT ) >= 0 )
                        // Shift is not down. Update selection
                        // start along with the caret.
                        m_nSelStart = m_nCaret;
                    ResetCaretBlink();
                    bHandled = true;
                    break;

                case VK_END:
                    PlaceCaret( m_Buffer.GetTextSize() );
                    if( GetKeyState( VK_SHIFT ) >= 0 )
                        // Shift is not down. Update selection
                        // start along with the caret.
                        m_nSelStart = m_nCaret;
                    ResetCaretBlink();
                    bHandled = true;
                    break;

                case VK_INSERT:
                    if( GetKeyState( VK_CONTROL ) < 0 )
                    {
                        // Control Insert. Copy to clipboard
                        CopyToClipboard();
                    }
                    else if( GetKeyState( VK_SHIFT ) < 0 )
                    {
                        // Shift Insert. Paste from clipboard
                        PasteFromClipboard();
                    }
                    else
                    {
                        // Toggle caret insert mode
                        m_bInsertMode = !m_bInsertMode;
                    }
                    break;

                case VK_DELETE:
                    // Check if there is a text selection.
                    if( m_nCaret != m_nSelStart )
                    {
                        DeleteSelectionText();
                        m_pDialog->SendEvent( EVENT_EDITBOX_CHANGE, true, this );
                    }
                    else
                    {
                        // Deleting one character
                        if( m_Buffer.RemoveChar( m_nCaret ) )
                            m_pDialog->SendEvent( EVENT_EDITBOX_CHANGE, true, this );
                    }
                    ResetCaretBlink();
                    bHandled = true;
                    break;

                case VK_LEFT:
                    if( GetKeyState( VK_CONTROL ) < 0 )
                    {
                        // Control is down. Move the caret to a new item
                        // instead of a character.
                        m_Buffer.GetPriorItemPos( m_nCaret, &m_nCaret );
                        PlaceCaret( m_nCaret );
                    }
                    else if( m_nCaret > 0 )
                        PlaceCaret( m_nCaret - 1 );
                    if( GetKeyState( VK_SHIFT ) >= 0 )
                        // Shift is not down. Update selection
                        // start along with the caret.
                        m_nSelStart = m_nCaret;
                    ResetCaretBlink();
                    bHandled = true;
                    break;

                case VK_RIGHT:
                    if( GetKeyState( VK_CONTROL ) < 0 )
                    {
                        // Control is down. Move the caret to a new item
                        // instead of a character.
                        m_Buffer.GetNextItemPos( m_nCaret, &m_nCaret );
                        PlaceCaret( m_nCaret );
                    }
                    else if( m_nCaret < m_Buffer.GetTextSize() )
                        PlaceCaret( m_nCaret + 1 );
                    if( GetKeyState( VK_SHIFT ) >= 0 )
                        // Shift is not down. Update selection
                        // start along with the caret.
                        m_nSelStart = m_nCaret;
                    ResetCaretBlink();
                    bHandled = true;
                    break;

                case VK_UP:
                case VK_DOWN:
                    // Trap up and down arrows so that the dialog
                    // does not switch focus to another control.
                    bHandled = true;
                    break;

                default:
                    bHandled = wParam != VK_ESCAPE;  // Let the application handle Esc.
            }
        }
    }
    return bHandled;
}


//--------------------------------------------------------------------------------------
bool CDXUTEditBox::HandleMouse( UINT uMsg, POINT pt, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    switch( uMsg )
    {
        case WM_LBUTTONDOWN:
        case WM_LBUTTONDBLCLK:
            {
                if( !m_bHasFocus )
                    m_pDialog->RequestFocus( this );

                if( !ContainsPoint( pt ) )
                    return false;

                m_bMouseDrag = true;
                SetCapture( DXUTGetHWND() );
                // Determine the character corresponding to the coordinates.
                int nCP, nTrail, nX1st;
                m_Buffer.CPtoX( m_nFirstVisible, FALSE, &nX1st );  // X offset of the 1st visible char
                if( SUCCEEDED( m_Buffer.XtoCP( pt.x - m_rcText.left + nX1st, &nCP, &nTrail ) ) )
                {
                    // Cap at the NULL character.
                    if( nTrail && nCP < m_Buffer.GetTextSize() )
                        PlaceCaret( nCP + 1 );
                    else
                        PlaceCaret( nCP );
                    m_nSelStart = m_nCaret;
                    ResetCaretBlink();
                }
                return true;
            }

        case WM_LBUTTONUP:
            ReleaseCapture();
            m_bMouseDrag = false;
            break;

        case WM_MOUSEMOVE:
            if( m_bMouseDrag )
            {
                // Determine the character corresponding to the coordinates.
                int nCP, nTrail, nX1st;
                m_Buffer.CPtoX( m_nFirstVisible, FALSE, &nX1st );  // X offset of the 1st visible char
                if( SUCCEEDED( m_Buffer.XtoCP( pt.x - m_rcText.left + nX1st, &nCP, &nTrail ) ) )
                {
                    // Cap at the NULL character.
                    if( nTrail && nCP < m_Buffer.GetTextSize() )
                        PlaceCaret( nCP + 1 );
                    else
                        PlaceCaret( nCP );
                }
            }
            break;
    }

    return false;
}


//--------------------------------------------------------------------------------------
void CDXUTEditBox::OnFocusIn()
{
    CDXUTControl::OnFocusIn();

    ResetCaretBlink();
}


//--------------------------------------------------------------------------------------
bool CDXUTEditBox::MsgProc( UINT uMsg, WPARAM wParam, LPARAM lParam )
{
    if( !m_bEnabled || !m_bVisible )
        return false;

    switch( uMsg )
    {
            // Make sure that while editing, the keyup and keydown messages associated with 
            // WM_CHAR messages don't go to any non-focused controls or cameras
        case WM_KEYUP:
        case WM_KEYDOWN:
            return true;

        case WM_CHAR:
        {
            switch( ( WCHAR )wParam )
            {
                    // Backspace
                case VK_BACK:
                {
                    // If there's a selection, treat this
                    // like a delete key.
                    if( m_nCaret != m_nSelStart )
                    {
                        DeleteSelectionText();
                        m_pDialog->SendEvent( EVENT_EDITBOX_CHANGE, true, this );
                    }
                    else if( m_nCaret > 0 )
                    {
                        // Move the caret, then delete the char.
                        PlaceCaret( m_nCaret - 1 );
                        m_nSelStart = m_nCaret;
                        m_Buffer.RemoveChar( m_nCaret );
                        m_pDialog->SendEvent( EVENT_EDITBOX_CHANGE, true, this );
                    }
                    ResetCaretBlink();
                    break;
                }

                case 24:        // Ctrl-X Cut
                case VK_CANCEL: // Ctrl-C Copy
                    {
                        CopyToClipboard();

                        // If the key is Ctrl-X, delete the selection too.
                        if( ( WCHAR )wParam == 24 )
                        {
                            DeleteSelectionText();
                            m_pDialog->SendEvent( EVENT_EDITBOX_CHANGE, true, this );
                        }

                        break;
                    }

                    // Ctrl-V Paste
                case 22:
                {
                    PasteFromClipboard();
                    m_pDialog->SendEvent( EVENT_EDITBOX_CHANGE, true, this );
                    break;
                }

                    // Ctrl-A Select All
                case 1:
                    if( m_nSelStart == m_nCaret )
                    {
                        m_nSelStart = 0;
                        PlaceCaret( m_Buffer.GetTextSize() );
                    }
                    break;

                case VK_RETURN:
                    // Invoke the callback when the user presses Enter.
                    m_pDialog->SendEvent( EVENT_EDITBOX_STRING, true, this );
                    break;

                    // Junk characters we don't want in the string
                case 26:  // Ctrl Z
                case 2:   // Ctrl B
                case 14:  // Ctrl N
                case 19:  // Ctrl S
                case 4:   // Ctrl D
                case 6:   // Ctrl F
                case 7:   // Ctrl G
                case 10:  // Ctrl J
                case 11:  // Ctrl K
                case 12:  // Ctrl L
                case 17:  // Ctrl Q
                case 23:  // Ctrl W
                case 5:   // Ctrl E
                case 18:  // Ctrl R
                case 20:  // Ctrl T
                case 25:  // Ctrl Y
                case 21:  // Ctrl U
                case 9:   // Ctrl I
                case 15:  // Ctrl O
                case 16:  // Ctrl P
                case 27:  // Ctrl [
                case 29:  // Ctrl ]
                case 28:  // Ctrl \ 
                    break;

                default:
                {
                    // If there's a selection and the user
                    // starts to type, the selection should
                    // be deleted.
                    if( m_nCaret != m_nSelStart )
                        DeleteSelectionText();

                    // If we are in overwrite mode and there is already
                    // a char at the caret's position, simply replace it.
                    // Otherwise, we insert the char as normal.
                    if( !m_bInsertMode && m_nCaret < m_Buffer.GetTextSize() )
                    {
                        m_Buffer[m_nCaret] = ( WCHAR )wParam;
                        PlaceCaret( m_nCaret + 1 );
                        m_nSelStart = m_nCaret;
                    }
                    else
                    {
                        // Insert the char
                        if( m_Buffer.InsertChar( m_nCaret, ( WCHAR )wParam ) )
                        {
                            PlaceCaret( m_nCaret + 1 );
                            m_nSelStart = m_nCaret;
                        }
                    }
                    ResetCaretBlink();
                    m_pDialog->SendEvent( EVENT_EDITBOX_CHANGE, true, this );
                }
            }
            return true;
        }
    }
    return false;
}


//--------------------------------------------------------------------------------------
void CDXUTEditBox::Render( float fElapsedTime )
{
    if( m_bVisible == false )
        return;

    HRESULT hr;
    int nSelStartX = 0, nCaretX = 0;  // Left and right X cordinates of the selection region

    CDXUTElement* pElement = GetElement( 0 );
    if( pElement )
    {
        m_Buffer.SetFontNode( m_pDialog->GetFont( pElement->iFont ) );
        PlaceCaret( m_nCaret );  // Call PlaceCaret now that we have the font info (node),
        // so that scrolling can be handled.
    }

    // Render the control graphics
    for( int e = 0; e < 9; ++e )
    {
        pElement = m_Elements.GetAt( e );
        pElement->TextureColor.Blend( DXUT_STATE_NORMAL, fElapsedTime );

        m_pDialog->DrawSprite( pElement, &m_rcRender[e], DXUT_FAR_BUTTON_DEPTH );
    }

    //
    // Compute the X coordinates of the first visible character.
    //
    int nXFirst;
    m_Buffer.CPtoX( m_nFirstVisible, FALSE, &nXFirst );

    //
    // Compute the X coordinates of the selection rectangle
    //
    hr = m_Buffer.CPtoX( m_nCaret, FALSE, &nCaretX );
    if( m_nCaret != m_nSelStart )
        hr = m_Buffer.CPtoX( m_nSelStart, FALSE, &nSelStartX );
    else
        nSelStartX = nCaretX;

    //
    // Render the selection rectangle
    //
    RECT rcSelection;  // Make this available for rendering selected text
    if( m_nCaret != m_nSelStart )
    {
        int nSelLeftX = nCaretX, nSelRightX = nSelStartX;
        // Swap if left is bigger than right
        if( nSelLeftX > nSelRightX )
        {
            int nTemp = nSelLeftX; nSelLeftX = nSelRightX; nSelRightX = nTemp;
        }

        SetRect( &rcSelection, nSelLeftX, m_rcText.top, nSelRightX, m_rcText.bottom );
        OffsetRect( &rcSelection, m_rcText.left - nXFirst, 0 );
        IntersectRect( &rcSelection, &m_rcText, &rcSelection );

        IDirect3DDevice9* pd3dDevice = m_pDialog->GetManager()->GetD3D9Device();
        if( pd3dDevice )
            pd3dDevice->SetRenderState( D3DRS_ZENABLE, FALSE );
        m_pDialog->DrawRect( &rcSelection, m_SelBkColor );
        if( pd3dDevice )
            pd3dDevice->SetRenderState( D3DRS_ZENABLE, TRUE );
    }

    //
    // Render the text
    //
    // Element 0 for text
    m_Elements.GetAt( 0 )->FontColor.Current = m_TextColor;
    m_pDialog->DrawText( m_Buffer.GetBuffer() + m_nFirstVisible, m_Elements.GetAt( 0 ), &m_rcText );

    // Render the selected text
    if( m_nCaret != m_nSelStart )
    {
        int nFirstToRender = __max( m_nFirstVisible, __min( m_nSelStart, m_nCaret ) );
        int nNumChatToRender = __max( m_nSelStart, m_nCaret ) - nFirstToRender;
        m_Elements.GetAt( 0 )->FontColor.Current = m_SelTextColor;
        m_pDialog->DrawText( m_Buffer.GetBuffer() + nFirstToRender,
                             m_Elements.GetAt( 0 ), &rcSelection, false, nNumChatToRender );
    }

    //
    // Blink the caret
    //
    if( DXUTGetGlobalTimer()->GetAbsoluteTime() - m_dfLastBlink >= m_dfBlink )
    {
        m_bCaretOn = !m_bCaretOn;
        m_dfLastBlink = DXUTGetGlobalTimer()->GetAbsoluteTime();
    }

    //
    // Render the caret if this control has the focus
    //
    if( m_bHasFocus && m_bCaretOn && !s_bHideCaret )
    {
        // Start the rectangle with insert mode caret
        RECT rcCaret =
        {
            m_rcText.left - nXFirst + nCaretX - 1, m_rcText.top,
            m_rcText.left - nXFirst + nCaretX + 1, m_rcText.bottom
        };

        // If we are in overwrite mode, adjust the caret rectangle
        // to fill the entire character.
        if( !m_bInsertMode )
        {
            // Obtain the right edge X coord of the current character
            int nRightEdgeX;
            m_Buffer.CPtoX( m_nCaret, TRUE, &nRightEdgeX );
            rcCaret.right = m_rcText.left - nXFirst + nRightEdgeX;
        }

        m_pDialog->DrawRect( &rcCaret, m_CaretColor );
    }
}


#define IN_FLOAT_CHARSET( c ) \
    ( (c) == L'-' || (c) == L'.' || ( (c) >= L'0' && (c) <= L'9' ) )

void CDXUTEditBox::ParseFloatArray( float* pNumbers, int nCount )
{
    int nWritten = 0;  // Number of floats written
    const WCHAR* pToken, *pEnd;
    WCHAR wszToken[60];

    pToken = m_Buffer.GetBuffer();
    while( nWritten < nCount && *pToken != L'\0' )
    {
        // Skip leading spaces
        while( *pToken == L' ' )
            ++pToken;

        if( *pToken == L'\0' )
            break;

        // Locate the end of number
        pEnd = pToken;
        while( IN_FLOAT_CHARSET( *pEnd ) )
            ++pEnd;

        // Copy the token to our buffer
        int nTokenLen = __min( sizeof( wszToken ) / sizeof( wszToken[0] ) - 1, int( pEnd - pToken ) );
        wcscpy_s( wszToken, nTokenLen, pToken );
        *pNumbers = ( float )wcstod( wszToken, NULL );
        ++nWritten;
        ++pNumbers;
        pToken = pEnd;
    }
}


void CDXUTEditBox::SetTextFloatArray( const float* pNumbers, int nCount )
{
    WCHAR wszBuffer[512] =
    {
        0
    };
    WCHAR wszTmp[64];

    if( pNumbers == NULL )
        return;

    for( int i = 0; i < nCount; ++i )
    {
        swprintf_s( wszTmp, 64, L"%.4f ", pNumbers[i] );
        wcscat_s( wszBuffer, 512, wszTmp );
    }

    // Don't want the last space
    if( nCount > 0 && wcslen( wszBuffer ) > 0 )
        wszBuffer[wcslen( wszBuffer ) - 1] = 0;

    SetText( wszBuffer );
}




//--------------------------------------------------------------------------------------
void CUniBuffer::Initialize()
{
    if( s_hDll ) // Only need to do once
        return;

    s_hDll = LoadLibrary( UNISCRIBE_DLLNAME );
    if( s_hDll )
    {
        FARPROC Temp;
        GETPROCADDRESS( s_hDll, ScriptApplyDigitSubstitution, Temp );
        GETPROCADDRESS( s_hDll, ScriptStringAnalyse, Temp );
        GETPROCADDRESS( s_hDll, ScriptStringCPtoX, Temp );
        GETPROCADDRESS( s_hDll, ScriptStringXtoCP, Temp );
        GETPROCADDRESS( s_hDll, ScriptStringFree, Temp );
        GETPROCADDRESS( s_hDll, ScriptString_pLogAttr, Temp );
        GETPROCADDRESS( s_hDll, ScriptString_pcOutChars, Temp );
    }
}


//--------------------------------------------------------------------------------------
void CUniBuffer::Uninitialize()
{
    if( s_hDll )
    {
        PLACEHOLDERPROC( ScriptApplyDigitSubstitution );
        PLACEHOLDERPROC( ScriptStringAnalyse );
        PLACEHOLDERPROC( ScriptStringCPtoX );
        PLACEHOLDERPROC( ScriptStringXtoCP );
        PLACEHOLDERPROC( ScriptStringFree );
        PLACEHOLDERPROC( ScriptString_pLogAttr );
        PLACEHOLDERPROC( ScriptString_pcOutChars );

        FreeLibrary( s_hDll );
        s_hDll = NULL;
    }
}


//--------------------------------------------------------------------------------------
bool CUniBuffer::SetBufferSize( int nNewSize )
{
    // If the current size is already the maximum allowed,
    // we can't possibly allocate more.
    if( m_nBufferSize == DXUT_MAX_EDITBOXLENGTH )
        return false;

    int nAllocateSize = ( nNewSize == -1 || nNewSize < m_nBufferSize * 2 ) ? ( m_nBufferSize ? m_nBufferSize *
                                                                               2 : 256 ) : nNewSize * 2;

    // Cap the buffer size at the maximum allowed.
    if( nAllocateSize > DXUT_MAX_EDITBOXLENGTH )
        nAllocateSize = DXUT_MAX_EDITBOXLENGTH;

    WCHAR* pTempBuffer = new WCHAR[nAllocateSize];
    if( !pTempBuffer )
        return false;

    ZeroMemory( pTempBuffer, sizeof( WCHAR ) * nAllocateSize );

    if( m_pwszBuffer )
    {
        CopyMemory( pTempBuffer, m_pwszBuffer, m_nBufferSize * sizeof( WCHAR ) );
        delete[] m_pwszBuffer;
    }

    m_pwszBuffer = pTempBuffer;
    m_nBufferSize = nAllocateSize;
    return true;
}


//--------------------------------------------------------------------------------------
// Uniscribe -- Analyse() analyses the string in the buffer
//--------------------------------------------------------------------------------------
HRESULT CUniBuffer::Analyse()
{
    if( m_Analysis )
        _ScriptStringFree( &m_Analysis );

    SCRIPT_CONTROL ScriptControl; // For uniscribe
    SCRIPT_STATE ScriptState;   // For uniscribe
    ZeroMemory( &ScriptControl, sizeof( ScriptControl ) );
    ZeroMemory( &ScriptState, sizeof( ScriptState ) );
    _ScriptApplyDigitSubstitution( NULL, &ScriptControl, &ScriptState );

    if( !m_pFontNode )
        return E_FAIL;

    HDC hDC = 
        ( m_pFontNode->pFont9 ? m_pFontNode->pFont9->GetDC() : NULL );
    HRESULT hr = _ScriptStringAnalyse( hDC,
                                       m_pwszBuffer,
                                       lstrlenW( m_pwszBuffer ) + 1,  // NULL is also analyzed.
                                       lstrlenW( m_pwszBuffer ) * 3 / 2 + 16,
                                       -1,
                                       SSA_BREAK | SSA_GLYPHS | SSA_FALLBACK | SSA_LINK,
                                       0,
                                       &ScriptControl,
                                       &ScriptState,
                                       NULL,
                                       NULL,
                                       NULL,
                                       &m_Analysis );
    if( SUCCEEDED( hr ) )
        m_bAnalyseRequired = false;  // Analysis is up-to-date
    return hr;
}


//--------------------------------------------------------------------------------------
CUniBuffer::CUniBuffer( int nInitialSize )
{
    CUniBuffer::Initialize();  // ensure static vars are properly init'ed first

    m_nBufferSize = 0;
    m_pwszBuffer = NULL;
    m_bAnalyseRequired = true;
    m_Analysis = NULL;
    m_pFontNode = NULL;

    if( nInitialSize > 0 )
        SetBufferSize( nInitialSize );
}


//--------------------------------------------------------------------------------------
CUniBuffer::~CUniBuffer()
{
    delete[] m_pwszBuffer;
    if( m_Analysis )
        _ScriptStringFree( &m_Analysis );
}


//--------------------------------------------------------------------------------------
WCHAR& CUniBuffer::operator[]( int n )  // No param checking
{
    // This version of operator[] is called only
    // if we are asking for write access, so
    // re-analysis is required.
    m_bAnalyseRequired = true;
    return m_pwszBuffer[n];
}


//--------------------------------------------------------------------------------------
void CUniBuffer::Clear()
{
    *m_pwszBuffer = L'\0';
    m_bAnalyseRequired = true;
}


//--------------------------------------------------------------------------------------
// Inserts the char at specified index.
// If nIndex == -1, insert to the end.
//--------------------------------------------------------------------------------------
bool CUniBuffer::InsertChar( int nIndex, WCHAR wChar )
{
    assert( nIndex >= 0 );

    if( nIndex < 0 || nIndex > lstrlenW( m_pwszBuffer ) )
        return false;  // invalid index

    // Check for maximum length allowed
    if( GetTextSize() + 1 >= DXUT_MAX_EDITBOXLENGTH )
        return false;

    if( lstrlenW( m_pwszBuffer ) + 1 >= m_nBufferSize )
    {
        if( !SetBufferSize( -1 ) )
            return false;  // out of memory
    }

    assert( m_nBufferSize >= 2 );

    // Shift the characters after the index, start by copying the null terminator
    WCHAR* dest = m_pwszBuffer + lstrlenW( m_pwszBuffer ) + 1;
    WCHAR* stop = m_pwszBuffer + nIndex;
    WCHAR* src = dest - 1;

    while( dest > stop )
    {
        *dest-- = *src--;
    }

    // Set new character
    m_pwszBuffer[ nIndex ] = wChar;
    m_bAnalyseRequired = true;

    return true;
}


//--------------------------------------------------------------------------------------
// Removes the char at specified index.
// If nIndex == -1, remove the last char.
//--------------------------------------------------------------------------------------
bool CUniBuffer::RemoveChar( int nIndex )
{
    if( !lstrlenW( m_pwszBuffer ) || nIndex < 0 || nIndex >= lstrlenW( m_pwszBuffer ) )
        return false;  // Invalid index

    MoveMemory( m_pwszBuffer + nIndex, m_pwszBuffer + nIndex + 1, sizeof( WCHAR ) *
                ( lstrlenW( m_pwszBuffer ) - nIndex ) );
    m_bAnalyseRequired = true;
    return true;
}


//--------------------------------------------------------------------------------------
// Inserts the first nCount characters of the string pStr at specified index.
// If nCount == -1, the entire string is inserted.
// If nIndex == -1, insert to the end.
//--------------------------------------------------------------------------------------
bool CUniBuffer::InsertString( int nIndex, const WCHAR* pStr, int nCount )
{
    assert( nIndex >= 0 );
    if( nIndex < 0 )
        return false;

    if( nIndex > lstrlenW( m_pwszBuffer ) )
        return false;  // invalid index

    if( -1 == nCount )
        nCount = lstrlenW( pStr );

    // Check for maximum length allowed
    if( GetTextSize() + nCount >= DXUT_MAX_EDITBOXLENGTH )
        return false;

    if( lstrlenW( m_pwszBuffer ) + nCount >= m_nBufferSize )
    {
        if( !SetBufferSize( lstrlenW( m_pwszBuffer ) + nCount + 1 ) )
            return false;  // out of memory
    }

    MoveMemory( m_pwszBuffer + nIndex + nCount, m_pwszBuffer + nIndex, sizeof( WCHAR ) *
                ( lstrlenW( m_pwszBuffer ) - nIndex + 1 ) );
    CopyMemory( m_pwszBuffer + nIndex, pStr, nCount * sizeof( WCHAR ) );
    m_bAnalyseRequired = true;

    return true;
}


//--------------------------------------------------------------------------------------
bool CUniBuffer::SetText( LPCWSTR wszText )
{
    assert( wszText != NULL );

    int nRequired = int( wcslen( wszText ) + 1 );

    // Check for maximum length allowed
    if( nRequired >= DXUT_MAX_EDITBOXLENGTH )
        return false;

    while( GetBufferSize() < nRequired )
        if( !SetBufferSize( -1 ) )
            break;
    // Check again in case out of memory occurred inside while loop.
    if( GetBufferSize() >= nRequired )
    {
        wcscpy_s( m_pwszBuffer, GetBufferSize(), wszText );
        m_bAnalyseRequired = true;
        return true;
    }
    else
        return false;
}


//--------------------------------------------------------------------------------------
HRESULT CUniBuffer::CPtoX( int nCP, BOOL bTrail, int* pX )
{
    assert( pX );
    *pX = 0;  // Default

    HRESULT hr = S_OK;
    if( m_bAnalyseRequired )
        hr = Analyse();

    if( SUCCEEDED( hr ) )
        hr = _ScriptStringCPtoX( m_Analysis, nCP, bTrail, pX );

    return hr;
}


//--------------------------------------------------------------------------------------
HRESULT CUniBuffer::XtoCP( int nX, int* pCP, int* pnTrail )
{
    assert( pCP && pnTrail );
    *pCP = 0; *pnTrail = FALSE;  // Default

    HRESULT hr = S_OK;
    if( m_bAnalyseRequired )
        hr = Analyse();

    if( SUCCEEDED( hr ) )
        hr = _ScriptStringXtoCP( m_Analysis, nX, pCP, pnTrail );

    // If the coordinate falls outside the text region, we
    // can get character positions that don't exist.  We must
    // filter them here and convert them to those that do exist.
    if( *pCP == -1 && *pnTrail == TRUE )
    {
        *pCP = 0; *pnTrail = FALSE;
    }
    else if( *pCP > lstrlenW( m_pwszBuffer ) && *pnTrail == FALSE )
    {
        *pCP = lstrlenW( m_pwszBuffer ); *pnTrail = TRUE;
    }

    return hr;
}


//--------------------------------------------------------------------------------------
void CUniBuffer::GetPriorItemPos( int nCP, int* pPrior )
{
    *pPrior = nCP;  // Default is the char itself

    if( m_bAnalyseRequired )
        if( FAILED( Analyse() ) )
            return;

    const SCRIPT_LOGATTR* pLogAttr = _ScriptString_pLogAttr( m_Analysis );
    if( !pLogAttr )
        return;

    if( !_ScriptString_pcOutChars( m_Analysis ) )
        return;
    int nInitial = *_ScriptString_pcOutChars( m_Analysis );
    if( nCP - 1 < nInitial )
        nInitial = nCP - 1;
    for( int i = nInitial; i > 0; --i )
        if( pLogAttr[i].fWordStop ||       // Either the fWordStop flag is set
            ( !pLogAttr[i].fWhiteSpace &&  // Or the previous char is whitespace but this isn't.
              pLogAttr[i - 1].fWhiteSpace ) )
        {
            *pPrior = i;
            return;
        }
    // We have reached index 0.  0 is always a break point, so simply return it.
    *pPrior = 0;
}


//--------------------------------------------------------------------------------------
void CUniBuffer::GetNextItemPos( int nCP, int* pPrior )
{
    *pPrior = nCP;  // Default is the char itself

    HRESULT hr = S_OK;
    if( m_bAnalyseRequired )
        hr = Analyse();
    if( FAILED( hr ) )
        return;

    const SCRIPT_LOGATTR* pLogAttr = _ScriptString_pLogAttr( m_Analysis );
    if( !pLogAttr )
        return;

    if( !_ScriptString_pcOutChars( m_Analysis ) )
        return;
    int nInitial = *_ScriptString_pcOutChars( m_Analysis );
    if( nCP + 1 < nInitial )
        nInitial = nCP + 1;

    int i = nInitial;
    int limit = *_ScriptString_pcOutChars( m_Analysis );
    while( limit > 0 && i < limit - 1 )
    {
        if( pLogAttr[i].fWordStop )      // Either the fWordStop flag is set
        {
            *pPrior = i;
            return;
        }
        else if( pLogAttr[i].fWhiteSpace &&  // Or this whitespace but the next char isn't.
                 !pLogAttr[i + 1].fWhiteSpace )
        {
            *pPrior = i + 1;  // The next char is a word stop
            return;
        }

        ++i;
        limit = *_ScriptString_pcOutChars( m_Analysis );
    }
    // We have reached the end. It's always a word stop, so simply return it.
    *pPrior = *_ScriptString_pcOutChars( m_Analysis ) - 1;
}


//--------------------------------------------------------------------------------------
void CDXUTEditBox::ResetCaretBlink()
{
    m_bCaretOn = true;
    m_dfLastBlink = DXUTGetGlobalTimer()->GetAbsoluteTime();
}


//--------------------------------------------------------------------------------------
void DXUTBlendColor::Init( D3DCOLOR defaultColor, D3DCOLOR disabledColor, D3DCOLOR hiddenColor )
{
    for( int i = 0; i < MAX_CONTROL_STATES; i++ )
    {
        States[ i ] = defaultColor;
    }

    States[ DXUT_STATE_DISABLED ] = disabledColor;
    States[ DXUT_STATE_HIDDEN ] = hiddenColor;
    Current = hiddenColor;
}


//--------------------------------------------------------------------------------------
void DXUTBlendColor::Blend( UINT iState, float fElapsedTime, float fRate )
{
    D3DXCOLOR destColor = States[ iState ];
    D3DXColorLerp( &Current, &Current, &destColor, 1.0f - powf( fRate, 30 * fElapsedTime ) );
}



//--------------------------------------------------------------------------------------
void CDXUTElement::SetTexture( UINT iTexture, RECT* prcTexture, D3DCOLOR defaultTextureColor )
{
    this->iTexture = iTexture;

    if( prcTexture )
        rcTexture = *prcTexture;
    else
        SetRectEmpty( &rcTexture );

    TextureColor.Init( defaultTextureColor );
}


//--------------------------------------------------------------------------------------
void CDXUTElement::SetFont( UINT iFont, D3DCOLOR defaultFontColor, DWORD dwTextFormat )
{
    this->iFont = iFont;
    this->dwTextFormat = dwTextFormat;

    FontColor.Init( defaultFontColor );
}


//--------------------------------------------------------------------------------------
void CDXUTElement::Refresh()
{
    TextureColor.Current = TextureColor.States[ DXUT_STATE_HIDDEN ];
    FontColor.Current = FontColor.States[ DXUT_STATE_HIDDEN ];
}


