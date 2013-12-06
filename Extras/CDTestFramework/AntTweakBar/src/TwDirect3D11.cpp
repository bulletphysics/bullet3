//  ---------------------------------------------------------------------------
//
//  @file       TwDirect3D11.cpp
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  ---------------------------------------------------------------------------


#include "TwPrecomp.h"
#include "TwDirect3D11.h"
#include "TwMgr.h"
#include "TwColors.h"

#include "d3d10vs2003.h" // Workaround to include D3D10.h and D3D11.h with VS2003
#define D3D11_IGNORE_SDK_LAYERS // d3d11sdklayers.h may not exist
#include <d3d11.h>


using namespace std;

const char *g_ErrCantLoadD3D11   = "Cannot load Direct3D11 library dynamically";
const char *g_ErrCreateVS11      = "Direct3D11 vertex shader creation failed";
const char *g_ErrCreatePS11      = "Direct3D11 pixel shader creation failed";
const char *g_ErrCreateLayout11  = "Direct3D11 vertex layout creation failed";
const char *g_ErrCreateBuffer11  = "Direct3D11 vertex buffer creation failed";
const char *g_ErrCreateSampler11 = "Direct3D11 sampler state creation failed";

//  ---------------------------------------------------------------------------
//  Shaders : In order to avoid linkage with D3DX11 or D3DCompile libraries,
//  vertex and pixel shaders are compiled offline in a pre-build step using
//  the fxc.exe compiler (from the DirectX SDK Aug'09 or later)

#ifdef _WIN64
#   ifdef _DEBUG
#       include "debug64\TwDirect3D11_LineRectVS.h"
#       include "debug64\TwDirect3D11_LineRectCstColorVS.h"
#       include "debug64\TwDirect3D11_LineRectPS.h"
#       include "debug64\TwDirect3D11_TextVS.h"
#       include "debug64\TwDirect3D11_TextCstColorVS.h"
#       include "debug64\TwDirect3D11_TextPS.h"
#   else
#       include "release64\TwDirect3D11_LineRectVS.h"
#       include "release64\TwDirect3D11_LineRectCstColorVS.h"
#       include "release64\TwDirect3D11_LineRectPS.h"
#       include "release64\TwDirect3D11_TextVS.h"
#       include "release64\TwDirect3D11_TextCstColorVS.h"
#       include "release64\TwDirect3D11_TextPS.h"
#   endif
#else
#   ifdef _DEBUG
#       include "debug32\TwDirect3D11_LineRectVS.h"
#       include "debug32\TwDirect3D11_LineRectCstColorVS.h"
#       include "debug32\TwDirect3D11_LineRectPS.h"
#       include "debug32\TwDirect3D11_TextVS.h"
#       include "debug32\TwDirect3D11_TextCstColorVS.h"
#       include "debug32\TwDirect3D11_TextPS.h"
#   else
#       include "release32\TwDirect3D11_LineRectVS.h"
#       include "release32\TwDirect3D11_LineRectCstColorVS.h"
#       include "release32\TwDirect3D11_LineRectPS.h"
#       include "release32\TwDirect3D11_TextVS.h"
#       include "release32\TwDirect3D11_TextCstColorVS.h"
#       include "release32\TwDirect3D11_TextPS.h"
#   endif
#endif

//  ---------------------------------------------------------------------------

const RECT FullRect = {0, 0, 16000, 16000};
static bool RectIsFull(const RECT& r) { return r.left==FullRect.left && r.right==FullRect.right && r.top==FullRect.top && r.bottom==FullRect.bottom; }

//  ---------------------------------------------------------------------------

static void BindFont(ID3D11Device *_Dev, const CTexFont *_Font, ID3D11Texture2D **_Tex, ID3D11ShaderResourceView **_TexRV)
{
    assert(_Font!=NULL);
    *_Tex = NULL;
    *_TexRV = NULL;

    int w = _Font->m_TexWidth;
    int h = _Font->m_TexHeight;
    color32 *font32 = new color32[w*h];
    color32 *p = font32;
    for( int i=0; i<w*h; ++i, ++p )
        *p = 0x00ffffff | (((color32)(_Font->m_TexBytes[i]))<<24);

    D3D11_TEXTURE2D_DESC desc;
    desc.Width = w;
    desc.Height = h;
    desc.MipLevels = 1;
    desc.ArraySize = 1;
    desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    desc.SampleDesc.Count = 1;
    desc.SampleDesc.Quality = 0;
    desc.Usage = D3D11_USAGE_IMMUTABLE;
    desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
    desc.CPUAccessFlags = 0;
    desc.MiscFlags = 0;
    D3D11_SUBRESOURCE_DATA data;
    data.pSysMem = font32;
    data.SysMemPitch = w*sizeof(color32);
    data.SysMemSlicePitch = 0;

    if( SUCCEEDED(_Dev->CreateTexture2D(&desc, &data, _Tex)) )
        _Dev->CreateShaderResourceView(*_Tex, NULL, _TexRV);

    delete[] font32;
}

//  ---------------------------------------------------------------------------

static void UnbindFont(ID3D11Device *_Dev, ID3D11Texture2D *_Tex, ID3D11ShaderResourceView *_TexRV)
{
    (void)_Dev;

    if( _TexRV )
    {
        ULONG rc = _TexRV->Release();
        assert( rc==0 ); (void)rc;
    }
    if( _Tex )
    {
        ULONG rc = _Tex->Release();
        assert( rc==0 ); (void)rc;
    }
}

//  ---------------------------------------------------------------------------

struct CState11
{
    ID3D11ComputeShader *   m_CSShader;
    ID3D11ClassInstance **  m_CSClassInstances;
    UINT                    m_CSNumClassInstances;
    ID3D11DomainShader *    m_DSShader;
    ID3D11ClassInstance **  m_DSClassInstances;
    UINT                    m_DSNumClassInstances;
    ID3D11GeometryShader *  m_GSShader;
    ID3D11ClassInstance **  m_GSClassInstances;
    UINT                    m_GSNumClassInstances;
    ID3D11HullShader *      m_HSShader;
    ID3D11ClassInstance **  m_HSClassInstances;
    UINT                    m_HSNumClassInstances;
    ID3D11PixelShader *     m_PSShader;
    ID3D11ClassInstance **  m_PSClassInstances;
    UINT                    m_PSNumClassInstances;
    ID3D11Buffer *          m_PSConstantBuffer; // backup the first constant buffer only
    ID3D11SamplerState *    m_PSSampler; // backup the first sampler only
    ID3D11ShaderResourceView*m_PSShaderResourceView; // backup the first shader resource only
    ID3D11VertexShader *    m_VSShader;
    ID3D11ClassInstance **  m_VSClassInstances;
    UINT                    m_VSNumClassInstances;
    ID3D11Buffer *          m_VSConstantBuffer; // backup the first constant buffer only

    ID3D11Buffer *          m_IAIndexBuffer;
    DXGI_FORMAT             m_IAIndexBufferFormat;
    UINT                    m_IAIndexBufferOffset;
    ID3D11InputLayout *     m_IAInputLayout;
    D3D11_PRIMITIVE_TOPOLOGY m_IATopology;
    ID3D11Buffer *          m_IAVertexBuffer; // backup the first buffer only
    UINT                    m_IAVertexBufferStride;
    UINT                    m_IAVertexBufferOffset;

    ID3D11BlendState *      m_OMBlendState;
    FLOAT                   m_OMBlendFactor[4];
    UINT                    m_OMSampleMask;
    ID3D11DepthStencilState*m_OMDepthStencilState;
    UINT                    m_OMStencilRef;

    UINT                    m_RSScissorNumRects;
    D3D11_RECT              m_RSScissorRects[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE];
    ID3D11RasterizerState * m_RSRasterizerState;
    UINT                    m_RSNumViewports;
    D3D11_VIEWPORT          m_RSViewports[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE];

    void                    Save();
    void                    Restore();
    void                    Release();
                            CState11(ID3D11Device *_Dev, ID3D11DeviceContext *_ImmCtx);
                            ~CState11();
private:
    ID3D11Device *          m_D3DDev;
    ID3D11DeviceContext *   m_D3DDevImmContext;
};

CState11::CState11(ID3D11Device *_Dev, ID3D11DeviceContext *_ImmCtx)
{
    ZeroMemory(this, sizeof(CState11));
    m_D3DDev = _Dev;
    m_D3DDevImmContext = _ImmCtx;
}

CState11::~CState11()
{
    Release();
    m_D3DDev = NULL;
    m_D3DDevImmContext = NULL;
}

void CState11::Save()
{
    // Release previous state if needed
    Release();

    // Save shaders.
    // Not sure how xxGetShader works, D3D11 doc is evasive... Attempt:
    // First call GetShader with NULL ClassInstances to get the number of class instances.
    // Second, if not zero allocate an array of class instances and call GetShader again
    // with this array ptr to get the class instances and release the shader since its
    // ref count has been incremented a second time.

    m_CSShader = NULL;
    m_CSClassInstances = NULL;
    m_CSNumClassInstances = 0;
    m_D3DDevImmContext->CSGetShader(&m_CSShader, NULL, &m_CSNumClassInstances);
    if (m_CSNumClassInstances > 0) 
    {
        m_CSClassInstances = new ID3D11ClassInstance*[m_CSNumClassInstances];
        for (UINT i = 0; i < m_CSNumClassInstances; i++)
            m_CSClassInstances[i] = NULL;
        m_D3DDevImmContext->CSGetShader(&m_CSShader, m_CSClassInstances, &m_CSNumClassInstances);
        if (m_CSShader != NULL) 
            m_CSShader->Release();
    }

    m_DSShader = NULL;
    m_DSClassInstances = NULL;
    m_DSNumClassInstances = 0;
    m_D3DDevImmContext->DSGetShader(&m_DSShader, NULL, &m_DSNumClassInstances);
    if (m_DSNumClassInstances > 0) 
    {
        m_DSClassInstances = new ID3D11ClassInstance*[m_DSNumClassInstances];
        for (UINT i = 0; i < m_DSNumClassInstances; i++)
            m_DSClassInstances[i] = NULL;
        m_D3DDevImmContext->DSGetShader(&m_DSShader, m_DSClassInstances, &m_DSNumClassInstances);
        if (m_DSShader != NULL) 
            m_DSShader->Release();
    }

    m_GSShader = NULL;
    m_GSClassInstances = NULL;
    m_GSNumClassInstances = 0;
    m_D3DDevImmContext->GSGetShader(&m_GSShader, NULL, &m_GSNumClassInstances);
    if (m_GSNumClassInstances > 0) 
    {
        m_GSClassInstances = new ID3D11ClassInstance*[m_GSNumClassInstances];
        for (UINT i = 0; i < m_GSNumClassInstances; i++)
            m_GSClassInstances[i] = NULL;
        m_D3DDevImmContext->GSGetShader(&m_GSShader, m_GSClassInstances, &m_GSNumClassInstances);
        if (m_GSShader != NULL) 
            m_GSShader->Release();
    }

    m_HSShader = NULL;
    m_HSClassInstances = NULL;
    m_HSNumClassInstances = 0;
    m_D3DDevImmContext->HSGetShader(&m_HSShader, NULL, &m_HSNumClassInstances);
    if (m_HSNumClassInstances > 0) 
    {
        m_HSClassInstances = new ID3D11ClassInstance*[m_HSNumClassInstances];
        for (UINT i = 0; i < m_HSNumClassInstances; i++)
            m_HSClassInstances[i] = NULL;
        m_D3DDevImmContext->HSGetShader(&m_HSShader, m_HSClassInstances, &m_HSNumClassInstances);
        if (m_HSShader != NULL) 
            m_HSShader->Release();
    }

    m_PSShader = NULL;
    m_PSClassInstances = NULL;
    m_PSNumClassInstances = 0;
    m_D3DDevImmContext->PSGetShader(&m_PSShader, NULL, &m_PSNumClassInstances);
    if (m_PSNumClassInstances > 0) 
    {
        m_PSClassInstances = new ID3D11ClassInstance*[m_PSNumClassInstances];
        for (UINT i = 0; i < m_PSNumClassInstances; i++)
            m_PSClassInstances[i] = NULL;
        m_D3DDevImmContext->PSGetShader(&m_PSShader, m_PSClassInstances, &m_PSNumClassInstances);
        if (m_PSShader != NULL) 
            m_PSShader->Release();
    }
    m_D3DDevImmContext->PSGetConstantBuffers(0, 1, &m_PSConstantBuffer);
    m_D3DDevImmContext->PSGetSamplers(0, 1, &m_PSSampler);
    m_D3DDevImmContext->PSGetShaderResources(0, 1, &m_PSShaderResourceView);

    m_VSShader = NULL;
    m_VSClassInstances = NULL;
    m_VSNumClassInstances = 0;
    m_D3DDevImmContext->VSGetShader(&m_VSShader, NULL, &m_VSNumClassInstances);
    if (m_VSNumClassInstances > 0) 
    {
        m_VSClassInstances = new ID3D11ClassInstance*[m_VSNumClassInstances];
        for (UINT i = 0; i < m_VSNumClassInstances; i++)
            m_VSClassInstances[i] = NULL;
        m_D3DDevImmContext->VSGetShader(&m_VSShader, m_VSClassInstances, &m_VSNumClassInstances);
        if (m_VSShader != NULL) 
            m_VSShader->Release();
    }
    m_D3DDevImmContext->VSGetConstantBuffers(0, 1, &m_VSConstantBuffer);

    // Save Input-Assembler states
    m_D3DDevImmContext->IAGetIndexBuffer(&m_IAIndexBuffer, &m_IAIndexBufferFormat, &m_IAIndexBufferOffset);
    m_D3DDevImmContext->IAGetInputLayout(&m_IAInputLayout);
    m_D3DDevImmContext->IAGetPrimitiveTopology(&m_IATopology);
    m_D3DDevImmContext->IAGetVertexBuffers(0, 1, &m_IAVertexBuffer, &m_IAVertexBufferStride, &m_IAVertexBufferOffset);

    // Save Ouput-Merger states
    m_D3DDevImmContext->OMGetBlendState(&m_OMBlendState, m_OMBlendFactor, &m_OMSampleMask);
    m_D3DDevImmContext->OMGetDepthStencilState(&m_OMDepthStencilState, &m_OMStencilRef);

    // Save Rasterizer states
    m_D3DDevImmContext->RSGetScissorRects(&m_RSScissorNumRects, NULL);
    if (m_RSScissorNumRects > 0)
        m_D3DDevImmContext->RSGetScissorRects(&m_RSScissorNumRects, m_RSScissorRects);
    m_D3DDevImmContext->RSGetViewports(&m_RSNumViewports, NULL);
    if (m_RSNumViewports > 0)
        m_D3DDevImmContext->RSGetViewports(&m_RSNumViewports, m_RSViewports);
    m_D3DDevImmContext->RSGetState(&m_RSRasterizerState);
}

void CState11::Restore()
{
    // Restore shaders
    m_D3DDevImmContext->CSSetShader(m_CSShader, m_CSClassInstances, m_CSNumClassInstances);
    m_D3DDevImmContext->DSSetShader(m_DSShader, m_DSClassInstances, m_DSNumClassInstances);
    m_D3DDevImmContext->GSSetShader(m_GSShader, m_GSClassInstances, m_GSNumClassInstances);
    m_D3DDevImmContext->HSSetShader(m_HSShader, m_HSClassInstances, m_HSNumClassInstances);
    m_D3DDevImmContext->PSSetShader(m_PSShader, m_PSClassInstances, m_PSNumClassInstances);
    m_D3DDevImmContext->PSSetConstantBuffers(0, 1, &m_PSConstantBuffer);
    m_D3DDevImmContext->PSSetSamplers(0, 1, &m_PSSampler);
    m_D3DDevImmContext->PSSetShaderResources(0, 1, &m_PSShaderResourceView);
    m_D3DDevImmContext->VSSetShader(m_VSShader, m_VSClassInstances, m_VSNumClassInstances);
    m_D3DDevImmContext->VSSetConstantBuffers(0, 1, &m_VSConstantBuffer);

    // Restore Input-Assembler
    m_D3DDevImmContext->IASetIndexBuffer(m_IAIndexBuffer, m_IAIndexBufferFormat, m_IAIndexBufferOffset);
    m_D3DDevImmContext->IASetInputLayout(m_IAInputLayout);
    m_D3DDevImmContext->IASetPrimitiveTopology(m_IATopology);
    m_D3DDevImmContext->IASetVertexBuffers(0, 1, &m_IAVertexBuffer, &m_IAVertexBufferStride, &m_IAVertexBufferOffset);

    // Restore Ouput-Merger
    m_D3DDevImmContext->OMSetBlendState(m_OMBlendState, m_OMBlendFactor, m_OMSampleMask);
    m_D3DDevImmContext->OMSetDepthStencilState(m_OMDepthStencilState, m_OMStencilRef);

    // Restore Rasterizer states
    m_D3DDevImmContext->RSSetScissorRects(m_RSScissorNumRects, m_RSScissorRects);
    m_D3DDevImmContext->RSSetViewports(m_RSNumViewports, m_RSViewports);
    m_D3DDevImmContext->RSSetState(m_RSRasterizerState);
}

void CState11::Release()
{
    // Release stored shaders

    if (m_CSClassInstances != NULL) 
    {
        for (UINT i = 0; i < m_CSNumClassInstances; i++)
            if (m_CSClassInstances[i] != NULL) 
                m_CSClassInstances[i]->Release();
        delete[] m_CSClassInstances;
        m_CSClassInstances = NULL;
        m_CSNumClassInstances = 0;
    }
    if (m_CSShader != NULL)
    {
        m_CSShader->Release();
        m_CSShader = NULL;
    }

    if (m_DSClassInstances != NULL) 
    {
        for (UINT i = 0; i < m_DSNumClassInstances; i++)
            if (m_DSClassInstances[i] != NULL) 
                m_DSClassInstances[i]->Release();
        delete[] m_DSClassInstances;
        m_DSClassInstances = NULL;
        m_DSNumClassInstances = 0;
    }
    if (m_DSShader != NULL)
    {
        m_DSShader->Release();
        m_DSShader = NULL;
    }

    if (m_GSClassInstances != NULL) 
    {
        for (UINT i = 0; i < m_GSNumClassInstances; i++)
            if (m_GSClassInstances[i] != NULL) 
                m_GSClassInstances[i]->Release();
        delete[] m_GSClassInstances;
        m_GSClassInstances = NULL;
        m_GSNumClassInstances = 0;
    }
    if (m_GSShader != NULL)
    {
        m_GSShader->Release();
        m_GSShader = NULL;
    }

    if (m_HSClassInstances != NULL) 
    {
        for (UINT i = 0; i < m_HSNumClassInstances; i++)
            if (m_HSClassInstances[i] != NULL) 
                m_HSClassInstances[i]->Release();
        delete[] m_HSClassInstances;
        m_HSClassInstances = NULL;
        m_HSNumClassInstances = 0;
    }
    if (m_HSShader != NULL)
    {
        m_HSShader->Release();
        m_HSShader = NULL;
    }

    if (m_PSClassInstances != NULL) 
    {
        for (UINT i = 0; i < m_PSNumClassInstances; i++)
            if (m_PSClassInstances[i] != NULL) 
                m_PSClassInstances[i]->Release();
        delete[] m_PSClassInstances;
        m_PSClassInstances = NULL;
        m_PSNumClassInstances = 0;
    }
    if (m_PSShader != NULL)
    {
        m_PSShader->Release();
        m_PSShader = NULL;
    }
    if (m_PSConstantBuffer != NULL)
    {
        m_PSConstantBuffer->Release();
        m_PSConstantBuffer = NULL;
    }
    if (m_PSSampler != NULL)
    {
        m_PSSampler->Release();
        m_PSSampler = NULL;
    }
    if (m_PSShaderResourceView != NULL)
    {
        m_PSShaderResourceView->Release();
        m_PSShaderResourceView = NULL;
    }

    if (m_VSClassInstances != NULL) 
    {
        for (UINT i = 0; i < m_VSNumClassInstances; i++)
            if (m_VSClassInstances[i] != NULL) 
                m_VSClassInstances[i]->Release();
        delete[] m_VSClassInstances;
        m_VSClassInstances = NULL;
        m_VSNumClassInstances = 0;
    }
    if (m_VSShader != NULL)
    {
        m_VSShader->Release();
        m_VSShader = NULL;
    }
    if (m_VSConstantBuffer != NULL)
    {
        m_VSConstantBuffer->Release();
        m_VSConstantBuffer = NULL;
    }

    // Release Input-Assembler states
    if (m_IAIndexBuffer != NULL) 
    {
        m_IAIndexBuffer->Release();
        m_IAIndexBuffer = NULL;
    }
    if (m_IAInputLayout != NULL)
    {
        m_IAInputLayout->Release();
        m_IAInputLayout = 0;
    }
    if (m_IAVertexBuffer != NULL)
    {
        m_IAVertexBuffer->Release();
        m_IAVertexBuffer = NULL;
    }

    // Release Output-Merger states
    if (m_OMBlendState != NULL) 
    {
        m_OMBlendState->Release();
        m_OMBlendState = NULL;
    }
    if (m_OMDepthStencilState != NULL) 
    {
        m_OMDepthStencilState->Release();
        m_OMDepthStencilState = NULL;
    }

    // Release Rasterizer state
    if (m_RSRasterizerState != 0) 
    {
        m_RSRasterizerState->Release();
        m_RSRasterizerState = NULL;
    }
    m_RSNumViewports = 0;
    m_RSScissorNumRects = 0;
}

//  ---------------------------------------------------------------------------

int CTwGraphDirect3D11::Init()
{
    assert(g_TwMgr!=NULL);
    assert(g_TwMgr->m_Device!=NULL);

    m_D3DDev = static_cast<ID3D11Device *>(g_TwMgr->m_Device);
    m_D3DDevInitialRefCount = m_D3DDev->AddRef() - 1;
    m_D3DDev->GetImmediateContext(&m_D3DDevImmContext);

    m_Drawing = false;
    m_OffsetX = m_OffsetY = 0;
    m_ViewportInit = new D3D11_VIEWPORT;
    m_FontTex = NULL;
    m_FontD3DTex = NULL;
    m_FontD3DTexRV = NULL;
    m_WndWidth = 0;
    m_WndHeight = 0;
    m_State = NULL;
    m_DepthStencilState = NULL;
    m_BlendState = NULL;
    m_RasterState = NULL;
    m_RasterStateAntialiased = NULL;
    m_RasterStateMultisample = NULL;
    m_RasterStateCullCW = NULL;
    m_RasterStateCullCCW = NULL;
    m_LineRectVS = NULL;
    m_LineRectCstColorVS = NULL;
    m_LineRectPS = NULL;
    m_LineRectVertexLayout = NULL;
    m_TextVS = NULL;
    m_TextCstColorVS = NULL;
    m_TextPS = NULL;
    m_TextVertexLayout = NULL;
    m_LineVertexBuffer = NULL;
    m_RectVertexBuffer = NULL;
    m_TrianglesVertexBuffer = NULL;
    m_TrianglesVertexBufferCount = 0;
    m_ConstantBuffer = NULL;
    m_SamplerState = NULL;

    // Allocate state object
    m_State = new CState11(m_D3DDev, m_D3DDevImmContext);

    // Disable client shaders
    m_D3DDevImmContext->CSSetShader(NULL, NULL, 0);
    m_D3DDevImmContext->DSSetShader(NULL, NULL, 0);
    m_D3DDevImmContext->GSSetShader(NULL, NULL, 0);
    m_D3DDevImmContext->HSSetShader(NULL, NULL, 0);
    m_D3DDevImmContext->PSSetShader(NULL, NULL, 0);
    m_D3DDevImmContext->VSSetShader(NULL, NULL, 0);

    // Create shaders
    HRESULT hr = m_D3DDev->CreateVertexShader(g_LineRectVS, sizeof(g_LineRectVS), NULL, &m_LineRectVS);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateVS11);
        Shut();
        return 0;
    }
    hr = m_D3DDev->CreateVertexShader(g_LineRectCstColorVS, sizeof(g_LineRectCstColorVS), NULL, &m_LineRectCstColorVS);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateVS11);
        Shut();
        return 0;
    }
    hr = m_D3DDev->CreatePixelShader(g_LineRectPS, sizeof(g_LineRectPS), NULL, &m_LineRectPS);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreatePS11);
        Shut();
        return 0;
    }
    hr = m_D3DDev->CreateVertexShader(g_TextVS, sizeof(g_TextVS), NULL, &m_TextVS);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateVS11);
        Shut();
        return 0;
    }
    hr = m_D3DDev->CreateVertexShader(g_TextCstColorVS, sizeof(g_TextCstColorVS), NULL, &m_TextCstColorVS);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateVS11);
        Shut();
        return 0;
    }
    hr = m_D3DDev->CreatePixelShader(g_TextPS, sizeof(g_TextPS), NULL, &m_TextPS);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreatePS11);
        Shut();
        return 0;
    }
 
    // Create input layout for lines & rect
    D3D11_INPUT_ELEMENT_DESC lineRectLayout[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },  
        { "COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, offsetof(CLineRectVtx, m_Color), D3D11_INPUT_PER_VERTEX_DATA, 0 }
    };
    hr = m_D3DDev->CreateInputLayout(lineRectLayout, sizeof(lineRectLayout)/sizeof(lineRectLayout[0]), g_LineRectVS, sizeof(g_LineRectVS), &m_LineRectVertexLayout);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateLayout11);
        Shut();
        return 0;
    }

    // Create line vertex buffer
    D3D11_BUFFER_DESC bd;
    bd.Usage = D3D11_USAGE_DYNAMIC;
    bd.ByteWidth = 2 * sizeof(CLineRectVtx);
    bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
    bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    bd.MiscFlags = 0;
    bd.StructureByteStride = 0;
    hr = m_D3DDev->CreateBuffer(&bd, NULL, &m_LineVertexBuffer);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateBuffer11);
        Shut();
        return 0;
    }

    // Create rect vertex buffer
    bd.ByteWidth = 4 * sizeof(CLineRectVtx);
    hr = m_D3DDev->CreateBuffer(&bd, NULL, &m_RectVertexBuffer);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateBuffer11);
        Shut();
        return 0;
    }

    // Create constant buffer
    bd.ByteWidth = sizeof(CConstants);
    bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    hr = m_D3DDev->CreateBuffer(&bd, NULL, &m_ConstantBuffer);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateBuffer11);
        Shut();
        return 0;
    }

    // Create sampler
    D3D11_SAMPLER_DESC sd;
    sd.AddressU = sd.AddressV = sd.AddressW = D3D11_TEXTURE_ADDRESS_BORDER;
    sd.BorderColor[0] = sd.BorderColor[1] = sd.BorderColor[2] = sd.BorderColor[3] = 0;
    sd.ComparisonFunc = D3D11_COMPARISON_NEVER;
    sd.Filter = D3D11_FILTER_MIN_MAG_MIP_POINT;
    sd.MaxAnisotropy = 1;
    sd.MaxLOD = sd.MinLOD = 0;
    sd.MipLODBias = 0;
    hr = m_D3DDev->CreateSamplerState(&sd, &m_SamplerState);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateSampler11);
        Shut();
        return 0;
    }

    // Create input layout for text
    D3D11_INPUT_ELEMENT_DESC textLayout[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },  
        { "COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, offsetof(CTextVtx, m_Color), D3D11_INPUT_PER_VERTEX_DATA, 0 }, 
        { "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, offsetof(CTextVtx, m_UV), D3D11_INPUT_PER_VERTEX_DATA, 0 }
    };
    hr = m_D3DDev->CreateInputLayout(textLayout, sizeof(textLayout)/sizeof(textLayout[0]), g_TextVS, sizeof(g_TextVS), &m_TextVertexLayout);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateLayout11);
        Shut();
        return 0;
    }

    // Create depth stencil state object
    D3D11_DEPTH_STENCILOP_DESC od;
    od.StencilFunc = D3D11_COMPARISON_ALWAYS;
    od.StencilFailOp = D3D11_STENCIL_OP_KEEP;
    od.StencilPassOp = D3D11_STENCIL_OP_KEEP;
    od.StencilDepthFailOp = D3D11_STENCIL_OP_KEEP;
    D3D11_DEPTH_STENCIL_DESC dsd;
    dsd.DepthEnable = FALSE;
    dsd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
    dsd.DepthFunc = D3D11_COMPARISON_ALWAYS;
    dsd.StencilEnable = FALSE;
    dsd.StencilReadMask = D3D11_DEFAULT_STENCIL_READ_MASK;
    dsd.StencilWriteMask = D3D11_DEFAULT_STENCIL_WRITE_MASK;
    dsd.FrontFace = od;
    dsd.BackFace = od;
    m_D3DDev->CreateDepthStencilState(&dsd, &m_DepthStencilState);

    // Create blend state object
    D3D11_BLEND_DESC bsd;
    bsd.AlphaToCoverageEnable = FALSE;
    bsd.IndependentBlendEnable = FALSE;
    for(int i=0; i<8; ++i)
    {
        bsd.RenderTarget[i].BlendEnable = TRUE;
        bsd.RenderTarget[i].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
        bsd.RenderTarget[i].SrcBlend = D3D11_BLEND_SRC_ALPHA;
        bsd.RenderTarget[i].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
        bsd.RenderTarget[i].BlendOp =  D3D11_BLEND_OP_ADD;
        bsd.RenderTarget[i].SrcBlendAlpha = D3D11_BLEND_SRC_ALPHA;
        bsd.RenderTarget[i].DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
        bsd.RenderTarget[i].BlendOpAlpha = D3D11_BLEND_OP_ADD;
    }
    m_D3DDev->CreateBlendState(&bsd, &m_BlendState);

    // Create rasterizer state object
    D3D11_RASTERIZER_DESC rd;
    rd.FillMode = D3D11_FILL_SOLID;
    rd.CullMode = D3D11_CULL_NONE;
    rd.FrontCounterClockwise = true;
    rd.DepthBias = false;
    rd.DepthBiasClamp = 0;
    rd.SlopeScaledDepthBias = 0;
    rd.DepthClipEnable = false;
    rd.ScissorEnable = true;
    rd.MultisampleEnable = false; // do not allow msaa (fonts would be degraded)
    rd.AntialiasedLineEnable = false;
    m_D3DDev->CreateRasterizerState(&rd, &m_RasterState);

    rd.AntialiasedLineEnable = true;
    m_D3DDev->CreateRasterizerState(&rd, &m_RasterStateAntialiased);
    rd.AntialiasedLineEnable = false;

    // the three following raster states allow msaa
    rd.MultisampleEnable = true;
    m_D3DDev->CreateRasterizerState(&rd, &m_RasterStateMultisample);

    rd.CullMode = D3D11_CULL_BACK;
    m_D3DDev->CreateRasterizerState(&rd, &m_RasterStateCullCW);

    rd.CullMode = D3D11_CULL_FRONT;
    m_D3DDev->CreateRasterizerState(&rd, &m_RasterStateCullCCW);

    return 1;
}

//  ---------------------------------------------------------------------------

int CTwGraphDirect3D11::Shut()
{
    assert(m_Drawing==false);

    UnbindFont(m_D3DDev, m_FontD3DTex, m_FontD3DTexRV);
    m_FontD3DTex = NULL;
    m_FontD3DTexRV = NULL;
    if( m_State )
    {
        delete m_State;
        m_State = NULL;
    }
    if( m_ViewportInit )
    {
        delete m_ViewportInit;
        m_ViewportInit = NULL;
    }

    if( m_DepthStencilState )
    {
        ULONG rc = m_DepthStencilState->Release();
        //assert( rc==0 ); // no assert: the client can use a similar (then shared) state
        (void)rc;
        m_DepthStencilState = NULL;
    }
    if( m_BlendState )
    {
        ULONG rc = m_BlendState->Release();
        //assert( rc==0 ); // no assert: the client can use a similar (then shared) state
        (void)rc;
        m_BlendState = NULL;
    }
    if( m_RasterState )
    {
        ULONG rc = m_RasterState->Release();
        //assert( rc==0 ); // no assert: the client can use a similar (then shared) state
        (void)rc;
        m_RasterState = NULL;
    }
    if( m_RasterStateAntialiased )
    {
        ULONG rc = m_RasterStateAntialiased->Release();
        //assert( rc==0 ); // no assert: the client can use a similar (then shared) state
        (void)rc;
        m_RasterStateAntialiased = NULL;
    }
    if( m_RasterStateMultisample )
    {
        ULONG rc = m_RasterStateMultisample->Release();
        //assert( rc==0 ); // no assert: the client can use a similar (then shared) state
        (void)rc;
        m_RasterStateMultisample = NULL;
    }
    if( m_RasterStateCullCW )
    {
        ULONG rc = m_RasterStateCullCW->Release();
        //assert( rc==0 ); // no assert: the client can use a similar (then shared) state
        (void)rc;
        m_RasterStateCullCW = NULL;
    }
    if( m_RasterStateCullCCW )
    {
        ULONG rc = m_RasterStateCullCCW->Release();
        //assert( rc==0 ); // no assert: the client can use a similar (then shared) state
        (void)rc;
        m_RasterStateCullCCW = NULL;
    }
    if( m_SamplerState )
    {
        ULONG rc = m_SamplerState->Release();
        //assert( rc==0 ); // no assert: the client can use a similar (then shared) state
        (void)rc;
        m_SamplerState = NULL;
    }

    if( m_LineRectVS )
    {
        ULONG rc = m_LineRectVS->Release();
        assert( rc==0 ); (void)rc;
        m_LineRectVS = NULL;
    }
    if( m_LineRectCstColorVS )
    {
        ULONG rc = m_LineRectCstColorVS->Release();
        assert( rc==0 ); (void)rc;
        m_LineRectCstColorVS = NULL;
    }
    if( m_LineRectPS )
    {
        ULONG rc = m_LineRectPS->Release();
        assert( rc==0 ); (void)rc;
        m_LineRectPS = NULL;
    }
    if( m_TextVS )
    {
        ULONG rc = m_TextVS->Release();
        assert( rc==0 ); (void)rc;
        m_TextVS = NULL;
    }
    if( m_TextCstColorVS )
    {
        ULONG rc = m_TextCstColorVS->Release();
        assert( rc==0 ); (void)rc;
        m_TextCstColorVS = NULL;
    }
    if( m_TextPS )
    {
        ULONG rc = m_TextPS->Release();
        assert( rc==0 ); (void)rc;
        m_TextPS = NULL;
    }
    if( m_LineVertexBuffer )
    {
        ULONG rc = m_LineVertexBuffer->Release();
        assert( rc==0 ); (void)rc;
        m_LineVertexBuffer = NULL;
    }
    if( m_RectVertexBuffer )
    {
        ULONG rc = m_RectVertexBuffer->Release();
        assert( rc==0 ); (void)rc;
        m_RectVertexBuffer = NULL;
    }
    if( m_TrianglesVertexBuffer )
    {
        ULONG rc = m_TrianglesVertexBuffer->Release();
        assert( rc==0 ); (void)rc;
        m_TrianglesVertexBuffer = NULL;
        m_TrianglesVertexBufferCount = 0;
    }
    if( m_ConstantBuffer )
    {
        ULONG rc = m_ConstantBuffer->Release();
        assert( rc==0 ); (void)rc;
        m_ConstantBuffer = NULL;
    }
    if( m_LineRectVertexLayout ) 
    {
        ULONG rc = m_LineRectVertexLayout->Release();
        assert( rc==0 ); (void)rc;
        m_LineRectVertexLayout = NULL;
    }
    if( m_TextVertexLayout ) 
    {
        ULONG rc = m_TextVertexLayout->Release();
        assert( rc==0 ); (void)rc;
        m_TextVertexLayout = NULL;
    }

    if( m_D3DDevImmContext )
    {
        m_D3DDevImmContext->Release();
        m_D3DDevImmContext = NULL;
    }

    if( m_D3DDev )
    {
        //unsigned int rc = m_D3DDev->Release();
        //assert( m_D3DDevInitialRefCount==rc ); (void)rc;
        m_D3DDev->Release();
        m_D3DDev = NULL;
    }

    return 1;
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D11::BeginDraw(int _WndWidth, int _WndHeight)
{
    assert(m_Drawing==false && _WndWidth>0 && _WndHeight>0);
    m_Drawing = true;

    m_WndWidth  = _WndWidth;
    m_WndHeight = _WndHeight;
    m_OffsetX = m_OffsetY = 0;

    // save client context state
    m_State->Save();

    // Setup the viewport
    D3D11_VIEWPORT vp;
    vp.Width = (FLOAT)_WndWidth;
    vp.Height = (FLOAT)_WndHeight;
    vp.MinDepth = 0.0f;
    vp.MaxDepth = 1.0f;
    vp.TopLeftX = 0;
    vp.TopLeftY = 0;
    m_D3DDevImmContext->RSSetViewports(1, &vp);
    *static_cast<D3D11_VIEWPORT *>(m_ViewportInit) = vp;

    m_ViewportAndScissorRects[0] = FullRect;
    m_ViewportAndScissorRects[1] = FullRect;
    m_D3DDevImmContext->RSSetScissorRects(1, m_ViewportAndScissorRects);

    m_D3DDevImmContext->RSSetState(m_RasterState);

    m_D3DDevImmContext->OMSetDepthStencilState(m_DepthStencilState, 0);
    float blendFactors[4] = { 1, 1, 1, 1 };
    m_D3DDevImmContext->OMSetBlendState(m_BlendState, blendFactors, 0xffffffff);

    m_D3DDevImmContext->CSSetShader(NULL, NULL, 0);
    m_D3DDevImmContext->DSSetShader(NULL, NULL, 0);
    m_D3DDevImmContext->GSSetShader(NULL, NULL, 0);
    m_D3DDevImmContext->HSSetShader(NULL, NULL, 0);
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D11::EndDraw()
{
    m_D3DDevImmContext->RSSetState(NULL);
    m_D3DDevImmContext->OMSetDepthStencilState(NULL, 0);
    m_D3DDevImmContext->OMSetBlendState(NULL, NULL, 0xffffffff);

    assert(m_Drawing==true);
    m_Drawing = false;

    // restore and release client context state
    m_State->Restore();
    m_State->Release();
}

//  ---------------------------------------------------------------------------

bool CTwGraphDirect3D11::IsDrawing()
{
    return m_Drawing;
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D11::Restore()
{
    if( m_State )
        m_State->Release();

    UnbindFont(m_D3DDev, m_FontD3DTex, m_FontD3DTexRV);
    m_FontD3DTexRV = NULL;
    m_FontD3DTex = NULL;
    
    m_FontTex = NULL;
}

//  ---------------------------------------------------------------------------

static inline float ToNormScreenX(int x, int wndWidth)
{
    return 2.0f*((float)x-0.5f)/wndWidth - 1.0f;
}

static inline float ToNormScreenY(int y, int wndHeight)
{
    return 1.0f - 2.0f*((float)y-0.5f)/wndHeight;
}

static inline color32 ToR8G8B8A8(color32 col)
{
    return (col & 0xff00ff00) | ((col>>16) & 0xff) | ((col<<16) & 0xff0000);
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D11::DrawLine(int _X0, int _Y0, int _X1, int _Y1, color32 _Color0, color32 _Color1, bool _AntiAliased)
{
    assert(m_Drawing==true);

    float x0 = ToNormScreenX(_X0 + m_OffsetX, m_WndWidth);
    float y0 = ToNormScreenY(_Y0 + m_OffsetY, m_WndHeight);
    float x1 = ToNormScreenX(_X1 + m_OffsetX, m_WndWidth);
    float y1 = ToNormScreenY(_Y1 + m_OffsetY, m_WndHeight);
 
    D3D11_MAPPED_SUBRESOURCE mappedResource;
    HRESULT hr = m_D3DDevImmContext->Map(m_LineVertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
    if( SUCCEEDED(hr) )
    {
        CLineRectVtx *vertices = (CLineRectVtx *)mappedResource.pData;
        // Fill vertex buffer
        vertices[0].m_Pos[0] = x0;
        vertices[0].m_Pos[1] = y0;
        vertices[0].m_Pos[2] = 0;
        vertices[0].m_Color = ToR8G8B8A8(_Color0);
        vertices[1].m_Pos[0] = x1;
        vertices[1].m_Pos[1] = y1;
        vertices[1].m_Pos[2] = 0;
        vertices[1].m_Color = ToR8G8B8A8(_Color1);

        m_D3DDevImmContext->Unmap(m_LineVertexBuffer, 0);

        if( _AntiAliased )
            m_D3DDevImmContext->RSSetState(m_RasterStateAntialiased);

        // Reset shader constants
        hr = m_D3DDevImmContext->Map(m_ConstantBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
        if( SUCCEEDED(hr) )
        {
            CConstants *constants = (CConstants *)mappedResource.pData;
            constants->m_Offset[0] = 0;
            constants->m_Offset[1] = 0;
            constants->m_Offset[2] = 0;
            constants->m_Offset[3] = 0;
            constants->m_CstColor[0] = 1;
            constants->m_CstColor[1] = 1;
            constants->m_CstColor[2] = 1;
            constants->m_CstColor[3] = 1;

            m_D3DDevImmContext->Unmap(m_ConstantBuffer, 0);
        }

        // Set the input layout
        m_D3DDevImmContext->IASetInputLayout(m_LineRectVertexLayout);

        // Set vertex buffer
        UINT stride = sizeof(CLineRectVtx);
        UINT offset = 0;
        m_D3DDevImmContext->IASetVertexBuffers(0, 1, &m_LineVertexBuffer, &stride, &offset);

        // Set primitive topology
        m_D3DDevImmContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_LINELIST);

        // Render the line
        m_D3DDevImmContext->VSSetConstantBuffers(0, 1, &m_ConstantBuffer);
        m_D3DDevImmContext->VSSetShader(m_LineRectVS, NULL, 0);
        m_D3DDevImmContext->PSSetShader(m_LineRectPS, NULL, 0);
        m_D3DDevImmContext->Draw(2, 0);

        if( _AntiAliased )
            m_D3DDevImmContext->RSSetState(m_RasterState); // restore default raster state
    }
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D11::DrawRect(int _X0, int _Y0, int _X1, int _Y1, color32 _Color00, color32 _Color10, color32 _Color01, color32 _Color11)
{
    assert(m_Drawing==true);

    // border adjustment
    if(_X0<_X1)
        ++_X1;
    else if(_X0>_X1)
        ++_X0;
    if(_Y0<_Y1)
        ++_Y1;
    else if(_Y0>_Y1)
        ++_Y0;

    float x0 = ToNormScreenX(_X0 + m_OffsetX, m_WndWidth);
    float y0 = ToNormScreenY(_Y0 + m_OffsetY, m_WndHeight);
    float x1 = ToNormScreenX(_X1 + m_OffsetX, m_WndWidth);
    float y1 = ToNormScreenY(_Y1 + m_OffsetY, m_WndHeight);
 
    D3D11_MAPPED_SUBRESOURCE mappedResource;
    HRESULT hr = m_D3DDevImmContext->Map(m_RectVertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
    if( SUCCEEDED(hr) )
    {
        CLineRectVtx *vertices = (CLineRectVtx *)mappedResource.pData;
        // Fill vertex buffer
        vertices[0].m_Pos[0] = x0;
        vertices[0].m_Pos[1] = y0;
        vertices[0].m_Pos[2] = 0;
        vertices[0].m_Color = ToR8G8B8A8(_Color00);
        vertices[1].m_Pos[0] = x1;
        vertices[1].m_Pos[1] = y0;
        vertices[1].m_Pos[2] = 0;
        vertices[1].m_Color = ToR8G8B8A8(_Color10);
        vertices[2].m_Pos[0] = x0;
        vertices[2].m_Pos[1] = y1;
        vertices[2].m_Pos[2] = 0;
        vertices[2].m_Color = ToR8G8B8A8(_Color01);
        vertices[3].m_Pos[0] = x1;
        vertices[3].m_Pos[1] = y1;
        vertices[3].m_Pos[2] = 0;
        vertices[3].m_Color = ToR8G8B8A8(_Color11);

        m_D3DDevImmContext->Unmap(m_RectVertexBuffer, 0);

        // Reset shader constants
        hr = m_D3DDevImmContext->Map(m_ConstantBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
        if( SUCCEEDED(hr) )
        {
            CConstants *constants = (CConstants *)mappedResource.pData;
            constants->m_Offset[0] = 0;
            constants->m_Offset[1] = 0;
            constants->m_Offset[2] = 0;
            constants->m_Offset[3] = 0;
            constants->m_CstColor[0] = 1;
            constants->m_CstColor[1] = 1;
            constants->m_CstColor[2] = 1;
            constants->m_CstColor[3] = 1;

            m_D3DDevImmContext->Unmap(m_ConstantBuffer, 0);
        }

        // Set the input layout
        m_D3DDevImmContext->IASetInputLayout(m_LineRectVertexLayout);

        // Set vertex buffer
        UINT stride = sizeof(CLineRectVtx);
        UINT offset = 0;
        m_D3DDevImmContext->IASetVertexBuffers(0, 1, &m_RectVertexBuffer, &stride, &offset);

        // Set primitive topology
        m_D3DDevImmContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);

        // Render the rect
        m_D3DDevImmContext->VSSetConstantBuffers(0, 1, &m_ConstantBuffer);
        m_D3DDevImmContext->VSSetShader(m_LineRectVS, NULL, 0);
        m_D3DDevImmContext->PSSetShader(m_LineRectPS, NULL, 0);
        m_D3DDevImmContext->Draw(4, 0);
    }
}

//  ---------------------------------------------------------------------------

void *CTwGraphDirect3D11::NewTextObj()
{
    CTextObj *textObj = new CTextObj;
    memset(textObj, 0, sizeof(CTextObj));
    return textObj;
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D11::DeleteTextObj(void *_TextObj)
{
    assert(_TextObj!=NULL);
    CTextObj *textObj = static_cast<CTextObj *>(_TextObj);
    if( textObj->m_TextVertexBuffer )
        textObj->m_TextVertexBuffer->Release();
    if( textObj->m_BgVertexBuffer )
        textObj->m_BgVertexBuffer->Release();
    memset(textObj, 0, sizeof(CTextObj));
    delete textObj;
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D11::BuildText(void *_TextObj, const std::string *_TextLines, color32 *_LineColors, color32 *_LineBgColors, int _NbLines, const CTexFont *_Font, int _Sep, int _BgWidth)
{
    assert(m_Drawing==true);
    assert(_TextObj!=NULL);
    assert(_Font!=NULL);

    if( _Font != m_FontTex )
    {
        UnbindFont(m_D3DDev, m_FontD3DTex, m_FontD3DTexRV);
        BindFont(m_D3DDev, _Font, &m_FontD3DTex, &m_FontD3DTexRV);
        m_FontTex = _Font;
    }

    int nbTextVerts = 0;
    int line;
    for( line=0; line<_NbLines; ++line )
        nbTextVerts += 6 * (int)_TextLines[line].length();
    int nbBgVerts = 0;
    if( _BgWidth>0 )
        nbBgVerts = _NbLines*6;

    CTextObj *textObj = static_cast<CTextObj *>(_TextObj);
    textObj->m_LineColors = (_LineColors!=NULL);
    textObj->m_LineBgColors = (_LineBgColors!=NULL);

    // (re)create text vertex buffer if needed, and map it
    CTextVtx *textVerts = NULL;
    if( nbTextVerts>0 )
    {
        if( textObj->m_TextVertexBuffer==NULL || textObj->m_TextVertexBufferSize<nbTextVerts )
        {
            if( textObj->m_TextVertexBuffer!=NULL )
            {
                ULONG rc = textObj->m_TextVertexBuffer->Release();
                assert( rc==0 ); (void)rc;
                textObj->m_TextVertexBuffer = NULL;
            }
            textObj->m_TextVertexBufferSize = nbTextVerts + 6*256; // add a reserve of 256 characters
            D3D11_BUFFER_DESC bd;
            bd.Usage = D3D11_USAGE_DYNAMIC;
            bd.ByteWidth = textObj->m_TextVertexBufferSize * sizeof(CTextVtx);
            bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
            bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            bd.MiscFlags = 0;
            bd.StructureByteStride = 0;
            m_D3DDev->CreateBuffer(&bd, NULL, &textObj->m_TextVertexBuffer);
        }

        if( textObj->m_TextVertexBuffer!=NULL )
        {
            D3D11_MAPPED_SUBRESOURCE mappedResource;
            m_D3DDevImmContext->Map(textObj->m_TextVertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
            textVerts = (CTextVtx *)mappedResource.pData;
        }
    }

    // (re)create bg vertex buffer if needed, and map it
    CLineRectVtx *bgVerts = NULL;
    if( nbBgVerts>0 )
    {
        if( textObj->m_BgVertexBuffer==NULL || textObj->m_BgVertexBufferSize<nbBgVerts )
        {
            if( textObj->m_BgVertexBuffer!=NULL )
            {
                ULONG rc = textObj->m_BgVertexBuffer->Release();
                assert( rc==0 ); (void)rc;
                textObj->m_BgVertexBuffer = NULL;
            }
            textObj->m_BgVertexBufferSize = nbBgVerts + 6*32; // add a reserve of 32 rects
            D3D11_BUFFER_DESC bd;
            bd.Usage = D3D11_USAGE_DYNAMIC;
            bd.ByteWidth = textObj->m_BgVertexBufferSize * sizeof(CLineRectVtx);
            bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
            bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
            bd.MiscFlags = 0;
            bd.StructureByteStride = 0;
            m_D3DDev->CreateBuffer(&bd, NULL, &textObj->m_BgVertexBuffer);
        }

        if( textObj->m_BgVertexBuffer!=NULL )
        {
            D3D11_MAPPED_SUBRESOURCE mappedResource;
            m_D3DDevImmContext->Map(textObj->m_BgVertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
            bgVerts = (CLineRectVtx *)mappedResource.pData;
        }
    }

    int x, x1, y, y1, i, len;
    float px, px1, py, py1;
    unsigned char ch;
    const unsigned char *text;
    color32 lineColor = COLOR32_RED;
    CTextVtx vtx;
    vtx.m_Pos[2] = 0;
    CLineRectVtx bgVtx;
    bgVtx.m_Pos[2] = 0;
    int textVtxIndex = 0;
    int bgVtxIndex = 0;
    for( line=0; line<_NbLines; ++line )
    {
        x = 0;
        y = line * (_Font->m_CharHeight+_Sep);
        y1 = y+_Font->m_CharHeight;
        len = (int)_TextLines[line].length();
        text = (const unsigned char *)(_TextLines[line].c_str());
        if( _LineColors!=NULL )
            lineColor = ToR8G8B8A8(_LineColors[line]);

        if( textVerts!=NULL )
            for( i=0; i<len; ++i )
            {
                ch = text[i];
                x1 = x + _Font->m_CharWidth[ch];

                px  = ToNormScreenX(x,  m_WndWidth);
                py  = ToNormScreenY(y,  m_WndHeight);
                px1 = ToNormScreenX(x1, m_WndWidth);
                py1 = ToNormScreenY(y1, m_WndHeight);

                vtx.m_Color  = lineColor;

                vtx.m_Pos[0] = px;
                vtx.m_Pos[1] = py;
                vtx.m_UV [0] = _Font->m_CharU0[ch];
                vtx.m_UV [1] = _Font->m_CharV0[ch];
                textVerts[textVtxIndex++] = vtx;

                vtx.m_Pos[0] = px1;
                vtx.m_Pos[1] = py;
                vtx.m_UV [0] = _Font->m_CharU1[ch];
                vtx.m_UV [1] = _Font->m_CharV0[ch];
                textVerts[textVtxIndex++] = vtx;

                vtx.m_Pos[0] = px;
                vtx.m_Pos[1] = py1;
                vtx.m_UV [0] = _Font->m_CharU0[ch];
                vtx.m_UV [1] = _Font->m_CharV1[ch];
                textVerts[textVtxIndex++] = vtx;

                vtx.m_Pos[0] = px1;
                vtx.m_Pos[1] = py;
                vtx.m_UV [0] = _Font->m_CharU1[ch];
                vtx.m_UV [1] = _Font->m_CharV0[ch];
                textVerts[textVtxIndex++] = vtx;

                vtx.m_Pos[0] = px1;
                vtx.m_Pos[1] = py1;
                vtx.m_UV [0] = _Font->m_CharU1[ch];
                vtx.m_UV [1] = _Font->m_CharV1[ch];
                textVerts[textVtxIndex++] = vtx;

                vtx.m_Pos[0] = px;
                vtx.m_Pos[1] = py1;
                vtx.m_UV [0] = _Font->m_CharU0[ch];
                vtx.m_UV [1] = _Font->m_CharV1[ch];
                textVerts[textVtxIndex++] = vtx;

                x = x1;
            }

        if( _BgWidth>0 && bgVerts!=NULL )
        {
            if( _LineBgColors!=NULL )
                bgVtx.m_Color = ToR8G8B8A8(_LineBgColors[line]);
            else
                bgVtx.m_Color = ToR8G8B8A8(COLOR32_BLACK);

            px  = ToNormScreenX(-1, m_WndWidth);
            py  = ToNormScreenY(y,  m_WndHeight);
            px1 = ToNormScreenX(_BgWidth+1, m_WndWidth);
            py1 = ToNormScreenY(y1, m_WndHeight);

            bgVtx.m_Pos[0] = px;
            bgVtx.m_Pos[1] = py;
            bgVerts[bgVtxIndex++] = bgVtx;

            bgVtx.m_Pos[0] = px1;
            bgVtx.m_Pos[1] = py;
            bgVerts[bgVtxIndex++] = bgVtx;

            bgVtx.m_Pos[0] = px;
            bgVtx.m_Pos[1] = py1;
            bgVerts[bgVtxIndex++] = bgVtx;

            bgVtx.m_Pos[0] = px1;
            bgVtx.m_Pos[1] = py;
            bgVerts[bgVtxIndex++] = bgVtx;

            bgVtx.m_Pos[0] = px1;
            bgVtx.m_Pos[1] = py1;
            bgVerts[bgVtxIndex++] = bgVtx;

            bgVtx.m_Pos[0] = px;
            bgVtx.m_Pos[1] = py1;
            bgVerts[bgVtxIndex++] = bgVtx;
        }
    }
    assert( textVtxIndex==nbTextVerts );
    assert( bgVtxIndex==nbBgVerts );
    textObj->m_NbTextVerts = nbTextVerts;
    textObj->m_NbBgVerts = nbBgVerts;

    if( textVerts!=NULL )
        m_D3DDevImmContext->Unmap(textObj->m_TextVertexBuffer, 0);
    if( bgVerts!=NULL )
        m_D3DDevImmContext->Unmap(textObj->m_BgVertexBuffer, 0);
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D11::DrawText(void *_TextObj, int _X, int _Y, color32 _Color, color32 _BgColor)
{
    assert(m_Drawing==true);
    assert(_TextObj!=NULL);
    CTextObj *textObj = static_cast<CTextObj *>(_TextObj);
    float dx = 2.0f*(float)(_X + m_OffsetX)/m_WndWidth;
    float dy = -2.0f*(float)(_Y + m_OffsetY)/m_WndHeight;

    // Draw background
    if( textObj->m_NbBgVerts>=4 && textObj->m_BgVertexBuffer!=NULL )
    {
        // Set offset and constant color
        D3D11_MAPPED_SUBRESOURCE mappedResource;
        HRESULT hr = m_D3DDevImmContext->Map(m_ConstantBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
        if( SUCCEEDED(hr) )
        {
            CConstants *constants = (CConstants *)mappedResource.pData;
            constants->m_Offset[0] = dx;
            constants->m_Offset[1] = dy;
            constants->m_Offset[2] = 0;
            constants->m_Offset[3] = 0;
            Color32ToARGBf(_BgColor, constants->m_CstColor+3, constants->m_CstColor+0, constants->m_CstColor+1, constants->m_CstColor+2);
            m_D3DDevImmContext->Unmap(m_ConstantBuffer, 0);
        }

        // Set the input layout
        m_D3DDevImmContext->IASetInputLayout(m_LineRectVertexLayout);

        // Set vertex buffer
        UINT stride = sizeof(CLineRectVtx);
        UINT offset = 0;
        m_D3DDevImmContext->IASetVertexBuffers(0, 1, &textObj->m_BgVertexBuffer, &stride, &offset);

        // Set primitive topology
        m_D3DDevImmContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        // Render the bg rectangles
        m_D3DDevImmContext->VSSetConstantBuffers(0, 1, &m_ConstantBuffer);
        if( _BgColor!=0 || !textObj->m_LineBgColors ) // use a constant bg color
            m_D3DDevImmContext->VSSetShader(m_LineRectCstColorVS, NULL, 0);
        else
            m_D3DDevImmContext->VSSetShader(m_LineRectVS, NULL, 0);
        m_D3DDevImmContext->PSSetSamplers(0, 1, &m_SamplerState);
        m_D3DDevImmContext->PSSetShader(m_LineRectPS, NULL, 0);
        m_D3DDevImmContext->Draw(textObj->m_NbBgVerts, 0);
    }

    // Draw text
    if( textObj->m_NbTextVerts>=4 && textObj->m_TextVertexBuffer!=NULL )
    {
        // Set offset and constant color
        D3D11_MAPPED_SUBRESOURCE mappedResource;
        HRESULT hr = m_D3DDevImmContext->Map(m_ConstantBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
        if( SUCCEEDED(hr) )
        {
            CConstants *constants = (CConstants *)mappedResource.pData;
            constants->m_Offset[0] = dx;
            constants->m_Offset[1] = dy;
            constants->m_Offset[2] = 0;
            constants->m_Offset[3] = 0;
            Color32ToARGBf(_Color, constants->m_CstColor+3, constants->m_CstColor+0, constants->m_CstColor+1, constants->m_CstColor+2);
            m_D3DDevImmContext->Unmap(m_ConstantBuffer, 0);
        }

        // Set the input layout
        m_D3DDevImmContext->IASetInputLayout(m_TextVertexLayout);

        // Set vertex buffer
        UINT stride = sizeof(CTextVtx);
        UINT offset = 0;
        m_D3DDevImmContext->IASetVertexBuffers(0, 1, &textObj->m_TextVertexBuffer, &stride, &offset);

        // Set primitive topology
        m_D3DDevImmContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        // Render the text
        m_D3DDevImmContext->VSSetConstantBuffers(0, 1, &m_ConstantBuffer);
        if( _Color!=0 || !textObj->m_LineColors ) // use a constant color
            m_D3DDevImmContext->VSSetShader(m_TextCstColorVS, NULL, 0);
        else
            m_D3DDevImmContext->VSSetShader(m_TextVS, NULL, 0);
        m_D3DDevImmContext->PSSetShaderResources(0, 1, &m_FontD3DTexRV);
        m_D3DDevImmContext->PSSetSamplers(0, 1, &m_SamplerState);
        m_D3DDevImmContext->PSSetShader(m_TextPS, NULL, 0);
        m_D3DDevImmContext->Draw(textObj->m_NbTextVerts, 0);
    }
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D11::ChangeViewport(int _X0, int _Y0, int _Width, int _Height, int _OffsetX, int _OffsetY)
{
    if( _Width>0 && _Height>0 )
    {
	    /* viewport changes screen coordinates, use scissor instead
        D3D11_VIEWPORT vp;
        vp.TopLeftX = _X0;
        vp.TopLeftY = _Y0;
        vp.Width = _Width;
        vp.Height = _Height;
        vp.MinDepth = 0;
        vp.MaxDepth = 1;
        m_D3DDev->RSSetViewports(1, &vp);
        */
            
        m_ViewportAndScissorRects[0].left = _X0;
        m_ViewportAndScissorRects[0].right = _X0 + _Width - 1;
        m_ViewportAndScissorRects[0].top = _Y0;
        m_ViewportAndScissorRects[0].bottom = _Y0 + _Height - 1;
        if( RectIsFull(m_ViewportAndScissorRects[1]) )
            m_D3DDevImmContext->RSSetScissorRects(1, m_ViewportAndScissorRects); // viewport clipping only
        else
            m_D3DDevImmContext->RSSetScissorRects(2, m_ViewportAndScissorRects);

        m_OffsetX = _X0 + _OffsetX;
        m_OffsetY = _Y0 + _OffsetY;
    }
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D11::RestoreViewport()
{
    //m_D3DDevImmContext->RSSetViewports(1, static_cast<D3D11_VIEWPORT *>(m_ViewportInit));
    m_ViewportAndScissorRects[0] = FullRect;
    m_D3DDevImmContext->RSSetScissorRects(1, m_ViewportAndScissorRects+1); // scissor only
        
    m_OffsetX = m_OffsetY = 0;
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D11::SetScissor(int _X0, int _Y0, int _Width, int _Height)
{
    if( _Width>0 && _Height>0 )
    {
        m_ViewportAndScissorRects[1].left = _X0 - 2;
        m_ViewportAndScissorRects[1].right = _X0 + _Width - 3;
        m_ViewportAndScissorRects[1].top = _Y0 - 1;
        m_ViewportAndScissorRects[1].bottom = _Y0 + _Height - 1;
        if( RectIsFull(m_ViewportAndScissorRects[0]) )
            m_D3DDevImmContext->RSSetScissorRects(1, m_ViewportAndScissorRects+1); // no viewport clipping
        else
            m_D3DDevImmContext->RSSetScissorRects(2, m_ViewportAndScissorRects);
    }
    else
    {
        m_ViewportAndScissorRects[1] = FullRect;
        m_D3DDevImmContext->RSSetScissorRects(1, m_ViewportAndScissorRects); // apply viewport clipping only
    }
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D11::DrawTriangles(int _NumTriangles, int *_Vertices, color32 *_Colors, Cull _CullMode)
{
    assert(m_Drawing==true);

    if( _NumTriangles<=0 )
        return;

    if( m_TrianglesVertexBufferCount<3*_NumTriangles ) // force re-creation
    {
	    if( m_TrianglesVertexBuffer!=NULL )
	        m_TrianglesVertexBuffer->Release();
        m_TrianglesVertexBuffer = NULL;
        m_TrianglesVertexBufferCount = 0;
    }

    // DrawTriangles uses LineRect layout and shaders

    if( m_TrianglesVertexBuffer==NULL )
    {
        // Create triangles vertex buffer
        D3D11_BUFFER_DESC bd;
        bd.Usage = D3D11_USAGE_DYNAMIC;
        bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        bd.MiscFlags = 0;
        bd.ByteWidth = 3*_NumTriangles * sizeof(CLineRectVtx);
        bd.StructureByteStride = 0;
        HRESULT hr = m_D3DDev->CreateBuffer(&bd, NULL, &m_TrianglesVertexBuffer);
        if( SUCCEEDED(hr) )
            m_TrianglesVertexBufferCount = 3*_NumTriangles;
        else
        {
            m_TrianglesVertexBuffer = NULL;
            m_TrianglesVertexBufferCount = 0;
            return; // Problem: cannot create triangles VB
        }
    }
    assert( m_TrianglesVertexBufferCount>=3*_NumTriangles );
    assert( m_TrianglesVertexBuffer!=NULL );

    D3D11_MAPPED_SUBRESOURCE mappedResource;
    HRESULT hr = m_D3DDevImmContext->Map(m_TrianglesVertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
    if( SUCCEEDED(hr) )
    {
        CLineRectVtx *vertices = (CLineRectVtx *)mappedResource.pData;
        // Fill vertex buffer
        for( int i=0; i<3*_NumTriangles; ++ i )
        {
            vertices[i].m_Pos[0] = ToNormScreenX(_Vertices[2*i+0] + m_OffsetX, m_WndWidth);
            vertices[i].m_Pos[1] = ToNormScreenY(_Vertices[2*i+1] + m_OffsetY, m_WndHeight);
            vertices[i].m_Pos[2] = 0;
            vertices[i].m_Color = ToR8G8B8A8(_Colors[i]);
        }
        m_D3DDevImmContext->Unmap(m_TrianglesVertexBuffer, 0);

        // Reset shader constants
        hr = m_D3DDevImmContext->Map(m_ConstantBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
        if( SUCCEEDED(hr) )
        {
            CConstants *constants = (CConstants *)mappedResource.pData;
            constants->m_Offset[0] = 0;
            constants->m_Offset[1] = 0;
            constants->m_Offset[2] = 0;
            constants->m_Offset[3] = 0;
            constants->m_CstColor[0] = 1;
            constants->m_CstColor[1] = 1;
            constants->m_CstColor[2] = 1;
            constants->m_CstColor[3] = 1;
            m_D3DDevImmContext->Unmap(m_ConstantBuffer, 0);
        }

        // Set the input layout
        m_D3DDevImmContext->IASetInputLayout(m_LineRectVertexLayout);

        // Set vertex buffer
        UINT stride = sizeof(CLineRectVtx);
        UINT offset = 0;
        m_D3DDevImmContext->IASetVertexBuffers(0, 1, &m_TrianglesVertexBuffer, &stride, &offset);

        // Set primitive topology
        m_D3DDevImmContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        if( _CullMode==CULL_CW )
            m_D3DDevImmContext->RSSetState(m_RasterStateCullCW);
        else if( _CullMode==CULL_CCW )
            m_D3DDevImmContext->RSSetState(m_RasterStateCullCCW);
        else 
            m_D3DDevImmContext->RSSetState(m_RasterStateMultisample);

        // Render the triangles
        m_D3DDevImmContext->VSSetConstantBuffers(0, 1, &m_ConstantBuffer);
        m_D3DDevImmContext->VSSetShader(m_LineRectVS, NULL, 0);
        m_D3DDevImmContext->PSSetShader(m_LineRectPS, NULL, 0);
        m_D3DDevImmContext->Draw(3*_NumTriangles, 0);

        m_D3DDevImmContext->RSSetState(m_RasterState); // restore default raster state
    }
}

//  ---------------------------------------------------------------------------
