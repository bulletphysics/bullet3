//  ---------------------------------------------------------------------------
//
//  @file       TwDirect3D10.cpp
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  ---------------------------------------------------------------------------


#include "TwPrecomp.h"
#include "TwDirect3D10.h"
#include "TwMgr.h"
#include "TwColors.h"

#include "d3d10vs2003.h" // Workaround to include D3D10.h with VS2003
#define D3D10_IGNORE_SDK_LAYERS // d3d10sdklayers.h may not exist
#include <d3d10.h>


using namespace std;

const char *g_ErrCantLoadD3D10   = "Cannot load Direct3D10 library dynamically";
const char *g_ErrCompileFX10     = "Direct3D10 effect compilation failed";
const char *g_ErrCreateFX10      = "Direct3D10 effect creation failed";
const char *g_ErrTechNotFound10  = "Cannot find Direct3D10 technique effect";
const char *g_ErrCreateLayout10  = "Direct3D10 vertex layout creation failed";
const char *g_ErrCreateBuffer10  = "Direct3D10 vertex buffer creation failed";

//  ---------------------------------------------------------------------------

// Dynamically loaded D3D10 functions (to avoid static linkage with d3d10.lib)
HMODULE g_D3D10Module = NULL;
typedef HRESULT (WINAPI *D3D10CompileEffectFromMemoryProc)(void *pData, SIZE_T DataLength, LPCSTR pSrcFileName, CONST D3D10_SHADER_MACRO *pDefines, ID3D10Include *pInclude, UINT HLSLFlags, UINT FXFlags, ID3D10Blob **ppCompiledEffect, ID3D10Blob **ppErrors);
typedef HRESULT (WINAPI *D3D10CreateEffectFromMemoryProc)(void *pData, SIZE_T DataLength, UINT FXFlags, ID3D10Device *pDevice, ID3D10EffectPool *pEffectPool, ID3D10Effect **ppEffect);
typedef HRESULT (WINAPI *D3D10StateBlockMaskEnableAllProc)(D3D10_STATE_BLOCK_MASK *pMask);
typedef HRESULT (WINAPI *D3D10CreateStateBlockProc)(ID3D10Device *pDevice, D3D10_STATE_BLOCK_MASK *pStateBlockMask, ID3D10StateBlock **ppStateBlock);
D3D10CompileEffectFromMemoryProc _D3D10CompileEffectFromMemory = NULL;
D3D10CreateEffectFromMemoryProc _D3D10CreateEffectFromMemory = NULL;
D3D10StateBlockMaskEnableAllProc _D3D10StateBlockMaskEnableAll = NULL;
D3D10CreateStateBlockProc _D3D10CreateStateBlock = NULL;

const RECT FullRect = {0, 0, 16000, 16000};
static bool RectIsFull(const RECT& r) { return r.left==FullRect.left && r.right==FullRect.right && r.top==FullRect.top && r.bottom==FullRect.bottom; }

static int LoadDirect3D10()
{
    if( g_D3D10Module!=NULL )
        return 1; // Direct3D10 library already loaded

    g_D3D10Module = LoadLibrary("D3D10.DLL");
    if( g_D3D10Module )
    {
        int res = 1;
        _D3D10CompileEffectFromMemory = reinterpret_cast<D3D10CompileEffectFromMemoryProc>(GetProcAddress(g_D3D10Module, "D3D10CompileEffectFromMemory"));
        if( _D3D10CompileEffectFromMemory==NULL )
            res = 0;
        _D3D10CreateEffectFromMemory = reinterpret_cast<D3D10CreateEffectFromMemoryProc>(GetProcAddress(g_D3D10Module, "D3D10CreateEffectFromMemory"));
        if( _D3D10CreateEffectFromMemory==NULL )
            res = 0;
        _D3D10StateBlockMaskEnableAll = reinterpret_cast<D3D10StateBlockMaskEnableAllProc>(GetProcAddress(g_D3D10Module, "D3D10StateBlockMaskEnableAll"));
        if( _D3D10StateBlockMaskEnableAll==NULL )
            res = 0;
        _D3D10CreateStateBlock = reinterpret_cast<D3D10CreateStateBlockProc>(GetProcAddress(g_D3D10Module, "D3D10CreateStateBlock"));
        if( _D3D10CreateStateBlock==NULL )
            res = 0;
        return res;
    }
    else
        return 0;   // cannot load DLL
}

static int UnloadDirect3D10()
{
    _D3D10CompileEffectFromMemory = NULL;
    _D3D10CreateEffectFromMemory =  NULL;
    _D3D10StateBlockMaskEnableAll = NULL;
    _D3D10CreateStateBlock = NULL;

    if( g_D3D10Module==NULL )
        return 1; // Direct3D10 library not loaded

    if( FreeLibrary(g_D3D10Module) )
    {
        g_D3D10Module = NULL;
        return 1;
    }
    else
        return 0; // cannot unload d3d10.dll
}

//  ---------------------------------------------------------------------------

static ID3D10ShaderResourceView *BindFont(ID3D10Device *_Dev, ID3D10EffectShaderResourceVariable *_ResVar, const CTexFont *_Font)
{
    assert(_Font!=NULL);
    assert(_ResVar!=NULL);

    int w = _Font->m_TexWidth;
    int h = _Font->m_TexHeight;
    color32 *font32 = new color32[w*h];
    color32 *p = font32;
    for( int i=0; i<w*h; ++i, ++p )
        *p = 0x00ffffff | (((color32)(_Font->m_TexBytes[i]))<<24);

    D3D10_TEXTURE2D_DESC desc;
    desc.Width = w;
    desc.Height = h;
    desc.MipLevels = 1;
    desc.ArraySize = 1;
    desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    desc.SampleDesc.Count = 1;
    desc.SampleDesc.Quality = 0;
    desc.Usage = D3D10_USAGE_IMMUTABLE;
    desc.BindFlags = D3D10_BIND_SHADER_RESOURCE;
    desc.CPUAccessFlags = 0;
    desc.MiscFlags = 0;
    D3D10_SUBRESOURCE_DATA data;
    data.pSysMem = font32;
    data.SysMemPitch = w*sizeof(color32);
    data.SysMemSlicePitch = 0;
    ID3D10Texture2D *tex = NULL;
    ID3D10ShaderResourceView *texRV = NULL;
    if( SUCCEEDED(_Dev->CreateTexture2D(&desc, &data, &tex)) )
    {
        if( SUCCEEDED(_Dev->CreateShaderResourceView(tex, NULL, &texRV)) )
            if( _ResVar )
                _ResVar->SetResource(texRV);
        tex->Release();
        tex = NULL;
    }

    delete[] font32;
    return texRV;
}

//  ---------------------------------------------------------------------------

static void UnbindFont(ID3D10Device *_Dev, ID3D10EffectShaderResourceVariable *_ResVar, ID3D10ShaderResourceView *_TexRV)
{
    (void)_Dev;

    if( _ResVar )
        _ResVar->SetResource(NULL);

    if( _TexRV )
    {
        ULONG rc = _TexRV->Release();
        assert( rc==0 ); (void)rc;
    }
}

//  ---------------------------------------------------------------------------

struct CState10
{
    ID3D10StateBlock *  m_StateBlock;

    void            Save();
    void            Restore();
                    CState10(ID3D10Device *_Dev);
                    ~CState10();
private:
    ID3D10Device *  m_D3DDev;
};

CState10::CState10(ID3D10Device *_Dev)
{
    ZeroMemory(this, sizeof(CState10));
    m_D3DDev = _Dev;
}

CState10::~CState10()
{
    if( m_StateBlock )
    {
        UINT rc = m_StateBlock->Release();
        assert( rc==0 ); (void)rc;
        m_StateBlock = NULL;
    }
}

void CState10::Save()
{
    if( !m_StateBlock )
    {
        D3D10_STATE_BLOCK_MASK stateMask;
        _D3D10StateBlockMaskEnableAll(&stateMask);
        _D3D10CreateStateBlock(m_D3DDev, &stateMask, &m_StateBlock);
    }

    if( m_StateBlock )
        m_StateBlock->Capture();
}

void CState10::Restore()
{
    if( m_StateBlock )
        m_StateBlock->Apply();
}

//  ---------------------------------------------------------------------------

char g_ShaderFX[] = "// AntTweakBar shaders and techniques \n"
    " float4 g_Offset = 0; float4 g_CstColor = 1; \n"
    " struct LineRectPSInput { float4 Pos : SV_POSITION; float4 Color : COLOR0; }; \n"
    " LineRectPSInput LineRectVS(float4 pos : POSITION, float4 color : COLOR, uniform bool useCstColor) { \n"
    "   LineRectPSInput ps; ps.Pos = pos + g_Offset; \n"
    "   ps.Color = useCstColor ? g_CstColor : color; return ps; } \n"
    " float4 LineRectPS(LineRectPSInput input) : SV_Target { return input.Color; } \n"
    " technique10 LineRect { pass P0 { \n"
    "   SetVertexShader( CompileShader( vs_4_0, LineRectVS(false) ) ); \n"
    "   SetGeometryShader( NULL ); \n"
    "   SetPixelShader( CompileShader( ps_4_0, LineRectPS() ) ); \n"
    " } }\n"
    " technique10 LineRectCstColor { pass P0 { \n"
    "   SetVertexShader( CompileShader( vs_4_0, LineRectVS(true) ) ); \n"
    "   SetGeometryShader( NULL ); \n"
    "   SetPixelShader( CompileShader( ps_4_0, LineRectPS() ) ); \n"
    " } }\n"
    " Texture2D Font; \n"
    " SamplerState FontSampler { Filter = MIN_MAG_MIP_POINT; AddressU = BORDER; AddressV = BORDER; BorderColor=float4(0, 0, 0, 0); }; \n"
    " struct TextPSInput { float4 Pos : SV_POSITION; float4 Color : COLOR0; float2 Tex : TEXCOORD0; }; \n"
    " TextPSInput TextVS(float4 pos : POSITION, float4 color : COLOR, float2 tex : TEXCOORD0, uniform bool useCstColor) { \n"
    "   TextPSInput ps; ps.Pos = pos + g_Offset; \n"
    "   ps.Color = useCstColor ? g_CstColor : color; ps.Tex = tex; return ps; } \n"
    " float4 TextPS(TextPSInput input) : SV_Target { return Font.Sample(FontSampler, input.Tex)*input.Color; } \n"
    " technique10 Text { pass P0 { \n"
    "   SetVertexShader( CompileShader( vs_4_0, TextVS(false) ) ); \n"
    "   SetGeometryShader( NULL ); \n"
    "   SetPixelShader( CompileShader( ps_4_0, TextPS() ) ); \n"
    " } }\n"
    " technique10 TextCstColor { pass P0 { \n"
    "   SetVertexShader( CompileShader( vs_4_0, TextVS(true) ) ); \n"
    "   SetGeometryShader( NULL ); \n"
    "   SetPixelShader( CompileShader( ps_4_0, TextPS() ) ); \n"
    " } }\n"
    " // End of AntTweakBar shaders and techniques \n";

//  ---------------------------------------------------------------------------

int CTwGraphDirect3D10::Init()
{
    assert(g_TwMgr!=NULL);
    assert(g_TwMgr->m_Device!=NULL);

    m_D3DDev = static_cast<ID3D10Device *>(g_TwMgr->m_Device);
    m_D3DDevInitialRefCount = m_D3DDev->AddRef() - 1;

    m_Drawing = false;
    m_OffsetX = m_OffsetY = 0;
    m_ViewportInit = new D3D10_VIEWPORT;
    m_FontTex = NULL;
    m_FontD3DTexRV = NULL;
    m_WndWidth = 0;
    m_WndHeight = 0;
    m_State = NULL;
    m_DepthStencilState = NULL;
    m_BlendState = NULL;
    m_RasterState = NULL;
    m_RasterStateAntialiased = NULL;
    m_RasterStateCullCW = NULL;
    m_RasterStateCullCCW = NULL;
    m_Effect = NULL;
    m_LineRectTech = NULL;
    m_LineRectCstColorTech = NULL;
    m_LineRectVertexLayout = NULL;
    m_LineVertexBuffer = NULL;
    m_RectVertexBuffer = NULL;
    m_TrianglesVertexBuffer = NULL;
    m_TrianglesVertexBufferCount = 0;
    m_TextTech = NULL;
    m_TextCstColorTech = NULL;
    m_TextVertexLayout = NULL;
    m_FontD3DResVar = NULL;
    m_OffsetVar = NULL;
    m_CstColorVar = NULL;

    // Load some D3D10 functions
    if( !LoadDirect3D10() )
    {
        g_TwMgr->SetLastError(g_ErrCantLoadD3D10);
        Shut();
        return 0;
    }

    // Allocate state object
    m_State = new CState10(m_D3DDev);

    // Compile shaders
    DWORD shaderFlags = D3D10_SHADER_ENABLE_STRICTNESS;
    #if defined( DEBUG ) || defined( _DEBUG )
        // shaderFlags |= D3D10_SHADER_DEBUG; // no more supported
    #endif
    ID3D10Blob *compiledFX = NULL;
    ID3D10Blob *errors = NULL;
    HRESULT hr = _D3D10CompileEffectFromMemory(g_ShaderFX, strlen(g_ShaderFX), "AntTweakBarFX", NULL, NULL, shaderFlags, 0, &compiledFX, &errors);
    if( FAILED(hr) )
    {
        const size_t ERR_MSG_MAX_LEN = 4096;
        static char s_ErrorMsg[ERR_MSG_MAX_LEN]; // must be static to be sent to SetLastError
        strncpy(s_ErrorMsg, g_ErrCompileFX10, ERR_MSG_MAX_LEN-1);
        size_t errOffset = strlen(s_ErrorMsg);
        size_t errLen = 0;
        if( errors!=NULL )
        {
            s_ErrorMsg[errOffset++] = ':';
            s_ErrorMsg[errOffset++] = '\n';
            errLen = min(errors->GetBufferSize(), ERR_MSG_MAX_LEN-errOffset-2);
            strncpy(s_ErrorMsg+errOffset, static_cast<char *>(errors->GetBufferPointer()), errLen);
            errors->Release();
            errors = NULL;
        }
        s_ErrorMsg[errOffset+errLen] = '\0';
        g_TwMgr->SetLastError(s_ErrorMsg);
        Shut();
        return 0;
    }
    hr = _D3D10CreateEffectFromMemory(compiledFX->GetBufferPointer(), compiledFX->GetBufferSize(), 0, m_D3DDev, NULL, &m_Effect);
    compiledFX->Release();
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateFX10);
        Shut();
        return 0;
    }

    // Obtain the techniques
    m_LineRectTech = m_Effect->GetTechniqueByName("LineRect");
    m_LineRectCstColorTech = m_Effect->GetTechniqueByName("LineRectCstColor");
    m_TextTech = m_Effect->GetTechniqueByName("Text");
    m_TextCstColorTech = m_Effect->GetTechniqueByName("TextCstColor");
    if( m_LineRectTech==NULL || m_TextTech==NULL || m_LineRectCstColorTech==NULL || m_TextCstColorTech==NULL )
    {
        g_TwMgr->SetLastError(g_ErrTechNotFound10);
        Shut();
        return 0;
    }
 
    // Create input layout for lines & rect
    D3D10_INPUT_ELEMENT_DESC lineRectLayout[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D10_INPUT_PER_VERTEX_DATA, 0 },  
        { "COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, offsetof(CLineRectVtx, m_Color), D3D10_INPUT_PER_VERTEX_DATA, 0 }
    };
    D3D10_PASS_DESC passDesc;
    hr = m_LineRectTech->GetPassByIndex(0)->GetDesc(&passDesc);
    if( SUCCEEDED(hr) )
        hr = m_D3DDev->CreateInputLayout(lineRectLayout, sizeof(lineRectLayout)/sizeof(lineRectLayout[0]), passDesc.pIAInputSignature, passDesc.IAInputSignatureSize, &m_LineRectVertexLayout);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateLayout10);
        Shut();
        return 0;
    }

    // Create line vertex buffer
    D3D10_BUFFER_DESC bd;
    bd.Usage = D3D10_USAGE_DYNAMIC;
    bd.ByteWidth = 2 * sizeof(CLineRectVtx);
    bd.BindFlags = D3D10_BIND_VERTEX_BUFFER;
    bd.CPUAccessFlags = D3D10_CPU_ACCESS_WRITE;
    bd.MiscFlags = 0;
    hr = m_D3DDev->CreateBuffer(&bd, NULL, &m_LineVertexBuffer);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateBuffer10);
        Shut();
        return 0;
    }

    // Create rect vertex buffer
    bd.ByteWidth = 4 * sizeof(CLineRectVtx);
    hr = m_D3DDev->CreateBuffer(&bd, NULL, &m_RectVertexBuffer);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateBuffer10);
        Shut();
        return 0;
    }

    // Create input layout for text
    D3D10_INPUT_ELEMENT_DESC textLayout[] =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D10_INPUT_PER_VERTEX_DATA, 0 },  
        { "COLOR", 0, DXGI_FORMAT_R8G8B8A8_UNORM, 0, offsetof(CTextVtx, m_Color), D3D10_INPUT_PER_VERTEX_DATA, 0 }, 
        { "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, offsetof(CTextVtx, m_UV), D3D10_INPUT_PER_VERTEX_DATA, 0 }
    };
    hr = m_TextTech->GetPassByIndex(0)->GetDesc(&passDesc);
    if( SUCCEEDED(hr) )
        hr = m_D3DDev->CreateInputLayout(textLayout, sizeof(textLayout)/sizeof(textLayout[0]), passDesc.pIAInputSignature, passDesc.IAInputSignatureSize, &m_TextVertexLayout);
    if( FAILED(hr) )
    {
        g_TwMgr->SetLastError(g_ErrCreateLayout10);
        Shut();
        return 0;
    }

    // Create depth stencil state object
    D3D10_DEPTH_STENCILOP_DESC od;
    od.StencilFunc = D3D10_COMPARISON_ALWAYS;
    od.StencilFailOp = D3D10_STENCIL_OP_KEEP;
    od.StencilPassOp = D3D10_STENCIL_OP_KEEP;
    od.StencilDepthFailOp = D3D10_STENCIL_OP_KEEP;
    D3D10_DEPTH_STENCIL_DESC dsd;
    dsd.DepthEnable = FALSE;
    dsd.DepthWriteMask = D3D10_DEPTH_WRITE_MASK_ZERO;
    dsd.DepthFunc = D3D10_COMPARISON_ALWAYS;
    dsd.StencilEnable = FALSE;
    dsd.StencilReadMask = D3D10_DEFAULT_STENCIL_READ_MASK;
    dsd.StencilWriteMask = D3D10_DEFAULT_STENCIL_WRITE_MASK;
    dsd.FrontFace = od;
    dsd.BackFace = od;
    m_D3DDev->CreateDepthStencilState(&dsd, &m_DepthStencilState);

    // Create blend state object
    D3D10_BLEND_DESC bsd;
    bsd.AlphaToCoverageEnable = FALSE;
    for(int i=0; i<8; ++i)
    {
        bsd.BlendEnable[i] = TRUE;
        bsd.RenderTargetWriteMask[i] = D3D10_COLOR_WRITE_ENABLE_ALL;
    }
    bsd.SrcBlend = D3D10_BLEND_SRC_ALPHA;
    bsd.DestBlend = D3D10_BLEND_INV_SRC_ALPHA;
    bsd.BlendOp =  D3D10_BLEND_OP_ADD;
    bsd.SrcBlendAlpha = D3D10_BLEND_SRC_ALPHA;
    bsd.DestBlendAlpha = D3D10_BLEND_INV_SRC_ALPHA;
    bsd.BlendOpAlpha = D3D10_BLEND_OP_ADD;
    m_D3DDev->CreateBlendState(&bsd, &m_BlendState);

    // Create rasterizer state object
    D3D10_RASTERIZER_DESC rd;
    rd.FillMode = D3D10_FILL_SOLID;
    rd.CullMode = D3D10_CULL_NONE;
    rd.FrontCounterClockwise = true;
    rd.DepthBias = false;
    rd.DepthBiasClamp = 0;
    rd.SlopeScaledDepthBias = 0;
    rd.DepthClipEnable = false;
    rd.ScissorEnable = true;
    rd.MultisampleEnable = false;
    rd.AntialiasedLineEnable = false;
    m_D3DDev->CreateRasterizerState(&rd, &m_RasterState);

    rd.AntialiasedLineEnable = true;
    m_D3DDev->CreateRasterizerState(&rd, &m_RasterStateAntialiased);
    rd.AntialiasedLineEnable = false;

    rd.CullMode = D3D10_CULL_BACK;
    m_D3DDev->CreateRasterizerState(&rd, &m_RasterStateCullCW);

    rd.CullMode = D3D10_CULL_FRONT;
    m_D3DDev->CreateRasterizerState(&rd, &m_RasterStateCullCCW);

    m_ViewportAndScissorRects[0] = FullRect;
    m_ViewportAndScissorRects[1] = FullRect;
    m_D3DDev->RSSetScissorRects(1, m_ViewportAndScissorRects);    
    
    // Get effect globals
    if( m_Effect->GetVariableByName("Font") )
        m_FontD3DResVar = m_Effect->GetVariableByName("Font")->AsShaderResource();
    assert( m_FontD3DResVar!=NULL );
    if( m_Effect->GetVariableByName("g_Offset") )
        m_OffsetVar = m_Effect->GetVariableByName("g_Offset")->AsVector();
    assert( m_OffsetVar!=NULL );
    if( m_Effect->GetVariableByName("g_CstColor") )
        m_CstColorVar = m_Effect->GetVariableByName("g_CstColor")->AsVector();
    assert( m_CstColorVar!=NULL );

    return 1;
}

//  ---------------------------------------------------------------------------

int CTwGraphDirect3D10::Shut()
{
    assert(m_Drawing==false);

    UnbindFont(m_D3DDev, m_FontD3DResVar, m_FontD3DTexRV);
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

    m_FontD3DResVar = NULL;
    m_OffsetVar = NULL;
    m_CstColorVar = NULL;

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
    if( m_Effect )
    {
        ULONG rc = m_Effect->Release();
        assert( rc==0 ); (void)rc;
        m_Effect = NULL;
    }

    if( m_D3DDev )
    {
        //unsigned int rc = m_D3DDev->Release();
        //assert( m_D3DDevInitialRefCount==rc ); (void)rc;
        m_D3DDev->Release();
        m_D3DDev = NULL;
    }

    // Unload D3D10
    UnloadDirect3D10(); // this is not a problem if it cannot be unloaded

    return 1;
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D10::BeginDraw(int _WndWidth, int _WndHeight)
{
    assert(m_Drawing==false && _WndWidth>0 && _WndHeight>0);
    m_Drawing = true;

    m_WndWidth  = _WndWidth;
    m_WndHeight = _WndHeight;
    m_OffsetX = m_OffsetY = 0;

    // save context
    m_State->Save();

    // Setup the viewport
    D3D10_VIEWPORT vp;
    vp.Width = _WndWidth;
    vp.Height = _WndHeight;
    vp.MinDepth = 0.0f;
    vp.MaxDepth = 1.0f;
    vp.TopLeftX = 0;
    vp.TopLeftY = 0;
    m_D3DDev->RSSetViewports(1, &vp);
    *static_cast<D3D10_VIEWPORT *>(m_ViewportInit) = vp;

    m_D3DDev->RSSetState(m_RasterState);

    m_D3DDev->OMSetDepthStencilState(m_DepthStencilState, 0);
    float blendFactors[4] = { 1, 1, 1, 1 };
    m_D3DDev->OMSetBlendState(m_BlendState, blendFactors, 0xffffffff);
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D10::EndDraw()
{
    m_D3DDev->RSSetState(NULL);
    m_D3DDev->OMSetDepthStencilState(NULL, 0);
    m_D3DDev->OMSetBlendState(NULL, NULL, 0xffffffff);

    assert(m_Drawing==true);
    m_Drawing = false;

    // restore context
    m_State->Restore();
}

//  ---------------------------------------------------------------------------

bool CTwGraphDirect3D10::IsDrawing()
{
    return m_Drawing;
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D10::Restore()
{
    if( m_State )
    {
        if( m_State->m_StateBlock )
        {
            UINT rc = m_State->m_StateBlock->Release();
            assert( rc==0 ); (void)rc;
            m_State->m_StateBlock = NULL;
        }
    }

    UnbindFont(m_D3DDev, m_FontD3DResVar, m_FontD3DTexRV);
    m_FontD3DTexRV = NULL;
    
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

void CTwGraphDirect3D10::DrawLine(int _X0, int _Y0, int _X1, int _Y1, color32 _Color0, color32 _Color1, bool _AntiAliased)
{
    assert(m_Drawing==true);

    float x0 = ToNormScreenX(_X0 + m_OffsetX, m_WndWidth);
    float y0 = ToNormScreenY(_Y0 + m_OffsetY, m_WndHeight);
    float x1 = ToNormScreenX(_X1 + m_OffsetX, m_WndWidth);
    float y1 = ToNormScreenY(_Y1 + m_OffsetY, m_WndHeight);
 
    CLineRectVtx *vertices = NULL;
    HRESULT hr = m_LineVertexBuffer->Map(D3D10_MAP_WRITE_DISCARD, 0, (void **)&vertices);
    if( SUCCEEDED(hr) )
    {
        // Fill vertex buffer
        vertices[0].m_Pos[0] = x0;
        vertices[0].m_Pos[1] = y0;
        vertices[0].m_Pos[2] = 0;
        vertices[0].m_Color = ToR8G8B8A8(_Color0);
        vertices[1].m_Pos[0] = x1;
        vertices[1].m_Pos[1] = y1;
        vertices[1].m_Pos[2] = 0;
        vertices[1].m_Color = ToR8G8B8A8(_Color1);

        m_LineVertexBuffer->Unmap();

        if( _AntiAliased )
            m_D3DDev->RSSetState(m_RasterStateAntialiased);

        // Reset shader globals
        float offsetVec[4] = { 0, 0, 0, 0 };
        if( m_OffsetVar )
            m_OffsetVar->SetFloatVector(offsetVec);
        float colorVec[4] = { 1, 1, 1, 1 };
        if( m_CstColorVar )
            m_CstColorVar->SetFloatVector(colorVec);

        // Set the input layout
        m_D3DDev->IASetInputLayout(m_LineRectVertexLayout);

        // Set vertex buffer
        UINT stride = sizeof(CLineRectVtx);
        UINT offset = 0;
        m_D3DDev->IASetVertexBuffers(0, 1, &m_LineVertexBuffer, &stride, &offset);

        // Set primitive topology
        m_D3DDev->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_LINELIST);

        // Render the line
        D3D10_TECHNIQUE_DESC techDesc;
        m_LineRectTech->GetDesc(&techDesc);
        for(UINT p=0; p<techDesc.Passes; ++p)
        {
            m_LineRectTech->GetPassByIndex(p)->Apply(0);
            m_D3DDev->Draw(2, 0);
        }

        if( _AntiAliased )
            m_D3DDev->RSSetState(m_RasterState); // restore default raster state
    }
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D10::DrawRect(int _X0, int _Y0, int _X1, int _Y1, color32 _Color00, color32 _Color10, color32 _Color01, color32 _Color11)
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
 
    CLineRectVtx *vertices = NULL;
    HRESULT hr = m_RectVertexBuffer->Map(D3D10_MAP_WRITE_DISCARD, 0, (void **)&vertices);
    if( SUCCEEDED(hr) )
    {
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

        m_RectVertexBuffer->Unmap();

        // Reset shader globals
        float offsetVec[4] = { 0, 0, 0, 0 };
        if( m_OffsetVar )
            m_OffsetVar->SetFloatVector(offsetVec);
        float colorVec[4] = { 1, 1, 1, 1 };
        if( m_CstColorVar )
            m_CstColorVar->SetFloatVector(colorVec);

        // Set the input layout
        m_D3DDev->IASetInputLayout(m_LineRectVertexLayout);

        // Set vertex buffer
        UINT stride = sizeof(CLineRectVtx);
        UINT offset = 0;
        m_D3DDev->IASetVertexBuffers(0, 1, &m_RectVertexBuffer, &stride, &offset);

        // Set primitive topology
        m_D3DDev->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);

        // Render the rect
        D3D10_TECHNIQUE_DESC techDesc;
        m_LineRectTech->GetDesc(&techDesc);
        for(UINT p=0; p<techDesc.Passes; ++p)
        {
            m_LineRectTech->GetPassByIndex(p)->Apply(0);
            m_D3DDev->Draw(4, 0);
        }
    }
}

//  ---------------------------------------------------------------------------

void *CTwGraphDirect3D10::NewTextObj()
{
    CTextObj *textObj = new CTextObj;
    memset(textObj, 0, sizeof(CTextObj));
    return textObj;
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D10::DeleteTextObj(void *_TextObj)
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

void CTwGraphDirect3D10::BuildText(void *_TextObj, const std::string *_TextLines, color32 *_LineColors, color32 *_LineBgColors, int _NbLines, const CTexFont *_Font, int _Sep, int _BgWidth)
{
    assert(m_Drawing==true);
    assert(_TextObj!=NULL);
    assert(_Font!=NULL);

    if( _Font != m_FontTex )
    {
        UnbindFont(m_D3DDev, m_FontD3DResVar, m_FontD3DTexRV);
        m_FontD3DTexRV = BindFont(m_D3DDev, m_FontD3DResVar, _Font);
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
            D3D10_BUFFER_DESC bd;
            bd.Usage = D3D10_USAGE_DYNAMIC;
            bd.ByteWidth = textObj->m_TextVertexBufferSize * sizeof(CTextVtx);
            bd.BindFlags = D3D10_BIND_VERTEX_BUFFER;
            bd.CPUAccessFlags = D3D10_CPU_ACCESS_WRITE;
            bd.MiscFlags = 0;
            m_D3DDev->CreateBuffer(&bd, NULL, &textObj->m_TextVertexBuffer);
        }

        if( textObj->m_TextVertexBuffer!=NULL )
            textObj->m_TextVertexBuffer->Map(D3D10_MAP_WRITE_DISCARD, 0, (void **)&textVerts);
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
            D3D10_BUFFER_DESC bd;
            bd.Usage = D3D10_USAGE_DYNAMIC;
            bd.ByteWidth = textObj->m_BgVertexBufferSize * sizeof(CLineRectVtx);
            bd.BindFlags = D3D10_BIND_VERTEX_BUFFER;
            bd.CPUAccessFlags = D3D10_CPU_ACCESS_WRITE;
            bd.MiscFlags = 0;
            m_D3DDev->CreateBuffer(&bd, NULL, &textObj->m_BgVertexBuffer);
        }

        if( textObj->m_BgVertexBuffer!=NULL )
            textObj->m_BgVertexBuffer->Map(D3D10_MAP_WRITE_DISCARD, 0, (void **)&bgVerts);
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
        textObj->m_TextVertexBuffer->Unmap();
    if( bgVerts!=NULL )
        textObj->m_BgVertexBuffer->Unmap();
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D10::DrawText(void *_TextObj, int _X, int _Y, color32 _Color, color32 _BgColor)
{
    assert(m_Drawing==true);
    assert(_TextObj!=NULL);
    CTextObj *textObj = static_cast<CTextObj *>(_TextObj);
    float dx = 2.0f*(float)(_X + m_OffsetX)/m_WndWidth;
    float dy = -2.0f*(float)(_Y + m_OffsetY)/m_WndHeight;
 
    float offsetVec[4] = { 0, 0, 0, 0 };
    offsetVec[0] = dx;
    offsetVec[1] = dy;
    if( m_OffsetVar )
        m_OffsetVar->SetFloatVector(offsetVec);

    // Draw background
    if( textObj->m_NbBgVerts>=4 && textObj->m_BgVertexBuffer!=NULL )
    {
        float color[4];
        Color32ToARGBf(_BgColor, color+3, color+0, color+1, color+2);
        if( m_CstColorVar )
            m_CstColorVar->SetFloatVector(color);

        // Set the input layout
        m_D3DDev->IASetInputLayout(m_LineRectVertexLayout);

        // Set vertex buffer
        UINT stride = sizeof(CLineRectVtx);
        UINT offset = 0;
        m_D3DDev->IASetVertexBuffers(0, 1, &textObj->m_BgVertexBuffer, &stride, &offset);

        // Set primitive topology
        m_D3DDev->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        // Render the bg rectangles
        ID3D10EffectTechnique *tech;
        if( _BgColor!=0 || !textObj->m_LineBgColors ) // use a constant bg color
            tech = m_LineRectCstColorTech;
        else // use vertex buffer colors
            tech = m_LineRectTech;
        D3D10_TECHNIQUE_DESC techDesc;
        tech->GetDesc(&techDesc);
        for( UINT p=0; p<techDesc.Passes; ++p )
        {
            tech->GetPassByIndex(p)->Apply(0);
            m_D3DDev->Draw(textObj->m_NbBgVerts, 0);
        }
    }

    // Draw text
    if( textObj->m_NbTextVerts>=4 && textObj->m_TextVertexBuffer!=NULL )
    {
        float color[4];
        Color32ToARGBf(_Color, color+3, color+0, color+1, color+2);
        if( m_CstColorVar )
            m_CstColorVar->SetFloatVector(color);

        // Set the input layout
        m_D3DDev->IASetInputLayout(m_TextVertexLayout);

        // Set vertex buffer
        UINT stride = sizeof(CTextVtx);
        UINT offset = 0;
        m_D3DDev->IASetVertexBuffers(0, 1, &textObj->m_TextVertexBuffer, &stride, &offset);

        // Set primitive topology
        m_D3DDev->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        // Render text
        ID3D10EffectTechnique *tech;
        if( _Color!=0 || !textObj->m_LineColors ) // use a constant color
            tech = m_TextCstColorTech;
        else // use vertex buffer colors
            tech = m_TextTech;
        D3D10_TECHNIQUE_DESC techDesc;
        tech->GetDesc(&techDesc);
        for( UINT p=0; p<techDesc.Passes; ++p )
        {
            tech->GetPassByIndex(p)->Apply(0);
            m_D3DDev->Draw(textObj->m_NbTextVerts, 0);
        }
    }
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D10::ChangeViewport(int _X0, int _Y0, int _Width, int _Height, int _OffsetX, int _OffsetY)
{
    if( _Width>0 && _Height>0 )
    {
	    /* viewport changes screen coordinates, use scissor instead
        D3D10_VIEWPORT vp;
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
            m_D3DDev->RSSetScissorRects(1, m_ViewportAndScissorRects); // viewport clipping only
        else
            m_D3DDev->RSSetScissorRects(2, m_ViewportAndScissorRects);

        m_OffsetX = _X0 + _OffsetX;
        m_OffsetY = _Y0 + _OffsetY;
    }
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D10::RestoreViewport()
{
    //m_D3DDev->RSSetViewports(1, static_cast<D3D10_VIEWPORT *>(m_ViewportInit));
    m_ViewportAndScissorRects[0] = FullRect;
    m_D3DDev->RSSetScissorRects(1, m_ViewportAndScissorRects+1); // scissor only
        
    m_OffsetX = m_OffsetY = 0;
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D10::SetScissor(int _X0, int _Y0, int _Width, int _Height)
{
    if( _Width>0 && _Height>0 )
    {
        m_ViewportAndScissorRects[1].left = _X0 - 2;
        m_ViewportAndScissorRects[1].right = _X0 + _Width - 3;
        m_ViewportAndScissorRects[1].top = _Y0 - 1;
        m_ViewportAndScissorRects[1].bottom = _Y0 + _Height - 1;
        if( RectIsFull(m_ViewportAndScissorRects[0]) )
            m_D3DDev->RSSetScissorRects(1, m_ViewportAndScissorRects+1); // no viewport clipping
        else
            m_D3DDev->RSSetScissorRects(2, m_ViewportAndScissorRects);
    }
    else
    {
        m_ViewportAndScissorRects[1] = FullRect;
        m_D3DDev->RSSetScissorRects(1, m_ViewportAndScissorRects); // apply viewport clipping only
    }
}

//  ---------------------------------------------------------------------------

void CTwGraphDirect3D10::DrawTriangles(int _NumTriangles, int *_Vertices, color32 *_Colors, Cull _CullMode)
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

    // DrawTriangles uses LineRect layout and technique

    if( m_TrianglesVertexBuffer==NULL )
    {
        // Create triangles vertex buffer
        D3D10_BUFFER_DESC bd;
        bd.Usage = D3D10_USAGE_DYNAMIC;
        bd.BindFlags = D3D10_BIND_VERTEX_BUFFER;
        bd.CPUAccessFlags = D3D10_CPU_ACCESS_WRITE;
        bd.MiscFlags = 0;
        bd.ByteWidth = 3*_NumTriangles * sizeof(CLineRectVtx);
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

    CLineRectVtx *vertices = NULL;
    HRESULT hr = m_TrianglesVertexBuffer->Map(D3D10_MAP_WRITE_DISCARD, 0, (void **)&vertices);
    if( SUCCEEDED(hr) )
    {
        // Fill vertex buffer
        for( int i=0; i<3*_NumTriangles; ++ i )
        {
            vertices[i].m_Pos[0] = ToNormScreenX(_Vertices[2*i+0] + m_OffsetX, m_WndWidth);
            vertices[i].m_Pos[1] = ToNormScreenY(_Vertices[2*i+1] + m_OffsetY, m_WndHeight);
            vertices[i].m_Pos[2] = 0;
            vertices[i].m_Color = ToR8G8B8A8(_Colors[i]);
        }
        m_TrianglesVertexBuffer->Unmap();

        // Reset shader globals
        float offsetVec[4] = { 0, 0, 0, 0 };
        if( m_OffsetVar )
            m_OffsetVar->SetFloatVector(offsetVec);
        float colorVec[4] = { 1, 1, 1, 1 };
        if( m_CstColorVar )
            m_CstColorVar->SetFloatVector(colorVec);

        // Set the input layout
        m_D3DDev->IASetInputLayout(m_LineRectVertexLayout);

        // Set vertex buffer
        UINT stride = sizeof(CLineRectVtx);
        UINT offset = 0;
        m_D3DDev->IASetVertexBuffers(0, 1, &m_TrianglesVertexBuffer, &stride, &offset);

        // Set primitive topology
        m_D3DDev->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        if( _CullMode==CULL_CW )
            m_D3DDev->RSSetState(m_RasterStateCullCW);
        else if( _CullMode==CULL_CCW )
            m_D3DDev->RSSetState(m_RasterStateCullCCW);

        // Render the triangles
        D3D10_TECHNIQUE_DESC techDesc;
        m_LineRectTech->GetDesc(&techDesc);
        for(UINT p=0; p<techDesc.Passes; ++p)
        {
            m_LineRectTech->GetPassByIndex(p)->Apply(0);
            m_D3DDev->Draw(3*_NumTriangles, 0);
        }

        if( _CullMode==CULL_CW || _CullMode==CULL_CCW )
            m_D3DDev->RSSetState(m_RasterState); // restore default raster state

        // Unset vertex buffer
        ID3D10Buffer *vb = NULL;
        m_D3DDev->IASetVertexBuffers(0, 1, &vb, &stride, &offset);
    }
}

//  ---------------------------------------------------------------------------
