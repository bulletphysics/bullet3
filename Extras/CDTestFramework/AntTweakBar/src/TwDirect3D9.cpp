//	---------------------------------------------------------------------------
//
//	@file		TwDirect3D9.cpp
//	@author		Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//				Copyright © 2005, 2006 Philippe Decaudin.
//              For conditions of distribution and use, see License.txt
//
//	note:		TAB=4
//
//	---------------------------------------------------------------------------


#include "TwPrecomp.h"
#include "TwDirect3D9.h"
#include "TwMgr.h"

#include <d3d9.h>
#ifdef _DEBUG
	#include <dxerr9.h>
	#pragma comment(lib, "dxerr9")
#endif // _DEBUG


using namespace std;

const char *g_ErrCantLoadD3D9	= "Cannot load Direct3D9 library dynamically";
const char *g_ErrCantUnloadD3D9	= "Cannot unload Direct3D9 library";


//	---------------------------------------------------------------------------

static IDirect3DTexture9 *BindFont(IDirect3DDevice9 *_Dev, const CTexFont *_Font)
{
	assert(_Font!=NULL);

	IDirect3DTexture9 *Tex = NULL;
	HRESULT hr = _Dev->CreateTexture(_Font->m_TexWidth, _Font->m_TexHeight, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &Tex, NULL);
	if( FAILED(hr) )
		return NULL;

	D3DLOCKED_RECT r;
	hr = Tex->LockRect(0, &r, NULL, 0);
	if( SUCCEEDED(hr) )
	{
		color32 *p = static_cast<color32 *>(r.pBits);
		for( int i=0; i<_Font->m_TexWidth*_Font->m_TexHeight; ++i, ++p )
			*p = 0x00ffffff | (((color32)(_Font->m_TexBytes[i]))<<24);
		Tex->UnlockRect(0);
	}
	return Tex;
}

//	---------------------------------------------------------------------------

static void UnbindFont(IDirect3DDevice9 *_Dev, IDirect3DTexture9 *_Tex)
{
	(void)_Dev;

	if( _Tex )
		_Tex->Release();
}

//	---------------------------------------------------------------------------

struct CState
{
	// viewport
	D3DVIEWPORT9	m_Vp;

	// render states
	DWORD			m_Z;
	DWORD			m_CullMode;
	DWORD			m_AlphaTest;
	DWORD			m_AlphaBlend;
	DWORD			m_BlendOp;
	DWORD			m_SrcBlend;
	DWORD			m_DstBlend;
	DWORD			m_ClipPlane;
	DWORD			m_FillMode;
	DWORD			m_LastPixel;
	DWORD			m_Fog;
	DWORD			m_Stencil;
	DWORD			m_ColorWrite;
	DWORD			m_Scissor;
	DWORD			m_SeparateAlphaBlend;
	DWORD			m_AntiAliasedLine;

	// primitive
	DWORD			m_FVF;
	IDirect3DVertexBuffer9 *m_StreamData;
	IDirect3DVertexShader9 *m_VertexShader;
	UINT			m_StreamOffset;
	UINT			m_StreamStride;

	// texture
	IDirect3DBaseTexture9 *m_Tex;
	IDirect3DPixelShader9 *m_PixelShader;

	// texture stage states
	DWORD			m_ColorOp;
	DWORD			m_ColorArg1;
	DWORD			m_ColorArg2;
	DWORD			m_AlphaOp;
	DWORD			m_AlphaArg1;
	DWORD			m_TexCoordIndex;
	DWORD			m_TexTransfFlags;
	DWORD			m_AddressU;
	DWORD			m_AddressV;
	DWORD			m_MinFilter;
	DWORD			m_MagFilter;
	DWORD			m_MipFilter;

	// DeviceCaps (filled by constructor)
	D3DCAPS9		m_Caps;

	void			Save();
	void			Restore();
					CState(IDirect3DDevice9 *_Dev);
private:
	IDirect3DDevice9 *m_D3DDev;
};

CState::CState(IDirect3DDevice9 *_Dev)
{
	ZeroMemory(this, sizeof(CState));
	m_D3DDev = _Dev;

	m_D3DDev->GetDeviceCaps(&m_Caps);
}

void CState::Save()
{
	// viewport
	m_D3DDev->GetViewport(&m_Vp);

	// render states
	m_D3DDev->GetRenderState(D3DRS_ZENABLE, &m_Z);
	m_D3DDev->GetRenderState(D3DRS_CULLMODE, &m_CullMode);
	m_D3DDev->GetRenderState(D3DRS_ALPHATESTENABLE, &m_AlphaTest);
	m_D3DDev->GetRenderState(D3DRS_ALPHABLENDENABLE, &m_AlphaBlend);
	m_D3DDev->GetRenderState(D3DRS_BLENDOP, &m_BlendOp);
	m_D3DDev->GetRenderState(D3DRS_SRCBLEND, &m_SrcBlend);
	m_D3DDev->GetRenderState(D3DRS_DESTBLEND, &m_DstBlend);
	m_D3DDev->GetRenderState(D3DRS_CLIPPLANEENABLE, &m_ClipPlane);
	m_D3DDev->GetRenderState(D3DRS_FILLMODE, &m_FillMode);
	m_D3DDev->GetRenderState(D3DRS_LASTPIXEL, &m_LastPixel);
	m_D3DDev->GetRenderState(D3DRS_FOGENABLE, &m_Fog);
	m_D3DDev->GetRenderState(D3DRS_STENCILENABLE, &m_Stencil);
	m_D3DDev->GetRenderState(D3DRS_COLORWRITEENABLE, &m_ColorWrite);
	m_D3DDev->GetRenderState(D3DRS_SCISSORTESTENABLE, &m_Scissor);
	if( m_Caps.PrimitiveMiscCaps & D3DPMISCCAPS_SEPARATEALPHABLEND )
		m_D3DDev->GetRenderState(D3DRS_SEPARATEALPHABLENDENABLE, &m_SeparateAlphaBlend);
	//if( m_Caps.LineCaps & D3DLINECAPS_ANTIALIAS )
		m_D3DDev->GetRenderState(D3DRS_ANTIALIASEDLINEENABLE, &m_AntiAliasedLine);

	// primitive
	m_D3DDev->GetFVF(&m_FVF);
	m_D3DDev->GetStreamSource(0, &m_StreamData, &m_StreamOffset, &m_StreamStride);
	m_D3DDev->GetVertexShader(&m_VertexShader);

	// texture
	m_D3DDev->GetTexture(0, &m_Tex);
	m_D3DDev->GetPixelShader(&m_PixelShader);

	// texture stage states
	m_D3DDev->GetTextureStageState(0, D3DTSS_COLOROP, &m_ColorOp);
	m_D3DDev->GetTextureStageState(0, D3DTSS_COLORARG1, &m_ColorArg1);
	m_D3DDev->GetTextureStageState(0, D3DTSS_COLORARG2, &m_ColorArg2);
	m_D3DDev->GetTextureStageState(0, D3DTSS_ALPHAOP, &m_AlphaOp);
	m_D3DDev->GetTextureStageState(0, D3DTSS_ALPHAARG1, &m_AlphaArg1);
	m_D3DDev->GetTextureStageState(0, D3DTSS_TEXCOORDINDEX, &m_TexCoordIndex);
	m_D3DDev->GetTextureStageState(0, D3DTSS_TEXTURETRANSFORMFLAGS, &m_TexTransfFlags);
	m_D3DDev->GetSamplerState(0, D3DSAMP_ADDRESSU, &m_AddressU);
	m_D3DDev->GetSamplerState(0, D3DSAMP_ADDRESSV, &m_AddressV);
	m_D3DDev->GetSamplerState(0, D3DSAMP_MAGFILTER, &m_MagFilter);
	m_D3DDev->GetSamplerState(0, D3DSAMP_MINFILTER, &m_MinFilter);
	m_D3DDev->GetSamplerState(0, D3DSAMP_MIPFILTER, &m_MipFilter);
}

void CState::Restore()
{
	// viewport
	m_D3DDev->SetViewport(&m_Vp);

	// render states
	m_D3DDev->SetRenderState(D3DRS_ZENABLE, m_Z);
	m_D3DDev->SetRenderState(D3DRS_CULLMODE, m_CullMode);
	m_D3DDev->SetRenderState(D3DRS_ALPHATESTENABLE, m_AlphaTest);
	m_D3DDev->SetRenderState(D3DRS_ALPHABLENDENABLE, m_AlphaBlend);
	m_D3DDev->SetRenderState(D3DRS_BLENDOP, m_BlendOp);
	m_D3DDev->SetRenderState(D3DRS_SRCBLEND, m_SrcBlend);
	m_D3DDev->SetRenderState(D3DRS_DESTBLEND, m_DstBlend);
	m_D3DDev->SetRenderState(D3DRS_CLIPPLANEENABLE, m_ClipPlane);
	m_D3DDev->SetRenderState(D3DRS_FILLMODE, m_FillMode);
	m_D3DDev->SetRenderState(D3DRS_LASTPIXEL, m_LastPixel);
	m_D3DDev->SetRenderState(D3DRS_FOGENABLE, m_Fog);
	m_D3DDev->SetRenderState(D3DRS_STENCILENABLE, m_Stencil);
	m_D3DDev->SetRenderState(D3DRS_COLORWRITEENABLE, m_ColorWrite);
	m_D3DDev->SetRenderState(D3DRS_SCISSORTESTENABLE, m_Scissor);
	if( m_Caps.PrimitiveMiscCaps & D3DPMISCCAPS_SEPARATEALPHABLEND )
		m_D3DDev->SetRenderState(D3DRS_SEPARATEALPHABLENDENABLE, m_SeparateAlphaBlend);
	//if( m_Caps.LineCaps & D3DLINECAPS_ANTIALIAS )
		m_D3DDev->SetRenderState(D3DRS_ANTIALIASEDLINEENABLE, m_AntiAliasedLine);

	// primitive
	m_D3DDev->SetFVF(m_FVF);
	m_D3DDev->SetStreamSource(0, m_StreamData, m_StreamOffset, m_StreamStride);
	if( m_StreamData )
		m_StreamData->Release();
	m_D3DDev->SetVertexShader(m_VertexShader);
	if( m_VertexShader )
		m_VertexShader->Release();

	// texture
	m_D3DDev->SetTexture(0, m_Tex);
	if( m_Tex )
		m_Tex->Release();
	m_D3DDev->SetPixelShader(m_PixelShader);
	if( m_PixelShader )
		m_PixelShader->Release();

	// texture stage states
	m_D3DDev->SetTextureStageState(0, D3DTSS_COLOROP, m_ColorOp);
	m_D3DDev->SetTextureStageState(0, D3DTSS_COLORARG1, m_ColorArg1);
	m_D3DDev->SetTextureStageState(0, D3DTSS_COLORARG2, m_ColorArg2);
	m_D3DDev->SetTextureStageState(0, D3DTSS_ALPHAOP, m_AlphaOp);
	m_D3DDev->SetTextureStageState(0, D3DTSS_ALPHAARG1, m_AlphaArg1);
	m_D3DDev->SetTextureStageState(0, D3DTSS_TEXCOORDINDEX, m_TexCoordIndex);
	m_D3DDev->SetTextureStageState(0, D3DTSS_TEXTURETRANSFORMFLAGS, m_TexTransfFlags);
	m_D3DDev->SetSamplerState(0, D3DSAMP_ADDRESSU, m_AddressU);
	m_D3DDev->SetSamplerState(0, D3DSAMP_ADDRESSV, m_AddressV);
	m_D3DDev->SetSamplerState(0, D3DSAMP_MAGFILTER, m_MagFilter);
	m_D3DDev->SetSamplerState(0, D3DSAMP_MINFILTER, m_MinFilter);
	m_D3DDev->SetSamplerState(0, D3DSAMP_MIPFILTER, m_MipFilter);
}

//	---------------------------------------------------------------------------

int CTwGraphDirect3D9::Init()
{
	assert(g_TwMgr->m_Device!=NULL);

	m_D3DDev = static_cast<IDirect3DDevice9 *>(g_TwMgr->m_Device);
	m_Drawing = false;
	m_FontTex = NULL;
	m_FontD3DTex = NULL;
	D3DDEVICE_CREATION_PARAMETERS cp;
	m_D3DDev->GetCreationParameters(&cp);
	m_PureDevice = ( cp.BehaviorFlags & D3DCREATE_PUREDEVICE ) ? true : false;
	m_WndWidth = 0;
	m_WndHeight = 0;
	m_State = new CState(m_D3DDev);

	return 1;
}

//	---------------------------------------------------------------------------

int CTwGraphDirect3D9::Shut()
{
	assert(m_Drawing==false);

	UnbindFont(m_D3DDev, m_FontD3DTex);
	m_FontD3DTex = NULL;
	delete m_State;
	m_State = NULL;
	m_D3DDev = NULL;

	return 1;
}

//	---------------------------------------------------------------------------

void CTwGraphDirect3D9::BeginDraw(int _WndWidth, int _WndHeight)
{
	assert(m_Drawing==false && _WndWidth>0 && _WndHeight>0);
	m_Drawing = true;

	m_WndWidth  = _WndWidth;
	m_WndHeight = _WndHeight;

	// save context
	if( !m_PureDevice )
		m_State->Save();

	if( m_WndWidth>0 && m_WndHeight>0 )
	{
		D3DVIEWPORT9 Vp;
		Vp.X = 0;
		Vp.Y = 0;
		Vp.Width = m_WndWidth;
		Vp.Height = m_WndHeight;
		Vp.MinZ = 0;
		Vp.MaxZ = 1;
		m_D3DDev->SetViewport(&Vp);

		//D3DMATRIX Transfo = { 2.0f/_WndWidth,0,0,0, 0,2.0f/_WndHeight,0,0, 0,0,-1,0, 0,0,0,1 };
		//m_D3DDev->SetTransform(D3DTS_PROJECTION, &Transfo);
	}
	//  const D3DMATRIX id = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
	//	m_D3DDev->SetTransform(D3DTS_VIEW, &id);
	//	m_D3DDev->SetTransform(D3DTS_WORLD, &id);
	//  m_D3DDev->SetTransform(D3DTS_TEXTURE0, &id);

	m_D3DDev->SetRenderState(D3DRS_ZENABLE, D3DZB_FALSE);
	m_D3DDev->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
	m_D3DDev->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE);
	m_D3DDev->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
	m_D3DDev->SetRenderState(D3DRS_BLENDOP, D3DBLENDOP_ADD);
	m_D3DDev->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
	m_D3DDev->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
	m_D3DDev->SetRenderState(D3DRS_CLIPPLANEENABLE, 0);
	m_D3DDev->SetRenderState(D3DRS_FILLMODE, D3DFILL_SOLID);
	m_D3DDev->SetRenderState(D3DRS_LASTPIXEL, FALSE);
	m_D3DDev->SetRenderState(D3DRS_FOGENABLE, FALSE);
	m_D3DDev->SetRenderState(D3DRS_STENCILENABLE, FALSE);
	m_D3DDev->SetRenderState(D3DRS_COLORWRITEENABLE, 0x0000000F);
	m_D3DDev->SetRenderState(D3DRS_SCISSORTESTENABLE, FALSE);
	if( m_State->m_Caps.PrimitiveMiscCaps & D3DPMISCCAPS_SEPARATEALPHABLEND )
		m_D3DDev->SetRenderState(D3DRS_SEPARATEALPHABLENDENABLE, FALSE);
	//if( m_State->m_Caps.LineCaps & D3DLINECAPS_ANTIALIAS )
		m_D3DDev->SetRenderState(D3DRS_ANTIALIASEDLINEENABLE, FALSE);

	m_D3DDev->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_MODULATE);
	m_D3DDev->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
	m_D3DDev->SetTextureStageState(0, D3DTSS_COLORARG2, D3DTA_DIFFUSE);
	m_D3DDev->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
	m_D3DDev->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE);
	m_D3DDev->SetTextureStageState(0, D3DTSS_TEXCOORDINDEX, D3DTSS_TCI_PASSTHRU);
	m_D3DDev->SetTextureStageState(0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
	m_D3DDev->SetSamplerState(0, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP);
	m_D3DDev->SetSamplerState(0, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
	m_D3DDev->SetSamplerState(0, D3DSAMP_MAGFILTER, D3DTEXF_POINT);
	m_D3DDev->SetSamplerState(0, D3DSAMP_MINFILTER, D3DTEXF_POINT);
	m_D3DDev->SetSamplerState(0, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
 
	m_D3DDev->SetVertexShader(NULL);
	m_D3DDev->SetPixelShader(NULL);
}

//	---------------------------------------------------------------------------

void CTwGraphDirect3D9::EndDraw()
{
	assert(m_Drawing==true);
	m_Drawing = false;

	// restore context
	if( !m_PureDevice )
		m_State->Restore();

	/*
	_glPolygonMode(GL_FRONT, m_PrevPolygonMode[0]);
	_glPolygonMode(GL_BACK, m_PrevPolygonMode[1]);
	_glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, m_PrevTexEnv);
	_glLineWidth(m_PrevLineWidth);
	_glMatrixMode(GL_PROJECTION);
	_glPopMatrix();
	_glMatrixMode(GL_MODELVIEW);
	_glPopMatrix();
	_glMatrixMode(GL_TEXTURE);
	_glPopMatrix();
	_glPopClientAttrib();
	_glPopAttrib();
	*/
}

//	---------------------------------------------------------------------------

bool CTwGraphDirect3D9::IsDrawing()
{
	return m_Drawing;
}

//	---------------------------------------------------------------------------

void CTwGraphDirect3D9::Restore()
{

	UnbindFont(m_D3DDev, m_FontD3DTex);
	m_FontD3DTex = NULL;
	
	m_FontTex = NULL;
}


//	---------------------------------------------------------------------------

void CTwGraphDirect3D9::DrawLine(int _X0, int _Y0, int _X1, int _Y1, color32 _Color0, color32 _Color1, bool _AntiAliased)
{
	assert(m_Drawing==true);

	struct CVtx
	{
		float m_Pos[4];
		DWORD m_Color;
	};
	CVtx p[2];

	p[0].m_Pos[0] = (float)_X0;
	p[0].m_Pos[1] = (float)_Y0;
	p[0].m_Pos[2] = 0;
	p[0].m_Pos[3] = 0;
	p[0].m_Color  = _Color0;

	p[1].m_Pos[0] = (float)_X1;
	p[1].m_Pos[1] = (float)_Y1;
	p[1].m_Pos[2] = 0;
	p[1].m_Pos[3] = 0;
	p[1].m_Color  = _Color1;

	//if( m_State->m_Caps.LineCaps & D3DLINECAPS_ANTIALIAS )
		m_D3DDev->SetRenderState(D3DRS_ANTIALIASEDLINEENABLE, (_AntiAliased ? TRUE : FALSE));
	m_D3DDev->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_DISABLE);
	m_D3DDev->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_DISABLE);
	m_D3DDev->SetFVF(D3DFVF_XYZRHW|D3DFVF_DIFFUSE);
	m_D3DDev->DrawPrimitiveUP(D3DPT_LINELIST, 1, p, sizeof(CVtx));
	//if( m_State->m_Caps.LineCaps & D3DLINECAPS_ANTIALIAS )
		m_D3DDev->SetRenderState(D3DRS_ANTIALIASEDLINEENABLE, FALSE);
}

//	---------------------------------------------------------------------------

void CTwGraphDirect3D9::DrawRect(int _X0, int _Y0, int _X1, int _Y1, color32 _Color00, color32 _Color10, color32 _Color01, color32 _Color11)
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

	struct CVtx
	{
		float m_Pos[4];
		DWORD m_Color;
	};
	CVtx p[4];

	p[0].m_Pos[0] = (float)_X1;
	p[0].m_Pos[1] = (float)_Y0;
	p[0].m_Pos[2] = 0;
	p[0].m_Pos[3] = 1;
	p[0].m_Color  = _Color10;

	p[1].m_Pos[0] = (float)_X0;
	p[1].m_Pos[1] = (float)_Y0;
	p[1].m_Pos[2] = 0;
	p[1].m_Pos[3] = 1;
	p[1].m_Color  = _Color00;

	p[2].m_Pos[0] = (float)_X1;
	p[2].m_Pos[1] = (float)_Y1;
	p[2].m_Pos[2] = 0;
	p[2].m_Pos[3] = 1;
	p[2].m_Color  = _Color11;

	p[3].m_Pos[0] = (float)_X0;
	p[3].m_Pos[1] = (float)_Y1;
	p[3].m_Pos[2] = 0;
	p[3].m_Pos[3] = 1;
	p[3].m_Color  = _Color01;

	m_D3DDev->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_DISABLE);
	m_D3DDev->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_DISABLE);
	m_D3DDev->SetFVF(D3DFVF_XYZRHW|D3DFVF_DIFFUSE);
	m_D3DDev->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2, p, sizeof(CVtx));
}

//	---------------------------------------------------------------------------

void *CTwGraphDirect3D9::NewTextObj()
{
	return new CTextObj;
}

//	---------------------------------------------------------------------------

void CTwGraphDirect3D9::DeleteTextObj(void *_TextObj)
{
	assert(_TextObj!=NULL);
	delete static_cast<CTextObj *>(_TextObj);
}

//	---------------------------------------------------------------------------

void CTwGraphDirect3D9::BuildText(void *_TextObj, const std::string *_TextLines, color32 *_LineColors, color32 *_LineBgColors, int _NbLines, const CTexFont *_Font, int _Sep, int _BgWidth)
{
	assert(m_Drawing==true);
	assert(_TextObj!=NULL);
	assert(_Font!=NULL);

	if( _Font != m_FontTex )
	{
		UnbindFont(m_D3DDev, m_FontD3DTex);
		m_FontD3DTex = BindFont(m_D3DDev, _Font);
		m_FontTex = _Font;
	}

	CTextObj *TextObj = static_cast<CTextObj *>(_TextObj);
	TextObj->m_TextVerts.resize(0);
	TextObj->m_BgVerts.resize(0);
	TextObj->m_LineColors = (_LineColors!=NULL);
	TextObj->m_LineBgColors = (_LineBgColors!=NULL);

	int x, x1, y, y1, i, Len;
	unsigned char ch;
	const unsigned char *Text;
	color32 LineColor = COLOR32_RED;
	CTextVtx Vtx;
	Vtx.m_Pos[2] = 0;
	Vtx.m_Pos[3] = 1;
	CBgVtx BgVtx;
	BgVtx.m_Pos[2] = 0;
	BgVtx.m_Pos[3] = 1;
	for( int Line=0; Line<_NbLines; ++Line )
	{
		x = 0;
		y = Line * (_Font->m_CharHeight+_Sep);
		y1 = y+_Font->m_CharHeight;
		Len = (int)_TextLines[Line].length();
		Text = (const unsigned char *)(_TextLines[Line].c_str());
		if( _LineColors!=NULL )
			LineColor = _LineColors[Line];

		for( i=0; i<Len; ++i )
		{
			ch = Text[i];
			x1 = x + _Font->m_CharWidth[ch];

			Vtx.m_Color  = LineColor;

			Vtx.m_Pos[0] = (float)x;
			Vtx.m_Pos[1] = (float)y;
			Vtx.m_UV [0] = _Font->m_CharU0[ch];
			Vtx.m_UV [1] = _Font->m_CharV0[ch];
			TextObj->m_TextVerts.push_back(Vtx);

			Vtx.m_Pos[0] = (float)x1;
			Vtx.m_Pos[1] = (float)y;
			Vtx.m_UV [0] = _Font->m_CharU1[ch];
			Vtx.m_UV [1] = _Font->m_CharV0[ch];
			TextObj->m_TextVerts.push_back(Vtx);

			Vtx.m_Pos[0] = (float)x;
			Vtx.m_Pos[1] = (float)y1;
			Vtx.m_UV [0] = _Font->m_CharU0[ch];
			Vtx.m_UV [1] = _Font->m_CharV1[ch];
			TextObj->m_TextVerts.push_back(Vtx);

			Vtx.m_Pos[0] = (float)x1;
			Vtx.m_Pos[1] = (float)y;
			Vtx.m_UV [0] = _Font->m_CharU1[ch];
			Vtx.m_UV [1] = _Font->m_CharV0[ch];
			TextObj->m_TextVerts.push_back(Vtx);

			Vtx.m_Pos[0] = (float)x1;
			Vtx.m_Pos[1] = (float)y1;
			Vtx.m_UV [0] = _Font->m_CharU1[ch];
			Vtx.m_UV [1] = _Font->m_CharV1[ch];
			TextObj->m_TextVerts.push_back(Vtx);

			Vtx.m_Pos[0] = (float)x;
			Vtx.m_Pos[1] = (float)y1;
			Vtx.m_UV [0] = _Font->m_CharU0[ch];
			Vtx.m_UV [1] = _Font->m_CharV1[ch];
			TextObj->m_TextVerts.push_back(Vtx);

			x = x1;
		}
		if( _BgWidth>0 && Len>0 )
		{
			if( _LineBgColors!=NULL )
				BgVtx.m_Color = _LineBgColors[Line];
			else
				BgVtx.m_Color  = COLOR32_BLACK;

			BgVtx.m_Pos[0] = -1;
			BgVtx.m_Pos[1] = (float)y;
			TextObj->m_BgVerts.push_back(BgVtx);

			BgVtx.m_Pos[0] = (float)(_BgWidth+1);
			BgVtx.m_Pos[1] = (float)y;
			TextObj->m_BgVerts.push_back(BgVtx);

			BgVtx.m_Pos[0] = -1;
			BgVtx.m_Pos[1] = (float)y1;
			TextObj->m_BgVerts.push_back(BgVtx);

			BgVtx.m_Pos[0] = (float)(_BgWidth+1);
			BgVtx.m_Pos[1] = (float)y;
			TextObj->m_BgVerts.push_back(BgVtx);

			BgVtx.m_Pos[0] = (float)(_BgWidth+1);
			BgVtx.m_Pos[1] = (float)y1;
			TextObj->m_BgVerts.push_back(BgVtx);

			BgVtx.m_Pos[0] = -1;
			BgVtx.m_Pos[1] = (float)y1;
			TextObj->m_BgVerts.push_back(BgVtx);
		}
	}

}

//	---------------------------------------------------------------------------

void CTwGraphDirect3D9::DrawText(void *_TextObj, int _X, int _Y, color32 _Color, color32 _BgColor)
{
	assert(m_Drawing==true);
	assert(_TextObj!=NULL);
	CTextObj *TextObj = static_cast<CTextObj *>(_TextObj);
	float x = (float)_X;
	float y = (float)_Y;

	int nv = (int)TextObj->m_TextVerts.size();
	if( nv<4 )
		return;	// no character to draw

	int i;
	int nb = (int)TextObj->m_BgVerts.size();

	if( nb>=4 )
	{
		for( i=0; i<nb; ++i )
		{
			TextObj->m_BgVerts[i].m_Pos[0] += x;
			TextObj->m_BgVerts[i].m_Pos[1] += y;
			if( _BgColor!=0 || !TextObj->m_LineBgColors )
				TextObj->m_BgVerts[i].m_Color = _BgColor;
		}

		m_D3DDev->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_DISABLE);
		m_D3DDev->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_DISABLE);
		m_D3DDev->SetFVF(D3DFVF_XYZRHW|D3DFVF_DIFFUSE);
		m_D3DDev->DrawPrimitiveUP(D3DPT_TRIANGLELIST, nb/3, &(TextObj->m_BgVerts[0]), sizeof(CBgVtx));

		for( i=0; i<nb; ++i )
		{
			TextObj->m_BgVerts[i].m_Pos[0] -= x;
			TextObj->m_BgVerts[i].m_Pos[1] -= y;
		}
	}

	for( i=0; i<nv; ++i )
	{
		TextObj->m_TextVerts[i].m_Pos[0] += x;
		TextObj->m_TextVerts[i].m_Pos[1] += y;
	}
	if( _Color!=0 || !TextObj->m_LineColors )
		for( i=0; i<nv; ++i )
			TextObj->m_TextVerts[i].m_Color = _Color;

	m_D3DDev->SetTexture(0, m_FontD3DTex);
	m_D3DDev->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_MODULATE);
	m_D3DDev->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
	m_D3DDev->SetFVF(D3DFVF_XYZRHW|D3DFVF_DIFFUSE|D3DFVF_TEX1|D3DFVF_TEXCOORDSIZE2(0));
	m_D3DDev->DrawPrimitiveUP(D3DPT_TRIANGLELIST, nv/3, &(TextObj->m_TextVerts[0]), sizeof(CTextVtx));

	for( i=0; i<nv; ++i )
	{
		TextObj->m_TextVerts[i].m_Pos[0] -= x;
		TextObj->m_TextVerts[i].m_Pos[1] -= y;
	}

}

//	---------------------------------------------------------------------------
