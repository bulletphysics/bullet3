//	---------------------------------------------------------------------------
//
//	@file		TwDirect3D9.h
//	@brief		Direct3D9 graph functions
//	@author		Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//				Copyright © 2005, 2006 Philippe Decaudin.
//              For conditions of distribution and use, see License.txt
//
//	notes:		Private header
//				TAB=4
//
//	---------------------------------------------------------------------------


#if !defined ANT_TW_DIRECT3D9_INCLUDED
#define ANT_TW_DIRECT3D9_INCLUDED

#include "TwGraph.h"

//	---------------------------------------------------------------------------

class CTwGraphDirect3D9 : public ITwGraph
{
public:
	virtual int					Init();
	virtual int					Shut();
	virtual void				BeginDraw(int _WndWidth, int _WndHeight);
	virtual void				EndDraw();
	virtual bool				IsDrawing();
	virtual void				Restore();
	virtual void				DrawLine(int _X0, int _Y0, int _X1, int _Y1, color32 _Color0, color32 _Color1, bool _AntiAliased=false);
	virtual void				DrawLine(int _X0, int _Y0, int _X1, int _Y1, color32 _Color, bool _AntiAliased=false) { DrawLine(_X0, _Y0, _X1, _Y1, _Color, _Color, _AntiAliased); }
	virtual void				DrawRect(int _X0, int _Y0, int _X1, int _Y1, color32 _Color00, color32 _Color10, color32 _Color01, color32 _Color11);
	virtual void				DrawRect(int _X0, int _Y0, int _X1, int _Y1, color32 _Color) { DrawRect(_X0, _Y0, _X1, _Y1, _Color, _Color, _Color, _Color); }

	virtual void *				NewTextObj();
	virtual void				DeleteTextObj(void *_TextObj);
	virtual void				BuildText(void *_TextObj, const std::string *_TextLines, color32 *_LineColors, color32 *_LineBgColors, int _NbLines, const CTexFont *_Font, int _Sep, int _BgWidth);
	virtual void				DrawText(void *_TextObj, int _X, int _Y, color32 _Color, color32 _BgColor);

protected:
	struct IDirect3DDevice9 *	m_D3DDev;
	bool						m_Drawing;
	const CTexFont *			m_FontTex;
	struct IDirect3DTexture9 *	m_FontD3DTex;
	bool						m_PureDevice;
	int							m_WndWidth;
	int							m_WndHeight;

	struct CTextVtx
	{
		float					m_Pos[4];
		color32					m_Color;
		float					m_UV[2];
	};
	struct CBgVtx
	{
		float					m_Pos[4];
		color32					m_Color;
	};

	struct CTextObj
	{
		std::vector<CTextVtx>	m_TextVerts;
		std::vector<CBgVtx>		m_BgVerts;
		bool					m_LineColors;
		bool					m_LineBgColors;
	};

	struct CState *				m_State;
};

//	---------------------------------------------------------------------------


#endif // !defined ANT_TW_DIRECT3D9_INCLUDED
