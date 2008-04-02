//	---------------------------------------------------------------------------
//
//	@file		TwGraph.h
//	@brief		ITwGraph pure interface
//	@author		Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//				Copyright © 2005, 2006 Philippe Decaudin.
//              For conditions of distribution and use, see License.txt
//
//	notes:		Private header
//				TAB=4
//
//	---------------------------------------------------------------------------


#if !defined ANT_TW_GRAPH_INCLUDED
#define ANT_TW_GRAPH_INCLUDED

#include "TwColors.h"
#include "TwFonts.h"


//	---------------------------------------------------------------------------

#ifdef DrawText		// DirectX redefines 'DrawText' !!
#	undef DrawText
#endif	// DrawText

class ITwGraph
{
public:
	virtual int			Init() = 0;
	virtual int			Shut() = 0;
	virtual void		BeginDraw(int _WndWidth, int _WndHeight) = 0;
	virtual void		EndDraw() = 0;
	virtual bool		IsDrawing() = 0;
	virtual void		Restore() = 0;
	virtual void		DrawLine(int _X0, int _Y0, int _X1, int _Y1, color32 _Color0, color32 _Color1, bool _AntiAliased=false) = 0;
	virtual void		DrawLine(int _X0, int _Y0, int _X1, int _Y1, color32 _Color, bool _AntiAliased=false) = 0;

	virtual void		DrawRect(int _X0, int _Y0, int _X1, int _Y1, color32 _Color00, color32 _Color10, color32 _Color01, color32 _Color11) = 0;
	virtual void		DrawRect(int _X0, int _Y0, int _X1, int _Y1, color32 _Color) = 0;

	virtual void *		NewTextObj() = 0;
	virtual void		DeleteTextObj(void *_TextObj) = 0;
	virtual void		BuildText(void *_TextObj, const std::string *_TextLines, color32 *_LineColors, color32 *_LineBgColors, int _NbLines, const CTexFont *_Font, int _Sep, int _BgWidth) = 0;
	virtual void		DrawText(void *_TextObj, int _X, int _Y, color32 _Color, color32 _BgColor) = 0;

	virtual 			~ITwGraph() {}	// required by gcc
};

//	---------------------------------------------------------------------------

#endif	// ANT_TW_GRAPH_INCLUDED
