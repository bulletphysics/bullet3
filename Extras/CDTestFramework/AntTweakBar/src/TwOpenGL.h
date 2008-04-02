//	---------------------------------------------------------------------------
//
//	@file		TwOpenGL.h
//	@brief		OpenGL graph functions
//	@author		Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//				Copyright © 2005, 2006 Philippe Decaudin.
//              For conditions of distribution and use, see License.txt
//
//	notes:		Private header
//				TAB=4
//
//	---------------------------------------------------------------------------


#if !defined ANT_TW_OPENGL_INCLUDED
#define ANT_TW_OPENGL_INCLUDED

#include "TwGraph.h"

//	---------------------------------------------------------------------------

class CTwGraphOpenGL : public ITwGraph
{
public:
	virtual int			Init();
	virtual int			Shut();
	virtual void		BeginDraw(int _WndWidth, int _WndHeight);
	virtual void		EndDraw();
	virtual bool		IsDrawing();
	virtual void		Restore();
	virtual void		DrawLine(int _X0, int _Y0, int _X1, int _Y1, color32 _Color0, color32 _Color1, bool _AntiAliased=false);
	virtual void		DrawLine(int _X0, int _Y0, int _X1, int _Y1, color32 _Color, bool _AntiAliased=false) { DrawLine(_X0, _Y0, _X1, _Y1, _Color, _Color, _AntiAliased); }
	virtual void		DrawRect(int _X0, int _Y0, int _X1, int _Y1, color32 _Color00, color32 _Color10, color32 _Color01, color32 _Color11);
	virtual void		DrawRect(int _X0, int _Y0, int _X1, int _Y1, color32 _Color) { DrawRect(_X0, _Y0, _X1, _Y1, _Color, _Color, _Color, _Color); }


	virtual void *		NewTextObj();
	virtual void		DeleteTextObj(void *_TextObj);
	virtual void		BuildText(void *_TextObj, const std::string *_TextLines, color32 *_LineColors, color32 *_LineBgColors, int _NbLines, const CTexFont *_Font, int _Sep, int _BgWidth);
	virtual void		DrawText(void *_TextObj, int _X, int _Y, color32 _Color, color32 _BgColor);

protected:
	bool				m_Drawing;
	GLuint				m_FontTexID;
	const CTexFont *	m_FontTex;
	GLfloat				m_PrevLineWidth;
	GLint				m_PrevTexEnv;
	GLint				m_PrevPolygonMode[2];
	GLint				m_MaxClipPlanes;
	GLint				m_PrevTexture;
	GLint				m_PrevArrayBufferARB;
	GLint				m_PrevElementArrayBufferARB;
	GLboolean			m_PrevVertexProgramARB;
	GLboolean			m_PrevFragmentProgramARB;
	GLuint				m_PrevProgramObjectARB;

	struct Vec2			{ GLfloat x, y; Vec2(){} Vec2(GLfloat _X, GLfloat _Y):x(_X),y(_Y){} Vec2(int _X, int _Y):x(GLfloat(_X)),y(GLfloat(_Y)){} };
	struct CTextObj
	{
		std::vector<Vec2>	m_TextVerts;
		std::vector<Vec2>	m_TextUVs;
		std::vector<Vec2>	m_BgVerts;
		std::vector<color32>m_Colors;
		std::vector<color32>m_BgColors;
	};
};

//	---------------------------------------------------------------------------


#endif // !defined ANT_TW_OPENGL_INCLUDED
