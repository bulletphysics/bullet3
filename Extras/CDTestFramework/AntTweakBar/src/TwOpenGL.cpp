//	---------------------------------------------------------------------------
//
//	@file		TwOpenGL.cpp
//	@author		Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//				Copyright © 2005, 2006 Philippe Decaudin.
//              For conditions of distribution and use, see License.txt
//
//	note:		TAB=4
//
//	---------------------------------------------------------------------------


#include "TwPrecomp.h"
#include "LoadOGL.h"
#include "TwOpenGL.h"
#include "TwMgr.h"

using namespace std;

const char *g_ErrCantLoadOGL	= "Cannot load OpenGL library dynamically";
const char *g_ErrCantUnloadOGL	= "Cannot unload OpenGL library";

GLuint g_SmallFontTexID = 0;
GLuint g_NormalFontTexID = 0;
GLuint g_LargeFontTexID = 0;

//	---------------------------------------------------------------------------
//	Extensions

typedef void (APIENTRY * PFNGLBindBufferARB)(GLenum target, GLuint buffer);
typedef void (APIENTRY * PFNGLBindProgramARB)(GLenum target, GLuint program);
typedef GLuint (APIENTRY * PFNGLGetHandleARB)(GLenum pname);
typedef void (APIENTRY * PFNGLUseProgramObjectARB)(GLuint programObj);
PFNGLBindBufferARB _glBindBufferARB = NULL;
PFNGLBindProgramARB _glBindProgramARB = NULL;
PFNGLGetHandleARB _glGetHandleARB = NULL;
PFNGLUseProgramObjectARB _glUseProgramObjectARB = NULL;
#ifndef GL_ARRAY_BUFFER_ARB
#	define GL_ARRAY_BUFFER_ARB 0x8892
#endif
#ifndef GL_ELEMENT_ARRAY_BUFFER_ARB
#	define GL_ELEMENT_ARRAY_BUFFER_ARB 0x8893
#endif
#ifndef GL_ARRAY_BUFFER_BINDING_ARB
#	define GL_ARRAY_BUFFER_BINDING_ARB 0x8894
#endif
#ifndef GL_ELEMENT_ARRAY_BUFFER_BINDING_ARB
#	define GL_ELEMENT_ARRAY_BUFFER_BINDING_ARB 0x8895
#endif
#ifndef GL_VERTEX_PROGRAM_ARB
#	define GL_VERTEX_PROGRAM_ARB 0x8620
#endif
#ifndef GL_FRAGMENT_PROGRAM_ARB
#	define GL_FRAGMENT_PROGRAM_ARB 0x8804
#endif
#ifndef GL_PROGRAM_OBJECT_ARB
#	define GL_PROGRAM_OBJECT_ARB 0x8B40
#endif

//	---------------------------------------------------------------------------

static GLuint BindFont(const CTexFont *_Font)
{
	GLuint TexID = 0;
	_glGenTextures(1, &TexID);
	_glBindTexture(GL_TEXTURE_2D, TexID);
	_glPixelStorei(GL_UNPACK_SWAP_BYTES, GL_FALSE);
	_glPixelStorei(GL_UNPACK_LSB_FIRST, GL_FALSE);
	_glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	_glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
	_glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
	_glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	_glPixelTransferf(GL_ALPHA_SCALE, 1);
	_glPixelTransferf(GL_ALPHA_BIAS, 0);
	_glPixelTransferf(GL_RED_BIAS, 1);
	_glPixelTransferf(GL_GREEN_BIAS, 1);
	_glPixelTransferf(GL_BLUE_BIAS, 1);
	_glTexImage2D(GL_TEXTURE_2D, 0, 4, _Font->m_TexWidth, _Font->m_TexHeight, 0, GL_ALPHA, GL_UNSIGNED_BYTE, _Font->m_TexBytes);
	_glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    _glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    _glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    _glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_NEAREST);
	_glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	_glBindTexture(GL_TEXTURE_2D, 0);
	_glPixelTransferf(GL_ALPHA_BIAS, 0);
	_glPixelTransferf(GL_RED_BIAS, 0);
	_glPixelTransferf(GL_GREEN_BIAS, 0);
	_glPixelTransferf(GL_BLUE_BIAS, 0);

	return TexID;
}

static void UnbindFont(GLuint _FontTexID)
{
	if( _FontTexID>0 )
		_glDeleteTextures(1, &_FontTexID);
}

//	---------------------------------------------------------------------------

int CTwGraphOpenGL::Init()
{
	if( LoadOpenGL()==0 )
	{
		g_TwMgr->SetLastError(g_ErrCantLoadOGL);
		return 0;
	}

	m_Drawing = false;
	m_FontTexID = 0;
	m_FontTex = NULL;
	m_MaxClipPlanes = -1;

	// Get extensions
	_glBindBufferARB = reinterpret_cast<PFNGLBindBufferARB>(_glGetProcAddress("glBindBufferARB"));
	_glBindProgramARB = reinterpret_cast<PFNGLBindProgramARB>(_glGetProcAddress("glBindProgramARB"));
	_glGetHandleARB = reinterpret_cast<PFNGLGetHandleARB>(_glGetProcAddress("glGetHandleARB"));
	_glUseProgramObjectARB = reinterpret_cast<PFNGLUseProgramObjectARB>(_glGetProcAddress("glUseProgramObjectARB"));

	return 1;
}

//	---------------------------------------------------------------------------

int CTwGraphOpenGL::Shut()
{
	assert(m_Drawing==false);

	UnbindFont(m_FontTexID);

	int Res = 1;
	if( UnloadOpenGL()==0 )
	{
		g_TwMgr->SetLastError(g_ErrCantUnloadOGL);
		Res = 0;
	}

	return Res;
}

//	---------------------------------------------------------------------------

void CTwGraphOpenGL::BeginDraw(int _WndWidth, int _WndHeight)
{
	assert(m_Drawing==false && _WndWidth>0 && _WndHeight>0);
	m_Drawing = true;

#ifdef _DEBUG
	{ 
		int err=0;
		char msg[128];
		while( (err=_glGetError())!=0 )
		{
			sprintf(msg, "Before BeginDraw GL_ERROR=0x%x\n", err);
			#ifdef ANT_WINDOWS
				OutputDebugString(msg);
			#endif
			fprintf(stderr, msg);
		}
	}
#endif

	_glPushAttrib(GL_ALL_ATTRIB_BITS);
	_glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);
	_glMatrixMode(GL_TEXTURE);
	_glPushMatrix();
	_glLoadIdentity();
	_glMatrixMode(GL_MODELVIEW);
	_glPushMatrix();
	_glLoadIdentity();
	_glMatrixMode(GL_PROJECTION);
	_glPushMatrix();
	GLint Vp[4];
	_glGetIntegerv(GL_VIEWPORT, Vp);
	/*
	if( _WndWidth>0 && _WndHeight>0 )
	{
		Vp[0] = 0;
		Vp[1] = 0;
		Vp[2] = _WndWidth;
		Vp[3] = _WndHeight;
		_glViewport(Vp[0], Vp[1], Vp[2], Vp[3]);
	}
	_glLoadIdentity();
	//_glOrtho(Vp[0], Vp[0]+Vp[2]-1, Vp[1]+Vp[3]-1, Vp[1], -1, 1); // Doesn't work
	_glOrtho(Vp[0], Vp[0]+Vp[2], Vp[1]+Vp[3], Vp[1], -1, 1);
	*/
	if( _WndWidth>0 && _WndHeight>0 )
	{
		Vp[0] = 0;
		Vp[1] = 0;
		Vp[2] = _WndWidth-1;
		Vp[3] = _WndHeight-1;
		_glViewport(Vp[0], Vp[1], Vp[2], Vp[3]);
	}
	_glLoadIdentity();
	_glOrtho(Vp[0], Vp[0]+Vp[2], Vp[1]+Vp[3], Vp[1], -1, 1);

	_glGetFloatv(GL_LINE_WIDTH, &m_PrevLineWidth);
	_glDisable(GL_POLYGON_STIPPLE);
	_glLineWidth(1);
	_glDisable(GL_LINE_SMOOTH);
	_glDisable(GL_LINE_STIPPLE);
	_glDisable(GL_CULL_FACE);
	_glDisable(GL_DEPTH_TEST);
	_glDisable(GL_LIGHTING);
	_glEnable(GL_BLEND);
	_glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	_glGetTexEnviv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, &m_PrevTexEnv);
	_glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	_glGetIntegerv(GL_POLYGON_MODE, m_PrevPolygonMode);
	_glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	_glDisable(GL_ALPHA_TEST);
	//_glEnable(GL_ALPHA_TEST);
	//_glAlphaFunc(GL_GREATER, 0);
	_glDisable(GL_FOG);
	_glDisable(GL_LOGIC_OP);
	_glDisable(GL_SCISSOR_TEST);
	if( m_MaxClipPlanes<0 )
	{
		_glGetIntegerv(GL_MAX_CLIP_PLANES, &m_MaxClipPlanes);
		if( m_MaxClipPlanes<0 || m_MaxClipPlanes>255 )
			m_MaxClipPlanes = 6;
	}
	for( int i=0; i<m_MaxClipPlanes; ++i )
		_glDisable(GL_CLIP_PLANE0+i);
	m_PrevTexture = 0;
	_glGetIntegerv(GL_TEXTURE_BINDING_2D, &m_PrevTexture);

	_glDisableClientState(GL_VERTEX_ARRAY);
	_glDisableClientState(GL_NORMAL_ARRAY);
	_glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	_glDisableClientState(GL_INDEX_ARRAY);
	_glDisableClientState(GL_COLOR_ARRAY);
	_glDisableClientState(GL_EDGE_FLAG_ARRAY);

	if( _glBindBufferARB!=NULL )
	{
		m_PrevArrayBufferARB = m_PrevElementArrayBufferARB = 0;
		_glGetIntegerv(GL_ARRAY_BUFFER_BINDING_ARB, &m_PrevArrayBufferARB);
		_glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING_ARB, &m_PrevElementArrayBufferARB);
		_glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
		_glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
	}
	if( _glBindProgramARB!=NULL )
	{
		m_PrevVertexProgramARB = _glIsEnabled(GL_VERTEX_PROGRAM_ARB);
		m_PrevFragmentProgramARB = _glIsEnabled(GL_FRAGMENT_PROGRAM_ARB);
		_glDisable(GL_VERTEX_PROGRAM_ARB);
		_glDisable(GL_FRAGMENT_PROGRAM_ARB);
	}
	if( _glGetHandleARB!=NULL && _glUseProgramObjectARB!=NULL )
	{
		m_PrevProgramObjectARB = _glGetHandleARB(GL_PROGRAM_OBJECT_ARB);
		_glUseProgramObjectARB(0);
	}
}

//	---------------------------------------------------------------------------

void CTwGraphOpenGL::EndDraw()
{
	assert(m_Drawing==true);
	m_Drawing = false;

	_glBindTexture(GL_TEXTURE_2D, m_PrevTexture);
	if( _glBindBufferARB!=NULL )
	{
		_glBindBufferARB(GL_ARRAY_BUFFER_ARB, m_PrevArrayBufferARB);
		_glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, m_PrevElementArrayBufferARB);
	}
	if( _glBindProgramARB!=NULL )
	{
		if( m_PrevVertexProgramARB )
			_glEnable(GL_VERTEX_PROGRAM_ARB);
		if( m_PrevFragmentProgramARB )
			_glEnable(GL_FRAGMENT_PROGRAM_ARB);
	}
	if( _glGetHandleARB!=NULL && _glUseProgramObjectARB!=NULL )
		_glUseProgramObjectARB(m_PrevProgramObjectARB);
	
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

#ifdef _DEBUG
	{ 
		int err=0;
		char msg[128];
		while( (err=_glGetError())!=0 )
		{
			sprintf(msg, "After EndDraw GL_ERROR=0x%x\n", err);
			#ifdef ANT_WINDOWS
				OutputDebugString(msg);
			#endif
			fprintf(stderr, msg);
		}
	}
#endif
}

//	---------------------------------------------------------------------------

bool CTwGraphOpenGL::IsDrawing()
{
	return m_Drawing;
}

//	---------------------------------------------------------------------------

void CTwGraphOpenGL::Restore()
{
	UnbindFont(m_FontTexID);
	m_FontTexID = 0;
	m_FontTex = NULL;
}


//	---------------------------------------------------------------------------

void CTwGraphOpenGL::DrawLine(int _X0, int _Y0, int _X1, int _Y1, color32 _Color0, color32 _Color1, bool _AntiAliased)
{
	assert(m_Drawing==true);
	/* 
	// border adjustment NO!!
	if(_X0<_X1)
		++_X1;
	else if(_X0>_X1)
		++_X0;
	if(_Y0<_Y1)
		++_Y1;
	else if(_Y0>_Y1)
		++_Y0;
	*/
	const GLfloat dx = +0.0f;
	//GLfloat dy = -0.2f;
	const GLfloat dy = -0.5f;
	if( _AntiAliased )
		_glEnable(GL_LINE_SMOOTH);
	else
		_glDisable(GL_LINE_SMOOTH);
	_glDisable(GL_TEXTURE_2D);
	_glMatrixMode(GL_MODELVIEW);
	_glLoadIdentity();
	_glBegin(GL_LINES);
		_glColor4ub(GLubyte(_Color0>>16), GLubyte(_Color0>>8), GLubyte(_Color0), GLubyte(_Color0>>24));
		_glVertex2f((GLfloat)_X0+dx, (GLfloat)_Y0+dy);
		_glColor4ub(GLubyte(_Color1>>16), GLubyte(_Color1>>8), GLubyte(_Color1), GLubyte(_Color1>>24));
		_glVertex2f((GLfloat)_X1+dx, (GLfloat)_Y1+dy);
		//_glVertex2i(_X0, _Y0);
		//_glVertex2i(_X1, _Y1);
	_glEnd();
	_glDisable(GL_LINE_SMOOTH);
}
  
//	---------------------------------------------------------------------------

void CTwGraphOpenGL::DrawRect(int _X0, int _Y0, int _X1, int _Y1, color32 _Color00, color32 _Color10, color32 _Color01, color32 _Color11)
{
	assert(m_Drawing==true);

	/*
	// border adjustment
	if(_X0<_X1)
		++_X1;
	else if(_X0>_X1)
		++_X0;
	if(_Y0<_Y1)
		++_Y1;
	else if(_Y0>_Y1)
		++_Y0;
	*/
	// border adjustment
	if(_X0<_X1)
		++_X1;
	else if(_X0>_X1)
		++_X0;
	if(_Y0<_Y1)
		--_Y0;
	else if(_Y0>_Y1)
		--_Y1;
	const GLfloat dx = +0.0f;
	const GLfloat dy = +0.0f;

	_glDisable(GL_TEXTURE_2D);
	_glMatrixMode(GL_MODELVIEW);
	_glLoadIdentity();
	//GLubyte r = GLubyte(_Color>>16);
	//GLubyte g = GLubyte(_Color>>8);
	//GLubyte b = GLubyte(_Color);
	//GLubyte a = GLubyte(_Color>>24);
	//_glColor4ub(GLubyte(_Color>>16), GLubyte(_Color>>8), GLubyte(_Color), GLubyte(_Color>>24));
	//_glColor4ub(r, g, b, a);
	_glBegin(GL_QUADS);
		_glColor4ub(GLubyte(_Color00>>16), GLubyte(_Color00>>8), GLubyte(_Color00), GLubyte(_Color00>>24));
		_glVertex2f((GLfloat)_X0+dx, (GLfloat)_Y0+dy);
		_glColor4ub(GLubyte(_Color10>>16), GLubyte(_Color10>>8), GLubyte(_Color10), GLubyte(_Color10>>24));
		_glVertex2f((GLfloat)_X1+dx, (GLfloat)_Y0+dy);
		_glColor4ub(GLubyte(_Color11>>16), GLubyte(_Color11>>8), GLubyte(_Color11), GLubyte(_Color11>>24));
		_glVertex2f((GLfloat)_X1+dx, (GLfloat)_Y1+dy);
		_glColor4ub(GLubyte(_Color01>>16), GLubyte(_Color01>>8), GLubyte(_Color01), GLubyte(_Color01>>24));
		_glVertex2f((GLfloat)_X0+dx, (GLfloat)_Y1+dy);
	_glEnd();
}

//	---------------------------------------------------------------------------

void *CTwGraphOpenGL::NewTextObj()
{
	return new CTextObj;
}

//	---------------------------------------------------------------------------

void CTwGraphOpenGL::DeleteTextObj(void *_TextObj)
{
	assert(_TextObj!=NULL);
	delete static_cast<CTextObj *>(_TextObj);
}

//	---------------------------------------------------------------------------

void CTwGraphOpenGL::BuildText(void *_TextObj, const std::string *_TextLines, color32 *_LineColors, color32 *_LineBgColors, int _NbLines, const CTexFont *_Font, int _Sep, int _BgWidth)
{
	assert(m_Drawing==true);
	assert(_TextObj!=NULL);
	assert(_Font!=NULL);

	if( _Font != m_FontTex )
	{
		UnbindFont(m_FontTexID);
		m_FontTexID = BindFont(_Font);
		m_FontTex = _Font;
	}
	CTextObj *TextObj = static_cast<CTextObj *>(_TextObj);
	TextObj->m_TextVerts.resize(0);
	TextObj->m_TextUVs.resize(0);
	TextObj->m_BgVerts.resize(0);
	TextObj->m_Colors.resize(0);
	TextObj->m_BgColors.resize(0);

	int x, x1, y, y1, i, Len;
	unsigned char ch;
	const unsigned char *Text;
	color32 LineColor = COLOR32_RED;
	for( int Line=0; Line<_NbLines; ++Line )
	{
		x = 0;
		y = Line * (_Font->m_CharHeight+_Sep);
		y1 = y+_Font->m_CharHeight;
		Len = (int)_TextLines[Line].length();
		Text = (const unsigned char *)(_TextLines[Line].c_str());
		if( _LineColors!=NULL )
			LineColor = (_LineColors[Line]&0xff00ff00) | GLubyte(_LineColors[Line]>>16) | (GLubyte(_LineColors[Line])<<16);

		for( i=0; i<Len; ++i )
		{
			ch = Text[i];
			x1 = x + _Font->m_CharWidth[ch];

			TextObj->m_TextVerts.push_back(Vec2(x , y ));
			TextObj->m_TextVerts.push_back(Vec2(x1, y ));
			TextObj->m_TextVerts.push_back(Vec2(x , y1));
			TextObj->m_TextVerts.push_back(Vec2(x1, y ));
			TextObj->m_TextVerts.push_back(Vec2(x1, y1));
			TextObj->m_TextVerts.push_back(Vec2(x , y1));

			TextObj->m_TextUVs.push_back(Vec2(_Font->m_CharU0[ch], _Font->m_CharV0[ch]));
			TextObj->m_TextUVs.push_back(Vec2(_Font->m_CharU1[ch], _Font->m_CharV0[ch]));
			TextObj->m_TextUVs.push_back(Vec2(_Font->m_CharU0[ch], _Font->m_CharV1[ch]));
			TextObj->m_TextUVs.push_back(Vec2(_Font->m_CharU1[ch], _Font->m_CharV0[ch]));
			TextObj->m_TextUVs.push_back(Vec2(_Font->m_CharU1[ch], _Font->m_CharV1[ch]));
			TextObj->m_TextUVs.push_back(Vec2(_Font->m_CharU0[ch], _Font->m_CharV1[ch]));

			if( _LineColors!=NULL )
			{
				TextObj->m_Colors.push_back(LineColor);
				TextObj->m_Colors.push_back(LineColor);
				TextObj->m_Colors.push_back(LineColor);
				TextObj->m_Colors.push_back(LineColor);
				TextObj->m_Colors.push_back(LineColor);
				TextObj->m_Colors.push_back(LineColor);
			}

			x = x1;
		}
		if( _BgWidth>0 && Len>0 )
		{
			TextObj->m_BgVerts.push_back(Vec2(-1        , y ));
			TextObj->m_BgVerts.push_back(Vec2(_BgWidth+1, y ));
			TextObj->m_BgVerts.push_back(Vec2(-1        , y1));
			TextObj->m_BgVerts.push_back(Vec2(_BgWidth+1, y ));
			TextObj->m_BgVerts.push_back(Vec2(_BgWidth+1, y1));
			TextObj->m_BgVerts.push_back(Vec2(-1        , y1));

			if( _LineBgColors!=NULL )
			{
				color32 LineBgColor = (_LineBgColors[Line]&0xff00ff00) | GLubyte(_LineBgColors[Line]>>16) | (GLubyte(_LineBgColors[Line])<<16);
				TextObj->m_BgColors.push_back(LineBgColor);
				TextObj->m_BgColors.push_back(LineBgColor);
				TextObj->m_BgColors.push_back(LineBgColor);
				TextObj->m_BgColors.push_back(LineBgColor);
				TextObj->m_BgColors.push_back(LineBgColor);
				TextObj->m_BgColors.push_back(LineBgColor);
			}
		}
	}
}

//	---------------------------------------------------------------------------

void CTwGraphOpenGL::DrawText(void *_TextObj, int _X, int _Y, color32 _Color, color32 _BgColor)
{
	assert(m_Drawing==true);
	assert(_TextObj!=NULL);
	CTextObj *TextObj = static_cast<CTextObj *>(_TextObj);

	if( TextObj->m_TextVerts.size()<4 )
		return;	// no character to draw

	_glMatrixMode(GL_MODELVIEW);
	_glLoadIdentity();
	_glTranslatef((GLfloat)_X, (GLfloat)_Y, 0);
	_glEnableClientState(GL_VERTEX_ARRAY);
	if( (_BgColor!=0 || TextObj->m_BgColors.size()==TextObj->m_BgVerts.size()) && TextObj->m_BgVerts.size()>=4 )
	{
		_glDisable(GL_TEXTURE_2D);
		_glVertexPointer(2, GL_FLOAT, 0, &(TextObj->m_BgVerts[0]));
		if( TextObj->m_BgColors.size()==TextObj->m_BgVerts.size() && _BgColor==0 )
		{
			_glEnableClientState(GL_COLOR_ARRAY);
			_glColorPointer(4, GL_UNSIGNED_BYTE, 0, &(TextObj->m_BgColors[0]));
		}
		else
		{
			_glDisableClientState(GL_COLOR_ARRAY);
			_glColor4ub(GLubyte(_BgColor>>16), GLubyte(_BgColor>>8), GLubyte(_BgColor), GLubyte(_BgColor>>24));
		}
		_glDrawArrays(GL_TRIANGLES, 0, (int)TextObj->m_BgVerts.size());
	}
	_glEnable(GL_TEXTURE_2D);
	_glBindTexture(GL_TEXTURE_2D, m_FontTexID);
	_glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	_glVertexPointer(2, GL_FLOAT, 0, &(TextObj->m_TextVerts[0]));
	_glTexCoordPointer(2, GL_FLOAT, 0, &(TextObj->m_TextUVs[0]));
	if( TextObj->m_Colors.size()==TextObj->m_TextVerts.size() && _Color==0 )
	{
		_glEnableClientState(GL_COLOR_ARRAY);
		_glColorPointer(4, GL_UNSIGNED_BYTE, 0, &(TextObj->m_Colors[0]));
	}
	else
	{
		_glDisableClientState(GL_COLOR_ARRAY);
		_glColor4ub(GLubyte(_Color>>16), GLubyte(_Color>>8), GLubyte(_Color), GLubyte(_Color>>24));
	}

	_glDrawArrays(GL_TRIANGLES, 0, (int)TextObj->m_TextVerts.size());
	
	_glDisableClientState(GL_VERTEX_ARRAY);
	_glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	_glDisableClientState(GL_COLOR_ARRAY);
}

//	---------------------------------------------------------------------------
