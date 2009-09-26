/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "GL_DialogWindow.h"
#include "GlutStuff.h"
#include "GLDebugFont.h"
#include "btBulletDynamicsCommon.h"
//  ---------------------------------------------------------------------------
//  Extensions

typedef void (APIENTRY * PFNGLBindBufferARB)(GLenum target, GLuint buffer);
typedef void (APIENTRY * PFNGLBindProgramARB)(GLenum target, GLuint program);
typedef GLuint (APIENTRY * PFNGLGetHandleARB)(GLenum pname);
typedef void (APIENTRY * PFNGLUseProgramObjectARB)(GLuint programObj);
typedef void (APIENTRY * PFNGLTexImage3D)(GLenum target, GLint level, GLint internalformat, GLsizei width, GLsizei height, GLsizei depth, GLint border, GLenum format, GLenum type, const GLvoid *pixels);
typedef void (APIENTRY * PFNGLActiveTextureARB)(GLenum texture);
typedef void (APIENTRY * PFNGLClientActiveTextureARB)(GLenum texture);
typedef void (APIENTRY * PFNGLBlendEquation)(GLenum mode);
typedef void (APIENTRY * PFNGLBlendEquationSeparate)(GLenum srcMode, GLenum dstMode);
typedef void (APIENTRY * PFNGLBlendFuncSeparate)(GLenum srcRGB, GLenum dstRGB, GLenum srcAlpha, GLenum dstAlpha);
PFNGLBindBufferARB glBindBufferARB = NULL;
PFNGLBindProgramARB glBindProgramARB = NULL;
PFNGLGetHandleARB glGetHandleARB = NULL;
PFNGLUseProgramObjectARB glUseProgramObjectARB = NULL;
PFNGLTexImage3D glTexImage3D = NULL;
PFNGLActiveTextureARB glActiveTextureARB = NULL;
PFNGLClientActiveTextureARB glClientActiveTextureARB = NULL;
PFNGLBlendEquation glBlendEquation = NULL;
PFNGLBlendEquationSeparate glBlendEquationSeparate = NULL;
PFNGLBlendFuncSeparate glBlendFuncSeparate = NULL;
#ifndef GL_ARRAY_BUFFER_ARB
#   define GL_ARRAY_BUFFER_ARB 0x8892
#endif
#ifndef GL_ELEMENT_ARRAY_BUFFER_ARB
#   define GL_ELEMENT_ARRAY_BUFFER_ARB 0x8893
#endif
#ifndef GL_ARRAY_BUFFER_BINDING_ARB
#   define GL_ARRAY_BUFFER_BINDING_ARB 0x8894
#endif
#ifndef GL_ELEMENT_ARRAY_BUFFER_BINDING_ARB
#   define GL_ELEMENT_ARRAY_BUFFER_BINDING_ARB 0x8895
#endif
#ifndef GL_VERTEX_PROGRAM_ARB
#   define GL_VERTEX_PROGRAM_ARB 0x8620
#endif
#ifndef GL_FRAGMENT_PROGRAM_ARB
#   define GL_FRAGMENT_PROGRAM_ARB 0x8804
#endif
#ifndef GL_PROGRAM_OBJECT_ARB
#   define GL_PROGRAM_OBJECT_ARB 0x8B40
#endif
#ifndef GL_TEXTURE_3D
#   define GL_TEXTURE_3D 0x806F
#endif
#ifndef GL_TEXTURE0_ARB
#   define GL_TEXTURE0_ARB 0x84C0
#endif
#ifndef GL_ACTIVE_TEXTURE_ARB
#   define GL_ACTIVE_TEXTURE_ARB 0x84E0
#endif
#ifndef GL_MAX_TEXTURE_UNITS_ARB
#   define GL_MAX_TEXTURE_UNITS_ARB 0x84E2
#endif
#ifndef GL_TEXTURE_RECTANGLE_ARB
#   define GL_TEXTURE_RECTANGLE_ARB 0x84F5
#endif
#ifndef GL_FUNC_ADD
#   define GL_FUNC_ADD 0x8006
#endif
#ifndef GL_BLEND_EQUATION
#   define GL_BLEND_EQUATION 0x8009
#endif
#ifndef GL_BLEND_EQUATION_RGB
#   define GL_BLEND_EQUATION_RGB GL_BLEND_EQUATION
#endif
#ifndef GL_BLEND_EQUATION_ALPHA
#   define GL_BLEND_EQUATION_ALPHA 0x883D
#endif
#ifndef GL_BLEND_SRC_RGB
#   define GL_BLEND_SRC_RGB 0x80C9
#endif
#ifndef GL_BLEND_DST_RGB
#   define GL_BLEND_DST_RGB 0x80C8
#endif
#ifndef GL_BLEND_SRC_ALPHA
#   define GL_BLEND_SRC_ALPHA 0x80CB
#endif
#ifndef GL_BLEND_DST_ALPHA
#   define GL_BLEND_DST_ALPHA 0x80CA
#endif


GL_DialogWindow::GL_DialogWindow(int horPos,int vertPos,int dialogWidth,int dialogHeight, btCollisionObject* collisionObject,const char* dialogTitle)
:m_dialogHorPos(horPos),
m_dialogVertPos(vertPos),
m_dialogWidth(dialogWidth),
m_dialogHeight(dialogHeight),
m_screenWidth(0),
m_screenHeight(0),
m_MaxClipPlanes(-1),
m_collisionObject(collisionObject),
m_dialogTitle(dialogTitle)
{
}

void	GL_DialogWindow::setScreenSize(int width, int height)
{
	m_screenWidth = width;
	m_screenHeight = height;
}
GL_DialogWindow::~GL_DialogWindow()
{

}



static void drawLine(int _X0, int _Y0, int _X1, int _Y1, unsigned int _Color0, unsigned int _Color1)
{
    const GLfloat dx = +0.5f;
    const GLfloat dy = -0.5f;

	bool antiAliased = false;
    if( antiAliased )
        glEnable(GL_LINE_SMOOTH);
    else
        glDisable(GL_LINE_SMOOTH);
    glDisable(GL_TEXTURE_2D);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glBegin(GL_LINES);
        glColor4ub(GLubyte(_Color0>>16), GLubyte(_Color0>>8), GLubyte(_Color0), GLubyte(_Color0>>24));
        glVertex2f((GLfloat)_X0+dx, (GLfloat)_Y0+dy);
        glColor4ub(GLubyte(_Color1>>16), GLubyte(_Color1>>8), GLubyte(_Color1), GLubyte(_Color1>>24));
        glVertex2f((GLfloat)_X1+dx, (GLfloat)_Y1+dy);
    glEnd();
    glDisable(GL_LINE_SMOOTH);
}

static void	drawRect(int horStart, int vertStart, int horEnd, int vertEnd, unsigned int argbColor00,unsigned int argbColor10,unsigned int argbColor01,unsigned int argbColor11)
{
	float dx = 0;
	float dy = 0;
	glDisable(GL_TEXTURE_2D);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glBegin(GL_QUADS);
         glColor4ub(GLubyte(argbColor00>>16), GLubyte(argbColor00>>8), GLubyte(argbColor00), GLubyte(argbColor00>>24));
        glVertex2f((GLfloat)horStart+dx, (GLfloat)vertStart+dy);
        glColor4ub(GLubyte(argbColor10>>16), GLubyte(argbColor10>>8), GLubyte(argbColor10), GLubyte(argbColor10>>24));
        glVertex2f((GLfloat)horEnd+dx, (GLfloat)vertStart+dy);
        glColor4ub(GLubyte(argbColor11>>16), GLubyte(argbColor11>>8), GLubyte(argbColor11), GLubyte(argbColor11>>24));
        glVertex2f((GLfloat)horEnd+dx, (GLfloat)vertEnd+dy);
        glColor4ub(GLubyte(argbColor01>>16), GLubyte(argbColor01>>8), GLubyte(argbColor01), GLubyte(argbColor01>>24));
        glVertex2f((GLfloat)horStart+dx, (GLfloat)vertEnd+dy);
    glEnd();
}

void GL_DialogWindow::draw(btScalar deltaTime)
{
	if (!m_screenWidth || !m_screenHeight)
		return;

	m_dialogHorPos = m_collisionObject->getWorldTransform().getOrigin()[0]+m_screenWidth/2-m_dialogWidth/2;
	
	m_dialogVertPos = m_collisionObject->getWorldTransform().getOrigin()[1]+m_screenHeight/2-m_dialogHeight/2;
	saveOpenGLState();

	//drawRect(m_dialogHorPos,m_dialogVertPos,m_dialogHorPos+m_dialogWidth,m_dialogVertPos+m_dialogHeight,0xa6000000);
	unsigned int argbColor = 0x86000000;
	int charHeight = 16;
	int charWidth = 10;

	int titleHeight = charHeight + 2;

	drawRect(m_dialogHorPos,m_dialogVertPos,m_dialogHorPos+m_dialogWidth-1,m_dialogVertPos+titleHeight,argbColor,argbColor,argbColor,argbColor);
	//const unsigned int COL0 = 0x50ffffff;
	const unsigned int COL0 = 0xffffffff;
	const unsigned int COL1 = 0xff1f1f1f;

	drawRect(m_dialogHorPos,m_dialogVertPos,m_dialogHorPos+m_dialogWidth-1,m_dialogVertPos+1,COL0,COL0,COL1,COL1);

	argbColor = 0x864f4f4f;
	drawRect(m_dialogHorPos+1,m_dialogVertPos+titleHeight,m_dialogHorPos+m_dialogWidth-1,m_dialogVertPos+m_dialogHeight,argbColor,argbColor,argbColor,argbColor);
	

	int y = m_dialogVertPos+charHeight+1;
	glLineWidth(3);
	drawLine(m_dialogHorPos, y, m_dialogHorPos+m_dialogWidth-1, y, 0x80afafaf,0x80afafaf);



	unsigned int  clight = 0x5FFFFFFF; // bar contour
    drawLine(m_dialogHorPos, m_dialogVertPos, m_dialogHorPos, m_dialogVertPos+m_dialogHeight, clight,clight);
    drawLine(m_dialogHorPos, m_dialogVertPos, m_dialogHorPos+m_dialogWidth, m_dialogVertPos, clight,clight);
    drawLine(m_dialogHorPos+m_dialogWidth, m_dialogVertPos, m_dialogHorPos+m_dialogWidth, m_dialogVertPos+m_dialogHeight, clight,clight);
    drawLine(m_dialogHorPos, m_dialogVertPos+m_dialogHeight, m_dialogHorPos+m_dialogWidth, m_dialogVertPos+m_dialogHeight, clight,clight);
    int dshad = 3;  // bar shadows
	
    unsigned int  cshad = (((0x40000000>>24)/2)<<24) & 0xFF000000;
    drawRect(m_dialogHorPos, m_dialogVertPos+m_dialogHeight, m_dialogHorPos+dshad, m_dialogVertPos+m_dialogHeight+dshad, 0, cshad, 0, 0);
    drawRect(m_dialogHorPos+dshad+1, m_dialogVertPos+m_dialogHeight, m_dialogHorPos+m_dialogWidth-1, m_dialogVertPos+m_dialogHeight+dshad, cshad, cshad, 0, 0);
    drawRect(m_dialogHorPos+m_dialogWidth, m_dialogVertPos+m_dialogHeight, m_dialogHorPos+m_dialogWidth+dshad, m_dialogVertPos+m_dialogHeight+dshad, cshad, 0, 0, 0);
    drawRect(m_dialogHorPos+m_dialogWidth, m_dialogVertPos, m_dialogHorPos+m_dialogWidth+dshad, m_dialogVertPos+dshad, 0, 0, cshad, 0);
    drawRect(m_dialogHorPos+m_dialogWidth, m_dialogVertPos+dshad+1, m_dialogHorPos+m_dialogWidth+dshad, m_dialogVertPos+m_dialogHeight-1, cshad, 0, cshad, 0);

	int yInc = 16;
	int curHorPos = m_dialogHorPos+5;
	int curVertPos = m_dialogVertPos;
	curVertPos += yInc;

	GLDebugDrawString(m_dialogHorPos+m_dialogWidth/2-((strlen(m_dialogTitle)/2)*charWidth),m_dialogVertPos+yInc ,m_dialogTitle);
	curVertPos += 20;
	

	for (int i=0;i<m_controls.size();i++)
	{
		m_controls[i]->draw(curHorPos,curVertPos,deltaTime);
	}

	restoreOpenGLState();
}


void	GL_DialogWindow::saveOpenGLState()
{
    
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);

    if( glActiveTextureARB )
    {
        glGetIntegerv(GL_ACTIVE_TEXTURE_ARB, &m_PrevActiveTextureARB);
        int maxTexUnits = 1;
        glGetIntegerv(GL_MAX_TEXTURE_UNITS_ARB, &maxTexUnits);
        maxTexUnits = max(1, min(32, maxTexUnits));
        for( int i=0; i<maxTexUnits; ++i )
        {
            glActiveTextureARB(GL_TEXTURE0_ARB+i);
            m_PrevActiveTexture1D[i] = glIsEnabled(GL_TEXTURE_1D);
            m_PrevActiveTexture2D[i] = glIsEnabled(GL_TEXTURE_2D);
            m_PrevActiveTexture3D[i] = glIsEnabled(GL_TEXTURE_3D);
            glDisable(GL_TEXTURE_1D);
            glDisable(GL_TEXTURE_2D);
            glDisable(GL_TEXTURE_3D);
        }
        glActiveTextureARB(GL_TEXTURE0_ARB);
    }

    glMatrixMode(GL_TEXTURE);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    GLint Vp[4];
    glGetIntegerv(GL_VIEWPORT, Vp);
    
    if( m_screenWidth>0 && m_screenHeight>0 )
    {
        Vp[0] = 0;
        Vp[1] = 0;
        Vp[2] = m_screenWidth-1;
        Vp[3] = m_screenHeight-1;
        glViewport(Vp[0], Vp[1], Vp[2], Vp[3]);
    }
    glLoadIdentity();
    glOrtho(Vp[0], Vp[0]+Vp[2], Vp[1]+Vp[3], Vp[1], -1, 1);
    glGetIntegerv(GL_VIEWPORT, m_ViewportInit);
    glGetFloatv(GL_PROJECTION_MATRIX, m_ProjMatrixInit);

    glGetFloatv(GL_LINE_WIDTH, &m_PrevLineWidth);
    glDisable(GL_POLYGON_STIPPLE);
    glLineWidth(1);
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_LINE_STIPPLE);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glGetTexEnviv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, &m_PrevTexEnv);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glGetIntegerv(GL_POLYGON_MODE, m_PrevPolygonMode);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDisable(GL_ALPHA_TEST);
    //glEnable(GL_ALPHA_TEST);
    //glAlphaFunc(GL_GREATER, 0);
    glDisable(GL_FOG);
    glDisable(GL_LOGIC_OP);
    glDisable(GL_SCISSOR_TEST);
    if( m_MaxClipPlanes<0 )
    {
        glGetIntegerv(GL_MAX_CLIP_PLANES, &m_MaxClipPlanes);
        if( m_MaxClipPlanes<0 || m_MaxClipPlanes>255 )
            m_MaxClipPlanes = 6;
    }
    for( int i=0; i<m_MaxClipPlanes; ++i )
        glDisable(GL_CLIP_PLANE0+i);
    m_PrevTexture = 0;
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &m_PrevTexture);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_INDEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_EDGE_FLAG_ARRAY);

    if( glBindBufferARB!=NULL )
    {
        m_PrevArrayBufferARB = m_PrevElementArrayBufferARB = 0;
        glGetIntegerv(GL_ARRAY_BUFFER_BINDING_ARB, &m_PrevArrayBufferARB);
        glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING_ARB, &m_PrevElementArrayBufferARB);
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
    }
    if( glBindProgramARB!=NULL )
    {
        m_PrevVertexProgramARB = glIsEnabled(GL_VERTEX_PROGRAM_ARB);
        m_PrevFragmentProgramARB = glIsEnabled(GL_FRAGMENT_PROGRAM_ARB);
        glDisable(GL_VERTEX_PROGRAM_ARB);
        glDisable(GL_FRAGMENT_PROGRAM_ARB);
    }
    if( glGetHandleARB!=NULL && glUseProgramObjectARB!=NULL )
    {
        m_PrevProgramObjectARB = glGetHandleARB(GL_PROGRAM_OBJECT_ARB);
        glUseProgramObjectARB(0);
    }
    glDisable(GL_TEXTURE_1D);
    glDisable(GL_TEXTURE_2D);
    if( glTexImage3D!=NULL )
    {
        m_PrevTexture3D = glIsEnabled(GL_TEXTURE_3D);
        glDisable(GL_TEXTURE_3D);
    }

    if( m_SupportTexRect )
    {
        m_PrevTexRectARB = glIsEnabled(GL_TEXTURE_RECTANGLE_ARB);
        glDisable(GL_TEXTURE_RECTANGLE_ARB);
    }
    if( glBlendEquationSeparate!=NULL )
    {
        glGetIntegerv(GL_BLEND_EQUATION_RGB, &m_PrevBlendEquationRGB);
        glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &m_PrevBlendEquationAlpha);
        glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
    }
    if( glBlendFuncSeparate!=NULL )
    {
        glGetIntegerv(GL_BLEND_SRC_RGB, &m_PrevBlendSrcRGB);
        glGetIntegerv(GL_BLEND_DST_RGB, &m_PrevBlendDstRGB);
        glGetIntegerv(GL_BLEND_SRC_ALPHA, &m_PrevBlendSrcAlpha);
        glGetIntegerv(GL_BLEND_DST_ALPHA, &m_PrevBlendDstAlpha);
        glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    if( glBlendEquation!=NULL )
    {
        glGetIntegerv(GL_BLEND_EQUATION, &m_PrevBlendEquation);
        glBlendEquation(GL_FUNC_ADD);
    }

}

void	GL_DialogWindow::restoreOpenGLState()
{
  
    glBindTexture(GL_TEXTURE_2D, m_PrevTexture);
    if( glBindBufferARB!=NULL )
    {
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, m_PrevArrayBufferARB);
        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, m_PrevElementArrayBufferARB);
    }
    if( glBindProgramARB!=NULL )
    {
        if( m_PrevVertexProgramARB )
            glEnable(GL_VERTEX_PROGRAM_ARB);
        if( m_PrevFragmentProgramARB )
            glEnable(GL_FRAGMENT_PROGRAM_ARB);
    }
    if( glGetHandleARB!=NULL && glUseProgramObjectARB!=NULL )
        glUseProgramObjectARB(m_PrevProgramObjectARB);
    if( glTexImage3D!=NULL && m_PrevTexture3D )
        glEnable(GL_TEXTURE_3D);
    if( m_SupportTexRect && m_PrevTexRectARB )
        glEnable(GL_TEXTURE_RECTANGLE_ARB);
    if( glBlendEquation!=NULL )
        glBlendEquation(m_PrevBlendEquation);
    if( glBlendEquationSeparate!=NULL )
        glBlendEquationSeparate(m_PrevBlendEquationRGB, m_PrevBlendEquationAlpha);
    if( glBlendFuncSeparate!=NULL )
        glBlendFuncSeparate(m_PrevBlendSrcRGB, m_PrevBlendDstRGB, m_PrevBlendSrcAlpha, m_PrevBlendDstAlpha);
    
    glPolygonMode(GL_FRONT, m_PrevPolygonMode[0]);
    glPolygonMode(GL_BACK, m_PrevPolygonMode[1]);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, m_PrevTexEnv);
    glLineWidth(m_PrevLineWidth);
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_TEXTURE);
    glPopMatrix();
    glPopClientAttrib();
    glPopAttrib();

    if( glActiveTextureARB )
    {
        int maxTexUnits = 1;
        glGetIntegerv(GL_MAX_TEXTURE_UNITS_ARB, &maxTexUnits);
        maxTexUnits = max(1, min(32, maxTexUnits));
        for( int i=0; i<maxTexUnits; ++i )
        {
            glActiveTextureARB(GL_TEXTURE0_ARB+i);
            if( m_PrevActiveTexture1D[i] )
                glEnable(GL_TEXTURE_1D);
            if( m_PrevActiveTexture2D[i] )
                glEnable(GL_TEXTURE_2D);
            if( m_PrevActiveTexture3D[i] )
                glEnable(GL_TEXTURE_3D);
        }
        glActiveTextureARB(m_PrevActiveTextureARB);
    }
}


void	GL_TextControl::draw(int& parentHorPos,int& parentVertPos,btScalar deltaTime)
{
	for (int i=0;i<m_textLines.size();i++)
	{
		GLDebugDrawString(parentHorPos,parentVertPos,m_textLines[i]);
		parentVertPos+=20;
	}
}



void GL_ToggleControl::draw(int& parentHorPos2,int& parentVertPos2,btScalar deltaTime)
{

	int controlHorPos = m_toggleBody->getWorldTransform().getOrigin()[0]+m_parentWindow->getScreenWidth()/2;//-m_parentWindow->getDialogWidth()/2;
	int controlVertPos = m_toggleBody->getWorldTransform().getOrigin()[1]+m_parentWindow->getScreenHeight()/2;//-m_parentWindow->getDialogHeight()/2;
	
	int parentHorPos = controlHorPos-8;
	int parentVertPos = controlVertPos-8;

	unsigned int grey = 0xff6f6f6f;
	
	drawRect(parentHorPos, parentVertPos, parentHorPos+16, parentVertPos+16, grey, grey, grey, grey);
	
	int borderSize = 2;
	unsigned int white = 0xffefefef;
	drawRect(parentHorPos+borderSize, parentVertPos+borderSize, parentHorPos+16-borderSize, parentVertPos+16-borderSize, white,white,white,white);
	
	
	if (m_active)
	{
		//unsigned int red = 0xff8f0000;
		unsigned int white = 0xff8f0000;
		unsigned int black = 0xff1f1f1f;
		borderSize = 4;
		drawRect(parentHorPos+borderSize, parentVertPos+borderSize, parentHorPos+16-borderSize, parentVertPos+16-borderSize, black,black,black,black);
	} 
	
	btVector3 rgb(1,1,0);

	if (!m_active)
	{
		rgb.setValue(1,1,1);
	}
	
	GLDebugDrawStringInternal(parentHorPos2,parentVertPos+16,m_toggleText,rgb);
	parentVertPos2+=20;
	
}


void GL_SliderControl::draw(int& parentHorPos2,int& parentVertPos2,btScalar deltaTime)
{

	int controlHorPos = m_sliderBody->getWorldTransform().getOrigin()[0]+m_parentWindow->getScreenWidth()/2;
	int controlVertPos = m_sliderBody->getWorldTransform().getOrigin()[1]+m_parentWindow->getScreenHeight()/2;
	
	int parentHorPos = controlHorPos-8;
	int parentVertPos = controlVertPos-8;

	unsigned int grey = 0xff6f6f6f;
	int borderSize = 2;
	unsigned int white = 0xffefefef;
	drawRect(parentHorPos2+80+borderSize, parentVertPos2+borderSize, parentHorPos2+m_parentWindow->getDialogWidth()-16-borderSize, parentVertPos2+2-borderSize, white,white,white,white);

	drawRect(parentHorPos, parentVertPos, parentHorPos+16, parentVertPos+16, grey, grey, grey, grey);
	
	

	drawRect(parentHorPos+borderSize, parentVertPos+borderSize, parentHorPos+16-borderSize, parentVertPos+16-borderSize, white,white,white,white);
	
	
	
	btVector3 rgb(1,1,1);
	
	GLDebugDrawStringInternal(parentHorPos2,parentVertPos2+8,m_sliderText,rgb);
	parentVertPos2+=20;
	
}
