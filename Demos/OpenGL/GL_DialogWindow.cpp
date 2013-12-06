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



#include "GLDebugFont.h"
#include "btBulletDynamicsCommon.h"

#include <stdio.h> // for sprintf()

#define USE_ARRAYS 1


GL_DialogWindow::GL_DialogWindow(int horPos,int vertPos,int dialogWidth,int dialogHeight, btCollisionObject* collisionObject,const char* dialogTitle)
:m_dialogHorPos(horPos),
m_dialogVertPos(vertPos),
m_dialogWidth(dialogWidth),
m_dialogHeight(dialogHeight),
m_screenWidth(0),
m_screenHeight(0),
m_dialogTitle(dialogTitle),
m_MaxClipPlanes(-1),
m_collisionObject(collisionObject)

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

	GLfloat vVertices[] = {(GLfloat)_X0+dx,(GLfloat)_Y0+dy,(GLfloat)_X1+dx,(GLfloat)_Y1+dy}; 

	bool antiAliased = false;
    if( antiAliased )
        glEnable(GL_LINE_SMOOTH);
    else
        glDisable(GL_LINE_SMOOTH);
    glDisable(GL_TEXTURE_2D);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

#ifdef USE_ARRAYS
	glColor4ub(GLubyte(_Color0>>16), GLubyte(_Color0>>8), GLubyte(_Color0), GLubyte(_Color0>>24));
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glLineWidth(2.0f);
	glVertexPointer(2, GL_FLOAT, 0, vVertices);
	glEnableClientState(GL_VERTEX_ARRAY);
	glDrawArrays(GL_LINES,0,2);
#else
	glLineWidth(13.0f);
    glBegin(GL_LINES);
        glColor4ub(GLubyte(_Color0>>16), GLubyte(_Color0>>8), GLubyte(_Color0), GLubyte(_Color0>>24));
        glVertex2f((GLfloat)_X0+dx, (GLfloat)_Y0+dy);
        glColor4ub(GLubyte(_Color1>>16), GLubyte(_Color1>>8), GLubyte(_Color1), GLubyte(_Color1>>24));
        glVertex2f((GLfloat)_X1+dx, (GLfloat)_Y1+dy);
    glEnd();
#endif
    glDisable(GL_LINE_SMOOTH);
}

static void	drawRect(int horStart, int vertStart, int horEnd, int vertEnd, unsigned int argbColor00,unsigned int argbColor10,unsigned int argbColor01,unsigned int argbColor11)
{
	float dx = 0;
	float dy = 0;
	glDisable(GL_TEXTURE_2D);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
#ifdef USE_ARRAYS
	GLfloat verts[] ={
			0.0f, 1.0f, 0.0f,
			-1.0f, -1.0f, 0.0f,
			1.0f, -1.0f, 0.0f,
			0.f,0.f,0.f
		};

	 glColor4ub(GLubyte(argbColor00>>16), GLubyte(argbColor00>>8), GLubyte(argbColor00), GLubyte(argbColor00>>24));
	verts[0] = (GLfloat)horStart+dx;		verts[1] = (GLfloat)vertStart+dy;
	verts[2] = (GLfloat)horEnd+dx;			verts[3] = (GLfloat)vertStart+dy;
	verts[4] = (GLfloat)horEnd+dx;			verts[5] = (GLfloat)vertEnd+dy;
	verts[6] = (GLfloat)horStart+dx;		verts[7] = (GLfloat)vertEnd+dy;

	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState (GL_TEXTURE_COORD_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(2, GL_FLOAT, 0, verts);
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

#else
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
#endif

}

void GL_DialogWindow::draw(btScalar deltaTime)
{
	if (!m_screenWidth || !m_screenHeight)
		return;

	m_dialogHorPos = int(m_collisionObject->getWorldTransform().getOrigin()[0]+m_screenWidth/2.f-m_dialogWidth/2.f);
	
	m_dialogVertPos = int(m_collisionObject->getWorldTransform().getOrigin()[1]+m_screenHeight/2.f-m_dialogHeight/2.f);
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

	GLDebugDrawString(m_dialogHorPos+m_dialogWidth/2-((int(strlen(m_dialogTitle)/2))*charWidth),m_dialogVertPos+yInc ,m_dialogTitle);
	curVertPos += 20;
	

	for (int i=0;i<m_controls.size();i++)
	{
		m_controls[i]->draw(curHorPos,curVertPos,deltaTime);
	}

	restoreOpenGLState();
}


void	GL_DialogWindow::saveOpenGLState()
{
#if 0   
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);
#endif

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
 //   glDisable(GL_POLYGON_STIPPLE);
    glLineWidth(1);

    glDisable(GL_LINE_SMOOTH);
//    glDisable(GL_LINE_STIPPLE);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glGetTexEnviv(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, &m_PrevTexEnv);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glDisable(GL_TEXTURE_2D);

}

void	GL_DialogWindow::restoreOpenGLState()
{
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

	int controlHorPos = int(m_toggleBody->getWorldTransform().getOrigin()[0]+m_parentWindow->getScreenWidth()/2);
	int controlVertPos = int(m_toggleBody->getWorldTransform().getOrigin()[1]+m_parentWindow->getScreenHeight()/2);
	
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
	//	unsigned int white = 0xff8f0000;
		unsigned int black = 0xff1f1f1f;
		borderSize = 4;
		drawRect(parentHorPos+borderSize, parentVertPos+borderSize, parentHorPos+16-borderSize, parentVertPos+16-borderSize, black,black,black,black);
	} 
	
	btVector3 rgb(1,1,1);

	GLDebugDrawStringInternal(parentHorPos2,parentVertPos+16,m_toggleText,rgb);
	parentVertPos2+=20;
	
}

void GL_SliderControl::draw(int& parentHorPos2,int& parentVertPos2,btScalar deltaTime)
{

	int controlHorPos = int(m_sliderBody->getWorldTransform().getOrigin()[0]+m_parentWindow->getScreenWidth()/2);
	int controlVertPos = int(m_sliderBody->getWorldTransform().getOrigin()[1]+m_parentWindow->getScreenHeight()/2);
	
	int parentHorPos = controlHorPos-8;
	int parentVertPos = controlVertPos-8;

	unsigned int grey = 0xff6f6f6f;
	int borderSize = 2;
	unsigned int white = 0xffefefef;
	int sliderPosS = parentHorPos2+150+borderSize;
	int sliderPosE = parentHorPos2+m_parentWindow->getDialogWidth()-40-borderSize;
	int sliderPos = controlHorPos;
	if(sliderPos < sliderPosS) sliderPos = sliderPosS;
	if(sliderPos > sliderPosE) sliderPos = sliderPosE;
//	drawRect(parentHorPos2+80+borderSize, parentVertPos2+borderSize, parentHorPos2+m_parentWindow->getDialogWidth()-16-borderSize, parentVertPos2+2-borderSize, white,white,white,white);
	drawRect(	sliderPosS, 
				parentVertPos2+borderSize, 
				sliderPosE, 
				parentVertPos2+2-borderSize, 
				white,white,white,white);

	drawRect(parentHorPos, parentVertPos, parentHorPos+16, parentVertPos+16, grey, grey, grey, grey);
	
	

	drawRect(parentHorPos+borderSize, parentVertPos+borderSize, parentHorPos+16-borderSize, parentVertPos+16-borderSize, white,white,white,white);
	
	
	
	btVector3 rgb(1,1,1);

//	btSliderConstraint* pSlider = (btSliderConstraint*)m_constraint;
//	btScalar currPos = pSlider->getLinearPos();
//	if(currPos < pSlider->getLowerLinLimit()) currPos = pSlider->getLowerLinLimit();
//	if(currPos > pSlider->getUpperLinLimit()) currPos = pSlider->getUpperLinLimit();
//	m_fraction = (currPos - pSlider->getLowerLinLimit()) / (pSlider->getUpperLinLimit() - pSlider->getLowerLinLimit());
	m_fraction = (btScalar)(sliderPos - sliderPosS) / (btScalar)(sliderPosE - sliderPosS);

	char tmpBuf[256];
	sprintf(tmpBuf, "%s %3d%%", m_sliderText, (int)(m_fraction * 100.f));

//	GLDebugDrawStringInternal(parentHorPos2,parentVertPos2+8,m_sliderText,rgb);
	GLDebugDrawStringInternal(parentHorPos2,parentVertPos2+8, tmpBuf, rgb);
	parentVertPos2+=20;
	
}
