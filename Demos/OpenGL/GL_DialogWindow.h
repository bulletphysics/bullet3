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

#ifndef GL_DIALOG_WINDOW_H
#define GL_DIALOG_WINDOW_H

class btCollisionObject;


//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <TargetConditionals.h>
#if (defined (TARGET_OS_IPHONE) && TARGET_OS_IPHONE) || (defined (TARGET_IPHONE_SIMULATOR) && TARGET_IPHONE_SIMULATOR)
#import <OpenGLES/ES1/gl.h>
#define glOrtho glOrthof
#else
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#endif
#else


#ifdef _WIN32
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glut.h>
#endif
#endif


#include "LinearMath/btScalar.h"
#include "LinearMath/btAlignedObjectArray.h"
class btTypedConstraint;

class GL_DialogWindow;

enum GL_DIALOG_CONTROL_TYPES
{
	GL_TEXT_CONTROL=1,
	GL_TOGGLE_CONTROL,
	GL_SLIDER_CONTROL,
	GL_CONTROL_MAX_TYPE
};

class GL_DialogControl
{
protected:
	int m_type;

public:

	virtual ~GL_DialogControl()
	{
	}
	virtual void	draw(int& parentHorPos,int& parentVertPos,btScalar deltaTime)=0;

	int	getType() const
	{
		return m_type;
	}
};

struct GL_TextControl : public GL_DialogControl
{
public:

	btAlignedObjectArray<const char*> m_textLines;

	GL_TextControl()
	{
		m_type = GL_TEXT_CONTROL;
	}
	
	virtual ~GL_TextControl() {}

	virtual void draw(int& parentHorPos,int& parentVertPos,btScalar deltaTime);
};


struct GL_ToggleControl : public GL_DialogControl
{
	btCollisionObject* m_toggleBody;
	GL_DialogWindow* m_parentWindow;

	const char* m_toggleText;

public:

	bool	m_active;

	
	GL_ToggleControl(const char* toggleText,btCollisionObject* toggleBody, GL_DialogWindow* parentWindow)
		:m_toggleBody(toggleBody),
		m_parentWindow(parentWindow),
		m_toggleText(toggleText),
		m_active(false)
	{
		m_type = GL_TOGGLE_CONTROL;
	}

	virtual void draw(int& parentHorPos,int& parentVertPos,btScalar deltaTime);
};

struct GL_SliderControl : public GL_DialogControl
{
	btCollisionObject* m_sliderBody;
	GL_DialogWindow* m_parentWindow;
	btScalar m_lowerLimit;
	btScalar m_upperLimit;
	btTypedConstraint* m_constraint;
	btScalar m_fraction;

	const char* m_sliderText;
public:

	GL_SliderControl(const char* sliderText,btCollisionObject* sliderBody, GL_DialogWindow* parentWindow, btScalar lowerLimit,btScalar upperLimit,btTypedConstraint* constaint)
		:m_sliderBody(sliderBody),
		m_parentWindow(parentWindow),
		m_lowerLimit(lowerLimit),
		m_upperLimit(upperLimit),
		m_constraint(constaint),
		m_sliderText(sliderText)
	{
		m_type = GL_SLIDER_CONTROL;
	}

	virtual void draw(int& parentHorPos,int& parentVertPos,btScalar deltaTime);

	btScalar	btGetFraction() { return m_fraction; }

	btScalar getLowerLimit()
	{
		return m_lowerLimit;
	}
	btScalar getUpperLimit()
	{
		return m_upperLimit;
	}
	btTypedConstraint* getConstraint()
	{
		return m_constraint;
	}
};

///Very basic OpenGL Graphical Userinterface Window with text, toggle, slider control
class GL_DialogWindow
{

	int	m_dialogHorPos;
	int	m_dialogVertPos;
	int	m_dialogWidth;
	int	m_dialogHeight;

	int m_screenWidth;
	int m_screenHeight;

	const char* m_dialogTitle;

	//saved OpenGL settings
	  GLfloat             m_PrevLineWidth;
    GLint               m_PrevTexEnv;
    GLint               m_PrevPolygonMode[2];
    GLint               m_MaxClipPlanes;
    GLint               m_PrevTexture;
    GLint               m_PrevArrayBufferARB;
    GLint               m_PrevElementArrayBufferARB;
    GLboolean           m_PrevVertexProgramARB;
    GLboolean           m_PrevFragmentProgramARB;
    GLuint              m_PrevProgramObjectARB;
    GLboolean           m_PrevTexture3D;
    GLboolean           m_PrevActiveTexture1D[32];
    GLboolean           m_PrevActiveTexture2D[32];
    GLboolean           m_PrevActiveTexture3D[32];
    GLint               m_PrevActiveTextureARB;
    bool                m_SupportTexRect;
    GLboolean           m_PrevTexRectARB;
    GLint               m_PrevBlendEquation;
    GLint               m_PrevBlendEquationRGB;
    GLint               m_PrevBlendEquationAlpha;
    GLint               m_PrevBlendSrcRGB;
    GLint               m_PrevBlendDstRGB;
    GLint               m_PrevBlendSrcAlpha;
    GLint               m_PrevBlendDstAlpha;
    GLint               m_ViewportInit[4];
    GLfloat             m_ProjMatrixInit[16];

	btCollisionObject*	m_collisionObject;

	btAlignedObjectArray<GL_DialogControl*> m_controls;

protected:


	void	saveOpenGLState();
	void	restoreOpenGLState();

//	void	drawLine(int _X0, int _Y0, int _X1, int _Y1, unsigned int _Color0, unsigned int _Color1, bool antiAliased);
//	void	drawRect(int horStart, int vertStart, int horEnd, int vertEnd, unsigned int argbColor00,unsigned int argbColor10,unsigned int argbColor01,unsigned int argbColor11);

public:

	
	GL_DialogWindow(int horPos,int vertPos,int dialogWidth,int dialogHeight,btCollisionObject* colObject, const char* dialogTitle);

	virtual ~GL_DialogWindow();

	void	draw(btScalar deltaTime);
	
	void	setScreenSize(int width, int height);

	void	setStartPosition(int	dialogHorPos,int	dialogVertPos);

	void	addControl(GL_DialogControl* control)
	{
		m_controls.push_back(control);
	}

	void	removeControl(GL_DialogControl* control)
	{
		m_controls.remove(control);
	}

	btCollisionObject* getCollisionObject()
	{
		return m_collisionObject;
	}

	int	getDialogHorPos() const
	{
		return m_dialogHorPos;
	}
	int getDialogVertPos() const
	{
		return m_dialogVertPos;
	}
	int getDialogWidth() const
	{
		return m_dialogWidth;
	}

	int	getDialogHeight() const 
	{
		return m_dialogHeight;
	}
	int getScreenWidth() const
	{
		return m_screenWidth;
	}
	int getScreenHeight() const
	{
		return m_screenHeight;
	}

	int getNumControls() const
	{
		return m_controls.size();
	}

	const GL_DialogControl* getControl(int index) const
	{
		return m_controls[index];
	}
	GL_DialogControl* getControl(int index)
	{
		return m_controls[index];
	}
};

#endif //GL_DIALOG_WINDOW_H
