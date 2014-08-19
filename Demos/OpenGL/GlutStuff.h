/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef GLUT_STUFF_H
#define GLUT_STUFF_H

#ifdef DONT_USE_GLUT
#include "OpenGLWindow/OpenGLInclude.h"
#define BT_ACTIVE_ALT   8192
#define BT_ACTIVE_SHIFT 8193
#define BT_ACTIVE_CTRL  8194
#else //DONT_USE_GLUT

//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/OpenGL.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else//(__APPLE__) && !defined (VMDMESA)
	

#ifdef _WINDOWS
	#include <windows.h>
	#include <GL/gl.h>
	#include <GL/glu.h>
#else	//_WINDOWS

#ifdef _WIN32
	#include <windows.h>
#endif//_WIN32
#include <GL/gl.h>
#include <GL/glut.h>

#endif//_WINDOWS
#endif //(__APPLE__) && !defined (VMDMESA)

#ifdef _WINDOWS
	#define BT_ACTIVE_ALT   VK_LMENU
        #define BT_ACTIVE_SHIFT VK_LSHIFT
        #define BT_ACTIVE_CTRL  VK_LCONTROL
#else   //_WINDOWS
        #define BT_KEY_K 'k'
        #define BT_KEY_LEFT                     GLUT_KEY_LEFT
        #define BT_KEY_RIGHT            GLUT_KEY_RIGHT
        #define BT_KEY_UP                       GLUT_KEY_UP
        #define BT_KEY_DOWN                     GLUT_KEY_DOWN
        #define BT_KEY_F1                       GLUT_KEY_F1
        #define BT_KEY_F2                       GLUT_KEY_F2
        #define BT_KEY_F3                       GLUT_KEY_F3
        #define BT_KEY_F4                       GLUT_KEY_F4
        #define BT_KEY_F5                       GLUT_KEY_F5
        #define BT_KEY_PAGEUP           GLUT_KEY_PAGE_UP
        #define BT_KEY_PAGEDOWN         GLUT_KEY_PAGE_DOWN
        #define BT_KEY_END                      GLUT_KEY_END
        #define BT_KEY_HOME                     GLUT_KEY_HOME
        #define BT_ACTIVE_ALT           GLUT_ACTIVE_ALT
        #define BT_ACTIVE_CTRL          GLUT_ACTIVE_ALT
        #define BT_ACTIVE_SHIFT         GLUT_ACTIVE_SHIFT
#endif
#if BT_USE_FREEGLUT
#include "GL/freeglut_ext.h" //to be able to return from glutMainLoop()
#endif
#endif // DONT_USE_GLUT


class DemoApplication;

int glutmain(int argc, char **argv,int width,int height,const char* title,DemoApplication* demoApp);

#if defined(BT_USE_DOUBLE_PRECISION)
#define btglLoadMatrix glLoadMatrixd
#define btglMultMatrix glMultMatrixd
#define btglColor3 glColor3d
#define btglVertex3 glVertex3d
#else
#define btglLoadMatrix glLoadMatrixf
#define btglMultMatrix glMultMatrixf
#define btglColor3 glColor3f
#define btglVertex3 glVertex3d
#endif

#endif //GLUT_STUFF_H
