/*
Bullet Continuous Collision Detection and Physics Library Maya Plugin
Copyright (c) 2008 Walt Disney Studios
 
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising
from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:
 
1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this
software in a product, an acknowledgment in the product documentation
would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must
not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 
Written by: Nicola Candussi <nicola@fluidinteractive.com>
*/

//drawUtils.cpp

#include <GL/gl.h>
#include <GL/glu.h>

void wire_cube()
{
    static GLuint dlist = 0;
    if(glIsList(dlist)) {
        glCallList(dlist);
    } else  {
        dlist = glGenLists(1);
        glNewList(dlist, GL_COMPILE_AND_EXECUTE);

        glBegin(GL_LINE_STRIP);
        glVertex3f(-0.5f, -0.5, 0.5);
        glVertex3f(0.5f, -0.5, 0.5);
        glVertex3f(0.5f, 0.5, 0.5);
        glVertex3f(-0.5f, 0.5, 0.5);
        glVertex3f(-0.5f, -0.5, 0.5);
        glEnd();

        glBegin(GL_LINE_STRIP);
        glVertex3f(-0.5f, -0.5, -0.5);
        glVertex3f(0.5f, -0.5, -0.5);
        glVertex3f(0.5f, 0.5, -0.5);
        glVertex3f(-0.5f, 0.5, -0.5);
        glVertex3f(-0.5f, -0.5, -0.5);
        glEnd();

        glBegin(GL_LINES);
        glVertex3f(-0.5f, -0.5f, 0.5f);
        glVertex3f(-0.5f, -0.5f, -0.5f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, -0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);
        glVertex3f(0.5f, 0.5f, -0.5f);
        glVertex3f(-0.5f, 0.5f, 0.5f);
        glVertex3f(-0.5f, 0.5f, -0.5f);
        glEnd();

        glEndList();
    }
}

void solid_cube()
{
    static GLuint dlist = 0;
    if(glIsList(dlist)) {
        glCallList(dlist);
    } else  {
        dlist = glGenLists(1);
        glNewList(dlist, GL_COMPILE_AND_EXECUTE);

        glBegin(GL_QUADS);
        glNormal3f(-1.0f, 0.0f, 0.0f);
        glVertex3f(-0.5f, -0.5f, -0.5f);
        glVertex3f(-0.5f, -0.5f, 0.5f);
        glVertex3f(-0.5f, 0.5f, 0.5f);
        glVertex3f(-0.5f, 0.5f, -0.5f);
    
        glNormal3f(0.0f, 1.0f, 0.0f);
        glVertex3f(-0.5f, 0.5f, -0.5f);
        glVertex3f(-0.5f, 0.5f, 0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);
        glVertex3f(0.5f, 0.5f, -0.5f);
    
        glNormal3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.5f, 0.5f, -0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, -0.5f);
    
        glNormal3f(0.0f, -1.0f, 0.0f);
        glVertex3f(0.5f, -0.5f, -0.5f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        glVertex3f(-0.5f, -0.5f, 0.5f);
        glVertex3f(-0.5f, -0.5f, -0.5f);
    
        glNormal3f(1.0f, 0.0f, 0.0f);
        glVertex3f(0.5f, 0.5f, -0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, -0.5f);
    
        glNormal3f(0.0f, 0.0f, 1.0f);
        glVertex3f(-0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, -0.5f, 0.5f);
        glVertex3f(0.5f, 0.5f, 0.5f);
        glVertex3f(-0.5f, 0.5f, 0.5f);
    
        glNormal3f(0.0f, 0.0f, -1.0f);
        glVertex3f(0.5f, -0.5f, -0.5f);
        glVertex3f(-0.5f, -0.5f, -0.5f);
        glVertex3f(-0.5f, 0.5f, -0.5f);
        glVertex3f(0.5f, 0.5f, -0.5f);
        glEnd();

        glEndList();
    }
}

void wire_sphere()
{
    static GLuint dlist = 0;
    if(glIsList(dlist)) {
        glCallList(dlist);
    } else  {
        GLUquadricObj* quadric = gluNewQuadric();
        gluQuadricDrawStyle(quadric, GLU_LINE);

        dlist = glGenLists(1);
        glNewList(dlist, GL_COMPILE_AND_EXECUTE);
        gluSphere(quadric, 1.0, 10, 10);
        glEndList();

        gluDeleteQuadric(quadric);
    }
}

void solid_sphere()
{
    static GLuint dlist = 0;
    if(glIsList(dlist)) {
        glCallList(dlist);
    } else  {
        GLUquadricObj* quadric = gluNewQuadric();
        gluQuadricDrawStyle(quadric, GLU_FILL);

        dlist = glGenLists(1);
        glNewList(dlist, GL_COMPILE_AND_EXECUTE);
        gluSphere(quadric, 1.0, 10, 10);
        glEndList();

        gluDeleteQuadric(quadric);
    }
}
