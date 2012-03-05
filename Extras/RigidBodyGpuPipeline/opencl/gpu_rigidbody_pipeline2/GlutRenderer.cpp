/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans


#include <GL/glew.h>
#include "GlutRenderer.h"
#include <stdio.h>


GlutRenderer* GlutRenderer::gDemoApplication;



void GlutRenderer::runMainLoop()
{
	glutMainLoop();

}


static	void glutKeyboardCallback(unsigned char key, int x, int y) {	GlutRenderer::gDemoApplication->keyboardCallback(key,x,y); }
static	void glutKeyboardUpCallback(unsigned char key, int x, int y){  GlutRenderer::gDemoApplication->keyboardUpCallback(key,x,y);}
static void glutSpecialKeyboardCallback(int key, int x, int y){	GlutRenderer::gDemoApplication->specialKeyboard(key,x,y);}
static void glutSpecialKeyboardUpCallback(int key, int x, int y){	GlutRenderer::gDemoApplication->specialKeyboardUp(key,x,y);}
static void glutReshapeCallback(int w, int h){	GlutRenderer::gDemoApplication->resize(w,h);}
static void glutIdleCallback(){ glutPostRedisplay (); }
static void glutMouseFuncCallback(int button, int state, int x, int y){	GlutRenderer::gDemoApplication->mouseFunc(button,state,x,y);}
static void	glutMotionFuncCallback(int x,int y){	GlutRenderer::gDemoApplication->mouseMotionFunc(x,y);}
static void glutDisplayCallback(void){	GlutRenderer::gDemoApplication->displayCallback();}


void GlutRenderer::resize(int width, int height)
{
	m_glutScreenWidth = width;
	m_glutScreenHeight = height;
}

void GlutRenderer::mouseFunc(int button, int state, int x, int y)
{
}
void	GlutRenderer::mouseMotionFunc(int x,int y)
{
}

void GlutRenderer::renderScene()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glutSwapBuffers();
	glutPostRedisplay();

	GLint err = glGetError();
	assert(err==GL_NO_ERROR);
}

void GlutRenderer::displayCallback()
{
	updateScene();
	
	renderScene();
}

GlutRenderer::GlutRenderer(int argc, char* argv[])
{
	glutInit(&argc, argv);
	gDemoApplication = this;
}

void GlutRenderer::initGraphics(int width, int height)
{
	m_glutScreenWidth = width;
	m_glutScreenHeight = height;
		
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);

	glutInitWindowSize(m_glutScreenWidth, m_glutScreenHeight);
	glutCreateWindow("GPU rigid body pipeline2");
	glutKeyboardFunc(glutKeyboardCallback);
	glutKeyboardUpFunc(glutKeyboardUpCallback);
	glutSpecialFunc(glutSpecialKeyboardCallback);
	glutSpecialUpFunc(glutSpecialKeyboardUpCallback);
	glutReshapeFunc(glutReshapeCallback);
	glutIdleFunc(glutIdleCallback);
	glutMouseFunc(glutMouseFuncCallback);
	glutPassiveMotionFunc(glutMotionFuncCallback);
	glutMotionFunc(glutMotionFuncCallback);
	glutDisplayFunc( glutDisplayCallback );

	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		printf("Error: %s\n", glewGetErrorString(err));
	}

	glClearColor(0.6f,0.6f,1.f,1.f);
}