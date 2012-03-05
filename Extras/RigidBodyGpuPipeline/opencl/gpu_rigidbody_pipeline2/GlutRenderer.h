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

#ifndef GLUT_RENDERER_H
#define GLUT_RENDERER_H

#include "btGlutInclude.h"
#include "LinearMath/btVector3.h"

struct GlutRenderer
{
	static GlutRenderer* gDemoApplication;
	int m_glutScreenWidth;
	int m_glutScreenHeight;

	btVector3 m_cameraPosition;
	btVector3 m_cameraTargetPosition;
	btScalar m_cameraDistance;
	btVector3 m_cameraUp;
	float m_azimuth;
	float m_elevation;


	GlutRenderer(int argc, char* argv[]);
	
	virtual void initGraphics(int width, int height);
	virtual void cleanup() {}
	
	void runMainLoop();

	virtual void updateScene(){};
	
	virtual void renderScene();

	virtual void	keyboardCallback(unsigned char key, int x, int y) {};
	virtual void	keyboardUpCallback(unsigned char key, int x, int y) {}
	virtual void	specialKeyboard(int key, int x, int y){}
	virtual void	specialKeyboardUp(int key, int x, int y){}
	virtual void	resize(int w, int h);
	virtual void	mouseFunc(int button, int state, int x, int y);
	virtual void	mouseMotionFunc(int x,int y);
	virtual void displayCallback();
	

};

#endif //GLUT_RENDERER_H
