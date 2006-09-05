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

//to be implemented by the demo
void clientDisplay();
void clientMoveAndDisplay();
void clientResetScene();


int glutmain(int argc, char **argv,int width,int height,const char* title);

void	setCameraDistance(float dist);
int		getDebugMode();
void	setDebugMode(int mode);

void defaultKeyboard(unsigned char key, int x, int y);
void clientKeyboard(unsigned char key, int x, int y);

void defaultSpecialKeyboard(int key, int x, int y);
void clientSpecialKeyboard(int key, int x, int y);

void clientMouseFunc(int button, int state, int x, int y);
void	clientMotionFunc(int x,int y);
#endif //GLUT_STUFF_H
