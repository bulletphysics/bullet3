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

#ifdef _WINDOWS

#include "Win32DemoApplication.h"




#if 0
void	Win32DemoApplication::renderme()
{
}
void	Win32DemoApplication::setTexturing(bool useTexture)
{
}
	
void	Win32DemoApplication::setShadows(bool useShadows)
{
}
	
void	Win32DemoApplication::setCameraDistance(float camDist)
{
}
void	Win32DemoApplication::clientResetScene()
{

}
#endif

void Win32DemoApplication::updateModifierKeys()
{
	//not yet
}



void Win32DemoApplication::specialKeyboard(int key, int x, int y)	
{
	(void)x;
	(void)y;

	switch (key) 
	{
	case VK_LEFT : stepLeft(); break;
	case VK_RIGHT : stepRight(); break;
	case VK_UP : stepFront(); break;
	case VK_DOWN : stepBack(); break;

//	case GLUT_KEY_PAGE_UP : zoomIn(); break;
//	case GLUT_KEY_PAGE_DOWN : zoomOut(); break;
//	case GLUT_KEY_HOME : toggleIdle(); break;

	default:
		//        std::cout << "unused (special) key : " << key << std::endl;
		break;
	}

}

void	Win32DemoApplication::swapBuffers()
{
}
	
#endif
	
