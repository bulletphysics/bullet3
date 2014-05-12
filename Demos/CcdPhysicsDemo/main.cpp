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

#include "CcdPhysicsDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"
#ifdef __DEBUG_FPU_ISSUES
#define _GNU_SOURCE
#include <fenv.h>
#endif

GLDebugDrawer	gDebugDrawer;

int main(int argc,char** argv)
{

	CcdPhysicsDemo* ccdDemo = new CcdPhysicsDemo();

#ifdef __DEBUG_FPU_ISSUES
//	feenableexcept (FE_DIVBYZERO);
//	feenableexcept (FE_INEXACT);
//	feenableexcept (FE_INVALID);
//	feenableexcept (FE_OVERFLOW|FE_DIVBYZERO|FE_UNDERFLOW);
//	feenableexcept (FE_UNDERFLOW);
#endif

	ccdDemo->initPhysics();
	ccdDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);


	glutmain(argc, argv,640,480,"Bullet Physics Demo. http://bulletphysics.com",ccdDemo);

	delete ccdDemo;
	return 0;

}
