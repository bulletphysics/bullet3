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

#include "DemoEntries.h"

#include "../CcdPhysicsDemo/CcdPhysicsDemo.h"
#include "../BspDemo/BspDemo.h"
#include "../BasicDemo/BasicDemo.h"
#include "../ConcaveDemo/ConcaveDemo.h"
#include "../ConcaveRaycastDemo/ConcaveRaycastDemo.h"
#include "../ConcaveConvexcastDemo/ConcaveConvexcastDemo.h"
#include "../ConvexDecompositionDemo/ConvexDecompositionDemo.h"
#include "../DynamicControlDemo/MotorDemo.h"
#include "../RagdollDemo/RagdollDemo.h"
#include "../GimpactTestDemo/GimpactTestDemo.h"
#include "../MultiThreadedDemo/MultiThreadedDemo.h"
#include "../Raytracer/Raytracer.h"
#include "../GjkConvexCastDemo/LinearConvexCastDemo.h"
#include "../VehicleDemo/VehicleDemo.h"
#include "../ConstraintDemo/ConstraintDemo.h"

#include "GlutStuff.h"//OpenGL stuff
#include "BMF_Api.h"//font stuff

extern int gNumAlignedAllocs;
extern int gNumAlignedFree;
extern int gTotalBytesAlignedAllocs;

class btEmptyDebugDemo : public DemoApplication
{
public:
	btEmptyDebugDemo()
	{

	}

	virtual void clientMoveAndDisplay()
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

		float xOffset = 10.f;
		float yStart = 20.f;
		float yIncr = 20.f;
		char buf[124];


		glColor3f(0, 0, 0);

		setOrthographicProjection();

		glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"gNumAlignedAllocs= %d",gNumAlignedAllocs);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"gNumAlignedFree= %d",gNumAlignedFree);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"# alloc-free = %d",gNumAlignedAllocs-gNumAlignedFree);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
		glRasterPos3f(xOffset,yStart,0);
		sprintf(buf,"gTotalBytesAlignedAllocs = %d",gTotalBytesAlignedAllocs);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;
#endif //BT_DEBUG_MEMORY_ALLOCATIONS

	glFlush();
	glutSwapBuffers();
			
	}

	static DemoApplication* Create()
	{
		btEmptyDebugDemo* demo = new btEmptyDebugDemo();
		demo->myinit();
		return demo;
	}

};

btDemoEntry g_demoEntries[] =
{
	{"DynamicControlDemo",MotorDemo::Create},
	{"RagdollDemo",RagdollDemo::Create},
	{"CcdPhysicsDemo", CcdPhysicsDemo::Create},
	{"ConcaveDemo",ConcaveDemo::Create},
	{"ConcaveRaycastDemo",ConcaveRaycastDemo::Create},
	{"ConcaveConvexcastDemo",ConcaveConvexcastDemo::Create},
	{"ConvexDecomposition",ConvexDecompositionDemo::Create},
	{"BasicDemo", BasicDemo::Create},
	{"BspDemo", BspDemo::Create},
	{"Gimpact Test", GimpactConcaveDemo::Create},
	{"MultiThreaded", MultiThreadedDemo::Create},
	{"Raytracer Test",Raytracer::Create},
	{"GjkConvexCast",LinearConvexCastDemo::Create},
	{"VehicleDemo",VehicleDemo::Create},
	{"ConstraintDemo",ConstraintDemo::Create},
	{"MemoryLeakChecker",btEmptyDebugDemo::Create},	
	{0, 0}
};


