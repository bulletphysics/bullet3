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
#include "../SliderConstraintDemo/SliderConstraintDemo.h"
#include "../RagdollDemo/RagdollDemo.h"
#include "../GimpactTestDemo/GimpactTestDemo.h"
#include "../Raytracer/Raytracer.h"
#include "../GjkConvexCastDemo/LinearConvexCastDemo.h"
#include "../VehicleDemo/VehicleDemo.h"
#include "../ConstraintDemo/ConstraintDemo.h"
#include "../Benchmarks/BenchmarkDemo.h"
#include "../SoftDemo/SoftDemo.h"


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
	
	{"SoftBody Cloth",SoftDemo0::Create},
	{"SoftBody Pressure",SoftDemo1::Create},
	{"SoftBody Volume",SoftDemo2::Create},
	{"SoftBody Ropes",SoftDemo3::Create},
	{"SoftBody Ropes Attach",SoftDemo4::Create},
	{"SoftBody Cloth Attach",SoftDemo5::Create},
	{"SoftBody Sticks",SoftDemo6::Create},
	{"SoftBody Collide",SoftDemo7::Create},
	{"SoftBody Collide2",SoftDemo8::Create},
	{"SoftBody Collide3",SoftDemo9::Create},
	{"SoftBody Impact",SoftDemo10::Create},
	{"SoftBody Aero",SoftDemo11::Create},
	{"SoftBody Friction",SoftDemo12::Create},
	{"SoftBody Torus",SoftDemo13::Create},
	{"SoftBody Torus Match",SoftDemo14::Create},
	{"SoftBody Bunny",SoftDemo15::Create},
	{"SoftBody Bunny Match",SoftDemo16::Create},
	{"SoftBody Init Cutting",SoftDemo17::Create},
	{"SoftBody Cluster Deform",SoftDemo18::Create},
	{"SoftBody Cluster Collide1",SoftDemo19::Create},
	{"SoftBody Cluster Collide2",SoftDemo20::Create},
	{"SoftBody Cluster Socket",SoftDemo21::Create},
	{"SoftBody Cluster Hinge",SoftDemo22::Create},
	{"SoftBody Cluster Combine",SoftDemo23::Create},
	{"SoftBody Cluster Car",SoftDemo24::Create},
	{"SoftBody Cluster Robot",SoftDemo25::Create},
	{"SoftBody Cluster Stack Soft",SoftDemo26::Create},
	{"SoftBody Cluster Stack Mixed",SoftDemo27::Create},
	{"DynamicControlDemo",MotorDemo::Create},
	{"RagdollDemo",RagdollDemo::Create},
	{"SliderConstraint",SliderConstraintDemo::Create},
	{"BasicDemo", BasicDemo::Create},	
	{"CcdPhysicsDemo", CcdPhysicsDemo::Create},
	{"ConcaveDemo",ConcaveDemo::Create},
	{"ConcaveRaycastDemo",ConcaveRaycastDemo::Create},
	{"ConcaveConvexcastDemo",ConcaveConvexcastDemo::Create},
	{"ConvexDecomposition",ConvexDecompositionDemo::Create},
	{"BspDemo", BspDemo::Create},
	{"Gimpact Test", GimpactConcaveDemo::Create},
	{"Raytracer Test",Raytracer::Create},
	{"GjkConvexCast",LinearConvexCastDemo::Create},
	{"VehicleDemo",VehicleDemo::Create},
	{"Benchmark 3000 FALL",BenchmarkDemo1::Create},
	{"Benchmark 1000 STACK",BenchmarkDemo2::Create},
	{"Benchmark 136 RAGDOLLS",BenchmarkDemo3::Create},
	{"Benchmark 1000 CONVEX",BenchmarkDemo4::Create},
	{"Benchmark Mesh-Prim",BenchmarkDemo5::Create},
	{"Benchmark Mesh-Convex",BenchmarkDemo6::Create},
	{"Benchmark Raycast",BenchmarkDemo7::Create},

	{"MemoryLeakChecker",btEmptyDebugDemo::Create},	
	{0, 0}
};


