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


///
/// Collision Demo shows a degenerate case, where the Simplex solver has to deal with near-affine dependent cases
/// See the define CATCH_DEGENERATE_TETRAHEDRON in Bullet's btVoronoiSimplexSolver.cpp
///


#define CHECK_GENSHER_TRIANGLE_CASE 1


///This low-level internal demo does intentionally NOT use the btBulletCollisionCommon.h header
///It needs internal access
#include "GL_Simplex1to4.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"

#include "CollisionDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"
#include "LinearMath/btIDebugDraw.h"


float yaw=0.f,pitch=0.f,roll=0.f;
const int maxNumObjects = 4;
const int numObjects = 2;

GL_Simplex1to4 simplex;

btPolyhedralConvexShape*	shapePtr[maxNumObjects];

btTransform tr[numObjects];
int screenWidth = 640;
int screenHeight = 480;

void DrawRasterizerLine(float const* , float const*, int)
{

}

int main(int argc,char** argv)
{
	CollisionDemo* colDemo = new CollisionDemo();

#ifdef CHECK_GENSHER_TRIANGLE_CASE
	colDemo->setCameraDistance(8.f);
#else
	colDemo->setCameraDistance(18.f);
	
#endif //
	colDemo->initPhysics();

	
	
	return glutmain(argc, argv,screenWidth,screenHeight,"Collision Demo",colDemo);
}

void CollisionDemo::initPhysics()
{
	m_debugMode |= btIDebugDraw::DBG_DrawWireframe;
#ifdef CHECK_GENSHER_TRIANGLE_CASE
	m_azi = 140.f;
#else
	m_azi = 250.f;
#endif
	m_ele = 25.f;

	tr[0].setOrigin(btVector3(0.0013328250f,8.1363249f,7.0390840f));
	tr[1].setOrigin(btVector3(0.00000000f,9.1262732f,2.0343180f));

	//tr[0].setOrigin(btVector3(0,0,0));
	//tr[1].setOrigin(btVector3(0,10,0));

	btMatrix3x3 basisA;
	basisA.setValue(0.99999958f,0.00022980258f,0.00090992288f,
		-0.00029313788f,0.99753088f,0.070228584f,
		-0.00089153741f,-0.070228823f,0.99753052f);

	btMatrix3x3 basisB;
	basisB.setValue(1.0000000f,4.4865553e-018f,-4.4410586e-017f,
		4.4865495e-018f,0.97979438f,0.20000751f,
		4.4410586e-017f,-0.20000751f,0.97979438f);

	tr[0].setBasis(basisA);
	tr[1].setBasis(basisB);

#ifdef CHECK_GENSHER_TRIANGLE_CASE
	tr[0].setIdentity();
	tr[1].setIdentity();
#endif //CHECK_GENSHER_TRIANGLE_CASE

	btVector3 boxHalfExtentsA(1.0000004768371582f,1.0000004768371582f,1.0000001192092896f);
	btVector3 boxHalfExtentsB(3.2836332321166992f,3.2836332321166992f,3.2836320400238037f);

	btBoxShape*	boxA = new btBoxShape(boxHalfExtentsA);
	btBoxShape*	boxB = new btBoxShape(boxHalfExtentsB);

	float p1[3], p2[3], p3[3], q1[3],q2[3], q3[3]; 

   p1[0] = -3.9242966175; 
   p1[1] = -0.5582823175; 
   p1[2] = 2.0101921558; 
    
   p2[0] = -3.6731941700; 
   p2[1] = -0.5604774356; 
   p2[2] = 2.00301921558; 
    
   p3[0] = -3.6698703766; 
   p3[1] = -0.3097069263; 
   p3[2] = 2.0073683262; 
    
    
    
    
   q1[0] = -2.6317186356; 
   q1[1] = -1.0000005960; 
   q1[2] = 1.9999998808; 
    
   q2[0] = -2.6317174435; 
   q2[1] = 0.9999994636; 
   q2[2] = 1.9999998808; 
    
   q3[0] = -4.6317176819; 
   q3[1] = 1.0f; 
   q3[2] = 1.9999998808; 



	btTriangleShape* trishapeA = new btTriangleShape(btVector3(p1[0], p1[1], p1[2]), btVector3(p2[0], p2[1], p2[2]), btVector3(p3[0], p3[1], p3[2])); 
	trishapeA->setMargin(0.001f); 
  
	btTriangleShape* trishapeB = new btTriangleShape(btVector3(q1[0], q1[1], q1[2]), btVector3(q2[0], q2[1], q2[2]), btVector3(q3[0], q3[1], q3[2])); 
	trishapeB->setMargin(0.001f); 


#ifdef CHECK_GENSHER_TRIANGLE_CASE
	shapePtr[0] = trishapeA;//boxA;
	shapePtr[1] = trishapeB;//boxB;
#else
	shapePtr[0] = boxA;
	shapePtr[1] = boxB;
#endif

}

void CollisionDemo::clientMoveAndDisplay()
{
	
	displayCallback();
}


static btVoronoiSimplexSolver sGjkSimplexSolver;
btSimplexSolverInterface& gGjkSimplexSolver = sGjkSimplexSolver;



void CollisionDemo::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glDisable(GL_LIGHTING);


   
    
   btVoronoiSimplexSolver sGjkSimplexSolver;    
    
   btGjkPairDetector convexConvex(shapePtr[0],shapePtr[1],&sGjkSimplexSolver,0); 
   btPointCollector gjkOutput; 
   btGjkPairDetector::ClosestPointInput input; 
    

    
   input.m_transformA = tr[0]; 
   input.m_transformB = tr[1]; 
    
   convexConvex.getClosestPoints(input, gjkOutput, 0); 
    
   if (gjkOutput.m_hasResult) 
   { 
      //VECCOPY(pa, gjkOutput.m_pointInWorld); 
      //VECCOPY(pb, gjkOutput.m_pointInWorld); 
      //VECADDFAC(pb, pb, gjkOutput.m_normalOnBInWorld, gjkOutput.m_distance); 
             printf("bullet: %10.10f\n", gjkOutput.m_distance); // = 0.24 => that's absolutely wrong! 
	 		btVector3 endPt = gjkOutput.m_pointInWorld +
			gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;

			 glBegin(GL_LINES);
			glColor3f(1, 0, 0);
			glVertex3d(gjkOutput.m_pointInWorld.x(), gjkOutput.m_pointInWorld.y(),gjkOutput.m_pointInWorld.z());
			glVertex3d(endPt.x(),endPt.y(),endPt.z());
			//glVertex3d(gjkOutputm_pointInWorld.x(), gjkOutputm_pointInWorld.y(),gjkOutputm_pointInWorld.z());
			//glVertex3d(gjkOutputm_pointInWorld.x(), gjkOutputm_pointInWorld.y(),gjkOutputm_pointInWorld.z());
			glEnd();
   } 

   //GL_ShapeDrawer::drawCoordSystem();

	float m[16];
	int i;

//	btGjkPairDetector	convexConvex(shapePtr[0],shapePtr[1],&sGjkSimplexSolver,0);

	convexConvex.getClosestPoints(input ,gjkOutput,0);


	for (i=0;i<numObjects;i++)
	{
		
		tr[i].getOpenGLMatrix( m );

		GL_ShapeDrawer::drawOpenGL(m,shapePtr[i],btVector3(1,1,1),getDebugMode());


	}

	simplex.setSimplexSolver(&sGjkSimplexSolver);
	btPoint3 ybuf[4],pbuf[4],qbuf[4];
	int numpoints = sGjkSimplexSolver.getSimplex(pbuf,qbuf,ybuf);
	simplex.reset();
	
	for (i=0;i<numpoints;i++)
		simplex.addVertex(ybuf[i]);

	btTransform ident;
	ident.setIdentity();
	ident.getOpenGLMatrix(m);
	GL_ShapeDrawer::drawOpenGL(m,&simplex,btVector3(1,1,1),getDebugMode());


	btQuaternion orn;
	orn.setEuler(yaw,pitch,roll);
	//let it rotate
	//tr[0].setRotation(orn);

	pitch += 0.005f;
	yaw += 0.01f;

	glFlush();
    glutSwapBuffers();
}

