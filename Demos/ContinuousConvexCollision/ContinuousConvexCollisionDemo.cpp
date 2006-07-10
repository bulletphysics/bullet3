/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
 */


/*
	Continuous Convex Collision Demo demonstrates an efficient continuous collision detection algorithm.
	Both linear and angular velocities are supported. Convex Objects are sampled using Supporting Vertex.
	Motion using Exponential Map.
	Future ideas: Comparison with Screwing Motion. 
	Also comparision with Algebraic CCD and Interval Arithmetic methods (Stephane Redon)
*/

#include "SimdQuaternion.h"
#include "SimdTransform.h"
#include "NarrowPhaseCollision/VoronoiSimplexSolver.h"
#include "CollisionShapes/BoxShape.h"
#include "CollisionShapes/MinkowskiSumShape.h"

#include "NarrowPhaseCollision/GjkPairDetector.h"
#include "NarrowPhaseCollision/GjkConvexCast.h"
#include "NarrowPhaseCollision/SubSimplexConvexCast.h"
#include "NarrowPhaseCollision/ContinuousConvexCollision.h"

#include "SimdTransformUtil.h"
#include "DebugCastResult.h"

#include "CollisionShapes/SphereShape.h"

#include "CollisionShapes/Simplex1to4Shape.h"

#include "GL_ShapeDrawer.h"
#ifdef WIN32 //needed for glut.h
#include <windows.h>
#endif
//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "GlutStuff.h"


float yaw=0.f,pitch=0.f,roll=0.f;
const int maxNumObjects = 4;
const int numObjects = 2;

SimdVector3 angVels[numObjects];
SimdVector3 linVels[numObjects];

PolyhedralConvexShape*	shapePtr[maxNumObjects];


SimdTransform	fromTrans[maxNumObjects];
SimdTransform	toTrans[maxNumObjects];

//SimdTransform tr[numObjects];

void DrawRasterizerLine(float const* , float const*, int)
{

}
int screenWidth = 640.f;
int screenHeight = 480.f;


int main(int argc,char** argv)
{

	setCameraDistance(40.f);

	fromTrans[0].setOrigin(SimdVector3(0,10,20));
	  toTrans[0].setOrigin(SimdVector3(0,10,-20));
	fromTrans[1].setOrigin(SimdVector3(-2,7,0));
	  toTrans[1].setOrigin(SimdVector3(-2,10,0));

	  SimdMatrix3x3 identBasis;
	identBasis.setIdentity();

	SimdMatrix3x3 basisA;
	basisA.setIdentity();
	basisA.setEulerZYX(0.f,-SIMD_HALF_PI,0.f);

	fromTrans[0].setBasis(identBasis);
	  toTrans[0].setBasis(basisA);

	fromTrans[1].setBasis(identBasis);
	  toTrans[1].setBasis(identBasis);

	toTrans[1].setBasis(identBasis);
	SimdVector3 boxHalfExtentsA(10,1,1);
	SimdVector3 boxHalfExtentsB(1.1f,1.1f,1.1f);
	BoxShape	boxA(boxHalfExtentsA);
//	BU_Simplex1to4 boxA(SimdPoint3(-2,0,-2),SimdPoint3(2,0,-2),SimdPoint3(0,0,2),SimdPoint3(0,2,0));
//	BU_Simplex1to4 boxA(SimdPoint3(-12,0,0),SimdPoint3(12,0,0));
	

	BoxShape	boxB(boxHalfExtentsB);
//	BU_Simplex1to4 boxB(SimdPoint3(0,10,0),SimdPoint3(0,-10,0));


	shapePtr[0] = &boxA;
	shapePtr[1] = &boxB;

	shapePtr[0]->SetMargin(0.01f);
	shapePtr[1]->SetMargin(0.01f);

	for (int i=0;i<numObjects;i++)
	{
		SimdTransformUtil::CalculateVelocity(fromTrans[i],toTrans[i],1.f,linVels[i],angVels[i]);
	}

	return glutmain(argc, argv,screenWidth,screenHeight,"Continuous Convex Collision Demo");
}

//to be implemented by the demo

void clientMoveAndDisplay()
{
	clientDisplay();
}

#include "NarrowPhaseCollision/VoronoiSimplexSolver.h"
#include "NarrowPhaseCollision/ConvexPenetrationDepthSolver.h"

static VoronoiSimplexSolver sVoronoiSimplexSolver;

SimplexSolverInterface& gGjkSimplexSolver = sVoronoiSimplexSolver;

bool drawLine= false;

int minlines = 0;

int maxlines = 512;


void clientDisplay(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glDisable(GL_LIGHTING);

	//GL_ShapeDrawer::DrawCoordSystem();

	float m[16];
	int i;

	/*for (i=0;i<numObjects;i++)
	{
		fromTrans[i].getOpenGLMatrix( m );
		GL_ShapeDrawer::DrawOpenGL(m,shapePtr[i]);
	}
*/

	if (getDebugMode()==IDebugDraw::DBG_DrawAabb)
	{
		i=0;//for (i=1;i<numObjects;i++)
		{
			SimdScalar dt = 1.f;
			SimdScalar boundingRadius = shapePtr[i]->GetAngularMotionDisc();
			SimdScalar angspeed = angVels[i].length() * boundingRadius * dt;
			SimdScalar linspeed = linVels[i].length() * dt;

			SimdScalar	totalspeed = angspeed + linspeed;

			//for each object, subdivide the from/to transform in 10 equal steps

			int numSubSteps = 10.f;
			for (int s=0;s<10;s++)
			{
				SimdScalar subStep = s * 1.f/(float)numSubSteps;
				SimdTransform interpolatedTrans;
				
				SimdTransformUtil::IntegrateTransform(fromTrans[i],linVels[i],angVels[i],subStep,interpolatedTrans);

				//fromTrans[i].getOpenGLMatrix(m);
				//GL_ShapeDrawer::DrawOpenGL(m,shapePtr[i]);

				//toTrans[i].getOpenGLMatrix(m);
				//GL_ShapeDrawer::DrawOpenGL(m,shapePtr[i]);

				interpolatedTrans.getOpenGLMatrix( m );
				GL_ShapeDrawer::DrawOpenGL(m,shapePtr[i],SimdVector3(1,0,1),getDebugMode());
			}
		}
	}

	
	int shapeIndex = 1;

	SimdMatrix3x3 mat;
	mat.setEulerZYX(yaw,pitch,roll);
	SimdQuaternion orn;
	mat.getRotation(orn);
	orn.setEuler(yaw,pitch,roll);
	fromTrans[1].setRotation(orn);
	toTrans[1].setRotation(orn);
	
	extern bool stepping;
	extern bool singleStep;

	if (stepping || singleStep)
	{
		singleStep = false;
		pitch += 0.005f;
//		yaw += 0.01f;
	}
//	SimdVector3 fromA(-25,11,0);
//	SimdVector3 toA(-15,11,0);

//	SimdQuaternion ornFromA(0.f,0.f,0.f,1.f);
//	SimdQuaternion ornToA(0.f,0.f,0.f,1.f);

//	SimdTransform	rayFromWorld(ornFromA,fromA);
//	SimdTransform	rayToWorld(ornToA,toA);

	SimdTransform	rayFromWorld = fromTrans[0];
	SimdTransform	rayToWorld = toTrans[0];
	

	if (drawLine)
	{
		glBegin(GL_LINES);
		glColor3f(0, 0, 1);
		glVertex3d(rayFromWorld.getOrigin().x(), rayFromWorld.getOrigin().y(),rayFromWorld.getOrigin().z());
		glVertex3d(rayToWorld.getOrigin().x(),rayToWorld.getOrigin().y(),rayToWorld.getOrigin().z());
		glEnd();
	}

	//now perform a raycast on the shapes, in local (shape) space
	gGjkSimplexSolver.reset();
	
	//choose one of the following lines


	for (i=0;i<numObjects;i++)
	{	
		fromTrans[i].getOpenGLMatrix(m);
		GL_ShapeDrawer::DrawOpenGL(m,shapePtr[i],SimdVector3(1,1,1),getDebugMode());
	}

	DebugCastResult	rayResult1(fromTrans[0],shapePtr[0],linVels[0],angVels[0]);
	

	for (i=1;i<numObjects;i++)
	{
		ConvexCast::CastResult	rayResult2;
		ConvexCast::CastResult*	rayResultPtr;
		if (IDebugDraw::DBG_DrawAabb)
		{
			rayResultPtr = &rayResult1;
		} else
		{
			rayResultPtr = &rayResult2;
		}

		//GjkConvexCast	convexCaster(&gGjkSimplexSolver);
		//SubsimplexConvexCast convexCaster(&gGjkSimplexSolver);

		//optional
		ConvexPenetrationDepthSolver* penetrationDepthSolver = 0;
		ContinuousConvexCollision convexCaster(shapePtr[0],shapePtr[i],&gGjkSimplexSolver,penetrationDepthSolver );

		gGjkSimplexSolver.reset();
	
		
	
		if (convexCaster.calcTimeOfImpact(fromTrans[0],toTrans[0],fromTrans[i] ,toTrans[i] ,*rayResultPtr))
		{

			glDisable(GL_DEPTH_TEST);

			SimdTransform hitTrans;
			SimdTransformUtil::IntegrateTransform(fromTrans[0],linVels[0],angVels[0],rayResultPtr->m_fraction,hitTrans);

			hitTrans.getOpenGLMatrix(m);
			GL_ShapeDrawer::DrawOpenGL(m,shapePtr[0],SimdVector3(0,1,0),getDebugMode());

			SimdTransformUtil::IntegrateTransform(fromTrans[i],linVels[i],angVels[i],rayResultPtr->m_fraction,hitTrans);

			hitTrans.getOpenGLMatrix(m);
			GL_ShapeDrawer::DrawOpenGL(m,shapePtr[i],SimdVector3(0,1,1),getDebugMode());
	

		}
	}

	glFlush();
    glutSwapBuffers();
}

void clientResetScene()
{
}

void clientKeyboard(unsigned char key, int x, int y)
{
	defaultKeyboard(key, x, y);
}


void clientMouseFunc(int button, int state, int x, int y)
{

}
void	clientMotionFunc(int x,int y)
{
}
