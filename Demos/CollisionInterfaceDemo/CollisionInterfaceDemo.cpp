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
/// CollisionInterfaceDemo shows high level usage of the Collision Detection.
///

#include "GL_Simplex1to4.h"
#include "SimdQuaternion.h"
#include "SimdTransform.h"
#include "CollisionShapes/ConvexHullShape.h"

#include "CollisionShapes/BoxShape.h"



#include "CollisionDispatch/CollisionWorld.h"
#include "CollisionDispatch/CollisionObject.h"
#include "CollisionDispatch/CollisionDispatcher.h"
#include "BroadphaseCollision/SimpleBroadphase.h"
#include "BroadphaseCollision/AxisSweep3.h"



#include "GL_ShapeDrawer.h"
#ifdef WIN32 //needed for glut.h
#include <windows.h>
#endif
#include <GL/glut.h>
#include "GlutStuff.h"


float yaw=0.f,pitch=0.f,roll=0.f;
const int maxNumObjects = 4;
const int numObjects = 2;

GL_Simplex1to4 simplex;


CollisionObject	objects[maxNumObjects];
CollisionWorld*	collisionWorld = 0;

int screenWidth = 640.f;
int screenHeight = 480.f;


int main(int argc,char** argv)
{
	clientResetScene();

	SimdMatrix3x3 basisA;
	basisA.setIdentity();

	SimdMatrix3x3 basisB;
	basisB.setIdentity();

	objects[0].m_worldTransform.setBasis(basisA);
	objects[1].m_worldTransform.setBasis(basisB);

	SimdPoint3	points0[3]={SimdPoint3(1,0,0),SimdPoint3(0,1,0),SimdPoint3(0,0,1)};
	SimdPoint3	points1[5]={SimdPoint3(1,0,0),SimdPoint3(0,1,0),SimdPoint3(0,0,1),SimdPoint3(0,0,-1),SimdPoint3(-1,-1,0)};
	
	BoxShape boxA(SimdVector3(1,1,1));
	BoxShape boxB(SimdVector3(0.5,0.5,0.5));
	//ConvexHullShape	hullA(points0,3);
	//hullA.setLocalScaling(SimdVector3(3,3,3));
	//ConvexHullShape	hullB(points1,4);
	//hullB.setLocalScaling(SimdVector3(4,4,4));


	objects[0].m_collisionShape = &boxA;//&hullA;
	objects[1].m_collisionShape = &boxB;//&hullB;

	CollisionDispatcher dispatcher;
	//SimpleBroadphase	broadphase;
	SimdVector3	worldAabbMin(-1000,-1000,-1000);
	SimdVector3	worldAabbMax(1000,1000,1000);

	AxisSweep3	broadphase(worldAabbMin,worldAabbMax);

	collisionWorld = new CollisionWorld(&dispatcher,&broadphase);
	
	collisionWorld->AddCollisionObject(&objects[0]);
	collisionWorld->AddCollisionObject(&objects[1]);

	return glutmain(argc, argv,screenWidth,screenHeight,"Collision Interface Demo");
}

//to be implemented by the demo

void clientMoveAndDisplay()
{
	
	clientDisplay();
}


static VoronoiSimplexSolver sGjkSimplexSolver;
SimplexSolverInterface& gGjkSimplexSolver = sGjkSimplexSolver;



void clientDisplay(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glDisable(GL_LIGHTING);

	if (collisionWorld)
		collisionWorld->PerformDiscreteCollisionDetection();

	///one way to draw all the contact points is iterating over contact manifolds / points:
	int i;
	int numManifolds = collisionWorld->GetDispatcher()->GetNumManifolds();
	for (i=0;i<numManifolds;i++)
	{
		PersistentManifold* contactManifold = collisionWorld->GetDispatcher()->GetManifoldByIndexInternal(i);
		CollisionObject* obA = static_cast<CollisionObject*>(contactManifold->GetBody0());
		CollisionObject* obB = static_cast<CollisionObject*>(contactManifold->GetBody1());
		contactManifold->RefreshContactPoints(obA->m_worldTransform,obB->m_worldTransform);

		int numContacts = contactManifold->GetNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			ManifoldPoint& pt = contactManifold->GetContactPoint(j);

			glBegin(GL_LINES);
			glColor3f(1, 0, 1);
			
			SimdVector3 ptA = pt.GetPositionWorldOnA();
			SimdVector3 ptB = pt.GetPositionWorldOnB();

			glVertex3d(ptA.x(),ptA.y(),ptA.z());
			glVertex3d(ptB.x(),ptB.y(),ptB.z());
			glEnd();
		}
	}


	//GL_ShapeDrawer::DrawCoordSystem();

	float m[16];
	


	for (i=0;i<numObjects;i++)
	{
		
		objects[i].m_worldTransform.getOpenGLMatrix( m );
		GL_ShapeDrawer::DrawOpenGL(m,objects[i].m_collisionShape,SimdVector3(1,1,1),getDebugMode());

	}


	SimdQuaternion orn;
	orn.setEuler(yaw,pitch,roll);
	objects[1].m_worldTransform.setOrigin(objects[1].m_worldTransform.getOrigin()+SimdVector3(0,-0.01,0));

	//objects[0].m_worldTransform.setRotation(orn);

	pitch += 0.005f;
	yaw += 0.01f;

	glFlush();
    glutSwapBuffers();
}

void clientResetScene()
{
	objects[0].m_worldTransform.setOrigin(SimdVector3(0.0f,3.f,0.f));
	objects[1].m_worldTransform.setOrigin(SimdVector3(0.0f,9.f,0.f));
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