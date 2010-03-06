/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Experimental Buoyancy fluid demo written by John McCutchan
*/

#include "btBulletDynamicsCommon.h"

#include "BulletHfFluid/btHfFluidRigidDynamicsWorld.h"
#include "BulletHfFluid/btHfFluid.h"
#include "BulletHfFluid/btHfFluidRigidCollisionConfiguration.h"
#include "BulletHfFluid/btHfFluidBuoyantConvexShape.h"
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btRandom.h"
#include <stdio.h> //printf debugging
#include "LinearMath/btConvexHull.h"

#include "HfFluidDemo.h"
#include "GL_ShapeDrawer.h"
#include "HfFluidDemo_GL_ShapeDrawer.h"

#include "GlutStuff.h"

extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;

const int maxProxies = 32766;
const int maxOverlap = 65535;

static btVector3*	gGroundVertices=0;
static int*	gGroundIndices=0;
static btBvhTriangleMeshShape* trimeshShape =0;
static btRigidBody* staticBody = 0;
static float waveheight = 5.f;

const float TRIANGLE_SIZE=8.f;

#define ARRAY_SIZE_X 1
#define ARRAY_SIZE_Y 1
#define ARRAY_SIZE_Z 1

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

#define START_POS_X 5
#define START_POS_Y -5
#define START_POS_Z 3

unsigned int current_draw_mode=DRAWMODE_NORMAL;
unsigned int current_body_draw_mode = 0;
unsigned	current_demo=0;

void Init_Floatyness (HfFluidDemo* fluidDemo)
{
	btHfFluid* fluid = NULL;

	fluid = new btHfFluid (btScalar(0.25), 100, 100);
	btTransform xform;
	xform.setIdentity ();
	xform.getOrigin() = btVector3(btScalar(-10.0), btScalar(-5.0), btScalar(-10.0));
	fluid->setWorldTransform (xform);
	fluid->setHorizontalVelocityScale (btScalar(0.0f));
	fluid->setVolumeDisplacementScale (btScalar(0.0f));
	fluidDemo->getHfFluidDynamicsWorld()->addHfFluid (fluid);

	for (int i = 0; i < fluid->getNumNodesLength()*fluid->getNumNodesWidth(); i++)
	{
		fluid->setFluidHeight(i, btScalar(5.0f));
	}

	fluid->prep ();

	const int numObjects = 5;
	btScalar floatyness = btScalar(1.0f);
	btScalar dfloatyness = btScalar(0.25f);
	btScalar start_x = btScalar(-5.0f);
	btScalar step_x = btScalar(3.0f);
	btScalar start_z = btScalar(-5.0f);
	for (int i = 0; i < numObjects; i++)
	{
		//btConvexShape* colShape = new btBoxShape(btVector3(1.0, 1.0, 1.0));
		btConvexShape* colShape = new btSphereShape(btScalar(1.));
		btHfFluidBuoyantConvexShape* buoyantShape = new btHfFluidBuoyantConvexShape(colShape);
		buoyantShape->generateShape (btScalar(0.25f), btScalar(0.05f));
		buoyantShape->setFloatyness (floatyness + dfloatyness * i);
		fluidDemo->m_collisionShapes.push_back (buoyantShape);

		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		btVector3 localInertia(0,0,0);
		colShape->calculateLocalInertia(mass,localInertia);

		btVector3 origin = btVector3(step_x * i + start_x, 7.5f, start_z);
		startTransform.setOrigin(origin);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,buoyantShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		fluidDemo->getHfFluidDynamicsWorld()->addRigidBody(body);
	}
	floatyness = btScalar(2.0f);
	start_z = btScalar(5.0f);
	for (int i = 0; i < numObjects; i++)
	{
		//btConvexShape* colShape = new btBoxShape(btVector3(1.0, 1.0, 1.0));
		btConvexShape* colShape = new btSphereShape(btScalar(1.));
		btHfFluidBuoyantConvexShape* buoyantShape = new btHfFluidBuoyantConvexShape(colShape);
		buoyantShape->generateShape (btScalar(0.25f), btScalar(0.05f));
		buoyantShape->setFloatyness (floatyness + dfloatyness * i);
		fluidDemo->m_collisionShapes.push_back (buoyantShape);

		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		btVector3 localInertia(0,0,0);
		colShape->calculateLocalInertia(mass,localInertia);

		btVector3 origin = btVector3(step_x * i + start_x, -4.0f, start_z);
		startTransform.setOrigin(origin);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,buoyantShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		fluidDemo->getHfFluidDynamicsWorld()->addRigidBody(body);
	}
}

void Init_Bowl (HfFluidDemo* fluidDemo)
{
	btHfFluid* fluid = NULL;

	fluid = new btHfFluid (btScalar(1.0), 50, 50);
	btTransform xform;
	xform.setIdentity ();
	xform.getOrigin() = btVector3(btScalar(-10.0), btScalar(-5.0), btScalar(-10.0));
	fluid->setWorldTransform (xform);
	fluidDemo->getHfFluidDynamicsWorld()->addHfFluid (fluid);

	btScalar* ground = fluid->getGroundArray();
	btScalar* eta = fluid->getEtaArray();
	btScalar amplitude = btScalar(200.0);
	for (int i = 0; i < fluid->getNumNodesWidth(); i++)
	{
		btScalar x = btScalar(i - fluid->getNumNodesWidth()/2)/btScalar(fluid->getNumNodesWidth()*2);
		btScalar xh = amplitude * (x * x) + btScalar(5.0);
		for (int j = 0; j < fluid->getNumNodesLength(); j++)
		{
			btScalar y = btScalar(j - fluid->getNumNodesLength()/2)/btScalar(fluid->getNumNodesLength()*2);
			btScalar yh = amplitude * (y * y) + btScalar(5.0);
			btScalar gHeight = btMax(xh,yh);
			int index = fluid->arrayIndex (i, j);
			ground[index] = gHeight;
			btScalar wHeight = btScalar(0.0f);
			if (gHeight > 14.0)
			{
				wHeight = btScalar(0.0f);
			} else {
				wHeight = btScalar(14.0f) - gHeight;
			}
			eta[index] = wHeight;		}
	}

	fluid->prep ();

	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btConvexShape* colShape = new btBoxShape(btVector3(1,1,1));
		btHfFluidBuoyantConvexShape* buoyantShape = new btHfFluidBuoyantConvexShape(colShape);
		buoyantShape->generateShape (btScalar(0.25f), btScalar(0.05f));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		fluidDemo->m_collisionShapes.push_back(colShape);
		fluidDemo->m_collisionShapes.push_back (buoyantShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(btVector3(
										2.0*i + start_x,
										10+2.0*k + start_y,
										2.0*j + start_z));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,buoyantShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					fluidDemo->getHfFluidDynamicsWorld()->addRigidBody(body);
				}
			}
		}
	}
}

void Init_Drops (HfFluidDemo* fluidDemo)
{
	btHfFluid* fluid = NULL;

	fluid = new btHfFluid (btScalar(0.5), 50, 50);
	btTransform xform;
	xform.setIdentity ();
	xform.getOrigin() = btVector3(btScalar(-10.0), btScalar(-5.0), btScalar(-10.0));
	fluid->setWorldTransform (xform);
	fluidDemo->getHfFluidDynamicsWorld()->addHfFluid (fluid);

	for (int i = 0; i < fluid->getNumNodesLength()*fluid->getNumNodesWidth(); i++)
	{
		fluid->setFluidHeight(i, btScalar(5.0f));
	}

	fluid->prep ();
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btConvexShape* colShape = new btBoxShape(btVector3(5,0.5,5));
		btHfFluidBuoyantConvexShape* buoyantShape = new btHfFluidBuoyantConvexShape(colShape);
		buoyantShape->generateShape (btScalar(0.25f), btScalar(0.05f));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		fluidDemo->m_collisionShapes.push_back(colShape);
		fluidDemo->m_collisionShapes.push_back (buoyantShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(btVector3(
										2.0*i + start_x,
										10+2.0*k + start_y,
										2.0*j + start_z));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,buoyantShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					fluidDemo->getHfFluidDynamicsWorld()->addRigidBody(body);
				}
			}
		}
	}
}

void Init_Wave (HfFluidDemo* fluidDemo)
{
	btHfFluid* fluid = NULL;

	fluid = new btHfFluid (btScalar(1.0f), 75, 50);
	btTransform xform;
	xform.setIdentity ();
	xform.getOrigin() = btVector3(btScalar(-50.0), btScalar(-5.0), btScalar(-50.0));
	fluid->setWorldTransform (xform);
	fluidDemo->getHfFluidDynamicsWorld()->addHfFluid (fluid);

	for (int i = 0; i < fluid->getNumNodesLength()*fluid->getNumNodesWidth(); i++)
	{
		fluid->getEtaArray()[i] = btScalar(10.0f);
	}

	for (int i = 1; i < fluid->getNumNodesWidth()-1; i++)
	{
		fluid->getEtaArray()[fluid->arrayIndex (i, fluid->getNumNodesLength()/2-1)] = btScalar (2.0);
		fluid->getEtaArray()[fluid->arrayIndex (i, fluid->getNumNodesLength()/2)] = btScalar (2.0);
		fluid->getEtaArray()[fluid->arrayIndex (i, fluid->getNumNodesLength()/2+1)] = btScalar (2.0);
	}

	fluid->prep ();
}

void Init_RandomDrops (HfFluidDemo* fluidDemo)
{
	btHfFluid* fluid = NULL;

	fluid = new btHfFluid (btScalar(1.0),75, 50);
	btTransform xform;
	xform.setIdentity ();
	xform.getOrigin() = btVector3(btScalar(-50.0), btScalar(-5.0), btScalar(-50.0));
	fluid->setWorldTransform (xform);
	fluidDemo->getHfFluidDynamicsWorld()->addHfFluid (fluid);
	for (int i = 0; i < fluid->getNumNodesLength()*fluid->getNumNodesWidth(); i++)
	{
		fluid->getEtaArray()[i] = btScalar(0.0f);
	}
	fluid->prep ();
}

void Init_FillPool (HfFluidDemo* fluidDemo)
{
	btHfFluid* fluid = NULL;
	const int gridLength = 50;
	const int gridWidth = 50;
	fluid = new btHfFluid (btScalar(1.0), gridLength, gridWidth);
	btTransform xform;
	xform.setIdentity ();
	xform.getOrigin() = btVector3(btScalar(-20.0), btScalar(-5.0), btScalar(-20.0));
	fluid->setWorldTransform (xform);
	fluidDemo->getHfFluidDynamicsWorld()->addHfFluid (fluid);

	btScalar* ground = fluid->getGroundArray();
	btScalar* eta = fluid->getEtaArray();

	const btScalar poolEdgeHeight = btScalar(10.0f);
	const btScalar poolBottomHeight = btScalar(1.0f);
	const btScalar poolPourerHeight = btScalar(6.0f);
	for (int j = 1; j < fluid->getNumNodesLength()-1; j++)
	{
		for (int i = 1; i < fluid->getNumNodesWidth()-1; i++)
		{
			int index = fluid->arrayIndex (i, j);
			// pool edge
			if (j == 1 || i == 1 || j == fluid->getNumNodesLength()-2 || i == fluid->getNumNodesWidth()-2)
			{
				ground[index] = poolEdgeHeight;
				continue;
			}
			if (j > 35)
			{
				if (i <= 25 || i >= 30)
				{
					ground[index] = poolEdgeHeight;
				} else {
					ground[index] = poolPourerHeight;
				}
				continue;
			}
			ground[index] = poolBottomHeight;
			//eta[index] = btScalar(3.0f);
		}
	}
	fluid->prep ();

	{
		btConvexShape* colShape = new btBoxShape(btVector3(btScalar(1.), btScalar(1.), btScalar(1.)));
		btHfFluidBuoyantConvexShape* buoyantShape = new btHfFluidBuoyantConvexShape(colShape);
		buoyantShape->generateShape (btScalar(0.25f), btScalar(0.05f));
		fluidDemo->m_collisionShapes.push_back(colShape);
		fluidDemo->m_collisionShapes.push_back(buoyantShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		int gridSize = 2;
		btScalar startPosX = btScalar(-10.0f);
		btScalar startPosY = btScalar(2.0f);
		btScalar startPosZ = btScalar(-10.f);
		float start_x = startPosX - gridSize/2;
		float start_y = startPosY;
		float start_z = startPosZ - gridSize/2;
		
		for (int k=0;k<gridSize;k++)
		{
			for (int i=0;i<gridSize;i++)
			{
				for(int j = 0;j<gridSize;j++)
				{
					startTransform.setOrigin(btVector3(
										2.0*i + start_x,
										10+2.0*k + start_y,
										2.0*j + start_z));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,buoyantShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					fluidDemo->getHfFluidDynamicsWorld()->addRigidBody(body);
				}
			}
		}
	}
}

void Run_FillPool (HfFluidDemo* fluidDemo)
{
	static btScalar dtSinceLastDrop = btScalar(0.0f);
	btScalar dt = btScalar(1.0/60.);
	btHfFluidArray& fluids = fluidDemo->getHfFluidDynamicsWorld ()->getHfFluidArray ();
	btHfFluid* fluid = fluids[0];

	for (int i = 26; i < 30; i++)
	{
		fluid->setFluidHeight (i, fluid->getNumNodesLength()-3, btScalar(3.0f));
	}
}


void Run_RandomDrops (HfFluidDemo* fluidDemo)
{
	static btScalar dtSinceLastDrop = btScalar(0.0f);
	btScalar dt = btScalar(1.0/60.);	

	
	btHfFluidArray& fluids = fluidDemo->getHfFluidDynamicsWorld ()->getHfFluidArray ();
	btHfFluid* fluid = fluids[0];

	if (dtSinceLastDrop > btScalar(0.5f))
	{
		dtSinceLastDrop = btScalar(0.0f);
		int randomXNode = GEN_rand () % (fluid->getNumNodesWidth()-2);
		int randomZNode = GEN_rand () % (fluid->getNumNodesLength()-2);
		if (randomXNode <= 1)
			randomXNode = 2;
		if (randomZNode <= 1)
			randomZNode = 2;

		btScalar* eta = fluid->getEtaArray ();
		btScalar* height = fluid->getHeightArray ();
		const btScalar* ground = fluid->getGroundArray ();
		bool* flags = fluid->getFlagsArray();
		int index = fluid->arrayIndex (randomXNode, randomZNode);
		eta[index] += btScalar(4.5f);
		eta[index-1] += btScalar(2.25f);
		eta[index+1] += btScalar(2.25f);
		eta[index+fluid->getNumNodesWidth()] += btScalar(2.25f);
		eta[index-fluid->getNumNodesWidth()] += btScalar(2.25f);
		height[index] = eta[index] + ground[index];
		height[index-1] = eta[index-1] + ground[index-1];
		height[index+1] = eta[index+1] + ground[index+1];
		height[index+fluid->getNumNodesWidth()] = eta[index+fluid->getNumNodesWidth()] + ground[index+fluid->getNumNodesWidth()];
		height[index-fluid->getNumNodesWidth()] = eta[index-fluid->getNumNodesWidth()] + ground[index-fluid->getNumNodesWidth()];
		flags[index] = true;
		flags[index-1] = true;
		flags[index+1] = true;
		flags[index+fluid->getNumNodesWidth()] = true;
		flags[index-fluid->getNumNodesWidth()] = true;
	} else {
		dtSinceLastDrop += dt;
	}
}

void Init_Fill (HfFluidDemo* fluidDemo)
{
	btHfFluid* fluid = NULL;

	fluid = new btHfFluid (btScalar(1.0f), 75, 50);
	btTransform xform;
	xform.setIdentity ();
	xform.getOrigin() = btVector3(btScalar(-50.0), btScalar(-5.0), btScalar(-50.0));
	fluid->setWorldTransform (xform);
	fluidDemo->getHfFluidDynamicsWorld()->addHfFluid (fluid);
	for (int i = 0; i < fluid->getNumNodesLength()*fluid->getNumNodesWidth(); i++)
	{
		fluid->getEtaArray()[i] = btScalar(0.0f);
	}
	fluid->prep ();
}

void Run_Fill (HfFluidDemo* fluidDemo)
{
	static btScalar dtSinceLastDrop = btScalar(0.0f);
	btScalar dt = btScalar(1.0/60.);

	btHfFluidArray& fluids = fluidDemo->getHfFluidDynamicsWorld ()->getHfFluidArray ();
	btHfFluid* fluid = fluids[0];

	if (dtSinceLastDrop > btScalar(0.25f))
	{
		dtSinceLastDrop = btScalar(0.0f);

		btScalar* eta = fluid->getEtaArray ();
		btScalar* velocityU = fluid->getVelocityUArray ();
		btScalar* velocityV = fluid->getVelocityVArray ();
		btScalar* height = fluid->getHeightArray ();
		const btScalar* ground = fluid->getGroundArray ();
		bool* flags = fluid->getFlagsArray();
		int index = fluid->arrayIndex (fluid->getNumNodesWidth()/2, fluid->getNumNodesLength()/2);
		eta[index] += btScalar(4.5f);
		eta[index-1] += btScalar(2.25f);
		eta[index+1] += btScalar(2.25f);
		eta[index+fluid->getNumNodesWidth()] += btScalar(2.25f);
		eta[index-fluid->getNumNodesWidth()] += btScalar(2.25f);

		velocityU[index] = btScalar(0.0f);
		velocityU[index-1] = btScalar(-10.0f);
		velocityU[index+1] = btScalar(10.0f);
		velocityU[index+fluid->getNumNodesWidth()] = btScalar(0.0f);
		velocityU[index-fluid->getNumNodesWidth()] = btScalar(0.0f);

		velocityV[index] = btScalar(0.0f);
		velocityV[index-1] = btScalar(0.0f);
		velocityV[index+1] = btScalar(0.0f);
		velocityV[index+fluid->getNumNodesWidth()] = btScalar(10.0f);
		velocityV[index-fluid->getNumNodesWidth()] = btScalar(-10.0f);

		height[index] = eta[index] + ground[index];
		height[index-1] = eta[index-1] + ground[index-1];
		height[index+1] = eta[index+1] + ground[index+1];
		height[index+fluid->getNumNodesWidth()] = eta[index+fluid->getNumNodesWidth()] + ground[index+fluid->getNumNodesWidth()];
		height[index-fluid->getNumNodesWidth()] = eta[index-fluid->getNumNodesWidth()] + ground[index-fluid->getNumNodesWidth()];
		flags[index] = true;
		flags[index-1] = true;
		flags[index+1] = true;
		flags[index+fluid->getNumNodesWidth()] = true;
		flags[index-fluid->getNumNodesWidth()] = true;
	} else {
		dtSinceLastDrop += dt;
	}
	
}

void Init_BlockWave (HfFluidDemo* fluidDemo)
{
	btHfFluid* fluid = NULL;

	fluid = new btHfFluid (btScalar(1.0), 75, 50);
	btTransform xform;
	xform.setIdentity ();
	xform.getOrigin() = btVector3(btScalar(-50.0), btScalar(-5.0), btScalar(-50.0));
	fluid->setWorldTransform (xform);
	fluidDemo->getHfFluidDynamicsWorld()->addHfFluid (fluid);

	btScalar* eta = fluid->getEtaArray ();

	for (int i = 0; i < fluid->getNumNodesLength() * fluid->getNumNodesWidth(); i++)
	{
		eta[i] = btScalar(12.0f);
	}

	for (int i = fluid->getNumNodesWidth()/8; i < fluid->getNumNodesWidth()/4; i++)
	{
		for (int j = fluid->getNumNodesLength()/8; j < fluid->getNumNodesLength()/4; j++)
		{
			int index = fluid->arrayIndex(i, j);
			eta[index] = btScalar(4.0f);
		}
	}
	fluid->prep ();

	{
		btConvexShape* colShape = new btSphereShape(btScalar(1.));
		btHfFluidBuoyantConvexShape* buoyantShape = new btHfFluidBuoyantConvexShape(colShape);
		buoyantShape->generateShape (btScalar(0.25f), btScalar(0.05f));
		fluidDemo->m_collisionShapes.push_back(buoyantShape);
		fluidDemo->m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		int gridSize = 2;
		btScalar startPosX = btScalar(-10.0f);
		btScalar startPosY = btScalar(2.0f);
		btScalar startPosZ = btScalar(-10.f);
		float start_x = startPosX - gridSize/2;
		float start_y = startPosY;
		float start_z = startPosZ - gridSize/2;
		
		for (int k=0;k<gridSize;k++)
		{
			for (int i=0;i<gridSize;i++)
			{
				for(int j = 0;j<gridSize;j++)
				{
					startTransform.setOrigin(btVector3(
										2.0*i + start_x,
										10+2.0*k + start_y,
										2.0*j + start_z));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,buoyantShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					fluidDemo->getHfFluidDynamicsWorld()->addRigidBody(body);
				}
			}
		}
	}
}

void Init_Ground (HfFluidDemo* fluidDemo)
{
	btHfFluid* fluid = NULL;

	fluid = new btHfFluid (btScalar(1.0f),75, 50);
	btTransform xform;
	xform.setIdentity ();
	xform.getOrigin() = btVector3(btScalar(-50.0), btScalar(5.0), btScalar(-50.0));
	fluid->setWorldTransform (xform);
	fluidDemo->getHfFluidDynamicsWorld()->addHfFluid (fluid);

	btScalar* eta = fluid->getEtaArray ();

	for (int i = 0; i < fluid->getNumNodesLength() * fluid->getNumNodesWidth(); i++)
	{
		eta[i] = btScalar(4.0f);
	}

	btScalar* ground = fluid->getGroundArray ();
	for (int i = 0; i < fluid->getNumNodesWidth(); i++)
	{
		for (int j = 0; j < fluid->getNumNodesLength(); j++)
		{
			int index = fluid->arrayIndex (i, j);

			if (j <= fluid->getNumNodesLength()/2)
			{
				
				ground[index] = btScalar(5.0f);
			} else if (j > (fluid->getNumNodesLength()/8*6)) { 
				ground[index] = btScalar(0.0f);
			} else {
				ground[index] = btScalar(6.5f);
			}

			if (j <= fluid->getNumNodesLength()/4 && j > fluid->getNumNodesLength()/8)
			{
				eta[index] = btScalar(8.0f);
			} else if (j <= fluid->getNumNodesLength()/8)
			{
				eta[index] = btScalar(20.0f);
			} else {
				eta[index] = btScalar(0.0f);
			}
			
		}
	}
	fluid->prep ();
}

void Init_Ground2 (HfFluidDemo* fluidDemo)
{
	btHfFluid* fluid = NULL;

	fluid = new btHfFluid (btScalar(1.0f), 75, 50);
	btTransform xform;
	xform.setIdentity ();
	xform.getOrigin() = btVector3(btScalar(-50.0), btScalar(5.0), btScalar(-50.0));
	fluid->setWorldTransform (xform);
	fluidDemo->getHfFluidDynamicsWorld()->addHfFluid (fluid);

	btScalar* eta = fluid->getEtaArray ();

	for (int i = 0; i < fluid->getNumNodesLength() * fluid->getNumNodesWidth(); i++)
	{
		eta[i] = btScalar(4.0f);
	}

	btScalar* ground = fluid->getGroundArray ();
	for (int i = 0; i < fluid->getNumNodesWidth(); i++)
	{
		for (int j = 0; j < fluid->getNumNodesLength(); j++)
		{
			int index = fluid->arrayIndex (i, j);

			ground[index] = (btScalar(j)/fluid->getNumNodesLength()-1)*btScalar(8.0f);
		}
	}

	for (int i = 0; i < fluid->getNumNodesLength() * fluid->getNumNodesWidth(); i++)
	{
		eta[i] = btScalar(2.0f);
	}

	for (int i = fluid->getNumNodesWidth()/8; i < fluid->getNumNodesWidth()/4; i++)
	{
		for (int j = fluid->getNumNodesLength()/8; j < fluid->getNumNodesLength()/4; j++)
		{
			int index = fluid->arrayIndex(i, j);
			eta[index] = btScalar(8.0f);
		}
	}
	fluid->prep ();
}

void Init_Fill2 (HfFluidDemo* fluidDemo)
{
	btHfFluid* fluid = NULL;

	fluid = new btHfFluid (btScalar(1.0), 100, 100);
	btTransform xform;
	xform.setIdentity ();
	xform.getOrigin() = btVector3(btScalar(-50.0), btScalar(-5.0), btScalar(-50.0));
	fluid->setWorldTransform (xform);
	fluidDemo->getHfFluidDynamicsWorld()->addHfFluid (fluid);
	for (int i = 0; i < fluid->getNumNodesLength()*fluid->getNumNodesWidth(); i++)
	{
		fluid->getEtaArray()[i] = btScalar(0.0f);
	}
	fluid->prep ();
}

void Run_Fill2 (HfFluidDemo* fluidDemo)
{
	static btScalar dtSinceLastDrop = btScalar(0.0f);
	btScalar dt = btScalar(1.0/60.);

	btHfFluidArray& fluids = fluidDemo->getHfFluidDynamicsWorld ()->getHfFluidArray ();
	btHfFluid* fluid = fluids[0];

	if (dtSinceLastDrop > btScalar(0.25f))
	{
		dtSinceLastDrop = btScalar(0.0f);

		btScalar* eta = fluid->getEtaArray ();
		btScalar* velocityU = fluid->getVelocityUArray ();
		btScalar* velocityV = fluid->getVelocityVArray ();
		btScalar* height = fluid->getHeightArray ();
		const btScalar* ground = fluid->getGroundArray ();
		bool* flags = fluid->getFlagsArray();

		for (int i = 1; i < fluid->getNumNodesWidth()-1; i++)
		{
			int index = fluid->arrayIndex (i, 1);
			eta[index] += btScalar(3.0f);
			velocityU[index] = btScalar(4.0f);
			height[index] = ground[index] + eta[index];
			flags[index] = true;
		}
	} else {
		dtSinceLastDrop += dt;
	}
	
}

void Init_MovingPour (HfFluidDemo* fluidDemo)
{
	btHfFluid* fluid = NULL;

	fluid = new btHfFluid (btScalar(1.0),75, 50);
	btTransform xform;
	xform.setIdentity ();
	xform.getOrigin() = btVector3(btScalar(-50.0), btScalar(-5.0), btScalar(-50.0));
	fluid->setWorldTransform (xform);
	fluidDemo->getHfFluidDynamicsWorld()->addHfFluid (fluid);

	for (int i = 0; i < fluid->getNumNodesLength()*fluid->getNumNodesWidth(); i++)
	{
		fluid->getEtaArray()[i] = btScalar(5.0f);
	}
	

	fluid->prep ();
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		fluidDemo->m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(btVector3(
										2.0*i + start_x,
										10+2.0*k + start_y,
										2.0*j + start_z));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					fluidDemo->getHfFluidDynamicsWorld()->addRigidBody(body);
				}
			}
		}
	}
}

void Run_MovingPour(HfFluidDemo* fluidDemo)
{
	static btScalar dtSinceLastDrop = btScalar(0.0f);
	static btScalar x = 4;
	static btScalar z = 4;
	static btScalar dx = btScalar(20.0f);
	static btScalar dz = btScalar(30.0f);
	btScalar dt = btScalar(1.0/60.);

	btHfFluidArray& fluids = fluidDemo->getHfFluidDynamicsWorld ()->getHfFluidArray ();
	btHfFluid* fluid = fluids[0];

	int minX = 2;
	int minZ = 2;
	int maxX = fluid->getNumNodesWidth() - 2;
	int maxZ = fluid->getNumNodesLength() - 2;

	x += dx * dt;
	
	if (x <= minX)
	{
		dx *= btScalar(-1.0f);
		x = minX;
	} else if (x >= maxX) {
		dx *= btScalar(-1.0f);
		x = maxX;
	}
	z += dz * dt;
	
	if (z <= minZ)
	{
		dz *= btScalar(-1.0f);
		z = minZ;
	} else if (z >= maxZ) {
		dz *= btScalar(-1.0f);
		z = maxZ;
	}

	const btScalar dropHeight = btScalar(3.0f);
	
	{
		int iX = (int)x;
		int iZ = (int)z;
		fluid->addFluidHeight (iX,iZ, dropHeight);
		//fluid->addFluidHeight (x, z+1, dropHeight);
		//fluid->addFluidHeight (x+1, z, dropHeight);
		//fluid->addFluidHeight (x+1, z+1, dropHeight);
	}
}

#define NUM_DEMOS 12

void (*demo_run_functions[NUM_DEMOS])(HfFluidDemo*)=
{
	NULL, // Run_Floatyness
	NULL, // Run_Bowl
	Run_FillPool, //Run_FillPool
	NULL, // Run_Drops
	NULL, // Run_Wave
	Run_RandomDrops,
	Run_Fill,
	Run_Fill2,
	NULL, // Run_BlockWave
	NULL, // Run_Ground
	NULL, // Run_Ground2
	Run_MovingPour,
};
void (*demo_init_functions[NUM_DEMOS])(HfFluidDemo*)=
{
	Init_Floatyness,
	Init_Bowl,
	Init_FillPool,
	Init_Drops,
	Init_Wave,
	Init_RandomDrops,
	Init_Fill,
	Init_Fill2,
	Init_BlockWave,
	Init_Ground,
	Init_Ground2,
	Init_MovingPour,
};

btScalar g_ele_array[NUM_DEMOS] = {
	btScalar(10),
	btScalar(45),
	btScalar(35),
	btScalar(35),
	btScalar(10),
	btScalar(10),
	btScalar(35),
	btScalar(45),
	btScalar(35),
	btScalar(20),
	btScalar(20),
};

btScalar g_azi_array[NUM_DEMOS] = {
	btScalar(0),
	btScalar(55),
	btScalar(245),
	btScalar(270),
	btScalar(55),
	btScalar(55),
	btScalar(180),
	btScalar(205),
	btScalar(255),
	btScalar(305),
	btScalar(305),
};

btScalar g_cameraDistance_array[NUM_DEMOS] = {
	btScalar(20),
	btScalar(29),
	btScalar(43),
	btScalar(26),
	btScalar(77),
	btScalar(77),
	btScalar(77),
	btScalar(32),
	btScalar(62),
	btScalar(70),
	btScalar(70),
};

#ifdef _DEBUG
const int gNumObjects = 1;
#else
const int gNumObjects = 1;//try this in release mode: 3000. never go above 16384, unless you increate maxNumObjects  value in DemoApplication.cp
#endif

const int maxNumObjects = 32760;

#define CUBE_HALF_EXTENTS 1.5
#define EXTRA_HEIGHT -10.f

//
void HfFluidDemo::createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos )
{
	btTransform trans;
	trans.setIdentity();

	for(int i=0; i<size; i++)
	{
		// This constructs a row, from left to right
		int rowSize = size - i;
		for(int j=0; j< rowSize; j++)
		{
			btVector3 pos;
			pos.setValue(
				-rowSize * halfCubeSize + halfCubeSize + j * 2.0f * halfCubeSize,
				halfCubeSize + i * halfCubeSize * 2.0f,
				zPos);

			trans.setOrigin(pos);
			btScalar mass = 1.f;

			btRigidBody* body = 0;
			body = localCreateRigidBody(mass,trans,boxShape);
		}
	}
}


void HfFluidDemo::setShootBoxShape ()
{
	if (!m_shootBoxShape)
	{
		m_shootBoxShape = new btBoxShape(btVector3(0.3f,1.f,0.2f));
		btHfFluidBuoyantConvexShape* buoyantShape = new btHfFluidBuoyantConvexShape((btConvexShape*)m_shootBoxShape);
		buoyantShape->generateShape (btScalar(0.25f), btScalar(0.05f));
		m_shootBoxShape = buoyantShape;
	}
}

////////////////////////////////////

extern int gNumManifold;
extern int gOverlappingPairs;

void HfFluidDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT); 


	float dt = 1.0/60.;	

	if (m_dynamicsWorld)
	{	
		if (demo_run_functions[current_demo])
		{
			demo_run_functions[current_demo](this);
		}
	}

	if (m_dynamicsWorld)
	{
	if(m_drag)
		{
		const int				x=m_lastmousepos[0];
		const int				y=m_lastmousepos[1];
		const btVector3			rayFrom=m_cameraPosition;
		const btVector3			rayTo=getRayTo(x,y);
		const btVector3			rayDir=(rayTo-rayFrom).normalized();
		const btVector3			N=(m_cameraTargetPosition-m_cameraPosition).normalized();
		const btScalar			O=btDot(m_impact,N);
		const btScalar			den=btDot(N,rayDir);
		if((den*den)>0)
			{
			const btScalar			num=O-btDot(N,rayFrom);
			const btScalar			hit=num/den;
			if((hit>0)&&(hit<1500))
				{				
				m_goal=rayFrom+rayDir*hit;
				}				
			}		
		btVector3				delta;
		static const btScalar	maxdrag=10;
		if(delta.length2()>(maxdrag*maxdrag))
			{
			delta=delta.normalized()*maxdrag;
			}
		}
	
#define FIXED_STEP
#ifdef FIXED_STEP
		m_dynamicsWorld->stepSimulation(dt=1.0f/60.f,0);

#else
		//during idle mode, just run 1 simulation step maximum, otherwise 4 at max
		int maxSimSubSteps = m_idle ? 1 : 4;
		//if (m_idle)
		//	dt = 1.0/420.f;

		int numSimSteps;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt);

#ifdef VERBOSE_TIMESTEPPING_CONSOLEOUTPUT
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_TIMESTEPPING_CONSOLEOUTPUT

#endif		

		//optional but useful: debug drawing
		
	}

#ifdef USE_QUICKPROF 
	btProfiler::beginBlock("render"); 
#endif //USE_QUICKPROF 

	renderme(); 

	//render the graphics objects, with center of mass shift

	updateCamera();



#ifdef USE_QUICKPROF 
	btProfiler::endBlock("render"); 
#endif 
	glFlush();
	//some additional debugging info
#ifdef PRINT_CONTACT_STATISTICS
	printf("num manifolds: %i\n",gNumManifold);
	printf("num gOverlappingPairs: %i\n",gOverlappingPairs);
	printf("num gTotalContactPoints : %i\n",gTotalContactPoints );
#endif //PRINT_CONTACT_STATISTICS

	//gTotalContactPoints = 0;
	glutSwapBuffers();

}



void HfFluidDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	renderme();

	glFlush();
	glutSwapBuffers();
}




void	HfFluidDemo::clientResetScene()
{
	DemoApplication::clientResetScene();
	/* Clean up	*/ 
	for(int i=m_dynamicsWorld->getNumCollisionObjects()-1;i>0;i--)
	{
		btCollisionObject*	obj=m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody*		body=btRigidBody::upcast(obj);
		if(body&&body->getMotionState())
		{
			delete body->getMotionState();
		}
		while(m_dynamicsWorld->getNumConstraints())
			{
			btTypedConstraint*	pc=m_dynamicsWorld->getConstraint(0);
			m_dynamicsWorld->removeConstraint(pc);
			delete pc;
			}
		btHfFluid* hfFluid = btHfFluid::upcast(obj);
		if (hfFluid)
		{
			getHfFluidDynamicsWorld()->removeHfFluid(hfFluid);
		} else
		{
			m_dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}
	
		/* Init		*/ 


	m_autocam						=	false;
	m_raycast						=	false;
	m_cutting						=	false;
	printf("current_demo = %d\n", current_demo);
	m_azi = g_azi_array[current_demo];
	m_ele = g_ele_array[current_demo];
	m_cameraDistance = g_cameraDistance_array[current_demo];
	updateCamera();
	demo_init_functions[current_demo](this);
}

void	HfFluidDemo::renderme()
{
	btIDebugDraw*	idraw=m_dynamicsWorld->getDebugDrawer();
	
	m_dynamicsWorld->debugDrawWorld();
	
	/* Bodies		*/ 
	btVector3	ps(0,0,0);
	int			nps=0;

	DemoApplication::renderme();	
}


void	HfFluidDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch(key)
	{
	case	']':
		current_demo = (current_demo+1)%NUM_DEMOS;
		clientResetScene();
	break;
	case	'[':
		current_demo = (current_demo-1)%NUM_DEMOS;
		clientResetScene();
	break;
	case	'.':
		current_draw_mode = (current_draw_mode+1) % DRAWMODE_MAX;
		getHfFluidDynamicsWorld()->setDrawMode (current_draw_mode);
	break;
	case	'v':
		current_body_draw_mode = (current_body_draw_mode+1) % BODY_DRAWMODE_MAX;
		getHfFluidDynamicsWorld()->setBodyDrawMode (current_body_draw_mode);
	break;
	default:
		DemoApplication::keyboardCallback(key,x,y);
	break;
	}
}

//
void	HfFluidDemo::mouseMotionFunc(int x,int y)
{
	DemoApplication::mouseMotionFunc(x,y);
}

//
void	HfFluidDemo::mouseFunc(int button, int state, int x, int y)
{
if(button==0)
	{
	switch(state)
		{
		case	0:
			{
				DemoApplication::mouseFunc(button,state,x,y);
			}
		break;
		case	1:
				DemoApplication::mouseFunc(button,state,x,y);
		break;
		}
	}
	else
	{
		DemoApplication::mouseFunc(button,state,x,y);
	}
}


void	HfFluidDemo::initPhysics()
{
///create concave ground mesh

	btCollisionShape* groundShape = 0;
	bool useConcaveMesh = false;//not ready yet true;

	if (useConcaveMesh)
	{
		int i;
		int j;

		const int NUM_VERTS_X = 30;
		const int NUM_VERTS_Y = 30;
		const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
		const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

		gGroundVertices = new btVector3[totalVerts];
		gGroundIndices = new int[totalTriangles*3];

		btScalar offset(-50);

		for ( i=0;i<NUM_VERTS_X;i++)
		{
			for (j=0;j<NUM_VERTS_Y;j++)
			{
				gGroundVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
					//0.f,
					waveheight*sinf((float)i)*cosf((float)j+offset),
					(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
			}
		}

		int vertStride = sizeof(btVector3);
		int indexStride = 3*sizeof(int);
		
		int index=0;
		for ( i=0;i<NUM_VERTS_X-1;i++)
		{
			for (int j=0;j<NUM_VERTS_Y-1;j++)
			{
				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = j*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

				gGroundIndices[index++] = j*NUM_VERTS_X+i;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
				gGroundIndices[index++] = (j+1)*NUM_VERTS_X+i;
			}
		}

		btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
			gGroundIndices,
			indexStride,
			totalVerts,(btScalar*) &gGroundVertices[0].x(),vertStride);

		bool useQuantizedAabbCompression = true;

		groundShape = new btBvhTriangleMeshShape(indexVertexArrays,useQuantizedAabbCompression);
	} else
	{
		groundShape = new btBoxShape (btVector3(200,CUBE_HALF_EXTENTS,200));
	}

	 
	m_collisionShapes.push_back(groundShape);
	
	btCompoundShape* cylinderCompound = new btCompoundShape;
	btCollisionShape* cylinderShape = new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS));
	btTransform localTransform;
	localTransform.setIdentity();
	cylinderCompound->addChildShape(localTransform,cylinderShape);
	btQuaternion orn(btVector3(0,1,0),SIMD_PI);
	localTransform.setRotation(orn);
	cylinderCompound->addChildShape(localTransform,cylinderShape);
	
	m_collisionShapes.push_back(cylinderCompound);


	m_dispatcher=0;

	/* FIXME: Register new collision algorithm */
	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btHfFluidRigidCollisionConfiguration();


	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	////////////////////////////
	///Register HfFluid versus rigidbody collision algorithm


	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;

	btDiscreteDynamicsWorld* world = new btHfFluidRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = world;


	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,-12,0));



	localCreateRigidBody(0.f,tr,m_collisionShapes[0]);


	//	clientResetScene();

	clientResetScene();
}






void	HfFluidDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;



	delete m_collisionConfiguration;


}

HfFluidDemo::HfFluidDemo() : m_drag(false)
{
	overrideGLShapeDrawer (new HfFluidDemo_GL_ShapeDrawer());
	setTexturing(true);
	setShadows(true);
}
