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

#include "BasicDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "CustomConvexShape.h"
#include "CustomConvexPairCollision.h"
#include "CustomCollisionDispatcher.h"

#include "ConvexHeightFieldShape.h"
#include "GLDebugDrawer.h"
static GLDebugDrawer sDebugDraw;

#include <stdio.h> //printf debugging

#ifdef CL_PLATFORM_AMD
#include "../../opencl/basic_initialize/btOpenCLUtils.h"

cl_context			g_cxMainContext=0;
cl_command_queue	g_cqCommandQue=0;
cl_device_id		g_clDevice=0;
#endif

///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 6
#define ARRAY_SIZE_Y 6
#define ARRAY_SIZE_Z 4

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X 0
#define START_POS_Y -0.8
#define START_POS_Z 0

#define BoxVtxCount 8

static float BoxVtx[] = {
-0.5,-0.5,-0.5,
-0.5,-0.5,0.5,
-0.5,0.5,-0.5,
-0.5,0.5,0.5,
0.5,-0.5,-0.5,
0.5,-0.5,0.5,
0.5,0.5,-0.5,
0.5,0.5,0.5,
};

static float BoxVtx2[] = {
-20.3,-10.3,-20.3,
-20.3,-10.3,20.3,
-20.3,10.3,-20.3,
-20.3,10.3,20.3,
20.3,-10.3,-20.3,
20.3,-10.3,20.3,
20.3,10.3,-20.3,
20.3,10.3,20.3,
};


#define BarrelVtxCount2 57

static float BarrelVtx2[] = {
0.0f,-0.5f,0.0f,				0.0f,-1.0f,0.0f,
0.282362f,-0.5f,-0.205148f,     0.0f,-1.0f,0.0f,
0.349018f,-0.5f,0.0f,           0.0f,-1.0f,0.0f,
0.107853f,-0.5f,-0.331936f,     0.0f,-1.0f,0.0f,
-0.107853f,-0.5f,-0.331936f,    0.0f,-1.0f,0.0f,
0.107853f,-0.5f,-0.331936f,     0.0f,-1.0f,0.0f,
-0.282362f,-0.5f,-0.205148f,    0.0f,-1.0f,0.0f,
-0.349018f,-0.5f,0.0f,          0.0f,-1.0f,0.0f,
-0.282362f,-0.5f,0.205148f,     0.0f,-1.0f,0.0f,
-0.107853f,-0.5f,0.331936f,     0.0f,-1.0f,0.0f,
0.107853f,-0.5f,0.331936f,      0.0f,-1.0f,0.0f,
0.282362f,-0.5f,0.205148f,      0.0f,-1.0f,0.0f,
0.0f,0.5f,0.0f,                 0.0f,1.0f,0.0f,
0.349018f,0.5f,0.0f,            0.0f,1.0f,0.0f,
0.282362f,0.5f,-0.205148f,      0.0f,1.0f,0.0f,
0.107853f,0.5f,-0.331936f,      0.0f,1.0f,0.0f,
0.107853f,0.5f,-0.331936f,      0.0f,1.0f,0.0f,
-0.107853f,0.5f,-0.331936f,     0.0f,1.0f,0.0f,
-0.282362f,0.5f,-0.205148f,     0.0f,1.0f,0.0f,
-0.349018f,0.5f,0.0f,           0.0f,1.0f,0.0f,
-0.282362f,0.5f,0.205148f,      0.0f,1.0f,0.0f,
-0.107853f,0.5f,0.331936f,      0.0f,1.0f,0.0f,
0.107853f,0.5f,0.331936f,       0.0f,1.0f,0.0f,
0.282362f,0.5f,0.205148f,       0.0f,1.0f,0.0f,
0.349018f,-0.5f,0.0f,           0.957307f,-0.289072f,0.0f,
0.404509f,0.0f,-0.293893f,      0.809017f,0.0f,-0.587785f,
0.5f,0.0f,0.0f,                 1.0f,0.0f,0.0f,
0.282362f,-0.5f,-0.205148f,     0.774478f,-0.289072f,-0.562691f,
0.154508f,0.0f,-0.475528f,      0.309017f,0.0f,-0.951057f,
0.107853f,-0.5f,-0.331936f,     0.295824f,-0.289072f,-0.910453f,
0.107853f,-0.5f,-0.331936f,     0.295824f,-0.289072f,-0.910453f,
-0.154509f,0.0f,-0.475528f,     -0.309017f,0.0f,-0.951057f,
0.154508f,0.0f,-0.475528f,      0.309017f,0.0f,-0.951057f,
-0.107853f,-0.5f,-0.331936f,    -0.295824f,-0.289072f,-0.910453f,
-0.404509f,0.0f,-0.293893f,     -0.809017f,0.0f,-0.587785f,
-0.282362f,-0.5f,-0.205148f,    -0.774478f,-0.289072f,-0.562691f,
-0.5f,0.0f,0.0f,                -1.0f,0.0f,0.0f,
-0.349018f,-0.5f,0.0f,          -0.957307f,-0.289072f,0.0f,
-0.404508f,0.0f,0.293893f,      -0.809017f,0.0f,0.587785f,
-0.282362f,-0.5f,0.205148f,     -0.774478f,-0.289072f,0.562691f,
-0.154509f,0.0f,0.475528f,      -0.309017f,0.0f,0.951056f,
-0.107853f,-0.5f,0.331936f,     -0.295824f,-0.289072f,0.910453f,
0.154509f,0.0f,0.475528f,       0.309017f,0.0f,0.951056f,
0.107853f,-0.5f,0.331936f,      0.295824f,-0.289072f,0.910453f,
0.404509f,0.0f,0.293892f,       0.809017f,0.0f,0.587785f,
0.282362f,-0.5f,0.205148f,      0.774478f,-0.289072f,0.562691f,
0.282362f,0.5f,-0.205148f,      0.774478f,0.289072f,-0.562691f,
0.349018f,0.5f,0.0f,            0.957307f,0.289072f,0.0f,
0.107853f,0.5f,-0.331936f,      0.295824f,0.289072f,-0.910453f,
-0.107853f,0.5f,-0.331936f,     -0.295824f,0.289072f,-0.910453f,
0.107853f,0.5f,-0.331936f,      0.295824f,0.289072f,-0.910453f,
-0.282362f,0.5f,-0.205148f,     -0.774478f,0.289072f,-0.562691f,
-0.349018f,0.5f,0.0f,           -0.957307f,0.289072f,0.0f,
-0.282362f,0.5f,0.205148f,      -0.774478f,0.289072f,0.562691f,
-0.107853f,0.5f,0.331936f,      -0.295824f,0.289072f,0.910453f,
0.107853f,0.5f,0.331936f,       0.295824f,0.289072f,0.910453f,
0.282362f,0.5f,0.205148f,       0.774478f,0.289072f,0.562691f,
};


static int BarrelIdx[] = {
0,1,2,
0,3,1,
0,4,5,
0,6,4,
0,7,6,
0,8,7,
0,9,8,
0,10,9,
0,11,10,
0,2,11,
12,13,14,
12,14,15,
12,16,17,
12,17,18,
12,18,19,
12,19,20,
12,20,21,
12,21,22,
12,22,23,
12,23,13,
24,25,26,
24,27,25,
27,28,25,
27,29,28,
30,31,32,
30,33,31,
33,34,31,
33,35,34,
35,36,34,
35,37,36,
37,38,36,
37,39,38,
39,40,38,
39,41,40,
41,42,40,
41,43,42,
43,44,42,
43,45,44,
45,26,44,
45,24,26,
26,46,47,
26,25,46,
25,48,46,
25,28,48,
32,49,50,
32,31,49,
31,51,49,
31,34,51,
34,52,51,
34,36,52,
36,53,52,
36,38,53,
38,54,53,
38,40,54,
40,55,54,
40,42,55,
42,56,55,
42,44,56,
44,47,56,
44,26,47,
};


__inline void glVertexFloat4( const float4& v )
{
	glVertex3f( v.x, v.y, v.z );
}

__inline void drawPointListTransformed(const float4* vtx,  int nVtx, const float4& translation, const Quaternion& quat)
{
	glPushMatrix();

	Matrix3x3 rotMat = mtTranspose( qtGetRotationMatrix( quat ) );
	float transformMat[16] =
	{
		rotMat.m_row[0].x, rotMat.m_row[0].y, rotMat.m_row[0].z, 0,
		rotMat.m_row[1].x, rotMat.m_row[1].y, rotMat.m_row[1].z, 0,
		rotMat.m_row[2].x, rotMat.m_row[2].y, rotMat.m_row[2].z, 0,
		translation.x, translation.y, translation.z,1
	};

	glMultMatrixf( transformMat );

	float4 c = make_float4(1,1,0,0);

	glPointSize(3.f);
	glBegin(GL_POINTS);
	for(int i=0; i<nVtx; i++)
	{
		glColor4f(c.x, c.y, c.z, 1);
		glVertexFloat4( vtx[i] );
	}
	glEnd();

	glPopMatrix();
}
void displaySamples(const float4* vertices, int numVertices, const float4& translation, const Quaternion& quaternion) 
{
	drawPointListTransformed( vertices,numVertices, translation, quaternion );
}



void BasicDemo::renderSurfacePoints()
{
	if (m_dynamicsWorld->getDebugDrawer()->getDebugMode()& btIDebugDraw::DBG_DrawContactPoints)
	for (int i=0;i<m_dynamicsWorld->getCollisionObjectArray().size();i++)
	{
		btCollisionObject* ob = m_dynamicsWorld->getCollisionObjectArray()[i];
		if (ob->getCollisionShape()->getShapeType() == CUSTOM_POLYHEDRAL_SHAPE_TYPE)
		{
			CustomConvexShape* customConvex = (CustomConvexShape*)ob->getCollisionShape();
			ConvexHeightField* cvxShape= customConvex->m_ConvexHeightField;
			if (!cvxShape)
			{
				printf("aargh\n");
			}

				float4 bodyApos;
			Quaternion bodyAquat;

	
	const btVector3& pA = ob->getWorldTransform().getOrigin();
	btQuaternion qA = ob->getWorldTransform().getRotation();
	
	bodyApos.x = pA.getX();
	bodyApos.y = pA.getY();
	bodyApos.z = pA.getZ();
	bodyApos.w = 0.f;
	bodyAquat.x = qA.getX();
	bodyAquat.y = qA.getY();
	bodyAquat.z = qA.getZ();
	bodyAquat.w = qA.getW();


	displaySamples(cvxShape->getSamplePoints(),cvxShape->getNumSamplePoints(),bodyApos,bodyAquat);

		}

	}
}
void BasicDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
		
	renderme(); 

	renderSurfacePoints();


	glFlush();

	swapBuffers();

}



void BasicDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	renderSurfacePoints();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}





void	BasicDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	m_acceleratedRigidBodies = 0;

	setCameraDistance(btScalar(SCALING*20.));

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	
#ifdef CL_PLATFORM_AMD
	m_dispatcher = new	CustomCollisionDispatcher(m_collisionConfiguration,	g_cxMainContext,g_clDevice,g_cqCommandQue);
#else
	m_dispatcher = new	CustomCollisionDispatcher(m_collisionConfiguration);
#endif

	m_dispatcher->registerCollisionCreateFunc(CUSTOM_POLYHEDRAL_SHAPE_TYPE,CUSTOM_POLYHEDRAL_SHAPE_TYPE,new CustomConvexConvexPairCollision::CreateFunc(m_collisionConfiguration->getSimplexSolver(), m_collisionConfiguration->getPenetrationDepthSolver()));

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	m_dynamicsWorld->setDebugDrawer(&sDebugDraw);

	///create a few basic rigid bodies
	//btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
#if 1
	CustomConvexShape* groundShape = new CustomConvexShape(BoxVtx2,BoxVtxCount,3*sizeof(float));
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),0);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-11,0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
#endif


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		//btCollisionShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
#define USE_CUSTOM_HEIGHTFIELD_SHAPE 
#ifdef USE_CUSTOM_HEIGHTFIELD_SHAPE
	CustomConvexShape* colShape = new CustomConvexShape(BarrelVtx2,BarrelVtxCount2,6*sizeof(float));

	//CustomConvexShape* colShape = new CustomConvexShape(BoxVtx,BoxVtxCount,3*sizeof(float));
#else
	btConvexHullShape* colShape = new btConvexHullShape(BarrelVtx2,BarrelVtxCount2,6*sizeof(float));
		colShape->setLocalScaling(btVector3(0.9,0.9,0.9));

#endif //USE_CUSTOM_HEIGHTFIELD_SHAPE
	btScalar scale = 0.5f;
	
	//btScalar scale = 1.f;

		//next line is already called inside the CustomConvexShape constructor
		//colShape->initializePolyhedralFeatures();

		m_collisionShapes.push_back(colShape);

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
			for(int j = 0;j<ARRAY_SIZE_Z;j++)	
			{
				for (int i=0;i<ARRAY_SIZE_X;i++)
				{
					
					{
					//	if ((k>0) && ((j<2) || (j>(ARRAY_SIZE_Z-3))))
					//		continue;
					//	if ((k>0) && ((i<2) || (i>(ARRAY_SIZE_X-3))))
					//		continue;

					startTransform.setOrigin(SCALING*btVector3(
										btScalar(scale*2.0*i + start_x),
										btScalar(scale*1+scale*2.0*k + start_y),
										btScalar(scale*2.0*j + start_z)));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody* body=0;

					if (0)//k==0)
					{
						btVector3 zeroInertia(0,0,0);
						btRigidBody::btRigidBodyConstructionInfo rbInfo(0.f,myMotionState,colShape,zeroInertia);
						body = new btRigidBody(rbInfo);
					} else
					{
						btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
						body = new btRigidBody(rbInfo);
					}

					//m_acceleratedRigidBodies is used as a mapping to the accelerated rigid body index
					body->setCompanionId(m_acceleratedRigidBodies++);
					m_dynamicsWorld->addRigidBody(body);
						
					}
				}
			}
		}
	}


}
void	BasicDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}
	

void	BasicDemo::exitPhysics()
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
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}




