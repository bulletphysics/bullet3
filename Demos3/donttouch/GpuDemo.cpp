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


#include "b3CpuDynamicsWorld.h"
#include "b3GpuDynamicsWorld.h"


#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y 10
#define START_POS_Z -3

#include "LinearMath/b3Vector3.h"

#include "GpuDemo.h"
//#include "GlutStuff.h"
///b3BulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
//#include "b3BulletDynamicsCommon.h"


#include "BulletCollision/CollisionShapes/b3TriangleMesh.h"
#include "BulletCollision/CollisionShapes/b3BvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/b3SphereShape.h"
#include "BulletCollision/CollisionShapes/b3ConvexHullShape.h"
#include "BulletCollision/CollisionShapes/b3BoxShape.h"
#include "BulletCollision/CollisionShapes/b3CompoundShape.h"
#include "BulletCollision/CollisionShapes/b3StaticPlaneShape.h"

#include "BulletDynamics/Dynamics/b3RigidBody.h"
#include "LinearMath/b3DefaultMotionState.h"
#include "LinearMath/b3Quickprof.h"


#include <stdio.h> //printf debugging


void GpuDemo::clientMoveAndDisplay()
{
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float dt = getDeltaTimeInSeconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		static bool once = true;
		if (once)
		{
			once=false;
			b3DefaultSerializer*	serializer = new b3DefaultSerializer();
			m_dynamicsWorld->serialize(serializer);
 
			FILE* file = fopen("testFile.bullet","wb");
			fwrite(serializer->getBufferPointer(),serializer->getCurrentBufferSize(),1, file);
			fclose(file);
		}

		m_dynamicsWorld->stepSimulation(dt);
		static int count=0;
		count++;
		if (count==25)
		{
			//b3ProfileManager::dumpAll();
		}
	}
		
//	renderme(); 


	//swapBuffers();

}






b3AlignedObjectArray<b3Vector3> vertices;

void	EmptyDemo::setupScene(const ConstructionInfo& ci)
{
	//empty test
}

void SpheresDemo::setupScene(const ConstructionInfo& ci)
{
	
	
	if (1)
	{
		b3SphereShape* sphere = new b3SphereShape(1);
			m_collisionShapes.push_back(sphere);

		/// Create Dynamic Objects
		b3Transform startTransform;
		startTransform.setIdentity();

	

		float start_x = START_POS_X - ci.gapX*ci.arraySizeX/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ci.gapZ*ci.arraySizeZ/2;

		for (int k=0;k<ci.arraySizeY;k++)
		{
			int sizeX = ci.arraySizeX;
			int startX = -sizeX/2;
			float gapX = ci.gapX;

			for (int i=0;i<sizeX;i++)
			{
				int sizeZ = ci.arraySizeZ;
				int startZ = -sizeX/2;
				float gapZ =ci.gapZ;
				for(int j = 0;j<sizeZ;j++)
				{
					//b3CollisionShape* shape = k==0? boxShape : colShape;

					b3CollisionShape* shape = sphere;

					
					b3Scalar	mass  = 1;
					if (!ci.m_useConcaveMesh && k==0)
						mass = k==0? 0.f : 1.f;

					//rigidbody is dynamic if and only if mass is non zero, otherwise static
					bool isDynamic = (mass != 0.f);

					b3Vector3 localInertia(0,0,0);
					if (isDynamic)
						shape->calculateLocalInertia(mass,localInertia);

					startTransform.setOrigin(SCALING*b3Vector3(
										b3Scalar(gapX*i + start_x),
										b3Scalar(ci.gapY*k + start_y),
										b3Scalar(gapZ*j + start_z)));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					b3DefaultMotionState* myMotionState = new b3DefaultMotionState(startTransform);
					b3RigidBody::b3RigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
					b3RigidBody* body = new b3RigidBody(rbInfo);
					

					m_dynamicsWorld->addRigidBody(body);
				}
			}
		}
	}

	{
		b3Vector3 planeNormal(0,1,0);
		b3Scalar planeConstant=0;

		b3CollisionShape* shape = new b3StaticPlaneShape(planeNormal,planeConstant);
		//b3BoxShape* plane = new b3BoxShape(b3Vector3(100,1,100));
		//plane->initializePolyhedralFeatures();
		//b3SphereShape* shape = new b3SphereShape(1000);

		b3Scalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		b3Vector3 localInertia(0,0,0);
		b3Transform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setRotation(b3Quaternion(b3Vector3(1,0,0),0.3));
		groundTransform.setOrigin(b3Vector3(0,0,0));

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		b3DefaultMotionState* myMotionState = new b3DefaultMotionState(groundTransform);
		b3RigidBody::b3RigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		b3RigidBody* body = new b3RigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
}



void	GpuCompoundDemo::setupScene(const ConstructionInfo& ci)
{
		b3CollisionShape* groundShape =0;
//	b3CollisionShape* groundShape = new b3StaticPlaneShape(b3Vector3(0,1,0),50);

	if (ci.m_useConcaveMesh)
	{
		b3TriangleMesh* meshInterface = new b3TriangleMesh();

		b3AlignedObjectArray<b3Vector3> concaveVertices;
		concaveVertices.push_back(b3Vector3(0,-20,0));
		concaveVertices.push_back(b3Vector3(80,10,80));
		concaveVertices.push_back(b3Vector3(80,10,-80));
		concaveVertices.push_back(b3Vector3(-80,10,-80));
		concaveVertices.push_back(b3Vector3(-80,10,80));

		meshInterface->addTriangle(concaveVertices[0],concaveVertices[1],concaveVertices[2],true);
		meshInterface->addTriangle(concaveVertices[0],concaveVertices[2],concaveVertices[3],true);
		meshInterface->addTriangle(concaveVertices[0],concaveVertices[3],concaveVertices[4],true);
		meshInterface->addTriangle(concaveVertices[0],concaveVertices[4],concaveVertices[1],true);

#if 0
		groundShape = new b3BvhTriangleMeshShape(meshInterface,true);//b3StaticPlaneShape(b3Vector3(0,1,0),50);
#else
		b3BoxShape* shape =new b3BoxShape(b3Vector3(b3Scalar(250.),b3Scalar(10.),b3Scalar(250.)));
		shape->initializePolyhedralFeatures();
		groundShape = shape;
#endif

	} else
	{
		groundShape  = new b3BoxShape(b3Vector3(b3Scalar(250.),b3Scalar(50.),b3Scalar(250.)));
	}
	
	m_collisionShapes.push_back(groundShape);

	b3Transform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(b3Vector3(0,0,0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	if (ci.m_useConcaveMesh)
	{
		b3Scalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		b3Vector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		b3DefaultMotionState* myMotionState = new b3DefaultMotionState(groundTransform);
		b3RigidBody::b3RigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		b3RigidBody* body = new b3RigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		//vertices.push_back(b3Vector3(0,1,0));
		vertices.push_back(b3Vector3(1,1,1));
		vertices.push_back(b3Vector3(1,1,-1));
		vertices.push_back(b3Vector3(-1,1,-1));
		vertices.push_back(b3Vector3(-1,1,1));
		vertices.push_back(b3Vector3(1,-1,1));
		vertices.push_back(b3Vector3(1,-1,-1));
		vertices.push_back(b3Vector3(-1,-1,-1));
		vertices.push_back(b3Vector3(-1,-1,1));
		
#if 0
		b3PolyhedralConvexShape* colShape = new b3ConvexHullShape(&vertices[0].getX(),vertices.size());
		colShape->initializePolyhedralFeatures();
#else
		b3CompoundShape* compoundShape = 0;
		{
			b3PolyhedralConvexShape* colShape = new b3ConvexHullShape(&vertices[0].getX(),vertices.size());
			colShape->initializePolyhedralFeatures();
			compoundShape = new b3CompoundShape();
			b3Transform tr;
			tr.setIdentity();
			tr.setOrigin(b3Vector3(0,-1,0));
			compoundShape->addChildShape(tr,colShape);
			tr.setOrigin(b3Vector3(0,0,2));
			compoundShape->addChildShape(tr,colShape);
			tr.setOrigin(b3Vector3(2,0,0));
			compoundShape->addChildShape(tr,colShape);
		}
		b3CollisionShape* colShape = compoundShape;
#endif



		b3PolyhedralConvexShape* boxShape = new b3BoxShape(b3Vector3(SCALING*1,SCALING*1,SCALING*1));
		boxShape->initializePolyhedralFeatures();
		



		//b3CollisionShape* colShape = new b3SphereShape(b3Scalar(1.));
		m_collisionShapes.push_back(colShape);
		m_collisionShapes.push_back(boxShape);

		/// Create Dynamic Objects
		b3Transform startTransform;
		startTransform.setIdentity();

	

		float start_x = START_POS_X - ci.arraySizeX/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ci.arraySizeZ/2;

		for (int k=0;k<ci.arraySizeY;k++)
		{
			int sizeX = ci.arraySizeX;
			if (!ci.m_useConcaveMesh && k==0)
				sizeX = 50;

			int startX = !ci.m_useConcaveMesh&&k==0? -20 : 0;
			float gapX = !ci.m_useConcaveMesh&&k==0? 3.05 : ci.gapX;
			for (int i=0;i<sizeX;i++)
			{
				int sizeZ = !ci.m_useConcaveMesh&&k==0? 50 : ci.arraySizeZ;
				int startZ = (!ci.m_useConcaveMesh)&&k==0? -20 : 0;
				float gapZ = !ci.m_useConcaveMesh&&k==0? 3.05 : ci.gapZ;
				for(int j = 0;j<sizeZ;j++)
				{
					//b3CollisionShape* shape = k==0? boxShape : colShape;

					b3CollisionShape* shape = colShape;

					
					b3Scalar	mass  = 1;
					if (!ci.m_useConcaveMesh && k==0)
						mass = k==0? 0.f : 1.f;

					//rigidbody is dynamic if and only if mass is non zero, otherwise static
					bool isDynamic = (mass != 0.f);

					b3Vector3 localInertia(0,0,0);
					if (isDynamic)
						shape->calculateLocalInertia(mass,localInertia);

					startTransform.setOrigin(SCALING*b3Vector3(
										b3Scalar(startX+gapX*i + start_x),
										b3Scalar(20+ci.gapY*k + start_y),
										b3Scalar(startZ+gapZ*j + start_z)));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					b3DefaultMotionState* myMotionState = new b3DefaultMotionState(startTransform);
					b3RigidBody::b3RigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
					b3RigidBody* body = new b3RigidBody(rbInfo);
					

					m_dynamicsWorld->addRigidBody(body);
				}
			}
		}
	}
}



void	GpuBoxDemo::setupScene(const ConstructionInfo& ci)
{
		b3CollisionShape* groundShape =0;
//	b3CollisionShape* groundShape = new b3StaticPlaneShape(b3Vector3(0,1,0),50);

	if (ci.m_useConcaveMesh)
	{
		b3TriangleMesh* meshInterface = new b3TriangleMesh();

		b3AlignedObjectArray<b3Vector3> concaveVertices;
		concaveVertices.push_back(b3Vector3(0,-20,0));
		concaveVertices.push_back(b3Vector3(80,10,80));
		concaveVertices.push_back(b3Vector3(80,10,-80));
		concaveVertices.push_back(b3Vector3(-80,10,-80));
		concaveVertices.push_back(b3Vector3(-80,10,80));

		meshInterface->addTriangle(concaveVertices[0],concaveVertices[1],concaveVertices[2],true);
		meshInterface->addTriangle(concaveVertices[0],concaveVertices[2],concaveVertices[3],true);
		meshInterface->addTriangle(concaveVertices[0],concaveVertices[3],concaveVertices[4],true);
		meshInterface->addTriangle(concaveVertices[0],concaveVertices[4],concaveVertices[1],true);

#if 0
		groundShape = new b3BvhTriangleMeshShape(meshInterface,true);//b3StaticPlaneShape(b3Vector3(0,1,0),50);
#else
		b3BoxShape* shape =new b3BoxShape(b3Vector3(b3Scalar(250.),b3Scalar(10.),b3Scalar(250.)));
		shape->initializePolyhedralFeatures();
		groundShape = shape;
#endif

	} else
	{
		groundShape  = new b3BoxShape(b3Vector3(b3Scalar(250.),b3Scalar(50.),b3Scalar(250.)));
	}
	
	m_collisionShapes.push_back(groundShape);

	b3Transform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(b3Vector3(0,0,0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	if (ci.m_useConcaveMesh)
	{
		b3Scalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		b3Vector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		b3DefaultMotionState* myMotionState = new b3DefaultMotionState(groundTransform);
		b3RigidBody::b3RigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		b3RigidBody* body = new b3RigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		//vertices.push_back(b3Vector3(0,1,0));
		vertices.push_back(b3Vector3(1,1,1));
		vertices.push_back(b3Vector3(1,1,-1));
		vertices.push_back(b3Vector3(-1,1,-1));
		vertices.push_back(b3Vector3(-1,1,1));
		vertices.push_back(b3Vector3(1,-1,1));
		vertices.push_back(b3Vector3(1,-1,-1));
		vertices.push_back(b3Vector3(-1,-1,-1));
		vertices.push_back(b3Vector3(-1,-1,1));
		
#if 1
		b3PolyhedralConvexShape* colShape = new b3ConvexHullShape(&vertices[0].getX(),vertices.size());
		colShape->initializePolyhedralFeatures();
#else
		b3CompoundShape* compoundShape = 0;
		{
			b3PolyhedralConvexShape* colShape = new b3ConvexHullShape(&vertices[0].getX(),vertices.size());
			colShape->initializePolyhedralFeatures();
			compoundShape = new b3CompoundShape();
			b3Transform tr;
			tr.setIdentity();
			tr.setOrigin(b3Vector3(0,-1,0));
			compoundShape->addChildShape(tr,colShape);
			tr.setOrigin(b3Vector3(0,0,2));
			compoundShape->addChildShape(tr,colShape);
			tr.setOrigin(b3Vector3(2,0,0));
			compoundShape->addChildShape(tr,colShape);
		}
		b3CollisionShape* colShape = compoundShape;
#endif



		b3PolyhedralConvexShape* boxShape = new b3BoxShape(b3Vector3(SCALING*1,SCALING*1,SCALING*1));
		boxShape->initializePolyhedralFeatures();
		



		//b3CollisionShape* colShape = new b3SphereShape(b3Scalar(1.));
		m_collisionShapes.push_back(colShape);
		m_collisionShapes.push_back(boxShape);

		/// Create Dynamic Objects
		b3Transform startTransform;
		startTransform.setIdentity();

	

		float start_x = START_POS_X - ci.arraySizeX/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ci.arraySizeZ/2;

		for (int k=0;k<ci.arraySizeY;k++)
		{
			int sizeX = ci.arraySizeX;
			if (!ci.m_useConcaveMesh && k==0)
				sizeX = 50;

			int startX = !ci.m_useConcaveMesh&&k==0? -20 : 0;
			float gapX = !ci.m_useConcaveMesh&&k==0? 3.05 : ci.gapX;
			for (int i=0;i<sizeX;i++)
			{
				int sizeZ = !ci.m_useConcaveMesh&&k==0? 50 : ci.arraySizeZ;
				int startZ = (!ci.m_useConcaveMesh)&&k==0? -20 : 0;
				float gapZ = !ci.m_useConcaveMesh&&k==0? 3.05 : ci.gapZ;
				for(int j = 0;j<sizeZ;j++)
				{
					//b3CollisionShape* shape = k==0? boxShape : colShape;

					b3CollisionShape* shape = colShape;

					
					b3Scalar	mass  = 1;
					if (!ci.m_useConcaveMesh && k==0)
						mass = k==0? 0.f : 1.f;

					//rigidbody is dynamic if and only if mass is non zero, otherwise static
					bool isDynamic = (mass != 0.f);

					b3Vector3 localInertia(0,0,0);
					if (isDynamic)
						shape->calculateLocalInertia(mass,localInertia);

					startTransform.setOrigin(SCALING*b3Vector3(
										b3Scalar(startX+gapX*i + start_x),
										b3Scalar(ci.gapY*(k+0.5) + start_y),
										b3Scalar(startZ+gapZ*j + start_z)));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					b3DefaultMotionState* myMotionState = new b3DefaultMotionState(startTransform);
					b3RigidBody::b3RigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
					b3RigidBody* body = new b3RigidBody(rbInfo);
					

					m_dynamicsWorld->addRigidBody(body);
				}
			}
		}
	}
}

void	GpuDemo::initPhysics(const ConstructionInfo& ci)
{

//	setTexturing(true);
	//setShadows(false);

//	setCameraDistance(b3Scalar(SCALING*250.));

	///collision configuration contains default setup for memory, collision setup
	if (ci.useOpenCL)
	{
		m_dynamicsWorld = new b3GpuDynamicsWorld(ci.preferredOpenCLPlatformIndex,ci.preferredOpenCLDeviceIndex);
	} else
	{
		m_dynamicsWorld = new b3CpuDynamicsWorld();
	}

	
	m_dynamicsWorld->setGravity(b3Vector3(0,-10,0));

	///create a few basic rigid bodies

	setupScene(ci);


}

/*void	GpuDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}
	*/


void	GpuDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	if (m_dynamicsWorld)
	{
		for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
		{
			b3CollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
			b3RigidBody* body = b3RigidBody::upcast(obj);
			if (body && body->getMotionState())
			{
				delete body->getMotionState();
			}
			m_dynamicsWorld->removeCollisionObject( obj );
			delete obj;
		}
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		b3CollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	m_dynamicsWorld=0;
	

	
}



