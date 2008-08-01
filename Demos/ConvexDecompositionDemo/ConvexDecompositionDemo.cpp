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

#include "cd_wavefront.h"
#include "ConvexBuilder.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btGeometryUtil.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"

//#define USE_PARALLEL_DISPATCHER 1
#ifdef USE_PARALLEL_DISPATCHER
#include "../../Extras/BulletMultiThreaded/SpuGatheringCollisionDispatcher.h"
#include "../../Extras/BulletMultiThreaded/Win32ThreadSupport.h"
#include "../../Extras/BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#endif//USE_PARALLEL_DISPATCHER





#include "BMF_Api.h"
#include <stdio.h> //printf debugging
#include <vector>


#include "ConvexDecompositionDemo.h"
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"


btVector3	centroid;
btVector3   convexDecompositionObjectOffset(10,0,0);

#define CUBE_HALF_EXTENTS 4


////////////////////////////////////

unsigned int tcount = 0;



void ConvexDecompositionDemo::initPhysics(const char* filename)
{
	setTexturing(true);
	setShadows(true);

	setCameraDistance(26.f);

	ConvexDecomposition::WavefrontObj wo;

	tcount = wo.loadObj(filename);

	if (!tcount)
	{
		//when running this app from visual studio, the default starting folder is different, so make a second attempt...
		tcount = wo.loadObj("../../file.obj");
	}


	m_collisionConfiguration = new btDefaultCollisionConfiguration();

#ifdef USE_PARALLEL_DISPATCHER
#ifdef USE_WIN32_THREADING

	int maxNumOutstandingTasks = 4;//number of maximum outstanding tasks
	Win32ThreadSupport* threadSupport = new Win32ThreadSupport(Win32ThreadSupport::Win32ThreadConstructionInfo(
								"collision",
								processCollisionTask,
								createCollisionLocalStoreMemory,
								maxNumOutstandingTasks));
#else
///todo other platform threading
///Playstation 3 SPU (SPURS)  version is available through PS3 Devnet
///Libspe2 SPU support will be available soon
///pthreads version
///you can hook it up to your custom task scheduler by deriving from btThreadSupportInterface
#endif

	m_dispatcher = new	SpuGatheringCollisionDispatcher(threadSupport,maxNumOutstandingTasks,m_collisionConfiguration);
#else
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
#endif//USE_PARALLEL_DISPATCHER



	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax);
	//m_broadphase = new btSimpleBroadphase();

	m_solver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

#ifdef USE_PARALLEL_DISPATCHER
	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
#endif //USE_PARALLEL_DISPATCHER

	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,-4.5,0));

	btCollisionShape* boxShape = new btBoxShape(btVector3(30,2,30));
	m_collisionShapes.push_back(boxShape);
	localCreateRigidBody(0.f,startTransform,boxShape);

	class MyConvexDecomposition : public ConvexDecomposition::ConvexDecompInterface
	{

		ConvexDecompositionDemo*	m_convexDemo;
		public:

		MyConvexDecomposition (FILE* outputFile,ConvexDecompositionDemo* demo)
			:m_convexDemo(demo),
				mBaseCount(0),
			mHullCount(0),
			mOutputFile(outputFile)

		{
		}
		
			virtual void ConvexDecompResult(ConvexDecomposition::ConvexResult &result)
			{

				btTriangleMesh* trimesh = new btTriangleMesh();
				m_convexDemo->m_trimeshes.push_back(trimesh);

				btVector3 localScaling(6.f,6.f,6.f);

				//export data to .obj
				printf("ConvexResult. ");
				if (mOutputFile)
				{
					fprintf(mOutputFile,"## Hull Piece %d with %d vertices and %d triangles.\r\n", mHullCount, result.mHullVcount, result.mHullTcount );

					fprintf(mOutputFile,"usemtl Material%i\r\n",mBaseCount);
					fprintf(mOutputFile,"o Object%i\r\n",mBaseCount);

					for (unsigned int i=0; i<result.mHullVcount; i++)
					{
						const float *p = &result.mHullVertices[i*3];
						fprintf(mOutputFile,"v %0.9f %0.9f %0.9f\r\n", p[0], p[1], p[2] );
					}

					//calc centroid, to shift vertices around center of mass
					centroid.setValue(0,0,0);
					btAlignedObjectArray<btVector3> vertices;
					if ( 1 )
					{
						//const unsigned int *src = result.mHullIndices;
						for (unsigned int i=0; i<result.mHullVcount; i++)
						{
							btVector3 vertex(result.mHullVertices[i*3],result.mHullVertices[i*3+1],result.mHullVertices[i*3+2]);
							vertex *= localScaling;
							centroid += vertex;
							
						}
					}

					centroid *= 1.f/(float(result.mHullVcount) );

					if ( 1 )
					{
						//const unsigned int *src = result.mHullIndices;
						for (unsigned int i=0; i<result.mHullVcount; i++)
						{
							btVector3 vertex(result.mHullVertices[i*3],result.mHullVertices[i*3+1],result.mHullVertices[i*3+2]);
							vertex *= localScaling;
							vertex -= centroid ;
							vertices.push_back(vertex);
						}
					}
					
			

					if ( 1 )
					{
						const unsigned int *src = result.mHullIndices;
						for (unsigned int i=0; i<result.mHullTcount; i++)
						{
							unsigned int index0 = *src++;
							unsigned int index1 = *src++;
							unsigned int index2 = *src++;


							btVector3 vertex0(result.mHullVertices[index0*3], result.mHullVertices[index0*3+1],result.mHullVertices[index0*3+2]);
							btVector3 vertex1(result.mHullVertices[index1*3], result.mHullVertices[index1*3+1],result.mHullVertices[index1*3+2]);
							btVector3 vertex2(result.mHullVertices[index2*3], result.mHullVertices[index2*3+1],result.mHullVertices[index2*3+2]);
							vertex0 *= localScaling;
							vertex1 *= localScaling;
							vertex2 *= localScaling;
							
							vertex0 -= centroid;
							vertex1 -= centroid;
							vertex2 -= centroid;


							trimesh->addTriangle(vertex0,vertex1,vertex2);

							index0+=mBaseCount;
							index1+=mBaseCount;
							index2+=mBaseCount;
							
							fprintf(mOutputFile,"f %d %d %d\r\n", index0+1, index1+1, index2+1 );
						}
					}

					float mass = 1.f;
					//float collisionMargin = 0.01f;

//this is a tools issue: due to collision margin, convex objects overlap, compensate for it here:
//#define SHRINK_OBJECT_INWARDS 1
#ifdef SHRINK_OBJECT_INWARDS

					
					std::vector<btVector3> planeEquations;
					btGeometryUtil::getPlaneEquationsFromVertices(vertices,planeEquations);

					std::vector<btVector3> shiftedPlaneEquations;
					for (int p=0;p<planeEquations.size();p++)
					{
						btVector3 plane = planeEquations[p];
						plane[3] += 5*collisionMargin;
						shiftedPlaneEquations.push_back(plane);
					}
					std::vector<btVector3> shiftedVertices;
					btGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations,shiftedVertices);

					
					btCollisionShape* convexShape = new btConvexHullShape(&(shiftedVertices[0].getX()),shiftedVertices.size());
					
#else //SHRINK_OBJECT_INWARDS
					
#ifdef USE_PARALLEL_DISPATCHER
					//SPU/multi threaded version only supports convex hull with contiguous vertices at the moment
					btCollisionShape* convexShape = new btConvexHullShape(&(vertices[0].getX()),vertices.size());
#else
					btCollisionShape* convexShape = new btConvexTriangleMeshShape(trimesh);
#endif //USE_PARALLEL_DISPATCHER

#endif 

					convexShape->setMargin(0.01);

					m_convexDemo->m_collisionShapes.push_back(convexShape);

					btTransform trans;
					trans.setIdentity();
					trans.setOrigin(centroid-convexDecompositionObjectOffset);
					
					//btRigidBody* body = m_convexDemo->localCreateRigidBody( mass, trans,convexShape);
					m_convexDemo->localCreateRigidBody( mass, trans,convexShape);
					mBaseCount+=result.mHullVcount; // advance the 'base index' counter.


				}
			}

			int   	mBaseCount;
  			int		mHullCount;
			FILE*	mOutputFile;

	};

	if (tcount)
	{
		btTriangleMesh* trimesh = new btTriangleMesh();
		m_trimeshes.push_back(trimesh);

		btVector3 localScaling(6.f,6.f,6.f);
		
		int i;
		for ( i=0;i<wo.mTriCount;i++)
		{
			int index0 = wo.mIndices[i*3];
			int index1 = wo.mIndices[i*3+1];
			int index2 = wo.mIndices[i*3+2];

			btVector3 vertex0(wo.mVertices[index0*3], wo.mVertices[index0*3+1],wo.mVertices[index0*3+2]);
			btVector3 vertex1(wo.mVertices[index1*3], wo.mVertices[index1*3+1],wo.mVertices[index1*3+2]);
			btVector3 vertex2(wo.mVertices[index2*3], wo.mVertices[index2*3+1],wo.mVertices[index2*3+2]);
			
			vertex0 *= localScaling;
			vertex1 *= localScaling;
			vertex2 *= localScaling;

			trimesh->addTriangle(vertex0,vertex1,vertex2);
		}

		
		btConvexShape* tmpConvexShape = new btConvexTriangleMeshShape(trimesh);
	
		printf("old numTriangles= %d\n",wo.mTriCount);
		printf("old numIndices = %d\n",wo.mTriCount*3);
		printf("old numVertices = %d\n",wo.mVertexCount);
		
		printf("reducing vertices by creating a convex hull\n");

		//create a hull approximation
		btShapeHull* hull = new btShapeHull(tmpConvexShape);
		btScalar margin = tmpConvexShape->getMargin();
		hull->buildHull(margin);
		tmpConvexShape->setUserPointer(hull);
		
		
		printf("new numTriangles = %d\n", hull->numTriangles ());
		printf("new numIndices = %d\n", hull->numIndices ());
		printf("new numVertices = %d\n", hull->numVertices ());
		
		btConvexHullShape* convexShape = new btConvexHullShape();
		for (i=0;i<hull->numVertices();i++)
		{
			convexShape->addPoint(btPoint3(hull->getVertexPointer()[i]));	
		}

		delete tmpConvexShape;
		delete hull;



		m_collisionShapes.push_back(convexShape);

		float mass = 1.f;
		
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(0,2,0));

		localCreateRigidBody(mass, startTransform,convexShape);
		
		bool useQuantization = true;
		btCollisionShape* concaveShape = new btBvhTriangleMeshShape(trimesh,useQuantization);
		startTransform.setOrigin(convexDecompositionObjectOffset);
		localCreateRigidBody(0.f,startTransform,concaveShape);

	}
			

	if (tcount)
	{

		char outputFileName[512];
  		strcpy(outputFileName,filename);
  		char *dot = strstr(outputFileName,".");
  		if ( dot ) 
			*dot = 0;
		strcat(outputFileName,"_convex.obj");
  		FILE* outputFile = fopen(outputFileName,"wb");
				
		unsigned int depth = 5;
		float cpercent     = 5;
		float ppercent     = 15;
		unsigned int maxv  = 16;
		float skinWidth    = 0.0;

		printf("WavefrontObj num triangles read %i\n",tcount);
		ConvexDecomposition::DecompDesc desc;
		desc.mVcount       =	wo.mVertexCount;
		desc.mVertices     = wo.mVertices;
		desc.mTcount       = wo.mTriCount;
		desc.mIndices      = (unsigned int *)wo.mIndices;
		desc.mDepth        = depth;
		desc.mCpercent     = cpercent;
		desc.mPpercent     = ppercent;
		desc.mMaxVertices  = maxv;
		desc.mSkinWidth    = skinWidth;

		MyConvexDecomposition	convexDecomposition(outputFile,this);
		desc.mCallback = &convexDecomposition;
		
		

		//convexDecomposition.performConvexDecomposition(desc);

		ConvexBuilder cb(desc.mCallback);
		cb.process(desc);
		
		if (outputFile)
			fclose(outputFile);


	}


	
}

void ConvexDecompositionDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	float dt = getDeltaTimeMicroseconds() * 0.000001f;

	m_dynamicsWorld->stepSimulation(dt);

	//optional but useful: debug drawing
	m_dynamicsWorld->debugDrawWorld();

	renderme();

	glFlush();
	glutSwapBuffers();

}



void ConvexDecompositionDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	renderme();


	glFlush();
	glutSwapBuffers();
}




void	ConvexDecompositionDemo::exitPhysics()
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
	for (i=0;i<m_collisionShapes.size();i++)
	{
		btCollisionShape* shape = m_collisionShapes[i];
		delete shape;
	}

	for (i=0;i<m_trimeshes.size();i++)
	{
		btTriangleMesh* mesh = m_trimeshes[i];
		delete mesh;
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




