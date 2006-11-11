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

#include "GLDebugDrawer.h"

#include "BMF_Api.h"
#include <stdio.h> //printf debugging
#include <vector>

float deltaTime = 1.f/60.f;

#include "ConvexDecompositionDemo.h"
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"

const int maxNumObjects = 450;

int	shapeIndex[maxNumObjects];

btVector3	centroid;

#define CUBE_HALF_EXTENTS 4

btCollisionShape* shapePtr[maxNumObjects];



////////////////////////////////////

unsigned int tcount = 0;


GLDebugDrawer debugDrawer;

int main(int argc,char** argv)
{
	char* filename = "file.obj";


	ConvexDecompositionDemo* convexDecompDemo = new ConvexDecompositionDemo();

	convexDecompDemo->initPhysics(filename);



	convexDecompDemo->clientResetScene();

	convexDecompDemo->setCameraDistance(26.f);

	return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://www.continuousphysics.com/Bullet/phpBB2/",convexDecompDemo);
}

void ConvexDecompositionDemo::initPhysics(const char* filename)
{
	ConvexDecomposition::WavefrontObj wo;

	tcount = wo.loadObj(filename);

	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher();


	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);

	btOverlappingPairCache* broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax);
	//OverlappingPairCache* broadphase = new btSimpleBroadphase();

	btConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver);

	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,-4.5,0));

	localCreateRigidBody(0.f,startTransform,new btBoxShape(btVector3(30,2,30)));

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

				btVector3 localScaling(6.f,6.f,6.f);

				//export data to .obj
				printf("ConvexResult\n");
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
					std::vector<btVector3> vertices;
					if ( 1 )
					{
						const unsigned int *src = result.mHullIndices;
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
						const unsigned int *src = result.mHullIndices;
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
					float collisionMargin = 0.01f;

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
					
					//btCollisionShape* convexShape = new btConvexHullShape(&(vertices[0].getX()),vertices.size());
					btCollisionShape* convexShape = new btConvexTriangleMeshShape(trimesh);
#endif 

					convexShape->setMargin(0.01);
					

					btTransform trans;
					trans.setIdentity();
					trans.setOrigin(centroid);
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

		btVector3 localScaling(6.f,6.f,6.f);
		
		for (int i=0;i<wo.mTriCount;i++)
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

		btCollisionShape* convexShape = new btConvexTriangleMeshShape(trimesh);
		float mass = 1.f;
		
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(20,2,0));

		localCreateRigidBody(mass, startTransform,convexShape);

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

		printf("WavefrontObj num triangles read %i",tcount);
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


	m_dynamicsWorld->setDebugDrawer(&debugDrawer);

}

void ConvexDecompositionDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	float dt = m_clock.getTimeMicroseconds() * 0.000001f;
	m_clock.reset();

	m_dynamicsWorld->stepSimulation(dt);

	renderme();

	glFlush();
	glutSwapBuffers();

}



void ConvexDecompositionDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	m_dynamicsWorld->updateAabbs();

	renderme();


	glFlush();
	glutSwapBuffers();
}

