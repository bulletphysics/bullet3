/*
Copyright (c) 2013 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans

#include "ConstraintsDemo.h"
#include "OpenGLWindow/ShapeData.h"

#include "OpenGLWindow/GLInstancingRenderer.h"
#include "Bullet3Common/b3Quaternion.h"
#include "OpenGLWindow/b3gWindowInterface.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "../GpuDemoInternalData.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "OpenGLWindow/OpenGLInclude.h"
#include "OpenGLWindow/GLInstanceRendererInternalData.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Config.h"
#include "../rigidbody/GpuRigidBodyDemoInternalData.h"
#include "../gwenUserInterface.h"
#include "Bullet3Dynamics/ConstraintSolver/b3Point2PointConstraint.h"
#include "Bullet3Dynamics/ConstraintSolver/b3Generic6DofConstraint.h"
#include "Bullet3Dynamics/ConstraintSolver/b3FixedConstraint.h"

#include "OpenGLWindow/GLPrimitiveRenderer.h"
#include "Bullet3OpenCL/Raycast/b3GpuRaycast.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3ConvexUtility.h"

#include "OpenGLWindow/GLRenderToTexture.h"

void GpuConstraintsDemo::setupScene(const ConstructionInfo& ci)
{
	m_primRenderer = ci.m_primRenderer;

	m_raycaster = new b3GpuRaycast(m_clData->m_clContext,m_clData->m_clDevice,m_clData->m_clQueue);

	int index=0;
	createStaticEnvironment(ci);

	index+=createDynamicsObjects(ci);

	m_data->m_rigidBodyPipeline->writeAllInstancesToGpu();
//	m_data->m_rigidBodyPipeline->setGravity(b3Vector3(4,-10,0));
	float camPos[4]={ci.arraySizeX,0.5*ci.arraySizeY*ci.gapY,ci.arraySizeZ,0};
	//float camPos[4]={1,12.5,1.5,0};
	
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(180);
	
	m_instancingRenderer->setCameraPitch(200);//90);
	m_instancingRenderer->updateCamera();

	char msg[1024];
	int numInstances = index;
	sprintf(msg,"Num objects = %d",numInstances);
	if (ci.m_gui)
		ci.m_gui->setStatusBarMessage(msg,true);
}

void	GpuConstraintsDemo::destroyScene()
{
	delete m_raycaster;
	m_raycaster = 0;
}

int	GpuConstraintsDemo::createDynamicsObjects(const ConstructionInfo& ci)
{
/*	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(barrel_vertices)/strideInBytes;
	int numIndices = sizeof(barrel_indices)/sizeof(int);
	return createDynamicsObjects2(ci,barrel_vertices,numVertices,barrel_indices,numIndices);
	*/

	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	return createDynamicsObjects2(ci,cube_vertices,numVertices,cube_indices,numIndices);
	
}



int	GpuConstraintsDemo::createDynamicsObjects2(const ConstructionInfo& ci, const float* vertices, int numVertices, const int* indices, int numIndices)
{
	int strideInBytes = 9*sizeof(float);

	int shapeId = ci.m_instancingRenderer->registerShape(&vertices[0],numVertices,indices,numIndices);
	int group=1;
	int mask=1;
	int index=0;





	{
		b3Vector4 colors[4] =
		{
			b3MakeVector4(1,0,0,1),
			b3MakeVector4(0,1,0,1),
			b3MakeVector4(0,1,1,1),
			b3MakeVector4(1,1,0,1),
		};

		int curColor = 0;
		float scaling[4] = {1,1,1,1};
		int insta = 0;

		b3ConvexUtility* utilPtr = new b3ConvexUtility();

		{
			b3AlignedObjectArray<b3Vector3> verts;

			unsigned char* vts = (unsigned char*) vertices;
			for (int i=0;i<numVertices;i++)
			{
				float* vertex = (float*) &vts[i*strideInBytes];
				verts.push_back(b3MakeVector3(vertex[0]*scaling[0],vertex[1]*scaling[1],vertex[2]*scaling[2]));
			}

			bool merge = true;
			if (numVertices)
			{
				utilPtr->initializePolyhedralFeatures(&verts[0],verts.size(),merge);
			}
		}

		int colIndex=-1;
		if (ci.m_useInstancedCollisionShapes)
			colIndex = m_data->m_np->registerConvexHullShape(utilPtr);

		int constraintType=0;
		for (int i=0;i<ci.arraySizeZ;i++)
		{
			constraintType=(constraintType+1)&0x01;

			for (int k=0;k<ci.arraySizeX;k++)
			{

				int prevBody = -1;

			//printf("%d of %d\n", i, ci.arraySizeX);
				for (int j=0;j<ci.arraySizeY;j++)
				{

	
					//int colIndex = m_data->m_np->registerConvexHullShape(&vertices[0],strideInBytes,numVertices, scaling);
					if (!ci.m_useInstancedCollisionShapes)
						colIndex = m_data->m_np->registerConvexHullShape(utilPtr);

					float mass = 1.f;
					if (j==0 || j==ci.arraySizeY-1)
					{
						mass=0.f;
					}
					//b3Vector3 position((j&1)+i*2.2,1+j*2.,(j&1)+k*2.2);
					//b3Vector3 position((-ci.arraySizeX/2*ci.gapX)+i*ci.gapX,1+j*2.,(-ci.arraySizeZ/2*ci.gapZ)+k*ci.gapZ);
					b3Vector3 position=b3MakeVector3(-ci.arraySizeX/2*2+1+j*2.,
								10+i*ci.gapY,
								(-ci.arraySizeZ/2*ci.gapZ)+k*ci.gapZ);
					
					b3Quaternion orn(0,0,0,1);

					b3Vector4 color = colors[curColor];
					curColor++;
					curColor&=3;
					b3Vector4 scaling=b3MakeVector4(1,1,1,1);
					int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
					int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass,position,orn,colIndex,index,false);

					bool useGpu = false;
					b3TypedConstraint* c = 0;

					if (prevBody>=0)
					{
						switch (constraintType)
						{
						case 0:
							{
								///enable next line to force CPU constraint solving
								//c = new b3Point2PointConstraint(pid,prevBody,b3Vector3(-1.1,0,0),b3Vector3(1.1,0,0));
								float breakingThreshold=44;
//								c->setBreakingImpulseThreshold(breakingThreshold);
								b3Vector3 pivotInA=b3MakeVector3(-1.1,0,0);
								b3Vector3 pivotInB=b3MakeVector3(1.1,0,0);
								int cid = m_data->m_rigidBodyPipeline->createPoint2PointConstraint(pid,prevBody,pivotInA,pivotInB,breakingThreshold);
								break;
							}
						case 1:
							{
							
								b3Vector3 pivotInA=b3MakeVector3(-1.05,0,0);
								b3Vector3 pivotInB=b3MakeVector3(1.05,0,0);

							b3Transform frameInA,frameInB;
							frameInA.setIdentity();
							frameInB.setIdentity();
							frameInA.setOrigin(pivotInA);
							frameInB.setOrigin(pivotInB);
							b3Quaternion relTargetAB = frameInA.getRotation()*frameInB.getRotation().inverse();
							
							//c = new b3FixedConstraint(pid,prevBody,frameInA,frameInB);
							float breakingThreshold = 45;//37.f;
							//c->setBreakingImpulseThreshold(37.1);
							int cid = m_data->m_rigidBodyPipeline->createFixedConstraint(pid,prevBody,pivotInA,pivotInB,relTargetAB,breakingThreshold);
							
							

							break;
							}
						case 2:
							{
								/*
							b3Transform frameInA,frameInB;
							frameInA.setIdentity();
							frameInB.setIdentity();
							frameInA.setOrigin(b3Vector3(0,-1.1,0));
							frameInB.setOrigin(b3Vector3(0,1.1,0));

							b3Generic6DofConstraint* dof6 = new b3Generic6DofConstraint(pid,prevBody,frameInA,frameInB,false,m_data->m_np->getBodiesCpu());
							for (int i=0;i<6;i++)
								dof6->setLimit(i,0,0);
							c=dof6;
							*/
							break;
							}
						default:
							{

								b3Assert(0);
							}
						};
						if (c)
						{
							m_data->m_rigidBodyPipeline->addConstraint(c);
						}
						

					}


					prevBody = pid;

					index++;
				}
			}
		}
		delete utilPtr;
	}
	return index;
}


void GpuConstraintsDemo::createStaticEnvironment(const ConstructionInfo& ci)
{
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int group=1;
	int mask=1;
	int index=0;


	{
		b3Vector4 scaling=b3MakeVector4(400,400,400,1);
		int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
		b3Vector3 position=b3MakeVector3(0,-405,0);
		b3Quaternion orn(0,0,0,1);

		b3Vector4 color=b3MakeVector4(0,0,1,1);

		int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
		int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(0.f,position,orn,colIndex,index,false);

	}
}