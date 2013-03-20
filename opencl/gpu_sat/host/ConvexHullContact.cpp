/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


///This file was written by Erwin Coumans
///Separating axis rest based on work from Pierre Terdiman, see
///And contact clipping based on work from Simon Hobbs

//#define BT_DEBUG_SAT_FACE

#include "ConvexHullContact.h"
#include <string.h>//memcpy
#include "btConvexPolyhedronCL.h"
#include "btOptimizedBvh.h"

typedef btAlignedObjectArray<btVector3> btVertexArray;
#include "BulletCommon/btQuickprof.h"

#include <float.h> //for FLT_MAX
#include "basic_initialize/btOpenCLUtils.h"
#include "parallel_primitives/host/btLauncherCL.h"
//#include "AdlQuaternion.h"

#include "../kernels/satKernels.h"
#include "../kernels/satClipHullContacts.h"
#include "../kernels/bvhTraversal.h"

#include "BulletGeometry/btAabbUtil2.h"


#define dot3F4 btDot

GpuSatCollision::GpuSatCollision(cl_context ctx,cl_device_id device, cl_command_queue  q )
:m_context(ctx),
m_device(device),
m_queue(q),
m_findSeparatingAxisKernel(0),
m_totalContactsOut(m_context, m_queue)
{
	m_totalContactsOut.push_back(0);
	
	cl_int errNum=0;

	if (1)
	{
		const char* src = satKernelsCL;
		cl_program satProg = btOpenCLUtils::compileCLProgramFromString(m_context,m_device,src,&errNum,"","opencl/gpu_sat/kernels/sat.cl");
		btAssert(errNum==CL_SUCCESS);

		m_findSeparatingAxisKernel = btOpenCLUtils::compileCLKernelFromString(m_context, m_device,src, "findSeparatingAxisKernel",&errNum,satProg );


		m_findCompoundPairsKernel = btOpenCLUtils::compileCLKernelFromString(m_context, m_device,src, "findCompoundPairsKernel",&errNum,satProg );
	
		m_processCompoundPairsKernel = btOpenCLUtils::compileCLKernelFromString(m_context, m_device,src, "processCompoundPairsKernel",&errNum,satProg );
		btAssert(errNum==CL_SUCCESS);
	}

	if (1)
	{
		const char* srcClip = satClipKernelsCL;
		cl_program satClipContactsProg = btOpenCLUtils::compileCLProgramFromString(m_context,m_device,srcClip,&errNum,"","opencl/gpu_sat/kernels/satClipHullContacts.cl");
		btAssert(errNum==CL_SUCCESS);

		m_clipHullHullKernel = btOpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip, "clipHullHullKernel",&errNum,satClipContactsProg);
		btAssert(errNum==CL_SUCCESS);

		m_clipCompoundsHullHullKernel = btOpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip, "clipCompoundsHullHullKernel",&errNum,satClipContactsProg);
		btAssert(errNum==CL_SUCCESS);
		

        m_findClippingFacesKernel = btOpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip, "findClippingFacesKernel",&errNum,satClipContactsProg);
		btAssert(errNum==CL_SUCCESS);

        m_clipFacesAndContactReductionKernel = btOpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip, "clipFacesAndContactReductionKernel",&errNum,satClipContactsProg);
		btAssert(errNum==CL_SUCCESS);        

		m_clipHullHullConcaveConvexKernel = btOpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip, "clipHullHullConcaveConvexKernel",&errNum,satClipContactsProg);
		btAssert(errNum==CL_SUCCESS);

		m_extractManifoldAndAddContactKernel = btOpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip, "extractManifoldAndAddContactKernel",&errNum,satClipContactsProg);
		btAssert(errNum==CL_SUCCESS);

        m_newContactReductionKernel = btOpenCLUtils::compileCLKernelFromString(m_context, m_device,srcClip,
                            "newContactReductionKernel",&errNum,satClipContactsProg);
		btAssert(errNum==CL_SUCCESS);
	}
   else
	{
		m_clipHullHullKernel=0;
		m_clipCompoundsHullHullKernel = 0;
        m_findClippingFacesKernel = 0;
        m_newContactReductionKernel=0;
        m_clipFacesAndContactReductionKernel = 0;
		m_clipHullHullConcaveConvexKernel = 0;
		m_extractManifoldAndAddContactKernel = 0;
	}

	 if (1)
	{
		const char* srcBvh = bvhTraversalKernelCL;
		cl_program bvhTraversalProg = btOpenCLUtils::compileCLProgramFromString(m_context,m_device,srcBvh,&errNum,"","opencl/gpu_sat/kernels/bvhTraversal.cl");
		btAssert(errNum==CL_SUCCESS);

		m_bvhTraversalKernel = btOpenCLUtils::compileCLKernelFromString(m_context, m_device,srcBvh, "bvhTraversalKernel",&errNum,bvhTraversalProg);
		btAssert(errNum==CL_SUCCESS);

	}
        
	

}

GpuSatCollision::~GpuSatCollision()
{
	
	if (m_findSeparatingAxisKernel)
		clReleaseKernel(m_findSeparatingAxisKernel);

	if (m_findCompoundPairsKernel)
		clReleaseKernel(m_findCompoundPairsKernel);

	if (m_processCompoundPairsKernel)
		clReleaseKernel(m_processCompoundPairsKernel);
    
    if (m_findClippingFacesKernel)
        clReleaseKernel(m_findClippingFacesKernel);
   
    if (m_clipFacesAndContactReductionKernel)
        clReleaseKernel(m_clipFacesAndContactReductionKernel);
    if (m_newContactReductionKernel)
        clReleaseKernel(m_newContactReductionKernel);
    
	if (m_clipHullHullKernel)
		clReleaseKernel(m_clipHullHullKernel);
	if (m_clipCompoundsHullHullKernel)
		clReleaseKernel(m_clipCompoundsHullHullKernel);

	if (m_clipHullHullConcaveConvexKernel)
		clReleaseKernel(m_clipHullHullConcaveConvexKernel);
	if (m_extractManifoldAndAddContactKernel)
		clReleaseKernel(m_extractManifoldAndAddContactKernel);

	if (m_bvhTraversalKernel)
		clReleaseKernel(m_bvhTraversalKernel);

}

struct MyTriangleCallback : public btNodeOverlapCallback
{
	int m_bodyIndexA;
	int m_bodyIndexB;

	virtual void processNode(int subPart, int triangleIndex)
	{
		printf("bodyIndexA %d, bodyIndexB %d\n",m_bodyIndexA,m_bodyIndexB);
		printf("triangleIndex %d\n", triangleIndex);
	}
};

void GpuSatCollision::computeConvexConvexContactsGPUSAT( const btOpenCLArray<btInt2>* pairs, int nPairs,
			const btOpenCLArray<btRigidBodyCL>* bodyBuf,
			btOpenCLArray<btContact4>* contactOut, int& nContacts,
														
			const btOpenCLArray<btConvexPolyhedronCL>& convexData,
			const btOpenCLArray<btVector3>& gpuVertices,
			const btOpenCLArray<btVector3>& gpuUniqueEdges,
			const btOpenCLArray<btGpuFace>& gpuFaces,
			const btOpenCLArray<int>& gpuIndices,
			const btOpenCLArray<btCollidable>& gpuCollidables,
			const btOpenCLArray<btGpuChildShape>& gpuChildShapes,

			const btOpenCLArray<btYetAnotherAabb>& clAabbsWS,
            btOpenCLArray<btVector3>& worldVertsB1GPU,
            btOpenCLArray<btInt4>& clippingFacesOutGPU,
            btOpenCLArray<btVector3>& worldNormalsAGPU,
            btOpenCLArray<btVector3>& worldVertsA1GPU,
            btOpenCLArray<btVector3>& worldVertsB2GPU,    
			btAlignedObjectArray<class btOptimizedBvh*>& bvhData,
			int numObjects,
			int maxTriConvexPairCapacity,
			btOpenCLArray<btInt4>& triangleConvexPairsOut,
			int& numTriConvexPairsOut
			)
{
	if (!nPairs)
		return;

	BT_PROFILE("computeConvexConvexContactsGPUSAT");
   // printf("nContacts = %d\n",nContacts);
    
	btOpenCLArray<btVector3> sepNormals(m_context,m_queue);
	sepNormals.resize(nPairs);
	btOpenCLArray<int> hasSeparatingNormals(m_context,m_queue);
	hasSeparatingNormals.resize(nPairs);
	
	int concaveCapacity=maxTriConvexPairCapacity;
	btOpenCLArray<btVector3> concaveSepNormals(m_context,m_queue);
	concaveSepNormals.resize(concaveCapacity);

	btOpenCLArray<int> numConcavePairsOut(m_context,m_queue);
	numConcavePairsOut.push_back(0);

	int compoundPairCapacity=65536*10;
	btOpenCLArray<btCompoundOverlappingPair> gpuCompoundPairs(m_context,m_queue);
	gpuCompoundPairs.resize(compoundPairCapacity);

	btOpenCLArray<btVector3> gpuCompoundSepNormals(m_context,m_queue);
	gpuCompoundSepNormals.resize(compoundPairCapacity);
	
	
	btOpenCLArray<int> gpuHasCompoundSepNormals(m_context,m_queue);
	gpuHasCompoundSepNormals.resize(compoundPairCapacity);
	
	btOpenCLArray<int> numCompoundPairsOut(m_context,m_queue);
	numCompoundPairsOut.push_back(0);

	int numCompoundPairs = 0;

	bool findSeparatingAxisOnGpu = true;//false;
	int numConcave =0;

	{
		clFinish(m_queue);
		if (findSeparatingAxisOnGpu)
		{
	
		
			BT_PROFILE("findSeparatingAxisKernel");
			btBufferInfoCL bInfo[] = { 
				btBufferInfoCL( pairs->getBufferCL(), true ), 
				btBufferInfoCL( bodyBuf->getBufferCL(),true), 
				btBufferInfoCL( gpuCollidables.getBufferCL(),true), 
				btBufferInfoCL( convexData.getBufferCL(),true),
				btBufferInfoCL( gpuVertices.getBufferCL(),true),
				btBufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
				btBufferInfoCL( gpuFaces.getBufferCL(),true),
				btBufferInfoCL( gpuIndices.getBufferCL(),true),
				btBufferInfoCL( clAabbsWS.getBufferCL(),true),
				btBufferInfoCL( sepNormals.getBufferCL()),
				btBufferInfoCL( hasSeparatingNormals.getBufferCL()),
				btBufferInfoCL( triangleConvexPairsOut.getBufferCL()),
				btBufferInfoCL( concaveSepNormals.getBufferCL()),
				btBufferInfoCL( numConcavePairsOut.getBufferCL())
			};

			btLauncherCL launcher(m_queue, m_findSeparatingAxisKernel);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
			launcher.setConst( nPairs  );
			launcher.setConst( maxTriConvexPairCapacity);

			int num = nPairs;
			launcher.launch1D( num);
			clFinish(m_queue);

			numConcave = numConcavePairsOut.at(0);
			if (numConcave)
			{
				if (numConcave > maxTriConvexPairCapacity)
					numConcave = maxTriConvexPairCapacity;

				triangleConvexPairsOut.resize(numConcave);
				btAlignedObjectArray<btInt4> triangleConvexPairsOutCPU;
				triangleConvexPairsOut.copyToHost(triangleConvexPairsOutCPU);
				printf("-----------------------\n", numConcave);
				printf("got %d concave pairs\n", numConcave);
				btAssert(numConcave = triangleConvexPairsOutCPU.size());

				for (int i=0;i<triangleConvexPairsOutCPU.size();i++)
				{
					printf("bodyIndexA = %d, bodyIndexB = %d\n", triangleConvexPairsOutCPU[i].x,triangleConvexPairsOutCPU[i].y);
					printf("triangleIndex = %d\n", triangleConvexPairsOutCPU[i].z);
				}
				printf("-----------------------\n", numConcave);
				printf("Now using BVH query\n" );
				btAlignedObjectArray<btCollidable> collidablesCPU;
				gpuCollidables.copyToHost(collidablesCPU);
				btAlignedObjectArray<btRigidBodyCL> bodiesCPU;
				bodyBuf->copyToHost(bodiesCPU);
				btAlignedObjectArray<btInt2> pairsCPU;

				btAlignedObjectArray<btYetAnotherAabb> aabbsWSCPU;
				clAabbsWS.copyToHost(aabbsWSCPU);

				pairs->copyToHost(pairsCPU);
				MyTriangleCallback triCallback;
				

				for (int i=0;i<pairsCPU.size();i++)
				{
					int bodyIndexA = pairsCPU[i].x;
					int bodyIndexB = pairsCPU[i].y;

					triCallback.m_bodyIndexA = bodyIndexA;
					triCallback.m_bodyIndexB = bodyIndexB;

					int collidableIndexA = bodiesCPU[bodyIndexA].m_collidableIdx;
					int collidableIndexB = bodiesCPU[bodyIndexB].m_collidableIdx;

					if (collidablesCPU[collidableIndexA].m_shapeType==SHAPE_CONCAVE_TRIMESH)
					{
						//check aabbWS for bodyB against optimized BVH
						btVector3 aabbMin = (const btVector3&)aabbsWSCPU[bodyIndexB].m_min[0];
						aabbMin[3] = 0.f;
						btVector3 aabbMax = (const btVector3&)aabbsWSCPU[bodyIndexB].m_max[0];
						aabbMax[3] = 0.f;
						bvhData[0]->reportAabbOverlappingNodex(&triCallback, aabbMin,aabbMax);
					}
				}

				//now perform the tree query on GPU

				int numNodes = bvhData[0]->getLeafNodeArray().size();
				btOpenCLArray<btQuantizedBvhNode>	treeNodesGPU(this->m_context,this->m_queue,numNodes);
				treeNodesGPU.copyFromHost(bvhData[0]->getQuantizedNodeArray());
				int numSubTrees = bvhData[0]->getSubtreeInfoArray().size();
				btOpenCLArray<btBvhSubtreeInfo>	subTreesGPU(this->m_context,this->m_queue,numSubTrees);
				subTreesGPU.copyFromHost(bvhData[0]->getSubtreeInfoArray());

				btVector3 bvhAabbMin = bvhData[0]->m_bvhAabbMin;
				btVector3 bvhAabbMax = bvhData[0]->m_bvhAabbMax;
				btVector3 bvhQuantization = bvhData[0]->m_bvhQuantization;
				{
					int np = numConcavePairsOut.at(0);
					printf("np=%d\n", np);
					btLauncherCL launcher(m_queue, m_bvhTraversalKernel);
					launcher.setBuffer( pairs->getBufferCL());
					launcher.setBuffer(  bodyBuf->getBufferCL());
					launcher.setBuffer( gpuCollidables.getBufferCL());
					launcher.setBuffer( clAabbsWS.getBufferCL());
					launcher.setBuffer( triangleConvexPairsOut.getBufferCL());
					launcher.setBuffer( numConcavePairsOut.getBufferCL());
					launcher.setBuffer( subTreesGPU.getBufferCL());
					launcher.setBuffer( treeNodesGPU.getBufferCL());
					launcher.setConst( bvhAabbMin);
					launcher.setConst( bvhAabbMax);
					launcher.setConst( bvhQuantization);
					launcher.setConst(numSubTrees);
					launcher.setConst( nPairs  );
					launcher.setConst( maxTriConvexPairCapacity);
					int num = nPairs;
					launcher.launch1D( num);
					clFinish(m_queue);
					np = numConcavePairsOut.at(0);
					triangleConvexPairsOut.resize(np);
					btAlignedObjectArray<btInt4> pairsOutCPU;
					triangleConvexPairsOut.copyToHost(pairsOutCPU);


					printf("np=%d\n", np);

				}
				printf("-----------------------\n", numConcave);
			}
			



			{
				BT_PROFILE("findCompoundPairsKernel");
				btBufferInfoCL bInfo[] = 
				{ 
					btBufferInfoCL( pairs->getBufferCL(), true ), 
					btBufferInfoCL( bodyBuf->getBufferCL(),true), 
					btBufferInfoCL( gpuCollidables.getBufferCL(),true), 
					btBufferInfoCL( convexData.getBufferCL(),true),
					btBufferInfoCL( gpuVertices.getBufferCL(),true),
					btBufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
					btBufferInfoCL( gpuFaces.getBufferCL(),true),
					btBufferInfoCL( gpuIndices.getBufferCL(),true),
					btBufferInfoCL( clAabbsWS.getBufferCL(),true),
					btBufferInfoCL( gpuChildShapes.getBufferCL(),true),
					btBufferInfoCL( gpuCompoundPairs.getBufferCL()),
					btBufferInfoCL( numCompoundPairsOut.getBufferCL())
				};

				btLauncherCL launcher(m_queue, m_findCompoundPairsKernel);
				launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
				launcher.setConst( nPairs  );
				launcher.setConst( compoundPairCapacity);

				int num = nPairs;
				launcher.launch1D( num);
				clFinish(m_queue);
			}


			numCompoundPairs = numCompoundPairsOut.at(0);
			//printf("numCompoundPairs =%d\n",numCompoundPairs );
			if (numCompoundPairs > compoundPairCapacity)
				numCompoundPairs = compoundPairCapacity;

			gpuCompoundPairs.resize(numCompoundPairs);
			gpuHasCompoundSepNormals.resize(numCompoundPairs);
			gpuCompoundSepNormals.resize(numCompoundPairs);
			

			if (numCompoundPairs)
			{

				BT_PROFILE("processCompoundPairsKernel");
				btBufferInfoCL bInfo[] = 
				{ 
					btBufferInfoCL( gpuCompoundPairs.getBufferCL(), true ), 
					btBufferInfoCL( bodyBuf->getBufferCL(),true), 
					btBufferInfoCL( gpuCollidables.getBufferCL(),true), 
					btBufferInfoCL( convexData.getBufferCL(),true),
					btBufferInfoCL( gpuVertices.getBufferCL(),true),
					btBufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
					btBufferInfoCL( gpuFaces.getBufferCL(),true),
					btBufferInfoCL( gpuIndices.getBufferCL(),true),
					btBufferInfoCL( clAabbsWS.getBufferCL(),true),
					btBufferInfoCL( gpuChildShapes.getBufferCL(),true),
					btBufferInfoCL( gpuCompoundSepNormals.getBufferCL()),
					btBufferInfoCL( gpuHasCompoundSepNormals.getBufferCL())
				};

				btLauncherCL launcher(m_queue, m_processCompoundPairsKernel);
				launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
				launcher.setConst( numCompoundPairs  );

				int num = numCompoundPairs;
				launcher.launch1D( num);
				clFinish(m_queue);
			
			}


			//printf("numConcave  = %d\n",numConcave);

		}//if (findSeparatingAxisOnGpu)


//		printf("hostNormals.size()=%d\n",hostNormals.size());
		//int numPairs = pairCount.at(0);
		
		
		
	}
#ifdef __APPLE__
	bool contactClippingOnGpu = true;
#else
 bool contactClippingOnGpu = true;
#endif
	
	if (contactClippingOnGpu)
	{
		//BT_PROFILE("clipHullHullKernel");

		
		m_totalContactsOut.copyFromHostPointer(&nContacts,1,0,true);

		//concave-convex contact clipping

		if (numConcave)
		{
			BT_PROFILE("clipHullHullConcaveConvexKernel");
			nContacts = m_totalContactsOut.at(0);
			btBufferInfoCL bInfo[] = { 
				btBufferInfoCL( triangleConvexPairsOut.getBufferCL(), true ), 
				btBufferInfoCL( bodyBuf->getBufferCL(),true), 
				btBufferInfoCL( gpuCollidables.getBufferCL(),true), 
				btBufferInfoCL( convexData.getBufferCL(),true),
				btBufferInfoCL( gpuVertices.getBufferCL(),true),
				btBufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
				btBufferInfoCL( gpuFaces.getBufferCL(),true),
				btBufferInfoCL( gpuIndices.getBufferCL(),true),
				btBufferInfoCL( concaveSepNormals.getBufferCL()),
				btBufferInfoCL( contactOut->getBufferCL()),
				btBufferInfoCL( m_totalContactsOut.getBufferCL())	
			};
			btLauncherCL launcher(m_queue, m_clipHullHullConcaveConvexKernel);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
			launcher.setConst( numConcave  );
			int num = numConcave;
			launcher.launch1D( num);
			clFinish(m_queue);
			nContacts = m_totalContactsOut.at(0);
		}


		//convex-convex contact clipping
        if (1)
		{
			BT_PROFILE("clipHullHullKernel");
			bool breakupKernel = false;

#ifdef __APPLE__
			breakupKernel = true;
#endif

			if (breakupKernel)
			{


			
            int vertexFaceCapacity = 64;
            
            
            worldVertsB1GPU.resize(vertexFaceCapacity*nPairs);
            
            
            clippingFacesOutGPU.resize(nPairs);
            
            
            worldNormalsAGPU.resize(nPairs);
            
            
            worldVertsA1GPU.resize(vertexFaceCapacity*nPairs);
            
             
            worldVertsB2GPU.resize(vertexFaceCapacity*nPairs);
        
            
            
            {
				BT_PROFILE("findClippingFacesKernel");
            btBufferInfoCL bInfo[] = {
                btBufferInfoCL( pairs->getBufferCL(), true ),
                btBufferInfoCL( bodyBuf->getBufferCL(),true),
                btBufferInfoCL( gpuCollidables.getBufferCL(),true),
                btBufferInfoCL( convexData.getBufferCL(),true),
                btBufferInfoCL( gpuVertices.getBufferCL(),true),
                btBufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
                btBufferInfoCL( gpuFaces.getBufferCL(),true), 
                btBufferInfoCL( gpuIndices.getBufferCL(),true),
                btBufferInfoCL( sepNormals.getBufferCL()),
                btBufferInfoCL( hasSeparatingNormals.getBufferCL()),
                btBufferInfoCL( clippingFacesOutGPU.getBufferCL()),
                btBufferInfoCL( worldVertsA1GPU.getBufferCL()),
                btBufferInfoCL( worldNormalsAGPU.getBufferCL()),
                btBufferInfoCL( worldVertsB1GPU.getBufferCL())
            };
            
            btLauncherCL launcher(m_queue, m_findClippingFacesKernel);
            launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
            launcher.setConst( vertexFaceCapacity);
            launcher.setConst( nPairs  );
            int num = nPairs;
            launcher.launch1D( num);
            clFinish(m_queue);

            }
            
  
          
            

            ///clip face B against face A, reduce contacts and append them to a global contact array
            if (1)
            {
				BT_PROFILE("clipFacesAndContactReductionKernel");
				//nContacts = m_totalContactsOut.at(0);
				//int h = hasSeparatingNormals.at(0);
				//int4 p = clippingFacesOutGPU.at(0);
                btBufferInfoCL bInfo[] = {
                    btBufferInfoCL( pairs->getBufferCL(), true ),
                    btBufferInfoCL( bodyBuf->getBufferCL(),true),
                    btBufferInfoCL( sepNormals.getBufferCL()),
                    btBufferInfoCL( hasSeparatingNormals.getBufferCL()),
					btBufferInfoCL( contactOut->getBufferCL()),
                    btBufferInfoCL( clippingFacesOutGPU.getBufferCL()),
                    btBufferInfoCL( worldVertsA1GPU.getBufferCL()),
                    btBufferInfoCL( worldNormalsAGPU.getBufferCL()),
                    btBufferInfoCL( worldVertsB1GPU.getBufferCL()),
                    btBufferInfoCL( worldVertsB2GPU.getBufferCL()),
					btBufferInfoCL( m_totalContactsOut.getBufferCL())
                };
                
                btLauncherCL launcher(m_queue, m_clipFacesAndContactReductionKernel);
                launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
                launcher.setConst(vertexFaceCapacity);

				launcher.setConst( nPairs  );
                int debugMode = 0;
				launcher.setConst( debugMode);

				/*
				int serializationBytes = launcher.getSerializationBufferSize();
				unsigned char* buf = (unsigned char*)malloc(serializationBytes+1);
				int actualWritten = launcher.serializeArguments(buf,serializationBytes+1);
				FILE* f = fopen("clipFacesAndContactReductionKernel.bin","wb");
				fwrite(buf,actualWritten,1,f);
				fclose(f);
				free(buf);
				printf("serializationBytes=%d, actualWritten=%d\n",serializationBytes,actualWritten);
				*/

                int num = nPairs;

                launcher.launch1D( num);
                clFinish(m_queue);
                {
//                    nContacts = m_totalContactsOut.at(0);
  //                  printf("nContacts = %d\n",nContacts);
                    
                    contactOut->reserve(nContacts+nPairs);
                    
                    {
                        BT_PROFILE("newContactReductionKernel");
                            btBufferInfoCL bInfo[] =
                        {
                            btBufferInfoCL( pairs->getBufferCL(), true ),
                            btBufferInfoCL( bodyBuf->getBufferCL(),true),
                            btBufferInfoCL( sepNormals.getBufferCL()),
                            btBufferInfoCL( hasSeparatingNormals.getBufferCL()),
                            btBufferInfoCL( contactOut->getBufferCL()),
                            btBufferInfoCL( clippingFacesOutGPU.getBufferCL()),
                            btBufferInfoCL( worldVertsB2GPU.getBufferCL()),
                            btBufferInfoCL( m_totalContactsOut.getBufferCL())
                        };
                        
                        btLauncherCL launcher(m_queue, m_newContactReductionKernel);
                        launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
                        launcher.setConst(vertexFaceCapacity);
                        launcher.setConst( nPairs  );
                        int num = nPairs;
                        
                        launcher.launch1D( num);
                    }
                    nContacts = m_totalContactsOut.at(0);
                    contactOut->resize(nContacts);
                    
//                    Contact4 pt = contactOut->at(0);
                    
  //                  printf("nContacts = %d\n",nContacts);
                }
            }
	}            
	else
	{
	 
		if (nPairs)
		{
			btBufferInfoCL bInfo[] = {
				btBufferInfoCL( pairs->getBufferCL(), true ), 
				btBufferInfoCL( bodyBuf->getBufferCL(),true), 
				btBufferInfoCL( gpuCollidables.getBufferCL(),true), 
				btBufferInfoCL( convexData.getBufferCL(),true),
				btBufferInfoCL( gpuVertices.getBufferCL(),true),
				btBufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
				btBufferInfoCL( gpuFaces.getBufferCL(),true),
				btBufferInfoCL( gpuIndices.getBufferCL(),true),
				btBufferInfoCL( sepNormals.getBufferCL()),
				btBufferInfoCL( hasSeparatingNormals.getBufferCL()),
				btBufferInfoCL( contactOut->getBufferCL()),
				btBufferInfoCL( m_totalContactsOut.getBufferCL())	
			};
			btLauncherCL launcher(m_queue, m_clipHullHullKernel);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
			launcher.setConst( nPairs  );
			int num = nPairs;
			launcher.launch1D( num);
			clFinish(m_queue);
		
			nContacts = m_totalContactsOut.at(0);
			contactOut->resize(nContacts);
		}

		int nCompoundsPairs = gpuCompoundPairs.size();

		if (nCompoundsPairs)
		{
				btBufferInfoCL bInfo[] = {
				btBufferInfoCL( gpuCompoundPairs.getBufferCL(), true ), 
				btBufferInfoCL( bodyBuf->getBufferCL(),true), 
				btBufferInfoCL( gpuCollidables.getBufferCL(),true), 
				btBufferInfoCL( convexData.getBufferCL(),true),
				btBufferInfoCL( gpuVertices.getBufferCL(),true),
				btBufferInfoCL( gpuUniqueEdges.getBufferCL(),true),
				btBufferInfoCL( gpuFaces.getBufferCL(),true),
				btBufferInfoCL( gpuIndices.getBufferCL(),true),
				btBufferInfoCL( gpuChildShapes.getBufferCL(),true),
				btBufferInfoCL( gpuCompoundSepNormals.getBufferCL(),true),
				btBufferInfoCL( gpuHasCompoundSepNormals.getBufferCL(),true),
				btBufferInfoCL( contactOut->getBufferCL()),
				btBufferInfoCL( m_totalContactsOut.getBufferCL())	
			};
			btLauncherCL launcher(m_queue, m_clipCompoundsHullHullKernel);
			launcher.setBuffers( bInfo, sizeof(bInfo)/sizeof(btBufferInfoCL) );
			launcher.setConst( nCompoundsPairs  );
			int num = nCompoundsPairs;
			launcher.launch1D( num);
			clFinish(m_queue);
		
			nContacts = m_totalContactsOut.at(0);
			contactOut->resize(nContacts);
		}
		}
		}

	}
}
