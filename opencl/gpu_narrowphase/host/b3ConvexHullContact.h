
#ifndef _CONVEX_HULL_CONTACT_H
#define _CONVEX_HULL_CONTACT_H

#include "parallel_primitives/host/btOpenCLArray.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3RigidBodyCL.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "b3ConvexUtility.h"
#include "b3ConvexPolyhedronCL.h"
#include "b3Collidable.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Contact4.h"
#include "parallel_primitives/host/btInt2.h"
#include "parallel_primitives/host/btInt4.h"
#include "b3OptimizedBvh.h"

//#include "../../dynamics/basic_demo/Stubs/ChNarrowPhase.h"


struct btYetAnotherAabb
{
	union
	{
		float m_min[4];
		int m_minIndices[4];
	};
	union
	{
		float m_max[4];
		//int m_signedMaxIndices[4];
		//unsigned int m_unsignedMaxIndices[4];
	};
};

struct GpuSatCollision
{
	cl_context				m_context;
	cl_device_id			m_device;
	cl_command_queue		m_queue;
	cl_kernel				m_findSeparatingAxisKernel;
	cl_kernel				m_findConcaveSeparatingAxisKernel;
	cl_kernel				m_findCompoundPairsKernel;
	cl_kernel				m_processCompoundPairsKernel;

	cl_kernel				m_clipHullHullKernel;
	cl_kernel				m_clipCompoundsHullHullKernel;
    
    cl_kernel               m_clipFacesAndContactReductionKernel;
    cl_kernel               m_findClippingFacesKernel;
    
	cl_kernel				m_clipHullHullConcaveConvexKernel;
	cl_kernel				m_extractManifoldAndAddContactKernel;
    cl_kernel               m_newContactReductionKernel;

	cl_kernel				m_bvhTraversalKernel;
	cl_kernel				m_primitiveContactsKernel;
	cl_kernel				m_findConcaveSphereContactsKernel;

	cl_kernel				m_processCompoundPairsPrimitivesKernel;
    

	btOpenCLArray<int>		m_totalContactsOut;

	GpuSatCollision(cl_context ctx,cl_device_id device, cl_command_queue  q );
	virtual ~GpuSatCollision();
	

	void computeConvexConvexContactsGPUSAT( const btOpenCLArray<btInt2>* pairs, int nPairs, 
			const btOpenCLArray<b3RigidBodyCL>* bodyBuf,
			btOpenCLArray<b3Contact4>* contactOut, int& nContacts,
			int maxContactCapacity,
			const btOpenCLArray<b3ConvexPolyhedronCL>& hostConvexData,
			const btOpenCLArray<b3Vector3>& vertices,
			const btOpenCLArray<b3Vector3>& uniqueEdges,
			const btOpenCLArray<btGpuFace>& faces,
			const btOpenCLArray<int>& indices,
			const btOpenCLArray<b3Collidable>& gpuCollidables,
			const btOpenCLArray<btGpuChildShape>& gpuChildShapes,

			const btOpenCLArray<btYetAnotherAabb>& clAabbs,
           btOpenCLArray<b3Vector3>& worldVertsB1GPU,
           btOpenCLArray<btInt4>& clippingFacesOutGPU,
           btOpenCLArray<b3Vector3>& worldNormalsAGPU,
           btOpenCLArray<b3Vector3>& worldVertsA1GPU,
           btOpenCLArray<b3Vector3>& worldVertsB2GPU,
		   b3AlignedObjectArray<class b3OptimizedBvh*>& bvhData,
		   btOpenCLArray<btQuantizedBvhNode>*	treeNodesGPU,
			btOpenCLArray<btBvhSubtreeInfo>*	subTreesGPU,
			int numObjects,
			int maxTriConvexPairCapacity,
			btOpenCLArray<btInt4>& triangleConvexPairs,
			int& numTriConvexPairsOut
			);


};

#endif //_CONVEX_HULL_CONTACT_H
