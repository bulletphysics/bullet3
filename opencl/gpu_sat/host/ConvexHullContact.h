
#ifndef _CONVEX_HULL_CONTACT_H
#define _CONVEX_HULL_CONTACT_H

#include "parallel_primitives/host/btOpenCLArray.h"
#include "btRigidBodyCL.h"
#include "BulletCommon/btAlignedObjectArray.h"
#include "btConvexUtility.h"
#include "btConvexPolyhedronCL.h"
#include "btCollidable.h"
#include "btContact4.h"
#include "parallel_primitives/host/btInt2.h"
#include "parallel_primitives/host/btInt4.h"

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
	cl_kernel				m_findCompoundPairsKernel;
	cl_kernel				m_processCompoundPairsKernel;

	cl_kernel				m_clipHullHullKernel;
	cl_kernel				m_clipCompoundsHullHullKernel;
    
    cl_kernel               m_clipFacesAndContactReductionKernel;
    cl_kernel               m_findClippingFacesKernel;
    
	cl_kernel				m_clipHullHullConcaveConvexKernel;
	cl_kernel				m_extractManifoldAndAddContactKernel;
    cl_kernel               m_newContactReductionKernel;
    

	btOpenCLArray<int>		m_totalContactsOut;

	GpuSatCollision(cl_context ctx,cl_device_id device, cl_command_queue  q );
	virtual ~GpuSatCollision();
	

	void computeConvexConvexContactsGPUSAT( const btOpenCLArray<btInt2>* pairs, int nPairs, 
			const btOpenCLArray<btRigidBodyCL>* bodyBuf,
			btOpenCLArray<btContact4>* contactOut, int& nContacts,
			const btOpenCLArray<btConvexPolyhedronCL>& hostConvexData,
			const btOpenCLArray<btVector3>& vertices,
			const btOpenCLArray<btVector3>& uniqueEdges,
			const btOpenCLArray<btGpuFace>& faces,
			const btOpenCLArray<int>& indices,
			const btOpenCLArray<btCollidable>& gpuCollidables,
			const btOpenCLArray<btGpuChildShape>& gpuChildShapes,

			const btOpenCLArray<btYetAnotherAabb>& clAabbs,
           btOpenCLArray<btVector3>& worldVertsB1GPU,
           btOpenCLArray<btInt4>& clippingFacesOutGPU,
           btOpenCLArray<btVector3>& worldNormalsAGPU,
           btOpenCLArray<btVector3>& worldVertsA1GPU,
           btOpenCLArray<btVector3>& worldVertsB2GPU,
			int numObjects,
			int maxTriConvexPairCapacity,
			btOpenCLArray<btInt4>& triangleConvexPairs,
			int& numTriConvexPairsOut
			);


};

#endif //_CONVEX_HULL_CONTACT_H
