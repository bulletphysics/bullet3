#ifndef B3_GPU_RAYCAST_H
#define B3_GPU_RAYCAST_H

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLInclude.h"

#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3RaycastInfo.h"



class b3GpuRaycast
{
protected:
	struct b3GpuRaycastInternalData* m_data;
public:
	b3GpuRaycast(cl_context ctx,cl_device_id device, cl_command_queue  q);
	virtual ~b3GpuRaycast();

	void castRaysHost(const b3AlignedObjectArray<b3RayInfo>& raysIn,	b3AlignedObjectArray<b3RayHit>& hitResults,
		int numBodies, const struct b3RigidBodyCL* bodies, int numCollidables, const struct b3Collidable* collidables,
		const struct b3GpuNarrowPhaseInternalData* narrowphaseData);

	void castRays(const b3AlignedObjectArray<b3RayInfo>& rays,	b3AlignedObjectArray<b3RayHit>& hitResults,
		int numBodies,const struct b3RigidBodyCL* bodies, int numCollidables, const struct b3Collidable* collidables,
		const struct b3GpuNarrowPhaseInternalData* narrowphaseData
		);
	
/*			const b3OpenCLArray<b3RigidBodyCL>* bodyBuf,
			b3OpenCLArray<b3Contact4>* contactOut, int& nContacts,
			int maxContactCapacity,
			const b3OpenCLArray<b3ConvexPolyhedronCL>& hostConvexData,
			const b3OpenCLArray<b3Vector3>& vertices,
			const b3OpenCLArray<b3Vector3>& uniqueEdges,
			const b3OpenCLArray<b3GpuFace>& faces,
			const b3OpenCLArray<int>& indices,
			const b3OpenCLArray<b3Collidable>& gpuCollidables,
			const b3OpenCLArray<b3GpuChildShape>& gpuChildShapes,

			const b3OpenCLArray<b3YetAnotherAabb>& clAabbs,
           b3OpenCLArray<b3Vector3>& worldVertsB1GPU,
           b3OpenCLArray<b3Int4>& clippingFacesOutGPU,
           b3OpenCLArray<b3Vector3>& worldNormalsAGPU,
           b3OpenCLArray<b3Vector3>& worldVertsA1GPU,
           b3OpenCLArray<b3Vector3>& worldVertsB2GPU,
		   b3AlignedObjectArray<class b3OptimizedBvh*>& bvhData,
		   b3OpenCLArray<b3QuantizedBvhNode>*	treeNodesGPU,
			b3OpenCLArray<b3BvhSubtreeInfo>*	subTreesGPU,
			b3OpenCLArray<b3BvhInfo>*	bvhInfo,
			int numObjects,
			int maxTriConvexPairCapacity,
			b3OpenCLArray<b3Int4>& triangleConvexPairs,
			int& numTriConvexPairsOut
			*/
		
};

#endif //B3_GPU_RAYCAST_H
