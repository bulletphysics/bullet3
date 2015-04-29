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
	
	int m_maxRayRigidPairs;
	
public:
	b3GpuRaycast(cl_context ctx,cl_device_id device, cl_command_queue  q);
	virtual ~b3GpuRaycast();

	void castRaysHost(const b3AlignedObjectArray<b3RayInfo>& raysIn,	b3AlignedObjectArray<b3RayHit>& hitResults,
		int numBodies, const struct b3RigidBodyData* bodies, int numCollidables, const struct b3Collidable* collidables,
		const struct b3GpuNarrowPhaseInternalData* narrowphaseData);
	
	void castRays(const b3AlignedObjectArray<b3RayInfo>& rays, b3AlignedObjectArray<b3RayHit>& hitResults,
		int numBodies, const struct b3RigidBodyData* bodies, int numCollidables, const struct b3Collidable* collidables,
		const struct b3GpuNarrowPhaseInternalData* narrowphaseData, class b3GpuBroadphaseInterface* broadphase);
	
	///@brief Make sure that setMaxRayRigidPairs() is correct.
	///@remarks
	///An intermediate step of b3GpuRaycast::castRaysUsingPairs() involves the generation of 'ray-rigid' pairs;
	///each pair represents an intersection between a ray and a rigid body AABB.
	///The number of ray-rigid pairs to allocate is controlled by setMaxRayRigidPairs().
	///If the number of ray-rigid pairs is exceeded, additional intersections are discarded.
	///In other words, there will be missing collisions.
	///@par
	///In order to never miss any intersections, it should be set to num_rays * num_rigid_bodies.
	///However, it can be set lower to conserve memory. Ideally, the number of ray-rigid pairs is
	///set to num_rays * average_num_rigids, where average_num_rigids is the number of rigid body
	///AABBs that are expected to intersect each ray on average. Each pair consumes
	///sizeof(b3Int2) + sizeof(b3Vector3), or 24 bytes.
	void castRaysUsingPairs(const b3AlignedObjectArray<b3RayInfo>& rays, b3AlignedObjectArray<b3RayHit>& hitResults,
		int numBodies, const struct b3RigidBodyData* bodies, int numCollidables, const struct b3Collidable* collidables,
		const b3GpuNarrowPhaseInternalData* narrowphaseData, b3GpuBroadphaseInterface* broadphase);
	
	///Only used by castRaysUsingPairs()
	void setMaxRayRigidPairs(int maxRayRigidPairs) { m_maxRayRigidPairs = maxRayRigidPairs; }
	int getMaxRayRigidPairs() const { return m_maxRayRigidPairs; }
};

#endif //B3_GPU_RAYCAST_H
