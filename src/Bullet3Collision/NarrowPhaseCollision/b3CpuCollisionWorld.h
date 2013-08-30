
#ifndef B3_CPU2_COLLISION_WORLD_H
#define B3_CPU2_COLLISION_WORLD_H

class b3CpuNarrowPhase;
struct b3DynamicBvhBroadphase;
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Vector3.h"

class b3CpuCollisionWorld
{
protected:

	b3DynamicBvhBroadphase* m_bp;
	b3CpuNarrowPhase*		m_np;

public:

	b3CpuCollisionWorld(b3DynamicBvhBroadphase* bp, b3CpuNarrowPhase* np);

	void	addCollidable(int bodyIndex, int collidableIndex,const b3Vector3& position, const b3Quaternion& orientation);

	virtual ~b3CpuCollisionWorld();

};


#endif //B3_CPU_COLLISION_WORLD_H