
#ifndef B3_CPU_DYNAMICS_WORLD_H
#define B3_CPU_DYNAMICS_WORLD_H

class b3DefaultCollisionConfiguration;
class b3CollisionDispatcher;
struct b3DynamicBvhBroadphase;
class b3SequentialImpulseConstraintSolver;

#include "BulletDynamics/Dynamics/b3DiscreteDynamicsWorld.h"

class b3CpuDynamicsWorld : public b3DiscreteDynamicsWorld
{
	
public:
	
	b3CpuDynamicsWorld();
	
	virtual ~b3CpuDynamicsWorld();


};

#endif //B3_CPU_DYNAMICS_WORLD_H
