
#ifndef BT_CPU_DYNAMICS_WORLD_H
#define BT_CPU_DYNAMICS_WORLD_H

class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
struct b3DynamicBvhBroadphase;
class btSequentialImpulseConstraintSolver;

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

class btCpuDynamicsWorld : public btDiscreteDynamicsWorld
{
	
public:
	
	btCpuDynamicsWorld();
	
	virtual ~btCpuDynamicsWorld();


};

#endif //BT_CPU_DYNAMICS_WORLD_H
