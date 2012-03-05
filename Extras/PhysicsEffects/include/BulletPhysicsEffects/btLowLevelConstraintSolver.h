/*
   Copyright (C) 2010 Sony Computer Entertainment Inc.
   All rights reserved.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/

#ifndef __BT_LOW_LEVEL_CONSTRAINT_SOLVER_H
#define __BT_LOW_LEVEL_CONSTRAINT_SOLVER_H

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"




#include "LinearMath/btScalar.h"
#include "BulletMultiThreaded/PlatformDefinitions.h"
#include "physics_effects/low_level/pfx_low_level_include.h"
#include "../src/low_level/solver/pfx_parallel_group.h"

using namespace sce::PhysicsEffects;


class btPersistentManifold;

enum {
	PFX_CONSTRAINT_SOLVER_CMD_SETUP_SOLVER_BODIES,
	PFX_CONSTRAINT_SOLVER_CMD_SETUP_CONTACT_CONSTRAINTS,
	PFX_CONSTRAINT_SOLVER_CMD_SETUP_JOINT_CONSTRAINTS,
	PFX_CONSTRAINT_SOLVER_CMD_SOLVE_CONSTRAINTS,
	PFX_CONSTRAINT_SOLVER_CMD_POST_SOLVER
};


struct PfxSetupContactConstraintsIO {
	PfxConstraintPair *offsetContactPairs;
	uint32_t numContactPairs1;
	class TrbState *offsetRigStates;
	struct PfxSolverBody *offsetSolverBodies;
	uint32_t numRigidBodies;
	float separateBias;
	float timeStep;
	class btCriticalSection* criticalSection;
};



struct PfxSolveConstraintsIO {
	PfxParallelGroup *contactParallelGroup;
	PfxParallelBatch *contactParallelBatches;
	PfxConstraintPair *contactPairs;
	uint32_t numContactPairs;
	btPersistentManifold *offsetContactManifolds;
	PfxParallelGroup *jointParallelGroup;
	PfxParallelBatch *jointParallelBatches;
	PfxConstraintPair *jointPairs;
	uint32_t numJointPairs;
	TrbState *offsetRigStates;
	PfxSolverBody *offsetSolverBodies;
	uint32_t numRigidBodies;
	uint32_t iteration;

	uint32_t	taskId;
	
	class btBarrier* barrier;

};

struct PfxPostSolverIO {
	TrbState *states;
	PfxSolverBody *solverBodies;
	uint32_t numRigidBodies;
};

ATTRIBUTE_ALIGNED16(struct) btConstraintSolverIO {
	uint8_t cmd;
	union {
		PfxSetupContactConstraintsIO setupContactConstraints;
		PfxSolveConstraintsIO solveConstraints;
		PfxPostSolverIO postSolver;
	};
	
	//SPU only
	uint32_t barrierAddr2;
	uint32_t criticalsectionAddr2;
	uint32_t maxTasks1;
};




void	SolverThreadFunc(void* userPtr,void* lsMemory);
void*	SolverlsMemoryFunc();
///The btLowLevelConstraintSolver performs computations on constraint rows in parallel
///Using the cross-platform threading it supports Windows, Linux, Mac OSX and PlayStation 3 Cell SPUs
class btLowLevelConstraintSolver : public btSequentialImpulseConstraintSolver
{
	
protected:
	struct btParallelSolverMemoryCache*	m_memoryCache;

	class btThreadSupportInterface*	m_solverThreadSupport;

	struct btConstraintSolverIO* m_solverIO;
	class btBarrier*			m_barrier;
	class btCriticalSection*	m_criticalSection;


public:

	btLowLevelConstraintSolver(class btThreadSupportInterface* solverThreadSupport);
	
	virtual ~btLowLevelConstraintSolver();

	virtual btScalar solveGroup(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifold,int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& info, btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc,btDispatcher* dispatcher);

};



#endif //__BT_LOW_LEVEL_CONSTRAINT_SOLVER_H