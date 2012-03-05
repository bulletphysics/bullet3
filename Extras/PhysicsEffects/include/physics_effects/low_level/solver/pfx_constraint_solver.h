/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#ifndef _SCE_PFX_CONSTRAINT_SOLVER_H
#define _SCE_PFX_CONSTRAINT_SOLVER_H

#include "../../base_level/rigidbody/pfx_rigid_body.h"
#include "../../base_level/rigidbody/pfx_rigid_state.h"
#include "../../base_level/solver/pfx_solver_body.h"
#include "../../base_level/solver/pfx_constraint_pair.h"
#include "../../base_level/solver/pfx_joint.h"
#include "../../base_level/collision/pfx_contact_manifold.h"

#include "../task/pfx_task_manager.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// Setup Solver Bodies

struct PfxSetupSolverBodiesParam {
	PfxRigidState *states;
	PfxRigidBody *bodies;
	PfxSolverBody *solverBodies;
	PfxUInt32 numRigidBodies;
};

PfxInt32 pfxSetupSolverBodies(PfxSetupSolverBodiesParam &param);

PfxInt32 pfxSetupSolverBodies(PfxSetupSolverBodiesParam &param,PfxTaskManager *taskManager);

///////////////////////////////////////////////////////////////////////////////
// Setup Constraints

struct PfxSetupContactConstraintsParam {
	PfxConstraintPair *contactPairs;
	PfxUInt32 numContactPairs;
	PfxContactManifold *offsetContactManifolds;
	PfxRigidState *offsetRigidStates;
	PfxRigidBody *offsetRigidBodies;
	PfxSolverBody *offsetSolverBodies;
	PfxUInt32 numRigidBodies;
	PfxFloat timeStep;
	PfxFloat separateBias;
	
	PfxSetupContactConstraintsParam()
	{
		timeStep = 0.016f;
		separateBias = 0.2f;
	}
};

PfxInt32 pfxSetupContactConstraints(PfxSetupContactConstraintsParam &param);

PfxInt32 pfxSetupContactConstraints(PfxSetupContactConstraintsParam &param,PfxTaskManager *taskManager);

struct PfxSetupJointConstraintsParam {
	PfxConstraintPair *jointPairs;
	PfxUInt32 numJointPairs;
	PfxJoint *offsetJoints;
	PfxRigidState *offsetRigidStates;
	PfxRigidBody *offsetRigidBodies;
	PfxSolverBody *offsetSolverBodies;
	PfxUInt32 numRigidBodies;
	PfxFloat timeStep;

	PfxSetupJointConstraintsParam()
	{
		timeStep = 0.016f;
	}
};

PfxInt32 pfxSetupJointConstraints(PfxSetupJointConstraintsParam &param);

PfxInt32 pfxSetupJointConstraints(PfxSetupJointConstraintsParam &param,PfxTaskManager *taskManager);

///////////////////////////////////////////////////////////////////////////////
// Solve Constraints

struct PfxSolveConstraintsParam {
	void *workBuff;
	PfxUInt32 workBytes;
	PfxConstraintPair *contactPairs;
	PfxUInt32 numContactPairs;
	PfxContactManifold *offsetContactManifolds;
	PfxConstraintPair *jointPairs;
	PfxUInt32 numJointPairs;
	PfxJoint *offsetJoints;
	PfxRigidState *offsetRigidStates;
	PfxSolverBody *offsetSolverBodies;
	PfxUInt32 numRigidBodies;
	PfxUInt32 iteration;
	
	PfxSolveConstraintsParam()
	{
		iteration = 5;
	}
};

PfxUInt32 pfxGetWorkBytesOfSolveConstraints(PfxUInt32 numRigidBodies,PfxUInt32 numContactPairs,PfxUInt32 numJointPairs,PfxUInt32 maxTasks=1);

PfxInt32 pfxSolveConstraints(PfxSolveConstraintsParam &param);

PfxInt32 pfxSolveConstraints(PfxSolveConstraintsParam &param,PfxTaskManager *taskManager);

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_CONSTRAINT_SOLVER_H

