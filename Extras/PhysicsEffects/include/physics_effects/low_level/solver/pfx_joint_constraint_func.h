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

#ifndef _SCE_PFX_JOINT_CONSTRAINT_FUNC_H_
#define _SCE_PFX_JOINT_CONSTRAINT_FUNC_H_

#include "../../base_level/rigidbody/pfx_rigid_state.h"
#include "../../base_level/solver/pfx_solver_body.h"
#include "../../base_level/solver/pfx_joint.h"
#include "../../base_level/solver/pfx_joint_constraint.h"

namespace sce {
namespace PhysicsEffects {
///////////////////////////////////////////////////////////////////////////////
// Setup Joint Constraint Func

typedef void (*PfxSetupJointConstraintFunc)(
	PfxJoint &joint,
	const PfxRigidState &stateA,
	const PfxRigidState &stateB,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB,
	PfxFloat timeStep);

PfxSetupJointConstraintFunc pfxGetSetupJointConstraintFunc(PfxUInt8 jointType);

int pfxSetSetupJointConstraintFunc(PfxUInt8 jointType,PfxSetupJointConstraintFunc func);

///////////////////////////////////////////////////////////////////////////////
// Warm Start Joint Constraint Func

typedef void (*PfxWarmStartJointConstraintFunc)(
	PfxJoint &joint,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB);

PfxWarmStartJointConstraintFunc pfxGetWarmStartJointConstraintFunc(PfxUInt8 jointType);

int pfxSetWarmStartJointConstraintFunc(PfxUInt8 jointType,PfxWarmStartJointConstraintFunc func);

///////////////////////////////////////////////////////////////////////////////
// Solve Joint Constraint Func

typedef void (*PfxSolveJointConstraintFunc)(
	PfxJoint &joint,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB);

PfxSolveJointConstraintFunc pfxGetSolveJointConstraintFunc(PfxUInt8 jointType);

int pfxSetSolveJointConstraintFunc(PfxUInt8 jointType,PfxSolveJointConstraintFunc func);

} //namespace PhysicsEffects
} //namespace sce

#endif /* _PFX_JOINT_CONSTRAINT_FUNC_H_ */
