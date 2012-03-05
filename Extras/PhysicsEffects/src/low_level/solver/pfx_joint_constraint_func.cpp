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

#include "../../../include/physics_effects/base_level/solver/pfx_joint.h"
#include "../../../include/physics_effects/low_level/solver/pfx_joint_constraint_func.h"

#include "../../../include/physics_effects/base_level/solver/pfx_joint_ball.h"
#include "../../../include/physics_effects/base_level/solver/pfx_joint_swing_twist.h"
#include "../../../include/physics_effects/base_level/solver/pfx_joint_hinge.h"
#include "../../../include/physics_effects/base_level/solver/pfx_joint_slider.h"
#include "../../../include/physics_effects/base_level/solver/pfx_joint_fix.h"
#include "../../../include/physics_effects/base_level/solver/pfx_joint_universal.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// Setup Joint Constraint Function Table

void setupJointConstraintDummy(
	PfxJoint &joint,
	const PfxRigidState &stateA,
	const PfxRigidState &stateB,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB,
	PfxFloat timeStep)
{
	(void)joint,(void)stateA,(void)stateB,(void)solverBodyA,(void)solverBodyB,(void)timeStep;
}

PfxSetupJointConstraintFunc funcTbl_setupJointConstraint[kPfxJointCount] = {
	pfxSetupBallJoint,
	pfxSetupSwingTwistJoint,
	pfxSetupSwingTwistJoint,
	pfxSetupSwingTwistJoint,
	pfxSetupSwingTwistJoint,
	pfxSetupUniversalJoint,
	setupJointConstraintDummy,
	setupJointConstraintDummy,
	setupJointConstraintDummy,
	setupJointConstraintDummy,
	setupJointConstraintDummy,
	setupJointConstraintDummy,
	setupJointConstraintDummy,
	setupJointConstraintDummy,
	setupJointConstraintDummy,
};

///////////////////////////////////////////////////////////////////////////////
// Setup Joint Constraint Function Table Interface

PfxSetupJointConstraintFunc pfxGetSetupJointConstraintFunc(PfxUInt8 jointType)
{
	SCE_PFX_ASSERT(jointType<kPfxJointCount);
	return funcTbl_setupJointConstraint[jointType];
}

int pfxSetSetupJointConstraintFunc(PfxUInt8 jointType,PfxSetupJointConstraintFunc func)
{
	if(jointType >= kPfxJointCount) {
		return SCE_PFX_ERR_OUT_OF_RANGE;
	}
	
	funcTbl_setupJointConstraint[jointType] = func;
	
	return SCE_PFX_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Warm Start Joint Constraint Function Table

void warmStartJointConstraintDummy(
	PfxJoint &joint,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB)
{
	(void)joint,(void)solverBodyA,(void)solverBodyB;
}

PfxWarmStartJointConstraintFunc funcTbl_warmStartJointConstraint[kPfxJointCount] = {
	pfxWarmStartBallJoint,
	pfxWarmStartSwingTwistJoint,
	pfxWarmStartSwingTwistJoint,
	pfxWarmStartSwingTwistJoint,
	pfxWarmStartSwingTwistJoint,
	pfxWarmStartSwingTwistJoint,
	warmStartJointConstraintDummy,
	warmStartJointConstraintDummy,
	warmStartJointConstraintDummy,
	warmStartJointConstraintDummy,
	warmStartJointConstraintDummy,
	warmStartJointConstraintDummy,
	warmStartJointConstraintDummy,
	warmStartJointConstraintDummy,
	warmStartJointConstraintDummy,
};

///////////////////////////////////////////////////////////////////////////////
// Warm Start Joint Constraint Function Table Interface

PfxWarmStartJointConstraintFunc pfxGetWarmStartJointConstraintFunc(PfxUInt8 jointType)
{
	SCE_PFX_ASSERT(jointType<kPfxJointCount);
	return funcTbl_warmStartJointConstraint[jointType];
}

int pfxSetWarmStartJointConstraintFunc(PfxUInt8 jointType,PfxWarmStartJointConstraintFunc func)
{
	if(jointType >= kPfxJointCount) {
		return SCE_PFX_ERR_OUT_OF_RANGE;
	}
	
	funcTbl_warmStartJointConstraint[jointType] = func;
	
	return SCE_PFX_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Solve Joint Constraint Function Table

void solveJointConstraintDummy(
	PfxJoint &joint,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB)
{
	(void)joint,(void)solverBodyA,(void)solverBodyB;
}

PfxSolveJointConstraintFunc funcTbl_solveJointConstraint[kPfxJointCount] = {
	pfxSolveBallJoint,
	pfxSolveSwingTwistJoint,
	pfxSolveSwingTwistJoint,
	pfxSolveSwingTwistJoint,
	pfxSolveSwingTwistJoint,
	pfxSolveSwingTwistJoint,
	solveJointConstraintDummy,
	solveJointConstraintDummy,
	solveJointConstraintDummy,
	solveJointConstraintDummy,
	solveJointConstraintDummy,
	solveJointConstraintDummy,
	solveJointConstraintDummy,
	solveJointConstraintDummy,
	solveJointConstraintDummy,
};

///////////////////////////////////////////////////////////////////////////////
// Solve Joint Constraint Function Table Interface

PfxSolveJointConstraintFunc pfxGetSolveJointConstraintFunc(PfxUInt8 jointType)
{
	SCE_PFX_ASSERT(jointType<kPfxJointCount);
	return funcTbl_solveJointConstraint[jointType];
}

int pfxSetSolveJointConstraintFunc(PfxUInt8 jointType,PfxSolveJointConstraintFunc func)
{
	if(jointType >= kPfxJointCount) {
		return SCE_PFX_ERR_OUT_OF_RANGE;
	}
	
	funcTbl_solveJointConstraint[jointType] = func;
	
	return SCE_PFX_OK;
}
} //namespace PhysicsEffects
} //namespace sce
