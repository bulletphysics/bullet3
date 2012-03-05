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

#ifndef _SCE_PFX_JOINT_SWING_TWIST_H
#define _SCE_PFX_JOINT_SWING_TWIST_H

#include "pfx_joint.h"
#include "pfx_solver_body.h"
namespace sce {
namespace PhysicsEffects {

struct PfxSwingTwistJointInitParam {
	PfxVector3 anchorPoint;
	PfxVector3 twistAxis;
	PfxFloat twistLowerAngle;
	PfxFloat twistUpperAngle;
	PfxFloat swingLowerAngle;
	PfxFloat swingUpperAngle;
	
	PfxSwingTwistJointInitParam()
	{
		anchorPoint = PfxVector3(0.0f);
		twistAxis = PfxVector3(1.0f,0.0f,0.0f);
		twistLowerAngle = -0.26f;
		twistUpperAngle =  0.26f;
		swingLowerAngle = 0.0f;
		swingUpperAngle = 0.7f;
	}
};

PfxInt32 pfxInitializeSwingTwistJoint(
	PfxJoint &joint,
	const PfxRigidState &stateA,
	const PfxRigidState &stateB,
	const PfxSwingTwistJointInitParam &param);

void pfxSetupSwingTwistJoint(
	PfxJoint &joint,
	const PfxRigidState &stateA,
	const PfxRigidState &stateB,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB,
	PfxFloat timeStep);

void pfxWarmStartSwingTwistJoint(
	PfxJoint &joint,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB);

void pfxSolveSwingTwistJoint(
	PfxJoint &joint,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB);

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_JOINT_SWING_TWIST_H
