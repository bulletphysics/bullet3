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

#ifndef _SCE_PFX_JOINT_UNIVERSAL_H
#define _SCE_PFX_JOINT_UNIVERSAL_H

#include "pfx_joint.h"
#include "pfx_solver_body.h"

namespace sce {
namespace PhysicsEffects {

struct PfxUniversalJointInitParam {
	PfxVector3 anchorPoint;
	PfxVector3 axis;
	PfxFloat swing1LowerAngle;
	PfxFloat swing1UpperAngle;
	PfxFloat swing2LowerAngle;
	PfxFloat swing2UpperAngle;
	
	PfxUniversalJointInitParam()
	{
		anchorPoint = PfxVector3(0.0f);
		axis = PfxVector3(1.0f,0.0f,0.0f);
		swing1LowerAngle = -0.7f;
		swing1UpperAngle =  0.7f;
		swing2LowerAngle = -0.7f;
		swing2UpperAngle =  0.7f;
	}
};

PfxInt32 pfxInitializeUniversalJoint(
	PfxJoint &joint,
	const PfxRigidState &stateA,
	const PfxRigidState &stateB,
	const PfxUniversalJointInitParam &param);

void pfxSetupUniversalJoint(
	PfxJoint &joint,
	const PfxRigidState &stateA,
	const PfxRigidState &stateB,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB,
	PfxFloat timeStep);

// pfxWarmStartUniversalJoint = pfxWarmStartSwingTwistJoint

// pfxSolveUniversalJoint = pfxSolveSwingTwistJoint

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_JOINT_UNIVERSAL_H
