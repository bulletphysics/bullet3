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

#ifndef _SCE_PFX_JOINT_HINGE_H_
#define _SCE_PFX_JOINT_HINGE_H_

#include "pfx_joint.h"
#include "pfx_solver_body.h"

namespace sce {
namespace PhysicsEffects {

struct PfxHingeJointInitParam {
	PfxVector3 anchorPoint;
	PfxVector3 axis;
	PfxFloat lowerAngle;
	PfxFloat upperAngle;
	SCE_PFX_PADDING(1,8)
	
	PfxHingeJointInitParam()
	{
		anchorPoint = PfxVector3(0.0f);
		axis = PfxVector3(1.0f,0.0f,0.0f);
		lowerAngle = 0.0f;
		upperAngle = 0.0f;
	}
};

PfxInt32 pfxInitializeHingeJoint(
	PfxJoint &joint,
	const PfxRigidState &stateA,
	const PfxRigidState &stateB,
	const PfxHingeJointInitParam &param);

// pfxSetupHingeJoint = pfxSetupSwingTwistJoint

// pfxWarmStartHingeJoint = pfxWarmStartSwingTwistJoint

// pfxSolveHingeJoint = pfxSolveSwingTwistJoint

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_JOINT_HINGE_H
