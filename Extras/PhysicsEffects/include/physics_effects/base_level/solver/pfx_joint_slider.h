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

#ifndef _SCE_PFX_JOINT_SLIDER_H
#define _SCE_PFX_JOINT_SLIDER_H

#include "pfx_joint.h"
#include "pfx_solver_body.h"

namespace sce {
namespace PhysicsEffects {

struct PfxSliderJointInitParam {
	PfxVector3 anchorPoint;
	PfxVector3 direction;
	PfxFloat lowerDistance;
	PfxFloat upperDistance;
	SCE_PFX_PADDING(1,8)
	
	PfxSliderJointInitParam()
	{
		anchorPoint = PfxVector3(0.0f);
		direction = PfxVector3(1.0f,0.0f,0.0f);
		lowerDistance = 0.0f;
		upperDistance = 0.0f;
	}
};

PfxInt32 pfxInitializeSliderJoint(
	PfxJoint &joint,
	const PfxRigidState &stateA,
	const PfxRigidState &stateB,
	const PfxSliderJointInitParam &param);

// pfxSetupSliderJoint = pfxSetupSwingTwistJoint

// pfxWarmStartSliderJoint = pfxWarmStartSwingTwistJoint

// pfxSolveSliderJoint = pfxSolveSwingTwistJoint

} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_JOINT_SLIDER_H
