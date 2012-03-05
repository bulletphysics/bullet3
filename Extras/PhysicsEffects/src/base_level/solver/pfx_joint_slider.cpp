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

#include "../../../include/physics_effects/base_level/base/pfx_vec_utils.h"
#include "../../../include/physics_effects/base_level/solver/pfx_joint_slider.h"
#include "pfx_constraint_row_solver.h"

namespace sce {
namespace PhysicsEffects {

PfxInt32 pfxInitializeSliderJoint(PfxJoint &joint,
	const PfxRigidState &stateA,const PfxRigidState &stateB,
	const PfxSliderJointInitParam &param)
{
	joint.m_active = 1;
	joint.m_numConstraints = 6;
	joint.m_userData = NULL;
	joint.m_type = kPfxJointSlider;
	joint.m_rigidBodyIdA = stateA.getRigidBodyId();
	joint.m_rigidBodyIdB = stateB.getRigidBodyId();
	
	for(int i=0;i<6;i++) {
		joint.m_constraints[i].reset();
	}

	if(param.lowerDistance == 0.0f && param.upperDistance == 0.0f) {
		joint.m_constraints[0].m_lock = SCE_PFX_JOINT_LOCK_FREE;
	}
	else {
		joint.m_constraints[0].m_lock = SCE_PFX_JOINT_LOCK_LIMIT;
	}
	joint.m_constraints[1].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	joint.m_constraints[2].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	joint.m_constraints[3].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	joint.m_constraints[4].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	joint.m_constraints[5].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	
	if(param.lowerDistance > param.upperDistance ) {
		return SCE_PFX_ERR_OUT_OF_RANGE;
	}
	
	joint.m_constraints[0].m_movableLowerLimit = param.lowerDistance;
	joint.m_constraints[0].m_movableUpperLimit = param.upperDistance;
	
	// Calc joint frame
	PfxMatrix3 rotA = transpose(PfxMatrix3(stateA.getOrientation()));
	PfxMatrix3 rotB = transpose(PfxMatrix3(stateB.getOrientation()));
	
	PfxVector3 axisInA = rotA * normalize(param.direction);
	
	joint.m_anchorA = rotA * (param.anchorPoint - stateA.getPosition());
	joint.m_anchorB = rotB * (param.anchorPoint - stateB.getPosition());
	
	PfxVector3 axis1, axis2;
	
	pfxGetPlaneSpace(axisInA, axis1, axis2 );
	
	joint.m_frameA = PfxMatrix3(axisInA, axis1, axis2);
	joint.m_frameB = rotB * PfxMatrix3(stateA.getOrientation()) * joint.m_frameA;
	
	return SCE_PFX_OK;
}

} //namespace PhysicsEffects
} //namespace sce
