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
#include "../../../include/physics_effects/base_level/solver/pfx_joint_fix.h"
#include "pfx_constraint_row_solver.h"

namespace sce {
namespace PhysicsEffects {

PfxInt32 pfxInitializeFixJoint(PfxJoint &joint,
	const PfxRigidState &stateA,const PfxRigidState &stateB,
	const PfxFixJointInitParam &param)
{
	joint.m_active = 1;
	joint.m_numConstraints = 6;
	joint.m_userData = NULL;
	joint.m_type = kPfxJointFix;
	joint.m_rigidBodyIdA = stateA.getRigidBodyId();
	joint.m_rigidBodyIdB = stateB.getRigidBodyId();
	
	for(int i=0;i<6;i++) {
		joint.m_constraints[i].reset();
		joint.m_constraints[i].m_lock = SCE_PFX_JOINT_LOCK_FIX;
	}
	
	// Calc joint frame
	PfxMatrix3 rotA = transpose(PfxMatrix3(stateA.getOrientation()));
	PfxMatrix3 rotB = transpose(PfxMatrix3(stateB.getOrientation()));
	
	joint.m_anchorA = rotA * (param.anchorPoint - stateA.getPosition());
	joint.m_anchorB = rotB * (param.anchorPoint - stateB.getPosition());
	
	joint.m_frameA = PfxMatrix3::identity();
	joint.m_frameB = rotB * PfxMatrix3(stateA.getOrientation()) * joint.m_frameA;
	
	return SCE_PFX_OK;
}

} //namespace PhysicsEffects
} //namespace sce
