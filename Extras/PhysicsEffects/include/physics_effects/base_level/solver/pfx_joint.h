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

#ifndef _SCE_PFX_JOINT_H
#define _SCE_PFX_JOINT_H

#include "../rigidbody/pfx_rigid_state.h"
#include "pfx_joint_constraint.h"
#include "pfx_constraint_pair.h"

namespace sce {
namespace PhysicsEffects {

// Joint Type
enum ePfxJointType {
	kPfxJointBall = 0,
	kPfxJointSwingtwist,
	kPfxJointHinge,
	kPfxJointSlider,
	kPfxJointFix,
	kPfxJointUniversal,
	kPfxJointAnimation,
	kPfxJointReserved0,
	kPfxJointReserved1,
	kPfxJointReserved2,
	kPfxJointUser0,
	kPfxJointUser1,
	kPfxJointUser2,
	kPfxJointUser3,
	kPfxJointUser4,
	kPfxJointCount // = 15
};

// Joint Structure
struct SCE_PFX_ALIGNED(128) PfxJoint {
	PfxUInt8 m_active;
	PfxUInt8 m_numConstraints;
	PfxUInt8 m_type;
	SCE_PFX_PADDING(1,1)
	PfxUInt16 m_rigidBodyIdA;
	PfxUInt16 m_rigidBodyIdB;
	SCE_PFX_PADDING(2,8)
	PfxJointConstraint m_constraints[6];
	void *m_userData;
	SCE_PFX_PADDING(3,12)
	PfxVector3 m_param[4]; // Used by some joints for specific features
	PfxVector3 m_anchorA;
	PfxVector3 m_anchorB;
	PfxMatrix3 m_frameA;
	PfxMatrix3 m_frameB;
	SCE_PFX_PADDING(4,32)
};

SCE_PFX_FORCE_INLINE 
void pfxUpdateJointPairs(
	PfxConstraintPair &pair,
	PfxUInt32 jointId,
	const PfxJoint &joint,
	const PfxRigidState &stateA,
	const PfxRigidState &stateB
	)
{
	SCE_PFX_ALWAYS_ASSERT(stateA.getRigidBodyId()==joint.m_rigidBodyIdA);
	SCE_PFX_ALWAYS_ASSERT(stateB.getRigidBodyId()==joint.m_rigidBodyIdB);
	pfxSetObjectIdA(pair,stateA.getRigidBodyId());
	pfxSetObjectIdB(pair,stateB.getRigidBodyId());
	pfxSetMotionMaskA(pair,stateA.getMotionMask());
	pfxSetMotionMaskB(pair,stateB.getMotionMask());
	pfxSetConstraintId(pair,jointId);
	pfxSetNumConstraints(pair,joint.m_numConstraints);
	pfxSetActive(pair,joint.m_active>0);
}
} //namespace PhysicsEffects
} //namespace sce


#endif // _PFX_JOINT_H
