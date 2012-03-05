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

#ifndef _SCE_PFX_JOINT_CONSTRAINT_H
#define _SCE_PFX_JOINT_CONSTRAINT_H

#include "../base/pfx_common.h"
#include "../base/pfx_vec_utils.h"
#include "pfx_constraint_row.h"

namespace sce {
namespace PhysicsEffects {

// Lock type
#define SCE_PFX_JOINT_LOCK_FREE		0
#define SCE_PFX_JOINT_LOCK_LIMIT	1
#define SCE_PFX_JOINT_LOCK_FIX		2

// Slop
#define SCE_PFX_JOINT_LINEAR_SLOP	0.01f
#define SCE_PFX_JOINT_ANGULAR_SLOP	0.01f

///////////////////////////////////////////////////////////////////////////////// Joint Constraint

struct PfxJointConstraint {
	PfxInt8 m_lock;
	PfxInt8 m_warmStarting;
	SCE_PFX_PADDING(1,2)
	PfxFloat m_movableLowerLimit;
	PfxFloat m_movableUpperLimit;
	PfxFloat m_bias;
	PfxFloat m_weight;
	PfxFloat m_damping;
	PfxFloat m_maxImpulse;
	SCE_PFX_PADDING(2,4)
	PfxConstraintRow m_constraintRow;
	
	void reset()
	{
		m_lock = SCE_PFX_JOINT_LOCK_FIX;
		m_warmStarting = 0;
		m_movableLowerLimit = 0.0f;
		m_movableUpperLimit = 0.0f;
		m_bias = 0.2f;
		m_weight = 1.0f;
		m_damping = 0.0f;
		m_maxImpulse = SCE_PFX_FLT_MAX;
		memset(&m_constraintRow,0,sizeof(PfxConstraintRow));
	}
};
} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_JOINT_CONSTRAINT_H
