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

#ifndef _SCE_PFX_SOLVER_BODY_H
#define _SCE_PFX_SOLVER_BODY_H

#include "../base/pfx_common.h"

namespace sce {
namespace PhysicsEffects {

struct SCE_PFX_ALIGNED(128) PfxSolverBody {
	PfxVector3 m_deltaLinearVelocity;
	PfxVector3 m_deltaAngularVelocity;
	PfxQuat    m_orientation;
	PfxMatrix3 m_inertiaInv;
	PfxFloat   m_massInv;
	PfxUInt32  m_motionType;
	SCE_PFX_PADDING(1,24)
};

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_SOLVER_BODY_H
