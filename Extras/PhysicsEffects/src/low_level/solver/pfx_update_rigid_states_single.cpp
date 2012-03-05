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

#include "../../../include/physics_effects/base_level/base/pfx_perf_counter.h"
#include "../../../include/physics_effects/base_level/solver/pfx_integrate.h"
#include "../../../include/physics_effects/low_level/solver/pfx_update_rigid_states.h"

namespace sce {
namespace PhysicsEffects {

PfxInt32 pfxCheckParamOfUpdateRigidStates(const PfxUpdateRigidStatesParam &param)
{
	if(!param.states || !param.bodies || param.timeStep <= 0.0f) return SCE_PFX_ERR_INVALID_VALUE;
	if(!SCE_PFX_PTR_IS_ALIGNED16(param.states) || !SCE_PFX_PTR_IS_ALIGNED16(param.bodies)) return SCE_PFX_ERR_INVALID_ALIGN;
	return SCE_PFX_OK;
}

///////////////////////////////////////////////////////////////////////////////
// SINGLE THREAD

PfxInt32 pfxUpdateRigidStates(PfxUpdateRigidStatesParam &param)
{
	PfxInt32 ret = pfxCheckParamOfUpdateRigidStates(param);
	if(ret != SCE_PFX_OK) return ret;

	SCE_PFX_PUSH_MARKER("pfxUpdateRigidStates");

	for(PfxUInt32 i=0;i<param.numRigidBodies;i++) {
		pfxIntegrate(param.states[i],param.bodies[i],param.timeStep);
	}

	SCE_PFX_POP_MARKER();

	return SCE_PFX_OK;
}

} //namespace PhysicsEffects
} //namespace sce
