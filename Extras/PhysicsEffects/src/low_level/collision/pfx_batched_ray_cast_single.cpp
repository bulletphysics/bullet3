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

#include "../../../include/physics_effects/low_level/collision/pfx_batched_ray_cast.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// SINGLE THREAD

void pfxCastRaysStart(PfxRayInput *rayInputs,PfxRayOutput *rayOutputs,int numRays,PfxRayCastParam &param)
{
	for(int i=0;i<numRays;i++) {
		pfxCastSingleRay(rayInputs[i],rayOutputs[i],param);
	}
}

void pfxCastRays(PfxRayInput *rayInputs,PfxRayOutput *rayOutputs,int numRays,PfxRayCastParam &param)
{
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesX));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesY));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesZ));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesXb));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesYb));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesZb));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.offsetRigidStates));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.offsetCollidables));
	
	pfxCastRaysStart(rayInputs,rayOutputs,numRays,param);
}

} //namespace PhysicsEffects
} //namespace sce
