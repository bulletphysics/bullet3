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

#ifndef _SCE_PFX_RAY_CAST_H
#define _SCE_PFX_RAY_CAST_H

#include "../../base_level/rigidbody/pfx_rigid_state.h"
#include "../../base_level/collision/pfx_collidable.h"
#include "../../base_level/collision/pfx_ray.h"
#include "../../base_level/broadphase/pfx_broadphase_proxy.h"

///////////////////////////////////////////////////////////////////////////////
// RayCast

namespace sce {
namespace PhysicsEffects {

struct PfxRayCastParam {
	PfxRigidState *offsetRigidStates;
	PfxCollidable *offsetCollidables;
	PfxBroadphaseProxy *proxiesX;
	PfxBroadphaseProxy *proxiesY;
	PfxBroadphaseProxy *proxiesZ;
	PfxBroadphaseProxy *proxiesXb;
	PfxBroadphaseProxy *proxiesYb;
	PfxBroadphaseProxy *proxiesZb;
	PfxUInt32 numProxies;
	SCE_PFX_PADDING(1,12)
	PfxVector3 rangeCenter;
	PfxVector3 rangeExtent;
};

void pfxCastSingleRay(const PfxRayInput &rayInput,PfxRayOutput &rayOutput,const PfxRayCastParam &param);

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_RAY_CAST_H
