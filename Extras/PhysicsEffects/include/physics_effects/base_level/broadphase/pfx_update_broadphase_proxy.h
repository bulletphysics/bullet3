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

#ifndef _SCE_PFX_UPDATE_BROADPHASE_PROXY_H
#define _SCE_PFX_UPDATE_BROADPHASE_PROXY_H

#include "pfx_broadphase_pair.h"
#include "pfx_broadphase_proxy.h"
#include "../rigidbody/pfx_rigid_state.h"
#include "../collision/pfx_collidable.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// Update Broadphase Proxy

//E For single axis
//J 単一軸に対して作成
PfxInt32 pfxUpdateBroadphaseProxy(
	PfxBroadphaseProxy &proxy,
	const PfxRigidState &state,
	const PfxCollidable &coll,
	const PfxVector3 &worldCenter,
	const PfxVector3 &worldExtent,
	PfxUInt32 axis);

PfxInt32 pfxUpdateBroadphaseProxy(
	PfxBroadphaseProxy &proxy,
	const PfxRigidState &state,
	const PfxVector3 &objectCenter,
	const PfxVector3 &objectHalf,
	const PfxVector3 &worldCenter,
	const PfxVector3 &worldExtent,
	PfxUInt32 axis);

//E For all axes
//J 全軸に対して作成
PfxInt32 pfxUpdateBroadphaseProxy(
	PfxBroadphaseProxy &proxyX,
	PfxBroadphaseProxy &proxyY,
	PfxBroadphaseProxy &proxyZ,
	PfxBroadphaseProxy &proxyXb,
	PfxBroadphaseProxy &proxyYb,
	PfxBroadphaseProxy &proxyZb,
	const PfxRigidState &state,
	const PfxCollidable &coll,
	const PfxVector3 &worldCenter,
	const PfxVector3 &worldExtent);

PfxInt32 pfxUpdateBroadphaseProxy(
	PfxBroadphaseProxy &proxyX,
	PfxBroadphaseProxy &proxyY,
	PfxBroadphaseProxy &proxyZ,
	PfxBroadphaseProxy &proxyXb,
	PfxBroadphaseProxy &proxyYb,
	PfxBroadphaseProxy &proxyZb,
	const PfxRigidState &state,
	const PfxVector3 &objectCenter,
	const PfxVector3 &objectHalf,
	const PfxVector3 &worldCenter,
	const PfxVector3 &worldExtent);

} // namespace PhysicsEffects
} // namespace sce

#endif // _SCE_PFX_UPDATE_BROADPHASE_PROXY_H
