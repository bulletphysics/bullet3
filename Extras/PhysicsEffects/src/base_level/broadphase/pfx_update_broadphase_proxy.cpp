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
#include "../../../include/physics_effects/base_level/base/pfx_vec_utils.h"
#include "../../../include/physics_effects/base_level/collision/pfx_aabb.h"
#include "../../../include/physics_effects/base_level/broadphase/pfx_update_broadphase_proxy.h"

namespace sce {
namespace PhysicsEffects {

static SCE_PFX_FORCE_INLINE
PfxBool operator < (const PfxVector3 &v1,const PfxVector3 &v2)
{
	return maxElem(v2-v1) > 0.0f;
}

PfxInt32 pfxUpdateBroadphaseProxy(
	PfxBroadphaseProxy &proxy,
	const PfxRigidState &state,
	const PfxCollidable &coll,
	const PfxVector3 &worldCenter,
	const PfxVector3 &worldExtent,
	PfxUInt32 axis)
{
	SCE_PFX_ALWAYS_ASSERT(axis<3);
	
	PfxInt32 ret = SCE_PFX_OK;
	
	PfxVector3 center = state.getPosition() + coll.getCenter();
	PfxVector3 half = absPerElem(PfxMatrix3(state.getOrientation())) * coll.getHalf();
	
	PfxVector3 minRig = center - half;
	PfxVector3 maxRig = center + half;
	
	PfxVector3 minWld = worldCenter - worldExtent;
	PfxVector3 maxWld = worldCenter + worldExtent;
	
	if(maxWld < minRig || maxRig < minWld) {
		ret = SCE_PFX_ERR_OUT_OF_WORLD;
	}

	PfxVecInt3 aabbMin,aabbMax;
	pfxConvertCoordWorldToLocal(worldCenter,worldExtent,minRig,maxRig,aabbMin,aabbMax);
	
	pfxSetXMin(proxy,aabbMin.getX());
	pfxSetXMax(proxy,aabbMax.getX());
	pfxSetYMin(proxy,aabbMin.getY());
	pfxSetYMax(proxy,aabbMax.getY());
	pfxSetZMin(proxy,aabbMin.getZ());
	pfxSetZMax(proxy,aabbMax.getZ());
	pfxSetKey(proxy,aabbMin.get(axis));
	pfxSetObjectId(proxy,state.getRigidBodyId());
	pfxSetMotionMask(proxy,state.getMotionMask());
	pfxSetSelf(proxy,state.getContactFilterSelf());
	pfxSetTarget(proxy,state.getContactFilterTarget());
	
	return ret;
}

PfxInt32 pfxUpdateBroadphaseProxy(
	PfxBroadphaseProxy &proxy,
	const PfxRigidState &state,
	const PfxVector3 &objectCenter,
	const PfxVector3 &objectHalf,
	const PfxVector3 &worldCenter,
	const PfxVector3 &worldExtent,
	PfxUInt32 axis)
{
	SCE_PFX_ALWAYS_ASSERT(axis<3);
	
	PfxInt32 ret = SCE_PFX_OK;
	
	PfxVector3 minRig = objectCenter - objectHalf;
	PfxVector3 maxRig = objectCenter + objectHalf;
	
	PfxVector3 minWld = worldCenter - worldExtent;
	PfxVector3 maxWld = worldCenter + worldExtent;
	
	if(maxWld < minRig || maxRig < minWld) {
		ret = SCE_PFX_ERR_OUT_OF_WORLD;
	}
	
	PfxVecInt3 aabbMin,aabbMax;
	pfxConvertCoordWorldToLocal(worldCenter,worldExtent,minRig,maxRig,aabbMin,aabbMax);

	pfxSetXMin(proxy,aabbMin.getX());
	pfxSetXMax(proxy,aabbMax.getX());
	pfxSetYMin(proxy,aabbMin.getY());
	pfxSetYMax(proxy,aabbMax.getY());
	pfxSetZMin(proxy,aabbMin.getZ());
	pfxSetZMax(proxy,aabbMax.getZ());
	pfxSetKey(proxy,aabbMin.get(axis));
	pfxSetObjectId(proxy,state.getRigidBodyId());
	pfxSetMotionMask(proxy,state.getMotionMask());
	pfxSetSelf(proxy,state.getContactFilterSelf());
	pfxSetTarget(proxy,state.getContactFilterTarget());
	
	return ret;
}

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
	const PfxVector3 &worldExtent)
{
	PfxInt32 ret = SCE_PFX_OK;
	
	PfxVector3 center = state.getPosition() + coll.getCenter();
	PfxVector3 half = absPerElem(PfxMatrix3(state.getOrientation())) * coll.getHalf();
	
	PfxVector3 minRig = center - half;
	PfxVector3 maxRig = center + half;
	
	PfxVector3 minWld = worldCenter - worldExtent;
	PfxVector3 maxWld = worldCenter + worldExtent;
	
	if(maxWld < minRig || maxRig < minWld) {
		ret = SCE_PFX_ERR_OUT_OF_WORLD;
	}
	
	PfxVecInt3 aabbMin,aabbMax;
	pfxConvertCoordWorldToLocal(worldCenter,worldExtent,minRig,maxRig,aabbMin,aabbMax);

	PfxBroadphaseProxy proxy;
	
	pfxSetXMin(proxy,aabbMin.getX());
	pfxSetXMax(proxy,aabbMax.getX());
	pfxSetYMin(proxy,aabbMin.getY());
	pfxSetYMax(proxy,aabbMax.getY());
	pfxSetZMin(proxy,aabbMin.getZ());
	pfxSetZMax(proxy,aabbMax.getZ());
	pfxSetObjectId(proxy,state.getRigidBodyId());
	pfxSetMotionMask(proxy,state.getMotionMask());
	pfxSetSelf(proxy,state.getContactFilterSelf());
	pfxSetTarget(proxy,state.getContactFilterTarget());
	
	proxyX  = proxy;
	proxyXb = proxy;
	proxyY  = proxy;
	proxyYb = proxy;
	proxyZ  = proxy;
	proxyZb = proxy;
	
	pfxSetKey(proxyX ,aabbMin.getX());
	pfxSetKey(proxyXb,aabbMax.getX());
	pfxSetKey(proxyY ,aabbMin.getY());
	pfxSetKey(proxyYb,aabbMax.getY());
	pfxSetKey(proxyZ ,aabbMin.getZ());
	pfxSetKey(proxyZb,aabbMax.getZ());
	
	return ret;
}

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
	const PfxVector3 &worldExtent)
{
	PfxInt32 ret = SCE_PFX_OK;
	
	PfxVector3 minRig = objectCenter - objectHalf;
	PfxVector3 maxRig = objectCenter + objectHalf;
	
	PfxVector3 minWld = worldCenter - worldExtent;
	PfxVector3 maxWld = worldCenter + worldExtent;
	
	if(maxWld < minRig || maxRig < minWld) {
		ret = SCE_PFX_ERR_OUT_OF_WORLD;
	}
	
	PfxVecInt3 aabbMin,aabbMax;
	pfxConvertCoordWorldToLocal(worldCenter,worldExtent,minRig,maxRig,aabbMin,aabbMax);

	PfxBroadphaseProxy proxy;
	
	pfxSetXMin(proxy,aabbMin.getX());
	pfxSetXMax(proxy,aabbMax.getX());
	pfxSetYMin(proxy,aabbMin.getY());
	pfxSetYMax(proxy,aabbMax.getY());
	pfxSetZMin(proxy,aabbMin.getZ());
	pfxSetZMax(proxy,aabbMax.getZ());
	pfxSetObjectId(proxy,state.getRigidBodyId());
	pfxSetMotionMask(proxy,state.getMotionMask());
	pfxSetSelf(proxy,state.getContactFilterSelf());
	pfxSetTarget(proxy,state.getContactFilterTarget());
	
	proxyX  = proxy;
	proxyXb = proxy;
	proxyY  = proxy;
	proxyYb = proxy;
	proxyZ  = proxy;
	proxyZb = proxy;
	
	pfxSetKey(proxyX ,aabbMin.getX());
	pfxSetKey(proxyXb,aabbMax.getX());
	pfxSetKey(proxyY ,aabbMin.getY());
	pfxSetKey(proxyYb,aabbMax.getY());
	pfxSetKey(proxyZ ,aabbMin.getZ());
	pfxSetKey(proxyZb,aabbMax.getZ());
	
	return ret;
}

} // namespace PhysicsEffects
} // namespace sce
