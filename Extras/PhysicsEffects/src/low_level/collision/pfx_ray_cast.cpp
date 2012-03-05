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
#include "../../../include/physics_effects/low_level/collision/pfx_ray_cast.h"
#include "../../../include/physics_effects/base_level/collision/pfx_shape_iterator.h"
#include "pfx_intersect_ray_func.h"
#include "../../base_level/collision/pfx_intersect_common.h"


namespace sce {
namespace PhysicsEffects {


void pfxRayTraverseForward(
	const PfxRayInput &ray,PfxRayOutput &out,const PfxAabb16 &rayAABB,
	PfxBroadphaseProxy *proxies,int numProxies,
	PfxRigidState *offsetRigidStates,
	PfxCollidable *offsetCollidables,
	int axis,const PfxVector3 &center,const PfxVector3 &half)
{
#ifdef SCE_PFX_USE_GEOMETRY
	PfxGeomSegment segment((PfxPoint3)ray.m_startPosition,ray.m_direction);
#endif

	for(int i=0;i<numProxies;i++) {
		PfxBroadphaseProxy &proxy = proxies[i];

		// 終了条件のチェック
		if(pfxGetXYZMax(rayAABB,axis) < pfxGetXYZMin(proxy,axis)) {
			return;
		}

		PfxVector3 boundOnRay = ray.m_startPosition + out.m_variable * ray.m_direction;
		PfxVector3 AABBmin = pfxConvertCoordLocalToWorld(PfxVecInt3((PfxInt32)pfxGetXMin(proxy),(PfxInt32)pfxGetYMin(proxy),(PfxInt32)pfxGetZMin(proxy)),center,half);
		PfxVector3 AABBmax = pfxConvertCoordLocalToWorld(PfxVecInt3((PfxInt32)pfxGetXMax(proxy),(PfxInt32)pfxGetYMax(proxy),(PfxInt32)pfxGetZMax(proxy)),center,half);

		if(boundOnRay[axis] < AABBmin[axis]) {
			return;
		}

		// スキップ
		if(pfxGetXYZMax(proxy,axis) < pfxGetXYZMin(rayAABB,axis)) {
			continue;
		}

		PfxUInt16 rigidbodyId = pfxGetObjectId(proxy);
		PfxUInt32 contactFilterSelf = pfxGetSelf(proxy);
		PfxUInt32 contactFilterTarget = pfxGetTarget(proxy);

#ifdef SCE_PFX_USE_GEOMETRY
		PfxFloatInVec t_(1.0f);
		PfxGeomAabb aabb((PfxPoint3)AABBmin,(PfxPoint3)AABBmax);
		if( (ray.m_contactFilterSelf&contactFilterTarget) && (ray.m_contactFilterTarget&contactFilterSelf) && pfxTestAabb(rayAABB,proxy) &&
			intersectionPoint(segment,aabb,&t_) && t_ < out.m_variable ) {
#else
		float t_=1.0f;
		if( (ray.m_contactFilterSelf&contactFilterTarget) && (ray.m_contactFilterTarget&contactFilterSelf) && pfxTestAabb(rayAABB,proxy) &&
			pfxIntersectRayAABBFast(ray.m_startPosition,ray.m_direction,(AABBmax+AABBmin)*0.5f,(AABBmax-AABBmin)*0.5f,t_) && t_ < out.m_variable ) {
#endif

			PfxRigidState &state = offsetRigidStates[rigidbodyId];
			PfxCollidable &coll = offsetCollidables[rigidbodyId];
			PfxTransform3 transform(state.getOrientation(), state.getPosition());
			
			PfxRayOutput tout = out;
			
			PfxShapeIterator itrShape(coll);
			for(PfxUInt32 j=0;j<coll.getNumShapes();j++,++itrShape) {
				const PfxShape &shape = *itrShape;
				PfxTransform3 shapeTr = transform * shape.getOffsetTransform();
				
				if(pfxGetIntersectRayFunc(shape.getType())(ray,tout,shape,shapeTr) && tout.m_variable < out.m_variable) {
					out = tout;
					out.m_shapeId = j;
					out.m_objectId = rigidbodyId;
				}
			}
		}
	}
}

void pfxRayTraverseBackward(
	const PfxRayInput &ray,PfxRayOutput &out,const PfxAabb16 &rayAABB,
	PfxBroadphaseProxy *proxies,int numProxies,
	PfxRigidState *offsetRigidStates,
	PfxCollidable *offsetCollidables,
	int axis,const PfxVector3 &center,const PfxVector3 &half)
{
#ifdef SCE_PFX_USE_GEOMETRY
	PfxGeomSegment segment((PfxPoint3)ray.m_startPosition,ray.m_direction);
#endif

	for(int i=numProxies-1;i>=0;i--) {
		PfxBroadphaseProxy &proxy = proxies[i];

		// 終了条件のチェック
		if(pfxGetXYZMax(proxy,axis) < pfxGetXYZMin(rayAABB,axis)) {
			return;
		}
		
		PfxVector3 boundOnRay = ray.m_startPosition + out.m_variable * ray.m_direction;
		PfxVector3 AABBmin = pfxConvertCoordLocalToWorld(PfxVecInt3((PfxInt32)pfxGetXMin(proxy),(PfxInt32)pfxGetYMin(proxy),(PfxInt32)pfxGetZMin(proxy)),center,half);
		PfxVector3 AABBmax = pfxConvertCoordLocalToWorld(PfxVecInt3((PfxInt32)pfxGetXMax(proxy),(PfxInt32)pfxGetYMax(proxy),(PfxInt32)pfxGetZMax(proxy)),center,half);
		
		if(AABBmax[axis] < boundOnRay[axis]) {
			return;
		}
		
		// スキップ
		if(pfxGetXYZMax(rayAABB,axis) < pfxGetXYZMin(proxy,axis)) {
			continue;
		}
		
		PfxUInt16 rigidbodyId = pfxGetObjectId(proxy);
		PfxUInt32 contactFilterSelf = pfxGetSelf(proxy);
		PfxUInt32 contactFilterTarget = pfxGetTarget(proxy);
		
#ifdef SCE_PFX_USE_GEOMETRY
		PfxFloatInVec t_(1.0f);
		PfxGeomAabb aabb((PfxPoint3)AABBmin,(PfxPoint3)AABBmax);
		if( (ray.m_contactFilterSelf&contactFilterTarget) && (ray.m_contactFilterTarget&contactFilterSelf) && pfxTestAabb(rayAABB,proxy) &&
			intersectionPoint(segment,aabb,&t_) && t_ < out.m_variable ) {
#else
		float t_=1.0f;
		if( (ray.m_contactFilterSelf&contactFilterTarget) && (ray.m_contactFilterTarget&contactFilterSelf) && pfxTestAabb(rayAABB,proxy) &&
			pfxIntersectRayAABBFast(ray.m_startPosition,ray.m_direction,(AABBmax+AABBmin)*0.5f,(AABBmax-AABBmin)*0.5f,t_) && t_ < out.m_variable ) {
#endif

			PfxRigidState &state = offsetRigidStates[rigidbodyId];
			PfxCollidable &coll = offsetCollidables[rigidbodyId];
			PfxTransform3 transform(state.getOrientation(), state.getPosition());
			
			PfxRayOutput tout = out;
			
			PfxShapeIterator itrShape(coll);
			for(PfxUInt32 j=0;j<coll.getNumShapes();j++,++itrShape) {
				const PfxShape &shape = *itrShape;
				PfxTransform3 shapeTr = transform * shape.getOffsetTransform();
				
				if(pfxGetIntersectRayFunc(shape.getType())(ray,tout,shape,shapeTr) && tout.m_variable < out.m_variable) {
					out = tout;
					out.m_shapeId = j;
					out.m_objectId = rigidbodyId;
				}
			}
		}
	}
}


void pfxCastSingleRay(const PfxRayInput &ray,PfxRayOutput &out,const PfxRayCastParam &param)
{
	SCE_PFX_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesX));
	SCE_PFX_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesY));
	SCE_PFX_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesZ));
	SCE_PFX_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesXb));
	SCE_PFX_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesYb));
	SCE_PFX_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesZb));
	SCE_PFX_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.offsetRigidStates));
	SCE_PFX_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.offsetCollidables));

	PfxBroadphaseProxy *proxies[] = {
		param.proxiesX,
		param.proxiesY,
		param.proxiesZ,
		param.proxiesXb,
		param.proxiesYb,
		param.proxiesZb,
	};
	
	out.m_variable = 1.0f;
	out.m_contactFlag = false;
	
	// 探索軸
	PfxVector3 chkAxisVec = absPerElem(ray.m_direction);
	int axis = 0;
	if(chkAxisVec[1] < chkAxisVec[0]) axis = 1;
	if(chkAxisVec[2] < chkAxisVec[axis]) axis = 2;

	// レイのAABB作成
	PfxVector3 p1 = ray.m_startPosition;
	PfxVector3 p2 = ray.m_startPosition + ray.m_direction;
	PfxVecInt3 rayMin,rayMax;
	pfxConvertCoordWorldToLocal(param.rangeCenter,param.rangeExtent,minPerElem(p1,p2),maxPerElem(p1,p2),rayMin,rayMax);

	PfxAabb16 rayAABB;
	pfxSetXMin(rayAABB,rayMin.getX());
	pfxSetXMax(rayAABB,rayMax.getX());
	pfxSetYMin(rayAABB,rayMin.getY());
	pfxSetYMax(rayAABB,rayMax.getY());
	pfxSetZMin(rayAABB,rayMin.getZ());
	pfxSetZMax(rayAABB,rayMax.getZ());
	
	// AABB探索開始
	int sign = ray.m_direction[axis] < 0.0f ? -1 : 1; // 探索方向

	if(sign > 0) {
		pfxRayTraverseForward(
			ray,out,rayAABB,
			proxies[axis],param.numProxies,
			param.offsetRigidStates,param.offsetCollidables,
			axis,param.rangeCenter,param.rangeExtent);
	}
	else {
		pfxRayTraverseBackward(
			ray,out,rayAABB,
			proxies[axis+3],param.numProxies,
			param.offsetRigidStates,param.offsetCollidables,
			axis,param.rangeCenter,param.rangeExtent);
	}
}

} //namespace PhysicsEffects
} //namespace sce
