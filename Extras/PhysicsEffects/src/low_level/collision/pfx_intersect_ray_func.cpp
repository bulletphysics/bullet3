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

#include "../../../include/physics_effects/base_level/collision/pfx_shape.h"
#include "../../base_level/collision/pfx_intersect_ray_box.h"
#include "../../base_level/collision/pfx_intersect_ray_sphere.h"
#include "../../base_level/collision/pfx_intersect_ray_capsule.h"
#include "../../base_level/collision/pfx_intersect_ray_cylinder.h"
#include "../../base_level/collision/pfx_intersect_ray_convex.h"
#include "../../base_level/collision/pfx_intersect_ray_large_tri_mesh.h"
#include "pfx_intersect_ray_func.h"


namespace sce {
namespace PhysicsEffects {


///////////////////////////////////////////////////////////////////////////////
// Ray Intersection Function Table

PfxBool intersectRayFuncDummy(
				const PfxRayInput &ray,PfxRayOutput &out,
				const PfxShape &shape,const PfxTransform3 &transform)
{
	(void)ray,(void)out,(void)shape,(void)transform;
	return false;
}

PfxBool intersectRayFuncBox(
				const PfxRayInput &ray,PfxRayOutput &out,
				const PfxShape &shape,const PfxTransform3 &transform)
{
	return pfxIntersectRayBox(ray,out,shape.getBox(),transform);
}

PfxBool intersectRayFuncSphere(
				const PfxRayInput &ray,PfxRayOutput &out,
				const PfxShape &shape,const PfxTransform3 &transform)
{
	return pfxIntersectRaySphere(ray,out,shape.getSphere(),transform);
}

PfxBool intersectRayFuncCapsule(
				const PfxRayInput &ray,PfxRayOutput &out,
				const PfxShape &shape,const PfxTransform3 &transform)
{
	return pfxIntersectRayCapsule(ray,out,shape.getCapsule(),transform);
}

PfxBool intersectRayFuncCylinder(
				const PfxRayInput &ray,PfxRayOutput &out,
				const PfxShape &shape,const PfxTransform3 &transform)
{
	return pfxIntersectRayCylinder(ray,out,shape.getCylinder(),transform);
}

PfxBool intersectRayFuncConvex(
				const PfxRayInput &ray,PfxRayOutput &out,
				const PfxShape &shape,const PfxTransform3 &transform)
{
const PfxConvexMesh *convex = shape.getConvexMesh();
	
	PfxBool ret = pfxIntersectRayConvex(ray,out,(const void*)convex,transform);
	
	
	return ret;
}

PfxBool intersectRayFuncLargeTriMesh(
				const PfxRayInput &ray,PfxRayOutput &out,
				const PfxShape &shape,const PfxTransform3 &transform)
{
const PfxLargeTriMesh *lmesh = shape.getLargeTriMesh();
	
	PfxBool ret = pfxIntersectRayLargeTriMesh(ray,out,(const void*)lmesh,transform);
	
	
	return ret;
}

PfxIntersectRayFunc funcTbl_intersectRay[kPfxShapeCount] = {
	intersectRayFuncSphere,
	intersectRayFuncBox,
	intersectRayFuncCapsule,
	intersectRayFuncCylinder,
	intersectRayFuncConvex,
	intersectRayFuncLargeTriMesh,
	intersectRayFuncDummy,
	intersectRayFuncDummy,
	intersectRayFuncDummy,
	intersectRayFuncDummy,
	intersectRayFuncDummy,
	intersectRayFuncDummy,
};

///////////////////////////////////////////////////////////////////////////////
// Ray Intersection Function Table Interface

PfxIntersectRayFunc pfxGetIntersectRayFunc(PfxUInt8 shapeType)
{
	return funcTbl_intersectRay[shapeType];
}

PfxInt32 pfxSetIntersectRayFunc(PfxUInt8 shapeType,PfxIntersectRayFunc func)
{
	if(shapeType >= kPfxShapeCount) {
		return SCE_PFX_ERR_OUT_OF_RANGE;
	}
	
	funcTbl_intersectRay[shapeType] = func;
	
	return SCE_PFX_OK;
}

} //namespace PhysicsEffects
} //namespace sce
