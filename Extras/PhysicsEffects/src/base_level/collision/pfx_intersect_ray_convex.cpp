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

#include "../../../include/physics_effects/base_level/collision/pfx_tri_mesh.h"
#include "pfx_intersect_common.h"
#include "pfx_intersect_ray_convex.h"

namespace sce {
namespace PhysicsEffects {

PfxBool pfxIntersectRayConvex(const PfxRayInput &ray,PfxRayOutput &out,const void *shape,const PfxTransform3 &transform)
{
	const PfxConvexMesh *convex = (const PfxConvexMesh*)shape;
	
	// レイをConvexのローカル座標へ変換
	PfxTransform3 transformConvex = orthoInverse(transform);
	PfxVector3 startPosL = transformConvex.getUpper3x3() * ray.m_startPosition + transformConvex.getTranslation();
	PfxVector3 rayDirL = transformConvex.getUpper3x3() * ray.m_direction;

	// レイとConvexの交差判定
	PfxFloat tmpVariable(0.0f);
	PfxVector3 tmpNormal;
	bool ret = false;
	for(PfxUInt32 f=0;f<(PfxUInt32)convex->m_numIndices/3;f++) {
		PfxTriangle triangle(
			convex->m_verts[convex->m_indices[f*3  ]],
			convex->m_verts[convex->m_indices[f*3+1]],
			convex->m_verts[convex->m_indices[f*3+2]]);

		if(pfxIntersectRayTriangleWithoutBackFace(startPosL,rayDirL,triangle,tmpVariable) && tmpVariable < out.m_variable) {
			out.m_contactFlag = true;
			out.m_variable = tmpVariable;
			out.m_contactPoint = ray.m_startPosition + tmpVariable * ray.m_direction;
			out.m_contactNormal = transform.getUpper3x3() * normalize(cross(triangle.points[2]-triangle.points[1],triangle.points[0]-triangle.points[1]));
			out.m_subData.m_type = PfxSubData::NONE;
			ret = true;
		}
	}

	return ret;
}
} //namespace PhysicsEffects
} //namespace sce
