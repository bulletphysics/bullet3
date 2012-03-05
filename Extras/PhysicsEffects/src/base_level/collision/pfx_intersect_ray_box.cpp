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

#include "pfx_intersect_common.h"
#include "pfx_intersect_ray_box.h"

namespace sce {
namespace PhysicsEffects {

PfxBool pfxIntersectRayBox(const PfxRayInput &ray,PfxRayOutput &out,const PfxBox &box,const PfxTransform3 &transform)
{
	// レイをBoxのローカル座標へ変換
	PfxTransform3 transformBox = orthoInverse(transform);
	PfxVector3 rayStartPosition = transformBox.getUpper3x3() * ray.m_startPosition + transformBox.getTranslation();
	PfxVector3 rayDirection = transformBox.getUpper3x3() * ray.m_direction;
	
	// 交差判定
	PfxFloat tmpVariable=0.0f;
	PfxVector3 tmpNormal(0.0f);
	if(pfxIntersectRayAABB(rayStartPosition,rayDirection,PfxVector3(0.0f),box.m_half,tmpVariable,tmpNormal)) {
		if(tmpVariable > 0.0f && tmpVariable < out.m_variable) {
			out.m_contactFlag = true;
			out.m_variable = tmpVariable;
			out.m_contactPoint = ray.m_startPosition + tmpVariable * ray.m_direction;
			out.m_contactNormal = transform.getUpper3x3() * tmpNormal;
			out.m_subData.m_type = PfxSubData::NONE;
			return true;
		}
	}
	
	return false;
}
} //namespace PhysicsEffects
} //namespace sce
