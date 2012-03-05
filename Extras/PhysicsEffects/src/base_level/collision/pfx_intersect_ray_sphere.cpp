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

#include "../../../include/physics_effects/base_level/collision/pfx_sphere.h"
#include "pfx_intersect_common.h"
#include "pfx_intersect_ray_sphere.h"

namespace sce {
namespace PhysicsEffects {

PfxBool pfxIntersectRaySphere(const PfxRayInput &ray,PfxRayOutput &out,const PfxSphere &sphere,const PfxTransform3 &transform)
{
	PfxVector3 v = ray.m_startPosition - transform.getTranslation();

	PfxFloat a = dot(ray.m_direction,ray.m_direction);
	PfxFloat b = dot(v,ray.m_direction);
	PfxFloat c = dot(v,v) - sphere.m_radius * sphere.m_radius;

	if(c < 0.0f) return false;

	PfxFloat d = b * b - a * c;
	
	if(d < 0.0f || fabsf(a) < 0.00001f) return false;
	
	PfxFloat tt = ( -b - sqrtf(d) ) / a;
	
	if(tt < 0.0f || tt > 1.0f) return false;
	
	if(tt < out.m_variable) {
		out.m_contactFlag = true;
		out.m_variable = tt;
		out.m_contactPoint = ray.m_startPosition + tt * ray.m_direction;
		out.m_contactNormal = normalize(out.m_contactPoint - transform.getTranslation());
		out.m_subData.m_type = PfxSubData::NONE;
		return true;
	}
	
	return false;
}

} //namespace PhysicsEffects
} //namespace sce
