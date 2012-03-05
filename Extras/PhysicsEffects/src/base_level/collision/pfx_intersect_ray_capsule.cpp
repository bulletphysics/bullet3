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
#include "pfx_intersect_ray_capsule.h"

namespace sce {
namespace PhysicsEffects {
PfxBool pfxIntersectRayCapsule(const PfxRayInput &ray,PfxRayOutput &out,const PfxCapsule &capsule,const PfxTransform3 &transform)
{
	// レイをCapsuleのローカル座標へ変換
	PfxTransform3 transformCapsule = orthoInverse(transform);
	PfxVector3 startPosL = transformCapsule.getUpper3x3() * ray.m_startPosition + transformCapsule.getTranslation();
	PfxVector3 rayDirL = transformCapsule.getUpper3x3() * ray.m_direction;
	
	PfxFloat radSqr = capsule.m_radius * capsule.m_radius;

	// 始点がカプセルの内側にあるか判定
	{
		PfxFloat h = fabsf(startPosL[0]);
		if(h > capsule.m_halfLen) h = capsule.m_halfLen;
		PfxVector3 Px(out.m_variable,0,0);
		PfxFloat sqrLen = lengthSqr(startPosL-Px);
		if(sqrLen <= radSqr) return false;
	}

	// カプセルの胴体との交差判定
	do {
		PfxVector3 P(startPosL);
		PfxVector3 D(rayDirL);
		
		P[0] = 0.0f;
		D[0] = 0.0f;
		
		PfxFloat a = dot(D,D);
		PfxFloat b = dot(P,D);
		PfxFloat c = dot(P,P) - radSqr;
		
		PfxFloat d = b * b - a * c;
		
		if(d < 0.0f || fabs(a) < 0.00001f) return false;
		
		PfxFloat tt = ( -b - sqrtf(d) ) / a;
		
		if(tt < 0.0f)
			break;
		else if(tt > 1.0f)
			return false;
		
		if(tt < out.m_variable) {
			PfxVector3 cp = startPosL + tt * rayDirL;
			
			if(fabsf(cp[0]) <= capsule.m_halfLen) {
				out.m_contactFlag = true;
				out.m_variable = tt;
				out.m_contactPoint = PfxVector3(transform * PfxPoint3(cp));
				out.m_contactNormal = transform.getUpper3x3() * normalize(cp);
				out.m_subData.m_type = PfxSubData::NONE;
				return true;
			}
		}
	} while(0);
	
	// カプセルの両端にある球体との交差判定
	PfxFloat a = dot(rayDirL,rayDirL);
	if(fabs(a) < 0.00001f) return false;
	
	do {
		PfxVector3 center(capsule.m_halfLen,0.0f,0.0f);
		PfxVector3 v = startPosL - center;

		PfxFloat b = dot(v,rayDirL);
		PfxFloat c = dot(v,v) - radSqr;

		PfxFloat d = b * b - a * c;
		
		if(d < 0.0f) break;
		
		PfxFloat tt = ( -b - sqrtf(d) ) / a;
		
		if(tt < 0.0f || tt > 1.0f) break;
		
		if(tt < out.m_variable) {
			PfxVector3 cp = startPosL + tt * rayDirL;
			out.m_contactFlag = true;
			out.m_variable = tt;
			out.m_contactPoint = ray.m_startPosition + tt * ray.m_direction;
			out.m_contactNormal = transform.getUpper3x3() * normalize(cp-center);
			out.m_subData.m_type = PfxSubData::NONE;
			return true;
		}
	} while(0);
	
	{
		PfxVector3 center(-capsule.m_halfLen,0.0f,0.0f);
		PfxVector3 v = startPosL - center;

		PfxFloat b = dot(v,rayDirL);
		PfxFloat c = dot(v,v) - radSqr;

		PfxFloat d = b * b - a * c;
		
		if(d < 0.0f) return false;
		
		PfxFloat tt = ( -b - sqrtf(d) ) / a;
		
		if(tt < 0.0f || tt > 1.0f) return false;
		
		if(tt < out.m_variable) {
			PfxVector3 cp = startPosL + out.m_variable * rayDirL;
			out.m_contactFlag = true;
			out.m_variable = tt;
			out.m_contactPoint = ray.m_startPosition + tt * ray.m_direction;
			out.m_contactNormal = transform.getUpper3x3() * normalize(cp-center);
			out.m_subData.m_type = PfxSubData::NONE;
			return true;
		}
	}
	
	return false;
}
} //namespace PhysicsEffects
} //namespace sce
