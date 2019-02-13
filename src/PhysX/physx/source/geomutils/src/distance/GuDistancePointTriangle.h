//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef GU_DISTANCE_POINT_TRIANGLE_H
#define GU_DISTANCE_POINT_TRIANGLE_H

#include "foundation/PxVec3.h"
#include "PxPhysXCommonConfig.h"
#include "CmPhysXCommon.h"

namespace physx
{
namespace Gu
{
	// PT: special version:
	// - inlined
	// - doesn't compute (s,t) output params
	// - expects precomputed edges in input
	PX_FORCE_INLINE PxVec3 closestPtPointTriangle2(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& ab, const PxVec3& ac)
	{
		// Check if P in vertex region outside A
		//const PxVec3 ab = b - a;
		//const PxVec3 ac = c - a;
		const PxVec3 ap = p - a;
		const float d1 = ab.dot(ap);
		const float d2 = ac.dot(ap);
		if(d1<=0.0f && d2<=0.0f)
			return a;	// Barycentric coords 1,0,0

		// Check if P in vertex region outside B
		const PxVec3 bp = p - b;
		const float d3 = ab.dot(bp);
		const float d4 = ac.dot(bp);
		if(d3>=0.0f && d4<=d3)
			return b;	// Barycentric coords 0,1,0

		// Check if P in edge region of AB, if so return projection of P onto AB
		const float vc = d1*d4 - d3*d2;
		if(vc<=0.0f && d1>=0.0f && d3<=0.0f)
		{
			const float v = d1 / (d1 - d3);
			return a + v * ab;	// barycentric coords (1-v, v, 0)
		}

		// Check if P in vertex region outside C
		const PxVec3 cp = p - c;
		const float d5 = ab.dot(cp);
		const float d6 = ac.dot(cp);
		if(d6>=0.0f && d5<=d6)
			return c;	// Barycentric coords 0,0,1

		// Check if P in edge region of AC, if so return projection of P onto AC
		const float vb = d5*d2 - d1*d6;
		if(vb<=0.0f && d2>=0.0f && d6<=0.0f)
		{
			const float w = d2 / (d2 - d6);
			return a + w * ac;	// barycentric coords (1-w, 0, w)
		}

		// Check if P in edge region of BC, if so return projection of P onto BC
		const float va = d3*d6 - d5*d4;
		if(va<=0.0f && (d4-d3)>=0.0f && (d5-d6)>=0.0f)
		{
			const float w = (d4-d3) / ((d4 - d3) + (d5-d6));
			return b + w * (c-b);	// barycentric coords (0, 1-w, w)
		}

		// P inside face region. Compute Q through its barycentric coords (u,v,w)
		const float denom = 1.0f / (va + vb + vc);
		const float v = vb * denom;
		const float w = vc * denom;
		return a + ab*v + ac*w;
	}

	PX_PHYSX_COMMON_API PxVec3 closestPtPointTriangle(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, float& s, float& t);

	PX_FORCE_INLINE PxReal distancePointTriangleSquared(const PxVec3& point, 
														const PxVec3& triangleOrigin, 
														const PxVec3& triangleEdge0, 
														const PxVec3& triangleEdge1,
														PxReal* param0=NULL, 
														PxReal* param1=NULL)
	{
		const PxVec3 pt0 = triangleEdge0 + triangleOrigin;
		const PxVec3 pt1 = triangleEdge1 + triangleOrigin;
		float s,t;
		const PxVec3 cp = closestPtPointTriangle(point, triangleOrigin, pt0, pt1, s, t);
		if(param0)
			*param0 = s;
		if(param1)
			*param1 = t;
		return (cp - point).magnitudeSquared();
	}

} // namespace Gu

}

#endif
