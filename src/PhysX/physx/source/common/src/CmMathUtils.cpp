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
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "foundation/PxPreprocessor.h"
#include "foundation/PxTransform.h"
#include "foundation/PxMathUtils.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{


PX_PHYSX_COMMON_API PxTransform PxTransformFromPlaneEquation(const PxPlane& plane)
{
	PxPlane p = plane; 
	p.normalize();

	// special case handling for axis aligned planes
	const PxReal halfsqrt2 = 0.707106781f;
	PxQuat q;
	if(2 == (p.n.x == 0.0f) + (p.n.y == 0.0f) + (p.n.z == 0.0f)) // special handling for axis aligned planes
	{
		if(p.n.x > 0)		q = PxQuat(PxIdentity);
		else if(p.n.x < 0)	q = PxQuat(0, 0, 1.0f, 0);
		else				q = PxQuat(0.0f, -p.n.z, p.n.y, 1.0f) * halfsqrt2;
	}
	else q = PxShortestRotation(PxVec3(1.f,0,0), p.n);

	return PxTransform(-p.n * p.d, q);

}

PX_PHYSX_COMMON_API PxTransform PxTransformFromSegment(const PxVec3& p0, const PxVec3& p1, PxReal* halfHeight)
{
	const PxVec3 axis = p1-p0;
	const PxReal height = axis.magnitude();
	if(halfHeight)
		*halfHeight = height/2;

	return PxTransform((p1+p0) * 0.5f, 
						height<1e-6f ? PxQuat(PxIdentity) : PxShortestRotation(PxVec3(1.f,0,0), axis/height));		
}

}
