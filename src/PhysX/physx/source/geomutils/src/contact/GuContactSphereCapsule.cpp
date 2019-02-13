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

#include "GuDistancePointSegment.h"
#include "GuContactMethodImpl.h"
#include "GuContactBuffer.h"
#include "GuInternal.h"
#include "GuGeometryUnion.h"

namespace physx
{
namespace Gu
{
bool contactSphereCapsule(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);

	const PxSphereGeometry& sphereGeom = shape0.get<const PxSphereGeometry>();
	const PxCapsuleGeometry& capsuleGeom = shape1.get<const PxCapsuleGeometry>();

	// PT: get capsule in local space
	const PxVec3 capsuleLocalSegment = getCapsuleHalfHeightVector(transform1, capsuleGeom);
	const Segment localSegment(capsuleLocalSegment, -capsuleLocalSegment);

	// PT: get sphere in capsule space
	const PxVec3 sphereCenterInCapsuleSpace = transform0.p - transform1.p;

	const PxReal radiusSum = sphereGeom.radius + capsuleGeom.radius;
	const PxReal inflatedSum = radiusSum + params.mContactDistance;

	// PT: compute distance between sphere center & capsule's segment
	PxReal u;
	const PxReal squareDist = distancePointSegmentSquared(localSegment, sphereCenterInCapsuleSpace, &u);
	if(squareDist >= inflatedSum*inflatedSum)
		return false;

	// PT: compute contact normal
	PxVec3 normal = sphereCenterInCapsuleSpace - localSegment.getPointAt(u);
		
	// We do a *manual* normalization to check for singularity condition
	const PxReal lenSq = normal.magnitudeSquared();
	if(lenSq==0.0f) 
		normal = PxVec3(1.0f, 0.0f, 0.0f);	// PT: zero normal => pick up random one
	else
		normal *= PxRecipSqrt(lenSq);

	// PT: compute contact point
	const PxVec3 point = sphereCenterInCapsuleSpace + transform1.p - normal * sphereGeom.radius;

	// PT: output unique contact
	contactBuffer.contact(point, normal, PxSqrt(squareDist) - radiusSum);
	return true;
}
}//Gu
}//physx
