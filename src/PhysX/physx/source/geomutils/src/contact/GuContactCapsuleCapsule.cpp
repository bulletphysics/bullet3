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

#include "GuContactBuffer.h"
#include "GuDistanceSegmentSegment.h"
#include "GuContactMethodImpl.h"
#include "GuInternal.h"
#include "GuGeometryUnion.h"

using namespace physx;

namespace physx
{
namespace Gu
{
bool contactCapsuleCapsule(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);

	const PxCapsuleGeometry& capsuleGeom0 = shape0.get<const PxCapsuleGeometry>();
	const PxCapsuleGeometry& capsuleGeom1 = shape1.get<const PxCapsuleGeometry>();

	// PT: get capsules in local space
	PxVec3 dir[2];
	Segment segment[2];
	{
		const PxVec3 capsuleLocalSegment0 = getCapsuleHalfHeightVector(transform0, capsuleGeom0);
		const PxVec3 capsuleLocalSegment1 = getCapsuleHalfHeightVector(transform1, capsuleGeom1);

		const PxVec3 delta = transform1.p - transform0.p;
		segment[0].p0 = capsuleLocalSegment0;
		segment[0].p1 = -capsuleLocalSegment0;
		dir[0] = -capsuleLocalSegment0*2.0f;
		segment[1].p0 = capsuleLocalSegment1 + delta;
		segment[1].p1 = -capsuleLocalSegment1 + delta;
		dir[1] = -capsuleLocalSegment1*2.0f;
	}

	// PT: compute distance between capsules' segments
	PxReal s,t;
	const PxReal squareDist = distanceSegmentSegmentSquared(segment[0], segment[1], &s, &t);
	const PxReal radiusSum = capsuleGeom0.radius + capsuleGeom1.radius;
	const PxReal inflatedSum = radiusSum + params.mContactDistance;
	const PxReal inflatedSumSquared = inflatedSum*inflatedSum;

	if(squareDist >= inflatedSumSquared)
		return false;

	// PT: TODO: optimize this away
	PxReal segLen[2];
	segLen[0] = dir[0].magnitude();
	segLen[1] = dir[1].magnitude();

	if (segLen[0]) dir[0] *= 1.0f / segLen[0];
	if (segLen[1]) dir[1] *= 1.0f / segLen[1];

	if (PxAbs(dir[0].dot(dir[1])) > 0.9998f)	//almost parallel, ca. 1 degree difference --> generate two contact points at ends
	{
		PxU32 numCons = 0;

		PxReal segLenEps[2];
		segLenEps[0] = segLen[0] * 0.001f;//0.1% error is ok.
		segLenEps[1] = segLen[1] * 0.001f;
			
		//project the two end points of each onto the axis of the other and take those 4 points.
		//we could also generate a single normal at the single closest point, but this would be 'unstable'.

		for (PxU32 destShapeIndex = 0; destShapeIndex < 2; destShapeIndex ++)
		{
			for (PxU32 startEnd = 0; startEnd < 2; startEnd ++)
			{
				const PxU32 srcShapeIndex = 1-destShapeIndex;
				//project start/end of srcShapeIndex onto destShapeIndex.
				PxVec3 pos[2];
				pos[destShapeIndex] = startEnd ? segment[srcShapeIndex].p1 : segment[srcShapeIndex].p0;
				const PxReal p = dir[destShapeIndex].dot(pos[destShapeIndex] - segment[destShapeIndex].p0);
				if (p >= -segLenEps[destShapeIndex] && p <= (segLen[destShapeIndex] + segLenEps[destShapeIndex]))
				{
					pos[srcShapeIndex] = p * dir[destShapeIndex] + segment[destShapeIndex].p0;

					PxVec3 normal = pos[1] - pos[0];

					const PxReal normalLenSq = normal.magnitudeSquared();
					if (normalLenSq > 1e-6f && normalLenSq < inflatedSumSquared)
					{
						const PxReal distance = PxSqrt(normalLenSq);
						normal *= 1.0f/distance;
						PxVec3 point = pos[1] - normal * (srcShapeIndex ? capsuleGeom1 : capsuleGeom0).radius;
						point += transform0.p;
						contactBuffer.contact(point, normal, distance - radiusSum);
						numCons++;
					}					
				}
			}
		}

		if (numCons)	//if we did not have contacts, then we may have the case where they are parallel, but are stacked end to end, in which case the old code will generate good contacts.
			return true;
	}

	// Collision response
	PxVec3 pos1 = segment[0].getPointAt(s);
	PxVec3 pos2 = segment[1].getPointAt(t);

	PxVec3 normal = pos1 - pos2;

	const PxReal normalLenSq = normal.magnitudeSquared();
	if (normalLenSq < 1e-6f)
	{
		// PT: TODO: revisit this. "FW" sounds old.
		// Zero normal -> pick the direction of segment 0.
		// Not always accurate but consistent with FW.
		if (segLen[0] > 1e-6f)
			normal = dir[0];
		else 
			normal = PxVec3(1.0f, 0.0f, 0.0f);
	}
	else
	{
		normal *= PxRecipSqrt(normalLenSq);
	}
	
	pos1 += transform0.p;
	contactBuffer.contact(pos1 - normal * capsuleGeom0.radius, normal, PxSqrt(squareDist) - radiusSum);
	return true;
}
}//Gu
}//physx
