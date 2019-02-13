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

#include "GuSweepBoxBox.h"
#include "GuBox.h"
#include "GuIntersectionBoxBox.h"
#include "GuIntersectionRayBox.h"
#include "GuIntersectionEdgeEdge.h"
#include "GuSweepSharedTests.h"
#include "CmMatrix34.h"
#include "GuSweepTriangleUtils.h"
#include "GuInternal.h"

using namespace physx;
using namespace Gu;
using namespace Cm;

namespace
{
// PT: TODO: get rid of this copy
static const PxReal gFatBoxEdgeCoeff = 0.01f;

// PT: TODO: get rid of this copy
static const PxVec3 gNearPlaneNormal[] = 
{
	PxVec3(1.0f, 0.0f, 0.0f),
	PxVec3(0.0f, 1.0f, 0.0f),
	PxVec3(0.0f, 0.0f, 1.0f),
	PxVec3(-1.0f, 0.0f, 0.0f),
	PxVec3(0.0f, -1.0f, 0.0f),
	PxVec3(0.0f, 0.0f, -1.0f)
};

#define	INVSQRT2 0.707106781188f	//!< 1 / sqrt(2)

static PxVec3 EdgeNormals[] = 
{
	PxVec3(0,			-INVSQRT2,	-INVSQRT2),	// 0-1
	PxVec3(INVSQRT2,	0,			-INVSQRT2),	// 1-2
	PxVec3(0,			INVSQRT2,	-INVSQRT2),	// 2-3
	PxVec3(-INVSQRT2,	0,			-INVSQRT2),	// 3-0

	PxVec3(0,			INVSQRT2,	INVSQRT2),	// 7-6
	PxVec3(INVSQRT2,	0,			INVSQRT2),	// 6-5
	PxVec3(0,			-INVSQRT2,	INVSQRT2),	// 5-4
	PxVec3(-INVSQRT2,	0,			INVSQRT2),	// 4-7

	PxVec3(INVSQRT2,	-INVSQRT2,	0),			// 1-5
	PxVec3(INVSQRT2,	INVSQRT2,	0),			// 6-2
	PxVec3(-INVSQRT2,	INVSQRT2,	0),			// 3-7
	PxVec3(-INVSQRT2,	-INVSQRT2,	0)			// 4-0
};

// PT: TODO: get rid of this copy
static const PxVec3* getBoxLocalEdgeNormals()
{
	return EdgeNormals;
}

/**
Returns world edge normal
\param		edgeIndex	[in] 0 <= edge index < 12
\param		worldNormal	[out] edge normal in world space
*/
static void computeBoxWorldEdgeNormal(const Box& box, PxU32 edgeIndex, PxVec3& worldNormal)
{
	PX_ASSERT(edgeIndex<12);
	worldNormal = box.rotate(getBoxLocalEdgeNormals()[edgeIndex]);
}

}

// ### optimize! and refactor. And optimize for aabbs
bool Gu::sweepBoxBox(const Box& box0, const Box& box1, const PxVec3& dir, PxReal length, PxHitFlags hitFlags, PxSweepHit& sweepHit)
{
	if(!(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP))
	{
		// PT: test if shapes initially overlap
		if(intersectOBBOBB(box0.extents, box0.center, box0.rot, box1.extents, box1.center, box1.rot, true))
		{
			sweepHit.flags		= PxHitFlag::eNORMAL;
			sweepHit.distance	= 0.0f;
			sweepHit.normal		= -dir;
			return true;
		}
	}

	PxVec3 boxVertices0[8];	box0.computeBoxPoints(boxVertices0);
	PxVec3 boxVertices1[8];	box1.computeBoxPoints(boxVertices1);

	//	float MinDist = PX_MAX_F32;
	PxReal MinDist = length;
	int col = -1;

	// In following VF tests:
	// - the direction is FW/BK since we project one box onto the other *and vice versa*
	// - the normal reaction is FW/BK for the same reason

	// Vertices1 against Box0
	{
		// We need:

		// - Box0 in local space
		const PxVec3 Min0 = -box0.extents;
		const PxVec3 Max0 = box0.extents;

		// - Vertices1 in Box0 space
		Matrix34 worldToBox0;
		computeWorldToBoxMatrix(worldToBox0, box0);

		// - the dir in Box0 space
		const PxVec3 localDir0 = worldToBox0.rotate(dir);

		const PxVec3* boxNormals0 = gNearPlaneNormal;

		for(PxU32 i=0; i<8; i++)
		{
			PxReal tnear, tfar;
			const int plane = intersectRayAABB(Min0, Max0, worldToBox0.transform(boxVertices1[i]), -localDir0, tnear, tfar);

			if(plane==-1 || tnear<0.0f)
				continue;

			if(tnear <= MinDist)
			{
				MinDist = tnear;
				sweepHit.normal = box0.rotate(boxNormals0[plane]);
				sweepHit.position = boxVertices1[i];
				col = 0;
			}
		}
	}

	// Vertices0 against Box1
	{
		// We need:

		// - Box1 in local space
		const PxVec3 Min1 = -box1.extents;
		const PxVec3 Max1 = box1.extents;

		// - Vertices0 in Box1 space
		Matrix34 worldToBox1;
		computeWorldToBoxMatrix(worldToBox1, box1);

		// - the dir in Box1 space
		const PxVec3 localDir1 = worldToBox1.rotate(dir);

		const PxVec3* boxNormals1 = gNearPlaneNormal;

		for(PxU32 i=0; i<8; i++)
		{
			PxReal tnear, tfar;
			const int plane = intersectRayAABB(Min1, Max1, worldToBox1.transform(boxVertices0[i]), localDir1, tnear, tfar);

			if(plane==-1 || tnear<0.0f)
				continue;

			if(tnear <= MinDist)
			{
				MinDist = tnear;
				sweepHit.normal = box1.rotate(-boxNormals1[plane]);
				sweepHit.position = boxVertices0[i] + tnear * dir;
				col = 1;
			}
		}
	}

	PxVec3 p1s, p2s, p3s, p4s;
	{
		const PxU8* PX_RESTRICT edges0 = getBoxEdges();
		const PxU8* PX_RESTRICT edges1 = getBoxEdges();

		PxVec3 edgeNormals0[12];
		PxVec3 edgeNormals1[12];
		for(PxU32 i=0; i<12; i++)
			computeBoxWorldEdgeNormal(box0, i, edgeNormals0[i]);
		for(PxU32 i=0; i<12; i++)
			computeBoxWorldEdgeNormal(box1, i, edgeNormals1[i]);

		// Loop through box edges
		for(PxU32 i=0; i<12; i++)	// 12 edges
		{
			if(!(edgeNormals0[i].dot(dir) >= 0.0f))
				continue;

			// Catch current box edge // ### one vertex already known using line-strips

			// Make it fat ###
			PxVec3 p1 = boxVertices0[edges0[i*2+0]];
			PxVec3 p2 = boxVertices0[edges0[i*2+1]];
			Ps::makeFatEdge(p1, p2, gFatBoxEdgeCoeff);

			// Loop through box edges
			for(PxU32 j=0;j<12;j++)
			{
				if(edgeNormals1[j].dot(dir) >= 0.0f)
					continue;

				// Orientation culling
				// PT: this was commented for some reason, but it fixes the "stuck" bug reported by Ubi.
				// So I put it back. We'll have to see whether it produces Bad Things in particular cases.
				if(edgeNormals0[i].dot(edgeNormals1[j]) >= 0.0f)
					continue;

				// Catch current box edge

				// Make it fat ###
				PxVec3 p3 = boxVertices1[edges1[j*2+0]];
				PxVec3 p4 = boxVertices1[edges1[j*2+1]];
				Ps::makeFatEdge(p3, p4, gFatBoxEdgeCoeff);

				PxReal Dist;
				PxVec3 ip;
				if(intersectEdgeEdge(p1, p2, dir, p3, p4, Dist, ip))
				{
					if(Dist<=MinDist)
					{
						p1s = p1;
						p2s = p2;
						p3s = p3;
						p4s = p4;

						sweepHit.position = ip + Dist * dir;

						col = 2;
						MinDist = Dist;
					}
				}
			}
		}
	}

	if(col==-1)
		return false;

	if(col==2)
	{
		computeEdgeEdgeNormal(sweepHit.normal, p1s, p2s-p1s, p3s, p4s-p3s, dir, MinDist);
		sweepHit.normal.normalize();
	}

	sweepHit.flags		= PxHitFlag::eNORMAL|PxHitFlag::ePOSITION;
	sweepHit.distance	= MinDist;
	return true;
}
