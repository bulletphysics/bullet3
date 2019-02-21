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

#include "GuShapeConvex.h"
#include "GuBigConvexData.h"
#include "GuEdgeListData.h"
#include "GuInternal.h"

#include "CmMatrix34.h"
#include "GuHillClimbing.h"
#include "PsFPU.h"

using namespace physx;
using namespace Gu;

static PX_FORCE_INLINE PxU32 selectClosestPolygon(PxReal& maxDp_, PxU32 numPolygons, const Gu::HullPolygonData* polys, const PxVec3& axis)
{
	float maxDp = polys[0].mPlane.n.dot(axis);
	PxU32 closest = 0;

	// Loop through polygons
	for(PxU32 i=1; i <numPolygons; i++)
	{
		// Catch current polygon and test its plane
		const PxReal dp = polys[i].mPlane.n.dot(axis);
		if(dp>maxDp)
		{
			maxDp = dp;
			closest = i;
		}
	}
	maxDp_ = maxDp;
	return closest;
}

static PxU32 SelectClosestEdgeCB_Convex(const PolygonalData& data, const Cm::FastVertex2ShapeScaling& scaling, const PxVec3& localSpaceDirection)
{
	//vertex1TOShape1Skew is a symmetric matrix.  
	//it has the property that (vertex1TOShape1Skew * v)|localSpaceDirection == (vertex1TOShape1Skew * localSpaceDirection)|v 
	const PxVec3 vertexSpaceDirection = scaling * localSpaceDirection;

	const Gu::HullPolygonData* PX_RESTRICT polys = data.mPolygons;

	PxReal maxDp;
	// ##might not be needed
	PxU32 closest = ::selectClosestPolygon(maxDp, data.mNbPolygons, polys, vertexSpaceDirection);

	// Since the convex is closed, at least some poly must satisfy this
	PX_ASSERT(maxDp>=0);

	const PxU32 numEdges = data.mNbEdges;
	const PxU8* const edgeToFace = data.mFacesByEdges;

	//Loop through edges
	PxU32 closestEdge = 0xffffffff;
	PxReal maxDpSq = maxDp * maxDp;
	for(PxU32 i=0; i < numEdges; i++)
	{
		const PxU8 f0 = edgeToFace[i*2];
		const PxU8 f1 = edgeToFace[i*2+1];

		// unnormalized edge normal
		const PxVec3 edgeNormal = polys[f0].mPlane.n + polys[f1].mPlane.n;
		const PxReal enMagSq = edgeNormal.magnitudeSquared();
		//Test normal of current edge - squared test is valid if dp and maxDp both >= 0
		const float dp = edgeNormal.dot(vertexSpaceDirection);
		if(dp>=0.0f && dp*dp>maxDpSq*enMagSq)
		{
			maxDpSq = dp*dp/enMagSq;
			closestEdge = i;
		}
	}

	if(closestEdge!=0xffffffff)
	{
		const PxU8* FBE = edgeToFace;

		const PxU32 f0 = FBE[closestEdge*2];
		const PxU32 f1 = FBE[closestEdge*2+1];

		const PxReal dp0 = polys[f0].mPlane.n.dot(vertexSpaceDirection);
		const PxReal dp1 = polys[f1].mPlane.n.dot(vertexSpaceDirection);
		if(dp0>dp1)
			closest = f0;
		else
			closest = f1;
	}
	return closest;
}
 
// Hull projection callback for "small" hulls
static void HullProjectionCB_SmallConvex(const PolygonalData& data, const PxVec3& dir,
										 const Cm::Matrix34& world,
										 const Cm::FastVertex2ShapeScaling& scaling,
										 PxReal& min, PxReal& max)
{
	const PxVec3 localSpaceDirection = world.rotateTranspose(dir);
	//vertex1TOShape1Skew is a symmetric matrix.  
	//it has the property that (vertex1TOShape1Skew * v)|localSpaceDirection == (vertex1TOShape1Skew * localSpaceDirection)|v 
	const PxVec3 vertexSpaceDirection = scaling * localSpaceDirection;

	// PT: prevents aliasing
	PxReal minimum = PX_MAX_REAL;
	PxReal maximum = -PX_MAX_REAL;

	//brute-force, localspace
	{
		const PxVec3* PX_RESTRICT verts = data.mVerts;
		PxU32 numVerts = data.mNbVerts;
		while(numVerts--)
		{
			const PxReal dp = (*verts++).dot(vertexSpaceDirection);
			minimum = physx::intrinsics::selectMin(minimum, dp);
			maximum = physx::intrinsics::selectMax(maximum, dp);
		}

	}

	const PxReal offset = world.p.dot(dir);
	min = minimum + offset;
	max = maximum + offset;
}

static PxU32 computeNearestOffset(const PxU32 subdiv, const PxVec3& dir)
{
	// ComputeCubemapNearestOffset(const Point& dir, udword subdiv)

	// PT: ok so why exactly was the code duplicated here?
	// PxU32 CI = CubemapLookup(dir,u,v)

	PxU32 index;
	PxReal coeff;
	// find largest axis
	PxReal absNx = PxAbs(dir.x);
	PxReal absNy = PxAbs(dir.y);
	PxReal absNz = PxAbs(dir.z);

	if( absNy > absNx &&
		absNy > absNz)
	{
		//y biggest
		index = 1;
		coeff = 1.0f/absNy;
	}
	else if(absNz > absNx)
	{
		index = 2;
		coeff = 1.0f/absNz;
	}
	else
	{
		index = 0;
		coeff = 1.0f/absNx;
	}

	union
	{
		PxU32 aU32;
		PxReal aFloat;
	} conv;

	conv.aFloat = dir[index];
	PxU32 sign = conv.aU32>>31;
	
	const PxU32 index2 = Ps::getNextIndex3(index);
	const PxU32 index3 = Ps::getNextIndex3(index2);
	PxReal u = dir[index2] * coeff;
	PxReal v = dir[index3] * coeff;
	
	PxU32 CI = (sign | (index+index));

	//Remap to [0, subdiv[
	coeff = 0.5f * PxReal(subdiv-1);
	u += 1.0f; u *= coeff;
	v += 1.0f; v *= coeff;

	//Round to nearest
	PxU32 ui = PxU32(u);
	PxU32 vi = PxU32(v);

	PxReal du = u - PxReal(ui);
	PxReal dv = v - PxReal(vi);
	if(du>0.5f) ui++;
	if(dv>0.5f) vi++;

	//Compute offset
	return CI*(subdiv*subdiv) + ui*subdiv + vi;
}

// Hull projection callback for "big" hulls
static void HullProjectionCB_BigConvex(const PolygonalData& data, const PxVec3& dir, const Cm::Matrix34& world, const Cm::FastVertex2ShapeScaling& scaling, PxReal& minimum, PxReal& maximum)
{
	const PxVec3* PX_RESTRICT verts = data.mVerts;

	const PxVec3 localSpaceDirection = world.rotateTranspose(dir);
	//vertex1TOShape1Skew is a symmetric matrix.  
	//it has the property that (vertex1TOShape1Skew * v)|localSpaceDirection == (vertex1TOShape1Skew * localSpaceDirection)|v 
	const PxVec3 vertexSpaceDirection = scaling * localSpaceDirection;	//NB: triangles are always shape 1! eek!

	// This version is better for objects with a lot of vertices
	const Gu::BigConvexRawData* bigData = data.mBigData;
	PxU32 minID = 0, maxID = 0;
	{
		const PxU32 offset = computeNearestOffset(bigData->mSubdiv, -vertexSpaceDirection);
		minID = bigData->mSamples[offset];
		maxID = bigData->getSamples2()[offset];
	}
	
	// Do hillclimbing!
	localSearch(minID, -vertexSpaceDirection, verts, bigData);
	localSearch(maxID, vertexSpaceDirection, verts, bigData);

	const PxReal offset = world.p.dot(dir);
	minimum = offset + verts[minID].dot(vertexSpaceDirection);
	maximum = offset + verts[maxID].dot(vertexSpaceDirection);
	PX_ASSERT(maximum >= minimum);
}

void Gu::getPolygonalData_Convex(PolygonalData* PX_RESTRICT dst, const Gu::ConvexHullData* PX_RESTRICT src, const Cm::FastVertex2ShapeScaling& scaling)
{
	dst->mCenter			= scaling * src->mCenterOfMass;
	dst->mNbVerts			= src->mNbHullVertices;
	dst->mNbPolygons		= src->mNbPolygons;
	dst->mNbEdges			= src->mNbEdges;
	dst->mPolygons			= src->mPolygons;
	dst->mVerts				= src->getHullVertices();
	dst->mPolygonVertexRefs	= src->getVertexData8();
	dst->mFacesByEdges		= src->getFacesByEdges8();

// TEST_INTERNAL_OBJECTS
	dst->mInternal			= src->mInternal;
//~TEST_INTERNAL_OBJECTS

	dst->mBigData			= src->mBigConvexRawData;

	// This threshold test doesnt cost much and many customers cook on PC and use this on 360.
	// 360 has a much higher threshold than PC(and it makes a big difference)
	// PT: the cool thing is that this test is now done once by contact generation call, not once by hull projection
	if(!src->mBigConvexRawData)
		dst->mProjectHull = HullProjectionCB_SmallConvex;
	else
		dst->mProjectHull = HullProjectionCB_BigConvex;
	dst->mSelectClosestEdgeCB = SelectClosestEdgeCB_Convex;
}

// Box emulating convex mesh

// Face0: 0-1-2-3
// Face1: 1-5-6-2
// Face2: 5-4-7-6
// Face3: 4-0-3-7
// Face4; 3-2-6-7
// Face5: 4-5-1-0

//     7+------+6			0 = ---
//     /|     /|			1 = +--
//    / |    / |			2 = ++-
//   / 4+---/--+5			3 = -+-
// 3+------+2 /    y   z	4 = --+
//  | /    | /     |  /		5 = +-+
//  |/     |/      |/		6 = +++
// 0+------+1      *---x	7 = -++

static const PxU8 gPxcBoxPolygonData[] = {
	0, 1, 2, 3,
	1, 5, 6, 2,
	5, 4, 7, 6,
	4, 0, 3, 7,
	3, 2, 6, 7,
	4, 5, 1, 0,
};

#define	INVSQRT2	0.707106781188f				//!< 1 / sqrt(2)
static PxVec3 gPxcBoxEdgeNormals[] = 
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
#undef INVSQRT2

// ### needs serious checkings
	// Flags(16), Count(16), Offset(32);
static Gu::EdgeDescData gPxcBoxEdgeDesc[] = {
	{Gu::PX_EDGE_ACTIVE, 2, 0},
	{Gu::PX_EDGE_ACTIVE, 2, 2},
	{Gu::PX_EDGE_ACTIVE, 2, 4},
	{Gu::PX_EDGE_ACTIVE, 2, 6},
	{Gu::PX_EDGE_ACTIVE, 2, 8},
	{Gu::PX_EDGE_ACTIVE, 2, 10},
	{Gu::PX_EDGE_ACTIVE, 2, 12},
	{Gu::PX_EDGE_ACTIVE, 2, 14},
	{Gu::PX_EDGE_ACTIVE, 2, 16},
	{Gu::PX_EDGE_ACTIVE, 2, 18},
	{Gu::PX_EDGE_ACTIVE, 2, 20},
	{Gu::PX_EDGE_ACTIVE, 2, 22},
};

// ### needs serious checkings
static PxU8 gPxcBoxFaceByEdge[] = {
	0,5, 	// Edge 0-1
	0,1, 	// Edge 1-2
	0,4, 	// Edge 2-3
	0,3, 	// Edge 3-0
	2,4, 	// Edge 7-6
	1,2, 	// Edge 6-5
	2,5, 	// Edge 5-4
	2,3, 	// Edge 4-7
	1,5, 	// Edge 1-5
	1,4, 	// Edge 6-2
	3,4, 	// Edge 3-7
	3,5, 	// Edge 4-0
};

static PxU32 SelectClosestEdgeCB_Box(const PolygonalData& data, const Cm::FastVertex2ShapeScaling& scaling, const PxVec3& localDirection)
{
	PX_UNUSED(scaling);

	PxReal maxDp;
	// ##might not be needed
	PxU32 closest = ::selectClosestPolygon(maxDp, 6, data.mPolygons, localDirection);

	PxU32 numEdges = 12;
	const PxVec3* PX_RESTRICT edgeNormals = gPxcBoxEdgeNormals;

	//Loop through edges
	PxU32 closestEdge = 0xffffffff;
	for(PxU32 i=0; i < numEdges; i++)
	{
		//Test normal of current edge
		const float dp = edgeNormals[i].dot(localDirection);
		if(dp>maxDp)
		{
			maxDp = dp;
			closestEdge = i;
		}
	}

	if(closestEdge!=0xffffffff)
	{
		const Gu::EdgeDescData* PX_RESTRICT ED = gPxcBoxEdgeDesc;
		const PxU8* PX_RESTRICT FBE = gPxcBoxFaceByEdge;

		PX_ASSERT(ED[closestEdge].Count==2);
		const PxU32 f0 = FBE[ED[closestEdge].Offset];
		const PxU32 f1 = FBE[ED[closestEdge].Offset+1];

		const PxReal dp0 = data.mPolygons[f0].mPlane.n.dot(localDirection);
		const PxReal dp1 = data.mPolygons[f1].mPlane.n.dot(localDirection);
		if(dp0>dp1)
			closest = f0;
		else
			closest = f1;
	}

	return closest;
}

static PX_FORCE_INLINE void projectBox(PxVec3& p, const PxVec3& localDir, const PxVec3& extents)
{
	// PT: the original code didn't have branches or FPU comparisons. Why rewrite it ?
//	p.x = (localDir.x >= 0) ? extents.x : -extents.x;
//	p.y = (localDir.y >= 0) ? extents.y : -extents.y;
//	p.z = (localDir.z >= 0) ? extents.z : -extents.z;
	p.x = physx::intrinsics::fsel(localDir.x, extents.x, -extents.x);
	p.y = physx::intrinsics::fsel(localDir.y, extents.y, -extents.y);
	p.z = physx::intrinsics::fsel(localDir.z, extents.z, -extents.z);
}

static void	HullProjectionCB_Box(const PolygonalData& data, const PxVec3& dir, const Cm::Matrix34& world, const Cm::FastVertex2ShapeScaling& scaling, PxReal& minimum, PxReal& maximum)
{
	PX_UNUSED(scaling);

	const PxVec3 localDir = world.rotateTranspose(dir);

	PxVec3 p;
	projectBox(p, localDir, *data.mHalfSide);

	const PxReal offset = world.p.dot(dir);
	const PxReal tmp = p.dot(localDir);
	maximum = offset + tmp;
	minimum = offset - tmp;
}

PolygonalBox::PolygonalBox(const PxVec3& halfSide) : mHalfSide(halfSide)
{
	//Precompute the convex data
	//     7+------+6			0 = ---
	//     /|     /|			1 = +--
	//    / |    / |			2 = ++-
	//   / 4+---/--+5			3 = -+-
	// 3+------+2 /    y   z	4 = --+
	//  | /    | /     |  /		5 = +-+
	//  |/     |/      |/		6 = +++
	// 0+------+1      *---x	7 = -++

	PxVec3 minimum = -mHalfSide;
	PxVec3 maximum = mHalfSide;
	// Generate 8 corners of the bbox
	mVertices[0] = PxVec3(minimum.x, minimum.y, minimum.z);
	mVertices[1] = PxVec3(maximum.x, minimum.y, minimum.z);
	mVertices[2] = PxVec3(maximum.x, maximum.y, minimum.z);
	mVertices[3] = PxVec3(minimum.x, maximum.y, minimum.z);
	mVertices[4] = PxVec3(minimum.x, minimum.y, maximum.z);
	mVertices[5] = PxVec3(maximum.x, minimum.y, maximum.z);
	mVertices[6] = PxVec3(maximum.x, maximum.y, maximum.z);
	mVertices[7] = PxVec3(minimum.x, maximum.y, maximum.z);

	//Setup the polygons
	for(PxU8 i=0; i < 6; i++)
	{
		mPolygons[i].mNbVerts = 4;
		mPolygons[i].mVRef8 = PxU16(i*4);
	}

	// ### planes needs *very* careful checks
	// X axis
	mPolygons[1].mPlane.n = PxVec3(1.0f, 0.0f, 0.0f);
	mPolygons[1].mPlane.d = -mHalfSide.x;
	mPolygons[3].mPlane.n = PxVec3(-1.0f, 0.0f, 0.0f);
	mPolygons[3].mPlane.d = -mHalfSide.x;
	
	mPolygons[1].mMinIndex = 0;
	mPolygons[3].mMinIndex = 1;

//	mPolygons[1].mMinObsolete = -mHalfSide.x;
//	mPolygons[3].mMinObsolete = -mHalfSide.x;

	PX_ASSERT(mPolygons[1].getMin(mVertices) == -mHalfSide.x); 
	PX_ASSERT(mPolygons[3].getMin(mVertices) == -mHalfSide.x);


	// Y axis
	mPolygons[4].mPlane.n = PxVec3(0.f, 1.0f, 0.0f);
	mPolygons[4].mPlane.d = -mHalfSide.y;
	mPolygons[5].mPlane.n = PxVec3(0.0f, -1.0f, 0.0f);
	mPolygons[5].mPlane.d = -mHalfSide.y;

	mPolygons[4].mMinIndex = 0;
	mPolygons[5].mMinIndex = 2;
//	mPolygons[4].mMinObsolete = -mHalfSide.y;
//	mPolygons[5].mMinObsolete = -mHalfSide.y;

	PX_ASSERT(mPolygons[4].getMin(mVertices) == -mHalfSide.y); 
	PX_ASSERT(mPolygons[5].getMin(mVertices) == -mHalfSide.y);

	// Z axis
	mPolygons[2].mPlane.n = PxVec3(0.f, 0.0f, 1.0f);
	mPolygons[2].mPlane.d = -mHalfSide.z;
	mPolygons[0].mPlane.n = PxVec3(0.0f, 0.0f, -1.0f);
	mPolygons[0].mPlane.d = -mHalfSide.z;

	mPolygons[2].mMinIndex = 0;
	mPolygons[0].mMinIndex = 4;
//	mPolygons[2].mMinObsolete = -mHalfSide.z;
//	mPolygons[0].mMinObsolete = -mHalfSide.z;
	PX_ASSERT(mPolygons[2].getMin(mVertices) == -mHalfSide.z); 
	PX_ASSERT(mPolygons[0].getMin(mVertices) == -mHalfSide.z);
}

void PolygonalBox::getPolygonalData(PolygonalData* PX_RESTRICT dst) const
{
	dst->mCenter				= PxVec3(0.0f, 0.0f, 0.0f);
	dst->mNbVerts				= 8;
	dst->mNbPolygons			= 6;
	dst->mPolygons				= mPolygons;
	dst->mNbEdges				= 0;
	dst->mVerts					= mVertices;
	dst->mPolygonVertexRefs		= gPxcBoxPolygonData;
	dst->mFacesByEdges			= NULL;
	dst->mInternal.mRadius		= 0.0f;
	dst->mInternal.mExtents[0]	= 0.0f;
	dst->mInternal.mExtents[1]	= 0.0f;
	dst->mInternal.mExtents[2]	= 0.0f;
//	dst->mBigData				= NULL;
	dst->mHalfSide				= &mHalfSide;
	dst->mProjectHull			= HullProjectionCB_Box;
	dst->mSelectClosestEdgeCB	= SelectClosestEdgeCB_Box;
}
