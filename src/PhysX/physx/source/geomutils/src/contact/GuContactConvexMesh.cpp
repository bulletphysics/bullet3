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
            
#include "GuConvexUtilsInternal.h"
#include "GuInternal.h"
#include "GuContactPolygonPolygon.h"
#include "GuConvexEdgeFlags.h"
#include "GuSeparatingAxes.h"
#include "GuContactMethodImpl.h"
#include "GuContactBuffer.h"
#include "GuMidphaseInterface.h"
#include "GuConvexHelper.h"
#include "GuTriangleCache.h"
#include "GuHeightFieldUtil.h"
#include "GuEntityReport.h"
#include "GuGeometryUnion.h"
#include "GuIntersectionTriangleBox.h"
#include "CmUtils.h"
#include "PsAllocator.h"
#include "GuBox.h"
#include "PsFPU.h"

using namespace physx;
using namespace Gu;
using namespace shdfnd::aos;
using namespace intrinsics;

#define LOCAL_TOUCHED_TRIG_SIZE 192

//#define USE_TRIANGLE_NORMAL
#define TEST_INTERNAL_OBJECTS

static PX_FORCE_INLINE void projectTriangle(const PxVec3& localSpaceDirection, const PxVec3* PX_RESTRICT triangle, PxReal& min1, PxReal& max1)
{
	const PxReal dp0 = triangle[0].dot(localSpaceDirection);
	const PxReal dp1 = triangle[1].dot(localSpaceDirection);
	min1 = selectMin(dp0, dp1);
	max1 = selectMax(dp0, dp1);

	const PxReal dp2 = triangle[2].dot(localSpaceDirection);
	min1 = selectMin(min1, dp2);
	max1 = selectMax(max1, dp2);
}

#ifdef TEST_INTERNAL_OBJECTS

static PX_FORCE_INLINE void boxSupport(const float extents[3], const PxVec3& sv, float p[3])
{
	const PxU32* iextents = reinterpret_cast<const PxU32*>(extents);
	const PxU32* isv = reinterpret_cast<const PxU32*>(&sv);
	PxU32* ip = reinterpret_cast<PxU32*>(p);

	ip[0] = iextents[0]|(isv[0]&PX_SIGN_BITMASK);
	ip[1] = iextents[1]|(isv[1]&PX_SIGN_BITMASK);
	ip[2] = iextents[2]|(isv[2]&PX_SIGN_BITMASK);
}

#if PX_DEBUG
static const PxReal testInternalObjectsEpsilon = 1.0e-3f;
#endif

static PX_FORCE_INLINE bool testInternalObjects(const PxVec3& localAxis0,
												const PolygonalData& polyData0,
												const PxVec3* PX_RESTRICT triangleInHullSpace,
												float dmin)
{
	PxReal min1, max1;
	projectTriangle(localAxis0, triangleInHullSpace, min1, max1);

	const float dp = polyData0.mCenter.dot(localAxis0);

	float p0[3];
	boxSupport(polyData0.mInternal.mExtents, localAxis0, p0);
	const float Radius0 = p0[0]*localAxis0.x + p0[1]*localAxis0.y + p0[2]*localAxis0.z;
	const float bestRadius = selectMax(Radius0, polyData0.mInternal.mRadius);
	const PxReal min0 = dp - bestRadius;
	const PxReal max0 = dp + bestRadius;

	const PxReal d0 = max0 - min1;
	const PxReal d1 = max1 - min0;

	const float depth = selectMin(d0, d1);
	if(depth>dmin)
		return false;
	return true;
}
#endif

static PX_FORCE_INLINE bool testNormal(	const PxVec3& sepAxis, PxReal min0, PxReal max0,
										const PxVec3* PX_RESTRICT triangle,
										PxReal& depth, PxReal contactDistance)
{
	PxReal min1, max1;
	projectTriangle(sepAxis, triangle, min1, max1);

	if(max0+contactDistance<min1 || max1+contactDistance<min0)
		return false;

	const PxReal d0 = max0 - min1;
	const PxReal d1 = max1 - min0;
	depth = selectMin(d0, d1);
	return true;
}

static PX_FORCE_INLINE bool testSepAxis(const PxVec3& sepAxis,
										const PolygonalData& polyData0,
										const PxVec3* PX_RESTRICT triangle,
										const Cm::Matrix34& m0to1,
										const Cm::FastVertex2ShapeScaling& convexScaling,
										PxReal& depth, PxReal contactDistance)
{
	PxReal min0, max0;
	(polyData0.mProjectHull)(polyData0, sepAxis, m0to1, convexScaling, min0, max0);

	PxReal min1, max1;
	projectTriangle(sepAxis, triangle, min1, max1);

	if(max0+contactDistance < min1 || max1+contactDistance < min0)
		return false;

	const PxReal d0 = max0 - min1;
	const PxReal d1 = max1 - min0;
	depth = selectMin(d0, d1);
	return true;
}

static bool testFacesSepAxesBackface(	const PolygonalData& polyData0,
										const Cm::Matrix34& /*world0*/, const Cm::Matrix34& /*world1*/, 
										const Cm::Matrix34& m0to1, const PxVec3& witness, 
										const PxVec3* PX_RESTRICT triangle,
										const Cm::FastVertex2ShapeScaling& convexScaling,
										PxU32& numHullIndices,
										PxU32* hullIndices_, PxReal& dmin, PxVec3& sep, PxU32& id, PxReal contactDistance,
										bool idtConvexScale
										)
{
	id = PX_INVALID_U32;

	const PxU32 numHullPolys = polyData0.mNbPolygons;
	const HullPolygonData* PX_RESTRICT polygons = polyData0.mPolygons;
	const PxVec3* PX_RESTRICT vertices = polyData0.mVerts;
	const PxVec3& trans = m0to1.p;
	{
		PxU32* hullIndices = hullIndices_;

		// PT: when the center of one object is inside the other object (deep penetrations) this discards everything!
		// PT: when this happens, the backup procedure is used to come up with good results anyway.
		// PT: it's worth having a special codepath here for identity scales, to skip all the normalizes/division. Lot faster without.

		if(idtConvexScale)
		{
			for(PxU32 i=0; i<numHullPolys; i++)
			{
				const HullPolygonData& P = polygons[i];
				const PxPlane& PL = P.mPlane;

#ifdef USE_TRIANGLE_NORMAL
				if(PL.normal.dot(witness) > 0.0f)
#else
		// ### this is dubious since the triangle center is likely to be very close to the hull, if not inside. Why not use the triangle normal?
				if(PL.distance(witness) < 0.0f)
#endif
					continue; //backface culled

				*hullIndices++ = i;

				const PxVec3 sepAxis = m0to1.rotate(PL.n);
				const PxReal dp = sepAxis.dot(trans);

				PxReal d;
				if(!testNormal(sepAxis, P.getMin(vertices) + dp, P.getMax() + dp, triangle,
					d, contactDistance))
					return false;

				if(d < dmin)
				{
					dmin = d;
					sep = sepAxis;
					id = i;
				}
			}
		}
		else
		{
#ifndef USE_TRIANGLE_NORMAL
		//transform delta from hull0 shape into vertex space:
			const PxVec3 vertSpaceWitness = convexScaling % witness;
#endif
			for(PxU32 i=0; i<numHullPolys; i++)
			{
				const HullPolygonData& P = polygons[i];
				const PxPlane& PL = P.mPlane;

#ifdef USE_TRIANGLE_NORMAL
				if(PL.normal.dot(witness) > 0.0f)
#else
		// ### this is dubious since the triangle center is likely to be very close to the hull, if not inside. Why not use the triangle normal?
				if(PL.distance(vertSpaceWitness) < 0.0f)
#endif
					continue; //backface culled

				//normals transform by inverse transpose: (and transpose(skew) == skew as its symmetric)
				PxVec3 shapeSpaceNormal = convexScaling % PL.n;
				//renormalize: (Arr!)
				const PxReal magnitude = shapeSpaceNormal.normalize();

				*hullIndices++ = i;
				const PxVec3 sepAxis = m0to1.rotate(shapeSpaceNormal);
				PxReal d;
				const PxReal dp = sepAxis.dot(trans);

				const float oneOverM = 1.0f / magnitude;
				if(!testNormal(sepAxis, P.getMin(vertices) * oneOverM + dp, P.getMax() * oneOverM + dp, triangle,
					d, contactDistance))
					return false;

				if(d < dmin)
				{
					dmin = d;
					sep = sepAxis;
					id = i;
				}
			}
		}
		numHullIndices = PxU32(hullIndices - hullIndices_);
	}

	// Backup
	if(id == PX_INVALID_U32)
	{
		if(idtConvexScale)
		{
			for(PxU32 i=0; i<numHullPolys; i++)
			{
				const HullPolygonData& P = polygons[i];
				const PxPlane& PL = P.mPlane;

				const PxVec3 sepAxis = m0to1.rotate(PL.n);
				const PxReal dp = sepAxis.dot(trans);

				PxReal d;
				if(!testNormal(sepAxis, P.getMin(vertices) + dp, P.getMax() + dp, triangle,
					d, contactDistance))
					return false;

				if(d < dmin)
				{
					dmin = d;
					sep = sepAxis;
					id = i;
				}
				hullIndices_[i] = i;
			}
		}
		else
		{
			for(PxU32 i=0; i<numHullPolys; i++)
			{
				const HullPolygonData& P = polygons[i];
				const PxPlane& PL = P.mPlane;

				PxVec3 shapeSpaceNormal = convexScaling % PL.n;
				//renormalize: (Arr!)
				const PxReal magnitude = shapeSpaceNormal.normalize();

				const PxVec3 sepAxis = m0to1.rotate(shapeSpaceNormal);

				PxReal d;
				const PxReal dp = sepAxis.dot(trans);
			
				const float oneOverM = 1.0f / magnitude;
				if(!testNormal(sepAxis, P.getMin(vertices) * oneOverM + dp, P.getMax() * oneOverM + dp, triangle,
					d, contactDistance))
					return false;

				if(d < dmin)
				{
					dmin = d;
					sep = sepAxis;
					id = i;
				}
				hullIndices_[i] = i;
			}
		}
		numHullIndices = numHullPolys;
	}
	return true;
}

static PX_FORCE_INLINE bool edgeCulling(const PxPlane& plane, const PxVec3& p0, const PxVec3& p1, PxReal contactDistance)
{
	return plane.distance(p0)<=contactDistance || plane.distance(p1)<=contactDistance;
}

static bool performEETests(
						const PolygonalData& polyData0,
						const PxU8 triFlags,
						const Cm::Matrix34& m0to1, const Cm::Matrix34& m1to0,
						const PxVec3* PX_RESTRICT triangle,
						PxU32 numHullIndices, const PxU32* PX_RESTRICT hullIndices,
						const PxPlane& localTriPlane,
						const Cm::FastVertex2ShapeScaling& convexScaling,
						PxVec3& vec, PxReal& dmin, PxReal contactDistance, PxReal toleranceLength,
						PxU32 id0, PxU32 /*triangleIndex*/)
{
	PX_UNUSED(toleranceLength);	// Only used in Debug

	// Cull candidate triangle edges vs to hull plane
	PxU32 nbTriangleAxes = 0;
	PxVec3 triangleAxes[3];
	{
		const HullPolygonData& P = polyData0.mPolygons[id0];
		const PxPlane& vertSpacePlane = P.mPlane;

		const PxVec3 newN = m1to0.rotate(vertSpacePlane.n);
		PxPlane hullWitness(convexScaling * newN, vertSpacePlane.d - m1to0.p.dot(newN)); //technically not a fully xformed plane, just use property of x|My == Mx|y for symmetric M.
		
		if((triFlags & ETD_CONVEX_EDGE_01) && edgeCulling(hullWitness, triangle[0], triangle[1], contactDistance))
			triangleAxes[nbTriangleAxes++] = (triangle[0] - triangle[1]);

		if((triFlags & ETD_CONVEX_EDGE_12) && edgeCulling(hullWitness, triangle[1], triangle[2], contactDistance))
			triangleAxes[nbTriangleAxes++] = (triangle[1] - triangle[2]);

		if((triFlags & ETD_CONVEX_EDGE_20) && edgeCulling(hullWitness, triangle[2], triangle[0], contactDistance))
			triangleAxes[nbTriangleAxes++] = (triangle[2] - triangle[0]);
	}

	//PxcPlane vertexSpacePlane = localTriPlane.getTransformed(m1to0);
	//vertexSpacePlane.normal = convexScaling * vertexSpacePlane.normal;	//technically not a fully xformed plane, just use property of x|My == Mx|y for symmetric M.
	const PxVec3 newN = m1to0.rotate(localTriPlane.n);
	PxPlane vertexSpacePlane(convexScaling * newN, localTriPlane.d - m1to0.p.dot(newN));

	const PxVec3* PX_RESTRICT hullVerts = polyData0.mVerts;

	SeparatingAxes SA;
	SA.reset();

	const PxU8* PX_RESTRICT vrefBase0 = polyData0.mPolygonVertexRefs;
	const HullPolygonData* PX_RESTRICT polygons = polyData0.mPolygons;
	while(numHullIndices--)
	{
		const HullPolygonData& P = polygons[*hullIndices++];
		const PxU8* PX_RESTRICT data = vrefBase0 + P.mVRef8;

		PxU32 numEdges = nbTriangleAxes;
		const PxVec3* edges = triangleAxes;

		// TODO: cheap edge culling as in convex/convex!
		while(numEdges--)
		{
			const PxVec3& currentPolyEdge = *edges++;

			// Loop through polygon vertices == polygon edges;
			PxU32 numVerts = P.mNbVerts;
			for(PxU32 j = 0; j < numVerts; j++)
			{
				PxU32 j1 = j+1;
				if(j1>=numVerts) j1 = 0;

				const PxU32 VRef0 = data[j];
				const PxU32 VRef1 = data[j1];

				if(edgeCulling(vertexSpacePlane, hullVerts[VRef0], hullVerts[VRef1], contactDistance))
				{
					const PxVec3 currentHullEdge = m0to1.rotate(convexScaling * (hullVerts[VRef0] - hullVerts[VRef1]));	//matrix mult is distributive!

					PxVec3 sepAxis = currentHullEdge.cross(currentPolyEdge);
					if(!Ps::isAlmostZero(sepAxis))
						SA.addAxis(sepAxis.getNormalized());
				}
			}
		}
	}

	dmin = PX_MAX_REAL;
	PxU32 numAxes = SA.getNumAxes();
	const PxVec3* PX_RESTRICT axes = SA.getAxes();

#ifdef TEST_INTERNAL_OBJECTS
	PxVec3 triangleInHullSpace[3];
	if(numAxes)
	{
		triangleInHullSpace[0] = m1to0.transform(triangle[0]);
		triangleInHullSpace[1] = m1to0.transform(triangle[1]);
		triangleInHullSpace[2] = m1to0.transform(triangle[2]);
	}
#endif

	while(numAxes--)
	{
		const PxVec3& currentAxis = *axes++;

#ifdef TEST_INTERNAL_OBJECTS
		const PxVec3 localAxis0 = m1to0.rotate(currentAxis);
		if(!testInternalObjects(localAxis0, polyData0, triangleInHullSpace, dmin))
		{
	#if PX_DEBUG
			PxReal dtest;
			if(testSepAxis(currentAxis, polyData0, triangle, m0to1, convexScaling, dtest, contactDistance))
			{
				PX_ASSERT(dtest + testInternalObjectsEpsilon*toleranceLength >= dmin);
			}
	#endif
			continue;
		}
#endif

		PxReal d;
		if(!testSepAxis(currentAxis, polyData0, triangle,
			m0to1, convexScaling, d, contactDistance))
		{
			return false;
		}

		if(d < dmin)
		{
			dmin = d;
			vec = currentAxis;
		}
	}
	return true;
}

static bool triangleConvexTest(	const PolygonalData& polyData0,
								const PxU8 triFlags,
								PxU32 index, const PxVec3* PX_RESTRICT localPoints,
								const PxPlane& localPlane,
								const PxVec3& groupCenterHull,
								const Cm::Matrix34& world0, const Cm::Matrix34& world1, const Cm::Matrix34& m0to1, const Cm::Matrix34& m1to0,
								const Cm::FastVertex2ShapeScaling& convexScaling,
								PxReal contactDistance, PxReal toleranceLength,
								PxVec3& groupAxis, PxReal& groupMinDepth, bool& faceContact,
								bool idtConvexScale
								)
{
	PxU32 id0 = PX_INVALID_U32;
	PxReal dmin0 = PX_MAX_REAL;
	PxVec3 vec0;

	PxU32 numHullIndices = 0;
	PxU32* PX_RESTRICT const hullIndices = reinterpret_cast<PxU32*>(PxAlloca(polyData0.mNbPolygons*sizeof(PxU32)));

	// PT: we test the hull normals first because they don't need any hull projection. If we can early exit thanks
	// to those, we completely avoid all hull projections.
	bool status = testFacesSepAxesBackface(polyData0, world0, world1, m0to1, groupCenterHull, localPoints,
		convexScaling, numHullIndices, hullIndices, dmin0, vec0, id0, contactDistance, idtConvexScale);
	if(!status)
		return false;

	groupAxis = PxVec3(0);
	groupMinDepth = PX_MAX_REAL;

	const PxReal eps = 0.0001f; // DE7748

	//Test in mesh-space
	PxVec3 sepAxis;
	PxReal depth;
	
	{
		// Test triangle normal
		PxReal d;
		if(!testSepAxis(localPlane.n, polyData0, localPoints,
			m0to1, convexScaling, d, contactDistance))
			return false;

		if(d<dmin0+eps)
//		if(d<dmin0)
		{
			depth = d;
			sepAxis = localPlane.n;
			faceContact = true;
		}
		else
		{
			depth = dmin0;
			sepAxis = vec0;
			faceContact = false;
		}
	}

	if(depth < groupMinDepth)
	{
		groupMinDepth = depth;
		groupAxis = world1.rotate(sepAxis);
	}

	if(!performEETests(polyData0, triFlags, m0to1, m1to0,
		localPoints,
		numHullIndices, hullIndices, localPlane,
		convexScaling, sepAxis, depth, contactDistance, toleranceLength,
		id0, index))
		return false;

	if(depth < groupMinDepth)
	{
		groupMinDepth = depth;
		groupAxis = world1.rotate(sepAxis);
		faceContact = false;
	}

	return true;
}

namespace
{
	struct ConvexMeshContactGeneration
	{
		Ps::InlineArray<PxU32,LOCAL_CONTACTS_SIZE>&	mDelayedContacts;
		Gu::CacheMap<Gu::CachedEdge, 128>			mEdgeCache;
		Gu::CacheMap<Gu::CachedVertex, 128>			mVertCache;

		const Cm::Matrix34							m0to1;
		const Cm::Matrix34							m1to0;

		PxVec3										mHullCenterMesh;
		PxVec3										mHullCenterWorld;

		const PolygonalData&						mPolyData0;
		const Cm::Matrix34&							mWorld0;
		const Cm::Matrix34&							mWorld1;

		const Cm::FastVertex2ShapeScaling&			mConvexScaling;

		PxReal										mContactDistance;
		PxReal										mToleranceLength;
		bool										mIdtMeshScale, mIdtConvexScale;
		PxReal										mCCDEpsilon;
		const PxTransform&							mTransform0;
		const PxTransform&							mTransform1;
		ContactBuffer&								mContactBuffer;
		bool										mAnyHits;

		ConvexMeshContactGeneration(
			Ps::InlineArray<PxU32,LOCAL_CONTACTS_SIZE>& delayedContacts,
			const PxTransform& t0to1, const PxTransform& t1to0,
			const PolygonalData& polyData0, const Cm::Matrix34& world0, const Cm::Matrix34& world1,
			const Cm::FastVertex2ShapeScaling& convexScaling,
			PxReal contactDistance,
			PxReal toleranceLength,
			bool idtConvexScale,
			PxReal cCCDEpsilon,
			const PxTransform& transform0, const PxTransform& transform1,
			ContactBuffer& contactBuffer
		);

		void processTriangle(const PxVec3* verts, PxU32 triangleIndex, PxU8 triFlags, const PxU32* vertInds);
		void generateLastContacts();

		bool	generateContacts(
			const PxPlane& localPlane,
			const PxVec3* PX_RESTRICT localPoints,
			const PxVec3& triCenter, PxVec3& groupAxis,
			PxReal groupMinDepth, PxU32 index)	const;

	private:
		ConvexMeshContactGeneration& operator=(const ConvexMeshContactGeneration&);
	};

// 17 entries. 1088/17 = 64 triangles in the local array
struct SavedContactData
{
	PxU32		mTriangleIndex;
	PxVec3		mVerts[3];
	PxU32		mInds[3];
	PxVec3		mGroupAxis;
	PxReal		mGroupMinDepth;
};
}

ConvexMeshContactGeneration::ConvexMeshContactGeneration(
	Ps::InlineArray<PxU32,LOCAL_CONTACTS_SIZE>& delayedContacts,
	const PxTransform& t0to1, const PxTransform& t1to0,
	const PolygonalData& polyData0, const Cm::Matrix34& world0, const Cm::Matrix34& world1,
	const Cm::FastVertex2ShapeScaling& convexScaling,
	PxReal contactDistance,
	PxReal toleranceLength,
	bool idtConvexScale,
	PxReal cCCDEpsilon,
	const PxTransform& transform0, const PxTransform& transform1,
	ContactBuffer& contactBuffer
) :
	mDelayedContacts(delayedContacts),
	m0to1			(t0to1),
	m1to0			(t1to0),
	mPolyData0		(polyData0),
	mWorld0			(world0),
	mWorld1			(world1),
	mConvexScaling	(convexScaling),
	mContactDistance(contactDistance),
	mToleranceLength(toleranceLength),
	mIdtConvexScale	(idtConvexScale),
	mCCDEpsilon		(cCCDEpsilon),
	mTransform0		(transform0),
	mTransform1		(transform1),
	mContactBuffer	(contactBuffer)
{
	delayedContacts.forceSize_Unsafe(0);
	mAnyHits = false;

	// Hull center in local space
	const PxVec3& hullCenterLocal = mPolyData0.mCenter;
	// Hull center in mesh space
	mHullCenterMesh = m0to1.transform(hullCenterLocal); 
	// Hull center in world space
	mHullCenterWorld = mWorld0.transform(hullCenterLocal);
}

struct ConvexMeshContactGenerationCallback : MeshHitCallback<PxRaycastHit>
{
	ConvexMeshContactGeneration			mGeneration;
	const Cm::FastVertex2ShapeScaling&	mMeshScaling;
	const PxU8* PX_RESTRICT				mExtraTrigData;
	bool								mIdtMeshScale;
	const TriangleMesh*					mMeshData;
	const BoxPadded&					mBox;

	ConvexMeshContactGenerationCallback(
		Ps::InlineArray<PxU32,LOCAL_CONTACTS_SIZE>& delayedContacts,
		const PxTransform& t0to1, const PxTransform& t1to0,
		const PolygonalData& polyData0, const Cm::Matrix34& world0, const Cm::Matrix34& world1,
		const TriangleMesh* meshData,
		const PxU8* PX_RESTRICT extraTrigData,
		const Cm::FastVertex2ShapeScaling& meshScaling,
		const Cm::FastVertex2ShapeScaling& convexScaling,
		PxReal contactDistance,
		PxReal toleranceLength,
		bool idtMeshScale, bool idtConvexScale,
		PxReal cCCDEpsilon,
		const PxTransform& transform0, const PxTransform& transform1,
		ContactBuffer& contactBuffer,
		const BoxPadded& box
	)	:
		MeshHitCallback<PxRaycastHit>	(CallbackMode::eMULTIPLE),
		mGeneration						(delayedContacts, t0to1, t1to0, polyData0, world0, world1, convexScaling, contactDistance, toleranceLength, idtConvexScale, cCCDEpsilon, transform0, transform1, contactBuffer),
		mMeshScaling					(meshScaling),
		mExtraTrigData					(extraTrigData),
		mIdtMeshScale					(idtMeshScale),
		mMeshData						(meshData),
		mBox							(box)
	{
	}

	virtual PxAgain processHit( // all reported coords are in mesh local space including hit.position
		const PxRaycastHit& hit, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, PxReal&, const PxU32* vinds)
	{
		// PT: this one is safe because incoming vertices from midphase are always safe to V4Load (by design)
		// PT: TODO: is this test really needed? Not done in midphase already?
		if(!intersectTriangleBox(mBox, v0, v1, v2))
			return true;

		PxVec3 verts[3];
		getScaledVertices(verts, v0, v1, v2, mIdtMeshScale, mMeshScaling);

		const PxU32 triangleIndex = hit.faceIndex;

		const PxU8 extraData = getConvexEdgeFlags(mExtraTrigData, triangleIndex);
		mGeneration.processTriangle(verts, triangleIndex, extraData, vinds);
		return true;
	}

protected:
	ConvexMeshContactGenerationCallback &operator=(const ConvexMeshContactGenerationCallback &);
};

bool ConvexMeshContactGeneration::generateContacts(
	const PxPlane& localPlane,
	const PxVec3* PX_RESTRICT localPoints,
	const PxVec3& triCenter, PxVec3& groupAxis,
	PxReal groupMinDepth, PxU32 index) const
{
	const PxVec3 worldGroupCenter = mWorld1.transform(triCenter);
	const PxVec3 deltaC = mHullCenterWorld - worldGroupCenter;
	if(deltaC.dot(groupAxis) < 0.0f)
		groupAxis = -groupAxis;

	const PxU32 id = (mPolyData0.mSelectClosestEdgeCB)(mPolyData0, mConvexScaling, mWorld0.rotateTranspose(-groupAxis));

	const HullPolygonData& HP = mPolyData0.mPolygons[id];
	PX_ALIGN(16, PxPlane) shapeSpacePlane0;
	if(mIdtConvexScale)
		V4StoreA(V4LoadU(&HP.mPlane.n.x), &shapeSpacePlane0.n.x);
	else
		mConvexScaling.transformPlaneToShapeSpace(HP.mPlane.n, HP.mPlane.d, shapeSpacePlane0.n, shapeSpacePlane0.d);

	const PxVec3 hullNormalWorld = mWorld0.rotate(shapeSpacePlane0.n);

	const PxReal d0 = PxAbs(hullNormalWorld.dot(groupAxis));

	const PxVec3 triNormalWorld = mWorld1.rotate(localPlane.n);
	const PxReal d1 = PxAbs(triNormalWorld.dot(groupAxis));
	const bool d0biggerd1 = d0 > d1;

////////////////////NEW DIST HANDLING//////////////////////
	//TODO: skip this if there is no dist involved!

PxReal separation = - groupMinDepth;	//convert to real distance. 

separation = fsel(separation, separation, 0.0f);	//don't do anything when penetrating!

//printf("\nseparation = %f", separation);

PxReal contactGenPositionShift = separation + mCCDEpsilon;	//if we're at a distance, shift so we're within penetration.

PxVec3 contactGenPositionShiftVec = groupAxis * contactGenPositionShift;	//shift one of the bodies this distance toward the other just for Pierre's contact generation.  Then the bodies should be penetrating exactly by MIN_SEPARATION_FOR_PENALTY - ideal conditions for this contact generator.

//note: for some reason this has to change sign!

//this will make contact gen always generate contacts at about MSP.  Shift them back to the true real distance, and then to a solver compliant distance given that 
//the solver converges to MSP penetration, while we want it to converge to 0 penetration.
//to real distance:
//		PxReal polyPolySeparationShift = separation; //(+ or - depending on which way normal goes)

//The system:  We always shift convex 0 (arbitrary).  If the contact is attached to convex 0 then we will need to shift the contact point, otherwise not.

//TODO: make these overwrite orig location if its safe to do so.

Cm::Matrix34 world0_(mWorld0);
PxTransform transform0_(mTransform0);

world0_.p -= contactGenPositionShiftVec;
transform0_.p = world0_.p;	//reset this too.

PxTransform t0to1_ = mTransform1.transformInv(transform0_);
PxTransform t1to0_ = transform0_.transformInv(mTransform1);
Cm::Matrix34 m0to1_(t0to1_);	
Cm::Matrix34 m1to0_(t1to0_);

	PxVec3* scaledVertices0;
	PxU8* stackIndices0;
	GET_SCALEX_CONVEX(scaledVertices0, stackIndices0, mIdtConvexScale, HP.mNbVerts, mConvexScaling, mPolyData0.mVerts, mPolyData0.getPolygonVertexRefs(HP))

	const PxU8 indices[3] = {0, 1, 2};

	const PxMat33 RotT0 = findRotationMatrixFromZ(shapeSpacePlane0.n);
	const PxMat33 RotT1 = findRotationMatrixFromZ(localPlane.n);

	if(d0biggerd1)
	{
		if(contactPolygonPolygonExt(
			HP.mNbVerts, scaledVertices0, stackIndices0, world0_, shapeSpacePlane0,
			RotT0,
			3, localPoints, indices, mWorld1, localPlane,
			RotT1,
			hullNormalWorld, m0to1_, m1to0_, PXC_CONTACT_NO_FACE_INDEX, index,
			mContactBuffer,
			true,
			contactGenPositionShiftVec, contactGenPositionShift))
		{
			return true;
		}
	}
	else
	{
		if(contactPolygonPolygonExt(
			3, localPoints, indices, mWorld1, localPlane,
			RotT1,
			HP.mNbVerts, scaledVertices0, stackIndices0, world0_, shapeSpacePlane0,
			RotT0,
			triNormalWorld, m1to0_, m0to1_, PXC_CONTACT_NO_FACE_INDEX, index,
			mContactBuffer,
			false,
			contactGenPositionShiftVec, contactGenPositionShift))
		{
			return true;
		}
	}
	return false;
}

enum FeatureCode
{
	FC_VERTEX0,
	FC_VERTEX1,
	FC_VERTEX2,
	FC_EDGE01,
	FC_EDGE12,
	FC_EDGE20,
	FC_FACE,

	FC_UNDEFINED
};

static FeatureCode computeFeatureCode(const PxVec3& point, const PxVec3* verts)
{
	const PxVec3& triangleOrigin	= verts[0];
	const PxVec3 triangleEdge0		= verts[1] - verts[0];
	const PxVec3 triangleEdge1		= verts[2] - verts[0];

	const PxVec3 kDiff	= triangleOrigin - point;
	const PxReal fA00	= triangleEdge0.magnitudeSquared();
	const PxReal fA01	= triangleEdge0.dot(triangleEdge1);
	const PxReal fA11	= triangleEdge1.magnitudeSquared();
	const PxReal fB0	= kDiff.dot(triangleEdge0);
	const PxReal fB1	= kDiff.dot(triangleEdge1);
	const PxReal fDet	= PxAbs(fA00*fA11 - fA01*fA01);
	const PxReal u		= fA01*fB1-fA11*fB0;
	const PxReal v		= fA01*fB0-fA00*fB1;

	FeatureCode fc		= FC_UNDEFINED;

	if(u + v <= fDet)
	{
		if(u < 0.0f)
		{
			if(v < 0.0f)  // region 4
			{
				if(fB0 < 0.0f)
				{
					if(-fB0 >= fA00)
						fc = FC_VERTEX1;
					else
						fc = FC_EDGE01;
				}
				else
				{
					if(fB1 >= 0.0f)
						fc = FC_VERTEX0;
					else if(-fB1 >= fA11)
						fc = FC_VERTEX2;
					else
						fc = FC_EDGE20;
				}
			}
			else  // region 3
			{
				if(fB1 >= 0.0f)
					fc = FC_VERTEX0;
				else if(-fB1 >= fA11)
					fc = FC_VERTEX2;
				else
					fc = FC_EDGE20;
			}
		}
		else if(v < 0.0f)  // region 5
		{
			if(fB0 >= 0.0f)
				fc = FC_VERTEX0;
			else if(-fB0 >= fA00)
				fc = FC_VERTEX1;
			else
				fc = FC_EDGE01;
		}
		else  // region 0
		{
			// minimum at interior PxVec3
			if(fDet==0.0f)
				fc = FC_VERTEX0;
			else
				fc = FC_FACE;
		}
	}
	else
	{
		PxReal fTmp0, fTmp1, fNumer, fDenom;

		if(u < 0.0f)  // region 2
		{
			fTmp0 = fA01 + fB0;
			fTmp1 = fA11 + fB1;
			if(fTmp1 > fTmp0)
			{
				fNumer = fTmp1 - fTmp0;
				fDenom = fA00-2.0f*fA01+fA11;
				if(fNumer >= fDenom)
					fc = FC_VERTEX1;
				else
					fc = FC_EDGE12;
			}
			else
			{
				if(fTmp1 <= 0.0f)
					fc = FC_VERTEX2;
				else if(fB1 >= 0.0f)
					fc = FC_VERTEX0;
				else
					fc = FC_EDGE20;
			}
		}
		else if(v < 0.0f)  // region 6
		{
			fTmp0 = fA01 + fB1;
			fTmp1 = fA00 + fB0;
			if(fTmp1 > fTmp0)
			{
				fNumer = fTmp1 - fTmp0;
				fDenom = fA00-2.0f*fA01+fA11;
				if(fNumer >= fDenom)
					fc = FC_VERTEX2;
				else
					fc = FC_EDGE12;
			}
			else
			{
				if(fTmp1 <= 0.0f)
					fc = FC_VERTEX1;
				else if(fB0 >= 0.0f)
					fc = FC_VERTEX0;
				else
					fc = FC_EDGE01;
			}
		}
		else  // region 1
		{
			fNumer = fA11 + fB1 - fA01 - fB0;
			if(fNumer <= 0.0f)
			{
				fc = FC_VERTEX2;
			}
			else
			{
				fDenom = fA00-2.0f*fA01+fA11;
				if(fNumer >= fDenom)
					fc = FC_VERTEX1;
				else
					fc = FC_EDGE12;
			}
		}
	}
	return fc;
}


//static bool validateVertex(PxU32 vref, const PxU32 count, const ContactPoint* PX_RESTRICT contacts, const TriangleMesh& meshData)
//{
//	PxU32 previous = 0xffffffff;
//	for(PxU32 i=0;i<count;i++)
//	{
//		if(contacts[i].internalFaceIndex1==previous)
//			continue;
//		previous = contacts[i].internalFaceIndex1;
//
//		const TriangleIndices T(meshData, contacts[i].internalFaceIndex1);
//		if(		T.mVRefs[0]==vref
//			||	T.mVRefs[1]==vref
//			||	T.mVRefs[2]==vref)
//			return false;
//	}
//	return true;
//}


//static PX_FORCE_INLINE bool testEdge(PxU32 vref0, PxU32 vref1, PxU32 tvref0, PxU32 tvref1)
//{
//	if(tvref0>tvref1)
//		Ps::swap(tvref0, tvref1);
//
//	if(tvref0==vref0 && tvref1==vref1)
//		return false;
//	return true;
//}

//static bool validateEdge(PxU32 vref0, PxU32 vref1, const PxU32 count, const ContactPoint* PX_RESTRICT contacts, const TriangleMesh& meshData)
//{
//	if(vref0>vref1)
//		Ps::swap(vref0, vref1);
//
//	PxU32 previous = 0xffffffff;
//	for(PxU32 i=0;i<count;i++)
//	{
//		if(contacts[i].internalFaceIndex1==previous)
//			continue;
//		previous = contacts[i].internalFaceIndex1;
//
//		const TriangleIndices T(meshData, contacts[i].internalFaceIndex1);
//
///*		if(T.mVRefs[0]==vref0 || T.mVRefs[0]==vref1)
//			return false;
//		if(T.mVRefs[1]==vref0 || T.mVRefs[1]==vref1)
//			return false;
//		if(T.mVRefs[2]==vref0 || T.mVRefs[2]==vref1)
//			return false;*/
//		// PT: wow, this was wrong??? ###FIX
//		if(!testEdge(vref0, vref1, T.mVRefs[0], T.mVRefs[1]))
//			return false;
//		if(!testEdge(vref0, vref1, T.mVRefs[1], T.mVRefs[2]))
//			return false;
//		if(!testEdge(vref0, vref1, T.mVRefs[2], T.mVRefs[0]))
//			return false;
//	}
//	return true;
//}

//static bool validateEdge(PxU32 vref0, PxU32 vref1, const PxU32* vertIndices, const PxU32 nbIndices)
//{
//	if(vref0>vref1)
//		Ps::swap(vref0, vref1);
//
//	for(PxU32 i=0;i<nbIndices;i+=3)
//	{
//		if(!testEdge(vref0, vref1, vertIndices[i+0], vertIndices[i+1]))
//			return false;
//		if(!testEdge(vref0, vref1, vertIndices[i+1], vertIndices[i+2]))
//			return false;
//		if(!testEdge(vref0, vref1, vertIndices[i+2], vertIndices[i+0]))
//			return false;
//	}
//	return true;
//}

//#endif

void ConvexMeshContactGeneration::processTriangle(const PxVec3* verts, PxU32 triangleIndex, PxU8 triFlags, const PxU32* vertInds)
{
	const PxPlane localPlane(verts[0], verts[1], verts[2]);

	// Backface culling
	if(localPlane.distance(mHullCenterMesh)<0.0f)
//		if(localPlane.normal.dot(mHullCenterMesh - T.mVerts[0]) <= 0.0f)
		return;

	//////////////////////////////////////////////////////////////////////////

	const PxVec3 triCenter = (verts[0] + verts[1] + verts[2])*(1.0f/3.0f);

	// Group center in hull space
#ifdef USE_TRIANGLE_NORMAL
	const PxVec3 groupCenterHull = m1to0.rotate(localPlane.normal);
#else
	const PxVec3 groupCenterHull = m1to0.transform(triCenter);
#endif
	//////////////////////////////////////////////////////////////////////////

	PxVec3 groupAxis;
	PxReal groupMinDepth;
	bool faceContact;
	if(!triangleConvexTest(	mPolyData0, triFlags,
							triangleIndex, verts,
							localPlane,
							groupCenterHull,
							mWorld0, mWorld1, m0to1, m1to0,
							mConvexScaling,
							mContactDistance, mToleranceLength,
							groupAxis, groupMinDepth, faceContact,
							mIdtConvexScale
							))
		return;

	if(faceContact)
	{
		// PT: generate face contacts immediately to save memory & avoid recomputing triangle data later
		if(generateContacts(
			localPlane,
			verts,
			triCenter, groupAxis,
			groupMinDepth, triangleIndex))
		{
			mAnyHits = true;
			mEdgeCache.addData(CachedEdge(vertInds[0], vertInds[1]));
			mEdgeCache.addData(CachedEdge(vertInds[0], vertInds[2]));
			mEdgeCache.addData(CachedEdge(vertInds[1], vertInds[2]));
			mVertCache.addData(CachedVertex(vertInds[0]));
			mVertCache.addData(CachedVertex(vertInds[1]));
			mVertCache.addData(CachedVertex(vertInds[2]));
		}
		else
		{
			int stop=1;
			(void)stop;
		}
	}
	else
	{
		const PxU32 nb = sizeof(SavedContactData)/sizeof(PxU32);

		// PT: no "pushBack" please (useless data copy + LHS)	
		PxU32 newSize = nb + mDelayedContacts.size();
	    mDelayedContacts.reserve(newSize);
		SavedContactData* PX_RESTRICT cd = reinterpret_cast<SavedContactData*>(mDelayedContacts.end());
	    mDelayedContacts.forceSize_Unsafe(newSize);
		
		cd->mTriangleIndex	= triangleIndex;
		cd->mVerts[0]		= verts[0];
		cd->mVerts[1]		= verts[1];
		cd->mVerts[2]		= verts[2];
		cd->mInds[0]		= vertInds[0];
		cd->mInds[1]		= vertInds[1];
		cd->mInds[2]		= vertInds[2];
		cd->mGroupAxis		= groupAxis;
		cd->mGroupMinDepth	= groupMinDepth;
	}
}

void ConvexMeshContactGeneration::generateLastContacts()
{
	// Process delayed contacts
	PxU32 nbEntries = mDelayedContacts.size();
	if(nbEntries)
	{
		nbEntries /= sizeof(SavedContactData)/sizeof(PxU32);

		// PT: TODO: replicate this fix in sphere-vs-mesh ###FIX
		//const PxU32 count = mContactBuffer.count;
		//const ContactPoint* PX_RESTRICT contacts = mContactBuffer.contacts;

		const SavedContactData* PX_RESTRICT cd = reinterpret_cast<const SavedContactData*>(mDelayedContacts.begin());
		for(PxU32 i=0;i<nbEntries;i++)
		{
			const SavedContactData& currentContact = cd[i];

			const PxU32 triangleIndex = currentContact.mTriangleIndex;

			// PT: unfortunately we must recompute this triangle-data here.
			// PT: TODO: find a way not to
//			const TriangleVertices T(*mMeshData, mMeshScaling, triangleIndex);
//			const TriangleIndices T(*mMeshData, triangleIndex);

			const PxU32 ref0 = currentContact.mInds[0];
			const PxU32 ref1 = currentContact.mInds[1];
			const PxU32 ref2 = currentContact.mInds[2];

			// PT: TODO: why bother with the feature code at all? Use edge cache directly?
//			const FeatureCode FC = computeFeatureCode(mHullCenterMesh, T.mVerts);
			const FeatureCode FC = computeFeatureCode(mHullCenterMesh, currentContact.mVerts);

			bool generateContact = false;
			switch(FC)
			{
				// PT: trying the same as in sphere-mesh here
				case FC_VERTEX0:
					generateContact = !mVertCache.contains(CachedVertex(ref0));
					break;
				case FC_VERTEX1:
					generateContact =!mVertCache.contains(CachedVertex(ref1));
					break;
				case FC_VERTEX2:
					generateContact = !mVertCache.contains(CachedVertex(ref2));
					break;
				case FC_EDGE01:
					generateContact = !mEdgeCache.contains(CachedEdge(ref0, ref1));
					break;
				case FC_EDGE12:
					generateContact = !mEdgeCache.contains(CachedEdge(ref1, ref2));
					break;
				case FC_EDGE20:
					generateContact = !mEdgeCache.contains(CachedEdge(ref0, ref2));
					break;
				case FC_FACE:
					generateContact = true;
					break;
				case FC_UNDEFINED:
					break;
			};

			if(!generateContact)
				continue;

//			const PxcPlane localPlane(T.mVerts[0], T.mVerts[1], T.mVerts[2]);
			const PxPlane localPlane(currentContact.mVerts[0], currentContact.mVerts[1], currentContact.mVerts[2]);
//			const PxVec3 triCenter = (T.mVerts[0] + T.mVerts[1] + T.mVerts[2])*(1.0f/3.0f);
			const PxVec3 triCenter = (currentContact.mVerts[0] + currentContact.mVerts[1] + currentContact.mVerts[2])*(1.0f/3.0f);

			PxVec3 groupAxis = currentContact.mGroupAxis;
			if(generateContacts(
				localPlane,
//				T.mVerts,
				currentContact.mVerts,
				triCenter, groupAxis,
				currentContact.mGroupMinDepth, triangleIndex))
			{
				mAnyHits = true;

				//We don't add the edges to the data - this is important because we don't want to reject triangles
				//because we generated an edge contact with an adjacent triangle
				/*mEdgeCache.addData(CachedEdge(ref0, ref1));
				mEdgeCache.addData(CachedEdge(ref0, ref2));
				mEdgeCache.addData(CachedEdge(ref1, ref2));
				mVertCache.addData(CachedVertex(ref0));
				mVertCache.addData(CachedVertex(ref1));
				mVertCache.addData(CachedVertex(ref2));*/

			}
		}
	}
}

/////////////

static bool contactHullMesh2(const PolygonalData& polyData0, const PxBounds3& hullAABB, const PxTriangleMeshGeometryLL& shape1,
						const PxTransform& transform0, const PxTransform& transform1,
						const NarrowPhaseParams& params, ContactBuffer& contactBuffer,
						const Cm::FastVertex2ShapeScaling& convexScaling, const Cm::FastVertex2ShapeScaling& meshScaling,
						bool idtConvexScale, bool idtMeshScale)
{
	//Just a sanity-check in debug-mode
	PX_ASSERT(shape1.getType() == PxGeometryType::eTRIANGLEMESH);
	////////////////////

	// Compute matrices
	const Cm::Matrix34 world0(transform0);
	const Cm::Matrix34 world1(transform1);

	// Compute relative transforms
	const PxTransform t0to1 = transform1.transformInv(transform0);
	const PxTransform t1to0 = transform0.transformInv(transform1);

	BoxPadded hullOBB;
	computeHullOBB(hullOBB, hullAABB, params.mContactDistance, world0, world1, meshScaling, idtMeshScale);

	// Setup the collider
	const TriangleMesh* PX_RESTRICT meshData = shape1.meshData;

	Ps::InlineArray<PxU32,LOCAL_CONTACTS_SIZE> delayedContacts;
	
	ConvexMeshContactGenerationCallback blockCallback(
		delayedContacts,
		t0to1, t1to0, polyData0, world0, world1, meshData, meshData->getExtraTrigData(), meshScaling,
		convexScaling, params.mContactDistance, params.mToleranceLength,
		idtMeshScale, idtConvexScale, params.mMeshContactMargin,
		transform0, transform1,
		contactBuffer, hullOBB
	);

	Midphase::intersectOBB(meshData, hullOBB, blockCallback, false);

	blockCallback.mGeneration.generateLastContacts();
	
	return blockCallback.mGeneration.mAnyHits;
}

/////////////

bool Gu::contactConvexMesh(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(cache);
	PX_UNUSED(renderOutput);

	const PxTriangleMeshGeometryLL& shapeMesh = shape1.get<const PxTriangleMeshGeometryLL>();

	const bool idtScaleMesh = shapeMesh.scale.isIdentity();

	Cm::FastVertex2ShapeScaling meshScaling;
	if(!idtScaleMesh)
		meshScaling.init(shapeMesh.scale);

	Cm::FastVertex2ShapeScaling convexScaling;
	PxBounds3 hullAABB;
	PolygonalData polyData0;
	const bool idtScaleConvex = getConvexData(shape0, convexScaling, hullAABB, polyData0);

	return contactHullMesh2(polyData0, hullAABB, shapeMesh, transform0, transform1, params, contactBuffer, convexScaling, meshScaling, idtScaleConvex, idtScaleMesh);
}

bool Gu::contactBoxMesh(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(cache);
	PX_UNUSED(renderOutput);

	const PxBoxGeometry& shapeBox = shape0.get<const PxBoxGeometry>();
	const PxTriangleMeshGeometryLL& shapeMesh = shape1.get<const PxTriangleMeshGeometryLL>();

	PolygonalData polyData0;
	PolygonalBox polyBox(shapeBox.halfExtents);
	polyBox.getPolygonalData(&polyData0);

	const PxBounds3 hullAABB(-shapeBox.halfExtents, shapeBox.halfExtents);

	const bool idtScaleMesh = shapeMesh.scale.isIdentity();

	Cm::FastVertex2ShapeScaling meshScaling;
	if(!idtScaleMesh)
		meshScaling.init(shapeMesh.scale);

	Cm::FastVertex2ShapeScaling idtScaling;
	return contactHullMesh2(polyData0, hullAABB, shapeMesh, transform0, transform1, params, contactBuffer, idtScaling, meshScaling, true, idtScaleMesh);
}

/////////////

namespace
{
struct ConvexVsHeightfieldContactGenerationCallback : EntityReport<PxU32>
{
	ConvexMeshContactGeneration mGeneration;
	HeightFieldUtil&			mHfUtil;

	ConvexVsHeightfieldContactGenerationCallback(
		HeightFieldUtil& hfUtil,
		Ps::InlineArray<PxU32,LOCAL_CONTACTS_SIZE>& delayedContacts,
		const PxTransform& t0to1, const PxTransform& t1to0,
		const PolygonalData& polyData0, const Cm::Matrix34& world0, const Cm::Matrix34& world1,
		const Cm::FastVertex2ShapeScaling& convexScaling,
		PxReal contactDistance,
		PxReal toleranceLength,
		bool idtConvexScale,
		PxReal cCCDEpsilon,
		const PxTransform& transform0, const PxTransform& transform1,
		ContactBuffer& contactBuffer
		) : mGeneration(delayedContacts, t0to1, t1to0, polyData0, world0, world1, convexScaling, contactDistance, toleranceLength, idtConvexScale, cCCDEpsilon, transform0,
			transform1, contactBuffer),
			mHfUtil(hfUtil)
	{
	}

	// PT: TODO: refactor/unify with similar code in other places
	virtual bool onEvent(PxU32 nb, PxU32* indices)
	{
		const PxU8 nextInd[] = {2,0,1};

		while(nb--)
		{
			const PxU32 triangleIndex = *indices++;

			PxU32 vertIndices[3];
			PxTriangle currentTriangle;	// in world space
			PxU32 adjInds[3];
			mHfUtil.getTriangle(mGeneration.mTransform1, currentTriangle, vertIndices, adjInds, triangleIndex, false, false);

			PxVec3 normal;
			currentTriangle.normal(normal);

			PxU8 triFlags = 0; //KS - temporary until we can calculate triFlags for HF

			for(PxU32 a = 0; a < 3; ++a)
			{
				if(adjInds[a] != 0xFFFFFFFF)
				{
					PxTriangle adjTri;
					mHfUtil.getTriangle(mGeneration.mTransform1, adjTri, NULL, NULL, adjInds[a], false, false);
					//We now compare the triangles to see if this edge is active

					PxVec3 adjNormal;
					adjTri.denormalizedNormal(adjNormal);
					PxU32 otherIndex = nextInd[a];
					PxF32 projD = adjNormal.dot(currentTriangle.verts[otherIndex] - adjTri.verts[0]);
					if(projD < 0.f)
					{
						adjNormal.normalize();

						PxF32 proj = adjNormal.dot(normal);

						if(proj < 0.999f)
						{
							triFlags |= 1 << (a+3);
						}
					}
				}
				else
					triFlags |= (1 << (a+3));
			}
			mGeneration.processTriangle(currentTriangle.verts, triangleIndex, triFlags, vertIndices);
		}
		return true;
	}

protected:
	ConvexVsHeightfieldContactGenerationCallback &operator=(const ConvexVsHeightfieldContactGenerationCallback &);
};
}

static bool contactHullHeightfield2(const PolygonalData& polyData0, const PxBounds3& hullAABB, const PxHeightFieldGeometry& shape1,
						const PxTransform& transform0, const PxTransform& transform1,
						const NarrowPhaseParams& params, ContactBuffer& contactBuffer,
						const Cm::FastVertex2ShapeScaling& convexScaling,
						bool idtConvexScale)
{
	//We need to create a callback that fills triangles from the HF

	HeightFieldUtil hfUtil(shape1);

	const Cm::Matrix34 world0(transform0);
	const Cm::Matrix34 world1(transform1);

	////////////////////

	// Compute relative transforms
	const PxTransform t0to1 = transform1.transformInv(transform0);
	const PxTransform t1to0 = transform0.transformInv(transform1);

	Ps::InlineArray<PxU32, LOCAL_CONTACTS_SIZE> delayedContacts;

	ConvexVsHeightfieldContactGenerationCallback blockCallback(hfUtil, delayedContacts, t0to1, t1to0, polyData0, world0, world1, convexScaling, params.mContactDistance, params.mToleranceLength,
		idtConvexScale, params.mMeshContactMargin, transform0, transform1, contactBuffer);

	hfUtil.overlapAABBTriangles(transform1, PxBounds3::transformFast(t0to1, hullAABB), 0, &blockCallback);

	blockCallback.mGeneration.generateLastContacts();

	return blockCallback.mGeneration.mAnyHits;
}

bool Gu::contactConvexHeightfield(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(cache);
	PX_UNUSED(renderOutput);

	//Create a triangle cache from the HF triangles and then feed triangles to NP mesh methods
	const physx::PxHeightFieldGeometryLL& shapeMesh = shape1.get<const PxHeightFieldGeometryLL>();

	Cm::FastVertex2ShapeScaling convexScaling;
	PxBounds3 hullAABB;
	PolygonalData polyData0;
	const bool idtScaleConvex = getConvexData(shape0, convexScaling, hullAABB, polyData0);
	const PxVec3 inflation(params.mContactDistance);
	hullAABB.minimum -= inflation;
	hullAABB.maximum += inflation;

	return contactHullHeightfield2(polyData0, hullAABB, shapeMesh, transform0, transform1, params, contactBuffer, convexScaling, idtScaleConvex);
}

bool Gu::contactBoxHeightfield(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(cache);
	PX_UNUSED(renderOutput);

	//Create a triangle cache from the HF triangles and then feed triangles to NP mesh methods
	const physx::PxHeightFieldGeometryLL& shapeMesh = shape1.get<const PxHeightFieldGeometryLL>();

	const PxBoxGeometry& shapeBox = shape0.get<const PxBoxGeometry>();

	PolygonalData polyData0;
	PolygonalBox polyBox(shapeBox.halfExtents);
	polyBox.getPolygonalData(&polyData0);

	const PxVec3 inflatedExtents = shapeBox.halfExtents + PxVec3(params.mContactDistance);

	const PxBounds3 hullAABB = PxBounds3(-inflatedExtents, inflatedExtents);

	const Cm::FastVertex2ShapeScaling idtScaling;

	return contactHullHeightfield2(polyData0, hullAABB, shapeMesh, transform0, transform1, params, contactBuffer, idtScaling, true);
}
