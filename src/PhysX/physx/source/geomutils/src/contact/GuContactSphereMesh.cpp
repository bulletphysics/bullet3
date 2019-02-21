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

#include "GuDistancePointTriangle.h"
#include "GuContactMethodImpl.h"
#include "GuContactBuffer.h"
#include "GuGeometryUnion.h"
#include "GuFeatureCode.h"
#include "GuMidphaseInterface.h"
#include "GuEntityReport.h"
#include "GuHeightFieldUtil.h"
#include "GuBox.h"
#include "PsSort.h"

#include "CmRenderOutput.h"

#define DEBUG_RENDER_MESHCONTACTS	0

using namespace physx;
using namespace Gu;

static const bool gDrawTouchedTriangles = false;

static void outputErrorMessage()
{
#if PX_CHECKED
	Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Dropping contacts in sphere vs mesh: exceeded limit of 64 ");
#endif
}

///////////////////////////////////////////////////////////////////////////////

// PT: a customized version that also returns the feature code
static PxVec3 closestPtPointTriangle(const PxVec3& p, const PxVec3& a, const PxVec3& b, const PxVec3& c, float& s, float& t, FeatureCode& fc)
{
	// Check if P in vertex region outside A
	const PxVec3 ab = b - a;
	const PxVec3 ac = c - a;
	const PxVec3 ap = p - a;
	const float d1 = ab.dot(ap);
	const float d2 = ac.dot(ap);
	if(d1<=0.0f && d2<=0.0f)
	{
		s = 0.0f;
		t = 0.0f;
		fc = FC_VERTEX0;
		return a;	// Barycentric coords 1,0,0
	}

	// Check if P in vertex region outside B
	const PxVec3 bp = p - b;
	const float d3 = ab.dot(bp);
	const float d4 = ac.dot(bp);
	if(d3>=0.0f && d4<=d3)
	{
		s = 1.0f;
		t = 0.0f;
		fc = FC_VERTEX1;
		return b;	// Barycentric coords 0,1,0
	}

	// Check if P in edge region of AB, if so return projection of P onto AB
	const float vc = d1*d4 - d3*d2;
	if(vc<=0.0f && d1>=0.0f && d3<=0.0f)
	{
		const float v = d1 / (d1 - d3);
		s = v;
		t = 0.0f;
		fc = FC_EDGE01;
		return a + v * ab;	// barycentric coords (1-v, v, 0)
	}

	// Check if P in vertex region outside C
	const PxVec3 cp = p - c;
	const float d5 = ab.dot(cp);
	const float d6 = ac.dot(cp);
	if(d6>=0.0f && d5<=d6)
	{
		s = 0.0f;
		t = 1.0f;
		fc = FC_VERTEX2;
		return c;	// Barycentric coords 0,0,1
	}

	// Check if P in edge region of AC, if so return projection of P onto AC
	const float vb = d5*d2 - d1*d6;
	if(vb<=0.0f && d2>=0.0f && d6<=0.0f)
	{
		const float w = d2 / (d2 - d6);
		s = 0.0f;
		t = w;
		fc = FC_EDGE20;
		return a + w * ac;	// barycentric coords (1-w, 0, w)
	}

	// Check if P in edge region of BC, if so return projection of P onto BC
	const float va = d3*d6 - d5*d4;
	if(va<=0.0f && (d4-d3)>=0.0f && (d5-d6)>=0.0f)
	{
		const float w = (d4-d3) / ((d4 - d3) + (d5-d6));
		s = 1.0f-w;
		t = w;
		fc = FC_EDGE12;
		return b + w * (c-b);	// barycentric coords (0, 1-w, w)
	}

	// P inside face region. Compute Q through its barycentric coords (u,v,w)
	const float denom = 1.0f / (va + vb + vc);
	const float v = vb * denom;
	const float w = vc * denom;
	s = v;
	t = w;
	fc = FC_FACE;
	return a + ab*v + ac*w;
}

///////////////////////////////////////////////////////////////////////////////

// PT: we use a separate structure to make sorting faster
struct SortKey
{
	float		mSquareDist;
	PxU32		mIndex;

	PX_FORCE_INLINE bool operator < (const SortKey& data) const
	{
		return mSquareDist < data.mSquareDist;
	}		
};

struct TriangleData
{
	PxVec3		mDelta;
	FeatureCode	mFC;
	PxU32		mTriangleIndex;
	PxU32		mVRef[3];
};

struct CachedTriangleIndices
{
	PxU32		mVRef[3];
};

static PX_FORCE_INLINE bool validateSquareDist(PxReal squareDist)
{
	return squareDist>0.0001f;
}

static bool validateEdge(PxU32 vref0, PxU32 vref1, const CachedTriangleIndices* cachedTris, PxU32 nbCachedTris)
{
	while(nbCachedTris--)
	{
		const CachedTriangleIndices& inds = *cachedTris++;
		const PxU32 vi0 = inds.mVRef[0];
		const PxU32 vi1 = inds.mVRef[1];
		const PxU32 vi2 = inds.mVRef[2];

		if(vi0==vref0)
		{
			if(vi1==vref1 || vi2==vref1)
				return false;
		}
		else if(vi1==vref0)
		{
			if(vi0==vref1 || vi2==vref1)
				return false;
		}
		else if(vi2==vref0)
		{
			if(vi1==vref1 || vi0==vref1)
				return false;
		}
	}
	return true;
}

static bool validateVertex(PxU32 vref, const CachedTriangleIndices* cachedTris, PxU32 nbCachedTris)
{
	while(nbCachedTris--)
	{
		const CachedTriangleIndices& inds = *cachedTris++;
		if(inds.mVRef[0]==vref || inds.mVRef[1]==vref || inds.mVRef[2]==vref)
			return false;
	}
	return true;
}

namespace
{
	class NullAllocator
	{
	public:
		PX_FORCE_INLINE NullAllocator()								{				}
		PX_FORCE_INLINE	void* allocate(size_t, const char*, int) 	{ return NULL;	}
		PX_FORCE_INLINE	void deallocate(void*)						{				}
	};

struct SphereMeshContactGeneration
{
	const PxSphereGeometry&	mShapeSphere;
	const PxTransform&		mTransform0;
	const PxTransform&		mTransform1;
	ContactBuffer&			mContactBuffer;
	const PxVec3&			mSphereCenterShape1Space;
	PxF32					mInflatedRadius2;
	PxU32					mNbDelayed;
	TriangleData			mSavedData[ContactBuffer::MAX_CONTACTS];
	SortKey					mSortKey[ContactBuffer::MAX_CONTACTS];
	PxU32					mNbCachedTris;
	CachedTriangleIndices	mCachedTris[ContactBuffer::MAX_CONTACTS];
	Cm::RenderOutput*		mRenderOutput;

	SphereMeshContactGeneration(const PxSphereGeometry& shapeSphere, const PxTransform& transform0, const PxTransform& transform1,
		ContactBuffer& contactBuffer, const PxVec3& sphereCenterShape1Space, PxF32 inflatedRadius,
		Cm::RenderOutput* renderOutput) :
		mShapeSphere				(shapeSphere),
		mTransform0					(transform0),
		mTransform1					(transform1),
		mContactBuffer				(contactBuffer),
		mSphereCenterShape1Space	(sphereCenterShape1Space),
		mInflatedRadius2			(inflatedRadius*inflatedRadius),
		mNbDelayed					(0),
		mNbCachedTris				(0),
		mRenderOutput				(renderOutput)
	{
	}

	PX_FORCE_INLINE void cacheTriangle(PxU32 ref0, PxU32 ref1, PxU32 ref2)
	{
		const PxU32 nb = mNbCachedTris++;
		mCachedTris[nb].mVRef[0] = ref0;
		mCachedTris[nb].mVRef[1] = ref1;
		mCachedTris[nb].mVRef[2] = ref2;
	}

	PX_FORCE_INLINE void addContact(const PxVec3& d, PxReal squareDist, PxU32 triangleIndex)
	{
		float dist;
		PxVec3 delta;
		if(validateSquareDist(squareDist))
		{
			// PT: regular contact. Normalize 'delta'.
			dist = PxSqrt(squareDist);
			delta = d / dist;
		}
		else
		{
			// PT: singular contact: 'd' is the non-unit triangle's normal in this case.
			dist = 0.0f;
			delta = -d.getNormalized();
		}

		const PxVec3 worldNormal = -mTransform1.rotate(delta);

		const PxVec3 localHit = mSphereCenterShape1Space + mShapeSphere.radius*delta;
		const PxVec3 hit = mTransform1.transform(localHit);

		if(!mContactBuffer.contact(hit, worldNormal, dist - mShapeSphere.radius, triangleIndex))
			outputErrorMessage();
	}

	void processTriangle(PxU32 triangleIndex, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, const PxU32* vertInds)
	{
		// PT: compute closest point between sphere center and triangle
		PxReal u, v;
		FeatureCode fc;
		const PxVec3 cp = closestPtPointTriangle(mSphereCenterShape1Space, v0, v1, v2, u, v, fc);

		// PT: compute 'delta' vector between closest point and sphere center
		const PxVec3 delta = cp - mSphereCenterShape1Space;
		const PxReal squareDist = delta.magnitudeSquared();
		if(squareDist >= mInflatedRadius2)
			return;

		// PT: backface culling without the normalize
		// PT: TODO: consider doing before the pt-triangle distance test if it's cheaper
		// PT: TODO: e0/e1 already computed in closestPtPointTriangle
		const PxVec3 e0 = v1 - v0;
		const PxVec3 e1 = v2 - v0;
		const PxVec3 planeNormal = e0.cross(e1);
		const PxF32 planeD = planeNormal.dot(v0);	// PT: actually -d compared to PxcPlane
		if(planeNormal.dot(mSphereCenterShape1Space) < planeD)
			return;

		// PT: for a regular contact, 'delta' is non-zero (and so is 'squareDist'). However when the sphere's center exactly touches
		// the triangle, then both 'delta' and 'squareDist' become zero. This needs to be handled as a special case to avoid dividing
		// by zero. We will use the triangle's normal as a contact normal in this special case.
		//
		// 'validateSquareDist' is called twice because there are conflicting goals here. We could call it once now and already
		// compute the proper data for generating the contact. But this would mean doing a square-root and a division right here,
		// even when the contact is not actually needed in the end. We could also call it only once in "addContact', but the plane's
		// normal would not always be available (in case of delayed contacts), and thus it would need to be either recomputed (slower)
		// or stored within 'TriangleData' (using more memory). Calling 'validateSquareDist' twice is a better option overall.
		PxVec3 d;
		if(validateSquareDist(squareDist))
			d = delta;
		else
			d = planeNormal;

		if(fc==FC_FACE)
		{
			addContact(d, squareDist, triangleIndex);

			if(mNbCachedTris<ContactBuffer::MAX_CONTACTS)
				cacheTriangle(vertInds[0], vertInds[1], vertInds[2]);
		}
		else
		{
			if(mNbDelayed<ContactBuffer::MAX_CONTACTS)
			{
				const PxU32 index = mNbDelayed++;
				mSortKey[index].mSquareDist = squareDist;
				mSortKey[index].mIndex = index;

				TriangleData* saved = mSavedData + index;
				saved->mDelta			= d;
				saved->mVRef[0]			= vertInds[0];
				saved->mVRef[1]			= vertInds[1];
				saved->mVRef[2]			= vertInds[2];
				saved->mFC				= fc;
				saved->mTriangleIndex	= triangleIndex;
			}
			else outputErrorMessage();
		}
	}

	void generateLastContacts()
	{
		const PxU32 count = mNbDelayed;
		if(!count)
			return;

		Ps::sort(mSortKey, count, Ps::Less<SortKey>(), NullAllocator(), ContactBuffer::MAX_CONTACTS);

		TriangleData* touchedTris = mSavedData;
		for(PxU32 i=0;i<count;i++)
		{
			const TriangleData& data = touchedTris[mSortKey[i].mIndex];

			const PxU32 ref0 = data.mVRef[0];
			const PxU32 ref1 = data.mVRef[1];
			const PxU32 ref2 = data.mVRef[2];

			bool generateContact = false;

			switch(data.mFC)
			{
				case FC_VERTEX0:
					generateContact = ::validateVertex(ref0, mCachedTris, mNbCachedTris);
					break;

				case FC_VERTEX1:
					generateContact = ::validateVertex(ref1, mCachedTris, mNbCachedTris);
					break;

				case FC_VERTEX2:
					generateContact = ::validateVertex(ref2, mCachedTris, mNbCachedTris);
					break;

				case FC_EDGE01:
					generateContact = ::validateEdge(ref0, ref1, mCachedTris, mNbCachedTris);
					break;

				case FC_EDGE12:
					generateContact = ::validateEdge(ref1, ref2, mCachedTris, mNbCachedTris);
					break;

				case FC_EDGE20:
					generateContact = ::validateEdge(ref0, ref2, mCachedTris, mNbCachedTris);
					break;

				case FC_FACE:
				case FC_UNDEFINED:
					PX_ASSERT(0);	// PT: should not be possible
					break;
			};
	
			if(generateContact)
				addContact(data.mDelta, mSortKey[i].mSquareDist, data.mTriangleIndex);

			if(mNbCachedTris<ContactBuffer::MAX_CONTACTS)
				cacheTriangle(ref0, ref1, ref2);
			else
				outputErrorMessage();
		}
	}

private:
	SphereMeshContactGeneration& operator=(const SphereMeshContactGeneration&);
};

struct SphereMeshContactGenerationCallback_NoScale : MeshHitCallback<PxRaycastHit>
{
	SphereMeshContactGeneration			mGeneration;
	const TriangleMesh&					mMeshData;

	SphereMeshContactGenerationCallback_NoScale(const TriangleMesh& meshData, const PxSphereGeometry& shapeSphere,
		const PxTransform& transform0, const PxTransform& transform1, ContactBuffer& contactBuffer,
		const PxVec3& sphereCenterShape1Space, PxF32 inflatedRadius, Cm::RenderOutput* renderOutput
	) : MeshHitCallback<PxRaycastHit>	(CallbackMode::eMULTIPLE),
		mGeneration						(shapeSphere, transform0, transform1, contactBuffer, sphereCenterShape1Space, inflatedRadius, renderOutput),
		mMeshData						(meshData)
	{
	}

	virtual ~SphereMeshContactGenerationCallback_NoScale()
	{
		mGeneration.generateLastContacts();
	}

	virtual PxAgain processHit(
		const PxRaycastHit& hit, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, PxReal&, const PxU32* vinds)
	{
		if(gDrawTouchedTriangles)
		{
			(*mGeneration.mRenderOutput) << 0xffffffff;
			(*mGeneration.mRenderOutput) << PxMat44(PxIdentity);
			const PxVec3 wp0 = mGeneration.mTransform1.transform(v0);
			const PxVec3 wp1 = mGeneration.mTransform1.transform(v1);
			const PxVec3 wp2 = mGeneration.mTransform1.transform(v2);
			mGeneration.mRenderOutput->outputSegment(wp0, wp1);
			mGeneration.mRenderOutput->outputSegment(wp1, wp2);
			mGeneration.mRenderOutput->outputSegment(wp2, wp0);
		}

		mGeneration.processTriangle(hit.faceIndex, v0, v1, v2, vinds);
		return true;
	}

protected:
	SphereMeshContactGenerationCallback_NoScale &operator=(const SphereMeshContactGenerationCallback_NoScale &);
};

struct SphereMeshContactGenerationCallback_Scale : SphereMeshContactGenerationCallback_NoScale
{
	const Cm::FastVertex2ShapeScaling&	mMeshScaling;

	SphereMeshContactGenerationCallback_Scale(const TriangleMesh& meshData, const PxSphereGeometry& shapeSphere,
		const PxTransform& transform0, const PxTransform& transform1, const Cm::FastVertex2ShapeScaling& meshScaling,
		ContactBuffer& contactBuffer, const PxVec3& sphereCenterShape1Space, PxF32 inflatedRadius, Cm::RenderOutput* renderOutput
	) : SphereMeshContactGenerationCallback_NoScale(meshData, shapeSphere,
		transform0, transform1, contactBuffer, sphereCenterShape1Space, inflatedRadius, renderOutput),
		mMeshScaling	(meshScaling)
	{
	}

	virtual ~SphereMeshContactGenerationCallback_Scale() {}

	virtual PxAgain processHit(const PxRaycastHit& hit, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, PxReal&, const PxU32* vinds)
	{
		PxVec3 verts[3];
		getScaledVertices(verts, v0, v1, v2, false, mMeshScaling);

		if(gDrawTouchedTriangles)
		{
			(*mGeneration.mRenderOutput) << 0xffffffff;
			(*mGeneration.mRenderOutput) << PxMat44(PxIdentity);
			const PxVec3 wp0 = mGeneration.mTransform1.transform(verts[0]);
			const PxVec3 wp1 = mGeneration.mTransform1.transform(verts[1]);
			const PxVec3 wp2 = mGeneration.mTransform1.transform(verts[2]);
			mGeneration.mRenderOutput->outputSegment(wp0, wp1);
			mGeneration.mRenderOutput->outputSegment(wp1, wp2);
			mGeneration.mRenderOutput->outputSegment(wp2, wp0);
		}

		mGeneration.processTriangle(hit.faceIndex, verts[0], verts[1], verts[2], vinds);
		return true;
	}
protected:
	SphereMeshContactGenerationCallback_Scale &operator=(const SphereMeshContactGenerationCallback_Scale &);
};

}

///////////////////////////////////////////////////////////////////////////////

bool Gu::contactSphereMesh(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(cache);

	const PxSphereGeometry& shapeSphere = shape0.get<const PxSphereGeometry>();
	const PxTriangleMeshGeometryLL& shapeMesh = shape1.get<const PxTriangleMeshGeometryLL>();

	// We must be in local space to use the cache
	const PxVec3 sphereCenterInMeshSpace = transform1.transformInv(transform0.p);
	const PxReal inflatedRadius = shapeSphere.radius + params.mContactDistance;
	const TriangleMesh* meshData = shapeMesh.meshData;

	// mesh scale is not baked into cached verts
	if(shapeMesh.scale.isIdentity())
	{
		SphereMeshContactGenerationCallback_NoScale callback(
			*meshData, shapeSphere, transform0, transform1,
			contactBuffer, sphereCenterInMeshSpace, inflatedRadius, renderOutput);

		// PT: TODO: switch to sphere query here
		const Box obb(sphereCenterInMeshSpace, PxVec3(inflatedRadius), PxMat33(PxIdentity));
		Midphase::intersectOBB(meshData, obb, callback, true);
	}
	else
	{
		const Cm::FastVertex2ShapeScaling meshScaling(shapeMesh.scale);

		SphereMeshContactGenerationCallback_Scale callback(
			*meshData, shapeSphere, transform0, transform1,
			meshScaling, contactBuffer, sphereCenterInMeshSpace, inflatedRadius, renderOutput);

		PxVec3 obbCenter = sphereCenterInMeshSpace;
		PxVec3 obbExtents = PxVec3(inflatedRadius);
		PxMat33 obbRot(PxIdentity);
		meshScaling.transformQueryBounds(obbCenter, obbExtents, obbRot);

		const Box obb(obbCenter, obbExtents, obbRot);

		Midphase::intersectOBB(meshData, obb, callback, true);
	}
	return contactBuffer.count > 0;
}

///////////////////////////////////////////////////////////////////////////////

namespace
{
struct SphereHeightfieldContactGenerationCallback : EntityReport<PxU32>
{
	SphereMeshContactGeneration mGeneration;
	HeightFieldUtil&			mHfUtil;

	SphereHeightfieldContactGenerationCallback(
		HeightFieldUtil&		hfUtil,
		const PxSphereGeometry&	shapeSphere,
		const PxTransform&		transform0,
		const PxTransform&		transform1,
		ContactBuffer&			contactBuffer,
		const PxVec3&			sphereCenterInMeshSpace,
		PxF32					inflatedRadius,
		Cm::RenderOutput*		renderOutput
	) :
		mGeneration				(shapeSphere, transform0, transform1, contactBuffer, sphereCenterInMeshSpace, inflatedRadius, renderOutput),
		mHfUtil					(hfUtil)
	{
	}

	// PT: TODO: refactor/unify with similar code in other places
	virtual bool onEvent(PxU32 nb, PxU32* indices)
	{
		while(nb--)
		{
			const PxU32 triangleIndex = *indices++;
			PxU32 vertIndices[3];
			PxTriangle currentTriangle;
			mHfUtil.getTriangle(mGeneration.mTransform1, currentTriangle, vertIndices, NULL, triangleIndex, false, false);

			mGeneration.processTriangle(triangleIndex, currentTriangle.verts[0], currentTriangle.verts[1], currentTriangle.verts[2], vertIndices);
		}
		return true;
	}
protected:
	SphereHeightfieldContactGenerationCallback &operator=(const SphereHeightfieldContactGenerationCallback &);
};
}

bool Gu::contactSphereHeightfield(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(cache);
	PX_UNUSED(renderOutput);

	const PxSphereGeometry& shapeSphere = shape0.get<const PxSphereGeometry>();
	const PxHeightFieldGeometryLL& shapeMesh = shape1.get<const PxHeightFieldGeometryLL>();

	HeightFieldUtil hfUtil(shapeMesh);

	const PxVec3 sphereCenterInMeshSpace = transform1.transformInv(transform0.p);
	const PxReal inflatedRadius = shapeSphere.radius + params.mContactDistance;
	const PxVec3 inflatedRV3(inflatedRadius);

	const PxBounds3 bounds(sphereCenterInMeshSpace - inflatedRV3, sphereCenterInMeshSpace + inflatedRV3);

	SphereHeightfieldContactGenerationCallback blockCallback(hfUtil, shapeSphere, transform0, transform1, contactBuffer, sphereCenterInMeshSpace, inflatedRadius, renderOutput);

	hfUtil.overlapAABBTriangles(transform1, bounds, 0, &blockCallback);

	blockCallback.mGeneration.generateLastContacts();

	return contactBuffer.count > 0;
}

///////////////////////////////////////////////////////////////////////////////
