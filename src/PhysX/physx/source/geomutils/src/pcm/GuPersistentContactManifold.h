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

#ifndef Gu_PERSISTENTCONTACTMANIFOLD_H
#define Gu_PERSISTENTCONTACTMANIFOLD_H

#include "PxPhysXCommonConfig.h"
#include "foundation/PxUnionCast.h"
#include "foundation/PxMemory.h"
#include "CmPhysXCommon.h"
#include "PsVecTransform.h"

#define PCM_LOW_LEVEL_DEBUG 0

namespace physx
{

#define VISUALIZE_PERSISTENT_CONTACT 1
//This is for pritimives vs primitives 
#define GU_MANIFOLD_CACHE_SIZE 4
//These are for pritimives vs mesh
#define GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE 5
#define GU_SINGLE_MANIFOLD_CACHE_SIZE 6
#define GU_SPHERE_MANIFOLD_CACHE_SIZE 1
#define GU_CAPSULE_MANIFOLD_CACHE_SIZE 3
#define GU_MAX_MANIFOLD_SIZE 6
#define GU_MESH_CONTACT_REDUCTION_THRESHOLD	16

#define GU_MANIFOLD_INVALID_INDEX	0xffffffff


//ML: this is used to compared with the shape's margin to decide the final tolerance used in the manifold to validate the existing contacts.
//In the case of big shape and relatively speaking small triangles in the mesh, we need to take a smaller margin. This helps because the PCM 
//recycling thresholds are proportionate to margin so it makes it less likely to discard previous contacts due to separation
#define GU_PCM_MESH_MANIFOLD_EPSILON 0.05f


namespace Cm
{
	class RenderOutput;
}

  
namespace Gu
{
	struct ContactPoint;
	class ContactBuffer;
	struct GjkOutput;

extern const PxF32 invalidateThresholds[5]; 
extern const PxF32 invalidateQuatThresholds[5];
extern const PxF32 invalidateThresholds2[3]; 
extern const PxF32 invalidateQuatThresholds2[3];


Ps::aos::Mat33V findRotationMatrixFromZAxis(const Ps::aos::Vec3VArg to);

//This contact is used in the primitives vs primitives contact gen
class PersistentContact
{
public:
	PersistentContact()
	{
	}

	PersistentContact(Ps::aos::Vec3V _localPointA, Ps::aos::Vec3V _localPointB, Ps::aos::Vec4V _localNormalPen) : 
	mLocalPointA(_localPointA), mLocalPointB(_localPointB), mLocalNormalPen(_localNormalPen)
	{

	}

	Ps::aos::Vec3V mLocalPointA;
	Ps::aos::Vec3V mLocalPointB;
	Ps::aos::Vec4V mLocalNormalPen; // the (x, y, z) is the local normal, and the w is the penetration depth
};

//This contact is used in the mesh contact gen to store an extra variable
class MeshPersistentContact : public PersistentContact
{
public:
	PxU32 mFaceIndex;
};

//This contact is used in the compress stream buffer(NpCacheStreamPair in the PxcNpThreadContext) to store the data we need
PX_ALIGN_PREFIX(16)
class CachedMeshPersistentContact
{
public:
	PxVec3 mLocalPointA; //16 byte aligned
	PxU32 mFaceIndex;	 //face index
	PxVec3 mLocalPointB; //16 byte aligned
	PxU32  mPad;		 //pad
	PxVec3 mLocalNormal; //16 byte aligned
	PxReal mPen;		 //penetration
}PX_ALIGN_SUFFIX(16);


#if PX_VC 
    #pragma warning(push)   
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

//This class is used to store the start and end index of the mesh persistent contacts in an array.
struct PCMContactPatch
{
public:
	PCMContactPatch()
	{
		mNextPatch = NULL;
		mEndPatch = NULL;
		mRoot = this;
		mPatchMaxPen = Ps::aos::FMax();
	}
	Ps::aos::Vec3V mPatchNormal;
	PCMContactPatch* mNextPatch;//store the next patch pointer in the patch list
	PCMContactPatch* mEndPatch;//store the last patch pointer in the patch list
	PCMContactPatch* mRoot;//if this is the start of the patch list which has very similar patch normal, the root will be itself
	Ps::aos::FloatV mPatchMaxPen;//store the deepest penetration of the whole patch
	PxU32 mStartIndex;//store the start index of the manifold contacts in the manifold contacts stream
	PxU32 mEndIndex;//store the end index of the manifold contacts in the manifold contacts stream
	PxU32 mTotalSize;//if this is the root, the total size will store the total number of manifold contacts in the whole patch list
};

#if PX_VC 
     #pragma warning(pop) 
#endif

//ML: this is needed because it seems NEON doesn't force the alignment on SIMD type, which cause some of the PCM unit tests fail
PX_ALIGN_PREFIX(16)
class PersistentContactManifold
{
public:

	PersistentContactManifold(PersistentContact* contactPointsBuff, PxU8 capacity): mNumContacts(0), mCapacity(capacity), mNumWarmStartPoints(0), mContactPoints(contactPointsBuff)
	{
		mRelativeTransform.Invalidate();
	}

	PX_FORCE_INLINE PxU32	getNumContacts() const { return mNumContacts;}

	PX_FORCE_INLINE bool	isEmpty() { return mNumContacts==0; }

	PX_FORCE_INLINE PersistentContact& getContactPoint(const PxU32 index)
	{
		PX_ASSERT(index < GU_MANIFOLD_CACHE_SIZE);
		return mContactPoints[index];
	}
	
	PX_FORCE_INLINE Ps::aos::FloatV maxTransformdelta(const Ps::aos::PsTransformV& curTransform)
	{
		using namespace Ps::aos;
		const Vec4V p0 = Vec4V_From_Vec3V(mRelativeTransform.p);
		const Vec4V q0 = mRelativeTransform.q;
		const Vec4V p1 = Vec4V_From_Vec3V(curTransform.p);
		const Vec4V q1 = curTransform.q;

		const Vec4V dp = V4Abs(V4Sub(p1, p0));
		const Vec4V dq = V4Abs(V4Sub(q1, q0));

		const Vec4V max0 = V4Max(dp, dq);

		//need to work out max from a single vector...
		return V4ExtractMax(max0);
	}

	PX_FORCE_INLINE Ps::aos::Vec4V maxTransformdelta2(const Ps::aos::PsTransformV& curTransform)
	{
		using namespace Ps::aos;
		const Vec4V p0 = Vec4V_From_Vec3V(mRelativeTransform.p);
		const Vec4V q0 = mRelativeTransform.q;
		const Vec4V p1 = Vec4V_From_Vec3V(curTransform.p);
		const Vec4V q1 = curTransform.q;

		const Vec4V dp = V4Abs(V4Sub(p1, p0));
		const Vec4V dq = V4Abs(V4Sub(q1, q0));

		const Vec4V max0 = V4Max(dp, dq);

		//need to work out max from a single vector...
		return max0;
	}

	PX_FORCE_INLINE Ps::aos::FloatV maxTransformPositionDelta(const Ps::aos::Vec3V& curP)
	{
		using namespace Ps::aos;
	
		const Vec3V deltaP = V3Sub(curP, mRelativeTransform.p);
		const Vec4V delta = Vec4V_From_Vec3V(V3Abs(deltaP));
		//need to work out max from a single vector...
		return V4ExtractMax(delta);
	}

	PX_FORCE_INLINE Ps::aos::FloatV maxTransformQuatDelta(const Ps::aos::QuatV& curQ)
	{
		using namespace Ps::aos;
	
		const Vec4V deltaQ = V4Sub(curQ, mRelativeTransform.q);
		const Vec4V delta = V4Abs(deltaQ);
		//need to work out max from a single vector...
		return V4ExtractMax(delta);
	}

	PX_FORCE_INLINE void setRelativeTransform(const Ps::aos::PsTransformV& transform)
	{
		mRelativeTransform = transform;
	}

	//This is used for the box/convexhull vs box/convexhull contact gen to decide whether the relative movement of a pair of objects are 
	//small enough. In this case, we can skip the collision detection all together
	PX_FORCE_INLINE PxU32 invalidate_BoxConvex(const Ps::aos::PsTransformV& curRTrans, const Ps::aos::FloatVArg minMargin)
	{
		using namespace Ps::aos;
		PX_ASSERT(mNumContacts <= GU_MANIFOLD_CACHE_SIZE);
		const FloatV ratio = FLoad(invalidateThresholds[mNumContacts]);
		const FloatV thresholdP = FMul(minMargin, ratio);
		const FloatV deltaP = maxTransformPositionDelta(curRTrans.p);

		const FloatV thresholdQ = FLoad(invalidateQuatThresholds[mNumContacts]);
		const FloatV deltaQ = QuatDot(curRTrans.q, mRelativeTransform.q);
		const BoolV con = BOr(FIsGrtr(deltaP, thresholdP), FIsGrtr(thresholdQ, deltaQ));

		return BAllEqTTTT(con);
	}

	//This is used for the sphere/capsule vs other primitives contact gen to decide whether the relative movement of a pair of objects are 
	//small enough. In this case, we can skip the collision detection all together
	PX_FORCE_INLINE PxU32 invalidate_SphereCapsule(const Ps::aos::PsTransformV& curRTrans, const Ps::aos::FloatVArg minMargin)
	{
		using namespace Ps::aos;
		PX_ASSERT(mNumContacts <= 2);
		const FloatV ratio = FLoad(invalidateThresholds2[mNumContacts]);
		
		const FloatV thresholdP = FMul(minMargin, ratio);
		const FloatV deltaP = maxTransformPositionDelta(curRTrans.p);

		const FloatV thresholdQ = FLoad(invalidateQuatThresholds2[mNumContacts]);
		const FloatV deltaQ = QuatDot(curRTrans.q, mRelativeTransform.q);
		const BoolV con = BOr(FIsGrtr(deltaP, thresholdP), FIsGrtr(thresholdQ, deltaQ));

		return BAllEqTTTT(con);
	}

	//This is used for plane contact gen to decide whether the relative movement of a pair of objects are small enough. In this case, 
	//we can skip the collision detection all together
	PX_FORCE_INLINE PxU32 invalidate_PrimitivesPlane(const Ps::aos::PsTransformV& curRTrans, const Ps::aos::FloatVArg minMargin, const Ps::aos::FloatVArg ratio)
	{
		using namespace Ps::aos;
		const FloatV thresholdP = FMul(minMargin, ratio);
		const FloatV deltaP = maxTransformPositionDelta(curRTrans.p);
	
		const FloatV thresholdQ = FLoad(0.9998f);//about 1 degree
		const FloatV deltaQ = QuatDot(curRTrans.q, mRelativeTransform.q);
		const BoolV con = BOr(FIsGrtr(deltaP, thresholdP), FIsGrtr(thresholdQ, deltaQ));

		return BAllEqTTTT(con);
	}

	PX_FORCE_INLINE void removeContactPoint (PxU32 index)
	{
		mNumContacts--;
		mContactPoints[index] = mContactPoints[mNumContacts];
	}      
    
	bool validContactDistance(const PersistentContact& pt, const Ps::aos::FloatVArg breakingThreshold) const
	{
		using namespace Ps::aos;
		const FloatV dist = V4GetW(pt.mLocalNormalPen);
		return FAllGrtr(breakingThreshold, dist) != 0;
	}

	PX_FORCE_INLINE void clearManifold()
	{
		mNumWarmStartPoints = 0;
		mNumContacts = 0;
		mRelativeTransform.Invalidate();
	}

	PX_FORCE_INLINE void initialize()
	{
		clearManifold();
	}

	//This function is used to replace the existing contact with the newly created contact if their distance are within some threshold
	bool replaceManifoldPoint(const Ps::aos::Vec3VArg localPointA, const Ps::aos::Vec3VArg localPointB, const Ps::aos::Vec4VArg localNormalPen, const Ps::aos::FloatVArg replaceBreakingThreshold);

	//This function is to add a point(in box/convexhull contact gen) to the exising manifold. If the number of manifold is more than 4, we need to do contact reduction
	PxU32 addManifoldPoint( const Ps::aos::Vec3VArg localPointA, const Ps::aos::Vec3VArg localPointB, const Ps::aos::Vec4VArg localNormalAPen, const Ps::aos::FloatVArg replaceBreakingThreshold);
	//This function is to add a point(in capsule contact gen) to the exising manifold. If the number of manifold is more than 4, we need to do contact reduction
	PxU32 addManifoldPoint2( const Ps::aos::Vec3VArg localPointA, const Ps::aos::Vec3VArg localPointB, const Ps::aos::Vec4VArg localNormalAPen, const Ps::aos::FloatVArg replaceBreakingThreshold);//max two points of contacts  
	//This function is used in box-plane contact gen to add the plane contacts to the manifold 
	void addBatchManifoldContactsCluster( const PersistentContact* manifoldPoints, const PxU32 numPoints);      
	
	//This function is used in the capsule full manifold contact genenation(maximum 2 points). 
	void addBatchManifoldContacts2( const PersistentContact* manifoldPoints, const PxU32 numPoints);//max two points of contacts             
	
	//This function is used in the box/convexhull full manifold contact generation(maximum 4 points). 
	void addBatchManifoldContacts(const PersistentContact* manifoldPoints, const PxU32 numPoints, const PxReal toleranceLength);
	//This function is using the cluster algorithm to reduce contacts
	void reduceBatchContactsCluster(const PersistentContact* manifoldPoints, const PxU32 numPoints);
	//This function is called by addBatchManifoldContacts2 to reduce the manifold contacts to 2 points;
	void reduceBatchContacts2(const PersistentContact* manifoldPoints, const PxU32 numPoints);
	//This function is called by addBatchManifoldContacts to reduce the manifold contacts to 4 points
	void reduceBatchContacts(const PersistentContact* manifoldPoints, const PxU32 numPoints, const PxReal toleranceLength);
	

	//This function is used for incremental manifold contact reduction for box/convexhull
	PxU32 reduceContactsForPCM(const Ps::aos::Vec3VArg localPointA, const Ps::aos::Vec3VArg localPointB, const Ps::aos::Vec4VArg localNormalPen);
	//This function is used for incremental manifold contact reduction for capsule
	PxU32 reduceContactSegment(const Ps::aos::Vec3VArg localPointA, const Ps::aos::Vec3VArg localPointB, const Ps::aos::Vec4VArg localNormalPen);
  

	/*
		This function recalculate the contacts in the manifold based on the current relative transform between a pair of objects. If the recalculated contacts are within some threshold,
		we will keep the contacts; Otherwise, we will remove the contacts.
	*/
	void refreshContactPoints(const Ps::aos::PsMatTransformV& relTra, const Ps::aos::FloatVArg projectBreakingThreshold, const Ps::aos::FloatVArg contactOffset);
	//This function is just used in boxbox contact gen for fast transform
	void addManifoldContactsToContactBuffer(Gu::ContactBuffer& contactBuffer, const Ps::aos::Vec3VArg normal, const Ps::aos::PsMatTransformV& transf1);
	//This function is for adding box/convexhull manifold contacts to the contact buffer
	void addManifoldContactsToContactBuffer(Gu::ContactBuffer& contactBuffer, const Ps::aos::Vec3VArg normal, const Ps::aos::PsTransformV& transf1, const Ps::aos::FloatVArg contactOffset);
	//This function is for adding sphere/capsule manifold contacts to the contact buffer
	void addManifoldContactsToContactBuffer(Gu::ContactBuffer& contactBuffer, const Ps::aos::Vec3VArg normal, const Ps::aos::Vec3VArg projectionNormal, const Ps::aos::PsTransformV& transf0, const Ps::aos::FloatVArg radius, const Ps::aos::FloatVArg contactOffset);

	//get the average normal in the manifold in world space
	Ps::aos::Vec3V getWorldNormal(const Ps::aos::PsTransformV& trB);
	//get the average normal in the manifold in local B object space
	Ps::aos::Vec3V getLocalNormal();
	
	void recordWarmStart(PxU8* aIndices, PxU8* bIndices, PxU8& nbWarmStartPoints);
	void setWarmStart(const PxU8* aIndices, const PxU8* bIndices, const PxU8 nbWarmStartPoints);
	void drawManifold(Cm::RenderOutput& out, const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB);
	void drawManifold(Cm::RenderOutput& out, const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB, const Ps::aos::FloatVArg radius);
	void drawManifold(const PersistentContact& m, Cm::RenderOutput& out, const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB);
	static void drawPoint(Cm::RenderOutput& out, const Ps::aos::Vec3VArg p, const PxF32 size, const PxU32 color = 0x0000ff00);
	static void drawLine(Cm::RenderOutput& out, const Ps::aos::Vec3VArg p0, const Ps::aos::Vec3VArg p1, const PxU32 color = 0xff00ffff);  
	static void drawTriangle(Cm::RenderOutput& out, const Ps::aos::Vec3VArg p0, const Ps::aos::Vec3VArg p1, const Ps::aos::Vec3VArg p2, const PxU32 color = 0xffff0000);
	static void drawPolygon( Cm::RenderOutput& out, const Ps::aos::PsTransformV& transform,  Ps::aos::Vec3V* points, const PxU32 numVerts, const PxU32 color = 0xff00ffff);
	static void drawPolygon( Cm::RenderOutput& out, const Ps::aos::PsMatTransformV& transform,  Ps::aos::Vec3V* points, const PxU32 numVerts, const PxU32 color = 0xff00ffff);

	Ps::aos::PsTransformV mRelativeTransform;//aToB
	PxU8 mNumContacts;
	PxU8 mCapacity;
	PxU8 mNumWarmStartPoints;
	PxU8 mAIndice[4];
	PxU8 mBIndice[4]; 
	PersistentContact* mContactPoints;
} PX_ALIGN_SUFFIX(16);

PX_ALIGN_PREFIX(16)
class LargePersistentContactManifold : public PersistentContactManifold
{
public:
	LargePersistentContactManifold() : PersistentContactManifold(mContactPointsBuff, GU_MANIFOLD_CACHE_SIZE)
	{
	}

	PersistentContact mContactPointsBuff[GU_MANIFOLD_CACHE_SIZE];
}PX_ALIGN_SUFFIX(16);

PX_ALIGN_PREFIX(16)
class SpherePersistentContactManifold : public PersistentContactManifold
{
public:
	SpherePersistentContactManifold() : PersistentContactManifold(mContactPointsBuff, GU_SPHERE_MANIFOLD_CACHE_SIZE)
	{
	}

	PersistentContact mContactPointsBuff[GU_SPHERE_MANIFOLD_CACHE_SIZE];
}PX_ALIGN_SUFFIX(16);

PX_ALIGN_PREFIX(16)
class SinglePersistentContactManifold
{
public:

	SinglePersistentContactManifold(): mNumContacts(0)
	{
	}

	PX_FORCE_INLINE PxU32	getNumContacts() const { return mNumContacts;}

	PX_FORCE_INLINE bool	isEmpty() { return mNumContacts==0; }

	PX_FORCE_INLINE MeshPersistentContact& getContactPoint(const PxU32 index)
	{
		PX_ASSERT(index < GU_SINGLE_MANIFOLD_CACHE_SIZE);
		return mContactPoints[index];
	}

	PX_FORCE_INLINE void removeContactPoint (PxU32 index)
	{
		mNumContacts--;
		mContactPoints[index] = mContactPoints[mNumContacts];
	}  

	PX_FORCE_INLINE void clearManifold()
	{
		mNumContacts = 0;
	}

	PX_FORCE_INLINE void initialize()
	{
		clearManifold();
	}


	PX_FORCE_INLINE Ps::aos::Vec3V getWorldNormal(const Ps::aos::PsTransformV& trB)
	{
		using namespace Ps::aos;
		Vec4V nPen = mContactPoints[0].mLocalNormalPen;
		for(PxU32 i =1; i < mNumContacts; ++i)
		{
			nPen = V4Add(nPen, mContactPoints[i].mLocalNormalPen);
		}

		const Vec3V n = Vec3V_From_Vec4V(nPen);
		return V3Normalize(trB.rotate(n));
	}

	PX_FORCE_INLINE Ps::aos::Vec3V getLocalNormal()
	{
		using namespace Ps::aos;
		Vec4V nPen = mContactPoints[0].mLocalNormalPen;
		for(PxU32 i =1; i < mNumContacts; ++i)
		{
			nPen = V4Add(nPen, mContactPoints[i].mLocalNormalPen);
		}
		return V3Normalize(Vec3V_From_Vec4V(nPen));
	}
	
	//This function reduces the manifold contact list in a patch for box/convexhull vs mesh
	Ps::aos::FloatV reduceBatchContactsConvex(const MeshPersistentContact* manifoldContactExt, const PxU32 numContacts, PCMContactPatch& patch);
	//This function reduces the manifold contact list in a patch for sphere vs mesh
	Ps::aos::FloatV reduceBatchContactsSphere(const MeshPersistentContact* manifoldContactExt, const PxU32 numContacts, PCMContactPatch& patch);
	//This function reduces the manifold contact list in a pathc for capsuel vs mesh
	Ps::aos::FloatV reduceBatchContactsCapsule(const MeshPersistentContact* manifoldContactExt, const PxU32 numContacts, PCMContactPatch& patch);

	//This function adds the manifold contact list in a patch for box/convexhull vs mesh
	Ps::aos::FloatV addBatchManifoldContactsConvex(const MeshPersistentContact* manifoldContact, const PxU32 numContacts, PCMContactPatch& patch, const Ps::aos::FloatVArg replaceBreakingThreshold);
	//This function adds the manifold contact list in a patch for sphere vs mesh
	Ps::aos::FloatV addBatchManifoldContactsSphere(const MeshPersistentContact* manifoldContact, const PxU32 numContacts, PCMContactPatch& patch, const Ps::aos::FloatVArg replaceBreakingThreshold);
	//This function adds the manifold contact list in a patch for capsule vs mesh
	Ps::aos::FloatV addBatchManifoldContactsCapsule(const MeshPersistentContact* manifoldContact, const PxU32 numContacts, PCMContactPatch& patch, const Ps::aos::FloatVArg replaceBreakingThreshold);
	
	//This is used for in the addContactsToPatch for convex mesh contact gen. 
	static PxU32 reduceContacts(MeshPersistentContact* manifoldContactExt, PxU32 numContacts);

	//This function is to recalculate the contacts based on the relative transform between a pair of objects
	Ps::aos::FloatV refreshContactPoints(const Ps::aos::PsMatTransformV& relTra, const Ps::aos::FloatVArg projectBreakingThreshold, const Ps::aos::FloatVArg contactOffset);

	void drawManifold(Cm::RenderOutput& out, const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB);

	MeshPersistentContact mContactPoints[GU_SINGLE_MANIFOLD_CACHE_SIZE];//384 bytes
	PxU32 mNumContacts;//400 bytes
	
} PX_ALIGN_SUFFIX(16);

//This is a structure used to cache a multi-persistent-manifold in the cache stream
struct MultiPersistentManifoldHeader
{
	Ps::aos::PsTransformV mRelativeTransform;//aToB
	PxU32 mNumManifolds;
	PxU32 pad[3];
};

struct SingleManifoldHeader
{
	PxU32 mNumContacts;
	PxU32 pad[3];
};

#if PX_VC 
#pragma warning(push)
#pragma warning( disable : 4251 ) // class needs to have dll-interface to be used by clients of class
#endif

PX_ALIGN_PREFIX(16)
class PX_PHYSX_COMMON_API MultiplePersistentContactManifold
{
public:
	MultiplePersistentContactManifold():mNumManifolds(0), mNumTotalContacts(0)
	{
		mRelativeTransform.Invalidate();
	}

	PX_FORCE_INLINE void setRelativeTransform(const Ps::aos::PsTransformV& transform)
	{
		mRelativeTransform = transform;
	}

	PX_FORCE_INLINE Ps::aos::FloatV maxTransformPositionDelta(const Ps::aos::Vec3V& curP)
	{
		using namespace Ps::aos;
	
		const Vec3V deltaP = V3Sub(curP, mRelativeTransform.p);
		const Vec4V delta = Vec4V_From_Vec3V(V3Abs(deltaP));
		//need to work out max from a single vector...
		return V4ExtractMax(delta);
	}

	PX_FORCE_INLINE Ps::aos::FloatV maxTransformQuatDelta(const Ps::aos::QuatV& curQ)
	{
		using namespace Ps::aos;
	
		const Vec4V deltaQ = V4Sub(curQ, mRelativeTransform.q);
		const Vec4V delta = V4Abs(deltaQ);
		//need to work out max from a single vector...
		return V4ExtractMax(delta);
	}

	PX_FORCE_INLINE PxU32 invalidate(const Ps::aos::PsTransformV& curRTrans, const Ps::aos::FloatVArg minMargin, const Ps::aos::FloatVArg ratio)
	{
		using namespace Ps::aos;
		
		const FloatV thresholdP = FMul(minMargin, ratio);
		const FloatV deltaP = maxTransformPositionDelta(curRTrans.p);
		const FloatV thresholdQ = FLoad(0.9998f);//about 1 degree
		const FloatV deltaQ = QuatDot(curRTrans.q, mRelativeTransform.q);
		const BoolV con = BOr(FIsGrtr(deltaP, thresholdP), FIsGrtr(thresholdQ, deltaQ));

		return BAllEqTTTT(con);
	}

	PX_FORCE_INLINE PxU32 invalidate(const Ps::aos::PsTransformV& curRTrans, const Ps::aos::FloatVArg minMargin)
	{
		using namespace Ps::aos;
		return invalidate(curRTrans, minMargin, FLoad(0.2f));
	}

	/*
		This function work out the contact patch connectivity. If two patches's normal are within 5 degree, we would link these two patches together and reset the total size.
	*/
	PX_FORCE_INLINE void refineContactPatchConnective(PCMContactPatch** contactPatch, PxU32 numContactPatch, MeshPersistentContact* manifoldContacts, const Ps::aos::FloatVArg acceptanceEpsilon)
	{
		PX_UNUSED(manifoldContacts);

		using namespace Ps::aos;
	
		//work out the contact patch connectivity, the patchNormal should be in the local space of mesh
		for(PxU32 i=0; i<numContactPatch; ++i)
		{
			PCMContactPatch* patch = contactPatch[i];
			patch->mRoot = patch;
			patch->mEndPatch = patch;
			patch->mTotalSize = patch->mEndIndex - patch->mStartIndex;
			patch->mNextPatch = NULL;
	
			for(PxU32 j=i; j>0; --j)
			{
				PCMContactPatch* other = contactPatch[j-1];
				const FloatV d = V3Dot(patch->mPatchNormal, other->mRoot->mPatchNormal);
				if(FAllGrtrOrEq(d, acceptanceEpsilon))//less than 5 degree
				{
					
					other->mNextPatch = patch;
					other->mRoot->mEndPatch = patch;
					patch->mRoot = other->mRoot;
					other->mRoot->mTotalSize += patch->mEndIndex - patch->mStartIndex;
					break;
				}
			}
		}
	}


	/*
		This function uses to reduce the manifold contacts which are in different connected patchs but are within replace breaking threshold 
	*/
	PX_FORCE_INLINE PxU32 reduceManifoldContactsInDifferentPatches(PCMContactPatch** contactPatch, PxU32 numContactPatch, MeshPersistentContact* manifoldContacts, PxU32 numContacts, const Ps::aos::FloatVArg sqReplaceBreaking)
	{
		using namespace Ps::aos;

		for(PxU32 i=0; i<numContactPatch; ++i)
		{
			PCMContactPatch* currentPatch = contactPatch[i];
			//this make sure the patch is the root before we do the contact reduction, otherwise, we will do duplicate work
			if(currentPatch->mRoot == currentPatch)
			{
				while(currentPatch)
				{
					PCMContactPatch* nextPatch = currentPatch->mNextPatch;
					if(nextPatch)
					{
						for(PxU32 k = currentPatch->mStartIndex; k<currentPatch->mEndIndex; ++k)
						{
							for(PxU32 l = nextPatch->mStartIndex; l < nextPatch->mEndIndex; ++l)
							{
								Vec3V dif = V3Sub(manifoldContacts[l].mLocalPointB, manifoldContacts[k].mLocalPointB);
								FloatV d = V3Dot(dif, dif);
								if(FAllGrtr(sqReplaceBreaking, d))
								{
									//if two manifold contacts are within threshold, we will get rid of the manifold contacts in the other contact patch
									manifoldContacts[l] = manifoldContacts[nextPatch->mEndIndex-1];
									nextPatch->mEndIndex--;
									numContacts--;
									l--;
								}
							}
						}
					}
					currentPatch = nextPatch;
				}
			}
		}
	
		return numContacts;
	}


	/*
		This function is for the multiple manifold loop through each individual single manifold to recalculate the contacts based on the relative transform between a pair of objects
	*/
	PX_FORCE_INLINE void refreshManifold(const Ps::aos::PsMatTransformV& relTra, const Ps::aos::FloatVArg projectBreakingThreshold, const Ps::aos::FloatVArg contactDist)
	{
		using namespace Ps::aos;
	
		//refresh manifold contacts
		for(PxU32 i=0; i < mNumManifolds; ++i)
		{
			PxU8 ind = mManifoldIndices[i];
			PX_ASSERT(mManifoldIndices[i] < GU_MAX_MANIFOLD_SIZE);
			PxU32 nextInd = PxMin(i, mNumManifolds-2u)+1;
			Ps::prefetchLine(&mManifolds[mManifoldIndices[nextInd]]);
			Ps::prefetchLine(&mManifolds[mManifoldIndices[nextInd]],128);
			Ps::prefetchLine(&mManifolds[mManifoldIndices[nextInd]],256);
			FloatV _maxPen = mManifolds[ind].refreshContactPoints(relTra, projectBreakingThreshold, contactDist);
			if(mManifolds[ind].isEmpty())
			{
				//swap the index with the next manifolds
				PxU8 index = mManifoldIndices[--mNumManifolds];
				mManifoldIndices[mNumManifolds] = ind;
				mManifoldIndices[i] = index;
				i--;
			}
			else
			{
				FStore(_maxPen, &mMaxPen[ind]);
			}
		}
	}


	PX_FORCE_INLINE void initialize()
	{
		mNumManifolds = 0;
		mNumTotalContacts = 0;
		mRelativeTransform.Invalidate();
		for(PxU8 i=0; i<GU_MAX_MANIFOLD_SIZE; ++i)
		{
			mManifolds[i].initialize();
			mManifoldIndices[i] = i;
		}
	}

	PX_FORCE_INLINE void clearManifold()
	{
		for(PxU8 i=0; i<mNumManifolds; ++i)
		{
			mManifolds[i].clearManifold();
		}
		mNumManifolds = 0;
		mNumTotalContacts = 0;
		mRelativeTransform.Invalidate();
	}

	PX_FORCE_INLINE SinglePersistentContactManifold* getManifold(const PxU32 index)
	{
		PX_ASSERT(index < GU_MAX_MANIFOLD_SIZE);
		return &mManifolds[mManifoldIndices[index]];
	}

	PX_FORCE_INLINE SinglePersistentContactManifold* getEmptyManifold()
	{
		if(mNumManifolds < GU_MAX_MANIFOLD_SIZE)
			return  &mManifolds[mManifoldIndices[mNumManifolds]];
		return NULL;
	}

	//This function adds the manifold contacts with different patches into the corresponding single persistent contact manifold
	void addManifoldContactPoints(MeshPersistentContact* manifoldContact, PxU32 numManifoldContacts, PCMContactPatch** contactPatch, const PxU32 numPatch, 
		const Ps::aos::FloatVArg sqReplaceBreakingThreshold, const Ps::aos::FloatVArg acceptanceEpsilon, PxU8 maxContactsPerManifold);
	//This function adds the box/convexhull manifold contacts to the contact buffer 
	bool addManifoldContactsToContactBuffer(Gu::ContactBuffer& contactBuffer, const Ps::aos::PsTransformV& transf1);
	//This function adds the sphere/capsule manifold contacts to the contact buffer
	bool addManifoldContactsToContactBuffer(Gu::ContactBuffer& contactBuffer, const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB, const Ps::aos::FloatVArg radius);
	void drawManifold(Cm::RenderOutput& out, const Ps::aos::PsTransformV& trA, const Ps::aos::PsTransformV& trB);

	//Code to load from a buffer and store to a buffer.
	void fromBuffer(PxU8*  PX_RESTRICT buffer);
	void toBuffer(PxU8*  PX_RESTRICT buffer);

	static void drawLine(Cm::RenderOutput& out, const Ps::aos::Vec3VArg p0, const Ps::aos::Vec3VArg p1, const PxU32 color = 0xff00ffff);
	static void drawLine(Cm::RenderOutput& out, const PxVec3 p0, const PxVec3 p1, const PxU32 color = 0xff00ffff);
	static void drawPoint(Cm::RenderOutput& out, const Ps::aos::Vec3VArg p, const PxF32 size, const PxU32 color = 0x00ff0000);
	static void drawPolygon( Cm::RenderOutput& out, const Ps::aos::PsTransformV& transform,  Ps::aos::Vec3V* points, const PxU32 numVerts, const PxU32 color = 0xff00ffff);

	Ps::aos::PsTransformV mRelativeTransform;//aToB
	PxF32 mMaxPen[GU_MAX_MANIFOLD_SIZE];
	PxU8 mManifoldIndices[GU_MAX_MANIFOLD_SIZE];
	PxU8 mNumManifolds;
	PxU8 mNumTotalContacts;
	SinglePersistentContactManifold mManifolds[GU_MAX_MANIFOLD_SIZE];
	
	
} PX_ALIGN_SUFFIX(16);

#if PX_VC 
#pragma warning(pop)
#endif

/*
	This function calculates the average normal in the manifold in world space
*/
PX_FORCE_INLINE Ps::aos::Vec3V PersistentContactManifold::getWorldNormal(const Ps::aos::PsTransformV& trB)
{
	using namespace Ps::aos;
	
	Vec4V nPen = mContactPoints[0].mLocalNormalPen;
	for(PxU32 i =1; i < mNumContacts; ++i)
	{
		nPen = V4Add(nPen, mContactPoints[i].mLocalNormalPen);
	}

	const Vec3V n = Vec3V_From_Vec4V(nPen);
	const FloatV sqLength = V3Dot(n, n);
	const Vec3V nn = V3Sel(FIsGrtr(sqLength, FEps()), n, Vec3V_From_Vec4V(mContactPoints[0].mLocalNormalPen));
	return V3Normalize(trB.rotate(nn));
}

/*
	This function calculates the average normal in the manifold in local B space
*/
PX_FORCE_INLINE Ps::aos::Vec3V PersistentContactManifold::getLocalNormal()
{
	using namespace Ps::aos;
	
	Vec4V nPen = mContactPoints[0].mLocalNormalPen;
	for(PxU32 i =1; i < mNumContacts; ++i)
	{
		nPen = V4Add(nPen, mContactPoints[i].mLocalNormalPen);
	}
	return V3Normalize(Vec3V_From_Vec4V(nPen));
}

/*
	This function recalculates the contacts in the manifold based on the current relative transform between a pair of objects. If the recalculated contacts are within some threshold,
	we will keep the contacts; Otherwise, we will remove the contacts.
*/
PX_FORCE_INLINE void PersistentContactManifold::refreshContactPoints(const Ps::aos::PsMatTransformV& aToB, const Ps::aos::FloatVArg projectBreakingThreshold, const Ps::aos::FloatVArg /*contactOffset*/)
{
	using namespace Ps::aos;
	const FloatV sqProjectBreakingThreshold =  FMul(projectBreakingThreshold, projectBreakingThreshold); 

	// first refresh worldspace positions and distance
	for (PxU32 i=mNumContacts; i > 0; --i)
	{
		PersistentContact& manifoldPoint = mContactPoints[i-1];
		const Vec3V localAInB = aToB.transform( manifoldPoint.mLocalPointA ); // from a to b
		const Vec3V localBInB = manifoldPoint.mLocalPointB;
		const Vec3V v = V3Sub(localAInB, localBInB); 

		const Vec3V localNormal = Vec3V_From_Vec4V(manifoldPoint.mLocalNormalPen); // normal in b space
		const FloatV dist= V3Dot(v, localNormal);

		const Vec3V projectedPoint = V3NegScaleSub(localNormal,  dist, localAInB);//manifoldPoint.worldPointA - manifoldPoint.worldPointB * manifoldPoint.m_distance1;
		const Vec3V projectedDifference = V3Sub(localBInB, projectedPoint);

		const FloatV distance2d = V3Dot(projectedDifference, projectedDifference);
		//const BoolV con = BOr(FIsGrtr(dist, contactOffset), FIsGrtr(distance2d, sqProjectBreakingThreshold));
		const BoolV con = FIsGrtr(distance2d, sqProjectBreakingThreshold);
		if(BAllEqTTTT(con))
		{
			removeContactPoint(i-1);
		} 
		else
		{
			manifoldPoint.mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), dist);
		}
	}
}

/*
	This function copies the mesh persistent contact from the multiple manifold to compress buffer(NpCacheStreamPair in the PxcNpThreadContext)
*/
PX_INLINE void MultiplePersistentContactManifold::toBuffer(PxU8*  PX_RESTRICT buffer)
{
	using namespace Ps::aos;
	PxU8* buff = buffer;

	PX_ASSERT(((uintptr_t(buff)) & 0xF) == 0);
	MultiPersistentManifoldHeader* PX_RESTRICT header = reinterpret_cast<MultiPersistentManifoldHeader*>(buff);
	buff += sizeof(MultiPersistentManifoldHeader);

	PX_ASSERT(mNumManifolds <= GU_MAX_MANIFOLD_SIZE);
	header->mNumManifolds = mNumManifolds;
	header->mRelativeTransform = mRelativeTransform;

	for(PxU32 a = 0; a < mNumManifolds; ++a)
	{
		SingleManifoldHeader* manHeader = reinterpret_cast<SingleManifoldHeader*>(buff);
		buff += sizeof(SingleManifoldHeader);
		SinglePersistentContactManifold& manifold = *getManifold(a);
		manHeader->mNumContacts = manifold.mNumContacts;
		PX_ASSERT((uintptr_t(buff) & 0xf) == 0);
		CachedMeshPersistentContact* contacts = reinterpret_cast<CachedMeshPersistentContact*>(buff);
		//convert the mesh persistent contact to cached mesh persistent contact to save 16 byte memory per contact
		for(PxU32 b = 0; b<manifold.mNumContacts; ++b)
		{	
			V4StoreA(Vec4V_From_Vec3V(manifold.mContactPoints[b].mLocalPointA), &contacts[b].mLocalPointA.x);
			V4StoreA(Vec4V_From_Vec3V(manifold.mContactPoints[b].mLocalPointB), &contacts[b].mLocalPointB.x);
			V4StoreA(manifold.mContactPoints[b].mLocalNormalPen, &contacts[b].mLocalNormal.x);
			//Note - this must be written last because we just wrote mLocalPointA to this memory so need to make sure
			//that face index is written after that.
			contacts[b].mFaceIndex = manifold.mContactPoints[b].mFaceIndex;
		}
		buff += sizeof(CachedMeshPersistentContact) * manifold.mNumContacts;
	}
}

#define PX_CP_TO_PCP(contactPoint)				(reinterpret_cast<PersistentContact*>(contactPoint)) //this is used in the normal pcm contact gen
#define PX_CP_TO_MPCP(contactPoint)				(reinterpret_cast<MeshPersistentContact*>(contactPoint))//this is used in the mesh pcm contact gen

void addManifoldPoint(PersistentContact* manifoldContacts, PersistentContactManifold& manifold, GjkOutput& output,
	const Ps::aos::PsMatTransformV& aToB, const Ps::aos::FloatV replaceBreakingThreshold);

}//Gu
}//physx

#endif
