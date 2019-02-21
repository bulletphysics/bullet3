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



#ifndef DY_ARTICULATION_H
#define DY_ARTICULATION_H

#include "PsVecMath.h"
#include "CmSpatialVector.h"
#include "DySpatial.h"
#include "PsBitUtils.h"
#include "DyArticulation.h"
#include "DyArticulationHelper.h"

namespace physx
{

namespace Dy
{
	struct ArticulationCore;
	struct ArticulationLink;
	typedef size_t ArticulationLinkHandle;
	class Articulation;

#define DY_ARTICULATION_DEBUG_VERIFY 0

PX_FORCE_INLINE PxU32 ArticulationLowestSetBit(ArticulationBitField val)
{
	PxU32 low = PxU32(val&0xffffffff), high = PxU32(val>>32);
	PxU32 mask = PxU32((!low)-1);
	PxU32 result = (mask&Ps::lowestSetBitUnsafe(low)) | ((~mask)&(Ps::lowestSetBitUnsafe(high)+32));
	PX_ASSERT(val & (PxU64(1)<<result));
	PX_ASSERT(!(val & ((PxU64(1)<<result)-1)));
	return result;
}

PX_FORCE_INLINE PxU32 ArticulationHighestSetBit(ArticulationBitField val)
{
	PxU32 low = PxU32(val & 0xffffffff), high = PxU32(val >> 32);
	PxU32 mask = PxU32((!high) - 1);
	PxU32 result = ((~mask)&Ps::highestSetBitUnsafe(low)) | ((mask)&(Ps::highestSetBitUnsafe(high) + 32));
	PX_ASSERT(val & (PxU64(1) << result));
	return result;
}

using namespace Ps::aos;



PX_FORCE_INLINE Cm::SpatialVector& unsimdRef(Cm::SpatialVectorV& v)				{ return reinterpret_cast<Cm::SpatialVector&>(v); }
PX_FORCE_INLINE const Cm::SpatialVector& unsimdRef(const Cm::SpatialVectorV& v) { return reinterpret_cast<const Cm::SpatialVector&>(v); }


PX_ALIGN_PREFIX(16)
struct FsJointVectors
{
	Vec3V					parentOffset;		// 16 bytes world-space offset from parent to child
	Vec3V					jointOffset;		// 16 bytes world-space offset from child to joint
}
PX_ALIGN_SUFFIX(16);

PX_ALIGN_PREFIX(16)
struct FsRow
{	
	Cm::SpatialVectorV			DSI[3];				// 96 bytes
	Mat33V					D;					// 48 bytes
	ArticulationBitField	children;			// 8 bytes bitmap of children
	ArticulationBitField	pathToRoot;			// 8 bytes bitmap of nodes to root, including self and root
}
PX_ALIGN_SUFFIX(16);

PX_COMPILE_TIME_ASSERT(sizeof(FsRow)==160);



PX_ALIGN_PREFIX(16)
struct FsInertia
{
	Mat33V ll, la, aa;
	PX_FORCE_INLINE FsInertia(const Mat33V& _ll, const Mat33V& _la, const Mat33V& _aa): ll(_ll), la(_la), aa(_aa) {}
	PX_FORCE_INLINE FsInertia(const SpInertia& I)
	: ll(Mat33V_From_PxMat33(I.mLL)), la(Mat33V_From_PxMat33(I.mLA)), aa(Mat33V_From_PxMat33(I.mAA)) {}
	PX_FORCE_INLINE FsInertia() {}

	PX_FORCE_INLINE void operator=(const FsInertia& other)
	{
		ll.col0 = other.ll.col0;	ll.col1 = other.ll.col1;	ll.col2 = other.ll.col2;
		la.col0 = other.la.col0;	la.col1 = other.la.col1;	la.col2 = other.la.col2;
		aa.col0 = other.aa.col0;	aa.col1 = other.aa.col1;	aa.col2 = other.aa.col2;
	}

	PX_FORCE_INLINE FsInertia(const FsInertia& other)
	{
		ll.col0 = other.ll.col0;	ll.col1 = other.ll.col1;	ll.col2 = other.ll.col2;
		la.col0 = other.la.col0;	la.col1 = other.la.col1;	la.col2 = other.la.col2;
		aa.col0 = other.aa.col0;	aa.col1 = other.aa.col1;	aa.col2 = other.aa.col2;
	}

}PX_ALIGN_SUFFIX(16);

PX_ALIGN_PREFIX(16)
struct LtbRow
{
	FsInertia		inertia;			// body inertia in world space
	Cm::SpatialVectorV		j0[3], j1[3];		// jacobians
	Mat33V				jResponse;			// inverse response matrix of joint
	Vec3V				jC;
} PX_ALIGN_SUFFIX(16);

PX_ALIGN_PREFIX(16)
struct FsRowAux
{
	Cm::SpatialVectorV		S[3];				// motion subspace
}PX_ALIGN_SUFFIX(16);


struct FsData
{
	//Articulation*	articulationX;																//4
	
#if !PX_P64_FAMILY
	PxU32				pad0;																		//8	
#endif
	PxU16				linkCount;						// number of links							//10
	PxU16				jointVectorOffset;				// offset of read-only data					//12
//	PxU16				maxSolverNormalProgress;													//14
//	PxU16				maxSolverFrictionProgress;													//16

	PxU64				dirty;																		//24
	PxU16				ltbDataOffset;					// offset of save-velocity data				//26
	PxU16				fsDataOffset;					// offset of joint references				//28
	//PxU32				solverProgress;																//32
	

	Cm::SpatialVectorV		deferredZ;																	//64
	PxU8				parent[DY_ARTICULATION_MAX_SIZE];											//128
};

PX_COMPILE_TIME_ASSERT(0 == (sizeof(FsData) & 0x0f));

#define SOLVER_BODY_SOLVER_PROGRESS_OFFSET 28	
#define SOLVER_BODY_MAX_SOLVER_PROGRESS_OFFSET 12

namespace
{
	template<class T> PX_FORCE_INLINE T addAddr(void* addr, PxU32 increment) 
	{ 
		return reinterpret_cast<T>(reinterpret_cast<char*>(addr)+increment);
	}

	template<class T> PX_FORCE_INLINE T addAddr(const void* addr, PxU32 increment) 
	{ 
		return reinterpret_cast<T>(reinterpret_cast<const char*>(addr)+increment);
	}
}

PX_FORCE_INLINE Cm::SpatialVectorV* getVelocity(FsData& matrix)
{
	return addAddr<Cm::SpatialVectorV*>(&matrix, sizeof(FsData));
}

PX_FORCE_INLINE const Cm::SpatialVectorV* getVelocity(const FsData& matrix)
{
	return addAddr<const Cm::SpatialVectorV*>(&matrix, sizeof(FsData));
}

PX_FORCE_INLINE Cm::SpatialVectorV* getDeferredVel(FsData& matrix)
{
	return addAddr<Cm::SpatialVectorV*>(getVelocity(matrix), sizeof(Cm::SpatialVectorV) * matrix.linkCount);
}

PX_FORCE_INLINE const Cm::SpatialVectorV* getDeferredVel(const FsData& matrix)
{
	return addAddr<const Cm::SpatialVectorV*>(getVelocity(matrix), sizeof(Cm::SpatialVectorV) * matrix.linkCount);
}

PX_FORCE_INLINE const Cm::SpatialVectorV* getMotionVector(const FsData& matrix)
{
	return addAddr<const Cm::SpatialVectorV*>(getDeferredVel(matrix), sizeof(Cm::SpatialVectorV) * matrix.linkCount);
}

PX_FORCE_INLINE Cm::SpatialVectorV* getMotionVector(FsData& matrix)
{
	return addAddr<Cm::SpatialVectorV*>(getDeferredVel(matrix), sizeof(Cm::SpatialVectorV) * matrix.linkCount);
}

PX_FORCE_INLINE Vec3V* getDeferredSZ(FsData& matrix)
{
	return addAddr<Vec3V*>(getMotionVector(matrix), sizeof(Cm::SpatialVectorV) * matrix.linkCount);
}

PX_FORCE_INLINE const Vec3V* getDeferredSZ(const FsData& matrix)
{
	return addAddr<const Vec3V*>(getMotionVector(matrix), sizeof(Cm::SpatialVectorV) * matrix.linkCount);
}

PX_FORCE_INLINE const PxReal* getMaxPenBias(const FsData& matrix)
{
	return addAddr<const PxReal*>(getDeferredSZ(matrix), sizeof(Vec3V) * matrix.linkCount);
}

PX_FORCE_INLINE PxReal* getMaxPenBias(FsData& matrix)
{
	return addAddr<PxReal*>(getDeferredSZ(matrix), sizeof(Vec3V) * matrix.linkCount);
}


PX_FORCE_INLINE FsJointVectors* getJointVectors(FsData& matrix)
{
	return addAddr<FsJointVectors *>(&matrix,matrix.jointVectorOffset);
}

PX_FORCE_INLINE const FsJointVectors* getJointVectors(const FsData& matrix)
{
	return addAddr<const FsJointVectors *>(&matrix,matrix.jointVectorOffset);
}

PX_FORCE_INLINE FsInertia& getRootInverseInertia(FsData& matrix)
{
	return *addAddr<FsInertia*>(&matrix,matrix.fsDataOffset);
}

PX_FORCE_INLINE const FsInertia& getRootInverseInertia(const FsData& matrix)
{
	return *addAddr<const FsInertia*>(&matrix,matrix.fsDataOffset);
	
}

PX_FORCE_INLINE FsRow* getFsRows(FsData& matrix)
{
	return addAddr<FsRow*>(&getRootInverseInertia(matrix),sizeof(FsInertia));
}

PX_FORCE_INLINE const FsRow* getFsRows(const FsData& matrix)
{
	return addAddr<const FsRow*>(&getRootInverseInertia(matrix),sizeof(FsInertia));
}


PX_FORCE_INLINE LtbRow* getLtbRows(FsData& matrix)
{
	return addAddr<LtbRow*>(&matrix,matrix.ltbDataOffset);
}

PX_FORCE_INLINE const LtbRow* getLtbRows(const FsData& matrix)
{
	return addAddr<const LtbRow*>(&matrix,matrix.ltbDataOffset);
}


PX_FORCE_INLINE Cm::SpatialVectorV* getRefVelocity(FsData& matrix)
{
	return addAddr<Cm::SpatialVectorV*>(getLtbRows(matrix), sizeof(LtbRow)*matrix.linkCount);
}

PX_FORCE_INLINE const Cm::SpatialVectorV* getRefVelocity(const FsData& matrix)
{
	return addAddr<const Cm::SpatialVectorV*>(getLtbRows(matrix), sizeof(LtbRow)*matrix.linkCount);
}

PX_FORCE_INLINE FsRowAux* getAux(FsData& matrix)
{
	return addAddr<FsRowAux*>(getRefVelocity(matrix),sizeof(Cm::SpatialVectorV)*matrix.linkCount);
}

PX_FORCE_INLINE const FsRowAux* getAux(const FsData& matrix)
{
	return addAddr<const FsRowAux*>(getRefVelocity(matrix),sizeof(Cm::SpatialVectorV)*matrix.linkCount);
}

//void PxcFsApplyImpulse(ArticulationV& articulation,
//					   PxU32 linkID,
//					   Vec3V linear,
//					   Vec3V angular);

//Cm::SpatialVectorV PxcFsGetVelocity(ArticulationV& articulation,
//							    PxU32 linkID);

Cm::SpatialVectorV PxcFsGetMotionVector(ArticulationV& articulation,
	PxU32 linkID);


#if DY_ARTICULATION_DEBUG_VERIFY
namespace ArticulationRef 
{	
	Cm::SpatialVector propagateVelocity(const FsRow& row,
										const FsJointVectors& jv,
										const PxVec3& SZ, 
										const Cm::SpatialVector& v, 
										const FsRowAux& aux); 

	Cm::SpatialVector propagateImpulse(const FsRow& row, 
									   const FsJointVectors& jv,
									   PxVec3& SZ, 
									   const Cm::SpatialVector& Z,	
									   const FsRowAux& aux); 

	void applyImpulse(const FsData& matrix,
					  Cm::SpatialVector* velocity,
					  PxU32 linkID, 
					  const Cm::SpatialVector& impulse);

}
#endif

}
}

#endif //DY_ARTICULATION_H
