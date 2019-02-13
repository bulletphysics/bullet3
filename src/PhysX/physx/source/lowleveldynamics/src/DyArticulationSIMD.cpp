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


#include "foundation/PxPreprocessor.h"
#include "DySpatial.h"
#include "DyArticulation.h"
#include "DyArticulationScalar.h"
#include "DyArticulationFnsScalar.h"
#include "DyArticulationReference.h"
#include "DyArticulationFnsSimd.h"


namespace physx
{
namespace Dy
{

#if DY_ARTICULATION_DEBUG_VERIFY
namespace 
{
	Cm::SpatialVector SpV(Vec3V linear, Vec3V angular) 
	{
		return Cm::SpatialVector((PxVec3 &)linear, (PxVec3&)angular);
	}
}
#endif

//void PxcFsApplyImpulse(ArticulationV &articulationV,
//					   PxU32 linkID, 
//					   Vec3V linear,
//					   Vec3V angular)
//{
//#if DY_ARTICULATION_DEBUG_VERIFY
//	{	
//		Cm::SpatialVectorV imp(linear, angular);
//		ArticulationRef::applyImpulse(matrix, reinterpret_cast<Cm::SpatialVector *>(getRefVelocity(matrix)), linkID, reinterpret_cast<Cm::SpatialVector&>(imp));
//	}
//#endif
//	Articulation& articulation = static_cast<>(articulationV
//	FsData& matrix = *articulation.getFsDataPtr();
//	Vec3V linZ = V3Neg(linear);
//	Vec3V angZ = V3Neg(angular);
//
//	const FsRow *rows = getFsRows(matrix);
//	const FsJointVectors* jointVectors = getJointVectors(matrix);
//
//#if DY_ARTICULATION_DEBUG_VERIFY
//	const FsRowAux *aux = getAux(matrix);
//#endif
//	Vec3V *deferredSZ = getDeferredSZ(matrix);
//
//	for(PxU32 i = linkID; i!=0; i = matrix.parent[i])
//	{
//		const FsRow &row = rows[i];
//		const FsJointVectors& jv = jointVectors[i];
//
//#if DY_ARTICULATION_DEBUG_VERIFY
//		PxVec3 SZcheck;
//		Cm::SpatialVector Zcheck = ArticulationRef::propagateImpulse(row, jv, SZcheck, SpV(linZ, angZ), aux[i]);
//#endif
//
//		Vec3V SZ = V3Add(angZ, V3Cross(linZ, jv.jointOffset));
//		Vec3V lrLinear = V3Sub(linZ,  V3ScaleAdd(row.DSI[0].linear, V3GetX(SZ),
//									   V3ScaleAdd(row.DSI[1].linear, V3GetY(SZ),
//									       V3Scale(row.DSI[2].linear, V3GetZ(SZ)))));
//
//		Vec3V lrAngular = V3Sub(angZ,  V3ScaleAdd(row.DSI[0].angular, V3GetX(SZ),
//									    V3ScaleAdd(row.DSI[1].angular, V3GetY(SZ),
//									        V3Scale(row.DSI[2].angular, V3GetZ(SZ)))));
//
//		linZ = lrLinear;
//		angZ = V3Add(lrAngular, V3Cross(jv.parentOffset, lrLinear));
//		deferredSZ[i] = V3Add(deferredSZ[i], SZ);
//
//		PX_ASSERT(Ps::aos::isFiniteVec3V(linZ));
//		PX_ASSERT(Ps::aos::isFiniteVec3V(angZ));
//
//#if DY_ARTICULATION_DEBUG_VERIFY
//		Cm::SpatialVector Z = SpV(linZ,angZ);
//		PX_ASSERT((Z - Zcheck).magnitude()<1e-4*PxMax(Zcheck.magnitude(), 1.0f));
//		PX_ASSERT(((PxVec3&)SZ-SZcheck).magnitude()<1e-4*PxMax(SZcheck.magnitude(), 1.0f));
//#endif
//	}
//
//	matrix.deferredZ.linear = V3Add(matrix.deferredZ.linear, linZ);
//	matrix.deferredZ.angular = V3Add(matrix.deferredZ.angular, angZ);
//
//	matrix.dirty |= rows[linkID].pathToRoot;
//}

//Cm::SpatialVectorV PxcFsGetVelocity(Articulation &articulation,
//								PxU32 linkID)
//{
//	FsData& matrix = *articulation.getFsDataPtr();
//	const FsRow *rows = getFsRows(matrix);
//	const FsJointVectors* jointVectors = getJointVectors(matrix);
//
//#if DY_ARTICULATION_DEBUG_VERIFY
//	const FsRowAux *aux = getAux(matrix);
//#endif
//	Cm::SpatialVectorV* PX_RESTRICT V = getVelocity(matrix);
//
//	// find the dirty node on the path (including the root) with the lowest index
//	ArticulationBitField toUpdate = rows[linkID].pathToRoot & matrix.dirty;
//
//
//	if(toUpdate)
//	{
//		// store the dV elements densely and use an array map to decode - hopefully cache friendlier
//		PxU32 indexToStackLoc[DY_ARTICULATION_MAX_SIZE], count = 0;
//		Cm::SpatialVectorV dVStack[DY_ARTICULATION_MAX_SIZE];
//
//		ArticulationBitField ignoreNodes = (toUpdate & (0-toUpdate))-1;
//		ArticulationBitField path = rows[linkID].pathToRoot & ~ignoreNodes, p = path;
//		ArticulationBitField newDirty = 0;
//
//		Vec3V ldV = V3Zero(), adV = V3Zero();
//		Cm::SpatialVectorV* PX_RESTRICT defV = getDeferredVel(matrix);
//		Vec3V* PX_RESTRICT SZ = getDeferredSZ(matrix);
//
//		if(p & 1)
//		{
//			const FsInertia &m = getRootInverseInertia(matrix);
//			Vec3V lZ = V3Neg(matrix.deferredZ.linear);
//			Vec3V aZ = V3Neg(matrix.deferredZ.angular);
//
//			ldV = V3Add(M33MulV3(m.ll,lZ),		 M33MulV3(m.la,aZ));
//			adV = V3Add(M33TrnspsMulV3(m.la,lZ), M33MulV3(m.aa,aZ));
//
//			V[0].linear = V3Add(V[0].linear, ldV);
//			V[0].angular = V3Add(V[0].angular, adV);
//
//			matrix.deferredZ.linear = V3Zero();
//			matrix.deferredZ.angular = V3Zero();
//
//			indexToStackLoc[0] = count;
//			Cm::SpatialVectorV &e = dVStack[count++];
//
//			e.linear = ldV;
//			e.angular = adV;
//
//			newDirty = rows[0].children;
//			p--;
//		}
//
//		
//		while(p)	// using "for(;p;p &= (p-1))" here generates LHSs from the ArticulationLowestSetBit
//		{
//			PxU32 i = ArticulationLowestSetBit(p);
//			const FsJointVectors& jv = jointVectors[i];
//
//			p &= (p-1);
//	
//			const FsRow* PX_RESTRICT row = rows + i;
//
//			ldV = V3Add(ldV, defV[i].linear);
//			adV = V3Add(adV, defV[i].angular);
//
//#if DY_ARTICULATION_DEBUG_VERIFY
//			Cm::SpatialVector dVcheck = ArticulationRef::propagateVelocity(*row, jv, (PxVec3&)SZ[i], SpV(ldV,adV), aux[i]);
//#endif
//
//			Vec3V DSZ = M33MulV3(row->D, SZ[i]);
//
//			Vec3V lW = V3Add(ldV, V3Cross(adV,jv.parentOffset));
//			Vec3V aW = adV;
//
//			const Cm::SpatialVectorV*PX_RESTRICT DSI = row->DSI;
//			Vec3V lN = V3Merge(V3Dot(DSI[0].linear, lW),   V3Dot(DSI[1].linear, lW),  V3Dot(DSI[2].linear, lW));
//			Vec3V aN = V3Merge(V3Dot(DSI[0].angular, aW),  V3Dot(DSI[1].angular, aW), V3Dot(DSI[2].angular, aW));
//
//			Vec3V n = V3Add(V3Add(lN, aN), DSZ);
//
//			ldV = V3Sub(lW, V3Cross(jv.jointOffset,n));
//			adV = V3Sub(aW, n);
//
//#if DY_ARTICULATION_DEBUG_VERIFY
//			Cm::SpatialVector dV = SpV(ldV,adV);
//			PX_ASSERT((dV-dVcheck).magnitude()<1e-4*PxMax(dVcheck.magnitude(), 1.0f));
//#endif
//
//			V[i].linear = V3Add(V[i].linear, ldV);
//			V[i].angular = V3Add(V[i].angular, adV);
//
//			defV[i].linear = V3Zero();
//			defV[i].angular = V3Zero();
//			SZ[i] = V3Zero();
//
//			indexToStackLoc[i] = count;
//			Cm::SpatialVectorV &e = dVStack[count++];
//			newDirty |= rows[i].children;
//
//			e.linear = ldV;
//			e.angular = adV;
//		}
//
//		for(ArticulationBitField defer = newDirty&~path; defer; defer &= (defer-1))
//		{
//			PxU32 i = ArticulationLowestSetBit(defer);
//			PxU32 parent = indexToStackLoc[matrix.parent[i]];
//
//			defV[i].linear = V3Add(defV[i].linear, dVStack[parent].linear);
//			defV[i].angular = V3Add(defV[i].angular, dVStack[parent].angular);
//		}
//
//		matrix.dirty = (matrix.dirty | newDirty)&~path;
//	}
//#if DY_ARTICULATION_DEBUG_VERIFY
//	Cm::SpatialVector v = reinterpret_cast<Cm::SpatialVector&>(V[linkID]);
//	Cm::SpatialVector rv = reinterpret_cast<Cm::SpatialVector&>(getRefVelocity(matrix)[linkID]);
//	PX_ASSERT((v-rv).magnitude()<1e-4f * PxMax(rv.magnitude(),1.0f));
//#endif
//
//	return V[linkID];
//}

Cm::SpatialVectorV PxcFsGetMotionVector(ArticulationV& articulation,
	PxU32 linkID)
{

	/*const Cm::SpatialVectorV* PX_RESTRICT V = articulation.getMotionVector();
	return V[linkID];*/

	return articulation.getLinkMotionVector(linkID);
}

PX_FORCE_INLINE Cm::SpatialVectorV propagateVelocitySIMD(const FsRow& row,
														const FsJointVectors& jv,
														const Vec3V& SZ,
														const Cm::SpatialVectorV& v,
														const FsRowAux& aux)
{
	PX_UNUSED(aux);

	typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;

	Cm::SpatialVectorV w(V3Add(v.linear, V3Cross(v.angular, jv.parentOffset)), v.angular);
	Vec3V DSZ = M33MulV3(row.D, SZ);

	Vec3V n = V3Add(Fns::axisDot(row.DSI, w), DSZ);
	Cm::SpatialVectorV result = w - Cm::SpatialVectorV(V3Cross(jv.jointOffset,n), n);

#if DY_ARTICULATION_DEBUG_VERIFY
	Cm::SpatialVector check = ArticulationRef::propagateVelocity(row, jv, reinterpret_cast<const PxVec3&>(SZ), reinterpret_cast<const Cm::SpatialVector&>(v), aux);
	PX_ASSERT((reinterpret_cast<const Cm::SpatialVector&>(result)-check).magnitude()<1e-4*PxMax(check.magnitude(), 1.0f));
#endif

	return result;
}

void PxcFsFlushVelocity(FsData& matrix)
{
	typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;

	const FsRow* PX_RESTRICT rows = getFsRows(matrix);
	const FsRowAux* PX_RESTRICT aux = getAux(matrix);
	const FsJointVectors*PX_RESTRICT jointVectors = getJointVectors(matrix);

	Cm::SpatialVectorV V0 =  Fns::multiply(getRootInverseInertia(matrix), -matrix.deferredZ);
	matrix.deferredZ = Cm::SpatialVectorV(PxZero);

	getVelocity(matrix)[0] += V0;
	for(ArticulationBitField defer = rows[0].children; defer; defer &= (defer-1))
		getDeferredVel(matrix)[ArticulationLowestSetBit(defer)] += V0;

	for(PxU32 i = 1; i<matrix.linkCount; i++)
	{
		Cm::SpatialVectorV V = propagateVelocitySIMD(rows[i], jointVectors[i], getDeferredSZ(matrix)[i], getDeferredVel(matrix)[i], aux[i]);
		getDeferredVel(matrix)[i] = Cm::SpatialVectorV(PxZero);
		getDeferredSZ(matrix)[i] = V3Zero();
		getVelocity(matrix)[i] += V;
		for(ArticulationBitField defer = rows[i].children; defer; defer &= (defer-1))
			getDeferredVel(matrix)[ArticulationLowestSetBit(defer)] += V;
	}

#if DY_ARTICULATION_DEBUG_VERIFY
	for(PxU32 i=0;i<matrix.linkCount;i++)
	{
		Cm::SpatialVector v = velocityRef(matrix,i), rv = reinterpret_cast<Cm::SpatialVector&>(getRefVelocity(matrix)[i]);
		Cm::SpatialVector diff = v-rv;
		PxReal m = rv.magnitude();
		PX_UNUSED(m);
		PX_ASSERT(diff.magnitude()<1e-4*PxMax(1.0f,m));
	}
#endif

	matrix.dirty = 0;
}
}
}
