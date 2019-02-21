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


#include "DyArticulationUtils.h"
#include "DyArticulationScalar.h"
#include "DyArticulationReference.h"
#include "DyArticulationFnsDebug.h"

namespace physx
{
namespace Dy
{
namespace ArticulationRef
{
	Cm::SpatialVector propagateImpulse(const FsRow& row, 
									   const FsJointVectors& j,
									   PxVec3& SZ,
									   const Cm::SpatialVector& Z,
									   const FsRowAux& aux)
	{
		typedef ArticulationFnsScalar Fns;
	
		SZ = Fns::axisDot(reinterpret_cast<const Cm::SpatialVector*>(aux.S), Z);
		return Fns::translateForce(getParentOffset(j), Z - Fns::axisMultiply(getDSI(row), SZ));
	}

	Cm::SpatialVector propagateVelocity(const FsRow& row,
									    const FsJointVectors& j,
										const PxVec3& SZ,
										const Cm::SpatialVector& v,
										const FsRowAux& aux)
	{
		typedef ArticulationFnsScalar Fns;

		Cm::SpatialVector w = Fns::translateMotion(-getParentOffset(j), v);
		PxVec3 DSZ = Fns::multiply(row.D, SZ);

		return w - Fns::axisMultiply(reinterpret_cast<const Cm::SpatialVector*>(aux.S), DSZ + Fns::axisDot(getDSI(row), w));
	}

	void applyImpulse(const FsData& matrix, 
					  Cm::SpatialVector* velocity,
					  PxU32 linkID, 
					  const Cm::SpatialVector& impulse)
	{
		typedef ArticulationFnsScalar Fns;

		PX_ASSERT(matrix.linkCount<=DY_ARTICULATION_MAX_SIZE);

		const FsRow* rows = getFsRows(matrix);
		const FsRowAux* aux = getAux(matrix);
		const FsJointVectors* jointVectors = getJointVectors(matrix);

		Cm::SpatialVector dV[DY_ARTICULATION_MAX_SIZE];
		PxVec3 SZ[DY_ARTICULATION_MAX_SIZE];

		for(PxU32 i=0;i<matrix.linkCount;i++)
			SZ[i] = PxVec3(0);

		Cm::SpatialVector Z = -impulse;

		for(;linkID!=0; linkID = matrix.parent[linkID])
			Z = ArticulationRef::propagateImpulse(rows[linkID], jointVectors[linkID], SZ[linkID], Z, aux[linkID]);

		dV[0] = Fns::getRootDeltaV(matrix,-Z);

		for(PxU32 i=1;i<matrix.linkCount; i++)
			dV[i] = ArticulationRef::propagateVelocity(rows[i], jointVectors[i], SZ[i], dV[matrix.parent[i]], aux[i]);

		for(PxU32 i=0;i<matrix.linkCount;i++)
			velocity[i] += dV[i];
	}

	void ltbFactor(FsData& m)
	{
		typedef ArticulationFnsScalar Fns;
		LtbRow* rows = getLtbRows(m);

		SpInertia inertia[DY_ARTICULATION_MAX_SIZE];
		for(PxU32 i=0;i<m.linkCount;i++)
			inertia[i] = ArticulationFnsDebug::unsimdify(rows[i].inertia);

		Cm::SpatialVector j[3];
		for(PxU32 i=m.linkCount; --i>0;)
		{
			LtbRow& b = rows[i];
			inertia[i] = Fns::invertInertia(inertia[i]);
			PxU32 p = m.parent[i];

			Cm::SpatialVector* j0 = &reinterpret_cast<Cm::SpatialVector&>(*b.j0),
							 * j1 = &reinterpret_cast<Cm::SpatialVector&>(*b.j1);

			Fns::multiply(j, inertia[i], j1);
			PxMat33 jResponse = Fns::invertSym33(-Fns::multiplySym(j, j1));
			j1[0] = j[0]; j1[1] = j[1]; j1[2] = j[2];

			b.jResponse = Mat33V_From_PxMat33(jResponse);
			Fns::multiply(j, j0, jResponse);
			inertia[p] = Fns::multiplySubtract(inertia[p], j, j0);
			j0[0] = j[0]; j0[1] = j[1]; j0[2] = j[2];
		}

		rows[0].inertia = Fns::invertInertia(inertia[0]);
		for(PxU32 i=1;i<m.linkCount;i++)
			rows[i].inertia = inertia[i];
	}


}

#if 0


void ltbSolve(const FsData& m, 
			  Vec3V* c,					// rhs error to solve for
			  Cm::SpatialVector* y)		// velocity delta output
{
	typedef ArticulationFnsScalar Fns;

	PxVec4* b = reinterpret_cast<PxVec4*>(c);
	const LtbRow* rows = getLtbRows(m);
	PxMemZero(y, m.linkCount*sizeof(Cm::SpatialVector));

	for(PxU32 i=m.linkCount;i-->1;)
	{
		PxU32 p = m.parent[i];
		const LtbRow& r = rows[i];
		b[i] -= PxVec4(Fns::axisDot(&static_cast<const Cm::SpatialVector&>(*r.j1), y[i]),0);
		y[p] -= Fns::axisMultiply(&static_cast<const Cm::SpatialVector&>(*r.j0), b[i].getXYZ());
	}

	y[0] = Fns::multiply(rows[0].inertia,y[0]);

	for(PxU32 i=1; i<m.linkCount; i++)
	{
		PxU32 p = m.parent[i];
		const LtbRow& r = rows[i];
		PxVec3 t = Fns::multiply(r.jResponse, b[i].getXYZ()) - Fns::axisDot(&static_cast<const Cm::SpatialVector&>(*r.j0), y[p]);
		y[i] = Fns::multiply(r.inertia, y[i]) - Fns::axisMultiply(&static_cast<const Cm::SpatialVector&>(*r.j1), t);
	}
}

void PxcFsPropagateDrivenInertiaScalar(FsData& matrix,
								 const FsInertia* baseInertia,
								 const PxReal* isf,
								 const Mat33V* load)
{
	typedef ArticulationFnsScalar Fns;

	Cm::SpatialVector IS[3], DSI[3];
	PxMat33 D;

	FsRow* rows = getFsRows(matrix);
	const FsRowAux* aux = getAux(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

	SpInertia inertia[DY_ARTICULATION_MAX_SIZE];
	for(PxU32 i=0;i<matrix.linkCount;i++)
		inertia[i] = ArticulationFnsDebug::unsimdify(baseInertia[i]);

	for(PxU32 i=matrix.linkCount; --i>0;)
	{
		FsRow& r = rows[i];
		const FsRowAux& a = aux[i];
		const FsJointVectors& jv = jointVectors[i];

		Fns::multiply(IS, inertia[i], &static_cast<const Cm::SpatialVector&>(*a.S));

		PX_ALIGN(16, PxMat33) L;
		PxMat33_From_Mat33V(load[i], L);
		D = Fns::invertSym33(Fns::multiplySym(&static_cast<const Cm::SpatialVector&>(*a.S), IS) + L*isf[i]);

		Fns::multiply(DSI, IS, D);

		r.D = Mat33V_From_PxMat33(D);
		static_cast<Cm::SpatialVector&>(r.DSI[0]) = DSI[0];
		static_cast<Cm::SpatialVector&>(r.DSI[1]) = DSI[1];
		static_cast<Cm::SpatialVector&>(r.DSI[2]) = DSI[2];

		inertia[matrix.parent[i]] += Fns::translate(getParentOffset(jv), Fns::multiplySubtract(inertia[i], DSI, IS));
	}

	FsInertia& m = getRootInverseInertia(matrix);
	m = FsInertia(Fns::invertInertia(inertia[0]));
}

// no need to compile this ecxcept for verification, and it consumes huge amounts of stack space
void PxcFsComputeJointLoadsScalar(const FsData& matrix,
							      const FsInertia*PX_RESTRICT baseInertia,
							      Mat33V*PX_RESTRICT load,
   							      const PxReal*PX_RESTRICT isf,
							      PxU32 linkCount,
							      PxU32 maxIterations)
{
	typedef ArticulationFnsScalar Fns;

	// the childward S
	SpInertia leafwardInertia[DY_ARTICULATION_MAX_SIZE];
	SpInertia rootwardInertia[DY_ARTICULATION_MAX_SIZE];
	SpInertia inertia[DY_ARTICULATION_MAX_SIZE];
	SpInertia contribToParent[DY_ARTICULATION_MAX_SIZE];

	// total articulated inertia assuming the articulation is rooted here

	const FsRow* row = getFsRows(matrix);
	const FsRowAux* aux = getAux(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

	PX_UNUSED(row);

	PxMat33 load_[DY_ARTICULATION_MAX_SIZE];

	for(PxU32 iter=0;iter<maxIterations;iter++)
	{
		for(PxU32 i=0;i<linkCount;i++)
			inertia[i] = ArticulationFnsDebug::unsimdify(baseInertia[i]);
			
		for(PxU32 i=linkCount;i-->1;)
		{
			const FsJointVectors& j = jointVectors[i];

			leafwardInertia[i] = inertia[i];
			contribToParent[i] = Fns::propagate(inertia[i], &static_cast<const Cm::SpatialVector&>(*aux[i].S), load_[i], isf[i]);
			inertia[matrix.parent[i]] += Fns::translate((PxVec3&)j.parentOffset, contribToParent[i]);
		}

		for(PxU32 i=1;i<linkCount;i++)
		{
			rootwardInertia[i] = Fns::translate(-(PxVec3&)jointVectors[i].parentOffset, inertia[matrix.parent[i]]) - contribToParent[i];				
			inertia[i] += Fns::propagate(rootwardInertia[i], &static_cast<const Cm::SpatialVector&>(*aux[i].S), load_[i], isf[i]);
		}

		for(PxU32 i=1;i<linkCount;i++)
		{
			load_[i] = Fns::computeDriveInertia(leafwardInertia[i], rootwardInertia[i], &static_cast<const Cm::SpatialVector&>(*aux[i].S));
			PX_ASSERT(load_[i][0].isFinite() && load_[i][1].isFinite() && load_[2][i].isFinite());
		}					
	}
	for(PxU32 i=1;i<linkCount;i++)
		load[i] = Mat33V_From_PxMat33(load_[i]);
}


void PxcFsApplyImpulse(const FsData& matrix, 
					   PxU32 linkID, 
					   const Cm::SpatialVector& impulse)
{
#if DY_ARTICULATION_DEBUG_VERIFY
	PxcFsRefApplyImpulse(matrix, state.refVelocity, linkID, impulse);
#endif

	Cm::SpatialVector Z = -impulse;

	for(PxU32 i = linkID; i!=0; i = matrix.row[i].parent)
	{
		PxVec3 SZ;
		Z = propagateImpulse(matrix.row[i], SZ, Z, matrix.aux[i]);
		deferredSZRef(state,i) += SZ;
	}

	static_cast<Cm::SpatialVector &>(state.deferredZ) += Z;
	state.dirty |= matrix.row[linkID].pathToRoot;
}

Cm::SpatialVector PxcFsGetVelocity(const FsData& matrix,
								  PxU32 linkID)
{
	// find the dirty node on the path (including the root) with the lowest index
	ArticulationBitField toUpdate = matrix.row[linkID].pathToRoot & state.dirty;

	if(toUpdate)
	{
		ArticulationBitField ignoreNodes = (toUpdate & (0-toUpdate))-1;
		ArticulationBitField path = matrix.row[linkID].pathToRoot & ~ignoreNodes, p = path;
		ArticulationBitField newDirty = 0;

		Cm::SpatialVector dV = Cm::SpatialVector::zero();
		if(p & 1)
		{
			dV = getRootDeltaV(matrix, -deferredZ(state));

			velocityRef(state, 0) += dV;
			for(ArticulationBitField defer = matrix.row[0].children & ~path; defer; defer &= (defer-1))
				deferredVelRef(state, ArticulationLowestSetBit(defer)) += dV;

			deferredZRef(state) = Cm::SpatialVector::zero();
			newDirty = matrix.row[0].children;
			p--;
		}

		for(; p; p &= (p-1))
		{
			PxU32 i = ArticulationLowestSetBit(p);

			dV = propagateVelocity(matrix.row[i], deferredSZ(state,i), dV + state.deferredVel[i], matrix.aux[i]);

			velocityRef(state,i) += dV;
			for(ArticulationBitField defer = matrix.row[i].children & ~path; defer; defer &= (defer-1))
				deferredVelRef(state,ArticulationLowestSetBit(defer)) += dV;

			newDirty |= matrix.row[i].children;
			deferredVelRef(state,i) = Cm::SpatialVector::zero();
			deferredSZRef(state,i) = PxVec3(0);
		}

		state.dirty = (state.dirty | newDirty)&~path;
	}
#if DY_ARTICULATION_DEBUG_VERIFY
	Cm::SpatialVector v = state.velocity[linkID];
	Cm::SpatialVector rv = state.refVelocity[linkID];
	PX_ASSERT((v-rv).magnitude()<1e-4f * rv.magnitude());
#endif

	return state.velocity[linkID];
}

void PxcFsFlushVelocity(const FsData& matrix)
{
	Cm::SpatialVector V = getRootDeltaV(matrix, -deferredZ(state));
	deferredZRef(state) = Cm::SpatialVector::zero();
	velocityRef(state,0) += V;
	for(ArticulationBitField defer = matrix.row[0].children; defer; defer &= (defer-1))
		deferredVelRef(state,ArticulationLowestSetBit(defer)) += V;

	for(PxU32 i = 1; i<matrix.linkCount; i++)
	{
		Cm::SpatialVector V = propagateVelocity(matrix.row[i], deferredSZ(state,i), state.deferredVel[i], matrix.aux[i]);
		deferredVelRef(state,i) = Cm::SpatialVector::zero();
		deferredSZRef(state,i) = PxVec3(0);
		velocityRef(state,i) += V;
		for(ArticulationBitField defer = matrix.row[i].children; defer; defer &= (defer-1))
			deferredVelRef(state,ArticulationLowestSetBit(defer)) += V;
	}

	state.dirty = 0;
}

void PxcFsPropagateDrivenInertiaScalar(FsData& matrix,
									   const FsInertia* baseInertia,
									   const PxReal* isf,
									   const Mat33V* load,
									   PxcFsScratchAllocator allocator)
{
	typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;

	Cm::SpatialVectorV IS[3];
	PxMat33 D;

	FsRow* rows = getFsRows(matrix);
	const FsRowAux* aux = getAux(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

	FsInertia *inertia = allocator.alloc<FsInertia>(matrix.linkCount);
	PxMemCopy(inertia, baseInertia, matrix.linkCount*sizeof(FsInertia));

	for(PxU32 i=matrix.linkCount; --i>0;)
	{
		FsRow& r = rows[i];
		const FsRowAux& a = aux[i];
		const FsJointVectors& jv = jointVectors[i];

		Mat33V m = Fns::computeSIS(inertia[i], a.S, IS);
		FloatV f = FLoad(isf[i]);

		Mat33V D = Fns::invertSym33(Mat33V(V3ScaleAdd(load[i].col0, f, m.col0),
										   V3ScaleAdd(load[i].col1, f, m.col1),
										   V3ScaleAdd(load[i].col2, f, m.col2)));
		r.D = D;

		inertia[matrix.parent[i]] = Fns::addInertia(inertia[matrix.parent[i]], 
													Fns::translateInertia(jv.parentOffset, Fns::multiplySubtract(inertia[i], D,  IS,  r.DSI)));
	}

	getRootInverseInertia(matrix) = Fns::invertInertia(inertia[0]);
}

void PxcLtbSolve(const FsData& m, 
				 Vec3V* c,					// rhs error to solve for
				 Cm::SpatialVector* y)		// velocity delta output
{
	typedef ArticulationFnsScalar Fns;

	PxVec4* b = reinterpret_cast<PxVec4*>(c);
	const LtbRow* rows = getLtbRows(m);
	PxMemZero(y, m.linkCount*sizeof(Cm::SpatialVector));

	for(PxU32 i=m.linkCount;i-->1;)
	{
		PxU32 p = m.parent[i];
		const LtbRow& r = rows[i];
		b[i] -= PxVec4(Fns::axisDot(&static_cast<const Cm::SpatialVector&>(*r.j1), y[i]),0);
		y[p] -= Fns::axisMultiply(&static_cast<const Cm::SpatialVector&>(*r.j0), b[i].getXYZ());
	}

	y[0] = Fns::multiply(rows[0].inertia,y[0]);

	for(PxU32 i=1; i<m.linkCount; i++)
	{
		PxU32 p = m.parent[i];
		const LtbRow& r = rows[i];
		PxVec3 t = Fns::multiply(r.jResponse, b[i].getXYZ()) - Fns::axisDot(&static_cast<const Cm::SpatialVector&>(*r.j0), y[p]);
		y[i] = Fns::multiply(r.inertia, y[i]) - Fns::axisMultiply(&static_cast<const Cm::SpatialVector&>(*r.j1), t);
	}
}


#endif


#if DY_ARTICULATION_DEBUG_VERIFY
void PxcLtbFactorScalar(FsData& m)
{
	typedef ArticulationFnsScalar Fns;
	LtbRow* rows = getLtbRows(m);

	SpInertia inertia[DY_ARTICULATION_MAX_SIZE];
	for(PxU32 i=0;i<m.linkCount;i++)
		inertia[i] = ArticulationFnsDebug::unsimdify(rows[i].inertia);

	Cm::SpatialVector j[3];
	for(PxU32 i=m.linkCount; --i>0;)
	{
		LtbRow& b = rows[i];
		inertia[i] = Fns::invertInertia(inertia[i]);
		PxU32 p = m.parent[i];

		Cm::SpatialVector* j0 = &reinterpret_cast<Cm::SpatialVector&>(*b.j0),
						 * j1 = &reinterpret_cast<Cm::SpatialVector&>(*b.j1);

		Fns::multiply(j, inertia[i], j1);
		PxMat33 jResponse = Fns::invertSym33(-Fns::multiplySym(j, j1));
		j1[0] = j[0]; j1[1] = j[1]; j1[2] = j[2];

		b.jResponse = Mat33V_From_PxMat33(jResponse);
		Fns::multiply(j, j0, jResponse);
		inertia[p] = Fns::multiplySubtract(inertia[p], j, j0);
		j0[0] = j[0]; j0[1] = j[1]; j0[2] = j[2];
	}

	rows[0].inertia = Fns::invertInertia(inertia[0]);
	for(PxU32 i=1;i<m.linkCount;i++)
		rows[i].inertia = inertia[i];
}

void PxcFsPropagateDrivenInertiaScalar(FsData& matrix,
									   const FsInertia* baseInertia,
									   const PxReal* isf,
									   const Mat33V* load)
{
	typedef ArticulationFnsScalar Fns;

	Cm::SpatialVector IS[3], DSI[3];
	PxMat33 D;

	FsRow* rows = getFsRows(matrix);
	const FsRowAux* aux = getAux(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

	SpInertia inertia[DY_ARTICULATION_MAX_SIZE];
	for(PxU32 i=0;i<matrix.linkCount;i++)
		inertia[i] = ArticulationFnsDebug::unsimdify(baseInertia[i]);

	for(PxU32 i=matrix.linkCount; --i>0;)
	{
		FsRow& r = rows[i];
		const FsRowAux& a = aux[i];
		const FsJointVectors& jv = jointVectors[i];

		Fns::multiply(IS, inertia[i], &reinterpret_cast<const Cm::SpatialVector&>(*a.S));

		PX_ALIGN(16, PxMat33) L;
		PxMat33_From_Mat33V(load[i], L);
		D = Fns::invertSym33(Fns::multiplySym(&reinterpret_cast<const Cm::SpatialVector&>(*a.S), IS) + L*isf[i]);

		Fns::multiply(DSI, IS, D);

		r.D = Mat33V_From_PxMat33(D);
		reinterpret_cast<Cm::SpatialVector&>(r.DSI[0]) = DSI[0];
		reinterpret_cast<Cm::SpatialVector&>(r.DSI[1]) = DSI[1];
		reinterpret_cast<Cm::SpatialVector&>(r.DSI[2]) = DSI[2];

		inertia[matrix.parent[i]] += Fns::translate(getParentOffset(jv), Fns::multiplySubtract(inertia[i], DSI, IS));
	}

	FsInertia& m = getRootInverseInertia(matrix);
	m = FsInertia(Fns::invertInertia(inertia[0]));
}

// no need to compile this ecxcept for verification, and it consumes huge amounts of stack space
void PxcFsComputeJointLoadsScalar(const FsData& matrix,
								  const FsInertia*PX_RESTRICT baseInertia,
								  Mat33V*PX_RESTRICT load,
								  const PxReal*PX_RESTRICT isf,
								  PxU32 linkCount,
								  PxU32 maxIterations)
{
	typedef ArticulationFnsScalar Fns;

	// the childward S
	SpInertia leafwardInertia[DY_ARTICULATION_MAX_SIZE];
	SpInertia rootwardInertia[DY_ARTICULATION_MAX_SIZE];
	SpInertia inertia[DY_ARTICULATION_MAX_SIZE];
	SpInertia contribToParent[DY_ARTICULATION_MAX_SIZE];

	// total articulated inertia assuming the articulation is rooted here

	const FsRow* row = getFsRows(matrix);
	const FsRowAux* aux = getAux(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

	PX_UNUSED(row);

	PxMat33 load_[DY_ARTICULATION_MAX_SIZE];

	for(PxU32 iter=0;iter<maxIterations;iter++)
	{
		for(PxU32 i=0;i<linkCount;i++)
			inertia[i] = ArticulationFnsDebug::unsimdify(baseInertia[i]);
			
		for(PxU32 i=linkCount;i-->1;)
		{
			const FsJointVectors& j = jointVectors[i];

			leafwardInertia[i] = inertia[i];
			contribToParent[i] = Fns::propagate(inertia[i], &reinterpret_cast<const Cm::SpatialVector&>(*aux[i].S), load_[i], isf[i]);
			inertia[matrix.parent[i]] += Fns::translate((PxVec3&)j.parentOffset, contribToParent[i]);
		}

		for(PxU32 i=1;i<linkCount;i++)
		{
			rootwardInertia[i] = Fns::translate(-(PxVec3&)jointVectors[i].parentOffset, inertia[matrix.parent[i]]) - contribToParent[i];				
			inertia[i] += Fns::propagate(rootwardInertia[i], &reinterpret_cast<const Cm::SpatialVector&>(*aux[i].S), load_[i], isf[i]);
		}

		for(PxU32 i=1;i<linkCount;i++)
		{
			load_[i] = Fns::computeDriveInertia(leafwardInertia[i], rootwardInertia[i], &reinterpret_cast<const Cm::SpatialVector&>(*aux[i].S));
			PX_ASSERT(load_[i][0].isFinite() && load_[i][1].isFinite() && load_[2][i].isFinite());
		}					
	}
	for(PxU32 i=1;i<linkCount;i++)
		load[i] = Mat33V_From_PxMat33(load_[i]);
}
#endif

}

}
