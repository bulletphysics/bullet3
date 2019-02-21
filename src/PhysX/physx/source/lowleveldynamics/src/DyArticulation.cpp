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


#include "PsMathUtils.h"
#include "CmConeLimitHelper.h"
#include "DySolverConstraint1D.h"
#include "DyArticulation.h"
#include "DyArticulationHelper.h"
#include "PxsRigidBody.h"
#include "PxcConstraintBlockStream.h"
#include "DyArticulationContactPrep.h"
#include "DyDynamics.h"
#include "DyArticulationReference.h"
#include "DyArticulationPImpl.h"
#include <stdio.h>
#include "DyArticulationUtils.h"
#include "foundation/PxProfiler.h"
#include "DyArticulationFnsSimd.h"
#include "DyTGSDynamics.h"
#include "common/PxProfileZone.h"

using namespace physx;

// we encode articulation link handles in the lower bits of the pointer, so the
// articulation has to be aligned, which in an aligned pool means we need to size it
// appropriately

namespace physx
{
	namespace Dy
	{
		void SolverCoreRegisterArticulationFns();

		void SolverCoreRegisterArticulationFnsCoulomb();

		void PxcFsFlushVelocity(FsData& matrix);

		// we pass this around by value so that when we return from a function the size is unaltered. That means we don't preserve state
		// across functions - even though that could be handy to preserve baseInertia and jointTransforms across the solver so that if we 
		// need to run position projection  positions they don't get recomputed.

		void PxcLtbFactor(FsData& m)
		{
			typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;
			LtbRow* rows = getLtbRows(m);

			for (PxU32 i = m.linkCount; --i>0;)
			{
				LtbRow& b = rows[i];
				PxU32 p = m.parent[i];
				const FsInertia inertia = Fns::invertInertia(b.inertia);
				const Mat33V jResponse = Fns::invertSym33(M33Neg(Fns::computeSIS(inertia, b.j1, b.j1)));
				b.inertia = inertia;
				rows[p].inertia = Fns::multiplySubtract(rows[p].inertia, jResponse, b.j0, b.j0);
				b.jResponse = jResponse;

			}
			rows[0].inertia = Fns::invertInertia(rows[0].inertia);
		}

PX_COMPILE_TIME_ASSERT((sizeof(Articulation)&(DY_ARTICULATION_MAX_SIZE-1))==0);

Articulation::Articulation(Sc::ArticulationSim* sim)
:	ArticulationV(sim, PxArticulationBase::eMaximumCoordinate)
,	mFsDataBytes(PX_DEBUG_EXP("Articulation::fsData"))
,	mInternalLoads(PX_DEBUG_EXP("ScArticulationSim::internalLoads"))
,	mExternalLoads(PX_DEBUG_EXP("ScArticulationSim::externalLoads"))
,	mScratchMemory(PX_DEBUG_EXP("ScArticulationSim::scratchMemory"))
,	mPose(PX_DEBUG_EXP("ScArticulationSim::poses"))
,	mDeltaQ(PX_DEBUG_EXP("ScArticulationSim::poses"))
,	mMotionVelocity(PX_DEBUG_EXP("ScArticulationSim::motion velocity"))
{
	PX_ASSERT((reinterpret_cast<size_t>(this) & (DY_ARTICULATION_MAX_SIZE-1))==0);
}

Articulation::~Articulation()
{
}

#if DY_DEBUG_ARTICULATION

void Articulation::computeResiduals(const Cm::SpatialVector *v, 
									   const ArticulationJointTransforms* jointTransforms,
									   bool /*dump*/) const
{
	typedef ArticulationFnsScalar Fns;

	PxReal error = 0, energy = 0;
	for(PxU32 i=1;i<mSolverDesc->linkCount;i++)
	{
		const ArticulationJointTransforms &b = jointTransforms[i];
		PxU32 parent = mSolverDesc->links[i].parent;
		const ArticulationJointCore &j = *mSolverDesc->links[i].inboundJoint;
		PX_UNUSED(j);

		Cm::SpatialVector residual = Fns::translateMotion(mSolverDesc->poses[i].p - b.cB2w.p, v[i])
								   - Fns::translateMotion(mSolverDesc->poses[parent].p - b.cB2w.p, v[parent]);

		error += residual.linear.magnitudeSquared();
		energy += residual.angular.magnitudeSquared();

	}
//	if(dump)
		printf("Energy %f, Error %f\n", energy, error);
}

Cm::SpatialVector Articulation::computeMomentum(const FsInertia *inertia) const
{
	typedef ArticulationFnsScalar Fns;

	Cm::SpatialVector *velocity = reinterpret_cast<Cm::SpatialVector*>(getVelocity(*mSolverDesc->fsData));
	Cm::SpatialVector m = Cm::SpatialVector::zero();
	for(PxU32 i=0;i<mSolverDesc->linkCount;i++)
		m += Fns::translateForce(mSolverDesc->poses[i].p - mSolverDesc->poses[0].p, ArticulationFnsScalar::multiply(inertia[i], velocity[i]));
	return m;
}

void Articulation::checkLimits() const
{
	for(PxU32 i=1;i<mSolverDesc->linkCount;i++)
	{
		PxTransform cA2w = mSolverDesc->poses[mSolverDesc->links[i].parent].transform(mSolverDesc->links[i].inboundJoint->parentPose);
		PxTransform cB2w = mSolverDesc->poses[i].transform(mSolverDesc->links[i].inboundJoint->childPose);
		
		PxTransform cB2cA = cA2w.transformInv(cB2w);

		// the relative quat must be the short way round for limits to work...

		if(cB2cA.q.w<0)
			cB2cA.q	= -cB2cA.q;

		const ArticulationJointCore& j = *mSolverDesc->links[i].inboundJoint;
		
		PxQuat swing, twist;
		if(j.twistLimited || j.swingLimited)
			Ps::separateSwingTwist(cB2cA.q, swing, twist);
		
		if(j.swingLimited)
		{
			PxReal swingLimitContactDistance = PxMin(j.swingYLimit, j.swingZLimit)/4;

			Cm::ConeLimitHelper eh(PxTan(j.swingYLimit/4), 
								   PxTan(j.swingZLimit/4),
								   PxTan(swingLimitContactDistance/4));

			PxVec3 axis;
			PxReal error = 0.0f;
			if(eh.getLimit(swing, axis, error))
				printf("%u, (%f, %f), %f, (%f, %f, %f), %f\n", i, j.swingYLimit, j.swingZLimit, swingLimitContactDistance, axis.x, axis.y, axis.z, error);
		}

//		if(j.twistLimited)
//		{
//			PxReal tqTwistHigh = PxTan(j.twistLimitHigh/4),
//				   tqTwistLow  = PxTan(j.twistLimitLow/4),
//				   twistPad = (tqTwistHigh - tqTwistLow)*0.25f;
//				   //twistPad = j.twistLimitContactDistance;
//
//			PxVec3 axis = jointTransforms[i].cB2w.rotate(PxVec3(1,0,0));
//			PxReal tqPhi = Ps::tanHalf(twist.x, twist.w);
//
//			if(tqPhi < tqTwistLow + twistPad)
//				constraintData.pushBack(ConstraintData(-axis, -(tqTwistLow - tqPhi)*4));
//
//			if(tqPhi > tqTwistHigh - twistPad)
//				constraintData.pushBack(ConstraintData(axis, (tqTwistHigh - tqPhi)*4));
//		}
	}
	puts("");
}

#endif

bool Dy::Articulation::resize(const PxU32 linkCount)
{
	if (mUpdateSolverData)
	{
		if (linkCount != mSolverDesc.linkCount)
		{
			PxU32 solverDataSize, totalSize, scratchSize;
			getDataSizes(linkCount, solverDataSize, totalSize, scratchSize);

			PX_ASSERT(mFsDataBytes.size() != totalSize);
			PX_ASSERT(!(totalSize & 15) && !(solverDataSize & 15));
			mFsDataBytes.resize(totalSize);

			mExternalLoads.resize(linkCount, Ps::aos::M33Identity());
			mInternalLoads.resize(linkCount, Ps::aos::M33Identity());
			mPose.resize(linkCount, PxTransform(PxIdentity));

			mDeltaQ.resize(linkCount, PxQuat(PxIdentity));

			mSolverDesc.externalLoads = mExternalLoads.begin();
			mSolverDesc.internalLoads = mInternalLoads.begin();

			mScratchMemory.resize(scratchSize);
			mSolverDesc.scratchMemory = mScratchMemory.begin();
			mSolverDesc.scratchMemorySize = Ps::to16(scratchSize);

			mSolverDesc.solverDataSize = Ps::to16(solverDataSize);
			mSolverDesc.totalDataSize = Ps::to16(totalSize);
			mSolverDesc.poses = mPose.begin();
			mSolverDesc.deltaQ = mDeltaQ.begin();

			mMotionVelocity.resize(linkCount, Cm::SpatialVector(PxVec3(0.0f), PxVec3(0.0f)));

			mSolverDesc.motionVelocity = mMotionVelocity.begin();
		}
		Dy::ArticulationV::resize(linkCount);
		
		return true;
	}

	return false;
}

bool Dy::ArticulationV::resize(const PxU32 linkCount)
{
	if (!mUpdateSolverData)
		return false;

	if (linkCount != mSolverDesc.linkCount)
	{
		mSolverDesc.acceleration = mAcceleration.begin();
		mSolverDesc.articulation = this;

	}
	mUpdateSolverData = false;
	return true;
}

void PxvRegisterArticulations()
{
	const PxU32 type = PxU32(PxArticulationBase::eMaximumCoordinate);
	ArticulationPImpl::sComputeUnconstrainedVelocities[type] = &Articulation::computeUnconstrainedVelocities;
	ArticulationPImpl::sUpdateBodies[type] = &Articulation::updateBodies;
	ArticulationPImpl::sUpdateBodiesTGS[type] = &Articulation::updateBodies;
	ArticulationPImpl::sSaveVelocity[type] = &Articulation::saveVelocity;
	ArticulationPImpl::sSaveVelocityTGS[type] = &Articulation::saveVelocityTGS;

	ArticulationPImpl::sUpdateDeltaMotion[type] = &Articulation::recordDeltaMotion;
	ArticulationPImpl::sDeltaMotionToMotionVel[type] = &Articulation::deltaMotionToMotionVelocity;
	ArticulationPImpl::sComputeUnconstrainedVelocitiesTGS[type] = &Articulation::computeUnconstrainedVelocitiesTGS;
	ArticulationPImpl::sSetupInternalConstraintsTGS[type] = &Articulation::setupSolverConstraintsTGS;

	SolverCoreRegisterArticulationFns();
	SolverCoreRegisterArticulationFnsCoulomb();
}

void Articulation::getDataSizes(PxU32 linkCount, PxU32& solverDataSize, PxU32& totalSize, PxU32& scratchSize)
{
	solverDataSize = sizeof(FsData)													// header
				   + sizeof(Cm::SpatialVectorV)	* linkCount								// velocity
				   + sizeof(Cm::SpatialVectorV)	* linkCount								// deferredVelocity
				   + sizeof(Cm::SpatialVectorV)	* linkCount								// motion size
				   + sizeof(Vec3V)				* linkCount								// deferredSZ
				   + sizeof(PxReal)				* ((linkCount + 15) & 0xFFFFFFF0)		// The maxPenBias values
				   + sizeof(FsJointVectors)	* linkCount								// joint offsets
			   	   + sizeof(FsInertia)												// featherstone root inverse inertia
				   + sizeof(FsRow)			* linkCount;							// featherstone matrix rows

	totalSize = solverDataSize
			  + sizeof(LtbRow)		 * linkCount			// lagrange matrix rows
			  + sizeof(Cm::SpatialVectorV) * linkCount			// ref velocity
			  + sizeof(FsRowAux)	 * linkCount;

	scratchSize = PxU32(sizeof(FsInertia)*linkCount*3
		        + ((sizeof(ArticulationJointTransforms)+15)&~15) * linkCount
				+ sizeof(Mat33V) * linkCount
				+ ((sizeof(ArticulationJointTransforms)+15)&~15) * linkCount);
}

void Articulation::getImpulseResponse(
	PxU32 linkID,
	Cm::SpatialVectorF* /*Z*/,
	const Cm::SpatialVector& impulse,
	Cm::SpatialVector& deltaV) const
{
	const FsData& matrix = *getFsDataPtr();
	ArticulationHelper::getImpulseResponse(matrix, linkID, impulse, deltaV);
}

void Articulation::getImpulseSelfResponse(
	PxU32 linkID0,
	PxU32 linkID1,
	Cm::SpatialVectorF* /*Z*/,
	const Cm::SpatialVector& impulse0,
	const Cm::SpatialVector& impulse1,
	Cm::SpatialVector& deltaV0,
	Cm::SpatialVector& deltaV1) const
{
	const FsData& matrix = *getFsDataPtr();
	ArticulationHelper::getImpulseSelfResponse(matrix, linkID0, reinterpret_cast<const Cm::SpatialVectorV&>(impulse0),
		reinterpret_cast<Cm::SpatialVectorV&>(deltaV0), linkID1,
		reinterpret_cast<const Cm::SpatialVectorV&>(impulse1),
		reinterpret_cast<Cm::SpatialVectorV&>(deltaV1));
}

Cm::SpatialVectorV Articulation::getLinkVelocity(const PxU32 linkID) const
{
	FsData& matrix = *getFsDataPtr();
	Cm::SpatialVectorV* velocites = addAddr<Cm::SpatialVectorV*>(&matrix, sizeof(FsData));
	return velocites[linkID];
}

Cm::SpatialVectorV Articulation::getLinkMotionVector(const PxU32 linkID) const
{
	FsData& matrix = *getFsDataPtr();
	Cm::SpatialVectorV* velocites = addAddr<Cm::SpatialVectorV*>(getDeferredVel(matrix), sizeof(Cm::SpatialVectorV) * matrix.linkCount);
	return velocites[linkID];
}

Cm::SpatialVector Articulation::getMotionVelocity(const PxU32 linkID) const
{
	PxVec3 linear, angular;
	const Cm::SpatialVectorV& motionVelocity = mMotionVelocity[linkID];
	V3StoreU(motionVelocity.linear, linear);
	V3StoreU(motionVelocity.angular, angular);

	return Cm::SpatialVector(linear, angular);
}

PxReal Articulation::getLinkMaxPenBias(const PxU32 linkID) const
{
	FsData& matrix = *getFsDataPtr();
	PxReal* maxPenBias = addAddr<PxReal*>(getDeferredSZ(matrix), sizeof(Vec3V) * matrix.linkCount);
	return maxPenBias[linkID];
}

PxU32 Articulation::getFsDataSize(PxU32 linkCount)
{
	return sizeof(FsInertia) + sizeof(FsRow) * linkCount;
}

PxU32 Articulation::getLtbDataSize(PxU32 linkCount)
{
	return sizeof(LtbRow) * linkCount;
}

void Articulation::setInertia(FsInertia& inertia, const PxsBodyCore& body, const PxTransform& pose)
{
	// assumes that elements that are supposed to be zero (i.e. la matrix and off diagonal elements of ll) are zero

	const PxMat33 R(pose.q);
	const PxVec3& v = body.inverseInertia;
	const PxReal m = 1.0f / body.inverseMass;
	V3WriteX(inertia.ll.col0, m);
	V3WriteY(inertia.ll.col1, m);
	V3WriteZ(inertia.ll.col2, m);

	PX_ALIGN_PREFIX(16) PxMat33 PX_ALIGN_SUFFIX(16) alignedInertia = R * PxMat33::createDiagonal(PxVec3(1.0f / v.x, 1.0f / v.y, 1.0f / v.z)) * R.getTranspose();
	alignedInertia = (alignedInertia + alignedInertia.getTranspose())*0.5f;
	inertia.aa = Mat33V_From_PxMat33(alignedInertia);
}

void Articulation::setJointTransforms(ArticulationJointTransforms& transforms,
	const PxTransform& parentPose,
	const PxTransform& childPose,
	const ArticulationJointCore& joint)
{
	transforms.cA2w = parentPose.transform(joint.parentPose);
	transforms.cB2w = childPose.transform(joint.childPose);
	transforms.cB2cA = transforms.cA2w.transformInv(transforms.cB2w);
	if (transforms.cB2cA.q.w<0)	// the relative quat must be the short way round for limits to work...
	{
		transforms.cB2cA.q = -transforms.cB2cA.q;
		transforms.cB2w.q = -transforms.cB2w.q;
	}
}

void Articulation::prepareDataBlock(FsData& fsData,
	const ArticulationLink* links,
	PxU16 linkCount,
	PxTransform* poses,
	PxQuat* deltaQ,
	FsInertia* baseInertia,
	ArticulationJointTransforms* jointTransforms,
	PxU32 expectedSize)
{
	PxU32 stateSize = sizeof(FsData)
		+ sizeof(Cm::SpatialVectorV) * linkCount
		+ sizeof(Cm::SpatialVectorV) * linkCount
		+ sizeof(Cm::SpatialVectorV) * linkCount
		+ sizeof(Vec3V)			 * linkCount
		+ sizeof(PxReal)		 * ((linkCount + 15) & 0xfffffff0);

	PxU32 jointVectorSize = sizeof(FsJointVectors) * linkCount;

	PxU32 fsDataSize = getFsDataSize(linkCount);
	PxU32 ltbDataSize = getLtbDataSize(linkCount);

	PxU32 totalSize = stateSize
		+ jointVectorSize
		+ fsDataSize
		+ ltbDataSize
		+ sizeof(Cm::SpatialVectorV) * linkCount
		+ sizeof(FsRowAux)    * linkCount;

	PX_UNUSED(totalSize);
	PX_UNUSED(expectedSize);
	PX_ASSERT(expectedSize == 0 || totalSize == expectedSize);

	PxMemZero(&fsData, stateSize);
	fsData.jointVectorOffset = PxU16(stateSize);
	fsData.fsDataOffset = PxU16(stateSize + jointVectorSize);
	fsData.ltbDataOffset = PxU16(stateSize + jointVectorSize + fsDataSize);
	fsData.linkCount = linkCount;

	for (PxU32 i = 1; i<linkCount; i++)
		fsData.parent[i] = PxU8(links[i].parent);
	fsData.deferredZ = Cm::SpatialVectorV(PxZero);

	Cm::SpatialVector* velocity = reinterpret_cast<Cm::SpatialVector*>(getVelocity(fsData));
	Cm::SpatialVector* motionVector = reinterpret_cast<Cm::SpatialVector*>(getMotionVector(fsData));

	PxMemZero(baseInertia, sizeof(FsInertia)*linkCount);

	PxReal* maxPenBias = getMaxPenBias(fsData);

	for (PxU32 i = 0; i<linkCount; i++)
	{
		if ((i + 2)<linkCount)
		{
			Ps::prefetch(links[i + 2].bodyCore);
			Ps::prefetch(links[i + 2].inboundJoint);
		}
		PxsBodyCore& core = *links[i].bodyCore;
		poses[i] = core.body2World;
		deltaQ[i] = PxQuat(PxIdentity);
		velocity[i] = Cm::SpatialVector(core.linearVelocity, core.angularVelocity);
		motionVector[i] = Cm::SpatialVector(PxVec3(0.f), PxVec3(0.f));
		setInertia(baseInertia[i], core, core.body2World);
		maxPenBias[i] = core.maxPenBias;

		if (i)
		{
			ArticulationJointCore* inboundJoint = static_cast<ArticulationJointCore*>(links[i].inboundJoint);
			setJointTransforms(jointTransforms[i], poses[links[i].parent], core.body2World, *inboundJoint);
		}
	}

	FsJointVectors* jointVectors = getJointVectors(fsData);
	for (PxU32 i = 1; i<linkCount; i++)
	{
		PX_ALIGN(16, PxVec3) parentOffset = poses[i].p - poses[fsData.parent[i]].p;
		PX_ALIGN(16, PxVec3) jointOffset = jointTransforms[i].cB2w.p - poses[i].p;
		jointVectors[i].parentOffset = V3LoadA(parentOffset);
		jointVectors[i].jointOffset = V3LoadA(jointOffset);
	}
}

void Articulation::prepareFsData(FsData& fsData, const ArticulationLink* links)
{
	typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;

	PxU32 linkCount = fsData.linkCount;
	FsRow* rows = getFsRows(fsData);
	FsRowAux* aux = getAux(fsData);
	const FsJointVectors* jointVectors = getJointVectors(fsData);

	rows[0].children = links[0].children;
	rows[0].pathToRoot = 1;

	PX_ALIGN_PREFIX(16) PxVec4 v[] PX_ALIGN_SUFFIX(16) = { PxVec4(1.f,0,0,0), PxVec4(0,1.f,0,0), PxVec4(0,0,1.f,0) };
	const Vec3V* axes = reinterpret_cast<const Vec3V*>(v);

	for (PxU32 i = 1; i<linkCount; i++)
	{
		PxU32 p = links[i].parent;
		FsRow& r = rows[i];
		FsRowAux& a = aux[i];

		PX_UNUSED(p);

		r.children = links[i].children;
		r.pathToRoot = links[i].pathToRoot;

		const Vec3V jointOffset = jointVectors[i].jointOffset;

		// the joint coords are world oriented, located at the joint.
		a.S[0] = Fns::translateMotion(jointOffset, Cm::SpatialVectorV(V3Zero(), axes[0]));
		a.S[1] = Fns::translateMotion(jointOffset, Cm::SpatialVectorV(V3Zero(), axes[1]));
		a.S[2] = Fns::translateMotion(jointOffset, Cm::SpatialVectorV(V3Zero(), axes[2]));
	}
}

PxReal Articulation::getResistance(PxReal compliance)
{
	PX_ASSERT(compliance>0);
	return 1.0f / compliance;
}

void Articulation::prepareLtbMatrix(FsData& fsData,
	const FsInertia* baseInertia,
	const PxTransform* poses,
	const ArticulationJointTransforms* jointTransforms,
	PxReal recipDt)
{
	PxU32 linkCount = fsData.linkCount;
	LtbRow* rows = getLtbRows(fsData);

	rows[0].inertia = baseInertia[0];

	const PxVec3 axis[3] = { PxVec3(1.0f,0.0f,0.0f), PxVec3(0.0f,1.0f,0.0f), PxVec3(0.0f,0.0f,1.0f) };
	for (PxU32 i = 1; i<linkCount; i++)
	{
		rows[i].inertia = baseInertia[i];
		const ArticulationJointTransforms& s = jointTransforms[i];

		const PxU32 p = fsData.parent[i];

		// we put the action point of the constraint at the root of the child

		const PxVec3 ra = s.cB2w.p - poses[p].p;
		const PxVec3 rb = s.cB2w.p - poses[i].p;

		// A bit different from the 1D solver, 
		// there we use a formulation	j0.v0 - j1.v1 + c = 0
		// here we use the homogeneous	j0.v0 + j1.v1 + c = 0

		const PxVec3 error = (s.cA2w.p - s.cB2w.p) * 0.99f;

		Cm::SpatialVectorV* j0 = rows[i].j0;
		Cm::SpatialVectorV* j1 = rows[i].j1;

		for (PxU32 j = 0; j<3; j++)
		{
			PxVec3 n = axis[j];
			j0[j] = Cm::SpatialVector(n, ra.cross(n));
			j1[j] = Cm::SpatialVector(-n, -rb.cross(n));
		}

		rows[i].jC = V3LoadU(error*recipDt);
	}
}

void PxcLtbComputeJv(Vec3V* jv, const FsData& m, const Cm::SpatialVectorV* velocity)
{
	const LtbRow* rows = getLtbRows(m);
	const FsRow* fsRows = getFsRows(m);
	const FsJointVectors* jointVectors = getJointVectors(m);

	PX_UNUSED(rows);
	PX_UNUSED(fsRows);

	for (PxU32 i = 1; i<m.linkCount; i++)
	{
		Cm::SpatialVectorV pv = velocity[m.parent[i]], v = velocity[i];

		Vec3V parentOffset = V3Add(jointVectors[i].jointOffset, jointVectors[i].parentOffset);

		Vec3V k0v = V3Add(pv.linear, V3Cross(pv.angular, parentOffset)),
			k1v = V3Add(v.linear, V3Cross(v.angular, jointVectors[i].jointOffset));
		jv[i] = V3Sub(k0v, k1v);
	}
}

void PxcLtbSolve(const FsData& m,
	Vec3V* b,					// rhs error to solve for
	Cm::SpatialVectorV* y)		// velocity delta output
{
	typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;

	const LtbRow* rows = getLtbRows(m);
	PxMemZero(y, m.linkCount * sizeof(Cm::SpatialVectorV));

	for (PxU32 i = m.linkCount; i-->1;)
	{
		const LtbRow& r = rows[i];
		const PxU32 p = m.parent[i];

		const Vec3V t = V3Sub(b[i], Fns::axisDot(r.j1, y[i]));
		b[i] = t;
		y[p] = Fns::subtract(y[p], Fns::axisMultiply(r.j0, t));
	}

	y[0] = Fns::multiply(rows[0].inertia, y[0]);

	for (PxU32 i = 1; i<m.linkCount; i++)
	{
		const LtbRow& r = rows[i];
		const PxU32 p = m.parent[i];

		const Vec3V t = V3Sub(M33MulV3(r.jResponse, b[i]), Fns::axisDot(r.j0, y[p]));
		y[i] = Fns::subtract(Fns::multiply(r.inertia, y[i]), Fns::axisMultiply(r.j1, t));
	}
}

void PxcLtbProject(const FsData& m,
	Cm::SpatialVectorV* velocity,
	Vec3V* b)
{
	PX_ASSERT(m.linkCount <= DY_ARTICULATION_MAX_SIZE);
	Cm::SpatialVectorV y[DY_ARTICULATION_MAX_SIZE];

	PxcLtbSolve(m, b, y);

	for (PxU32 i = 0; i<m.linkCount; i++)
		velocity[i] -= y[i];
}

void PxcFsComputeJointLoadsSimd(const FsData& matrix,
	const FsInertia*PX_RESTRICT baseInertia,
	Mat33V*PX_RESTRICT load,
	const PxReal*PX_RESTRICT isf_,
	PxU32 linkCount,
	PxU32 maxIterations,
	PxcFsScratchAllocator allocator)
{
	// dsequeira: this is really difficult to optimize on XBox: not inlining generates lots of LHSs, 
	// inlining generates lots of cache misses because the fn is so huge (almost 2000 instrs.) 
	// Timing says even for 1 iteration the cache misses are slighly preferable for a
	// 20-bone articulation, for more iters it's *much* better to take the cache misses.
	//
	// about 400 instructions come from unnecessary and inexplicable branch checks

	if (!maxIterations)
		return;

	typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;

	FloatV isf[DY_ARTICULATION_MAX_SIZE];

	for (PxU32 i = 1; i<linkCount; i++)
		isf[i] = FLoad(isf_[i]);

	FsInertia*PX_RESTRICT inertia = allocator.alloc<FsInertia>(linkCount);
	FsInertia*PX_RESTRICT contribToParent = allocator.alloc<FsInertia>(linkCount);

	const FsRow*PX_RESTRICT row = getFsRows(matrix);
	const FsRowAux*PX_RESTRICT aux = getAux(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

	PX_UNUSED(row);

	// gets rid of about 200 LHSs, need to change the matrix format to make this part of it
	PxU64 parent[DY_ARTICULATION_MAX_SIZE];
	for (PxU32 i = 0; i<linkCount; i++)
		parent[i] = matrix.parent[i];

	while (maxIterations--)
	{
		PxMemCopy(inertia, baseInertia, sizeof(FsInertia)*linkCount);

		for (PxU32 i = linkCount; i-->1;)
		{
			const Cm::SpatialVectorV*PX_RESTRICT S = aux[i].S;

			Ps::prefetch(&load[i - 1]);
			Ps::prefetch(&jointVectors[i - 1]);
			const FsInertia tmp = Fns::propagate(inertia[i], S, load[i], isf[i]);
			inertia[parent[i]] = Fns::addInertia(inertia[parent[i]], Fns::translateInertia(jointVectors[i].parentOffset, tmp));
			contribToParent[i] = tmp;
		}

		for (PxU32 i = 1; i<linkCount; i++)
		{
			const Cm::SpatialVectorV*PX_RESTRICT S = aux[i].S;

			const FsInertia rootwardInertia = Fns::subtractInertia(Fns::translateInertia(V3Neg(jointVectors[i].parentOffset), inertia[parent[i]]), contribToParent[i]);
			const FsInertia tmp = Fns::propagate(rootwardInertia, S, load[i], isf[i]);
			load[i] = Fns::computeDriveInertia(inertia[i], rootwardInertia, S);
			inertia[i] = Fns::addInertia(inertia[i], tmp);
		}
	}
}

void PxcFsPropagateDrivenInertiaSimd(FsData& matrix,
	const FsInertia* baseInertia,
	const PxReal* isf,
	const Mat33V* load,
	PxcFsScratchAllocator allocator)
{
	typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;

	Cm::SpatialVectorV IS[3];

	FsRow* rows = getFsRows(matrix);
	const FsRowAux* aux = getAux(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

	FsInertia* inertia = allocator.alloc<FsInertia>(matrix.linkCount);
	PxMemCopy(inertia, baseInertia, matrix.linkCount * sizeof(FsInertia));

	for (PxU32 i = matrix.linkCount; --i>0;)
	{
		FsRow& r = rows[i];
		const FsRowAux& a = aux[i];
		const FsJointVectors& jv = jointVectors[i];

		const Mat33V m = Fns::computeSIS(inertia[i], a.S, IS);
		const FloatV f = FLoad(isf[i]);

		const Mat33V D = Fns::invertSym33(Mat33V(V3ScaleAdd(load[i].col0, f, m.col0),
			V3ScaleAdd(load[i].col1, f, m.col1),
			V3ScaleAdd(load[i].col2, f, m.col2)));
		r.D = D;

		inertia[matrix.parent[i]] = Fns::addInertia(inertia[matrix.parent[i]],
			Fns::translateInertia(jv.parentOffset, Fns::multiplySubtract(inertia[i], D, IS, r.DSI)));
	}

	getRootInverseInertia(matrix) = Fns::invertInertia(inertia[0]);
}

PX_FORCE_INLINE Cm::SpatialVectorV propagateDrivenImpulse(const FsRow& row,
	const FsJointVectors& jv,
	Vec3V& SZMinusQ,
	const Cm::SpatialVectorV& Z,
	const Vec3V& Q)
{
	typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;

	SZMinusQ = V3Sub(V3Add(Z.angular, V3Cross(Z.linear, jv.jointOffset)), Q);
	Cm::SpatialVectorV result = Fns::translateForce(jv.parentOffset, Z - Fns::axisMultiply(row.DSI, SZMinusQ));

	return result;
}

void PxcFsApplyJointDrives(FsData& matrix, const Vec3V* Q)
{
	typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;

	PX_ASSERT(matrix.linkCount <= DY_ARTICULATION_MAX_SIZE);

	const FsRow* rows = getFsRows(matrix);
	const FsRowAux* aux = getAux(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

	Cm::SpatialVectorV Z[DY_ARTICULATION_MAX_SIZE];
	Cm::SpatialVectorV dV[DY_ARTICULATION_MAX_SIZE];
	Vec3V SZminusQ[DY_ARTICULATION_MAX_SIZE];

	PxMemZero(Z, matrix.linkCount * sizeof(Cm::SpatialVectorV));

	for (PxU32 i = matrix.linkCount; i-->1;)
		Z[matrix.parent[i]] += propagateDrivenImpulse(rows[i], jointVectors[i], SZminusQ[i], Z[i], Q[i]);

	dV[0] = Fns::multiply(getRootInverseInertia(matrix), -Z[0]);

	for (PxU32 i = 1; i<matrix.linkCount; i++)
		dV[i] = Fns::propagateVelocity(rows[i], jointVectors[i], SZminusQ[i], dV[matrix.parent[i]], aux[i]);

	Cm::SpatialVectorV* V = getVelocity(matrix);
	for (PxU32 i = 0; i<matrix.linkCount; i++)
		V[i] += dV[i];
}

void Articulation::applyImpulses(const FsData& matrix, Cm::SpatialVectorV* Z, Cm::SpatialVectorV* V)
{
	// note: Z is the negated impulse

	typedef ArticulationFnsSimd<ArticulationFnsSimdBase> Fns;

	PX_ASSERT(matrix.linkCount <= DY_ARTICULATION_MAX_SIZE);
	const FsRow* rows = getFsRows(matrix);
	const FsRowAux* aux = getAux(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

	Cm::SpatialVectorV dV[DY_ARTICULATION_MAX_SIZE];
	Vec3V SZ[DY_ARTICULATION_MAX_SIZE];

	for (PxU32 i = matrix.linkCount; i-->1;)
		Z[matrix.parent[i]] += Fns::propagateImpulse(rows[i], jointVectors[i], SZ[i], Z[i], aux[i]);

	dV[0] = Fns::multiply(getRootInverseInertia(matrix), -Z[0]);

	for (PxU32 i = 1; i<matrix.linkCount; i++)
		dV[i] = Fns::propagateVelocity(rows[i], jointVectors[i], SZ[i], dV[matrix.parent[i]], aux[i]);

	for (PxU32 i = 0; i<matrix.linkCount; i++)
		V[i] += dV[i];
}

void Articulation::computeUnconstrainedVelocitiesInternal(const ArticulationSolverDesc& desc,
	PxReal dt,
	const PxVec3& gravity, PxU64 contextID,
	FsInertia* PX_RESTRICT baseInertia,
	ArticulationJointTransforms* PX_RESTRICT jointTransforms,
	PxcFsScratchAllocator& allocator)
{
	PX_UNUSED(contextID);
	const ArticulationLink* links = desc.links;
	PxU16 linkCount = desc.linkCount;
	Articulation* articulation = static_cast<Articulation*>(desc.articulation);
	FsData& fsData = *articulation->getFsDataPtr();
	PxTransform* poses = desc.poses;
	PxQuat* deltaQ = desc.deltaQ;

	{
		PX_PROFILE_ZONE("Articulations.prepareDataBlock", contextID);
		prepareDataBlock(fsData, links, linkCount, poses, deltaQ, baseInertia, jointTransforms, desc.totalDataSize);
	}

	const PxReal recipDt = 1.0f / dt;

	Cm::SpatialVectorV* velocity = getVelocity(fsData);

	{
		PX_PROFILE_ZONE("Articulations.setupProject", contextID);

		PxMemZero(getLtbRows(fsData), getLtbDataSize(linkCount));
		prepareLtbMatrix(fsData, baseInertia, poses, jointTransforms, recipDt);

		PxcLtbFactor(fsData);

		Vec3V b[DY_ARTICULATION_MAX_SIZE];
		PxcLtbComputeJv(b, fsData, velocity);

		LtbRow* rows = getLtbRows(fsData);
		for (PxU32 i = 1; i<linkCount; i++)
			b[i] = V3Add(b[i], rows[i].jC);

		PxcLtbProject(fsData, velocity, b);
	}

	{
		PX_PROFILE_ZONE("Articulations.prepareFsData", contextID);
		PxMemZero(addAddr<void*>(&fsData, fsData.fsDataOffset), getFsDataSize(linkCount));
		prepareFsData(fsData, links);
	}

	{
		PX_PROFILE_ZONE("Articulations.setupDrives", contextID);

		if (!(desc.core->externalDriveIterations & 0x80000000))
			PxMemZero(desc.externalLoads, sizeof(Mat33V) * linkCount);

		if (!(desc.core->internalDriveIterations & 0x80000000))
			PxMemZero(desc.internalLoads, sizeof(Mat33V) * linkCount);

		PxReal				isf[DY_ARTICULATION_MAX_SIZE], esf[DY_ARTICULATION_MAX_SIZE];			// spring factors
		Vec3V				drive[DY_ARTICULATION_MAX_SIZE];

		bool externalEqualsInternalCompliance = (desc.core->internalDriveIterations & 0xffff) == (desc.core->externalDriveIterations & 0xffff);
		for (PxU32 i = 1; i<linkCount; i++)
		{
			const ArticulationJointCore& j = static_cast<const ArticulationJointCore&>(*links[i].inboundJoint);
			isf[i] = (1 + j.damping * dt + j.spring * dt * dt) * getResistance(j.internalCompliance);
			esf[i] = (1 + j.damping * dt + j.spring * dt * dt) * getResistance(j.externalCompliance);

			externalEqualsInternalCompliance = externalEqualsInternalCompliance && j.internalCompliance == j.externalCompliance;
		}

		{
			PX_PROFILE_ZONE("Articulations.jointInternalLoads", contextID);
			PxcFsComputeJointLoadsSimd(fsData, baseInertia, desc.internalLoads, isf, linkCount, desc.core->internalDriveIterations & 0xffff, allocator);
		}

		{
			PX_PROFILE_ZONE("Articulations.propagateDrivenInertia", contextID);
			PxcFsPropagateDrivenInertiaSimd(fsData, baseInertia, isf, desc.internalLoads, allocator);
		}

		{
			PX_PROFILE_ZONE("Articulations.computeJointDrives", contextID);
			computeJointDrives(fsData, drive, links, poses, jointTransforms, desc.internalLoads, dt);
		}

		{
			PX_PROFILE_ZONE("Articulations.applyJointDrives", contextID);
			PxcFsApplyJointDrives(fsData, drive);
		}

		if (!externalEqualsInternalCompliance)
		{
			{
				PX_PROFILE_ZONE("Articulations.jointExternalLoads", contextID);
				PxcFsComputeJointLoadsSimd(fsData, baseInertia, desc.externalLoads, esf, linkCount, desc.core->externalDriveIterations & 0xffff, allocator);
			}

			{
				PX_PROFILE_ZONE("Articulations.propagateDrivenInertia", contextID);
				PxcFsPropagateDrivenInertiaSimd(fsData, baseInertia, esf, desc.externalLoads, allocator);
			}
		}
	}

	{
		PX_PROFILE_ZONE("Articulations.applyExternalImpulses", contextID);
		Cm::SpatialVectorV	Z[DY_ARTICULATION_MAX_SIZE];

		FloatV h = FLoad(dt);

		Cm::SpatialVector* acceleration = desc.acceleration;

		const Vec3V vGravity = V3LoadU(gravity);

		for (PxU32 i = 0; i<linkCount; i++)
		{
			Vec3V linearAccel = V3LoadA(acceleration[i].linear);

			if (!(desc.links[i].body->mInternalFlags & PxcRigidBody::eDISABLE_GRAVITY))
				linearAccel = V3Add(linearAccel, vGravity);
			Cm::SpatialVectorV a(linearAccel, V3LoadA(acceleration[i].angular));
			Z[i] = -ArticulationFnsSimd<ArticulationFnsSimdBase>::multiply(baseInertia[i], a) * h;
			//KS - zero accelerations to ensure they don't get re-applied next frame if nothing touches them again.
			acceleration[i].linear = PxVec3(0.f); acceleration[i].angular = PxVec3(0.f);
		}

		applyImpulses(fsData, Z, getVelocity(fsData));
	}

	// save off the motion velocity in case there are no constraints with the articulation

	PxMemCopy(desc.motionVelocity, velocity, linkCount * sizeof(Cm::SpatialVectorV));

	// set up for deferred-update solve

	fsData.dirty = 0;

	// solver progress counters
	maxSolverNormalProgress = 0;
	maxSolverFrictionProgress = 0;
	solverProgress = 0;

#if DY_ARTICULATION_DEBUG_VERIFY
	for (PxU32 i = 0; i<linkCount; i++)
		getRefVelocity(fsData)[i] = getVelocity(fsData)[i];
#endif
}

PxU32 Articulation::computeUnconstrainedVelocities(
	const ArticulationSolverDesc& desc,
	PxReal dt,
	PxConstraintAllocator& allocator,
	PxSolverConstraintDesc* constraintDesc,
	PxU32& acCount,
	const PxVec3& gravity, PxU64 contextID,
	Cm::SpatialVectorF* /*Z*/, Cm::SpatialVectorF* /*deltaV*/)
{
	const ArticulationLink* links = desc.links;
	Articulation& articulation = static_cast<Articulation&>(*desc.articulation);

	PxcFsScratchAllocator fsAllocator(desc.scratchMemory, desc.scratchMemorySize);
	FsInertia*						PX_RESTRICT baseInertia = fsAllocator.alloc<FsInertia>(desc.linkCount);
	ArticulationJointTransforms*	PX_RESTRICT jointTransforms = fsAllocator.alloc<ArticulationJointTransforms>(desc.linkCount);

	articulation.computeUnconstrainedVelocitiesInternal(desc, dt, gravity, contextID, baseInertia, jointTransforms, fsAllocator);

	{
		PX_PROFILE_ZONE("Articulations.setupConstraints", contextID);
		return ArticulationHelper::setupSolverConstraints(articulation, desc.solverDataSize, allocator, constraintDesc, links, jointTransforms, dt, acCount);
	}
}

void Articulation::computeUnconstrainedVelocitiesTGS(
	const ArticulationSolverDesc& desc,
	PxReal dt, const PxVec3& gravity,
	PxU64 contextID, Cm::SpatialVectorF* /*Z*/, Cm::SpatialVectorF* /*deltaV*/)
{
	Articulation& articulation = static_cast<Articulation&>(*desc.articulation);
	PxcFsScratchAllocator allocator(desc.scratchMemory, desc.scratchMemorySize);
	FsInertia*						PX_RESTRICT baseInertia = allocator.alloc<FsInertia>(desc.linkCount);
	ArticulationJointTransforms*	PX_RESTRICT jointTransforms = allocator.alloc<ArticulationJointTransforms>(desc.linkCount);

	articulation.computeUnconstrainedVelocitiesInternal(desc, dt, gravity, 
		contextID, baseInertia, jointTransforms, allocator);
}

void Articulation::computeJointDrives(FsData& fsData,
	Ps::aos::Vec3V* drives,
	const ArticulationLink* links,
	const PxTransform* poses,
	const ArticulationJointTransforms* transforms,
	const Ps::aos::Mat33V* loads,
	PxReal dt)
{
	typedef ArticulationFnsScalar Fns;

	const PxU32 linkCount = fsData.linkCount;
	const Cm::SpatialVector* velocity = reinterpret_cast<const Cm::SpatialVector*>(getVelocity(fsData));

	for (PxU32 i = 1; i<linkCount; i++)
	{
		PxU32 parent = links[i].parent;
		const ArticulationJointTransforms& b = transforms[i];
		const ArticulationJointCore& j = static_cast<const ArticulationJointCore&>(*links[i].inboundJoint);

		const Cm::SpatialVector currentVel = Fns::translateMotion(poses[i].p - b.cA2w.p, velocity[i])
			- Fns::translateMotion(poses[parent].p - b.cA2w.p, velocity[parent]);

		// we want the quat such that q * cB2cA = targetPosition
		PxVec3 rotVec;
		if (j.driveType == PxU8(PxArticulationJointDriveType::eERROR))
			rotVec = j.targetPosition.getImaginaryPart();
		else
			rotVec = Ps::log(j.targetPosition * b.cB2cA.q.getConjugate()); // as a rotation vector

		// NM's Tests indicate behavior is better without the term commented out below, even though
		// an implicit spring derivation suggests it should be there.

		const PxVec3 posError = b.cA2w.rotate(rotVec); // - currentVel.angular * 0.5f * dt
		const PxVec3 velError = b.cA2w.rotate(j.targetVelocity) - currentVel.angular;

		drives[i] = M33MulV3(loads[i], V3LoadU((j.spring * posError + j.damping * velError) * dt * getResistance(j.internalCompliance)));
	}
}

void Articulation::saveVelocity(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* /*deltaV*/)
{
	Vec3V b[DY_ARTICULATION_MAX_SIZE];

	Articulation* articulation = static_cast<Articulation*>(desc.articulation);
	FsData& m = *articulation->getFsDataPtr();

	Cm::SpatialVectorV* velocity = getVelocity(m);
	PxcFsFlushVelocity(m);

	// save off the motion velocity

	for (PxU32 i = 0; i<m.linkCount; i++)
	{
		desc.motionVelocity[i] = velocity[i];
		PX_ASSERT(isFiniteVec3V(velocity[i].linear));
		PX_ASSERT(isFiniteVec3V(velocity[i].angular));
	}

	// and now re-solve to use the unbiased velocities

	PxcLtbComputeJv(b, m, velocity);
	PxcLtbProject(m, velocity, b);

#if DY_ARTICULATION_DEBUG_VERIFY
	for (PxU32 i = 0; i<m.linkCount; i++)
		getRefVelocity(m)[i] = velocity[i];
#endif
}

void Articulation::saveVelocityTGS(const ArticulationSolverDesc& desc, PxReal invDtF32)
{
	Vec3V b[DY_ARTICULATION_MAX_SIZE];

	Articulation* articulation = static_cast<Articulation*>(desc.articulation);
	FsData& m = *articulation->getFsDataPtr();

	Cm::SpatialVectorV* velocity = getVelocity(m);
	PxcFsFlushVelocity(m);

	const FloatV invDt = FLoad(invDtF32);

	Cm::SpatialVectorV* motionVector = getMotionVector(m);

	// save off the motion velocity

	for (PxU32 i = 0; i<m.linkCount; i++)
	{
		Cm::SpatialVectorV vel = motionVector[i] * invDt;

		//velocity[i] = vel;

		desc.motionVelocity[i] = vel;
		PX_ASSERT(isFiniteVec3V(velocity[i].linear));
		PX_ASSERT(isFiniteVec3V(velocity[i].angular));
	}

	// and now re-solve to use the unbiased velocities

	PxcLtbComputeJv(b, m, velocity);
	PxcLtbProject(m, velocity, b);

#if DY_ARTICULATION_DEBUG_VERIFY
	for (PxU32 i = 0; i<m.linkCount; i++)
		getRefVelocity(m)[i] = velocity[i];
#endif
}

void Articulation::recordDeltaMotion(const ArticulationSolverDesc &desc, const PxReal dt, Cm::SpatialVectorF* /*deltaV*/)
{
	Articulation* articulation = static_cast<Articulation*>(desc.articulation);
	FsData& m = *articulation->getFsDataPtr();

	PxQuat* deltaQ = desc.deltaQ;

	const Cm::SpatialVectorV* velocity = getVelocity(m);
	Cm::SpatialVectorV* deltaMotion = getMotionVector(m);
	PxcFsFlushVelocity(m);

	const FloatV fDt = FLoad(dt);

	for (PxU32 i = 0; i<m.linkCount; i++)
	{
		deltaMotion[i] += velocity[i] * fDt;

		deltaQ[i] = Ps::exp(unsimdRef(velocity[i]).angular*dt) * deltaQ[i];

		PX_ASSERT(isFiniteVec3V(velocity[i].linear));
		PX_ASSERT(isFiniteVec3V(velocity[i].angular));
	}
}

void Articulation::deltaMotionToMotionVelocity(const ArticulationSolverDesc& desc, PxReal invDt)
{
	Articulation* articulation = static_cast<Articulation*>(desc.articulation);
	FsData& m = *articulation->getFsDataPtr();

	const Cm::SpatialVectorV* deltaMotion = getMotionVector(m);

	Cm::SpatialVectorV* velocity = getVelocity(m);

	const FloatV fInvDt = FLoad(invDt);

	for (PxU32 i = 0; i<m.linkCount; i++)
	{
		velocity[i] = desc.motionVelocity[i] = deltaMotion[i] * fInvDt;
	}
}

void Articulation::pxcFsApplyImpulse(PxU32 linkID, Ps::aos::Vec3V linear, 
	Ps::aos::Vec3V angular, Cm::SpatialVectorF* /*Z*/, Cm::SpatialVectorF* /*deltaV*/)
{
#if DY_ARTICULATION_DEBUG_VERIFY
	{
		Cm::SpatialVectorV imp(linear, angular);
		ArticulationRef::applyImpulse(matrix, reinterpret_cast<Cm::SpatialVector *>(getRefVelocity(matrix)), linkID, reinterpret_cast<Cm::SpatialVector&>(imp));
	}
#endif

	FsData& matrix = *getFsDataPtr();
	Vec3V linZ = V3Neg(linear);
	Vec3V angZ = V3Neg(angular);

	const FsRow *rows = getFsRows(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

#if DY_ARTICULATION_DEBUG_VERIFY
	const FsRowAux *aux = getAux(matrix);
#endif
	Vec3V *deferredSZ = getDeferredSZ(matrix);

	for (PxU32 i = linkID; i != 0; i = matrix.parent[i])
	{
		const FsRow &row = rows[i];
		const FsJointVectors& jv = jointVectors[i];

#if DY_ARTICULATION_DEBUG_VERIFY
		PxVec3 SZcheck;
		Cm::SpatialVector Zcheck = ArticulationRef::propagateImpulse(row, jv, SZcheck, SpV(linZ, angZ), aux[i]);
#endif

		Vec3V SZ = V3Add(angZ, V3Cross(linZ, jv.jointOffset));
		Vec3V lrLinear = V3Sub(linZ, V3ScaleAdd(row.DSI[0].linear, V3GetX(SZ),
			V3ScaleAdd(row.DSI[1].linear, V3GetY(SZ),
				V3Scale(row.DSI[2].linear, V3GetZ(SZ)))));

		Vec3V lrAngular = V3Sub(angZ, V3ScaleAdd(row.DSI[0].angular, V3GetX(SZ),
			V3ScaleAdd(row.DSI[1].angular, V3GetY(SZ),
				V3Scale(row.DSI[2].angular, V3GetZ(SZ)))));

		linZ = lrLinear;
		angZ = V3Add(lrAngular, V3Cross(jv.parentOffset, lrLinear));
		deferredSZ[i] = V3Add(deferredSZ[i], SZ);

		PX_ASSERT(Ps::aos::isFiniteVec3V(linZ));
		PX_ASSERT(Ps::aos::isFiniteVec3V(angZ));

#if DY_ARTICULATION_DEBUG_VERIFY
		Cm::SpatialVector Z = SpV(linZ, angZ);
		PX_ASSERT((Z - Zcheck).magnitude()<1e-4*PxMax(Zcheck.magnitude(), 1.0f));
		PX_ASSERT(((PxVec3&)SZ - SZcheck).magnitude()<1e-4*PxMax(SZcheck.magnitude(), 1.0f));
#endif
	}

	matrix.deferredZ.linear = V3Add(matrix.deferredZ.linear, linZ);
	matrix.deferredZ.angular = V3Add(matrix.deferredZ.angular, angZ);

	matrix.dirty |= rows[linkID].pathToRoot;
}

void Articulation::pxcFsGetVelocities(PxU32 linkID, PxU32 linkID1, Cm::SpatialVectorV& v0, Cm::SpatialVectorV& v1)
{
	//Common case - linkID = parent of linkID1...
	//In this case, we compute the delta velocity between the 2 and return 

	v0 = pxcFsGetVelocity(linkID);
	v1 = pxcFsGetVelocity(linkID1);
}

void Articulation::pxcFsApplyImpulses(PxU32 linkID, const Ps::aos::Vec3V& linear,
	const Ps::aos::Vec3V& angular, PxU32 linkID2, const Ps::aos::Vec3V& linear2,
	const Ps::aos::Vec3V& angular2, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV)
{
	pxcFsApplyImpulse(linkID, linear, angular, Z, deltaV);
	pxcFsApplyImpulse(linkID2, linear2, angular2, Z, deltaV);
}

void Articulation::solveInternalConstraints(const PxReal /*dt*/, const PxReal /*invDt*/, Cm::SpatialVectorF* /*impulses*/,
	Cm::SpatialVectorF* /*DeltaV*/, bool)
{
}

Cm::SpatialVectorV Articulation::pxcFsGetVelocity(PxU32 linkID)
{
	FsData& matrix = *getFsDataPtr();
	const FsRow *rows = getFsRows(matrix);
	const FsJointVectors* jointVectors = getJointVectors(matrix);

#if DY_ARTICULATION_DEBUG_VERIFY
	const FsRowAux *aux = getAux(matrix);
#endif
	Cm::SpatialVectorV* PX_RESTRICT V = getVelocity(matrix);

	// find the dirty node on the path (including the root) with the lowest index
	ArticulationBitField toUpdate = rows[linkID].pathToRoot & matrix.dirty;

	if (toUpdate)
	{
		// store the dV elements densely and use an array map to decode - hopefully cache friendlier
		PxU32 indexToStackLoc[DY_ARTICULATION_MAX_SIZE], count = 0;
		Cm::SpatialVectorV dVStack[DY_ARTICULATION_MAX_SIZE];

		ArticulationBitField ignoreNodes = (toUpdate & (0 - toUpdate)) - 1;
		//PX_ASSERT(ignoreNodes == 0);
		ArticulationBitField path = rows[linkID].pathToRoot & ~ignoreNodes, p = path;
		ArticulationBitField newDirty = 0;

		Vec3V ldV = V3Zero(), adV = V3Zero();
		Cm::SpatialVectorV* PX_RESTRICT defV = getDeferredVel(matrix);
		Vec3V* PX_RESTRICT SZ = getDeferredSZ(matrix);

		if (p & 1)
		{
			const FsInertia &m = getRootInverseInertia(matrix);
			Vec3V lZ = V3Neg(matrix.deferredZ.linear);
			Vec3V aZ = V3Neg(matrix.deferredZ.angular);

			ldV = V3Add(M33MulV3(m.ll, lZ), M33MulV3(m.la, aZ));
			adV = V3Add(M33TrnspsMulV3(m.la, lZ), M33MulV3(m.aa, aZ));

			V[0].linear = V3Add(V[0].linear, ldV);
			V[0].angular = V3Add(V[0].angular, adV);

			matrix.deferredZ.linear = V3Zero();
			matrix.deferredZ.angular = V3Zero();

			indexToStackLoc[0] = count;
			Cm::SpatialVectorV &e = dVStack[count++];

			e.linear = ldV;
			e.angular = adV;

			newDirty = rows[0].children;
			p--;
		}

		while (p)	// using "for(;p;p &= (p-1))" here generates LHSs from the ArticulationLowestSetBit
		{
			PxU32 i = ArticulationLowestSetBit(p);
			const FsJointVectors& jv = jointVectors[i];

			p &= (p - 1);

			const FsRow* PX_RESTRICT row = rows + i;

			ldV = V3Add(ldV, defV[i].linear);
			adV = V3Add(adV, defV[i].angular);

#if DY_ARTICULATION_DEBUG_VERIFY
			Cm::SpatialVector dVcheck = ArticulationRef::propagateVelocity(*row, jv, (PxVec3&)SZ[i], SpV(ldV, adV), aux[i]);
#endif

			Vec3V DSZ = M33MulV3(row->D, SZ[i]);

			Vec3V lW = V3Add(ldV, V3Cross(adV, jv.parentOffset));
			Vec3V aW = adV;

			const Cm::SpatialVectorV*PX_RESTRICT DSI = row->DSI;
			Vec3V lN = V3Merge(V3Dot(DSI[0].linear, lW), V3Dot(DSI[1].linear, lW), V3Dot(DSI[2].linear, lW));
			Vec3V aN = V3Merge(V3Dot(DSI[0].angular, aW), V3Dot(DSI[1].angular, aW), V3Dot(DSI[2].angular, aW));

			Vec3V n = V3Add(V3Add(lN, aN), DSZ);

			ldV = V3Sub(lW, V3Cross(jv.jointOffset, n));
			adV = V3Sub(aW, n);

#if DY_ARTICULATION_DEBUG_VERIFY
			Cm::SpatialVector dV = SpV(ldV, adV);
			PX_ASSERT((dV - dVcheck).magnitude()<1e-4*PxMax(dVcheck.magnitude(), 1.0f));
#endif

			V[i].linear = V3Add(V[i].linear, ldV);
			V[i].angular = V3Add(V[i].angular, adV);

			defV[i].linear = V3Zero();
			defV[i].angular = V3Zero();
			SZ[i] = V3Zero();

			indexToStackLoc[i] = count;
			Cm::SpatialVectorV &e = dVStack[count++];
			newDirty |= rows[i].children;

			e.linear = ldV;
			e.angular = adV;
		}

		for (ArticulationBitField defer = newDirty&~path; defer; defer &= (defer - 1))
		{
			PxU32 i = ArticulationLowestSetBit(defer);
			PxU32 parent = indexToStackLoc[matrix.parent[i]];

			defV[i].linear = V3Add(defV[i].linear, dVStack[parent].linear);
			defV[i].angular = V3Add(defV[i].angular, dVStack[parent].angular);
		}

		matrix.dirty = (matrix.dirty | newDirty)&~path;
	}
#if DY_ARTICULATION_DEBUG_VERIFY
	Cm::SpatialVector v = reinterpret_cast<Cm::SpatialVector&>(V[linkID]);
	Cm::SpatialVector rv = reinterpret_cast<Cm::SpatialVector&>(getRefVelocity(matrix)[linkID]);
	PX_ASSERT((v - rv).magnitude()<1e-4f * PxMax(rv.magnitude(), 1.0f));
#endif

	return V[linkID];
}

//Cm::SpatialVectorV PxcFsGetVelocity(Articulation &articulation,
//	PxU32 linkID)
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
//	if (toUpdate)
//	{
//		// store the dV elements densely and use an array map to decode - hopefully cache friendlier
//		PxU32 indexToStackLoc[DY_ARTICULATION_MAX_SIZE], count = 0;
//		Cm::SpatialVectorV dVStack[DY_ARTICULATION_MAX_SIZE];
//
//		ArticulationBitField ignoreNodes = (toUpdate & (0 - toUpdate)) - 1;
//		ArticulationBitField path = rows[linkID].pathToRoot & ~ignoreNodes, p = path;
//		ArticulationBitField newDirty = 0;
//
//		Vec3V ldV = V3Zero(), adV = V3Zero();
//		Cm::SpatialVectorV* PX_RESTRICT defV = getDeferredVel(matrix);
//		Vec3V* PX_RESTRICT SZ = getDeferredSZ(matrix);
//
//		if (p & 1)
//		{
//			const FsInertia &m = getRootInverseInertia(matrix);
//			Vec3V lZ = V3Neg(matrix.deferredZ.linear);
//			Vec3V aZ = V3Neg(matrix.deferredZ.angular);
//
//			ldV = V3Add(M33MulV3(m.ll, lZ), M33MulV3(m.la, aZ));
//			adV = V3Add(M33TrnspsMulV3(m.la, lZ), M33MulV3(m.aa, aZ));
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
//		while (p)	// using "for(;p;p &= (p-1))" here generates LHSs from the ArticulationLowestSetBit
//		{
//			PxU32 i = ArticulationLowestSetBit(p);
//			const FsJointVectors& jv = jointVectors[i];
//
//			p &= (p - 1);
//
//			const FsRow* PX_RESTRICT row = rows + i;
//
//			ldV = V3Add(ldV, defV[i].linear);
//			adV = V3Add(adV, defV[i].angular);
//
//#if DY_ARTICULATION_DEBUG_VERIFY
//			Cm::SpatialVector dVcheck = ArticulationRef::propagateVelocity(*row, jv, (PxVec3&)SZ[i], SpV(ldV, adV), aux[i]);
//#endif
//
//			Vec3V DSZ = M33MulV3(row->D, SZ[i]);
//
//			Vec3V lW = V3Add(ldV, V3Cross(adV, jv.parentOffset));
//			Vec3V aW = adV;
//
//			const Cm::SpatialVectorV*PX_RESTRICT DSI = row->DSI;
//			Vec3V lN = V3Merge(V3Dot(DSI[0].linear, lW), V3Dot(DSI[1].linear, lW), V3Dot(DSI[2].linear, lW));
//			Vec3V aN = V3Merge(V3Dot(DSI[0].angular, aW), V3Dot(DSI[1].angular, aW), V3Dot(DSI[2].angular, aW));
//
//			Vec3V n = V3Add(V3Add(lN, aN), DSZ);
//
//			ldV = V3Sub(lW, V3Cross(jv.jointOffset, n));
//			adV = V3Sub(aW, n);
//
//#if DY_ARTICULATION_DEBUG_VERIFY
//			Cm::SpatialVector dV = SpV(ldV, adV);
//			PX_ASSERT((dV - dVcheck).magnitude()<1e-4*PxMax(dVcheck.magnitude(), 1.0f));
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
//		for (ArticulationBitField defer = newDirty&~path; defer; defer &= (defer - 1))
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
//	PX_ASSERT((v - rv).magnitude()<1e-4f * PxMax(rv.magnitude(), 1.0f));
//#endif
//
//	return V[linkID];
//}

void Articulation::updateBodies(const ArticulationSolverDesc& desc, PxReal dt)
{
	Articulation* articulation = static_cast<Articulation*>(desc.articulation);
	FsData& fsData = *articulation->getFsDataPtr();
	const ArticulationCore& core = *desc.core;
	const ArticulationLink* links = desc.links;
	PxTransform* poses = desc.poses;
	Cm::SpatialVectorV* motionVelocity = desc.motionVelocity;

	Vec3V b[DY_ARTICULATION_MAX_SIZE];

	PxU32 linkCount = fsData.linkCount;

	PxcFsFlushVelocity(fsData);
	PxcLtbComputeJv(b, fsData, getVelocity(fsData));
	PxcLtbProject(fsData, getVelocity(fsData), b);

	// update positions
	PxcFsScratchAllocator allocator(desc.scratchMemory, desc.scratchMemorySize);
	PxTransform*		PX_RESTRICT oldPose = allocator.alloc<PxTransform>(desc.linkCount);

	for (PxU32 i = 0; i<linkCount; i++)
	{
		const PxVec3& lv = reinterpret_cast<PxVec3&>(motionVelocity[i].linear);
		const PxVec3& av = reinterpret_cast<PxVec3&>(motionVelocity[i].angular);
		oldPose[i] = poses[i];
		poses[i] = PxTransform(poses[i].p + lv * dt, Ps::exp(av*dt) * poses[i].q);
	}

	bool projected = false;
	const PxReal recipDt = 1.0f / dt;

	FsInertia*						PX_RESTRICT baseInertia = allocator.alloc<FsInertia>(desc.linkCount);
	ArticulationJointTransforms*	PX_RESTRICT jointTransforms = allocator.alloc<ArticulationJointTransforms>(desc.linkCount);

	for (PxU32 iterations = 0; iterations < core.maxProjectionIterations; iterations++)
	{
		PxReal maxSeparation = -PX_MAX_F32;
		for (PxU32 i = 1; i<linkCount; i++)
		{
			const ArticulationJointCore& j = static_cast<const ArticulationJointCore&>(*links[i].inboundJoint);
			maxSeparation = PxMax(maxSeparation,
				(poses[links[i].parent].transform(j.parentPose).p -
					poses[i].transform(j.childPose).p).magnitude());
		}

		if (maxSeparation <= core.separationTolerance)
			break;

		projected = true;

		// we go around again, finding velocities which pull us back together - this
		// form of projection is momentum-preserving but slow compared to hierarchical
		// projection

		PxMemZero(baseInertia, sizeof(FsInertia)*linkCount);

		setInertia(baseInertia[0], *links[0].bodyCore, poses[0]);
		for (PxU32 i = 1; i<linkCount; i++)
		{
			setInertia(baseInertia[i], *links[i].bodyCore, poses[i]);
			setJointTransforms(jointTransforms[i], poses[links[i].parent], poses[i], static_cast<const ArticulationJointCore&>(*links[i].inboundJoint));
		}

		articulation->prepareLtbMatrix(fsData, baseInertia, poses, jointTransforms, recipDt);
		PxcLtbFactor(fsData);

		LtbRow* rows = getLtbRows(fsData);

		for (PxU32 i = 1; i<linkCount; i++)
			b[i] = rows[i].jC;

		PxMemZero(motionVelocity, linkCount * sizeof(Cm::SpatialVectorV));

		PxcLtbProject(fsData, motionVelocity, b);

		for (PxU32 i = 0; i<linkCount; i++)
		{
			const PxVec3& lv = reinterpret_cast<PxVec3&>(motionVelocity[i].linear);
			const PxVec3& av = reinterpret_cast<PxVec3&>(motionVelocity[i].angular);
			poses[i] = PxTransform(poses[i].p + lv * dt, Ps::exp(av*dt) * poses[i].q);
		}
	}

	if (projected)
	{
		// recompute motion velocities.
		for (PxU32 i = 0; i<linkCount; i++)
		{
			motionVelocity[i].linear = V3LoadU((poses[i].p - oldPose[i].p) * recipDt);
			motionVelocity[i].angular = V3LoadU(Ps::log(poses[i].q * oldPose[i].q.getConjugate()) * recipDt);
		}
	}

	Cm::SpatialVectorV* velocity = getVelocity(fsData);
	for (PxU32 i = 0; i<linkCount; i++)
	{
		links[i].bodyCore->body2World = poses[i];

		V3StoreA(velocity[i].linear, links[i].bodyCore->linearVelocity);
		V3StoreA(velocity[i].angular, links[i].bodyCore->angularVelocity);
	}
}

void PxvArticulationDriveCache::initialize(
											FsData &fsData,
											PxU16 linkCount,
											const ArticulationLink* links,
											PxReal compliance,
											PxU32 iterations,
											char* scratchMemory,
											PxU32 scratchMemorySize)
{
	PxcFsScratchAllocator allocator(scratchMemory, scratchMemorySize);
	FsInertia*						PX_RESTRICT baseInertia = allocator.alloc<FsInertia>(linkCount);
	ArticulationJointTransforms*	PX_RESTRICT jointTransforms = allocator.alloc<ArticulationJointTransforms>(linkCount);
	PxTransform*					PX_RESTRICT poses = allocator.alloc<PxTransform>(linkCount);
	PxQuat*							PX_RESTRICT deltaQ = allocator.alloc<PxQuat>(linkCount);
	Mat33V*							PX_RESTRICT jointLoads = allocator.alloc<Mat33V>(linkCount);

	PxReal								springFactor[DY_ARTICULATION_MAX_SIZE];			// spring factors

	Articulation::prepareDataBlock(fsData, links, linkCount, poses, deltaQ, baseInertia, jointTransforms, 0);

	PxMemZero(addAddr<void*>(&fsData, fsData.fsDataOffset), Articulation::getFsDataSize(linkCount));
	Articulation::prepareFsData(fsData, links);

	springFactor[0] = 0.0f;
	for (PxU32 i = 1; i<linkCount; i++)
		springFactor[i] = Articulation::getResistance(compliance);

	PxMemZero(jointLoads, sizeof(Mat33V)*linkCount);
	PxcFsComputeJointLoadsSimd(fsData, baseInertia, jointLoads, springFactor, linkCount, iterations & 0xffff, allocator);
	PxcFsPropagateDrivenInertiaSimd(fsData, baseInertia, springFactor, jointLoads, allocator);
	//ArticulationHelper::initializeDriveCache(articulation, cache, linkCount, links, compliance, iterations, scratchMemory, scratchMemorySize);
}

PxU32 PxvArticulationDriveCache::getLinkCount(const FsData& cache)
{
	return cache.linkCount;
}

void PxvArticulationDriveCache::applyImpulses(const FsData& cache,
										 	  Cm::SpatialVectorV* Z,
											  Cm::SpatialVectorV* V)
{
	//ArticulationHelper::applyImpulses(cache, Z, V);
	Articulation::applyImpulses(cache, Z, V);
}

void PxvArticulationDriveCache::getImpulseResponse(const FsData& cache, 
													  PxU32 linkID, 
													  const Cm::SpatialVectorV& impulse,
													  Cm::SpatialVectorV& deltaV)
{
	ArticulationHelper::getImpulseResponse(cache, linkID, impulse, deltaV);
}


}//namespace Dy
}
