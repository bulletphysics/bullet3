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
#include "DyFeatherstoneArticulation.h"
#include "PxsRigidBody.h"
#include "PxcConstraintBlockStream.h"
#include "DyArticulationContactPrep.h"
#include "DyDynamics.h"
#include "DyArticulationReference.h"
#include "DyArticulationPImpl.h"
#include "DyArticulationFnsSimd.h"
#include "PxArticulation.h"
#include "PsFoundation.h"
#include "DyFeatherstoneArticulationLink.h"
#include "DyFeatherstoneArticulationJointData.h"
#include "DySolverConstraint1DStep.h"
#include "DyTGSDynamics.h"
#include "DyConstraintPrep.h"
#include "common/PxProfileZone.h"

#ifndef FEATURESTONE_DEBUG
#define FEATURESTONE_DEBUG 0
#endif

// we encode articulation link handles in the lower bits of the pointer, so the
// articulation has to be aligned, which in an aligned pool means we need to size it
// appropriately

namespace physx
{

namespace Dy
{
	void SolverCoreRegisterArticulationFns();

	void SolverCoreRegisterArticulationFnsCoulomb();

	void PxvRegisterArticulationsReducedCoordinate()
	{
		const PxArticulationBase::Enum type = PxArticulationBase::eReducedCoordinate;
		ArticulationPImpl::sComputeUnconstrainedVelocities[type] = &FeatherstoneArticulation::computeUnconstrainedVelocities;
		ArticulationPImpl::sUpdateBodies[type] = &FeatherstoneArticulation::updateBodies;
		ArticulationPImpl::sUpdateBodiesTGS[type] = &FeatherstoneArticulation::updateBodiesTGS;
		ArticulationPImpl::sSaveVelocity[type] = &FeatherstoneArticulation::saveVelocity;
		ArticulationPImpl::sSaveVelocityTGS[type] = &FeatherstoneArticulation::saveVelocityTGS;

		ArticulationPImpl::sUpdateDeltaMotion[type] = &FeatherstoneArticulation::recordDeltaMotion;
		ArticulationPImpl::sDeltaMotionToMotionVel[type] = &FeatherstoneArticulation::deltaMotionToMotionVelocity;
		ArticulationPImpl::sComputeUnconstrainedVelocitiesTGS[type] = &FeatherstoneArticulation::computeUnconstrainedVelocitiesTGS;
		ArticulationPImpl::sSetupInternalConstraintsTGS[type] = &FeatherstoneArticulation::setupSolverConstraintsTGS;

		SolverCoreRegisterArticulationFns();
		SolverCoreRegisterArticulationFnsCoulomb();
	}

	ArticulationData::~ArticulationData()
	{
		if (mLinksData)
			PX_FREE_AND_RESET(mLinksData);

		if (mJointData)
			PX_FREE_AND_RESET(mJointData);
	}

	void ArticulationData::resizeLinkData(const PxU32 linkCount)
	{
		mMotionVelocities.reserve(linkCount);
		mMotionVelocities.forceSize_Unsafe(linkCount);

		mMotionAccelerations.reserve(linkCount);
		mMotionAccelerations.forceSize_Unsafe(linkCount);

		mCorioliseVectors.reserve(linkCount);
		mCorioliseVectors.forceSize_Unsafe(linkCount);

		mZAForces.reserve(linkCount);
		mZAForces.forceSize_Unsafe(linkCount);

		mDeltaMotionVector.reserve(linkCount);
		mDeltaMotionVector.forceSize_Unsafe(linkCount);

		mPreTransform.reserve(linkCount);
		mPreTransform.forceSize_Unsafe(linkCount);

		//mTempSpatialMatrix.reserve(linkCount);
		//mTempSpatialMatrix.forceSize_Unsafe(linkCount);

		mAccumulatedPoses.reserve(linkCount);
		mAccumulatedPoses.forceSize_Unsafe(linkCount);

		mDeltaQ.reserve(linkCount);
		mDeltaQ.forceSize_Unsafe(linkCount);

		mPosIterMotionVelocities.reserve(linkCount);
		mPosIterMotionVelocities.forceSize_Unsafe(linkCount);

		mJointTransmittedForce.reserve(linkCount);
		mJointTransmittedForce.forceSize_Unsafe(linkCount);

		if (mLinksData)
			PX_FREE_AND_RESET(mLinksData);

		if (mJointData)
			PX_FREE_AND_RESET(mJointData);
		
		mLinksData = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(ArticulationLinkData) * linkCount, PX_DEBUG_EXP("ArticulationLinkData")), ArticulationLinkData)();
		mJointData = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(ArticulationJointCoreData) * linkCount, PX_DEBUG_EXP("ArticulationJointCoreData")), ArticulationJointCoreData)();
		
		const PxU32 size = sizeof(Cm::SpatialVectorF) * linkCount;

		PxMemZero(mMotionVelocities.begin(), size);
		PxMemZero(mMotionAccelerations.begin(), size);
		PxMemZero(mCorioliseVectors.begin(), size);
		PxMemZero(mZAForces.begin(), size);
		PxMemZero(mDeltaMotionVector.begin(), size);

		PxMemZero(mPreTransform.begin(), sizeof(PxTransform) * linkCount);

		PxMemZero(mLinksData, sizeof(ArticulationLinkData) * linkCount);
		PxMemZero(mJointData, sizeof(ArticulationJointCoreData) * linkCount);
	}

	void ArticulationData::resizeJointData(const PxU32 dofs)
	{
		mJointAcceleration.reserve(dofs);
		mJointAcceleration.forceSize_Unsafe(dofs);

		mJointVelocity.reserve(dofs);
		mJointVelocity.forceSize_Unsafe(dofs);

		mJointDeltaVelocity.reserve(dofs);
		mJointDeltaVelocity.forceSize_Unsafe(dofs);

		mJointPosition.reserve(dofs);
		mJointPosition.forceSize_Unsafe(dofs);

		mJointForce.reserve(dofs);
		mJointForce.forceSize_Unsafe(dofs);

	/*	mJointFrictionForce.reserve(dofs);
		mJointFrictionForce.forceSize_Unsafe(dofs);*/

		mPosIterJointDeltaVelocities.reserve(dofs);
		mPosIterJointDeltaVelocities.forceSize_Unsafe(dofs);

		/*mTempData.mJointForce.reserve(dofs);
		mTempData.mJointForce.forceSize_Unsafe(dofs);*/

		PxMemZero(mJointAcceleration.begin(), sizeof(PxReal) * dofs);
		PxMemZero(mJointVelocity.begin(), sizeof(PxReal) * dofs);
		PxMemZero(mJointDeltaVelocity.begin(), sizeof(PxReal) * dofs);
		PxMemZero(mPosIterJointDeltaVelocities.begin(), sizeof(PxReal)*dofs);
		PxMemZero(mJointPosition.begin(), sizeof(PxReal) * dofs);
		PxMemZero(mJointForce.begin(), sizeof(PxReal) * dofs);
		//PxMemZero(mJointFrictionForce.begin(), sizeof(PxReal) * dofs);
	}

	ArticulationLinkData&	ArticulationData::getLinkData(PxU32 index) const
	{
		PX_ASSERT(index < mLinkCount);
		return mLinksData[index];
	}

	//ArticulationJointCoreData&	ArticulationData::getJointData(PxU32 index) const
	//{
	//	PX_ASSERT(index < mLinkCount);
	//	return mJointData[index];
	//}

	void ArticulationJointCore::setJointPose(ArticulationJointCoreData& jointDatum)
	{
		if (dirtyFlag & ArticulationJointCoreDirtyFlag::ePOSE)
		{
			relativeQuat = (childPose.q * (parentPose.q.getConjugate())).getNormalized();

			jointDatum.computeMotionMatrix(static_cast<ArticulationJointCore*>(this));

			dirtyFlag &= ~ArticulationJointCoreDirtyFlag::ePOSE;
		}
	}

	PX_COMPILE_TIME_ASSERT((sizeof(Articulation)&(DY_ARTICULATION_MAX_SIZE - 1)) == 0);

	FeatherstoneArticulation::FeatherstoneArticulation(Sc::ArticulationSim* sim)
		: ArticulationV(sim, PxArticulationBase::eReducedCoordinate), mHasSphericalJoint(false), 
		mGpuRemapId(0xffffffff)
	{
		PX_ASSERT((reinterpret_cast<size_t>(this) & (DY_ARTICULATION_MAX_SIZE - 1)) == 0);
	}

	FeatherstoneArticulation::~FeatherstoneArticulation()
	{
	}

	void FeatherstoneArticulation::copyJointData(ArticulationData& data, PxReal* toJointData, const PxReal* fromJointData)
	{
		const PxU32 linkCount = data.getLinkCount();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationJointCoreData& jointDatum = data.getJointData(linkID);

			PxReal* dest = &toJointData[jointDatum.jointOffset];

			const PxReal* source = &fromJointData[jointDatum.jointOffset];

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				dest[ind] = source[ind];
			}
		}
	}

	void FeatherstoneArticulation::computeDofs()
	{
		const PxU32 linkCount = mArticulationData.getLinkCount();
		PxU32 totalDofs = 0;
		PxU32 totalLocks = 0;
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = mArticulationData.getLink(linkID);

			ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);
			
			jointDatum.computeJointDof(link.inboundJoint, true);
			jointDatum.jointOffset = totalDofs;
			totalDofs += jointDatum.dof;
			totalLocks += jointDatum.lockedAxes;
		}

		if (totalDofs != mArticulationData.getDofs())
		{
			mArticulationData.resizeJointData(totalDofs);
		}

		mArticulationData.setDofs(totalDofs);
		mArticulationData.setLocks(totalLocks);
	}

	bool FeatherstoneArticulation::resize(const PxU32 linkCount)
	{
		if (Dy::ArticulationV::resize(linkCount))
		{
			if (linkCount != mSolverDesc.linkCount)
			{
				//compute scratchSize
				const PxU32 linkCount4 = (linkCount + 3)&(~3);
				PxU32 scratchSize = sizeof(Cm::SpatialVectorF) * linkCount4 * 4
					+ sizeof(Cm::SpatialVector) * linkCount4
					+ sizeof(Dy::SpatialMatrix) * linkCount4
					+ sizeof(PxReal) * linkCount4 * 4;

				mScratchMemory.resize(scratchSize);
				mSolverDesc.scratchMemory = mScratchMemory.begin();
				mSolverDesc.scratchMemorySize = Ps::to16(scratchSize);

				mArticulationData.resizeLinkData(linkCount);
				
			}
			return true;
		}
		return false;
	}

	void FeatherstoneArticulation::getDataSizes(PxU32 /*linkCount*/, PxU32& solverDataSize, PxU32& totalSize, PxU32& scratchSize)
	{		
		solverDataSize = 0;
		totalSize = 0;
		scratchSize = 0;
	}

	void FeatherstoneArticulation::onUpdateSolverDesc()
	{
		Dy::ArticulationV::onUpdateSolverDesc();
		mArticulationData.mLinks = mSolverDesc.links;
		mArticulationData.mLinkCount = mSolverDesc.linkCount;
		mArticulationData.mCore = mSolverDesc.core;
		mArticulationData.mExternalAcceleration = mSolverDesc.acceleration;
		mArticulationData.mSolverDataSize = mSolverDesc.solverDataSize;
		mArticulationData.mArticulation = this;

		computeDofs();
	}

	PxU32 FeatherstoneArticulation::getDofs()
	{
		PxU32 dofs = mArticulationData.getDofs();
		
		if (dofs == 0xffffffff)
		{
			computeDofs();
		}

		return mArticulationData.getDofs();
	}

	PxU32 FeatherstoneArticulation::getDof(const PxU32 linkID)
	{
		ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);
		return jointDatum.dof;
	}

	void FeatherstoneArticulation::applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag)
	{
		applyCacheToDest(mArticulationData, cache, mArticulationData.getJointVelocities(), mArticulationData.getJointAccelerations(),
			mArticulationData.getJointPositions(), mArticulationData.getJointForces(), flag);
	}

	void FeatherstoneArticulation::copyInternalStateToCache(PxArticulationCache& cache,
		const PxArticulationCacheFlags flag)
	{
		if (flag & PxArticulationCache::eVELOCITY)
		{
			copyJointData(mArticulationData, cache.jointVelocity, mArticulationData.getJointVelocities());
		}

		if (flag & PxArticulationCache::eACCELERATION)
		{
			copyJointData(mArticulationData, cache.jointAcceleration, mArticulationData.getJointAccelerations());
		}

		if (flag & PxArticulationCache::ePOSITION)
		{
			copyJointData(mArticulationData, cache.jointPosition, mArticulationData.getJointPositions());
		}

		if (flag & PxArticulationCache::eFORCE)
		{
			copyJointData(mArticulationData, cache.jointForce, mArticulationData.getJointForces());
		}

		if (flag & PxArticulationCache::eROOT)
		{
			ArticulationLink& rLink = mArticulationData.getLink(0);
			Cm::SpatialVectorF& vel = mArticulationData.getMotionVelocity(0);
			Cm::SpatialVectorF& acel = mArticulationData.getMotionAcceleration(0);
			PxsBodyCore& rBodyCore = *rLink.bodyCore;
			const PxTransform& body2World = rBodyCore.body2World;
			cache.rootLinkData.transform = body2World;
			cache.rootLinkData.linVel = vel.bottom;
			cache.rootLinkData.angVel = vel.top;
			cache.rootLinkData.linAcel = body2World.rotate(acel.bottom);
			cache.rootLinkData.angAcel = body2World.rotate(acel.top);
		}
	}

	void FeatherstoneArticulation::transformInertia(const SpatialTransform& sTod, SpatialMatrix& spatialInertia)
	{
		const SpatialTransform dTos = sTod.getTranspose();

		PxMat33 tl = sTod.R * spatialInertia.topLeft;
		PxMat33 tr = sTod.R * spatialInertia.topRight;
		PxMat33 bl = sTod.T * spatialInertia.topLeft + sTod.R * spatialInertia.bottomLeft;
		PxMat33 br = sTod.T * spatialInertia.topRight + sTod.R * spatialInertia.getBottomRight();

		spatialInertia.topLeft = tl * dTos.R + tr * dTos.T;
		spatialInertia.topRight = tr * dTos.R;
		spatialInertia.bottomLeft = bl * dTos.R + br * dTos.T;

		//aligned inertia
		spatialInertia.bottomLeft = (spatialInertia.bottomLeft + spatialInertia.bottomLeft.getTranspose()) * 0.5f;
	}

	void FeatherstoneArticulation::getImpulseResponse(
		PxU32 linkID,
		Cm::SpatialVectorF* Z,
		const Cm::SpatialVector& impulse,
		Cm::SpatialVector& deltaVV) const
	{
		PX_ASSERT(impulse.pad0 == 0.f && impulse.pad1 == 0.f);

		//impulse lin is contact normal, and ang is raxn. R is body2World, R(t) is world2Body
		//| R(t),	0	|
		//| R(t)*r, R(t)|
		//r is the vector from center of mass to contact point
		//p(impluse) =	|n|
		//				|0|

		//ArticulationLink* links = mArticulationData.getLinks();
		
		//ArticulationLink& link = links[linkID];
		//PxTransform& body2World = link.bodyCore->body2World;
		const PxTransform& body2World = mArticulationData.getPreTransform(linkID);

		//transform p(impluse) from world space to the local space of linkId
		Cm::SpatialVectorF impl = Cm::SpatialVectorF(body2World.rotateInv(impulse.linear), body2World.rotateInv(impulse.angular));

		getZ(linkID, mArticulationData, Z, impl);

		const bool fixBase = mArticulationData.getCore()->flags & PxArticulationFlag::eFIX_BASE;
		
		Cm::SpatialVectorF deltaV = getDeltaV(fixBase, linkID, mArticulationData, Z);
		PX_ASSERT(deltaV.pad0 == 0.f && deltaV.pad1 == 0.f);

		//this is in world space
		deltaVV.linear = body2World.rotate(deltaV.bottom);
		deltaVV.angular = body2World.rotate(deltaV.top);
	}

	//This will return world space SpatialVectorV
	Cm::SpatialVectorV FeatherstoneArticulation::getLinkVelocity(const PxU32 linkID) const
	{
		//This is in the world space
		const Cm::SpatialVectorF& motionVelocity = mArticulationData.getMotionVelocity(linkID);

		Cm::SpatialVectorV velocity;
		velocity.linear = V3LoadU(motionVelocity.bottom);
		velocity.angular = V3LoadU(motionVelocity.top);

		return velocity;
	}

	Cm::SpatialVectorV FeatherstoneArticulation::getLinkMotionVector(const PxU32 linkID) const
	{
		const Cm::SpatialVectorF& motionVector = mArticulationData.getDeltaMotionVector(linkID);
		
		//ArticulationLink& link = mArticulationData.getLink(linkID);
		//PxTransform& body2World = link.bodyCore->body2World;

		/*const PxVec3 mLinear = body2World.rotate(motionVector.bottom);
		const PxVec3 mAngular = body2World.rotate(motionVector.top);*/

		Cm::SpatialVectorV velocity;
		velocity.linear = V3LoadU(motionVector.bottom);
		velocity.angular = V3LoadU(motionVector.top);

		return velocity;
	}

	//this is called by island gen to determine whether the articulation should be awake or sleep
	Cm::SpatialVector FeatherstoneArticulation::getMotionVelocity(const PxU32 linkID) const
	{
		//This is in the body space
		const Cm::SpatialVectorF& motionVelocity = mArticulationData.getMotionVelocity(linkID);

	//	ArticulationLink& link = mArticulationData.getLink(linkID);
	//	PxTransform& body2World = link.bodyCore->body2World;

		/*const PxVec3 linear = body2World.rotate(motionVelocity.bottom);
		const PxVec3 angular = body2World.rotate(motionVelocity.top);*/

		return Cm::SpatialVector(motionVelocity.bottom, motionVelocity.top);
	}

	PxReal FeatherstoneArticulation::getLinkMaxPenBias(const PxU32 linkID) const
	{
		return mArticulationData.getLinkData(linkID).maxPenBias;
	}

	void PxcFsFlushVelocity(FeatherstoneArticulation& articulation, Cm::SpatialVectorF* deltaV)
	{
		ArticulationData& data = articulation.mArticulationData;
		const bool fixBase = data.getCore()->flags & PxArticulationFlag::eFIX_BASE;
		Cm::SpatialVectorF* motionVelocities = data.getMotionVelocities();
		Cm::SpatialVectorF* deferredZ = data.getSpatialZAVectors();
		ArticulationLink* links = data.getLinks();
		ArticulationLinkData* linkData = data.getLinkData();
		ArticulationJointCoreData* jointData = data.getJointData();

		//PxTransform* poses = data.getAccumulatedPoses();
		PxTransform* poses = data.getPreTransform();

		//This will be zero at the begining of the frame
		PxReal* jointDeltaVelocities = data.getJointDeltaVelocities();

		if (fixBase)
		{
			deltaV[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
		}
		else
		{
			//ArticulationLink& link = links[0];

			deltaV[0] = data.getBaseInvSpatialArticulatedInertia() * (-deferredZ[0]);
			//const PxTransform& body2World0 = link.bodyCore->body2World;
			const PxTransform& body2World0 = poses[0];

			motionVelocities[0] += deltaV[0].rotate(body2World0);

			PX_ASSERT(motionVelocities[0].isFinite());
		}

		const PxU32 linkCount = data.getLinkCount();

		for (PxU32 i = 1; i < linkCount; i++)
		{
			ArticulationLink& tLink = links[i];
			ArticulationLinkData& tLinkDatum = linkData[i];
			ArticulationJointCoreData& tJointDatum = jointData[i];

			Cm::SpatialVectorF dV = FeatherstoneArticulation::propagateVelocity(tLinkDatum, tJointDatum, deferredZ[i],
				&jointDeltaVelocities[tJointDatum.jointOffset], deltaV[tLink.parent]);

			deltaV[i] = dV;

			const PxTransform& tBody2World = poses[i];
			motionVelocities[i] += dV.rotate(tBody2World);

			PX_ASSERT(motionVelocities[i].isFinite());
		}

		PxMemZero(deferredZ, sizeof(Cm::SpatialVectorF)*linkCount);
	}

	void FeatherstoneArticulation::recordDeltaMotion(const ArticulationSolverDesc& desc, 
		const PxReal dt, Cm::SpatialVectorF* deltaV)
	{
		FeatherstoneArticulation* articulation = static_cast<FeatherstoneArticulation*>(desc.articulation);
		ArticulationData& data = articulation->mArticulationData;
		const PxU32 linkCount = data.getLinkCount();

		if(data.mJointDirty)
			PxcFsFlushVelocity(*articulation, deltaV);

		Cm::SpatialVectorF* deltaMotion = data.getDeltaMotionVector();
		Cm::SpatialVectorF* motionVeloties = data.getMotionVelocities();
		PX_UNUSED(motionVeloties);

		PxReal* jointPosition = data.getJointPositions();
		PxReal* jointVelocities = data.getJointVelocities();
		PxReal* jointDeltaVelocities = data.getJointDeltaVelocities();
		
		data.mAccumulatedDt += dt;
		data.setDt(dt);

		const bool fixBase = data.getCore()->flags & PxArticulationFlag::eFIX_BASE;

		if (!fixBase)
		{
			const Cm::SpatialVectorF& motionVelocity = data.getMotionVelocity(0);
			PX_ASSERT(motionVelocity.top.isFinite());
			PX_ASSERT(motionVelocity.bottom.isFinite());

			const PxTransform preTrans = data.mAccumulatedPoses[0];

			PxVec3 lin = motionVelocity.bottom;
			PxVec3 ang = motionVelocity.top;

			PxVec3 newP = preTrans.p + lin * dt;

			PxTransform newPose = PxTransform(newP, Ps::exp(ang*dt) * preTrans.q);

			//PxVec3 lin, ang;
			/*calculateNewVelocity(newPose, data.mPreTransform[0],
				1.f, lin, ang);		*/	

			data.mAccumulatedPoses[0] = newPose;

			PxQuat dq = newPose.q * data.mPreTransform[0].q.getConjugate();

			if (dq.w < 0.f)
				dq = -dq;

			data.mDeltaQ[0] = dq;

			deltaMotion[0] += motionVelocity * dt;
			/*deltaMotion[0].top = ang;
			deltaMotion[0].bottom = lin;*/
		}

		for (PxU32 linkID = 1; linkID < linkCount; linkID++)
		{
			ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
			
			PxTransform newPose = articulation->propagateTransform(linkID, data.getLinks(), jointDatum, data.getMotionVelocities(),
				dt, data.mAccumulatedPoses[data.getLink(linkID).parent], data.mAccumulatedPoses[linkID], 
				jointVelocities, jointDeltaVelocities, jointPosition);

			//data.mDeltaQ[linkID] = data.mPreTransform[linkID].q.getConjugate() * newPose.q;
			PxQuat dq = newPose.q * data.mPreTransform[linkID].q.getConjugate();

			if(dq.w < 0.f)
				dq = -dq;

			data.mDeltaQ[linkID] = dq;

			for (PxU32 i = 0, idx = jointDatum.jointOffset; i < jointDatum.dof; ++i, idx++)
			{
				jointDeltaVelocities[idx] = 0.f;
			}

			PxVec3 lin, ang;
			calculateNewVelocity(newPose, data.mPreTransform[linkID],
				1.f, lin, ang);
			
			deltaMotion[linkID].top = ang;// motionVeloties[linkID].top * dt;
			deltaMotion[linkID].bottom = lin;// motionVeloties[linkID].top * dt;

			//Record the new current pose
			data.mAccumulatedPoses[linkID] = newPose;

#if 0
			ArticulationLink& link = data.getLink(linkID);

			if (link.inboundJoint->jointType != PxArticulationJointType::eSPHERICAL)
			{
				const PxTransform& parentPose = data.mAccumulatedPoses[link.parent];
				const PxTransform& body2World = newPose;


				PxVec3 rw = body2World.p - parentPose.p;

				PxVec3 linVel = data.getMotionVelocity(link.parent).bottom + data.getMotionVelocity(link.parent).top.cross(rw);
				PxVec3 angVel = data.getMotionVelocity(link.parent).top;

				const PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];

				PxVec3 tDeltaV(0.f);
				PxVec3 tDeltaA(0.f);
				for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
				{
					tDeltaV += jointDatum.motionMatrix[ind].bottom * jVelocity[ind];
					tDeltaA += jointDatum.motionMatrix[ind].top * jVelocity[ind];
				}

				linVel += body2World.rotate(tDeltaV);
				angVel += body2World.rotate(tDeltaA);

				PX_UNUSED(linVel);

				motionVeloties[linkID].bottom = linVel;
				//motionVeloties[linkID].bottom = deltaChange/dt;
				motionVeloties[linkID].top = angVel;
			}
#endif
		}
	}

	void FeatherstoneArticulation::deltaMotionToMotionVelocity(const ArticulationSolverDesc& desc, PxReal invDt)
	{
		FeatherstoneArticulation* articulation = static_cast<FeatherstoneArticulation*>(desc.articulation);
		ArticulationData& data = articulation->mArticulationData;
		const PxU32 linkCount = data.getLinkCount();
		Cm::SpatialVectorF* deltaMotion = data.getDeltaMotionVector();

		for (PxU32 linkID = 0; linkID<linkCount; linkID++)
		{

			Cm::SpatialVectorF& v = data.getMotionVelocity(linkID);

			Cm::SpatialVectorF delta = deltaMotion[linkID] * invDt;

			v = delta;

			desc.motionVelocity[linkID] = reinterpret_cast<Cm::SpatialVectorV&>(delta);
		}
	}

	//This is used in the solveExt1D, solveExtContact
	Cm::SpatialVectorV FeatherstoneArticulation::pxcFsGetVelocity(PxU32 linkID)
	{
		Cm::SpatialVectorF* deferredZ = mArticulationData.getSpatialZAVectors();

		const bool fixBase = mArticulationData.getCore()->flags & PxArticulationFlag::eFIX_BASE;
	
		ArticulationLink* links = mArticulationData.getLinks();
		ArticulationLinkData* linkData = mArticulationData.getLinkData();
		ArticulationJointCoreData* jointData = mArticulationData.getJointData();

		Cm::SpatialVectorF deltaV = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));

		if (!fixBase)
		{
			deltaV = mArticulationData.mBaseInvSpatialArticulatedInertia * (-deferredZ[0]);
		}

		for (ArticulationBitField i = links[linkID].pathToRoot - 1; i; i &= (i - 1))
		{
			//index of child of link h on path to link linkID
			const PxU32 index = ArticulationLowestSetBit(i);
			ArticulationLinkData& tLinkDatum = linkData[index];
			ArticulationJointCoreData& tJointDatum = jointData[index];
			PX_ASSERT(links[index].parent < index);
			deltaV = propagateVelocityTestImpulse(tLinkDatum, tJointDatum, deferredZ[index], deltaV);
		}

		//ArticulationLink& link = mArticulationData.getLink(linkID);
		//PxTransform& body2World = link.bodyCore->body2World;
		//PxTransform& body2World = mArticulationData.getAccumulatedPoses()[linkID];
		PxTransform& body2World = mArticulationData.getPreTransform(linkID);
		Cm::SpatialVectorF vel = mArticulationData.getMotionVelocity(linkID) + deltaV.rotate(body2World);

		return Cm::SpatialVector(vel.bottom, vel.top);
	}

	void FeatherstoneArticulation::pxcFsGetVelocities(PxU32 linkID, PxU32 linkID1, Cm::SpatialVectorV& v0, Cm::SpatialVectorV& v1)
	{
		//if (mArticulationData.getLink(linkID1).parent == linkID)
		//{

		//	Cm::SpatialVectorF* deferredZ = mArticulationData.getSpatialZAVectors();

		//	const bool fixBase = mArticulationData.getCore()->flags & PxArticulationFlag::eFIX_BASE;

		//	ArticulationLink* links = mArticulationData.getLinks();
		//	ArticulationLinkData* linkData = mArticulationData.getLinkData();
		//	ArticulationJointCoreData* jointData = mArticulationData.getJointData();

		//	Cm::SpatialVectorF deltaV = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));

		//	if (!fixBase)
		//	{
		//		deltaV = mArticulationData.mBaseInvSpatialArticulatedInertia * (-deferredZ[0]);
		//	}

		//	for (ArticulationBitField i = links[linkID].pathToRoot - 1; i; i &= (i - 1))
		//	{
		//		//index of child of link h on path to link linkID
		//		const PxU32 index = ArticulationLowestSetBit(i);
		//		ArticulationLinkData& tLinkDatum = linkData[index];
		//		ArticulationJointCoreData& tJointDatum = jointData[index];
		//		PX_ASSERT(links[index].parent < index);
		//		deltaV = propagateVelocityTestImpulse(tLinkDatum, tJointDatum, deferredZ[index], deltaV);
		//	}

		//	ArticulationLink& link = mArticulationData.getLink(linkID);
		//	PxTransform& body2World = link.bodyCore->body2World;
		//	Cm::SpatialVectorF vel = mArticulationData.getMotionVelocity(linkID) + deltaV.rotate(body2World);

		//	v0 = Cm::SpatialVector(vel.bottom, vel.top);

		//	{
		//		ArticulationLinkData& tLinkDatum = linkData[linkID1];
		//		ArticulationJointCoreData& tJointDatum = jointData[linkID1];
		//		//Now do the final step for the child link...
		//		deltaV = propagateVelocityTestImpulse(tLinkDatum, tJointDatum, deferredZ[linkID1], deltaV);

		//		ArticulationLink& link1 = mArticulationData.getLink(linkID1);
		//		PxTransform& body2World1 = link1.bodyCore->body2World;
		//		Cm::SpatialVectorF vel1 = mArticulationData.getMotionVelocity(linkID1) + deltaV.rotate(body2World1);
		//		v1 = Cm::SpatialVector(vel1.bottom, vel1.top);
		//	}

		//}
		//else
		{
			Cm::SpatialVectorF* deferredZ = mArticulationData.getSpatialZAVectors();

			const bool fixBase = mArticulationData.getCore()->flags & PxArticulationFlag::eFIX_BASE;

			ArticulationLink* links = mArticulationData.getLinks();
			ArticulationLinkData* linkData = mArticulationData.getLinkData();
			ArticulationJointCoreData* jointData = mArticulationData.getJointData();

			Cm::SpatialVectorF deltaV = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));

			if (!fixBase)
			{
				deltaV = mArticulationData.mBaseInvSpatialArticulatedInertia * (-deferredZ[0]);
			}

			PxU64 common = links[linkID].pathToRoot & links[linkID1].pathToRoot;
			PxU64 exclusive0 = links[linkID].pathToRoot ^ common;
			PxU64 exclusive1 = links[linkID1].pathToRoot ^ common;

			for (ArticulationBitField i = common - 1; i; i &= (i - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 index = ArticulationLowestSetBit(i);
				ArticulationLinkData& tLinkDatum = linkData[index];
				ArticulationJointCoreData& tJointDatum = jointData[index];
				PX_ASSERT(links[index].parent < index);
				deltaV = propagateVelocityTestImpulse(tLinkDatum, tJointDatum, deferredZ[index], deltaV);
			}

			Cm::SpatialVectorF deltaV1 = deltaV;

			for (ArticulationBitField i = exclusive0; i; i &= (i - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 index = ArticulationLowestSetBit(i);
				ArticulationLinkData& tLinkDatum = linkData[index];
				ArticulationJointCoreData& tJointDatum = jointData[index];
				PX_ASSERT(links[index].parent < index);
				deltaV = propagateVelocityTestImpulse(tLinkDatum, tJointDatum, deferredZ[index], deltaV);
			}

			for (ArticulationBitField i = exclusive1; i; i &= (i - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 index = ArticulationLowestSetBit(i);
				ArticulationLinkData& tLinkDatum = linkData[index];
				ArticulationJointCoreData& tJointDatum = jointData[index];
				PX_ASSERT(links[index].parent < index);
				deltaV1 = propagateVelocityTestImpulse(tLinkDatum, tJointDatum, deferredZ[index], deltaV1);
			}

			//ArticulationLink& link = mArticulationData.getLink(linkID);
			//PxTransform& body2World = link.bodyCore->body2World;
			//PxTransform& body2World = mArticulationData.getAccumulatedPoses()[linkID];
			PxTransform& body2World = mArticulationData.getPreTransform(linkID);
			Cm::SpatialVectorF vel = mArticulationData.getMotionVelocity(linkID) + deltaV.rotate(body2World);

			v0 = Cm::SpatialVector(vel.bottom, vel.top);

			//ArticulationLink& link1 = mArticulationData.getLink(linkID1);
			//PxTransform& body2World1 = link1.bodyCore->body2World;
			//PxTransform& body2World1 = mArticulationData.getAccumulatedPoses()[linkID1];
			PxTransform& body2World1 = mArticulationData.getPreTransform(linkID1);
			Cm::SpatialVectorF vel1 = mArticulationData.getMotionVelocity(linkID1) + deltaV1.rotate(body2World1);

			v1 = Cm::SpatialVector(vel1.bottom, vel1.top);
		}
	}

	/*Cm::SpatialVectorV FeatherstoneArticulation::pxcFsGetVelocity(PxU32 linkID)
	{

	Cm::SpatialVectorF& vel = mArticulationData.getMotionVelocity(linkID);

	return Cm::SpatialVector(vel.bottom, vel.top);
	}*/

	Cm::SpatialVectorV FeatherstoneArticulation::pxcFsGetVelocityTGS(PxU32 linkID)
	{
		return getLinkVelocity(linkID);
	}

	//This is used in the solveExt1D, solveExtContact
	void FeatherstoneArticulation::pxcFsApplyImpulse(PxU32 linkID, 
		Ps::aos::Vec3V linear, Ps::aos::Vec3V angular,
		Cm::SpatialVectorF* /*Z*/, Cm::SpatialVectorF* /*deltaV*/)
	{
		const ArticulationSolverDesc* desc = &mSolverDesc;

		ArticulationLink* links = static_cast<ArticulationLink*>(desc->links);

		//initialize all zero acceration impulse to be zero
		ArticulationData& data = mArticulationData;
		ArticulationLinkData* linkData = data.getLinkData();
		ArticulationJointCoreData* jointData = data.getJointData();

		Cm::SpatialVectorF* deferredZ = data.getSpatialZAVectors();

		//This will be zero at the begining of the frame
		//PxReal* jointDeltaVelocities = data.getJointDeltaVelocities();

		//Cm::SpatialVectorF* motionVelocity = data.getMotionVelocities();

		data.mJointDirty = true;
		
		//set all zero acceleration forces to be zero
		//PxMemZero(Z, sizeof(Cm::SpatialVectorF) * desc->linkCount);

		//impulse is in world space
		Cm::SpatialVector impulse;
		V3StoreU(angular, impulse.angular);
		V3StoreU(linear, impulse.linear);

		//transform p(impluse) from world space to the local space of link
		//ArticulationLink& collidedlink = links[linkID];
		//PxTransform& body2World = collidedlink.bodyCore->body2World;
		//PxTransform& body2World = data.getAccumulatedPoses()[linkID];
		PxTransform& body2World = data.getPreTransform(linkID);

		Cm::SpatialVectorF Z0(-body2World.rotateInv(impulse.linear), -body2World.rotateInv(impulse.angular));
		deferredZ[linkID] += Z0;

		for (PxU32 i = linkID; i; i = links[i].parent)
		{
			ArticulationLink& tLink = links[i];
			ArticulationLinkData& tLinkDatum = linkData[i];
			ArticulationJointCoreData& jointDatum = jointData[i];
			Z0 = propagateImpulse(tLinkDatum, jointDatum, Z0);
			deferredZ[tLink.parent] += Z0;
		}
	}

	void FeatherstoneArticulation::pxcFsApplyImpulses(Cm::SpatialVectorF* Z)
	{
		ArticulationLink* links = mArticulationData.getLinks();

		//initialize all zero acceration impulse to be zero
		ArticulationData& data = mArticulationData;
		ArticulationLinkData* linkData = data.getLinkData();
		ArticulationJointCoreData* jointData = data.getJointData();

		const PxU32 linkCount = mArticulationData.getLinkCount();

		Cm::SpatialVectorF* deferredZ = data.getSpatialZAVectors();

		const PxU32 startIndex = PxU32(linkCount - 1);

		data.mJointDirty = true;

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			ArticulationLink& tLink = links[linkID];
			ArticulationLinkData& tLinkDatum = linkData[linkID];
			ArticulationJointCoreData& jointDatum = jointData[linkID];

			Z[tLink.parent] += propagateImpulse(tLinkDatum, jointDatum, Z[linkID]);
			deferredZ[linkID] += Z[linkID];
		}

		deferredZ[0] += Z[0];
	}

	void FeatherstoneArticulation::pxcFsApplyImpulses(PxU32 linkID, const Ps::aos::Vec3V& linear,
		const Ps::aos::Vec3V& angular, PxU32 linkID2, const Ps::aos::Vec3V& linear2,
		const Ps::aos::Vec3V& angular2, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* /*deltaV*/)
	{
		if (0)
		{
			pxcFsApplyImpulse(linkID, linear, angular, Z, NULL);
			pxcFsApplyImpulse(linkID2, linear2, angular2, Z, NULL);
		}
		else
		{
			const ArticulationSolverDesc* desc = &mSolverDesc;
			ArticulationData& data = mArticulationData;
			data.mJointDirty = true;
			Cm::SpatialVectorF* deferredZ = data.getSpatialZAVectors();
			ArticulationLink* links = static_cast<ArticulationLink*>(desc->links);
			ArticulationLinkData* linkData = data.getLinkData();
			ArticulationJointCoreData* jointData = data.getJointData();

			//impulse is in world space
			Cm::SpatialVector impulse0;
			V3StoreU(angular, impulse0.angular);
			V3StoreU(linear, impulse0.linear);

			Cm::SpatialVector impulse1;
			V3StoreU(angular2, impulse1.angular);
			V3StoreU(linear2, impulse1.linear);

			//transform p(impluse) from world space to the local space of link
			//ArticulationLink& collidedlink = links[linkID];
			//ArticulationLink& collidedlink2 = links[linkID2];
			/*PxTransform& body2World = collidedlink.bodyCore->body2World;
			PxTransform& body2World2 = collidedlink2.bodyCore->body2World;*/

			/*PxTransform& body2World = data.getAccumulatedPoses()[linkID];
			PxTransform& body2World2 = data.getAccumulatedPoses()[linkID2];*/

			PxTransform& body2World = data.getPreTransform(linkID);
			PxTransform& body2World2 = data.getPreTransform(linkID2);

			PxU64 commonId = links[linkID].pathToRoot & links[linkID2].pathToRoot;
			PxU32 commonLink = Dy::ArticulationHighestSetBit(commonId); //Now, work from one to that common, then the other to that common, then go from there upwards...

			Cm::SpatialVectorF Z1 = Cm::SpatialVectorF(-body2World.rotateInv(impulse0.linear), -body2World.rotateInv(impulse0.angular));
			Cm::SpatialVectorF Z2 = Cm::SpatialVectorF(-body2World2.rotateInv(impulse1.linear), -body2World2.rotateInv(impulse1.angular));

			Z[linkID2] = Z2;
			deferredZ[linkID2] += Z2;

			for (PxU32 i = linkID2; i != commonLink; i = links[i].parent)
			{
				ArticulationLink& tLink = links[i];
				ArticulationLinkData& tLinkDatum = linkData[i];
				ArticulationJointCoreData& jointDatum = jointData[i];
				Z2 = propagateImpulse(tLinkDatum, jointDatum, Z2);
				Z[tLink.parent] = Z2;
				deferredZ[tLink.parent] += Z2;
			}

			Z[linkID] = Z1;
			deferredZ[linkID] += Z1;

			for (PxU32 i = linkID; i != commonLink; i = links[i].parent)
			{
				ArticulationLink& tLink = links[i];
				ArticulationLinkData& tLinkDatum = linkData[i];
				ArticulationJointCoreData& jointDatum = jointData[i];
				Z1 = propagateImpulse(tLinkDatum, jointDatum, Z1);
				Z[tLink.parent] = Z1;
				deferredZ[tLink.parent] += Z1;
			}

			Z[commonLink] = Z1 + Z2;

			for (PxU32 i = commonLink; i; i = links[i].parent)
			{
				ArticulationLink& tLink = links[i];
				ArticulationLinkData& tLinkDatum = linkData[i];
				ArticulationJointCoreData& jointDatum = jointData[i];
				Z[tLink.parent] = propagateImpulse(tLinkDatum, jointDatum, Z[i]);
				deferredZ[tLink.parent] += Z[tLink.parent];
			}
		}
	}

	//Z is the link space(drag force)
	void FeatherstoneArticulation::applyImpulses(Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV)
	{
		ArticulationLink* links = mArticulationData.getLinks();

		//initialize all zero acceration impulse to be zero
		ArticulationData& data = mArticulationData;
		ArticulationLinkData* linkData = data.getLinkData();
		ArticulationJointCoreData* jointData = data.getJointData();

		const PxU32 linkCount = mArticulationData.getLinkCount();

		const PxU32 startIndex = PxU32(linkCount - 1);

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			ArticulationLink& tLink = links[linkID];
			ArticulationLinkData& tLinkDatum = linkData[linkID];
			ArticulationJointCoreData& jointDatum = jointData[linkID];
			
			Z[tLink.parent] += propagateImpulse(tLinkDatum, jointDatum, Z[linkID]);
		}

		getDeltaV(Z, deltaV);
	}

	void FeatherstoneArticulation::getDeltaV(Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV)
	{
		const bool fixBase = mArticulationData.getCore()->flags & PxArticulationFlag::eFIX_BASE;
		Cm::SpatialVectorF* motionVelocities = mArticulationData.getMotionVelocities();
		ArticulationLink* links = mArticulationData.getLinks();
		ArticulationLinkData* linkData = mArticulationData.getLinkData();
		ArticulationJointCoreData* jointData = mArticulationData.getJointData();

		//This will be zero at the begining of the frame
		PxReal* jointDeltaVelocities = mArticulationData.getJointDeltaVelocities();

		if (fixBase)
		{
			deltaV[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
		}
		else
		{
			ArticulationLink& link = links[0];

			deltaV[0] = mArticulationData.mBaseInvSpatialArticulatedInertia * (-Z[0]);
			const PxTransform& body2World0 = link.bodyCore->body2World;
			motionVelocities[0] += deltaV[0].rotate(body2World0);

			PX_ASSERT(motionVelocities[0].isFinite());
		}

		const PxU32 linkCount = mArticulationData.getLinkCount();
		
		for (PxU32 i = 1; i < linkCount; i++)
		{
			ArticulationLink& tLink = links[i];
			ArticulationLinkData& tLinkDatum = linkData[i];
			ArticulationJointCoreData& tJointDatum = jointData[i];
			Cm::SpatialVectorF dV = propagateVelocity(tLinkDatum, tJointDatum, Z[i],
				&jointDeltaVelocities[tJointDatum.jointOffset], deltaV[tLink.parent]);

			deltaV[i] = dV;
			const PxTransform& tBody2World = tLink.bodyCore->body2World;

			motionVelocities[i] += dV.rotate(tBody2World);

			PX_ASSERT(motionVelocities[i].isFinite());
		}
	}

	PxQuat computeSphericalJointPositions(ArticulationJointCore* joint,
		const PxQuat newRot, const PxQuat pBody2WorldRot,
		PxReal* jPositions);

	PxTransform FeatherstoneArticulation::propagateTransform(const PxU32 linkID, ArticulationLink* links,
		ArticulationJointCoreData& jointDatum, Cm::SpatialVectorF* motionVelocities, const PxReal dt, const PxTransform& pBody2World, 
		const PxTransform& currentTransform, PxReal* jointVelocities, PxReal* jointDeltaVelocities, PxReal* jointPositions)
	{
			ArticulationLink& link = links[linkID];
		
			ArticulationJointCore* joint = link.inboundJoint;
		
			PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
			PxReal* jDeltaVelocity = &jointDeltaVelocities[jointDatum.jointOffset];
			PxReal* jPosition = &jointPositions[jointDatum.jointOffset];
		
			PxQuat newParentToChild;
			PxQuat newWorldQ;
			PxVec3 r;
		
			const PxVec3 childOffset = -joint->childPose.p;
			const PxVec3 parentOffset = joint->parentPose.p;
		
			switch (joint->jointType)
			{
			case PxArticulationJointType::ePRISMATIC:
			{
				PxReal tJointPosition = jPosition[0] + (jVelocity[0] + jDeltaVelocity[0]) * dt;

				if (link.inboundJoint->prismaticLimited)
				{
					if (tJointPosition < (link.inboundJoint->limits[link.inboundJoint->dofIds[0]].low))
						tJointPosition = link.inboundJoint->limits[link.inboundJoint->dofIds[0]].low;
					if (tJointPosition >(link.inboundJoint->limits[link.inboundJoint->dofIds[0]].high))
						tJointPosition = link.inboundJoint->limits[link.inboundJoint->dofIds[0]].high;
				}

				jPosition[0] = tJointPosition;
				jVelocity[0] += jDeltaVelocity[0];
				jDeltaVelocity[0] = 0.f;

				newParentToChild = joint->relativeQuat;
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
		
				r = e + d + jointDatum.motionMatrix[0].bottom * tJointPosition;
				break;
			}
			case PxArticulationJointType::eREVOLUTE:
			{
				PxReal tJointPosition = jPosition[0] + (jVelocity[0] + jDeltaVelocity[0]) * dt;

				if (link.inboundJoint->twistLimited)
				{
					if (tJointPosition < (link.inboundJoint->limits[link.inboundJoint->dofIds[0]].low))
						tJointPosition = link.inboundJoint->limits[link.inboundJoint->dofIds[0]].low;
					if (tJointPosition >(link.inboundJoint->limits[link.inboundJoint->dofIds[0]].high))
						tJointPosition = link.inboundJoint->limits[link.inboundJoint->dofIds[0]].high;
				}
				else
				{
					if (tJointPosition > PxTwoPi)
						tJointPosition -= 2.f*PxTwoPi;
					else if (tJointPosition < -PxTwoPi)
						tJointPosition += 2.f*PxTwoPi;

					tJointPosition = PxClamp(tJointPosition, -2.f*PxTwoPi, 2.f*PxTwoPi);
				}

				jPosition[0] = tJointPosition;
				jVelocity[0] += jDeltaVelocity[0];
				jDeltaVelocity[0] = 0.f;
		
				const PxVec3& u = jointDatum.motionMatrix[0].top;
		
				PxQuat jointRotation = PxQuat(-tJointPosition, u);
				if (jointRotation.w < 0)	//shortest angle.
					jointRotation = -jointRotation;
		
				newParentToChild = (jointRotation * joint->relativeQuat).getNormalized();
		
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
				r = e + d;
		
				PX_ASSERT(r.isFinite());
		
				break;
			}
			case PxArticulationJointType::eSPHERICAL:
			{
				if (jointDatum.dof < 3)
				{
					PxQuat jointRotation(PxIdentity);
					for (PxU32 i = 0; i < jointDatum.dof; ++i)
					{
						const PxReal delta = (jVelocity[i] + jDeltaVelocity[i]) * dt;

						jVelocity[i] += jDeltaVelocity[i];

						PxReal jPos = jPosition[i] + delta;
						//jPosition[i] += delta;
						if (jPos > PxTwoPi)
							jPos -= 2.f*PxTwoPi;
						else if (jPos < -PxTwoPi)
							jPos += 2.f*PxTwoPi;

						jPos = PxClamp(jPos, -2.f*PxTwoPi, 2.f*PxTwoPi);

						jPosition[i] = jPos;

						jDeltaVelocity[i] = 0.f;

						const PxVec3& u = jointDatum.motionMatrix[i].top;

						PxQuat jRotation = PxQuat(-jPosition[i], u);
						if (jRotation.w < 0)	//shortest angle.
							jRotation = -jRotation;

						jointRotation = jointRotation * jRotation;	
					}
					newParentToChild = (jointRotation * joint->relativeQuat).getNormalized();
					const PxVec3 e = newParentToChild.rotate(parentOffset);
					const PxVec3 d = childOffset;
					r = e + d;

					PX_ASSERT(r.isFinite());
				}
				else
				{
					PxVec3 worldAngVel = motionVelocities[linkID].top;

					newWorldQ = Ps::exp(worldAngVel*dt) * currentTransform.q;

					newParentToChild = computeSphericalJointPositions(joint, newWorldQ,
						pBody2World.q, jPosition);

					PxQuat newQ = (pBody2World.q * newParentToChild.getConjugate()).getNormalized();

					const PxQuat cB2w = newQ * joint->childPose.q;

					const PxMat33 cB2w_m(cB2w);

					PxVec3 axis0 = cB2w_m.column0;
					PxVec3 axis1 = cB2w_m.column1;
					PxVec3 axis2 = cB2w_m.column2;

					PxVec3 relAngVel = worldAngVel - motionVelocities[link.parent].top;

					PxU32 dofIdx = 0;
					if (joint->motion[PxArticulationAxis::eTWIST] != PxArticulationMotion::eLOCKED)
						jVelocity[dofIdx++] = axis0.dot(relAngVel);
					if (joint->motion[PxArticulationAxis::eSWING1] != PxArticulationMotion::eLOCKED)
						jVelocity[dofIdx++] = axis1.dot(relAngVel);
					if (joint->motion[PxArticulationAxis::eSWING2] != PxArticulationMotion::eLOCKED)
						jVelocity[dofIdx++] = axis2.dot(relAngVel);
					if (joint->motion[PxArticulationAxis::eTWIST] == PxArticulationMotion::eLOCKED)
						jVelocity[dofIdx++] = axis0.dot(relAngVel);
					if (joint->motion[PxArticulationAxis::eSWING1] == PxArticulationMotion::eLOCKED)
						jVelocity[dofIdx++] = axis1.dot(relAngVel);
					if (joint->motion[PxArticulationAxis::eSWING2] == PxArticulationMotion::eLOCKED)
						jVelocity[dofIdx++] = axis2.dot(relAngVel);
				}
		
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
				r = e + d;
		
				PX_ASSERT(r.isFinite());
				break;
			}
			case PxArticulationJointType::eFIX:
			{
				//this is fix joint so joint don't have velocity
				newParentToChild = joint->relativeQuat;
		
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
		
				r = e + d;
				break;
			}
			default:
				break;
			}
		
			PxTransform cBody2World;
			cBody2World.q = (pBody2World.q * newParentToChild.getConjugate()).getNormalized();
			cBody2World.p = pBody2World.p + cBody2World.q.rotate(r);
		
			PX_ASSERT(cBody2World.isSane());
		
			return cBody2World;
	}

	const PxTransform& FeatherstoneArticulation::getCurrentTransform(PxU32 linkID) const
	{
		return mArticulationData.mAccumulatedPoses[linkID];
	}

	const PxQuat& FeatherstoneArticulation::getDeltaQ(PxU32 linkID) const
	{
		return mArticulationData.mDeltaQ[linkID];
	}

	//Z is the spatial acceleration impulse of links[linkID]
	Cm::SpatialVectorF FeatherstoneArticulation::propagateVelocity(const ArticulationLinkData& linkDatum, const ArticulationJointCoreData& jointDatum,
		const Cm::SpatialVectorF& Z, PxReal* jointVelocity, const Cm::SpatialVectorF& hDeltaV)
	{
		Cm::SpatialVectorF pDeltaV = linkDatum.childToParent.transposeTransform(hDeltaV); //parent velocity change

		Cm::SpatialVectorF temp = linkDatum.spatialArticulatedInertia * pDeltaV + Z;

		PxReal tJointDelta[6];
		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
		{
			const Cm::SpatialVectorF& sa = jointDatum.motionMatrix[ind];
			tJointDelta[ind] = -sa.innerProduct(temp);
		}

		Cm::SpatialVectorF jointSpatialDeltaV(PxVec3(0.f), PxVec3(0.f));

		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
		{
			PxReal jDelta = 0.f;
			for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
			{
				jDelta += linkDatum.invStIs[ind2][ind] * tJointDelta[ind2];
			}

			jointVelocity[ind] += jDelta;
			//PX_ASSERT(PxAbs(jointVelocity[ind]) < 500.f);


			const Cm::SpatialVectorF& sa = jointDatum.motionMatrix[ind];
			jointSpatialDeltaV += sa * jDelta;
		}

		return pDeltaV + jointSpatialDeltaV;
	}

	//This method calculate the velocity change due to collision/constraint impulse
	Cm::SpatialVectorF FeatherstoneArticulation::propagateVelocityTestImpulse(const ArticulationLinkData& linkDatum, const ArticulationJointCoreData& jointDatum, const Cm::SpatialVectorF& Z,
		const Cm::SpatialVectorF& hDeltaV)
	{
		const SpatialTransform& c2p = linkDatum.childToParent;
		Cm::SpatialVectorF pDeltaV = c2p.transposeTransform(hDeltaV); //parent velocity change

		Cm::SpatialVectorF temp = linkDatum.spatialArticulatedInertia * pDeltaV + Z;

		PxReal tJointDelta[6];
		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
		{
			const Cm::SpatialVectorF& sa = jointDatum.motionMatrix[ind];
			tJointDelta[ind] = -sa.innerProduct(temp);
		}

		Cm::SpatialVectorF jointSpatialDeltaV(PxVec3(0.f), PxVec3(0.f));

		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
		{
			PxReal jDelta = 0.f;
			for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
			{
				jDelta += linkDatum.invStIs[ind2][ind] * tJointDelta[ind2];
			}

			const Cm::SpatialVectorF& sa = jointDatum.motionMatrix[ind];
			jointSpatialDeltaV += sa * jDelta;
		}

		return pDeltaV + jointSpatialDeltaV;
	}

	//PX_FORCE_INLINE Cm::SpatialVectorF ComputeDeltaVelocity(const ArticulationLinkData& linkDatum, 
	//	const ArticulationJointCoreData& jointDatum, Cm::SpatialVectorF& hDeltaV)
	//{
	//	const SpatialTransform& c2p = linkDatum.childToParent;
	//	Cm::SpatialVectorF pDeltaV = c2p.transposeTransform(hDeltaV); //parent velocity change
	//	Cm::SpatialVectorF temp = linkDatum.spatialArticulatedInertia * pDeltaV;
	//	PxReal tJointDelta[6];
	//	for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
	//	{
	//		const Cm::SpatialVectorF& sa = jointDatum.motionMatrix[ind];
	//		tJointDelta[ind] = -sa.innerProduct(temp);
	//	}

	//	Cm::SpatialVectorF jointSpatialDeltaV(PxVec3(0.f), PxVec3(0.f));

	//	for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
	//	{
	//		PxReal jDelta = 0.f;
	//		for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
	//		{
	//			jDelta += linkDatum.invStIs[ind2][ind] * tJointDelta[ind2];
	//		}

	//		const Cm::SpatialVectorF& sa = jointDatum.motionMatrix[ind];
	//		jointSpatialDeltaV += sa * jDelta;
	//	}

	//	return pDeltaV + jointSpatialDeltaV;
	//}

	Cm::SpatialVectorF FeatherstoneArticulation::propagateImpulse(const ArticulationLinkData& linkDatum, 
		const ArticulationJointCoreData& jointDatum, const Cm::SpatialVectorF& Z)
	{
		Cm::SpatialVectorF temp(PxVec3(0.f), PxVec3(0.f));

		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
		{
			const Cm::SpatialVectorF& sa = jointDatum.motionMatrix[ind];
			const PxReal stZ = sa.innerProduct(Z);
			temp += linkDatum.IsInvD[ind] * stZ;
		}

		//parent space's spatial zero acceleration impulse
		return  linkDatum.childToParent * (Z - temp);
	}

	Cm::SpatialVectorF FeatherstoneArticulation::getDeltaVWithDeltaJV(const bool fixBase, const PxU32 linkID, 
		const ArticulationData& data, Cm::SpatialVectorF* Z,
		PxReal* jointVelocities)
	{
		Cm::SpatialVectorF deltaV = Cm::SpatialVectorF::Zero();
		if (!fixBase)
		{
			//velocity change
			//SpatialMatrix inverseArticulatedInertia = hLinkDatum.spatialArticulatedInertia.getInverse();
			const SpatialMatrix& inverseArticulatedInertia = data.mBaseInvSpatialArticulatedInertia;
			deltaV = inverseArticulatedInertia * (-Z[0]);
		}

		for (ArticulationBitField i = data.getLink(linkID).pathToRoot - 1; i; i &= (i - 1))
		{
			const PxU32 index = ArticulationLowestSetBit(i);
			ArticulationLinkData& tLinkDatum = data.getLinkData(index);
			ArticulationJointCoreData& tJointDatum = data.getJointData(index);
			PxReal* jVelocity = &jointVelocities[tJointDatum.jointOffset];
			deltaV = FeatherstoneArticulation::propagateVelocity(tLinkDatum, tJointDatum, Z[index], jVelocity, deltaV);
		}

		return deltaV;
	}

	Cm::SpatialVectorF FeatherstoneArticulation::getDeltaV(const bool fixBase, const PxU32 linkID,
		const ArticulationData& data, Cm::SpatialVectorF* Z)
	{
		Cm::SpatialVectorF deltaV = Cm::SpatialVectorF::Zero();
		if (!fixBase)
		{
			//velocity change
			//SpatialMatrix inverseArticulatedInertia = hLinkDatum.spatialArticulatedInertia.getInverse();
			const SpatialMatrix& inverseArticulatedInertia = data.mBaseInvSpatialArticulatedInertia;
			deltaV = inverseArticulatedInertia * (-Z[0]);
		}

		for (ArticulationBitField i = data.getLink(linkID).pathToRoot - 1; i; i &= (i - 1))
		{
			const PxU32 index = ArticulationLowestSetBit(i);
			ArticulationLinkData& tLinkDatum = data.getLinkData(index);
			ArticulationJointCoreData& tJointDatum = data.getJointData(index);
			deltaV = FeatherstoneArticulation::propagateVelocityTestImpulse(tLinkDatum, tJointDatum, Z[index], deltaV);
		}

		return deltaV;
	}

	void  FeatherstoneArticulation::getZ(const PxU32 linkID,
		const ArticulationData& data, Cm::SpatialVectorF* Z, 
		const Cm::SpatialVectorF& impulse)
	{
		ArticulationLink* links = data.getLinks();

		//impulse need to be in linkID space!!!
		Z[linkID] = -impulse;

		for (PxU32 i = linkID; i; i = links[i].parent)
		{
			ArticulationLink& tLink = links[i];
			const ArticulationLinkData& tLinkDatum = data.getLinkData(i);
			const ArticulationJointCoreData& tJointDatum = data.getJointData(i);
			Z[tLink.parent] = FeatherstoneArticulation::propagateImpulse(tLinkDatum, tJointDatum, Z[i]);
		}
	}

	//This method use in impulse self response. The input impulse is in the link space
	Cm::SpatialVectorF FeatherstoneArticulation::getImpulseResponse(
		const PxU32 linkID,
		const bool fixBase,
		const ArticulationData& data,
		Cm::SpatialVectorF* Z,
		const Cm::SpatialVectorF& impulse)
	{
		getZ(linkID, data, Z, impulse);

		return getDeltaV(fixBase, linkID, data, Z);
	}

	//This method use in impulse self response. The input impulse is in the link space
	Cm::SpatialVectorF FeatherstoneArticulation::getImpulseResponseWithJ(
		const PxU32 linkID,
		const bool fixBase,
		const ArticulationData& data,
		Cm::SpatialVectorF* Z,
		const Cm::SpatialVectorF& impulse,
		PxReal* jointVelocites)
	{
		getZ(linkID, data, Z, impulse);

		return getDeltaVWithDeltaJV(fixBase, linkID, data, Z, jointVelocites);
	}

	void FeatherstoneArticulation::saveVelocity(const ArticulationSolverDesc& d, Cm::SpatialVectorF* deltaV)
	{
		FeatherstoneArticulation* arti = static_cast<FeatherstoneArticulation*>(d.articulation);
		ArticulationData& data = arti->mArticulationData;

		//update all links' motion velocity, joint delta velocity if there are contacts/constraints
		if (data.mJointDirty)
			PxcFsFlushVelocity(*arti, deltaV);

		const PxU32 linkCount = data.getLinkCount();
		//copy motion velocites
		Cm::SpatialVectorF* vels = data.getMotionVelocities();
		Cm::SpatialVectorF* posVels = data.getPosIterMotionVelocities();
		PxMemCopy(posVels, vels, sizeof(Cm::SpatialVectorF) * linkCount);
	
		//copy joint velocities
		const PxU32 dofs = data.getDofs();

		PxReal* jPosDeltaVels = data.getPosIterJointDeltaVelocities();

		const PxReal* jDeltaVels = data.getJointDeltaVelocities();

		PxMemCopy(jPosDeltaVels, jDeltaVels, sizeof(PxReal) * dofs);

	/*	for (PxU32 i = 0; i < dofs; ++i)
		{
			PX_ASSERT(PxAbs(jPosDeltaVels[i]) < 30.f);
		}*/
	}

	void FeatherstoneArticulation::saveVelocityTGS(const ArticulationSolverDesc& d, PxReal invDtF32)
	{
		PX_UNUSED(d);
		PX_UNUSED(invDtF32);
	}

	void FeatherstoneArticulation::getImpulseSelfResponse(
		PxU32 linkID0,
		PxU32 linkID1,
		Cm::SpatialVectorF* Z,
		const Cm::SpatialVector& impulse0,
		const Cm::SpatialVector& impulse1,
		Cm::SpatialVector& deltaV0,
		Cm::SpatialVector& deltaV1) const
	{
		FeatherstoneArticulation::getImpulseSelfResponse(mArticulationData.getLinks(), !!(mArticulationData.getCore()->flags & PxArticulationFlag::eFIX_BASE),
			Z, mArticulationData, linkID0, reinterpret_cast<const Cm::SpatialVectorV&>(impulse0), 
			reinterpret_cast<Cm::SpatialVectorV&>(deltaV0), linkID1, reinterpret_cast<const Cm::SpatialVectorV&>(impulse1), 
			reinterpret_cast<Cm::SpatialVectorV&>(deltaV1));
	}

	void getImpulseResponseSlow(Dy::ArticulationLink* links,
		const ArticulationData& data,
		PxU32 linkID0_,
		const Cm::SpatialVector& impulse0,
		Cm::SpatialVector& deltaV0,
		PxU32 linkID1_,
		const Cm::SpatialVector& impulse1,
		Cm::SpatialVector& deltaV1)
	{
		PxU32 stack[DY_ARTICULATION_MAX_SIZE];
		Cm::SpatialVectorF Z[DY_ARTICULATION_MAX_SIZE];
		//Cm::SpatialVectorF ZZV[DY_ARTICULATION_MAX_SIZE];

		PxU32 i0, i1, ic;

		PxU32 linkID0 = linkID0_;
		PxU32 linkID1 = linkID1_;

		const PxTransform& transform0 = data.getLink(linkID0_).bodyCore->body2World;
		const PxTransform& transform1 = data.getLink(linkID1_).bodyCore->body2World;

		for (i0 = linkID0, i1 = linkID1; i0 != i1;)	// find common path
		{
			if (i0<i1)
				i1 = links[i1].parent;
			else
				i0 = links[i0].parent;
		}

		PxU32 common = i0;

		Cm::SpatialVectorF Z0(-transform0.rotateInv(impulse0.linear), -transform0.rotateInv(impulse0.angular));
		Cm::SpatialVectorF Z1(-transform1.rotateInv(impulse1.linear), -transform1.rotateInv(impulse1.angular));

		Z[linkID0] = Z0;
		Z[linkID1] = Z1;

		for (i0 = 0; linkID0 != common; linkID0 = links[linkID0].parent)
		{
			Z0 = FeatherstoneArticulation::propagateImpulse(data.getLinkData(linkID0), data.getJointData(linkID0), Z0);
			Z[links[linkID0].parent] = Z0;
			stack[i0++] = linkID0;
		}

		for (i1 = i0; linkID1 != common; linkID1 = links[linkID1].parent)
		{
			Z1 = FeatherstoneArticulation::propagateImpulse(data.getLinkData(linkID1), data.getJointData(linkID1), Z1);
			Z[links[linkID1].parent] = Z1;
			stack[i1++] = linkID1;
		}

		Cm::SpatialVectorF ZZ = Z0 + Z1;
		Z[common] = ZZ;
		for (ic = i1; common; common = links[common].parent)
		{
			Z[links[common].parent] = FeatherstoneArticulation::propagateImpulse(data.getLinkData(common), data.getJointData(common), Z[common]);
			stack[ic++] = common;
		}

		if (data.getCore()->flags & PxArticulationFlag::eFIX_BASE)
		{
			Z[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
		}

		//SpatialMatrix inverseArticulatedInertia = data.getLinkData(0).spatialArticulatedInertia.getInverse();
		const SpatialMatrix& inverseArticulatedInertia = data.getBaseInvSpatialArticulatedInertia();
		Cm::SpatialVectorF v = inverseArticulatedInertia * (-Z[0]);

		for (PxU32 index = ic; (index--) > i1;)
			v = FeatherstoneArticulation::propagateVelocityTestImpulse(data.getLinkData(stack[index]), data.getJointData(stack[index]), Z[stack[index]], v);
	
		Cm::SpatialVectorF dv1 = v;
		for (PxU32 index = i1; (index--) > i0;)
			dv1 = FeatherstoneArticulation::propagateVelocityTestImpulse(data.getLinkData(stack[index]), data.getJointData(stack[index]), Z[stack[index]], dv1);

		Cm::SpatialVectorF dv0= v;
		for (PxU32 index = i0; (index--) > 0;)
			dv0 = FeatherstoneArticulation::propagateVelocityTestImpulse(data.getLinkData(stack[index]), data.getJointData(stack[index]), Z[stack[index]], dv0);

		deltaV0.linear = transform0.rotate(dv0.bottom);
		deltaV0.angular = transform0.rotate(dv0.top);

		deltaV1.linear = transform1.rotate(dv1.bottom);
		deltaV1.angular = transform1.rotate(dv1.top);
	}

	void FeatherstoneArticulation::getImpulseSelfResponse(ArticulationLink* links,
		const bool fixBase,
		Cm::SpatialVectorF* Z,
		const ArticulationData& data,
		PxU32 linkID0,
		const Cm::SpatialVectorV& impulse0,
		Cm::SpatialVectorV& deltaV0,
		PxU32 linkID1,
		const Cm::SpatialVectorV& impulse1,
		Cm::SpatialVectorV& deltaV1)
	{
#if 0
		ArticulationLink& pLink = links[linkID0];
		ArticulationLink& cLink = links[linkID1];

		PxTransform& pBody2World = pLink.bodyCore->body2World;
		PxTransform& cBody2World = cLink.bodyCore->body2World;

		Cm::SpatialVectorF pImpulse = Cm::SpatialVectorF(pBody2World.rotateInv(reinterpret_cast<const PxVec3&>(impulse0.linear)), pBody2World.rotateInv(reinterpret_cast<const PxVec3&>(impulse0.angular)));
		Cm::SpatialVectorF cImpulse = Cm::SpatialVectorF(cBody2World.rotateInv(reinterpret_cast<const PxVec3&>(impulse1.linear)), cBody2World.rotateInv(reinterpret_cast<const PxVec3&>(impulse1.angular)));

		Cm::SpatialVectorF resp0 = FeatherstoneArticulation::getImpulseResponse(linkID0, fixBase, data, Z, pImpulse).rotate(pBody2World);
		Cm::SpatialVectorF resp1 = FeatherstoneArticulation::getImpulseResponse(linkID1, fixBase, data, Z, cImpulse).rotate(cBody2World);

		deltaV0 = Cm::SpatialVector(resp0.bottom, resp0.top);
		deltaV1 = Cm::SpatialVector(resp1.bottom, resp1.top);
#else
		
		ArticulationLink& link = links[linkID1];
		ArticulationLinkData& linkDatum = data.getLinkData(linkID1);
		ArticulationJointCoreData& jointDatum = data.getJointData(linkID1);
		if (link.parent == linkID0)
		{
			PX_ASSERT(linkID0 == link.parent);
			PX_ASSERT(linkID0 < linkID1);

			ArticulationLink& pLink = links[linkID0];
			//impulse is in world space
			Cm::SpatialVector imp1;
			V3StoreU(impulse1.angular, imp1.angular);
			V3StoreU(impulse1.linear, imp1.linear);

			Cm::SpatialVector imp0;
			V3StoreU(impulse0.angular, imp0.angular);
			V3StoreU(impulse0.linear, imp0.linear);

			PxTransform& pBody2World = pLink.bodyCore->body2World;

			Cm::SpatialVectorF pImpulse = Cm::SpatialVectorF(pBody2World.rotateInv(imp0.linear), pBody2World.rotateInv(imp0.angular));

			PX_ASSERT(linkID0 == link.parent);

			PxTransform& body2World = link.bodyCore->body2World;

			//initialize child link spatial zero acceleration impulse
			Cm::SpatialVectorF Z1 = Cm::SpatialVectorF(-body2World.rotateInv(imp1.linear), -body2World.rotateInv(imp1.angular));
			//this calculate parent link spatial zero acceleration impulse
			Cm::SpatialVectorF Z0 = FeatherstoneArticulation::propagateImpulse(linkDatum, jointDatum, Z1);

			//in parent space
			const Cm::SpatialVectorF impulseDif = pImpulse - Z0;

			Cm::SpatialVectorF delV0(PxVec3(0.f), PxVec3(0.f));
			Cm::SpatialVectorF delV1(PxVec3(0.f), PxVec3(0.f));

			//calculate velocity change start from the parent link to the root
			delV0 = FeatherstoneArticulation::getImpulseResponse(linkID0, fixBase, data, Z, impulseDif);

			//calculate velocity change for child link
			delV1 = FeatherstoneArticulation::propagateVelocityTestImpulse(linkDatum, jointDatum, Z1, delV0);

			//translate delV0 and delV1 into world space again
			const PxVec3 lin0 = pBody2World.rotate(delV0.bottom);
			const PxVec3 ang0 = pBody2World.rotate(delV0.top);
			const PxVec3 lin1 = body2World.rotate(delV1.bottom);
			const PxVec3 ang1 = body2World.rotate(delV1.top);

			deltaV0.linear = V3LoadU(lin0);
			deltaV0.angular = V3LoadU(ang0);
			deltaV1.linear = V3LoadU(lin1);
			deltaV1.angular = V3LoadU(ang1);
		}
		else
		{
			getImpulseResponseSlow(links, data, linkID0, reinterpret_cast<const Cm::SpatialVector&>(impulse0), 
				reinterpret_cast<Cm::SpatialVector&>(deltaV0), linkID1, 
				reinterpret_cast<const Cm::SpatialVector&>(impulse1), reinterpret_cast<Cm::SpatialVector&>(deltaV1));
		}
#endif
	}

	void FeatherstoneArticulation::createHardLimit(
		ArticulationLink* links,
		const bool fixBase,
		Cm::SpatialVectorF* Z,
		ArticulationData& data,
		PxU32 linkIndex,
		SolverConstraint1DExt& s,
		const PxVec3& axis,
		PxReal err,
		PxReal recipDt)
	{
		init(s, PxVec3(0), PxVec3(0), axis, axis, 0, PX_MAX_F32);

		FeatherstoneArticulation::getImpulseSelfResponse(links, fixBase, Z, data,
			links[linkIndex].parent, Cm::SpatialVector(PxVec3(0), axis), s.deltaVA,
			linkIndex, Cm::SpatialVector(PxVec3(0), -axis), s.deltaVB);

		const PxReal unitResponse = axis.dot(reinterpret_cast<PxVec3&>(s.deltaVA.angular)) - axis.dot(reinterpret_cast<PxVec3&>(s.deltaVB.angular));
		if (unitResponse<0.0f)
			Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Warning: articulation ill-conditioned or under severe stress, joint limit ignored");

		const PxReal recipResponse = unitResponse> DY_ARTICULATION_MIN_RESPONSE ? 1.0f / unitResponse : 0.0f;

		s.constant = recipResponse * -err * recipDt;
		s.unbiasedConstant = err>0.0f ? s.constant : 0.0f;
		s.velMultiplier = -recipResponse;
		s.impulseMultiplier = 1.0f;
	}

	void FeatherstoneArticulation::createHardLimits(
		SolverConstraint1DExt& s0,
		SolverConstraint1DExt& s1,
		const PxVec3& axis,
		PxReal err0,
		PxReal err1,
		PxReal recipDt,
		const Cm::SpatialVectorV& deltaVA,
		const Cm::SpatialVectorV& deltaVB,
		PxReal recipResponse)
	{
		init(s0, PxVec3(0), PxVec3(0), axis, axis, 0, PX_MAX_F32);
		init(s1, PxVec3(0), PxVec3(0), -axis, -axis, 0, PX_MAX_F32);

		s0.deltaVA = deltaVA;
		s0.deltaVB = deltaVB;
		s1.deltaVA = -deltaVA;
		s1.deltaVB = -deltaVB;

		s0.constant = recipResponse * -err0 * recipDt;
		s0.unbiasedConstant = err0>0.0f ? s0.constant : 0.0f;
		s0.velMultiplier = -recipResponse;
		s0.impulseMultiplier = 1.0f;

		s1.constant = recipResponse * -err1 * recipDt;
		s1.unbiasedConstant = err1>0.0f ? s1.constant : 0.0f;
		s1.velMultiplier = -recipResponse;
		s1.impulseMultiplier = 1.0f;
	}

	void FeatherstoneArticulation::createTangentialSpring(
		ArticulationLink* links,
		const bool fixBase,
		Cm::SpatialVectorF* Z,
		ArticulationData& data,
		PxU32 linkIndex,
		SolverConstraint1DExt& s,
		const PxVec3& axis,
		PxReal stiffness,
		PxReal damping,
		PxReal dt)
	{
		init(s, PxVec3(0), PxVec3(0), axis, axis, -PX_MAX_F32, PX_MAX_F32);

		Cm::SpatialVector axis6(PxVec3(0), axis);
		PxU32 parent = links[linkIndex].parent;
		FeatherstoneArticulation::getImpulseSelfResponse(links, fixBase, Z, data, parent, axis6, s.deltaVA, linkIndex, -axis6, s.deltaVB);

		const PxReal unitResponse = axis.dot(reinterpret_cast<PxVec3&>(s.deltaVA.angular)) - axis.dot(reinterpret_cast<PxVec3&>(s.deltaVB.angular));
		if (unitResponse<0.0f)
			Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Warning: articulation ill-conditioned or under severe stress, tangential spring ignored");
		const PxReal recipResponse = unitResponse> DY_ARTICULATION_MIN_RESPONSE ? 1.0f / unitResponse : 0.0f;

		// this is a specialization of the spring code in setSolverConstants() for acceleration springs.
		// general case is  b = dt * (c.mods.spring.damping * c.velocityTarget - c.mods.spring.stiffness * geomError);
		// but geomError and velocityTarget are both zero

		const PxReal a = dt * dt * stiffness + dt * damping;
		const PxReal x = 1.0f / (1.0f + a);
		s.constant = s.unbiasedConstant = 0.0f;
		s.velMultiplier = -x * recipResponse * a;
		s.impulseMultiplier = 1.0f - x;
	}

	void createDriveOrLimit(Dy::SolverConstraint1DExt& c, PxReal bias, const PxReal minImpulse, const PxReal maxImpulse,
		bool keepBias, bool isLimit, const Cm::SpatialVectorV& deltaVA, const Cm::SpatialVectorV& deltaVB, const PxReal recipResponse)
	{
		c.deltaVA = deltaVA;
		c.deltaVB = deltaVB;

		c.minImpulse = minImpulse;
		c.maxImpulse = maxImpulse;

		c.velMultiplier = -recipResponse;
		c.impulseMultiplier = 1.0f;

		{
			// see usage of 'for internal use' in preprocessRows()					
			c.constant = recipResponse * (-bias);
			if (!keepBias && (!isLimit || bias < 0.f))
				c.unbiasedConstant = 0.f;
			else
				c.unbiasedConstant = c.constant;
		}
		c.appliedForce = 0.f;
		c.flags = 0;
		c.ang0Writeback = c.ang0;
	}

	void createSpringDrive(Dy::SolverConstraint1DExt& c, PxReal error, PxReal targetVelocity, PxReal maxImpulse,
		PxReal stiffness, PxReal damping, PxReal dt, const Cm::SpatialVectorV& deltaVA, const Cm::SpatialVectorV& deltaVB,
		const PxReal unitResponse)
	{
		c.deltaVA = deltaVA;
		c.deltaVB = deltaVB;
		
		PxReal a = dt * dt * stiffness + dt * damping;
		PxReal b = dt * (damping * targetVelocity - stiffness * error);

		PxReal x = 1.0f / (1.0f + a*unitResponse);
		c.constant = c.unbiasedConstant = x * b;
		c.velMultiplier = -x*a;
		c.impulseMultiplier = 1.0f - x;

		c.minImpulse = -maxImpulse;
		c.maxImpulse = maxImpulse;

		c.appliedForce = 0.f;
		c.flags = 0;
		c.ang0Writeback = c.ang0;
	}

	void createSpringDrive(Dy::SolverConstraint1DExtStep& c, PxReal error, PxReal targetVelocity, PxReal maxImpulse,
		PxReal stiffness, PxReal damping, PxReal dt, PxReal totalDt, const Cm::SpatialVectorV& deltaVA, const Cm::SpatialVectorV& deltaVB,
		PxReal unitResponse)
	{
		PX_UNUSED(dt);

		c.deltaVA = deltaVA;
		c.deltaVB = deltaVB;

		PxReal a2 = totalDt *  (totalDt*stiffness + damping);
		PxReal b = totalDt * (damping * targetVelocity - stiffness * error);

		PxReal x = 1.0f / (1.0f + a2*unitResponse);

		c.velTarget = x * b * unitResponse;
		c.velMultiplier = -x*a2 * unitResponse;
		c.biasScale = 0.f;//c.velMultiplier / dt;
		c.impulseMultiplier = 1.0f -x; //KS - add back in -x term?

		c.minImpulse = -maxImpulse;
		c.maxImpulse = maxImpulse;
		c.recipResponse = unitResponse > DY_ARTICULATION_MIN_RESPONSE ? 1.f / unitResponse : 0.f;
		c.error = 0.f;
		c.maxBias = 100.f;
		c.angularErrorScale = 1.f;

		c.appliedForce = 0.f;
		c.flags = 0;
	}

	void createDriveOrLimit(Dy::SolverConstraint1DExtStep& c, PxReal error, const PxReal minImpulse, const PxReal maxImpulse,
		bool keepBias, PxReal maxBias, const PxReal velTarget, const PxReal biasCoefficient, bool isLimit,
		const Cm::SpatialVectorV& deltaVA, const Cm::SpatialVectorV& deltaVB, PxReal recipResponse)
	{
		c.deltaVA = deltaVA;
		c.deltaVB = deltaVB;

		c.minImpulse = minImpulse;
		c.maxImpulse = maxImpulse;

		c.velMultiplier = -1.f;
		c.impulseMultiplier = 1.0f;

		c.error = error;
		c.velTarget = velTarget;
		c.biasScale = -biasCoefficient;
		c.recipResponse = recipResponse;
		c.maxBias = maxBias;
		c.appliedForce = 0.f;

		c.flags = PxU32(keepBias || (isLimit && error > 0.f) ? DY_SC_FLAG_KEEP_BIAS | DY_SC_FLAG_INEQUALITY : 0);
		c.angularErrorScale = 0.f;
	}

	PX_INLINE void computeJacobianAxes(PxVec3 row[3], const PxQuat& qa, const PxQuat& qb)
	{
		// Compute jacobian matrix for (qa* qb)  [[* means conjugate in this expr]]
		// d/dt (qa* qb) = 1/2 L(qa*) R(qb) (omega_b - omega_a)
		// result is L(qa*) R(qb), where L(q) and R(q) are left/right q multiply matrix

		const PxReal wa = qa.w, wb = qb.w;
		const PxVec3 va(qa.x, qa.y, qa.z), vb(qb.x, qb.y, qb.z);

		const PxVec3 c = vb*wa + va*wb;
		const PxReal d0 = wa*wb;
		const PxReal d1 = va.dot(vb);
		const PxReal d = d0 - d1;

		row[0] = (va * vb.x + vb * va.x + PxVec3(d, c.z, -c.y)) * 0.5f;
		row[1] = (va * vb.y + vb * va.y + PxVec3(-c.z, d, c.x)) * 0.5f;
		row[2] = (va * vb.z + vb * va.z + PxVec3(c.y, -c.x, d)) * 0.5f;

		if ((d0 + d1) != 0.0f)  // check if relative rotation is 180 degrees which can lead to singular matrix
			return;
		else
		{
			row[0].x += PX_EPS_F32;
			row[1].y += PX_EPS_F32;
			row[2].z += PX_EPS_F32;
		}
	}

	void FeatherstoneArticulation::setupInternalConstraints(
		ArticulationLink* links,
		const PxU32 linkCount,
		const bool fixBase,
		ArticulationData& data,
		Cm::SpatialVectorF* Z,
		PxReal dt,
		PxReal invDt,
		PxReal erp,
		bool isTGSSolver)
	{
		PX_PROFILE_ZONE("Articulations:setupSolverConstraintsInternal", 0);

		const PxConstraintInvMassScale ims(1.0f, 1.0f, 1.0f, 1.0f);

		Cm::SpatialVectorF* jointTransmittedForce = data.getTransmittedForces();
		PxReal* jointPositions = data.getJointPositions();

		data.mInternalConstraints.forceSize_Unsafe(0);
		data.mInternalConstraints.resizeUninitialized(data.getDofs());

		data.mInternalLockedAxes.forceSize_Unsafe(0);
		data.mInternalLockedAxes.resizeUninitialized(data.getLocks());

		ArticulationInternalConstraint* constraints = data.mInternalConstraints.begin();
		ArticulationInternalLockedAxis* locks = data.mInternalLockedAxes.begin();

		for (PxU16 linkID = 1; linkID < linkCount; linkID++)
		{
			const ArticulationLink& link = links[linkID];

			ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
			const PxReal* jPosition = &jointPositions[jointDatum.jointOffset];
			PX_UNUSED(jPosition);

			const ArticulationLink& pLink = links[link.parent];

			const ArticulationJointCore& j = *link.inboundJoint;

			//const bool jointDrive = (j.driveType != PxArticulationJointDriveType::eNONE);

			const bool hasFriction = j.frictionCoefficient > 0.f;

			const PxReal fCoefficient = j.frictionCoefficient * dt;
			const PxReal transmissionForce = jointTransmittedForce[linkID].magnitude() * fCoefficient;
			PX_UNUSED(transmissionForce);

			const PxU32 limitedRows = 2u * jointDatum.limitedAxes;
			const PxU32 lockedRows = jointDatum.lockedAxes;

			PxU8 driveRows = 0;

			for (PxU32 i = 0; i < PxArticulationAxis::eCOUNT; ++i)
			{
				if (j.drives[i].maxForce > 0.f && (j.drives[i].stiffness > 0.f || j.drives[i].damping > 0.f))
					driveRows++;
			}

			const PxU8 frictionRows = hasFriction ? jointDatum.dof : PxU8(0);

			const PxU8 constraintCount = PxU8(limitedRows + driveRows + frictionRows + lockedRows);
			if (!constraintCount)
			{
				//Skip these constraints...
				//constraints += jointDatum.dof;
				jointDatum.dofInternalConstraintMask = 0;
				continue;
			}

			const PxTransform cA2w = pLink.bodyCore->body2World.transform(j.parentPose);
			const PxTransform cB2w = link.bodyCore->body2World.transform(j.childPose);

			PxU32 dofId = 0;

			PxU8 dofMask = 0;
			for (PxU32 i = 0; i < PxArticulationAxis::eX; ++i)
			{
				if (j.motion[i] != PxArticulationMotion::eLOCKED)
				{
					const bool hasDrive = (j.motion[i] != PxArticulationMotion::eLOCKED && j.drives[i].maxForce > 0.f && (j.drives[i].stiffness > 0.f || j.drives[i].damping > 0.f));

					if (j.motion[i] == PxArticulationMotion::eLIMITED || hasDrive || frictionRows)
					{
						dofMask |= (1 << dofId);
						//Impulse response vector and axes are common for all constraints on this axis besides locked axis!!!
						const PxVec3 axis = link.bodyCore->body2World.rotate(jointDatum.motionMatrix[dofId].top);

						Cm::SpatialVectorV deltaVA, deltaVB;
						FeatherstoneArticulation::getImpulseSelfResponse(links, fixBase, Z, data,
							links[linkID].parent, Cm::SpatialVector(PxVec3(0), axis), deltaVA,
							linkID, Cm::SpatialVector(PxVec3(0), -axis), deltaVB);

						//Now add in friction rows, then limit rows, then drives...

						/*PxReal unitResponse = unsimdRef(deltaVA).angular.dot(axis) - unsimdRef(deltaVB).angular.dot(axis);
						PxReal totalMag = (unsimdRef(deltaVA) - unsimdRef(deltaVB)).magnitude();

						PxReal ratio = unitResponse / totalMag;
						if (ratio < 1e-4f)
							unitResponse = 0.f;*/

						const Cm::SpatialVector& deltaV0 = unsimdRef(deltaVA);
						const Cm::SpatialVector& deltaV1 = unsimdRef(deltaVB);

						const PxReal r0 = deltaV0.angular.dot(axis);
						const PxReal r1 = deltaV1.angular.dot(axis);

						const PxReal unitResponse = r0 - r1;

						//Cm::SpatialVector rem0(deltaV0.linear, deltaV0.angular - (axis * r0));
						//Cm::SpatialVector rem1(deltaV1.linear, deltaV1.angular - (axis*r1));

						//PxReal rem = (rem0 - rem1).magnitude();

						///*if ((unitResponse / rem) < DY_ARTICULATION_BAD_RESPONSE)
						//	unitResponse = 0.f;*/

						//unitResponse += rem;

						const PxReal recipResponse = unitResponse> DY_ARTICULATION_MIN_RESPONSE ? 1.0f / unitResponse : 0.0f;

						//constraints->unitResponse = unitResponse;
						constraints->recipResponse = recipResponse;
						constraints->response = unitResponse;
						constraints->row0 = Cm::SpatialVectorF(PxVec3(0), axis);
						constraints->row1 = Cm::SpatialVectorF(PxVec3(0), axis);
						constraints->deltaVA.top = unsimdRef(deltaVA).angular;
						constraints->deltaVA.bottom = unsimdRef(deltaVA).linear;
						constraints->deltaVB.top = unsimdRef(deltaVB).angular;
						constraints->deltaVB.bottom = unsimdRef(deltaVB).linear;
						constraints->erp = erp;
						constraints->isLinearConstraint = false;

						if (j.motion[i] == PxArticulationMotion::eLIMITED)
						{
							constraints->lowLimit = j.limits[i].low;
							constraints->highLimit = j.limits[i].high;
						}
						else
						{
							constraints->lowLimit = -PX_MAX_F32;
							constraints->highLimit = PX_MAX_F32;
						}

						constraints->lowImpulse = 0.f;
						constraints->highImpulse = 0.f;

						constraints->frictionForce = 0.f;
						constraints->maxFrictionForce = hasFriction ? transmissionForce/dt : 0.f;
						constraints->frictionForceCoefficient = isTGSSolver ? 0.f : 1.f;

						if (hasDrive)
						{
							const PxReal targetVelocity = jointDatum.targetJointVelocity[dofId];
							PxReal targetPos = jointDatum.targetJointPosition[dofId];

							//KS - clamp drive target within limits - no point in having 2 parts fight against each-other
							if (j.motion[i] == PxArticulationMotion::eLIMITED)
								targetPos = PxClamp(targetPos, j.limits[i].low, j.limits[i].high);
							
							const PxReal a = dt * dt * j.drives[i].stiffness + dt * j.drives[i].damping;
							const PxReal b = dt * (j.drives[i].damping * targetVelocity /*+ j.drives[i].stiffness * (targetPos - jointPos)*/);
							PxReal x = 0.f;

							if (!j.drives[i].isAcceleration)
							{
								x = unitResponse > 0.f ? 1.0f / (1.0f + a*unitResponse) : 0.f;
								constraints->driveTargetVel = x * b;
								constraints->driveBiasCoefficient = x*j.drives[i].stiffness*dt;
								constraints->driveVelMultiplier = -x*a;
							}
							else
							{
								x = 1.0f / (1.0f + a);
								constraints->driveTargetVel = x * b*recipResponse;
								constraints->driveBiasCoefficient = x*j.drives[i].stiffness*dt*recipResponse;
								constraints->driveVelMultiplier = -x*a*recipResponse;
							}
					 
							constraints->driveTarget = targetPos;
							constraints->driveImpulseMultiplier = isTGSSolver ? 1.f : 1.0f - x;
							constraints->maxDriveForce = j.drives[i].maxForce;// *dt;
							constraints->driveForce = 0.f;
						}
						else
						{
							constraints->driveTargetVel = 0.f;
							constraints->driveTarget = 0.f;
							constraints->driveBiasCoefficient = 0.f;
							constraints->driveVelMultiplier = 0.f;
							constraints->driveImpulseMultiplier = 0.f;
							constraints->maxDriveForce = 0.f;
							constraints->driveForce = 0.f;
						}
						constraints++;
					}
					dofId++;
				}
			}

			for (PxU32 i = PxArticulationAxis::eX; i < PxArticulationAxis::eCOUNT; ++i)
			{
				if (j.motion[i] != PxArticulationMotion::eLOCKED)
				{
					const bool hasDrive = (j.motion[i] != PxArticulationMotion::eLOCKED && j.drives[i].maxForce > 0.f && (j.drives[i].stiffness > 0.f || j.drives[i].damping > 0.f));

					if (j.motion[i] == PxArticulationMotion::eLIMITED || hasDrive || frictionRows)
					{

						dofMask |= (1 << dofId);
						//Impulse response vector and axes are common for all constraints on this axis besides locked axis!!!
						const PxVec3 axis = link.bodyCore->body2World.rotate(jointDatum.motionMatrix[dofId].bottom);
						const PxVec3 ang0 = (cA2w.p - pLink.bodyCore->body2World.p).cross(axis);
						const PxVec3 ang1 = (cB2w.p - link.bodyCore->body2World.p).cross(axis);

						Cm::SpatialVectorV deltaVA, deltaVB;
						FeatherstoneArticulation::getImpulseSelfResponse(links, fixBase, Z, data,
							links[linkID].parent, Cm::SpatialVector(axis, ang0), deltaVA,
							linkID, Cm::SpatialVector(-axis, -ang1), deltaVB);

						//Now add in friction rows, then limit rows, then drives...

						const Cm::SpatialVector& deltaV0 = unsimdRef(deltaVA);
						const Cm::SpatialVector& deltaV1 = unsimdRef(deltaVB);

						const PxReal r0 = deltaV0.linear.dot(axis) + deltaV0.angular.dot(ang0);
						const PxReal r1 = deltaV1.linear.dot(axis) +
							deltaV1.angular.dot(ang1);

						const PxReal unitResponse = r0 - r1;

						//Cm::SpatialVector rem0(deltaV0.linear - axis * r0, deltaV0.angular - (ang0 * r0));
						//Cm::SpatialVector rem1(deltaV1.linear - axis * r1, deltaV1.angular - (ang1*r1));

						//PxReal rem = (rem0 - rem1).magnitude();

						////unitResponse += rem;

						///*if ((unitResponse / rem) < DY_ARTICULATION_BAD_RESPONSE)
						//	unitResponse = 0.f;*/

						//unitResponse += rem;

						/*PxReal unitResponse = unsimdRef(deltaVA).linear.dot(axis) + unsimdRef(deltaVA).angular.dot(ang0) - unsimdRef(deltaVB).linear.dot(axis) -
							unsimdRef(deltaVB).angular.dot(ang1);

						PxReal totalMag = (unsimdRef(deltaVA) - unsimdRef(deltaVB)).magnitude();

						PxReal ratio = unitResponse / totalMag;
						if (ratio < 1e-4f)
							unitResponse = 0.f;*/

						const PxReal recipResponse = unitResponse> DY_ARTICULATION_MIN_RESPONSE ? 1.0f / unitResponse : 0.0f;

						constraints->response = unitResponse;
						constraints->recipResponse = recipResponse;
						constraints->row0 = Cm::SpatialVectorF(axis, ang0);
						constraints->row1 = Cm::SpatialVectorF(axis, ang1);
						constraints->deltaVA.top = unsimdRef(deltaVA).angular;
						constraints->deltaVA.bottom = unsimdRef(deltaVA).linear;
						constraints->deltaVB.top = unsimdRef(deltaVB).angular;
						constraints->deltaVB.bottom = unsimdRef(deltaVB).linear;
						constraints->erp = erp;
						constraints->isLinearConstraint = true;
						
						constraints->lowImpulse = 0.f;
						constraints->highImpulse = 0.f;

						constraints->frictionForce = 0.f;
						constraints->maxFrictionForce = hasFriction ? transmissionForce/dt : 0.f;

						constraints->frictionForceCoefficient = isTGSSolver ? 0.f : 1.f;

						if (j.motion[i] == PxArticulationMotion::eLIMITED)
						{
							constraints->lowLimit = j.limits[i].low;
							constraints->highLimit = j.limits[i].high;
						}
						else
						{
							constraints->lowLimit = -PX_MAX_F32;
							constraints->highLimit = PX_MAX_F32;
						}

						if (hasDrive)
						{
							const PxReal targetVelocity = -jointDatum.targetJointVelocity[dofId];
							PxReal targetPos = jointDatum.targetJointPosition[dofId];

							//KS - clamp drive target within limits - no point in having 2 parts fight against each-other
							if (j.motion[i] == PxArticulationMotion::eLIMITED)
								targetPos = PxClamp(targetPos, j.limits[i].low, j.limits[i].high);

							const PxReal a = dt * dt * j.drives[i].stiffness + dt * j.drives[i].damping;
							const PxReal b = dt * (j.drives[i].damping * targetVelocity /*+ j.drives[i].stiffness * (targetPos - jointPos)*/);
							PxReal x = 0.f;

							if (!j.drives[i].isAcceleration)
							{
								x = unitResponse > 0.f ? 1.0f / (1.0f + a*unitResponse) : 0.f;
								constraints->driveTargetVel = x * b;
								constraints->driveBiasCoefficient = x*j.drives[i].stiffness*dt;
								constraints->driveVelMultiplier = -x*a;
							}
							else
							{
								x = 1.0f / (1.0f + a);
								constraints->driveTargetVel = x * b*recipResponse;
								constraints->driveBiasCoefficient = x*j.drives[i].stiffness*dt*recipResponse;
								constraints->driveVelMultiplier = -x*a*recipResponse;
							}

							constraints->driveTarget = targetPos;
							constraints->driveImpulseMultiplier = isTGSSolver ? 1.f : 1.0f - x;
							constraints->maxDriveForce = j.drives[i].maxForce;// *dt;
							constraints->driveForce = 0.f;
						}
						else
						{
							constraints->driveTargetVel = 0.f;
							constraints->driveTarget = 0.f;
							constraints->driveBiasCoefficient = 0.f;
							constraints->driveVelMultiplier = 0.f;
							constraints->driveImpulseMultiplier = 0.f;
							constraints->maxDriveForce = 0.f;
							constraints->driveForce = 0.f;
						}
						constraints++;
					}
					dofId++;
				}
			}

			if (jointDatum.lockedAxes)
			{
				const PxQuat qB2qA = cA2w.q.getConjugate() * cB2w.q;
				PxVec3 row[3];
				computeJacobianAxes(row, cA2w.q, cB2w.q);

				PxReal err[] = { -qB2qA.x, -qB2qA.y, -qB2qA.z };

				for (PxU32 i = PxArticulationAxis::eTWIST; i <= PxArticulationAxis::eSWING2; ++i)
				{
					//Get axis, then add lock constraint...
					if (j.motion[i] == PxArticulationMotion::eLOCKED)
					{
						const PxVec3 axis = row[i];
						PxReal error = err[i];

						Cm::SpatialVectorV deltaVA, deltaVB;
						FeatherstoneArticulation::getImpulseSelfResponse(links, fixBase, Z, data,
							links[linkID].parent, Cm::SpatialVector(PxVec3(0), axis), deltaVA,
							linkID, Cm::SpatialVector(PxVec3(0), -axis), deltaVB);

						//Now add in friction rows, then limit rows, then drives...

						/*PxReal unitResponse = unsimdRef(deltaVA).angular.dot(axis) - unsimdRef(deltaVB).angular.dot(axis);

						PxReal totalMag = (unsimdRef(deltaVA) - unsimdRef(deltaVB)).magnitude();

						PxReal ratio = unitResponse / totalMag;
						if (ratio < 1e-4f)
							unitResponse = 0.f;*/

						const Cm::SpatialVector& deltaV0 = unsimdRef(deltaVA);
						const Cm::SpatialVector& deltaV1 = unsimdRef(deltaVB);

						const PxReal r0 = deltaV0.angular.dot(axis);
						const PxReal r1 = deltaV1.angular.dot(axis);

						const PxReal unitResponse = r0 - r1;

						//Cm::SpatialVector rem0(deltaV0.linear, deltaV0.angular - (axis * r0));
						//Cm::SpatialVector rem1(deltaV1.linear, deltaV1.angular - (axis * r1));

						//PxReal rem = (rem0 - rem1).magnitude();

						////
						///*if ((unitResponse / rem) < DY_ARTICULATION_BAD_RESPONSE)
						//	unitResponse = 0.f;*/

						//unitResponse += rem;

						const PxReal recipResponse = unitResponse > DY_ARTICULATION_MIN_RESPONSE ? 1.0f / unitResponse : 0.0f;

						locks->axis = axis;
						locks->deltaVA.top = unsimdRef(deltaVA).angular;
						locks->deltaVA.bottom = unsimdRef(deltaVA).linear;
						locks->deltaVB.top = unsimdRef(deltaVB).angular;
						locks->deltaVB.bottom = unsimdRef(deltaVB).linear;
						locks->recipResponse = recipResponse;
						locks->error = error;
						locks->biasScale = invDt*erp;//*0.7f;
						locks++;
					}
				}
			}

			jointDatum.dofInternalConstraintMask = dofMask;
		}
	}


#if 1

	PxU32 FeatherstoneArticulation::setupSolverConstraints(
		ArticulationLink* links,
		const PxU32 linkCount,
		const bool fixBase,
		ArticulationData& data,
		Cm::SpatialVectorF* Z,
		PxU32& acCount)
	{
		acCount = 0;

		setupInternalConstraints(links, linkCount, fixBase, data, Z, data.getDt(), 1.f / data.getDt(), 1.f, false);

		return 0;
	}

#else

	PxU32 FeatherstoneArticulation::setupSolverConstraints(
		PxConstraintAllocator& allocator,
		PxSolverConstraintDesc* constraintDesc,
		ArticulationLink* links,
		const PxU32 linkCount,
		const bool fixBase,
		ArticulationData& data,
		Cm::SpatialVectorF* Z,
		PxU32& acCount)
	{
		PX_PROFILE_ZONE("Articulations:setupSolverConstraints", 0);
		acCount = 0;

		const PxReal dt = data.getDt();
		PxU32 descCount = 0;
		const PxReal recipDt = 1.0f / dt;

		const PxConstraintInvMassScale ims(1.0f, 1.0f, 1.0f, 1.0f);

		Cm::SpatialVectorF* jointTransmittedForce = data.getTransmittedForces();
		PxReal* jointPositions = data.getJointPositions();

		for (PxU16 linkID = 1; linkID < linkCount; linkID++)
		{
			const ArticulationLink& link = links[linkID];
			
			const ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
			PxReal* jPosition = &jointPositions[jointDatum.jointOffset];
			PX_UNUSED(jPosition);

			const ArticulationLink& pLink = links[link.parent];

			const ArticulationJointCore& j = *link.inboundJoint;

			//const bool jointDrive = (j.driveType != PxArticulationJointDriveType::eNONE);
			
			bool hasFriction = j.frictionCoefficient > 0.f;

			const PxReal fCoefficient = j.frictionCoefficient * dt;
			const PxReal transmissionForce = jointTransmittedForce[linkID].magnitude() * fCoefficient;

			const PxU32 limitedRows = 2u * jointDatum.limitedAxes;

			PxU8 lockedAxes = jointDatum.lockedAxes;


			PxU8 driveRows = 0;

			for (PxU32 i = 0; i < PxArticulationAxis::eCOUNT; ++i)
			{
				if (j.motion[i] != PxArticulationMotion::eLOCKED && j.drives[i].maxForce > 0.f && (j.drives[i].stiffness > 0.f || j.drives[i].damping > 0.f))
					driveRows++;
			}

			PxU8 frictionRows = hasFriction ? jointDatum.dof : PxU8(0);

			const PxU8 constraintCount = PxU8(limitedRows + driveRows + frictionRows + lockedAxes);
			if (!constraintCount)
				continue;

			const PxTransform cA2w = pLink.bodyCore->body2World.transform(j.parentPose);
			const PxTransform cB2w = link.bodyCore->body2World.transform(j.childPose);

			PxSolverConstraintDesc& desc = constraintDesc[descCount++];

			desc.articulationA = data.getArticulation();
			desc.linkIndexA = Ps::to16(links[linkID].parent);
			desc.articulationALength = Ps::to16(data.getSolverDataSize());

			desc.articulationB = data.getArticulation();
			desc.linkIndexB = linkID;
			desc.articulationBLength = Ps::to16(data.getSolverDataSize());

			const PxU32 constraintLength = sizeof(SolverConstraint1DHeader) +
				sizeof(SolverConstraint1DExt) * constraintCount;

			PX_ASSERT(0 == (constraintLength & 0x0f));
			desc.constraintLengthOver16 = Ps::to16(constraintLength / 16);

			desc.constraint = allocator.reserveConstraintData(constraintLength + 16u);

			desc.writeBack = NULL;

			SolverConstraint1DHeader* header = reinterpret_cast<SolverConstraint1DHeader*>(desc.constraint);
			SolverConstraint1DExt* constraints = reinterpret_cast<SolverConstraint1DExt*>(desc.constraint + sizeof(SolverConstraint1DHeader));

			init(*header, constraintCount, true, ims);

			PxU32 cIndex = 0;


			PxU32 dofId = 0;

			for (PxU32 i = 0; i < PxArticulationAxis::eX; ++i)
			{
				if (j.motion[i] != PxArticulationMotion::eLOCKED)
				{
					bool hasDrive = (j.motion[i] != PxArticulationMotion::eLOCKED && j.drives[i].maxForce > 0.f && (j.drives[i].stiffness > 0.f || j.drives[i].damping > 0.f));

					if (j.motion[i] == PxArticulationMotion::eLIMITED || hasDrive || frictionRows)
					{
						//Impulse response vector and axes are common for all constraints on this axis besides locked axis!!!
						const PxVec3 axis = link.bodyCore->body2World.rotate(jointDatum.motionMatrix[dofId].top);

						Cm::SpatialVectorV deltaVA, deltaVB;
						FeatherstoneArticulation::getImpulseSelfResponse(links, fixBase, Z, data,
							links[linkID].parent, Cm::SpatialVector(PxVec3(0), axis), deltaVA,
							linkID, Cm::SpatialVector(PxVec3(0), -axis), deltaVB);

						//Now add in friction rows, then limit rows, then drives...

						PxReal unitResponse = unsimdRef(deltaVA).angular.dot(axis) - unsimdRef(deltaVB).angular.dot(axis);

						const PxReal recipResponse = unitResponse> DY_ARTICULATION_MIN_RESPONSE ? 1.0f / unitResponse : 0.0f;


						if (frictionRows)
						{
							Dy::SolverConstraint1DExt& frictionRow = constraints[cIndex];
							cIndex++;

							frictionRow.lin0 = PxVec3(0.f);
							frictionRow.lin1 = PxVec3(0.f);
							frictionRow.ang0 = axis;
							frictionRow.ang1 = axis;

							createDriveOrLimit(frictionRow, 0.f, -transmissionForce, transmissionForce, false, false,
								deltaVA, deltaVB, recipResponse);
						}

						if (j.motion[i] == PxArticulationMotion::eLIMITED)
						{
							const PxReal jointPos = data.mJointPosition[jointDatum.jointOffset + dofId];
							const PxReal lowLimit = j.limits[i].low;
							const PxReal highLimit = j.limits[i].high;

							createHardLimits(constraints[cIndex], constraints[cIndex + 1], -axis, jointPos - lowLimit, 
								highLimit - jointPos, recipDt, -deltaVA, -deltaVB, recipResponse);
							cIndex += 2;
						}

						if (hasDrive)
						{
							PxReal targetVelocity = jointDatum.targetJointVelocity[dofId];
							PxReal targetPos = jointDatum.targetJointPosition[dofId];
							const PxReal jointPos = -data.mJointPosition[jointDatum.jointOffset + dofId];

							Dy::SolverConstraint1DExt& c = constraints[cIndex++];

							c.lin0 = PxVec3(0);
							c.lin1 = PxVec3(0);
							c.ang0 = axis;
							c.ang1 = axis;

							createSpringDrive(c, jointPos + targetPos, -targetVelocity, j.drives[i].maxForce,
								j.drives[i].stiffness, j.drives[i].damping, dt, deltaVA, deltaVB, unitResponse);
						}



					}
					dofId++;
				}
				
			}

			for (PxU32 i = PxArticulationAxis::eX; i < PxArticulationAxis::eCOUNT; ++i)
			{
				if (j.motion[i] != PxArticulationMotion::eLOCKED)
				{
					bool hasDrive = (j.motion[i] != PxArticulationMotion::eLOCKED && j.drives[i].maxForce > 0.f && (j.drives[i].stiffness > 0.f || j.drives[i].damping > 0.f));

					if (j.motion[i] == PxArticulationMotion::eLIMITED || hasDrive || frictionRows)
					{


						//Impulse response vector and axes are common for all constraints on this axis besides locked axis!!!
						const PxVec3 axis = link.bodyCore->body2World.rotate(jointDatum.motionMatrix[dofId].bottom);
						const PxVec3 ang0 = (cA2w.p - pLink.bodyCore->body2World.p).cross(axis);
						const PxVec3 ang1 = (cB2w.p - pLink.bodyCore->body2World.p).cross(axis);

						Cm::SpatialVectorV deltaVA, deltaVB;
						FeatherstoneArticulation::getImpulseSelfResponse(links, fixBase, Z, data,
							links[linkID].parent, Cm::SpatialVector(axis, ang0), deltaVA,
							linkID, Cm::SpatialVector(-axis, -ang1), deltaVB);

						//Now add in friction rows, then limit rows, then drives...

						PxReal unitResponse = unsimdRef(deltaVA).linear.dot(axis) + unsimdRef(deltaVA).angular.dot(ang0) - unsimdRef(deltaVB).linear.dot(axis) -
							unsimdRef(deltaVB).angular.dot(ang1);

						const PxReal recipResponse = unitResponse> DY_ARTICULATION_MIN_RESPONSE ? 1.0f / unitResponse : 0.0f;


						if (frictionRows)
						{
							Dy::SolverConstraint1DExt& frictionRow = constraints[cIndex];
							cIndex++;

							frictionRow.lin0 = axis;
							frictionRow.lin1 = axis;
							frictionRow.ang0 = ang0;
							frictionRow.ang1 = ang1;

							createDriveOrLimit(frictionRow, 0.f, -transmissionForce, transmissionForce, false, false,
								deltaVA, deltaVB, recipResponse);
						}

						if (j.motion[i] == PxArticulationMotion::eLIMITED)
						{
							//Do linear limits...
							const PxReal jointPos = data.mJointPosition[jointDatum.jointOffset + dofId];
							const PxReal lowLimit = j.limits[i].low;
							const PxReal highLimit = j.limits[i].high;

							Dy::SolverConstraint1DExt* prismaticRows = &constraints[cIndex];
							cIndex += 2;

							prismaticRows[0].lin0 = -axis;
							prismaticRows[0].lin1 = -axis;
							prismaticRows[0].ang0 = -ang0;
							prismaticRows[0].ang1 = -ang1;
							prismaticRows[1].lin0 = axis;
							prismaticRows[1].lin1 = axis;
							prismaticRows[1].ang0 = ang0;
							prismaticRows[1].ang1 = ang1;

							createDriveOrLimit(prismaticRows[0], (jointPos - lowLimit)*recipDt, 0.f, PX_MAX_F32, false, true, -deltaVA, -deltaVB, recipResponse);
							createDriveOrLimit(prismaticRows[1], (highLimit - jointPos)*recipDt, 0.f, PX_MAX_F32, false, true, deltaVA, deltaVB, recipResponse);

						}

						if (hasDrive)
						{
							PxReal targetVelocity = jointDatum.targetJointVelocity[dofId];
							PxReal targetPos = jointDatum.targetJointPosition[dofId];
							const PxReal jointPos = -data.mJointPosition[jointDatum.jointOffset + dofId];

							Dy::SolverConstraint1DExt& c = constraints[cIndex++];

							c.lin0 = axis;
							c.lin1 = axis;
							c.ang0 = ang0;
							c.ang1 = ang1;

							createSpringDrive(c, jointPos + targetPos, -targetVelocity, j.drives[i].maxForce,
								j.drives[i].stiffness, j.drives[i].damping, dt, -deltaVA, -deltaVB, unitResponse);
						}

						
					}

					dofId++;
				}
			}

			if (lockedAxes)
			{
				const PxQuat qB2qA = cA2w.q.getConjugate() * cB2w.q;
				PxVec3 row[3];
				computeJacobianAxes(row, cA2w.q, cB2w.q);

				PxReal err[] = { -qB2qA.x, -qB2qA.y, -qB2qA.z };

				for (PxU32 i = PxArticulationAxis::eTWIST; i <= PxArticulationAxis::eSWING2; ++i)
				{
					//Get axis, then add lock constraint...
					if (j.motion[i] == PxArticulationMotion::eLOCKED)
					{
						const PxVec3 axis = row[i];
						PxReal error = err[i];
						Dy::SolverConstraint1DExt& lockRow = constraints[cIndex];
						cIndex++;

						//const PxVec3 axis = link.bodyCore->body2World.rotate(jointDatum.motionMatrix[i].top);

						lockRow.lin0 = PxVec3(0.f);
						lockRow.lin1 = PxVec3(0.f);
						lockRow.ang0 = axis;
						lockRow.ang1 = axis;

						Cm::SpatialVectorV deltaVA, deltaVB;
						FeatherstoneArticulation::getImpulseSelfResponse(links, fixBase, Z, data,
							links[linkID].parent, Cm::SpatialVector(PxVec3(0), axis), deltaVA,
							linkID, Cm::SpatialVector(PxVec3(0), -axis), deltaVB);

						//Now add in friction rows, then limit rows, then drives...

						PxReal unitResponse = unsimdRef(deltaVA).angular.dot(axis) - unsimdRef(deltaVB).angular.dot(axis);

						const PxReal recipResponse = unitResponse > DY_ARTICULATION_MIN_RESPONSE ? 1.0f / unitResponse : 0.0f;


						createDriveOrLimit(lockRow, error*recipDt, -PX_MAX_F32, PX_MAX_F32, false, false, deltaVA, deltaVB, recipResponse);
					}
				}
			}


			*(desc.constraint + getConstraintLength(desc)) = 0;

			PX_ASSERT(cIndex == constraintCount);
			acCount += constraintCount;
		}

		return descCount;
	}

#endif

#if 0

	static PxU32 getConstraintLength(const PxTGSSolverConstraintDesc& desc)
	{
		return PxU32(desc.constraintLengthOver16 << 4);
	}


	PxU32 FeatherstoneArticulation::setupSolverConstraintsTGS(const ArticulationSolverDesc& articDesc,
		PxcConstraintBlockStream& stream,
		PxTGSSolverConstraintDesc* constraintDesc,
		PxReal dt,
		PxReal invDt,
		PxReal totalDt,
		PxU32& acCount,
		PxsConstraintBlockManager& constraintBlockManager,
		Cm::SpatialVectorF* Z)
	{

		PX_PROFILE_ZONE("Articulations:setupSolverConstraintsTGS", 0);
		FeatherstoneArticulation* articulation = static_cast<FeatherstoneArticulation*>(articDesc.articulation);
		ArticulationData& data = articulation->mArticulationData;

		const PxU32 solverDataSize = articDesc.solverDataSize;

		PX_UNUSED(dt);
		acCount = 0;

		const bool fixBase = data.getCore()->flags & PxArticulationFlag::eFIX_BASE;

		const PxU32 linkCount = data.getLinkCount();
		ArticulationLink* links = data.getLinks();
		//ArticulationLinkData* linkData = data.getLinkData();
		ArticulationJointCoreData* jointData = data.getJointData();
		PxU32 descCount = 0;
		const PxReal recipDt = invDt;

		const PxConstraintInvMassScale ims(1.0f, 1.0f, 1.0f, 1.0f);

		
		Cm::SpatialVectorF* jointTransmittedForce = data.getTransmittedForces();

		//for (PxU16 linkID = PxU16(linkCount-1); linkID>=1; linkID--)
		for (PxU16 linkID = 1; linkID < linkCount; linkID++)
		{
			const ArticulationLink& link = links[linkID];
			//const ArticulationLinkData& linkDatum = linkData[linkID];
			const ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
			const ArticulationLink& pLink = links[link.parent];

			const ArticulationJointCore& j = static_cast<const ArticulationJointCore&>(*links[linkID].inboundJoint);

			//const bool jointDrive = (j.driveType != PxArticulationJointDriveType::eNONE);

			bool hasFriction = j.frictionCoefficient > 0.f;
			
			const PxReal fCoefficient = j.frictionCoefficient * dt;
			const PxReal transmissionForce = jointTransmittedForce[linkID].magnitude() * fCoefficient;

			PxU32 limitedRows = jointDatum.limitedAxes * 2u;

			PxU8 driveRows = 0;

			for (PxU32 i = 0; i < PxArticulationAxis::eCOUNT; ++i)
			{
				if (j.motion[i] != PxArticulationMotion::eLOCKED && j.drives[i].maxForce > 0.f && (j.drives[i].stiffness > 0.f || j.drives[i].damping > 0.f))
					driveRows++;
			}

			PxU8 frictionRows = hasFriction ? PxU8(jointData[linkID].dof) : PxU8(0);

			PxU32 lockedAxes = jointDatum.lockedAxes;


			const PxU8 constraintCount = PxU8(limitedRows + driveRows + frictionRows + lockedAxes);
			if (!constraintCount)
				continue;

			const PxTransform cA2w = pLink.bodyCore->body2World.transform(j.parentPose);
			const PxTransform cB2w = link.bodyCore->body2World.transform(j.childPose);

			//const PxTransform cB2cA = cA2w.transformInv(cB2w);

			PxTGSSolverConstraintDesc& desc = constraintDesc[descCount++];

			desc.articulationA = articulation;
			desc.linkIndexA = Ps::to16(links[linkID].parent);
			desc.articulationALength = Ps::to16(solverDataSize);

			desc.articulationB = articulation;
			desc.linkIndexB = linkID;
			desc.articulationBLength = Ps::to16(solverDataSize);

			const PxU32 constraintLength = sizeof(SolverConstraint1DHeaderStep) +
				sizeof(SolverConstraint1DExtStep) * constraintCount;

			PX_ASSERT(0 == (constraintLength & 0x0f));
			desc.constraintLengthOver16 = Ps::to16(constraintLength / 16);

			desc.constraint = stream.reserve(constraintLength + 16u, constraintBlockManager);

			desc.writeBack = NULL;

			SolverConstraint1DHeaderStep* header = reinterpret_cast<SolverConstraint1DHeaderStep*>(desc.constraint);
			SolverConstraint1DExtStep* constraints = reinterpret_cast<SolverConstraint1DExtStep*>(desc.constraint + sizeof(SolverConstraint1DHeaderStep));

			init(*header, constraintCount, true, 0.f, ims);

			header->rAWorld = cA2w.p - pLink.bodyCore->body2World.p;
			header->rBWorld = cB2w.p - link.bodyCore->body2World.p;
			header->rALocal = j.parentPose.p;
			header->rBLocal = j.childPose.p;

			PxU32 cIndex = 0;
			
			PxU32 dofId = 0;

			for (PxU32 i = 0; i < PxArticulationAxis::eX; ++i)
			{
				if (j.motion[i] != PxArticulationMotion::eLOCKED)
				{
					bool hasDrive = (j.motion[i] != PxArticulationMotion::eLOCKED && j.drives[i].maxForce > 0.f && (j.drives[i].stiffness > 0.f || j.drives[i].damping > 0.f));

					if (j.motion[i] == PxArticulationMotion::eLIMITED || hasDrive || frictionRows)
					{
						//Impulse response vector and axes are common for all constraints on this axis besides locked axis!!!
						const PxVec3 axis = link.bodyCore->body2World.rotate(jointDatum.motionMatrix[dofId].top);

						Cm::SpatialVectorV deltaVA, deltaVB;
						FeatherstoneArticulation::getImpulseSelfResponse(links, fixBase, Z, data,
							links[linkID].parent, Cm::SpatialVector(PxVec3(0), axis), deltaVA,
							linkID, Cm::SpatialVector(PxVec3(0), -axis), deltaVB);

						//Now add in friction rows, then limit rows, then drives...

						PxReal unitResponse = unsimdRef(deltaVA).angular.dot(axis) - unsimdRef(deltaVB).angular.dot(axis);

						const PxReal recipResponse = unitResponse > DY_ARTICULATION_MIN_RESPONSE ? 1.0f / unitResponse : 0.0f;


						if (frictionRows)
						{
							Dy::SolverConstraint1DExtStep& frictionRow = constraints[cIndex];
							cIndex++;

							frictionRow.lin0 = PxVec3(0.f);
							frictionRow.lin1 = PxVec3(0.f);
							frictionRow.ang0 = axis;
							frictionRow.ang1 = axis;


							createDriveOrLimit(frictionRow, 0.f, -transmissionForce, transmissionForce, false, 0.f, 0.f, 0.f, false,
								deltaVA, deltaVB, recipResponse);
						}

						if (j.motion[i] == PxArticulationMotion::eLIMITED)
						{
							const PxReal jointPos = data.mJointPosition[jointDatum.jointOffset + dofId];
							const PxReal lowLimit = j.limits[i].low;
							const PxReal highLimit = j.limits[i].high;

							articulation->createHardLimitTGS(constraints[cIndex++], -axis, jointPos - lowLimit, recipDt, -deltaVA, -deltaVB, recipResponse);
							articulation->createHardLimitTGS(constraints[cIndex++], axis, highLimit - jointPos, recipDt, deltaVA, deltaVB, recipResponse);
						}

						if (hasDrive)
						{
							PxReal targetVelocity = jointDatum.targetJointVelocity[dofId];
							PxReal targetPos = jointDatum.targetJointPosition[dofId];
							const PxReal jointPos = data.mJointPosition[jointDatum.jointOffset + dofId];

							Dy::SolverConstraint1DExtStep& c = constraints[cIndex++];

							c.lin0 = PxVec3(0);
							c.lin1 = PxVec3(0);
							c.ang0 = axis;
							c.ang1 = axis;

							createSpringDrive(c, targetPos - jointPos, -targetVelocity, j.drives[i].maxForce,
								j.drives[i].stiffness, j.drives[i].damping, dt, totalDt, deltaVA, deltaVB, unitResponse);
						}



					}
					dofId++;
				}
			}

			for (PxU32 i = PxArticulationAxis::eX; i < PxArticulationAxis::eCOUNT; ++i)
			{
				if (j.motion[i] != PxArticulationMotion::eLOCKED)
				{
					bool hasDrive = (j.motion[i] != PxArticulationMotion::eLOCKED && j.drives[i].maxForce > 0.f && (j.drives[i].stiffness > 0.f || j.drives[i].damping > 0.f));

					if (j.motion[i] == PxArticulationMotion::eLIMITED || hasDrive || frictionRows)
					{


						//Impulse response vector and axes are common for all constraints on this axis besides locked axis!!!
						const PxVec3 axis = link.bodyCore->body2World.rotate(jointDatum.motionMatrix[dofId].bottom);
						const PxVec3 ang0 = (cA2w.p - pLink.bodyCore->body2World.p).cross(axis);
						const PxVec3 ang1 = (cB2w.p - pLink.bodyCore->body2World.p).cross(axis);

						Cm::SpatialVectorV deltaVA, deltaVB;
						FeatherstoneArticulation::getImpulseSelfResponse(links, fixBase, Z, data,
							links[linkID].parent, Cm::SpatialVector(axis, ang0), deltaVA,
							linkID, Cm::SpatialVector(-axis, -ang1), deltaVB);

						//Now add in friction rows, then limit rows, then drives...

						PxReal unitResponse = unsimdRef(deltaVA).linear.dot(axis) + unsimdRef(deltaVA).angular.dot(ang0) - unsimdRef(deltaVB).linear.dot(axis) -
							unsimdRef(deltaVB).angular.dot(ang1);

						const PxReal recipResponse = unitResponse> DY_ARTICULATION_MIN_RESPONSE ? 1.0f / unitResponse : 0.0f;


						if (frictionRows)
						{
							Dy::SolverConstraint1DExtStep& frictionRow = constraints[cIndex];
							cIndex++;

							frictionRow.lin0 = axis;
							frictionRow.lin1 = axis;
							frictionRow.ang0 = ang0;
							frictionRow.ang1 = ang1;

							createDriveOrLimit(frictionRow, 0.f, -transmissionForce, transmissionForce, false, 0.f, 0.f, 0.f, false,
								deltaVA, deltaVB, recipResponse);
						}

						if (j.motion[i] == PxArticulationMotion::eLIMITED)
						{
							//Do linear limits...
							const PxReal jointPos = data.mJointPosition[jointDatum.jointOffset + dofId];
							const PxReal lowLimit = j.limits[i].low;
							const PxReal highLimit = j.limits[i].high;

							Dy::SolverConstraint1DExtStep* prismaticRows = &constraints[cIndex];
							cIndex += 2;

							prismaticRows[0].lin0 = -axis;
							prismaticRows[0].lin1 = -axis;
							prismaticRows[0].ang0 = -ang0;
							prismaticRows[0].ang1 = -ang1;
							prismaticRows[1].lin0 = axis;
							prismaticRows[1].lin1 = axis;
							prismaticRows[1].ang0 = ang0;
							prismaticRows[1].ang1 = ang1;

							createDriveOrLimit(prismaticRows[0], (jointPos - lowLimit), 0.f, PX_MAX_F32, false, 100.f, 0.f, 0.7f*recipDt, true, -deltaVA, -deltaVB, recipResponse);
							createDriveOrLimit(prismaticRows[1], (highLimit - jointPos), 0.f, PX_MAX_F32, false, 100.f, 0.f, 0.7f*recipDt, true, deltaVA, deltaVB, recipResponse);
						}

						if (hasDrive)
						{
							PxReal targetVelocity = jointDatum.targetJointVelocity[dofId];
							PxReal targetPos = jointDatum.targetJointPosition[dofId];
							const PxReal jointPos = data.mJointPosition[jointDatum.jointOffset + dofId];

							Dy::SolverConstraint1DExtStep& c = constraints[cIndex++];

							c.lin0 = axis;
							c.lin1 = axis;
							c.ang0 = ang0;
							c.ang1 = ang1;

							createSpringDrive(c, targetPos - jointPos, -targetVelocity, j.drives[i].maxForce,
								j.drives[i].stiffness, j.drives[i].damping, dt, totalDt, -deltaVA, -deltaVB, unitResponse);
						}


					}
					dofId++;
				}
			}	

			if (lockedAxes)
			{
				const PxQuat qB2qA = cA2w.q.getConjugate() * cB2w.q;
				PxVec3 row[3];
				computeJacobianAxes(row, cA2w.q, cB2w.q);

				PxReal err[] = { -qB2qA.x, -qB2qA.y, -qB2qA.z };

				for (PxU32 i = PxArticulationAxis::eTWIST; i <= PxArticulationAxis::eSWING2; ++i)
				{
					//Get axis, then add lock constraint...
					if (j.motion[i] == PxArticulationMotion::eLOCKED)
					{
						Dy::SolverConstraint1DExtStep& lockRow = constraints[cIndex];
						cIndex++;

						const PxVec3 axis = row[i];
						PxReal error = err[i];

						lockRow.lin0 = PxVec3(0.f);
						lockRow.lin1 = PxVec3(0.f);
						lockRow.ang0 = row[i];
						lockRow.ang1 = row[i];

						Cm::SpatialVectorV deltaVA, deltaVB;
						FeatherstoneArticulation::getImpulseSelfResponse(links, fixBase, Z, data,
							links[linkID].parent, Cm::SpatialVector(PxVec3(0), row[i]), deltaVA,
							linkID, Cm::SpatialVector(PxVec3(0), -row[i]), deltaVB);

						PxReal unitResponse = unsimdRef(deltaVA).angular.dot(row[i]) - unsimdRef(deltaVB).angular.dot(row[i]);

						const PxReal recipResponse = unitResponse> DY_ARTICULATION_MIN_RESPONSE ? 1.0f / unitResponse : 0.0f;

						createDriveOrLimit(lockRow, error, -PX_MAX_F32, PX_MAX_F32, false, 30.f, 0.f, 0.7f*invDt, false, deltaVA, deltaVB, recipResponse);
						lockRow.angularErrorScale = 1.f;
					}
				}
			}



			*(desc.constraint + getConstraintLength(desc)) = 0;

			PX_ASSERT(cIndex == constraintCount);
			acCount += constraintCount;
		}

		return descCount;
	}

#else

	PxU32 FeatherstoneArticulation::setupSolverConstraintsTGS(const ArticulationSolverDesc& articDesc,
	PxcConstraintBlockStream& /*stream*/,
	PxTGSSolverConstraintDesc* /*constraintDesc*/,
	PxReal dt,
	PxReal invDt,
	PxReal totalDt,
	PxU32& acCount,
	PxsConstraintBlockManager& /*constraintBlockManager*/,
	Cm::SpatialVectorF* Z)
	{
		PX_UNUSED(dt);
		PX_UNUSED(totalDt);
		acCount = 0;

		FeatherstoneArticulation* thisArtic = static_cast<FeatherstoneArticulation*>(articDesc.articulation);

		ArticulationLink* links = thisArtic->mArticulationData.getLinks();
		const PxU32 linkCount = thisArtic->mArticulationData.getLinkCount();
		const bool fixBase = thisArtic->mArticulationData.getCore()->flags & PxArticulationFlag::eFIX_BASE;

		thisArtic->setupInternalConstraints(links, linkCount, fixBase, thisArtic->mArticulationData, Z, totalDt, invDt, 0.7f, true);

		return 0;
	}

#endif

	void FeatherstoneArticulation::createHardLimitTGS(
		SolverConstraint1DExtStep& s,
		const PxVec3& axis,
		PxReal err,
		PxReal recipDt,
		const Cm::SpatialVectorV& deltaVA,
		const Cm::SpatialVectorV& deltaVB,
		PxReal recipResponse)
	{
		PxReal error = err;
		init(s, PxVec3(0), PxVec3(0), axis, axis, 0, PX_MAX_F32);

		s.deltaVA = deltaVA;
		s.deltaVB = deltaVB;
		
		s.error = error;
		s.biasScale = -recipDt*0.7f;
		s.maxBias = 30.f;
		s.velMultiplier = -1.f;
		s.recipResponse = recipResponse;
		s.impulseMultiplier = 1.0f;
		s.velTarget = 0.f;
		s.angularErrorScale = 1.f;
		s.flags |= DY_SC_FLAG_INEQUALITY;
		if (error > 0.f)
			s.flags |= DY_SC_FLAG_KEEP_BIAS;
	}

	void FeatherstoneArticulation::createTangentialSpringTGS(
		ArticulationLink* links,
		const bool fixBase,
		Cm::SpatialVectorF* Z,
		ArticulationData& data,
		PxU32 linkIndex,
		SolverConstraint1DExtStep& s,
		const PxVec3& axis,
		PxReal stiffness,
		PxReal damping,
		PxReal dt)
	{
		init(s, PxVec3(0), PxVec3(0), axis, axis, -PX_MAX_F32, PX_MAX_F32);

		Cm::SpatialVector axis6(PxVec3(0), axis);
		const PxU32 parent = links[linkIndex].parent;
		FeatherstoneArticulation::getImpulseSelfResponse(links, fixBase, Z, data, parent, axis6, s.deltaVA, linkIndex, -axis6, s.deltaVB);

		const PxReal unitResponse = axis.dot(reinterpret_cast<PxVec3&>(s.deltaVA.angular)) - axis.dot(reinterpret_cast<PxVec3&>(s.deltaVB.angular));
		if (unitResponse<0.0f)
			Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "Warning: articulation ill-conditioned or under severe stress, tangential spring ignored");
		const PxReal recipResponse = unitResponse> DY_ARTICULATION_MIN_RESPONSE ? 1.0f / unitResponse : 0.0f;

		// this is a specialization of the spring code in setSolverConstants() for acceleration springs.
		// general case is  b = dt * (c.mods.spring.damping * c.velocityTarget - c.mods.spring.stiffness * geomError);
		// but geomError and velocityTarget are both zero

		const PxReal a = dt * dt * stiffness + dt * damping;
		const PxReal x = 1.0f / (1.0f + a);
		s.error = 0.f;
		s.biasScale = 0.f;
		s.maxBias = 0.f;
		s.velMultiplier = -x * a;
		s.impulseMultiplier = 1.0f - x;
		s.velTarget = 0.f;
		s.recipResponse = recipResponse;
		s.angularErrorScale = 1.f;
	}

	void FeatherstoneArticulation::teleportLinks(ArticulationData& data)
	{
		jcalc(data);

		ArticulationLink* links = mArticulationData.getLinks();
	
		ArticulationJointCoreData* jointData = mArticulationData.getJointData();

		PxReal* jointPositions = data.getJointPositions();

		const PxU32 linkCount = mArticulationData.getLinkCount();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];
			
			ArticulationJointCoreData& jointDatum = jointData[linkID];

			ArticulationLink& pLink = links[link.parent];
			const PxTransform pBody2World = pLink.bodyCore->body2World;

			ArticulationJointCore* joint = link.inboundJoint;

			const PxReal* jPosition = &jointPositions[jointDatum.jointOffset];

			PxQuat newParentToChild;
			PxQuat newWorldQ;
			PxVec3 r;

			const PxVec3 childOffset = -joint->childPose.p;
			const PxVec3 parentOffset = joint->parentPose.p;

			switch (joint->jointType)
			{
			case PxArticulationJointType::ePRISMATIC:
			{
				newParentToChild = joint->relativeQuat;
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				const PxVec3& u = jointDatum.motionMatrix[0].bottom;

				r = e + d + u * jPosition[0];
				break;
			}
			case PxArticulationJointType::eREVOLUTE:
			{
				const PxVec3& u = jointDatum.motionMatrix[0].top;

				PxQuat jointRotation = PxQuat(-jPosition[0], u);
				if (jointRotation.w < 0)	//shortest angle.
					jointRotation = -jointRotation;

				newParentToChild = (jointRotation * joint->relativeQuat).getNormalized();

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
				r = e + d;

				PX_ASSERT(r.isFinite());

				break;
			}
			case PxArticulationJointType::eSPHERICAL:
			{
				PxQuat jointRotation(PxIdentity);
				for (PxU32 d = jointDatum.dof; d > 0; --d)
				{
					PxQuat deltaRot(jPosition[d - 1], jointDatum.motionMatrix[d - 1].top);
					jointRotation = jointRotation * deltaRot;
				}

				newParentToChild = (jointRotation * joint->relativeQuat).getNormalized();

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
				r = e + d;
				break;
			}
			case PxArticulationJointType::eFIX:
			{
				//this is fix joint so joint don't have velocity
				newParentToChild = joint->relativeQuat;

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				r = e + d;
				break;
			}
			default:
				break;
			}

			PxTransform& body2World = link.bodyCore->body2World;
			body2World.q = (pBody2World.q * newParentToChild.getConjugate()).getNormalized();
			body2World.p = pBody2World.p + body2World.q.rotate(r);

			PX_ASSERT(body2World.isSane());
		}
	}

	void FeatherstoneArticulation::jcalc(ArticulationData& data)
	{	
		if (getDirty())
		{
			ArticulationLink* links = data.getLinks();
			ArticulationJointCoreData* jointData = data.getJointData();
			const PxU32 linkCount = data.getLinkCount();

			PxU32 totalDof = 0;
			bool hasSphericalJoint = false;

			for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
			{

				ArticulationLink& link = links[linkID];
				ArticulationJointCore* joint = link.inboundJoint;
				ArticulationJointCoreData& jointDatum = jointData[linkID];
				
				PX_CHECK_AND_RETURN(joint->jointType != PxArticulationJointType::eUNDEFINED, "FeatherstoneArticulation::jcalc application need to define valid joint type and motion");
				jointDatum.computeJointDof(joint, false);
				joint->setJointPose(jointDatum);
				jointDatum.setJointVelocityDrive(joint);
				jointDatum.setJointPoseDrive(joint);

				if (joint->jointType == PxArticulationJointType::eSPHERICAL)
					hasSphericalJoint = true;

				jointDatum.jointOffset = totalDof;
				joint->jointOffset = totalDof;
				totalDof += jointDatum.dof;
			}

			if (totalDof != mArticulationData.getDofs())
			{
				mArticulationData.resizeJointData(totalDof);
				mArticulationData.setDofs(totalDof);
			}

			mHasSphericalJoint = hasSphericalJoint;

			setDirty(false);
		}
	}

	//compute link's spatial inertia tensor
	void  FeatherstoneArticulation::computeSpatialInertia(ArticulationData& data)
	{
		for (PxU32 linkID = 0; linkID < data.getLinkCount(); ++linkID)
		{
			ArticulationLink& link = data.getLink(linkID);
			ArticulationLinkData& linkDatum = data.getLinkData(linkID);

			PxsBodyCore& core = *link.bodyCore;

			const PxVec3& ii = core.inverseInertia;

			const PxReal m = core.inverseMass == 0.f ? 0.f : 1.0f / core.inverseMass;

			//construct mass matric
			linkDatum.spatialArticulatedInertia.topLeft = PxMat33(PxZero);
			//linkDatum.spatialArticulatedInertia.bottomRight = PxMat33(PxZero);
			linkDatum.spatialArticulatedInertia.topRight = PxMat33::createDiagonal(PxVec3(m));

			//construct inertia matrix
			PxMat33& I = linkDatum.spatialArticulatedInertia.bottomLeft;
			const PxVec3 inertiaTensor = PxVec3(ii.x == 0.f ? 0.f : (1.f / ii.x), ii.y == 0.f ? 0.f : (1.f / ii.y), ii.z == 0.f ? 0.f : (1.f / ii.z));
			I = PxMat33::createDiagonal(inertiaTensor);

			linkDatum.spatialInertia = linkDatum.spatialArticulatedInertia;
		}
	}

	void FeatherstoneArticulation::computeZ(ArticulationData& data, 
		const PxVec3& gravity, ScratchData& scratchData)
	{
		Cm::SpatialVectorF* motionVelocities = scratchData.motionVelocities;
		Cm::SpatialVectorF* spatialZAForces = scratchData.spatialZAVectors;
		Cm::SpatialVector* externalAccels = scratchData.externalAccels;

		for (PxU32 linkID = 0; linkID < data.getLinkCount(); ++linkID)
		{
			ArticulationLink& link = data.getLink(linkID);
			ArticulationLinkData& linkDatum = data.getLinkData(linkID);

			PxsBodyCore& core = *link.bodyCore;
			const PxTransform& body2World = core.body2World;

			//PxMat33& I = linkDatum.spatialArticulatedInertia.bottomLeft;

			//construct spatial zero acceleration
			Cm::SpatialVectorF& z = spatialZAForces[linkID];

			Cm::SpatialVectorF v;
			v.top = body2World.rotateInv(motionVelocities[linkID].top);
			v.bottom = body2World.rotateInv(motionVelocities[linkID].bottom);

			const PxVec3 exLinAccel = -body2World.rotateInv(gravity);

			/*Cm::SpatialVectorF Iv = linkDatum.spatialArticulatedInertia*v;
			z.top = -v.bottom.cross(Iv.top) + (exLinAccel * m);

			z.bottom = v.top.cross(Iv.bottom);*/
			const PxReal m = core.inverseMass == 0.f ? 0.f : 1.0f / core.inverseMass;
			const PxMat33& I = linkDatum.spatialInertia.bottomLeft;

			const PxVec3 inertiaTensor(I.column0.x, I.column1.y, I.column2.z);
			z.top = (exLinAccel * m);
			z.bottom = v.top.cross(inertiaTensor.multiply(v.top));

			PX_ASSERT(z.top.isFinite());
			PX_ASSERT(z.bottom.isFinite());
		}

		if (externalAccels)
		{
			for (PxU32 linkID = 0; linkID < data.getLinkCount(); ++linkID)
			{
				Cm::SpatialVector& externalAccel = externalAccels[linkID];

				ArticulationLink& link = data.getLink(linkID);
				ArticulationLinkData& linkDatum = data.getLinkData(linkID);

				PxsBodyCore& core = *link.bodyCore;
				const PxTransform& body2World = core.body2World;

				const PxReal m = core.inverseMass == 0.f ? 0.f : 1.0f / core.inverseMass;

				const PxMat33& I = linkDatum.spatialInertia.bottomLeft;

				const PxVec3 inertiaTensor(I.column0.x, I.column1.y, I.column2.z);

				//construct spatial zero acceleration
				Cm::SpatialVectorF& z = spatialZAForces[linkID];

				const PxVec3 exLinAccel = -body2World.rotateInv(externalAccel.linear);
				const PxVec3 exAngAccel = -body2World.rotateInv(externalAccel.angular);
				
				z.top += (exLinAccel * m);
				z.bottom += inertiaTensor.multiply(exAngAccel);

				PX_ASSERT(z.top.isFinite());
				PX_ASSERT(z.bottom.isFinite());

				//ML: we can't clear the external accelerations in here because it will break
				//TGS and when we are using inverse dynamic as well
				//externalAccel = Cm::SpatialVector(PxVec3(0), PxVec3(0));
			}
		}
	}

	//This can be called by forward dynamic
	void FeatherstoneArticulation::computeD(ArticulationData& data, ScratchData& scratchData,
		Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV)
	{
		Cm::SpatialVectorF* motionVelocities = scratchData.motionVelocities;
		
		Cm::SpatialVectorF* dragImpulse = Z;

		Cm::SpatialVectorF* ZAForces = scratchData.spatialZAVectors;

		PxMemZero(dragImpulse, sizeof(Cm::SpatialVectorF)*data.getLinkCount());

		const PxReal dt = data.getDt();

		bool bAppliedImpusle = false;

		for (PxU32 linkID = 0; linkID < data.getLinkCount(); ++linkID)
		{
			ArticulationLink& link = data.getLink(linkID);

			PxsBodyCore& core = *link.bodyCore;
			const PxTransform& body2World = core.body2World;

			const PxReal m = core.inverseMass == 0.f ? 0.f : 1.0f / core.inverseMass;

			//PxMat33& I = linkDatum.spatialArticulatedInertia.bottomLeft;
			const PxVec3 I = PxVec3(1.f / core.inverseInertia.x, 1.f / core.inverseInertia.y, 1.f / core.inverseInertia.z);

			Cm::SpatialVectorF& d = dragImpulse[linkID];

			Cm::SpatialVectorF v;
			v.top = body2World.rotateInv(motionVelocities[linkID].top);
			v.bottom = body2World.rotateInv(motionVelocities[linkID].bottom);  

			if (core.linearDamping > 0.f || core.angularDamping > 0.f)
			{
				const PxReal linDamp = PxMin(core.linearDamping*dt, 1.f);
				const PxReal angDamp = PxMin(core.angularDamping*dt, 1.f);

				d.top += (v.bottom * linDamp*m) -ZAForces[linkID].top * dt * linDamp;
				d.bottom += I.multiply(v.top* angDamp) -ZAForces[linkID].bottom * dt * angDamp;
				bAppliedImpusle = true;
			}

			const PxReal maxAng = core.maxAngularVelocitySq;
			const PxReal maxLin = core.maxLinearVelocitySq;

			const PxReal angMag = v.top.magnitudeSquared();
			const PxReal linMag = v.bottom.magnitudeSquared();

			if (angMag > maxAng || linMag > maxLin)
			{
				if (angMag > maxAng)
				{
					const PxReal scale = 1.f - PxSqrt(maxAng) / PxSqrt(angMag);
					const PxVec3 tmpaccelerationAng = (I.multiply(v.top)*scale);
					PX_UNUSED(tmpaccelerationAng);
					d.bottom = tmpaccelerationAng;

					bAppliedImpusle = true;
				}

				if (linMag > maxLin)
				{
					const PxReal scale = 1.f - (PxSqrt(maxLin) / PxSqrt(linMag))*0.8f;
					const PxVec3 tmpaccelerationLin = (v.bottom*m*scale);
					PX_UNUSED(tmpaccelerationLin);
					d.top = tmpaccelerationLin;

					bAppliedImpusle = true;
				}
			}
		}

		if (bAppliedImpusle)
		{
			applyImpulses(dragImpulse, DeltaV);

			PxReal* deltaV = data.getJointDeltaVelocities();
			PxReal* jointV = data.getJointVelocities();

			for (PxU32 linkID = 1; linkID < data.getLinkCount(); ++linkID)
			{
				ArticulationJointCoreData& tJointDatum = data.getJointData()[linkID];
				for (PxU32 i = 0; i < tJointDatum.dof; ++i)
				{
					jointV[i + tJointDatum.jointOffset] += deltaV[i + tJointDatum.jointOffset];
					deltaV[i + tJointDatum.jointOffset] = 0.f;
				}
			}
		}
	}

	//compute coriolis and centrifugal term
	void FeatherstoneArticulation::computeC(ArticulationData& data, ScratchData& scratchData)
	{
		const PxReal* jointVelocities = scratchData.jointVelocities;
		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;

		const PxU32 linkCount = data.getLinkCount();

		if (jointVelocities)
		{
			for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
			{
				ArticulationLink& link = data.getLink(linkID);
				ArticulationLinkData& linkDatum = data.getLinkData(linkID);
				ArticulationJointCoreData& jointDatum = data.getJointData(linkID);

				const PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
				PX_UNUSED(jVelocity);
				Cm::SpatialVectorF& coriolis = coriolisVectors[linkID];

				if (jointDatum.dof > 0)
				{
					//transform parent link's angular velocity into current link's body space
					const PxVec3 parentAngular = scratchData.motionVelocities[link.parent].top;
					const PxTransform& body2World = link.bodyCore->body2World;
					const PxVec3 pAngular = body2World.q.rotateInv(parentAngular);
					const PxVec3 temp0 = pAngular.cross(pAngular.cross(linkDatum.r));

#if 1
					Cm::SpatialVectorF relVel(PxVec3(0.f), PxVec3(0.f));
					for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
					{
						const PxReal jV = jVelocity[ind];
						relVel += jointDatum.motionMatrix[ind] * jV;
					}

					const PxVec3 aVec = relVel.top;
					const PxVec3 force = pAngular.cross(aVec);

					//compute linear part
					const PxVec3 lVel = relVel.bottom;

					const PxVec3 temp1 = 2.f * pAngular.cross(lVel);
					const PxVec3 temp2 = aVec.cross(lVel);
					const PxVec3 torque = temp0 + temp1 + temp2;
					coriolis = Cm::SpatialVectorF(force, torque);
#else
					coriolis = Cm::SpatialVectorF(PxVec3(0.f), temp0);

					//motionMatrix is in link space
					for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
					{
						const PxReal jV = jVelocity[ind];
						
						//compute angular part
						const PxVec3 aVec = jointDatum.motionMatrix[ind].top * jV;
						const PxVec3 angular = pAngular.cross(aVec);

						//compute linear part
						const PxVec3 lVel = jointDatum.motionMatrix[ind].bottom * jV;		
					
						const PxVec3 temp1 = 2 * pAngular.cross(lVel);
						const PxVec3 temp2 = aVec.cross(lVel);
						const PxVec3 linear = temp1 + temp2;

						Cm::SpatialVectorF tCoriolis(angular, linear);

						coriolis += tCoriolis;
						PX_ASSERT(coriolis.isFinite());
					}
#endif
				}
				else
				{
					//fix joint
					coriolis = Cm::SpatialVectorF(PxVec3(0), PxVec3(0));
				}
			}
		}
		else
		{
			PxMemZero(coriolisVectors, sizeof(Cm::SpatialVectorF)*linkCount);
		}
	}

	static PxMat33 constructSkewSymmetricMatrix(const PxVec3 r)
	{
		return PxMat33(	PxVec3(0.0f, r.z, -r.y),
						PxVec3(-r.z, 0.0f, r.x),
						PxVec3(r.y, -r.x, 0.0f));
	}

	void FeatherstoneArticulation::computeRelativeTransformC2P(ArticulationData& data)
	{
		ArticulationLink* links = data.getLinks();
		ArticulationLinkData* linkData = data.getLinkData();
		const PxU32 linkCount = data.getLinkCount();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];
			ArticulationLinkData& linkDatum = linkData[linkID];
			PxsBodyCore& bodyCore = *link.bodyCore;

			const PxTransform& body2World = bodyCore.body2World;

			ArticulationLink& pLink = links[link.parent];
			PxsBodyCore& pBodyCore = *pLink.bodyCore;
			const PxTransform& pBody2World = pBodyCore.body2World;

			const PxTransform tC2P = pBody2World.transformInv(body2World).getNormalized();
			
			linkDatum.childToParent.R = PxMat33(tC2P.q);
			linkDatum.childToParent.q = tC2P.q;
			linkDatum.r = body2World.rotateInv(body2World.p - pBody2World.p);//body space of link i
			linkDatum.rw =body2World.p - pBody2World.p;

			//child to parent rotation matrix
			const PxMat33& c2p = linkDatum.childToParent.R;
			//r is in link body space
			const PxMat33 skewMatrixPR = -constructSkewSymmetricMatrix(linkDatum.r);
			//rotation matrix cToP's inverse is rotation matrix pToC 
			linkDatum.childToParent.T = c2p * (-skewMatrixPR);

#if FEATURESTONE_DEBUG
			{
				//debug
				PxMat33 pToC = c2p.getTranspose();
				//parentToChild -rR
				PxMat33 T2 = skewMatrixPR * pToC;

				PX_ASSERT(SpatialMatrix::isTranspose(linkDatum.childToParent.T, T2));
			}
#endif
		}
	}

	void FeatherstoneArticulation::computeRelativeTransformC2B(ArticulationData& data)
	{
		ArticulationLink* links = data.getLinks();
		ArticulationLinkData* linkData = data.getLinkData();
		const PxU32 linkCount = data.getLinkCount();

		ArticulationLink& bLink = links[0];
		const PxTransform& bBody2World = bLink.bodyCore->body2World;

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];
			ArticulationLinkData& linkDatum = linkData[linkID];
			PxsBodyCore& bodyCore = *link.bodyCore;

			const PxTransform& body2World = bodyCore.body2World;

			const PxTransform tC2B = bBody2World.transformInv(body2World).getNormalized();
			linkDatum.childToBase.R= PxMat33(tC2B.q);
			linkDatum.childToBase.q = tC2B.q;
			const PxVec3 r = body2World.rotateInv(body2World.p - bBody2World.p);//body space of link i
	
			//child to parent rotation matrix
			const PxMat33& c2b = linkDatum.childToParent.R;
			//r is in link body space
			const PxMat33 skewMatrixPR = -constructSkewSymmetricMatrix(r);
			//rotation matrix cToP's inverse is rotation matrix pToC 
			linkDatum.childToBase.T = c2b * (-skewMatrixPR);
		}
	}

	//compute all links velocities
	void FeatherstoneArticulation::computeLinkVelocities(ArticulationData& data,
		ScratchData& scratchData)
	{
		ArticulationLink* links = data.getLinks();
		ArticulationLinkData* linkData = data.getLinkData();
		const PxU32 linkCount = data.getLinkCount();
		const bool fixBase = data.getCore()->flags & PxArticulationFlag::eFIX_BASE;

		//motion velocities has to be in world space to avoid numerical errors caused by space 
		Cm::SpatialVectorF* motionVelocities = scratchData.motionVelocities;
		Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;

		PxReal* jointVelocities = scratchData.jointVelocities;
		
		ArticulationLink& baseLink = links[0];
		ArticulationLinkData& baseLinkDatum = linkData[0];

		PxsBodyCore& core0 = *baseLink.bodyCore;

		baseLinkDatum.maxPenBias = core0.maxPenBias;
		
		if (fixBase)
		{
			motionVelocities[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			motionAccelerations[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
		}
		else
		{
			motionVelocities[0] = Cm::SpatialVectorF(core0.angularVelocity, core0.linearVelocity);
		}

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];
			ArticulationLinkData& linkDatum = linkData[linkID];
			PxsBodyCore& bodyCore = *link.bodyCore;

			linkDatum.maxPenBias = bodyCore.maxPenBias;

			//SpatialTransform p2c = linkDatum.childToParent.getTranspose();

			//motionVelocites[linkID] = p2c * motionVelocites[link.parent];
			//motionVelocites[linkID] = motionVelocites[link.parent];
			/*motionVelocities[linkID].top = motionVelocities[link.parent].top;
			motionVelocities[linkID].bottom = motionVelocities[link.parent].bottom + motionVelocities[link.parent].top.cross(linkDatum.rw);*/

			PxVec3 ang = motionVelocities[link.parent].top;
			PxVec3 lin = motionVelocities[link.parent].bottom + motionVelocities[link.parent].top.cross(linkDatum.rw);
			const PxTransform& body2World = bodyCore.body2World;

			if (jointVelocities)
			{
				ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
				//const PxReal* const jV = &jointVelocity[linkID * 6];
				PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];

				Cm::SpatialVectorF deltaV = Cm::SpatialVectorF::Zero();
				for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
				{
					//KS - TODO - only hack for rotational constraints!!!
					//jVelocity[ind] = PxClamp(jVelocity[ind], -100.f, 100.f);
					deltaV += jointDatum.motionMatrix[ind] * jVelocity[ind];
				}

				ang += body2World.rotate(deltaV.top);
				lin += body2World.rotate(deltaV.bottom);
			}

			motionVelocities[linkID] = Cm::SpatialVectorF(ang, lin);
		}
	}

	void FeatherstoneArticulation::solveInternalConstraints(const PxReal dt, const PxReal invDt,
		Cm::SpatialVectorF* impulses, Cm::SpatialVectorF* DeltaV, bool velocityIteration)
	{
		PX_UNUSED(dt);
		PX_UNUSED(invDt);
		PX_UNUSED(velocityIteration);
		const PxU32 count = mArticulationData.getLinkCount();

		if (count <= 1)
			return;

		const bool fixBase = mArticulationData.getCore()->flags & PxArticulationFlag::eFIX_BASE;

		////(1) Flush velocities first...TODO - incorporate into code below properly...done!
		//if(mArticulationData.mJointDirty)
		//	PxcFsFlushVelocity(*this, DeltaV);
		
		ArticulationLink* links = mArticulationData.getLinks();
		PxReal* jointPositions = mArticulationData.getJointPositions();
		Cm::SpatialVectorF* baseVelocities = mArticulationData.getMotionVelocities();
		//PxTransform* transforms = mArticulationData.mAccumulatedPoses.begin();
		PxTransform* transforms = mArticulationData.mPreTransform.begin();
		Cm::SpatialVectorF* deltaP = mArticulationData.getDeltaMotionVector();
		
		PX_UNUSED(baseVelocities);
		PX_UNUSED(transforms);
		
		//PxMemZero(DeltaV, sizeof(Cm::SpatialVector)*count);

		Cm::SpatialVectorF* deferredZ = mArticulationData.getSpatialZAVectors();

		bool impulse = false;
		{
			impulses[0].top = PxVec3(0.f);
			impulses[0].bottom = PxVec3(0.f);

			if (!fixBase)
			{
				//ArticulationLink& link = links[0];

				Cm::SpatialVectorF temp =  mArticulationData.getBaseInvSpatialArticulatedInertia() * (-deferredZ[0]);
				
				const PxTransform& body2World0 = transforms[0];
				DeltaV[0] = temp.rotate(body2World0);
			}
			else
			{
				DeltaV[0].top = PxVec3(0.f);
				DeltaV[0].bottom = PxVec3(0.f);
			}

			PxU32 dofId = 0;
			PxU32 lockId = 0;
			for (PxU32 linkID = 1; linkID < count; ++linkID)
			{				
				const ArticulationLink& link = links[linkID];
				//const ArticulationLink& plink = links[link.parent];
				ArticulationLinkData& linkDatum = mArticulationData.getLinkData(linkID);

				PX_UNUSED(linkDatum);

				const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);
				PxReal* jPosition = &jointPositions[jointDatum.jointOffset];
				PX_UNUSED(jPosition);
				//PxReal* jVelocity = &jointVelocity[jointDatum.jointOffset];
				//PX_UNUSED(jVelocity);

				Cm::UnAlignedSpatialVector i1(PxVec3(0.f), PxVec3(0.f));
				
				Cm::SpatialVectorF parentV = DeltaV[link.parent] + baseVelocities[link.parent];

				const Cm::SpatialVectorF localParentAccel = DeltaV[link.parent].rotateInv(transforms[link.parent]);

				const Cm::SpatialVectorF parentVelContrib = propagateVelocityTestImpulse(linkDatum, jointDatum, deferredZ[linkID], localParentAccel).rotate(transforms[linkID]);
				//Cm::SpatialVectorF parentVelContrib = ComputeDeltaVelocity(linkDatum, jointDatum, localParentAccel).rotate(transforms[linkID]);

				//DeltaV[linkID] will always be zero so doesn't need to be used in here...
				Cm::SpatialVectorF childV = baseVelocities[linkID] + parentVelContrib;

				//KS - we need to record this in dv1. Even if the constraint doesn't apply any impulses, a previous
				//constraint could have done, and this could lead to something not being accumulated.
				Cm::UnAlignedSpatialVector dv1(parentVelContrib.top, parentVelContrib.bottom);

				if (jointDatum.dofInternalConstraintMask || jointDatum.lockedAxes)
				{
					Cm::UnAlignedSpatialVector i0(PxVec3(0.f), PxVec3(0.f));
					Cm::UnAlignedSpatialVector dv0(PxVec3(0.f), PxVec3(0.f));

					for (PxU32 dof = 0; dof < jointDatum.dof; ++dof)
					{
						if (jointDatum.dofInternalConstraintMask & (1 << dof))
						{
							ArticulationInternalConstraint& constraint = mArticulationData.mInternalConstraints[dofId++];

							PxReal jointP = jPosition[dof];

							if (!constraint.isLinearConstraint)
							{
								//Clamp jointP within +/- PxPi
								if (jointP > PxTwoPi)
									jointP -= 2.f*PxTwoPi;
								else if (jointP < -PxTwoPi)
									jointP += 2.f*PxTwoPi;
							}

							PxReal error = (constraint.driveTarget - jointP);

							//if (!constraint.isLinearConstraint)
							//{
							//	if (PxAbs(error) > PxPi)
							//	{
							//		//drive target and current angle are in different hemispheres, so we 
							//		//should see if we can legally reach the target going the "shorter" route,
							//		//otherwise we stick with the straight line route...
							//		PxReal targetInPHemisphere = constraint.driveTarget;
							//		if(error < 0.f)
							//			targetInPHemisphere += PxTwoPi;
							//		else
							//			targetInPHemisphere -= PxTwoPi;

							//		if (constraint.lowLimit <= targetInPHemisphere && targetInPHemisphere <= constraint.highLimit)
							//		{
							//			error = targetInPHemisphere - jointP;
							//		}
							//	}
							//}
							
							PxReal jointV = constraint.row1.innerProduct(childV) - constraint.row0.innerProduct(parentV);

							const PxReal appliedFriction = constraint.frictionForce*constraint.frictionForceCoefficient;

							const PxReal frictionForce = PxClamp(-jointV *constraint.recipResponse + appliedFriction,
								-constraint.maxFrictionForce*dt, constraint.maxFrictionForce*dt);

							const PxReal frictionDeltaF = frictionForce - appliedFriction;

							constraint.frictionForce += frictionDeltaF;

							jointV += frictionDeltaF * constraint.response;

							const PxReal unclampedForce = constraint.driveImpulseMultiplier * constraint.driveForce +
								jointV * constraint.driveVelMultiplier + constraint.driveTargetVel + error * constraint.driveBiasCoefficient;

							const PxReal clampedForce = PxClamp(unclampedForce, -constraint.maxDriveForce, constraint.maxDriveForce);
							PxReal driveDeltaF = (clampedForce - constraint.driveForce);

							//Where we will be next frame - we use this to compute error bias terms to correct limits and drives...
							
							jointV += driveDeltaF * constraint.response;							

							driveDeltaF += frictionDeltaF;

							PxReal deltaF = 0.f;

							if (velocityIteration)
							{
								deltaF = PxClamp(-jointV*constraint.recipResponse, -constraint.lowImpulse, -constraint.highImpulse);
							}
							else
							{
								const PxReal futureP = jointP + jointV * dt;
								if (futureP > constraint.highLimit)
								{
									const PxReal erp = jointP > constraint.highLimit ? constraint.erp : 1.f;
									//PxReal deltaV = (constraint.highLimit - jointP)*invDt*erp - jointV;
									const PxReal deltaV = (constraint.highLimit - futureP)*invDt*erp;
									deltaF = PxMin(constraint.highImpulse + deltaV * constraint.recipResponse, 0.f) - constraint.highImpulse;

									constraint.highImpulse += deltaF;
								}
								else if (futureP < constraint.lowLimit)
								{
									const PxReal erp = jointP < constraint.lowLimit ? constraint.erp : 1.f;
									//PxReal deltaV = (constraint.lowLimit - jointP)*invDt*erp - jointV;
									const PxReal deltaV = (constraint.lowLimit - futureP)*invDt*erp;
									deltaF = PxMax(constraint.lowImpulse + deltaV * constraint.recipResponse, 0.f) - constraint.lowImpulse;
									constraint.lowImpulse += deltaF;
								}
							}

							deltaF += driveDeltaF;

							if (deltaF != 0.f)
							{
								impulse = true;
								constraint.driveForce = clampedForce;

								i0 += constraint.row0 * deltaF;
								i1 -= constraint.row1 * deltaF;

								const Cm::UnAlignedSpatialVector deltaVP = -constraint.deltaVA * deltaF;
								const Cm::UnAlignedSpatialVector deltaVC = -constraint.deltaVB * deltaF;

								dv0 += deltaVP;
								dv1 += deltaVC;

								parentV += Cm::SpatialVectorF(deltaVP.top, deltaVP.bottom);
								childV += Cm::SpatialVectorF(deltaVC.top, deltaVC.bottom);
							}						
						}
					}

					PxU32 startIdx = PxU32(jointDatum.dof - jointDatum.lockedAxes);

					for (PxU32 i = startIdx; i < jointDatum.dof; ++i)
					{
						ArticulationInternalLockedAxis& lockAxis = mArticulationData.mInternalLockedAxes[lockId++];

						const PxReal jointV = lockAxis.axis.dot(childV.top) - lockAxis.axis.dot(parentV.top);

						const PxReal deltaJointP = lockAxis.axis.dot(deltaP[linkID].top) - lockAxis.axis.dot(deltaP[link.parent].top);
						PX_UNUSED(deltaJointP);

						PxReal deltaV = -jointV;
						//We aim to zero velocity but also correct bias...
						if(!velocityIteration)
							deltaV += PxClamp((lockAxis.error - deltaJointP) * lockAxis.biasScale, -30.f, 30.f);
							//deltaV +=PxClamp((lockAxis.error - deltaJointP) * lockAxis.biasScale, -10.f, 10.f);

						//deltaV = PxClamp(deltaV, -5.f, 5.f);

						const PxReal deltaF = deltaV * lockAxis.recipResponse;

						if (deltaF != 0.f)
						{
							impulse = true;

							i0.bottom += lockAxis.axis * deltaF;
							i1.bottom -= lockAxis.axis * deltaF;

							const Cm::UnAlignedSpatialVector tDeltaVP = -lockAxis.deltaVA * deltaF;
							const Cm::UnAlignedSpatialVector tDeltaVC = -lockAxis.deltaVB * deltaF;

							const Cm::SpatialVectorF deltaVP = Cm::SpatialVectorF(tDeltaVP.top, tDeltaVP.bottom);
							const Cm::SpatialVectorF deltaVC = Cm::SpatialVectorF(tDeltaVC.top, tDeltaVC.bottom);

							dv0 += deltaVP;
							dv1 += deltaVC;

							parentV += deltaVP;
							childV += deltaVC;
						}
					}

					DeltaV[link.parent] += Cm::SpatialVectorF(dv0.top, dv0.bottom);
					impulses[link.parent] += Cm::SpatialVectorF(i0.top, i0.bottom);
				}

				DeltaV[linkID] = Cm::SpatialVectorF(dv1.top, dv1.bottom);
				impulses[linkID] = Cm::SpatialVectorF(i1.top, i1.bottom);
			}

			if (1)
			{
				//Now propagate impulses...
				if (impulse)
				{
					for (PxU32 i = 0; i < count; ++i)
					{
						//impulses[i] = impulses[i].rotateInv(/*transforms[i]*/links[i].bodyCore->body2World);
						impulses[i] = impulses[i].rotateInv(transforms[i]);
					}
					pxcFsApplyImpulses(impulses);
					//applyImpulses(impulses, DeltaV);
				}
			}
		}
	}

	//This method is for user update the root link transform so we need to
	//fix up other link's position. In this case, we should assume all joint
	//velocity/pose is to be zero
	void FeatherstoneArticulation::teleportRootLink()
	{
		//make sure motionMatrix has been set
		jcalc(mArticulationData);

		const PxU32 linkCount = mArticulationData.getLinkCount();
		ArticulationLink* links = mArticulationData.getLinks();
		PxReal* jointPositions = mArticulationData.getJointPositions();
		Cm::SpatialVectorF* motionVelocities = mArticulationData.getMotionVelocities();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];
			const PxTransform oldTransform = link.bodyCore->body2World;

			ArticulationLink& pLink = links[link.parent];
			const PxTransform pBody2World = pLink.bodyCore->body2World;

			ArticulationJointCore* joint = link.inboundJoint;
			ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);

			PxReal* jPosition = &jointPositions[jointDatum.jointOffset];

			PxQuat newParentToChild;
			PxQuat newWorldQ;
			PxVec3 r;

			const PxVec3 childOffset = -joint->childPose.p;
			const PxVec3 parentOffset = joint->parentPose.p;

			switch (joint->jointType)
			{
			case PxArticulationJointType::ePRISMATIC:
			{
				newParentToChild = joint->relativeQuat;
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				const PxVec3& u = jointDatum.motionMatrix[0].bottom;

				r = e + d + u * jPosition[0];
				break;
			}
			case PxArticulationJointType::eREVOLUTE:
			{
				const PxVec3& u = jointDatum.motionMatrix[0].top;

				PxQuat jointRotation = PxQuat(-jPosition[0], u);
				if (jointRotation.w < 0)	//shortest angle.
					jointRotation = -jointRotation;

				newParentToChild = (jointRotation * joint->relativeQuat).getNormalized();

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
				r = e + d;

				/*PxVec3 worldAngVel = oldTransform.rotate(link.motionVelocity.top);

				newWorldQ = Ps::exp(worldAngVel*dt) * oldTransform.q;

				PxQuat newParentToChild2 = (newWorldQ.getConjugate() * joint->relativeQuat * pBody2World.q).getNormalized();

				const PxVec3 e2 = newParentToChild2.rotate(parentOffset);
				const PxVec3 d2 = childOffset;
				r = e2 + d2;*/

				PX_ASSERT(r.isFinite());

				break;
			}
			case PxArticulationJointType::eSPHERICAL:
			{

				//PxVec3 angVel(joint->jointVelocity[0], joint->jointVelocity[1], joint->jointVelocity[2]);
				//PxVec3 worldAngVel = pLink.bodyCore->angularVelocity + oldTransform.rotate(angVel);

				PxVec3 worldAngVel = motionVelocities[linkID].top;

				/*const PxReal eps = 0.001f;
				const PxVec3 dif = worldAngVel - worldAngVel2;
				PX_ASSERT(PxAbs(dif.x) < eps && PxAbs(dif.y) < eps && PxAbs(dif.z) < eps);*/

				newWorldQ = Ps::exp(worldAngVel) * oldTransform.q;

				newParentToChild = (newWorldQ.getConjugate() * joint->relativeQuat * pBody2World.q).getNormalized();

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
				r = e + d;

				PX_ASSERT(r.isFinite());
				break;
			}
			case PxArticulationJointType::eFIX:
			{
				//this is fix joint so joint don't have velocity
				newParentToChild = joint->relativeQuat;

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				r = e + d;
				break;
			}
			default:
				break;
			}

			PxTransform& body2World = link.bodyCore->body2World;
			body2World.q = (pBody2World.q * newParentToChild.getConjugate()).getNormalized();
			body2World.p = pBody2World.p + body2World.q.rotate(r);

			PX_ASSERT(body2World.isSane());
		}
	}


	PxU8* FeatherstoneArticulation::allocateScratchSpatialData(PxcScratchAllocator* allocator,
		const PxU32 linkCount, ScratchData& scratchData, bool fallBackToHeap)
	{
		const PxU32 size = sizeof(Cm::SpatialVectorF) * linkCount;
		const PxU32 totalSize = size * 4 + sizeof(Dy::SpatialMatrix) * linkCount;

		PxU8* tempMemory = reinterpret_cast<PxU8*>(allocator->alloc(totalSize, fallBackToHeap));

		scratchData.motionVelocities = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory);
		PxU32 offset = size;
		scratchData.motionAccelerations = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory + offset);
		offset += size;
		scratchData.coriolisVectors = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory + offset);
		offset += size;
		scratchData.spatialZAVectors = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory + offset);
		offset += size;
		scratchData.compositeSpatialInertias = reinterpret_cast<Dy::SpatialMatrix*>(tempMemory + offset);

		return tempMemory;
	}

/*	void FeatherstoneArticulation::allocateScratchSpatialData(DyScratchAllocator& allocator,
		const PxU32 linkCount, ScratchData& scratchData)
	{
		const PxU32 size = sizeof(Cm::SpatialVectorF) * linkCount;
		const PxU32 totalSize = size * 5 + sizeof(Dy::SpatialMatrix) * linkCount;

		PxU8* tempMemory = allocator.alloc<PxU8>(totalSize);

		scratchData.motionVelocities = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory);
		PxU32 offset = size;
		scratchData.motionAccelerations = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory + offset);
		offset += size;
		scratchData.coriolisVectors = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory + offset);
		offset += size;
		scratchData.spatialZAVectors = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory + offset);
		offset += size;
		scratchData.externalAccels = reinterpret_cast<Cm::SpatialVector*>(tempMemory + offset);
		offset += size;
		scratchData.compositeSpatialInertias = reinterpret_cast<Dy::SpatialMatrix*>(tempMemory + offset);
		
	}*/


}//namespace Dy
}
