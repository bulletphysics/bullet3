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
#include "DyFeatherstoneArticulationLink.h"
#include "DyFeatherstoneArticulationJointData.h"
#include "PsFoundation.h"
#include "PxsIslandSim.h"
#include "common/PxProfileZone.h"
#include <stdio.h>


#ifdef _MSC_VER
#pragma warning(disable:4505)
#endif

namespace physx
{
namespace Dy
{
	void PxcFsFlushVelocity(FeatherstoneArticulation& articulation, Cm::SpatialVectorF* deltaV);

	//initialize spatial articualted matrix and coriolis spatial force
	void FeatherstoneArticulation::initLinks(ArticulationData& data,
		const PxVec3& gravity, ScratchData& scratchData, Cm::SpatialVectorF* Z,
		Cm::SpatialVectorF* DeltaV)
	{
		PX_UNUSED(Z);
		PX_UNUSED(DeltaV);
		//compute individual link's spatial inertia tensor
		//[0, M]
		//[I, 0]
		computeSpatialInertia(data);

		//compute inidividual zero acceleration force
		computeZ(data, gravity, scratchData);

		Cm::SpatialVectorF* za = mArticulationData.getTransmittedForces();
		//copy individual zero acceleration force to mTempData zaForce buffer
		PxMemCopy(za, mArticulationData.getSpatialZAVectors(), sizeof(Cm::SpatialVectorF) * mArticulationData.getLinkCount());

		computeArticulatedSpatialInertia(data);
	
		computeD(data, scratchData, Z, DeltaV);

		//compute corolis and centrifugal force
		computeC(data, scratchData);

		computeArticulatedSpatialZ(mArticulationData, scratchData);
	}


#if (FEATHERSTONE_DEBUG && (PX_DEBUG || PX_CHECKED))
	static bool isSpatialVectorEqual(Cm::SpatialVectorF& t0, Cm::SpatialVectorF& t1)
	{
		float eps = 0.0001f;
		bool e0 = PxAbs(t0.top.x - t1.top.x) < eps &&
			PxAbs(t0.top.y - t1.top.y) < eps &&
			PxAbs(t0.top.z - t1.top.z) < eps;

		bool e1 = PxAbs(t0.bottom.x - t1.bottom.x) < eps &&
			PxAbs(t0.bottom.y - t1.bottom.y) < eps &&
			PxAbs(t0.bottom.z - t1.bottom.z) < eps;

		return e0 && e1;
	}

	static bool isSpatialVectorZero(Cm::SpatialVectorF& t0)
	{
		float eps = 0.000001f;

		const bool c0 = PxAbs(t0.top.x) < eps && PxAbs(t0.top.y) < eps && PxAbs(t0.top.z) < eps;
		const bool c1 = PxAbs(t0.bottom.x) < eps && PxAbs(t0.bottom.y) < eps && PxAbs(t0.bottom.z) < eps;

		return c0 && c1;
	}
#endif

	//calculate Is
	void FeatherstoneArticulation::computeIs(ArticulationLinkData& linkDatum, ArticulationJointCoreData& jointDatum)
	{
		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
		{
			Cm::SpatialVectorF& sa = jointDatum.motionMatrix[ind];
			linkDatum.Is[ind] = linkDatum.spatialArticulatedInertia * sa;
		}
	}


	//compute inertia contribution part
	SpatialMatrix FeatherstoneArticulation::computePropagateSpatialInertia(const PxU8 jointType, ArticulationLinkData& linkDatum, ArticulationJointCoreData& jointDatum)
	{
		SpatialMatrix spatialInertia;

		switch (jointType)
		{
		case PxArticulationJointType::ePRISMATIC:
		case PxArticulationJointType::eREVOLUTE:
		{
			Cm::SpatialVectorF& sa = jointDatum.motionMatrix[0];

			Cm::SpatialVectorF& Is = linkDatum.Is[0];

			const PxReal stIs = sa.innerProduct(linkDatum.Is[0]);

			linkDatum.invStIs[0][0] = (stIs > /*PX_EPS_REAL*/1e-5f) ? (1.f / stIs) : 0.f;

			linkDatum.IsInvD[0] = Is * linkDatum.invStIs[0][0];


			//(6x1)Is = [v0, v1]; (1x6)stI = [v1, v0]
			//Cm::SpatialVector stI1(Is1.angular, Is1.linear);
			Cm::SpatialVectorF stI(Is.bottom, Is.top);

			spatialInertia = SpatialMatrix::constructSpatialMatrix(linkDatum.IsInvD[0], stI);

			break;
		}
		case PxArticulationJointType::eSPHERICAL:
		{


#if FEATHERSTONE_DEBUG
			//This is for debugging
			Temp6x6Matrix bigInertia(linkDatum.spatialArticulatedInertia);
			Temp6x3Matrix bigS(jointDatum.motionMatrix.getColumns());

			Temp6x3Matrix bigIs = bigInertia * bigS;


			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				Cm::SpatialVectorF tempIs = bigInertia * jointDatum.motionMatrix[ind];

				PX_ASSERT(isSpatialVectorEqual(tempIs, linkDatum.Is[ind]));

				PX_ASSERT(bigIs.isColumnEqual(ind, tempIs));

			}
#endif

			PxMat33 D(PxIdentity);
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					Cm::SpatialVectorF& sa = jointDatum.motionMatrix[ind2];
					D[ind][ind2] = sa.innerProduct(linkDatum.Is[ind]);
				}
			}

			PxMat33 invD = SpatialMatrix::invertSym33(D);
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					linkDatum.invStIs[ind][ind2] = invD[ind][ind2];

				}
			}

#if FEATHERSTONE_DEBUG
			//debugging
			Temp6x3Matrix bigIsInvD = bigIs * invD;
#endif

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				linkDatum.IsInvD[ind] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));

				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					Cm::SpatialVectorF& Is = linkDatum.Is[ind2];
					linkDatum.IsInvD[ind] += Is * linkDatum.invStIs[ind][ind2];
				}

#if FEATHERSTONE_DEBUG

				const bool equal = bigIsInvD.isColumnEqual(ind, linkDatum.IsInvD[ind]);
				PX_ASSERT(equal);
#endif
			}


#if FEATHERSTONE_DEBUG
			Temp6x6Matrix transpose6x6 = bigInertia.getTranspose();
#endif
			PxReal stI[6][3]; //[column][row]
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
#if FEATHERSTONE_DEBUG
				Cm::SpatialVectorF& sa = jointDatum.motionMatrix[ind];

				Cm::SpatialVectorF sat = Cm::SpatialVectorF(sa.bottom, sa.top);
				Cm::SpatialVectorF tstI = transpose6x6 * sat;
#endif

				Cm::SpatialVectorF& Is = linkDatum.Is[ind];

#if FEATHERSTONE_DEBUG
				Cm::SpatialVectorF temp(Is.bottom, Is.top);
				const bool equal = isSpatialVectorEqual(temp, tstI);
				PX_ASSERT(equal);
#endif

				//(6x1)Is = [v0, v1]; (1x6)stI = [v1, v0]
				stI[0][ind] = Is.bottom.x;
				stI[1][ind] = Is.bottom.y;
				stI[2][ind] = Is.bottom.z;
				stI[3][ind] = Is.top.x;
				stI[4][ind] = Is.top.y;
				stI[5][ind] = Is.top.z;
			}

			Cm::SpatialVectorF columns[6];
			for (PxU32 ind = 0; ind < 6; ++ind)
			{
				columns[ind] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					columns[ind] += linkDatum.IsInvD[ind2] * stI[ind][ind2];
				}
			}

			spatialInertia = SpatialMatrix::constructSpatialMatrix(columns);
#if FEATHERSTONE_DEBUG
			Temp6x6Matrix result = bigIsInvD * stI;
			PX_ASSERT(result.isEqual(columns));
#endif

			break;
		}
		default:
			spatialInertia.setZero();
			break;
		}

		//(I - Is*Inv(sIs)*sI)
		spatialInertia = linkDatum.spatialArticulatedInertia - spatialInertia;

		return spatialInertia;
	}
	
	void FeatherstoneArticulation::computeArticulatedSpatialInertia(ArticulationData& data)
	{
		ArticulationLink* links = data.getLinks();
		ArticulationLinkData* linkData = data.getLinkData();
		ArticulationJointCoreData* jointData = data.getJointData();
		const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = PxU32(linkCount - 1);

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			ArticulationLink& link = links[linkID];
			ArticulationLinkData& linkDatum = linkData[linkID];
			
			ArticulationJointCoreData& jointDatum = jointData[linkID];
			computeIs(linkDatum, jointDatum);

			//(I - Is*Inv(sIs)*sI)
			SpatialMatrix spatialInertia = computePropagateSpatialInertia(link.inboundJoint->jointType,
				linkDatum, jointDatum);

			//transform spatial inertia into parent space
			transformInertia(linkDatum.childToParent, spatialInertia);
			//PX_ASSERT(spatialInertia.isTranspose(spatialInertia.topLeft, spatialInertia.bottomRight));
			
			//accumulate child's articulated spatial inertia to the parent's articulated spatial inertia
			ArticulationLinkData& pLinkDatum = linkData[link.parent];
			pLinkDatum.spatialArticulatedInertia += spatialInertia;
		}

		//cache base link inverse spatial inertia
		ArticulationLinkData& bLinkDatum = linkData[0];
		data.mBaseInvSpatialArticulatedInertia = bLinkDatum.spatialArticulatedInertia.invertInertia();
	}

	void FeatherstoneArticulation::computeArticulatedSpatialZ(ArticulationData& data,
		ScratchData& scratchData)
	{
		ArticulationLink* links = data.getLinks();
		ArticulationLinkData* linkData = data.getLinkData();
		ArticulationJointCoreData* jointData = data.getJointData();

		const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = PxU32(linkCount - 1);

		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;
		Cm::SpatialVectorF* articulatedZA = scratchData.spatialZAVectors;

		PxReal* jointForces = scratchData.jointForces;
		
		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			ArticulationLink& link = links[linkID];
			ArticulationLinkData& linkDatum = linkData[linkID];
		
			ArticulationJointCoreData& jointDatum = jointData[linkID];

			//calculate spatial zero acceleration force, this can move out of the loop
			Cm::SpatialVectorF Ic = linkDatum.spatialArticulatedInertia * coriolisVectors[linkID];
			Cm::SpatialVectorF ZIc = articulatedZA[linkID] + Ic;

			const PxReal* jF = &jointForces[jointDatum.jointOffset];
			
			Cm::SpatialVectorF ZA(PxVec3(0.f), PxVec3(0.f));
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				Cm::SpatialVectorF& sa = jointDatum.motionMatrix[ind];
				const PxReal stZ = sa.innerProduct(ZIc);

				//link.qstZIc[ind] = jF[ind] - stZ;
				linkDatum.qstZIc[ind] = jF[ind] - stZ;
				PX_ASSERT(PxIsFinite(linkDatum.qstZIc[ind]));

				ZA += linkDatum.IsInvD[ind] * linkDatum.qstZIc[ind];

			}
			//accumulate childen's articulated zero acceleration force to parent's articulated zero acceleration
			ZA += ZIc;
			articulatedZA[link.parent] += linkDatum.childToParent * ZA;
		}
	}

	void FeatherstoneArticulation::computeJointAcceleration(ArticulationLinkData& linkDatum, ArticulationJointCoreData& jointDatum, 
		const Cm::SpatialVectorF& pMotionAcceleration, PxReal* jointAcceleration)
	{
		
		PxReal tJAccel[6];
		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
		{
			//stI * pAcceleration
			const PxReal temp = linkDatum.Is[ind].innerProduct(pMotionAcceleration);

			tJAccel[ind] = (linkDatum.qstZIc[ind] - temp);
		}

		//calculate jointAcceleration
		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
		{
			jointAcceleration[ind] = 0.f;
			for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
			{
				jointAcceleration[ind] += linkDatum.invStIs[ind2][ind] * tJAccel[ind2];
			}
			//PX_ASSERT(PxAbs(jointAcceleration[ind]) < 5000);
		}

	}

	void FeatherstoneArticulation::computeLinkAcceleration(ArticulationData& data,
		ScratchData& scratchData)
	{
		const PxU32 linkCount = data.getLinkCount();
		const PxReal dt = data.getDt();
		const bool fixBase = data.getCore()->flags & PxArticulationFlag::eFIX_BASE;
		//we have initialized motionVelocity and motionAcceleration to be zero in the root link if
		//fix based flag is raised
		Cm::SpatialVectorF* motionVelocities = scratchData.motionVelocities;
		Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;
		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;
		Cm::SpatialVectorF* spatialZAForces = scratchData.spatialZAVectors;

		if (!fixBase)
		{
			//ArticulationLinkData& baseLinkDatum = data.getLinkData(0);
			SpatialMatrix invInertia = data.mBaseInvSpatialArticulatedInertia;//baseLinkDatum.spatialArticulatedInertia.invertInertia();

#if FEATHERSTONE_DEBUG
			SpatialMatrix result = invInertia * baseLinkDatum.spatialArticulatedInertia;

			bool isIdentity = result.isIdentity();

			PX_ASSERT(isIdentity);

			PX_UNUSED(isIdentity);
#endif


			ArticulationLink& baseLink = data.getLink(0);
			const PxTransform& body2World = baseLink.bodyCore->body2World;

			motionAccelerations[0] = -(invInertia * spatialZAForces[0]);
			Cm::SpatialVectorF deltaV = motionAccelerations[0] * dt;

			//Cm::SpatialVectorF oldMotionVel = motionVelocities[0];
			Cm::SpatialVectorF oldVel = motionVelocities[0];

			motionVelocities[0].top += body2World.rotate(deltaV.top);
			motionVelocities[0].bottom += body2World.rotate(deltaV.bottom);

		}
#if FEATHERSTONE_DEBUG
		else
		{
			PX_ASSERT(isSpatialVectorZero(motionAccelerations[0]));
			PX_ASSERT(isSpatialVectorZero(motionVelocities[0]));
		}
#endif

		/*PxReal* jointAccelerations = data.getJointAccelerations();
		PxReal* jointVelocities = data.getJointVelocities();
		PxReal* jointPositions = data.getJointPositions();*/

		PxReal* jointAccelerations = scratchData.jointAccelerations;
		PxReal* jointVelocities = scratchData.jointVelocities;

		//printf("===========================\n");

		//calculate acceleration
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = data.getLink(linkID);
			ArticulationLinkData& linkDatum = data.getLinkData(linkID);

			ArticulationJointCore& joint = *link.inboundJoint;
			PX_UNUSED(joint);

			SpatialTransform p2C = linkDatum.childToParent.getTranspose();
			Cm::SpatialVectorF pMotionAcceleration = p2C * motionAccelerations[link.parent];


			ArticulationJointCoreData& jointDatum = data.getJointData(linkID);

			//calculate jointAcceleration
			PxReal* jA = &jointAccelerations[jointDatum.jointOffset];
			computeJointAcceleration(linkDatum, jointDatum, pMotionAcceleration, jA);
			//printf("jA %f\n", jA[0]);

			Cm::SpatialVectorF motionAcceleration(PxVec3(0.f), PxVec3(0.f));
			PxReal* jointVelocity = &jointVelocities[jointDatum.jointOffset];

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				PxReal jVel = jointVelocity[ind] + jA[ind] * dt;
				if (PxAbs(jVel) > joint.maxJointVelocity)
				{
					jVel = jVel < 0.f ? -joint.maxJointVelocity : joint.maxJointVelocity;
					jA[ind] = (jVel - jointVelocity[ind]) / dt;
				}

				jointVelocity[ind] = jVel;
				motionAcceleration += jointDatum.motionMatrix[ind] * jA[ind];
			}

			//KS - can we just work out velocities by projecting out the joint velocities instead of accumulating all this?
			motionAccelerations[linkID] = pMotionAcceleration + coriolisVectors[linkID] + motionAcceleration;
			PX_ASSERT(motionAccelerations[linkID].isFinite());

			const PxTransform body2World = link.bodyCore->body2World;
			//motionVelocities[linkID] += motionAccelerations[linkID] * dt;

			const Cm::SpatialVectorF deltaV = motionAccelerations[linkID] * dt;
			motionVelocities[linkID].top += body2World.rotate(deltaV.top);
			motionVelocities[linkID].bottom += body2World.rotate(deltaV.bottom);
		}


	}

	
	void FeatherstoneArticulation::computeJointTransmittedFrictionForce(
		ArticulationData& data, ScratchData& scratchData, Cm::SpatialVectorF* /*Z*/, Cm::SpatialVectorF* /*DeltaV*/)
	{
		//const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = data.getLinkCount() - 1;

		//const PxReal frictionCoefficent =30.5f;
		Cm::SpatialVectorF* transmittedForce = scratchData.spatialZAVectors;

		for (PxU32 linkID = startIndex; linkID > 1; --linkID)
		{
			ArticulationLink& link = data.getLink(linkID);
			ArticulationLinkData& linkDatum = data.getLinkData(linkID);

			//joint force transmitted from parent to child
			transmittedForce[link.parent] += linkDatum.childToParent *  transmittedForce[linkID];
		}

		transmittedForce[0] = Cm::SpatialVectorF::Zero();

		//const PxReal dt = data.getDt();
		//for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		//{
		//	//ArticulationLink& link = data.getLink(linkID);
		//	//ArticulationLinkData& linkDatum = data.getLinkData(linkID);
		//	transmittedForce[linkID] = transmittedForce[linkID] * (frictionCoefficent) * dt;
		//	//transmittedForce[link.parent] -= linkDatum.childToParent * transmittedForce[linkID];
		//}

		//

		//applyImpulses(transmittedForce, Z, DeltaV);

		//PxReal* deltaV = data.getJointDeltaVelocities();
		//PxReal* jointV = data.getJointVelocities();

		//for (PxU32 linkID = 1; linkID < data.getLinkCount(); ++linkID)
		//{
		//	ArticulationJointCoreData& tJointDatum = data.getJointData()[linkID];
		//	for (PxU32 i = 0; i < tJointDatum.dof; ++i)
		//	{
		//		jointV[i + tJointDatum.jointOffset] += deltaV[i + tJointDatum.jointOffset];
		//		deltaV[i + tJointDatum.jointOffset] = 0.f;
		//	}
		//}
	}

	//void FeatherstoneArticulation::computeJointFriction(ArticulationData& data,
	//	ScratchData& scratchData)
	//{
	//	PX_UNUSED(scratchData);
	//	const PxU32 linkCount = data.getLinkCount();
	//	PxReal* jointForces = scratchData.jointForces;
	//	PxReal* jointFrictionForces = data.getJointFrictionForces();
	//	PxReal* jointVelocities = data.getJointVelocities();

	//	const PxReal coefficient = 0.5f;

	//	for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
	//	{
	//		ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
	//		//compute generalized force
	//		PxReal* jFs = &jointForces[jointDatum.jointOffset];
	//		PxReal* jVs = &jointVelocities[jointDatum.jointOffset];
	//		PxReal* jFFs = &jointFrictionForces[jointDatum.jointOffset];

	//		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
	//		{
	//			PxReal sign = jVs[ind] > 0 ? -1.f : 1.f;
	//			jFFs[ind] = coefficient * PxAbs(jFs[ind]) *sign;

	//			//jFFs[ind] = coefficient * jVs[ind];
	//		}
	//	}
	//}


	void FeatherstoneArticulation::applyExternalImpulse(ArticulationLink* links, const PxU32 linkCount,
		const bool fixBase, ArticulationData& data, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV,
		const PxReal dt, const PxVec3& gravity, Cm::SpatialVector* acceleration)
	{
		
		PxReal* jointVelocities = data.getJointVelocities();
		PxReal* jointAccelerations = data.getJointAccelerations();
		PxReal* jointDeltaVelocities = data.getJointDeltaVelocities();

		const PxU32 totalDofs = data.getDofs();
		PxMemZero(jointDeltaVelocities, sizeof(PxReal)*totalDofs);

		ArticulationLinkData* linkData = data.getLinkData();
		ArticulationJointCoreData* jointData = data.getJointData();

		Cm::SpatialVectorF* motionVelcities = data.getMotionVelocities();

		//compute external impulse
		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];
			PxsBodyCore& core = *link.bodyCore;

			const PxTransform& body2World = core.body2World;

			Cm::SpatialVector& externalAccel = acceleration[linkID];
			const PxVec3 linkGravity = body2World.rotateInv(gravity);
			PxVec3 linearAccel = body2World.rotateInv(externalAccel.linear);
			if (!(link.body->mInternalFlags & PxsRigidBody::eDISABLE_GRAVITY))
				linearAccel += linkGravity;

			PxVec3 angularAccel = body2World.rotateInv(externalAccel.angular);

			Cm::SpatialVectorF a(angularAccel, linearAccel);

			Z[linkID] = -linkData[linkID].spatialArticulatedInertia * a * dt;

			externalAccel.linear = PxVec3(0.f); externalAccel.angular = PxVec3(0.f);
		}


		for (PxU32 linkID = PxU32(linkCount - 1); linkID > 0; --linkID)
		{
			ArticulationLink& tLink = links[linkID];
			ArticulationLinkData& tLinkDatum = linkData[linkID];
			ArticulationJointCoreData& tJointDatum = jointData[linkID];
			Z[tLink.parent] += FeatherstoneArticulation::propagateImpulse(tLinkDatum, tJointDatum, Z[linkID]);
		}


		if (fixBase)
		{
			deltaV[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
		}
		else
		{
			ArticulationLinkData& hLinkDatum = linkData[0];
		
			SpatialMatrix inverseArticulatedInertia = hLinkDatum.spatialArticulatedInertia.getInverse();

			deltaV[0] = inverseArticulatedInertia * (-Z[0]);
			motionVelcities[0] += deltaV[0];

			PX_ASSERT(motionVelcities[0].isFinite());
		}

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& tLink = links[linkID];
			ArticulationLinkData& tLinkDatum = linkData[linkID];
			ArticulationJointCoreData& tJointDatum = jointData[linkID];
			PxReal* jV = &jointDeltaVelocities[tJointDatum.jointOffset];
			deltaV[linkID] = FeatherstoneArticulation::propagateVelocity(tLinkDatum, tJointDatum, Z[linkID],
				jV, deltaV[tLink.parent]);
			motionVelcities[linkID] += deltaV[linkID];
			PX_ASSERT(motionVelcities[linkID].isFinite());
		}

		const PxReal invDt = 1 / dt;
		//update joint acceleration
		for (PxU32 i = 0; i < data.getDofs(); ++i)
		{
			jointVelocities[i] += jointDeltaVelocities[i];
			jointAccelerations[i] = jointDeltaVelocities[i] * invDt;
		}
	}


	PxU32 FeatherstoneArticulation::computeUnconstrainedVelocities(
		const ArticulationSolverDesc& desc,
		PxReal dt,
		PxConstraintAllocator& allocator,
		PxSolverConstraintDesc* constraintDesc,
		PxU32& acCount,
		const PxVec3& gravity, PxU64 contextID,
		Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV)
	{
		PX_UNUSED(contextID);
		PX_UNUSED(desc);
		FeatherstoneArticulation* articulation = static_cast<FeatherstoneArticulation*>(desc.articulation);
		ArticulationData& data = articulation->mArticulationData;
		data.setDt(dt);

		return articulation->computeUnconstrainedVelocitiesInternal(desc, allocator, constraintDesc,
			acCount, gravity, Z, deltaV);
	}


	void FeatherstoneArticulation::computeUnconstrainedVelocitiesTGS(
		const ArticulationSolverDesc& desc,
		PxReal dt, const PxVec3& gravity,
		PxU64 contextID, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV)
	{
		PX_UNUSED(contextID);

		FeatherstoneArticulation* articulation = static_cast<FeatherstoneArticulation*>(desc.articulation);
		ArticulationData& data = articulation->mArticulationData;
		data.setDt(dt);

		return articulation->computeUnconstrainedVelocitiesTGSInternal(gravity, Z, DeltaV);
	}


	//void FeatherstoneArticulation::computeCounteractJointForce(const ArticulationSolverDesc& desc, ScratchData& /*scratchData*/, const PxVec3& gravity)
	//{
	//	const bool fixBase = mArticulationData.getCore()->flags & PxArticulationFlag::eFIX_BASE;

	//	const PxU32 linkCount = mArticulationData.getLinkCount();
	//	const PxU32 totalDofs = mArticulationData.getDofs();
	//	//common data
	//	computeRelativeTransform(mArticulationData);

	//	jcalc(mArticulationData);

	//	computeSpatialInertia(mArticulationData);

	//	DyScratchAllocator allocator(desc.scratchMemory, desc.scratchMemorySize);

	//	ScratchData tempScratchData;
	//	allocateScratchSpatialData(allocator, linkCount, tempScratchData);

	//	//PxReal* gravityJointForce = allocator.alloc<PxReal>(totalDofs);
	//	//{
	//	//	PxMemZero(gravityJointForce, sizeof(PxReal) * totalDofs);

	//	//	//compute joint force due to gravity
	//	//	tempScratchData.jointVelocities = NULL;
	//	//	tempScratchData.jointAccelerations = NULL;
	//	//	tempScratchData.jointForces = gravityJointForce;
	//	//	tempScratchData.externalAccels = NULL;

	//	//	if (fixBase)
	//	//		inverseDynamic(mArticulationData, gravity,tempScratchData);
	//	//	else
	//	//		inverseDynamicFloatingLink(mArticulationData, gravity, tempScratchData);
	//	//}

	//	////PxReal* jointForce = mArticulationData.getJointForces();
	//	//PxReal* tempJointForce = mArticulationData.getTempJointForces();
	//	//{
	//	//	PxMemZero(tempJointForce, sizeof(PxReal) * totalDofs);

	//	//	//compute joint force due to coriolis force
	//	//	tempScratchData.jointVelocities = mArticulationData.getJointVelocities();
	//	//	tempScratchData.jointAccelerations = NULL;
	//	//	tempScratchData.jointForces = tempJointForce;
	//	//	tempScratchData.externalAccels = NULL;

	//	//	if (fixBase)
	//	//		inverseDynamic(mArticulationData, PxVec3(0.f), tempScratchData);
	//	//	else
	//	//		inverseDynamicFloatingLink(mArticulationData, PxVec3(0.f), tempScratchData);
	//	//}

	//	//PxReal* jointForce = mArticulationData.getJointForces();
	//	//for (PxU32 i = 0; i < mArticulationData.getDofs(); ++i)
	//	//{
	//	//	jointForce[i] = tempJointForce[i] - gravityJointForce[i];
	//	//}

	//	//PxReal* jointForce = mArticulationData.getJointForces();
	//	PxReal* tempJointForce = mArticulationData.getTempJointForces();
	//	{
	//		PxMemZero(tempJointForce, sizeof(PxReal) * totalDofs);

	//		//compute joint force due to coriolis force
	//		tempScratchData.jointVelocities = mArticulationData.getJointVelocities();
	//		tempScratchData.jointAccelerations = NULL;
	//		tempScratchData.jointForces = tempJointForce;
	//		tempScratchData.externalAccels = mArticulationData.getExternalAccelerations();

	//		if (fixBase)
	//			inverseDynamic(mArticulationData, gravity, tempScratchData);
	//		else
	//			inverseDynamicFloatingLink(mArticulationData, gravity, tempScratchData);
	//	}

	//	PxReal* jointForce = mArticulationData.getJointForces();
	//	for (PxU32 i = 0; i < mArticulationData.getDofs(); ++i)
	//	{
	//		jointForce[i] = tempJointForce[i];
	//	}
	//}

	void FeatherstoneArticulation::updateArticulation(ScratchData& scratchData,
		const PxVec3& gravity, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV)
	{
		computeRelativeTransformC2P(mArticulationData);

		computeLinkVelocities(mArticulationData, scratchData);

		initLinks(mArticulationData, gravity, scratchData, Z, DeltaV);

		computeLinkAcceleration(mArticulationData, scratchData);

		//KS - force a recompute of link velocities
		////computeLinkVelocities(mArticulationData, scratchData);
		//{
		//	ArticulationData& data = mArticulationData;
		//	ArticulationLink* links = data.getLinks();
		//	ArticulationLinkData* linkData = data.getLinkData();
		//	const PxU32 linkCount = data.getLinkCount();
		//	const bool fixBase = data.getCore()->flags & PxArticulationFlag::eFIX_BASE;

		//	//motion velocities has to be in world space to avoid numerical errors caused by space 
		//	Cm::SpatialVectorF* motionVelocities = scratchData.motionVelocities;
		//	//Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;

		//	PxReal* jointVelocities = scratchData.jointVelocities;

		//	ArticulationLink& baseLink = links[0];
		//	ArticulationLinkData& baseLinkDatum = linkData[0];

		//	PxsBodyCore& core0 = *baseLink.bodyCore;

		//	baseLinkDatum.maxPenBias = core0.maxPenBias;

		//	for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		//	{
		//		ArticulationLink& link = links[linkID];
		//		ArticulationLinkData& linkDatum = linkData[linkID];
		//		PxsBodyCore& bodyCore = *link.bodyCore;

		//		linkDatum.maxPenBias = bodyCore.maxPenBias;

		//		//SpatialTransform p2c = linkDatum.childToParent.getTranspose();

		//		//motionVelocites[linkID] = p2c * motionVelocites[link.parent];
		//		//motionVelocites[linkID] = motionVelocites[link.parent];
		//		/*motionVelocities[linkID].top = motionVelocities[link.parent].top;
		//		motionVelocities[linkID].bottom = motionVelocities[link.parent].bottom + motionVelocities[link.parent].top.cross(linkDatum.rw);*/

		//		const PxTransform& body2World = bodyCore.body2World;
		//		Cm::SpatialVectorF deltaVel;
		//		deltaVel.top = body2World.rotateInv(motionVelocities[linkID].top - motionVelocities[link.parent].top);
		//		deltaVel.bottom = body2World.rotateInv(motionVelocities[linkID].bottom - (motionVelocities[link.parent].bottom + motionVelocities[link.parent].top.cross(linkDatum.rw)));

		//		PxVec3 ang = motionVelocities[link.parent].top;
		//		PxVec3 lin = motionVelocities[link.parent].bottom + motionVelocities[link.parent].top.cross(linkDatum.rw);
		//		

		//		if (jointVelocities)
		//		{
		//			ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
		//			//const PxReal* const jV = &jointVelocity[linkID * 6];
		//			//PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];

		//			Cm::SpatialVectorF deltaV = Cm::SpatialVectorF::Zero();
		//			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
		//			{
		//				/*if (jVelocity[ind] < -100.f)
		//					jVelocity[ind] = -100.f;
		//				else if (jVelocity[ind] > 100.f)
		//					jVelocity[ind] = 100.f;*/
		//				
		//				PxReal jointVel = Cm::SpatialVectorF(PxVec3(jointDatum.jointAxis[ind][0], jointDatum.jointAxis[ind][1], jointDatum.jointAxis[ind][2]),
		//					PxVec3(jointDatum.jointAxis[ind][3], jointDatum.jointAxis[ind][4], jointDatum.jointAxis[ind][5])).dot(deltaVel);

		//				//PxReal diff = jointVel - jVelocity[ind];
		//				//PX_ASSERT(PxAbs(diff) < 1e-3f);
		//				
		//				deltaV += jointDatum.motionMatrix[ind] * jointVel;
		//				//jVelocity[ind] = jointVel;
		//			}

		//			ang += body2World.rotate(deltaV.top);
		//			lin += body2World.rotate(deltaV.bottom);
		//		}

		//		PxVec3 angDiff = ang - motionVelocities[linkID].top;
		//		PxVec3 linDiff = lin - motionVelocities[linkID].bottom;

		//		motionVelocities[linkID] = Cm::SpatialVectorF(ang, lin);
		//	}
		//}
	}


	PxU32 FeatherstoneArticulation::computeUnconstrainedVelocitiesInternal(
		const ArticulationSolverDesc& desc,
		PxConstraintAllocator& allocator,
		PxSolverConstraintDesc* constraintDesc,
		PxU32& acCount,
		const PxVec3& gravity,
		Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV)
	{
		PX_PROFILE_ZONE("Articulations:computeUnconstrainedVelocities", 0);

		PX_UNUSED(desc);
		ArticulationLink* links = mArticulationData.getLinks();
		const PxU32 linkCount = mArticulationData.getLinkCount();
		const bool fixBase = mArticulationData.getCore()->flags & PxArticulationFlag::eFIX_BASE;

		mArticulationData.init();

		jcalc(mArticulationData);

		ScratchData scratchData;
		scratchData.motionVelocities = mArticulationData.getMotionVelocities();
		scratchData.motionAccelerations = mArticulationData.getMotionAccelerations();
		scratchData.coriolisVectors = mArticulationData.getCorioliseVectors();
		scratchData.spatialZAVectors = mArticulationData.getSpatialZAVectors();
		scratchData.jointAccelerations = mArticulationData.getJointAccelerations();
		scratchData.jointVelocities = mArticulationData.getJointVelocities();
		scratchData.jointPositions = mArticulationData.getJointPositions();
		scratchData.jointForces = mArticulationData.getJointForces();
		scratchData.externalAccels = mArticulationData.getExternalAccelerations();

		updateArticulation(scratchData, gravity, Z, DeltaV);

		//use individual zero acceleration force(we copy the initial Z value to the transmitted force buffers in initLink())
		scratchData.spatialZAVectors = mArticulationData.getTransmittedForces();
		computeZAForceInv(mArticulationData, scratchData);
		computeJointTransmittedFrictionForce(mArticulationData, scratchData, Z, DeltaV);

		//the dirty flag is used in inverse dynamic
		mArticulationData.setDataDirty(true);

		//zero zero acceleration vector in the articulation data so that we can use this buffer to accumulated
		//impulse for the contacts/constraints in the PGS/TGS solvers
		PxMemZero(mArticulationData.getSpatialZAVectors(), sizeof(Cm::SpatialVectorF) * linkCount);

		// solver progress counters
		maxSolverNormalProgress = 0;
		maxSolverFrictionProgress = 0;
		solverProgress = 0;
		numTotalConstraints = 0;

		for (PxU32 a = 0; a < mArticulationData.getLinkCount(); ++a)
		{
			mArticulationData.mAccumulatedPoses[a] = mArticulationData.getLink(a).bodyCore->body2World;
			mArticulationData.mPreTransform[a] = mArticulationData.getLink(a).bodyCore->body2World;
			mArticulationData.mDeltaQ[a] = PxQuat(PxIdentity);
		}

		return setupSolverConstraints(allocator, constraintDesc, links, linkCount,
			fixBase, mArticulationData, Z, acCount);
	}


	void FeatherstoneArticulation::computeUnconstrainedVelocitiesTGSInternal(const PxVec3& gravity,
		Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV)
	{
		PX_PROFILE_ZONE("Articulations:computeUnconstrainedVelocitiesSS", 0);
		const PxU32 linkCount = mArticulationData.getLinkCount();

		
		mArticulationData.init();

		jcalc(mArticulationData);

		ScratchData scratchData;
		scratchData.motionVelocities = mArticulationData.getMotionVelocities();
		scratchData.motionAccelerations = mArticulationData.getMotionAccelerations();
		scratchData.coriolisVectors = mArticulationData.getCorioliseVectors();
		scratchData.spatialZAVectors = mArticulationData.getSpatialZAVectors();
		scratchData.jointAccelerations = mArticulationData.getJointAccelerations();
		scratchData.jointVelocities = mArticulationData.getJointVelocities();
		scratchData.jointPositions = mArticulationData.getJointPositions();
		scratchData.jointForces = mArticulationData.getJointForces();
		scratchData.externalAccels = mArticulationData.getExternalAccelerations();

		updateArticulation(scratchData, gravity, Z, DeltaV);


		scratchData.spatialZAVectors = mArticulationData.getTransmittedForces();
		computeZAForceInv(mArticulationData, scratchData);
		computeJointTransmittedFrictionForce(mArticulationData, scratchData, Z, DeltaV);


		//zero zero acceleration vector in the articulation data so that we can use this buffer to accumulated
		//impulse for the contacts/constraints in the PGS/TGS solvers
		PxMemZero(mArticulationData.getSpatialZAVectors(), sizeof(Cm::SpatialVectorF) * linkCount);

		//the dirty flag is used in inverse dynamic
		mArticulationData.setDataDirty(true);

		for (PxU32 a = 0; a < mArticulationData.getLinkCount(); ++a)
		{
			mArticulationData.mAccumulatedPoses[a] = mArticulationData.getLink(a).bodyCore->body2World;
			mArticulationData.mPreTransform[a] = mArticulationData.getLink(a).bodyCore->body2World;
			mArticulationData.mDeltaQ[a] = PxQuat(PxIdentity);
		}
		mArticulationData.mAccumulatedDt = 0.f;
	}

	void FeatherstoneArticulation::enforcePrismaticLimits(PxReal* jPosition, ArticulationJointCore* joint)
	{
		if (joint->prismaticLimited)
		{
			if (jPosition[0] < (joint->limits[joint->dofIds[0]].low))
				jPosition[0] = joint->limits[joint->dofIds[0]].low;

			if (jPosition[0] > (joint->limits[joint->dofIds[0]].high))
				jPosition[0] = joint->limits[joint->dofIds[0]].high;
		}
	}

	PxQuat computeSphericalJointPositions(ArticulationJointCore* joint,
		const PxQuat newRot, const PxQuat pBody2WorldRot,
		PxReal* jPositions)
	{

		PxQuat newParentToChild = (newRot.getConjugate() * pBody2WorldRot).getNormalized();

		//PxQuat newQ = (pBody2WorldRot * newParentToChild.getConjugate()).getNormalized();

		const PxQuat cA2w = pBody2WorldRot * joint->parentPose.q;
		PxQuat cB2w = newRot * joint->childPose.q;

		if (cA2w.dot(cB2w)<0.0f)	// minimum dist quat (equiv to flipping cB2bB.q, which we don't use anywhere)
			cB2w = -cB2w;

		const PxQuat cB2cA = cA2w.getConjugate() * cB2w;

		PxQuat twist; //x
		PxQuat swing1;//y
		PxQuat swing2;//z
		separateSwingTwist(cB2cA, twist, swing1, swing2);
		//tan(t / 2) = sin(t) / (1 + cos t), so this is the quarter angle

		const PxReal theta0 = PxAtan2(twist.x, (1.f + twist.w))  * 4.f;
		const PxReal theta1 = PxAtan2(swing1.y, (1.f + swing1.w)) * 4.f;
		const PxReal theta2 = PxAtan2(swing2.z, (1.f + swing2.w)) * 4.f;		

		
		PxU32 dofIndex = 0;
		if (joint->motion[PxArticulationAxis::eTWIST] != PxArticulationMotion::eLOCKED)
			jPositions[dofIndex++] = theta0;
		if (joint->motion[PxArticulationAxis::eSWING1] != PxArticulationMotion::eLOCKED)
			jPositions[dofIndex++] = theta1;
		if (joint->motion[PxArticulationAxis::eSWING2] != PxArticulationMotion::eLOCKED)
			jPositions[dofIndex++] = theta2;
		if (joint->motion[PxArticulationAxis::eTWIST] == PxArticulationMotion::eLOCKED)
			jPositions[dofIndex++] = theta0;
		if (joint->motion[PxArticulationAxis::eSWING1] == PxArticulationMotion::eLOCKED)
			jPositions[dofIndex++] = theta1;
		if (joint->motion[PxArticulationAxis::eSWING2] == PxArticulationMotion::eLOCKED)
			jPositions[dofIndex++] = theta2;
	
		return newParentToChild;
	}

	void FeatherstoneArticulation::computeAndEnforceJointPositions(ArticulationData& data, PxReal* jointPositions)
	{
		ArticulationLink* links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();

		ArticulationJointCoreData* jointData = data.getJointData();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];
			ArticulationJointCore* joint = link.inboundJoint;
			ArticulationJointCoreData& jointDatum = jointData[linkID];
			PxReal* jPositions = &jointPositions[jointDatum.jointOffset];

			if (joint->jointType == PxArticulationJointType::eSPHERICAL)
			{
				ArticulationLink& pLink = links[link.parent];
				//const PxTransform pBody2World = pLink.bodyCore->body2World;

				computeSphericalJointPositions(joint, link.bodyCore->body2World.q, 
					pLink.bodyCore->body2World.q, jPositions);
			}
			else if (joint->jointType == PxArticulationJointType::eREVOLUTE)
			{
				/*if (jPositions[0] < -PxPi)
					jPositions[0] += PxTwoPi;
				else if (jPositions[0] > PxPi)
					jPositions[0] -= PxTwoPi;*/

				PxReal jPos = jPositions[0];

				if (jPos > PxTwoPi)
					jPos -= PxTwoPi*2.f;
				else if (jPos < -PxTwoPi)
					jPos += PxTwoPi*2.f;

				jPos = PxClamp(jPos, -PxTwoPi*2.f, PxTwoPi*2.f);

				jPositions[0] = jPos;
			}
			else if(joint->jointType == PxArticulationJointType::ePRISMATIC)
			{
				enforcePrismaticLimits(jPositions, joint);
			}
		}
	}

	void FeatherstoneArticulation::propagateLinksDown(ArticulationData& data, PxReal* jointDeltaVelocities, PxReal* jointPositions,
		Cm::SpatialVectorF* motionVelocities)
	{
		ArticulationLink* links = mArticulationData.getLinks();

		PxTransform* preTransforms = mArticulationData.getPreTransform();
		PX_UNUSED(preTransforms);
		PX_UNUSED(motionVelocities);

		ArticulationJointCoreData* jointData = mArticulationData.getJointData();

		const PxU32 linkCount = mArticulationData.getLinkCount();

		const PxReal dt = data.getDt();

		PxReal* jointVelocities = data.getJointVelocities();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];

			ArticulationJointCoreData& jointDatum = jointData[linkID];
			//ArticulationLinkData& linkDatum = mArticulationData.getLinkData(linkID);
			//const PxTransform oldTransform = preTransforms[linkID];

			ArticulationLink& pLink = links[link.parent];
			const PxTransform pBody2World = pLink.bodyCore->body2World;

			ArticulationJointCore* joint = link.inboundJoint;

			PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
			PxReal* jDeltaVelocity = &jointDeltaVelocities[jointDatum.jointOffset];
			PxReal* jPosition = &jointPositions[jointDatum.jointOffset];

			PxQuat newParentToChild;
			PxQuat newWorldQ;
			PxVec3 r;


			const PxVec3 childOffset = -joint->childPose.p;
			const PxVec3 parentOffset = joint->parentPose.p;

			PxTransform& body2World = link.bodyCore->body2World;


			switch (joint->jointType)
			{
			case PxArticulationJointType::ePRISMATIC:
			{
				const PxReal delta = (jVelocity[0] + jDeltaVelocity[0]) * dt;

				jPosition[0] += delta;

				enforcePrismaticLimits(jPosition, joint);

				newParentToChild = joint->relativeQuat;
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				const PxVec3& u = jointDatum.motionMatrix[0].bottom;

				r = e + d + u * jPosition[0];
				break;
			}
			case PxArticulationJointType::eREVOLUTE:
			{
				//use positional iteration JointVelociy to integrate
				const PxReal delta = (jVelocity[0] + jDeltaVelocity[0]) * dt;

				PxReal jPos = jPosition[0] + delta;

				if (jPos > PxTwoPi)
					jPos -= PxTwoPi*2.f;
				else if (jPos < -PxTwoPi)
					jPos += PxTwoPi*2.f;

				jPos = PxClamp(jPos, -PxTwoPi*2.f, PxTwoPi*2.f);
				jPosition[0] = jPos;

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
				if (jointDatum.dof < 3)
				{
					newParentToChild = PxQuat(PxIdentity);
					//We are simulating a revolute or 2d joint, so just integrate quaternions and joint positions as above...
					for (PxU32 i = 0; i < jointDatum.dof; ++i)
					{
						const PxReal delta = (jVelocity[i] + jDeltaVelocity[i]) * dt;

						PxReal jPos = jPosition[i] + delta;

						if (jPos > PxTwoPi)
							jPos -= PxTwoPi*2.f;
						else if (jPos < -PxTwoPi)
							jPos += PxTwoPi*2.f;

						jPos = PxClamp(jPos, -PxTwoPi*2.f, PxTwoPi*2.f);
						jPosition[i] = jPos;

						const PxVec3& u = jointDatum.motionMatrix[i].top;

						PxQuat jointRotation = PxQuat(-jPosition[i], u);
						if (jointRotation.w < 0)	//shortest angle.
							jointRotation = -jointRotation;

						newParentToChild = newParentToChild * (jointRotation * joint->relativeQuat).getNormalized();
					}
					const PxVec3 e = newParentToChild.rotate(parentOffset);
					const PxVec3 d = childOffset;
					r = e + d;

					PX_ASSERT(r.isFinite());

				}
				else
				{

					const PxTransform oldTransform = preTransforms[linkID];

					PxVec3 worldAngVel = motionVelocities[linkID].top;

					newWorldQ = Ps::exp(worldAngVel*dt) * oldTransform.q;

					newParentToChild = computeSphericalJointPositions(joint, newWorldQ,
						pBody2World.q, jPosition);

					const PxVec3 e = newParentToChild.rotate(parentOffset);
					const PxVec3 d = childOffset;
					r = e + d;
				}

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

			
			body2World.q = (pBody2World.q * newParentToChild.getConjugate()).getNormalized();
			body2World.p = pBody2World.p + body2World.q.rotate(r);


			PX_ASSERT(body2World.isSane());
			PX_ASSERT(body2World.isValid());
			
		}
	}

	void FeatherstoneArticulation::updateBodies(const ArticulationSolverDesc& desc, PxReal dt)
	{
		updateBodies(desc, dt, true);
	}

	void FeatherstoneArticulation::updateBodiesTGS(const ArticulationSolverDesc& desc, PxReal dt)
	{
		updateBodies(desc, dt, false);
	}

	void FeatherstoneArticulation::updateBodies(const ArticulationSolverDesc& desc, PxReal dt, bool integrateJointPositions)
	{
		FeatherstoneArticulation* articulation = static_cast<FeatherstoneArticulation*>(desc.articulation);
		ArticulationData& data = articulation->mArticulationData;
		ArticulationLink* links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();

		Cm::SpatialVectorF* motionVelocities = data.getMotionVelocities();

		Cm::SpatialVector* externalAccels = data.getExternalAccelerations();
		Cm::SpatialVector zero = Cm::SpatialVector::zero();

		if (integrateJointPositions)
			data.setDt(dt);
		else
			data.setDt(0.f);

		
		
		PxTransform* preTransforms = data.getPreTransform();

		if (articulation->mHasSphericalJoint)
		{
			for (PxU32 i = 0; i < linkCount; ++i)
			{
				ArticulationLink& link = links[i];

				PxsBodyCore* bodyCore = link.bodyCore;

				//record link's previous transform
				preTransforms[i] = bodyCore->body2World;
			}
		}

		const bool fixBase = data.getCore()->flags & PxArticulationFlag::eFIX_BASE;

		ArticulationLink& baseLink = links[0];

		PxsBodyCore* baseBodyCore = baseLink.bodyCore;

		if (!integrateJointPositions)
		{
			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				links[linkID].bodyCore->body2World = data.mAccumulatedPoses[linkID].getNormalized();
			}

			articulation->computeAndEnforceJointPositions(data, data.getJointPositions());
		}
		else
		{

			if (!fixBase)
			{

				//body2World store new body transform integrated from solver linear/angular velocity

				ArticulationLink& link = links[0];

				const PxTransform& preTrans = link.bodyCore->body2World;

				Cm::SpatialVectorF& posVel = data.getPosIterMotionVelocity(0);

				updateRootBody(posVel, preTrans, data, dt);
			}
			else if(!integrateJointPositions)
			{
				baseBodyCore->body2World = data.mAccumulatedPoses[0];
			}

			//using the original joint velocities and delta velocities changed in the positional iter to update joint position/body transform
			articulation->propagateLinksDown(data, data.getPosIterJointDeltaVelocities(), data.getJointPositions(), data.getPosIterMotionVelocities());
		}

		//update joint velocities/accelerations due to contacts/constraints
		if (data.mJointDirty && integrateJointPositions)
		{
			//update delta joint velocity and motion velocity due to velocity iteration changes
			Cm::SpatialVectorF deltaV[64];

			PxcFsFlushVelocity(*articulation, deltaV);

			//update joint velocity/accelerations
			PxReal* jointVelocities = data.getJointVelocities();
			PxReal* jointAccelerations = data.getJointAccelerations();
			PxReal* jointDeltaVelocities = data.getJointDeltaVelocities();
			const PxU32 totalDofs = data.getDofs();
			const PxReal invDt = 1.f / dt;
			for (PxU32 i = 0; i < totalDofs; ++i)
			{
				jointVelocities[i] += jointDeltaVelocities[i];
				jointAccelerations[i] += jointDeltaVelocities[i] * invDt;
			}
		}

		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];
			PxsBodyCore* bodyCore = link.bodyCore;

			bodyCore->linearVelocity = motionVelocities[linkID].bottom;
			bodyCore->angularVelocity = motionVelocities[linkID].top;
			//zero external accelerations
			externalAccels[linkID] = zero;
		}
	}


	//void FeatherstoneArticulation::updateBodies(const ArticulationSolverDesc& desc, PxReal dt, bool integrateJointPositions)
	//{
	//	PX_PROFILE_ZONE("Articulations:updateBodies", 0);
	//	FeatherstoneArticulation* articulation = static_cast<FeatherstoneArticulation*>(desc.articulation);
	//	ArticulationData& data = articulation->mArticulationData;
	//	ArticulationLink* links = data.getLinks();
	//	const PxU32 linkCount = data.getLinkCount();

	//	Cm::SpatialVectorF* motionVelocities = data.getMotionVelocities();
	//	Cm::SpatialVector* externalAccels = data.getExternalAccelerations();
	//	Cm::SpatialVector zero = Cm::SpatialVector::zero();

	//	if (data.mJointDirty || (!integrateJointPositions))
	//	{
	//		//printf("============collision change joint velocity============\n");
	//		
	//		if (data.mJointDirty)
	//		{
	//			Cm::SpatialVectorF deltaV[64];

	//			PxcFsFlushVelocity(*articulation, deltaV);
	//		}

	//		PxTransform* preTransforms = data.getPreTransform();

	//		if (integrateJointPositions)
	//			data.setDt(dt);
	//		else
	//			data.setDt(0.f);

	//		//update joint velocity/accelerations due to contact
	//		PxReal* jointVelocities = data.getJointVelocities();
	//		PxReal* jointAccelerations = data.getJointAccelerations();
	//		PxReal* jointDeltaVelocities = data.getJointDeltaVelocities();
	//		const PxU32 totalDofs = data.getDofs();
	//		const PxReal invDt = 1.f / dt;
	//		for (PxU32 i = 0; i < totalDofs; ++i)
	//		{
	//			jointVelocities[i] += jointDeltaVelocities[i];
	//			jointAccelerations[i] += jointDeltaVelocities[i] * invDt;
	//			//printf("jV %f\n", jointVelocities[i]);
	//		}

	//		if (articulation->mHasSphericalJoint)
	//		{
	//			for (PxU32 i = 0; i < linkCount; ++i)
	//			{
	//				ArticulationLink& link = links[i];

	//				PxsBodyCore* bodyCore = link.bodyCore;

	//				//record link's previous transform
	//				preTransforms[i] = bodyCore->body2World;
	//			}
	//		}

	//		const bool fixBase = data.getCore()->flags & PxArticulationFlag::eFIX_BASE;

	//		ArticulationLink& baseLink = links[0];

	//		PxsBodyCore* baseBodyCore = baseLink.bodyCore;

	//		if (!fixBase)
	//		{

	//			

	//			//body2World store new body transform integrated from solver linear/angular velocity
	//			if (integrateJointPositions)
	//			{
	//				ArticulationLink& link = links[0];

	//				const PxTransform& preTrans = link.bodyCore->body2World;

	//				Cm::SpatialVectorF& posVel = data.getPosIterMotionVelocity(0);

	//				updateRootBody(posVel, preTrans, data, dt);
	//			}
	//			else
	//			{
	//				baseBodyCore->body2World = data.mAccumulatedPoses[0];
	//			}

	//			Cm::SpatialVectorF& motionVelocity = motionVelocities[0];
	//			PX_ASSERT(motionVelocity.top.isFinite());
	//			PX_ASSERT(motionVelocity.bottom.isFinite());

	//			baseBodyCore->linearVelocity = motionVelocity.bottom;
	//			baseBodyCore->angularVelocity = motionVelocity.top;
	//		}


	//		articulation->propagateLinksDown(data, data.getPosIterJointDeltaVelocities(), data.getJointPositions(), data.getPosIterMotionVelocities());

	//		//zero all the external accelerations
	//		externalAccels[0] = zero;

	//		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
	//		{
	//			ArticulationLink& link = links[linkID];
	//			PxsBodyCore* bodyCore = link.bodyCore;

	//			bodyCore->linearVelocity = motionVelocities[linkID].bottom;
	//			bodyCore->angularVelocity = motionVelocities[linkID].top;

	//			externalAccels[linkID] = zero;
	//		}
	//	}
	//	else
	//	{
	//		for (PxU32 i = 0; i < linkCount; ++i)
	//		{
	//			ArticulationLink& link = links[i];

	//			PxsBodyCore* bodyCore = link.bodyCore;

	//			//record link's unconstrainted integrated transform
	//			bodyCore->body2World = data.mTempData.mLinkTransform[i];

	//		}

	//		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
	//		{
	//			ArticulationLink& link = links[linkID];
	//			PxsBodyCore* bodyCore = link.bodyCore;

	//			bodyCore->linearVelocity = motionVelocities[linkID].bottom;
	//			bodyCore->angularVelocity = motionVelocities[linkID].top;
	//			
	//			//zero all the external accelerations
	//			externalAccels[linkID] = zero;
	//		
	//		}
	//	}

	//}

	void FeatherstoneArticulation::updateRootBody(const Cm::SpatialVectorF& motionVelocity, 
		const PxTransform& preTransform, ArticulationData& data, const PxReal dt)
	{
		ArticulationLink* links = data.getLinks();
		//body2World store new body transform integrated from solver linear/angular velocity

		PX_ASSERT(motionVelocity.top.isFinite());
		PX_ASSERT(motionVelocity.bottom.isFinite());

		ArticulationLink& baseLink = links[0];

		PxsBodyCore* baseBodyCore = baseLink.bodyCore;

		//(1) project the current body's velocity (based on its pre-pose) to the geometric COM that we're integrating around...

		PxVec3 comLinVel = motionVelocity.bottom;

		//using the position iteration motion velocity to compute the body2World
		PxVec3 newP = (preTransform.p) + comLinVel * dt;

		PxQuat deltaQ = Ps::exp(motionVelocity.top*dt);

		baseBodyCore->body2World = PxTransform(newP, (deltaQ* preTransform.q).getNormalized());

		//ML: we are going to store the motionVelocity back later
	/*	baseBodyCore->linearVelocity = motionVelocity.bottom;
		baseBodyCore->angularVelocity = motionVelocity.top;*/

		PX_ASSERT(baseBodyCore->body2World.isFinite() && baseBodyCore->body2World.isValid());
	}

	void FeatherstoneArticulation::updateBodies()
	{
		ArticulationData& data = mArticulationData;
		const PxReal dt = data.getDt();

		ArticulationLink* links = data.getLinks();

		Cm::SpatialVectorF* motionVelocities = data.getMotionVelocities();
		PxTransform* preTransforms = data.getPreTransform();

		const PxU32 linkCount = data.getLinkCount();

		if (mHasSphericalJoint)
		{
			for (PxU32 i = 0; i < linkCount; ++i)
			{
				ArticulationLink& link = links[i];

				PxsBodyCore* bodyCore = link.bodyCore;

				//record link's previous transform
				PX_ASSERT(bodyCore->body2World.isFinite() && bodyCore->body2World.isValid());
				preTransforms[i] = bodyCore->body2World;
			}
		}

		const bool fixBase = data.getCore()->flags & PxArticulationFlag::eFIX_BASE;

		if (!fixBase)
		{
			ArticulationLink& link = links[0];

			const PxTransform& preTrans = link.bodyCore->body2World;
			
			Cm::SpatialVectorF& motionVelocity = motionVelocities[0];

			updateRootBody(motionVelocity, preTrans, data, dt);
		}

		propagateLinksDown(data, data.getJointVelocities(), data.getJointPositions(), data.getMotionVelocities());
	}

	void FeatherstoneArticulation::getJointAcceleration(const PxVec3& gravity, PxArticulationCache& cache)
	{

		if (mArticulationData.getDataDirty())
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Articulation::getJointAcceleration() commonInit need to be called first to initialize data!");
			return;
		}

		const PxU32 linkCount = mArticulationData.getLinkCount();
		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);

		ScratchData scratchData;
		PxU8* tempMemory = allocateScratchSpatialData(allocator, linkCount, scratchData);

		scratchData.jointVelocities = cache.jointVelocity;
		scratchData.jointForces = cache.jointForce;
	
		computeLinkVelocities(mArticulationData, scratchData);

		//compute individual link's spatial inertia tensor
		//[0, M]
		//[I, 0]
		computeSpatialInertia(mArticulationData);

		//compute inidividual zero acceleration force
		computeZ(mArticulationData, gravity, scratchData);

		computeArticulatedSpatialInertia(mArticulationData);

		//compute corolis and centrifugal force
		computeC(mArticulationData, scratchData);

		computeArticulatedSpatialZ(mArticulationData, scratchData);


		const bool fixBase = mArticulationData.getCore()->flags & PxArticulationFlag::eFIX_BASE;
		//we have initialized motionVelocity and motionAcceleration to be zero in the root link if
		//fix based flag is raised
		ArticulationLinkData& baseLinkDatum = mArticulationData.getLinkData(0);

		Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;
		Cm::SpatialVectorF* spatialZAForces = scratchData.spatialZAVectors;
		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;

		if (!fixBase)
		{
			SpatialMatrix inverseArticulatedInertia = baseLinkDatum.spatialArticulatedInertia.getInverse();
			motionAccelerations[0] = -(inverseArticulatedInertia * spatialZAForces[0]);
		}
#if FEATHERSTONE_DEBUG
		else
		{
			PX_ASSERT(isSpatialVectorZero(motionAccelerations[0]));
		}
#endif

		PxReal* jointAccelerations = cache.jointAcceleration;
		//calculate acceleration
		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = mArticulationData.getLink(linkID);

			ArticulationLinkData& linkDatum = mArticulationData.getLinkData(linkID);

			//SpatialTransform p2C = linkDatum.childToParent.getTranspose();
			Cm::SpatialVectorF pMotionAcceleration = linkDatum.childToParent.transposeTransform(motionAccelerations[link.parent]);

			ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);
			//calculate jointAcceleration
			PxReal* jA = &jointAccelerations[jointDatum.jointOffset];
			computeJointAcceleration(linkDatum, jointDatum, pMotionAcceleration, jA);

			Cm::SpatialVectorF motionAcceleration(PxVec3(0.f), PxVec3(0.f));

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				motionAcceleration += jointDatum.motionMatrix[ind] * jA[ind];
			}

			motionAccelerations[linkID] = pMotionAcceleration + coriolisVectors[linkID] + motionAcceleration;
			PX_ASSERT(motionAccelerations[linkID].isFinite());
		}

		allocator->free(tempMemory);
	}

}//namespace Dy
}
