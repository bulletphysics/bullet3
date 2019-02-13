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


#ifndef PX_PHYSICS_SCP_CONTACTSTREAM
#define PX_PHYSICS_SCP_CONTACTSTREAM

#include "foundation/Px.h"
#include "PxSimulationEventCallback.h"
#include "ScObjectIDTracker.h"
#include "ScRigidSim.h"
#include "ScStaticSim.h"
#include "ScBodySim.h"

namespace physx
{
	class PxShape;

namespace Sc
{
	class ActorPair;


	// Internal counterpart of PxContactPair
	struct ContactShapePair
	{
	public:
		PxShape*				shapes[2];
		const PxU8*				contactPatches;
		const PxU8*				contactPoints;
		const PxReal*			contactForces;
		PxU32					requiredBufferSize;
		PxU8					contactCount;
		PxU8					patchCount;
		PxU16					constraintStreamSize;
		PxU16					flags;
		PxU16					events;
		PxU32					shapeID[2];
		//26 (or 38 on 64bit)
	};
	PX_COMPILE_TIME_ASSERT(sizeof(ContactShapePair) == sizeof(PxContactPair));

	struct ContactStreamManagerFlag
	{
		enum Enum
		{
			/**
			\brief Need to test stream for shapes that were removed from the actor/scene

			Usually this is the case when a shape gets removed from the scene, however, other operations that remove the
			broadphase volume of a pair object have to be considered as well since the shape might get removed later after such an
			operation. The scenarios to consider are:

			\li shape gets removed (this includes raising PxActorFlag::eDISABLE_SIMULATION)
			\li shape switches to eSCENE_QUERY_SHAPE only
			\li shape switches to eTRIGGER_SHAPE
			\li resetFiltering()
			\li actor gets removed from an aggregate

			*/
			eTEST_FOR_REMOVED_SHAPES	= (1<<0),

			/**
			\brief Invalid stream memory not allocated
			*/
			eINVALID_STREAM				= (1<<1),

			/**
			\brief Incomplete stream will be reported
			*/
			eINCOMPLETE_STREAM			= (1<<2),

			/**
			\brief The stream contains extra data with PxContactPairVelocity items where the post solver velocity needs to get written to.
			       Only valid for discrete collision (in CCD the post response velocity is available immediately).
			*/
			eNEEDS_POST_SOLVER_VELOCITY	= (1<<3),

			/**
			\brief Contains pairs that lost touch
			
			This info is used as an optimization to only parse the stream and check for removed shapes if there is a potential for
			having removed shapes in the stream that won't get detected in any other way. For example, there is the scenario where
			during the simulation a pair loses AABB touch and gets deleted. At that point a lost touch event might get written to the
			stream. If at fetchResults a buffered shape removal takes place, and that shape was part of the mentioned pair, there is
			no way any longer to make the connection to the corresponding event stream (since the pair has been deleted during the sim).
			*/
			eHAS_PAIRS_THAT_LOST_TOUCH	= (1<<4),

			/**
			\brief Marker for the next available free flag
			*/
			eNEXT_FREE_FLAG				= (1<<5)
		};
	};

	struct ContactStreamHeader
	{
		PxU16 contactPass;  // marker for extra data to know when a new collison pass started (discrete collision -> CCD pass 1 -> CCD pass 2 -> ...)
		PxU16 pad;  // to keep the stream 4byte aligned
	};

	/**
	\brief Contact report logic and data management.

	The internal contact report stream has the following format:

	ContactStreamHeader | PxContactPairIndex0 | (PxContactPairPose0, PxContactPairVelocity0) | ... | PxContactPairIndexN | (PxContactPairPoseN, PxContactPairVelocityN) | (unused memory up to maxExtraDataSize ) |
	PxContactPair0 | ... | PxContactPairM | (unsued pairs up to maxPairCount)
	*/
	class ContactStreamManager
	{
	public:
		PX_FORCE_INLINE ContactStreamManager() : maxPairCount(0), flags_and_maxExtraDataBlocks(0) {}
		PX_FORCE_INLINE ~ContactStreamManager() {}

		PX_FORCE_INLINE void reset();

		PX_FORCE_INLINE PxU16 getFlags() const;
		PX_FORCE_INLINE void raiseFlags(PxU16 flags);
		PX_FORCE_INLINE void clearFlags(PxU16 flags);

		PX_FORCE_INLINE PxU32 getMaxExtraDataSize() const;
		PX_FORCE_INLINE void setMaxExtraDataSize(PxU32 size);  // size in bytes (will translate into blocks internally)

		PX_FORCE_INLINE Sc::ContactShapePair* getShapePairs(PxU8* contactReportPairData) const;

		PX_FORCE_INLINE static void convertDeletedShapesInContactStream(ContactShapePair*, PxU32 pairCount, const ObjectIDTracker&);
		
		PX_FORCE_INLINE static PxU32 computeExtraDataBlockCount(PxU32 extraDataSize);
		PX_FORCE_INLINE static PxU32 computeExtraDataBlockSize(PxU32 extraDataSize);
		PX_FORCE_INLINE static PxU16 computeContactReportExtraDataSize(PxU32 extraDataFlags, bool addHeader);
		PX_FORCE_INLINE static void fillInContactReportExtraData(PxContactPairVelocity*, PxU32 index, const RigidSim&, bool isCCDPass);
		PX_FORCE_INLINE static void fillInContactReportExtraData(PxContactPairPose*, PxU32 index, const RigidSim&, bool isCCDPass, const bool useCurrentTransform);
		PX_FORCE_INLINE void fillInContactReportExtraData(PxU8* stream, PxU32 extraDataFlags, const RigidSim&, const RigidSim&, PxU32 ccdPass, const bool useCurrentTransform, PxU32 pairIndex, PxU32 sizeOffset);
		PX_FORCE_INLINE void setContactReportPostSolverVelocity(PxU8* stream, const RigidSim&, const RigidSim&);

		PxU32				bufferIndex;  // marks the start of the shape pair stream of the actor pair (byte offset with respect to global contact buffer stream)
		PxU16				maxPairCount;  // used to reserve the same amount of memory as in the last frame (as an initial guess)
		PxU16				currentPairCount;  // number of shape pairs stored in the buffer
		PxU16				extraDataSize;  // size of the extra data section in the stream
	private:
		PxU16				flags_and_maxExtraDataBlocks;  // used to reserve the same amount of memory as in the last frame (as an initial guess)

	public:
		static const PxU32	sExtraDataBlockSizePow2 = 4;  // extra data gets allocated as a multiple of 2^sExtraDataBlockSizePow2 to keep memory low of this struct.
		static const PxU32	sFlagMask = (ContactStreamManagerFlag::eNEXT_FREE_FLAG - 1);
		static const PxU32	sMaxExtraDataShift = 5;  // shift necessary to extract the maximum number of blocks allocated for extra data

		PX_COMPILE_TIME_ASSERT(ContactStreamManagerFlag::eNEXT_FREE_FLAG == (1 << sMaxExtraDataShift));
	};

} // namespace Sc


PX_FORCE_INLINE void Sc::ContactStreamManager::reset()
{
	currentPairCount = 0;
	extraDataSize = 0;
	flags_and_maxExtraDataBlocks &= ~sFlagMask;
}


PX_FORCE_INLINE PxU16 Sc::ContactStreamManager::getFlags() const
{
	return (flags_and_maxExtraDataBlocks & sFlagMask);
}


PX_FORCE_INLINE void Sc::ContactStreamManager::raiseFlags(PxU16 flags)
{
	PX_ASSERT(flags < ContactStreamManagerFlag::eNEXT_FREE_FLAG);

	flags_and_maxExtraDataBlocks |= flags;
}


PX_FORCE_INLINE void Sc::ContactStreamManager::clearFlags(PxU16 flags)
{
	PX_ASSERT(flags < ContactStreamManagerFlag::eNEXT_FREE_FLAG);

	PxU16 tmpFlags = getFlags();
	tmpFlags &= ~flags;
	flags_and_maxExtraDataBlocks &= ~sFlagMask;
	raiseFlags(tmpFlags);
}


PX_FORCE_INLINE PxU32 Sc::ContactStreamManager::getMaxExtraDataSize() const
{
	return PxU32((flags_and_maxExtraDataBlocks >> sMaxExtraDataShift) << sExtraDataBlockSizePow2);
}


PX_FORCE_INLINE void Sc::ContactStreamManager::setMaxExtraDataSize(PxU32 size)
{
	PxU32 nbBlocks = computeExtraDataBlockCount(size);
	flags_and_maxExtraDataBlocks = Ps::to16((flags_and_maxExtraDataBlocks & sFlagMask) | (nbBlocks << sMaxExtraDataShift));
}


PX_FORCE_INLINE Sc::ContactShapePair* Sc::ContactStreamManager::getShapePairs(PxU8* contactReportPairData) const
{
	return reinterpret_cast<Sc::ContactShapePair*>(contactReportPairData + getMaxExtraDataSize());
}


PX_FORCE_INLINE void Sc::ContactStreamManager::convertDeletedShapesInContactStream(ContactShapePair* shapePairs, PxU32 pairCount, const ObjectIDTracker& tracker)
{
	for(PxU32 i=0; i < pairCount; i++)
	{
		ContactShapePair& csp = shapePairs[i];

		PxU32 shape0ID = csp.shapeID[0];
		PxU32 shape1ID = csp.shapeID[1];

		PxU16 flags = csp.flags;
		PX_COMPILE_TIME_ASSERT(sizeof(flags) == sizeof((reinterpret_cast<ContactShapePair*>(0))->flags));

		if (tracker.isDeletedID(shape0ID))
			flags |= PxContactPairFlag::eREMOVED_SHAPE_0;
		if (tracker.isDeletedID(shape1ID))
			flags |= PxContactPairFlag::eREMOVED_SHAPE_1;

		csp.flags = flags;
	}
}


PX_FORCE_INLINE PxU32 Sc::ContactStreamManager::computeExtraDataBlockCount(PxU32 extraDataSize_)
{
	PxU32 nbBlocks;
	if (extraDataSize_ & ((1 << sExtraDataBlockSizePow2) - 1))  // not a multiple of block size -> need one block more
		nbBlocks = (extraDataSize_ >> sExtraDataBlockSizePow2) + 1;
	else
		nbBlocks = (extraDataSize_ >> sExtraDataBlockSizePow2);

	return nbBlocks;
}


PX_FORCE_INLINE PxU32 Sc::ContactStreamManager::computeExtraDataBlockSize(PxU32 extraDataSize_)
{
	return (computeExtraDataBlockCount(extraDataSize_) << sExtraDataBlockSizePow2);
}


PX_FORCE_INLINE PxU16 Sc::ContactStreamManager::computeContactReportExtraDataSize(PxU32 extraDataFlags, bool addHeader)
{
	PX_ASSERT(extraDataFlags);

	PxU16 extraDataSize_ = sizeof(PxContactPairIndex);
	if (extraDataFlags & PxPairFlag::ePRE_SOLVER_VELOCITY)
		extraDataSize_ += sizeof(PxContactPairVelocity);
	if (extraDataFlags & PxPairFlag::ePOST_SOLVER_VELOCITY)
		extraDataSize_ += sizeof(PxContactPairVelocity);
	if (extraDataFlags & PxPairFlag::eCONTACT_EVENT_POSE)
		extraDataSize_ += sizeof(PxContactPairPose);
	if (addHeader)
		extraDataSize_ += sizeof(ContactStreamHeader);
	return extraDataSize_;
}


PX_FORCE_INLINE void Sc::ContactStreamManager::fillInContactReportExtraData(PxContactPairVelocity* cpVel, PxU32 index, const RigidSim& rs, bool isCCDPass)
{
	if (rs.getActorType() != PxActorType::eRIGID_STATIC)
	{
		const BodySim& bs = static_cast<const BodySim&>(rs);
		if ((!isCCDPass) || (cpVel->type == PxContactPairExtraDataType::ePOST_SOLVER_VELOCITY))
		{
			const BodyCore& bc = bs.getBodyCore();
			cpVel->linearVelocity[index] = bc.getLinearVelocity();
			cpVel->angularVelocity[index] = bc.getAngularVelocity();
		}
		else
		{
			PX_ASSERT(cpVel->type == PxContactPairExtraDataType::ePRE_SOLVER_VELOCITY);
			const Cm::SpatialVector& vel = bs.getLowLevelBody().getPreSolverVelocities();
			cpVel->linearVelocity[index] = vel.linear;
			cpVel->angularVelocity[index] = vel.angular;
		}
	}
	else
	{
		cpVel->linearVelocity[index] = PxVec3(0.0f);
		cpVel->angularVelocity[index] = PxVec3(0.0f);
	}
}


PX_FORCE_INLINE void Sc::ContactStreamManager::fillInContactReportExtraData(PxContactPairPose* cpPose, PxU32 index, const RigidSim& rs, bool isCCDPass, const bool useCurrentTransform)
{
	if(rs.getActorType() != PxActorType::eRIGID_STATIC)
	{
		const BodySim& bs = static_cast<const BodySim&>(rs);
		const BodyCore& bc = bs.getBodyCore();
		const PxTransform& src = (!isCCDPass && useCurrentTransform) ? bc.getBody2World() : bs.getLowLevelBody().getLastCCDTransform();
		cpPose->globalPose[index] = src * bc.getBody2Actor().getInverse();
	}
	else
	{
		const StaticSim& ss = static_cast<const StaticSim&>(rs);
		const StaticCore& sc = ss.getStaticCore();
		cpPose->globalPose[index] = sc.getActor2World();
	}
}


PX_FORCE_INLINE void Sc::ContactStreamManager::fillInContactReportExtraData(PxU8* stream, PxU32 extraDataFlags, const RigidSim& rs0, const RigidSim& rs1, PxU32 ccdPass, const bool useCurrentTransform,
	PxU32 pairIndex, PxU32 sizeOffset)
{
	ContactStreamHeader* strHeader = reinterpret_cast<ContactStreamHeader*>(stream);
	strHeader->contactPass = Ps::to16(ccdPass);

	stream += sizeOffset;
	PxU8* edStream = stream;
	bool isCCDPass = (ccdPass != 0);

	{
		PxContactPairIndex* cpIndex = reinterpret_cast<PxContactPairIndex*>(edStream);
		cpIndex->type = PxContactPairExtraDataType::eCONTACT_PAIR_INDEX;
		cpIndex->index = Ps::to16(pairIndex);
		edStream += sizeof(PxContactPairIndex);

		PX_ASSERT(edStream <= reinterpret_cast<PxU8*>(getShapePairs(stream)));
	}

	// Important: make sure this one is the first after the PxContactPairIndex item for discrete contacts as it needs to get filled in before the reports get sent
	//            (post solver velocity is not available when it gets created)
	if (extraDataFlags & PxPairFlag::ePOST_SOLVER_VELOCITY)
	{
		PxContactPairVelocity* cpVel = reinterpret_cast<PxContactPairVelocity*>(edStream);
		cpVel->type = PxContactPairExtraDataType::ePOST_SOLVER_VELOCITY;
		edStream += sizeof(PxContactPairVelocity);

		if (!isCCDPass)
			raiseFlags(ContactStreamManagerFlag::eNEEDS_POST_SOLVER_VELOCITY);  // don't know the post solver velocity yet
		else
		{
			ContactStreamManager::fillInContactReportExtraData(cpVel, 0, rs0, true);
			ContactStreamManager::fillInContactReportExtraData(cpVel, 1, rs1, true);
		}

		PX_ASSERT(edStream <= reinterpret_cast<PxU8*>(getShapePairs(stream)));
	}
	if (extraDataFlags & PxPairFlag::ePRE_SOLVER_VELOCITY)
	{
		PxContactPairVelocity* cpVel = reinterpret_cast<PxContactPairVelocity*>(edStream);
		cpVel->type = PxContactPairExtraDataType::ePRE_SOLVER_VELOCITY;
		ContactStreamManager::fillInContactReportExtraData(cpVel, 0, rs0, isCCDPass);
		ContactStreamManager::fillInContactReportExtraData(cpVel, 1, rs1, isCCDPass);
		edStream += sizeof(PxContactPairVelocity);

		PX_ASSERT(edStream <= reinterpret_cast<PxU8*>(getShapePairs(stream)));
	}
	if (extraDataFlags & PxPairFlag::eCONTACT_EVENT_POSE)
	{
		PxContactPairPose* cpPose = reinterpret_cast<PxContactPairPose*>(edStream);
		cpPose->type = PxContactPairExtraDataType::eCONTACT_EVENT_POSE;
		ContactStreamManager::fillInContactReportExtraData(cpPose, 0, rs0, isCCDPass, useCurrentTransform);
		ContactStreamManager::fillInContactReportExtraData(cpPose, 1, rs1, isCCDPass, useCurrentTransform);
		edStream += sizeof(PxContactPairPose);

		PX_ASSERT(edStream <= reinterpret_cast<PxU8*>(getShapePairs(stream)));
	}

	extraDataSize = Ps::to16(sizeOffset + PxU32(edStream - stream));
}


PX_FORCE_INLINE void Sc::ContactStreamManager::setContactReportPostSolverVelocity(PxU8* stream, const RigidSim& rs0, const RigidSim& rs1)
{
	PX_ASSERT(extraDataSize > (sizeof(ContactStreamHeader) + sizeof(PxContactPairIndex)));
	PxContactPairVelocity* cpVel = reinterpret_cast<PxContactPairVelocity*>(stream + sizeof(ContactStreamHeader) + sizeof(PxContactPairIndex));
	PX_ASSERT(cpVel->type == PxContactPairExtraDataType::ePOST_SOLVER_VELOCITY);

	fillInContactReportExtraData(cpVel, 0, rs0, false);
	fillInContactReportExtraData(cpVel, 1, rs1, false);

	clearFlags(ContactStreamManagerFlag::eNEEDS_POST_SOLVER_VELOCITY);
}


}

#endif
