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

#ifndef PX_COLLISION_ACTORPAIR
#define PX_COLLISION_ACTORPAIR

#include "ScRigidSim.h"
#include "ScContactStream.h"
#include "ScNPhaseCore.h"

namespace physx
{
namespace Sc
{
	class ActorPairContactReportData
	{
	public:
		ActorPairContactReportData() : 
			mStrmResetStamp			(0xffffffff),
			mActorAID				(0xffffffff),
			mActorBID				(0xffffffff),
			mPxActorA				(NULL),
			mPxActorB				(NULL)
			{}

		ContactStreamManager	mContactStreamManager;
		PxU32					mStrmResetStamp;
		PxU32					mActorAID;
		PxU32					mActorBID;
		PxActor*				mPxActorA;
		PxActor*				mPxActorB;
	};

	/**
	\brief Class shared by all shape interactions for a pair of actors.

	This base class is used if no shape pair of an actor pair has contact reports requested.
	*/
	class ActorPair
	{
	public:

		enum ActorPairFlags
		{
			eIS_REPORT_PAIR	= (1<<0),
			eNEXT_FREE		= (1<<1)
		};

		PX_FORCE_INLINE					ActorPair() : mInternalFlags(0), mTouchCount(0), mRefCount(0) {}
		PX_FORCE_INLINE					~ActorPair() {}

		PX_FORCE_INLINE	Ps::IntBool		isReportPair() const { return (mInternalFlags & eIS_REPORT_PAIR); }

		PX_FORCE_INLINE	void			incTouchCount() { mTouchCount++; PX_ASSERT(mTouchCount); }
		PX_FORCE_INLINE	void			decTouchCount() { PX_ASSERT(mTouchCount); mTouchCount--; }
		PX_FORCE_INLINE	PxU32			getTouchCount() const { return mTouchCount; }

		PX_FORCE_INLINE	void			incRefCount() { ++mRefCount; PX_ASSERT(mRefCount>0); }
		PX_FORCE_INLINE	PxU32			decRefCount() { PX_ASSERT(mRefCount>0); return --mRefCount; }
		PX_FORCE_INLINE	PxU32			getRefCount() const { return mRefCount; }

	private:
		ActorPair& operator=(const ActorPair&);

	protected:
						PxU16			mInternalFlags;
						PxU16			mTouchCount;
						PxU16			mRefCount;
						PxU16			mPad;  // instances of this class are stored in a pool which needs an item size of at least size_t
	};

	/**
	\brief Class shared by all shape interactions for a pair of actors if contact reports are requested.

	This class is used if at least one shape pair of an actor pair has contact reports requested.

	\note If a pair of actors had contact reports requested for some of the shape interactions but all of them switch to not wanting contact reports
	any longer, then the ActorPairReport instance is kept being used and won't get replaced by a simpler ActorPair instance.
	*/
	class ActorPairReport : public ActorPair
	{
	public:

		enum ActorPairReportFlags
		{
			eIS_IN_CONTACT_REPORT_ACTOR_PAIR_SET = ActorPair::eNEXT_FREE	// PT: whether the pair is already stored in the 'ContactReportActorPairSet' or not
		};

		PX_FORCE_INLINE					ActorPairReport(RigidSim&, RigidSim&);
		PX_FORCE_INLINE					~ActorPairReport();

		PX_INLINE ContactStreamManager&	createContactStreamManager(NPhaseCore&);
		PX_FORCE_INLINE ContactStreamManager& getContactStreamManager() const { PX_ASSERT(mReportData); return mReportData->mContactStreamManager; }
		PX_FORCE_INLINE	RigidSim&		getActorA() const { return mActorA; }
		PX_FORCE_INLINE	RigidSim&		getActorB() const { return mActorB; }
		PX_INLINE		PxU32			getActorAID() const { PX_ASSERT(mReportData); return mReportData->mActorAID; }
		PX_INLINE		PxU32			getActorBID() const { PX_ASSERT(mReportData); return mReportData->mActorBID; }
		PX_INLINE		PxActor*		getPxActorA() const { PX_ASSERT(mReportData); return mReportData->mPxActorA; }
		PX_INLINE		PxActor*		getPxActorB() const { PX_ASSERT(mReportData); return mReportData->mPxActorB; }
		PX_FORCE_INLINE	bool			streamResetNeeded(PxU32 cmpStamp) const;
		PX_INLINE		bool			streamResetStamp(PxU32 cmpStamp);

		PX_FORCE_INLINE	PxU16			isInContactReportActorPairSet() const { return PxU16(mInternalFlags & eIS_IN_CONTACT_REPORT_ACTOR_PAIR_SET); }
		PX_FORCE_INLINE	void			setInContactReportActorPairSet() { mInternalFlags |= eIS_IN_CONTACT_REPORT_ACTOR_PAIR_SET; }
		PX_FORCE_INLINE	void			clearInContactReportActorPairSet() { mInternalFlags &= ~eIS_IN_CONTACT_REPORT_ACTOR_PAIR_SET; }

		PX_FORCE_INLINE void			createContactReportData(NPhaseCore&);
		PX_FORCE_INLINE void			releaseContactReportData(NPhaseCore&);
		PX_FORCE_INLINE const ActorPairContactReportData* hasReportData() const { return mReportData; }

		PX_FORCE_INLINE	void			convert(ActorPair& aPair) { PX_ASSERT(!aPair.isReportPair()); mTouchCount = PxU16(aPair.getTouchCount()); mRefCount = PxU16(aPair.getRefCount()); }

		PX_FORCE_INLINE static ActorPairReport& cast(ActorPair& aPair) { PX_ASSERT(aPair.isReportPair()); return static_cast<ActorPairReport&>(aPair); }

	private:
		ActorPairReport& operator=(const ActorPairReport&);

						RigidSim&		mActorA;
						RigidSim&		mActorB;

			ActorPairContactReportData* mReportData;
	};

} // namespace Sc

PX_FORCE_INLINE Sc::ActorPairReport::ActorPairReport(RigidSim& actor0, RigidSim& actor1) : ActorPair(),
mActorA			(actor0),
mActorB			(actor1),
mReportData		(NULL)
{
	PX_ASSERT(mInternalFlags == 0);
	mInternalFlags = ActorPair::eIS_REPORT_PAIR;
}

PX_FORCE_INLINE Sc::ActorPairReport::~ActorPairReport()
{
	PX_ASSERT(mReportData == NULL);
}

PX_FORCE_INLINE bool Sc::ActorPairReport::streamResetNeeded(PxU32 cmpStamp) const
{
	return (cmpStamp != mReportData->mStrmResetStamp);
}

PX_INLINE bool Sc::ActorPairReport::streamResetStamp(PxU32 cmpStamp) 
{
	PX_ASSERT(mReportData);
	const bool ret = streamResetNeeded(cmpStamp);
	mReportData->mStrmResetStamp = cmpStamp; 
	return ret; 
}

PX_INLINE Sc::ContactStreamManager&	Sc::ActorPairReport::createContactStreamManager(NPhaseCore& npCore)
{
	// Lazy create report data
	if(!mReportData)
		createContactReportData(npCore);

	return mReportData->mContactStreamManager;
}

PX_FORCE_INLINE void Sc::ActorPairReport::createContactReportData(NPhaseCore& npCore)
{
	PX_ASSERT(!mReportData);
	Sc::ActorPairContactReportData* reportData = npCore.createActorPairContactReportData(); 
	mReportData = reportData;

	if(reportData)
	{
		reportData->mActorAID = mActorA.getRigidID();
		reportData->mActorBID = mActorB.getRigidID();

		reportData->mPxActorA = mActorA.getPxActor();
		reportData->mPxActorB = mActorB.getPxActor();
	}
}

PX_FORCE_INLINE void Sc::ActorPairReport::releaseContactReportData(NPhaseCore& npCore)
{
	// Can't take the NPhaseCore (scene) reference from the actors since they're already gone on scene release

	if(mReportData)
	{
		npCore.releaseActorPairContactReportData(mReportData);
		mReportData = NULL;
	}
}

}

#endif
