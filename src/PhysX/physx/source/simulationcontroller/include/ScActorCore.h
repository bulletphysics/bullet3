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


#ifndef PX_COLLISION_ACTOR_CORE
#define PX_COLLISION_ACTOR_CORE

#include "PsUserAllocated.h"
#include "CmPhysXCommon.h"
#include "PxMetaData.h"
#include "PxActor.h"

namespace physx
{

class PxActor;

namespace Sc
{

	class Scene;
	class ActorSim;

	class ActorCore : public Ps::UserAllocated
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================
	public:
// PX_SERIALIZATION
											ActorCore(const PxEMPTY) :	mSim(NULL), mActorFlags(PxEmpty)
											{
											}
		static			void				getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
											ActorCore(PxActorType::Enum actorType, PxU8 actorFlags, 
													  PxClientID owner, PxDominanceGroup dominanceGroup);
		/*virtual*/							~ActorCore();

		PX_FORCE_INLINE	ActorSim*			getSim()						const	{ return mSim;							}
		PX_FORCE_INLINE	void				setSim(ActorSim* sim)
											{
												PX_ASSERT((sim==NULL) ^ (mSim==NULL));
												mSim = sim;
											}

		PX_FORCE_INLINE	PxActorFlags		getActorFlags()					const	{ return mActorFlags;					}
						void				setActorFlags(PxActorFlags af);

		PX_FORCE_INLINE	PxDominanceGroup	getDominanceGroup()				const
											{
												return PxDominanceGroup(mDominanceGroup);	
											}
						void				setDominanceGroup(PxDominanceGroup g);

		PX_FORCE_INLINE	void				setOwnerClient(PxClientID inId)
											{
												const PxU32 aggid = mAggregateIDOwnerClient & 0x00ffffff;
												mAggregateIDOwnerClient = (PxU32(inId)<<24) | aggid;
											}
		PX_FORCE_INLINE	PxClientID			getOwnerClient()				const
											{
												return mAggregateIDOwnerClient>>24;
											}

		PX_FORCE_INLINE	PxActorType::Enum	getActorCoreType()				const 	{ return PxActorType::Enum(mActorType);	}

						void				reinsertShapes();

		PX_FORCE_INLINE	void				setAggregateID(PxU32 id)
											{
												PX_ASSERT(id==0xffffffff || id<(1<<24));
												const PxU32 ownerClient = mAggregateIDOwnerClient & 0xff000000;
												mAggregateIDOwnerClient = (id & 0x00ffffff) | ownerClient;
											}
		PX_FORCE_INLINE	PxU32				getAggregateID()				const
											{
												const PxU32 id = mAggregateIDOwnerClient & 0x00ffffff;
												return id == 0x00ffffff ? PX_INVALID_U32 : id;
											}
	private:
						ActorSim*			mSim;						// 
						PxU32				mAggregateIDOwnerClient;	// PxClientID (8bit) | aggregate ID (24bit)
		// PT: TODO: the remaining members could be packed into just a 16bit mask
						PxActorFlags		mActorFlags;				// PxActor's flags (PxU8) => only 4 bits used
						PxU8				mActorType;					// Actor type (8 bits, but 3 would be enough)
						PxU8				mDominanceGroup;			// Dominance group (8 bits, but 5 would be enough because "must be < 32")
	};

#if PX_P64_FAMILY
	PX_COMPILE_TIME_ASSERT(sizeof(Sc::ActorCore)==16);
#else
	PX_COMPILE_TIME_ASSERT(sizeof(Sc::ActorCore)==12);
#endif

} // namespace Sc

}

//////////////////////////////////////////////////////////////////////////

#endif
