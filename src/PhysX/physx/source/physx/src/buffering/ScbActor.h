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

#ifndef PX_PHYSICS_SCB_FSACTOR
#define PX_PHYSICS_SCB_FSACTOR

#include "ScActorCore.h"
#include "PsUtilities.h"
#include "ScbBase.h"
#include "ScbDefs.h"

#include "PxClient.h"


namespace physx
{
namespace Scb
{
struct ActorBuffer
{
#ifdef USE_NEW_SYSTEM
	PxActorFlags				mActorFlags;
	PxDominanceGroup			mDominanceGroup;
//	PxActorClientBehaviorFlags	mClientBehaviorFlags;
#else
	template <PxU32 I, PxU32 Dummy> struct Fns {};
	typedef Sc::ActorCore Core;
	typedef ActorBuffer Buf;

	SCB_REGULAR_ATTRIBUTE (0, PxActorFlags,					ActorFlags)
	SCB_REGULAR_ATTRIBUTE (1, PxDominanceGroup,				DominanceGroup)
//	SCB_REGULAR_ATTRIBUTE (2, PxActorClientBehaviorFlags,	ClientBehaviorFlags)
#endif
	enum { AttrCount = 3 };

protected:
	~ActorBuffer(){}
};

class Actor : public Base
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

	typedef ActorBuffer Buf;
	typedef Sc::ActorCore Core;

public:
// PX_SERIALIZATION
												Actor(const PxEMPTY) : Base(PxEmpty) {}
	static			void						getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION

	PX_INLINE									Actor() {}

#ifdef USE_NEW_SYSTEM
	SCB_MEMBER(Actor, getActorCore(), ActorFlags, PxActorFlags, 0)
	SCB_MEMBER(Actor, getActorCore(), DominanceGroup, PxDominanceGroup, 1)
//	SCB_MEMBER(Actor, getActorCore(), ClientBehaviorFlags, PxActorClientBehaviorFlags, 2)
#endif

	//---------------------------------------------------------------------------------
	// Wrapper for Sc::Actor interface
	//---------------------------------------------------------------------------------
#ifndef USE_NEW_SYSTEM
	PX_INLINE		PxActorFlags				getActorFlags() const								{ return read<Buf::BF_ActorFlags>(); }
	PX_INLINE		void						setActorFlags(PxActorFlags v);

	PX_INLINE		PxDominanceGroup			getDominanceGroup() const							{ return read<Buf::BF_DominanceGroup>();}
	PX_INLINE		void						setDominanceGroup(PxDominanceGroup v)				{ write<Buf::BF_DominanceGroup>(v);		}

//	PX_INLINE		PxActorClientBehaviorFlags	getClientBehaviorFlags() const						{ return read<Buf::BF_ClientBehaviorFlags>(); }
//	PX_INLINE		void						setClientBehaviorFlags(PxActorClientBehaviorFlags v){ write<Buf::BF_ClientBehaviorFlags>(v); }
#endif

	PX_INLINE		void						setOwnerClient(PxClientID inId);
	PX_INLINE		PxClientID					getOwnerClient() const								{ return getActorCore().getOwnerClient(); }	//immutable, so this should be fine. 

	//---------------------------------------------------------------------------------
	// Miscellaneous
	//---------------------------------------------------------------------------------
	PX_FORCE_INLINE const Core&					getActorCore() const 	{ return *reinterpret_cast<const Core*>(reinterpret_cast<size_t>(this) + sOffsets.scbToSc[getScbType()]); }
	PX_FORCE_INLINE	Core&						getActorCore()			{ return *reinterpret_cast<Core*>(reinterpret_cast<size_t>(this) + sOffsets.scbToSc[getScbType()]); }

	PX_FORCE_INLINE static const Actor&			fromSc(const Core& a)	{ return *reinterpret_cast<const Actor*>(reinterpret_cast<size_t>(&a) - sOffsets.scToScb[a.getActorCoreType()]); }
	PX_FORCE_INLINE static Actor&				fromSc(Core &a)			{ return *reinterpret_cast<Actor*>(reinterpret_cast<size_t>(&a) - sOffsets.scToScb[a.getActorCoreType()]); }

	PX_FORCE_INLINE PxActorType::Enum			getActorType()	const	{ return getActorCore().getActorCoreType();	}

protected:
												~Actor() {}
	PX_INLINE		void						syncState();

#ifndef USE_NEW_SYSTEM
	//---------------------------------------------------------------------------------
	// Infrastructure for regular attributes
	//---------------------------------------------------------------------------------
	struct Access: public BufferedAccess<Buf, Core, Actor> {};
	template<PxU32 f> PX_FORCE_INLINE typename Buf::Fns<f,0>::Arg read() const		{	return Access::read<Buf::Fns<f,0> >(*this, getActorCore());	}
	template<PxU32 f> PX_FORCE_INLINE void write(typename Buf::Fns<f,0>::Arg v)		{	Access::write<Buf::Fns<f,0> >(*this, getActorCore(), v);	}
	template<PxU32 f> PX_FORCE_INLINE void flush(Core& core, const Buf& buf)		{	Access::flush<Buf::Fns<f,0> >(*this, core, buf);			}
#endif

	struct Offsets
	{
		size_t scToScb[PxActorType::eACTOR_COUNT];
		size_t scbToSc[ScbType::eTYPE_COUNT];
		Offsets();
	};
	static const Offsets					sOffsets;
};

#ifndef USE_NEW_SYSTEM
PX_INLINE void Actor::setActorFlags(PxActorFlags v)
{
	// PT: TODO: move this check out of here, they should be done in Np!
#if PX_CHECKED
	const PxActorFlags aFlags = getActorFlags();
	const PxActorType::Enum aType = getActorType();
	if((!aFlags.isSet(PxActorFlag::eDISABLE_SIMULATION)) && v.isSet(PxActorFlag::eDISABLE_SIMULATION) &&
		(aType != PxActorType::eRIGID_DYNAMIC) && (aType != PxActorType::eRIGID_STATIC))
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
				"PxActor::setActorFlag: PxActorFlag::eDISABLE_SIMULATION is only supported by PxRigidDynamic and PxRigidStatic objects.");
	}
#endif

	write<Buf::BF_ActorFlags>(v);
}
#endif

PX_INLINE void Actor::setOwnerClient(PxClientID inId)
{
	//This call is only valid if we aren't in a scene.
	//Thus we can't be buffering yet
	if(!isBuffering())
	{
		getActorCore().setOwnerClient(inId);
		UPDATE_PVD_PROPERTIES_OBJECT()
	}
	else
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, 
				"Attempt to set the client id when an actor is buffering");
	}
}

PX_INLINE void Actor::syncState()
{
	//this should be called from syncState() of derived classes

	const PxU32 flags = getBufferFlags();
#ifdef USE_NEW_SYSTEM
	if(flags & (BF_ActorFlags|BF_DominanceGroup/*|BF_ClientBehaviorFlags*/))
	{
		syncActorFlags();
		syncDominanceGroup();
//		syncClientBehaviorFlags();
	}
#else
	if(flags & (Buf::BF_ActorFlags|Buf::BF_DominanceGroup/*|Buf::BF_ClientBehaviorFlags*/))
	{
		Core& core = getActorCore();
		const Buf& buffer = *reinterpret_cast<const Buf*>(getStream());

		flush<Buf::BF_ActorFlags>(core, buffer);
		flush<Buf::BF_DominanceGroup>(core, buffer);
//		flush<Buf::BF_ClientBehaviorFlags>(core, buffer);
	}
#endif
}

}  // namespace Scb

}

#endif
