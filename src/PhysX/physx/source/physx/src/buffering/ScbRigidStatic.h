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

#ifndef PX_PHYSICS_SCB_RIGID_STATIC
#define PX_PHYSICS_SCB_RIGID_STATIC

#include "ScStaticCore.h"
#include "ScbScene.h"
#include "ScbActor.h"
#include "ScbRigidObject.h"

namespace physx
{
namespace Scb
{
#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

struct RigidStaticBuffer : public RigidObjectBuffer
{
#ifdef USE_NEW_SYSTEM
	PX_ALIGN(16, PxTransform) mActor2World;
#else
	template <PxU32 I, PxU32 Dummy> struct Fns {};		// TODO: make the base class traits visible
	typedef Sc::StaticCore Core;
	typedef RigidStaticBuffer Buf;

	// regular attributes
	enum { BF_Base = RigidObjectBuffer::AttrCount };
//	SCB_REGULAR_ATTRIBUTE(BF_Base,	PxTransform,	Actor2World)
	SCB_REGULAR_ATTRIBUTE_ALIGNED(BF_Base,			PxTransform,		Actor2World, 16)
#endif
};

#if PX_VC 
     #pragma warning(pop) 
#endif

class RigidStatic : public Scb::RigidObject
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

	typedef RigidStaticBuffer Buf;
	typedef Sc::StaticCore Core;

public:
// PX_SERIALIZATION
										RigidStatic(const PxEMPTY) :	Scb::RigidObject(PxEmpty), mStatic(PxEmpty)	{}
	static		void					getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
	PX_INLINE							RigidStatic(const PxTransform& actor2World);
	PX_INLINE							~RigidStatic() {}

#ifdef USE_NEW_SYSTEM
	SCB_MEMBER(RigidStatic, mStatic, Actor2World, const PxTransform&, RigidObjectBuffer::AttrCount)
#else
	PX_INLINE		const PxTransform&	getActor2World() const					{ return read<Buf::BF_Actor2World>(); }
	PX_INLINE		void				setActor2World(const PxTransform& m)	{ write<Buf::BF_Actor2World>(m); }
#endif

	PX_FORCE_INLINE void				onOriginShift(const PxVec3& shift)		{ mStatic.onOriginShift(shift); }

	//---------------------------------------------------------------------------------
	// Data synchronization
	//---------------------------------------------------------------------------------
	PX_INLINE void						syncState();

	static size_t getScOffset()	{ return reinterpret_cast<size_t>(&reinterpret_cast<RigidStatic*>(0)->mStatic);	}

	PX_FORCE_INLINE Sc::StaticCore&		getScStatic()				{	return mStatic; }

	PX_FORCE_INLINE void				initBufferedState()			{}

private:
					Sc::StaticCore		mStatic;

	PX_FORCE_INLINE	const Buf*		getRigidActorBuffer()	const	{ return reinterpret_cast<const Buf*>(getStream()); }
	PX_FORCE_INLINE	Buf*			getRigidActorBuffer()			{ return reinterpret_cast<Buf*>(getStream()); }

#ifndef USE_NEW_SYSTEM
	//---------------------------------------------------------------------------------
	// Infrastructure for regular attributes
	//---------------------------------------------------------------------------------

	struct Access: public BufferedAccess<Buf, Core, RigidStatic> {};

	template<PxU32 f> PX_FORCE_INLINE typename Buf::Fns<f,0>::Arg read() const		{	return Access::read<Buf::Fns<f,0> >(*this, mStatic);	}
	template<PxU32 f> PX_FORCE_INLINE void write(typename Buf::Fns<f,0>::Arg v)		{	Access::write<Buf::Fns<f,0> >(*this, mStatic, v);		}
	template<PxU32 f> PX_FORCE_INLINE void flush(const Buf& buf)					{	Access::flush<Buf::Fns<f,0> >(*this, mStatic, buf);		}
#endif
};

RigidStatic::RigidStatic(const PxTransform& actor2World) : 
	mStatic(actor2World)
{
	setScbType(ScbType::eRIGID_STATIC);
}

//--------------------------------------------------------------
//
// Data synchronization
//
//--------------------------------------------------------------
PX_INLINE void RigidStatic::syncState()
{
	const PxU32 bufferFlags = getBufferFlags();

#ifdef USE_NEW_SYSTEM
	if(bufferFlags & BF_ActorFlags)
#else
	if(bufferFlags & Buf::BF_ActorFlags)
#endif
		syncNoSimSwitch(*getRigidActorBuffer(), mStatic, false);

	RigidObject::syncState();

#ifdef USE_NEW_SYSTEM
	syncActor2World();
#else
	if(bufferFlags & Buf::BF_Actor2World)
		flush<Buf::BF_Actor2World>(*getRigidActorBuffer());
#endif
	postSyncState();
}

}  // namespace Scb

}

#endif
