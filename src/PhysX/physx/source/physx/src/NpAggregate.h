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


#ifndef PX_PHYSICS_NP_AGGREGATE
#define PX_PHYSICS_NP_AGGREGATE

#include "CmPhysXCommon.h"
#include "PxAggregate.h"
#include "ScbAggregate.h"
#include "PsUserAllocated.h"

namespace physx
{

class NpScene;

class NpAggregate : public PxAggregate, public Ps::UserAllocated
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
// PX_SERIALIZATION
										NpAggregate(PxBaseFlags baseFlags) : PxAggregate(baseFlags), mAggregate(PxEmpty) {}
	    virtual	        void	     	exportExtraData(PxSerializationContext& stream);
						void			importExtraData(PxDeserializationContext& context);
						void			resolveReferences(PxDeserializationContext& context);
	    virtual	        void			requiresObjects(PxProcessPxBaseCallback& c);
		static			NpAggregate*	createObject(PxU8*& address, PxDeserializationContext& context);
		static			void			getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
										NpAggregate(PxU32 maxActors, bool selfCollision);
		virtual							~NpAggregate();

		virtual			void			release();
		virtual			bool			addActor(PxActor&, const PxBVHStructure* );
		virtual			bool			removeActor(PxActor&);
		virtual			bool			addArticulation(PxArticulationBase&);
		virtual			bool			removeArticulation(PxArticulationBase&);

		virtual			PxU32			getNbActors() const;
		virtual			PxU32			getMaxNbActors() const;
		virtual			PxU32			getActors(PxActor** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;

		virtual			PxScene*		getScene();
		virtual			bool			getSelfCollision()	const;

		PX_FORCE_INLINE	PxU32			getCurrentSizeFast()	const	{ return mNbActors; }
		PX_FORCE_INLINE	PxActor*		getActorFast(PxU32 i)	const	{ return mActors[i]; }
		PX_FORCE_INLINE	bool			getSelfCollideFast()	const	{ return mAggregate.getSelfCollide(); }

						NpScene*		getAPIScene() const;
						NpScene*		getOwnerScene() const; // the scene the user thinks the actor is in, or from which the actor is pending removal

						void			addActorInternal(PxActor& actor, NpScene& s, const PxBVHStructure* bvhStructure);
						void			removeAndReinsert(PxActor& actor, bool reinsert);
						bool			removeActorAndReinsert(PxActor& actor, bool reinsert);
						bool			removeArticulationAndReinsert(PxArticulationBase& art, bool reinsert);

		PX_FORCE_INLINE	Scb::Aggregate&	getScbAggregate() { return mAggregate; }

private:
						Scb::Aggregate	mAggregate;
						PxU32			mNbActors;
						PxActor**		mActors;
};

}

#endif
