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


#ifndef PX_PHYSICS_NP_ACTOR_TEMPLATE
#define PX_PHYSICS_NP_ACTOR_TEMPLATE

#include "PsUserAllocated.h"
#include "NpWriteCheck.h"
#include "NpReadCheck.h"
#include "NpActor.h"
#include "ScbActor.h"
#include "NpScene.h"

namespace physx
{

// PT: only API (virtual) functions should be implemented here. Other shared non-virtual functions should go to NpActor.

/**
This is an API class. API classes run in a different thread than the simulation.
For the sake of simplicity they have their own methods, and they do not call simulation
methods directly. To set simulation state, they also have their own custom set
methods in the implementation classes.

Changing the data layout of this class breaks the binary serialization format.
See comments for PX_BINARY_SERIAL_VERSION.
*/
template<class APIClass>
class NpActorTemplate : public APIClass, public NpActor, public Ps::UserAllocated
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
	PX_NOCOPY(NpActorTemplate)
public:
// PX_SERIALIZATION
											NpActorTemplate(PxBaseFlags baseFlags) : APIClass(baseFlags), NpActor(PxEmpty) {}
	virtual	void							exportExtraData(PxSerializationContext& stream) { NpActor::exportExtraData(stream); }
			void							importExtraData(PxDeserializationContext& context) { NpActor::importExtraData(context);	}
	virtual void							resolveReferences(PxDeserializationContext& context) { NpActor::resolveReferences(context);		}
//~PX_SERIALIZATION

											NpActorTemplate(PxType concreteType, PxBaseFlags baseFlags, const char* name, void* userData);
	virtual									~NpActorTemplate();

	//---------------------------------------------------------------------------------
	// PxActor implementation
	//---------------------------------------------------------------------------------
	virtual		void						release()												{ NpActor::release(*this);	}

	// The rule is: If an API method is used somewhere in here, it has to be redeclared, else GCC whines
	virtual		PxActorType::Enum			getType() const = 0;

	virtual		PxScene*					getScene()				const;
	
	// Debug name
	virtual		void						setName(const char*);
	virtual		const char*					getName()				const;

	virtual		PxBounds3					getWorldBounds(float inflation=1.01f) const = 0;

	// Flags
	virtual		void						setActorFlag(PxActorFlag::Enum flag, bool value);
	virtual		void						setActorFlags(PxActorFlags inFlags);
	virtual		PxActorFlags				getActorFlags() const;

	// Dominance
	virtual		void						setDominanceGroup(PxDominanceGroup dominanceGroup);
	virtual		PxDominanceGroup			getDominanceGroup() const;

	// Multiclient
	virtual		void						setOwnerClient( PxClientID inClient );
	virtual		PxClientID					getOwnerClient() const;

	// Aggregates
	virtual		PxAggregate*				getAggregate()	const	{ return NpActor::getAggregate();	}

	//---------------------------------------------------------------------------------
	// Miscellaneous
	//---------------------------------------------------------------------------------
protected:
	PX_FORCE_INLINE void					setActorFlagInternal(PxActorFlag::Enum flag, bool value);
	PX_FORCE_INLINE void					setActorFlagsInternal(PxActorFlags inFlags);
};

///////////////////////////////////////////////////////////////////////////////

template<class APIClass>
NpActorTemplate<APIClass>::NpActorTemplate(PxType concreteType,
										   PxBaseFlags baseFlags,
										   const char* name,
										   void* actorUserData)
:APIClass(concreteType, baseFlags),
NpActor(name)
{
	// don't ref Scb actor here, it hasn't been assigned yet

	APIClass::userData = actorUserData;
}


template<class APIClass>
NpActorTemplate<APIClass>::~NpActorTemplate()
{
	NpActor::onActorRelease(this);
}

///////////////////////////////////////////////////////////////////////////////

// PT: this one is very slow
template<class APIClass>
PxScene* NpActorTemplate<APIClass>::getScene() const
{
	return NpActor::getAPIScene(*this);
}

///////////////////////////////////////////////////////////////////////////////

template<class APIClass>
void NpActorTemplate<APIClass>::setName(const char* debugName)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	mName = debugName;

#if PX_SUPPORT_PVD
		Scb::Scene* scbScene = getScbFromPxActor(*this).getScbSceneForAPI();
		Scb::Actor& scbActor = NpActor::getScbFromPxActor(*this);
		//Name changing is not bufferred
		if(scbScene)
			scbScene->getScenePvdClient().updatePvdProperties(&scbActor);	
#endif
}

template<class APIClass>
const char* NpActorTemplate<APIClass>::getName() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));
	return mName;
}

///////////////////////////////////////////////////////////////////////////////

template<class APIClass>
void NpActorTemplate<APIClass>::setDominanceGroup(PxDominanceGroup dominanceGroup)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	NpActor::getScbFromPxActor(*this).setDominanceGroup(dominanceGroup);
}


template<class APIClass>
PxDominanceGroup NpActorTemplate<APIClass>::getDominanceGroup() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));
	return NpActor::getScbFromPxActor(*this).getDominanceGroup();
}

///////////////////////////////////////////////////////////////////////////////

template<class APIClass>
void NpActorTemplate<APIClass>::setOwnerClient( PxClientID inId )
{
	if ( NpActor::getOwnerScene(*this) != NULL )
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, 
				"Attempt to set the client id when an actor is already in a scene.");
	}
	else
		NpActor::getScbFromPxActor(*this).setOwnerClient( inId );
}

template<class APIClass>
PxClientID NpActorTemplate<APIClass>::getOwnerClient() const
{
	return NpActor::getScbFromPxActor(*this).getOwnerClient();
}

///////////////////////////////////////////////////////////////////////////////

template<class APIClass>
PX_FORCE_INLINE void NpActorTemplate<APIClass>::setActorFlagInternal(PxActorFlag::Enum flag, bool value)
{
	Scb::Actor& a = NpActor::getScbFromPxActor(*this);
	if (value)
		a.setActorFlags( a.getActorFlags() | flag );
	else
		a.setActorFlags( a.getActorFlags() & (~PxActorFlags(flag)) );
}

template<class APIClass>
PX_FORCE_INLINE void NpActorTemplate<APIClass>::setActorFlagsInternal(PxActorFlags inFlags)
{
	Scb::Actor& a = NpActor::getScbFromPxActor(*this);
	a.setActorFlags( inFlags );
}

template<class APIClass>
void NpActorTemplate<APIClass>::setActorFlag(PxActorFlag::Enum flag, bool value)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));

	setActorFlagInternal(flag, value);
}

template<class APIClass>
void NpActorTemplate<APIClass>::setActorFlags(PxActorFlags inFlags)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	
	setActorFlagsInternal(inFlags);
}

template<class APIClass>
PxActorFlags NpActorTemplate<APIClass>::getActorFlags() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));
	return NpActor::getScbFromPxActor(*this).getActorFlags();
}

///////////////////////////////////////////////////////////////////////////////

}

#endif
