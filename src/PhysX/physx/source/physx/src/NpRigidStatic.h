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


#ifndef PX_PHYSICS_NP_RIGIDSTATIC
#define PX_PHYSICS_NP_RIGIDSTATIC

#include "NpRigidActorTemplate.h"
#include "PxRigidStatic.h"
#include "ScbRigidStatic.h"

#include "PxMetaData.h"

namespace physx
{

namespace Scb
{
	class RigidObject;
}

class NpRigidStatic;
typedef NpRigidActorTemplate<PxRigidStatic> NpRigidStaticT;

class NpRigidStatic : public NpRigidStaticT
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
// PX_SERIALIZATION
											NpRigidStatic(PxBaseFlags baseFlags) : NpRigidStaticT(baseFlags), mRigidStatic(PxEmpty) {}
	virtual			void					requiresObjects(PxProcessPxBaseCallback& c);
	static			NpRigidStatic*			createObject(PxU8*& address, PxDeserializationContext& context);
	static			void					getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION

	virtual									~NpRigidStatic();

	//---------------------------------------------------------------------------------
	// PxActor implementation
	//---------------------------------------------------------------------------------
	virtual			void					release();

	virtual			PxActorType::Enum		getType() const { return PxActorType::eRIGID_STATIC; }

	//---------------------------------------------------------------------------------
	// PxRigidActor implementation
	//---------------------------------------------------------------------------------

	// Pose
	virtual			void 					setGlobalPose(const PxTransform& pose, bool wake);
	virtual			PxTransform				getGlobalPose() const;

	//---------------------------------------------------------------------------------
	// Miscellaneous
	//---------------------------------------------------------------------------------
											NpRigidStatic(const PxTransform& pose);

	virtual			void					switchToNoSim();
	virtual			void					switchFromNoSim();

#if PX_CHECKED
	bool									checkConstraintValidity() const;
#endif

	PX_FORCE_INLINE	const Scb::Actor&		getScbActorFast()		const	{ return mRigidStatic;	}
	PX_FORCE_INLINE	Scb::Actor&				getScbActorFast()				{ return mRigidStatic;	}

	PX_FORCE_INLINE	const Scb::RigidStatic&	getScbRigidStaticFast()	const	{ return mRigidStatic;	}
	PX_FORCE_INLINE	Scb::RigidStatic&		getScbRigidStaticFast()			{ return mRigidStatic;	}

	PX_FORCE_INLINE	const PxTransform&		getGlobalPoseFast()		const	{ return mRigidStatic.getActor2World();	}

#if PX_ENABLE_DEBUG_VISUALIZATION
public:
					void					visualize(Cm::RenderOutput& out, NpScene* scene);
#endif

private:
					Scb::RigidStatic 		mRigidStatic;
};

}

#endif
