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

#ifndef PX_PHYSICS_SCB_CONSTRAINTSHADER
#define PX_PHYSICS_SCB_CONSTRAINTSHADER

#include "CmPhysXCommon.h"
#include "../../../simulationcontroller/include/ScConstraintCore.h"

#include "ScbBody.h"

namespace physx
{

namespace Sc
{
	class RigidCore;
}

namespace Scb
{

struct ConstraintBuffer
{
public:
	Sc::RigidCore*		rigids[2];
	PxReal				linBreakForce;
	PxReal				angBreakForce;
	PxConstraintFlags	flags;
	PxReal				minResponseThreshold;
};

enum ConstraintBufferFlag
{
	BF_BODIES					= (1 << 0),
	BF_BREAK_IMPULSE			= (1 << 1),
	BF_FLAGS					= (1 << 2),
	BF_MIN_RESPONSE_THRESHOLD	= (1 << 3)
};

class Constraint : public Base, public Ps::UserAllocated
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

public:

	typedef ConstraintBuffer Buf;
	typedef Sc::ConstraintCore Core;

// PX_SERIALIZATION
										Constraint(const PxEMPTY) :	Base(PxEmpty), mConstraint(PxEmpty) {}
	static		void					getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
	PX_INLINE							Constraint(PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize);
	PX_INLINE							~Constraint() {}

	//---------------------------------------------------------------------------------
	// Wrapper for Sc::ConstraintCore interface
	//---------------------------------------------------------------------------------

	PX_INLINE PxConstraint*				getPxConstraint() const;
	PX_INLINE PxConstraintConnector*	getPxConnector() const;

	PX_INLINE void						setFlags(PxConstraintFlags f);
	PX_INLINE PxConstraintFlags			getFlags() const;

	PX_INLINE void						setBodies(Scb::RigidObject* r0, Scb::RigidObject* r1);

	PX_INLINE void						getForce(PxVec3& force, PxVec3& torque) const;

	PX_INLINE void						setBreakForce(PxReal linear, PxReal angular);
	PX_INLINE void						getBreakForce(PxReal& linear, PxReal& angular) const;

	PX_INLINE void						setMinResponseThreshold(PxReal threshold);
	PX_INLINE PxReal					getMinResponseThreshold() const;

	PX_INLINE bool						updateConstants(void* addr);

	//---------------------------------------------------------------------------------
	// Data synchronization
	//---------------------------------------------------------------------------------
	PX_INLINE void						prepareForActorRemoval();
	PX_INLINE void						syncState();

	//---------------------------------------------------------------------------------
	// Miscellaneous
	//---------------------------------------------------------------------------------
	PX_FORCE_INLINE const Core&			getScConstraint()				const	{ return mConstraint;	}  // Only use if you know what you're doing!
	PX_FORCE_INLINE Core&				getScConstraint()						{ return mConstraint;	}  // Only use if you know what you're doing!

	PX_FORCE_INLINE static Constraint&			fromSc(Core &a)					{ return *reinterpret_cast<Constraint*>(reinterpret_cast<PxU8*>(&a)-getScOffset()); }
	PX_FORCE_INLINE static const Constraint&	fromSc(const Core &a)			{ return *reinterpret_cast<const Constraint*>(reinterpret_cast<const PxU8*>(&a)-getScOffset()); }

	static size_t getScOffset()	{ return reinterpret_cast<size_t>(&reinterpret_cast<Constraint*>(0)->mConstraint);	}

private:
	Core		mConstraint;
	
	//---------------------------------------------------------------------------------
	// Permanently buffered data (simulation written data)
	//---------------------------------------------------------------------------------
	PxVec3					mBufferedForce;
	PxVec3					mBufferedTorque;
	PxConstraintFlags		mBrokenFlag;

	PX_FORCE_INLINE	const Buf*		getBufferedData()	const	{ return reinterpret_cast<const Buf*>(getStream()); }
	PX_FORCE_INLINE	Buf*			getBufferedData()			{ return reinterpret_cast<Buf*>(getStream());		}
};

}  // namespace Scb

PX_INLINE Scb::Constraint::Constraint(PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize) :
	mConstraint		(connector, shaders, dataSize),
	mBufferedForce	(0.0f),
	mBufferedTorque	(0.0f),
	mBrokenFlag		(0)
{
	setScbType(ScbType::eCONSTRAINT);
}

PX_INLINE PxConstraintConnector* Scb::Constraint::getPxConnector() const
{
	return mConstraint.getPxConnector();
}

PX_INLINE void Scb::Constraint::setFlags(PxConstraintFlags f)
{
	if(!isBuffering())
	{
		mConstraint.setFlags(f);
		UPDATE_PVD_PROPERTIES_OBJECT()
	}
	else
	{
		getBufferedData()->flags = f;
		markUpdated(BF_FLAGS);
	}
}

PX_INLINE PxConstraintFlags Scb::Constraint::getFlags() const
{
	return isBuffered(BF_FLAGS) ? getBufferedData()->flags & (~(PxConstraintFlag::eBROKEN | PxConstraintFlag::eGPU_COMPATIBLE) | mBrokenFlag)
								: mConstraint.getFlags() & (~(PxConstraintFlag::eBROKEN | PxConstraintFlag::eGPU_COMPATIBLE) | mBrokenFlag);
}

PX_INLINE void Scb::Constraint::setBodies(Scb::RigidObject* r0, Scb::RigidObject* r1)
{
	Sc::RigidCore* scR0 = r0 ? &r0->getScRigidCore() : NULL;
	Sc::RigidCore* scR1 = r1 ? &r1->getScRigidCore() : NULL;

	if(!isBuffering())
	{
		mConstraint.prepareForSetBodies();
		mConstraint.setBodies(scR0, scR1);
		UPDATE_PVD_PROPERTIES_OBJECT()
	}
	else
	{
		Buf* PX_RESTRICT bufferedData = getBufferedData();
		bufferedData->rigids[0] = scR0;
		bufferedData->rigids[1] = scR1;
		markUpdated(BF_BODIES);
	}

	mBufferedForce = PxVec3(0.0f);
	mBufferedTorque = PxVec3(0.0f);
}

PX_INLINE void Scb::Constraint::getForce(PxVec3& force, PxVec3& torque) const
{
	force = mBufferedForce;
	torque = mBufferedTorque;
}

PX_INLINE void Scb::Constraint::setBreakForce(PxReal linear, PxReal angular)
{
	if(!isBuffering())
	{
		mConstraint.setBreakForce(linear, angular);
		UPDATE_PVD_PROPERTIES_OBJECT()
	}
	else
	{
		Buf* PX_RESTRICT bufferedData = getBufferedData();
		bufferedData->linBreakForce = linear;
		bufferedData->angBreakForce = angular;
		markUpdated(BF_BREAK_IMPULSE);
	}
}

PX_INLINE void Scb::Constraint::getBreakForce(PxReal& linear, PxReal& angular) const
{
	if(isBuffered(BF_BREAK_IMPULSE))
	{
		const Buf* PX_RESTRICT bufferedData = getBufferedData();
		linear = bufferedData->linBreakForce;
		angular = bufferedData->angBreakForce;
	}
	else
		mConstraint.getBreakForce(linear, angular);
}

PX_INLINE void Scb::Constraint::setMinResponseThreshold(PxReal threshold)
{
	if(!isBuffering())
	{
		mConstraint.setMinResponseThreshold(threshold);
		UPDATE_PVD_PROPERTIES_OBJECT()
	}
	else
	{
		Buf* PX_RESTRICT bufferedData = getBufferedData();
		bufferedData->minResponseThreshold = threshold;
		markUpdated(BF_MIN_RESPONSE_THRESHOLD);
	}
}

PX_INLINE PxReal Scb::Constraint::getMinResponseThreshold() const
{
	if(isBuffered(BF_MIN_RESPONSE_THRESHOLD))
	{
		const Buf* PX_RESTRICT bufferedData = getBufferedData();
		return bufferedData->minResponseThreshold;
	}
	else
		return mConstraint.getMinResponseThreshold();
}

PX_INLINE bool Scb::Constraint::updateConstants(void* addr)
{
	PX_ASSERT(!getScbScene()->isPhysicsBuffering());

	return mConstraint.updateConstants(addr);
}

//--------------------------------------------------------------
//
// Data synchronization
//
//--------------------------------------------------------------

PX_INLINE void Scb::Constraint::prepareForActorRemoval()
{
	// when the bodies of a constraint have been changed during buffering, it's possible the
	// attached actor is going to get deleted. Sc expects that all interactions with that actor
	// will have been removed, so we give the Sc::Constraint a chance to ensure that before
	// the actors go away.
	if(getBufferFlags() & BF_BODIES)
		mConstraint.prepareForSetBodies();
}

PX_INLINE void Scb::Constraint::syncState()
{
	//!!! Force has to be synced every frame (might want to have a list of active constraint shaders?)
	mConstraint.getForce(mBufferedForce, mBufferedTorque);
	
	mBrokenFlag = mConstraint.getFlags() & PxConstraintFlag::eBROKEN;

	const PxU32 flags = getBufferFlags();
	if(flags)
	{
		const Buf* PX_RESTRICT bufferedData = getBufferedData();

		if(flags & BF_BODIES)
			mConstraint.setBodies(bufferedData->rigids[0], bufferedData->rigids[1]);

		if(flags & BF_BREAK_IMPULSE)
			mConstraint.setBreakForce(bufferedData->linBreakForce, bufferedData->angBreakForce);

		if(flags & BF_MIN_RESPONSE_THRESHOLD)
			mConstraint.setMinResponseThreshold(bufferedData->minResponseThreshold);

		if(flags & BF_FLAGS)
			mConstraint.setFlags(bufferedData->flags | mBrokenFlag);
	}

	postSyncState();
}

}

#endif
