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


#ifndef PXV_DYNAMICS_H
#define PXV_DYNAMICS_H

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "foundation/PxSimpleTypes.h"
#include "PsIntrinsics.h"
#include "PxRigidDynamic.h"

namespace physx
{

/*!
\file
Dynamics interface.
*/

/************************************************************************/
/* Atoms                                                                */
/************************************************************************/

class PxsContext;
class PxsRigidBody;
class PxShape;
class PxGeometry;
struct PxsShapeCore;


struct PxsRigidCore
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

	PxsRigidCore() : mFlags(0), mIdtBody2Actor(0), solverIterationCounts(0)	{}
	PxsRigidCore(const PxEMPTY) : mFlags(PxEmpty)							{}

	PX_ALIGN_PREFIX(16)
	PxTransform			body2World PX_ALIGN_SUFFIX(16);
	PxRigidBodyFlags	mFlags;					// API body flags
	PxU8				mIdtBody2Actor;			// PT: true if PxsBodyCore::body2Actor is identity
	PxU16				solverIterationCounts;	//vel iters are in low word and pos iters in high word.

	PX_FORCE_INLINE	PxU32 isKinematic() const
	{
		return mFlags & PxRigidBodyFlag::eKINEMATIC;
	}

	PX_FORCE_INLINE PxU32 hasCCD() const
	{
		return mFlags & PxRigidBodyFlag::eENABLE_CCD;
	}

	PX_FORCE_INLINE	PxU32 hasCCDFriction() const
	{
		return mFlags & PxRigidBodyFlag::eENABLE_CCD_FRICTION;
	}
};
PX_COMPILE_TIME_ASSERT(sizeof(PxsRigidCore) == 32);


struct PxsBodyCore: public PxsRigidCore
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

	PxsBodyCore() : PxsRigidCore()							{}
	PxsBodyCore(const PxEMPTY) : PxsRigidCore(PxEmpty)		{}

	PX_FORCE_INLINE	const PxTransform& getBody2Actor()	const	{ return body2Actor;	}
	PX_FORCE_INLINE	void setBody2Actor(const PxTransform& t)
	{
		mIdtBody2Actor = PxU8(t.p.isZero() && t.q.isIdentity());

		body2Actor = t;
	}
	protected:
	PxTransform				body2Actor;
	public:
	PxReal					ccdAdvanceCoefficient;		//64

	PxVec3					linearVelocity;
	PxReal					maxPenBias;

	PxVec3					angularVelocity;
	PxReal					contactReportThreshold;		//96
    
	PxReal					maxAngularVelocitySq;
	PxReal					maxLinearVelocitySq;
	PxReal					linearDamping;
	PxReal					angularDamping;				//112

	PxVec3					inverseInertia;
	PxReal					inverseMass;				//128
	
	PxReal					maxContactImpulse;			
	PxReal					sleepThreshold;				   
	PxReal					freezeThreshold;			
	PxReal					wakeCounter;				//144 this is authoritative wakeCounter

	PxReal					solverWakeCounter;			//this is calculated by the solver when it performs sleepCheck. It is committed to wakeCounter in ScAfterIntegrationTask if the body is still awake.
	PxU32					numCountedInteractions;
	PxU32					numBodyInteractions;		//Used by adaptive force to keep track of the total number of body interactions
	PxU16					isFastMoving;				//This could be a single bit but it's a u16 at the moment for simplicity's sake
	PxRigidDynamicLockFlags	lockFlags;					//160 This could be a u8 but it is a u16 for simplicity's sake. All fits into 16 byte alignment

	PX_FORCE_INLINE	bool	shouldCreateContactReports()	const
	{
		const PxU32* binary = reinterpret_cast<const PxU32*>(&contactReportThreshold);
		return *binary != 0x7f7fffff;	// PX_MAX_REAL
	}
};

PX_COMPILE_TIME_ASSERT(sizeof(PxsBodyCore) == 160);


}

#endif
