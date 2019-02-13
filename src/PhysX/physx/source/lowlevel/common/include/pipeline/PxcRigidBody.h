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


#ifndef PXC_RIGIDBODY_H
#define PXC_RIGIDBODY_H

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "PxvDynamics.h"
#include "CmSpatialVector.h"

namespace physx
{

class PxsContactManager;
struct PxsCCDPair;
struct PxsCCDBody;

#define PX_INTERNAL_LOCK_FLAG_START 8

PX_ALIGN_PREFIX(16)
class PxcRigidBody
{
public:

	enum PxcRigidBodyFlag
	{
		eFROZEN =						1 << 0,			//This flag indicates that the stabilization is enabled and the body is
														//"frozen". By "frozen", we mean that the body's transform is unchanged
														//from the previous frame. This permits various optimizations.
		eFREEZE_THIS_FRAME =			1 << 1,
		eUNFREEZE_THIS_FRAME =			1 << 2,
		eACTIVATE_THIS_FRAME =			1 << 3,
		eDEACTIVATE_THIS_FRAME =		1 << 4,
		eDISABLE_GRAVITY =				1 << 5,
		eSPECULATIVE_CCD =				1 << 6,
		//KS - copied here for GPU simulation to avoid needing to pass another set of flags around.
		eLOCK_LINEAR_X =				1 << (PX_INTERNAL_LOCK_FLAG_START),
		eLOCK_LINEAR_Y =				1 << (PX_INTERNAL_LOCK_FLAG_START + 1),
		eLOCK_LINEAR_Z =				1 << (PX_INTERNAL_LOCK_FLAG_START + 2),
		eLOCK_ANGULAR_X =				1 << (PX_INTERNAL_LOCK_FLAG_START + 3),
		eLOCK_ANGULAR_Y =				1 << (PX_INTERNAL_LOCK_FLAG_START + 4),
		eLOCK_ANGULAR_Z =				1 << (PX_INTERNAL_LOCK_FLAG_START + 5)
		

	};

	PX_FORCE_INLINE PxcRigidBody(PxsBodyCore* core)  
	: mLastTransform(core->body2World),
	  mCCD(NULL),
	  mCore(core)
	{
	}

	void						adjustCCDLastTransform();

protected:
	
	~PxcRigidBody()
	{
	}

public:
	
	PxTransform					mLastTransform;				//28 (28)

	PxU16						mInternalFlags;				//30 (30)
	PxU16						solverIterationCounts;		//32 (32)

	PxsCCDBody*					mCCD;						//36 (40)	// only valid during CCD	

	PxsBodyCore*				mCore;						//40 (48)
	
#if !PX_P64_FAMILY
	PxU32						alignmentPad[2];			//48 (48)
#endif

	PxVec3						sleepLinVelAcc;				//60 (60)
	PxReal						freezeCount;				//64 (64)
	   
	PxVec3						sleepAngVelAcc;				//76 (76)
	PxReal						accelScale;					//80 (80)
	

}
PX_ALIGN_SUFFIX(16);
PX_COMPILE_TIME_ASSERT(0 == (sizeof(PxcRigidBody) & 0x0f));

}

#endif //PXC_RIGIDBODY_H
