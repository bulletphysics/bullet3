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


         
#ifndef PXC_NPWORKUNIT_H
#define PXC_NPWORKUNIT_H

#include "PxcNpThreadContext.h"
#include "PxcMaterialMethodImpl.h"
#include "PxcNpCache.h"

namespace physx
{

class PxvContact;

struct PxsRigidCore;
struct PxsShapeCore;

class PxsMaterialManager;

struct PxcNpWorkUnitFlag
{
	enum Enum
	{
		eOUTPUT_CONTACTS			= 1,
		eOUTPUT_CONSTRAINTS			= 2,
		eDISABLE_STRONG_FRICTION	= 4,
		eARTICULATION_BODY0			= 8,
		eARTICULATION_BODY1			= 16,
		eDYNAMIC_BODY0				= 32,
		eDYNAMIC_BODY1				= 64,
		eMODIFIABLE_CONTACT			= 128,
		eFORCE_THRESHOLD			= 256,
		eDETECT_DISCRETE_CONTACT	= 512,
		eHAS_KINEMATIC_ACTOR		= 1024,
		eDISABLE_RESPONSE			= 2048,
		eDETECT_CCD_CONTACTS		= 4096
	};
};

struct PxcNpWorkUnitStatusFlag
{
	enum Enum
	{
		eHAS_NO_TOUCH				= (1 << 0),
		eHAS_TOUCH					= (1 << 1),
		//eHAS_SOLVER_CONSTRAINTS		= (1 << 2),
		eREQUEST_CONSTRAINTS		= (1 << 3),
		eHAS_CCD_RETOUCH			= (1 << 4),	// Marks pairs that are touching at a CCD pass and were touching at discrete collision or at a previous CCD pass already
												// but we can not tell whether they lost contact in a pass before. We send them as pure eNOTIFY_TOUCH_CCD events to the 
												// contact report callback if requested.
		eDIRTY_MANAGER				= (1 << 5),
		eREFRESHED_WITH_TOUCH		= (1 << 6),
		eTOUCH_KNOWN				= eHAS_NO_TOUCH | eHAS_TOUCH	// The touch status is known (if narrowphase never ran for a pair then no flag will be set)
	};
};

/*
 * A struct to record the number of work units a particular constraint pointer references.
 * This is created at the beginning of the constriant data and is used to bypass constraint preparation when the 
 * bodies are not moving a lot. In this case, we can recycle the constraints and save ourselves some cycles.
*/
struct PxcNpWorkUnit;
struct PxcNpWorkUnitBatch
{
	PxcNpWorkUnit* mUnits[4];
	PxU32 mSize;
};

struct PxcNpWorkUnit
{
	
	const PxsRigidCore*		rigidCore0;					// INPUT								//4		//8
	const PxsRigidCore*		rigidCore1;					// INPUT								//8		//16
		
	const PxsShapeCore*		shapeCore0;					// INPUT								//12	//24
	const PxsShapeCore*		shapeCore1;					// INPUT								//16	//32

	PxU8*					ccdContacts;				// OUTPUT								//20	//40

	PxU8*					frictionDataPtr;			// INOUT								//24	//48

	PxU16					flags;						// INPUT								//26	//50
	PxU8					frictionPatchCount;			// INOUT 								//27	//51
	PxU8					statusFlags;				// OUTPUT (see PxcNpWorkUnitStatusFlag) //28	//52

	PxU8					dominance0;					// INPUT								//29	//53
	PxU8					dominance1;					// INPUT								//30	//54
	PxU8					geomType0;					// INPUT								//31	//55
	PxU8					geomType1;					// INPUT								//32	//56

	PxU32					index;						// INPUT								//36	//60

	PxReal					restDistance;				// INPUT								//40	//64

	PxU32					mTransformCache0;			//										//44	//68
	PxU32					mTransformCache1;			//										//48	//72
	
	PxU32					mEdgeIndex;					//inout the island gen edge index		//52	//76
	PxU32					mNpIndex;					//INPUT									//56	//80

	PxReal					mTorsionalPatchRadius;												//60	//84
	PxReal					mMinTorsionalPatchRadius;											//64	//88

};

//#if !defined(PX_P64)
//PX_COMPILE_TIME_ASSERT(0 == (sizeof(PxcNpWorkUnit) & 0x0f));
//#endif

PX_FORCE_INLINE void PxcNpWorkUnitClearContactState(PxcNpWorkUnit& n)
{
	n.ccdContacts = NULL;
}


PX_FORCE_INLINE void PxcNpWorkUnitClearCachedState(PxcNpWorkUnit& n)
{
	n.frictionDataPtr = 0;
	n.frictionPatchCount = 0;
	n.ccdContacts = NULL;
}

PX_FORCE_INLINE void PxcNpWorkUnitClearFrictionCachedState(PxcNpWorkUnit& n)
{
	n.frictionDataPtr = 0;
	n.frictionPatchCount = 0;
	n.ccdContacts = NULL;
}

#if !defined(PX_P64)
//PX_COMPILE_TIME_ASSERT(sizeof(PxcNpWorkUnit)==128);
#endif

}

#endif
