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

#ifndef PXS_CONTACT_MANAGER_STATE_H
#define PXS_CONTACT_MANAGER_STATE_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{

	struct PxsShapeCore;

	/**
	There is an implicit 1:1 mapping between PxgContactManagerInput and PxsContactManagerOutput. The structures are split because PxgNpContactManagerInput contains constant
	data that is produced by the CPU code and PxgNpContactManagerOutput contains per-frame contact information produced by the NP.

	There is also a 1:1 mapping between the PxgNpContactManager and PxsContactManager. This mapping is handled within the PxgNPhaseCore.

	The previous contact states are implicitly cached in PxsContactManager and will be propagated to the solver. Friction correlation is also done implicitly using cached 
	information in PxsContactManager.
	The NP will produce a list of pairs that found/lost patches for the solver along with updating the PxgNpContactManagerOutput for all pairs.
	*/

	struct PxsContactManagerStatusFlag
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
			eTOUCH_KNOWN				= eHAS_NO_TOUCH | eHAS_TOUCH	// The touch status is known (if narrowphase never ran for a pair then no flag will be set)
		};
	};
		

	struct PX_ALIGN_PREFIX(16) PxsContactManagerOutput
	{
		PxU8* contactPatches;				//Start index/ptr for contact patches
		PxU8* contactPoints;				//Start index/ptr for contact points
		PxReal* contactForces;				//Start index/ptr for contact forces
		PxU8 nbContacts;					//Num contacts
		PxU8 nbPatches;						//Num patches
		PxU8 statusFlag;					//Status flag (has touch etc.)
		PxU8 prevPatches;					//Previous number of patches

		PX_FORCE_INLINE PxU32* getInternalFaceIndice()
		{
			return reinterpret_cast<PxU32*>(contactForces + nbContacts);
		}
	} 
	PX_ALIGN_SUFFIX(16);

	struct /*PX_ALIGN_PREFIX(16)*/ PxsContactManagerPersistency
	{
		PxU8 mPrevPatches;
		PxU8 mNbFrictionPatches;
		PxU8 mNbPrevFrictionPatches;
	}
	/*PX_ALIGN_SUFFIX(16)*/;

}

#endif //PXG_CONTACT_MANAGER_H
