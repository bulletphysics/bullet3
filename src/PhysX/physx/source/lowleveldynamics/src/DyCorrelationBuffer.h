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



#ifndef DY_CORRELATIONBUFFER_H
#define DY_CORRELATIONBUFFER_H

#include "PxvConfig.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "DyFrictionPatch.h"
#include "GuContactBuffer.h"
#include "foundation/PxBounds3.h"

namespace physx
{

struct PxcNpWorkUnit;
struct PxsMaterialInfo;

namespace Dy
{

struct CorrelationBuffer
{
	static const PxU32 MAX_FRICTION_PATCHES = 32;
	static const PxU16 LIST_END = 0xffff;

	struct ContactPatchData
	{
		PxU16 start;
		PxU16 next;
		PxU8 flags;
		PxU8 count;
		PxReal staticFriction, dynamicFriction, restitution;
		PxBounds3 patchBounds;
	};

	// we can have as many contact patches as contacts, unfortunately
	ContactPatchData	contactPatches[Gu::ContactBuffer::MAX_CONTACTS];

	FrictionPatch	PX_ALIGN(16, frictionPatches[MAX_FRICTION_PATCHES]);
	PxVec3				PX_ALIGN(16, frictionPatchWorldNormal[MAX_FRICTION_PATCHES]);
	PxBounds3		patchBounds[MAX_FRICTION_PATCHES];

	PxU32				frictionPatchContactCounts[MAX_FRICTION_PATCHES];
	PxU32				correlationListHeads[MAX_FRICTION_PATCHES+1];

	// contact IDs are only used to identify auxiliary contact data when velocity
	// targets have been set. 
	PxU16				contactID[MAX_FRICTION_PATCHES][2];

	PxU32 contactPatchCount, frictionPatchCount;

};

bool createContactPatches(CorrelationBuffer& fb, const Gu::ContactPoint* cb, PxU32 contactCount, PxReal normalTolerance);

bool correlatePatches(CorrelationBuffer& fb, 
					  const Gu::ContactPoint* cb,
					  const PxTransform& bodyFrame0,
					  const PxTransform& bodyFrame1,
					  PxReal normalTolerance,
					  PxU32 startContactPatchIndex,
					  PxU32 startFrictionPatchIndex);

void growPatches(CorrelationBuffer& fb,
				 const Gu::ContactPoint* buffer,
				 const PxTransform& bodyFrame0,
				 const PxTransform& bodyFrame1,
				 PxReal normalTolerance,
				 PxU32 frictionPatchStartIndex,
				 PxReal frictionOffsetThreshold);

}

}

#endif //DY_CORRELATIONBUFFER_H
