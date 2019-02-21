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



#ifndef PXC_FRICTIONPATCH_H
#define PXC_FRICTIONPATCH_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "PxvConfig.h"

namespace physx
{

namespace Dy
{

struct FrictionPatch
{
	PxU8				broken;				// PT: must be first byte of struct, see "frictionBrokenWritebackByte"
	PxU8				materialFlags;
	PxU16				anchorCount;
	PxReal				restitution;
	PxReal				staticFriction;
	PxReal				dynamicFriction;
	PxVec3				body0Normal;
	PxVec3				body1Normal;
	PxVec3				body0Anchors[2];
	PxVec3				body1Anchors[2];
	PxQuat				relativeQuat;

	PX_FORCE_INLINE	void	operator = (const FrictionPatch& other)
	{
		broken = other.broken;
		materialFlags = other.materialFlags;
		anchorCount = other.anchorCount;
		body0Normal = other.body0Normal;
		body1Normal = other.body1Normal;
		body0Anchors[0] = other.body0Anchors[0];   
		body0Anchors[1] = other.body0Anchors[1];
		body1Anchors[0] = other.body1Anchors[0];
		body1Anchors[1] = other.body1Anchors[1];
		relativeQuat = other.relativeQuat;
		restitution = other.restitution;
		staticFriction = other.staticFriction;
		dynamicFriction = other.dynamicFriction;
	}
};  

//PX_COMPILE_TIME_ASSERT(sizeof(FrictionPatch)==80);

}

}

#endif
