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


#include "foundation/PxBounds3.h"
#include "PxBroadPhaseExt.h"
#include "PsFoundation.h"
#include "CmPhysXCommon.h"

using namespace physx;

PxU32 PxBroadPhaseExt::createRegionsFromWorldBounds(PxBounds3* regions, const PxBounds3& globalBounds, PxU32 nbSubdiv, PxU32 upAxis)
{
	PX_CHECK_MSG(globalBounds.isValid(), "PxBroadPhaseExt::createRegionsFromWorldBounds(): invalid bounds provided!");
	PX_CHECK_MSG(upAxis<3, "PxBroadPhaseExt::createRegionsFromWorldBounds(): invalid up-axis provided!");

	const PxVec3& min = globalBounds.minimum;
	const PxVec3& max = globalBounds.maximum;
	const float dx = (max.x - min.x) / float(nbSubdiv);
	const float dy = (max.y - min.y) / float(nbSubdiv);
	const float dz = (max.z - min.z) / float(nbSubdiv);
	PxU32 nbRegions = 0;
	PxVec3 currentMin, currentMax;
	for(PxU32 j=0;j<nbSubdiv;j++)
	{
		for(PxU32 i=0;i<nbSubdiv;i++)
		{
			if(upAxis==0)
			{
				currentMin = PxVec3(min.x, min.y + dy * float(i),   min.z + dz * float(j));
				currentMax = PxVec3(max.x, min.y + dy * float(i+1), min.z + dz * float(j+1));
			}
			else if(upAxis==1)
			{
				currentMin = PxVec3(min.x + dx * float(i),   min.y, min.z + dz * float(j));
				currentMax = PxVec3(min.x + dx * float(i+1), max.y, min.z + dz * float(j+1));
			}
			else if(upAxis==2)
			{
				currentMin = PxVec3(min.x + dx * float(i),   min.y + dy * float(j), min.z);
				currentMax = PxVec3(min.x + dx * float(i+1), min.y + dy * float(j+1), max.z);
			}

			regions[nbRegions++] = PxBounds3(currentMin, currentMax);
		}
	}
	return nbRegions;
}
