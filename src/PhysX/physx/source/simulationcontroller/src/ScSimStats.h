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


#ifndef PX_PHYSICS_SCP_SIM_STATS
#define PX_PHYSICS_SCP_SIM_STATS

#include "PsAtomic.h"
#include "PsUserAllocated.h"
#include "CmPhysXCommon.h"
#include "PxGeometry.h"
#include "PxSimulationStatistics.h"

namespace physx
{

struct PxvSimStats;

namespace Sc
{

	/*
	Description: contains statistics for the scene.
	*/
	class SimStats : public Ps::UserAllocated
	{
	public:
		SimStats();

		void clear();		//set counters to zero
		void simStart();
		void readOut(PxSimulationStatistics& dest, const PxvSimStats& simStats) const;

		PX_INLINE void incBroadphaseAdds()
		{
			numBroadPhaseAddsPending++;
		}

		PX_INLINE void incBroadphaseRemoves()
		{
			numBroadPhaseRemovesPending++;
		}

	private:
		// Broadphase adds/removes for the current simulation step
		PxU32 numBroadPhaseAdds;
		PxU32 numBroadPhaseRemoves;

		// Broadphase adds/removes for the next simulation step
		PxU32 numBroadPhaseAddsPending;
		PxU32 numBroadPhaseRemovesPending;

	public:
		typedef PxI32 TriggerPairCountsNonVolatile[PxGeometryType::eCONVEXMESH+1][PxGeometryType::eGEOMETRY_COUNT];
		typedef volatile TriggerPairCountsNonVolatile TriggerPairCounts;
		TriggerPairCounts numTriggerPairs;
	};

} // namespace Sc

}

#endif
