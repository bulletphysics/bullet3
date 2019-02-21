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

#ifndef PX_PHYSICS_SCP_TRIGGER_PAIRS
#define PX_PHYSICS_SCP_TRIGGER_PAIRS

#include "PsArray.h"
#include "CmPhysXCommon.h"
#include "PxFiltering.h"
#include "PxClient.h"
#include "PxSimulationEventCallback.h"

namespace physx
{
class PxShape;

namespace Sc
{
	struct TriggerPairFlag
	{
		enum Enum
		{
			eTEST_FOR_REMOVED_SHAPES = PxTriggerPairFlag::eNEXT_FREE	// for cases where the pair got deleted because one of the shape volumes got removed from broadphase.
																		// This covers scenarios like volume re-insertion into broadphase as well since the shape might get removed
																		// after such an operation. The scenarios to consider are:
																		//
																		// - shape gets removed (this includes raising PxActorFlag::eDISABLE_SIMULATION)
																		// - shape switches to eSCENE_QUERY_SHAPE only
																		// - shape switches to eSIMULATION_SHAPE
																		// - resetFiltering()
																		// - actor gets removed from an aggregate
		};
	};

	PX_COMPILE_TIME_ASSERT((1 << (8*sizeof(PxTriggerPairFlags::InternalType))) > TriggerPairFlag::eTEST_FOR_REMOVED_SHAPES);

	struct TriggerPairExtraData
	{
		PX_INLINE TriggerPairExtraData() : 
			shape0ID(0xffffffff),
			shape1ID(0xffffffff),
			client0ID(0xff),
			client1ID(0xff)
		{
		}

		PX_INLINE TriggerPairExtraData(PxU32 s0ID, PxU32 s1ID,
										PxClientID cl0ID, PxClientID cl1ID) : 
			shape0ID(s0ID),
			shape1ID(s1ID),
			client0ID(cl0ID),
			client1ID(cl1ID)
		{
		}

		PxU32						shape0ID;
		PxU32						shape1ID;
		PxClientID					client0ID;
		PxClientID					client1ID;
	};

	typedef	Ps::Array<TriggerPairExtraData>	TriggerBufferExtraData;
	typedef	Ps::Array<PxTriggerPair>		TriggerBufferAPI;

} // namespace Sc

}

#endif
