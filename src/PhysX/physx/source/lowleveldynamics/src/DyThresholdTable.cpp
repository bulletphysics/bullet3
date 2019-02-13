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

#include "foundation/PxMemory.h"
#include "DyThresholdTable.h"
#include "PsHash.h"
#include "PsUtilities.h"
#include "PsAllocator.h"

namespace physx
{
	namespace Dy
	{
		bool ThresholdTable::check(const ThresholdStream& stream, const PxU32 nodeIndexA, const PxU32 nodeIndexB, PxReal dt)
		{
			PxU32* PX_RESTRICT hashes = mHash;
			PxU32* PX_RESTRICT nextIndices = mNexts;
			Pair* PX_RESTRICT pairs = mPairs;

			/*const PxsRigidBody* b0 = PxMin(body0, body1);
			const PxsRigidBody* b1 = PxMax(body0, body1);*/

			const PxU32 nA = PxMin(nodeIndexA, nodeIndexB);
			const PxU32 nB = PxMax(nodeIndexA, nodeIndexB);

			PxU32 hashKey = computeHashKey(nodeIndexA, nodeIndexB, mHashSize);

			PxU32 pairIndex = hashes[hashKey];
			while(NO_INDEX != pairIndex)
			{
				Pair& pair = pairs[pairIndex];
				const PxU32 thresholdStreamIndex = pair.thresholdStreamIndex;
				PX_ASSERT(thresholdStreamIndex < stream.size());
				const ThresholdStreamElement& otherElement = stream[thresholdStreamIndex];
				if(otherElement.nodeIndexA.index()==nA && otherElement.nodeIndexB.index()==nB)
					return (pair.accumulatedForce > (otherElement.threshold * dt));
				pairIndex = nextIndices[pairIndex];
			}
			return false;
		}
	}
}
