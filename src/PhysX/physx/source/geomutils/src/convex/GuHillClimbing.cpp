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

#include "foundation/PxVec3.h"
#include "foundation/PxAssert.h"
#include "PsUserAllocated.h"
#include "CmPhysXCommon.h"
#include "GuHillClimbing.h"
#include "GuBigConvexData2.h"

namespace physx
{

void localSearch(PxU32& id, const PxVec3& dir, const PxVec3* verts, const Gu::BigConvexRawData* val)
{
	// WARNING: there is a problem on x86 with a naive version of this code, where truncation
	// of values from 80 bits to 32 bits as they're stored in memory means that iteratively moving to 
	// an adjacent vertex of greater support can go into an infinite loop. So we use a version which 
	// never visits a vertex twice. Note - this might not be enough for GJK, since local
	// termination of the support function might not be enough to ensure convergence of GJK itself.

	// if we got here, we'd better have vertices and valencies
	PX_ASSERT(verts && val);

	class TinyBitMap
	{
	public:
		PxU32 m[8];
		PX_FORCE_INLINE TinyBitMap()			{ m[0] = m[1] = m[2] = m[3] = m[4] = m[5] = m[6] = m[7] = 0;	}
		PX_FORCE_INLINE void set(PxU8 v)		{ m[v>>5] |= 1<<(v&31);											}
		PX_FORCE_INLINE bool get(PxU8 v) const	{ return (m[v>>5] & 1<<(v&31)) != 0;							}
	};

	TinyBitMap visited;

	const Gu::Valency* Valencies	= val->mValencies;
	const PxU8* Adj					= val->mAdjacentVerts;

	PX_ASSERT(Valencies && Adj);

	// Get the initial value and the initial vertex
	float MaxVal	= dir.dot(verts[id]);
	PxU32 NextVtx	= id;

	do
	{
		PxU16 NbNeighbors = Valencies[NextVtx].mCount;
		const PxU8* Run	= Adj + Valencies[NextVtx].mOffset;
		id = NextVtx;
		while(NbNeighbors--)
		{
			const PxU8 Neighbor = *Run++;

			if(!visited.get(Neighbor))
			{
				visited.set(Neighbor);

				const float CurVal = dir.dot(verts[Neighbor]);

				if(CurVal>MaxVal)
				{
					MaxVal	= CurVal;
					NextVtx	= Neighbor;
				}
			}
		}
	} while(NextVtx!=id);
}

}
