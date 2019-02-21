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

#ifndef PXS_ISLAND_NODEINDEX_H
#define PXS_ISLAND_NODEINDEX_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{
namespace IG
{
#define IG_INVALID_NODE 0x1FFFFFFu

	class NodeIndex
	{
	private:
		PxU32 ind;

	public:

		explicit PX_CUDA_CALLABLE PX_FORCE_INLINE NodeIndex(PxU32 id, PxU32 articLinkId) : ind((id << 7) | (articLinkId << 1) | 1)
		{
		}

		explicit PX_CUDA_CALLABLE PX_FORCE_INLINE NodeIndex(PxU32 id = IG_INVALID_NODE) : ind((id << 7))
		{
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 index() const { return ind >> 7; }
		PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 articulationLinkId() const { return ((ind >> 1) & 63); }
		PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 isArticulation() const { return ind & 1; }

		PX_CUDA_CALLABLE PX_FORCE_INLINE bool isStaticBody() const { return (ind >> 7) == IG_INVALID_NODE; }

		PX_CUDA_CALLABLE bool isValid() const { return (ind >> 7) != IG_INVALID_NODE; }

		PX_CUDA_CALLABLE void setIndices(PxU32 index, PxU32 articLinkId) { ind = ((index << 7) | (articLinkId << 1) | 1); }

		PX_CUDA_CALLABLE void setIndices(PxU32 index) { ind = ((index << 7)); }

		PX_CUDA_CALLABLE bool operator < (const IG::NodeIndex& other) const { return ind < other.ind; }

		PX_CUDA_CALLABLE bool operator <= (const IG::NodeIndex& other) const { return ind <= other.ind; }

		PX_CUDA_CALLABLE bool operator == (const IG::NodeIndex& other) const { return ind == other.ind; }
	};

}
}

#endif
