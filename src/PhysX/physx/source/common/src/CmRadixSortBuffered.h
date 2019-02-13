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

#ifndef CM_RADIX_SORT_BUFFERED_H
#define CM_RADIX_SORT_BUFFERED_H

#include "CmPhysXCommon.h"
#include "CmRadixSort.h"

namespace physx
{
namespace Cm
{
	class PX_PHYSX_COMMON_API RadixSortBuffered : public RadixSort
	{
	public:
							RadixSortBuffered();
							~RadixSortBuffered();

		void				reset();

		RadixSortBuffered&	Sort(const PxU32* input, PxU32 nb, RadixHint hint=RADIX_SIGNED);
		RadixSortBuffered&	Sort(const float* input, PxU32 nb);

	private:
							RadixSortBuffered(const RadixSortBuffered& object);
							RadixSortBuffered& operator=(const RadixSortBuffered& object);

		// Internal methods
		void				CheckResize(PxU32 nb);
		bool				Resize(PxU32 nb);
	};
}

}

#endif // CM_RADIX_SORT_BUFFERED_H
