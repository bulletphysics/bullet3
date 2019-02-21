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

#ifndef CM_RADIX_SORT_H
#define CM_RADIX_SORT_H

#include "PxPhysXCommonConfig.h"

namespace physx
{
namespace Cm
{

	enum RadixHint
	{
		RADIX_SIGNED,		//!< Input values are signed
		RADIX_UNSIGNED,		//!< Input values are unsigned

		RADIX_FORCE_DWORD = 0x7fffffff
	};

#define INVALIDATE_RANKS	mCurrentSize|=0x80000000
#define VALIDATE_RANKS		mCurrentSize&=0x7fffffff
#define CURRENT_SIZE		(mCurrentSize&0x7fffffff)
#define INVALID_RANKS		(mCurrentSize&0x80000000)

	class PX_PHYSX_COMMON_API RadixSort
	{
		public:
										RadixSort();
		virtual							~RadixSort();
		// Sorting methods
						RadixSort&		Sort(const PxU32* input, PxU32 nb, RadixHint hint=RADIX_SIGNED);
						RadixSort&		Sort(const float* input, PxU32 nb);

		//! Access to results. mRanks is a list of indices in sorted order, i.e. in the order you may further process your data
		PX_FORCE_INLINE	const PxU32*	GetRanks()			const	{ return mRanks;		}

		//! mIndices2 gets trashed on calling the sort routine, but otherwise you can recycle it the way you want.
		PX_FORCE_INLINE	PxU32*			GetRecyclable()		const	{ return mRanks2;		}

		//! Returns the total number of calls to the radix sorter.
		PX_FORCE_INLINE	PxU32			GetNbTotalCalls()	const	{ return mTotalCalls;	}
		//! Returns the number of eraly exits due to temporal coherence.
		PX_FORCE_INLINE	PxU32			GetNbHits()			const	{ return mNbHits;		}

		PX_FORCE_INLINE	void			invalidateRanks()			{ INVALIDATE_RANKS;		}

						bool			SetBuffers(PxU32* ranks0, PxU32* ranks1, PxU32* histogram1024, PxU32** links256);
		private:
										RadixSort(const RadixSort& object);
										RadixSort& operator=(const RadixSort& object);
		protected:
						PxU32			mCurrentSize;		//!< Current size of the indices list
						PxU32*			mRanks;				//!< Two lists, swapped each pass
						PxU32*			mRanks2;
						PxU32*			mHistogram1024;
						PxU32**			mLinks256;
		// Stats
						PxU32			mTotalCalls;		//!< Total number of calls to the sort routine
						PxU32			mNbHits;			//!< Number of early exits due to coherence

						// Stack-radix
						bool			mDeleteRanks;		//!<
	};

	#define StackRadixSort(name, ranks0, ranks1)	\
		RadixSort name;								\
		PxU32 histogramBuffer[1024];				\
		PxU32* linksBuffer[256];					\
		name.SetBuffers(ranks0, ranks1, histogramBuffer, linksBuffer);
}

}

#endif // CM_RADIX_SORT_H
