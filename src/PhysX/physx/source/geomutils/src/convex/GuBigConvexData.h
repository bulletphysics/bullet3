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

#ifndef GU_BIG_CONVEX_DATA_H
#define GU_BIG_CONVEX_DATA_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{

class BigConvexDataBuilder;
class PxcHillClimb;
class BigConvexData;

// Data

namespace Gu
{

struct Valency
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

	PxU16		mCount;
	PxU16		mOffset;
};
PX_COMPILE_TIME_ASSERT(sizeof(Gu::Valency) == 4);

struct BigConvexRawData
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

	// Support vertex map
	PxU16		mSubdiv;		// "Gaussmap" subdivision
	PxU16		mNbSamples;		// Total #samples in gaussmap PT: this is not even needed at runtime!

	PxU8*		mSamples;
	PX_FORCE_INLINE const PxU8*	getSamples2()	const
	{
		return mSamples + mNbSamples;
	}
	//~Support vertex map

	// Valencies data
	PxU32			mNbVerts;		//!< Number of vertices
	PxU32			mNbAdjVerts;	//!< Total number of adjacent vertices  ### PT: this is useless at runtime and should not be stored here
	Gu::Valency*	mValencies;		//!< A list of mNbVerts valencies (= number of neighbors)
	PxU8*			mAdjacentVerts;	//!< List of adjacent vertices
	//~Valencies data
};
#if PX_P64_FAMILY
PX_COMPILE_TIME_ASSERT(sizeof(Gu::BigConvexRawData) == 40);
#else
PX_COMPILE_TIME_ASSERT(sizeof(Gu::BigConvexRawData) == 24);
#endif

} // namespace Gu

}

#endif
