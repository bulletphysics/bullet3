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

#ifndef GU_HEIGHTFIELD_DATA_H
#define GU_HEIGHTFIELD_DATA_H

#include "foundation/PxSimpleTypes.h"
#include "PxHeightFieldFlag.h"
#include "PxHeightFieldSample.h"
#include "GuCenterExtents.h"

namespace physx
{

namespace Gu
{

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4251 ) // class needs to have dll-interface to be used by clients of class
#endif
struct PX_PHYSX_COMMON_API HeightFieldData
{
// PX_SERIALIZATION
	PX_FORCE_INLINE								HeightFieldData()									{}
	PX_FORCE_INLINE								HeightFieldData(const PxEMPTY) :	flags(PxEmpty)	{}
//~PX_SERIALIZATION

	//properties
		// PT: WARNING: bounds must be followed by at least 32bits of data for safe SIMD loading
					CenterExtents				mAABB;
					PxU32						rows;					// PT: WARNING: don't change this member's name (used in ConvX)
					PxU32						columns;				// PT: WARNING: don't change this member's name (used in ConvX)
					PxReal						rowLimit;				// PT: to avoid runtime int-to-float conversions on Xbox
					PxReal						colLimit;				// PT: to avoid runtime int-to-float conversions on Xbox
					PxReal						nbColumns;				// PT: to avoid runtime int-to-float conversions on Xbox
					PxHeightFieldSample*		samples;				// PT: WARNING: don't change this member's name (used in ConvX)
					PxReal						convexEdgeThreshold;

					PxHeightFieldFlags			flags;

					PxHeightFieldFormat::Enum	format;

	PX_FORCE_INLINE	const CenterExtentsPadded&	getPaddedBounds()				const
												{
													// PT: see compile-time assert below
													return static_cast<const CenterExtentsPadded&>(mAABB);
												}
};
#if PX_VC 
     #pragma warning(pop) 
#endif

	// PT: 'getPaddedBounds()' is only safe if we make sure the bounds member is followed by at least 32bits of data
	PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF(Gu::HeightFieldData, rows)>=PX_OFFSET_OF(Gu::HeightFieldData, mAABB)+4);

} // namespace Gu

}

#endif
