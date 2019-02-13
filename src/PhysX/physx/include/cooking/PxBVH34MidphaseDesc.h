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


#ifndef PX_BVH_34_MIDPHASE_DESC_H
#define PX_BVH_34_MIDPHASE_DESC_H
/** \addtogroup cooking
@{
*/

#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**

\brief Structure describing parameters affecting BVH34 midphase mesh structure.

@see PxCookingParams, PxMidphaseDesc
*/
struct PxBVH34MidphaseDesc
{
	/**
	\brief Mesh cooking hint for max primitives per leaf limit. 
	Less primitives per leaf produces larger meshes with better runtime performance 
	and worse cooking performance. More triangles per leaf results in faster cooking speed and
	smaller mesh sizes, but with worse runtime performance.

	<b>Default value:</b> 4
	<b>Range:</b> <4, 15>
	*/
	PxU32			numPrimsPerLeaf;

	/**
	\brief Desc initialization to default value.
	*/
    void setToDefault()
    {
		numPrimsPerLeaf = 4;
    }

	/**
	\brief Returns true if the descriptor is valid.
	\return true if the current settings are valid.
	*/
	bool isValid() const
	{
		if(numPrimsPerLeaf < 4 || numPrimsPerLeaf > 15)
			return false;
		return true;
	}
};

#if !PX_DOXYGEN
} // namespace physx
#endif


  /** @} */
#endif // PX_BVH_34_MIDPHASE_DESC_H
