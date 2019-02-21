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


#ifndef PX_BVH_33_MIDPHASE_DESC_H
#define PX_BVH_33_MIDPHASE_DESC_H
/** \addtogroup cooking
@{
*/

#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/** \brief Enumeration for mesh cooking hints. */
struct PxMeshCookingHint
{
	enum Enum
	{
		eSIM_PERFORMANCE = 0,		//!< Default value. Favors higher quality hierarchy with higher runtime performance over cooking speed.
		eCOOKING_PERFORMANCE = 1	//!< Enables fast cooking path at the expense of somewhat lower quality hierarchy construction.
	};
};

/**

\brief Structure describing parameters affecting BVH33 midphase mesh structure.

@see PxCookingParams, PxMidphaseDesc
*/
struct PxBVH33MidphaseDesc
{
	/**
	\brief Controls the trade-off between mesh size and runtime performance.

	Using a value of 1.0 will produce a larger cooked mesh with generally higher runtime performance,
	using 0.0 will produce a smaller cooked mesh, with generally lower runtime performance.

	Values outside of [0,1] range will be clamped and cause a warning when any mesh gets cooked.

	<b>Default value:</b> 0.55
	<b>Range:</b> [0.0f, 1.0f]
	*/
	PxF32							meshSizePerformanceTradeOff;

	/**
	\brief Mesh cooking hint. Used to specify mesh hierarchy construction preference.

	<b>Default value:</b> PxMeshCookingHint::eSIM_PERFORMANCE
	*/
	PxMeshCookingHint::Enum			meshCookingHint;

	/**
	\brief Desc initialization to default value.
	*/
    void setToDefault()
    {
	    meshSizePerformanceTradeOff = 0.55f;
		meshCookingHint = PxMeshCookingHint::eSIM_PERFORMANCE;
    }

	/**
	\brief Returns true if the descriptor is valid.
	\return true if the current settings are valid.
	*/
	bool isValid() const
	{
		if(meshSizePerformanceTradeOff < 0.0f || meshSizePerformanceTradeOff > 1.0f)
			return false;
		return true;
	}
};

#if !PX_DOXYGEN
} // namespace physx
#endif


  /** @} */
#endif // PX_BVH_33_MIDPHASE_DESC_H
