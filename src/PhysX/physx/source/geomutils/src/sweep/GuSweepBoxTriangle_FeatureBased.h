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

#ifndef GU_SWEEP_BOX_TRIANGLE_FEATURE_BASED_H
#define GU_SWEEP_BOX_TRIANGLE_FEATURE_BASED_H

#include "foundation/PxVec3.h"
#include "foundation/PxPlane.h"
#include "CmPhysXCommon.h"

namespace physx
{
	class PxTriangle;
	
	namespace Gu
	{
		/**
		Sweeps a box against a triangle, using a 'feature-based' approach.

		This is currently only used for computing the box-sweep impact data, in a second pass,
		after the best triangle has been identified using faster approaches (SAT/GJK).

		\warning Returned impact normal is not normalized

		\param tri				[in] the triangle
		\param box				[in] the box
		\param motion			[in] (box) motion vector
		\param oneOverMotion	[in] precomputed inverse of motion vector
		\param hit				[out] impact point
		\param normal			[out] impact normal (warning: not normalized)
		\param d				[in/out] impact distance (please initialize with best current distance)
		\param isDoubleSided	[in] whether triangle is double-sided or not
		\return	true if an impact has been found
		*/
		bool sweepBoxTriangle(	const PxTriangle& tri, const PxBounds3& box,
								const PxVec3& motion, const PxVec3& oneOverMotion,
								PxVec3& hit, PxVec3& normal, PxReal& d, bool isDoubleSided=false);
	} // namespace Gu
}

#endif
