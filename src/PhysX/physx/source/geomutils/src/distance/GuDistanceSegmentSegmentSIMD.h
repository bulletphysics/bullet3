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

#ifndef GU_DISTANCE_SEGMENT_SEGMENT_SIMD_H
#define GU_DISTANCE_SEGMENT_SEGMENT_SIMD_H

#include "common/PxPhysXCommonConfig.h"
#include "PsVecMath.h"

namespace physx
{
namespace Gu
{
	PX_PHYSX_COMMON_API Ps::aos::FloatV distanceSegmentSegmentSquared(	const Ps::aos::Vec3VArg p1, const Ps::aos::Vec3VArg d1, const Ps::aos::Vec3VArg p2, const Ps::aos::Vec3VArg d2,
					 								Ps::aos::FloatV& param0, 
					 								Ps::aos::FloatV& param1);

	/*
		This function do four segment segment closest point test in one go
	*/
	Ps::aos::Vec4V distanceSegmentSegmentSquared4(  const Ps::aos::Vec3VArg p, const Ps::aos::Vec3VArg d, 
													const Ps::aos::Vec3VArg p02, const Ps::aos::Vec3VArg d02, 
													const Ps::aos::Vec3VArg p12, const Ps::aos::Vec3VArg d12, 
													const Ps::aos::Vec3VArg p22, const Ps::aos::Vec3VArg d22,
													const Ps::aos::Vec3VArg p32, const Ps::aos::Vec3VArg d32,
													Ps::aos::Vec4V& s, Ps::aos::Vec4V& t);
} // namespace Gu

}

#endif
