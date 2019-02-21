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

#ifndef GU_BARYCENTRIC_COORDINATES_H
#define GU_BARYCENTRIC_COORDINATES_H

#include "PxPhysXCommonConfig.h"
#include "CmPhysXCommon.h"
#include "PsVecMath.h"

namespace physx
{
namespace Gu
{
	//calculate the barycentric coorinates for a point in a segment
	void barycentricCoordinates(const Ps::aos::Vec3VArg p, 
		const Ps::aos::Vec3VArg a, 
		const Ps::aos::Vec3VArg b, 
		Ps::aos::FloatV& v);

	//calculate the barycentric coorinates for a point in a triangle
	void barycentricCoordinates(const Ps::aos::Vec3VArg p, 
		const Ps::aos::Vec3VArg a, 
		const Ps::aos::Vec3VArg b, 
		const Ps::aos::Vec3VArg c, 
		Ps::aos::FloatV& v, 
		Ps::aos::FloatV& w);

	void barycentricCoordinates(const Ps::aos::Vec3VArg v0, 
		const Ps::aos::Vec3VArg v1, 
		const Ps::aos::Vec3VArg v2,  
		Ps::aos::FloatV& v, 
		Ps::aos::FloatV& w);

	PX_INLINE Ps::aos::BoolV isValidTriangleBarycentricCoord(const Ps::aos::FloatVArg v, const Ps::aos::FloatVArg w)
	{
		using namespace Ps::aos;
		const FloatV zero = FNeg(FEps());
		const FloatV one = FAdd(FOne(), FEps());

		const BoolV con0 = BAnd(FIsGrtrOrEq(v, zero), FIsGrtrOrEq(one, v));
		const BoolV con1 = BAnd(FIsGrtrOrEq(w, zero), FIsGrtrOrEq(one, w));
		const BoolV con2 = FIsGrtr(one, FAdd(v, w));
		return BAnd(con0, BAnd(con1, con2));
	}

	PX_INLINE Ps::aos::BoolV isValidTriangleBarycentricCoord2(const Ps::aos::Vec4VArg vwvw)
	{
		using namespace Ps::aos;
		const Vec4V eps = V4Splat(FEps());
		const Vec4V zero =V4Neg(eps);
		const Vec4V one = V4Add(V4One(), eps);

		const Vec4V v0v1v0v1 = V4PermXZXZ(vwvw);
		const Vec4V w0w1w0w1 = V4PermYWYW(vwvw);

		const BoolV con0 = BAnd(V4IsGrtrOrEq(v0v1v0v1, zero), V4IsGrtrOrEq(one, v0v1v0v1));
		const BoolV con1 = BAnd(V4IsGrtrOrEq(w0w1w0w1, zero), V4IsGrtrOrEq(one, w0w1w0w1));
		const BoolV con2 = V4IsGrtr(one, V4Add(v0v1v0v1, w0w1w0w1));
		return BAnd(con0, BAnd(con1, con2));
	}

} // namespace Gu

}

#endif
