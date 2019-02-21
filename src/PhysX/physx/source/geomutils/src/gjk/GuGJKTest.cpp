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


#include "GuGJK.h"
#include "GuGJKRaycast.h"
#include "GuGJKPenetration.h"
#include "GuGJKTest.h"

namespace physx
{
namespace Gu
{

using namespace Ps::aos;

GjkStatus testGjk(GjkConvex& a, GjkConvex& b,  const Vec3VArg initialSearchDir, const FloatVArg contactDist, Vec3V& closestA, Vec3V& closestB, Vec3V& normal, FloatV& dist)
{
	return gjk<GjkConvex, GjkConvex>(a, b, initialSearchDir, contactDist, closestA, closestB, normal, dist);
}
	

bool testGjkRaycast(GjkConvex& a, GjkConvex& b, const Vec3VArg initialSearchDir, const Ps::aos::FloatVArg initialLambda, const Ps::aos::Vec3VArg s, const Ps::aos::Vec3VArg r, Ps::aos::FloatV& lambda, 
		Ps::aos::Vec3V& normal, Ps::aos::Vec3V& closestA, const PxReal inflation)
{
	return gjkRaycast(a, b, initialSearchDir, initialLambda, s, r, lambda, normal, closestA, inflation);
}

GjkStatus testGjkPenetration(GjkConvex& a, GjkConvex& b, const Vec3VArg initialSearchDir, const FloatVArg contactDist,
	PxU8* aIndices, PxU8* bIndices, PxU8& size, GjkOutput& output)
{
	return gjkPenetration<GjkConvex, GjkConvex>(a, b, initialSearchDir, contactDist, true,
		aIndices, bIndices, size, output);
}

GjkStatus testEpaPenetration(GjkConvex& a, GjkConvex& b, const PxU8* aIndices, const PxU8* bIndices, const PxU8 size,
	GjkOutput& output)
{
	return epaPenetration(a, b, aIndices, bIndices, size, true, Ps::aos::FLoad(1.f), output);
}

}
}

