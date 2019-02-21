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


#include "GuBarycentricCoordinates.h"

using namespace physx;
using namespace Ps::aos;

void Gu::barycentricCoordinates(const Vec3VArg p, const Vec3VArg a, const Vec3VArg b, FloatV& v)
{
	const Vec3V v0 = V3Sub(a, p);
	const Vec3V v1 = V3Sub(b, p);
	const Vec3V d = V3Sub(v1, v0);
	const FloatV denominator = V3Dot(d, d);
	const FloatV numerator = V3Dot(V3Neg(v0), d);
	const FloatV zero = FZero();
	const FloatV denom = FSel(FIsGrtr(denominator, zero), FRecip(denominator), zero);
	v = FMul(numerator, denom);
}

void Gu::barycentricCoordinates(const Ps::aos::Vec3VArg p, const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b, const Ps::aos::Vec3VArg c, Ps::aos::FloatV& v, Ps::aos::FloatV& w)
{
	const Vec3V ab = V3Sub(b, a);
	const Vec3V ac = V3Sub(c, a);

	const Vec3V n = V3Cross(ab, ac);

	const VecCrossV crossA = V3PrepareCross(V3Sub(a, p));
	const VecCrossV crossB = V3PrepareCross(V3Sub(b, p));
	const VecCrossV crossC = V3PrepareCross(V3Sub(c, p));
	const Vec3V bCrossC = V3Cross(crossB, crossC);
	const Vec3V cCrossA = V3Cross(crossC, crossA);
	const Vec3V aCrossB = V3Cross(crossA, crossB);

	const FloatV va = V3Dot(n, bCrossC);//edge region of BC, signed area rbc, u = S(rbc)/S(abc) for a
	const FloatV vb = V3Dot(n, cCrossA);//edge region of AC, signed area rac, v = S(rca)/S(abc) for b
	const FloatV vc = V3Dot(n, aCrossB);//edge region of AB, signed area rab, w = S(rab)/S(abc) for c
	const FloatV totalArea =FAdd(va, FAdd(vb, vc));
	const FloatV zero = FZero();
	const FloatV denom = FSel(FIsEq(totalArea, zero), zero, FRecip(totalArea));
	v = FMul(vb, denom);
	w = FMul(vc, denom);
	
}

/*
	v0 = b - a;
	v1 = c - a;
	v2 = p - a;
*/
void Gu::barycentricCoordinates(const Vec3VArg v0, const Vec3VArg v1, const Vec3VArg v2, FloatV& v, FloatV& w)
{
	const FloatV d00 = V3Dot(v0, v0);
	const FloatV d01 = V3Dot(v0, v1);
	const FloatV d11 = V3Dot(v1, v1);
	const FloatV d20 = V3Dot(v2, v0);
	const FloatV d21 = V3Dot(v2, v1);
	const FloatV denom = FRecip(FSub(FMul(d00,d11), FMul(d01, d01)));
	v = FMul(FSub(FMul(d11, d20), FMul(d01, d21)), denom);
	w = FMul(FSub(FMul(d00, d21), FMul(d01, d20)), denom);
}

